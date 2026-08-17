# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Smoke tests for the `dc_mcap_writer` CLI/process (#210).

Unlike `test_writer.py` (which exercises `RotatingMcapWriter` as a plain Python object),
these tests run the real console entrypoint as a subprocess and talk to it the way its
actual callers do: a TCP client sending newline-delimited JSON, exactly what Vector's
`socket` sink does (`dc_demos/config/mcap_sink.toml`), and `SIGTERM`, exactly what
`podman stop`/systemd send on shutdown — neither is exercised by the unit tests.
"""

import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest
from mcap.reader import make_reader

PACKAGE_ROOT = Path(__file__).resolve().parent.parent


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def _spawn(*extra_args: str) -> subprocess.Popen:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([str(PACKAGE_ROOT), env.get("PYTHONPATH", "")])
    return subprocess.Popen(
        [sys.executable, "-m", "dc_mcap_writer.cli", *extra_args],
        env=env,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def _read_all_messages(output_dir: Path):
    messages = []
    for mcap_file in sorted(output_dir.glob("*.mcap")):
        with open(mcap_file, "rb") as f:
            messages.extend(make_reader(f).iter_messages())
    return messages


def _record(tag, seq, date=1786118523000000000):
    return {"tag": tag, "name": tag.rsplit(".", 1)[-1], "date": date + seq, "seq": seq}


def test_stdin_mode_end_to_end(tmp_path):
    """The `--stdin` path: real process, real argv, real stdin pipe, real exit code."""
    records = [_record("dc.measurement.cpu", i) for i in range(3)] + [
        _record("dc.measurement.memory", i) for i in range(2)
    ]
    payload = "".join(json.dumps(r) + "\n" for r in records)

    proc = _spawn("--output-dir", str(tmp_path), "--stdin")
    stdout, _ = proc.communicate(input=payload, timeout=30)

    assert proc.returncode == 0, stdout
    messages = _read_all_messages(tmp_path)
    assert len(messages) == 5
    topics = {channel.topic for _, channel, _ in messages}
    assert topics == {"dc.measurement.cpu", "dc.measurement.memory"}


def test_two_process_restarts_in_the_same_second_do_not_collide(tmp_path):
    """Regression test for a real bug the e2e zero-loss harness caught (#210).

    Two independent `dc_mcap_writer` processes writing to the same `--output-dir` —
    exactly what a container restart is (the zero-loss harness's `podman restart`) —
    used to be able to compute the identical filename if both opened their first file
    in the same wall-clock second, since the rotation-index-based uniqueness only held
    *within* one process's lifetime. The second process's `open(path, "wb")` then
    silently truncated the first process's already-finished file. Back-to-back
    `--stdin` runs are the fastest reliable way to force two starts into the same
    second.

    Args:
        tmp_path: pytest's per-test temporary directory fixture.
    """
    first = _spawn("--output-dir", str(tmp_path), "--stdin")
    first_stdout, _ = first.communicate(input=json.dumps(_record("dc.measurement.a", 0)) + "\n")
    assert first.returncode == 0, first_stdout

    second = _spawn("--output-dir", str(tmp_path), "--stdin")
    second_stdout, _ = second.communicate(input=json.dumps(_record("dc.measurement.b", 0)) + "\n")
    assert second.returncode == 0, second_stdout

    files = sorted(tmp_path.glob("*.mcap"))
    assert len(files) == 2, f"expected one file per process, got {[f.name for f in files]}"
    messages = _read_all_messages(tmp_path)
    topics = {channel.topic for _, channel, _ in messages}
    assert topics == {"dc.measurement.a", "dc.measurement.b"}, (
        f"one process's file was truncated by the other's — {first_stdout}\n{second_stdout}"
    )


def test_tcp_socket_mode_survives_sigterm_with_a_valid_mcap_file(tmp_path):
    """Simulates the real deployment shape end to end.

    A Vector `socket` sink connects over TCP and streams NDJSON (see
    `dc_demos/config/mcap_sink.toml`); the process is later stopped with SIGTERM, the
    signal a container/systemd sends, not Ctrl-C's SIGINT. Both the TCP server path and
    the SIGTERM handler are otherwise untested by `test_writer.py`, which only calls
    `RotatingMcapWriter` directly.

    Args:
        tmp_path: pytest's per-test temporary directory fixture.
    """
    port = _free_port()
    proc = _spawn(
        "--output-dir",
        str(tmp_path),
        "--host",
        "127.0.0.1",
        "--port",
        str(port),
    )
    try:
        # The server takes a moment to bind; retry the connection rather than sleeping a
        # fixed amount, since a fixed sleep is either flaky under load or wastefully long.
        deadline = time.monotonic() + 10.0
        client = None
        last_error = None
        while time.monotonic() < deadline:
            try:
                client = socket.create_connection(("127.0.0.1", port), timeout=1.0)
                break
            except OSError as exc:
                last_error = exc
                time.sleep(0.1)
        assert client is not None, f"dc_mcap_writer never opened its listening port: {last_error}"

        records = [_record("dc.measurement.uptime", i) for i in range(4)]
        with client:
            for record in records:
                client.sendall((json.dumps(record) + "\n").encode("utf-8"))
            client.shutdown(socket.SHUT_WR)
            # Confirms the server actually read and processed everything before it is
            # torn down below — the handler's own connection-closed log line, read off
            # the (redirected) child stdout, is that confirmation.
            client.recv(1)

        deadline = time.monotonic() + 10.0
        closed_logged = False
        while time.monotonic() < deadline:
            line = proc.stdout.readline()
            if not line:
                break
            if "connection from" in line and "closed" in line:
                closed_logged = True
                break
        assert closed_logged, "server never logged that the client connection closed"

        proc.send_signal(signal.SIGTERM)
        stdout, _ = proc.communicate(timeout=10)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.communicate(timeout=10)

    assert proc.returncode == 0, stdout

    messages = _read_all_messages(tmp_path)
    assert len(messages) == 4
    schema, channel, _ = messages[0]
    assert channel.topic == "dc.measurement.uptime"
    assert schema.encoding == "jsonschema"
    payloads = sorted(json.loads(message.data)["seq"] for _, _, message in messages)
    assert payloads == [0, 1, 2, 3]


@pytest.mark.parametrize("signum", [signal.SIGTERM, signal.SIGINT])
def test_server_mode_exits_cleanly_with_no_records(tmp_path, signum):
    """Even an idle listener must finish a readable (empty) .mcap file on shutdown."""
    port = _free_port()
    proc = _spawn("--output-dir", str(tmp_path), "--host", "127.0.0.1", "--port", str(port))
    try:
        deadline = time.monotonic() + 10.0
        connected = False
        while time.monotonic() < deadline:
            try:
                with socket.create_connection(("127.0.0.1", port), timeout=1.0):
                    connected = True
                    break
            except OSError:
                time.sleep(0.1)
        assert connected, "dc_mcap_writer never opened its listening port"

        proc.send_signal(signum)
        stdout, _ = proc.communicate(timeout=10)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.communicate(timeout=10)

    assert proc.returncode == 0, stdout
    files = list(tmp_path.glob("*.mcap"))
    assert len(files) == 1
    assert _read_all_messages(tmp_path) == []
