# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Unit tests for tools/e2e/scripts/measure_rtt.py (#366), against a real loopback
listening socket — no container, no network shaping needed to pin the sampler's own
arithmetic and error handling.
"""

import os
import socket
import sys
import threading

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts"))

import measure_rtt


@pytest.fixture
def accepting_server():
    """A listening socket on 127.0.0.1 that accepts and immediately drops every
    connection, yielding (host, port)."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("127.0.0.1", 0))
    server.listen(8)
    host, port = server.getsockname()
    stop = threading.Event()

    def accept_loop():
        server.settimeout(0.2)
        while not stop.is_set():
            try:
                conn, _ = server.accept()
                conn.close()
            except TimeoutError:
                continue

    thread = threading.Thread(target=accept_loop, daemon=True)
    thread.start()
    try:
        yield host, port
    finally:
        stop.set()
        thread.join()
        server.close()


def test_sample_returns_one_measurement_per_count(accepting_server):
    host, port = accepting_server
    samples = measure_rtt.sample(host, port, count=5, timeout=2.0)
    assert len(samples) == 5
    assert all(s >= 0 for s in samples)


def test_sample_raises_on_a_port_nothing_is_listening_on():
    closed_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    closed_server.bind(("127.0.0.1", 0))
    _, port = closed_server.getsockname()
    closed_server.close()  # bound then closed: nothing listens on this port now

    with pytest.raises(OSError):
        measure_rtt.sample("127.0.0.1", port, count=1, timeout=1.0)


def test_main_prints_min_avg_max_as_json(accepting_server, capsys):
    host, port = accepting_server
    # main() reads argv directly, so drive it through sys.argv instead of calling with args.
    old_argv = sys.argv
    try:
        sys.argv = ["measure_rtt.py", host, str(port), "--count", "3"]
        exit_code = measure_rtt.main()
    finally:
        sys.argv = old_argv

    assert exit_code == 0
    import json

    out = json.loads(capsys.readouterr().out)
    assert out["host"] == host
    assert out["port"] == port
    assert len(out["samples_ms"]) == 3
    assert out["min_ms"] <= out["avg_ms"] <= out["max_ms"]


def test_main_fails_loudly_with_nothing_listening(capsys):
    closed_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    closed_server.bind(("127.0.0.1", 0))
    _, port = closed_server.getsockname()
    closed_server.close()

    old_argv = sys.argv
    try:
        sys.argv = ["measure_rtt.py", "127.0.0.1", str(port), "--count", "1", "--timeout", "0.5"]
        exit_code = measure_rtt.main()
    finally:
        sys.argv = old_argv

    assert exit_code == 1
    assert "error" in capsys.readouterr().err
