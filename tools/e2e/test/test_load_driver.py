# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Unit tests for tools/e2e/scripts/load_driver.py (#378).

Two layers: pure wire-format tests that pin the frame() bytes against the same
EventTime encoding dc_bridge/test/forwarder_test.cpp pins for the real Forwarder (no
network at all), and behavioural tests against an in-process fake fluent-protocol
listener on loopback (no container, no ROS — this is exactly what #378's acceptance
criterion means by "runs and is tested standalone"). Byte-compatibility against the
*real* Shipper build is a separate, container-based integration test
(run_load_driver_shipper_test.sh) — that property can't be established against a fake.
"""

import asyncio
import os
import sys

import msgpack

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts"))

from load_driver import (
    LoadDriverConfig,
    frame,
    generate_chunk_id,
    run,
)


def _expected_event_time_bytes(secs: int, nanos: int) -> bytes:
    """fixext8, ext type 0x00, big-endian secs then big-endian nanos — matches
    forwarder_test.cpp's expected_event_time_bytes() byte-for-byte."""
    return bytes([0xD7, 0x00]) + secs.to_bytes(4, "big") + nanos.to_bytes(4, "big")


# --- Wire format ----------------------------------------------------------------------


def test_frame_is_well_formed_with_correct_tag():
    raw = frame("dc.measurement.uptime", 1700000000, 0, {"uptime_s": 42}, "chunk-a")
    top = msgpack.unpackb(raw, raw=False)
    assert len(top) == 3  # [tag, entries, option]
    tag, entries, option = top
    assert tag == "dc.measurement.uptime"
    assert len(entries) == 1
    _time, record = entries[0]
    assert record == {"uptime_s": 42}
    assert option == {"chunk": "chunk-a"}


def test_frame_carries_sub_second_precision_as_event_time():
    nanos = 123456789
    raw = frame("dc.test", 1700000000, nanos, {"n": 1}, "chunk-ns")
    assert _expected_event_time_bytes(1700000000, nanos) in raw


def test_frames_within_the_same_second_are_distinguishable():
    a = frame("dc.test", 1700000000, 1, {"n": 1}, "c1")
    b = frame("dc.test", 1700000000, 2, {"n": 1}, "c1")
    assert a != b
    assert _expected_event_time_bytes(1700000000, 1) in a
    assert _expected_event_time_bytes(1700000000, 2) in b


def test_whole_second_timestamp_still_uses_event_time():
    raw = frame("dc.test", 1700000000, 0, {"n": 1}, "c0")
    assert _expected_event_time_bytes(1700000000, 0) in raw


def test_frame_carries_the_chunk_option():
    raw = frame("dc.test", 1700000000, 0, {"n": 1}, "my-chunk-id")
    top = msgpack.unpackb(raw, raw=False)
    assert top[2] == {"chunk": "my-chunk-id"}


def test_different_records_carry_their_own_tag():
    a = msgpack.unpackb(frame("dc.measurement.cpu", 1700000000, 0, {"pct": 1}, "c1"), raw=False)
    b = msgpack.unpackb(
        frame("dc.measurement.uptime", 1700000000, 0, {"uptime_s": 2}, "c2"), raw=False
    )
    assert a[0] == "dc.measurement.cpu"
    assert b[0] == "dc.measurement.uptime"


def test_generate_chunk_id_is_unique_and_url_safe_ish():
    ids = {generate_chunk_id() for _ in range(1000)}
    assert len(ids) == 1000
    for chunk_id in list(ids)[:5]:
        assert chunk_id.isascii()
        assert "\n" not in chunk_id


# --- Behavioural, against an in-process fake fluent listener --------------------------


class _FakeIngestServer:
    """Accepts connections, decodes shipper-ingest-protocol frames with a real
    msgpack.Unpacker (so a frame split across TCP reads is still handled correctly,
    mirroring the Forwarder's own drain_acks()), and acks each chunk. `never_ack`
    disables the ack half entirely, for the unacked/timeout tests."""

    def __init__(self, never_ack: bool = False, ack_delay_s: float = 0.0):
        self.never_ack = never_ack
        self.ack_delay_s = ack_delay_s
        self.received_chunk_ids: list[str] = []
        self._server: asyncio.AbstractServer | None = None
        self.port: int = 0

    async def start(self) -> None:
        self._server = await asyncio.start_server(self._handle, "127.0.0.1", 0)
        self.port = self._server.sockets[0].getsockname()[1]

    async def stop(self) -> None:
        # Not `await self._server.wait_closed()`: since Python 3.12 that also waits for
        # every already-completed connection handler task to be reaped, which has been
        # observed here to hang well past the handler actually returning — a close()
        # without waiting is sufficient for a test's cleanup (the event loop tears down
        # the listening socket at process exit regardless).
        self._server.close()

    async def _handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        unpacker = msgpack.Unpacker(raw=False)
        try:
            while True:
                data = await reader.read(65536)
                if not data:
                    return
                unpacker.feed(data)
                for tag, entries, option in unpacker:
                    del tag, entries
                    chunk_id = option["chunk"]
                    self.received_chunk_ids.append(chunk_id)
                    if self.never_ack:
                        continue
                    if self.ack_delay_s:
                        await asyncio.sleep(self.ack_delay_s)
                    writer.write(msgpack.packb({"ack": chunk_id}, use_bin_type=True))
                    await writer.drain()
        except (ConnectionError, asyncio.IncompleteReadError):
            return


def test_driver_reports_a_ledger_entry_and_ack_for_every_frame_sent():
    asyncio.run(_driver_reports_a_ledger_entry_and_ack_for_every_frame_sent())


async def _driver_reports_a_ledger_entry_and_ack_for_every_frame_sent():
    server = _FakeIngestServer()
    await server.start()
    try:
        config = LoadDriverConfig(
            host="127.0.0.1",
            port=server.port,
            connection_count=3,
            record_rate_hz=20.0,
            duration_s=0.5,
            drain_timeout_s=2.0,
        )
        result = await run(config)
    finally:
        await server.stop()

    assert result.sent_count > 0
    assert len(result.observations) == result.sent_count
    assert result.acked_count == result.sent_count
    assert result.unacked_chunk_ids == []
    assert all(latency >= 0 for latency in result.ack_latencies_s())

    ledger_chunk_ids = {entry.chunk_id for entry in result.ledger}
    assert ledger_chunk_ids == set(server.received_chunk_ids)


def test_driver_spreads_frames_across_every_connection():
    asyncio.run(_driver_spreads_frames_across_every_connection())


async def _driver_spreads_frames_across_every_connection():
    server = _FakeIngestServer()
    await server.start()
    try:
        config = LoadDriverConfig(
            host="127.0.0.1",
            port=server.port,
            connection_count=4,
            record_rate_hz=10.0,
            duration_s=0.4,
        )
        result = await run(config)
    finally:
        await server.stop()

    connection_ids = {entry.connection_id for entry in result.ledger}
    assert connection_ids == {0, 1, 2, 3}


def test_never_acked_frames_are_reported_as_unacked_not_dropped():
    asyncio.run(_never_acked_frames_are_reported_as_unacked_not_dropped())


async def _never_acked_frames_are_reported_as_unacked_not_dropped():
    server = _FakeIngestServer(never_ack=True)
    await server.start()
    try:
        config = LoadDriverConfig(
            host="127.0.0.1",
            port=server.port,
            connection_count=1,
            record_rate_hz=10.0,
            duration_s=0.2,
            drain_timeout_s=0.3,
        )
        result = await run(config)
    finally:
        await server.stop()

    assert result.sent_count > 0
    assert result.acked_count == 0
    assert len(result.unacked_chunk_ids) == result.sent_count
    assert all(o.latency_s is None for o in result.observations)


def test_a_slow_ack_within_the_drain_window_still_resolves():
    asyncio.run(_a_slow_ack_within_the_drain_window_still_resolves())


async def _a_slow_ack_within_the_drain_window_still_resolves():
    server = _FakeIngestServer(ack_delay_s=0.3)
    await server.start()
    try:
        config = LoadDriverConfig(
            host="127.0.0.1",
            port=server.port,
            connection_count=1,
            record_rate_hz=5.0,
            duration_s=0.2,
            drain_timeout_s=2.0,
        )
        result = await run(config)
    finally:
        await server.stop()

    assert result.sent_count > 0
    assert result.acked_count == result.sent_count
    assert min(result.ack_latencies_s()) >= 0.3


def test_custom_payload_fn_is_carried_onto_the_wire():
    asyncio.run(_custom_payload_fn_is_carried_onto_the_wire())


async def _custom_payload_fn_is_carried_onto_the_wire():
    server = _FakeIngestServer()
    await server.start()
    try:
        config = LoadDriverConfig(
            host="127.0.0.1",
            port=server.port,
            connection_count=1,
            record_rate_hz=10.0,
            duration_s=0.15,
            payload_fn=lambda connection_id, seq: {"marker": "custom", "seq": seq},
        )
        result = await run(config)
    finally:
        await server.stop()

    assert result.sent_count > 0
    assert all(entry.payload["marker"] == "custom" for entry in result.ledger)
