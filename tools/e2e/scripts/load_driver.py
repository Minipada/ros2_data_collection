#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Synthetic load driver (#378): a sender for #323's limits harness that speaks the
shipper ingest protocol — the same msgpack-over-TCP Forward Mode frames, including the
chunk/acknowledgement option, that dc_bridge's Forwarder emits (see
dc_bridge/src/forwarder.cpp and dc_bridge/include/dc_bridge/forwarder.hpp) — closely
enough that the Shipper's `fluent` ingest listener cannot distinguish it from a real
Bridge at the socket.

Given a target, a connection count, a per-connection Record rate, and a duration, run()
opens that many concurrent TCP connections, emits frames at that rate on each, records
every frame it sent to a ledger, and reports per-frame acknowledgement latency. Rate is
per-connection (not divided across connections) so connection count (the fan-in axis)
and per-connection rate (the single-robot-ceiling axis) can be varied independently, per
#323's PRD user stories.

Has no ROS dependency and is exercised standalone here (test_load_driver.py, against an
in-process fake ingest listener) as well as against a real Vector container
(run_load_driver_shipper_test.sh) — the property that can only be established against
the genuine article, not a mock.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import random
import statistics
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass, field

import msgpack

# The fluent forward protocol's EventTime extension type (see forwarder.cpp's
# pack_event_time): a fixext8 whose 8-byte body is 4 bytes of big-endian seconds then 4
# of big-endian nanoseconds within that second.
EVENT_TIME_EXT_TYPE = 0x00

DEFAULT_TAG = "dc.e2e.synthetic"


def _pack_event_time(seconds: int, nanos: int) -> msgpack.ExtType:
    body = (seconds & 0xFFFFFFFF).to_bytes(4, "big") + (nanos & 0xFFFFFFFF).to_bytes(4, "big")
    return msgpack.ExtType(EVENT_TIME_EXT_TYPE, body)


def generate_chunk_id() -> str:
    """An opaque, unique-enough id for one in-flight frame, echoed back verbatim in the
    peer's ack (unpadded base64 of 16 random bytes — the protocol only requires
    uniqueness, not a specific encoding, and this mirrors the shape of
    Forwarder::generate_chunk_id() without needing to match its exact bytes)."""
    return base64.b64encode(random.randbytes(16)).decode("ascii").rstrip("=")


def frame(tag: str, seconds: int, nanos: int, payload: dict, chunk_id: str) -> bytes:
    """The shipper ingest protocol "Forward Mode" frame `[tag, [[time, record]],
    {"chunk": chunk_id}]`, packed as msgpack bytes. Structurally identical to what
    dc_bridge::Forwarder::frame() emits for the same tag/time/payload/chunk_id — see
    test_load_driver.py's wire-format tests, built against forwarder_test.cpp's own
    pinned EventTime bytes."""
    entry = [_pack_event_time(seconds, nanos), payload]
    return msgpack.packb([tag, [entry], {"chunk": chunk_id}], use_bin_type=True)


def default_payload(connection_id: int, seq: int) -> dict:
    """The record body a caller gets if it doesn't supply its own: just enough for a
    receiver-side observer (e.g. the integration test's Vector file sink) to identify
    which ledger entry a received Record corresponds to."""
    return {"connection_id": connection_id, "seq": seq}


@dataclass(frozen=True)
class LedgerEntry:
    """One frame this driver sent. `sent_at` is a `time.monotonic()` reading, so it's
    only meaningful relative to other readings from the same run, never as a wall clock."""

    connection_id: int
    seq: int
    tag: str
    chunk_id: str
    frame_bytes: int
    sent_at: float
    payload: dict


@dataclass(frozen=True)
class AckObservation:
    """The acknowledgement outcome for one ledger entry. `latency_s` is None when no ack
    arrived before the run's drain deadline — never coerced to 0 or omitted, since "never
    acked" and "acked instantly" are different findings for the saturation probe."""

    connection_id: int
    seq: int
    chunk_id: str
    latency_s: float | None


@dataclass(frozen=True)
class LoadDriverConfig:
    host: str
    port: int
    connection_count: int
    record_rate_hz: float
    duration_s: float
    tag: str = DEFAULT_TAG
    connect_timeout_s: float = 5.0
    # How long, after duration_s elapses, to keep waiting for acks still outstanding
    # before giving up on them (as opposed to giving up the instant duration_s elapses,
    # which would misreport a merely-slow-to-ack peer as never-acked).
    drain_timeout_s: float = 5.0
    payload_fn: Callable[[int, int], dict] = default_payload


@dataclass
class LoadDriverResult:
    ledger: list[LedgerEntry] = field(default_factory=list)
    observations: list[AckObservation] = field(default_factory=list)

    @property
    def sent_count(self) -> int:
        return len(self.ledger)

    @property
    def acked_count(self) -> int:
        return sum(1 for o in self.observations if o.latency_s is not None)

    @property
    def unacked_chunk_ids(self) -> list[str]:
        return [o.chunk_id for o in self.observations if o.latency_s is None]

    def ack_latencies_s(self) -> list[float]:
        return [o.latency_s for o in self.observations if o.latency_s is not None]

    def summary(self) -> dict:
        latencies = self.ack_latencies_s()
        return {
            "sent": self.sent_count,
            "acked": self.acked_count,
            "unacked": self.sent_count - self.acked_count,
            "ack_latency_mean_s": statistics.fmean(latencies) if latencies else None,
            "ack_latency_max_s": max(latencies) if latencies else None,
        }


async def _run_connection(
    connection_id: int, config: LoadDriverConfig, result: LoadDriverResult
) -> None:
    reader, writer = await asyncio.wait_for(
        asyncio.open_connection(config.host, config.port), timeout=config.connect_timeout_s
    )

    pending: dict[str, tuple[int, float]] = {}  # chunk_id -> (seq, sent_at)
    unpacker = msgpack.Unpacker(raw=False)

    async def drain_acks() -> None:
        try:
            while True:
                data = await reader.read(65536)
                if not data:
                    return  # peer closed
                unpacker.feed(data)
                for obj in unpacker:
                    if not isinstance(obj, dict) or "ack" not in obj:
                        continue
                    chunk_id = obj["ack"]
                    entry = pending.pop(chunk_id, None)
                    if entry is None:
                        continue
                    seq, sent_at = entry
                    result.observations.append(
                        AckObservation(
                            connection_id=connection_id,
                            seq=seq,
                            chunk_id=chunk_id,
                            latency_s=time.monotonic() - sent_at,
                        )
                    )
        except (ConnectionError, asyncio.IncompleteReadError):
            return

    reader_task = asyncio.ensure_future(drain_acks())
    try:
        interval = 1.0 / config.record_rate_hz if config.record_rate_hz > 0 else 0.0
        start = time.monotonic()
        seq = 0
        next_tick = start
        while (time.monotonic() - start) < config.duration_s:
            now = time.time()
            seconds = int(now)
            nanos = int((now - seconds) * 1_000_000_000)
            payload = config.payload_fn(connection_id, seq)
            chunk_id = generate_chunk_id()
            frame_bytes = frame(config.tag, seconds, nanos, payload, chunk_id)

            sent_at = time.monotonic()
            writer.write(frame_bytes)
            await writer.drain()

            result.ledger.append(
                LedgerEntry(
                    connection_id=connection_id,
                    seq=seq,
                    tag=config.tag,
                    chunk_id=chunk_id,
                    frame_bytes=len(frame_bytes),
                    sent_at=sent_at,
                    payload=payload,
                )
            )
            pending[chunk_id] = (seq, sent_at)
            seq += 1

            next_tick += interval
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                await asyncio.sleep(sleep_for)

        # Drain grace period: keep the reader running so late acks still resolve.
        drain_deadline = time.monotonic() + config.drain_timeout_s
        while pending and time.monotonic() < drain_deadline:
            await asyncio.sleep(0.02)
    finally:
        reader_task.cancel()
        writer.close()
        try:
            await writer.wait_closed()
        except (ConnectionError, asyncio.CancelledError):
            pass

    for chunk_id, (seq, _sent_at) in pending.items():
        result.observations.append(
            AckObservation(connection_id=connection_id, seq=seq, chunk_id=chunk_id, latency_s=None)
        )


async def run(config: LoadDriverConfig) -> LoadDriverResult:
    """Runs `config.connection_count` connections concurrently for `config.duration_s`
    seconds and returns the combined sent-ledger plus per-frame acknowledgement-latency
    observations (#378's acceptance criterion). Every ledger entry has exactly one
    corresponding observation, acked or not."""
    result = LoadDriverResult()
    await asyncio.gather(
        *(
            _run_connection(connection_id, config, result)
            for connection_id in range(config.connection_count)
        )
    )
    return result


def run_sync(config: LoadDriverConfig) -> LoadDriverResult:
    """Synchronous wrapper for CLI/non-async callers."""
    return asyncio.run(run(config))


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host")
    parser.add_argument("port", type=int)
    parser.add_argument("--connections", type=int, default=10)
    parser.add_argument("--rate", type=float, default=1.0, help="per-connection Records/s")
    parser.add_argument("--duration", type=float, default=10.0, help="seconds")
    parser.add_argument("--tag", default=DEFAULT_TAG)
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--drain-timeout", type=float, default=5.0)
    parser.add_argument(
        "--ledger-out", help="write the full sent-ledger as JSON lines to this path"
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    config = LoadDriverConfig(
        host=args.host,
        port=args.port,
        connection_count=args.connections,
        record_rate_hz=args.rate,
        duration_s=args.duration,
        tag=args.tag,
        connect_timeout_s=args.connect_timeout,
        drain_timeout_s=args.drain_timeout,
    )
    result = run_sync(config)

    if args.ledger_out:
        with open(args.ledger_out, "w") as f:
            for entry in result.ledger:
                f.write(
                    json.dumps(
                        {
                            "connection_id": entry.connection_id,
                            "seq": entry.seq,
                            "tag": entry.tag,
                            "chunk_id": entry.chunk_id,
                            "frame_bytes": entry.frame_bytes,
                            "sent_at": entry.sent_at,
                            "payload": entry.payload,
                        }
                    )
                    + "\n"
                )

    print(json.dumps(result.summary()))
    return 0 if result.acked_count == result.sent_count else 1


if __name__ == "__main__":
    sys.exit(main())
