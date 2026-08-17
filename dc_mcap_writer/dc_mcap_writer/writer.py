# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Rotating MCAP writer for Records consumed over the ADR-0003 passthrough (#210)."""

import json
import logging
import os
import threading
import time
from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import BinaryIO, TextIO

from mcap.writer import Writer

logger = logging.getLogger("dc_mcap_writer")

# Records carry arbitrary application-defined fields per Measurement (ADR-0003); a
# permissive schema is the only one that can describe every Tag without per-Tag
# configuration. This is the jsonschema-channel pattern from the foxglove/mcap
# `jsonschema/writer.cpp` example linked in issue #210.
_RECORD_JSON_SCHEMA = json.dumps({"type": "object"}).encode("utf-8")


def extract_log_time_ns(record: dict, time_key: str) -> int:
    """Convert a Record's normalized timestamp field to epoch nanoseconds.

    Handles all three `time_format` values the Bridge can write (see
    doc/src/dc/destinations.md): `epoch_nanos` (int), `double` (float seconds) and
    `iso8601` (string). Falls back to wall-clock time if the field is missing or
    unparsable, so one malformed Record never stops the writer.

    Args:
        record: The decoded Record.
        time_key: The Record field holding the normalized timestamp (`time_key`
            Bridge parameter, default `date`).

    Returns:
        Epoch nanoseconds to use as the MCAP message's log/publish time.
    """
    value = record.get(time_key)
    if isinstance(value, bool):
        return time.time_ns()
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return int(value * 1_000_000_000)
    if isinstance(value, str):
        try:
            parsed = datetime.fromisoformat(value)
        except ValueError:
            return time.time_ns()
        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=UTC)
        return int(parsed.timestamp() * 1_000_000_000)
    return time.time_ns()


def iter_ndjson(stream: TextIO) -> Iterator[dict]:
    """Yield decoded JSON objects from a newline-delimited JSON stream.

    A malformed line is logged and skipped rather than raised, since this feeds a
    long-running process that must not go down over one bad Record.

    Args:
        stream: A text stream yielding one JSON document per line.

    Yields:
        Decoded Records.
    """
    for line in stream:
        line = line.strip()
        if not line:
            continue
        try:
            yield json.loads(line)
        except json.JSONDecodeError:
            logger.warning("dropping malformed NDJSON line: %r", line[:200])


@dataclass
class RotationPolicy:
    """When to roll over to a new `.mcap` file — whichever limit is hit first."""

    max_bytes: int | None = None
    max_duration_secs: float | None = None


class RotatingMcapWriter:
    """Writes Records into a sequence of rotated `.mcap` files.

    One JSON-schema Channel per distinct Tag; each file is finished (index written)
    before the next one opens, so every `.mcap` on disk is independently valid and
    readable — a crash never leaves the *previous* rotation's file truncated, only
    the one currently being written.
    """

    def __init__(
        self,
        output_dir: Path,
        prefix: str = "records",
        rotation: RotationPolicy | None = None,
        time_key: str = "date",
    ):
        """Create a writer and open its first output file.

        Args:
            output_dir: Directory `.mcap` files are written into; created if missing.
            prefix: Filename prefix for each rotation.
            rotation: Size/duration rotation policy; unset fields never trigger.
            time_key: Record field holding the normalized timestamp.
        """
        self._output_dir = output_dir
        self._prefix = prefix
        self._rotation = rotation or RotationPolicy()
        self._time_key = time_key
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._file: BinaryIO | None = None
        self._writer: Writer | None = None
        self._schema_id: int = 0
        self._channel_ids: dict[str, int] = {}
        self._opened_at: float = 0.0
        self._sequence: int = 0
        self._current_path: Path | None = None
        self._rotation_index: int = 0
        self._open_next_file()

    @property
    def current_path(self) -> Path | None:
        """Path of the `.mcap` file currently being written."""
        return self._current_path

    def _open_next_file(self) -> None:
        if self._writer is not None:
            self._writer.finish()
            self._file.close()
            logger.info("closed %s", self._current_path)
        timestamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        # The rotation index alone only guarantees uniqueness *within* one process's
        # lifetime — it resets to 0 on every new process, so two independent
        # dc_mcap_writer processes (a container restart is exactly this, e.g. the
        # zero-loss e2e harness's `podman restart`) that each open their first file in
        # the same wall-clock second would otherwise both compute rotation index 1 and
        # collide on the identical filename, and the second process's `open(..., "wb")`
        # would silently truncate the first process's already-finished file. The PID
        # makes that collision impossible across processes; the rotation index still
        # covers same-second rotations within one process.
        self._rotation_index += 1
        index = str(self._rotation_index).zfill(4)
        pid = os.getpid()
        self._current_path = self._output_dir / f"{self._prefix}_{timestamp}_{pid}_{index}.mcap"
        self._file = open(self._current_path, "wb")
        # Chunking (mcap's default) buffers messages entirely in memory — a Chunk isn't
        # written to `self._file` at all until it reaches `chunk_size` (1 MiB default)
        # or `finish()` runs. At this passthrough's Record rate that chunk can easily
        # outlive the process: a container killed abruptly (SIGKILL, or a restart grace
        # period too short for the graceful SIGTERM path below to complete — a real
        # failure mode, not hypothetical) loses every message since the file opened,
        # not just the last few. Disabling it makes `add_message()` write straight
        # through to `self._file` (verified: `self._file.tell()` grows immediately per
        # message), which also makes the `max_bytes` rotation check above meaningful —
        # with chunking it was only ever reacting to header/schema overhead, since
        # message bytes never touched `self._file` until a chunk closed.
        self._writer = Writer(self._file, use_chunking=False)
        self._writer.start(profile="", library="dc_mcap_writer")
        self._schema_id = self._writer.register_schema(
            name="dc/record", encoding="jsonschema", data=_RECORD_JSON_SCHEMA
        )
        self._channel_ids = {}
        self._opened_at = time.monotonic()
        self._sequence = 0
        logger.info("opened %s", self._current_path)

    def _rotate_if_needed(self) -> None:
        rotation = self._rotation
        if rotation.max_bytes is not None and self._file.tell() >= rotation.max_bytes:
            self._open_next_file()
            return
        if (
            rotation.max_duration_secs is not None
            and (time.monotonic() - self._opened_at) >= rotation.max_duration_secs
        ):
            self._open_next_file()

    def _channel_id(self, tag: str) -> int:
        channel_id = self._channel_ids.get(tag)
        if channel_id is None:
            channel_id = self._writer.register_channel(
                topic=tag, message_encoding="json", schema_id=self._schema_id
            )
            self._channel_ids[tag] = channel_id
        return channel_id

    def write_record(self, record: dict) -> None:
        """Append one Record to the current `.mcap` file, rotating first if due.

        Args:
            record: A decoded Record; routed to a Channel by its `tag` field
                (falling back to `name`, then a fixed `dc.unknown` catch-all).
        """
        tag = record.get("tag") or record.get("name") or "dc.unknown"
        log_time = extract_log_time_ns(record, self._time_key)
        payload = json.dumps(record).encode("utf-8")
        with self._lock:
            channel_id = self._channel_id(tag)
            self._writer.add_message(
                channel_id,
                log_time=log_time,
                publish_time=log_time,
                data=payload,
                sequence=self._sequence,
            )
            self._sequence += 1
            self._rotate_if_needed()

    def write_records(self, records: Iterable[dict]) -> None:
        """Write each Record in `records` in order.

        Args:
            records: An iterable of decoded Records.
        """
        for record in records:
            self.write_record(record)

    def close(self) -> None:
        """Finish and close the file currently being written."""
        with self._lock:
            if self._writer is not None:
                self._writer.finish()
                self._file.close()
                logger.info("closed %s", self._current_path)
                self._writer = None
