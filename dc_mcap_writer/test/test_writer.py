"""Tests for RotatingMcapWriter and its NDJSON/timestamp helpers (#210)."""

import json
import time

import pytest
from dc_mcap_writer.writer import (
    RotatingMcapWriter,
    RotationPolicy,
    extract_log_time_ns,
    iter_ndjson,
)
from mcap.reader import make_reader


def _read_all(path):
    with open(path, "rb") as f:
        reader = make_reader(f)
        return list(reader.iter_messages())


def _record(tag="dc.measurement.uptime", date=1786118523000000000, **extra):
    record = {"tag": tag, "date": date, "name": "uptime"}
    record.update(extra)
    return record


def test_write_record_produces_a_valid_mcap_file(tmp_path):
    writer = RotatingMcapWriter(output_dir=tmp_path)
    writer.write_record(_record())
    writer.close()

    files = list(tmp_path.glob("*.mcap"))
    assert len(files) == 1

    messages = _read_all(files[0])
    assert len(messages) == 1
    schema, channel, message = messages[0]
    assert channel.topic == "dc.measurement.uptime"
    assert schema.encoding == "jsonschema"
    assert message.log_time == 1786118523000000000
    assert json.loads(message.data)["name"] == "uptime"


def test_distinct_tags_get_distinct_channels_in_one_file(tmp_path):
    writer = RotatingMcapWriter(output_dir=tmp_path)
    writer.write_record(_record(tag="dc.measurement.cpu"))
    writer.write_record(_record(tag="dc.measurement.memory"))
    writer.write_record(_record(tag="dc.measurement.cpu"))
    writer.close()

    messages = _read_all(next(tmp_path.glob("*.mcap")))
    topics = {channel.topic for _, channel, _ in messages}
    assert topics == {"dc.measurement.cpu", "dc.measurement.memory"}
    assert len(messages) == 3


def test_record_without_a_tag_falls_back_to_name_then_unknown(tmp_path):
    writer = RotatingMcapWriter(output_dir=tmp_path)
    writer.write_record({"name": "uptime", "date": 1})
    writer.write_record({"date": 1})
    writer.close()

    messages = _read_all(next(tmp_path.glob("*.mcap")))
    topics = {channel.topic for _, channel, _ in messages}
    assert topics == {"uptime", "dc.unknown"}


def test_max_bytes_rotation_opens_a_new_file(tmp_path):
    writer = RotatingMcapWriter(output_dir=tmp_path, rotation=RotationPolicy(max_bytes=1))
    writer.write_record(_record())
    writer.write_record(_record())
    writer.close()

    # Every message pushes the file straight past a 1-byte threshold, so each of the two
    # Records lands in its own rotation file — plus a final empty file, since the last
    # write is also the write that triggers the rotation `close()` then finishes.
    files = sorted(tmp_path.glob("*.mcap"))
    assert len(files) == 3
    all_messages = [message for f in files for message in _read_all(f)]
    assert len(all_messages) == 2


def test_max_bytes_rotation_reacts_to_real_message_volume_not_just_the_header(tmp_path):
    """Regression test: `max_bytes` must count actual written bytes, not just overhead.

    `mcap.writer.Writer`'s default chunking buffers every message in memory and never
    hands it to the file object until a 1 MiB chunk closes or `finish()` runs — so
    `self._file.tell()` (what `max_bytes` checks) would keep reading the same small
    header-only size no matter how many real messages had been queued, making
    `max_bytes` react only to per-file overhead, never to actual content.
    `RotatingMcapWriter` disables chunking specifically so this works.

    Args:
        tmp_path: pytest's per-test temporary directory fixture.
    """
    writer = RotatingMcapWriter(output_dir=tmp_path, rotation=RotationPolicy(max_bytes=300))
    for i in range(5):
        writer.write_record(_record(date=1786118523000000000 + i))
    writer.close()

    files = sorted(tmp_path.glob("*.mcap"))
    assert len(files) > 1, "5 real messages past a 300-byte threshold never rotated"
    all_messages = [message for f in files for message in _read_all(f)]
    assert len(all_messages) == 5


def test_max_duration_rotation_opens_a_new_file(tmp_path):
    writer = RotatingMcapWriter(
        output_dir=tmp_path, rotation=RotationPolicy(max_duration_secs=0.05)
    )
    writer.write_record(_record())
    time.sleep(0.1)
    writer.write_record(_record())
    writer.close()

    files = sorted(tmp_path.glob("*.mcap"))
    assert len(files) == 2


def test_no_rotation_policy_keeps_everything_in_one_file(tmp_path):
    writer = RotatingMcapWriter(output_dir=tmp_path)
    for _ in range(50):
        writer.write_record(_record())
    writer.close()

    assert len(list(tmp_path.glob("*.mcap"))) == 1
    assert len(_read_all(next(tmp_path.glob("*.mcap")))) == 50


def test_close_finishes_a_readable_file_even_with_no_records(tmp_path):
    writer = RotatingMcapWriter(output_dir=tmp_path)
    writer.close()

    files = list(tmp_path.glob("*.mcap"))
    assert len(files) == 1
    assert _read_all(files[0]) == []


@pytest.mark.parametrize(
    "time_key,value,expected_ns",
    [
        ("date", 1786118523000000000, 1786118523000000000),
        ("date", 1786118523.5, 1786118523500000000),
        # `datetime.fromisoformat` keeps microsecond precision, truncating the extra
        # nanosecond digits, so this matches the `double` time_format's own
        # documented lossiness (destinations.md), not a bug in the conversion.
        ("date", "2026-08-09T11:36:28.123456789", 1786275388123456000),
    ],
)
def test_extract_log_time_ns_handles_every_time_format(time_key, value, expected_ns):
    ns = extract_log_time_ns({time_key: value}, time_key)
    # float64 seconds only keeps ~microsecond precision (ADR-noted lossiness of
    # `double`/iso8601 with fractional seconds parsed back through a float `timestamp()`);
    # assert to the nearest microsecond rather than the exact nanosecond.
    assert abs(ns - expected_ns) < 1000


def test_extract_log_time_ns_falls_back_to_wall_clock_when_missing_or_unparsable():
    before = time.time_ns()
    assert extract_log_time_ns({}, "date") >= before
    assert extract_log_time_ns({"date": "not-a-timestamp"}, "date") >= before
    assert extract_log_time_ns({"date": True}, "date") >= before


def test_iter_ndjson_skips_blank_and_malformed_lines():
    stream = iter(
        [
            '{"a": 1}\n',
            "\n",
            "not json\n",
            '{"a": 2}\n',
        ]
    )
    assert list(iter_ndjson(stream)) == [{"a": 1}, {"a": 2}]
