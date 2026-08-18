# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Fixture tests for the shared raw-volume accounting (tools/e2e/scripts/raw_volume.py).

Two consumers now publish figures from this module — the E2E harness's zero-loss verifier
and the sim-backed benchmark (#326) whose numbers doc/src/dc/raw_topics.md quotes — so
what a Record is counted as costing is worth pinning down.
"""

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts"))

import raw_volume


def write_ndjson(path, events):
    with open(path, "w") as f:
        for event in events:
            f.write(json.dumps(event, sort_keys=True) + "\n")
    return str(path)


@pytest.fixture
def two_tags(tmp_path):
    """Ten Records over two Tags, one arriving every second."""
    events = [
        {"tag": "dc.raw.imu", "timestamp": f"2026-08-18T09:00:{i:02d}.000Z", "value": i}
        for i in range(10)
    ]
    events += [
        {"tag": "dc.raw.odom", "timestamp": f"2026-08-18T09:00:{i:02d}.000Z", "pad": "x" * 100}
        for i in range(4)
    ]
    return write_ndjson(tmp_path / "raw.ndjson", events)


def test_a_record_costs_its_line_plus_its_newline(tmp_path):
    path = write_ndjson(tmp_path / "raw.ndjson", [{"tag": "dc.raw.imu"}])
    report = raw_volume.summarize_by_tag(path)
    assert report["total"]["stored_bytes"] == os.path.getsize(path)
    assert report["total"]["bytes_per_record"] == os.path.getsize(path)


def test_volume_is_split_per_tag_and_summed(two_tags):
    report = raw_volume.summarize_by_tag(two_tags)
    tags = report["tags"]
    assert set(tags) == {"dc.raw.imu", "dc.raw.odom"}
    assert tags["dc.raw.imu"]["records"] == 10
    assert tags["dc.raw.odom"]["records"] == 4
    assert report["total"]["records"] == 14
    assert (
        tags["dc.raw.imu"]["stored_bytes"] + tags["dc.raw.odom"]["stored_bytes"]
        == report["total"]["stored_bytes"]
    )
    # Biggest first: the report's whole job is saying what to exclude.
    stored = [v["stored_bytes"] for v in tags.values()]
    assert stored == sorted(stored, reverse=True)


def test_arrival_rates_come_from_the_sink_timestamps(two_tags):
    imu = raw_volume.summarize_by_tag(two_tags)["tags"]["dc.raw.imu"]
    assert imu["span_seconds"] == 9.0
    assert imu["records_per_second"] == round(10 / 9, 2)
    assert imu["bytes_per_second"] == round(imu["stored_bytes"] / 9, 1)
    assert imu["projected_mb_per_day"] == round(imu["stored_bytes"] / 9 * 86400 / 1e6, 1)


def test_arrival_rates_are_omitted_when_asked(two_tags):
    imu = raw_volume.summarize_by_tag(two_tags, arrival_rates=False)["tags"]["dc.raw.imu"]
    assert "span_seconds" not in imu
    assert imu["bytes_per_record"] > 0


def test_a_span_under_a_second_is_not_a_rate(tmp_path):
    path = write_ndjson(
        tmp_path / "raw.ndjson",
        [{"tag": "dc.raw.imu", "timestamp": f"2026-08-18T09:00:00.{i:03d}Z"} for i in range(3)],
    )
    assert "bytes_per_second" not in raw_volume.summarize_by_tag(path)["tags"]["dc.raw.imu"]


def test_unreadable_lines_still_cost_their_bytes(tmp_path):
    path = tmp_path / "raw.ndjson"
    path.write_text('{"tag": "dc.raw.imu"}\nnot json at all\n\n')
    report = raw_volume.summarize_by_tag(str(path))
    assert report["total"]["records"] == 2
    assert report["tags"]["(unparsable)"]["records"] == 1
    # Everything but the blank line, which is no Record and cost nothing to store.
    assert report["total"]["stored_bytes"] == os.path.getsize(path) - 1


def test_the_projection_caps_the_publish_rate_at_the_limiter():
    summary = {"records": 2000, "bytes_per_record": 500}
    projected = raw_volume.project(summary, sim_seconds=10.0, max_rate_hz=10.0)
    assert projected["published_hz"] == 200.0
    assert projected["shipped_hz"] == 10.0
    assert projected["projected_bytes_per_second"] == 5000.0
    assert projected["projected_gb_per_day"] == round(5000 * 86400 / 1e9, 3)


def test_a_topic_slower_than_the_limiter_is_projected_whole():
    summary = {"records": 50, "bytes_per_record": 600}
    projected = raw_volume.project(summary, sim_seconds=10.0, max_rate_hz=10.0)
    assert projected["published_hz"] == 5.0
    assert projected["shipped_hz"] == 5.0
    assert projected["projected_bytes_per_second"] == 3000.0


def test_an_unlimited_profile_projects_the_full_publish_rate():
    summary = {"records": 2000, "bytes_per_record": 500}
    assert raw_volume.project(summary, 10.0, max_rate_hz=0)["shipped_hz"] == 200.0


def test_nothing_measured_projects_nothing():
    assert raw_volume.project({"records": 0, "bytes_per_record": None}, 10.0, 10.0) == {}
    assert raw_volume.project({"records": 5, "bytes_per_record": 100}, 0.0, 10.0) == {}
