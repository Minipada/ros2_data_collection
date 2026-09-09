# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Unit tests for verify_zero_loss.py's incident/retention check profiles (#496).

The two profiles absorbed run_incident.sh's and run_retention.sh's inline SQL, so their
thresholds (what counts as "flowing", "silent", "a released window", "shed", "uploaded")
are now verdicts of this module and get the same kind of unit coverage the zero-loss
harness's own helpers get elsewhere in this directory. psql/scalar_int are fake; nothing
here needs a container.
"""

import os
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts"))

import verify_zero_loss as vzl


def test_the_live_measurement_must_be_actually_delivering():
    violations = []
    vzl.check_live_flowing(vzl.MIN_LIVE_ROWS, "dc.measurement.memory", 180, violations, {})
    assert violations == []
    vzl.check_live_flowing(vzl.MIN_LIVE_ROWS - 1, "dc.measurement.memory", 180, violations, {})
    assert len(violations) == 1


def test_armed_means_silent_means_zero_rows():
    violations = []
    vzl.check_armed_silent(0, "dc.measurement.uptime", violations, {})
    assert violations == []
    vzl.check_armed_silent(1, "dc.measurement.uptime", violations, {})
    assert len(violations) == 1


def test_a_released_window_is_checked_on_four_axes(monkeypatch):
    counts = {"wrong_tag": 0, "other_ids": 0, "live_tagged": 0}

    def fake_scalar_int(pg, query):
        # The three side queries are told apart by their predicates, not by the id/tag
        # literals they interpolate (those repeat across queries).
        if "AND tag <>" in query:
            return counts["wrong_tag"]
        if "AND incident_id <>" in query:
            return counts["other_ids"]
        if "AND incident_id IS NOT NULL" in query:
            return counts["live_tagged"]
        return vzl.MIN_INCIDENT_ROWS

    monkeypatch.setattr(vzl, "scalar_int", fake_scalar_int)

    violations = []
    details = {}
    vzl.check_released_window(
        "pg",
        "e2e-incident-0001",
        "dc.measurement.uptime",
        "dc.measurement.memory",
        vzl.MIN_INCIDENT_ROWS,
        violations,
        details,
    )
    assert violations == []

    counts.update(wrong_tag=1, other_ids=2, live_tagged=3)
    vzl.check_released_window(
        "pg",
        "e2e-incident-0001",
        "dc.measurement.uptime",
        "dc.measurement.memory",
        vzl.MIN_INCIDENT_ROWS - 1,
        violations,
        details,
    )
    # A short window, a window row from another Measurement, a foreign id, and a tagged
    # live row are four separate failures — not one lumped "window looks wrong".
    assert len(violations) == 4


def test_retention_verdicts_are_one_row_or_a_failure():
    shed, uploaded = [], []
    vzl.check_shed_row(1, 480, shed, {})
    vzl.check_uploaded_row(1, 60, uploaded, {})
    assert shed == [] and uploaded == []
    vzl.check_shed_row(0, 480, shed, {})
    vzl.check_uploaded_row(0, 60, uploaded, {})
    assert len(shed) == 1 and len(uploaded) == 1


def test_wait_for_count_stops_polling_once_the_minimum_is_reached(monkeypatch):
    calls = []

    def fake_scalar_int(pg, query):
        calls.append(query)
        return len(calls)

    monkeypatch.setattr(vzl, "scalar_int", fake_scalar_int)
    assert vzl.wait_for_count("pg", "SELECT count(*) FROM dc_records", 3, 60) == 3
    assert len(calls) == 3


def test_wait_for_count_returns_the_last_count_when_the_wait_expires(monkeypatch):
    clock = {"now": 0.0}
    monkeypatch.setattr(
        vzl,
        "time",
        SimpleNamespace(
            monotonic=lambda: clock["now"],
            sleep=lambda s: clock.__setitem__("now", clock["now"] + s),
        ),
    )
    monkeypatch.setattr(vzl, "scalar_int", lambda pg, query: 1)
    assert vzl.wait_for_count("pg", "SELECT count(*) FROM dc_records", 5, 10) == 1


def _run_main(monkeypatch, argv, scalar_int=None, psql=None):
    monkeypatch.setattr(sys, "argv", ["verify_zero_loss.py"] + argv)
    if scalar_int is not None:
        monkeypatch.setattr(vzl, "scalar_int", scalar_int)
    if psql is not None:
        monkeypatch.setattr(vzl, "psql", psql)
    return vzl.main()


def test_incident_and_retention_need_their_stage_and_its_arguments(monkeypatch):
    with pytest.raises(SystemExit):
        _run_main(monkeypatch, ["--profile", "incident"])
    with pytest.raises(SystemExit):
        _run_main(monkeypatch, ["--profile", "retention", "--stage", "shed"])
    with pytest.raises(SystemExit):
        _run_main(
            monkeypatch,
            [
                "--profile",
                "incident",
                "--stage",
                "armed",
                "--timeout-seconds",
                "5",
            ],
        )
    with pytest.raises(SystemExit):
        _run_main(monkeypatch, ["--stage", "armed", "--ledger-file", "ledger.txt"])


def test_the_zero_loss_profile_still_needs_the_ledger(monkeypatch):
    with pytest.raises(SystemExit):
        _run_main(monkeypatch, [])


def test_incident_armed_stage_passes_when_the_pipeline_flows_and_the_armed_tag_is_silent(
    monkeypatch,
):
    def fake_psql(pg, query):
        if "information_schema" in query:
            return "text"
        if "dc.measurement.memory" in query:
            return str(vzl.MIN_LIVE_ROWS)
        return "0"

    argv = [
        "--profile",
        "incident",
        "--stage",
        "armed",
        "--timeout-seconds",
        "180",
        "--incident-id",
        "e2e-incident-0001",
        "--live-tag",
        "dc.measurement.memory",
        "--buffered-tag",
        "dc.measurement.uptime",
    ]
    assert _run_main(monkeypatch, argv, psql=fake_psql) == 0


def test_incident_released_stage_fails_when_no_row_carries_the_id(monkeypatch):
    clock = {"now": 0.0}
    monkeypatch.setattr(
        vzl,
        "time",
        SimpleNamespace(
            monotonic=lambda: clock["now"],
            sleep=lambda s: clock.__setitem__("now", clock["now"] + s),
        ),
    )
    argv = [
        "--profile",
        "incident",
        "--stage",
        "released",
        "--timeout-seconds",
        "5",
        "--incident-id",
        "e2e-incident-0001",
        "--live-tag",
        "dc.measurement.memory",
        "--buffered-tag",
        "dc.measurement.uptime",
    ]
    assert _run_main(monkeypatch, argv, scalar_int=lambda pg, query: 0) == 1


def test_retention_stage_fails_when_no_file_was_shed(monkeypatch):
    clock = {"now": 0.0}
    monkeypatch.setattr(
        vzl,
        "time",
        SimpleNamespace(
            monotonic=lambda: clock["now"],
            sleep=lambda s: clock.__setitem__("now", clock["now"] + s),
        ),
    )
    assert (
        _run_main(
            monkeypatch,
            ["--profile", "retention", "--stage", "shed", "--timeout-seconds", "5"],
            scalar_int=lambda pg, query: 0,
        )
        == 1
    )


def test_exec_sql_is_the_psql_seam_the_harness_shells_out_to(monkeypatch, capsys):
    monkeypatch.setattr(vzl, "psql", lambda pg, query: "7")
    assert _run_main(monkeypatch, ["--exec-sql", "SELECT count(*) FROM dc_records"]) == 0
    assert capsys.readouterr().out.strip() == "7"


def test_the_profile_table_is_the_whole_surface():
    assert sorted(vzl.PROFILE_STAGES) == ["incident", "retention", "zero-loss"]
    assert vzl.PROFILE_STAGES["zero-loss"] == ()
    assert vzl.PROFILE_STAGES["incident"] == ("armed", "released")
    assert vzl.PROFILE_STAGES["retention"] == ("shed", "uploaded")


def test_a_run_writes_its_profile_and_stage_into_the_report(monkeypatch, tmp_path):
    report = tmp_path / "report.json"
    argv = [
        "--profile",
        "retention",
        "--stage",
        "uploaded",
        "--timeout-seconds",
        "5",
        "--report",
        str(report),
    ]
    clock = {"now": 0.0}
    monkeypatch.setattr(
        vzl,
        "time",
        SimpleNamespace(
            monotonic=lambda: clock["now"],
            sleep=lambda s: clock.__setitem__("now", clock["now"] + s),
        ),
    )
    monkeypatch.setattr(vzl, "scalar_int", lambda pg, query: 3)
    assert _run_main(monkeypatch, argv) == 0

    import json

    written = json.loads(report.read_text())
    assert written["profile"] == "retention"
    assert written["stage"] == "uploaded"
    assert written["pass"] is True
