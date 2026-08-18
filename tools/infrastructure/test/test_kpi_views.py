# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Fixture test for the KPI views (tools/infrastructure/sql/kpi_views.sql).

Seeds a known Record sequence into PostgreSQL and asserts what the views report over it —
one section per metric. The SQL files under test are the ones the demo stack applies, run
into a throwaway schema of their own — tools/infrastructure/scripts/test_kpi_views.sh
brings up the database.

The intervention and fault sections seed the Records the Measurements in #362 and #365 will
emit; nothing collects them yet, which is exactly why a definition change has to break a
test here rather than a dashboard later.
"""

import os
import pathlib
import uuid
from datetime import UTC, datetime, timedelta

import psycopg2
import psycopg2.extras
import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
TABLES_SQL = (
    REPO_ROOT / "tools" / "infrastructure" / "docker" / "config" / "postgresql" / "init.sql"
)
VIEWS_SQL = REPO_ROOT / "tools" / "infrastructure" / "sql" / "kpi_views.sql"

DSN = os.environ.get("DC_KPI_TEST_DSN", "postgresql://dc:password@127.0.0.1:5432/dc")

T0 = datetime(2026, 1, 1, tzinfo=UTC)
STEP = 5  # the demos' uptime polling interval, in seconds
GRACE = 30  # dc_kpi_max_gap()
ROBOT = "Turtlebot"


def at(offset):
    """Wall clock `offset` seconds into the test sequence."""
    return T0 + timedelta(seconds=offset)


@pytest.fixture(scope="session")
def connection():
    # No skip-if-absent: a run without a database is a broken run, not a passing one.
    conn = psycopg2.connect(DSN)
    conn.autocommit = True
    schema = f"dc_kpi_test_{uuid.uuid4().hex[:8]}"
    with conn.cursor() as cur:
        cur.execute(f'CREATE SCHEMA "{schema}"')
        cur.execute(f'SET search_path TO "{schema}"')
        cur.execute(TABLES_SQL.read_text())
        cur.execute(VIEWS_SQL.read_text())
    yield conn
    with conn.cursor() as cur:
        cur.execute(f'DROP SCHEMA "{schema}" CASCADE')
    conn.close()


@pytest.fixture
def db(connection):
    with connection.cursor(cursor_factory=psycopg2.extras.DictCursor) as cur:
        cur.execute("TRUNCATE dc")
        yield cur


def heartbeat(db, first=0, last=1800, step=STEP, silences=(), robot=ROBOT, uptime_at_start=1000.0):
    """Seed one uptime Record every `step` seconds, minus the (from, to) `silences`."""
    rows = [
        (int(at(offset).timestamp()) * 10**9, "uptime", robot, uptime_at_start + offset)
        for offset in range(first, last + 1, step)
        if not any(start < offset < end for start, end in silences)
    ]
    psycopg2.extras.execute_values(
        db, 'INSERT INTO dc (date, name, robot_name, "time") VALUES %s', rows
    )
    return rows


def availability(db, start, end):
    """dc_kpi_availability() over [start, end], keyed by robot."""
    db.execute("SELECT * FROM dc_kpi_availability(%s, %s)", (at(start), at(end)))
    return {row["robot_name"]: row for row in db.fetchall()}


def test_samples_view_only_reports_uptime_records(db):
    heartbeat(db, first=0, last=10)
    db.execute(
        "INSERT INTO dc (date, name, robot_name, used) VALUES (%s, %s, %s, %s)",
        (int(T0.timestamp()) * 10**9, "memory", ROBOT, 42.0),
    )
    db.execute(
        "INSERT INTO dc (date, name, robot_name) VALUES (%s, %s, %s)",
        (int(T0.timestamp()) * 10**9, "uptime", ROBOT),
    )

    db.execute("SELECT * FROM dc_kpi_uptime_samples ORDER BY sample_time")
    samples = db.fetchall()

    assert [row["uptime_seconds"] for row in samples] == [1000.0, 1005.0, 1010.0]
    assert [row["sample_time"] for row in samples] == [at(0), at(5), at(10)]
    assert samples[0]["prev_sample_time"] is None
    assert samples[2]["prev_sample_time"] == at(5)


def test_unbroken_heartbeat_is_fully_available(db):
    heartbeat(db)

    kpi = availability(db, 300, 900)[ROBOT]

    assert kpi["availability"] == pytest.approx(1.0)
    assert kpi["covered_seconds"] == pytest.approx(600.0)
    assert kpi["window_seconds"] == pytest.approx(600.0)
    assert kpi["samples"] == 600 // STEP + 1
    assert kpi["first_sample"] == at(300)
    assert kpi["last_sample"] == at(900)


def test_silence_beyond_the_grace_period_is_downtime(db):
    heartbeat(db, silences=[(600, 900)])

    kpi = availability(db, 300, 1200)[ROBOT]

    # The 300 s silence costs everything past the grace period the last Record vouches for.
    assert kpi["covered_seconds"] == pytest.approx(900.0 - (300 - GRACE))
    assert kpi["availability"] == pytest.approx(630.0 / 900.0)
    assert kpi["samples"] == 2 * (300 // STEP + 1)


def test_silence_within_the_grace_period_still_counts_as_up(db):
    heartbeat(db, silences=[(600, 620)])

    kpi = availability(db, 300, 900)[ROBOT]

    assert kpi["availability"] == pytest.approx(1.0)
    assert kpi["samples"] == 600 // STEP + 1 - 3


def test_the_window_is_a_query_parameter(db):
    heartbeat(db, silences=[(600, 900)])

    # Same Records, three windows: before the silence, around it, after it.
    assert availability(db, 0, 600)[ROBOT]["availability"] == pytest.approx(1.0)
    assert availability(db, 600, 900)[ROBOT]["availability"] == pytest.approx(GRACE / 300.0)
    assert availability(db, 900, 1500)[ROBOT]["availability"] == pytest.approx(1.0)


def test_records_outside_the_window_are_not_counted(db):
    heartbeat(db, first=0, last=600)

    kpi = availability(db, 300, 900)[ROBOT]

    # Coverage stops at the last Record; the Records before the window only feed the
    # first in-window Record its predecessor.
    assert kpi["samples"] == 300 // STEP + 1
    assert kpi["first_sample"] == at(300)
    assert kpi["last_sample"] == at(600)
    assert kpi["covered_seconds"] == pytest.approx(300.0)
    assert kpi["availability"] == pytest.approx(0.5)


def test_each_robot_is_reported_separately(db):
    heartbeat(db, robot="Turtlebot")
    heartbeat(db, robot="Waffle", silences=[(600, 900)], uptime_at_start=50.0)

    kpi = availability(db, 300, 1200)

    assert set(kpi) == {"Turtlebot", "Waffle"}
    assert kpi["Turtlebot"]["availability"] == pytest.approx(1.0)
    assert kpi["Waffle"]["availability"] == pytest.approx(630.0 / 900.0)


def test_uptime_is_the_latest_reading_in_the_window(db):
    heartbeat(db)

    assert availability(db, 300, 900)[ROBOT]["uptime_seconds"] == pytest.approx(1900.0)
    assert availability(db, 300, 1200)[ROBOT]["uptime_seconds"] == pytest.approx(2200.0)


def test_a_window_without_records_reports_nothing(db):
    heartbeat(db, first=600, last=1200)

    assert availability(db, 0, 300) == {}


def test_the_bucketed_view_charts_the_same_metric(db):
    heartbeat(db, first=0, last=1800, silences=[(600, 900)])

    db.execute(
        "SELECT * FROM dc_kpi_availability_5m"
        " WHERE bucket_start >= %s AND bucket_start < %s ORDER BY bucket_start",
        (at(300), at(1500)),
    )
    buckets = db.fetchall()

    assert [row["bucket_start"] for row in buckets] == [at(300), at(600), at(900), at(1200)]
    assert [row["samples"] for row in buckets] == [60, 1, 60, 60]
    # Only the bucket the silence falls in loses availability.
    assert [round(row["availability"], 6) for row in buckets] == [
        1.0,
        round(STEP / 300.0, 6),
        1.0,
        1.0,
    ]
    assert buckets[-1]["uptime_seconds"] == pytest.approx(2495.0)


# --- Utilisation -----------------------------------------------------------------------


def driving(db, first=0, last=600, step=STEP, mode="autonomous", robot=ROBOT):
    """Seed one driving_type Record every `step` seconds; `mode` may be a callable."""
    pick = mode if callable(mode) else (lambda _offset: mode)
    rows = [
        (int(at(offset).timestamp()) * 10**9, "driving_type", robot, pick(offset))
        for offset in range(first, last + 1, step)
    ]
    psycopg2.extras.execute_values(
        db, "INSERT INTO dc (date, name, robot_name, mode) VALUES %s", rows
    )
    return rows


def speeds(db, first=0, last=600, step=STEP, speed=0.4, robot=ROBOT):
    """Seed one speed Record every `step` seconds; `speed` may be a callable."""
    pick = speed if callable(speed) else (lambda _offset: speed)
    rows = [
        (int(at(offset).timestamp()) * 10**9, "speed", robot, pick(offset))
        for offset in range(first, last + 1, step)
    ]
    psycopg2.extras.execute_values(
        db, "INSERT INTO dc (date, name, robot_name, computed) VALUES %s", rows
    )
    return rows


def utilisation(db, start, end):
    """dc_kpi_utilisation() over [start, end], keyed by robot."""
    db.execute("SELECT * FROM dc_kpi_utilisation(%s, %s)", (at(start), at(end)))
    return {row["robot_name"]: row for row in db.fetchall()}


def test_utilisation_is_moving_time_over_reported_time(db):
    driving(db, last=1200)
    speeds(db, last=1200, speed=lambda offset: 0.4 if offset <= 600 else 0.0)

    kpi = utilisation(db, 300, 900)[ROBOT]

    assert kpi["reported_seconds"] == pytest.approx(600.0)
    assert kpi["autonomous_seconds"] == pytest.approx(600.0)
    assert kpi["productive_seconds"] == pytest.approx(300.0)
    assert kpi["utilisation"] == pytest.approx(0.5)
    assert kpi["speed_samples"] == 600 // STEP + 1


def test_time_in_an_unknown_mode_is_reported_but_never_productive(db):
    driving(db, last=1200, mode=lambda offset: "autonomous" if offset <= 600 else "unknown")
    speeds(db, last=1200)

    kpi = utilisation(db, 300, 900)[ROBOT]

    assert kpi["autonomous_seconds"] == pytest.approx(300.0)
    assert kpi["unknown_seconds"] == pytest.approx(300.0)
    # Moving the whole window, but only the half under a known mode counts.
    assert kpi["productive_seconds"] == pytest.approx(300.0)
    assert kpi["utilisation"] == pytest.approx(0.5)


def test_a_human_driving_the_robot_is_still_utilisation(db):
    driving(db, last=1200, mode=lambda offset: "autonomous" if offset <= 600 else "teleop")
    speeds(db, last=1200)

    kpi = utilisation(db, 300, 900)[ROBOT]

    assert kpi["teleop_seconds"] == pytest.approx(300.0)
    assert kpi["manual_seconds"] == pytest.approx(0.0)
    assert kpi["utilisation"] == pytest.approx(1.0)


def test_utilisation_without_speed_records_is_unknown_not_zero(db):
    driving(db, last=1200)

    kpi = utilisation(db, 300, 900)[ROBOT]

    assert kpi["speed_samples"] == 0
    assert kpi["reported_seconds"] == pytest.approx(600.0)
    assert kpi["productive_seconds"] == pytest.approx(0.0)
    assert kpi["utilisation"] is None


def test_a_speed_record_older_than_the_grace_period_does_not_vouch_for_movement(db):
    driving(db, last=1200)
    # Movement reported once, then silence: it vouches for one grace period, no longer.
    speeds(db, first=300, last=300)

    kpi = utilisation(db, 300, 900)[ROBOT]

    # The sample at the window's own start credits nothing, so a grace period minus one
    # polling interval is all that speed Record can vouch for.
    assert kpi["productive_seconds"] == pytest.approx(GRACE - STEP)
    assert kpi["utilisation"] == pytest.approx((GRACE - STEP) / 600.0)


def test_utilisation_ignores_another_robots_speed(db):
    driving(db, last=1200, robot="Turtlebot")
    speeds(db, last=1200, robot="Waffle")

    kpi = utilisation(db, 300, 900)["Turtlebot"]

    assert kpi["speed_samples"] == 0
    assert kpi["utilisation"] is None


def test_the_bucketed_utilisation_view_charts_the_same_metric(db):
    driving(db, last=1200)
    speeds(db, last=1200, speed=lambda offset: 0.4 if offset <= 600 else 0.0)

    db.execute(
        "SELECT * FROM dc_kpi_utilisation_5m"
        " WHERE bucket_start >= %s AND bucket_start < %s ORDER BY bucket_start",
        (at(300), at(900)),
    )
    buckets = db.fetchall()

    assert [row["bucket_start"] for row in buckets] == [at(300), at(600)]
    assert [round(row["utilisation"], 6) for row in buckets] == [1.0, round(STEP / 300.0, 6)]


# --- Intervention rate -----------------------------------------------------------------


def intervention(db, offset, from_mode, to_mode, previous_duration, sequence, open_=False):
    """One intervention Record: a driving-mode transition the takeover is derived from."""
    db.execute(
        "INSERT INTO dc (date, name, robot_name, from_mode, to_mode, previous_duration,"
        ' "sequence", "open") VALUES (%s, %s, %s, %s, %s, %s, %s, %s)',
        (
            int(at(offset).timestamp()) * 10**9,
            "intervention",
            ROBOT,
            from_mode,
            to_mode,
            previous_duration,
            sequence,
            open_,
        ),
    )


def travelled(db, first=0, last=600, step=STEP, metres=10.0, robot=ROBOT):
    """Seed distance_traveled Records, each carrying the distance since the previous one."""
    rows = [
        (int(at(offset).timestamp()) * 10**9, "distance_traveled", robot, metres)
        for offset in range(first, last + 1, step)
    ]
    psycopg2.extras.execute_values(
        db, "INSERT INTO dc (date, name, robot_name, distance_traveled) VALUES %s", rows
    )
    return rows


def intervention_rate(db, start, end):
    """dc_kpi_intervention_rate() over [start, end], keyed by robot."""
    db.execute("SELECT * FROM dc_kpi_intervention_rate(%s, %s)", (at(start), at(end)))
    return {row["robot_name"]: row for row in db.fetchall()}


def test_intervention_rate_is_per_autonomous_hour_and_per_kilometre(db):
    driving(db, last=1200)
    # 600 s autonomous and 1 km inside the window, with two takeovers in it.
    travelled(db, first=305, last=900, metres=1000.0 / (600 // STEP))
    intervention(db, 400, "autonomous", "teleop", 100.0, 1)
    intervention(db, 460, "teleop", "autonomous", 60.0, 2)
    intervention(db, 700, "autonomous", "manual", 240.0, 3)
    intervention(db, 760, "manual", "autonomous", 60.0, 4)

    kpi = intervention_rate(db, 300, 900)[ROBOT]

    assert kpi["interventions"] == 2
    assert kpi["ended_interventions"] == 2
    assert kpi["autonomous_seconds"] == pytest.approx(600.0)
    assert kpi["distance_km"] == pytest.approx(1.0)
    assert kpi["per_autonomous_hour"] == pytest.approx(12.0)
    assert kpi["per_km"] == pytest.approx(2.0)
    assert kpi["mean_intervention_seconds"] == pytest.approx(60.0)
    assert kpi["total_intervention_seconds"] == pytest.approx(120.0)


def test_an_intervention_still_running_is_counted_but_never_timed(db):
    driving(db, last=1200)
    intervention(db, 400, "autonomous", "teleop", 100.0, 1)
    intervention(db, 460, "teleop", "autonomous", 60.0, 2)
    # A takeover the window ends inside: it happened, but it has no duration yet.
    intervention(db, 880, "autonomous", "manual", 420.0, 3, open_=True)

    kpi = intervention_rate(db, 300, 900)[ROBOT]

    assert kpi["interventions"] == 2
    assert kpi["ended_interventions"] == 1
    assert kpi["open_interventions"] == 1
    assert kpi["mean_intervention_seconds"] == pytest.approx(60.0)
    assert kpi["total_intervention_seconds"] == pytest.approx(60.0)


def test_a_denominator_nothing_reported_gives_no_rate(db):
    intervention(db, 400, "autonomous", "teleop", 100.0, 1)

    kpi = intervention_rate(db, 300, 900)[ROBOT]

    assert kpi["interventions"] == 1
    assert kpi["autonomous_seconds"] is None
    assert kpi["distance_km"] is None
    assert kpi["per_autonomous_hour"] is None
    assert kpi["per_km"] is None


def test_a_robot_with_no_interventions_still_reports_its_denominators(db):
    driving(db, last=1200)
    travelled(db, first=305, last=900)

    kpi = intervention_rate(db, 300, 900)[ROBOT]

    assert kpi["interventions"] == 0
    assert kpi["per_autonomous_hour"] == pytest.approx(0.0)
    assert kpi["mean_intervention_seconds"] is None


def test_the_bucketed_intervention_view_charts_the_same_events(db):
    intervention(db, 400, "autonomous", "teleop", 100.0, 1)
    intervention(db, 460, "teleop", "autonomous", 60.0, 2)

    db.execute("SELECT * FROM dc_kpi_interventions_1h ORDER BY bucket_start")
    buckets = db.fetchall()

    assert [row["interventions"] for row in buckets] == [1]
    assert buckets[0]["mean_intervention_seconds"] == pytest.approx(60.0)


# --- MTBF and MTTR ---------------------------------------------------------------------


def fault(db, offset, component, from_level, to_level, previous_duration, sequence, open_=False):
    """One fault Record: a diagnostic level change of one component."""
    db.execute(
        "INSERT INTO dc (date, name, robot_name, component, from_level, to_level,"
        ' previous_duration, "sequence", reason, "open")'
        " VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s)",
        (
            int(at(offset).timestamp()) * 10**9,
            "fault",
            ROBOT,
            component,
            from_level,
            to_level,
            previous_duration,
            sequence,
            f"{component} {to_level}",
            open_,
        ),
    )


def reliability(db, start, end, failure_levels=None):
    """dc_kpi_reliability() over [start, end], keyed by component."""
    if failure_levels is None:
        db.execute("SELECT * FROM dc_kpi_reliability(%s, %s)", (at(start), at(end)))
    else:
        db.execute(
            "SELECT * FROM dc_kpi_reliability(%s, %s, %s)",
            (at(start), at(end), failure_levels),
        )
    return {row["component"]: row for row in db.fetchall()}


def test_mtbf_and_mttr_average_the_records_own_durations(db):
    # Two failures, each preceded by healthy time and followed by a repair.
    fault(db, 400, "motors", "OK", "ERROR", 400.0, 1)
    fault(db, 460, "motors", "ERROR", "OK", 60.0, 2)
    fault(db, 700, "motors", "OK", "ERROR", 240.0, 3)
    fault(db, 820, "motors", "ERROR", "OK", 120.0, 4)

    kpi = reliability(db, 300, 900)["motors"]

    assert kpi["failures"] == 2
    assert kpi["repairs"] == 2
    assert kpi["mtbf_seconds"] == pytest.approx(320.0)
    assert kpi["mttr_seconds"] == pytest.approx(90.0)
    assert kpi["downtime_seconds"] == pytest.approx(180.0)


def test_a_change_inside_the_failure_set_is_not_a_second_failure(db):
    # A component that goes quiet while already broken is still one fault.
    fault(db, 400, "motors", "OK", "ERROR", 400.0, 1)
    fault(db, 430, "motors", "ERROR", "STALE", 30.0, 2)
    fault(db, 490, "motors", "STALE", "OK", 60.0, 3)

    kpi = reliability(db, 300, 900)["motors"]

    assert kpi["failures"] == 1
    assert kpi["repairs"] == 1
    assert kpi["mttr_seconds"] == pytest.approx(60.0)


def test_a_warning_is_not_a_failure(db):
    fault(db, 400, "motors", "OK", "WARN", 400.0, 1)
    fault(db, 460, "motors", "WARN", "OK", 60.0, 2)

    assert reliability(db, 300, 900) == {}


def test_the_failure_levels_are_a_query_parameter(db):
    fault(db, 400, "lidar", "OK", "STALE", 400.0, 1)
    fault(db, 460, "lidar", "STALE", "OK", 60.0, 2)

    assert reliability(db, 300, 900)["lidar"]["failures"] == 1
    # A deployment that treats a silent component as reportable but not broken.
    assert reliability(db, 300, 900, ["ERROR"]) == {}


def test_a_fault_never_cleared_has_no_repair_time(db):
    fault(db, 400, "motors", "OK", "ERROR", 400.0, 1, open_=True)

    kpi = reliability(db, 300, 900)["motors"]

    assert kpi["failures"] == 1
    assert kpi["repairs"] == 0
    assert kpi["open_faults"] == 1
    assert kpi["mtbf_seconds"] == pytest.approx(400.0)
    assert kpi["mttr_seconds"] is None
    assert kpi["downtime_seconds"] == pytest.approx(0.0)


def test_each_component_is_reported_separately(db):
    fault(db, 400, "motors", "OK", "ERROR", 400.0, 1)
    fault(db, 460, "motors", "ERROR", "OK", 60.0, 2)
    fault(db, 500, "lidar", "OK", "STALE", 500.0, 1)

    kpi = reliability(db, 300, 900)

    assert set(kpi) == {"motors", "lidar"}
    assert kpi["lidar"]["mttr_seconds"] is None
    assert kpi["motors"]["mttr_seconds"] == pytest.approx(60.0)


def test_the_bucketed_fault_view_charts_the_same_events(db):
    fault(db, 400, "motors", "OK", "ERROR", 400.0, 1)
    fault(db, 460, "motors", "ERROR", "OK", 60.0, 2)

    db.execute("SELECT * FROM dc_kpi_faults_1h ORDER BY bucket_start, component")
    buckets = db.fetchall()

    assert [row["failures"] for row in buckets] == [1]
    assert [row["repairs"] for row in buckets] == [1]
    assert buckets[0]["downtime_seconds"] == pytest.approx(60.0)
