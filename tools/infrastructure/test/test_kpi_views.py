# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Fixture test for the starter KPI views (tools/infrastructure/sql/kpi_views.sql).

Seeds a known uptime Record sequence into PostgreSQL and asserts what the views report
over it. The SQL files under test are the ones the demo stack applies, run into a throwaway
schema of their own — tools/infrastructure/scripts/test_kpi_views.sh brings up the database.
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
