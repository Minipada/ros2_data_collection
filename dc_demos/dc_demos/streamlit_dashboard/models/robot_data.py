# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""ORM view of the flat `dc` table dc_bridge's `postgres` Destination writes into.

The DC 1.x embedded Fluent Bit `out_pgsql` plugin stored a whole Record as one JSON blob
under a single `data` column. DC 2.0's Destination is Vector's built-in `postgres` sink
(`dc_bridge/src/render.cpp`), which maps each **top-level key** of a Record's JSON payload
onto an **existing column of the same name** — it creates neither tables nor columns. So
there is no `data` blob to path into any more: every field is its own column, and a column
that no Record populates simply stays NULL (see #272).

The column list below mirrors `tools/infrastructure/docker/config/postgresql/init.sql`,
which is the schema the demo compose stack creates. Columns are a superset across the demos,
so most are NULL for any given row; the measurement that populates each one is noted.

**Which demo this dashboard reads**: `dc_demos/params/tb3_simulation_pgsql_minio.yaml`. Its
Measurements are exactly this dashboard's pages (cpu/memory/os -> System, speed/cmd_vel/
distance_traveled/camera -> Robot, map -> Environment, `*_health` -> Infrastructure), and
each is shipped to Postgres as its own Record, so its fields land on top-level columns.
The other pgsql demo (`qrcodes_minio_pgsql.yaml`) ships a merged **Group** Record instead,
whose top-level keys are its members' `group_key`s (`cmd_vel`, `position`, `speed` as jsonb
objects) and which carries no top-level `robot_name`/`run_id` at all — it cannot drive this
dashboard's robot/run selectors. That demo is covered by the Grafana dashboards in
`tools/infrastructure/docker/config/grafana/dashboards/` instead.
"""

from config import config
from lib import PGBase
from sqlalchemy import BigInteger, Boolean, Column, Float, Integer, Text, cast, func
from sqlalchemy.dialects.postgresql import JSONB
from sqlalchemy.types import TIMESTAMP


class RobotData(PGBase):
    """One row of the `dc` table — a single Record, one column per top-level JSON key."""

    __tablename__ = config.PGSQL_TABLE

    # --- Common to every Record -------------------------------------------------------
    # The Record timestamp, as epoch **nanoseconds** — see `RECORD_TIME` below. Declared
    # primary key because SQLAlchemy's declarative mapper requires one; the real table has
    # none (Vector only ever INSERTs). Harmless here: this model is read-only and every
    # query selects columns rather than entities, so no identity map is involved.
    date = Column(BigInteger, primary_key=True)
    name = Column(Text)
    robot_name = Column(Text)
    # `id` is the machine-id the demo reads out of /etc/machine-id, mapped under a
    # non-builtin attribute name.
    machine_id = Column("id", Text)
    run_id = Column(Text)
    flattened = Column(Boolean)
    nested = Column(Boolean)
    tags = Column(JSONB)
    plugins = Column(JSONB)

    # --- System: memory / os / uptime / cpu --------------------------------------------
    used = Column(Float)  # memory
    os = Column(Text)
    kernel = Column(Text)
    cpus = Column(Integer)
    memory = Column(Float)
    free_percent = Column(Float)
    free = Column(BigInteger)
    capacity = Column(BigInteger)
    # `time` is the **uptime** Measurement's seconds-since-boot, not the Record timestamp
    # (that is `date`). Mapped under a different attribute name so the two can't be
    # confused the way they were before #272.
    uptime = Column("time", Float)
    average = Column(Float)  # cpu
    processes = Column(Integer)  # cpu

    # --- Robot: camera / map / speed / cmd_vel / distance_traveled ---------------------
    camera_name = Column(Text)
    local_paths = Column(JSONB)
    remote_paths = Column(JSONB)
    base64 = Column(JSONB)
    inspected = Column(JSONB)
    resolution = Column(Float)  # map
    origin = Column(JSONB)  # map
    width = Column(Integer)  # map
    height = Column(Integer)  # map
    # Magnitude of the twist, emitted by both the speed and the cmd_vel Measurements —
    # they are separate Records, told apart by `name`.
    computed = Column(Float)
    distance_traveled = Column(Float)
    # Group-Record columns, populated by qrcodes_minio_pgsql.yaml's "robot" Group only.
    cmd_vel = Column(JSONB)
    position = Column(JSONB)
    speed = Column(JSONB)

    # --- Infrastructure: tcp_health ----------------------------------------------------
    host = Column(Text)
    port = Column(Integer)
    server_name = Column(Text)
    active = Column(Boolean)


# The demos configure the postgres Destination with `time_key: "date"` and leave
# `time_format` at its `epoch_nanos` default (dc_bridge/src/render.cpp), so the Record
# timestamp arrives as an i64 of nanoseconds. Every page here wants a real timestamp, so
# queries select and order on this expression — the same conversion the demo's Grafana
# dashboards use. Cast back down to a naive `timestamp` (local time, as the server sees it)
# so pandas/Streamlit keep comparing it against the naive datetimes the sidebar produces.
RECORD_TIME = cast(func.to_timestamp(RobotData.date / 1e9), TIMESTAMP)


def to_epoch_nanos(moment) -> int:
    """Convert a Python datetime to the epoch-nanosecond integer stored in `date`.

    Filtering on the raw `date` column keeps the comparison exact and index-friendly,
    rather than converting every row before comparing it.

    Args:
        moment (datetime.datetime): Naive local, or timezone-aware, datetime to convert

    Returns:
        int: Nanoseconds since the epoch, as dc_bridge's postgres Destination stores them
    """
    return int(moment.timestamp() * 1e9)
