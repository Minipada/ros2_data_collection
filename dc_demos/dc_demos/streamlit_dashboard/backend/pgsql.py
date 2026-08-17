# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Queries against the flat `dc` table dc_bridge's `postgres` Destination writes.

Every query here reads top-level columns. See `models/robot_data.py` for why there is no
`data` JSON blob to path into any more (#272) and which demo this dashboard reads.
"""

import datetime

from config import Storage
from lib import pgsql_session
from models import RECORD_TIME, RobotData, to_epoch_nanos
from sqlalchemy import asc, desc, func, select


def add_mode_query(
    query,
    run_id: str | None = "",
    start_date: datetime.datetime | None = None,
    end_date: datetime.datetime | None = None,
):
    if run_id:
        query = query.where(RobotData.run_id == run_id)
    else:
        if start_date:
            query = query.where(RobotData.date >= to_epoch_nanos(start_date))
        if end_date:
            query = query.where(RobotData.date <= to_epoch_nanos(end_date))
    return query


def remote_path(storage: Storage, key: str):
    """Path into the `remote_paths` jsonb column, as text.

    Measurements that upload Files record them as
    `remote_paths.<destination name>.<file key>` (dc_measurements' camera.cpp / map.cpp),
    so the storage level is the dc_bridge Destination the File went to.

    Args:
        storage (Storage): dc_bridge Destination the File was uploaded to
        key (str): The Measurement's own name for that File, e.g. "png" or "raw"

    Returns:
        The object path as text, or NULL when this Record uploaded no such File
    """
    return RobotData.remote_paths[(storage.value, key)].astext


class PGSQLService:
    @staticmethod
    def get_unique_robots():
        query = (
            select(RobotData.robot_name.label("robot_name"))
            .where(RobotData.robot_name.isnot(None))
            .group_by("robot_name")
            .order_by(asc("robot_name"))
        )
        result = pgsql_session.execute(query)
        result = result.all()

        return result

    @staticmethod
    def get_start_end_date(
        *,
        robot_name: str | None = "",
    ):
        query = select(
            func.min(RECORD_TIME).label("start_date"),
            func.max(RECORD_TIME).label("end_date"),
        )

        if robot_name:
            query = query.where(RobotData.robot_name == robot_name)

        result = pgsql_session.execute(query)
        result = result.one()

        return result

    @staticmethod
    def get_unique_run_ids(
        *,
        robot_name: str | None = "",
        start_date: str | None = "",
        end_date: str | None = "",
    ):
        query = select(
            RobotData.robot_name.label("robot_name"),
            RobotData.run_id.label("run_id"),
            func.min(RECORD_TIME).label("start_date"),
            func.max(RECORD_TIME).label("end_date"),
        ).where(RobotData.run_id.isnot(None))

        if start_date:
            subquery_start = select(RobotData.run_id).where(
                RobotData.date
                >= to_epoch_nanos(datetime.datetime.strptime(start_date, "%Y-%m-%d %H:%M:%S"))
            )
            if robot_name:
                subquery_start = subquery_start.where(RobotData.robot_name == robot_name)
            query = query.where(RobotData.run_id.in_(subquery_start))

        if end_date:
            subquery_end = select(RobotData.run_id).where(
                RobotData.date
                <= to_epoch_nanos(datetime.datetime.strptime(end_date, "%Y-%m-%d %H:%M:%S"))
            )
            if robot_name:
                subquery_end = subquery_end.where(RobotData.robot_name == robot_name)
            query = query.where(RobotData.run_id.in_(subquery_end))

        if robot_name:
            query = query.where(RobotData.robot_name == robot_name)

        query = query.group_by("run_id", "robot_name").order_by(desc("start_date"))
        result = pgsql_session.execute(query)
        result = result.all()

        return result

    @staticmethod
    def get_os(
        *,
        robot_name: str,
        run_id: str | None = "",
        start_date: datetime.datetime | None = None,
        end_date: datetime.datetime | None = None,
    ):
        query = (
            select(
                RobotData.os.label("os"),
                RobotData.cpus.label("cpus"),
                RobotData.kernel.label("kernel"),
                RobotData.memory.label("memory"),
            )
            .where(RobotData.robot_name == robot_name)
            .where(RobotData.os.isnot(None))
            .where(RobotData.cpus.isnot(None))
            .where(RobotData.kernel.isnot(None))
            .where(RobotData.memory.isnot(None))
            .order_by(desc(RobotData.date))
        )

        query = add_mode_query(query, run_id=run_id, start_date=start_date, end_date=end_date)

        result = pgsql_session.execute(query)
        result = result.first()

        return result

    @staticmethod
    def get_memory(
        *,
        robot_name: str,
        run_id: str | None = "",
        start_date: datetime.datetime | None = None,
        end_date: datetime.datetime | None = None,
    ):
        query = (
            select(
                RECORD_TIME.label("time"),
                RobotData.used,
            )
            .where(RobotData.robot_name == robot_name)
            .where(RobotData.name == "memory")
            .where(RobotData.used.isnot(None))
            .order_by(asc(RobotData.date))
        )

        query = add_mode_query(query, run_id=run_id, start_date=start_date, end_date=end_date)

        result = pgsql_session.execute(query)
        result = result.all()

        return result

    @staticmethod
    def get_cpu_average(
        *,
        robot_name: str,
        run_id: str | None = "",
        start_date: datetime.datetime | None = None,
        end_date: datetime.datetime | None = None,
    ):
        query = (
            select(
                RECORD_TIME.label("time"),
                RobotData.average,
                RobotData.processes,
            )
            .where(RobotData.robot_name == robot_name)
            .where(RobotData.name == "cpu")
            .where(RobotData.average.isnot(None))
            .where(RobotData.processes.isnot(None))
            .order_by(asc(RobotData.date))
        )

        query = add_mode_query(query, run_id=run_id, start_date=start_date, end_date=end_date)

        result = pgsql_session.execute(query)
        result = result.all()

        return result

    @staticmethod
    def get_speed(
        *,
        robot_name: str,
        run_id: str | None = "",
        start_date: datetime.datetime | None = None,
        end_date: datetime.datetime | None = None,
    ):
        query = (
            select(
                RECORD_TIME.label("time"),
                RobotData.computed,
            )
            .where(RobotData.robot_name == robot_name)
            .where(RobotData.name == "speed")
            .where(RobotData.computed.isnot(None))
            .order_by(asc(RobotData.date))
        )

        query = add_mode_query(query, run_id=run_id, start_date=start_date, end_date=end_date)

        result = pgsql_session.execute(query)
        result = result.all()

        return result

    @staticmethod
    def get_cmd_vel(
        *,
        robot_name: str,
        run_id: str | None = "",
        start_date: datetime.datetime | None = None,
        end_date: datetime.datetime | None = None,
    ):
        query = (
            select(
                RECORD_TIME.label("time"),
                RobotData.computed,
            )
            .where(RobotData.robot_name == robot_name)
            .where(RobotData.name == "cmd_vel")
            .where(RobotData.computed.isnot(None))
            .order_by(asc(RobotData.date))
        )

        query = add_mode_query(query, run_id=run_id, start_date=start_date, end_date=end_date)

        result = pgsql_session.execute(query)
        result = result.all()

        return result

    @staticmethod
    def get_total_distance(
        *,
        robot_name: str,
        run_id: str | None = "",
        start_date: datetime.datetime | None = None,
        end_date: datetime.datetime | None = None,
    ):
        query = (
            select(func.sum(RobotData.distance_traveled))
            .where(RobotData.robot_name == robot_name)
            .where(RobotData.name == "distance_traveled")
            .where(RobotData.distance_traveled.isnot(None))
        )

        query = add_mode_query(query, run_id=run_id, start_date=start_date, end_date=end_date)

        result = pgsql_session.execute(query)
        result = result.one()[0]

        if not result:
            result = -1

        return result

    @staticmethod
    def get_average_speed(
        *,
        robot_name: str,
        run_id: str | None = "",
        start_date: datetime.datetime | None = None,
        end_date: datetime.datetime | None = None,
    ):
        query = (
            select(func.avg(RobotData.computed))
            .where(RobotData.robot_name == robot_name)
            .where(RobotData.name == "speed")
            .where(RobotData.computed.isnot(None))
        )

        query = add_mode_query(query, run_id=run_id, start_date=start_date, end_date=end_date)

        result = pgsql_session.execute(query)
        result = result.one()[0]

        if not result:
            result = -1

        return result

    @staticmethod
    def get_last_map(
        *,
        robot_name: str,
        run_id: str | None = "",
        start_date: datetime.datetime | None = None,
        end_date: datetime.datetime | None = None,
        storage: Storage = Storage.RUSTFS,
    ):
        complete_map = (
            (RobotData.name == "map")
            & remote_path(storage, "png").isnot(None)
            & remote_path(storage, "pgm").isnot(None)
            & remote_path(storage, "yaml").isnot(None)
        )

        # Newest Record that uploaded a complete map (all three Files) within the selected
        # run / time range.
        subquery_last = (
            select(func.max(RobotData.date))
            .where(RobotData.robot_name == robot_name)
            .where(complete_map)
        )
        subquery_last = add_mode_query(
            subquery_last, run_id=run_id, start_date=start_date, end_date=end_date
        )

        query = (
            select(
                remote_path(storage, "png"),
                remote_path(storage, "pgm"),
                remote_path(storage, "yaml"),
            )
            .where(RobotData.robot_name == robot_name)
            .where(complete_map)
            .where(RobotData.date == subquery_last.scalar_subquery())
        )

        result = pgsql_session.execute(query)
        result = result.first()

        return result

    @staticmethod
    def get_tcp_health(
        *,
        robot_name: str,
        run_id: str | None = "",
        start_date: datetime.datetime | None = None,
        end_date: datetime.datetime | None = None,
    ):
        query = (
            select(
                RECORD_TIME.label("time"),
                RobotData.active,
                RobotData.server_name,
                RobotData.host,
                RobotData.port,
            )
            .where(RobotData.robot_name == robot_name)
            .where(RobotData.server_name.isnot(None))
            .order_by(asc(RobotData.date))
        )
        query = add_mode_query(query, run_id=run_id, start_date=start_date, end_date=end_date)

        result = pgsql_session.execute(query)
        result = result.all()

        return result

    @staticmethod
    def get_camera_images(
        *,
        robot_name: str,
        camera_name: str | None = "",
        run_id: str | None = "",
        start_date: datetime.datetime | None = None,
        end_date: datetime.datetime | None = None,
        storage: Storage = Storage.RUSTFS,
    ):
        query = (
            select(
                RECORD_TIME.label("time"),
                RobotData.camera_name.label("camera_name"),
                remote_path(storage, "raw").label("raw"),
                remote_path(storage, "rotated").label("rotated"),
                remote_path(storage, "inspected").label("inspected"),
                RobotData.run_id,
            )
            .where(RobotData.robot_name == robot_name)
            .where(RobotData.camera_name.isnot(None))
            .order_by(asc(RobotData.date))
        )

        if camera_name:
            query = query.where(RobotData.camera_name == camera_name)

        query = add_mode_query(query, run_id=run_id, start_date=start_date, end_date=end_date)

        result = pgsql_session.execute(query)
        result = result.all()

        return result
