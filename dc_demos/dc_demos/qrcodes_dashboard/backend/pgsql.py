import datetime

from config import Storage
from lib import pgsql_session
from models import RobotData
from sqlalchemy import asc, desc, func, select


class PGSQLService:
    def get_unique_robots(self):
        query = select(RobotData.data["robot_name"].label("robot_name"))

        query = query.group_by("robot_name").order_by(asc("robot_name"))
        result = pgsql_session.execute(query)
        result = result.all()

        return result

    def get_start_end_time(
        self,
        *,
        robot_name: str | None = "",
    ):
        query = select(
            func.min(RobotData.time).label("start_time"),
            func.max(RobotData.time).label("end_time"),
        )

        if robot_name:
            query = query.where(RobotData.data["robot_name"].as_string() == robot_name)

        # query = query..order_by(desc("start_time"))
        result = pgsql_session.execute(query)
        result = result.one()

        return result

    def get_unique_run_ids(
        self,
        *,
        robot_name: str | None = "",
        start_date: str | None = "",
        end_date: str | None = "",
    ):
        if start_date:
            subquery_start = select(RobotData.data["run_id"]).where(
                RobotData.time >= datetime.datetime.strptime(start_date, "%Y-%m-%d %H:%M:%S")
            )
            if robot_name:
                subquery_start = subquery_start.where(
                    RobotData.data["robot_name"].as_string() == robot_name
                )

        if end_date:
            subquery_end = select(RobotData.data["run_id"]).where(
                RobotData.time <= datetime.datetime.strptime(end_date, "%Y-%m-%d %H:%M:%S")
            )
            if robot_name:
                subquery_end = subquery_end.where(
                    RobotData.data["robot_name"].as_string() == robot_name
                )

        query = select(
            RobotData.data["robot_name"].label("robot_name"),
            RobotData.data["run_id"].label("run_id"),
            func.min(RobotData.time).label("start_time"),
            func.max(RobotData.time).label("end_time"),
        )

        if start_date:
            query = query.where(RobotData.data["run_id"].in_(subquery_start))
        if end_date:
            query = query.where(RobotData.data["run_id"].in_(subquery_end))
        if robot_name:
            query = query.where(RobotData.data["robot_name"].as_string() == robot_name)

        query = query.group_by("run_id", "robot_name").order_by(desc("start_time"))
        result = pgsql_session.execute(query)
        result = result.all()

        return result

    def get_os(self, *, robot_name: str):
        # Find last time with os present
        subquery_last = (
            select(
                RobotData.data["robot_name"].label("robot_name"),
                func.max(RobotData.time).label("max_time"),
            )
            .where(RobotData.data["robot_name"].as_string() == robot_name)
            .where(RobotData.data["os"].isnot(None))
            .where(RobotData.data["cpus"].isnot(None))
            .where(RobotData.data["kernel"].isnot(None))
            .where(RobotData.data["memory"].isnot(None))
            .group_by("robot_name")
            .alias("subquery_last")
        )

        query = (
            select(
                RobotData.data["os"].label("os"),
                RobotData.data["cpus"].label("cpus"),
                RobotData.data["kernel"].label("kernel"),
                RobotData.data["memory"].label("memory"),
            )
            .where(RobotData.data["robot_name"].as_string() == robot_name)
            .where(RobotData.time == subquery_last.c.max_time)
        )

        result = pgsql_session.execute(query)
        result = result.first()

        return result

    def get_memory(self, *, robot_name: str, run_id: str):
        query = (
            select(
                RobotData.time.label("time"),
                RobotData.data["used"],
            )
            .where(RobotData.data["robot_name"].as_string() == robot_name)
            .where(RobotData.data["name"].as_string() == "memory")
            .where(RobotData.data["used"].isnot(None))
            .where(RobotData.data["run_id"].as_string() == run_id)
            .order_by(asc("time"))
        )

        result = pgsql_session.execute(query)
        result = result.all()

        return result

    def get_cpu_average(self, *, robot_name: str, run_id: str):
        query = (
            select(
                RobotData.time.label("time"),
                RobotData.data["average"],
                RobotData.data["processes"],
            )
            .where(RobotData.data["robot_name"].as_string() == robot_name)
            .where(RobotData.data["name"].as_string() == "cpu")
            .where(RobotData.data["average"].isnot(None))
            .where(RobotData.data["processes"].isnot(None))
            .where(RobotData.data["run_id"].as_string() == run_id)
            .order_by(asc("time"))
        )

        result = pgsql_session.execute(query)
        result = result.all()

        return result

    def get_speed(self, *, robot_name: str, run_id: str):
        query = (
            select(
                RobotData.time.label("time"),
                RobotData.data["computed"],
            )
            .where(RobotData.data["robot_name"].as_string() == robot_name)
            .where(RobotData.data["name"].as_string() == "speed")
            .where(RobotData.data["computed"].isnot(None))
            .where(RobotData.data["run_id"].as_string() == run_id)
            .order_by(asc("time"))
        )

        result = pgsql_session.execute(query)
        result = result.all()

        return result

    def get_cmd_vel(self, *, robot_name: str, run_id: str):
        query = (
            select(
                RobotData.time.label("time"),
                RobotData.data["computed"],
            )
            .where(RobotData.data["robot_name"].as_string() == robot_name)
            .where(RobotData.data["name"].as_string() == "cmd_vel")
            .where(RobotData.data["computed"].isnot(None))
            .where(RobotData.data["run_id"].as_string() == run_id)
            .order_by(asc("time"))
        )

        result = pgsql_session.execute(query)
        result = result.all()

        return result

    def get_total_distance(self, *, robot_name: str, run_id: str | None = ""):
        query = (
            select(func.sum(RobotData.data["distance_traveled"].as_float()))
            .where(RobotData.data["robot_name"].as_string() == robot_name)
            .where(RobotData.data["name"].as_string() == "distance_traveled")
            .where(RobotData.data["distance_traveled"].isnot(None))
        )

        if run_id:
            query = query.where(RobotData.data["run_id"].as_string() == run_id)

        result = pgsql_session.execute(query)
        result = result.one()[0]
        if not result:
            result = 0

        return result

    def get_average_speed(self, *, robot_name: str, run_id: str | None = ""):
        query = (
            select(func.avg(RobotData.data["computed"].as_float()))
            .where(RobotData.data["robot_name"].as_string() == robot_name)
            .where(RobotData.data["name"].as_string() == "speed")
            .where(RobotData.data["computed"].isnot(None))
        )

        if run_id:
            query = query.where(RobotData.data["run_id"].as_string() == run_id)

        result = pgsql_session.execute(query)
        result = result.one()[0]

        if not result:
            result = 0

        return result

    def get_last_map(
        self, *, robot_name: str, run_id: str | None = "", storage: Storage = Storage.MINIO
    ):
        # Find last time with map present
        subquery_last = (
            select(
                func.max(RobotData.time).label("max_time"),
            )
            .where(RobotData.data["robot_name"].as_string() == robot_name)
            .where(RobotData.data["name"].as_string() == "map")
            .where(RobotData.data["remote_paths"][storage]["png"].isnot(None))
            .where(RobotData.data["remote_paths"][storage]["pgm"].isnot(None))
            .where(RobotData.data["remote_paths"][storage]["yaml"].isnot(None))
            .alias("subquery_last")
        )
        query = (
            select(
                RobotData.data["remote_paths"][storage]["png"],
                RobotData.data["remote_paths"][storage]["pgm"],
                RobotData.data["remote_paths"][storage]["yaml"],
            )
            .where(RobotData.data["robot_name"].as_string() == robot_name)
            .where(RobotData.data["remote_paths"][storage]["png"].isnot(None))
            .where(RobotData.data["remote_paths"][storage]["pgm"].isnot(None))
            .where(RobotData.data["remote_paths"][storage]["yaml"].isnot(None))
            .where(RobotData.time == subquery_last.c.max_time)
        )

        if run_id:
            query = query.where(RobotData.data["run_id"].as_string() == run_id)

        result = pgsql_session.execute(query)
        result = result.first()

        return result

    def get_tcp_health(self, *, robot_name: str, run_id: str | None = ""):
        query = (
            select(
                RobotData.data["server_name"],
                RobotData.data["host"],
                RobotData.data["port"],
                RobotData.data["active"],
                RobotData.time.label("time"),
            )
            .where(RobotData.data["robot_name"].as_string() == robot_name)
            .where(RobotData.data["server_name"].isnot(None))
            .order_by(asc("time"))
        )
        if run_id:
            query = query.where(RobotData.data["run_id"].as_string() == run_id)

        result = pgsql_session.execute(query)
        result = result.all()

        return result

    def get_camera_images(
        self,
        *,
        robot_name: str,
        camera_name: str | None = "",
        run_id: str | None = "",
        storage: Storage = Storage.MINIO,
    ):
        query = (
            select(
                RobotData.data["camera_name"].label("camera_name"),
                RobotData.data["remote_paths"][storage]["raw"].label(f"remote_paths.{storage}.raw"),
                RobotData.data["remote_paths"][storage]["rotated"].label(
                    f"remote_paths.{storage}.rotated"
                ),
                RobotData.data["remote_paths"][storage]["inspected"].label(
                    f"remote_paths.{storage}.inspected"
                ),
                RobotData.data["run_id"],
                RobotData.time.label("time"),
            )
            .where(RobotData.data["robot_name"].as_string() == robot_name)
            .where(RobotData.data["camera_name"].isnot(None))
            .order_by(asc("time"))
        )

        if camera_name:
            query = query.where(RobotData.data["camera_name"] == camera_name)

        if run_id:
            query = query.where(RobotData.data["run_id"].as_string() == run_id)

        result = pgsql_session.execute(query)
        result = result.all()

        return result