from config import config
from lib import PGBase
from sqlalchemy import Column, DateTime
from sqlalchemy.dialects.postgresql import JSON


class RobotData(PGBase):
    __tablename__ = config.PGSQL_TABLE

    time = Column(DateTime, nullable=False, primary_key=True)
    data = Column(JSON, nullable=False)
