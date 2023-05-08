import datetime
import json

from config import config
from sqlalchemy import create_engine
from sqlalchemy.orm import declarative_base, sessionmaker


class DatetimeEncoder(json.JSONEncoder):
    def default(self, obj: datetime.datetime):
        try:
            return super().default(obj)
        except TypeError:
            return obj.isoformat()


engine = create_engine(config.READER_DB_URL_DATA, json_serializer=DatetimeEncoder)

PGBase = declarative_base()

SessionLocal = sessionmaker(bind=engine)

pgsql_session = SessionLocal()
