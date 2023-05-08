import os
from enum import Enum

from pydantic import BaseSettings


class GetDataMode(str, Enum):
    TODAY = "Today"
    YESTERDAY = "Yesterday"
    THIS_WEEK = "This week"
    LAST_WEEK = "Last week"
    THIS_MONTH = "This month"
    LAST_3_MONTHS = "Last 3 months"
    SELECT_TIME = "Select time"
    RUN_ID = "Run id"


class Storage(str, Enum):
    # Follows JSON model structure
    MINIO = "minio"


class Backend(str, Enum):
    POSTGRESQL = "postgresql"


class Config(BaseSettings):
    APP_HOST: str = "0.0.0.0"
    APP_PORT: int = 80
    DEBUG: bool = True
    ENV: str = "dev"
    MINIO_ACCESS_KEY: str = os.getenv("MINIO_ACCESS_KEY", "fyTGdQUk1nTOu3VO")
    MINIO_SECRET_KEY: str = os.getenv("MINIO_SECRET_KEY", "fRIS95M8Qmn5Uwqgi1aeUIIOvGejK4qa")
    MINIO_URL: str = "localhost:9000"
    MINIO_BUCKET: str = "mybucket"
    PGSQL_TABLE: str = "dc"
    READER_DB_URL_DATA: str = "postgresql+psycopg2://dc:password@localhost:5432/dc"
    STORAGE: Storage = Storage.MINIO
    BACKEND: Backend = Backend.POSTGRESQL


# Edit here if you have different configuration for development
class DevelopmentConfig(Config):
    READER_DB_URL_DATA: str = "postgresql+psycopg2://dc:password@localhost:5432/dc"


# Edit here if you have different configuration for production
class ProductionConfig(Config):
    DEBUG: str = False
    READER_DB_URL_DATA: str = "postgresql+psycopg2://dc:password@localhost:5432/dc"


def get_config():
    env = os.getenv("ENV", "dev")
    config_type = {
        "dev": DevelopmentConfig(),
        "prod": ProductionConfig(),
    }
    return config_type[env]


config: Config = get_config()
