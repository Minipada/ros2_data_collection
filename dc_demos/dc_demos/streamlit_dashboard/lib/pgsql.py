# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

from config import config
from sqlalchemy import create_engine
from sqlalchemy.orm import declarative_base, sessionmaker

# Read-only: the dashboard only ever SELECTs. dc_bridge's `postgres` Destination (Vector's
# postgres sink) is what writes this table, and it writes flat columns rather than a JSON
# blob — so nothing here serialises JSON on the way in any more (#272).
engine = create_engine(config.READER_DB_URL_DATA)

PGBase = declarative_base()

SessionLocal = sessionmaker(bind=engine)

pgsql_session = SessionLocal()
