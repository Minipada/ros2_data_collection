#!/bin/bash

# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Applies the KPI view definitions to any PostgreSQL holding DC Records — the demo's, or a
# real deployment's. Every object is CREATE OR REPLACE, so re-running is how a definition
# change ships. Uses psql when installed, a postgres container otherwise.

set -e

help() {
usage="$(basename "$0") [--host=HOST] [--port=PORT] [--user=USER] [--database=DB] [--file=FILE]

Apply the KPI views (availability and uptime) to a PostgreSQL database.

Arguments:
--help/-h     Show this help text
--host        Database host (default: 127.0.0.1)
--port        Database port (default: 5432)
--user        Database user, needs CREATE on the schema (default: dc)
--database    Database holding the dc table (default: dc)
--file        SQL file to apply (default: tools/infrastructure/sql/kpi_views.sql)

The password is read from PGPASSWORD (default: password).
"
    echo "${usage}"
}

SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

HOST="127.0.0.1"
PORT="5432"
USERNAME="dc"
DATABASE="dc"
FILE="$(dirname "${SCRIPTPATH}")/sql/kpi_views.sql"

for i in "$@"
do
case $i in
    --host=*)
    HOST="${i#*=}"
    shift
    ;;
    --port=*)
    PORT="${i#*=}"
    shift
    ;;
    --user=*)
    USERNAME="${i#*=}"
    shift
    ;;
    --database=*)
    DATABASE="${i#*=}"
    shift
    ;;
    --file=*)
    FILE="${i#*=}"
    shift
    ;;
    --help|-h)
    help
    exit 0
    ;;
    *)
    echo "Unknown argument: ${i}"
    help
    exit 1
    ;;
esac
done

if [ ! -f "${FILE}" ]; then
    echo "SQL file not found: ${FILE}"
    exit 1
fi

export PGPASSWORD="${PGPASSWORD:-password}"

if command -v psql &> /dev/null; then
    psql -h "${HOST}" -p "${PORT}" -U "${USERNAME}" -d "${DATABASE}" \
        -v ON_ERROR_STOP=1 -f - < "${FILE}"
else
    echo "psql not installed, running it from a container instead."
    podman run --rm -i --network host -e PGPASSWORD \
        docker.io/library/postgres:13 \
        psql -h "${HOST}" -p "${PORT}" -U "${USERNAME}" -d "${DATABASE}" \
        -v ON_ERROR_STOP=1 -f - < "${FILE}"
fi

echo "KPI views applied to ${DATABASE} on ${HOST}:${PORT}."
