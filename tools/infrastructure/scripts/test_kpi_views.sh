#!/bin/bash

# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Runs tools/infrastructure/test against a throwaway PostgreSQL: the KPI views' fixture
# test applies the repo's own SQL into a schema of its own, so nothing else is needed.

set -euo pipefail

REPO_ROOT="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
CONTAINER="${DC_KPI_TEST_CONTAINER:-dc_kpi_views_test}"
PORT="${DC_KPI_TEST_PORT:-55432}"

cleanup() {
    podman rm -f "${CONTAINER}" >/dev/null 2>&1 || true
}
trap cleanup EXIT
cleanup

podman run -d --name "${CONTAINER}" \
    -e POSTGRES_USER=dc -e POSTGRES_PASSWORD=password -e POSTGRES_DB=dc \
    -p "${PORT}:5432" \
    docker.io/library/postgres:13 >/dev/null

timeout 90 bash -c "until podman exec ${CONTAINER} pg_isready -U dc >/dev/null 2>&1; do sleep 1; done"

DC_KPI_TEST_DSN="postgresql://dc:password@127.0.0.1:${PORT}/dc" \
    uv run --frozen --directory "${REPO_ROOT}" pytest tools/infrastructure/test "$@"
