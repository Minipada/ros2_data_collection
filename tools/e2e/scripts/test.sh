#!/bin/bash
# Runs `colcon test` (C++ gtest across every dc_* package, including dc_bridge) against
# the already-built tools/e2e/Containerfile image, plus C++ coverage. Used identically
# by a developer locally and by .github/workflows/ci.yaml — there's no separate CI-only
# test script; this is it (see CLAUDE.md "Containers: Podman, not Docker").
#
# dc_bridge's own store-backed tests (if any need external services) expect Postgres
# (127.0.0.1:5432, dc/password/dc) and an S3-compatible store (127.0.0.1:9000,
# rustfsadmin/rustfsadmin, bucket `dc-records`); ci.yaml starts them itself, and the
# harness (run.sh) exercises the full pipeline against real stores.
#
# Usage: IMAGE=dc-e2e:latest ./tools/e2e/scripts/test.sh [coverage-output-dir]
set -euo pipefail

IMAGE="${IMAGE:-dc-workspace:latest}"
COVERAGE_OUT="${1:-$(pwd)/coverage}"
mkdir -p "$COVERAGE_OUT"

podman run --rm --network host \
  -v "$COVERAGE_OUT:/root/ws/coverage" \
  --entrypoint bash \
  "$IMAGE" \
  -c '
    set -euo pipefail
    cd /root/ws
    # shellcheck disable=SC1091
    source install/setup.bash

    set +e
    colcon test --event-handlers desktop_notification- status-
    TEST_EXIT=$?
    colcon test-result --verbose
    RESULT_EXIT=$?
    set -e

    # C++ coverage (lcov/fastcov, reused from the humble CI pipeline).
    src/ros2_data_collection/tools/code_coverage_report.bash ci || true
    [ -f lcov/total_coverage.info ] && cp lcov/total_coverage.info coverage/cpp.info

    if [ "$TEST_EXIT" -ne 0 ] || [ "$RESULT_EXIT" -ne 0 ]; then
      echo "colcon test reported failures" >&2
      exit 1
    fi
  '
