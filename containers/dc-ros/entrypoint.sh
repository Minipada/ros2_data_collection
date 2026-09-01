#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Entrypoint for the published `dc-ros` image (#448): launches the real DC stack and
# nothing else — no workload generator, no E2E-only params (contrast
# tools/e2e/scripts/entrypoint.sh, which is test-harness-only). Every argument this
# script receives is forwarded to `dc_bringup.launch.py` as-is, so a deployment
# selects unmanaged-shipper mode and its own params file with e.g.:
#
#   podman run ghcr.io/<repo>/dc-ros:<tag> \
#     dc_params_file:=/etc/dc/params.yaml run_uploader:=false
set -euo pipefail

# colcon/ROS setup files reference unset vars (COLCON_TRACE, AMENT_TRACE_SETUP_FILES,
# …); sourcing them under `set -u` aborts this script before the stack ever launches.
set +u
# shellcheck disable=SC1091
source /root/ws/install/setup.bash
set -u

exec ros2 launch dc_bringup dc_bringup.launch.py "$@"
