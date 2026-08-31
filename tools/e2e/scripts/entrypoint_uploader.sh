#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Entrypoint for the dc-uploader container of the three-container split topology (#447):
# runs the standalone dc_uploader binary (#446, docs/adr/0014) directly, configured
# entirely by DC_UPLOADER_* env vars from compose.split.yaml — no ROS launch, no ROS
# params. Sourcing install/setup.bash is still needed to put dc_uploader's shared
# library dependencies (dc_common, aws_sdk_vendor, ...) on LD_LIBRARY_PATH, even though
# it has no rclcpp/ROS-node dependency of its own.
set -euo pipefail

# colcon/ROS setup files reference unset vars under `set -u` (see entrypoint.sh).
set +u
# shellcheck disable=SC1091
source /root/ws/install/setup.bash
set -u

exec "$(ros2 pkg prefix dc_bridge)/lib/dc_bridge/dc_uploader"
