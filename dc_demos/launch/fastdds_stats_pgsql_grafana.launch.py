# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Fast DDS network statistics into PostgreSQL, shown on a provisioned Grafana dashboard.

Hardware-free: one Measurement (fastdds_stats) and a `postgres` Destination, matching #304's
provisioning convention (tools/infrastructure/docker/config/grafana/dashboards/fastdds_stats.json
is picked up automatically). Only produces data when Fast DDS is the RMW and was built with its
Statistics Module enabled -- see doc/src/dc/demos/fastdds_stats_pgsql_grafana.md.
"""

from dc_demos.launch_common import dc_demo_launch_description


def generate_launch_description():
    return dc_demo_launch_description("fastdds_stats_pgsql_grafana.yaml")
