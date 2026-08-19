-- SPDX-FileCopyrightText: 2022-2026 David Bensoussan
--
-- SPDX-License-Identifier: MPL-2.0

-- Pre-created tables for the DC 2.0 demo infrastructure's PostgreSQL destinations.
-- dc_bridge's `postgres` destination is Vector's built-in `postgres` sink (see
-- dc_bridge/src/render.cpp), which maps each top-level key of a Record's JSON
-- payload onto an existing column of the same name — it does not create the table
-- or add columns itself, so every column a demo's Records can populate must exist
-- here up front. Columns are a superset across all dc_demos params files that use
-- the `pgsql`/`pgsql_files` destinations (qrcodes_minio_pgsql.yaml,
-- tb3_simulation_pgsql_minio.yaml); a given demo only ever populates the subset of
-- columns its own measurements/groups actually emit, the rest stay NULL for its rows.
--
-- Field names verified against the actual JSON-building code (not guessed):
-- dc_measurements/include/dc_measurements/measurement.hpp (common fields: name,
-- run_id, flattened, nested), .../plugins/measurements/camera.cpp (camera_name,
-- local_paths, remote_paths, base64, inspected), .../map.cpp (resolution, origin,
-- local_paths, remote_paths, width, height), .../tcp_health.cpp (host, port,
-- server_name, active), .../memory.cpp/os.cpp/uptime.cpp (used, os, kernel, cpus,
-- memory, free_percent, free, capacity, time — same fields tools/e2e/sql/init.sql
-- already verified for these same plugins), and dc_group/dc_group/group_server.py
-- (a merged Group Record's top-level keys are its members' own `group_key` names —
-- `cmd_vel`, `position`, `speed` for the "robot" group used here — plus `tags`).
--
-- The cpu Measurement's `sorted` per-process array and the position/speed/cmd_vel
-- Measurements' `linear`/`angular`/`x`/`y`/`yaw` components are still intentionally not
-- enumerated below: nothing charts them, so the sink silently drops those keys rather
-- than storing them — add columns here first if you wire up a panel that needs one.
CREATE TABLE IF NOT EXISTS dc (
  -- Common (every Record)
  date bigint,
  name text,
  robot_name text,
  id text,
  run_id text,
  flattened boolean,
  nested boolean,
  tags jsonb,
  plugins jsonb,
  -- The incident a Measurement released this Record under (#291). NULL for every Record
  -- collected outside an incident, which is most of them; the UUID the Trigger broadcast
  -- node minted for one firing otherwise, shared by every Record and File of that incident.
  incident_id text,
  -- System (tb3_simulation_pgsql_minio: memory/os/uptime)
  used double precision,
  os text,
  kernel text,
  cpus integer,
  memory double precision,
  free_percent double precision,
  free bigint,
  capacity bigint,
  time double precision,
  -- cpu.cpp's average CPU load and total process count (the demo dashboard's CPU panel)
  average double precision,
  processes integer,
  -- Robot (camera.cpp, map.cpp, and the "robot" group's merged cmd_vel/position/speed)
  camera_name text,
  local_paths jsonb,
  remote_paths jsonb,
  base64 jsonb,
  inspected jsonb,
  resolution double precision,
  origin jsonb,
  width integer,
  height integer,
  cmd_vel jsonb,
  position jsonb,
  speed jsonb,
  distance_traveled double precision,
  -- Twist magnitude, emitted by both speed.cpp and cmd_vel.cpp as standalone Records
  -- (tb3_simulation_pgsql_minio); told apart by `name`. Nested under the `speed`/`cmd_vel`
  -- jsonb columns above instead when the Records go through a Group (qrcodes_minio_pgsql).
  computed double precision,
  -- Operational events the KPI views read (tools/infrastructure/sql/kpi_views.sql):
  -- driving_type.cpp's current mode, and the transition Records the intervention (#362)
  -- and fault (#365) Measurements emit. Both are StateTransitionDetector projections, so
  -- they share previous_duration (how long the state just left was held), `sequence` (a
  -- dropped Record is a gap, not a wrong duration) and `open` (collection stopped inside
  -- the interval, so it has no duration rather than a zero one).
  mode text,
  from_mode text,
  to_mode text,
  component text,
  from_level text,
  to_level text,
  reason text,
  previous_duration double precision,
  sequence bigint,
  open boolean,
  -- Infrastructure (tcp_health.cpp)
  host text,
  port integer,
  server_name text,
  active boolean,
  -- Fast DDS statistics (fastdds_stats.cpp, #392, fastdds_stats_pgsql_grafana demo). `event` is
  -- always "sample" for this Measurement -- listed for querying, unlike the Measurements above
  -- that never populate it, none of which use this column.
  event text,
  domain_id integer,
  participant_count integer,
  datawriter_count integer,
  datareader_count integer,
  latency_ns_mean double precision,
  publication_throughput_bytes_per_sec_mean double precision,
  subscription_throughput_bytes_per_sec_mean double precision,
  rtps_packets_sent bigint,
  rtps_packets_lost bigint,
  participants jsonb,
  hosts jsonb,
  users jsonb,
  processes jsonb
);

-- File upload status + group-completion Records from dc_bridge's Uploader
-- (ADR-0005), verified against dc_bridge/src/uploader/status.cpp and group.cpp.
CREATE TABLE IF NOT EXISTS dc_files (
  kind text,
  group_name text,
  robot_name text,
  robot_id text,
  local_path text,
  remote_path text,
  storage_type text,
  uploaded boolean,
  on_filesystem boolean,
  deleted boolean,
  content_type text,
  size bigint,
  duration double precision,
  complete boolean,
  file_count integer,
  files jsonb,
  updated_at double precision
);
