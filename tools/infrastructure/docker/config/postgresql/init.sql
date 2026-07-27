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
-- CPU measurement fields are intentionally not enumerated below: no demo's Grafana
-- dashboard currently charts them (see doc/src/dc/demos/), so their JSON keys are
-- silently dropped by the sink rather than stored — add columns here first if you
-- wire up a CPU panel.
CREATE TABLE IF NOT EXISTS dc (
  -- Common (every Record)
  date double precision,
  name text,
  robot_name text,
  id text,
  run_id text,
  flattened boolean,
  nested boolean,
  tags jsonb,
  plugins jsonb,
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
  -- Infrastructure (tcp_health.cpp)
  host text,
  port integer,
  server_name text,
  active boolean
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
