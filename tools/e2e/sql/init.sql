-- SPDX-FileCopyrightText: 2022-2026 David Bensoussan
--
-- SPDX-License-Identifier: MPL-2.0

-- Pre-created tables for the E2E harness's Postgres destinations (dc_bridge/render.rs's
-- postgres sink maps JSON event keys onto existing columns 1:1 — it does not create the
-- table or add columns itself, see dc_bridge/dc_bridge_core/tests/end_to_end.rs).
--
-- dc_records: every "receives: records" Measurement in tools/e2e/params/e2e_params.yaml
-- lands here. `tag` (added to every event by Vector's fluent source) + `date` (the
-- normalized event timestamp, per the destination's time_key/time_format) are the
-- verification key tools/e2e/scripts/verify_zero_loss.py uses.
--
-- `date` is bigint, not double precision: the default time_format is epoch_nanos, an
-- exact integer count of nanoseconds since the epoch (#308). A double would round it
-- back off below roughly microsecond resolution, which is what made timestamps useless
-- for telling two Records of a fast Measurement apart.
CREATE TABLE IF NOT EXISTS dc_records (
  date bigint,
  tag text,
  group_key text,
  -- The incident a Measurement released this Record under (#291) — NULL unless the Record
  -- came out of a buffered pre-roll window or the post-roll that follows it. A column, not a
  -- payload key, so `WHERE incident_id = ...` is a real query; scripts/run_incident.sh is
  -- what proves that end to end.
  incident_id text,
  -- memory
  used double precision,
  -- os
  os text,
  kernel text,
  cpus integer,
  memory double precision,
  -- storage
  free_percent double precision,
  free bigint,
  capacity bigint,
  -- uptime
  time double precision,
  -- tcp_health
  port integer,
  host text,
  server_name text,
  active boolean,
  -- dummy
  message text,
  -- synth00..synth13 (tools/e2e/scripts/workload_generator.py)
  value integer,
  source text
);

-- dc_files: file_status + group_complete Records from the Uploader (ADR-0005), same
-- column set proven against real dc_bridge behavior in
-- dc_bridge/dc_bridge_core/tests/uploader_tests.rs and dc_bridge/tests/end_to_end.rs.
CREATE TABLE IF NOT EXISTS dc_files (
  date bigint,
  kind text,
  group_name text,
  robot_name text,
  local_path text,
  remote_path text,
  storage_type text,
  uploaded boolean,
  on_filesystem boolean,
  deleted boolean,
  content_type text,
  size bigint,
  -- The File's derived preview (#256), when files.thumbnails is enabled — nullable, and
  -- always NULL with the feature off (its default) or for a File no preview can be
  -- derived from.
  thumbnail_path text,
  file_count integer,
  complete boolean,
  updated_at double precision
);
