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
  file_count integer,
  complete boolean,
  updated_at double precision
);

-- Idempotent writes (#309). The Shipper is at-least-once (ADR-0002): after an outage or
-- a restart, a Record that was in flight can be delivered a second time. It arrives
-- byte-identical — the Forwarder re-sends the frame it serialised the first time — so
-- (tag, date) identifies it exactly, now that `date` is nanosecond-precise (#308).
--
-- This is a trigger rather than a UNIQUE index on purpose. Vector's `postgres` sink has
-- no conflict handling (its config accepts only endpoint/table/pool_size/batch/request/
-- acknowledgements), and it writes a whole batch in one statement. A constraint violation
-- therefore fails the entire statement, which Vector classes as non-retriable and drops:
-- one re-delivered Record would take every Record batched with it down as well. Measured:
-- a batch of 4 containing 1 duplicate lost all 4. A BEFORE INSERT trigger returning NULL
-- skips just the offending row and lets the rest of the batch commit.
--
-- The index is deliberately NOT unique: the trigger is what enforces identity, the index
-- only makes its lookup cheap.
CREATE OR REPLACE FUNCTION dc_skip_duplicate() RETURNS trigger AS $$
DECLARE
  already_present boolean;
BEGIN
  EXECUTE format('SELECT EXISTS (SELECT 1 FROM %I WHERE tag = $1 AND date = $2)', TG_TABLE_NAME)
    INTO already_present USING NEW.tag, NEW.date;
  IF already_present THEN
    RETURN NULL;  -- skip this row; the statement (and the rest of the batch) continues
  END IF;
  RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE INDEX IF NOT EXISTS dc_records_identity ON dc_records (tag, date);

DROP TRIGGER IF EXISTS dc_records_skip_duplicate ON dc_records;
CREATE TRIGGER dc_records_skip_duplicate BEFORE INSERT ON dc_records
  FOR EACH ROW EXECUTE FUNCTION dc_skip_duplicate();
