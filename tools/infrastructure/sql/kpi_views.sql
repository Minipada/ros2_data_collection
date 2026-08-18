-- SPDX-FileCopyrightText: 2022-2026 David Bensoussan
--
-- SPDX-License-Identifier: MPL-2.0

-- Starter KPI definitions over the `dc` table created by
-- tools/infrastructure/docker/config/postgresql/init.sql. Availability and uptime are
-- computed here, in SQL, never on the robot: the time window is a query parameter, so
-- changing a definition is re-applying this file rather than redeploying a fleet.
--
-- Every object is CREATE OR REPLACE: re-applying to a live database is safe.
-- See doc/src/dc/kpi_views.md for how to point these at a real deployment.

-- How long DC may stay silent before the robot counts as down. One uptime Record vouches
-- for at most this much time. The demos poll uptime every 5 s.
CREATE OR REPLACE FUNCTION dc_kpi_max_gap() RETURNS interval
  LANGUAGE sql IMMUTABLE AS $$ SELECT INTERVAL '30 seconds' $$;

-- The wide `dc` table narrowed to the uptime Records the KPIs read, one row per Record.
-- `name` is only set when the Measurement runs with include_measurement_name: true.
CREATE OR REPLACE VIEW dc_kpi_uptime_samples AS
SELECT
  COALESCE(d.robot_name, 'unknown') AS robot_name,
  to_timestamp(d.date / 1e9) AS sample_time,
  d."time" AS uptime_seconds,
  d.run_id,
  lag(to_timestamp(d.date / 1e9)) OVER (
    PARTITION BY COALESCE(d.robot_name, 'unknown') ORDER BY d.date
  ) AS prev_sample_time
FROM dc d
WHERE d.name = 'uptime'
  AND d.date IS NOT NULL
  AND d."time" IS NOT NULL;

-- Availability and uptime per robot over an arbitrary window. A sample vouches for the
-- time back to the previous one, capped at the grace period and clipped to the window, so
-- a silence longer than the grace period is the only thing that costs availability.
CREATE OR REPLACE FUNCTION dc_kpi_availability(
  window_start timestamptz,
  window_end timestamptz,
  max_gap interval DEFAULT dc_kpi_max_gap()
)
RETURNS TABLE (
  robot_name text,
  samples bigint,
  first_sample timestamptz,
  last_sample timestamptz,
  uptime_seconds double precision,
  covered_seconds double precision,
  window_seconds double precision,
  availability double precision
)
LANGUAGE sql
STABLE
AS $$
  WITH credited AS (
    SELECT
      s.robot_name,
      s.sample_time,
      s.uptime_seconds,
      GREATEST(
        EXTRACT(EPOCH FROM (
          s.sample_time - GREATEST(
            COALESCE(s.prev_sample_time, s.sample_time - max_gap),
            s.sample_time - max_gap,
            window_start
          )
        )),
        0
      )::double precision AS covered_seconds
    FROM dc_kpi_uptime_samples s
    -- One grace period of history so the first in-window sample knows its predecessor;
    -- such a sample credits nothing itself, since its coverage clips to the window.
    WHERE s.sample_time > window_start - max_gap
      AND s.sample_time <= window_end
  )
  SELECT
    c.robot_name,
    count(*) FILTER (WHERE c.sample_time >= window_start),
    min(c.sample_time) FILTER (WHERE c.sample_time >= window_start),
    max(c.sample_time) FILTER (WHERE c.sample_time >= window_start),
    (array_agg(c.uptime_seconds ORDER BY c.sample_time DESC))[1],
    sum(c.covered_seconds)::double precision,
    EXTRACT(EPOCH FROM (window_end - window_start))::double precision,
    LEAST(
      sum(c.covered_seconds) / NULLIF(EXTRACT(EPOCH FROM (window_end - window_start)), 0),
      1
    )::double precision
  FROM credited c
  GROUP BY c.robot_name
$$;

-- The same metric bucketed for charting: filter on bucket_start for any window
-- ($__timeFilter(bucket_start) in Grafana). Coverage straddling a bucket boundary is
-- credited to the later bucket, which a 5-minute bucket makes negligible.
CREATE OR REPLACE VIEW dc_kpi_availability_5m AS
SELECT
  to_timestamp(floor(EXTRACT(EPOCH FROM s.sample_time) / 300) * 300) AS bucket_start,
  s.robot_name,
  count(*)::bigint AS samples,
  (array_agg(s.uptime_seconds ORDER BY s.sample_time DESC))[1] AS uptime_seconds,
  LEAST(
    sum(
      LEAST(
        COALESCE(EXTRACT(EPOCH FROM (s.sample_time - s.prev_sample_time)), 0),
        EXTRACT(EPOCH FROM dc_kpi_max_gap())
      )
    ) / 300,
    1
  )::double precision AS availability
FROM dc_kpi_uptime_samples s
GROUP BY 1, 2;

-- The lookup every object above starts from.
CREATE INDEX IF NOT EXISTS dc_name_date_idx ON dc (name, date);
