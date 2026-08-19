-- SPDX-FileCopyrightText: 2022-2026 David Bensoussan
--
-- SPDX-License-Identifier: MPL-2.0

-- KPI definitions over the `dc` table created by
-- tools/infrastructure/docker/config/postgresql/init.sql. Every metric is computed
-- here, in SQL, never on the robot: the time window is a query parameter, so changing a
-- definition is re-applying this file rather than redeploying a fleet.
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

-- Speed at or above which the robot counts as moving. Below it a Record says the robot
-- was reporting, not that it was working.
CREATE OR REPLACE FUNCTION dc_kpi_min_speed() RETURNS double precision
  LANGUAGE sql IMMUTABLE AS $$ SELECT 0.05::double precision $$;

-- The wide `dc` table narrowed to driving_type Records, each carrying the speed the robot
-- last reported at or before it. NULL speed means no speed Record within the grace period,
-- which reads as "not moving" — dc_kpi_utilisation() counts those samples separately.
CREATE OR REPLACE VIEW dc_kpi_driving_samples AS
SELECT
  COALESCE(d.robot_name, 'unknown') AS robot_name,
  to_timestamp(d.date / 1e9) AS sample_time,
  d.mode,
  lag(to_timestamp(d.date / 1e9)) OVER (
    PARTITION BY COALESCE(d.robot_name, 'unknown') ORDER BY d.date
  ) AS prev_sample_time,
  (
    SELECT s.computed
    FROM dc s
    WHERE s.name = 'speed'
      AND s.computed IS NOT NULL
      AND COALESCE(s.robot_name, 'unknown') = COALESCE(d.robot_name, 'unknown')
      AND s.date <= d.date
      AND s.date > d.date - EXTRACT(EPOCH FROM dc_kpi_max_gap()) * 1e9
    ORDER BY s.date DESC
    LIMIT 1
  ) AS speed
FROM dc d
WHERE d.name = 'driving_type'
  AND d.date IS NOT NULL
  AND d.mode IS NOT NULL;

-- Utilisation per robot over an arbitrary window: time spent moving under a known driving
-- mode, over the time the robot reported a mode at all. Coverage is credited the same way
-- dc_kpi_availability() credits it — back to the previous sample, capped at the grace
-- period and clipped to the window — and attributed to the mode of the sample that closes
-- the interval. Per-mode seconds are broken out so the ratio can be checked against them;
-- autonomous_seconds is also dc_kpi_intervention_rate()'s denominator.
CREATE OR REPLACE FUNCTION dc_kpi_utilisation(
  window_start timestamptz,
  window_end timestamptz,
  max_gap interval DEFAULT dc_kpi_max_gap(),
  min_speed double precision DEFAULT dc_kpi_min_speed()
)
RETURNS TABLE (
  robot_name text,
  samples bigint,
  speed_samples bigint,
  reported_seconds double precision,
  autonomous_seconds double precision,
  manual_seconds double precision,
  teleop_seconds double precision,
  unknown_seconds double precision,
  productive_seconds double precision,
  window_seconds double precision,
  utilisation double precision
)
LANGUAGE sql
STABLE
AS $$
  WITH credited AS (
    SELECT
      s.robot_name,
      s.sample_time,
      s.mode,
      s.speed,
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
    FROM dc_kpi_driving_samples s
    -- One grace period of history, so the first in-window sample knows its predecessor.
    WHERE s.sample_time > window_start - max_gap
      AND s.sample_time <= window_end
  )
  SELECT
    c.robot_name,
    count(*) FILTER (WHERE c.sample_time >= window_start),
    count(*) FILTER (WHERE c.sample_time >= window_start AND c.speed IS NOT NULL),
    sum(c.covered_seconds)::double precision,
    COALESCE(sum(c.covered_seconds) FILTER (WHERE c.mode = 'autonomous'), 0)::double precision,
    COALESCE(sum(c.covered_seconds) FILTER (WHERE c.mode = 'manual'), 0)::double precision,
    COALESCE(sum(c.covered_seconds) FILTER (WHERE c.mode = 'teleop'), 0)::double precision,
    COALESCE(sum(c.covered_seconds) FILTER (WHERE c.mode = 'unknown'), 0)::double precision,
    COALESCE(
      sum(c.covered_seconds) FILTER (WHERE c.mode <> 'unknown' AND c.speed >= min_speed), 0
    )::double precision,
    EXTRACT(EPOCH FROM (window_end - window_start))::double precision,
    -- No speed Record in the window means the robot's movement is unreported, not zero:
    -- the ratio is NULL rather than a plausible-looking 0 %.
    CASE
      WHEN count(*) FILTER (WHERE c.sample_time >= window_start AND c.speed IS NOT NULL) = 0
        THEN NULL
      ELSE LEAST(
        COALESCE(
          sum(c.covered_seconds) FILTER (WHERE c.mode <> 'unknown' AND c.speed >= min_speed), 0
        ) / NULLIF(sum(c.covered_seconds), 0),
        1
      )
    END::double precision
  FROM credited c
  GROUP BY c.robot_name
$$;

-- The same metric bucketed for charting, on the grace period's own defaults.
CREATE OR REPLACE VIEW dc_kpi_utilisation_5m AS
SELECT
  to_timestamp(floor(EXTRACT(EPOCH FROM s.sample_time) / 300) * 300) AS bucket_start,
  s.robot_name,
  count(*)::bigint AS samples,
  sum(
    LEAST(
      COALESCE(EXTRACT(EPOCH FROM (s.sample_time - s.prev_sample_time)), 0),
      EXTRACT(EPOCH FROM dc_kpi_max_gap())
    )
  )::double precision AS reported_seconds,
  COALESCE(
    sum(
      LEAST(
        COALESCE(EXTRACT(EPOCH FROM (s.sample_time - s.prev_sample_time)), 0),
        EXTRACT(EPOCH FROM dc_kpi_max_gap())
      )
    ) FILTER (WHERE s.mode <> 'unknown' AND s.speed >= dc_kpi_min_speed()),
    0
  )::double precision AS productive_seconds,
  CASE
    WHEN count(*) FILTER (WHERE s.speed IS NOT NULL) = 0 THEN NULL
    ELSE LEAST(
      COALESCE(
        sum(
          LEAST(
            COALESCE(EXTRACT(EPOCH FROM (s.sample_time - s.prev_sample_time)), 0),
            EXTRACT(EPOCH FROM dc_kpi_max_gap())
          )
        ) FILTER (WHERE s.mode <> 'unknown' AND s.speed >= dc_kpi_min_speed()),
        0
      ) / NULLIF(
        sum(
          LEAST(
            COALESCE(EXTRACT(EPOCH FROM (s.sample_time - s.prev_sample_time)), 0),
            EXTRACT(EPOCH FROM dc_kpi_max_gap())
          )
        ),
        0
      ),
      1
    )
  END::double precision AS utilisation
FROM dc_kpi_driving_samples s
GROUP BY 1, 2;

-- The wide `dc` table narrowed to the intervention Measurement's transition Records (#362).
-- A start leaves `autonomous` for a human-driven mode; an end returns to it, and its
-- previous_duration is the takeover's own length. Only an end carries a length, so a
-- takeover that never ended is absent from the averages rather than a zero in them.
CREATE OR REPLACE VIEW dc_kpi_intervention_events AS
SELECT
  COALESCE(d.robot_name, 'unknown') AS robot_name,
  to_timestamp(d.date / 1e9) AS event_time,
  d.from_mode,
  d.to_mode,
  d.previous_duration,
  d."sequence",
  COALESCE(d."open", false) AS open,
  (d.from_mode = 'autonomous' AND d.to_mode IN ('manual', 'teleop')) AS is_start,
  (d.from_mode IN ('manual', 'teleop') AND d.to_mode = 'autonomous') AS is_end
FROM dc d
WHERE d.name = 'intervention'
  AND d.date IS NOT NULL
  AND d.from_mode IS NOT NULL
  AND d.to_mode IS NOT NULL;

-- Interventions per hour of autonomous operation and per kilometre travelled, per robot
-- over an arbitrary window. The two denominators come from Measurements of their own —
-- driving_type through dc_kpi_utilisation(), distance_traveled summed over the window —
-- so a denominator nothing reported gives a NULL rate, never a rate over zero.
CREATE OR REPLACE FUNCTION dc_kpi_intervention_rate(
  window_start timestamptz,
  window_end timestamptz,
  max_gap interval DEFAULT dc_kpi_max_gap()
)
RETURNS TABLE (
  robot_name text,
  interventions bigint,
  open_interventions bigint,
  ended_interventions bigint,
  autonomous_seconds double precision,
  distance_km double precision,
  per_autonomous_hour double precision,
  per_km double precision,
  mean_intervention_seconds double precision,
  total_intervention_seconds double precision
)
LANGUAGE sql
STABLE
AS $$
  WITH events AS (
    SELECT *
    FROM dc_kpi_intervention_events e
    WHERE e.event_time >= window_start AND e.event_time <= window_end
  ),
  counted AS (
    SELECT
      e.robot_name,
      count(*) FILTER (WHERE e.is_start)::bigint AS interventions,
      count(*) FILTER (WHERE e.open)::bigint AS open_interventions,
      count(*) FILTER (WHERE e.is_end)::bigint AS ended_interventions,
      avg(e.previous_duration) FILTER (WHERE e.is_end) AS mean_intervention_seconds,
      COALESCE(sum(e.previous_duration) FILTER (WHERE e.is_end), 0)::double precision
        AS total_intervention_seconds
    FROM events e
    GROUP BY e.robot_name
  ),
  driven AS (
    SELECT u.robot_name, u.autonomous_seconds
    FROM dc_kpi_utilisation(window_start, window_end, max_gap) u
  ),
  travelled AS (
    SELECT
      COALESCE(d.robot_name, 'unknown') AS robot_name,
      sum(d.distance_traveled) / 1000.0 AS distance_km
    FROM dc d
    WHERE d.name = 'distance_traveled'
      AND d.distance_traveled IS NOT NULL
      AND to_timestamp(d.date / 1e9) >= window_start
      AND to_timestamp(d.date / 1e9) <= window_end
    GROUP BY 1
  ),
  robots AS (
    SELECT counted.robot_name FROM counted
    UNION SELECT driven.robot_name FROM driven
    UNION SELECT travelled.robot_name FROM travelled
  )
  SELECT
    r.robot_name,
    COALESCE(c.interventions, 0),
    COALESCE(c.open_interventions, 0),
    COALESCE(c.ended_interventions, 0),
    d.autonomous_seconds,
    t.distance_km,
    (COALESCE(c.interventions, 0) / NULLIF(d.autonomous_seconds / 3600.0, 0))::double precision,
    (COALESCE(c.interventions, 0) / NULLIF(t.distance_km, 0))::double precision,
    c.mean_intervention_seconds::double precision,
    COALESCE(c.total_intervention_seconds, 0)::double precision
  FROM robots r
  LEFT JOIN counted c ON c.robot_name = r.robot_name
  LEFT JOIN driven d ON d.robot_name = r.robot_name
  LEFT JOIN travelled t ON t.robot_name = r.robot_name
$$;

-- Interventions bucketed for charting. An hour, not the 5 minutes availability uses: a
-- rate over a bucket most robots see no takeover in charts as noise.
CREATE OR REPLACE VIEW dc_kpi_interventions_1h AS
SELECT
  to_timestamp(floor(EXTRACT(EPOCH FROM e.event_time) / 3600) * 3600) AS bucket_start,
  e.robot_name,
  count(*) FILTER (WHERE e.is_start)::bigint AS interventions,
  count(*) FILTER (WHERE e.open)::bigint AS open_interventions,
  avg(e.previous_duration) FILTER (WHERE e.is_end)::double precision
    AS mean_intervention_seconds
FROM dc_kpi_intervention_events e
GROUP BY 1, 2;

-- The levels a component is broken at. STALE is a failure of its own — a silent component
-- and a broken one are different faults — and WARN is deliberately not one.
CREATE OR REPLACE FUNCTION dc_kpi_failure_levels() RETURNS text[]
  LANGUAGE sql IMMUTABLE AS $$ SELECT ARRAY['ERROR', 'STALE'] $$;

-- The wide `dc` table narrowed to the fault Measurement's transition Records (#365), one
-- row per level change of one component. previous_duration is how long the component held
-- the level it just left, which is what MTBF and MTTR are averages of.
CREATE OR REPLACE VIEW dc_kpi_fault_events AS
SELECT
  COALESCE(d.robot_name, 'unknown') AS robot_name,
  to_timestamp(d.date / 1e9) AS event_time,
  d.component,
  d.from_level,
  d.to_level,
  d.previous_duration,
  d."sequence",
  d.reason,
  COALESCE(d."open", false) AS open
FROM dc d
WHERE d.name = 'fault'
  AND d.date IS NOT NULL
  AND d.component IS NOT NULL
  AND d.to_level IS NOT NULL;

-- MTBF and MTTR per component over an arbitrary window. A raise enters a failure level
-- from a healthy one and carries the healthy time that preceded it; a clear leaves a
-- failure level for a healthy one and carries the repair. A change inside the failure set
-- (ERROR to STALE) is neither, so a fault that changes shape stays one fault.
--
-- Grouped by component, not rolled up per robot: "the robot is down" is a policy over
-- components that nothing here knows, and averaging independent components together
-- reports a number for a failure mode no component has.
CREATE OR REPLACE FUNCTION dc_kpi_reliability(
  window_start timestamptz,
  window_end timestamptz,
  failure_levels text[] DEFAULT dc_kpi_failure_levels()
)
RETURNS TABLE (
  robot_name text,
  component text,
  failures bigint,
  repairs bigint,
  open_faults bigint,
  operating_seconds double precision,
  downtime_seconds double precision,
  mtbf_seconds double precision,
  mttr_seconds double precision
)
LANGUAGE sql
STABLE
AS $$
  WITH events AS (
    SELECT
      e.*,
      (e.to_level = ANY(failure_levels) AND NOT (e.from_level = ANY(failure_levels)))
        AS is_raise,
      (e.from_level = ANY(failure_levels) AND NOT (e.to_level = ANY(failure_levels)))
        AS is_clear
    FROM dc_kpi_fault_events e
    WHERE e.event_time >= window_start
      AND e.event_time <= window_end
      AND e.from_level IS NOT NULL
  )
  SELECT
    e.robot_name,
    e.component,
    count(*) FILTER (WHERE e.is_raise)::bigint,
    count(*) FILTER (WHERE e.is_clear)::bigint,
    count(*) FILTER (WHERE e.open)::bigint,
    COALESCE(sum(e.previous_duration) FILTER (WHERE e.is_raise), 0)::double precision,
    COALESCE(sum(e.previous_duration) FILTER (WHERE e.is_clear), 0)::double precision,
    -- Averages over the events in the window, never over the window itself: a fault still
    -- open has no repair time yet and is left out of MTTR rather than counted as instant.
    avg(e.previous_duration) FILTER (WHERE e.is_raise)::double precision,
    avg(e.previous_duration) FILTER (WHERE e.is_clear)::double precision
  FROM events e
  GROUP BY e.robot_name, e.component
  HAVING count(*) FILTER (WHERE e.is_raise OR e.is_clear) > 0
$$;

-- Faults bucketed for charting, hourly for the same reason interventions are.
CREATE OR REPLACE VIEW dc_kpi_faults_1h AS
SELECT
  to_timestamp(floor(EXTRACT(EPOCH FROM e.event_time) / 3600) * 3600) AS bucket_start,
  e.robot_name,
  e.component,
  count(*) FILTER (
    WHERE e.to_level = ANY(dc_kpi_failure_levels())
      AND NOT (e.from_level = ANY(dc_kpi_failure_levels()))
  )::bigint AS failures,
  count(*) FILTER (
    WHERE e.from_level = ANY(dc_kpi_failure_levels())
      AND NOT (e.to_level = ANY(dc_kpi_failure_levels()))
  )::bigint AS repairs,
  COALESCE(
    sum(e.previous_duration) FILTER (
      WHERE e.from_level = ANY(dc_kpi_failure_levels())
        AND NOT (e.to_level = ANY(dc_kpi_failure_levels()))
    ),
    0
  )::double precision AS downtime_seconds
FROM dc_kpi_fault_events e
WHERE e.from_level IS NOT NULL
GROUP BY 1, 2, 3;

-- The wide `dc` table narrowed to the slam_toolbox_quality Measurement's single-shot
-- loop-closure Records (#394). The source topic carries nothing but its own occurrence, so
-- there is nothing more to project than when it happened.
CREATE OR REPLACE VIEW dc_kpi_loop_closure_events AS
SELECT
  COALESCE(d.robot_name, 'unknown') AS robot_name,
  to_timestamp(d.date / 1e9) AS event_time
FROM dc d
WHERE d.name = 'slam_toolbox_quality'
  AND d.event = 'loop_closure'
  AND d.date IS NOT NULL;

-- Loop closures per hour of the window, per robot, plus how long it's been since the last one
-- as of window_end -- a growing gap is a localization-drift risk indicator on its own, distinct
-- from the rate. last_loop_closure looks past window_start (unlike the interventions/faults
-- rate functions) because "how long since" is meaningless clipped to the window: a robot with no
-- loop closure yet in a short window would otherwise read as recently corrected instead of never.
CREATE OR REPLACE FUNCTION dc_kpi_loop_closure_rate(
  window_start timestamptz,
  window_end timestamptz
)
RETURNS TABLE (
  robot_name text,
  loop_closures bigint,
  window_seconds double precision,
  per_hour double precision,
  last_loop_closure timestamptz,
  seconds_since_last double precision
)
LANGUAGE sql
STABLE
AS $$
  WITH counted AS (
    SELECT e.robot_name, count(*)::bigint AS loop_closures
    FROM dc_kpi_loop_closure_events e
    WHERE e.event_time >= window_start AND e.event_time <= window_end
    GROUP BY e.robot_name
  ),
  last_seen AS (
    SELECT e.robot_name, max(e.event_time) AS last_loop_closure
    FROM dc_kpi_loop_closure_events e
    WHERE e.event_time <= window_end
    GROUP BY e.robot_name
  ),
  robots AS (
    SELECT counted.robot_name FROM counted
    UNION SELECT last_seen.robot_name FROM last_seen
  )
  SELECT
    r.robot_name,
    COALESCE(c.loop_closures, 0),
    EXTRACT(EPOCH FROM (window_end - window_start))::double precision,
    (
      COALESCE(c.loop_closures, 0) /
      NULLIF(EXTRACT(EPOCH FROM (window_end - window_start)) / 3600.0, 0)
    )::double precision,
    l.last_loop_closure,
    EXTRACT(EPOCH FROM (window_end - l.last_loop_closure))::double precision
  FROM robots r
  LEFT JOIN counted c ON c.robot_name = r.robot_name
  LEFT JOIN last_seen l ON l.robot_name = r.robot_name
$$;

-- Loop closures bucketed hourly for charting, the same bucket size interventions and faults use.
CREATE OR REPLACE VIEW dc_kpi_loop_closures_1h AS
SELECT
  to_timestamp(floor(EXTRACT(EPOCH FROM e.event_time) / 3600) * 3600) AS bucket_start,
  e.robot_name,
  count(*)::bigint AS loop_closures
FROM dc_kpi_loop_closure_events e
GROUP BY 1, 2;

-- The lookup every object above starts from.
CREATE INDEX IF NOT EXISTS dc_name_date_idx ON dc (name, date);
