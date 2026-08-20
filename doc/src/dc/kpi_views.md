# KPI views

DC collects Records; the numbers an operations team reports on are computed from those
Records in SQL, on the database, never on the robot. The window is a query parameter, so
changing a KPI definition is re-applying one file — not redeploying a fleet.

The set covers **availability and uptime**, **utilisation**, **intervention rate**,
**MTBF/MTTR** and **loop closure rate**. It lives in `tools/infrastructure/sql/kpi_views.sql`
and reads the `dc` table the [PostgreSQL](./infrastructure_setup/postgresql.md) Destination
writes into.

## What it defines

| Object                                                | Kind     | Reports                                                                                                                                             |
| ----------------------------------------------------- | -------- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| `dc_kpi_uptime_samples`                               | view     | One row per uptime Record: `robot_name`, `sample_time`, `uptime_seconds`, `run_id`, `prev_sample_time`                                                 |
| `dc_kpi_availability(from, to [, max_gap])`           | function | Per robot: `samples`, `first_sample`, `last_sample`, `uptime_seconds`, `covered_seconds`, `window_seconds`, `availability`                             |
| `dc_kpi_availability_5m`                              | view     | The same metric in 5-minute buckets, for charting — filter on `bucket_start`                                                                          |
| `dc_kpi_driving_samples`                              | view     | One row per driving_type Record: `robot_name`, `sample_time`, `mode`, `prev_sample_time`, and the `speed` last reported at or before it                |
| `dc_kpi_utilisation(from, to [, max_gap [, min_speed]])` | function | Per robot: `samples`, `speed_samples`, `reported_seconds`, `autonomous_seconds`, `manual_seconds`, `teleop_seconds`, `unknown_seconds`, `productive_seconds`, `window_seconds`, `utilisation` |
| `dc_kpi_utilisation_5m`                               | view     | Utilisation in 5-minute buckets                                                                                                                       |
| `dc_kpi_intervention_events`                          | view     | One row per intervention Record, with `is_start`/`is_end` derived from the modes it names                                                              |
| `dc_kpi_intervention_rate(from, to [, max_gap])`      | function | Per robot: `interventions`, `open_interventions`, `ended_interventions`, `autonomous_seconds`, `distance_km`, `per_autonomous_hour`, `per_km`, `mean_intervention_seconds`, `total_intervention_seconds` |
| `dc_kpi_interventions_1h`                             | view     | Interventions in 1-hour buckets                                                                                                                       |
| `dc_kpi_fault_events`                                 | view     | One row per fault Record: `component`, `from_level`, `to_level`, `previous_duration`, `sequence`, `reason`, `open`                                     |
| `dc_kpi_reliability(from, to [, failure_levels])`     | function | Per component: `failures`, `repairs`, `open_faults`, `operating_seconds`, `downtime_seconds`, `mtbf_seconds`, `mttr_seconds`                           |
| `dc_kpi_faults_1h`                                    | view     | Failures, repairs and downtime in 1-hour buckets                                                                                                      |
| `dc_kpi_loop_closure_events`                          | view     | One row per slam_toolbox_quality `loop_closure` Record ([#394])                                                                                        |
| `dc_kpi_loop_closure_rate(from, to)`                  | function | Per robot: `loop_closures`, `window_seconds`, `per_hour`, `last_loop_closure`, `seconds_since_last`                                                     |
| `dc_kpi_loop_closures_1h`                             | view     | Loop closures in 1-hour buckets                                                                                                                       |
| `dc_kpi_max_gap()`                                    | function | The grace period every definition above shares (30 s)                                                                                                 |
| `dc_kpi_min_speed()`                                  | function | The speed at or above which the robot counts as moving (0.05 m/s)                                                                                     |
| `dc_kpi_failure_levels()`                             | function | The diagnostic levels that count as broken (`ERROR`, `STALE`)                                                                                         |

```sql
-- Every metric for the last 24 hours, per robot.
SELECT * FROM dc_kpi_availability(now() - INTERVAL '24 hours', now());
SELECT * FROM dc_kpi_utilisation(now() - INTERVAL '24 hours', now());
SELECT * FROM dc_kpi_intervention_rate(now() - INTERVAL '24 hours', now());
SELECT * FROM dc_kpi_reliability(now() - INTERVAL '24 hours', now());

-- Any of them as a chart series.
SELECT bucket_start, robot_name, availability FROM dc_kpi_availability_5m
WHERE bucket_start >= now() - INTERVAL '24 hours' ORDER BY 1;
```

## Availability and uptime

Reads the [uptime Measurement](./measurements/uptime.md). Each uptime Record vouches for
the time back to the previous one, capped at the grace period and clipped to the window. A
robot polling uptime every 5 s therefore reports 100 % availability, and only a silence
longer than the grace period costs anything: a 5-minute outage in a 15-minute window leaves
630 of 900 seconds covered, so 70 %.

Consequences worth knowing before you put the number on a report:

- Availability is *reported* availability. It answers "was the robot up and shipping data",
  which is what an operations team can act on — not "was the robot doing useful work".
- A window longer than DC has been running reads low, because the time before the first
  Record is genuinely unreported.
- A robot whose Records stopped before the window starts drops out of the result entirely
  rather than reporting 0 %: nothing in the database distinguishes it from a robot that was
  never deployed. A fleet registry is what would fix that, and DC has none.

## Utilisation

Reads the [driving_type](./measurements/driving_type.md) and [speed](./measurements/speed.md)
Measurements. Time is credited exactly the way availability credits it — back to the
previous driving_type Record, capped at the grace period, clipped to the window — and
attributed to the mode of the Record that closes the interval. Of that reported time, the
share spent **moving under a known mode** is productive:

```text
utilisation = productive_seconds / reported_seconds
```

`manual` and `teleop` count as productive: a human driving the robot is still the robot
being used. Only `unknown` — no command source has published within `velocity_timeout_s`,
or no mode has ever been observed — does not.

What it deliberately does not claim:

- **Not "useful work".** A robot standing still while it inspects something reads as idle,
  and a robot driving in circles reads as productive. Distinguishing the two needs the
  Mission Measurement, which does not exist yet (see below).
- **No speed Record in the window means `utilisation` is NULL, not 0 %.** A deployment that
  collects `driving_type` but not `speed` has unreported movement, not zero movement.
  `speed_samples` is in the output so a NULL can be told from an empty range.
- **A speed Record vouches for the grace period after it, no longer.** Movement reported
  once and then never again buys 30 seconds of productive time, not the rest of the window.
- The per-mode seconds add up to `reported_seconds`, so the ratio can always be checked
  against its parts.

## Intervention rate

Reads the `intervention` Measurement ([#362]) for the numerator and `driving_type` plus
[distance_traveled](./measurements/distance_traveled.md) for the denominators. An
intervention starts when a Record leaves `autonomous` for `manual` or `teleop`, and ends
when one returns to `autonomous`; the end Record's `previous_duration` is the takeover's own
length.

```text
per_autonomous_hour = interventions / (autonomous_seconds / 3600)
per_km              = interventions / distance_km
```

Two denominators rather than one because they fail differently: a robot that spends a shift
parked has few autonomous hours and no kilometres, and a robot doing tight manoeuvring has
plenty of hours and few kilometres. Reporting both makes the difference visible instead of
picking a winner.

What it deliberately does not claim:

- **A denominator nothing reported gives a NULL rate**, never a rate over zero. A robot with
  no `driving_type` Records has `autonomous_seconds` NULL and `per_autonomous_hour` NULL;
  the same holds for `distance_traveled` and `per_km`.
- **A takeover still running is counted but never timed.** Only an end Record carries a
  duration, so an open interval is absent from `mean_intervention_seconds` and
  `total_intervention_seconds` rather than a zero in them. `open_interventions` counts the
  Records the Measurement flagged open.
- **A takeover that starts before the window or ends after it is counted in the window its
  Record falls in.** The window selects events, not intervals.

## MTBF and MTTR

Reads the `fault` Measurement ([#365]). A **raise** is a Record entering a failure level
from a healthy one; a **clear** is one leaving a failure level for a healthy one. Both carry
`previous_duration` — how long the level just left was held — which is what the two averages
are averages of:

```text
mtbf_seconds = mean healthy time preceding a raise
mttr_seconds = mean time in a failure level preceding a clear
```

Failure levels default to `ERROR` and `STALE` and are a query parameter:
`dc_kpi_reliability(from, to, ARRAY['ERROR'])` treats a silent component as reportable but
not broken. `WARN` is not a failure by default, so `OK → WARN → ERROR` is one failure, timed
from the last healthy state.

What it deliberately does not claim:

- **A change inside the failure set is not a second failure.** `ERROR → STALE` is neither a
  raise nor a clear: a component that goes quiet while already broken is still one fault.
- **A fault never cleared has no repair time.** It counts in `failures` and `open_faults`,
  and is absent from `mttr_seconds` rather than averaged in as an instant repair.
- **Grouped by component, never rolled up per robot.** "The robot is down" is a policy over
  components that nothing here knows, and averaging independent components together reports
  a number for a failure mode no component has.
- **The durations come from the Records, so they can reach outside the window.** A failure
  after eight healthy hours reports eight hours of MTBF in a one-hour window; the window
  chooses which events count, not how long their intervals were.

## Loop closure rate

Reads the `slam_toolbox_quality` Measurement ([#394]). `slam_toolbox/LoopClosureEvent` carries
nothing but its own occurrence, so the KPI value is entirely in Record timing:

```text
per_hour = loop_closures / (window_seconds / 3600)
```

alongside `seconds_since_last` — how long it's been since the most recent loop closure as of
`window_end`, a localization-drift risk indicator distinct from the rate: a robot can have a
healthy rate over a long window and still be mid-drift right now if its last correction was a
while ago.

What it deliberately does not claim:

- **`seconds_since_last` looks past `window_start`.** Clipped to the window it would read a
  robot with no loop closure yet in a short window as "just corrected" instead of "never
  corrected" — the other rate functions clip strictly to the window, this one deliberately
  doesn't, for this one column.
- **A robot with no loop closure Record at all, ever, is absent from the result** — same as
  every other rate function here: nothing distinguishes "never ran slam_toolbox" from "ran it
  and never closed a loop" without a fleet registry.

## Mission success rate is not here yet

The fourth starter metric — completed, failed, cancelled and aborted missions as a share of
missions started — has no source and no agreed contract. ROS has no standard mission
lifecycle interface, so what a Mission Measurement consumes is a design decision still open
in [#305]: how far a nav2 adapter infers, what the escape-hatch message in `dc_interfaces`
looks like, and how a mission still running at shutdown is represented. A view written
before that decision would pin the contract from the reporting end, which is the wrong end.

## What the data has to look like

The views read the `dc` table's columns directly, so the robot's configuration has to fill
them:

- every Measurement the views read runs with `include_measurement_name: true` — `name` is
  what tells one Measurement's Records apart from another's;
- `robot_name` is set, through `custom_keys_str` — Records without it are grouped under
  `unknown`;
- the `postgres` Destination writes to the `dc` table with `time_key: "date"`, whose default
  format is epoch nanoseconds.

| Metric              | Measurements it needs                                    |
| ------------------- | -------------------------------------------------------- |
| Availability        | `uptime`                                                  |
| Utilisation         | `driving_type`, `speed`                                   |
| Intervention rate   | `intervention` ([#362]), `driving_type`, `distance_traveled` |
| MTBF / MTTR         | `fault` ([#365])                                          |
| Loop closure rate   | `slam_toolbox_quality` ([#394])                            |

`dc_demos/params/tb3_simulation_pgsql_minio.yaml` is a working example of everything that
exists today. The `intervention` and `fault` Measurements do not, so their views return no
rows until they land — the columns they write into are already in
`tools/infrastructure/docker/config/postgresql/init.sql`, since Vector's `postgres` sink
maps a Record's keys onto existing columns and silently drops the rest.

## In the demo

`tools/infrastructure/docker/docker-compose.postgresql.yaml` mounts the tables and the KPI
definitions into PostgreSQL's init directory, so
[bringing PostgreSQL up](./infrastructure_setup/postgresql.md) applies both. Grafana's
provisioning ships a **ROS 2 Data Collection - KPI** dashboard reading them. Start the demo,
open [http://localhost:3000](http://localhost:3000) (admin/admin), and the availability and
utilisation panels populate as Records arrive; nothing has to be imported by hand.

The demo runs `driving_type` off Nav2's `/cmd_vel`, which is the only command source the
simulation has: the mode is `autonomous` while Nav2 publishes and `unknown` otherwise. A
real deployment adds its teleop topic to `velocity_topics` — that is also what makes an
intervention distinguishable from an idle robot.

```admonish warning
PostgreSQL runs its init directory only on an empty data directory. A database container
that already existed before this file did needs the views applied explicitly — that is what
the script below is for.
```

## Pointing the views at a real deployment

The definitions have nothing demo-specific in them. Apply them to any PostgreSQL that holds
DC Records:

```bash
PGPASSWORD=... ./tools/infrastructure/scripts/apply_kpi_views.bash \
  --host=db.example.com \
  --port=5432 \
  --user=dc \
  --database=dc
```

The script uses `psql` when it is installed and a `postgres` container otherwise. Every
object is `CREATE OR REPLACE`, so re-running it is how a changed definition ships — no robot
is touched, and the next dashboard refresh picks up the new numbers.

Then, on the reporting side:

- **Grafana**: point the PostgreSQL datasource at the deployment
  (`tools/infrastructure/docker/config/grafana/provisioning/grafana-datasource.yml`, uid
  `dc_postgres`) and the dashboard follows, since every panel queries by name. Grafana that
  isn't provisioned from this repo can import
  `tools/infrastructure/docker/config/grafana/dashboards/kpi.json` instead.
- **A read-only dashboard user** needs no more than:

  ```sql
  GRANT SELECT ON dc_kpi_uptime_samples, dc_kpi_availability_5m, dc_kpi_driving_samples,
                  dc_kpi_utilisation_5m, dc_kpi_intervention_events, dc_kpi_interventions_1h,
                  dc_kpi_fault_events, dc_kpi_faults_1h, dc_kpi_loop_closure_events,
                  dc_kpi_loop_closures_1h TO grafana;
  GRANT EXECUTE ON FUNCTION dc_kpi_availability(timestamptz, timestamptz, interval),
                            dc_kpi_utilisation(timestamptz, timestamptz, interval, double precision),
                            dc_kpi_intervention_rate(timestamptz, timestamptz, interval),
                            dc_kpi_reliability(timestamptz, timestamptz, text[]),
                            dc_kpi_loop_closure_rate(timestamptz, timestamptz) TO grafana;
  ```

- **Records stored elsewhere** — another schema, another table name — only affect the four
  sample/event views: they are the only objects that name the `dc` table, and everything
  else is defined on top of them.

## Testing a change

The fixture test seeds PostgreSQL with a known Record sequence and asserts what the
definitions report over it, including the outage, open-interval and missing-denominator
cases that are awkward to reproduce by hand:

```bash
./tools/infrastructure/scripts/test_kpi_views.sh
```

It brings up a throwaway PostgreSQL with Podman, applies these same SQL files into a schema
of its own, and runs `tools/infrastructure/test`. The intervention and fault cases seed the
Records their Measurements will emit, so a definition change breaks a test here rather than
a dashboard later. Add a case for every definition you add.

[#305]: https://github.com/Minipada/ros2_data_collection/issues/305
[#362]: https://github.com/Minipada/ros2_data_collection/issues/362
[#365]: https://github.com/Minipada/ros2_data_collection/issues/365
[#394]: https://github.com/Minipada/ros2_data_collection/issues/394
