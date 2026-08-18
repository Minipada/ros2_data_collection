# KPI views

DC collects Records; the numbers an operations team reports on are computed from those
Records in SQL, on the database, never on the robot. The window is a query parameter, so
changing a KPI definition is re-applying one file — not redeploying a fleet.

The starter set covers **availability** and **uptime**, from the
[uptime Measurement](./measurements/uptime.md) alone. It lives in
`tools/infrastructure/sql/kpi_views.sql` and reads the `dc` table the
[PostgreSQL](./infrastructure_setup/postgresql.md) Destination writes into.

## What it defines

| Object                                        | Kind     | Reports                                                                                          |
| --------------------------------------------- | -------- | ------------------------------------------------------------------------------------------------ |
| `dc_kpi_uptime_samples`                       | view     | One row per uptime Record: `robot_name`, `sample_time`, `uptime_seconds`, `run_id`, `prev_sample_time` |
| `dc_kpi_availability(from, to [, max_gap])`   | function | Per robot over an arbitrary window: `samples`, `first_sample`, `last_sample`, `uptime_seconds`, `covered_seconds`, `window_seconds`, `availability` |
| `dc_kpi_availability_5m`                      | view     | The same metric in 5-minute buckets, for charting — filter on `bucket_start`                      |
| `dc_kpi_max_gap()`                            | function | The grace period every definition above shares (30 s)                                             |

```sql
-- Availability and uptime for the last 24 hours, per robot.
SELECT * FROM dc_kpi_availability(now() - INTERVAL '24 hours', now());

-- The same metric as a chart series.
SELECT bucket_start, robot_name, availability FROM dc_kpi_availability_5m
WHERE bucket_start >= now() - INTERVAL '24 hours' ORDER BY 1;
```

## How availability is defined

Each uptime Record vouches for the time back to the previous one, capped at the grace
period and clipped to the window. A robot polling uptime every 5 s therefore reports 100 %
availability, and only a silence longer than the grace period costs anything: a 5-minute
outage in a 15-minute window leaves 630 of 900 seconds covered, so 70 %.

Consequences worth knowing before you put the number on a report:

- Availability is *reported* availability. It answers "was the robot up and shipping data",
  which is what an operations team can act on — not "was the robot doing useful work".
- A window longer than DC has been running reads low, because the time before the first
  Record is genuinely unreported.
- A robot whose Records stopped before the window starts drops out of the result entirely
  rather than reporting 0 %: nothing in the database distinguishes it from a robot that was
  never deployed. A fleet registry is what would fix that, and DC has none.

## What the data has to look like

The views read the `dc` table's `name`, `robot_name`, `date` and `time` columns, so the
robot's configuration has to fill them:

- the `uptime` Measurement runs with `include_measurement_name: true` (`name` is what tells
  uptime Records apart from every other Measurement's);
- `robot_name` is set, through `custom_keys_str` — Records without it are grouped under
  `unknown`;
- the `postgres` Destination writes to the `dc` table with `time_key: "date"`, whose default
  format is epoch nanoseconds.

`dc_demos/params/tb3_simulation_pgsql_minio.yaml` is a working example of all three.

## In the demo

`tools/infrastructure/docker/docker-compose.postgresql.yaml` mounts the tables and the KPI
definitions into PostgreSQL's init directory, so
[bringing PostgreSQL up](./infrastructure_setup/postgresql.md) applies both. Grafana's
provisioning ships a **ROS 2 Data Collection - KPI** dashboard reading them — availability,
uptime, unreported time and Record counts over the dashboard's own time range. Start the
demo, open [http://localhost:3000](http://localhost:3000) (admin/admin), and the panels
populate as Records arrive; nothing has to be imported by hand.

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
  GRANT SELECT ON dc_kpi_uptime_samples, dc_kpi_availability_5m TO grafana;
  GRANT EXECUTE ON FUNCTION dc_kpi_availability(timestamptz, timestamptz, interval) TO grafana;
  ```

- **Records stored elsewhere** — another schema, another table name — only affect
  `dc_kpi_uptime_samples`: it is the single object that names the `dc` table, and everything
  else is defined on top of it.

## Testing a change

The fixture test seeds PostgreSQL with a known Record sequence and asserts what the
definitions report over it, including the outage cases that are awkward to reproduce by
hand:

```bash
./tools/infrastructure/scripts/test_kpi_views.sh
```

It brings up a throwaway PostgreSQL with Podman, applies these same SQL files into a schema
of its own, and runs `tools/infrastructure/test`. Add a case there for every definition you
add.
