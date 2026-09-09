#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Hard-failing zero-loss verification for the E2E harness (#249).

Queries Postgres through `podman exec` on the Postgres container (no host-side
psycopg2/psql dependency — only podman itself, already required). Every check is a real
assertion: a query that can't run, or a table that doesn't exist, is a FAIL, not a
skip — matching the #246 follow-up decision that a verification which silently didn't
execute must never look identical to one that passed.

Delivery model — why loss fails but a duplicate doesn't. The shipper (Vector) buffers
to disk and replays on recovery with end-to-end acknowledgements: that's *at-least-once*
(ADR-0002). It guarantees nothing is ever lost across the outage, but a record that was
in-flight to Postgres when the outage hit — committed, but with its ack lost — is
re-sent on recovery and lands twice. Deduping on *write* (a UNIQUE index + per-insert
skip trigger) would tax every insert forever to erase a rare boundary re-send; the
standard, cheap answer for an at-least-once pipeline is to dedupe on *read* — collapse
exact re-sends on the natural key (a one-line DISTINCT) at query time. So:

  - LOSS is a hard failure: a value the pipeline accepted must come out (zero-loss).
  - A boundary DUPLICATE is expected and reported as a note, not a failure. The checks
    below assert the *deduplicated* data is exactly-once — every record present once
    after collapsing exact re-sends — which is exactly-once-on-read.

Checks:

1. Per synth topic: every value the generator recorded in its ledger
   (tools/e2e/scripts/workload_generator.py, on the dc_e2e_data volume) is present in
   Postgres. The ledger is written independently of the pipeline, so this compares what
   was *sent* against what *arrived*, one to one. It replaces `expected = max(value) + 1`,
   which read its expectation out of the very table it was checking and therefore could
   not notice a missing tail: lose the last 40 of 100 and `max` drops to 59, the bar drops
   with it, and the check passed (#312).
2. Per real-measurement Tag (memory/os/storage/uptime/tcp_health/dummy): at least one
   Record landed; duplicate (tag, date) rows are reported as notes.
3. File pipeline (ADR-0005): at least one verified file_status row and one
   group_complete marker for the camera group; duplicate "uploaded" status rows for the
   same local_path are reported as notes (the Uploader's own idempotency guarantee,
   ADR-0005/#248, holding up under this harness's induced outage + restart).
4. Passthrough (ADR-0003, #131 follow-up): the raw Vector sink from
   params/e2e_passthrough_sink.toml — a sink dc_bridge knows nothing about, wired to the
   public `dc.<tag>` routes — received the same synth counters with no gaps, and received
   *only* the Tags it subscribed to (proving the per-Tag route branches actually
   discriminate rather than fanning everything out).
5. Raw / generic-subscription mode (#227): `/dc/e2e/synth/synth00` carries
   `dc_interfaces/msg/StringStamped`, a custom non-std_msgs type dc_bridge was never
   compiled against — the `raw:` block subscribes to it generically and ships it to its
   own `file` Destination under the `dc.raw.` Tag namespace. Asserts the Records
   arrived, decoded field-for-field (both strings plus the nested Header/Time), carry no
   Tag from outside the namespace, and have no gaps inside the window raw mode was
   subscribed for.
6. MCAP passthrough (ADR-0009, #210): a second, differently-shaped passthrough — a
   Vector `socket` sink (params/e2e_mcap_sink.toml) streaming synth02/synth03 to the
   standalone `dc_mcap_writer` process — held to the same zero-loss/only-subscribed-Tags
   standard as the NDJSON passthrough above, via a JSON summary of its `.mcap` capture
   (scripts/mcap_summary.py, which needs the `mcap` library and so runs inside the DC
   image via run.sh, not on the CI host runner).

Profiles (#496). `--profile` selects which set of checks runs, so a scenario's Record
assertions all live here instead of as inline SQL in its bash script:

  - `zero-loss` (the default): the checks above, as run.sh and its siblings invoke them.
  - `incident`: run_incident.sh's assertions (#291) — that a Measurement armed with
    `buffer_duration_sec` ships nothing while a live one flows, and that after one
    FlushEvent the released window is queryable by `incident_id` column and carries no
    other Measurement's rows. Two stages, because a FlushEvent is published between
    them: `--stage armed`, then `--stage released`.
  - `retention`: run_retention.sh's assertions (#267) — that a File is shed without upload
    once the pool exceeds `files.retention.max_bytes` against a down store, and that a
    later File uploads normally once the store is back. One stage per half, `--stage
    shed` then `--stage uploaded`; the shed File's absence *on disk* stays in bash, which
    is where the volume mount is.

Every check is still a hard assertion, in every profile: a query that can't run is a
FAIL, not a skip.
"""

import argparse
import json
import os
import subprocess
import sys
import time

import raw_volume

# Records lost at a point where the harness kills the workload generator — the mid-run
# container restart, and the stop that ends the run. The generator publishes from inside
# that container, so a tick in flight when it dies was never handed to measurement_server
# and no buffer could have held it. Bounded and reported, never a blanket excuse: a gap
# anywhere other than a kill point is loss and fails.
MAX_KILL_GAP = 3

REAL_TAGS = [
    "dc.measurement.memory",
    "dc.measurement.os",
    "dc.measurement.storage",
    "dc.measurement.uptime",
    "dc.measurement.tcp_health",
    "dc.measurement.dummy",
]

# The synth topics params/e2e_passthrough_sink.toml routes into the passthrough sink, and
# the Tags they arrive under. Kept in lockstep with that file's `inputs`.
PASSTHROUGH_SOURCES = ["synth00", "synth01"]
PASSTHROUGH_TAGS = {f"dc.measurement.{s}" for s in PASSTHROUGH_SOURCES}

# Same idea for params/e2e_mcap_sink.toml (ADR-0009, #210) — a distinct pair of synth
# topics from the NDJSON passthrough above, so the two checks stay independent.
MCAP_SOURCES = ["synth02", "synth03"]
MCAP_TAGS = {f"dc.measurement.{s}" for s in MCAP_SOURCES}

# Raw / generic-subscription mode (#227): the `raw:` block in params/e2e_params.yaml
# subscribes to the generator's own source topic and ships it under this Tag.
RAW_SOURCE = "synth00"
RAW_TAG = "dc.raw.dc.e2e.synth.synth00"


def psql(pg_container: str, query: str) -> str:
    result = subprocess.run(
        ["podman", "exec", pg_container, "psql", "-U", "dc", "-d", "dc", "-tAc", query],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(f"psql query failed: {query!r}\n{result.stderr}")
    return result.stdout.strip()


def scalar_int(pg_container: str, query: str) -> int:
    out = psql(pg_container, query)
    return int(out) if out else 0


def read_ledger(path: str) -> dict:
    """Load the generator's published-Record ledger as {source: set(values)}.

    Args:
        path: newline-delimited "source,value" file extracted from the dc_e2e_data volume.

    Returns:
        dict: per-source set of published counter values.

    Raises:
        RuntimeError: the ledger is missing or empty — the generator never wrote it, so
            there is nothing to verify against and silence would look like success.
    """
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        raise RuntimeError(
            f"workload ledger missing or empty at {path} — without it there is no "
            "independent record of what was published, so loss cannot be checked"
        )
    published: dict = {}
    with open(path) as ledger:
        for line in ledger:
            source, _, value = line.strip().partition(",")
            if source and value.isdigit():
                published.setdefault(source, set()).add(int(value))
    return published


def read_boundaries(path: str) -> dict:
    """Load the kill-point markers the generator wrote when it resumed after a restart.

    Args:
        path: the same ledger file read by read_ledger.

    Returns:
        dict: {source: set(resume values)} — a gap just below one of these is the
            generator having been killed mid-tick, not the pipeline losing a Record.
    """
    boundaries: dict = {}
    with open(path) as ledger:
        for line in ledger:
            if not line.startswith("#boundary,"):
                continue
            _, _, rest = line.strip().partition(",")
            source, _, value = rest.partition(",")
            if source and value.isdigit():
                boundaries.setdefault(source, set()).add(int(value))
    return boundaries


def check_synth_topic(
    pg_container: str,
    name: str,
    published: set,
    boundaries: set,
    violations: list,
    notes: list,
    details: dict,
) -> None:
    """Compare what the generator published against what reached Postgres, one to one.

    Args:
        pg_container: name of the Postgres container to query via podman exec.
        name: synth topic name.
        published: values the generator recorded as published for this topic.
        boundaries: counter values the generator resumed at after being killed.
        violations: hard failures, appended to in place.
        notes: informational observations, appended to in place.
        details: counters for the JSON report, populated in place.
    """
    total = scalar_int(pg_container, f"SELECT count(*) FROM dc_records WHERE source='{name}'")
    rows = psql(pg_container, f"SELECT DISTINCT value FROM dc_records WHERE source='{name}'")
    arrived = {int(v) for v in rows.split() if v.strip().isdigit()}

    missing = published - arrived
    unexpected = arrived - published

    # Every point at which the generator was killed: each restart it resumed from, plus
    # the end of the run (the stop that ends the harness). Only the few Records
    # immediately below such a point may go missing without it being pipeline loss.
    kill_points = set(boundaries) | {max(published) + 1 if published else 0}
    tolerated = {
        value
        for value in missing
        if any(0 < point - value <= MAX_KILL_GAP for point in kill_points)
    }
    lost = missing - tolerated

    details[name] = {
        "published": len(published),
        "arrived": len(arrived),
        "rows": total,
        "lost": len(lost),
        "tolerated_at_kill_points": len(tolerated),
        "unexpected": len(unexpected),
    }

    if not published:
        violations.append(f"{name}: the ledger recorded nothing published")
        return
    if lost:
        violations.append(
            f"{name}: {len(lost)} published Record(s) never arrived, away from any kill "
            f"point — first missing: {sorted(lost)[:10]} — data loss"
        )
    if tolerated:
        notes.append(
            f"{name}: {len(tolerated)} Record(s) lost in flight where the generator was "
            f"killed ({sorted(tolerated)}) — the workload runs inside the restarted "
            "container, so nothing could persist them"
        )
    if unexpected:
        notes.append(
            f"{name}: {len(unexpected)} row(s) in Postgres with no ledger entry "
            "(generator killed between publish and ledger write)"
        )
    # A value now identifies exactly one Record for the whole run, so a repeat is a
    # genuine double delivery rather than the counter having restarted (#312).
    if total > len(arrived):
        notes.append(
            f"{name}: {total - len(arrived)} duplicate row(s) "
            "(at-least-once re-delivery; deduplicated on read)"
        )


def check_real_tag(
    pg_container: str, tag: str, violations: list, notes: list, details: dict
) -> None:
    total = scalar_int(pg_container, f"SELECT count(*) FROM dc_records WHERE tag='{tag}'")
    dup_count = scalar_int(
        pg_container,
        f"SELECT count(*) FROM (SELECT date FROM dc_records "
        f"WHERE tag='{tag}' GROUP BY date HAVING count(*) > 1) t",
    )
    details[tag] = {"total": total, "duplicate_dates": dup_count}
    # These Measurements carry no counter, so loss can't be checked by sequence; the
    # presence + the synth topics' zero-loss result stand in. A duplicate timestamp is
    # the expected at-least-once boundary re-send, deduped on read — a note, not a fail.
    if total == 0:
        violations.append(f"{tag}: zero Records landed in Postgres")
    if dup_count > 0:
        notes.append(
            f"{tag}: {dup_count} duplicate timestamp(s) "
            "(at-least-once outage-boundary re-send; deduped on read)"
        )


def check_files(pg_container: str, violations: list, notes: list, details: dict) -> None:
    status_count = scalar_int(
        pg_container,
        "SELECT count(*) FROM dc_files WHERE kind='file_status' AND group_name='camera'",
    )
    complete_count = scalar_int(
        pg_container,
        "SELECT count(*) FROM dc_files WHERE kind='group_complete' AND group_name='camera'",
    )
    dup_uploaded = scalar_int(
        pg_container,
        "SELECT count(*) FROM (SELECT local_path FROM dc_files "
        "WHERE kind='file_status' AND group_name='camera' AND uploaded=true "
        "GROUP BY local_path HAVING count(*) > 1) t",
    )
    details["files"] = {
        "file_status_rows": status_count,
        "group_complete_rows": complete_count,
        "duplicate_uploaded_rows": dup_uploaded,
    }
    if status_count == 0:
        violations.append(
            "files: zero file_status rows for the camera group — Uploader produced nothing"
        )
    if complete_count == 0:
        violations.append("files: zero group_complete markers for the camera group")
    if dup_uploaded > 0:
        notes.append(
            f"files: {dup_uploaded} local_path(s) have more than one 'uploaded' status row "
            "(at-least-once outage-boundary re-send; deduped on read)"
        )


# --- incident profile (#291) -------------------------------------------------------------

# "Armed means silent" is only meaningful against a pipeline known to be delivering: the
# live Measurement has to have shipped several Records first, enough that an unarmed one
# would have shipped several too.
MIN_LIVE_ROWS = 5

# A released *window* is several Records (the pre-event buffer plus the post-roll), so
# one row would not prove a window came out at all.
MIN_INCIDENT_ROWS = 2

INCIDENT_STAGE_ARMED = "armed"
INCIDENT_STAGE_RELEASED = "released"
INCIDENT_STAGES = (INCIDENT_STAGE_ARMED, INCIDENT_STAGE_RELEASED)

RETENTION_STAGE_SHED = "shed"
RETENTION_STAGE_UPLOADED = "uploaded"
RETENTION_STAGES = (RETENTION_STAGE_SHED, RETENTION_STAGE_UPLOADED)


def check_live_flowing(
    live_count: int,
    live_tag: str,
    timeout_s: int,
    violations: list,
    details: dict,
) -> None:
    """Assert the never-armed Measurement delivered enough to make silence meaningful."""
    details["live_measurement"] = {"tag": live_tag, "rows": live_count}
    if live_count < MIN_LIVE_ROWS:
        violations.append(
            f"incident: the live Measurement produced only {live_count} row(s) in "
            f"{timeout_s}s — the pipeline is not running"
        )


def check_armed_silent(
    buffered_count: int, buffered_tag: str, violations: list, details: dict
) -> None:
    """Assert a Measurement armed with buffer_duration_sec shipped nothing pre-event."""
    details["armed_measurement"] = {"tag": buffered_tag, "rows": buffered_count}
    if buffered_count != 0:
        violations.append(
            f"incident: the armed Measurement shipped {buffered_count} row(s) before any "
            "FlushEvent — it is not buffering"
        )


def check_released_window(
    pg_container: str,
    incident_id: str,
    buffered_tag: str,
    live_tag: str,
    incident_count: int,
    violations: list,
    details: dict,
) -> None:
    """Assert one FlushEvent's window is queryable by column, and by nothing else.

    Args:
        pg_container: name of the Postgres container to query via podman exec.
        incident_id: the id the FlushEvent carried.
        buffered_tag: Tag of the armed Measurement the window must have come from.
        live_tag: Tag of the never-armed Measurement, whose rows must stay NULL.
        incident_count: rows already known to match WHERE incident_id = <id>.
        violations: hard failures, appended to in place.
        details: counters for the JSON report, populated in place.
    """
    wrong_tag = scalar_int(
        pg_container,
        f"SELECT count(*) FROM dc_records WHERE incident_id = '{incident_id}' "
        f"AND tag <> '{buffered_tag}'",
    )
    other_ids = scalar_int(
        pg_container,
        f"SELECT count(*) FROM dc_records WHERE incident_id IS NOT NULL "
        f"AND incident_id <> '{incident_id}'",
    )
    live_tagged = scalar_int(
        pg_container,
        f"SELECT count(*) FROM dc_records WHERE tag = '{live_tag}' AND incident_id IS NOT NULL",
    )
    details["released_window"] = {
        "incident_id": incident_id,
        "rows": incident_count,
        "rows_from_another_measurement": wrong_tag,
        "rows_with_another_incident_id": other_ids,
        "live_measurement_rows_tagged": live_tagged,
    }

    if incident_count < MIN_INCIDENT_ROWS:
        violations.append(
            f"incident: only {incident_count} row(s) matched WHERE incident_id = "
            f"'{incident_id}' (a released *window* is several Records; 0 means the id "
            "never became a column value at all)"
        )
    if wrong_tag:
        violations.append(
            f"incident: {wrong_tag} row(s) of the window came from a Measurement other "
            f"than {buffered_tag}"
        )
    if other_ids:
        violations.append(
            f"incident: {other_ids} row(s) carry an incident_id other than the one the "
            "FlushEvent minted"
        )
    if live_tagged:
        violations.append(
            f"incident: {live_tagged} row(s) from the never-armed Measurement carry an "
            "incident_id — the id marks the incident, it is not stamped on everything"
        )


# --- retention profile (#267) -------------------------------------------------------------


def check_shed_row(shed_count: int, timeout_s: int, violations: list, details: dict) -> None:
    """Assert a File was shed without upload while the object store was unreachable."""
    details["shed"] = {"deleted_without_upload_rows": shed_count, "wait_seconds": timeout_s}
    if shed_count == 0:
        violations.append(
            f"retention: no shed-without-upload audit row (deleted=true, uploaded=false) "
            f"landed within {timeout_s}s"
        )


def check_uploaded_row(
    uploaded_count: int, timeout_s: int, violations: list, details: dict
) -> None:
    """Assert a File uploaded and verified normally once the store came back."""
    details["uploaded"] = {"uploaded_rows": uploaded_count, "wait_seconds": timeout_s}
    if uploaded_count == 0:
        violations.append(
            f"retention: no File uploaded/verified within {timeout_s}s of the store coming back"
        )


def wait_for_count(
    pg_container: str, query: str, minimum: int, timeout_s: int, poll_seconds: float = 2.0
) -> int:
    """Poll a scalar count until it reaches `minimum` or the deadline passes.

    The bounded waits the incident/retention scenario scripts used to carry inline, next
    to the verdicts they feed: the count is only ever *checked* after the wait, so a wait
    that expires returns the last observed count and the caller's check fails on it.

    Args:
        pg_container: name of the Postgres container to query via podman exec.
        query: scalar SQL statement to poll.
        minimum: count that ends the wait early.
        timeout_s: how long to keep polling.
        poll_seconds: interval between polls.

    Returns:
        int: the last count seen, whether or not it reached `minimum`.
    """
    deadline = time.monotonic() + timeout_s
    count = 0
    while True:
        count = scalar_int(pg_container, query)
        if count >= minimum:
            return count
        if time.monotonic() >= deadline:
            return count
        time.sleep(poll_seconds)


# The Measurement configured above 1 Hz in params/e2e_params.yaml. It is the only topic
# that puts several Records inside one wall-clock second, which is what gives the
# resolution check below something to actually collide on.
FAST_TAG = "dc.measurement.memory"


def check_timestamp_resolution(
    pg_container: str, tag: str, violations: list, notes: list, details: dict
) -> None:
    """Assert sub-second timestamps survive the pipeline (#308).

    `date` is epoch_nanos, an exact integer of nanoseconds. If the Bridge went back to
    sending whole seconds, every value would be an exact multiple of 1e9 and the Records
    of a 5 Hz Measurement would collapse onto shared timestamps.

    Args:
        pg_container: name of the Postgres container to query via podman exec.
        tag: Tag of the faster-than-1-Hz Measurement to check.
        violations: hard failures, appended to in place.
        notes: informational observations, appended to in place.
        details: counters for the JSON report, populated in place.
    """
    total = scalar_int(pg_container, f"SELECT count(*) FROM dc_records WHERE tag='{tag}'")
    distinct = scalar_int(
        pg_container, f"SELECT count(DISTINCT date) FROM dc_records WHERE tag='{tag}'"
    )
    sub_second = scalar_int(
        pg_container,
        f"SELECT count(*) FROM dc_records WHERE tag='{tag}' AND date % 1000000000 <> 0",
    )
    # Seconds holding more than one Record — proves the Measurement really ran above
    # 1 Hz, so "no two Records share a timestamp" is a real assertion, not a vacuous one.
    shared_seconds = scalar_int(
        pg_container,
        f"SELECT count(*) FROM (SELECT date / 1000000000 AS sec FROM dc_records "
        f"WHERE tag='{tag}' GROUP BY sec HAVING count(*) > 1) t",
    )
    details["timestamp_resolution"] = {
        "tag": tag,
        "total": total,
        "distinct_timestamps": distinct,
        "sub_second_timestamps": sub_second,
        "seconds_holding_multiple_records": shared_seconds,
    }

    if total == 0:
        violations.append(f"timestamps: zero Records for {tag} — cannot check resolution")
        return
    if sub_second == 0:
        violations.append(
            f"timestamps: every {tag} Record has a whole-second timestamp ({total} Records, "
            "none with a nanosecond remainder) — sub-second resolution was lost on the "
            "wire (#308)"
        )
    if shared_seconds == 0:
        violations.append(
            f"timestamps: no wall-clock second holds more than one {tag} Record, so this "
            "check proves nothing — is that Measurement still polling above 1 Hz?"
        )
    elif distinct < total:
        violations.append(
            f"timestamps: {total - distinct} {tag} Record(s) share a timestamp with another "
            f"({distinct} distinct out of {total}) — Records taken at different instants "
            "must be distinguishable in time"
        )


def check_passthrough(
    path: str,
    published: dict,
    boundaries: dict,
    violations: list,
    notes: list,
    details: dict,
) -> None:
    """Assert the ADR-0003 passthrough sink got the same zero-loss treatment.

    A missing or empty output file is a FAIL, never a skip: a passthrough that silently
    produced nothing must not look identical to one that worked (the same rule the
    Postgres checks above follow).

    Args:
        path: newline-delimited JSON run.sh extracted from the passthrough sink's output.
        published: per-source values the generator recorded as published.
        boundaries: per-source counter values the generator resumed at after a kill.
        violations: hard failures, appended to in place.
        notes: informational at-least-once observations, appended to in place.
        details: per-source counters for the JSON report, populated in place.
    """
    if not os.path.exists(path):
        violations.append(
            f"passthrough: no output at {path} — the sink produced nothing, or run.sh "
            "could not extract it from the dc_e2e_data volume"
        )
        return

    per_source: dict = {s: [] for s in PASSTHROUGH_SOURCES}
    unexpected_tags: set = set()
    malformed = 0
    total_lines = 0

    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            total_lines += 1
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                malformed += 1
                continue
            tag = event.get("tag")
            if tag not in PASSTHROUGH_TAGS:
                unexpected_tags.add(tag)
                continue
            source = event.get("source")
            value = event.get("value")
            if source in per_source and isinstance(value, int):
                per_source[source].append(value)
            else:
                malformed += 1

    details["passthrough"] = {
        "total_events": total_lines,
        "malformed_events": malformed,
        "unexpected_tags": sorted(t for t in unexpected_tags if t is not None),
        "per_source": {
            s: {"total": len(v), "distinct": len(set(v)), "max_value": max(v) if v else None}
            for s, v in per_source.items()
        },
    }

    if total_lines == 0:
        violations.append(f"passthrough: {path} is empty — the sink received no Records")
        return
    if malformed:
        violations.append(
            f"passthrough: {malformed} event(s) were not parseable as a synth Record "
            "(missing/!int `value` or missing `source`)"
        )
    # The route transform has one branch per Tag with reroute_unmatched=false, so a Tag
    # the sink never subscribed to arriving here means routing stopped discriminating.
    if unexpected_tags:
        violations.append(
            f"passthrough: received Tag(s) it never subscribed to: "
            f"{sorted(t for t in unexpected_tags if t is not None)} — dc.<tag> routing is "
            "not discriminating between Tags"
        )

    for source, values in per_source.items():
        if not values:
            violations.append(
                f"passthrough/{source}: zero Records reached the passthrough sink "
                "(the dc.<tag> route delivered nothing)"
            )
            continue
        arrived = set(values)
        # Same basis as check_synth_topic (#312): diff against what the generator recorded
        # as published, not against `max(arrived) + 1`. Inferring the expectation from the
        # data being checked cannot see a missing tail — and a passthrough sink deserves
        # the same standard as a blessed one, which is the whole point of covering it.
        expected = published.get(source, set())
        if not expected:
            violations.append(
                f"passthrough/{source}: the ledger recorded nothing published, so the "
                "passthrough cannot be verified"
            )
            continue
        missing = expected - arrived
        kill_points = set(boundaries.get(source, set())) | {max(expected) + 1}
        tolerated = {
            value
            for value in missing
            if any(0 < point - value <= MAX_KILL_GAP for point in kill_points)
        }
        lost = missing - tolerated
        if lost:
            violations.append(
                f"passthrough/{source}: {len(lost)} published Record(s) never reached the "
                f"passthrough sink, away from any kill point — first missing: "
                f"{sorted(lost)[:10]} — data loss through the passthrough"
            )
        if tolerated:
            notes.append(
                f"passthrough/{source}: {len(tolerated)} Record(s) lost in flight where "
                "the generator was killed (tolerated)"
            )
        if len(values) > len(arrived):
            notes.append(
                f"passthrough/{source}: {len(values) - len(arrived)} duplicate value(s) "
                "(at-least-once re-delivery; deduplicated on read)"
            )


def check_raw(
    path: str,
    boundaries: dict,
    violations: list,
    notes: list,
    details: dict,
) -> None:
    """Assert raw / generic-subscription mode reached a real Destination (#227).

    This is the end-to-end proof for a message type dc_bridge has no compile-time
    knowledge of: `/dc/e2e/synth/synth00` carries `dc_interfaces/msg/StringStamped` (a
    custom, non-std_msgs type), and every event here was produced by loading that type's
    introspection type support at run time, walking its fields, and shipping the result
    through the `dc.raw.` Tag namespace to a `file` Destination.

    Loss is judged only *inside* the window raw mode was actually subscribed for. Unlike
    a Measurement, a raw subscription does not exist until the topic is discovered
    (`raw.rescan_interval_secs`), and the harness restarts the whole container mid-run —
    so values published before the first discovery, or during a re-discovery, were never
    offered to the pipeline at all. Between the first and last value that did arrive, a
    gap is real loss and fails.

    Args:
        path: newline-delimited JSON run.sh extracted from the raw `file` Destination.
        boundaries: per-source counter values the generator resumed at after a kill.
        violations: hard failures, appended to in place.
        notes: informational at-least-once observations, appended to in place.
        details: counters for the JSON report, populated in place.
    """
    if not os.path.exists(path):
        violations.append(
            f"raw: no output at {path} — the raw Destination produced nothing, or run.sh "
            "could not extract it from the dc_e2e_data volume"
        )
        return

    values: list = []
    unexpected_tags: set = set()
    malformed = 0
    total_lines = 0
    namespace_events = 0
    decoded_fields = 0
    # Stored volume (#227): the bytes a raw Record actually costs in a Destination, and
    # the wall-clock span they arrived over, so every run reports what "collect this
    # topic raw" is worth in MB/day rather than leaving operators to guess. Reported,
    # never gated: throughput on a CI runner is not a property worth failing a build on.
    # Shared with the sim-backed benchmark (#326) so both report the same accounting.
    volume = raw_volume.Volume()

    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            total_lines += 1
            volume.add_line(line)
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                malformed += 1
                continue
            if event.get("tag") != RAW_TAG:
                unexpected_tags.add(event.get("tag"))
                continue
            namespace_events += 1
            volume.add_time(event.get("timestamp"))
            # The introspection walk has to have produced every field of the message:
            # the two `string`s, and the nested Header with its nested builtin Time.
            stamp = (event.get("header") or {}).get("stamp") or {}
            if (
                event.get("group_key") == RAW_SOURCE
                and isinstance(stamp.get("sec"), int)
                and isinstance(stamp.get("nanosec"), int)
                and stamp["sec"] > 0
            ):
                decoded_fields += 1
            try:
                values.append(int(json.loads(event["data"])["value"]))
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                malformed += 1

    arrived = set(values)
    details["raw"] = {
        "total_events": total_lines,
        "namespace_events": namespace_events,
        "malformed_events": malformed,
        "fully_decoded_events": decoded_fields,
        "unexpected_tags": sorted(t for t in unexpected_tags if t is not None),
        "distinct_values": len(arrived),
        "first_value": min(arrived) if arrived else None,
        "last_value": max(arrived) if arrived else None,
        **volume.summary(),
    }

    if total_lines == 0:
        violations.append(
            f"raw: {path} is empty — no generic-subscription Record reached the Destination"
        )
        return
    if malformed:
        violations.append(
            f"raw: {malformed} event(s) did not carry a decodable StringStamped payload "
            "— the introspection walk produced something the message does not contain"
        )
    if decoded_fields != namespace_events:
        violations.append(
            f"raw: only {decoded_fields} of {namespace_events} event(s) carried the full "
            "decoded message (group_key + nested header.stamp) — field-by-field "
            "conversion of the custom type is incomplete"
        )
    # Same reasoning as the passthrough's Tag check: the namespace route branch must
    # still discriminate, or every Measurement Record would land here too.
    if unexpected_tags:
        violations.append(
            f"raw: received Tag(s) outside the raw namespace: "
            f"{sorted(t for t in unexpected_tags if t is not None)} — the dc.raw. route "
            "branch is not discriminating"
        )
    if not arrived:
        violations.append("raw: no event carried a usable counter value")
        return

    kill_points = set(boundaries.get(RAW_SOURCE, set()))
    gaps = set(range(min(arrived), max(arrived) + 1)) - arrived
    tolerated = {
        value for value in gaps if any(0 < point - value <= MAX_KILL_GAP for point in kill_points)
    }
    lost = gaps - tolerated
    if lost:
        violations.append(
            f"raw: {len(lost)} Record(s) missing between the first and last raw value "
            f"received, away from any kill point — first missing: {sorted(lost)[:10]} — "
            "data loss through raw mode"
        )
    if tolerated:
        notes.append(
            f"raw: {len(tolerated)} Record(s) lost in flight where the generator was "
            "killed (tolerated)"
        )
    if len(values) > len(arrived):
        notes.append(
            f"raw: {len(values) - len(arrived)} duplicate value(s) (at-least-once "
            "re-delivery; deduplicated on read)"
        )


def check_mcap_passthrough(
    path: str,
    published: dict,
    boundaries: dict,
    violations: list,
    notes: list,
    details: dict,
) -> None:
    """Assert the ADR-0009 MCAP passthrough writer got the same zero-loss treatment.

    A second, differently-shaped passthrough from check_passthrough() above: Vector has
    no MCAP sink, so params/e2e_mcap_sink.toml is a `socket` sink streaming to the
    standalone `dc_mcap_writer` process instead of to a Vector-native sink. `path` is a
    JSON summary of its `.mcap` capture (scripts/mcap_summary.py's output,
    {tag: [record, ...]}), not the raw binary — parsing MCAP needs the `mcap` library,
    which only the DC image has, not this script's own host runner. A missing or empty
    summary is a FAIL, never a skip, same rule as check_passthrough().

    Args:
        path: JSON summary file produced by scripts/mcap_summary.py.
        published: per-source values the generator recorded as published.
        boundaries: per-source counter values the generator resumed at after a kill.
        violations: hard failures, appended to in place.
        notes: informational at-least-once observations, appended to in place.
        details: per-source counters for the JSON report, populated in place.
    """
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        violations.append(
            f"mcap: no output at {path} — dc_mcap_writer produced nothing, or run.sh "
            "could not summarize its .mcap capture"
        )
        return

    with open(path) as f:
        try:
            per_tag = json.load(f)
        except json.JSONDecodeError as exc:
            violations.append(f"mcap: {path} is not valid JSON ({exc})")
            return

    # The route transform has one branch per Tag with reroute_unmatched=false, so a Tag
    # the sink never subscribed to arriving here means routing stopped discriminating.
    unexpected_tags = sorted(set(per_tag) - MCAP_TAGS)
    if unexpected_tags:
        violations.append(
            f"mcap: received Tag(s) it never subscribed to: {unexpected_tags} — dc.<tag> "
            "routing is not discriminating between Tags"
        )

    per_source: dict = {s: [] for s in MCAP_SOURCES}
    malformed = 0
    for tag, records in per_tag.items():
        if tag not in MCAP_TAGS:
            continue
        source = tag.rsplit(".", 1)[-1]
        for record in records:
            value = record.get("value")
            if source in per_source and isinstance(value, int):
                per_source[source].append(value)
            else:
                malformed += 1

    details["mcap"] = {
        "malformed_events": malformed,
        "unexpected_tags": unexpected_tags,
        "per_source": {
            s: {"total": len(v), "distinct": len(set(v)), "max_value": max(v) if v else None}
            for s, v in per_source.items()
        },
    }

    if malformed:
        violations.append(
            f"mcap: {malformed} event(s) were not parseable as a synth Record "
            "(missing/non-int `value`)"
        )

    for source, values in per_source.items():
        if not values:
            violations.append(
                f"mcap/{source}: zero Records reached dc_mcap_writer "
                "(the dc.<tag> route delivered nothing)"
            )
            continue
        arrived = set(values)
        # Same basis as check_passthrough (#312): diff against what the generator
        # recorded as published, not against `max(arrived) + 1`.
        expected = published.get(source, set())
        if not expected:
            violations.append(
                f"mcap/{source}: the ledger recorded nothing published, so the MCAP "
                "capture cannot be verified"
            )
            continue
        missing = expected - arrived
        kill_points = set(boundaries.get(source, set())) | {max(expected) + 1}
        tolerated = {
            value
            for value in missing
            if any(0 < point - value <= MAX_KILL_GAP for point in kill_points)
        }
        lost = missing - tolerated
        if lost:
            violations.append(
                f"mcap/{source}: {len(lost)} published Record(s) never reached "
                f"dc_mcap_writer, away from any kill point — first missing: "
                f"{sorted(lost)[:10]} — data loss through the MCAP passthrough"
            )
        if tolerated:
            notes.append(
                f"mcap/{source}: {len(tolerated)} Record(s) lost in flight where the "
                "generator was killed (tolerated)"
            )
        if len(values) > len(arrived):
            notes.append(
                f"mcap/{source}: {len(values) - len(arrived)} duplicate value(s) "
                "(at-least-once re-delivery; deduplicated on read)"
            )


def verify_zero_loss(args, violations: list, notes: list, details: dict) -> None:
    """The checks the zero-loss harness has always run, in the order it always ran them."""
    pg = args.postgres_container
    published = read_ledger(args.ledger_file)
    boundaries = read_boundaries(args.ledger_file)
    for i in range(args.num_synth_topics):
        name = f"synth{i:02d}"
        check_synth_topic(
            pg,
            name,
            published.get(name, set()),
            boundaries.get(name, set()),
            violations,
            notes,
            details,
        )
    for tag in REAL_TAGS:
        check_real_tag(pg, tag, violations, notes, details)
    check_files(pg, violations, notes, details)
    check_timestamp_resolution(pg, FAST_TAG, violations, notes, details)
    if args.passthrough_file is not None:
        check_passthrough(args.passthrough_file, published, boundaries, violations, notes, details)
    if args.mcap_summary_file is not None:
        check_mcap_passthrough(
            args.mcap_summary_file, published, boundaries, violations, notes, details
        )
    if args.raw_file is not None:
        check_raw(args.raw_file, boundaries, violations, notes, details)


def verify_incident(args, violations: list, notes: list, details: dict) -> None:
    """run_incident.sh's assertions, one stage per invocation (a FlushEvent is published
    between them, which is an action for the scenario script, not a check of ours)."""
    pg = args.postgres_container

    if args.stage == INCIDENT_STAGE_ARMED:
        live_count = wait_for_count(
            pg,
            f"SELECT count(*) FROM dc_records WHERE tag = '{args.live_tag}'",
            MIN_LIVE_ROWS,
            args.timeout_seconds,
        )
        check_live_flowing(live_count, args.live_tag, args.timeout_seconds, violations, details)
        buffered_count = scalar_int(
            pg, f"SELECT count(*) FROM dc_records WHERE tag = '{args.buffered_tag}'"
        )
        check_armed_silent(buffered_count, args.buffered_tag, violations, details)
        return

    incident_count = wait_for_count(
        pg,
        f"SELECT count(*) FROM dc_records WHERE incident_id = '{args.incident_id}'",
        MIN_INCIDENT_ROWS,
        args.timeout_seconds,
    )
    check_released_window(
        pg,
        args.incident_id,
        args.buffered_tag,
        args.live_tag,
        incident_count,
        violations,
        details,
    )


def verify_retention(args, violations: list, notes: list, details: dict) -> None:
    """run_retention.sh's assertions, one stage per invocation (the store comes up between
    them, and the shed File's absence on disk is the scenario script's volume mount)."""
    pg = args.postgres_container
    if args.stage == RETENTION_STAGE_SHED:
        query = "SELECT count(*) FROM dc_files WHERE deleted = true AND uploaded = false"
        check_shed_row(
            wait_for_count(pg, query, 1, args.timeout_seconds),
            args.timeout_seconds,
            violations,
            details,
        )
        return
    query = "SELECT count(*) FROM dc_files WHERE uploaded = true"
    check_uploaded_row(
        wait_for_count(pg, query, 1, args.timeout_seconds),
        args.timeout_seconds,
        violations,
        details,
    )


PROFILE_STAGES = {
    "zero-loss": (),
    "incident": INCIDENT_STAGES,
    "retention": RETENTION_STAGES,
}

PROFILE_LABELS = {
    "zero-loss": "ZERO-LOSS",
    "incident": "INCIDENT",
    "retention": "RETENTION",
}

# The unit the pass line counts: the zero-loss profile checks per-source detail blocks,
# the other two run a handful of assertions over one table.
PROFILE_COUNT_NOUN = {
    "zero-loss": "sources checked",
    "incident": "checks run",
    "retention": "checks run",
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--postgres-container",
        default="dc_e2e_postgres",
        help="name of the running Postgres container to query via podman exec",
    )
    parser.add_argument(
        "--exec-sql",
        default=None,
        help="run one SQL statement against the Postgres container, print the result and "
        "exit. lib/harness.sh's pg_exec() shells out to this: the module is the only SQL "
        "author in the e2e tree, so there is one psql seam, not one implementation in bash "
        "and another here",
    )
    parser.add_argument(
        "--profile",
        choices=sorted(PROFILE_STAGES),
        default="zero-loss",
        help="which set of checks to run (see the module docstring): the zero-loss harness's "
        "own, run_incident.sh's (#291) or run_retention.sh's (#267)",
    )
    parser.add_argument(
        "--stage",
        default=None,
        help="which half of an incident/retention run to verify — armed/released for "
        "--profile incident, shed/uploaded for --profile retention. A FlushEvent is "
        "published (or the store brought up) between the two halves, so no single "
        "invocation can cover both",
    )
    parser.add_argument(
        "--timeout-seconds",
        type=int,
        default=None,
        help="bound for the one wait the incident/retention stage polls under",
    )
    parser.add_argument(
        "--incident-id",
        default=None,
        help="the incident_id the FlushEvent carried (--profile incident)",
    )
    parser.add_argument(
        "--live-tag",
        default=None,
        help="Tag of the Measurement that is never armed (--profile incident)",
    )
    parser.add_argument(
        "--buffered-tag",
        default=None,
        help="Tag of the Measurement armed with buffer_duration_sec (--profile incident)",
    )
    parser.add_argument("--num-synth-topics", type=int, default=14)
    parser.add_argument(
        "--ledger-file",
        default=None,
        help="workload ledger extracted from the dc_e2e_data volume (what was published); "
        "required by --profile zero-loss, unused by the others",
    )
    parser.add_argument(
        "--passthrough-file",
        default=None,
        help="newline-delimited JSON extracted from the ADR-0003 passthrough sink's output; "
        "omit for a scenario that doesn't configure the passthrough sink (#445's split-"
        "topology mode has no custom_config_files at all — Vector runs from the upstream "
        "image alone, so there's no passthrough output to extract)",
    )
    parser.add_argument(
        "--mcap-summary-file",
        default=None,
        help="JSON summary (scripts/mcap_summary.py output) of the ADR-0009 MCAP "
        "passthrough writer's .mcap capture; omit like --passthrough-file above",
    )
    parser.add_argument(
        "--raw-file",
        default=None,
        help="newline-delimited JSON extracted from the raw (#227) `file` Destination; "
        "omit like --passthrough-file above",
    )
    parser.add_argument("--report", default=None, help="write a JSON report to this path")
    parser.add_argument(
        "--conditions-file",
        default=None,
        help="JSON file (e.g. from run_degraded.sh, #366) describing the network conditions "
        "this run was verified under; merged into --report's 'conditions' key unchanged. A "
        "result is never separable from the conditions that produced it — omitted, the report "
        "carries no 'conditions' key at all, rather than implying an unstated (loopback) one.",
    )
    args = parser.parse_args()

    if args.exec_sql is not None:
        print(psql(args.postgres_container, args.exec_sql))
        return 0

    stages = PROFILE_STAGES[args.profile]
    if stages:
        if args.stage not in stages:
            parser.error(f"--profile {args.profile} needs --stage {'|'.join(stages)}")
        if args.timeout_seconds is None:
            parser.error(f"--profile {args.profile} needs --timeout-seconds")
        if args.profile == "incident":
            for name in ("incident-id", "live-tag", "buffered-tag"):
                if getattr(args, name.replace("-", "_")) is None:
                    parser.error(f"--profile incident needs --{name}")
    elif args.stage is not None:
        parser.error("--stage only applies to --profile incident/--profile retention")
    elif args.ledger_file is None:
        parser.error("--profile zero-loss needs --ledger-file")

    violations: list = []
    notes: list = []
    details: dict = {}
    if args.profile == "zero-loss":
        verify_zero_loss(args, violations, notes, details)
    elif args.profile == "incident":
        verify_incident(args, violations, notes, details)
    else:
        verify_retention(args, violations, notes, details)

    label = PROFILE_LABELS[args.profile]
    stage = f", stage={args.stage}" if args.stage else ""
    report = {
        "profile": args.profile,
        "stage": args.stage,
        "pass": not violations,
        "violations": violations,
        "notes": notes,
        "details": details,
    }
    if args.conditions_file:
        with open(args.conditions_file) as f:
            report["conditions"] = json.load(f)
    if args.report:
        with open(args.report, "w") as f:
            json.dump(report, f, indent=2)

    # Notes (at-least-once boundary re-sends) print in both the pass and fail paths — they
    # are expected and never affect the exit status; only violations do.
    if notes:
        print(f"NOTES ({len(notes)} at-least-once duplicate(s), deduped on read):")
        for n in notes:
            print(f"  - {n}")

    if violations:
        print(f"{label} VERIFICATION FAILED{stage}:", file=sys.stderr)
        for v in violations:
            print(f"  - {v}", file=sys.stderr)
        return 1

    print(
        f"{label} VERIFICATION PASSED{stage} "
        f"({len(details)} {PROFILE_COUNT_NOUN[args.profile]}, 0 violations)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
