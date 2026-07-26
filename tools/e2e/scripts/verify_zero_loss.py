#!/usr/bin/env python3
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

1. Per synth topic (tools/e2e/scripts/workload_generator.py's strictly-incrementing
   `value` counter): the distinct values are exactly 0..max with no gap (loss) — a check
   independent of timestamp precision. Repeated values are counted and reported as notes.
2. Per real-measurement Tag (memory/os/storage/uptime/tcp_health/dummy): at least one
   Record landed; duplicate (tag, date) rows are reported as notes.
3. File pipeline (ADR-0005): at least one verified file_status row and one
   group_complete marker for the camera group; duplicate "uploaded" status rows for the
   same local_path are reported as notes (the Uploader's own idempotency guarantee,
   ADR-0005/#248, holding up under this harness's induced outage + restart).
"""
import argparse
import json
import subprocess
import sys

REAL_TAGS = [
    "dc.measurement.memory",
    "dc.measurement.os",
    "dc.measurement.storage",
    "dc.measurement.uptime",
    "dc.measurement.tcp_health",
    "dc.measurement.dummy",
]


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


def check_synth_topic(
    pg_container: str, name: str, violations: list, notes: list, details: dict
) -> None:
    total = scalar_int(pg_container, f"SELECT count(*) FROM dc_records WHERE source='{name}'")
    distinct = scalar_int(
        pg_container, f"SELECT count(DISTINCT value) FROM dc_records WHERE source='{name}'"
    )
    dup_values = scalar_int(
        pg_container,
        f"SELECT count(*) FROM (SELECT value FROM dc_records "
        f"WHERE source='{name}' GROUP BY value HAVING count(*) > 1) t",
    )
    max_out = psql(pg_container, f"SELECT max(value) FROM dc_records WHERE source='{name}'")
    max_value = int(max_out) if max_out else None

    details[name] = {
        "total": total,
        "distinct": distinct,
        "max_value": max_value,
        "duplicate_values": dup_values,
    }

    if max_value is None or total == 0:
        violations.append(f"{name}: zero Records landed in Postgres")
        return
    # Zero-loss (hard): every value 0..max must be present after dedup. A gap here is a
    # value the pipeline accepted but never delivered — real loss.
    expected = max_value + 1
    if distinct < expected:
        violations.append(
            f"{name}: {expected - distinct} value(s) missing (expected {expected} distinct "
            f"values 0..{max_value}, got {distinct}) — data loss"
        )
    # At-least-once boundary re-send (informational): collapsed by dedup-on-read.
    if total > distinct:
        notes.append(
            f"{name}: {total - distinct} duplicate row(s) across {dup_values} value(s) "
            "(at-least-once outage-boundary re-send; deduped on read)"
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--postgres-container",
        default="dc_e2e_postgres",
        help="name of the running Postgres container to query via podman exec",
    )
    parser.add_argument("--num-synth-topics", type=int, default=14)
    parser.add_argument("--report", default=None, help="write a JSON report to this path")
    args = parser.parse_args()

    pg = args.postgres_container
    violations: list = []
    notes: list = []
    details: dict = {}

    for i in range(args.num_synth_topics):
        check_synth_topic(pg, f"synth{i:02d}", violations, notes, details)
    for tag in REAL_TAGS:
        check_real_tag(pg, tag, violations, notes, details)
    check_files(pg, violations, notes, details)

    report = {"pass": not violations, "violations": violations, "notes": notes, "details": details}
    if args.report:
        with open(args.report, "w") as f:
            json.dump(report, f, indent=2)

    # Notes (at-least-once boundary re-sends) print in both the pass and fail paths — they
    # are expected and never affect the exit status; only violations (loss) do.
    if notes:
        print(f"NOTES ({len(notes)} at-least-once duplicate(s), deduped on read):")
        for n in notes:
            print(f"  - {n}")

    if violations:
        print("ZERO-LOSS VERIFICATION FAILED:", file=sys.stderr)
        for v in violations:
            print(f"  - {v}", file=sys.stderr)
        return 1

    print(f"ZERO-LOSS VERIFICATION PASSED ({len(details)} sources checked, 0 violations)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
