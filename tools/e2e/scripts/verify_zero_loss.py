#!/usr/bin/env python3
"""Hard-failing zero-loss / exactly-once verification for the E2E harness (#249).

Queries Postgres through `podman compose exec` (no host-side psycopg2/psql dependency
— only podman itself, already required to run the harness). Every check is a real
assertion: a query that can't run, or a table that doesn't exist, is a FAIL, not a
skip — matching the #246 follow-up decision that a verification which silently didn't
execute must never look identical to one that passed.

Checks, per the PRD acceptance criteria ("every Record lands in PostgreSQL exactly
once and every File status row is consistent"):

1. Per synth topic (tools/e2e/scripts/workload_generator.py's strictly-incrementing
   `value` counter): no duplicate values (double delivery) and no gap between 0 and the
   observed max (loss) — a check independent of timestamp precision.
2. Per real-measurement Tag (memory/os/storage/uptime/tcp_health/dummy): at least one
   Record landed, and no duplicate (tag, date) rows (double delivery).
3. File pipeline (ADR-0005): at least one verified file_status row and one
   group_complete marker for the camera group, and no duplicate "uploaded" status row
   for the same local_path (the Uploader's own idempotency guarantee, ADR-0005/#248,
   holding up under this harness's induced outage + restart).
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


def psql(compose_file: str, query: str) -> str:
    result = subprocess.run(
        [
            "podman",
            "compose",
            "-f",
            compose_file,
            "exec",
            "-T",
            "postgres",
            "psql",
            "-U",
            "dc",
            "-d",
            "dc",
            "-tAc",
            query,
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(f"psql query failed: {query!r}\n{result.stderr}")
    return result.stdout.strip()


def scalar_int(compose_file: str, query: str) -> int:
    out = psql(compose_file, query)
    return int(out) if out else 0


def check_synth_topic(compose_file: str, name: str, violations: list, details: dict) -> None:
    dup_count = scalar_int(
        compose_file,
        f"SELECT count(*) FROM (SELECT value FROM dc_records "
        f"WHERE source='{name}' GROUP BY value HAVING count(*) > 1) t",
    )
    total = scalar_int(compose_file, f"SELECT count(*) FROM dc_records WHERE source='{name}'")
    max_out = psql(compose_file, f"SELECT max(value) FROM dc_records WHERE source='{name}'")
    max_value = int(max_out) if max_out else None

    details[name] = {"total": total, "max_value": max_value, "duplicate_values": dup_count}

    if max_value is None or total == 0:
        violations.append(f"{name}: zero Records landed in Postgres")
        return
    if dup_count > 0:
        violations.append(
            f"{name}: {dup_count} value(s) delivered more than once (double delivery)"
        )
    expected = max_value + 1
    if total < expected:
        violations.append(
            f"{name}: {expected - total} Record(s) missing (expected {expected} values "
            f"0..{max_value}, got {total} rows) — data loss"
        )


def check_real_tag(compose_file: str, tag: str, violations: list, details: dict) -> None:
    total = scalar_int(compose_file, f"SELECT count(*) FROM dc_records WHERE tag='{tag}'")
    dup_count = scalar_int(
        compose_file,
        f"SELECT count(*) FROM (SELECT date FROM dc_records "
        f"WHERE tag='{tag}' GROUP BY date HAVING count(*) > 1) t",
    )
    details[tag] = {"total": total, "duplicate_dates": dup_count}
    if total == 0:
        violations.append(f"{tag}: zero Records landed in Postgres")
    if dup_count > 0:
        violations.append(
            f"{tag}: {dup_count} timestamp(s) delivered more than once (double delivery)"
        )


def check_files(compose_file: str, violations: list, details: dict) -> None:
    status_count = scalar_int(
        compose_file,
        "SELECT count(*) FROM dc_files WHERE kind='file_status' AND group_name='camera'",
    )
    complete_count = scalar_int(
        compose_file,
        "SELECT count(*) FROM dc_files WHERE kind='group_complete' AND group_name='camera'",
    )
    dup_uploaded = scalar_int(
        compose_file,
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
        violations.append(
            f"files: {dup_uploaded} local_path(s) have more than one 'uploaded' status row "
            "(Uploader idempotency violated)"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compose-file", default="compose.yaml")
    parser.add_argument("--num-synth-topics", type=int, default=14)
    parser.add_argument("--report", default=None, help="write a JSON report to this path")
    args = parser.parse_args()

    violations: list = []
    details: dict = {}

    for i in range(args.num_synth_topics):
        check_synth_topic(args.compose_file, f"synth{i:02d}", violations, details)
    for tag in REAL_TAGS:
        check_real_tag(args.compose_file, tag, violations, details)
    check_files(args.compose_file, violations, details)

    report = {"pass": not violations, "violations": violations, "details": details}
    if args.report:
        with open(args.report, "w") as f:
            json.dump(report, f, indent=2)

    if violations:
        print("ZERO-LOSS VERIFICATION FAILED:", file=sys.stderr)
        for v in violations:
            print(f"  - {v}", file=sys.stderr)
        return 1

    print(f"ZERO-LOSS VERIFICATION PASSED ({len(details)} sources checked, 0 violations)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
