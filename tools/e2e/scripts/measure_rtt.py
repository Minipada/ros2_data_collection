# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""TCP connect-time sampler (#366): measures how long it takes to open a TCP connection
to host:port, `--count` times, and reports min/avg/max in milliseconds as JSON.

Used by run_degraded.sh for two things that are the same measurement at different
points in a run: (1) a readiness probe, retried until a Destination accepts connections
at all, and (2) the shaping pre-check that a degraded run must never skip — a profile's
`tc netem delay` shows up almost entirely in this number, since it delays every packet
the shaped Destination sends, including the SYN-ACK a TCP connect blocks on. A run whose
shaping silently failed to apply would otherwise produce a green result under a degraded
label while having tested nothing (see run_degraded.sh's own header and #366's PRD).

Deliberately stdlib-only (no `ping`/`iputils`, which the Postgres/RustFS images this
measures do not ship) and run from inside the `dc-e2e` image, which does have Python and
sits on the same shaped network as the real DC stack it stands in for.
"""

import argparse
import json
import socket
import sys
import time


def sample(host: str, port: int, count: int, timeout: float) -> list[float]:
    """Returns `count` connect-time samples in milliseconds. Raises OSError on the
    first failed connection — a caller wanting readiness-style retries wraps this in
    its own retry loop rather than getting partial success back from here."""
    samples = []
    for _ in range(count):
        start = time.monotonic()
        with socket.create_connection((host, port), timeout=timeout):
            pass
        samples.append((time.monotonic() - start) * 1000)
    return samples


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host")
    parser.add_argument("port", type=int)
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=5.0, help="per-connect timeout, seconds")
    args = parser.parse_args()

    try:
        samples = sample(args.host, args.port, args.count, args.timeout)
    except OSError as exc:
        print(json.dumps({"error": str(exc)}), file=sys.stderr)
        return 1

    result = {
        "host": args.host,
        "port": args.port,
        "samples_ms": [round(s, 2) for s in samples],
        "min_ms": round(min(samples), 2),
        "avg_ms": round(sum(samples) / len(samples), 2),
        "max_ms": round(max(samples), 2),
    }
    print(json.dumps(result))
    return 0


if __name__ == "__main__":
    sys.exit(main())
