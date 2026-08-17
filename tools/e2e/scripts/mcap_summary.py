#!/usr/bin/env python3
"""Summarize a dc_mcap_writer capture for verify_zero_loss.py (#210 e2e coverage).

Runs *inside* a one-off container against the dc_e2e_data volume — the same pattern
run.sh already uses to read back the workload ledger and the NDJSON passthrough sink's
output — never on the CI host runner. It needs the `mcap` library, which is only
installed in the DC workspace image (tools/e2e/Containerfile), not on the runner.

Emits one JSON object to stdout: {"<tag>": [<record>, ...], ...}, one entry per Channel
(Tag) found across every `.mcap` file in the given directory.
"""

import argparse
import glob
import json
import sys

from mcap.exceptions import McapError
from mcap.reader import make_reader


def summarize(directory: str) -> dict:
    """Read every `.mcap` file in `directory` and group decoded Records by Tag.

    A file that fails to parse (e.g. left truncated by a process that was killed
    before it could finish writing) is skipped with a warning rather than aborting the
    whole summary — verify_zero_loss.py's own loss check is what should fail the run
    in that case, on whatever Records the truncation actually cost, not this script.

    Args:
        directory: directory containing dc_mcap_writer's rotated `.mcap` files.

    Returns:
        dict: {Tag (Channel topic): [decoded Record, ...]}, in file/message order.
    """
    per_tag: dict = {}
    for path in sorted(glob.glob(f"{directory}/*.mcap")):
        try:
            with open(path, "rb") as mcap_file:
                for _, channel, message in make_reader(mcap_file).iter_messages():
                    per_tag.setdefault(channel.topic, []).append(json.loads(message.data))
        except McapError as exc:
            print(f"mcap_summary: skipping unreadable {path}: {exc}", file=sys.stderr)
    return per_tag


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", help="directory containing dc_mcap_writer's .mcap files")
    args = parser.parse_args()
    print(json.dumps(summarize(args.directory)))


if __name__ == "__main__":
    main()
