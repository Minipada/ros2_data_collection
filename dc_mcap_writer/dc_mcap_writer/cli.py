"""`dc_mcap_writer` console entrypoint.

Consumes Records over the ADR-0003 `dc.<tag>` passthrough — a Vector `socket` sink
(TCP, newline-delimited JSON, see `dc_demos/config/mcap_sink.toml`) connects to this
process — and writes them to rotated `.mcap` files. See
doc/src/dc/demos/mcap_recording.md.
"""
import argparse
import io
import logging
import signal
import socketserver
import sys
from collections.abc import Sequence
from pathlib import Path

from .writer import RotatingMcapWriter, RotationPolicy, iter_ndjson

logger = logging.getLogger("dc_mcap_writer")


class _RecordHandler(socketserver.StreamRequestHandler):
    """Reads NDJSON Records off one Vector `socket` sink connection."""

    def handle(self) -> None:
        peer = self.client_address
        logger.info("connection from %s", peer)
        text_stream = io.TextIOWrapper(self.rfile, encoding="utf-8")
        for record in iter_ndjson(text_stream):
            self.server.mcap_writer.write_record(record)
        logger.info("connection from %s closed", peer)


class _McapTcpServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address, handler_class, mcap_writer: RotatingMcapWriter):
        super().__init__(server_address, handler_class)
        self.mcap_writer = mcap_writer


def build_arg_parser() -> argparse.ArgumentParser:
    """Build the `dc_mcap_writer` argument parser.

    Returns:
        The configured parser.
    """
    parser = argparse.ArgumentParser(
        description=(
            "Consume Records over the ADR-0003 dc.<tag> passthrough and write them "
            "as rotated .mcap files (issue #210)."
        )
    )
    parser.add_argument(
        "--output-dir", type=Path, required=True, help="Directory .mcap files are written into"
    )
    parser.add_argument("--prefix", default="records", help="Filename prefix for each rotation")
    parser.add_argument(
        "--host", default="127.0.0.1", help="Address to listen on for the Vector socket sink"
    )
    parser.add_argument("--port", type=int, default=9191, help="Port to listen on")
    parser.add_argument(
        "--max-bytes",
        type=int,
        default=128 * 1024 * 1024,
        help="Rotate once the current file reaches this size; 0 disables the size limit",
    )
    parser.add_argument(
        "--max-duration-secs",
        type=float,
        default=300.0,
        help="Rotate once the current file has been open this long; 0 disables the time limit. "
        "A file only becomes readable once its rotation finishes (writes the footer) — this "
        "bounds how much a process that never gets to shut down gracefully (SIGKILL, a "
        "container/restart grace period too short for a clean exit) can cost: everything in "
        "already-finished rotations stays safe, only the file open at the moment of the kill "
        "is at risk.",
    )
    parser.add_argument(
        "--time-key",
        default="date",
        help="Record field holding the normalized timestamp (Bridge's time_key)",
    )
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="Read NDJSON Records from stdin instead of listening on a TCP socket "
        "(manual conversion of an existing NDJSON capture)",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> None:
    """Run the `dc_mcap_writer` process until interrupted.

    Args:
        argv: Command-line arguments, or None to use `sys.argv`.
    """
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    # A container/systemd stop sends SIGTERM, not SIGINT — without this, `finally:
    # mcap_writer.close()` below never runs and the current .mcap is left without its
    # finishing Data End record/index, unreadable. Mapping it onto the same
    # KeyboardInterrupt Ctrl-C already raises reuses that one shutdown path for both.
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    args = build_arg_parser().parse_args(argv)
    rotation = RotationPolicy(
        max_bytes=args.max_bytes or None, max_duration_secs=args.max_duration_secs or None
    )
    mcap_writer = RotatingMcapWriter(
        output_dir=args.output_dir,
        prefix=args.prefix,
        rotation=rotation,
        time_key=args.time_key,
    )
    try:
        if args.stdin:
            mcap_writer.write_records(iter_ndjson(sys.stdin))
        else:
            server = _McapTcpServer((args.host, args.port), _RecordHandler, mcap_writer)
            logger.info("listening on %s:%s, writing to %s", args.host, args.port, args.output_dir)
            try:
                server.serve_forever()
            except KeyboardInterrupt:
                pass
            finally:
                server.server_close()
    finally:
        mcap_writer.close()


if __name__ == "__main__":
    main()
