#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Workspace-level checks over every dc_* ROS 2 package manifest.

    versions   one version across every dc_*/package.xml, declared by each dc_*/setup.py
    metadata   no package left the template's placeholder license/maintainer/description

Vendored from minipada/ros2_pre-commit: its version check finds setup.py with an
unbounded `find`, so it walks into the ./.venv `uv sync` creates and checks numpy's
setup.py against the DC version, and both its checks need an xmllint CI doesn't have.
"""

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# tools/ci/pre-commit/<this file> -> the workspace root.
ROOT = Path(__file__).resolve().parents[3]


def manifests() -> list[Path]:
    """Every dc_* package manifest, in a stable order.

    Returns:
        The sorted list of `dc_*/package.xml` paths under the workspace root.
    """
    return sorted(ROOT.glob("dc_*/package.xml"))


def texts(manifest: Path, tag: str) -> list[str]:
    """The stripped text of every `tag` element directly under `<package>`.

    Args:
        manifest: the package.xml to read.
        tag: the child element name, e.g. "version" or "maintainer".

    Returns:
        One entry per matching element; an empty list if the manifest has none.
    """
    root = ET.parse(manifest).getroot()
    return [(node.text or "").strip() for node in root.findall(tag)]


def check_versions() -> list[str]:
    """Check that one version covers every package.xml and every setup.py.

    Returns:
        A message per inconsistency; empty if the workspace is consistent.
    """
    found = manifests()
    if not found:
        return ["No dc_*/package.xml found -- run this from the workspace root"]

    failures = []
    expected = (texts(found[0], "version") or [""])[0]
    for manifest in found:
        version = (texts(manifest, "version") or [""])[0]
        if version != expected:
            failures.append(
                f"Version is inconsistent in {manifest.relative_to(ROOT)}: "
                f"{version or '<missing>'}, expected {expected}"
            )

    declares_version = re.compile(rf"""version\s*=\s*["']{re.escape(expected)}["']""")
    for setup in sorted(ROOT.glob("dc_*/setup.py")):
        if not declares_version.search(setup.read_text()):
            failures.append(
                f"Version is incorrect in {setup.relative_to(ROOT)}. Should be {expected}"
            )
    return failures


def check_metadata() -> list[str]:
    """Check that no manifest still carries the package template's placeholders.

    Returns:
        A message per unset field; empty if every package sets all three.
    """
    failures = []
    for manifest in manifests():
        where = manifest.relative_to(ROOT)
        if any("TODO" in value for value in texts(manifest, "license")):
            failures.append(f"Need to set license in {where}")
        if any("root" in value for value in texts(manifest, "maintainer")):
            failures.append(f"Need to set maintainer in {where}")
        if any("TODO" in value for value in texts(manifest, "description")):
            failures.append(f"Need to set description in {where}")
    return failures


CHECKS = {"versions": check_versions, "metadata": check_metadata}


def main() -> int:
    """Run the requested check.

    Returns:
        0 if it passed, 1 if it reported failures, 2 on a usage error.
    """
    if len(sys.argv) != 2 or sys.argv[1] not in CHECKS:
        print(f"usage: {Path(sys.argv[0]).name} {{{'|'.join(CHECKS)}}}", file=sys.stderr)
        return 2

    failures = CHECKS[sys.argv[1]]()
    for failure in failures:
        print(failure)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
