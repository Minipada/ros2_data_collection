#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Mirror docs/adr/*.md into the mdbook site as real, linkable pages.

mdbook can only render files under doc/src/ that are listed in doc/src/SUMMARY.md, and
doc/book.toml forbids linking outside the book root -- so it can't reach docs/adr/
directly. This copies each ADR verbatim into doc/src/dc/adr/ and rewrites the generated
block in SUMMARY.md to list them, sorted by number. Both are disposable build output
(doc/src/dc/adr/ is gitignored); docs/adr/ stays the only source of truth.

Run before `mdbook build` -- see tools/ci/pre-commit/build_doc.sh.
"""

import re
import shutil
import sys
from pathlib import Path

# tools/ci/pre-commit/<this file> -> the workspace root.
ROOT = Path(__file__).resolve().parents[3]
ADR_SRC = ROOT / "docs" / "adr"
ADR_DEST = ROOT / "doc" / "src" / "dc" / "adr"
SUMMARY = ROOT / "doc" / "src" / "SUMMARY.md"

BEGIN_MARKER = "<!-- BEGIN GENERATED ADR LIST -->"
END_MARKER = "<!-- END GENERATED ADR LIST -->"
NUMBER_RE = re.compile(r"^(\d+)-")


def adrs() -> list[Path]:
    """Every ADR source file, sorted by its leading number.

    Returns:
        docs/adr/*.md paths, e.g. 0002-... before 0015-....
    """
    return sorted(ADR_SRC.glob("*.md"), key=lambda p: int(NUMBER_RE.match(p.name).group(1)))


def title_of(adr: Path) -> str:
    """The ADR's title, from its first `# ` line.

    Args:
        adr: an ADR source file.

    Returns:
        The H1 text, without the leading `# `.
    """
    first_line = adr.read_text().splitlines()[0]
    return first_line.removeprefix("# ").strip()


def copy_pages(found: list[Path]) -> None:
    """Replace doc/src/dc/adr/ with a fresh verbatim copy of `found`.

    Recreating the directory each run means an ADR removed from docs/adr/ doesn't
    linger as a stale, unlisted page.
    """
    if ADR_DEST.exists():
        shutil.rmtree(ADR_DEST)
    ADR_DEST.mkdir(parents=True)
    for adr in found:
        shutil.copyfile(adr, ADR_DEST / adr.name)


def generated_block(found: list[Path]) -> str:
    """The SUMMARY.md lines to place between the markers, one per ADR."""
    lines = [BEGIN_MARKER]
    for adr in found:
        number = NUMBER_RE.match(adr.name).group(1)
        lines.append(f"  - [{number} - {title_of(adr)}](./dc/adr/{adr.name})")
    lines.append(END_MARKER)
    return "\n".join(lines)


def rewrite_summary(found: list[Path]) -> None:
    """Replace the marker-delimited ADR list in SUMMARY.md, leaving the rest untouched."""
    text = SUMMARY.read_text()
    pattern = re.compile(f"{re.escape(BEGIN_MARKER)}.*?{re.escape(END_MARKER)}", re.DOTALL)
    if not pattern.search(text):
        sys.exit(
            f"{SUMMARY}: no {BEGIN_MARKER} .. {END_MARKER} block found -- "
            "add the 'Architecture Decision Records' parent entry with an empty marker "
            "block under it first"
        )
    SUMMARY.write_text(pattern.sub(generated_block(found), text))


def main() -> int:
    found = adrs()
    copy_pages(found)
    rewrite_summary(found)
    print(f"Generated {len(found)} ADR page(s) in {ADR_DEST.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
