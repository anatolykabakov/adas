#!/usr/bin/env python3
"""Execute every Python block in the book and fail on the first one that does not run.

The course is read by people who copy the snippets. A block that raises is worse than no block: it
teaches the reader to distrust the whole chapter.

The contract this enforces is **a chapter is one program, read top to bottom**: blocks share a namespace
within a chapter, so a later block may use constants defined earlier, and every chapter starts clean.
That is how a course chapter is actually read. It is also why the first block of a chapter has to carry
its own imports — and why `--isolated` exists, to list the blocks that would not survive being copied
out on their own. Ten of the thirty-seven blocks in this book are in that category, all of them in
chapters that define shared constants once at the top.

  python3 docs/book/check_snippets.py            # all chapters
  python3 docs/book/check_snippets.py Localization/Overview.md
"""

from __future__ import annotations

import io
import re
import sys
import traceback
from contextlib import redirect_stdout
from pathlib import Path

BOOK = Path(__file__).resolve().parent
FENCE = re.compile(r"^```python\s*$(.*?)^```\s*$", re.MULTILINE | re.DOTALL)

# Blocks that are illustrative fragments rather than runnable programs are marked in the source with
# this comment on their first line, so skipping is explicit and visible to the reader too.
SKIP_MARK = "# not-runnable"


def blocks(md: Path):
    text = md.read_text()
    for m in FENCE.finditer(text):
        line_no = text[: m.start()].count("\n") + 1
        yield line_no, m.group(1)


def main() -> int:
    targets = [BOOK / a for a in sys.argv[1:]] or sorted(
        p for p in BOOK.rglob("*.md") if "_build" not in p.parts
    )

    isolated = "--isolated" in sys.argv
    targets = [t for t in targets if t.name != "--isolated"]

    total = failed = skipped = 0
    for md in targets:
        chapter_ns = {"__name__": "__main__"}
        for line_no, code in blocks(md):
            # The fence regex captures a leading newline, so the marker sits on the first *non-empty*
            # line, not on `splitlines()[0]`.
            first = next((ln for ln in code.splitlines() if ln.strip()), "")
            if SKIP_MARK in first:
                skipped += 1
                continue
            total += 1
            ns = {"__name__": "__main__"} if isolated else chapter_ns
            try:
                with redirect_stdout(io.StringIO()):
                    exec(compile(code, f"{md.relative_to(BOOK)}:{line_no}", "exec"), ns)
            except Exception:
                failed += 1
                print(
                    f"\n=== {'NOT SELF-CONTAINED' if isolated else 'FAIL'} "
                    f"{md.relative_to(BOOK)} line {line_no}"
                )
                traceback.print_exc(limit=3)

    mode = "in isolation" if isolated else "per chapter, shared namespace"
    print(
        f"\n{total} python blocks executed {mode}, {failed} failed, {skipped} skipped as fragments"
    )
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
