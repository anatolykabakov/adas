#!/usr/bin/env python3
"""Every repository file the book names must exist.

The project deletes aggressively — verticals, controllers, whole docs — and a book reference to a
deleted file is worse than no reference: the reader goes looking and concludes the book cannot be
trusted. This walks both language trees and checks three kinds of mention:

* markdown links whose target is a relative ``.md`` path (chapter cross-references);
* backticked ``docs/*.md`` mentions (the internal documents);
* backticked ``scripts/…`` and source-tree paths ending in a real extension.

Placeholders in angle brackets (``adas_logs/<session>``) and glob-ish mentions are ignored.

  python3 docs/book/check_repo_links.py          # report, exit 1 on any dead reference
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

BOOK = Path(__file__).resolve().parent
DOCS = BOOK.parent
REPO = DOCS.parent

MD_LINK = re.compile(r"\]\((\.\.?/[^)#\s]+\.md)\)")
BACKTICK = re.compile(r"`((?:docs|scripts|app|include|src|tools)/[A-Za-z0-9_./-]+\.(?:md|py|sh|cpp|h|hpp|java|json|proto|yml))`")

# Paths that moved into git history on purpose get called out with that phrase next to them; a
# mention of the *historical* name inside such a sentence is not a dead link.
HISTORY_HINT = ("git history", "истории git", "история в git", "in git history")

def targets_of(md: Path):
    text = md.read_text()
    for m in MD_LINK.finditer(text):
        yield m.group(1), (md.parent / m.group(1)).resolve(), text, m.start()
    for m in BACKTICK.finditer(text):
        rel = m.group(1)
        root = REPO / rel
        # `src/...` and `include/...` are written relative to the C++ tree in prose
        if not root.exists() and rel.split("/", 1)[0] in ("src", "include", "tools"):
            root = REPO / "app/src/main/cpp" / rel
        if not root.exists() and rel.startswith("tools/"):
            root = REPO / "scripts" / rel
        yield rel, root, text, m.start()

def main() -> int:
    dead = 0
    for tree in (BOOK, DOCS / "book_ru"):
        for md in sorted(tree.rglob("*.md")):
            if "_build" in md.parts:
                continue
            for rel, path, text, pos in targets_of(md):
                if "<" in rel or "*" in rel:
                    continue
                if path.exists():
                    continue
                window = text[max(0, pos - 200) : pos + 200]
                if any(h in window for h in HISTORY_HINT):
                    continue
                dead += 1
                print(f"DEAD  {md.relative_to(DOCS)}: {rel}")
    if dead:
        print(f"\n{dead} dead reference(s) — the file was moved or deleted; fix the mention or "
              f"mark it as living in git history")
        return 1
    print("all repository files the book names exist")
    return 0

if __name__ == "__main__":
    sys.exit(main())
