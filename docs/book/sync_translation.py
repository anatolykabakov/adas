#!/usr/bin/env python3
"""Keep the Russian book structurally identical to the English one.

Two source trees is the pragmatic choice for prose-heavy technical writing — `.po` files are worse than
markdown for paragraphs full of formulas, tables and code — but it has one real failure mode: the
translation drifts. A chapter gains a section in English, the Russian copy keeps the old one, and nobody
notices for a year.

This tool makes that failure loud. It checks, for every chapter:

* the file exists in both trees (or is an explicit stub);
* the chapter has the same number of ```python blocks;
* **the code inside those blocks is byte-identical** — prose is translated, code is not. A translated
  variable name or a stray localised decimal separator is a snippet that silently means something else;
* headings correspond one-to-one in count and nesting, so the tables of contents cannot diverge.

  python3 docs/book/sync_translation.py            # report
  python3 docs/book/sync_translation.py --strict   # exit 1 on any drift, for the build
  python3 docs/book/sync_translation.py --fix      # restore code blocks from English

`--fix` exists because code is never translated: if a Russian chapter's n-th python block differs from the
English one, the English one is right by definition. That makes copying it back a mechanical repair rather
than a judgement call, and it means a translator can concentrate on prose without hand-checking three
thousand lines of code. It only touches blocks, never prose, and only when the block *count* already
matches — a count mismatch is a structural problem a script must not paper over.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

EN = Path(__file__).resolve().parent
RU = EN.parent / "book_ru"

FENCE = re.compile(r"^```python\s*$(.*?)^```\s*$", re.MULTILINE | re.DOTALL)
HEADING = re.compile(r"(?m)^(#{1,4}) +\S")
# Any fenced block, not just python: a `#` comment inside code is not a heading, and counting it as one
# made the heading check compare code comments across languages. It passed only while the two versions
# happened to contain the same comments.
ANY_FENCE = re.compile(r"^```.*?^```\s*$", re.MULTILINE | re.DOTALL)
# A Russian page that only points at the English one. Marked so the checker can tell "not translated yet"
# from "translated and drifted" — two very different problems.
STUB_MARK = "<!-- translation-stub -->"


def chapters() -> list[Path]:
    out = []
    for p in sorted(EN.rglob("*.md")):
        if "_build" in p.parts:
            continue
        out.append(p.relative_to(EN))
    return out


def code_blocks(text: str) -> list[str]:
    return [m.group(1) for m in FENCE.finditer(text)]


def headings(text: str) -> list[int]:
    return [len(m.group(1)) for m in HEADING.finditer(ANY_FENCE.sub("", text))]


def fix_chapter(rel: Path) -> int:
    """Replace the Russian chapter's python blocks with the English ones. Returns how many changed."""
    en_text = (EN / rel).read_text()
    ru_path = RU / rel
    ru_text = ru_path.read_text()
    en_code = code_blocks(en_text)
    ru_code = code_blocks(ru_text)
    if len(en_code) != len(ru_code):
        return 0  # structural, not mechanical — leave it to a human

    changed = 0
    out = []
    last = 0
    for i, m in enumerate(FENCE.finditer(ru_text)):
        if ru_code[i] != en_code[i]:
            out.append(ru_text[last : m.start(1)])
            out.append(en_code[i])
            last = m.end(1)
            changed += 1
    out.append(ru_text[last:])
    if changed:
        ru_path.write_text("".join(out))
    return changed


def main() -> int:
    strict = "--strict" in sys.argv
    fix = "--fix" in sys.argv

    if fix:
        total = 0
        for rel in chapters():
            ru_path = RU / rel
            if not ru_path.exists() or STUB_MARK in ru_path.read_text():
                continue
            n = fix_chapter(rel)
            if n:
                print(f"восстановлено блоков кода: {n} в {rel}")
                total += n
        print(f"всего восстановлено: {total}")
    missing, stubs, drifted, ok = [], [], [], []

    for rel in chapters():
        ru_path = RU / rel
        if not ru_path.exists():
            missing.append(rel)
            continue

        ru_text = ru_path.read_text()
        if STUB_MARK in ru_text:
            stubs.append(rel)
            continue

        en_text = (EN / rel).read_text()
        problems = []

        en_code, ru_code = code_blocks(en_text), code_blocks(ru_text)
        if len(en_code) != len(ru_code):
            problems.append(f"{len(en_code)} python blocks in EN, {len(ru_code)} in RU")
        else:
            for i, (a, b) in enumerate(zip(en_code, ru_code)):
                if a != b:
                    problems.append(
                        f"python block {i + 1} differs — code must not be translated"
                    )

        en_h, ru_h = headings(en_text), headings(ru_text)
        if en_h != ru_h:
            problems.append(
                f"heading structure differs: {len(en_h)} vs {len(ru_h)} headings"
            )

        (drifted if problems else ok).append((rel, problems))

    print(f"{len(ok)} chapters translated and in sync")
    if stubs:
        print(f"\n{len(stubs)} awaiting translation (stub in place, navigation intact):")
        for rel in stubs:
            print(f"  {rel}")
    if missing:
        print(f"\n{len(missing)} MISSING in the Russian tree — navigation would break:")
        for rel in missing:
            print(f"  {rel}")
    if drifted:
        print(f"\n{len(drifted)} DRIFTED:")
        for rel, problems in drifted:
            print(f"  {rel}")
            for pr in problems:
                print(f"      {pr}")

    bad = bool(missing or drifted)
    if strict and bad:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
