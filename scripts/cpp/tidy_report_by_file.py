#!/usr/bin/env python3
"""Group a clang-tidy report by file and sort it by how much the findings can actually hurt.

Reads the same `-export-fixes` YAML as `tidy_report_handler.py` and applies the same config filter, so
the two never disagree about what counts. The difference is the ordering: a flat list of 1400 findings
says nothing about where to start, and the checks are not equally serious.

Tiers, and the reason for each:

* **P0 — can be wrong at runtime.** The compiler could not parse the file at all, the analyser proved a
  path, or a conversion silently loses value. These change behaviour, not appearance.
* **P1 — unsafe by construction.** Bounds, casts away const, C arrays, globals: nothing is provably
  broken today, but the guarantee is missing and a later edit collects the bill.
* **P2 — cosmetic.** Braces, naming, include hygiene, modernisation. Worth a sweep, never worth a
  night: `misc-include-cleaner` alone accounts for hundreds and every fix is a line of `#include`.

  python3 ../../../../scripts/cpp/tidy_report_by_file.py tidy_errors.yml .clang-tidy
  python3 ../../../../scripts/cpp/tidy_report_by_file.py tidy_errors.yml .clang-tidy --md > report.md
"""

from __future__ import annotations

import argparse
import os.path
import sys
from collections import Counter, defaultdict

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tidy_report_handler import (
    IGNORE_LIST,
    is_enabled,
    line_of_offset,
    parse_checks,
)  # noqa: E402

# Prefix → tier. The longest matching prefix wins, so a specific check can override its family.
TIERS = {
    "clang-diagnostic-error": "P0",
    "clang-analyzer-": "P0",
    "bugprone-": "P0",
    "bugprone-easily-swappable-parameters": "P2",
    "cppcoreguidelines-narrowing-conversions": "P0",
    "cppcoreguidelines-": "P1",
    "performance-": "P1",
    "misc-no-recursion": "P1",
    "misc-": "P2",
    "google-": "P2",
    "modernize-": "P2",
    "readability-": "P2",
    "portability-": "P1",
    "android-": "P1",
    "clang-diagnostic-": "P2",
}


def tier_of(name: str) -> str:
    best, tier = "", "P2"
    for prefix, value in TIERS.items():
        if name.startswith(prefix) and len(prefix) > len(best):
            best, tier = prefix, value
    return tier


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report")
    parser.add_argument("config")
    parser.add_argument("--root", default="", help="strip this prefix from paths")
    parser.add_argument(
        "--md", action="store_true", help="markdown instead of plain text"
    )
    parser.add_argument(
        "--top", type=int, default=25, help="how many P0/P1 findings to list"
    )
    args = parser.parse_args()

    with open(args.config, encoding="utf-8") as handle:
        patterns = parse_checks(yaml.safe_load(handle)["Checks"])
    with open(args.report, encoding="utf-8") as handle:
        diagnostics = yaml.safe_load(handle).get("Diagnostics", [])

    per_file: dict[str, Counter] = defaultdict(Counter)
    per_file_checks: dict[str, Counter] = defaultdict(Counter)
    details: list[tuple[str, str, str, int, str]] = []
    totals = Counter()

    for entry in diagnostics:
        name = entry.get("DiagnosticName", "")
        message = entry.get("DiagnosticMessage", {})
        if not is_enabled(name, patterns):
            continue
        if any(ignore in message.get("Message", "") for ignore in IGNORE_LIST):
            continue
        path = message.get("FilePath", "")
        short = (
            path[len(args.root) :] if args.root and path.startswith(args.root) else path
        )
        tier = tier_of(name)
        per_file[short][tier] += 1
        per_file_checks[short][name] += 1
        totals[tier] += 1
        if tier in ("P0", "P1"):
            number, _ = line_of_offset(path, int(message.get("FileOffset", 0)))
            details.append((tier, short, name, number, message.get("Message", "")))

    order = sorted(
        per_file,
        key=lambda f: (
            -per_file[f]["P0"],
            -per_file[f]["P1"],
            -sum(per_file[f].values()),
        ),
    )
    total = sum(totals.values())

    if args.md:
        print(
            f'Findings: **{total}** — P0 {totals["P0"]}, P1 {totals["P1"]}, P2 {totals["P2"]}\n'
        )
        print("| file | P0 | P1 | P2 | dominant check |")
        print("|---|---|---|---|---|")
        for f in order:
            top = per_file_checks[f].most_common(1)[0]
            print(
                f'| `{f}` | {per_file[f]["P0"]} | {per_file[f]["P1"]} | {per_file[f]["P2"]} '
                f"| {top[0]} ({top[1]}) |"
            )
        print(f"\n### P0 and P1, first {args.top}\n")
        for tier, f, name, number, msg in sorted(details)[: args.top]:
            print(f"* **{tier}** `{f}:{number}` — {msg} `[{name}]`")
    else:
        print(
            f'findings: {total}  (P0 {totals["P0"]}, P1 {totals["P1"]}, P2 {totals["P2"]})\n'
        )
        print(f'{"file":54s} {"P0":>4s} {"P1":>4s} {"P2":>5s}  dominant check')
        for f in order:
            top = per_file_checks[f].most_common(1)[0]
            print(
                f'{f[:54]:54s} {per_file[f]["P0"]:4d} {per_file[f]["P1"]:4d} {per_file[f]["P2"]:5d}'
                f"  {top[0]} ({top[1]})"
            )
        print(f"\nP0 and P1, first {args.top}:")
        for tier, f, name, number, msg in sorted(details)[: args.top]:
            print(f"  {tier}  {f}:{number}  {msg}  [{name}]")

    return 0


if __name__ == "__main__":
    sys.exit(main())
