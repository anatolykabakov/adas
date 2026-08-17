#!/usr/bin/env python3
"""Turn a clang-tidy `-export-fixes` YAML into a readable report, honouring the config's globs.

Adapted from `adas_dev_calibrations/utils/tidy_report_handler.py`. Two changes, both about the report
telling the truth:

* **glob matching.** The original split `Checks` on `", "` and tested `check in DiagnosticName` as a
  substring, so every entry containing `*` — that is, all the wildcard families — matched nothing.
  On this repository that turned 1485 diagnostics into 3. Checks are now matched the way clang-tidy
  matches them: a comma-separated list of globs, `-` prefix excludes, and the **last** pattern that
  matches a name decides.
* **byte offsets.** `FileOffset` is a byte offset; the original counted characters, so any non-ASCII
  byte earlier in the file shifted every line number after it. Files are read as bytes now.

Output is one JSON object per finding on stdout (as before), a summary by check on stderr, and exit
code 1 when anything was reported — so a CI job fails on findings and the artifact stays parseable.
"""

from __future__ import annotations

import argparse
import fnmatch
import json
import os.path
import re
import sys
from collections import Counter

import yaml

# Findings whose message mentions any of these are dropped: third-party headers and test macros that
# no amount of local editing can fix.
IGNORE_LIST = [
    "Geometry",
    "Core",
    "pose3.hpp",
    "Eigen",
    "is not used directly",
    "TEST_F",
    "MOCK_METHOD",
    "TEST",
]


def parse_checks(checks: str) -> list[tuple[bool, re.Pattern]]:
    """`Checks` string → ordered [(is_exclude, compiled glob)], in config order."""
    out: list[tuple[bool, re.Pattern]] = []
    for raw in checks.replace("\n", ",").split(","):
        token = raw.strip()
        if not token or token.startswith("#"):
            continue
        exclude = token.startswith("-")
        if exclude:
            token = token[1:]
        if not token:
            continue
        out.append((exclude, re.compile(fnmatch.translate(token))))
    return out


def is_enabled(name: str, patterns: list[tuple[bool, re.Pattern]]) -> bool:
    """clang-tidy semantics: the last pattern that matches decides.

    A diagnostic may carry several aliases (`bugprone-x,cppcoreguidelines-x`); it counts as enabled
    when any alias is, which is what clang-tidy itself reports.
    """
    for alias in name.split(","):
        alias = alias.strip()
        enabled = False
        for exclude, pattern in patterns:
            if pattern.match(alias):
                enabled = not exclude
        if enabled:
            return True
    return False


def line_of_offset(path: str, offset: int) -> tuple[int, str]:
    """(1-based line number, line text) for a byte offset."""
    try:
        with open(path, "rb") as handle:
            data = handle.read()
    except OSError:
        return 0, ""
    offset = max(0, min(offset, len(data)))
    number = data.count(b"\n", 0, offset) + 1
    start = data.rfind(b"\n", 0, offset) + 1
    end = data.find(b"\n", offset)
    if end < 0:
        end = len(data)
    return number, data[start:end].decode("utf-8", "replace")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", help="clang-tidy YAML from -export-fixes")
    parser.add_argument("config", help=".clang-tidy whose Checks decide what is reported")
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="print only the per-check counts, no JSON bodies",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.report) or os.stat(args.report).st_size == 0:
        return 0

    with open(args.config, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    patterns = parse_checks(config["Checks"])

    with open(args.report, "r", encoding="utf-8") as handle:
        report = yaml.safe_load(handle)

    counts: Counter[str] = Counter()
    skipped_by_config = Counter()
    skipped_by_ignore = 0
    reported = 0

    for entry in report.get("Diagnostics", []):
        name = entry.get("DiagnosticName", "")
        message = entry.get("DiagnosticMessage", {})
        if not is_enabled(name, patterns):
            skipped_by_config[name] += 1
            continue
        if any(ignore in message.get("Message", "") for ignore in IGNORE_LIST):
            skipped_by_ignore += 1
            continue

        path = message.get("FilePath", "")
        number, text = line_of_offset(path, int(message.get("FileOffset", 0)))
        message["FilePath"] = f"{path}:{number}"
        entry["Line"] = text
        entry["LineNumber"] = number

        counts[name] += 1
        reported += 1
        if not args.summary_only:
            print(json.dumps(entry, indent=4))

    print(f"\nfindings: {reported}", file=sys.stderr)
    for name, count in counts.most_common():
        print(f"  {count:5d}  {name}", file=sys.stderr)
    if skipped_by_config:
        print(f"disabled by config: {sum(skipped_by_config.values())}", file=sys.stderr)
    if skipped_by_ignore:
        print(f"dropped by IGNORE_LIST: {skipped_by_ignore}", file=sys.stderr)

    return 1 if reported else 0


if __name__ == "__main__":
    sys.exit(main())
