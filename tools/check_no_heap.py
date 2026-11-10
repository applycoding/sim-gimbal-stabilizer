#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Token-aware lint: fail on heap containers/allocation in core paths."""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CORE_DIRS = [ROOT / "cpp/src/core"]
CORE_HEADERS = [p for p in (ROOT / "cpp/include/sgs").glob("*.hpp")]
FORBIDDEN = [
    r"<vector>",
    r"<string>",
    r"<memory>",
    r"\bnew\b",
    r"\bmalloc\b",
    r"\bcalloc\b",
    r"\brealloc\b",
    r"std::vector",
    r"std::string",
]
ALLOWLIST: set[str] = set()


def strip_comments_and_strings(text: str) -> str:
    out: list[str] = []
    i = 0
    while i < len(text):
        if text.startswith("//", i):
            while i < len(text) and text[i] != "\n":
                i += 1
            continue
        if text.startswith("/*", i):
            end = text.find("*/", i + 2)
            i = len(text) if end < 0 else end + 2
            continue
        if text[i] in "\"'":
            q = text[i]
            i += 1
            while i < len(text) and text[i] != q:
                if text[i] == "\\":
                    i += 1
                i += 1
            i += 1
            continue
        out.append(text[i])
        i += 1
    return "".join(out)


def main() -> int:
    failed = False
    paths = list(CORE_HEADERS)
    for d in CORE_DIRS:
        paths.extend(sorted(d.rglob("*")))
    for path in paths:
            if path.suffix not in {".hpp", ".cpp", ".h"}:
                continue
            rel = str(path.relative_to(ROOT))
            if rel in ALLOWLIST:
                continue
            text = strip_comments_and_strings(path.read_text(encoding="utf-8"))
            for pat in FORBIDDEN:
                if re.search(pat, text):
                    print(f"FORBIDDEN {pat!r} in {rel}")
                    failed = True
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
