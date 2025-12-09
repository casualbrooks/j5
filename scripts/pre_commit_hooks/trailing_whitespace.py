#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path


def strip_trailing_whitespace(path: Path) -> bool:
    original = path.read_text().splitlines(keepends=True)

    def clean_line(line: str) -> str:
        has_newline = line.endswith("\n")
        trimmed = line.rstrip(" \t\r\n")
        return f"{trimmed}\n" if has_newline else trimmed

    cleaned = [clean_line(line) for line in original]

    if cleaned != original:
        path.write_text("".join(cleaned))
        return True
    return False


def main(argv: list[str]) -> int:
    changed_any = False
    for filename in argv[1:]:
        path = Path(filename)
        if not path.exists() or path.is_dir():
            continue
        if strip_trailing_whitespace(path):
            changed_any = True
    return int(changed_any)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
