#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path


def ensure_newline(path: Path) -> bool:
    text = path.read_text()
    if text and not text.endswith("\n"):
        path.write_text(text + "\n")
        return True
    return False


def main(argv: list[str]) -> int:
    changed_any = False
    for filename in argv[1:]:
        path = Path(filename)
        if not path.exists() or path.is_dir():
            continue
        if ensure_newline(path):
            changed_any = True
    return int(changed_any)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
