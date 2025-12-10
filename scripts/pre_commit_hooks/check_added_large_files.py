#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

MAX_BYTES = 500_000


def is_too_large(path: Path) -> bool:
    try:
        return path.stat().st_size > MAX_BYTES
    except OSError:
        return False


def main(argv: list[str]) -> int:
    too_large: list[tuple[Path, int]] = []
    for filename in argv[1:]:
        path = Path(filename)
        if not path.exists() or path.is_dir():
            continue
        if is_too_large(path):
            too_large.append((path, path.stat().st_size))
    if too_large:
        for path, size in too_large:
            print(
                f"{path} is {size} bytes which exceeds {MAX_BYTES} bytes",
                file=sys.stderr,
            )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
