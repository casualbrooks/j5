#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

import yaml


def check_file(path: Path) -> bool:
    try:
        with path.open("r", encoding="utf-8") as handle:
            yaml.safe_load(handle)
    except yaml.YAMLError as exc:  # type: ignore[attr-defined]
        print(f"YAML validation failed for {path}: {exc}", file=sys.stderr)
        return False
    return True


def main(argv: list[str]) -> int:
    ok = True
    for filename in argv[1:]:
        path = Path(filename)
        if not path.exists() or path.is_dir():
            continue
        ok = check_file(path) and ok
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
