#!/usr/bin/env python3
"""Fail when the recorded frontend contract drifts from coordinator externs."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "contracts" / "zome-api.json"
EXTERN_RE = re.compile(r"#\[hdk_extern\]\s*pub\s+fn\s+([A-Za-z_][A-Za-z0-9_]*)", re.MULTILINE)


def load_expected() -> dict[str, set[str]]:
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    return {
        zome: set(spec["functions"])
        for zome, spec in data["zomes"].items()
    }


def discover_actual() -> dict[str, set[str]]:
    actual: dict[str, set[str]] = {}
    pattern = ROOT / "backend" / "zomes"
    for source in sorted(pattern.glob("*/coordinator/src/lib.rs")):
        zome = source.parents[2].name
        functions = set(EXTERN_RE.findall(source.read_text(encoding="utf-8")))
        actual[zome] = functions
    return actual


def main() -> int:
    expected = load_expected()
    actual = discover_actual()
    failed = False

    for zome in sorted(expected.keys() | actual.keys()):
        expected_functions = expected.get(zome, set())
        actual_functions = actual.get(zome, set())
        missing = sorted(expected_functions - actual_functions)
        unexpected = sorted(actual_functions - expected_functions)

        if missing or unexpected:
            failed = True
            print(f"contract drift in zome {zome!r}", file=sys.stderr)
            if missing:
                print(f"  recorded but absent: {', '.join(missing)}", file=sys.stderr)
            if unexpected:
                print(f"  implemented but unrecorded: {', '.join(unexpected)}", file=sys.stderr)

    if failed:
        print(
            "Update the backend and contracts/zome-api.json in the same reviewed change.",
            file=sys.stderr,
        )
        return 1

    function_count = sum(len(functions) for functions in actual.values())
    print(f"zome contract matches: {len(actual)} zomes, {function_count} functions")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
