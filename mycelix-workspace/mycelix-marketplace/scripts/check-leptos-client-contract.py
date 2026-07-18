#!/usr/bin/env python3
"""Verify that every Leptos client constant names a real coordinator extern."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "contracts" / "zome-api.json"
CONTRACT_RS = (
    ROOT
    / "frontend-leptos"
    / "crates"
    / "marketplace-client"
    / "src"
    / "contract.rs"
)
MODULE_RE = re.compile(r"^pub mod ([a-z_]+) \{$")
CONST_RE = re.compile(r'^\s*pub const ([A-Z0-9_]+): &str = "([a-z_]+)";$')


def main() -> int:
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    expected = {
        zome: set(spec["functions"])
        for zome, spec in data["zomes"].items()
    }

    current_module: str | None = None
    failures: list[str] = []
    checked = 0

    for raw_line in CONTRACT_RS.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        module_match = MODULE_RE.match(line)
        if module_match:
            current_module = module_match.group(1)
            if current_module not in expected:
                failures.append(f"unknown zome module {current_module!r}")
            continue
        if line == "}":
            current_module = None
            continue

        constant_match = CONST_RE.match(raw_line)
        if not constant_match or current_module is None:
            continue
        name, value = constant_match.groups()
        if name == "ZOME":
            if value != current_module:
                failures.append(
                    f"module {current_module!r} declares ZOME={value!r}"
                )
            continue

        checked += 1
        if value not in expected.get(current_module, set()):
            failures.append(
                f"{current_module}.{name} points to absent extern {value!r}"
            )

    if failures:
        for failure in failures:
            print(f"client contract error: {failure}", file=sys.stderr)
        return 1

    print(f"Leptos client contract matches backend manifest: {checked} calls")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
