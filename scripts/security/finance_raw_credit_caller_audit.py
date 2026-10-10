#!/usr/bin/env python3
# Copyright (C) 2024-2026 Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Fail-closed caller census for removing payments::credit_sap as a public zome ABI.

Cross-zome calls are name-dispatched and can survive Rust compilation after an
extern is made private. Search all Finance coordinator sources except the
Payments coordinator itself, and require canonical/workspace mirrors to agree.
Any remaining external reference blocks qualification; do not whitelist known
callers merely to turn the check green.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CANONICAL_ZOMES = ROOT / "mycelix-finance" / "zomes"
WORKSPACE_ZOMES = ROOT / "mycelix-workspace" / "mycelix-finance" / "zomes"

RAW_CREDIT_REFERENCE = re.compile(
    r'(?:FunctionName\s*::\s*from\s*\(\s*"credit_sap"\s*\)|'
    r'"credit_sap"\s*\.into\s*\(\s*\))'
)


def scan(zomes_root: Path) -> list[tuple[str, int, str]]:
    """Return name-dispatched raw-credit references outside payments coordinator."""
    hits: list[tuple[str, int, str]] = []
    for source in sorted(zomes_root.rglob("*.rs")):
        relative = source.relative_to(zomes_root)
        parts = relative.parts

        # Private intra-coordinator helper calls are not cross-zome ABI consumers.
        if len(parts) >= 2 and parts[0:2] == ("payments", "coordinator"):
            continue
        if "coordinator" not in parts:
            continue

        try:
            lines = source.read_text(encoding="utf-8").splitlines()
        except (OSError, UnicodeError) as exc:
            raise RuntimeError(f"cannot read {source}: {exc}") from exc

        for line_number, line in enumerate(lines, start=1):
            match = RAW_CREDIT_REFERENCE.search(line)
            if match:
                hits.append((relative.as_posix(), line_number, match.group(0)))
    return hits


def main() -> int:
    missing = [
        path for path in (CANONICAL_ZOMES, WORKSPACE_ZOMES)
        if not path.is_dir()
    ]
    if missing:
        for path in missing:
            print(f"ERROR: required Finance source root missing: {path}", file=sys.stderr)
        return 2

    try:
        canonical = scan(CANONICAL_ZOMES)
        workspace = scan(WORKSPACE_ZOMES)
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    if canonical != workspace:
        print("FAIL: canonical and workspace raw-credit caller inventories differ.")
        print(f"canonical: {canonical}")
        print(f"workspace: {workspace}")
        return 1

    if canonical:
        print("FAIL: external zome callers still target raw payments::credit_sap.")
        print("Do not qualify removal of the public ABI until each path is migrated")
        print("to a source-specific authorization flow or explicitly disabled.")
        for relative, line_number, expression in canonical:
            print(f"  {relative}:{line_number}: {expression}")
        print(f"Found {len(canonical)} call site(s) in each Finance projection.")
        return 1

    print("PASS: no cross-zome raw payments::credit_sap references remain.")
    print("This source audit does not prove SAP conservation or exact-once settlement.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
