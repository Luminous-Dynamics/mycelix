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
PUBLIC_RAW_CREDIT_ABI = re.compile(
    r"#\s*\[\s*hdk_extern\s*\]\s*(?:pub\s+)?fn\s+credit_sap\s*\("
)


def has_public_raw_credit_abi(source_text: str) -> bool:
    """Return whether raw credit is still declared as a Holochain extern."""
    return PUBLIC_RAW_CREDIT_ABI.search(source_text) is not None


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

        # Search the complete source file: string-dispatched zome calls can be
        # formatted across several lines. A line-at-a-time search silently misses
        # those calls and could let a private-extern migration appear complete.
        source_text = "\n".join(lines)
        for match in RAW_CREDIT_REFERENCE.finditer(source_text):
            line_number = source_text.count("\n", 0, match.start()) + 1
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

    # Audit the ABI itself as well as its consumers. Otherwise the caller census
    # could pass while an unrestricted raw-credit extern is accidentally restored.
    payments_coordinators = (
        ROOT / "mycelix-finance" / "zomes" / "payments" / "coordinator" / "src" / "lib.rs",
        ROOT / "mycelix-workspace" / "mycelix-finance" / "zomes" / "payments" / "coordinator" / "src" / "lib.rs",
    )
    for path in payments_coordinators:
        try:
            source_text = path.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as exc:
            print(f"ERROR: cannot read required Payments coordinator {path}: {exc}", file=sys.stderr)
            return 2
        if has_public_raw_credit_abi(source_text):
            print(f"FAIL: raw payments::credit_sap remains exposed as a Holochain extern: {path}")
            print("Remove the extern only with its source-specific caller migration and runtime coverage.")
            return 1

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
