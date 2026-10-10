#!/usr/bin/env python3
# Copyright (C) 2024-2026 Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Fail-closed caller census for removing payments::credit_sap as a public zome ABI.

Cross-zome calls are name-dispatched and can survive Rust compilation after an
extern is made private. Search all Finance coordinator sources except the
Payments coordinator itself, verify the ABI is private in both projections,
and require canonical/workspace caller inventories to agree. Any remaining
external reference blocks qualification; do not whitelist known callers merely
to turn the check green.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

# Match the exact function-name literal, independent of which Holochain
# constructor/conversion API wraps it. This intentionally errs toward a false
# positive: any external coordinator literal should be reviewed before the ABI is
# removed. It also catches Rust raw-string spellings such as r#"credit_sap"#.
RAW_CREDIT_REFERENCE = re.compile(r'(?:r#*)?"credit_sap"#*')
# Permit other outer attributes between hdk_extern and the function declaration.
# Stop at the first function declaration so an unrelated extern cannot make a later
# private helper look externally exported.
PUBLIC_RAW_CREDIT_ABI = re.compile(
    r"#\s*\[\s*hdk_extern\s*\]"
    r"(?:(?:\s*#\s*\[[^\]]*\])|(?:\s*//[^\n]*(?:\n|$))|(?:\s*/\*.*?\*/))*"
    r"\s*(?:pub\s+)?fn\s+credit_sap\s*\(",
    re.DOTALL,
)


def has_public_raw_credit_abi(source_text: str) -> bool:
    """Return whether raw credit is still declared as a Holochain extern."""
    return PUBLIC_RAW_CREDIT_ABI.search(source_text) is not None


def scan(zomes_root: Path) -> list[tuple[str, int, str]]:
    """Return exact raw-credit name literals outside payments coordinator."""
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
            source_text = source.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as exc:
            raise RuntimeError(f"cannot read {source}: {exc}") from exc

        # Search the complete source file for exact normal/raw string literals,
        # independent of constructor spelling or line breaks. This errs toward
        # false positives (including comments) so each occurrence receives review.
        for match in RAW_CREDIT_REFERENCE.finditer(source_text):
            line_number = source_text.count("\n", 0, match.start()) + 1
            hits.append((relative.as_posix(), line_number, match.group(0)))
    return hits


def audit(root: Path = ROOT) -> int:
    """Run the whole fail-closed ABI and caller audit under a project root."""
    canonical_zomes = root / "mycelix-finance" / "zomes"
    workspace_zomes = root / "mycelix-workspace" / "mycelix-finance" / "zomes"
    missing = [path for path in (canonical_zomes, workspace_zomes) if not path.is_dir()]
    if missing:
        for path in missing:
            print(f"ERROR: required Finance source root missing: {path}", file=sys.stderr)
        return 2

    # Audit the ABI itself as well as its consumers. Otherwise the caller census
    # could pass while an unrestricted raw-credit extern is accidentally restored.
    payments_coordinators = (
        root / "mycelix-finance" / "zomes" / "payments" / "coordinator" / "src" / "lib.rs",
        root / "mycelix-workspace" / "mycelix-finance" / "zomes" / "payments" / "coordinator" / "src" / "lib.rs",
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
        canonical = scan(canonical_zomes)
        workspace = scan(workspace_zomes)
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    if canonical != workspace:
        print("FAIL: canonical and workspace raw-credit caller inventories differ.")
        print(f"canonical: {canonical}")
        print(f"workspace: {workspace}")
        return 1

    if canonical:
        print("FAIL: external coordinator sources still contain the raw-credit function-name literal.")
        print("Review every occurrence; do not qualify removal of the public ABI until each")
        print("is migrated to source-specific authorization or deliberately disabled.")
        for relative, line_number, expression in canonical:
            print(f"  {relative}:{line_number}: {expression}")
        print(f"Found {len(canonical)} matching literal(s) in each Finance projection.")
        return 1

    print("PASS: raw credit ABI is private and no exact raw-credit name literal remains outside Payments coordinator.")
    print("This literal scan cannot detect dynamically constructed names and does not prove SAP conservation or exact-once settlement.")
    return 0


def main() -> int:
    return audit(ROOT)


if __name__ == "__main__":
    raise SystemExit(main())
