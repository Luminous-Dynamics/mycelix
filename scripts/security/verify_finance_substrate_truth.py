#!/usr/bin/env python3
"""Verify Finance substrate declarations against the authoritative matrix.

The check is deliberately split:
  --mode current: prove the two public Finance trees declare the same substrate.
  --mode target: refuse qualification unless their declarations equal the target matrix.

It does not infer runtime Holochain from a Holonix git ref. Runtime identity must
be measured with `holochain --build-info`/`hn-introspect` in the qualification runner.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MATRIX = ROOT / "docs/security/finance-substrate-matrix.json"
FINANCE_TOMLS = [
    ROOT / "mycelix-finance/Cargo.toml",
    ROOT / "mycelix-workspace/mycelix-finance/Cargo.toml",
]
FINANCE_FLAKES = [
    ROOT / "mycelix-finance/flake.nix",
    ROOT / "mycelix-workspace/flake.nix",
]

CARGO_KEYS = (
    "hdk", "hdi", "holochain_zome_types",
    "holochain_integrity_types", "holo_hash", "hdk_derive",
    "holochain_serialized_bytes",
)

def cargo_value(text: str, key: str) -> str | None:
    m = re.search(rf"^\s*{re.escape(key)}\s*=\s*=?\"([^\"]+)\"", text, re.M)
    return m.group(1).lstrip('=') if m else None

def holonix_ref(text: str) -> str | None:
    m = re.search(r'url\s*=\s*"github:holochain/holonix/([^\"]+)"', text)
    return m.group(1) if m else None

def load_matrix() -> dict:
    return json.loads(MATRIX.read_text(encoding="utf-8"))

def declared_cargo(path: Path) -> dict[str, str | None]:
    text = path.read_text(encoding="utf-8")
    return {k: cargo_value(text, k) for k in CARGO_KEYS}

def declared_holonix(path: Path) -> str | None:
    return holonix_ref(path.read_text(encoding="utf-8"))

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=("current", "target"), default="current")
    args = ap.parse_args()
    matrix = load_matrix()
    observed_cargo = [declared_cargo(p) for p in FINANCE_TOMLS]
    observed_holonix = [declared_holonix(p) for p in FINANCE_FLAKES]
    errors: list[str] = []

    if any(v is None for row in observed_cargo for v in row.values()):
        errors.append("missing required Finance Cargo substrate dependency pin")
    if any(v is None for v in observed_holonix):
        errors.append("missing Holonix git ref in a Finance flake")

    if observed_cargo[0] != observed_cargo[1]:
        errors.append(f"Finance Cargo substrate declarations diverge: {observed_cargo}")
    if observed_holonix[0] != observed_holonix[1]:
        errors.append(f"Finance Holonix declarations diverge: {observed_holonix}")

    if args.mode == "target":
        target = matrix["target"]
        expected = {
            "hdk": target["hdk"],
            "hdi": target["hdi"],
            "holochain_zome_types": target["holochain"],
            "holochain_integrity_types": target["holochain"],
            "holo_hash": target["holochain"],
            "hdk_derive": target["holochain"],
            "holochain_serialized_bytes": matrix["current_declaration"]["cargo"]["holochain_serialized_bytes"],
        }
        if observed_cargo[0] != expected:
            errors.append(
                "Finance Cargo declarations do not match the Holochain 0.7 target: "
                + f"observed={observed_cargo[0]} expected={expected}"
            )
        errors.append(
            "runtime Holochain identity is not measured by this static check; "
            "qualification must supply build-info evidence"
        )

    print(f"FINANCE_SUBSTRATE_MATRIX_MODE={args.mode}")
    print(f"FINANCE_SUBSTRATE_MATRIX_STATUS={'FAIL' if errors else 'PASS'}")
    for i, row in enumerate(observed_cargo, 1):
        print(f"cargo_tree_{i}={row}")
    print(f"holonix_refs={observed_holonix}")
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        return 1
    return 0

if __name__ == "__main__":
    raise SystemExit(main())