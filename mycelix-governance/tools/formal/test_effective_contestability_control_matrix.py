#!/usr/bin/env python3
"""Fail-closed static checker for cross-model contestability control alignment."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

REQUIRED_KEYS = {
    "id",
    "boundary",
    "tla_invariant",
    "tla_control",
    "alloy_witness",
    "alloy_mutant_fact",
    "reference_marker",
}

def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))

def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--matrix", type=Path, required=True)
    p.add_argument("--tla", type=Path, required=True)
    p.add_argument("--negative-tla", type=Path, required=True)
    p.add_argument("--alloy", type=Path, required=True)
    p.add_argument("--reference", type=Path, required=True)
    args = p.parse_args()

    matrix = load(args.matrix)
    if matrix.get("schema") != "mycelix.effective-contestability-control-matrix.v1":
        raise SystemExit("CONTROL_MATRIX_FAIL: schema mismatch")
    controls = matrix.get("controls", [])
    if not controls:
        raise SystemExit("CONTROL_MATRIX_FAIL: no controls")

    ids = [c.get("id") for c in controls]
    if len(ids) != len(set(ids)):
        raise SystemExit("CONTROL_MATRIX_FAIL: duplicate control id")
    if any(not REQUIRED_KEYS.issubset(c) for c in controls):
        raise SystemExit("CONTROL_MATRIX_FAIL: incomplete control entry")

    tla = args.tla.read_text(encoding="utf-8")
    negative = args.negative_tla.read_text(encoding="utf-8")
    alloy = args.alloy.read_text(encoding="utf-8")
    reference = args.reference.read_text(encoding="utf-8")

    seen_tla_controls = set()
    for c in controls:
        invariant = c["tla_invariant"]
        control = c["tla_control"]
        witness = c["alloy_witness"]
        mutant = c["alloy_mutant_fact"]
        marker = c["reference_marker"]

        if invariant not in tla:
            raise SystemExit(f"CONTROL_MATRIX_FAIL: missing TLA invariant: {invariant}")
        if control not in negative:
            raise SystemExit(f"CONTROL_MATRIX_FAIL: missing TLA control: {control}")
        if control in seen_tla_controls:
            raise SystemExit(f"CONTROL_MATRIX_FAIL: TLA control reused: {control}")
        seen_tla_controls.add(control)

        if witness not in alloy:
            raise SystemExit(f"CONTROL_MATRIX_FAIL: missing Alloy witness: {witness}")
        if f"fact {mutant} " not in alloy:
            raise SystemExit(f"CONTROL_MATRIX_FAIL: missing Alloy mutant fact: {mutant}")
        ref_parts = [part.strip() for part in marker.split("->", 1)]
        if not ref_parts[0] or ref_parts[0] not in reference:
            raise SystemExit(f"CONTROL_MATRIX_FAIL: missing reference control: {ref_parts[0]}")
        if len(ref_parts) == 2:
            ref_target = ref_parts[1].split(":", 1)[0].strip()
            if ref_target and ref_target not in reference:
                raise SystemExit(f"CONTROL_MATRIX_FAIL: missing reference target: {ref_target}")

    print(f"CONTROL MATRIX PASS: {len(controls)} semantic controls are represented across TLA, Alloy, and reference artifacts")
    print("NON-AUTHORITATIVE: static alignment evidence only")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
