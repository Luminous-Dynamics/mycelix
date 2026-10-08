#!/usr/bin/env python3
"""Static cross-model control alignment checker for the evidence provenance seam."""
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
    "reference_marker",
}


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--matrix", type=Path, required=True)
    p.add_argument("--tla", type=Path, required=True)
    p.add_argument("--negative-tla", type=Path, required=True)
    p.add_argument("--alloy", type=Path, required=True)
    p.add_argument("--reference", type=Path, required=True)
    args = p.parse_args()

    matrix = json.loads(args.matrix.read_text(encoding="utf-8"))
    if matrix.get("schema") != "mycelix.agent-delegation-authority-evidence-control-matrix.v1":
        raise SystemExit("AUTHORITY_MATRIX_FAIL: schema mismatch")

    controls = matrix.get("controls", [])
    if len(controls) != 1:
        raise SystemExit("AUTHORITY_MATRIX_FAIL: evidence seam must have exactly one dedicated control")

    ids = [c.get("id") for c in controls]
    if len(ids) != len(set(ids)):
        raise SystemExit("AUTHORITY_MATRIX_FAIL: duplicate control id")

    tla = args.tla.read_text(encoding="utf-8")
    negative = args.negative_tla.read_text(encoding="utf-8")
    alloy = args.alloy.read_text(encoding="utf-8")
    reference = args.reference.read_text(encoding="utf-8")

    for c in controls:
        if not REQUIRED_KEYS.issubset(c):
            raise SystemExit("AUTHORITY_MATRIX_FAIL: incomplete control entry")
        if c["tla_invariant"] not in tla:
            raise SystemExit(f"AUTHORITY_MATRIX_FAIL: missing TLA invariant: {c['tla_invariant']}")
        if c["tla_control"] not in negative:
            raise SystemExit(f"AUTHORITY_MATRIX_FAIL: missing TLA control: {c['tla_control']}")
        if c["alloy_witness"] not in alloy:
            raise SystemExit(f"AUTHORITY_MATRIX_FAIL: missing Alloy witness: {c['alloy_witness']}")
        for marker in c["reference_marker"].split(" -> "):
            if marker not in reference:
                raise SystemExit(f"AUTHORITY_MATRIX_FAIL: missing reference marker: {marker}")

    print("AUTHORITY MATRIX PASS: evidence provenance seam is represented in TLA+, Alloy, and reference artifacts")
    print("NON-AUTHORITATIVE: static alignment evidence only")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
