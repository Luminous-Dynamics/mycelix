#!/usr/bin/env python3
"""Fail if binding governance packages more than one constitutional authority plane."""

from __future__ import annotations

from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
DNA = ROOT / "dna" / "dna.yaml"
text = DNA.read_text(encoding="utf-8")

errors: list[str] = []

# The binding DNA must carry the immutable authority path.
for required in (
    "- name: constitution_authority_integrity",
    "- name: constitution_authority",
    "governance_constitution: null",
):
    if required not in text:
        errors.append(f"missing required constitutional authority marker: {required!r}")

# The legacy mutable plane must never be packaged in this DNA.
for forbidden in (
    r"(?m)^\s*- name: constitution_integrity\s*$",
    r"(?m)^\s*- name: constitution\s*$",
    r"constitution_integrity\.wasm",
    r"(?<!authority_)constitution\.wasm",
):
    if re.search(forbidden, text):
        errors.append(f"legacy mutable constitution is packaged: /{forbidden}/")

# Do not permit source control to silently bless a concrete production
# constitution. Deployments must inject an exact manifest and therefore produce
# their own DNA hash/network identity.
if not re.search(r"(?m)^\s*governance_constitution:\s*null\s*$", text):
    errors.append(
        "checked-in binding DNA must leave governance_constitution null; "
        "deployment-specific manifests belong in the packing/release input"
    )

if errors:
    print("CONSTITUTION QUARANTINE: FAIL", file=sys.stderr)
    for error in errors:
        print(f" - {error}", file=sys.stderr)
    raise SystemExit(1)

print("CONSTITUTION QUARANTINE: PASS")
print(" - DNA-bound constitution_authority is packaged")
print(" - legacy mutable constitution zomes are absent")
print(" - source-controlled genesis manifest remains unprovisioned")
