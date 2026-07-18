#!/usr/bin/env python3
from __future__ import annotations

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = json.loads((ROOT / "contracts/promotion-artifacts-v1.json").read_text())
DNA = (ROOT / SPEC["marketplace"]["dna_manifest"]).read_text()
errors: list[str] = []

manifest_paths = set(re.findall(r"^\s*path:\s*([^#\n]+?)\s*$", DNA, re.MULTILINE))
spec_paths = {item["dna_path"] for item in SPEC["marketplace"]["zomes"]}
if manifest_paths != spec_paths:
    errors.append(
        "DNA/spec path drift: missing="
        + repr(sorted(manifest_paths - spec_paths))
        + " extra="
        + repr(sorted(spec_paths - manifest_paths))
    )

artifacts = [item["rust_artifact"] for item in SPEC["marketplace"]["zomes"]]
if len(artifacts) != len(set(artifacts)):
    errors.append("Rust artifact names must be unique")
if len(spec_paths) != 14:
    errors.append(f"expected 14 packed zomes, found {len(spec_paths)}")

for item in SPEC["marketplace"]["zomes"]:
    if not item["dna_path"].endswith(".wasm") or not item["rust_artifact"].endswith(".wasm"):
        errors.append(f"non-WASM mapping: {item}")
    if Path(item["dna_path"]).name in {"integrity.wasm", "coordinator.wasm"}:
        errors.append(f"bundle basename is not globally unique: {item['dna_path']}")

for profile, value in SPEC["profiles"].items():
    roles = set(value["required_roles"])
    if "marketplace" not in roles:
        errors.append(f"{profile} does not require Marketplace")
    if ("finance" in roles) != bool(value["requires_finance_dna"]):
        errors.append(f"{profile} Finance role/DNA requirement mismatch")

if errors:
    raise SystemExit("\n".join(f"- {error}" for error in errors))
print("promotion artifact contract passed: 14 exact zome artifacts and 4 profiles")
