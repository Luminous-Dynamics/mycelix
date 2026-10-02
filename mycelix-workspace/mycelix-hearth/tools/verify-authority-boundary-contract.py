#!/usr/bin/env python3
"""Validate the Hearth 0.7 authority-boundary manifest against the executable fixture."""

from __future__ import annotations

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
manifest = ROOT / "tests" / "hearth-07-authority-boundary-cases.json"
rust = ROOT / "tests" / "sweettest_authority_boundary.rs"

data = json.loads(manifest.read_text(encoding="utf-8"))
cases = data["cases"]
assert data["schema_version"] == "HEARTH-AUTH-0.7-CASESET-1"
assert data["claim_ceiling"].startswith("RuntimeQualificationPending;")
assert len(cases) == 12

ids = [case["case_id"] for case in cases]
assert ids == [f"AUTH-{i:02d}" for i in range(1, 13)], ids

source = rust.read_text(encoding="utf-8")
for case in cases:
    name = case["test"]
    pattern = rf"async fn {re.escape(name)}\s*\("
    assert re.search(pattern, source), f"manifest test missing from Rust fixture: {name}"

for case in cases:
    name = case["test"]
    pattern = (
        rf"#\[tokio::test\(flavor = "multi_thread"\)\]\s*"
        rf"#\[ignore = "requires Holochain conductor \(nix develop\)"\]\s*"
        rf"async fn {re.escape(name)}\s*\("
    )
    assert re.search(pattern, source), f"qualification test lost its ignore gate: {name}"

print(f"AUTHORITY_CONTRACT_OK cases={len(cases)} schema={data['schema_version']}")
