#!/usr/bin/env python3
from __future__ import annotations

import json
import tempfile
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
SPEC = json.loads((ROOT / "contracts/deployment-promotion-v1.json").read_text())
BASE = (ROOT / SPEC["base_profile"]["manifest"]).read_text()
BRIDGE = (ROOT / "frontend-leptos/bridge/src/index.ts").read_text()
RUST_BRIDGE = (ROOT / "frontend-leptos/crates/marketplace-client/src/js_bridge.rs").read_text()
APP = (ROOT / "frontend-leptos/apps/marketplace-web/src/main.rs").read_text()
PACKAGE = json.loads((ROOT / "frontend-leptos/bridge/package.json").read_text())

errors: list[str] = []

if "- name: marketplace" not in BASE:
    errors.append("base hApp manifest no longer contains the Marketplace role")
if "- name: finance" in BASE:
    errors.append("base hApp manifest silently became settlement-capable")
for needle in [
    "summarizeRoleCapabilities",
    'activeRoles.includes("marketplace")',
    "CellType.Provisioned",
    "CellType.Cloned",
]:
    if needle not in BRIDGE:
        errors.append(f"browser role discovery missing: {needle}")
for needle in ["configured_roles", "active_roles", "bridge reported no active marketplace role"]:
    if needle not in RUST_BRIDGE:
        errors.append(f"Rust bridge role contract missing: {needle}")
for needle in [
    'has_active_role("finance")',
    "!finance_role_available",
    "connected hApp has no active Finance role",
    "Live role inventory",
    "Finance is configured but deferred or inactive",
]:
    if needle not in APP:
        errors.append(f"runtime settlement gate missing: {needle}")

client_version = PACKAGE.get("dependencies", {}).get("@holochain/client", "")
if not client_version or any(marker in client_version for marker in ["^", "~", "*", ">", "<"]):
    errors.append("@holochain/client must be pinned to an exact version")

with tempfile.TemporaryDirectory() as temp:
    temp_path = Path(temp)
    marketplace = temp_path / "marketplace.dna"
    finance = temp_path / "finance.dna"
    identity = temp_path / "identity.dna"
    for bundle in [marketplace, finance, identity]:
        bundle.write_bytes(b"fixture bundle path only")
    output = temp_path / "settlement.happ.yaml"
    subprocess.run(
        [
            "python3",
            str(ROOT / SPEC["settlement_profile"]["generator"]),
            "--marketplace-dna",
            str(marketplace),
            "--finance-dna",
            str(finance),
            "--identity-dna",
            str(identity),
            "--output",
            str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    rendered = output.read_text()
    for role_name in ["marketplace", "finance", "identity"]:
        if f"- name: {role_name}" not in rendered:
            errors.append(f"generated deployment omitted {role_name} role")
    if "deferred: false" not in rendered:
        errors.append("generated deployment has no active required roles")

if errors:
    raise SystemExit("\n".join(f"- {error}" for error in errors))

print(
    "deployment promotion contract passed: base profile is non-settling, "
    "settlement profile requires active Marketplace+Finance roles"
)
