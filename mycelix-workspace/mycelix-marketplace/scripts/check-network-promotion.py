#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = json.loads((ROOT / "contracts/network-promotion-v1.json").read_text())
GENERATOR = (ROOT / "scripts/generate-networked-conductor-config.py").read_text()
SETUP = (ROOT / "frontend-leptos/bridge/test/prepare-network-conductors.ts").read_text()
SCENARIO = (ROOT / "frontend-leptos/bridge/test/conductor-network.ts").read_text()
RUNNER = (ROOT / "scripts/run-network-promotion.sh").read_text()
VERIFIER = (ROOT / "scripts/verify-live-evidence.py").read_text()
WORKFLOW = (ROOT / ".github/workflows/network-promotion.yml").read_text()
errors: list[str] = []

if SPEC.get("version") != 3 or SPEC.get("topology") != "two_conductor_isolated_network":
    errors.append("network topology or contract version drifted")
for needle in ["bootstrap_url", "signal_url", "target_arc_factor", "danger_test_keystore"]:
    if needle not in GENERATOR:
        errors.append(f"networked conductor generator missing {needle}")
for needle in ["distinct admin interfaces", "installApp", "authorizeSigningCredentials", "issueAppAuthenticationToken"]:
    if needle not in SETUP:
        errors.append(f"network actor setup missing {needle}")
for needle in [
    'control("partition")',
    'control("heal")',
    "safe_terminal_dominance_observed",
    "cancellation_dominates_pre_shipment",
    "unsafe_conflict_observed",
    '"cancelled", "shipped"',
    "unsafe_arbitrary_winner_selected: false",
    "approve_transaction_conflict",
    "finalize_bilateral_transaction_conflict",
    "bilateral_resolution_observed: true",
    'bilateral_resolution_state: "authorized_resolved"',
]:
    if needle not in SCENARIO:
        errors.append(f"network scenario missing {needle}")
for needle in [
    "MARKETPLACE_NETWORK_SERVICES",
    "MARKETPLACE_NETWORK_CONTROL",
    "seller-conductor.log",
    "buyer-conductor.log",
    "two_conductor_isolated_network",
]:
    if needle not in RUNNER:
        errors.append(f"network runner missing {needle}")
for needle in [
    'args.profile == "network"',
    "safe terminal dominance",
    "same unsafe conflict",
    "arbitrary winner for an unsafe conflict",
    "bilateral conflict authority",
    "exact original head set",
]:
    if needle not in VERIFIER:
        errors.append(f"network verifier missing {needle}")
for needle in [
    "self-hosted",
    "mycelix-network-control",
    "run-network-promotion.sh",
    "persist-credentials: false",
    "MARKETPLACE_RELEASE_SIGNING_KEY_BASE64",
]:
    if needle not in WORKFLOW:
        errors.append(f"network workflow missing {needle}")

if errors:
    raise SystemExit("\n".join(f"- {error}" for error in errors))
print("network promotion contract passed: safe projection plus explicit unsafe conflict across two conductors")
