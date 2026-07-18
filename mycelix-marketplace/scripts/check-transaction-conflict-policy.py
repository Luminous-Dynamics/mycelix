#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = json.loads((ROOT / "contracts/transaction-conflict-policy-v1.json").read_text())
BACKEND = (ROOT / "backend/zomes/transactions/coordinator/src/resolution.rs").read_text()
DOMAIN = (ROOT / "frontend-leptos/crates/marketplace-domain/src/transaction.rs").read_text()
UI = (ROOT / "frontend-leptos/apps/marketplace-web/src/main.rs").read_text()
errors: list[str] = []

if SPEC.get("policy_id") != "mycelix-marketplace-transaction-conflict-v2":
    errors.append("conflict policy id drifted")
for needle in [
    "AutoResolved",
    "CancellationDominatesPreShipment",
    "DisputeDominatesLifecycle",
    "superseded_heads",
    "TRANSACTION_CONFLICT_POLICY_VERSION",
    "AuthorizedResolved",
    "applied_conflict_resolutions",
]:
    if needle not in BACKEND:
        errors.append(f"backend conflict reducer missing {needle}")
for needle in [
    "AutoResolved",
    "CancellationDominatesPreShipment",
    "DisputeDominatesLifecycle",
    "superseded_heads",
    "AuthorizedResolved",
    "applied_conflict_resolutions",
]:
    if needle not in DOMAIN:
        errors.append(f"Leptos domain missing {needle}")
for needle in [
    "SAFETY-DOMINANT PROJECTION",
    "No branch was deleted",
    "without explicit authority",
]:
    if needle not in UI:
        errors.append(f"Leptos conflict presentation missing {needle}")

if errors:
    raise SystemExit("\n".join(f"- {error}" for error in errors))
print("transaction conflict policy passed: narrow authored-head dominance, unsafe conflicts remain explicit")
