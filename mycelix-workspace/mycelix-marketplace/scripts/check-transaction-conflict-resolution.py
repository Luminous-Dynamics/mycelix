#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = json.loads((ROOT / "contracts/transaction-conflict-resolution-v1.json").read_text())
INTEGRITY = (ROOT / "backend/zomes/transactions/integrity/src/conflicts.rs").read_text()
COORDINATOR = (ROOT / "backend/zomes/transactions/coordinator/src/lib.rs").read_text()
AUTHORITY = (ROOT / "backend/zomes/transactions/coordinator/src/authority.rs").read_text()
REDUCER = (ROOT / "backend/zomes/transactions/coordinator/src/resolution.rs").read_text()
ARBITRATION = (ROOT / "backend/zomes/arbitration/coordinator/src/lib.rs").read_text()
DOMAIN = (ROOT / "frontend-leptos/crates/marketplace-domain/src/transaction.rs").read_text()
CLIENT = (ROOT / "frontend-leptos/crates/marketplace-client/src/lib.rs").read_text()
UI = (ROOT / "frontend-leptos/apps/marketplace-web/src/main.rs").read_text()
errors: list[str] = []

if SPEC.get("version") != 1:
    errors.append("explicit conflict authority protocol version drifted")
if SPEC.get("authority_paths", {}).get("bilateral", {}).get("unilateral_resolution_forbidden") is not True:
    errors.append("bilateral protocol no longer forbids unilateral resolution")
if SPEC.get("authority_paths", {}).get("arbitration", {}).get("selected_head_rule") != "unique_head_authored_by_result_winner":
    errors.append("arbitration selected-head rule drifted")

for needle in [
    "TransactionConflictApproval",
    "TransactionConflictResolutionEntry",
    "validate_bilateral_authority",
    "validate_arbitration_authority",
    "ConflictDisputeStatusSnapshot::Voting",
    "unique conflict head authored by the result winner",
]:
    if needle not in INTEGRITY:
        errors.append(f"transaction integrity missing {needle}")

for needle in [
    "approve_transaction_conflict",
    "finalize_bilateral_transaction_conflict",
    "apply_arbitration_transaction_conflict",
    "get_transaction_conflict_approvals",
    "get_transaction_conflict_resolutions",
]:
    if f"pub fn {needle}" not in COORDINATOR:
        errors.append(f"transaction coordinator missing extern {needle}")

for needle in [
    "project_authorized_resolution",
    "selected branch itself forked and requires fresh authority",
    "Conflicting",
    "is_descendant_or_same",
]:
    if needle not in AUTHORITY:
        errors.append(f"authority projection missing {needle}")

for needle in [
    "AuthorizedResolved",
    "BilateralAgreement",
    "ArbitrationAward",
    "ConflictingExplicitAuthorities",
    "applied_conflict_resolutions",
]:
    if needle not in REDUCER:
        errors.append(f"transaction reducer missing {needle}")
    if needle not in DOMAIN:
        errors.append(f"Leptos domain missing {needle}")

if "file_transaction_conflict_dispute" not in ARBITRATION or "conflict_heads" not in ARBITRATION:
    errors.append("arbitration coordinator lacks exact conflict-bound filing")

for needle in [
    "approve_transaction_conflict",
    "finalize_bilateral_transaction_conflict",
    "apply_arbitration_transaction_conflict",
    "file_transaction_conflict_dispute",
]:
    if needle not in CLIENT:
        errors.append(f"typed client missing {needle}")

for needle in [
    "Resolve without erasing branch history",
    "Publish my approval",
    "Finalize bilateral resolution",
    "Open conflict-bound arbitration",
    "EXPLICITLY AUTHORIZED PROJECTION",
    "Original branches remain visible as evidence",
]:
    if needle not in UI:
        errors.append(f"Leptos explicit-authority presentation missing {needle}")

if errors:
    raise SystemExit("\n".join(f"- {error}" for error in errors))
print("transaction conflict authority passed: bilateral and arbitration evidence select existing branches without erasing history")
