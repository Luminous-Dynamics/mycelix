#!/usr/bin/env python3
"""Verify Marketplace's Finance finality and evidence-derived reputation contract."""
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = json.loads((ROOT / "contracts/finance-reputation-integrity-v1.json").read_text())
TX = (ROOT / "backend/zomes/transactions/coordinator/src/lib.rs").read_text()
TX_INTEGRITY = (ROOT / "backend/zomes/transactions/integrity/src/lib.rs").read_text()
REP = (ROOT / "backend/zomes/reputation/coordinator/src/lib.rs").read_text()
REP_INTEGRITY = (ROOT / "backend/zomes/reputation/integrity/src/lib.rs").read_text()
ARB = (ROOT / "backend/zomes/arbitration/coordinator/src/lib.rs").read_text()
CLIENT = (ROOT / "frontend-leptos/crates/marketplace-client/src/lib.rs").read_text()
APP = (ROOT / "frontend-leptos/apps/marketplace-web/src/main.rs").read_text()
CARGO = (ROOT / "frontend-leptos/apps/marketplace-web/Cargo.toml").read_text()

errors: list[str] = []
for call in SPEC["typed_calls"]:
    function = call["function"]
    if function not in CLIENT:
        errors.append(f"typed client no longer exposes {function}")

markers = {
    "settlement_reference_uses_stable_root": (TX, 'format!("marketplace_tx:{root_transaction_hash}")'),
    "settlement_terms_derive_from_authoritative_transaction": (TX, "settlement_terms(&root_transaction_hash, &current.transaction)"),
    "marketplace_does_not_write_local_completed_finality": (TX_INTEGRITY, "Marketplace lifecycle state remains Delivered"),
    "finance_status_is_recoverable_by_reference": (TX, 'call_finance_optional_record("verify_payment_status_remote"'),
    "fulfillment_event_binds_exact_delivered_revision": (REP_INTEGRITY, "Fulfillment reputation requires an exact Delivered transaction revision"),
    "arbitration_events_bind_exact_result": (REP_INTEGRITY, "Arbitration reputation event is not bound to the declared dispute and transaction"),
    "semantic_event_keys_are_idempotent": (REP, "create_or_get_reputation_event"),
    "conflicting_duplicate_events_fail_closed": (REP, "Conflicting reputation events share key"),
    "direct_matl_mutation_is_disabled": (REP, "Direct MATL mutation is disabled"),
    "derived_reputation_is_not_an_authorization_weight": (REP, "never used as an integrity authorization weight"),
}
for invariant in SPEC["security_invariants"]:
    text, marker = markers[invariant]
    if marker not in text:
        errors.append(f"security invariant marker missing: {invariant}")

required_orchestration = [
    (TX, "project_fulfillment_reputation(delivered.transaction_hash.clone())"),
    (ARB, "project_result_reputation(result_hash.clone())"),
    (TX, 'call_finance_record("process_payment_remote"'),
]
for text, marker in required_orchestration:
    if marker not in text:
        errors.append(f"orchestration marker missing: {marker}")

feature = SPEC["ui_gate"]["feature"]
label = SPEC["ui_gate"]["label"]
if f"{feature} = []" not in CARGO:
    errors.append(f"missing opt-in Cargo feature {feature}")
if label not in APP:
    errors.append(f"settlement UI label missing: {label}")
if f'cfg!(feature = "{feature}")' not in APP:
    errors.append("settlement action is not guarded by the opt-in feature")

if errors:
    raise SystemExit("Finance/reputation contract failed:\n- " + "\n- ".join(errors))

print(
    "Finance/reputation integrity matches: "
    f"{len(SPEC['typed_calls'])} typed calls, "
    f"{len(SPEC['security_invariants'])} invariants, "
    "external Finance finality, immutable reputation events"
)
