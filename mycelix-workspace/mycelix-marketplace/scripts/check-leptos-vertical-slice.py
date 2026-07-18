#!/usr/bin/env python3
"""Fail CI when the first Leptos purchase/recovery slice drifts from its evidence contract."""
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = json.loads((ROOT / "contracts/leptos-vertical-slice-v1.json").read_text())
CLIENT = (ROOT / "frontend-leptos/crates/marketplace-client/src/lib.rs").read_text()
APP = (ROOT / "frontend-leptos/apps/marketplace-web/src/main.rs").read_text()
TX_COORDINATOR = (ROOT / "backend/zomes/transactions/coordinator/src/lib.rs").read_text()
TX_INTEGRITY = (ROOT / "backend/zomes/transactions/integrity/src/lib.rs").read_text()

errors: list[str] = []
for step in SPEC["steps"]:
    function = step["function"]
    if function not in CLIENT:
        errors.append(f"typed client no longer exposes {function}")

for route in ('path!("/listing/:listing_id")', 'path!("/transaction/:transaction_id")'):
    if route not in APP:
        errors.append(f"missing route {route}")

required_markers = {
    "purchase_terms_resolved_from_listing": (TX_COORDINATOR, "validate_purchase_terms"),
    "transaction_create_authored_by_buyer": (TX_INTEGRITY, "authored by the buyer"),
    "transaction_core_terms_immutable": (TX_INTEGRITY, "Transaction parties, listing, quantity, price, and creation time are immutable"),
    "transaction_state_machine_validated_in_integrity_zome": (TX_INTEGRITY, "validate_status_transition"),
    "no_automatic_fixture_fallback": (APP, "No fixture data was substituted"),
}
for invariant in SPEC["security_invariants"]:
    text, marker = required_markers[invariant]
    if marker not in text:
        errors.append(f"security invariant marker missing: {invariant}")

if "Lifecycle controls remain gated" not in APP:
    reducer_markers = (
        "get_transaction_resolution" in CLIENT
        and "TransactionConflict" in APP
        and "require_resolved_transaction" in TX_COORDINATOR
    )
    if not reducer_markers:
        errors.append("deferred latest-revision gate was removed without a conflict-aware reducer")

if errors:
    raise SystemExit("Leptos vertical-slice contract failed:\n- " + "\n- ".join(errors))

print(
    "Leptos vertical slice matches: "
    f"{len(SPEC['steps'])} calls, {len(SPEC['security_invariants'])} security invariants, "
    "latest-revision gate retained or closed by the v2 conflict-aware reducer"
)
