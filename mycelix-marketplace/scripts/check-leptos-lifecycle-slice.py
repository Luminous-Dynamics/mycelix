#!/usr/bin/env python3
"""Verify the conflict-aware Leptos transaction lifecycle evidence contract."""
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = json.loads((ROOT / "contracts/leptos-lifecycle-slice-v2.json").read_text())
CLIENT = (ROOT / "frontend-leptos/crates/marketplace-client/src/lib.rs").read_text()
APP = (ROOT / "frontend-leptos/apps/marketplace-web/src/main.rs").read_text()
TX_COORDINATOR = (ROOT / "backend/zomes/transactions/coordinator/src/lib.rs").read_text()
TX_RESOLUTION = (ROOT / "backend/zomes/transactions/coordinator/src/resolution.rs").read_text()
TX_INTEGRITY = (ROOT / "backend/zomes/transactions/integrity/src/lib.rs").read_text()

errors: list[str] = []
for step in SPEC["read_calls"] + SPEC["enabled_mutations"]:
    function = step["function"]
    if function not in CLIENT:
        errors.append(f"typed client no longer exposes {function}")

markers = {
    "stable_create_action_identity": (TX_RESOLUTION, "find_root"),
    "single_leaf_required_for_mutation": (TX_COORDINATOR, "require_resolved_transaction"),
    "concurrent_heads_surface_as_conflict": (TX_RESOLUTION, "TransactionResolutionState::Conflicted"),
    "deleted_transaction_revisions_rejected": (TX_INTEGRITY, "permanent audit evidence and cannot be deleted"),
    "actor_and_transition_validation_remain_in_integrity_zome": (TX_INTEGRITY, "validate_status_transition"),
    "post_mutation_resolution_refresh": (APP, "get_transaction_resolution(&root).await"),
}
for invariant in SPEC["security_invariants"]:
    text, marker = markers[invariant]
    if marker not in text:
        errors.append(f"security invariant marker missing: {invariant}")

ui_markers = [
    "TransactionConflict",
    "LifecycleCommand::Confirm",
    "LifecycleCommand::Ship",
    "LifecycleCommand::Deliver",
    "LifecycleCommand::Cancel",
    "further actions are halted",
]
for marker in ui_markers:
    if marker not in APP:
        errors.append(f"lifecycle UI marker missing: {marker}")

for deferred in SPEC["deferred_mutations"]:
    function = deferred["function"]
    if function == "settle_transaction":
        if "Settle with Finance" not in APP:
            errors.append("settlement gate is no longer visible")
        if 'cfg!(feature = "finance-settlement")' not in APP:
            errors.append("settlement control is not guarded by the finance-settlement feature")
    if function == "dispute_transaction" and "Open dispute" not in APP:
        errors.append("dispute gate is no longer visible")

if 'disabled=true>"Open dispute"' not in APP:
    errors.append("dispute control is not statically disabled")

if errors:
    raise SystemExit("Leptos lifecycle contract failed:\n- " + "\n- ".join(errors))

print(
    "Leptos lifecycle slice matches: "
    f"{len(SPEC['read_calls'])} conflict-aware reads, "
    f"{len(SPEC['enabled_mutations'])} guarded mutations, "
    f"{len(SPEC['security_invariants'])} invariants, "
    f"{len(SPEC['deferred_mutations'])} explicit gates"
)
