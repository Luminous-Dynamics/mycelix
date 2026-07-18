#!/usr/bin/env python3
"""Verify the guarded arbitration evidence contract against source and client code."""
from __future__ import annotations

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = json.loads((ROOT / "contracts/arbitration-integrity-v1.json").read_text())
INTEGRITY = (ROOT / "backend/zomes/arbitration/integrity/src/lib.rs").read_text()
COORDINATOR = (ROOT / "backend/zomes/arbitration/coordinator/src/lib.rs").read_text()
RESOLUTION = (ROOT / "backend/zomes/arbitration/coordinator/src/resolution.rs").read_text()
TRANSACTIONS = (ROOT / "backend/zomes/transactions/coordinator/src/lib.rs").read_text()
CLIENT = (ROOT / "frontend-leptos/crates/marketplace-client/src/lib.rs").read_text()
DOMAIN = (ROOT / "frontend-leptos/crates/marketplace-domain/src/arbitration.rs").read_text()
APP = (ROOT / "frontend-leptos/apps/marketplace-web/src/main.rs").read_text()

errors: list[str] = []

for call in SPEC["typed_calls"]:
    function = call["function"]
    if function not in CLIENT:
        errors.append(f"typed client no longer exposes {call['zome']}.{function}")

markers = {
    "dispute_author_is_transaction_party": (INTEGRITY, "Dispute creation must be authored by filed_by"),
    "dispute_parties_match_authoritative_transaction": (INTEGRITY, "Dispute parties do not match the transaction"),
    "dispute_revision_conflicts_fail_closed": (RESOLUTION, "DisputeResolutionState::Conflicted"),
    "assigned_arbitrators_are_unique_and_not_parties": (INTEGRITY, "validate_arbitrator_set"),
    "votes_are_authored_by_assigned_arbitrators": (INTEGRITY, "Vote author is not assigned to this dispute"),
    "guarded_votes_have_equal_weight_one": (INTEGRITY, "arbitrator_matl_score - 1.0"),
    "result_contains_exactly_one_vote_per_assigned_arbitrator": (INTEGRITY, "Result must include exactly one vote from every assigned arbitrator"),
    "winner_ratio_and_compensation_are_recomputed": (INTEGRITY, "recommended_compensation"),
    "linked_result_retry_is_idempotent_and_duplicates_fail_closed": (COORDINATOR, "bind_result_if_needed"),
    "filing_does_not_mutate_reputation_and_finalization_projects_immutable_events": (COORDINATOR, "project_result_reputation(result_hash.clone())"),
}
for invariant in SPEC["security_invariants"]:
    text, marker = markers[invariant]
    if marker not in text:
        errors.append(f"security invariant marker missing: {invariant}")

for marker in [
    "transaction_revision_hash",
    "dispute_revision_hash",
    "vote_hashes",
    "result_hash",
]:
    if marker not in INTEGRITY or marker not in DOMAIN:
        errors.append(f"wire binding missing from integrity or typed domain: {marker}")

open_dispute_match = re.search(
    r"pub fn open_dispute\(.*?\n\}", TRANSACTIONS, flags=re.DOTALL
)
if not open_dispute_match:
    errors.append("transactions.open_dispute implementation is missing")
else:
    body = open_dispute_match.group(0)
    for forbidden in ["update_matl_score", "report_reputation", "Bridge"]:
        if forbidden in body:
            errors.append(f"open_dispute contains premature reputation side effect: {forbidden}")

if "update_matl_score" in COORDINATOR:
    errors.append("arbitration directly mutates a legacy reputation score")
if "get_agent_matl_score" in COORDINATOR or "MIN_ARBITRATOR_SCORE" in COORDINATOR:
    errors.append("arbitration still depends on a coordinator-only reputation score")
if "arbitrator_matl_score: 1.0" not in COORDINATOR:
    errors.append("coordinator no longer emits equal-weight guarded votes")
if "eligible_arbitrators.sort_by_key(ToString::to_string)" not in COORDINATOR:
    errors.append("provisional arbitrator assignment is no longer deterministic")
if "get_single_result" not in COORDINATOR:
    errors.append("unique-result read boundary is missing")

# The UI gate must stay visible until live conductor evidence exists.
if 'disabled=true>"Open dispute"' not in APP:
    errors.append("Leptos dispute control is no longer statically gated")

if errors:
    raise SystemExit("Arbitration integrity contract failed:\n- " + "\n- ".join(errors))

print(
    "arbitration integrity matches: "
    f"{len(SPEC['typed_calls'])} typed calls, "
    f"{len(SPEC['security_invariants'])} invariants, "
    f"{len(SPEC['deliberate_limits'])} explicit limits"
)
