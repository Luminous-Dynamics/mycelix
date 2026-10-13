#!/usr/bin/env python3
"""Fail-closed validator for CIV-RES-002A0 current-main owner census v1.

This validates a frozen source census and synthetic semantic controls. It does
not inspect a live City API, process resident data, or qualify deployment.
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
DOC = ROOT / "mycelix-workspace/docs/civic-resilience/CIV_RES_002A0_SERVICE_REQUEST_OWNER_CENSUS_V1.md"
MANIFEST = ROOT / "mycelix-workspace/docs/civic-resilience/civ_res_002a0_owner_census.json"
EXPECTED_MAIN = "f16c41b5264c632ca3efa32c78b37bfc21bcfe68"
VOCABULARY = [
    "AuthoritativeOwner", "OperationalOwner", "ProcedureOwner", "EvidenceOwner",
    "ExternalSourceAdapter", "PresentationOnly", "HistoricalOrSuperseded",
    "NotApplicable", "Missing",
]
EXPECTED_CANDIDATES = {
    "commons_support_ticket": ("OperationalOwner", "mycelix-commons/zomes/support-tickets/integrity/src/lib.rs", "c66ff344f6f3c0a43533d9370c887f2f200619fd"),
    "commons_support_coordinator": ("OperationalOwner", "mycelix-commons/zomes/support-tickets/coordinator/src/lib.rs", "e04fdd61227aa485a801a9e2dfe39087d3953e7d"),
    "commons_support_types": ("OperationalOwner", "mycelix-commons/crates/support-types/src/lib.rs", "bfb405e6d8f9f0afbc30cf33fabbfbe1f2197654"),
    "civic_evidence_types": ("EvidenceOwner", "mycelix-civic/crates/civic-types/src/evidence.rs", "f128a6c4bdab409208ddb4c26127736f03101c8e"),
    "justice_case_model": ("NotApplicable", "mycelix-civic/zomes/justice-cases/integrity/src/lib.rs", "041b56c7d21ab1cfc92d3a72363c8d3c817ee2f6"),
    "commons_bridge": ("NotApplicable", "mycelix-commons/zomes/commons-bridge/coordinator/src/lib.rs", "93f0fa08937637f0441308c43cffe295a2e6935d"),
    "community_water_ui": ("PresentationOnly", "mycelix-commons/apps/leptos/src/pages/water.rs", "a8e9007b7f26b4ae11f6e8fece3ee2dfbe8dd9bd"),
    "water_resource_model": ("NotApplicable", "mycelix-workspace/mycelix-water/zomes/capture/integrity/src/lib.rs", "5d926aab5171cffff3e4f5eb2ffef4bcabf8af79"),
    "manufacturing_work_order": ("NotApplicable", "mycelix-workspace/mycelix-manufacturing/zomes/workorders/integrity/src/lib.rs", "ae10f3f85613f83ab4ac31cd059e8041032ba3b8"),
    "municipal_service_issue_owner": ("Missing", None, None),
    "city_provider_adapter": ("Missing", None, None),
    "service_level_registry": ("Missing", None, None),
    "municipal_completion_verifier": ("Missing", None, None),
    "administrative_procedure_owner": ("Missing", None, None),
    "external_idempotency_adapter": ("Missing", None, None),
}
EXPECTED_MATRIX = [
    ("stable_service_issue_identity", "municipal_service_issue_owner", "Missing"),
    ("immutable_intake_lineage", "commons_support_coordinator", "OperationalOwner"),
    ("operational_status_and_external_currentness", "commons_support_ticket", "OperationalOwner"),
    ("assignment_and_dispatch", "commons_support_ticket", "OperationalOwner"),
    ("resident_intake_presentation", "community_water_ui", "PresentationOnly"),
    ("evidence_and_attachment_custody", "civic_evidence_types", "EvidenceOwner"),
    ("rights_affecting_administrative_procedure", "administrative_procedure_owner", "Missing"),
    ("external_city_reference_and_source_currentness", "city_provider_adapter", "Missing"),
    ("completion_claim_and_independent_verification", "municipal_completion_verifier", "Missing"),
    ("service_level_contract_and_clock", "service_level_registry", "Missing"),
    ("offline_retry_and_idempotency", "external_idempotency_adapter", "Missing"),
    ("physical_water_service_resolution", "water_resource_model", "NotApplicable"),
    ("municipal_work_order_lifecycle", "manufacturing_work_order", "NotApplicable"),
    ("formal_dispute_procedure", "justice_case_model", "NotApplicable"),
    ("external_provider_transport", "commons_bridge", "NotApplicable"),
    ("symthaea_action_authority", "municipal_service_issue_owner", "Missing"),
]
EXPECTED_PR_STATES = {2009, 2014, 2026, 2045, 2146, 1082}
REQUIRED_FLAGS = [
    "source_access_authorized", "external_reference_preserved",
    "external_currentness_verified", "stable_mapping_explicitly_approved",
    "idempotency_contract_qualified", "privacy_and_legal_review_bound",
    "human_approval_required", "symthaea_advisory_only",
    "per_report_sla_clocks_preserved", "completion_verification_separate",
    "service_level_source_pinned", "production_side_effects_disabled",
]
UNSAFE_FLAGS = [
    "city_reference_used_as_internal_identity",
    "municipal_acceptance_treated_as_physical_resolution",
    "ticket_closed_treated_as_physical_resolution",
    "symthaea_recommendation_exercises_authority",
    "unmerged_branch_assumed_qualified",
    "duplicate_merge_resets_individual_sla_clock",
    "protected_resident_data_written_to_public_dht",
]
EXPECTED_PROBE_IDS = [
    "city-reference-overpromotion",
    "municipal-acceptance-is-not-repair",
    "ticket-close-is-not-restoration",
    "model-recommendation-is-not-authority",
    "open-pr-does-not-inherit-qualification",
    "dedup-must-preserve-each-clock",
    "stale-or-unverified-external-state",
    "unqualified-provider-retry",
    "no-authorised-source-contract",
    "all-shadow-preconditions-explicit",
]


def fail(message: str) -> None:
    raise ValueError(message)


def reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            fail(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def reject_nonfinite(value: str) -> None:
    fail(f"non-finite JSON value rejected: {value}")


def exact_keys(value: Any, keys: set[str], where: str) -> None:
    if not isinstance(value, dict) or set(value) != keys:
        fail(f"{where}: expected exact keys {sorted(keys)}")


def classify_probe(facts: dict[str, Any]) -> str:
    """Classify evidence readiness; never grants production or civic authority."""
    if any(facts.get(flag) is True for flag in UNSAFE_FLAGS):
        return "REJECT"
    if not all(facts.get(flag) is True for flag in REQUIRED_FLAGS):
        return "UNRESOLVED"
    return "ADVISORY_SHADOW_ELIGIBLE"


def validate() -> None:
    if not DOC.is_file():
        fail(f"document missing: {DOC}")
    raw = MANIFEST.read_text(encoding="utf-8")
    data = json.loads(
        raw,
        object_pairs_hook=reject_duplicate_keys,
        parse_constant=reject_nonfinite,
    )
    exact_keys(data, {
        "schema", "program", "observed_at", "source_snapshot",
        "classification_vocabulary", "candidates", "owner_matrix",
        "branch_only_proposals", "source_anchors", "municipal_sla_source_posture",
        "decision", "invariants", "required_gate_flags", "unsafe_flags",
        "probe_cases", "nonclaims",
    }, "manifest")
    if data["schema"] != "mycelix.civic-resilience.service-request-owner-census.v1":
        fail("unexpected schema")
    if data["program"] != "CIV-RES-002A0" or data["observed_at"] != "2026-10-13":
        fail("program or snapshot date drift")
    snapshot = data["source_snapshot"]
    exact_keys(snapshot, {"repository", "branch", "commit_sha", "scope", "duplicate_workspace_note"}, "source_snapshot")
    if snapshot["repository"] != "Luminous-Dynamics/mycelix" or snapshot["branch"] != "main":
        fail("unexpected source repository/branch")
    if snapshot["commit_sha"] != EXPECTED_MAIN:
        fail("source snapshot main commit drift")
    if data["classification_vocabulary"] != VOCABULARY:
        fail("classification vocabulary drift")
    if data["required_gate_flags"] != REQUIRED_FLAGS or data["unsafe_flags"] != UNSAFE_FLAGS:
        fail("gate or safety flag drift")

    candidates = data["candidates"]
    if not isinstance(candidates, list):
        fail("candidates must be a list")
    seen: set[str] = set()
    by_id: dict[str, dict[str, Any]] = {}
    for row in candidates:
        exact_keys(row, {
            "candidate_id", "semantic_concern", "classification", "path",
            "blob_sha", "scope_limit", "finding",
        }, "candidate")
        cid = row["candidate_id"]
        if not isinstance(cid, str) or cid in seen:
            fail(f"invalid/duplicate candidate id: {cid}")
        seen.add(cid)
        by_id[cid] = row
        if row["classification"] not in VOCABULARY:
            fail(f"{cid}: unknown classification")
        expected = EXPECTED_CANDIDATES.get(cid)
        if expected is None:
            fail(f"unreviewed candidate added without validator update: {cid}")
        if (row["classification"], row["path"], row["blob_sha"]) != expected:
            fail(f"{cid}: classification/path/blob snapshot mismatch")
        if row["path"] is None:
            if row["blob_sha"] is not None or row["classification"] != "Missing":
                fail(f"{cid}: a missing candidate must have null path/blob and Missing classification")
        elif not re.fullmatch(r"[0-9a-f]{40}", row["blob_sha"] or ""):
            fail(f"{cid}: malformed Git blob SHA")
    if seen != set(EXPECTED_CANDIDATES):
        fail("candidate census has omissions or unexpected ids")

    matrix = data["owner_matrix"]
    observed_matrix = []
    for row in matrix:
        exact_keys(row, {"concern_id", "candidate_id", "classification", "decision"}, "owner matrix row")
        cid = row["candidate_id"]
        if cid not in by_id:
            fail(f"owner matrix references unknown candidate {cid}")
        if row["classification"] != by_id[cid]["classification"]:
            fail(f"{row['concern_id']}: classification does not match candidate")
        observed_matrix.append((row["concern_id"], cid, row["classification"]))
    if observed_matrix != EXPECTED_MATRIX:
        fail("owner matrix coverage/order/classification drift")

    proposals = data["branch_only_proposals"]
    proposal_ids: set[int] = set()
    for proposal in proposals:
        exact_keys(proposal, {"program", "pr", "branch", "head_sha", "state_at_snapshot", "scope_note"}, "branch-only proposal")
        if type(proposal["pr"]) is not int or proposal["pr"] in proposal_ids:
            fail("invalid/duplicate PR number")
        proposal_ids.add(proposal["pr"])
        if proposal["state_at_snapshot"] != "open_unmerged":
            fail("branch-only proposal must remain explicitly non-main at the frozen snapshot")
        if proposal["head_sha"] is not None and not re.fullmatch(r"[0-9a-f]{40}", proposal["head_sha"]):
            fail("malformed branch-only head SHA")
    if proposal_ids != EXPECTED_PR_STATES:
        fail("branch-only proposal census drift")

    posture = data["municipal_sla_source_posture"]
    exact_keys(posture, {
        "latest_city_service_level_charter_listed_by_official_index",
        "current_planning_documents_listed", "adopted_2026_27_water_sla_bound_into_this_census",
        "rule",
    }, "municipal SLA source posture")
    if posture["adopted_2026_27_water_sla_bound_into_this_census"] is not False:
        fail("must not assert an adopted 2026/27 SLA without source qualification")

    decision = data["decision"]
    exact_keys(decision, {
        "direction", "internal_candidate", "authoritative_source",
        "new_generic_service_issue_runtime_authorized", "provider_integration_authorized",
        "live_resident_data_processing_authorized", "production_side_effects_authorized",
        "symthaea_mode", "build_gate", "required_preconditions",
    }, "decision")
    if decision["direction"] != "NoNewServiceIssueOwner;ExternalSourceAdapterOnlyAfterGates":
        fail("decision direction drift")
    for key in (
        "new_generic_service_issue_runtime_authorized", "provider_integration_authorized",
        "live_resident_data_processing_authorized", "production_side_effects_authorized",
    ):
        if decision[key] is not False:
            fail(f"{key} must remain false")
    if decision["symthaea_mode"] != "AdvisoryOnly" or decision["build_gate"] != "BLOCKED":
        fail("unsafe autonomy/build-gate value")

    nonclaims = data["nonclaims"]
    expected_nonclaims = {
        "canonical_municipal_service_owner_absence_proven",
        "city_provider_api_available_or_authorized", "city_adoption_or_endorsement",
        "municipal_authority_granted", "popia_compliance_established",
        "current_2026_27_water_sla_adopted", "production_ready",
        "real_world_processing_authorized",
    }
    exact_keys(nonclaims, expected_nonclaims, "nonclaims")
    if any(value is not False for value in nonclaims.values()):
        fail("all nonclaims must remain explicit false assertions")

    cases = data["probe_cases"]
    if not isinstance(cases, list):
        fail("probe_cases must be a list")
    ids = []
    for case in cases:
        exact_keys(case, {"case_id", "facts", "expected_disposition"}, "probe case")
        ids.append(case["case_id"])
        if not isinstance(case["facts"], dict):
            fail("probe facts must be an object")
        actual = classify_probe(case["facts"])
        if actual != case["expected_disposition"]:
            fail(f"{case['case_id']}: expected {case['expected_disposition']}, derived {actual}")
    if ids != EXPECTED_PROBE_IDS:
        fail("probe case coverage/order drift")

    all_ready = {flag: True for flag in REQUIRED_FLAGS}
    if classify_probe(all_ready) != "ADVISORY_SHADOW_ELIGIBLE":
        fail("positive shadow-control failed")
    if classify_probe({}) != "UNRESOLVED":
        fail("empty/unknown evidence must fail closed")
    for flag in UNSAFE_FLAGS:
        mutated = dict(all_ready)
        mutated[flag] = True
        if classify_probe(mutated) != "REJECT":
            fail(f"unsafe mutation not rejected: {flag}")
    if classify_probe({**all_ready, "external_currentness_verified": False}) != "UNRESOLVED":
        fail("unknown external currentness must remain unresolved")
    if classify_probe({**all_ready, "idempotency_contract_qualified": False}) != "UNRESOLVED":
        fail("unqualified retry/idempotency must remain unresolved")

    print(f"CIV-RES-002A0 PASS: {len(candidates)} candidates, {len(matrix)} owner concerns, {len(proposals)} branch-only proposals, {len(cases)} semantic probes.")
    print("Disposition ceiling: ADVISORY_SHADOW_ELIGIBLE only; no live access, authority, or deployment qualification.")


if __name__ == "__main__":
    try:
        validate()
    except Exception as exc:
        print(f"CIV-RES-002A0 FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
