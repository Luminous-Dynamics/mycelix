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
    "external_reference_namespace_bound", "source_reference_origin_recorded",
    "official_channel_instructions_present", "current_adopted_sla_source_verified",
    "2026_27_budget_book_source_pinned", "metric_definition_and_period_preserved",
    "source_pdf_review_and_clock_semantics_qualified",
]
UNSAFE_FLAGS = [
    "city_reference_used_as_internal_identity",
    "municipal_acceptance_treated_as_physical_resolution",
    "ticket_closed_treated_as_physical_resolution",
    "symthaea_recommendation_exercises_authority",
    "unmerged_branch_assumed_qualified",
    "duplicate_merge_resets_individual_sla_clock",
    "protected_resident_data_written_to_public_dht",
    "portal_access_treated_as_api_authorization",
    "same_reference_follow_up_creates_new_external_intake",
    "unverified_city_status_claimed_authoritative",
    "prior_year_target_substituted_for_current",
    "planned_metric_reinterpreted_as_unplanned_12h_target",
    "indexed_budget_target_claimed_as_live_status",
]
EXPECTED_PROBE_IDS = [
    "city-reference-overpromotion",
    "municipal-acceptance-is-not-repair",
    "ticket-close-is-not-restoration",
    "model-recommendation-is-not-authority",
    "open-pr-does-not-inherit-qualification",
    "dedup-must-preserve-each-clock",
    "protected-resident-data-stays-private",
    "portal-access-is-not-api-authority",
    "same-reference-followup-is-not-new-intake",
    "unverified-status-is-not-authoritative",
    "prior-year-target-is-not-current",
    "planned-interruption-is-not-12h-general-target",
    "indexed-target-is-not-live-status",
    "stale-or-unverified-external-state",
    "unqualified-provider-retry",
    "no-authorized-source-contract",
    "missing-namespaced-source-reference",
    "missing-source-reference-origin",
    "no-current-adopted-sla",
    "source-pdf-not-reviewed",
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
    expected = set(REQUIRED_FLAGS + UNSAFE_FLAGS)
    if set(facts) != expected or any(type(v) is not bool for v in facts.values()):
        return "UNRESOLVED"
    if any(facts[flag] for flag in UNSAFE_FLAGS):
        return "REJECT"
    if not all(facts[flag] for flag in REQUIRED_FLAGS):
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
        "channel_first_pilot_contract", "current_year_budget_book_metrics",
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
        "rule", "service_level_standards_unit",
        "2026_27_service_level_standards_charter_listed",
        "official_source_index_observed", "adopted_2026_27_water_sla_verified",
        "reporting_clock_status", "2026_27_water_standards_in_budget_book_indexed",
        "current_target_claim_ceiling", "direct_pdf_visual_review_complete",
        "planned_interruption_metric_semantics",
    }, "municipal SLA source posture")
    if posture["adopted_2026_27_water_sla_bound_into_this_census"] is not False:
        fail("must not assert an adopted 2026/27 SLA without source qualification")
    if posture["2026_27_service_level_standards_charter_listed"] is not False:
        fail("do not claim a 2026/27 charter is listed")
    if posture["adopted_2026_27_water_sla_verified"] is not False:
        fail("do not claim a current adopted SLA without qualification")
    if posture["2026_27_water_standards_in_budget_book_indexed"] is not True:
        fail("the indexed current-year budget source must remain recorded")
    if posture["direct_pdf_visual_review_complete"] is not False:
        fail("the direct PDF review ceiling must remain explicit")
    if posture["current_target_claim_ceiling"] != "IndexedTargetReferencePendingDirectPdfReviewAndClockSemantics":
        fail("current-target claim ceiling drift")
    if posture["reporting_clock_status"] != "NotEvaluableUntilCurrentAdoptedTargetAndClockSemanticsArePinned":
        fail("SLA reporting must remain not-evaluable until source and clock semantics are qualified")

    book = data["current_year_budget_book_metrics"]
    exact_keys(book, {
        "source_id", "source_url", "source_section", "source_page", "effective_period",
        "extraction_status", "direct_pdf_visual_review_complete",
        "production_threshold_use_authorized", "metrics", "historical_comparison",
    }, "current-year budget-book metrics")
    if book["source_id"] != "COJ_FINAL_MEDIUM_TERM_BUDGET_2026_27_2028_29" or book["source_url"] != "https://joburg.org.za/documents_/Documents/2026-27-Final-Medium-Budget-Reports/2026-27-FINAL-BUDGET-BOOK.pdf":
        fail("budget book source identity mismatch")
    if book["source_page"] != 125 or book["direct_pdf_visual_review_complete"] is not False:
        fail("budget book page/review status drift")
    if book["production_threshold_use_authorized"] is not False:
        fail("indexed thresholds must not be used as qualified production gates")
    expected_metrics = [
        ("planned_water_interruptions_prescribed_timeline", 50, None),
        ("fire_hydrants_repaired_48h", 80, 48),
        ("stolen_meters_replaced_24h", 90, 24),
        ("defective_meters_repaired_3d", 80, 72),
        ("leaking_valves_repaired_48h", 92, 48),
        ("missing_manhole_covers_replaced_24h", 95, 24),
        ("water_pipe_bursts_repaired_48h", 85, 48),
        ("sewer_blockages_cleared_24h", 92, 24),
    ]
    actual_metrics = []
    for metric in book["metrics"]:
        exact_keys(metric, {"metric_id", "service", "target_percent", "window_hours", "scope", "clock_semantics", "source_page"}, "budget-book metric")
        if type(metric["target_percent"]) is not int or not 0 <= metric["target_percent"] <= 100:
            fail("invalid budget metric target")
        if metric["window_hours"] is not None and (type(metric["window_hours"]) is not int or metric["window_hours"] <= 0):
            fail("invalid budget metric window")
        actual_metrics.append((metric["metric_id"], metric["target_percent"], metric["window_hours"]))
    if actual_metrics != expected_metrics:
        fail("current-year budget-book metric values/identities drift")
    historical = book["historical_comparison"]
    exact_keys(historical, {"source_id", "source_url", "metric_id", "effective_period", "target_percent", "window_hours", "comparability", "direct_pdf_visual_review_complete"}, "historical SLA comparison")
    if (historical["effective_period"], historical["target_percent"], historical["window_hours"], historical["comparability"]) != ("2025/26", 90, 12, "NotEquivalentTo2026_27PlannedInterruptionMetric"):
        fail("historical metric must remain separate from the current planned-interruption measure")
    if historical["direct_pdf_visual_review_complete"] is not False:
        fail("historical PDF review status must remain explicit")

    source_anchors = {item.get("id"): item.get("url") for item in data["source_anchors"] if isinstance(item, dict)}
    if source_anchors.get("COJ_FINAL_MEDIUM_TERM_BUDGET_2026_27_2028_29") != book["source_url"]:
        fail("current-year budget-book source anchor missing or mismatched")

    pilot = data["channel_first_pilot_contract"]
    exact_keys(pilot, {
        "mode", "live_provider_calls_authorized", "automatic_city_submission_authorized",
        "provider_api_contract_identified", "official_channels_preserved",
        "external_reference_required_for_authoritative_followup", "reference_namespace_required",
        "reference_record_default_status", "references_persisted_to_public_dht",
        "raw_resident_identifiers_or_precise_protected_locations_persisted_to_public_dht",
        "same_reference_followup_action", "new_reference_action", "missing_reference_behavior",
        "manual_offline_reference_behavior", "external_status_without_authorized_readback",
        "current_sla_without_adopted_effective_source", "emergency_behavior",
    }, "channel-first pilot contract")
    for key in ("live_provider_calls_authorized", "automatic_city_submission_authorized",
                "provider_api_contract_identified", "references_persisted_to_public_dht",
                "raw_resident_identifiers_or_precise_protected_locations_persisted_to_public_dht"):
        if pilot[key] is not False:
            fail(f"{key} must remain false")
    if pilot["mode"] != "ResidentCompanionOnly" or pilot["reference_record_default_status"] != "UserReportedUnverified":
        fail("unsafe channel-first pilot mode or default reference state")
    if pilot["same_reference_followup_action"] != "AppendFollowUpToExistingExternalCase":
        fail("same-reference follow-up must not create a new external intake")
    if pilot["new_reference_action"] != "CreateSeparateSourceReferenceRecord;NoAutomaticMerge":
        fail("new external references must remain separate by default")
    if pilot["external_status_without_authorized_readback"] != "UserReportedUnverified":
        fail("resident-reported status must not be labelled authoritative")
    if pilot["current_sla_without_adopted_effective_source"] != "NotEvaluable":
        fail("SLA cannot be evaluated without a qualified current target")

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
        exact_keys(case["facts"], set(REQUIRED_FLAGS + UNSAFE_FLAGS), f"{case['case_id']} facts")
        if any(type(v) is not bool for v in case["facts"].values()):
            fail(f"{case['case_id']}: every fact must be an explicit boolean")
        actual = classify_probe(case["facts"])
        if actual != case["expected_disposition"]:
            fail(f"{case['case_id']}: expected {case['expected_disposition']}, derived {actual}")
    if ids != EXPECTED_PROBE_IDS:
        fail("probe case coverage/order drift")

    all_ready = {flag: True for flag in REQUIRED_FLAGS}
    all_ready.update({flag: False for flag in UNSAFE_FLAGS})
    if classify_probe(all_ready) != "ADVISORY_SHADOW_ELIGIBLE":
        fail("positive shadow-control failed")
    if classify_probe({}) != "UNRESOLVED":
        fail("empty/unknown evidence must fail closed")
    for flag in UNSAFE_FLAGS:
        mutated = dict(all_ready)
        mutated[flag] = True
        if classify_probe(mutated) != "REJECT":
            fail(f"unsafe mutation not rejected: {flag}")
    for flag in REQUIRED_FLAGS:
        mutated = dict(all_ready)
        mutated[flag] = False
        if classify_probe(mutated) != "UNRESOLVED":
            fail(f"missing positive evidence not unresolved: {flag}")

    print(f"CIV-RES-002A0 PASS: {len(candidates)} candidates, {len(matrix)} owner concerns, {len(proposals)} branch-only proposals, {len(cases)} semantic probes.")
    print("Disposition ceiling: ADVISORY_SHADOW_ELIGIBLE only; no live access, authority, or deployment qualification.")


if __name__ == "__main__":
    try:
        validate()
    except Exception as exc:
        print(f"CIV-RES-002A0 FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
