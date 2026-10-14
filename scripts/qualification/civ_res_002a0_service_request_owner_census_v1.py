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
    ("water_specific_channel_interoperability", "city_provider_adapter", "Missing"),
    ("public_turnaround_semantics_vs_adopted_sla", "service_level_registry", "Missing"),
    ("official_utility_source_conflict_precedence", "service_level_registry", "Missing"),
    ("utility_contact_instructions_by_role", "city_provider_adapter", "Missing"),
    ("utility_customer_service_standard_semantics", "service_level_registry", "Missing"),
    ("completion_claim_and_independent_verification", "municipal_completion_verifier", "Missing"),
    ("service_level_contract_and_clock", "service_level_registry", "Missing"),
    ("offline_retry_and_idempotency", "external_idempotency_adapter", "Missing"),
    ("physical_water_service_resolution", "water_resource_model", "NotApplicable"),
    ("municipal_work_order_lifecycle", "manufacturing_work_order", "NotApplicable"),
    ("formal_dispute_procedure", "justice_case_model", "NotApplicable"),
    ("external_provider_transport", "commons_bridge", "NotApplicable"),
    ("symthaea_action_authority", "municipal_service_issue_owner", "Missing"),
    ("support_ticket_field_mutability", "commons_support_coordinator", "OperationalOwner"),
    ("support_status_index_currentness", "commons_support_coordinator", "OperationalOwner"),
    ("support_typed_transition_algebra", "commons_support_ticket", "OperationalOwner"),
    ("support_autonomous_action_authority", "commons_support_coordinator", "OperationalOwner"),
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
    "support_immutable_field_rules_qualified",
    "support_status_index_currentness_qualified",
    "support_transition_algebra_qualified",
    "support_autonomy_boundary_qualified",
    "utility_channel_contract_pinned",
    "operational_turnaround_semantics_separated",
    "source_precedence_or_conflict_resolution_bound",
    "utility_customer_standards_source_pinned",
    "response_vs_resolution_semantics_preserved",
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
    "whole_ticket_replacement_treated_as_civic_safe",
    "status_index_assumed_current_after_update",
    "untyped_support_transition_accepted",
    "support_autonomy_boolean_treated_as_civic_authority",
    "utility_email_treated_as_api_authority",
    "systemwide_water_bulletin_treated_as_case_resolution",
    "same_external_reference_across_issuers_treated_as_same_identity",
    "customer_fault_portal_access_treated_as_api_authority",
    "published_turnaround_treated_as_adopted_sla",
    "non_equivalent_service_category_crosswalked_automatically",
    "official_contact_conflict_silently_canonicalized",
    "one_official_page_silently_preferred_over_conflicting_page",
    "page_crawl_date_treated_as_service_target_effective_date",
    "uncorroborated_emergency_contact_treated_as_verified_contact",
    "route_name_used_as_proof_of_service_scope",
    "service_response_treated_as_restoration_deadline",
    "working_days_treated_as_calendar_days",
    "notification_lead_time_treated_as_repair_window",
    "notification_subscription_treated_as_case_submission",
    "unversioned_utility_web_standard_treated_as_current_adopted_sla",
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
    "whole-ticket-replacement-is-not-civic-safe",
    "status-index-is-not-current-by-assumption",
    "support-transition-needs-algebra",
    "support-autonomy-is-not-civic-authority",
    "support-immutable-fields-not-qualified",
    "support-index-currentness-not-qualified",
    "support-transitions-not-qualified",
    "support-autonomy-boundary-not-qualified",
    "utility-email-is-not-api-authority",
    "systemwide-water-status-is-not-case-resolution",
    "external-reference-is-issuer-channel-scoped",
    "utility-channel-contract-is-required",
    "customer-fault-portal-is-not-api-authority",
    "published-turnaround-is-not-adopted-sla",
    "service-category-crosswalk-needs-proof",
    "daily-notices-contact-conflict-is-not-silently-canonicalized",
    "turnaround-semantics-gate-is-required",
    "conflicting-official-page-needs-precedence",
    "crawl-date-is-not-effective-date",
    "unverified-emergency-contact-is-not-authoritative",
    "route-name-does-not-establish-scope",
    "source-precedence-gate-is-required",
    "response-is-not-restoration",
    "working-days-are-not-calendar-days",
    "advance-notice-is-not-repair-window",
    "notification-channel-is-not-intake",
    "unversioned-web-target-needs-adoption",
    "customer-standards-source-is-required",
    "response-resolution-semantics-is-required",
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
        "support_civic_adoption_boundary", "external_utility_channel_contract", "decision", "invariants",
        "required_gate_flags", "unsafe_flags", "probe_cases", "nonclaims",
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
        "planned_interruption_metric_semantics", "2026_27_sdbip_listed",
        "2026_27_sdbip_document_retrieval_status", "current_sdbip_water_metrics_reconciled_to_budget_book",
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
    if posture["current_target_claim_ceiling"] != "OfficialBudgetBookTextObservedPendingVisualReviewAdoptionAndClockSemantics":
        fail("current-target claim ceiling drift")
    if posture["2026_27_sdbip_listed"] is not True:
        fail("the officially indexed 2026/27 SDBIP link must remain recorded")
    if posture["2026_27_sdbip_document_retrieval_status"] != "OfficialIndexLinkObserved;PdfTextExtractionTimedOut":
        fail("do not claim the SDBIP PDF was inspected when extraction timed out")
    if posture["current_sdbip_water_metrics_reconciled_to_budget_book"] is not False:
        fail("do not claim SDBIP/budget metrics are reconciled without source review")
    if posture["reporting_clock_status"] != "NotEvaluableUntilCurrentAdoptedTargetAndClockSemanticsArePinned":
        fail("SLA reporting must remain not-evaluable until source and clock semantics are qualified")

    book = data["current_year_budget_book_metrics"]
    exact_keys(book, {
        "source_id", "source_url", "source_section", "source_page", "effective_period",
        "extraction_status", "direct_pdf_text_review_complete", "direct_pdf_visual_review_complete",
        "visual_review_attempt", "production_threshold_use_authorized", "metrics", "historical_comparison",
    }, "current-year budget-book metrics")
    if book["source_id"] != "COJ_FINAL_MEDIUM_TERM_BUDGET_2026_27_2028_29" or book["source_url"] != "https://joburg.org.za/documents_/Documents/2026-27-Final-Medium-Budget-Reports/2026-27-FINAL-BUDGET-BOOK.pdf":
        fail("budget book source identity mismatch")
    if book["source_page"] != 125 or book["direct_pdf_visual_review_complete"] is not False:
        fail("budget book page/review status drift")
    if book["extraction_status"] != "OfficialPdfTextLayerFromDocumentViewer" or book["direct_pdf_text_review_complete"] is not True:
        fail("direct text-layer source review must be recorded accurately")
    if not isinstance(book["visual_review_attempt"], str) or "TimeoutError" not in book["visual_review_attempt"]:
        fail("visual-review timeout and limitation must remain explicit")
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
    if source_anchors.get("COJ_SDBIP_2026_27") != "https://joburg.org.za/documents_/Documents/2026-27_SDBIP.pdf":
        fail("official-index-listed 2026/27 SDBIP source anchor missing")
    if source_anchors.get("JOHANNESBURG_WATER_PUBLIC_CHANNELS") != "https://www.johannesburgwater.co.za/":
        fail("Johannesburg Water public channel source anchor missing")
    if source_anchors.get("JOHANNESBURG_WATER_CUSTOMER_FAULT_PORTAL") != "https://customer.forcelink.net/joburg_water/login":
        fail("customer fault portal source anchor missing")
    if source_anchors.get("JOHANNESBURG_WATER_DAILY_NOTICES") != "https://www.johannesburgwater.co.za/daily-water-notices/":
        fail("daily notices source anchor missing")
    if source_anchors.get("JOHANNESBURG_WATER_EMERGENCIES_PAGE") != "https://www.johannesburgwater.co.za/emergencies/":
        fail("official emergencies route source anchor missing")
    if source_anchors.get("JOHANNESBURG_WATER_CONTACT_US") != "https://www.johannesburgwater.co.za/contact-us/":
        fail("dedicated Contact Us source anchor missing")
    if source_anchors.get("JOHANNESBURG_WATER_SERVICES_STANDARDS") != "https://www.johannesburgwater.co.za/services/":
        fail("customer service standards source anchor missing")

    utility_channels = data["external_utility_channel_contract"]
    exact_keys(utility_channels, {
        "schema", "source_id", "source_url", "source_observation_status",
        "documented_claims", "channel_records", "reference_key_components",
        "reference_namespace_collision_policy", "system_status_update_policy",
        "utility_fault_email_address", "utility_fault_email_scope",
        "resident_companion_mode_only", "automatic_submission_authorized",
        "provider_api_contract_identified", "provider_acceptance_proof",
        "per_case_currentness_qualified", "completion_verification_qualified",
        "production_side_effects_authorized", "status",
        "customer_fault_portal_url", "portal_authentication_boundary", "daily_water_notices_url",
        "published_turnaround_rows", "turnaround_semantics_policy",
        "contact_number_source_conflict", "contact_number_values_by_section", "contact_numbers_use_policy",
        "published_page_conflicts", "customer_contact_instructions", "customer_service_standards",
    }, "external utility channel contract")
    if (utility_channels["schema"], utility_channels["source_id"], utility_channels["source_url"]) != (
        "mycelix.civic-resilience.utility-channel-boundary.v1",
        "JOHANNESBURG_WATER_PUBLIC_CUSTOMER_CHANNELS",
        "https://www.johannesburgwater.co.za/",
    ):
        fail("Johannesburg Water public channel source identity mismatch")
    if utility_channels["source_observation_status"] != "OfficialWebsiteObserved":
        fail("public utility channel observation status drift")
    if utility_channels["reference_key_components"] != ["issuer_id", "channel_id", "external_reference"]:
        fail("external references must be issuer- and channel-namespaced")
    if utility_channels["utility_fault_email_address"] != "fault@jwater.co.za" or utility_channels["utility_fault_email_scope"] != "Water-related incidents only, as stated by the published site":
        fail("water incident channel scope drift")
    for key in (
        "resident_companion_mode_only", "provider_api_contract_identified",
        "automatic_submission_authorized", "provider_acceptance_proof",
        "per_case_currentness_qualified", "completion_verification_qualified",
        "production_side_effects_authorized",
    ):
        expected = key == "resident_companion_mode_only"
        if utility_channels[key] is not expected:
            fail(f"unsafe utility channel authority value: {key}")
    if utility_channels["status"] != "PubliclyDocumentedChannelsOnly;NoAPIOrPerIssueReadbackQualified":
        fail("utility channel claim ceiling drift")
    expected_claims = [
        ("water_fault_email", "ResidentFacingChannelInstruction"),
        ("daily_water_status_bulletin", "UtilityWideStatusBulletin"),
        ("water_call_centre", "ResidentFacingContactChannel"),
    ]
    claims = []
    for claim in utility_channels["documented_claims"]:
        exact_keys(claim, {"claim_id", "published_fact", "fact_kind"}, "utility source claim")
        claims.append((claim["claim_id"], claim["fact_kind"]))
    expected_claims.extend([
        ("fault_logging_portal", "ResidentFacingPortalLink"),
        ("published_turnaround_times", "CustomerFacingTurnaroundStatement"),
        ("emergencies_route_turnaround_and_contact_variant", "OfficialPageVariantConflict"),
        ("official_contact_us_role_specific_instructions", "RoleSpecificContactInstruction"),
        ("customer_service_standards_response_and_clock_semantics", "CustomerServiceStandardWithoutObservedEffectiveDate"),
    ])
    if claims != expected_claims:
        fail("documented water channel claim census drift")
    expected_channel_ids = [
        ("CityOfJohannesburg", "JoburgConnect"),
        ("JohannesburgWaterSOC", "water_fault_email"),
        ("JohannesburgWaterSOC", "customer_fault_portal"),
        ("JohannesburgWaterSOC", "daily_water_notices"),
        ("JohannesburgWaterSOC", "daily_water_status_bulletin"),
        ("JohannesburgWaterSOC", "emergencies_page"),
        ("JohannesburgWaterSOC", "contact_us_calls_queries"),
        ("JohannesburgWaterSOC", "interruption_notifications_email"),
    ]
    channel_ids = []
    for channel in utility_channels["channel_records"]:
        exact_keys(channel, {
            "issuer_id", "channel_id", "reference_semantics",
            "resident_facing_channel_published", "automated_mycelix_access_authorized",
            "per_case_status_readback_qualified", "physical_repair_evidence_qualified",
        }, "utility channel record")
        channel_ids.append((channel["issuer_id"], channel["channel_id"]))
        if channel["resident_facing_channel_published"] is not True:
            fail("listed resident channel must remain source-backed")
        for key in (
            "automated_mycelix_access_authorized", "per_case_status_readback_qualified",
            "physical_repair_evidence_qualified",
        ):
            if channel[key] is not False:
                fail(f"{channel['channel_id']}: unsafe authority/currentness assertion")
    if channel_ids != expected_channel_ids:
        fail("utility channel issuer/namespace census drift")
    if "third-party Forcelink login" not in utility_channels["portal_authentication_boundary"]:
        fail("the portal login/authentication boundary must remain explicit")
    if utility_channels["customer_fault_portal_url"] != "https://customer.forcelink.net/joburg_water/login":
        fail("customer portal route mismatch")
    if utility_channels["daily_water_notices_url"] != "https://www.johannesburgwater.co.za/daily-water-notices/":
        fail("daily notices source URL mismatch")
    if "not an individual ticket status" not in utility_channels["channel_records"][4]["reference_semantics"]:
        fail("system-wide bulletin must not masquerade as a per-issue reference")
    if utility_channels["contact_number_source_conflict"] is not True:
        fail("published contact conflict must be retained")
    expected_contact_sections = [
        ("DailyWaterNoticesLoggingCalls", ["+27 (0) 10 061 4470", "+27 (0) 86 056 2874"]),
        ("DailyWaterNoticesFooter", ["+27 (0) 11 688 1699", "+27 (0) 86 056 2874"]),
    ]
    observed_contact_sections = [(c["source_section"], c["numbers"]) for c in utility_channels["contact_number_values_by_section"]]
    if observed_contact_sections != expected_contact_sections:
        fail("page-local contact-number conflict must remain source-scoped")
    if "do not silently canonicalize" not in utility_channels["contact_numbers_use_policy"]:
        fail("contact-number conflict use policy drift")
    conflict = utility_channels["published_page_conflicts"]
    exact_keys(conflict, {"conflict_set_id", "observed_on", "observation_method", "route_scope_relationship", "effective_date_rule", "cross_page_precedence", "source_variants", "observed_differences", "allowed_conclusion", "forbidden_inferences"}, "official page conflict set")
    if conflict["conflict_set_id"] != "JOHANNESBURG_WATER_DAILY_NOTICES_VS_EMERGENCIES" or conflict["observed_on"] != "2026-10-13":
        fail("official-page conflict identity/observation date drift")
    if conflict["observation_method"] != "OfficialPageTextObserved" or conflict["cross_page_precedence"] != "NoneEstablished":
        fail("source precedence must remain explicitly unresolved")
    if conflict["effective_date_rule"] != "NotPublishedOnTheObservedPageText":
        fail("do not infer page-effective dates")
    if len(conflict["source_variants"]) != 2 or len(conflict["observed_differences"]) != 5:
        fail("source conflict row count drift")
    expected_variants = [
        ("daily_water_notices", "https://www.johannesburgwater.co.za/daily-water-notices/", "Daily water Notices", "fault@jwater.co.za", "+27 (0) 10 061 4470", "+27 (0) 11 688 1699", {"sewer":48,"water_outage":24,"leaking_meter":72,"burst_meter":24,"water_leak":48,"major_burst_pipe":48}),
        ("emergencies", "https://www.johannesburgwater.co.za/emergencies/", "Daily water Notices", "customerserviceemails@jwater.co.za", "+27 (0) 11 375 5555", "+27 (0) 0 061 4470", {"sewer":24,"water_outage":12,"leaking_meter":72,"burst_meter":24,"water_leak":48,"major_burst_pipe":48}),
    ]
    observed_variants = []
    for variant in conflict["source_variants"]:
        exact_keys(variant, {"page_id", "url", "page_heading", "crawl_observation_date", "crawl_date_is_effective_date", "logging_email", "logging_primary_contact", "logging_secondary_contact", "footer_email", "footer_primary_contact", "footer_secondary_contact", "turnarounds_hours"}, "official page variant")
        if variant["crawl_observation_date"] != conflict["observed_on"] or variant["crawl_date_is_effective_date"] is not False:
            fail("crawl date must not be treated as source effective date")
        observed_variants.append((variant["page_id"], variant["url"], variant["page_heading"], variant["logging_email"], variant["logging_primary_contact"], variant["footer_primary_contact"], variant["turnarounds_hours"]))
    if observed_variants != expected_variants:
        fail("official page variant values/identity drift")
    expected_differences = [
        ("turnarounds_hours.sewer", 48, 24),
        ("turnarounds_hours.water_outage", 24, 12),
        ("logging_email", "fault@jwater.co.za", "customerserviceemails@jwater.co.za"),
        ("logging_primary_contact", "+27 (0) 10 061 4470", "+27 (0) 11 375 5555"),
        ("footer_primary_contact", "+27 (0) 11 688 1699", "+27 (0) 0 061 4470"),
    ]
    observed_difference_tuples = [(d["field_id"], d["daily_water_notices_value"], d["emergencies_value"]) for d in conflict["observed_differences"]]
    if observed_difference_tuples != expected_differences:
        fail("official page conflict value set drift")
    if conflict["allowed_conclusion"] != "OfficialPageVariantConflict;NoPrecedenceOrEffectiveDateQualified":
        fail("allowed claim ceiling for page conflict drift")
    if len(conflict["forbidden_inferences"]) != 4:
        fail("forbidden page-precedence inferences must remain explicit")
    if len(utility_channels["published_turnaround_rows"]) != 6:
        fail("public turnaround row coverage drift")
    expected_turnarounds = [
        ("sewer", "Sewer: 48 hours", 48, "sewer_blockages_cleared_24h", "DifferentPublishedWindowAndScope;NotEquivalent"),
        ("water_outage", "Water outage: 24 hours", 24, "planned_water_interruptions_prescribed_timeline", "BudgetMetricHasUnknownClockAndDifferentMetricScope;NotEquivalent"),
        ("leaking_meter", "Leaking meter: 72 hours", 72, "defective_meters_repaired_3d", "MatchingNominalWindowDoesNotEstablishAssetOrProcessEquivalence"),
        ("burst_meter", "Burst meter: 24 hours", 24, "stolen_meters_replaced_24h", "DifferentAssetEventAndOperation;NotEquivalent"),
        ("water_leak", "Water leak: 48 hours", 48, "leaking_valves_repaired_48h", "DifferentAssetType;NotEquivalent"),
        ("major_burst_pipe", "Major burst pipe: 48 hours", 48, "water_pipe_bursts_repaired_48h", "PotentialCategoryRelation;NoOfficialCrosswalk"),
    ]
    observed_turnarounds = []
    for row in utility_channels["published_turnaround_rows"]:
        exact_keys(row, {"service_id", "published_text", "published_window_hours", "comparator_metric_id", "crosswalk_status"}, "published turnaround row")
        observed_turnarounds.append((row["service_id"], row["published_text"], row["published_window_hours"], row["comparator_metric_id"], row["crosswalk_status"]))
    if observed_turnarounds != expected_turnarounds:
        fail("published turnaround values/crosswalk ceiling drift")
    if "do not prove identical scope" not in utility_channels["turnaround_semantics_policy"]:
        fail("turnaround-versus-SLA policy drift")
    if "must not be auto-merged" not in utility_channels["reference_namespace_collision_policy"]:
        fail("cross-issuer/channel reference collision policy drift")
    if "cannot set an individual issue to resolved" not in utility_channels["system_status_update_policy"]:
        fail("system-wide status update must not resolve an individual issue")

    contact = utility_channels["customer_contact_instructions"]
    exact_keys(contact, {
        "source_id", "url", "source_observation_status", "page_effective_date",
        "body_entries", "footer_entries", "interpretation",
        "global_precedence_over_other_pages", "automated_submission_authorized",
        "per_case_status_readback_qualified", "subscriber_api_contract_identified",
    }, "customer contact instructions")
    if (contact["source_id"], contact["url"]) != ("JOHANNESBURG_WATER_CONTACT_US", "https://www.johannesburgwater.co.za/contact-us/"):
        fail("Contact Us source identity mismatch")
    if contact["source_observation_status"] != "OfficialPageTextObserved" or contact["page_effective_date"] != "NotObserved":
        fail("do not infer Contact Us page effective date")
    if contact["global_precedence_over_other_pages"] != "NotEstablished":
        fail("Contact Us body does not establish global website precedence")
    for key in ("automated_submission_authorized", "per_case_status_readback_qualified", "subscriber_api_contract_identified"):
        if contact[key] is not False:
            fail(f"contact instructions must not grant API/currentness capability: {key}")
    exact_body_entries = [
        ("log_calls_queries", "To log calls/queries", "+27 (0) 10 061 4470", "fault@jwater.co.za", "ResidentIncidentIntakeInstruction"),
        ("interruption_notifications", "To receive notifications on interruptions", None, "sms@jwater.co.za", "NotificationChannelInstruction"),
        ("reception", "Reception", "+27 (0) 0 061 4450", None, "GeneralReception"),
    ]
    body_rows = []
    for row in contact["body_entries"]:
        exact_keys(row, {"role_id", "heading", "phone", "email", "semantic_kind"}, "role-specific contact entry")
        body_rows.append((row["role_id"], row["heading"], row["phone"], row["email"], row["semantic_kind"]))
    if body_rows != exact_body_entries:
        fail("purpose-labelled contact instruction census drift")
    if len(contact["footer_entries"]) != 1:
        fail("generic footer contact row count drift")
    for row in contact["footer_entries"]:
        exact_keys(row, {"role_id", "heading", "phone_values", "email", "semantic_kind"}, "generic footer contact entry")
        if (row["role_id"], row["heading"], row["phone_values"], row["email"], row["semantic_kind"]) != (
            "generic_contact_center", "Contact Center", ["+27 (0) 0 061 4470", "+27 (0) 86 056 2874"], "fault@jwater.co.za", "GenericSiteFooterContact"
        ):
            fail("generic footer contact detail drift")

    standards = utility_channels["customer_service_standards"]
    exact_keys(standards, {
        "source_id", "url", "source_observation_status", "page_heading", "table_heading",
        "effective_date", "version", "production_sla_use_authorized", "rows", "semantics_policy",
    }, "customer service standards source")
    if (standards["source_id"], standards["url"]) != ("JOHANNESBURG_WATER_SERVICES_CUSTOMER_STANDARDS", "https://www.johannesburgwater.co.za/services/"):
        fail("Services standards source identity mismatch")
    if standards["source_observation_status"] != "OfficialPageTextObserved" or standards["effective_date"] != "NotObserved" or standards["version"] != "NotObserved":
        fail("do not infer Services page standard version/effective date")
    if standards["page_heading"] != "Services" or standards["table_heading"] != "CUSTOMER SERVICE STANDARDS TARGET" or standards["production_sla_use_authorized"] is not False:
        fail("customer standard source claim ceiling drift")
    expected_standard_rows = [
        ("appointments", "Appointments", "ArrangeAppointment", None, None, "UponRequest", "AppointmentScheduling"),
        ("domestic_faulty_meter", "Domestic faulty meter", "RepairOrReplace", 2, "WorkingDays", "Notification", "RepairOrReplace"),
        ("water_meter_installation", "Requested water meter installation", "Install", 25, "WorkingDays", "PaymentOrAgreedPaymentMethod", "Install"),
        ("monthly_billing_and_actual_reading", "Metered supply bills/readings", "BillAndSeekActualReading", 1, "Months", "MonthlyWithUnrestrictedMeterAccess", "BillingAndReadingCadence"),
        ("moving_house_meter_reading", "Meter reading when moving house", "ReadMeterOnMoveDay", 5, "WorkingDaysNotice", "AdvanceNoticeBeforeMoveDay", "PrerequisiteNoticeAndSameDayReading"),
        ("planned_interruptions_over_4h_notice", "Planned supply interruption over 4 hours", "Notify", 7, "DaysConventionUnspecified", "BeforePlannedInterruption", "AdvanceNotice"),
        ("planned_interruptions_1_to_4h_notice", "Planned supply interruption 1–4 hours", "Notify", 12, "Hours", "BeforePlannedInterruption", "AdvanceNotice"),
        ("planned_restoration", "Restoration after planned work", "AttemptRestoreBySpecifiedTime", None, None, "SpecifiedTime", "AttemptedRestoration"),
        ("unplanned_emergency_response", "Unplanned supply interruptions (emergencies)", "Respond", 12, "Hours", "Unspecified", "ResponseNotRestoration"),
        ("in_home_water_or_sewage_flooding", "Water/sewage flooding in customer home from Johannesburg Water pipe", "RespondAndOfferImmediateHelp", 2, "HoursUpperBoundTextWithImmediatelyAlternative", "Unspecified", "EmergencyResponse"),
        ("water_quality_customer_response", "Respond to customers on water quality", "Respond", 12, "Hours", "Unspecified", "CustomerResponse"),
        ("water_quality_first_sample_results", "Water quality sample", "ProvideFirstSampleResults", 12, "Hours", "Unspecified", "TestResult"),
    ]
    standard_rows = []
    for row in standards["rows"]:
        exact_keys(row, {"standard_id", "published_text", "subject", "action", "window_value", "window_unit", "clock_start", "verb_semantics"}, "customer standard row")
        standard_rows.append((row["standard_id"], row["subject"], row["action"], row["window_value"], row["window_unit"], row["clock_start"], row["verb_semantics"]))
    if standard_rows != expected_standard_rows:
        fail("customer service standard rows/semantics drift")
    if "Notify != Respond != Restore" not in standards["semantics_policy"] or "WorkingDays != calendar days" not in standards["semantics_policy"]:
        fail("response/restoration/working-day semantic boundaries missing")

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

    support = data["support_civic_adoption_boundary"]
    exact_keys(support, {
        "status", "source_pins", "author_binding_note", "blockers",
        "civic_support_mutations_authorized", "support_status_index_currentness_qualified",
        "immutable_field_rules_qualified", "typed_transition_algebra_qualified",
        "autonomy_authority_boundary_qualified", "permitted_use",
    }, "support civic adoption boundary")
    if support["status"] != "BlockedPendingExactHeadQualifiedRepair":
        fail("Support civic adoption must remain blocked")
    for key in ("civic_support_mutations_authorized", "support_status_index_currentness_qualified",
                "immutable_field_rules_qualified", "typed_transition_algebra_qualified",
                "autonomy_authority_boundary_qualified"):
        if support[key] is not False:
            fail(f"{key} must remain false")
    source_pins = {item.get("path"): item.get("blob_sha") for item in support["source_pins"] if isinstance(item, dict)}
    if source_pins != {
        "mycelix-commons/zomes/support-tickets/integrity/src/lib.rs": "c66ff344f6f3c0a43533d9370c887f2f200619fd",
        "mycelix-commons/zomes/support-tickets/coordinator/src/lib.rs": "e04fdd61227aa485a801a9e2dfe39087d3953e7d",
        "mycelix-commons/crates/support-types/src/lib.rs": "bfb405e6d8f9f0afbc30cf33fabbfbe1f2197654",
    }:
        fail("Support source paths/blob pins drift")
    blocker_ids = [b.get("blocker_id") for b in support["blockers"] if isinstance(b, dict)]
    if blocker_ids != ["stable_logical_identity", "whole_entry_mutation", "status_and_category_index_currentness", "typed_transition_algebra", "support_autonomy_not_civic_authority"]:
        fail("Support adoption blocker census drift")

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
