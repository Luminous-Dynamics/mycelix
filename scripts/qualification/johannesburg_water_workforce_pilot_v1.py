#!/usr/bin/env python3
"""Fail-closed structural and semantic checks for the Joburg pilot corpus.

This is a documentation/test qualification aid. It does not exercise production
runtime behavior, access a provider, or qualify a field deployment.
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "mycelix-workspace/docs/civic-resilience"
MANIFEST_PATH = BASE / "johannesburg-water-workforce-pilot-v1.json"
CORPUS_PATH = BASE / "johannesburg-water-workforce-pilot-synthetic-corpus-v1.json"
MARKDOWN_PATH = BASE / "JOHANNESBURG_WATER_WORKFORCE_PILOT_V1.md"

EXPECTED_SCHEMA = "mycelix.civic-resilience.johannesburg-water-workforce-pilot.v1"
EXPECTED_CORPUS_SCHEMA = "mycelix.civic-resilience.johannesburg-water-workforce-pilot-synthetic-corpus.v1"
EXPECTED_CORPUS_BLOB_SHA = "65273f33994cf1c8829004d974aee0821e61e7cf"
EXPECTED_SOURCE_IDS = ["JW-1","JW-3","COJ-1","COJ-SDBIP-2026-27","GOV-PWR-WTR-1","JW-2","SA-1","JOBS-1","LAW-1","SA-JOBS-2","SA-JOBS-3","SA-WAGE-2026-1","COJ-PROC-1","PIKITUP-PROC-1","PIKITUP-CURRENT-INDEX-1","SA-TREASURY-OCDS-REGISTRY-1","SA-ETENDER-PORTAL-1","OCDS-STANDARD-1","PIKITUP-RFQ-INDEX-1","PIKITUP-PIK016-DOC-1","SA-MUNI-FIN-API-1","SA-TREASURY-REPMAINT-Q2-2025-26-1","JHB-REPMAINT-V2-VIEW-1"]
EXPECTED_GATES = ["G0", "G1", "G2", "G3", "G4", "G5"]
EXPECTED_INVARIANTS = [
    "InternalServiceIssueId != external reference number",
    "Local mirror != external current status",
    "Submission != external acceptance",
    "External acceptance != entitlement",
    "RepairClaimed != IndependentlyVerified",
    "Reported != Inspected",
    "Response != Restoration",
    "Notification != RepairWindow",
    "WorkingDays != CalendarDays",
    "ClosedSupportTicket != VerifiedServiceOutcome",
    "UnknownOrConflicted != Success",
]
EXPECTED_PROBES = {
    "issuer-reference-collision": ("IdentityAndAuthority", "KeepSeparate", "NamespacedReferencesRemainDistinct"),
    "same-source-possible-duplicate": ("Deduplication", "CandidateDuplicateOnly", "Ambiguous"),
    "stale-systemwide-bulletin": ("Currentness", "RecordObservationOnly", "StaleOrUnqualified"),
    "source-fetch-unavailable": ("Availability", "NeedsHumanReview", "UnavailableNotNegative"),
    "response-without-restoration": ("ServiceSemantics", "RecordResponseOnly", "ResponseObservedRestorationUnknown"),
    "working-days-calendar-conversion": ("ServiceSemantics", "DoNotCalculateProductionDeadline", "ClockUnqualified"),
    "unversioned-web-standard": ("ServiceSemantics", "SourceObservationOnly", "Unversioned"),
    "repair-claim-without-independent-evidence": ("CompletionVerification", "ClaimUnverified", "VerificationMissing"),
    "verified-repair-followed-by-repeat-fault": ("CorrectionAndReopening", "ReopenOrLinkNewEvent", "RepeatFaultUnderReview"),
    "unauthorized-sensor-feed": ("DataPermission", "BlockIngestion", "Unauthorized"),
    "duplicate-invoice-with-missing-delivery-evidence": ("ProcurementIntegrity", "HumanAuditReview", "AnomalyNotFinding"),
    "unfunded-work-pathway": ("EmploymentIntegrity", "DoNotAnnounceFundedPosition", "Unfunded"),
    "sensitive-location-publication": ("PrivacyAndAssetProtection", "BlockPublicationOrRedact", "DisclosureBlocked"),
    "demographic-risk-proxy": ("FairnessAndRights", "RejectAdverseAction", "UnsupportedAndProhibited"),
    "working-on-infrastructure-geography-mismatch": ("EmploymentIntegrity", "TreatAsPartnerDiscoveryOnly", "GeographyNotInAnnouncedLaunchScope"),
    "sdbip-cooperative-kpi-is-not-contract-award": ("EmploymentIntegrity", "PartnerDiscoveryOnlyNoContractAward", "PlanningTargetNotContract"),
    "national-epwp-target-vs-city-sdbip-target": ("EmploymentIntegrity", "PreserveSeparateTargetSeries", "DistinctProgrammeTargets"),
    "same-title-water-kpis-need-crosswalk": ("ServiceSemantics", "PreserveSeparateMetricSeries", "MetricCrosswalkUnqualified"),
    "planned-controller-rollout-date-is-not-completion": ("Currentness", "KeepRolloutStatusUnverified", "RolloutCompletionNotObserved"),
    "unknown-wage-regime-cannot-default-to-epwp": ("EmploymentIntegrity", "BlockPositionUntilWageReview", "EmploymentClassificationUnqualified"),
    "mixed-stage-tender-index-is-not-open-work": ("ProcurementIntegrity", "PartnerDiscoveryOnlyNoOpenNotice", "TenderStatusNotQualified"),
    "current-tender-header-without-dated-notice": ("ProcurementIntegrity", "RequireNoticeSpecificCurrentness", "ListingOnlyCurrentnessUnqualified"),
    "national-registry-absence-is-not-city-tender-absence": ("ProcurementIntegrity", "ContinueIssuerSpecificDiscovery", "NationalRegistryCoverageIncomplete"),
    "ocds-empty-status-is-not-open": ("ProcurementIntegrity", "HoldStatusUnqualified", "EmptyStatusAndClosingDate"),
    "award-is-not-contract-or-funded-role": ("EmploymentIntegrity", "RecordAwardOnly", "AwardObservedContractAndRoleFundingUnknown"),
    "duplicate-ocds-release-is-not-second-opportunity": ("ProcurementIntegrity", "DeduplicateExactReleaseForCounting", "IdenticalDuplicateRelease"),
    "pikitup-deadline-without-timezone-is-not-open-verified": ("ProcurementIntegrity", "KeepAtTenderNoticeObserved", "TimezoneAndAmendmentCurrentnessUnqualified"),
    "cidb-tender-requirement-is-not-cooperative-job-eligibility": ("EmploymentIntegrity", "RequireBidderAndScopeReview", "BidderAndSubcontractEligibilityUnverified"),
    "tender-document-fee-or-bribe-solicitation-is-not-official": ("ProcurementIntegrity", "BlockPaymentAndUseOfficialChannel", "SuspectedProcurementScamUnadjudicated"),
    "municipal-repairs-view-without-amount-type-is-not-zero": ("FinanceDataIntegrity", "HoldFinancialObservationUnqualified", "FinanceFiltersIncomplete"),
    "aggregate-repairs-spend-is-not-role-funding": ("EmploymentIntegrity", "AggregateContextOnly", "MunicipalAggregateNotProcurementEvidence"),
    "latest-api-quarter-is-not-report-period": ("FinanceDataIntegrity", "HoldCrossSnapshotComparison", "SnapshotsNotAligned"),
    "r-thousands-values-are-not-rands": ("FinanceDataIntegrity", "RequireUnitNormalization", "SourceReportsValuesInThousands"),
    "monthly-quarter-total-rounding-needs-review": ("FinanceDataIntegrity", "PreservePublishedTotalAndFlagReconciliation", "PublishedTotalAndRoundedMonthSumDifferByOneRThousand"),
    "published-snapshot-checksum-is-not-payload-verification": ("FinanceDataIntegrity", "RecordMetadataReceiptOnly", "PublishedChecksumOnly"),
    "current-rfq-block-with-past-deadline-is-not-open": ("ProcurementIntegrity", "RequireNoticeSpecificCurrentness", "ConflictingIssuerListingCurrentness"),
    "joint-water-power-statement-is-not-incident-causality": ("CausalityAndAuthority", "HypothesisOnlyNoIncidentCausalEdge", "CrossUtilityHypothesisOnly"),
}
REQUIRED_GUARDS = {
    "issuer-reference-collision": {"DoNotMergeByReferenceStringAlone", "ExternalReferenceIsNotInternalIdentity"},
    "same-source-possible-duplicate": {"DuplicateCandidateIsNotProofOfDuplicate", "DoNotInferPreciseLocation"},
    "stale-systemwide-bulletin": {"BulletinDoesNotResolveIndividualCase", "RetrievalDoesNotProveCurrentness"},
    "source-fetch-unavailable": {"FetchFailureIsNotSourceAbsence", "NoInferredCurrentStatus"},
    "response-without-restoration": {"ResponseIsNotRestoration", "NoFalseResolutionClaim"},
    "working-days-calendar-conversion": {"WorkingDaysAreNotCalendarDays", "MissingClockSemanticsFailsClosed"},
    "unversioned-web-standard": {"PublicSourceDoesNotEstablishAdoption", "NoProductionSlaCalculation"},
    "repair-claim-without-independent-evidence": {"RepairClaimIsNotVerifiedOutcome", "TicketClosureIsNotAuthority"},
    "verified-repair-followed-by-repeat-fault": {"DoNotDeleteEarlierEvidence", "PriorVerificationDoesNotProveCurrentHealth"},
    "unauthorized-sensor-feed": {"NoUnauthorizedProviderAccess", "DataAvailabilityDoesNotImplyPermission"},
    "duplicate-invoice-with-missing-delivery-evidence": {"AnomalyIsNotProofOfFraud", "NoAutomaticBlacklisting"},
    "unfunded-work-pathway": {"NoSpeculativePayroll", "OpportunityIsNotAJob"},
    "sensitive-location-publication": {"NoPublicSensitiveAssetCoordinates", "NoExploitableOperationalDetail"},
    "demographic-risk-proxy": {"PovertyIsNotEvidenceOfWrongdoing", "ModelFlagIsNotAdjudication"},
    "working-on-infrastructure-geography-mismatch": {"DoNotInferGautengPilotEligibility", "NationalProgrammeAnnouncementIsNotLocalFunding", "NoSpeculativePayroll"},
    "sdbip-cooperative-kpi-is-not-contract-award": {"SdbipPlanningKpiIsNotProcurementAward", "NoCooperativeContractWithoutOpenAuthorizedProcess", "NoSpeculativePayroll"},
    "national-epwp-target-vs-city-sdbip-target": {"NationalTargetIsNotCitySdbipTarget", "WorkOpportunityIsNotDurableJob", "DoNotReplacePublishedTarget"},
    "same-title-water-kpis-need-crosswalk": {"SameTitleDoesNotProveSameMetric", "UnqualifiedMetricCrosswalkFailsClosed", "NoProductionSlaCalculation"},
    "planned-controller-rollout-date-is-not-completion": {"PlannedCompletionDateIsNotEvidenceOfCompletion", "NoLaterConfirmationMeansUnknown", "SourceSearchScopeMustBeRecorded"},
    "unknown-wage-regime-cannot-default-to-epwp": {"WageRegimeRequiresQualifiedClassification", "EPWPRateIsNotDefaultForUnknownStatus", "NoSpeculativePayroll", "NoPositionBeforeFullFunding"},
    "mixed-stage-tender-index-is-not-open-work": {"TenderLandingPageIsDiscoveryOnly", "CaptureNoticeStageAndClosingDate", "NoAbsenceProofFromListing", "NoContractWithoutAwardAndAuthority", "NoSpeculativePayroll"},
    "current-tender-header-without-dated-notice": {"CurrentTendersHeadingDoesNotProveOpenStatus", "TenderIdSuffixIsNotDateEvidence", "RecordTenderScopeAndLifecycleFromNotice", "NoOpportunityFromUnrelatedTender", "NoAbsenceProofFromListing"},
    "national-registry-absence-is-not-city-tender-absence": {"RegistryCoverageExcludesSomeMunicipalAndSoeData", "MissingRecordIsNotEvidenceOfNoTender", "UseIssuerSpecificNotice", "NoAbsenceProofFromListing"},
    "ocds-empty-status-is-not-open": {"EmptyTenderStatusIsNotOpen", "UnknownIsNotOpen", "RequireAuthoritativeCurrentNotice", "NoAutomaticStatusPromotion"},
    "award-is-not-contract-or-funded-role": {"AwardIsNotSignedContract", "ContractValueIsNotRoleBudget", "NoPositionBeforeFullFunding", "QualifiedWageReviewRequired", "NoSpeculativePayroll"},
    "duplicate-ocds-release-is-not-second-opportunity": {"DeduplicateOnlyAfterExactReleaseIdentityAndContentMatch", "DoNotCountReleaseAsTenderProcess", "ConflictingContentRequiresHumanReview"},
    "pikitup-deadline-without-timezone-is-not-open-verified": {"TenderDeadlineNeedsTimezone", "FuturePublishedDeadlineAloneNotFullOpenGate", "CheckLatestAddendaBeforeOpenStatus", "NoAutomaticStagePromotion"},
    "cidb-tender-requirement-is-not-cooperative-job-eligibility": {"SpecificTenderRequirementsControl", "DoNotAssumeCooperativeEligibility", "ContractScopeDoesNotEqualRoleBudget", "NoPositionBeforeFullFunding"},
    "tender-document-fee-or-bribe-solicitation-is-not-official": {"TenderDocumentsAndCsdRegistrationAreFree", "UseIssuerPublishedChannels", "NoBribeOrPreferentialTreatment", "ReportThroughOfficialChannelOnly", "AllegationIsNotAdjudicatedFinding"},
    "municipal-repairs-view-without-amount-type-is-not-zero": {"StaticTextCaptureIsNotRenderedTableState", "ClientSideDefaultsMustBeCapturedForReproducibility", "AmountTypeAndPeriodMustBePinnedForValueClaims", "NoDisplayedCellIsNotMeasuredZero", "AggregateSpendIsNotTenderFunding"},
    "aggregate-repairs-spend-is-not-role-funding": {"AggregateFinanceIsNotTenderEvidence", "ContractValueIsNotRoleBudget", "NoPositionBeforeFullFunding", "Section71FunctionAndPeriodMustBePinned"},
    "latest-api-quarter-is-not-report-period": {"APIAndSection71ReportSnapshotsMayDiffer", "ReportingPeriodAndSnapshotMustMatch", "AmountTypeAndPeriodMustBePinned", "NoAggregateFundingClaim"},
    "r-thousands-values-are-not-rands": {"ReportedUnitMustTravelWithValue", "DoNotPresentThousandsAsRands", "NormalizeBeforeCrossSourceCompare", "NoFundingInferenceFromUnits"},
    "monthly-quarter-total-rounding-needs-review": {"DoNotOverwritePublishedAggregate", "KeepSourceUnitAttached", "FlagArithmeticDifferenceForReview", "NoSilentReconciliation"},
    "published-snapshot-checksum-is-not-payload-verification": {"PublishedChecksumIsNotLocalChecksumVerification", "RecordSnapshotTimestampSeparatelyFromFinancialPeriod", "NoValueClaimWithoutDataPayload", "PayloadBytesMustMatchPublishedDigest"},
    "current-rfq-block-with-past-deadline-is-not-open": {"CurrentRfqHeadingDoesNotOverrideClosingDate", "PastPublishedDeadlineIsNotOpenWithoutExtensionEvidence", "ListingPlacementIsNotLifecycleStatus", "LatestAddendaMustBeChecked", "NoAutomaticStagePromotion"},
    "joint-water-power-statement-is-not-incident-causality": {"OfficialContextIsNotIncidentEvidence", "NoCausalEdgeWithoutIncidentSpecificEvidence", "NoLiveStatusFromHistoricStatement", "NoUnauthorizedTelemetry"},
}

EXPECTED_BASE_KEYS = {
    "automated_external_write", "automated_closure", "production_sla_calculation",
    "criminality_conclusion", "real_resident_data_present",
    "precise_sensitive_asset_coordinates_present", "disposition",
    "issue_status_effect", "required_human_review", "source_status",
}


def fail(message: str) -> None:
    raise ValueError(message)


def unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in pairs:
        if key in out:
            fail(f"duplicate JSON key: {key}")
        out[key] = value
    return out


def reject_constant(value: str) -> None:
    fail(f"non-standard/non-finite JSON constant: {value}")


def load_json(path: Path) -> tuple[Any, bytes]:
    if not path.is_file():
        fail(f"required file missing: {path.relative_to(ROOT)}")
    raw = path.read_bytes()
    try:
        value = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=unique_object,
            parse_constant=reject_constant,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        fail(f"{path.relative_to(ROOT)}: invalid UTF-8/JSON: {exc}")
    return value, raw


def exact_keys(value: Any, expected: set[str], where: str) -> None:
    if not isinstance(value, dict) or set(value) != expected:
        found = sorted(value) if isinstance(value, dict) else type(value).__name__
        fail(f"{where}: exact-key mismatch; expected {sorted(expected)}, got {found}")


def git_blob_sha1(raw: bytes) -> str:
    header = f"blob {len(raw)}\0".encode("ascii")
    return hashlib.sha1(header + raw).hexdigest()


def validate() -> None:
    if not MARKDOWN_PATH.is_file():
        fail("narrative pilot specification missing")

    manifest, _ = load_json(MANIFEST_PATH)
    corpus, corpus_raw = load_json(CORPUS_PATH)

    exact_keys(manifest, {
        "schema", "schema_version", "research_snapshot_date", "claim_ceiling",
        "maturity", "parent_issue", "markdown_spec", "pilot", "source_ledger",
        "event_model", "job_pathways", "payer_and_job_quality_gate",
        "crime_profitability_controls", "evaluation", "gates", "stop_conditions",
        "prohibited_capabilities", "synthetic_corpus", "pilot_selection_matrix",
        "procurement_discovery_gate", "procurement_evidence_lifecycle",
    }, "manifest")
    if manifest["schema"] != EXPECTED_SCHEMA or manifest["schema_version"] != 1:
        fail("manifest schema/version mismatch")
    if manifest["claim_ceiling"] != "ResearchSpecificationOnly":
        fail("manifest must preserve research-only claim ceiling")
    if manifest["markdown_spec"] != "mycelix-workspace/docs/civic-resilience/JOHANNESBURG_WATER_WORKFORCE_PILOT_V1.md":
        fail("manifest narrative path drift")

    selection = manifest["pilot_selection_matrix"]
    exact_keys(selection, {"selection_status", "candidates", "decision_rule", "selected_candidate", "public_citywide_claim_authorized"}, "pilot selection matrix")
    if selection.get("selection_status") != "NotSelected;PartnerDiscoveryRequired" or selection.get("selected_candidate") is not None:
        fail("pilot candidate must remain unselected until partner-readiness evidence exists")
    if selection.get("public_citywide_claim_authorized") is not False:
        fail("no citywide pilot claim is authorized")
    candidate_ids = [c.get("candidate_id") for c in selection.get("candidates", []) if isinstance(c, dict)]
    if candidate_ids != ["water_fault_coordination", "pikitup_cooperative_basic_removal"]:
        fail("pilot candidate set/order drift")
    for candidate in selection["candidates"]:
        if candidate.get("funding_or_contract_confirmed") is not False:
            fail(f"{candidate['candidate_id']}: do not claim funding/contract")
    if selection["candidates"][1].get("readiness") != "PartnerDiscoveryOnly":
        fail("Pikitup option must remain a partner-discovery lead, not a live pilot")
    if not any("written data-use permission" in str(rule).lower() for rule in selection["decision_rule"]):
        fail("pilot selection must require written permissions")

    pilot = manifest["pilot"]
    exact_keys(pilot, {
        "id", "title", "rationale", "geography", "operating_mode",
        "live_provider_calls_authorized", "external_writes_authorized",
        "automated_dispatch_authorized", "automated_closure_authorized",
        "production_sla_calculation_authorized", "telemetry_access_authorized",
        "payer_identified", "operational_partner_identified", "funded_positions_confirmed",
    }, "pilot")
    for key in (
        "live_provider_calls_authorized", "external_writes_authorized",
        "automated_dispatch_authorized", "automated_closure_authorized",
        "production_sla_calculation_authorized", "telemetry_access_authorized",
        "payer_identified", "operational_partner_identified", "funded_positions_confirmed",
    ):
        if pilot[key] is not False:
            fail(f"pilot capability/commitment must remain false: {key}")
    if pilot["operating_mode"] != "SyntheticFirstThenAuthorizedReadOnlyShadow":
        fail("pilot must remain synthetic-first/read-only shadow")

    source_ids = [s.get("source_id") for s in manifest["source_ledger"] if isinstance(s, dict)]
    if source_ids != EXPECTED_SOURCE_IDS:
        fail(f"source ledger ids/order drift: {source_ids}")
    for source in manifest["source_ledger"]:
        if not isinstance(source.get("source_url"), str) or not source["source_url"].startswith("https://"):
            fail(f"source {source.get('source_id')}: expected HTTPS source URL")
    source_by_id = {s["source_id"]: s for s in manifest["source_ledger"]}
    rollout_source = source_by_id["JW-1"]
    if rollout_source.get("planned_completion_window") != "EndOfSeptember2026":
        fail("Johannesburg Water rollout plan date drift")
    recheck = rollout_source.get("currentness_recheck", {})
    if recheck.get("status") != "NotIndependentlyVerified" or recheck.get("absence_of_notice_means_completion_false") is not False:
        fail("rollout currentness must remain unverified without treating missing notice as non-completion")
    recent_notice = source_by_id["JW-3"]
    if recent_notice.get("source_kind") != "OfficialSystemUpdateNotice" or recent_notice.get("publication_date") != "2026-10-06":
        fail("later water-system notice source identity/date drift")
    if recent_notice.get("smart_controller_rollout_completion_status_in_this_notice") != "NotObserved":
        fail("do not infer rollout completion from unrelated operational notices")
    sdbip_source = source_by_id["COJ-SDBIP-2026-27"]
    if sdbip_source.get("source_kind") != "OfficialServiceDeliveryBudgetImplementationPlan" or sdbip_source.get("pdf_page_count") != 151:
        fail("current-year SDBIP source identity/page count drift")
    if sdbip_source.get("source_observation_status") != "OfficialPDFTextObserved;PrintedPages28And58VisuallyReviewed":
        fail("SDBIP source review ceiling drift")
    if sdbip_source.get("water_loss_kpi", {}).get("kpi_number") != 17:
        fail("SDBIP water-loss KPI identity drift")
    water_kpi = sdbip_source["water_loss_kpi"]
    summary = water_kpi["summary_table"]
    if summary.get("target_2026_27") != "200 293Ml" or summary.get("quarterly_targets_2026_27") != ["204 270Ml", "203 054Ml", "201 839Ml", "200 293Ml"]:
        fail("SDBIP water-loss target literal/quarter order drift")
    if water_kpi.get("summary_unit_vs_definition_status") != "ConflictOrAmbiguityObserved;NoCrosswalkApplied":
        fail("summary-unit/technical-definition conflict must remain unresolved")
    if water_kpi.get("production_loss_calculation_authorized") is not False or water_kpi.get("case_level_sla_equivalence_authorized") is not False:
        fail("annual water-loss KPI cannot authorize production calculations or case SLA equivalence")
    technical = water_kpi.get("technical_definition", {})
    if technical.get("source_page_printed") != 58 or "Number of service connections" not in technical.get("formula_text", ""):
        fail("SDBIP technical formula or printed page pin drift")
    green = sdbip_source.get("green_economy_job_kpi", {})
    if green.get("kpi_number") != 55 or green.get("target_2026_27_projects") != 6 or green.get("quarterly_targets_2026_27_projects") != [1, 1, 2, 2]:
        fail("green-economy project KPI literal drift")
    if green.get("visually_reviewed") is not False:
        fail("do not claim visual review of the green-economy table")
    epwp = sdbip_source.get("epwp_context", {})
    if epwp.get("nationally_set_phase5_target_for_city_work_opportunities") != 50030 or epwp.get("nationally_set_target_2026_27_work_opportunities_for_city") != 9996:
        fail("nationally set/apportioned EPWP Phase 5 target literal drift")
    city_epwp = epwp.get("city_sdbip_kpi24", {})
    if city_epwp.get("kpi_number") != 24 or city_epwp.get("target_2026_27") != 12500:
        fail("City SDBIP KPI 24 EPWP target identity/value drift")
    if city_epwp.get("quarterly_targets_2026_27") != [3500, 7000, 9800, 12500]:
        fail("City SDBIP quarterly EPWP target series drift")
    if city_epwp.get("target_2026_27") == epwp.get("nationally_set_target_2026_27_work_opportunities_for_city"):
        fail("distinct national/City EPWP target series unexpectedly collapsed")
    if city_epwp.get("visually_reviewed") is not False:
        fail("do not claim visual review of City SDBIP KPI 24 page")
    if "25% increment" not in city_epwp.get("relation_to_national_target", ""):
        fail("City/national EPWP target relation was not recorded")
    if epwp.get("work_opportunity_equals_durable_job") is not False or epwp.get("mycelix_funding_or_eligibility_confirmed") is not False:
        fail("EPWP opportunities must not be promoted to durable jobs or pilot funding")
    if epwp.get("visually_reviewed") is not False:
        fail("do not claim visual review of the EPWP footnote")
    water_outcome = sdbip_source.get("water_loss_outcome_indicator", {})
    if water_outcome.get("indicator_kpi_number") != 52 or water_outcome.get("circular88_reference") != "WS5.2":
        fail("second water-loss outcome indicator identity drift")
    if water_outcome.get("target_values_as_extracted") != ["196 099Ml", "188 163Ml", "180 274Ml", "173 495Ml"]:
        fail("second water-loss target series literal drift")
    if water_outcome.get("year_to_value_mapping") != "NotFullyBoundFromTextLayer":
        fail("do not infer financial-year mapping from an ambiguous text layer")
    if water_outcome.get("crosswalk_status") != "NotQualified;PreserveSeparateMetricSeries" or water_outcome.get("visually_reviewed") is not False:
        fail("second same-title water-loss series must remain unqualified and text-layer-only")
    if water_outcome.get("same_title_as_business_plan_kpi17") is not True:
        fail("duplicate water-loss title must be explicitly linked as a crosswalk concern")
    pikitup_kpi = sdbip_source.get("pikitup_cooperative_service_kpi", {})
    if pikitup_kpi.get("kpi_number") != 15 or pikitup_kpi.get("operational_entity") != "PIKITUP":
        fail("Pikitup basic-removal KPI identity drift")
    if pikitup_kpi.get("target_2026_27_percent") != 100 or pikitup_kpi.get("quarterly_targets_2026_27_percent") != [100, 100, 100, 100]:
        fail("Pikitup target literals drift")
    if pikitup_kpi.get("visually_reviewed") is not False:
        fail("do not claim visual review of SDBIP printed page 27")
    forbidden_claims = set(pikitup_kpi.get("prohibited_inference", []))
    if not {
        "Specific tender or contract is open",
        "Any supplier/cooperative has been awarded a contract",
        "Budget or work package is available to Mycelix",
        "100% outcome has been achieved",
    }.issubset(forbidden_claims):
        fail("Pikitup planning KPI must not imply contract, funding, or achieved outcome")
    programme_source = source_by_id["SA-JOBS-3"]
    if programme_source.get("source_kind") != "OfficialPilotLaunchSpeech" or programme_source.get("publication_date") != "2026-05-04":
        fail("Working on Infrastructure launch source identity/date drift")
    if programme_source.get("announced_initial_pilot_provinces") != ["KwaZulu-Natal", "Limpopo"]:
        fail("initial programme geography drift")
    if programme_source.get("joburg_gauteng_participation_explicitly_confirmed") is not False:
        fail("do not infer Johannesburg/Gauteng participation from initial launch")
    if programme_source.get("local_johannesburg_funding_or_slots_verified") is not False or programme_source.get("allowed_to_claim_active_joburg_pilot") is not False:
        fail("national pilot announcement must not become local funding or participation evidence")
    if programme_source.get("intended_programme_duration_months") != {"minimum": 8, "maximum": 10}:
        fail("structured programme-duration source claim drift")
    if programme_source.get("recruitment_platform") != "SAYouth.mobi" or programme_source.get("platform_support_partner") != "Harambee":
        fail("official programme recruitment platform record drift")
    wage_source = source_by_id["SA-WAGE-2026-1"]
    if wage_source.get("source_kind") != "OfficialWageAnnouncement" or wage_source.get("publication_date") != "2026-02-03" or wage_source.get("effective_date") != "2026-03-01":
        fail("2026 wage source identity/effective date drift")
    if "R30.23 per hour" not in " ".join(wage_source.get("claims_recorded", [])) or "R16.62 per hour" not in " ".join(wage_source.get("claims_recorded", [])):
        fail("ordinary/EPWP wage-rate distinctions must be explicit")
    wage_gate = manifest["payer_and_job_quality_gate"].get("wage_regime_gate", {})
    if wage_gate.get("source_id") != "SA-WAGE-2026-1" or wage_gate.get("ordinary_national_minimum_zar_per_hour") != 30.23 or wage_gate.get("epwp_minimum_zar_per_hour") != 16.62:
        fail("wage gate/source rates drift")
    if wage_gate.get("automated_classification_authorized") is not False or wage_gate.get("epwp_rate_is_not_default_for_unknown_classification") is not True or wage_gate.get("hourly_rate_alone_is_complete_cost") is not False:
        fail("wage regime must require qualified human classification and full-cost accounting")
    required_paid_work = {
        "Named legal employer or authorized contracting entity",
        "Written funded scope and lawful engagement route",
        "Qualified human confirmation of worker and programme classification",
        "Full-period committed budget, payroll route and responsible payment owner",
        "Budget for applicable employer costs, supervision, tools/PPE, transport, training and credentials, safety/insurance, administration and evaluation",
    }
    if not required_paid_work.issubset(set(manifest["payer_and_job_quality_gate"].get("required_before_any_role_is_announced", []))):
        fail("paid-work integrity gate is missing mandatory role/funding/cost controls")

    procurement = manifest["procurement_discovery_gate"]
    if procurement.get("source_ids") != ["COJ-PROC-1", "PIKITUP-PROC-1", "PIKITUP-CURRENT-INDEX-1", "SA-TREASURY-OCDS-REGISTRY-1", "SA-ETENDER-PORTAL-1", "OCDS-STANDARD-1", "PIKITUP-RFQ-INDEX-1", "PIKITUP-PIK016-DOC-1"] or procurement.get("listing_page_alone_proves_open_opportunity") is not False:
        fail("procurement discovery sources or landing-page authority limit drift")
    if procurement.get("current_finding") != "NoMatchingOpenBasicRemovalOrCooperativeWorkPackageIdentifiedInInspectedListings;NotProofOfAbsence":
        fail("bounded procurement non-observation must not become proof of absence")
    notice_fields = set(procurement.get("required_notice_record_fields", []))
    if not {"issuer_and_tender_id", "notice_url", "lifecycle_status", "closing_at_and_timezone", "latest_addenda_and_q_and_a", "award_or_signed_contract_evidence", "named_payer_and_payment_terms", "checked_at"}.issubset(notice_fields):
        fail("procurement gate lacks tender-specific status, timing, authority or payer evidence")

    city_proc = source_by_id["COJ-PROC-1"]
    if city_proc.get("source_kind") != "OfficialProcurementIndex" or city_proc.get("checked_on") != "2026-10-14" or city_proc.get("relevant_open_basic_removal_or_water_work_package_identified_in_inspection") is not False:
        fail("City procurement index observation/currentness drift")
    pikitup_proc = source_by_id["PIKITUP-PROC-1"]
    if pikitup_proc.get("source_kind") != "OfficialTenderIndex" or pikitup_proc.get("checked_on") != "2026-10-14" or pikitup_proc.get("relevant_open_basic_removal_or_cooperative_work_package_identified_in_inspection") is not False:
        fail("Pikitup tender-index observation/currentness drift")
    if "not proof of absence" not in pikitup_proc.get("search_ceiling", "").lower():
        fail("tender listing non-observation must not be reported as proof no tender exists")

    current_index = source_by_id["PIKITUP-CURRENT-INDEX-1"]
    if current_index.get("page_label") != "Current Tenders" or current_index.get("checked_on") != "2026-10-14":
        fail("City-hosted Pikitup current-tender listing identity/date drift")
    if current_index.get("visible_rows") != [
        {"tender_reference": "PIK259/17", "description": "Appointment of an Accredited Service Provider for Servicing Fire Equipment"},
        {"tender_reference": "PIK255/17", "description": "Appointment of a Supplier for the Supply and Delivery of 4x Drones as Depicted Under Annexure A"},
    ]:
        fail("visible City-hosted Pikitup tender listing rows drift")
    if current_index.get("visible_publication_or_closing_dates") is not False or current_index.get("visible_lifecycle_status") is not False:
        fail("listing without dates/status must remain currentness-unqualified")
    if current_index.get("matching_basic_removal_or_cooperative_work_package_in_visible_rows") is not False:
        fail("unrelated equipment tenders cannot establish a cooperative/basic-removal work package")
    if "not independently interpreted as a date" not in current_index.get("search_ceiling", "").lower():
        fail("do not infer publication date from tender reference suffix")
    current_obs = procurement.get("current_index_observations", {})
    if current_obs.get("source_id") != "PIKITUP-CURRENT-INDEX-1" or current_obs.get("lifecycle_status_or_closing_date_visible") is not False or current_obs.get("relevant_work_package_observed") is not False:
        fail("procurement current-index observation must require notice-specific currentness")

    cross_utility = source_by_id["GOV-PWR-WTR-1"]
    if cross_utility.get("publication_date") != "2026-09-10" or cross_utility.get("source_kind") != "OfficialJointStatement":
        fail("joint water/power government statement source identity/date drift")
    if not any("Incident-level causal edge" in str(x) for x in cross_utility.get("prohibited_inference", [])):
        fail("cross-utility context cannot be promoted to incident-level causality")
    municipal_finance = source_by_id["SA-MUNI-FIN-API-1"]
    if municipal_finance.get("latest_published_data_period") != "2026 Q2" or "Repairs and maintenance (v2)" not in municipal_finance.get("covered_financial_data", []):
        fail("Municipal Money currentness/dataset scope drift")
    if "not procurement" not in municipal_finance.get("use_limit", "").lower() or "not role-specific" not in municipal_finance.get("use_limit", "").lower():
        fail("aggregate municipal finance data must not be promoted to tender or role funding")
    bulk_meta = municipal_finance.get("bulk_snapshot_metadata", {})
    if bulk_meta.get("observed_on") != "2026-10-15" or bulk_meta.get("published_snapshot_last_updated") != "2026-07-30_15-59-35":
        fail("Municipal Money bulk snapshot timestamp/currentness pin drift")
    if bulk_meta.get("csv_file_name") != "repairs_maintenance_facts_v2_2026-07-30_15-59-35.csv" or bulk_meta.get("published_file_size_bytes") != 4181869:
        fail("Municipal Money bulk snapshot file identity/size drift")
    if bulk_meta.get("published_md5") != "69d3dbe54679a0d1e3ec45c8b62f477f" or bulk_meta.get("published_sha1") != "32db68c02d2fd704e60ca95bc2fdb7c65188efb6":
        fail("Municipal Money published snapshot checksum receipt drift")
    if bulk_meta.get("payload_bytes_observed_in_this_pass") is not False or bulk_meta.get("payload_checksum_independently_verified") is not False:
        fail("published checksum metadata must not imply local payload verification")
    if "must not be substituted for a pinned financial year/period/amount type" not in bulk_meta.get("interpretation", ""):
        fail("bulk snapshot timestamp must remain distinct from reporting period")
    finance_view = source_by_id["JHB-REPMAINT-V2-VIEW-1"]
    if finance_view.get("checked_on") != "2026-10-15" or finance_view.get("selected_amount_type") != "NotExposedInStaticTextCapture" or finance_view.get("numeric_amount_recorded") is not False:
        fail("Johannesburg finance view must preserve static-capture limits and no-number observation")
    if finance_view.get("page_text_snapshot_kind") != "StaticTextCapture;ClientSideJavascriptNotExecutedByReader" or finance_view.get("client_rendered_selection_state_observed") is not False or finance_view.get("rendered_data_cells_observed") is not False:
        fail("static page text must not be presented as the rendered client-side filter/table state")
    if "not a measured zero" not in finance_view.get("interpretation", "").lower():
        fail("unqualified static finance capture must not be treated as measured zero")
    if "client-side logic can supply a year and amount type" not in finance_view.get("interpretation", "").lower():
        fail("finance view must preserve the client-side default-selection limitation")
    if procurement.get("finance_context_source_ids") != ["SA-MUNI-FIN-API-1", "SA-TREASURY-REPMAINT-Q2-2025-26-1", "JHB-REPMAINT-V2-VIEW-1"]:
        fail("aggregate municipal finance context source pin drift")
    q2_report = source_by_id["SA-TREASURY-REPMAINT-Q2-2025-26-1"]
    values = q2_report.get("reported_values_r_thousands", {})
    if q2_report.get("checked_on") != "2026-10-15" or q2_report.get("figures_finalized_as") != "2026-01-30" or q2_report.get("report_period") != "2025-26 second quarter; quarter ended 2025-12-31":
        fail("Treasury Section 71 report date/period pin drift")
    if q2_report.get("municipality", {}).get("demarcation_code") != "JHB" or q2_report.get("unit_as_printed") != "R thousands":
        fail("Treasury report municipality/unit pin drift")
    if values.get("original_budget") != 3467209 or values.get("adjusted_budget") != 3467209 or values.get("ytd_actual_through_december") != 1182846 or values.get("quarter_2_october_december_actual_as_published") != 620893:
        fail("Treasury report published budget/actual values drift")
    if values.get("october_december_monthly_arithmetic_sum") != 620894 or values.get("monthly_sum_minus_published_quarter_total") != 1:
        fail("Treasury report month-versus-published-quarter arithmetic observation drift")
    if q2_report.get("api_snapshot_crosscheck") != "NotMatchedInThisPass; the current Municipal Money table view did not have year/period/amount type pinned, and the API/report values were not queried under an identical set of filters and snapshot.":
        fail("Treasury report must remain explicitly unmatched to the API snapshot")

    treasury_registry = source_by_id["SA-TREASURY-OCDS-REGISTRY-1"]
    if treasury_registry.get("dataset_date_range") != "2017-02 through 2026-09" or treasury_registry.get("last_retrieved_by_registry") != "2026-09-20":
        fail("National Treasury OCDS registry snapshot date range/currentness drift")
    if treasury_registry.get("procurement_process_identifier_prefix") != "ocds-9t57fa":
        fail("National Treasury OCID prefix drift")
    if "municipal governments and state-owned entities are excluded unless they voluntarily share data" not in treasury_registry.get("coverage_limit", ""):
        fail("registry coverage limitation must be explicit")
    if not {"Empty tender/status values", "A few duplicate releases with identical release IDs and content"}.issubset(set(treasury_registry.get("known_quality_issues", []))):
        fail("registry status/duplicate-release data-quality risks must be explicit")
    ocds_standard = source_by_id["OCDS-STANDARD-1"]
    if ocds_standard.get("documented_stages") != ["Tender", "Award", "Contract", "Implementation"] or ocds_standard.get("process_join_key") != "OCID":
        fail("OCDS lifecycle vocabulary/identity record drift")

    lifecycle = manifest["procurement_evidence_lifecycle"]
    if lifecycle.get("standard_source_id") != "OCDS-STANDARD-1" or lifecycle.get("national_registry_source_id") != "SA-TREASURY-OCDS-REGISTRY-1":
        fail("procurement evidence ladder source pins drift")
    leads = lifecycle.get("research_leads", [])
    if len(leads) != 1:
        fail("expected exactly one bounded real-world procurement discovery lead")
    lead = leads[0]
    if lead.get("source_ids") != ["PIKITUP-RFQ-INDEX-1", "PIKITUP-PIK016-DOC-1"] or lead.get("evidence_stage") != "TenderNoticeObserved":
        fail("PIKITUP PIK016 discovery lead source/stage drift")
    if lead.get("closing_date_as_published") != "2026-10-19" or lead.get("closing_time_as_published") != "11:00":
        fail("PIK016 closing-date/time observation drift")
    if lead.get("closing_timezone") != "NotSpecifiedInSource" or lead.get("open_status_verified") is not False:
        fail("PIK016 must not be over-promoted to verified-open without timezone and currentness review")
    if lead.get("minimum_cidb_grading") != "1GB" or lead.get("basic_removal_or_cooperative_scope") is not False:
        fail("PIK016 scope/eligibility must not be conflated with cooperative/basic-removal work")
    if lead.get("role_funding_confirmed") is not False or lead.get("mycelix_or_cooperative_eligibility_assessed") is not False or lead.get("award_or_signed_contract_observed") is not False:
        fail("procurement discovery lead must not imply award, employer, or funded role")
    rfq = source_by_id["PIKITUP-RFQ-INDEX-1"]
    if rfq.get("checked_on") != "2026-10-15" or rfq.get("listing_entry", {}).get("tender_reference") != "PIK 016/2026-2027":
        fail("issuer-hosted RFQ index source/date/reference drift")
    issuer_recheck = rfq.get("issuer_page_recheck", {})
    if issuer_recheck.get("checked_on") != "2026-10-15" or issuer_recheck.get("visible_current_rfqs_block_contains_this_tender") is not False or issuer_recheck.get("same_page_lists_tender_elsewhere") is not True:
        fail("conflicting Pikitup RFQ index placement must remain lifecycle-unqualified")
    if issuer_recheck.get("pikitup_specific_addendum_row_observed_in_page_text") is not False or "does not prove that no later addendum exists" not in issuer_recheck.get("addenda_search_ceiling", ""):
        fail("absence of an observed PIK016 addendum row must not be treated as exhaustive proof")
    current_rows = issuer_recheck.get("visible_current_rfqs_block_examples", [])
    if not any(row.get("tender_reference") == "PIK 018/2026-2027" and row.get("published_closing_date") == "2026-10-08" for row in current_rows):
        fail("stale-dated PIK018 row must be preserved as conflicting Current RFQs evidence")
    if not any(row.get("tender_reference") == "PIK 014/2026-2027" and row.get("published_closing_date") == "2026-09-17" for row in current_rows):
        fail("stale-dated PIK014 row must be preserved as conflicting Current RFQs evidence")
    if issuer_recheck.get("current_block_reliability") != "HeadingAndMembershipAreNotReliableLifecycleProof;RowsWithPublishedDeadlinesAlreadyPastRemainVisible":
        fail("Current RFQs heading must not override dated notice evidence")
    if issuer_recheck.get("official_pdf_opened") is not True or issuer_recheck.get("official_pdf_pages_1_and_2_visually_reviewed") is not True or issuer_recheck.get("lifecycle_conclusion") != "NotQualified;DoNotPromoteToTenderOpenVerified":
        fail("official tender recheck must record visual review and block unsupported open-status promotion")
    tender_doc = source_by_id["PIKITUP-PIK016-DOC-1"]
    if tender_doc.get("checked_on") != "2026-10-15" or tender_doc.get("pdf_extraction_status") != "OfficialPdfTextObserved;Pages1And2VisuallyReviewedOn2026-10-15":
        fail("official tender PDF review evidence/date drift")
    if "visible Current RFQs block" not in tender_doc.get("open_status_assessment", "") or "Do not promote to TenderOpenVerified" not in tender_doc.get("open_status_assessment", ""):
        fail("current-index grouping conflict must block open-status promotion")
    if tender_doc.get("minimum_cidb_grading") != "1GB" or tender_doc.get("closing_timezone") != "NotSpecifiedInSource" or tender_doc.get("role_funding_or_mycelix_eligibility_confirmed") is not False:
        fail("tender document eligibility/timezone authority limits drift")
    if not {"Bid/RFQ documents are issued free of charge", "No payment is required for Central Supplier Database registration", "Bidders are warned against bribery and promises of preferential treatment"}.issubset(set(tender_doc.get("anti_fraud_instructions_observed", []))):
        fail("official Pikitup anti-fraud safeguards missing from source record")
    expected_lifecycle_stages = ["IndexOnly", "TenderNoticeObserved", "TenderOpenVerified", "AwardObserved", "ContractObserved", "RoleFundingCommitted", "PaidWorkVerified"]
    if [stage.get("stage") for stage in lifecycle.get("stages", [])] != expected_lifecycle_stages:
        fail("procurement evidence lifecycle stage order drift")
    transition_policy = lifecycle.get("state_transition_policy", {})
    if transition_policy.get("automatic_stage_promotion_authorized") is not False or transition_policy.get("role_funding_is_distinct_from_contract_value") is not True or transition_policy.get("award_is_not_contract") is not True or transition_policy.get("contract_is_not_payment") is not True:
        fail("procurement evidence stages may not be automatically promoted or conflated")
    coverage_gate = lifecycle.get("national_registry_coverage_gate", {})
    if coverage_gate.get("absence_from_registry_proves_no_opportunity") is not False or coverage_gate.get("municipal_and_state_owned_entity_records_may_be_missing_unless_voluntarily_shared") is not True:
        fail("National Treasury registry absence must remain inconclusive for City/utility discovery")
    quality_gate = lifecycle.get("known_data_quality_gate", {})
    if quality_gate.get("empty_tender_status_may_be_treated_as_open") is not False or quality_gate.get("duplicate_identical_release_counts_as_second_opportunity") is not False or quality_gate.get("conflicting_same_release_identity_is_silently_overwritten") is not False:
        fail("OCDS data-quality failures must not create open tenders or overwrite conflicts")
    privacy_gate = lifecycle.get("privacy_and_integrity", {})
    if privacy_gate.get("automated_bidding_or_supplier_contact_authorized") is not False or privacy_gate.get("automated_criminality_or_corruption_conclusion_authorized") is not False:
        fail("procurement discovery cannot authorize bidding/contact or automated accusation")
    job_source = source_by_id["SA-JOBS-2"]
    if job_source.get("geography_scope") != "South Africa nationally":
        fail("youth employment source must be labelled national, not Johannesburg-specific")
    if job_source.get("local_johannesburg_estimate_claimed") is not False:
        fail("national labour statistics must not be promoted to a Johannesburg estimate")

    event_model = manifest["event_model"]
    exact_keys(event_model, {"record_identity", "required_event_fields", "states", "non_equivalence_invariants"}, "event model")
    if event_model["non_equivalence_invariants"] != EXPECTED_INVARIANTS:
        fail("event non-equivalence invariants changed without validator update")
    if event_model["record_identity"] != "InternalServiceIssueIdSeparateFromExternalReferences":
        fail("external references must not become internal identity")

    if [g.get("gate") for g in manifest["gates"]] != EXPECTED_GATES:
        fail("phase gate IDs/order drift")
    if len(manifest["gates"]) != 6:
        fail("expected six phase gates")
    for gate in manifest["gates"]:
        if not isinstance(gate.get("required"), list) or not gate["required"]:
            fail(f"{gate.get('gate')}: phase gate needs at least one required condition")

    evaluation = manifest["evaluation"]
    if evaluation.get("numerical_targets_invented") is not False or evaluation.get("citywide_causal_effect_claimed") is not False:
        fail("no invented thresholds or citywide causal claims allowed")
    if evaluation.get("baseline_required") is not True:
        fail("evaluation must require a baseline")

    exact_keys(corpus, {
        "schema", "corpus_version", "snapshot_date", "maturity", "synthetic_only",
        "no_real_resident_data", "no_real_case_references", "no_precise_sensitive_asset_locations",
        "intended_use", "expected_probe_count", "probes",
    }, "synthetic corpus")
    if corpus["schema"] != EXPECTED_CORPUS_SCHEMA or corpus["corpus_version"] != 1:
        fail("synthetic corpus schema/version mismatch")
    for flag in ("synthetic_only", "no_real_resident_data", "no_real_case_references", "no_precise_sensitive_asset_locations"):
        if corpus[flag] is not True:
            fail(f"synthetic corpus safety flag must be true: {flag}")
    if corpus["maturity"] != "SyntheticSpecificationOnly":
        fail("synthetic corpus maturity claim drift")
    if corpus["expected_probe_count"] != len(EXPECTED_PROBES) or len(corpus["probes"]) != corpus["expected_probe_count"]:
        fail("synthetic probe count mismatch")
    if git_blob_sha1(corpus_raw) != EXPECTED_CORPUS_BLOB_SHA:
        fail("corpus Git blob SHA does not match frozen reference; update manifest only after review")
    if manifest["synthetic_corpus"].get("git_blob_sha") != EXPECTED_CORPUS_BLOB_SHA:
        fail("manifest-pinned corpus blob SHA mismatch")
    if manifest["synthetic_corpus"].get("path") != "mycelix-workspace/docs/civic-resilience/johannesburg-water-workforce-pilot-synthetic-corpus-v1.json":
        fail("manifest-pinned corpus path mismatch")
    if manifest["synthetic_corpus"].get("probe_count") != corpus["expected_probe_count"]:
        fail("manifest corpus count mismatch")

    seen: set[str] = set()
    for probe in corpus["probes"]:
        exact_keys(probe, {"id", "category", "scenario", "input", "expected", "guards"}, "probe")
        probe_id = probe["id"]
        if probe_id in seen:
            fail(f"duplicate probe id: {probe_id}")
        seen.add(probe_id)
        expected_spec = EXPECTED_PROBES.get(probe_id)
        if expected_spec is None:
            fail(f"unreviewed probe added without validator update: {probe_id}")
        category, disposition, source_status = expected_spec
        if probe["category"] != category:
            fail(f"{probe_id}: category drift")
        exact_keys(probe["expected"], EXPECTED_BASE_KEYS, f"{probe_id} expected")
        exp = probe["expected"]
        if exp["disposition"] != disposition or exp["source_status"] != source_status:
            fail(f"{probe_id}: expected disposition/source-status drift")
        if exp["automated_external_write"] is not False or exp["automated_closure"] is not False:
            fail(f"{probe_id}: unsafe external-write/closure default")
        if exp["production_sla_calculation"] != "Disabled":
            fail(f"{probe_id}: production SLA calculation must remain disabled")
        if exp["criminality_conclusion"] != "None":
            fail(f"{probe_id}: model may not infer criminality")
        if exp["real_resident_data_present"] is not False or exp["precise_sensitive_asset_coordinates_present"] is not False:
            fail(f"{probe_id}: corpus must remain synthetic and non-sensitive")
        if type(exp["required_human_review"]) is not bool:
            fail(f"{probe_id}: required_human_review must be a Boolean")
        if not isinstance(probe["guards"], list) or not probe["guards"]:
            fail(f"{probe_id}: each probe needs explicit safety guards")
        required = REQUIRED_GUARDS.get(probe_id, set())
        if not required.issubset(set(probe["guards"])):
            fail(f"{probe_id}: required safety guard missing")

    if seen != set(EXPECTED_PROBES):
        fail(f"probe set mismatch: missing={sorted(set(EXPECTED_PROBES)-seen)}, extra={sorted(seen-set(EXPECTED_PROBES))}")

    # Targeted semantic assertions prevent a structurally valid fixture from
    # quietly reversing its key safety conclusions.
    by_id = {p["id"]: p for p in corpus["probes"]}
    if by_id["response-without-restoration"]["expected"]["issue_status_effect"] != "ResponseRecordedRestorationUnknown":
        fail("response cannot establish restoration")
    if by_id["repair-claim-without-independent-evidence"]["expected"]["issue_status_effect"] != "RepairClaimedNotVerified":
        fail("repair claim cannot establish verified completion")
    if by_id["unauthorized-sensor-feed"]["expected"]["disposition"] != "BlockIngestion":
        fail("unauthorized sensor feed must be blocked")
    if by_id["unfunded-work-pathway"]["expected"]["disposition"] != "DoNotAnnounceFundedPosition":
        fail("unfunded pathway must not be represented as a funded job")
    if by_id["demographic-risk-proxy"]["expected"]["disposition"] != "RejectAdverseAction":
        fail("demographic/poverty proxy cannot produce adverse action")
    coop_probe = by_id["sdbip-cooperative-kpi-is-not-contract-award"]
    if coop_probe["expected"]["disposition"] != "PartnerDiscoveryOnlyNoContractAward":
        fail("SDBIP planning KPI must not be treated as a procurement award")
    if coop_probe["expected"]["issue_status_effect"] != "NoFundedPositionsOrContractClaim":
        fail("cooperative contract candidate cannot imply funded positions")
    epwp_probe = by_id["national-epwp-target-vs-city-sdbip-target"]
    if epwp_probe["expected"]["disposition"] != "PreserveSeparateTargetSeries":
        fail("national and City EPWP planning targets must remain distinct")
    if epwp_probe["expected"]["issue_status_effect"] != "NoTargetSubstitution":
        fail("target series cannot overwrite or substitute each other")
    water_metric_probe = by_id["same-title-water-kpis-need-crosswalk"]
    if water_metric_probe["expected"]["disposition"] != "PreserveSeparateMetricSeries":
        fail("same-title water-loss metrics must not auto-crosswalk")
    if water_metric_probe["expected"]["issue_status_effect"] != "NoDerivedValueOrSla":
        fail("unqualified water metric crosswalk cannot create a derived value or SLA")
    municipal_finance_probe = by_id["municipal-repairs-view-without-amount-type-is-not-zero"]
    if municipal_finance_probe["expected"]["disposition"] != "HoldFinancialObservationUnqualified" or municipal_finance_probe["expected"]["issue_status_effect"] != "NoZeroSpendOrFundingClaim":
        fail("municipal finance view with unselected amount type cannot establish zero spend or funding")
    aggregate_funding_probe = by_id["aggregate-repairs-spend-is-not-role-funding"]
    if aggregate_funding_probe["expected"]["disposition"] != "AggregateContextOnly" or aggregate_funding_probe["expected"]["issue_status_effect"] != "NoTenderOrRoleFundingClaim":
        fail("aggregate repairs/maintenance expenditure cannot establish tender or role funding")
    snapshot_alignment_probe = by_id["latest-api-quarter-is-not-report-period"]
    if snapshot_alignment_probe["expected"]["disposition"] != "HoldCrossSnapshotComparison" or snapshot_alignment_probe["expected"]["issue_status_effect"] != "NoUnqualifiedTrendOrFundingClaim":
        fail("unaligned Treasury report/API snapshots cannot support an unqualified trend or funding claim")
    unit_probe = by_id["r-thousands-values-are-not-rands"]
    if unit_probe["expected"]["disposition"] != "RequireUnitNormalization" or unit_probe["expected"]["issue_status_effect"] != "NoSilentUnitConversion":
        fail("R-thousand report values must not be silently interpreted as individual Rands")
    reconciliation_probe = by_id["monthly-quarter-total-rounding-needs-review"]
    if reconciliation_probe["expected"]["disposition"] != "PreservePublishedTotalAndFlagReconciliation" or reconciliation_probe["expected"]["issue_status_effect"] != "NoSilentRecomputedQuarterTotal":
        fail("published quarter total/month arithmetic difference must be preserved for review")
    checksum_receipt_probe = by_id["published-snapshot-checksum-is-not-payload-verification"]
    if checksum_receipt_probe["expected"]["disposition"] != "RecordMetadataReceiptOnly" or checksum_receipt_probe["expected"]["issue_status_effect"] != "PayloadAndCurrentValuesUnverified":
        fail("published snapshot checksum must not be promoted to payload verification or financial-value evidence")
    cross_utility_probe = by_id["joint-water-power-statement-is-not-incident-causality"]
    if cross_utility_probe["expected"]["disposition"] != "HypothesisOnlyNoIncidentCausalEdge" or cross_utility_probe["expected"]["issue_status_effect"] != "NoLiveStatusOrCausalClaim":
        fail("joint government statement cannot establish incident-level causality or current outage")
    timezone_tender_probe = by_id["pikitup-deadline-without-timezone-is-not-open-verified"]
    if timezone_tender_probe["expected"]["disposition"] != "KeepAtTenderNoticeObserved" or timezone_tender_probe["expected"]["issue_status_effect"] != "NoTenderOpenVerifiedClaim":
        fail("specific Pikitup notice must stay below verified-open stage until timezone/currentness gates pass")
    cidb_probe = by_id["cidb-tender-requirement-is-not-cooperative-job-eligibility"]
    if cidb_probe["expected"]["disposition"] != "RequireBidderAndScopeReview" or cidb_probe["expected"]["issue_status_effect"] != "NoCooperativeEligibilityOrPaidRoleClaim":
        fail("tender eligibility cannot be assumed for a cooperative or paid role")
    scam_probe = by_id["tender-document-fee-or-bribe-solicitation-is-not-official"]
    if scam_probe["expected"]["disposition"] != "BlockPaymentAndUseOfficialChannel" or scam_probe["expected"]["criminality_conclusion"] != "None":
        fail("procurement scam safeguards must block payment without automatic accusation")
    registry_absence_probe = by_id["national-registry-absence-is-not-city-tender-absence"]
    if registry_absence_probe["expected"]["disposition"] != "ContinueIssuerSpecificDiscovery" or registry_absence_probe["expected"]["issue_status_effect"] != "NoNoTenderOrNoOpportunityClaim":
        fail("missing National Treasury registry record cannot establish absence of a local opportunity")
    empty_status_probe = by_id["ocds-empty-status-is-not-open"]
    if empty_status_probe["expected"]["disposition"] != "HoldStatusUnqualified" or empty_status_probe["expected"]["issue_status_effect"] != "NoOpenTenderClaim":
        fail("empty OCDS tender status must not be interpreted as open")
    award_probe = by_id["award-is-not-contract-or-funded-role"]
    if award_probe["expected"]["disposition"] != "RecordAwardOnly" or award_probe["expected"]["issue_status_effect"] != "NoFundedPositionOrContractClaim":
        fail("award observation must not imply signed contract or funded role")
    duplicate_release_probe = by_id["duplicate-ocds-release-is-not-second-opportunity"]
    if duplicate_release_probe["expected"]["disposition"] != "DeduplicateExactReleaseForCounting" or duplicate_release_probe["expected"]["issue_status_effect"] != "NoArtificialOpportunityOrVolumeIncrease":
        fail("duplicate procurement release must not inflate opportunity counts")
    current_tender_probe = by_id["current-tender-header-without-dated-notice"]
    if current_tender_probe["expected"]["disposition"] != "RequireNoticeSpecificCurrentness" or current_tender_probe["expected"]["issue_status_effect"] != "NoOpenTenderOrWorkPackageClaim":
        fail("current-tenders page heading cannot establish an open eligible opportunity")
    stale_rfq_block_probe = by_id["current-rfq-block-with-past-deadline-is-not-open"]
    if stale_rfq_block_probe["expected"]["disposition"] != "RequireNoticeSpecificCurrentness" or stale_rfq_block_probe["expected"]["issue_status_effect"] != "NoOpenTenderOrExtensionClaim":
        fail("past-dated RFQs in a Current RFQs block cannot establish live open status or extension")
    wage_probe = by_id["unknown-wage-regime-cannot-default-to-epwp"]
    if wage_probe["expected"]["disposition"] != "BlockPositionUntilWageReview" or wage_probe["expected"]["issue_status_effect"] != "NoWageRateOrPositionClaim":
        fail("unknown legal classification must block a wage or position claim")
    tender_probe = by_id["mixed-stage-tender-index-is-not-open-work"]
    if tender_probe["expected"]["disposition"] != "PartnerDiscoveryOnlyNoOpenNotice" or tender_probe["expected"]["issue_status_effect"] != "NoAvailableWorkPackageClaim":
        fail("mixed-stage tender landing page must not establish a live work package")
    rollout_probe = by_id["planned-controller-rollout-date-is-not-completion"]
    if rollout_probe["expected"]["disposition"] != "KeepRolloutStatusUnverified":
        fail("a passed planned completion date cannot establish rollout completion")
    if rollout_probe["expected"]["issue_status_effect"] != "NoCompletionClaim":
        fail("unknown rollout completion must not be promoted to a completion claim")

    # Ensure every public-facing data/authority limiter is still explicit.
    if "Automated municipal case submission" not in manifest["prohibited_capabilities"]:
        fail("prohibited capability list lost external-write boundary")
    if not any("payer" in str(x).lower() for x in manifest["stop_conditions"]):
        fail("field work must stop without an identified payer")
    if "Unauthorized data access or external write" not in manifest["stop_conditions"]:
        fail("field work must stop on unauthorized data access or external writes")

    print(
        "JOBURG-WATER-WORKFORCE-PILOT PASS: "
        f"{len(source_ids)} sources, {len(manifest['gates'])} phase gates, "
        f"{len(seen)} synthetic probes; blob={EXPECTED_CORPUS_BLOB_SHA}."
    )
    print("Claim ceiling: synthetic specification only; no runtime, provider, funding, or field qualification.")


if __name__ == "__main__":
    try:
        validate()
    except Exception as exc:
        print(f"JOBURG-WATER-WORKFORCE-PILOT FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
