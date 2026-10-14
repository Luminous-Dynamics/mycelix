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
EXPECTED_CORPUS_BLOB_SHA = "1e75f7702e6e3c2e1a44956a9571ad7461c664c7"
EXPECTED_SOURCE_IDS = ["JW-1", "COJ-1", "JW-2", "SA-1", "JOBS-1", "LAW-1", "SA-JOBS-2"]
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
}
REQUIRED_GUARDS = {
    "issuer-reference-collision": {"DoNotMergeByReferenceStringAlone", "ExternalReferenceIsNotInternalIdentity"},
    "stale-systemwide-bulletin": {"BulletinDoesNotResolveIndividualCase", "RetrievalDoesNotProveCurrentness"},
    "source-fetch-unavailable": {"FetchFailureIsNotSourceAbsence", "NoInferredCurrentStatus"},
    "response-without-restoration": {"ResponseIsNotRestoration", "NoFalseResolutionClaim"},
    "working-days-calendar-conversion": {"WorkingDaysAreNotCalendarDays", "MissingClockSemanticsFailsClosed"},
    "repair-claim-without-independent-evidence": {"RepairClaimIsNotVerifiedOutcome", "TicketClosureIsNotAuthority"},
    "unauthorized-sensor-feed": {"NoUnauthorizedProviderAccess", "DataAvailabilityDoesNotImplyPermission"},
    "duplicate-invoice-with-missing-delivery-evidence": {"AnomalyIsNotProofOfFraud", "NoAutomaticBlacklisting"},
    "unfunded-work-pathway": {"NoSpeculativePayroll", "OpportunityIsNotAJob"},
    "sensitive-location-publication": {"NoPublicSensitiveAssetCoordinates", "NoExploitableOperationalDetail"},
    "demographic-risk-proxy": {"PovertyIsNotEvidenceOfWrongdoing", "ModelFlagIsNotAdjudication"},
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
        "prohibited_capabilities", "synthetic_corpus",
    }, "manifest")
    if manifest["schema"] != EXPECTED_SCHEMA or manifest["schema_version"] != 1:
        fail("manifest schema/version mismatch")
    if manifest["claim_ceiling"] != "ResearchSpecificationOnly":
        fail("manifest must preserve research-only claim ceiling")
    if manifest["markdown_spec"] != "mycelix-workspace/docs/civic-resilience/JOHANNESBURG_WATER_WORKFORCE_PILOT_V1.md":
        fail("manifest narrative path drift")

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

    # Ensure every public-facing data/authority limiter is still explicit.
    if "Automated municipal case submission" not in manifest["prohibited_capabilities"]:
        fail("prohibited capability list lost external-write boundary")
    if not any("payer" in str(x).lower() for x in manifest["stop_conditions"]):
        fail("field work must stop without an identified payer")
    if "No unauthorized provider access" not in manifest["stop_conditions"]:
        fail("field work must stop on unauthorized provider access")

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
