//! Deterministic, fail-closed source-observability assessment.
//!
//! The only negative-like result is scoped to one source and one time window.
//! It never means that a physical object is absent.

use std::collections::BTreeSet;

use jsonschema::Validator;
use serde::Serialize;
use serde_json::Value;
use sha2::{Digest, Sha256};

use crate::{parse_strict_json, schema_has_external_refs};

pub const OBSERVABILITY_DGP_JSON: &str =
    include_str!("../../../docs/research/civil-awareness-observability-dgp-v0.1.json");
pub const OBSERVABILITY_SCHEMA_JSON: &str =
    include_str!("../../../docs/research/civil-awareness-observability-dgp-v0.1.schema.json");
pub const OBSERVABILITY_REPORT_CONTRACT: &str = "gcpn-observability-report/0.1";
const CLAIM_CEILING: &str = "Source-scoped observation-state assessment over the exact supplied scope, coverage assertion, source-health record, rights state and dependency frontier. NOT_OBSERVED_WITHIN_DECLARED_COVERAGE is not physical absence, a complete census, identity proof, intent or safety clearance.";
const MAX_DGP_BYTES: usize = 2 * 1024 * 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ObservabilityState {
    ObservedPresent,
    NotObservedWithinDeclaredCoverage,
    Unobservable,
    CoverageUnknown,
    FeedUnavailable,
    NotEquippedOrNotRequired,
    Stale,
    WithheldOrRestricted,
    Conflict,
    NotApplicable,
}

impl ObservabilityState {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::ObservedPresent => "OBSERVED_PRESENT",
            Self::NotObservedWithinDeclaredCoverage => "NOT_OBSERVED_WITHIN_DECLARED_COVERAGE",
            Self::Unobservable => "UNOBSERVABLE",
            Self::CoverageUnknown => "COVERAGE_UNKNOWN",
            Self::FeedUnavailable => "FEED_UNAVAILABLE",
            Self::NotEquippedOrNotRequired => "NOT_EQUIPPED_OR_NOT_REQUIRED",
            Self::Stale => "STALE",
            Self::WithheldOrRestricted => "WITHHELD_OR_RESTRICTED",
            Self::Conflict => "CONFLICT",
            Self::NotApplicable => "NOT_APPLICABLE",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct SourceClaim {
    pub source_id: String,
    pub claim: String,
    pub assertion_ref: String,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ObservabilityAssessment {
    pub source_id: String,
    pub scope_id: String,
    pub window_start: f64,
    pub window_end: f64,
    pub state: ObservabilityState,
    pub reason_codes: Vec<String>,
    pub source_claims: Vec<SourceClaim>,
    pub claim_scope: String,
    pub claim_ceiling: &'static str,
}

impl ObservabilityAssessment {
    fn new(
        case: &Value,
        state: ObservabilityState,
        reasons: impl IntoIterator<Item = &'static str>,
        source_claims: Vec<SourceClaim>,
    ) -> Result<Self, String> {
        let source_id = string_at(case, "/source/id")?;
        let scope_id = string_at(case, "/scope/scope_id")?;
        let window_start = number_at(case, "/scope/window_start")?;
        let window_end = number_at(case, "/scope/window_end")?;
        let reason_codes = reasons
            .into_iter()
            .map(str::to_owned)
            .collect::<BTreeSet<_>>();
        let claim_scope = format!(
            "source={source_id};scope={scope_id};window=[{window_start},{window_end}]"
        );
        Ok(Self {
            source_id,
            scope_id,
            window_start,
            window_end,
            state,
            reason_codes: reason_codes.into_iter().collect(),
            source_claims,
            claim_scope,
            claim_ceiling: CLAIM_CEILING,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ObservabilityEvaluationReport {
    pub report_contract: &'static str,
    pub dgp_sha256: String,
    pub schema_sha256: String,
    pub case_count: usize,
    pub assessments: Vec<ObservabilityAssessment>,
    pub claim_ceiling: &'static str,
}

#[derive(Debug)]
pub struct ObservabilityEvaluator {
    schema_validator: Validator,
    schema_sha256: String,
}

impl ObservabilityEvaluator {
    /// Compile the pinned challenge schema; no references are resolved online.
    pub fn from_embedded() -> Result<Self, String> {
        let schema = parse_strict_json(OBSERVABILITY_SCHEMA_JSON.as_bytes())
            .map_err(|_| "observability.schema_json_invalid".to_string())?;
        if schema_has_external_refs(&schema) {
            return Err("observability.schema_external_reference_forbidden".into());
        }
        if !jsonschema::meta::is_valid(&schema) {
            return Err("observability.schema_meta_invalid".into());
        }
        let schema_validator = jsonschema::draft202012::options()
            .should_validate_formats(true)
            .build(&schema)
            .map_err(|_| "observability.schema_compile_failed".to_string())?;
        Ok(Self {
            schema_validator,
            schema_sha256: format!("sha256:{}", sha256_hex(OBSERVABILITY_SCHEMA_JSON.as_bytes())),
        })
    }

    /// Evaluate every case in the embedded frozen DGP.
    pub fn evaluate_embedded(&self) -> Result<ObservabilityEvaluationReport, String> {
        self.evaluate_dgp_bytes(OBSERVABILITY_DGP_JSON.as_bytes())
    }

    /// Parse and validate exact DGP bytes, then assess every case. The function
    /// deliberately does not read any expected-oracle field.
    pub fn evaluate_dgp_bytes(
        &self,
        bytes: &[u8],
    ) -> Result<ObservabilityEvaluationReport, String> {
        if bytes.len() > MAX_DGP_BYTES {
            return Err("observability.dgp_too_large".into());
        }
        let dgp = parse_strict_json(bytes)
            .map_err(|_| "observability.dgp_parse_or_duplicate_key_failure".to_string())?;
        if self.schema_validator.iter_errors(&dgp).next().is_some() {
            return Err("observability.dgp_schema_validation_failed".into());
        }
        let cases = dgp
            .get("cases")
            .and_then(Value::as_array)
            .ok_or_else(|| "observability.cases_missing".to_string())?;
        let mut assessments = Vec::with_capacity(cases.len());
        for case in cases {
            assessments.push(assess_case(case)?);
        }
        Ok(ObservabilityEvaluationReport {
            report_contract: OBSERVABILITY_REPORT_CONTRACT,
            dgp_sha256: format!("sha256:{}", sha256_hex(bytes)),
            schema_sha256: self.schema_sha256.clone(),
            case_count: assessments.len(),
            assessments,
            claim_ceiling: CLAIM_CEILING,
        })
    }

    /// Assess one case using the same strict duplicate-key parser used by the
    /// evidence-envelope validator.
    pub fn assess_case_bytes(&self, bytes: &[u8]) -> Result<ObservabilityAssessment, String> {
        if bytes.len() > MAX_DGP_BYTES {
            return Err("observability.case_too_large".into());
        }
        let case = parse_strict_json(bytes)
            .map_err(|_| "observability.case_parse_or_duplicate_key_failure".to_string())?;
        assess_case(&case)
    }
}

fn assess_case(case: &Value) -> Result<ObservabilityAssessment, String> {
    let rights = string_at(case, "/rights")?;
    if rights != "allowed_for_synthetic_test" {
        return assessment(
            case,
            ObservabilityState::WithheldOrRestricted,
            ["rights.source_unavailable_for_declared_purpose"],
            Vec::new(),
        );
    }
    let completeness = string_at(case, "/data_completeness")?;
    // Do not parse or expose observation content until rights and availability
    // preconditions have passed.
    let claims = source_claims(case)?;
    let applicability = string_at(case, "/equipment_applicability")?;
    let health = string_at(case, "/source_health")?;
    let coverage_status = string_at(case, "/coverage/status")?;
    let freshness = string_at(case, "/freshness")?;
    let modality = string_at(case, "/source/modality")?;

    // Rights/purpose and equipment applicability precede source interpretation.
    if completeness == "withheld" {
        return assessment(
            case,
            ObservabilityState::WithheldOrRestricted,
            ["rights.source_data_withheld"],
            Vec::new(),
        );
    }
    if modality == "meteorological_context" || coverage_status == "not_applicable" {
        return assessment(
            case,
            ObservabilityState::NotApplicable,
            ["source.modality_not_applicable_to_presence"],
            claims,
        );
    }

    // A known gap or outage prevents a full-window non-observation.
    if matches!(health, "failed" | "partial_outage")
        || (coverage_status == "partial"
            && has_nonempty_array(case, "/coverage/uncovered_intervals"))
    {
        return assessment(
            case,
            ObservabilityState::FeedUnavailable,
            ["source.feed_or_observation_window_incomplete"],
            claims,
        );
    }

    // Known blind regions/profile mismatch are unobservable rather than unknown.
    if matches!(
        coverage_status,
        "qualified_with_declared_blind_region" | "qualified_for_other_classes_only"
    ) {
        return assessment(
            case,
            ObservabilityState::Unobservable,
            ["coverage.known_blind_region_or_profile_mismatch"],
            claims,
        );
    }

    // A single fresh positive source observation is still reportable when
    // the broader coverage envelope is partial or unknown. It proves only that
    // the source emitted an observation; it does not prove full-window coverage
    // or physical identity. Multi-source aggregates remain subject to common
    // coverage/lineage qualification.
    let distinct_sources: BTreeSet<&str> =
        claims.iter().map(|claim| claim.source_id.as_str()).collect();
    let has_positive = claims.iter().any(|claim| {
        matches!(
            claim.claim.as_str(),
            "OBSERVED_PRESENT" | "OBSERVED_PRESENT_WITHIN_SCOPE"
        )
    });
    let has_negative = claims.iter().any(|claim| {
        matches!(
            claim.claim.as_str(),
            "NO_MESSAGE" | "NOT_OBSERVED_WITHIN_DECLARED_COVERAGE"
        )
    });
    if distinct_sources.len() == 1
        && has_positive
        && !has_negative
        && health == "healthy"
        && freshness == "fresh"
        && completeness == "complete"
    {
        return assessment(
            case,
            ObservabilityState::ObservedPresent,
            ["source.positive_observation_recorded_coverage_may_be_partial"],
            claims,
        );
    }
    if matches!(
        applicability,
        "not_required_for_declared_vessel_class" | "not_equipped_or_not_required"
    ) && claims.is_empty()
    {
        return assessment(
            case,
            ObservabilityState::NotEquippedOrNotRequired,
            ["source.equipment_or_carriage_not_applicable"],
            claims,
        );
    }

    // A negative cannot be assessed without a complete dependency inventory
    // and evidence frontier binding the query's source set.
    if !dependency_context_is_complete(case) {
        return assessment(
            case,
            ObservabilityState::CoverageUnknown,
            ["dependence.snapshot_or_evidence_frontier_unavailable"],
            claims,
        );
    }

    if matches!(coverage_status, "unknown" | "stale" | "partial_unknown" | "partial")
        || value_at(case, "/coverage/complete_for_scope") != Some(&Value::Bool(true))
    {
        return assessment(
            case,
            ObservabilityState::CoverageUnknown,
            ["coverage.assertion_missing_stale_or_incomplete"],
            claims,
        );
    }

    if !coverage_assertion_is_pinned_and_valid(case) {
        return assessment(
            case,
            ObservabilityState::CoverageUnknown,
            ["coverage.assertion_not_bound_to_full_scope_window"],
            claims,
        );
    }

    if freshness == "stale" {
        return assessment(
            case,
            ObservabilityState::Stale,
            ["source.observation_or_coverage_stale"],
            claims,
        );
    }
    if freshness != "fresh" || completeness != "complete" {
        return assessment(
            case,
            ObservabilityState::CoverageUnknown,
            ["source.freshness_or_completeness_unknown"],
            claims,
        );
    }

    // Conflicting qualified source outcomes remain visible; no vote or
    // majority rule can turn a source-specific non-observation into absence.
    if qualified_cross_source_conflict(case, &claims) {
        return assessment(
            case,
            ObservabilityState::Conflict,
            ["evidence.qualified_source_assertions_conflict"],
            claims,
        );
    }

    if health == "unknown" || (health != "healthy" && health != "mixed") {
        return assessment(
            case,
            ObservabilityState::CoverageUnknown,
            ["source.health_not_qualified_for_observation"],
            claims,
        );
    }
    if health == "mixed" {
        return assessment(
            case,
            ObservabilityState::CoverageUnknown,
            ["source.health_mixed_without_adjudicated_conflict"],
            claims,
        );
    }

    if claims.iter().any(|claim| {
        matches!(
            claim.claim.as_str(),
            "OBSERVED_PRESENT" | "OBSERVED_PRESENT_WITHIN_SCOPE"
        )
    }) {
        return assessment(
            case,
            ObservabilityState::ObservedPresent,
            ["source.positive_observation_recorded"],
            claims,
        );
    }

    if claims
        .iter()
        .any(|claim| claim.claim == "OLD_REPORT_OUTSIDE_FRESHNESS_WINDOW")
    {
        return assessment(
            case,
            ObservabilityState::Stale,
            ["source.observation_outside_freshness_cutoff"],
            claims,
        );
    }

    if claims.iter().any(|claim| {
        !matches!(
            claim.claim.as_str(),
            "NO_MESSAGE" | "NOT_OBSERVED_WITHIN_DECLARED_COVERAGE"
        )
    }) {
        return assessment(
            case,
            ObservabilityState::CoverageUnknown,
            ["evidence.observation_claim_unrecognized_or_unresolved"],
            claims,
        );
    }

    if claims.is_empty()
        || claims.iter().all(|claim| {
            matches!(
                claim.claim.as_str(),
                "NO_MESSAGE" | "NOT_OBSERVED_WITHIN_DECLARED_COVERAGE"
            )
        })
    {
        if applicability_is_supported(applicability) {
            return assessment(
                case,
                ObservabilityState::NotObservedWithinDeclaredCoverage,
                ["coverage.full_window_qualified_no_source_observation"],
                claims,
            );
        }
    }

    assessment(
        case,
        ObservabilityState::CoverageUnknown,
        ["evidence.insufficient_for_observation_state"],
        claims,
    )
}

fn applicability_is_supported(value: &str) -> bool {
    matches!(value, "applicable" | "equipped_or_obligation_confirmed")
}

fn dependency_context_is_complete(case: &Value) -> bool {
    let Some(context) = case.get("dependency_context") else {
        return false;
    };
    string_at(case, "/dependency_context/status").ok() == Some("complete")
        && string_at(case, "/dependency_context/snapshot_ref")
            .is_ok_and(|value| !value.trim().is_empty())
        && string_at(case, "/dependency_context/evidence_frontier_ref")
            .is_ok_and(|value| !value.trim().is_empty())
        && context.get("inventory_complete") == Some(&Value::Bool(true))
}

fn coverage_assertion_is_pinned_and_valid(case: &Value) -> bool {
    let Ok(assertion_id) = string_at(case, "/coverage/assertion_id") else {
        return false;
    };
    let Ok(oracle) = string_at(case, "/coverage/independent_oracle") else {
        return false;
    };
    let (Ok(start), Ok(end), Ok(coverage_start), Ok(coverage_end)) = (
        number_at(case, "/scope/window_start"),
        number_at(case, "/scope/window_end"),
        number_at(case, "/coverage/valid_from"),
        number_at(case, "/coverage/valid_until"),
    ) else {
        return false;
    };
    let digest = string_at(case, "/coverage/coverage_profile_digest").unwrap_or_default();
    !assertion_id.trim().is_empty()
        && !oracle.trim().is_empty()
        && end > start
        && coverage_end > coverage_start
        && coverage_start <= start
        && coverage_end >= end
        && is_sha256_digest(digest)
}

fn qualified_cross_source_conflict(case: &Value, claims: &[SourceClaim]) -> bool {
    if string_at(case, "/coverage/status").ok() != Some("qualified_for_both_sources")
        || value_at(case, "/coverage/complete_for_scope") != Some(&Value::Bool(true))
        || string_at(case, "/source_health").ok() != Some("mixed")
    {
        return false;
    }
    let has_positive = claims.iter().any(|claim| {
        matches!(
            claim.claim.as_str(),
            "OBSERVED_PRESENT" | "OBSERVED_PRESENT_WITHIN_SCOPE"
        )
    });
    let has_negative = claims.iter().any(|claim| {
        matches!(
            claim.claim.as_str(),
            "NO_MESSAGE" | "NOT_OBSERVED_WITHIN_DECLARED_COVERAGE"
        )
    });
    let distinct_sources: BTreeSet<&str> = claims
        .iter()
        .map(|claim| claim.source_id.as_str())
        .collect();
    let coverage_refs = case
        .pointer("/coverage/source_coverage_refs")
        .and_then(Value::as_array);
    has_positive
        && has_negative
        && distinct_sources.len() >= 2
        && coverage_refs.is_some_and(|refs| {
            refs.len() >= 2
                && refs
                    .iter()
                    .all(|item| item.as_str().is_some_and(|value| !value.trim().is_empty()))
        })
        && coverage_assertion_is_pinned_and_valid(case)
}

fn source_claims(case: &Value) -> Result<Vec<SourceClaim>, String> {
    let observations = case
        .get("observations")
        .and_then(Value::as_array)
        .ok_or_else(|| "observability.observations_missing".to_string())?;
    let mut claims = Vec::with_capacity(observations.len());
    for item in observations {
        let source_id = item
            .get("source_id")
            .and_then(Value::as_str)
            .filter(|value| !value.trim().is_empty())
            .ok_or_else(|| "observability.observation_source_id_missing".to_string())?;
        let claim = item
            .get("claim")
            .and_then(Value::as_str)
            .filter(|value| {
                matches!(
                    *value,
                    "OBSERVED_PRESENT"
                        | "OBSERVED_PRESENT_WITHIN_SCOPE"
                        | "NOT_OBSERVED_WITHIN_DECLARED_COVERAGE"
                        | "NO_MESSAGE"
                        | "OLD_REPORT_OUTSIDE_FRESHNESS_WINDOW"
                )
            })
            .ok_or_else(|| "observability.observation_claim_invalid".to_string())?;
        let assertion_ref = item
            .get("assertion_ref")
            .and_then(Value::as_str)
            .filter(|value| !value.trim().is_empty())
            .ok_or_else(|| "observability.observation_assertion_ref_missing".to_string())?;
        claims.push(SourceClaim {
            source_id: source_id.to_owned(),
            claim: claim.to_owned(),
            assertion_ref: assertion_ref.to_owned(),
        });
    }
    claims.sort_by(|left, right| {
        (&left.source_id, &left.claim, &left.assertion_ref).cmp(&(
            &right.source_id,
            &right.claim,
            &right.assertion_ref,
        ))
    });
    Ok(claims)
}

fn assessment(
    case: &Value,
    state: ObservabilityState,
    reasons: impl IntoIterator<Item = &'static str>,
    claims: Vec<SourceClaim>,
) -> Result<ObservabilityAssessment, String> {
    ObservabilityAssessment::new(case, state, reasons, claims)
}

fn string_at<'a>(value: &'a Value, pointer: &str) -> Result<&'a str, String> {
    value
        .pointer(pointer)
        .and_then(Value::as_str)
        .ok_or_else(|| format!("observability.field_missing_or_invalid:{pointer}"))
}

fn number_at(value: &Value, pointer: &str) -> Result<f64, String> {
    value
        .pointer(pointer)
        .and_then(Value::as_f64)
        .filter(|number| number.is_finite())
        .ok_or_else(|| format!("observability.number_missing_or_invalid:{pointer}"))
}

fn value_at<'a>(value: &'a Value, pointer: &str) -> Option<&'a Value> {
    value.pointer(pointer)
}

fn has_nonempty_array(value: &Value, pointer: &str) -> bool {
    value
        .pointer(pointer)
        .and_then(Value::as_array)
        .is_some_and(|items| !items.is_empty())
}

fn is_sha256_digest(value: &str) -> bool {
    value.strip_prefix("sha256:").is_some_and(|hex| {
        hex.len() == 64
            && hex
                .bytes()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    })
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write as _;
        let _ = write!(&mut output, "{byte:02x}");
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    fn evaluator() -> ObservabilityEvaluator {
        ObservabilityEvaluator::from_embedded().expect("pinned observability schema compiles")
    }

    fn fixture() -> Value {
        parse_strict_json(OBSERVABILITY_DGP_JSON.as_bytes()).expect("frozen DGP is strict JSON")
    }

    fn case_by_id<'a>(dgp: &'a Value, id: &str) -> &'a Value {
        dgp["cases"]
            .as_array()
            .unwrap()
            .iter()
            .find(|case| case["id"].as_str() == Some(id))
            .expect("case exists in frozen DGP")
    }

    #[test]
    fn embedded_dgp_schema_and_evaluator_pass_all_frozen_cases() {
        let report = evaluator().evaluate_embedded().expect("DGP is valid");
        let dgp = fixture();
        let cases = dgp["cases"].as_array().unwrap();
        assert_eq!(report.case_count, cases.len());
        for (case, assessment) in cases.iter().zip(report.assessments.iter()) {
            assert_eq!(
                assessment.state.as_str(),
                case["expected"]["state"].as_str().unwrap(),
                "{}",
                case["id"].as_str().unwrap()
            );
        }
    }

    #[test]
    fn evaluator_does_not_depend_on_expected_oracle_field() {
        let dgp = fixture();
        let evaluator = evaluator();
        let bytes = serde_json::to_vec(&dgp).unwrap();
        let first = evaluator.evaluate_dgp_bytes(&bytes).unwrap();
        let mut changed_oracle = dgp;
        for case in changed_oracle["cases"].as_array_mut().unwrap() {
            case["expected"]["state"] = Value::String("CONFLICT".into());
        }
        let changed_bytes = serde_json::to_vec(&changed_oracle).unwrap();
        let second = evaluator.evaluate_dgp_bytes(&changed_bytes).unwrap();
        let first_states: Vec<_> = first.assessments.iter().map(|item| item.state).collect();
        let second_states: Vec<_> = second.assessments.iter().map(|item| item.state).collect();
        assert_eq!(first_states, second_states);
    }

    #[test]
    fn removing_coverage_assertion_downgrades_non_observation_to_unknown() {
        let dgp = fixture();
        let mut case = case_by_id(&dgp, "OBS-012").clone();
        case["coverage"]["assertion_id"] = Value::Null;
        case["coverage"]["status"] = Value::String("unknown".into());
        case["coverage"]["complete_for_scope"] = Value::Bool(false);
        let report = assess_case(&case).expect("mutated case has enough structure");
        assert_eq!(report.state, ObservabilityState::CoverageUnknown);
    }

    #[test]
    fn missing_dependency_frontier_blocks_coverage_qualified_non_observation() {
        let dgp = fixture();
        let mut case = case_by_id(&dgp, "OBS-011").clone();
        case["dependency_context"]["snapshot_ref"] = Value::Null;
        let report = assess_case(&case).unwrap();
        assert_eq!(report.state, ObservabilityState::CoverageUnknown);
    }

    #[test]
    fn stale_dependency_frontier_blocks_coverage_qualified_non_observation() {
        let dgp = fixture();
        let case = case_by_id(&dgp, "OBS-018");
        let report = assess_case(case).unwrap();
        assert_eq!(report.state, ObservabilityState::CoverageUnknown);
    }

    #[test]
    fn coverage_window_must_contain_the_full_observation_window() {
        let mut case = case_by_id(&fixture(), "OBS-011").clone();
        case["coverage"]["valid_until"] = serde_json::json!(150);
        let report = assess_case(&case).unwrap();
        assert_eq!(report.state, ObservabilityState::CoverageUnknown);
    }

    #[test]
    fn coverage_profile_digest_is_mandatory_for_non_observation() {
        let mut case = case_by_id(&fixture(), "OBS-011").clone();
        case["coverage"]["coverage_profile_digest"] = Value::String("missing".into());
        let report = assess_case(&case).unwrap();
        assert_eq!(report.state, ObservabilityState::CoverageUnknown);
    }

    #[test]
    fn rights_restriction_precedes_any_detection_inference() {
        let dgp = fixture();
        let mut case = case_by_id(&dgp, "OBS-013").clone();
        case["rights"] = Value::String("restricted_for_declared_purpose".into());
        let report = assess_case(&case).unwrap();
        assert_eq!(report.state, ObservabilityState::WithheldOrRestricted);
    }

    #[test]
    fn qualified_cross_source_disagreement_is_preserved_as_conflict() {
        let dgp = fixture();
        let case = case_by_id(&dgp, "OBS-016");
        let report = assess_case(case).unwrap();
        assert_eq!(report.state, ObservabilityState::Conflict);
        assert_eq!(report.source_claims.len(), 2);
        assert_ne!(report.source_claims[0].source_id, report.source_claims[1].source_id);
    }

    #[test]
    fn fresh_single_source_positive_is_preserved_when_coverage_is_incomplete() {
        let dgp = fixture();
        let case = case_by_id(&dgp, "OBS-019");
        let report = assess_case(case).unwrap();
        assert_eq!(report.state, ObservabilityState::ObservedPresent);
        assert_eq!(report.source_claims.len(), 1);
        assert!(report.reason_codes.contains(
            &"source.positive_observation_recorded_coverage_may_be_partial".to_owned()
        ));
    }

    #[test]
    fn positive_observation_is_not_erased_by_empty_other_sources() {
        let dgp = fixture();
        let case = case_by_id(&dgp, "OBS-013");
        let report = assess_case(case).unwrap();
        assert_eq!(report.state, ObservabilityState::ObservedPresent);
    }

    #[test]
    fn failed_and_partial_outage_sources_are_not_negative() {
        for id in ["OBS-002", "OBS-007"] {
            let dgp = fixture();
            let case = case_by_id(&dgp, id);
            let report = assess_case(case).unwrap();
            assert_eq!(report.state, ObservabilityState::FeedUnavailable, "{id}");
        }
    }

    #[test]
    fn duplicate_keys_in_dgp_are_rejected() {
        let evaluator = evaluator();
        let result = evaluator.evaluate_dgp_bytes(br#"{"schema":"a","schema":"b"}"#);
        assert!(result.is_err());
    }

    #[test]
    fn output_never_claims_physical_absence() {
        let report = evaluator().evaluate_embedded().unwrap();
        for item in &report.assessments {
            assert!(item.claim_ceiling.contains("not physical absence"));
        }
    }
}
