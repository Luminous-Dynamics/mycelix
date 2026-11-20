//! Deterministic semantic-assertion comparison for aerospace digital-thread adapters.
//!
//! This crate compares normalized assertion sets. It is not a STEP/AP242 or
//! QIF parser, nor a standards-conformance engine.

use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

pub const CASE_SCHEMA: &str = "AeroSemanticAssertionV1";

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SemanticRefV1 {
    pub id: String,
    pub kind: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SemanticType {
    PersistentIdentity,
    RequirementLink,
    FeatureAssociation,
    NominalDimension,
    Tolerance,
    DatumReference,
    Unit,
    CoordinateFrame,
    MaterialProperty,
    CompositeLayup,
    ProcessOperation,
    ExecutionInterval,
    MeasurementResult,
    CalibrationValidity,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SemanticAssertionV1 {
    pub assertion_id: String,
    pub semantic_type: SemanticType,
    pub subject: SemanticRefV1,
    pub requirement: Option<SemanticRefV1>,
    pub value: Value,
    pub unit: Option<String>,
    pub source_locator: String,
    pub required: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SourceClass {
    AuthoredSynthetic,
    ExternalReference,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CaseKind {
    PositiveControl,
    NegativeControl,
    RoundTrip,
    InteroperabilityReference,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum RightsStatus {
    NotReviewed,
    Restricted,
    ClearedForTesting,
    ClearedForRedistribution,
    Rejected,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SemanticSourceV1 {
    pub source_id: String,
    pub publisher: String,
    pub url: String,
    pub version: String,
    pub acquired_at: String,
    pub rights_status: RightsStatus,
    pub sha256: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum MutationOperation {
    DropField,
    ChangeValue,
    ChangeUnit,
    ChangeToleranceType,
    ChangeDatum,
    ChangeSubjectId,
    DuplicatePersistentId,
    StaleRevision,
    ShiftEventTime,
    DropTraceEdge,
    ReplaceSemanticWithGraphicOnly,
    OmitUnsupportedField,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MutationControlV1 {
    pub mutation_id: String,
    pub target_assertion_id: String,
    pub operation: MutationOperation,
    pub expected_disposition: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SemanticCaseV1 {
    pub schema: String,
    pub benchmark_id: String,
    pub case_id: String,
    pub source_class: SourceClass,
    pub case_kind: CaseKind,
    pub configuration: SemanticRefV1,
    #[serde(default)]
    pub source_manifest: Vec<SemanticSourceV1>,
    pub assertions: Vec<SemanticAssertionV1>,
    pub mutations: Vec<MutationControlV1>,
    pub expected_disposition: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum AssertionStatus {
    Preserved,
    SemanticLoss,
    WrongSubjectJoin,
    Unsupported,
    Ambiguous,
    NotRequired,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AssertionResult {
    pub assertion_id: String,
    pub status: AssertionStatus,
    pub explanation: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvaluationCounts {
    pub preserved: usize,
    pub semantic_loss: usize,
    pub wrong_subject_join: usize,
    pub unsupported: usize,
    pub ambiguous: usize,
    pub not_required: usize,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvaluationReport {
    pub benchmark_id: String,
    pub case_id: String,
    pub disposition: AssertionStatus,
    pub counts: EvaluationCounts,
    pub results: Vec<AssertionResult>,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum EvaluationError {
    #[error("invalid case schema")]
    WrongSchema,
    #[error("missing or malformed identity: {0}")]
    InvalidIdentity(String),
    #[error("external reference case has no source manifest")]
    MissingExternalSourceManifest,
    #[error("external source is not rights-cleared for testing: {0}")]
    SourceRightsNotAdmitted(String),
    #[error("source digest must be 64 lowercase hexadecimal characters: {0}")]
    InvalidSourceDigest(String),
    #[error("duplicate expected assertion id: {0}")]
    DuplicateExpectedAssertion(String),
    #[error("duplicate observed assertion id: {0}")]
    DuplicateObservedAssertion(String),
    #[error("mutation target assertion not found: {0}")]
    MutationTargetNotFound(String),
    #[error("invalid semantic case JSON: {0}")]
    InvalidCaseJson(String),
    #[error("unknown field in {scope}: {field}")]
    UnknownField { scope: String, field: String },
    #[error("missing required field in {scope}: {field}")]
    MissingField { scope: String, field: String },
    #[error("malformed JSON value at {0}")]
    MalformedJsonValue(String),
}

fn check_object_fields(value: &Value, allowed: &[&str], required: &[&str], scope: &str) -> Result<(), EvaluationError> {
    let object = value.as_object().ok_or_else(|| EvaluationError::MalformedJsonValue(scope.to_owned()))?;
    for field in object.keys() {
        if !allowed.contains(&field.as_str()) {
            return Err(EvaluationError::UnknownField { scope: scope.to_owned(), field: field.clone() });
        }
    }
    for field in required {
        if !object.contains_key(*field) {
            return Err(EvaluationError::MissingField { scope: scope.to_owned(), field: (*field).to_owned() });
        }
    }
    Ok(())
}

fn check_reference(value: &Value, scope: &str) -> Result<(), EvaluationError> {
    check_object_fields(value, &["id", "kind"], &["id", "kind"], scope)
}

fn check_assertion(value: &Value, scope: &str) -> Result<(), EvaluationError> {
    const FIELDS: &[&str] = &["assertion_id", "semantic_type", "subject", "requirement", "value", "unit", "source_locator", "required"];
    check_object_fields(value, FIELDS, FIELDS, scope)?;
    check_reference(value.get("subject").ok_or_else(|| EvaluationError::MissingField { scope: scope.to_owned(), field: "subject".to_owned() })?, &format!("{scope}.subject"))?;
    if let Some(requirement) = value.get("requirement") {
        if !requirement.is_null() {
            check_reference(requirement, &format!("{scope}.requirement"))?;
        }
    }
    Ok(())
}

/// Strict JSON parser for the normalized assertion-set contract.
pub fn parse_semantic_case(bytes: &[u8]) -> Result<SemanticCaseV1, EvaluationError> {
    let value: Value = serde_json::from_slice(bytes).map_err(|error| EvaluationError::InvalidCaseJson(error.to_string()))?;
    const CASE_FIELDS: &[&str] = &["schema", "benchmark_id", "case_id", "source_class", "case_kind", "configuration", "source_manifest", "assertions", "mutations", "expected_disposition"];
    check_object_fields(&value, CASE_FIELDS, CASE_FIELDS, "case")?;
    check_reference(value.get("configuration").ok_or_else(|| EvaluationError::MissingField { scope: "case".into(), field: "configuration".into() })?, "case.configuration")?;

    let assertions = value.get("assertions").and_then(Value::as_array).ok_or_else(|| EvaluationError::MalformedJsonValue("case.assertions".into()))?;
    for (index, assertion) in assertions.iter().enumerate() {
        check_assertion(assertion, &format!("case.assertions[{index}]"))?;
    }
    let sources = value.get("source_manifest").and_then(Value::as_array).ok_or_else(|| EvaluationError::MalformedJsonValue("case.source_manifest".into()))?;
    const SOURCE_FIELDS: &[&str] = &["source_id", "publisher", "url", "version", "acquired_at", "rights_status", "sha256"];
    for (index, source) in sources.iter().enumerate() {
        check_object_fields(source, SOURCE_FIELDS, SOURCE_FIELDS, &format!("case.source_manifest[{index}]"))?;
    }
    let mutations = value.get("mutations").and_then(Value::as_array).ok_or_else(|| EvaluationError::MalformedJsonValue("case.mutations".into()))?;
    const MUTATION_FIELDS: &[&str] = &["mutation_id", "target_assertion_id", "operation", "expected_disposition"];
    for (index, mutation) in mutations.iter().enumerate() {
        check_object_fields(mutation, MUTATION_FIELDS, MUTATION_FIELDS, &format!("case.mutations[{index}]"))?;
    }

    serde_json::from_value(value).map_err(|error| EvaluationError::InvalidCaseJson(error.to_string()))
}

fn is_valid_sha256(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
}

/// Rights gate for external source manifests. Synthetic cases need no external
/// source and may not be mislabeled as externally cleared evidence.
pub fn validate_case(case: &SemanticCaseV1) -> Result<(), EvaluationError> {
    if case.schema != CASE_SCHEMA {
        return Err(EvaluationError::WrongSchema);
    }
    for (value, label) in [
        (case.benchmark_id.as_str(), "benchmark_id"),
        (case.case_id.as_str(), "case_id"),
        (case.configuration.id.as_str(), "configuration.id"),
        (case.configuration.kind.as_str(), "configuration.kind"),
    ] {
        if value.trim().is_empty() {
            return Err(EvaluationError::InvalidIdentity(label.to_owned()));
        }
    }
    if case.source_class == SourceClass::ExternalReference && case.source_manifest.is_empty() {
        return Err(EvaluationError::MissingExternalSourceManifest);
    }
    for source in &case.source_manifest {
        if !is_valid_sha256(&source.sha256) {
            return Err(EvaluationError::InvalidSourceDigest(source.source_id.clone()));
        }
        if source.rights_status != RightsStatus::ClearedForTesting
            && source.rights_status != RightsStatus::ClearedForRedistribution
        {
            return Err(EvaluationError::SourceRightsNotAdmitted(source.source_id.clone()));
        }
    }
    let mut assertion_ids = BTreeSet::new();
    for assertion in &case.assertions {
        if assertion.assertion_id.trim().is_empty()
            || assertion.subject.id.trim().is_empty()
            || assertion.subject.kind.trim().is_empty()
            || assertion.source_locator.trim().is_empty()
        {
            return Err(EvaluationError::InvalidIdentity(assertion.assertion_id.clone()));
        }
        if !assertion_ids.insert(assertion.assertion_id.clone()) {
            return Err(EvaluationError::DuplicateExpectedAssertion(assertion.assertion_id.clone()));
        }
    }
    Ok(())
}

/// Apply one deterministic adversarial mutation to a normalized assertion set.
///
/// The returned unsupported set is explicit: an omitted assertion is classified
/// as unsupported only when the adapter declares that limitation; otherwise a
/// missing required assertion is semantic loss.
pub fn apply_mutation(
    assertions: &[SemanticAssertionV1],
    mutation: &MutationControlV1,
) -> Result<(Vec<SemanticAssertionV1>, BTreeSet<String>), EvaluationError> {
    let mut observed = assertions.to_vec();
    let mut unsupported = BTreeSet::new();
    let Some(index) = observed
        .iter()
        .position(|item| item.assertion_id == mutation.target_assertion_id)
    else {
        return Err(EvaluationError::MutationTargetNotFound(
            mutation.target_assertion_id.clone(),
        ));
    };

    match mutation.operation {
        MutationOperation::DropField => {
            observed.remove(index);
        }
        MutationOperation::OmitUnsupportedField => {
            observed.remove(index);
            unsupported.insert(mutation.target_assertion_id.clone());
        }
        MutationOperation::ChangeValue => {
            observed[index].value = Value::Null;
        }
        MutationOperation::ChangeUnit => {
            observed[index].unit = Some("mutation-unit".to_owned());
        }
        MutationOperation::ChangeToleranceType => {
            let mut value = observed[index]
                .value
                .as_object()
                .cloned()
                .unwrap_or_default();
            value.insert("kind".to_owned(), Value::String("mutated_tolerance_kind".to_owned()));
            observed[index].value = Value::Object(value);
        }
        MutationOperation::ChangeDatum => {
            observed[index].value = serde_json::json!({"datum": "B", "order": 2});
        }
        MutationOperation::ChangeSubjectId
        | MutationOperation::DuplicatePersistentId
        | MutationOperation::StaleRevision => {
            observed[index].subject.id = format!("mutated:{}", observed[index].subject.id);
        }
        MutationOperation::ShiftEventTime => {
            observed[index].value = match observed[index].value.as_i64() {
                Some(value) => serde_json::json!(value.saturating_add(1)),
                None => serde_json::json!({"mutated_time": true}),
            };
        }
        MutationOperation::DropTraceEdge => {
            observed[index].requirement = None;
        }
        MutationOperation::ReplaceSemanticWithGraphicOnly => {
            observed[index].semantic_type = SemanticType::FeatureAssociation;
        }
    }
    Ok((observed, unsupported))
}

/// Compare two normalized assertion sets. Source locators are intentionally
/// not compared because different standards/toolchains use different locators.
pub fn evaluate_assertions(
    benchmark_id: &str,
    case_id: &str,
    expected: &[SemanticAssertionV1],
    observed: &[SemanticAssertionV1],
    unsupported_assertions: &BTreeSet<String>,
) -> Result<EvaluationReport, EvaluationError> {
    let mut expected_map = BTreeMap::new();
    for assertion in expected {
        if expected_map.insert(assertion.assertion_id.clone(), assertion).is_some() {
            return Err(EvaluationError::DuplicateExpectedAssertion(assertion.assertion_id.clone()));
        }
    }
    let mut observed_map = BTreeMap::new();
    for assertion in observed {
        if observed_map.insert(assertion.assertion_id.clone(), assertion).is_some() {
            return Err(EvaluationError::DuplicateObservedAssertion(assertion.assertion_id.clone()));
        }
    }

    let mut results = Vec::new();
    for (id, expected_assertion) in &expected_map {
        let result = match observed_map.get(id) {
            None if unsupported_assertions.contains(id) => AssertionResult {
                assertion_id: id.clone(),
                status: AssertionStatus::Unsupported,
                explanation: "adapter explicitly declared this assertion unsupported".into(),
            },
            None if expected_assertion.required => AssertionResult {
                assertion_id: id.clone(),
                status: AssertionStatus::SemanticLoss,
                explanation: "required assertion is absent from the observed assertion set".into(),
            },
            None => AssertionResult {
                assertion_id: id.clone(),
                status: AssertionStatus::NotRequired,
                explanation: "optional assertion is absent".into(),
            },
            Some(actual) if actual.subject != expected_assertion.subject
                || actual.requirement != expected_assertion.requirement => AssertionResult {
                assertion_id: id.clone(),
                status: AssertionStatus::WrongSubjectJoin,
                explanation: "assertion is linked to a different subject or requirement".into(),
            },
            Some(actual) if actual.semantic_type != expected_assertion.semantic_type
                || actual.value != expected_assertion.value
                || actual.unit != expected_assertion.unit => AssertionResult {
                assertion_id: id.clone(),
                status: AssertionStatus::SemanticLoss,
                explanation: "semantic type, value or unit changed".into(),
            },
            Some(_) => AssertionResult {
                assertion_id: id.clone(),
                status: AssertionStatus::Preserved,
                explanation: "normalized semantic assertion matches".into(),
            },
        };
        results.push(result);
    }

    for id in observed_map.keys() {
        if !expected_map.contains_key(id) {
            results.push(AssertionResult {
                assertion_id: id.clone(),
                status: AssertionStatus::Ambiguous,
                explanation: "unexpected assertion has no corresponding frozen source assertion".into(),
            });
        }
    }
    results.sort_by(|left, right| left.assertion_id.cmp(&right.assertion_id));

    let mut counts = EvaluationCounts {
        preserved: 0,
        semantic_loss: 0,
        wrong_subject_join: 0,
        unsupported: 0,
        ambiguous: 0,
        not_required: 0,
    };
    for result in &results {
        match result.status {
            AssertionStatus::Preserved => counts.preserved += 1,
            AssertionStatus::SemanticLoss => counts.semantic_loss += 1,
            AssertionStatus::WrongSubjectJoin => counts.wrong_subject_join += 1,
            AssertionStatus::Unsupported => counts.unsupported += 1,
            AssertionStatus::Ambiguous => counts.ambiguous += 1,
            AssertionStatus::NotRequired => counts.not_required += 1,
        }
    }
    let disposition = if counts.wrong_subject_join > 0 {
        AssertionStatus::WrongSubjectJoin
    } else if counts.ambiguous > 0 {
        AssertionStatus::Ambiguous
    } else if counts.semantic_loss > 0 {
        AssertionStatus::SemanticLoss
    } else if counts.unsupported > 0 {
        AssertionStatus::Unsupported
    } else {
        AssertionStatus::Preserved
    };

    Ok(EvaluationReport {
        benchmark_id: benchmark_id.to_owned(),
        case_id: case_id.to_owned(),
        disposition,
        counts,
        results,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reference(id: &str, kind: &str) -> SemanticRefV1 {
        SemanticRefV1 { id: id.into(), kind: kind.into() }
    }

    fn assertion(id: &str, value: Value, unit: Option<&str>) -> SemanticAssertionV1 {
        SemanticAssertionV1 {
            assertion_id: id.into(),
            semantic_type: SemanticType::NominalDimension,
            subject: reference("feature:hole-01", "feature"),
            requirement: Some(reference("requirement:hole-diameter", "requirement")),
            value,
            unit: unit.map(str::to_owned),
            source_locator: format!("synthetic://source/{id}"),
            required: true,
        }
    }

    fn fixture_case() -> SemanticCaseV1 {
        let bytes = include_bytes!("../../../fixtures/aero-semantic-interop/reference-cases-v1.json");
        parse_semantic_case(bytes).expect("semantic fixture parses")
    }

    #[test]
    fn checked_in_semantic_fixture_parses_and_passes_rights_gate() {
        let case = fixture_case();
        assert_eq!(case.assertions.len(), 6);
        assert_eq!(case.mutations.len(), 6);
        assert_eq!(validate_case(&case), Ok(()));
        let baseline = evaluate_assertions(
            &case.benchmark_id,
            &case.case_id,
            &case.assertions,
            &case.assertions,
            &BTreeSet::new(),
        ).expect("baseline assertion parity");
        assert_eq!(baseline.disposition, AssertionStatus::Preserved);
        assert_eq!(baseline.counts.preserved, case.assertions.len());
    }

    #[test]
    fn exact_assertion_set_is_preserved() {
        let value = assertion("dimension", serde_json::json!(10.0), Some("mm"));
        let report = evaluate_assertions("bench", "case", &[value.clone()], &[value], &BTreeSet::new()).unwrap();
        assert_eq!(report.disposition, AssertionStatus::Preserved);
        assert_eq!(report.counts.preserved, 1);
    }

    #[test]
    fn unit_change_is_semantic_loss_not_silent_success() {
        let expected = assertion("dimension", serde_json::json!(10.0), Some("mm"));
        let observed = assertion("dimension", serde_json::json!(10.0), Some("cm"));
        let report = evaluate_assertions("bench", "case", &[expected], &[observed], &BTreeSet::new()).unwrap();
        assert_eq!(report.disposition, AssertionStatus::SemanticLoss);
    }

    #[test]
    fn wrong_feature_or_requirement_link_is_rejected() {
        let expected = assertion("dimension", serde_json::json!(10.0), Some("mm"));
        let mut observed = expected.clone();
        observed.subject = reference("feature:other", "feature");
        let report = evaluate_assertions("bench", "case", &[expected.clone()], &[observed], &BTreeSet::new()).unwrap();
        assert_eq!(report.disposition, AssertionStatus::WrongSubjectJoin);

        let mut observed = expected.clone();
        observed.requirement = Some(reference("requirement:other", "requirement"));
        let report = evaluate_assertions("bench", "case", &[expected], &[observed], &BTreeSet::new()).unwrap();
        assert_eq!(report.disposition, AssertionStatus::WrongSubjectJoin);
    }

    #[test]
    fn missing_required_and_explicitly_unsupported_are_distinct() {
        let expected = assertion("dimension", serde_json::json!(10.0), Some("mm"));
        let missing = evaluate_assertions("bench", "case", &[expected.clone()], &[], &BTreeSet::new()).unwrap();
        assert_eq!(missing.disposition, AssertionStatus::SemanticLoss);
        let unsupported = evaluate_assertions(
            "bench",
            "case",
            &[expected],
            &[],
            &BTreeSet::from(["dimension".to_owned()]),
        ).unwrap();
        assert_eq!(unsupported.disposition, AssertionStatus::Unsupported);
    }

    #[test]
    fn unexpected_and_duplicate_observations_are_not_ignored() {
        let expected = assertion("dimension", serde_json::json!(10.0), Some("mm"));
        let unexpected = assertion("extra", serde_json::json!(1), None);
        let report = evaluate_assertions("bench", "case", &[expected.clone()], &[expected.clone(), unexpected], &BTreeSet::new()).unwrap();
        assert_eq!(report.disposition, AssertionStatus::Ambiguous);
        assert!(matches!(
            evaluate_assertions("bench", "case", &[expected.clone()], &[expected.clone(), expected], &BTreeSet::new()),
            Err(EvaluationError::DuplicateObservedAssertion(_))
        ));
    }

    #[test]
    fn input_order_does_not_change_evaluation() {
        let a = assertion("a", serde_json::json!(10.0), Some("mm"));
        let b = assertion("b", serde_json::json!(20.0), Some("mm"));
        let first = evaluate_assertions("bench", "case", &[a.clone(), b.clone()], &[a.clone(), b.clone()], &BTreeSet::new()).unwrap();
        let second = evaluate_assertions("bench", "case", &[b.clone(), a.clone()], &[b, a], &BTreeSet::new()).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn frozen_mutation_corpus_matches_expected_dispositions() {
        let case = fixture_case();
        for mutation in &case.mutations {
            let (observed, unsupported) =
                apply_mutation(&case.assertions, mutation).expect("valid mutation target");
            let report = evaluate_assertions(
                &case.benchmark_id,
                &format!("{}:{}", case.case_id, mutation.mutation_id),
                &case.assertions,
                &observed,
                &unsupported,
            ).expect("evaluate mutation");
            let expected = match mutation.expected_disposition.as_str() {
                "PRESERVED" => AssertionStatus::Preserved,
                "SEMANTIC_LOSS" => AssertionStatus::SemanticLoss,
                "WRONG_SUBJECT_JOIN" => AssertionStatus::WrongSubjectJoin,
                "UNSUPPORTED" => AssertionStatus::Unsupported,
                "AMBIGUOUS" => AssertionStatus::Ambiguous,
                other => panic!("unexpected mutation disposition: {other}"),
            };
            assert_eq!(report.disposition, expected, "mutation {}", mutation.mutation_id);
        }
    }

    #[test]
    fn strict_parser_rejects_missing_mutation_fields() {
        let mut value: Value = serde_json::from_slice(
            include_bytes!("../../../fixtures/aero-semantic-interop/reference-cases-v1.json"),
        ).unwrap();
        value["mutations"][0].as_object_mut().unwrap().remove("operation");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(matches!(parse_semantic_case(&bytes), Err(EvaluationError::MissingField { .. })));
    }

    #[test]
    fn external_source_manifest_must_be_rights_admitted() {
        let mut case = fixture_case();
        case.source_class = SourceClass::ExternalReference;
        case.source_manifest.push(SemanticSourceV1 {
            source_id: "cax-if-test-case-7".into(),
            publisher: "CAx-IF".into(),
            url: "https://www.mbx-if.org/home/cax/recpractices/".into(),
            version: "2025-12-16".into(),
            acquired_at: "2026-11-20T00:00:00Z".into(),
            rights_status: RightsStatus::NotReviewed,
            sha256: "a".repeat(64),
        });
        assert!(matches!(validate_case(&case), Err(EvaluationError::SourceRightsNotAdmitted(_))));
        case.source_manifest[0].rights_status = RightsStatus::ClearedForTesting;
        assert_eq!(validate_case(&case), Ok(()));
    }

    #[test]
    fn strict_parser_rejects_unknown_nested_assertion_fields() {
        let mut value: Value = serde_json::from_slice(
            include_bytes!("../../../fixtures/aero-semantic-interop/reference-cases-v1.json"),
        ).unwrap();
        value["assertions"][0]["hidden_approval"] = serde_json::json!(true);
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(matches!(parse_semantic_case(&bytes), Err(EvaluationError::UnknownField { .. })));
    }

    #[test]
    fn strict_parser_rejects_missing_optional_fields() {
        let mut value: Value = serde_json::from_slice(
            include_bytes!("../../../fixtures/aero-semantic-interop/reference-cases-v1.json"),
        ).unwrap();
        value["assertions"][0].as_object_mut().unwrap().remove("requirement");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(matches!(parse_semantic_case(&bytes), Err(EvaluationError::MissingField { .. })));
    }
}
