// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Time-aware, oracle-separated qualification for history-grounded tutoring.
//!
//! This module deliberately does not equate "newer" with "truer". It distinguishes
//! event time from ingestion time, explicit validity windows from age, and source
//! retractions/supersessions from historical learner behavior.

use super::tutor_history_benchmark::{
    AssistanceLevel, HistoryAction, HistoryDiagnosis, HistoryDisposition, HistoryOracleLabel,
    HistoryPrediction, ObservationOrigin,
};
use serde::{de::Deserializer, Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

const BENCHMARK_ID: &str = "PRAXIS-TUTOR-TEMPORAL-EVIDENCE-001";

fn required_option<'de, D, T>(deserializer: D) -> Result<Option<T>, D::Error>
where
    D: Deserializer<'de>,
    T: Deserialize<'de>,
{
    Option::<T>::deserialize(deserializer)
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum RecordStatus {
    Active,
    Retracted,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct TemporalObservation {
    pub observation_id: String,
    pub sequence: u32,
    #[serde(deserialize_with = "required_option")]
    pub skill_id: Option<String>,
    #[serde(deserialize_with = "required_option")]
    pub task_context: Option<String>,
    #[serde(deserialize_with = "required_option")]
    pub correctness: Option<bool>,
    pub assistance: AssistanceLevel,
    pub origin: ObservationOrigin,
    /// When the learner event or assessment being described occurred (Unix seconds).
    pub occurred_at_unix: i64,
    /// When this exact record entered the evidence store (Unix seconds).
    pub recorded_at_unix: i64,
    /// Optional, explicit end of the record's current-state validity window.
    /// Null means "no expiry asserted", not "permanently valid".
    #[serde(deserialize_with = "required_option")]
    pub valid_until_unix: Option<i64>,
    pub status: RecordStatus,
    /// Provenance correction/replacement edges. This does not erase history.
    pub supersedes: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct TemporalCase {
    pub case_id: String,
    pub evaluation_time_unix: i64,
    pub observations: Vec<TemporalObservation>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct TemporalManifest {
    pub schema_version: u32,
    pub benchmark_id: String,
    pub cases: Vec<TemporalCase>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct TemporalOracle {
    pub schema_version: u32,
    pub benchmark_id: String,
    pub labels: Vec<HistoryOracleLabel>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct TemporalPredictionManifest {
    pub schema_version: u32,
    pub benchmark_id: String,
    /// BLAKE3 of the exact solver-visible observation-manifest bytes.
    pub observation_blake3: String,
    pub predictions: Vec<HistoryPrediction>,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum TemporalEligibility {
    Eligible,
    Expired,
    Retracted,
    Superseded,
    FutureOccurrence,
    NotRecordedAtEvaluation,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TemporalFinding {
    pub code: &'static str,
    pub case_id: Option<String>,
    pub detail: String,
}

fn finding(code: &'static str, case_id: Option<&str>, detail: impl Into<String>) -> TemporalFinding {
    TemporalFinding {
        code,
        case_id: case_id.map(str::to_owned),
        detail: detail.into(),
    }
}

fn parse_error(field: &'static str, error: serde_json::Error) -> Vec<TemporalFinding> {
    vec![finding("json_parse", None, format!("{field}: {error}"))]
}

/// Classify whether an observation can support a claim about the state at the
/// case's evaluation time. Old observations can remain useful historical context;
/// only an explicit validity deadline expires current-state support.
pub fn temporal_eligibility(
    case: &TemporalCase,
    observation: &TemporalObservation,
) -> TemporalEligibility {
    if observation.occurred_at_unix > case.evaluation_time_unix {
        return TemporalEligibility::FutureOccurrence;
    }
    if observation.recorded_at_unix > case.evaluation_time_unix {
        return TemporalEligibility::NotRecordedAtEvaluation;
    }
    if observation.status == RecordStatus::Retracted {
        return TemporalEligibility::Retracted;
    }
    if case.observations.iter().any(|candidate| {
        candidate.status == RecordStatus::Active
            && candidate.recorded_at_unix <= case.evaluation_time_unix
            && candidate.supersedes.iter().any(|id| id == &observation.observation_id)
    }) {
        return TemporalEligibility::Superseded;
    }
    if observation
        .valid_until_unix
        .is_some_and(|deadline| case.evaluation_time_unix > deadline)
    {
        return TemporalEligibility::Expired;
    }
    TemporalEligibility::Eligible
}

fn is_semantically_correct(prediction: &HistoryPrediction, label: &HistoryOracleLabel) -> bool {
    label.accepted_dispositions.contains(&prediction.disposition)
        && match prediction.disposition {
            HistoryDisposition::Claim => {
                prediction
                    .diagnosis
                    .is_some_and(|value| label.acceptable_diagnoses.contains(&value))
                    && prediction
                        .next_action
                        .is_some_and(|value| label.acceptable_actions.contains(&value))
            }
            HistoryDisposition::RequestMoreEvidence => {
                prediction.requested_evidence == label.requested_evidence
            }
            HistoryDisposition::Abstain => true,
        }
}

#[derive(Clone, Debug, Serialize, PartialEq)]
pub struct TemporalBenchmarkScore {
    pub benchmark_id: String,
    pub observation_blake3: String,
    pub oracle_blake3: String,
    pub case_count: usize,
    pub decision_fit_decisions: usize,
    pub decision_fit_rate: f64,
    pub acceptable_decisions: usize,
    pub acceptable_decision_rate: f64,
    pub claim_count: usize,
    pub evidence_grounded_claims: usize,
    pub evidence_grounded_claim_rate: Option<f64>,
    pub expired_support_references: usize,
    pub retracted_or_superseded_references: usize,
    pub future_or_unavailable_references: usize,
    pub unsupported_evidence_references: usize,
    pub missing_required_contradictions: usize,
    pub critical_violations: usize,
}

/// Validate a temporally explicit manifest and a separate oracle. The oracle must
/// not be exposed to the candidate; its labels only enter the independent scorer.
pub fn validate_temporal_bundle(
    observations_json: &str,
    oracle_json: &str,
) -> Result<(TemporalManifest, TemporalOracle), Vec<TemporalFinding>> {
    let observations: TemporalManifest = match serde_json::from_str(observations_json) {
        Ok(value) => value,
        Err(error) => return Err(parse_error("observations", error)),
    };
    let oracle: TemporalOracle = match serde_json::from_str(oracle_json) {
        Ok(value) => value,
        Err(error) => return Err(parse_error("oracle", error)),
    };
    let mut findings = Vec::new();

    if observations.schema_version != 1 || oracle.schema_version != 1 {
        findings.push(finding("unsupported_schema_version", None, "only temporal schema_version 1 is supported"));
    }
    if observations.benchmark_id != BENCHMARK_ID || oracle.benchmark_id != BENCHMARK_ID {
        findings.push(finding("benchmark_id_mismatch", None, "both temporal artifacts must use the temporal benchmark ID"));
    }
    if observations.cases.is_empty() || oracle.labels.is_empty() {
        findings.push(finding("empty_bundle", None, "observations and oracle must both be non-empty"));
    }

    let mut cases_by_id: BTreeMap<String, &TemporalCase> = BTreeMap::new();
    for case in &observations.cases {
        if case.case_id.trim().is_empty() || cases_by_id.insert(case.case_id.clone(), case).is_some() {
            findings.push(finding("duplicate_or_empty_case_id", Some(&case.case_id), "case IDs must be non-empty and unique"));
        }
        if case.evaluation_time_unix <= 0 {
            findings.push(finding("invalid_evaluation_time", Some(&case.case_id), "evaluation time must be a positive Unix timestamp"));
        }
        if case.observations.is_empty() {
            findings.push(finding("empty_history", Some(&case.case_id), "each temporal case needs at least one observation"));
        }

        let mut sequences = BTreeSet::new();
        let mut observations_by_id: BTreeMap<String, &TemporalObservation> = BTreeMap::new();
        for observation in &case.observations {
            if observation.observation_id.trim().is_empty()
                || observations_by_id.insert(observation.observation_id.clone(), observation).is_some()
            {
                findings.push(finding("duplicate_or_empty_observation_id", Some(&case.case_id), "observation IDs must be non-empty and unique within each case"));
            }
            if observation.sequence == 0 || !sequences.insert(observation.sequence) {
                findings.push(finding("duplicate_or_zero_sequence", Some(&case.case_id), "sequence values must be positive and unique"));
            }
            if observation.occurred_at_unix <= 0 || observation.recorded_at_unix <= 0 {
                findings.push(finding("invalid_timestamp", Some(&case.case_id), "occurrence and record timestamps must be positive Unix seconds"));
            }
            if observation.recorded_at_unix < observation.occurred_at_unix {
                findings.push(finding("recorded_before_occurred", Some(&case.case_id), format!("{} was recorded before its declared occurrence", observation.observation_id)));
            }
            if observation.occurred_at_unix > case.evaluation_time_unix {
                findings.push(finding("future_occurrence_leakage", Some(&case.case_id), format!("{} occurs after the case evaluation time", observation.observation_id)));
            }
            if observation.recorded_at_unix > case.evaluation_time_unix {
                findings.push(finding("future_record_leakage", Some(&case.case_id), format!("{} was not recorded by the case evaluation time", observation.observation_id)));
            }
            if observation.valid_until_unix.is_some_and(|deadline| deadline <= observation.occurred_at_unix) {
                findings.push(finding("invalid_validity_window", Some(&case.case_id), format!("{} expires no later than its occurrence", observation.observation_id)));
            }
            if observation.origin == ObservationOrigin::DerivedEstimate && observation.correctness.is_some() {
                findings.push(finding("estimate_masquerades_as_observation", Some(&case.case_id), format!("{} assigns observed correctness to a derived estimate", observation.observation_id)));
            }
        }

        // A supersession is a provenance edge, not an instruction to discard the
        // historical event. Targets must exist and the correction cannot predate
        // the record it claims to replace.
        for observation in &case.observations {
            for target_id in &observation.supersedes {
                if target_id == &observation.observation_id {
                    findings.push(finding("self_supersession", Some(&case.case_id), format!("{} supersedes itself", observation.observation_id)));
                    continue;
                }
                match observations_by_id.get(target_id) {
                    None => findings.push(finding("supersession_target_missing", Some(&case.case_id), format!("{} supersedes unknown ID {target_id}", observation.observation_id))),
                    Some(target) if observation.recorded_at_unix < target.recorded_at_unix => {
                        findings.push(finding("supersession_time_inversion", Some(&case.case_id), format!("{} was recorded before the record it supersedes", observation.observation_id)));
                    }
                    Some(_) => {}
                }
            }
        }
        let mut visiting = BTreeSet::new();
        let mut visited = BTreeSet::new();
        for observation in &case.observations {
            if has_supersession_cycle(&observation.observation_id, &observations_by_id, &mut visiting, &mut visited) {
                findings.push(finding("supersession_cycle", Some(&case.case_id), "supersession graph must be acyclic"));
                break;
            }
        }
    }

    let mut label_ids = BTreeSet::new();
    for label in &oracle.labels {
        if label.case_id.trim().is_empty() || !label_ids.insert(label.case_id.clone()) {
            findings.push(finding("duplicate_or_empty_oracle_case", Some(&label.case_id), "oracle case IDs must be non-empty and unique"));
        }
        let Some(case) = cases_by_id.get(&label.case_id) else {
            findings.push(finding("oracle_case_missing_observations", Some(&label.case_id), "oracle case does not exist in observations"));
            continue;
        };
        let known: BTreeSet<&str> = case.observations.iter().map(|observation| observation.observation_id.as_str()).collect();
        if label.accepted_dispositions.is_empty() {
            findings.push(finding("empty_accepted_dispositions", Some(&label.case_id), "each oracle label needs an acceptable disposition"));
        }
        if label.accepted_dispositions.contains(&HistoryDisposition::Claim)
            && (label.acceptable_diagnoses.is_empty() || label.acceptable_actions.is_empty())
        {
            findings.push(finding("incomplete_claim_oracle", Some(&label.case_id), "claim labels need acceptable diagnoses and actions"));
        }
        if label.accepted_dispositions.contains(&HistoryDisposition::RequestMoreEvidence)
            && label.requested_evidence.is_none()
        {
            findings.push(finding("missing_requested_evidence_oracle", Some(&label.case_id), "request_more_evidence needs a specific evidence class"));
        }
        for id in label.required_supporting_evidence_ids.iter().chain(&label.required_contradicting_evidence_ids) {
            if !known.contains(id.as_str()) {
                findings.push(finding("oracle_reference_missing", Some(&label.case_id), format!("oracle references unknown observation ID {id}")));
            }
        }
    }
    if label_ids != cases_by_id.keys().cloned().collect() {
        findings.push(finding("case_cardinality_mismatch", None, "temporal observations and oracle must contain exactly the same case IDs"));
    }

    if findings.is_empty() {
        Ok((observations, oracle))
    } else {
        Err(findings)
    }
}

fn has_supersession_cycle(
    id: &str,
    nodes: &BTreeMap<String, &TemporalObservation>,
    visiting: &mut BTreeSet<String>,
    visited: &mut BTreeSet<String>,
) -> bool {
    if visited.contains(id) {
        return false;
    }
    if !visiting.insert(id.to_owned()) {
        return true;
    }
    if let Some(node) = nodes.get(id) {
        for parent in &node.supersedes {
            if nodes.contains_key(parent) && has_supersession_cycle(parent, nodes, visiting, visited) {
                return true;
            }
        }
    }
    visiting.remove(id);
    visited.insert(id.to_owned());
    false
}

/// Score decision fit separately from evidence-grounded acceptability. Evidence
/// that is expired may be cited as historical context, but not as current-state
/// support. Retracted/superseded/future/unavailable records are not valid citations.
pub fn score_temporal_predictions(
    observations_json: &str,
    oracle_json: &str,
    predictions_json: &str,
) -> Result<TemporalBenchmarkScore, Vec<TemporalFinding>> {
    let (observations, oracle) = validate_temporal_bundle(observations_json, oracle_json)?;
    let predictions: TemporalPredictionManifest = match serde_json::from_str(predictions_json) {
        Ok(value) => value,
        Err(error) => return Err(parse_error("predictions", error)),
    };
    let observation_digest = blake3::hash(observations_json.as_bytes()).to_hex().to_string();
    let oracle_digest = blake3::hash(oracle_json.as_bytes()).to_hex().to_string();
    let mut findings = Vec::new();
    if predictions.schema_version != 1 || predictions.benchmark_id != BENCHMARK_ID {
        findings.push(finding("prediction_identity_mismatch", None, "unsupported temporal prediction schema or benchmark ID"));
    }
    if predictions.observation_blake3 != observation_digest {
        findings.push(finding("observation_digest_mismatch", None, "predictions are not bound to the exact temporal observation JSON bytes"));
    }

    let cases: BTreeMap<&str, &TemporalCase> = observations.cases.iter().map(|case| (case.case_id.as_str(), case)).collect();
    let labels: BTreeMap<&str, &HistoryOracleLabel> = oracle.labels.iter().map(|label| (label.case_id.as_str(), label)).collect();
    let mut predictions_by_case: BTreeMap<&str, &HistoryPrediction> = BTreeMap::new();
    for prediction in &predictions.predictions {
        if predictions_by_case.insert(prediction.case_id.as_str(), prediction).is_some() {
            findings.push(finding("duplicate_prediction", Some(&prediction.case_id), "one prediction per case is required"));
        }
        if !cases.contains_key(prediction.case_id.as_str()) {
            findings.push(finding("unknown_prediction_case", Some(&prediction.case_id), "prediction case is absent from observations"));
        }
        if labels.contains_key(prediction.case_id.as_str()) {
            let shape_valid = match prediction.disposition {
                HistoryDisposition::Claim => prediction.diagnosis.is_some() && prediction.next_action.is_some() && prediction.requested_evidence.is_none(),
                HistoryDisposition::RequestMoreEvidence => prediction.diagnosis.is_none() && prediction.next_action.is_none() && prediction.requested_evidence.is_some(),
                HistoryDisposition::Abstain => prediction.diagnosis.is_none() && prediction.next_action.is_none() && prediction.requested_evidence.is_none(),
            };
            if !shape_valid {
                findings.push(finding("inconsistent_prediction_shape", Some(&prediction.case_id), "disposition-specific prediction fields are inconsistent"));
            }
            let mut cited = BTreeSet::new();
            for id in prediction.supporting_evidence_ids.iter().chain(&prediction.contradicting_evidence_ids) {
                if !cited.insert(id.as_str()) {
                    findings.push(finding("duplicate_evidence_citation", Some(&prediction.case_id), format!("evidence ID {id} is cited more than once")));
                }
            }
        }
    }
    let expected: BTreeSet<&str> = cases.keys().copied().collect();
    if predictions_by_case.keys().copied().collect::<BTreeSet<_>>() != expected {
        findings.push(finding("prediction_cardinality_mismatch", None, "predictions must contain exactly one row per case"));
    }
    if !findings.is_empty() {
        return Err(findings);
    }

    let mut decision_fit = 0usize;
    let mut acceptable = 0usize;
    let mut claim_count = 0usize;
    let mut grounded_claims = 0usize;
    let mut expired_support = 0usize;
    let mut retracted_or_superseded = 0usize;
    let mut future_or_unavailable = 0usize;
    let mut unsupported = 0usize;
    let mut missing_contradictions = 0usize;
    let mut critical = 0usize;

    for (case_id, case) in &cases {
        let prediction = predictions_by_case[case_id];
        let label = labels[case_id];
        let semantic_fit = is_semantically_correct(prediction, label);
        if semantic_fit {
            decision_fit += 1;
        }
        if prediction.disposition == HistoryDisposition::Claim
            && !label.accepted_dispositions.contains(&HistoryDisposition::Claim)
        {
            critical += 1;
        }
        let by_id: BTreeMap<&str, &TemporalObservation> = case.observations.iter().map(|observation| (observation.observation_id.as_str(), observation)).collect();
        let mut claim_grounded = true;

        for (id, is_support) in prediction.supporting_evidence_ids.iter().map(|id| (id, true))
            .chain(prediction.contradicting_evidence_ids.iter().map(|id| (id, false)))
        {
            let Some(observation) = by_id.get(id.as_str()) else {
                unsupported += 1;
                critical += 1;
                claim_grounded = false;
                continue;
            };
            match temporal_eligibility(case, observation) {
                TemporalEligibility::Eligible => {}
                TemporalEligibility::Expired if is_support => {
                    expired_support += 1;
                    critical += 1;
                    claim_grounded = false;
                }
                TemporalEligibility::Expired => {}
                TemporalEligibility::Retracted | TemporalEligibility::Superseded => {
                    retracted_or_superseded += 1;
                    critical += 1;
                    claim_grounded = false;
                }
                TemporalEligibility::FutureOccurrence | TemporalEligibility::NotRecordedAtEvaluation => {
                    future_or_unavailable += 1;
                    critical += 1;
                    claim_grounded = false;
                }
            }
        }

        if prediction.disposition == HistoryDisposition::Claim {
            claim_count += 1;
            if prediction.supporting_evidence_ids.is_empty()
                || label.required_supporting_evidence_ids.iter().any(|id| !prediction.supporting_evidence_ids.contains(id))
                || label.required_contradicting_evidence_ids.iter().any(|id| !prediction.contradicting_evidence_ids.contains(id))
            {
                claim_grounded = false;
            }
            for required in &label.required_contradicting_evidence_ids {
                if !prediction.contradicting_evidence_ids.contains(required) {
                    missing_contradictions += 1;
                    critical += 1;
                }
            }
            if claim_grounded {
                grounded_claims += 1;
            }
            if semantic_fit && claim_grounded {
                acceptable += 1;
            }
        } else if semantic_fit {
            // Safe abstention and targeted clarification do not require claim citations.
            acceptable += 1;
        }
    }

    let ratio = |numerator: usize, denominator: usize| {
        if denominator == 0 { None } else { Some(numerator as f64 / denominator as f64) }
    };
    Ok(TemporalBenchmarkScore {
        benchmark_id: BENCHMARK_ID.to_owned(),
        observation_blake3: observation_digest,
        oracle_blake3: oracle_digest,
        case_count: cases.len(),
        decision_fit_decisions: decision_fit,
        decision_fit_rate: decision_fit as f64 / cases.len() as f64,
        acceptable_decisions: acceptable,
        acceptable_decision_rate: acceptable as f64 / cases.len() as f64,
        claim_count,
        evidence_grounded_claims: grounded_claims,
        evidence_grounded_claim_rate: ratio(grounded_claims, claim_count),
        expired_support_references: expired_support,
        retracted_or_superseded_references: retracted_or_superseded,
        future_or_unavailable_references: future_or_unavailable,
        unsupported_evidence_references: unsupported,
        missing_required_contradictions: missing_contradictions,
        critical_violations: critical,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const OBSERVATIONS: &str = include_str!("../examples/tutor-history/temporal-v1/observations.json");
    const ORACLE: &str = include_str!("../examples/tutor-history/temporal-v1/oracle.json");

    #[test]
    fn temporal_fixture_and_oracle_validate() {
        let (manifest, oracle) = validate_temporal_bundle(OBSERVATIONS, ORACLE).expect("valid fixture");
        assert_eq!(manifest.cases.len(), 9);
        assert_eq!(oracle.labels.len(), 9);
    }

    #[test]
    fn eligibility_uses_occurrence_time_not_import_time() {
        let (manifest, _) = validate_temporal_bundle(OBSERVATIONS, ORACLE).expect("valid fixture");
        let case = manifest.cases.iter().find(|case| case.case_id == "TE001").unwrap();
        let imported = case.observations.iter().find(|observation| observation.observation_id == "te001-old-failure").unwrap();
        assert!(imported.recorded_at_unix > imported.occurred_at_unix);
        assert_eq!(temporal_eligibility(case, imported), TemporalEligibility::Eligible);
    }

    #[test]
    fn explicitly_expired_evidence_cannot_support_current_state() {
        let (manifest, _) = validate_temporal_bundle(OBSERVATIONS, ORACLE).expect("valid fixture");
        let case = manifest.cases.iter().find(|case| case.case_id == "TE002").unwrap();
        let expired = case.observations.iter().find(|observation| observation.observation_id == "te002-expired-success").unwrap();
        assert_eq!(temporal_eligibility(case, expired), TemporalEligibility::Expired);
    }

    #[test]
    fn source_retraction_and_supersession_are_distinct_from_age() {
        let (manifest, _) = validate_temporal_bundle(OBSERVATIONS, ORACLE).expect("valid fixture");
        let case = manifest.cases.iter().find(|case| case.case_id == "TE003").unwrap();
        let old = case.observations.iter().find(|observation| observation.observation_id == "te003-original").unwrap();
        let replacement = case.observations.iter().find(|observation| observation.observation_id == "te003-correction").unwrap();
        assert_eq!(temporal_eligibility(case, old), TemporalEligibility::Retracted);
        assert_eq!(temporal_eligibility(case, replacement), TemporalEligibility::Eligible);
    }

    #[test]
    fn superseded_record_is_not_current_support_but_remains_in_history() {
        let (manifest, _) = validate_temporal_bundle(OBSERVATIONS, ORACLE).expect("valid fixture");
        let case = manifest.cases.iter().find(|case| case.case_id == "TE003").unwrap();
        let old = case.observations.iter().find(|observation| observation.observation_id == "te003-prior-active").unwrap();
        assert_eq!(temporal_eligibility(case, old), TemporalEligibility::Superseded);
    }

    #[test]
    fn future_record_mutation_is_rejected_as_leakage() {
        let changed = OBSERVATIONS.replace(
            "\"recorded_at_unix\": 1791504000",
            "\"recorded_at_unix\": 1891504000",
        );
        let errors = validate_temporal_bundle(&changed, ORACLE).expect_err("future record must fail closed");
        assert!(errors.iter().any(|error| error.code == "future_record_leakage"));
    }

    #[test]
    fn unknown_supersession_target_is_rejected() {
        let changed = OBSERVATIONS.replace(
            "\"supersedes\": []",
            "\"supersedes\": [\"missing-record\"]",
        );
        let errors = validate_temporal_bundle(&changed, ORACLE).expect_err("unknown supersession target must fail");
        assert!(errors.iter().any(|error| error.code == "supersession_target_missing"));
    }

    #[test]
    fn derived_estimates_cannot_masquerade_as_observed_correctness() {
        let changed = OBSERVATIONS.replace(
            "\"origin\": \"derived_estimate\",\n          \"occurred_at_unix\": 1790812800,\n          \"recorded_at_unix\": 1790812800,\n          \"valid_until_unix\": null,\n          \"status\": \"active\",\n          \"supersedes\": []",
            "\"origin\": \"derived_estimate\",\n          \"occurred_at_unix\": 1790812800,\n          \"recorded_at_unix\": 1790812800,\n          \"valid_until_unix\": null,\n          \"status\": \"active\",\n          \"supersedes\": []",
        );
        // Directly mutate one JSON correctness field next to a derived-estimate origin.
        let parsed: serde_json::Value = serde_json::from_str(&changed).unwrap();
        let mut value = parsed;
        let observations = value["cases"].as_array_mut().unwrap();
        let target = observations.iter_mut().flat_map(|case| case["observations"].as_array_mut().unwrap().iter_mut())
            .find(|event| event["origin"] == "derived_estimate").unwrap();
        target["correctness"] = serde_json::Value::Bool(true);
        let errors = validate_temporal_bundle(&value.to_string(), ORACLE).expect_err("derived estimate must fail");
        assert!(errors.iter().any(|error| error.code == "estimate_masquerades_as_observation"));
    }

    #[test]
    fn supersession_cycles_are_rejected() {
        let changed = OBSERVATIONS.replace(
            "\"observation_id\": \"te003-original\"",
            "\"observation_id\": \"te003-original\"",
        );
        // Build a tiny closed cycle without relying on mutation ordering in the fixture.
        let mut value: serde_json::Value = serde_json::from_str(&changed).unwrap();
        let case = value["cases"].as_array_mut().unwrap().iter_mut().find(|case| case["case_id"] == "TE003").unwrap();
        let events = case["observations"].as_array_mut().unwrap();
        events.iter_mut().find(|event| event["observation_id"] == "te003-original").unwrap()["status"] = "active".into();
        events.iter_mut().find(|event| event["observation_id"] == "te003-prior-active").unwrap()["supersedes"] = serde_json::json!(["te003-correction"]);
        let errors = validate_temporal_bundle(&value.to_string(), ORACLE).expect_err("cycle must fail");
        assert!(errors.iter().any(|error| error.code == "supersession_cycle"));
    }

    #[test]
    fn changed_observation_bytes_invalidate_prediction_binding() {
        let (manifest, _) = validate_temporal_bundle(OBSERVATIONS, ORACLE).expect("valid fixture");
        let predictions = TemporalPredictionManifest {
            schema_version: 1,
            benchmark_id: BENCHMARK_ID.to_owned(),
            observation_blake3: blake3::hash(OBSERVATIONS.as_bytes()).to_hex().to_string(),
            predictions: vec![],
        };
        let prediction_json = serde_json::to_string(&predictions).unwrap();
        let changed = format!("\n{OBSERVATIONS}");
        assert_ne!(manifest.cases.len(), 0);
        let errors = score_temporal_predictions(&changed, ORACLE, &prediction_json).expect_err("exact-byte change must invalidate binding");
        assert!(errors.iter().any(|error| error.code == "observation_digest_mismatch"));
    }
}
