// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Synthetic, oracle-separated benchmark for history-grounded teaching decisions.
//! This module scores benchmark contracts; it is not a learner-state estimator.

use serde::{de::Deserializer, Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

const BENCHMARK_ID: &str = "PRAXIS-TUTOR-HISTORY-001";

fn required_option<'de, D, T>(deserializer: D) -> Result<Option<T>, D::Error>
where
    D: Deserializer<'de>,
    T: Deserialize<'de>,
{
    Option::<T>::deserialize(deserializer)
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum AssistanceLevel {
    None,
    Hint,
    PartialSolution,
    FullSolution,
    Unknown,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ObservationOrigin {
    RecordedAttempt,
    EducatorObservation,
    ImportedSummary,
    DerivedEstimate,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct HistoryObservation {
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
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct HistoryCase {
    pub case_id: String,
    #[serde(deserialize_with = "required_option")]
    pub counterfactual_group: Option<String>,
    pub observations: Vec<HistoryObservation>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct HistoryManifest {
    pub schema_version: u32,
    pub benchmark_id: String,
    pub cases: Vec<HistoryCase>,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum HistoryDisposition {
    Claim,
    RequestMoreEvidence,
    Abstain,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum HistoryDiagnosis {
    PersistentMisconception,
    ScaffoldDependence,
    RecentUnassistedImprovement,
    ContextSpecificConflict,
    ProvenUnassistedProgress,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum HistoryAction {
    CollectUnassistedAttempt,
    FadedScaffoldPractice,
    SpacedRetrieval,
    ClarifyTaskContext,
    CompareContexts,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum RequestedEvidence {
    SkillMetadata,
    ContextMatchedAttempt,
    UnassistedAttempt,
    DelayedRetrieval,
    AssistanceProvenance,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct HistoryOracleLabel {
    pub case_id: String,
    pub accepted_dispositions: Vec<HistoryDisposition>,
    pub acceptable_diagnoses: Vec<HistoryDiagnosis>,
    pub acceptable_actions: Vec<HistoryAction>,
    pub required_supporting_evidence_ids: Vec<String>,
    pub required_contradicting_evidence_ids: Vec<String>,
    #[serde(deserialize_with = "required_option")]
    pub requested_evidence: Option<RequestedEvidence>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct HistoryOracle {
    pub schema_version: u32,
    pub benchmark_id: String,
    pub labels: Vec<HistoryOracleLabel>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct HistoryPrediction {
    pub case_id: String,
    pub disposition: HistoryDisposition,
    #[serde(deserialize_with = "required_option")]
    pub diagnosis: Option<HistoryDiagnosis>,
    #[serde(deserialize_with = "required_option")]
    pub next_action: Option<HistoryAction>,
    pub supporting_evidence_ids: Vec<String>,
    pub contradicting_evidence_ids: Vec<String>,
    #[serde(deserialize_with = "required_option")]
    pub requested_evidence: Option<RequestedEvidence>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct HistoryPredictionManifest {
    pub schema_version: u32,
    pub benchmark_id: String,
    pub observation_blake3: String,
    pub predictions: Vec<HistoryPrediction>,
}

#[derive(Clone, Debug, Serialize, PartialEq)]
pub struct HistoryBenchmarkScore {
    pub benchmark_id: String,
    pub observation_blake3: String,
    pub oracle_blake3: String,
    pub case_count: usize,
    pub acceptable_decisions: usize,
    pub acceptable_decision_rate: f64,
    pub required_supporting_evidence_coverage: Option<f64>,
    pub required_contradicting_evidence_coverage: Option<f64>,
    pub supporting_reference_precision: Option<f64>,
    pub contradicting_reference_precision: Option<f64>,
    pub unsupported_evidence_references: usize,
    pub missing_required_contradictions: usize,
    pub critical_violations: usize,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HistoryFinding {
    pub code: &'static str,
    pub case_id: Option<String>,
    pub detail: String,
}

fn finding(code: &'static str, case_id: Option<&str>, detail: impl Into<String>) -> HistoryFinding {
    HistoryFinding {
        code,
        case_id: case_id.map(str::to_owned),
        detail: detail.into(),
    }
}

fn parse_error(field: &'static str, error: serde_json::Error) -> Vec<HistoryFinding> {
    vec![finding("json_parse", None, format!("{field}: {error}"))]
}

/// Parse and cross-check the solver-visible manifest against the separate oracle.
/// The caller must not send the oracle to the candidate system.
pub fn validate_history_bundle(
    observations_json: &str,
    oracle_json: &str,
) -> Result<(HistoryManifest, HistoryOracle), Vec<HistoryFinding>> {
    let observations: HistoryManifest = match serde_json::from_str(observations_json) {
        Ok(value) => value,
        Err(error) => return Err(parse_error("observations", error)),
    };
    let oracle: HistoryOracle = match serde_json::from_str(oracle_json) {
        Ok(value) => value,
        Err(error) => return Err(parse_error("oracle", error)),
    };
    let mut findings = Vec::new();

    if observations.schema_version != 1 || oracle.schema_version != 1 {
        findings.push(finding("unsupported_schema_version", None, "only schema_version 1 is supported"));
    }
    if observations.benchmark_id != BENCHMARK_ID || oracle.benchmark_id != BENCHMARK_ID {
        findings.push(finding("benchmark_id_mismatch", None, "both artifacts must use the v1 benchmark ID"));
    }

    let mut case_ids = BTreeSet::new();
    let mut case_observation_ids: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for case in &observations.cases {
        if case.case_id.trim().is_empty() || !case_ids.insert(case.case_id.clone()) {
            findings.push(finding("duplicate_or_empty_case_id", Some(case.case_id.as_str()), "case IDs must be non-empty and unique"));
        }
        if case.observations.is_empty() {
            findings.push(finding("empty_history", Some(case.case_id.as_str()), "each case requires at least one observation"));
        }
        let mut event_ids = BTreeSet::new();
        let mut sequences = BTreeSet::new();
        let mut known_ids = BTreeSet::new();
        for event in &case.observations {
            if event.observation_id.trim().is_empty() || !event_ids.insert(event.observation_id.clone()) {
                findings.push(finding("duplicate_or_empty_observation_id", Some(case.case_id.as_str()), "observation IDs must be non-empty and unique within a case"));
            }
            known_ids.insert(event.observation_id.clone());
            if event.sequence == 0 || !sequences.insert(event.sequence) {
                findings.push(finding("duplicate_or_zero_sequence", Some(case.case_id.as_str()), "sequence values must be positive and unique; input row order is not significant"));
            }
            if event.origin == ObservationOrigin::DerivedEstimate && event.correctness.is_some() {
                findings.push(finding("estimate_masquerades_as_observation", Some(case.case_id.as_str()), "derived estimates cannot be represented as recorded correctness"));
            }
        }
        case_observation_ids.insert(case.case_id.clone(), known_ids);
    }

    let mut label_ids = BTreeSet::new();
    for label in &oracle.labels {
        if label.case_id.trim().is_empty() || !label_ids.insert(label.case_id.clone()) {
            findings.push(finding("duplicate_or_empty_oracle_case", Some(label.case_id.as_str()), "oracle case IDs must be non-empty and unique"));
        }
        let Some(known) = case_observation_ids.get(&label.case_id) else {
            findings.push(finding("oracle_case_missing_observations", Some(label.case_id.as_str()), "oracle case does not exist in observations"));
            continue;
        };
        if label.accepted_dispositions.is_empty() {
            findings.push(finding("empty_accepted_dispositions", Some(label.case_id.as_str()), "each oracle label needs at least one acceptable disposition"));
        }
        if label.accepted_dispositions.contains(&HistoryDisposition::Claim)
            && (label.acceptable_diagnoses.is_empty() || label.acceptable_actions.is_empty())
        {
            findings.push(finding("incomplete_claim_oracle", Some(label.case_id.as_str()), "claim is acceptable only when diagnosis and action labels are supplied"));
        }
        if label.accepted_dispositions.contains(&HistoryDisposition::RequestMoreEvidence)
            && label.requested_evidence.is_none()
        {
            findings.push(finding("missing_requested_evidence_oracle", Some(label.case_id.as_str()), "request_more_evidence needs a specific evidence class"));
        }
        for id in label.required_supporting_evidence_ids.iter().chain(&label.required_contradicting_evidence_ids) {
            if !known.contains(id) {
                findings.push(finding("oracle_reference_missing", Some(label.case_id.as_str()), format!("oracle references unknown observation ID {id}")));
            }
        }
    }

    if case_ids != label_ids {
        findings.push(finding("case_cardinality_mismatch", None, "observations and oracle must contain exactly the same case IDs"));
    }
    if findings.is_empty() {
        Ok((observations, oracle))
    } else {
        Err(findings)
    }
}

/// Score one exact prediction set. Structural and digest failures reject the run;
/// incorrect diagnoses/actions and missing/unsupported evidence are reported as metrics.
pub fn score_history_predictions(
    observations_json: &str,
    oracle_json: &str,
    predictions_json: &str,
) -> Result<HistoryBenchmarkScore, Vec<HistoryFinding>> {
    let (observations, oracle) = validate_history_bundle(observations_json, oracle_json)?;
    let predictions: HistoryPredictionManifest = match serde_json::from_str(predictions_json) {
        Ok(value) => value,
        Err(error) => return Err(parse_error("predictions", error)),
    };
    let observation_digest = blake3::hash(observations_json.as_bytes()).to_hex().to_string();
    let oracle_digest = blake3::hash(oracle_json.as_bytes()).to_hex().to_string();
    let mut findings = Vec::new();
    if predictions.schema_version != 1 || predictions.benchmark_id != BENCHMARK_ID {
        findings.push(finding("prediction_identity_mismatch", None, "unsupported prediction schema or benchmark ID"));
    }
    if predictions.observation_blake3 != observation_digest {
        findings.push(finding("observation_digest_mismatch", None, "prediction set is not bound to the exact observation JSON bytes"));
    }

    let cases: BTreeMap<&str, &HistoryCase> = observations.cases.iter().map(|case| (case.case_id.as_str(), case)).collect();
    let labels: BTreeMap<&str, &HistoryOracleLabel> = oracle.labels.iter().map(|label| (label.case_id.as_str(), label)).collect();
    let mut seen_predictions: BTreeSet<String> = BTreeSet::new();
    for prediction in &predictions.predictions {
        if !seen_predictions.insert(prediction.case_id.clone()) {
            findings.push(finding("duplicate_prediction", Some(prediction.case_id.as_str()), "one prediction per case is required"));
        }
        if !cases.contains_key(prediction.case_id.as_str()) {
            findings.push(finding("unknown_prediction_case", Some(prediction.case_id.as_str()), "prediction case is absent from the observation manifest"));
        }
        if labels.contains_key(prediction.case_id.as_str()) {
            let shape_valid = match prediction.disposition {
                HistoryDisposition::Claim => prediction.diagnosis.is_some() && prediction.next_action.is_some() && prediction.requested_evidence.is_none(),
                HistoryDisposition::RequestMoreEvidence => prediction.diagnosis.is_none() && prediction.next_action.is_none() && prediction.requested_evidence.is_some(),
                HistoryDisposition::Abstain => prediction.diagnosis.is_none() && prediction.next_action.is_none() && prediction.requested_evidence.is_none(),
            };
            if !shape_valid {
                findings.push(finding("inconsistent_prediction_shape", Some(prediction.case_id.as_str()), "disposition-specific fields are inconsistent"));
            }
            let mut cited: BTreeSet<&str> = BTreeSet::new();
            for id in prediction.supporting_evidence_ids.iter().chain(&prediction.contradicting_evidence_ids) {
                if !cited.insert(id.as_str()) {
                    findings.push(finding("duplicate_evidence_citation", Some(prediction.case_id.as_str()), format!("evidence ID {id} is cited more than once")));
                }
            }
        }
    }
    let expected_cases: BTreeSet<String> = cases.keys().map(|case_id| (*case_id).to_owned()).collect();
    if seen_predictions != expected_cases {
        findings.push(finding("prediction_cardinality_mismatch", None, "predictions must contain exactly one row per observation case"));
    }
    if !findings.is_empty() {
        return Err(findings);
    }

    let prediction_by_case: BTreeMap<&str, &HistoryPrediction> = predictions.predictions.iter().map(|prediction| (prediction.case_id.as_str(), prediction)).collect();
    let mut acceptable = 0usize;
    let mut required_support_total = 0usize;
    let mut required_support_found = 0usize;
    let mut required_contradiction_total = 0usize;
    let mut required_contradiction_found = 0usize;
    let mut support_refs = 0usize;
    let mut grounded_support_refs = 0usize;
    let mut contradiction_refs = 0usize;
    let mut grounded_contradiction_refs = 0usize;
    let mut unsupported = 0usize;
    let mut missing_contradictions = 0usize;
    let mut critical = 0usize;

    for (case_id, case) in &cases {
        let prediction = prediction_by_case[*case_id];
        let label = labels[*case_id];
        let known: BTreeSet<&str> = case.observations.iter().map(|event| event.observation_id.as_str()).collect();
        let decision_acceptable = label.accepted_dispositions.contains(&prediction.disposition)
            && match prediction.disposition {
                HistoryDisposition::Claim => {
                    prediction.diagnosis.is_some_and(|value| label.acceptable_diagnoses.contains(&value))
                    && prediction.next_action.is_some_and(|value| label.acceptable_actions.contains(&value))
                },
                HistoryDisposition::RequestMoreEvidence => prediction.requested_evidence == label.requested_evidence,
                HistoryDisposition::Abstain => true,
            };
        if decision_acceptable { acceptable += 1; }
        if prediction.disposition == HistoryDisposition::Claim
            && !label.accepted_dispositions.contains(&HistoryDisposition::Claim)
        {
            critical += 1;
        }
        for id in &prediction.supporting_evidence_ids {
            support_refs += 1;
            if known.contains(id.as_str()) { grounded_support_refs += 1; } else { unsupported += 1; critical += 1; }
        }
        for id in &prediction.contradicting_evidence_ids {
            contradiction_refs += 1;
            if known.contains(id.as_str()) { grounded_contradiction_refs += 1; } else { unsupported += 1; critical += 1; }
        }
        for required in &label.required_supporting_evidence_ids {
            required_support_total += 1;
            if prediction.supporting_evidence_ids.contains(required) { required_support_found += 1; }
        }
        for required in &label.required_contradicting_evidence_ids {
            required_contradiction_total += 1;
            if prediction.contradicting_evidence_ids.contains(required) {
                required_contradiction_found += 1;
            } else {
                missing_contradictions += 1;
                critical += 1;
            }
        }
    }

    let ratio = |numerator: usize, denominator: usize| {
        if denominator == 0 { None } else { Some(numerator as f64 / denominator as f64) }
    };
    Ok(HistoryBenchmarkScore {
        benchmark_id: BENCHMARK_ID.to_owned(),
        observation_blake3: observation_digest,
        oracle_blake3: oracle_digest,
        case_count: cases.len(),
        acceptable_decisions: acceptable,
        acceptable_decision_rate: acceptable as f64 / cases.len() as f64,
        required_supporting_evidence_coverage: ratio(required_support_found, required_support_total),
        required_contradicting_evidence_coverage: ratio(required_contradiction_found, required_contradiction_total),
        supporting_reference_precision: ratio(grounded_support_refs, support_refs),
        contradicting_reference_precision: ratio(grounded_contradiction_refs, contradiction_refs),
        unsupported_evidence_references: unsupported,
        missing_required_contradictions: missing_contradictions,
        critical_violations: critical,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const OBSERVATIONS: &str = include_str!("../../../examples/tutor-history/v1/observations.json");
    const ORACLE: &str = include_str!("../../../examples/tutor-history/v1/oracle.json");

    fn abstaining_prediction(case_id: String) -> HistoryPrediction {
        HistoryPrediction {
            case_id,
            disposition: HistoryDisposition::Abstain,
            diagnosis: None,
            next_action: None,
            supporting_evidence_ids: vec![],
            contradicting_evidence_ids: vec![],
            requested_evidence: None,
        }
    }

    fn prediction_manifest(predictions: Vec<HistoryPrediction>) -> HistoryPredictionManifest {
        HistoryPredictionManifest {
            schema_version: 1,
            benchmark_id: BENCHMARK_ID.into(),
            observation_blake3: blake3::hash(OBSERVATIONS.as_bytes()).to_hex().to_string(),
            predictions,
        }
    }

    #[test]
    fn synthetic_bundle_has_exact_unique_case_and_event_identity() {
        let (manifest, oracle) = validate_history_bundle(OBSERVATIONS, ORACLE).expect("valid synthetic bundle");
        assert_eq!(manifest.cases.len(), 9);
        assert_eq!(oracle.labels.len(), 9);
    }

    #[test]
    fn case_order_is_not_the_sequence_semantics() {
        let mut value: serde_json::Value = serde_json::from_str(OBSERVATIONS).expect("JSON");
        let events = value["cases"][0]["observations"].as_array_mut().expect("array");
        events.swap(0, 3);
        assert!(validate_history_bundle(&value.to_string(), ORACLE).is_ok());
    }

    #[test]
    fn derived_estimates_cannot_masquerade_as_recorded_correctness() {
        let mut value: serde_json::Value = serde_json::from_str(OBSERVATIONS).expect("JSON");
        value["cases"][8]["observations"][0]["correctness"] = serde_json::Value::Bool(true);
        let error = validate_history_bundle(&value.to_string(), ORACLE).expect_err("must fail closed");
        assert!(error.iter().any(|item| item.code == "estimate_masquerades_as_observation"));
    }

    #[test]
    fn oracle_must_not_reference_an_unknown_observation() {
        let mut value: serde_json::Value = serde_json::from_str(ORACLE).expect("JSON");
        value["labels"][0]["required_supporting_evidence_ids"][0] = serde_json::Value::String("missing-event".into());
        let error = validate_history_bundle(OBSERVATIONS, &value.to_string()).expect_err("must fail closed");
        assert!(error.iter().any(|item| item.code == "oracle_reference_missing"));
    }

    #[test]
    fn exact_observation_bytes_are_bound_to_prediction_set() {
        let (manifest, _) = validate_history_bundle(OBSERVATIONS, ORACLE).expect("valid bundle");
        let predictions = manifest.cases.into_iter().map(|case| abstaining_prediction(case.case_id)).collect();
        let raw = serde_json::to_string(&prediction_manifest(predictions)).expect("serialize");
        assert!(score_history_predictions(OBSERVATIONS, ORACLE, &raw).is_ok());
        assert!(score_history_predictions(&format!("{OBSERVATIONS}\n"), ORACLE, &raw).is_err());
    }

    #[test]
    fn unsupported_evidence_is_counted_as_a_critical_violation() {
        let (manifest, _) = validate_history_bundle(OBSERVATIONS, ORACLE).expect("valid bundle");
        let predictions = manifest.cases.into_iter().map(|case| {
            let mut prediction = abstaining_prediction(case.case_id.clone());
            if case.case_id == "TH001" {
                prediction.supporting_evidence_ids.push("invented-id".into());
            }
            prediction
        }).collect();
        let raw = serde_json::to_string(&prediction_manifest(predictions)).expect("serialize");
        let score = score_history_predictions(OBSERVATIONS, ORACLE, &raw).expect("valid shape");
        assert_eq!(score.unsupported_evidence_references, 1);
        assert!(score.critical_violations >= 1);
    }

    #[test]
    fn same_surface_accuracy_with_different_assistance_has_different_labels() {
        let (_, oracle) = validate_history_bundle(OBSERVATIONS, ORACLE).expect("valid bundle");
        let labels: BTreeMap<&str, &HistoryOracleLabel> = oracle.labels.iter().map(|label| (label.case_id.as_str(), label)).collect();
        assert_eq!(labels["TH007"].acceptable_diagnoses, vec![HistoryDiagnosis::ScaffoldDependence]);
        assert_eq!(labels["TH008"].acceptable_diagnoses, vec![HistoryDiagnosis::ProvenUnassistedProgress]);
    }
}
