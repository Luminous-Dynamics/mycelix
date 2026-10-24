// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Reproducible, leakage-resistant qualification for Praxis tutoring benchmarks.
//!
//! This module treats the run record as an auditable protocol artifact. It does
//! not authenticate remote providers or prove that the candidate never saw a
//! hidden artifact: those require trusted harness isolation and signed receipts.

use super::tutor_temporal_evidence::TemporalManifest;
use serde::{de::Deserializer, Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

const INTEGRITY_PROTOCOL_ID: &str = "PRAXIS-BENCHMARK-RUN-INTEGRITY-001";
const TUTOR_BENCHMARK_ID: &str = "PRAXIS-TUTOR-TEMPORAL-EVIDENCE-001";

fn required_option<'de, D, T>(deserializer: D) -> Result<Option<T>, D::Error>
where
    D: Deserializer<'de>,
    T: Deserialize<'de>,
{
    Option::<T>::deserialize(deserializer)
}

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn is_lower_hex(value: &str, len: usize) -> bool {
    value.len() == len && value.bytes().all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn is_digest(value: &str) -> bool {
    is_lower_hex(value, 64)
}

fn is_revision(value: &str) -> bool {
    is_lower_hex(value, 40)
}

fn exact_digest(bytes: &[u8]) -> String {
    blake3::hash(bytes).to_hex().to_string()
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq, PartialOrd, Ord)]
#[serde(rename_all = "snake_case")]
pub enum SplitRole {
    Train,
    Validation,
    Test,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum SplitUnit {
    Trajectory,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct TrajectoryAssignment {
    pub trajectory_id: String,
    pub diagnostic_family_id: String,
    pub split: SplitRole,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct TrajectorySplitManifest {
    pub schema_version: u32,
    pub benchmark_id: String,
    pub split_id: String,
    pub split_unit: SplitUnit,
    /// If true, every diagnostic family must belong to exactly one split and
    /// the declared held-out families must be exactly the families in test.
    pub whole_family_holdout: bool,
    pub held_out_diagnostic_families: Vec<String>,
    pub assignments: Vec<TrajectoryAssignment>,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq, PartialOrd, Ord)]
#[serde(rename_all = "snake_case")]
pub enum ArtifactAudience {
    CandidateVisible,
    EvaluatorOnly,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq, PartialOrd, Ord)]
#[serde(rename_all = "snake_case")]
pub enum ArtifactRole {
    ObservationManifest,
    ObservationSchema,
    CandidatePolicy,
    Oracle,
    SplitAssignments,
    Scorer,
    Verifier,
    BenchmarkProtocol,
    MutationMatrix,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct RunArtifact {
    pub artifact_id: String,
    pub role: ArtifactRole,
    pub audience: ArtifactAudience,
    pub digest_blake3: String,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum AdvisorMode {
    Baseline,
    SymthaeaAssisted,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct SystemIdentity {
    pub system_id: String,
    /// Immutable system revision (e.g. Git commit SHA).
    pub system_revision: String,
    pub model_config_blake3: String,
    pub prompt_config_blake3: String,
    pub policy_gate_blake3: String,
    pub advisor_mode: AdvisorMode,
    #[serde(deserialize_with = "required_option")]
    pub advisor_artifact_blake3: Option<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct SamplingConfig {
    /// Temperature scaled by 1,000 to avoid floating-point serialization drift.
    pub temperature_milli: u32,
    /// Top-p scaled by 1,000; 1,000 means 1.0.
    pub top_p_milli: u32,
    pub max_output_tokens: u32,
    pub seed: u64,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct BenchmarkRunManifest {
    pub schema_version: u32,
    pub integrity_protocol_id: String,
    pub benchmark_id: String,
    pub run_id: String,
    pub split_id: String,
    pub split_role: SplitRole,
    pub split_manifest_blake3: String,
    /// Exact trajectory-level evaluation set for this run, not a sample of turns.
    pub trajectory_ids: Vec<String>,
    pub system: SystemIdentity,
    pub sampling: SamplingConfig,
    pub execution_environment_blake3: String,
    pub candidate_schema_blake3: String,
    pub observation_manifest_blake3: String,
    pub oracle_blake3: String,
    pub scorer_blake3: String,
    pub verifier_blake3: String,
    pub protocol_blake3: String,
    pub mutation_matrix_blake3: String,
    pub evaluator_revision: String,
    /// Digest of the canonical prediction artifact emitted by the candidate.
    pub predictions_blake3: String,
    pub candidate_visible_artifact_ids: Vec<String>,
    pub evaluator_only_artifact_ids: Vec<String>,
    pub artifacts: Vec<RunArtifact>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct IntegrityFinding {
    pub code: &'static str,
    pub detail: String,
}

fn finding(code: &'static str, detail: impl Into<String>) -> IntegrityFinding {
    IntegrityFinding { code, detail: detail.into() }
}

fn parse_error(field: &'static str, error: serde_json::Error) -> Vec<IntegrityFinding> {
    vec![finding("json_parse", format!("{field}: {error}"))]
}

/// Validate that whole trajectories, not individual turns, are the split unit.
/// When whole-family holdout is enabled, no diagnostic family may cross splits.
pub fn validate_trajectory_split(
    split_json: &str,
) -> Result<TrajectorySplitManifest, Vec<IntegrityFinding>> {
    let split: TrajectorySplitManifest = match serde_json::from_str(split_json) {
        Ok(value) => value,
        Err(error) => return Err(parse_error("split_manifest", error)),
    };
    let mut findings = Vec::new();

    if split.schema_version != 1 {
        findings.push(finding("unsupported_split_schema", "only split schema version 1 is supported"));
    }
    if split.benchmark_id != TUTOR_BENCHMARK_ID {
        findings.push(finding("split_benchmark_mismatch", "split manifest is for a different benchmark"));
    }
    if !nonempty(&split.split_id) {
        findings.push(finding("empty_split_id", "split ID must be non-empty"));
    }
    if split.assignments.is_empty() {
        findings.push(finding("empty_split_assignments", "at least one trajectory assignment is required"));
    }

    let mut trajectory_ids = BTreeSet::new();
    let mut family_roles: BTreeMap<String, SplitRole> = BTreeMap::new();
    let mut families_by_role: BTreeMap<SplitRole, BTreeSet<String>> = BTreeMap::new();
    let mut counts: BTreeMap<SplitRole, usize> = BTreeMap::new();

    for assignment in &split.assignments {
        if !nonempty(&assignment.trajectory_id) {
            findings.push(finding("empty_trajectory_id", "trajectory IDs must be non-empty"));
        }
        if !nonempty(&assignment.diagnostic_family_id) {
            findings.push(finding("empty_diagnostic_family_id", format!("trajectory {} has no diagnostic family", assignment.trajectory_id)));
        }
        if !trajectory_ids.insert(assignment.trajectory_id.clone()) {
            findings.push(finding("duplicate_trajectory_assignment", format!("trajectory {} appears more than once; turns from one trajectory must not be split independently", assignment.trajectory_id)));
        }
        *counts.entry(assignment.split).or_insert(0) += 1;
        families_by_role.entry(assignment.split).or_default().insert(assignment.diagnostic_family_id.clone());
        if let Some(previous) = family_roles.insert(assignment.diagnostic_family_id.clone(), assignment.split) {
            if previous != assignment.split && split.whole_family_holdout {
                findings.push(finding("diagnostic_family_split_leakage", format!("diagnostic family {} appears in both {:?} and {:?} splits", assignment.diagnostic_family_id, previous, assignment.split)));
            }
        }
    }

    for role in [SplitRole::Train, SplitRole::Validation, SplitRole::Test] {
        if counts.get(&role).copied().unwrap_or(0) == 0 {
            findings.push(finding("empty_required_split", format!("split {:?} must contain at least one complete trajectory", role)));
        }
    }

    let mut declared_holdout = BTreeSet::new();
    for family in &split.held_out_diagnostic_families {
        if !nonempty(family) {
            findings.push(finding("empty_held_out_family", "held-out diagnostic family IDs must be non-empty"));
        }
        if !declared_holdout.insert(family.clone()) {
            findings.push(finding("duplicate_held_out_family", format!("held-out diagnostic family {family} appears more than once")));
        }
    }

    let test_families = families_by_role.get(&SplitRole::Test).cloned().unwrap_or_default();
    if split.whole_family_holdout {
        if declared_holdout != test_families {
            findings.push(finding("held_out_family_declaration_mismatch", "declared held-out families must equal the set of families assigned to test"));
        }
        for family in &declared_holdout {
            if families_by_role.get(&SplitRole::Train).is_some_and(|families| families.contains(family))
                || families_by_role.get(&SplitRole::Validation).is_some_and(|families| families.contains(family))
            {
                findings.push(finding("held_out_family_seen_before_test", format!("held-out family {family} occurs outside test")));
            }
        }
    } else if !declared_holdout.is_empty() {
        findings.push(finding("holdout_declared_when_disabled", "held-out family list must be empty when whole-family holdout is disabled"));
    }

    if findings.is_empty() { Ok(split) } else { Err(findings) }
}

fn expected_audience(role: ArtifactRole) -> ArtifactAudience {
    match role {
        ArtifactRole::ObservationManifest | ArtifactRole::ObservationSchema | ArtifactRole::CandidatePolicy => ArtifactAudience::CandidateVisible,
        ArtifactRole::Oracle | ArtifactRole::SplitAssignments | ArtifactRole::Scorer | ArtifactRole::Verifier | ArtifactRole::BenchmarkProtocol | ArtifactRole::MutationMatrix => ArtifactAudience::EvaluatorOnly,
    }
}

fn digest_for_role(run: &BenchmarkRunManifest, role: ArtifactRole) -> &str {
    match role {
        ArtifactRole::ObservationManifest => &run.observation_manifest_blake3,
        ArtifactRole::ObservationSchema => &run.candidate_schema_blake3,
        ArtifactRole::CandidatePolicy => &run.system.policy_gate_blake3,
        ArtifactRole::Oracle => &run.oracle_blake3,
        ArtifactRole::SplitAssignments => &run.split_manifest_blake3,
        ArtifactRole::Scorer => &run.scorer_blake3,
        ArtifactRole::Verifier => &run.verifier_blake3,
        ArtifactRole::BenchmarkProtocol => &run.protocol_blake3,
        ArtifactRole::MutationMatrix => &run.mutation_matrix_blake3,
    }
}

/// Validate a run against the exact split and candidate-visible observation bytes.
/// The supplied candidate dataset must contain only the complete trajectories for
/// the declared split role. This checks the local artifact boundary, but it cannot
/// prove a remote model never received hidden content outside this harness.
pub fn validate_benchmark_run(
    run_json: &str,
    split_json: &str,
    candidate_observations_json: &str,
) -> Result<(BenchmarkRunManifest, TrajectorySplitManifest), Vec<IntegrityFinding>> {
    let mut findings = Vec::new();
    let run: BenchmarkRunManifest = match serde_json::from_str(run_json) {
        Ok(value) => value,
        Err(error) => return Err(parse_error("run_manifest", error)),
    };
    let split = match validate_trajectory_split(split_json) {
        Ok(value) => value,
        Err(errors) => return Err(errors),
    };

    if run.schema_version != 1 || run.integrity_protocol_id != INTEGRITY_PROTOCOL_ID {
        findings.push(finding("unsupported_run_protocol", "run manifest protocol/schema is unsupported"));
    }
    if run.benchmark_id != TUTOR_BENCHMARK_ID || split.benchmark_id != run.benchmark_id {
        findings.push(finding("run_benchmark_mismatch", "run and split must target the tutor temporal-evidence benchmark"));
    }
    if !nonempty(&run.run_id) || !nonempty(&run.split_id) || run.split_id != split.split_id {
        findings.push(finding("run_split_identity_mismatch", "run ID and split ID must be non-empty and split IDs must match"));
    }
    if run.split_manifest_blake3 != exact_digest(split_json.as_bytes()) {
        findings.push(finding("split_digest_mismatch", "run is not bound to the exact split-manifest bytes"));
    }

    for (name, digest) in [
        ("execution_environment_blake3", run.execution_environment_blake3.as_str()),
        ("candidate_schema_blake3", run.candidate_schema_blake3.as_str()),
        ("observation_manifest_blake3", run.observation_manifest_blake3.as_str()),
        ("oracle_blake3", run.oracle_blake3.as_str()),
        ("scorer_blake3", run.scorer_blake3.as_str()),
        ("verifier_blake3", run.verifier_blake3.as_str()),
        ("protocol_blake3", run.protocol_blake3.as_str()),
        ("mutation_matrix_blake3", run.mutation_matrix_blake3.as_str()),
        ("system.model_config_blake3", run.system.model_config_blake3.as_str()),
        ("system.prompt_config_blake3", run.system.prompt_config_blake3.as_str()),
        ("system.policy_gate_blake3", run.system.policy_gate_blake3.as_str()),
        ("predictions_blake3", run.predictions_blake3.as_str()),
    ] {
        if !is_digest(digest) {
            findings.push(finding("invalid_artifact_digest", format!("{name} must be 64 lowercase hex characters")));
        }
    }
    if !is_revision(&run.system.system_revision) || !is_revision(&run.evaluator_revision) {
        findings.push(finding("invalid_revision", "system and evaluator revisions must be full 40-character lowercase Git SHAs"));
    }
    if !nonempty(&run.system.system_id) {
        findings.push(finding("empty_system_id", "system ID must be non-empty"));
    }
    if !(1..=1000).contains(&run.sampling.top_p_milli) || run.sampling.max_output_tokens == 0 {
        findings.push(finding("invalid_sampling_config", "top-p must be 0.001..1.000 and max output tokens must be positive"));
    }
    if run.sampling.temperature_milli > 5000 {
        findings.push(finding("invalid_sampling_config", "temperature exceeds the protocol's 5.0 limit"));
    }
    match (run.system.advisor_mode, run.system.advisor_artifact_blake3.as_deref()) {
        (AdvisorMode::Baseline, None) => {}
        (AdvisorMode::Baseline, Some(_)) => findings.push(finding("baseline_has_advisor_artifact", "baseline runs must not include an advisor artifact")),
        (AdvisorMode::SymthaeaAssisted, Some(digest)) if is_digest(digest) => {}
        (AdvisorMode::SymthaeaAssisted, _) => findings.push(finding("missing_advisor_artifact", "Symthaea-assisted runs require a pinned advisor artifact digest")),
    }

    let assignments_for_role: BTreeSet<String> = split.assignments.iter()
        .filter(|assignment| assignment.split == run.split_role)
        .map(|assignment| assignment.trajectory_id.clone())
        .collect();
    let mut trajectory_ids = BTreeSet::new();
    for trajectory_id in &run.trajectory_ids {
        if !nonempty(trajectory_id) || !trajectory_ids.insert(trajectory_id.clone()) {
            findings.push(finding("duplicate_or_empty_run_trajectory", "run trajectory IDs must be non-empty and unique"));
        }
    }
    if trajectory_ids != assignments_for_role {
        findings.push(finding("run_trajectory_set_mismatch", "run must cover exactly the complete trajectories assigned to its declared split role"));
    }

    let candidate_observations: TemporalManifest = match serde_json::from_str(candidate_observations_json) {
        Ok(value) => value,
        Err(error) => return Err(parse_error("candidate_observations", error)),
    };
    if candidate_observations.schema_version != 1 || candidate_observations.benchmark_id != TUTOR_BENCHMARK_ID {
        findings.push(finding("candidate_observation_identity_mismatch", "candidate observation manifest has an unsupported schema or benchmark ID"));
    }
    let mut candidate_trajectory_ids = BTreeSet::new();
    for case in &candidate_observations.cases {
        if !nonempty(&case.case_id) || !candidate_trajectory_ids.insert(case.case_id.clone()) {
            findings.push(finding("duplicate_or_empty_candidate_trajectory", "candidate observation case IDs must be non-empty and unique"));
        }
    }
    if candidate_trajectory_ids != assignments_for_role {
        let all_assigned: BTreeSet<String> = split.assignments.iter().map(|assignment| assignment.trajectory_id.clone()).collect();
        if candidate_trajectory_ids.iter().any(|id| all_assigned.contains(id) && !assignments_for_role.contains(id)) {
            findings.push(finding("candidate_split_leakage", "candidate-visible observations include trajectories assigned to a different split role"));
        } else {
            findings.push(finding("candidate_trajectory_set_mismatch", "candidate-visible observations must contain exactly the full trajectories assigned to this split role"));
        }
    }
    if run.observation_manifest_blake3 != exact_digest(candidate_observations_json.as_bytes()) {
        findings.push(finding("observation_manifest_digest_mismatch", "run is not bound to the exact candidate-visible observation-manifest bytes"));
    }

    let mut artifacts_by_id: BTreeMap<String, &RunArtifact> = BTreeMap::new();
    let mut roles = BTreeSet::new();
    let mut candidate_ids = BTreeSet::new();
    let mut evaluator_ids = BTreeSet::new();
    for artifact in &run.artifacts {
        if !nonempty(&artifact.artifact_id) {
            findings.push(finding("empty_artifact_id", "artifact IDs must be non-empty"));
        }
        if artifacts_by_id.insert(artifact.artifact_id.clone(), artifact).is_some() {
            findings.push(finding("duplicate_artifact_id", format!("artifact ID {} is duplicated", artifact.artifact_id)));
        }
        if !roles.insert(artifact.role) {
            findings.push(finding("duplicate_artifact_role", format!("artifact role {:?} appears more than once", artifact.role)));
        }
        if !is_digest(&artifact.digest_blake3) {
            findings.push(finding("invalid_catalog_artifact_digest", format!("artifact {} has an invalid digest", artifact.artifact_id)));
        }
        if artifact.audience != expected_audience(artifact.role) {
            findings.push(finding("artifact_audience_role_mismatch", format!("artifact {} has an unsafe audience for role {:?}", artifact.artifact_id, artifact.role)));
        }
        match artifact.audience {
            ArtifactAudience::CandidateVisible => { candidate_ids.insert(artifact.artifact_id.clone()); }
            ArtifactAudience::EvaluatorOnly => { evaluator_ids.insert(artifact.artifact_id.clone()); }
        }
        if artifact.digest_blake3 != digest_for_role(&run, artifact.role) {
            findings.push(finding("artifact_digest_binding_mismatch", format!("artifact {} digest does not match the run's bound digest for {:?}", artifact.artifact_id, artifact.role)));
        }
    }

    let required_roles = [
        ArtifactRole::ObservationManifest,
        ArtifactRole::ObservationSchema,
        ArtifactRole::CandidatePolicy,
        ArtifactRole::Oracle,
        ArtifactRole::SplitAssignments,
        ArtifactRole::Scorer,
        ArtifactRole::Verifier,
        ArtifactRole::BenchmarkProtocol,
        ArtifactRole::MutationMatrix,
    ].into_iter().collect::<BTreeSet<_>>();
    if roles != required_roles {
        findings.push(finding("artifact_role_set_mismatch", "run artifact catalog must contain each required role exactly once"));
    }
    let declared_candidate: BTreeSet<String> = run.candidate_visible_artifact_ids.iter().cloned().collect();
    let declared_evaluator: BTreeSet<String> = run.evaluator_only_artifact_ids.iter().cloned().collect();
    if declared_candidate.len() != run.candidate_visible_artifact_ids.len()
        || declared_candidate != candidate_ids
    {
        findings.push(finding("candidate_artifact_set_mismatch", "candidate-visible artifact IDs must exactly match the candidate-visible catalog and contain no duplicates"));
    }
    if declared_evaluator.len() != run.evaluator_only_artifact_ids.len()
        || declared_evaluator != evaluator_ids
    {
        findings.push(finding("evaluator_artifact_set_mismatch", "evaluator-only artifact IDs must exactly match the evaluator-only catalog and contain no duplicates"));
    }
    for forbidden in [ArtifactRole::Oracle, ArtifactRole::SplitAssignments, ArtifactRole::Scorer, ArtifactRole::Verifier, ArtifactRole::BenchmarkProtocol, ArtifactRole::MutationMatrix] {
        if run.candidate_visible_artifact_ids.iter().any(|id| artifacts_by_id.get(id).is_some_and(|artifact| artifact.role == forbidden)) {
            findings.push(finding("evaluator_artifact_exposed_to_candidate", format!("evaluator-only artifact role {:?} is included in candidate-visible IDs", forbidden)));
        }
    }

    if findings.is_empty() { Ok((run, split)) } else { Err(findings) }
}

/// Ensure B0 and B1 are a paired experiment over the same examples, oracle,
/// verifier, policy gates, evaluator, sampling budget, and split. Only system
/// identity and the pinned Symthaea advisor addition may differ.
pub fn validate_comparable_pair(
    baseline_json: &str,
    augmented_json: &str,
    split_json: &str,
    candidate_observations_json: &str,
) -> Result<(BenchmarkRunManifest, BenchmarkRunManifest), Vec<IntegrityFinding>> {
    let (baseline, _) = validate_benchmark_run(baseline_json, split_json, candidate_observations_json)?;
    let (augmented, _) = validate_benchmark_run(augmented_json, split_json, candidate_observations_json)?;
    let mut findings = Vec::new();

    if baseline.run_id == augmented.run_id {
        findings.push(finding("paired_run_id_collision", "baseline and augmented runs must have distinct run IDs"));
    }
    if baseline.split_role != SplitRole::Test || augmented.split_role != SplitRole::Test {
        findings.push(finding("paired_run_not_test_split", "comparative claims must use the held-out test split"));
    }
    if baseline.system.advisor_mode != AdvisorMode::Baseline
        || augmented.system.advisor_mode != AdvisorMode::SymthaeaAssisted
    {
        findings.push(finding("paired_advisor_mode_mismatch", "pair must compare baseline B0 against Symthaea-assisted B1"));
    }
    if baseline.split_id != augmented.split_id
        || baseline.split_manifest_blake3 != augmented.split_manifest_blake3
        || baseline.trajectory_ids.iter().collect::<BTreeSet<_>>() != augmented.trajectory_ids.iter().collect::<BTreeSet<_>>()
        || baseline.observation_manifest_blake3 != augmented.observation_manifest_blake3
        || baseline.candidate_schema_blake3 != augmented.candidate_schema_blake3
        || baseline.oracle_blake3 != augmented.oracle_blake3
        || baseline.scorer_blake3 != augmented.scorer_blake3
        || baseline.verifier_blake3 != augmented.verifier_blake3
        || baseline.protocol_blake3 != augmented.protocol_blake3
        || baseline.mutation_matrix_blake3 != augmented.mutation_matrix_blake3
        || baseline.system.system_id != augmented.system.system_id
        || baseline.system.system_revision != augmented.system.system_revision
        || baseline.system.model_config_blake3 != augmented.system.model_config_blake3
        || baseline.system.prompt_config_blake3 != augmented.system.prompt_config_blake3
        || baseline.system.policy_gate_blake3 != augmented.system.policy_gate_blake3
        || baseline.evaluator_revision != augmented.evaluator_revision
        || baseline.sampling != augmented.sampling
        || baseline.execution_environment_blake3 != augmented.execution_environment_blake3
    {
        findings.push(finding("paired_run_protocol_drift", "B0/B1 must share the exact split, trajectories, data, oracle, scorer, verifier, policy gates, evaluation protocol, sampling config, and environment"));
    }
    if baseline.system.advisor_artifact_blake3.is_some() || augmented.system.advisor_artifact_blake3.is_none() {
        findings.push(finding("paired_advisor_artifact_invalid", "B0 must have no advisor digest and B1 must pin one"));
    }

    if findings.is_empty() { Ok((baseline, augmented)) } else { Err(findings) }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SPLIT: &str = include_str!("../examples/tutor-history/temporal-v1/split_manifest.json");
    const OBSERVATIONS: &str = include_str!("../examples/tutor-history/temporal-v1/observations.json");

    fn digest(char_: char) -> String {
        std::iter::repeat(char_).take(64).collect()
    }

    fn revision(char_: char) -> String {
        std::iter::repeat(char_).take(40).collect()
    }

    fn candidate_dataset_for_role(split_json: &str, role: SplitRole) -> String {
        let split: TrajectorySplitManifest = serde_json::from_str(split_json).unwrap();
        let wanted: BTreeSet<String> = split.assignments.iter()
            .filter(|assignment| assignment.split == role)
            .map(|assignment| assignment.trajectory_id.clone())
            .collect();
        let mut dataset: serde_json::Value = serde_json::from_str(OBSERVATIONS).unwrap();
        dataset["cases"].as_array_mut().unwrap().retain(|case| {
            case["case_id"].as_str().is_some_and(|id| wanted.contains(id))
        });
        serde_json::to_string_pretty(&dataset).unwrap()
    }

    fn run_manifest(split_json: &str, mode: AdvisorMode, run_id: &str) -> BenchmarkRunManifest {
        let candidate_observations_json = candidate_dataset_for_role(split_json, SplitRole::Test);
        let split: TrajectorySplitManifest = serde_json::from_str(split_json).unwrap();
        let split_digest = exact_digest(split_json.as_bytes());
        let observation_digest = exact_digest(candidate_observations_json.as_bytes());
        let role = SplitRole::Test;
        let trajectory_ids = split.assignments.iter()
            .filter(|assignment| assignment.split == role)
            .map(|assignment| assignment.trajectory_id.clone())
            .collect();
        let system = SystemIdentity {
            system_id: "praxis-tutor".to_owned(),
            system_revision: revision('1'),
            model_config_blake3: digest('a'),
            prompt_config_blake3: digest('b'),
            policy_gate_blake3: digest('d'),
            advisor_mode: mode,
            advisor_artifact_blake3: match mode { AdvisorMode::Baseline => None, AdvisorMode::SymthaeaAssisted => Some(digest('e')) },
        };
        let artifact_defs = [
            ("observations", ArtifactRole::ObservationManifest, ArtifactAudience::CandidateVisible, observation_digest.clone()),
            ("observation-schema", ArtifactRole::ObservationSchema, ArtifactAudience::CandidateVisible, digest('2')),
            ("candidate-policy", ArtifactRole::CandidatePolicy, ArtifactAudience::CandidateVisible, digest('d')),
            ("oracle", ArtifactRole::Oracle, ArtifactAudience::EvaluatorOnly, digest('3')),
            ("split-assignments", ArtifactRole::SplitAssignments, ArtifactAudience::EvaluatorOnly, split_digest.clone()),
            ("scorer", ArtifactRole::Scorer, ArtifactAudience::EvaluatorOnly, digest('4')),
            ("verifier", ArtifactRole::Verifier, ArtifactAudience::EvaluatorOnly, digest('5')),
            ("protocol", ArtifactRole::BenchmarkProtocol, ArtifactAudience::EvaluatorOnly, digest('6')),
            ("mutation-matrix", ArtifactRole::MutationMatrix, ArtifactAudience::EvaluatorOnly, digest('7')),
        ];
        let artifacts = artifact_defs.iter().map(|(id, role, audience, digest)| RunArtifact {
            artifact_id: (*id).to_owned(), role: *role, audience: *audience, digest_blake3: (*digest).clone()
        }).collect();
        BenchmarkRunManifest {
            schema_version: 1,
            integrity_protocol_id: INTEGRITY_PROTOCOL_ID.to_owned(),
            benchmark_id: TUTOR_BENCHMARK_ID.to_owned(),
            run_id: run_id.to_owned(),
            split_id: split.split_id.clone(),
            split_role: role,
            split_manifest_blake3: split_digest,
            trajectory_ids,
            system,
            sampling: SamplingConfig { temperature_milli: 0, top_p_milli: 1000, max_output_tokens: 2048, seed: 7 },
            execution_environment_blake3: digest('8'),
            candidate_schema_blake3: digest('2'),
            observation_manifest_blake3: observation_digest,
            oracle_blake3: digest('3'),
            scorer_blake3: digest('4'),
            verifier_blake3: digest('5'),
            protocol_blake3: digest('6'),
            mutation_matrix_blake3: digest('7'),
            evaluator_revision: revision('9'),
            predictions_blake3: digest('f'),
            candidate_visible_artifact_ids: vec!["observations".to_owned(), "observation-schema".to_owned(), "candidate-policy".to_owned()],
            evaluator_only_artifact_ids: vec!["oracle".to_owned(), "split-assignments".to_owned(), "scorer".to_owned(), "verifier".to_owned(), "protocol".to_owned(), "mutation-matrix".to_owned()],
            artifacts,
        }
    }

    #[test]
    fn split_fixture_has_whole_trajectory_and_family_holdout() {
        let split = validate_trajectory_split(SPLIT).expect("valid split fixture");
        assert_eq!(split.assignments.len(), 9);
        assert!(split.whole_family_holdout);
    }

    #[test]
    fn duplicate_trajectory_assignments_are_rejected() {
        let mut value: serde_json::Value = serde_json::from_str(SPLIT).unwrap();
        let first = value["assignments"][0].clone();
        value["assignments"].as_array_mut().unwrap().push(first);
        let errors = validate_trajectory_split(&value.to_string()).expect_err("duplicate trajectory must fail");
        assert!(errors.iter().any(|error| error.code == "duplicate_trajectory_assignment"));
    }

    #[test]
    fn diagnostic_family_cannot_cross_train_validation_and_test() {
        let mut value: serde_json::Value = serde_json::from_str(SPLIT).unwrap();
        let test_family = value["assignments"].as_array().unwrap().iter()
            .find(|entry| entry["split"] == "test").unwrap()["diagnostic_family_id"].clone();
        value["assignments"].as_array_mut().unwrap().iter_mut()
            .find(|entry| entry["split"] == "train").unwrap()["diagnostic_family_id"] = test_family;
        let errors = validate_trajectory_split(&value.to_string()).expect_err("family leakage must fail");
        assert!(errors.iter().any(|error| error.code == "diagnostic_family_split_leakage"));
    }

    #[test]
    fn held_out_family_declaration_must_match_test_families() {
        let mut value: serde_json::Value = serde_json::from_str(SPLIT).unwrap();
        value["held_out_diagnostic_families"].as_array_mut().unwrap().push("seen_in_train".into());
        let errors = validate_trajectory_split(&value.to_string()).expect_err("bad holdout manifest must fail");
        assert!(errors.iter().any(|error| error.code == "held_out_family_declaration_mismatch"));
    }

    #[test]
    fn valid_run_binds_all_required_artifact_roles_and_exact_split_bytes() {
        let run = run_manifest(SPLIT, AdvisorMode::Baseline, "b0-test-run");
        let run_json = serde_json::to_string(&run).unwrap();
        let (validated, split) = validate_benchmark_run(&run_json, SPLIT, &candidate_dataset_for_role(SPLIT, SplitRole::Test)).expect("valid run");
        assert_eq!(validated.trajectory_ids.len(), 3);
        assert_eq!(split.split_id, "praxis-temporal-synthetic-v1");
    }

    #[test]
    fn changed_split_bytes_invalidate_run_binding() {
        let run = run_manifest(SPLIT, AdvisorMode::Baseline, "b0-test-run");
        let run_json = serde_json::to_string(&run).unwrap();
        let changed_split = format!("\n{SPLIT}");
        let errors = validate_benchmark_run(&run_json, &changed_split, &candidate_dataset_for_role(SPLIT, SplitRole::Test)).expect_err("exact byte change must invalidate split binding");
        assert!(errors.iter().any(|error| error.code == "split_digest_mismatch"));
    }

    #[test]
    fn candidate_observation_digest_binds_exact_split_subset_bytes() {
        let candidate_data = candidate_dataset_for_role(SPLIT, SplitRole::Test);
        let run = run_manifest(SPLIT, AdvisorMode::Baseline, "b0-test-run");
        let result = validate_benchmark_run(&serde_json::to_string(&run).unwrap(), SPLIT, &candidate_data)
            .expect("split-only candidate dataset is correctly bound");
        assert_eq!(result.0.trajectory_ids.len(), 3);
    }

    #[test]
    fn candidate_receiving_all_trajectories_is_rejected_as_split_leakage() {
        let run = run_manifest(SPLIT, AdvisorMode::Baseline, "b0-test-run");
        let errors = validate_benchmark_run(
            &serde_json::to_string(&run).unwrap(),
            SPLIT,
            OBSERVATIONS,
        ).expect_err("full manifest must not be visible to the test-only candidate run");
        assert!(errors.iter().any(|error| error.code == "candidate_split_leakage"));
        assert!(errors.iter().any(|error| error.code == "observation_manifest_digest_mismatch"));
    }

    #[test]
    fn evaluator_only_oracle_cannot_be_added_to_candidate_inputs() {
        let mut run = run_manifest(SPLIT, AdvisorMode::Baseline, "b0-test-run");
        run.candidate_visible_artifact_ids.push("oracle".to_owned());
        let errors = validate_benchmark_run(&serde_json::to_string(&run).unwrap(), SPLIT, &candidate_dataset_for_role(SPLIT, SplitRole::Test))
            .expect_err("oracle cannot be candidate visible");
        assert!(errors.iter().any(|error| error.code == "candidate_artifact_set_mismatch"));
    }

    #[test]
    fn b0_b1_pair_is_comparable_only_under_same_protocol() {
        let baseline = run_manifest(SPLIT, AdvisorMode::Baseline, "b0-test-run");
        let augmented = run_manifest(SPLIT, AdvisorMode::SymthaeaAssisted, "b1-test-run");
        let result = validate_comparable_pair(
            &serde_json::to_string(&baseline).unwrap(),
            &serde_json::to_string(&augmented).unwrap(),
            SPLIT,
            &candidate_dataset_for_role(SPLIT, SplitRole::Test),
        ).expect("valid paired test runs");
        assert_eq!(result.0.system.advisor_mode, AdvisorMode::Baseline);
        assert_eq!(result.1.system.advisor_mode, AdvisorMode::SymthaeaAssisted);
    }

    #[test]
    fn b0_b1_policy_gate_drift_is_rejected() {
        let baseline = run_manifest(SPLIT, AdvisorMode::Baseline, "b0-test-run");
        let mut augmented = run_manifest(SPLIT, AdvisorMode::SymthaeaAssisted, "b1-test-run");
        augmented.system.policy_gate_blake3 = digest('0');
        augmented.artifacts.iter_mut().find(|artifact| artifact.role == ArtifactRole::CandidatePolicy).unwrap().digest_blake3 = digest('0');
        let errors = validate_comparable_pair(
            &serde_json::to_string(&baseline).unwrap(),
            &serde_json::to_string(&augmented).unwrap(),
            SPLIT,
            &candidate_dataset_for_role(SPLIT, SplitRole::Test),
        ).expect_err("policy gate drift must fail");
        assert!(errors.iter().any(|error| error.code == "paired_run_protocol_drift"));
    }

    #[test]
    fn b0_b1_different_test_trajectories_are_rejected() {
        let baseline = run_manifest(SPLIT, AdvisorMode::Baseline, "b0-test-run");
        let mut augmented = run_manifest(SPLIT, AdvisorMode::SymthaeaAssisted, "b1-test-run");
        augmented.trajectory_ids.pop();
        let errors = validate_comparable_pair(
            &serde_json::to_string(&baseline).unwrap(),
            &serde_json::to_string(&augmented).unwrap(),
            SPLIT,
            &candidate_dataset_for_role(SPLIT, SplitRole::Test),
        ).expect_err("paired runs must use identical test trajectories");
        assert!(errors.iter().any(|error| error.code == "run_trajectory_set_mismatch"));
    }

    #[test]
    fn b0_b1_model_prompt_and_revision_drift_are_rejected() {
        let baseline = run_manifest(SPLIT, AdvisorMode::Baseline, "b0-test-run");
        let mut augmented = run_manifest(SPLIT, AdvisorMode::SymthaeaAssisted, "b1-test-run");
        augmented.system.prompt_config_blake3 = digest('c');
        let errors = validate_comparable_pair(
            &serde_json::to_string(&baseline).unwrap(),
            &serde_json::to_string(&augmented).unwrap(),
            SPLIT,
            &candidate_dataset_for_role(SPLIT, SplitRole::Test),
        ).expect_err("prompt drift must fail");
        assert!(errors.iter().any(|error| error.code == "paired_run_protocol_drift"));

        augmented = run_manifest(SPLIT, AdvisorMode::SymthaeaAssisted, "b1-test-run");
        augmented.system.system_revision = revision('2');
        let errors = validate_comparable_pair(
            &serde_json::to_string(&baseline).unwrap(),
            &serde_json::to_string(&augmented).unwrap(),
            SPLIT,
            &candidate_dataset_for_role(SPLIT, SplitRole::Test),
        ).expect_err("system revision drift must fail");
        assert!(errors.iter().any(|error| error.code == "paired_run_protocol_drift"));
    }

    #[test]
    fn mutation_matrix_is_closed_and_test_linked() {
        let matrix: serde_json::Value = serde_json::from_str(include_str!("../examples/tutor-history/temporal-v1/run_integrity_mutation_matrix.json")).unwrap();
        let entries = matrix["entries"].as_array().unwrap();
        let source = include_str!("tutor_benchmark_integrity.rs");
        let mut ids = BTreeSet::new();
        for entry in entries {
            let id = entry["id"].as_str().unwrap();
            assert!(ids.insert(id.to_owned()), "duplicate mutation ID {id}");
            let test_name = entry["test_name"].as_str().unwrap();
            assert!(source.contains(&format!("fn {test_name}(")), "matrix row {id} points to missing test {test_name}");
        }
        assert_eq!(entries.len(), 14);
    }
}
