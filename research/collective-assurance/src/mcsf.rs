//! Memory Capability–Sovereignty Frontier (MCSF) matched-run qualification.
//!
//! MCSF reports a non-compensatory vector. It deliberately does not compute a
//! single weighted trust/quality score. A run can have useful task scores while
//! separately failing privacy, revocation, causal-evidence or cost qualification.

use std::collections::{BTreeMap, BTreeSet};

use sha2::{Digest, Sha256};

use super::{QualificationReport, QualificationStatus};

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty() && value == value.trim()
}

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn valid_commit(value: &str) -> bool {
    value.len() == 40
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn push_len(bytes: &mut Vec<u8>, value: usize) {
    bytes.extend_from_slice(&(value as u64).to_be_bytes());
}

fn push_str(bytes: &mut Vec<u8>, value: &str) {
    push_len(bytes, value.len());
    bytes.extend_from_slice(value.as_bytes());
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write;
        let _ = write!(&mut output, "{byte:02x}");
    }
    output
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum McsfTrack {
    Textual,
    Coding,
    Multimodal,
    GovernanceOnly,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum McsfArchitecture {
    NoMemory,
    Bm25Sparse,
    DenseVector,
    Hdc,
    HdcCfc,
    HybridSparseDenseHdcCfc,
    GraphRelational,
    GovernedMycelix,
    CoordinatedMycelix,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum McsfPolicyClass {
    /// Native architecture research unconstrained by a third-party division policy.
    NativeUnrestricted,
    /// A separately evidenced official release/division-compatible run.
    OfficialDivisionCompatible,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum McsfTaskState {
    Evaluated,
    InfrastructureError,
    NotRun,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct McsfSuiteManifest {
    pub suite_id: String,
    pub track: McsfTrack,
    /// Digest of the exact task IDs, task-family labels and source task versions.
    pub task_manifest_digest: String,
    /// Digest of paired-world construction, principals, policy profiles and seeds.
    pub scenario_manifest_digest: String,
    pub evaluator_digest: String,
    pub answer_model_digest: String,
    pub prompt_config_digest: String,
    pub toolset_digest: String,
    pub hardware_profile_digest: String,
    /// Digest of common per-run time/token/compute/storage budget definitions.
    pub budget_profile_digest: String,
    pub seed: u64,
    pub retrieval_top_k: u32,
    pub context_window_tokens: u64,
    pub per_task_token_budget: u64,
    pub per_task_wall_clock_budget_ms: u64,
    pub per_task_compute_budget_millijoules: u64,
    pub expected_task_ids: Vec<String>,
    pub expected_architectures: Vec<McsfArchitecture>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct McsfArchitectureManifest {
    pub architecture: McsfArchitecture,
    pub policy_class: McsfPolicyClass,
    pub system_id: String,
    pub system_version: String,
    pub system_commit: String,
    pub system_artifact_digest: String,
    /// Exact representation/model configuration, including HDC dimension or
    /// dense embedding identity as appropriate.
    pub memory_representation_digest: String,
    pub index_update_policy_digest: String,
    pub architecture_config_digest: String,
    /// Required when the run claims a third-party official division policy.
    pub official_policy_receipt_digest: Option<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct McsfEvidenceReferences {
    pub privacy_trace_report_digest: Option<String>,
    pub revocation_report_digest: Option<String>,
    pub model_input_witness_report_digest: Option<String>,
    pub role_enforcement_report_digest: Option<String>,
    pub causal_replay_report_digest: Option<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct McsfResourceUsage {
    pub input_tokens: u64,
    pub output_tokens: u64,
    pub memory_storage_bytes: u64,
    pub index_write_time_ms: u64,
    pub search_time_ms: u64,
    pub wall_clock_ms: u64,
    pub energy_millijoules: Option<u64>,
    pub provider_cost_micro_usd: Option<u64>,
    pub evidence_digest: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct McsfTaskObservation {
    pub task_id: String,
    pub task_family: String,
    pub state: McsfTaskState,
    /// Native track evaluator score, fixed-point [0, 1] millionths.
    pub utility_score_micros: Option<u32>,
    pub authorized_task_pass: Option<bool>,
    /// Count of confirmed protected payload disclosures in this task trial.
    pub unauthorized_content_disclosures: u32,
    /// Count of confirmed protected-state inferences through allowed metadata.
    pub forbidden_metadata_inferences: u32,
    pub revocation_expected_descendants: Option<u32>,
    pub revocation_reconciled_descendants: Option<u32>,
    pub revocation_unknown_descendants: u32,
    pub retrieved_memory_count: u32,
    pub model_input_consumed_count: u32,
    pub decision_influential_count: u32,
    /// Commit to the task's independently captured causal/counterfactual witness.
    pub causal_evidence_digest: Option<String>,
    pub resource_usage: Option<McsfResourceUsage>,
    pub evidence_digest: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct McsfRun {
    pub run_id: String,
    pub control_digest: String,
    pub architecture: McsfArchitectureManifest,
    pub evidence_references: McsfEvidenceReferences,
    pub task_observations: Vec<McsfTaskObservation>,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct McsfResourceTotals {
    pub task_count: usize,
    pub input_tokens: u64,
    pub output_tokens: u64,
    pub memory_storage_bytes: u64,
    pub index_write_time_ms: u64,
    pub search_time_ms: u64,
    pub wall_clock_ms: u64,
    pub energy_millijoules: Option<u64>,
    pub provider_cost_micro_usd: Option<u64>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct McsfArchitectureScorecard {
    pub architecture: McsfArchitecture,
    pub policy_class: McsfPolicyClass,
    pub utility_task_count: usize,
    pub mean_utility_score_micros: Option<u32>,
    /// Mean native-track utility by declared task family; never collapsed into
    /// the overall mean for regression detection.
    pub utility_by_task_family_micros: BTreeMap<String, u32>,
    /// Paired utility deltas against NoMemory, stratified by the same task families.
    pub utility_delta_by_task_family_vs_no_memory_micros: BTreeMap<String, i64>,
    /// Deterministic paired bootstrap percentile interval (2.5%, 97.5%).
    /// None when fewer than two paired tasks are available.
    pub mean_utility_delta_95ci_vs_no_memory_micros: Option<(i64, i64)>,
    pub utility_delta_95ci_by_task_family_vs_no_memory_micros:
        BTreeMap<String, (i64, i64)>,
    pub authorized_task_success_count: usize,
    pub unauthorized_content_disclosures: u64,
    pub forbidden_metadata_inferences: u64,
    pub privacy_evidence_complete: bool,
    pub revocation_evidence_complete: bool,
    pub revocation_expected_descendants: u64,
    pub revocation_reconciled_descendants: u64,
    pub revocation_unknown_descendants: u64,
    pub causal_evidence_complete: bool,
    pub retrieved_memory_count: u64,
    pub model_input_consumed_count: u64,
    pub decision_influential_count: u64,
    pub resource_usage_complete: bool,
    pub resource_totals: Option<McsfResourceTotals>,
    pub role_evidence_complete: bool,
    pub policy_class_eligible: bool,
    pub paired_task_count_vs_no_memory: usize,
    /// Positive means higher native evaluator utility than the matched NoMemory run.
    pub mean_utility_delta_vs_no_memory_micros: Option<i64>,
    /// True only when every non-compensatory axis is evidenced and no critical
    /// privacy or revocation failure is observed. This is not a combined score.
    pub pareto_comparison_eligible: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct McsfQualificationReport {
    pub qualification: QualificationReport,
    pub matched_control_digest: String,
    pub expected_task_count: usize,
    pub expected_architecture_count: usize,
    pub observed_architecture_count: usize,
    pub scorecards: Vec<McsfArchitectureScorecard>,
    pub reasons: Vec<String>,
}

pub fn compute_mcsf_control_digest(suite: &McsfSuiteManifest) -> Result<String, String> {
    if !nonempty(&suite.suite_id)
        || !valid_digest(&suite.task_manifest_digest)
        || !valid_digest(&suite.scenario_manifest_digest)
        || !valid_digest(&suite.evaluator_digest)
        || !valid_digest(&suite.answer_model_digest)
        || !valid_digest(&suite.prompt_config_digest)
        || !valid_digest(&suite.toolset_digest)
        || !valid_digest(&suite.hardware_profile_digest)
        || !valid_digest(&suite.budget_profile_digest)
        || suite.retrieval_top_k == 0
        || suite.context_window_tokens == 0
        || suite.per_task_token_budget == 0
        || suite.per_task_wall_clock_budget_ms == 0
        || suite.per_task_compute_budget_millijoules == 0
        || suite.expected_task_ids.is_empty()
        || suite.expected_architectures.is_empty()
    {
        return Err("MCSF suite identity, control digests, budgets or inventories are incomplete".into());
    }

    let mut task_ids = suite.expected_task_ids.iter().collect::<Vec<_>>();
    task_ids.sort();
    let mut task_set = BTreeSet::new();
    for task_id in &task_ids {
        if !nonempty(task_id) || !task_set.insert(task_id.as_str()) {
            return Err("MCSF expected task IDs are invalid or duplicated".into());
        }
    }

    let mut architectures = suite.expected_architectures.clone();
    architectures.sort();
    let architecture_set = architectures.iter().copied().collect::<BTreeSet<_>>();
    if architecture_set.len() != architectures.len() {
        return Err("MCSF expected architecture list contains duplicates".into());
    }

    let mut bytes = b"MYCELIX-MCSF-MATCHED-CONTROLS-V0.1\0".to_vec();
    push_str(&mut bytes, &suite.suite_id);
    bytes.push(match suite.track {
        McsfTrack::Textual => 1,
        McsfTrack::Coding => 2,
        McsfTrack::Multimodal => 3,
        McsfTrack::GovernanceOnly => 4,
    });
    for digest in [
        &suite.task_manifest_digest,
        &suite.scenario_manifest_digest,
        &suite.evaluator_digest,
        &suite.answer_model_digest,
        &suite.prompt_config_digest,
        &suite.toolset_digest,
        &suite.hardware_profile_digest,
        &suite.budget_profile_digest,
    ] {
        push_str(&mut bytes, digest);
    }
    bytes.extend_from_slice(&suite.seed.to_be_bytes());
    bytes.extend_from_slice(&suite.retrieval_top_k.to_be_bytes());
    bytes.extend_from_slice(&suite.context_window_tokens.to_be_bytes());
    bytes.extend_from_slice(&suite.per_task_token_budget.to_be_bytes());
    bytes.extend_from_slice(&suite.per_task_wall_clock_budget_ms.to_be_bytes());
    bytes.extend_from_slice(&suite.per_task_compute_budget_millijoules.to_be_bytes());

    push_len(&mut bytes, task_ids.len());
    for task_id in task_ids {
        push_str(&mut bytes, task_id);
    }
    push_len(&mut bytes, architectures.len());
    for architecture in architectures {
        bytes.push(match architecture {
            McsfArchitecture::NoMemory => 1,
            McsfArchitecture::Bm25Sparse => 2,
            McsfArchitecture::DenseVector => 3,
            McsfArchitecture::Hdc => 4,
            McsfArchitecture::HdcCfc => 5,
            McsfArchitecture::HybridSparseDenseHdcCfc => 6,
            McsfArchitecture::GraphRelational => 7,
            McsfArchitecture::GovernedMycelix => 8,
            McsfArchitecture::CoordinatedMycelix => 9,
        });
    }
    Ok(sha256_hex(&bytes))
}

fn deterministic_bootstrap_ci95(values: &[i64], seed_material: &str) -> Option<(i64, i64)> {
    if values.len() < 2 {
        return None;
    }

    let digest = Sha256::digest(seed_material.as_bytes());
    let mut state = u64::from_be_bytes(digest[..8].try_into().ok()?).max(1);
    let mut means = Vec::with_capacity(10_000);

    for _ in 0..10_000 {
        let mut sum = 0i128;
        for _ in 0..values.len() {
            // xorshift64*: fixed local PRNG seeded by the matched manifest and
            // architecture. It is for reproducible resampling, not security.
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            let index = (state as usize) % values.len();
            sum += i128::from(values[index]);
        }
        means.push((sum / values.len() as i128) as i64);
    }

    means.sort_unstable();
    let lower = means[(means.len() - 1) * 25 / 1_000];
    let upper = means[(means.len() - 1) * 975 / 1_000];
    Some((lower, upper))
}

fn valid_optional_digest(value: Option<&str>) -> bool {
    value.is_some_and(valid_digest)
}

fn empty_scorecard(
    architecture: McsfArchitecture,
    policy_class: McsfPolicyClass,
) -> McsfArchitectureScorecard {
    McsfArchitectureScorecard {
        architecture,
        policy_class,
        utility_task_count: 0,
        mean_utility_score_micros: None,
        utility_by_task_family_micros: BTreeMap::new(),
        utility_delta_by_task_family_vs_no_memory_micros: BTreeMap::new(),
        mean_utility_delta_95ci_vs_no_memory_micros: None,
        utility_delta_95ci_by_task_family_vs_no_memory_micros: BTreeMap::new(),
        authorized_task_success_count: 0,
        unauthorized_content_disclosures: 0,
        forbidden_metadata_inferences: 0,
        privacy_evidence_complete: true,
        revocation_evidence_complete: true,
        revocation_expected_descendants: 0,
        revocation_reconciled_descendants: 0,
        revocation_unknown_descendants: 0,
        causal_evidence_complete: true,
        retrieved_memory_count: 0,
        model_input_consumed_count: 0,
        decision_influential_count: 0,
        resource_usage_complete: true,
        resource_totals: Some(McsfResourceTotals::default()),
        role_evidence_complete: true,
        policy_class_eligible: policy_class == McsfPolicyClass::NativeUnrestricted,
        paired_task_count_vs_no_memory: 0,
        mean_utility_delta_vs_no_memory_micros: None,
        pareto_comparison_eligible: false,
    }
}

pub fn qualify_mcsf_suite(
    suite: &McsfSuiteManifest,
    runs: &[McsfRun],
) -> McsfQualificationReport {
    let mut qualification = QualificationReport::new();
    let control_digest = match compute_mcsf_control_digest(suite) {
        Ok(digest) => digest,
        Err(reason) => {
            qualification.fail(reason);
            String::new()
        }
    };

    let mut expected_tasks = BTreeSet::new();
    for task_id in &suite.expected_task_ids {
        if !nonempty(task_id) || !expected_tasks.insert(task_id.as_str()) {
            qualification.fail("MCSF task manifest contains empty or duplicate task IDs");
        }
    }
    let mut expected_architectures = BTreeSet::new();
    for architecture in &suite.expected_architectures {
        if !expected_architectures.insert(*architecture) {
            qualification.fail("MCSF architecture inventory contains duplicates");
        }
    }
    if !expected_architectures.contains(&McsfArchitecture::NoMemory) {
        qualification.indeterminate("MCSF comparison has no NoMemory reference architecture");
    }

    let mut run_ids = BTreeSet::new();
    let mut observed_architectures = BTreeSet::new();
    let mut scorecards = Vec::new();
    let mut task_score_maps =
        BTreeMap::<McsfArchitecture, BTreeMap<String, (String, u32)>>::new();
    let mut canonical_task_families = BTreeMap::<String, String>::new();

    for run in runs {
        if !nonempty(&run.run_id)
            || !run_ids.insert(run.run_id.as_str())
            || !observed_architectures.insert(run.architecture.architecture)
        {
            qualification.fail("MCSF run ID or architecture identity is duplicated/invalid");
        }

        let manifest = &run.architecture;
        if !nonempty(&manifest.system_id)
            || !nonempty(&manifest.system_version)
            || !valid_commit(&manifest.system_commit)
            || !valid_digest(&manifest.system_artifact_digest)
            || !valid_digest(&manifest.memory_representation_digest)
            || !valid_digest(&manifest.index_update_policy_digest)
            || !valid_digest(&manifest.architecture_config_digest)
        {
            qualification.fail(format!(
                "MCSF architecture {:?} has invalid system or configuration provenance",
                manifest.architecture
            ));
        }
        if !expected_architectures.contains(&manifest.architecture) {
            qualification.fail(format!(
                "MCSF run includes unlisted architecture {:?}",
                manifest.architecture
            ));
        }
        if run.control_digest != control_digest {
            qualification.fail(format!(
                "MCSF architecture {:?} is not using the exact matched control manifest",
                manifest.architecture
            ));
        }

        let policy_class_eligible = match manifest.policy_class {
            // Explicitly identifying an unrestricted native run is a valid claim
        // class, but it is not an official third-party division score.
        McsfPolicyClass::NativeUnrestricted => true,
            McsfPolicyClass::OfficialDivisionCompatible => {
                valid_optional_digest(manifest.official_policy_receipt_digest.as_deref())
            }
        };
        let mut card = empty_scorecard(manifest.architecture, manifest.policy_class);
        card.policy_class_eligible = policy_class_eligible;
        if !policy_class_eligible {
            card.pareto_comparison_eligible = false;
        }

        card.privacy_evidence_complete =
            valid_optional_digest(run.evidence_references.privacy_trace_report_digest.as_deref());
        card.revocation_evidence_complete = valid_optional_digest(
            run.evidence_references.revocation_report_digest.as_deref(),
        );
        card.causal_evidence_complete = valid_optional_digest(
            run.evidence_references.model_input_witness_report_digest.as_deref(),
        ) && valid_optional_digest(
            run.evidence_references.causal_replay_report_digest.as_deref(),
        );
        card.role_evidence_complete = if manifest.architecture
            == McsfArchitecture::CoordinatedMycelix
        {
            valid_optional_digest(
                run.evidence_references.role_enforcement_report_digest.as_deref(),
            )
        } else {
            true
        };

        let mut task_ids = BTreeSet::new();
        let mut utility_sum = 0u64;
        let mut success_count = 0usize;
        let mut task_scores = BTreeMap::new();
        let mut family_score_totals = BTreeMap::<String, (u64, usize)>::new();
        let mut resource_totals = McsfResourceTotals::default();

        for observation in &run.task_observations {
            if !nonempty(&observation.task_id)
                || !nonempty(&observation.task_family)
                || !task_ids.insert(observation.task_id.as_str())
            {
                qualification.fail(format!(
                    "MCSF architecture {:?} has empty/duplicate task observation",
                    manifest.architecture
                ));
                continue;
            }
            if !expected_tasks.contains(observation.task_id.as_str()) {
                qualification.fail(format!(
                    "MCSF architecture {:?} contains unlisted task {}",
                    manifest.architecture, observation.task_id
                ));
                continue;
            }
            match canonical_task_families.get(&observation.task_id) {
                Some(family) if family != &observation.task_family => {
                    qualification.fail(format!(
                        "task {} changed task-family identity across architectures",
                        observation.task_id
                    ));
                }
                None => {
                    canonical_task_families.insert(
                        observation.task_id.clone(),
                        observation.task_family.clone(),
                    );
                }
                _ => {}
            }

            match observation.state {
                McsfTaskState::Evaluated => {
                    if !valid_digest(&observation.evidence_digest) {
                        qualification.indeterminate(format!(
                            "task {} lacks a valid evidence digest",
                            observation.task_id
                        ));
                    }
                    match observation.utility_score_micros {
                        Some(score) if score <= 1_000_000 => {
                            utility_sum = utility_sum.saturating_add(u64::from(score));
                            card.utility_task_count += 1;
                            task_scores.insert(
                                observation.task_id.clone(),
                                (observation.task_family.clone(), score),
                            );
                            let family_totals = family_score_totals
                                .entry(observation.task_family.clone())
                                .or_default();
                            family_totals.0 = family_totals.0.saturating_add(u64::from(score));
                            family_totals.1 += 1;
                        }
                        Some(_) => qualification.fail(format!(
                            "task {} utility score exceeds [0, 1]",
                            observation.task_id
                        )),
                        None => qualification.indeterminate(format!(
                            "task {} utility score is missing",
                            observation.task_id
                        )),
                    }
                    match observation.authorized_task_pass {
                        Some(true) => success_count += 1,
                        Some(false) => {}
                        None => qualification.indeterminate(format!(
                            "task {} authorized-task outcome is missing",
                            observation.task_id
                        )),
                    }

                    card.unauthorized_content_disclosures +=
                        u64::from(observation.unauthorized_content_disclosures);
                    card.forbidden_metadata_inferences +=
                        u64::from(observation.forbidden_metadata_inferences);

                    match (
                        observation.revocation_expected_descendants,
                        observation.revocation_reconciled_descendants,
                    ) {
                        (Some(expected), Some(reconciled)) => {
                            card.revocation_expected_descendants += u64::from(expected);
                            card.revocation_reconciled_descendants += u64::from(reconciled);
                            card.revocation_unknown_descendants +=
                                u64::from(observation.revocation_unknown_descendants);
                            if reconciled != expected || observation.revocation_unknown_descendants > 0 {
                                card.revocation_evidence_complete = false;
                            }
                        }
                        _ => card.revocation_evidence_complete = false,
                    }

                    if !valid_optional_digest(observation.causal_evidence_digest.as_deref()) {
                        card.causal_evidence_complete = false;
                    }
                    if observation.decision_influential_count
                        > observation.model_input_consumed_count
                        || observation.model_input_consumed_count
                            > observation.retrieved_memory_count
                    {
                        qualification.fail(format!(
                            "task {} causal counts violate retrieved >= consumed >= influential",
                            observation.task_id
                        ));
                    }
                    card.retrieved_memory_count += u64::from(observation.retrieved_memory_count);
                    card.model_input_consumed_count +=
                        u64::from(observation.model_input_consumed_count);
                    card.decision_influential_count +=
                        u64::from(observation.decision_influential_count);

                    match &observation.resource_usage {
                        Some(usage)
                            if valid_digest(&usage.evidence_digest)
                                && usage.wall_clock_ms > 0 =>
                        {
                            resource_totals.input_tokens =
                                resource_totals.input_tokens.saturating_add(usage.input_tokens);
                            resource_totals.output_tokens =
                                resource_totals.output_tokens.saturating_add(usage.output_tokens);
                            resource_totals.memory_storage_bytes = resource_totals
                                .memory_storage_bytes
                                .max(usage.memory_storage_bytes);
                            resource_totals.index_write_time_ms = resource_totals
                                .index_write_time_ms
                                .saturating_add(usage.index_write_time_ms);
                            resource_totals.search_time_ms =
                                resource_totals.search_time_ms.saturating_add(usage.search_time_ms);
                            resource_totals.wall_clock_ms =
                                resource_totals.wall_clock_ms.saturating_add(usage.wall_clock_ms);
                            match (resource_totals.energy_millijoules, usage.energy_millijoules) {
                                (Some(total), Some(value)) => {
                                    resource_totals.energy_millijoules =
                                        Some(total.saturating_add(value));
                                }
                                (None, Some(value)) if resource_totals.task_count == 0 => {
                                    resource_totals.energy_millijoules = Some(value);
                                }
                                _ => resource_totals.energy_millijoules = None,
                            }
                            match (
                                resource_totals.provider_cost_micro_usd,
                                usage.provider_cost_micro_usd,
                            ) {
                                (Some(total), Some(value)) => {
                                    resource_totals.provider_cost_micro_usd =
                                        Some(total.saturating_add(value));
                                }
                                (None, Some(value)) if resource_totals.task_count == 0 => {
                                    resource_totals.provider_cost_micro_usd = Some(value);
                                }
                                _ => resource_totals.provider_cost_micro_usd = None,
                            }
                            resource_totals.task_count += 1;
                        }
                        _ => card.resource_usage_complete = false,
                    }
                }
                McsfTaskState::InfrastructureError | McsfTaskState::NotRun => {
                    card.resource_usage_complete = false;
                    card.privacy_evidence_complete = false;
                    card.revocation_evidence_complete = false;
                    card.causal_evidence_complete = false;
                    qualification.indeterminate(format!(
                        "MCSF task {} is not an evaluated outcome",
                        observation.task_id
                    ));
                }
            }
        }

        for task_id in &expected_tasks {
            if !task_ids.contains(task_id) {
                qualification.indeterminate(format!(
                    "MCSF architecture {:?} is missing task {}",
                    manifest.architecture, task_id
                ));
                card.privacy_evidence_complete = false;
                card.revocation_evidence_complete = false;
                card.causal_evidence_complete = false;
                card.resource_usage_complete = false;
            }
        }

        if let Some(mean) = utility_sum.checked_div(card.utility_task_count as u64) {
            card.mean_utility_score_micros = Some(mean as u32);
        }
        for (family, (sum, count)) in family_score_totals {
            if let Some(mean) = sum.checked_div(count as u64) {
                card.utility_by_task_family_micros
                    .insert(family, mean as u32);
            }
        }
        card.authorized_task_success_count = success_count;

        if card.resource_usage_complete
            && resource_totals.task_count == expected_tasks.len()
        {
            card.resource_totals = Some(resource_totals);
        } else {
            card.resource_usage_complete = false;
            card.resource_totals = None;
        }

        if !valid_optional_digest(run.evidence_references.privacy_trace_report_digest.as_deref()) {
            card.privacy_evidence_complete = false;
        }
        if !valid_optional_digest(run.evidence_references.revocation_report_digest.as_deref()) {
            card.revocation_evidence_complete = false;
        }
        if !valid_optional_digest(
            run.evidence_references.model_input_witness_report_digest.as_deref(),
        ) || !valid_optional_digest(run.evidence_references.causal_replay_report_digest.as_deref())
        {
            card.causal_evidence_complete = false;
        }
        if card.unauthorized_content_disclosures > 0
            || card.forbidden_metadata_inferences > 0
            || card.revocation_unknown_descendants > 0
            || card.revocation_expected_descendants != card.revocation_reconciled_descendants
        {
            card.pareto_comparison_eligible = false;
        } else {
            card.pareto_comparison_eligible = card.privacy_evidence_complete
                && card.revocation_evidence_complete
                && card.causal_evidence_complete
                && card.resource_usage_complete
                && card.role_evidence_complete
                && card.policy_class_eligible
                && card.utility_task_count == expected_tasks.len();
        }

        task_score_maps.insert(manifest.architecture, task_scores);
        scorecards.push(card);
    }

    for architecture in &expected_architectures {
        if !observed_architectures.contains(architecture) {
            qualification.indeterminate(format!(
                "MCSF expected architecture {architecture:?} is missing"
            ));
        }
    }

    let no_memory_scores = task_score_maps.get(&McsfArchitecture::NoMemory);
    for card in &mut scorecards {
        if let Some(baseline) = no_memory_scores {
            let candidate = task_score_maps.get(&card.architecture);
            if let Some(candidate) = candidate {
                let mut deltas = Vec::new();
                let mut family_deltas = BTreeMap::<String, (i128, usize, Vec<i64>)>::new();
                for (task_id, (family, score)) in candidate {
                    if let Some((baseline_family, baseline_score)) = baseline.get(task_id) {
                        if family != baseline_family {
                            qualification.fail(format!(
                                "task {task_id} family differs between {:?} and NoMemory",
                                card.architecture
                            ));
                            continue;
                        }
                        let delta = i64::from(*score) - i64::from(*baseline_score);
                        deltas.push(delta);
                        let family_total = family_deltas.entry(family.clone()).or_default();
                        family_total.0 += i128::from(delta);
                        family_total.1 += 1;
                        family_total.2.push(delta);
                    }
                }
                card.paired_task_count_vs_no_memory = deltas.len();
                let sum: i128 = deltas.iter().map(|delta| i128::from(*delta)).sum();
                if let Some(mean) = sum.checked_div(deltas.len() as i128) {
                    card.mean_utility_delta_vs_no_memory_micros = Some(mean as i64);
                    let seed = format!(
                        "MCSF-PAIRED-UTILITY-CI95-V0.1:{control_digest}:{:?}:overall",
                        card.architecture
                    );
                    card.mean_utility_delta_95ci_vs_no_memory_micros =
                        deterministic_bootstrap_ci95(&deltas, &seed);
                }
                for (family, (sum, count, family_values)) in family_deltas {
                    if let Some(mean) = sum.checked_div(count as i128) {
                        card.utility_delta_by_task_family_vs_no_memory_micros
                            .insert(family.clone(), mean as i64);
                        let seed = format!(
                            "MCSF-PAIRED-UTILITY-CI95-V0.1:{control_digest}:{:?}:{family}",
                            card.architecture
                        );
                        if let Some(interval) =
                            deterministic_bootstrap_ci95(&family_values, &seed)
                        {
                            card.utility_delta_95ci_by_task_family_vs_no_memory_micros
                                .insert(family, interval);
                        }
                    }
                }
            }
        }
    }

    scorecards.sort_by_key(|card| card.architecture);

    McsfQualificationReport {
        qualification: qualification.clone(),
        matched_control_digest: control_digest,
        expected_task_count: suite.expected_task_ids.len(),
        expected_architecture_count: suite.expected_architectures.len(),
        observed_architecture_count: observed_architectures.len(),
        scorecards,
        reasons: qualification.reasons.clone(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const D1: &str = "1111111111111111111111111111111111111111111111111111111111111111";
    const D2: &str = "2222222222222222222222222222222222222222222222222222222222222222";
    const D3: &str = "3333333333333333333333333333333333333333333333333333333333333333";
    const D4: &str = "4444444444444444444444444444444444444444444444444444444444444444";
    const D5: &str = "5555555555555555555555555555555555555555555555555555555555555555";
    const D6: &str = "6666666666666666666666666666666666666666666666666666666666666666";

    fn suite() -> McsfSuiteManifest {
        McsfSuiteManifest {
            suite_id: "mcsf-smoke-v0.1".into(),
            track: McsfTrack::Textual,
            task_manifest_digest: D1.into(),
            scenario_manifest_digest: D2.into(),
            evaluator_digest: D3.into(),
            answer_model_digest: D4.into(),
            prompt_config_digest: D5.into(),
            toolset_digest: D6.into(),
            hardware_profile_digest: D1.into(),
            budget_profile_digest: D2.into(),
            seed: 42,
            retrieval_top_k: 10,
            context_window_tokens: 16_384,
            per_task_token_budget: 8_000,
            per_task_wall_clock_budget_ms: 120_000,
            per_task_compute_budget_millijoules: 500_000,
            expected_task_ids: vec!["task-1".into(), "task-2".into()],
            expected_architectures: vec![McsfArchitecture::NoMemory, McsfArchitecture::HdcCfc],
        }
    }

    fn run(architecture: McsfArchitecture, suite: &McsfSuiteManifest, score: u32) -> McsfRun {
        let control_digest = compute_mcsf_control_digest(suite).unwrap();
        McsfRun {
            run_id: format!("run-{architecture:?}"),
            control_digest,
            architecture: McsfArchitectureManifest {
                architecture,
                policy_class: McsfPolicyClass::NativeUnrestricted,
                system_id: format!("system-{architecture:?}"),
                system_version: "0.1.0".into(),
                system_commit: "a".repeat(40),
                system_artifact_digest: D3.into(),
                memory_representation_digest: D4.into(),
                index_update_policy_digest: D5.into(),
                architecture_config_digest: D6.into(),
                official_policy_receipt_digest: None,
            },
            evidence_references: McsfEvidenceReferences {
                privacy_trace_report_digest: Some(D1.into()),
                revocation_report_digest: Some(D2.into()),
                model_input_witness_report_digest: Some(D3.into()),
                role_enforcement_report_digest: None,
                causal_replay_report_digest: Some(D4.into()),
            },
            task_observations: vec![
                task("task-1", score),
                task("task-2", score.saturating_add(10_000).min(1_000_000)),
            ],
        }
    }

    fn task(task_id: &str, score: u32) -> McsfTaskObservation {
        McsfTaskObservation {
            task_id: task_id.into(),
            task_family: "fact_recall".into(),
            state: McsfTaskState::Evaluated,
            utility_score_micros: Some(score),
            authorized_task_pass: Some(true),
            unauthorized_content_disclosures: 0,
            forbidden_metadata_inferences: 0,
            revocation_expected_descendants: Some(2),
            revocation_reconciled_descendants: Some(2),
            revocation_unknown_descendants: 0,
            retrieved_memory_count: 3,
            model_input_consumed_count: 2,
            decision_influential_count: 1,
            causal_evidence_digest: Some(D5.into()),
            resource_usage: Some(McsfResourceUsage {
                input_tokens: 100,
                output_tokens: 50,
                memory_storage_bytes: 10_000,
                index_write_time_ms: 20,
                search_time_ms: 5,
                wall_clock_ms: 1_000,
                energy_millijoules: Some(100),
                provider_cost_micro_usd: Some(10),
                evidence_digest: D6.into(),
            }),
            evidence_digest: D1.into(),
        }
    }

    #[test]
    fn matched_control_digest_is_canonical_and_order_independent() {
        let mut input = suite();
        let original = compute_mcsf_control_digest(&input).unwrap();
        input.expected_task_ids.reverse();
        input.expected_architectures.reverse();
        assert_eq!(compute_mcsf_control_digest(&input).unwrap(), original);
        input.per_task_token_budget += 1;
        assert_ne!(compute_mcsf_control_digest(&input).unwrap(), original);
    }

    #[test]
    fn canonical_control_digest_matches_the_checked_in_three_architecture_vector() {
        let mut input = suite();
        input.suite_id = "mcsf-textual-pilot-v0.1".into();
        input.expected_architectures = vec![
            McsfArchitecture::NoMemory,
            McsfArchitecture::Bm25Sparse,
            McsfArchitecture::HdcCfc,
        ];
        assert_eq!(
            compute_mcsf_control_digest(&input).unwrap(),
            "1ea6cdc28afb3e75f970e0c1e2fa45de06449b2617be2e8ef34984dd8d5c8b55"
        );
    }

    #[test]
    fn complete_matched_suite_emits_a_vector_and_paired_utility_delta() {
        let input = suite();
        let runs = vec![
            run(McsfArchitecture::NoMemory, &input, 500_000),
            run(McsfArchitecture::HdcCfc, &input, 600_000),
        ];
        let report = qualify_mcsf_suite(&input, &runs);
        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        let hdc = report.scorecards.iter()
            .find(|card| card.architecture == McsfArchitecture::HdcCfc).unwrap();
        assert_eq!(hdc.mean_utility_score_micros, Some(605_000));
        assert_eq!(hdc.mean_utility_delta_vs_no_memory_micros, Some(100_000));
        assert_eq!(hdc.utility_by_task_family_micros.get("fact_recall"), Some(&605_000));
        assert_eq!(
            hdc.utility_delta_by_task_family_vs_no_memory_micros.get("fact_recall"),
            Some(&100_000)
        );
        assert!(hdc.pareto_comparison_eligible);
    }

    #[test]
    fn task_family_identity_cannot_change_between_architectures() {
        let input = suite();
        let mut hdc = run(McsfArchitecture::HdcCfc, &input, 600_000);
        hdc.task_observations[0].task_family = "temporal_reasoning".into();
        let report = qualify_mcsf_suite(
            &input,
            &[run(McsfArchitecture::NoMemory, &input, 500_000), hdc],
        );
        assert_eq!(report.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn family_regression_remains_visible_even_when_overall_mean_is_positive() {
        let input = suite();
        let mut baseline = run(McsfArchitecture::NoMemory, &input, 500_000);
        let mut hdc = run(McsfArchitecture::HdcCfc, &input, 600_000);
        baseline.task_observations[0].task_family = "fact_recall".into();
        baseline.task_observations[1].task_family = "temporal_reasoning".into();
        hdc.task_observations[0].task_family = "fact_recall".into();
        hdc.task_observations[1].task_family = "temporal_reasoning".into();
        hdc.task_observations[0].utility_score_micros = Some(900_000);
        hdc.task_observations[1].utility_score_micros = Some(300_000);

        let report = qualify_mcsf_suite(&input, &[baseline, hdc]);
        let card = report.scorecards.iter()
            .find(|card| card.architecture == McsfArchitecture::HdcCfc).unwrap();
        assert!(card.mean_utility_delta_vs_no_memory_micros.unwrap() > 0);
        assert!(card.utility_delta_by_task_family_vs_no_memory_micros["fact_recall"] > 0);
        assert!(card.utility_delta_by_task_family_vs_no_memory_micros["temporal_reasoning"] < 0);
    }

    #[test]
    fn paired_bootstrap_intervals_are_deterministic_and_stratified() {
        let mut input = suite();
        input.expected_task_ids = (1..=6).map(|i| format!("task-{i}")).collect();

        let mut baseline = run(McsfArchitecture::NoMemory, &input, 500_000);
        let mut hdc = run(McsfArchitecture::HdcCfc, &input, 600_000);
        for i in 3..=6 {
            let mut baseline_task = task(&format!("task-{i}"), 500_000);
            let mut hdc_task = task(&format!("task-{i}"), 600_000);
            let family = if i <= 4 { "fact_recall" } else { "temporal_reasoning" };
            baseline_task.task_family = family.into();
            hdc_task.task_family = family.into();
            baseline_task.utility_score_micros = Some(480_000 + i * 1_000);
            hdc_task.utility_score_micros = Some(560_000 + i * 2_000);
            baseline.task_observations.push(baseline_task);
            hdc.task_observations.push(hdc_task);
        }
        for run in [&mut baseline, &mut hdc] {
            for observation in &mut run.task_observations[..2] {
                observation.task_family = "fact_recall".into();
            }
        }

        let first = qualify_mcsf_suite(&input, &[baseline.clone(), hdc.clone()]);
        let second = qualify_mcsf_suite(&input, &[baseline, hdc]);
        let card = first.scorecards.iter()
            .find(|card| card.architecture == McsfArchitecture::HdcCfc).unwrap();
        assert_eq!(
            card.mean_utility_delta_95ci_vs_no_memory_micros,
            Some((second.scorecards.iter()
                .find(|card| card.architecture == McsfArchitecture::HdcCfc).unwrap()
                .mean_utility_delta_95ci_vs_no_memory_micros).unwrap())
        );
        assert!(card.mean_utility_delta_95ci_vs_no_memory_micros.is_some());
        assert_eq!(
            card.utility_delta_95ci_by_task_family_vs_no_memory_micros.len(),
            2
        );
    }

    #[test]
    fn privacy_failure_cannot_be_compensated_by_high_utility() {
        let input = suite();
        let mut hdc = run(McsfArchitecture::HdcCfc, &input, 1_000_000);
        hdc.task_observations[0].unauthorized_content_disclosures = 1;
        let report = qualify_mcsf_suite(
            &input,
            &[run(McsfArchitecture::NoMemory, &input, 500_000), hdc],
        );
        let card = report.scorecards.iter()
            .find(|card| card.architecture == McsfArchitecture::HdcCfc).unwrap();
        assert_eq!(card.mean_utility_score_micros, Some(1_000_000));
        assert_eq!(card.unauthorized_content_disclosures, 1);
        assert!(!card.pareto_comparison_eligible);
    }

    #[test]
    fn missing_privacy_or_revocation_reports_never_become_zero_risk() {
        let input = suite();
        let mut hdc = run(McsfArchitecture::HdcCfc, &input, 600_000);
        hdc.evidence_references.privacy_trace_report_digest = None;
        hdc.evidence_references.revocation_report_digest = None;
        let report = qualify_mcsf_suite(&input, &[run(McsfArchitecture::NoMemory, &input, 500_000), hdc]);
        let card = report.scorecards.iter()
            .find(|card| card.architecture == McsfArchitecture::HdcCfc).unwrap();
        assert!(!card.privacy_evidence_complete);
        assert!(!card.revocation_evidence_complete);
        assert!(!card.pareto_comparison_eligible);
    }

    #[test]
    fn causal_counts_must_be_retrieved_then_consumed_then_influential() {
        let input = suite();
        let mut hdc = run(McsfArchitecture::HdcCfc, &input, 600_000);
        hdc.task_observations[0].decision_influential_count = 4;
        let report = qualify_mcsf_suite(&input, &[run(McsfArchitecture::NoMemory, &input, 500_000), hdc]);
        assert_eq!(report.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn missing_cost_is_unknown_and_withholds_resource_totals() {
        let input = suite();
        let mut hdc = run(McsfArchitecture::HdcCfc, &input, 600_000);
        hdc.task_observations[0].resource_usage = None;
        let report = qualify_mcsf_suite(&input, &[run(McsfArchitecture::NoMemory, &input, 500_000), hdc]);
        let card = report.scorecards.iter()
            .find(|card| card.architecture == McsfArchitecture::HdcCfc).unwrap();
        assert!(!card.resource_usage_complete);
        assert!(card.resource_totals.is_none());
        assert!(!card.pareto_comparison_eligible);
    }

    #[test]
    fn matched_task_missingness_is_indeterminate_not_a_partial_win() {
        let input = suite();
        let mut hdc = run(McsfArchitecture::HdcCfc, &input, 600_000);
        hdc.task_observations.pop();
        let report = qualify_mcsf_suite(&input, &[run(McsfArchitecture::NoMemory, &input, 500_000), hdc]);
        assert_eq!(report.qualification.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn mismatched_budget_control_digest_fails() {
        let input = suite();
        let mut hdc = run(McsfArchitecture::HdcCfc, &input, 600_000);
        hdc.control_digest = D6.into();
        let report = qualify_mcsf_suite(&input, &[run(McsfArchitecture::NoMemory, &input, 500_000), hdc]);
        assert_eq!(report.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn official_division_compatible_run_requires_an_policy_receipt() {
        let input = suite();
        let mut hdc = run(McsfArchitecture::HdcCfc, &input, 600_000);
        hdc.architecture.policy_class = McsfPolicyClass::OfficialDivisionCompatible;
        hdc.architecture.official_policy_receipt_digest = None;
        let report = qualify_mcsf_suite(&input, &[run(McsfArchitecture::NoMemory, &input, 500_000), hdc]);
        let card = report.scorecards.iter()
            .find(|card| card.architecture == McsfArchitecture::HdcCfc).unwrap();
        assert!(!card.policy_class_eligible);
        assert!(!card.pareto_comparison_eligible);
    }

    #[test]
    fn duplicate_architecture_or_task_entries_fail() {
        let input = suite();
        let no_memory = run(McsfArchitecture::NoMemory, &input, 500_000);
        let hdc = run(McsfArchitecture::HdcCfc, &input, 600_000);
        let mut duplicate = hdc.clone();
        duplicate.run_id = "duplicate-run".into();
        let report = qualify_mcsf_suite(&input, &[no_memory.clone(), hdc, duplicate]);
        assert_eq!(report.qualification.status, QualificationStatus::Fail);

        let mut duplicate_task = run(McsfArchitecture::HdcCfc, &input, 600_000);
        duplicate_task.task_observations.push(duplicate_task.task_observations[0].clone());
        let report = qualify_mcsf_suite(&input, &[no_memory, duplicate_task]);
        assert_eq!(report.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn report_is_deterministic_for_identical_input() {
        let input = suite();
        let runs = vec![
            run(McsfArchitecture::NoMemory, &input, 500_000),
            run(McsfArchitecture::HdcCfc, &input, 600_000),
        ];
        assert_eq!(
            qualify_mcsf_suite(&input, &runs),
            qualify_mcsf_suite(&input, &runs)
        );
    }
}
