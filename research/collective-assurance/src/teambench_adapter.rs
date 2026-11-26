//! Read-only ingestion and qualification of native TeamBench run artifacts.
//!
//! This does not execute TeamBench, replace its native grader, or infer the
//! truth of external results. It qualifies a version-pinned run package and
//! keeps mock harness smoke evidence distinct from measured model evaluation.

use std::collections::{BTreeMap, BTreeSet};

use sha2::{Digest, Sha256};

use super::{QualificationReport, QualificationStatus};

const UPSTREAM_REPOSITORY: &str = "https://github.com/ybkim95/TeamBench";
const UPSTREAM_COMMIT: &str = "d185aef1916fd86a9ba554d581fd256319a973af";
const UPSTREAM_TASK_MANIFEST_PATH: &str = "leaderboard/data/leaderboard_90_tasks.json";
const UPSTREAM_TASK_MANIFEST_BLOB_SHA: &str =
    "59125b3fa9a82eede1f5a832c51c2ba32f5af591";
const UPSTREAM_README_PATH: &str = "README.md";
const UPSTREAM_README_BLOB_SHA: &str =
    "6c39b66fed83c52f397c2d3b4f84e7bafe854061";

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

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum TeamBenchCondition {
    Oracle,
    Restricted,
    TeamNoVerify,
    TeamNoPlan,
    Full,
}

impl TeamBenchCondition {
    fn all() -> [Self; 5] {
        [
            Self::Oracle,
            Self::Restricted,
            Self::TeamNoVerify,
            Self::TeamNoPlan,
            Self::Full,
        ]
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TeamBenchRunMode {
    /// Deterministic mock-provider smoke: integration evidence, never a score.
    MockProviderSmoke,
    /// Named model/provider under a pinned evaluation configuration.
    ModelEvaluation,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum TeamBenchRole {
    Oracle,
    Restricted,
    Planner,
    Executor,
    Verifier,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TeamBenchExecutionBackend {
    /// Role capabilities are enforced by in-process tool/path wrappers.
    /// This is not OS/container isolation.
    InProcessToolBoundary,
    /// Roles execute in independently constrained containers with a
    /// separately committed runtime/sandbox observation manifest.
    DockerContainerIsolation,
    /// The supplied run package does not identify its enforcement backend.
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TeamBenchTaskState {
    Evaluated,
    InfrastructureError,
    NotRun,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TeamBenchGraderDisposition {
    Pass,
    Fail,
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TeamBenchVerifierDisposition {
    Accept,
    Reject,
    NoVerdict,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TeamBenchBoundaryObservation {
    Allowed,
    Blocked,
    Unknown,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TeamBenchExcludedTask {
    pub task_id: String,
    /// E.g. upstream re-curation, invalid fixture or unavailable source asset.
    pub reason: String,
    /// Commitment to the upstream notice or local adjudication evidence.
    pub evidence_digest: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TeamBenchRoleBoundaryEvent {
    pub event_id: String,
    pub actor_role: TeamBenchRole,
    /// Exact capability boundary identifier from the pinned role profile.
    pub boundary_id: String,
    /// Expected allow/deny disposition in the exact role profile.
    pub expected_allowed: bool,
    /// Independently observed boundary outcome, not a model self-report.
    pub observed: TeamBenchBoundaryObservation,
    pub evidence_digest: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TeamBenchResourceUsage {
    pub input_tokens: u64,
    pub output_tokens: u64,
    pub wall_clock_ms: u64,
    pub provider_cost_micro_usd: u64,
    pub tool_call_count: u64,
    pub inter_agent_message_count: u64,
    /// Digest of independently captured usage/cost telemetry for this task.
    pub evidence_digest: String,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct TeamBenchResourceTotals {
    pub input_tokens: u64,
    pub output_tokens: u64,
    pub wall_clock_ms: u64,
    pub provider_cost_micro_usd: u64,
    pub tool_call_count: u64,
    pub inter_agent_message_count: u64,
    pub task_count: usize,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TeamBenchTaskObservation {
    pub task_id: String,
    pub state: TeamBenchTaskState,
    /// Commitment to raw upstream task output/artifact bytes.
    pub raw_output_digest: Option<String>,
    /// Commitment to the native deterministic grader output.
    pub native_grader_output_digest: Option<String>,
    pub native_grader_disposition: Option<TeamBenchGraderDisposition>,
    /// Native TeamBench partial score in millionths on the [0, 1] scale.
    pub native_score_micros: Option<u32>,
    /// Optional, independently captured per-task usage/cost observation.
    /// Missing telemetry does not invalidate native benchmark score, but it
    /// prevents resource/cost comparison qualification.
    pub resource_usage: Option<TeamBenchResourceUsage>,
    /// None means absent or not captured; role inventory disambiguates.
    pub verifier_disposition: Option<TeamBenchVerifierDisposition>,
    pub role_boundary_events: Vec<TeamBenchRoleBoundaryEvent>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TeamBenchConditionRun {
    pub run_id: String,
    pub condition: TeamBenchCondition,
    pub system_config_digest: String,
    pub role_profile_digest: String,
    pub raw_result_manifest_digest: String,
    pub agent_count: u32,
    pub execution_backend: TeamBenchExecutionBackend,
    /// Required only when Docker/container isolation is independently evidenced.
    pub runtime_manifest_digest: Option<String>,
    /// Exact role inventory enabled by the pinned upstream condition.
    pub roles: Vec<TeamBenchRole>,
    /// False is an explicit incomplete observer inventory, not zero events.
    pub role_boundary_inventory_complete: bool,
    pub tasks: Vec<TeamBenchTaskObservation>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TeamBenchSuite {
    pub suite_id: String,
    pub run_mode: TeamBenchRunMode,
    pub upstream_repository: String,
    /// Exact 40-character commit in the official upstream repository.
    pub upstream_commit: String,
    pub benchmark_version: String,
    pub dataset_manifest_digest: String,
    /// SHA-256 of the canonical task set plus the source-blob/notice commitments.
    pub task_manifest_digest: String,
    pub task_manifest_path: String,
    /// Git blob SHA-1 of the exact upstream task-list file, not a content SHA-256.
    pub task_manifest_blob_sha: String,
    pub re_curation_notice_path: String,
    pub re_curation_notice_blob_sha: String,
    pub native_grader_digest: String,
    pub excluded_task_manifest_digest: String,
    /// Exact task IDs included in every condition; order is not significant.
    pub eligible_task_ids: Vec<String>,
    pub excluded_tasks: Vec<TeamBenchExcludedTask>,
    /// Shared controls across the five ablations.
    pub model_snapshot_digest: String,
    pub tokenizer_digest: String,
    pub prompt_config_digest: String,
    pub toolset_digest: String,
    pub decoding_config_digest: String,
    pub seed_policy_id: String,
    pub seed: u64,
    pub per_task_time_budget_ms: u64,
    pub per_task_token_budget: u64,
    pub per_task_compute_budget_millijoules: u64,
    pub condition_runs: Vec<TeamBenchConditionRun>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TeamBenchQualificationReport {
    pub qualification: QualificationReport,
    /// True only for a complete PASS of a real model-evaluation suite.
    pub benchmark_score_eligible: bool,
    /// True only when every model-evaluation condition has an independently
    /// committed Docker/container runtime manifest. Does not follow from a
    /// benchmark score or logical tool-boundary test.
    pub os_isolation_eligible: bool,
    /// Native benchmark output completeness is separate from role-audit evidence.
    pub role_boundary_evidence_complete: bool,
    pub role_enforcement_eligible: bool,
    pub verifier_evidence_complete: bool,
    pub verifier_evidence_missing_count: usize,
    /// Resource totals are published only when all tasks/conditions have
    /// complete independently committed telemetry.
    pub resource_usage_complete: bool,
    pub resource_usage_missing_or_invalid_count: usize,
    pub resource_totals_by_condition: BTreeMap<TeamBenchCondition, TeamBenchResourceTotals>,
    pub condition_count: usize,
    pub eligible_task_count: usize,
    pub task_observation_count: usize,
    /// Mean native partial score in millionths, keyed by official condition.
    /// This map is empty for mock-provider smoke and incomplete suites.
    pub mean_score_micros_by_condition: BTreeMap<TeamBenchCondition, u32>,
    /// Exact upstream TeamBench aggregate: mean per-task TNI, excluding
    /// absolute oracle-restricted gaps below 0.05 and aggregating only
    /// positive gaps above 0.05, matching harness/compute_tni.py.
    pub average_tni_micros: Option<i64>,
    pub valid_tni_task_count: usize,
    pub average_team_uplift_micros: Option<i64>,
    pub team_helps_count: usize,
    pub average_collaboration_efficiency_micros: Option<i64>,
    pub average_planning_value_micros: Option<i64>,
    pub planning_value_task_count: usize,
    pub average_verification_value_micros: Option<i64>,
    pub verification_value_task_count: usize,
    pub grader_pass_count: usize,
    pub grader_fail_count: usize,
    pub grader_unknown_count: usize,
    pub verifier_accept_count: usize,
    pub verifier_reject_count: usize,
    pub verifier_no_verdict_count: usize,
    pub false_accept_count: usize,
    /// Denominator: all known native-grader failures in verifier-enabled runs.
    pub false_accept_denominator: usize,
    pub false_reject_count: usize,
    /// Denominator: all known native-grader passes in verifier-enabled runs.
    pub false_reject_denominator: usize,
    pub role_boundary_violation_count: usize,
    pub role_boundary_false_denial_count: usize,
    pub role_boundary_unknown_count: usize,
    pub infrastructure_error_count: usize,
    pub not_run_count: usize,
    pub reasons: Vec<String>,
}

fn valid_git_blob_sha(value: &str) -> bool {
    value.len() == 40
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn push_len(output: &mut Vec<u8>, value: usize) {
    output.extend_from_slice(&(value as u64).to_be_bytes());
}

fn push_str(output: &mut Vec<u8>, value: &str) {
    push_len(output, value.len());
    output.extend_from_slice(value.as_bytes());
}

fn sha256_hex(value: &[u8]) -> String {
    let digest = Sha256::digest(value);
    let mut output = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write;
        let _ = write!(&mut output, "{byte:02x}");
    }
    output
}

/// Canonical, domain-separated digest of the explicit upstream re-curation
/// exclusions. The list order is normalized; identifiers and evidence cannot
/// be changed without changing the digest.
pub fn compute_teambench_excluded_manifest_digest(
    excluded_tasks: &[TeamBenchExcludedTask],
) -> Result<String, String> {
    let mut records = excluded_tasks.iter().collect::<Vec<_>>();
    records.sort_by(|left, right| left.task_id.cmp(&right.task_id));
    let mut seen = BTreeSet::new();
    let mut bytes = b"MYCELIX-TEAMBENCH-EXCLUSIONS-V0.1\0".to_vec();
    push_len(&mut bytes, records.len());
    for task in records {
        if !nonempty(&task.task_id)
            || !nonempty(&task.reason)
            || !valid_digest(&task.evidence_digest)
            || !seen.insert(task.task_id.as_str())
        {
            return Err("excluded task inventory is invalid or duplicated".into());
        }
        push_str(&mut bytes, &task.task_id);
        push_str(&mut bytes, &task.reason);
        push_str(&mut bytes, &task.evidence_digest);
    }
    Ok(sha256_hex(&bytes))
}

/// Canonical digest binding the pinned official source files and the exact
/// eligible/excluded task inventory used by a run.
pub fn compute_teambench_task_manifest_digest(
    upstream_commit: &str,
    task_manifest_path: &str,
    task_manifest_blob_sha: &str,
    re_curation_notice_path: &str,
    re_curation_notice_blob_sha: &str,
    eligible_task_ids: &[String],
    excluded_tasks: &[TeamBenchExcludedTask],
) -> Result<String, String> {
    if !valid_commit(upstream_commit)
        || task_manifest_path != UPSTREAM_TASK_MANIFEST_PATH
        || !valid_git_blob_sha(task_manifest_blob_sha)
        || re_curation_notice_path != UPSTREAM_README_PATH
        || !valid_git_blob_sha(re_curation_notice_blob_sha)
    {
        return Err("upstream task-manifest provenance is invalid".into());
    }

    let mut eligible = eligible_task_ids.iter().collect::<Vec<_>>();
    eligible.sort();
    let mut seen = BTreeSet::new();
    for task_id in &eligible {
        if !nonempty(task_id) || !seen.insert(task_id.as_str()) {
            return Err("eligible task inventory is invalid or duplicated".into());
        }
    }

    let mut excluded = excluded_tasks.iter().collect::<Vec<_>>();
    excluded.sort_by(|left, right| left.task_id.cmp(&right.task_id));
    for task in &excluded {
        if !nonempty(&task.task_id)
            || !nonempty(&task.reason)
            || !valid_digest(&task.evidence_digest)
            || !seen.insert(task.task_id.as_str())
        {
            return Err("eligible/excluded task inventory overlaps or is invalid".into());
        }
    }

    let exclusion_digest = compute_teambench_excluded_manifest_digest(excluded_tasks)?;
    let mut bytes = b"MYCELIX-TEAMBENCH-TASK-MANIFEST-V0.1\0".to_vec();
    push_str(&mut bytes, upstream_commit);
    push_str(&mut bytes, task_manifest_path);
    push_str(&mut bytes, task_manifest_blob_sha);
    push_str(&mut bytes, re_curation_notice_path);
    push_str(&mut bytes, re_curation_notice_blob_sha);
    push_len(&mut bytes, eligible.len());
    for task_id in eligible {
        push_str(&mut bytes, task_id);
    }
    push_str(&mut bytes, &exclusion_digest);
    Ok(sha256_hex(&bytes))
}

fn valid_optional_digest(value: Option<&str>) -> bool {
    value.is_some_and(valid_digest)
}

fn qualify_task(
    task: &TeamBenchTaskObservation,
    roles: &[TeamBenchRole],
    report: &mut TeamBenchQualificationReport,
) {
    let verifier_present = roles.contains(&TeamBenchRole::Verifier);
    match task.state {
        TeamBenchTaskState::Evaluated => {
            if !valid_optional_digest(task.raw_output_digest.as_deref()) {
                report.qualification.indeterminate(format!(
                    "task {} raw output evidence missing or invalid",
                    task.task_id
                ));
            }
            if !valid_optional_digest(task.native_grader_output_digest.as_deref()) {
                report.qualification.indeterminate(format!(
                    "task {} native grader output missing or invalid",
                    task.task_id
                ));
            }
            match task.native_score_micros {
                Some(score) if score <= 1_000_000 => {}
                Some(_) => report.qualification.fail(format!(
                    "task {} native score lies outside [0, 1]",
                    task.task_id
                )),
                None => report.qualification.indeterminate(format!(
                    "task {} native partial score missing",
                    task.task_id
                )),
            }
            match task.native_grader_disposition {
                Some(TeamBenchGraderDisposition::Pass) => report.grader_pass_count += 1,
                Some(TeamBenchGraderDisposition::Fail) => report.grader_fail_count += 1,
                Some(TeamBenchGraderDisposition::Unknown) => {
                    report.grader_unknown_count += 1;
                    report.qualification.indeterminate(format!(
                        "task {} native grader disposition unknown",
                        task.task_id
                    ));
                }
                None => report.qualification.indeterminate(format!(
                    "task {} native grader disposition missing",
                    task.task_id
                )),
            }

            if verifier_present {
                match task.verifier_disposition {
                    Some(TeamBenchVerifierDisposition::Accept) => {
                        report.verifier_accept_count += 1;
                        if task.native_grader_disposition
                            == Some(TeamBenchGraderDisposition::Fail)
                        {
                            report.false_accept_count += 1;
                        }
                    }
                    Some(TeamBenchVerifierDisposition::Reject) => {
                        report.verifier_reject_count += 1;
                        if task.native_grader_disposition
                            == Some(TeamBenchGraderDisposition::Pass)
                        {
                            report.false_reject_count += 1;
                        }
                    }
                    Some(TeamBenchVerifierDisposition::NoVerdict) => {
                        report.verifier_no_verdict_count += 1;
                    }
                    None => {
                        report.verifier_evidence_complete = false;
                        report.verifier_evidence_missing_count += 1;
                    }
                }
            } else if task.verifier_disposition.is_some() {
                report.qualification.fail(format!(
                    "task {} contains a verifier verdict although this condition declares no verifier",
                    task.task_id
                ));
            }
        }
        TeamBenchTaskState::InfrastructureError => {
            report.infrastructure_error_count += 1;
            if task.raw_output_digest.is_some()
                || task.native_grader_output_digest.is_some()
                || task.native_grader_disposition.is_some()
                || task.native_score_micros.is_some()
                || task.resource_usage.is_some()
                || task.verifier_disposition.is_some()
            {
                report.qualification.fail(format!(
                    "task {} infrastructure-error state carries evaluated output fields",
                    task.task_id
                ));
            } else {
                report.qualification.indeterminate(format!(
                    "task {} has no benchmark-evaluable outcome due to infrastructure error",
                    task.task_id
                ));
            }
        }
        TeamBenchTaskState::NotRun => {
            report.not_run_count += 1;
            if task.raw_output_digest.is_some()
                || task.native_grader_output_digest.is_some()
                || task.native_grader_disposition.is_some()
                || task.native_score_micros.is_some()
                || task.resource_usage.is_some()
                || task.verifier_disposition.is_some()
            {
                report.qualification.fail(format!(
                    "task {} marked not-run but carries result fields",
                    task.task_id
                ));
            } else {
                report
                    .qualification
                    .indeterminate(format!("task {} was not run", task.task_id));
            }
        }
    }

    let mut event_ids = BTreeSet::new();
    for event in &task.role_boundary_events {
        if !roles.contains(&event.actor_role) {
            report.role_boundary_unknown_count += 1;
            report.role_boundary_evidence_complete = false;
            report.role_enforcement_eligible = false;
            continue;
        }
        if !nonempty(&event.event_id)
            || !nonempty(&event.boundary_id)
            || !valid_digest(&event.evidence_digest)
            || !event_ids.insert(event.event_id.as_str())
        {
            report.role_boundary_unknown_count += 1;
            report.role_boundary_evidence_complete = false;
            report.role_enforcement_eligible = false;
            continue;
        }
        match event.observed {
            TeamBenchBoundaryObservation::Allowed if !event.expected_allowed => {
                report.role_boundary_violation_count += 1;
                report.role_enforcement_eligible = false;
            }
            TeamBenchBoundaryObservation::Blocked if event.expected_allowed => {
                report.role_boundary_false_denial_count += 1;
                report.role_enforcement_eligible = false;
            }
            TeamBenchBoundaryObservation::Unknown => {
                report.role_boundary_unknown_count += 1;
                report.role_boundary_evidence_complete = false;
                report.role_enforcement_eligible = false;
            }
            _ => {}
        }
    }
}

fn mean_i64(values: &[i64]) -> Option<i64> {
    if values.is_empty() {
        None
    } else {
        let sum: i128 = values.iter().map(|value| i128::from(*value)).sum();
        Some((sum / values.len() as i128) as i64)
    }
}

fn expected_roles(condition: TeamBenchCondition) -> BTreeSet<TeamBenchRole> {
    let roles: &[TeamBenchRole] = match condition {
        TeamBenchCondition::Oracle => &[TeamBenchRole::Oracle],
        TeamBenchCondition::Restricted => &[TeamBenchRole::Restricted],
        TeamBenchCondition::TeamNoVerify => {
            &[TeamBenchRole::Planner, TeamBenchRole::Executor]
        }
        TeamBenchCondition::TeamNoPlan => {
            &[TeamBenchRole::Executor, TeamBenchRole::Verifier]
        }
        TeamBenchCondition::Full => &[
            TeamBenchRole::Planner,
            TeamBenchRole::Executor,
            TeamBenchRole::Verifier,
        ],
    };
    roles.iter().copied().collect()
}

pub fn qualify_teambench_suite(suite: &TeamBenchSuite) -> TeamBenchQualificationReport {
    let mut report = TeamBenchQualificationReport {
        qualification: QualificationReport::new(),
        benchmark_score_eligible: false,
        os_isolation_eligible: false,
        role_boundary_evidence_complete: true,
        role_enforcement_eligible: true,
        verifier_evidence_complete: true,
        verifier_evidence_missing_count: 0,
        resource_usage_complete: true,
        resource_usage_missing_or_invalid_count: 0,
        resource_totals_by_condition: BTreeMap::new(),
        condition_count: suite.condition_runs.len(),
        eligible_task_count: suite.eligible_task_ids.len(),
        task_observation_count: 0,
        grader_pass_count: 0,
        grader_fail_count: 0,
        grader_unknown_count: 0,
        verifier_accept_count: 0,
        verifier_reject_count: 0,
        verifier_no_verdict_count: 0,
        false_accept_count: 0,
        false_accept_denominator: 0,
        false_reject_count: 0,
        false_reject_denominator: 0,
        role_boundary_violation_count: 0,
        role_boundary_false_denial_count: 0,
        role_boundary_unknown_count: 0,
        infrastructure_error_count: 0,
        not_run_count: 0,
        mean_score_micros_by_condition: BTreeMap::new(),
        average_tni_micros: None,
        valid_tni_task_count: 0,
        average_team_uplift_micros: None,
        team_helps_count: 0,
        average_collaboration_efficiency_micros: None,
        average_planning_value_micros: None,
        planning_value_task_count: 0,
        average_verification_value_micros: None,
        verification_value_task_count: 0,
        reasons: Vec::new(),
    };

    if !nonempty(&suite.suite_id)
        || suite.upstream_repository != UPSTREAM_REPOSITORY
        || suite.upstream_commit != UPSTREAM_COMMIT
        || !nonempty(&suite.benchmark_version)
        || !nonempty(&suite.seed_policy_id)
    {
        report.qualification.fail(
            "suite identity, official upstream source, commit, benchmark version or seed policy is invalid",
        );
    }
    for (name, digest) in [
        ("dataset manifest", suite.dataset_manifest_digest.as_str()),
        ("task manifest", suite.task_manifest_digest.as_str()),
        ("native grader", suite.native_grader_digest.as_str()),
        ("excluded task manifest", suite.excluded_task_manifest_digest.as_str()),
        ("model snapshot", suite.model_snapshot_digest.as_str()),
        ("tokenizer", suite.tokenizer_digest.as_str()),
        ("prompt configuration", suite.prompt_config_digest.as_str()),
        ("toolset", suite.toolset_digest.as_str()),
        ("decoding configuration", suite.decoding_config_digest.as_str()),
    ] {
        if !valid_digest(digest) {
            report
                .qualification
                .fail(format!("{name} digest is not lowercase SHA-256 hex"));
        }
    }
    if suite.task_manifest_path != UPSTREAM_TASK_MANIFEST_PATH
        || !valid_git_blob_sha(&suite.task_manifest_blob_sha)
        || suite.task_manifest_blob_sha != UPSTREAM_TASK_MANIFEST_BLOB_SHA
        || suite.re_curation_notice_path != UPSTREAM_README_PATH
        || !valid_git_blob_sha(&suite.re_curation_notice_blob_sha)
        || suite.re_curation_notice_blob_sha != UPSTREAM_README_BLOB_SHA
    {
        report.qualification.fail("TeamBench source file paths or Git blob identities differ from the pinned upstream manifest");
    }
    if suite.per_task_time_budget_ms == 0
        || suite.per_task_token_budget == 0
        || suite.per_task_compute_budget_millijoules == 0
    {
        report.qualification.fail(
            "per-task time, token and compute budgets must be explicit and non-zero",
        );
    }
    if suite.eligible_task_ids.is_empty() {
        report
            .qualification
            .indeterminate("eligible task inventory is empty");
    }

    let mut eligible = BTreeSet::new();
    for task_id in &suite.eligible_task_ids {
        if !nonempty(task_id) || !eligible.insert(task_id.as_str()) {
            report
                .qualification
                .fail("eligible task IDs must be non-empty and unique");
        }
    }
    let mut excluded = BTreeSet::new();
    for task in &suite.excluded_tasks {
        if !nonempty(&task.task_id)
            || !nonempty(&task.reason)
            || !valid_digest(&task.evidence_digest)
            || !excluded.insert(task.task_id.as_str())
        {
            report
                .qualification
                .fail("excluded-task record is malformed or duplicated");
        }
        if eligible.contains(task.task_id.as_str()) {
            report
                .qualification
                .fail("task is simultaneously eligible and excluded");
        }
    }

    match compute_teambench_excluded_manifest_digest(&suite.excluded_tasks) {
        Ok(digest) if digest == suite.excluded_task_manifest_digest => {}
        Ok(_) => report.qualification.fail("excluded-task manifest digest does not match its canonical records"),
        Err(reason) => report.qualification.fail(reason),
    }
    match compute_teambench_task_manifest_digest(
        &suite.upstream_commit,
        &suite.task_manifest_path,
        &suite.task_manifest_blob_sha,
        &suite.re_curation_notice_path,
        &suite.re_curation_notice_blob_sha,
        &suite.eligible_task_ids,
        &suite.excluded_tasks,
    ) {
        Ok(digest) if digest == suite.task_manifest_digest => {}
        Ok(_) => report.qualification.fail("task manifest digest does not match its canonical source/task records"),
        Err(reason) => report.qualification.fail(reason),
    }

    let mut conditions = BTreeMap::new();
    let mut run_ids = BTreeSet::new();
    let mut score_totals = BTreeMap::<TeamBenchCondition, (u64, u64)>::new();
    let mut task_scores = BTreeMap::<(String, TeamBenchCondition), u32>::new();
    let mut resource_totals = BTreeMap::<TeamBenchCondition, TeamBenchResourceTotals>::new();
    for run in &suite.condition_runs {
        if !nonempty(&run.run_id)
            || !valid_digest(&run.system_config_digest)
            || !valid_digest(&run.role_profile_digest)
            || !valid_digest(&run.raw_result_manifest_digest)
            || run.agent_count == 0
            || !run_ids.insert(run.run_id.as_str())
        {
            report.qualification.fail(
                "condition-run identity, configuration digests, agent count or run uniqueness is invalid",
            );
        }
        let actual_roles = run.roles.iter().copied().collect::<BTreeSet<_>>();
        if actual_roles.len() != run.roles.len() || actual_roles != expected_roles(run.condition) {
            report.qualification.fail(format!(
                "condition {:?} role inventory differs from the official upstream ablation",
                run.condition
            ));
        }
        if run.agent_count as usize != run.roles.len() {
            report.qualification.fail(format!(
                "condition {:?} agent count differs from its role inventory",
                run.condition
            ));
        }
        match run.execution_backend {
            TeamBenchExecutionBackend::InProcessToolBoundary => {
                if run.runtime_manifest_digest.is_some() {
                    report.qualification.fail(format!(
                        "condition {:?} claims a container runtime manifest for an in-process tool-boundary run",
                        run.condition
                    ));
                }
            }
            TeamBenchExecutionBackend::DockerContainerIsolation => {
                // A missing runtime digest blocks only the OS-isolation claim,
                // not native benchmark score eligibility.
            }
            TeamBenchExecutionBackend::Unknown => {
                // Unknown enforcement backend is not evidence of OS isolation.
            }
        }
        if conditions.insert(run.condition, run).is_some() {
            report
                .qualification
                .fail(format!("duplicate TeamBench condition {:?}", run.condition));
        }
        if !run.role_boundary_inventory_complete {
            report.role_boundary_evidence_complete = false;
            report.role_enforcement_eligible = false;
        }

        let mut task_ids = BTreeSet::new();
        for task in &run.tasks {
            report.task_observation_count += 1;
            if !nonempty(&task.task_id) || !task_ids.insert(task.task_id.as_str()) {
                report.qualification.fail(format!(
                    "condition {:?} has empty or duplicate task ID",
                    run.condition
                ));
                continue;
            }
            if !eligible.contains(task.task_id.as_str()) {
                report.qualification.fail(format!(
                    "condition {:?} contains task {} outside the eligible manifest",
                    run.condition, task.task_id
                ));
                continue;
            }
            qualify_task(task, &run.roles, &mut report);
            match &task.resource_usage {
                Some(usage) if valid_digest(&usage.evidence_digest) && usage.wall_clock_ms > 0 => {
                    let totals = resource_totals.entry(run.condition).or_default();
                    totals.input_tokens = totals.input_tokens.saturating_add(usage.input_tokens);
                    totals.output_tokens = totals.output_tokens.saturating_add(usage.output_tokens);
                    totals.wall_clock_ms = totals.wall_clock_ms.saturating_add(usage.wall_clock_ms);
                    totals.provider_cost_micro_usd = totals
                        .provider_cost_micro_usd
                        .saturating_add(usage.provider_cost_micro_usd);
                    totals.tool_call_count =
                        totals.tool_call_count.saturating_add(usage.tool_call_count);
                    totals.inter_agent_message_count = totals
                        .inter_agent_message_count
                        .saturating_add(usage.inter_agent_message_count);
                    totals.task_count += 1;
                }
                _ => {
                    report.resource_usage_complete = false;
                    report.resource_usage_missing_or_invalid_count += 1;
                }
            }
            if task.state == TeamBenchTaskState::Evaluated {
                if let Some(score) = task.native_score_micros.filter(|score| *score <= 1_000_000) {
                    let totals = score_totals.entry(run.condition).or_default();
                    totals.0 += u64::from(score);
                    totals.1 += 1;
                    if task_scores
                        .insert((task.task_id.clone(), run.condition), score)
                        .is_some()
                    {
                        report.qualification.fail(format!(
                            "task {} has duplicate score for condition {:?}",
                            task.task_id, run.condition
                        ));
                    }
                }
            }
        }

        for task_id in &eligible {
            if !task_ids.contains(task_id) {
                report.qualification.indeterminate(format!(
                    "condition {:?} is missing eligible task {}",
                    run.condition, task_id
                ));
                report.resource_usage_complete = false;
                report.resource_usage_missing_or_invalid_count += 1;
            }
        }
    }

    for condition in TeamBenchCondition::all() {
        if !conditions.contains_key(&condition) {
            report
                .qualification
                .indeterminate(format!("required TeamBench condition {condition:?} missing"));
            report.resource_usage_complete = false;
            report.resource_usage_missing_or_invalid_count += suite.eligible_task_ids.len();
        }
    }

    // Denominators use all evaluated native-grader outcomes in verifier-enabled
    // conditions. Missing/no-verdict rows remain separately visible.
    for run in &suite.condition_runs {
        if !run.roles.contains(&TeamBenchRole::Verifier) {
            continue;
        }
        for task in &run.tasks {
            if task.state != TeamBenchTaskState::Evaluated {
                continue;
            }
            match task.native_grader_disposition {
                Some(TeamBenchGraderDisposition::Fail) => {
                    report.false_accept_denominator += 1;
                }
                Some(TeamBenchGraderDisposition::Pass) => {
                    report.false_reject_denominator += 1;
                }
                Some(TeamBenchGraderDisposition::Unknown) | None => {}
            }
        }
    }

    let complete_model_suite = suite.run_mode == TeamBenchRunMode::ModelEvaluation
        && report.qualification.status == QualificationStatus::Pass
        && TeamBenchCondition::all()
            .iter()
            .all(|condition| conditions.contains_key(condition));

    if complete_model_suite {
        if report.resource_usage_complete
            && TeamBenchCondition::all().iter().all(|condition| {
                resource_totals
                    .get(condition)
                    .is_some_and(|totals| totals.task_count == suite.eligible_task_ids.len())
            })
        {
            report.resource_totals_by_condition = resource_totals;
        } else {
            report.resource_usage_complete = false;
            report.resource_usage_missing_or_invalid_count = report
                .resource_usage_missing_or_invalid_count
                .max(1);
        }

        for (condition, (sum, count)) in score_totals {
            if count > 0 {
                report
                    .mean_score_micros_by_condition
                    .insert(condition, (sum / count) as u32);
            }
        }

        let mut tni_values = Vec::new();
        let mut uplifts = Vec::new();
        let mut efficiencies = Vec::new();
        let mut planning_values = Vec::new();
        let mut verification_values = Vec::new();

        for task_id in &suite.eligible_task_ids {
            let score = |condition| {
                task_scores
                    .get(&(task_id.clone(), condition))
                    .copied()
                    .map(i64::from)
            };
            let (Some(oracle), Some(restricted), Some(full), Some(no_plan), Some(no_verify)) = (
                score(TeamBenchCondition::Oracle),
                score(TeamBenchCondition::Restricted),
                score(TeamBenchCondition::Full),
                score(TeamBenchCondition::TeamNoPlan),
                score(TeamBenchCondition::TeamNoVerify),
            ) else {
                // Complete model qualification should guarantee this. Preserve
                // the claim ceiling if a future adapter change breaks it.
                report.qualification.indeterminate(format!(
                    "task {task_id} lacks a score for one or more official conditions"
                ));
                continue;
            };

            let gap = oracle - restricted;
            let uplift = full - restricted;
            uplifts.push(uplift);
            if uplift > 10_000 {
                report.team_helps_count += 1;
            }

            // Match harness/compute_tni.py: omit TNI when |gap| < .05,
            // clamp each task to [-2, 2], and aggregate only positive gaps.
            if gap.unsigned_abs() >= 50_000 {
                let raw = uplift.saturating_mul(1_000_000) / gap;
                let clamped = raw.clamp(-2_000_000, 2_000_000);
                if gap > 50_000 {
                    tni_values.push(clamped);
                }
            }

            // Match upstream's collaboration-efficiency zero rule.
            let efficiency = if oracle < 10_000 {
                0
            } else {
                uplift.saturating_mul(1_000_000) / oracle
            };
            efficiencies.push(efficiency);

            // Match upstream NaN convention: omit only where both values are zero.
            if full != 0 || no_plan != 0 {
                planning_values.push(full - no_plan);
            }
            if full != 0 || no_verify != 0 {
                verification_values.push(full - no_verify);
            }
        }

        report.valid_tni_task_count = tni_values.len();
        report.average_tni_micros = mean_i64(&tni_values);
        report.average_team_uplift_micros = mean_i64(&uplifts);
        report.average_collaboration_efficiency_micros = mean_i64(&efficiencies);
        report.planning_value_task_count = planning_values.len();
        report.average_planning_value_micros = mean_i64(&planning_values);
        report.verification_value_task_count = verification_values.len();
        report.average_verification_value_micros = mean_i64(&verification_values);
    } else if suite.run_mode == TeamBenchRunMode::MockProviderSmoke {
        report.qualification.reasons.push(
            "mock-provider smoke is integration evidence only; no model-performance score or TNI is eligible".into(),
        );
    }

    report.benchmark_score_eligible = complete_model_suite
        && report.qualification.status == QualificationStatus::Pass;
    report.os_isolation_eligible = suite.run_mode == TeamBenchRunMode::ModelEvaluation
        && report.condition_count == TeamBenchCondition::all().len()
        && conditions.len() == TeamBenchCondition::all().len()
        && report.role_boundary_evidence_complete
        && report.role_enforcement_eligible
        && TeamBenchCondition::all().iter().all(|condition| {
            conditions.get(condition).is_some_and(|run| {
                run.execution_backend == TeamBenchExecutionBackend::DockerContainerIsolation
                    && valid_optional_digest(run.runtime_manifest_digest.as_deref())
            })
        });

    report.reasons = report.qualification.reasons.clone();
    report
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

    fn task(
        task_id: &str,
        verifier_present: bool,
        actor_role: TeamBenchRole,
    ) -> TeamBenchTaskObservation {
        TeamBenchTaskObservation {
            task_id: task_id.into(),
            state: TeamBenchTaskState::Evaluated,
            raw_output_digest: Some(D4.into()),
            native_grader_output_digest: Some(D5.into()),
            native_grader_disposition: Some(TeamBenchGraderDisposition::Pass),
            native_score_micros: Some(750_000),
            resource_usage: Some(TeamBenchResourceUsage {
                input_tokens: 100,
                output_tokens: 50,
                wall_clock_ms: 1_000,
                provider_cost_micro_usd: 0,
                tool_call_count: 3,
                inter_agent_message_count: 1,
                evidence_digest: D6.into(),
            }),
            verifier_disposition: verifier_present.then_some(TeamBenchVerifierDisposition::Accept),
            role_boundary_events: vec![TeamBenchRoleBoundaryEvent {
                event_id: format!("event-{task_id}"),
                actor_role,
                boundary_id: if actor_role == TeamBenchRole::Verifier {
                    "verifier-write-workspace".into()
                } else {
                    "task-scope-boundary".into()
                },
                expected_allowed: false,
                observed: TeamBenchBoundaryObservation::Blocked,
                evidence_digest: D6.into(),
            }],
        }
    }

    fn run(condition: TeamBenchCondition) -> TeamBenchConditionRun {
        let roles = expected_roles(condition);
        let verifier_present = roles.contains(&TeamBenchRole::Verifier);
        let actor_role = if verifier_present {
            TeamBenchRole::Verifier
        } else {
            *roles.iter().next().unwrap()
        };
        TeamBenchConditionRun {
            run_id: format!("run-{condition:?}"),
            condition,
            system_config_digest: D1.into(),
            role_profile_digest: D2.into(),
            raw_result_manifest_digest: D3.into(),
            agent_count: roles.len() as u32,
            execution_backend: TeamBenchExecutionBackend::InProcessToolBoundary,
            runtime_manifest_digest: None,
            roles: roles.into_iter().collect(),
            role_boundary_inventory_complete: true,
            tasks: vec![
                task("task-1", verifier_present, actor_role),
                task("task-2", verifier_present, actor_role),
            ],
        }
    }

    fn suite() -> TeamBenchSuite {
        let mut input = TeamBenchSuite {
            suite_id: "teambench-smoke-001".into(),
            run_mode: TeamBenchRunMode::ModelEvaluation,
            upstream_repository: UPSTREAM_REPOSITORY.into(),
            upstream_commit: "d185aef1916fd86a9ba554d581fd256319a973af".into(),
            benchmark_version: "2.1".into(),
            dataset_manifest_digest: D1.into(),
            task_manifest_digest: String::new(),
            task_manifest_path: UPSTREAM_TASK_MANIFEST_PATH.into(),
            task_manifest_blob_sha: UPSTREAM_TASK_MANIFEST_BLOB_SHA.into(),
            re_curation_notice_path: UPSTREAM_README_PATH.into(),
            re_curation_notice_blob_sha: UPSTREAM_README_BLOB_SHA.into(),
            native_grader_digest: D3.into(),
            excluded_task_manifest_digest: String::new(),
            eligible_task_ids: vec!["task-1".into(), "task-2".into()],
            excluded_tasks: vec![TeamBenchExcludedTask {
                task_id: "task-recuration-1".into(),
                reason: "upstream re-curation pending".into(),
                evidence_digest: D5.into(),
            }],
            model_snapshot_digest: D1.into(),
            tokenizer_digest: D2.into(),
            prompt_config_digest: D3.into(),
            toolset_digest: D4.into(),
            decoding_config_digest: D5.into(),
            seed_policy_id: "fixed-seed-v1".into(),
            seed: 17,
            per_task_time_budget_ms: 120_000,
            per_task_token_budget: 8_000,
            per_task_compute_budget_millijoules: 500_000,
            condition_runs: TeamBenchCondition::all().iter().copied().map(run).collect(),
        };
        input.excluded_task_manifest_digest =
            compute_teambench_excluded_manifest_digest(&input.excluded_tasks).unwrap();
        input.task_manifest_digest = compute_teambench_task_manifest_digest(
            &input.upstream_commit,
            &input.task_manifest_path,
            &input.task_manifest_blob_sha,
            &input.re_curation_notice_path,
            &input.re_curation_notice_blob_sha,
            &input.eligible_task_ids,
            &input.excluded_tasks,
        )
        .unwrap();
        input
    }

    #[test]
    fn canonical_task_manifest_is_order_independent_and_detects_mutation() {
        let mut input = suite();
        let original_task_digest = input.task_manifest_digest.clone();
        let original_exclusion_digest = input.excluded_task_manifest_digest.clone();

        input.eligible_task_ids.reverse();
        input.excluded_tasks.reverse();
        assert_eq!(
            compute_teambench_excluded_manifest_digest(&input.excluded_tasks).unwrap(),
            original_exclusion_digest
        );
        assert_eq!(
            compute_teambench_task_manifest_digest(
                &input.upstream_commit,
                &input.task_manifest_path,
                &input.task_manifest_blob_sha,
                &input.re_curation_notice_path,
                &input.re_curation_notice_blob_sha,
                &input.eligible_task_ids,
                &input.excluded_tasks,
            )
            .unwrap(),
            original_task_digest
        );

        input.eligible_task_ids[0] = "mutated-task-id".into();
        assert_eq!(
            qualify_teambench_suite(&input).qualification.status,
            QualificationStatus::Fail
        );

        let mut input = suite();
        input.excluded_tasks[0].reason = "silently changed exclusion reason".into();
        assert_eq!(
            qualify_teambench_suite(&input).qualification.status,
            QualificationStatus::Fail
        );

        let mut input = suite();
        input.upstream_commit = "a".repeat(40);
        assert_eq!(
            qualify_teambench_suite(&input).qualification.status,
            QualificationStatus::Fail
        );
    }

    #[test]
    fn complete_pinned_model_suite_is_score_eligible() {
        let report = qualify_teambench_suite(&suite());
        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        assert!(report.benchmark_score_eligible);
        assert_eq!(report.condition_count, 5);
        assert_eq!(report.eligible_task_count, 2);
        assert_eq!(report.mean_score_micros_by_condition.len(), 5);
        assert!(report.resource_usage_complete);
        assert_eq!(report.resource_totals_by_condition[&TeamBenchCondition::Full].input_tokens, 200);
        assert_eq!(report.resource_totals_by_condition[&TeamBenchCondition::Full].output_tokens, 100);
        assert_eq!(report.resource_totals_by_condition[&TeamBenchCondition::Full].wall_clock_ms, 2_000);
        assert_eq!(report.average_tni_micros, None);
        assert_eq!(report.valid_tni_task_count, 0);
        assert_eq!(report.average_team_uplift_micros, Some(0));
        assert_eq!(report.average_collaboration_efficiency_micros, Some(0));
        assert_eq!(report.planning_value_task_count, 0);
        assert_eq!(report.verification_value_task_count, 0);
    }

    #[test]
    fn mock_provider_smoke_can_pass_integration_without_becoming_a_score() {
        let mut input = suite();
        input.run_mode = TeamBenchRunMode::MockProviderSmoke;
        let report = qualify_teambench_suite(&input);
        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        assert!(!report.benchmark_score_eligible);
        assert!(!report.os_isolation_eligible);
        assert!(report.mean_score_micros_by_condition.is_empty());
        assert!(report.average_tni_micros.is_none());
        assert!(report.average_team_uplift_micros.is_none());
        assert!(report.resource_totals_by_condition.is_empty());
    }

    #[test]
    fn resource_evidence_is_not_inferred_from_missing_task_telemetry() {
        let mut input = suite();
        input.condition_runs[0].tasks[0].resource_usage = None;
        let report = qualify_teambench_suite(&input);
        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        assert!(report.benchmark_score_eligible);
        assert!(!report.resource_usage_complete);
        assert_eq!(report.resource_usage_missing_or_invalid_count, 1);
        assert!(report.resource_totals_by_condition.is_empty());
    }

    #[test]
    fn model_score_and_os_isolation_are_independent_qualifications() {
        let mut input = suite();
        let report = qualify_teambench_suite(&input);
        assert!(report.benchmark_score_eligible);
        assert!(!report.os_isolation_eligible);

        for run in &mut input.condition_runs {
            run.execution_backend = TeamBenchExecutionBackend::DockerContainerIsolation;
            run.runtime_manifest_digest = None;
        }
        let report = qualify_teambench_suite(&input);
        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        assert!(report.benchmark_score_eligible);
        assert!(!report.os_isolation_eligible);

        for run in &mut input.condition_runs {
            run.runtime_manifest_digest = Some(D6.into());
        }
        let report = qualify_teambench_suite(&input);
        assert!(report.benchmark_score_eligible);
        assert!(report.os_isolation_eligible);
    }

    #[test]
    fn unknown_runtime_backend_is_indeterminate_not_os_isolated() {
        let mut input = suite();
        input.condition_runs[0].execution_backend = TeamBenchExecutionBackend::Unknown;
        let report = qualify_teambench_suite(&input);
        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        assert!(report.benchmark_score_eligible);
        assert!(!report.os_isolation_eligible);
    }

    #[test]
    fn native_tni_and_component_values_match_upstream_formula() {
        let mut input = suite();
        let values = [
            (800_000, 200_000, 700_000, 500_000, 400_000),
            (800_000, 300_000, 500_000, 400_000, 300_000),
        ];
        for run in &mut input.condition_runs {
            for (task_index, task) in run.tasks.iter_mut().enumerate() {
                let (oracle, restricted, full, no_plan, no_verify) = values[task_index];
                task.native_score_micros = Some(match run.condition {
                    TeamBenchCondition::Oracle => oracle,
                    TeamBenchCondition::Restricted => restricted,
                    TeamBenchCondition::Full => full,
                    TeamBenchCondition::TeamNoPlan => no_plan,
                    TeamBenchCondition::TeamNoVerify => no_verify,
                });
            }
        }
        let report = qualify_teambench_suite(&input);
        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        assert_eq!(report.average_tni_micros, Some(616_666));
        assert_eq!(report.valid_tni_task_count, 2);
        assert_eq!(report.average_team_uplift_micros, Some(350_000));
        assert_eq!(report.team_helps_count, 2);
        assert_eq!(report.average_collaboration_efficiency_micros, Some(437_500));
        assert_eq!(report.average_planning_value_micros, Some(150_000));
        assert_eq!(report.planning_value_task_count, 2);
        assert_eq!(report.average_verification_value_micros, Some(250_000));
        assert_eq!(report.verification_value_task_count, 2);
    }

    #[test]
    fn weak_or_negative_oracle_restricted_gaps_are_excluded_from_aggregate_tni() {
        let mut input = suite();
        for run in &mut input.condition_runs {
            for task in &mut run.tasks {
                task.native_score_micros = Some(match run.condition {
                    TeamBenchCondition::Oracle => 300_000,
                    TeamBenchCondition::Restricted => 280_000,
                    TeamBenchCondition::Full => 500_000,
                    TeamBenchCondition::TeamNoPlan => 400_000,
                    TeamBenchCondition::TeamNoVerify => 450_000,
                });
            }
        }
        let report = qualify_teambench_suite(&input);
        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        assert_eq!(report.valid_tni_task_count, 0);
        assert_eq!(report.average_tni_micros, None);
        assert_eq!(report.average_team_uplift_micros, Some(220_000));
    }

    #[test]
    fn missing_condition_is_indeterminate_not_a_partial_leaderboard_score() {
        let mut input = suite();
        input.condition_runs.pop();
        let report = qualify_teambench_suite(&input);
        assert_eq!(report.qualification.status, QualificationStatus::Indeterminate);
        assert!(!report.benchmark_score_eligible);
        assert!(!report.resource_usage_complete);
    }

    #[test]
    fn duplicate_condition_fails() {
        let mut input = suite();
        input.condition_runs.push(input.condition_runs[0].clone());
        assert_eq!(
            qualify_teambench_suite(&input).qualification.status,
            QualificationStatus::Fail
        );
    }

    #[test]
    fn wrong_upstream_repository_or_malformed_commit_fails() {
        let mut input = suite();
        input.upstream_repository = "https://github.com/some-fork/TeamBench".into();
        assert_eq!(
            qualify_teambench_suite(&input).qualification.status,
            QualificationStatus::Fail
        );

        let mut input = suite();
        input.upstream_commit = "latest".into();
        assert_eq!(
            qualify_teambench_suite(&input).qualification.status,
            QualificationStatus::Fail
        );
    }

    #[test]
    fn eligible_excluded_overlap_fails() {
        let mut input = suite();
        input.excluded_tasks.push(TeamBenchExcludedTask {
            task_id: "task-1".into(),
            reason: "duplicate disposition".into(),
            evidence_digest: D6.into(),
        });
        assert_eq!(
            qualify_teambench_suite(&input).qualification.status,
            QualificationStatus::Fail
        );
    }

    #[test]
    fn missing_task_in_one_condition_is_indeterminate() {
        let mut input = suite();
        input.condition_runs[0].tasks.pop();
        assert_eq!(
            qualify_teambench_suite(&input).qualification.status,
            QualificationStatus::Indeterminate
        );
    }

    #[test]
    fn duplicate_or_unlisted_task_fails() {
        let mut input = suite();
        let duplicate_task = input.condition_runs[0].tasks[0].clone();
        input.condition_runs[0].tasks.push(duplicate_task);
        assert_eq!(
            qualify_teambench_suite(&input).qualification.status,
            QualificationStatus::Fail
        );

        let mut input = suite();
        input.condition_runs[0].tasks[0].task_id = "unknown-task".into();
        assert_eq!(
            qualify_teambench_suite(&input).qualification.status,
            QualificationStatus::Fail
        );
    }

    #[test]
    fn false_accepts_and_false_rejects_have_distinct_denominators() {
        let mut input = suite();
        let run = input.condition_runs.iter_mut()
            .find(|r| r.condition == TeamBenchCondition::Full).unwrap();
        run.tasks[0].native_grader_disposition = Some(TeamBenchGraderDisposition::Fail);
        run.tasks[0].verifier_disposition = Some(TeamBenchVerifierDisposition::Accept);
        run.tasks[1].native_grader_disposition = Some(TeamBenchGraderDisposition::Pass);
        run.tasks[1].verifier_disposition = Some(TeamBenchVerifierDisposition::Reject);
        let report = qualify_teambench_suite(&input);
        assert_eq!(report.false_accept_count, 1);
        assert_eq!(report.false_accept_denominator, 1);
        assert_eq!(report.false_reject_count, 1);
        assert_eq!(report.false_reject_denominator, 3);
    }

    #[test]
    fn no_verdict_is_counted_separately_not_as_a_pass() {
        let mut input = suite();
        let run = input.condition_runs.iter_mut()
            .find(|r| r.condition == TeamBenchCondition::Full).unwrap();
        run.tasks[0].native_grader_disposition = Some(TeamBenchGraderDisposition::Fail);
        run.tasks[0].verifier_disposition = Some(TeamBenchVerifierDisposition::NoVerdict);
        let report = qualify_teambench_suite(&input);
        assert_eq!(report.verifier_no_verdict_count, 1);
        assert_eq!(report.false_accept_count, 0);
        assert_eq!(report.false_accept_denominator, 1);
    }

    #[test]
    fn grader_unknown_and_missing_output_are_indeterminate() {
        let mut input = suite();
        let run = input.condition_runs.iter_mut()
            .find(|r| r.condition == TeamBenchCondition::Full).unwrap();
        run.tasks[0].native_grader_disposition = Some(TeamBenchGraderDisposition::Unknown);
        run.tasks[0].native_grader_output_digest = None;
        let report = qualify_teambench_suite(&input);
        assert_eq!(report.qualification.status, QualificationStatus::Indeterminate);
        assert_eq!(report.grader_unknown_count, 1);
    }

    #[test]
    fn observed_allow_on_a_deny_boundary_fails() {
        let mut input = suite();
        let event = &mut input.condition_runs[0].tasks[0].role_boundary_events[0];
        event.observed = TeamBenchBoundaryObservation::Allowed;
        let report = qualify_teambench_suite(&input);
        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        assert!(report.benchmark_score_eligible);
        assert!(!report.role_enforcement_eligible);
        assert_eq!(report.role_boundary_violation_count, 1);
    }

    #[test]
    fn malformed_role_observation_does_not_rewrite_native_benchmark_score() {
        let mut input = suite();
        let run = input.condition_runs.iter_mut()
            .find(|r| r.condition == TeamBenchCondition::Full).unwrap();
        run.tasks[0].role_boundary_events[0].evidence_digest = "bad".into();
        let report = qualify_teambench_suite(&input);
        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        assert!(report.benchmark_score_eligible);
        assert!(!report.role_boundary_evidence_complete);
        assert!(!report.role_enforcement_eligible);
    }

    #[test]
    fn unknown_boundary_inventory_is_indeterminate() {
        let mut input = suite();
        input.condition_runs[0].role_boundary_inventory_complete = false;
        input.condition_runs[0].tasks[0].role_boundary_events[0].observed =
            TeamBenchBoundaryObservation::Unknown;
        let report = qualify_teambench_suite(&input);
        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        assert!(report.benchmark_score_eligible);
        assert!(!report.role_boundary_evidence_complete);
        assert!(!report.role_enforcement_eligible);
        assert!(report.role_boundary_unknown_count > 0);
    }

    #[test]
    fn infrastructure_errors_are_not_counted_as_task_failures() {
        let mut input = suite();
        input.condition_runs[0].tasks[0] = TeamBenchTaskObservation {
            task_id: "task-1".into(),
            state: TeamBenchTaskState::InfrastructureError,
            raw_output_digest: None,
            native_grader_output_digest: None,
            native_grader_disposition: None,
            native_score_micros: None,
            resource_usage: None,
            verifier_disposition: None,
            role_boundary_events: vec![],
        };
        let report = qualify_teambench_suite(&input);
        assert_eq!(report.qualification.status, QualificationStatus::Indeterminate);
        assert_eq!(report.infrastructure_error_count, 1);
        assert_eq!(report.grader_pass_count + report.grader_fail_count, 9);
    }

    #[test]
    fn verifier_absent_condition_cannot_supply_verifier_judgements() {
        let mut input = suite();
        let run = input.condition_runs.iter_mut()
            .find(|r| r.condition == TeamBenchCondition::TeamNoVerify).unwrap();
        run.tasks[0].verifier_disposition = Some(TeamBenchVerifierDisposition::Accept);
        assert_eq!(
            qualify_teambench_suite(&input).qualification.status,
            QualificationStatus::Fail
        );
    }

    #[test]
    fn missing_verifier_judgement_does_not_invalidate_native_score() {
        let mut input = suite();
        let run = input.condition_runs.iter_mut()
            .find(|r| r.condition == TeamBenchCondition::Full).unwrap();
        run.tasks[0].verifier_disposition = None;
        let report = qualify_teambench_suite(&input);
        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        assert!(report.benchmark_score_eligible);
        assert!(!report.verifier_evidence_complete);
        assert_eq!(report.verifier_evidence_missing_count, 1);
    }
}
