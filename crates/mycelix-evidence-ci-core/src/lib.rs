#![deny(unsafe_code)]

//! Pure qualification liveness, conjunction, and bounded runner-failover semantics.
//!
//! Provider-specific facts such as GitHub runner IDs, runner names, and timestamps stay
//! outside this crate. The theorem kernel consumes only normalized job status, registered
//! theorem-step execution, dependency state, and explicitly classified failure semantics.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const MAX_TEXT_BYTES_V1: usize = 256;
pub const MAX_REQUIRED_JOBS_V1: usize = 128;
pub const MAX_OBSERVED_JOBS_V1: usize = 512;
pub const MAX_PROFILE_ITEMS_V1: usize = 128;

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct EvidenceId(String);

impl EvidenceId {
    pub fn new(value: impl Into<String>) -> Result<Self, EvidenceError> {
        let value = value.into();
        if value.is_empty()
            || value.len() > MAX_TEXT_BYTES_V1
            || value.chars().any(char::is_control)
        {
            return Err(EvidenceError::InvalidIdentifier);
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JobStatusV1 {
    Queued,
    InProgress,
    Completed,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JobConclusionV1 {
    Success,
    Failure,
    Cancelled,
    Skipped,
    TimedOut,
    ActionRequired,
    Neutral,
    StartupFailure,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GateExecutionStateV1 {
    NoTheoremStepExecuted,
    SomeTheoremStepsExecuted,
    AllRegisteredTheoremStepsExecuted,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DependencyStateV1 {
    EligibleForRunner,
    WaitingOnRequiredDependency,
    DependencyFailed,
    DependencySkipped,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FailureClassV1 {
    NotApplicable,
    RegisteredTheoremGate,
    RunnerInfrastructureBeforeTheoremGate,
    Unknown,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JobObservationV1 {
    pub job_id: u64,
    pub job_key: EvidenceId,
    pub status: JobStatusV1,
    pub conclusion: Option<JobConclusionV1>,
    pub gate_execution: GateExecutionStateV1,
    pub dependency_state: DependencyStateV1,
    /// Operational metadata only. It never participates in theorem PASS/FAIL.
    pub queue_age_seconds: Option<u64>,
    pub failure_class: FailureClassV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WorkflowRunObservationV1 {
    pub repository_id: u64,
    pub workflow_run_id: u64,
    pub qualification_head: EvidenceId,
    pub workflow_path: EvidenceId,
    pub jobs: Vec<JobObservationV1>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RequiredJobSpecV1 {
    pub job_key: EvidenceId,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RequiredJobManifestV1 {
    pub theorem_id: EvidenceId,
    pub repository_id: u64,
    pub workflow_path: EvidenceId,
    pub qualification_head: EvidenceId,
    pub required_jobs: Vec<RequiredJobSpecV1>,
}

impl RequiredJobManifestV1 {
    pub fn validate(&self) -> Result<(), EvidenceError> {
        if self.required_jobs.is_empty() || self.required_jobs.len() > MAX_REQUIRED_JOBS_V1 {
            return Err(EvidenceError::InvalidRequiredJobCount);
        }
        reject_duplicate_job_specs(&self.required_jobs)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RunLivenessV1 {
    AllRequiredJobsNoStart,
    RequiredDependencyBlocked,
    PartiallyExecutedAwaitingRequiredJob,
    RequiredJobExecuting,
    CompletedConjunctivePass,
    CompletedConjunctiveFail,
    InfrastructureInterrupted,
    Indeterminate,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConjunctiveVerdictV1 {
    Pass,
    SemanticFail,
    Incomplete,
    InfrastructureInterrupted,
    Indeterminate,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ConjunctiveQualificationReceiptV1 {
    pub repository_id: u64,
    pub workflow_run_id: u64,
    pub workflow_path: EvidenceId,
    pub qualification_head: EvidenceId,
    pub theorem_id: EvidenceId,
    pub liveness: RunLivenessV1,
    pub verdict: ConjunctiveVerdictV1,
    pub required_job_ids: Vec<u64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NetworkPolicyV1 {
    Offline,
    ReadOnlyNetwork,
    NetworkAllowed,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CachePolicyV1 {
    Disabled,
    ReadOnly,
    ReadWrite,
    Unknown,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RunnerAdapterProfileV1 {
    pub id: EvidenceId,
    pub requested_runner_selector: EvidenceId,
    pub os_family: EvidenceId,
    pub architecture: EvidenceId,
    pub network_policy: NetworkPolicyV1,
    pub cache_policy: CachePolicyV1,
    pub action_pins: Vec<EvidenceId>,
    pub required_tools: Vec<EvidenceId>,
}

impl RunnerAdapterProfileV1 {
    pub fn validate(&self) -> Result<(), EvidenceError> {
        if self.action_pins.len() > MAX_PROFILE_ITEMS_V1
            || self.required_tools.len() > MAX_PROFILE_ITEMS_V1
        {
            return Err(EvidenceError::ProfileTooLarge);
        }
        reject_duplicate_ids(&self.action_pins)?;
        reject_duplicate_ids(&self.required_tools)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualificationHarnessFingerprintV1 {
    pub theorem_subject: EvidenceId,
    pub theorem_predicates: EvidenceId,
    pub command_set: EvidenceId,
    pub canonical_oracle: EvidenceId,
    pub object_identity_profile: EvidenceId,
    pub claim_boundary: EvidenceId,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FailoverPolicyV1 {
    pub minimum_no_start_age_seconds: u64,
    pub maximum_concurrent_attempts: u16,
    pub eligible_source_runner_ids: Vec<EvidenceId>,
    pub eligible_target_runner_ids: Vec<EvidenceId>,
    pub allow_os_family_change: bool,
    pub allow_architecture_change: bool,
    pub allow_cache_policy_change: bool,
}

impl FailoverPolicyV1 {
    pub fn validate(&self) -> Result<(), EvidenceError> {
        if self.maximum_concurrent_attempts == 0
            || self.eligible_source_runner_ids.is_empty()
            || self.eligible_target_runner_ids.is_empty()
            || self.eligible_source_runner_ids.len() > MAX_PROFILE_ITEMS_V1
            || self.eligible_target_runner_ids.len() > MAX_PROFILE_ITEMS_V1
        {
            return Err(EvidenceError::InvalidFailoverPolicy);
        }
        reject_duplicate_ids(&self.eligible_source_runner_ids)?;
        reject_duplicate_ids(&self.eligible_target_runner_ids)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WholeTheoremFailoverRequestV1 {
    pub source_harness: QualificationHarnessFingerprintV1,
    pub target_harness: QualificationHarnessFingerprintV1,
    pub source_runner: RunnerAdapterProfileV1,
    pub target_runner: RunnerAdapterProfileV1,
    pub source_liveness: RunLivenessV1,
    /// Age of the blocked required theorem job, not aggregate workflow age.
    pub blocked_required_job_age_seconds: Option<u64>,
    pub active_attempts: u16,
    pub policy: FailoverPolicyV1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FailoverEligibilityV1 {
    Eligible,
    Ineligible(FailoverBlockV1),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FailoverBlockV1 {
    SourceStateNotEligible,
    TheoremHarnessChanged,
    InvalidSourceRunnerProfile,
    InvalidTargetRunnerProfile,
    InvalidFailoverPolicy,
    SourceRunnerNotAdmitted,
    TargetRunnerNotAdmitted,
    AttemptLimitReached,
    MissingQueueAge,
    QueueAgeBelowThreshold,
    NetworkPolicyChanged,
    ActionPinsChanged,
    RequiredToolsChanged,
    OsFamilyChangeNotAdmitted,
    ArchitectureChangeNotAdmitted,
    CachePolicyChangeNotAdmitted,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EvidenceError {
    InvalidIdentifier,
    InvalidRequiredJobCount,
    DuplicateRequiredJob,
    DuplicateObservedJob,
    MissingRequiredJob,
    RepositoryMismatch,
    WorkflowPathMismatch,
    QualificationHeadMismatch,
    TooManyObservedJobs,
    InvalidJobObservation,
    ProfileTooLarge,
    DuplicateProfileItem,
    InvalidFailoverPolicy,
}

impl fmt::Display for EvidenceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidIdentifier => "identifier is empty, too long, or contains control characters",
            Self::InvalidRequiredJobCount => "required theorem job count is outside the V1 bound",
            Self::DuplicateRequiredJob => "required theorem job manifest contains a duplicate job key",
            Self::DuplicateObservedJob => "workflow observation contains a duplicate job key",
            Self::MissingRequiredJob => "workflow observation is missing a registered required theorem job",
            Self::RepositoryMismatch => "manifest and workflow observation bind different repositories",
            Self::WorkflowPathMismatch => "manifest and workflow observation bind different workflow paths",
            Self::QualificationHeadMismatch => "manifest and workflow observation bind different qualifier heads",
            Self::TooManyObservedJobs => "workflow observation exceeds the V1 job bound",
            Self::InvalidJobObservation => "job observation contains an impossible or unsupported V1 state",
            Self::ProfileTooLarge => "runner adapter profile exceeds the V1 item bound",
            Self::DuplicateProfileItem => "profile contains a duplicate item",
            Self::InvalidFailoverPolicy => "failover policy is empty, oversized, or has an invalid attempt bound",
        };
        f.write_str(message)
    }
}

impl std::error::Error for EvidenceError {}

pub fn classify_liveness_v1(
    manifest: &RequiredJobManifestV1,
    observation: &WorkflowRunObservationV1,
) -> Result<RunLivenessV1, EvidenceError> {
    manifest.validate()?;
    validate_observation_v1(observation)?;
    bind_observation_to_manifest(manifest, observation)?;

    let indexed = index_jobs(&observation.jobs)?;
    let required = required_jobs(manifest, &indexed)?;

    if required.iter().all(|job| completed_success(job)) {
        return Ok(RunLivenessV1::CompletedConjunctivePass);
    }
    if required.iter().any(|job| semantic_gate_failed(job)) {
        return Ok(RunLivenessV1::CompletedConjunctiveFail);
    }
    if required.iter().any(|job| infrastructure_failed(job)) {
        return Ok(RunLivenessV1::InfrastructureInterrupted);
    }
    if required.iter().any(|job| dependency_blocked(job)) {
        return Ok(RunLivenessV1::RequiredDependencyBlocked);
    }
    if required
        .iter()
        .any(|job| matches!(job.status, JobStatusV1::InProgress))
    {
        return Ok(RunLivenessV1::RequiredJobExecuting);
    }

    let any_executed = required.iter().any(|job| {
        matches!(
            job.gate_execution,
            GateExecutionStateV1::SomeTheoremStepsExecuted
                | GateExecutionStateV1::AllRegisteredTheoremStepsExecuted
        ) || matches!(job.status, JobStatusV1::Completed)
    });
    let any_eligible_no_start = required.iter().any(|job| eligible_queued_no_start(job));

    if any_executed && any_eligible_no_start {
        return Ok(RunLivenessV1::PartiallyExecutedAwaitingRequiredJob);
    }

    if required.iter().all(|job| eligible_queued_no_start(job)) {
        return Ok(RunLivenessV1::AllRequiredJobsNoStart);
    }

    Ok(RunLivenessV1::Indeterminate)
}

pub fn derive_conjunctive_receipt_v1(
    manifest: &RequiredJobManifestV1,
    observation: &WorkflowRunObservationV1,
) -> Result<ConjunctiveQualificationReceiptV1, EvidenceError> {
    let liveness = classify_liveness_v1(manifest, observation)?;
    let verdict = match liveness {
        RunLivenessV1::CompletedConjunctivePass => ConjunctiveVerdictV1::Pass,
        RunLivenessV1::CompletedConjunctiveFail => ConjunctiveVerdictV1::SemanticFail,
        RunLivenessV1::InfrastructureInterrupted => {
            ConjunctiveVerdictV1::InfrastructureInterrupted
        }
        RunLivenessV1::Indeterminate => ConjunctiveVerdictV1::Indeterminate,
        RunLivenessV1::AllRequiredJobsNoStart
        | RunLivenessV1::RequiredDependencyBlocked
        | RunLivenessV1::PartiallyExecutedAwaitingRequiredJob
        | RunLivenessV1::RequiredJobExecuting => ConjunctiveVerdictV1::Incomplete,
    };

    let indexed = index_jobs(&observation.jobs)?;
    let required_job_ids = manifest
        .required_jobs
        .iter()
        .map(|spec| {
            indexed
                .get(&spec.job_key)
                .map(|job| job.job_id)
                .ok_or(EvidenceError::MissingRequiredJob)
        })
        .collect::<Result<Vec<_>, _>>()?;

    Ok(ConjunctiveQualificationReceiptV1 {
        repository_id: observation.repository_id,
        workflow_run_id: observation.workflow_run_id,
        workflow_path: observation.workflow_path.clone(),
        qualification_head: observation.qualification_head.clone(),
        theorem_id: manifest.theorem_id.clone(),
        liveness,
        verdict,
        required_job_ids,
    })
}

pub fn assess_whole_theorem_failover_v1(
    request: &WholeTheoremFailoverRequestV1,
) -> FailoverEligibilityV1 {
    if request.source_runner.validate().is_err() {
        return FailoverEligibilityV1::Ineligible(FailoverBlockV1::InvalidSourceRunnerProfile);
    }
    if request.target_runner.validate().is_err() {
        return FailoverEligibilityV1::Ineligible(FailoverBlockV1::InvalidTargetRunnerProfile);
    }
    if request.policy.validate().is_err() {
        return FailoverEligibilityV1::Ineligible(FailoverBlockV1::InvalidFailoverPolicy);
    }
    if request.active_attempts >= request.policy.maximum_concurrent_attempts {
        return FailoverEligibilityV1::Ineligible(FailoverBlockV1::AttemptLimitReached);
    }
    if request.source_harness != request.target_harness {
        return FailoverEligibilityV1::Ineligible(FailoverBlockV1::TheoremHarnessChanged);
    }
    if !request
        .policy
        .eligible_source_runner_ids
        .contains(&request.source_runner.id)
    {
        return FailoverEligibilityV1::Ineligible(FailoverBlockV1::SourceRunnerNotAdmitted);
    }
    if !request
        .policy
        .eligible_target_runner_ids
        .contains(&request.target_runner.id)
    {
        return FailoverEligibilityV1::Ineligible(FailoverBlockV1::TargetRunnerNotAdmitted);
    }
    if request.source_runner.network_policy != request.target_runner.network_policy {
        return FailoverEligibilityV1::Ineligible(FailoverBlockV1::NetworkPolicyChanged);
    }
    if request.source_runner.action_pins != request.target_runner.action_pins {
        return FailoverEligibilityV1::Ineligible(FailoverBlockV1::ActionPinsChanged);
    }
    if request.source_runner.required_tools != request.target_runner.required_tools {
        return FailoverEligibilityV1::Ineligible(FailoverBlockV1::RequiredToolsChanged);
    }
    if request.source_runner.os_family != request.target_runner.os_family
        && !request.policy.allow_os_family_change
    {
        return FailoverEligibilityV1::Ineligible(FailoverBlockV1::OsFamilyChangeNotAdmitted);
    }
    if request.source_runner.architecture != request.target_runner.architecture
        && !request.policy.allow_architecture_change
    {
        return FailoverEligibilityV1::Ineligible(FailoverBlockV1::ArchitectureChangeNotAdmitted);
    }
    if request.source_runner.cache_policy != request.target_runner.cache_policy
        && !request.policy.allow_cache_policy_change
    {
        return FailoverEligibilityV1::Ineligible(FailoverBlockV1::CachePolicyChangeNotAdmitted);
    }

    match request.source_liveness {
        RunLivenessV1::AllRequiredJobsNoStart
        | RunLivenessV1::PartiallyExecutedAwaitingRequiredJob => {
            let Some(age) = request.blocked_required_job_age_seconds else {
                return FailoverEligibilityV1::Ineligible(FailoverBlockV1::MissingQueueAge);
            };
            if age < request.policy.minimum_no_start_age_seconds {
                return FailoverEligibilityV1::Ineligible(
                    FailoverBlockV1::QueueAgeBelowThreshold,
                );
            }
        }
        RunLivenessV1::InfrastructureInterrupted => {}
        RunLivenessV1::RequiredDependencyBlocked
        | RunLivenessV1::RequiredJobExecuting
        | RunLivenessV1::CompletedConjunctivePass
        | RunLivenessV1::CompletedConjunctiveFail
        | RunLivenessV1::Indeterminate => {
            return FailoverEligibilityV1::Ineligible(FailoverBlockV1::SourceStateNotEligible);
        }
    }

    FailoverEligibilityV1::Eligible
}

fn bind_observation_to_manifest(
    manifest: &RequiredJobManifestV1,
    observation: &WorkflowRunObservationV1,
) -> Result<(), EvidenceError> {
    if manifest.repository_id != observation.repository_id {
        return Err(EvidenceError::RepositoryMismatch);
    }
    if manifest.workflow_path != observation.workflow_path {
        return Err(EvidenceError::WorkflowPathMismatch);
    }
    if manifest.qualification_head != observation.qualification_head {
        return Err(EvidenceError::QualificationHeadMismatch);
    }
    Ok(())
}

fn validate_observation_v1(observation: &WorkflowRunObservationV1) -> Result<(), EvidenceError> {
    if observation.jobs.len() > MAX_OBSERVED_JOBS_V1 {
        return Err(EvidenceError::TooManyObservedJobs);
    }
    index_jobs(&observation.jobs)?;

    for job in &observation.jobs {
        if matches!(job.status, JobStatusV1::Completed) != job.conclusion.is_some() {
            return Err(EvidenceError::InvalidJobObservation);
        }
        if matches!(job.status, JobStatusV1::Queued)
            && matches!(
                job.gate_execution,
                GateExecutionStateV1::SomeTheoremStepsExecuted
                    | GateExecutionStateV1::AllRegisteredTheoremStepsExecuted
            )
        {
            return Err(EvidenceError::InvalidJobObservation);
        }

        match job.failure_class {
            FailureClassV1::RegisteredTheoremGate => {
                if !matches!(job.status, JobStatusV1::Completed)
                    || !matches!(job.conclusion, Some(JobConclusionV1::Failure))
                    || !matches!(
                        job.gate_execution,
                        GateExecutionStateV1::SomeTheoremStepsExecuted
                            | GateExecutionStateV1::AllRegisteredTheoremStepsExecuted
                    )
                {
                    return Err(EvidenceError::InvalidJobObservation);
                }
            }
            FailureClassV1::RunnerInfrastructureBeforeTheoremGate => {
                if !matches!(job.status, JobStatusV1::Completed)
                    || !matches!(
                        job.conclusion,
                        Some(
                            JobConclusionV1::Failure
                                | JobConclusionV1::Cancelled
                                | JobConclusionV1::TimedOut
                                | JobConclusionV1::StartupFailure
                        )
                    )
                    || !matches!(
                        job.gate_execution,
                        GateExecutionStateV1::NoTheoremStepExecuted
                    )
                {
                    return Err(EvidenceError::InvalidJobObservation);
                }
            }
            FailureClassV1::NotApplicable => {
                if matches!(
                    job.conclusion,
                    Some(
                        JobConclusionV1::Failure
                            | JobConclusionV1::TimedOut
                            | JobConclusionV1::StartupFailure
                    )
                ) {
                    return Err(EvidenceError::InvalidJobObservation);
                }
            }
            FailureClassV1::Unknown => {
                if !matches!(job.status, JobStatusV1::Completed) {
                    return Err(EvidenceError::InvalidJobObservation);
                }
            }
        }
    }

    Ok(())
}

fn index_jobs(
    jobs: &[JobObservationV1],
) -> Result<BTreeMap<EvidenceId, &JobObservationV1>, EvidenceError> {
    let mut indexed = BTreeMap::new();
    for job in jobs {
        if indexed.insert(job.job_key.clone(), job).is_some() {
            return Err(EvidenceError::DuplicateObservedJob);
        }
    }
    Ok(indexed)
}

fn required_jobs<'a>(
    manifest: &RequiredJobManifestV1,
    indexed: &'a BTreeMap<EvidenceId, &'a JobObservationV1>,
) -> Result<Vec<&'a JobObservationV1>, EvidenceError> {
    manifest
        .required_jobs
        .iter()
        .map(|spec| {
            indexed
                .get(&spec.job_key)
                .copied()
                .ok_or(EvidenceError::MissingRequiredJob)
        })
        .collect()
}

fn completed_success(job: &JobObservationV1) -> bool {
    matches!(job.status, JobStatusV1::Completed)
        && matches!(job.conclusion, Some(JobConclusionV1::Success))
        && matches!(job.failure_class, FailureClassV1::NotApplicable)
        && matches!(
            job.gate_execution,
            GateExecutionStateV1::AllRegisteredTheoremStepsExecuted
        )
}

fn semantic_gate_failed(job: &JobObservationV1) -> bool {
    matches!(job.status, JobStatusV1::Completed)
        && matches!(job.failure_class, FailureClassV1::RegisteredTheoremGate)
}

fn infrastructure_failed(job: &JobObservationV1) -> bool {
    matches!(job.status, JobStatusV1::Completed)
        && matches!(
            job.failure_class,
            FailureClassV1::RunnerInfrastructureBeforeTheoremGate
        )
}

fn dependency_blocked(job: &JobObservationV1) -> bool {
    matches!(
        job.dependency_state,
        DependencyStateV1::WaitingOnRequiredDependency
            | DependencyStateV1::DependencyFailed
            | DependencyStateV1::DependencySkipped
    )
}

fn eligible_queued_no_start(job: &JobObservationV1) -> bool {
    matches!(job.status, JobStatusV1::Queued)
        && matches!(
            job.gate_execution,
            GateExecutionStateV1::NoTheoremStepExecuted
        )
        && matches!(
            job.dependency_state,
            DependencyStateV1::EligibleForRunner
        )
}

fn reject_duplicate_job_specs(items: &[RequiredJobSpecV1]) -> Result<(), EvidenceError> {
    let mut seen = BTreeSet::new();
    for item in items {
        if !seen.insert(&item.job_key) {
            return Err(EvidenceError::DuplicateRequiredJob);
        }
    }
    Ok(())
}

fn reject_duplicate_ids(items: &[EvidenceId]) -> Result<(), EvidenceError> {
    let mut seen = BTreeSet::new();
    for item in items {
        if !seen.insert(item) {
            return Err(EvidenceError::DuplicateProfileItem);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const REPOSITORY_ID: u64 = 1_176_351_975;

    fn id(value: &str) -> EvidenceId {
        EvidenceId::new(value).expect("static test identifier")
    }

    fn manifest(keys: &[&str]) -> RequiredJobManifestV1 {
        RequiredJobManifestV1 {
            theorem_id: id("THEOREM-1"),
            repository_id: REPOSITORY_ID,
            workflow_path: id(".github/workflows/exact.yml"),
            qualification_head: id("qual-head"),
            required_jobs: keys
                .iter()
                .map(|key| RequiredJobSpecV1 { job_key: id(key) })
                .collect(),
        }
    }

    fn observation(jobs: Vec<JobObservationV1>) -> WorkflowRunObservationV1 {
        WorkflowRunObservationV1 {
            repository_id: REPOSITORY_ID,
            workflow_run_id: 42,
            qualification_head: id("qual-head"),
            workflow_path: id(".github/workflows/exact.yml"),
            jobs,
        }
    }

    fn queued(job_id: u64, key: &str, queue_age_seconds: u64) -> JobObservationV1 {
        JobObservationV1 {
            job_id,
            job_key: id(key),
            status: JobStatusV1::Queued,
            conclusion: None,
            gate_execution: GateExecutionStateV1::NoTheoremStepExecuted,
            dependency_state: DependencyStateV1::EligibleForRunner,
            queue_age_seconds: Some(queue_age_seconds),
            failure_class: FailureClassV1::NotApplicable,
        }
    }

    fn success(job_id: u64, key: &str) -> JobObservationV1 {
        JobObservationV1 {
            job_id,
            job_key: id(key),
            status: JobStatusV1::Completed,
            conclusion: Some(JobConclusionV1::Success),
            gate_execution: GateExecutionStateV1::AllRegisteredTheoremStepsExecuted,
            dependency_state: DependencyStateV1::EligibleForRunner,
            queue_age_seconds: Some(0),
            failure_class: FailureClassV1::NotApplicable,
        }
    }

    #[test]
    fn zero_step_required_job_is_no_start_not_fail() {
        let receipt = derive_conjunctive_receipt_v1(
            &manifest(&["qualify"]),
            &observation(vec![queued(1, "qualify", 86_400)]),
        )
        .expect("valid receipt");
        assert_eq!(receipt.liveness, RunLivenessV1::AllRequiredJobsNoStart);
        assert_eq!(receipt.verdict, ConjunctiveVerdictV1::Incomplete);
    }

    #[test]
    fn queue_age_cannot_change_theorem_verdict() {
        let young = derive_conjunctive_receipt_v1(
            &manifest(&["qualify"]),
            &observation(vec![queued(1, "qualify", 1)]),
        )
        .expect("young receipt");
        let old = derive_conjunctive_receipt_v1(
            &manifest(&["qualify"]),
            &observation(vec![queued(1, "qualify", u64::MAX)]),
        )
        .expect("old receipt");
        assert_eq!(young.liveness, old.liveness);
        assert_eq!(young.verdict, old.verdict);
    }

    #[test]
    fn completed_plus_queued_is_partial_awaiting() {
        let receipt = derive_conjunctive_receipt_v1(
            &manifest(&["oracle", "rust"]),
            &observation(vec![success(1, "oracle"), queued(2, "rust", 100)]),
        )
        .expect("valid receipt");
        assert_eq!(
            receipt.liveness,
            RunLivenessV1::PartiallyExecutedAwaitingRequiredJob
        );
    }

    #[test]
    fn pure_conjunction_closes_without_presentation_job() {
        let run = observation(vec![
            success(1, "full-history"),
            success(2, "shallow-history"),
            queued(3, "presentation-verdict", 10_000),
        ]);
        let receipt = derive_conjunctive_receipt_v1(
            &manifest(&["full-history", "shallow-history"]),
            &run,
        )
        .expect("presentation job is not theorem-critical");
        assert_eq!(receipt.liveness, RunLivenessV1::CompletedConjunctivePass);
        assert_eq!(receipt.verdict, ConjunctiveVerdictV1::Pass);
        assert_eq!(receipt.required_job_ids, vec![1, 2]);
    }

    #[test]
    fn workflow_substitution_is_rejected() {
        let mut run = observation(vec![success(1, "qualify")]);
        run.workflow_path = id(".github/workflows/other.yml");
        assert_eq!(
            derive_conjunctive_receipt_v1(&manifest(&["qualify"]), &run),
            Err(EvidenceError::WorkflowPathMismatch)
        );
    }

    #[test]
    fn repository_substitution_is_rejected() {
        let mut run = observation(vec![success(1, "qualify")]);
        run.repository_id += 1;
        assert_eq!(
            derive_conjunctive_receipt_v1(&manifest(&["qualify"]), &run),
            Err(EvidenceError::RepositoryMismatch)
        );
    }

    #[test]
    fn semantic_gate_failure_is_theorem_fail() {
        let failed = JobObservationV1 {
            job_id: 1,
            job_key: id("qualify"),
            status: JobStatusV1::Completed,
            conclusion: Some(JobConclusionV1::Failure),
            gate_execution: GateExecutionStateV1::SomeTheoremStepsExecuted,
            dependency_state: DependencyStateV1::EligibleForRunner,
            queue_age_seconds: Some(0),
            failure_class: FailureClassV1::RegisteredTheoremGate,
        };
        let receipt = derive_conjunctive_receipt_v1(
            &manifest(&["qualify"]),
            &observation(vec![failed]),
        )
        .expect("valid semantic failure");
        assert_eq!(receipt.liveness, RunLivenessV1::CompletedConjunctiveFail);
        assert_eq!(receipt.verdict, ConjunctiveVerdictV1::SemanticFail);
    }

    #[test]
    fn inconsistent_failure_class_cannot_become_pass() {
        let mut impossible = success(1, "qualify");
        impossible.failure_class = FailureClassV1::RegisteredTheoremGate;
        assert_eq!(
            derive_conjunctive_receipt_v1(
                &manifest(&["qualify"]),
                &observation(vec![impossible])
            ),
            Err(EvidenceError::InvalidJobObservation)
        );
    }

    #[test]
    fn infrastructure_failure_is_not_semantic_red() {
        let failed = JobObservationV1 {
            job_id: 1,
            job_key: id("qualify"),
            status: JobStatusV1::Completed,
            conclusion: Some(JobConclusionV1::Failure),
            gate_execution: GateExecutionStateV1::NoTheoremStepExecuted,
            dependency_state: DependencyStateV1::EligibleForRunner,
            queue_age_seconds: Some(0),
            failure_class: FailureClassV1::RunnerInfrastructureBeforeTheoremGate,
        };
        let receipt = derive_conjunctive_receipt_v1(
            &manifest(&["qualify"]),
            &observation(vec![failed]),
        )
        .expect("valid infrastructure failure");
        assert_eq!(receipt.liveness, RunLivenessV1::InfrastructureInterrupted);
        assert_eq!(
            receipt.verdict,
            ConjunctiveVerdictV1::InfrastructureInterrupted
        );
    }

    #[test]
    fn infrastructure_failure_cannot_claim_theorem_steps() {
        let failed = JobObservationV1 {
            job_id: 1,
            job_key: id("qualify"),
            status: JobStatusV1::Completed,
            conclusion: Some(JobConclusionV1::Failure),
            gate_execution: GateExecutionStateV1::SomeTheoremStepsExecuted,
            dependency_state: DependencyStateV1::EligibleForRunner,
            queue_age_seconds: Some(0),
            failure_class: FailureClassV1::RunnerInfrastructureBeforeTheoremGate,
        };
        assert_eq!(
            derive_conjunctive_receipt_v1(
                &manifest(&["qualify"]),
                &observation(vec![failed])
            ),
            Err(EvidenceError::InvalidJobObservation)
        );
    }

    #[test]
    fn dependency_block_is_not_runner_no_start() {
        let mut blocked = queued(1, "qualify", 1_000);
        blocked.dependency_state = DependencyStateV1::WaitingOnRequiredDependency;
        assert_eq!(
            classify_liveness_v1(&manifest(&["qualify"]), &observation(vec![blocked]))
                .expect("valid observation"),
            RunLivenessV1::RequiredDependencyBlocked
        );
    }

    #[test]
    fn unknown_dependency_state_does_not_claim_runner_no_start() {
        let mut unknown = queued(1, "qualify", 1_000);
        unknown.dependency_state = DependencyStateV1::Unknown;
        assert_eq!(
            classify_liveness_v1(&manifest(&["qualify"]), &observation(vec![unknown]))
                .expect("valid observation"),
            RunLivenessV1::Indeterminate
        );
    }

    #[test]
    fn skipped_required_job_never_counts_as_pass() {
        let skipped = JobObservationV1 {
            job_id: 1,
            job_key: id("qualify"),
            status: JobStatusV1::Completed,
            conclusion: Some(JobConclusionV1::Skipped),
            gate_execution: GateExecutionStateV1::NoTheoremStepExecuted,
            dependency_state: DependencyStateV1::DependencySkipped,
            queue_age_seconds: Some(0),
            failure_class: FailureClassV1::NotApplicable,
        };
        let receipt = derive_conjunctive_receipt_v1(
            &manifest(&["qualify"]),
            &observation(vec![skipped]),
        )
        .expect("valid skipped receipt");
        assert_ne!(receipt.verdict, ConjunctiveVerdictV1::Pass);
    }

    #[test]
    fn in_progress_required_job_is_executing() {
        let running = JobObservationV1 {
            job_id: 1,
            job_key: id("qualify"),
            status: JobStatusV1::InProgress,
            conclusion: None,
            gate_execution: GateExecutionStateV1::NoTheoremStepExecuted,
            dependency_state: DependencyStateV1::EligibleForRunner,
            queue_age_seconds: Some(0),
            failure_class: FailureClassV1::NotApplicable,
        };
        assert_eq!(
            classify_liveness_v1(&manifest(&["qualify"]), &observation(vec![running]))
                .expect("valid observation"),
            RunLivenessV1::RequiredJobExecuting
        );
    }

    #[test]
    fn missing_required_job_fails_receipt_derivation() {
        assert_eq!(
            derive_conjunctive_receipt_v1(
                &manifest(&["oracle", "rust"]),
                &observation(vec![success(1, "oracle")])
            ),
            Err(EvidenceError::MissingRequiredJob)
        );
    }

    #[test]
    fn duplicate_required_job_is_rejected() {
        assert_eq!(
            manifest(&["qualify", "qualify"]).validate(),
            Err(EvidenceError::DuplicateRequiredJob)
        );
    }

    #[test]
    fn duplicate_observed_job_is_rejected() {
        assert_eq!(
            classify_liveness_v1(
                &manifest(&["qualify"]),
                &observation(vec![success(1, "qualify"), success(2, "qualify")])
            ),
            Err(EvidenceError::DuplicateObservedJob)
        );
    }

    #[test]
    fn completed_success_requires_all_registered_gates() {
        let mut incomplete = success(1, "qualify");
        incomplete.gate_execution = GateExecutionStateV1::SomeTheoremStepsExecuted;
        let receipt = derive_conjunctive_receipt_v1(
            &manifest(&["qualify"]),
            &observation(vec![incomplete]),
        )
        .expect("valid incomplete receipt");
        assert_ne!(receipt.verdict, ConjunctiveVerdictV1::Pass);
    }

    fn runner(id_value: &str, selector: &str) -> RunnerAdapterProfileV1 {
        RunnerAdapterProfileV1 {
            id: id(id_value),
            requested_runner_selector: id(selector),
            os_family: id("linux"),
            architecture: id("x86_64"),
            network_policy: NetworkPolicyV1::ReadOnlyNetwork,
            cache_policy: CachePolicyV1::Disabled,
            action_pins: vec![id("checkout@immutable-sha")],
            required_tools: vec![id("rust-1.96.0")],
        }
    }

    fn harness() -> QualificationHarnessFingerprintV1 {
        QualificationHarnessFingerprintV1 {
            theorem_subject: id("product-sha"),
            theorem_predicates: id("predicate-profile"),
            command_set: id("command-profile"),
            canonical_oracle: id("oracle-profile"),
            object_identity_profile: id("object-profile"),
            claim_boundary: id("claim-profile"),
        }
    }

    fn policy() -> FailoverPolicyV1 {
        FailoverPolicyV1 {
            minimum_no_start_age_seconds: 3_600,
            maximum_concurrent_attempts: 2,
            eligible_source_runner_ids: vec![id("hosted")],
            eligible_target_runner_ids: vec![id("alternate")],
            allow_os_family_change: false,
            allow_architecture_change: false,
            allow_cache_policy_change: false,
        }
    }

    fn failover_request() -> WholeTheoremFailoverRequestV1 {
        WholeTheoremFailoverRequestV1 {
            source_harness: harness(),
            target_harness: harness(),
            source_runner: runner("hosted", "ubuntu-slim"),
            target_runner: runner("alternate", "self-hosted-qualification"),
            source_liveness: RunLivenessV1::AllRequiredJobsNoStart,
            blocked_required_job_age_seconds: Some(7_200),
            active_attempts: 1,
            policy: policy(),
        }
    }

    #[test]
    fn admitted_no_start_may_fail_over_when_theorem_is_unchanged() {
        assert_eq!(
            assess_whole_theorem_failover_v1(&failover_request()),
            FailoverEligibilityV1::Eligible
        );
    }

    #[test]
    fn changed_command_set_blocks_failover() {
        let mut request = failover_request();
        request.target_harness.command_set = id("weaker-command-profile");
        assert_eq!(
            assess_whole_theorem_failover_v1(&request),
            FailoverEligibilityV1::Ineligible(FailoverBlockV1::TheoremHarnessChanged)
        );
    }

    #[test]
    fn online_downgrade_blocks_failover() {
        let mut request = failover_request();
        request.target_runner.network_policy = NetworkPolicyV1::NetworkAllowed;
        assert_eq!(
            assess_whole_theorem_failover_v1(&request),
            FailoverEligibilityV1::Ineligible(FailoverBlockV1::NetworkPolicyChanged)
        );
    }

    #[test]
    fn changed_action_pin_blocks_failover() {
        let mut request = failover_request();
        request.target_runner.action_pins = vec![id("checkout@different-sha")];
        assert_eq!(
            assess_whole_theorem_failover_v1(&request),
            FailoverEligibilityV1::Ineligible(FailoverBlockV1::ActionPinsChanged)
        );
    }

    #[test]
    fn dependency_block_is_not_runner_failover_eligible() {
        let mut request = failover_request();
        request.source_liveness = RunLivenessV1::RequiredDependencyBlocked;
        assert_eq!(
            assess_whole_theorem_failover_v1(&request),
            FailoverEligibilityV1::Ineligible(FailoverBlockV1::SourceStateNotEligible)
        );
    }

    #[test]
    fn no_start_must_cross_registered_age_threshold() {
        let mut request = failover_request();
        request.blocked_required_job_age_seconds = Some(3_599);
        assert_eq!(
            assess_whole_theorem_failover_v1(&request),
            FailoverEligibilityV1::Ineligible(FailoverBlockV1::QueueAgeBelowThreshold)
        );
    }

    #[test]
    fn completed_theorem_is_not_failover_eligible() {
        let mut request = failover_request();
        request.source_liveness = RunLivenessV1::CompletedConjunctivePass;
        assert_eq!(
            assess_whole_theorem_failover_v1(&request),
            FailoverEligibilityV1::Ineligible(FailoverBlockV1::SourceStateNotEligible)
        );
    }

    #[test]
    fn attempt_limit_prevents_queue_flooding() {
        let mut request = failover_request();
        request.active_attempts = 2;
        assert_eq!(
            assess_whole_theorem_failover_v1(&request),
            FailoverEligibilityV1::Ineligible(FailoverBlockV1::AttemptLimitReached)
        );
    }
}
