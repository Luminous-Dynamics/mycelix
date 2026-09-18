#![deny(unsafe_code)]

//! Pure GitHub Actions observation normalization for EVIDENCE-CI-004B.
//!
//! No HTTP client, credential, filesystem, process, scheduler, or repository mutation
//! capability exists here. An outer authenticated adapter supplies GitHub-shaped facts;
//! this crate conservatively maps them into the provider-neutral EVIDENCE-CI core types.

use mycelix_evidence_ci_core::{
    DependencyStateV1, EvidenceId, FailureClassV1, GateExecutionStateV1, JobConclusionV1,
    JobObservationV1, JobStatusV1, WorkflowRunObservationV1,
};
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::fmt;

pub const GITHUB_NORMALIZATION_PROFILE_V1: u16 = 1;
pub const MAX_JOBS_V1: usize = 512;
pub const MAX_STEPS_PER_JOB_V1: usize = 256;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GitHubJobStatusV1 {
    Queued,
    InProgress,
    Completed,
    Waiting,
    Requested,
    Pending,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GitHubJobConclusionV1 {
    Success,
    Failure,
    Cancelled,
    Skipped,
    TimedOut,
    ActionRequired,
    Neutral,
    StartupFailure,
    Stale,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GitHubStepStatusV1 {
    Queued,
    InProgress,
    Completed,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GitHubStepConclusionV1 {
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

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawGitHubStepObservationV1 {
    pub number: u32,
    pub name: EvidenceId,
    pub status: GitHubStepStatusV1,
    pub conclusion: Option<GitHubStepConclusionV1>,
    pub started_at_unix_seconds: Option<u64>,
    pub completed_at_unix_seconds: Option<u64>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawGitHubJobObservationV1 {
    pub job_id: u64,
    pub job_key: EvidenceId,
    pub status: GitHubJobStatusV1,
    pub conclusion: Option<GitHubJobConclusionV1>,
    pub created_at_unix_seconds: Option<u64>,
    /// Diagnostic only. V1 never uses this to prove theorem execution or queue eligibility.
    pub started_at_unix_seconds: Option<u64>,
    pub completed_at_unix_seconds: Option<u64>,
    /// Diagnostic only. Runner assignment is not a theorem-kernel fact.
    pub runner_id: Option<u64>,
    pub steps: Vec<RawGitHubStepObservationV1>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawGitHubRunObservationV1 {
    pub repository_id: u64,
    pub workflow_run_id: u64,
    pub workflow_path: EvidenceId,
    pub qualification_head: EvidenceId,
    pub observed_at_unix_seconds: u64,
    pub jobs: Vec<RawGitHubJobObservationV1>,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct GitHubStepSelectorV1 {
    pub number: u32,
    pub name: EvidenceId,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GitHubJobNormalizationProfileV1 {
    pub job_key: EvidenceId,
    pub needs: Vec<EvidenceId>,
    pub theorem_steps: Vec<GitHubStepSelectorV1>,
    pub infrastructure_steps: Vec<GitHubStepSelectorV1>,
    pub admit_startup_failure_as_infrastructure: bool,
    pub derive_root_queue_age_from_created_at: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GitHubWorkflowNormalizationProfileV1 {
    pub profile_revision: u16,
    pub repository_id: u64,
    pub workflow_path: EvidenceId,
    pub qualification_head: EvidenceId,
    pub jobs: Vec<GitHubJobNormalizationProfileV1>,
}

impl GitHubWorkflowNormalizationProfileV1 {
    pub fn validate(&self) -> Result<(), AdapterError> {
        if self.profile_revision != GITHUB_NORMALIZATION_PROFILE_V1 {
            return Err(AdapterError::UnsupportedProfileRevision);
        }
        if self.jobs.is_empty() || self.jobs.len() > MAX_JOBS_V1 {
            return Err(AdapterError::InvalidJobProfileCount);
        }

        let mut keys = BTreeSet::new();
        for job in &self.jobs {
            if !keys.insert(job.job_key.clone()) {
                return Err(AdapterError::DuplicateJobProfile);
            }
            if job.theorem_steps.len() > MAX_STEPS_PER_JOB_V1
                || job.infrastructure_steps.len() > MAX_STEPS_PER_JOB_V1
            {
                return Err(AdapterError::TooManyStepSelectors);
            }
            reject_duplicate_selectors(&job.theorem_steps)?;
            reject_duplicate_selectors(&job.infrastructure_steps)?;
            let theorem = job.theorem_steps.iter().cloned().collect::<BTreeSet<_>>();
            if job.infrastructure_steps.iter().any(|s| theorem.contains(s)) {
                return Err(AdapterError::StepRoleOverlap);
            }
            let mut needs = BTreeSet::new();
            for dependency in &job.needs {
                if dependency == &job.job_key {
                    return Err(AdapterError::SelfDependency);
                }
                if !needs.insert(dependency.clone()) {
                    return Err(AdapterError::DuplicateDependency);
                }
            }
        }
        if self
            .jobs
            .iter()
            .any(|job| job.needs.iter().any(|need| !keys.contains(need)))
        {
            return Err(AdapterError::UnknownDependency);
        }
        reject_dependency_cycle(&self.jobs)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AdapterError {
    UnsupportedProfileRevision,
    InvalidJobProfileCount,
    DuplicateJobProfile,
    TooManyStepSelectors,
    DuplicateStepSelector,
    StepRoleOverlap,
    SelfDependency,
    DuplicateDependency,
    UnknownDependency,
    DependencyCycle,
    RepositoryMismatch,
    WorkflowPathMismatch,
    QualificationHeadMismatch,
    TooManyObservedJobs,
    DuplicateObservedJob,
    MissingProfiledJob,
    DuplicateObservedStep,
    InvalidJobShape,
    InvalidStepShape,
    SuccessfulJobMissingTheoremStep,
    TimeOrderInvalid,
}

impl fmt::Display for AdapterError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let text = match self {
            Self::UnsupportedProfileRevision => "unsupported normalization profile revision",
            Self::InvalidJobProfileCount => "job profile count is outside the V1 bound",
            Self::DuplicateJobProfile => "duplicate job profile",
            Self::TooManyStepSelectors => "step-selector count exceeds the V1 bound",
            Self::DuplicateStepSelector => "duplicate exact step selector",
            Self::StepRoleOverlap => "step is both theorem and infrastructure",
            Self::SelfDependency => "job depends on itself",
            Self::DuplicateDependency => "duplicate dependency",
            Self::UnknownDependency => "unknown dependency",
            Self::DependencyCycle => "workflow dependency cycle",
            Self::RepositoryMismatch => "repository mismatch",
            Self::WorkflowPathMismatch => "workflow path mismatch",
            Self::QualificationHeadMismatch => "qualification head mismatch",
            Self::TooManyObservedJobs => "observed job count exceeds the V1 bound",
            Self::DuplicateObservedJob => "duplicate observed job key",
            Self::MissingProfiledJob => "profiled job missing from observation",
            Self::DuplicateObservedStep => "duplicate observed exact step",
            Self::InvalidJobShape => "contradictory GitHub job shape",
            Self::InvalidStepShape => "contradictory GitHub step shape",
            Self::SuccessfulJobMissingTheoremStep => "successful job omitted a registered theorem step",
            Self::TimeOrderInvalid => "provider timestamp order is impossible",
        };
        f.write_str(text)
    }
}

impl std::error::Error for AdapterError {}

pub fn normalize_github_run_v1(
    profile: &GitHubWorkflowNormalizationProfileV1,
    raw: &RawGitHubRunObservationV1,
) -> Result<WorkflowRunObservationV1, AdapterError> {
    profile.validate()?;
    validate_binding(profile, raw)?;
    if raw.jobs.len() > MAX_JOBS_V1 {
        return Err(AdapterError::TooManyObservedJobs);
    }
    let observed = index_jobs(&raw.jobs)?;
    for job in &raw.jobs {
        validate_raw_job(job)?;
    }

    let mut jobs = Vec::with_capacity(profile.jobs.len());
    for spec in &profile.jobs {
        let raw_job = observed
            .get(&spec.job_key)
            .copied()
            .ok_or(AdapterError::MissingProfiledJob)?;
        let dependency_state = derive_dependency_state(spec, &observed)?;
        let gate_execution = derive_gate_execution(spec, raw_job)?;
        let failure_class = derive_failure_class(spec, raw_job, gate_execution)?;
        let queue_age_seconds = derive_queue_age(
            spec,
            raw_job,
            &observed,
            raw.observed_at_unix_seconds,
            dependency_state,
        )?;

        jobs.push(JobObservationV1 {
            job_id: raw_job.job_id,
            job_key: spec.job_key.clone(),
            status: map_job_status(raw_job.status),
            conclusion: raw_job.conclusion.map(map_job_conclusion),
            gate_execution,
            dependency_state,
            queue_age_seconds,
            failure_class,
        });
    }

    Ok(WorkflowRunObservationV1 {
        repository_id: raw.repository_id,
        workflow_run_id: raw.workflow_run_id,
        qualification_head: raw.qualification_head.clone(),
        workflow_path: raw.workflow_path.clone(),
        jobs,
    })
}

fn validate_binding(
    profile: &GitHubWorkflowNormalizationProfileV1,
    raw: &RawGitHubRunObservationV1,
) -> Result<(), AdapterError> {
    if profile.repository_id != raw.repository_id {
        return Err(AdapterError::RepositoryMismatch);
    }
    if profile.workflow_path != raw.workflow_path {
        return Err(AdapterError::WorkflowPathMismatch);
    }
    if profile.qualification_head != raw.qualification_head {
        return Err(AdapterError::QualificationHeadMismatch);
    }
    Ok(())
}

fn validate_raw_job(job: &RawGitHubJobObservationV1) -> Result<(), AdapterError> {
    if job.steps.len() > MAX_STEPS_PER_JOB_V1 {
        return Err(AdapterError::TooManyStepSelectors);
    }
    validate_time(job.created_at_unix_seconds, job.started_at_unix_seconds)?;
    validate_time(job.created_at_unix_seconds, job.completed_at_unix_seconds)?;
    match job.status {
        GitHubJobStatusV1::Completed => {
            if job.conclusion.is_none() || job.completed_at_unix_seconds.is_none() {
                return Err(AdapterError::InvalidJobShape);
            }
        }
        GitHubJobStatusV1::Queued
        | GitHubJobStatusV1::InProgress
        | GitHubJobStatusV1::Waiting
        | GitHubJobStatusV1::Requested
        | GitHubJobStatusV1::Pending => {
            if job.conclusion.is_some() || job.completed_at_unix_seconds.is_some() {
                return Err(AdapterError::InvalidJobShape);
            }
        }
        GitHubJobStatusV1::Unknown => {}
    }

    let mut seen = BTreeSet::new();
    for step in &job.steps {
        validate_raw_step(step)?;
        if !seen.insert((step.number, step.name.clone())) {
            return Err(AdapterError::DuplicateObservedStep);
        }
    }
    Ok(())
}

fn validate_raw_step(step: &RawGitHubStepObservationV1) -> Result<(), AdapterError> {
    validate_time(step.started_at_unix_seconds, step.completed_at_unix_seconds)?;
    match step.status {
        GitHubStepStatusV1::Completed if step.conclusion.is_none() => {
            Err(AdapterError::InvalidStepShape)
        }
        GitHubStepStatusV1::Queued | GitHubStepStatusV1::InProgress
            if step.conclusion.is_some() || step.completed_at_unix_seconds.is_some() =>
        {
            Err(AdapterError::InvalidStepShape)
        }
        _ => Ok(()),
    }
}

fn validate_time(start: Option<u64>, end: Option<u64>) -> Result<(), AdapterError> {
    if matches!((start, end), (Some(a), Some(b)) if b < a) {
        return Err(AdapterError::TimeOrderInvalid);
    }
    Ok(())
}

fn derive_dependency_state(
    spec: &GitHubJobNormalizationProfileV1,
    jobs: &BTreeMap<EvidenceId, &RawGitHubJobObservationV1>,
) -> Result<DependencyStateV1, AdapterError> {
    if spec.needs.is_empty() {
        return Ok(DependencyStateV1::EligibleForRunner);
    }
    let mut waiting = false;
    for key in &spec.needs {
        let dep = jobs.get(key).copied().ok_or(AdapterError::MissingProfiledJob)?;
        match (dep.status, dep.conclusion) {
            (GitHubJobStatusV1::Completed, Some(GitHubJobConclusionV1::Success)) => {}
            (GitHubJobStatusV1::Completed, Some(GitHubJobConclusionV1::Skipped)) => {
                return Ok(DependencyStateV1::DependencySkipped)
            }
            (
                GitHubJobStatusV1::Completed,
                Some(
                    GitHubJobConclusionV1::Failure
                    | GitHubJobConclusionV1::Cancelled
                    | GitHubJobConclusionV1::TimedOut
                    | GitHubJobConclusionV1::ActionRequired
                    | GitHubJobConclusionV1::StartupFailure
                    | GitHubJobConclusionV1::Stale,
                ),
            ) => return Ok(DependencyStateV1::DependencyFailed),
            (GitHubJobStatusV1::Completed, Some(GitHubJobConclusionV1::Neutral | GitHubJobConclusionV1::Unknown))
            | (GitHubJobStatusV1::Completed, None)
            | (GitHubJobStatusV1::Unknown, _) => return Ok(DependencyStateV1::Unknown),
            (
                GitHubJobStatusV1::Queued
                | GitHubJobStatusV1::InProgress
                | GitHubJobStatusV1::Waiting
                | GitHubJobStatusV1::Requested
                | GitHubJobStatusV1::Pending,
                None,
            ) => waiting = true,
            (_, Some(_)) => return Ok(DependencyStateV1::Unknown),
        }
    }
    Ok(if waiting {
        DependencyStateV1::WaitingOnRequiredDependency
    } else {
        DependencyStateV1::EligibleForRunner
    })
}

fn derive_gate_execution(
    spec: &GitHubJobNormalizationProfileV1,
    job: &RawGitHubJobObservationV1,
) -> Result<GateExecutionStateV1, AdapterError> {
    if spec.theorem_steps.is_empty() {
        return Ok(GateExecutionStateV1::NoTheoremStepExecuted);
    }
    let steps = index_steps(&job.steps)?;
    let mut executed = 0usize;
    let mut completed = 0usize;
    let mut unknown = false;
    let mut missing = false;

    for selector in &spec.theorem_steps {
        match steps.get(selector).copied() {
            None => missing = true,
            Some(step) => match step_execution(step) {
                StepExecutionV1::NotExecuted => {}
                StepExecutionV1::Executing => executed += 1,
                StepExecutionV1::Completed => {
                    executed += 1;
                    completed += 1;
                }
                StepExecutionV1::Unknown => unknown = true,
            },
        }
    }

    if matches!(job.status, GitHubJobStatusV1::Completed)
        && matches!(job.conclusion, Some(GitHubJobConclusionV1::Success))
        && (missing || completed != spec.theorem_steps.len())
    {
        return Err(AdapterError::SuccessfulJobMissingTheoremStep);
    }
    if unknown {
        return Ok(GateExecutionStateV1::Unknown);
    }
    if completed == spec.theorem_steps.len() {
        return Ok(GateExecutionStateV1::AllRegisteredTheoremStepsExecuted);
    }
    if executed > 0 {
        return Ok(GateExecutionStateV1::SomeTheoremStepsExecuted);
    }
    Ok(GateExecutionStateV1::NoTheoremStepExecuted)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum StepExecutionV1 {
    NotExecuted,
    Executing,
    Completed,
    Unknown,
}

fn step_execution(step: &RawGitHubStepObservationV1) -> StepExecutionV1 {
    match (step.status, step.conclusion) {
        (GitHubStepStatusV1::Queued, None) => StepExecutionV1::NotExecuted,
        (GitHubStepStatusV1::InProgress, None) => StepExecutionV1::Executing,
        (GitHubStepStatusV1::Completed, Some(GitHubStepConclusionV1::Skipped)) => {
            StepExecutionV1::NotExecuted
        }
        (GitHubStepStatusV1::Completed, Some(_)) => StepExecutionV1::Completed,
        _ => StepExecutionV1::Unknown,
    }
}

fn derive_failure_class(
    spec: &GitHubJobNormalizationProfileV1,
    job: &RawGitHubJobObservationV1,
    gate_execution: GateExecutionStateV1,
) -> Result<FailureClassV1, AdapterError> {
    let Some(job_conclusion) = job.conclusion else {
        return Ok(FailureClassV1::NotApplicable);
    };

    if matches!(job_conclusion, GitHubJobConclusionV1::Success) {
        if !spec.theorem_steps.is_empty()
            && !matches!(gate_execution, GateExecutionStateV1::AllRegisteredTheoremStepsExecuted)
        {
            return Err(AdapterError::SuccessfulJobMissingTheoremStep);
        }
        return Ok(FailureClassV1::NotApplicable);
    }
    if matches!(job_conclusion, GitHubJobConclusionV1::Skipped | GitHubJobConclusionV1::Neutral) {
        return Ok(FailureClassV1::NotApplicable);
    }
    if matches!(job_conclusion, GitHubJobConclusionV1::Unknown | GitHubJobConclusionV1::Stale) {
        return Ok(FailureClassV1::Unknown);
    }

    let steps = index_steps(&job.steps)?;
    if spec.theorem_steps.iter().any(|selector| {
        matches!(
            steps.get(selector).and_then(|step| step.conclusion),
            Some(GitHubStepConclusionV1::Failure)
        )
    }) {
        return Ok(FailureClassV1::RegisteredTheoremGate);
    }

    if matches!(gate_execution, GateExecutionStateV1::NoTheoremStepExecuted) {
        if matches!(job_conclusion, GitHubJobConclusionV1::StartupFailure)
            && spec.admit_startup_failure_as_infrastructure
        {
            return Ok(FailureClassV1::RunnerInfrastructureBeforeTheoremGate);
        }
        if spec.infrastructure_steps.iter().any(|selector| {
            steps
                .get(selector)
                .and_then(|step| step.conclusion)
                .is_some_and(is_infrastructure_negative)
        }) {
            return Ok(FailureClassV1::RunnerInfrastructureBeforeTheoremGate);
        }
    }
    Ok(FailureClassV1::Unknown)
}

fn is_infrastructure_negative(value: GitHubStepConclusionV1) -> bool {
    matches!(
        value,
        GitHubStepConclusionV1::Failure
            | GitHubStepConclusionV1::Cancelled
            | GitHubStepConclusionV1::TimedOut
            | GitHubStepConclusionV1::ActionRequired
            | GitHubStepConclusionV1::StartupFailure
    )
}

fn derive_queue_age(
    spec: &GitHubJobNormalizationProfileV1,
    job: &RawGitHubJobObservationV1,
    jobs: &BTreeMap<EvidenceId, &RawGitHubJobObservationV1>,
    observed_at: u64,
    dependency_state: DependencyStateV1,
) -> Result<Option<u64>, AdapterError> {
    if !matches!(job.status, GitHubJobStatusV1::Queued)
        || !matches!(dependency_state, DependencyStateV1::EligibleForRunner)
    {
        return Ok(None);
    }

    let origin = if spec.needs.is_empty() {
        if spec.derive_root_queue_age_from_created_at {
            job.created_at_unix_seconds
        } else {
            None
        }
    } else {
        let mut latest_dependency = None;
        for key in &spec.needs {
            let dep = jobs.get(key).copied().ok_or(AdapterError::MissingProfiledJob)?;
            if !matches!(
                (dep.status, dep.conclusion),
                (GitHubJobStatusV1::Completed, Some(GitHubJobConclusionV1::Success))
            ) {
                return Ok(None);
            }
            let Some(completed) = dep.completed_at_unix_seconds else {
                return Ok(None);
            };
            latest_dependency = Some(
                latest_dependency.map_or(completed, |current: u64| current.max(completed)),
            );
        }
        match (latest_dependency, job.created_at_unix_seconds) {
            (Some(dep_ready), Some(created)) => Some(dep_ready.max(created)),
            (Some(dep_ready), None) => Some(dep_ready),
            _ => None,
        }
    };

    match origin {
        Some(value) if observed_at >= value => Ok(Some(observed_at - value)),
        Some(_) => Err(AdapterError::TimeOrderInvalid),
        None => Ok(None),
    }
}

fn map_job_status(value: GitHubJobStatusV1) -> JobStatusV1 {
    match value {
        GitHubJobStatusV1::Queued => JobStatusV1::Queued,
        GitHubJobStatusV1::InProgress => JobStatusV1::InProgress,
        GitHubJobStatusV1::Completed => JobStatusV1::Completed,
        GitHubJobStatusV1::Waiting
        | GitHubJobStatusV1::Requested
        | GitHubJobStatusV1::Pending
        | GitHubJobStatusV1::Unknown => JobStatusV1::Unknown,
    }
}

fn map_job_conclusion(value: GitHubJobConclusionV1) -> JobConclusionV1 {
    match value {
        GitHubJobConclusionV1::Success => JobConclusionV1::Success,
        GitHubJobConclusionV1::Failure => JobConclusionV1::Failure,
        GitHubJobConclusionV1::Cancelled => JobConclusionV1::Cancelled,
        GitHubJobConclusionV1::Skipped => JobConclusionV1::Skipped,
        GitHubJobConclusionV1::TimedOut => JobConclusionV1::TimedOut,
        GitHubJobConclusionV1::ActionRequired => JobConclusionV1::ActionRequired,
        GitHubJobConclusionV1::Neutral => JobConclusionV1::Neutral,
        GitHubJobConclusionV1::StartupFailure => JobConclusionV1::StartupFailure,
        GitHubJobConclusionV1::Stale | GitHubJobConclusionV1::Unknown => JobConclusionV1::Unknown,
    }
}

fn index_jobs<'a>(
    jobs: &'a [RawGitHubJobObservationV1],
) -> Result<BTreeMap<EvidenceId, &'a RawGitHubJobObservationV1>, AdapterError> {
    let mut out = BTreeMap::new();
    for job in jobs {
        if out.insert(job.job_key.clone(), job).is_some() {
            return Err(AdapterError::DuplicateObservedJob);
        }
    }
    Ok(out)
}

fn index_steps<'a>(
    steps: &'a [RawGitHubStepObservationV1],
) -> Result<BTreeMap<GitHubStepSelectorV1, &'a RawGitHubStepObservationV1>, AdapterError> {
    let mut out = BTreeMap::new();
    for step in steps {
        let key = GitHubStepSelectorV1 {
            number: step.number,
            name: step.name.clone(),
        };
        if out.insert(key, step).is_some() {
            return Err(AdapterError::DuplicateObservedStep);
        }
    }
    Ok(out)
}

fn reject_duplicate_selectors(values: &[GitHubStepSelectorV1]) -> Result<(), AdapterError> {
    let mut seen = BTreeSet::new();
    for value in values {
        if !seen.insert(value.clone()) {
            return Err(AdapterError::DuplicateStepSelector);
        }
    }
    Ok(())
}

fn reject_dependency_cycle(jobs: &[GitHubJobNormalizationProfileV1]) -> Result<(), AdapterError> {
    let mut indegree = jobs
        .iter()
        .map(|job| (job.job_key.clone(), job.needs.len()))
        .collect::<BTreeMap<_, _>>();
    let mut dependents = BTreeMap::<EvidenceId, Vec<EvidenceId>>::new();
    for job in jobs {
        for dependency in &job.needs {
            dependents
                .entry(dependency.clone())
                .or_default()
                .push(job.job_key.clone());
        }
    }
    let mut queue = indegree
        .iter()
        .filter_map(|(key, degree)| (*degree == 0).then_some(key.clone()))
        .collect::<VecDeque<_>>();
    let mut visited = 0usize;
    while let Some(job) = queue.pop_front() {
        visited += 1;
        if let Some(children) = dependents.get(&job) {
            for child in children {
                let degree = indegree.get_mut(child).ok_or(AdapterError::UnknownDependency)?;
                *degree -= 1;
                if *degree == 0 {
                    queue.push_back(child.clone());
                }
            }
        }
    }
    if visited == jobs.len() {
        Ok(())
    } else {
        Err(AdapterError::DependencyCycle)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_evidence_ci_core::{
        derive_conjunctive_receipt_v1, ConjunctiveVerdictV1, RequiredJobManifestV1,
        RequiredJobSpecV1, RunLivenessV1,
    };

    fn id(value: &str) -> EvidenceId {
        EvidenceId::new(value).expect("static id")
    }

    fn selector(number: u32, name: &str) -> GitHubStepSelectorV1 {
        GitHubStepSelectorV1 { number, name: id(name) }
    }

    fn completed_step(
        number: u32,
        name: &str,
        conclusion: GitHubStepConclusionV1,
    ) -> RawGitHubStepObservationV1 {
        RawGitHubStepObservationV1 {
            number,
            name: id(name),
            status: GitHubStepStatusV1::Completed,
            conclusion: Some(conclusion),
            started_at_unix_seconds: Some(100),
            completed_at_unix_seconds: Some(101),
        }
    }

    fn queued_job(key: &str, created: u64) -> RawGitHubJobObservationV1 {
        RawGitHubJobObservationV1 {
            job_id: created,
            job_key: id(key),
            status: GitHubJobStatusV1::Queued,
            conclusion: None,
            created_at_unix_seconds: Some(created),
            // Mirrors the live GitHub quirk: queued job, started_at populated, no runner/steps.
            started_at_unix_seconds: Some(created),
            completed_at_unix_seconds: None,
            runner_id: Some(0),
            steps: Vec::new(),
        }
    }

    fn successful_job(
        key: &str,
        job_id: u64,
        theorem: &GitHubStepSelectorV1,
        completed_at: u64,
    ) -> RawGitHubJobObservationV1 {
        RawGitHubJobObservationV1 {
            job_id,
            job_key: id(key),
            status: GitHubJobStatusV1::Completed,
            conclusion: Some(GitHubJobConclusionV1::Success),
            created_at_unix_seconds: Some(10),
            started_at_unix_seconds: Some(20),
            completed_at_unix_seconds: Some(completed_at),
            runner_id: Some(1000 + job_id),
            steps: vec![completed_step(
                theorem.number,
                theorem.name.as_str(),
                GitHubStepConclusionV1::Success,
            )],
        }
    }

    fn theorem_job(key: &str, theorem: GitHubStepSelectorV1) -> GitHubJobNormalizationProfileV1 {
        GitHubJobNormalizationProfileV1 {
            job_key: id(key),
            needs: Vec::new(),
            theorem_steps: vec![theorem],
            infrastructure_steps: Vec::new(),
            admit_startup_failure_as_infrastructure: true,
            derive_root_queue_age_from_created_at: true,
        }
    }

    fn workflow(jobs: Vec<GitHubJobNormalizationProfileV1>) -> GitHubWorkflowNormalizationProfileV1 {
        GitHubWorkflowNormalizationProfileV1 {
            profile_revision: 1,
            repository_id: 1176351975,
            workflow_path: id(".github/workflows/qual.yml"),
            qualification_head: id("abc123"),
            jobs,
        }
    }

    fn run(jobs: Vec<RawGitHubJobObservationV1>, observed_at: u64) -> RawGitHubRunObservationV1 {
        RawGitHubRunObservationV1 {
            repository_id: 1176351975,
            workflow_run_id: 42,
            workflow_path: id(".github/workflows/qual.yml"),
            qualification_head: id("abc123"),
            observed_at_unix_seconds: observed_at,
            jobs,
        }
    }

    #[test]
    fn queued_started_at_does_not_create_theorem_execution() {
        let theorem = selector(2, "Qualify exact subject");
        let normalized = normalize_github_run_v1(
            &workflow(vec![theorem_job("qualify", theorem)]),
            &run(vec![queued_job("qualify", 100)], 500),
        )
        .expect("normalize");
        assert_eq!(normalized.jobs[0].status, JobStatusV1::Queued);
        assert_eq!(normalized.jobs[0].gate_execution, GateExecutionStateV1::NoTheoremStepExecuted);
        assert_eq!(normalized.jobs[0].dependency_state, DependencyStateV1::EligibleForRunner);
        assert_eq!(normalized.jobs[0].queue_age_seconds, Some(400));
    }

    #[test]
    fn completed_theorem_jobs_close_without_queued_presentation_job() {
        let shallow_step = selector(3, "Prove parent is available and exact");
        let full_step = selector(3, "Prove exact workflow-only child and clean tree");
        let shallow = theorem_job("Exact head with fetch-depth 2", shallow_step.clone());
        let full = theorem_job("Exact head with full history", full_step.clone());
        let verdict = GitHubJobNormalizationProfileV1 {
            job_key: id("Checkout v7 qualification verdict"),
            needs: vec![shallow.job_key.clone(), full.job_key.clone()],
            theorem_steps: Vec::new(),
            infrastructure_steps: Vec::new(),
            admit_startup_failure_as_infrastructure: false,
            derive_root_queue_age_from_created_at: false,
        };
        let normalized = normalize_github_run_v1(
            &workflow(vec![shallow.clone(), full.clone(), verdict]),
            &run(
                vec![
                    successful_job(shallow.job_key.as_str(), 1, &shallow_step, 200),
                    successful_job(full.job_key.as_str(), 2, &full_step, 220),
                    queued_job("Checkout v7 qualification verdict", 221),
                ],
                500,
            ),
        )
        .expect("normalize #1493 shape");
        let receipt = derive_conjunctive_receipt_v1(
            &RequiredJobManifestV1 {
                theorem_id: id("checkout-v7"),
                repository_id: 1176351975,
                workflow_path: id(".github/workflows/qual.yml"),
                qualification_head: id("abc123"),
                required_jobs: vec![
                    RequiredJobSpecV1 { job_key: shallow.job_key },
                    RequiredJobSpecV1 { job_key: full.job_key },
                ],
            },
            &normalized,
        )
        .expect("derive conjunction");
        assert_eq!(receipt.verdict, ConjunctiveVerdictV1::Pass);
        assert_eq!(receipt.liveness, RunLivenessV1::CompletedConjunctivePass);
        let verdict = normalized.jobs.iter().find(|j| j.job_key.as_str() == "Checkout v7 qualification verdict").unwrap();
        assert_eq!(verdict.dependency_state, DependencyStateV1::EligibleForRunner);
        assert_eq!(verdict.queue_age_seconds, Some(279));
    }

    #[test]
    fn dependency_wait_failure_and_skip_are_distinct() {
        let theorem = selector(2, "Dependency theorem");
        for (status, conclusion, expected) in [
            (GitHubJobStatusV1::InProgress, None, DependencyStateV1::WaitingOnRequiredDependency),
            (GitHubJobStatusV1::Completed, Some(GitHubJobConclusionV1::Failure), DependencyStateV1::DependencyFailed),
            (GitHubJobStatusV1::Completed, Some(GitHubJobConclusionV1::Skipped), DependencyStateV1::DependencySkipped),
        ] {
            let dependency = theorem_job("dependency", theorem.clone());
            let target = GitHubJobNormalizationProfileV1 {
                job_key: id("target"),
                needs: vec![dependency.job_key.clone()],
                theorem_steps: Vec::new(),
                infrastructure_steps: Vec::new(),
                admit_startup_failure_as_infrastructure: false,
                derive_root_queue_age_from_created_at: false,
            };
            let mut dep = if matches!(status, GitHubJobStatusV1::Completed) {
                successful_job("dependency", 1, &theorem, 50)
            } else {
                let mut job = queued_job("dependency", 10);
                job.status = status;
                job.steps.push(RawGitHubStepObservationV1 {
                    number: theorem.number,
                    name: theorem.name.clone(),
                    status: GitHubStepStatusV1::InProgress,
                    conclusion: None,
                    started_at_unix_seconds: Some(20),
                    completed_at_unix_seconds: None,
                });
                job
            };
            dep.status = status;
            dep.conclusion = conclusion;
            if matches!(status, GitHubJobStatusV1::Completed) {
                dep.steps[0].conclusion = Some(match conclusion.unwrap() {
                    GitHubJobConclusionV1::Failure => GitHubStepConclusionV1::Failure,
                    GitHubJobConclusionV1::Skipped => GitHubStepConclusionV1::Skipped,
                    _ => unreachable!(),
                });
            }
            let normalized = normalize_github_run_v1(
                &workflow(vec![dependency, target]),
                &run(vec![dep, queued_job("target", 51)], 100),
            )
            .expect("normalize dependency state");
            let target = normalized.jobs.iter().find(|j| j.job_key.as_str() == "target").unwrap();
            assert_eq!(target.dependency_state, expected);
            assert_eq!(target.queue_age_seconds, None);
        }
    }

    #[test]
    fn only_exact_theorem_failure_becomes_semantic_red() {
        let theorem = selector(3, "Theorem gate");
        for (step_conclusion, expected) in [
            (GitHubStepConclusionV1::Failure, FailureClassV1::RegisteredTheoremGate),
            (GitHubStepConclusionV1::Cancelled, FailureClassV1::Unknown),
            (GitHubStepConclusionV1::TimedOut, FailureClassV1::Unknown),
        ] {
            let mut job = successful_job("qualify", 1, &theorem, 50);
            job.conclusion = Some(match step_conclusion {
                GitHubStepConclusionV1::Failure => GitHubJobConclusionV1::Failure,
                GitHubStepConclusionV1::Cancelled => GitHubJobConclusionV1::Cancelled,
                GitHubStepConclusionV1::TimedOut => GitHubJobConclusionV1::TimedOut,
                _ => unreachable!(),
            });
            job.steps[0].conclusion = Some(step_conclusion);
            let normalized = normalize_github_run_v1(
                &workflow(vec![theorem_job("qualify", theorem.clone())]),
                &run(vec![job], 100),
            )
            .expect("normalize terminal theorem state");
            assert_eq!(normalized.jobs[0].failure_class, expected);
        }
    }

    #[test]
    fn preregistered_infrastructure_failure_before_theorem_is_infrastructure() {
        let theorem = selector(3, "Theorem gate");
        let infra = selector(2, "Checkout exact subject");
        let spec = GitHubJobNormalizationProfileV1 {
            job_key: id("qualify"),
            needs: Vec::new(),
            theorem_steps: vec![theorem],
            infrastructure_steps: vec![infra.clone()],
            admit_startup_failure_as_infrastructure: true,
            derive_root_queue_age_from_created_at: true,
        };
        let job = RawGitHubJobObservationV1 {
            job_id: 1,
            job_key: id("qualify"),
            status: GitHubJobStatusV1::Completed,
            conclusion: Some(GitHubJobConclusionV1::Failure),
            created_at_unix_seconds: Some(10),
            started_at_unix_seconds: Some(20),
            completed_at_unix_seconds: Some(30),
            runner_id: Some(1),
            steps: vec![completed_step(infra.number, infra.name.as_str(), GitHubStepConclusionV1::Failure)],
        };
        let normalized = normalize_github_run_v1(&workflow(vec![spec]), &run(vec![job], 40)).unwrap();
        assert_eq!(normalized.jobs[0].failure_class, FailureClassV1::RunnerInfrastructureBeforeTheoremGate);
        assert_eq!(normalized.jobs[0].gate_execution, GateExecutionStateV1::NoTheoremStepExecuted);
    }

    #[test]
    fn post_theorem_failure_without_failed_theorem_step_is_unknown() {
        let theorem = selector(2, "Theorem gate");
        let mut job = successful_job("qualify", 1, &theorem, 50);
        job.conclusion = Some(GitHubJobConclusionV1::Failure);
        job.steps.push(completed_step(3, "Upload report", GitHubStepConclusionV1::Failure));
        let normalized = normalize_github_run_v1(
            &workflow(vec![theorem_job("qualify", theorem)]),
            &run(vec![job], 100),
        )
        .unwrap();
        assert_eq!(normalized.jobs[0].failure_class, FailureClassV1::Unknown);
    }

    #[test]
    fn successful_job_missing_theorem_step_is_rejected() {
        let theorem = selector(2, "Missing theorem");
        let job = RawGitHubJobObservationV1 {
            job_id: 1,
            job_key: id("qualify"),
            status: GitHubJobStatusV1::Completed,
            conclusion: Some(GitHubJobConclusionV1::Success),
            created_at_unix_seconds: Some(10),
            started_at_unix_seconds: Some(20),
            completed_at_unix_seconds: Some(30),
            runner_id: Some(1),
            steps: vec![completed_step(1, "Set up job", GitHubStepConclusionV1::Success)],
        };
        assert_eq!(
            normalize_github_run_v1(&workflow(vec![theorem_job("qualify", theorem)]), &run(vec![job], 40)),
            Err(AdapterError::SuccessfulJobMissingTheoremStep)
        );
    }

    #[test]
    fn dependent_queue_age_uses_latest_dependency_completion_not_started_at() {
        let a_step = selector(2, "A theorem");
        let b_step = selector(2, "B theorem");
        let a = theorem_job("a", a_step.clone());
        let b = theorem_job("b", b_step.clone());
        let target = GitHubJobNormalizationProfileV1 {
            job_key: id("target"),
            needs: vec![a.job_key.clone(), b.job_key.clone()],
            theorem_steps: Vec::new(),
            infrastructure_steps: Vec::new(),
            admit_startup_failure_as_infrastructure: false,
            derive_root_queue_age_from_created_at: false,
        };
        let normalized = normalize_github_run_v1(
            &workflow(vec![a, b, target]),
            &run(
                vec![
                    successful_job("a", 1, &a_step, 100),
                    successful_job("b", 2, &b_step, 140),
                    queued_job("target", 120),
                ],
                200,
            ),
        )
        .unwrap();
        let target = normalized.jobs.iter().find(|j| j.job_key.as_str() == "target").unwrap();
        assert_eq!(target.queue_age_seconds, Some(60));
    }

    #[test]
    fn duplicate_step_and_binding_substitution_are_rejected() {
        let theorem = selector(2, "Theorem gate");
        let mut job = successful_job("qualify", 1, &theorem, 50);
        job.steps.push(job.steps[0].clone());
        assert_eq!(
            normalize_github_run_v1(&workflow(vec![theorem_job("qualify", theorem.clone())]), &run(vec![job], 100)),
            Err(AdapterError::DuplicateObservedStep)
        );

        let p = workflow(vec![theorem_job("qualify", theorem)]);
        let base = run(vec![queued_job("qualify", 10)], 100);
        let mut wrong = base.clone();
        wrong.repository_id += 1;
        assert_eq!(normalize_github_run_v1(&p, &wrong), Err(AdapterError::RepositoryMismatch));
        let mut wrong = base.clone();
        wrong.workflow_path = id(".github/workflows/other.yml");
        assert_eq!(normalize_github_run_v1(&p, &wrong), Err(AdapterError::WorkflowPathMismatch));
        let mut wrong = base;
        wrong.qualification_head = id("other-head");
        assert_eq!(normalize_github_run_v1(&p, &wrong), Err(AdapterError::QualificationHeadMismatch));
    }

    #[test]
    fn unknown_status_is_not_upgraded_to_queued() {
        let theorem = selector(2, "Theorem gate");
        let mut job = queued_job("qualify", 10);
        job.status = GitHubJobStatusV1::Unknown;
        let normalized = normalize_github_run_v1(
            &workflow(vec![theorem_job("qualify", theorem)]),
            &run(vec![job], 100),
        )
        .unwrap();
        assert_eq!(normalized.jobs[0].status, JobStatusV1::Unknown);
        assert_eq!(normalized.jobs[0].queue_age_seconds, None);
    }

    #[test]
    fn dependency_cycle_is_rejected() {
        let a = GitHubJobNormalizationProfileV1 {
            job_key: id("a"),
            needs: vec![id("b")],
            theorem_steps: Vec::new(),
            infrastructure_steps: Vec::new(),
            admit_startup_failure_as_infrastructure: false,
            derive_root_queue_age_from_created_at: false,
        };
        let b = GitHubJobNormalizationProfileV1 {
            job_key: id("b"),
            needs: vec![id("a")],
            theorem_steps: Vec::new(),
            infrastructure_steps: Vec::new(),
            admit_startup_failure_as_infrastructure: false,
            derive_root_queue_age_from_created_at: false,
        };
        assert_eq!(workflow(vec![a, b]).validate(), Err(AdapterError::DependencyCycle));
    }
}
