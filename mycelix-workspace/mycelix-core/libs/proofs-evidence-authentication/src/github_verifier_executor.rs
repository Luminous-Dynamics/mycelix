use std::env;
use std::fs::{self, File, OpenOptions};
use std::io::{self, Read, Write};
use std::path::Path;
use std::process::{Child, Command, ExitStatus, Stdio};
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

#[cfg(unix)]
use std::os::unix::fs::{MetadataExt, PermissionsExt};
#[cfg(unix)]
use std::os::unix::process::ExitStatusExt;

use serde_json::Value;
use sha2::{Digest, Sha256};

use crate::{
    GitHubPublicCommandPlanV1, GitHubPublicVerifierExecutionPolicyV1,
    GitHubVerificationResultParseErrorV1, GitHubVerifierExecutionStepV1,
    GitHubVerifierStepPurposeV1, GitHubVerifierStdoutDispositionV1,
    ParsedGitHubAttestationVerifierOutputV1, QualificationReceiptV1,
    ReceiptCanonicalizationErrorV1, Sha256DigestV1, VerifierExecutionEvidenceErrorV1,
    VerifierProcessExecutionReceiptV1, VerifierProcessOutcomeV1, VerifierTrustRootModeV1,
    VERIFIER_EXECUTION_EVIDENCE_VERSION_V1, github_public_verifier_execution_steps_v1,
    parse_github_attestation_verifier_output_v1,
};

pub const GITHUB_VERIFIER_EXECUTOR_PROFILE_V1: &str =
    "mycelix-github-verifier-executor-v1";
pub const GITHUB_VERIFIER_EXECUTOR_CLOCK_PROFILE_V1: &str =
    "host-observed-system-time-v1";
pub const MAX_GITHUB_EXECUTOR_RETAINED_FILE_BYTES_V1: usize = 4 * 1024 * 1024;
const PROCESS_POLL_INTERVAL: Duration = Duration::from_millis(5);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ExecutedGitHubVerificationAuthorityV1 {
    ActualVerifierExecutionOnly,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GitHubExecutedStepEvidenceV1 {
    purpose: GitHubVerifierStepPurposeV1,
    stdout_digest: Sha256DigestV1,
    stderr_digest: Sha256DigestV1,
    stdout_len: usize,
    stderr_len: usize,
}

impl GitHubExecutedStepEvidenceV1 {
    pub const fn purpose(&self) -> GitHubVerifierStepPurposeV1 {
        self.purpose
    }

    pub const fn stdout_digest(&self) -> Sha256DigestV1 {
        self.stdout_digest
    }

    pub const fn stderr_digest(&self) -> Sha256DigestV1 {
        self.stderr_digest
    }

    pub const fn stdout_len(&self) -> usize {
        self.stdout_len
    }

    pub const fn stderr_len(&self) -> usize {
        self.stderr_len
    }
}

/// Process-local proof that the exact sealed GitHub verifier plan was actually
/// executed through this crate's bounded executor.
///
/// This type intentionally implements neither `Clone` nor serde traits. Exportable
/// structural receipts remain audit evidence only and cannot recreate this capability.
#[derive(Debug)]
pub struct ExecutedGitHubVerificationV1 {
    profile_id: &'static str,
    clock_profile_id: &'static str,
    execution_policy: GitHubPublicVerifierExecutionPolicyV1,
    command_plan: GitHubPublicCommandPlanV1,
    process_receipt: VerifierProcessExecutionReceiptV1,
    step_evidence: [GitHubExecutedStepEvidenceV1; 3],
    attestation_bundle_bytes: Vec<u8>,
    retained_root_bytes: Vec<u8>,
    download_stdout_bytes: Vec<u8>,
    download_stderr_bytes: Vec<u8>,
    trusted_root_stderr_bytes: Vec<u8>,
    verifier_stdout_bytes: Vec<u8>,
    verifier_stderr_bytes: Vec<u8>,
    authority: ExecutedGitHubVerificationAuthorityV1,
}

impl ExecutedGitHubVerificationV1 {
    pub const fn profile_id(&self) -> &'static str {
        self.profile_id
    }

    /// Records where chronology came from. This is not trusted-time authority.
    pub const fn clock_profile_id(&self) -> &'static str {
        self.clock_profile_id
    }

    pub fn execution_policy(&self) -> &GitHubPublicVerifierExecutionPolicyV1 {
        &self.execution_policy
    }

    pub fn command_plan(&self) -> &GitHubPublicCommandPlanV1 {
        &self.command_plan
    }

    pub fn process_receipt(&self) -> &VerifierProcessExecutionReceiptV1 {
        &self.process_receipt
    }

    pub fn step_evidence(&self) -> &[GitHubExecutedStepEvidenceV1; 3] {
        &self.step_evidence
    }

    pub fn attestation_bundle_bytes(&self) -> &[u8] {
        &self.attestation_bundle_bytes
    }

    pub fn retained_root_bytes(&self) -> &[u8] {
        &self.retained_root_bytes
    }

    pub fn download_stdout_bytes(&self) -> &[u8] {
        &self.download_stdout_bytes
    }

    pub fn download_stderr_bytes(&self) -> &[u8] {
        &self.download_stderr_bytes
    }

    pub fn trusted_root_stderr_bytes(&self) -> &[u8] {
        &self.trusted_root_stderr_bytes
    }

    pub fn verifier_stdout_bytes(&self) -> &[u8] {
        &self.verifier_stdout_bytes
    }

    pub fn verifier_stderr_bytes(&self) -> &[u8] {
        &self.verifier_stderr_bytes
    }

    pub const fn authority_scope(&self) -> ExecutedGitHubVerificationAuthorityV1 {
        self.authority
    }

    pub const fn establishes_actual_process_execution(&self) -> bool {
        true
    }

    pub const fn establishes_trusted_clock(&self) -> bool {
        false
    }

    pub const fn establishes_parsed_verifier_result(&self) -> bool {
        false
    }

    pub const fn establishes_receipt_authentication(&self) -> bool {
        false
    }

    pub const fn grants_production_authority(&self) -> bool {
        false
    }

    pub const fn grants_application_authority(&self) -> bool {
        false
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GitHubVerifierExecutorErrorV1 {
    UnsupportedHostPlatform,
    InvalidExecutionPolicy,
    ExecutablePathMismatch,
    UnsupportedUnverifiedNixClosureIdentity,
    ExecutablePathNotCanonical,
    ExecutableMetadataUnavailable,
    ExecutableSymlink,
    ExecutableNotRegularFile,
    ExecutableNotMarkedExecutable,
    ExecutableReadFailed,
    ExecutableDigestMismatch,
    ExecutableChangedDuringExecution,
    WorkingDirectoryPathNotCanonical,
    WorkingDirectoryMetadataUnavailable,
    WorkingDirectorySymlink,
    WorkingDirectoryNotDirectory,
    WorkingDirectoryChangedDuringExecution,
    InvalidPlannedChildPath { field: &'static str },
    MissingFixedEnvironment { key: &'static str },
    PreExistingPath { field: &'static str },
    CreateDirectoryFailed { field: &'static str },
    ReceiptCanonicalization(ReceiptCanonicalizationErrorV1),
    CanonicalReceiptDigestMismatch,
    SubjectCreateFailed,
    SubjectWriteFailed,
    SubjectReadbackMismatch,
    MissingInheritedSecret { key: String },
    EnvironmentMustBeCleared,
    UnexpectedSecretEnvironmentPolicy,
    SpawnFailed { purpose: GitHubVerifierStepPurposeV1 },
    MissingChildStdout { purpose: GitHubVerifierStepPurposeV1 },
    MissingChildStderr { purpose: GitHubVerifierStepPurposeV1 },
    ChildPollFailed { purpose: GitHubVerifierStepPurposeV1 },
    ChildKillFailed { purpose: GitHubVerifierStepPurposeV1 },
    ChildWaitFailed { purpose: GitHubVerifierStepPurposeV1 },
    ProcessTimedOut { purpose: GitHubVerifierStepPurposeV1 },
    StdoutTooLarge { purpose: GitHubVerifierStepPurposeV1 },
    StderrTooLarge { purpose: GitHubVerifierStepPurposeV1 },
    StdoutReadFailed { purpose: GitHubVerifierStepPurposeV1 },
    StderrReadFailed { purpose: GitHubVerifierStepPurposeV1 },
    ReaderThreadFailed { purpose: GitHubVerifierStepPurposeV1 },
    NonZeroExit {
        purpose: GitHubVerifierStepPurposeV1,
        code: Option<i32>,
        signal: Option<i32>,
    },
    UnexpectedStepGraph,
    OutputMissing { field: &'static str },
    OutputSymlink { field: &'static str },
    OutputNotRegularFile { field: &'static str },
    OutputTooLarge { field: &'static str },
    OutputReadFailed { field: &'static str },
    TrustedRootCreateFailed,
    TrustedRootWriteFailed,
    ClockBeforeUnixEpoch,
    ClockRegression,
    ProcessReceiptInvalid(VerifierExecutionEvidenceErrorV1),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ExecutedGitHubVerificationParseErrorV1 {
    InvalidProcessReceipt(VerifierExecutionEvidenceErrorV1),
    StdoutTooLarge { maximum: usize, actual: usize },
    InvalidJson,
    EmptyResults,
    TooManyResults { maximum: usize, actual: usize },
    ProcessReceiptPromotion(VerifierExecutionEvidenceErrorV1),
    StrictParser(GitHubVerificationResultParseErrorV1),
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct FileFingerprint {
    len: u64,
    #[cfg(unix)]
    device: u64,
    #[cfg(unix)]
    inode: u64,
    #[cfg(unix)]
    mode: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct DirectoryFingerprint {
    #[cfg(unix)]
    device: u64,
    #[cfg(unix)]
    inode: u64,
    #[cfg(unix)]
    mode: u32,
}

#[derive(Debug)]
struct StepOutput {
    stdout: Vec<u8>,
    stderr: Vec<u8>,
}

#[derive(Debug)]
enum BoundedReadError {
    TooLarge,
    Io,
}

pub fn execute_github_public_verifier_v1(
    execution_policy: &GitHubPublicVerifierExecutionPolicyV1,
    command_plan: &GitHubPublicCommandPlanV1,
    receipt: &QualificationReceiptV1,
) -> Result<ExecutedGitHubVerificationV1, GitHubVerifierExecutorErrorV1> {
    if !cfg!(all(target_os = "linux", target_arch = "x86_64")) {
        return Err(GitHubVerifierExecutorErrorV1::UnsupportedHostPlatform);
    }

    execution_policy
        .validate()
        .map_err(|_| GitHubVerifierExecutorErrorV1::InvalidExecutionPolicy)?;
    if command_plan.executable_path() != execution_policy.executable_path() {
        return Err(GitHubVerifierExecutorErrorV1::ExecutablePathMismatch);
    }
    if execution_policy.expected_verifier().nix_closure.is_some() {
        return Err(
            GitHubVerifierExecutorErrorV1::UnsupportedUnverifiedNixClosureIdentity,
        );
    }

    let work_root = Path::new(command_plan.working_directory());
    require_canonical_path(
        work_root,
        GitHubVerifierExecutorErrorV1::WorkingDirectoryPathNotCanonical,
    )?;
    let work_fingerprint = directory_fingerprint(work_root)?;

    let executable_path = Path::new(command_plan.executable_path());
    require_canonical_path(
        executable_path,
        GitHubVerifierExecutorErrorV1::ExecutablePathNotCanonical,
    )?;
    let executable_fingerprint = executable_fingerprint(executable_path)?;
    verify_executable_identity(
        executable_path,
        &executable_fingerprint,
        execution_policy.expected_verifier().executable_sha256,
    )?;

    validate_direct_child(work_root, command_plan.subject_path(), "subject_path")?;
    validate_direct_child(work_root, command_plan.bundle_path(), "bundle_path")?;
    validate_direct_child(
        work_root,
        command_plan.trusted_root_path(),
        "trusted_root_path",
    )?;

    let gh_config_dir = fixed_environment_path(command_plan, "GH_CONFIG_DIR")?;
    let home_dir = fixed_environment_path(command_plan, "HOME")?;
    validate_direct_child(work_root, &gh_config_dir, "GH_CONFIG_DIR")?;
    validate_direct_child(work_root, &home_dir, "HOME")?;

    require_absent(command_plan.subject_path(), "subject_path")?;
    require_absent(command_plan.bundle_path(), "bundle_path")?;
    require_absent(command_plan.trusted_root_path(), "trusted_root_path")?;
    require_absent(&gh_config_dir, "GH_CONFIG_DIR")?;
    require_absent(&home_dir, "HOME")?;

    fs::create_dir(&gh_config_dir).map_err(|_| {
        GitHubVerifierExecutorErrorV1::CreateDirectoryFailed {
            field: "GH_CONFIG_DIR",
        }
    })?;
    fs::create_dir(&home_dir).map_err(|_| {
        GitHubVerifierExecutorErrorV1::CreateDirectoryFailed { field: "HOME" }
    })?;

    let canonical_bytes = receipt
        .canonical_bytes()
        .map_err(GitHubVerifierExecutorErrorV1::ReceiptCanonicalization)?;
    let receipt_digest = receipt
        .digest()
        .map_err(GitHubVerifierExecutorErrorV1::ReceiptCanonicalization)?;
    if receipt_digest != command_plan.canonical_receipt_digest()
        || raw_sha256(&canonical_bytes) != command_plan.canonical_receipt_digest().sha256
    {
        return Err(GitHubVerifierExecutorErrorV1::CanonicalReceiptDigestMismatch);
    }
    materialize_subject(command_plan.subject_path(), &canonical_bytes)?;

    let secret_environment = collect_inherited_secrets(command_plan)?;
    let steps = github_public_verifier_execution_steps_v1(command_plan);
    validate_step_graph(&steps, command_plan)?;

    let execution_started_at_unix_seconds = unix_seconds_now()?;

    verify_executable_identity(
        executable_path,
        &executable_fingerprint,
        execution_policy.expected_verifier().executable_sha256,
    )?;
    let download = run_step(&steps[0], &secret_environment)?;
    let attestation_bundle_bytes = read_regular_bounded(
        command_plan.bundle_path(),
        MAX_GITHUB_EXECUTOR_RETAINED_FILE_BYTES_V1,
        "bundle_path",
    )?;

    verify_executable_identity(
        executable_path,
        &executable_fingerprint,
        execution_policy.expected_verifier().executable_sha256,
    )?;
    let trusted_root = run_step(&steps[1], &secret_environment)?;
    write_create_new_synced(command_plan.trusted_root_path(), &trusted_root.stdout)?;
    let retained_root_bytes = read_regular_bounded(
        command_plan.trusted_root_path(),
        MAX_GITHUB_EXECUTOR_RETAINED_FILE_BYTES_V1,
        "trusted_root_path",
    )?;
    if retained_root_bytes != trusted_root.stdout {
        return Err(GitHubVerifierExecutorErrorV1::TrustedRootWriteFailed);
    }
    let trusted_root_acquired_at_unix_seconds = unix_seconds_now()?;
    if trusted_root_acquired_at_unix_seconds < execution_started_at_unix_seconds {
        return Err(GitHubVerifierExecutorErrorV1::ClockRegression);
    }

    verify_executable_identity(
        executable_path,
        &executable_fingerprint,
        execution_policy.expected_verifier().executable_sha256,
    )?;
    let verify = run_step(&steps[2], &secret_environment)?;
    let execution_completed_at_unix_seconds = unix_seconds_now()?;
    if execution_completed_at_unix_seconds < trusted_root_acquired_at_unix_seconds {
        return Err(GitHubVerifierExecutorErrorV1::ClockRegression);
    }

    verify_executable_identity(
        executable_path,
        &executable_fingerprint,
        execution_policy.expected_verifier().executable_sha256,
    )?;
    if directory_fingerprint(work_root)? != work_fingerprint {
        return Err(GitHubVerifierExecutorErrorV1::WorkingDirectoryChangedDuringExecution);
    }
    if read_regular_bounded(
        command_plan.subject_path(),
        canonical_bytes.len(),
        "subject_path",
    )? != canonical_bytes
    {
        return Err(GitHubVerifierExecutorErrorV1::SubjectReadbackMismatch);
    }
    if read_regular_bounded(
        command_plan.bundle_path(),
        MAX_GITHUB_EXECUTOR_RETAINED_FILE_BYTES_V1,
        "bundle_path",
    )? != attestation_bundle_bytes
    {
        return Err(GitHubVerifierExecutorErrorV1::OutputReadFailed {
            field: "bundle_path",
        });
    }
    if read_regular_bounded(
        command_plan.trusted_root_path(),
        MAX_GITHUB_EXECUTOR_RETAINED_FILE_BYTES_V1,
        "trusted_root_path",
    )? != retained_root_bytes
    {
        return Err(GitHubVerifierExecutorErrorV1::OutputReadFailed {
            field: "trusted_root_path",
        });
    }

    let process_receipt = VerifierProcessExecutionReceiptV1 {
        evidence_version: VERIFIER_EXECUTION_EVIDENCE_VERSION_V1,
        verifier: execution_policy.expected_verifier().clone(),
        canonical_receipt_digest: command_plan.canonical_receipt_digest(),
        attestation_bundle_digest: raw_sha256(&attestation_bundle_bytes),
        trust_root_mode: VerifierTrustRootModeV1::OnlineFetchThenRetainedVerifyV1,
        trusted_root_material_digest: raw_sha256(&retained_root_bytes),
        trusted_root_acquired_at_unix_seconds,
        command_arguments_digest: command_plan.command_arguments_digest(),
        environment_profile_digest: command_plan.environment_profile_digest(),
        verifier_stdout_digest: raw_sha256(&verify.stdout),
        verifier_stderr_digest: raw_sha256(&verify.stderr),
        execution_started_at_unix_seconds,
        execution_completed_at_unix_seconds,
        process_outcome: VerifierProcessOutcomeV1::ExitCode(0),
    };
    process_receipt
        .validate()
        .map_err(GitHubVerifierExecutorErrorV1::ProcessReceiptInvalid)?;

    let step_evidence = [
        step_evidence(steps[0].purpose(), &download),
        step_evidence(steps[1].purpose(), &trusted_root),
        step_evidence(steps[2].purpose(), &verify),
    ];

    Ok(ExecutedGitHubVerificationV1 {
        profile_id: GITHUB_VERIFIER_EXECUTOR_PROFILE_V1,
        clock_profile_id: GITHUB_VERIFIER_EXECUTOR_CLOCK_PROFILE_V1,
        execution_policy: execution_policy.clone(),
        command_plan: command_plan.clone(),
        process_receipt,
        step_evidence,
        attestation_bundle_bytes,
        retained_root_bytes,
        download_stdout_bytes: download.stdout,
        download_stderr_bytes: download.stderr,
        trusted_root_stderr_bytes: trusted_root.stderr,
        verifier_stdout_bytes: verify.stdout,
        verifier_stderr_bytes: verify.stderr,
        authority: ExecutedGitHubVerificationAuthorityV1::ActualVerifierExecutionOnly,
    })
}

/// Bind sealed executor output into the existing strict GitHub result parser without
/// making the process executor interpret certificate/predicate semantics.
pub fn parse_executed_github_attestation_verifier_output_v1(
    executed: &ExecutedGitHubVerificationV1,
) -> Result<ParsedGitHubAttestationVerifierOutputV1, ExecutedGitHubVerificationParseErrorV1> {
    executed
        .process_receipt
        .validate()
        .map_err(ExecutedGitHubVerificationParseErrorV1::InvalidProcessReceipt)?;

    if executed.verifier_stdout_bytes.len() > crate::MAX_GITHUB_VERIFICATION_STDOUT_BYTES_V1 {
        return Err(ExecutedGitHubVerificationParseErrorV1::StdoutTooLarge {
            maximum: crate::MAX_GITHUB_VERIFICATION_STDOUT_BYTES_V1,
            actual: executed.verifier_stdout_bytes.len(),
        });
    }
    let raw: Value = serde_json::from_slice(&executed.verifier_stdout_bytes)
        .map_err(|_| ExecutedGitHubVerificationParseErrorV1::InvalidJson)?;
    let array = raw
        .as_array()
        .ok_or(ExecutedGitHubVerificationParseErrorV1::InvalidJson)?;
    if array.is_empty() {
        return Err(ExecutedGitHubVerificationParseErrorV1::EmptyResults);
    }
    if array.len() > crate::MAX_GITHUB_VERIFICATION_RESULTS_V1 {
        return Err(ExecutedGitHubVerificationParseErrorV1::TooManyResults {
            maximum: crate::MAX_GITHUB_VERIFICATION_RESULTS_V1,
            actual: array.len(),
        });
    }

    let execution_receipt = executed
        .process_receipt
        .clone()
        .into_parsed_receipt(array.len() as u32)
        .map_err(ExecutedGitHubVerificationParseErrorV1::ProcessReceiptPromotion)?;
    parse_github_attestation_verifier_output_v1(
        &executed.verifier_stdout_bytes,
        &execution_receipt,
    )
    .map_err(ExecutedGitHubVerificationParseErrorV1::StrictParser)
}

fn validate_step_graph(
    steps: &[GitHubVerifierExecutionStepV1; 3],
    plan: &GitHubPublicCommandPlanV1,
) -> Result<(), GitHubVerifierExecutorErrorV1> {
    if steps[0].purpose() != GitHubVerifierStepPurposeV1::DownloadAttestations
        || steps[1].purpose() != GitHubVerifierStepPurposeV1::FetchTrustedRoot
        || steps[2].purpose() != GitHubVerifierStepPurposeV1::VerifyAttestation
        || steps[0].expected_output_files() != [plan.bundle_path()]
        || steps[1].expected_output_files() != [plan.trusted_root_path()]
        || !steps[2].expected_output_files().is_empty()
        || !matches!(
            steps[0].stdout_disposition(),
            GitHubVerifierStdoutDispositionV1::CaptureBounded { .. }
        )
        || !matches!(
            steps[1].stdout_disposition(),
            GitHubVerifierStdoutDispositionV1::WriteFileBounded { path, .. }
                if path == plan.trusted_root_path()
        )
        || !matches!(
            steps[2].stdout_disposition(),
            GitHubVerifierStdoutDispositionV1::CaptureBounded { .. }
        )
    {
        return Err(GitHubVerifierExecutorErrorV1::UnexpectedStepGraph);
    }
    Ok(())
}

fn step_evidence(
    purpose: GitHubVerifierStepPurposeV1,
    output: &StepOutput,
) -> GitHubExecutedStepEvidenceV1 {
    GitHubExecutedStepEvidenceV1 {
        purpose,
        stdout_digest: raw_sha256(&output.stdout),
        stderr_digest: raw_sha256(&output.stderr),
        stdout_len: output.stdout.len(),
        stderr_len: output.stderr.len(),
    }
}

fn require_canonical_path(
    path: &Path,
    error: GitHubVerifierExecutorErrorV1,
) -> Result<(), GitHubVerifierExecutorErrorV1> {
    let canonical = fs::canonicalize(path).map_err(|_| error.clone())?;
    if canonical != path {
        return Err(error);
    }
    Ok(())
}

fn directory_fingerprint(
    path: &Path,
) -> Result<DirectoryFingerprint, GitHubVerifierExecutorErrorV1> {
    let metadata = fs::symlink_metadata(path)
        .map_err(|_| GitHubVerifierExecutorErrorV1::WorkingDirectoryMetadataUnavailable)?;
    if metadata.file_type().is_symlink() {
        return Err(GitHubVerifierExecutorErrorV1::WorkingDirectorySymlink);
    }
    if !metadata.is_dir() {
        return Err(GitHubVerifierExecutorErrorV1::WorkingDirectoryNotDirectory);
    }
    Ok(DirectoryFingerprint {
        #[cfg(unix)]
        device: metadata.dev(),
        #[cfg(unix)]
        inode: metadata.ino(),
        #[cfg(unix)]
        mode: metadata.mode(),
    })
}

fn executable_fingerprint(path: &Path) -> Result<FileFingerprint, GitHubVerifierExecutorErrorV1> {
    let metadata = fs::symlink_metadata(path)
        .map_err(|_| GitHubVerifierExecutorErrorV1::ExecutableMetadataUnavailable)?;
    if metadata.file_type().is_symlink() {
        return Err(GitHubVerifierExecutorErrorV1::ExecutableSymlink);
    }
    if !metadata.is_file() {
        return Err(GitHubVerifierExecutorErrorV1::ExecutableNotRegularFile);
    }
    #[cfg(unix)]
    if metadata.permissions().mode() & 0o111 == 0 {
        return Err(GitHubVerifierExecutorErrorV1::ExecutableNotMarkedExecutable);
    }
    Ok(FileFingerprint {
        len: metadata.len(),
        #[cfg(unix)]
        device: metadata.dev(),
        #[cfg(unix)]
        inode: metadata.ino(),
        #[cfg(unix)]
        mode: metadata.mode(),
    })
}

fn verify_executable_identity(
    path: &Path,
    expected_fingerprint: &FileFingerprint,
    expected_digest: Sha256DigestV1,
) -> Result<(), GitHubVerifierExecutorErrorV1> {
    let actual_fingerprint = executable_fingerprint(path)?;
    if &actual_fingerprint != expected_fingerprint {
        return Err(GitHubVerifierExecutorErrorV1::ExecutableChangedDuringExecution);
    }
    let digest = hash_file(path)
        .map_err(|_| GitHubVerifierExecutorErrorV1::ExecutableReadFailed)?;
    if digest != expected_digest {
        return Err(GitHubVerifierExecutorErrorV1::ExecutableChangedDuringExecution);
    }
    Ok(())
}

fn fixed_environment_path(
    plan: &GitHubPublicCommandPlanV1,
    key: &'static str,
) -> Result<String, GitHubVerifierExecutorErrorV1> {
    plan.fixed_environment()
        .iter()
        .find(|entry| entry.key() == key)
        .map(|entry| entry.value().to_owned())
        .ok_or(GitHubVerifierExecutorErrorV1::MissingFixedEnvironment { key })
}

fn validate_direct_child(
    work_root: &Path,
    child: &str,
    field: &'static str,
) -> Result<(), GitHubVerifierExecutorErrorV1> {
    let child = Path::new(child);
    if !child.is_absolute()
        || child.parent() != Some(work_root)
        || child.file_name().is_none()
        || child.components().any(|component| {
            matches!(
                component,
                std::path::Component::ParentDir | std::path::Component::CurDir
            )
        })
    {
        return Err(GitHubVerifierExecutorErrorV1::InvalidPlannedChildPath { field });
    }
    Ok(())
}

fn require_absent(
    path: impl AsRef<Path>,
    field: &'static str,
) -> Result<(), GitHubVerifierExecutorErrorV1> {
    match fs::symlink_metadata(path.as_ref()) {
        Ok(_) => Err(GitHubVerifierExecutorErrorV1::PreExistingPath { field }),
        Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(_) => Err(GitHubVerifierExecutorErrorV1::PreExistingPath { field }),
    }
}

fn materialize_subject(
    path: &str,
    bytes: &[u8],
) -> Result<(), GitHubVerifierExecutorErrorV1> {
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .map_err(|_| GitHubVerifierExecutorErrorV1::SubjectCreateFailed)?;
    file.write_all(bytes)
        .and_then(|_| file.sync_all())
        .map_err(|_| GitHubVerifierExecutorErrorV1::SubjectWriteFailed)?;
    let readback = read_regular_bounded(path, bytes.len(), "subject_path")?;
    if readback != bytes {
        return Err(GitHubVerifierExecutorErrorV1::SubjectReadbackMismatch);
    }
    Ok(())
}

fn write_create_new_synced(
    path: &str,
    bytes: &[u8],
) -> Result<(), GitHubVerifierExecutorErrorV1> {
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .map_err(|_| GitHubVerifierExecutorErrorV1::TrustedRootCreateFailed)?;
    file.write_all(bytes)
        .and_then(|_| file.sync_all())
        .map_err(|_| GitHubVerifierExecutorErrorV1::TrustedRootWriteFailed)
}

fn read_regular_bounded(
    path: impl AsRef<Path>,
    maximum: usize,
    field: &'static str,
) -> Result<Vec<u8>, GitHubVerifierExecutorErrorV1> {
    let path = path.as_ref();
    let metadata = fs::symlink_metadata(path).map_err(|error| {
        if error.kind() == io::ErrorKind::NotFound {
            GitHubVerifierExecutorErrorV1::OutputMissing { field }
        } else {
            GitHubVerifierExecutorErrorV1::OutputReadFailed { field }
        }
    })?;
    if metadata.file_type().is_symlink() {
        return Err(GitHubVerifierExecutorErrorV1::OutputSymlink { field });
    }
    if !metadata.is_file() {
        return Err(GitHubVerifierExecutorErrorV1::OutputNotRegularFile { field });
    }
    if metadata.len() > maximum as u64 {
        return Err(GitHubVerifierExecutorErrorV1::OutputTooLarge { field });
    }
    let file = File::open(path)
        .map_err(|_| GitHubVerifierExecutorErrorV1::OutputReadFailed { field })?;
    let mut bytes = Vec::with_capacity(metadata.len() as usize);
    file.take(maximum as u64 + 1)
        .read_to_end(&mut bytes)
        .map_err(|_| GitHubVerifierExecutorErrorV1::OutputReadFailed { field })?;
    if bytes.len() > maximum {
        return Err(GitHubVerifierExecutorErrorV1::OutputTooLarge { field });
    }
    Ok(bytes)
}

fn collect_inherited_secrets(
    plan: &GitHubPublicCommandPlanV1,
) -> Result<Vec<(String, std::ffi::OsString)>, GitHubVerifierExecutorErrorV1> {
    let keys = plan.inherited_secret_environment_keys();
    if keys.len() != 1 || keys[0] != "GH_TOKEN" {
        return Err(GitHubVerifierExecutorErrorV1::UnexpectedSecretEnvironmentPolicy);
    }
    let value = env::var_os("GH_TOKEN").ok_or_else(|| {
        GitHubVerifierExecutorErrorV1::MissingInheritedSecret {
            key: "GH_TOKEN".into(),
        }
    })?;
    if value.is_empty() {
        return Err(GitHubVerifierExecutorErrorV1::MissingInheritedSecret {
            key: "GH_TOKEN".into(),
        });
    }
    Ok(vec![("GH_TOKEN".into(), value)])
}

fn run_step(
    step: &GitHubVerifierExecutionStepV1,
    secret_environment: &[(String, std::ffi::OsString)],
) -> Result<StepOutput, GitHubVerifierExecutorErrorV1> {
    if !step.clear_environment() {
        return Err(GitHubVerifierExecutorErrorV1::EnvironmentMustBeCleared);
    }

    let stdout_maximum = match step.stdout_disposition() {
        GitHubVerifierStdoutDispositionV1::CaptureBounded { maximum_bytes }
        | GitHubVerifierStdoutDispositionV1::WriteFileBounded { maximum_bytes, .. } => {
            *maximum_bytes
        }
    };
    let purpose = step.purpose();
    let mut command = Command::new(step.executable_path());
    command
        .args(step.args())
        .current_dir(step.working_directory())
        .env_clear()
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    for assignment in step.fixed_environment() {
        command.env(assignment.key(), assignment.value());
    }
    for (key, value) in secret_environment {
        command.env(key, value);
    }

    let mut child = command
        .spawn()
        .map_err(|_| GitHubVerifierExecutorErrorV1::SpawnFailed { purpose })?;
    let stdout = child
        .stdout
        .take()
        .ok_or(GitHubVerifierExecutorErrorV1::MissingChildStdout { purpose })?;
    let stderr = child
        .stderr
        .take()
        .ok_or(GitHubVerifierExecutorErrorV1::MissingChildStderr { purpose })?;

    let (stdout_rx, stdout_handle) = spawn_bounded_reader(stdout, stdout_maximum);
    let (stderr_rx, stderr_handle) = spawn_bounded_reader(stderr, step.max_stderr_bytes());
    wait_for_step(
        &mut child,
        purpose,
        Duration::from_secs(step.timeout_seconds()),
        stdout_rx,
        stdout_handle,
        stderr_rx,
        stderr_handle,
    )
}

fn spawn_bounded_reader<R>(
    mut reader: R,
    maximum: usize,
) -> (
    Receiver<Result<Vec<u8>, BoundedReadError>>,
    JoinHandle<()>,
)
where
    R: Read + Send + 'static,
{
    let (tx, rx) = mpsc::channel();
    let handle = thread::spawn(move || {
        let mut bytes = Vec::with_capacity(maximum.min(8 * 1024));
        let mut buffer = [0_u8; 8 * 1024];
        loop {
            match reader.read(&mut buffer) {
                Ok(0) => {
                    let _ = tx.send(Ok(bytes));
                    return;
                }
                Ok(read) => {
                    if bytes.len().saturating_add(read) > maximum {
                        let _ = tx.send(Err(BoundedReadError::TooLarge));
                        return;
                    }
                    bytes.extend_from_slice(&buffer[..read]);
                }
                Err(_) => {
                    let _ = tx.send(Err(BoundedReadError::Io));
                    return;
                }
            }
        }
    });
    (rx, handle)
}

fn wait_for_step(
    child: &mut Child,
    purpose: GitHubVerifierStepPurposeV1,
    timeout: Duration,
    stdout_rx: Receiver<Result<Vec<u8>, BoundedReadError>>,
    stdout_handle: JoinHandle<()>,
    stderr_rx: Receiver<Result<Vec<u8>, BoundedReadError>>,
    stderr_handle: JoinHandle<()>,
) -> Result<StepOutput, GitHubVerifierExecutorErrorV1> {
    let started = Instant::now();
    let mut stdout_result = None;
    let mut stderr_result = None;

    let status = loop {
        if let Some(error) = poll_reader(&stdout_rx, &mut stdout_result, purpose, true)? {
            kill_and_wait(child, purpose)?;
            let _ = stdout_handle.join();
            let _ = stderr_handle.join();
            return Err(error);
        }
        if let Some(error) = poll_reader(&stderr_rx, &mut stderr_result, purpose, false)? {
            kill_and_wait(child, purpose)?;
            let _ = stdout_handle.join();
            let _ = stderr_handle.join();
            return Err(error);
        }

        if started.elapsed() >= timeout {
            kill_and_wait(child, purpose)?;
            let _ = stdout_handle.join();
            let _ = stderr_handle.join();
            return Err(GitHubVerifierExecutorErrorV1::ProcessTimedOut { purpose });
        }

        match child.try_wait() {
            Ok(Some(status)) => break status,
            Ok(None) => thread::sleep(PROCESS_POLL_INTERVAL),
            Err(_) => {
                kill_and_wait(child, purpose)?;
                let _ = stdout_handle.join();
                let _ = stderr_handle.join();
                return Err(GitHubVerifierExecutorErrorV1::ChildPollFailed { purpose });
            }
        }
    };

    let stdout_result = receive_reader_result(stdout_result, &stdout_rx, purpose)?;
    let stderr_result = receive_reader_result(stderr_result, &stderr_rx, purpose)?;
    if stdout_handle.join().is_err() || stderr_handle.join().is_err() {
        return Err(GitHubVerifierExecutorErrorV1::ReaderThreadFailed { purpose });
    }

    let stdout = map_reader_result(stdout_result, purpose, true)?;
    let stderr = map_reader_result(stderr_result, purpose, false)?;
    if !status.success() {
        return Err(GitHubVerifierExecutorErrorV1::NonZeroExit {
            purpose,
            code: status.code(),
            signal: exit_signal(&status),
        });
    }
    Ok(StepOutput { stdout, stderr })
}

fn poll_reader(
    receiver: &Receiver<Result<Vec<u8>, BoundedReadError>>,
    result: &mut Option<Result<Vec<u8>, BoundedReadError>>,
    purpose: GitHubVerifierStepPurposeV1,
    stdout: bool,
) -> Result<Option<GitHubVerifierExecutorErrorV1>, GitHubVerifierExecutorErrorV1> {
    if result.is_some() {
        return Ok(None);
    }
    match receiver.try_recv() {
        Ok(value) => match value {
            Ok(bytes) => {
                *result = Some(Ok(bytes));
                Ok(None)
            }
            Err(error) => Ok(Some(map_bounded_read_error(error, purpose, stdout))),
        },
        Err(TryRecvError::Empty) => Ok(None),
        Err(TryRecvError::Disconnected) => {
            Err(GitHubVerifierExecutorErrorV1::ReaderThreadFailed { purpose })
        }
    }
}

fn receive_reader_result(
    existing: Option<Result<Vec<u8>, BoundedReadError>>,
    receiver: &Receiver<Result<Vec<u8>, BoundedReadError>>,
    purpose: GitHubVerifierStepPurposeV1,
) -> Result<Result<Vec<u8>, BoundedReadError>, GitHubVerifierExecutorErrorV1> {
    match existing {
        Some(value) => Ok(value),
        None => receiver
            .recv()
            .map_err(|_| GitHubVerifierExecutorErrorV1::ReaderThreadFailed { purpose }),
    }
}

fn map_reader_result(
    result: Result<Vec<u8>, BoundedReadError>,
    purpose: GitHubVerifierStepPurposeV1,
    stdout: bool,
) -> Result<Vec<u8>, GitHubVerifierExecutorErrorV1> {
    result.map_err(|error| map_bounded_read_error(error, purpose, stdout))
}

fn map_bounded_read_error(
    error: BoundedReadError,
    purpose: GitHubVerifierStepPurposeV1,
    stdout: bool,
) -> GitHubVerifierExecutorErrorV1 {
    match (error, stdout) {
        (BoundedReadError::TooLarge, true) => {
            GitHubVerifierExecutorErrorV1::StdoutTooLarge { purpose }
        }
        (BoundedReadError::TooLarge, false) => {
            GitHubVerifierExecutorErrorV1::StderrTooLarge { purpose }
        }
        (BoundedReadError::Io, true) => {
            GitHubVerifierExecutorErrorV1::StdoutReadFailed { purpose }
        }
        (BoundedReadError::Io, false) => {
            GitHubVerifierExecutorErrorV1::StderrReadFailed { purpose }
        }
    }
}

fn kill_and_wait(
    child: &mut Child,
    purpose: GitHubVerifierStepPurposeV1,
) -> Result<(), GitHubVerifierExecutorErrorV1> {
    match child.kill() {
        Ok(()) => {}
        Err(error) if error.kind() == io::ErrorKind::InvalidInput => {}
        Err(_) => return Err(GitHubVerifierExecutorErrorV1::ChildKillFailed { purpose }),
    }
    child
        .wait()
        .map_err(|_| GitHubVerifierExecutorErrorV1::ChildWaitFailed { purpose })?;
    Ok(())
}

#[cfg(unix)]
fn exit_signal(status: &ExitStatus) -> Option<i32> {
    status.signal()
}

#[cfg(not(unix))]
fn exit_signal(_status: &ExitStatus) -> Option<i32> {
    None
}

fn unix_seconds_now() -> Result<u64, GitHubVerifierExecutorErrorV1> {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_secs())
        .map_err(|_| GitHubVerifierExecutorErrorV1::ClockBeforeUnixEpoch)
}

fn hash_file(path: &Path) -> Result<Sha256DigestV1, io::Error> {
    let mut file = File::open(path)?;
    let mut hasher = Sha256::new();
    let mut buffer = [0_u8; 64 * 1024];
    loop {
        let read = file.read(&mut buffer)?;
        if read == 0 {
            break;
        }
        hasher.update(&buffer[..read]);
    }
    Ok(Sha256DigestV1::from_bytes(hasher.finalize().into()))
}

fn raw_sha256(bytes: &[u8]) -> Sha256DigestV1 {
    Sha256DigestV1::from_bytes(Sha256::digest(bytes).into())
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;

    #[test]
    fn executor_authority_ceiling_is_explicit() {
        assert_eq!(
            GITHUB_VERIFIER_EXECUTOR_PROFILE_V1,
            "mycelix-github-verifier-executor-v1"
        );
        assert_eq!(
            GITHUB_VERIFIER_EXECUTOR_CLOCK_PROFILE_V1,
            "host-observed-system-time-v1"
        );
        assert_eq!(MAX_GITHUB_EXECUTOR_RETAINED_FILE_BYTES_V1, 4 * 1024 * 1024);
    }

    #[test]
    fn direct_child_validation_rejects_nested_or_parent_paths() {
        let root = PathBuf::from("/tmp/mycelix-executor");
        assert!(validate_direct_child(&root, "/tmp/mycelix-executor/a", "x").is_ok());
        assert!(validate_direct_child(&root, "/tmp/mycelix-executor/nested/a", "x").is_err());
        assert!(validate_direct_child(&root, "/tmp/a", "x").is_err());
    }

    #[test]
    fn raw_digest_is_exact_sha256() {
        assert_eq!(
            raw_sha256(b"abc").to_hex(),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }
}
