// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-004D3B2D2C: concrete host orchestration for one hermetic Forge run.
//!
//! Policy and evidence semantics remain in the existing Forge contracts. This
//! crate owns only the host lifecycle: sealed input transport, bubblewrap
//! control FDs, blocked parent observation, pidfd process binding, guest stdout
//! transport, post-run requalification, and ordered same-run evidence assembly.

use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_execution::{ExecutionContractError, ExecutionSpec};
use mycelix_forge_guest_envelope::{
    qualify_guest_evidence_envelope, GuestEnvelopeError, QualifiedGuestEvidenceEnvelope,
};
use mycelix_forge_guest_output_frame::{
    read_guest_envelope_frame, GuestOutputFrameError,
};
use mycelix_forge_guest_plan::{GuestPlanError, GuestVerificationPlanV1};
use mycelix_forge_guest_tool_map::{
    qualify_guest_tool_map, GuestToolMapError, GuestToolMapV1, QualifiedGuestToolMap,
};
use mycelix_forge_hermetic_run_evidence::{
    qualify_hermetic_run, HermeticRunError, HermeticRunObservation, HermeticRunPlan,
    QualifiedHermeticRunEvidence, RunPhase, RunPhaseEvidence,
};
use mycelix_forge_linux_isolation::{
    BubblewrapCommand, IsolationPolicyError, LinuxIsolationPolicyV1, NixClosureManifest,
    RuntimeArtifact, VerifierInvocation,
};
use mycelix_forge_linux_isolation_collector::{
    observe_parent_start, IsolationCollectorError, PendingParentObservation,
};
use mycelix_forge_linux_isolation_evidence::{
    qualify_linux_isolation, IsolationEvidenceError, QualifiedIsolationEvidence,
};
use mycelix_forge_nar_auditor::{
    audit_and_qualify_runtime_closure, NarAuditorError,
};
use mycelix_forge_runtime_closure_evidence::QualifiedRuntimeClosureEvidence;
use mycelix_forge_sealed_inputs::{
    build_sealed_bubblewrap_command, seal_runtime_inputs, QualifiedSealedInputEvidence,
    SealedInputError,
};
use semver::Version;
use serde::Deserialize;
use std::{
    fs::File,
    io::{BufRead, BufReader, Read, Write},
    os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd},
    path::{Path, PathBuf},
    process::{Child, Command, ExitStatus, Stdio},
    thread,
};
use thiserror::Error;

const HOST_EVIDENCE_DOMAIN_V1: &[u8] = b"mycelix-forge/hermetic-host-evidence/v1\0";
const BUBBLEWRAP_MIN_VERSION: &str = "0.10.0";
const HOST_EXECUTABLE_SUFFIX: &str = "/bin/forge-hermetic-host";
const BUBBLEWRAP_EXECUTABLE_SUFFIX: &str = "/bin/bwrap";
const MAX_STATUS_BYTES: usize = 64 * 1024;
const MAX_DIAGNOSTIC_BYTES: usize = 64 * 1024;
const PIDFD_EXIT_TIMEOUT_MS: i32 = 5_000;

pub struct HermeticHostInputs<'a> {
    pub execution_spec: &'a ExecutionSpec,
    pub run_plan: &'a HermeticRunPlan,
    pub guest_plan: &'a GuestVerificationPlanV1,
    pub guest_tool_map: &'a GuestToolMapV1,
    pub isolation_policy: &'a LinuxIsolationPolicyV1,
    pub closure: &'a NixClosureManifest,
    pub invocation: &'a VerifierInvocation,
    pub runtime_artifacts: &'a [RuntimeArtifact],
    pub bubblewrap_program: &'a Path,
}

pub struct QualifiedHermeticHostRun {
    evidence_digest: Digest,
    execution_spec_digest: Digest,
    run_plan_digest: Digest,
    host_executable: String,
    bubblewrap_executable: String,
    child_pid: u32,
    start_status_commitment: Digest,
    pidfd_observed_exit: bool,
    sealed_inputs: QualifiedSealedInputEvidence,
    guest_tools: QualifiedGuestToolMap,
    guest: QualifiedGuestEvidenceEnvelope,
    pre_runtime: QualifiedRuntimeClosureEvidence,
    isolation: QualifiedIsolationEvidence,
    post_runtime: QualifiedRuntimeClosureEvidence,
    same_run: QualifiedHermeticRunEvidence,
}

impl QualifiedHermeticHostRun {
    pub fn evidence_digest(&self) -> &Digest {
        &self.evidence_digest
    }

    pub fn execution_spec_digest(&self) -> &Digest {
        &self.execution_spec_digest
    }

    pub fn run_plan_digest(&self) -> &Digest {
        &self.run_plan_digest
    }

    pub fn host_executable(&self) -> &str {
        &self.host_executable
    }

    pub fn bubblewrap_executable(&self) -> &str {
        &self.bubblewrap_executable
    }

    pub const fn child_pid(&self) -> u32 {
        self.child_pid
    }

    pub fn start_status_commitment(&self) -> &Digest {
        &self.start_status_commitment
    }

    pub const fn pidfd_observed_exit(&self) -> bool {
        self.pidfd_observed_exit
    }

    pub fn sealed_inputs(&self) -> &QualifiedSealedInputEvidence {
        &self.sealed_inputs
    }

    pub fn guest_tools(&self) -> &QualifiedGuestToolMap {
        &self.guest_tools
    }

    pub fn guest(&self) -> &QualifiedGuestEvidenceEnvelope {
        &self.guest
    }

    pub fn pre_runtime(&self) -> &QualifiedRuntimeClosureEvidence {
        &self.pre_runtime
    }

    pub fn isolation(&self) -> &QualifiedIsolationEvidence {
        &self.isolation
    }

    pub fn post_runtime(&self) -> &QualifiedRuntimeClosureEvidence {
        &self.post_runtime
    }

    pub fn same_run(&self) -> &QualifiedHermeticRunEvidence {
        &self.same_run
    }
}

pub fn execute_hermetic_guest(
    inputs: HermeticHostInputs<'_>,
) -> Result<QualifiedHermeticHostRun, HermeticHostError> {
    inputs
        .isolation_policy
        .validate_dependencies(inputs.closure, inputs.invocation)?;
    let execution_spec_digest = inputs.execution_spec.digest(DigestAlgorithm::Sha256)?;
    let run_plan_digest = inputs.run_plan.digest(DigestAlgorithm::Sha256)?;
    let isolation_policy_digest = inputs
        .isolation_policy
        .digest(DigestAlgorithm::Sha256)?;
    require_run_plan_links(
        inputs.run_plan,
        inputs.guest_plan,
        &execution_spec_digest,
        &isolation_policy_digest,
    )?;

    let (host_executable, bubblewrap_executable) = require_host_executables(
        inputs.execution_spec,
        inputs.closure,
        inputs.bubblewrap_program,
    )?;
    let guest_tools = qualify_guest_tool_map(
        inputs.guest_tool_map,
        inputs.guest_plan,
        inputs.closure,
        inputs.execution_spec,
    )?;

    let pre_runtime = audit_and_qualify_runtime_closure(inputs.closure)?;
    let mut sealed = seal_runtime_inputs(inputs.isolation_policy, inputs.runtime_artifacts)?;
    let sealed_inputs = sealed.qualified().clone();
    let command = build_sealed_bubblewrap_command(
        inputs.bubblewrap_program,
        inputs.isolation_policy,
        inputs.closure,
        inputs.invocation,
        &mut sealed,
    )?;

    let (status_read, status_write) = pipe_cloexec()?;
    let (block_read, block_write) = pipe_cloexec()?;
    clear_cloexec(status_write.as_raw_fd())?;
    clear_cloexec(block_read.as_raw_fd())?;

    let status_write_fd = status_write.as_raw_fd();
    let block_read_fd = block_read.as_raw_fd();
    let controlled = add_control_fds(command, status_write_fd, block_read_fd);
    let mut child = spawn_bubblewrap(controlled)?;
    let stdout = child
        .stdout
        .take()
        .ok_or(HermeticHostError::MissingChildStdout)?;
    let stderr = child
        .stderr
        .take()
        .ok_or(HermeticHostError::MissingChildStderr)?;
    let stdout_thread = thread::spawn(move || read_guest_envelope_frame(stdout));
    let stderr_thread = thread::spawn(move || read_limited(stderr, MAX_DIAGNOSTIC_BYTES));
    let mut guard = ChildGuard::new(child);

    // The parent no longer needs the child-side pipe ends after exec. Their
    // inherited copies remain in bubblewrap because CLOEXEC was cleared only
    // on those exact ends.
    drop(status_write);
    drop(block_read);

    let status_file: File = status_read.into();
    let mut status_reader = BufReader::new(status_file);
    let pending = read_parent_start(&mut status_reader)?;
    let child_pid = pending.child_pid();
    let start_status_commitment = pending.start_status_commitment().clone();

    let pidfd = pidfd_open(child_pid)?;
    if pidfd_ready(&pidfd, 0)? {
        return Err(HermeticHostError::ChildExitedWhileBlocked(child_pid));
    }

    let mut block_writer: File = block_write.into();
    block_writer.write_all(&[1])?;
    block_writer.flush()?;
    drop(block_writer);

    let bubblewrap_status = guard.wait()?;
    let remaining_status = read_limited(&mut status_reader, MAX_STATUS_BYTES)?;
    let frame_result = stdout_thread
        .join()
        .map_err(|_| HermeticHostError::ReaderThreadPanicked)?;
    let stderr_bytes = stderr_thread
        .join()
        .map_err(|_| HermeticHostError::ReaderThreadPanicked)??;

    if !bubblewrap_status.success() {
        return Err(HermeticHostError::BubblewrapFailed {
            status: bubblewrap_status.code().unwrap_or(-1),
            stderr: String::from_utf8_lossy(&stderr_bytes).into_owned(),
        });
    }
    if !pidfd_ready(&pidfd, PIDFD_EXIT_TIMEOUT_MS)? {
        return Err(HermeticHostError::PidfdDidNotObserveExit(child_pid));
    }
    let envelope = frame_result?;

    let parent = pending.finish(&remaining_status)?;
    let guest = qualify_guest_evidence_envelope(
        &envelope,
        inputs.guest_plan,
        inputs.guest_tool_map,
    )?;
    if guest.tool_map_digest() != guest_tools.map_digest() {
        return Err(HermeticHostError::GuestToolMapQualificationMismatch);
    }

    let isolation = qualify_linux_isolation(
        inputs.isolation_policy,
        inputs.closure,
        inputs.invocation,
        envelope.inside_isolation(),
        &parent,
    )?;
    if isolation.inside_digest() != guest.inside_digest() {
        return Err(HermeticHostError::InsideIsolationMismatch);
    }

    let post_runtime = audit_and_qualify_runtime_closure(inputs.closure)?;
    if pre_runtime.evidence_digest() != post_runtime.evidence_digest() {
        return Err(HermeticHostError::RuntimeClosureChangedDuringRun);
    }

    let phases = vec![
        RunPhaseEvidence::new(
            RunPhase::PreRuntimeClosure,
            pre_runtime.evidence_digest().clone(),
        ),
        RunPhaseEvidence::new(
            RunPhase::GitObjectValidation,
            guest.transcript().git_object_validation().clone(),
        ),
        RunPhaseEvidence::new(
            RunPhase::PolicyTrustQualification,
            guest.trust().evidence_digest().clone(),
        ),
        RunPhaseEvidence::new(RunPhase::IsolationProbe, guest.inside_digest().clone()),
        RunPhaseEvidence::new(
            RunPhase::VerifierExecution,
            guest.transcript().evidence_digest().clone(),
        ),
        RunPhaseEvidence::new(
            RunPhase::IsolationQualification,
            isolation.evidence_digest().clone(),
        ),
        RunPhaseEvidence::new(
            RunPhase::PostRuntimeClosure,
            post_runtime.evidence_digest().clone(),
        ),
    ];
    let observation = HermeticRunObservation::new(
        run_plan_digest.clone(),
        inputs.run_plan.run_challenge().clone(),
        child_pid,
        start_status_commitment.clone(),
        true,
        true,
        phases,
        bubblewrap_status.code().unwrap_or(-1),
    )?;
    let same_run = qualify_hermetic_run(inputs.run_plan, &observation)?;

    let evidence_digest = derive_host_evidence(
        &execution_spec_digest,
        &run_plan_digest,
        &host_executable,
        &bubblewrap_executable,
        child_pid,
        &start_status_commitment,
        sealed_inputs.evidence_digest(),
        guest_tools.map_digest(),
        guest.envelope_digest(),
        pre_runtime.evidence_digest(),
        isolation.evidence_digest(),
        post_runtime.evidence_digest(),
        same_run.evidence_digest(),
    )?;

    Ok(QualifiedHermeticHostRun {
        evidence_digest,
        execution_spec_digest,
        run_plan_digest,
        host_executable,
        bubblewrap_executable,
        child_pid,
        start_status_commitment,
        pidfd_observed_exit: true,
        sealed_inputs,
        guest_tools,
        guest,
        pre_runtime,
        isolation,
        post_runtime,
        same_run,
    })
}

fn require_run_plan_links(
    run_plan: &HermeticRunPlan,
    guest_plan: &GuestVerificationPlanV1,
    execution_spec: &Digest,
    isolation_policy: &Digest,
) -> Result<(), HermeticHostError> {
    if run_plan.execution_spec() != execution_spec {
        return Err(HermeticHostError::RunPlanExecutionSpecMismatch);
    }
    if run_plan.isolation_policy() != isolation_policy {
        return Err(HermeticHostError::RunPlanIsolationPolicyMismatch);
    }
    if run_plan.git_object_validation_policy() != guest_plan.git_object_validation_policy() {
        return Err(HermeticHostError::RunPlanGitPolicyMismatch);
    }
    if run_plan.run_challenge() != guest_plan.run_challenge() {
        return Err(HermeticHostError::RunChallengeMismatch);
    }
    Ok(())
}

fn require_host_executables(
    spec: &ExecutionSpec,
    closure: &NixClosureManifest,
    bubblewrap_program: &Path,
) -> Result<(String, String), HermeticHostError> {
    let host = std::env::current_exe()?;
    let host = host
        .to_str()
        .ok_or(HermeticHostError::NonUtf8ExecutablePath)?
        .to_owned();
    let bubblewrap = bubblewrap_program
        .to_str()
        .ok_or(HermeticHostError::NonUtf8ExecutablePath)?
        .to_owned();
    validate_exact_store_executable(&host, HOST_EXECUTABLE_SUFFIX, closure)?;
    validate_exact_store_executable(&bubblewrap, BUBBLEWRAP_EXECUTABLE_SUFFIX, closure)?;

    if !spec.tools().iter().any(|tool| tool.role() == "forge-hermetic-host") {
        return Err(HermeticHostError::MissingExecutionTool(
            "forge-hermetic-host".into(),
        ));
    }
    let bubblewrap_tool = spec
        .tools()
        .iter()
        .find(|tool| tool.role() == "bubblewrap")
        .ok_or_else(|| HermeticHostError::MissingExecutionTool("bubblewrap".into()))?;
    let version = Version::parse(bubblewrap_tool.semantic_version())?;
    if version < Version::parse(BUBBLEWRAP_MIN_VERSION).expect("fixed version is valid") {
        return Err(HermeticHostError::BubblewrapVersionTooOld(
            bubblewrap_tool.semantic_version().to_owned(),
        ));
    }
    Ok((host, bubblewrap))
}

fn validate_exact_store_executable(
    executable: &str,
    suffix: &str,
    closure: &NixClosureManifest,
) -> Result<(), HermeticHostError> {
    if !executable.starts_with("/nix/store/")
        || !executable.ends_with(suffix)
        || executable.contains("//")
        || executable.contains("/../")
        || executable.contains("/./")
    {
        return Err(HermeticHostError::InvalidHostExecutable(
            executable.to_owned(),
        ));
    }
    let tail = executable
        .strip_prefix("/nix/store/")
        .ok_or_else(|| HermeticHostError::InvalidHostExecutable(executable.to_owned()))?;
    let (entry, _) = tail
        .split_once('/')
        .ok_or_else(|| HermeticHostError::InvalidHostExecutable(executable.to_owned()))?;
    let root = format!("/nix/store/{entry}");
    if !closure
        .entries()
        .iter()
        .any(|candidate| candidate.store_path() == root)
    {
        return Err(HermeticHostError::HostExecutableOutsideClosure(
            executable.to_owned(),
        ));
    }
    Ok(())
}

fn add_control_fds(
    mut command: BubblewrapCommand,
    status_fd: RawFd,
    block_fd: RawFd,
) -> BubblewrapCommand {
    command.args.splice(
        0..0,
        [
            "--json-status-fd".to_owned(),
            status_fd.to_string(),
            "--block-fd".to_owned(),
            block_fd.to_string(),
        ],
    );
    command
}

fn spawn_bubblewrap(command: BubblewrapCommand) -> Result<Child, HermeticHostError> {
    let mut process = Command::new(&command.program);
    process
        .args(&command.args)
        .env_clear()
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    Ok(process.spawn()?)
}

fn pipe_cloexec() -> Result<(OwnedFd, OwnedFd), HermeticHostError> {
    let mut fds = [0_i32; 2];
    let result = unsafe { libc::pipe2(fds.as_mut_ptr(), libc::O_CLOEXEC) };
    if result != 0 {
        return Err(std::io::Error::last_os_error().into());
    }
    let read = unsafe { OwnedFd::from_raw_fd(fds[0]) };
    let write = unsafe { OwnedFd::from_raw_fd(fds[1]) };
    Ok((read, write))
}

fn clear_cloexec(fd: RawFd) -> Result<(), HermeticHostError> {
    let flags = unsafe { libc::fcntl(fd, libc::F_GETFD) };
    if flags < 0 {
        return Err(std::io::Error::last_os_error().into());
    }
    if unsafe { libc::fcntl(fd, libc::F_SETFD, flags & !libc::FD_CLOEXEC) } < 0 {
        return Err(std::io::Error::last_os_error().into());
    }
    Ok(())
}

fn read_parent_start(
    reader: &mut BufReader<File>,
) -> Result<PendingParentObservation, HermeticHostError> {
    let mut status = Vec::new();
    loop {
        let mut line = Vec::new();
        let read = reader.read_until(b'\n', &mut line)?;
        if read == 0 {
            return Err(HermeticHostError::MissingChildStartStatus);
        }
        status.extend_from_slice(&line);
        if status.len() > MAX_STATUS_BYTES {
            return Err(HermeticHostError::StatusStreamTooLarge);
        }
        match observe_parent_start(&status) {
            Ok(pending) => return Ok(pending),
            Err(IsolationCollectorError::MissingChildPid) => continue,
            Err(error) => return Err(error.into()),
        }
    }
}

fn pidfd_open(pid: u32) -> Result<OwnedFd, HermeticHostError> {
    let pid = i32::try_from(pid).map_err(|_| HermeticHostError::InvalidChildPid)?;
    let raw = unsafe { libc::syscall(libc::SYS_pidfd_open, pid, 0) };
    if raw < 0 {
        return Err(std::io::Error::last_os_error().into());
    }
    Ok(unsafe { OwnedFd::from_raw_fd(raw as RawFd) })
}

fn pidfd_ready(pidfd: &OwnedFd, timeout_ms: i32) -> Result<bool, HermeticHostError> {
    let mut entry = libc::pollfd {
        fd: pidfd.as_raw_fd(),
        events: libc::POLLIN,
        revents: 0,
    };
    let result = unsafe { libc::poll(&mut entry, 1, timeout_ms) };
    if result < 0 {
        return Err(std::io::Error::last_os_error().into());
    }
    if entry.revents & libc::POLLNVAL != 0 {
        return Err(HermeticHostError::InvalidPidfd);
    }
    Ok(result > 0
        && entry.revents & (libc::POLLIN | libc::POLLHUP | libc::POLLERR) != 0)
}

fn read_limited<R: Read>(mut reader: R, max: usize) -> Result<Vec<u8>, std::io::Error> {
    let limit = u64::try_from(max).unwrap_or(u64::MAX).saturating_add(1);
    let mut bytes = Vec::new();
    reader.by_ref().take(limit).read_to_end(&mut bytes)?;
    if bytes.len() > max {
        bytes.truncate(max);
    }
    Ok(bytes)
}

struct ChildGuard {
    child: Option<Child>,
}

impl ChildGuard {
    fn new(child: Child) -> Self {
        Self { child: Some(child) }
    }

    fn wait(&mut self) -> Result<ExitStatus, std::io::Error> {
        let status = self
            .child
            .as_mut()
            .expect("child exists until successful wait")
            .wait()?;
        self.child.take();
        Ok(status)
    }
}

impl Drop for ChildGuard {
    fn drop(&mut self) {
        if let Some(mut child) = self.child.take() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn derive_host_evidence(
    execution_spec: &Digest,
    run_plan: &Digest,
    host_executable: &str,
    bubblewrap_executable: &str,
    child_pid: u32,
    start_status: &Digest,
    sealed_inputs: &Digest,
    guest_tool_map: &Digest,
    guest_envelope: &Digest,
    pre_runtime: &Digest,
    isolation: &Digest,
    post_runtime: &Digest,
    same_run: &Digest,
) -> Result<Digest, HermeticHostError> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(HOST_EVIDENCE_DOMAIN_V1);
    push_digest(&mut bytes, execution_spec)?;
    push_digest(&mut bytes, run_plan)?;
    push_string(&mut bytes, host_executable)?;
    push_string(&mut bytes, bubblewrap_executable)?;
    bytes.extend_from_slice(&child_pid.to_be_bytes());
    push_digest(&mut bytes, start_status)?;
    bytes.push(1); // pidfd was bound before release and observed exit.
    for digest in [
        sealed_inputs,
        guest_tool_map,
        guest_envelope,
        pre_runtime,
        isolation,
        post_runtime,
        same_run,
    ] {
        push_digest(&mut bytes, digest)?;
    }
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &bytes))
}

fn push_string(out: &mut Vec<u8>, value: &str) -> Result<(), HermeticHostError> {
    let len = u16::try_from(value.len()).map_err(|_| HermeticHostError::CanonicalLengthOverflow)?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), HermeticHostError> {
    push_string(out, digest.algorithm().id())?;
    let len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| HermeticHostError::CanonicalLengthOverflow)?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

#[derive(Clone, Debug, Deserialize)]
pub struct HostRuntimeArtifactPath {
    pub role: String,
    pub source: PathBuf,
}

#[derive(Clone, Debug, Deserialize)]
pub struct HostLaunchManifestV1 {
    pub execution_spec: PathBuf,
    pub run_plan: PathBuf,
    pub guest_plan: PathBuf,
    pub guest_tool_map: PathBuf,
    pub isolation_policy: PathBuf,
    pub nix_closure: PathBuf,
    pub sandbox_invocation: PathBuf,
    pub bubblewrap_program: PathBuf,
    pub runtime_artifacts: Vec<HostRuntimeArtifactPath>,
}

pub fn execute_from_launch_manifest(
    manifest_path: &Path,
) -> Result<QualifiedHermeticHostRun, HermeticHostError> {
    let manifest: HostLaunchManifestV1 = load_json(manifest_path)?;
    let execution_spec: ExecutionSpec = load_json(&manifest.execution_spec)?;
    let run_plan: HermeticRunPlan = load_json(&manifest.run_plan)?;
    let guest_plan: GuestVerificationPlanV1 = load_json(&manifest.guest_plan)?;
    let guest_tool_map: GuestToolMapV1 = load_json(&manifest.guest_tool_map)?;
    let isolation_policy: LinuxIsolationPolicyV1 = load_json(&manifest.isolation_policy)?;
    let closure: NixClosureManifest = load_json(&manifest.nix_closure)?;
    let invocation: VerifierInvocation = load_json(&manifest.sandbox_invocation)?;
    let runtime_artifacts = manifest
        .runtime_artifacts
        .into_iter()
        .map(|artifact| RuntimeArtifact {
            role: artifact.role,
            source: artifact.source,
        })
        .collect::<Vec<_>>();

    execute_hermetic_guest(HermeticHostInputs {
        execution_spec: &execution_spec,
        run_plan: &run_plan,
        guest_plan: &guest_plan,
        guest_tool_map: &guest_tool_map,
        isolation_policy: &isolation_policy,
        closure: &closure,
        invocation: &invocation,
        runtime_artifacts: &runtime_artifacts,
        bubblewrap_program: &manifest.bubblewrap_program,
    })
}

fn load_json<T: serde::de::DeserializeOwned>(path: &Path) -> Result<T, HermeticHostError> {
    let bytes = std::fs::read(path)?;
    Ok(serde_json::from_slice(&bytes)?)
}

#[derive(Debug, Error)]
pub enum HermeticHostError {
    #[error(transparent)]
    Execution(#[from] ExecutionContractError),
    #[error(transparent)]
    GuestPlan(#[from] GuestPlanError),
    #[error(transparent)]
    GuestToolMap(#[from] GuestToolMapError),
    #[error(transparent)]
    GuestEnvelope(#[from] GuestEnvelopeError),
    #[error(transparent)]
    GuestOutput(#[from] GuestOutputFrameError),
    #[error(transparent)]
    HermeticRun(#[from] HermeticRunError),
    #[error(transparent)]
    IsolationPolicy(#[from] IsolationPolicyError),
    #[error(transparent)]
    IsolationCollector(#[from] IsolationCollectorError),
    #[error(transparent)]
    IsolationEvidence(#[from] IsolationEvidenceError),
    #[error(transparent)]
    NarAuditor(#[from] NarAuditorError),
    #[error(transparent)]
    SealedInputs(#[from] SealedInputError),
    #[error(transparent)]
    Semver(#[from] semver::Error),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error("same-run plan names a different ExecutionSpec")]
    RunPlanExecutionSpecMismatch,
    #[error("same-run plan names a different isolation policy")]
    RunPlanIsolationPolicyMismatch,
    #[error("same-run plan and guest plan use different Git validation policies")]
    RunPlanGitPolicyMismatch,
    #[error("same-run plan and guest plan use different challenges")]
    RunChallengeMismatch,
    #[error("execution spec is missing required tool role {0}")]
    MissingExecutionTool(String),
    #[error("bubblewrap version does not support required FD bind transport: {0}")]
    BubblewrapVersionTooOld(String),
    #[error("non-UTF-8 executable path")]
    NonUtf8ExecutablePath,
    #[error("invalid exact host executable path: {0}")]
    InvalidHostExecutable(String),
    #[error("host executable lies outside the committed Nix closure: {0}")]
    HostExecutableOutsideClosure(String),
    #[error("bubblewrap child stdout was not piped")]
    MissingChildStdout,
    #[error("bubblewrap child stderr was not piped")]
    MissingChildStderr,
    #[error("reader thread panicked")]
    ReaderThreadPanicked,
    #[error("bubblewrap status stream ended before reporting child-pid")]
    MissingChildStartStatus,
    #[error("bubblewrap status stream exceeded the host limit")]
    StatusStreamTooLarge,
    #[error("bubblewrap child PID does not fit pid_t")]
    InvalidChildPid,
    #[error("invalid pidfd")]
    InvalidPidfd,
    #[error("sandbox child exited while still expected to be blocked: {0}")]
    ChildExitedWhileBlocked(u32),
    #[error("pidfd did not observe sandbox child exit: {0}")]
    PidfdDidNotObserveExit(u32),
    #[error("bubblewrap failed: status={status}, stderr={stderr}")]
    BubblewrapFailed { status: i32, stderr: String },
    #[error("qualified guest tool map differs from the envelope tool-map subject")]
    GuestToolMapQualificationMismatch,
    #[error("parent-qualified inside evidence differs from guest envelope")]
    InsideIsolationMismatch,
    #[error("runtime NAR closure changed between preflight and postflight")]
    RuntimeClosureChangedDuringRun,
    #[error("canonical host evidence field length overflow")]
    CanonicalLengthOverflow,
}
