// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-004D4A: acyclic sealed capsule transport for Forge M0.
//!
//! The Linux isolation policy owns only the five data/runtime artifacts that
//! are independent of the guest plan. Three control-plane files are sealed and
//! mounted read-only outside the policy artifact set:
//!
//! - guest verification plan JSON;
//! - guest tool map JSON;
//! - Linux isolation policy JSON itself.
//!
//! This removes the former policy -> plan/tool-map/policy-bytes hash cycle while
//! preserving exact byte identity for all eight ExecutionSpec inputs.

use mycelix_forge_core::{Digest, DigestAlgorithm, ForgeCoreError};
use mycelix_forge_execution::{ExecutionContractError, ExecutionSpec};
use mycelix_forge_guest_plan::{
    GUEST_BUNDLE_PATH, GUEST_ISOLATION_POLICY_PATH, GUEST_MANIFEST_PATH,
    GUEST_NIX_CLOSURE_PATH, GUEST_PLAN_PATH, GUEST_RUN_CHALLENGE_PATH,
    GUEST_SANDBOX_INVOCATION_PATH,
};
use mycelix_forge_linux_isolation::{
    ArtifactMount, BubblewrapCommand, IsolationPolicyError, LinuxIsolationPolicyV1,
    NixClosureManifest, RuntimeArtifact, VerifierInvocation,
};
use mycelix_forge_sealed_inputs::{
    build_sealed_bubblewrap_command, seal_runtime_inputs, SealedInputError, SealedRuntimeInputs,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest as ShaDigest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    ffi::CString,
    fs::File,
    io::{Read, Seek, SeekFrom, Write},
    os::fd::{AsRawFd, FromRawFd, RawFd},
    path::{Path, PathBuf},
};
use thiserror::Error;

const CAPSULE_EVIDENCE_DOMAIN_V1: &[u8] = b"mycelix-forge/sealed-m0-capsule-evidence/v1\0";
const CONTROL_EVIDENCE_DOMAIN_V1: &[u8] = b"mycelix-forge/sealed-m0-control-inputs/v1\0";
const REQUIRED_SEALS: i32 =
    libc::F_SEAL_WRITE | libc::F_SEAL_GROW | libc::F_SEAL_SHRINK | libc::F_SEAL_SEAL;

pub const GUEST_TOOL_MAP_PATH: &str = "/inputs/guest-tool-map.json";

/// Exact artifact set whose byte identities are part of Linux isolation policy.
/// None of these depends on the guest plan or on the policy JSON bytes.
pub const POLICY_INPUTS: &[(&str, &str)] = &[
    ("nix-closure-manifest", GUEST_NIX_CLOSURE_PATH),
    ("repository-bundle", GUEST_BUNDLE_PATH),
    ("repository-bundle-manifest", GUEST_MANIFEST_PATH),
    ("run-challenge", GUEST_RUN_CHALLENGE_PATH),
    ("sandbox-invocation", GUEST_SANDBOX_INVOCATION_PATH),
];

/// Control-plane files are still exact sealed ExecutionSpec inputs, but are not
/// ancestors of the policy digest they carry/use.
pub const CONTROL_INPUTS: &[(&str, &str)] = &[
    ("guest-tool-map", GUEST_TOOL_MAP_PATH),
    ("guest-verification-plan", GUEST_PLAN_PATH),
    ("linux-isolation-policy", GUEST_ISOLATION_POLICY_PATH),
];

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct SealedControlObservation {
    role: String,
    destination: String,
    digest: Digest,
    size: u64,
    seals: i32,
}

impl SealedControlObservation {
    pub fn role(&self) -> &str {
        &self.role
    }

    pub fn destination(&self) -> &str {
        &self.destination
    }

    pub fn digest(&self) -> &Digest {
        &self.digest
    }

    pub const fn size(&self) -> u64 {
        self.size
    }

    pub const fn seals(&self) -> i32 {
        self.seals
    }

    pub const fn fully_sealed(&self) -> bool {
        self.seals & REQUIRED_SEALS == REQUIRED_SEALS
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualifiedSealedM0CapsuleEvidence {
    execution_spec_digest: Digest,
    policy_digest: Digest,
    policy_input_evidence: Digest,
    control_input_evidence: Digest,
    controls: Vec<SealedControlObservation>,
    evidence_digest: Digest,
}

impl QualifiedSealedM0CapsuleEvidence {
    pub fn execution_spec_digest(&self) -> &Digest {
        &self.execution_spec_digest
    }

    pub fn policy_digest(&self) -> &Digest {
        &self.policy_digest
    }

    pub fn policy_input_evidence(&self) -> &Digest {
        &self.policy_input_evidence
    }

    pub fn control_input_evidence(&self) -> &Digest {
        &self.control_input_evidence
    }

    pub fn controls(&self) -> &[SealedControlObservation] {
        &self.controls
    }

    pub fn evidence_digest(&self) -> &Digest {
        &self.evidence_digest
    }
}

struct SealedControlArtifact {
    role: String,
    destination: String,
    file: File,
    observation: SealedControlObservation,
}

impl SealedControlArtifact {
    fn raw_fd(&self) -> RawFd {
        self.file.as_raw_fd()
    }

    fn rewind(&mut self) -> Result<(), SealedCapsuleError> {
        self.file.seek(SeekFrom::Start(0))?;
        Ok(())
    }
}

pub struct SealedM0CapsuleInputs {
    execution_spec_digest: Digest,
    policy_digest: Digest,
    policy_inputs: SealedRuntimeInputs,
    controls: Vec<SealedControlArtifact>,
    qualified: QualifiedSealedM0CapsuleEvidence,
}

impl SealedM0CapsuleInputs {
    pub fn execution_spec_digest(&self) -> &Digest {
        &self.execution_spec_digest
    }

    pub fn policy_digest(&self) -> &Digest {
        &self.policy_digest
    }

    pub fn qualified(&self) -> &QualifiedSealedM0CapsuleEvidence {
        &self.qualified
    }
}

/// Seal the exact eight M0 ExecutionSpec inputs while preserving an acyclic
/// policy surface: five artifacts belong to policy, three to control transport.
pub fn seal_m0_capsule_inputs(
    spec: &ExecutionSpec,
    policy: &LinuxIsolationPolicyV1,
    runtime_artifacts: &[RuntimeArtifact],
) -> Result<SealedM0CapsuleInputs, SealedCapsuleError> {
    require_exact_spec_surface(spec)?;
    require_exact_policy_surface(policy)?;

    let runtime = runtime_artifacts
        .iter()
        .map(|artifact| (artifact.role.as_str(), artifact.source.as_path()))
        .collect::<BTreeMap<_, _>>();
    if runtime.len() != runtime_artifacts.len() {
        return Err(SealedCapsuleError::DuplicateRuntimeRole);
    }
    let actual_roles = runtime.keys().copied().collect::<BTreeSet<_>>();
    if actual_roles != all_input_roles() {
        return Err(SealedCapsuleError::RuntimeRoleSetMismatch);
    }

    let policy_runtime = policy
        .artifact_mounts()
        .iter()
        .map(|mount| RuntimeArtifact {
            role: mount.role().to_owned(),
            source: runtime
                .get(mount.role())
                .expect("exact runtime role set checked")
                .to_path_buf(),
        })
        .collect::<Vec<_>>();
    let policy_inputs = seal_runtime_inputs(policy, &policy_runtime)?;

    let mut controls = Vec::with_capacity(CONTROL_INPUTS.len());
    for (role, destination) in CONTROL_INPUTS {
        let input = spec
            .inputs()
            .iter()
            .find(|input| input.role() == *role)
            .ok_or_else(|| SealedCapsuleError::MissingExecutionInput((*role).to_owned()))?;
        let source = runtime
            .get(*role)
            .expect("exact runtime role set checked");
        controls.push(seal_control(
            role,
            destination,
            input.digest(),
            input.size(),
            source,
        )?);
    }
    controls.sort_by(|a, b| a.role.cmp(&b.role));

    let execution_spec_digest = spec.digest(DigestAlgorithm::Sha256)?;
    let policy_digest = policy.digest(DigestAlgorithm::Sha256)?;
    if policy_inputs.qualified().policy_digest() != &policy_digest {
        return Err(SealedCapsuleError::PolicyDigestMismatch);
    }

    let control_observations = controls
        .iter()
        .map(|control| control.observation.clone())
        .collect::<Vec<_>>();
    let control_input_evidence = derive_control_input_evidence(&control_observations)?;
    let evidence_digest = derive_capsule_evidence(
        &execution_spec_digest,
        &policy_digest,
        policy_inputs.qualified().evidence_digest(),
        &control_input_evidence,
    )?;

    let qualified = QualifiedSealedM0CapsuleEvidence {
        execution_spec_digest: execution_spec_digest.clone(),
        policy_digest: policy_digest.clone(),
        policy_input_evidence: policy_inputs.qualified().evidence_digest().clone(),
        control_input_evidence,
        controls: control_observations,
        evidence_digest,
    };

    Ok(SealedM0CapsuleInputs {
        execution_spec_digest,
        policy_digest,
        policy_inputs,
        controls,
        qualified,
    })
}

/// Build bubblewrap argv from the canonical five-mount policy command and add
/// the three sealed control inputs as read-only FD mounts before environment
/// setup. The control mounts never become policy ancestors.
pub fn build_m0_capsule_bubblewrap_command(
    bubblewrap_program: impl Into<PathBuf>,
    spec: &ExecutionSpec,
    policy: &LinuxIsolationPolicyV1,
    closure: &NixClosureManifest,
    invocation: &VerifierInvocation,
    sealed: &mut SealedM0CapsuleInputs,
) -> Result<BubblewrapCommand, SealedCapsuleError> {
    let spec_digest = spec.digest(DigestAlgorithm::Sha256)?;
    let policy_digest = policy.digest(DigestAlgorithm::Sha256)?;
    if sealed.execution_spec_digest != spec_digest
        || sealed.policy_digest != policy_digest
        || sealed.qualified.execution_spec_digest() != &spec_digest
        || sealed.qualified.policy_digest() != &policy_digest
    {
        return Err(SealedCapsuleError::CapsuleSubjectMismatch);
    }

    for control in &mut sealed.controls {
        control.rewind()?;
    }

    let mut command = build_sealed_bubblewrap_command(
        bubblewrap_program,
        policy,
        closure,
        invocation,
        &mut sealed.policy_inputs,
    )?;

    let insertion = command
        .args
        .iter()
        .position(|arg| arg == "--setenv")
        .ok_or(SealedCapsuleError::BubblewrapInsertionPointMissing)?;

    let mut additions = Vec::with_capacity(sealed.controls.len() * 3);
    for control in &sealed.controls {
        additions.extend([
            "--ro-bind-fd".to_owned(),
            control.raw_fd().to_string(),
            control.destination.clone(),
        ]);
    }
    command.args.splice(insertion..insertion, additions);
    Ok(command)
}

fn require_exact_spec_surface(spec: &ExecutionSpec) -> Result<(), SealedCapsuleError> {
    let actual = spec
        .inputs()
        .iter()
        .map(|input| input.role())
        .collect::<BTreeSet<_>>();
    if actual == all_input_roles() {
        Ok(())
    } else {
        Err(SealedCapsuleError::ExecutionInputRoleSetMismatch)
    }
}

fn require_exact_policy_surface(policy: &LinuxIsolationPolicyV1) -> Result<(), SealedCapsuleError> {
    let actual = policy
        .artifact_mounts()
        .iter()
        .map(|mount| (mount.role(), mount.destination()))
        .collect::<BTreeSet<_>>();
    let expected = POLICY_INPUTS.iter().copied().collect::<BTreeSet<_>>();
    if actual != expected {
        return Err(SealedCapsuleError::PolicyArtifactSurfaceMismatch);
    }

    let forbidden = CONTROL_INPUTS
        .iter()
        .map(|(role, _)| *role)
        .collect::<BTreeSet<_>>();
    if policy
        .artifact_mounts()
        .iter()
        .any(|mount| forbidden.contains(mount.role()))
    {
        return Err(SealedCapsuleError::ControlInputInsidePolicy(
            "control-plane artifact".to_owned(),
        ));
    }
    Ok(())
}

fn all_input_roles() -> BTreeSet<&'static str> {
    POLICY_INPUTS
        .iter()
        .chain(CONTROL_INPUTS.iter())
        .map(|(role, _)| *role)
        .collect()
}

fn seal_control(
    role: &str,
    destination: &str,
    expected_digest: &Digest,
    expected_size: u64,
    source: &Path,
) -> Result<SealedControlArtifact, SealedCapsuleError> {
    // Reuse ArtifactMount validation for role/destination/digest/size syntax,
    // without making the control file part of LinuxIsolationPolicyV1.
    let _validated = ArtifactMount::new(
        role,
        destination,
        expected_digest.clone(),
        expected_size,
    )?;

    let name = CString::new(format!("mycelix-forge-control-{role}"))
        .map_err(|_| SealedCapsuleError::InvalidMemfdName)?;
    let fd = unsafe { libc::memfd_create(name.as_ptr(), libc::MFD_ALLOW_SEALING as u32) };
    if fd < 0 {
        return Err(std::io::Error::last_os_error().into());
    }
    let mut memfd = unsafe { File::from_raw_fd(fd) };
    let mut source_file = File::open(source)?;
    let (digest, size) = copy_with_digest(&mut source_file, &mut memfd, expected_digest.algorithm())?;
    if &digest != expected_digest || size != expected_size {
        return Err(SealedCapsuleError::SourceArtifactMismatch(role.to_owned()));
    }
    memfd.flush()?;

    if unsafe { libc::fcntl(memfd.as_raw_fd(), libc::F_ADD_SEALS, REQUIRED_SEALS) } != 0 {
        return Err(std::io::Error::last_os_error().into());
    }
    let seals = unsafe { libc::fcntl(memfd.as_raw_fd(), libc::F_GET_SEALS) };
    if seals < 0 {
        return Err(std::io::Error::last_os_error().into());
    }
    if seals & REQUIRED_SEALS != REQUIRED_SEALS {
        return Err(SealedCapsuleError::MissingRequiredSeals(role.to_owned()));
    }

    memfd.seek(SeekFrom::Start(0))?;
    let (sealed_digest, sealed_size) = digest_reader(&mut memfd, expected_digest.algorithm())?;
    if sealed_digest != *expected_digest || sealed_size != expected_size {
        return Err(SealedCapsuleError::SealedRereadMismatch(role.to_owned()));
    }
    memfd.seek(SeekFrom::Start(0))?;

    let observation = SealedControlObservation {
        role: role.to_owned(),
        destination: destination.to_owned(),
        digest: sealed_digest,
        size: sealed_size,
        seals,
    };
    Ok(SealedControlArtifact {
        role: role.to_owned(),
        destination: destination.to_owned(),
        file: memfd,
        observation,
    })
}

fn derive_control_input_evidence(
    observations: &[SealedControlObservation],
) -> Result<Digest, SealedCapsuleError> {
    let expected = CONTROL_INPUTS
        .iter()
        .map(|(role, destination)| (*role, *destination))
        .collect::<BTreeSet<_>>();
    let actual = observations
        .iter()
        .map(|item| (item.role(), item.destination()))
        .collect::<BTreeSet<_>>();
    if actual != expected || observations.iter().any(|item| !item.fully_sealed()) {
        return Err(SealedCapsuleError::ControlObservationMismatch);
    }

    let mut bytes = Vec::new();
    bytes.extend_from_slice(CONTROL_EVIDENCE_DOMAIN_V1);
    push_count(&mut bytes, observations.len(), "control inputs")?;
    for item in observations {
        push_string(&mut bytes, item.role(), "control role")?;
        push_string(&mut bytes, item.destination(), "control destination")?;
        push_digest(&mut bytes, item.digest())?;
        bytes.extend_from_slice(&item.size().to_be_bytes());
        bytes.extend_from_slice(&item.seals().to_be_bytes());
    }
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &bytes))
}

fn derive_capsule_evidence(
    execution_spec: &Digest,
    policy: &Digest,
    policy_inputs: &Digest,
    controls: &Digest,
) -> Result<Digest, SealedCapsuleError> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(CAPSULE_EVIDENCE_DOMAIN_V1);
    for digest in [execution_spec, policy, policy_inputs, controls] {
        push_digest(&mut bytes, digest)?;
    }
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &bytes))
}

fn copy_with_digest(
    source: &mut File,
    destination: &mut File,
    algorithm: DigestAlgorithm,
) -> Result<(Digest, u64), SealedCapsuleError> {
    let mut buffer = [0_u8; 64 * 1024];
    let mut size = 0_u64;
    match algorithm {
        DigestAlgorithm::Sha256 => {
            let mut hasher = Sha256::new();
            loop {
                let read = source.read(&mut buffer)?;
                if read == 0 {
                    break;
                }
                destination.write_all(&buffer[..read])?;
                hasher.update(&buffer[..read]);
                size = size
                    .checked_add(u64::try_from(read).map_err(|_| SealedCapsuleError::ArtifactTooLarge)?)
                    .ok_or(SealedCapsuleError::ArtifactTooLarge)?;
            }
            Ok((
                Digest::new(DigestAlgorithm::Sha256, hasher.finalize().to_vec())?,
                size,
            ))
        }
        DigestAlgorithm::Blake3_256 => {
            let mut hasher = blake3::Hasher::new();
            loop {
                let read = source.read(&mut buffer)?;
                if read == 0 {
                    break;
                }
                destination.write_all(&buffer[..read])?;
                hasher.update(&buffer[..read]);
                size = size
                    .checked_add(u64::try_from(read).map_err(|_| SealedCapsuleError::ArtifactTooLarge)?)
                    .ok_or(SealedCapsuleError::ArtifactTooLarge)?;
            }
            Ok((
                Digest::new(
                    DigestAlgorithm::Blake3_256,
                    hasher.finalize().as_bytes().to_vec(),
                )?,
                size,
            ))
        }
    }
}

fn digest_reader(
    reader: &mut File,
    algorithm: DigestAlgorithm,
) -> Result<(Digest, u64), SealedCapsuleError> {
    let mut buffer = [0_u8; 64 * 1024];
    let mut size = 0_u64;
    match algorithm {
        DigestAlgorithm::Sha256 => {
            let mut hasher = Sha256::new();
            loop {
                let read = reader.read(&mut buffer)?;
                if read == 0 {
                    break;
                }
                hasher.update(&buffer[..read]);
                size = size
                    .checked_add(u64::try_from(read).map_err(|_| SealedCapsuleError::ArtifactTooLarge)?)
                    .ok_or(SealedCapsuleError::ArtifactTooLarge)?;
            }
            Ok((
                Digest::new(DigestAlgorithm::Sha256, hasher.finalize().to_vec())?,
                size,
            ))
        }
        DigestAlgorithm::Blake3_256 => {
            let mut hasher = blake3::Hasher::new();
            loop {
                let read = reader.read(&mut buffer)?;
                if read == 0 {
                    break;
                }
                hasher.update(&buffer[..read]);
                size = size
                    .checked_add(u64::try_from(read).map_err(|_| SealedCapsuleError::ArtifactTooLarge)?)
                    .ok_or(SealedCapsuleError::ArtifactTooLarge)?;
            }
            Ok((
                Digest::new(
                    DigestAlgorithm::Blake3_256,
                    hasher.finalize().as_bytes().to_vec(),
                )?,
                size,
            ))
        }
    }
}

fn push_count(
    out: &mut Vec<u8>,
    count: usize,
    field: &'static str,
) -> Result<(), SealedCapsuleError> {
    let count =
        u16::try_from(count).map_err(|_| SealedCapsuleError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&count.to_be_bytes());
    Ok(())
}

fn push_string(
    out: &mut Vec<u8>,
    value: &str,
    field: &'static str,
) -> Result<(), SealedCapsuleError> {
    let len =
        u16::try_from(value.len()).map_err(|_| SealedCapsuleError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), SealedCapsuleError> {
    push_string(out, digest.algorithm().id(), "digest algorithm")?;
    let len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| SealedCapsuleError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

#[derive(Debug, Error)]
pub enum SealedCapsuleError {
    #[error(transparent)]
    Core(#[from] ForgeCoreError),
    #[error(transparent)]
    Execution(#[from] ExecutionContractError),
    #[error(transparent)]
    Isolation(#[from] IsolationPolicyError),
    #[error(transparent)]
    Sealed(#[from] SealedInputError),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error("duplicate runtime artifact role")]
    DuplicateRuntimeRole,
    #[error("runtime artifact roles differ from exact M0 ExecutionSpec inputs")]
    RuntimeRoleSetMismatch,
    #[error("ExecutionSpec input roles differ from exact M0 eight-input surface")]
    ExecutionInputRoleSetMismatch,
    #[error("Linux isolation policy artifact surface is not the exact acyclic five-input set")]
    PolicyArtifactSurfaceMismatch,
    #[error("control-plane input unexpectedly appears inside Linux isolation policy: {0}")]
    ControlInputInsidePolicy(String),
    #[error("missing required execution input: {0}")]
    MissingExecutionInput(String),
    #[error("source bytes differ from expected control artifact: {0}")]
    SourceArtifactMismatch(String),
    #[error("sealed control artifact lacks required kernel seals: {0}")]
    MissingRequiredSeals(String),
    #[error("sealed control artifact re-read differs from expected bytes: {0}")]
    SealedRereadMismatch(String),
    #[error("invalid memfd name")]
    InvalidMemfdName,
    #[error("artifact byte length overflow")]
    ArtifactTooLarge,
    #[error("control observation set or sealing state mismatch")]
    ControlObservationMismatch,
    #[error("sealed capsule subject differs from supplied ExecutionSpec/policy")]
    CapsuleSubjectMismatch,
    #[error("canonical bubblewrap command has no environment insertion point")]
    BubblewrapInsertionPointMissing,
    #[error("Linux policy digest differs from sealed policy-input evidence")]
    PolicyDigestMismatch,
    #[error("canonical field is too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn policy_and_control_surfaces_are_disjoint_and_total() {
        let policy = POLICY_INPUTS
            .iter()
            .map(|(role, _)| *role)
            .collect::<BTreeSet<_>>();
        let control = CONTROL_INPUTS
            .iter()
            .map(|(role, _)| *role)
            .collect::<BTreeSet<_>>();
        assert!(policy.is_disjoint(&control));
        assert_eq!(policy.len(), 5);
        assert_eq!(control.len(), 3);
        assert_eq!(all_input_roles().len(), 8);
    }

    #[test]
    fn policy_surface_excludes_all_recursive_control_files() {
        let policy = POLICY_INPUTS
            .iter()
            .map(|(role, _)| *role)
            .collect::<BTreeSet<_>>();
        for forbidden in [
            "guest-verification-plan",
            "guest-tool-map",
            "linux-isolation-policy",
        ] {
            assert!(!policy.contains(forbidden));
        }
    }
}
