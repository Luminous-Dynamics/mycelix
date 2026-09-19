// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-004D4G: receipt-bound verification and launch for materialized M0 capsules.
//!
//! D4F publishes a complete filesystem tree plus a receipt. D4G independently
//! reopens that tree after process restart, binds it to externally expected D4E
//! subjects, rejects tree/path/fingerprint drift, validates the D4B launch
//! manifest, and only then executes the host. This is operational provenance,
//! not repository `OfflineEvidence` authority.

use mycelix_forge_core::{Digest, DigestAlgorithm, ForgeCoreError};
use mycelix_forge_execution::{ExecutionContractError, ExecutionSpec};
use mycelix_forge_guest_plan::GuestVerificationPlanV1;
use mycelix_forge_hermetic_host_v2::{
    execute_from_launch_manifest_v2, HermeticHostV2Error, HostLaunchManifestV2,
    QualifiedHermeticHostRunV2,
};
use mycelix_forge_hermetic_run_evidence::{HermeticRunError, HermeticRunPlan};
use mycelix_forge_linux_isolation::{IsolationPolicyError, LinuxIsolationPolicyV1};
use mycelix_forge_m0_capsule_builder::BuiltM0Capsule;
use mycelix_forge_m0_capsule_materializer::{
    CapsuleMaterializationReceiptV1, MaterializationError, MaterializedFileV1,
    HOST_LAUNCH_MANIFEST_FILENAME, MATERIALIZATION_RECEIPT_FILENAME,
};
use mycelix_forge_sealed_capsule_inputs::{CONTROL_INPUTS, POLICY_INPUTS};
use sha2::{Digest as ShaDigest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::{self, OpenOptions},
    io::Read,
    os::unix::fs::{OpenOptionsExt, PermissionsExt},
    path::{Path, PathBuf},
};
use thiserror::Error;

const VERIFIED_MATERIALIZATION_DOMAIN_V1: &[u8] =
    b"mycelix-forge/verified-materialized-m0-capsule/v1\0";
const RECEIPT_BOUND_HOST_RUN_DOMAIN_V1: &[u8] =
    b"mycelix-forge/receipt-bound-host-run/v1\0";
const FILE_MODE: u32 = 0o400;
const DIRECTORY_MODE: u32 = 0o700;
const MAX_CONTROL_BYTES: usize = 16 * 1024 * 1024;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MaterializedCapsuleExpectation {
    construction_commitment: Digest,
    execution_subject: Digest,
    execution_spec: Digest,
    run_plan: Digest,
}

impl MaterializedCapsuleExpectation {
    pub fn new(
        construction_commitment: Digest,
        execution_subject: Digest,
        execution_spec: Digest,
        run_plan: Digest,
    ) -> Self {
        Self {
            construction_commitment,
            execution_subject,
            execution_spec,
            run_plan,
        }
    }

    pub fn from_capsule(capsule: &BuiltM0Capsule) -> Result<Self, ReceiptBoundLaunchError> {
        Ok(Self::new(
            capsule.construction_commitment().clone(),
            capsule.execution_subject().clone(),
            capsule.execution_spec().digest(DigestAlgorithm::Sha256)?,
            capsule.run_plan().digest(DigestAlgorithm::Sha256)?,
        ))
    }

    pub fn construction_commitment(&self) -> &Digest {
        &self.construction_commitment
    }
    pub fn execution_subject(&self) -> &Digest {
        &self.execution_subject
    }
    pub fn execution_spec(&self) -> &Digest {
        &self.execution_spec
    }
    pub fn run_plan(&self) -> &Digest {
        &self.run_plan
    }
}

pub struct QualifiedMaterializedM0Capsule {
    root: PathBuf,
    launch_manifest: PathBuf,
    receipt: CapsuleMaterializationReceiptV1,
    receipt_digest: Digest,
    receipt_file_digest: Digest,
    receipt_file_size: u64,
    launch_manifest_digest: Digest,
    evidence_digest: Digest,
}

impl QualifiedMaterializedM0Capsule {
    pub fn root(&self) -> &Path {
        &self.root
    }
    pub fn launch_manifest(&self) -> &Path {
        &self.launch_manifest
    }
    pub fn receipt(&self) -> &CapsuleMaterializationReceiptV1 {
        &self.receipt
    }
    pub fn receipt_digest(&self) -> &Digest {
        &self.receipt_digest
    }
    pub fn receipt_file_digest(&self) -> &Digest {
        &self.receipt_file_digest
    }
    pub const fn receipt_file_size(&self) -> u64 {
        self.receipt_file_size
    }
    pub fn launch_manifest_digest(&self) -> &Digest {
        &self.launch_manifest_digest
    }
    pub fn evidence_digest(&self) -> &Digest {
        &self.evidence_digest
    }
}

pub struct ReceiptBoundHostRun {
    materialized: QualifiedMaterializedM0Capsule,
    host: QualifiedHermeticHostRunV2,
    evidence_digest: Digest,
}

impl ReceiptBoundHostRun {
    pub fn materialized(&self) -> &QualifiedMaterializedM0Capsule {
        &self.materialized
    }
    pub fn host(&self) -> &QualifiedHermeticHostRunV2 {
        &self.host
    }
    pub fn evidence_digest(&self) -> &Digest {
        &self.evidence_digest
    }
}

pub fn verify_materialized_m0_capsule(
    root: &Path,
    expected: &MaterializedCapsuleExpectation,
) -> Result<QualifiedMaterializedM0Capsule, ReceiptBoundLaunchError> {
    let root = normalize_existing_root(root)?;
    require_directory(&root, DIRECTORY_MODE)?;

    let receipt_path = root.join(MATERIALIZATION_RECEIPT_FILENAME);
    let receipt_bytes = read_limited_nofollow(&receipt_path, FILE_MODE)?;
    let receipt_file_size = u64::try_from(receipt_bytes.len())
        .map_err(|_| ReceiptBoundLaunchError::FileTooLarge)?;
    let receipt_file_digest = Digest::of_bytes(DigestAlgorithm::Sha256, &receipt_bytes);
    let receipt: CapsuleMaterializationReceiptV1 = serde_json::from_slice(&receipt_bytes)?;
    receipt.validate()?;
    require_expected_receipt_subjects(&receipt, expected)?;
    let receipt_digest = receipt.digest(DigestAlgorithm::Sha256)?;

    let expected_tree = expected_tree_layout(&receipt)?;
    verify_exact_tree(&root, &expected_tree)?;
    for file in receipt.files() {
        require_fingerprint(
            &root.join(file.relative_path()),
            file.digest(),
            file.size(),
            file.mode(),
        )?;
    }

    let launch_entry = receipt_file(&receipt, "control:host-launch-manifest-v2")?;
    if launch_entry.relative_path() != HOST_LAUNCH_MANIFEST_FILENAME {
        return Err(ReceiptBoundLaunchError::LaunchManifestPathMismatch);
    }
    let launch_path = root.join(launch_entry.relative_path());
    let launch_bytes = read_limited_nofollow(&launch_path, launch_entry.mode())?;
    let launch_manifest_digest = Digest::of_bytes(launch_entry.digest().algorithm(), &launch_bytes);
    let launch_size = u64::try_from(launch_bytes.len())
        .map_err(|_| ReceiptBoundLaunchError::FileTooLarge)?;
    if &launch_manifest_digest != launch_entry.digest() || launch_size != launch_entry.size() {
        return Err(ReceiptBoundLaunchError::LaunchManifestFingerprintMismatch);
    }
    let launch: HostLaunchManifestV2 = serde_json::from_slice(&launch_bytes)?;
    validate_launch_manifest(&root, &receipt, &launch)?;
    validate_control_subjects(&root, &receipt, expected)?;

    // Detect a non-privileged concurrent add/remove during receipt/control parsing.
    verify_exact_tree(&root, &expected_tree)?;

    let evidence_digest = derive_verified_materialization_evidence(
        expected,
        &receipt_digest,
        &receipt_file_digest,
        receipt_file_size,
        &launch_manifest_digest,
    )?;

    Ok(QualifiedMaterializedM0Capsule {
        root,
        launch_manifest: launch_path,
        receipt,
        receipt_digest,
        receipt_file_digest,
        receipt_file_size,
        launch_manifest_digest,
        evidence_digest,
    })
}

pub fn execute_receipt_bound_m0_capsule(
    root: &Path,
    expected: &MaterializedCapsuleExpectation,
) -> Result<ReceiptBoundHostRun, ReceiptBoundLaunchError> {
    let materialized = verify_materialized_m0_capsule(root, expected)?;
    let host = execute_from_launch_manifest_v2(materialized.launch_manifest())?;
    if host.execution_spec_digest() != expected.execution_spec()
        || host.run_plan_digest() != expected.run_plan()
    {
        return Err(ReceiptBoundLaunchError::HostSubjectMismatch);
    }
    let evidence_digest =
        derive_receipt_bound_host_run(materialized.evidence_digest(), host.evidence_digest())?;
    Ok(ReceiptBoundHostRun {
        materialized,
        host,
        evidence_digest,
    })
}

fn require_expected_receipt_subjects(
    receipt: &CapsuleMaterializationReceiptV1,
    expected: &MaterializedCapsuleExpectation,
) -> Result<(), ReceiptBoundLaunchError> {
    if receipt.construction_commitment() != expected.construction_commitment()
        || receipt.execution_subject() != expected.execution_subject()
        || receipt.execution_spec() != expected.execution_spec()
        || receipt.run_plan() != expected.run_plan()
    {
        return Err(ReceiptBoundLaunchError::ReceiptSubjectMismatch);
    }
    if !receipt.launch_manifest_absolute_paths() {
        return Err(ReceiptBoundLaunchError::LaunchManifestPathModeMismatch);
    }
    Ok(())
}

fn validate_control_subjects(
    root: &Path,
    receipt: &CapsuleMaterializationReceiptV1,
    expected: &MaterializedCapsuleExpectation,
) -> Result<(), ReceiptBoundLaunchError> {
    let spec_entry = receipt_file(receipt, "control:execution-spec")?;
    let spec: ExecutionSpec = serde_json::from_slice(&read_limited_nofollow(
        &root.join(spec_entry.relative_path()),
        spec_entry.mode(),
    )?)?;
    let spec_digest = spec.digest(expected.execution_spec().algorithm())?;
    if &spec_digest != expected.execution_spec()
        || spec.subject() != expected.execution_subject()
        || &spec_digest != receipt.execution_spec()
    {
        return Err(ReceiptBoundLaunchError::ExecutionSpecSubjectMismatch);
    }

    let run_entry = receipt_file(receipt, "control:hermetic-run-plan")?;
    let run_plan: HermeticRunPlan = serde_json::from_slice(&read_limited_nofollow(
        &root.join(run_entry.relative_path()),
        run_entry.mode(),
    )?)?;
    let run_digest = run_plan.digest(expected.run_plan().algorithm())?;
    if &run_digest != expected.run_plan()
        || &run_digest != receipt.run_plan()
        || run_plan.execution_spec() != expected.execution_spec()
    {
        return Err(ReceiptBoundLaunchError::RunPlanSubjectMismatch);
    }

    let policy_entry = receipt_file(receipt, "input:linux-isolation-policy")?;
    let policy: LinuxIsolationPolicyV1 = serde_json::from_slice(&read_limited_nofollow(
        &root.join(policy_entry.relative_path()),
        policy_entry.mode(),
    )?)?;
    let policy_digest = policy.digest(run_plan.isolation_policy().algorithm())?;
    if &policy_digest != run_plan.isolation_policy() {
        return Err(ReceiptBoundLaunchError::IsolationPolicySubjectMismatch);
    }

    let plan_entry = receipt_file(receipt, "input:guest-verification-plan")?;
    let guest_plan: GuestVerificationPlanV1 = serde_json::from_slice(&read_limited_nofollow(
        &root.join(plan_entry.relative_path()),
        plan_entry.mode(),
    )?)?;
    if guest_plan.execution_subject() != expected.execution_subject()
        || guest_plan.git_object_validation_policy() != run_plan.git_object_validation_policy()
        || guest_plan.run_challenge() != run_plan.run_challenge()
    {
        return Err(ReceiptBoundLaunchError::GuestPlanSubjectMismatch);
    }
    Ok(())
}

fn validate_launch_manifest(
    root: &Path,
    receipt: &CapsuleMaterializationReceiptV1,
    launch: &HostLaunchManifestV2,
) -> Result<(), ReceiptBoundLaunchError> {
    for (actual, role) in [
        (&launch.execution_spec, "control:execution-spec"),
        (&launch.run_plan, "control:hermetic-run-plan"),
        (&launch.guest_plan, "input:guest-verification-plan"),
        (&launch.guest_tool_map, "input:guest-tool-map"),
        (&launch.isolation_policy, "input:linux-isolation-policy"),
        (&launch.nix_closure, "input:nix-closure-manifest"),
        (&launch.sandbox_invocation, "input:sandbox-invocation"),
    ] {
        let entry = receipt_file(receipt, role)?;
        if actual.as_path() != root.join(entry.relative_path()) {
            return Err(ReceiptBoundLaunchError::LaunchPathMismatch(role.to_owned()));
        }
    }
    validate_bubblewrap_path(&launch.bubblewrap_program)?;

    let expected_roles = POLICY_INPUTS
        .iter()
        .chain(CONTROL_INPUTS.iter())
        .map(|(role, _)| *role)
        .collect::<BTreeSet<_>>();
    if launch.runtime_artifacts.len() != expected_roles.len() {
        return Err(ReceiptBoundLaunchError::RuntimeArtifactSetMismatch);
    }
    let mut observed = BTreeMap::new();
    for artifact in &launch.runtime_artifacts {
        if observed
            .insert(artifact.role.as_str(), artifact.source.as_path())
            .is_some()
        {
            return Err(ReceiptBoundLaunchError::RuntimeArtifactSetMismatch);
        }
    }
    if observed.keys().copied().collect::<BTreeSet<_>>() != expected_roles {
        return Err(ReceiptBoundLaunchError::RuntimeArtifactSetMismatch);
    }
    for role in expected_roles {
        let entry = receipt_file(receipt, &format!("input:{role}"))?;
        if observed.get(role).copied() != Some(root.join(entry.relative_path()).as_path()) {
            return Err(ReceiptBoundLaunchError::RuntimeArtifactPathMismatch(
                role.to_owned(),
            ));
        }
    }
    Ok(())
}

fn validate_bubblewrap_path(path: &Path) -> Result<(), ReceiptBoundLaunchError> {
    let text = path.to_str().ok_or(ReceiptBoundLaunchError::NonUtf8Path)?;
    if !path.is_absolute()
        || !text.starts_with("/nix/store/")
        || !text.ends_with("/bin/bwrap")
        || text.contains("//")
        || text.contains("/../")
        || text.contains("/./")
    {
        return Err(ReceiptBoundLaunchError::InvalidBubblewrapProgram(
            path.to_path_buf(),
        ));
    }
    Ok(())
}

fn receipt_file<'a>(
    receipt: &'a CapsuleMaterializationReceiptV1,
    role: &str,
) -> Result<&'a MaterializedFileV1, ReceiptBoundLaunchError> {
    receipt
        .files()
        .iter()
        .find(|file| file.role() == role)
        .ok_or_else(|| ReceiptBoundLaunchError::MissingReceiptRole(role.to_owned()))
}

struct ExpectedTree {
    files: BTreeSet<PathBuf>,
    directories: BTreeSet<PathBuf>,
}

fn expected_tree_layout(
    receipt: &CapsuleMaterializationReceiptV1,
) -> Result<ExpectedTree, ReceiptBoundLaunchError> {
    let mut files = BTreeSet::new();
    let mut directories = BTreeSet::new();
    for file in receipt.files() {
        let relative = PathBuf::from(file.relative_path());
        validate_safe_relative(&relative)?;
        if !files.insert(relative.clone()) {
            return Err(ReceiptBoundLaunchError::TreeLayoutMismatch);
        }
        let mut parent = relative.parent();
        while let Some(value) = parent {
            if value.as_os_str().is_empty() {
                break;
            }
            directories.insert(value.to_path_buf());
            parent = value.parent();
        }
    }
    files.insert(PathBuf::from(MATERIALIZATION_RECEIPT_FILENAME));
    Ok(ExpectedTree { files, directories })
}

fn verify_exact_tree(root: &Path, expected: &ExpectedTree) -> Result<(), ReceiptBoundLaunchError> {
    let mut files = BTreeSet::new();
    let mut directories = BTreeSet::new();
    walk_tree(root, Path::new(""), &mut files, &mut directories)?;
    if files != expected.files || directories != expected.directories {
        return Err(ReceiptBoundLaunchError::TreeLayoutMismatch);
    }
    for directory in &directories {
        require_directory(&root.join(directory), DIRECTORY_MODE)?;
    }
    Ok(())
}

fn walk_tree(
    root: &Path,
    relative: &Path,
    files: &mut BTreeSet<PathBuf>,
    directories: &mut BTreeSet<PathBuf>,
) -> Result<(), ReceiptBoundLaunchError> {
    let current = if relative.as_os_str().is_empty() {
        root.to_path_buf()
    } else {
        root.join(relative)
    };
    for entry in fs::read_dir(&current)? {
        let entry = entry?;
        let child = relative.join(entry.file_name());
        let metadata = fs::symlink_metadata(entry.path())?;
        if metadata.file_type().is_symlink() {
            return Err(ReceiptBoundLaunchError::SymlinkInCapsule(child));
        }
        if metadata.is_dir() {
            directories.insert(child.clone());
            walk_tree(root, &child, files, directories)?;
        } else if metadata.is_file() {
            files.insert(child);
        } else {
            return Err(ReceiptBoundLaunchError::UnsupportedTreeEntry(child));
        }
    }
    Ok(())
}

fn normalize_existing_root(root: &Path) -> Result<PathBuf, ReceiptBoundLaunchError> {
    let absolute = if root.is_absolute() {
        root.to_path_buf()
    } else {
        std::env::current_dir()?.join(root)
    };
    let name = absolute
        .file_name()
        .ok_or(ReceiptBoundLaunchError::InvalidRoot)?;
    let parent = fs::canonicalize(
        absolute
            .parent()
            .ok_or(ReceiptBoundLaunchError::InvalidRoot)?,
    )?;
    let normalized = parent.join(name);
    let metadata = fs::symlink_metadata(&normalized)?;
    if metadata.file_type().is_symlink() || !metadata.is_dir() {
        return Err(ReceiptBoundLaunchError::InvalidRoot);
    }
    Ok(normalized)
}

fn require_directory(path: &Path, expected_mode: u32) -> Result<(), ReceiptBoundLaunchError> {
    let metadata = fs::symlink_metadata(path)?;
    if metadata.file_type().is_symlink() || !metadata.is_dir() {
        return Err(ReceiptBoundLaunchError::InvalidDirectory(path.to_path_buf()));
    }
    let actual = metadata.permissions().mode() & 0o777;
    if actual != expected_mode {
        return Err(ReceiptBoundLaunchError::DirectoryModeMismatch {
            path: path.to_path_buf(),
            expected: expected_mode,
            actual,
        });
    }
    Ok(())
}

fn require_fingerprint(
    path: &Path,
    expected_digest: &Digest,
    expected_size: u64,
    expected_mode: u32,
) -> Result<(), ReceiptBoundLaunchError> {
    let (digest, size) = hash_nofollow(path, expected_digest.algorithm(), expected_mode)?;
    if &digest != expected_digest || size != expected_size {
        return Err(ReceiptBoundLaunchError::FileFingerprintMismatch(
            path.to_path_buf(),
        ));
    }
    Ok(())
}

fn hash_nofollow(
    path: &Path,
    algorithm: DigestAlgorithm,
    expected_mode: u32,
) -> Result<(Digest, u64), ReceiptBoundLaunchError> {
    let mut file = open_nofollow_regular(path, expected_mode)?;
    let mut buffer = [0_u8; 64 * 1024];
    let mut size = 0_u64;
    match algorithm {
        DigestAlgorithm::Sha256 => {
            let mut hasher = Sha256::new();
            loop {
                let read = file.read(&mut buffer)?;
                if read == 0 {
                    break;
                }
                hasher.update(&buffer[..read]);
                size = size
                    .checked_add(read as u64)
                    .ok_or(ReceiptBoundLaunchError::FileTooLarge)?;
            }
            Ok((
                Digest::new(DigestAlgorithm::Sha256, hasher.finalize().to_vec())?,
                size,
            ))
        }
        DigestAlgorithm::Blake3_256 => {
            let mut hasher = blake3::Hasher::new();
            loop {
                let read = file.read(&mut buffer)?;
                if read == 0 {
                    break;
                }
                hasher.update(&buffer[..read]);
                size = size
                    .checked_add(read as u64)
                    .ok_or(ReceiptBoundLaunchError::FileTooLarge)?;
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

fn read_limited_nofollow(
    path: &Path,
    expected_mode: u32,
) -> Result<Vec<u8>, ReceiptBoundLaunchError> {
    let mut file = open_nofollow_regular(path, expected_mode)?;
    let limit = u64::try_from(MAX_CONTROL_BYTES)
        .map_err(|_| ReceiptBoundLaunchError::FileTooLarge)?
        .saturating_add(1);
    let mut bytes = Vec::new();
    file.by_ref().take(limit).read_to_end(&mut bytes)?;
    if bytes.len() > MAX_CONTROL_BYTES {
        return Err(ReceiptBoundLaunchError::ControlFileTooLarge(
            path.to_path_buf(),
        ));
    }
    Ok(bytes)
}

fn open_nofollow_regular(
    path: &Path,
    expected_mode: u32,
) -> Result<std::fs::File, ReceiptBoundLaunchError> {
    let metadata = fs::symlink_metadata(path)?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(ReceiptBoundLaunchError::InvalidFile(path.to_path_buf()));
    }
    let actual = metadata.permissions().mode() & 0o777;
    if actual != expected_mode {
        return Err(ReceiptBoundLaunchError::FileModeMismatch {
            path: path.to_path_buf(),
            expected: expected_mode,
            actual,
        });
    }
    Ok(OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
        .open(path)?)
}

fn validate_safe_relative(path: &Path) -> Result<(), ReceiptBoundLaunchError> {
    if path.is_absolute() || path.as_os_str().is_empty() {
        return Err(ReceiptBoundLaunchError::UnsafeRelativePath(
            path.to_path_buf(),
        ));
    }
    let text = path.to_str().ok_or(ReceiptBoundLaunchError::NonUtf8Path)?;
    if text.starts_with('/')
        || text.ends_with('/')
        || text
            .split('/')
            .any(|segment| segment.is_empty() || segment == "." || segment == "..")
    {
        return Err(ReceiptBoundLaunchError::UnsafeRelativePath(
            path.to_path_buf(),
        ));
    }
    Ok(())
}

fn derive_verified_materialization_evidence(
    expected: &MaterializedCapsuleExpectation,
    receipt_digest: &Digest,
    receipt_file_digest: &Digest,
    receipt_file_size: u64,
    launch_manifest_digest: &Digest,
) -> Result<Digest, ReceiptBoundLaunchError> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(VERIFIED_MATERIALIZATION_DOMAIN_V1);
    for digest in [
        expected.construction_commitment(),
        expected.execution_subject(),
        expected.execution_spec(),
        expected.run_plan(),
        receipt_digest,
        receipt_file_digest,
        launch_manifest_digest,
    ] {
        push_digest(&mut bytes, digest)?;
    }
    bytes.extend_from_slice(&receipt_file_size.to_be_bytes());
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &bytes))
}

fn derive_receipt_bound_host_run(
    materialization: &Digest,
    host: &Digest,
) -> Result<Digest, ReceiptBoundLaunchError> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(RECEIPT_BOUND_HOST_RUN_DOMAIN_V1);
    push_digest(&mut bytes, materialization)?;
    push_digest(&mut bytes, host)?;
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &bytes))
}

fn push_string(out: &mut Vec<u8>, value: &str) -> Result<(), ReceiptBoundLaunchError> {
    let len = u16::try_from(value.len())
        .map_err(|_| ReceiptBoundLaunchError::CanonicalLengthOverflow)?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), ReceiptBoundLaunchError> {
    push_string(out, digest.algorithm().id())?;
    let len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| ReceiptBoundLaunchError::CanonicalLengthOverflow)?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

#[derive(Debug, Error)]
pub enum ReceiptBoundLaunchError {
    #[error(transparent)] Core(#[from] ForgeCoreError),
    #[error(transparent)] Execution(#[from] ExecutionContractError),
    #[error(transparent)] HermeticRun(#[from] HermeticRunError),
    #[error(transparent)] Isolation(#[from] IsolationPolicyError),
    #[error(transparent)] Materialization(#[from] MaterializationError),
    #[error(transparent)] Host(#[from] HermeticHostV2Error),
    #[error(transparent)] Json(#[from] serde_json::Error),
    #[error(transparent)] Io(#[from] std::io::Error),
    #[error("invalid materialized capsule root")] InvalidRoot,
    #[error("invalid capsule directory: {0:?}")] InvalidDirectory(PathBuf),
    #[error("invalid capsule file: {0:?}")] InvalidFile(PathBuf),
    #[error("capsule directory mode mismatch for {path:?}: expected {expected:o}, got {actual:o}")]
    DirectoryModeMismatch { path: PathBuf, expected: u32, actual: u32 },
    #[error("capsule file mode mismatch for {path:?}: expected {expected:o}, got {actual:o}")]
    FileModeMismatch { path: PathBuf, expected: u32, actual: u32 },
    #[error("materialization receipt names different expected subjects")] ReceiptSubjectMismatch,
    #[error("materialization receipt launch-path mode differs")] LaunchManifestPathModeMismatch,
    #[error("launch manifest receipt path differs")] LaunchManifestPathMismatch,
    #[error("launch manifest file fingerprint differs")] LaunchManifestFingerprintMismatch,
    #[error("execution-spec subject differs from expected materialization")]
    ExecutionSpecSubjectMismatch,
    #[error("run-plan subject differs from expected materialization")] RunPlanSubjectMismatch,
    #[error("isolation-policy subject differs from run plan")] IsolationPolicySubjectMismatch,
    #[error("guest plan differs from expected execution/run-plan subjects")]
    GuestPlanSubjectMismatch,
    #[error("launch manifest path differs for receipt role: {0}")] LaunchPathMismatch(String),
    #[error("invalid bubblewrap program path: {0:?}")] InvalidBubblewrapProgram(PathBuf),
    #[error("launch runtime-artifact set differs from exact D4A 5+3 surface")]
    RuntimeArtifactSetMismatch,
    #[error("launch runtime-artifact path differs for role: {0}")]
    RuntimeArtifactPathMismatch(String),
    #[error("missing materialization receipt role: {0}")] MissingReceiptRole(String),
    #[error("materialized tree contains a symlink: {0:?}")] SymlinkInCapsule(PathBuf),
    #[error("materialized tree contains an unsupported entry: {0:?}")]
    UnsupportedTreeEntry(PathBuf),
    #[error("materialized filesystem tree differs from exact receipt layout")]
    TreeLayoutMismatch,
    #[error("unsafe relative path in materialization receipt: {0:?}")]
    UnsafeRelativePath(PathBuf),
    #[error("materialized file fingerprint differs: {0:?}")] FileFingerprintMismatch(PathBuf),
    #[error("receipt-bound D4B host run names different spec/run-plan subjects")]
    HostSubjectMismatch,
    #[error("path is not UTF-8")] NonUtf8Path,
    #[error("file size overflow")] FileTooLarge,
    #[error("bounded control file exceeds verifier limit: {0:?}")] ControlFileTooLarge(PathBuf),
    #[error("canonical receipt-bound evidence field length overflow")]
    CanonicalLengthOverflow,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    #[test]
    fn expectation_preserves_external_subjects() {
        let expected = MaterializedCapsuleExpectation::new(digest(1), digest(2), digest(3), digest(4));
        assert_eq!(expected.construction_commitment(), &digest(1));
        assert_eq!(expected.execution_subject(), &digest(2));
        assert_eq!(expected.execution_spec(), &digest(3));
        assert_eq!(expected.run_plan(), &digest(4));
    }

    #[test]
    fn d4a_runtime_role_surface_is_exactly_eight() {
        let roles = POLICY_INPUTS
            .iter()
            .chain(CONTROL_INPUTS.iter())
            .map(|(role, _)| *role)
            .collect::<BTreeSet<_>>();
        assert_eq!(roles.len(), 8);
    }

    #[test]
    fn noncanonical_relative_paths_are_rejected() {
        assert!(validate_safe_relative(Path::new("inputs/file.json")).is_ok());
        assert!(validate_safe_relative(Path::new("../escape")).is_err());
        assert!(validate_safe_relative(Path::new("/absolute")).is_err());
        assert!(validate_safe_relative(Path::new("inputs/./file")).is_err());
        assert!(validate_safe_relative(Path::new("inputs//file")).is_err());
    }

    #[test]
    fn combined_run_evidence_binds_both_layers() {
        let first = derive_receipt_bound_host_run(&digest(0x11), &digest(0x22)).unwrap();
        let changed_materialization =
            derive_receipt_bound_host_run(&digest(0x12), &digest(0x22)).unwrap();
        let changed_host = derive_receipt_bound_host_run(&digest(0x11), &digest(0x23)).unwrap();
        assert_ne!(first, changed_materialization);
        assert_ne!(first, changed_host);
    }
}
