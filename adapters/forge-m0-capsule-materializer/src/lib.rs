// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-004D4F: atomic filesystem materialization for one constructed M0 capsule.
//!
//! D4E proves finite deterministic construction. This crate closes the next
//! operational gap: publish those exact bytes as one complete filesystem tree
//! without partial visibility or silent overwrite. Materialization is not an
//! authority theorem and never substitutes for D4A/D4B sealing and runtime
//! requalification.

use mycelix_forge_core::{Digest, DigestAlgorithm, ForgeCoreError};
use mycelix_forge_execution::ExecutionContractError;
use mycelix_forge_hermetic_run_evidence::HermeticRunError;
use mycelix_forge_m0_capsule_builder::{BuiltM0Capsule, GeneratedCapsuleArtifact};
use mycelix_forge_sealed_capsule_inputs::{CONTROL_INPUTS, POLICY_INPUTS};
use serde::{Deserialize, Serialize};
use sha2::{Digest as ShaDigest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    ffi::CString,
    fs::{self, File, OpenOptions},
    io::{Read, Write},
    os::unix::{
        ffi::OsStrExt,
        fs::{OpenOptionsExt, PermissionsExt},
    },
    path::{Component, Path, PathBuf},
    sync::atomic::{AtomicU64, Ordering},
};
use thiserror::Error;

const RECEIPT_DOMAIN_V1: &[u8] = b"mycelix-forge/m0-capsule-materialization/v1\0";
pub const MATERIALIZATION_SCHEMA_VERSION: u16 = 1;
pub const HOST_LAUNCH_MANIFEST_FILENAME: &str = "host-launch-manifest-v2.json";
pub const MATERIALIZATION_RECEIPT_FILENAME: &str = "materialization-receipt-v1.json";
const EXECUTION_SPEC_PATH: &str = "control/execution-spec.json";
const RUN_PLAN_PATH: &str = "control/hermetic-run-plan.json";
const FILE_MODE: u32 = 0o400;
const DIRECTORY_MODE: u32 = 0o700;
const MAX_TEXT: usize = u16::MAX as usize;
const MAX_STAGE_ATTEMPTS: usize = 32;
static STAGING_COUNTER: AtomicU64 = AtomicU64::new(1);

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MaterializedFileV1 {
    role: String,
    relative_path: String,
    digest: Digest,
    size: u64,
    mode: u32,
}

impl MaterializedFileV1 {
    pub fn role(&self) -> &str {
        &self.role
    }

    pub fn relative_path(&self) -> &str {
        &self.relative_path
    }

    pub fn digest(&self) -> &Digest {
        &self.digest
    }

    pub const fn size(&self) -> u64 {
        self.size
    }

    pub const fn mode(&self) -> u32 {
        self.mode
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CapsuleMaterializationReceiptV1 {
    schema_version: u16,
    construction_commitment: Digest,
    execution_subject: Digest,
    execution_spec: Digest,
    run_plan: Digest,
    launch_manifest_absolute_paths: bool,
    files: Vec<MaterializedFileV1>,
}

impl CapsuleMaterializationReceiptV1 {
    fn new(
        construction_commitment: Digest,
        execution_subject: Digest,
        execution_spec: Digest,
        run_plan: Digest,
        mut files: Vec<MaterializedFileV1>,
    ) -> Result<Self, MaterializationError> {
        files.sort_by(|a, b| a.role.cmp(&b.role).then(a.relative_path.cmp(&b.relative_path)));
        let receipt = Self {
            schema_version: MATERIALIZATION_SCHEMA_VERSION,
            construction_commitment,
            execution_subject,
            execution_spec,
            run_plan,
            launch_manifest_absolute_paths: true,
            files,
        };
        receipt.validate()?;
        Ok(receipt)
    }

    pub const fn schema_version(&self) -> u16 {
        self.schema_version
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

    pub const fn launch_manifest_absolute_paths(&self) -> bool {
        self.launch_manifest_absolute_paths
    }

    pub fn files(&self) -> &[MaterializedFileV1] {
        &self.files
    }

    pub fn validate(&self) -> Result<(), MaterializationError> {
        if self.schema_version != MATERIALIZATION_SCHEMA_VERSION {
            return Err(MaterializationError::UnsupportedReceiptVersion(
                self.schema_version,
            ));
        }
        if !self.launch_manifest_absolute_paths {
            return Err(MaterializationError::UnexpectedLaunchPathMode);
        }
        let expected = expected_receipt_paths()?;
        if self.files.len() != expected.len() {
            return Err(MaterializationError::ReceiptFileSetMismatch);
        }
        let mut roles = BTreeSet::new();
        let mut paths = BTreeSet::new();
        for file in &self.files {
            validate_text("receipt file role", &file.role)?;
            validate_safe_relative(&file.relative_path)?;
            if file.size == 0 {
                return Err(MaterializationError::EmptyMaterializedFile(
                    file.role.clone(),
                ));
            }
            if file.mode != FILE_MODE {
                return Err(MaterializationError::UnexpectedReceiptFileMode {
                    role: file.role.clone(),
                    mode: file.mode,
                });
            }
            if !roles.insert(file.role.clone()) || !paths.insert(file.relative_path.clone()) {
                return Err(MaterializationError::DuplicateReceiptEntry);
            }
            let expected_path = expected
                .get(file.role())
                .ok_or(MaterializationError::ReceiptFileSetMismatch)?;
            if expected_path != file.relative_path() {
                return Err(MaterializationError::ReceiptPathMismatch(
                    file.role.clone(),
                ));
            }
        }
        if roles != expected.keys().cloned().collect::<BTreeSet<_>>() {
            return Err(MaterializationError::ReceiptFileSetMismatch);
        }
        Ok(())
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, MaterializationError> {
        self.validate()?;
        let mut out = Vec::new();
        out.extend_from_slice(RECEIPT_DOMAIN_V1);
        out.extend_from_slice(&self.schema_version.to_be_bytes());
        push_digest(&mut out, &self.construction_commitment)?;
        push_digest(&mut out, &self.execution_subject)?;
        push_digest(&mut out, &self.execution_spec)?;
        push_digest(&mut out, &self.run_plan)?;
        out.push(u8::from(self.launch_manifest_absolute_paths));
        push_count(&mut out, self.files.len())?;
        for file in &self.files {
            push_string(&mut out, file.role())?;
            push_string(&mut out, file.relative_path())?;
            push_digest(&mut out, file.digest())?;
            out.extend_from_slice(&file.size().to_be_bytes());
            out.extend_from_slice(&file.mode().to_be_bytes());
        }
        Ok(out)
    }

    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, MaterializationError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

pub struct M0CapsuleMaterializationRequest<'a> {
    pub capsule: &'a BuiltM0Capsule,
    pub repository_bundle: &'a Path,
    pub bubblewrap_program: &'a Path,
    pub destination: &'a Path,
}

pub struct PublishedM0Capsule {
    root: PathBuf,
    launch_manifest: PathBuf,
    receipt_path: PathBuf,
    receipt: CapsuleMaterializationReceiptV1,
    receipt_digest: Digest,
    receipt_file_digest: Digest,
    receipt_file_size: u64,
}

impl PublishedM0Capsule {
    pub fn root(&self) -> &Path {
        &self.root
    }

    pub fn launch_manifest(&self) -> &Path {
        &self.launch_manifest
    }

    pub fn receipt_path(&self) -> &Path {
        &self.receipt_path
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
}

#[derive(Serialize)]
struct HostRuntimeArtifactPathWire {
    role: String,
    source: PathBuf,
}

#[derive(Serialize)]
struct HostLaunchManifestV2Wire {
    execution_spec: PathBuf,
    run_plan: PathBuf,
    guest_plan: PathBuf,
    guest_tool_map: PathBuf,
    isolation_policy: PathBuf,
    nix_closure: PathBuf,
    sandbox_invocation: PathBuf,
    bubblewrap_program: PathBuf,
    runtime_artifacts: Vec<HostRuntimeArtifactPathWire>,
}

pub fn materialize_m0_capsule(
    request: M0CapsuleMaterializationRequest<'_>,
) -> Result<PublishedM0Capsule, MaterializationError> {
    let final_root = prepare_final_root(request.destination)?;
    let bubblewrap_program = validate_bubblewrap_program(request.bubblewrap_program)?;
    require_capsule_cross_links(request.capsule)?;

    let final_name = final_root
        .file_name()
        .and_then(|name| name.to_str())
        .ok_or(MaterializationError::NonUtf8Path)?;
    let parent = final_root
        .parent()
        .ok_or(MaterializationError::InvalidDestination)?;
    let staging = create_staging_directory(parent, final_name)?;
    let mut guard = StagingGuard::new(staging.clone());
    create_private_directory(&staging.join("inputs"))?;
    create_private_directory(&staging.join("control"))?;

    let expected_inputs = expected_input_destinations()?;
    require_exact_spec_input_surface(request.capsule, &expected_inputs)?;
    let generated = generated_by_role(request.capsule)?;
    let mut input_paths = BTreeMap::<String, PathBuf>::new();
    let mut files = Vec::new();

    for (role, guest_destination) in &expected_inputs {
        let relative = guest_destination_to_relative(guest_destination)?;
        let staging_path = staging.join(&relative);
        let final_path = final_root.join(&relative);
        let spec_input = request
            .capsule
            .execution_spec()
            .inputs()
            .iter()
            .find(|input| input.role() == role)
            .ok_or_else(|| MaterializationError::MissingExecutionInput(role.clone()))?;

        let observed = if role == "repository-bundle" {
            copy_exact_file(
                request.repository_bundle,
                &staging_path,
                spec_input.digest(),
                spec_input.size(),
            )?
        } else {
            let artifact = generated
                .get(role)
                .ok_or_else(|| MaterializationError::MissingGeneratedArtifact(role.clone()))?;
            if artifact.destination() != guest_destination
                || artifact.digest() != spec_input.digest()
                || artifact.size() != spec_input.size()
            {
                return Err(MaterializationError::GeneratedArtifactMismatch(
                    role.clone(),
                ));
            }
            write_exact_file(
                &staging_path,
                artifact.bytes(),
                spec_input.digest(),
                spec_input.size(),
            )?
        };

        files.push(MaterializedFileV1 {
            role: format!("input:{role}"),
            relative_path: relative_to_string(&relative)?,
            digest: observed.0,
            size: observed.1,
            mode: FILE_MODE,
        });
        input_paths.insert(role.clone(), final_path);
    }

    let spec_digest = request
        .capsule
        .execution_spec()
        .digest(DigestAlgorithm::Sha256)?;
    let run_plan_digest = request
        .capsule
        .run_plan()
        .digest(DigestAlgorithm::Sha256)?;

    let spec_bytes = serde_json::to_vec(request.capsule.execution_spec())?;
    files.push(write_control(
        &staging,
        "control:execution-spec",
        EXECUTION_SPEC_PATH,
        &spec_bytes,
    )?);
    let run_plan_bytes = serde_json::to_vec(request.capsule.run_plan())?;
    files.push(write_control(
        &staging,
        "control:hermetic-run-plan",
        RUN_PLAN_PATH,
        &run_plan_bytes,
    )?);

    let launch = HostLaunchManifestV2Wire {
        execution_spec: final_root.join(EXECUTION_SPEC_PATH),
        run_plan: final_root.join(RUN_PLAN_PATH),
        guest_plan: required_input_path(&input_paths, "guest-verification-plan")?,
        guest_tool_map: required_input_path(&input_paths, "guest-tool-map")?,
        isolation_policy: required_input_path(&input_paths, "linux-isolation-policy")?,
        nix_closure: required_input_path(&input_paths, "nix-closure-manifest")?,
        sandbox_invocation: required_input_path(&input_paths, "sandbox-invocation")?,
        bubblewrap_program,
        runtime_artifacts: input_paths
            .iter()
            .map(|(role, source)| HostRuntimeArtifactPathWire {
                role: role.clone(),
                source: source.clone(),
            })
            .collect(),
    };
    let launch_bytes = serde_json::to_vec(&launch)?;
    files.push(write_control(
        &staging,
        "control:host-launch-manifest-v2",
        HOST_LAUNCH_MANIFEST_FILENAME,
        &launch_bytes,
    )?);

    let receipt = CapsuleMaterializationReceiptV1::new(
        request.capsule.construction_commitment().clone(),
        request.capsule.execution_subject().clone(),
        spec_digest,
        run_plan_digest,
        files,
    )?;
    let receipt_digest = receipt.digest(DigestAlgorithm::Sha256)?;
    let receipt_bytes = serde_json::to_vec(&receipt)?;
    let receipt_file_path = staging.join(MATERIALIZATION_RECEIPT_FILENAME);
    let (receipt_file_digest, receipt_file_size) = write_uncommitted_file(
        &receipt_file_path,
        &receipt_bytes,
        DigestAlgorithm::Sha256,
    )?;
    reparse_receipt(&receipt_file_path, &receipt, &receipt_digest)?;

    sync_directory(&staging.join("inputs"))?;
    sync_directory(&staging.join("control"))?;
    sync_directory(&staging)?;
    rename_noreplace(&staging, &final_root)?;
    guard.disarm();
    sync_directory(parent)?;

    verify_published_tree(
        &final_root,
        &receipt,
        &receipt_digest,
        &receipt_file_digest,
        receipt_file_size,
    )?;

    Ok(PublishedM0Capsule {
        launch_manifest: final_root.join(HOST_LAUNCH_MANIFEST_FILENAME),
        receipt_path: final_root.join(MATERIALIZATION_RECEIPT_FILENAME),
        root: final_root,
        receipt,
        receipt_digest,
        receipt_file_digest,
        receipt_file_size,
    })
}

fn require_capsule_cross_links(capsule: &BuiltM0Capsule) -> Result<(), MaterializationError> {
    if capsule.execution_spec().subject() != capsule.execution_subject() {
        return Err(MaterializationError::CapsuleCrossLinkMismatch);
    }
    let spec = capsule
        .execution_spec()
        .digest(DigestAlgorithm::Sha256)?;
    if capsule.run_plan().execution_spec() != &spec {
        return Err(MaterializationError::CapsuleCrossLinkMismatch);
    }
    let policy = capsule
        .isolation_policy()
        .digest(DigestAlgorithm::Sha256)
        .map_err(|error| MaterializationError::Isolation(error.to_string()))?;
    if capsule.run_plan().isolation_policy() != &policy {
        return Err(MaterializationError::CapsuleCrossLinkMismatch);
    }
    Ok(())
}

fn expected_input_destinations() -> Result<BTreeMap<String, String>, MaterializationError> {
    let mut values = BTreeMap::new();
    for (role, destination) in POLICY_INPUTS.iter().chain(CONTROL_INPUTS.iter()) {
        if values
            .insert((*role).to_owned(), (*destination).to_owned())
            .is_some()
        {
            return Err(MaterializationError::DuplicateProtocolInputRole(
                (*role).to_owned(),
            ));
        }
    }
    if values.len() != 8 {
        return Err(MaterializationError::ProtocolInputSurfaceMismatch);
    }
    Ok(values)
}

fn require_exact_spec_input_surface(
    capsule: &BuiltM0Capsule,
    expected: &BTreeMap<String, String>,
) -> Result<(), MaterializationError> {
    let actual = capsule
        .execution_spec()
        .inputs()
        .iter()
        .map(|input| input.role())
        .collect::<BTreeSet<_>>();
    let wanted = expected.keys().map(String::as_str).collect::<BTreeSet<_>>();
    if actual != wanted || actual.len() != capsule.execution_spec().inputs().len() {
        return Err(MaterializationError::ProtocolInputSurfaceMismatch);
    }
    Ok(())
}

fn generated_by_role<'a>(
    capsule: &'a BuiltM0Capsule,
) -> Result<BTreeMap<String, &'a GeneratedCapsuleArtifact>, MaterializationError> {
    let mut values = BTreeMap::new();
    for artifact in capsule.generated_artifacts() {
        if values.insert(artifact.role().to_owned(), artifact).is_some() {
            return Err(MaterializationError::DuplicateGeneratedRole(
                artifact.role().to_owned(),
            ));
        }
    }
    if values.len() != 7 || values.contains_key("repository-bundle") {
        return Err(MaterializationError::GeneratedArtifactSetMismatch);
    }
    Ok(values)
}

fn required_input_path(
    paths: &BTreeMap<String, PathBuf>,
    role: &str,
) -> Result<PathBuf, MaterializationError> {
    paths
        .get(role)
        .cloned()
        .ok_or_else(|| MaterializationError::MissingExecutionInput(role.to_owned()))
}

fn guest_destination_to_relative(value: &str) -> Result<PathBuf, MaterializationError> {
    if !value.starts_with("/inputs/") || value.ends_with('/') {
        return Err(MaterializationError::UnsafeGuestDestination(value.to_owned()));
    }
    let relative = Path::new(value.trim_start_matches('/'));
    let components = relative.components().collect::<Vec<_>>();
    if components.len() != 2
        || !matches!(components[0], Component::Normal(name) if name == "inputs")
        || !matches!(components[1], Component::Normal(_))
    {
        return Err(MaterializationError::UnsafeGuestDestination(value.to_owned()));
    }
    Ok(relative.to_path_buf())
}

fn expected_receipt_paths() -> Result<BTreeMap<String, String>, MaterializationError> {
    let mut expected = BTreeMap::new();
    for (role, destination) in expected_input_destinations()? {
        expected.insert(
            format!("input:{role}"),
            relative_to_string(&guest_destination_to_relative(&destination)?)?,
        );
    }
    expected.insert(
        "control:execution-spec".to_owned(),
        EXECUTION_SPEC_PATH.to_owned(),
    );
    expected.insert(
        "control:hermetic-run-plan".to_owned(),
        RUN_PLAN_PATH.to_owned(),
    );
    expected.insert(
        "control:host-launch-manifest-v2".to_owned(),
        HOST_LAUNCH_MANIFEST_FILENAME.to_owned(),
    );
    Ok(expected)
}

fn prepare_final_root(destination: &Path) -> Result<PathBuf, MaterializationError> {
    let absolute = if destination.is_absolute() {
        destination.to_path_buf()
    } else {
        std::env::current_dir()?.join(destination)
    };
    let name = absolute
        .file_name()
        .ok_or(MaterializationError::InvalidDestination)?;
    let parent = absolute
        .parent()
        .ok_or(MaterializationError::InvalidDestination)?;
    let parent = fs::canonicalize(parent)?;
    if !fs::metadata(&parent)?.is_dir() {
        return Err(MaterializationError::InvalidDestination);
    }
    let final_root = parent.join(name);
    match fs::symlink_metadata(&final_root) {
        Ok(_) => return Err(MaterializationError::DestinationExists(final_root)),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
        Err(error) => return Err(error.into()),
    }
    Ok(final_root)
}

fn validate_bubblewrap_program(path: &Path) -> Result<PathBuf, MaterializationError> {
    let text = path.to_str().ok_or(MaterializationError::NonUtf8Path)?;
    if !path.is_absolute()
        || !text.starts_with("/nix/store/")
        || !text.ends_with("/bin/bwrap")
        || text.contains("//")
        || text.contains("/../")
        || text.contains("/./")
    {
        return Err(MaterializationError::InvalidBubblewrapProgram(
            path.to_path_buf(),
        ));
    }
    Ok(path.to_path_buf())
}

fn create_staging_directory(parent: &Path, final_name: &str) -> Result<PathBuf, MaterializationError> {
    validate_text("destination name", final_name)?;
    for _ in 0..MAX_STAGE_ATTEMPTS {
        let sequence = STAGING_COUNTER.fetch_add(1, Ordering::Relaxed);
        let candidate = parent.join(format!(
            ".{final_name}.forge-stage-{}-{sequence}",
            std::process::id()
        ));
        match fs::create_dir(&candidate) {
            Ok(()) => {
                fs::set_permissions(&candidate, fs::Permissions::from_mode(DIRECTORY_MODE))?;
                return Ok(candidate);
            }
            Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
            Err(error) => return Err(error.into()),
        }
    }
    Err(MaterializationError::UnableToCreateStagingDirectory)
}

fn create_private_directory(path: &Path) -> Result<(), MaterializationError> {
    fs::create_dir(path)?;
    fs::set_permissions(path, fs::Permissions::from_mode(DIRECTORY_MODE))?;
    Ok(())
}

fn write_control(
    staging: &Path,
    role: &str,
    relative: &str,
    bytes: &[u8],
) -> Result<MaterializedFileV1, MaterializationError> {
    validate_safe_relative(relative)?;
    let digest = Digest::of_bytes(DigestAlgorithm::Sha256, bytes);
    let size = u64::try_from(bytes.len()).map_err(|_| MaterializationError::FileTooLarge)?;
    let observed = write_exact_file(&staging.join(relative), bytes, &digest, size)?;
    Ok(MaterializedFileV1 {
        role: role.to_owned(),
        relative_path: relative.to_owned(),
        digest: observed.0,
        size: observed.1,
        mode: FILE_MODE,
    })
}

fn write_exact_file(
    path: &Path,
    bytes: &[u8],
    expected_digest: &Digest,
    expected_size: u64,
) -> Result<(Digest, u64), MaterializationError> {
    let size = u64::try_from(bytes.len()).map_err(|_| MaterializationError::FileTooLarge)?;
    let digest = Digest::of_bytes(expected_digest.algorithm(), bytes);
    if &digest != expected_digest || size != expected_size {
        return Err(MaterializationError::SourceArtifactMismatch(
            path.to_path_buf(),
        ));
    }
    let mut file = open_new_file(path)?;
    file.write_all(bytes)?;
    file.sync_all()?;
    file.set_permissions(fs::Permissions::from_mode(FILE_MODE))?;
    file.sync_all()?;
    drop(file);
    require_fingerprint(path, expected_digest, expected_size, FILE_MODE)
}

fn write_uncommitted_file(
    path: &Path,
    bytes: &[u8],
    algorithm: DigestAlgorithm,
) -> Result<(Digest, u64), MaterializationError> {
    let digest = Digest::of_bytes(algorithm, bytes);
    let size = u64::try_from(bytes.len()).map_err(|_| MaterializationError::FileTooLarge)?;
    write_exact_file(path, bytes, &digest, size)
}

fn copy_exact_file(
    source: &Path,
    destination: &Path,
    expected_digest: &Digest,
    expected_size: u64,
) -> Result<(Digest, u64), MaterializationError> {
    let metadata = fs::symlink_metadata(source)?;
    if !metadata.file_type().is_file() {
        return Err(MaterializationError::BundleSourceNotRegular(
            source.to_path_buf(),
        ));
    }
    let mut source_file = OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
        .open(source)?;
    let mut destination_file = open_new_file(destination)?;
    let (digest, size) = copy_with_digest(
        &mut source_file,
        &mut destination_file,
        expected_digest.algorithm(),
    )?;
    if &digest != expected_digest || size != expected_size {
        return Err(MaterializationError::SourceArtifactMismatch(
            source.to_path_buf(),
        ));
    }
    destination_file.sync_all()?;
    destination_file.set_permissions(fs::Permissions::from_mode(FILE_MODE))?;
    destination_file.sync_all()?;
    drop(destination_file);
    require_fingerprint(destination, expected_digest, expected_size, FILE_MODE)
}

fn open_new_file(path: &Path) -> Result<File, MaterializationError> {
    Ok(OpenOptions::new()
        .write(true)
        .create_new(true)
        .mode(0o600)
        .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
        .open(path)?)
}

fn require_fingerprint(
    path: &Path,
    expected_digest: &Digest,
    expected_size: u64,
    expected_mode: u32,
) -> Result<(Digest, u64), MaterializationError> {
    let metadata = fs::symlink_metadata(path)?;
    if !metadata.file_type().is_file() {
        return Err(MaterializationError::MaterializedPathNotRegular(
            path.to_path_buf(),
        ));
    }
    let mode = metadata.permissions().mode() & 0o777;
    if mode != expected_mode {
        return Err(MaterializationError::FileModeMismatch {
            path: path.to_path_buf(),
            expected: expected_mode,
            actual: mode,
        });
    }
    let mut file = OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
        .open(path)?;
    let (digest, size) = hash_reader(&mut file, expected_digest.algorithm())?;
    if &digest != expected_digest || size != expected_size {
        return Err(MaterializationError::WrittenArtifactMismatch(
            path.to_path_buf(),
        ));
    }
    Ok((digest, size))
}

fn copy_with_digest(
    source: &mut File,
    destination: &mut File,
    algorithm: DigestAlgorithm,
) -> Result<(Digest, u64), MaterializationError> {
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
                    .checked_add(read as u64)
                    .ok_or(MaterializationError::FileTooLarge)?;
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
                    .checked_add(read as u64)
                    .ok_or(MaterializationError::FileTooLarge)?;
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

fn hash_reader(
    reader: &mut File,
    algorithm: DigestAlgorithm,
) -> Result<(Digest, u64), MaterializationError> {
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
                    .checked_add(read as u64)
                    .ok_or(MaterializationError::FileTooLarge)?;
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
                    .checked_add(read as u64)
                    .ok_or(MaterializationError::FileTooLarge)?;
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

fn reparse_receipt(
    path: &Path,
    expected: &CapsuleMaterializationReceiptV1,
    expected_digest: &Digest,
) -> Result<(), MaterializationError> {
    let bytes = read_nofollow(path)?;
    let parsed: CapsuleMaterializationReceiptV1 = serde_json::from_slice(&bytes)?;
    parsed.validate()?;
    if &parsed != expected || &parsed.digest(expected_digest.algorithm())? != expected_digest {
        return Err(MaterializationError::ReceiptRoundTripMismatch);
    }
    Ok(())
}

fn read_nofollow(path: &Path) -> Result<Vec<u8>, MaterializationError> {
    let metadata = fs::symlink_metadata(path)?;
    if !metadata.file_type().is_file() {
        return Err(MaterializationError::MaterializedPathNotRegular(
            path.to_path_buf(),
        ));
    }
    let mut file = OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
        .open(path)?;
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes)?;
    Ok(bytes)
}

fn sync_directory(path: &Path) -> Result<(), MaterializationError> {
    File::open(path)?.sync_all()?;
    Ok(())
}

fn rename_noreplace(source: &Path, destination: &Path) -> Result<(), MaterializationError> {
    let source = c_path(source)?;
    let destination_c = c_path(destination)?;
    let result = unsafe {
        libc::syscall(
            libc::SYS_renameat2,
            libc::AT_FDCWD,
            source.as_ptr(),
            libc::AT_FDCWD,
            destination_c.as_ptr(),
            libc::RENAME_NOREPLACE,
        )
    };
    if result != 0 {
        let error = std::io::Error::last_os_error();
        if error.raw_os_error() == Some(libc::EEXIST) {
            return Err(MaterializationError::DestinationExists(
                destination.to_path_buf(),
            ));
        }
        return Err(error.into());
    }
    Ok(())
}

fn c_path(path: &Path) -> Result<CString, MaterializationError> {
    CString::new(path.as_os_str().as_bytes())
        .map_err(|_| MaterializationError::PathContainsNul(path.to_path_buf()))
}

fn verify_published_tree(
    root: &Path,
    receipt: &CapsuleMaterializationReceiptV1,
    receipt_digest: &Digest,
    receipt_file_digest: &Digest,
    receipt_file_size: u64,
) -> Result<(), MaterializationError> {
    let metadata = fs::symlink_metadata(root)?;
    if !metadata.file_type().is_dir() {
        return Err(MaterializationError::PublishedRootNotDirectory);
    }
    let mode = metadata.permissions().mode() & 0o777;
    if mode != DIRECTORY_MODE {
        return Err(MaterializationError::RootModeMismatch(mode));
    }
    for file in receipt.files() {
        require_fingerprint(
            &root.join(file.relative_path()),
            file.digest(),
            file.size(),
            file.mode(),
        )?;
    }
    let receipt_path = root.join(MATERIALIZATION_RECEIPT_FILENAME);
    require_fingerprint(
        &receipt_path,
        receipt_file_digest,
        receipt_file_size,
        FILE_MODE,
    )?;
    reparse_receipt(&receipt_path, receipt, receipt_digest)?;
    Ok(())
}

fn relative_to_string(path: &Path) -> Result<String, MaterializationError> {
    validate_safe_relative_path(path)?;
    path.to_str()
        .map(ToOwned::to_owned)
        .ok_or(MaterializationError::NonUtf8Path)
}

fn validate_safe_relative(value: &str) -> Result<(), MaterializationError> {
    validate_text("relative path", value)?;
    validate_safe_relative_path(Path::new(value))
}

fn validate_safe_relative_path(path: &Path) -> Result<(), MaterializationError> {
    if path.is_absolute() || path.as_os_str().is_empty() {
        return Err(MaterializationError::UnsafeRelativePath(
            path.to_path_buf(),
        ));
    }
    if path
        .components()
        .any(|component| !matches!(component, Component::Normal(_)))
    {
        return Err(MaterializationError::UnsafeRelativePath(
            path.to_path_buf(),
        ));
    }
    Ok(())
}

fn validate_text(field: &'static str, value: &str) -> Result<(), MaterializationError> {
    if value.is_empty() || value.len() > MAX_TEXT || value.contains('\0') {
        Err(MaterializationError::InvalidText(field))
    } else {
        Ok(())
    }
}

fn push_count(out: &mut Vec<u8>, count: usize) -> Result<(), MaterializationError> {
    let count = u16::try_from(count).map_err(|_| MaterializationError::CanonicalLengthOverflow)?;
    out.extend_from_slice(&count.to_be_bytes());
    Ok(())
}

fn push_string(out: &mut Vec<u8>, value: &str) -> Result<(), MaterializationError> {
    validate_text("canonical string", value)?;
    let len = u16::try_from(value.len()).map_err(|_| MaterializationError::CanonicalLengthOverflow)?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), MaterializationError> {
    push_string(out, digest.algorithm().id())?;
    let len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| MaterializationError::CanonicalLengthOverflow)?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

struct StagingGuard {
    path: PathBuf,
    armed: bool,
}

impl StagingGuard {
    fn new(path: PathBuf) -> Self {
        Self { path, armed: true }
    }

    fn disarm(&mut self) {
        self.armed = false;
    }
}

impl Drop for StagingGuard {
    fn drop(&mut self) {
        if self.armed {
            let _ = fs::remove_dir_all(&self.path);
        }
    }
}

#[derive(Debug, Error)]
pub enum MaterializationError {
    #[error(transparent)]
    Core(#[from] ForgeCoreError),
    #[error(transparent)]
    Execution(#[from] ExecutionContractError),
    #[error(transparent)]
    HermeticRun(#[from] HermeticRunError),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error("isolation-policy digest failed during materialization: {0}")]
    Isolation(String),
    #[error("invalid materialization destination")]
    InvalidDestination,
    #[error("materialization destination already exists: {0:?}")]
    DestinationExists(PathBuf),
    #[error("unable to create a unique staging directory")]
    UnableToCreateStagingDirectory,
    #[error("path is not UTF-8")]
    NonUtf8Path,
    #[error("path contains NUL: {0:?}")]
    PathContainsNul(PathBuf),
    #[error("invalid bubblewrap program path: {0:?}")]
    InvalidBubblewrapProgram(PathBuf),
    #[error("protocol input role is duplicated: {0}")]
    DuplicateProtocolInputRole(String),
    #[error("D4A input surface is not exactly five policy plus three control inputs")]
    ProtocolInputSurfaceMismatch,
    #[error("execution spec is missing input role: {0}")]
    MissingExecutionInput(String),
    #[error("generated artifact role is duplicated: {0}")]
    DuplicateGeneratedRole(String),
    #[error("D4E generated artifact set is not exactly seven non-bundle inputs")]
    GeneratedArtifactSetMismatch,
    #[error("missing generated capsule artifact: {0}")]
    MissingGeneratedArtifact(String),
    #[error("generated artifact differs from final ExecutionSpec: {0}")]
    GeneratedArtifactMismatch(String),
    #[error("unsafe guest artifact destination: {0}")]
    UnsafeGuestDestination(String),
    #[error("unsafe relative capsule path: {0:?}")]
    UnsafeRelativePath(PathBuf),
    #[error("repository bundle source is not a regular file: {0:?}")]
    BundleSourceNotRegular(PathBuf),
    #[error("source artifact fingerprint mismatch: {0:?}")]
    SourceArtifactMismatch(PathBuf),
    #[error("materialized path is not a regular file: {0:?}")]
    MaterializedPathNotRegular(PathBuf),
    #[error("materialized file fingerprint mismatch: {0:?}")]
    WrittenArtifactMismatch(PathBuf),
    #[error("materialized file mode mismatch for {path:?}: expected {expected:o}, got {actual:o}")]
    FileModeMismatch {
        path: PathBuf,
        expected: u32,
        actual: u32,
    },
    #[error("file size overflow")]
    FileTooLarge,
    #[error("capsule semantic cross-links differ")]
    CapsuleCrossLinkMismatch,
    #[error("unsupported materialization receipt version: {0}")]
    UnsupportedReceiptVersion(u16),
    #[error("materialization receipt must declare absolute launch paths")]
    UnexpectedLaunchPathMode,
    #[error("materialization receipt file set differs from exact M0 layout")]
    ReceiptFileSetMismatch,
    #[error("materialization receipt path differs for role: {0}")]
    ReceiptPathMismatch(String),
    #[error("materialization receipt contains a duplicate role or path")]
    DuplicateReceiptEntry,
    #[error("materialization receipt file is empty: {0}")]
    EmptyMaterializedFile(String),
    #[error("materialization receipt mode differs for {role}: {mode:o}")]
    UnexpectedReceiptFileMode { role: String, mode: u32 },
    #[error("materialization receipt JSON round trip differs")]
    ReceiptRoundTripMismatch,
    #[error("published root is not a directory")]
    PublishedRootNotDirectory,
    #[error("published root mode differs from 0700: {0:o}")]
    RootModeMismatch(u32),
    #[error("invalid text field: {0}")]
    InvalidText(&'static str),
    #[error("canonical materialization field length overflow")]
    CanonicalLengthOverflow,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn test_directory(name: &str) -> PathBuf {
        let sequence = STAGING_COUNTER.fetch_add(1, Ordering::Relaxed);
        std::env::temp_dir().join(format!(
            "mycelix-forge-materializer-{name}-{}-{sequence}",
            std::process::id()
        ))
    }

    #[test]
    fn receipt_layout_is_exactly_eight_inputs_plus_three_controls() {
        let expected = expected_receipt_paths().unwrap();
        assert_eq!(expected.len(), 11);
        assert_eq!(
            expected.get("input:guest-tool-map").map(String::as_str),
            Some("inputs/guest-tool-map.json")
        );
        assert_eq!(
            expected
                .get("control:host-launch-manifest-v2")
                .map(String::as_str),
            Some(HOST_LAUNCH_MANIFEST_FILENAME)
        );
    }

    #[test]
    fn guest_destinations_are_flat_inputs_only() {
        assert_eq!(
            guest_destination_to_relative("/inputs/example.json").unwrap(),
            PathBuf::from("inputs/example.json")
        );
        assert!(guest_destination_to_relative("/inputs/a/b").is_err());
        assert!(guest_destination_to_relative("/trust/example.json").is_err());
        assert!(guest_destination_to_relative("/inputs/../escape").is_err());
    }

    #[test]
    fn exclusive_writer_rehashes_and_freezes_mode() {
        let root = test_directory("writer");
        fs::create_dir(&root).unwrap();
        let path = root.join("artifact");
        let bytes = b"exact capsule bytes";
        let expected = Digest::of_bytes(DigestAlgorithm::Sha256, bytes);
        let observed = write_exact_file(&path, bytes, &expected, bytes.len() as u64).unwrap();
        assert_eq!(observed.0, expected);
        assert_eq!(observed.1, bytes.len() as u64);
        assert_eq!(
            fs::metadata(&path).unwrap().permissions().mode() & 0o777,
            FILE_MODE
        );
        assert!(write_exact_file(&path, bytes, &expected, bytes.len() as u64).is_err());
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn no_replace_publish_refuses_existing_destination() {
        let root = test_directory("rename");
        fs::create_dir(&root).unwrap();
        let source = root.join("source");
        let destination = root.join("destination");
        fs::create_dir(&source).unwrap();
        fs::create_dir(&destination).unwrap();
        assert!(matches!(
            rename_noreplace(&source, &destination),
            Err(MaterializationError::DestinationExists(_))
        ));
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn receipt_digest_is_stable_for_canonical_file_order() {
        let expected = expected_receipt_paths().unwrap();
        let files = expected
            .iter()
            .enumerate()
            .map(|(index, (role, relative_path))| MaterializedFileV1 {
                role: role.clone(),
                relative_path: relative_path.clone(),
                digest: digest(index as u8 + 1),
                size: index as u64 + 1,
                mode: FILE_MODE,
            })
            .rev()
            .collect::<Vec<_>>();
        let first = CapsuleMaterializationReceiptV1::new(
            digest(0xA1),
            digest(0xA2),
            digest(0xA3),
            digest(0xA4),
            files,
        )
        .unwrap();
        let second = serde_json::from_slice::<CapsuleMaterializationReceiptV1>(
            &serde_json::to_vec(&first).unwrap(),
        )
        .unwrap();
        assert_eq!(
            first.digest(DigestAlgorithm::Sha256).unwrap(),
            second.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }
}
