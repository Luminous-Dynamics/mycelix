// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Structural collector for the complete active gittuf policy trust inventory.
//!
//! The collector reads Git plumbing output and DSSE payloads directly. Human
//! CLI presentation output is never part of this security boundary.

use base64::{engine::general_purpose::STANDARD as BASE64, Engine as _};
use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_gittuf::{gittuf_policy_subject_commitment, POLICY_REF};
use mycelix_forge_gittuf_trust_profile::{
    qualify_local_key_profile, PolicyTrustInventory, QualifiedLocalKeyTrustProfile,
    TrustProfileError, VerificationMethod,
};
use mycelix_forge_repository::{
    GitObjectAlgorithm, GitObjectId, RepositoryPolicyState, RepositoryVerificationError,
};
use serde_json::{Map, Value};
use std::{
    collections::BTreeSet,
    path::{Path, PathBuf},
    process::Command,
};
use thiserror::Error;

const RECEIPT_DOMAIN_V1: &[u8] = b"mycelix-forge/gittuf-policy-inventory/v1\0";
const DSSE_PAYLOAD_TYPE: &str = "application/vnd.gittuf+json";
const ROOT_V02: &str = "https://gittuf.dev/policy/root/v0.2";
const TARGETS_V02: &str = "http://gittuf.dev/policy/rule-file/v0.2";
const MAX_METADATA_FILES: usize = 4096;
const MAX_ROLE_NAME: usize = 1024;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ByteCommandSpec {
    pub program: PathBuf,
    pub args: Vec<String>,
    pub cwd: PathBuf,
    pub env: Vec<(String, String)>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ByteCommandOutput {
    pub status: i32,
    pub stdout: Vec<u8>,
    pub stderr: Vec<u8>,
}

pub trait ByteCommandRunner {
    fn run(&self, spec: &ByteCommandSpec) -> Result<ByteCommandOutput, PolicyInventoryError>;
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SystemByteCommandRunner;

impl ByteCommandRunner for SystemByteCommandRunner {
    fn run(&self, spec: &ByteCommandSpec) -> Result<ByteCommandOutput, PolicyInventoryError> {
        let output = Command::new(&spec.program)
            .args(&spec.args)
            .current_dir(&spec.cwd)
            .envs(spec.env.iter().map(|(key, value)| (key, value)))
            .output()
            .map_err(|source| PolicyInventoryError::SpawnCommand {
                program: spec.program.clone(),
                source,
            })?;
        Ok(ByteCommandOutput {
            status: output.status.code().unwrap_or(-1),
            stdout: output.stdout,
            stderr: output.stderr,
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct MetadataFileObservation {
    role: String,
    blob: GitObjectId,
    payload_digest: Digest,
    methods: Vec<VerificationMethod>,
}

impl MetadataFileObservation {
    pub fn role(&self) -> &str {
        &self.role
    }

    pub fn blob(&self) -> &GitObjectId {
        &self.blob
    }

    pub fn payload_digest(&self) -> &Digest {
        &self.payload_digest
    }

    pub fn methods(&self) -> &[VerificationMethod] {
        &self.methods
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PolicyInventoryReceipt {
    policy_state_digest: Digest,
    policy_tip: GitObjectId,
    policy_tree: GitObjectId,
    metadata_tree: GitObjectId,
    files: Vec<MetadataFileObservation>,
    inventory: PolicyTrustInventory,
}

impl PolicyInventoryReceipt {
    pub fn policy_state_digest(&self) -> &Digest {
        &self.policy_state_digest
    }

    pub fn policy_tip(&self) -> &GitObjectId {
        &self.policy_tip
    }

    pub fn policy_tree(&self) -> &GitObjectId {
        &self.policy_tree
    }

    pub fn metadata_tree(&self) -> &GitObjectId {
        &self.metadata_tree
    }

    pub fn files(&self) -> &[MetadataFileObservation] {
        &self.files
    }

    pub fn inventory(&self) -> &PolicyTrustInventory {
        &self.inventory
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, PolicyInventoryError> {
        let mut out = Vec::new();
        out.extend_from_slice(RECEIPT_DOMAIN_V1);
        push_digest(&mut out, &self.policy_state_digest)?;
        push_git_object(&mut out, &self.policy_tip)?;
        push_git_object(&mut out, &self.policy_tree)?;
        push_git_object(&mut out, &self.metadata_tree)?;
        push_count(&mut out, self.files.len(), "metadata files")?;
        for file in &self.files {
            push_string(&mut out, &file.role, "role")?;
            push_git_object(&mut out, &file.blob)?;
            push_digest(&mut out, &file.payload_digest)?;
            push_count(&mut out, file.methods.len(), "methods")?;
            for method in &file.methods {
                out.push(method_code(*method));
            }
        }
        push_digest(
            &mut out,
            &self.inventory.digest(DigestAlgorithm::Sha256)?,
        )?;
        Ok(out)
    }

    pub fn commitment(&self) -> Result<Digest, PolicyInventoryError> {
        Ok(Digest::of_bytes(
            DigestAlgorithm::Sha256,
            &self.canonical_bytes()?,
        ))
    }
}

pub struct GittufPolicyInventoryCollector<R> {
    runner: R,
    git_binary: PathBuf,
}

impl GittufPolicyInventoryCollector<SystemByteCommandRunner> {
    pub fn system() -> Self {
        Self::new(SystemByteCommandRunner, "git")
    }
}

impl<R: ByteCommandRunner> GittufPolicyInventoryCollector<R> {
    pub fn new(runner: R, git_binary: impl Into<PathBuf>) -> Self {
        Self {
            runner,
            git_binary: git_binary.into(),
        }
    }

    pub fn collect(
        &self,
        repository_path: &Path,
        policy_state: &RepositoryPolicyState,
    ) -> Result<PolicyInventoryReceipt, PolicyInventoryError> {
        let object_format = self.read_object_format(repository_path)?;
        let before = self.read_required_ref(repository_path, POLICY_REF, object_format)?;
        validate_policy_binding(policy_state, object_format, &before)?;

        let policy_tree = self.resolve_commit_tree(repository_path, &before, object_format)?;
        let metadata_tree =
            self.require_local_only_top_level(repository_path, &policy_tree, object_format)?;
        let entries = self.list_metadata_tree(repository_path, &metadata_tree, object_format)?;
        if entries.len() > MAX_METADATA_FILES {
            return Err(PolicyInventoryError::TooManyMetadataFiles(entries.len()));
        }
        if !entries.iter().any(|entry| entry.path == "root.json") {
            return Err(PolicyInventoryError::MissingRequiredRole("root"));
        }
        if !entries.iter().any(|entry| entry.path == "targets.json") {
            return Err(PolicyInventoryError::MissingRequiredRole("targets"));
        }

        let mut files = Vec::with_capacity(entries.len());
        let mut all_methods = BTreeSet::new();
        for entry in entries {
            let role = role_from_metadata_path(&entry.path)?;
            let envelope = self.read_blob(repository_path, &entry.object)?;
            let payload = decode_dsse_payload(&envelope)?;
            let methods = parse_metadata_payload(&role, &payload)?;
            all_methods.extend(methods.iter().copied());
            files.push(MetadataFileObservation {
                role,
                blob: entry.object,
                payload_digest: Digest::of_bytes(DigestAlgorithm::Sha256, &payload),
                methods,
            });
        }
        files.sort();

        let after = self.read_required_ref(repository_path, POLICY_REF, object_format)?;
        if before != after {
            return Err(PolicyInventoryError::PolicyChangedDuringInventory);
        }

        let policy_state_digest = policy_state.digest(DigestAlgorithm::Sha256)?;
        let inventory = PolicyTrustInventory::new(
            policy_state_digest.clone(),
            all_methods.into_iter().collect(),
        )?;

        Ok(PolicyInventoryReceipt {
            policy_state_digest,
            policy_tip: before,
            policy_tree,
            metadata_tree,
            files,
            inventory,
        })
    }

    pub fn collect_and_qualify_local_key_profile(
        &self,
        repository_path: &Path,
        policy_state: &RepositoryPolicyState,
    ) -> Result<(PolicyInventoryReceipt, QualifiedLocalKeyTrustProfile), PolicyInventoryError> {
        let receipt = self.collect(repository_path, policy_state)?;
        let qualified = qualify_local_key_profile(receipt.inventory())?;
        Ok((receipt, qualified))
    }

    fn read_object_format(&self, cwd: &Path) -> Result<GitObjectAlgorithm, PolicyInventoryError> {
        let output = self.run(
            cwd,
            vec!["rev-parse".into(), "--show-object-format".into()],
        )?;
        require_success(
            &self.git_binary,
            &["rev-parse", "--show-object-format"],
            &output,
        )?;
        match utf8_trimmed(&output.stdout, "object format")? {
            "sha1" => Ok(GitObjectAlgorithm::Sha1),
            "sha256" => Ok(GitObjectAlgorithm::Sha256),
            other => Err(PolicyInventoryError::UnsupportedObjectFormat(other.to_owned())),
        }
    }

    fn read_required_ref(
        &self,
        cwd: &Path,
        reference: &str,
        algorithm: GitObjectAlgorithm,
    ) -> Result<GitObjectId, PolicyInventoryError> {
        let output = self.run(
            cwd,
            vec!["rev-parse".into(), "--verify".into(), reference.into()],
        )?;
        require_success(
            &self.git_binary,
            &["rev-parse", "--verify", reference],
            &output,
        )?;
        parse_git_object_id(utf8_trimmed(&output.stdout, "Git object ID")?, algorithm)
    }

    fn resolve_commit_tree(
        &self,
        cwd: &Path,
        policy_tip: &GitObjectId,
        algorithm: GitObjectAlgorithm,
    ) -> Result<GitObjectId, PolicyInventoryError> {
        let spec = format!("{}^{{tree}}", object_hex(policy_tip));
        let output = self.run(
            cwd,
            vec!["rev-parse".into(), "--verify".into(), spec.clone()],
        )?;
        require_success(
            &self.git_binary,
            &["rev-parse", "--verify", &spec],
            &output,
        )?;
        parse_git_object_id(utf8_trimmed(&output.stdout, "policy tree ID")?, algorithm)
    }

    fn require_local_only_top_level(
        &self,
        cwd: &Path,
        tree: &GitObjectId,
        algorithm: GitObjectAlgorithm,
    ) -> Result<GitObjectId, PolicyInventoryError> {
        let entries = self.list_tree(cwd, tree, algorithm)?;
        if entries.len() != 1 {
            return Err(PolicyInventoryError::UnexpectedPolicyTopLevel);
        }
        let entry = &entries[0];
        if entry.mode != "040000" || entry.kind != "tree" || entry.path != "metadata" {
            return Err(PolicyInventoryError::UnexpectedPolicyTopLevel);
        }
        Ok(entry.object.clone())
    }

    fn list_metadata_tree(
        &self,
        cwd: &Path,
        tree: &GitObjectId,
        algorithm: GitObjectAlgorithm,
    ) -> Result<Vec<RawTreeEntry>, PolicyInventoryError> {
        let entries = self.list_tree(cwd, tree, algorithm)?;
        for entry in &entries {
            if entry.mode != "100644" || entry.kind != "blob" {
                return Err(PolicyInventoryError::UnsupportedMetadataTreeEntry(
                    entry.path.clone(),
                ));
            }
            role_from_metadata_path(&entry.path)?;
        }
        Ok(entries)
    }

    fn list_tree(
        &self,
        cwd: &Path,
        tree: &GitObjectId,
        algorithm: GitObjectAlgorithm,
    ) -> Result<Vec<RawTreeEntry>, PolicyInventoryError> {
        let tree_hex = object_hex(tree);
        let output = self.run(
            cwd,
            vec!["ls-tree".into(), "-z".into(), tree_hex.clone()],
        )?;
        require_success(&self.git_binary, &["ls-tree", "-z", &tree_hex], &output)?;
        parse_ls_tree(&output.stdout, algorithm)
    }

    fn read_blob(
        &self,
        cwd: &Path,
        blob: &GitObjectId,
    ) -> Result<Vec<u8>, PolicyInventoryError> {
        let blob_hex = object_hex(blob);
        let output = self.run(
            cwd,
            vec!["cat-file".into(), "blob".into(), blob_hex.clone()],
        )?;
        require_success(
            &self.git_binary,
            &["cat-file", "blob", &blob_hex],
            &output,
        )?;
        Ok(output.stdout)
    }

    fn run(
        &self,
        cwd: &Path,
        args: Vec<String>,
    ) -> Result<ByteCommandOutput, PolicyInventoryError> {
        self.runner.run(&ByteCommandSpec {
            program: self.git_binary.clone(),
            args,
            cwd: cwd.to_path_buf(),
            env: vec![
                ("LC_ALL".into(), "C".into()),
                ("LANG".into(), "C".into()),
                ("GIT_CONFIG_NOSYSTEM".into(), "1".into()),
                ("GIT_CONFIG_GLOBAL".into(), "/dev/null".into()),
                ("GIT_NO_LAZY_FETCH".into(), "1".into()),
                ("GIT_TERMINAL_PROMPT".into(), "0".into()),
            ],
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct RawTreeEntry {
    mode: String,
    kind: String,
    object: GitObjectId,
    path: String,
}

fn validate_policy_binding(
    policy_state: &RepositoryPolicyState,
    object_format: GitObjectAlgorithm,
    policy_tip: &GitObjectId,
) -> Result<(), PolicyInventoryError> {
    let expected = gittuf_policy_subject_commitment(
        object_format,
        policy_tip,
        policy_state.policy_digest().algorithm(),
    )?;
    if &expected == policy_state.policy_digest() {
        Ok(())
    } else {
        Err(PolicyInventoryError::PolicySubjectMismatch)
    }
}

fn parse_ls_tree(
    bytes: &[u8],
    algorithm: GitObjectAlgorithm,
) -> Result<Vec<RawTreeEntry>, PolicyInventoryError> {
    let mut entries = Vec::new();
    let mut paths = BTreeSet::new();
    for raw in bytes.split(|byte| *byte == 0).filter(|entry| !entry.is_empty()) {
        let tab = raw
            .iter()
            .position(|byte| *byte == b'\t')
            .ok_or(PolicyInventoryError::MalformedTreeEntry)?;
        let header = std::str::from_utf8(&raw[..tab])
            .map_err(|_| PolicyInventoryError::NonUtf8TreeHeader)?;
        let path = std::str::from_utf8(&raw[tab + 1..])
            .map_err(|_| PolicyInventoryError::NonUtf8MetadataName)?;
        let mut parts = header.split_whitespace();
        let mode = parts
            .next()
            .ok_or(PolicyInventoryError::MalformedTreeEntry)?;
        let kind = parts
            .next()
            .ok_or(PolicyInventoryError::MalformedTreeEntry)?;
        let oid = parts
            .next()
            .ok_or(PolicyInventoryError::MalformedTreeEntry)?;
        if parts.next().is_some() {
            return Err(PolicyInventoryError::MalformedTreeEntry);
        }
        if !paths.insert(path.to_owned()) {
            return Err(PolicyInventoryError::DuplicateTreePath(path.to_owned()));
        }
        entries.push(RawTreeEntry {
            mode: mode.to_owned(),
            kind: kind.to_owned(),
            object: parse_git_object_id(oid, algorithm)?,
            path: path.to_owned(),
        });
    }
    entries.sort_by(|a, b| a.path.cmp(&b.path));
    Ok(entries)
}

fn role_from_metadata_path(path: &str) -> Result<String, PolicyInventoryError> {
    let role = path
        .strip_suffix(".json")
        .ok_or_else(|| PolicyInventoryError::UnexpectedMetadataFile(path.to_owned()))?;
    if role.is_empty()
        || role.len() > MAX_ROLE_NAME
        || role.contains('/')
        || role.contains('\\')
        || role == "."
        || role == ".."
    {
        return Err(PolicyInventoryError::UnexpectedMetadataFile(
            path.to_owned(),
        ));
    }
    Ok(role.to_owned())
}

fn decode_dsse_payload(envelope_bytes: &[u8]) -> Result<Vec<u8>, PolicyInventoryError> {
    let value: Value = serde_json::from_slice(envelope_bytes)?;
    let object = value
        .as_object()
        .ok_or(PolicyInventoryError::MalformedDsseEnvelope)?;
    let payload = object
        .get("payload")
        .and_then(Value::as_str)
        .ok_or(PolicyInventoryError::MalformedDsseEnvelope)?;
    let payload_type = object
        .get("payloadType")
        .and_then(Value::as_str)
        .ok_or(PolicyInventoryError::MalformedDsseEnvelope)?;
    if payload_type != DSSE_PAYLOAD_TYPE {
        return Err(PolicyInventoryError::UnexpectedDssePayloadType(
            payload_type.to_owned(),
        ));
    }
    Ok(BASE64.decode(payload.as_bytes())?)
}

fn parse_metadata_payload(
    role: &str,
    payload: &[u8],
) -> Result<Vec<VerificationMethod>, PolicyInventoryError> {
    let value: Value = serde_json::from_slice(payload)?;
    let object = value
        .as_object()
        .ok_or(PolicyInventoryError::MalformedPolicyMetadata)?;
    match role {
        "root" => parse_root_metadata(object),
        _ => parse_targets_metadata(role, object),
    }
}

fn parse_root_metadata(
    object: &Map<String, Value>,
) -> Result<Vec<VerificationMethod>, PolicyInventoryError> {
    require_type(object, "root")?;
    reject_external_root_features(object)?;
    let schema = schema_version(object)?;
    let mut methods = BTreeSet::new();
    match schema {
        None => collect_required_key_map(object, "keys", &mut methods)?,
        Some(ROOT_V02) => collect_required_principal_map(object, "principals", &mut methods)?,
        Some(other) => {
            return Err(PolicyInventoryError::UnsupportedRootSchema(
                other.to_owned(),
            ))
        }
    }
    Ok(methods.into_iter().collect())
}

fn parse_targets_metadata(
    role: &str,
    object: &Map<String, Value>,
) -> Result<Vec<VerificationMethod>, PolicyInventoryError> {
    require_type(object, "targets")?;
    let schema = schema_version(object)?;
    if !matches!(schema, None | Some(TARGETS_V02)) {
        return Err(PolicyInventoryError::UnsupportedTargetsSchema {
            role: role.to_owned(),
            schema: schema.unwrap_or_default().to_owned(),
        });
    }
    let Some(delegations) = object.get("delegations") else {
        return Ok(Vec::new());
    };
    if delegations.is_null() {
        return Ok(Vec::new());
    }
    let delegations = delegations
        .as_object()
        .ok_or_else(|| PolicyInventoryError::MalformedDelegations(role.to_owned()))?;
    let mut methods = BTreeSet::new();
    match schema {
        None => collect_optional_key_map(delegations, "keys", &mut methods)?,
        Some(TARGETS_V02) => {
            collect_optional_principal_map(delegations, "principals", &mut methods)?
        }
        Some(_) => unreachable!("unsupported schema rejected above"),
    }
    Ok(methods.into_iter().collect())
}

fn schema_version(object: &Map<String, Value>) -> Result<Option<&str>, PolicyInventoryError> {
    match object.get("schemaVersion") {
        None => Ok(None),
        Some(Value::String(value)) => Ok(Some(value.as_str())),
        Some(_) => Err(PolicyInventoryError::MalformedSchemaVersion),
    }
}

fn reject_external_root_features(object: &Map<String, Value>) -> Result<(), PolicyInventoryError> {
    for field in ["propagations", "hooks"] {
        if let Some(value) = object.get(field) {
            let empty = match value {
                Value::Null => true,
                Value::Array(values) => values.is_empty(),
                Value::Object(values) => values.is_empty(),
                _ => false,
            };
            if !empty {
                return Err(PolicyInventoryError::UnsupportedExternalPolicyFeature(
                    field,
                ));
            }
        }
    }
    if object
        .get("multiRepository")
        .is_some_and(|value| !value.is_null())
    {
        return Err(PolicyInventoryError::UnsupportedExternalPolicyFeature(
            "multiRepository",
        ));
    }
    Ok(())
}

fn collect_required_key_map(
    object: &Map<String, Value>,
    field: &'static str,
    methods: &mut BTreeSet<VerificationMethod>,
) -> Result<(), PolicyInventoryError> {
    let value = object
        .get(field)
        .ok_or(PolicyInventoryError::MissingPrincipalCollection(field))?;
    collect_key_map_value(value, field, methods, true)
}

fn collect_optional_key_map(
    object: &Map<String, Value>,
    field: &'static str,
    methods: &mut BTreeSet<VerificationMethod>,
) -> Result<(), PolicyInventoryError> {
    match object.get(field) {
        Some(value) => collect_key_map_value(value, field, methods, false),
        None => Ok(()),
    }
}

fn collect_key_map_value(
    value: &Value,
    field: &'static str,
    methods: &mut BTreeSet<VerificationMethod>,
    require_nonempty: bool,
) -> Result<(), PolicyInventoryError> {
    let keys = value
        .as_object()
        .ok_or(PolicyInventoryError::MalformedPrincipalCollection(field))?;
    if require_nonempty && keys.is_empty() {
        return Err(PolicyInventoryError::MissingPrincipalCollection(field));
    }
    for key in keys.values() {
        methods.insert(method_from_key(key)?);
    }
    Ok(())
}

fn collect_required_principal_map(
    object: &Map<String, Value>,
    field: &'static str,
    methods: &mut BTreeSet<VerificationMethod>,
) -> Result<(), PolicyInventoryError> {
    let value = object
        .get(field)
        .ok_or(PolicyInventoryError::MissingPrincipalCollection(field))?;
    collect_principal_map_value(value, field, methods, true)
}

fn collect_optional_principal_map(
    object: &Map<String, Value>,
    field: &'static str,
    methods: &mut BTreeSet<VerificationMethod>,
) -> Result<(), PolicyInventoryError> {
    match object.get(field) {
        Some(value) => collect_principal_map_value(value, field, methods, false),
        None => Ok(()),
    }
}

fn collect_principal_map_value(
    value: &Value,
    field: &'static str,
    methods: &mut BTreeSet<VerificationMethod>,
    require_nonempty: bool,
) -> Result<(), PolicyInventoryError> {
    let principals = value
        .as_object()
        .ok_or(PolicyInventoryError::MalformedPrincipalCollection(field))?;
    if require_nonempty && principals.is_empty() {
        return Err(PolicyInventoryError::MissingPrincipalCollection(field));
    }
    for principal in principals.values() {
        collect_principal_methods(principal, methods)?;
    }
    Ok(())
}

fn collect_principal_methods(
    principal: &Value,
    methods: &mut BTreeSet<VerificationMethod>,
) -> Result<(), PolicyInventoryError> {
    let object = principal
        .as_object()
        .ok_or(PolicyInventoryError::UnknownPrincipalShape)?;
    if object.contains_key("keytype") {
        methods.insert(method_from_key(principal)?);
        return Ok(());
    }
    if object.get("personID").and_then(Value::as_str).is_some() {
        let keys = object
            .get("keys")
            .and_then(Value::as_object)
            .ok_or(PolicyInventoryError::PersonMissingKeys)?;
        if keys.is_empty() {
            return Err(PolicyInventoryError::PersonMissingKeys);
        }
        for key in keys.values() {
            methods.insert(method_from_key(key)?);
        }
        return Ok(());
    }
    Err(PolicyInventoryError::UnknownPrincipalShape)
}

fn method_from_key(value: &Value) -> Result<VerificationMethod, PolicyInventoryError> {
    let object = value
        .as_object()
        .ok_or(PolicyInventoryError::MalformedKey)?;
    let key_type = object
        .get("keytype")
        .and_then(Value::as_str)
        .ok_or(PolicyInventoryError::MalformedKey)?;
    match key_type {
        "gpg" => Ok(VerificationMethod::Gpg),
        "ssh" => Ok(VerificationMethod::Ssh),
        "sigstore-oidc" => Ok(VerificationMethod::Sigstore),
        other => Err(PolicyInventoryError::UnknownKeyType(other.to_owned())),
    }
}

fn require_type(
    object: &Map<String, Value>,
    expected: &'static str,
) -> Result<(), PolicyInventoryError> {
    match object.get("type") {
        Some(Value::String(actual)) if actual == expected => Ok(()),
        Some(Value::String(actual)) => Err(PolicyInventoryError::UnexpectedMetadataType {
            expected,
            actual: actual.clone(),
        }),
        _ => Err(PolicyInventoryError::MalformedPolicyMetadata),
    }
}

fn parse_git_object_id(
    value: &str,
    algorithm: GitObjectAlgorithm,
) -> Result<GitObjectId, PolicyInventoryError> {
    let bytes = hex::decode(value)
        .map_err(|_| PolicyInventoryError::InvalidGitObjectHex(value.to_owned()))?;
    Ok(GitObjectId::new(algorithm, bytes)?)
}

fn object_hex(object: &GitObjectId) -> String {
    hex::encode(object.as_bytes())
}

fn utf8_trimmed<'a>(
    bytes: &'a [u8],
    field: &'static str,
) -> Result<&'a str, PolicyInventoryError> {
    std::str::from_utf8(bytes)
        .map(str::trim)
        .map_err(|_| PolicyInventoryError::NonUtf8CommandOutput(field))
}

fn require_success(
    program: &Path,
    args: &[&str],
    output: &ByteCommandOutput,
) -> Result<(), PolicyInventoryError> {
    if output.status == 0 {
        Ok(())
    } else {
        Err(PolicyInventoryError::CommandFailed {
            program: program.to_path_buf(),
            args: args.iter().map(|arg| (*arg).to_owned()).collect(),
            status: output.status,
            stderr: String::from_utf8_lossy(&output.stderr).into_owned(),
        })
    }
}

fn method_code(method: VerificationMethod) -> u8 {
    match method {
        VerificationMethod::Gpg => 1,
        VerificationMethod::Ssh => 2,
        VerificationMethod::Sigstore => 3,
    }
}

fn push_count(
    out: &mut Vec<u8>,
    count: usize,
    field: &'static str,
) -> Result<(), PolicyInventoryError> {
    let count = u16::try_from(count)
        .map_err(|_| PolicyInventoryError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&count.to_be_bytes());
    Ok(())
}

fn push_string(
    out: &mut Vec<u8>,
    value: &str,
    field: &'static str,
) -> Result<(), PolicyInventoryError> {
    let len = u16::try_from(value.len())
        .map_err(|_| PolicyInventoryError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), PolicyInventoryError> {
    push_string(out, digest.algorithm().id(), "digest algorithm")?;
    let len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| PolicyInventoryError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

fn push_git_object(
    out: &mut Vec<u8>,
    object: &GitObjectId,
) -> Result<(), PolicyInventoryError> {
    out.push(match object.algorithm() {
        GitObjectAlgorithm::Sha1 => 1,
        GitObjectAlgorithm::Sha256 => 2,
    });
    let len = u16::try_from(object.as_bytes().len())
        .map_err(|_| PolicyInventoryError::CanonicalFieldTooLarge("Git object"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(object.as_bytes());
    Ok(())
}

#[derive(Debug, Error)]
pub enum PolicyInventoryError {
    #[error(transparent)]
    Repository(#[from] RepositoryVerificationError),
    #[error(transparent)]
    Adapter(#[from] mycelix_forge_gittuf::AdapterError),
    #[error(transparent)]
    TrustProfile(#[from] TrustProfileError),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error(transparent)]
    Base64(#[from] base64::DecodeError),
    #[error("failed to spawn {program}: {source}")]
    SpawnCommand {
        program: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("command failed: {program:?} {args:?}, status={status}, stderr={stderr}")]
    CommandFailed {
        program: PathBuf,
        args: Vec<String>,
        status: i32,
        stderr: String,
    },
    #[error("non-UTF-8 command output: {0}")]
    NonUtf8CommandOutput(&'static str),
    #[error("unsupported Git object format: {0}")]
    UnsupportedObjectFormat(String),
    #[error("invalid Git object hex: {0}")]
    InvalidGitObjectHex(String),
    #[error("gittuf policy ref does not match the supplied Forge policy state")]
    PolicySubjectMismatch,
    #[error("gittuf policy ref moved during trust inventory")]
    PolicyChangedDuringInventory,
    #[error("local-only profile requires policy top level to contain only metadata/")]
    UnexpectedPolicyTopLevel,
    #[error("too many metadata files: {0}")]
    TooManyMetadataFiles(usize),
    #[error("missing required metadata role: {0}")]
    MissingRequiredRole(&'static str),
    #[error("malformed Git tree entry")]
    MalformedTreeEntry,
    #[error("duplicate Git tree path: {0}")]
    DuplicateTreePath(String),
    #[error("non-UTF-8 Git tree header")]
    NonUtf8TreeHeader,
    #[error("non-UTF-8 gittuf metadata filename")]
    NonUtf8MetadataName,
    #[error("unsupported metadata tree entry: {0}")]
    UnsupportedMetadataTreeEntry(String),
    #[error("unexpected metadata file: {0}")]
    UnexpectedMetadataFile(String),
    #[error("malformed DSSE envelope")]
    MalformedDsseEnvelope,
    #[error("unexpected DSSE payload type: {0}")]
    UnexpectedDssePayloadType(String),
    #[error("malformed gittuf policy metadata")]
    MalformedPolicyMetadata,
    #[error("malformed schemaVersion field")]
    MalformedSchemaVersion,
    #[error("unexpected metadata type: expected {expected}, got {actual}")]
    UnexpectedMetadataType {
        expected: &'static str,
        actual: String,
    },
    #[error("unsupported root metadata schema: {0}")]
    UnsupportedRootSchema(String),
    #[error("unsupported targets metadata schema for {role}: {schema}")]
    UnsupportedTargetsSchema { role: String, schema: String },
    #[error("unsupported external policy feature in local-only profile: {0}")]
    UnsupportedExternalPolicyFeature(&'static str),
    #[error("malformed delegations in role {0}")]
    MalformedDelegations(String),
    #[error("missing or empty principal collection: {0}")]
    MissingPrincipalCollection(&'static str),
    #[error("malformed principal collection: {0}")]
    MalformedPrincipalCollection(&'static str),
    #[error("unrecognized principal shape")]
    UnknownPrincipalShape,
    #[error("person principal has no usable keys")]
    PersonMissingKeys,
    #[error("malformed key metadata")]
    MalformedKey,
    #[error("unknown gittuf key type: {0}")]
    UnknownKeyType(String),
    #[error("canonical field is too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn v02_root_finds_nested_person_and_sigstore_methods() {
        let payload = serde_json::json!({
            "type": "root",
            "schemaVersion": ROOT_V02,
            "principals": {
                "alice": {
                    "personID": "alice",
                    "keys": {
                        "a": {"keytype": "ssh"},
                        "b": {"keytype": "gpg"}
                    },
                    "associatedIdentities": {},
                    "custom": {}
                },
                "sig": {"keytype": "sigstore-oidc"}
            },
            "roles": {}
        });
        let methods =
            parse_metadata_payload("root", &serde_json::to_vec(&payload).unwrap()).unwrap();
        assert_eq!(
            methods,
            vec![
                VerificationMethod::Gpg,
                VerificationMethod::Ssh,
                VerificationMethod::Sigstore
            ]
        );
    }

    #[test]
    fn v01_targets_finds_delegated_keys() {
        let payload = serde_json::json!({
            "type": "targets",
            "delegations": {
                "keys": {
                    "a": {"keytype": "ssh"},
                    "b": {"keytype": "gpg"}
                },
                "roles": []
            }
        });
        let methods =
            parse_metadata_payload("targets", &serde_json::to_vec(&payload).unwrap()).unwrap();
        assert_eq!(methods, vec![VerificationMethod::Gpg, VerificationMethod::Ssh]);
    }

    #[test]
    fn external_policy_features_fail_closed() {
        let payload = serde_json::json!({
            "type": "root",
            "schemaVersion": ROOT_V02,
            "principals": {"a": {"keytype": "ssh"}},
            "roles": {},
            "propagations": [{"name": "remote"}]
        });
        assert!(matches!(
            parse_metadata_payload("root", &serde_json::to_vec(&payload).unwrap()),
            Err(PolicyInventoryError::UnsupportedExternalPolicyFeature("propagations"))
        ));
    }

    #[test]
    fn root_without_principal_collection_fails_closed() {
        let payload = serde_json::json!({
            "type": "root",
            "schemaVersion": ROOT_V02,
            "roles": {}
        });
        assert!(matches!(
            parse_metadata_payload("root", &serde_json::to_vec(&payload).unwrap()),
            Err(PolicyInventoryError::MissingPrincipalCollection("principals"))
        ));
    }

    #[test]
    fn malformed_schema_version_does_not_downgrade_to_v01() {
        let payload = serde_json::json!({
            "type": "root",
            "schemaVersion": 2,
            "keys": {"a": {"keytype": "ssh"}},
            "roles": {}
        });
        assert!(matches!(
            parse_metadata_payload("root", &serde_json::to_vec(&payload).unwrap()),
            Err(PolicyInventoryError::MalformedSchemaVersion)
        ));
    }

    #[test]
    fn unknown_key_type_fails_closed() {
        let payload = serde_json::json!({
            "type": "root",
            "schemaVersion": ROOT_V02,
            "principals": {"a": {"keytype": "future-crypto"}},
            "roles": {}
        });
        assert!(matches!(
            parse_metadata_payload("root", &serde_json::to_vec(&payload).unwrap()),
            Err(PolicyInventoryError::UnknownKeyType(value)) if value == "future-crypto"
        ));
    }

    #[test]
    fn tree_parser_preserves_top_level_shape() {
        let mut tree = Vec::new();
        tree.extend_from_slice(
            b"040000 tree 1111111111111111111111111111111111111111\tmetadata\0",
        );
        tree.extend_from_slice(
            b"040000 tree 2222222222222222222222222222222222222222\tgittuf-controller\0",
        );
        let parsed = parse_ls_tree(&tree, GitObjectAlgorithm::Sha1).unwrap();
        assert_eq!(parsed.len(), 2);
        assert_eq!(parsed[0].path, "gittuf-controller");
        assert_eq!(parsed[1].path, "metadata");
    }

    #[test]
    fn metadata_role_validation_rejects_nested_names() {
        assert!(matches!(
            role_from_metadata_path("nested/security.json"),
            Err(PolicyInventoryError::UnexpectedMetadataFile(_))
        ));
    }

    #[test]
    fn metadata_tree_accepts_all_json_roles() {
        let mut tree = Vec::new();
        tree.extend_from_slice(
            b"100644 blob 1111111111111111111111111111111111111111\troot.json\0",
        );
        tree.extend_from_slice(
            b"100644 blob 2222222222222222222222222222222222222222\ttargets.json\0",
        );
        tree.extend_from_slice(
            b"100644 blob 3333333333333333333333333333333333333333\tsecurity.json\0",
        );
        let parsed = parse_ls_tree(&tree, GitObjectAlgorithm::Sha1).unwrap();
        assert_eq!(parsed.len(), 3);
        assert_eq!(role_from_metadata_path(&parsed[0].path).unwrap(), "root");
        assert_eq!(role_from_metadata_path(&parsed[1].path).unwrap(), "security");
        assert_eq!(role_from_metadata_path(&parsed[2].path).unwrap(), "targets");
    }

    #[test]
    fn dsse_payload_is_decoded_structurally() {
        let payload = br#"{"type":"targets","delegations":null}"#;
        let envelope = serde_json::json!({
            "payloadType": DSSE_PAYLOAD_TYPE,
            "payload": BASE64.encode(payload),
            "signatures": []
        });
        assert_eq!(
            decode_dsse_payload(&serde_json::to_vec(&envelope).unwrap()).unwrap(),
            payload
        );
    }

    #[test]
    fn wrong_dsse_payload_type_fails_closed() {
        let envelope = serde_json::json!({
            "payloadType": "application/example",
            "payload": BASE64.encode(b"{}"),
            "signatures": []
        });
        assert!(matches!(
            decode_dsse_payload(&serde_json::to_vec(&envelope).unwrap()),
            Err(PolicyInventoryError::UnexpectedDssePayloadType(_))
        ));
    }
}
