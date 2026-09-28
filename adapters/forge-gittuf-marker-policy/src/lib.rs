// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-009H: concrete gittuf v0.16 marker-namespace policy evidence.
//!
//! This adapter inspects the exact policy commit already bound by a
//! [`forge_gittuf::GittufLocalReceipt`]. It never parses human CLI output.
//! Instead it reads the DSSE policy envelopes directly from the exact Git
//! object, decodes the metadata payloads, and mirrors gittuf v0.16's delegation
//! traversal for one exact consumption-marker ref.
//!
//! The positive result proves only:
//!
//! - the exact active policy tip equals the receipt policy tip;
//! - the receipt's external policy subject recomputes from that tip;
//! - one or more gittuf delegation rules protect the exact marker ref;
//! - every applicable delegation path is recorded with its principals and
//!   threshold;
//! - at least one `block-force-pushes` global rule covers the exact marker ref.
//!
//! gittuf's `block-force-pushes` rule permits descendant/fast-forward updates,
//! so this tranche deliberately does **not** claim create-only marker
//! immutability. A later constrained-executor theorem must account for every
//! principal exposed here and prove that those principals cannot issue a
//! general marker update.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use base64::{engine::general_purpose::STANDARD as BASE64_STANDARD, Engine as _};
use forge_git_ref_transaction::GitRefTransactionPlanV1;
use forge_gittuf::{gittuf_policy_subject_commitment, GittufLocalReceipt, POLICY_REF};
use mycelix_forge_core::{Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion};
use mycelix_forge_protected_ref_execution_policy::CONSUMPTION_MARKER_NAMESPACE_V1;
use mycelix_forge_repository::{GitObjectAlgorithm, GitObjectId, RepositoryRef};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::{
    collections::{BTreeMap, BTreeSet, VecDeque},
    path::{Path, PathBuf},
    process::{Command, Stdio},
};
use thiserror::Error;

const EVIDENCE_DOMAIN_V1: &[u8] = b"mycelix-forge/gittuf-marker-namespace-policy/v1\0";
const METADATA_DOMAIN_V1: &[u8] = b"mycelix-forge/gittuf-policy-metadata-set/v1\0";
const GITTUF_ALLOW_RULE: &str = "gittuf-allow-rule";
const ROOT_V02: &str = "https://gittuf.dev/policy/root/v0.2";
const TARGETS_V02: &str = "http://gittuf.dev/policy/rule-file/v0.2";
const GLOBAL_BLOCK_FORCE_PUSHES: &str = "block-force-pushes";
const MAX_METADATA_FILES_V1: usize = 4096;
const MAX_METADATA_FILE_BYTES_V1: usize = 8 * 1024 * 1024;

/// One exact gittuf rule that v0.16 considers an authorization path for the
/// inspected marker ref.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct GittufMarkerAuthorizationRuleV1 {
    role_file: String,
    rule_name: String,
    depth: u16,
    terminating: bool,
    protected_namespaces: Vec<String>,
    principal_ids: Vec<String>,
    threshold: u16,
}

impl GittufMarkerAuthorizationRuleV1 {
    /// Metadata role file containing this rule, without `.json`.
    pub fn role_file(&self) -> &str {
        &self.role_file
    }

    /// Exact delegation/rule name.
    pub fn rule_name(&self) -> &str {
        &self.rule_name
    }

    /// Delegation depth from top-level targets metadata.
    pub const fn depth(&self) -> u16 {
        self.depth
    }

    /// Whether gittuf treats this delegation as terminating for its rule file.
    pub const fn terminating(&self) -> bool {
        self.terminating
    }

    /// Exact protected namespace patterns carried by this rule.
    pub fn protected_namespaces(&self) -> &[String] {
        &self.protected_namespaces
    }

    /// Canonically sorted principal IDs that may satisfy this authorization
    /// path.
    pub fn principal_ids(&self) -> &[String] {
        &self.principal_ids
    }

    /// Required signatures/approvals for this authorization path.
    pub const fn threshold(&self) -> u16 {
        self.threshold
    }
}

/// Positive, non-deserializable evidence for one exact marker ref under one
/// exact active gittuf policy tip.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct GittufMarkerNamespacePolicyEvidenceV1 {
    project: ProjectIdentity,
    repository_policy_state: Digest,
    external_policy_subject: Digest,
    policy_tip: GitObjectId,
    marker_ref: RepositoryRef,
    marker_target: String,
    applicable_rules: Vec<GittufMarkerAuthorizationRuleV1>,
    authorized_principal_ids: Vec<String>,
    block_force_push_rule_names: Vec<String>,
    metadata_commitment: Digest,
    evidence_commitment: Digest,
}

impl GittufMarkerNamespacePolicyEvidenceV1 {
    /// Project whose repository policy was inspected.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact monotonic Forge repository-policy state commitment.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    /// Exact external gittuf policy subject committed by that state.
    pub fn external_policy_subject(&self) -> &Digest {
        &self.external_policy_subject
    }

    /// Exact `refs/gittuf/policy` Git object inspected.
    pub fn policy_tip(&self) -> &GitObjectId {
        &self.policy_tip
    }

    /// Exact durable consumption marker ref whose policy was evaluated.
    pub fn marker_ref(&self) -> &RepositoryRef {
        &self.marker_ref
    }

    /// Exact gittuf namespace target (`git:<ref>`) used for rule matching.
    pub fn marker_target(&self) -> &str {
        &self.marker_target
    }

    /// Every authorization path discovered using the v0.16 delegation-search
    /// semantics mirrored by this adapter.
    pub fn applicable_rules(&self) -> &[GittufMarkerAuthorizationRuleV1] {
        &self.applicable_rules
    }

    /// Canonical union of all principals appearing in any applicable rule.
    ///
    /// Because gittuf treats matching rule verifiers as alternatives, later
    /// executor confinement must account for every principal in this set.
    pub fn authorized_principal_ids(&self) -> &[String] {
        &self.authorized_principal_ids
    }

    /// Matching root global-rule names whose type is `block-force-pushes`.
    pub fn block_force_push_rule_names(&self) -> &[String] {
        &self.block_force_push_rule_names
    }

    /// Commitment to every exact DSSE metadata envelope inspected.
    pub fn metadata_commitment(&self) -> &Digest {
        &self.metadata_commitment
    }

    /// Aggregate 009H evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Raw Git command result used by the policy reader abstraction.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GitPolicyCommandResult {
    /// Exit status, or `-1` if unavailable.
    pub status: i32,
    /// Raw stdout bytes.
    pub stdout: Vec<u8>,
    /// Raw stderr bytes.
    pub stderr: Vec<u8>,
}

/// Minimal Git runner used by the exact policy inspector.
pub trait GitPolicyCommandRunner {
    /// Execute exact Git arguments inside `repository`.
    fn run(
        &self,
        repository: &Path,
        args: &[String],
    ) -> Result<GitPolicyCommandResult, GittufMarkerPolicyError>;
}

/// Process-backed Git runner with cleared ambient Git configuration.
#[derive(Clone, Debug)]
pub struct ProcessGitPolicyRunner {
    executable: PathBuf,
}

impl ProcessGitPolicyRunner {
    /// Construct a runner using an absolute Git executable path.
    pub fn new(executable: PathBuf) -> Result<Self, GittufMarkerPolicyError> {
        if !executable.is_absolute() {
            return Err(GittufMarkerPolicyError::GitExecutableNotAbsolute);
        }
        Ok(Self { executable })
    }

    /// Exact Git executable path.
    pub fn executable(&self) -> &Path {
        &self.executable
    }
}

impl GitPolicyCommandRunner for ProcessGitPolicyRunner {
    fn run(
        &self,
        repository: &Path,
        args: &[String],
    ) -> Result<GitPolicyCommandResult, GittufMarkerPolicyError> {
        let output = Command::new(&self.executable)
            .env_clear()
            .env("GIT_CONFIG_NOSYSTEM", "1")
            .env("GIT_CONFIG_GLOBAL", "/dev/null")
            .env("GIT_TERMINAL_PROMPT", "0")
            .env("GIT_NO_LAZY_FETCH", "1")
            .env("GIT_NO_REPLACE_OBJECTS", "1")
            .arg("-C")
            .arg(repository)
            .args(args)
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .output()
            .map_err(|source| GittufMarkerPolicyError::Process(source.to_string()))?;
        Ok(GitPolicyCommandResult {
            status: output.status.code().unwrap_or(-1),
            stdout: output.stdout,
            stderr: output.stderr,
        })
    }
}

/// Exact policy inspector bound to one repository directory.
#[derive(Clone, Debug)]
pub struct GittufMarkerPolicyInspector<R> {
    runner: R,
    repository: PathBuf,
}

impl<R> GittufMarkerPolicyInspector<R>
where
    R: GitPolicyCommandRunner,
{
    /// Construct a read-only inspector.
    pub fn new(runner: R, repository: PathBuf) -> Self {
        Self { runner, repository }
    }

    /// Inspect the exact policy tip from `receipt` for the exact marker in the
    /// positive FORGE-009F plan.
    pub fn inspect(
        &self,
        receipt: &GittufLocalReceipt,
        plan: &GitRefTransactionPlanV1,
    ) -> Result<GittufMarkerNamespacePolicyEvidenceV1, GittufMarkerPolicyError> {
        if !plan
            .consumption_marker_ref()
            .as_str()
            .starts_with(CONSUMPTION_MARKER_NAMESPACE_V1)
        {
            return Err(GittufMarkerPolicyError::MarkerOutsideReservedNamespace);
        }

        let snapshot = receipt.snapshot();
        let current_policy_tip = self.read_current_policy_tip(snapshot.object_format())?;
        if &current_policy_tip != snapshot.policy_tip() {
            return Err(GittufMarkerPolicyError::PolicyTipChanged);
        }

        let expected_external = gittuf_policy_subject_commitment(
            snapshot.object_format(),
            snapshot.policy_tip(),
            receipt.policy_state().policy_digest().algorithm(),
        )
        .map_err(|error| GittufMarkerPolicyError::ForgeGittuf(error.to_string()))?;
        if &expected_external != receipt.policy_state().policy_digest() {
            return Err(GittufMarkerPolicyError::ExternalPolicySubjectMismatch);
        }

        let files = self.read_metadata_files(snapshot.policy_tip())?;
        qualify_metadata_set(
            receipt.policy_state().project().clone(),
            receipt.policy_state().digest(DigestAlgorithm::Sha256)?,
            receipt.policy_state().policy_digest().clone(),
            snapshot.policy_tip().clone(),
            plan.consumption_marker_ref().clone(),
            files,
        )
    }

    fn read_current_policy_tip(
        &self,
        object_format: GitObjectAlgorithm,
    ) -> Result<GitObjectId, GittufMarkerPolicyError> {
        let output = self.run_success(&[
            "--no-replace-objects".to_owned(),
            "rev-parse".to_owned(),
            "--verify".to_owned(),
            POLICY_REF.to_owned(),
        ])?;
        let value = std::str::from_utf8(trim_ascii(&output.stdout))
            .map_err(|_| GittufMarkerPolicyError::NonUtf8GitObjectId)?;
        parse_git_object_id(value, object_format)
    }

    fn read_metadata_files(
        &self,
        policy_tip: &GitObjectId,
    ) -> Result<BTreeMap<String, Vec<u8>>, GittufMarkerPolicyError> {
        let tip = hex::encode(policy_tip.as_bytes());
        let listing = self.run_success(&[
            "--no-replace-objects".to_owned(),
            "ls-tree".to_owned(),
            "-r".to_owned(),
            "-z".to_owned(),
            "--name-only".to_owned(),
            tip.clone(),
            "--".to_owned(),
            "metadata".to_owned(),
        ])?;
        let mut paths = listing
            .stdout
            .split(|byte| *byte == 0)
            .filter(|path| !path.is_empty())
            .map(|path| {
                std::str::from_utf8(path)
                    .map(str::to_owned)
                    .map_err(|_| GittufMarkerPolicyError::NonUtf8MetadataPath)
            })
            .collect::<Result<Vec<_>, _>>()?;
        paths.retain(|path| path.starts_with("metadata/") && path.ends_with(".json"));
        paths.sort();
        paths.dedup();
        if paths.len() > MAX_METADATA_FILES_V1 {
            return Err(GittufMarkerPolicyError::TooManyMetadataFiles(paths.len()));
        }

        let mut files = BTreeMap::new();
        for path in paths {
            let spec = format!("{tip}:{path}");
            let output = self.run_success(&[
                "--no-replace-objects".to_owned(),
                "cat-file".to_owned(),
                "blob".to_owned(),
                spec,
            ])?;
            if output.stdout.len() > MAX_METADATA_FILE_BYTES_V1 {
                return Err(GittufMarkerPolicyError::MetadataFileTooLarge {
                    path,
                    len: output.stdout.len(),
                });
            }
            files.insert(path, output.stdout);
        }
        Ok(files)
    }

    fn run_success(
        &self,
        args: &[String],
    ) -> Result<GitPolicyCommandResult, GittufMarkerPolicyError> {
        let output = self.runner.run(&self.repository, args)?;
        if output.status != 0 {
            return Err(GittufMarkerPolicyError::GitCommandFailed {
                args: args.to_vec(),
                status: output.status,
            });
        }
        Ok(output)
    }
}

#[derive(Clone, Debug, Deserialize)]
struct DsseEnvelopeWire {
    #[serde(rename = "payloadType")]
    _payload_type: String,
    payload: String,
}

#[derive(Clone, Debug, Deserialize)]
struct RootPayloadWire {
    #[serde(rename = "type")]
    metadata_type: String,
    #[serde(rename = "schemaVersion")]
    schema_version: Option<String>,
    #[serde(rename = "globalRules", default)]
    global_rules: Vec<GlobalRuleWire>,
    #[serde(rename = "multiRepository")]
    multi_repository: Option<Value>,
}

#[derive(Clone, Debug, Deserialize)]
struct GlobalRuleWire {
    name: String,
    #[serde(rename = "type")]
    rule_type: String,
    #[serde(default)]
    paths: Vec<String>,
}

#[derive(Clone, Debug, Deserialize)]
struct TargetsPayloadWire {
    #[serde(rename = "type")]
    metadata_type: String,
    #[serde(rename = "schemaVersion")]
    schema_version: Option<String>,
    delegations: Option<DelegationsWire>,
}

#[derive(Clone, Debug, Deserialize)]
struct DelegationsWire {
    #[serde(default)]
    roles: Vec<RuleWire>,
}

#[derive(Clone, Debug, Deserialize)]
struct RuleWire {
    name: String,
    #[serde(default)]
    paths: Vec<String>,
    #[serde(default)]
    terminating: bool,
    #[serde(rename = "principalIDs", alias = "keyids", default)]
    principal_ids: Vec<String>,
    threshold: i64,
}

fn qualify_metadata_set(
    project: ProjectIdentity,
    repository_policy_state: Digest,
    external_policy_subject: Digest,
    policy_tip: GitObjectId,
    marker_ref: RepositoryRef,
    files: BTreeMap<String, Vec<u8>>,
) -> Result<GittufMarkerNamespacePolicyEvidenceV1, GittufMarkerPolicyError> {
    let root_bytes = files
        .get("metadata/root.json")
        .ok_or(GittufMarkerPolicyError::MissingRootMetadata)?;
    let targets_bytes = files
        .get("metadata/targets.json")
        .ok_or(GittufMarkerPolicyError::MissingTargetsMetadata)?;

    let root_payload = decode_dsse_payload::<RootPayloadWire>(root_bytes)?;
    validate_root_payload(&root_payload)?;
    if root_payload.multi_repository.is_some() {
        // Controller-propagated policy is intentionally a later theorem. This
        // avoids silently missing additional controller rules.
        return Err(GittufMarkerPolicyError::MultiRepositoryPolicyUnsupported);
    }

    let top_targets = decode_dsse_payload::<TargetsPayloadWire>(targets_bytes)?;
    validate_targets_payload(&top_targets)?;

    let mut roles = BTreeMap::<String, Vec<RuleWire>>::new();
    roles.insert(
        "targets".to_owned(),
        top_targets
            .delegations
            .map(|delegations| delegations.roles)
            .unwrap_or_default(),
    );

    for (path, bytes) in &files {
        if path == "metadata/root.json" || path == "metadata/targets.json" {
            continue;
        }
        let Some(role_name) = path
            .strip_prefix("metadata/")
            .and_then(|value| value.strip_suffix(".json"))
        else {
            continue;
        };
        let payload = decode_dsse_payload::<TargetsPayloadWire>(bytes)?;
        validate_targets_payload(&payload)?;
        if roles
            .insert(
                role_name.to_owned(),
                payload
                    .delegations
                    .map(|delegations| delegations.roles)
                    .unwrap_or_default(),
            )
            .is_some()
        {
            return Err(GittufMarkerPolicyError::DuplicateRoleFile(
                role_name.to_owned(),
            ));
        }
    }

    let marker_target = format!("git:{}", marker_ref.as_str());
    let applicable_rules = find_applicable_rules(&roles, &marker_target)?;
    if applicable_rules.is_empty() {
        return Err(GittufMarkerPolicyError::MarkerNamespaceUnprotected);
    }

    let mut authorized_principal_ids = BTreeSet::new();
    for rule in &applicable_rules {
        authorized_principal_ids.extend(rule.principal_ids.iter().cloned());
    }
    if authorized_principal_ids.is_empty() {
        return Err(GittufMarkerPolicyError::NoAuthorizedMarkerPrincipals);
    }

    let mut block_force_push_rule_names = Vec::new();
    for rule in &root_payload.global_rules {
        for path in &rule.paths {
            validate_supported_pattern(path)?;
        }
        if rule.rule_type == GLOBAL_BLOCK_FORCE_PUSHES
            && rule
                .paths
                .iter()
                .any(|pattern| simple_fnmatch(pattern, &marker_target))
        {
            block_force_push_rule_names.push(rule.name.clone());
        }
    }
    block_force_push_rule_names.sort();
    block_force_push_rule_names.dedup();
    if block_force_push_rule_names.is_empty() {
        return Err(GittufMarkerPolicyError::MissingBlockForcePushCoverage);
    }

    let metadata_commitment = metadata_commitment(&files)?;
    let algorithm = external_policy_subject.algorithm();
    let mut out = Vec::new();
    out.extend_from_slice(EVIDENCE_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, &project)?;
    push_digest(&mut out, &repository_policy_state)?;
    push_digest(&mut out, &external_policy_subject)?;
    push_git_object(&mut out, &policy_tip)?;
    push_ref(&mut out, &marker_ref)?;
    push_string(&mut out, &marker_target)?;
    push_digest(&mut out, &metadata_commitment)?;
    let rule_count = u16::try_from(applicable_rules.len())
        .map_err(|_| GittufMarkerPolicyError::CanonicalFieldTooLarge("rules"))?;
    out.extend_from_slice(&rule_count.to_be_bytes());
    for rule in &applicable_rules {
        push_string(&mut out, &rule.role_file)?;
        push_string(&mut out, &rule.rule_name)?;
        out.extend_from_slice(&rule.depth.to_be_bytes());
        out.push(u8::from(rule.terminating));
        out.extend_from_slice(&rule.threshold.to_be_bytes());
        let principal_count = u16::try_from(rule.principal_ids.len())
            .map_err(|_| GittufMarkerPolicyError::CanonicalFieldTooLarge("principals"))?;
        out.extend_from_slice(&principal_count.to_be_bytes());
        for principal in &rule.principal_ids {
            push_string(&mut out, principal)?;
        }
        let path_count = u16::try_from(rule.protected_namespaces.len())
            .map_err(|_| GittufMarkerPolicyError::CanonicalFieldTooLarge("paths"))?;
        out.extend_from_slice(&path_count.to_be_bytes());
        for path in &rule.protected_namespaces {
            push_string(&mut out, path)?;
        }
    }
    let union = authorized_principal_ids.into_iter().collect::<Vec<_>>();
    let principal_count = u16::try_from(union.len())
        .map_err(|_| GittufMarkerPolicyError::CanonicalFieldTooLarge("principal_union"))?;
    out.extend_from_slice(&principal_count.to_be_bytes());
    for principal in &union {
        push_string(&mut out, principal)?;
    }
    let global_count = u16::try_from(block_force_push_rule_names.len())
        .map_err(|_| GittufMarkerPolicyError::CanonicalFieldTooLarge("global_rules"))?;
    out.extend_from_slice(&global_count.to_be_bytes());
    for name in &block_force_push_rule_names {
        push_string(&mut out, name)?;
    }
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(GittufMarkerNamespacePolicyEvidenceV1 {
        project,
        repository_policy_state,
        external_policy_subject,
        policy_tip,
        marker_ref,
        marker_target,
        applicable_rules,
        authorized_principal_ids: union,
        block_force_push_rule_names,
        metadata_commitment,
        evidence_commitment,
    })
}

fn find_applicable_rules(
    role_files: &BTreeMap<String, Vec<RuleWire>>,
    target: &str,
) -> Result<Vec<GittufMarkerAuthorizationRuleV1>, GittufMarkerPolicyError> {
    let top = role_files
        .get("targets")
        .ok_or(GittufMarkerPolicyError::MissingTargetsMetadata)?;
    validate_rule_group(top, "targets")?;

    let mut groups = VecDeque::from([("targets".to_owned(), 0_u16, top.clone())]);
    let mut seen_roles = BTreeSet::from(["targets".to_owned()]);
    let mut applicable = Vec::new();

    while let Some((role_file, depth, rules)) = groups.pop_front() {
        validate_rule_group(&rules, &role_file)?;
        for rule in rules.iter().take(rules.len().saturating_sub(1)) {
            for pattern in &rule.paths {
                validate_supported_pattern(pattern)?;
            }
            if !rule
                .paths
                .iter()
                .any(|pattern| simple_fnmatch(pattern, target))
            {
                continue;
            }

            let threshold = validate_threshold(rule)?;
            let mut principals = rule.principal_ids.clone();
            principals.sort();
            principals.dedup();
            if principals.len() < usize::from(threshold) {
                return Err(GittufMarkerPolicyError::RuleCannotMeetThreshold {
                    rule: rule.name.clone(),
                });
            }

            applicable.push(GittufMarkerAuthorizationRuleV1 {
                role_file: role_file.clone(),
                rule_name: rule.name.clone(),
                depth,
                terminating: rule.terminating,
                protected_namespaces: rule.paths.clone(),
                principal_ids: principals,
                threshold,
            });

            if seen_roles.contains(&rule.name) {
                continue;
            }
            if let Some(delegated) = role_files.get(&rule.name) {
                validate_rule_group(delegated, &rule.name)?;
                seen_roles.insert(rule.name.clone());
                let next_depth = depth
                    .checked_add(1)
                    .ok_or(GittufMarkerPolicyError::DelegationDepthOverflow)?;
                groups.push_front((rule.name.clone(), next_depth, delegated.clone()));
                if rule.terminating {
                    break;
                }
            }
        }
    }

    Ok(applicable)
}

fn validate_rule_group(rules: &[RuleWire], role: &str) -> Result<(), GittufMarkerPolicyError> {
    if rules.is_empty() {
        return Ok(());
    }
    if rules.last().map(|rule| rule.name.as_str()) != Some(GITTUF_ALLOW_RULE) {
        return Err(GittufMarkerPolicyError::MissingTerminalAllowRule(
            role.to_owned(),
        ));
    }
    Ok(())
}

fn validate_threshold(rule: &RuleWire) -> Result<u16, GittufMarkerPolicyError> {
    if rule.threshold <= 0 {
        return Err(GittufMarkerPolicyError::InvalidRuleThreshold {
            rule: rule.name.clone(),
            threshold: rule.threshold,
        });
    }
    u16::try_from(rule.threshold).map_err(|_| GittufMarkerPolicyError::InvalidRuleThreshold {
        rule: rule.name.clone(),
        threshold: rule.threshold,
    })
}

fn validate_root_payload(payload: &RootPayloadWire) -> Result<(), GittufMarkerPolicyError> {
    if payload.metadata_type != "root" {
        return Err(GittufMarkerPolicyError::UnexpectedMetadataType(
            payload.metadata_type.clone(),
        ));
    }
    if let Some(version) = &payload.schema_version {
        if version != ROOT_V02 {
            return Err(GittufMarkerPolicyError::UnsupportedRootSchema(
                version.clone(),
            ));
        }
    }
    Ok(())
}

fn validate_targets_payload(payload: &TargetsPayloadWire) -> Result<(), GittufMarkerPolicyError> {
    if payload.metadata_type != "targets" {
        return Err(GittufMarkerPolicyError::UnexpectedMetadataType(
            payload.metadata_type.clone(),
        ));
    }
    if let Some(version) = &payload.schema_version {
        if version != TARGETS_V02 {
            return Err(GittufMarkerPolicyError::UnsupportedTargetsSchema(
                version.clone(),
            ));
        }
    }
    Ok(())
}

fn decode_dsse_payload<T: for<'de> Deserialize<'de>>(
    envelope_bytes: &[u8],
) -> Result<T, GittufMarkerPolicyError> {
    let envelope: DsseEnvelopeWire = serde_json::from_slice(envelope_bytes)?;
    let payload = BASE64_STANDARD
        .decode(envelope.payload.as_bytes())
        .map_err(|_| GittufMarkerPolicyError::InvalidDssePayloadEncoding)?;
    Ok(serde_json::from_slice(&payload)?)
}

fn metadata_commitment(
    files: &BTreeMap<String, Vec<u8>>,
) -> Result<Digest, GittufMarkerPolicyError> {
    let mut out = Vec::new();
    out.extend_from_slice(METADATA_DOMAIN_V1);
    let count = u16::try_from(files.len())
        .map_err(|_| GittufMarkerPolicyError::CanonicalFieldTooLarge("metadata_files"))?;
    out.extend_from_slice(&count.to_be_bytes());
    for (path, bytes) in files {
        push_string(&mut out, path)?;
        let digest = Digest::of_bytes(DigestAlgorithm::Sha256, bytes);
        push_digest(&mut out, &digest)?;
    }
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &out))
}

// Strict subset of fnmatch sufficient for ordinary gittuf Git-ref rules. Any
// bracket expression or escape fails qualification rather than risking a Rust
// matcher disagreeing with gittuf v0.16's fnmatch implementation.
fn validate_supported_pattern(pattern: &str) -> Result<(), GittufMarkerPolicyError> {
    if pattern.contains('[') || pattern.contains(']') || pattern.contains('\\') {
        return Err(GittufMarkerPolicyError::UnsupportedRulePattern(
            pattern.to_owned(),
        ));
    }
    Ok(())
}

fn simple_fnmatch(pattern: &str, value: &str) -> bool {
    let pattern = pattern.as_bytes();
    let value = value.as_bytes();
    let mut dp = vec![false; value.len() + 1];
    dp[0] = true;
    for token in pattern {
        let mut next = vec![false; value.len() + 1];
        match token {
            b'*' => {
                next[0] = dp[0];
                for index in 1..=value.len() {
                    next[index] = dp[index] || next[index - 1];
                }
            }
            b'?' => {
                for index in 1..=value.len() {
                    next[index] = dp[index - 1];
                }
            }
            literal => {
                for index in 1..=value.len() {
                    next[index] = dp[index - 1] && value[index - 1] == *literal;
                }
            }
        }
        dp = next;
    }
    dp[value.len()]
}

fn parse_git_object_id(
    value: &str,
    algorithm: GitObjectAlgorithm,
) -> Result<GitObjectId, GittufMarkerPolicyError> {
    let bytes = hex::decode(value)
        .map_err(|_| GittufMarkerPolicyError::InvalidGitObjectId(value.to_owned()))?;
    GitObjectId::new(algorithm, bytes)
        .map_err(|error| GittufMarkerPolicyError::Repository(error.to_string()))
}

fn trim_ascii(bytes: &[u8]) -> &[u8] {
    let mut start = 0;
    let mut end = bytes.len();
    while start < end && bytes[start].is_ascii_whitespace() {
        start += 1;
    }
    while end > start && bytes[end - 1].is_ascii_whitespace() {
        end -= 1;
    }
    &bytes[start..end]
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), GittufMarkerPolicyError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_ref(out: &mut Vec<u8>, reference: &RepositoryRef) -> Result<(), GittufMarkerPolicyError> {
    push_string(out, reference.as_str())
}

fn push_git_object(
    out: &mut Vec<u8>,
    object: &GitObjectId,
) -> Result<(), GittufMarkerPolicyError> {
    out.push(object.algorithm().code());
    let len = u16::try_from(object.as_bytes().len())
        .map_err(|_| GittufMarkerPolicyError::CanonicalFieldTooLarge("git_object"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(object.as_bytes());
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), GittufMarkerPolicyError> {
    push_string(out, digest.algorithm().id())?;
    let len = u32::try_from(digest.as_bytes().len())
        .map_err(|_| GittufMarkerPolicyError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

fn push_string(out: &mut Vec<u8>, value: &str) -> Result<(), GittufMarkerPolicyError> {
    let bytes = value.as_bytes();
    let len = u32::try_from(bytes.len())
        .map_err(|_| GittufMarkerPolicyError::CanonicalFieldTooLarge("string"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Concrete 009H inspection failures.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum GittufMarkerPolicyError {
    /// Git executable path must be absolute.
    #[error("Git executable path must be absolute")]
    GitExecutableNotAbsolute,
    /// Process launch failed.
    #[error("Git policy inspection process failed: {0}")]
    Process(String),
    /// Git command exited unsuccessfully.
    #[error("Git policy inspection command failed with status {status}: {args:?}")]
    GitCommandFailed { args: Vec<String>, status: i32 },
    /// Current active policy ref changed from the receipt's verified snapshot.
    #[error("active gittuf policy ref changed after repository verification")]
    PolicyTipChanged,
    /// Exact policy tip no longer maps to the repository-policy external subject.
    #[error("gittuf policy tip does not match repository-policy external subject")]
    ExternalPolicySubjectMismatch,
    /// Marker ref is not in the reserved durable-consumption namespace.
    #[error("consumption marker is outside the reserved namespace")]
    MarkerOutsideReservedNamespace,
    /// Policy tree lacks root metadata.
    #[error("gittuf policy is missing metadata/root.json")]
    MissingRootMetadata,
    /// Policy tree lacks top-level targets metadata.
    #[error("gittuf policy is missing metadata/targets.json")]
    MissingTargetsMetadata,
    /// Metadata type is not root/targets as expected.
    #[error("unexpected gittuf metadata type: {0}")]
    UnexpectedMetadataType(String),
    /// Unsupported root metadata schema.
    #[error("unsupported gittuf root metadata schema: {0}")]
    UnsupportedRootSchema(String),
    /// Unsupported targets metadata schema.
    #[error("unsupported gittuf targets metadata schema: {0}")]
    UnsupportedTargetsSchema(String),
    /// Controller/multi-repository policy requires a separate proof because
    /// controller metadata may add policy paths not inspected here.
    #[error("multi-repository gittuf policy is not supported by marker-policy v1")]
    MultiRepositoryPolicyUnsupported,
    /// Too many metadata files for the bounded parser.
    #[error("too many gittuf metadata files: {0}")]
    TooManyMetadataFiles(usize),
    /// One metadata file exceeded the bounded parser size.
    #[error("gittuf metadata file {path} is too large: {len} bytes")]
    MetadataFileTooLarge { path: String, len: usize },
    /// Duplicate metadata role file.
    #[error("duplicate gittuf role metadata file: {0}")]
    DuplicateRoleFile(String),
    /// Rule group is missing the final special allow rule expected by v0.16.
    #[error("gittuf role file {0} is missing terminal allow rule")]
    MissingTerminalAllowRule(String),
    /// Rule threshold is invalid.
    #[error("gittuf rule {rule} has invalid threshold {threshold}")]
    InvalidRuleThreshold { rule: String, threshold: i64 },
    /// Rule has fewer distinct principals than required threshold.
    #[error("gittuf rule cannot meet threshold: {rule}")]
    RuleCannotMeetThreshold { rule: String },
    /// No delegation rule protects this exact marker ref.
    #[error("exact consumption marker ref is not protected by a gittuf delegation rule")]
    MarkerNamespaceUnprotected,
    /// Applicable rules contain no authorized principal.
    #[error("gittuf marker rules contain no authorized principals")]
    NoAuthorizedMarkerPrincipals,
    /// No matching block-force-push global rule exists.
    #[error("exact consumption marker ref lacks gittuf block-force-push coverage")]
    MissingBlockForcePushCoverage,
    /// Pattern uses syntax outside the intentionally mirrored fnmatch subset.
    #[error("unsupported gittuf rule pattern for exact Rust verifier: {0}")]
    UnsupportedRulePattern(String),
    /// Delegation depth overflowed v1 encoding.
    #[error("gittuf delegation depth exceeds v1 bounds")]
    DelegationDepthOverflow,
    /// DSSE payload is not valid base64.
    #[error("invalid DSSE payload encoding")]
    InvalidDssePayloadEncoding,
    /// Metadata path is not UTF-8.
    #[error("gittuf metadata path is not UTF-8")]
    NonUtf8MetadataPath,
    /// Git object ID output is not UTF-8.
    #[error("Git object ID output is not UTF-8")]
    NonUtf8GitObjectId,
    /// Invalid object ID hex.
    #[error("invalid Git object ID: {0}")]
    InvalidGitObjectId(String),
    /// Forge gittuf adapter failure.
    #[error("Forge gittuf adapter failure: {0}")]
    ForgeGittuf(String),
    /// Repository contract failure.
    #[error("repository contract failure: {0}")]
    Repository(String),
    /// JSON/DSSE metadata decode failure.
    #[error("gittuf metadata JSON failure: {0}")]
    Json(String),
    /// Canonical field overflow.
    #[error("canonical field too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
}

impl From<serde_json::Error> for GittufMarkerPolicyError {
    fn from(error: serde_json::Error) -> Self {
        Self::Json(error.to_string())
    }
}

impl From<mycelix_forge_repository::RepositoryVerificationError> for GittufMarkerPolicyError {
    fn from(error: mycelix_forge_repository::RepositoryVerificationError) -> Self {
        Self::Repository(error.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn project() -> ProjectIdentity {
        ProjectIdentity::derive(
            &ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], digest(0x12)),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn git_id(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn envelope(payload: Value) -> Vec<u8> {
        let payload = serde_json::to_vec(&payload).unwrap();
        serde_json::to_vec(&serde_json::json!({
            "payloadType": "application/vnd.gittuf+json",
            "payload": BASE64_STANDARD.encode(payload),
            "signatures": []
        }))
        .unwrap()
    }

    fn marker_ref() -> RepositoryRef {
        RepositoryRef::new(
            "refs/mycelix/forge/consumed/sha256/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
        )
        .unwrap()
    }

    fn base_files(extra_rules: Vec<Value>) -> BTreeMap<String, Vec<u8>> {
        let marker_pattern = "git:refs/mycelix/forge/consumed/*";
        let mut roles = vec![serde_json::json!({
            "name": "forge-marker-executor",
            "paths": [marker_pattern],
            "terminating": false,
            "principalIDs": ["executor-A"],
            "threshold": 1
        })];
        roles.extend(extra_rules);
        roles.push(serde_json::json!({
            "name": GITTUF_ALLOW_RULE,
            "paths": ["*"],
            "terminating": true,
            "principalIDs": [],
            "threshold": 1
        }));

        BTreeMap::from([
            (
                "metadata/root.json".to_owned(),
                envelope(serde_json::json!({
                    "type": "root",
                    "schemaVersion": ROOT_V02,
                    "globalRules": [{
                        "name": "no-marker-force-push",
                        "type": GLOBAL_BLOCK_FORCE_PUSHES,
                        "paths": [marker_pattern]
                    }]
                })),
            ),
            (
                "metadata/targets.json".to_owned(),
                envelope(serde_json::json!({
                    "type": "targets",
                    "schemaVersion": TARGETS_V02,
                    "delegations": { "roles": roles }
                })),
            ),
        ])
    }

    #[test]
    fn exact_marker_policy_exposes_all_authorization_paths() {
        let second = serde_json::json!({
            "name": "broader-forge-rule",
            "paths": ["git:refs/mycelix/forge/*"],
            "terminating": false,
            "principalIDs": ["executor-B", "executor-C"],
            "threshold": 2
        });
        let result = qualify_metadata_set(
            project(),
            digest(0x30),
            digest(0x31),
            git_id(0x32),
            marker_ref(),
            base_files(vec![second]),
        )
        .unwrap();
        assert_eq!(result.applicable_rules().len(), 2);
        assert_eq!(
            result.authorized_principal_ids(),
            &["executor-A".to_owned(), "executor-B".to_owned(), "executor-C".to_owned()]
        );
        assert_eq!(result.block_force_push_rule_names(), &["no-marker-force-push"]);
    }

    #[test]
    fn missing_force_push_rule_fails_closed() {
        let mut files = base_files(vec![]);
        files.insert(
            "metadata/root.json".to_owned(),
            envelope(serde_json::json!({
                "type": "root",
                "schemaVersion": ROOT_V02,
                "globalRules": []
            })),
        );
        assert_eq!(
            qualify_metadata_set(
                project(),
                digest(0x30),
                digest(0x31),
                git_id(0x32),
                marker_ref(),
                files,
            )
            .unwrap_err(),
            GittufMarkerPolicyError::MissingBlockForcePushCoverage
        );
    }

    #[test]
    fn delegated_matching_rule_is_included() {
        let mut files = base_files(vec![serde_json::json!({
            "name": "delegated-marker-role",
            "paths": ["git:refs/mycelix/forge/consumed/*"],
            "terminating": true,
            "principalIDs": ["parent-executor"],
            "threshold": 1
        })]);
        files.insert(
            "metadata/delegated-marker-role.json".to_owned(),
            envelope(serde_json::json!({
                "type": "targets",
                "schemaVersion": TARGETS_V02,
                "delegations": { "roles": [
                    {
                        "name": "child-marker-rule",
                        "paths": ["git:refs/mycelix/forge/consumed/*"],
                        "terminating": false,
                        "principalIDs": ["child-executor"],
                        "threshold": 1
                    },
                    {
                        "name": GITTUF_ALLOW_RULE,
                        "paths": ["*"],
                        "terminating": true,
                        "principalIDs": [],
                        "threshold": 1
                    }
                ] }
            })),
        );
        let result = qualify_metadata_set(
            project(),
            digest(0x30),
            digest(0x31),
            git_id(0x32),
            marker_ref(),
            files,
        )
        .unwrap();
        assert!(result
            .applicable_rules()
            .iter()
            .any(|rule| rule.rule_name() == "child-marker-rule" && rule.depth() == 1));
    }

    #[test]
    fn complex_fnmatch_syntax_fails_closed() {
        let files = base_files(vec![serde_json::json!({
            "name": "complex",
            "paths": ["git:refs/mycelix/forge/consumed/[ab]*"],
            "terminating": false,
            "principalIDs": ["executor-X"],
            "threshold": 1
        })]);
        assert!(matches!(
            qualify_metadata_set(
                project(),
                digest(0x30),
                digest(0x31),
                git_id(0x32),
                marker_ref(),
                files,
            ),
            Err(GittufMarkerPolicyError::UnsupportedRulePattern(_))
        ));
    }

    #[test]
    fn v01_keyids_are_accepted_without_reinterpreting_principal_identity() {
        let marker_pattern = "git:refs/mycelix/forge/consumed/*";
        let files = BTreeMap::from([
            (
                "metadata/root.json".to_owned(),
                envelope(serde_json::json!({
                    "type": "root",
                    "globalRules": [{
                        "name": "no-marker-force-push",
                        "type": GLOBAL_BLOCK_FORCE_PUSHES,
                        "paths": [marker_pattern]
                    }]
                })),
            ),
            (
                "metadata/targets.json".to_owned(),
                envelope(serde_json::json!({
                    "type": "targets",
                    "delegations": { "roles": [
                        {
                            "name": "legacy-marker-rule",
                            "paths": [marker_pattern],
                            "terminating": false,
                            "keyids": ["legacy-key"],
                            "threshold": 1
                        },
                        {
                            "name": GITTUF_ALLOW_RULE,
                            "paths": ["*"],
                            "terminating": true,
                            "keyids": [],
                            "threshold": 1
                        }
                    ] }
                })),
            ),
        ]);
        let result = qualify_metadata_set(
            project(),
            digest(0x30),
            digest(0x31),
            git_id(0x32),
            marker_ref(),
            files,
        )
        .unwrap();
        assert_eq!(result.authorized_principal_ids(), &["legacy-key".to_owned()]);
    }
}
