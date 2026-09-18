// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Provider-facing evidence contract for observed Linux isolation.
//!
//! FORGE-004D2B1 deliberately contains no `/proc`, socket, process-launch, or
//! bubblewrap I/O. It defines the observations a concrete Linux probe must
//! produce and the exact conditions under which two independent observation
//! channels can qualify one isolation run.
//!
//! The resulting [`QualifiedIsolationEvidence`] is still not repository
//! `OfflineEvidence`: FORGE-004D2B2 must produce these observations with a
//! qualified launcher/probe, and FORGE-004D2C must close verifier trust.

use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_linux_isolation::{
    ArtifactMount, IsolationPolicyError, LinuxIsolationPolicyV1, NixClosureManifest,
    VerifierInvocation, SANDBOX_HOME, SANDBOX_HOSTNAME, SANDBOX_TMP, SANDBOX_WORKDIR,
};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use std::collections::BTreeSet;
use thiserror::Error;

const INSIDE_DOMAIN_V1: &[u8] = b"mycelix-forge/linux-isolation-inside-evidence/v1\0";
const PARENT_DOMAIN_V1: &[u8] = b"mycelix-forge/linux-isolation-parent-evidence/v1\0";
const QUALIFIED_DOMAIN_V1: &[u8] = b"mycelix-forge/linux-isolation-qualified/v1\0";
const MAX_TEXT: usize = 4096;
const MAX_ITEMS: usize = 8192;

const EXPECTED_ENVIRONMENT: &[(&str, &str)] = &[
    ("GIT_CONFIG_GLOBAL", "/dev/null"),
    ("GIT_CONFIG_NOSYSTEM", "1"),
    ("GIT_NO_LAZY_FETCH", "1"),
    ("GIT_TERMINAL_PROMPT", "0"),
    ("GITTUF_DEBUG", "0"),
    ("GITTUF_DEV", "0"),
    ("HOME", SANDBOX_HOME),
    ("LANG", "C"),
    ("LC_ALL", "C"),
    ("PWD", SANDBOX_WORKDIR),
    ("TMPDIR", SANDBOX_TMP),
];

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct NamespaceSnapshot {
    pub user: String,
    pub mount: String,
    pub pid: String,
    pub ipc: String,
    pub net: String,
    pub uts: String,
}

impl NamespaceSnapshot {
    fn validate(&self) -> Result<(), IsolationEvidenceError> {
        for value in [
            &self.user,
            &self.mount,
            &self.pid,
            &self.ipc,
            &self.net,
            &self.uts,
        ] {
            validate_text("namespace identity", value)?;
        }
        Ok(())
    }

    fn all_differ_from(&self, other: &Self) -> bool {
        self.user != other.user
            && self.mount != other.mount
            && self.pid != other.pid
            && self.ipc != other.ipc
            && self.net != other.net
            && self.uts != other.uts
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CapabilitySnapshot {
    pub inheritable: u64,
    pub permitted: u64,
    pub effective: u64,
    pub bounding: u64,
    pub ambient: u64,
}

impl CapabilitySnapshot {
    pub const fn all_zero(self) -> bool {
        self.inheritable == 0
            && self.permitted == 0
            && self.effective == 0
            && self.bounding == 0
            && self.ambient == 0
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct EnvironmentEntry {
    key: String,
    value: String,
}

impl EnvironmentEntry {
    pub fn new(
        key: impl Into<String>,
        value: impl Into<String>,
    ) -> Result<Self, IsolationEvidenceError> {
        let key = key.into();
        let value = value.into();
        validate_text("environment key", &key)?;
        if key.contains('=') {
            return Err(IsolationEvidenceError::InvalidEnvironmentKey(key));
        }
        validate_text("environment value", &value)?;
        Ok(Self { key, value })
    }

    pub fn key(&self) -> &str {
        &self.key
    }

    pub fn value(&self) -> &str {
        &self.value
    }
}

impl<'de> Deserialize<'de> for EnvironmentEntry {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            key: String,
            value: String,
        }
        let wire = Wire::deserialize(deserializer)?;
        Self::new(wire.key, wire.value).map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct ArtifactObservation {
    role: String,
    destination: String,
    digest: Digest,
    size: u64,
    regular_file: bool,
    writable: bool,
}

impl ArtifactObservation {
    pub fn new(
        role: impl Into<String>,
        destination: impl Into<String>,
        digest: Digest,
        size: u64,
        regular_file: bool,
        writable: bool,
    ) -> Result<Self, IsolationEvidenceError> {
        let role = role.into();
        let destination = destination.into();
        validate_text("artifact role", &role)?;
        validate_mount_destination(&destination)?;
        if size == 0 {
            return Err(IsolationEvidenceError::EmptyArtifact(role));
        }
        Ok(Self {
            role,
            destination,
            digest,
            size,
            regular_file,
            writable,
        })
    }

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

    pub const fn regular_file(&self) -> bool {
        self.regular_file
    }

    pub const fn writable(&self) -> bool {
        self.writable
    }
}

impl<'de> Deserialize<'de> for ArtifactObservation {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            role: String,
            destination: String,
            digest: Digest,
            size: u64,
            regular_file: bool,
            writable: bool,
        }
        let wire = Wire::deserialize(deserializer)?;
        Self::new(
            wire.role,
            wire.destination,
            wire.digest,
            wire.size,
            wire.regular_file,
            wire.writable,
        )
        .map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct InsideIsolationEvidence {
    policy_digest: Digest,
    namespaces: NamespaceSnapshot,
    capabilities: CapabilitySnapshot,
    hostname: String,
    environment: Vec<EnvironmentEntry>,
    visible_store_roots: Vec<String>,
    artifacts: Vec<ArtifactObservation>,
    workdir_writable: bool,
    tmp_writable: bool,
    home_entries: Vec<String>,
    sys_present: bool,
    run_present: bool,
    mountinfo_digest: Digest,
    route_table_digest: Digest,
    non_loopback_route_present: bool,
    outbound_connect_succeeded: bool,
    nested_userns_created: bool,
}

impl InsideIsolationEvidence {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        policy_digest: Digest,
        namespaces: NamespaceSnapshot,
        capabilities: CapabilitySnapshot,
        hostname: impl Into<String>,
        mut environment: Vec<EnvironmentEntry>,
        mut visible_store_roots: Vec<String>,
        mut artifacts: Vec<ArtifactObservation>,
        workdir_writable: bool,
        tmp_writable: bool,
        mut home_entries: Vec<String>,
        sys_present: bool,
        run_present: bool,
        mountinfo_digest: Digest,
        route_table_digest: Digest,
        non_loopback_route_present: bool,
        outbound_connect_succeeded: bool,
        nested_userns_created: bool,
    ) -> Result<Self, IsolationEvidenceError> {
        namespaces.validate()?;
        let hostname = hostname.into();
        validate_text("hostname", &hostname)?;
        canonicalize_environment(&mut environment)?;
        canonicalize_strings(&mut visible_store_roots, "store roots")?;
        canonicalize_artifacts(&mut artifacts)?;
        canonicalize_strings(&mut home_entries, "home entries")?;
        Ok(Self {
            policy_digest,
            namespaces,
            capabilities,
            hostname,
            environment,
            visible_store_roots,
            artifacts,
            workdir_writable,
            tmp_writable,
            home_entries,
            sys_present,
            run_present,
            mountinfo_digest,
            route_table_digest,
            non_loopback_route_present,
            outbound_connect_succeeded,
            nested_userns_created,
        })
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, IsolationEvidenceError> {
        let mut out = Vec::new();
        out.extend_from_slice(INSIDE_DOMAIN_V1);
        push_digest(&mut out, &self.policy_digest)?;
        push_namespaces(&mut out, &self.namespaces)?;
        for capability in [
            self.capabilities.inheritable,
            self.capabilities.permitted,
            self.capabilities.effective,
            self.capabilities.bounding,
            self.capabilities.ambient,
        ] {
            out.extend_from_slice(&capability.to_be_bytes());
        }
        push_string(&mut out, &self.hostname, "hostname")?;
        push_count(&mut out, self.environment.len(), "environment")?;
        for entry in &self.environment {
            push_string(&mut out, entry.key(), "environment key")?;
            push_string(&mut out, entry.value(), "environment value")?;
        }
        push_strings(&mut out, &self.visible_store_roots, "store roots")?;
        push_count(&mut out, self.artifacts.len(), "artifacts")?;
        for artifact in &self.artifacts {
            push_string(&mut out, artifact.role(), "artifact role")?;
            push_string(&mut out, artifact.destination(), "artifact destination")?;
            push_digest(&mut out, artifact.digest())?;
            out.extend_from_slice(&artifact.size().to_be_bytes());
            out.push(u8::from(artifact.regular_file()));
            out.push(u8::from(artifact.writable()));
        }
        for value in [
            self.workdir_writable,
            self.tmp_writable,
            self.sys_present,
            self.run_present,
            self.non_loopback_route_present,
            self.outbound_connect_succeeded,
            self.nested_userns_created,
        ] {
            out.push(u8::from(value));
        }
        push_strings(&mut out, &self.home_entries, "home entries")?;
        push_digest(&mut out, &self.mountinfo_digest)?;
        push_digest(&mut out, &self.route_table_digest)?;
        Ok(out)
    }

    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, IsolationEvidenceError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for InsideIsolationEvidence {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            policy_digest: Digest,
            namespaces: NamespaceSnapshot,
            capabilities: CapabilitySnapshot,
            hostname: String,
            environment: Vec<EnvironmentEntry>,
            visible_store_roots: Vec<String>,
            artifacts: Vec<ArtifactObservation>,
            workdir_writable: bool,
            tmp_writable: bool,
            home_entries: Vec<String>,
            sys_present: bool,
            run_present: bool,
            mountinfo_digest: Digest,
            route_table_digest: Digest,
            non_loopback_route_present: bool,
            outbound_connect_succeeded: bool,
            nested_userns_created: bool,
        }
        let wire = Wire::deserialize(deserializer)?;
        Self::new(
            wire.policy_digest,
            wire.namespaces,
            wire.capabilities,
            wire.hostname,
            wire.environment,
            wire.visible_store_roots,
            wire.artifacts,
            wire.workdir_writable,
            wire.tmp_writable,
            wire.home_entries,
            wire.sys_present,
            wire.run_present,
            wire.mountinfo_digest,
            wire.route_table_digest,
            wire.non_loopback_route_present,
            wire.outbound_connect_succeeded,
            wire.nested_userns_created,
        )
        .map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ParentIsolationEvidence {
    host_namespaces: NamespaceSnapshot,
    child_namespaces: NamespaceSnapshot,
    child_mountinfo_digest: Digest,
    bubblewrap_status_commitment: Digest,
    child_exit_code: i32,
}

impl ParentIsolationEvidence {
    pub fn new(
        host_namespaces: NamespaceSnapshot,
        child_namespaces: NamespaceSnapshot,
        child_mountinfo_digest: Digest,
        bubblewrap_status_commitment: Digest,
        child_exit_code: i32,
    ) -> Result<Self, IsolationEvidenceError> {
        host_namespaces.validate()?;
        child_namespaces.validate()?;
        Ok(Self {
            host_namespaces,
            child_namespaces,
            child_mountinfo_digest,
            bubblewrap_status_commitment,
            child_exit_code,
        })
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, IsolationEvidenceError> {
        let mut out = Vec::new();
        out.extend_from_slice(PARENT_DOMAIN_V1);
        push_namespaces(&mut out, &self.host_namespaces)?;
        push_namespaces(&mut out, &self.child_namespaces)?;
        push_digest(&mut out, &self.child_mountinfo_digest)?;
        push_digest(&mut out, &self.bubblewrap_status_commitment)?;
        out.extend_from_slice(&self.child_exit_code.to_be_bytes());
        Ok(out)
    }

    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, IsolationEvidenceError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for ParentIsolationEvidence {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            host_namespaces: NamespaceSnapshot,
            child_namespaces: NamespaceSnapshot,
            child_mountinfo_digest: Digest,
            bubblewrap_status_commitment: Digest,
            child_exit_code: i32,
        }
        let wire = Wire::deserialize(deserializer)?;
        Self::new(
            wire.host_namespaces,
            wire.child_namespaces,
            wire.child_mountinfo_digest,
            wire.bubblewrap_status_commitment,
            wire.child_exit_code,
        )
        .map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualifiedIsolationEvidence {
    policy_digest: Digest,
    inside_digest: Digest,
    parent_digest: Digest,
    evidence_digest: Digest,
}

impl QualifiedIsolationEvidence {
    pub fn policy_digest(&self) -> &Digest {
        &self.policy_digest
    }

    pub fn inside_digest(&self) -> &Digest {
        &self.inside_digest
    }

    pub fn parent_digest(&self) -> &Digest {
        &self.parent_digest
    }

    pub fn evidence_digest(&self) -> &Digest {
        &self.evidence_digest
    }
}

pub fn qualify_linux_isolation(
    policy: &LinuxIsolationPolicyV1,
    closure: &NixClosureManifest,
    invocation: &VerifierInvocation,
    inside: &InsideIsolationEvidence,
    parent: &ParentIsolationEvidence,
) -> Result<QualifiedIsolationEvidence, IsolationEvidenceError> {
    policy.validate_dependencies(closure, invocation)?;
    let policy_digest = policy.digest(DigestAlgorithm::Sha256)?;
    if inside.policy_digest != policy_digest {
        return Err(IsolationEvidenceError::PolicyDigestMismatch);
    }
    if inside.namespaces != parent.child_namespaces {
        return Err(IsolationEvidenceError::InsideParentNamespaceMismatch);
    }
    if !parent.child_namespaces.all_differ_from(&parent.host_namespaces) {
        return Err(IsolationEvidenceError::NamespaceNotIsolated);
    }
    if inside.mountinfo_digest != parent.child_mountinfo_digest {
        return Err(IsolationEvidenceError::MountInfoMismatch);
    }
    if parent.child_exit_code != 0 {
        return Err(IsolationEvidenceError::SandboxChildFailed(
            parent.child_exit_code,
        ));
    }
    if !inside.capabilities.all_zero() {
        return Err(IsolationEvidenceError::CapabilitiesRemain);
    }
    if inside.hostname != SANDBOX_HOSTNAME {
        return Err(IsolationEvidenceError::UnexpectedHostname(
            inside.hostname.clone(),
        ));
    }
    if inside.environment != expected_environment() {
        return Err(IsolationEvidenceError::EnvironmentMismatch);
    }
    if inside.visible_store_roots != expected_store_roots(closure) {
        return Err(IsolationEvidenceError::StoreVisibilityMismatch);
    }
    require_artifact_observations(policy.artifact_mounts(), &inside.artifacts)?;
    if !inside.workdir_writable || !inside.tmp_writable {
        return Err(IsolationEvidenceError::EphemeralWorkspaceNotWritable);
    }
    if inside.home_entries != vec!["forge".to_owned()] {
        return Err(IsolationEvidenceError::HostHomeVisible);
    }
    if inside.sys_present || inside.run_present {
        return Err(IsolationEvidenceError::UnexpectedHostFilesystemVisible);
    }
    if inside.non_loopback_route_present || inside.outbound_connect_succeeded {
        return Err(IsolationEvidenceError::NetworkIsolationFailed);
    }
    if inside.nested_userns_created {
        return Err(IsolationEvidenceError::NestedUserNamespaceAllowed);
    }

    let inside_digest = inside.digest(DigestAlgorithm::Sha256)?;
    let parent_digest = parent.digest(DigestAlgorithm::Sha256)?;
    let mut evidence = Vec::new();
    evidence.extend_from_slice(QUALIFIED_DOMAIN_V1);
    push_digest(&mut evidence, &policy_digest)?;
    push_digest(&mut evidence, &inside_digest)?;
    push_digest(&mut evidence, &parent_digest)?;
    let evidence_digest = Digest::of_bytes(DigestAlgorithm::Sha256, &evidence);

    Ok(QualifiedIsolationEvidence {
        policy_digest,
        inside_digest,
        parent_digest,
        evidence_digest,
    })
}

pub fn expected_environment() -> Vec<EnvironmentEntry> {
    EXPECTED_ENVIRONMENT
        .iter()
        .map(|(key, value)| EnvironmentEntry::new(*key, *value).expect("fixed environment is valid"))
        .collect()
}

fn expected_store_roots(closure: &NixClosureManifest) -> Vec<String> {
    closure
        .entries()
        .iter()
        .map(|entry| {
            entry
                .store_path()
                .rsplit('/')
                .next()
                .expect("validated store root has basename")
                .to_owned()
        })
        .collect()
}

fn require_artifact_observations(
    mounts: &[ArtifactMount],
    observations: &[ArtifactObservation],
) -> Result<(), IsolationEvidenceError> {
    if mounts.len() != observations.len() {
        return Err(IsolationEvidenceError::ArtifactObservationMismatch);
    }
    for (mount, observation) in mounts.iter().zip(observations) {
        if mount.role() != observation.role()
            || mount.destination() != observation.destination()
            || mount.digest() != observation.digest()
            || mount.size() != observation.size()
            || !observation.regular_file()
            || observation.writable()
        {
            return Err(IsolationEvidenceError::ArtifactObservationMismatch);
        }
    }
    Ok(())
}

fn canonicalize_environment(
    values: &mut Vec<EnvironmentEntry>,
) -> Result<(), IsolationEvidenceError> {
    if values.len() > MAX_ITEMS {
        return Err(IsolationEvidenceError::TooManyItems("environment"));
    }
    values.sort();
    let mut keys = BTreeSet::new();
    for value in values {
        if !keys.insert(value.key.clone()) {
            return Err(IsolationEvidenceError::DuplicateEnvironmentKey(
                value.key.clone(),
            ));
        }
    }
    Ok(())
}

fn canonicalize_artifacts(
    values: &mut Vec<ArtifactObservation>,
) -> Result<(), IsolationEvidenceError> {
    if values.len() > MAX_ITEMS {
        return Err(IsolationEvidenceError::TooManyItems("artifacts"));
    }
    values.sort();
    let mut roles = BTreeSet::new();
    let mut destinations = BTreeSet::new();
    for value in values {
        if !roles.insert(value.role.clone()) {
            return Err(IsolationEvidenceError::DuplicateArtifactRole(
                value.role.clone(),
            ));
        }
        if !destinations.insert(value.destination.clone()) {
            return Err(IsolationEvidenceError::DuplicateArtifactDestination(
                value.destination.clone(),
            ));
        }
    }
    Ok(())
}

fn canonicalize_strings(
    values: &mut Vec<String>,
    field: &'static str,
) -> Result<(), IsolationEvidenceError> {
    if values.len() > MAX_ITEMS {
        return Err(IsolationEvidenceError::TooManyItems(field));
    }
    for value in values.iter() {
        validate_text(field, value)?;
    }
    values.sort();
    for pair in values.windows(2) {
        if pair[0] == pair[1] {
            return Err(IsolationEvidenceError::DuplicateString(field));
        }
    }
    Ok(())
}

fn validate_mount_destination(value: &str) -> Result<(), IsolationEvidenceError> {
    validate_text("artifact destination", value)?;
    if !(value.starts_with("/inputs/") || value.starts_with("/trust/"))
        || value.ends_with('/')
        || value.contains("//")
        || value.contains("/../")
        || value.contains("/./")
    {
        return Err(IsolationEvidenceError::InvalidMountDestination(
            value.to_owned(),
        ));
    }
    Ok(())
}

fn validate_text(field: &'static str, value: &str) -> Result<(), IsolationEvidenceError> {
    if value.is_empty() || value.len() > MAX_TEXT || value.contains('\0') {
        Err(IsolationEvidenceError::InvalidText(field))
    } else {
        Ok(())
    }
}

fn push_namespaces(
    out: &mut Vec<u8>,
    namespaces: &NamespaceSnapshot,
) -> Result<(), IsolationEvidenceError> {
    namespaces.validate()?;
    for value in [
        &namespaces.user,
        &namespaces.mount,
        &namespaces.pid,
        &namespaces.ipc,
        &namespaces.net,
        &namespaces.uts,
    ] {
        push_string(out, value, "namespace identity")?;
    }
    Ok(())
}

fn push_strings(
    out: &mut Vec<u8>,
    values: &[String],
    field: &'static str,
) -> Result<(), IsolationEvidenceError> {
    push_count(out, values.len(), field)?;
    for value in values {
        push_string(out, value, field)?;
    }
    Ok(())
}

fn push_count(
    out: &mut Vec<u8>,
    count: usize,
    field: &'static str,
) -> Result<(), IsolationEvidenceError> {
    let count = u16::try_from(count)
        .map_err(|_| IsolationEvidenceError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&count.to_be_bytes());
    Ok(())
}

fn push_string(
    out: &mut Vec<u8>,
    value: &str,
    field: &'static str,
) -> Result<(), IsolationEvidenceError> {
    let len = u16::try_from(value.len())
        .map_err(|_| IsolationEvidenceError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), IsolationEvidenceError> {
    push_string(out, digest.algorithm().id(), "digest algorithm")?;
    let len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| IsolationEvidenceError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum IsolationEvidenceError {
    #[error(transparent)]
    Isolation(#[from] IsolationPolicyError),
    #[error("invalid text field: {0}")]
    InvalidText(&'static str),
    #[error("invalid environment key: {0}")]
    InvalidEnvironmentKey(String),
    #[error("invalid mount destination: {0}")]
    InvalidMountDestination(String),
    #[error("artifact {0} may not be empty")]
    EmptyArtifact(String),
    #[error("too many items in {0}")]
    TooManyItems(&'static str),
    #[error("duplicate environment key: {0}")]
    DuplicateEnvironmentKey(String),
    #[error("duplicate artifact role: {0}")]
    DuplicateArtifactRole(String),
    #[error("duplicate artifact destination: {0}")]
    DuplicateArtifactDestination(String),
    #[error("duplicate string in {0}")]
    DuplicateString(&'static str),
    #[error("canonical field is too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
    #[error("isolation policy digest mismatch")]
    PolicyDigestMismatch,
    #[error("inside and parent namespace observations differ")]
    InsideParentNamespaceMismatch,
    #[error("required namespaces were not isolated from the parent")]
    NamespaceNotIsolated,
    #[error("inside and parent mount observations differ")]
    MountInfoMismatch,
    #[error("sandbox child exited with code {0}")]
    SandboxChildFailed(i32),
    #[error("Linux capabilities remain inside the sandbox")]
    CapabilitiesRemain,
    #[error("unexpected sandbox hostname {0}")]
    UnexpectedHostname(String),
    #[error("sandbox environment differs from strict baseline")]
    EnvironmentMismatch,
    #[error("visible Nix store roots differ from committed closure")]
    StoreVisibilityMismatch,
    #[error("artifact identity/read-only observation mismatch")]
    ArtifactObservationMismatch,
    #[error("ephemeral work/tmp directory is not writable")]
    EphemeralWorkspaceNotWritable,
    #[error("host home state appears visible")]
    HostHomeVisible,
    #[error("unexpected host filesystem surface is visible")]
    UnexpectedHostFilesystemVisible,
    #[error("network isolation observation failed")]
    NetworkIsolationFailed,
    #[error("nested user namespace creation unexpectedly succeeded")]
    NestedUserNamespaceAllowed,
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_linux_isolation::NixClosureEntry;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn closure() -> NixClosureManifest {
        NixClosureManifest::new(vec![
            NixClosureEntry::new("/nix/store/aaaa-runner", digest(1)).unwrap(),
            NixClosureEntry::new("/nix/store/bbbb-gittuf", digest(2)).unwrap(),
        ])
        .unwrap()
    }

    fn invocation(closure: &NixClosureManifest) -> VerifierInvocation {
        VerifierInvocation::new(
            "/nix/store/aaaa-runner/bin/probe",
            vec!["/inputs/manifest.json".into()],
            closure,
        )
        .unwrap()
    }

    fn policy(closure: &NixClosureManifest) -> LinuxIsolationPolicyV1 {
        let invocation = invocation(closure);
        LinuxIsolationPolicyV1::strict(
            closure,
            &invocation,
            vec![ArtifactMount::new(
                "manifest",
                "/inputs/manifest.json",
                digest(3),
                64,
            )
            .unwrap()],
        )
        .unwrap()
    }

    fn child_namespaces() -> NamespaceSnapshot {
        NamespaceSnapshot {
            user: "user:[101]".into(),
            mount: "mnt:[102]".into(),
            pid: "pid:[103]".into(),
            ipc: "ipc:[104]".into(),
            net: "net:[105]".into(),
            uts: "uts:[106]".into(),
        }
    }

    fn host_namespaces() -> NamespaceSnapshot {
        NamespaceSnapshot {
            user: "user:[1]".into(),
            mount: "mnt:[2]".into(),
            pid: "pid:[3]".into(),
            ipc: "ipc:[4]".into(),
            net: "net:[5]".into(),
            uts: "uts:[6]".into(),
        }
    }

    fn inside(policy: &LinuxIsolationPolicyV1) -> InsideIsolationEvidence {
        InsideIsolationEvidence::new(
            policy.digest(DigestAlgorithm::Sha256).unwrap(),
            child_namespaces(),
            CapabilitySnapshot {
                inheritable: 0,
                permitted: 0,
                effective: 0,
                bounding: 0,
                ambient: 0,
            },
            SANDBOX_HOSTNAME,
            expected_environment(),
            vec!["aaaa-runner".into(), "bbbb-gittuf".into()],
            vec![ArtifactObservation::new(
                "manifest",
                "/inputs/manifest.json",
                digest(3),
                64,
                true,
                false,
            )
            .unwrap()],
            true,
            true,
            vec!["forge".into()],
            false,
            false,
            digest(4),
            digest(5),
            false,
            false,
            false,
        )
        .unwrap()
    }

    fn parent() -> ParentIsolationEvidence {
        ParentIsolationEvidence::new(
            host_namespaces(),
            child_namespaces(),
            digest(4),
            digest(6),
            0,
        )
        .unwrap()
    }

    #[test]
    fn healthy_two_channel_evidence_qualifies() {
        let closure = closure();
        let invocation = invocation(&closure);
        let policy = policy(&closure);
        let qualified = qualify_linux_isolation(
            &policy,
            &closure,
            &invocation,
            &inside(&policy),
            &parent(),
        )
        .unwrap();
        assert_eq!(
            qualified.policy_digest(),
            &policy.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn shared_network_namespace_fails_closed() {
        let closure = closure();
        let invocation = invocation(&closure);
        let policy = policy(&closure);
        let inside = inside(&policy);
        let mut child = child_namespaces();
        child.net = host_namespaces().net;
        let parent = ParentIsolationEvidence::new(
            host_namespaces(),
            child,
            digest(4),
            digest(6),
            0,
        )
        .unwrap();
        assert!(matches!(
            qualify_linux_isolation(&policy, &closure, &invocation, &inside, &parent),
            Err(IsolationEvidenceError::InsideParentNamespaceMismatch)
        ));
    }

    #[test]
    fn artifact_digest_substitution_fails_closed() {
        let closure = closure();
        let invocation = invocation(&closure);
        let policy = policy(&closure);
        let mut inside = inside(&policy);
        inside.artifacts[0] = ArtifactObservation::new(
            "manifest",
            "/inputs/manifest.json",
            digest(0xff),
            64,
            true,
            false,
        )
        .unwrap();
        assert!(matches!(
            qualify_linux_isolation(&policy, &closure, &invocation, &inside, &parent()),
            Err(IsolationEvidenceError::ArtifactObservationMismatch)
        ));
    }

    #[test]
    fn directory_cannot_masquerade_as_read_only_file() {
        let closure = closure();
        let invocation = invocation(&closure);
        let policy = policy(&closure);
        let mut inside = inside(&policy);
        inside.artifacts[0] = ArtifactObservation::new(
            "manifest",
            "/inputs/manifest.json",
            digest(3),
            64,
            false,
            false,
        )
        .unwrap();
        assert!(matches!(
            qualify_linux_isolation(&policy, &closure, &invocation, &inside, &parent()),
            Err(IsolationEvidenceError::ArtifactObservationMismatch)
        ));
    }

    #[test]
    fn remaining_capability_fails_closed() {
        let closure = closure();
        let invocation = invocation(&closure);
        let policy = policy(&closure);
        let mut inside = inside(&policy);
        inside.capabilities.effective = 1;
        assert!(matches!(
            qualify_linux_isolation(&policy, &closure, &invocation, &inside, &parent()),
            Err(IsolationEvidenceError::CapabilitiesRemain)
        ));
    }

    #[test]
    fn serialized_duplicate_environment_key_is_rejected() {
        let policy = policy(&closure());
        let mut value = serde_json::to_value(inside(&policy)).unwrap();
        value["environment"].as_array_mut().unwrap().push(
            serde_json::json!({"key":"HOME","value":"/evil"}),
        );
        assert!(serde_json::from_value::<InsideIsolationEvidence>(value).is_err());
    }
}
