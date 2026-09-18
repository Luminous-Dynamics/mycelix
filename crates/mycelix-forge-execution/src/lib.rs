// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Evidence-bound execution contracts for Mycelix Forge.
//!
//! This crate describes *what* a constrained execution claims to have used:
//! exact tools, inputs, trust material, environment, clock, filesystem policy,
//! and network policy. It deliberately does not prove that an executor really
//! enforced those constraints. A later qualified executor (FORGE-004D2) must
//! provide that stronger theorem before repository replay may mint
//! `OfflineEvidence`.

use mycelix_forge_core::{Digest, DigestAlgorithm, ProtocolVersion};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use std::fmt;
use thiserror::Error;

const EXECUTION_SPEC_DOMAIN_V1: &[u8] = b"mycelix-forge/execution-spec/v1\0";
const EXECUTION_OBSERVATION_DOMAIN_V1: &[u8] = b"mycelix-forge/execution-observation/v1\0";
const MAX_ROLE_LEN: usize = 128;
const MAX_VERSION_LEN: usize = 128;
const MAX_MEDIA_TYPE_LEN: usize = 256;
const MAX_ENV_KEY_LEN: usize = 128;
const MAX_ENV_VALUE_LEN: usize = 4096;
const MAX_EXECUTOR_FIELD_LEN: usize = 128;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ExecutionPurpose {
    RepositoryVerification,
    BuildQualification,
    ReleaseVerification,
}

impl ExecutionPurpose {
    const fn code(self) -> u8 {
        match self {
            Self::RepositoryVerification => 1,
            Self::BuildQualification => 2,
            Self::ReleaseVerification => 3,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum NetworkPolicy {
    Denied,
    LoopbackOnly,
    Unrestricted,
}

impl NetworkPolicy {
    const fn code(self) -> u8 {
        match self {
            Self::Denied => 1,
            Self::LoopbackOnly => 2,
            Self::Unrestricted => 3,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ClockPolicy {
    FixedUnixSeconds(u64),
    HostRealtime,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FilesystemPolicy {
    read_only_inputs: bool,
    ephemeral_workdir: bool,
    host_home_visible: bool,
}

impl FilesystemPolicy {
    pub const fn hermetic() -> Self {
        Self {
            read_only_inputs: true,
            ephemeral_workdir: true,
            host_home_visible: false,
        }
    }

    pub const fn new(
        read_only_inputs: bool,
        ephemeral_workdir: bool,
        host_home_visible: bool,
    ) -> Self {
        Self {
            read_only_inputs,
            ephemeral_workdir,
            host_home_visible,
        }
    }

    pub const fn read_only_inputs(self) -> bool {
        self.read_only_inputs
    }

    pub const fn ephemeral_workdir(self) -> bool {
        self.ephemeral_workdir
    }

    pub const fn host_home_visible(self) -> bool {
        self.host_home_visible
    }

    pub const fn is_hermetic(self) -> bool {
        self.read_only_inputs && self.ephemeral_workdir && !self.host_home_visible
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct ToolArtifact {
    role: String,
    semantic_version: String,
    digest: Digest,
    size: u64,
    derivation: Option<Digest>,
}

impl ToolArtifact {
    pub fn new(
        role: impl Into<String>,
        semantic_version: impl Into<String>,
        digest: Digest,
        size: u64,
        derivation: Option<Digest>,
    ) -> Result<Self, ExecutionContractError> {
        let role = role.into();
        let semantic_version = semantic_version.into();
        validate_text("tool role", &role, MAX_ROLE_LEN)?;
        validate_text("tool semantic version", &semantic_version, MAX_VERSION_LEN)?;
        if size == 0 {
            return Err(ExecutionContractError::EmptyArtifact("tool"));
        }
        Ok(Self {
            role,
            semantic_version,
            digest,
            size,
            derivation,
        })
    }

    pub fn role(&self) -> &str {
        &self.role
    }

    pub fn semantic_version(&self) -> &str {
        &self.semantic_version
    }

    pub fn digest(&self) -> &Digest {
        &self.digest
    }

    pub const fn size(&self) -> u64 {
        self.size
    }

    pub fn derivation(&self) -> Option<&Digest> {
        self.derivation.as_ref()
    }
}

impl<'de> Deserialize<'de> for ToolArtifact {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireToolArtifact {
            role: String,
            semantic_version: String,
            digest: Digest,
            size: u64,
            derivation: Option<Digest>,
        }

        let wire = WireToolArtifact::deserialize(deserializer)?;
        Self::new(
            wire.role,
            wire.semantic_version,
            wire.digest,
            wire.size,
            wire.derivation,
        )
        .map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct TrustMaterial {
    role: String,
    media_type: String,
    digest: Digest,
    size: u64,
}

impl TrustMaterial {
    pub fn new(
        role: impl Into<String>,
        media_type: impl Into<String>,
        digest: Digest,
        size: u64,
    ) -> Result<Self, ExecutionContractError> {
        let role = role.into();
        let media_type = media_type.into();
        validate_text("trust role", &role, MAX_ROLE_LEN)?;
        validate_text("trust media type", &media_type, MAX_MEDIA_TYPE_LEN)?;
        if size == 0 {
            return Err(ExecutionContractError::EmptyArtifact("trust material"));
        }
        Ok(Self {
            role,
            media_type,
            digest,
            size,
        })
    }

    pub fn role(&self) -> &str {
        &self.role
    }

    pub fn media_type(&self) -> &str {
        &self.media_type
    }

    pub fn digest(&self) -> &Digest {
        &self.digest
    }

    pub const fn size(&self) -> u64 {
        self.size
    }
}

impl<'de> Deserialize<'de> for TrustMaterial {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireTrustMaterial {
            role: String,
            media_type: String,
            digest: Digest,
            size: u64,
        }

        let wire = WireTrustMaterial::deserialize(deserializer)?;
        Self::new(wire.role, wire.media_type, wire.digest, wire.size).map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct InputArtifact {
    role: String,
    digest: Digest,
    size: u64,
}

impl InputArtifact {
    pub fn new(
        role: impl Into<String>,
        digest: Digest,
        size: u64,
    ) -> Result<Self, ExecutionContractError> {
        let role = role.into();
        validate_text("input role", &role, MAX_ROLE_LEN)?;
        if size == 0 {
            return Err(ExecutionContractError::EmptyArtifact("input"));
        }
        Ok(Self { role, digest, size })
    }

    pub fn role(&self) -> &str {
        &self.role
    }

    pub fn digest(&self) -> &Digest {
        &self.digest
    }

    pub const fn size(&self) -> u64 {
        self.size
    }
}

impl<'de> Deserialize<'de> for InputArtifact {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireInputArtifact {
            role: String,
            digest: Digest,
            size: u64,
        }

        let wire = WireInputArtifact::deserialize(deserializer)?;
        Self::new(wire.role, wire.digest, wire.size).map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct EnvironmentBinding {
    key: String,
    value: String,
}

impl EnvironmentBinding {
    pub fn new(
        key: impl Into<String>,
        value: impl Into<String>,
    ) -> Result<Self, ExecutionContractError> {
        let key = key.into();
        let value = value.into();
        validate_text("environment key", &key, MAX_ENV_KEY_LEN)?;
        if key.contains('=') {
            return Err(ExecutionContractError::InvalidEnvironmentKey(key));
        }
        if value.len() > MAX_ENV_VALUE_LEN || value.contains('\0') {
            return Err(ExecutionContractError::InvalidText {
                field: "environment value",
                len: value.len(),
                max: MAX_ENV_VALUE_LEN,
            });
        }
        Ok(Self { key, value })
    }

    pub fn key(&self) -> &str {
        &self.key
    }

    pub fn value(&self) -> &str {
        &self.value
    }
}

impl<'de> Deserialize<'de> for EnvironmentBinding {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireEnvironmentBinding {
            key: String,
            value: String,
        }

        let wire = WireEnvironmentBinding::deserialize(deserializer)?;
        Self::new(wire.key, wire.value).map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ExecutionSpec {
    version: ProtocolVersion,
    purpose: ExecutionPurpose,
    subject: Digest,
    tools: Vec<ToolArtifact>,
    trust_material: Vec<TrustMaterial>,
    inputs: Vec<InputArtifact>,
    environment: Vec<EnvironmentBinding>,
    network: NetworkPolicy,
    clock: ClockPolicy,
    filesystem: FilesystemPolicy,
}

impl ExecutionSpec {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        purpose: ExecutionPurpose,
        subject: Digest,
        tools: Vec<ToolArtifact>,
        trust_material: Vec<TrustMaterial>,
        inputs: Vec<InputArtifact>,
        environment: Vec<EnvironmentBinding>,
        network: NetworkPolicy,
        clock: ClockPolicy,
        filesystem: FilesystemPolicy,
    ) -> Result<Self, ExecutionContractError> {
        if tools.is_empty() {
            return Err(ExecutionContractError::MissingTools);
        }
        if inputs.is_empty() {
            return Err(ExecutionContractError::MissingInputs);
        }

        let tools = canonicalize_tools(tools)?;
        let trust_material = canonicalize_trust_material(trust_material)?;
        let inputs = canonicalize_inputs(inputs)?;
        let environment = canonicalize_environment(environment)?;

        Ok(Self {
            version: ProtocolVersion::CURRENT,
            purpose,
            subject,
            tools,
            trust_material,
            inputs,
            environment,
            network,
            clock,
            filesystem,
        })
    }

    pub const fn purpose(&self) -> ExecutionPurpose {
        self.purpose
    }

    pub fn subject(&self) -> &Digest {
        &self.subject
    }

    pub fn tools(&self) -> &[ToolArtifact] {
        &self.tools
    }

    pub fn trust_material(&self) -> &[TrustMaterial] {
        &self.trust_material
    }

    pub fn inputs(&self) -> &[InputArtifact] {
        &self.inputs
    }

    pub fn environment(&self) -> &[EnvironmentBinding] {
        &self.environment
    }

    pub const fn network(&self) -> NetworkPolicy {
        self.network
    }

    pub const fn clock(&self) -> ClockPolicy {
        self.clock
    }

    pub const fn filesystem(&self) -> FilesystemPolicy {
        self.filesystem
    }

    pub const fn is_hermetic_candidate(&self) -> bool {
        matches!(self.network, NetworkPolicy::Denied)
            && matches!(self.clock, ClockPolicy::FixedUnixSeconds(_))
            && self.filesystem.is_hermetic()
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ExecutionContractError> {
        let mut out = Vec::new();
        out.extend_from_slice(EXECUTION_SPEC_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        out.push(self.purpose.code());
        push_digest(&mut out, &self.subject)?;

        push_count(&mut out, self.tools.len(), "tools")?;
        for tool in &self.tools {
            push_string(&mut out, &tool.role, "tool role")?;
            push_string(
                &mut out,
                &tool.semantic_version,
                "tool semantic version",
            )?;
            push_digest(&mut out, &tool.digest)?;
            out.extend_from_slice(&tool.size.to_be_bytes());
            push_optional_digest(&mut out, tool.derivation.as_ref())?;
        }

        push_count(&mut out, self.trust_material.len(), "trust material")?;
        for trust in &self.trust_material {
            push_string(&mut out, &trust.role, "trust role")?;
            push_string(&mut out, &trust.media_type, "trust media type")?;
            push_digest(&mut out, &trust.digest)?;
            out.extend_from_slice(&trust.size.to_be_bytes());
        }

        push_count(&mut out, self.inputs.len(), "inputs")?;
        for input in &self.inputs {
            push_string(&mut out, &input.role, "input role")?;
            push_digest(&mut out, &input.digest)?;
            out.extend_from_slice(&input.size.to_be_bytes());
        }

        push_count(&mut out, self.environment.len(), "environment")?;
        for binding in &self.environment {
            push_string(&mut out, &binding.key, "environment key")?;
            push_string(&mut out, &binding.value, "environment value")?;
        }

        out.push(self.network.code());
        match self.clock {
            ClockPolicy::FixedUnixSeconds(seconds) => {
                out.push(1);
                out.extend_from_slice(&seconds.to_be_bytes());
            }
            ClockPolicy::HostRealtime => out.push(2),
        }
        out.push(u8::from(self.filesystem.read_only_inputs));
        out.push(u8::from(self.filesystem.ephemeral_workdir));
        out.push(u8::from(self.filesystem.host_home_visible));
        Ok(out)
    }

    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, ExecutionContractError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for ExecutionSpec {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireExecutionSpec {
            version: ProtocolVersion,
            purpose: ExecutionPurpose,
            subject: Digest,
            tools: Vec<ToolArtifact>,
            trust_material: Vec<TrustMaterial>,
            inputs: Vec<InputArtifact>,
            environment: Vec<EnvironmentBinding>,
            network: NetworkPolicy,
            clock: ClockPolicy,
            filesystem: FilesystemPolicy,
        }

        let wire = WireExecutionSpec::deserialize(deserializer)?;
        if wire.version != ProtocolVersion::CURRENT {
            return Err(D::Error::custom("unsupported execution-spec version"));
        }
        Self::new(
            wire.purpose,
            wire.subject,
            wire.tools,
            wire.trust_material,
            wire.inputs,
            wire.environment,
            wire.network,
            wire.clock,
            wire.filesystem,
        )
        .map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ExecutorIdentity {
    name: String,
    version: String,
}

impl ExecutorIdentity {
    pub fn new(
        name: impl Into<String>,
        version: impl Into<String>,
    ) -> Result<Self, ExecutionContractError> {
        let name = name.into();
        let version = version.into();
        validate_text("executor name", &name, MAX_EXECUTOR_FIELD_LEN)?;
        validate_text("executor version", &version, MAX_EXECUTOR_FIELD_LEN)?;
        Ok(Self { name, version })
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn version(&self) -> &str {
        &self.version
    }
}

impl<'de> Deserialize<'de> for ExecutorIdentity {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireExecutorIdentity {
            name: String,
            version: String,
        }

        let wire = WireExecutorIdentity::deserialize(deserializer)?;
        Self::new(wire.name, wire.version).map_err(D::Error::custom)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ExecutionOutcome {
    Succeeded,
    Failed,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ExecutionObservation {
    executor: ExecutorIdentity,
    spec_digest: Digest,
    subject: Digest,
    outcome: ExecutionOutcome,
    output: Option<Digest>,
    execution_evidence: Option<Digest>,
    observed_clock_unix_seconds: u64,
}

impl ExecutionObservation {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        executor: ExecutorIdentity,
        spec_digest: Digest,
        subject: Digest,
        outcome: ExecutionOutcome,
        output: Option<Digest>,
        execution_evidence: Option<Digest>,
        observed_clock_unix_seconds: u64,
    ) -> Result<Self, ExecutionContractError> {
        if outcome == ExecutionOutcome::Succeeded && output.is_none() {
            return Err(ExecutionContractError::SuccessfulObservationMissingOutput);
        }
        Ok(Self {
            executor,
            spec_digest,
            subject,
            outcome,
            output,
            execution_evidence,
            observed_clock_unix_seconds,
        })
    }

    pub fn executor(&self) -> &ExecutorIdentity {
        &self.executor
    }

    pub fn spec_digest(&self) -> &Digest {
        &self.spec_digest
    }

    pub fn subject(&self) -> &Digest {
        &self.subject
    }

    pub const fn outcome(&self) -> ExecutionOutcome {
        self.outcome
    }

    pub fn output(&self) -> Option<&Digest> {
        self.output.as_ref()
    }

    pub fn execution_evidence(&self) -> Option<&Digest> {
        self.execution_evidence.as_ref()
    }

    pub const fn observed_clock_unix_seconds(&self) -> u64 {
        self.observed_clock_unix_seconds
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ExecutionContractError> {
        let mut out = Vec::new();
        out.extend_from_slice(EXECUTION_OBSERVATION_DOMAIN_V1);
        push_string(&mut out, self.executor.name(), "executor name")?;
        push_string(&mut out, self.executor.version(), "executor version")?;
        push_digest(&mut out, &self.spec_digest)?;
        push_digest(&mut out, &self.subject)?;
        out.push(match self.outcome {
            ExecutionOutcome::Succeeded => 1,
            ExecutionOutcome::Failed => 2,
        });
        push_optional_digest(&mut out, self.output.as_ref())?;
        push_optional_digest(&mut out, self.execution_evidence.as_ref())?;
        out.extend_from_slice(&self.observed_clock_unix_seconds.to_be_bytes());
        Ok(out)
    }

    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, ExecutionContractError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for ExecutionObservation {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireExecutionObservation {
            executor: ExecutorIdentity,
            spec_digest: Digest,
            subject: Digest,
            outcome: ExecutionOutcome,
            output: Option<Digest>,
            execution_evidence: Option<Digest>,
            observed_clock_unix_seconds: u64,
        }

        let wire = WireExecutionObservation::deserialize(deserializer)?;
        Self::new(
            wire.executor,
            wire.spec_digest,
            wire.subject,
            wire.outcome,
            wire.output,
            wire.execution_evidence,
            wire.observed_clock_unix_seconds,
        )
        .map_err(D::Error::custom)
    }
}

/// Structurally evidence-bound execution result.
///
/// This type proves that an observation matches one exact hermetic-candidate
/// execution specification and contains an executor-evidence commitment. It
/// does *not* independently prove that the executor enforced the sandbox.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EvidenceBoundExecution {
    spec_digest: Digest,
    executor: ExecutorIdentity,
    output: Digest,
    execution_evidence: Digest,
}

impl EvidenceBoundExecution {
    pub fn spec_digest(&self) -> &Digest {
        &self.spec_digest
    }

    pub fn executor(&self) -> &ExecutorIdentity {
        &self.executor
    }

    pub fn output(&self) -> &Digest {
        &self.output
    }

    pub fn execution_evidence(&self) -> &Digest {
        &self.execution_evidence
    }
}

pub fn qualify_evidence_bound_execution(
    spec: &ExecutionSpec,
    observation: ExecutionObservation,
) -> Result<EvidenceBoundExecution, ExecutionContractError> {
    if !spec.is_hermetic_candidate() {
        return Err(ExecutionContractError::NotHermeticCandidate);
    }

    let expected_spec = spec.digest(observation.spec_digest.algorithm())?;
    if expected_spec != observation.spec_digest {
        return Err(ExecutionContractError::SpecDigestMismatch);
    }
    if &observation.subject != spec.subject() {
        return Err(ExecutionContractError::SubjectMismatch);
    }
    if observation.outcome != ExecutionOutcome::Succeeded {
        return Err(ExecutionContractError::ExecutionFailed);
    }

    let expected_clock = match spec.clock() {
        ClockPolicy::FixedUnixSeconds(seconds) => seconds,
        ClockPolicy::HostRealtime => return Err(ExecutionContractError::NotHermeticCandidate),
    };
    if observation.observed_clock_unix_seconds != expected_clock {
        return Err(ExecutionContractError::ClockMismatch {
            expected: expected_clock,
            observed: observation.observed_clock_unix_seconds,
        });
    }

    let output = observation
        .output
        .ok_or(ExecutionContractError::SuccessfulObservationMissingOutput)?;
    let execution_evidence = observation
        .execution_evidence
        .ok_or(ExecutionContractError::MissingExecutionEvidence)?;

    Ok(EvidenceBoundExecution {
        spec_digest: observation.spec_digest,
        executor: observation.executor,
        output,
        execution_evidence,
    })
}

fn canonicalize_tools(
    mut values: Vec<ToolArtifact>,
) -> Result<Vec<ToolArtifact>, ExecutionContractError> {
    values.sort();
    for pair in values.windows(2) {
        if pair[0].role == pair[1].role {
            return Err(ExecutionContractError::DuplicateRole {
                kind: "tool",
                role: pair[0].role.clone(),
            });
        }
    }
    Ok(values)
}

fn canonicalize_trust_material(
    mut values: Vec<TrustMaterial>,
) -> Result<Vec<TrustMaterial>, ExecutionContractError> {
    values.sort();
    for pair in values.windows(2) {
        if pair[0].role == pair[1].role {
            return Err(ExecutionContractError::DuplicateRole {
                kind: "trust material",
                role: pair[0].role.clone(),
            });
        }
    }
    Ok(values)
}

fn canonicalize_inputs(
    mut values: Vec<InputArtifact>,
) -> Result<Vec<InputArtifact>, ExecutionContractError> {
    values.sort();
    for pair in values.windows(2) {
        if pair[0].role == pair[1].role {
            return Err(ExecutionContractError::DuplicateRole {
                kind: "input",
                role: pair[0].role.clone(),
            });
        }
    }
    Ok(values)
}

fn canonicalize_environment(
    mut values: Vec<EnvironmentBinding>,
) -> Result<Vec<EnvironmentBinding>, ExecutionContractError> {
    values.sort();
    for pair in values.windows(2) {
        if pair[0].key == pair[1].key {
            return Err(ExecutionContractError::DuplicateEnvironmentKey(
                pair[0].key.clone(),
            ));
        }
    }
    Ok(values)
}

fn validate_text(
    field: &'static str,
    value: &str,
    max: usize,
) -> Result<(), ExecutionContractError> {
    if value.is_empty() || value.len() > max || value.contains('\0') {
        return Err(ExecutionContractError::InvalidText {
            field,
            len: value.len(),
            max,
        });
    }
    Ok(())
}

fn push_count(
    out: &mut Vec<u8>,
    count: usize,
    field: &'static str,
) -> Result<(), ExecutionContractError> {
    let count = u16::try_from(count).map_err(|_| ExecutionContractError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&count.to_be_bytes());
    Ok(())
}

fn push_string(
    out: &mut Vec<u8>,
    value: &str,
    field: &'static str,
) -> Result<(), ExecutionContractError> {
    let bytes = value.as_bytes();
    let len = u16::try_from(bytes.len())
        .map_err(|_| ExecutionContractError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), ExecutionContractError> {
    push_string(out, digest.algorithm().id(), "digest algorithm")?;
    let len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| ExecutionContractError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

fn push_optional_digest(
    out: &mut Vec<u8>,
    digest: Option<&Digest>,
) -> Result<(), ExecutionContractError> {
    match digest {
        Some(value) => {
            out.push(1);
            push_digest(out, value)?;
        }
        None => out.push(0),
    }
    Ok(())
}

#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ExecutionContractError {
    #[error("invalid {field}: length {len}, maximum {max}, and value must be non-empty/NUL-free")]
    InvalidText {
        field: &'static str,
        len: usize,
        max: usize,
    },
    #[error("invalid environment key: {0}")]
    InvalidEnvironmentKey(String),
    #[error("{0} artifact may not be empty")]
    EmptyArtifact(&'static str),
    #[error("execution specification requires at least one tool")]
    MissingTools,
    #[error("execution specification requires at least one input")]
    MissingInputs,
    #[error("duplicate {kind} role: {role}")]
    DuplicateRole { kind: &'static str, role: String },
    #[error("duplicate environment key: {0}")]
    DuplicateEnvironmentKey(String),
    #[error("canonical field is too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
    #[error("successful execution observation requires an output digest")]
    SuccessfulObservationMissingOutput,
    #[error("execution specification is not a hermetic candidate")]
    NotHermeticCandidate,
    #[error("execution observation does not bind the exact execution specification")]
    SpecDigestMismatch,
    #[error("execution observation subject does not match the specification subject")]
    SubjectMismatch,
    #[error("execution did not succeed")]
    ExecutionFailed,
    #[error("successful hermetic-candidate execution requires executor evidence")]
    MissingExecutionEvidence,
    #[error("fixed verification clock mismatch: expected {expected}, observed {observed}")]
    ClockMismatch { expected: u64, observed: u64 },
}

impl fmt::Display for ExecutionPurpose {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::RepositoryVerification => "repository-verification",
            Self::BuildQualification => "build-qualification",
            Self::ReleaseVerification => "release-verification",
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn tool(role: &str, byte: u8) -> ToolArtifact {
        ToolArtifact::new(role, "1.0.0", digest(byte), 100, Some(digest(byte + 1))).unwrap()
    }

    fn input(role: &str, byte: u8) -> InputArtifact {
        InputArtifact::new(role, digest(byte), 200).unwrap()
    }

    fn spec() -> ExecutionSpec {
        ExecutionSpec::new(
            ExecutionPurpose::RepositoryVerification,
            digest(0x10),
            vec![tool("gittuf", 0x20), tool("git", 0x30)],
            vec![TrustMaterial::new(
                "sigstore-trusted-root",
                "application/vnd.dev.sigstore.trustedroot+json;version=0.1",
                digest(0x40),
                300,
            )
            .unwrap()],
            vec![input("repository-bundle", 0x50), input("bundle-manifest", 0x60)],
            vec![
                EnvironmentBinding::new("LANG", "C").unwrap(),
                EnvironmentBinding::new("GIT_NO_LAZY_FETCH", "1").unwrap(),
            ],
            NetworkPolicy::Denied,
            ClockPolicy::FixedUnixSeconds(1_800_000_000),
            FilesystemPolicy::hermetic(),
        )
        .unwrap()
    }

    fn observation(spec: &ExecutionSpec) -> ExecutionObservation {
        ExecutionObservation::new(
            ExecutorIdentity::new("spore-verifier", "0.1.0").unwrap(),
            spec.digest(DigestAlgorithm::Sha256).unwrap(),
            spec.subject().clone(),
            ExecutionOutcome::Succeeded,
            Some(digest(0x70)),
            Some(digest(0x80)),
            1_800_000_000,
        )
        .unwrap()
    }

    #[test]
    fn canonical_order_does_not_depend_on_input_order() {
        let a = spec();
        let b = ExecutionSpec::new(
            ExecutionPurpose::RepositoryVerification,
            digest(0x10),
            vec![tool("git", 0x30), tool("gittuf", 0x20)],
            vec![TrustMaterial::new(
                "sigstore-trusted-root",
                "application/vnd.dev.sigstore.trustedroot+json;version=0.1",
                digest(0x40),
                300,
            )
            .unwrap()],
            vec![input("bundle-manifest", 0x60), input("repository-bundle", 0x50)],
            vec![
                EnvironmentBinding::new("GIT_NO_LAZY_FETCH", "1").unwrap(),
                EnvironmentBinding::new("LANG", "C").unwrap(),
            ],
            NetworkPolicy::Denied,
            ClockPolicy::FixedUnixSeconds(1_800_000_000),
            FilesystemPolicy::hermetic(),
        )
        .unwrap();
        assert_eq!(
            a.digest(DigestAlgorithm::Sha256).unwrap(),
            b.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn duplicate_tool_role_is_rejected() {
        let result = ExecutionSpec::new(
            ExecutionPurpose::RepositoryVerification,
            digest(1),
            vec![tool("git", 2), tool("git", 3)],
            vec![],
            vec![input("bundle", 4)],
            vec![],
            NetworkPolicy::Denied,
            ClockPolicy::FixedUnixSeconds(10),
            FilesystemPolicy::hermetic(),
        );
        assert!(matches!(
            result,
            Err(ExecutionContractError::DuplicateRole { kind: "tool", .. })
        ));
    }

    #[test]
    fn non_hermetic_spec_cannot_qualify() {
        let non_hermetic = ExecutionSpec::new(
            ExecutionPurpose::RepositoryVerification,
            digest(1),
            vec![tool("git", 2)],
            vec![],
            vec![input("bundle", 3)],
            vec![],
            NetworkPolicy::Unrestricted,
            ClockPolicy::HostRealtime,
            FilesystemPolicy::new(false, false, true),
        )
        .unwrap();
        let observed = ExecutionObservation::new(
            ExecutorIdentity::new("test", "1").unwrap(),
            non_hermetic.digest(DigestAlgorithm::Sha256).unwrap(),
            non_hermetic.subject().clone(),
            ExecutionOutcome::Succeeded,
            Some(digest(4)),
            Some(digest(5)),
            10,
        )
        .unwrap();
        assert_eq!(
            qualify_evidence_bound_execution(&non_hermetic, observed).unwrap_err(),
            ExecutionContractError::NotHermeticCandidate
        );
    }

    #[test]
    fn exact_observation_qualifies_structurally() {
        let spec = spec();
        let qualified = qualify_evidence_bound_execution(&spec, observation(&spec)).unwrap();
        assert_eq!(qualified.output(), &digest(0x70));
        assert_eq!(qualified.execution_evidence(), &digest(0x80));
    }

    #[test]
    fn changed_spec_is_rejected() {
        let spec = spec();
        let mut observed = observation(&spec);
        observed.spec_digest = digest(0xff);
        assert_eq!(
            qualify_evidence_bound_execution(&spec, observed).unwrap_err(),
            ExecutionContractError::SpecDigestMismatch
        );
    }

    #[test]
    fn wrong_fixed_clock_is_rejected() {
        let spec = spec();
        let mut observed = observation(&spec);
        observed.observed_clock_unix_seconds += 1;
        assert!(matches!(
            qualify_evidence_bound_execution(&spec, observed),
            Err(ExecutionContractError::ClockMismatch { .. })
        ));
    }

    #[test]
    fn missing_executor_evidence_is_rejected() {
        let spec = spec();
        let observed = ExecutionObservation::new(
            ExecutorIdentity::new("spore-verifier", "0.1.0").unwrap(),
            spec.digest(DigestAlgorithm::Sha256).unwrap(),
            spec.subject().clone(),
            ExecutionOutcome::Succeeded,
            Some(digest(0x70)),
            None,
            1_800_000_000,
        )
        .unwrap();
        assert_eq!(
            qualify_evidence_bound_execution(&spec, observed).unwrap_err(),
            ExecutionContractError::MissingExecutionEvidence
        );
    }

    #[test]
    fn deserialization_revalidates_duplicate_environment_keys() {
        let spec = spec();
        let mut value = serde_json::to_value(&spec).unwrap();
        let env = value["environment"].as_array_mut().unwrap();
        env.push(env[0].clone());
        assert!(serde_json::from_value::<ExecutionSpec>(value).is_err());
    }
}
