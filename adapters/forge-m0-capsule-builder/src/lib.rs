// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-004D4E: deterministic, acyclic M0 capsule construction.
//!
//! This crate has no authority role. It captures one already-validated portable
//! replay into an immutable construction seed, then constructs the D4A-D4C
//! objects in topological order. The later host/composer still perform all
//! runtime and repository qualification.

use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_execution::{
    EnvironmentBinding, ExecutionContractError, ExecutionPurpose, ExecutionSpec,
    FilesystemPolicy, InputArtifact, NetworkPolicy, ToolArtifact, VerificationTimePolicy,
};
use mycelix_forge_gittuf_adapter::{AdapterError, REQUIRED_GITTUF_VERSION};
use mycelix_forge_gittuf_bundle::{OfflineManifestError, PortableRepositoryReplay};
use mycelix_forge_gittuf_trust_profile::local_embedded_keys_v1;
use mycelix_forge_guest_plan::{
    GitObjectValidationPolicyV1, GuestPlanError, GuestVerificationPlanV1, GUEST_MANIFEST_PATH,
    GUEST_PLAN_PATH,
};
use mycelix_forge_guest_tool_map::{
    qualify_guest_tool_map, GuestToolMapError, GuestToolMapV1,
};
use mycelix_forge_hermetic_run_evidence::{
    HermeticRunError, HermeticRunPlan, RUN_CHALLENGE_SIZE,
};
use mycelix_forge_linux_isolation::{
    ArtifactMount, IsolationPolicyError, LinuxIsolationPolicyV1, NixClosureManifest,
    VerifierInvocation,
};
use mycelix_forge_linux_isolation_evidence::expected_environment;
use mycelix_forge_m0_offline_evidence_v6::{
    derive_execution_subject_v6, M0CompositionV6Error,
};
use mycelix_forge_nar_auditor::auditor_subject;
use mycelix_forge_repository::{RepositoryVerificationError, RepositoryVerificationRequest};
use mycelix_forge_sealed_capsule_inputs::{CONTROL_INPUTS, GUEST_TOOL_MAP_PATH, POLICY_INPUTS};
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

const CONSTRUCTION_DOMAIN_V1: &[u8] = b"mycelix-forge/m0-capsule-construction/v1\0";
const HOST_VERSION: &str = "0.2.0";
const GUEST_VERSION: &str = "0.1.0";
const REQUIRED_TOOL_ROLES: &[&str] = &[
    "bubblewrap",
    "forge-hermetic-guest",
    "forge-hermetic-host",
    "forge-isolation-probe",
    "git",
    "gittuf",
];

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GeneratedCapsuleArtifact {
    role: String,
    destination: String,
    bytes: Vec<u8>,
    digest: Digest,
    size: u64,
}

impl GeneratedCapsuleArtifact {
    pub fn role(&self) -> &str {
        &self.role
    }

    pub fn destination(&self) -> &str {
        &self.destination
    }

    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    pub fn digest(&self) -> &Digest {
        &self.digest
    }

    pub const fn size(&self) -> u64 {
        self.size
    }
}

/// Positive construction input captured from one already-validated portable
/// repository replay. Its fields are private so production callers cannot forge
/// a seed without passing [`Self::from_replay`].
#[derive(Clone, Debug)]
pub struct M0CapsuleSeed {
    request: RepositoryVerificationRequest,
    request_digest: Digest,
    manifest_artifact: GeneratedCapsuleArtifact,
    manifest_commitment: Digest,
    replay_receipt: Digest,
    policy_state: Digest,
    bundle_digest: Digest,
    bundle_size: u64,
}

impl M0CapsuleSeed {
    pub fn from_replay(
        request: &RepositoryVerificationRequest,
        replay: &PortableRepositoryReplay,
    ) -> Result<Self, CapsuleBuilderError> {
        replay.manifest().validate_for_request(request)?;
        replay
            .replay_receipt()
            .clone()
            .into_observation_for(request)?;

        let request_digest = request.digest(DigestAlgorithm::Sha256)?;
        let manifest_commitment = replay
            .manifest()
            .commitment(DigestAlgorithm::Sha256)?;
        let replay_receipt = replay
            .replay_receipt()
            .commitment(DigestAlgorithm::Sha256)?;
        if &replay_receipt != replay.manifest().local_receipt_commitment() {
            return Err(CapsuleBuilderError::ReplayReceiptManifestMismatch);
        }
        let policy_state = replay
            .manifest()
            .policy_state()
            .digest(DigestAlgorithm::Sha256)?;
        let manifest_artifact = generated_json(
            "repository-bundle-manifest",
            GUEST_MANIFEST_PATH,
            replay.manifest(),
        )?;

        Ok(Self {
            request: (*request).clone(),
            request_digest,
            manifest_artifact,
            manifest_commitment,
            replay_receipt,
            policy_state,
            bundle_digest: replay.manifest().bundle_digest().clone(),
            bundle_size: replay.manifest().bundle_size(),
        })
    }

    pub fn request(&self) -> &RepositoryVerificationRequest {
        &self.request
    }

    pub fn request_digest(&self) -> &Digest {
        &self.request_digest
    }

    pub fn manifest_commitment(&self) -> &Digest {
        &self.manifest_commitment
    }

    pub fn replay_receipt(&self) -> &Digest {
        &self.replay_receipt
    }

    pub fn policy_state(&self) -> &Digest {
        &self.policy_state
    }

    pub fn bundle_digest(&self) -> &Digest {
        &self.bundle_digest
    }

    pub const fn bundle_size(&self) -> u64 {
        self.bundle_size
    }
}

pub struct M0CapsuleBuilderInputs<'a> {
    pub request: &'a RepositoryVerificationRequest,
    pub replay: &'a PortableRepositoryReplay,
    pub closure: &'a NixClosureManifest,
    pub invocation: &'a VerifierInvocation,
    pub tools: &'a [ToolArtifact],
    pub guest_tool_paths: BTreeMap<String, String>,
    pub run_challenge: [u8; RUN_CHALLENGE_SIZE as usize],
}

pub struct M0CapsuleSeedInputs<'a> {
    pub seed: &'a M0CapsuleSeed,
    pub closure: &'a NixClosureManifest,
    pub invocation: &'a VerifierInvocation,
    pub tools: &'a [ToolArtifact],
    pub guest_tool_paths: BTreeMap<String, String>,
    pub run_challenge: [u8; RUN_CHALLENGE_SIZE as usize],
}

pub struct BuiltM0Capsule {
    execution_subject: Digest,
    isolation_policy: LinuxIsolationPolicyV1,
    guest_plan: GuestVerificationPlanV1,
    guest_tool_map: GuestToolMapV1,
    execution_spec: ExecutionSpec,
    run_plan: HermeticRunPlan,
    generated_artifacts: Vec<GeneratedCapsuleArtifact>,
    construction_commitment: Digest,
}

impl BuiltM0Capsule {
    pub fn execution_subject(&self) -> &Digest {
        &self.execution_subject
    }

    pub fn isolation_policy(&self) -> &LinuxIsolationPolicyV1 {
        &self.isolation_policy
    }

    pub fn guest_plan(&self) -> &GuestVerificationPlanV1 {
        &self.guest_plan
    }

    pub fn guest_tool_map(&self) -> &GuestToolMapV1 {
        &self.guest_tool_map
    }

    pub fn execution_spec(&self) -> &ExecutionSpec {
        &self.execution_spec
    }

    pub fn run_plan(&self) -> &HermeticRunPlan {
        &self.run_plan
    }

    pub fn generated_artifacts(&self) -> &[GeneratedCapsuleArtifact] {
        &self.generated_artifacts
    }

    pub fn generated_artifact(&self, role: &str) -> Option<&GeneratedCapsuleArtifact> {
        self.generated_artifacts
            .iter()
            .find(|artifact| artifact.role() == role)
    }

    pub fn construction_commitment(&self) -> &Digest {
        &self.construction_commitment
    }
}

pub fn build_m0_capsule(
    inputs: M0CapsuleBuilderInputs<'_>,
) -> Result<BuiltM0Capsule, CapsuleBuilderError> {
    let seed = M0CapsuleSeed::from_replay(inputs.request, inputs.replay)?;
    build_m0_capsule_from_seed(M0CapsuleSeedInputs {
        seed: &seed,
        closure: inputs.closure,
        invocation: inputs.invocation,
        tools: inputs.tools,
        guest_tool_paths: inputs.guest_tool_paths,
        run_challenge: inputs.run_challenge,
    })
}

/// Pure deterministic construction after portable replay validation has been
/// captured into an [`M0CapsuleSeed`]. This function still grants no authority;
/// it only constructs and self-checks the exact M0 capsule graph.
pub fn build_m0_capsule_from_seed(
    inputs: M0CapsuleSeedInputs<'_>,
) -> Result<BuiltM0Capsule, CapsuleBuilderError> {
    require_exact_tools(inputs.tools)?;
    require_guest_invocation(inputs.invocation)?;

    let closure_digest = inputs.closure.digest(DigestAlgorithm::Sha256)?;
    let invocation_digest = inputs.invocation.digest(DigestAlgorithm::Sha256)?;
    let trust_profile = local_embedded_keys_v1().profile_digest().clone();
    let challenge_digest = Digest::of_bytes(DigestAlgorithm::Sha256, &inputs.run_challenge);
    let git_policy = GitObjectValidationPolicyV1::strict().digest(DigestAlgorithm::Sha256)?;
    let nar_auditor = auditor_subject();

    let manifest_json = inputs.seed.manifest_artifact.clone();
    let closure_json = generated_json(
        "nix-closure-manifest",
        POLICY_INPUTS
            .iter()
            .find(|(role, _)| *role == "nix-closure-manifest")
            .expect("D4A closure role exists")
            .1,
        inputs.closure,
    )?;
    let invocation_json = generated_json(
        "sandbox-invocation",
        POLICY_INPUTS
            .iter()
            .find(|(role, _)| *role == "sandbox-invocation")
            .expect("D4A invocation role exists")
            .1,
        inputs.invocation,
    )?;
    let challenge = generated_bytes(
        "run-challenge",
        POLICY_INPUTS
            .iter()
            .find(|(role, _)| *role == "run-challenge")
            .expect("D4A challenge role exists")
            .1,
        inputs.run_challenge.to_vec(),
    )?;

    let policy_identities = BTreeMap::from([
        (
            "nix-closure-manifest",
            (closure_json.digest.clone(), closure_json.size),
        ),
        (
            "repository-bundle",
            (inputs.seed.bundle_digest.clone(), inputs.seed.bundle_size),
        ),
        (
            "repository-bundle-manifest",
            (manifest_json.digest.clone(), manifest_json.size),
        ),
        (
            "run-challenge",
            (challenge.digest.clone(), challenge.size),
        ),
        (
            "sandbox-invocation",
            (invocation_json.digest.clone(), invocation_json.size),
        ),
    ]);
    let policy_mounts = POLICY_INPUTS
        .iter()
        .map(|(role, destination)| {
            let (digest, size) = policy_identities
                .get(role)
                .expect("D4A policy role has a deterministic builder identity");
            ArtifactMount::new(*role, *destination, digest.clone(), *size)
        })
        .collect::<Result<Vec<_>, _>>()?;

    let isolation_policy =
        LinuxIsolationPolicyV1::strict(inputs.closure, inputs.invocation, policy_mounts)?;
    isolation_policy.validate_dependencies(inputs.closure, inputs.invocation)?;
    require_exact_policy_surface(&isolation_policy)?;
    let isolation_policy_digest = isolation_policy.digest(DigestAlgorithm::Sha256)?;

    let execution_subject = derive_execution_subject_v6(
        &inputs.seed.request_digest,
        &inputs.seed.manifest_commitment,
        &inputs.seed.replay_receipt,
        &inputs.seed.policy_state,
        &isolation_policy_digest,
        &closure_digest,
        &invocation_digest,
        &trust_profile,
        &git_policy,
        &challenge_digest,
        &nar_auditor,
    )?;

    let guest_plan = GuestVerificationPlanV1::new(
        execution_subject.clone(),
        inputs.seed.request.clone(),
        inputs.seed.manifest_commitment.clone(),
        inputs.seed.policy_state.clone(),
        isolation_policy_digest.clone(),
        closure_digest,
        invocation_digest,
        trust_profile,
        challenge_digest.clone(),
        inputs.seed.replay_receipt.clone(),
    )?;

    let guest_tool_map = GuestToolMapV1::from_tool_artifacts(
        &guest_plan,
        inputs.closure,
        inputs.tools,
        inputs.guest_tool_paths,
    )?;

    let policy_json = generated_json(
        "linux-isolation-policy",
        CONTROL_INPUTS
            .iter()
            .find(|(role, _)| *role == "linux-isolation-policy")
            .expect("D4A policy control exists")
            .1,
        &isolation_policy,
    )?;
    let plan_json = generated_json(
        "guest-verification-plan",
        CONTROL_INPUTS
            .iter()
            .find(|(role, _)| *role == "guest-verification-plan")
            .expect("D4A plan control exists")
            .1,
        &guest_plan,
    )?;
    let tool_map_json = generated_json(
        "guest-tool-map",
        GUEST_TOOL_MAP_PATH,
        &guest_tool_map,
    )?;

    let execution_spec = ExecutionSpec::new(
        ExecutionPurpose::RepositoryVerification,
        execution_subject.clone(),
        inputs.tools.to_vec(),
        vec![],
        vec![
            InputArtifact::new(
                "repository-bundle",
                inputs.seed.bundle_digest.clone(),
                inputs.seed.bundle_size,
            )?,
            to_input(&manifest_json)?,
            to_input(&closure_json)?,
            to_input(&invocation_json)?,
            to_input(&challenge)?,
            to_input(&policy_json)?,
            to_input(&plan_json)?,
            to_input(&tool_map_json)?,
        ],
        execution_environment()?,
        NetworkPolicy::Denied,
        VerificationTimePolicy::NotUsed,
        FilesystemPolicy::hermetic(),
    )?;

    if execution_spec.subject() != &execution_subject {
        return Err(CapsuleBuilderError::ExecutionSubjectMismatch);
    }
    require_exact_spec_inputs(&execution_spec)?;
    qualify_guest_tool_map(
        &guest_tool_map,
        &guest_plan,
        inputs.closure,
        &execution_spec,
    )?;

    let execution_spec_digest = execution_spec.digest(DigestAlgorithm::Sha256)?;
    let run_plan = HermeticRunPlan::new(
        execution_spec_digest.clone(),
        isolation_policy_digest.clone(),
        guest_plan.git_object_validation_policy().clone(),
        challenge_digest,
        RUN_CHALLENGE_SIZE,
    )?;

    if run_plan.execution_spec() != &execution_spec_digest
        || run_plan.isolation_policy() != &isolation_policy_digest
        || run_plan.git_object_validation_policy() != guest_plan.git_object_validation_policy()
        || run_plan.run_challenge() != guest_plan.run_challenge()
    {
        return Err(CapsuleBuilderError::RunPlanCrossLinkMismatch);
    }

    let mut generated_artifacts = vec![
        manifest_json,
        closure_json,
        invocation_json,
        challenge,
        policy_json,
        plan_json,
        tool_map_json,
    ];
    generated_artifacts.sort_by(|a, b| a.role.cmp(&b.role));
    require_generated_artifacts_match_spec(&generated_artifacts, &execution_spec)?;

    let run_plan_digest = run_plan.digest(DigestAlgorithm::Sha256)?;
    let construction_commitment = derive_construction_commitment(
        &execution_subject,
        &isolation_policy_digest,
        &execution_spec_digest,
        &run_plan_digest,
        &inputs.seed.bundle_digest,
        inputs.seed.bundle_size,
        &generated_artifacts,
    )?;

    Ok(BuiltM0Capsule {
        execution_subject,
        isolation_policy,
        guest_plan,
        guest_tool_map,
        execution_spec,
        run_plan,
        generated_artifacts,
        construction_commitment,
    })
}

fn require_exact_tools(tools: &[ToolArtifact]) -> Result<(), CapsuleBuilderError> {
    let roles = tools.iter().map(ToolArtifact::role).collect::<BTreeSet<_>>();
    let expected = REQUIRED_TOOL_ROLES.iter().copied().collect::<BTreeSet<_>>();
    if roles != expected || roles.len() != tools.len() {
        return Err(CapsuleBuilderError::ToolRoleSetMismatch);
    }
    for (role, expected_version) in [
        ("forge-hermetic-host", HOST_VERSION),
        ("forge-hermetic-guest", GUEST_VERSION),
        ("forge-isolation-probe", GUEST_VERSION),
        ("gittuf", REQUIRED_GITTUF_VERSION),
    ] {
        let tool = tools
            .iter()
            .find(|tool| tool.role() == role)
            .ok_or_else(|| CapsuleBuilderError::MissingTool(role.to_owned()))?;
        if tool.semantic_version() != expected_version {
            return Err(CapsuleBuilderError::ToolVersionMismatch(role.to_owned()));
        }
    }
    Ok(())
}

fn require_exact_spec_inputs(spec: &ExecutionSpec) -> Result<(), CapsuleBuilderError> {
    let actual = spec
        .inputs()
        .iter()
        .map(InputArtifact::role)
        .collect::<BTreeSet<_>>();
    let expected = POLICY_INPUTS
        .iter()
        .chain(CONTROL_INPUTS.iter())
        .map(|(role, _)| *role)
        .collect::<BTreeSet<_>>();
    if actual != expected || actual.len() != spec.inputs().len() {
        return Err(CapsuleBuilderError::InputRoleSetMismatch);
    }
    Ok(())
}

fn require_exact_policy_surface(
    policy: &LinuxIsolationPolicyV1,
) -> Result<(), CapsuleBuilderError> {
    let actual = policy
        .artifact_mounts()
        .iter()
        .map(|mount| (mount.role(), mount.destination()))
        .collect::<BTreeSet<_>>();
    let expected = POLICY_INPUTS.iter().copied().collect::<BTreeSet<_>>();
    if actual != expected {
        return Err(CapsuleBuilderError::PolicySurfaceMismatch);
    }
    Ok(())
}

fn require_guest_invocation(invocation: &VerifierInvocation) -> Result<(), CapsuleBuilderError> {
    if !invocation.executable().ends_with("/bin/forge-hermetic-guest") {
        return Err(CapsuleBuilderError::GuestInvocationMismatch);
    }
    let expected = ["--plan", GUEST_PLAN_PATH];
    if invocation.args().len() != expected.len()
        || invocation
            .args()
            .iter()
            .map(String::as_str)
            .ne(expected.into_iter())
    {
        return Err(CapsuleBuilderError::GuestInvocationMismatch);
    }
    Ok(())
}

fn execution_environment() -> Result<Vec<EnvironmentBinding>, CapsuleBuilderError> {
    expected_environment()
        .into_iter()
        .map(|entry| {
            EnvironmentBinding::new(entry.key().to_owned(), entry.value().to_owned())
                .map_err(CapsuleBuilderError::from)
        })
        .collect()
}

fn generated_json<T: Serialize>(
    role: &str,
    destination: &str,
    value: &T,
) -> Result<GeneratedCapsuleArtifact, CapsuleBuilderError> {
    generated_bytes(role, destination, serde_json::to_vec(value)?)
}

fn generated_bytes(
    role: &str,
    destination: &str,
    bytes: Vec<u8>,
) -> Result<GeneratedCapsuleArtifact, CapsuleBuilderError> {
    if bytes.is_empty() {
        return Err(CapsuleBuilderError::EmptyGeneratedArtifact(role.to_owned()));
    }
    let size = u64::try_from(bytes.len())
        .map_err(|_| CapsuleBuilderError::ArtifactTooLarge(role.to_owned()))?;
    Ok(GeneratedCapsuleArtifact {
        role: role.to_owned(),
        destination: destination.to_owned(),
        digest: Digest::of_bytes(DigestAlgorithm::Sha256, &bytes),
        size,
        bytes,
    })
}

fn to_input(artifact: &GeneratedCapsuleArtifact) -> Result<InputArtifact, CapsuleBuilderError> {
    Ok(InputArtifact::new(
        artifact.role.clone(),
        artifact.digest.clone(),
        artifact.size,
    )?)
}

fn require_generated_artifacts_match_spec(
    artifacts: &[GeneratedCapsuleArtifact],
    spec: &ExecutionSpec,
) -> Result<(), CapsuleBuilderError> {
    for artifact in artifacts {
        let input = spec
            .inputs()
            .iter()
            .find(|input| input.role() == artifact.role())
            .ok_or_else(|| CapsuleBuilderError::MissingGeneratedInput(artifact.role.clone()))?;
        if input.digest() != artifact.digest() || input.size() != artifact.size() {
            return Err(CapsuleBuilderError::GeneratedInputMismatch(
                artifact.role.clone(),
            ));
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn derive_construction_commitment(
    execution_subject: &Digest,
    isolation_policy: &Digest,
    execution_spec: &Digest,
    run_plan: &Digest,
    bundle_digest: &Digest,
    bundle_size: u64,
    generated: &[GeneratedCapsuleArtifact],
) -> Result<Digest, CapsuleBuilderError> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(CONSTRUCTION_DOMAIN_V1);
    for digest in [
        execution_subject,
        isolation_policy,
        execution_spec,
        run_plan,
        bundle_digest,
    ] {
        push_digest(&mut bytes, digest)?;
    }
    bytes.extend_from_slice(&bundle_size.to_be_bytes());
    let count = u16::try_from(generated.len())
        .map_err(|_| CapsuleBuilderError::CanonicalLengthOverflow)?;
    bytes.extend_from_slice(&count.to_be_bytes());
    for artifact in generated {
        push_string(&mut bytes, artifact.role())?;
        push_string(&mut bytes, artifact.destination())?;
        push_digest(&mut bytes, artifact.digest())?;
        bytes.extend_from_slice(&artifact.size().to_be_bytes());
    }
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &bytes))
}

fn push_string(out: &mut Vec<u8>, value: &str) -> Result<(), CapsuleBuilderError> {
    let len = u16::try_from(value.len()).map_err(|_| CapsuleBuilderError::CanonicalLengthOverflow)?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), CapsuleBuilderError> {
    push_string(out, digest.algorithm().id())?;
    let len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| CapsuleBuilderError::CanonicalLengthOverflow)?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

#[derive(Debug, Error)]
pub enum CapsuleBuilderError {
    #[error(transparent)]
    Core(#[from] mycelix_forge_core::ForgeCoreError),
    #[error(transparent)]
    Execution(#[from] ExecutionContractError),
    #[error(transparent)]
    Repository(#[from] RepositoryVerificationError),
    #[error(transparent)]
    Adapter(#[from] AdapterError),
    #[error(transparent)]
    Manifest(#[from] OfflineManifestError),
    #[error(transparent)]
    GuestPlan(#[from] GuestPlanError),
    #[error(transparent)]
    GuestToolMap(#[from] GuestToolMapError),
    #[error(transparent)]
    Isolation(#[from] IsolationPolicyError),
    #[error(transparent)]
    HermeticRun(#[from] HermeticRunError),
    #[error(transparent)]
    Composer(#[from] M0CompositionV6Error),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error("portable replay receipt differs from bundle manifest")]
    ReplayReceiptManifestMismatch,
    #[error("tool-role set differs from exact M0 six-tool set")]
    ToolRoleSetMismatch,
    #[error("final ExecutionSpec input-role set differs from D4A 5+3 input surface")]
    InputRoleSetMismatch,
    #[error("constructed Linux isolation policy differs from D4A five-input surface")]
    PolicySurfaceMismatch,
    #[error("missing required tool: {0}")]
    MissingTool(String),
    #[error("tool semantic version mismatch: {0}")]
    ToolVersionMismatch(String),
    #[error("sandbox invocation is not the exact guest plan invocation")]
    GuestInvocationMismatch,
    #[error("generated artifact is empty: {0}")]
    EmptyGeneratedArtifact(String),
    #[error("generated artifact is too large: {0}")]
    ArtifactTooLarge(String),
    #[error("final ExecutionSpec names a different execution subject")]
    ExecutionSubjectMismatch,
    #[error("run plan does not cross-link final spec/policy/guest plan")]
    RunPlanCrossLinkMismatch,
    #[error("final ExecutionSpec is missing generated artifact: {0}")]
    MissingGeneratedInput(String),
    #[error("final ExecutionSpec fingerprint differs from generated artifact: {0}")]
    GeneratedInputMismatch(String),
    #[error("canonical construction field length overflow")]
    CanonicalLengthOverflow,
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_core::{
        ProjectIdentity, ProjectIdentitySeed, GENESIS_NONCE_LEN,
    };
    use mycelix_forge_linux_isolation::NixClosureEntry;
    use mycelix_forge_repository::{
        GitObjectAlgorithm, GitObjectId, RepositoryAdoption, RepositoryPolicyState, RepositoryRef,
        RepositoryTip,
    };

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn git_sha1(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn project() -> ProjectIdentity {
        ProjectIdentity::derive(
            &ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], digest(0x12)),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn request() -> RepositoryVerificationRequest {
        let project = project();
        let adoption = RepositoryAdoption::new(
            project.clone(),
            RepositoryTip::new(
                RepositoryRef::new("refs/heads/main").unwrap(),
                git_sha1(0x21),
            ),
            digest(0x22),
            digest(0x23),
            digest(0x24),
            1_000,
        );
        let policy = RepositoryPolicyState::new(project, 0, None, digest(0x24)).unwrap();
        RepositoryVerificationRequest::new(
            &adoption,
            git_sha1(0x21),
            git_sha1(0x25),
            digest(0x22),
            digest(0x23),
            &policy,
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn seed() -> M0CapsuleSeed {
        let request = request();
        M0CapsuleSeed {
            request_digest: request.digest(DigestAlgorithm::Sha256).unwrap(),
            request,
            manifest_artifact: generated_bytes(
                "repository-bundle-manifest",
                GUEST_MANIFEST_PATH,
                br#"{\"fixture\":\"manifest\"}"#.to_vec(),
            )
            .unwrap(),
            manifest_commitment: digest(0x31),
            replay_receipt: digest(0x32),
            policy_state: digest(0x33),
            bundle_digest: digest(0x34),
            bundle_size: 4096,
        }
    }

    fn closure() -> NixClosureManifest {
        NixClosureManifest::new(vec![
            NixClosureEntry::new("/nix/store/aaaa-guest", digest(0x41)).unwrap(),
            NixClosureEntry::new("/nix/store/bbbb-probe", digest(0x42)).unwrap(),
            NixClosureEntry::new("/nix/store/cccc-git", digest(0x43)).unwrap(),
            NixClosureEntry::new("/nix/store/dddd-gittuf", digest(0x44)).unwrap(),
        ])
        .unwrap()
    }

    fn invocation(closure: &NixClosureManifest) -> VerifierInvocation {
        VerifierInvocation::new(
            "/nix/store/aaaa-guest/bin/forge-hermetic-guest",
            vec!["--plan".to_owned(), GUEST_PLAN_PATH.to_owned()],
            closure,
        )
        .unwrap()
    }

    fn tools() -> Vec<ToolArtifact> {
        vec![
            ToolArtifact::new("bubblewrap", "0.10.0", digest(0x51), 1, None).unwrap(),
            ToolArtifact::new(
                "forge-hermetic-guest",
                GUEST_VERSION,
                digest(0x52),
                1,
                None,
            )
            .unwrap(),
            ToolArtifact::new(
                "forge-hermetic-host",
                HOST_VERSION,
                digest(0x53),
                1,
                None,
            )
            .unwrap(),
            ToolArtifact::new(
                "forge-isolation-probe",
                GUEST_VERSION,
                digest(0x54),
                1,
                None,
            )
            .unwrap(),
            ToolArtifact::new("git", "2.49.0", digest(0x55), 1, None).unwrap(),
            ToolArtifact::new(
                "gittuf",
                REQUIRED_GITTUF_VERSION,
                digest(0x56),
                1,
                None,
            )
            .unwrap(),
        ]
    }

    fn guest_paths() -> BTreeMap<String, String> {
        BTreeMap::from([
            (
                "forge-isolation-probe".to_owned(),
                "/nix/store/bbbb-probe/bin/forge-isolation-probe".to_owned(),
            ),
            (
                "git".to_owned(),
                "/nix/store/cccc-git/bin/git".to_owned(),
            ),
            (
                "gittuf".to_owned(),
                "/nix/store/dddd-gittuf/bin/gittuf".to_owned(),
            ),
        ])
    }

    #[test]
    fn exact_tool_role_surface_is_six() {
        assert_eq!(REQUIRED_TOOL_ROLES.len(), 6);
        assert!(REQUIRED_TOOL_ROLES.contains(&"forge-hermetic-host"));
        assert!(REQUIRED_TOOL_ROLES.contains(&"gittuf"));
    }

    #[test]
    fn d4a_input_surface_is_five_plus_three() {
        assert_eq!(POLICY_INPUTS.len(), 5);
        assert_eq!(CONTROL_INPUTS.len(), 3);
        let roles = POLICY_INPUTS
            .iter()
            .chain(CONTROL_INPUTS.iter())
            .map(|(role, _)| *role)
            .collect::<BTreeSet<_>>();
        assert_eq!(roles.len(), 8);
    }

    #[test]
    fn d4a_destinations_are_globally_distinct() {
        let destinations = POLICY_INPUTS
            .iter()
            .chain(CONTROL_INPUTS.iter())
            .map(|(_, destination)| *destination)
            .collect::<BTreeSet<_>>();
        assert_eq!(destinations.len(), 8);
    }

    #[test]
    fn construction_is_byte_deterministic_for_identical_seed() {
        let seed = seed();
        let closure = closure();
        let invocation = invocation(&closure);
        let tools = tools();
        let build = |paths| {
            build_m0_capsule_from_seed(M0CapsuleSeedInputs {
                seed: &seed,
                closure: &closure,
                invocation: &invocation,
                tools: &tools,
                guest_tool_paths: paths,
                run_challenge: [0xA5; RUN_CHALLENGE_SIZE as usize],
            })
            .unwrap()
        };

        let first = build(guest_paths());
        let second = build(guest_paths());

        assert_eq!(first.execution_subject(), second.execution_subject());
        assert_eq!(
            first
                .isolation_policy()
                .digest(DigestAlgorithm::Sha256)
                .unwrap(),
            second
                .isolation_policy()
                .digest(DigestAlgorithm::Sha256)
                .unwrap()
        );
        assert_eq!(
            first
                .execution_spec()
                .digest(DigestAlgorithm::Sha256)
                .unwrap(),
            second
                .execution_spec()
                .digest(DigestAlgorithm::Sha256)
                .unwrap()
        );
        assert_eq!(
            first.run_plan().digest(DigestAlgorithm::Sha256).unwrap(),
            second.run_plan().digest(DigestAlgorithm::Sha256).unwrap()
        );
        assert_eq!(
            first.construction_commitment(),
            second.construction_commitment()
        );
        assert_eq!(first.generated_artifacts(), second.generated_artifacts());
    }
}
