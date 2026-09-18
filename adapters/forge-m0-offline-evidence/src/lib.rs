// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-004D3A: exact M0 composition gate for hermetic offline evidence.
//!
//! Individually valid receipts are insufficient. This crate re-derives every
//! cross-link among one portable replay, one execution subject/spec/result,
//! one strict Linux isolation policy, one runtime NAR closure, one local-key
//! trust profile, one same-run transcript, and one self-contained guest plan
//! before minting repository-level `OfflineEvidence`.

use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_execution::{
    EvidenceBoundExecution, ExecutionContractError, ExecutionSpec, VerificationTimePolicy,
};
use mycelix_forge_gittuf_adapter::AdapterError;
use mycelix_forge_gittuf_bundle::{OfflineManifestError, PortableRepositoryReplay};
use mycelix_forge_gittuf_trust_profile::{
    local_embedded_keys_v1, QualifiedLocalKeyTrustProfile,
};
use mycelix_forge_guest_plan::{
    GuestPlanError, GuestVerificationPlanV1, GUEST_BUNDLE_PATH, GUEST_ISOLATION_POLICY_PATH,
    GUEST_MANIFEST_PATH, GUEST_NIX_CLOSURE_PATH, GUEST_PLAN_PATH, GUEST_RUN_CHALLENGE_PATH,
    GUEST_SANDBOX_INVOCATION_PATH,
};
use mycelix_forge_hermetic_run_evidence::{
    HermeticRunError, HermeticRunPlan, QualifiedHermeticRunEvidence, RUN_CHALLENGE_SIZE,
};
use mycelix_forge_linux_isolation::{
    IsolationPolicyError, LinuxIsolationPolicyV1, NixClosureManifest, VerifierInvocation,
};
use mycelix_forge_linux_isolation_evidence::{
    expected_environment, QualifiedIsolationEvidence,
};
use mycelix_forge_repository::{
    qualify_evidence_backed_observation, AdapterIdentity, AdapterObservation, AdapterOutcome,
    EvidenceBackedAdapterObservation, QualifiedRepositoryVerification, RepositoryEvidenceError,
    RepositoryVerificationError, RepositoryVerificationRequest, VerificationCapability,
    VerificationProfile,
};
use mycelix_forge_runtime_closure_evidence::QualifiedRuntimeClosureEvidence;
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

const EXECUTION_SUBJECT_DOMAIN_V3: &[u8] = b"mycelix-forge/m0-hermetic-execution-subject/v3\0";
const CAPSULE_ARTIFACTS_DOMAIN_V1: &[u8] = b"mycelix-forge/m0-guest-capsule-artifacts/v1\0";
const RUNTIME_EVIDENCE_DOMAIN_V2: &[u8] = b"mycelix-forge/m0-runtime-enforcement-evidence/v2\0";
const OFFLINE_EVIDENCE_DOMAIN_V4: &[u8] = b"mycelix-forge/m0-offline-evidence/v4\0";
const COMPOSER_NAME: &str = "mycelix-forge-m0";
const COMPOSER_VERSION: &str = "0.4.0-gittuf-0.16.0-local-keys";

const REQUIRED_TOOL_ROLES: &[&str] = &[
    "bubblewrap",
    "forge-hermetic-guest",
    "forge-hermetic-host",
    "forge-isolation-probe",
    "git",
    "gittuf",
    "nar-auditor",
];

const REQUIRED_INPUT_ROLES: &[&str] = &[
    "guest-verification-plan",
    "linux-isolation-policy",
    "nix-closure-manifest",
    "repository-bundle",
    "repository-bundle-manifest",
    "run-challenge",
    "sandbox-invocation",
];

const GUEST_EXECUTABLE_SUFFIX: &str = "/bin/forge-hermetic-guest";
const GUEST_PLAN_FLAG: &str = "--plan";

pub struct M0CompositionInputs<'a> {
    pub request: &'a RepositoryVerificationRequest,
    pub replay: &'a PortableRepositoryReplay,
    pub guest_plan: &'a GuestVerificationPlanV1,
    pub execution_spec: &'a ExecutionSpec,
    pub execution: &'a EvidenceBoundExecution,
    pub isolation_policy: &'a LinuxIsolationPolicyV1,
    pub closure: &'a NixClosureManifest,
    /// Exact bubblewrap child invocation. The child is the Forge guest runner,
    /// not gittuf directly; the guest derives gittuf invocation from `request`.
    pub invocation: &'a VerifierInvocation,
    pub isolation: &'a QualifiedIsolationEvidence,
    pub runtime_closure: &'a QualifiedRuntimeClosureEvidence,
    pub trust: &'a QualifiedLocalKeyTrustProfile,
    pub run_plan: &'a HermeticRunPlan,
    pub run: &'a QualifiedHermeticRunEvidence,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualifiedM0OfflineEvidence {
    repository: QualifiedRepositoryVerification,
    execution_subject: Digest,
    guest_plan: Digest,
    capsule_artifacts: Digest,
    runtime_evidence: Digest,
    evidence_commitment: Digest,
}

impl QualifiedM0OfflineEvidence {
    pub fn repository(&self) -> &QualifiedRepositoryVerification {
        &self.repository
    }

    pub fn execution_subject(&self) -> &Digest {
        &self.execution_subject
    }

    pub fn guest_plan(&self) -> &Digest {
        &self.guest_plan
    }

    pub fn capsule_artifacts(&self) -> &Digest {
        &self.capsule_artifacts
    }

    pub fn runtime_evidence(&self) -> &Digest {
        &self.runtime_evidence
    }

    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

pub fn qualify_m0_offline_evidence(
    inputs: M0CompositionInputs<'_>,
) -> Result<QualifiedM0OfflineEvidence, M0CompositionError> {
    inputs.replay.manifest().validate_for_request(inputs.request)?;
    inputs
        .isolation_policy
        .validate_dependencies(inputs.closure, inputs.invocation)?;
    require_guest_runner_invocation(inputs.invocation)?;

    if inputs.execution_spec.clock() != &VerificationTimePolicy::NotUsed {
        return Err(M0CompositionError::LocalKeyProfileRequiresTimelessVerification);
    }
    if !inputs.execution_spec.is_hermetic_candidate() {
        return Err(M0CompositionError::ExecutionSpecNotHermeticCandidate);
    }
    require_exact_tool_roles(inputs.execution_spec)?;
    require_exact_input_roles(inputs.execution_spec)?;
    require_exact_mount_roles(inputs.isolation_policy)?;
    require_exact_environment(inputs.execution_spec)?;
    if !inputs.execution_spec.trust_material().is_empty() {
        return Err(M0CompositionError::UnexpectedExternalTrustMaterial);
    }

    let request_digest = inputs.request.digest(DigestAlgorithm::Sha256)?;
    let manifest_commitment = inputs
        .replay
        .manifest()
        .commitment(DigestAlgorithm::Sha256)?;
    let replay_receipt = inputs
        .replay
        .replay_receipt()
        .commitment(DigestAlgorithm::Sha256)?;
    let policy_state = inputs
        .replay
        .manifest()
        .policy_state()
        .digest(DigestAlgorithm::Sha256)?;

    let local_profile = local_embedded_keys_v1();
    if inputs.trust.policy_state() != &policy_state {
        return Err(M0CompositionError::TrustPolicyStateMismatch);
    }
    if inputs.trust.profile_digest() != local_profile.profile_digest() {
        return Err(M0CompositionError::TrustProfileMismatch);
    }

    let isolation_policy = inputs
        .isolation_policy
        .digest(DigestAlgorithm::Sha256)?;
    if inputs.isolation.policy_digest() != &isolation_policy {
        return Err(M0CompositionError::IsolationPolicyMismatch);
    }
    let closure = inputs.closure.digest(DigestAlgorithm::Sha256)?;
    if inputs.runtime_closure.closure_digest() != &closure {
        return Err(M0CompositionError::RuntimeClosureMismatch);
    }
    let invocation = inputs.invocation.digest(DigestAlgorithm::Sha256)?;

    let execution_subject = derive_execution_subject(
        &request_digest,
        &manifest_commitment,
        &replay_receipt,
        &policy_state,
        &isolation_policy,
        &closure,
        &invocation,
        inputs.runtime_closure.auditor_subject(),
        inputs.trust.profile_digest(),
        inputs.trust.inventory_digest(),
        inputs.guest_plan.git_object_validation_policy(),
        inputs.run_plan.run_challenge(),
    )?;
    if inputs.execution_spec.subject() != &execution_subject {
        return Err(M0CompositionError::ExecutionSubjectMismatch);
    }

    require_guest_plan_links(
        inputs.guest_plan,
        &execution_subject,
        &request_digest,
        &manifest_commitment,
        &policy_state,
        &isolation_policy,
        &closure,
        &invocation,
        inputs.trust.profile_digest(),
        inputs.run_plan.run_challenge(),
        &replay_receipt,
        inputs.run_plan.git_object_validation_policy(),
    )?;

    let manifest_artifact = json_artifact(inputs.replay.manifest(), "bundle manifest JSON")?;
    let guest_plan_artifact = json_artifact(inputs.guest_plan, "guest plan JSON")?;
    let isolation_policy_artifact =
        json_artifact(inputs.isolation_policy, "isolation policy JSON")?;
    let closure_artifact = json_artifact(inputs.closure, "Nix closure JSON")?;
    let invocation_artifact = json_artifact(inputs.invocation, "sandbox invocation JSON")?;

    require_input_artifact(
        inputs.execution_spec,
        "repository-bundle",
        inputs.replay.manifest().bundle_digest(),
        inputs.replay.manifest().bundle_size(),
    )?;
    require_input_artifact(
        inputs.execution_spec,
        "repository-bundle-manifest",
        &manifest_artifact.digest,
        manifest_artifact.size,
    )?;
    require_input_artifact(
        inputs.execution_spec,
        "guest-verification-plan",
        &guest_plan_artifact.digest,
        guest_plan_artifact.size,
    )?;
    require_input_artifact(
        inputs.execution_spec,
        "linux-isolation-policy",
        &isolation_policy_artifact.digest,
        isolation_policy_artifact.size,
    )?;
    require_input_artifact(
        inputs.execution_spec,
        "nix-closure-manifest",
        &closure_artifact.digest,
        closure_artifact.size,
    )?;
    require_input_artifact(
        inputs.execution_spec,
        "sandbox-invocation",
        &invocation_artifact.digest,
        invocation_artifact.size,
    )?;
    require_input_artifact(
        inputs.execution_spec,
        "run-challenge",
        inputs.run_plan.run_challenge(),
        RUN_CHALLENGE_SIZE,
    )?;

    require_policy_artifact(
        inputs.isolation_policy,
        "repository-bundle",
        GUEST_BUNDLE_PATH,
        inputs.replay.manifest().bundle_digest(),
        inputs.replay.manifest().bundle_size(),
    )?;
    require_policy_artifact(
        inputs.isolation_policy,
        "repository-bundle-manifest",
        GUEST_MANIFEST_PATH,
        &manifest_artifact.digest,
        manifest_artifact.size,
    )?;
    require_policy_artifact(
        inputs.isolation_policy,
        "guest-verification-plan",
        GUEST_PLAN_PATH,
        &guest_plan_artifact.digest,
        guest_plan_artifact.size,
    )?;
    require_policy_artifact(
        inputs.isolation_policy,
        "linux-isolation-policy",
        GUEST_ISOLATION_POLICY_PATH,
        &isolation_policy_artifact.digest,
        isolation_policy_artifact.size,
    )?;
    require_policy_artifact(
        inputs.isolation_policy,
        "nix-closure-manifest",
        GUEST_NIX_CLOSURE_PATH,
        &closure_artifact.digest,
        closure_artifact.size,
    )?;
    require_policy_artifact(
        inputs.isolation_policy,
        "sandbox-invocation",
        GUEST_SANDBOX_INVOCATION_PATH,
        &invocation_artifact.digest,
        invocation_artifact.size,
    )?;
    require_policy_artifact(
        inputs.isolation_policy,
        "run-challenge",
        GUEST_RUN_CHALLENGE_PATH,
        inputs.run_plan.run_challenge(),
        RUN_CHALLENGE_SIZE,
    )?;

    let capsule_artifacts = derive_capsule_artifacts(
        inputs.replay.manifest().bundle_digest(),
        &manifest_artifact.digest,
        &guest_plan_artifact.digest,
        &isolation_policy_artifact.digest,
        &closure_artifact.digest,
        &invocation_artifact.digest,
        inputs.run_plan.run_challenge(),
    )?;

    let execution_spec_digest = inputs.execution_spec.digest(DigestAlgorithm::Sha256)?;
    if inputs.execution.spec_digest() != &execution_spec_digest {
        return Err(M0CompositionError::ExecutionSpecMismatch);
    }
    if inputs.run_plan.execution_spec() != &execution_spec_digest {
        return Err(M0CompositionError::RunPlanExecutionSpecMismatch);
    }
    if inputs.run_plan.isolation_policy() != &isolation_policy {
        return Err(M0CompositionError::RunPlanIsolationPolicyMismatch);
    }
    if inputs.run_plan.git_object_validation_policy()
        != inputs.guest_plan.git_object_validation_policy()
    {
        return Err(M0CompositionError::RunPlanGitPolicyMismatch);
    }
    if inputs.run_plan.run_challenge() != inputs.guest_plan.run_challenge() {
        return Err(M0CompositionError::RunChallengeMismatch);
    }

    let run_plan_digest = inputs.run_plan.digest(DigestAlgorithm::Sha256)?;
    if inputs.run.plan_digest() != &run_plan_digest {
        return Err(M0CompositionError::RunPlanDigestMismatch);
    }
    if inputs.run.run_challenge() != inputs.run_plan.run_challenge() {
        return Err(M0CompositionError::RunChallengeMismatch);
    }
    if inputs.run.pre_runtime_closure() != inputs.runtime_closure.evidence_digest()
        || inputs.run.post_runtime_closure() != inputs.runtime_closure.evidence_digest()
    {
        return Err(M0CompositionError::RunRuntimeClosureMismatch);
    }
    if inputs.run.policy_trust() != inputs.trust.evidence_digest() {
        return Err(M0CompositionError::RunTrustEvidenceMismatch);
    }
    if inputs.run.isolation() != inputs.isolation.evidence_digest() {
        return Err(M0CompositionError::RunIsolationEvidenceMismatch);
    }
    if inputs.run.verifier_output() != &replay_receipt {
        return Err(M0CompositionError::RunVerifierOutputMismatch);
    }

    if inputs.execution.output() != &replay_receipt {
        return Err(M0CompositionError::HermeticOutputMismatch);
    }
    let runtime_evidence = derive_runtime_evidence(
        inputs.isolation.evidence_digest(),
        inputs.runtime_closure.evidence_digest(),
        inputs.run.evidence_digest(),
    )?;
    if inputs.execution.execution_evidence() != &runtime_evidence {
        return Err(M0CompositionError::RuntimeEvidenceMismatch);
    }

    let guest_plan_commitment = inputs.guest_plan.digest(DigestAlgorithm::Sha256)?;
    let evidence_commitment = derive_offline_evidence_commitment(
        &request_digest,
        &manifest_commitment,
        &replay_receipt,
        &guest_plan_commitment,
        &capsule_artifacts,
        &execution_subject,
        &execution_spec_digest,
        &runtime_evidence,
        inputs.run.evidence_digest(),
        inputs.isolation.evidence_digest(),
        inputs.runtime_closure.evidence_digest(),
        inputs.trust.evidence_digest(),
    )?;

    let adapter_observation = AdapterObservation::new(
        AdapterIdentity::new(COMPOSER_NAME, COMPOSER_VERSION)?,
        request_digest,
        inputs.request.to().clone(),
        inputs.request.repository_policy_state().clone(),
        Some(inputs.replay.replay_receipt().history_commitment().clone()),
        Some(evidence_commitment.clone()),
        [
            VerificationCapability::RefTipBinding,
            VerificationCapability::FullHistory,
            VerificationCapability::ProtectedRewriteDetection,
            VerificationCapability::PolicyLineageMonotonic,
            VerificationCapability::OfflineEvidence,
        ],
        AdapterOutcome::Verified,
    )?;
    let backed = EvidenceBackedAdapterObservation::new(
        adapter_observation,
        Some(
            inputs
                .replay
                .replay_receipt()
                .policy_lineage_commitment()
                .clone(),
        ),
    )?;
    let repository = qualify_evidence_backed_observation(
        &VerificationProfile::m0_protected_source(),
        inputs.request,
        backed,
    )?;

    Ok(QualifiedM0OfflineEvidence {
        repository,
        execution_subject,
        guest_plan: guest_plan_commitment,
        capsule_artifacts,
        runtime_evidence,
        evidence_commitment,
    })
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct JsonArtifact {
    digest: Digest,
    size: u64,
}

fn json_artifact<T: serde::Serialize>(
    value: &T,
    field: &'static str,
) -> Result<JsonArtifact, M0CompositionError> {
    let bytes = serde_json::to_vec(value)?;
    Ok(JsonArtifact {
        digest: Digest::of_bytes(DigestAlgorithm::Sha256, &bytes),
        size: u64::try_from(bytes.len())
            .map_err(|_| M0CompositionError::CanonicalLengthOverflow(field))?,
    })
}

#[allow(clippy::too_many_arguments)]
pub fn derive_execution_subject(
    request: &Digest,
    manifest: &Digest,
    expected_replay_receipt: &Digest,
    policy_state: &Digest,
    isolation_policy: &Digest,
    closure: &Digest,
    invocation: &Digest,
    nar_auditor_subject: &Digest,
    trust_profile: &Digest,
    trust_inventory: &Digest,
    git_object_validation_policy: &Digest,
    run_challenge: &Digest,
) -> Result<Digest, M0CompositionError> {
    let mut out = Vec::new();
    out.extend_from_slice(EXECUTION_SUBJECT_DOMAIN_V3);
    for digest in [
        request,
        manifest,
        expected_replay_receipt,
        policy_state,
        isolation_policy,
        closure,
        invocation,
        nar_auditor_subject,
        trust_profile,
        trust_inventory,
        git_object_validation_policy,
        run_challenge,
    ] {
        push_digest(&mut out, digest)?;
    }
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &out))
}

#[allow(clippy::too_many_arguments)]
fn derive_capsule_artifacts(
    bundle: &Digest,
    manifest_json: &Digest,
    guest_plan_json: &Digest,
    isolation_policy_json: &Digest,
    closure_json: &Digest,
    invocation_json: &Digest,
    run_challenge: &Digest,
) -> Result<Digest, M0CompositionError> {
    let mut out = Vec::new();
    out.extend_from_slice(CAPSULE_ARTIFACTS_DOMAIN_V1);
    for digest in [
        bundle,
        manifest_json,
        guest_plan_json,
        isolation_policy_json,
        closure_json,
        invocation_json,
        run_challenge,
    ] {
        push_digest(&mut out, digest)?;
    }
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &out))
}

pub fn derive_runtime_evidence(
    isolation: &Digest,
    runtime_closure: &Digest,
    same_run: &Digest,
) -> Result<Digest, M0CompositionError> {
    let mut out = Vec::new();
    out.extend_from_slice(RUNTIME_EVIDENCE_DOMAIN_V2);
    push_digest(&mut out, isolation)?;
    push_digest(&mut out, runtime_closure)?;
    push_digest(&mut out, same_run)?;
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &out))
}

#[allow(clippy::too_many_arguments)]
fn derive_offline_evidence_commitment(
    request: &Digest,
    manifest: &Digest,
    replay_receipt: &Digest,
    guest_plan: &Digest,
    capsule_artifacts: &Digest,
    execution_subject: &Digest,
    execution_spec: &Digest,
    runtime_evidence: &Digest,
    run_evidence: &Digest,
    isolation_evidence: &Digest,
    runtime_closure_evidence: &Digest,
    trust_evidence: &Digest,
) -> Result<Digest, M0CompositionError> {
    let mut out = Vec::new();
    out.extend_from_slice(OFFLINE_EVIDENCE_DOMAIN_V4);
    for digest in [
        request,
        manifest,
        replay_receipt,
        guest_plan,
        capsule_artifacts,
        execution_subject,
        execution_spec,
        runtime_evidence,
        run_evidence,
        isolation_evidence,
        runtime_closure_evidence,
        trust_evidence,
    ] {
        push_digest(&mut out, digest)?;
    }
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &out))
}

#[allow(clippy::too_many_arguments)]
fn require_guest_plan_links(
    plan: &GuestVerificationPlanV1,
    execution_subject: &Digest,
    request: &Digest,
    manifest: &Digest,
    policy_state: &Digest,
    isolation_policy: &Digest,
    closure: &Digest,
    invocation: &Digest,
    trust_profile: &Digest,
    run_challenge: &Digest,
    replay_receipt: &Digest,
    git_object_validation_policy: &Digest,
) -> Result<(), M0CompositionError> {
    if plan.execution_subject() != execution_subject {
        return Err(M0CompositionError::GuestPlanExecutionSubjectMismatch);
    }
    if &plan.request_digest()? != request {
        return Err(M0CompositionError::GuestPlanRequestMismatch);
    }
    if plan.bundle_manifest() != manifest {
        return Err(M0CompositionError::GuestPlanManifestMismatch);
    }
    if plan.policy_state() != policy_state {
        return Err(M0CompositionError::GuestPlanPolicyStateMismatch);
    }
    if plan.isolation_policy() != isolation_policy {
        return Err(M0CompositionError::GuestPlanIsolationPolicyMismatch);
    }
    if plan.nix_closure() != closure {
        return Err(M0CompositionError::GuestPlanClosureMismatch);
    }
    if plan.sandbox_invocation() != invocation {
        return Err(M0CompositionError::GuestPlanInvocationMismatch);
    }
    if plan.trust_profile() != trust_profile {
        return Err(M0CompositionError::GuestPlanTrustProfileMismatch);
    }
    if plan.run_challenge() != run_challenge {
        return Err(M0CompositionError::GuestPlanChallengeMismatch);
    }
    if plan.expected_replay_receipt() != replay_receipt {
        return Err(M0CompositionError::GuestPlanReplayReceiptMismatch);
    }
    if plan.git_object_validation_policy() != git_object_validation_policy {
        return Err(M0CompositionError::GuestPlanGitPolicyMismatch);
    }
    Ok(())
}

fn require_guest_runner_invocation(invocation: &VerifierInvocation) -> Result<(), M0CompositionError> {
    if !invocation.executable().ends_with(GUEST_EXECUTABLE_SUFFIX) {
        return Err(M0CompositionError::SandboxInvocationExecutableMismatch);
    }
    let expected = [GUEST_PLAN_FLAG, GUEST_PLAN_PATH];
    if invocation.args().len() != expected.len()
        || invocation
            .args()
            .iter()
            .map(String::as_str)
            .ne(expected.into_iter())
    {
        return Err(M0CompositionError::SandboxInvocationArgumentsMismatch);
    }
    Ok(())
}

fn require_exact_tool_roles(spec: &ExecutionSpec) -> Result<(), M0CompositionError> {
    let actual = spec
        .tools()
        .iter()
        .map(|tool| tool.role())
        .collect::<BTreeSet<_>>();
    let expected = REQUIRED_TOOL_ROLES.iter().copied().collect::<BTreeSet<_>>();
    if actual != expected {
        return Err(M0CompositionError::ToolRoleSetMismatch);
    }
    Ok(())
}

fn require_exact_input_roles(spec: &ExecutionSpec) -> Result<(), M0CompositionError> {
    let actual = spec
        .inputs()
        .iter()
        .map(|input| input.role())
        .collect::<BTreeSet<_>>();
    let expected = REQUIRED_INPUT_ROLES.iter().copied().collect::<BTreeSet<_>>();
    if actual != expected {
        return Err(M0CompositionError::InputRoleSetMismatch);
    }
    Ok(())
}

fn require_exact_mount_roles(policy: &LinuxIsolationPolicyV1) -> Result<(), M0CompositionError> {
    let actual = policy
        .artifact_mounts()
        .iter()
        .map(|mount| mount.role())
        .collect::<BTreeSet<_>>();
    let expected = REQUIRED_INPUT_ROLES.iter().copied().collect::<BTreeSet<_>>();
    if actual != expected {
        return Err(M0CompositionError::PolicyMountRoleSetMismatch);
    }
    Ok(())
}

fn require_exact_environment(spec: &ExecutionSpec) -> Result<(), M0CompositionError> {
    let actual = spec
        .environment()
        .iter()
        .map(|entry| (entry.key().to_owned(), entry.value().to_owned()))
        .collect::<BTreeMap<_, _>>();
    let expected = expected_environment()
        .into_iter()
        .map(|entry| (entry.key().to_owned(), entry.value().to_owned()))
        .collect::<BTreeMap<_, _>>();
    if actual != expected {
        return Err(M0CompositionError::ExecutionEnvironmentMismatch);
    }
    Ok(())
}

fn require_input_artifact(
    spec: &ExecutionSpec,
    role: &str,
    expected_digest: &Digest,
    expected_size: u64,
) -> Result<(), M0CompositionError> {
    let Some(input) = spec.inputs().iter().find(|input| input.role() == role) else {
        return Err(M0CompositionError::MissingRequiredInput(role.to_owned()));
    };
    if input.digest() != expected_digest || input.size() != expected_size {
        return Err(M0CompositionError::InputArtifactMismatch(role.to_owned()));
    }
    Ok(())
}

fn require_policy_artifact(
    policy: &LinuxIsolationPolicyV1,
    role: &str,
    expected_destination: &str,
    expected_digest: &Digest,
    expected_size: u64,
) -> Result<(), M0CompositionError> {
    let Some(mount) = policy
        .artifact_mounts()
        .iter()
        .find(|mount| mount.role() == role)
    else {
        return Err(M0CompositionError::MissingPolicyArtifact(role.to_owned()));
    };
    if mount.destination() != expected_destination
        || mount.digest() != expected_digest
        || mount.size() != expected_size
    {
        return Err(M0CompositionError::PolicyArtifactMismatch(role.to_owned()));
    }
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), M0CompositionError> {
    let algorithm = digest.algorithm().id().as_bytes();
    let algorithm_len = u16::try_from(algorithm.len())
        .map_err(|_| M0CompositionError::CanonicalLengthOverflow("digest algorithm"))?;
    let digest_len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| M0CompositionError::CanonicalLengthOverflow("digest"))?;
    out.extend_from_slice(&algorithm_len.to_be_bytes());
    out.extend_from_slice(algorithm);
    out.extend_from_slice(&digest_len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

#[derive(Debug, Error)]
pub enum M0CompositionError {
    #[error(transparent)]
    Manifest(#[from] OfflineManifestError),
    #[error(transparent)]
    Adapter(#[from] AdapterError),
    #[error(transparent)]
    Execution(#[from] ExecutionContractError),
    #[error(transparent)]
    GuestPlan(#[from] GuestPlanError),
    #[error(transparent)]
    HermeticRun(#[from] HermeticRunError),
    #[error(transparent)]
    Isolation(#[from] IsolationPolicyError),
    #[error(transparent)]
    Repository(#[from] RepositoryVerificationError),
    #[error(transparent)]
    RepositoryEvidence(#[from] RepositoryEvidenceError),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error("local embedded-key profile requires VerificationTimePolicy::NotUsed")]
    LocalKeyProfileRequiresTimelessVerification,
    #[error("execution specification is not a hermetic candidate")]
    ExecutionSpecNotHermeticCandidate,
    #[error("execution tool-role set differs from the exact M0 set")]
    ToolRoleSetMismatch,
    #[error("execution input-role set differs from the exact M0 set")]
    InputRoleSetMismatch,
    #[error("Linux policy mount-role set differs from the exact M0 input set")]
    PolicyMountRoleSetMismatch,
    #[error("execution environment differs from the exact Linux isolation environment")]
    ExecutionEnvironmentMismatch,
    #[error("local embedded-key profile forbids external trust material")]
    UnexpectedExternalTrustMaterial,
    #[error("sandbox invocation executable is not forge-hermetic-guest")]
    SandboxInvocationExecutableMismatch,
    #[error("sandbox invocation arguments differ from the exact guest-plan invocation")]
    SandboxInvocationArgumentsMismatch,
    #[error("trust evidence names a different repository policy state")]
    TrustPolicyStateMismatch,
    #[error("trust evidence names a different local-key profile")]
    TrustProfileMismatch,
    #[error("isolation evidence names a different Linux policy")]
    IsolationPolicyMismatch,
    #[error("runtime closure evidence names a different Nix closure")]
    RuntimeClosureMismatch,
    #[error("execution specification names a different execution subject")]
    ExecutionSubjectMismatch,
    #[error("guest plan names a different execution subject")]
    GuestPlanExecutionSubjectMismatch,
    #[error("guest plan names a different repository request")]
    GuestPlanRequestMismatch,
    #[error("guest plan names a different semantic bundle manifest")]
    GuestPlanManifestMismatch,
    #[error("guest plan names a different repository policy state")]
    GuestPlanPolicyStateMismatch,
    #[error("guest plan names a different isolation policy")]
    GuestPlanIsolationPolicyMismatch,
    #[error("guest plan names a different Nix closure")]
    GuestPlanClosureMismatch,
    #[error("guest plan names a different sandbox invocation")]
    GuestPlanInvocationMismatch,
    #[error("guest plan names a different trust profile")]
    GuestPlanTrustProfileMismatch,
    #[error("guest plan names a different run challenge")]
    GuestPlanChallengeMismatch,
    #[error("guest plan names a different expected replay receipt")]
    GuestPlanReplayReceiptMismatch,
    #[error("guest plan names a different Git object-validation policy")]
    GuestPlanGitPolicyMismatch,
    #[error("missing required execution input: {0}")]
    MissingRequiredInput(String),
    #[error("execution input artifact mismatch: {0}")]
    InputArtifactMismatch(String),
    #[error("missing required Linux policy artifact: {0}")]
    MissingPolicyArtifact(String),
    #[error("Linux policy artifact mismatch: {0}")]
    PolicyArtifactMismatch(String),
    #[error("evidence-bound execution names a different execution specification")]
    ExecutionSpecMismatch,
    #[error("same-run plan names a different execution specification")]
    RunPlanExecutionSpecMismatch,
    #[error("same-run plan names a different isolation policy")]
    RunPlanIsolationPolicyMismatch,
    #[error("same-run plan and guest plan use different Git object-validation policies")]
    RunPlanGitPolicyMismatch,
    #[error("same-run plan/result and guest plan use different challenges")]
    RunChallengeMismatch,
    #[error("same-run evidence names a different run plan")]
    RunPlanDigestMismatch,
    #[error("same-run pre/post runtime closure differs from qualified closure evidence")]
    RunRuntimeClosureMismatch,
    #[error("same-run policy-trust phase differs from qualified trust evidence")]
    RunTrustEvidenceMismatch,
    #[error("same-run isolation-qualification phase differs from qualified isolation evidence")]
    RunIsolationEvidenceMismatch,
    #[error("same-run verifier output differs from portable replay receipt")]
    RunVerifierOutputMismatch,
    #[error("hermetic execution output differs from portable replay receipt")]
    HermeticOutputMismatch,
    #[error("execution runtime evidence differs from isolation/closure/same-run composition")]
    RuntimeEvidenceMismatch,
    #[error("canonical length overflow: {0}")]
    CanonicalLengthOverflow(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_execution::{
        FilesystemPolicy, InputArtifact, NetworkPolicy, ToolArtifact,
    };
    use mycelix_forge_linux_isolation::{NixClosureEntry, VerifierInvocation};

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn tool(role: &str, byte: u8) -> ToolArtifact {
        ToolArtifact::new(role, "1", digest(byte), 1, None).unwrap()
    }

    #[test]
    fn execution_subject_changes_when_policy_state_changes() {
        let a = derive_execution_subject(
            &digest(1),
            &digest(2),
            &digest(3),
            &digest(4),
            &digest(5),
            &digest(6),
            &digest(7),
            &digest(8),
            &digest(9),
            &digest(10),
            &digest(11),
            &digest(12),
        )
        .unwrap();
        let b = derive_execution_subject(
            &digest(1),
            &digest(2),
            &digest(3),
            &digest(99),
            &digest(5),
            &digest(6),
            &digest(7),
            &digest(8),
            &digest(9),
            &digest(10),
            &digest(11),
            &digest(12),
        )
        .unwrap();
        assert_ne!(a, b);
    }

    #[test]
    fn extra_tool_role_fails_exact_m0_set() {
        let mut tools = REQUIRED_TOOL_ROLES
            .iter()
            .enumerate()
            .map(|(index, role)| tool(role, index as u8 + 1))
            .collect::<Vec<_>>();
        tools.push(tool("unexpected-shell", 0xf0));
        let spec = ExecutionSpec::new(
            mycelix_forge_execution::ExecutionPurpose::RepositoryVerification,
            digest(0x20),
            tools,
            vec![],
            vec![InputArtifact::new("placeholder", digest(0x21), 1).unwrap()],
            vec![],
            NetworkPolicy::Denied,
            VerificationTimePolicy::NotUsed,
            FilesystemPolicy::hermetic(),
        )
        .unwrap();
        assert!(matches!(
            require_exact_tool_roles(&spec),
            Err(M0CompositionError::ToolRoleSetMismatch)
        ));
    }

    #[test]
    fn sandbox_invocation_must_execute_guest_with_exact_plan_arg() {
        let closure = NixClosureManifest::new(vec![
            NixClosureEntry::new("/nix/store/aaaa-guest", digest(1)).unwrap(),
        ])
        .unwrap();
        let invocation = VerifierInvocation::new(
            "/nix/store/aaaa-guest/bin/forge-hermetic-guest",
            vec![GUEST_PLAN_FLAG.into(), GUEST_PLAN_PATH.into()],
            &closure,
        )
        .unwrap();
        require_guest_runner_invocation(&invocation).unwrap();
    }
}
