// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-004D4C: acyclic final M0 composition gate.
//!
//! This supersedes the unconstructible D3A-v5 capsule surface. The final
//! ExecutionSpec still commits exactly eight inputs, but LinuxIsolationPolicyV1
//! commits only five independent policy/data artifacts. The guest plan, guest
//! tool map, and policy JSON are separately sealed control-plane inputs proven
//! by D4A and the v2 host receipt.

use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_execution::{
    qualify_evidence_bound_execution, EvidenceBoundExecution, ExecutionContractError,
    ExecutionObservation, ExecutionOutcome, ExecutionPurpose, ExecutionSpec, ExecutorIdentity,
    NetworkPolicy, VerificationTimeObservation, VerificationTimePolicy,
};
use mycelix_forge_gittuf_adapter::{AdapterError, REQUIRED_GITTUF_VERSION};
use mycelix_forge_gittuf_bundle::{OfflineManifestError, PortableRepositoryReplay};
use mycelix_forge_gittuf_trust_profile::local_embedded_keys_v1;
use mycelix_forge_guest_plan::{
    GuestPlanError, GuestVerificationPlanV1, GUEST_BUNDLE_PATH, GUEST_ISOLATION_POLICY_PATH,
    GUEST_MANIFEST_PATH, GUEST_NIX_CLOSURE_PATH, GUEST_PLAN_PATH, GUEST_RUN_CHALLENGE_PATH,
    GUEST_SANDBOX_INVOCATION_PATH,
};
use mycelix_forge_guest_tool_map::{GuestToolMapError, GuestToolMapV1};
use mycelix_forge_hermetic_host_v2::QualifiedHermeticHostRunV2;
use mycelix_forge_hermetic_run_evidence::{HermeticRunError, HermeticRunPlan, RUN_CHALLENGE_SIZE};
use mycelix_forge_linux_isolation::{
    IsolationPolicyError, LinuxIsolationPolicyV1, NixClosureManifest, VerifierInvocation,
};
use mycelix_forge_linux_isolation_evidence::expected_environment;
use mycelix_forge_nar_auditor::auditor_subject;
use mycelix_forge_repository::{
    qualify_evidence_backed_observation, AdapterIdentity, AdapterObservation, AdapterOutcome,
    EvidenceBackedAdapterObservation, QualifiedRepositoryVerification, RepositoryEvidenceError,
    RepositoryVerificationError, RepositoryVerificationRequest, VerificationProfile,
};
use mycelix_forge_runtime_tool_evidence::{
    audit_runtime_tools, QualifiedRuntimeToolEvidence, RuntimeToolEvidenceError,
};
use mycelix_forge_sealed_capsule_inputs::{
    CONTROL_INPUTS, GUEST_TOOL_MAP_PATH, POLICY_INPUTS,
};
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

const EXECUTION_SUBJECT_DOMAIN_V6: &[u8] = b"mycelix-forge/m0-hermetic-execution-subject/v6\0";
const CAPSULE_ARTIFACTS_DOMAIN_V3: &[u8] = b"mycelix-forge/m0-capsule-artifacts/v3\0";
const OFFLINE_EVIDENCE_DOMAIN_V6: &[u8] = b"mycelix-forge/m0-offline-evidence/v6\0";
const COMPOSER_NAME: &str = "mycelix-forge-m0";
const COMPOSER_VERSION: &str = "0.6.0-gittuf-0.16.0-local-keys";
const HOST_EXECUTOR_NAME: &str = "mycelix-forge-hermetic-host";
const HOST_EXECUTOR_VERSION: &str = "0.2.0";
const GUEST_EXECUTOR_VERSION: &str = "0.1.0";
const GUEST_EXECUTABLE_SUFFIX: &str = "/bin/forge-hermetic-guest";

const REQUIRED_TOOL_ROLES: &[&str] = &[
    "bubblewrap",
    "forge-hermetic-guest",
    "forge-hermetic-host",
    "forge-isolation-probe",
    "git",
    "gittuf",
];

pub struct M0CompositionInputsV6<'a> {
    pub request: &'a RepositoryVerificationRequest,
    pub replay: &'a PortableRepositoryReplay,
    pub execution_spec: &'a ExecutionSpec,
    pub run_plan: &'a HermeticRunPlan,
    pub guest_plan: &'a GuestVerificationPlanV1,
    pub guest_tool_map: &'a GuestToolMapV1,
    pub isolation_policy: &'a LinuxIsolationPolicyV1,
    pub closure: &'a NixClosureManifest,
    pub invocation: &'a VerifierInvocation,
    pub host: &'a QualifiedHermeticHostRunV2,
}

pub struct QualifiedM0OfflineEvidenceV6 {
    repository: QualifiedRepositoryVerification,
    execution: EvidenceBoundExecution,
    runtime_tools: QualifiedRuntimeToolEvidence,
    execution_subject: Digest,
    guest_plan: Digest,
    guest_tool_map: Digest,
    capsule_artifacts: Digest,
    sealed_capsule: Digest,
    host_evidence: Digest,
    evidence_commitment: Digest,
}

impl QualifiedM0OfflineEvidenceV6 {
    pub fn repository(&self) -> &QualifiedRepositoryVerification {
        &self.repository
    }

    pub fn execution(&self) -> &EvidenceBoundExecution {
        &self.execution
    }

    pub fn runtime_tools(&self) -> &QualifiedRuntimeToolEvidence {
        &self.runtime_tools
    }

    pub fn execution_subject(&self) -> &Digest {
        &self.execution_subject
    }

    pub fn guest_plan(&self) -> &Digest {
        &self.guest_plan
    }

    pub fn guest_tool_map(&self) -> &Digest {
        &self.guest_tool_map
    }

    pub fn capsule_artifacts(&self) -> &Digest {
        &self.capsule_artifacts
    }

    pub fn sealed_capsule(&self) -> &Digest {
        &self.sealed_capsule
    }

    pub fn host_evidence(&self) -> &Digest {
        &self.host_evidence
    }

    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

pub fn qualify_m0_offline_evidence_v6(
    inputs: M0CompositionInputsV6<'_>,
) -> Result<QualifiedM0OfflineEvidenceV6, M0CompositionV6Error> {
    inputs.replay.manifest().validate_for_request(inputs.request)?;
    inputs
        .replay
        .replay_receipt()
        .clone()
        .into_observation_for(inputs.request)?;
    inputs
        .isolation_policy
        .validate_dependencies(inputs.closure, inputs.invocation)?;

    require_execution_profile(inputs.execution_spec)?;
    require_exact_tool_roles(inputs.execution_spec)?;
    require_exact_tool_versions(inputs.execution_spec)?;
    require_exact_input_roles(inputs.execution_spec)?;
    require_exact_policy_surface(inputs.isolation_policy)?;
    require_exact_environment(inputs.execution_spec)?;
    require_guest_runner_invocation(inputs.invocation)?;

    let request_digest = inputs.request.digest(DigestAlgorithm::Sha256)?;
    let manifest_commitment = inputs
        .replay
        .manifest()
        .commitment(DigestAlgorithm::Sha256)?;
    let replay_receipt = inputs
        .replay
        .replay_receipt()
        .commitment(DigestAlgorithm::Sha256)?;
    if &replay_receipt != inputs.replay.manifest().local_receipt_commitment() {
        return Err(M0CompositionV6Error::ReplayReceiptManifestMismatch);
    }
    let policy_state = inputs
        .replay
        .manifest()
        .policy_state()
        .digest(DigestAlgorithm::Sha256)?;
    let isolation_policy = inputs
        .isolation_policy
        .digest(DigestAlgorithm::Sha256)?;
    let closure = inputs.closure.digest(DigestAlgorithm::Sha256)?;
    let invocation = inputs.invocation.digest(DigestAlgorithm::Sha256)?;
    let trust_profile = local_embedded_keys_v1().profile_digest().clone();
    let nar_auditor = auditor_subject();

    let execution_subject = derive_execution_subject_v6(
        &request_digest,
        &manifest_commitment,
        &replay_receipt,
        &policy_state,
        &isolation_policy,
        &closure,
        &invocation,
        &trust_profile,
        inputs.guest_plan.git_object_validation_policy(),
        inputs.run_plan.run_challenge(),
        &nar_auditor,
    )?;
    if inputs.execution_spec.subject() != &execution_subject {
        return Err(M0CompositionV6Error::ExecutionSubjectMismatch);
    }

    let guest_plan_digest = inputs.guest_plan.digest(DigestAlgorithm::Sha256)?;
    require_guest_plan_links(
        inputs.guest_plan,
        &execution_subject,
        &request_digest,
        &manifest_commitment,
        &policy_state,
        &isolation_policy,
        &closure,
        &invocation,
        &trust_profile,
        inputs.run_plan.run_challenge(),
        &replay_receipt,
    )?;

    let guest_tool_map_digest = inputs.guest_tool_map.digest(DigestAlgorithm::Sha256)?;
    if inputs.guest_tool_map.plan_digest() != &guest_plan_digest
        || inputs.guest_tool_map.execution_subject() != &execution_subject
        || inputs.guest_tool_map.nix_closure() != &closure
    {
        return Err(M0CompositionV6Error::GuestToolMapSubjectMismatch);
    }

    let execution_spec_digest = inputs.execution_spec.digest(DigestAlgorithm::Sha256)?;
    require_capsule_artifacts(
        &inputs,
        &manifest_commitment,
        &guest_plan_digest,
        &execution_spec_digest,
    )?;

    let run_plan_digest = inputs.run_plan.digest(DigestAlgorithm::Sha256)?;
    require_run_plan_links(
        inputs.run_plan,
        &execution_spec_digest,
        &isolation_policy,
        inputs.guest_plan.git_object_validation_policy(),
        inputs.guest_plan.run_challenge(),
    )?;

    require_host_links(
        inputs.host,
        &execution_spec_digest,
        &run_plan_digest,
        &guest_plan_digest,
        &guest_tool_map_digest,
        &isolation_policy,
        &closure,
        &nar_auditor,
        &replay_receipt,
        &execution_subject,
        &policy_state,
        &trust_profile,
    )?;

    let runtime_tools = audit_runtime_tools(
        inputs.execution_spec,
        inputs.closure,
        &runtime_tool_paths(inputs.host, inputs.invocation)?,
    )?;
    if runtime_tools.execution_spec_digest() != &execution_spec_digest
        || runtime_tools.closure_digest() != &closure
    {
        return Err(M0CompositionV6Error::RuntimeToolSubjectMismatch);
    }

    let execution = qualify_evidence_bound_execution(
        inputs.execution_spec,
        ExecutionObservation::new(
            ExecutorIdentity::new(HOST_EXECUTOR_NAME, HOST_EXECUTOR_VERSION)?,
            execution_spec_digest.clone(),
            execution_subject.clone(),
            ExecutionOutcome::Succeeded,
            Some(inputs.host.guest().transcript().evidence_digest().clone()),
            Some(inputs.host.evidence_digest().clone()),
            VerificationTimeObservation::NotUsed,
        )?,
    )?;
    if execution.output() != inputs.host.guest().transcript().evidence_digest()
        || execution.execution_evidence() != inputs.host.evidence_digest()
    {
        return Err(M0CompositionV6Error::ExecutionBindingMismatch);
    }

    let capsule_artifacts = derive_capsule_artifacts_v3(inputs.execution_spec)?;
    let sealed_capsule = inputs.host.sealed_capsule().evidence_digest().clone();
    let evidence_commitment = derive_offline_evidence_commitment_v6(
        &request_digest,
        &manifest_commitment,
        &replay_receipt,
        &execution_subject,
        &execution_spec_digest,
        &run_plan_digest,
        &guest_plan_digest,
        &guest_tool_map_digest,
        &capsule_artifacts,
        &sealed_capsule,
        runtime_tools.evidence_digest(),
        inputs.host.guest().envelope_digest(),
        inputs.host.guest().transcript().evidence_digest(),
        inputs.host.guest().trust().evidence_digest(),
        inputs.host.isolation().evidence_digest(),
        inputs.host.pre_runtime().evidence_digest(),
        inputs.host.post_runtime().evidence_digest(),
        inputs.host.same_run().evidence_digest(),
        inputs.host.evidence_digest(),
        &nar_auditor,
    )?;

    let profile = VerificationProfile::m0_protected_source();
    let observation = AdapterObservation::new(
        AdapterIdentity::new(COMPOSER_NAME, COMPOSER_VERSION)?,
        request_digest,
        inputs.request.to().clone(),
        inputs.request.repository_policy_state().clone(),
        Some(inputs.replay.replay_receipt().history_commitment().clone()),
        Some(evidence_commitment.clone()),
        profile.required().iter().copied(),
        AdapterOutcome::Verified,
    )?;
    let backed = EvidenceBackedAdapterObservation::new(
        observation,
        Some(
            inputs
                .replay
                .replay_receipt()
                .policy_lineage_commitment()
                .clone(),
        ),
    )?;
    let repository = qualify_evidence_backed_observation(&profile, inputs.request, backed)?;

    Ok(QualifiedM0OfflineEvidenceV6 {
        repository,
        execution,
        runtime_tools,
        execution_subject,
        guest_plan: guest_plan_digest,
        guest_tool_map: guest_tool_map_digest,
        capsule_artifacts,
        sealed_capsule,
        host_evidence: inputs.host.evidence_digest().clone(),
        evidence_commitment,
    })
}

fn require_execution_profile(spec: &ExecutionSpec) -> Result<(), M0CompositionV6Error> {
    if spec.purpose() != ExecutionPurpose::RepositoryVerification {
        return Err(M0CompositionV6Error::UnexpectedExecutionPurpose);
    }
    if spec.network() != NetworkPolicy::Denied
        || spec.clock() != &VerificationTimePolicy::NotUsed
        || !spec.filesystem().is_hermetic()
        || !spec.is_hermetic_candidate()
    {
        return Err(M0CompositionV6Error::ExecutionSpecNotExactM0Profile);
    }
    if !spec.trust_material().is_empty() {
        return Err(M0CompositionV6Error::UnexpectedExternalTrustMaterial);
    }
    Ok(())
}

fn require_exact_tool_roles(spec: &ExecutionSpec) -> Result<(), M0CompositionV6Error> {
    let actual = spec.tools().iter().map(|tool| tool.role()).collect::<BTreeSet<_>>();
    let expected = REQUIRED_TOOL_ROLES.iter().copied().collect::<BTreeSet<_>>();
    if actual == expected {
        Ok(())
    } else {
        Err(M0CompositionV6Error::ToolRoleSetMismatch)
    }
}

fn require_exact_tool_versions(spec: &ExecutionSpec) -> Result<(), M0CompositionV6Error> {
    for (role, expected) in [
        ("forge-hermetic-host", HOST_EXECUTOR_VERSION),
        ("forge-hermetic-guest", GUEST_EXECUTOR_VERSION),
        ("forge-isolation-probe", GUEST_EXECUTOR_VERSION),
        ("gittuf", REQUIRED_GITTUF_VERSION),
    ] {
        let tool = spec
            .tools()
            .iter()
            .find(|tool| tool.role() == role)
            .ok_or_else(|| M0CompositionV6Error::MissingRequiredTool(role.to_owned()))?;
        if tool.semantic_version() != expected {
            return Err(M0CompositionV6Error::ToolVersionMismatch(role.to_owned()));
        }
    }
    Ok(())
}

fn require_exact_input_roles(spec: &ExecutionSpec) -> Result<(), M0CompositionV6Error> {
    let actual = spec.inputs().iter().map(|input| input.role()).collect::<BTreeSet<_>>();
    let expected = POLICY_INPUTS
        .iter()
        .chain(CONTROL_INPUTS.iter())
        .map(|(role, _)| *role)
        .collect::<BTreeSet<_>>();
    if actual == expected {
        Ok(())
    } else {
        Err(M0CompositionV6Error::InputRoleSetMismatch)
    }
}

fn require_exact_policy_surface(
    policy: &LinuxIsolationPolicyV1,
) -> Result<(), M0CompositionV6Error> {
    let actual = policy
        .artifact_mounts()
        .iter()
        .map(|mount| (mount.role(), mount.destination()))
        .collect::<BTreeSet<_>>();
    let expected = POLICY_INPUTS.iter().copied().collect::<BTreeSet<_>>();
    if actual == expected {
        Ok(())
    } else {
        Err(M0CompositionV6Error::PolicyMountSetMismatch)
    }
}

fn require_exact_environment(spec: &ExecutionSpec) -> Result<(), M0CompositionV6Error> {
    let actual = spec
        .environment()
        .iter()
        .map(|entry| (entry.key().to_owned(), entry.value().to_owned()))
        .collect::<BTreeMap<_, _>>();
    let expected = expected_environment()
        .into_iter()
        .map(|entry| (entry.key().to_owned(), entry.value().to_owned()))
        .collect::<BTreeMap<_, _>>();
    if actual == expected {
        Ok(())
    } else {
        Err(M0CompositionV6Error::ExecutionEnvironmentMismatch)
    }
}

fn require_guest_runner_invocation(
    invocation: &VerifierInvocation,
) -> Result<(), M0CompositionV6Error> {
    if !invocation.executable().ends_with(GUEST_EXECUTABLE_SUFFIX) {
        return Err(M0CompositionV6Error::SandboxInvocationExecutableMismatch);
    }
    let expected = ["--plan", GUEST_PLAN_PATH];
    if invocation.args().len() != expected.len()
        || invocation
            .args()
            .iter()
            .map(String::as_str)
            .ne(expected.into_iter())
    {
        return Err(M0CompositionV6Error::SandboxInvocationArgumentsMismatch);
    }
    Ok(())
}

fn require_capsule_artifacts(
    inputs: &M0CompositionInputsV6<'_>,
    manifest_commitment: &Digest,
    guest_plan_digest: &Digest,
    execution_spec_digest: &Digest,
) -> Result<(), M0CompositionV6Error> {
    let manifest_json = json_artifact(inputs.replay.manifest(), "bundle manifest JSON")?;
    let guest_plan_json = json_artifact(inputs.guest_plan, "guest plan JSON")?;
    let guest_tool_map_json = json_artifact(inputs.guest_tool_map, "guest tool map JSON")?;
    let isolation_policy_json = json_artifact(inputs.isolation_policy, "isolation policy JSON")?;
    let closure_json = json_artifact(inputs.closure, "Nix closure JSON")?;
    let invocation_json = json_artifact(inputs.invocation, "sandbox invocation JSON")?;

    if inputs.guest_plan.bundle_manifest() != manifest_commitment
        || &inputs.guest_plan.digest(DigestAlgorithm::Sha256)? != guest_plan_digest
    {
        return Err(M0CompositionV6Error::CapsuleSemanticSubjectMismatch);
    }

    for (role, destination, digest, size) in [
        (
            "repository-bundle",
            GUEST_BUNDLE_PATH,
            inputs.replay.manifest().bundle_digest(),
            inputs.replay.manifest().bundle_size(),
        ),
        (
            "repository-bundle-manifest",
            GUEST_MANIFEST_PATH,
            &manifest_json.digest,
            manifest_json.size,
        ),
        (
            "nix-closure-manifest",
            GUEST_NIX_CLOSURE_PATH,
            &closure_json.digest,
            closure_json.size,
        ),
        (
            "sandbox-invocation",
            GUEST_SANDBOX_INVOCATION_PATH,
            &invocation_json.digest,
            invocation_json.size,
        ),
        (
            "run-challenge",
            GUEST_RUN_CHALLENGE_PATH,
            inputs.run_plan.run_challenge(),
            RUN_CHALLENGE_SIZE,
        ),
    ] {
        require_policy_input_and_mount(
            inputs.execution_spec,
            inputs.isolation_policy,
            role,
            destination,
            digest,
            size,
        )?;
    }

    for (role, destination, digest, size) in [
        (
            "guest-verification-plan",
            GUEST_PLAN_PATH,
            &guest_plan_json.digest,
            guest_plan_json.size,
        ),
        (
            "guest-tool-map",
            GUEST_TOOL_MAP_PATH,
            &guest_tool_map_json.digest,
            guest_tool_map_json.size,
        ),
        (
            "linux-isolation-policy",
            GUEST_ISOLATION_POLICY_PATH,
            &isolation_policy_json.digest,
            isolation_policy_json.size,
        ),
    ] {
        require_control_input(
            inputs.execution_spec,
            inputs.host,
            role,
            destination,
            digest,
            size,
        )?;
    }

    if inputs.host.sealed_capsule().execution_spec_digest() != execution_spec_digest
        || inputs.host.sealed_capsule().policy_digest()
            != &inputs.isolation_policy.digest(DigestAlgorithm::Sha256)?
    {
        return Err(M0CompositionV6Error::SealedCapsuleSubjectMismatch);
    }
    Ok(())
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
) -> Result<(), M0CompositionV6Error> {
    if plan.execution_subject() != execution_subject {
        return Err(M0CompositionV6Error::GuestPlanExecutionSubjectMismatch);
    }
    if &plan.request_digest()? != request {
        return Err(M0CompositionV6Error::GuestPlanRequestMismatch);
    }
    if plan.bundle_manifest() != manifest {
        return Err(M0CompositionV6Error::GuestPlanManifestMismatch);
    }
    if plan.policy_state() != policy_state {
        return Err(M0CompositionV6Error::GuestPlanPolicyStateMismatch);
    }
    if plan.isolation_policy() != isolation_policy {
        return Err(M0CompositionV6Error::GuestPlanIsolationPolicyMismatch);
    }
    if plan.nix_closure() != closure {
        return Err(M0CompositionV6Error::GuestPlanClosureMismatch);
    }
    if plan.sandbox_invocation() != invocation {
        return Err(M0CompositionV6Error::GuestPlanInvocationMismatch);
    }
    if plan.trust_profile() != trust_profile {
        return Err(M0CompositionV6Error::GuestPlanTrustProfileMismatch);
    }
    if plan.run_challenge() != run_challenge {
        return Err(M0CompositionV6Error::GuestPlanChallengeMismatch);
    }
    if plan.expected_replay_receipt() != replay_receipt {
        return Err(M0CompositionV6Error::GuestPlanReplayReceiptMismatch);
    }
    Ok(())
}

fn require_run_plan_links(
    plan: &HermeticRunPlan,
    execution_spec: &Digest,
    isolation_policy: &Digest,
    git_policy: &Digest,
    challenge: &Digest,
) -> Result<(), M0CompositionV6Error> {
    if plan.execution_spec() != execution_spec {
        return Err(M0CompositionV6Error::RunPlanExecutionSpecMismatch);
    }
    if plan.isolation_policy() != isolation_policy {
        return Err(M0CompositionV6Error::RunPlanIsolationPolicyMismatch);
    }
    if plan.git_object_validation_policy() != git_policy {
        return Err(M0CompositionV6Error::RunPlanGitPolicyMismatch);
    }
    if plan.run_challenge() != challenge {
        return Err(M0CompositionV6Error::RunChallengeMismatch);
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn require_host_links(
    host: &QualifiedHermeticHostRunV2,
    execution_spec: &Digest,
    run_plan: &Digest,
    guest_plan: &Digest,
    guest_tool_map: &Digest,
    isolation_policy: &Digest,
    closure: &Digest,
    nar_auditor: &Digest,
    replay_receipt: &Digest,
    execution_subject: &Digest,
    policy_state: &Digest,
    trust_profile: &Digest,
) -> Result<(), M0CompositionV6Error> {
    if host.execution_spec_digest() != execution_spec
        || host.sealed_capsule().execution_spec_digest() != execution_spec
    {
        return Err(M0CompositionV6Error::HostExecutionSpecMismatch);
    }
    if host.run_plan_digest() != run_plan || host.same_run().plan_digest() != run_plan {
        return Err(M0CompositionV6Error::HostRunPlanMismatch);
    }
    if host.guest().plan_digest() != guest_plan || host.guest_tools().plan_digest() != guest_plan {
        return Err(M0CompositionV6Error::HostGuestPlanMismatch);
    }
    if host.guest().tool_map_digest() != guest_tool_map
        || host.guest_tools().map_digest() != guest_tool_map
    {
        return Err(M0CompositionV6Error::HostGuestToolMapMismatch);
    }
    if host.sealed_capsule().policy_digest() != isolation_policy
        || host.isolation().policy_digest() != isolation_policy
    {
        return Err(M0CompositionV6Error::HostIsolationPolicyMismatch);
    }
    if host.pre_runtime().closure_digest() != closure
        || host.post_runtime().closure_digest() != closure
        || host.guest_tools().nix_closure() != closure
    {
        return Err(M0CompositionV6Error::HostClosureMismatch);
    }
    if host.pre_runtime().auditor_subject() != nar_auditor
        || host.post_runtime().auditor_subject() != nar_auditor
    {
        return Err(M0CompositionV6Error::HostNarAuditorMismatch);
    }
    if host.pre_runtime().evidence_digest() != host.post_runtime().evidence_digest() {
        return Err(M0CompositionV6Error::HostRuntimeClosureChanged);
    }
    if !host.pidfd_observed_exit() {
        return Err(M0CompositionV6Error::HostPidfdExitNotObserved);
    }
    if host.guest().gittuf_receipt() != replay_receipt
        || host.guest().transcript().gittuf_verification() != replay_receipt
    {
        return Err(M0CompositionV6Error::HostReplayReceiptMismatch);
    }
    if host.guest().transcript().execution_subject() != execution_subject {
        return Err(M0CompositionV6Error::HostExecutionSubjectMismatch);
    }
    if host.guest().trust().policy_state() != policy_state {
        return Err(M0CompositionV6Error::HostPolicyStateMismatch);
    }
    if host.guest().trust().profile_digest() != trust_profile {
        return Err(M0CompositionV6Error::HostTrustProfileMismatch);
    }
    if host.same_run().run_challenge() != host.guest().transcript().run_challenge() {
        return Err(M0CompositionV6Error::HostRunChallengeMismatch);
    }
    if host.same_run().pre_runtime_closure() != host.pre_runtime().evidence_digest()
        || host.same_run().post_runtime_closure() != host.post_runtime().evidence_digest()
        || host.same_run().git_object_validation()
            != host.guest().transcript().git_object_validation()
        || host.same_run().policy_trust() != host.guest().trust().evidence_digest()
        || host.same_run().isolation_probe() != host.guest().inside_digest()
        || host.same_run().verifier_output() != host.guest().transcript().evidence_digest()
        || host.same_run().isolation() != host.isolation().evidence_digest()
    {
        return Err(M0CompositionV6Error::HostSameRunCrossLinkMismatch);
    }
    Ok(())
}

fn runtime_tool_paths(
    host: &QualifiedHermeticHostRunV2,
    invocation: &VerifierInvocation,
) -> Result<BTreeMap<String, String>, M0CompositionV6Error> {
    let mut paths = BTreeMap::new();
    paths.insert(
        "forge-hermetic-host".to_owned(),
        host.host_executable().to_owned(),
    );
    paths.insert(
        "bubblewrap".to_owned(),
        host.bubblewrap_executable().to_owned(),
    );
    paths.insert(
        "forge-hermetic-guest".to_owned(),
        invocation.executable().to_owned(),
    );
    for role in ["forge-isolation-probe", "git", "gittuf"] {
        let executable = host
            .guest_tools()
            .executable(role)
            .ok_or_else(|| M0CompositionV6Error::MissingRuntimeToolPath(role.to_owned()))?;
        paths.insert(role.to_owned(), executable.to_owned());
    }
    Ok(paths)
}

fn require_policy_input_and_mount(
    spec: &ExecutionSpec,
    policy: &LinuxIsolationPolicyV1,
    role: &str,
    destination: &str,
    digest: &Digest,
    size: u64,
) -> Result<(), M0CompositionV6Error> {
    require_input(spec, role, digest, size)?;
    let mount = policy
        .artifact_mounts()
        .iter()
        .find(|mount| mount.role() == role)
        .ok_or_else(|| M0CompositionV6Error::MissingPolicyArtifact(role.to_owned()))?;
    if mount.destination() != destination || mount.digest() != digest || mount.size() != size {
        return Err(M0CompositionV6Error::PolicyArtifactMismatch(role.to_owned()));
    }
    Ok(())
}

fn require_control_input(
    spec: &ExecutionSpec,
    host: &QualifiedHermeticHostRunV2,
    role: &str,
    destination: &str,
    digest: &Digest,
    size: u64,
) -> Result<(), M0CompositionV6Error> {
    require_input(spec, role, digest, size)?;
    let observed = host
        .sealed_capsule()
        .controls()
        .iter()
        .find(|item| item.role() == role)
        .ok_or_else(|| M0CompositionV6Error::MissingControlObservation(role.to_owned()))?;
    if observed.destination() != destination
        || observed.digest() != digest
        || observed.size() != size
        || !observed.fully_sealed()
    {
        return Err(M0CompositionV6Error::ControlArtifactMismatch(role.to_owned()));
    }
    Ok(())
}

fn require_input(
    spec: &ExecutionSpec,
    role: &str,
    digest: &Digest,
    size: u64,
) -> Result<(), M0CompositionV6Error> {
    let input = spec
        .inputs()
        .iter()
        .find(|input| input.role() == role)
        .ok_or_else(|| M0CompositionV6Error::MissingRequiredInput(role.to_owned()))?;
    if input.digest() != digest || input.size() != size {
        return Err(M0CompositionV6Error::InputArtifactMismatch(role.to_owned()));
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct JsonArtifact {
    digest: Digest,
    size: u64,
}

fn json_artifact<T: serde::Serialize>(
    value: &T,
    field: &'static str,
) -> Result<JsonArtifact, M0CompositionV6Error> {
    let bytes = serde_json::to_vec(value)?;
    Ok(JsonArtifact {
        digest: Digest::of_bytes(DigestAlgorithm::Sha256, &bytes),
        size: u64::try_from(bytes.len())
            .map_err(|_| M0CompositionV6Error::CanonicalLengthOverflow(field))?,
    })
}

#[allow(clippy::too_many_arguments)]
pub fn derive_execution_subject_v6(
    request: &Digest,
    manifest: &Digest,
    replay_receipt: &Digest,
    policy_state: &Digest,
    isolation_policy: &Digest,
    closure: &Digest,
    invocation: &Digest,
    trust_profile: &Digest,
    git_validation_policy: &Digest,
    run_challenge: &Digest,
    nar_auditor: &Digest,
) -> Result<Digest, M0CompositionV6Error> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(EXECUTION_SUBJECT_DOMAIN_V6);
    for digest in [
        request,
        manifest,
        replay_receipt,
        policy_state,
        isolation_policy,
        closure,
        invocation,
        trust_profile,
        git_validation_policy,
        run_challenge,
        nar_auditor,
    ] {
        push_digest(&mut bytes, digest)?;
    }
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &bytes))
}

fn derive_capsule_artifacts_v3(spec: &ExecutionSpec) -> Result<Digest, M0CompositionV6Error> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(CAPSULE_ARTIFACTS_DOMAIN_V3);
    let count = u16::try_from(spec.inputs().len())
        .map_err(|_| M0CompositionV6Error::CanonicalLengthOverflow("capsule inputs"))?;
    bytes.extend_from_slice(&count.to_be_bytes());
    for input in spec.inputs() {
        push_string(&mut bytes, input.role())?;
        push_digest(&mut bytes, input.digest())?;
        bytes.extend_from_slice(&input.size().to_be_bytes());
    }
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &bytes))
}

#[allow(clippy::too_many_arguments)]
fn derive_offline_evidence_commitment_v6(
    request: &Digest,
    manifest: &Digest,
    replay_receipt: &Digest,
    execution_subject: &Digest,
    execution_spec: &Digest,
    run_plan: &Digest,
    guest_plan: &Digest,
    guest_tool_map: &Digest,
    capsule_artifacts: &Digest,
    sealed_capsule: &Digest,
    runtime_tools: &Digest,
    guest_envelope: &Digest,
    guest_transcript: &Digest,
    trust: &Digest,
    isolation: &Digest,
    pre_runtime: &Digest,
    post_runtime: &Digest,
    same_run: &Digest,
    host_evidence: &Digest,
    nar_auditor: &Digest,
) -> Result<Digest, M0CompositionV6Error> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(OFFLINE_EVIDENCE_DOMAIN_V6);
    for digest in [
        request,
        manifest,
        replay_receipt,
        execution_subject,
        execution_spec,
        run_plan,
        guest_plan,
        guest_tool_map,
        capsule_artifacts,
        sealed_capsule,
        runtime_tools,
        guest_envelope,
        guest_transcript,
        trust,
        isolation,
        pre_runtime,
        post_runtime,
        same_run,
        host_evidence,
        nar_auditor,
    ] {
        push_digest(&mut bytes, digest)?;
    }
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &bytes))
}

fn push_string(out: &mut Vec<u8>, value: &str) -> Result<(), M0CompositionV6Error> {
    let len = u16::try_from(value.len())
        .map_err(|_| M0CompositionV6Error::CanonicalLengthOverflow("string"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), M0CompositionV6Error> {
    push_string(out, digest.algorithm().id())?;
    let len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| M0CompositionV6Error::CanonicalLengthOverflow("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

#[derive(Debug, Error)]
pub enum M0CompositionV6Error {
    #[error(transparent)]
    Manifest(#[from] OfflineManifestError),
    #[error(transparent)]
    Adapter(#[from] AdapterError),
    #[error(transparent)]
    Execution(#[from] ExecutionContractError),
    #[error(transparent)]
    GuestPlan(#[from] GuestPlanError),
    #[error(transparent)]
    GuestToolMap(#[from] GuestToolMapError),
    #[error(transparent)]
    HermeticRun(#[from] HermeticRunError),
    #[error(transparent)]
    Isolation(#[from] IsolationPolicyError),
    #[error(transparent)]
    Repository(#[from] RepositoryVerificationError),
    #[error(transparent)]
    RepositoryEvidence(#[from] RepositoryEvidenceError),
    #[error(transparent)]
    RuntimeTools(#[from] RuntimeToolEvidenceError),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error("unexpected execution purpose for M0")]
    UnexpectedExecutionPurpose,
    #[error("execution spec does not use the exact M0 hermetic profile")]
    ExecutionSpecNotExactM0Profile,
    #[error("local embedded-key profile forbids external trust material")]
    UnexpectedExternalTrustMaterial,
    #[error("execution tool-role set differs from exact M0 v6 set")]
    ToolRoleSetMismatch,
    #[error("missing required execution tool: {0}")]
    MissingRequiredTool(String),
    #[error("execution tool version mismatch: {0}")]
    ToolVersionMismatch(String),
    #[error("execution input-role set differs from exact M0 eight-input surface")]
    InputRoleSetMismatch,
    #[error("Linux policy mount set differs from exact acyclic five-input surface")]
    PolicyMountSetMismatch,
    #[error("execution environment differs from exact isolation environment")]
    ExecutionEnvironmentMismatch,
    #[error("sandbox invocation executable is not forge-hermetic-guest")]
    SandboxInvocationExecutableMismatch,
    #[error("sandbox invocation args differ from exact guest-plan invocation")]
    SandboxInvocationArgumentsMismatch,
    #[error("portable replay receipt differs from manifest commitment")]
    ReplayReceiptManifestMismatch,
    #[error("execution specification names a different execution subject")]
    ExecutionSubjectMismatch,
    #[error("guest plan names a different execution subject")]
    GuestPlanExecutionSubjectMismatch,
    #[error("guest plan names a different request")]
    GuestPlanRequestMismatch,
    #[error("guest plan names a different bundle manifest")]
    GuestPlanManifestMismatch,
    #[error("guest plan names a different policy state")]
    GuestPlanPolicyStateMismatch,
    #[error("guest plan names a different isolation policy")]
    GuestPlanIsolationPolicyMismatch,
    #[error("guest plan names a different Nix closure")]
    GuestPlanClosureMismatch,
    #[error("guest plan names a different sandbox invocation")]
    GuestPlanInvocationMismatch,
    #[error("guest plan names a different trust profile")]
    GuestPlanTrustProfileMismatch,
    #[error("guest plan names a different challenge")]
    GuestPlanChallengeMismatch,
    #[error("guest plan names a different replay receipt")]
    GuestPlanReplayReceiptMismatch,
    #[error("guest tool map names different plan/execution/closure subjects")]
    GuestToolMapSubjectMismatch,
    #[error("capsule semantic subject mismatch")]
    CapsuleSemanticSubjectMismatch,
    #[error("missing required execution input: {0}")]
    MissingRequiredInput(String),
    #[error("execution input artifact mismatch: {0}")]
    InputArtifactMismatch(String),
    #[error("missing required Linux policy artifact: {0}")]
    MissingPolicyArtifact(String),
    #[error("Linux policy artifact mismatch: {0}")]
    PolicyArtifactMismatch(String),
    #[error("missing sealed control observation: {0}")]
    MissingControlObservation(String),
    #[error("sealed control artifact mismatch: {0}")]
    ControlArtifactMismatch(String),
    #[error("sealed capsule names a different ExecutionSpec or policy")]
    SealedCapsuleSubjectMismatch,
    #[error("same-run plan names a different execution spec")]
    RunPlanExecutionSpecMismatch,
    #[error("same-run plan names a different isolation policy")]
    RunPlanIsolationPolicyMismatch,
    #[error("same-run plan and guest plan use different Git validation policies")]
    RunPlanGitPolicyMismatch,
    #[error("same-run plan and guest plan use different challenges")]
    RunChallengeMismatch,
    #[error("host result names a different execution spec")]
    HostExecutionSpecMismatch,
    #[error("host result names a different run plan")]
    HostRunPlanMismatch,
    #[error("host result names a different guest plan")]
    HostGuestPlanMismatch,
    #[error("host result names a different guest tool map")]
    HostGuestToolMapMismatch,
    #[error("host result names a different isolation policy")]
    HostIsolationPolicyMismatch,
    #[error("host result names a different Nix closure")]
    HostClosureMismatch,
    #[error("host result names a different NAR auditor subject")]
    HostNarAuditorMismatch,
    #[error("host pre/post runtime closure evidence differs")]
    HostRuntimeClosureChanged,
    #[error("host did not observe the bound pidfd exit")]
    HostPidfdExitNotObserved,
    #[error("host guest result differs from portable replay receipt")]
    HostReplayReceiptMismatch,
    #[error("host guest transcript names a different execution subject")]
    HostExecutionSubjectMismatch,
    #[error("host trust result names a different policy state")]
    HostPolicyStateMismatch,
    #[error("host trust result names a different trust profile")]
    HostTrustProfileMismatch,
    #[error("host same-run result names a different challenge")]
    HostRunChallengeMismatch,
    #[error("host same-run phases differ from qualified component evidence")]
    HostSameRunCrossLinkMismatch,
    #[error("runtime tool evidence names a different spec/closure")]
    RuntimeToolSubjectMismatch,
    #[error("missing exact runtime tool path: {0}")]
    MissingRuntimeToolPath(String),
    #[error("generic evidence-bound execution differs from qualified host/guest evidence")]
    ExecutionBindingMismatch,
    #[error("canonical field length overflow: {0}")]
    CanonicalLengthOverflow(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    #[test]
    fn v6_subject_is_domain_separated_from_v5_shape() {
        let subject = derive_execution_subject_v6(
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
        )
        .unwrap();
        assert_ne!(subject, digest(0));
    }

    #[test]
    fn policy_and_control_surfaces_are_exactly_five_plus_three() {
        assert_eq!(POLICY_INPUTS.len(), 5);
        assert_eq!(CONTROL_INPUTS.len(), 3);
        let policy = POLICY_INPUTS
            .iter()
            .map(|(role, _)| *role)
            .collect::<BTreeSet<_>>();
        let controls = CONTROL_INPUTS
            .iter()
            .map(|(role, _)| *role)
            .collect::<BTreeSet<_>>();
        assert!(policy.is_disjoint(&controls));
    }
}
