// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fail-closed runtime composition for current operational authority freshness.
//!
//! Discovery, constitutional currentness, root adoption, probe provenance,
//! source authentication, witness/trust verification and transition verification
//! are deliberately separate roles. Positive authority is reconstructed locally
//! through the pure qualification kernels.

use hdk::prelude::*;
use mycelix_authority_control_plane_freshness::{
    qualify_control_plane_subject_freshness, QualifiedControlPlaneSubjectFreshness,
};
use mycelix_authority_freshness::{AuthoritySubjectRef, VerifiedAuthorityFreshness};
use mycelix_authority_operational_context::qualify_operational_policy_context;
use mycelix_authority_operational_freshness::qualify_operational_subject_freshness;
use mycelix_authority_state_bootstrap_root::{
    qualify_bootstrap_root, AuthorityStateBootstrapRootManifest,
    VerifiedBootstrapRootAdoption, VerifiedCurrentConstitutionReceipt,
};
use mycelix_authority_state_coverage::{
    VerifiedAuthorityCoveragePolicy, VerifiedAuthorityHeadWitness, VerifiedAuthoritySourceHead,
};
use mycelix_authority_state_coverage_context::{
    VerifiedCoverageChallenge, VerifiedCoverageTrustContextPolicy, VerifiedWitnessTrustBinding,
};
use mycelix_authority_state_source::VerifiedAuthorityStateTransition;
use mycelix_institutional_core::Digest32;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

const RUNTIME_PROTOCOL: &str = "mycelix-authority-current-freshness-verifier-v0.1";
const ROOT_MANIFEST_PROVIDER_ZOME: &str = "authority_state_bootstrap_root_manifest_provider";
const CURRENT_CONSTITUTION_VERIFIER_ZOME: &str = "authority_current_constitution_verifier";
const ROOT_ADOPTION_VERIFIER_ZOME: &str = "authority_bootstrap_root_adoption_verifier";
const POLICY_PROVIDER_ZOME: &str = "authority_operational_policy_provider";
const EVIDENCE_PLAN_ZOME: &str = "authority_state_evidence_plan_provider";
const PROBE_VERIFIER_ZOME: &str = "authority_state_challenge";
const SOURCE_HEAD_VERIFIER_ZOME: &str = "authority_state_source_head_verifier";
const WITNESS_VERIFIER_ZOME: &str = "authority_state_witness_verifier";
const TRANSITION_VERIFIER_ZOME: &str = "authority_state_transition_verifier";
const MAX_CONTROL_PLANE_PROBES: usize = 3;
const MAX_WITNESSES: usize = 64;
const MAX_TRUST_BINDINGS: usize = 64;
const MAX_TRANSITIONS: usize = 256;

/// Exact candidate manifest + independently verified current constitutional
/// identity. The adoption verifier authenticates whether that exact root was
/// adopted under that exact current statement; it does not receive a positive
/// current-constitution receipt object.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RootAdoptionVerificationRequest {
    pub manifest: AuthorityStateBootstrapRootManifest,
    pub current_constitution_digest: Digest32,
    pub current_constitution_profile: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OperationalPolicyCandidateBundle {
    pub coverage: VerifiedAuthorityCoveragePolicy,
    pub context: VerifiedCoverageTrustContextPolicy,
}

/// Discovery only. The plan provider chooses probe references but attests no truth.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ControlPlaneProbePlanRequest {
    pub target_subject: AuthoritySubjectRef,
    pub root_manifest_digest: Digest32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ControlPlaneProbePlan {
    pub probe_actions: Vec<ActionHash>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OperationalProbePlanRequest {
    pub target_subject: AuthoritySubjectRef,
    pub context_policy_digest: Digest32,
    pub coverage_policy_digest: Digest32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OperationalProbePlan {
    pub probe_action: ActionHash,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SourceHeadVerificationRequest {
    /// #131 reconstructs positive challenge verification itself from this probe.
    pub probe_action: ActionHash,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WitnessVerificationRequest {
    pub challenge: VerifiedCoverageChallenge,
    pub source_head: VerifiedAuthoritySourceHead,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VerifiedWitnessEvidenceBundle {
    pub witnesses: Vec<VerifiedAuthorityHeadWitness>,
    pub trust_bindings: Vec<VerifiedWitnessTrustBinding>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TransitionVerificationRequest {
    pub subject: AuthoritySubjectRef,
    pub source_head: VerifiedAuthoritySourceHead,
}

struct ResolvedAuthorityEvidence {
    challenge: VerifiedCoverageChallenge,
    source_head: VerifiedAuthoritySourceHead,
    witnesses: Vec<VerifiedAuthorityHeadWitness>,
    trust_bindings: Vec<VerifiedWitnessTrustBinding>,
    transitions: Vec<VerifiedAuthorityStateTransition>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VerifiedCurrentOperationalFreshnessReceipt {
    pub protocol: String,
    pub subject: AuthoritySubjectRef,
    pub bootstrap_root_digest: Digest32,
    pub bootstrap_root_profile: String,
    pub operational_context_digest: Digest32,
    pub operational_context_profile: String,
    pub authority_digest: Digest32,
    pub authority_profile: String,
    pub evidence_digest: Digest32,
    pub evidence_profile: String,
    pub current_freshness: VerifiedAuthorityFreshness,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub lease_until_ms: u64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CurrentFreshnessRuntimeStatus {
    pub protocol: String,
    pub composition_only: bool,
    pub root_manifest_provider_grants_authority: bool,
    pub current_constitution_verifier_separate: bool,
    pub root_adoption_verifier_separate: bool,
    pub evidence_plan_grants_authority: bool,
    pub latest_dht_record_authority: bool,
    pub provider_positive_object_authority: bool,
    pub probe_authority: bool,
    pub qualification_time_after_evidence: bool,
    pub external_effects_enabled: bool,
    pub operational: bool,
}

fn call_local<I, O>(zome: &str, function: &str, input: I) -> ExternResult<O>
where
    I: Serialize,
    O: DeserializeOwned,
{
    let response = call(
        CallTargetCell::Local,
        ZomeName::from(zome),
        FunctionName::from(function),
        None,
        ExternIO::encode(input)
            .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?,
    )?;
    let io = match response {
        ZomeCallResponse::Ok(io) => io,
        other => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "{zome}::{function} unavailable; current freshness fails closed: {other:?}"
            ))));
        }
    };
    io.decode::<O>().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot decode {zome}::{function} response: {error}"
        )))
    })
}

fn now_ms() -> ExternResult<u64> {
    let micros = sys_time()?.as_micros();
    if micros <= 0 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "current time must be positive".into(),
        )));
    }
    Ok(micros as u64 / 1_000)
}

fn resolve_root(
) -> ExternResult<mycelix_authority_state_bootstrap_root::QualifiedAuthorityStateBootstrapRoot> {
    // Candidate location/semantics only. This role cannot manufacture either
    // positive verifier receipt consumed by #111.
    let manifest: AuthorityStateBootstrapRootManifest = call_local(
        ROOT_MANIFEST_PROVIDER_ZOME,
        "resolve_bootstrap_root_manifest",
        (),
    )?;
    manifest.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "bootstrap-root manifest candidate is invalid: {error}"
        )))
    })?;

    // The current-constitution verifier is local and receives no manifest-derived
    // selector at all. It must derive the live constitutional domain/head from its
    // own trusted runtime/constitution context.
    let current_constitution: VerifiedCurrentConstitutionReceipt = call_local(
        CURRENT_CONSTITUTION_VERIFIER_ZOME,
        "verify_current_constitution",
        (),
    )?;

    let root_manifest_digest = manifest.identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot compute bootstrap-root manifest identity: {error}"
        )))
    })?;

    // The adoption verifier sees the exact candidate root plus the current
    // statement identity returned by the independent constitution verifier. It
    // does not receive the positive current-constitution receipt itself.
    let adoption: VerifiedBootstrapRootAdoption = call_local(
        ROOT_ADOPTION_VERIFIER_ZOME,
        "verify_bootstrap_root_adoption",
        RootAdoptionVerificationRequest {
            manifest: manifest.clone(),
            current_constitution_digest: current_constitution.statement_digest,
            current_constitution_profile: current_constitution.statement_profile.clone(),
        },
    )?;

    // Qualification time follows every evidence-producing verifier call.
    let now = now_ms()?;
    let root = qualify_bootstrap_root(&manifest, &current_constitution, &adoption, now)
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "bootstrap-root qualification denied: {error}"
            )))
        })?;

    if root.manifest().identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot recompute qualified bootstrap-root manifest identity: {error}"
        )))
    })? != root_manifest_digest
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "qualified bootstrap-root manifest identity changed during composition".into(),
        )));
    }

    Ok(root)
}

fn resolve_operational_policies(
    subject: &AuthoritySubjectRef,
) -> ExternResult<OperationalPolicyCandidateBundle> {
    call_local(
        POLICY_PROVIDER_ZOME,
        "resolve_operational_policy_candidates",
        subject.clone(),
    )
}

fn verify_probe(
    probe_action: ActionHash,
    expected_subject: Option<&AuthoritySubjectRef>,
) -> ExternResult<VerifiedCoverageChallenge> {
    let challenge: VerifiedCoverageChallenge = call_local(
        PROBE_VERIFIER_ZOME,
        "verify_issued_authority_state_probe",
        probe_action,
    )?;
    if let Some(subject) = expected_subject {
        if &challenge.challenge.subject != subject {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "verified authority-state probe belongs to another subject".into(),
            )));
        }
    }
    Ok(challenge)
}

fn resolve_evidence(
    probe_action: ActionHash,
    expected_subject: Option<&AuthoritySubjectRef>,
) -> ExternResult<ResolvedAuthorityEvidence> {
    let challenge = verify_probe(probe_action.clone(), expected_subject)?;

    let source_head: VerifiedAuthoritySourceHead = call_local(
        SOURCE_HEAD_VERIFIER_ZOME,
        "verify_source_head",
        SourceHeadVerificationRequest { probe_action },
    )?;

    let witness_bundle: VerifiedWitnessEvidenceBundle = call_local(
        WITNESS_VERIFIER_ZOME,
        "verify_witness_evidence",
        WitnessVerificationRequest {
            challenge: challenge.clone(),
            source_head: source_head.clone(),
        },
    )?;
    if witness_bundle.witnesses.len() > MAX_WITNESSES {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "authority-state witness count exceeds {MAX_WITNESSES}"
        ))));
    }
    if witness_bundle.trust_bindings.len() > MAX_TRUST_BINDINGS {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "authority-state trust-binding count exceeds {MAX_TRUST_BINDINGS}"
        ))));
    }

    let transitions: Vec<VerifiedAuthorityStateTransition> = call_local(
        TRANSITION_VERIFIER_ZOME,
        "verify_transition_lineage",
        TransitionVerificationRequest {
            subject: challenge.challenge.subject.clone(),
            source_head: source_head.clone(),
        },
    )?;
    if transitions.is_empty() || transitions.len() > MAX_TRANSITIONS {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "authority-state transition count must be 1-{MAX_TRANSITIONS}"
        ))));
    }

    Ok(ResolvedAuthorityEvidence {
        challenge,
        source_head,
        witnesses: witness_bundle.witnesses,
        trust_bindings: witness_bundle.trust_bindings,
        transitions,
    })
}

fn resolve_control_plane_freshness(
    root: &mycelix_authority_state_bootstrap_root::QualifiedAuthorityStateBootstrapRoot,
    target_subject: &AuthoritySubjectRef,
) -> ExternResult<Vec<QualifiedControlPlaneSubjectFreshness>> {
    let root_manifest_digest = root.manifest().identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot compute qualified bootstrap-root manifest identity: {error}"
        )))
    })?;
    let plan: ControlPlaneProbePlan = call_local(
        EVIDENCE_PLAN_ZOME,
        "plan_control_plane_probes",
        ControlPlaneProbePlanRequest {
            target_subject: target_subject.clone(),
            root_manifest_digest,
        },
    )?;
    if plan.probe_actions.is_empty() || plan.probe_actions.len() > MAX_CONTROL_PLANE_PROBES {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "control-plane probe count must be 1-{MAX_CONTROL_PLANE_PROBES}"
        ))));
    }

    let mut qualified = Vec::with_capacity(plan.probe_actions.len());
    for probe_action in plan.probe_actions {
        let evidence = resolve_evidence(probe_action, None)?;
        let now = now_ms()?;
        let value = qualify_control_plane_subject_freshness(
            root,
            &evidence.challenge,
            &evidence.source_head,
            &evidence.witnesses,
            &evidence.trust_bindings,
            &evidence.transitions,
            now,
        )
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "control-plane freshness qualification denied: {error}"
            )))
        })?;
        qualified.push(value);
    }
    Ok(qualified)
}

#[hdk_extern]
pub fn resolve_current_operational_freshness(
    subject: AuthoritySubjectRef,
) -> ExternResult<VerifiedCurrentOperationalFreshnessReceipt> {
    subject.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "invalid operational authority subject: {error}"
        )))
    })?;

    let root = resolve_root()?;
    let policies = resolve_operational_policies(&subject)?;
    let control_plane = resolve_control_plane_freshness(&root, &subject)?;

    let context_now = now_ms()?;
    let context = qualify_operational_policy_context(
        &root,
        &subject,
        &policies.context,
        &policies.coverage,
        &control_plane,
        context_now,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "operational policy-context qualification denied: {error}"
        )))
    })?;

    let plan: OperationalProbePlan = call_local(
        EVIDENCE_PLAN_ZOME,
        "plan_operational_probe",
        OperationalProbePlanRequest {
            target_subject: subject.clone(),
            context_policy_digest: context.context_policy_digest(),
            coverage_policy_digest: context.coverage_policy_digest(),
        },
    )?;
    let evidence = resolve_evidence(plan.probe_action, Some(&subject))?;

    let current_now = now_ms()?;
    let current = qualify_operational_subject_freshness(
        &context,
        &evidence.challenge,
        &evidence.source_head,
        &evidence.witnesses,
        &evidence.trust_bindings,
        &evidence.transitions,
        current_now,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "operational freshness qualification denied: {error}"
        )))
    })?;

    let freshness = current.to_verified_freshness();
    freshness.validate_at(current_now).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "qualified operational freshness is not currently usable: {error}"
        )))
    })?;

    Ok(VerifiedCurrentOperationalFreshnessReceipt {
        protocol: RUNTIME_PROTOCOL.into(),
        subject,
        bootstrap_root_digest: current.root_qualification_digest(),
        bootstrap_root_profile: current.root_qualification_profile().into(),
        operational_context_digest: current.operational_context_digest(),
        operational_context_profile: current.operational_context_profile().into(),
        authority_digest: current.authority_digest(),
        authority_profile: current.authority_profile().into(),
        evidence_digest: current.evidence_digest(),
        evidence_profile: current.evidence_profile().into(),
        verification_ref: format!(
            "current-operational-freshness:{}:{}",
            current.authority_profile(),
            digest_hex(current.authority_digest())
        ),
        verified_at_ms: freshness.verified_at_ms,
        lease_until_ms: freshness.lease_until_ms,
        current_freshness: freshness,
    })
}

#[hdk_extern]
pub fn current_freshness_runtime_status(_: ()) -> ExternResult<CurrentFreshnessRuntimeStatus> {
    Ok(CurrentFreshnessRuntimeStatus {
        protocol: RUNTIME_PROTOCOL.into(),
        composition_only: true,
        root_manifest_provider_grants_authority: false,
        current_constitution_verifier_separate: true,
        root_adoption_verifier_separate: true,
        evidence_plan_grants_authority: false,
        latest_dht_record_authority: false,
        provider_positive_object_authority: false,
        probe_authority: false,
        qualification_time_after_evidence: true,
        external_effects_enabled: false,
        operational: false,
    })
}

fn digest_hex(digest: Digest32) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(64);
    for byte in digest.0 {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}
