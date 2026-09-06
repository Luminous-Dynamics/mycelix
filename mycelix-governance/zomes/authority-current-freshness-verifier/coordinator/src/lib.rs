// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fail-closed runtime composition for deployment-bound current operational authority.
//!
//! Positive semantic authority is reconstructed locally through #148/#111/#115/#116/#117.
//! Only after that non-deserializable proof exists do we bind it to the exact local
//! host DNA and exact final binding constitutional statement.

use hdk::prelude::*;
use mycelix_authority_bootstrap_root_adoption_verifier::{
    build_adoption_claim, qualify_bootstrap_root_adoption, BootstrapRootAdoptionClaim,
    VerifiedBootstrapRootAdoptionProof, ADOPTION_CLAIM_PROFILE,
};
use mycelix_authority_control_plane_freshness::{
    qualify_control_plane_subject_freshness, QualifiedControlPlaneSubjectFreshness,
};
use mycelix_authority_freshness::{AuthoritySubjectRef, VerifiedAuthorityFreshness};
use mycelix_authority_operational_context::qualify_operational_policy_context;
use mycelix_authority_operational_deployment_fence::{
    qualify_operational_freshness_for_deployment, HostLocalDnaContext,
};
use mycelix_authority_operational_freshness::qualify_operational_subject_freshness;
use mycelix_authority_state_bootstrap_root::{
    qualify_bootstrap_root, AuthorityStateBootstrapRootManifest,
    VerifiedCurrentConstitutionReceipt, CURRENT_CONSTITUTION_RECEIPT_PROTOCOL,
};
use mycelix_authority_state_coverage::{
    VerifiedAuthorityCoveragePolicy, VerifiedAuthorityHeadWitness, VerifiedAuthoritySourceHead,
};
use mycelix_authority_state_coverage_context::{
    VerifiedCoverageChallenge, VerifiedCoverageTrustContextPolicy, VerifiedWitnessTrustBinding,
};
use mycelix_authority_state_source::VerifiedAuthorityStateTransition;
use mycelix_governance_constitution::{
    ConstitutionStatement, Digest32 as ConstitutionDigest32, STATEMENT_PROFILE,
};
use mycelix_institutional_core::Digest32;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

const RUNTIME_PROTOCOL: &str = "mycelix-authority-current-freshness-verifier-v0.2";
const ROOT_MANIFEST_PROVIDER_ZOME: &str = "authority_state_bootstrap_root_manifest_provider";
const CONSTITUTION_TRANSITION_ZOME: &str = "constitution_transition";
const CURRENT_CONSTITUTION_FUNCTION: &str = "get_verified_current_constitution";
const ROOT_ADOPTION_PROOF_VERIFIER_ZOME: &str =
    "authority_bootstrap_root_adoption_proof_verifier";
const ROOT_ADOPTION_PROOF_VERIFIER_FUNCTION: &str = "verify_bootstrap_root_adoption_proof";
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
const CURRENT_CONSTITUTION_COMPOSITION_LEASE_MS: u64 = 30_000;
const DEPLOYMENT_RETURN_LEASE_MS: u64 = 5_000;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
struct VerifiedCurrentConstitutionMirror {
    dna_hash: String,
    statement: ConstitutionStatement,
    statement_digest: ConstitutionDigest32,
    verified_transition_count: u64,
    candidate_count: u64,
    legacy_constitution_authoritative: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RootAdoptionProofVerificationRequest {
    pub claim: BootstrapRootAdoptionClaim,
    pub claim_digest: Digest32,
    pub claim_profile: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OperationalPolicyCandidateBundle {
    pub coverage: VerifiedAuthorityCoveragePolicy,
    pub context: VerifiedCoverageTrustContextPolicy,
}

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

struct ResolvedBootstrapRoot {
    root: mycelix_authority_state_bootstrap_root::QualifiedAuthorityStateBootstrapRoot,
    constitution: VerifiedCurrentConstitutionMirror,
}

/// Wire projection only after local semantic + deployment qualification.
/// `authority_*` / `evidence_*` preserve #117's substrate-neutral identities;
/// `deployment_*` are the identities live consumers must use.
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
    pub local_dna_hash: String,
    pub constitution_statement_digest: Digest32,
    pub constitution_statement_profile: String,
    pub deployment_authority_digest: Digest32,
    pub deployment_authority_profile: String,
    pub deployment_evidence_digest: Digest32,
    pub deployment_evidence_profile: String,
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
    pub current_constitution_source_is_binding_plane: bool,
    pub constitution_rechecked_after_adoption: bool,
    pub constitution_rechecked_before_return: bool,
    pub root_adoption_proof_verifier_separate: bool,
    pub root_adoption_constructed_locally: bool,
    pub host_local_dna_derived: bool,
    pub constitutional_dna_cross_checked: bool,
    pub deployment_authority_constructed_locally: bool,
    pub semantic_only_output: bool,
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

fn constitution_digest_to_core(digest: ConstitutionDigest32) -> Digest32 {
    Digest32(digest.0)
}

fn verify_constitution_projection(
    current: &VerifiedCurrentConstitutionMirror,
) -> ExternResult<()> {
    if current.legacy_constitution_authoritative {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "binding constitution plane reported legacy constitution authority".into(),
        )));
    }
    if current.dna_hash.trim().is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "binding constitution plane returned empty DNA identity".into(),
        )));
    }
    current.statement.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "binding constitution plane returned invalid statement: {error}"
        )))
    })?;
    let recomputed = current.statement.digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot digest binding current constitution: {error}"
        )))
    })?;
    if recomputed != current.statement_digest {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "binding constitution plane returned mismatched statement digest".into(),
        )));
    }
    Ok(())
}

fn resolve_binding_current_constitution() -> ExternResult<VerifiedCurrentConstitutionMirror> {
    let current: VerifiedCurrentConstitutionMirror = call_local(
        CONSTITUTION_TRANSITION_ZOME,
        CURRENT_CONSTITUTION_FUNCTION,
        (),
    )?;
    verify_constitution_projection(&current)?;
    Ok(current)
}

fn ensure_same_constitution(
    expected: &VerifiedCurrentConstitutionMirror,
    observed: &VerifiedCurrentConstitutionMirror,
) -> ExternResult<()> {
    if expected.dna_hash != observed.dna_hash
        || expected.statement_digest != observed.statement_digest
        || expected.statement != observed.statement
        || expected.verified_transition_count != observed.verified_transition_count
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "binding constitutional head changed during current-authority composition".into(),
        )));
    }
    Ok(())
}

fn current_constitution_receipt(
    current: &VerifiedCurrentConstitutionMirror,
    now: u64,
) -> ExternResult<VerifiedCurrentConstitutionReceipt> {
    let statement_digest = constitution_digest_to_core(current.statement_digest);
    let valid_until_ms = now
        .checked_add(CURRENT_CONSTITUTION_COMPOSITION_LEASE_MS)
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "current-constitution composition lease overflow".into(),
            ))
        })?;
    Ok(VerifiedCurrentConstitutionReceipt {
        protocol_version: CURRENT_CONSTITUTION_RECEIPT_PROTOCOL.into(),
        statement: current.statement.clone(),
        statement_digest,
        statement_profile: STATEMENT_PROFILE.into(),
        dna_hash: current.dna_hash.clone(),
        verification_ref: format!(
            "constitution-transition-current:{STATEMENT_PROFILE}:{}",
            digest_hex(statement_digest)
        ),
        verified_at_ms: now,
        valid_until_ms,
    })
}

fn resolve_root() -> ExternResult<ResolvedBootstrapRoot> {
    let constitution_before = resolve_binding_current_constitution()?;

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
    let root_manifest_digest = manifest.identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot compute bootstrap-root manifest identity: {error}"
        )))
    })?;

    let adoption_claim = build_adoption_claim(&manifest, &constitution_before.statement)
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "bootstrap-root adoption claim denied: {error}"
            )))
        })?;
    let adoption_claim_digest = adoption_claim.identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot compute bootstrap-root adoption claim identity: {error}"
        )))
    })?;
    let adoption_proof: VerifiedBootstrapRootAdoptionProof = call_local(
        ROOT_ADOPTION_PROOF_VERIFIER_ZOME,
        ROOT_ADOPTION_PROOF_VERIFIER_FUNCTION,
        RootAdoptionProofVerificationRequest {
            claim: adoption_claim,
            claim_digest: adoption_claim_digest,
            claim_profile: ADOPTION_CLAIM_PROFILE.into(),
        },
    )?;

    let constitution_after = resolve_binding_current_constitution()?;
    ensure_same_constitution(&constitution_before, &constitution_after)?;

    let now = now_ms()?;
    let qualified_adoption = qualify_bootstrap_root_adoption(
        &manifest,
        &constitution_after.statement,
        &adoption_proof,
        now,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "bootstrap-root adoption qualification denied: {error}"
        )))
    })?;
    if qualified_adoption.claim_digest() != adoption_claim_digest {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "qualified bootstrap-root adoption claim changed during composition".into(),
        )));
    }
    let adoption = qualified_adoption.to_verified_adoption();
    let constitution_receipt = current_constitution_receipt(&constitution_after, now)?;
    let root = qualify_bootstrap_root(&manifest, &constitution_receipt, &adoption, now)
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

    Ok(ResolvedBootstrapRoot {
        root,
        constitution: constitution_after,
    })
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

    let resolved_root = resolve_root()?;
    let root = &resolved_root.root;
    let policies = resolve_operational_policies(&subject)?;
    let control_plane = resolve_control_plane_freshness(root, &subject)?;

    let context_now = now_ms()?;
    let context = qualify_operational_policy_context(
        root,
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
    current.to_verified_freshness().validate_at(current_now).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "qualified operational freshness is not currently usable: {error}"
        )))
    })?;

    // Host DNA is obtained independently of the constitutional receipt. The
    // constitutional plane uses the same canonical DnaHash string encoding, but
    // equality is still checked only inside the pure deployment fence.
    let host_dna_hash = dna_info()?.hash.to_string();
    let host_dna_observed_at = now_ms()?;

    // Re-project constitutional truth after the host-DNA observation. Any
    // advancement during the entire semantic composition or host lookup denies.
    let final_constitution = resolve_binding_current_constitution()?;
    ensure_same_constitution(&resolved_root.constitution, &final_constitution)?;
    let final_now = now_ms()?;

    let deployment_until = final_now
        .checked_add(DEPLOYMENT_RETURN_LEASE_MS)
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "deployment-fence lease overflow".into(),
            ))
        })?;
    let host_context = HostLocalDnaContext::from_host_observation(
        host_dna_hash,
        host_dna_observed_at,
        deployment_until,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "host local-DNA context denied: {error}"
        )))
    })?;
    let constitution_statement_digest =
        constitution_digest_to_core(final_constitution.statement_digest);
    let deployment = qualify_operational_freshness_for_deployment(
        &current,
        &final_constitution.dna_hash,
        constitution_statement_digest,
        &host_context,
        final_now,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "deployment-bound operational freshness denied: {error}"
        )))
    })?;

    let freshness = deployment.to_verified_freshness();
    freshness.validate_at(final_now).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "deployment-bound freshness is not reusable: {error}"
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
        local_dna_hash: deployment.dna_hash().into(),
        constitution_statement_digest: deployment.constitution_statement_digest(),
        constitution_statement_profile: deployment.constitution_statement_profile().into(),
        deployment_authority_digest: deployment.deployment_authority_digest(),
        deployment_authority_profile: deployment.deployment_authority_profile().into(),
        deployment_evidence_digest: deployment.deployment_evidence_digest(),
        deployment_evidence_profile: deployment.deployment_evidence_profile().into(),
        verification_ref: deployment.verification_ref().into(),
        verified_at_ms: deployment.verified_at_ms(),
        lease_until_ms: deployment.valid_until_ms(),
        current_freshness: freshness,
    })
}

#[hdk_extern]
pub fn current_freshness_runtime_status(_: ()) -> ExternResult<CurrentFreshnessRuntimeStatus> {
    Ok(CurrentFreshnessRuntimeStatus {
        protocol: RUNTIME_PROTOCOL.into(),
        composition_only: true,
        root_manifest_provider_grants_authority: false,
        current_constitution_source_is_binding_plane: true,
        constitution_rechecked_after_adoption: true,
        constitution_rechecked_before_return: true,
        root_adoption_proof_verifier_separate: true,
        root_adoption_constructed_locally: true,
        host_local_dna_derived: true,
        constitutional_dna_cross_checked: true,
        deployment_authority_constructed_locally: true,
        semantic_only_output: false,
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
