// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fail-closed runtime composition for current operational authority freshness.
//!
//! Providers return evidence-shaped candidates only. This coordinator reconstructs
//! every positive authority layer locally through the pure kernels:
//!
//! constitution/root -> control-plane freshness -> operational policy context
//! -> covered operational freshness.
//!
//! It never selects a latest DHT record, persists authority state, mints policy,
//! interprets probe authorship as authority, or enables external effects.

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
const ROOT_PROVIDER_ZOME: &str = "authority_state_bootstrap_root_provider";
const POLICY_PROVIDER_ZOME: &str = "authority_operational_policy_provider";
const PROBE_VERIFIER_ZOME: &str = "authority_state_challenge";
const CONTROL_PLANE_EVIDENCE_ZOME: &str = "authority_control_plane_evidence_provider";
const OPERATIONAL_EVIDENCE_ZOME: &str = "authority_operational_evidence_provider";
const MAX_CONTROL_PLANE_BUNDLES: usize = 3;
const MAX_WITNESSES: usize = 64;
const MAX_TRANSITIONS: usize = 256;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BootstrapRootCandidateBundle {
    pub manifest: AuthorityStateBootstrapRootManifest,
    pub current_constitution: VerifiedCurrentConstitutionReceipt,
    pub adoption: VerifiedBootstrapRootAdoption,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OperationalPolicyCandidateBundle {
    pub coverage: VerifiedAuthorityCoveragePolicy,
    pub context: VerifiedCoverageTrustContextPolicy,
}

/// Evidence-shaped bundle for exactly one authority subject.
///
/// The generic provider supplies the probe action reference, never a preverified
/// `VerifiedCoverageChallenge`. The dedicated #114 probe runtime must reconstruct
/// that receipt from the private entropy provenance before the pure #96 path runs.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AuthorityStateEvidenceBundle {
    pub probe_action: ActionHash,
    pub source_head: VerifiedAuthoritySourceHead,
    pub witnesses: Vec<VerifiedAuthorityHeadWitness>,
    pub trust_bindings: Vec<VerifiedWitnessTrustBinding>,
    pub transitions: Vec<VerifiedAuthorityStateTransition>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ControlPlaneEvidenceRequest {
    pub target_subject: AuthoritySubjectRef,
    pub root_manifest_digest: Digest32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OperationalEvidenceRequest {
    pub target_subject: AuthoritySubjectRef,
    pub context_policy_digest: Digest32,
    pub coverage_policy_digest: Digest32,
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
    pub latest_dht_record_authority: bool,
    pub provider_positive_object_authority: bool,
    pub probe_authority: bool,
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

fn validate_evidence_shape(bundle: &AuthorityStateEvidenceBundle) -> ExternResult<()> {
    if bundle.witnesses.len() > MAX_WITNESSES {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "authority-state witness count exceeds {MAX_WITNESSES}"
        ))));
    }
    if bundle.transitions.is_empty() || bundle.transitions.len() > MAX_TRANSITIONS {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "authority-state transition count must be 1-{MAX_TRANSITIONS}"
        ))));
    }
    Ok(())
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

fn resolve_root(
    now: u64,
) -> ExternResult<mycelix_authority_state_bootstrap_root::QualifiedAuthorityStateBootstrapRoot> {
    let bundle: BootstrapRootCandidateBundle = call_local(
        ROOT_PROVIDER_ZOME,
        "resolve_bootstrap_root_candidates",
        (),
    )?;
    qualify_bootstrap_root(
        &bundle.manifest,
        &bundle.current_constitution,
        &bundle.adoption,
        now,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "bootstrap-root qualification denied: {error}"
        )))
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

fn resolve_control_plane_freshness(
    root: &mycelix_authority_state_bootstrap_root::QualifiedAuthorityStateBootstrapRoot,
    target_subject: &AuthoritySubjectRef,
    now: u64,
) -> ExternResult<Vec<QualifiedControlPlaneSubjectFreshness>> {
    let bundles: Vec<AuthorityStateEvidenceBundle> = call_local(
        CONTROL_PLANE_EVIDENCE_ZOME,
        "resolve_control_plane_evidence",
        ControlPlaneEvidenceRequest {
            target_subject: target_subject.clone(),
            root_manifest_digest: root.root_manifest_digest(),
        },
    )?;
    if bundles.is_empty() || bundles.len() > MAX_CONTROL_PLANE_BUNDLES {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "control-plane evidence bundle count must be 1-{MAX_CONTROL_PLANE_BUNDLES}"
        ))));
    }

    let mut qualified = Vec::with_capacity(bundles.len());
    for bundle in bundles {
        validate_evidence_shape(&bundle)?;
        let challenge = verify_probe(bundle.probe_action, None)?;
        let value = qualify_control_plane_subject_freshness(
            root,
            &challenge,
            &bundle.source_head,
            &bundle.witnesses,
            &bundle.trust_bindings,
            &bundle.transitions,
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

/// Resolve current freshness for one exact operational authority subject.
///
/// Every provider response is candidate evidence. Positive authority is created
/// only by local calls to the pure qualification kernels.
#[hdk_extern]
pub fn resolve_current_operational_freshness(
    subject: AuthoritySubjectRef,
) -> ExternResult<VerifiedCurrentOperationalFreshnessReceipt> {
    subject.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "invalid operational authority subject: {error}"
        )))
    })?;
    let now = now_ms()?;

    let root = resolve_root(now)?;
    let policies = resolve_operational_policies(&subject)?;
    let control_plane = resolve_control_plane_freshness(&root, &subject, now)?;

    let context = qualify_operational_policy_context(
        &root,
        &subject,
        &policies.context,
        &policies.coverage,
        &control_plane,
        now,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "operational policy-context qualification denied: {error}"
        )))
    })?;

    let target_bundle: AuthorityStateEvidenceBundle = call_local(
        OPERATIONAL_EVIDENCE_ZOME,
        "resolve_operational_subject_evidence",
        OperationalEvidenceRequest {
            target_subject: subject.clone(),
            context_policy_digest: context.context_policy_digest(),
            coverage_policy_digest: context.coverage_policy_digest(),
        },
    )?;
    validate_evidence_shape(&target_bundle)?;
    let challenge = verify_probe(target_bundle.probe_action, Some(&subject))?;

    let current = qualify_operational_subject_freshness(
        &context,
        &challenge,
        &target_bundle.source_head,
        &target_bundle.witnesses,
        &target_bundle.trust_bindings,
        &target_bundle.transitions,
        now,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "operational freshness qualification denied: {error}"
        )))
    })?;

    let freshness = current.to_verified_freshness();
    freshness.validate_at(now).map_err(|error| {
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

/// Declarative only. Code presence is not operational authority and this zome is
/// intentionally absent from the binding governance DNA in this tranche.
#[hdk_extern]
pub fn current_freshness_runtime_status(_: ()) -> ExternResult<CurrentFreshnessRuntimeStatus> {
    Ok(CurrentFreshnessRuntimeStatus {
        protocol: RUNTIME_PROTOCOL.into(),
        composition_only: true,
        latest_dht_record_authority: false,
        provider_positive_object_authority: false,
        probe_authority: false,
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
