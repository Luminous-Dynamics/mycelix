// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Verification Coordinator Zome
//!
//! Functions for submitting verifications, safety claims, and
//! bridging to the Knowledge hApp for epistemic scoring.

use hdk::prelude::*;
use verification_integrity::*;
use fabrication_common::*;

use sha2::{Digest, Sha256};
use std::cell::RefCell;
use std::collections::HashMap;

const EPISTEMIC_CACHE_TTL_MICROS: i64 = 300_000_000; // 5 min
const EPISTEMIC_CACHE_MAX_ENTRIES: usize = 128;
const EPISTEMIC_CACHE_DOMAIN: &[u8] = b"mycelix-fabrication-epistemic-cache:v1";

thread_local! {
    static CONFIG: RefCell<Option<FabricationConfig>> = const { RefCell::new(None) };
    static EPISTEMIC_CACHE: RefCell<HashMap<[u8; 32], (i64, ClaimEpistemic)>> = RefCell::new(HashMap::new());
}

fn get_config() -> FabricationConfig {
    CONFIG.with(|c| {
        c.borrow_mut()
            .get_or_insert_with(|| {
                dna_info()
                    .map(|info| FabricationConfig::from_properties_or_default(info.modifiers.properties.bytes()))
                    .unwrap_or_default()
            })
            .clone()
    })
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CreateFpmRegistrationAnchorInput {
    pub envelope: RegistrationEnvelope,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ResolveFpmRegistrationActionAnchorInput {
    pub action_hash: ActionHash,
    pub claimed_envelope_digest: String,
    pub expected_author: Option<AgentPubKey>,
    pub expected_signer: Option<AgentPubKey>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ResolveFpmRegistrationEntryAnchorInput {
    pub entry_hash: EntryHash,
    pub claimed_envelope_digest: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ResolvedFpmRegistrationAnchor {
    pub anchor_kind: RegistrationAnchorKind,
    pub anchor_reference: String,
    pub registration_envelope_digest: String,
    pub entry_hash: EntryHash,
    pub action_hash: Option<ActionHash>,
    pub author: Option<AgentPubKey>,
    pub signer: Option<AgentPubKey>,
    pub timestamp: Option<Timestamp>,
    pub action_seq: Option<u32>,
    pub prev_action: Option<ActionHash>,
    pub envelope: RegistrationEnvelope,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CreateFpmAcquisitionRootAnchorInput {
    pub source_system_id: String,
    pub capture_reference: String,
    pub artifact_digest: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ResolveFpmAcquisitionRootActionAnchorInput {
    pub action_hash: ActionHash,
    pub expected_root_digest: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ResolvedFpmAcquisitionRootAnchor {
    pub action_hash: ActionHash,
    pub entry_hash: EntryHash,
    pub root_digest: String,
    pub source_system_id: String,
    pub capture_reference: String,
    pub artifact_digest: String,
    pub author: AgentPubKey,
    pub signer: AgentPubKey,
    pub timestamp: Timestamp,
    pub action_seq: u32,
    pub prev_action: Option<ActionHash>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CreateFpmVerificationKeyTrustAnchorInput {
    pub verifier_agent: AgentPubKey,
    pub verification_key_id: Vec<u8>,
    pub public_key_sec1: Vec<u8>,
    pub attestation_format: String,
    pub verifier_profile_digest: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ResolveFpmVerificationKeyTrustAnchorInput {
    pub action_hash: ActionHash,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ResolvedFpmVerificationKeyTrustAnchor {
    pub action_hash: ActionHash,
    pub entry_hash: EntryHash,
    pub verifier_agent: AgentPubKey,
    pub verification_key_id: Vec<u8>,
    pub public_key_sec1: Vec<u8>,
    pub verification_key_digest: String,
    pub attestation_format: String,
    pub verifier_profile_digest: String,
    pub authority_agent: AgentPubKey,
    pub timestamp: Timestamp,
    pub action_seq: u32,
    pub prev_action: Option<ActionHash>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CreateFpmVerifierImplementationTrustAnchorInput {
    pub verifier_agent: AgentPubKey,
    pub verification_key_trust_anchor_action: ActionHash,
    pub implementation_digest: String,
    pub build_provenance_digest: String,
    pub builder_id: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ResolveFpmVerifierImplementationTrustAnchorInput {
    pub action_hash: ActionHash,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ResolvedFpmVerifierImplementationTrustAnchor {
    pub action_hash: ActionHash,
    pub entry_hash: EntryHash,
    pub verifier_agent: AgentPubKey,
    pub verification_key_trust_anchor_action: ActionHash,
    pub implementation_digest: String,
    pub build_provenance_digest: String,
    pub builder_id: String,
    pub verifier_profile_digest: String,
    pub authority_agent: AgentPubKey,
    pub timestamp: Timestamp,
    pub action_seq: u32,
    pub prev_action: Option<ActionHash>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CreateFpmAttestationChallengeInput {
    pub acquisition_root_action: ActionHash,
    pub audience: String,
    pub verification_key_trust_anchor_action: ActionHash,
    pub verifier_implementation_trust_anchor_action: ActionHash,
    pub appraisal_policy_digest: String,
    pub reference_values_digest: String,
    pub endorsement_digest: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ResolveFpmAttestationChallengeInput {
    pub action_hash: ActionHash,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ResolvedFpmAttestationChallenge {
    pub action_hash: ActionHash,
    pub subject_id: String,
    pub audience: String,
    pub verification_key_trust_anchor_action: ActionHash,
    pub verifier_implementation_trust_anchor_action: ActionHash,
    pub verification_key_id: Vec<u8>,
    pub verification_key_digest: String,
    pub verifier_implementation_digest: String,
    pub verifier_build_provenance_digest: String,
    pub verifier_builder_id: String,
    pub acquisition_root_action: ActionHash,
    pub acquisition_root_digest: String,
    pub verifier_agent: AgentPubKey,
    pub attestation_format: String,
    pub verifier_profile_digest: String,
    pub appraisal_policy_digest: String,
    pub reference_values_digest: String,
    pub endorsement_digest: String,
    pub nonce: Vec<u8>,
    pub nonce_digest: String,
    pub author: AgentPubKey,
    pub signer: AgentPubKey,
    pub timestamp: Timestamp,
    pub action_seq: u32,
    pub prev_action: Option<ActionHash>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CreateFpmSourceAttestationAnchorInput {
    pub challenge_action: ActionHash,
    pub eat_cose_verification_action: ActionHash,
    pub verifier_id: String,
    pub verifier_version: String,
    pub disposition: FpmAttestationDisposition,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct VerifyFpmEatCoseAgainstChallengeInput {
    pub challenge_action: ActionHash,
    pub token_bytes: Vec<u8>,
    pub expected_evidence_digest: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct FpmChallengeEatCoseVerification {
    pub challenge_action: ActionHash,
    pub acquisition_root_action: ActionHash,
    pub verification: FpmVerifiedEatCoseEvidence,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CreateFpmEatCoseVerificationAnchorInput {
    pub challenge_action: ActionHash,
    pub token_bytes: Vec<u8>,
    pub expected_evidence_digest: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ResolveFpmSourceAttestationAnchorInput {
    pub action_hash: ActionHash,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ResolvedFpmSourceAttestationAnchor {
    pub action_hash: ActionHash,
    pub entry_hash: EntryHash,
    pub challenge_action: ActionHash,
    pub eat_cose_verification_action: ActionHash,
    pub claim: FpmSourceAttestationClaim,
    pub claim_digest: String,
    pub author: AgentPubKey,
    pub signer: AgentPubKey,
    pub timestamp: Timestamp,
    pub action_seq: u32,
    pub prev_action: Option<ActionHash>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct QualifyFpmSourceAttestationInput {
    pub challenge_action: ActionHash,
    pub attestation_action: ActionHash,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct FpmSourceAttestationQualificationResult {
    pub qualification: FpmAttestationQualification,
    pub challenge_action: ActionHash,
    pub attestation_action: ActionHash,
    pub consumed: bool,
}

fn fpm_attestation_error(reason: impl Into<String>) -> WasmError {
    FabricationError::ValidationFailed {
        field: "fpm_source_attestation".into(),
        reason: reason.into(),
    }
    .to_wasm_error()
}
fn resolve_fpm_eat_cose_verification_anchor_impl(
    action_hash: ActionHash,
) -> ExternResult<ResolvedFpmEatCoseVerificationAnchor> {
    let details = get_details(action_hash.clone(), GetOptions::network())?
        .ok_or_else(|| FabricationError::not_found(
            "FpmEatCoseVerificationAnchor",
            &action_hash,
        ))?;
    let Details::Record(record_details) = details else {
        return Err(fpm_attestation_error(
            "EAT/COSE verification ActionHash did not resolve to record details",
        ));
    };
    if record_details.validation_status != ValidationStatus::Valid
        || !record_details.updates.is_empty()
        || !record_details.deletes.is_empty()
    {
        return Err(fpm_attestation_error(
            "EAT/COSE verification record is not currently valid and immutable",
        ));
    }

    let record = record_details.record;
    if record.action().action_type() != ActionType::Create {
        return Err(fpm_attestation_error(
            "EAT/COSE verification ActionHash must resolve to its original Create action",
        ));
    }

    let expected_entry_type = EntryType::App(
        UnitEntryTypes::FpmEatCoseVerificationAnchor
            .try_into()
            .map_err(|_| fpm_attestation_error(
                "could not construct EAT/COSE verification entry type",
            ))?,
    );
    if record.action().entry_type() != Some(&expected_entry_type) {
        return Err(fpm_attestation_error(
            "ActionHash does not reference the FPM EAT/COSE verification entry type",
        ));
    }

    let anchor: FpmEatCoseVerificationAnchor = record
        .entry()
        .to_app_option()
        .map_err(|e| fpm_attestation_error(format!(
            "could not decode EAT/COSE verification anchor: {e}"
        )))?
        .ok_or_else(|| fpm_attestation_error(
            "record is not an FPM EAT/COSE verification anchor",
        ))?;

    if anchor.schema_version != FPM_EAT_COSE_VERIFICATION_ANCHOR_SCHEMA_VERSION
        || !valid_fpm_digest(&anchor.evidence_digest)
        || !valid_fpm_digest(&anchor.payload_digest)
        || !valid_fpm_digest(&anchor.nonce_digest)
        || !valid_fpm_digest(&anchor.verification_key_digest)
        || anchor.key_id.is_empty()
        || anchor.key_id.len() > 128
    {
        return Err(fpm_attestation_error(
            "EAT/COSE verification anchor contains malformed commitments",
        ));
    }

    let verification =
        resolve_fpm_eat_cose_verification_anchor_impl(anchor.eat_cose_verification_action.clone())?;

    if verification.challenge_action != anchor.challenge_action
        || verification.evidence_digest != anchor.claim.evidence_digest
        || verification.subject_id != anchor.claim.subject_id
        || verification.audience != anchor.claim.audience
        || verification.key_id != anchor.claim.verification_key_id
        || verification.verification_key_digest != anchor.claim.verification_key_digest
        || verification.nonce_digest != anchor.claim.challenge_nonce_digest
    {
        return Err(fpm_attestation_error(
            "source attestation is not exactly bound to its EAT/COSE verification",
        ));
    }

    let challenge = resolve_fpm_attestation_challenge_impl(
        ResolveFpmAttestationChallengeInput {
            action_hash: anchor.challenge_action.clone(),
        },
    )?;

    if challenge.attestation_format != FPM_EAT_MEDIA_TYPE
        || challenge.subject_id != anchor.subject_id
        || challenge.audience != anchor.audience
        || challenge.verification_key_id != anchor.key_id
        || challenge.verification_key_digest != anchor.verification_key_digest
        || challenge.nonce_digest != anchor.nonce_digest
    {
        return Err(fpm_attestation_error(
            "EAT/COSE verification anchor does not match its challenge",
        ));
    }

    let entry_hash = record
        .action()
        .entry_hash()
        .ok_or_else(|| fpm_attestation_error(
            "EAT/COSE verification action has no entry hash",
        ))?
        .clone();

    Ok(ResolvedFpmEatCoseVerificationAnchor {
        action_hash,
        entry_hash,
        challenge_action: anchor.challenge_action,
        evidence_digest: anchor.evidence_digest,
        payload_digest: anchor.payload_digest,
        subject_id: anchor.subject_id,
        audience: anchor.audience,
        nonce_digest: anchor.nonce_digest,
        eat_profile_uri: anchor.eat_profile_uri,
        key_id: anchor.key_id,
        verification_key_digest: anchor.verification_key_digest,
        author: *record.action().author(),
        signer: *record.action().signer(),
        timestamp: record.action().timestamp(),
        action_seq: record.action().action_seq(),
        prev_action: record.action().prev_action().cloned(),
    })
}

fn verify_fpm_eat_cose_against_challenge_impl(
    input: VerifyFpmEatCoseAgainstChallengeInput,
) -> ExternResult<FpmChallengeEatCoseVerification> {
    let challenge = resolve_fpm_attestation_challenge_impl(
        ResolveFpmAttestationChallengeInput {
            action_hash: input.challenge_action.clone(),
        },
    )?;

    if challenge.attestation_format != FPM_EAT_MEDIA_TYPE {
        return Err(fpm_attestation_error(
            "challenge is not configured for application/eat+cwt",
        ));
    }

    let root = resolve_acquisition_root_action_anchor(
        ResolveFpmAcquisitionRootActionAnchorInput {
            action_hash: challenge.acquisition_root_action.clone(),
            expected_root_digest: Some(challenge.acquisition_root_digest.clone()),
        },
    )?;
    if root.source_system_id != challenge.subject_id {
        return Err(fpm_attestation_error(
            "challenge subject no longer matches authenticated acquisition root",
        ));
    }

    let trust = resolve_fpm_verification_key_trust_anchor_impl(
        ResolveFpmVerificationKeyTrustAnchorInput {
            action_hash: challenge.verification_key_trust_anchor_action.clone(),
        },
    )?;
    let verification = verify_fpm_eat_cose_sign1(&FpmEatCoseVerificationInput {
        expected_subject_id: challenge.subject_id.clone(),
        expected_audience: challenge.audience.clone(),
        expected_nonce: challenge.nonce.clone(),
        expected_verification_key_id: trust.verification_key_id.clone(),
        expected_verification_key_digest: trust.verification_key_digest.clone(),
        trusted_public_key_sec1: trust.public_key_sec1,
        expected_evidence_digest: input.expected_evidence_digest,
        token_bytes: input.token_bytes,
    });

    Ok(FpmChallengeEatCoseVerification {
        challenge_action: input.challenge_action,
        acquisition_root_action: challenge.acquisition_root_action,
        verification,
    })
}

fn create_fpm_eat_cose_verification_anchor_impl(
    input: CreateFpmEatCoseVerificationAnchorInput,
) -> ExternResult<Record> {
    let challenge = resolve_fpm_attestation_challenge_impl(
        ResolveFpmAttestationChallengeInput {
            action_hash: input.challenge_action.clone(),
        },
    )?;

    let current_agent = agent_info()?.agent_initial_pubkey;
    if current_agent != challenge.verifier_agent {
        return Err(fpm_attestation_error(
            "only the challenge verifier may create the EAT/COSE verification anchor",
        ));
    }

    let result = verify_fpm_eat_cose_against_challenge_impl(
        VerifyFpmEatCoseAgainstChallengeInput {
            challenge_action: input.challenge_action.clone(),
            token_bytes: input.token_bytes,
            expected_evidence_digest: input.expected_evidence_digest,
        },
    )?;

    if result.verification.status != FpmEatCoseVerificationStatus::QualifiedForProfile {
        return Err(fpm_attestation_error(format!(
            "EAT/COSE verification did not qualify: {:?}",
            result.verification.reasons,
        )));
    }

    let payload_digest = result.verification.payload_digest.clone().ok_or_else(|| {
        fpm_attestation_error("qualified EAT/COSE verification has no payload digest")
    })?;
    let subject_id = result.verification.subject_id.clone().ok_or_else(|| {
        fpm_attestation_error("qualified EAT/COSE verification has no subject")
    })?;
    let audience = result.verification.audience.clone().ok_or_else(|| {
        fpm_attestation_error("qualified EAT/COSE verification has no audience")
    })?;
    let nonce = result.verification.nonce.clone().ok_or_else(|| {
        fpm_attestation_error("qualified EAT/COSE verification has no nonce")
    })?;
    let eat_profile_uri = result.verification.eat_profile_uri.clone().ok_or_else(|| {
        fpm_attestation_error("qualified EAT/COSE verification has no profile")
    })?;
    let key_id = result.verification.key_id.clone().ok_or_else(|| {
        fpm_attestation_error("qualified EAT/COSE verification has no key id")
    })?;

    let anchor = FpmEatCoseVerificationAnchor {
        schema_version: FPM_EAT_COSE_VERIFICATION_ANCHOR_SCHEMA_VERSION.into(),
        challenge_action: input.challenge_action,
        evidence_digest: result.verification.evidence_digest,
        payload_digest,
        subject_id,
        audience,
        nonce_digest: fpm_attestation_nonce_digest(&nonce),
        eat_profile_uri,
        key_id,
        verification_key_digest: result.verification.verification_key_digest,
    };

    let action_hash = create_entry(EntryTypes::FpmEatCoseVerificationAnchor(anchor))?;
    get(action_hash, GetOptions::default())?.ok_or_else(|| {
        FabricationError::not_found(
            "FpmEatCoseVerificationAnchor",
            &"newly-created action",
        )
    })
}



fn resolve_fpm_attestation_challenge_impl(
    input: ResolveFpmAttestationChallengeInput,
) -> ExternResult<ResolvedFpmAttestationChallenge> {
    let details = get_details(input.action_hash.clone(), GetOptions::network())?
        .ok_or_else(|| FabricationError::not_found(
            "FpmAttestationChallenge",
            &input.action_hash,
        ))?;

    let Details::Record(record_details) = details else {
        return Err(fpm_attestation_error(
            "attestation challenge ActionHash did not resolve to record details",
        ));
    };

    if record_details.validation_status != ValidationStatus::Valid {
        return Err(fpm_attestation_error("attestation challenge record is not valid"));
    }
    if !record_details.updates.is_empty() {
        return Err(fpm_attestation_error("attestation challenge has updates"));
    }
    if !record_details.deletes.is_empty() {
        return Err(fpm_attestation_error("attestation challenge has deletes"));
    }

    let record = record_details.record;
    if record.action().action_type() != ActionType::Create {
        return Err(fpm_attestation_error(
            "attestation challenge must resolve to its original Create action",
        ));
    }

    let expected_entry_type = EntryType::App(
        UnitEntryTypes::FpmAttestationChallenge
            .try_into()
            .map_err(|_| fpm_attestation_error(
                "could not construct FPM attestation challenge entry type",
            ))?,
    );
    if record.action().entry_type() != Some(&expected_entry_type) {
        return Err(fpm_attestation_error(
            "ActionHash does not reference the FPM attestation challenge entry type",
        ));
    }

    let challenge: FpmAttestationChallenge = record
        .entry()
        .to_app_option()
        .map_err(|e| fpm_attestation_error(format!(
            "could not decode FPM attestation challenge: {e}"
        )))?
        .ok_or_else(|| fpm_attestation_error(
            "record is not an FPM attestation challenge entry",
        ))?;

    if challenge.schema_version != FPM_ATTESTATION_CHALLENGE_SCHEMA_VERSION
        || !valid_fpm_digest(&challenge.acquisition_root_digest)
        || !valid_fpm_digest(&challenge.verifier_profile_digest)
        || !valid_fpm_digest(&challenge.appraisal_policy_digest)
        || !valid_fpm_digest(&challenge.reference_values_digest)
        || !valid_fpm_digest(&challenge.endorsement_digest)
        || !valid_attestation_identifier(&challenge.subject_id, 128)
        || !valid_attestation_identifier(&challenge.attestation_format, 128)
        || !(8..=64).contains(&challenge.nonce.len())
        || challenge.verifier_agent != *record.action().author()
    {
        return Err(fpm_attestation_error(
            "FPM attestation challenge declaration is invalid",
        ));
    }

    let trust = resolve_fpm_verification_key_trust_anchor_impl(
        ResolveFpmVerificationKeyTrustAnchorInput {
            action_hash: challenge.verification_key_trust_anchor_action.clone(),
        },
    )?;
    if trust.verifier_agent != challenge.verifier_agent
        || trust.verification_key_id != challenge.verification_key_id
        || trust.verification_key_digest != challenge.verification_key_digest
        || trust.attestation_format != challenge.attestation_format
        || trust.verifier_profile_digest != challenge.verifier_profile_digest
    {
        return Err(fpm_attestation_error(
            "attestation challenge does not inherit its exact trusted verifier-key binding",
        ));
    }

    let implementation = resolve_fpm_verifier_implementation_trust_anchor_impl(
        ResolveFpmVerifierImplementationTrustAnchorInput {
            action_hash: challenge.verifier_implementation_trust_anchor_action.clone(),
        },
    )?;
    if implementation.verifier_agent != challenge.verifier_agent
        || implementation.verification_key_trust_anchor_action
            != challenge.verification_key_trust_anchor_action
        || implementation.implementation_digest != challenge.verifier_implementation_digest
        || implementation.build_provenance_digest != challenge.verifier_build_provenance_digest
        || implementation.builder_id != challenge.verifier_builder_id
        || implementation.verifier_profile_digest != challenge.verifier_profile_digest
    {
        return Err(fpm_attestation_error(
            "attestation challenge does not inherit its exact trusted verifier implementation binding",
        ));
    }

    Ok(ResolvedFpmAttestationChallenge {
        action_hash: input.action_hash,
        subject_id: challenge.subject_id,
        audience: challenge.audience,
        verification_key_trust_anchor_action: challenge.verification_key_trust_anchor_action,
        verifier_implementation_trust_anchor_action: challenge.verifier_implementation_trust_anchor_action,
        verification_key_id: challenge.verification_key_id,
        verification_key_digest: challenge.verification_key_digest,
        verifier_implementation_digest: challenge.verifier_implementation_digest,
        verifier_build_provenance_digest: challenge.verifier_build_provenance_digest,
        verifier_builder_id: challenge.verifier_builder_id,
        acquisition_root_action: challenge.acquisition_root_action,
        acquisition_root_digest: challenge.acquisition_root_digest,
        verifier_agent: challenge.verifier_agent,
        attestation_format: challenge.attestation_format,
        verifier_profile_digest: challenge.verifier_profile_digest,
        appraisal_policy_digest: challenge.appraisal_policy_digest,
        reference_values_digest: challenge.reference_values_digest,
        endorsement_digest: challenge.endorsement_digest,
        nonce_digest: fpm_attestation_nonce_digest(&challenge.nonce),
        nonce: challenge.nonce,
        author: *record.action().author(),
        signer: *record.action().signer(),
        timestamp: record.action().timestamp(),
        action_seq: record.action().action_seq(),
        prev_action: record.action().prev_action().cloned(),
    })
}

fn challenge_has_valid_use(
    challenge: &ResolvedFpmAttestationChallenge,
) -> ExternResult<bool> {
    let links = get_links(
        LinkQuery::try_new(
            challenge.action_hash.clone(),
            LinkTypes::FpmChallengeToUses,
        )?,
        GetStrategy::default(),
    )?;

    for link in links {
        let Some(use_action) = link.target.into_action_hash() else {
            continue;
        };
        let Some(details) = get_details(use_action.clone(), GetOptions::network())? else {
            continue;
        };
        let Details::Record(record_details) = details else {
            continue;
        };
        if record_details.validation_status != ValidationStatus::Valid
            || !record_details.updates.is_empty()
            || !record_details.deletes.is_empty()
        {
            continue;
        }
        let record = record_details.record;
        if record.action().action_type() != ActionType::Create
            || *record.action().author() != challenge.verifier_agent
        {
            continue;
        }
        let Some(use_entry) = record
            .entry()
            .to_app_option::<FpmAttestationChallengeUse>()
            .ok()
            .flatten()
        else {
            continue;
        };
        if use_entry.challenge_action == challenge.action_hash
            && use_entry.author == challenge.verifier_agent
            && use_entry.attestation_action != challenge.action_hash
        {
            let Some(attestation_details) =
                get_details(use_entry.attestation_action.clone(), GetOptions::network())?
            else {
                continue;
            };
            let Details::Record(attestation_details) = attestation_details else {
                continue;
            };
            if attestation_details.validation_status != ValidationStatus::Valid
                || !attestation_details.updates.is_empty()
                || !attestation_details.deletes.is_empty()
            {
                continue;
            }
            let attestation_record = attestation_details.record;
            if attestation_record.action().action_type() != ActionType::Create
                || *attestation_record.action().author() != challenge.verifier_agent
            {
                continue;
            }
            let expected_attestation_type = EntryType::App(
                UnitEntryTypes::FpmSourceAttestationAnchor
                    .try_into()
                    .map_err(|_| fpm_attestation_error(
                        "could not construct FPM source attestation entry type",
                    ))?,
            );
            if attestation_record.action().entry_type() != Some(&expected_attestation_type) {
                continue;
            }
            let Some(attestation_anchor) = attestation_record
                .entry()
                .to_app_option::<FpmSourceAttestationAnchor>()
                .ok()
                .flatten()
            else {
                continue;
            };
            if attestation_anchor.challenge_action == challenge.action_hash {
                return Ok(true);
            }
        }
    }

    Ok(false)
}

fn validate_attestation_claim_against_challenge(
    claim: &FpmSourceAttestationClaim,
    challenge: &ResolvedFpmAttestationChallenge,
) -> ExternResult<FpmAttestationQualification> {
    Ok(qualify_source_attestation(&FpmAttestationQualificationInput {
        expected_subject_id: challenge.subject_id.clone(),
        expected_audience: challenge.audience.clone(),
        expected_verification_key_id: challenge.verification_key_id.clone(),
        expected_verification_key_digest: challenge.verification_key_digest.clone(),
        expected_acquisition_root_digest: challenge.acquisition_root_digest.clone(),
        expected_challenge_nonce_digest: challenge.nonce_digest.clone(),
        expected_attestation_format: challenge.attestation_format.clone(),
        expected_verifier_profile_digest: challenge.verifier_profile_digest.clone(),
        expected_appraisal_policy_digest: challenge.appraisal_policy_digest.clone(),
        expected_reference_values_digest: challenge.reference_values_digest.clone(),
        expected_endorsement_digest: challenge.endorsement_digest.clone(),
        claim: claim.clone(),
    }))
}

fn resolve_fpm_source_attestation_anchor_impl(
    input: ResolveFpmSourceAttestationAnchorInput,
) -> ExternResult<ResolvedFpmSourceAttestationAnchor> {
    let details = get_details(input.action_hash.clone(), GetOptions::network())?
        .ok_or_else(|| FabricationError::not_found(
            "FpmSourceAttestationAnchor",
            &input.action_hash,
        ))?;
    let Details::Record(record_details) = details else {
        return Err(fpm_attestation_error(
            "source attestation ActionHash did not resolve to record details",
        ));
    };
    if record_details.validation_status != ValidationStatus::Valid
        || !record_details.updates.is_empty()
        || !record_details.deletes.is_empty()
    {
        return Err(fpm_attestation_error(
            "source attestation record is not currently valid and immutable",
        ));
    }

    let record = record_details.record;
    if record.action().action_type() != ActionType::Create {
        return Err(fpm_attestation_error(
            "source attestation ActionHash must resolve to the original Create action",
        ));
    }
    let expected_entry_type = EntryType::App(
        UnitEntryTypes::FpmSourceAttestationAnchor
            .try_into()
            .map_err(|_| fpm_attestation_error(
                "could not construct FPM source attestation entry type",
            ))?,
    );
    if record.action().entry_type() != Some(&expected_entry_type) {
        return Err(fpm_attestation_error(
            "ActionHash does not reference the FPM source attestation entry type",
        ));
    }

    let anchor: FpmSourceAttestationAnchor = record
        .entry()
        .to_app_option()
        .map_err(|e| fpm_attestation_error(format!(
            "could not decode FPM source attestation anchor: {e}"
        )))?
        .ok_or_else(|| fpm_attestation_error(
            "record is not an FPM source attestation anchor entry",
        ))?;

    if anchor.schema_version != FPM_SOURCE_ATTESTATION_ANCHOR_SCHEMA_VERSION
        || anchor.claim_digest != anchor.claim.digest()
    {
        return Err(fpm_attestation_error(
            "source attestation claim digest is invalid or mismatched",
        ));
    }

    let verification =
        resolve_fpm_eat_cose_verification_anchor_impl(anchor.eat_cose_verification_action.clone())?;
    if verification.challenge_action != anchor.challenge_action
        || verification.evidence_digest != anchor.claim.evidence_digest
        || verification.subject_id != anchor.claim.subject_id
        || verification.audience != anchor.claim.audience
        || verification.key_id != anchor.claim.verification_key_id
        || verification.verification_key_digest != anchor.claim.verification_key_digest
        || verification.nonce_digest != anchor.claim.challenge_nonce_digest
    {
        return Err(fpm_attestation_error(
            "source attestation is not exactly bound to its EAT/COSE verification",
        ));
    }

    let challenge = resolve_fpm_attestation_challenge_impl(
        ResolveFpmAttestationChallengeInput {
            action_hash: anchor.challenge_action.clone(),
        },
    )?;

    if *record.action().author() != challenge.verifier_agent {
        return Err(fpm_attestation_error(
            "source attestation author does not match challenge verifier",
        ));
    }

    let qualification = validate_attestation_claim_against_challenge(
        &anchor.claim,
        &challenge,
    )?;
    if matches!(
        qualification.status,
        FpmAttestationQualificationStatus::InvalidEvidence
            | FpmAttestationQualificationStatus::ConflictingAttestation
    ) {
        return Err(fpm_attestation_error(
            "source attestation claim conflicts with its authenticated challenge",
        ));
    }

    let entry_hash = record
        .action()
        .entry_hash()
        .ok_or_else(|| fpm_attestation_error(
            "source attestation action has no entry hash",
        ))?
        .clone();

    Ok(ResolvedFpmSourceAttestationAnchor {
        action_hash: input.action_hash,
        entry_hash,
        challenge_action: anchor.challenge_action,
        claim: anchor.claim,
        claim_digest: anchor.claim_digest,
        author: *record.action().author(),
        signer: *record.action().signer(),
        timestamp: record.action().timestamp(),
        action_seq: record.action().action_seq(),
        prev_action: record.action().prev_action().cloned(),
    })
}

fn create_fpm_verification_key_trust_anchor_impl(
    input: CreateFpmVerificationKeyTrustAnchorInput,
) -> ExternResult<Record> {
    let authority = FabricationDnaProperties::fpm_verifier_trust_authority()?;
    let current_agent = agent_info()?.agent_initial_pubkey;
    if current_agent != authority {
        return Err(fpm_attestation_error(
            "only the DNA-configured FPM trust authority may provision verifier keys",
        ));
    }

    if input.verification_key_id.is_empty()
        || input.verification_key_id.len() > 128
        || !is_valid_fpm_p256_public_key(&input.public_key_sec1)
        || !valid_attestation_identifier(&input.attestation_format, 128)
        || !valid_fpm_digest(&input.verifier_profile_digest)
    {
        return Err(fpm_attestation_error(
            "verifier-key trust anchor inputs are malformed",
        ));
    }

    let verification_key_digest = fpm_verification_key_digest(&input.public_key_sec1);
    let anchor = FpmVerificationKeyTrustAnchor {
        schema_version: FPM_VERIFICATION_KEY_TRUST_ANCHOR_SCHEMA_VERSION.into(),
        verifier_agent: input.verifier_agent,
        key_id: input.verification_key_id,
        public_key_sec1: input.public_key_sec1,
        verification_key_digest,
        attestation_format: input.attestation_format,
        verifier_profile_digest: input.verifier_profile_digest,
    };

    let action_hash = create_entry(EntryTypes::FpmVerificationKeyTrustAnchor(anchor))?;
    get(action_hash.clone(), GetOptions::default())?.ok_or_else(|| {
        FabricationError::not_found("FpmVerificationKeyTrustAnchor", &action_hash)
    })
}

fn resolve_fpm_verification_key_trust_anchor_impl(
    input: ResolveFpmVerificationKeyTrustAnchorInput,
) -> ExternResult<ResolvedFpmVerificationKeyTrustAnchor> {
    let details = get_details(input.action_hash.clone(), GetOptions::network())?
        .ok_or_else(|| FabricationError::not_found(
            "FpmVerificationKeyTrustAnchor",
            &input.action_hash,
        ))?;
    let Details::Record(record_details) = details else {
        return Err(fpm_attestation_error(
            "verifier-key trust-anchor ActionHash did not resolve to record details",
        ));
    };
    if record_details.validation_status != ValidationStatus::Valid
        || !record_details.updates.is_empty()
        || !record_details.deletes.is_empty()
    {
        return Err(fpm_attestation_error(
            "verifier-key trust anchor is not currently valid and immutable",
        ));
    }

    let record = record_details.record;
    if record.action().action_type() != ActionType::Create {
        return Err(fpm_attestation_error(
            "verifier-key trust anchor must resolve to its original Create action",
        ));
    }
    let expected_entry_type = EntryType::App(
        UnitEntryTypes::FpmVerificationKeyTrustAnchor
            .try_into()
            .map_err(|_| fpm_attestation_error(
                "could not construct FPM verifier-key trust-anchor entry type",
            ))?,
    );
    if record.action().entry_type() != Some(&expected_entry_type) {
        return Err(fpm_attestation_error(
            "ActionHash does not reference an FPM verifier-key trust anchor",
        ));
    }

    let authority = FabricationDnaProperties::fpm_verifier_trust_authority()?;
    if *record.action().author() != authority {
        return Err(fpm_attestation_error(
            "verifier-key trust anchor was not provisioned by the DNA-configured authority",
        ));
    }

    let anchor: FpmVerificationKeyTrustAnchor = record
        .entry()
        .to_app_option()
        .map_err(|e| fpm_attestation_error(format!(
            "could not decode FPM verifier-key trust anchor: {e}"
        )))?
        .ok_or_else(|| fpm_attestation_error(
            "record is not an FPM verifier-key trust anchor entry",
        ))?;

    if anchor.schema_version != FPM_VERIFICATION_KEY_TRUST_ANCHOR_SCHEMA_VERSION
        || anchor.key_id.is_empty()
        || anchor.key_id.len() > 128
        || !is_valid_fpm_p256_public_key(&anchor.public_key_sec1)
        || !valid_attestation_identifier(&anchor.attestation_format, 128)
        || !valid_fpm_digest(&anchor.verification_key_digest)
        || fpm_verification_key_digest(&anchor.public_key_sec1) != anchor.verification_key_digest
        || !valid_fpm_digest(&anchor.verifier_profile_digest)
    {
        return Err(fpm_attestation_error(
            "verifier-key trust anchor contains malformed or inconsistent commitments",
        ));
    }

    let entry_hash = record.action().entry_hash().ok_or_else(|| {
        fpm_attestation_error("verifier-key trust-anchor action has no entry hash")
    })?.clone();

    Ok(ResolvedFpmVerificationKeyTrustAnchor {
        action_hash: input.action_hash,
        entry_hash,
        verifier_agent: anchor.verifier_agent,
        verification_key_id: anchor.key_id,
        public_key_sec1: anchor.public_key_sec1,
        verification_key_digest: anchor.verification_key_digest,
        attestation_format: anchor.attestation_format,
        verifier_profile_digest: anchor.verifier_profile_digest,
        authority_agent: authority,
        timestamp: record.action().timestamp(),
        action_seq: record.action().action_seq(),
        prev_action: record.action().prev_action().cloned(),
    })
}

fn create_fpm_verifier_implementation_trust_anchor_impl(
    input: CreateFpmVerifierImplementationTrustAnchorInput,
) -> ExternResult<Record> {
    let authority = FabricationDnaProperties::fpm_verifier_trust_authority()?;
    let current_agent = agent_info()?.agent_initial_pubkey;
    if current_agent != authority {
        return Err(fpm_attestation_error(
            "only the DNA-configured FPM trust authority may provision verifier implementations",
        ));
    }

    let key = resolve_fpm_verification_key_trust_anchor_impl(
        ResolveFpmVerificationKeyTrustAnchorInput {
            action_hash: input.verification_key_trust_anchor_action.clone(),
        },
    )?;
    if key.verifier_agent != input.verifier_agent {
        return Err(fpm_attestation_error(
            "verifier implementation trust anchor must name the verifier bound to its key trust anchor",
        ));
    }
    if !is_canonical_fpm_verifier_digest(&input.implementation_digest)
        || !is_canonical_fpm_verifier_digest(&input.build_provenance_digest)
        || !is_valid_fpm_verifier_builder_id(&input.builder_id)
    {
        return Err(fpm_attestation_error(
            "verifier implementation trust anchor inputs are malformed",
        ));
    }

    let anchor = FpmVerifierImplementationTrustAnchor {
        schema_version: FPM_VERIFIER_IMPLEMENTATION_TRUST_ANCHOR_SCHEMA_VERSION.into(),
        verifier_agent: input.verifier_agent,
        verification_key_trust_anchor_action: key.action_hash,
        implementation_digest: input.implementation_digest,
        build_provenance_digest: input.build_provenance_digest,
        builder_id: input.builder_id,
        verifier_profile_digest: key.verifier_profile_digest,
    };

    let action_hash = create_entry(EntryTypes::FpmVerifierImplementationTrustAnchor(anchor))?;
    get(action_hash.clone(), GetOptions::default())?.ok_or_else(|| {
        FabricationError::not_found("FpmVerifierImplementationTrustAnchor", &action_hash)
    })
}

fn resolve_fpm_verifier_implementation_trust_anchor_impl(
    input: ResolveFpmVerifierImplementationTrustAnchorInput,
) -> ExternResult<ResolvedFpmVerifierImplementationTrustAnchor> {
    let details = get_details(input.action_hash.clone(), GetOptions::network())?
        .ok_or_else(|| FabricationError::not_found(
            "FpmVerifierImplementationTrustAnchor",
            &input.action_hash,
        ))?;
    let Details::Record(record_details) = details else {
        return Err(fpm_attestation_error(
            "verifier implementation trust-anchor ActionHash did not resolve to record details",
        ));
    };
    if record_details.validation_status != ValidationStatus::Valid
        || !record_details.updates.is_empty()
        || !record_details.deletes.is_empty()
    {
        return Err(fpm_attestation_error(
            "verifier implementation trust anchor is not currently valid and immutable",
        ));
    }

    let record = record_details.record;
    if record.action().action_type() != ActionType::Create {
        return Err(fpm_attestation_error(
            "verifier implementation trust anchor must resolve to its original Create action",
        ));
    }
    let expected_entry_type = EntryType::App(
        UnitEntryTypes::FpmVerifierImplementationTrustAnchor
            .try_into()
            .map_err(|_| fpm_attestation_error(
                "could not construct FPM verifier implementation trust-anchor entry type",
            ))?,
    );
    if record.action().entry_type() != Some(&expected_entry_type) {
        return Err(fpm_attestation_error(
            "ActionHash does not reference an FPM verifier implementation trust anchor",
        ));
    }
    let authority = FabricationDnaProperties::fpm_verifier_trust_authority()?;
    if *record.action().author() != authority {
        return Err(fpm_attestation_error(
            "verifier implementation trust anchor was not provisioned by the DNA-configured authority",
        ));
    }
    let anchor: FpmVerifierImplementationTrustAnchor = record
        .entry()
        .to_app_option()
        .map_err(|e| fpm_attestation_error(format!(
            "could not decode FPM verifier implementation trust anchor: {e}"
        )))?
        .ok_or_else(|| fpm_attestation_error(
            "record is not an FPM verifier implementation trust anchor entry",
        ))?;
    if !validate_fpm_verifier_implementation_identity_fields(&anchor) {
        return Err(fpm_attestation_error(
            "verifier implementation trust anchor contains malformed identity fields",
        ));
    }
    let key = resolve_fpm_verification_key_trust_anchor_impl(
        ResolveFpmVerificationKeyTrustAnchorInput {
            action_hash: anchor.verification_key_trust_anchor_action.clone(),
        },
    )?;
    if key.verifier_agent != anchor.verifier_agent
        || key.verifier_profile_digest != anchor.verifier_profile_digest
    {
        return Err(fpm_attestation_error(
            "verifier implementation trust anchor does not exactly inherit its key trust binding",
        ));
    }
    let entry_hash = record.action().entry_hash().ok_or_else(|| {
        fpm_attestation_error("verifier implementation trust-anchor action has no entry hash")
    })?.clone();
    Ok(ResolvedFpmVerifierImplementationTrustAnchor {
        action_hash: input.action_hash,
        entry_hash,
        verifier_agent: anchor.verifier_agent,
        verification_key_trust_anchor_action: anchor.verification_key_trust_anchor_action,
        implementation_digest: anchor.implementation_digest,
        build_provenance_digest: anchor.build_provenance_digest,
        builder_id: anchor.builder_id,
        verifier_profile_digest: anchor.verifier_profile_digest,
        authority_agent: authority,
        timestamp: record.action().timestamp(),
        action_seq: record.action().action_seq(),
        prev_action: record.action().prev_action().cloned(),
    })
}

fn create_fpm_attestation_challenge_impl(
    input: CreateFpmAttestationChallengeInput,
) -> ExternResult<Record> {
    let root = resolve_acquisition_root_action_anchor(
        ResolveFpmAcquisitionRootActionAnchorInput {
            action_hash: input.acquisition_root_action.clone(),
            expected_root_digest: None,
        },
    )?;
    let trust = resolve_fpm_verification_key_trust_anchor_impl(
        ResolveFpmVerificationKeyTrustAnchorInput {
            action_hash: input.verification_key_trust_anchor_action.clone(),
        },
    )?;
    let implementation = resolve_fpm_verifier_implementation_trust_anchor_impl(
        ResolveFpmVerifierImplementationTrustAnchorInput {
            action_hash: input.verifier_implementation_trust_anchor_action.clone(),
        },
    )?;
    if implementation.verification_key_trust_anchor_action != trust.action_hash
        || implementation.verifier_agent != trust.verifier_agent
        || implementation.verifier_profile_digest != trust.verifier_profile_digest
    {
        return Err(fpm_attestation_error(
            "verifier implementation trust anchor does not match the selected verification key trust anchor",
        ));
    }
    let verifier_agent = agent_info()?.agent_initial_pubkey;
    if verifier_agent != implementation.verifier_agent {
        return Err(fpm_attestation_error(
            "only the verifier bound to the trusted implementation may issue this challenge",
        ));
    }

    if !valid_attestation_identifier(&input.audience, 256)
        || !valid_fpm_digest(&input.appraisal_policy_digest)
        || !valid_fpm_digest(&input.reference_values_digest)
        || !valid_fpm_digest(&input.endorsement_digest)
    {
        return Err(fpm_attestation_error(
            "attestation challenge relying-party policy inputs are malformed",
        ));
    }

    let nonce = random_bytes(32)
        .map_err(|e| fpm_attestation_error(format!(
            "cryptographic challenge nonce generation failed: {e}"
        )))?
        .as_ref()
        .to_vec();

    let challenge = FpmAttestationChallenge {
        schema_version: FPM_ATTESTATION_CHALLENGE_SCHEMA_VERSION.into(),
        subject_id: root.source_system_id,
        audience: input.audience,
        verification_key_trust_anchor_action: trust.action_hash,
        verifier_implementation_trust_anchor_action: implementation.action_hash,
        verification_key_id: trust.verification_key_id,
        verification_key_digest: trust.verification_key_digest,
        verifier_implementation_digest: implementation.implementation_digest,
        verifier_build_provenance_digest: implementation.build_provenance_digest,
        verifier_builder_id: implementation.builder_id,
        acquisition_root_action: input.acquisition_root_action,
        acquisition_root_digest: root.root_digest,
        verifier_agent,
        attestation_format: trust.attestation_format,
        verifier_profile_digest: trust.verifier_profile_digest,
        appraisal_policy_digest: input.appraisal_policy_digest,
        reference_values_digest: input.reference_values_digest,
        endorsement_digest: input.endorsement_digest,
        nonce,
    };

    let action_hash = create_entry(EntryTypes::FpmAttestationChallenge(challenge))?;
    get(action_hash, GetOptions::default())?
        .ok_or_else(|| FabricationError::not_found(
            "FpmAttestationChallenge",
            &"newly-created action",
        ))
}

fn create_fpm_source_attestation_anchor_impl(
    input: CreateFpmSourceAttestationAnchorInput,
) -> ExternResult<Record> {
    let challenge = resolve_fpm_attestation_challenge_impl(
        ResolveFpmAttestationChallengeInput {
            action_hash: input.challenge_action.clone(),
        },
    )?;

    let current_agent = agent_info()?.agent_initial_pubkey;
    if current_agent != challenge.verifier_agent {
        return Err(fpm_attestation_error(
            "only the verifier that issued the challenge may create its attestation result",
        ));
    }

    let root = resolve_acquisition_root_action_anchor(
        ResolveFpmAcquisitionRootActionAnchorInput {
            action_hash: challenge.acquisition_root_action.clone(),
            expected_root_digest: Some(challenge.acquisition_root_digest.clone()),
        },
    )?;
    if root.source_system_id != challenge.subject_id {
        return Err(fpm_attestation_error(
            "challenge subject does not match authenticated acquisition-root source",
        ));
    }

    let verification =
        resolve_fpm_eat_cose_verification_anchor_impl(input.eat_cose_verification_action.clone())?;
    if verification.challenge_action != input.challenge_action {
        return Err(fpm_attestation_error(
            "cryptographic verification anchor references a different challenge",
        ));
    }
    if verification.author != challenge.verifier_agent {
        return Err(fpm_attestation_error(
            "cryptographic verification anchor was not created by the challenge verifier",
        ));
    }
    if !valid_attestation_identifier(&input.verifier_id, 128)
        || !valid_attestation_identifier(&input.verifier_version, 128)
    {
        return Err(fpm_attestation_error(
            "source attestation verifier declaration is malformed",
        ));
    }

    let claim = FpmSourceAttestationClaim {
        subject_id: challenge.subject_id.clone(),
        audience: challenge.audience.clone(),
        verification_key_id: challenge.verification_key_id.clone(),
        verification_key_digest: challenge.verification_key_digest.clone(),
        acquisition_root_digest: challenge.acquisition_root_digest.clone(),
        challenge_nonce_digest: challenge.nonce_digest.clone(),
        evidence_digest: verification.evidence_digest.clone(),
        attestation_format: challenge.attestation_format.clone(),
        verifier_id: input.verifier_id,
        verifier_version: input.verifier_version,
        verifier_profile_digest: challenge.verifier_profile_digest.clone(),
        appraisal_policy_digest: challenge.appraisal_policy_digest.clone(),
        reference_values_digest: challenge.reference_values_digest.clone(),
        endorsement_digest: challenge.endorsement_digest.clone(),
        disposition: input.disposition,
    };

    let qualification = validate_attestation_claim_against_challenge(
        &claim,
        &challenge,
    )?;
    if qualification.status == FpmAttestationQualificationStatus::InvalidEvidence {
        return Err(fpm_attestation_error(
            "source attestation declaration is structurally invalid",
        ));
    }

    let anchor = FpmSourceAttestationAnchor {
        schema_version: FPM_SOURCE_ATTESTATION_ANCHOR_SCHEMA_VERSION.into(),
        challenge_action: input.challenge_action,
        eat_cose_verification_action: input.eat_cose_verification_action,
        claim_digest: claim.digest(),
        claim,
    };

    let action_hash = create_entry(EntryTypes::FpmSourceAttestationAnchor(anchor))?;
    get(action_hash, GetOptions::default())?
        .ok_or_else(|| FabricationError::not_found(
            "FpmSourceAttestationAnchor",
            &"newly-created action",
        ))
}

fn qualify_fpm_source_attestation_impl(
    input: QualifyFpmSourceAttestationInput,
) -> ExternResult<FpmSourceAttestationQualificationResult> {
    let challenge = resolve_fpm_attestation_challenge_impl(
        ResolveFpmAttestationChallengeInput {
            action_hash: input.challenge_action.clone(),
        },
    )?;

    let current_agent = agent_info()?.agent_initial_pubkey;
    if current_agent != challenge.verifier_agent {
        return Err(fpm_attestation_error(
            "only the challenge verifier may consume an attestation challenge",
        ));
    }

    let root = resolve_acquisition_root_action_anchor(
        ResolveFpmAcquisitionRootActionAnchorInput {
            action_hash: challenge.acquisition_root_action.clone(),
            expected_root_digest: Some(challenge.acquisition_root_digest.clone()),
        },
    )?;
    if root.source_system_id != challenge.subject_id
        || root.root_digest != challenge.acquisition_root_digest
    {
        return Err(fpm_attestation_error(
            "challenge no longer matches the authenticated acquisition root",
        ));
    }

    if challenge_has_valid_use(&challenge)? {
        return Err(fpm_attestation_error(
            "attestation challenge has already been consumed",
        ));
    }

    let attestation = resolve_fpm_source_attestation_anchor_impl(
        ResolveFpmSourceAttestationAnchorInput {
            action_hash: input.attestation_action.clone(),
        },
    )?;
    if attestation.challenge_action != input.challenge_action {
        return Err(fpm_attestation_error(
            "attestation result references a different challenge",
        ));
    }

    let qualification = validate_attestation_claim_against_challenge(
        &attestation.claim,
        &challenge,
    )?;

    if matches!(
        qualification.status,
        FpmAttestationQualificationStatus::InvalidEvidence
            | FpmAttestationQualificationStatus::ConflictingAttestation
    ) {
        return Ok(FpmSourceAttestationQualificationResult {
            qualification,
            challenge_action: input.challenge_action,
            attestation_action: input.attestation_action,
            consumed: false,
        });
    }

    let verifier_agent = challenge.verifier_agent;
    let use_entry = FpmAttestationChallengeUse {
        schema_version: FPM_ATTESTATION_CHALLENGE_USE_SCHEMA_VERSION.into(),
        challenge_action: input.challenge_action.clone(),
        attestation_action: input.attestation_action.clone(),
        author: verifier_agent,
    };
    let use_action = create_entry(EntryTypes::FpmAttestationChallengeUse(use_entry))?;
    create_link(
        input.challenge_action.clone(),
        use_action,
        LinkTypes::FpmChallengeToUses,
        (),
    )?;

    Ok(FpmSourceAttestationQualificationResult {
        qualification,
        challenge_action: input.challenge_action,
        attestation_action: input.attestation_action,
        consumed: true,
    })
}

#[hdk_extern]
pub fn create_fpm_verification_key_trust_anchor(
    input: CreateFpmVerificationKeyTrustAnchorInput,
) -> ExternResult<Record> {
    rate_limit_caller()?;
    create_fpm_verification_key_trust_anchor_impl(input)
}

#[hdk_extern]
pub fn create_fpm_verifier_implementation_trust_anchor(
    input: CreateFpmVerifierImplementationTrustAnchorInput,
) -> ExternResult<Record> {
    rate_limit_caller()?;
    create_fpm_verifier_implementation_trust_anchor_impl(input)
}

#[hdk_extern]
pub fn resolve_fpm_verifier_implementation_trust_anchor(
    input: ResolveFpmVerifierImplementationTrustAnchorInput,
) -> ExternResult<ResolvedFpmVerifierImplementationTrustAnchor> {
    rate_limit_caller()?;
    resolve_fpm_verifier_implementation_trust_anchor_impl(input)
}

#[hdk_extern]
pub fn resolve_fpm_verification_key_trust_anchor(
    input: ResolveFpmVerificationKeyTrustAnchorInput,
) -> ExternResult<ResolvedFpmVerificationKeyTrustAnchor> {
    rate_limit_caller()?;
    resolve_fpm_verification_key_trust_anchor_impl(input)
}

#[hdk_extern]
pub fn verify_fpm_eat_cose_against_challenge(
    input: VerifyFpmEatCoseAgainstChallengeInput,
) -> ExternResult<FpmChallengeEatCoseVerification> {
    rate_limit_caller()?;
    verify_fpm_eat_cose_against_challenge_impl(input)
}

#[hdk_extern]
pub fn create_fpm_eat_cose_verification_anchor(
    input: CreateFpmEatCoseVerificationAnchorInput,
) -> ExternResult<Record> {
    rate_limit_caller()?;
    create_fpm_eat_cose_verification_anchor_impl(input)
}

#[hdk_extern]
pub fn resolve_fpm_eat_cose_verification_anchor(
    action_hash: ActionHash,
) -> ExternResult<ResolvedFpmEatCoseVerificationAnchor> {
    rate_limit_caller()?;
    resolve_fpm_eat_cose_verification_anchor_impl(action_hash)
}

#[hdk_extern]
pub fn create_fpm_attestation_challenge(
    input: CreateFpmAttestationChallengeInput,
) -> ExternResult<Record> {
    rate_limit_caller()?;
    create_fpm_attestation_challenge_impl(input)
}

#[hdk_extern]
pub fn resolve_fpm_attestation_challenge(
    input: ResolveFpmAttestationChallengeInput,
) -> ExternResult<ResolvedFpmAttestationChallenge> {
    rate_limit_caller()?;
    resolve_fpm_attestation_challenge_impl(input)
}

#[hdk_extern]
pub fn create_fpm_source_attestation_anchor(
    input: CreateFpmSourceAttestationAnchorInput,
) -> ExternResult<Record> {
    rate_limit_caller()?;
    create_fpm_source_attestation_anchor_impl(input)
}

#[hdk_extern]
pub fn resolve_fpm_source_attestation_anchor(
    input: ResolveFpmSourceAttestationAnchorInput,
) -> ExternResult<ResolvedFpmSourceAttestationAnchor> {
    rate_limit_caller()?;
    resolve_fpm_source_attestation_anchor_impl(input)
}

#[hdk_extern]
pub fn qualify_fpm_source_attestation(
    input: QualifyFpmSourceAttestationInput,
) -> ExternResult<FpmSourceAttestationQualificationResult> {
    rate_limit_caller()?;
    qualify_fpm_source_attestation_impl(input)
}

fn valid_fpm_digest(value: &str) -> bool {
    value.len() == 64
        && value.bytes().all(|byte| matches!(byte, b'0'..=b'9' | b'a'..=b'f'))
}

fn fpm_anchor_error(reason: impl Into<String>) -> WasmError {
    FabricationError::ValidationFailed {
        field: "fpm_registration_anchor".into(),
        reason: reason.into(),
    }
    .to_wasm_error()
}

fn validate_resolved_anchor_envelope(
    anchor: &FpmRegistrationAnchor,
    claimed_envelope_digest: &str,
) -> ExternResult<()> {
    if anchor.schema_version != FPM_REGISTRATION_ANCHOR_SCHEMA_VERSION {
        return Err(fpm_anchor_error("unsupported registration anchor schema"));
    }
    if !valid_fpm_digest(claimed_envelope_digest) {
        return Err(fpm_anchor_error("claimed envelope digest is not canonical SHA-256"));
    }
    if !valid_fpm_digest(&anchor.envelope_digest) {
        return Err(fpm_anchor_error("stored envelope digest is not canonical SHA-256"));
    }
    if anchor.envelope_digest != claimed_envelope_digest {
        return Err(fpm_anchor_error("claimed envelope digest does not match stored anchor"));
    }
    let computed = anchor.envelope.digest().map_err(|e| {
        fpm_anchor_error(format!("failed to hash anchored registration envelope: {e}"))
    })?;
    if computed != anchor.envelope_digest {
        return Err(fpm_anchor_error("stored envelope digest does not match envelope content"));
    }
    Ok(())
}

#[hdk_extern]
pub fn create_fpm_registration_anchor(
    input: CreateFpmRegistrationAnchorInput,
) -> ExternResult<Record> {
    rate_limit_caller()?;

    let envelope_digest = input.envelope.digest().map_err(|e| {
        fpm_anchor_error(format!("failed to hash registration envelope: {e}"))
    })?;

    let anchor = FpmRegistrationAnchor {
        schema_version: FPM_REGISTRATION_ANCHOR_SCHEMA_VERSION.into(),
        envelope: input.envelope,
        envelope_digest,
    };

    let action_hash =
        create_entry(EntryTypes::FpmRegistrationAnchor(anchor))?;

    get(action_hash, GetOptions::default())?
        .ok_or_else(|| FabricationError::not_found(
            "FpmRegistrationAnchor",
            &"newly-created action",
        ))
}

fn resolve_action_anchor(
    input: ResolveFpmRegistrationActionAnchorInput,
) -> ExternResult<ResolvedFpmRegistrationAnchor> {
    let details = get_details(input.action_hash.clone(), GetOptions::network())?
        .ok_or_else(|| FabricationError::not_found("FpmRegistrationAnchor", &input.action_hash))?;

    let Details::Record(record_details) = details else {
        return Err(fpm_anchor_error("ActionHash did not resolve to record details"));
    };

    if record_details.validation_status != ValidationStatus::Valid {
        return Err(fpm_anchor_error("registration anchor record is not valid"));
    }
    if !record_details.updates.is_empty() {
        return Err(fpm_anchor_error("registration anchor record has updates"));
    }
    if !record_details.deletes.is_empty() {
        return Err(fpm_anchor_error("registration anchor record has deletes"));
    }

    let record = record_details.record;
    if record.action().action_type() != ActionType::Create {
        return Err(fpm_anchor_error(
            "ActionHash anchor must resolve to the original Create action",
        ));
    }
    let expected_entry_type = EntryType::App(
        UnitEntryTypes::FpmRegistrationAnchor
            .try_into()
            .map_err(|_| fpm_anchor_error("could not construct FPM registration anchor entry type"))?,
    );
    if record.action().entry_type() != Some(&expected_entry_type) {
        return Err(fpm_anchor_error(
            "ActionHash does not reference the FPM registration anchor entry type",
        ));
    }
    let anchor: FpmRegistrationAnchor = record
        .entry()
        .to_app_option()
        .map_err(|e| fpm_anchor_error(format!("could not decode registration anchor: {e}")))?
        .ok_or_else(|| fpm_anchor_error("record is not an FPM registration anchor entry"))?;

    validate_resolved_anchor_envelope(&anchor, &input.claimed_envelope_digest)?;

    let author = *record.action().author();
    if let Some(expected_author) = input.expected_author.as_ref() {
        if *expected_author != author {
            return Err(fpm_anchor_error("ActionHash author does not match expected author"));
        }
    }

    let entry_hash = record
        .action()
        .entry_hash()
        .ok_or_else(|| fpm_anchor_error("registration anchor action has no entry hash"))?;
    let signer = *record.action().signer();
    if let Some(expected_signer) = input.expected_signer.as_ref() {
        if *expected_signer != signer {
            return Err(fpm_anchor_error("ActionHash signer does not match expected signer"));
        }
    }

    Ok(ResolvedFpmRegistrationAnchor {
        anchor_kind: RegistrationAnchorKind::HolochainAction,
        anchor_reference: format!("holochain-action:{input_action}", input_action = input.action_hash),
        registration_envelope_digest: anchor.envelope_digest,
        entry_hash,
        action_hash: Some(input.action_hash),
        author: Some(author),
        signer: Some(signer),
        timestamp: Some(record.action().timestamp()),
        action_seq: Some(record.action().action_seq()),
        prev_action: record.action().prev_action().cloned(),
        envelope: anchor.envelope,
    })
}

fn resolve_entry_anchor(
    input: ResolveFpmRegistrationEntryAnchorInput,
) -> ExternResult<ResolvedFpmRegistrationAnchor> {
    let details = get_details(input.entry_hash.clone(), GetOptions::network())?
        .ok_or_else(|| FabricationError::not_found("FpmRegistrationAnchor", &input.entry_hash))?;

    let Details::Entry(entry_details) = details else {
        return Err(fpm_anchor_error("EntryHash did not resolve to entry details"));
    };

    if entry_details.entry_dht_status != EntryDhtStatus::Live {
        return Err(fpm_anchor_error("registration anchor entry is not live"));
    }
    if !entry_details.rejected_actions.is_empty() {
        return Err(fpm_anchor_error("registration anchor entry has rejected creation actions"));
    }
    if !entry_details.deletes.is_empty() {
        return Err(fpm_anchor_error("registration anchor entry has deletes"));
    }
    if !entry_details.updates.is_empty() {
        return Err(fpm_anchor_error("registration anchor entry has updates"));
    }

    let record = get(input.entry_hash.clone(), GetOptions::network())?
        .ok_or_else(|| FabricationError::not_found("FpmRegistrationAnchor", &input.entry_hash))?;

    let actual_entry_hash = record
        .action()
        .entry_hash()
        .ok_or_else(|| fpm_anchor_error("resolved entry record has no entry hash"))?;
    if actual_entry_hash != input.entry_hash {
        return Err(fpm_anchor_error("resolved record entry hash does not match requested EntryHash"));
    }

    let expected_entry_type = EntryType::App(
        UnitEntryTypes::FpmRegistrationAnchor
            .try_into()
            .map_err(|_| fpm_anchor_error("could not construct FPM registration anchor entry type"))?,
    );
    if record.action().entry_type() != Some(&expected_entry_type) {
        return Err(fpm_anchor_error(
            "EntryHash does not resolve to the FPM registration anchor entry type",
        ));
    }

    let anchor: FpmRegistrationAnchor = record
        .entry()
        .to_app_option()
        .map_err(|e| fpm_anchor_error(format!("could not decode registration anchor: {e}")))?
        .ok_or_else(|| fpm_anchor_error("entry is not an FPM registration anchor entry"))?;

    validate_resolved_anchor_envelope(&anchor, &input.claimed_envelope_digest)?;

    Ok(ResolvedFpmRegistrationAnchor {
        anchor_kind: RegistrationAnchorKind::HolochainEntry,
        anchor_reference: format!("holochain-entry:{input_entry}", input_entry = input.entry_hash),
        registration_envelope_digest: anchor.envelope_digest,
        entry_hash: input.entry_hash,
        action_hash: None,
        author: None,
        signer: None,
        timestamp: None,
        action_seq: None,
        prev_action: None,
        envelope: anchor.envelope,
    })
}

#[hdk_extern]
pub fn resolve_fpm_registration_action_anchor(
    input: ResolveFpmRegistrationActionAnchorInput,
) -> ExternResult<ResolvedFpmRegistrationAnchor> {
    rate_limit_caller()?;
    resolve_action_anchor(input)
}

#[hdk_extern]
pub fn resolve_fpm_registration_entry_anchor(
    input: ResolveFpmRegistrationEntryAnchorInput,
) -> ExternResult<ResolvedFpmRegistrationAnchor> {
    rate_limit_caller()?;
    resolve_entry_anchor(input)
}


fn validate_provenance_witness_against_envelope(
    witness: &AcquisitionLineageWitness,
    envelope: &RegistrationEnvelope,
) -> ExternResult<()> {
    let participant = std::iter::once(&envelope.reference)
        .chain(envelope.related.iter())
        .find(|candidate| {
            candidate.source_id == witness.source_id && candidate.modality == witness.modality
        })
        .ok_or_else(|| fpm_anchor_error(
            "provenance witness does not name a registration participant"
        ))?;

    if witness.source_observation_digest != source_observation_binding_digest(participant) {
        return Err(fpm_anchor_error(
            "provenance witness source-observation binding does not match registration anchor",
        ));
    }
    if !valid_fpm_digest(&witness.acquisition_root_digest) {
        return Err(fpm_anchor_error(
            "provenance witness root commitment is not canonical SHA-256",
        ));
    }
    if !valid_provenance_identifier(&witness.node_id)
        || witness.parent_node_ids.iter().any(|parent| !valid_provenance_identifier(parent))
    {
        return Err(fpm_anchor_error("provenance witness contains an invalid node identifier"));
    }
    if witness.parent_node_ids.len() > 32 {
        return Err(fpm_anchor_error(
            "provenance witness has too many parents",
        ));
    }
    Ok(())
}

fn valid_acquisition_root_label(value: &str, max_len: usize) -> bool {
    !value.is_empty()
        && value == value.trim()
        && value.len() <= max_len
        && !value.chars().any(char::is_control)
}

fn fpm_root_anchor_error(reason: impl Into<String>) -> WasmError {
    FabricationError::ValidationFailed {
        field: "fpm_acquisition_root_anchor".into(),
        reason: reason.into(),
    }
    .to_wasm_error()
}

fn resolve_acquisition_root_action_anchor(
    input: ResolveFpmAcquisitionRootActionAnchorInput,
) -> ExternResult<ResolvedFpmAcquisitionRootAnchor> {
    let details = get_details(input.action_hash.clone(), GetOptions::network())?
        .ok_or_else(|| FabricationError::not_found(
            "FpmAcquisitionRootAnchor",
            &input.action_hash,
        ))?;

    let Details::Record(record_details) = details else {
        return Err(fpm_root_anchor_error(
            "acquisition-root ActionHash did not resolve to record details",
        ));
    };

    if record_details.validation_status != ValidationStatus::Valid {
        return Err(fpm_root_anchor_error("acquisition-root record is not valid"));
    }
    if !record_details.updates.is_empty() {
        return Err(fpm_root_anchor_error("acquisition-root record has updates"));
    }
    if !record_details.deletes.is_empty() {
        return Err(fpm_root_anchor_error("acquisition-root record has deletes"));
    }

    let record = record_details.record;
    if record.action().action_type() != ActionType::Create {
        return Err(fpm_root_anchor_error(
            "acquisition-root ActionHash must resolve to the original Create action",
        ));
    }

    let expected_entry_type = EntryType::App(
        UnitEntryTypes::FpmAcquisitionRootAnchor
            .try_into()
            .map_err(|_| {
                fpm_root_anchor_error(
                    "could not construct FPM acquisition-root entry type",
                )
            })?,
    );
    if record.action().entry_type() != Some(&expected_entry_type) {
        return Err(fpm_root_anchor_error(
            "ActionHash does not reference the FPM acquisition-root entry type",
        ));
    }

    let anchor: FpmAcquisitionRootAnchor = record
        .entry()
        .to_app_option()
        .map_err(|e| {
            fpm_root_anchor_error(format!(
                "could not decode FPM acquisition-root anchor: {e}"
            ))
        })?
        .ok_or_else(|| {
            fpm_root_anchor_error(
                "record is not an FPM acquisition-root anchor entry",
            )
        })?;

    if anchor.schema_version != FPM_ACQUISITION_ROOT_ANCHOR_SCHEMA_VERSION {
        return Err(fpm_root_anchor_error(
            "unsupported FPM acquisition-root anchor schema",
        ));
    }
    if !valid_acquisition_root_label(&anchor.source_system_id, 128)
        || !valid_acquisition_root_label(&anchor.capture_reference, 256)
        || !valid_fpm_digest(&anchor.artifact_digest)
        || !valid_fpm_digest(&anchor.root_digest)
    {
        return Err(fpm_root_anchor_error(
            "acquisition-root declaration is malformed",
        ));
    }
    if acquisition_root_binding_digest(
        &anchor.source_system_id,
        &anchor.capture_reference,
        &anchor.artifact_digest,
    ) != anchor.root_digest
    {
        return Err(fpm_root_anchor_error(
            "acquisition-root commitment does not match declaration",
        ));
    }
    if let Some(expected) = input.expected_root_digest.as_ref() {
        if expected != &anchor.root_digest {
            return Err(fpm_root_anchor_error(
                "acquisition-root digest does not match expected witness root",
            ));
        }
    }

    let entry_hash = record
        .action()
        .entry_hash()
        .ok_or_else(|| fpm_root_anchor_error(
            "acquisition-root action has no entry hash",
        ))?
        .clone();

    Ok(ResolvedFpmAcquisitionRootAnchor {
        action_hash: input.action_hash,
        entry_hash,
        root_digest: anchor.root_digest,
        source_system_id: anchor.source_system_id,
        capture_reference: anchor.capture_reference,
        artifact_digest: anchor.artifact_digest,
        author: *record.action().author(),
        signer: *record.action().signer(),
        timestamp: record.action().timestamp(),
        action_seq: record.action().action_seq(),
        prev_action: record.action().prev_action().cloned(),
    })
}

fn valid_provenance_identifier(value: &str) -> bool {
    !value.is_empty()
        && value == value.trim()
        && value.len() <= 128
        && !value.chars().any(char::is_control)
}

fn resolve_registration_anchor_for_provenance(    action_hash: &ActionHash,
) -> ExternResult<ResolvedFpmRegistrationAnchor> {
    let details = get_details(action_hash.clone(), GetOptions::network())?
        .ok_or_else(|| FabricationError::not_found("FpmRegistrationAnchor", action_hash))?;

    let Details::Record(record_details) = details else {
        return Err(fpm_anchor_error(
            "registration anchor did not resolve to record details",
        ));
    };

    let record = record_details.record;
    let anchor: FpmRegistrationAnchor = record
        .entry()
        .to_app_option()
        .map_err(|e| {
            fpm_anchor_error(format!(
                "could not decode registration anchor for provenance: {e}"
            ))
        })?
        .ok_or_else(|| {
            fpm_anchor_error("registration action is not an FPM registration anchor")
        })?;

    resolve_action_anchor(ResolveFpmRegistrationActionAnchorInput {
        action_hash: action_hash.clone(),
        claimed_envelope_digest: anchor.envelope_digest,
        expected_author: None,
        expected_signer: None,
    })
}

fn resolve_provenance_action_anchor(
    input: ResolveFpmProvenanceActionAnchorInput,
) -> ExternResult<ResolvedFpmProvenanceAnchor> {
    let details = get_details(input.provenance_action_hash.clone(), GetOptions::network())?
        .ok_or_else(|| FabricationError::not_found(
            "FpmProvenanceAnchor",
            &input.provenance_action_hash,
        ))?;

    let Details::Record(record_details) = details else {
        return Err(fpm_anchor_error(
            "provenance ActionHash did not resolve to record details",
        ));
    };

    if record_details.validation_status != ValidationStatus::Valid {
        return Err(fpm_anchor_error("provenance anchor record is not valid"));
    }
    if !record_details.updates.is_empty() {
        return Err(fpm_anchor_error("provenance anchor record has updates"));
    }
    if !record_details.deletes.is_empty() {
        return Err(fpm_anchor_error("provenance anchor record has deletes"));
    }

    let record = record_details.record;
    if record.action().action_type() != ActionType::Create {
        return Err(fpm_anchor_error(
            "provenance ActionHash must resolve to the original Create action",
        ));
    }

    let expected_entry_type = EntryType::App(
        UnitEntryTypes::FpmProvenanceAnchor
            .try_into()
            .map_err(|_| {
                fpm_anchor_error("could not construct FPM provenance anchor entry type")
            })?,
    );
    if record.action().entry_type() != Some(&expected_entry_type) {
        return Err(fpm_anchor_error(
            "ActionHash does not reference the FPM provenance anchor entry type",
        ));
    }

    let anchor: FpmProvenanceAnchor = record
        .entry()
        .to_app_option()
        .map_err(|e| {
            fpm_anchor_error(format!("could not decode FPM provenance anchor: {e}"))
        })?
        .ok_or_else(|| fpm_anchor_error("record is not an FPM provenance anchor entry"))?;

    if anchor.schema_version != FPM_PROVENANCE_ANCHOR_SCHEMA_VERSION {
        return Err(fpm_anchor_error("unsupported FPM provenance anchor schema"));
    }
    if !valid_fpm_digest(&anchor.witness_digest)
        || anchor.witness_digest != anchor.witness.digest()
    {
        return Err(fpm_anchor_error(
            "FPM provenance witness digest is invalid or mismatched",
        ));
    }

    let registration =
        resolve_registration_anchor_for_provenance(&anchor.registration_anchor_action)?;

    if let Some(expected) = input.expected_registration_anchor_action.as_ref() {
        if expected != &anchor.registration_anchor_action {
            return Err(fpm_anchor_error(
                "provenance anchor references an unexpected registration anchor",
            ));
        }
    }

    validate_provenance_witness_against_envelope(
        &anchor.witness,
        &registration.envelope,
    )?;

    let provenance_entry_hash = record
        .action()
        .entry_hash()
        .ok_or_else(|| fpm_anchor_error("provenance anchor action has no entry hash"))?;
    let provenance_entry_hash = provenance_entry_hash.clone();

    Ok(ResolvedFpmProvenanceAnchor {
        provenance_action_hash: input.provenance_action_hash,
        provenance_entry_hash,
        registration_anchor_action: anchor.registration_anchor_action,
        witness: anchor.witness,
        witness_digest: anchor.witness_digest,
        author: *record.action().author(),
        signer: *record.action().signer(),
        timestamp: record.action().timestamp(),
        action_seq: record.action().action_seq(),
        prev_action: record.action().prev_action().cloned(),
    })
}

fn authenticated_acquisition_root_manifest_digest(
    roots: &[ResolvedFpmAcquisitionRootAnchor],
) -> String {
    let mut entries = roots
        .iter()
        .map(|root| {
            (
                root.action_hash.to_string(),
                root.root_digest.clone(),
                root.artifact_digest.clone(),
            )
        })
        .collect::<Vec<_>>();
    entries.sort_unstable();

    let mut bytes = Vec::new();
    append_length_prefixed(
        &mut bytes,
        b"fpm.authenticated-acquisition-root-manifest.v1",
    );
    for (action_hash, root_digest, artifact_digest) in entries {
        append_length_prefixed(&mut bytes, action_hash.as_bytes());
        append_length_prefixed(&mut bytes, root_digest.as_bytes());
        append_length_prefixed(&mut bytes, artifact_digest.as_bytes());
    }
    hex_digest_bytes(&bytes)
}

fn authenticated_provenance_manifest_digest(
    witnesses: &[ResolvedFpmProvenanceAnchor],
) -> String {
    let mut entries = witnesses
        .iter()
        .map(|item| {
            (
                item.registration_anchor_action.to_string(),
                item.provenance_action_hash.to_string(),
                item.witness_digest.clone(),
            )
        })
        .collect::<Vec<_>>();
    entries.sort_unstable();

    let mut bytes = Vec::new();
    append_length_prefixed(&mut bytes, b"fpm.authenticated-provenance-manifest.v1");
    for (registration_hash, action_hash, witness_digest) in entries {
        append_length_prefixed(&mut bytes, registration_hash.as_bytes());
        append_length_prefixed(&mut bytes, action_hash.as_bytes());
        append_length_prefixed(&mut bytes, witness_digest.as_bytes());
    }
    hex_digest_bytes(&bytes)
}

fn append_length_prefixed(buffer: &mut Vec<u8>, field: &[u8]) {
    buffer.extend_from_slice(&(field.len() as u64).to_be_bytes());
    buffer.extend_from_slice(field);
}

fn hex_digest_bytes(bytes: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    hasher.finalize()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

#[hdk_extern]
pub fn create_fpm_acquisition_root_anchor(
    input: CreateFpmAcquisitionRootAnchorInput,
) -> ExternResult<Record> {
    rate_limit_caller()?;

    if !valid_acquisition_root_label(&input.source_system_id, 128)
        || !valid_acquisition_root_label(&input.capture_reference, 256)
        || !valid_fpm_digest(&input.artifact_digest)
    {
        return Err(fpm_root_anchor_error(
            "invalid acquisition-root declaration",
        ));
    }

    let root_digest = acquisition_root_binding_digest(
        &input.source_system_id,
        &input.capture_reference,
        &input.artifact_digest,
    );
    let anchor = FpmAcquisitionRootAnchor {
        schema_version: FPM_ACQUISITION_ROOT_ANCHOR_SCHEMA_VERSION.into(),
        source_system_id: input.source_system_id,
        capture_reference: input.capture_reference,
        artifact_digest: input.artifact_digest,
        root_digest,
    };

    let action_hash = create_entry(EntryTypes::FpmAcquisitionRootAnchor(anchor))?;
    get(action_hash, GetOptions::default())?.ok_or_else(|| {
        FabricationError::not_found(
            "FpmAcquisitionRootAnchor",
            &"newly-created action",
        )
    })
}

#[hdk_extern]
pub fn resolve_fpm_acquisition_root_action_anchor(
    input: ResolveFpmAcquisitionRootActionAnchorInput,
) -> ExternResult<ResolvedFpmAcquisitionRootAnchor> {
    rate_limit_caller()?;
    resolve_acquisition_root_action_anchor(input)
}

#[hdk_extern]
pub fn create_fpm_provenance_anchor(
    input: CreateFpmProvenanceAnchorInput,
) -> ExternResult<Record> {
    rate_limit_caller()?;

    let registration =
        resolve_registration_anchor_for_provenance(&input.registration_anchor_action)?;
    registration
        .envelope
        .validate_consistency()
        .map_err(|e| fpm_anchor_error(format!(
            "provenance anchor requires a consistent registration envelope: {e}"
        )))?;

    validate_provenance_witness_against_envelope(
        &input.witness,
        &registration.envelope,
    )?;

    let anchor = FpmProvenanceAnchor {
        schema_version: FPM_PROVENANCE_ANCHOR_SCHEMA_VERSION.into(),
        registration_anchor_action: input.registration_anchor_action,
        witness: input.witness.clone(),
        witness_digest: input.witness.digest(),
    };

    let action_hash = create_entry(EntryTypes::FpmProvenanceAnchor(anchor))?;

    get(action_hash, GetOptions::default())?.ok_or_else(|| {
        FabricationError::not_found("FpmProvenanceAnchor", &"newly-created action")
    })
}

#[hdk_extern]
pub fn resolve_fpm_provenance_action_anchor(
    input: ResolveFpmProvenanceActionAnchorInput,
) -> ExternResult<ResolvedFpmProvenanceAnchor> {
    rate_limit_caller()?;
    resolve_provenance_action_anchor(input)
}

#[hdk_extern]
pub fn qualify_authenticated_fpm_provenance(
    input: QualifyAuthenticatedFpmProvenanceInput,
) -> ExternResult<AuthenticatedFpmProvenanceQualification> {
    rate_limit_caller()?;

    const MAX_PROVENANCE_ANCHORS: usize = 256;
    if input.provenance_action_hashes.is_empty()
        || input.provenance_action_hashes.len() > MAX_PROVENANCE_ANCHORS
    {
        return Err(fpm_anchor_error(
            "authenticated provenance anchor count is outside supported bounds",
        ));
    }

    let registration =
        resolve_registration_anchor_for_provenance(&input.registration_anchor_action)?;

    let mut action_hashes = input.provenance_action_hashes.clone();
    action_hashes.sort_by_key(|hash| hash.to_string());
    if action_hashes.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(fpm_anchor_error(
            "duplicate authenticated provenance ActionHash",
        ));
    }

    let mut resolved = Vec::with_capacity(action_hashes.len());
    for action_hash in &action_hashes {
        resolved.push(resolve_provenance_action_anchor(
            ResolveFpmProvenanceActionAnchorInput {
                provenance_action_hash: action_hash.clone(),
                expected_registration_anchor_action: Some(
                    input.registration_anchor_action.clone(),
                ),
            },
        )?);
    }

    let mut root_action_hashes = input.acquisition_root_action_hashes.clone();
    root_action_hashes.sort_by_key(|hash| hash.to_string());
    if root_action_hashes.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(fpm_root_anchor_error(
            "duplicate authenticated acquisition-root ActionHash",
        ));
    }
    if root_action_hashes.is_empty() || root_action_hashes.len() > MAX_PROVENANCE_ANCHORS {
        return Err(fpm_root_anchor_error(
            "authenticated acquisition-root anchor count is outside supported bounds",
        ));
    }

    let mut roots = Vec::with_capacity(root_action_hashes.len());
    for action_hash in &root_action_hashes {
        roots.push(resolve_acquisition_root_action_anchor(
            ResolveFpmAcquisitionRootActionAnchorInput {
                action_hash: action_hash.clone(),
                expected_root_digest: None,
            },
        )?);
    }

    let witness_root_digests = resolved
        .iter()
        .map(|item| item.witness.acquisition_root_digest.clone())
        .collect::<std::collections::BTreeSet<_>>();
    let mut resolved_root_digest_list = roots
        .iter()
        .map(|root| root.root_digest.clone())
        .collect::<Vec<_>>();
    resolved_root_digest_list.sort_unstable();
    if resolved_root_digest_list.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(fpm_root_anchor_error(
            "duplicate authenticated acquisition-root commitment",
        ));
    }
    let resolved_root_digests = resolved_root_digest_list
        .into_iter()
        .collect::<std::collections::BTreeSet<_>>();

    if witness_root_digests != resolved_root_digests {
        return Err(fpm_root_anchor_error(
            "authenticated acquisition-root coverage does not exactly match witness roots",
        ));
    }

    let lineage = resolved
        .iter()
        .map(|item| item.witness.clone())
        .collect::<Vec<_>>();

    let structural_qualification = qualify_provenance(&ProvenanceQualificationInput {
        registration_envelope_digest: registration.registration_envelope_digest.clone(),
        envelope: registration.envelope.clone(),
        lineage,
    });

    let provenance_anchor_manifest_digest =
        authenticated_provenance_manifest_digest(&resolved);
    let acquisition_root_anchor_manifest_digest =
        authenticated_acquisition_root_manifest_digest(&roots);

    Ok(AuthenticatedFpmProvenanceQualification {
        schema_version: FPM_AUTHENTICATED_PROVENANCE_SCHEMA_VERSION.into(),
        registration_anchor_action: input.registration_anchor_action,
        registration_envelope_digest: registration.registration_envelope_digest,
        provenance_anchor_manifest_digest,
        acquisition_root_anchor_manifest_digest,
        structural_qualification,
        witnesses: resolved,
        roots,
    })
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct QualifyAuthenticatedFpmProvenanceInput {
    pub registration_anchor_action: ActionHash,
    pub provenance_action_hashes: Vec<ActionHash>,
    pub acquisition_root_action_hashes: Vec<ActionHash>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct AuthenticatedFpmProvenanceQualification {
    pub schema_version: String,
    pub registration_anchor_action: ActionHash,
    pub registration_envelope_digest: String,
    pub provenance_anchor_manifest_digest: String,
    pub acquisition_root_anchor_manifest_digest: String,
    pub structural_qualification: ProvenanceQualification,
    pub witnesses: Vec<ResolvedFpmProvenanceAnchor>,
    pub roots: Vec<ResolvedFpmAcquisitionRootAnchor>,
}

pub const FPM_AUTHENTICATED_PROVENANCE_SCHEMA_VERSION: &str =
    "fpm.registration.authenticated-provenance.v1";

impl AuthenticatedFpmProvenanceQualification {
    pub fn digest(&self) -> String {
        let bytes = serde_json::to_vec(self)
            .expect("authenticated FPM provenance qualification is serializable");
        let mut preimage = Vec::new();
        append_length_prefixed(
            &mut preimage,
            b"fpm.authenticated-provenance-qualification.v1",
        );
        append_length_prefixed(&mut preimage, &bytes);
        hex_digest_bytes(&preimage)
    }
}

// =============================================================================
// RATE LIMITING
// =============================================================================

fn rate_limit_anchor(agent: &AgentPubKey) -> ExternResult<EntryHash> {
    let anchor_bytes = SerializedBytes::from(UnsafeBytes::from(
        format!("rate_limit:{}", agent).into_bytes(),
    ));
    hash_entry(Entry::App(AppEntryBytes(anchor_bytes)))
}

fn enforce_rate_limit(caller: &AgentPubKey) -> ExternResult<()> {
    let cfg = get_config();
    let max_ops = cfg.rate_limit_max_ops as usize;
    let window_micros = cfg.rate_limit_window_secs as i64 * 1_000_000;

    let anchor = rate_limit_anchor(caller)?;
    let links = get_links(
        LinkQuery::try_new(anchor.clone(), LinkTypes::RateLimitBucket)?,
        GetStrategy::default(),
    )?;

    let now = sys_time()?;
    let window_start = now.as_micros() - window_micros;

    let recent_count = links
        .iter()
        .filter(|l| l.timestamp.as_micros() >= window_start)
        .count();

    if recent_count >= max_ops {
        return Err(FabricationError::RateLimited {
            max_ops: cfg.rate_limit_max_ops,
            window_secs: cfg.rate_limit_window_secs,
        }.to_wasm_error());
    }

    create_link(anchor.clone(), anchor, LinkTypes::RateLimitBucket, ())?;
    Ok(())
}

fn rate_limit_caller() -> ExternResult<()> {
    let agent = agent_info()?.agent_initial_pubkey;
    enforce_rfn epistemic_cache_key(claim_text: &str, claim_type_key: &str) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hasher.update(EPISTEMIC_CACHE_DOMAIN);
    hasher.update((claim_type_key.len() as u64).to_le_bytes());
    hasher.update(claim_type_key.as_bytes());
    hasher.update((claim_text.len() as u64).to_le_bytes());
    hasher.update(claim_text.as_bytes());
    hasher.finalize().into()
}

/// Validate an epistemic classification returned by Knowledge.
///
/// Only finite scores in the closed interval [0, 1] are accepted. This is a
/// semantic boundary in addition to wire decoding: valid serialization alone
/// does not make an out-of-range classification trustworthy.
fn validate_epistemic_response(ep: &ClaimEpistemic) -> ExternResult<()> {
    for (field, value) in [
        ("empirical", ep.empirical),
        ("normative", ep.normative),
        ("mythic", ep.mythic),
    ] {
        if !value.is_finite() || !(0.0..=1.0).contains(&value) {            return Err(FabricationError::ValidationFailed {
                field: format!("knowledge.{}", field),
                reason: "classification score must be finite and in [0, 1]".to_string(),
            }
            .to_wasm_error());
        }
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EpistemicFetchFailure {
    Unavailable,
    Malformed,
}

/// Fetch epistemic classification from Knowledge with an exact-claim cache.
///
/// Missing or malformed classification becomes an explicit unclassified state rather
/// than a fabricated score. Only validated classifications are cached.
fn fetch_epistemic(
    claim_text: &str,
    claim_type_key: &str,
) -> Result<ClaimEpistemic, EpistemicFetchFailure> {
    let now = match sys_time() {
        Ok(time) => time.as_micros(),
        Err(_) => return Err(EpistemicFetchFailure::Unavailable),
    };
    let key = epistemic_cache_key(claim_text, claim_type_key);

    let cached = EPISTEMIC_CACHE.with(|c| {
        c.borrow().get(&key).and_then(|(ts, ep)| {
            now.checked_sub(*ts)
                .filter(|age| *age >= 0 && *age < EPISTEMIC_CACHE_TTL_MICROS)
                .map(|_| ep.clone())
        })
    });
    if let Some(ep) = cached {
        return Ok(ep);
    }

    let response = match call(
        CallTargetCell::OtherRole("mycelix-knowledge".into()),
        ZomeName::from("epistemic"),
        FunctionName::from("classify_claim"),
        None,
        claim_text,
    ) {
        Ok(response) => response,
        Err(_) => return Err(EpistemicFetchFailure::Unavailable),
    };

    let ep = match response {
        ZomeCallResponse::Ok(bytes) => bytes
            .decode::<ClaimEpistemic>()
            .map_err(|_| EpistemicFetchFailure::Malformed)?,
        _ => return Err(EpistemicFetchFailure::Unavailable),
    };

    validate_epistemic_response(&ep)
        .map_err(|_| EpistemicFetchFailure::Malformed)?;

    EPISTEMIC_CACHE.with(|c| {
        let mut cache = c.borrow_mut();

        cache.retain(|_, (ts, _)| {
            now.checked_sub(*ts)
                .filter(|age| *age >= 0 && *age < EPISTEMIC_CACHE_TTL_MICROS)
                .is_some()
        });

        if !cache.contains_key(&key) && cache.len() >= EPISTEMIC_CACHE_MAX_ENTRIES {
            if let Some(oldest_key) = cache
                .iter()
                .min_by_key(|(_, (ts, _))| *ts)
                .map(|(key, _)| *key)
            {
                cache.remove(&oldest_key);
            }
        }

        cache.insert(key, (now, ep.clone()));
    });

    Ok(ep)
}

#[derive(Serialize, Deserialize, Debug)]
pub struct SubmitVerificationInput {
    pub design_hash: ActionHash,
    pub verification_type: VerificationType,
    pub result: VerificationResult,
    pub evidence: Vec<ActionHash>,
    pub credentials: Vec<String>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct SubmitClaimInput {
    pub design_hash: ActionHash,
    pub claim_type: SafetyClaimType,
    pub claim_text: String,
    pub supporting_evidence: Vec<String>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct VerificationSummary {
    pub design_hash: ActionHash,
    pub total_verifications: u32,
    pub passed: u32,
    pub failed: u32,
    pub claims_count: u32,
    pub average_confidence: f32,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct EpistemicScore {
    pub empirical: f32,
    pub normative: f32,
    pub mythic: f32,
    pub overall_confidence: f32,
    /// Explicitly distinguishes complete, partial, absent, and incomplete evidence.
    pub evidence_status: EpistemicAggregateStatus,
    /// Number of safety claims with validated Knowledge classifications.
    pub classified_claims: u32,
    /// Number of records linked from the design and considered by the aggregate.
    pub total_claims: u32,
    /// Number of linked records that could not be interpreted as SafetyClaim records.
    pub uninterpretable_records: u32,
}

#[hdk_extern]
pub fn submit_verification(input: SubmitVerificationInput) -> ExternResult<Record> {
    rate_limit_caller()?;
    let verifier = agent_info()?.agent_initial_pubkey;
    let now = sys_time()?;

    let verification = DesignVerification {
        design_hash: input.design_hash.clone(),
        verification_type: input.verification_type,
        result: input.result,
        evidence: input.evidence,
        verifier: verifier.clone(),
        verifier_credentials: input.credentials,
        created_at: Timestamp::from_micros(now.as_micros() as i64),
    };

    let hash = create_entry(EntryTypes::DesignVerification(verification))?;

    let _ = emit_signal(&TypedFabricationSignal {
        domain: FabricationDomain::Verification,
        event_type: FabricationEventType::VerificationSubmitted,
        payload: format!(r#"{{"hash":"{}"}}"#, hash),
    });

    create_link(input.design_hash, hash.clone(), LinkTypes::DesignToVerifications, ())?;
    create_link(verifier, hash.clone(), LinkTypes::VerifierToVerifications, ())?;

    get(hash.clone(), GetOptions::default())?.ok_or(FabricationError::not_found("Verification", &hash))
}

/// Internal: get all verifications for a design (used by summary/score functions)
fn get_design_verifications_all(design_hash: ActionHash) -> ExternResult<Vec<Record>> {
    let links = get_links(LinkQuery::try_new(design_hash, LinkTypes::DesignToVerifications)?, GetStrategy::default())?;
    let mut results = Vec::new();
    for link in links {
        if let Some(hash) = link.target.into_action_hash() {
            if let Some(record) = get(hash, GetOptions::default())? {
                results.push(record);
            }
        }
    }
    Ok(results)
}

#[hdk_extern]
pub fn get_design_verifications(input: HashPaginationInput) -> ExternResult<PaginatedResponse<Record>> {
    let items = get_design_verifications_all(input.hash)?;
    Ok(paginate(items, input.pagination.as_ref()))
}

#[hdk_extern]
pub fn get_verification_summary(design_hash: ActionHash) -> ExternResult<VerificationSummary> {
    let verifications = get_design_verifications_all(design_hash.clone())?;
    let claims = get_design_claims_all(design_hash.clone())?;

    let mut passed = 0u32;
    let mut failed = 0u32;
    let mut confidence_sum = 0.0f32;

    for record in &verifications {
        if let Some(v) = record.entry().to_app_option::<DesignVerification>().ok().flatten() {
            match v.result {
                VerificationResult::Passed { confidence, .. } => {
                    passed += 1;
                    confidence_sum += confidence;
                }
                VerificationResult::Failed { .. } => failed += 1,
                VerificationResult::ConditionalPass { confidence, .. } => {
                    passed += 1;
                    confidence_sum += confidence * 0.8;
                }
                _ => {}
            }
        }
    }

    let total = verifications.len() as u32;
    let avg_confidence = if passed > 0 { confidence_sum / passed as f32 } else { 0.0 };

    Ok(VerificationSummary {
        design_hash,
        total_verifications: total,
        passed,
        failed,
        claims_count: claims.len() as u32,
        average_confidence: avg_confidence,
    })
}

#[hdk_extern]
pub fn submit_safety_claim(input: SubmitClaimInput) -> ExternResult<Record> {
    rate_limit_caller()?;
    let author = agent_info()?.agent_initial_pubkey;
    let now = sys_time()?;

    // Preserve the safety claim itself, but never promote missing or malformed
    // Knowledge enrichment into a positive epistemic score.
    let claim_type_key = format!("{:?}", input.claim_type);
    let (epistemic, epistemic_provenance) =
        match fetch_epistemic(&input.claim_text, &claim_type_key) {
            Ok(ep) => (Some(ep), EpistemicProvenance::KnowledgeClassified),
            Err(EpistemicFetchFailure::Unavailable) => {
                (None, EpistemicProvenance::KnowledgeUnavailable)
            }
            Err(EpistemicFetchFailure::Malformed) => {
                (None, EpistemicProvenance::KnowledgeMalformed)
            }
        };

    let claim = SafetyClaim {
        design_hash: input.design_hash.clone(),
        claim_type: input.claim_type,
        claim_text: input.claim_text,
        epistemic,
        epistemic_provenance,
        supporting_evidence: input.supporting_evidence,
        knowledge_claim_hash: None,
        author,
        created_at: Timestamp::from_micros(now.as_micros() as i64),
    };

    let hash = create_entry(EntryTypes::SafetyClaim(claim))?;

    let _ = emit_signal(&TypedFabricationSignal {
        domain: FabricationDomain::Verification,
        event_type: FabricationEventType::ClaimSubmitted,
        payload: format!(r#"{{"hash":"{}"}}"#, hash),
    });

    create_link(input.design_hash, hash.clone(), LinkTypes::DesignToClaims, ())?;

    get(hash.clone(), GetOptions::default())?.ok_or(FabricationError::not_found("SafetyClaim", &hash))
}

/// Internal: get all claims for a design (used by summary/score functions)
fn get_design_claims_all(design_hash: ActionHash) -> ExternResult<Vec<Record>> {
    let links = get_links(LinkQuery::try_new(design_hash, LinkTypes::DesignToClaims)?, GetStrategy::default())?;
    let mut results = Vec::new();
    for link in links {
        if let Some(hash) = link.target.into_action_hash() {
            if let Some(record) = get(hash, GetOptions::default())? {
                results.push(record);
            }
        }
    }
    Ok(results)
}

#[hdk_extern]
pub fn get_design_claims(input: HashPaginationInput) -> ExternResult<PaginatedResponse<Record>> {
    let items = get_design_claims_all(input.hash)?;
    Ok(paginate(items, input.pagination.as_ref()))
}

fn epistemic_aggregate_status(
    classified_claims: u32,
    total_claims: u32,
    uninterpretable_records: u32,
) -> EpistemicAggregateStatus {
    if uninterpretable_records > 0 {
        return EpistemicAggregateStatus::IncompleteEvidence;
    }

    match (classified_claims, total_claims) {
        (0, _) => EpistemicAggregateStatus::NoClassifiedEvidence,
        (classified, total) if classified == total => EpistemicAggregateStatus::Classified,
        _ => EpistemicAggregateStatus::PartialClassifiedEvidence,
    }
}

#[hdk_extern]
pub fn get_epistemic_score(design_hash: ActionHash) -> ExternResult<EpistemicScore> {
    let claims = get_design_claims_all(design_hash)?;
    let mut e_sum = 0.0f32;
    let mut n_sum = 0.0f32;
    let mut m_sum = 0.0f32;
    let mut classified_count = 0u32;
    let total_claims = claims.len() as u32;
    let mut uninterpretable_records = 0u32;

    for record in claims {
        let claim = match record.entry().to_app_option::<SafetyClaim>() {
            Ok(Some(claim)) => claim,
            Ok(None) | Err(_) => {
                uninterpretable_records += 1;
                continue;
            }
        };

        if claim.epistemic_provenance != EpistemicProvenance::KnowledgeClassified {
            continue;
        }
        let Some(epistemic) = claim.epistemic else {
            continue;
        };
        e_sum += epistemic.empirical;
        n_sum += epistemic.normative;
        m_sum += epistemic.mythic;
        classified_count += 1;
    }

    let count_f = classified_count.max(1) as f32;
    let evidence_status =
        epistemic_aggregate_status(classified_count, total_claims, uninterpretable_records);

    Ok(EpistemicScore {
        empirical: e_sum / count_f,
        normative: n_sum / count_f,
        mythic: m_sum / count_f,
        overall_confidence: (e_sum + n_sum) / (2.0 * count_f),
        evidence_status,
        classified_claims: classified_count,
        total_claims,
        uninterpretable_records,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helpers ──────────────────────────────────────────────────────────────

    fn test_action_hash() -> ActionHash {
        ActionHash::from_raw_36(vec![0u8; 36])
    }

    // ── 1. SubmitVerificationInput serde roundtrip ────────────────────────────

    #[test]
    fn test_submit_verification_input_serde() {
        let input = SubmitVerificationInput {
            design_hash: test_action_hash(),
            verification_type: VerificationType::StructuralAnalysis,
            result: VerificationResult::Passed {
                confidence: 0.95,
                notes: "All finite-element checks passed".to_string(),
            },
            evidence: vec![ActionHash::from_raw_36(vec![1u8; 36])],
            credentials: vec!["PE License #12345".to_string()],
        };

        let json = serde_json::to_string(&input)
            .expect("SubmitVerificationInput should serialize to JSON");
        let restored: SubmitVerificationInput = serde_json::from_str(&json)
            .expect("SubmitVerificationInput should deserialize from JSON");

        // Verify structural fields survive the roundtrip.
        assert_eq!(restored.design_hash, input.design_hash);
        assert_eq!(restored.verification_type, input.verification_type);
        assert_eq!(restored.credentials, input.credentials);
        assert_eq!(restored.evidence.len(), 1);

        // Verify the result variant and its inner values.
        match restored.result {
            VerificationResult::Passed { confidence, ref notes } => {
                assert!((confidence - 0.95).abs() < f32::EPSILON);
                assert_eq!(notes, "All finite-element checks passed");
            }
            other => panic!("Expected Passed variant, got {:?}", other),
        }
    }

    // ── 2. SubmitClaimInput serde roundtrip ───────────────────────────────────

    #[test]
    fn test_submit_safety_claim_input_serde() {
        let input = SubmitClaimInput {
            design_hash: test_action_hash(),
            claim_type: SafetyClaimType::LoadCapacity("Supports 50kg static load".to_string()),
            claim_text: "Bracket rated for 50 kg at SWL 3:1 safety factor".to_string(),
            supporting_evidence: vec!["FEA report v2.1".to_string(), "Physical test #7".to_string()],
        };

        let json = serde_json::to_string(&input)
            .expect("SubmitClaimInput should serialize to JSON");
        let restored: SubmitClaimInput = serde_json::from_str(&json)
            .expect("SubmitClaimInput should deserialize from JSON");

        assert_eq!(restored.design_hash, input.design_hash);
        assert_eq!(restored.claim_text, input.claim_text);
        assert_eq!(restored.supporting_evidence, input.supporting_evidence);
        assert_eq!(
            restored.claim_type,
            SafetyClaimType::LoadCapacity("Supports 50kg static load".to_string())
        );
    }

    // ── 3. VerificationType — all variants roundtrip ──────────────────────────

    #[test]
    fn test_verification_type_all_variants_serde() {
        let variants = vec![
            VerificationType::StructuralAnalysis,
            VerificationType::MaterialCompatibility,
            VerificationType::PrintabilityTest,
            VerificationType::SafetyReview,
            VerificationType::FoodSafeCertification,
            VerificationType::MedicalCertification,
            VerificationType::CommunityReview,
        ];

        for variant in &variants {
            let json = serde_json::to_string(variant)
                .unwrap_or_else(|e| panic!("Failed to serialize {:?}: {}", variant, e));
            let restored: VerificationType = serde_json::from_str(&json)
                .unwrap_or_else(|e| panic!("Failed to deserialize {:?}: {}", json, e));
            assert_eq!(
                &restored, variant,
                "VerificationType::{:?} did not survive serde roundtrip",
                variant
            );
        }
    }

    // ── 4. SafetyClaimType — all variants roundtrip ───────────────────────────

    #[test]
    fn test_safety_claim_type_variants_serde() {
        let variants = vec![
            SafetyClaimType::LoadCapacity("Supports 80kg".to_string()),
            SafetyClaimType::MaterialSafety("Food-safe in PETG".to_string()),
            SafetyClaimType::DimensionalAccuracy("Fits M8 bolt".to_string()),
            SafetyClaimType::TemperatureRange("Safe to 80°C".to_string()),
            SafetyClaimType::ChemicalResistance("Resistant to IPA".to_string()),
            SafetyClaimType::Custom("Outdoor UV rating 10yr".to_string()),
        ];

        for variant in &variants {
            let json = serde_json::to_string(variant)
                .unwrap_or_else(|e| panic!("Failed to serialize {:?}: {}", variant, e));
            let restored: SafetyClaimType = serde_json::from_str(&json)
                .unwrap_or_else(|e| panic!("Failed to deserialize {:?}: {}", json, e));
            assert_eq!(
                &restored, variant,
                "SafetyClaimType::{:?} did not survive serde roundtrip",
                variant
            );
        }
    }

    // ── 5. Knowledge response validation ─────────────────────────────────────

    #[test]
    fn test_knowledge_epistemic_response_validation_accepts_bounds() {
        for ep in [
            ClaimEpistemic { empirical: 0.0, normative: 0.5, mythic: 1.0 },
            ClaimEpistemic { empirical: 0.5, normative: 0.5, mythic: 0.5 },
        ] {
            assert!(validate_epistemic_response(&ep).is_ok());
        }
    }

    #[test]
    fn test_knowledge_epistemic_response_validation_rejects_nonfinite_or_out_of_range() {
        for ep in [
            ClaimEpistemic { empirical: f32::NAN, normative: 0.5, mythic: 0.5 },
            ClaimEpistemic { empirical: f32::INFINITY, normative: 0.5, mythic: 0.5 },
            ClaimEpistemic { empirical: 0.5, normative: -0.01, mythic: 0.5 },
            ClaimEpistemic { empirical: 0.5, normative: 0.5, mythic: 1.01 },
        ] {
            assert!(validate_epistemic_response(&ep).is_err());
        }
    }

    #[test]
    fn test_epistemic_cache_key_includes_claim_text_and_type() {
        let a = epistemic_cache_key("same claim", "LoadCapacity");
        let b = epistemic_cache_key("different claim", "LoadCapacity");        let c = epistemic_cache_key("same claim", "MaterialSafety");

        assert_ne!(a, b, "distinct claim text must not share the cache key");
        assert_ne!(a, c, "distinct claim types must not share the cache key");
        assert_eq!(
            a,
            epistemic_cache_key("same claim", "LoadCapacity"),
            "cache key must be deterministic"
        );
    }

    #[test]
    fn test_epistemic_cache_ttl_constant() {
        assert_eq!(EPISTEMIC_CACHE_TTL_MICROS, 300_000_000);
        assert_eq!(EPISTEMIC_CACHE_TTL_MICROS / 1_000_000, 300);
        assert_eq!(EPISTEMIC_CACHE_MAX_ENTRIES, 128);
    }

    #[test]
    fn test_epistemic_aggregate_status_serde_roundtrip() {
        for status in [
            EpistemicAggregateStatus::Classified,
            EpistemicAggregateStatus::PartialClassifiedEvidence,
            EpistemicAggregateStatus::NoClassifiedEvidence,
            EpistemicAggregateStatus::IncompleteEvidence,
        ] {
            let encoded = serde_json::to_string(&status).unwrap();
            let decoded: EpistemicAggregateStatus = serde_json::from_str(&encoded).unwrap();
            assert_eq!(decoded, status);
        }
    }

    #[test]
    fn test_epistemic_aggregate_status_classification_matrix() {
        assert_eq!(
            epistemic_aggregate_status(0, 0, 0),
            EpistemicAggregateStatus::NoClassifiedEvidence
        );
        assert_eq!(
            epistemic_aggregate_status(0, 3, 0),
            EpistemicAggregateStatus::NoClassifiedEvidence
        );
        assert_eq!(
            epistemic_aggregate_status(1, 1, 0),
            EpistemicAggregateStatus::Classified
        );
        assert_eq!(
            epistemic_aggregate_status(2, 3, 0),
            EpistemicAggregateStatus::PartialClassifiedEvidence
        );
        assert_eq!(
            epistemic_aggregate_status(3, 3, 0),
            EpistemicAggregateStatus::Classified
        );
        assert_eq!(
            epistemic_aggregate_status(1, 1, 1),
            EpistemicAggregateStatus::IncompleteEvidence
        );
        assert_eq!(
            epistemic_aggregate_status(0, 1, 1),
            EpistemicAggregateStatus::IncompleteEvidence
        );
    }


    #[test]
    fn fpm_anchor_envelope_binding_accepts_exact_digest() {
        let reference = ModalityObservationRef {
            source_id: "thermal-1".into(),
            modality: "thermal".into(),
            clock_domain: "ptp-domain-1".into(),
            source_sequence: 10,
            correlation_domain: "frame-domain".into(),
            correlation_id: "frame-10".into(),
            source_timestamp_micros: Some(1_000_000),
            calibration_profile_digest: "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            process_context_digest: "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".into(),
            source_data_digest: "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc".into(),
        };
        let related = ModalityObservationRef {
            source_id: "vibration-1".into(),
            modality: "vibration".into(),
            ..reference.clone()
        };
        let envelope = RegistrationEnvelope {
            schema_version: FPM_REGISTRATION_SCHEMA_VERSION.into(),
            reference,
            related: vec![related],
            alignment_method: Some(AlignmentMethod::ExactCorrelationId),
        };
        let digest = envelope.digest().expect("envelope digest");
        let anchor = FpmRegistrationAnchor {
            schema_version: FPM_REGISTRATION_ANCHOR_SCHEMA_VERSION.into(),
            envelope,
            envelope_digest: digest.clone(),
        };

        assert!(validate_resolved_anchor_envelope(&anchor, &digest).is_ok());
    }

    #[test]
    fn fpm_anchor_envelope_substitution_is_rejected() {
        let mut envelope = RegistrationEnvelope {
            schema_version: FPM_REGISTRATION_SCHEMA_VERSION.into(),
            reference: ModalityObservationRef {
                source_id: "thermal-1".into(),
                modality: "thermal".into(),
                clock_domain: "ptp-domain-1".into(),
                source_sequence: 10,
                correlation_domain: "frame-domain".into(),
                correlation_id: "frame-10".into(),
                source_timestamp_micros: Some(1_000_000),
                calibration_profile_digest: "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
                process_context_digest: "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".into(),
                source_data_digest: "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc".into(),
            },
            related: vec![],
            alignment_method: None,
        };
        let digest = envelope.digest().expect("envelope digest");
        envelope.correlation_id = "replay".into();
        let anchor = FpmRegistrationAnchor {
            schema_version: FPM_REGISTRATION_ANCHOR_SCHEMA_VERSION.into(),
            envelope,
            envelope_digest: digest.clone(),
        };

        assert!(validate_resolved_anchor_envelope(&anchor, &digest).is_err());
    }

    #[test]
    fn fpm_anchor_claimed_digest_mismatch_is_rejected() {
        let envelope = RegistrationEnvelope {
            schema_version: FPM_REGISTRATION_SCHEMA_VERSION.into(),
            reference: ModalityObservationRef {
                source_id: "thermal-1".into(),
                modality: "thermal".into(),
                clock_domain: "ptp-domain-1".into(),
                source_sequence: 10,
                correlation_domain: "frame-domain".into(),
                correlation_id: "frame-10".into(),
                source_timestamp_micros: Some(1_000_000),
                calibration_profile_digest: "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
                process_context_digest: "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".into(),
                source_data_digest: "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc".into(),
            },
            related: vec![],
            alignment_method: None,
        };
        let digest = envelope.digest().expect("envelope digest");
        let anchor = FpmRegistrationAnchor {
            schema_version: FPM_REGISTRATION_ANCHOR_SCHEMA_VERSION.into(),
            envelope,
            envelope_digest: digest,
        };
        let wrong = "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd";

        assert!(validate_resolved_anchor_envelope(&anchor, wrong).is_err());
    }

    #[test]
    fn fpm_anchor_uppercase_digest_is_rejected() {
        let envelope = RegistrationEnvelope {
            schema_version: FPM_REGISTRATION_SCHEMA_VERSION.into(),
            reference: ModalityObservationRef {
                source_id: "thermal-1".into(),
                modality: "thermal".into(),
                clock_domain: "ptp-domain-1".into(),
                source_sequence: 10,
                correlation_domain: "frame-domain".into(),
                correlation_id: "frame-10".into(),
                source_timestamp_micros: Some(1_000_000),
                calibration_profile_digest: "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
                process_context_digest: "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".into(),
                source_data_digest: "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc".into(),
            },
            related: vec![],
            alignment_method: None,
        };
        let digest = envelope.digest().expect("envelope digest");
        let anchor = FpmRegistrationAnchor {
            schema_version: FPM_REGISTRATION_ANCHOR_SCHEMA_VERSION.into(),
            envelope,
            envelope_digest: digest.clone(),
        };

        assert!(validate_resolved_anchor_envelope(&anchor, &digest.to_uppercase()).is_err());
    }


    #[test]
    fn authenticated_provenance_manifest_is_order_independent() {
        fn action(byte: u8) -> ActionHash {
            ActionHash::from_raw_36(vec![byte; 36])
        }

        fn resolved(byte: u8, witness_digest: &str) -> ResolvedFpmProvenanceAnchor {
            let witness = AcquisitionLineageWitness {
                node_id: format!("node-{byte}"),
                source_id: format!("source-{byte}"),
                modality: format!("modality-{byte}"),
                source_observation_digest: "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
                acquisition_root_digest: "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".into(),
                parent_node_ids: vec![],
            };

            ResolvedFpmProvenanceAnchor {
                provenance_action_hash: action(byte),
                provenance_entry_hash: EntryHash::from_raw_36(vec![byte; 36]),
                registration_anchor_action: action(9),
                witness,
                witness_digest: witness_digest.into(),
                author: AgentPubKey::from_raw_36(vec![1u8; 36]),
                signer: AgentPubKey::from_raw_36(vec![2u8; 36]),
                timestamp: Timestamp::from_micros(1_000),
                action_seq: byte as u32,
                prev_action: None,
            }
        }

        let a = resolved(1, "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc");
        let b = resolved(2, "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd");
        let ordered = authenticated_provenance_manifest_digest(&[a.clone(), b.clone()]);
        assert_eq!(
            ordered,
            authenticated_provenance_manifest_digest(&[b.clone(), a.clone()]),
        );

        let mut scoped = b;
        scoped.registration_anchor_action = action(10);
        assert_ne!(
            ordered,
            authenticated_provenance_manifest_digest(&[a, scoped]),
            "manifest must bind provenance witnesses to registration-anchor scope",
        );
    }

    #[test]
    fn authenticated_root_manifest_is_order_independent_and_scope_bound() {
        fn root(byte: u8) -> ResolvedFpmAcquisitionRootAnchor {
            ResolvedFpmAcquisitionRootAnchor {
                action_hash: ActionHash::from_raw_36(vec![byte; 36]),
                entry_hash: EntryHash::from_raw_36(vec![byte; 36]),
                root_digest: format!("{byte:064x}"),
                source_system_id: format!("system-{byte}"),
                capture_reference: format!("capture-{byte}"),
                artifact_digest: format!("{:064x}", byte as u64),
                author: AgentPubKey::from_raw_36(vec![1u8; 36]),
                signer: AgentPubKey::from_raw_36(vec![2u8; 36]),
                timestamp: Timestamp::from_micros(1_000),
                action_seq: byte as u32,
                prev_action: None,
            }
        }

        let a = root(1);
        let b = root(2);
        let ordered = authenticated_acquisition_root_manifest_digest(&[a.clone(), b.clone()]);
        assert_eq!(
            ordered,
            authenticated_acquisition_root_manifest_digest(&[b.clone(), a.clone()]),
        );

        let mut scoped = b;
        scoped.root_digest = format!("{:064x}", 3u8 as u64);
        assert_ne!(
            ordered,
            authenticated_acquisition_root_manifest_digest(&[a, scoped]),
            "root manifest must commit the root declaration identity",
        );
    }

    #[test]
    fn authenticated_provenance_qualification_digest_is_deterministic() {
        let structural = ProvenanceQualification {
            schema_version: FPM_PROVENANCE_QUALIFICATION_SCHEMA_VERSION.into(),
            registration_envelope_digest: "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            lineage_manifest_digest: "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".into(),
            qualification_basis_digest: "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc".into(),
            profile_id: FPM_PROVENANCE_PROFILE_ID.into(),
            profile_version: FPM_PROVENANCE_PROFILE_VERSION.into(),
            status: ProvenanceQualificationStatus::QualifiedForProfile,
            reasons: vec![],
        };
        let qualification = AuthenticatedFpmProvenanceQualification {
            schema_version: FPM_AUTHENTICATED_PROVENANCE_SCHEMA_VERSION.into(),
            registration_anchor_action: ActionHash::from_raw_36(vec![7u8; 36]),
            registration_envelope_digest: "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            provenance_anchor_manifest_digest: "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd".into(),
            acquisition_root_anchor_manifest_digest: "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee".into(),
            structural_qualification: structural,
            witnesses: vec![],
            roots: vec![],
        };

        assert_eq!(qualification.digest(), qualification.digest());
        assert_eq!(qualification.digest().len(), 64);
    }


}