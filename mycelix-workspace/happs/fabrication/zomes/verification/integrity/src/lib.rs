// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Verification Integrity Zome
//!
//! Defines entry types for design verification and safety claims,
//! integrating with the Knowledge hApp for epistemic classification.

use hdi::prelude::*;
use fabrication_common::*;
use fabrication_common::validation;

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(visibility = "public")]
    DesignVerification(DesignVerification),
    #[entry_type(visibility = "public")]
    SafetyClaim(SafetyClaim),
    #[entry_type(visibility = "public")]
    VerificationRequest(VerificationRequest),
    #[entry_type(visibility = "public")]
    FpmRegistrationAnchor(FpmRegistrationAnchor),
    #[entry_type(visibility = "public")]
    FpmProvenanceAnchor(FpmProvenanceAnchor),
    #[entry_type(visibility = "public")]
    FpmAcquisitionRootAnchor(FpmAcquisitionRootAnchor),
    #[entry_type(visibility = "public")]
    FpmAttestationChallenge(FpmAttestationChallenge),
    #[entry_type(visibility = "public")]
    FpmSourceAttestationAnchor(FpmSourceAttestationAnchor),
    #[entry_type(visibility = "public")]
    FpmAttestationChallengeUse(FpmAttestationChallengeUse),
}

#[hdk_link_types]
pub enum LinkTypes {
    FpmChallengeToUses,
    DesignToVerifications,
    DesignToClaims,
    VerifierToVerifications,
    OpenRequests,
    ClaimToKnowledge,
    RateLimitBucket,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct DesignVerification {
    pub design_hash: ActionHash,
    pub verification_type: VerificationType,
    pub result: VerificationResult,
    pub evidence: Vec<ActionHash>,
    pub verifier: AgentPubKey,
    pub verifier_credentials: Vec<String>,
    pub created_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SafetyClaim {
    pub design_hash: ActionHash,
    pub claim_type: SafetyClaimType,
    pub claim_text: String,
    #[serde(default)]
    pub epistemic: Option<ClaimEpistemic>,
    #[serde(default)]
    pub epistemic_provenance: EpistemicProvenance,
    pub supporting_evidence: Vec<String>,
    pub knowledge_claim_hash: Option<ActionHash>,
    pub author: AgentPubKey,
    pub created_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct VerificationRequest {
    pub design_hash: ActionHash,
    pub requester: AgentPubKey,
    pub target_safety_class: SafetyClass,
    pub bounty: Option<u64>,
    pub deadline: Option<Timestamp>,
    pub status: RequestStatus,
    pub created_at: Timestamp,
}

pub const FPM_REGISTRATION_ANCHOR_SCHEMA_VERSION: &str = "fpm.registration.anchor.v1";

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct FpmRegistrationAnchor {
    pub schema_version: String,
    pub envelope: RegistrationEnvelope,
    pub envelope_digest: String,
}

pub const FPM_PROVENANCE_ANCHOR_SCHEMA_VERSION: &str = "fpm.provenance.anchor.v1";

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct FpmProvenanceAnchor {
    pub schema_version: String,
    pub registration_anchor_action: ActionHash,
    pub witness: AcquisitionLineageWitness,
    pub witness_digest: String,
}

pub const FPM_ACQUISITION_ROOT_ANCHOR_SCHEMA_VERSION: &str =
    "fpm.acquisition-root.anchor.v1";

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct FpmAcquisitionRootAnchor {
    pub schema_version: String,
    pub source_system_id: String,
    pub capture_reference: String,
    pub artifact_digest: String,
    pub root_digest: String,
}

pub const FPM_ATTESTATION_CHALLENGE_SCHEMA_VERSION: &str =
    "fpm.attestation.challenge.v1";
pub const FPM_SOURCE_ATTESTATION_ANCHOR_SCHEMA_VERSION: &str =
    "fpm.attestation.result-anchor.v1";
pub const FPM_ATTESTATION_CHALLENGE_USE_SCHEMA_VERSION: &str =
    "fpm.attestation.challenge-use.v1";

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct FpmAttestationChallenge {
    pub schema_version: String,
    pub subject_id: String,
    pub acquisition_root_action: ActionHash,
    pub acquisition_root_digest: String,
    pub verifier_agent: AgentPubKey,
    pub verifier_profile_digest: String,
    pub appraisal_policy_digest: String,
    pub reference_values_digest: String,
    pub endorsement_digest: String,
    pub nonce: Vec<u8>,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct FpmSourceAttestationAnchor {
    pub schema_version: String,
    pub challenge_action: ActionHash,
    pub claim: FpmSourceAttestationClaim,
    pub claim_digest: String,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct FpmAttestationChallengeUse {
    pub schema_version: String,
    pub challenge_action: ActionHash,
    pub attestation_action: ActionHash,
    pub author: AgentPubKey,
}

#[hdk_extern]
pub fn genesis_self_check(_: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(
            OpEntry::CreateEntry { app_entry, .. }
            | OpEntry::UpdateEntry { app_entry, .. }
        ) => match app_entry {
            EntryTypes::DesignVerification(v) => validate_verification(v),
            EntryTypes::SafetyClaim(c) => validate_safety_claim(c),
            EntryTypes::VerificationRequest(r) => validate_verification_request(r),
            EntryTypes::FpmRegistrationAnchor(a) => validate_fpm_registration_anchor(a),
            EntryTypes::FpmProvenanceAnchor(a) => validate_fpm_provenance_anchor(a),
            EntryTypes::FpmAcquisitionRootAnchor(a) => validate_fpm_acquisition_root_anchor(a),
            EntryTypes::FpmAttestationChallenge(a) => validate_fpm_attestation_challenge(a),
            EntryTypes::FpmSourceAttestationAnchor(a) => validate_fpm_source_attestation_anchor(a),
            EntryTypes::FpmAttestationChallengeUse(a) => validate_fpm_attestation_challenge_use(a),
        },
        FlatOp::StoreEntry(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterCreateLink { link_type, tag, .. } => {
            let max_len: usize = 256;
            check!(validation::require_max_tag_len(&tag, max_len, &format!("{:?}", link_type)));
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterDeleteLink { action, .. } => {
            let original_action = must_get_action(action.link_add_address.clone())?;
            if action.author != *original_action.action().author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original link creator can delete this link".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterUpdate(op_update) => {
            let update_action = match op_update {
                OpUpdate::Entry { action, .. }
                | OpUpdate::PrivateEntry { action, .. }
                | OpUpdate::Agent { action, .. }
                | OpUpdate::CapClaim { action, .. }
                | OpUpdate::CapGrant { action, .. } => action,
            };
            let original = must_get_action(update_action.original_action_address.clone())?;
            if update_action.author != *original.hashed.author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original author can update this entry".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterDelete(op_delete) => {
            let original = must_get_action(op_delete.action.deletes_address.clone())?;
            if op_delete.action.author != *original.hashed.author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original author can delete this entry".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

/// Validate a DesignVerification entry.
fn validate_verification(v: DesignVerification) -> ExternResult<ValidateCallbackResult> {
    // --- verifier_credentials: max 32 items, each max 256 chars ---
    check!(validation::require_max_vec_len(
        &v.verifier_credentials, 32, "verifier_credentials"
    ));
    for cred in &v.verifier_credentials {
        check!(validation::require_max_len(cred, 256, "verifier credential"));
    }

    // --- evidence: max 64 items ---
    check!(validation::require_max_vec_len(&v.evidence, 64, "evidence"));

    // --- VerificationResult variant fields ---
    match &v.result {
        VerificationResult::Passed { confidence, notes } => {
            check!(validation::require_in_range(*confidence, 0.0, 1.0, "Passed.confidence"));
            check!(validation::require_max_len(notes, 4096, "Passed.notes"));
        }
        VerificationResult::Failed { reasons } => {
            check!(validation::require_max_vec_len(reasons, 32, "Failed.reasons"));
            for reason in reasons {
                check!(validation::require_max_len(reason, 1024, "Failed reason"));
            }
        }
        VerificationResult::ConditionalPass { conditions, confidence } => {
            check!(validation::require_in_range(*confidence, 0.0, 1.0, "ConditionalPass.confidence"));
            check!(validation::require_max_vec_len(conditions, 32, "ConditionalPass.conditions"));
            for cond in conditions {
                check!(validation::require_max_len(cond, 1024, "ConditionalPass condition"));
            }
        }
        VerificationResult::NeedsMoreEvidence => {}
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a SafetyClaim entry.
fn validate_safety_claim(c: SafetyClaim) -> ExternResult<ValidateCallbackResult> {
    // --- claim_text: non-empty (trim), max 4096 chars ---
    check!(validation::require_non_empty(&c.claim_text, "claim_text"));
    check!(validation::require_max_len(&c.claim_text, 4096, "claim_text"));

    // --- epistemic enrichment/provenance consistency ---
    if let Some(epistemic) = &c.epistemic {
        check!(validation::require_in_range(
            epistemic.empirical,
            0.0,
            1.0,
            "epistemic.empirical"
        ));
        check!(validation::require_in_range(
            epistemic.normative,
            0.0,
            1.0,
            "epistemic.normative"
        ));
        check!(validation::require_in_range(
            epistemic.mythic,
            0.0,
            1.0,
            "epistemic.mythic"
        ));
    }

    match c.epistemic_provenance {
        EpistemicProvenance::KnowledgeClassified if c.epistemic.is_none() => {
            return Ok(ValidateCallbackResult::Invalid(
                "KnowledgeClassified requires an epistemic classification".to_string(),
            ));
        }
        EpistemicProvenance::KnowledgeUnavailable | EpistemicProvenance::KnowledgeMalformed
            if c.epistemic.is_some() =>
        {
            return Ok(ValidateCallbackResult::Invalid(
                "unavailable/malformed Knowledge provenance cannot carry an epistemic classification"
                    .to_string(),
            ));
        }
        _ => {}
    }

    // Legacy records deserialize as LegacyUnattributed and remain valid/queryable,
    // but current scoring excludes them from Knowledge-sourced aggregates.

    // --- supporting_evidence: max 64 items, each max 256 chars ---
    check!(validation::require_max_vec_len(
        &c.supporting_evidence, 64, "supporting_evidence"
    ));
    for ev in &c.supporting_evidence {
        check!(validation::require_max_len(ev, 256, "supporting evidence item"));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate an FPM registration anchor entry.
fn validate_fpm_registration_anchor(
    anchor: FpmRegistrationAnchor,
) -> ExternResult<ValidateCallbackResult> {
    if anchor.schema_version != FPM_REGISTRATION_ANCHOR_SCHEMA_VERSION {
        return Ok(ValidateCallbackResult::Invalid(
            "unsupported FPM registration anchor schema".into(),
        ));
    }
    anchor.envelope.validate_consistency().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "FPM registration anchor envelope is not consistent: {e}"
        )))
    })?;
    let computed_digest = anchor.envelope.digest().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "failed to hash FPM registration anchor envelope: {e}"
        )))
    })?;
    if computed_digest != anchor.envelope_digest {
        return Ok(ValidateCallbackResult::Invalid(
            "FPM registration anchor envelope digest mismatch".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate an FPM acquisition-root anchor entry.
fn validate_fpm_acquisition_root_anchor(
    anchor: FpmAcquisitionRootAnchor,
) -> ExternResult<ValidateCallbackResult> {
    if anchor.schema_version != FPM_ACQUISITION_ROOT_ANCHOR_SCHEMA_VERSION {
        return Ok(ValidateCallbackResult::Invalid(
            "unsupported FPM acquisition-root anchor schema".into(),
        ));
    }
    for (name, value, max_len) in [
        ("source_system_id", &anchor.source_system_id, 128usize),
        ("capture_reference", &anchor.capture_reference, 256usize),
    ] {
        if value.is_empty()
            || value != value.trim()
            || value.len() > max_len
            || value.chars().any(char::is_control)
        {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "invalid FPM acquisition-root {name}"
            )));
        }
    }
    for (name, value) in [
        ("artifact_digest", &anchor.artifact_digest),
        ("root_digest", &anchor.root_digest),
    ] {
        if value.len() != 64
            || !value.bytes().all(|byte| matches!(byte, b'0'..=b'9' | b'a'..=b'f'))
        {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "{name} must be canonical lowercase SHA-256"
            )));
        }
    }
    if acquisition_root_binding_digest(
        &anchor.source_system_id,
        &anchor.capture_reference,
        &anchor.artifact_digest,
    ) != anchor.root_digest {
        return Ok(ValidateCallbackResult::Invalid(
            "FPM acquisition-root digest mismatch".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}


fn canonical_attestation_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| matches!(byte, b'0'..=b'9' | b'a'..=b'f'))
}

fn valid_attestation_identifier(value: &str, max_len: usize) -> bool {
    !value.trim().is_empty()
        && value == value.trim()
        && value.len() <= max_len
        && !value.chars().any(char::is_control)
}

fn validate_fpm_attestation_challenge(
    challenge: FpmAttestationChallenge,
) -> ExternResult<ValidateCallbackResult> {
    if challenge.schema_version != FPM_ATTESTATION_CHALLENGE_SCHEMA_VERSION {
        return Ok(ValidateCallbackResult::Invalid(
            "unsupported FPM attestation challenge schema".into(),
        ));
    }
    if !valid_attestation_identifier(&challenge.subject_id, 128)
        || !canonical_attestation_digest(&challenge.acquisition_root_digest)
        || !canonical_attestation_digest(&challenge.verifier_profile_digest)
        || !canonical_attestation_digest(&challenge.appraisal_policy_digest)
        || !canonical_attestation_digest(&challenge.reference_values_digest)
        || !canonical_attestation_digest(&challenge.endorsement_digest)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "malformed FPM attestation challenge binding".into(),
        ));
    }
    if challenge.nonce.len() < 8 || challenge.nonce.len() > 64 {
        return Ok(ValidateCallbackResult::Invalid(
            "FPM attestation challenge nonce must contain 8..64 bytes".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_fpm_source_attestation_anchor(
    anchor: FpmSourceAttestationAnchor,
) -> ExternResult<ValidateCallbackResult> {
    if anchor.schema_version != FPM_SOURCE_ATTESTATION_ANCHOR_SCHEMA_VERSION {
        return Ok(ValidateCallbackResult::Invalid(
            "unsupported FPM source attestation anchor schema".into(),
        ));
    }
    let qualification = qualify_source_attestation(&FpmAttestationQualificationInput {
        expected_subject_id: anchor.claim.subject_id.clone(),
        expected_acquisition_root_digest: anchor.claim.acquisition_root_digest.clone(),
        expected_challenge_nonce_digest: anchor.claim.challenge_nonce_digest.clone(),
        expected_attestation_format: anchor.claim.attestation_format.clone(),
        expected_verifier_profile_digest: anchor.claim.verifier_profile_digest.clone(),
        expected_appraisal_policy_digest: anchor.claim.appraisal_policy_digest.clone(),
        expected_reference_values_digest: anchor.claim.reference_values_digest.clone(),
        expected_endorsement_digest: anchor.claim.endorsement_digest.clone(),
        claim: anchor.claim.clone(),
    });
    if qualification.status == FpmAttestationQualificationStatus::InvalidEvidence {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid FPM source attestation claim".into(),
        ));
    }
    if anchor.claim_digest != anchor.claim.digest() {
        return Ok(ValidateCallbackResult::Invalid(
            "FPM source attestation claim digest mismatch".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_fpm_attestation_challenge_use(
    use_entry: FpmAttestationChallengeUse,
) -> ExternResult<ValidateCallbackResult> {
    if use_entry.schema_version != FPM_ATTESTATION_CHALLENGE_USE_SCHEMA_VERSION {
        return Ok(ValidateCallbackResult::Invalid(
            "unsupported FPM attestation challenge-use schema".into(),
        ));
    }
    let challenge = must_get_action(use_entry.challenge_action.clone())?;
    if challenge.action_type() != ActionType::Create {
        return Ok(ValidateCallbackResult::Invalid(
            "attestation challenge-use must reference a Create action".into(),
        ));
    }
    let attestation = must_get_action(use_entry.attestation_action.clone())?;
    if attestation.action_type() != ActionType::Create {
        return Ok(ValidateCallbackResult::Invalid(
            "attestation challenge-use must reference a Create attestation action".into(),
        ));
    }
    if use_entry.attestation_action == use_entry.challenge_action {
        return Ok(ValidateCallbackResult::Invalid(
            "attestation challenge-use cannot self-reference challenge".into(),
        ));
    }
    if *challenge.author() != use_entry.author {
        return Ok(ValidateCallbackResult::Invalid(
            "attestation challenge-use author does not match challenge verifier".into(),
        ));
    }
    let expected_attestation_type = EntryType::App(
        UnitEntryTypes::FpmSourceAttestationAnchor
            .try_into()
            .map_err(|_| wasm_error!(WasmErrorInner::Guest(
                "could not construct FPM source attestation entry type".into()
            )))?,
    );
    if attestation.entry_type() != Some(&expected_attestation_type) {
        return Ok(ValidateCallbackResult::Invalid(
            "attestation challenge-use must reference an FPM source attestation".into(),
        ));
    }
    if *attestation.author() != use_entry.author {
        return Ok(ValidateCallbackResult::Invalid(
            "attestation challenge-use author does not match attestation author".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate an FPM provenance anchor entry.
fn validate_fpm_provenance_anchor(
    anchor: FpmProvenanceAnchor,
) -> ExternResult<ValidateCallbackResult> {
    if anchor.schema_version != FPM_PROVENANCE_ANCHOR_SCHEMA_VERSION {
        return Ok(ValidateCallbackResult::Invalid(
            "unsupported FPM provenance anchor schema".into(),
        ));
    }
    if anchor.witness.node_id.trim().is_empty()
        || anchor.witness.node_id != anchor.witness.node_id.trim()
        || anchor.witness.node_id.len() > 128
        || anchor.witness.node_id.chars().any(char::is_control)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid FPM provenance witness node id".into(),
        ));
    }
    for (name, value) in [
        ("source_observation_digest", &anchor.witness.source_observation_digest),
        ("acquisition_root_digest", &anchor.witness.acquisition_root_digest),
        ("witness_digest", &anchor.witness_digest),
    ] {
        if value.len() != 64
            || !value.bytes().all(|byte| matches!(byte, b'0'..=b'9' | b'a'..=b'f'))
        {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "{name} must be canonical lowercase SHA-256"
            )));
        }
    }
    if anchor.witness.parent_node_ids.len() > 32 {
        return Ok(ValidateCallbackResult::Invalid(
            "FPM provenance witness has too many parents".into(),
        ));
    }
    if anchor.witness.parent_node_ids.iter().any(|parent| {
        parent.is_empty()
            || parent != parent.trim()
            || parent.len() > 128
            || parent.chars().any(char::is_control)
    }) {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid FPM provenance witness parent node id".into(),
        ));
    }
    if anchor.witness.source_id.trim().is_empty()
        || anchor.witness.modality.trim().is_empty()
        || anchor.witness.source_id != anchor.witness.source_id.trim()
        || anchor.witness.modality != anchor.witness.modality.trim()
        || anchor.witness.source_id.len() > 128
        || anchor.witness.modality.len() > 128
        || anchor.witness.source_id.chars().any(char::is_control)
        || anchor.witness.modality.chars().any(char::is_control)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid FPM provenance witness participant identifier".into(),
        ));
    }
    if anchor.witness_digest != anchor.witness.digest() {
        return Ok(ValidateCallbackResult::Invalid(
            "FPM provenance witness digest mismatch".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate a VerificationRequest entry.
fn validate_verification_request(r: VerificationRequest) -> ExternResult<ValidateCallbackResult> {
    // --- bounty: if present, max 1_000_000_000 ---
    if let Some(bounty) = r.bounty {
        if bounty > 1_000_000_000 {
            return Ok(ValidateCallbackResult::Invalid(
                "bounty cannot exceed 1000000000".to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- Helpers ----

    fn valid_verification() -> DesignVerification {
        DesignVerification {
            design_hash: ActionHash::from_raw_36(vec![0u8; 36]),
            verification_type: VerificationType::StructuralAnalysis,
            result: VerificationResult::Passed {
                confidence: 0.95,
                notes: "All checks passed".to_string(),
            },
            evidence: vec![ActionHash::from_raw_36(vec![1u8; 36])],
            verifier: AgentPubKey::from_raw_36(vec![0u8; 36]),
            verifier_credentials: vec!["PE License #12345".to_string()],
            created_at: Timestamp::now(),
        }
    }

    fn valid_safety_claim() -> SafetyClaim {
        SafetyClaim {
            design_hash: ActionHash::from_raw_36(vec![0u8; 36]),
            claim_type: SafetyClaimType::LoadCapacity("Supports 50kg".to_string()),
            claim_text: "This bracket supports up to 50kg static load".to_string(),
            epistemic: Some(ClaimEpistemic {
                empirical: 0.8,
                normative: 0.5,
                mythic: 0.1,
            }),
            epistemic_provenance: EpistemicProvenance::KnowledgeClassified,
            supporting_evidence: vec!["FEA report v2.1".to_string()],
            knowledge_claim_hash: None,
            author: AgentPubKey::from_raw_36(vec![0u8; 36]),
            created_at: Timestamp::now(),
        }
    }

    fn valid_verification_request() -> VerificationRequest {
        VerificationRequest {
            design_hash: ActionHash::from_raw_36(vec![0u8; 36]),
            requester: AgentPubKey::from_raw_36(vec![0u8; 36]),
            target_safety_class: SafetyClass::Class2LoadBearing,
            bounty: Some(1000),
            deadline: None,
            status: RequestStatus::Open,
            created_at: Timestamp::now(),
        }
    }

    // ---- DesignVerification tests ----

    #[test]
    fn test_valid_verification_passes() {
        let result = validate_verification(valid_verification()).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_too_many_credentials_rejected() {
        let mut v = valid_verification();
        v.verifier_credentials = (0..33).map(|i| format!("cred-{}", i)).collect();
        let result = validate_verification(v).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(msg) if msg.contains("verifier_credentials")));
    }

    #[test]
    fn test_nan_passed_confidence_rejected() {
        let mut v = valid_verification();
        v.result = VerificationResult::Passed {
            confidence: f32::NAN,
            notes: "ok".to_string(),
        };
        let result = validate_verification(v).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(msg) if msg.contains("Passed.confidence")));
    }

    #[test]
    fn test_nan_conditional_pass_confidence_rejected() {
        let mut v = valid_verification();
        v.result = VerificationResult::ConditionalPass {
            conditions: vec!["Retest after 24h".to_string()],
            confidence: f32::NAN,
        };
        let result = validate_verification(v).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(msg) if msg.contains("ConditionalPass.confidence")));
    }

    // ---- SafetyClaim tests ----

    #[test]
    fn test_valid_claim_passes() {
        let result = validate_safety_claim(valid_safety_claim()).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_unavailable_claim_without_epistemic_score_passes() {
        let mut claim = valid_safety_claim();
        claim.epistemic = None;
        claim.epistemic_provenance = EpistemicProvenance::KnowledgeUnavailable;
        assert_eq!(
            validate_safety_claim(claim).unwrap(),
            ValidateCallbackResult::Valid
        );
    }

    #[test]
    fn test_unavailable_claim_serde_roundtrip_preserves_empty_score() {
        let mut claim = valid_safety_claim();
        claim.epistemic = None;
        claim.epistemic_provenance = EpistemicProvenance::KnowledgeUnavailable;

        let json = serde_json::to_string(&claim).unwrap();
        let restored: SafetyClaim = serde_json::from_str(&json).unwrap();

        assert_eq!(restored.epistemic, None);
        assert_eq!(
            restored.epistemic_provenance,
            EpistemicProvenance::KnowledgeUnavailable
        );
    }

    #[test]
    fn test_unavailable_claim_cannot_carry_epistemic_score() {
        let mut claim = valid_safety_claim();
        claim.epistemic_provenance = EpistemicProvenance::KnowledgeUnavailable;
        assert!(matches!(
            validate_safety_claim(claim).unwrap(),
            ValidateCallbackResult::Invalid(message) if message.contains("unavailable")
        ));
    }

    #[test]
    fn test_knowledge_classified_claim_requires_epistemic_score() {
        let mut claim = valid_safety_claim();
        claim.epistemic = None;
        assert!(matches!(
            validate_safety_claim(claim).unwrap(),
            ValidateCallbackResult::Invalid(message) if message.contains("KnowledgeClassified")
        ));
    }

    #[test]
    fn test_legacy_claim_missing_provenance_defaults_unattributed() {
        let claim = valid_safety_claim();
        let mut value = serde_json::to_value(&claim).unwrap();
        value.as_object_mut().unwrap().remove("epistemic_provenance");

        let restored: SafetyClaim = serde_json::from_value(value).unwrap();
        assert_eq!(
            restored.epistemic_provenance,
            EpistemicProvenance::LegacyUnattributed
        );
        assert_eq!(
            validate_safety_claim(restored).unwrap(),
            ValidateCallbackResult::Valid
        );
    }

    #[test]
    fn test_empty_claim_text_rejected() {
        let mut c = valid_safety_claim();
        c.claim_text = "   ".to_string();
        let result = validate_safety_claim(c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(msg) if msg.contains("claim_text")));
    }

    #[test]
    fn test_nan_empirical_epistemic_rejected() {
        let mut c = valid_safety_claim();
        c.epistemic.empirical = f32::NAN;
        let result = validate_safety_claim(c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(msg) if msg.contains("epistemic.empirical")));
    }

    #[test]
    fn test_nan_normative_epistemic_rejected() {
        let mut c = valid_safety_claim();
        c.epistemic.normative = f32::NAN;
        let result = validate_safety_claim(c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(msg) if msg.contains("epistemic.normative")));
    }

    #[test]
    fn test_nan_mythic_epistemic_rejected() {
        let mut c = valid_safety_claim();
        c.epistemic.mythic = f32::NAN;
        let result = validate_safety_claim(c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(msg) if msg.contains("epistemic.mythic")));
    }

    #[test]
    fn test_too_many_supporting_evidence_rejected() {
        let mut c = valid_safety_claim();
        c.supporting_evidence = (0..65).map(|i| format!("evidence-{}", i)).collect();
        let result = validate_safety_claim(c).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(msg) if msg.contains("supporting_evidence")));
    }

    // ---- VerificationRequest tests ----

    #[test]
    fn test_valid_request_passes() {
        let result = validate_verification_request(valid_verification_request()).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_bounty_too_large_rejected() {
        let mut r = valid_verification_request();
        r.bounty = Some(1_000_000_001);
        let result = validate_verification_request(r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(msg) if msg.contains("bounty")));
    }

    // =========================================================================
    // Link tag validation tests
    // =========================================================================

    #[test]
    fn test_link_tag_at_max_passes() {
        let tag = LinkTag::new(vec![0u8; 256]);
        let result = validation::require_max_tag_len(&tag, 256, "test");
        assert!(result.is_err()); // Err(()) means "no validation issue found"
    }

    #[test]
    fn test_link_tag_over_max_rejected() {
        let tag = LinkTag::new(vec![0u8; 257]);
        let result = validation::require_max_tag_len(&tag, 256, "test");
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(msg)) if msg.contains("link tag")));
    }
}
