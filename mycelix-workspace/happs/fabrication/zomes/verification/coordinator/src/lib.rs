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
pub struct CreateFpmAttestationChallengeInput {
    pub acquisition_root_action: ActionHash,
    pub attestation_format: String,
    pub verifier_profile_digest: String,
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
    pub evidence_digest: String,
    pub verifier_id: String,
    pub verifier_version: String,
    pub disposition: FpmAttestationDisposition,
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

    Ok(ResolvedFpmAttestationChallenge {
        action_hash: input.action_hash,
        subject_id: challenge.subject_id,
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
            return Ok(true);
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
        expected_acquisition_root_digest: challenge.acquisition_root_digest.clone(),
        expected_challenge_nonce_digest: challenge.nonce_digest.clone(),
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

fn create_fpm_attestation_challenge_impl(
    input: CreateFpmAttestationChallengeInput,
) -> ExternResult<Record> {
    let root = resolve_acquisition_root_action_anchor(
        ResolveFpmAcquisitionRootActionAnchorInput {
            action_hash: input.acquisition_root_action.clone(),
            expected_root_digest: None,
        },
    )?;

    if !valid_attestation_identifier(&input.attestation_format, 128)
        || !valid_fpm_digest(&input.verifier_profile_digest)
        || !valid_fpm_digest(&input.appraisal_policy_digest)
        || !valid_fpm_digest(&input.reference_values_digest)
        || !valid_fpm_digest(&input.endorsement_digest)
    {
        return Err(fpm_attestation_error(
            "attestation challenge verifier policy inputs are malformed",
        ));
    }

    let verifier_agent = agent_info()?.agent_initial_pubkey;
    let nonce = random_bytes(32)
        .map_err(|e| fpm_attestation_error(format!(
            "cryptographic challenge nonce generation failed: {e}"
        )))?
        .as_ref()
        .to_vec();

    let challenge = FpmAttestationChallenge {
        schema_version: FPM_ATTESTATION_CHALLENGE_SCHEMA_VERSION.into(),
        subject_id: root.source_system_id,
        acquisition_root_action: input.acquisition_root_action,
        acquisition_root_digest: root.root_digest,
        verifier_agent,
        attestation_format: input.attestation_format,
        verifier_profile_digest: input.verifier_profile_digest,
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
    let challenge = resolve_fpm_attestation_challenge(
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

    if input.evidence_digest.len() != 64
        || !valid_fpm_digest(&input.evidence_digest)
        || !valid_attestation_identifier(&input.verifier_id, 128)
        || !valid_attestation_identifier(&input.verifier_version, 128)
    {
        return Err(fpm_attestation_error(
            "source attestation evidence/verifier declaration is malformed",
        ));
    }

    let claim = FpmSourceAttestationClaim {
        subject_id: challenge.subject_id.clone(),
        acquisition_root_digest: challenge.acquisition_root_digest.clone(),
        challenge_nonce_digest: challenge.nonce_digest.clone(),
        evidence_digest: input.evidence_digest,
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
    let challenge = resolve_fpm_attestation_challenge(
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