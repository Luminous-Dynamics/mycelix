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
    pub appraisal_policy_digest: String,
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
    pub appraisal_policy_digest: String,
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
    pub attestation_format: String,
    pub verifier_id: String,
    pub verifier_version: String,
    pub verifier_profile_digest: String,
    pub reference_values_digest: String,
    pub endorsement_digest: String,
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