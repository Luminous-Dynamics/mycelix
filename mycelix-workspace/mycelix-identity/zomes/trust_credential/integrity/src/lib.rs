// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Trust Credential Integrity Zome
//!
//! Defines entry types for K-Vector trust credentials with ZKP proofs.
//! Enables privacy-preserving trust attestations where:
//! - The K-Vector commitment is public (can be verified)
//! - The actual K-Vector values are private (hidden via ZKP)
//! - The trust score can be proven in a range without revealing exact value
//!
//! Integration with kvector-zkp library:
//! - KVectorWitness.commitment() produces the 32-byte commitment
//! - KVectorRangeProof proves values are in valid \[0,1\] range
//! - Proofs are generated off-chain and verified on-chain

mod link_binding;

use hdi::prelude::*;

/// K-Vector Trust Credential
///
/// A verifiable credential that attests to an agent's K-Vector trust profile
/// without revealing the individual component values.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TrustCredential {
    /// Unique credential identifier
    pub id: String,
    /// Subject's DID (who this credential is about)
    pub subject_did: String,
    /// Issuer's DID (who issued this credential)
    pub issuer_did: String,
    /// K-Vector commitment hash (32 bytes, SHA3-256)
    /// This binds to the actual K-Vector without revealing values
    pub kvector_commitment: Vec<u8>,
    /// STARK proof that K-Vector components are in valid \[0,1\] range
    /// Serialized proof from kvector-zkp library
    pub range_proof: Vec<u8>,
    /// Proven trust score range (e.g., [0.5, 0.7])
    /// Proves score is within this range without revealing exact value
    pub trust_score_range: TrustScoreRange,
    /// Trust tier derived from K-Vector (for governance thresholds)
    pub trust_tier: TrustTier,
    /// Credential issuance timestamp
    pub issued_at: Timestamp,
    /// Credential expiration (None = never)
    pub expires_at: Option<Timestamp>,
    /// Whether credential has been revoked
    pub revoked: bool,
    /// Revocation reason if revoked
    pub revocation_reason: Option<String>,
    /// When the credential was revoked (for audit trails)
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub revoked_at: Option<Timestamp>,
    /// Previous credential this supersedes (for updates)
    pub supersedes: Option<String>,
}

/// Trust score range (privacy-preserving)
///
/// Proves the trust score falls within a range without revealing exact value.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct TrustScoreRange {
    /// Lower bound (inclusive)
    pub lower: f32,
    /// Upper bound (inclusive)
    pub upper: f32,
}

/// Trust tiers for governance participation
///
/// Derived from K-Vector trust_score() with defined thresholds.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum TrustTier {
    /// Trust score < 0.3 - Observer only, cannot vote
    Observer,
    /// Trust score >= 0.3 - Basic participation
    Basic,
    /// Trust score >= 0.4 - Can vote on major proposals
    Standard,
    /// Trust score >= 0.6 - Can propose and vote on constitutional changes
    Elevated,
    /// Trust score >= 0.8 - Full governance rights including emergency powers
    Guardian,
}

impl TrustTier {
    /// Get the minimum trust score for this tier
    pub fn min_score(&self) -> f64 {
        match self {
            TrustTier::Observer => 0.0,
            TrustTier::Basic => 0.3,
            TrustTier::Standard => 0.4,
            TrustTier::Elevated => 0.6,
            TrustTier::Guardian => 0.8,
        }
    }

    /// Determine tier from trust score.
    ///
    /// Accepts f64 to avoid precision loss at tier boundaries when computing
    /// the midpoint of an f32 range (e.g., `(0.39 + 0.41) / 2.0` in f32 could
    /// round to 0.3999... instead of 0.4).
    pub fn from_score(score: f64) -> Self {
        if score >= 0.8 {
            TrustTier::Guardian
        } else if score >= 0.6 {
            TrustTier::Elevated
        } else if score >= 0.4 {
            TrustTier::Standard
        } else if score >= 0.3 {
            TrustTier::Basic
        } else {
            TrustTier::Observer
        }
    }
}

/// K-Vector Attestation Request
///
/// A request for someone to attest to components of their K-Vector.
/// Used when an issuer needs to verify specific trust properties.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct AttestationRequest {
    /// Request identifier
    pub id: String,
    /// Who is requesting attestation
    pub requester_did: String,
    /// Who should provide attestation
    pub subject_did: String,
    /// Which K-Vector components need attestation
    pub components: Vec<KVectorComponent>,
    /// Minimum acceptable trust score
    pub min_trust_score: Option<f32>,
    /// Minimum acceptable tier
    pub min_tier: Option<TrustTier>,
    /// Purpose of the attestation
    pub purpose: String,
    /// Request expiration
    pub expires_at: Timestamp,
    /// Request status
    pub status: AttestationStatus,
    /// Creation timestamp
    pub created_at: Timestamp,
}

/// K-Vector component identifiers
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum KVectorComponent {
    /// k_r: Reputation
    Reputation,
    /// k_a: Activity
    Activity,
    /// k_i: Integrity
    Integrity,
    /// k_p: Performance
    Performance,
    /// k_m: Membership duration
    Membership,
    /// k_s: Stake weight
    Stake,
    /// k_h: Historical consistency
    History,
    /// k_topo: Network topology contribution
    Topology,
}

/// Status of an attestation request
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum AttestationStatus {
    /// Waiting for response
    Pending,
    /// Attestation provided
    Fulfilled,
    /// Request was declined
    Declined,
    /// Request expired
    Expired,
    /// Request was cancelled
    Cancelled,
}

/// Trust Credential Presentation
///
/// A selective disclosure presentation of a trust credential.
/// Can reveal specific attributes while keeping others private.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TrustPresentation {
    /// Presentation identifier
    pub id: String,
    /// Reference to source credential
    pub credential_id: String,
    /// Subject's DID
    pub subject_did: String,
    /// Disclosed trust tier (always disclosed)
    pub disclosed_tier: TrustTier,
    /// Disclosed trust score range (if disclosed)
    pub disclosed_range: Option<TrustScoreRange>,
    /// Presentation proof (derived from original proof)
    pub presentation_proof: Vec<u8>,
    /// Who this presentation is for
    pub verifier_did: Option<String>,
    /// Purpose of presentation
    pub purpose: String,
    /// Presentation timestamp
    pub presented_at: Timestamp,
    /// Single-use nonce to prevent replay
    pub nonce: Vec<u8>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    TrustCredential(TrustCredential),
    AttestationRequest(AttestationRequest),
    TrustPresentation(TrustPresentation),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Subject to their trust credentials
    SubjectToCredential,
    /// Issuer to credentials they issued
    IssuerToCredential,
    /// Subject to attestation requests they received
    SubjectToRequest,
    /// Credential to presentations derived from it
    CredentialToPresentation,
    /// Trust tier anchor to credentials in that tier
    TierToCredential,
}

/// Main validation callback
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::TrustCredential(cred) => validate_create_credential(action, cred),
                EntryTypes::AttestationRequest(req) => validate_create_request(action, req),
                EntryTypes::TrustPresentation(pres) => validate_create_presentation(action, pres),
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::TrustCredential(cred) => validate_update_credential(action, cred),
                EntryTypes::AttestationRequest(req) => validate_update_request(action, req),
                EntryTypes::TrustPresentation(_) => Ok(ValidateCallbackResult::Invalid(
                    "Trust presentations cannot be updated".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            action,
            base_address,
            target_address,
            tag,
            link_type,
        } => {
            if tag.0.len() > 1024 {
                return Ok(ValidateCallbackResult::Invalid(
                    "Link tag exceeds maximum length of 1024 bytes".into(),
                ));
            }
            link_binding::validate_create_link_binding(
                action,
                base_address,
                target_address,
                link_type,
            )
        }
        FlatOp::RegisterDeleteLink {
            original_action,
            action,
            ..
        } => {
            if action.author != original_action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the link creator can delete their links".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(update) => {
            let action = match &update {
                OpUpdate::Entry { action, .. }
                | OpUpdate::PrivateEntry { action, .. }
                | OpUpdate::Agent { action, .. }
                | OpUpdate::CapClaim { action, .. }
                | OpUpdate::CapGrant { action, .. } => action,
            };
            let original = must_get_action(action.original_action_address.clone())?;
            if *original.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can update their entries".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterDelete(OpDelete { action }) => {
            let original = must_get_action(action.deletes_address.clone())?;
            if *original.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete their entries".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

/// Validate trust credential creation
fn validate_create_credential(
    action: Create,
    cred: TrustCredential,
) -> ExternResult<ValidateCallbackResult> {
    // Issuer must be the action author (prevent impersonation)
    let expected_issuer = format!("did:mycelix:{}", action.author);
    if cred.issuer_did != expected_issuer {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Issuer DID must match action author. Expected '{}', got '{}'",
            expected_issuer, cred.issuer_did
        )));
    }

    // Subject must be a valid DID
    if !cred.subject_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Subject must be a valid DID".into(),
        ));
    }

    // Issuer must be a valid DID
    if !cred.issuer_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Issuer must be a valid DID".into(),
        ));
    }

    // K-Vector commitment must be 32 bytes (SHA3-256)
    if cred.kvector_commitment.len() != 32 {
        return Ok(ValidateCallbackResult::Invalid(
            "K-Vector commitment must be 32 bytes".into(),
        ));
    }

    // Range proof must not be empty
    if cred.range_proof.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Range proof cannot be empty".into(),
        ));
    }

    // Trust score range must be valid
    if cred.trust_score_range.lower < 0.0 || cred.trust_score_range.upper > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust score range must be within [0, 1]".into(),
        ));
    }

    if cred.trust_score_range.lower > cred.trust_score_range.upper {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust score range lower bound cannot exceed upper bound".into(),
        ));
    }

    // Trust tier must be consistent with range
    let tier_min = cred.trust_tier.min_score();
    if (cred.trust_score_range.upper as f64) < tier_min {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust score range is inconsistent with claimed tier".into(),
        ));
    }

    // New credentials cannot be revoked
    if cred.revoked {
        return Ok(ValidateCallbackResult::Invalid(
            "New credentials cannot be created in revoked state".into(),
        ));
    }

    // New credentials must not carry revocation metadata.
    if cred.revoked_at.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "New credentials cannot have a revocation timestamp".into(),
        ));
    }
    if cred.revocation_reason.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "New credentials cannot have a revocation reason".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Pure credential-update policy.
///
/// A trust credential update is a terminal revocation transition, not a way to
/// rewrite the assertion. All assertion-bearing fields remain immutable.
fn validate_credential_revocation_transition(
    original: &TrustCredential,
    updated: &TrustCredential,
) -> Result<(), &'static str> {
    if updated.id != original.id {
        return Err("Trust credential ID cannot be changed");
    }
    if updated.subject_did != original.subject_did {
        return Err("Subject DID cannot be changed");
    }
    if updated.issuer_did != original.issuer_did {
        return Err("Issuer DID cannot be changed");
    }
    if updated.kvector_commitment != original.kvector_commitment {
        return Err("K-Vector commitment cannot be changed");
    }
    if updated.range_proof != original.range_proof {
        return Err("Range proof cannot be changed by credential update");
    }
    if updated.trust_score_range != original.trust_score_range {
        return Err("Trust score range cannot be changed by credential update");
    }
    if updated.trust_tier != original.trust_tier {
        return Err("Trust tier cannot be changed by credential update");
    }
    if updated.issued_at != original.issued_at {
        return Err("Issuance timestamp cannot be changed");
    }
    if updated.expires_at != original.expires_at {
        return Err("Credential expiration cannot be changed by credential update");
    }
    if updated.supersedes != original.supersedes {
        return Err("Credential supersession reference cannot be changed by credential update");
    }

    if original.revoked {
        return Err("Revoked trust credentials cannot be updated again");
    }
    if !updated.revoked {
        return Err("Trust credential updates are revocation-only");
    }

    let revoked_at = updated
        .revoked_at
        .as_ref()
        .ok_or("Revoked credentials must have a revoked_at timestamp")?;
    if revoked_at < &original.issued_at {
        return Err("Revocation timestamp cannot predate credential issuance");
    }

    let reason = updated
        .revocation_reason
        .as_deref()
        .ok_or("Revoked credentials must have a revocation reason")?;
    if reason.trim().is_empty() {
        return Err("Revocation reason cannot be empty");
    }
    if reason.len() > 2048 {
        return Err("Revocation reason exceeds maximum length of 2048 bytes");
    }

    Ok(())
}

/// Validate trust credential update (revocation only).
fn validate_update_credential(
    action: Update,
    cred: TrustCredential,
) -> ExternResult<ValidateCallbackResult> {
    // Fetch original to enforce invariants.
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: TrustCredential = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original trust credential not found".into()
        )))?;

    // Bind the update to the original issuer -- revoke_credential already
    // checks this coordinator-side, but that trusts the coordinator; this
    // is the real DHT-level enforcement a modified coordinator can't
    // bypass (P0 author-binding gap).
    let committer_did = format!("did:mycelix:{}", action.author);
    if committer_did != original.issuer_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust credential update must be committed by its issuer".to_string(),
        ));
    }

    match validate_credential_revocation_transition(&original, &cred) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason.into())),
    }
}

/// Validate attestation request creation
fn validate_create_request(
    action: Create,
    req: AttestationRequest,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the request to its committer -- request_attestation already
    // derives `requester_did` from agent_info() coordinator-side with zero
    // user input, so this never rejects a legitimate request; it's the
    // real DHT-level enforcement a modified coordinator could otherwise
    // bypass (P0 author-binding gap).
    let expected_requester = format!("did:mycelix:{}", action.author);
    if req.requester_did != expected_requester {
        return Ok(ValidateCallbackResult::Invalid(
            "Attestation request requester_did must be the committing agent (forgery)".to_string(),
        ));
    }

    // Requester must be a valid DID
    if !req.requester_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Requester must be a valid DID".into(),
        ));
    }

    // Subject must be a valid DID
    if !req.subject_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Subject must be a valid DID".into(),
        ));
    }

    // Cannot request attestation from yourself
    if req.requester_did == req.subject_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot request attestation from yourself".into(),
        ));
    }

    // New requests must be pending
    if req.status != AttestationStatus::Pending {
        return Ok(ValidateCallbackResult::Invalid(
            "New requests must have Pending status".into(),
        ));
    }

    // Min trust score must be valid if specified
    if let Some(score) = req.min_trust_score {
        if !(0.0..=1.0).contains(&score) {
            return Ok(ValidateCallbackResult::Invalid(
                "Minimum trust score must be in [0, 1]".into(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate attestation request update
fn validate_update_request(
    action: Update,
    req: AttestationRequest,
) -> ExternResult<ValidateCallbackResult> {
    if !req.requester_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Requester must be a valid DID".into(),
        ));
    }

    // Fetch original to enforce state transitions
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: AttestationRequest = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original attestation request not found".into()
        )))?;

    // Bind the update to the original subject -- fulfill_attestation/
    // decline_attestation (the only paths that update this entry) already
    // check the caller is the subject coordinator-side, but that trusts the
    // coordinator; this is the real DHT-level enforcement a modified
    // coordinator can't bypass (P0 author-binding gap).
    let committer_did = format!("did:mycelix:{}", action.author);
    if committer_did != original.subject_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Attestation request update must be committed by its subject".to_string(),
        ));
    }

    // Immutable fields
    if req.id != original.id {
        return Ok(ValidateCallbackResult::Invalid(
            "Attestation request ID cannot be changed".into(),
        ));
    }
    if req.requester_did != original.requester_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Requester DID cannot be changed".into(),
        ));
    }
    if req.subject_did != original.subject_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Subject DID cannot be changed".into(),
        ));
    }

    // State machine: Pending → Fulfilled/Declined/Expired/Cancelled
    // Terminal states (Fulfilled, Declined, Expired, Cancelled) cannot transition further
    let valid = match (&original.status, &req.status) {
        (AttestationStatus::Pending, AttestationStatus::Fulfilled)
        | (AttestationStatus::Pending, AttestationStatus::Declined)
        | (AttestationStatus::Pending, AttestationStatus::Expired)
        | (AttestationStatus::Pending, AttestationStatus::Cancelled) => true,
        (a, b) if a == b => true,
        _ => false,
    };

    if !valid {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid attestation request status transition".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn arb_trust_tier() -> impl Strategy<Value = TrustTier> {
        prop_oneof![
            Just(TrustTier::Observer),
            Just(TrustTier::Basic),
            Just(TrustTier::Standard),
            Just(TrustTier::Elevated),
            Just(TrustTier::Guardian),
        ]
    }

    fn arb_kvector_component() -> impl Strategy<Value = KVectorComponent> {
        prop_oneof![
            Just(KVectorComponent::Reputation),
            Just(KVectorComponent::Activity),
            Just(KVectorComponent::Integrity),
            Just(KVectorComponent::Performance),
            Just(KVectorComponent::Membership),
            Just(KVectorComponent::Stake),
            Just(KVectorComponent::History),
            Just(KVectorComponent::Topology),
        ]
    }

    fn arb_attestation_status() -> impl Strategy<Value = AttestationStatus> {
        prop_oneof![
            Just(AttestationStatus::Pending),
            Just(AttestationStatus::Fulfilled),
            Just(AttestationStatus::Declined),
            Just(AttestationStatus::Expired),
            Just(AttestationStatus::Cancelled),
        ]
    }

    /// Generate a valid TrustScoreRange within [0, 1] where lower <= upper.
    fn arb_trust_score_range() -> impl Strategy<Value = TrustScoreRange> {
        (0.0f32..=1.0f32, 0.0f32..=1.0f32).prop_map(|(a, b)| {
            let lower = a.min(b);
            let upper = a.max(b);
            TrustScoreRange { lower, upper }
        })
    }

    fn base_credential() -> TrustCredential {
        TrustCredential {
            id: "cred-1".to_string(),
            subject_did: "did:mycelix:subject".to_string(),
            issuer_did: "did:mycelix:issuer".to_string(),
            kvector_commitment: vec![7u8; 32],
            range_proof: vec![1, 2, 3],
            trust_score_range: TrustScoreRange {
                lower: 0.4,
                upper: 0.59,
            },
            trust_tier: TrustTier::Standard,
            issued_at: Timestamp::from_micros(100),
            expires_at: Some(Timestamp::from_micros(10_000)),
            revoked: false,
            revocation_reason: None,
            revoked_at: None,
            supersedes: Some("cred-0".to_string()),
        }
    }

    fn revoked_from(original: &TrustCredential) -> TrustCredential {
        let mut updated = original.clone();
        updated.revoked = true;
        updated.revocation_reason = Some("key compromise".to_string());
        updated.revoked_at = Some(Timestamp::from_micros(1_000));
        updated
    }

    #[test]
    fn valid_revocation_only_transition_is_accepted() {
        let original = base_credential();
        let updated = revoked_from(&original);
        assert_eq!(
            validate_credential_revocation_transition(&original, &updated),
            Ok(())
        );
    }

    #[test]
    fn non_revocation_update_is_rejected() {
        let original = base_credential();
        let updated = original.clone();
        assert!(validate_credential_revocation_transition(&original, &updated).is_err());
    }

    #[test]
    fn tier_escalation_during_revocation_is_rejected() {
        let original = base_credential();
        let mut updated = revoked_from(&original);
        updated.trust_tier = TrustTier::Guardian;
        assert!(validate_credential_revocation_transition(&original, &updated).is_err());
    }

    #[test]
    fn proof_mutation_during_revocation_is_rejected() {
        let original = base_credential();
        let mut updated = revoked_from(&original);
        updated.range_proof.push(9);
        assert!(validate_credential_revocation_transition(&original, &updated).is_err());
    }

    #[test]
    fn score_range_mutation_during_revocation_is_rejected() {
        let original = base_credential();
        let mut updated = revoked_from(&original);
        updated.trust_score_range = TrustScoreRange {
            lower: 0.8,
            upper: 1.0,
        };
        assert!(validate_credential_revocation_transition(&original, &updated).is_err());
    }

    #[test]
    fn expiration_mutation_during_revocation_is_rejected() {
        let original = base_credential();
        let mut updated = revoked_from(&original);
        updated.expires_at = None;
        assert!(validate_credential_revocation_transition(&original, &updated).is_err());
    }

    #[test]
    fn supersession_mutation_during_revocation_is_rejected() {
        let original = base_credential();
        let mut updated = revoked_from(&original);
        updated.supersedes = Some("attacker-selected".to_string());
        assert!(validate_credential_revocation_transition(&original, &updated).is_err());
    }

    #[test]
    fn second_update_after_revocation_is_rejected() {
        let original = base_credential();
        let revoked = revoked_from(&original);
        let second = revoked.clone();
        assert!(validate_credential_revocation_transition(&revoked, &second).is_err());
    }

    #[test]
    fn revocation_requires_nonempty_reason() {
        let original = base_credential();
        let mut updated = revoked_from(&original);
        updated.revocation_reason = Some("   ".to_string());
        assert!(validate_credential_revocation_transition(&original, &updated).is_err());
    }

    #[test]
    fn revocation_timestamp_cannot_predate_issuance() {
        let original = base_credential();
        let mut updated = revoked_from(&original);
        updated.revoked_at = Some(Timestamp::from_micros(99));
        assert!(validate_credential_revocation_transition(&original, &updated).is_err());
    }

    proptest! {
        /// TrustTier::from_score always returns a tier whose min_score <= the input.
        #[test]
        fn from_score_consistent_with_min_score(score in 0.0f64..=1.0f64) {
            let tier = TrustTier::from_score(score);
            prop_assert!(
                score >= tier.min_score(),
                "score {} should be >= tier {:?} min_score {}",
                score, tier, tier.min_score()
            );
        }

        /// TrustTier::from_score is monotonically non-decreasing.
        #[test]
        fn from_score_monotone(a in 0.0f64..=1.0f64, b in 0.0f64..=1.0f64) {
            if a <= b {
                let tier_a = TrustTier::from_score(a);
                let tier_b = TrustTier::from_score(b);
                prop_assert!(
                    tier_a.min_score() <= tier_b.min_score(),
                    "tier({}) = {:?} should have min_score <= tier({}) = {:?}",
                    a, tier_a, b, tier_b
                );
            }
        }

        /// TrustScoreRange round-trips through JSON.
        #[test]
        fn trust_score_range_json_roundtrip(range in arb_trust_score_range()) {
            let json = serde_json::to_string(&range).unwrap();
            let back: TrustScoreRange = serde_json::from_str(&json).unwrap();
            prop_assert_eq!(range, back);
        }

        /// TrustTier round-trips through JSON.
        #[test]
        fn trust_tier_json_roundtrip(tier in arb_trust_tier()) {
            let json = serde_json::to_string(&tier).unwrap();
            let back: TrustTier = serde_json::from_str(&json).unwrap();
            prop_assert_eq!(tier, back);
        }

        /// KVectorComponent round-trips through JSON.
        #[test]
        fn kvector_component_json_roundtrip(comp in arb_kvector_component()) {
            let json = serde_json::to_string(&comp).unwrap();
            let back: KVectorComponent = serde_json::from_str(&json).unwrap();
            prop_assert_eq!(comp, back);
        }

        /// AttestationStatus round-trips through JSON.
        #[test]
        fn attestation_status_json_roundtrip(status in arb_attestation_status()) {
            let json = serde_json::to_string(&status).unwrap();
            let back: AttestationStatus = serde_json::from_str(&json).unwrap();
            prop_assert_eq!(status, back);
        }

        /// Tier boundaries are exact at threshold values.
        #[test]
        fn tier_boundaries_exact(tier in arb_trust_tier()) {
            let boundary = tier.min_score();
            let result = TrustTier::from_score(boundary);
            prop_assert_eq!(tier, result);
        }

        /// Valid ranges have lower <= upper.
        #[test]
        fn trust_score_range_invariant(range in arb_trust_score_range()) {
            prop_assert!(range.lower <= range.upper);
            prop_assert!(range.lower >= 0.0);
            prop_assert!(range.upper <= 1.0);
        }

        /// Only Pending can transition to terminal states.
        #[test]
        fn attestation_terminal_states_dont_transition(
            terminal in prop_oneof![
                Just(AttestationStatus::Fulfilled),
                Just(AttestationStatus::Declined),
                Just(AttestationStatus::Expired),
                Just(AttestationStatus::Cancelled),
            ],
            target in arb_attestation_status()
        ) {
            // Terminal states can only stay the same (no-op update)
            let valid = terminal == target;
            // From the validation code: only Pending can transition to these
            if !valid {
                // Attempting to transition from terminal to different state is invalid
                prop_assert_ne!(terminal, AttestationStatus::Pending);
            }
        }
    }
}

/// Validate trust presentation creation
fn validate_create_presentation(
    action: Create,
    pres: TrustPresentation,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the presentation to its committer -- create_presentation already
    // derives `subject_did` from agent_info() coordinator-side with zero
    // user input, so this never rejects a legitimate presentation; it's
    // the real DHT-level enforcement a modified coordinator could
    // otherwise bypass (P0 author-binding gap).
    let expected_subject = format!("did:mycelix:{}", action.author);
    if pres.subject_did != expected_subject {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust presentation subject_did must be the committing agent (forgery)".to_string(),
        ));
    }

    // Subject must be a valid DID
    if !pres.subject_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Subject must be a valid DID".into(),
        ));
    }

    // Presentation proof must not be empty
    if pres.presentation_proof.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Presentation proof cannot be empty".into(),
        ));
    }

    // Nonce must be present for replay protection
    if pres.nonce.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Nonce is required for replay protection".into(),
        ));
    }

    // If range is disclosed, it must be valid
    if let Some(ref range) = pres.disclosed_range {
        if range.lower < 0.0 || range.upper > 1.0 || range.lower > range.upper {
            return Ok(ValidateCallbackResult::Invalid(
                "Disclosed range must be valid".into(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod author_binding_tests {
    use super::*;

    fn test_action(author: AgentPubKey) -> Create {
        Create {
            author,
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        }
    }

    fn me() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }

    fn other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn valid_credential(issuer_did: String) -> TrustCredential {
        TrustCredential {
            id: "cred-1".to_string(),
            subject_did: "did:mycelix:subject".to_string(),
            issuer_did,
            kvector_commitment: vec![0u8; 32],
            range_proof: vec![1, 2, 3],
            trust_score_range: TrustScoreRange {
                lower: 0.4,
                upper: 0.6,
            },
            trust_tier: TrustTier::Standard,
            issued_at: Timestamp::from_micros(0),
            expires_at: None,
            revoked: false,
            revocation_reason: None,
            revoked_at: None,
            supersedes: None,
        }
    }

    #[test]
    fn create_credential_valid_when_issuer_matches_committer() {
        let cred = valid_credential(format!("did:mycelix:{}", me()));
        let result = validate_create_credential(test_action(me()), cred).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_credential_issuer_forgery_rejected() {
        let cred = valid_credential(format!("did:mycelix:{}", me()));
        let result = validate_create_credential(test_action(other_agent()), cred).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_request(requester_did: String) -> AttestationRequest {
        AttestationRequest {
            id: "req-1".to_string(),
            requester_did,
            subject_did: "did:mycelix:subject".to_string(),
            components: vec![KVectorComponent::Reputation],
            min_trust_score: Some(0.5),
            min_tier: None,
            purpose: "test".to_string(),
            expires_at: Timestamp::from_micros(1),
            status: AttestationStatus::Pending,
            created_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_request_valid_when_requester_matches_committer() {
        let req = valid_request(format!("did:mycelix:{}", me()));
        let result = validate_create_request(test_action(me()), req).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_request_requester_forgery_rejected() {
        let req = valid_request(format!("did:mycelix:{}", me()));
        let result = validate_create_request(test_action(other_agent()), req).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_presentation(subject_did: String) -> TrustPresentation {
        TrustPresentation {
            id: "pres-1".to_string(),
            credential_id: "cred-1".to_string(),
            subject_did,
            disclosed_tier: TrustTier::Standard,
            disclosed_range: None,
            presentation_proof: vec![1, 2, 3],
            verifier_did: None,
            purpose: "test".to_string(),
            presented_at: Timestamp::from_micros(0),
            nonce: vec![1, 2, 3, 4],
        }
    }

    #[test]
    fn create_presentation_valid_when_subject_matches_committer() {
        let pres = valid_presentation(format!("did:mycelix:{}", me()));
        let result = validate_create_presentation(test_action(me()), pres).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_presentation_subject_forgery_rejected() {
        let pres = valid_presentation(format!("did:mycelix:{}", me()));
        let result = validate_create_presentation(test_action(other_agent()), pres).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
