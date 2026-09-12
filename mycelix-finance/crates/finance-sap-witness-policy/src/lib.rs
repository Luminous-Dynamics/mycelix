#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-018 independently rooted witness-policy theorem for SAP V2 transfers.
//!
//! This crate does not verify Holochain countersigning signatures or session
//! structure. It consumes **already-authenticated session facts** supplied by a
//! future integration boundary, then checks them against an independently trusted
//! policy. Proof-carrying session/authority types are intentionally not serde
//! deserializable: caller JSON must never manufacture authenticated authority.

use finance_sap_transfer_finality::{
    SapTransferWitnessPolicyRefV1, SpendableTransferClaimV2, VerifiedWitnessSpendAssuranceV1,
};
use finance_sap_value_notes::{
    MAX_ACTION_REFERENCE_LEN, MAX_DID_LEN, MAX_ID_LEN, SapTransferOutputAvailabilityV2,
    SapTransferV2, ValidatedTransferClaimV2,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const SAP_WITNESS_POLICY_V1_SCHEMA_VERSION: u16 = 1;
pub const SAP_WITNESS_SESSION_FACTS_V1_SCHEMA_VERSION: u16 = 1;
pub const SAP_WITNESS_PROTOCOL_ID: &str = "mycelix-sap-witness-v1";
pub const MAX_OPTIONAL_WITNESSES: usize = 64;

const POLICY_FINGERPRINT_DOMAIN: &[u8] = b"mycelix:sap-witness-policy-v1\0";

/// DNA/governance-rooted configuration. This is data, not an authentication token.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapWitnessPolicyConfigV1 {
    pub enabled: bool,
    pub active_policy: Option<SapTransferWitnessPolicyV1>,
}

impl Default for SapWitnessPolicyConfigV1 {
    fn default() -> Self {
        Self {
            enabled: false,
            active_policy: None,
        }
    }
}

impl SapWitnessPolicyConfigV1 {
    pub fn validate(&self) -> Result<(), SapWitnessPolicyError> {
        match (&self.enabled, &self.active_policy) {
            (false, None) => Ok(()),
            (false, Some(policy)) | (true, Some(policy)) => policy.validate(),
            (true, None) => Err(SapWitnessPolicyError::EnabledWithoutPolicy),
        }
    }

    pub fn require_enabled_policy(
        &self,
    ) -> Result<&SapTransferWitnessPolicyV1, SapWitnessPolicyError> {
        self.validate()?;
        if !self.enabled {
            return Err(SapWitnessPolicyError::ProtocolDisabled);
        }
        self.active_policy
            .as_ref()
            .ok_or(SapWitnessPolicyError::EnabledWithoutPolicy)
    }
}

/// Independently trusted authority policy. The deterministic fingerprint binds the
/// complete enzyme/witness/quorum authority set so equal ID/version strings cannot
/// silently identify different authority configurations.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapTransferWitnessPolicyV1 {
    pub schema_version: u16,
    pub protocol_id: String,
    pub policy_id: String,
    pub policy_version: u16,
    pub enzyme_did: String,
    pub optional_witness_dids: Vec<String>,
    pub required_optional_witnesses: u16,
    pub policy_fingerprint: String,
}

impl SapTransferWitnessPolicyV1 {
    pub fn new(
        policy_id: String,
        policy_version: u16,
        enzyme_did: String,
        mut optional_witness_dids: Vec<String>,
        required_optional_witnesses: u16,
    ) -> Result<Self, SapWitnessPolicyError> {
        optional_witness_dids.sort();
        let mut policy = Self {
            schema_version: SAP_WITNESS_POLICY_V1_SCHEMA_VERSION,
            protocol_id: SAP_WITNESS_PROTOCOL_ID.into(),
            policy_id,
            policy_version,
            enzyme_did,
            optional_witness_dids,
            required_optional_witnesses,
            policy_fingerprint: String::new(),
        };
        policy.policy_fingerprint = policy.expected_fingerprint()?;
        policy.validate()?;
        Ok(policy)
    }

    pub fn policy_ref(&self) -> SapTransferWitnessPolicyRefV1 {
        SapTransferWitnessPolicyRefV1 {
            policy_id: self.policy_id.clone(),
            policy_version: self.policy_version,
        }
    }

    pub fn validate(&self) -> Result<(), SapWitnessPolicyError> {
        if self.schema_version != SAP_WITNESS_POLICY_V1_SCHEMA_VERSION {
            return Err(SapWitnessPolicyError::UnsupportedPolicySchema);
        }
        if self.protocol_id != SAP_WITNESS_PROTOCOL_ID {
            return Err(SapWitnessPolicyError::UnsupportedProtocol);
        }
        validate_id(&self.policy_id).map_err(|_| SapWitnessPolicyError::InvalidPolicyId)?;
        if self.policy_version == 0 {
            return Err(SapWitnessPolicyError::InvalidPolicyVersion);
        }
        validate_did(&self.enzyme_did)?;
        validate_sorted_unique_dids(&self.optional_witness_dids, false)?;
        if self
            .optional_witness_dids
            .iter()
            .any(|did| did == &self.enzyme_did)
        {
            return Err(SapWitnessPolicyError::EnzymeIsOptionalWitness);
        }
        let count = self.optional_witness_dids.len();
        let required = usize::from(self.required_optional_witnesses);
        if required == 0 || required > count || required <= count / 2 {
            return Err(SapWitnessPolicyError::InvalidWitnessQuorum);
        }
        if self.policy_fingerprint != self.expected_fingerprint()? {
            return Err(SapWitnessPolicyError::PolicyFingerprintMismatch);
        }
        Ok(())
    }

    pub fn expected_fingerprint(&self) -> Result<String, SapWitnessPolicyError> {
        if self.schema_version != SAP_WITNESS_POLICY_V1_SCHEMA_VERSION {
            return Err(SapWitnessPolicyError::UnsupportedPolicySchema);
        }
        if self.protocol_id != SAP_WITNESS_PROTOCOL_ID {
            return Err(SapWitnessPolicyError::UnsupportedProtocol);
        }
        validate_id(&self.policy_id).map_err(|_| SapWitnessPolicyError::InvalidPolicyId)?;
        if self.policy_version == 0 {
            return Err(SapWitnessPolicyError::InvalidPolicyVersion);
        }
        validate_did(&self.enzyme_did)?;
        if self.optional_witness_dids.len() > MAX_OPTIONAL_WITNESSES {
            return Err(SapWitnessPolicyError::TooManyWitnesses);
        }
        let mut hasher = blake3::Hasher::new();
        hasher.update(POLICY_FINGERPRINT_DOMAIN);
        hash_u16(&mut hasher, self.schema_version);
        hash_string(&mut hasher, &self.protocol_id);
        hash_string(&mut hasher, &self.policy_id);
        hash_u16(&mut hasher, self.policy_version);
        hash_string(&mut hasher, &self.enzyme_did);
        hash_u16(&mut hasher, self.required_optional_witnesses);
        hash_u32(
            &mut hasher,
            u32::try_from(self.optional_witness_dids.len())
                .map_err(|_| SapWitnessPolicyError::TooManyWitnesses)?,
        );
        for witness in &self.optional_witness_dids {
            hash_string(&mut hasher, witness);
        }
        Ok(format!("wpolicy:v1:{}", hasher.finalize().to_hex()))
    }
}

/// Ephemeral authenticated facts. There is deliberately no Deserialize impl.
/// A future Holochain adapter must construct this only after verifying the exact
/// countersigning/session evidence and deriving the actual signers itself.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuthenticatedWitnessSessionFactsV1 {
    schema_version: u16,
    protocol_id: String,
    policy_id: String,
    policy_version: u16,
    transfer_id: String,
    spend_action_reference: String,
    input_note_ids: Vec<String>,
    session_action_reference: String,
    enzyme_signer_did: String,
    optional_witness_signer_dids: Vec<String>,
}

impl AuthenticatedWitnessSessionFactsV1 {
    #[allow(clippy::too_many_arguments)]
    pub fn from_authenticated_holochain_session(
        policy_id: String,
        policy_version: u16,
        transfer_id: String,
        spend_action_reference: String,
        mut input_note_ids: Vec<String>,
        session_action_reference: String,
        enzyme_signer_did: String,
        mut optional_witness_signer_dids: Vec<String>,
    ) -> Result<Self, SapWitnessPolicyError> {
        input_note_ids.sort();
        optional_witness_signer_dids.sort();
        let facts = Self {
            schema_version: SAP_WITNESS_SESSION_FACTS_V1_SCHEMA_VERSION,
            protocol_id: SAP_WITNESS_PROTOCOL_ID.into(),
            policy_id,
            policy_version,
            transfer_id,
            spend_action_reference,
            input_note_ids,
            session_action_reference,
            enzyme_signer_did,
            optional_witness_signer_dids,
        };
        facts.validate_shape()?;
        Ok(facts)
    }

    pub fn session_action_reference(&self) -> &str {
        &self.session_action_reference
    }

    pub fn validate_shape(&self) -> Result<(), SapWitnessPolicyError> {
        if self.schema_version != SAP_WITNESS_SESSION_FACTS_V1_SCHEMA_VERSION {
            return Err(SapWitnessPolicyError::UnsupportedSessionSchema);
        }
        if self.protocol_id != SAP_WITNESS_PROTOCOL_ID {
            return Err(SapWitnessPolicyError::UnsupportedProtocol);
        }
        validate_id(&self.policy_id).map_err(|_| SapWitnessPolicyError::InvalidPolicyId)?;
        if self.policy_version == 0 {
            return Err(SapWitnessPolicyError::InvalidPolicyVersion);
        }
        validate_id(&self.transfer_id).map_err(|_| SapWitnessPolicyError::InvalidTransferId)?;
        validate_action_reference(&self.spend_action_reference)?;
        validate_action_reference(&self.session_action_reference)?;
        validate_did(&self.enzyme_signer_did)?;
        validate_sorted_unique_ids(&self.input_note_ids, false)?;
        validate_sorted_unique_dids(&self.optional_witness_signer_dids, true)
    }
}

/// Ephemeral policy-validated authority. Not deserializable; authoritative callers
/// must regenerate it from exact authenticated session facts each time.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PolicyValidatedWitnessAssuranceV1 {
    policy_fingerprint: String,
    enzyme_signer_did: String,
    optional_witness_signer_dids: Vec<String>,
    assurance: VerifiedWitnessSpendAssuranceV1,
}

impl PolicyValidatedWitnessAssuranceV1 {
    pub fn assurance(&self) -> &VerifiedWitnessSpendAssuranceV1 {
        &self.assurance
    }
    pub fn policy_fingerprint(&self) -> &str {
        &self.policy_fingerprint
    }
    pub fn enzyme_signer_did(&self) -> &str {
        &self.enzyme_signer_did
    }
    pub fn optional_witness_signer_dids(&self) -> &[String] {
        &self.optional_witness_signer_dids
    }

    pub fn validate_for(
        &self,
        policy: &SapTransferWitnessPolicyV1,
        transfer: &SapTransferV2,
        spend_action_reference: &str,
    ) -> Result<(), SapWitnessPolicyError> {
        policy.validate()?;
        transfer
            .validate_shape()
            .map_err(|_| SapWitnessPolicyError::InvalidTransfer)?;
        if self.policy_fingerprint != policy.policy_fingerprint {
            return Err(SapWitnessPolicyError::PolicyFingerprintMismatch);
        }
        if self.enzyme_signer_did != policy.enzyme_did {
            return Err(SapWitnessPolicyError::EnzymeSignerMismatch);
        }
        validate_signers_against_policy(policy, &self.optional_witness_signer_dids)?;
        self.assurance
            .validate_for(
                &policy.policy_ref(),
                &transfer.transfer_id,
                spend_action_reference,
            )
            .map_err(|_| SapWitnessPolicyError::DerivedAssuranceMismatch)
    }
}

/// Final ephemeral gate for a future transfer-output spend input. The constructor
/// below additionally requires `ConflictFreeObserved`; conflict/indeterminate
/// parent states can never become spendable under v1 merely because witnesses exist.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PolicyValidatedSpendableTransferClaimV2 {
    spendable_claim: SpendableTransferClaimV2,
    witness_authority: PolicyValidatedWitnessAssuranceV1,
}

impl PolicyValidatedSpendableTransferClaimV2 {
    pub fn claim(&self) -> &SpendableTransferClaimV2 {
        &self.spendable_claim
    }
    pub fn witness_authority(&self) -> &PolicyValidatedWitnessAssuranceV1 {
        &self.witness_authority
    }
    pub fn validate_for(
        &self,
        policy: &SapTransferWitnessPolicyV1,
        transfer: &SapTransferV2,
        spend_action_reference: &str,
    ) -> Result<(), SapWitnessPolicyError> {
        self.witness_authority
            .validate_for(policy, transfer, spend_action_reference)?;
        self.spendable_claim
            .validate_for_policy(&policy.policy_ref())
            .map_err(|_| SapWitnessPolicyError::SpendableClaimMismatch)
    }
}

pub fn validate_authenticated_witness_session(
    policy: &SapTransferWitnessPolicyV1,
    transfer: &SapTransferV2,
    spend_action_reference: &str,
    facts: &AuthenticatedWitnessSessionFactsV1,
) -> Result<PolicyValidatedWitnessAssuranceV1, SapWitnessPolicyError> {
    policy.validate()?;
    transfer
        .validate_shape()
        .map_err(|_| SapWitnessPolicyError::InvalidTransfer)?;
    facts.validate_shape()?;
    validate_action_reference(spend_action_reference)?;
    if facts.policy_id != policy.policy_id || facts.policy_version != policy.policy_version {
        return Err(SapWitnessPolicyError::PolicySelectorMismatch);
    }
    if facts.transfer_id != transfer.transfer_id {
        return Err(SapWitnessPolicyError::SessionTransferMismatch);
    }
    if facts.spend_action_reference != spend_action_reference {
        return Err(SapWitnessPolicyError::SessionSpendActionMismatch);
    }
    if facts.input_note_ids != transfer.input_note_ids {
        return Err(SapWitnessPolicyError::SessionInputSetMismatch);
    }
    if facts.enzyme_signer_did != policy.enzyme_did {
        return Err(SapWitnessPolicyError::EnzymeSignerMismatch);
    }
    validate_signers_against_policy(policy, &facts.optional_witness_signer_dids)?;
    let assurance = VerifiedWitnessSpendAssuranceV1::from_authenticated_evidence(
        transfer.transfer_id.clone(),
        spend_action_reference.to_string(),
        facts.session_action_reference.clone(),
        &policy.policy_ref(),
    )
    .map_err(|_| SapWitnessPolicyError::DerivedAssuranceMismatch)?;
    let validated = PolicyValidatedWitnessAssuranceV1 {
        policy_fingerprint: policy.policy_fingerprint.clone(),
        enzyme_signer_did: facts.enzyme_signer_did.clone(),
        optional_witness_signer_dids: facts.optional_witness_signer_dids.clone(),
        assurance,
    };
    validated.validate_for(policy, transfer, spend_action_reference)?;
    Ok(validated)
}

pub fn authorize_transfer_claim_spendability(
    policy: &SapTransferWitnessPolicyV1,
    transfer: &SapTransferV2,
    spend_action_reference: &str,
    availability: &SapTransferOutputAvailabilityV2,
    claim: ValidatedTransferClaimV2,
    facts: &AuthenticatedWitnessSessionFactsV1,
) -> Result<PolicyValidatedSpendableTransferClaimV2, SapWitnessPolicyError> {
    match availability {
        SapTransferOutputAvailabilityV2::ConflictFreeObserved => {}
        SapTransferOutputAvailabilityV2::FrozenConflict { .. } => {
            return Err(SapWitnessPolicyError::ParentTransferConflicted);
        }
        SapTransferOutputAvailabilityV2::Indeterminate { .. } => {
            return Err(SapWitnessPolicyError::ParentTransferIndeterminate);
        }
    }
    let witness_authority =
        validate_authenticated_witness_session(policy, transfer, spend_action_reference, facts)?;
    let spendable_claim = SpendableTransferClaimV2::from_verified_witness(
        claim,
        spend_action_reference.to_string(),
        witness_authority.assurance().clone(),
        &policy.policy_ref(),
    )
    .map_err(|_| SapWitnessPolicyError::SpendableClaimMismatch)?;
    let validated = PolicyValidatedSpendableTransferClaimV2 {
        spendable_claim,
        witness_authority,
    };
    validated.validate_for(policy, transfer, spend_action_reference)?;
    Ok(validated)
}

fn validate_signers_against_policy(
    policy: &SapTransferWitnessPolicyV1,
    signer_dids: &[String],
) -> Result<(), SapWitnessPolicyError> {
    validate_sorted_unique_dids(signer_dids, true)?;
    let eligible: BTreeSet<&str> = policy
        .optional_witness_dids
        .iter()
        .map(String::as_str)
        .collect();
    for signer in signer_dids {
        if !eligible.contains(signer.as_str()) {
            return Err(SapWitnessPolicyError::IneligibleWitnessSigner);
        }
    }
    if signer_dids.len() < usize::from(policy.required_optional_witnesses) {
        return Err(SapWitnessPolicyError::InsufficientWitnessQuorum);
    }
    Ok(())
}

fn validate_sorted_unique_ids(
    values: &[String],
    allow_empty: bool,
) -> Result<(), SapWitnessPolicyError> {
    if !allow_empty && values.is_empty() {
        return Err(SapWitnessPolicyError::EmptyInputSet);
    }
    let mut previous: Option<&str> = None;
    for value in values {
        validate_id(value).map_err(|_| SapWitnessPolicyError::InvalidTransferId)?;
        if let Some(prev) = previous {
            if prev >= value.as_str() {
                return Err(SapWitnessPolicyError::NonCanonicalInputSet);
            }
        }
        previous = Some(value);
    }
    Ok(())
}
fn validate_sorted_unique_dids(
    values: &[String],
    allow_empty: bool,
) -> Result<(), SapWitnessPolicyError> {
    if !allow_empty && values.is_empty() {
        return Err(SapWitnessPolicyError::EmptyWitnessSet);
    }
    if values.len() > MAX_OPTIONAL_WITNESSES {
        return Err(SapWitnessPolicyError::TooManyWitnesses);
    }
    let mut previous: Option<&str> = None;
    for value in values {
        validate_did(value)?;
        if let Some(prev) = previous {
            if prev >= value.as_str() {
                return Err(SapWitnessPolicyError::NonCanonicalWitnessSet);
            }
        }
        previous = Some(value);
    }
    Ok(())
}
fn validate_did(value: &str) -> Result<(), SapWitnessPolicyError> {
    if value.starts_with("did:") && value.len() <= MAX_DID_LEN {
        Ok(())
    } else {
        Err(SapWitnessPolicyError::InvalidDid)
    }
}
fn validate_id(value: &str) -> Result<(), ()> {
    if value.is_empty() || value.len() > MAX_ID_LEN {
        Err(())
    } else {
        Ok(())
    }
}
fn validate_action_reference(value: &str) -> Result<(), SapWitnessPolicyError> {
    if value.is_empty() || value.len() > MAX_ACTION_REFERENCE_LEN {
        Err(SapWitnessPolicyError::InvalidActionReference)
    } else {
        Ok(())
    }
}
fn hash_u16(hasher: &mut blake3::Hasher, value: u16) {
    hasher.update(&value.to_le_bytes());
}
fn hash_u32(hasher: &mut blake3::Hasher, value: u32) {
    hasher.update(&value.to_le_bytes());
}
fn hash_string(hasher: &mut blake3::Hasher, value: &str) {
    let bytes = value.as_bytes();
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SapWitnessPolicyError {
    ProtocolDisabled,
    EnabledWithoutPolicy,
    UnsupportedPolicySchema,
    UnsupportedSessionSchema,
    UnsupportedProtocol,
    InvalidPolicyId,
    InvalidPolicyVersion,
    InvalidDid,
    EmptyWitnessSet,
    TooManyWitnesses,
    EnzymeIsOptionalWitness,
    NonCanonicalWitnessSet,
    InvalidWitnessQuorum,
    PolicyFingerprintMismatch,
    InvalidTransferId,
    InvalidActionReference,
    EmptyInputSet,
    NonCanonicalInputSet,
    InvalidTransfer,
    PolicySelectorMismatch,
    SessionTransferMismatch,
    SessionSpendActionMismatch,
    SessionInputSetMismatch,
    EnzymeSignerMismatch,
    IneligibleWitnessSigner,
    InsufficientWitnessQuorum,
    DerivedAssuranceMismatch,
    ParentTransferConflicted,
    ParentTransferIndeterminate,
    SpendableClaimMismatch,
}

#[cfg(test)]
mod tests {
    use super::*;
    use finance_sap_account_v2::ValidatedCollateralClaimV2;
    use finance_sap_value_notes::{SapTransferClaimV2, SapValueNoteV2};

    fn policy() -> SapTransferWitnessPolicyV1 {
        SapTransferWitnessPolicyV1::new(
            "sap-witness-main".into(),
            1,
            "did:mycelix:enzyme".into(),
            vec![
                "did:mycelix:w3".into(),
                "did:mycelix:w1".into(),
                "did:mycelix:w2".into(),
            ],
            2,
        )
        .unwrap()
    }
    fn transfer() -> SapTransferV2 {
        let claim = ValidatedCollateralClaimV2 {
            claim_action_reference: "uhCkk-claim".into(),
            action_author_did: "did:mycelix:alice".into(),
            member_did: "did:mycelix:alice".into(),
            issuance_receipt_action_reference: "uhCkk-receipt".into(),
            mint_id: "mint-a".into(),
            deposit_id: "deposit-a".into(),
            amount: 100,
        };
        let input = SapValueNoteV2::from_collateral_claim(&claim).unwrap();
        SapTransferV2::derive(
            "did:mycelix:alice".into(),
            "did:mycelix:bob".into(),
            &[input],
            60,
        )
        .unwrap()
    }
    fn facts(
        policy: &SapTransferWitnessPolicyV1,
        transfer: &SapTransferV2,
    ) -> AuthenticatedWitnessSessionFactsV1 {
        AuthenticatedWitnessSessionFactsV1::from_authenticated_holochain_session(
            policy.policy_id.clone(),
            policy.policy_version,
            transfer.transfer_id.clone(),
            "uhCkk-spend".into(),
            transfer.input_note_ids.clone(),
            "uhCkk-session".into(),
            policy.enzyme_did.clone(),
            vec!["did:mycelix:w2".into(), "did:mycelix:w1".into()],
        )
        .unwrap()
    }
    fn recipient_claim(transfer: &SapTransferV2) -> ValidatedTransferClaimV2 {
        let claim = SapTransferClaimV2::new(
            "did:mycelix:bob".into(),
            transfer.recipient_note.note_id.clone(),
        )
        .unwrap();
        ValidatedTransferClaimV2::from_valid_output(
            "uhCkk-transfer-claim".into(),
            "did:mycelix:bob".into(),
            claim,
            transfer.recipient_note.clone(),
        )
        .unwrap()
    }

    #[test]
    fn default_config_is_disabled_without_authority() {
        let config = SapWitnessPolicyConfigV1::default();
        assert!(config.validate().is_ok());
        assert_eq!(
            config.require_enabled_policy(),
            Err(SapWitnessPolicyError::ProtocolDisabled)
        );
    }
    #[test]
    fn strict_majority_quorum_is_required() {
        assert_eq!(
            SapTransferWitnessPolicyV1::new(
                "policy".into(),
                1,
                "did:mycelix:enzyme".into(),
                vec![
                    "did:mycelix:w1".into(),
                    "did:mycelix:w2".into(),
                    "did:mycelix:w3".into(),
                    "did:mycelix:w4".into()
                ],
                2,
            ),
            Err(SapWitnessPolicyError::InvalidWitnessQuorum)
        );
    }
    #[test]
    fn enzyme_cannot_be_optional_witness() {
        assert_eq!(
            SapTransferWitnessPolicyV1::new(
                "policy".into(),
                1,
                "did:mycelix:enzyme".into(),
                vec!["did:mycelix:enzyme".into()],
                1,
            ),
            Err(SapWitnessPolicyError::EnzymeIsOptionalWitness)
        );
    }
    #[test]
    fn policy_fingerprint_binds_full_authority_set() {
        let a = policy();
        let b = SapTransferWitnessPolicyV1::new(
            a.policy_id.clone(),
            a.policy_version,
            a.enzyme_did.clone(),
            vec![
                "did:mycelix:w1".into(),
                "did:mycelix:w2".into(),
                "did:mycelix:w4".into(),
            ],
            2,
        )
        .unwrap();
        assert_ne!(a.policy_fingerprint, b.policy_fingerprint);
    }
    #[test]
    fn exact_authenticated_quorum_validates_and_revalidates() {
        let policy = policy();
        let transfer = transfer();
        let validated = validate_authenticated_witness_session(
            &policy,
            &transfer,
            "uhCkk-spend",
            &facts(&policy, &transfer),
        )
        .unwrap();
        assert_eq!(validated.enzyme_signer_did(), policy.enzyme_did);
        assert_eq!(validated.optional_witness_signer_dids().len(), 2);
        assert!(
            validated
                .validate_for(&policy, &transfer, "uhCkk-spend")
                .is_ok()
        );
    }
    #[test]
    fn ineligible_or_insufficient_signers_are_rejected() {
        let policy = policy();
        let transfer = transfer();
        let bad = AuthenticatedWitnessSessionFactsV1::from_authenticated_holochain_session(
            policy.policy_id.clone(),
            policy.policy_version,
            transfer.transfer_id.clone(),
            "uhCkk-spend".into(),
            transfer.input_note_ids.clone(),
            "uhCkk-session".into(),
            policy.enzyme_did.clone(),
            vec!["did:mycelix:w1".into(), "did:mycelix:mallory".into()],
        )
        .unwrap();
        assert_eq!(
            validate_authenticated_witness_session(&policy, &transfer, "uhCkk-spend", &bad),
            Err(SapWitnessPolicyError::IneligibleWitnessSigner)
        );
        let insufficient =
            AuthenticatedWitnessSessionFactsV1::from_authenticated_holochain_session(
                policy.policy_id.clone(),
                policy.policy_version,
                transfer.transfer_id.clone(),
                "uhCkk-spend".into(),
                transfer.input_note_ids.clone(),
                "uhCkk-session".into(),
                policy.enzyme_did.clone(),
                vec!["did:mycelix:w1".into()],
            )
            .unwrap();
        assert_eq!(
            validate_authenticated_witness_session(
                &policy,
                &transfer,
                "uhCkk-spend",
                &insufficient
            ),
            Err(SapWitnessPolicyError::InsufficientWitnessQuorum)
        );
    }
    #[test]
    fn conflict_or_indeterminate_parent_never_promotes_to_spendable_v1() {
        let policy = policy();
        let transfer = transfer();
        let claim = recipient_claim(&transfer);
        assert_eq!(
            authorize_transfer_claim_spendability(
                &policy,
                &transfer,
                "uhCkk-spend",
                &SapTransferOutputAvailabilityV2::FrozenConflict {
                    input_note_ids: transfer.input_note_ids.clone()
                },
                claim.clone(),
                &facts(&policy, &transfer)
            ),
            Err(SapWitnessPolicyError::ParentTransferConflicted)
        );
        assert_eq!(
            authorize_transfer_claim_spendability(
                &policy,
                &transfer,
                "uhCkk-spend",
                &SapTransferOutputAvailabilityV2::Indeterminate {
                    input_note_ids: transfer.input_note_ids.clone()
                },
                claim,
                &facts(&policy, &transfer)
            ),
            Err(SapWitnessPolicyError::ParentTransferIndeterminate)
        );
    }
    #[test]
    fn exact_conflict_free_witness_can_authorize_stronger_spendable_claim() {
        let policy = policy();
        let transfer = transfer();
        let spendable = authorize_transfer_claim_spendability(
            &policy,
            &transfer,
            "uhCkk-spend",
            &SapTransferOutputAvailabilityV2::ConflictFreeObserved,
            recipient_claim(&transfer),
            &facts(&policy, &transfer),
        )
        .unwrap();
        assert_eq!(spendable.claim().spendable_note(), &transfer.recipient_note);
        assert!(
            spendable
                .validate_for(&policy, &transfer, "uhCkk-spend")
                .is_ok()
        );
    }
}
