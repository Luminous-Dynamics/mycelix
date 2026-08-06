// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Causal Contribution Integrity Zome
//!
//! Defines entry types and validation rules for ZK-verified causal contributions
//! on Holochain 0.6. This zome tracks verified contributions to the federated
//! learning network with RISC Zero proof attestations.

use hdi::prelude::*;

/// Records the causal contribution of an agent for a single round.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CausalContributionRecord {
    /// The agent whose contribution is being measured.
    pub agent: AgentPubKey,
    /// The round or task ID for which the contribution is being measured.
    pub round_id: u64,
    /// The causal contribution score (C_i).
    pub contribution_score: f64,
    /// A link to the RISC Zero receipt (proof) that validates this calculation.
    pub proof_hash: EntryHash,
    /// The timestamp of the contribution.
    pub timestamp: Timestamp,
}

/// Stores the raw RISC0 receipt bytes (opaque to zome) and attestations
/// by committee members for later audit/verification.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ReceiptEntry {
    /// Raw receipt data (RISC Zero proof bytes)
    pub data: Vec<u8>,
    /// Committee members who have attested to this receipt
    pub attesters: Vec<AgentPubKey>,
    /// Signatures from attesters
    pub signatures: Vec<Signature>,
    /// Optional opaque metadata (e.g., envelope/spec)
    pub meta: Option<Vec<u8>>,
}

/// Configurable policy values for Civitas execution.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CivitasConfig {
    /// Minimum attestations required for a receipt to be valid
    pub min_attestations: u8,
    /// Administrators who can update the config
    pub admins: Vec<AgentPubKey>,
    /// Maximum L-infinity norm for gradient envelope
    pub max_envelope_linf: Option<f32>,
}

/// Committee membership for a given round.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CommitteeEntry {
    /// The round this committee is assigned to
    pub round_id: u64,
    /// Members of the verification committee
    pub members: Vec<AgentPubKey>,
}

/// An aggregated causal score for an agent, updated over time.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct AggregatedCausalScore {
    /// The agent whose score this is.
    pub agent: AgentPubKey,
    /// The aggregated causal score.
    pub total_score: f64,
    /// The number of rounds the agent has participated in.
    pub rounds_participated: u64,
    /// The last time the score was updated.
    pub last_updated: Timestamp,
}

/// Entry types for the causal contribution system
#[hdk_entry_types]
#[unit_enum(EntryTypesUnit)]
pub enum EntryTypes {
    CausalContributionRecord(CausalContributionRecord),
    ReceiptEntry(ReceiptEntry),
    CivitasConfig(CivitasConfig),
    CommitteeEntry(CommitteeEntry),
    AggregatedCausalScore(AggregatedCausalScore),
}

/// Link types for the causal contribution system
#[hdk_link_types]
pub enum LinkTypes {
    /// Links from agent to their causal score entries
    AgentToCausalScore,
    /// Links from agent to their contribution records
    AgentToContributionRecords,
    /// Anchor link for Civitas config
    CivitasConfigAnchor,
    /// Anchor link for committees
    CommitteeAnchor,
    /// Anchor link for rounds
    RoundAnchor,
    /// Links from a ReceiptEntry's proof_hash to a CausalContributionRecord
    /// created from it. Coordinator-side best-effort guard against creating
    /// more than one contribution record from the same receipt (see
    /// record_contribution's doc comment for why this isn't airtight).
    ProofHashToContribution,
}

// === Validation Functions ===

/// Validation for CausalContributionRecord
pub fn validate_contribution_record(
    record: CausalContributionRecord,
) -> ExternResult<ValidateCallbackResult> {
    // Contribution score should be reasonable (not NaN or infinite)
    if record.contribution_score.is_nan() || record.contribution_score.is_infinite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Contribution score must be a valid number".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validation for ReceiptEntry -- previously only checked that attesters/signatures had
/// matching *lengths*, never that any signature was actually valid. Any agent could submit
/// a receipt with a fabricated attester list paired with arbitrary (unverified) signature
/// bytes, forging committee attestations. Now cryptographically verifies each signature
/// against its claimed attester's public key over the receipt data, mirroring the
/// verify_signature_raw pattern already used correctly in mycelix-pulse.keys.
pub fn validate_receipt_entry(receipt: ReceiptEntry) -> ExternResult<ValidateCallbackResult> {
    // Receipt data must not be empty
    if receipt.data.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Receipt data cannot be empty".to_string(),
        ));
    }
    // Attesters and signatures must match in length
    if receipt.attesters.len() != receipt.signatures.len() {
        return Ok(ValidateCallbackResult::Invalid(
            "Number of attesters must match number of signatures".to_string(),
        ));
    }
    // A receipt with zero attesters would otherwise trivially pass the length check above --
    // require at least one real attestation.
    if receipt.attesters.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Receipt must have at least one attester".to_string(),
        ));
    }
    for (attester, signature) in receipt.attesters.iter().zip(receipt.signatures.iter()) {
        if !verify_signature_raw(attester.clone(), signature.clone(), receipt.data.clone())? {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Signature verification failed for attester {}",
                attester
            )));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validation for CivitasConfig
pub fn validate_civitas_config(config: CivitasConfig) -> ExternResult<ValidateCallbackResult> {
    if config.min_attestations == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Minimum attestations must be at least 1".to_string(),
        ));
    }
    if config.admins.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Config must have at least one admin".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validation for CommitteeEntry
pub fn validate_committee_entry(committee: CommitteeEntry) -> ExternResult<ValidateCallbackResult> {
    if committee.members.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Committee must have at least one member".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validation for AggregatedCausalScore
pub fn validate_aggregated_score(
    score: AggregatedCausalScore,
) -> ExternResult<ValidateCallbackResult> {
    if score.total_score.is_nan() || score.total_score.is_infinite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Total score must be a valid number".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation dispatcher
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op {
        Op::StoreEntry(store_entry) => match store_entry.action.hashed.content.entry_type() {
            EntryType::App(app_entry_def) => {
                let entry = store_entry.entry;
                match EntryTypes::deserialize_from_type(
                    app_entry_def.zome_index,
                    app_entry_def.entry_index,
                    &entry,
                )? {
                    Some(EntryTypes::CausalContributionRecord(record)) => {
                        validate_contribution_record(record)
                    }
                    Some(EntryTypes::ReceiptEntry(receipt)) => validate_receipt_entry(receipt),
                    Some(EntryTypes::CivitasConfig(config)) => validate_civitas_config(config),
                    Some(EntryTypes::CommitteeEntry(committee)) => {
                        validate_committee_entry(committee)
                    }
                    Some(EntryTypes::AggregatedCausalScore(score)) => {
                        validate_aggregated_score(score)
                    }
                    None => Ok(ValidateCallbackResult::Valid),
                }
            }
            _ => Ok(ValidateCallbackResult::Valid),
        },
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

/// Proves `validate_receipt_entry` now cryptographically verifies attester
/// signatures rather than only comparing attester/signature list lengths (the
/// P0 author-binding gap this fix closed). Mocks the HDI host's
/// `verify_signature` so this runs as a plain `cargo test`, no live conductor
/// needed -- the mock's return value stands in for "signature check result",
/// letting us prove the code path both rejects a failing check and accepts a
/// passing one, without needing real Ed25519 key material.
#[cfg(test)]
mod tests {
    use super::*;

    struct MockSignatureHdi {
        result: bool,
    }

    impl hdi::hdi::HdiT for MockSignatureHdi {
        fn verify_signature(&self, _: VerifySignature) -> ExternResult<bool> {
            Ok(self.result)
        }
        fn must_get_valid_record(&self, _: MustGetValidRecordInput) -> ExternResult<Record> {
            unimplemented!("not exercised by validate_receipt_entry")
        }
        fn must_get_entry(&self, _: MustGetEntryInput) -> ExternResult<EntryHashed> {
            unimplemented!("not exercised by validate_receipt_entry")
        }
        fn must_get_action(&self, _: MustGetActionInput) -> ExternResult<SignedActionHashed> {
            unimplemented!("not exercised by validate_receipt_entry")
        }
        fn must_get_agent_activity(
            &self,
            _: MustGetAgentActivityInput,
        ) -> ExternResult<Vec<RegisterAgentActivity>> {
            unimplemented!("not exercised by validate_receipt_entry")
        }
        fn dna_info(&self, _: ()) -> ExternResult<DnaInfo> {
            unimplemented!("not exercised by validate_receipt_entry")
        }
        fn zome_info(&self, _: ()) -> ExternResult<ZomeInfo> {
            unimplemented!("not exercised by validate_receipt_entry")
        }
        fn trace(&self, _: TraceMsg) -> ExternResult<()> {
            unimplemented!("not exercised by validate_receipt_entry")
        }
        fn x_salsa20_poly1305_decrypt(
            &self,
            _: XSalsa20Poly1305Decrypt,
        ) -> ExternResult<Option<XSalsa20Poly1305Data>> {
            unimplemented!("not exercised by validate_receipt_entry")
        }
        fn x_25519_x_salsa20_poly1305_decrypt(
            &self,
            _: X25519XSalsa20Poly1305Decrypt,
        ) -> ExternResult<Option<XSalsa20Poly1305Data>> {
            unimplemented!("not exercised by validate_receipt_entry")
        }
        fn ed_25519_x_salsa20_poly1305_decrypt(
            &self,
            _: Ed25519XSalsa20Poly1305Decrypt,
        ) -> ExternResult<XSalsa20Poly1305Data> {
            unimplemented!("not exercised by validate_receipt_entry")
        }
    }

    fn test_receipt() -> ReceiptEntry {
        ReceiptEntry {
            data: vec![1; 32],
            attesters: vec![AgentPubKey::from_raw_36(vec![2; 36])],
            signatures: vec![Signature([3; 64])],
            meta: None,
        }
    }

    #[test]
    fn receipt_with_mismatched_attester_signature_counts_is_rejected_without_host() {
        // Pure length-mismatch check -- no HDI host needed, still enforced.
        let mut receipt = test_receipt();
        receipt.signatures.clear();
        let result = validate_receipt_entry(receipt).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn receipt_rejected_when_signature_verification_fails() {
        hdi::hdi::set_hdi(MockSignatureHdi { result: false });
        let result = validate_receipt_entry(test_receipt()).unwrap();
        assert!(
            matches!(result, ValidateCallbackResult::Invalid(_)),
            "a receipt whose attester signature fails cryptographic verification must be rejected -- \
             previously only list lengths were compared, so any signature bytes of the right count passed"
        );
    }

    #[test]
    fn receipt_accepted_when_signature_verification_succeeds() {
        hdi::hdi::set_hdi(MockSignatureHdi { result: true });
        let result = validate_receipt_entry(test_receipt()).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }
}
