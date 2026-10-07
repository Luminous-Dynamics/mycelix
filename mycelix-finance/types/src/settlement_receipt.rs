//! External settlement receipts.
//!
//! A receipt is observational evidence emitted by a chain adapter.  It is not
//! a source of monetary authority and must be checked against the Mycelix claim
//! it purports to settle.

use serde::{Deserialize, Serialize};
use crate::{SettlementClaim, SettlementFinality, SettlementValidationError};

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SettlementReceipt {
    pub receipt_id: String,
    pub settlement_claim_id: String,
    pub rail_id: String,
    pub chain_id: u64,
    pub asset_id: String,
    pub token_contract: Option<String>,
    pub amount_minor_units: u128,
    pub unit: String,
    pub transaction_or_message_id: String,
    pub block_reference: Option<String>,
    pub proof_or_attestation_ref: Option<String>,
    pub finality: SettlementFinality,
    pub reconciled: bool,
    pub adapter_version: String,
    pub configuration_digest: String,
    pub observed_at_micros: i64,
    pub known_at_micros: i64,
    pub supersedes_receipt_id: Option<String>,
}

impl SettlementReceipt {
    pub fn validate(&self) -> Result<(), SettlementValidationError> {
        if self.receipt_id.trim().is_empty()
            || self.settlement_claim_id.trim().is_empty()
            || self.rail_id.trim().is_empty()
            || self.asset_id.trim().is_empty()
            || self.unit.trim().is_empty()
            || self.transaction_or_message_id.trim().is_empty()
            || self.adapter_version.trim().is_empty()
            || self.configuration_digest.trim().is_empty()
        {
            return Err(SettlementValidationError::InvalidField {
                field: "receipt_identity",
                reason: "required identifiers must not be empty",
            });
        }

        if self.chain_id == 0 {
            return Err(SettlementValidationError::InvalidField {
                field: "chain_id",
                reason: "must be non-zero",
            });
        }

        if self.amount_minor_units == 0 {
            return Err(SettlementValidationError::InvalidField {
                field: "amount_minor_units",
                reason: "must be greater than zero",
            });
        }

        if self.observed_at_micros < 0 || self.known_at_micros < self.observed_at_micros {
            return Err(SettlementValidationError::InvalidField {
                field: "timestamps",
                reason: "known_at must be >= observed_at and both non-negative",
            });
        }

        if matches!(
            &self.finality,
            SettlementFinality::Disputed | SettlementFinality::Superseded
        ) {
            return Err(SettlementValidationError::NonAdmissibleFinality);
        }

        if matches!(
            &self.finality,
            SettlementFinality::BridgeFinal
                | SettlementFinality::EconomicallyFinal
                | SettlementFinality::Reconciled
        ) && self.proof_or_attestation_ref.is_none()
        {
            return Err(SettlementValidationError::MissingProofOrAttestation);
        }

        if self.reconciled != matches!(&self.finality, SettlementFinality::Reconciled) {
            return Err(SettlementValidationError::ReconciliationStateMismatch);
        }

        Ok(())
    }

    pub fn bind_to_claim(
        &self,
        claim: &SettlementClaim,
    ) -> Result<(), SettlementValidationError> {
        self.validate()?;
        claim.validate()?;

        if self.settlement_claim_id != claim.settlement_claim_id
            || self.rail_id != claim.rail_id
            || self.chain_id != claim.chain_id
            || self.asset_id != claim.asset_id
            || self.token_contract != claim.token_contract
            || self.amount_minor_units != claim.amount_minor_units
            || self.unit != claim.unit
            || self.transaction_or_message_id != claim.transaction_or_message_id
            || self.adapter_version != claim.adapter_version
            || self.configuration_digest != claim.configuration_digest
        {
            return Err(SettlementValidationError::InvalidField { field: "claim_receipt_binding", reason: "claim and receipt identities or settlement terms differ", });
        }

        if self.finality.assurance_rank() < claim.finality.assurance_rank() {
            return Err(SettlementValidationError::InsufficientFinality);
        }

        if let Some(claim_proof) = &claim.proof_or_attestation_ref {
            if Some(claim_proof) != self.proof_or_attestation_ref.as_ref() {
                return Err(SettlementValidationError::ClaimReceiptMismatch);
            }
        }

        if claim.reconciled && !self.reconciled {
            return Err(SettlementValidationError::ReconciliationStateMismatch);
        }

        Ok(())
    }
}
