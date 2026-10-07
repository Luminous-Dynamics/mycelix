//! External settlement receipts.
//!
//! A receipt is observational evidence emitted by a chain adapter.  It is not
//! a source of monetary authority and must be checked against the Mycelix claim
//! it purports to settle.

use serde::{Deserialize, Serialize};
use super::{SettlementClaim, SettlementFinality, SettlementValidationError};

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


#[cfg(test)]
mod tests {
    use super::*;

    fn claim() -> SettlementClaim {
        SettlementClaim {
            settlement_claim_id: "claim-1".into(),
            mycelix_event_id: "event-1".into(),
            source_issuance_or_obligation_ref: "obligation-1".into(),
            rail_id: "polygon-pos-mainnet".into(),
            chain_id: 137,
            source_domain: "mycelix".into(),
            destination_domain: "polygon:137".into(),
            asset_id: "SAP".into(),
            token_contract: Some("token-1".into()),
            amount_minor_units: 100,
            unit: "micro-SAP".into(),
            transaction_or_message_id: "0xtx".into(),
            proof_or_attestation_ref: Some("proof-1".into()),
            finality: SettlementFinality::BridgeFinal,
            reconciled: false,
            adapter_version: "polygon-adapter-v1".into(),
            configuration_digest: "config-v1".into(),
            observed_at_micros: 10,
            known_at_micros: 20,
            supersedes_claim_id: None,
        }
    }

    fn receipt() -> SettlementReceipt {
        SettlementReceipt {
            receipt_id: "receipt-1".into(),
            settlement_claim_id: "claim-1".into(),
            rail_id: "polygon-pos-mainnet".into(),
            chain_id: 137,
            asset_id: "SAP".into(),
            token_contract: Some("token-1".into()),
            amount_minor_units: 100,
            unit: "micro-SAP".into(),
            transaction_or_message_id: "0xtx".into(),
            block_reference: Some("block-1".into()),
            proof_or_attestation_ref: Some("proof-1".into()),
            finality: SettlementFinality::BridgeFinal,
            reconciled: false,
            adapter_version: "polygon-adapter-v1".into(),
            configuration_digest: "config-v1".into(),
            observed_at_micros: 11,
            known_at_micros: 21,
            supersedes_receipt_id: None,
        }
    }

    #[test]
    fn matching_claim_and_receipt_bind() {
        assert!(receipt().bind_to_claim(&claim()).is_ok());
    }

    #[test]
    fn amount_mutation_fails_closed() {
        let mut r = receipt();
        r.amount_minor_units += 1;
        assert_eq!(
            r.bind_to_claim(&claim()),
            Err(SettlementValidationError::InvalidField {
                field: "claim_receipt_binding",
                reason: "claim and receipt identities or settlement terms differ",
            })
        );
    }

    #[test]
    fn chain_substitution_fails_closed() {
        let mut r = receipt();
        r.chain_id = 1;
        assert!(r.validate().is_ok());
        assert_eq!(
            r.bind_to_claim(&claim()),
            Err(SettlementValidationError::InvalidField {
                field: "claim_receipt_binding",
                reason: "claim and receipt identities or settlement terms differ",
            })
        );
    }

    #[test]
    fn proof_mutation_fails_closed() {
        let mut r = receipt();
        r.proof_or_attestation_ref = Some("forged-proof".into());
        assert_eq!(
            r.bind_to_claim(&claim()),
            Err(SettlementValidationError::InvalidField {
                field: "claim_receipt_binding",
                reason: "claim and receipt identities or settlement terms differ",
            })
        );
    }

    #[test]
    fn reconciled_state_requires_reconciled_finality() {
        let mut r = receipt();
        r.reconciled = true;
        assert_eq!(
            r.validate(),
            Err(SettlementValidationError::ReconciliationStateMismatch)
        );
    }

    #[test]
    fn disputed_receipt_is_not_admissible() {
        let mut r = receipt();
        r.finality = SettlementFinality::Disputed;
        assert_eq!(
            r.validate(),
            Err(SettlementValidationError::NonAdmissibleFinality)
        );
    }

    #[test]
    fn serde_roundtrip_is_stable() {
        let r = receipt();
        let encoded = serde_json::to_string(&r).unwrap();
        let decoded: SettlementReceipt = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, r);
    }
}
