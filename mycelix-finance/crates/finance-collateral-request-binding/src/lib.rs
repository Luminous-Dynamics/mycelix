#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-009 exact request-action binding for collateral settlement.
//!
//! This crate is intentionally storage-independent. Holochain integration must
//! obtain `request_action_reference` from one exact valid V2 request Create
//! action, then construct `BoundCollateralDepositRequest` from that record.

use finance_collateral_deposit::{
    CollateralDepositRequestError, CollateralDepositRequestV2,
};
use finance_collateral_settlement::{
    derive_settlement_intent, CollateralDepositTerms, CollateralSettlementAuthorityPolicy,
    CollateralSettlementError, CollateralSettlementIntent, CustodyEvidenceEnvelope,
    PriceEvidenceEnvelope,
};
use serde::{Deserialize, Serialize};

pub const MAX_REQUEST_ACTION_REFERENCE_LEN: usize = 256;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BoundCollateralDepositRequest {
    pub request_action_reference: String,
    pub request: CollateralDepositRequestV2,
}

impl BoundCollateralDepositRequest {
    pub fn validate(&self) -> Result<(), RequestBindingError> {
        validate_action_reference(&self.request_action_reference)?;
        self.request
            .validate()
            .map_err(RequestBindingError::DepositRequest)?;
        Ok(())
    }

    pub fn settlement_terms(&self) -> CollateralDepositTerms {
        self.request.to_settlement_terms()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BoundCollateralSettlementIntent {
    pub request_action_reference: String,
    pub request: CollateralDepositRequestV2,
    pub intent: CollateralSettlementIntent,
}

impl BoundCollateralSettlementIntent {
    pub fn validate(&self) -> Result<(), RequestBindingError> {
        let bound_request = BoundCollateralDepositRequest {
            request_action_reference: self.request_action_reference.clone(),
            request: self.request.clone(),
        };
        bound_request.validate()?;
        self.intent
            .validate()
            .map_err(RequestBindingError::Settlement)?;

        if self.intent.basis.terms != bound_request.settlement_terms() {
            return Err(RequestBindingError::RequestTermsMismatch);
        }
        Ok(())
    }

    pub fn deposit_id(&self) -> &str {
        &self.request.deposit_id
    }
}

pub fn derive_bound_settlement_intent(
    bound_request: BoundCollateralDepositRequest,
    authority_policy: &CollateralSettlementAuthorityPolicy,
    price_evidence: &PriceEvidenceEnvelope,
    custody_evidence: &CustodyEvidenceEnvelope,
    evaluated_at_micros: i64,
) -> Result<BoundCollateralSettlementIntent, RequestBindingError> {
    bound_request.validate()?;
    let terms = bound_request.settlement_terms();
    let intent = derive_settlement_intent(
        terms,
        authority_policy,
        price_evidence,
        custody_evidence,
        evaluated_at_micros,
    )
    .map_err(RequestBindingError::Settlement)?;

    let bound_intent = BoundCollateralSettlementIntent {
        request_action_reference: bound_request.request_action_reference,
        request: bound_request.request,
        intent,
    };
    bound_intent.validate()?;
    Ok(bound_intent)
}

fn validate_action_reference(reference: &str) -> Result<(), RequestBindingError> {
    if reference.is_empty() || reference.len() > MAX_REQUEST_ACTION_REFERENCE_LEN {
        return Err(RequestBindingError::InvalidRequestActionReference);
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RequestBindingError {
    InvalidRequestActionReference,
    DepositRequest(CollateralDepositRequestError),
    Settlement(CollateralSettlementError),
    RequestTermsMismatch,
}

#[cfg(test)]
mod tests {
    use super::*;
    use finance_collateral_deposit::COLLATERAL_DEPOSIT_NONCE_BYTES;
    use finance_collateral_settlement::{
        CollateralSettlementAuthorityPolicy, CustodyAttestationCapability,
        CustodyEvidenceOutcome, PriceAttestationCapability, PriceEvidenceOutcome, PriceRatio,
        SettlementFreshnessPolicy, COLLATERAL_SETTLEMENT_PROTOCOL_VERSION,
        CUSTODY_ATTESTATION_PROTOCOL_VERSION, PRICE_ATTESTATION_PROTOCOL_VERSION, SAP_ASSET_ID,
    };

    fn request(amount: u64, nonce_byte: u8) -> CollateralDepositRequestV2 {
        CollateralDepositRequestV2::new(
            "did:mycelix:alice".into(),
            "ETH".into(),
            amount,
            [nonce_byte; COLLATERAL_DEPOSIT_NONCE_BYTES],
            10,
        )
        .expect("valid request")
    }

    fn policy() -> CollateralSettlementAuthorityPolicy {
        CollateralSettlementAuthorityPolicy {
            policy_id: "settlement-policy".into(),
            policy_version: 1,
            settlement_protocol_version: COLLATERAL_SETTLEMENT_PROTOCOL_VERSION,
            quote_asset_id: SAP_ASSET_ID.into(),
            price_capability: PriceAttestationCapability {
                provider_id: "approved-price-provider".into(),
                method: "price".into(),
                protocol_version: PRICE_ATTESTATION_PROTOCOL_VERSION,
            },
            custody_capability: CustodyAttestationCapability {
                custodian_id: "did:mycelix:custodian".into(),
                method: "custody".into(),
                protocol_version: CUSTODY_ATTESTATION_PROTOCOL_VERSION,
            },
            price_freshness: SettlementFreshnessPolicy {
                policy_id: "price-fresh".into(),
                policy_version: 1,
                max_age_micros: 100,
            },
            custody_freshness: SettlementFreshnessPolicy {
                policy_id: "custody-fresh".into(),
                policy_version: 1,
                max_age_micros: 100,
            },
        }
    }

    fn price() -> PriceEvidenceEnvelope {
        PriceEvidenceEnvelope {
            base_asset_id: "ETH".into(),
            quote_asset_id: SAP_ASSET_ID.into(),
            capability: policy().price_capability,
            outcome: PriceEvidenceOutcome::Observed {
                rate: PriceRatio {
                    numerator: 2,
                    denominator: 1,
                },
                observation_reference: "price:1".into(),
                observed_at_micros: 11,
            },
        }
    }

    fn custody(req: &CollateralDepositRequestV2) -> CustodyEvidenceEnvelope {
        CustodyEvidenceEnvelope {
            deposit_id: req.deposit_id.clone(),
            depositor_did: req.depositor_did.clone(),
            collateral_asset_id: req.collateral_asset_id.clone(),
            collateral_amount: req.collateral_amount,
            capability: policy().custody_capability,
            outcome: CustodyEvidenceOutcome::Confirmed {
                external_reference: "custody:1".into(),
                confirmed_at_micros: 11,
            },
        }
    }

    #[test]
    fn bound_request_derives_settlement_without_free_form_terms() {
        let req = request(10, 1);
        let bound = BoundCollateralDepositRequest {
            request_action_reference: "uhCkk-request-action".into(),
            request: req.clone(),
        };
        let intent = derive_bound_settlement_intent(
            bound,
            &policy(),
            &price(),
            &custody(&req),
            12,
        )
        .expect("bound settlement");

        assert_eq!(intent.request, req);
        assert_eq!(intent.intent.basis.sap_amount, 20);
        assert_eq!(intent.validate(), Ok(()));
    }

    #[test]
    fn empty_action_reference_is_rejected() {
        let bound = BoundCollateralDepositRequest {
            request_action_reference: String::new(),
            request: request(10, 2),
        };
        assert_eq!(
            bound.validate(),
            Err(RequestBindingError::InvalidRequestActionReference)
        );
    }

    #[test]
    fn different_valid_request_cannot_be_swapped_into_existing_intent() {
        let req_a = request(10, 3);
        let req_b = request(11, 4);
        let mut bound_intent = derive_bound_settlement_intent(
            BoundCollateralDepositRequest {
                request_action_reference: "uhCkk-request-a".into(),
                request: req_a.clone(),
            },
            &policy(),
            &price(),
            &custody(&req_a),
            12,
        )
        .expect("bound settlement");

        bound_intent.request = req_b;
        assert_eq!(
            bound_intent.validate(),
            Err(RequestBindingError::RequestTermsMismatch)
        );
    }

    #[test]
    fn mismatched_custody_for_another_request_fails_closed() {
        let req_a = request(10, 5);
        let req_b = request(10, 6);
        let result = derive_bound_settlement_intent(
            BoundCollateralDepositRequest {
                request_action_reference: "uhCkk-request-a".into(),
                request: req_a,
            },
            &policy(),
            &price(),
            &custody(&req_b),
            12,
        );
        assert!(matches!(
            result,
            Err(RequestBindingError::Settlement(
                CollateralSettlementError::CustodySubjectMismatch
            ))
        ));
    }

    #[test]
    fn serialized_round_trip_retains_exact_action_reference() {
        let req = request(10, 7);
        let intent = derive_bound_settlement_intent(
            BoundCollateralDepositRequest {
                request_action_reference: "uhCkk-request-exact".into(),
                request: req.clone(),
            },
            &policy(),
            &price(),
            &custody(&req),
            12,
        )
        .expect("bound settlement");

        let bytes = serde_json::to_vec(&intent).expect("serialize");
        let decoded: BoundCollateralSettlementIntent =
            serde_json::from_slice(&bytes).expect("deserialize");
        assert_eq!(decoded, intent);
        assert_eq!(decoded.validate(), Ok(()));
    }
}
