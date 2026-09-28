#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Public, policy-bound API for the collateral settlement kernel.
//!
//! The lower-level implementation lives in `lib.rs` but is intentionally kept in
//! a private module here. Public receipt/finalization/retry APIs require the
//! consumer's expected authority policy, preventing a structurally valid
//! self-declared provider policy from becoming authoritative after deserialization.

#[path = "lib.rs"]
mod kernel;

pub use kernel::{
    derive_settlement_intent, CollateralDepositState, CollateralDepositTerms,
    CollateralSettlementAuthorityPolicy, CollateralSettlementBasis,
    CollateralSettlementError, CollateralSettlementIntent, CollateralSettlementReceipt,
    CustodyAttestationCapability, CustodyEvidenceEnvelope, CustodyEvidenceFailure,
    CustodyEvidenceOutcome, PaymentCommitEvidence, PriceAttestationCapability,
    PriceEvidenceEnvelope, PriceEvidenceFailure, PriceEvidenceOutcome, PriceRatio,
    SettlementExecutionPlan, SettlementFreshnessPolicy,
    COLLATERAL_SETTLEMENT_PROTOCOL_VERSION, CUSTODY_ATTESTATION_PROTOCOL_VERSION,
    MAX_EXTERNAL_REFERENCE_LEN, MAX_ID_LEN, PRICE_ATTESTATION_PROTOCOL_VERSION,
    SAP_ASSET_ID,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum PolicyBoundSettlementError {
    AuthorityPolicyMismatch,
    Kernel(CollateralSettlementError),
}

impl From<CollateralSettlementError> for PolicyBoundSettlementError {
    fn from(value: CollateralSettlementError) -> Self {
        Self::Kernel(value)
    }
}

impl CollateralSettlementIntent {
    /// Revalidate structural/economic consistency and require the exact authority
    /// policy expected by the current consumer.
    pub fn validate_against_policy(
        &self,
        expected_policy: &CollateralSettlementAuthorityPolicy,
    ) -> Result<(), PolicyBoundSettlementError> {
        expected_policy.validate()?;
        self.validate()?;
        if &self.basis.authority_policy != expected_policy {
            return Err(PolicyBoundSettlementError::AuthorityPolicyMismatch);
        }
        Ok(())
    }
}

impl CollateralSettlementReceipt {
    /// A persisted receipt is authoritative only under the exact policy expected
    /// by the consumer reading it.
    pub fn validate_against_policy(
        &self,
        expected_policy: &CollateralSettlementAuthorityPolicy,
    ) -> Result<(), PolicyBoundSettlementError> {
        self.validate()?;
        self.intent.validate_against_policy(expected_policy)
    }
}

/// Finalize only an intent authorized under the consumer's exact expected policy.
pub fn finalize_settlement_receipt(
    intent: CollateralSettlementIntent,
    expected_policy: &CollateralSettlementAuthorityPolicy,
    payment_commit: PaymentCommitEvidence,
    settled_at_micros: i64,
) -> Result<CollateralSettlementReceipt, PolicyBoundSettlementError> {
    intent.validate_against_policy(expected_policy)?;
    let receipt =
        kernel::finalize_settlement_receipt(intent, payment_commit, settled_at_micros)?;
    receipt.validate_against_policy(expected_policy)?;
    Ok(receipt)
}

/// Resolve retry semantics using the exact authority policy expected by the
/// consumer. Existing receipts may be returned after their original evidence has
/// aged out, but only if their embedded authority policy still matches exactly.
pub fn plan_settlement_for_deposit(
    terms: &CollateralDepositTerms,
    expected_policy: &CollateralSettlementAuthorityPolicy,
    existing_receipt: Option<&CollateralSettlementReceipt>,
    fresh_intent: Option<&CollateralSettlementIntent>,
) -> Result<SettlementExecutionPlan, PolicyBoundSettlementError> {
    expected_policy.validate()?;

    if let Some(receipt) = existing_receipt {
        receipt.validate_against_policy(expected_policy)?;
    }
    if let Some(intent) = fresh_intent {
        intent.validate_against_policy(expected_policy)?;
    }

    Ok(kernel::plan_settlement_for_deposit(
        terms,
        existing_receipt,
        fresh_intent,
    )?)
}

#[cfg(test)]
mod policy_binding_tests {
    use super::*;

    fn terms() -> CollateralDepositTerms {
        CollateralDepositTerms {
            deposit_id: "deposit:policy".into(),
            depositor_did: "did:mycelix:alice".into(),
            collateral_asset_id: "ETH".into(),
            collateral_amount: 10,
            quote_asset_id: SAP_ASSET_ID.into(),
        }
    }

    fn policy(provider: &str) -> CollateralSettlementAuthorityPolicy {
        CollateralSettlementAuthorityPolicy {
            policy_id: "settlement-policy".into(),
            policy_version: 1,
            settlement_protocol_version: COLLATERAL_SETTLEMENT_PROTOCOL_VERSION,
            quote_asset_id: SAP_ASSET_ID.into(),
            price_capability: PriceAttestationCapability {
                provider_id: provider.into(),
                method: "price".into(),
                protocol_version: PRICE_ATTESTATION_PROTOCOL_VERSION,
            },
            custody_capability: CustodyAttestationCapability {
                custodian_id: "did:mycelix:custodian".into(),
                method: "custody".into(),
                protocol_version: CUSTODY_ATTESTATION_PROTOCOL_VERSION,
            },
            price_freshness: SettlementFreshnessPolicy {
                policy_id: "price-current".into(),
                policy_version: 1,
                max_age_micros: 100,
            },
            custody_freshness: SettlementFreshnessPolicy {
                policy_id: "custody-current".into(),
                policy_version: 1,
                max_age_micros: 100,
            },
        }
    }

    fn intent(authority: &CollateralSettlementAuthorityPolicy) -> CollateralSettlementIntent {
        let price = PriceEvidenceEnvelope {
            base_asset_id: "ETH".into(),
            quote_asset_id: SAP_ASSET_ID.into(),
            capability: authority.price_capability.clone(),
            outcome: PriceEvidenceOutcome::Observed {
                rate: PriceRatio {
                    numerator: 2,
                    denominator: 1,
                },
                observation_reference: "price:1".into(),
                observed_at_micros: 10,
            },
        };
        let custody = CustodyEvidenceEnvelope {
            deposit_id: "deposit:policy".into(),
            depositor_did: "did:mycelix:alice".into(),
            collateral_asset_id: "ETH".into(),
            collateral_amount: 10,
            capability: authority.custody_capability.clone(),
            outcome: CustodyEvidenceOutcome::Confirmed {
                external_reference: "custody:1".into(),
                confirmed_at_micros: 10,
            },
        };
        derive_settlement_intent(terms(), authority, &price, &custody, 11)
            .expect("valid intent")
    }

    fn payment(intent: &CollateralSettlementIntent) -> PaymentCommitEvidence {
        PaymentCommitEvidence {
            idempotency_key: intent.basis.terms.deposit_id.clone(),
            member_did: intent.basis.terms.depositor_did.clone(),
            currency: SAP_ASSET_ID.into(),
            amount: intent.basis.sap_amount,
            payment_receipt_id: "payment:1".into(),
        }
    }

    #[test]
    fn self_declared_alternate_provider_is_not_authority() {
        let expected = policy("approved-price-provider");
        let alternate = policy("attacker-selected-provider");
        let alternate_intent = intent(&alternate);

        assert_eq!(
            alternate_intent.validate_against_policy(&expected),
            Err(PolicyBoundSettlementError::AuthorityPolicyMismatch)
        );
    }

    #[test]
    fn finalize_requires_exact_expected_policy() {
        let expected = policy("approved-price-provider");
        let alternate = policy("attacker-selected-provider");
        let alternate_intent = intent(&alternate);
        let commit = payment(&alternate_intent);

        assert_eq!(
            finalize_settlement_receipt(alternate_intent, &expected, commit, 12),
            Err(PolicyBoundSettlementError::AuthorityPolicyMismatch)
        );
    }

    #[test]
    fn retry_requires_existing_receipt_policy_to_match_consumer_policy() {
        let approved = policy("approved-price-provider");
        let alternate = policy("attacker-selected-provider");
        let alternate_intent = intent(&alternate);
        let receipt = kernel::finalize_settlement_receipt(
            alternate_intent.clone(),
            payment(&alternate_intent),
            12,
        )
        .expect("structurally valid alternate-policy receipt");

        assert_eq!(
            plan_settlement_for_deposit(&terms(), &approved, Some(&receipt), None),
            Err(PolicyBoundSettlementError::AuthorityPolicyMismatch)
        );
    }

    #[test]
    fn approved_policy_round_trip_can_finalize_and_retry() {
        let approved = policy("approved-price-provider");
        let approved_intent = intent(&approved);
        let receipt = finalize_settlement_receipt(
            approved_intent.clone(),
            &approved,
            payment(&approved_intent),
            12,
        )
        .expect("approved receipt");

        assert_eq!(
            plan_settlement_for_deposit(&terms(), &approved, Some(&receipt), None),
            Ok(SettlementExecutionPlan::AlreadySettled(receipt))
        );
    }
}
