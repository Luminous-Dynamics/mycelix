#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Evidence-gated collateral-backed SAP settlement.
//!
//! This crate separates deposit intent from monetary settlement. A client cannot
//! supply an authoritative price or settlement amount. Settlement can be derived
//! only when an exact expected price capability and an independent custody
//! capability both return current, bound evidence.
//!
//! This is a pure semantic kernel. It does not prove signatures, Holochain action
//! authorship, or external custody by itself. Integration code must obtain the
//! evidence envelopes and authority policy from trusted protocol boundaries, not
//! from an untrusted client request.

use serde::{Deserialize, Serialize};

pub const COLLATERAL_SETTLEMENT_PROTOCOL_VERSION: u16 = 1;
pub const PRICE_ATTESTATION_PROTOCOL_VERSION: u16 = 1;
pub const CUSTODY_ATTESTATION_PROTOCOL_VERSION: u16 = 1;
pub const SAP_ASSET_ID: &str = "SAP";
pub const MAX_ID_LEN: usize = 256;
pub const MAX_EXTERNAL_REFERENCE_LEN: usize = 512;

/// Immutable deposit terms. Creating these terms has no SAP balance effect.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollateralDepositTerms {
    pub deposit_id: String,
    pub depositor_did: String,
    pub collateral_asset_id: String,
    pub collateral_amount: u64,
    pub quote_asset_id: String,
}

impl CollateralDepositTerms {
    pub fn validate(&self) -> Result<(), CollateralSettlementError> {
        if !valid_id(&self.deposit_id)
            || !valid_id(&self.collateral_asset_id)
            || !valid_id(&self.quote_asset_id)
            || !valid_did(&self.depositor_did)
            || self.collateral_amount == 0
        {
            return Err(CollateralSettlementError::InvalidDepositTerms);
        }
        if self.quote_asset_id != SAP_ASSET_ID {
            return Err(CollateralSettlementError::UnsupportedQuoteAsset);
        }
        Ok(())
    }
}

/// Freshness policy selected by the trusted settlement integration.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SettlementFreshnessPolicy {
    pub policy_id: String,
    pub policy_version: u16,
    pub max_age_micros: i64,
}

impl SettlementFreshnessPolicy {
    pub fn validate(&self) -> Result<(), CollateralSettlementError> {
        if !valid_id(&self.policy_id)
            || self.policy_version == 0
            || self.max_age_micros <= 0
        {
            return Err(CollateralSettlementError::InvalidFreshnessPolicy);
        }
        Ok(())
    }
}

/// Exact price provider capability expected by the settlement authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PriceAttestationCapability {
    pub provider_id: String,
    pub method: String,
    pub protocol_version: u16,
}

impl PriceAttestationCapability {
    pub fn validate(&self) -> Result<(), CollateralSettlementError> {
        if !valid_id(&self.provider_id) || !valid_id(&self.method) {
            return Err(CollateralSettlementError::InvalidPriceCapability);
        }
        if self.protocol_version != PRICE_ATTESTATION_PROTOCOL_VERSION {
            return Err(CollateralSettlementError::UnsupportedPriceProtocol);
        }
        Ok(())
    }
}

/// Exact custody provider capability expected by the settlement authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CustodyAttestationCapability {
    pub custodian_id: String,
    pub method: String,
    pub protocol_version: u16,
}

impl CustodyAttestationCapability {
    pub fn validate(&self) -> Result<(), CollateralSettlementError> {
        if !valid_id(&self.custodian_id) || !valid_id(&self.method) {
            return Err(CollateralSettlementError::InvalidCustodyCapability);
        }
        if self.protocol_version != CUSTODY_ATTESTATION_PROTOCOL_VERSION {
            return Err(CollateralSettlementError::UnsupportedCustodyProtocol);
        }
        Ok(())
    }
}

/// Trusted settlement authority policy.
///
/// The caller of [`derive_settlement_intent`] supplies this as an *expected*
/// policy. Integration must source it from configuration/governance, never copy
/// provider identities or freshness limits from an untrusted deposit input.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollateralSettlementAuthorityPolicy {
    pub policy_id: String,
    pub policy_version: u16,
    pub settlement_protocol_version: u16,
    pub quote_asset_id: String,
    pub price_capability: PriceAttestationCapability,
    pub custody_capability: CustodyAttestationCapability,
    pub price_freshness: SettlementFreshnessPolicy,
    pub custody_freshness: SettlementFreshnessPolicy,
}

impl CollateralSettlementAuthorityPolicy {
    pub fn validate(&self) -> Result<(), CollateralSettlementError> {
        if !valid_id(&self.policy_id) || self.policy_version == 0 {
            return Err(CollateralSettlementError::InvalidAuthorityPolicy);
        }
        if self.settlement_protocol_version != COLLATERAL_SETTLEMENT_PROTOCOL_VERSION {
            return Err(CollateralSettlementError::UnsupportedSettlementProtocol);
        }
        if self.quote_asset_id != SAP_ASSET_ID {
            return Err(CollateralSettlementError::UnsupportedQuoteAsset);
        }
        self.price_capability.validate()?;
        self.custody_capability.validate()?;
        self.price_freshness.validate()?;
        self.custody_freshness.validate()?;
        Ok(())
    }
}

/// Positive rational price expressed as quote minor-units per collateral
/// minor-unit. Settlement uses deterministic integer floor division.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PriceRatio {
    pub numerator: u64,
    pub denominator: u64,
}

impl PriceRatio {
    pub fn validate(&self) -> Result<(), CollateralSettlementError> {
        if self.numerator == 0 || self.denominator == 0 {
            return Err(CollateralSettlementError::InvalidPriceRatio);
        }
        Ok(())
    }

    pub fn quote_amount_floor(
        &self,
        collateral_amount: u64,
    ) -> Result<u64, CollateralSettlementError> {
        self.validate()?;
        let product = (collateral_amount as u128)
            .checked_mul(self.numerator as u128)
            .ok_or(CollateralSettlementError::SettlementAmountOverflow)?;
        let amount = product / self.denominator as u128;
        if amount == 0 {
            return Err(CollateralSettlementError::ZeroSettlementAmount);
        }
        u64::try_from(amount).map_err(|_| CollateralSettlementError::SettlementAmountOverflow)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PriceEvidenceFailure {
    ProviderUnavailable,
    CapabilityMissing,
    UnsupportedPair,
    VersionMismatch,
    MalformedResponse,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PriceEvidenceOutcome {
    Observed {
        rate: PriceRatio,
        observation_reference: String,
        observed_at_micros: i64,
    },
    Unavailable {
        reason: PriceEvidenceFailure,
        attempted_at_micros: i64,
    },
}

/// Provider response for one exact price pair.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PriceEvidenceEnvelope {
    pub base_asset_id: String,
    pub quote_asset_id: String,
    pub capability: PriceAttestationCapability,
    pub outcome: PriceEvidenceOutcome,
}

impl PriceEvidenceEnvelope {
    pub fn validate_for(
        &self,
        terms: &CollateralDepositTerms,
        policy: &CollateralSettlementAuthorityPolicy,
    ) -> Result<(), CollateralSettlementError> {
        if self.base_asset_id != terms.collateral_asset_id
            || self.quote_asset_id != terms.quote_asset_id
            || self.quote_asset_id != policy.quote_asset_id
        {
            return Err(CollateralSettlementError::PriceSubjectMismatch);
        }
        if self.capability != policy.price_capability {
            return Err(CollateralSettlementError::PriceCapabilityMismatch);
        }
        self.capability.validate()?;
        if let PriceEvidenceOutcome::Observed {
            rate,
            observation_reference,
            ..
        } = &self.outcome
        {
            rate.validate()?;
            if observation_reference.is_empty()
                || observation_reference.len() > MAX_EXTERNAL_REFERENCE_LEN
            {
                return Err(CollateralSettlementError::InvalidPriceObservationReference);
            }
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CustodyEvidenceFailure {
    ProviderUnavailable,
    CapabilityMissing,
    DepositNotFound,
    AmountMismatch,
    NotFinal,
    VersionMismatch,
    MalformedResponse,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CustodyEvidenceOutcome {
    Confirmed {
        external_reference: String,
        confirmed_at_micros: i64,
    },
    Unavailable {
        reason: CustodyEvidenceFailure,
        attempted_at_micros: i64,
    },
}

/// Custody evidence binds the exact deposit, depositor, asset, and amount.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CustodyEvidenceEnvelope {
    pub deposit_id: String,
    pub depositor_did: String,
    pub collateral_asset_id: String,
    pub collateral_amount: u64,
    pub capability: CustodyAttestationCapability,
    pub outcome: CustodyEvidenceOutcome,
}

impl CustodyEvidenceEnvelope {
    pub fn validate_for(
        &self,
        terms: &CollateralDepositTerms,
        policy: &CollateralSettlementAuthorityPolicy,
    ) -> Result<(), CollateralSettlementError> {
        if self.deposit_id != terms.deposit_id
            || self.depositor_did != terms.depositor_did
            || self.collateral_asset_id != terms.collateral_asset_id
            || self.collateral_amount != terms.collateral_amount
        {
            return Err(CollateralSettlementError::CustodySubjectMismatch);
        }
        if self.capability != policy.custody_capability {
            return Err(CollateralSettlementError::CustodyCapabilityMismatch);
        }
        self.capability.validate()?;
        // Conservative baseline: the depositor cannot also be the attesting
        // custodian on the checked settlement path.
        if self.capability.custodian_id == terms.depositor_did {
            return Err(CollateralSettlementError::SelfCustodyAttestationNotAllowed);
        }
        if let CustodyEvidenceOutcome::Confirmed {
            external_reference,
            ..
        } = &self.outcome
        {
            if external_reference.is_empty()
                || external_reference.len() > MAX_EXTERNAL_REFERENCE_LEN
            {
                return Err(CollateralSettlementError::InvalidCustodyReference);
            }
        }
        Ok(())
    }
}

/// Deterministic settlement basis. It contains all economic/provenance fields
/// that must remain identical across retries. `authorized_at_micros` lives on the
/// outer intent and is intentionally excluded from retry equivalence.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollateralSettlementBasis {
    pub terms: CollateralDepositTerms,
    pub authority_policy: CollateralSettlementAuthorityPolicy,
    pub price_rate: PriceRatio,
    pub price_observation_reference: String,
    pub price_observed_at_micros: i64,
    pub custody_external_reference: String,
    pub custody_confirmed_at_micros: i64,
    pub sap_amount: u64,
}

impl CollateralSettlementBasis {
    pub fn validate(&self) -> Result<(), CollateralSettlementError> {
        self.terms.validate()?;
        self.authority_policy.validate()?;
        if self.terms.quote_asset_id != self.authority_policy.quote_asset_id {
            return Err(CollateralSettlementError::UnsupportedQuoteAsset);
        }
        if self.authority_policy.custody_capability.custodian_id == self.terms.depositor_did {
            return Err(CollateralSettlementError::SelfCustodyAttestationNotAllowed);
        }
        self.price_rate.validate()?;
        if self.price_observation_reference.is_empty()
            || self.price_observation_reference.len() > MAX_EXTERNAL_REFERENCE_LEN
        {
            return Err(CollateralSettlementError::InvalidPriceObservationReference);
        }
        if self.custody_external_reference.is_empty()
            || self.custody_external_reference.len() > MAX_EXTERNAL_REFERENCE_LEN
        {
            return Err(CollateralSettlementError::InvalidCustodyReference);
        }
        let expected = self
            .price_rate
            .quote_amount_floor(self.terms.collateral_amount)?;
        if expected != self.sap_amount {
            return Err(CollateralSettlementError::SettlementAmountMismatch);
        }
        Ok(())
    }
}

/// Evidence-ready settlement intent. Creating this object does not itself credit
/// SAP; integration must use an idempotent payments operation keyed by deposit ID.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollateralSettlementIntent {
    pub basis: CollateralSettlementBasis,
    pub authorized_at_micros: i64,
}

impl CollateralSettlementIntent {
    pub fn validate(&self) -> Result<(), CollateralSettlementError> {
        self.basis.validate()?;
        ensure_current(
            self.basis.price_observed_at_micros,
            self.authorized_at_micros,
            self.basis.authority_policy.price_freshness.max_age_micros,
            CollateralSettlementError::PriceEvidenceFromFuture,
            CollateralSettlementError::PriceEvidenceTooOld,
        )?;
        ensure_current(
            self.basis.custody_confirmed_at_micros,
            self.authorized_at_micros,
            self.basis.authority_policy.custody_freshness.max_age_micros,
            CollateralSettlementError::CustodyEvidenceFromFuture,
            CollateralSettlementError::CustodyEvidenceTooOld,
        )?;
        Ok(())
    }

    pub fn deposit_id(&self) -> &str {
        &self.basis.terms.deposit_id
    }

    pub fn same_settlement_basis(&self, other: &Self) -> bool {
        self.basis == other.basis
    }
}

/// Payment-layer commit proof required before the Finance settlement receipt can
/// be finalized.
///
/// The payments integration must implement `idempotency_key` as an exactly-once
/// key. For this protocol that key is exactly the immutable deposit ID.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PaymentCommitEvidence {
    pub idempotency_key: String,
    pub member_did: String,
    pub currency: String,
    pub amount: u64,
    pub payment_receipt_id: String,
}

impl PaymentCommitEvidence {
    pub fn validate_for(
        &self,
        intent: &CollateralSettlementIntent,
    ) -> Result<(), CollateralSettlementError> {
        if self.idempotency_key != intent.basis.terms.deposit_id
            || self.member_did != intent.basis.terms.depositor_did
            || self.currency != SAP_ASSET_ID
            || self.amount != intent.basis.sap_amount
        {
            return Err(CollateralSettlementError::PaymentCommitMismatch);
        }
        if !valid_id(&self.payment_receipt_id) {
            return Err(CollateralSettlementError::InvalidPaymentReceiptId);
        }
        Ok(())
    }
}

/// Final Finance-side receipt. The embedded payment commit is the recovery link
/// for the crash boundary between payment credit and Finance receipt persistence.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollateralSettlementReceipt {
    pub intent: CollateralSettlementIntent,
    pub payment_commit: PaymentCommitEvidence,
    pub settled_at_micros: i64,
}

impl CollateralSettlementReceipt {
    pub fn validate(&self) -> Result<(), CollateralSettlementError> {
        self.intent.validate()?;
        self.payment_commit.validate_for(&self.intent)?;
        if self.settled_at_micros < self.intent.authorized_at_micros {
            return Err(CollateralSettlementError::SettlementBeforeAuthorization);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SettlementExecutionPlan {
    /// No receipt exists; execute one idempotent payments credit using the
    /// deposit ID as idempotency key, then finalize a Finance receipt.
    Execute(CollateralSettlementIntent),
    /// A valid receipt already exists for this deposit. Return it; do not credit
    /// payments again and do not require evidence to still be fresh.
    AlreadySettled(CollateralSettlementReceipt),
}

/// Conservative deposit lifecycle. Monetary settlement begins only at `Settled`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollateralDepositState {
    Requested,
    EvidenceReady,
    Settled,
    Redeemed,
    Rejected,
    Failed,
}

impl CollateralDepositState {
    pub fn may_transition_to(self, next: Self) -> bool {
        matches!(
            (self, next),
            (Self::Requested, Self::EvidenceReady)
                | (Self::Requested, Self::Rejected)
                | (Self::Requested, Self::Failed)
                | (Self::EvidenceReady, Self::Settled)
                | (Self::EvidenceReady, Self::Rejected)
                | (Self::EvidenceReady, Self::Failed)
                | (Self::Settled, Self::Redeemed)
        )
    }

    /// Only the settled state has outstanding collateral-backed SAP issuance.
    pub fn has_outstanding_settled_sap(self) -> bool {
        matches!(self, Self::Settled)
    }

    /// Settled and redeemed history both require an immutable settlement receipt.
    pub fn requires_settlement_receipt(self) -> bool {
        matches!(self, Self::Settled | Self::Redeemed)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollateralSettlementError {
    InvalidDepositTerms,
    UnsupportedQuoteAsset,
    InvalidFreshnessPolicy,
    InvalidAuthorityPolicy,
    UnsupportedSettlementProtocol,
    InvalidPriceCapability,
    UnsupportedPriceProtocol,
    InvalidCustodyCapability,
    UnsupportedCustodyProtocol,
    PriceSubjectMismatch,
    PriceCapabilityMismatch,
    InvalidPriceRatio,
    InvalidPriceObservationReference,
    PriceUnavailable(PriceEvidenceFailure),
    PriceEvidenceFromFuture,
    PriceEvidenceTooOld,
    CustodySubjectMismatch,
    CustodyCapabilityMismatch,
    SelfCustodyAttestationNotAllowed,
    InvalidCustodyReference,
    CustodyUnavailable(CustodyEvidenceFailure),
    CustodyEvidenceFromFuture,
    CustodyEvidenceTooOld,
    TimestampArithmeticOverflow,
    SettlementAmountOverflow,
    ZeroSettlementAmount,
    SettlementAmountMismatch,
    PaymentCommitMismatch,
    InvalidPaymentReceiptId,
    SettlementBeforeAuthorization,
    MissingFreshSettlementIntent,
    SettlementIntentTermsMismatch,
    ConflictingExistingSettlement,
}

/// Derive a settlement intent only after both required evidence streams validate
/// against the trusted authority policy and are current at one explicit decision
/// time.
///
/// There is intentionally no caller-provided `oracle_rate` or `sap_amount`
/// parameter.
pub fn derive_settlement_intent(
    terms: CollateralDepositTerms,
    authority_policy: &CollateralSettlementAuthorityPolicy,
    price_evidence: &PriceEvidenceEnvelope,
    custody_evidence: &CustodyEvidenceEnvelope,
    evaluated_at_micros: i64,
) -> Result<CollateralSettlementIntent, CollateralSettlementError> {
    terms.validate()?;
    authority_policy.validate()?;

    if terms.quote_asset_id != authority_policy.quote_asset_id {
        return Err(CollateralSettlementError::UnsupportedQuoteAsset);
    }

    price_evidence.validate_for(&terms, authority_policy)?;
    custody_evidence.validate_for(&terms, authority_policy)?;

    let (price_rate, price_observation_reference, price_observed_at_micros) =
        match &price_evidence.outcome {
            PriceEvidenceOutcome::Observed {
                rate,
                observation_reference,
                observed_at_micros,
            } => (*rate, observation_reference.clone(), *observed_at_micros),
            PriceEvidenceOutcome::Unavailable { reason, .. } => {
                return Err(CollateralSettlementError::PriceUnavailable(*reason));
            }
        };
    ensure_current(
        price_observed_at_micros,
        evaluated_at_micros,
        authority_policy.price_freshness.max_age_micros,
        CollateralSettlementError::PriceEvidenceFromFuture,
        CollateralSettlementError::PriceEvidenceTooOld,
    )?;

    let (custody_external_reference, custody_confirmed_at_micros) =
        match &custody_evidence.outcome {
            CustodyEvidenceOutcome::Confirmed {
                external_reference,
                confirmed_at_micros,
            } => (external_reference.clone(), *confirmed_at_micros),
            CustodyEvidenceOutcome::Unavailable { reason, .. } => {
                return Err(CollateralSettlementError::CustodyUnavailable(*reason));
            }
        };
    ensure_current(
        custody_confirmed_at_micros,
        evaluated_at_micros,
        authority_policy.custody_freshness.max_age_micros,
        CollateralSettlementError::CustodyEvidenceFromFuture,
        CollateralSettlementError::CustodyEvidenceTooOld,
    )?;

    let sap_amount = price_rate.quote_amount_floor(terms.collateral_amount)?;

    let intent = CollateralSettlementIntent {
        basis: CollateralSettlementBasis {
            terms,
            authority_policy: authority_policy.clone(),
            price_rate,
            price_observation_reference,
            price_observed_at_micros,
            custody_external_reference,
            custody_confirmed_at_micros,
            sap_amount,
        },
        authorized_at_micros: evaluated_at_micros,
    };
    intent.validate()?;
    Ok(intent)
}

/// Finalize the Finance receipt only after the payments layer reports an exact
/// idempotent commit for the derived intent.
pub fn finalize_settlement_receipt(
    intent: CollateralSettlementIntent,
    payment_commit: PaymentCommitEvidence,
    settled_at_micros: i64,
) -> Result<CollateralSettlementReceipt, CollateralSettlementError> {
    intent.validate()?;
    payment_commit.validate_for(&intent)?;
    let receipt = CollateralSettlementReceipt {
        intent,
        payment_commit,
        settled_at_micros,
    };
    receipt.validate()?;
    Ok(receipt)
}

/// Resolve retry semantics *before* attempting a new payment credit.
///
/// If a valid receipt already exists for the deposit, return it without requiring
/// price/custody evidence to still be fresh. Otherwise a fresh derived intent is
/// mandatory.
pub fn plan_settlement_for_deposit(
    terms: &CollateralDepositTerms,
    existing_receipt: Option<&CollateralSettlementReceipt>,
    fresh_intent: Option<&CollateralSettlementIntent>,
) -> Result<SettlementExecutionPlan, CollateralSettlementError> {
    terms.validate()?;

    if let Some(receipt) = existing_receipt {
        receipt.validate()?;
        if receipt.intent.basis.terms != *terms {
            return Err(CollateralSettlementError::ConflictingExistingSettlement);
        }
        return Ok(SettlementExecutionPlan::AlreadySettled(receipt.clone()));
    }

    let intent = fresh_intent.ok_or(CollateralSettlementError::MissingFreshSettlementIntent)?;
    intent.validate()?;
    if intent.basis.terms != *terms {
        return Err(CollateralSettlementError::SettlementIntentTermsMismatch);
    }
    Ok(SettlementExecutionPlan::Execute(intent.clone()))
}

fn ensure_current(
    evidence_at_micros: i64,
    evaluated_at_micros: i64,
    max_age_micros: i64,
    from_future: CollateralSettlementError,
    too_old: CollateralSettlementError,
) -> Result<(), CollateralSettlementError> {
    if evaluated_at_micros < evidence_at_micros {
        return Err(from_future);
    }
    let age = evaluated_at_micros
        .checked_sub(evidence_at_micros)
        .ok_or(CollateralSettlementError::TimestampArithmeticOverflow)?;
    if age > max_age_micros {
        return Err(too_old);
    }
    Ok(())
}

fn valid_id(value: &str) -> bool {
    !value.is_empty() && value.len() <= MAX_ID_LEN
}

fn valid_did(value: &str) -> bool {
    value.starts_with("did:") && value.len() <= MAX_ID_LEN
}

#[cfg(test)]
mod tests {
    use super::*;

    fn terms() -> CollateralDepositTerms {
        CollateralDepositTerms {
            deposit_id: "deposit:1".into(),
            depositor_did: "did:mycelix:alice".into(),
            collateral_asset_id: "ETH".into(),
            collateral_amount: 3,
            quote_asset_id: SAP_ASSET_ID.into(),
        }
    }

    fn policy() -> CollateralSettlementAuthorityPolicy {
        CollateralSettlementAuthorityPolicy {
            policy_id: "collateral-settlement-main".into(),
            policy_version: 1,
            settlement_protocol_version: COLLATERAL_SETTLEMENT_PROTOCOL_VERSION,
            quote_asset_id: SAP_ASSET_ID.into(),
            price_capability: PriceAttestationCapability {
                provider_id: "price-oracle-consensus".into(),
                method: "get_consensus_price_v2".into(),
                protocol_version: PRICE_ATTESTATION_PROTOCOL_VERSION,
            },
            custody_capability: CustodyAttestationCapability {
                custodian_id: "did:mycelix:custodian".into(),
                method: "confirm_custody".into(),
                protocol_version: CUSTODY_ATTESTATION_PROTOCOL_VERSION,
            },
            price_freshness: SettlementFreshnessPolicy {
                policy_id: "price-5m".into(),
                policy_version: 1,
                max_age_micros: 300_000_000,
            },
            custody_freshness: SettlementFreshnessPolicy {
                policy_id: "custody-1h".into(),
                policy_version: 1,
                max_age_micros: 3_600_000_000,
            },
        }
    }

    fn price(rate: PriceRatio, at: i64) -> PriceEvidenceEnvelope {
        PriceEvidenceEnvelope {
            base_asset_id: "ETH".into(),
            quote_asset_id: SAP_ASSET_ID.into(),
            capability: policy().price_capability,
            outcome: PriceEvidenceOutcome::Observed {
                rate,
                observation_reference: "price-observation-1".into(),
                observed_at_micros: at,
            },
        }
    }

    fn custody(reference: &str, at: i64) -> CustodyEvidenceEnvelope {
        let t = terms();
        CustodyEvidenceEnvelope {
            deposit_id: t.deposit_id,
            depositor_did: t.depositor_did,
            collateral_asset_id: t.collateral_asset_id,
            collateral_amount: t.collateral_amount,
            capability: policy().custody_capability,
            outcome: CustodyEvidenceOutcome::Confirmed {
                external_reference: reference.into(),
                confirmed_at_micros: at,
            },
        }
    }

    fn intent() -> CollateralSettlementIntent {
        derive_settlement_intent(
            terms(),
            &policy(),
            &price(
                PriceRatio {
                    numerator: 5,
                    denominator: 2,
                },
                10,
            ),
            &custody("custody-proof-1", 11),
            12,
        )
        .expect("valid settlement intent")
    }

    fn payment_for(intent: &CollateralSettlementIntent) -> PaymentCommitEvidence {
        PaymentCommitEvidence {
            idempotency_key: intent.basis.terms.deposit_id.clone(),
            member_did: intent.basis.terms.depositor_did.clone(),
            currency: SAP_ASSET_ID.into(),
            amount: intent.basis.sap_amount,
            payment_receipt_id: "payment:receipt:1".into(),
        }
    }

    #[test]
    fn settlement_requires_both_bound_price_and_custody_evidence() {
        let intent = intent();
        assert_eq!(intent.basis.sap_amount, 7);
        assert_eq!(intent.basis.price_rate.numerator, 5);
        assert_eq!(intent.basis.price_rate.denominator, 2);
        assert_eq!(
            intent.basis.price_observation_reference,
            "price-observation-1"
        );
        assert_eq!(intent.basis.custody_external_reference, "custody-proof-1");
        assert_eq!(intent.validate(), Ok(()));
    }

    #[test]
    fn rational_settlement_uses_deterministic_floor_not_float_casting() {
        let rate = PriceRatio {
            numerator: 5,
            denominator: 2,
        };
        assert_eq!(rate.quote_amount_floor(3), Ok(7));
        assert_eq!(rate.quote_amount_floor(4), Ok(10));
    }

    #[test]
    fn unavailable_price_cannot_produce_a_mint_amount() {
        let mut evidence = price(
            PriceRatio {
                numerator: 1,
                denominator: 1,
            },
            10,
        );
        evidence.outcome = PriceEvidenceOutcome::Unavailable {
            reason: PriceEvidenceFailure::ProviderUnavailable,
            attempted_at_micros: 10,
        };
        assert_eq!(
            derive_settlement_intent(
                terms(),
                &policy(),
                &evidence,
                &custody("proof", 10),
                11,
            ),
            Err(CollateralSettlementError::PriceUnavailable(
                PriceEvidenceFailure::ProviderUnavailable
            ))
        );
    }

    #[test]
    fn unavailable_custody_cannot_settle() {
        let mut evidence = custody("proof", 10);
        evidence.outcome = CustodyEvidenceOutcome::Unavailable {
            reason: CustodyEvidenceFailure::NotFinal,
            attempted_at_micros: 10,
        };
        assert_eq!(
            derive_settlement_intent(
                terms(),
                &policy(),
                &price(
                    PriceRatio {
                        numerator: 1,
                        denominator: 1,
                    },
                    10,
                ),
                &evidence,
                11,
            ),
            Err(CollateralSettlementError::CustodyUnavailable(
                CustodyEvidenceFailure::NotFinal
            ))
        );
    }

    #[test]
    fn depositor_cannot_be_the_checked_custodian() {
        let mut p = policy();
        p.custody_capability.custodian_id = terms().depositor_did;
        let mut evidence = custody("proof", 10);
        evidence.capability = p.custody_capability.clone();
        assert_eq!(
            derive_settlement_intent(
                terms(),
                &p,
                &price(
                    PriceRatio {
                        numerator: 1,
                        denominator: 1,
                    },
                    10,
                ),
                &evidence,
                11,
            ),
            Err(CollateralSettlementError::SelfCustodyAttestationNotAllowed)
        );
    }

    #[test]
    fn wrong_price_pair_is_rejected() {
        let mut evidence = price(
            PriceRatio {
                numerator: 1,
                denominator: 1,
            },
            10,
        );
        evidence.base_asset_id = "USDC".into();
        assert_eq!(
            derive_settlement_intent(
                terms(),
                &policy(),
                &evidence,
                &custody("proof", 10),
                11,
            ),
            Err(CollateralSettlementError::PriceSubjectMismatch)
        );
    }

    #[test]
    fn wrong_custody_amount_is_rejected() {
        let mut evidence = custody("proof", 10);
        evidence.collateral_amount += 1;
        assert_eq!(
            derive_settlement_intent(
                terms(),
                &policy(),
                &price(
                    PriceRatio {
                        numerator: 1,
                        denominator: 1,
                    },
                    10,
                ),
                &evidence,
                11,
            ),
            Err(CollateralSettlementError::CustodySubjectMismatch)
        );
    }

    #[test]
    fn stale_price_is_rejected() {
        let mut p = policy();
        p.price_freshness.max_age_micros = 5;
        assert_eq!(
            derive_settlement_intent(
                terms(),
                &p,
                &price(
                    PriceRatio {
                        numerator: 1,
                        denominator: 1,
                    },
                    10,
                ),
                &custody("proof", 14),
                16,
            ),
            Err(CollateralSettlementError::PriceEvidenceTooOld)
        );
    }

    #[test]
    fn stale_custody_is_rejected() {
        let mut p = policy();
        p.custody_freshness.max_age_micros = 5;
        assert_eq!(
            derive_settlement_intent(
                terms(),
                &p,
                &price(
                    PriceRatio {
                        numerator: 1,
                        denominator: 1,
                    },
                    14,
                ),
                &custody("proof", 10),
                16,
            ),
            Err(CollateralSettlementError::CustodyEvidenceTooOld)
        );
    }

    #[test]
    fn future_evidence_is_rejected() {
        assert_eq!(
            derive_settlement_intent(
                terms(),
                &policy(),
                &price(
                    PriceRatio {
                        numerator: 1,
                        denominator: 1,
                    },
                    20,
                ),
                &custody("proof", 10),
                12,
            ),
            Err(CollateralSettlementError::PriceEvidenceFromFuture)
        );
    }

    #[test]
    fn malformed_or_zero_price_ratio_is_rejected() {
        assert_eq!(
            PriceRatio {
                numerator: 1,
                denominator: 0,
            }
            .quote_amount_floor(10),
            Err(CollateralSettlementError::InvalidPriceRatio)
        );
        assert_eq!(
            PriceRatio {
                numerator: 0,
                denominator: 1,
            }
            .quote_amount_floor(10),
            Err(CollateralSettlementError::InvalidPriceRatio)
        );
    }

    #[test]
    fn settlement_amount_overflow_is_rejected() {
        let mut t = terms();
        t.collateral_amount = u64::MAX;
        let mut c = custody("proof", 10);
        c.collateral_amount = u64::MAX;
        let result = derive_settlement_intent(
            t,
            &policy(),
            &price(
                PriceRatio {
                    numerator: u64::MAX,
                    denominator: 1,
                },
                10,
            ),
            &c,
            11,
        );
        assert_eq!(
            result,
            Err(CollateralSettlementError::SettlementAmountOverflow)
        );
    }

    #[test]
    fn settlement_rounding_to_zero_is_rejected() {
        let result = derive_settlement_intent(
            terms(),
            &policy(),
            &price(
                PriceRatio {
                    numerator: 1,
                    denominator: 100,
                },
                10,
            ),
            &custody("proof", 10),
            11,
        );
        assert_eq!(result, Err(CollateralSettlementError::ZeroSettlementAmount));
    }

    #[test]
    fn tampered_derived_amount_is_rejected_by_intent_validation() {
        let mut intent = intent();
        intent.basis.sap_amount += 1;
        assert_eq!(
            intent.validate(),
            Err(CollateralSettlementError::SettlementAmountMismatch)
        );
    }

    #[test]
    fn exact_payment_commit_binding_is_required() {
        let intent = intent();
        let mut payment = payment_for(&intent);
        payment.amount += 1;
        assert_eq!(
            finalize_settlement_receipt(intent, payment, 13),
            Err(CollateralSettlementError::PaymentCommitMismatch)
        );
    }

    #[test]
    fn valid_payment_commit_finalizes_receipt() {
        let intent = intent();
        let payment = payment_for(&intent);
        let receipt = finalize_settlement_receipt(intent.clone(), payment.clone(), 13)
            .expect("valid receipt");
        assert_eq!(receipt.intent, intent);
        assert_eq!(receipt.payment_commit, payment);
        assert_eq!(receipt.validate(), Ok(()));
    }

    #[test]
    fn retry_returns_existing_receipt_without_new_evidence_or_credit() {
        let intent = intent();
        let receipt =
            finalize_settlement_receipt(intent.clone(), payment_for(&intent), 13)
                .expect("valid receipt");

        assert_eq!(
            plan_settlement_for_deposit(&terms(), Some(&receipt), None),
            Ok(SettlementExecutionPlan::AlreadySettled(receipt))
        );
    }

    #[test]
    fn new_settlement_requires_fresh_intent_when_no_receipt_exists() {
        assert_eq!(
            plan_settlement_for_deposit(&terms(), None, None),
            Err(CollateralSettlementError::MissingFreshSettlementIntent)
        );
        let intent = intent();
        assert_eq!(
            plan_settlement_for_deposit(&terms(), None, Some(&intent)),
            Ok(SettlementExecutionPlan::Execute(intent))
        );
    }

    #[test]
    fn existing_receipt_for_different_terms_is_a_conflict() {
        let intent = intent();
        let receipt =
            finalize_settlement_receipt(intent.clone(), payment_for(&intent), 13)
                .expect("valid receipt");
        let mut other = terms();
        other.collateral_amount += 1;
        assert_eq!(
            plan_settlement_for_deposit(&other, Some(&receipt), None),
            Err(CollateralSettlementError::ConflictingExistingSettlement)
        );
    }

    #[test]
    fn unsupported_protocol_versions_fail_closed() {
        let mut p = policy();
        p.settlement_protocol_version += 1;
        assert_eq!(
            derive_settlement_intent(
                terms(),
                &p,
                &price(
                    PriceRatio {
                        numerator: 1,
                        denominator: 1,
                    },
                    10,
                ),
                &custody("proof", 10),
                11,
            ),
            Err(CollateralSettlementError::UnsupportedSettlementProtocol)
        );
    }

    #[test]
    fn timestamp_age_overflow_fails_closed() {
        let mut p = policy();
        p.price_freshness.max_age_micros = i64::MAX;
        let result = derive_settlement_intent(
            terms(),
            &p,
            &price(
                PriceRatio {
                    numerator: 1,
                    denominator: 1,
                },
                i64::MIN,
            ),
            &custody("proof", i64::MAX),
            i64::MAX,
        );
        assert_eq!(
            result,
            Err(CollateralSettlementError::TimestampArithmeticOverflow)
        );
    }

    #[test]
    fn requested_and_evidence_ready_states_have_no_outstanding_settled_sap() {
        assert!(!CollateralDepositState::Requested.has_outstanding_settled_sap());
        assert!(!CollateralDepositState::EvidenceReady.has_outstanding_settled_sap());
        assert!(CollateralDepositState::Settled.has_outstanding_settled_sap());
        assert!(!CollateralDepositState::Redeemed.has_outstanding_settled_sap());
        assert!(CollateralDepositState::Redeemed.requires_settlement_receipt());
    }

    #[test]
    fn lifecycle_disallows_settle_before_evidence_ready_and_double_settle() {
        assert!(!CollateralDepositState::Requested
            .may_transition_to(CollateralDepositState::Settled));
        assert!(CollateralDepositState::Requested
            .may_transition_to(CollateralDepositState::EvidenceReady));
        assert!(CollateralDepositState::EvidenceReady
            .may_transition_to(CollateralDepositState::Settled));
        assert!(!CollateralDepositState::Settled
            .may_transition_to(CollateralDepositState::Settled));
        assert!(CollateralDepositState::Settled
            .may_transition_to(CollateralDepositState::Redeemed));
    }

    #[test]
    fn serialization_round_trip_preserves_exact_settlement_basis() {
        let intent = intent();
        let encoded = serde_json::to_string(&intent).expect("serialize");
        let decoded: CollateralSettlementIntent =
            serde_json::from_str(&encoded).expect("deserialize");
        assert_eq!(decoded, intent);
        assert_eq!(decoded.validate(), Ok(()));
    }
}
