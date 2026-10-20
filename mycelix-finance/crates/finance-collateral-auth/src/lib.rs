#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-011 trusted collateral-settlement policy and evidence-authentication semantics.
//!
//! This crate is intentionally HDI-free. Holochain integration supplies action
//! author identity and action timestamp after loading an exact valid Create
//! action. These helpers then prove that the attestation fields match the exact
//! DNA-rooted policy before they are converted into FIN-SAFE-006 evidence.

use finance_collateral_settlement::{
    CollateralDepositTerms, CollateralSettlementAuthorityPolicy, CustodyEvidenceEnvelope,
    CustodyEvidenceOutcome, PriceEvidenceEnvelope, PriceEvidenceOutcome, PriceRatio,
    MAX_EXTERNAL_REFERENCE_LEN, MAX_ID_LEN,
};
use serde::{Deserialize, Serialize};

pub const COLLATERAL_EVIDENCE_AUTH_PROTOCOL_VERSION: u16 = 1;
pub const PRICE_ATTESTATION_V1_SCHEMA_VERSION: u16 = 1;
pub const CUSTODY_ATTESTATION_V1_SCHEMA_VERSION: u16 = 1;
pub const MAX_DID_LEN: usize = 256;
pub const MAX_TRUST_ROOT_ID_LEN: usize = 256;

/// DNA-level switch for collateral issuance authority.
///
/// `enabled = false, root = None` is the canonical fail-closed default. Enabling
/// the feature requires an explicit fully valid trust root in the DNA properties.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollateralSettlementAuthConfig {
    pub enabled: bool,
    pub root: Option<CollateralSettlementTrustRootV1>,
}

impl CollateralSettlementAuthConfig {
    pub fn disabled() -> Self {
        Self {
            enabled: false,
            root: None,
        }
    }

    pub fn validate(&self) -> Result<(), CollateralEvidenceAuthError> {
        match (self.enabled, self.root.as_ref()) {
            (false, None) => Ok(()),
            (false, Some(_)) => Err(CollateralEvidenceAuthError::AmbiguousDisabledConfiguration),
            (true, None) => Err(CollateralEvidenceAuthError::MissingTrustRoot),
            (true, Some(root)) => root.validate(),
        }
    }

    pub fn require_enabled_root(
        &self,
    ) -> Result<&CollateralSettlementTrustRootV1, CollateralEvidenceAuthError> {
        self.validate()?;
        if !self.enabled {
            return Err(CollateralEvidenceAuthError::CollateralIssuanceDisabled);
        }
        self.root
            .as_ref()
            .ok_or(CollateralEvidenceAuthError::MissingTrustRoot)
    }
}

/// Immutable settlement trust root intended to be stored in DNA properties.
///
/// V1 supports Holochain-native action-author authentication only. External
/// signatures can be added later under a distinct protocol version rather than
/// overloading a provider-name string with authenticity semantics.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollateralSettlementTrustRootV1 {
    pub root_id: String,
    pub root_version: u16,
    pub authentication_protocol_version: u16,
    pub settlement_policy: CollateralSettlementAuthorityPolicy,
    pub price_authority_did: String,
    pub custody_authority_did: String,
}

impl CollateralSettlementTrustRootV1 {
    pub fn validate(&self) -> Result<(), CollateralEvidenceAuthError> {
        if self.root_id.is_empty() || self.root_id.len() > MAX_TRUST_ROOT_ID_LEN {
            return Err(CollateralEvidenceAuthError::InvalidTrustRootId);
        }
        if self.root_version == 0 {
            return Err(CollateralEvidenceAuthError::InvalidTrustRootVersion);
        }
        if self.authentication_protocol_version != COLLATERAL_EVIDENCE_AUTH_PROTOCOL_VERSION {
            return Err(CollateralEvidenceAuthError::UnsupportedAuthenticationProtocol);
        }
        self.settlement_policy
            .validate()
            .map_err(CollateralEvidenceAuthError::SettlementPolicy)?;
        validate_did(&self.price_authority_did)
            .map_err(|_| CollateralEvidenceAuthError::InvalidPriceAuthorityDid)?;
        validate_did(&self.custody_authority_did)
            .map_err(|_| CollateralEvidenceAuthError::InvalidCustodyAuthorityDid)?;
        if self.price_authority_did == self.custody_authority_did {
            return Err(CollateralEvidenceAuthError::PriceAndCustodyAuthorityMustDiffer);
        }
        if self.settlement_policy.price_capability.provider_id != self.price_authority_did {
            return Err(CollateralEvidenceAuthError::PriceAuthorityCapabilityMismatch);
        }
        if self.settlement_policy.custody_capability.custodian_id != self.custody_authority_did {
            return Err(CollateralEvidenceAuthError::CustodyAuthorityCapabilityMismatch);
        }
        Ok(())
    }
}

/// Immutable successful price attestation payload.
///
/// This value is not authoritative by itself. The Holochain adapter must obtain
/// `action_author_did` and `action_timestamp_micros` from an exact valid Create
/// action before calling [`PriceAttestationV1::to_evidence_from_valid_action`].
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PriceAttestationV1 {
    pub schema_version: u16,
    pub base_asset_id: String,
    pub quote_asset_id: String,
    pub capability: finance_collateral_settlement::PriceAttestationCapability,
    pub rate: PriceRatio,
    pub observation_reference: String,
    pub observed_at_micros: i64,
}

impl PriceAttestationV1 {
    pub fn to_evidence_from_valid_action(
        &self,
        root: &CollateralSettlementTrustRootV1,
        action_author_did: &str,
        action_timestamp_micros: i64,
    ) -> Result<PriceEvidenceEnvelope, CollateralEvidenceAuthError> {
        root.validate()?;
        if self.schema_version != PRICE_ATTESTATION_V1_SCHEMA_VERSION {
            return Err(CollateralEvidenceAuthError::UnsupportedPriceAttestationSchema);
        }
        if action_author_did != root.price_authority_did {
            return Err(CollateralEvidenceAuthError::WrongPriceActionAuthor);
        }
        if self.capability != root.settlement_policy.price_capability {
            return Err(CollateralEvidenceAuthError::PriceCapabilityMismatch);
        }
        if !valid_id(&self.base_asset_id)
            || !valid_id(&self.quote_asset_id)
            || self.quote_asset_id != root.settlement_policy.quote_asset_id
        {
            return Err(CollateralEvidenceAuthError::InvalidPriceSubject);
        }
        self.rate
            .validate()
            .map_err(CollateralEvidenceAuthError::SettlementPolicy)?;
        if self.observation_reference.is_empty()
            || self.observation_reference.len() > MAX_EXTERNAL_REFERENCE_LEN
        {
            return Err(CollateralEvidenceAuthError::InvalidPriceObservationReference);
        }
        ensure_fresh_at_publication(
            self.observed_at_micros,
            action_timestamp_micros,
            root.settlement_policy.price_freshness.max_age_micros,
            CollateralEvidenceAuthError::PriceObservedAfterPublication,
            CollateralEvidenceAuthError::PriceStaleAtPublication,
        )?;

        Ok(PriceEvidenceEnvelope {
            base_asset_id: self.base_asset_id.clone(),
            quote_asset_id: self.quote_asset_id.clone(),
            capability: self.capability.clone(),
            outcome: PriceEvidenceOutcome::Observed {
                rate: self.rate,
                observation_reference: self.observation_reference.clone(),
                observed_at_micros: self.observed_at_micros,
            },
        })
    }
}

/// Immutable successful custody/finality attestation payload.
///
/// As with price evidence, this is authoritative only when projected from an
/// exact valid Create action whose author is supplied by the Holochain adapter.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CustodyAttestationV1 {
    pub schema_version: u16,
    pub deposit_id: String,
    pub depositor_did: String,
    pub collateral_asset_id: String,
    pub collateral_amount: u64,
    pub capability: finance_collateral_settlement::CustodyAttestationCapability,
    pub external_reference: String,
    pub confirmed_at_micros: i64,
}

impl CustodyAttestationV1 {
    pub fn to_evidence_from_valid_action(
        &self,
        root: &CollateralSettlementTrustRootV1,
        action_author_did: &str,
        action_timestamp_micros: i64,
    ) -> Result<CustodyEvidenceEnvelope, CollateralEvidenceAuthError> {
        root.validate()?;
        if self.schema_version != CUSTODY_ATTESTATION_V1_SCHEMA_VERSION {
            return Err(CollateralEvidenceAuthError::UnsupportedCustodyAttestationSchema);
        }
        if action_author_did != root.custody_authority_did {
            return Err(CollateralEvidenceAuthError::WrongCustodyActionAuthor);
        }
        if self.capability != root.settlement_policy.custody_capability {
            return Err(CollateralEvidenceAuthError::CustodyCapabilityMismatch);
        }
        if action_author_did == self.depositor_did {
            return Err(CollateralEvidenceAuthError::SelfCustodyAttestationNotAllowed);
        }
        if self.external_reference.is_empty()
            || self.external_reference.len() > MAX_EXTERNAL_REFERENCE_LEN
        {
            return Err(CollateralEvidenceAuthError::InvalidCustodyReference);
        }

        let terms = CollateralDepositTerms {
            deposit_id: self.deposit_id.clone(),
            depositor_did: self.depositor_did.clone(),
            collateral_asset_id: self.collateral_asset_id.clone(),
            collateral_amount: self.collateral_amount,
            quote_asset_id: root.settlement_policy.quote_asset_id.clone(),
        };
        terms
            .validate()
            .map_err(CollateralEvidenceAuthError::SettlementPolicy)?;

        ensure_fresh_at_publication(
            self.confirmed_at_micros,
            action_timestamp_micros,
            root.settlement_policy.custody_freshness.max_age_micros,
            CollateralEvidenceAuthError::CustodyConfirmedAfterPublication,
            CollateralEvidenceAuthError::CustodyStaleAtPublication,
        )?;

        let envelope = CustodyEvidenceEnvelope {
            deposit_id: self.deposit_id.clone(),
            depositor_did: self.depositor_did.clone(),
            collateral_asset_id: self.collateral_asset_id.clone(),
            collateral_amount: self.collateral_amount,
            capability: self.capability.clone(),
            outcome: CustodyEvidenceOutcome::Confirmed {
                external_reference: self.external_reference.clone(),
                confirmed_at_micros: self.confirmed_at_micros,
            },
        };
        envelope
            .validate_for(&terms, &root.settlement_policy)
            .map_err(CollateralEvidenceAuthError::SettlementPolicy)?;
        Ok(envelope)
    }
}

fn ensure_fresh_at_publication(
    evidence_timestamp_micros: i64,
    action_timestamp_micros: i64,
    max_age_micros: i64,
    future_error: CollateralEvidenceAuthError,
    stale_error: CollateralEvidenceAuthError,
) -> Result<(), CollateralEvidenceAuthError> {
    if max_age_micros <= 0 {
        return Err(CollateralEvidenceAuthError::InvalidFreshnessPolicy);
    }
    if evidence_timestamp_micros > action_timestamp_micros {
        return Err(future_error);
    }
    let age = action_timestamp_micros as i128 - evidence_timestamp_micros as i128;
    if age > max_age_micros as i128 {
        return Err(stale_error);
    }
    Ok(())
}

fn valid_id(value: &str) -> bool {
    !value.is_empty() && value.len() <= MAX_ID_LEN
}

fn validate_did(value: &str) -> Result<(), ()> {
    if value.starts_with("did:mycelix:") && value.len() <= MAX_DID_LEN {
        Ok(())
    } else {
        Err(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollateralEvidenceAuthError {
    AmbiguousDisabledConfiguration,
    MissingTrustRoot,
    CollateralIssuanceDisabled,
    InvalidTrustRootId,
    InvalidTrustRootVersion,
    UnsupportedAuthenticationProtocol,
    InvalidPriceAuthorityDid,
    InvalidCustodyAuthorityDid,
    PriceAndCustodyAuthorityMustDiffer,
    PriceAuthorityCapabilityMismatch,
    CustodyAuthorityCapabilityMismatch,
    UnsupportedPriceAttestationSchema,
    WrongPriceActionAuthor,
    PriceCapabilityMismatch,
    InvalidPriceSubject,
    InvalidPriceObservationReference,
    PriceObservedAfterPublication,
    PriceStaleAtPublication,
    UnsupportedCustodyAttestationSchema,
    WrongCustodyActionAuthor,
    CustodyCapabilityMismatch,
    SelfCustodyAttestationNotAllowed,
    InvalidCustodyReference,
    CustodyConfirmedAfterPublication,
    CustodyStaleAtPublication,
    InvalidFreshnessPolicy,
    SettlementPolicy(finance_collateral_settlement::CollateralSettlementError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use finance_collateral_settlement::{
        CollateralSettlementAuthorityPolicy, CustodyAttestationCapability,
        PriceAttestationCapability, SettlementFreshnessPolicy,
        COLLATERAL_SETTLEMENT_PROTOCOL_VERSION, CUSTODY_ATTESTATION_PROTOCOL_VERSION,
        PRICE_ATTESTATION_PROTOCOL_VERSION, SAP_ASSET_ID,
    };

    fn root() -> CollateralSettlementTrustRootV1 {
        CollateralSettlementTrustRootV1 {
            root_id: "finance-collateral-root".into(),
            root_version: 1,
            authentication_protocol_version: COLLATERAL_EVIDENCE_AUTH_PROTOCOL_VERSION,
            settlement_policy: CollateralSettlementAuthorityPolicy {
                policy_id: "finance-collateral-settlement".into(),
                policy_version: 1,
                settlement_protocol_version: COLLATERAL_SETTLEMENT_PROTOCOL_VERSION,
                quote_asset_id: SAP_ASSET_ID.into(),
                price_capability: PriceAttestationCapability {
                    provider_id: "did:mycelix:price-authority".into(),
                    method: "holochain-price-attestation-v1".into(),
                    protocol_version: PRICE_ATTESTATION_PROTOCOL_VERSION,
                },
                custody_capability: CustodyAttestationCapability {
                    custodian_id: "did:mycelix:custody-authority".into(),
                    method: "holochain-custody-attestation-v1".into(),
                    protocol_version: CUSTODY_ATTESTATION_PROTOCOL_VERSION,
                },
                price_freshness: SettlementFreshnessPolicy {
                    policy_id: "price-5m".into(),
                    policy_version: 1,
                    max_age_micros: 300_000_000,
                },
                custody_freshness: SettlementFreshnessPolicy {
                    policy_id: "custody-5m".into(),
                    policy_version: 1,
                    max_age_micros: 300_000_000,
                },
            },
            price_authority_did: "did:mycelix:price-authority".into(),
            custody_authority_did: "did:mycelix:custody-authority".into(),
        }
    }

    fn price() -> PriceAttestationV1 {
        let r = root();
        PriceAttestationV1 {
            schema_version: PRICE_ATTESTATION_V1_SCHEMA_VERSION,
            base_asset_id: "ETH".into(),
            quote_asset_id: SAP_ASSET_ID.into(),
            capability: r.settlement_policy.price_capability,
            rate: PriceRatio {
                numerator: 2,
                denominator: 1,
            },
            observation_reference: "price-observation-1".into(),
            observed_at_micros: 1_000,
        }
    }

    fn custody() -> CustodyAttestationV1 {
        let r = root();
        CustodyAttestationV1 {
            schema_version: CUSTODY_ATTESTATION_V1_SCHEMA_VERSION,
            deposit_id: "deposit:v2:abc".into(),
            depositor_did: "did:mycelix:alice".into(),
            collateral_asset_id: "ETH".into(),
            collateral_amount: 10,
            capability: r.settlement_policy.custody_capability,
            external_reference: "custody-finality-1".into(),
            confirmed_at_micros: 1_000,
        }
    }

    #[test]
    fn disabled_without_root_is_valid_but_cannot_authorize() {
        let config = CollateralSettlementAuthConfig::disabled();
        assert_eq!(config.validate(), Ok(()));
        assert_eq!(
            config.require_enabled_root(),
            Err(CollateralEvidenceAuthError::CollateralIssuanceDisabled)
        );
    }

    #[test]
    fn enabled_without_root_fails_closed() {
        let config = CollateralSettlementAuthConfig {
            enabled: true,
            root: None,
        };
        assert_eq!(
            config.validate(),
            Err(CollateralEvidenceAuthError::MissingTrustRoot)
        );
    }

    #[test]
    fn price_and_custody_must_be_independent_authorities() {
        let mut r = root();
        r.custody_authority_did = r.price_authority_did.clone();
        r.settlement_policy.custody_capability.custodian_id = r.price_authority_did.clone();
        assert_eq!(
            r.validate(),
            Err(CollateralEvidenceAuthError::PriceAndCustodyAuthorityMustDiffer)
        );
    }

    #[test]
    fn exact_price_author_action_derives_evidence() {
        let r = root();
        let envelope = price()
            .to_evidence_from_valid_action(&r, &r.price_authority_did, 1_001)
            .expect("verified price action");
        assert_eq!(envelope.capability, r.settlement_policy.price_capability);
    }

    #[test]
    fn matching_provider_string_from_wrong_author_is_rejected() {
        let r = root();
        assert_eq!(
            price().to_evidence_from_valid_action(&r, "did:mycelix:attacker", 1_001),
            Err(CollateralEvidenceAuthError::WrongPriceActionAuthor)
        );
    }

    #[test]
    fn stale_or_future_price_is_rejected_at_publication() {
        let r = root();
        let mut p = price();
        p.observed_at_micros = 1_002;
        assert_eq!(
            p.to_evidence_from_valid_action(&r, &r.price_authority_did, 1_001),
            Err(CollateralEvidenceAuthError::PriceObservedAfterPublication)
        );

        p.observed_at_micros = 1;
        assert_eq!(
            p.to_evidence_from_valid_action(
                &r,
                &r.price_authority_did,
                1 + r.settlement_policy.price_freshness.max_age_micros + 1,
            ),
            Err(CollateralEvidenceAuthError::PriceStaleAtPublication)
        );
    }

    #[test]
    fn exact_custody_author_action_derives_evidence() {
        let r = root();
        let envelope = custody()
            .to_evidence_from_valid_action(&r, &r.custody_authority_did, 1_001)
            .expect("verified custody action");
        assert_eq!(envelope.capability, r.settlement_policy.custody_capability);
    }

    #[test]
    fn depositor_cannot_be_the_custody_authority() {
        let mut c = custody();
        c.depositor_did = "did:mycelix:custody-authority".into();
        let r = root();
        assert_eq!(
            c.to_evidence_from_valid_action(&r, &r.custody_authority_did, 1_001),
            Err(CollateralEvidenceAuthError::SelfCustodyAttestationNotAllowed)
        );
    }

    #[test]
    fn capability_drift_fails_closed() {
        let r = root();
        let mut p = price();
        p.capability.method = "other-method".into();
        assert_eq!(
            p.to_evidence_from_valid_action(&r, &r.price_authority_did, 1_001),
            Err(CollateralEvidenceAuthError::PriceCapabilityMismatch)
        );
    }

    #[test]
    fn trust_root_round_trip_preserves_policy() {
        let r = root();
        let json = serde_json::to_string(&r).expect("serialize root");
        let decoded: CollateralSettlementTrustRootV1 =
            serde_json::from_str(&json).expect("deserialize root");
        assert_eq!(decoded, r);
        assert_eq!(decoded.validate(), Ok(()));
    }
}
