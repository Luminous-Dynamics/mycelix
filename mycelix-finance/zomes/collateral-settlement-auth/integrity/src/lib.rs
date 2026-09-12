#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-011 collateral evidence authentication integrity zome.
//!
//! This zome defines immutable successful price/custody attestations. Their
//! authority comes from Finance DNA properties plus the actual Holochain Create
//! author, never from a provider-name string embedded in the entry.

use finance_collateral_auth::{
    CollateralSettlementAuthConfig, CustodyAttestationV1, PriceAttestationV1,
};
use hdi::prelude::*;
use mycelix_bridge_entry_types::did_for_author;

#[dna_properties]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FinanceCollateralAuthDnaProperties {
    pub collateral_settlement_auth: CollateralSettlementAuthConfig,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct PriceAttestationV1Entry {
    pub attestation: PriceAttestationV1,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CustodyAttestationV1Entry {
    pub attestation: CustodyAttestationV1,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    PriceAttestationV1(PriceAttestationV1Entry),
    CustodyAttestationV1(CustodyAttestationV1Entry),
}

/// FIN-SAFE-011 v1 deliberately has no global attestation index. Settlement
/// consumes exact action hashes and reloads them with Holochain validity
/// semantics. This avoids creating an index-poisoning or latest-value authority.
#[hdk_link_types]
pub enum LinkTypes {
    ReservedAttestationIndex,
}

pub fn load_collateral_auth_config() -> ExternResult<CollateralSettlementAuthConfig> {
    Ok(FinanceCollateralAuthDnaProperties::try_from_dna_properties()?
        .collateral_settlement_auth)
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    let config = load_collateral_auth_config()?;
    match config.validate() {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(error) => Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid FIN-SAFE-011 DNA collateral auth configuration: {error:?}"
        ))),
    }
}

fn validate_price_create(
    action: Create,
    entry: PriceAttestationV1Entry,
) -> ExternResult<ValidateCallbackResult> {
    let config = load_collateral_auth_config()?;
    let root = match config.require_enabled_root() {
        Ok(root) => root,
        Err(error) => {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Collateral price attestation is not authorized by DNA policy: {error:?}"
            )))
        }
    };
    let author_did = did_for_author(&action.author);
    match entry.attestation.to_evidence_from_valid_action(
        root,
        &author_did,
        action.timestamp.as_micros(),
    ) {
        Ok(_) => Ok(ValidateCallbackResult::Valid),
        Err(error) => Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid collateral price attestation: {error:?}"
        ))),
    }
}

fn validate_custody_create(
    action: Create,
    entry: CustodyAttestationV1Entry,
) -> ExternResult<ValidateCallbackResult> {
    let config = load_collateral_auth_config()?;
    let root = match config.require_enabled_root() {
        Ok(root) => root,
        Err(error) => {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Collateral custody attestation is not authorized by DNA policy: {error:?}"
            )))
        }
    };
    let author_did = did_for_author(&action.author);
    match entry.attestation.to_evidence_from_valid_action(
        root,
        &author_did,
        action.timestamp.as_micros(),
    ) {
        Ok(_) => Ok(ValidateCallbackResult::Valid),
        Err(error) => Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid collateral custody attestation: {error:?}"
        ))),
    }
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::PriceAttestationV1(entry) => validate_price_create(action, entry),
                EntryTypes::CustodyAttestationV1(entry) => validate_custody_create(action, entry),
            },
            OpEntry::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "Collateral evidence attestations are immutable".into(),
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::StoreRecord(store_record) => match store_record {
            OpRecord::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::PriceAttestationV1(entry) => validate_price_create(action, entry),
                EntryTypes::CustodyAttestationV1(entry) => validate_custody_create(action, entry),
            },
            OpRecord::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "Collateral evidence attestations are immutable".into(),
            )),
            OpRecord::CreateLink { .. } | OpRecord::DeleteLink { .. } => {
                Ok(ValidateCallbackResult::Invalid(
                    "FIN-SAFE-011 v1 does not authorize attestation links".into(),
                ))
            }
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "Collateral evidence attestations are immutable".into(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Collateral evidence attestations cannot be deleted".into(),
        )),
        FlatOp::RegisterCreateLink { .. } | FlatOp::RegisterDeleteLink { .. } => {
            Ok(ValidateCallbackResult::Invalid(
                "FIN-SAFE-011 v1 does not authorize attestation links".into(),
            ))
        }
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use finance_collateral_auth::{
        CollateralSettlementTrustRootV1, COLLATERAL_EVIDENCE_AUTH_PROTOCOL_VERSION,
        CUSTODY_ATTESTATION_V1_SCHEMA_VERSION, PRICE_ATTESTATION_V1_SCHEMA_VERSION,
    };
    use finance_collateral_settlement::{
        CollateralSettlementAuthorityPolicy, CustodyAttestationCapability,
        PriceAttestationCapability, PriceRatio, SettlementFreshnessPolicy,
        COLLATERAL_SETTLEMENT_PROTOCOL_VERSION, CUSTODY_ATTESTATION_PROTOCOL_VERSION,
        PRICE_ATTESTATION_PROTOCOL_VERSION, SAP_ASSET_ID,
    };

    fn action(author_byte: u8, timestamp_micros: i64) -> Create {
        Create {
            author: AgentPubKey::from_raw_36(vec![author_byte; 36]),
            timestamp: Timestamp::from_micros(timestamp_micros),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::CapClaim,
            entry_hash: EntryHash::from_raw_36(vec![0; 36]),
            weight: Default::default(),
        }
    }

    fn author_did(author_byte: u8) -> String {
        did_for_author(&AgentPubKey::from_raw_36(vec![author_byte; 36]))
    }

    fn root(price_author: u8, custody_author: u8) -> CollateralSettlementTrustRootV1 {
        let price_did = author_did(price_author);
        let custody_did = author_did(custody_author);
        CollateralSettlementTrustRootV1 {
            root_id: "test-root".into(),
            root_version: 1,
            authentication_protocol_version: COLLATERAL_EVIDENCE_AUTH_PROTOCOL_VERSION,
            settlement_policy: CollateralSettlementAuthorityPolicy {
                policy_id: "test-policy".into(),
                policy_version: 1,
                settlement_protocol_version: COLLATERAL_SETTLEMENT_PROTOCOL_VERSION,
                quote_asset_id: SAP_ASSET_ID.into(),
                price_capability: PriceAttestationCapability {
                    provider_id: price_did.clone(),
                    method: "holochain-price-attestation-v1".into(),
                    protocol_version: PRICE_ATTESTATION_PROTOCOL_VERSION,
                },
                custody_capability: CustodyAttestationCapability {
                    custodian_id: custody_did.clone(),
                    method: "holochain-custody-attestation-v1".into(),
                    protocol_version: CUSTODY_ATTESTATION_PROTOCOL_VERSION,
                },
                price_freshness: SettlementFreshnessPolicy {
                    policy_id: "price".into(),
                    policy_version: 1,
                    max_age_micros: 100,
                },
                custody_freshness: SettlementFreshnessPolicy {
                    policy_id: "custody".into(),
                    policy_version: 1,
                    max_age_micros: 100,
                },
            },
            price_authority_did: price_did,
            custody_authority_did: custody_did,
        }
    }

    fn price_entry(price_author: u8) -> PriceAttestationV1Entry {
        let r = root(price_author, 2);
        PriceAttestationV1Entry {
            attestation: PriceAttestationV1 {
                schema_version: PRICE_ATTESTATION_V1_SCHEMA_VERSION,
                base_asset_id: "ETH".into(),
                quote_asset_id: SAP_ASSET_ID.into(),
                capability: r.settlement_policy.price_capability,
                rate: PriceRatio {
                    numerator: 2,
                    denominator: 1,
                },
                observation_reference: "price:1".into(),
                observed_at_micros: 1_000,
            },
        }
    }

    fn custody_entry(custody_author: u8) -> CustodyAttestationV1Entry {
        let r = root(1, custody_author);
        CustodyAttestationV1Entry {
            attestation: CustodyAttestationV1 {
                schema_version: CUSTODY_ATTESTATION_V1_SCHEMA_VERSION,
                deposit_id: "deposit:v2:test".into(),
                depositor_did: author_did(3),
                collateral_asset_id: "ETH".into(),
                collateral_amount: 10,
                capability: r.settlement_policy.custody_capability,
                external_reference: "custody:1".into(),
                confirmed_at_micros: 1_000,
            },
        }
    }

    #[test]
    fn configured_price_author_is_valid() {
        let r = root(1, 2);
        let author = author_did(1);
        let result = price_entry(1).attestation.to_evidence_from_valid_action(
            &r,
            &author,
            action(1, 1_001).timestamp.as_micros(),
        );
        assert!(result.is_ok());
    }

    #[test]
    fn wrong_price_author_is_rejected() {
        let r = root(1, 2);
        let result = price_entry(1).attestation.to_evidence_from_valid_action(
            &r,
            &author_did(9),
            1_001,
        );
        assert!(result.is_err());
    }

    #[test]
    fn configured_custody_author_is_valid() {
        let r = root(1, 2);
        let result = custody_entry(2).attestation.to_evidence_from_valid_action(
            &r,
            &author_did(2),
            1_001,
        );
        assert!(result.is_ok());
    }
}
