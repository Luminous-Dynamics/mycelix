#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! ABI-neutral serialized contracts shared by Finance V2 Holochain zomes.
//!
//! This crate deliberately declares no EntryTypes, LinkTypes, callbacks,
//! extern functions, or zome authority. Zomes own authority; this crate
//! owns only canonical payload serialization and shared property decoding.

use finance_collateral_auth::{
    CollateralSettlementAuthConfig, CustodyAttestationV1, PriceAttestationV1,
};
use finance_collateral_deposit::{COLLATERAL_DEPOSIT_NONCE_BYTES, CollateralDepositRequestV2};
use finance_collateral_issuance_persistence::{
    CollateralSapIssuanceReceiptRecordV2, CollateralSapMintAuthorizationRecordV2,
    CollateralSapMintRecordV2Compact,
};
use finance_sap_account_v2::{SapAccountOpenedV2, SapAccountV2Config, SapCollateralClaimV2};
use finance_sap_transfer_v2::{SapTransferSpendRecordV2, SapTransferV2Config};
use hdi::prelude::*;

pub const MAX_CREATE_TIMESTAMP_SKEW_MICROS: i64 = 5_000_000;

#[dna_properties]
#[derive(Clone)]
pub struct FinanceCollateralAuthDnaProperties {
    pub collateral_settlement_auth: CollateralSettlementAuthConfig,
}

pub fn load_collateral_auth_config() -> ExternResult<CollateralSettlementAuthConfig> {
    Ok(FinanceCollateralAuthDnaProperties::try_from_dna_properties()?.collateral_settlement_auth)
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CollateralDepositRequestV2Entry {
    pub schema_version: u16,
    pub deposit_id: String,
    pub depositor_did: String,
    pub collateral_asset_id: String,
    pub collateral_amount: u64,
    pub quote_asset_id: String,
    pub request_nonce: Vec<u8>,
    pub created_at_micros: i64,
}

impl CollateralDepositRequestV2Entry {
    pub fn to_model(&self) -> Result<CollateralDepositRequestV2, String> {
        let request_nonce: [u8; COLLATERAL_DEPOSIT_NONCE_BYTES] =
            self.request_nonce.as_slice().try_into().map_err(|_| {
                format!(
                    "request_nonce must contain exactly {} bytes",
                    COLLATERAL_DEPOSIT_NONCE_BYTES
                )
            })?;
        Ok(CollateralDepositRequestV2 {
            schema_version: self.schema_version,
            deposit_id: self.deposit_id.clone(),
            depositor_did: self.depositor_did.clone(),
            collateral_asset_id: self.collateral_asset_id.clone(),
            collateral_amount: self.collateral_amount,
            quote_asset_id: self.quote_asset_id.clone(),
            request_nonce,
            created_at_micros: self.created_at_micros,
        })
    }
}

impl From<CollateralDepositRequestV2> for CollateralDepositRequestV2Entry {
    fn from(value: CollateralDepositRequestV2) -> Self {
        Self {
            schema_version: value.schema_version,
            deposit_id: value.deposit_id,
            depositor_did: value.depositor_did,
            collateral_asset_id: value.collateral_asset_id,
            collateral_amount: value.collateral_amount,
            quote_asset_id: value.quote_asset_id,
            request_nonce: value.request_nonce.to_vec(),
            created_at_micros: value.created_at_micros,
        }
    }
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

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CollateralSapMintAuthorizationV2Entry {
    pub record: CollateralSapMintAuthorizationRecordV2,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CollateralSapMintRecordV2Entry {
    pub record: CollateralSapMintRecordV2Compact,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CollateralSapIssuanceReceiptV2Entry {
    pub record: CollateralSapIssuanceReceiptRecordV2,
}

#[dna_properties]
#[derive(Clone)]
pub struct FinanceSapAccountV2DnaProperties {
    pub sap_account_v2: SapAccountV2Config,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SapAccountOpenedV2Entry {
    pub opened: SapAccountOpenedV2,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SapCollateralClaimV2Entry {
    pub claim: SapCollateralClaimV2,
}

#[dna_properties]
#[derive(Clone)]
pub struct FinanceSapTransferV2DnaProperties {
    pub sap_transfer_v2: SapTransferV2Config,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SapTransferSpendV2Entry {
    pub spend: SapTransferSpendRecordV2,
}
