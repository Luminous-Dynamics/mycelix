#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-014 pure owner-authored SAP V2 collateral-claim journal.
//!
//! Positive collateral claims are immutable economic facts rather than scalar
//! balance rewrites. Projection de-duplicates by FIN-SAFE-010's canonical
//! `mint_id`, so physical duplicate claim/receipt/mint actions cannot multiply
//! value merely because their Holochain action hashes differ.

use finance_collateral_issuance_persistence::{
    CollateralSapIssuanceReceiptRecordV2,
    COLLATERAL_ISSUANCE_RECEIPT_RECORD_V2_SCHEMA_VERSION,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const SAP_ACCOUNT_V2_SCHEMA_VERSION: u16 = 1;
pub const MAX_ACTION_REFERENCE_LEN: usize = 256;
pub const MAX_DID_LEN: usize = 256;
pub const MAX_ID_LEN: usize = 256;

/// DNA-level rollout switch. Shipped Finance DNA keeps this `false` until the
/// V2 account protocol is intentionally enabled on a deliberate network/DNA.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapAccountV2Config {
    pub enabled: bool,
}

impl Default for SapAccountV2Config {
    fn default() -> Self {
        Self { enabled: false }
    }
}

impl SapAccountV2Config {
    pub fn validate(&self) -> Result<(), SapAccountV2Error> {
        Ok(())
    }

    pub fn require_enabled(&self) -> Result<(), SapAccountV2Error> {
        self.validate()?;
        if self.enabled {
            Ok(())
        } else {
            Err(SapAccountV2Error::ProtocolDisabled)
        }
    }
}

/// Zero-economic-effect account opening marker.
///
/// Multiple physical opening actions are harmless: opening contributes no value.
/// Authority comes from binding the real action author to `member_did`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapAccountOpenedV2 {
    pub schema_version: u16,
    pub member_did: String,
}

impl SapAccountOpenedV2 {
    pub fn new(member_did: String) -> Result<Self, SapAccountV2Error> {
        let opened = Self {
            schema_version: SAP_ACCOUNT_V2_SCHEMA_VERSION,
            member_did,
        };
        opened.validate_shape()?;
        Ok(opened)
    }

    pub fn validate_shape(&self) -> Result<(), SapAccountV2Error> {
        if self.schema_version != SAP_ACCOUNT_V2_SCHEMA_VERSION {
            return Err(SapAccountV2Error::UnsupportedSchemaVersion);
        }
        validate_did(&self.member_did)
    }

    pub fn validate_author(&self, action_author_did: &str) -> Result<(), SapAccountV2Error> {
        self.validate_shape()?;
        require_owner_author(action_author_did, &self.member_did)
    }
}

/// Owner-authored claim of one exact FIN-SAFE-010 issuance receipt.
///
/// The claim intentionally contains no amount, deposit ID, or mint ID. Those facts
/// are reconstructed from the exact valid receipt action by the integration layer.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapCollateralClaimV2 {
    pub schema_version: u16,
    pub member_did: String,
    pub issuance_receipt_action_reference: String,
}

impl SapCollateralClaimV2 {
    pub fn new(
        member_did: String,
        issuance_receipt_action_reference: String,
    ) -> Result<Self, SapAccountV2Error> {
        let claim = Self {
            schema_version: SAP_ACCOUNT_V2_SCHEMA_VERSION,
            member_did,
            issuance_receipt_action_reference,
        };
        claim.validate_shape()?;
        Ok(claim)
    }

    pub fn validate_shape(&self) -> Result<(), SapAccountV2Error> {
        if self.schema_version != SAP_ACCOUNT_V2_SCHEMA_VERSION {
            return Err(SapAccountV2Error::UnsupportedSchemaVersion);
        }
        validate_did(&self.member_did)?;
        validate_action_reference(&self.issuance_receipt_action_reference)
    }
}

/// Fully reconstructed economic claim used by projection.
///
/// Construction requires an exact, independently validated FIN-SAFE-010 receipt.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ValidatedCollateralClaimV2 {
    pub claim_action_reference: String,
    pub action_author_did: String,
    pub member_did: String,
    pub issuance_receipt_action_reference: String,
    pub mint_id: String,
    pub deposit_id: String,
    pub amount: u64,
}

impl ValidatedCollateralClaimV2 {
    pub fn from_valid_receipt(
        claim_action_reference: String,
        action_author_did: String,
        claim: &SapCollateralClaimV2,
        receipt: &CollateralSapIssuanceReceiptRecordV2,
    ) -> Result<Self, SapAccountV2Error> {
        claim.validate_shape()?;
        validate_action_reference(&claim_action_reference)?;
        require_owner_author(&action_author_did, &claim.member_did)?;
        validate_receipt_shape(receipt)?;
        if receipt.recipient_did != claim.member_did {
            return Err(SapAccountV2Error::ReceiptRecipientMismatch);
        }

        let validated = Self {
            claim_action_reference,
            action_author_did,
            member_did: claim.member_did.clone(),
            issuance_receipt_action_reference: claim.issuance_receipt_action_reference.clone(),
            mint_id: receipt.mint_id.clone(),
            deposit_id: receipt.deposit_id.clone(),
            amount: receipt.amount,
        };
        validated.validate_shape()?;
        Ok(validated)
    }

    pub fn validate_shape(&self) -> Result<(), SapAccountV2Error> {
        validate_action_reference(&self.claim_action_reference)?;
        validate_action_reference(&self.issuance_receipt_action_reference)?;
        validate_did(&self.action_author_did)?;
        validate_did(&self.member_did)?;
        require_owner_author(&self.action_author_did, &self.member_did)?;
        validate_id(&self.mint_id)?;
        validate_id(&self.deposit_id)?;
        if self.amount == 0 {
            return Err(SapAccountV2Error::ZeroAmount);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DuplicateCollateralClaimGroupV2 {
    pub mint_id: String,
    pub claim_action_references: Vec<String>,
    pub issuance_receipt_action_references: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SapCollateralProjectionV2 {
    pub member_did: String,
    /// Unique physical claim actions after duplicate query observations are removed.
    pub physical_claim_action_count: u64,
    /// Canonical economic issuance identities counted exactly once.
    pub unique_mint_count: u64,
    /// Extra physical claim actions beyond the unique economic mint set.
    pub duplicate_claim_action_count: u64,
    pub projected_balance: u64,
    pub duplicate_groups: Vec<DuplicateCollateralClaimGroupV2>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct CanonicalMintFacts {
    member_did: String,
    deposit_id: String,
    amount: u64,
    claim_action_references: BTreeSet<String>,
    issuance_receipt_action_references: BTreeSet<String>,
}

/// Project the collateral-claim subset of one owner's V2 SAP account.
///
/// This is intentionally a set projection, not a mutable scalar update. Different
/// valid mint IDs commute; duplicate physical actions for one mint ID count once.
pub fn project_collateral_claims(
    member_did: &str,
    observations: &[ValidatedCollateralClaimV2],
) -> Result<SapCollateralProjectionV2, SapAccountV2Error> {
    validate_did(member_did)?;

    // First de-duplicate repeated retrieval of the exact same claim action.
    let mut physical_actions: BTreeMap<String, ValidatedCollateralClaimV2> = BTreeMap::new();
    for observation in observations {
        observation.validate_shape()?;
        if observation.member_did != member_did {
            return Err(SapAccountV2Error::ForeignMemberObservation);
        }
        match physical_actions.get(&observation.claim_action_reference) {
            Some(existing) if existing == observation => {}
            Some(_) => return Err(SapAccountV2Error::InconsistentDuplicateClaimAction),
            None => {
                physical_actions.insert(
                    observation.claim_action_reference.clone(),
                    observation.clone(),
                );
            }
        }
    }

    let mut by_mint: BTreeMap<String, CanonicalMintFacts> = BTreeMap::new();
    let mut deposit_to_mint: BTreeMap<String, String> = BTreeMap::new();

    for observation in physical_actions.values() {
        match deposit_to_mint.get(&observation.deposit_id) {
            Some(existing_mint) if existing_mint != &observation.mint_id => {
                return Err(SapAccountV2Error::ConflictingDepositIdentity)
            }
            Some(_) => {}
            None => {
                deposit_to_mint.insert(
                    observation.deposit_id.clone(),
                    observation.mint_id.clone(),
                );
            }
        }

        match by_mint.get_mut(&observation.mint_id) {
            Some(existing) => {
                if existing.member_did != observation.member_did
                    || existing.deposit_id != observation.deposit_id
                    || existing.amount != observation.amount
                {
                    return Err(SapAccountV2Error::ConflictingEconomicIdentity);
                }
                existing
                    .claim_action_references
                    .insert(observation.claim_action_reference.clone());
                existing
                    .issuance_receipt_action_references
                    .insert(observation.issuance_receipt_action_reference.clone());
            }
            None => {
                by_mint.insert(
                    observation.mint_id.clone(),
                    CanonicalMintFacts {
                        member_did: observation.member_did.clone(),
                        deposit_id: observation.deposit_id.clone(),
                        amount: observation.amount,
                        claim_action_references: BTreeSet::from([
                            observation.claim_action_reference.clone(),
                        ]),
                        issuance_receipt_action_references: BTreeSet::from([
                            observation.issuance_receipt_action_reference.clone(),
                        ]),
                    },
                );
            }
        }
    }

    let mut projected_balance = 0u64;
    let mut duplicate_groups = Vec::new();
    for (mint_id, facts) in &by_mint {
        projected_balance = projected_balance
            .checked_add(facts.amount)
            .ok_or(SapAccountV2Error::BalanceOverflow)?;
        if facts.claim_action_references.len() > 1
            || facts.issuance_receipt_action_references.len() > 1
        {
            duplicate_groups.push(DuplicateCollateralClaimGroupV2 {
                mint_id: mint_id.clone(),
                claim_action_references: facts.claim_action_references.iter().cloned().collect(),
                issuance_receipt_action_references: facts
                    .issuance_receipt_action_references
                    .iter()
                    .cloned()
                    .collect(),
            });
        }
    }

    let physical_claim_action_count = u64::try_from(physical_actions.len())
        .map_err(|_| SapAccountV2Error::ObservationCountOverflow)?;
    let unique_mint_count =
        u64::try_from(by_mint.len()).map_err(|_| SapAccountV2Error::ObservationCountOverflow)?;
    let duplicate_claim_action_count = physical_claim_action_count
        .checked_sub(unique_mint_count)
        .ok_or(SapAccountV2Error::ObservationCountUnderflow)?;

    Ok(SapCollateralProjectionV2 {
        member_did: member_did.to_string(),
        physical_claim_action_count,
        unique_mint_count,
        duplicate_claim_action_count,
        projected_balance,
        duplicate_groups,
    })
}

fn validate_receipt_shape(
    receipt: &CollateralSapIssuanceReceiptRecordV2,
) -> Result<(), SapAccountV2Error> {
    if receipt.schema_version != COLLATERAL_ISSUANCE_RECEIPT_RECORD_V2_SCHEMA_VERSION {
        return Err(SapAccountV2Error::UnsupportedReceiptSchemaVersion);
    }
    for reference in [
        &receipt.authorization_action_reference,
        &receipt.mint_action_reference,
        &receipt.request_action_reference,
        &receipt.price_attestation_action_reference,
        &receipt.custody_attestation_action_reference,
    ] {
        validate_action_reference(reference)?;
    }
    validate_id(&receipt.trust_root_id)?;
    if receipt.trust_root_version == 0 {
        return Err(SapAccountV2Error::InvalidTrustRootVersion);
    }
    validate_id(&receipt.deposit_id)?;
    validate_id(&receipt.mint_id)?;
    validate_did(&receipt.recipient_did)?;
    if receipt.amount == 0 {
        return Err(SapAccountV2Error::ZeroAmount);
    }
    Ok(())
}

fn require_owner_author(action_author_did: &str, member_did: &str) -> Result<(), SapAccountV2Error> {
    validate_did(action_author_did)?;
    validate_did(member_did)?;
    if action_author_did != member_did {
        return Err(SapAccountV2Error::ActionAuthorMismatch);
    }
    Ok(())
}

fn validate_action_reference(value: &str) -> Result<(), SapAccountV2Error> {
    if value.is_empty() || value.len() > MAX_ACTION_REFERENCE_LEN {
        return Err(SapAccountV2Error::InvalidActionReference);
    }
    Ok(())
}

fn validate_did(value: &str) -> Result<(), SapAccountV2Error> {
    if !value.starts_with("did:") || value.len() > MAX_DID_LEN {
        return Err(SapAccountV2Error::InvalidDid);
    }
    Ok(())
}

fn validate_id(value: &str) -> Result<(), SapAccountV2Error> {
    if value.is_empty() || value.len() > MAX_ID_LEN {
        return Err(SapAccountV2Error::InvalidId);
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SapAccountV2Error {
    ProtocolDisabled,
    UnsupportedSchemaVersion,
    UnsupportedReceiptSchemaVersion,
    InvalidActionReference,
    InvalidDid,
    InvalidId,
    InvalidTrustRootVersion,
    ActionAuthorMismatch,
    ReceiptRecipientMismatch,
    ZeroAmount,
    ForeignMemberObservation,
    InconsistentDuplicateClaimAction,
    ConflictingEconomicIdentity,
    ConflictingDepositIdentity,
    BalanceOverflow,
    ObservationCountOverflow,
    ObservationCountUnderflow,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn receipt(mint_id: &str, deposit_id: &str, recipient: &str, amount: u64) -> CollateralSapIssuanceReceiptRecordV2 {
        CollateralSapIssuanceReceiptRecordV2 {
            schema_version: COLLATERAL_ISSUANCE_RECEIPT_RECORD_V2_SCHEMA_VERSION,
            authorization_action_reference: "uhCkk-auth".into(),
            mint_action_reference: "uhCkk-mint".into(),
            trust_root_id: "root".into(),
            trust_root_version: 1,
            request_action_reference: "uhCkk-request".into(),
            price_attestation_action_reference: "uhCkk-price".into(),
            custody_attestation_action_reference: "uhCkk-custody".into(),
            deposit_id: deposit_id.into(),
            mint_id: mint_id.into(),
            recipient_did: recipient.into(),
            amount,
        }
    }

    fn claim(receipt_ref: &str) -> SapCollateralClaimV2 {
        SapCollateralClaimV2::new("did:mycelix:alice".into(), receipt_ref.into()).unwrap()
    }

    fn validated(
        claim_ref: &str,
        receipt_ref: &str,
        mint_id: &str,
        deposit_id: &str,
        amount: u64,
    ) -> ValidatedCollateralClaimV2 {
        ValidatedCollateralClaimV2::from_valid_receipt(
            claim_ref.into(),
            "did:mycelix:alice".into(),
            &claim(receipt_ref),
            &receipt(mint_id, deposit_id, "did:mycelix:alice", amount),
        )
        .unwrap()
    }

    #[test]
    fn shipped_config_is_disabled() {
        let config = SapAccountV2Config::default();
        assert_eq!(config.validate(), Ok(()));
        assert_eq!(config.require_enabled(), Err(SapAccountV2Error::ProtocolDisabled));
    }

    #[test]
    fn opening_marker_is_owner_authored() {
        let opened = SapAccountOpenedV2::new("did:mycelix:alice".into()).unwrap();
        assert_eq!(opened.validate_author("did:mycelix:alice"), Ok(()));
        assert_eq!(
            opened.validate_author("did:mycelix:mallory"),
            Err(SapAccountV2Error::ActionAuthorMismatch)
        );
    }

    #[test]
    fn claim_contains_no_caller_supplied_amount() {
        let effect = ValidatedCollateralClaimV2::from_valid_receipt(
            "uhCkk-claim".into(),
            "did:mycelix:alice".into(),
            &claim("uhCkk-receipt"),
            &receipt("mint:one", "deposit:one", "did:mycelix:alice", 42),
        )
        .unwrap();
        assert_eq!(effect.amount, 42);
        assert_eq!(effect.mint_id, "mint:one");
    }

    #[test]
    fn third_party_or_wrong_recipient_cannot_claim() {
        assert_eq!(
            ValidatedCollateralClaimV2::from_valid_receipt(
                "uhCkk-claim".into(),
                "did:mycelix:mallory".into(),
                &claim("uhCkk-receipt"),
                &receipt("mint:one", "deposit:one", "did:mycelix:alice", 42),
            ),
            Err(SapAccountV2Error::ActionAuthorMismatch)
        );
        assert_eq!(
            ValidatedCollateralClaimV2::from_valid_receipt(
                "uhCkk-claim".into(),
                "did:mycelix:alice".into(),
                &claim("uhCkk-receipt"),
                &receipt("mint:one", "deposit:one", "did:mycelix:bob", 42),
            ),
            Err(SapAccountV2Error::ReceiptRecipientMismatch)
        );
    }

    #[test]
    fn duplicate_physical_claims_for_same_mint_count_once() {
        let projection = project_collateral_claims(
            "did:mycelix:alice",
            &[
                validated("uhCkk-claim-a", "uhCkk-receipt-a", "mint:one", "deposit:one", 50),
                validated("uhCkk-claim-b", "uhCkk-receipt-b", "mint:one", "deposit:one", 50),
            ],
        )
        .unwrap();
        assert_eq!(projection.physical_claim_action_count, 2);
        assert_eq!(projection.unique_mint_count, 1);
        assert_eq!(projection.duplicate_claim_action_count, 1);
        assert_eq!(projection.projected_balance, 50);
        assert_eq!(projection.duplicate_groups.len(), 1);
    }

    #[test]
    fn repeated_observation_of_same_action_is_not_an_economic_duplicate() {
        let observation = validated(
            "uhCkk-claim-a",
            "uhCkk-receipt-a",
            "mint:one",
            "deposit:one",
            50,
        );
        let projection = project_collateral_claims(
            "did:mycelix:alice",
            &[observation.clone(), observation],
        )
        .unwrap();
        assert_eq!(projection.physical_claim_action_count, 1);
        assert_eq!(projection.unique_mint_count, 1);
        assert_eq!(projection.duplicate_claim_action_count, 0);
        assert_eq!(projection.projected_balance, 50);
    }

    #[test]
    fn conflicting_facts_for_same_mint_fail_closed() {
        let mut second = validated(
            "uhCkk-claim-b",
            "uhCkk-receipt-b",
            "mint:one",
            "deposit:one",
            50,
        );
        second.amount = 51;
        assert_eq!(
            project_collateral_claims(
                "did:mycelix:alice",
                &[
                    validated(
                        "uhCkk-claim-a",
                        "uhCkk-receipt-a",
                        "mint:one",
                        "deposit:one",
                        50,
                    ),
                    second,
                ],
            ),
            Err(SapAccountV2Error::ConflictingEconomicIdentity)
        );
    }

    #[test]
    fn same_deposit_cannot_map_to_two_mint_ids() {
        assert_eq!(
            project_collateral_claims(
                "did:mycelix:alice",
                &[
                    validated(
                        "uhCkk-claim-a",
                        "uhCkk-receipt-a",
                        "mint:one",
                        "deposit:one",
                        50,
                    ),
                    validated(
                        "uhCkk-claim-b",
                        "uhCkk-receipt-b",
                        "mint:two",
                        "deposit:one",
                        50,
                    ),
                ],
            ),
            Err(SapAccountV2Error::ConflictingDepositIdentity)
        );
    }

    #[test]
    fn unique_mints_sum_with_checked_addition() {
        let projection = project_collateral_claims(
            "did:mycelix:alice",
            &[
                validated("uhCkk-a", "uhCkk-ra", "mint:a", "deposit:a", 10),
                validated("uhCkk-b", "uhCkk-rb", "mint:b", "deposit:b", 20),
            ],
        )
        .unwrap();
        assert_eq!(projection.projected_balance, 30);
        assert_eq!(projection.unique_mint_count, 2);
    }

    #[test]
    fn projection_overflow_fails_closed() {
        assert_eq!(
            project_collateral_claims(
                "did:mycelix:alice",
                &[
                    validated(
                        "uhCkk-a",
                        "uhCkk-ra",
                        "mint:a",
                        "deposit:a",
                        u64::MAX,
                    ),
                    validated("uhCkk-b", "uhCkk-rb", "mint:b", "deposit:b", 1),
                ],
            ),
            Err(SapAccountV2Error::BalanceOverflow)
        );
    }

    #[test]
    fn projection_round_trips() {
        let projection = project_collateral_claims(
            "did:mycelix:alice",
            &[validated(
                "uhCkk-a",
                "uhCkk-ra",
                "mint:a",
                "deposit:a",
                10,
            )],
        )
        .unwrap();
        let bytes = serde_json::to_vec(&projection).unwrap();
        let decoded: SapCollateralProjectionV2 = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(decoded, projection);
    }
}
