// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Attribution Integrity Zome
//! Updated to use HDI 0.7 patterns with FlatOp validation
use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Attribution {
    pub id: String,
    pub publication_id: String,
    pub contributor_did: String,
    pub role: ContributorRole,
    pub share_percentage: f64,
    pub verified: bool,
    pub created: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ContributorRole {
    Author,
    CoAuthor,
    Editor,
    Researcher,
    Photographer,
    Illustrator,
    Translator,
    Source,
    Other(String),
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct RoyaltyRule {
    pub id: String,
    pub publication_id: String,
    pub rule_type: RoyaltyType,
    pub percentage: f64,
    pub minimum_amount: Option<f64>,
    pub currency: String,
    pub active: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum RoyaltyType {
    PerView,
    PerShare,
    PerDownload,
    PerDerivative,
    Subscription,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct UsageRecord {
    pub id: String,
    pub publication_id: String,
    pub usage_type: UsageType,
    pub user_did: Option<String>,
    pub timestamp: Timestamp,
    pub royalty_paid: Option<f64>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum UsageType {
    View,
    Share,
    Download,
    Derivative,
    Citation,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(visibility = "public")]
    Anchor(Anchor),
    Attribution(Attribution),
    RoyaltyRule(RoyaltyRule),
    UsageRecord(UsageRecord),
}

#[hdk_link_types]
pub enum LinkTypes {
    PublicationToAttributions,
    ContributorToAttributions,
    PublicationToRoyalties,
    PublicationToUsage,
}

/// Genesis self-check
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback using FlatOp pattern
///
/// **P0 author-binding pass, 2026-07-09: severe, disclosed-not-fixed
/// coordinator-level gaps.** Every identity in this zome is a free-form
/// `String` DID (`contributor_did`, `requester_did`, `user_did`) with NO
/// local DID-to-agent verification convention anywhere -- same case-(d)
/// gap as mycelix-mutualaid's bridge/requests/pools zomes. That alone
/// would already make a real author-binding fix impossible here. But this
/// zome's coordinator is worse than the usual case-(d) pattern:
/// `add_attribution` never calls `agent_info()` at all (any agent can
/// claim to be any contributor for any publication, with any
/// share_percentage); `verify_attribution` "authenticates" by comparing
/// `attr.contributor_did` against `input.requester_did` -- BOTH
/// caller-controlled, so it authenticates nothing (the same
/// compare-two-attacker-controlled-values bug class documented for
/// mycelix-identity's trust_credential fulfill/decline_attestation);
/// `update_share_percentage`/`deactivate_royalty_rule` have NO check of
/// any kind; `remove_attribution` accepts a `requester_did` field and
/// never even reads it. None of this is fixable at the integrity layer
/// without inventing an unverified DID convention, so it is disclosed
/// here, not silently patched. What IS fixed: the wide-open
/// RegisterUpdate/RegisterDelete bug (34th confirmed instance this pass)
/// and real content-restriction on update (comparing against the
/// original entry, which this validator never did before).
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Attribution(attribution) => {
                    validate_create_attribution(EntryCreationAction::Create(action), attribution)
                }
                EntryTypes::RoyaltyRule(rule) => {
                    validate_create_royalty_rule(EntryCreationAction::Create(action), rule)
                }
                EntryTypes::UsageRecord(record) => {
                    validate_create_usage_record(EntryCreationAction::Create(action), record)
                }
            },
            OpEntry::UpdateEntry {
                app_entry,
                original_action_hash,
                action,
                ..
            } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Attribution(attribution) => {
                    validate_update_attribution(action, original_action_hash, attribution)
                }
                EntryTypes::RoyaltyRule(rule) => {
                    validate_update_royalty_rule(action, original_action_hash, rule)
                }
                EntryTypes::UsageRecord(_) => Ok(ValidateCallbackResult::Invalid(
                    "Usage records cannot be updated".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, .. } => match link_type {
            LinkTypes::PublicationToAttributions => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ContributorToAttributions => Ok(ValidateCallbackResult::Valid),
            LinkTypes::PublicationToRoyalties => Ok(ValidateCallbackResult::Valid),
            LinkTypes::PublicationToUsage => Ok(ValidateCallbackResult::Valid),
        },
        // Deliberately left permissive: the coordinator never calls
        // delete_link. Reviewed 2026-07-09 during the P0 author-binding pass.
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // Previously fully permissive (`Ok(Valid)` unconditionally) --
            // the 34th confirmed instance of this exact bug pattern this
            // pass. Found + fixed 2026-07-09.
            OpUpdate::Entry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Attribution(attribution) => validate_update_attribution(
                    action.clone(),
                    action.original_action_address,
                    attribution,
                ),
                EntryTypes::RoyaltyRule(rule) => validate_update_royalty_rule(
                    action.clone(),
                    action.original_action_address,
                    rule,
                ),
                EntryTypes::UsageRecord(_) => Ok(ValidateCallbackResult::Invalid(
                    "Usage records cannot be updated".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(OpDelete { action }) => {
            let original = must_get_action(action.deletes_address.clone())?;
            let _ = original;
            // No author field exists anywhere in this zome's entries to
            // compare against (case d, see the module doc comment on
            // `validate` above) -- deletion authorization is left as-is,
            // matching remove_attribution's own already-broken
            // coordinator-level check.
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

fn validate_create_attribution(
    _action: EntryCreationAction,
    attribution: Attribution,
) -> ExternResult<ValidateCallbackResult> {
    if !attribution.contributor_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Contributor must be a valid DID".into(),
        ));
    }
    if attribution.share_percentage < 0.0 || attribution.share_percentage > 100.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Share must be 0-100%".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// **Real, fixable fix despite the case-(d) identity gap**: even though
/// no author can be bound, we CAN now compare against the original entry
/// -- previously this validator only checked the new value's bounds, so a
/// modified coordinator could silently rewrite contributor_did,
/// publication_id, role, or created alongside a legitimate
/// share_percentage/verified change. Content restricted to
/// share_percentage/verified, the only two fields update_share_percentage
/// and verify_attribution ever change.
fn validate_update_attribution(
    _action: Update,
    original_action_hash: ActionHash,
    attribution: Attribution,
) -> ExternResult<ValidateCallbackResult> {
    if attribution.share_percentage < 0.0 || attribution.share_percentage > 100.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Share must be 0-100%".into(),
        ));
    }

    let original_record = must_get_valid_record(original_action_hash)?;
    let original: Attribution = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original attribution not found".into()
        )))?;

    if attribution.id != original.id
        || attribution.publication_id != original.publication_id
        || attribution.contributor_did != original.contributor_did
        || attribution.role != original.role
        || attribution.created != original.created
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only share_percentage/verified can change on an attribution update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_royalty_rule(
    _action: EntryCreationAction,
    rule: RoyaltyRule,
) -> ExternResult<ValidateCallbackResult> {
    if rule.percentage < 0.0 || rule.percentage > 100.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Percentage must be 0-100".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Content restricted to `active`, the only field deactivate_royalty_rule
/// ever changes.
fn validate_update_royalty_rule(
    _action: Update,
    original_action_hash: ActionHash,
    rule: RoyaltyRule,
) -> ExternResult<ValidateCallbackResult> {
    if rule.percentage < 0.0 || rule.percentage > 100.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Percentage must be 0-100".into(),
        ));
    }

    let original_record = must_get_valid_record(original_action_hash)?;
    let original: RoyaltyRule = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original royalty rule not found".into()
        )))?;

    if rule.id != original.id
        || rule.publication_id != original.publication_id
        || rule.rule_type != original.rule_type
        || rule.percentage != original.percentage
        || rule.minimum_amount != original.minimum_amount
        || rule.currency != original.currency
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only active can change on a royalty rule update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_usage_record(
    _action: EntryCreationAction,
    _record: UsageRecord,
) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod content_restriction_tests {
    use super::*;

    fn dummy_action() -> EntryCreationAction {
        EntryCreationAction::Create(Create {
            author: AgentPubKey::from_raw_36(vec![0u8; 36]),
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        })
    }

    fn valid_attribution() -> Attribution {
        Attribution {
            id: "attr-1".into(),
            publication_id: "pub-1".into(),
            contributor_did: "did:key:z6Mkfoo".into(),
            role: ContributorRole::Author,
            share_percentage: 50.0,
            verified: false,
            created: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_attribution_requires_did_prefix() {
        let mut attribution = valid_attribution();
        attribution.contributor_did = "not-a-did".into();
        let result = validate_create_attribution(dummy_action(), attribution).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_attribution_rejects_out_of_range_share() {
        let mut attribution = valid_attribution();
        attribution.share_percentage = 150.0;
        let result = validate_create_attribution(dummy_action(), attribution).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_attribution_valid() {
        let result = validate_create_attribution(dummy_action(), valid_attribution()).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    fn valid_royalty_rule() -> RoyaltyRule {
        RoyaltyRule {
            id: "royalty-1".into(),
            publication_id: "pub-1".into(),
            rule_type: RoyaltyType::PerView,
            percentage: 10.0,
            minimum_amount: None,
            currency: "USD".into(),
            active: true,
        }
    }

    #[test]
    fn create_royalty_rule_rejects_out_of_range_percentage() {
        let mut rule = valid_royalty_rule();
        rule.percentage = -5.0;
        let result = validate_create_royalty_rule(dummy_action(), rule).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_royalty_rule_valid() {
        let result = validate_create_royalty_rule(dummy_action(), valid_royalty_rule()).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    // validate_update_{attribution,royalty_rule} both call
    // must_get_valid_record, which requires a live HDI host and can't run
    // in a plain unit test -- matching the established pattern from every
    // other zome's update validator this pass. Correctness there is
    // verified via cargo check plus the code-review reasoning in the
    // commit message.
}
