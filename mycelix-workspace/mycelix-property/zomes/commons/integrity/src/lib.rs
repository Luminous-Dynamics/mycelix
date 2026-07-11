// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Commons Management Integrity Zome
use hdi::prelude::*;

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CommonResource {
    pub id: String,
    pub name: String,
    pub description: String,
    pub resource_type: ResourceType,
    pub property_id: Option<String>,
    pub stewards: Vec<String>,
    pub governance_rules: GovernanceRules,
    pub created: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ResourceType {
    Land,
    Water,
    Forest,
    Fishery,
    Pasture,
    Infrastructure,
    Digital,
    Other(String),
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct GovernanceRules {
    pub access_rules: Vec<String>,
    pub usage_limits: Vec<UsageLimit>,
    pub maintenance_rotation: bool,
    pub decision_method: DecisionMethod,
    pub penalty_for_violation: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct UsageLimit {
    pub limit_type: String,
    pub max_per_period: f64,
    pub period_days: u32,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum DecisionMethod {
    Consensus,
    Majority,
    SuperMajority,
    Stewards,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct UsageRight {
    pub id: String,
    pub resource_id: String,
    pub holder_did: String,
    pub right_type: RightType,
    pub quota: Option<f64>,
    pub granted: Timestamp,
    pub expires: Option<Timestamp>,
    pub active: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum RightType {
    Access,
    Extraction,
    Management,
    Exclusion,
    Alienation,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct UsageLog {
    pub id: String,
    pub resource_id: String,
    pub user_did: String,
    pub usage_type: String,
    pub quantity: f64,
    pub unit: String,
    pub timestamp: Timestamp,
}

/// Anchor entry for deterministic link bases from strings
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    CommonResource(CommonResource),
    UsageRight(UsageRight),
    UsageLog(UsageLog),
    #[entry_type(visibility = "public")]
    Anchor(Anchor),
}

#[hdk_link_types]
pub enum LinkTypes {
    StewardToResource,
    ResourceToRights,
    HolderToRights,
    ResourceToLogs,
}

/// Genesis self-check
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback using FlatOp pattern
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::CommonResource(resource) => {
                    validate_create_common_resource(EntryCreationAction::Create(action), resource)
                }
                EntryTypes::UsageRight(right) => {
                    validate_create_usage_right(EntryCreationAction::Create(action), right)
                }
                EntryTypes::UsageLog(log) => {
                    validate_create_usage_log(EntryCreationAction::Create(action), log)
                }
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::CommonResource(resource) => {
                    validate_update_common_resource(action, resource)
                }
                EntryTypes::UsageRight(right) => validate_update_usage_right(action, right),
                EntryTypes::UsageLog(_) => Ok(ValidateCallbackResult::Invalid(
                    "Usage logs cannot be updated".into(),
                )),
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, .. } => match link_type {
            LinkTypes::StewardToResource => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ResourceToRights => Ok(ValidateCallbackResult::Valid),
            LinkTypes::HolderToRights => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ResourceToLogs => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_common_resource(
    _action: EntryCreationAction,
    resource: CommonResource,
) -> ExternResult<ValidateCallbackResult> {
    // Deliberately NOT author-bound: CommonResource has no single "creator"
    // field distinct from `stewards` -- it's a group from the start, and a
    // founder legitimately names OTHER agents as co-stewards without those
    // agents having committed anything themselves. There's no forgery risk
    // at creation time itself since founding a resource grants the creator
    // no privilege they don't already have (stewardship is what's checked
    // downstream, not resource existence).
    if resource.stewards.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Resource must have at least one steward".into(),
        ));
    }
    for steward in &resource.stewards {
        if !steward.starts_with("did:") {
            return Ok(ValidateCallbackResult::Invalid(
                "Stewards must be valid DIDs".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_common_resource(
    action: Update,
    _resource: CommonResource,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the update to an EXISTING steward -- add_steward/remove_steward/
    // update_governance_rules all update this same entry after a
    // coordinator-side `stewards.contains(added_by_did/...)` check, but
    // that check trusts a caller-supplied DID unless also enforced here.
    // Fetch the resource as it stood before this update and require the
    // committing agent to already be one of its stewards (P0 author-binding
    // gap) -- this is the real DHT-level enforcement a modified coordinator
    // can't bypass.
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original_resource: CommonResource = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid original resource entry".to_string()
        )))?;
    let committer_did = format!("did:mycelix:{}", action.author);
    if !original_resource.stewards.contains(&committer_did) {
        return Ok(ValidateCallbackResult::Invalid(
            "Resource update must be committed by an existing steward".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_usage_right(
    _action: EntryCreationAction,
    right: UsageRight,
) -> ExternResult<ValidateCallbackResult> {
    // Deliberately NOT author-bound: holder_did names the party the right
    // is granted TO, not necessarily the committing agent (grant_usage_right
    // has no steward-of-resource check today either -- a separate,
    // deeper authorization gap than identity-forgery, flagged but not fixed
    // in this pass).
    if !right.holder_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Holder must be a valid DID".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_usage_right(
    _action: Update,
    _right: UsageRight,
) -> ExternResult<ValidateCallbackResult> {
    // Deliberately NOT author-bound: revoke_usage_right/update_right_quota
    // are gated on stewardship of the right's RESOURCE, not of the right
    // itself, and UsageRight only stores `resource_id: String` -- not the
    // resource's ActionHash -- so there's no must_get path from here to the
    // CommonResource entry to verify stewardship (must_get_valid_record
    // needs a hash, and HDI validation has no query-by-string-field
    // access). The coordinator-side fix (steward_did agent-derived) closes
    // the "unverified self-report" hole, but a fully modified coordinator
    // could still bypass the steward check for right revocation/quota
    // changes. Revisit if UsageRight ever carries the resource's ActionHash.
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_usage_log(
    action: EntryCreationAction,
    log: UsageLog,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the log to its committer -- log_usage already derives `user_did`
    // from agent_info() coordinator-side with zero user input, so this
    // never rejects a legitimate log entry; it's the real DHT-level
    // enforcement a modified coordinator could otherwise bypass (P0
    // author-binding gap).
    let expected_user = format!("did:mycelix:{}", action.author());
    if log.user_did != expected_user {
        return Ok(ValidateCallbackResult::Invalid(
            "Usage log user must be the committing agent (forgery)".to_string(),
        ));
    }

    if !log.user_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "User must be a valid DID".into(),
        ));
    }
    if log.quantity < 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Quantity cannot be negative".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_action() -> Create {
        Create {
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
        }
    }

    fn test_author_did() -> String {
        format!("did:mycelix:{}", test_action().author)
    }

    fn valid_resource() -> CommonResource {
        CommonResource {
            id: "commons:test:0".to_string(),
            name: "Test Commons".to_string(),
            description: "A test resource".to_string(),
            resource_type: ResourceType::Land,
            property_id: None,
            stewards: vec![test_author_did()],
            governance_rules: GovernanceRules {
                access_rules: vec![],
                usage_limits: vec![],
                maintenance_rotation: false,
                decision_method: DecisionMethod::Consensus,
                penalty_for_violation: None,
            },
            created: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn test_create_resource_valid() {
        let resource = valid_resource();
        let result =
            validate_create_common_resource(EntryCreationAction::Create(test_action()), resource)
                .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_create_resource_no_stewards_rejected() {
        let mut resource = valid_resource();
        resource.stewards = vec![];
        let result =
            validate_create_common_resource(EntryCreationAction::Create(test_action()), resource)
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_create_resource_non_did_steward_rejected() {
        let mut resource = valid_resource();
        resource.stewards = vec!["not-a-did".to_string()];
        let result =
            validate_create_common_resource(EntryCreationAction::Create(test_action()), resource)
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_create_usage_log_valid() {
        let log = UsageLog {
            id: "usage:test:0".to_string(),
            resource_id: "commons:test:0".to_string(),
            user_did: test_author_did(),
            usage_type: "water".to_string(),
            quantity: 10.0,
            unit: "liters".to_string(),
            timestamp: Timestamp::from_micros(0),
        };
        let result =
            validate_create_usage_log(EntryCreationAction::Create(test_action()), log).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_create_usage_log_user_forgery_rejected() {
        // user_did claims test_author_did(), but the committing action's
        // author is a different agent.
        let mut forged_action = test_action();
        forged_action.author = AgentPubKey::from_raw_36(vec![1u8; 36]);
        let log = UsageLog {
            id: "usage:test:0".to_string(),
            resource_id: "commons:test:0".to_string(),
            user_did: test_author_did(),
            usage_type: "water".to_string(),
            quantity: 10.0,
            unit: "liters".to_string(),
            timestamp: Timestamp::from_micros(0),
        };
        let result =
            validate_create_usage_log(EntryCreationAction::Create(forged_action), log).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_create_usage_log_negative_quantity_rejected() {
        let log = UsageLog {
            id: "usage:test:0".to_string(),
            resource_id: "commons:test:0".to_string(),
            user_did: test_author_did(),
            usage_type: "water".to_string(),
            quantity: -5.0,
            unit: "liters".to_string(),
            timestamp: Timestamp::from_micros(0),
        };
        let result =
            validate_create_usage_log(EntryCreationAction::Create(test_action()), log).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
