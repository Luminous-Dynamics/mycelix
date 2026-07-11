// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Property Registry Integrity Zome
use hdi::prelude::*;

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Property {
    pub id: String,
    pub property_type: PropertyType,
    pub title: String,
    pub description: String,
    pub owner_did: String,
    pub co_owners: Vec<CoOwner>,
    pub geolocation: Option<GeoLocation>,
    pub address: Option<Address>,
    pub metadata: PropertyMetadata,
    pub registered: Timestamp,
    pub last_transfer: Option<Timestamp>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum PropertyType {
    Land,
    Building,
    Unit,
    Equipment,
    Intellectual,
    Digital,
    Other(String),
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct CoOwner {
    pub did: String,
    pub share_percentage: f64,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct GeoLocation {
    pub latitude: f64,
    pub longitude: f64,
    pub boundaries: Option<Vec<(f64, f64)>>,
    pub area_sqm: Option<f64>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Address {
    pub street: String,
    pub city: String,
    pub region: String,
    pub country: String,
    pub postal_code: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct PropertyMetadata {
    pub appraised_value: Option<f64>,
    pub currency: Option<String>,
    pub legal_description: Option<String>,
    pub parcel_number: Option<String>,
    pub attachments: Vec<String>,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TitleDeed {
    pub id: String,
    pub property_id: String,
    pub owner_did: String,
    pub deed_type: DeedType,
    pub issued: Timestamp,
    pub previous_deed_id: Option<String>,
    pub encumbrances: Vec<Encumbrance>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum DeedType {
    Original,
    Transfer,
    Inheritance,
    CourtOrder,
    Fractional,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Encumbrance {
    pub encumbrance_type: EncumbranceType,
    pub holder_did: String,
    pub amount: Option<f64>,
    pub registered: Timestamp,
    pub expires: Option<Timestamp>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum EncumbranceType {
    Mortgage,
    Lien,
    Easement,
    Restriction,
    Lease,
}

/// Anchor entry for deterministic link bases from strings
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Property(Property),
    TitleDeed(TitleDeed),
    #[entry_type(visibility = "public")]
    Anchor(Anchor),
}

#[hdk_link_types]
pub enum LinkTypes {
    OwnerToProperties,
    PropertyToDeeds,
    LocationToProperty,
    PropertyToEncumbrances,
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
                EntryTypes::Property(property) => {
                    validate_create_property(EntryCreationAction::Create(action), property)
                }
                EntryTypes::TitleDeed(deed) => {
                    validate_create_title_deed(EntryCreationAction::Create(action), deed)
                }
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::Property(property) => validate_update_property(action, property),
                EntryTypes::TitleDeed(_) => Ok(ValidateCallbackResult::Invalid(
                    "Title deeds cannot be updated".into(),
                )),
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, .. } => match link_type {
            LinkTypes::OwnerToProperties => Ok(ValidateCallbackResult::Valid),
            LinkTypes::PropertyToDeeds => Ok(ValidateCallbackResult::Valid),
            LinkTypes::LocationToProperty => Ok(ValidateCallbackResult::Valid),
            LinkTypes::PropertyToEncumbrances => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_property(
    action: EntryCreationAction,
    property: Property,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the property to its committer -- register_property already
    // derives `owner_did` from agent_info() coordinator-side with zero user
    // input, so this never rejects a legitimate registration; it's the real
    // DHT-level enforcement a modified coordinator could otherwise bypass
    // (P0 author-binding gap).
    let expected_owner = format!("did:mycelix:{}", action.author());
    if property.owner_did != expected_owner {
        return Ok(ValidateCallbackResult::Invalid(
            "Property owner must be the committing agent (owner forgery)".to_string(),
        ));
    }

    if !property.owner_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Owner must be a valid DID".into(),
        ));
    }
    let mut total_share = 100.0;
    for co_owner in &property.co_owners {
        if !co_owner.did.starts_with("did:") {
            return Ok(ValidateCallbackResult::Invalid(
                "Co-owner must be a valid DID".into(),
            ));
        }
        if co_owner.share_percentage <= 0.0 || co_owner.share_percentage > 100.0 {
            return Ok(ValidateCallbackResult::Invalid(
                "Share must be between 0 and 100".into(),
            ));
        }
        total_share -= co_owner.share_percentage;
    }
    if total_share < 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Total shares exceed 100%".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_property(
    action: Update,
    _property: Property,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the update to the PRE-update owner -- otherwise any agent could
    // update (including transfer_ownership's update_entry call on) a
    // property they don't own, since a modified coordinator could skip
    // update_property_metadata/transfer_ownership's caller-side ownership
    // checks entirely. Fetch the entry as it stood before this update and
    // require the committing agent to be its owner (P0 author-binding gap).
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original_property: Property = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid original property entry".to_string()
        )))?;
    let committer_did = format!("did:mycelix:{}", action.author);
    if original_property.owner_did != committer_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Property update must be committed by the current owner".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_title_deed(
    _action: EntryCreationAction,
    deed: TitleDeed,
) -> ExternResult<ValidateCallbackResult> {
    // Deliberately NOT author-bound to deed.owner_did: a TitleDeed is
    // legitimately minted by the OLD owner during transfer_ownership, naming
    // the NEW owner as deed.owner_did -- author != owner_did is the expected
    // case for a transfer deed, not forgery. The real ownership-forgery
    // protection lives in validate_update_property above (the Property
    // entry's owner_did can only be changed by its current owner); a deed
    // is downstream of that already-enforced invariant. Verifying the
    // deed's own provenance against the property/deed chain would need
    // must_get chain-walking -- out of scope for this pass.
    if !deed.owner_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Owner must be a valid DID".into(),
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

    /// The DID that matches `test_action()`'s author -- valid_property()
    /// uses this so the new author-binding check passes for the
    /// "otherwise valid" baseline case; the forgery test uses a different
    /// author/action instead.
    fn test_author_did() -> String {
        format!("did:mycelix:{}", test_action().author)
    }

    fn valid_property(owner_did: String) -> Property {
        Property {
            id: "property:test:0".to_string(),
            property_type: PropertyType::Land,
            title: "Test Parcel".to_string(),
            description: "A test property".to_string(),
            owner_did,
            co_owners: vec![],
            geolocation: None,
            address: None,
            metadata: PropertyMetadata {
                appraised_value: None,
                currency: None,
                legal_description: None,
                parcel_number: None,
                attachments: vec![],
            },
            registered: Timestamp::from_micros(0),
            last_transfer: None,
        }
    }

    #[test]
    fn test_create_property_valid() {
        let property = valid_property(test_author_did());
        let result =
            validate_create_property(EntryCreationAction::Create(test_action()), property).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_create_property_owner_forgery_rejected() {
        // owner_did claims test_author_did(), but the committing action's
        // author is a different agent.
        let mut forged_action = test_action();
        forged_action.author = AgentPubKey::from_raw_36(vec![1u8; 36]);
        let property = valid_property(test_author_did());
        let result =
            validate_create_property(EntryCreationAction::Create(forged_action), property).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_create_property_non_did_owner_rejected() {
        let property = valid_property("not-a-did".to_string());
        let result =
            validate_create_property(EntryCreationAction::Create(test_action()), property).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_create_property_share_overflow_rejected() {
        let mut property = valid_property(test_author_did());
        property.co_owners = vec![CoOwner {
            did: "did:mycelix:co-owner".to_string(),
            share_percentage: 150.0,
        }];
        let result =
            validate_create_property(EntryCreationAction::Create(test_action()), property).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_create_title_deed_valid() {
        let deed = TitleDeed {
            id: "deed:test:0".to_string(),
            property_id: "property:test:0".to_string(),
            owner_did: test_author_did(),
            deed_type: DeedType::Original,
            issued: Timestamp::from_micros(0),
            previous_deed_id: None,
            encumbrances: vec![],
        };
        let result =
            validate_create_title_deed(EntryCreationAction::Create(test_action()), deed).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }
}
