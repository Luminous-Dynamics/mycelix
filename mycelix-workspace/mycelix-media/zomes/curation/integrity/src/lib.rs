// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Curation Integrity Zome
//! Updated to use HDI 0.7 patterns with FlatOp validation
use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Endorsement {
    pub id: String,
    pub publication_id: String,
    pub endorser_did: String,
    pub endorsement_type: EndorsementType,
    pub comment: Option<String>,
    pub created: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum EndorsementType {
    Upvote,
    Bookmark,
    Share,
    Recommend,
    Award(String),
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Collection {
    pub id: String,
    pub name: String,
    pub description: String,
    pub curator_did: String,
    pub visibility: Visibility,
    pub publication_ids: Vec<String>,
    pub created: Timestamp,
    pub updated: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum Visibility {
    Public,
    Private,
    Unlisted,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct QualityScore {
    pub publication_id: String,
    pub score: f64,
    pub endorsement_count: u32,
    pub share_count: u32,
    pub fact_check_score: f64,
    pub author_reputation: f64,
    pub last_calculated: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct FeaturedContent {
    pub id: String,
    pub publication_id: String,
    pub featured_by: String,
    pub reason: String,
    pub featured_from: Timestamp,
    pub featured_until: Option<Timestamp>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(visibility = "public")]
    Anchor(Anchor),
    Endorsement(Endorsement),
    Collection(Collection),
    QualityScore(QualityScore),
    FeaturedContent(FeaturedContent),
}

#[hdk_link_types]
pub enum LinkTypes {
    PublicationToEndorsements,
    EndorserToEndorsements,
    CuratorToCollections,
    CollectionToPublications,
    PublicationToQuality,
    FeaturedPublications,
}

/// Genesis self-check
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback using FlatOp pattern
///
/// **P0 author-binding pass, 2026-07-09: this zome's coordinator has ZERO
/// `agent_info()` calls anywhere** (verified via grep — 0 hits across the
/// whole file), worse even than the usual case-(d) String-DID gap
/// documented for this cluster's attribution/bridge zomes. Every
/// "authorization" check in the coordinator (`add_to_collection`,
/// `remove_from_collection`, `update_collection`, `remove_endorsement`:
/// all compare `collection.curator_did`/`endorsement.endorser_did`
/// against a caller-supplied `requester_did` field) compares two
/// entirely attacker-controlled values — it authenticates nothing, the
/// same compare-two-attacker-controlled-values bug class as this
/// cluster's attribution/verify_attribution and mycelix-identity's
/// trust_credential fulfill/decline_attestation. None of this is
/// fixable at the integrity layer without inventing an unverified DID
/// convention, so it is disclosed, not silently patched.
///
/// What IS fixed: the wide-open RegisterUpdate/RegisterDelete bug (36th
/// confirmed instance this pass). Real content-restriction added to
/// Collection updates (comparing against the original via must_get,
/// which this validator never did before) — restricted to
/// name/description/visibility/publication_ids/updated, the only fields
/// any of the three Collection-updating coordinator functions ever
/// change. QualityScore and FeaturedContent have NO live update path at
/// all (confirmed via grep for `update_entry` — QualityScore is always
/// recreated fresh by `calculate_quality_score`; FeaturedContent is only
/// ever deleted by `unfeature_content`, never updated) so both are now
/// made immutable, replacing a previously-accepted but dead-code update
/// path for each.
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Endorsement(endorsement) => {
                    validate_create_endorsement(EntryCreationAction::Create(action), endorsement)
                }
                EntryTypes::Collection(collection) => {
                    validate_create_collection(EntryCreationAction::Create(action), collection)
                }
                EntryTypes::QualityScore(score) => {
                    validate_create_quality_score(EntryCreationAction::Create(action), score)
                }
                EntryTypes::FeaturedContent(featured) => {
                    validate_create_featured_content(EntryCreationAction::Create(action), featured)
                }
            },
            OpEntry::UpdateEntry {
                app_entry,
                original_action_hash,
                action,
                ..
            } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Endorsement(_) => Ok(ValidateCallbackResult::Invalid(
                    "Endorsements cannot be updated".into(),
                )),
                EntryTypes::Collection(collection) => {
                    validate_update_collection(action, original_action_hash, collection)
                }
                EntryTypes::QualityScore(_) => Ok(ValidateCallbackResult::Invalid(
                    "Quality scores cannot be updated".into(),
                )),
                EntryTypes::FeaturedContent(_) => Ok(ValidateCallbackResult::Invalid(
                    "Featured content records cannot be updated".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, .. } => match link_type {
            LinkTypes::PublicationToEndorsements => Ok(ValidateCallbackResult::Valid),
            LinkTypes::EndorserToEndorsements => Ok(ValidateCallbackResult::Valid),
            LinkTypes::CuratorToCollections => Ok(ValidateCallbackResult::Valid),
            LinkTypes::CollectionToPublications => Ok(ValidateCallbackResult::Valid),
            LinkTypes::PublicationToQuality => Ok(ValidateCallbackResult::Valid),
            LinkTypes::FeaturedPublications => Ok(ValidateCallbackResult::Valid),
        },
        // Deliberately left permissive: the coordinator never calls
        // delete_link. Reviewed 2026-07-09 during the P0 author-binding pass.
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // Previously fully permissive (`Ok(Valid)` unconditionally) --
            // the 36th confirmed instance of this exact bug pattern this
            // pass. Found + fixed 2026-07-09.
            OpUpdate::Entry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Endorsement(_) => Ok(ValidateCallbackResult::Invalid(
                    "Endorsements cannot be updated".into(),
                )),
                EntryTypes::Collection(collection) => validate_update_collection(
                    action.clone(),
                    action.original_action_address,
                    collection,
                ),
                EntryTypes::QualityScore(_) => Ok(ValidateCallbackResult::Invalid(
                    "Quality scores cannot be updated".into(),
                )),
                EntryTypes::FeaturedContent(_) => Ok(ValidateCallbackResult::Invalid(
                    "Featured content records cannot be updated".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(OpDelete { action }) => {
            let _ = must_get_action(action.deletes_address.clone())?;
            // No author field exists anywhere in this zome's entries to
            // compare against (case d), and the coordinator's own delete
            // checks are already unauthenticated (see module doc comment
            // above) -- deletion authorization is left as-is.
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

fn validate_create_endorsement(
    _action: EntryCreationAction,
    endorsement: Endorsement,
) -> ExternResult<ValidateCallbackResult> {
    if !endorsement.endorser_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Endorser must be a valid DID".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_collection(
    _action: EntryCreationAction,
    collection: Collection,
) -> ExternResult<ValidateCallbackResult> {
    if !collection.curator_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Curator must be a valid DID".into(),
        ));
    }
    if collection.name.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Collection name required".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_collection(
    _action: Update,
    original_action_hash: ActionHash,
    collection: Collection,
) -> ExternResult<ValidateCallbackResult> {
    if collection.name.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Collection name required".into(),
        ));
    }

    let original_record = must_get_valid_record(original_action_hash)?;
    let original: Collection = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original collection not found".into()
        )))?;

    if collection.id != original.id
        || collection.curator_did != original.curator_did
        || collection.created != original.created
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only name/description/visibility/publication_ids/updated can change on a collection update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_quality_score(
    _action: EntryCreationAction,
    score: QualityScore,
) -> ExternResult<ValidateCallbackResult> {
    if score.score < 0.0 || score.score > 1.0 {
        return Ok(ValidateCallbackResult::Invalid("Score must be 0-1".into()));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_featured_content(
    _action: EntryCreationAction,
    featured: FeaturedContent,
) -> ExternResult<ValidateCallbackResult> {
    if !featured.featured_by.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Featured by must be a valid DID".into(),
        ));
    }
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

    #[test]
    fn endorsement_requires_did_prefix() {
        let endorsement = Endorsement {
            id: "e-1".into(),
            publication_id: "pub-1".into(),
            endorser_did: "not-a-did".into(),
            endorsement_type: EndorsementType::Upvote,
            comment: None,
            created: Timestamp::from_micros(0),
        };
        let result = validate_create_endorsement(dummy_action(), endorsement).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn collection_requires_did_prefix_and_name() {
        let collection = Collection {
            id: "c-1".into(),
            name: "".into(),
            description: "".into(),
            curator_did: "did:key:z6Mkfoo".into(),
            visibility: Visibility::Public,
            publication_ids: vec![],
            created: Timestamp::from_micros(0),
            updated: Timestamp::from_micros(0),
        };
        let result = validate_create_collection(dummy_action(), collection).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn quality_score_rejects_out_of_range() {
        let score = QualityScore {
            publication_id: "pub-1".into(),
            score: 1.5,
            endorsement_count: 0,
            share_count: 0,
            fact_check_score: 0.5,
            author_reputation: 0.5,
            last_calculated: Timestamp::from_micros(0),
        };
        let result = validate_create_quality_score(dummy_action(), score).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn featured_content_requires_did_prefix() {
        let featured = FeaturedContent {
            id: "f-1".into(),
            publication_id: "pub-1".into(),
            featured_by: "not-a-did".into(),
            reason: "quality".into(),
            featured_from: Timestamp::from_micros(0),
            featured_until: None,
        };
        let result = validate_create_featured_content(dummy_action(), featured).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // validate_update_collection calls must_get_valid_record, which
    // requires a live HDI host and can't run in a plain unit test --
    // matching the established pattern from every other zome's update
    // validator this pass.
}
