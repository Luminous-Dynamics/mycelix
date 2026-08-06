// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Curation Integrity Zome
//! Updated to use HDI 0.7 patterns with FlatOp validation
use hdi::prelude::*;
use mycelix_bridge_entry_types::{check_author_match, check_link_author_match};

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

/// A curated gallery of visual art or mixed-media publications.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Gallery {
    pub id: String,
    pub name: String,
    pub description: String,
    pub curator_did: String,
    pub theme: Option<String>,
    pub publication_ids: Vec<String>,
    pub created: Timestamp,
    pub updated: Timestamp,
}

/// A time-bounded exhibition within a gallery.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Exhibition {
    pub id: String,
    pub name: String,
    pub description: String,
    pub gallery_id: String,
    pub curator_did: String,
    pub featured_publication_ids: Vec<String>,
    pub start_date: Timestamp,
    pub end_date: Option<Timestamp>,
    pub created: Timestamp,
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
    Gallery(Gallery),
    Exhibition(Exhibition),
}

#[hdk_link_types]
pub enum LinkTypes {
    PublicationToEndorsements,
    EndorserToEndorsements,
    CuratorToCollections,
    CollectionToPublications,
    PublicationToQuality,
    FeaturedPublications,
    CuratorToGalleries,
    GalleryToPublications,
    GalleryToExhibitions,
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
                EntryTypes::Gallery(gallery) => validate_gallery(&gallery),
                EntryTypes::Exhibition(exhibition) => validate_exhibition(&exhibition),
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Endorsement(_) => Ok(ValidateCallbackResult::Invalid(
                    "Endorsements cannot be updated".into(),
                )),
                EntryTypes::Collection(collection) => {
                    validate_update_collection(action, collection)
                }
                EntryTypes::QualityScore(score) => validate_update_quality_score(action, score),
                EntryTypes::FeaturedContent(featured) => {
                    validate_update_featured_content(action, featured)
                }
                EntryTypes::Gallery(gallery) => validate_update_gallery(action, gallery),
                // No live update_entry call for Exhibition (confirmed via
                // grep) -- previously silently accepted any field change.
                // Made explicitly immutable.
                EntryTypes::Exhibition(_) => Ok(ValidateCallbackResult::Invalid(
                    "Exhibitions are immutable".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address: _,
            target_address: _,
            tag,
            action: _,
        } => {
            let tag_len = tag.0.len();
            match link_type {
                LinkTypes::PublicationToEndorsements
                | LinkTypes::EndorserToEndorsements
                | LinkTypes::CuratorToCollections
                | LinkTypes::CollectionToPublications
                | LinkTypes::PublicationToQuality
                | LinkTypes::FeaturedPublications
                | LinkTypes::CuratorToGalleries
                | LinkTypes::GalleryToPublications
                | LinkTypes::GalleryToExhibitions => {
                    if tag_len > 256 {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Link tag too long (max 256 bytes)".into(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
            }
        }
        FlatOp::RegisterDeleteLink { tag, action, .. } => {
            let original_action = must_get_action(action.link_add_address.clone())?;
            let result = check_link_author_match(original_action.action().author(), &action.author);
            if result != ValidateCallbackResult::Valid {
                return Ok(result);
            }
            if tag.0.len() > 256 {
                return Ok(ValidateCallbackResult::Invalid(
                    "Delete link tag too long (max 256 bytes)".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(update) => {
            let action = match &update {
                OpUpdate::Entry { action, .. }
                | OpUpdate::PrivateEntry { action, .. }
                | OpUpdate::Agent { action, .. }
                | OpUpdate::CapClaim { action, .. }
                | OpUpdate::CapGrant { action, .. } => action,
            };
            let original = must_get_action(action.original_action_address.clone())?;
            Ok(check_author_match(
                original.action().author(),
                &action.author,
                "update",
            ))
        }
        FlatOp::RegisterDelete(OpDelete { action, .. }) => {
            let original = must_get_action(action.deletes_address.clone())?;
            Ok(check_author_match(
                original.action().author(),
                &action.author,
                "delete",
            ))
        }
    }
}

fn validate_create_endorsement(
    _action: EntryCreationAction,
    endorsement: Endorsement,
) -> ExternResult<ValidateCallbackResult> {
    if endorsement.id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Endorsement ID cannot be empty".into(),
        ));
    }
    if endorsement.id.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Endorsement ID too long (max 256 chars)".into(),
        ));
    }
    if endorsement.publication_id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Endorsement publication_id cannot be empty".into(),
        ));
    }
    if endorsement.publication_id.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Endorsement publication_id too long (max 256 chars)".into(),
        ));
    }
    if !endorsement.endorser_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Endorser must be a valid DID".into(),
        ));
    }
    if endorsement.endorser_did.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Endorser DID too long (max 256 chars)".into(),
        ));
    }
    if let Some(ref comment) = endorsement.comment {
        if comment.len() > 4096 {
            return Ok(ValidateCallbackResult::Invalid(
                "Comment too long (max 4096 chars)".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_collection(
    _action: EntryCreationAction,
    collection: Collection,
) -> ExternResult<ValidateCallbackResult> {
    if collection.id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Collection ID cannot be empty".into(),
        ));
    }
    if collection.id.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Collection ID too long (max 256 chars)".into(),
        ));
    }
    if !collection.curator_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Curator must be a valid DID".into(),
        ));
    }
    if collection.curator_did.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Curator DID too long (max 256 chars)".into(),
        ));
    }
    if collection.name.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Collection name required".into(),
        ));
    }
    if collection.name.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Collection name too long (max 256 chars)".into(),
        ));
    }
    if collection.description.len() > 4096 {
        return Ok(ValidateCallbackResult::Invalid(
            "Collection description too long (max 4096 chars)".into(),
        ));
    }
    if collection.publication_ids.len() > 100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Too many publication IDs (max 100)".into(),
        ));
    }
    for pid in &collection.publication_ids {
        if pid.len() > 256 {
            return Ok(ValidateCallbackResult::Invalid(
                "Publication ID entry too long (max 256 chars)".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Content restricted to name/description/visibility/publication_ids/
/// updated, the only fields any of this zome's Collection-updating
/// coordinator functions (add_to_collection/remove_from_collection/
/// update_collection) ever change. No author requirement: none of those
/// three has any real caller-identity verification either (see the
/// module doc comment on `validate` -- update_collection compares
/// collection.curator_did against a caller-supplied requester_did, both
/// attacker-controlled).
fn validate_collection_update_fields(collection: &Collection) -> ValidateCallbackResult {
    if collection.name.trim().is_empty() {
        return ValidateCallbackResult::Invalid("Collection name required".into());
    }
    ValidateCallbackResult::Valid
}

fn validate_update_collection(
    action: Update,
    collection: Collection,
) -> ExternResult<ValidateCallbackResult> {
    if let ValidateCallbackResult::Invalid(msg) = validate_collection_update_fields(&collection) {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: Collection = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original collection not found".to_string()
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

fn validate_quality_score_fields(score: &QualityScore) -> ValidateCallbackResult {
    if score.publication_id.trim().is_empty() {
        return ValidateCallbackResult::Invalid(
            "QualityScore publication_id cannot be empty".into(),
        );
    }
    if !score.score.is_finite() {
        return ValidateCallbackResult::Invalid("score must be a finite number".into());
    }
    if score.score < 0.0 || score.score > 1.0 {
        return ValidateCallbackResult::Invalid("Score must be 0-1".into());
    }
    if !score.fact_check_score.is_finite() {
        return ValidateCallbackResult::Invalid("fact_check_score must be a finite number".into());
    }
    if score.fact_check_score < 0.0 || score.fact_check_score > 1.0 {
        return ValidateCallbackResult::Invalid("Fact check score must be 0-1".into());
    }
    if !score.author_reputation.is_finite() {
        return ValidateCallbackResult::Invalid("author_reputation must be a finite number".into());
    }
    if score.author_reputation < 0.0 || score.author_reputation > 1.0 {
        return ValidateCallbackResult::Invalid("Author reputation must be 0-1".into());
    }
    ValidateCallbackResult::Valid
}

fn validate_create_quality_score(
    _action: EntryCreationAction,
    score: QualityScore,
) -> ExternResult<ValidateCallbackResult> {
    Ok(validate_quality_score_fields(&score))
}

/// Unlike the standalone reference (`mycelix-media/zomes/curation`, which
/// confirmed no live update_entry call exists for QualityScore and made
/// it immutable), this shadow zome's coordinator has a real, if entirely
/// ungated, `update_quality_score` extern that replaces the WHOLE entry
/// (`update_entry(original_action_hash, updated_entry)`, no
/// `require_civic` call at all). Freezing content on update would make
/// that real coordinator flow non-functional -- so update reuses full
/// create-shaped content validation instead of immutability. This is a
/// deliberate divergence from standalone, not a missed port; the
/// coordinator's own lack of an authorization check on this extern is a
/// separate, pre-existing gap, not fixable at the integrity layer without
/// inventing an authority model.
fn validate_update_quality_score(
    _action: Update,
    score: QualityScore,
) -> ExternResult<ValidateCallbackResult> {
    Ok(validate_quality_score_fields(&score))
}

fn validate_featured_content_fields(featured: &FeaturedContent) -> ValidateCallbackResult {
    if featured.id.trim().is_empty() {
        return ValidateCallbackResult::Invalid("FeaturedContent ID cannot be empty".into());
    }
    if featured.id.len() > 256 {
        return ValidateCallbackResult::Invalid(
            "FeaturedContent ID too long (max 256 chars)".into(),
        );
    }
    if featured.publication_id.trim().is_empty() {
        return ValidateCallbackResult::Invalid(
            "FeaturedContent publication_id cannot be empty".into(),
        );
    }
    if featured.publication_id.len() > 256 {
        return ValidateCallbackResult::Invalid(
            "FeaturedContent publication_id too long (max 256 chars)".into(),
        );
    }
    if !featured.featured_by.starts_with("did:") {
        return ValidateCallbackResult::Invalid("Featured by must be a valid DID".into());
    }
    if featured.featured_by.len() > 256 {
        return ValidateCallbackResult::Invalid("Featured by DID too long (max 256 chars)".into());
    }
    if featured.reason.trim().is_empty() {
        return ValidateCallbackResult::Invalid("Featured reason cannot be empty".into());
    }
    if featured.reason.len() > 4096 {
        return ValidateCallbackResult::Invalid("Featured reason too long (max 4096 chars)".into());
    }
    ValidateCallbackResult::Valid
}

fn validate_create_featured_content(
    _action: EntryCreationAction,
    featured: FeaturedContent,
) -> ExternResult<ValidateCallbackResult> {
    Ok(validate_featured_content_fields(&featured))
}

/// Unlike the standalone reference (which confirmed FeaturedContent is
/// only ever deleted, never updated, and made it immutable), this shadow
/// zome's coordinator has a real, entirely ungated `update_featured_content`
/// extern that replaces the WHOLE entry. Freezing content on update would
/// make that real flow non-functional -- so update reuses full
/// create-shaped content validation instead of immutability. Deliberate
/// divergence from standalone, not a missed port; the coordinator's own
/// lack of an authorization check on this extern is a separate,
/// pre-existing gap.
fn validate_update_featured_content(
    _action: Update,
    featured: FeaturedContent,
) -> ExternResult<ValidateCallbackResult> {
    Ok(validate_featured_content_fields(&featured))
}

fn validate_gallery(gallery: &Gallery) -> ExternResult<ValidateCallbackResult> {
    if gallery.name.is_empty() || gallery.name.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Gallery name must be 1-256 chars".to_string(),
        ));
    }
    if gallery.description.len() > 4096 {
        return Ok(ValidateCallbackResult::Invalid(
            "Gallery description must be <= 4KB".to_string(),
        ));
    }
    if !gallery.curator_did.starts_with("did:") || gallery.curator_did.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "curator_did must be a valid DID".to_string(),
        ));
    }
    if gallery.publication_ids.len() > 500 {
        return Ok(ValidateCallbackResult::Invalid(
            "Gallery may contain at most 500 publications".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Content restricted to publication_ids/updated, the only fields
/// add_to_gallery (the sole coordinator function that calls update_entry
/// on Gallery) ever changes.
fn validate_update_gallery(
    action: Update,
    gallery: Gallery,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: Gallery = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original gallery not found".to_string()
        )))?;
    if gallery.id != original.id
        || gallery.name != original.name
        || gallery.description != original.description
        || gallery.curator_did != original.curator_did
        || gallery.theme != original.theme
        || gallery.created != original.created
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only publication_ids/updated can change on a gallery update".into(),
        ));
    }
    validate_gallery(&gallery)
}

fn validate_exhibition(exhibition: &Exhibition) -> ExternResult<ValidateCallbackResult> {
    if exhibition.name.is_empty() || exhibition.name.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Exhibition name must be 1-256 chars".to_string(),
        ));
    }
    if exhibition.description.len() > 4096 {
        return Ok(ValidateCallbackResult::Invalid(
            "Exhibition description must be <= 4KB".to_string(),
        ));
    }
    if !exhibition.curator_did.starts_with("did:") || exhibition.curator_did.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "curator_did must be a valid DID".to_string(),
        ));
    }
    if exhibition.featured_publication_ids.len() > 100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Exhibition may feature at most 100 publications".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    // ========================================================================
    // RESULT HELPERS
    // ========================================================================

    fn is_valid(result: &ExternResult<ValidateCallbackResult>) -> bool {
        matches!(result, Ok(ValidateCallbackResult::Valid))
    }

    fn is_invalid(result: &ExternResult<ValidateCallbackResult>) -> bool {
        matches!(result, Ok(ValidateCallbackResult::Invalid(_)))
    }

    fn invalid_msg(result: &ExternResult<ValidateCallbackResult>) -> String {
        match result {
            Ok(ValidateCallbackResult::Invalid(msg)) => msg.clone(),
            _ => panic!("Expected Invalid, got {:?}", result),
        }
    }

    // ========================================================================
    // CONSTRUCTION HELPERS
    // ========================================================================

    fn fake_create() -> Create {
        Create {
            author: AgentPubKey::from_raw_36(vec![0u8; 36]),
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex(0),
                ZomeIndex(0),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: EntryRateWeight::default(),
        }
    }

    fn fake_entry_creation_action() -> EntryCreationAction {
        EntryCreationAction::Create(fake_create())
    }

    fn fake_update() -> Update {
        Update {
            author: AgentPubKey::from_raw_36(vec![0u8; 36]),
            timestamp: Timestamp::from_micros(0),
            action_seq: 1,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            original_action_address: ActionHash::from_raw_36(vec![0u8; 36]),
            original_entry_address: EntryHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex(0),
                ZomeIndex(0),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: EntryRateWeight::default(),
        }
    }

    fn ts() -> Timestamp {
        Timestamp::from_micros(0)
    }

    fn make_endorsement() -> Endorsement {
        Endorsement {
            id: "end-1".into(),
            publication_id: "pub-1".into(),
            endorser_did: "did:example:endorser".into(),
            endorsement_type: EndorsementType::Upvote,
            comment: Some("Great article".into()),
            created: ts(),
        }
    }

    fn make_collection() -> Collection {
        Collection {
            id: "col-1".into(),
            name: "Best Articles".into(),
            description: "A curated collection".into(),
            curator_did: "did:example:curator".into(),
            visibility: Visibility::Public,
            publication_ids: vec!["pub-1".into(), "pub-2".into()],
            created: ts(),
            updated: ts(),
        }
    }

    fn make_quality_score() -> QualityScore {
        QualityScore {
            publication_id: "pub-1".into(),
            score: 0.85,
            endorsement_count: 10,
            share_count: 5,
            fact_check_score: 0.9,
            author_reputation: 0.75,
            last_calculated: ts(),
        }
    }

    fn make_featured_content() -> FeaturedContent {
        FeaturedContent {
            id: "feat-1".into(),
            publication_id: "pub-1".into(),
            featured_by: "did:example:editor".into(),
            reason: "Outstanding investigative journalism".into(),
            featured_from: ts(),
            featured_until: None,
        }
    }

    // ========================================================================
    // ENDORSEMENT CREATE VALIDATION TESTS
    // ========================================================================

    #[test]
    fn valid_endorsement_passes() {
        let result = validate_create_endorsement(fake_entry_creation_action(), make_endorsement());
        assert!(is_valid(&result));
    }

    #[test]
    fn endorsement_endorser_did_not_did_rejected() {
        let mut e = make_endorsement();
        e.endorser_did = "not-a-did".into();
        let result = validate_create_endorsement(fake_entry_creation_action(), e);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Endorser must be a valid DID");
    }

    #[test]
    fn endorsement_endorser_did_empty_rejected() {
        let mut e = make_endorsement();
        e.endorser_did = "".into();
        let result = validate_create_endorsement(fake_entry_creation_action(), e);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Endorser must be a valid DID");
    }

    #[test]
    fn endorsement_endorser_did_prefix_only_passes() {
        // "did:" alone passes the starts_with check
        let mut e = make_endorsement();
        e.endorser_did = "did:".into();
        let result = validate_create_endorsement(fake_entry_creation_action(), e);
        assert!(is_valid(&result));
    }

    #[test]
    fn endorsement_endorser_did_case_sensitive() {
        // "DID:" does not pass starts_with("did:")
        let mut e = make_endorsement();
        e.endorser_did = "DID:example:foo".into();
        let result = validate_create_endorsement(fake_entry_creation_action(), e);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Endorser must be a valid DID");
    }

    #[test]
    fn endorsement_every_type_variant_passes() {
        let variants = vec![
            EndorsementType::Upvote,
            EndorsementType::Bookmark,
            EndorsementType::Share,
            EndorsementType::Recommend,
            EndorsementType::Award("Best of 2026".into()),
        ];
        for variant in variants {
            let mut e = make_endorsement();
            e.endorsement_type = variant;
            let result = validate_create_endorsement(fake_entry_creation_action(), e);
            assert!(is_valid(&result));
        }
    }

    #[test]
    fn endorsement_with_no_comment_passes() {
        let mut e = make_endorsement();
        e.comment = None;
        let result = validate_create_endorsement(fake_entry_creation_action(), e);
        assert!(is_valid(&result));
    }

    #[test]
    fn endorsement_with_empty_comment_passes() {
        let mut e = make_endorsement();
        e.comment = Some("".into());
        let result = validate_create_endorsement(fake_entry_creation_action(), e);
        assert!(is_valid(&result));
    }

    #[test]
    fn endorsement_with_empty_id_rejected() {
        let mut e = make_endorsement();
        e.id = "".into();
        let result = validate_create_endorsement(fake_entry_creation_action(), e);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Endorsement ID cannot be empty");
    }

    #[test]
    fn endorsement_with_whitespace_id_rejected() {
        let mut e = make_endorsement();
        e.id = "   ".into();
        let result = validate_create_endorsement(fake_entry_creation_action(), e);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Endorsement ID cannot be empty");
    }

    #[test]
    fn endorsement_with_empty_publication_id_rejected() {
        let mut e = make_endorsement();
        e.publication_id = "".into();
        let result = validate_create_endorsement(fake_entry_creation_action(), e);
        assert!(is_invalid(&result));
        assert_eq!(
            invalid_msg(&result),
            "Endorsement publication_id cannot be empty"
        );
    }

    #[test]
    fn endorsement_with_whitespace_publication_id_rejected() {
        let mut e = make_endorsement();
        e.publication_id = "  \t ".into();
        let result = validate_create_endorsement(fake_entry_creation_action(), e);
        assert!(is_invalid(&result));
        assert_eq!(
            invalid_msg(&result),
            "Endorsement publication_id cannot be empty"
        );
    }

    // ========================================================================
    // COLLECTION CREATE VALIDATION TESTS
    // ========================================================================

    #[test]
    fn valid_collection_passes() {
        let result = validate_create_collection(fake_entry_creation_action(), make_collection());
        assert!(is_valid(&result));
    }

    #[test]
    fn collection_curator_did_not_did_rejected() {
        let mut c = make_collection();
        c.curator_did = "not-a-did".into();
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Curator must be a valid DID");
    }

    #[test]
    fn collection_curator_did_empty_rejected() {
        let mut c = make_collection();
        c.curator_did = "".into();
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Curator must be a valid DID");
    }

    #[test]
    fn collection_curator_did_prefix_only_passes() {
        let mut c = make_collection();
        c.curator_did = "did:".into();
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_valid(&result));
    }

    #[test]
    fn collection_curator_did_case_sensitive() {
        let mut c = make_collection();
        c.curator_did = "DID:example:curator".into();
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Curator must be a valid DID");
    }

    #[test]
    fn collection_name_empty_rejected() {
        let mut c = make_collection();
        c.name = "".into();
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Collection name required");
    }

    #[test]
    fn collection_name_whitespace_rejected() {
        let mut c = make_collection();
        c.name = "   ".into();
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_invalid(&result));
    }

    #[test]
    fn collection_multiple_failures_did_wins() {
        // Both curator_did and name invalid; DID check comes first
        let mut c = make_collection();
        c.curator_did = "bad".into();
        c.name = "".into();
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Curator must be a valid DID");
    }

    #[test]
    fn collection_bad_did_with_valid_name_rejected_on_did() {
        let mut c = make_collection();
        c.curator_did = "bad".into();
        c.name = "Valid Name".into();
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Curator must be a valid DID");
    }

    #[test]
    fn collection_valid_did_with_empty_name_rejected_on_name() {
        let mut c = make_collection();
        c.curator_did = "did:example:curator".into();
        c.name = "".into();
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Collection name required");
    }

    #[test]
    fn collection_empty_description_passes() {
        let mut c = make_collection();
        c.description = "".into();
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_valid(&result));
    }

    #[test]
    fn collection_empty_publication_ids_passes() {
        let mut c = make_collection();
        c.publication_ids = vec![];
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_valid(&result));
    }

    #[test]
    fn collection_every_visibility_variant_passes() {
        let variants = vec![
            Visibility::Public,
            Visibility::Private,
            Visibility::Unlisted,
        ];
        for v in variants {
            let mut c = make_collection();
            c.visibility = v;
            let result = validate_create_collection(fake_entry_creation_action(), c);
            assert!(is_valid(&result));
        }
    }

    // ========================================================================
    // COLLECTION UPDATE VALIDATION TESTS
    // ========================================================================

    // validate_update_collection now needs a live HDI (must_get_valid_record
    // has no test mock in this codebase), so these exercise the extracted
    // pure validate_collection_update_fields helper directly.

    #[test]
    fn update_collection_fields_valid_passes() {
        let result = validate_collection_update_fields(&make_collection());
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn update_collection_fields_name_empty_rejected() {
        let mut c = make_collection();
        c.name = "".into();
        let result = validate_collection_update_fields(&c);
        match result {
            ValidateCallbackResult::Invalid(msg) => {
                assert_eq!(msg, "Collection name required");
            }
            _ => panic!("Expected Invalid result"),
        }
    }

    #[test]
    fn update_collection_fields_name_whitespace_rejected() {
        let mut c = make_collection();
        c.name = "   ".into();
        let result = validate_collection_update_fields(&c);
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_collection_fields_does_not_check_curator_did() {
        // validate_collection_update_fields only checks name; curator_did
        // immutability is enforced separately by validate_update_collection
        // (via must_get_valid_record, not unit-testable here).
        let mut c = make_collection();
        c.curator_did = "not-a-did".into();
        let result = validate_collection_update_fields(&c);
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn update_collection_fields_empty_description_passes() {
        let mut c = make_collection();
        c.description = "".into();
        let result = validate_collection_update_fields(&c);
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    // ========================================================================
    // QUALITY SCORE CREATE VALIDATION TESTS
    // ========================================================================

    #[test]
    fn valid_quality_score_passes() {
        let result =
            validate_create_quality_score(fake_entry_creation_action(), make_quality_score());
        assert!(is_valid(&result));
    }

    #[test]
    fn quality_score_below_zero_rejected() {
        let mut qs = make_quality_score();
        qs.score = -0.01;
        let result = validate_create_quality_score(fake_entry_creation_action(), qs);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Score must be 0-1");
    }

    #[test]
    fn quality_score_above_one_rejected() {
        let mut qs = make_quality_score();
        qs.score = 1.01;
        let result = validate_create_quality_score(fake_entry_creation_action(), qs);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Score must be 0-1");
    }

    #[test]
    fn quality_score_exactly_zero_passes() {
        let mut qs = make_quality_score();
        qs.score = 0.0;
        let result = validate_create_quality_score(fake_entry_creation_action(), qs);
        assert!(is_valid(&result));
    }

    #[test]
    fn quality_score_exactly_one_passes() {
        let mut qs = make_quality_score();
        qs.score = 1.0;
        let result = validate_create_quality_score(fake_entry_creation_action(), qs);
        assert!(is_valid(&result));
    }

    #[test]
    fn quality_score_midpoint_passes() {
        let mut qs = make_quality_score();
        qs.score = 0.5;
        let result = validate_create_quality_score(fake_entry_creation_action(), qs);
        assert!(is_valid(&result));
    }

    #[test]
    fn quality_score_negative_large_rejected() {
        let mut qs = make_quality_score();
        qs.score = -999.0;
        let result = validate_create_quality_score(fake_entry_creation_action(), qs);
        assert!(is_invalid(&result));
    }

    #[test]
    fn quality_score_large_positive_rejected() {
        let mut qs = make_quality_score();
        qs.score = 999.0;
        let result = validate_create_quality_score(fake_entry_creation_action(), qs);
        assert!(is_invalid(&result));
    }

    #[test]
    fn quality_score_fact_check_score_above_one_rejected() {
        let mut qs = make_quality_score();
        qs.fact_check_score = 1.01;
        let result = validate_create_quality_score(fake_entry_creation_action(), qs);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Fact check score must be 0-1");
    }

    #[test]
    fn quality_score_fact_check_score_below_zero_rejected() {
        let mut qs = make_quality_score();
        qs.fact_check_score = -0.01;
        let result = validate_create_quality_score(fake_entry_creation_action(), qs);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Fact check score must be 0-1");
    }

    #[test]
    fn quality_score_fact_check_score_boundary_passes() {
        for val in [0.0, 0.5, 1.0] {
            let mut qs = make_quality_score();
            qs.fact_check_score = val;
            let result = validate_create_quality_score(fake_entry_creation_action(), qs);
            assert!(
                is_valid(&result),
                "fact_check_score={} should be valid",
                val
            );
        }
    }

    #[test]
    fn quality_score_author_reputation_above_one_rejected() {
        let mut qs = make_quality_score();
        qs.author_reputation = 1.01;
        let result = validate_create_quality_score(fake_entry_creation_action(), qs);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Author reputation must be 0-1");
    }

    #[test]
    fn quality_score_author_reputation_below_zero_rejected() {
        let mut qs = make_quality_score();
        qs.author_reputation = -5.0;
        let result = validate_create_quality_score(fake_entry_creation_action(), qs);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Author reputation must be 0-1");
    }

    #[test]
    fn quality_score_author_reputation_boundary_passes() {
        for val in [0.0, 0.5, 1.0] {
            let mut qs = make_quality_score();
            qs.author_reputation = val;
            let result = validate_create_quality_score(fake_entry_creation_action(), qs);
            assert!(
                is_valid(&result),
                "author_reputation={} should be valid",
                val
            );
        }
    }

    #[test]
    fn quality_score_empty_publication_id_rejected() {
        let mut qs = make_quality_score();
        qs.publication_id = "".into();
        let result = validate_create_quality_score(fake_entry_creation_action(), qs);
        assert!(is_invalid(&result));
        assert_eq!(
            invalid_msg(&result),
            "QualityScore publication_id cannot be empty"
        );
    }

    #[test]
    fn quality_score_whitespace_publication_id_rejected() {
        let mut qs = make_quality_score();
        qs.publication_id = "   ".into();
        let result = validate_create_quality_score(fake_entry_creation_action(), qs);
        assert!(is_invalid(&result));
        assert_eq!(
            invalid_msg(&result),
            "QualityScore publication_id cannot be empty"
        );
    }

    // ========================================================================
    // QUALITY SCORE UPDATE VALIDATION TESTS
    // ========================================================================

    #[test]
    fn update_quality_score_valid_passes() {
        let result = validate_update_quality_score(fake_update(), make_quality_score());
        assert!(is_valid(&result));
    }

    #[test]
    fn update_quality_score_below_zero_rejected() {
        let mut qs = make_quality_score();
        qs.score = -0.01;
        let result = validate_update_quality_score(fake_update(), qs);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Score must be 0-1");
    }

    #[test]
    fn update_quality_score_above_one_rejected() {
        let mut qs = make_quality_score();
        qs.score = 1.01;
        let result = validate_update_quality_score(fake_update(), qs);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Score must be 0-1");
    }

    #[test]
    fn update_quality_score_boundary_zero_passes() {
        let mut qs = make_quality_score();
        qs.score = 0.0;
        let result = validate_update_quality_score(fake_update(), qs);
        assert!(is_valid(&result));
    }

    #[test]
    fn update_quality_score_boundary_one_passes() {
        let mut qs = make_quality_score();
        qs.score = 1.0;
        let result = validate_update_quality_score(fake_update(), qs);
        assert!(is_valid(&result));
    }

    // ========================================================================
    // COLLECTION STRING LENGTH LIMIT TESTS
    // ========================================================================

    #[test]
    fn collection_id_empty_rejected() {
        let mut c = make_collection();
        c.id = "".into();
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Collection ID cannot be empty");
    }

    #[test]
    fn collection_id_at_limit_passes() {
        let mut c = make_collection();
        c.id = "c".repeat(256);
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_valid(&result));
    }

    #[test]
    fn collection_id_over_limit_rejected() {
        let mut c = make_collection();
        c.id = "c".repeat(257);
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_invalid(&result));
        assert_eq!(
            invalid_msg(&result),
            "Collection ID too long (max 256 chars)"
        );
    }

    #[test]
    fn collection_name_at_limit_passes() {
        let mut c = make_collection();
        c.name = "n".repeat(256);
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_valid(&result));
    }

    #[test]
    fn collection_name_over_limit_rejected() {
        let mut c = make_collection();
        c.name = "n".repeat(257);
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_invalid(&result));
        assert_eq!(
            invalid_msg(&result),
            "Collection name too long (max 256 chars)"
        );
    }

    #[test]
    fn collection_description_at_limit_passes() {
        let mut c = make_collection();
        c.description = "d".repeat(4096);
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_valid(&result));
    }

    #[test]
    fn collection_description_over_limit_rejected() {
        let mut c = make_collection();
        c.description = "d".repeat(4097);
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_invalid(&result));
        assert_eq!(
            invalid_msg(&result),
            "Collection description too long (max 4096 chars)"
        );
    }

    #[test]
    fn collection_curator_did_at_limit_passes() {
        let mut c = make_collection();
        c.curator_did = format!("did:{}", "x".repeat(252));
        assert_eq!(c.curator_did.len(), 256);
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_valid(&result));
    }

    #[test]
    fn collection_curator_did_over_limit_rejected() {
        let mut c = make_collection();
        c.curator_did = format!("did:{}", "x".repeat(253));
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Curator DID too long (max 256 chars)");
    }

    #[test]
    fn collection_publication_ids_count_at_limit_passes() {
        let mut c = make_collection();
        c.publication_ids = (0..100).map(|i| format!("pub-{}", i)).collect();
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_valid(&result));
    }

    #[test]
    fn collection_publication_ids_count_over_limit_rejected() {
        let mut c = make_collection();
        c.publication_ids = (0..101).map(|i| format!("pub-{}", i)).collect();
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Too many publication IDs (max 100)");
    }

    #[test]
    fn collection_publication_id_item_at_limit_passes() {
        let mut c = make_collection();
        c.publication_ids = vec!["p".repeat(256)];
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_valid(&result));
    }

    #[test]
    fn collection_publication_id_item_over_limit_rejected() {
        let mut c = make_collection();
        c.publication_ids = vec!["p".repeat(257)];
        let result = validate_create_collection(fake_entry_creation_action(), c);
        assert!(is_invalid(&result));
        assert_eq!(
            invalid_msg(&result),
            "Publication ID entry too long (max 256 chars)"
        );
    }

    // ========================================================================
    // ENDORSEMENT STRING LENGTH LIMIT TESTS
    // ========================================================================

    #[test]
    fn endorsement_id_at_limit_passes() {
        let mut e = make_endorsement();
        e.id = "e".repeat(256);
        let result = validate_create_endorsement(fake_entry_creation_action(), e);
        assert!(is_valid(&result));
    }

    #[test]
    fn endorsement_id_over_limit_rejected() {
        let mut e = make_endorsement();
        e.id = "e".repeat(257);
        let result = validate_create_endorsement(fake_entry_creation_action(), e);
        assert!(is_invalid(&result));
        assert_eq!(
            invalid_msg(&result),
            "Endorsement ID too long (max 256 chars)"
        );
    }

    #[test]
    fn endorsement_publication_id_at_limit_passes() {
        let mut e = make_endorsement();
        e.publication_id = "p".repeat(256);
        let result = validate_create_endorsement(fake_entry_creation_action(), e);
        assert!(is_valid(&result));
    }

    #[test]
    fn endorsement_publication_id_over_limit_rejected() {
        let mut e = make_endorsement();
        e.publication_id = "p".repeat(257);
        let result = validate_create_endorsement(fake_entry_creation_action(), e);
        assert!(is_invalid(&result));
        assert_eq!(
            invalid_msg(&result),
            "Endorsement publication_id too long (max 256 chars)"
        );
    }

    #[test]
    fn endorsement_endorser_did_at_limit_passes() {
        let mut e = make_endorsement();
        e.endorser_did = format!("did:{}", "x".repeat(252));
        assert_eq!(e.endorser_did.len(), 256);
        let result = validate_create_endorsement(fake_entry_creation_action(), e);
        assert!(is_valid(&result));
    }

    #[test]
    fn endorsement_endorser_did_over_limit_rejected() {
        let mut e = make_endorsement();
        e.endorser_did = format!("did:{}", "x".repeat(253));
        let result = validate_create_endorsement(fake_entry_creation_action(), e);
        assert!(is_invalid(&result));
        assert_eq!(
            invalid_msg(&result),
            "Endorser DID too long (max 256 chars)"
        );
    }

    #[test]
    fn endorsement_comment_at_limit_passes() {
        let mut e = make_endorsement();
        e.comment = Some("c".repeat(4096));
        let result = validate_create_endorsement(fake_entry_creation_action(), e);
        assert!(is_valid(&result));
    }

    #[test]
    fn endorsement_comment_over_limit_rejected() {
        let mut e = make_endorsement();
        e.comment = Some("c".repeat(4097));
        let result = validate_create_endorsement(fake_entry_creation_action(), e);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Comment too long (max 4096 chars)");
    }

    // ========================================================================
    // FEATURED CONTENT CREATE VALIDATION TESTS
    // ========================================================================

    #[test]
    fn valid_featured_content_passes() {
        let result =
            validate_create_featured_content(fake_entry_creation_action(), make_featured_content());
        assert!(is_valid(&result));
    }

    #[test]
    fn featured_content_featured_by_not_did_rejected() {
        let mut f = make_featured_content();
        f.featured_by = "not-a-did".into();
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Featured by must be a valid DID");
    }

    #[test]
    fn featured_content_featured_by_empty_rejected() {
        let mut f = make_featured_content();
        f.featured_by = "".into();
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Featured by must be a valid DID");
    }

    #[test]
    fn featured_content_featured_by_prefix_only_passes() {
        let mut f = make_featured_content();
        f.featured_by = "did:".into();
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_valid(&result));
    }

    #[test]
    fn featured_content_featured_by_case_sensitive() {
        let mut f = make_featured_content();
        f.featured_by = "DID:example:editor".into();
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Featured by must be a valid DID");
    }

    #[test]
    fn featured_content_with_until_timestamp_passes() {
        let mut f = make_featured_content();
        f.featured_until = Some(Timestamp::from_micros(1_000_000));
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_valid(&result));
    }

    #[test]
    fn featured_content_empty_reason_rejected() {
        let mut f = make_featured_content();
        f.reason = "".into();
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Featured reason cannot be empty");
    }

    #[test]
    fn featured_content_whitespace_reason_rejected() {
        let mut f = make_featured_content();
        f.reason = "   ".into();
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Featured reason cannot be empty");
    }

    #[test]
    fn featured_content_empty_id_rejected() {
        let mut f = make_featured_content();
        f.id = "".into();
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "FeaturedContent ID cannot be empty");
    }

    #[test]
    fn featured_content_whitespace_id_rejected() {
        let mut f = make_featured_content();
        f.id = "  \t ".into();
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "FeaturedContent ID cannot be empty");
    }

    #[test]
    fn featured_content_empty_publication_id_rejected() {
        let mut f = make_featured_content();
        f.publication_id = "".into();
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_invalid(&result));
        assert_eq!(
            invalid_msg(&result),
            "FeaturedContent publication_id cannot be empty"
        );
    }

    #[test]
    fn featured_content_whitespace_publication_id_rejected() {
        let mut f = make_featured_content();
        f.publication_id = "   ".into();
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_invalid(&result));
        assert_eq!(
            invalid_msg(&result),
            "FeaturedContent publication_id cannot be empty"
        );
    }

    // ========================================================================
    // FEATURED CONTENT STRING LENGTH LIMIT TESTS
    // ========================================================================

    #[test]
    fn featured_content_id_at_limit_passes() {
        let mut f = make_featured_content();
        f.id = "f".repeat(256);
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_valid(&result));
    }

    #[test]
    fn featured_content_id_over_limit_rejected() {
        let mut f = make_featured_content();
        f.id = "f".repeat(257);
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_invalid(&result));
        assert_eq!(
            invalid_msg(&result),
            "FeaturedContent ID too long (max 256 chars)"
        );
    }

    #[test]
    fn featured_content_publication_id_at_limit_passes() {
        let mut f = make_featured_content();
        f.publication_id = "p".repeat(256);
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_valid(&result));
    }

    #[test]
    fn featured_content_publication_id_over_limit_rejected() {
        let mut f = make_featured_content();
        f.publication_id = "p".repeat(257);
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_invalid(&result));
        assert_eq!(
            invalid_msg(&result),
            "FeaturedContent publication_id too long (max 256 chars)"
        );
    }

    #[test]
    fn featured_content_featured_by_at_limit_passes() {
        let mut f = make_featured_content();
        f.featured_by = format!("did:{}", "x".repeat(252));
        assert_eq!(f.featured_by.len(), 256);
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_valid(&result));
    }

    #[test]
    fn featured_content_featured_by_over_limit_rejected() {
        let mut f = make_featured_content();
        f.featured_by = format!("did:{}", "x".repeat(253));
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_invalid(&result));
        assert_eq!(
            invalid_msg(&result),
            "Featured by DID too long (max 256 chars)"
        );
    }

    #[test]
    fn featured_content_reason_at_limit_passes() {
        let mut f = make_featured_content();
        f.reason = "r".repeat(4096);
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_valid(&result));
    }

    #[test]
    fn featured_content_reason_over_limit_rejected() {
        let mut f = make_featured_content();
        f.reason = "r".repeat(4097);
        let result = validate_create_featured_content(fake_entry_creation_action(), f);
        assert!(is_invalid(&result));
        assert_eq!(
            invalid_msg(&result),
            "Featured reason too long (max 4096 chars)"
        );
    }

    // ========================================================================
    // FEATURED CONTENT UPDATE VALIDATION TESTS
    // ========================================================================

    #[test]
    fn update_featured_content_valid_passes() {
        let result = validate_update_featured_content(fake_update(), make_featured_content());
        assert!(is_valid(&result));
    }

    #[test]
    fn update_featured_content_bad_did_rejected() {
        // Update now runs full content validation (not "always Valid"),
        // since this shadow zome's coordinator has an ungated
        // update_featured_content extern that can replace the whole
        // entry -- see the divergence-rationale doc comment on
        // validate_update_featured_content.
        let mut f = make_featured_content();
        f.featured_by = "not-a-did".into();
        let result = validate_update_featured_content(fake_update(), f);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Featured by must be a valid DID");
    }

    #[test]
    fn update_featured_content_empty_reason_rejected() {
        let mut f = make_featured_content();
        f.reason = "".into();
        let result = validate_update_featured_content(fake_update(), f);
        assert!(is_invalid(&result));
        assert_eq!(invalid_msg(&result), "Featured reason cannot be empty");
    }

    // ========================================================================
    // SERDE ROUNDTRIP TESTS
    // ========================================================================

    #[test]
    fn endorsement_serde_roundtrip() {
        let original = make_endorsement();
        let json = serde_json::to_string(&original).expect("serialize");
        let restored: Endorsement = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(original, restored);
    }

    #[test]
    fn collection_serde_roundtrip() {
        let original = make_collection();
        let json = serde_json::to_string(&original).expect("serialize");
        let restored: Collection = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(original, restored);
    }

    #[test]
    fn quality_score_serde_roundtrip() {
        let original = make_quality_score();
        let json = serde_json::to_string(&original).expect("serialize");
        let restored: QualityScore = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(original, restored);
    }

    #[test]
    fn featured_content_serde_roundtrip() {
        let original = make_featured_content();
        let json = serde_json::to_string(&original).expect("serialize");
        let restored: FeaturedContent = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(original, restored);
    }

    #[test]
    fn endorsement_type_serde_roundtrip() {
        let variants = vec![
            EndorsementType::Upvote,
            EndorsementType::Bookmark,
            EndorsementType::Share,
            EndorsementType::Recommend,
            EndorsementType::Award("Best Reporting".into()),
        ];
        for original in variants {
            let json = serde_json::to_string(&original).expect("serialize");
            let restored: EndorsementType = serde_json::from_str(&json).expect("deserialize");
            assert_eq!(original, restored);
        }
    }

    #[test]
    fn visibility_serde_roundtrip() {
        let variants = vec![
            Visibility::Public,
            Visibility::Private,
            Visibility::Unlisted,
        ];
        for original in variants {
            let json = serde_json::to_string(&original).expect("serialize");
            let restored: Visibility = serde_json::from_str(&json).expect("deserialize");
            assert_eq!(original, restored);
        }
    }

    #[test]
    fn anchor_serde_roundtrip() {
        let original = Anchor("publications".into());
        let json = serde_json::to_string(&original).expect("serialize");
        let restored: Anchor = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(original, restored);
    }

    #[test]
    fn endorsement_with_none_comment_serde_roundtrip() {
        let mut original = make_endorsement();
        original.comment = None;
        let json = serde_json::to_string(&original).expect("serialize");
        let restored: Endorsement = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(original, restored);
    }

    #[test]
    fn featured_content_with_until_serde_roundtrip() {
        let mut original = make_featured_content();
        original.featured_until = Some(Timestamp::from_micros(1_000_000));
        let json = serde_json::to_string(&original).expect("serialize");
        let restored: FeaturedContent = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(original, restored);
    }

    // ========================================================================
    // LINK TAG VALIDATION TESTS
    // ========================================================================

    fn validate_create_link_tag(link_type: &LinkTypes, tag: &LinkTag) -> ValidateCallbackResult {
        let tag_len = tag.0.len();
        match link_type {
            LinkTypes::PublicationToEndorsements
            | LinkTypes::EndorserToEndorsements
            | LinkTypes::CuratorToCollections
            | LinkTypes::CollectionToPublications
            | LinkTypes::PublicationToQuality
            | LinkTypes::FeaturedPublications
            | LinkTypes::CuratorToGalleries
            | LinkTypes::GalleryToPublications
            | LinkTypes::GalleryToExhibitions => {
                if tag_len > 256 {
                    ValidateCallbackResult::Invalid("Link tag too long (max 256 bytes)".into())
                } else {
                    ValidateCallbackResult::Valid
                }
            }
        }
    }

    fn validate_delete_link_tag(tag: &LinkTag) -> ValidateCallbackResult {
        if tag.0.len() > 256 {
            ValidateCallbackResult::Invalid("Delete link tag too long (max 256 bytes)".into())
        } else {
            ValidateCallbackResult::Valid
        }
    }

    // -- PublicationToEndorsements (256-byte limit) boundary tests --

    #[test]
    fn link_tag_pub_to_endorsements_empty_valid() {
        let tag = LinkTag::new(vec![]);
        let result = validate_create_link_tag(&LinkTypes::PublicationToEndorsements, &tag);
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn link_tag_pub_to_endorsements_at_limit_valid() {
        let tag = LinkTag::new(vec![0u8; 256]);
        let result = validate_create_link_tag(&LinkTypes::PublicationToEndorsements, &tag);
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn link_tag_pub_to_endorsements_over_limit_invalid() {
        let tag = LinkTag::new(vec![0u8; 257]);
        let result = validate_create_link_tag(&LinkTypes::PublicationToEndorsements, &tag);
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // -- FeaturedPublications (256-byte limit) boundary tests --

    #[test]
    fn link_tag_featured_at_limit_valid() {
        let tag = LinkTag::new(vec![0xBB; 256]);
        let result = validate_create_link_tag(&LinkTypes::FeaturedPublications, &tag);
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn link_tag_featured_over_limit_invalid() {
        let tag = LinkTag::new(vec![0xBB; 257]);
        let result = validate_create_link_tag(&LinkTypes::FeaturedPublications, &tag);
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // -- DoS prevention: massive tags rejected for all link types --

    #[test]
    fn link_tag_dos_prevention_all_types() {
        let massive_tag = LinkTag::new(vec![0xFF; 10_000]);
        let all_types = [
            LinkTypes::PublicationToEndorsements,
            LinkTypes::EndorserToEndorsements,
            LinkTypes::CuratorToCollections,
            LinkTypes::CollectionToPublications,
            LinkTypes::PublicationToQuality,
            LinkTypes::FeaturedPublications,
            LinkTypes::CuratorToGalleries,
            LinkTypes::GalleryToPublications,
            LinkTypes::GalleryToExhibitions,
        ];
        for lt in &all_types {
            let result = validate_create_link_tag(lt, &massive_tag);
            assert!(
                matches!(result, ValidateCallbackResult::Invalid(_)),
                "Massive tag should be rejected for {:?}",
                lt
            );
        }
    }

    // -- Delete link tag tests --

    #[test]
    fn delete_link_tag_empty_valid() {
        let tag = LinkTag::new(vec![]);
        let result = validate_delete_link_tag(&tag);
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn delete_link_tag_at_limit_valid() {
        let tag = LinkTag::new(vec![0u8; 256]);
        let result = validate_delete_link_tag(&tag);
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn delete_link_tag_over_limit_invalid() {
        let tag = LinkTag::new(vec![0u8; 257]);
        let result = validate_delete_link_tag(&tag);
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
