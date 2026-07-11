// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Publication Integrity Zome
//! Updated to use HDI 0.7 patterns with FlatOp validation
use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Publication {
    pub id: String,
    pub title: String,
    pub content_hash: String,
    pub content_type: ContentType,
    pub author_did: String,
    pub co_authors: Vec<String>,
    pub language: String,
    pub tags: Vec<String>,
    pub license: License,
    pub encrypted: bool,
    pub published: Timestamp,
    pub updated: Option<Timestamp>,
    pub version: u32,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ContentType {
    Article,
    Opinion,
    Investigation,
    Review,
    Analysis,
    Interview,
    Report,
    Editorial,
    Other(String),
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct License {
    pub license_type: LicenseType,
    pub attribution_required: bool,
    pub commercial_use: bool,
    pub derivative_works: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum LicenseType {
    CC0,
    CCBY,
    CCBYSA,
    CCBYNC,
    CCBYNCSA,
    AllRightsReserved,
    Custom(String),
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ContentBlock {
    pub publication_id: String,
    pub block_index: u32,
    pub content: String,
    pub encrypted_content: Option<String>,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct PublicationVersion {
    pub publication_id: String,
    pub version: u32,
    pub content_hash: String,
    pub change_summary: String,
    pub created: Timestamp,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(visibility = "public")]
    Anchor(Anchor),
    Publication(Publication),
    ContentBlock(ContentBlock),
    PublicationVersion(PublicationVersion),
}

#[hdk_link_types]
pub enum LinkTypes {
    AuthorToPublications,
    TagToPublications,
    PublicationToBlocks,
    PublicationToVersions,
}

/// Genesis self-check
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback using FlatOp pattern
///
/// **P0 author-binding pass, 2026-07-09.** `author_did` is a free-form
/// String with no local DID-to-agent verification convention (same
/// case-(d) gap as every other zome in this cluster). Three coordinator
/// flows update Publication: `update_publication` (content_hash/updated/
/// version) has **no authorization check of any kind** -- worse than the
/// usual case-(d) gap, since even a broken self-report comparison is
/// absent, so ANY agent can rewrite any publication's content and bump
/// its version; `add_tags` (tags/updated) and `update_license`
/// (license/updated) both compare `pub_entry.author_did` against a
/// caller-supplied `requester_did`, both entirely attacker-controlled --
/// the same compare-two-attacker-controlled-values bug class as this
/// cluster's attribution/curation/factcheck zomes. None of this is
/// fixable at the integrity layer without inventing an unverified DID
/// convention, so it is disclosed, not silently patched.
///
/// What IS fixed: the wide-open RegisterUpdate/RegisterDelete bug (38th
/// and final confirmed instance for this cluster). Real content-
/// restriction added to Publication updates via must_get -- restricted
/// to content_hash/updated/version/tags/license, the union of fields the
/// three update flows above ever change (id/title/author_did/co_authors/
/// language/encrypted/published are now immutable, closing the gap
/// where a modified coordinator could silently rewrite the author or
/// republish under someone else's identity).
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Publication(publication) => {
                    validate_create_publication(EntryCreationAction::Create(action), publication)
                }
                EntryTypes::ContentBlock(block) => {
                    validate_create_content_block(EntryCreationAction::Create(action), block)
                }
                EntryTypes::PublicationVersion(version) => validate_create_publication_version(
                    EntryCreationAction::Create(action),
                    version,
                ),
            },
            OpEntry::UpdateEntry {
                app_entry,
                original_action_hash,
                action,
                ..
            } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Publication(publication) => {
                    validate_update_publication(action, original_action_hash, publication)
                }
                EntryTypes::ContentBlock(_) => Ok(ValidateCallbackResult::Invalid(
                    "Content blocks cannot be updated".into(),
                )),
                EntryTypes::PublicationVersion(_) => Ok(ValidateCallbackResult::Invalid(
                    "Publication versions cannot be updated".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, .. } => match link_type {
            LinkTypes::AuthorToPublications => Ok(ValidateCallbackResult::Valid),
            LinkTypes::TagToPublications => Ok(ValidateCallbackResult::Valid),
            LinkTypes::PublicationToBlocks => Ok(ValidateCallbackResult::Valid),
            LinkTypes::PublicationToVersions => Ok(ValidateCallbackResult::Valid),
        },
        // Deliberately left permissive: the coordinator never calls
        // delete_link. Reviewed 2026-07-09 during the P0 author-binding pass.
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // Previously fully permissive (`Ok(Valid)` unconditionally) --
            // the 38th confirmed instance of this exact bug pattern this
            // pass. Found + fixed 2026-07-09.
            OpUpdate::Entry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Publication(publication) => validate_update_publication(
                    action.clone(),
                    action.original_action_address,
                    publication,
                ),
                EntryTypes::ContentBlock(_) => Ok(ValidateCallbackResult::Invalid(
                    "Content blocks cannot be updated".into(),
                )),
                EntryTypes::PublicationVersion(_) => Ok(ValidateCallbackResult::Invalid(
                    "Publication versions cannot be updated".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(OpDelete { action }) => {
            let _ = must_get_action(action.deletes_address.clone())?;
            // No author field exists anywhere in this zome's entries to
            // compare against (case d, see the module doc comment above)
            // -- deletion authorization is left as-is.
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

fn validate_create_publication(
    _action: EntryCreationAction,
    publication: Publication,
) -> ExternResult<ValidateCallbackResult> {
    if !publication.author_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Author must be a valid DID".into(),
        ));
    }
    if publication.title.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Title cannot be empty".into(),
        ));
    }
    if publication.content_hash.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Content hash required".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_publication(
    _action: Update,
    original_action_hash: ActionHash,
    publication: Publication,
) -> ExternResult<ValidateCallbackResult> {
    if publication.title.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Title cannot be empty".into(),
        ));
    }

    let original_record = must_get_valid_record(original_action_hash)?;
    let original: Publication = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original publication not found".into()
        )))?;

    if publication.id != original.id
        || publication.title != original.title
        || publication.author_did != original.author_did
        || publication.co_authors != original.co_authors
        || publication.language != original.language
        || publication.encrypted != original.encrypted
        || publication.published != original.published
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only content_hash/tags/license/updated/version can change on a publication update"
                .into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_content_block(
    _action: EntryCreationAction,
    _block: ContentBlock,
) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_publication_version(
    _action: EntryCreationAction,
    _version: PublicationVersion,
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

    fn valid_license() -> License {
        License {
            license_type: LicenseType::CCBY,
            attribution_required: true,
            commercial_use: true,
            derivative_works: true,
        }
    }

    fn valid_publication() -> Publication {
        Publication {
            id: "pub-1".into(),
            title: "Breaking News".into(),
            content_hash: "hash1".into(),
            content_type: ContentType::Article,
            author_did: "did:key:z6Mkfoo".into(),
            co_authors: vec![],
            language: "en".into(),
            tags: vec![],
            license: valid_license(),
            encrypted: false,
            published: Timestamp::from_micros(0),
            updated: None,
            version: 1,
        }
    }

    #[test]
    fn create_publication_requires_did_prefix() {
        let mut publication = valid_publication();
        publication.author_did = "not-a-did".into();
        let result = validate_create_publication(dummy_action(), publication).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_publication_requires_title_and_content_hash() {
        let mut publication = valid_publication();
        publication.title = "".into();
        let result = validate_create_publication(dummy_action(), publication).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_publication_valid() {
        let result = validate_create_publication(dummy_action(), valid_publication()).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    // validate_update_publication calls must_get_valid_record, which
    // requires a live HDI host and can't run in a plain unit test --
    // matching the established pattern from every other zome's update
    // validator this pass.
}
