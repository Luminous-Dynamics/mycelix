// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! UESS Epistemic Storage - Integrity Zome
//!
//! Defines entry types and validation rules for the DHT backend
//! of the Unified Epistemic Storage System.
//!
//! Entry Types:
//! - EpistemicEntry: The core storage unit with E/N/M classification
//! - StorageIndex: Key-to-entry mapping for fast lookups
//!
//! Link Types:
//! - KeyToEntry: Links storage keys to their entries
//! - CidToEntry: Links content IDs to their entries

use hdi::prelude::*;

// =============================================================================
// Entry Types
// =============================================================================

/// Epistemic classification for stored data
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct EpistemicClassification {
    /// Empirical level (E0-E4): How verifiable is this data?
    /// E0: Subjective (personal experience)
    /// E1: Observable (witnessed by others)
    /// E2: Privately Verified (cryptographic proof, limited audience)
    /// E3: Cryptographic (publicly verifiable proof)
    /// E4: Reproducible (independently reproducible)
    pub empirical: u8,

    /// Normative level (N0-N3): Who has authority over this data?
    /// N0: Personal (owner only)
    /// N1: Communal (delegated access)
    /// N2: Network (public within network)
    /// N3: Axiomatic (universal truth)
    pub normative: u8,

    /// Materiality level (M0-M3): How long should this data persist?
    /// M0: Ephemeral (session-only)
    /// M1: Temporal (survives restart, not device change)
    /// M2: Persistent (survives node failure)
    /// M3: Foundational (survives network partition)
    pub materiality: u8,
}

/// Schema identity for stored data
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct SchemaIdentity {
    /// Schema identifier (e.g., "user_profile", "transaction")
    pub id: String,
    /// Semantic version
    pub version: String,
    /// Optional schema family for alternative worldviews
    pub family: Option<String>,
}

/// Storage metadata for an entry
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StorageMetadata {
    /// Content identifier (hash of data)
    pub cid: String,
    /// Epistemic classification
    pub classification: EpistemicClassification,
    /// Schema identity
    pub schema: SchemaIdentity,
    /// When this entry was stored (Unix timestamp ms)
    pub stored_at: i64,
    /// When this entry was last modified (Unix timestamp ms)
    pub modified_at: Option<i64>,
    /// Version number for append-only entries
    pub version: u64,
    /// Expiration timestamp (Unix timestamp ms), if any
    pub expires_at: Option<i64>,
    /// Size of the data in bytes
    pub size_bytes: u64,
    /// Agent who created this entry
    pub created_by: String,
    /// Whether this entry has been tombstoned (soft deleted)
    pub tombstone: bool,
    /// If tombstoned, who retracted it
    pub retracted_by: Option<String>,
}

/// The core epistemic storage entry
#[hdk_entry_helper]
#[derive(Clone)]
pub struct EpistemicEntry {
    /// Storage key (unique identifier within namespace)
    pub key: String,
    /// Serialized data (JSON string)
    pub data: String,
    /// Storage metadata
    pub metadata: StorageMetadata,
}

/// Anchor entry for key lookups
#[hdk_entry_helper]
#[derive(Clone)]
pub struct KeyAnchor {
    /// The storage key
    pub key: String,
}

/// Anchor entry for CID lookups
#[hdk_entry_helper]
#[derive(Clone)]
pub struct CidAnchor {
    /// The content identifier
    pub cid: String,
}

// =============================================================================
// Entry Definitions
// =============================================================================

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(name = "epistemic_entry")]
    EpistemicEntry(EpistemicEntry),

    #[entry_type(name = "key_anchor")]
    KeyAnchor(KeyAnchor),

    #[entry_type(name = "cid_anchor")]
    CidAnchor(CidAnchor),
}

// =============================================================================
// Link Types
// =============================================================================

#[hdk_link_types]
pub enum LinkTypes {
    /// Links a key anchor to an epistemic entry
    KeyToEntry,
    /// Links a CID anchor to an epistemic entry
    CidToEntry,
    /// Links to all entries (for listing)
    AllEntries,
}

// =============================================================================
// Validation
// =============================================================================

/// Validate entry creation
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => {
                validate_create_entry(&action.author.to_string(), app_entry)
            }
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash,
                ..
            } => validate_update_entry(&action.author.to_string(), app_entry, original_action_hash),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address,
            target_address,
            ..
        } => validate_create_link(link_type, base_address, target_address),
        FlatOp::RegisterDeleteLink { link_type, .. } => {
            // Allow all link deletions (for tombstoning)
            validate_delete_link(link_type)
        }
        FlatOp::StoreRecord(store_record) => match store_record {
            OpRecord::CreateEntry { app_entry, action } => {
                validate_create_entry(&action.author.to_string(), app_entry)
            }
            OpRecord::UpdateEntry {
                app_entry,
                action,
                original_action_hash,
                ..
            } => validate_update_entry(&action.author.to_string(), app_entry, original_action_hash),
            OpRecord::DeleteEntry { .. } => {
                // Allow deletions (will be tombstoned)
                Ok(ValidateCallbackResult::Valid)
            }
            _ => Ok(ValidateCallbackResult::Valid),
        },
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

/// Enforce that `metadata.created_by` is the committing agent's own DID.
/// Pure so it can be unit-tested without a full action. Applies to BOTH
/// create and update: `store_epistemic_entry` takes `created_by` directly
/// from caller input with zero derivation from `agent_info()` (a clean
/// forgery case -- anyone could claim any `created_by`), and
/// `delete_epistemic_entry` (the only update path -- tombstoning) never
/// changes `created_by`, has no moderator/admin role, and doesn't check who
/// is calling it at all. Binding both create and update to
/// `created_by == author` therefore closes TWO real gaps with one check:
/// creation-identity forgery, and "anyone can tombstone anyone's entry"
/// (since after create is bound, only the true original creator's author
/// key can ever match `created_by` on a later update).
fn require_created_by_is_author(created_by: &str, author_did: &str) -> ValidateCallbackResult {
    if created_by != author_did {
        return ValidateCallbackResult::Invalid(format!(
            "EpistemicEntry metadata.created_by must be the committing agent \
             (storage forgery / unauthorized retraction). Expected '{author_did}', \
             got '{created_by}'"
        ));
    }
    ValidateCallbackResult::Valid
}

fn validate_create_entry(
    author_did: &str,
    entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::EpistemicEntry(epistemic_entry) => {
            if let ValidateCallbackResult::Invalid(msg) =
                require_created_by_is_author(&epistemic_entry.metadata.created_by, author_did)
            {
                return Ok(ValidateCallbackResult::Invalid(msg));
            }
            validate_epistemic_entry(&epistemic_entry)
        }
        EntryTypes::KeyAnchor(key_anchor) => validate_key_anchor(&key_anchor),
        EntryTypes::CidAnchor(cid_anchor) => validate_cid_anchor(&cid_anchor),
    }
}

fn validate_update_entry(
    author_did: &str,
    entry: EntryTypes,
    _original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::EpistemicEntry(epistemic_entry) => {
            // For E3/E4 entries, updates should be blocked (immutable)
            if epistemic_entry.metadata.classification.empirical >= 3 {
                return Ok(ValidateCallbackResult::Invalid(
                    "E3+ entries are immutable and cannot be updated".to_string(),
                ));
            }
            if let ValidateCallbackResult::Invalid(msg) =
                require_created_by_is_author(&epistemic_entry.metadata.created_by, author_did)
            {
                return Ok(ValidateCallbackResult::Invalid(msg));
            }
            validate_epistemic_entry(&epistemic_entry)
        }
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_epistemic_entry(entry: &EpistemicEntry) -> ExternResult<ValidateCallbackResult> {
    // Validate key is not empty
    if entry.key.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Storage key cannot be empty".to_string(),
        ));
    }

    // Validate key length (max 256 characters)
    if entry.key.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Storage key exceeds maximum length of 256 characters".to_string(),
        ));
    }

    // Validate CID format (should start with "cid:")
    if !entry.metadata.cid.starts_with("cid:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid CID format: must start with 'cid:'".to_string(),
        ));
    }

    // Validate empirical level (0-4)
    if entry.metadata.classification.empirical > 4 {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid empirical level: must be 0-4".to_string(),
        ));
    }

    // Validate normative level (0-3)
    if entry.metadata.classification.normative > 3 {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid normative level: must be 0-3".to_string(),
        ));
    }

    // Validate materiality level (0-3)
    if entry.metadata.classification.materiality > 3 {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid materiality level: must be 0-3".to_string(),
        ));
    }

    // Validate schema ID is not empty
    if entry.metadata.schema.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Schema ID cannot be empty".to_string(),
        ));
    }

    // Validate version is not empty
    if entry.metadata.schema.version.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Schema version cannot be empty".to_string(),
        ));
    }

    // Validate version number is positive
    if entry.metadata.version == 0 && !entry.metadata.tombstone {
        return Ok(ValidateCallbackResult::Invalid(
            "Version must be at least 1 for non-tombstone entries".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_key_anchor(anchor: &KeyAnchor) -> ExternResult<ValidateCallbackResult> {
    if anchor.key.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Key anchor cannot have empty key".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_cid_anchor(anchor: &CidAnchor) -> ExternResult<ValidateCallbackResult> {
    if !anchor.cid.starts_with("cid:") {
        return Ok(ValidateCallbackResult::Invalid(
            "CID anchor must have valid CID format".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_link(
    link_type: LinkTypes,
    _base_address: AnyLinkableHash,
    _target_address: AnyLinkableHash,
) -> ExternResult<ValidateCallbackResult> {
    match link_type {
        LinkTypes::KeyToEntry => Ok(ValidateCallbackResult::Valid),
        LinkTypes::CidToEntry => Ok(ValidateCallbackResult::Valid),
        LinkTypes::AllEntries => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_delete_link(_link_type: LinkTypes) -> ExternResult<ValidateCallbackResult> {
    // Allow link deletions (for tombstoning)
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    const ME: &str = "uhCAkSELF";
    const VICTIM: &str = "uhCAkVICTIM";

    fn valid_entry() -> EpistemicEntry {
        EpistemicEntry {
            key: "user:123:profile".into(),
            data: "{\"name\":\"Ada\"}".into(),
            metadata: StorageMetadata {
                cid: "cid:abc123".into(),
                classification: EpistemicClassification {
                    empirical: 1,
                    normative: 1,
                    materiality: 1,
                },
                schema: SchemaIdentity {
                    id: "user_profile".into(),
                    version: "1.0.0".into(),
                    family: None,
                },
                stored_at: 0,
                modified_at: None,
                version: 1,
                expires_at: None,
                size_bytes: 14,
                created_by: ME.into(),
                tombstone: false,
                retracted_by: None,
            },
        }
    }

    #[test]
    fn test_valid_entry_create() {
        let result = validate_create_entry(ME, EntryTypes::EpistemicEntry(valid_entry())).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_create_forgery_rejected() {
        // The P0 case: an agent stores an entry claiming a DIFFERENT
        // created_by than their own -- must be rejected (storage identity
        // forgery). store_epistemic_entry previously took created_by
        // straight from caller input with zero derivation from
        // agent_info(), so this was a clean forgery vector.
        let mut entry = valid_entry();
        entry.metadata.created_by = VICTIM.into();
        let result = validate_create_entry(ME, EntryTypes::EpistemicEntry(entry)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_update_by_original_creator_valid() {
        // The legitimate tombstone flow: the original creator retracts
        // their own entry. created_by is unchanged from creation.
        let mut entry = valid_entry();
        entry.metadata.tombstone = true;
        let result = validate_update_entry(
            ME,
            EntryTypes::EpistemicEntry(entry),
            ActionHash::from_raw_36(vec![0u8; 36]),
        )
        .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_update_by_different_agent_rejected() {
        // The second real gap this fix closes: delete_epistemic_entry
        // never checked who was calling it, so anyone could tombstone
        // anyone else's entry. Here the entry's created_by is VICTIM but
        // the committing agent (author) is ME -- must be rejected.
        let mut entry = valid_entry();
        entry.metadata.created_by = VICTIM.into();
        entry.metadata.tombstone = true;
        let result = validate_update_entry(
            ME,
            EntryTypes::EpistemicEntry(entry),
            ActionHash::from_raw_36(vec![0u8; 36]),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_require_created_by_is_author_helper() {
        assert!(matches!(
            require_created_by_is_author(ME, ME),
            ValidateCallbackResult::Valid
        ));
        assert!(matches!(
            require_created_by_is_author(VICTIM, ME),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn test_key_anchor_and_cid_anchor_unaffected() {
        // Anchors have no created_by field -- confirm the new binding
        // check doesn't spuriously touch these entry types.
        let key_anchor = EntryTypes::KeyAnchor(KeyAnchor { key: "k".into() });
        assert_eq!(
            validate_create_entry(ME, key_anchor).unwrap(),
            ValidateCallbackResult::Valid
        );
        let cid_anchor = EntryTypes::CidAnchor(CidAnchor {
            cid: "cid:x".into(),
        });
        assert_eq!(
            validate_create_entry(ME, cid_anchor).unwrap(),
            ValidateCallbackResult::Valid
        );
    }
}
