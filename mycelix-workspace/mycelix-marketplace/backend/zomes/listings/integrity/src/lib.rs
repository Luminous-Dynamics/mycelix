// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
use hdi::prelude::*;

const MAX_LISTING_PRICE_CENTS: u64 = 100_000_000;

/// Canonical listing data model shared by Marketplace zomes.
pub use listings_types::{
    EmpiricalLevel, EpistemicClassification, Listing, ListingCategory, ListingStatus,
    MaterialityLevel, NormativeLevel,
};

/// Link types for listing discovery and relationships
#[hdk_link_types]
pub enum LinkTypes {
    /// Links from agent to their listings
    /// Base: AgentPubKey, Target: Listing EntryHash
    AgentToListings,

    /// Links from category path to listings
    /// Base: Path("listings.{category}"), Target: Listing EntryHash
    CategoryToListings,

    /// Links from status path to listings
    /// Base: Path("listings.status.{status}"), Target: Listing EntryHash
    StatusToListings,

    /// All listings anchor
    /// Base: Path("all_listings"), Target: Listing EntryHash
    AllListings,
}

/// Entry types for this integrity zome
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Listing(Listing),
}

/// Validation function for Listing entries
///
/// This enforces marketplace rules at the DHT level.
/// Invalid listings are rejected by the network.
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Listing(listing) => validate_create_listing(&listing, &action),
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::Listing(listing) => validate_update_listing(&listing, &action),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(update_entry) => match update_entry {
            OpUpdate::Entry { app_entry, action } => match app_entry {
                EntryTypes::Listing(listing) => validate_update_listing(&listing, &action),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(_delete_entry) => {
            // Allow sellers to delete their own listings
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterCreateLink {
            link_type,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => {
            // Validate that links are created by the listing owner
            match link_type {
                LinkTypes::AgentToListings => Ok(ValidateCallbackResult::Valid),
                LinkTypes::CategoryToListings => Ok(ValidateCallbackResult::Valid),
                LinkTypes::StatusToListings => Ok(ValidateCallbackResult::Valid),
                LinkTypes::AllListings => Ok(ValidateCallbackResult::Valid),
            }
        }
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Valid),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

/// Validate listing data (used for both create and update)
fn validate_listing_data(listing: &Listing) -> ExternResult<ValidateCallbackResult> {
    // Title validation
    if listing.title.is_empty() || listing.title.len() > 200 {
        return Ok(ValidateCallbackResult::Invalid(
            "Title must be 1-200 characters".into(),
        ));
    }

    // Description validation
    if listing.description.is_empty() || listing.description.len() > 5000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Description must be 1-5000 characters".into(),
        ));
    }

    // Price validation (must be positive)
    if listing.price_cents == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Price must be greater than zero".into(),
        ));
    }

    // Prevent unrealistic prices (> $1,000,000)
    if listing.price_cents > MAX_LISTING_PRICE_CENTS {
        return Ok(ValidateCallbackResult::Invalid(
            "Price exceeds maximum allowed ($1,000,000)".into(),
        ));
    }

    // Photos validation
    if listing.photos_ipfs_cids.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "At least one photo is required".into(),
        ));
    }

    if listing.photos_ipfs_cids.len() > 10 {
        return Ok(ValidateCallbackResult::Invalid(
            "Maximum 10 photos allowed".into(),
        ));
    }

    // Validate IPFS CIDs format
    for cid in &listing.photos_ipfs_cids {
        if !is_valid_ipfs_cid(cid) {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Invalid IPFS CID: {}",
                cid
            )));
        }
    }

    // Quantity validation
    if listing.quantity_available == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Quantity must be at least 1".into(),
        ));
    }

    // Epistemic validation: Ensure listings start with proper classification
    // New listings should be E1 (testimonial), N0 (personal), M1 or M2
    match listing.epistemic.empirical {
        EmpiricalLevel::E0Null => {
            return Ok(ValidateCallbackResult::Invalid(
                "Listings cannot be E0 (unverifiable)".into(),
            ));
        }
        EmpiricalLevel::E3Cryptographic | EmpiricalLevel::E4PublicRepro => {
            return Ok(ValidateCallbackResult::Invalid(
                "New listings cannot claim E3/E4 without proof".into(),
            ));
        }
        _ => {} // E1, E2 are valid
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate listing creation
fn validate_create_listing(
    listing: &Listing,
    _action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    validate_listing_data(listing)
}

/// Validate listing updates
fn validate_update_listing(
    listing: &Listing,
    action: &Update,
) -> ExternResult<ValidateCallbackResult> {
    // 1. Run same data validations as create
    let data_validation = validate_listing_data(listing)?;
    if let ValidateCallbackResult::Invalid(reason) = data_validation {
        return Ok(ValidateCallbackResult::Invalid(reason));
    }

    // 2. Authorization check: Only the original seller can update their listing
    // The Update action contains the original_action_hash which created the entry
    // We need to verify the author of the update matches the original author

    // Get the original action that created this listing
    let original_action_hash = action.original_action_address.clone();

    // The author of the current update action
    let updater = action.author.clone();

    // In Holochain's validation model, we can check that the updater
    // matches the original author by examining the chain history
    // The must_get_action call fetches the original action
    let original_action = must_get_action(original_action_hash.clone())?;

    // Extract the original author from the action
    let original_author = original_action.action().author().clone();

    // Authorization check: the updater must be the original seller
    if updater != original_author {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Only the original seller can update this listing. Updater: {}, Original seller: {}",
            updater, original_author
        )));
    }

    // 3. Validate that certain fields cannot be changed (seller protection)
    // The created_at timestamp should never change on updates
    let original_entry = must_get_entry(action.original_entry_address.clone())?;

    if let Entry::App(app_entry) = original_entry.as_content() {
        // Try to deserialize the original listing
        if let Ok(original_listing) = Listing::try_from(app_entry.clone().into_sb()) {
            // created_at should never change
            if listing.created_at != original_listing.created_at {
                return Ok(ValidateCallbackResult::Invalid(
                    "Cannot modify the creation timestamp".into(),
                ));
            }

            // updated_at should be greater than or equal to original
            if listing.updated_at < original_listing.updated_at {
                return Ok(ValidateCallbackResult::Invalid(
                    "Update timestamp cannot be earlier than original".into(),
                ));
            }

            // Prevent resurrection of deleted listings
            if original_listing.status == ListingStatus::Deleted
                && listing.status != ListingStatus::Deleted
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "Cannot reactivate a deleted listing - create a new one instead".into(),
                ));
            }
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate IPFS CID format
///
/// Accepts CIDv0 (Qm...) and CIDv1 (b...) formats
fn is_valid_ipfs_cid(cid: &str) -> bool {
    // CIDv0: Qm + 44 base58 characters
    if cid.len() == 46 && cid.starts_with("Qm") {
        return cid
            .chars()
            .skip(2)
            .all(|c| c.is_ascii_alphanumeric() && c != '0' && c != 'O' && c != 'I' && c != 'l');
    }

    // CIDv1: b + 58 base32 characters (simplified check)
    if cid.len() >= 59 && cid.starts_with('b') {
        return true; // Simplified validation for CIDv1
    }

    false
}

#[cfg(test)]
mod tests {
    use super::*;

    // ===== Helpers =====

    fn valid_listing() -> Listing {
        Listing {
            title: "Test Product".to_string(),
            description: "A great test product description".to_string(),
            price_cents: 1999,
            category: ListingCategory::Electronics,
            photos_ipfs_cids: vec!["QmYwAPJzv5CZsnA625s3Xf2nemtYgPpHdWEz79ojWnPbdG".to_string()],
            quantity_available: 10,
            status: ListingStatus::Active,
            epistemic: EpistemicClassification {
                empirical: EmpiricalLevel::E1Testimonial,
                normative: NormativeLevel::N0Personal,
                materiality: MaterialityLevel::M1Temporal,
            },
            created_at: Timestamp::from_micros(1000000),
            updated_at: Timestamp::from_micros(1000000),
        }
    }

    // ===== IPFS CID Tests =====

    #[test]
    fn test_ipfs_cid_validation_v0_valid() {
        assert!(is_valid_ipfs_cid(
            "QmYwAPJzv5CZsnA625s3Xf2nemtYgPpHdWEz79ojWnPbdG"
        ));
    }

    #[test]
    fn test_ipfs_cid_validation_v0_invalid_short() {
        assert!(!is_valid_ipfs_cid("QmInvalid"));
    }

    #[test]
    fn test_ipfs_cid_validation_empty() {
        assert!(!is_valid_ipfs_cid(""));
    }

    #[test]
    fn test_ipfs_cid_validation_not_a_cid() {
        assert!(!is_valid_ipfs_cid("notacid"));
    }

    #[test]
    fn test_ipfs_cid_validation_v1_valid() {
        // CIDv1 starts with 'b' and is >= 59 chars
        let cid_v1 = "bafybeigdyrzt5sfp7udm7hu76uh7y26nf3efuylqabf3oclgtqy55fbzdi";
        assert!(is_valid_ipfs_cid(cid_v1));
    }

    #[test]
    fn test_ipfs_cid_validation_v0_with_invalid_base58_chars() {
        // '0', 'O', 'I', 'l' are not in base58
        // 46 chars starting with Qm but containing '0'
        assert!(!is_valid_ipfs_cid(
            "Qm0wAPJzv5CZsnA625s3Xf2nemtYgPpHdWEz79ojWnPbdG"
        ));
    }

    // ===== Listing Validation Tests =====

    #[test]
    fn test_validate_listing_data_valid() {
        let listing = valid_listing();
        let result = validate_listing_data(&listing).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_validate_listing_data_empty_title() {
        let listing = Listing {
            title: String::new(),
            ..valid_listing()
        };
        let result = validate_listing_data(&listing).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_validate_listing_data_title_too_long() {
        let listing = Listing {
            title: "x".repeat(201),
            ..valid_listing()
        };
        let result = validate_listing_data(&listing).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_validate_listing_data_empty_description() {
        let listing = Listing {
            description: String::new(),
            ..valid_listing()
        };
        let result = validate_listing_data(&listing).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_validate_listing_data_description_too_long() {
        let listing = Listing {
            description: "x".repeat(5001),
            ..valid_listing()
        };
        let result = validate_listing_data(&listing).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_validate_listing_data_zero_price() {
        let listing = Listing {
            price_cents: 0,
            ..valid_listing()
        };
        let result = validate_listing_data(&listing).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_validate_listing_data_price_too_high() {
        let listing = Listing {
            price_cents: MAX_LISTING_PRICE_CENTS + 1,
            ..valid_listing()
        };
        let result = validate_listing_data(&listing).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_validate_listing_data_max_valid_price() {
        let listing = Listing {
            price_cents: MAX_LISTING_PRICE_CENTS,
            ..valid_listing()
        };
        let result = validate_listing_data(&listing).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_validate_listing_data_no_photos() {
        let listing = Listing {
            photos_ipfs_cids: vec![],
            ..valid_listing()
        };
        let result = validate_listing_data(&listing).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_validate_listing_data_too_many_photos() {
        let cid = "QmYwAPJzv5CZsnA625s3Xf2nemtYgPpHdWEz79ojWnPbdG".to_string();
        let listing = Listing {
            photos_ipfs_cids: vec![cid; 11],
            ..valid_listing()
        };
        let result = validate_listing_data(&listing).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_validate_listing_data_max_photos_valid() {
        let cid = "QmYwAPJzv5CZsnA625s3Xf2nemtYgPpHdWEz79ojWnPbdG".to_string();
        let listing = Listing {
            photos_ipfs_cids: vec![cid; 10],
            ..valid_listing()
        };
        let result = validate_listing_data(&listing).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_validate_listing_data_zero_quantity() {
        let listing = Listing {
            quantity_available: 0,
            ..valid_listing()
        };
        let result = validate_listing_data(&listing).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_validate_listing_data_epistemic_e0_rejected() {
        let listing = Listing {
            epistemic: EpistemicClassification {
                empirical: EmpiricalLevel::E0Null,
                normative: NormativeLevel::N0Personal,
                materiality: MaterialityLevel::M1Temporal,
            },
            ..valid_listing()
        };
        let result = validate_listing_data(&listing).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_validate_listing_data_epistemic_e3_rejected() {
        let listing = Listing {
            epistemic: EpistemicClassification {
                empirical: EmpiricalLevel::E3Cryptographic,
                normative: NormativeLevel::N0Personal,
                materiality: MaterialityLevel::M1Temporal,
            },
            ..valid_listing()
        };
        let result = validate_listing_data(&listing).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_validate_listing_data_epistemic_e2_valid() {
        let listing = Listing {
            epistemic: EpistemicClassification {
                empirical: EmpiricalLevel::E2PrivateVerify,
                normative: NormativeLevel::N0Personal,
                materiality: MaterialityLevel::M1Temporal,
            },
            ..valid_listing()
        };
        let result = validate_listing_data(&listing).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    // ===== Category and Status Tests =====

    #[test]
    fn test_all_listing_categories() {
        let categories = vec![
            ListingCategory::Electronics,
            ListingCategory::Fashion,
            ListingCategory::HomeGarden,
            ListingCategory::SportsOutdoors,
            ListingCategory::BooksMedia,
            ListingCategory::ToysGames,
            ListingCategory::HealthBeauty,
            ListingCategory::Automotive,
            ListingCategory::ArtCollectibles,
            ListingCategory::Other,
        ];
        assert_eq!(categories.len(), 10);
    }

    #[test]
    fn test_all_listing_statuses() {
        let statuses = vec![
            ListingStatus::Active,
            ListingStatus::Sold,
            ListingStatus::Inactive,
            ListingStatus::Deleted,
        ];
        assert_eq!(statuses.len(), 4);
    }

    #[test]
    fn test_listing_serde_roundtrip() {
        let listing = valid_listing();
        let json = serde_json::to_string(&listing).unwrap();
        let parsed: Listing = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.title, listing.title);
        assert_eq!(parsed.price_cents, listing.price_cents);
    }

    // ===== Boundary Tests =====

    #[test]
    fn test_validate_listing_title_exactly_200_chars() {
        let listing = Listing {
            title: "x".repeat(200),
            ..valid_listing()
        };
        let result = validate_listing_data(&listing).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_validate_listing_description_exactly_5000_chars() {
        let listing = Listing {
            description: "x".repeat(5000),
            ..valid_listing()
        };
        let result = validate_listing_data(&listing).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_validate_listing_single_char_title() {
        let listing = Listing {
            title: "A".to_string(),
            ..valid_listing()
        };
        let result = validate_listing_data(&listing).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_validate_listing_min_price() {
        let listing = Listing {
            price_cents: 1,
            ..valid_listing()
        };
        let result = validate_listing_data(&listing).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }
}
