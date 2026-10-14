// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

#![doc = "Canonical Marketplace Listing data model shared by integrity and coordinator crates."]

use hdi::prelude::*;

/// Listing entry type - core marketplace data structure
///
/// This represents a single item for sale in the marketplace.
/// Integrates with Epistemic Charter v2.0 for truth classification.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Listing {
    /// Product title (1-200 characters)
    pub title: String,

    /// Detailed description (1-5000 characters)
    pub description: String,

    /// Price in USD cents (to avoid floating point issues)
    /// Example: $19.99 = 1999 cents
    pub price_cents: u64,

    /// Product category for filtering
    pub category: ListingCategory,

    /// IPFS CIDs for product photos (max 10)
    pub photos_ipfs_cids: Vec<String>,

    /// Number of items available (inventory)
    pub quantity_available: u32,

    /// Current listing status
    pub status: ListingStatus,

    /// Epistemic classification (Epistemic Charter v2.0)
    /// This listing is a claim about a product existing at this price
    pub epistemic: EpistemicClassification,

    /// Creation timestamp (milliseconds since Unix epoch)
    pub created_at: Timestamp,

    /// Last update timestamp
    pub updated_at: Timestamp,
}

/// Product categories for marketplace
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(rename_all = "PascalCase")]
pub enum ListingCategory {
    Electronics,
    Fashion,
    #[serde(rename = "Home & Garden")]
    HomeGarden,
    #[serde(rename = "Sports & Outdoors")]
    SportsOutdoors,
    #[serde(rename = "Books & Media")]
    BooksMedia,
    #[serde(rename = "Toys & Games")]
    ToysGames,
    #[serde(rename = "Health & Beauty")]
    HealthBeauty,
    Automotive,
    #[serde(rename = "Art & Collectibles")]
    ArtCollectibles,
    Other,
}

/// Listing lifecycle status
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum ListingStatus {
    /// Active and available for purchase
    Active,
    /// Sold out (inventory = 0)
    Sold,
    /// Seller paused listing
    Inactive,
    /// Soft-deleted (hidden from UI)
    Deleted,
}

/// Epistemic Charter v2.0 Classification
///
/// Every listing is a claim about reality that can be verified.
/// See: Mycelix-Core/docs/architecture/THE EPISTEMIC CHARTER (v2.0).md
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct EpistemicClassification {
    /// E-Axis: How to verify this claim?
    /// For listings: Typically E1 (seller attestation) or E2 (buyer verification after purchase)
    pub empirical: EmpiricalLevel,

    /// N-Axis: Who agrees this is binding?
    /// For listings: N0 (seller's personal claim) until transaction creates N1 (buyer-seller agreement)
    pub normative: NormativeLevel,

    /// M-Axis: How long does this matter?
    /// For listings: M1 (prune when sold) or M2 (keep for reputation history)
    pub materiality: MaterialityLevel,
}

/// E-Axis: Empirical Verifiability
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum EmpiricalLevel {
    /// E0: Unverifiable belief
    E0Null,
    /// E1: Personal testimonial (seller's claim)
    E1Testimonial,
    /// E2: Privately verifiable (buyer can verify after purchase)
    E2PrivateVerify,
    /// E3: Cryptographically proven (signed by trusted authority)
    E3Cryptographic,
    /// E4: Publicly reproducible
    E4PublicRepro,
}

/// N-Axis: Normative Authority
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum NormativeLevel {
    /// N0: Personal (seller only)
    N0Personal,
    /// N1: Communal (buyer-seller agreement)
    N1Communal,
    /// N2: Network (marketplace consensus)
    N2Network,
    /// N3: Axiomatic (constitutional/legal)
    N3Axiomatic,
}

/// M-Axis: Materiality/State Management
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum MaterialityLevel {
    /// M0: Ephemeral (discard immediately)
    M0Ephemeral,
    /// M1: Temporal (prune after state change)
    M1Temporal,
    /// M2: Persistent (archive after time)
    M2Persistent,
    /// M3: Foundational (preserve forever)
    M3Foundational,
}
