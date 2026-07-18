use crate::{ActionHash, AgentPubKey, EpistemicClassification, TimestampMicros};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
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

impl ListingCategory {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Electronics => "Electronics",
            Self::Fashion => "Fashion",
            Self::HomeGarden => "Home & Garden",
            Self::SportsOutdoors => "Sports & Outdoors",
            Self::BooksMedia => "Books & Media",
            Self::ToysGames => "Toys & Games",
            Self::HealthBeauty => "Health & Beauty",
            Self::Automotive => "Automotive",
            Self::ArtCollectibles => "Art & Collectibles",
            Self::Other => "Other",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ListingStatus {
    Active,
    Sold,
    Inactive,
    Deleted,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Listing {
    pub title: String,
    pub description: String,
    pub price_cents: u64,
    pub category: ListingCategory,
    pub photos_ipfs_cids: Vec<String>,
    pub quantity_available: u32,
    pub status: ListingStatus,
    pub epistemic: EpistemicClassification,
    pub created_at: TimestampMicros,
    pub updated_at: TimestampMicros,
}

impl Listing {
    pub fn formatted_price(&self) -> String {
        format!("${}.{:02}", self.price_cents / 100, self.price_cents % 100)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ListingOutput {
    pub listing_hash: ActionHash,
    pub listing: Listing,
    pub seller_agent_id: AgentPubKey,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ListingsResponse {
    pub listings: Vec<ListingOutput>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CreateListingInput {
    pub title: String,
    pub description: String,
    pub price_cents: u64,
    pub category: ListingCategory,
    pub photos_ipfs_cids: Vec<String>,
    pub quantity_available: u32,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct UpdateListingInput {
    pub listing_hash: ActionHash,
    pub title: Option<String>,
    pub description: Option<String>,
    pub price_cents: Option<u64>,
    pub category: Option<ListingCategory>,
    pub photos_ipfs_cids: Option<Vec<String>>,
    pub quantity_available: Option<u32>,
    pub status: Option<ListingStatus>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn category_wire_labels_match_integrity_zome() {
        let encoded = serde_json::to_string(&ListingCategory::HomeGarden).unwrap();
        assert_eq!(encoded, "\"Home & Garden\"");
    }

    #[test]
    fn price_format_is_integer_based() {
        let listing = Listing {
            title: "test".into(),
            description: "test".into(),
            price_cents: 1_999,
            category: ListingCategory::Other,
            photos_ipfs_cids: vec!["bafyfixture".into()],
            quantity_available: 1,
            status: ListingStatus::Active,
            epistemic: EpistemicClassification {
                empirical: crate::EmpiricalLevel::E1Testimonial,
                normative: crate::NormativeLevel::N0Personal,
                materiality: crate::MaterialityLevel::M1Temporal,
            },
            created_at: TimestampMicros(0),
            updated_at: TimestampMicros(0),
        };
        assert_eq!(listing.formatted_price(), "$19.99");
    }
}
