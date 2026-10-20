use crate::{ActionHash, ListingOutput};
use serde::{Deserialize, Serialize};

/// Locally observed Marketplace listing revision state.
/// These names deliberately do not claim global DHT currentness.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ListingResolutionState {
    SingleObservedHead,
    ConcurrentHeads,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ListingObservationScope {
    LocalDhtUpdateGraph,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ListingResolutionReason {
    SingleObservedHead,
    ConcurrentObservedHeads,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ListingCatalogCoverage {
    /// Only discovery links visible to this conductor were examined.
    LocalObservedLinksOnly,
}

/// Wire-compatible with the versioned listings coordinator read endpoint.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ListingResolution {
    pub root_listing_hash: ActionHash,
    pub policy_version: u16,
    pub observation_scope: ListingObservationScope,
    pub state: ListingResolutionState,
    pub reason: ListingResolutionReason,
    pub sole_observed_head: Option<ListingOutput>,
    pub heads: Vec<ListingOutput>,
    pub revision_count: u32,
}

/// Versioned local observation of the listing links currently visible to one conductor.
/// This is intentionally not a global catalogue-completeness receipt.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ListingCatalogSnapshotV1 {
    pub policy_version: u16,
    pub coverage: ListingCatalogCoverage,
    pub observed_link_count: u32,
    pub unique_listing_count: u32,
    pub single_observed_head_count: u32,
    pub concurrent_head_listing_count: u32,
    pub listings: Vec<ListingResolution>,
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn revision_states_match_backend_snake_case_wire_values() {
        assert_eq!(
            serde_json::to_string(&ListingResolutionState::SingleObservedHead).unwrap(),
            "\\"single_observed_head\\""
        );
        assert_eq!(
            serde_json::to_string(&ListingResolutionState::ConcurrentHeads).unwrap(),
            "\\"concurrent_heads\\""
        );
        assert_eq!(
            serde_json::to_string(&ListingResolutionReason::ConcurrentObservedHeads).unwrap(),
            "\\"concurrent_observed_heads\\""
        );
        assert_eq!(
            serde_json::to_string(&ListingCatalogCoverage::LocalObservedLinksOnly).unwrap(),
            "\\"local_observed_links_only\\""
        );
    }
}
