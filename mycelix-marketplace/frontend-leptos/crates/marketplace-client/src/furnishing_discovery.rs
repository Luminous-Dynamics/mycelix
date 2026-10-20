//! Fail-closed furnishing discovery over the existing Marketplace listing read.
//!
//! This is a projection, not a product catalogue, live-offer guarantee, geometry
//! verifier, or checkout path. The current listing wire contract does not prove
//! that a returned action is the newest listing revision, that the read is
//! exhaustive, or that the transport is live rather than fixture-backed.

use crate::{ClientError, MarketplaceClient, ZomeTransport};
use async_trait::async_trait;
use marketplace_domain::{
    ActionHash, AgentPubKey, EmpiricalLevel, ListingCategory, ListingOutput, ListingStatus,
    ListingsResponse,
};

/// Minimal read capability required by the discovery adapter.
///
/// Deliberately omits create/update/delete listing, order, reservation, seller
/// messaging, payment, and transaction methods. The adapter cannot invoke those
/// operations through this interface.
#[async_trait(?Send)]
pub trait ListingReadPort {
    async fn read_all_listings(&self) -> Result<ListingsResponse, ClientError>;
}

#[async_trait(?Send)]
impl<T> ListingReadPort for MarketplaceClient<T>
where
    T: ZomeTransport,
{
    async fn read_all_listings(&self) -> Result<ListingsResponse, ClientError> {
        self.get_all_listings().await
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SnapshotCoverage {
    /// get_all_listings has no paging/completeness receipt in this client
    /// contract; eventual DHT convergence is not established by one read.
    UnknownUnpaginatedRead,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TransportProvenance {
    /// ZomeTransport does not currently identify whether the response came from
    /// a live conductor, a fixture transport, or another adapter.
    UnclassifiedLiveOrFixture,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RevisionFreshness {
    /// Exact action hash is retained, but this API does not prove it is the
    /// newest valid revision of the listing.
    LatestRevisionUnproven,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FurnitureIdentity {
    /// Home & Garden is broader than furniture. The listing contract contains
    /// no canonical furniture subtype, so this is not a furniture assertion.
    NeedsItemTypeConfirmation,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GeometryEvidence {
    /// The existing listing payload has no typed dimensions, coordinate frame,
    /// use-rights evidence, or geometry digest.
    NotPresentInListingContract,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CompatibilityState {
    /// No fit/access/clearance check is performed by this adapter.
    NotEvaluated,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ObservedEmpiricalClass {
    NoneSpecified,
    SellerTestimonial,
    PrivateVerificationClaimed,
    CryptographicClaimClassified,
    PublicReproducibilityClassified,
}

impl From<&EmpiricalLevel> for ObservedEmpiricalClass {
    fn from(value: &EmpiricalLevel) -> Self {
        match value {
            EmpiricalLevel::E0Null => Self::NoneSpecified,
            EmpiricalLevel::E1Testimonial => Self::SellerTestimonial,
            EmpiricalLevel::E2PrivateVerify => Self::PrivateVerificationClaimed,
            EmpiricalLevel::E3Cryptographic => Self::CryptographicClaimClassified,
            EmpiricalLevel::E4PublicRepro => Self::PublicReproducibilityClassified,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ReviewDisposition {
    /// Every row is discovery-only. Downstream code must not promote this state
    /// to fit, verified product claims, or a current commercial offer.
    ManualReviewOnly,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ObservedFurnishingListing {
    pub listing_hash: ActionHash,
    pub seller_agent_id: AgentPubKey,
    pub title: String,
    pub description: String,
    /// Price as present on the exact returned listing action—not a live quote.
    pub observed_price_cents: u64,
    /// Quantity as present on the exact returned listing action—not live stock.
    pub observed_quantity_available: u32,
    /// Status as present on the exact returned listing action—not current status.
    pub observed_status: ListingStatus,
    /// This preserves the Marketplace classification; it does not independently
    /// verify the evidence behind that classification.
    pub observed_empirical_class: ObservedEmpiricalClass,
    pub furniture_identity: FurnitureIdentity,
    pub geometry_evidence: GeometryEvidence,
    pub compatibility: CompatibilityState,
    pub revision_freshness: RevisionFreshness,
    pub review_disposition: ReviewDisposition,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SnapshotResult {
    NoHomeGardenListingsReturned,
    HomeGardenListingsReturned,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FurnishingDiscoverySnapshot {
    pub result: SnapshotResult,
    pub coverage: SnapshotCoverage,
    pub transport_provenance: TransportProvenance,
    pub listings: Vec<ObservedFurnishingListing>,
}

impl FurnishingDiscoverySnapshot {
    pub fn returned_count(&self) -> usize {
        self.listings.len()
    }

    /// Always false in v0.1: the current read contract supplies neither a
    /// completeness receipt nor latest-revision resolution.
    pub fn can_claim_complete_current_furniture_offers(&self) -> bool {
        false
    }
}

/// Typed, effects-disabled adapter. Its only required capability is a listing
/// read port; it contains no purchase or mutation handle.
pub struct FurnishingDiscoveryAdapter<P> {
    read_port: P,
}

impl<P> FurnishingDiscoveryAdapter<P>
where
    P: ListingReadPort,
{
    pub fn new(read_port: P) -> Self {
        Self { read_port }
    }

    pub async fn discover(&self) -> Result<FurnishingDiscoverySnapshot, ClientError> {
        let response = self.read_port.read_all_listings().await?;
        Ok(project_listing_snapshot(response))
    }
}

/// Project existing typed listing reads without inventing missing product facts.
///
/// Non-Home-&-Garden categories are omitted. Home-&-Garden is still broad; every
/// returned record remains marked for manual item-type review. No room plan,
/// address, occupant profile, preference, design digest, or query digest is
/// accepted by or transmitted through this function.
pub fn project_listing_snapshot(response: ListingsResponse) -> FurnishingDiscoverySnapshot {
    let listings = response
        .listings
        .into_iter()
        .filter(|output| matches!(&output.listing.category, ListingCategory::HomeGarden))
        .map(project_listing)
        .collect::<Vec<_>>();
    let result = if listings.is_empty() {
        SnapshotResult::NoHomeGardenListingsReturned
    } else {
        SnapshotResult::HomeGardenListingsReturned
    };

    FurnishingDiscoverySnapshot {
        result,
        coverage: SnapshotCoverage::UnknownUnpaginatedRead,
        transport_provenance: TransportProvenance::UnclassifiedLiveOrFixture,
        listings,
    }
}

fn project_listing(output: ListingOutput) -> ObservedFurnishingListing {
    ObservedFurnishingListing {
        listing_hash: output.listing_hash,
        seller_agent_id: output.seller_agent_id,
        title: output.listing.title,
        description: output.listing.description,
        observed_price_cents: output.listing.price_cents,
        observed_quantity_available: output.listing.quantity_available,
        observed_status: output.listing.status,
        observed_empirical_class: (&output.listing.epistemic.empirical).into(),
        furniture_identity: FurnitureIdentity::NeedsItemTypeConfirmation,
        geometry_evidence: GeometryEvidence::NotPresentInListingContract,
        compatibility: CompatibilityState::NotEvaluated,
        revision_freshness: RevisionFreshness::LatestRevisionUnproven,
        review_disposition: ReviewDisposition::ManualReviewOnly,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use marketplace_domain::{
        EpistemicClassification, Listing, MaterialityLevel, NormativeLevel, TimestampMicros,
    };

    fn listing(
        category: ListingCategory,
        status: ListingStatus,
        empirical: EmpiricalLevel,
    ) -> ListingOutput {
        ListingOutput {
            listing_hash: ActionHash::new(vec![0x81; 39]),
            seller_agent_id: AgentPubKey::new(vec![0x42; 39]),
            listing: Listing {
                title: "Refurbished modular desk".into(),
                description: "Synthetic fixture text; repair history unverified.".into(),
                price_cents: 12_500,
                category,
                photos_ipfs_cids: vec![],
                quantity_available: 2,
                status,
                epistemic: EpistemicClassification {
                    empirical,
                    normative: NormativeLevel::N0Personal,
                    materiality: MaterialityLevel::M2Persistent,
                },
                created_at: TimestampMicros(0),
                updated_at: TimestampMicros(7),
            },
        }
    }

    #[test]
    fn preserves_exact_listing_reference_and_marks_all_product_claims_unresolved() {
        let original_hash = ActionHash::new(vec![0x81; 39]);
        let snapshot = project_listing_snapshot(ListingsResponse {
            listings: vec![listing(
                ListingCategory::HomeGarden,
                ListingStatus::Active,
                EmpiricalLevel::E1Testimonial,
            )],
        });
        assert_eq!(snapshot.returned_count(), 1);
        let candidate = &snapshot.listings[0];
        assert_eq!(candidate.listing_hash, original_hash);
        assert_eq!(candidate.observed_price_cents, 12_500);
        assert_eq!(candidate.observed_quantity_available, 2);
        assert_eq!(candidate.observed_empirical_class, ObservedEmpiricalClass::SellerTestimonial);
        assert_eq!(candidate.furniture_identity, FurnitureIdentity::NeedsItemTypeConfirmation);
        assert_eq!(candidate.geometry_evidence, GeometryEvidence::NotPresentInListingContract);
        assert_eq!(candidate.compatibility, CompatibilityState::NotEvaluated);
        assert_eq!(candidate.revision_freshness, RevisionFreshness::LatestRevisionUnproven);
        assert_eq!(candidate.review_disposition, ReviewDisposition::ManualReviewOnly);
        assert!(!snapshot.can_claim_complete_current_furniture_offers());
    }

    #[test]
    fn home_garden_category_does_not_assert_that_item_is_furniture() {
        let snapshot = project_listing_snapshot(ListingsResponse {
            listings: vec![listing(
                ListingCategory::HomeGarden,
                ListingStatus::Active,
                EmpiricalLevel::E4PublicRepro,
            )],
        });
        assert_eq!(
            snapshot.listings[0].furniture_identity,
            FurnitureIdentity::NeedsItemTypeConfirmation
        );
        assert_eq!(
            snapshot.listings[0].observed_empirical_class,
            ObservedEmpiricalClass::PublicReproducibilityClassified
        );
        assert_eq!(snapshot.listings[0].review_disposition, ReviewDisposition::ManualReviewOnly);
    }

    #[test]
    fn non_active_observed_status_is_preserved_not_reinterpreted_as_current_stock() {
        let snapshot = project_listing_snapshot(ListingsResponse {
            listings: vec![listing(
                ListingCategory::HomeGarden,
                ListingStatus::Sold,
                EmpiricalLevel::E2PrivateVerify,
            )],
        });
        assert_eq!(snapshot.listings[0].observed_status, ListingStatus::Sold);
        assert_eq!(
            snapshot.listings[0].revision_freshness,
            RevisionFreshness::LatestRevisionUnproven
        );
        assert_eq!(
            snapshot.listings[0].observed_empirical_class,
            ObservedEmpiricalClass::PrivateVerificationClaimed
        );
    }

    #[test]
    fn empty_result_does_not_claim_live_empty_or_complete_catalogue() {
        let snapshot = project_listing_snapshot(ListingsResponse { listings: vec![] });
        assert_eq!(snapshot.result, SnapshotResult::NoHomeGardenListingsReturned);
        assert_eq!(snapshot.coverage, SnapshotCoverage::UnknownUnpaginatedRead);
        assert_eq!(
            snapshot.transport_provenance,
            TransportProvenance::UnclassifiedLiveOrFixture
        );
        assert!(!snapshot.can_claim_complete_current_furniture_offers());
    }

    #[test]
    fn unrelated_categories_are_not_projected() {
        let snapshot = project_listing_snapshot(ListingsResponse {
            listings: vec![
                listing(
                    ListingCategory::Electronics,
                    ListingStatus::Active,
                    EmpiricalLevel::E3Cryptographic,
                ),
                listing(
                    ListingCategory::HomeGarden,
                    ListingStatus::Active,
                    EmpiricalLevel::E0Null,
                ),
            ],
        });
        assert_eq!(snapshot.returned_count(), 1);
        assert_eq!(
            snapshot.listings[0].observed_empirical_class,
            ObservedEmpiricalClass::NoneSpecified
        );
    }

    #[test]
    fn all_empirical_levels_remain_metadata_not_product_truth() {
        let levels = [
            (EmpiricalLevel::E0Null, ObservedEmpiricalClass::NoneSpecified),
            (EmpiricalLevel::E1Testimonial, ObservedEmpiricalClass::SellerTestimonial),
            (EmpiricalLevel::E2PrivateVerify, ObservedEmpiricalClass::PrivateVerificationClaimed),
            (
                EmpiricalLevel::E3Cryptographic,
                ObservedEmpiricalClass::CryptographicClaimClassified,
            ),
            (
                EmpiricalLevel::E4PublicRepro,
                ObservedEmpiricalClass::PublicReproducibilityClassified,
            ),
        ];
        for (source, expected) in levels {
            assert_eq!(ObservedEmpiricalClass::from(&source), expected);
        }
    }
}
