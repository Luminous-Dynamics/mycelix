//! Fail-closed furnishing discovery over the existing Marketplace listing read.
//!
//! This is a projection, not a product catalogue, live-offer guarantee, geometry
//! verifier, or checkout path. The current listing wire contract does not prove
//! that a returned action is the newest listing revision, that the read is
//! exhaustive, or that the transport is live rather than fixture-backed.

use crate::{ClientError, MarketplaceClient, TransportKind, ZomeTransport};
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

    /// Describes only the configured adapter/transport path. It does not
    /// certify liveness, listing freshness, or snapshot completeness.
    fn transport_provenance(&self) -> TransportProvenance {
        TransportProvenance::UnclassifiedTransport
    }
}

#[async_trait(?Send)]
impl<T> ListingReadPort for MarketplaceClient<T>
where
    T: ZomeTransport,
{
    async fn read_all_listings(&self) -> Result<ListingsResponse, ClientError> {
        self.get_all_listings().await
    }

    fn transport_provenance(&self) -> TransportProvenance {
        self.transport_kind().into()
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
    /// The configured JS bridge path was selected; this is not a freshness,
    /// completeness, or independently verified live-conductor receipt.
    ConfiguredJsBridgeTransport,
    /// The response was requested through the explicit development fixture path.
    ExplicitDevelopmentFixture,
    /// The configured stub always fails closed; a successful snapshot should not
    /// normally be reachable through this mode.
    UnavailableStub,
    /// The transport has not declared a source mode.
    UnclassifiedTransport,
}

impl From<TransportKind> for TransportProvenance {
    fn from(value: TransportKind) -> Self {
        match value {
            TransportKind::JsBridge => Self::ConfiguredJsBridgeTransport,
            TransportKind::DevelopmentFixture => Self::ExplicitDevelopmentFixture,
            TransportKind::UnavailableStub => Self::UnavailableStub,
            TransportKind::Unclassified => Self::UnclassifiedTransport,
        }
    }
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
    /// Timestamp from the returned record, in the Marketplace wire unit (microseconds).
    pub observed_created_at_micros: i64,
    /// Timestamp from the returned record; not an independently verified freshness proof.
    pub observed_updated_at_micros: i64,
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
        let transport_provenance = self.read_port.transport_provenance();
        let response = self.read_port.read_all_listings().await?;
        let mut snapshot = project_listing_snapshot(response);
        snapshot.transport_provenance = transport_provenance;
        Ok(snapshot)
    }
}

/// Project existing typed listing reads without inventing missing product facts.
///
/// Non-Home-&-Garden categories are omitted. Home-&-Garden is still broad; every
/// returned record remains marked for manual item-type review. No room plan,
/// address, occupant profile, preference, design digest, or query digest is
/// accepted by or transmitted through this function.
pub fn project_listing_snapshot(response: ListingsResponse) -> FurnishingDiscoverySnapshot {
    let mut listings = response
        .listings
        .into_iter()
        .filter(|output| matches!(&output.listing.category, ListingCategory::HomeGarden))
        .map(project_listing)
        .collect::<Vec<_>>();
    // DHT/link traversal order is not a stable ranking. Sort by exact source
    // action hash so identical snapshots produce the same projection order.
    listings.sort_by_key(|candidate| candidate.listing_hash.to_holochain_b64());
    let result = if listings.is_empty() {
        SnapshotResult::NoHomeGardenListingsReturned
    } else {
        SnapshotResult::HomeGardenListingsReturned
    };

    FurnishingDiscoverySnapshot {
        result,
        coverage: SnapshotCoverage::UnknownUnpaginatedRead,
        transport_provenance: TransportProvenance::UnclassifiedTransport,
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
        observed_created_at_micros: output.listing.created_at.0,
        observed_updated_at_micros: output.listing.updated_at.0,
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
        assert_eq!(candidate.observed_created_at_micros, 0);
        assert_eq!(candidate.observed_updated_at_micros, 7);
        assert_eq!(
            candidate.observed_empirical_class,
            ObservedEmpiricalClass::SellerTestimonial
        );
        assert_eq!(
            candidate.furniture_identity,
            FurnitureIdentity::NeedsItemTypeConfirmation
        );
        assert_eq!(
            candidate.geometry_evidence,
            GeometryEvidence::NotPresentInListingContract
        );
        assert_eq!(candidate.compatibility, CompatibilityState::NotEvaluated);
        assert_eq!(
            candidate.revision_freshness,
            RevisionFreshness::LatestRevisionUnproven
        );
        assert_eq!(
            candidate.review_disposition,
            ReviewDisposition::ManualReviewOnly
        );
        assert!(!snapshot.can_claim_complete_current_furniture_offers());
    }

    #[cfg(feature = "dev-fixtures")]
    #[test]
    fn adapter_snapshot_preserves_fixture_mode_and_unknown_coverage() {
        use std::{
            future::Future,
            pin::Pin,
            sync::Arc,
            task::{Context, Poll, Wake, Waker},
        };

        struct NoopWake;
        impl Wake for NoopWake {
            fn wake(self: Arc<Self>) {}
        }

        fn block_on_ready<F: Future>(future: F) -> F::Output {
            let mut future = Box::pin(future);
            let waker = Waker::from(Arc::new(NoopWake));
            let mut context = Context::from_waker(&waker);
            match Pin::as_mut(&mut future).poll(&mut context) {
                Poll::Ready(value) => value,
                Poll::Pending => panic!("fixture listing read unexpectedly yielded"),
            }
        }

        let adapter = FurnishingDiscoveryAdapter::new(MarketplaceClient::new(
            crate::FixtureTransport::default(),
        ));
        let snapshot = block_on_ready(adapter.discover()).unwrap();

        assert_eq!(
            snapshot.transport_provenance,
            TransportProvenance::ExplicitDevelopmentFixture
        );
        assert_eq!(
            snapshot.coverage,
            SnapshotCoverage::UnknownUnpaginatedRead
        );
        assert_eq!(snapshot.returned_count(), 1);
        assert!(!snapshot.can_claim_complete_current_furniture_offers());
        assert!(snapshot.listings.iter().all(|listing| {
            listing.review_disposition == ReviewDisposition::ManualReviewOnly
                && listing.geometry_evidence == GeometryEvidence::NotPresentInListingContract
                && listing.compatibility == CompatibilityState::NotEvaluated
        }));
    }

    #[test]
    fn unknown_transport_defaults_to_unclassified_provenance() {
        assert_eq!(
            TransportProvenance::from(TransportKind::Unclassified),
            TransportProvenance::UnclassifiedTransport
        );
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
        assert_eq!(
            snapshot.listings[0].review_disposition,
            ReviewDisposition::ManualReviewOnly
        );
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
        assert_eq!(
            snapshot.result,
            SnapshotResult::NoHomeGardenListingsReturned
        );
        assert_eq!(snapshot.coverage, SnapshotCoverage::UnknownUnpaginatedRead);
        assert_eq!(
            snapshot.transport_provenance,
            TransportProvenance::UnclassifiedTransport
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
    fn snapshot_projection_order_is_independent_of_source_link_order() {
        let mut first = listing(
            ListingCategory::HomeGarden,
            ListingStatus::Active,
            EmpiricalLevel::E1Testimonial,
        );
        first.listing_hash = ActionHash::new(vec![0x91; 39]);
        let mut second = listing(
            ListingCategory::HomeGarden,
            ListingStatus::Active,
            EmpiricalLevel::E1Testimonial,
        );
        second.listing_hash = ActionHash::new(vec![0x11; 39]);

        let forward = project_listing_snapshot(ListingsResponse {
            listings: vec![first.clone(), second.clone()],
        });
        let reversed = project_listing_snapshot(ListingsResponse {
            listings: vec![second, first],
        });
        let forward_hashes = forward
            .listings
            .iter()
            .map(|candidate| candidate.listing_hash.clone())
            .collect::<Vec<_>>();
        let reversed_hashes = reversed
            .listings
            .iter()
            .map(|candidate| candidate.listing_hash.clone())
            .collect::<Vec<_>>();

        assert_eq!(forward_hashes, reversed_hashes);
        assert_eq!(forward_hashes[0], ActionHash::new(vec![0x11; 39]));
    }

    #[test]
    fn all_empirical_levels_remain_metadata_not_product_truth() {
        let levels = [
            (
                EmpiricalLevel::E0Null,
                ObservedEmpiricalClass::NoneSpecified,
            ),
            (
                EmpiricalLevel::E1Testimonial,
                ObservedEmpiricalClass::SellerTestimonial,
            ),
            (
                EmpiricalLevel::E2PrivateVerify,
                ObservedEmpiricalClass::PrivateVerificationClaimed,
            ),
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
