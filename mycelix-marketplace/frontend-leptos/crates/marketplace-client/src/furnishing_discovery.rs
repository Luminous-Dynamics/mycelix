//! Fail-closed furnishing discovery over the existing Marketplace listing read.
//!
//! This is a projection, not a product catalogue, live-offer guarantee, geometry
//! verifier, or checkout path. The current listing wire contract does not prove
//! that a returned action is the newest listing revision, that the read is
//! exhaustive, or that the transport is live rather than fixture-backed.

use crate::{ClientError, MarketplaceClient, TransportKind, ZomeTransport};
use async_trait::async_trait;
use marketplace_domain::{
    ActionHash, AgentPubKey, EmpiricalLevel, ListingCatalogCoverage, ListingCatalogSnapshotV1,
    ListingCategory, ListingOutput, ListingResolutionReason, ListingResolutionState, ListingStatus,
    ListingsResponse,
};
use std::collections::HashSet;

/// Minimal read capability required by the discovery adapter.
///
/// Deliberately omits create/update/delete listing, order, reservation, seller
/// messaging, payment, and transaction methods. The adapter cannot invoke those
/// operations through this interface.
#[async_trait(?Send)]
pub trait ListingReadPort {
    /// Read versioned, fork-aware revision states rather than treating linked
    /// listing actions as current records.
    async fn read_revision_snapshot(&self) -> Result<ListingCatalogSnapshotV1, ClientError>;

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
    async fn read_revision_snapshot(&self) -> Result<ListingCatalogSnapshotV1, ClientError> {
        self.get_all_listings_revision_state_v1().await
    }

    fn transport_provenance(&self) -> TransportProvenance {
        self.transport_kind().into()
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SnapshotCoverage {
    /// Versioned endpoint reports only the local DHT link set, not a global
    /// or converged catalogue.
    LocalObservedLinksOnly,
    /// Legacy projection helper with no revision/coverage receipt.
    UnknownUnpaginatedRead,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RevisionGraphState {
    /// Used only by the legacy, non-revision-aware projection helper.
    Unproven,
    /// Exactly one update-tree leaf was observed by this conductor.
    /// This is not global currentness.
    SingleObservedHead,
    /// Multiple update-tree leaves were observed; none is preferred.
    ConcurrentHeads,
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
    /// Stable listing identity (the original Create action).
    pub root_listing_hash: ActionHash,
    /// Exact action hash for this observed revision.
    pub listing_hash: ActionHash,
    pub revision_graph_state: RevisionGraphState,
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
    NoHomeGardenListingsObserved,
    HomeGardenListingsObserved,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FurnishingDiscoverySnapshot {
    pub result: SnapshotResult,
    pub coverage: SnapshotCoverage,
    pub transport_provenance: TransportProvenance,
    /// None means the legacy helper has no source-receipt counters.
    pub observed_link_count: Option<u32>,
    pub unique_listing_count: Option<u32>,
    pub single_observed_head_count: Option<u32>,
    pub concurrent_head_listing_count: Option<u32>,
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
        let response = self.read_port.read_revision_snapshot().await?;
        project_revision_snapshot(response, transport_provenance)
    }
}

/// Validate and project the versioned fork-aware catalogue contract.
/// Any inconsistent state/counter is treated as a decode failure, not normalized away.
pub fn project_revision_snapshot(
    snapshot: ListingCatalogSnapshotV1,
    transport_provenance: TransportProvenance,
) -> Result<FurnishingDiscoverySnapshot, ClientError> {
    validate_revision_snapshot(&snapshot)?;

    let mut listings = Vec::new();
    for resolution in &snapshot.listings {
        let graph_state = match &resolution.state {
            ListingResolutionState::SingleObservedHead => RevisionGraphState::SingleObservedHead,
            ListingResolutionState::ConcurrentHeads => RevisionGraphState::ConcurrentHeads,
        };
        for output in &resolution.heads {
            if matches!(&output.listing.category, ListingCategory::HomeGarden) {
                listings.push(project_listing(
                    output.clone(),
                    resolution.root_listing_hash.clone(),
                    graph_state.clone(),
                ));
            }
        }
    }
    listings.sort_by_key(|candidate| {
        (
            candidate.root_listing_hash.to_holochain_b64(),
            candidate.listing_hash.to_holochain_b64(),
        )
    });

    let result = if listings.is_empty() {
        SnapshotResult::NoHomeGardenListingsObserved
    } else {
        SnapshotResult::HomeGardenListingsObserved
    };

    Ok(FurnishingDiscoverySnapshot {
        result,
        coverage: SnapshotCoverage::LocalObservedLinksOnly,
        transport_provenance,
        observed_link_count: Some(snapshot.observed_link_count),
        unique_listing_count: Some(snapshot.unique_listing_count),
        single_observed_head_count: Some(snapshot.single_observed_head_count),
        concurrent_head_listing_count: Some(snapshot.concurrent_head_listing_count),
        listings,
    })
}

fn validate_revision_snapshot(snapshot: &ListingCatalogSnapshotV1) -> Result<(), ClientError> {
    let invalid =
        |reason: &str| ClientError::Decode(format!("invalid listing revision snapshot: {reason}"));
    if snapshot.policy_version != 1 {
        return Err(invalid("unsupported catalogue policy version"));
    }
    if snapshot.coverage != ListingCatalogCoverage::LocalObservedLinksOnly {
        return Err(invalid("unexpected catalogue coverage claim"));
    }
    if snapshot.unique_listing_count as usize != snapshot.listings.len() {
        return Err(invalid(
            "unique listing count does not match returned records",
        ));
    }
    if snapshot.observed_link_count < snapshot.unique_listing_count {
        return Err(invalid(
            "observed link count is smaller than unique listing count",
        ));
    }

    let mut roots = HashSet::with_capacity(snapshot.listings.len());
    let mut single_count = 0_u32;
    let mut conflict_count = 0_u32;
    for resolution in &snapshot.listings {
        if resolution.policy_version != 1 {
            return Err(invalid("unsupported listing-resolution policy version"));
        }
        if resolution.observation_scope
            != marketplace_domain::ListingObservationScope::LocalDhtUpdateGraph
        {
            return Err(invalid("unexpected listing observation scope"));
        }
        if !roots.insert(resolution.root_listing_hash.clone()) {
            return Err(invalid("duplicate stable listing root"));
        }
        if resolution.revision_count < u32::try_from(resolution.heads.len()).unwrap_or(u32::MAX) {
            return Err(invalid(
                "revision count is smaller than the observed head count",
            ));
        }

        let mut hashes = HashSet::with_capacity(resolution.heads.len());
        for head in &resolution.heads {
            if !hashes.insert(head.listing_hash.clone()) {
                return Err(invalid("duplicate action hash in observed heads"));
            }
        }

        match &resolution.state {
            ListingResolutionState::SingleObservedHead => {
                single_count = single_count.saturating_add(1);
                if resolution.reason != ListingResolutionReason::SingleObservedHead
                    || resolution.heads.len() != 1
                    || resolution.sole_observed_head.as_ref() != resolution.heads.first()
                {
                    return Err(invalid("single-head state contradicts its head evidence"));
                }
            }
            ListingResolutionState::ConcurrentHeads => {
                conflict_count = conflict_count.saturating_add(1);
                if resolution.reason != ListingResolutionReason::ConcurrentObservedHeads
                    || resolution.heads.len() < 2
                    || resolution.sole_observed_head.is_some()
                    || resolution.revision_count
                        <= u32::try_from(resolution.heads.len()).unwrap_or(u32::MAX)
                {
                    return Err(invalid(
                        "concurrent-head state selects heads or omits ancestor count evidence",
                    ));
                }
            }
        }
    }

    if single_count != snapshot.single_observed_head_count {
        return Err(invalid(
            "single-head count does not match per-listing states",
        ));
    }
    if conflict_count != snapshot.concurrent_head_listing_count {
        return Err(invalid(
            "concurrent-head count does not match per-listing states",
        ));
    }
    Ok(())
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
        .map(|output| {
            let root = output.listing_hash.clone();
            project_listing(output, root, RevisionGraphState::Unproven)
        })
        .collect::<Vec<_>>();
    // DHT/link traversal order is not a stable ranking. Sort by exact source
    // action hash so identical snapshots produce the same projection order.
    listings.sort_by_key(|candidate| candidate.listing_hash.to_holochain_b64());
    let result = if listings.is_empty() {
        SnapshotResult::NoHomeGardenListingsObserved
    } else {
        SnapshotResult::HomeGardenListingsObserved
    };

    FurnishingDiscoverySnapshot {
        result,
        coverage: SnapshotCoverage::UnknownUnpaginatedRead,
        transport_provenance: TransportProvenance::UnclassifiedTransport,
        observed_link_count: None,
        unique_listing_count: None,
        single_observed_head_count: None,
        concurrent_head_listing_count: None,
        listings,
    }
}

fn project_listing(
    output: ListingOutput,
    root_listing_hash: ActionHash,
    revision_graph_state: RevisionGraphState,
) -> ObservedFurnishingListing {
    ObservedFurnishingListing {
        root_listing_hash,
        listing_hash: output.listing_hash,
        revision_graph_state,
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

    fn catalog_snapshot(
        listings: Vec<ListingResolution>,
        observed_links: u32,
    ) -> ListingCatalogSnapshotV1 {
        let unique = u32::try_from(listings.len()).unwrap_or(u32::MAX);
        let single = u32::try_from(
            listings
                .iter()
                .filter(|row| row.state == ListingResolutionState::SingleObservedHead)
                .count(),
        )
        .unwrap_or(u32::MAX);
        let concurrent = u32::try_from(
            listings
                .iter()
                .filter(|row| row.state == ListingResolutionState::ConcurrentHeads)
                .count(),
        )
        .unwrap_or(u32::MAX);
        ListingCatalogSnapshotV1 {
            policy_version: 1,
            coverage: ListingCatalogCoverage::LocalObservedLinksOnly,
            observed_link_count: observed_links,
            unique_listing_count: unique,
            single_observed_head_count: single,
            concurrent_head_listing_count: concurrent,
            listings,
        }
    }

    fn concurrent_resolution(root: ActionHash, heads: Vec<ListingOutput>) -> ListingResolution {
        ListingResolution {
            root_listing_hash: root,
            policy_version: 1,
            observation_scope: marketplace_domain::ListingObservationScope::LocalDhtUpdateGraph,
            state: ListingResolutionState::ConcurrentHeads,
            reason: ListingResolutionReason::ConcurrentObservedHeads,
            sole_observed_head: None,
            revision_count: u32::try_from(heads.len())
                .unwrap_or(u32::MAX)
                .saturating_add(1),
            heads,
        }
    }

    #[test]
    fn conflicted_listing_preserves_all_heads_and_selects_none() {
        let mut first = listing(
            ListingCategory::HomeGarden,
            ListingStatus::Active,
            EmpiricalLevel::E1Testimonial,
        );
        first.listing_hash = ActionHash::new(vec![0x81; 39]);
        first.listing.price_cents = 25_000;
        let mut second = first.clone();
        second.listing_hash = ActionHash::new(vec![0x82; 39]);
        second.listing.price_cents = 21_000;
        second.listing.quantity_available = 1;
        let root = ActionHash::new(vec![0x80; 39]);

        let snapshot = project_revision_snapshot(
            catalog_snapshot(
                vec![concurrent_resolution(root.clone(), vec![second, first])],
                1,
            ),
            TransportProvenance::ConfiguredJsBridgeTransport,
        )
        .unwrap();

        assert_eq!(snapshot.coverage, SnapshotCoverage::LocalObservedLinksOnly);
        assert_eq!(snapshot.concurrent_head_listing_count, Some(1));
        assert_eq!(snapshot.listings.len(), 2);
        assert!(snapshot.listings.iter().all(|candidate| {
            candidate.root_listing_hash == root
                && candidate.revision_graph_state == RevisionGraphState::ConcurrentHeads
                && candidate.revision_freshness == RevisionFreshness::LatestRevisionUnproven
                && candidate.review_disposition == ReviewDisposition::ManualReviewOnly
        }));
        assert_ne!(
            snapshot.listings[0].observed_price_cents,
            snapshot.listings[1].observed_price_cents
        );
        assert!(!snapshot.can_claim_complete_current_furniture_offers());
    }

    #[test]
    fn contradictory_concurrent_state_with_a_selected_head_is_rejected() {
        let first = listing(
            ListingCategory::HomeGarden,
            ListingStatus::Active,
            EmpiricalLevel::E1Testimonial,
        );
        let mut second = first.clone();
        second.listing_hash = ActionHash::new(vec![0x82; 39]);
        let bad = ListingResolution {
            root_listing_hash: ActionHash::new(vec![0x80; 39]),
            policy_version: 1,
            observation_scope: marketplace_domain::ListingObservationScope::LocalDhtUpdateGraph,
            state: ListingResolutionState::ConcurrentHeads,
            reason: ListingResolutionReason::ConcurrentObservedHeads,
            sole_observed_head: Some(first.clone()),
            heads: vec![first, second],
            revision_count: 3,
        };
        let result = project_revision_snapshot(
            catalog_snapshot(vec![bad], 1),
            TransportProvenance::ConfiguredJsBridgeTransport,
        );
        assert!(matches!(result, Err(ClientError::Decode(_))));
    }

    #[test]
    fn empty_revision_snapshot_is_only_locally_empty_not_globally_complete() {
        let snapshot = project_revision_snapshot(
            catalog_snapshot(vec![], 0),
            TransportProvenance::ConfiguredJsBridgeTransport,
        )
        .unwrap();
        assert_eq!(
            snapshot.result,
            SnapshotResult::NoHomeGardenListingsObserved
        );
        assert_eq!(snapshot.coverage, SnapshotCoverage::LocalObservedLinksOnly);
        assert_eq!(snapshot.observed_link_count, Some(0));
        assert!(!snapshot.can_claim_complete_current_furniture_offers());
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
        assert_eq!(snapshot.coverage, SnapshotCoverage::LocalObservedLinksOnly);
        assert_eq!(snapshot.observed_link_count, Some(2));
        assert_eq!(snapshot.unique_listing_count, Some(2));
        assert_eq!(snapshot.concurrent_head_listing_count, Some(0));
        assert_eq!(snapshot.returned_count(), 1);
        assert_eq!(
            snapshot.listings[0].revision_graph_state,
            RevisionGraphState::SingleObservedHead
        );
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
            SnapshotResult::NoHomeGardenListingsObserved
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
