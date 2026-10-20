//! Dependency-free local benchmark for validating revision-aware furnishing snapshots.
//!
//! Run from the frontend-leptos workspace:
//!   cargo bench -p marketplace-client --bench furnishing_revision_projection
//!
//! This measures local validation/projection only. Catalogue construction and
//! all remote/conductor work are outside the timed region.

use marketplace_client::furnishing_discovery::{TransportProvenance, project_revision_snapshot};
use marketplace_domain::{
    ActionHash, AgentPubKey, EmpiricalLevel, EpistemicClassification, Listing,
    ListingCatalogCoverage, ListingCatalogSnapshotV1, ListingCategory, ListingObservationScope,
    ListingOutput, ListingResolution, ListingResolutionReason, ListingResolutionState,
    ListingStatus, MaterialityLevel, NormativeLevel, TimestampMicros,
};
use std::{
    hint::black_box,
    time::{Duration, Instant},
};

fn hash(tag: u8, index: usize) -> ActionHash {
    let mut bytes = [tag; 39];
    bytes[31..39].copy_from_slice(&(index as u64).to_be_bytes());
    ActionHash::new(bytes.to_vec())
}

fn listing_output(action_hash: ActionHash, index: usize) -> ListingOutput {
    ListingOutput {
        listing_hash: action_hash,
        seller_agent_id: AgentPubKey::new(vec![0x42; 39]),
        listing: Listing {
            title: format!("Synthetic furnishing {index}"),
            description: "Synthetic benchmark record; no seller or stock assertion.".into(),
            price_cents: 10_000 + index as u64,
            category: if index.is_multiple_of(3) {
                ListingCategory::HomeGarden
            } else {
                ListingCategory::Electronics
            },
            photos_ipfs_cids: vec![],
            quantity_available: 1 + u32::try_from(index % 4).unwrap_or(0),
            status: ListingStatus::Active,
            epistemic: EpistemicClassification {
                empirical: EmpiricalLevel::E1Testimonial,
                normative: NormativeLevel::N0Personal,
                materiality: MaterialityLevel::M2Persistent,
            },
            created_at: TimestampMicros(1_000),
            updated_at: TimestampMicros(index as i64),
        },
    }
}

fn resolution(index: usize) -> ListingResolution {
    let root = hash(0x10, index);
    if index.is_multiple_of(10) {
        let heads = vec![
            listing_output(hash(0x20, index), index),
            listing_output(hash(0x21, index), index.saturating_add(1)),
        ];
        ListingResolution {
            root_listing_hash: root,
            policy_version: 1,
            observation_scope: ListingObservationScope::LocalDhtUpdateGraph,
            state: ListingResolutionState::ConcurrentHeads,
            reason: ListingResolutionReason::ConcurrentObservedHeads,
            sole_observed_head: None,
            revision_count: 3,
            heads,
        }
    } else {
        let output = listing_output(root.clone(), index);
        ListingResolution {
            root_listing_hash: root,
            policy_version: 1,
            observation_scope: ListingObservationScope::LocalDhtUpdateGraph,
            state: ListingResolutionState::SingleObservedHead,
            reason: ListingResolutionReason::SingleObservedHead,
            sole_observed_head: Some(output.clone()),
            heads: vec![output],
            revision_count: 1,
        }
    }
}

fn corpus(size: usize) -> ListingCatalogSnapshotV1 {
    let listings = (0..size).map(resolution).collect::<Vec<_>>();
    let concurrent = u32::try_from(
        listings
            .iter()
            .filter(|row| row.state == ListingResolutionState::ConcurrentHeads)
            .count(),
    )
    .unwrap_or(u32::MAX);
    let count = u32::try_from(size).unwrap_or(u32::MAX);
    ListingCatalogSnapshotV1 {
        policy_version: 1,
        coverage: ListingCatalogCoverage::LocalObservedLinksOnly,
        observed_link_count: count,
        unique_listing_count: count,
        single_observed_head_count: count.saturating_sub(concurrent),
        concurrent_head_listing_count: concurrent,
        listings,
    }
}

fn percentile(samples: &[Duration], percent: usize) -> Duration {
    let index = samples.len().saturating_sub(1) * percent / 100;
    samples[index]
}

fn main() {
    for (size, iterations) in [(25, 100), (100, 100), (1_000, 30), (10_000, 10)] {
        let frozen = corpus(size);
        let mut samples = Vec::with_capacity(iterations);
        let mut candidate_count = 0;
        for _ in 0..iterations {
            let input = frozen.clone();
            let start = Instant::now();
            let projected = project_revision_snapshot(
                black_box(input),
                TransportProvenance::ConfiguredJsBridgeTransport,
            )
            .expect("synthetic benchmark corpus must satisfy the revision contract");
            samples.push(start.elapsed());
            candidate_count = projected.returned_count();
            black_box(candidate_count);
        }
        samples.sort_unstable();
        println!(
            "records={size} iterations={iterations} candidates={candidate_count} p50_ms={:.3} p95_ms={:.3}",
            percentile(&samples, 50).as_secs_f64() * 1_000.0,
            percentile(&samples, 95).as_secs_f64() * 1_000.0,
        );
    }
}
