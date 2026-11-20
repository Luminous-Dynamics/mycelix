// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Deterministic cross-provider conjunction-event identity reconciliation.
//!
//! This module matches event identities, not provider quality or collision risk.
//! Inputs must already be converted into the same declared canonical time basis.
//! Ambiguous many-to-many candidate groups are preserved as ambiguous rather
//! than resolved by an arbitrary nearest-neighbour or provider-priority rule.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

/// One provider's published conjunction event after explicit time normalization.
///
/// `tca_tai_micros` is a canonical TAI instant, not a raw source timestamp.
/// The caller must retain the source timestamp and its conversion receipt
/// separately. `time_basis_id` identifies the exact canonicalization profile
/// (including leap-second / time-conversion table revision).
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct ProviderConjunctionEvent {
    pub provider_id: String,
    pub event_id: String,
    /// SHA-256 of the exact source artifact that contains this event.
    pub source_artifact_sha256: String,
    /// Explicit source schema/profile identifier, not a guessed media-type label.
    pub source_profile_id: String,
    pub object_a_norad_id: u32,
    pub object_b_norad_id: u32,
    /// Microseconds since 1970-01-01T00:00:00 TAI.
    pub tca_tai_micros: i64,
    /// Canonicalization profile, including time-scale and conversion-table revision.
    pub time_basis_id: String,
    /// SHA-256 of the time-normalization receipt / conversion parameters.
    pub time_conversion_receipt_sha256: String,
    pub source_class: String,
    pub publication_policy_id: String,
}

/// Configuration for a cross-provider reconciliation run.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct EventReconciliationConfig {
    /// Exact canonical time basis that all event TCA values use.
    pub canonical_time_basis_id: String,
    /// Inclusive absolute TCA difference accepted as a candidate identity edge.
    pub tca_tolerance_micros: u64,
    /// Bound the per-provider working set before building candidate graphs.
    pub max_events_per_provider: usize,
    /// Bound candidate edges so pathological dense windows cannot exhaust memory.
    pub max_candidate_edges: usize,
}

/// A unique, one-to-one event match.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct MatchedProviderEvent {
    pub left_provider_id: String,
    pub right_provider_id: String,
    pub left_event_id: String,
    pub right_event_id: String,
    pub left_source_artifact_sha256: String,
    pub right_source_artifact_sha256: String,
    pub left_source_profile_id: String,
    pub right_source_profile_id: String,
    pub left_time_conversion_receipt_sha256: String,
    pub right_time_conversion_receipt_sha256: String,
    /// Signed difference: right TCA minus left TCA.
    pub delta_tca_micros: i64,
    /// True when the two providers assigned object A/B in opposite order.
    pub object_order_swapped: bool,
    pub left_source_class: String,
    pub right_source_class: String,
    pub left_publication_policy_id: String,
    pub right_publication_policy_id: String,
}

/// A connected candidate group with more than one possible one-to-one pairing.
///
/// No match is selected from this group. The caller can inspect original
/// provider artifacts, source lineage, and policy metadata before resolving it.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct AmbiguousProviderEventGroup {
    pub left_event_ids: Vec<String>,
    pub right_event_ids: Vec<String>,
}

/// Cross-provider event reconciliation result.
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
pub struct EventReconciliation {
    pub matched: Vec<MatchedProviderEvent>,
    pub left_only_event_ids: Vec<String>,
    pub right_only_event_ids: Vec<String>,
    pub ambiguous: Vec<AmbiguousProviderEventGroup>,
}

/// Validation or configuration failures; no partial reconciliation is returned.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum EventReconciliationError {
    EmptyCanonicalTimeBasis,
    ToleranceExceedsSupportedRange,
    ZeroEventLimit,
    EventCountLimitExceeded {
        provider_side: ProviderSide,
        count: usize,
        limit: usize,
    },
    CandidateEdgeLimitExceeded {
        count: usize,
        limit: usize,
    },
    EmptyProviderId { event_id: String },
    MixedProviderFeed {
        provider_side: ProviderSide,
        expected_provider_id: String,
        actual_provider_id: String,
    },
    EmptyEventId,
    DuplicateEventId { event_id: String },
    InvalidSourceArtifactDigest { event_id: String },
    InvalidSourceProfileId { event_id: String },
    InvalidTimeConversionReceiptDigest { event_id: String },
    InvalidNoradId { event_id: String, norad_id: u32 },
    SelfPair { event_id: String, norad_id: u32 },
    TimeBasisMismatch {
        event_id: String,
        expected: String,
        actual: String,
    },
    MissingSourceClass { event_id: String },
    MissingPublicationPolicy { event_id: String },
}

/// Which side of a two-provider reconciliation exceeded its declared input bound.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum ProviderSide {
    Left,
    Right,
}

type PairKey = (u32, u32);

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

fn pair_key(event: &ProviderConjunctionEvent) -> PairKey {
    if event.object_a_norad_id < event.object_b_norad_id {
        (event.object_a_norad_id, event.object_b_norad_id)
    } else {
        (event.object_b_norad_id, event.object_a_norad_id)
    }
}

fn validate_feed(
    events: &[ProviderConjunctionEvent],
    config: &EventReconciliationConfig,
    provider_side: ProviderSide,
) -> Result<(), EventReconciliationError> {
    let mut event_ids = BTreeSet::new();
    let mut provider_id: Option<&str> = None;

    for event in events {
        if event.provider_id.trim().is_empty() {
            return Err(EventReconciliationError::EmptyProviderId {
                event_id: event.event_id.clone(),
            });
        }
        if let Some(expected) = provider_id {
            if expected != event.provider_id.as_str() {
                return Err(EventReconciliationError::MixedProviderFeed {
                    provider_side,
                    expected_provider_id: expected.to_string(),
                    actual_provider_id: event.provider_id.clone(),
                });
            }
        } else {
            provider_id = Some(&event.provider_id);
        }
        if event.event_id.trim().is_empty() {
            return Err(EventReconciliationError::EmptyEventId);
        }
        if !event_ids.insert(event.event_id.as_str()) {
            return Err(EventReconciliationError::DuplicateEventId {
                event_id: event.event_id.clone(),
            });
        }
        if !is_sha256_hex(&event.source_artifact_sha256) {
            return Err(EventReconciliationError::InvalidSourceArtifactDigest {
                event_id: event.event_id.clone(),
            });
        }
        if event.source_profile_id.trim().is_empty() {
            return Err(EventReconciliationError::InvalidSourceProfileId {
                event_id: event.event_id.clone(),
            });
        }
        if !is_sha256_hex(&event.time_conversion_receipt_sha256) {
            return Err(EventReconciliationError::InvalidTimeConversionReceiptDigest {
                event_id: event.event_id.clone(),
            });
        }
        for norad_id in [event.object_a_norad_id, event.object_b_norad_id] {
            if norad_id == 0 || norad_id > 999_999 {
                return Err(EventReconciliationError::InvalidNoradId {
                    event_id: event.event_id.clone(),
                    norad_id,
                });
            }
        }
        if event.object_a_norad_id == event.object_b_norad_id {
            return Err(EventReconciliationError::SelfPair {
                event_id: event.event_id.clone(),
                norad_id: event.object_a_norad_id,
            });
        }
        if event.time_basis_id != config.canonical_time_basis_id {
            return Err(EventReconciliationError::TimeBasisMismatch {
                event_id: event.event_id.clone(),
                expected: config.canonical_time_basis_id.clone(),
                actual: event.time_basis_id.clone(),
            });
        }
        if event.source_class.trim().is_empty() {
            return Err(EventReconciliationError::MissingSourceClass {
                event_id: event.event_id.clone(),
            });
        }
        if event.publication_policy_id.trim().is_empty() {
            return Err(EventReconciliationError::MissingPublicationPolicy {
                event_id: event.event_id.clone(),
            });
        }
    }

    Ok(())
}

/// Reconcile two providers' event lists using object-pair identity and TCA.
///
/// The object pair is treated as unordered because provider A/B roles can
/// differ. A candidate edge exists when the canonical TAI TCA values differ
/// by no more than the inclusive configured tolerance.
///
/// Within each object pair, the candidate graph is decomposed into connected
/// components. A component containing exactly one left and one right event is
/// a unique match. Components with multiple candidates on either side are
/// returned as ambiguous. Events with no candidate edge remain provider-only.
///
/// This deliberately conservative rule avoids arbitrary greedy matches. It
/// does not rank source quality, normalize publication policies, compare Pc,
/// or establish that either provider's event is physically correct.
pub fn reconcile_provider_events(
    left: &[ProviderConjunctionEvent],
    right: &[ProviderConjunctionEvent],
    config: &EventReconciliationConfig,
) -> Result<EventReconciliation, EventReconciliationError> {
    if config.canonical_time_basis_id.trim().is_empty() {
        return Err(EventReconciliationError::EmptyCanonicalTimeBasis);
    }
    if config.tca_tolerance_micros > i64::MAX as u64 {
        return Err(EventReconciliationError::ToleranceExceedsSupportedRange);
    }
    if config.max_events_per_provider == 0 || config.max_candidate_edges == 0 {
        return Err(EventReconciliationError::ZeroEventLimit);
    }
    if left.len() > config.max_events_per_provider {
        return Err(EventReconciliationError::EventCountLimitExceeded {
            provider_side: ProviderSide::Left,
            count: left.len(),
            limit: config.max_events_per_provider,
        });
    }
    if right.len() > config.max_events_per_provider {
        return Err(EventReconciliationError::EventCountLimitExceeded {
            provider_side: ProviderSide::Right,
            count: right.len(),
            limit: config.max_events_per_provider,
        });
    }

    validate_feed(left, config, ProviderSide::Left)?;
    validate_feed(right, config, ProviderSide::Right)?;

    let mut left_by_pair: BTreeMap<PairKey, Vec<usize>> = BTreeMap::new();
    let mut right_by_pair: BTreeMap<PairKey, Vec<usize>> = BTreeMap::new();
    for (index, event) in left.iter().enumerate() {
        left_by_pair.entry(pair_key(event)).or_default().push(index);
    }
    for (index, event) in right.iter().enumerate() {
        right_by_pair.entry(pair_key(event)).or_default().push(index);
    }

    let mut all_pairs = BTreeSet::new();
    all_pairs.extend(left_by_pair.keys().copied());
    all_pairs.extend(right_by_pair.keys().copied());

    let mut result = EventReconciliation::default();
    let tolerance = config.tca_tolerance_micros as i128;
    let mut candidate_edge_count = 0usize;

    for key in all_pairs {
        let mut left_indices = left_by_pair.remove(&key).unwrap_or_default();
        let mut right_indices = right_by_pair.remove(&key).unwrap_or_default();

        if left_indices.is_empty() {
            result.right_only_event_ids.extend(
                right_indices
                    .iter()
                    .map(|index| right[*index].event_id.clone()),
            );
            continue;
        }
        if right_indices.is_empty() {
            result.left_only_event_ids.extend(
                left_indices
                    .iter()
                    .map(|index| left[*index].event_id.clone()),
            );
            continue;
        }

        left_indices.sort_by(|a, b| {
            left[*a]
                .tca_tai_micros
                .cmp(&left[*b].tca_tai_micros)
                .then_with(|| left[*a].event_id.cmp(&left[*b].event_id))
        });
        right_indices.sort_by(|a, b| {
            right[*a]
                .tca_tai_micros
                .cmp(&right[*b].tca_tai_micros)
                .then_with(|| right[*a].event_id.cmp(&right[*b].event_id))
        });

        // Build the TCA-tolerance bipartite graph with a sliding window.
        // This is O(L log L + R log R + E) per object pair, where E is the
        // number of candidate edges inside the declared TCA tolerance.
        let mut left_edges = vec![Vec::<usize>::new(); left_indices.len()];
        let mut right_edges = vec![Vec::<usize>::new(); right_indices.len()];
        let mut window_start = 0usize;
        let mut window_end = 0usize;

        for (left_local, left_global) in left_indices.iter().enumerate() {
            let left_tca = left[*left_global].tca_tai_micros as i128;
            let lower = left_tca - tolerance;
            let upper = left_tca + tolerance;

            while window_start < right_indices.len()
                && (right[right_indices[window_start]].tca_tai_micros as i128) < lower
            {
                window_start += 1;
            }
            if window_end < window_start {
                window_end = window_start;
            }
            while window_end < right_indices.len()
                && (right[right_indices[window_end]].tca_tai_micros as i128) <= upper
            {
                window_end += 1;
            }

            for right_local in window_start..window_end {
                candidate_edge_count = candidate_edge_count.saturating_add(1);
                if candidate_edge_count > config.max_candidate_edges {
                    return Err(EventReconciliationError::CandidateEdgeLimitExceeded {
                        count: candidate_edge_count,
                        limit: config.max_candidate_edges,
                    });
                }
                left_edges[left_local].push(right_local);
                right_edges[right_local].push(left_local);
            }
        }

        let mut seen_left = vec![false; left_indices.len()];
        let mut seen_right = vec![false; right_indices.len()];

        // Isolated left events are provider-only.
        for left_local in 0..left_indices.len() {
            if left_edges[left_local].is_empty() {
                result
                    .left_only_event_ids
                    .push(left[left_indices[left_local]].event_id.clone());
                seen_left[left_local] = true;
            }
        }
        // Isolated right events are provider-only.
        for right_local in 0..right_indices.len() {
            if right_edges[right_local].is_empty() {
                result
                    .right_only_event_ids
                    .push(right[right_indices[right_local]].event_id.clone());
                seen_right[right_local] = true;
            }
        }

        // Resolve each non-isolated connected component. Multi-candidate
        // components are never collapsed into an arbitrary one-to-one match.
        for seed_left in 0..left_indices.len() {
            if seen_left[seed_left] || left_edges[seed_left].is_empty() {
                continue;
            }

            let mut queue = VecDeque::new();
            let mut component_left = Vec::new();
            let mut component_right = Vec::new();
            seen_left[seed_left] = true;
            queue.push_back((true, seed_left));

            while let Some((is_left, local_index)) = queue.pop_front() {
                if is_left {
                    component_left.push(local_index);
                    for &right_local in &left_edges[local_index] {
                        if !seen_right[right_local] {
                            seen_right[right_local] = true;
                            queue.push_back((false, right_local));
                        }
                    }
                } else {
                    component_right.push(local_index);
                    for &left_local in &right_edges[local_index] {
                        if !seen_left[left_local] {
                            seen_left[left_local] = true;
                            queue.push_back((true, left_local));
                        }
                    }
                }
            }

            if component_left.len() == 1 && component_right.len() == 1 {
                let left_event = &left[left_indices[component_left[0]]];
                let right_event = &right[right_indices[component_right[0]]];
                let delta = right_event.tca_tai_micros as i128
                    - left_event.tca_tai_micros as i128;
                result.matched.push(MatchedProviderEvent {
                    left_provider_id: left_event.provider_id.clone(),
                    right_provider_id: right_event.provider_id.clone(),
                    left_event_id: left_event.event_id.clone(),
                    right_event_id: right_event.event_id.clone(),
                    left_source_artifact_sha256: left_event.source_artifact_sha256.clone(),
                    right_source_artifact_sha256: right_event.source_artifact_sha256.clone(),
                    left_source_profile_id: left_event.source_profile_id.clone(),
                    right_source_profile_id: right_event.source_profile_id.clone(),
                    left_time_conversion_receipt_sha256: left_event
                        .time_conversion_receipt_sha256
                        .clone(),
                    right_time_conversion_receipt_sha256: right_event
                        .time_conversion_receipt_sha256
                        .clone(),
                    delta_tca_micros: delta as i64,
                    object_order_swapped: left_event.object_a_norad_id
                        == right_event.object_b_norad_id
                        && left_event.object_b_norad_id == right_event.object_a_norad_id,
                    left_source_class: left_event.source_class.clone(),
                    right_source_class: right_event.source_class.clone(),
                    left_publication_policy_id: left_event.publication_policy_id.clone(),
                    right_publication_policy_id: right_event.publication_policy_id.clone(),
                });
            } else {
                let mut left_ids: Vec<String> = component_left
                    .iter()
                    .map(|index| left[left_indices[*index]].event_id.clone())
                    .collect();
                let mut right_ids: Vec<String> = component_right
                    .iter()
                    .map(|index| right[right_indices[*index]].event_id.clone())
                    .collect();
                left_ids.sort();
                right_ids.sort();
                result.ambiguous.push(AmbiguousProviderEventGroup {
                    left_event_ids: left_ids,
                    right_event_ids: right_ids,
                });
            }
        }
    }

    result.matched.sort_by(|a, b| {
        a.left_event_id
            .cmp(&b.left_event_id)
            .then_with(|| a.right_event_id.cmp(&b.right_event_id))
    });
    result.left_only_event_ids.sort();
    result.right_only_event_ids.sort();
    result.ambiguous.sort_by(|a, b| {
        a.left_event_ids
            .cmp(&b.left_event_ids)
            .then_with(|| a.right_event_ids.cmp(&b.right_event_ids))
    });

    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    const BASIS: &str = "TAI-unix-epoch-micros+UTC-conversion-table-2026-01";

    fn config(tolerance: u64) -> EventReconciliationConfig {
        EventReconciliationConfig {
            canonical_time_basis_id: BASIS.to_string(),
            tca_tolerance_micros: tolerance,
            max_events_per_provider: 100_000,
            max_candidate_edges: 1_000_000,
        }
    }

    fn event(id: &str, a: u32, b: u32, tca: i64) -> ProviderConjunctionEvent {
        ProviderConjunctionEvent {
            provider_id: "provider-a".to_string(),
            event_id: id.to_string(),
            source_artifact_sha256: "a".repeat(64),
            source_profile_id: "tracss-cdm-v2.1-json".to_string(),
            object_a_norad_id: a,
            object_b_norad_id: b,
            tca_tai_micros: tca,
            time_basis_id: BASIS.to_string(),
            time_conversion_receipt_sha256: "b".repeat(64),
            source_class: "operator-ephemeris-with-covariance".to_string(),
            publication_policy_id: "policy-v1".to_string(),
        }
    }

    #[test]
    fn matches_unordered_object_pair_within_inclusive_tolerance() {
        let result = reconcile_provider_events(
            &[event("left-1", 25544, 49863, 1_000)],
            &[event("right-1", 49863, 25544, 1_100)],
            &config(100),
        )
        .unwrap();

        assert_eq!(result.matched.len(), 1);
        assert_eq!(result.matched[0].delta_tca_micros, 100);
        assert!(result.matched[0].object_order_swapped);
        assert!(result.left_only_event_ids.is_empty());
        assert!(result.right_only_event_ids.is_empty());
    }

    #[test]
    fn preserves_provider_only_events_outside_tca_tolerance() {
        let result = reconcile_provider_events(
            &[event("left-1", 25544, 49863, 1_000)],
            &[event("right-1", 25544, 49863, 1_101)],
            &config(100),
        )
        .unwrap();

        assert_eq!(result.left_only_event_ids, vec!["left-1"]);
        assert_eq!(result.right_only_event_ids, vec!["right-1"]);
        assert!(result.matched.is_empty());
    }

    #[test]
    fn marks_many_to_one_candidate_group_ambiguous_instead_of_guessing() {
        let result = reconcile_provider_events(
            &[
                event("left-1", 25544, 49863, 1_000),
                event("left-2", 25544, 49863, 1_010),
            ],
            &[event("right-1", 25544, 49863, 1_005)],
            &config(10),
        )
        .unwrap();

        assert_eq!(result.ambiguous.len(), 1);
        assert_eq!(result.ambiguous[0].left_event_ids, vec!["left-1", "left-2"]);
        assert_eq!(result.ambiguous[0].right_event_ids, vec!["right-1"]);
        assert!(result.matched.is_empty());
    }

    #[test]
    fn matches_separate_encounters_for_the_same_object_pair() {
        let result = reconcile_provider_events(
            &[
                event("left-late", 25544, 49863, 10_000),
                event("left-early", 25544, 49863, 100),
            ],
            &[
                event("right-early", 49863, 25544, 110),
                event("right-late", 49863, 25544, 10_010),
            ],
            &config(20),
        )
        .unwrap();

        assert_eq!(result.matched.len(), 2);
        assert_eq!(result.matched[0].left_event_id, "left-early");
        assert_eq!(result.matched[1].left_event_id, "left-late");
    }

    #[test]
    fn canonical_time_basis_mismatch_is_rejected() {
        let mut input = event("left-1", 25544, 49863, 1_000);
        input.time_basis_id = "UTC-un-normalized".to_string();
        assert_eq!(
            reconcile_provider_events(&[input], &[], &config(10)),
            Err(EventReconciliationError::TimeBasisMismatch {
                event_id: "left-1".to_string(),
                expected: BASIS.to_string(),
                actual: "UTC-un-normalized".to_string(),
            })
        );
    }

    #[test]
    fn duplicate_event_ids_are_rejected_within_each_provider_feed() {
        assert_eq!(
            reconcile_provider_events(
                &[
                    event("duplicate", 25544, 49863, 1_000),
                    event("duplicate", 25544, 49863, 2_000),
                ],
                &[],
                &config(10),
            ),
            Err(EventReconciliationError::DuplicateEventId {
                event_id: "duplicate".to_string(),
            })
        );
    }

    #[test]
    fn invalid_and_self_pair_norad_ids_are_rejected() {
        assert_eq!(
            reconcile_provider_events(
                &[event("bad-zero", 0, 49863, 1_000)],
                &[],
                &config(10),
            ),
            Err(EventReconciliationError::InvalidNoradId {
                event_id: "bad-zero".to_string(),
                norad_id: 0,
            })
        );
        assert_eq!(
            reconcile_provider_events(
                &[event("self-pair", 25544, 25544, 1_000)],
                &[],
                &config(10),
            ),
            Err(EventReconciliationError::SelfPair {
                event_id: "self-pair".to_string(),
                norad_id: 25544,
            })
        );
    }

    #[test]
    fn output_is_deterministic_under_input_permutation() {
        let left_a = [
            event("left-b", 25544, 49863, 2_000),
            event("left-a", 25544, 49863, 1_000),
        ];
        let right_a = [
            event("right-b", 49863, 25544, 2_001),
            event("right-a", 49863, 25544, 1_001),
        ];
        let left_b = [left_a[1].clone(), left_a[0].clone()];
        let right_b = [right_a[1].clone(), right_a[0].clone()];

        assert_eq!(
            reconcile_provider_events(&left_a, &right_a, &config(10)),
            reconcile_provider_events(&left_b, &right_b, &config(10))
        );
    }

    #[test]
    fn preserves_source_class_and_publication_policy_on_matches() {
        let mut left = event("left", 25544, 49863, 1_000);
        let mut right = event("right", 25544, 49863, 1_000);
        right.provider_id = "provider-b".to_string();
        left.source_class = "government-catalog-no-covariance".to_string();
        left.publication_policy_id = "policy-actionable-only".to_string();
        right.source_class = "operator-ephemeris-with-covariance".to_string();
        right.publication_policy_id = "policy-comprehensive".to_string();

        let result =
            reconcile_provider_events(&[left], &[right], &config(0)).unwrap();
        assert_eq!(result.matched[0].left_provider_id, "provider-a");
        assert_eq!(result.matched[0].right_provider_id, "provider-b");
        assert_eq!(result.matched[0].left_source_artifact_sha256, "a".repeat(64));
        assert_eq!(result.matched[0].left_source_profile_id, "tracss-cdm-v2.1-json");
        assert_eq!(
            result.matched[0].left_time_conversion_receipt_sha256,
            "b".repeat(64)
        );
        assert_eq!(
            result.matched[0].left_source_class,
            "government-catalog-no-covariance"
        );
        assert_eq!(
            result.matched[0].right_source_class,
            "operator-ephemeris-with-covariance"
        );
        assert_eq!(
            result.matched[0].left_publication_policy_id,
            "policy-actionable-only"
        );
        assert_eq!(
            result.matched[0].right_publication_policy_id,
            "policy-comprehensive"
        );
    }

    #[test]
    fn rejects_empty_provenance_labels_and_unrepresentable_tolerance() {
        let mut input = event("left", 25544, 49863, 1_000);
        input.source_class.clear();
        assert_eq!(
            reconcile_provider_events(&[input], &[], &config(10)),
            Err(EventReconciliationError::MissingSourceClass {
                event_id: "left".to_string(),
            })
        );

        assert_eq!(
            reconcile_provider_events(
                &[],
                &[],
                &config(i64::MAX as u64 + 1),
            ),
            Err(EventReconciliationError::ToleranceExceedsSupportedRange)
        );
    }

    #[test]
    fn rejects_mixed_provider_ids_within_one_input_feed() {
        let mut second_left_event = event("left-2", 25544, 49863, 1_010);
        second_left_event.provider_id = "provider-b".to_string();
        assert_eq!(
            reconcile_provider_events(
                &[event("left-1", 25544, 49863, 1_000), second_left_event],
                &[],
                &config(10),
            ),
            Err(EventReconciliationError::MixedProviderFeed {
                provider_side: ProviderSide::Left,
                expected_provider_id: "provider-a".to_string(),
                actual_provider_id: "provider-b".to_string(),
            })
        );
    }

    #[test]
    fn rejects_empty_provider_and_missing_publication_policy() {
        let mut input = event("empty-provider", 25544, 49863, 1_000);
        input.provider_id.clear();
        assert_eq!(
            reconcile_provider_events(&[input], &[], &config(10)),
            Err(EventReconciliationError::EmptyProviderId {
                event_id: "empty-provider".to_string(),
            })
        );

        let mut input = event("missing-policy", 25544, 49863, 1_000);
        input.publication_policy_id.clear();
        assert_eq!(
            reconcile_provider_events(&[input], &[], &config(10)),
            Err(EventReconciliationError::MissingPublicationPolicy {
                event_id: "missing-policy".to_string(),
            })
        );
    }

    #[test]
    fn rejects_missing_or_invalid_source_artifact_and_time_receipt_digests() {
        let mut input = event("bad-source-digest", 25544, 49863, 1_000);
        input.source_artifact_sha256 = "not-a-digest".to_string();
        assert_eq!(
            reconcile_provider_events(&[input], &[], &config(10)),
            Err(EventReconciliationError::InvalidSourceArtifactDigest {
                event_id: "bad-source-digest".to_string(),
            })
        );

        let mut input = event("bad-time-receipt", 25544, 49863, 1_000);
        input.time_conversion_receipt_sha256 = "not-a-digest".to_string();
        assert_eq!(
            reconcile_provider_events(&[input], &[], &config(10)),
            Err(EventReconciliationError::InvalidTimeConversionReceiptDigest {
                event_id: "bad-time-receipt".to_string(),
            })
        );
    }

    #[test]
    fn rejects_event_and_candidate_graph_resource_limit_exhaustion() {
        let mut policy = config(10);
        policy.max_events_per_provider = 1;
        assert_eq!(
            reconcile_provider_events(
                &[
                    event("left-1", 25544, 49863, 1_000),
                    event("left-2", 25544, 49863, 1_000),
                ],
                &[],
                &policy,
            ),
            Err(EventReconciliationError::EventCountLimitExceeded {
                provider_side: ProviderSide::Left,
                count: 2,
                limit: 1,
            })
        );

        let mut policy = config(10);
        policy.max_candidate_edges = 1;
        assert_eq!(
            reconcile_provider_events(
                &[
                    event("left-1", 25544, 49863, 1_000),
                    event("left-2", 25544, 49863, 1_001),
                ],
                &[event("right-1", 25544, 49863, 1_000)],
                &policy,
            ),
            Err(EventReconciliationError::CandidateEdgeLimitExceeded {
                count: 2,
                limit: 1,
            })
        );
    }

    #[test]
    fn handles_extreme_timestamps_without_signed_overflow() {
        let result = reconcile_provider_events(
            &[event("left-old", 25544, 49863, i64::MIN)],
            &[event("right-new", 25544, 49863, i64::MAX)],
            &config(1),
        )
        .unwrap();

        assert_eq!(result.left_only_event_ids, vec!["left-old"]);
        assert_eq!(result.right_only_event_ids, vec!["right-new"]);
    }
}
