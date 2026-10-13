// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
#![forbid(unsafe_code)]

pub mod rights;

//! Deterministic, simulation-only music creator revenue allocation policies.
//!
//! This crate compares payout rules over pre-aggregated inputs. It does not
//! authenticate playback, make rights determinations, or execute transfers.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

/// One listener's contribution to a single, consistently denominated creator pool.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ListenerContribution {
    /// Pseudonymous identifier used only to group one contribution and its plays.
    pub listener_id: String,
    /// Contribution in integer minor units of one currency/unit.
    pub amount_minor_units: u64,
    /// Pre-aggregated eligible play counts, at most one row per artist.
    pub plays: Vec<ArtistPlay>,
}

/// An artist's eligible aggregate play count for one listener.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ArtistPlay {
    /// Stable artist identifier; this is not an identity or rights verification.
    pub artist_id: String,
    pub play_count: u64,
}

/// Inputs to a deterministic allocation run.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct AllocationInput {
    /// Contributions and per-listener aggregated eligible plays.
    pub listeners: Vec<ListenerContribution>,
    /// Required only for CooperativeEqualShare; can include members with no plays.
    pub cooperative_members: Vec<String>,
}

/// Allocation policy. Each rule answers a different normative question.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AllocationPolicy {
    /// Allocate the entire creator pool by total platform-wide eligible plays.
    ProRata,
    /// Allocate each listener contribution in proportion to that listener's plays.
    UserCentric,
    /// Divide each listener contribution equally among artists that listener played.
    ShapleyDistinctArtist,
    /// Divide the pool equally among an explicit cooperative membership list.
    CooperativeEqualShare,
}

/// Result of one allocation. Unallocated value is explicit and never silently dropped.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AllocationReport {
    pub policy: AllocationPolicy,
    pub pool_minor_units: u64,
    pub payouts_minor_units: BTreeMap<String, u64>,
    pub unallocated_minor_units: u64,
}

impl AllocationReport {
    /// Check the conservation identity: payouts + explicitly unallocated = input pool.
    pub fn reconciles(&self) -> bool {
        self.payouts_minor_units
            .values()
            .try_fold(0_u64, |sum, value| sum.checked_add(*value))
            .and_then(|paid| paid.checked_add(self.unallocated_minor_units))
            == Some(self.pool_minor_units)
    }

    pub fn paid_minor_units(&self) -> Option<u64> {
        self.payouts_minor_units
            .values()
            .try_fold(0_u64, |sum, value| sum.checked_add(*value))
    }
}

/// Input validation or exact-accounting failure.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AllocationError {
    InvalidListenerId(String),
    DuplicateListenerId(String),
    InvalidArtistId {
        listener_id: String,
        artist_id: String,
    },
    DuplicateArtistForListener {
        listener_id: String,
        artist_id: String,
    },
    ZeroPlayCount {
        listener_id: String,
        artist_id: String,
    },
    InvalidCooperativeMemberId(String),
    DuplicateCooperativeMember(String),
    NoCooperativeMembers,
    NoEligibleWeights,
    ArithmeticOverflow,
    ConservationViolation {
        pool_minor_units: u64,
        paid_minor_units: u64,
        unallocated_minor_units: u64,
    },
}

impl fmt::Display for AllocationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidListenerId(id) => {
                write!(f, "listener ID must be non-empty and trimmed: {id:?}")
            }
            Self::DuplicateListenerId(id) => write!(f, "duplicate listener ID: {id:?}"),
            Self::InvalidArtistId {
                listener_id,
                artist_id,
            } => write!(
                f,
                "artist ID must be non-empty and trimmed for listener {listener_id:?}: {artist_id:?}"
            ),
            Self::DuplicateArtistForListener {
                listener_id,
                artist_id,
            } => write!(
                f,
                "duplicate artist row for listener {listener_id:?}: {artist_id:?}"
            ),
            Self::ZeroPlayCount {
                listener_id,
                artist_id,
            } => write!(
                f,
                "play count must be positive for listener {listener_id:?}, artist {artist_id:?}"
            ),
            Self::InvalidCooperativeMemberId(id) => {
                write!(f, "cooperative member ID must be non-empty and trimmed: {id:?}")
            }
            Self::DuplicateCooperativeMember(id) => {
                write!(f, "duplicate cooperative member ID: {id:?}")
            }
            Self::NoCooperativeMembers => {
                write!(f, "cooperative equal-share policy requires at least one member")
            }
            Self::NoEligibleWeights => write!(f, "allocation requires positive eligible weights"),
            Self::ArithmeticOverflow => write!(f, "money or play-weight arithmetic overflow"),
            Self::ConservationViolation {
                pool_minor_units,
                paid_minor_units,
                unallocated_minor_units,
            } => write!(
                f,
                "pool does not reconcile: pool={pool_minor_units}, paid={paid_minor_units}, unallocated={unallocated_minor_units}"
            ),
        }
    }
}

impl std::error::Error for AllocationError {}

/// Compute artist-level payouts without executing payments.
///
/// The contribution pool is the checked sum of listener contribution amounts.
/// Play records must already be eligible according to an independently specified
/// upstream rule. The returned payout map is deterministically ordered by artist ID.
pub fn allocate(
    input: &AllocationInput,
    policy: AllocationPolicy,
) -> Result<AllocationReport, AllocationError> {
    let pool_minor_units = validate_input(input)?;
    let mut payouts_minor_units = BTreeMap::new();
    let mut unallocated_minor_units = 0_u64;

    match policy {
        AllocationPolicy::ProRata => {
            let mut weights = BTreeMap::new();
            for listener in &input.listeners {
                for play in &listener.plays {
                    add_weight(&mut weights, &play.artist_id, play.play_count)?;
                }
            }

            if weights.is_empty() {
                unallocated_minor_units = pool_minor_units;
            } else {
                allocate_weighted(pool_minor_units, &weights, &mut payouts_minor_units)?;
            }
        }
        AllocationPolicy::UserCentric => {
            for listener in &input.listeners {
                if listener.plays.is_empty() {
                    add_unallocated(
                        &mut unallocated_minor_units,
                        listener.amount_minor_units,
                    )?;
                    continue;
                }

                let weights = play_weights(listener);
                allocate_weighted(
                    listener.amount_minor_units,
                    &weights,
                    &mut payouts_minor_units,
                )?;
            }
        }
        AllocationPolicy::ShapleyDistinctArtist => {
            for listener in &input.listeners {
                if listener.plays.is_empty() {
                    add_unallocated(
                        &mut unallocated_minor_units,
                        listener.amount_minor_units,
                    )?;
                    continue;
                }

                let weights = listener
                    .plays
                    .iter()
                    .map(|play| (play.artist_id.clone(), 1_u64))
                    .collect();
                allocate_weighted(
                    listener.amount_minor_units,
                    &weights,
                    &mut payouts_minor_units,
                )?;
            }
        }
        AllocationPolicy::CooperativeEqualShare => {
            let weights = cooperative_weights(&input.cooperative_members)?;
            allocate_weighted(pool_minor_units, &weights, &mut payouts_minor_units)?;
        }
    }

    let paid_minor_units = payouts_minor_units
        .values()
        .try_fold(0_u64, |sum, amount| sum.checked_add(*amount))
        .ok_or(AllocationError::ArithmeticOverflow)?;
    let accounted_minor_units = paid_minor_units
        .checked_add(unallocated_minor_units)
        .ok_or(AllocationError::ArithmeticOverflow)?;

    if accounted_minor_units != pool_minor_units {
        return Err(AllocationError::ConservationViolation {
            pool_minor_units,
            paid_minor_units,
            unallocated_minor_units,
        });
    }

    let report = AllocationReport {
        policy,
        pool_minor_units,
        payouts_minor_units,
        unallocated_minor_units,
    };
    debug_assert!(report.reconciles());
    Ok(report)
}

fn validate_input(input: &AllocationInput) -> Result<u64, AllocationError> {
    let mut listener_ids = BTreeSet::new();
    let mut pool_minor_units = 0_u64;

    for listener in &input.listeners {
        if !is_valid_id(&listener.listener_id) {
            return Err(AllocationError::InvalidListenerId(
                listener.listener_id.clone(),
            ));
        }
        if !listener_ids.insert(listener.listener_id.as_str()) {
            return Err(AllocationError::DuplicateListenerId(
                listener.listener_id.clone(),
            ));
        }

        pool_minor_units = pool_minor_units
            .checked_add(listener.amount_minor_units)
            .ok_or(AllocationError::ArithmeticOverflow)?;

        let mut artist_ids = BTreeSet::new();
        for play in &listener.plays {
            if !is_valid_id(&play.artist_id) {
                return Err(AllocationError::InvalidArtistId {
                    listener_id: listener.listener_id.clone(),
                    artist_id: play.artist_id.clone(),
                });
            }
            if !artist_ids.insert(play.artist_id.as_str()) {
                return Err(AllocationError::DuplicateArtistForListener {
                    listener_id: listener.listener_id.clone(),
                    artist_id: play.artist_id.clone(),
                });
            }
            if play.play_count == 0 {
                return Err(AllocationError::ZeroPlayCount {
                    listener_id: listener.listener_id.clone(),
                    artist_id: play.artist_id.clone(),
                });
            }
        }
    }

    Ok(pool_minor_units)
}

fn is_valid_id(value: &str) -> bool {
    !value.trim().is_empty() && value.trim() == value
}

fn play_weights(listener: &ListenerContribution) -> BTreeMap<String, u64> {
    listener
        .plays
        .iter()
        .map(|play| (play.artist_id.clone(), play.play_count))
        .collect()
}

fn cooperative_weights(members: &[String]) -> Result<BTreeMap<String, u64>, AllocationError> {
    if members.is_empty() {
        return Err(AllocationError::NoCooperativeMembers);
    }

    let mut weights = BTreeMap::new();
    for member in members {
        if !is_valid_id(member) {
            return Err(AllocationError::InvalidCooperativeMemberId(
                member.clone(),
            ));
        }
        if weights.insert(member.clone(), 1_u64).is_some() {
            return Err(AllocationError::DuplicateCooperativeMember(
                member.clone(),
            ));
        }
    }
    Ok(weights)
}

fn add_weight(
    weights: &mut BTreeMap<String, u64>,
    artist_id: &str,
    additional_weight: u64,
) -> Result<(), AllocationError> {
    let weight = weights.entry(artist_id.to_owned()).or_insert(0);
    *weight = weight
        .checked_add(additional_weight)
        .ok_or(AllocationError::ArithmeticOverflow)?;
    Ok(())
}

fn add_unallocated(
    unallocated: &mut u64,
    amount_minor_units: u64,
) -> Result<(), AllocationError> {
    *unallocated = unallocated
        .checked_add(amount_minor_units)
        .ok_or(AllocationError::ArithmeticOverflow)?;
    Ok(())
}

fn add_payout(
    payouts: &mut BTreeMap<String, u64>,
    artist_id: &str,
    amount_minor_units: u64,
) -> Result<(), AllocationError> {
    let payout = payouts.entry(artist_id.to_owned()).or_insert(0);
    *payout = payout
        .checked_add(amount_minor_units)
        .ok_or(AllocationError::ArithmeticOverflow)?;
    Ok(())
}

/// Largest-remainder apportionment, with stable artist-ID tie-breaking.
fn allocate_weighted(
    amount_minor_units: u64,
    weights: &BTreeMap<String, u64>,
    payouts: &mut BTreeMap<String, u64>,
) -> Result<(), AllocationError> {
    let total_weight = weights
        .values()
        .try_fold(0_u64, |sum, weight| sum.checked_add(*weight))
        .ok_or(AllocationError::ArithmeticOverflow)?;
    if total_weight == 0 {
        return Err(AllocationError::NoEligibleWeights);
    }

    let mut fractional_remainders = Vec::with_capacity(weights.len());
    let mut assigned_minor_units = 0_u64;

    for (artist_id, weight) in weights {
        if *weight == 0 {
            continue;
        }

        // A u64 amount times a u64 weight always fits in u128.
        let numerator = u128::from(amount_minor_units) * u128::from(*weight);
        let denominator = u128::from(total_weight);
        let base_share = u64::try_from(numerator / denominator)
            .map_err(|_| AllocationError::ArithmeticOverflow)?;
        let remainder = u64::try_from(numerator % denominator)
            .map_err(|_| AllocationError::ArithmeticOverflow)?;

        add_payout(payouts, artist_id, base_share)?;
        assigned_minor_units = assigned_minor_units
            .checked_add(base_share)
            .ok_or(AllocationError::ArithmeticOverflow)?;
        fractional_remainders.push((artist_id.clone(), remainder));
    }

    let leftover = amount_minor_units
        .checked_sub(assigned_minor_units)
        .ok_or(AllocationError::ArithmeticOverflow)?;

    fractional_remainders.sort_by(|(left_id, left_remainder), (right_id, right_remainder)| {
        right_remainder
            .cmp(left_remainder)
            .then_with(|| left_id.cmp(right_id))
    });

    // The largest-remainder method guarantees leftover < positive recipient count.
    for (artist_id, _) in fractional_remainders.into_iter().take(leftover as usize) {
        add_payout(payouts, &artist_id, 1)?;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn listener(id: &str, amount: u64, plays: &[(&str, u64)]) -> ListenerContribution {
        ListenerContribution {
            listener_id: id.to_owned(),
            amount_minor_units: amount,
            plays: plays
                .iter()
                .map(|(artist_id, play_count)| ArtistPlay {
                    artist_id: (*artist_id).to_owned(),
                    play_count: *play_count,
                })
                .collect(),
        }
    }

    fn sample_input() -> AllocationInput {
        AllocationInput {
            listeners: vec![
                listener("listener-1", 101, &[("artist-a", 9), ("artist-b", 1)]),
                listener("listener-2", 99, &[("artist-b", 1), ("artist-c", 1)]),
            ],
            cooperative_members: vec![
                "artist-a".to_owned(),
                "artist-b".to_owned(),
                "artist-c".to_owned(),
                "artist-inactive".to_owned(),
            ],
        }
    }

    #[test]
    fn every_policy_reconciles_to_the_same_input_pool() {
        let input = sample_input();
        for policy in [
            AllocationPolicy::ProRata,
            AllocationPolicy::UserCentric,
            AllocationPolicy::ShapleyDistinctArtist,
            AllocationPolicy::CooperativeEqualShare,
        ] {
            let report = allocate(&input, policy).expect("valid sample allocation");
            assert_eq!(report.pool_minor_units, 200);
            assert!(report.reconciles(), "{policy:?} did not reconcile");
            assert_eq!(
                report.paid_minor_units().unwrap() + report.unallocated_minor_units,
                200
            );
        }
    }

    #[test]
    fn user_centric_and_shapley_answer_different_questions() {
        let input = sample_input();
        let user = allocate(&input, AllocationPolicy::UserCentric).unwrap();
        let shapley = allocate(&input, AllocationPolicy::ShapleyDistinctArtist).unwrap();

        assert_eq!(user.payouts_minor_units["artist-a"], 91);
        assert_eq!(user.payouts_minor_units["artist-b"], 60);
        assert_eq!(user.payouts_minor_units["artist-c"], 49);

        assert_eq!(shapley.payouts_minor_units["artist-a"], 51);
        assert_eq!(shapley.payouts_minor_units["artist-b"], 100);
        assert_eq!(shapley.payouts_minor_units["artist-c"], 49);
        assert_ne!(user.payouts_minor_units, shapley.payouts_minor_units);
    }

    #[test]
    fn pro_rata_uses_global_play_counts() {
        let report = allocate(&sample_input(), AllocationPolicy::ProRata).unwrap();
        assert_eq!(report.payouts_minor_units["artist-a"], 150);
        assert_eq!(report.payouts_minor_units["artist-b"], 33);
        assert_eq!(report.payouts_minor_units["artist-c"], 17);
        assert_eq!(report.unallocated_minor_units, 0);
    }

    #[test]
    fn largest_remainder_ties_are_broken_by_artist_id() {
        let input = AllocationInput {
            listeners: vec![listener("listener-1", 10, &[])],
            cooperative_members: vec![
                "artist-c".to_owned(),
                "artist-b".to_owned(),
                "artist-a".to_owned(),
            ],
        };
        let report = allocate(&input, AllocationPolicy::CooperativeEqualShare).unwrap();
        assert_eq!(report.payouts_minor_units["artist-a"], 4);
        assert_eq!(report.payouts_minor_units["artist-b"], 3);
        assert_eq!(report.payouts_minor_units["artist-c"], 3);
    }

    #[test]
    fn pro_rata_holds_pool_as_unallocated_when_there_are_no_plays() {
        let input = AllocationInput {
            listeners: vec![listener("listener-1", 123, &[])],
            cooperative_members: vec![],
        };
        let report = allocate(&input, AllocationPolicy::ProRata).unwrap();
        assert!(report.payouts_minor_units.is_empty());
        assert_eq!(report.unallocated_minor_units, 123);
        assert!(report.reconciles());
    }

    #[test]
    fn user_centric_keeps_contributions_without_plays_explicitly_unallocated() {
        let input = AllocationInput {
            listeners: vec![
                listener("active", 75, &[("artist-a", 3)]),
                listener("inactive", 25, &[]),
            ],
            cooperative_members: vec![],
        };
        let report = allocate(&input, AllocationPolicy::UserCentric).unwrap();
        assert_eq!(report.payouts_minor_units["artist-a"], 75);
        assert_eq!(report.unallocated_minor_units, 25);
        assert!(report.reconciles());
    }

    #[test]
    fn cooperative_share_can_include_member_without_plays() {
        let input = AllocationInput {
            listeners: vec![listener("listener-1", 12, &[("artist-a", 10)])],
            cooperative_members: vec![
                "artist-a".to_owned(),
                "artist-b".to_owned(),
                "artist-c".to_owned(),
            ],
        };
        let report = allocate(&input, AllocationPolicy::CooperativeEqualShare).unwrap();
        assert_eq!(report.payouts_minor_units["artist-a"], 4);
        assert_eq!(report.payouts_minor_units["artist-b"], 4);
        assert_eq!(report.payouts_minor_units["artist-c"], 4);
    }

    #[test]
    fn allocation_does_not_depend_on_input_order() {
        let original = sample_input();
        let mut reversed = sample_input();
        reversed.listeners.reverse();
        for listener in &mut reversed.listeners {
            listener.plays.reverse();
        }

        for policy in [
            AllocationPolicy::ProRata,
            AllocationPolicy::UserCentric,
            AllocationPolicy::ShapleyDistinctArtist,
            AllocationPolicy::CooperativeEqualShare,
        ] {
            assert_eq!(
                allocate(&original, policy).unwrap().payouts_minor_units,
                allocate(&reversed, policy).unwrap().payouts_minor_units
            );
        }
    }

    #[test]
    fn duplicate_listener_ids_are_rejected() {
        let input = AllocationInput {
            listeners: vec![
                listener("same", 1, &[("artist-a", 1)]),
                listener("same", 1, &[("artist-b", 1)]),
            ],
            cooperative_members: vec![],
        };
        assert!(matches!(
            allocate(&input, AllocationPolicy::ProRata),
            Err(AllocationError::DuplicateListenerId(_))
        ));
    }

    #[test]
    fn duplicate_artist_rows_per_listener_are_rejected() {
        let input = AllocationInput {
            listeners: vec![listener("listener-1", 1, &[("artist-a", 1), ("artist-a", 2)])],
            cooperative_members: vec![],
        };
        assert!(matches!(
            allocate(&input, AllocationPolicy::ProRata),
            Err(AllocationError::DuplicateArtistForListener { .. })
        ));
    }

    #[test]
    fn zero_play_count_is_rejected() {
        let input = AllocationInput {
            listeners: vec![listener("listener-1", 1, &[("artist-a", 0)])],
            cooperative_members: vec![],
        };
        assert!(matches!(
            allocate(&input, AllocationPolicy::ProRata),
            Err(AllocationError::ZeroPlayCount { .. })
        ));
    }

    #[test]
    fn contribution_pool_overflow_is_rejected() {
        let input = AllocationInput {
            listeners: vec![
                listener("listener-1", u64::MAX, &[]),
                listener("listener-2", 1, &[]),
            ],
            cooperative_members: vec![],
        };
        assert_eq!(
            allocate(&input, AllocationPolicy::ProRata),
            Err(AllocationError::ArithmeticOverflow)
        );
    }

    #[test]
    fn global_pro_rata_play_weight_overflow_is_rejected() {
        let input = AllocationInput {
            listeners: vec![
                listener("listener-1", 1, &[("artist-a", u64::MAX)]),
                listener("listener-2", 1, &[("artist-a", 1)]),
            ],
            cooperative_members: vec![],
        };
        assert_eq!(
            allocate(&input, AllocationPolicy::ProRata),
            Err(AllocationError::ArithmeticOverflow)
        );
    }

    #[test]
    fn cooperative_membership_must_be_non_empty_and_unique() {
        let empty = AllocationInput {
            listeners: vec![listener("listener-1", 1, &[])],
            cooperative_members: vec![],
        };
        assert_eq!(
            allocate(&empty, AllocationPolicy::CooperativeEqualShare),
            Err(AllocationError::NoCooperativeMembers)
        );

        let duplicate = AllocationInput {
            listeners: vec![listener("listener-1", 1, &[])],
            cooperative_members: vec!["artist-a".to_owned(), "artist-a".to_owned()],
        };
        assert!(matches!(
            allocate(&duplicate, AllocationPolicy::CooperativeEqualShare),
            Err(AllocationError::DuplicateCooperativeMember(_))
        ));
    }

    #[test]
    fn zero_pool_is_conserved() {
        let input = AllocationInput {
            listeners: vec![listener("listener-1", 0, &[("artist-a", 1)])],
            cooperative_members: vec![],
        };
        let report = allocate(&input, AllocationPolicy::UserCentric).unwrap();
        assert_eq!(report.pool_minor_units, 0);
        assert!(report.reconciles());
    }
}
