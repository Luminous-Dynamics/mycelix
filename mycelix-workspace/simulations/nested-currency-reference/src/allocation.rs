//! Deterministic pre-partition allocation of non-overlapping escrow risk rights.
//!
//! Policies choose where a fixed gross-exposure budget is placed before disconnection.
//! They never authorize transfer or resizing during a partition. This is a planning
//! reference model, not a distributed allocator or authority to spend.

use std::collections::BTreeSet;

pub const PARTS_PER_MILLION: u32 = 1_000_000;
pub const BASIS_POINTS: u16 = 10_000;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ShardDemand {
    /// Canonical currency/instrument identity for every demand sample and slice.
    /// Ticker symbols and display labels are not valid authority identifiers.
    pub instrument_id: String,
    /// Stable, canonical shard identifier. It is also the deterministic tie-break key.
    pub shard_id: String,
    /// Integer demand samples from a declared training window, in one canonical unit.
    pub samples: Vec<u64>,
    /// Optional minimum slice reserved for this shard before policy-weighted allocation.
    pub minimum_capacity: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AllocationPolicy {
    /// Give all shards equal weight, after satisfying explicit minimum slices.
    EqualSplit,
    /// Weight rights by the ceiling of the mean historical demand.
    DemandWeighted,
    /// Reserve an explicit fraction of discretionary capacity as equal per-shard
    /// slices, then distribute the remainder by historical mean demand. This trades
    /// some stationary utilization for resilience to demand shifts.
    DemandWeightedWithEqualFloor { equal_floor_ppm: u32 },
    /// Weight rights by a nearest-rank historical quantile plus a proportional margin.
    /// Quantile uses basis points: 5_000 = 50th percentile, 9_500 = 95th percentile.
    RobustQuantile {
        quantile_basis_points: u16,
        margin_ppm: u32,
    },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AllocationSlice {
    pub shard_id: String,
    pub minimum_capacity: u64,
    pub capacity: u64,
    /// Effective integer weight used by the policy; recorded for reproducibility.
    pub allocation_weight: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AllocationPlan {
    /// Exact instrument whose risk rights this plan allocates.
    pub instrument_id: String,
    pub total_capacity: u64,
    pub spendable_capacity: u64,
    pub contingency_reserve: u64,
    pub policy: AllocationPolicy,
    /// Sorted by shard ID to make serialization and receipts deterministic.
    pub slices: Vec<AllocationSlice>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AllocationError {
    NoShards,
    InvalidInstrumentId,
    InstrumentMismatch,
    InvalidShardId,
    DuplicateShardId,
    ReserveOutOfRange,
    FloorOutOfRange,
    QuantileOutOfRange,
    MarginOutOfRange,
    MissingDemandSamples,
    InsufficientCapacityForMinimums,
    Overflow,
    InternalAccountingViolation,
}

impl AllocationPlan {
    /// Recheck the integer conservation invariants independently of allocation policy.
    pub fn validate(&self) -> Result<(), AllocationError> {
        if self.instrument_id.trim().is_empty() {
            return Err(AllocationError::InvalidInstrumentId);
        }
        if self.slices.is_empty() {
            return Err(AllocationError::NoShards);
        }
        if self.contingency_reserve > self.total_capacity
            || self.spendable_capacity.checked_add(self.contingency_reserve)
                != Some(self.total_capacity)
        {
            return Err(AllocationError::InternalAccountingViolation);
        }
        let mut sum = 0_u64;
        let mut previous: Option<&str> = None;
        for slice in &self.slices {
            if slice.shard_id.trim().is_empty() {
                return Err(AllocationError::InvalidShardId);
            }
            if slice.capacity < slice.minimum_capacity {
                return Err(AllocationError::InternalAccountingViolation);
            }
            if previous.is_some_and(|value| value >= slice.shard_id.as_str()) {
                return Err(AllocationError::InternalAccountingViolation);
            }
            previous = Some(slice.shard_id.as_str());
            sum = sum.checked_add(slice.capacity).ok_or(AllocationError::Overflow)?;
        }
        if sum != self.spendable_capacity {
            return Err(AllocationError::InternalAccountingViolation);
        }
        Ok(())
    }

    pub fn slice_for(&self, shard_id: &str) -> Option<u64> {
        self.slices.iter()
            .find(|slice| slice.shard_id == shard_id)
            .map(|slice| slice.capacity)
    }

    pub fn total_allocated(&self) -> Result<u64, AllocationError> {
        self.slices.iter().try_fold(0_u64, |sum, slice| {
            sum.checked_add(slice.capacity).ok_or(AllocationError::Overflow)
        })
    }
}

/// Allocate a fixed risk budget into immutable pre-partition slices.
///
/// The input samples must already be normalized to the same canonical instrument and
/// time/window semantics. This function never combines values from unlike instruments,
/// never changes rights based on post-partition demand, and never lets the allocations
/// exceed total_capacity. Any contingency reserve remains unallocated and is not locally
/// spendable by an isolated shard.
pub fn allocate_static_escrow(
    shards: &[ShardDemand],
    total_capacity: u64,
    contingency_reserve_ppm: u32,
    policy: AllocationPolicy,
) -> Result<AllocationPlan, AllocationError> {
    if shards.is_empty() {
        return Err(AllocationError::NoShards);
    }
    if contingency_reserve_ppm > PARTS_PER_MILLION {
        return Err(AllocationError::ReserveOutOfRange);
    }

    let instrument_id = shards[0].instrument_id.trim();
    if instrument_id.is_empty() {
        return Err(AllocationError::InvalidInstrumentId);
    }
    let canonical_instrument = shards[0].instrument_id.clone();

    let mut ids = BTreeSet::new();
    for shard in shards {
        if shard.instrument_id.trim().is_empty() {
            return Err(AllocationError::InvalidInstrumentId);
        }
        if shard.instrument_id != canonical_instrument {
            return Err(AllocationError::InstrumentMismatch);
        }
        if shard.shard_id.trim().is_empty() {
            return Err(AllocationError::InvalidShardId);
        }
        if !ids.insert(shard.shard_id.as_str()) {
            return Err(AllocationError::DuplicateShardId);
        }
    }

    let reserve = (u128::from(total_capacity) * u128::from(contingency_reserve_ppm)
        / u128::from(PARTS_PER_MILLION)) as u64;
    let spendable = total_capacity.checked_sub(reserve).ok_or(AllocationError::Overflow)?;

    let minimum_total = shards.iter().try_fold(0_u64, |total, shard| {
        total.checked_add(shard.minimum_capacity).ok_or(AllocationError::Overflow)
    })?;
    if minimum_total > spendable {
        return Err(AllocationError::InsufficientCapacityForMinimums);
    }

    let equal_floor_ppm = match policy {
        AllocationPolicy::DemandWeightedWithEqualFloor { equal_floor_ppm } => {
            if equal_floor_ppm > PARTS_PER_MILLION {
                return Err(AllocationError::FloorOutOfRange);
            }
            equal_floor_ppm
        }
        _ => 0,
    };

    let mut weights = Vec::with_capacity(shards.len());
    for shard in shards {
        let weight = match policy {
            AllocationPolicy::EqualSplit => 1,
            AllocationPolicy::DemandWeighted
            | AllocationPolicy::DemandWeightedWithEqualFloor { .. } => mean_ceiling(&shard.samples)?,
            AllocationPolicy::RobustQuantile { quantile_basis_points, margin_ppm } => {
                if !(1..=BASIS_POINTS).contains(&quantile_basis_points) {
                    return Err(AllocationError::QuantileOutOfRange);
                }
                if margin_ppm > PARTS_PER_MILLION {
                    return Err(AllocationError::MarginOutOfRange);
                }
                let quantile = nearest_rank_quantile(&shard.samples, quantile_basis_points)?;
                let margin = (u128::from(quantile) * u128::from(margin_ppm)
                    / u128::from(PARTS_PER_MILLION)) as u64;
                quantile.checked_add(margin).ok_or(AllocationError::Overflow)?
            }
        };
        weights.push(weight);
    }

    if weights.iter().all(|weight| *weight == 0) {
        // No usable historical demand signal: fail toward deterministic equal shares,
        // rather than starving every shard or inventing a demand ranking.
        weights.fill(1);
    }

    let distributable = spendable.checked_sub(minimum_total).ok_or(AllocationError::Overflow)?;
    let equal_floor_budget = (u128::from(distributable) * u128::from(equal_floor_ppm)
        / u128::from(PARTS_PER_MILLION)) as u64;
    let weighted_budget = distributable.checked_sub(equal_floor_budget).ok_or(AllocationError::Overflow)?;

    // The equal floor is a pre-authorized placement preference, not permission to
    // resize allocations after a partition. Both portions use the same deterministic
    // integer apportionment and their sum remains bounded by the original budget.
    let equal_weights = vec![1_u64; shards.len()];
    let equal_floor_extra = apportion_largest_remainder(equal_floor_budget, &equal_weights, shards)?;
    let weighted_extra = apportion_largest_remainder(weighted_budget, &weights, shards)?;
    let extra: Vec<u64> = equal_floor_extra.iter().zip(&weighted_extra).map(|(equal, weighted)| {
        equal.checked_add(*weighted).ok_or(AllocationError::Overflow)
    }).collect::<Result<_, AllocationError>>()?;

    let mut slices: Vec<AllocationSlice> = shards.iter().enumerate().map(|(index, shard)| {
        Ok(AllocationSlice {
            shard_id: shard.shard_id.clone(),
            minimum_capacity: shard.minimum_capacity,
            capacity: shard.minimum_capacity.checked_add(extra[index])
                .ok_or(AllocationError::Overflow)?,
            allocation_weight: weights[index],
        })
    }).collect::<Result<_, AllocationError>>()?;
    slices.sort_by(|a, b| a.shard_id.cmp(&b.shard_id));

    let plan = AllocationPlan {
        instrument_id: canonical_instrument,
        total_capacity,
        spendable_capacity: spendable,
        contingency_reserve: reserve,
        policy,
        slices,
    };
    plan.validate()?;
    Ok(plan)
}

fn apportion_largest_remainder(
    budget: u64,
    weights: &[u64],
    shards: &[ShardDemand],
) -> Result<Vec<u64>, AllocationError> {
    if shards.is_empty() || weights.len() != shards.len() {
        return Err(AllocationError::InternalAccountingViolation);
    }
    let total_weight = weights.iter().try_fold(0_u64, |total, weight| {
        total.checked_add(*weight).ok_or(AllocationError::Overflow)
    })?;
    if total_weight == 0 {
        return Err(AllocationError::InternalAccountingViolation);
    }

    // Largest-remainder apportionment is integer-only and deterministic. Product of
    // two u64 values fits u128; ties are always broken by lexical canonical shard ID.
    let mut shares = vec![0_u64; shards.len()];
    let mut remainders = Vec::with_capacity(shards.len());
    let mut distributed = 0_u64;
    for (index, weight) in weights.iter().enumerate() {
        let product = u128::from(budget) * u128::from(*weight);
        let share = (product / u128::from(total_weight)) as u64;
        let remainder = product % u128::from(total_weight);
        shares[index] = share;
        distributed = distributed.checked_add(share).ok_or(AllocationError::Overflow)?;
        remainders.push((index, remainder, shards[index].shard_id.as_str()));
    }

    let leftover = budget.checked_sub(distributed).ok_or(AllocationError::Overflow)?;
    remainders.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.2.cmp(b.2)));
    let leftover_usize = usize::try_from(leftover).map_err(|_| AllocationError::Overflow)?;
    if leftover_usize > remainders.len() {
        return Err(AllocationError::InternalAccountingViolation);
    }
    for (index, _, _) in remainders.iter().take(leftover_usize) {
        shares[*index] = shares[*index].checked_add(1).ok_or(AllocationError::Overflow)?;
    }
    Ok(shares)
}

fn mean_ceiling(samples: &[u64]) -> Result<u64, AllocationError> {
    if samples.is_empty() {
        // Missing history is unknown demand, not a measured zero. Data-dependent
        // policies must not silently starve a new shard because it lacks telemetry.
        return Err(AllocationError::MissingDemandSamples);
    }
    let total = samples.iter().try_fold(0_u64, |sum, sample| {
        sum.checked_add(*sample).ok_or(AllocationError::Overflow)
    })?;
    let count = u64::try_from(samples.len()).map_err(|_| AllocationError::Overflow)?;
    let quotient = total / count;
    if total % count == 0 {
        Ok(quotient)
    } else {
        quotient.checked_add(1).ok_or(AllocationError::Overflow)
    }
}

fn nearest_rank_quantile(samples: &[u64], quantile_basis_points: u16) -> Result<u64, AllocationError> {
    if samples.is_empty() {
        return Err(AllocationError::MissingDemandSamples);
    }
    let mut ordered = samples.to_vec();
    ordered.sort_unstable();
    let numerator = u128::from(quantile_basis_points)
        * u128::try_from(ordered.len()).map_err(|_| AllocationError::Overflow)?;
    let rank = numerator.div_ceil(u128::from(BASIS_POINTS)).max(1);
    let index = usize::try_from(rank - 1).map_err(|_| AllocationError::Overflow)?;
    ordered.get(index).copied().ok_or(AllocationError::InternalAccountingViolation)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn shard(id: &str, samples: &[u64], minimum_capacity: u64) -> ShardDemand {
        ShardDemand {
            instrument_id: "instrument:unit".to_owned(),
            shard_id: id.to_owned(),
            samples: samples.to_vec(),
            minimum_capacity,
        }
    }

    #[test]
    fn plans_reject_mixed_or_missing_instrument_identity() {
        let mut mixed = vec![shard("a", &[10], 0), shard("b", &[20], 0)];
        mixed[1].instrument_id = "instrument:other".into();
        assert_eq!(
            allocate_static_escrow(&mixed, 100, 0, AllocationPolicy::EqualSplit),
            Err(AllocationError::InstrumentMismatch),
        );

        let mut missing = vec![shard("a", &[10], 0)];
        missing[0].instrument_id.clear();
        assert_eq!(
            allocate_static_escrow(&missing, 100, 0, AllocationPolicy::EqualSplit),
            Err(AllocationError::InvalidInstrumentId),
        );
    }

    #[test]
    fn equal_split_is_deterministic_and_conserves_integer_capacity() {
        let input = vec![
            shard("c", &[], 0),
            shard("a", &[], 0),
            shard("b", &[], 0),
        ];
        let plan = allocate_static_escrow(&input, 101, 0, AllocationPolicy::EqualSplit).unwrap();
        assert_eq!(plan.slice_for("a"), Some(34));
        assert_eq!(plan.slice_for("b"), Some(34));
        assert_eq!(plan.slice_for("c"), Some(33));
        assert_eq!(plan.total_allocated().unwrap(), 101);
        assert_eq!(plan.contingency_reserve, 0);
        plan.validate().unwrap();
    }

    #[test]
    fn demand_weighted_uses_ceiling_mean_and_largest_remainder() {
        let input = vec![
            shard("a", &[10], 0),
            shard("b", &[20], 0),
            shard("c", &[30], 0),
        ];
        let plan = allocate_static_escrow(&input, 60, 0, AllocationPolicy::DemandWeighted).unwrap();
        assert_eq!(plan.slice_for("a"), Some(10));
        assert_eq!(plan.slice_for("b"), Some(20));
        assert_eq!(plan.slice_for("c"), Some(30));
        plan.validate().unwrap();
    }

    #[test]
    fn demand_weighted_equal_floor_trades_peak_utilization_for_shift_resilience() {
        let input = vec![shard("a", &[10], 0), shard("b", &[90], 0)];
        let weighted = allocate_static_escrow(&input, 100, 0, AllocationPolicy::DemandWeighted).unwrap();
        let hybrid = allocate_static_escrow(
            &input,
            100,
            0,
            AllocationPolicy::DemandWeightedWithEqualFloor { equal_floor_ppm: 500_000 },
        ).unwrap();
        assert_eq!(weighted.slice_for("a"), Some(10));
        assert_eq!(weighted.slice_for("b"), Some(90));
        assert_eq!(hybrid.slice_for("a"), Some(30));
        assert_eq!(hybrid.slice_for("b"), Some(70));
        assert_eq!(hybrid.total_allocated().unwrap(), 100);
        hybrid.validate().unwrap();
    }

    #[test]
    fn full_equal_floor_matches_equal_split_after_minimums() {
        let input = vec![shard("a", &[10], 5), shard("b", &[90], 5)];
        let equal = allocate_static_escrow(&input, 100, 0, AllocationPolicy::EqualSplit).unwrap();
        let hybrid = allocate_static_escrow(
            &input,
            100,
            0,
            AllocationPolicy::DemandWeightedWithEqualFloor { equal_floor_ppm: PARTS_PER_MILLION },
        ).unwrap();
        assert_eq!(equal.slice_for("a"), hybrid.slice_for("a"));
        assert_eq!(equal.slice_for("b"), hybrid.slice_for("b"));
    }

    #[test]
    fn robust_quantile_uses_upper_sample_and_explicit_margin() {
        let input = vec![
            shard("a", &[10, 20, 100], 0),
            shard("b", &[30, 30, 30], 0),
        ];
        let policy = AllocationPolicy::RobustQuantile {
            quantile_basis_points: 5_000,
            margin_ppm: 500_000,
        };
        let plan = allocate_static_escrow(&input, 150, 0, policy).unwrap();
        assert_eq!(plan.slice_for("a"), Some(60));
        assert_eq!(plan.slice_for("b"), Some(90));
        assert_eq!(plan.slices.iter().find(|s| s.shard_id == "a").unwrap().allocation_weight, 30);
        plan.validate().unwrap();
    }

    #[test]
    fn contingency_reserve_is_not_allocated_to_any_shard() {
        let input = vec![shard("a", &[1], 0), shard("b", &[2], 0)];
        let plan = allocate_static_escrow(&input, 1_000, 100_000, AllocationPolicy::DemandWeighted).unwrap();
        assert_eq!(plan.contingency_reserve, 100);
        assert_eq!(plan.spendable_capacity, 900);
        assert_eq!(plan.total_allocated().unwrap(), 900);
        plan.validate().unwrap();
    }

    #[test]
    fn policy_reordering_does_not_change_per_shard_rights() {
        let input = vec![
            shard("z", &[3, 4, 5], 1),
            shard("a", &[8, 2, 6], 2),
            shard("m", &[5, 4, 7], 0),
        ];
        let mut reversed = input.clone();
        reversed.reverse();
        for policy in [
            AllocationPolicy::EqualSplit,
            AllocationPolicy::DemandWeighted,
            AllocationPolicy::RobustQuantile {
                quantile_basis_points: 6_000,
                margin_ppm: 250_000,
            },
            AllocationPolicy::DemandWeightedWithEqualFloor { equal_floor_ppm: 250_000 },
            AllocationPolicy::DemandWeightedWithEqualFloor { equal_floor_ppm: 500_000 },
            AllocationPolicy::DemandWeightedWithEqualFloor { equal_floor_ppm: 750_000 },
        ] {
            let a = allocate_static_escrow(&input, 1_007, 10_000, policy).unwrap();
            let b = allocate_static_escrow(&reversed, 1_007, 10_000, policy).unwrap();
            assert_eq!(a, b);
            a.validate().unwrap();
        }
    }

    #[test]
    fn explicit_minimum_slices_are_preserved() {
        let input = vec![shard("a", &[1], 7), shard("b", &[9], 11)];
        let plan = allocate_static_escrow(&input, 100, 0, AllocationPolicy::DemandWeighted).unwrap();
        assert!(plan.slice_for("a").unwrap() >= 7);
        assert!(plan.slice_for("b").unwrap() >= 11);
        assert_eq!(plan.total_allocated().unwrap(), 100);
    }

    #[test]
    fn all_zero_demand_falls_back_to_equal_without_creating_capacity() {
        let input = vec![shard("a", &[0, 0], 0), shard("b", &[0], 0), shard("c", &[0, 0], 0)];
        let plan = allocate_static_escrow(&input, 12, 0, AllocationPolicy::DemandWeighted).unwrap();
        assert_eq!(plan.slice_for("a"), Some(4));
        assert_eq!(plan.slice_for("b"), Some(4));
        assert_eq!(plan.slice_for("c"), Some(4));
    }

    #[test]
    fn demand_weighted_policy_does_not_treat_missing_history_as_zero_demand() {
        assert_eq!(
            allocate_static_escrow(
                &[shard("known", &[10, 20], 0), shard("new", &[], 0)],
                100,
                0,
                AllocationPolicy::DemandWeighted,
            ),
            Err(AllocationError::MissingDemandSamples),
        );
    }

    #[test]
    fn duplicate_or_empty_shard_ids_reject() {
        assert_eq!(
            allocate_static_escrow(&[shard("a", &[], 0), shard("a", &[], 0)], 5, 0, AllocationPolicy::EqualSplit),
            Err(AllocationError::DuplicateShardId),
        );
        assert_eq!(
            allocate_static_escrow(&[shard("  ", &[], 0)], 5, 0, AllocationPolicy::EqualSplit),
            Err(AllocationError::InvalidShardId),
        );
    }

    #[test]
    fn robust_policy_fails_closed_without_samples_or_with_bad_parameters() {
        assert_eq!(
            allocate_static_escrow(
                &[shard("a", &[], 0)],
                5,
                0,
                AllocationPolicy::RobustQuantile { quantile_basis_points: 5_000, margin_ppm: 0 },
            ),
            Err(AllocationError::MissingDemandSamples),
        );
        assert_eq!(
            allocate_static_escrow(
                &[shard("a", &[1], 0)],
                5,
                0,
                AllocationPolicy::RobustQuantile { quantile_basis_points: 0, margin_ppm: 0 },
            ),
            Err(AllocationError::QuantileOutOfRange),
        );
        assert_eq!(
            allocate_static_escrow(&[shard("a", &[], 0)], 5, PARTS_PER_MILLION + 1, AllocationPolicy::EqualSplit),
            Err(AllocationError::ReserveOutOfRange),
        );
    }

    #[test]
    fn insufficient_capacity_for_minimums_rejects_before_allocation() {
        assert_eq!(
            allocate_static_escrow(
                &[shard("a", &[], 4), shard("b", &[], 4)],
                10,
                300_000,
                AllocationPolicy::EqualSplit,
            ),
            Err(AllocationError::InsufficientCapacityForMinimums),
        );
    }

    #[test]
    fn allocation_plan_slices_fit_the_reservation_book_caps() {
        use crate::reservation::{ReservationApply, ReservationBook, ReservationKey};

        let input = vec![
            shard("shard-c", &[30, 32, 28], 0),
            shard("shard-a", &[10, 11, 9], 0),
            shard("shard-b", &[20, 21, 19], 0),
        ];
        let plan = allocate_static_escrow(
            &input,
            600,
            60_000,
            AllocationPolicy::DemandWeighted,
        ).unwrap();

        let mut book = ReservationBook::default();
        for community in ["community:a", "community:b"] {
            book.set_community_cap(community, "instrument:unit", 600).unwrap();
        }
        book.set_agreement_cap("agreement:a-b", "instrument:unit", 600).unwrap();

        for slice in &plan.slices {
            let result = book.reserve(
                ReservationKey {
                    agreement_id: "agreement:a-b".into(),
                    instrument: "instrument:unit".into(),
                    shard_id: slice.shard_id.clone(),
                    epoch: 1,
                },
                "community:a",
                "community:b",
                slice.capacity,
            ).unwrap();
            assert_eq!(result, ReservationApply::Reserved);
        }
        assert_eq!(book.allocated_to_community("community:a", "instrument:unit").unwrap(),
            plan.spendable_capacity);
        assert_eq!(book.allocated_to_community("community:b", "instrument:unit").unwrap(),
            plan.spendable_capacity);
        assert_eq!(book.allocated_to_agreement("agreement:a-b", "instrument:unit").unwrap(),
            plan.spendable_capacity);
        assert_eq!(plan.total_allocated().unwrap().checked_add(plan.contingency_reserve).unwrap(), plan.total_capacity);
    }

    #[test]
    fn ten_thousand_generated_allocation_plans_conserve_capacity_and_ignore_input_order() {
        struct Rng(u64);
        impl Rng {
            fn next(&mut self) -> u64 {
                self.0 = self.0.wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                self.0
            }
        }

        let mut rng = Rng(0x4e43462d414c4c31);
        for case_index in 0..10_000 {
            let count = 2 + (rng.next() % 15) as usize;
            let capacity = count as u64 + rng.next() % 100_000;
            let reserve_ppm = (rng.next() % 500_001) as u32;
            let policy = match case_index % 6 {
                0 => AllocationPolicy::EqualSplit,
                1 => AllocationPolicy::DemandWeighted,
                2 => AllocationPolicy::RobustQuantile {
                    quantile_basis_points: (rng.next() % u64::from(BASIS_POINTS) + 1) as u16,
                    margin_ppm: (rng.next() % u64::from(PARTS_PER_MILLION) + 1) as u32,
                },
                3 => AllocationPolicy::DemandWeightedWithEqualFloor { equal_floor_ppm: 250_000 },
                4 => AllocationPolicy::DemandWeightedWithEqualFloor { equal_floor_ppm: 500_000 },
                _ => AllocationPolicy::DemandWeightedWithEqualFloor { equal_floor_ppm: 750_000 },
            };
            let input: Vec<ShardDemand> = (0..count).map(|index| {
                let sample_count = 1 + (rng.next() % 8) as usize;
                let samples = (0..sample_count).map(|_| rng.next() % 10_001).collect();
                ShardDemand {
                    instrument_id: "instrument:unit".into(),
                    shard_id: format!("shard-{index:03}"),
                    samples,
                    minimum_capacity: 0,
                }
            }).collect();

            let plan = allocate_static_escrow(&input, capacity, reserve_ppm, policy)
                .unwrap_or_else(|error| panic!("case {case_index} failed: {error:?}"));
            plan.validate().unwrap_or_else(|error| panic!("case {case_index} invalid: {error:?}"));
            assert_eq!(
                plan.total_allocated().unwrap().checked_add(plan.contingency_reserve),
                Some(plan.total_capacity),
                "capacity conservation failed for generated case {case_index}",
            );

            let mut reversed = input.clone();
            reversed.reverse();
            let permuted_plan = allocate_static_escrow(&reversed, capacity, reserve_ppm, policy)
                .unwrap_or_else(|error| panic!("permuted case {case_index} failed: {error:?}"));
            assert_eq!(
                plan,
                permuted_plan,
                "input order changed rights in generated case {case_index}",
            );
        }
    }

    #[test]
    fn demand_history_overflow_rejects_instead_of_wrapping() {
        assert_eq!(
            allocate_static_escrow(
                &[shard("a", &[u64::MAX, 1], 0)],
                100,
                0,
                AllocationPolicy::DemandWeighted,
            ),
            Err(AllocationError::Overflow),
        );
    }
}
