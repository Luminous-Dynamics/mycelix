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
    InvalidShardId,
    DuplicateShardId,
    ReserveOutOfRange,
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
        if self.contingency_reserve > self.total_capacity
            || self.spendable_capacity.checked_add(self.contingency_reserve)
                != Some(self.total_capacity)
        {
            return Err(AllocationError::InternalAccountingViolation);
        }
        let mut sum = 0_u64;
        let mut previous: Option<&str> = None;
        for slice in &self.slices {
            if slice.capacity < slice.minimum_capacity {
                return Err(AllocationError::InternalAccountingViolation);
            }
            if previous.is_some_and(|value| value >= slice.shard_id.as_str()) {
                return Err(AllocationError::InternalAccountingViolation);
            }
            previous = Some(&slice.shard_id);
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

    pub fn total_allocated(&self) -> u64 {
        self.slices.iter().map(|slice| slice.capacity).sum()
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

    let mut ids = BTreeSet::new();
    for shard in shards {
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

    let mut weights = Vec::with_capacity(shards.len());
    for shard in shards {
        let weight = match policy {
            AllocationPolicy::EqualSplit => 1,
            AllocationPolicy::DemandWeighted => mean_ceiling(&shard.samples)?,
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

    let total_weight = weights.iter().try_fold(0_u64, |total, weight| {
        total.checked_add(*weight).ok_or(AllocationError::Overflow)
    })?;
    let distributable = spendable.checked_sub(minimum_total).ok_or(AllocationError::Overflow)?;

    // Largest-remainder apportionment is integer-only and deterministic. Product of
    // two u64 values fits u128; tie breaks are always lexical shard IDs.
    let mut extra = vec![0_u64; shards.len()];
    let mut remainders = Vec::with_capacity(shards.len());
    let mut distributed = 0_u64;
    for (index, weight) in weights.iter().enumerate() {
        let product = u128::from(distributable) * u128::from(*weight);
        let share = (product / u128::from(total_weight)) as u64;
        let remainder = product % u128::from(total_weight);
        extra[index] = share;
        distributed = distributed.checked_add(share).ok_or(AllocationError::Overflow)?;
        remainders.push((index, remainder, shards[index].shard_id.as_str()));
    }

    let leftover = distributable.checked_sub(distributed).ok_or(AllocationError::Overflow)?;
    remainders.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.2.cmp(b.2)));
    let leftover_usize = usize::try_from(leftover).map_err(|_| AllocationError::Overflow)?;
    if leftover_usize > remainders.len() {
        return Err(AllocationError::InternalAccountingViolation);
    }
    for (index, _, _) in remainders.iter().take(leftover_usize) {
        extra[*index] = extra[*index].checked_add(1).ok_or(AllocationError::Overflow)?;
    }

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
        total_capacity,
        spendable_capacity: spendable,
        contingency_reserve: reserve,
        policy,
        slices,
    };
    plan.validate()?;
    Ok(plan)
}

fn mean_ceiling(samples: &[u64]) -> Result<u64, AllocationError> {
    if samples.is_empty() {
        return Ok(0);
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
            shard_id: id.to_owned(),
            samples: samples.to_vec(),
            minimum_capacity,
        }
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
        assert_eq!(plan.total_allocated(), 101);
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
        assert_eq!(plan.total_allocated(), 900);
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
        assert_eq!(plan.total_allocated(), 100);
    }

    #[test]
    fn all_zero_demand_falls_back_to_equal_without_creating_capacity() {
        let input = vec![shard("a", &[0, 0], 0), shard("b", &[0], 0), shard("c", &[], 0)];
        let plan = allocate_static_escrow(&input, 12, 0, AllocationPolicy::DemandWeighted).unwrap();
        assert_eq!(plan.slice_for("a"), Some(4));
        assert_eq!(plan.slice_for("b"), Some(4));
        assert_eq!(plan.slice_for("c"), Some(4));
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
