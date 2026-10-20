#![forbid(unsafe_code)]

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! FIN-ECO-005E1 deterministic mutual-credit netting and fee-savings reference kernel.
//!
//! This crate is a simulator, not a ledger, clearinghouse, credit authority, legal-netting
//! opinion, or payment executor. It consumes only obligations already classified under an
//! externally qualified netting-set policy. No Holochain action or external payment occurs.
//!
//! Positive net positions mean a participant owes the clearing set; negative positions mean
//! a participant is a net creditor. All amounts are integer minor units. Variable fees use
//! exact basis points and ceiling rounding.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const MAX_PROFILE_MEMBERS: usize = 10_000;
pub const MAX_INPUT_OBLIGATIONS: usize = 100_000;
pub const MAX_IDENTIFIER_LEN: usize = 256;
pub const BASIS_POINTS_DENOMINATOR: u128 = 10_000;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CostModel {
    /// Flat fee assessed once per external settlement leg, in the modeled asset's minor unit.
    pub fixed_fee_per_leg_minor: u64,
    /// Variable fee in basis points, calculated and rounded independently for each leg.
    pub variable_fee_bps: u32,
    /// Minimum variable component per nonzero settlement leg.
    pub minimum_variable_fee_per_leg_minor: u64,
    /// Optional cap on the variable component per leg.
    pub maximum_variable_fee_per_leg_minor: Option<u64>,
    /// Fixed fee per batch of legs; a partial final batch incurs the same fee.
    pub batch_fee_minor: u64,
    /// Maximum number of settlement legs in one batch (must be positive).
    pub batch_capacity_legs: usize,
    /// Additional fee for running the netting process, not charged to direct settlement.
    pub netting_overhead_minor: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NettingProfile {
    pub network_id: String,
    pub asset_profile_id: String,
    pub netting_set_id: String,
    /// Versioned policy identity/digest. This crate checks shape only; an upstream authority
    /// must authenticate the policy and its netting permission.
    pub policy_digest: String,
    pub member_ids: BTreeSet<String>,
    pub max_input_obligations: usize,
    pub max_single_obligation_minor: u64,
    pub max_abs_member_net_position_minor: u64,
    pub cost_model: CostModel,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ObligationStatus {
    Accepted,
    Proposed,
    Disputed,
    Cancelled,
    AlreadySettled,
    Unknown,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Obligation {
    /// Canonical economic obligation identity, not a physical Holochain action hash.
    pub canonical_id: String,
    pub network_id: String,
    pub asset_profile_id: String,
    pub netting_set_id: String,
    pub payer_id: String,
    pub payee_id: String,
    pub amount_minor: u64,
    pub status: ObligationStatus,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ExcludedReason {
    Proposed,
    Disputed,
    Cancelled,
    AlreadySettled,
    Unknown,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExcludedObligation {
    pub canonical_id: String,
    pub amount_minor: u64,
    pub reason: ExcludedReason,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NetPosition {
    pub member_id: String,
    /// Positive = net payable; negative = net receivable; zero = exactly offset.
    pub net_position_minor: i128,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SettlementLeg {
    pub payer_id: String,
    pub payee_id: String,
    pub amount_minor: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CostEstimate {
    pub modeled_volume_minor: u128,
    pub modeled_leg_count: usize,
    pub transaction_fixed_cost_minor: u128,
    pub batch_cost_minor: u128,
    pub netting_overhead_minor: u128,
    /// Sum of transaction-fixed, batch, and netting-overhead components.
    pub fixed_cost_minor: u128,
    pub variable_cost_minor: u128,
    pub total_cost_minor: u128,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NettingReport {
    pub network_id: String,
    pub asset_profile_id: String,
    pub netting_set_id: String,
    pub policy_digest: String,
    pub included_obligation_ids: Vec<String>,
    pub excluded_obligations: Vec<ExcludedObligation>,
    /// Number of additional identical physical observations removed by canonical ID.
    pub duplicate_observation_count: u64,
    /// True only when every supplied canonical obligation is eligible; does not prove the
    /// upstream obligation census is complete.
    pub all_supplied_obligations_eligible: bool,
    pub gross_obligation_count: usize,
    pub gross_volume_minor: u128,
    pub net_positions: Vec<NetPosition>,
    pub net_settlement_legs: Vec<SettlementLeg>,
    pub net_settlement_volume_minor: u128,
    pub gross_cost_estimate: CostEstimate,
    pub net_cost_estimate: CostEstimate,
    /// May be negative when the modeled net process costs more than direct settlement.
    pub modeled_savings_minor: i128,
    /// Signed basis points of gross modeled cost; None when gross modeled cost is zero.
    pub modeled_savings_bps: Option<i128>,
    /// This pure kernel can never set a payment or clearing action to executed.
    pub settlement_executed: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NettingError {
    EmptyInput,
    InvalidProfile(&'static str),
    InvalidIdentifier(&'static str),
    TooManyObligations { actual: usize, maximum: usize },
    WrongScope { obligation_id: String, field: &'static str },
    UnknownMember { obligation_id: String, member_id: String },
    SelfObligation { obligation_id: String },
    ZeroAmount { obligation_id: String },
    SingleAmountLimit { obligation_id: String, amount_minor: u64, limit_minor: u64 },
    ConflictingDuplicate { canonical_id: String },
    ExposureLimitExceeded { member_id: String, absolute_position_minor: u128, limit_minor: u64 },
    ArithmeticOverflow(&'static str),
    UnbalancedPositions { sum_minor: i128 },
    InvariantViolation(&'static str),
}

impl fmt::Display for NettingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for NettingError {}

pub fn simulate_netting(
    profile: &NettingProfile,
    obligations: &[Obligation],
) -> Result<NettingReport, NettingError> {
    validate_profile(profile)?;
    if obligations.is_empty() {
        return Err(NettingError::EmptyInput);
    }
    if obligations.len() > profile.max_input_obligations {
        return Err(NettingError::TooManyObligations {
            actual: obligations.len(),
            maximum: profile.max_input_obligations,
        });
    }

    // Canonical BTreeMap gives deterministic duplicate handling and ordering.
    let mut canonical: BTreeMap<String, Obligation> = BTreeMap::new();
    let mut duplicate_observation_count = 0_u64;

    for obligation in obligations {
        validate_obligation_shape(profile, obligation)?;
        match canonical.get(&obligation.canonical_id) {
            Some(previous) if previous == obligation => {
                duplicate_observation_count = duplicate_observation_count
                    .checked_add(1)
                    .ok_or(NettingError::ArithmeticOverflow("duplicate observation count"))?;
            }
            Some(_) => {
                return Err(NettingError::ConflictingDuplicate {
                    canonical_id: obligation.canonical_id.clone(),
                });
            }
            None => {
                canonical.insert(obligation.canonical_id.clone(), obligation.clone());
            }
        }
    }

    let mut positions: BTreeMap<String, i128> = profile
        .member_ids
        .iter()
        .cloned()
        .map(|member| (member, 0_i128))
        .collect();
    let mut included_obligation_ids = Vec::new();
    let mut excluded_obligations = Vec::new();
    let mut gross_leg_amounts = Vec::new();
    let mut gross_volume_minor = 0_u128;

    for obligation in canonical.values() {
        match &obligation.status {
            ObligationStatus::Accepted => {
                let amount = i128::from(obligation.amount_minor);
                let payer_position = positions
                    .get_mut(&obligation.payer_id)
                    .ok_or_else(|| NettingError::UnknownMember {
                        obligation_id: obligation.canonical_id.clone(),
                        member_id: obligation.payer_id.clone(),
                    })?;
                *payer_position = payer_position
                    .checked_add(amount)
                    .ok_or(NettingError::ArithmeticOverflow("payer net position"))?;

                let payee_position = positions
                    .get_mut(&obligation.payee_id)
                    .ok_or_else(|| NettingError::UnknownMember {
                        obligation_id: obligation.canonical_id.clone(),
                        member_id: obligation.payee_id.clone(),
                    })?;
                *payee_position = payee_position
                    .checked_sub(amount)
                    .ok_or(NettingError::ArithmeticOverflow("payee net position"))?;

                gross_volume_minor = gross_volume_minor
                    .checked_add(u128::from(obligation.amount_minor))
                    .ok_or(NettingError::ArithmeticOverflow("gross volume"))?;
                included_obligation_ids.push(obligation.canonical_id.clone());
                gross_leg_amounts.push(obligation.amount_minor);
            }
            status => excluded_obligations.push(ExcludedObligation {
                canonical_id: obligation.canonical_id.clone(),
                amount_minor: obligation.amount_minor,
                reason: excluded_reason(status)?,
            }),
        }
    }

    let mut position_sum = 0_i128;
    let mut debtor_positions: Vec<(String, u128)> = Vec::new();
    let mut creditor_positions: Vec<(String, u128)> = Vec::new();
    let mut net_settlement_volume_minor = 0_u128;
    let mut net_positions = Vec::with_capacity(positions.len());

    for (member_id, position) in &positions {
        position_sum = position_sum
            .checked_add(*position)
            .ok_or(NettingError::ArithmeticOverflow("sum of net positions"))?;
        let absolute = position
            .checked_abs()
            .ok_or(NettingError::ArithmeticOverflow("absolute net position"))? as u128;
        if absolute > u128::from(profile.max_abs_member_net_position_minor) {
            return Err(NettingError::ExposureLimitExceeded {
                member_id: member_id.clone(),
                absolute_position_minor: absolute,
                limit_minor: profile.max_abs_member_net_position_minor,
            });
        }

        if *position > 0 {
            let amount = u128::try_from(*position)
                .map_err(|_| NettingError::ArithmeticOverflow("debtor position conversion"))?;
            net_settlement_volume_minor = net_settlement_volume_minor
                .checked_add(amount)
                .ok_or(NettingError::ArithmeticOverflow("net settlement volume"))?;
            debtor_positions.push((member_id.clone(), amount));
        } else if *position < 0 {
            let absolute_credit = position
                .checked_abs()
                .ok_or(NettingError::ArithmeticOverflow("creditor position absolute value"))?;
            let amount = u128::try_from(absolute_credit)
                .map_err(|_| NettingError::ArithmeticOverflow("creditor position conversion"))?;
            creditor_positions.push((member_id.clone(), amount));
        }

        net_positions.push(NetPosition {
            member_id: member_id.clone(),
            net_position_minor: *position,
        });
    }

    if position_sum != 0 {
        return Err(NettingError::UnbalancedPositions { sum_minor: position_sum });
    }

    let net_settlement_legs = match_debtors_to_creditors(debtor_positions, creditor_positions)?;
    verify_settlement_conservation(&positions, &net_settlement_legs, net_settlement_volume_minor)?;

    let net_leg_amounts: Vec<u64> = net_settlement_legs.iter().map(|leg| leg.amount_minor).collect();
    let gross_cost_estimate = estimate_cost(&gross_leg_amounts, &profile.cost_model, false)?;
    let net_cost_estimate = estimate_cost(&net_leg_amounts, &profile.cost_model, true)?;
    if gross_cost_estimate.modeled_volume_minor != gross_volume_minor
        || net_cost_estimate.modeled_volume_minor != net_settlement_volume_minor
    {
        return Err(NettingError::InvariantViolation(
            "cost estimate volume differs from the qualified settlement plan",
        ));
    }
    let gross_cost = i128::try_from(gross_cost_estimate.total_cost_minor)
        .map_err(|_| NettingError::ArithmeticOverflow("gross cost signed conversion"))?;
    let net_cost = i128::try_from(net_cost_estimate.total_cost_minor)
        .map_err(|_| NettingError::ArithmeticOverflow("net cost signed conversion"))?;
    let modeled_savings_minor = gross_cost
        .checked_sub(net_cost)
        .ok_or(NettingError::ArithmeticOverflow("modeled cost savings"))?;
    let modeled_savings_bps = if gross_cost == 0 {
        None
    } else {
        Some(
            modeled_savings_minor
                .checked_mul(10_000_i128)
                .ok_or(NettingError::ArithmeticOverflow("modeled savings basis points"))?
                / gross_cost,
        )
    };

    Ok(NettingReport {
        network_id: profile.network_id.clone(),
        asset_profile_id: profile.asset_profile_id.clone(),
        netting_set_id: profile.netting_set_id.clone(),
        policy_digest: profile.policy_digest.clone(),
        all_supplied_obligations_eligible: excluded_obligations.is_empty(),
        gross_obligation_count: included_obligation_ids.len(),
        gross_volume_minor,
        included_obligation_ids,
        excluded_obligations,
        duplicate_observation_count,
        net_positions,
        net_settlement_legs,
        net_settlement_volume_minor,
        gross_cost_estimate,
        net_cost_estimate,
        modeled_savings_minor,
        modeled_savings_bps,
        settlement_executed: false,
    })
}

fn validate_profile(profile: &NettingProfile) -> Result<(), NettingError> {
    validate_identifier(&profile.network_id, "network ID")?;
    validate_identifier(&profile.asset_profile_id, "asset profile ID")?;
    validate_identifier(&profile.netting_set_id, "netting set ID")?;
    validate_identifier(&profile.policy_digest, "policy digest")?;

    if profile.member_ids.len() < 2 {
        return Err(NettingError::InvalidProfile("at least two members are required"));
    }
    if profile.member_ids.len() > MAX_PROFILE_MEMBERS {
        return Err(NettingError::InvalidProfile("member count exceeds implementation bound"));
    }
    for member in &profile.member_ids {
        validate_identifier(member, "member ID")?;
    }
    if profile.max_input_obligations == 0 || profile.max_input_obligations > MAX_INPUT_OBLIGATIONS {
        return Err(NettingError::InvalidProfile("obligation bound is outside implementation limits"));
    }
    if profile.max_single_obligation_minor == 0 {
        return Err(NettingError::InvalidProfile("single-obligation limit must be positive"));
    }
    if profile.max_abs_member_net_position_minor == 0 {
        return Err(NettingError::InvalidProfile("member exposure limit must be positive"));
    }
    if profile.cost_model.variable_fee_bps > 10_000 {
        return Err(NettingError::InvalidProfile("variable fee exceeds 100 percent"));
    }
    if profile.cost_model.batch_capacity_legs == 0 {
        return Err(NettingError::InvalidProfile("batch capacity must be positive"));
    }
    if profile.cost_model.maximum_variable_fee_per_leg_minor
        .is_some_and(|maximum| maximum < profile.cost_model.minimum_variable_fee_per_leg_minor)
    {
        return Err(NettingError::InvalidProfile(
            "maximum variable fee per leg cannot be below its minimum",
        ));
    }
    Ok(())
}

fn validate_obligation_shape(
    profile: &NettingProfile,
    obligation: &Obligation,
) -> Result<(), NettingError> {
    validate_identifier(&obligation.canonical_id, "obligation ID")?;
    validate_identifier(&obligation.payer_id, "payer ID")?;
    validate_identifier(&obligation.payee_id, "payee ID")?;
    if obligation.payer_id == obligation.payee_id {
        return Err(NettingError::SelfObligation {
            obligation_id: obligation.canonical_id.clone(),
        });
    }
    if obligation.amount_minor == 0 {
        return Err(NettingError::ZeroAmount {
            obligation_id: obligation.canonical_id.clone(),
        });
    }
    if obligation.amount_minor > profile.max_single_obligation_minor {
        return Err(NettingError::SingleAmountLimit {
            obligation_id: obligation.canonical_id.clone(),
            amount_minor: obligation.amount_minor,
            limit_minor: profile.max_single_obligation_minor,
        });
    }
    if obligation.network_id != profile.network_id {
        return Err(NettingError::WrongScope {
            obligation_id: obligation.canonical_id.clone(),
            field: "network ID",
        });
    }
    if obligation.asset_profile_id != profile.asset_profile_id {
        return Err(NettingError::WrongScope {
            obligation_id: obligation.canonical_id.clone(),
            field: "asset profile ID",
        });
    }
    if obligation.netting_set_id != profile.netting_set_id {
        return Err(NettingError::WrongScope {
            obligation_id: obligation.canonical_id.clone(),
            field: "netting set ID",
        });
    }
    if !profile.member_ids.contains(&obligation.payer_id) {
        return Err(NettingError::UnknownMember {
            obligation_id: obligation.canonical_id.clone(),
            member_id: obligation.payer_id.clone(),
        });
    }
    if !profile.member_ids.contains(&obligation.payee_id) {
        return Err(NettingError::UnknownMember {
            obligation_id: obligation.canonical_id.clone(),
            member_id: obligation.payee_id.clone(),
        });
    }
    Ok(())
}

fn validate_identifier(value: &str, label: &'static str) -> Result<(), NettingError> {
    if value.trim().is_empty() || value.len() > MAX_IDENTIFIER_LEN {
        return Err(NettingError::InvalidIdentifier(label));
    }
    Ok(())
}

fn excluded_reason(status: &ObligationStatus) -> Result<ExcludedReason, NettingError> {
    match status {
        ObligationStatus::Accepted => Err(NettingError::InvariantViolation(
            "accepted obligation must not be excluded",
        )),
        ObligationStatus::Proposed => Ok(ExcludedReason::Proposed),
        ObligationStatus::Disputed => Ok(ExcludedReason::Disputed),
        ObligationStatus::Cancelled => Ok(ExcludedReason::Cancelled),
        ObligationStatus::AlreadySettled => Ok(ExcludedReason::AlreadySettled),
        ObligationStatus::Unknown => Ok(ExcludedReason::Unknown),
    }
}

fn match_debtors_to_creditors(
    mut debtors: Vec<(String, u128)>,
    mut creditors: Vec<(String, u128)>,
) -> Result<Vec<SettlementLeg>, NettingError> {
    let mut legs = Vec::new();
    let mut debtor_index = 0_usize;
    let mut creditor_index = 0_usize;

    while debtor_index < debtors.len() && creditor_index < creditors.len() {
        let amount = debtors[debtor_index].1.min(creditors[creditor_index].1);
        if amount == 0 {
            return Err(NettingError::InvariantViolation("zero-sized settlement leg"));
        }
        let amount_minor = u64::try_from(amount)
            .map_err(|_| NettingError::ArithmeticOverflow("settlement leg amount conversion"))?;
        legs.push(SettlementLeg {
            payer_id: debtors[debtor_index].0.clone(),
            payee_id: creditors[creditor_index].0.clone(),
            amount_minor,
        });
        debtors[debtor_index].1 -= amount;
        creditors[creditor_index].1 -= amount;

        if debtors[debtor_index].1 == 0 {
            debtor_index += 1;
        }
        if creditors[creditor_index].1 == 0 {
            creditor_index += 1;
        }
    }

    if debtor_index != debtors.len() || creditor_index != creditors.len() {
        return Err(NettingError::InvariantViolation(
            "debtor and creditor totals did not match",
        ));
    }
    Ok(legs)
}

fn verify_settlement_conservation(
    positions: &BTreeMap<String, i128>,
    legs: &[SettlementLeg],
    expected_volume_minor: u128,
) -> Result<(), NettingError> {
    let mut reconstructed: BTreeMap<String, i128> = positions
        .keys()
        .cloned()
        .map(|member| (member, 0_i128))
        .collect();
    let mut total_volume = 0_u128;

    for leg in legs {
        if leg.amount_minor == 0 || leg.payer_id == leg.payee_id {
            return Err(NettingError::InvariantViolation(
                "settlement leg must have distinct parties and a positive amount",
            ));
        }
        let amount = i128::from(leg.amount_minor);
        let payer = reconstructed
            .get_mut(&leg.payer_id)
            .ok_or(NettingError::InvariantViolation("settlement payer is not a profile member"))?;
        *payer = payer
            .checked_add(amount)
            .ok_or(NettingError::ArithmeticOverflow("reconstructed payer position"))?;
        let payee = reconstructed
            .get_mut(&leg.payee_id)
            .ok_or(NettingError::InvariantViolation("settlement payee is not a profile member"))?;
        *payee = payee
            .checked_sub(amount)
            .ok_or(NettingError::ArithmeticOverflow("reconstructed payee position"))?;
        total_volume = total_volume
            .checked_add(u128::from(leg.amount_minor))
            .ok_or(NettingError::ArithmeticOverflow("settlement leg volume"))?;
    }

    if &reconstructed != positions {
        return Err(NettingError::InvariantViolation(
            "net settlement legs do not reconstruct member net positions",
        ));
    }
    if total_volume != expected_volume_minor {
        return Err(NettingError::InvariantViolation(
            "net settlement leg volume differs from net payable volume",
        ));
    }
    Ok(())
}

fn estimate_cost(
    leg_amounts_minor: &[u64],
    cost_model: &CostModel,
    include_netting_overhead: bool,
) -> Result<CostEstimate, NettingError> {
    let modeled_volume_minor = leg_amounts_minor.iter().try_fold(0_u128, |sum, amount| {
        sum.checked_add(u128::from(*amount))
            .ok_or(NettingError::ArithmeticOverflow("modeled volume"))
    })?;
    let leg_count = leg_amounts_minor.len();
    let transaction_fixed_cost_minor = u128::from(cost_model.fixed_fee_per_leg_minor)
        .checked_mul(leg_count as u128)
        .ok_or(NettingError::ArithmeticOverflow("transaction fixed fee total"))?;

    let batch_count = if leg_count == 0 {
        0
    } else {
        1 + (leg_count - 1) / cost_model.batch_capacity_legs
    };
    let batch_cost_minor = u128::from(cost_model.batch_fee_minor)
        .checked_mul(batch_count as u128)
        .ok_or(NettingError::ArithmeticOverflow("batch fee total"))?;
    let netting_overhead_minor = if include_netting_overhead {
        u128::from(cost_model.netting_overhead_minor)
    } else {
        0
    };
    let fixed_cost_minor = transaction_fixed_cost_minor
        .checked_add(batch_cost_minor)
        .and_then(|sum| sum.checked_add(netting_overhead_minor))
        .ok_or(NettingError::ArithmeticOverflow("total fixed cost"))?;

    let variable_cost_minor = leg_amounts_minor.iter().try_fold(0_u128, |sum, amount| {
        let numerator = u128::from(*amount)
            .checked_mul(u128::from(cost_model.variable_fee_bps))
            .ok_or(NettingError::ArithmeticOverflow("per-leg variable fee numerator"))?;
        let rounded = numerator
            .checked_add(BASIS_POINTS_DENOMINATOR - 1)
            .ok_or(NettingError::ArithmeticOverflow("per-leg variable fee rounding"))?
            / BASIS_POINTS_DENOMINATOR;
        let minimum = u128::from(cost_model.minimum_variable_fee_per_leg_minor);
        let mut fee = rounded.max(minimum);
        if let Some(maximum) = cost_model.maximum_variable_fee_per_leg_minor {
            fee = fee.min(u128::from(maximum));
        }
        sum.checked_add(fee)
            .ok_or(NettingError::ArithmeticOverflow("per-leg variable fee total"))
    })?;
    let total_cost_minor = fixed_cost_minor
        .checked_add(variable_cost_minor)
        .ok_or(NettingError::ArithmeticOverflow("total cost"))?;

    Ok(CostEstimate {
        modeled_volume_minor,
        modeled_leg_count: leg_count,
        transaction_fixed_cost_minor,
        batch_cost_minor,
        netting_overhead_minor,
        fixed_cost_minor,
        variable_cost_minor,
        total_cost_minor,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> NettingProfile {
        NettingProfile {
            network_id: "community-001".into(),
            asset_profile_id: "tend-minute-v1".into(),
            netting_set_id: "netting-set-2026-q4".into(),
            policy_digest: "policy-digest-v1".into(),
            member_ids: ["alice", "bob", "carol", "dave"]
                .into_iter()
                .map(str::to_string)
                .collect(),
            max_input_obligations: 100,
            max_single_obligation_minor: 1_000_000,
            max_abs_member_net_position_minor: 1_000_000,
            cost_model: CostModel {
                fixed_fee_per_leg_minor: 100,
                variable_fee_bps: 100,
                minimum_variable_fee_per_leg_minor: 0,
                maximum_variable_fee_per_leg_minor: None,
                batch_fee_minor: 0,
                batch_capacity_legs: 1_000,
                netting_overhead_minor: 0,
            },
        }
    }

    fn obligation(id: &str, payer: &str, payee: &str, amount: u64) -> Obligation {
        Obligation {
            canonical_id: id.into(),
            network_id: "community-001".into(),
            asset_profile_id: "tend-minute-v1".into(),
            netting_set_id: "netting-set-2026-q4".into(),
            payer_id: payer.into(),
            payee_id: payee.into(),
            amount_minor: amount,
            status: ObligationStatus::Accepted,
        }
    }

    #[test]
    fn closed_three_member_cycle_has_zero_net_settlement_volume() {
        let report = simulate_netting(
            &profile(),
            &[
                obligation("o1", "alice", "bob", 100),
                obligation("o2", "bob", "carol", 100),
                obligation("o3", "carol", "alice", 100),
            ],
        )
        .unwrap();

        assert_eq!(report.gross_volume_minor, 300);
        assert_eq!(report.gross_obligation_count, 3);
        assert_eq!(report.net_settlement_volume_minor, 0);
        assert!(report.net_settlement_legs.is_empty());
        assert_eq!(report.net_cost_estimate.total_cost_minor, 0);
        assert_eq!(report.modeled_savings_minor, 303);
        assert!(report.all_supplied_obligations_eligible);
        assert!(!report.settlement_executed);
    }

    #[test]
    fn intermediate_receivable_offsets_payable() {
        let report = simulate_netting(
            &profile(),
            &[
                obligation("o1", "alice", "bob", 100),
                obligation("o2", "bob", "carol", 60),
            ],
        )
        .unwrap();

        assert_eq!(report.gross_volume_minor, 160);
        assert_eq!(report.net_settlement_volume_minor, 100);
        assert_eq!(report.net_settlement_legs.len(), 2);
        assert_eq!(report.net_cost_estimate.total_cost_minor, 202);
        assert_eq!(report.gross_cost_estimate.total_cost_minor, 202);
        assert_eq!(report.modeled_savings_minor, 0);
        assert_eq!(
            report.net_settlement_legs,
            vec![
                SettlementLeg {
                    payer_id: "alice".into(),
                    payee_id: "bob".into(),
                    amount_minor: 40,
                },
                SettlementLeg {
                    payer_id: "alice".into(),
                    payee_id: "carol".into(),
                    amount_minor: 60,
                },
            ]
        );
    }

    #[test]
    fn reciprocal_obligations_collapse_to_one_net_leg() {
        let report = simulate_netting(
            &profile(),
            &[
                obligation("o1", "alice", "bob", 100),
                obligation("o2", "bob", "alice", 60),
            ],
        )
        .unwrap();
        assert_eq!(report.gross_volume_minor, 160);
        assert_eq!(report.net_settlement_volume_minor, 40);
        assert_eq!(report.net_settlement_legs.len(), 1);
        assert_eq!(report.net_settlement_legs[0].amount_minor, 40);
    }

    #[test]
    fn duplicate_physical_observation_of_same_obligation_counts_once() {
        let o = obligation("same-id", "alice", "bob", 25);
        let report = simulate_netting(&profile(), &[o.clone(), o]).unwrap();
        assert_eq!(report.gross_obligation_count, 1);
        assert_eq!(report.gross_volume_minor, 25);
        assert_eq!(report.duplicate_observation_count, 1);
    }

    #[test]
    fn conflicting_duplicate_canonical_id_fails_closed() {
        let a = obligation("same-id", "alice", "bob", 25);
        let b = obligation("same-id", "alice", "bob", 26);
        assert_eq!(
            simulate_netting(&profile(), &[a, b]).unwrap_err(),
            NettingError::ConflictingDuplicate {
                canonical_id: "same-id".into()
            }
        );
    }

    #[test]
    fn unresolved_obligation_is_excluded_and_coverage_is_incomplete() {
        let accepted = obligation("accepted", "alice", "bob", 25);
        let mut disputed = obligation("disputed", "bob", "carol", 10);
        disputed.status = ObligationStatus::Disputed;
        let report = simulate_netting(&profile(), &[accepted, disputed]).unwrap();
        assert_eq!(report.gross_obligation_count, 1);
        assert_eq!(report.excluded_obligations.len(), 1);
        assert_eq!(report.excluded_obligations[0].reason, ExcludedReason::Disputed);
        assert!(!report.all_supplied_obligations_eligible);
        assert!(!report.settlement_executed);
    }

    #[test]
    fn wrong_asset_profile_cannot_enter_netting_set() {
        let mut o = obligation("wrong-asset", "alice", "bob", 25);
        o.asset_profile_id = "zar-v1".into();
        assert!(matches!(
            simulate_netting(&profile(), &[o]),
            Err(NettingError::WrongScope {
                field: "asset profile ID",
                ..
            })
        ));
    }

    #[test]
    fn result_is_independent_of_input_order() {
        let inputs = vec![
            obligation("o1", "alice", "bob", 100),
            obligation("o2", "bob", "carol", 60),
            obligation("o3", "carol", "alice", 10),
        ];
        let first = simulate_netting(&profile(), &inputs).unwrap();
        let mut reverse = inputs;
        reverse.reverse();
        let second = simulate_netting(&profile(), &reverse).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn member_exposure_limit_is_checked() {
        let mut p = profile();
        p.max_abs_member_net_position_minor = 50;
        assert!(matches!(
            simulate_netting(&p, &[obligation("too-large", "alice", "bob", 100)]),
            Err(NettingError::ExposureLimitExceeded { .. })
        ));
    }

    #[test]
    fn variable_fee_rounds_up_without_floating_point() {
        let mut p = profile();
        p.cost_model.fixed_fee_per_leg_minor = 0;
        p.cost_model.variable_fee_bps = 1;
        let report = simulate_netting(&p, &[obligation("tiny", "alice", "bob", 1)]).unwrap();
        assert_eq!(report.gross_cost_estimate.variable_cost_minor, 1);
        assert_eq!(report.net_cost_estimate.variable_cost_minor, 1);
    }

    #[test]
    fn minimum_per_leg_fees_are_calculated_independently() {
        let mut p = profile();
        p.cost_model.fixed_fee_per_leg_minor = 0;
        p.cost_model.variable_fee_bps = 0;
        p.cost_model.minimum_variable_fee_per_leg_minor = 5;

        let report = simulate_netting(
            &p,
            &[
                obligation("small-1", "alice", "bob", 100),
                obligation("small-2", "bob", "alice", 60),
            ],
        )
        .unwrap();

        assert_eq!(report.gross_cost_estimate.variable_cost_minor, 10);
        assert_eq!(report.net_cost_estimate.variable_cost_minor, 5);
        assert_eq!(report.modeled_savings_minor, 5);
    }

    #[test]
    fn netting_overhead_can_make_netting_more_expensive() {
        let mut p = profile();
        p.cost_model.fixed_fee_per_leg_minor = 1;
        p.cost_model.variable_fee_bps = 0;
        p.cost_model.minimum_variable_fee_per_leg_minor = 0;
        p.cost_model.netting_overhead_minor = 10;

        let report = simulate_netting(
            &p,
            &[
                obligation("o1", "alice", "bob", 100),
                obligation("o2", "bob", "carol", 100),
                obligation("o3", "carol", "alice", 100),
            ],
        )
        .unwrap();

        assert_eq!(report.gross_cost_estimate.total_cost_minor, 3);
        assert_eq!(report.net_cost_estimate.total_cost_minor, 10);
        assert_eq!(report.modeled_savings_minor, -7);
        assert!(report.modeled_savings_bps.unwrap() < 0);
    }

    #[test]
    fn batch_and_maximum_variable_fee_are_enforced() {
        let mut p = profile();
        p.cost_model.fixed_fee_per_leg_minor = 0;
        p.cost_model.variable_fee_bps = 10_000;
        p.cost_model.minimum_variable_fee_per_leg_minor = 1;
        p.cost_model.maximum_variable_fee_per_leg_minor = Some(7);
        p.cost_model.batch_fee_minor = 11;
        p.cost_model.batch_capacity_legs = 1;

        let report = simulate_netting(
            &p,
            &[
                obligation("o1", "alice", "bob", 100),
                obligation("o2", "carol", "dave", 200),
            ],
        )
        .unwrap();

        assert_eq!(report.gross_cost_estimate.variable_cost_minor, 14);
        assert_eq!(report.gross_cost_estimate.batch_cost_minor, 22);
        assert_eq!(report.gross_cost_estimate.total_cost_minor, 36);
    }

    #[test]
    fn invalid_batch_capacity_and_fee_floor_fail_closed() {
        let mut p = profile();
        p.cost_model.batch_capacity_legs = 0;
        assert!(matches!(
            simulate_netting(&p, &[obligation("o1", "alice", "bob", 25)]),
            Err(NettingError::InvalidProfile("batch capacity must be positive"))
        ));

        let mut p = profile();
        p.cost_model.minimum_variable_fee_per_leg_minor = 5;
        p.cost_model.maximum_variable_fee_per_leg_minor = Some(4);
        assert!(matches!(
            simulate_netting(&p, &[obligation("o1", "alice", "bob", 25)]),
            Err(NettingError::InvalidProfile(
                "maximum variable fee per leg cannot be below its minimum"
            ))
        ));
    }

    #[test]
    fn zero_and_self_obligations_are_rejected() {
        assert!(matches!(
            simulate_netting(&profile(), &[obligation("zero", "alice", "bob", 0)]),
            Err(NettingError::ZeroAmount { .. })
        ));
        assert!(matches!(
            simulate_netting(&profile(), &[obligation("self", "alice", "alice", 1)]),
            Err(NettingError::SelfObligation { .. })
        ));
    }

    #[test]
    fn deterministic_500_obligation_stress_corpus_preserves_conservation() {
        let mut p = profile();
        p.max_input_obligations = 1_000;
        let members = ["alice", "bob", "carol", "dave"];
        let mut inputs = Vec::new();

        for i in 0_u64..500 {
            let payer_index = (i as usize) % members.len();
            let payee_index = (payer_index + 1 + (i as usize % 3)) % members.len();
            let amount = (i.wrapping_mul(37) % 1_000) + 1;
            inputs.push(obligation(
                &format!("stress-{i:04}"),
                members[payer_index],
                members[payee_index],
                amount,
            ));
        }

        let first = simulate_netting(&p, &inputs).unwrap();
        let mut reversed = inputs;
        reversed.reverse();
        let second = simulate_netting(&p, &reversed).unwrap();

        assert_eq!(first, second);
        assert_eq!(first.gross_obligation_count, 500);
        assert_eq!(
            first
                .net_positions
                .iter()
                .map(|position| position.net_position_minor)
                .sum::<i128>(),
            0
        );
        assert_eq!(
            first
                .net_settlement_legs
                .iter()
                .map(|leg| u128::from(leg.amount_minor))
                .sum::<u128>(),
            first.net_settlement_volume_minor
        );
        assert!(first.all_supplied_obligations_eligible);
        assert!(!first.settlement_executed);
    }

    #[test]
    fn scoped_policy_and_member_set_are_mandatory() {
        let mut p = profile();
        p.netting_set_id.clear();
        assert!(matches!(
            simulate_netting(&p, &[obligation("o1", "alice", "bob", 25)]),
            Err(NettingError::InvalidIdentifier("netting set ID"))
        ));

        let mut o = obligation("o2", "alice", "bob", 25);
        o.payer_id = "outsider".into();
        assert!(matches!(
            simulate_netting(&profile(), &[o]),
            Err(NettingError::UnknownMember { .. })
        ));
    }
}
