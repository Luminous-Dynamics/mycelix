// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Sector-level projection of valuation changes.
//!
//! This matrix is intentionally distinct from the monetary transaction matrix,
//! the financial-claim flow matrix, and the other-volume matrix. A revaluation
//! changes the monetary carrying value of an existing asset or liability because
//! of a price/value change; it is not a payment and it is not an asset-volume
//! extinction such as a debt write-off.

use serde::{Deserialize, Serialize};

use super::sector_balance::{BalanceSheetInstrument, SectorAssignment, SectorBalanceSheet};
use super::sector_flow::EconomicSector;
use super::stock_flow::EconomicState;
use super::transition::EconomicTransition;

/// Category of a sector-level valuation change.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum RevaluationChangeCategory {
    /// Signed holding gain/loss on an existing monetary-valued real asset.
    RealAssetHoldingGainLoss,
}

/// One signed sector-level valuation change.
///
/// The amount is a value change, not a cash flow. Positive amounts are holding
/// gains; negative amounts are holding losses. Equity uses the balance-sheet
/// module's signed-liability convention, so its stock delta is the negation of
/// the revalued asset's carrying amount.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SectorRevaluationChange {
    pub sector: EconomicSector,
    pub instrument: BalanceSheetInstrument,
    pub category: RevaluationChangeCategory,
    pub amount: i128,
}

impl SectorRevaluationChange {
    pub fn new(
        sector: EconomicSector,
        instrument: BalanceSheetInstrument,
        category: RevaluationChangeCategory,
        amount: i128,
    ) -> Result<Self, String> {
        if amount == 0 {
            return Err("sector revaluation amount must be non-zero".into());
        }

        if !matches!(
            instrument,
            BalanceSheetInstrument::ProductiveCapital
                | BalanceSheetInstrument::InventoryCarryingValue
        ) {
            return Err(format!(
                "sector revaluation cannot target non-real instrument {:?}",
                instrument
            ));
        }

        Ok(Self {
            sector,
            instrument,
            category,
            amount,
        })
    }
}

/// Deterministic projection of valuation changes from the authoritative
/// transition sequence.
///
/// This currently contains only the narrow `RealAssetRevaluation` primitive.
/// Financial-instrument and FX valuation belong here only after those
/// instruments have explicit identity and currency semantics.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct SectorRevaluationChangeMatrix {
    pub changes: Vec<SectorRevaluationChange>,
}

impl SectorRevaluationChangeMatrix {
    pub fn push(&mut self, change: SectorRevaluationChange) {
        self.changes.push(change);
    }

    /// Sum signed valuation changes for one sector.
    pub fn try_net_change(&self, sector: EconomicSector) -> Result<i128, String> {
        self.changes
            .iter()
            .filter(|change| change.sector == sector)
            .try_fold(0i128, |sum, change| {
                sum.checked_add(change.amount)
                    .ok_or_else(|| "sector revaluation total overflow".to_string())
            })
    }

    pub fn net_change(&self, sector: EconomicSector) -> i128 {
        self.try_net_change(sector)
            .expect("sector revaluation total overflow")
    }

    /// Derive the valuation projection directly from the authoritative
    /// transition log.
    pub fn from_transitions(
        transitions: &[EconomicTransition],
        assignments: &[SectorAssignment],
    ) -> Result<Self, String> {
        let sector_for = |actor: &str| {
            let matching = assignments
                .iter()
                .filter(|assignment| assignment.actor == actor)
                .collect::<Vec<_>>();
            if matching.len() != 1 {
                return Err(format!(
                    "actor {actor} must have exactly one sector assignment"
                ));
            }
            Ok(matching[0].sector)
        };

        let mut matrix = Self::default();
        for transition in transitions {
            transition.validate()?;
            match transition {
                EconomicTransition::RealAssetRevaluation(revaluation) => {
                    let instrument = match revaluation.target {
                        super::stock_flow::RealAssetRevaluationTarget::ProductiveCapital => {
                            BalanceSheetInstrument::ProductiveCapital
                        }
                        super::stock_flow::RealAssetRevaluationTarget::InventoryCarryingValue => {
                            BalanceSheetInstrument::InventoryCarryingValue
                        }
                    };
                    matrix.push(SectorRevaluationChange::new(
                        sector_for(&revaluation.actor)?,
                        instrument,
                        RevaluationChangeCategory::RealAssetHoldingGainLoss,
                        revaluation.amount,
                    )?);
                }
                EconomicTransition::DebtForgiveness(_)
                | EconomicTransition::MonetaryTransfer(_)
                | EconomicTransition::IncomeTransfer(_)
                | EconomicTransition::CapitalInvestment(_)
                | EconomicTransition::Production(_)
                | EconomicTransition::InventoryTransfer(_)
                | EconomicTransition::InventoryConsumption(_)
                | EconomicTransition::GoodsSale(_)
                | EconomicTransition::TradeCreditSale(_)
                | EconomicTransition::TradeCreditSettlement(_)
                | EconomicTransition::InventoryCostAddition(_)
                | EconomicTransition::InventoryCostRelief(_)
                | EconomicTransition::Depreciation(_)
                | EconomicTransition::CreditCreation(_)
                | EconomicTransition::DebtRepayment(_)
                | EconomicTransition::DebtWriteOff(_) => {}
            }
        }

        Ok(matrix)
    }

    /// Validate exact actor-to-sector coverage and projection provenance.
    pub fn validate_against(
        &self,
        state: &EconomicState,
        assignments: &[SectorAssignment],
        transitions: &[EconomicTransition],
    ) -> Result<(), String> {
        SectorBalanceSheet::from_state(state, assignments)?;

        // Re-run the authoritative transition semantics so reporting cannot
        // accept a valuation that is structurally well formed but impossible
        // against the supplied opening state (for example, inventory
        // carrying-value revaluation with no physical inventory).
        super::transition::apply_step(state, 0, transitions, None)
            .map_err(|error| format!("sector revaluation state replay failed: {error}"))?;

        let expected = Self::from_transitions(transitions, assignments)?;
        if self.changes != expected.changes {
            return Err("sector revaluation projection does not match transition projection".into());
        }

        Ok(())
    }

    /// Validate the valuation projection against the actual closing balance sheet.
    ///
    /// Other transition types may legitimately change the same balance-sheet
    /// instruments (for example depreciation changes productive capital).
    /// Therefore this isolates valuation by replaying every transition except
    /// revaluation and comparing that counterfactual terminal state with the
    /// supplied post-state. The residual must equal this matrix's asset and
    /// equity deltas exactly.
    pub fn validate_against_balance_sheet_delta(
        &self,
        pre_state: &EconomicState,
        post_state: &EconomicState,
        assignments: &[SectorAssignment],
        transitions: &[EconomicTransition],
    ) -> Result<(), String> {
        SectorBalanceSheet::from_state(pre_state, assignments)?;
        let post_balance = SectorBalanceSheet::from_state(post_state, assignments)?;

        let non_revaluation = transitions
            .iter()
            .filter(|transition| {
                !matches!(transition, EconomicTransition::RealAssetRevaluation(_))
            })
            .cloned()
            .collect::<Vec<_>>();
        let (without_revaluation, _) =
            super::transition::apply_step(pre_state, 0, &non_revaluation, None)
                .map_err(|error| {
                    format!(
                        "sector revaluation baseline replay failed: {error}"
                    )
                })?;

        let baseline_balance = SectorBalanceSheet::from_state(&without_revaluation, assignments)?;
        let mut sectors = self.changes.iter().map(|change| change.sector).collect::<Vec<_>>();
        sectors.sort();
        sectors.dedup();

        for sector in sectors {
            for instrument in [
                BalanceSheetInstrument::ProductiveCapital,
                BalanceSheetInstrument::InventoryCarryingValue,
                BalanceSheetInstrument::Equity,
            ] {
                let actual = post_balance
                    .sector_instrument_total_checked(sector, instrument)?
                    .checked_sub(
                        baseline_balance
                            .sector_instrument_total_checked(sector, instrument)?,
                    )
                    .ok_or_else(|| "sector revaluation stock delta overflow".to_string())?;
                let expected = self.instrument_delta(sector, instrument)?;
                if actual != expected {
                    return Err(format!(
                        "sector {:?} {:?} revaluation delta mismatch: expected {}, actual {}",
                        sector, instrument, expected, actual
                    ));
                }
            }
        }

        Ok(())
    }

    /// Checked signed balance-sheet stock delta attributable to this valuation
    /// projection for one sector/instrument.
    ///
    /// Equity is stored as a signed liability-side residual, so a +gain in an
    /// asset's carrying value produces a -gain in the Equity row.
    pub fn instrument_delta(
        &self,
        sector: EconomicSector,
        instrument: BalanceSheetInstrument,
    ) -> Result<i128, String> {
        self.changes.iter().try_fold(0i128, |sum, change| {
            if change.sector != sector {
                return Ok(sum);
            }

            let delta = match instrument {
                BalanceSheetInstrument::Equity
                    if change.instrument == BalanceSheetInstrument::ProductiveCapital
                        || change.instrument == BalanceSheetInstrument::InventoryCarryingValue =>
                {
                    change
                        .amount
                        .checked_neg()
                        .ok_or_else(|| "sector revaluation equity delta overflow".to_string())?
                }
                _ if instrument == change.instrument => change.amount,
                _ => 0,
            };

            sum.checked_add(delta)
                .ok_or_else(|| "sector revaluation instrument delta overflow".to_string())
        })
    }

    pub fn equity_delta(&self, sector: EconomicSector) -> Result<i128, String> {
        self.instrument_delta(sector, BalanceSheetInstrument::Equity)
    }

    pub fn verify_against(
        &self,
        state: &EconomicState,
        assignments: &[SectorAssignment],
        transitions: &[EconomicTransition],
    ) -> Result<(), String> {
        self.validate_against(state, assignments, transitions)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::sector_financial_flow::SectorFinancialFlowMatrix;
    use crate::economics::sector_other_volume::SectorOtherVolumeChangeMatrix;
    use crate::economics::stock_flow::{
        ActorBalanceSheet, DebtWriteOff, RealAssetRevaluation, RealAssetRevaluationTarget,
    };

    fn assignments() -> Vec<SectorAssignment> {
        vec![
            SectorAssignment {
                actor: "bank".into(),
                sector: EconomicSector::Bank,
            },
            SectorAssignment {
                actor: "firm".into(),
                sector: EconomicSector::Firm,
            },
        ]
    }

    #[test]
    fn revaluation_projects_to_valuation_not_financial_or_other_volume_flow() {
        let mut firm = ActorBalanceSheet::new("firm");
        firm.real.productive_capital = 100;
        let state = EconomicState::new(vec![
            ActorBalanceSheet::new("bank"),
            firm,
        ]);
        let transitions = vec![EconomicTransition::RealAssetRevaluation(
            RealAssetRevaluation::new(
                "firm",
                RealAssetRevaluationTarget::ProductiveCapital,
                10,
            )
            .unwrap(),
        )];

        let revaluation =
            SectorRevaluationChangeMatrix::from_transitions(&transitions, &assignments()).unwrap();
        assert_eq!(revaluation.changes.len(), 1);
        assert_eq!(revaluation.changes[0].sector, EconomicSector::Firm);
        assert_eq!(
            revaluation.changes[0].instrument,
            BalanceSheetInstrument::ProductiveCapital
        );
        assert_eq!(revaluation.changes[0].amount, 10);
        assert_eq!(revaluation.net_change(EconomicSector::Firm), 10);
        assert_eq!(revaluation.equity_delta(EconomicSector::Firm).unwrap(), -10);

        assert!(SectorFinancialFlowMatrix::from_transitions(
            &transitions,
            &assignments(),
        )
        .unwrap()
        .flows
        .is_empty());
        assert!(SectorOtherVolumeChangeMatrix::from_transitions(
            &transitions,
            &assignments(),
        )
        .unwrap()
        .changes
        .is_empty());
        revaluation
            .verify_against(&state, &assignments(), &transitions)
            .unwrap();
    }

    #[test]
    fn inventory_revaluation_preserves_signed_gain_loss() {
        let mut firm = ActorBalanceSheet::new("firm");
        firm.real.inventories = 5;
        firm.inventory_carrying_value = 100;
        let state = EconomicState::new(vec![
            ActorBalanceSheet::new("bank"),
            firm,
        ]);

        let transitions = vec![
            EconomicTransition::RealAssetRevaluation(
                RealAssetRevaluation::new(
                    "firm",
                    RealAssetRevaluationTarget::InventoryCarryingValue,
                    -15,
                )
                .unwrap(),
            ),
            EconomicTransition::DebtWriteOff(DebtWriteOff::new("bank", "firm", 20).unwrap()),
        ];

        let matrix =
            SectorRevaluationChangeMatrix::from_transitions(&transitions, &assignments()).unwrap();
        assert_eq!(matrix.changes.len(), 1);
        assert_eq!(matrix.net_change(EconomicSector::Firm), -15);
        assert_eq!(
            matrix.instrument_delta(
                EconomicSector::Firm,
                BalanceSheetInstrument::InventoryCarryingValue,
            )
            .unwrap(),
            -15
        );
        assert_eq!(matrix.equity_delta(EconomicSector::Firm).unwrap(), 15);
        matrix
            .verify_against(&state, &assignments(), &transitions)
            .unwrap();
    }

    #[test]
    fn projection_rejects_non_real_balance_sheet_target() {
        assert!(SectorRevaluationChange::new(
            EconomicSector::Firm,
            BalanceSheetInstrument::Loans,
            RevaluationChangeCategory::RealAssetHoldingGainLoss,
            10,
        )
        .is_err());
    }



    #[test]
    fn revaluation_residual_reconciles_against_mixed_nonvaluation_changes() {
        let mut firm = ActorBalanceSheet::new("firm");
        firm.real.productive_capital = 100;
        let pre = EconomicState::new(vec![
            ActorBalanceSheet::new("bank"),
            firm,
        ]);
        let transitions = vec![
            EconomicTransition::Depreciation(
                crate::economics::stock_flow::Depreciation::new("firm", 20).unwrap(),
            ),
            EconomicTransition::RealAssetRevaluation(
                RealAssetRevaluation::new(
                    "firm",
                    RealAssetRevaluationTarget::ProductiveCapital,
                    15,
                )
                .unwrap(),
            ),
        ];
        let (post, _) = super::super::transition::apply_step(&pre, 1, &transitions, None).unwrap();
        let matrix =
            SectorRevaluationChangeMatrix::from_transitions(&transitions, &assignments()).unwrap();

        matrix
            .validate_against_balance_sheet_delta(
                &pre,
                &post,
                &assignments(),
                &transitions,
            )
            .unwrap();
    }

    #[test]
    fn revaluation_residual_rejects_tampered_post_state() {
        let mut firm = ActorBalanceSheet::new("firm");
        firm.real.productive_capital = 100;
        let pre = EconomicState::new(vec![ActorBalanceSheet::new("bank"), firm]);
        let transitions = vec![EconomicTransition::RealAssetRevaluation(
            RealAssetRevaluation::new(
                "firm",
                RealAssetRevaluationTarget::ProductiveCapital,
                10,
            )
            .unwrap(),
        )];
        let (mut post, _) =
            super::super::transition::apply_step(&pre, 1, &transitions, None).unwrap();
        post.actors
            .iter_mut()
            .find(|actor| actor.actor == "firm")
            .unwrap()
            .real
            .productive_capital += 1;

        let matrix =
            SectorRevaluationChangeMatrix::from_transitions(&transitions, &assignments()).unwrap();
        assert!(matrix
            .validate_against_balance_sheet_delta(
                &pre,
                &post,
                &assignments(),
                &transitions,
            )
            .is_err());
    }

    #[test]
    fn projection_rejects_unexecutable_inventory_revaluation() {
        let state = EconomicState::new(vec![
            ActorBalanceSheet::new("bank"),
            ActorBalanceSheet::new("firm"),
        ]);
        let transitions = vec![EconomicTransition::RealAssetRevaluation(
            RealAssetRevaluation::new(
                "firm",
                RealAssetRevaluationTarget::InventoryCarryingValue,
                10,
            )
            .unwrap(),
        )];
        let matrix =
            SectorRevaluationChangeMatrix::from_transitions(&transitions, &assignments()).unwrap();

        assert!(matrix
            .validate_against(&state, &assignments(), &transitions)
            .is_err());
    }

    #[test]
    fn revaluation_delta_fails_closed_on_equity_negation_overflow() {
        let mut matrix = SectorRevaluationChangeMatrix::default();
        matrix.push(
            SectorRevaluationChange::new(
                EconomicSector::Firm,
                BalanceSheetInstrument::ProductiveCapital,
                RevaluationChangeCategory::RealAssetHoldingGainLoss,
                i128::MIN,
            )
            .unwrap(),
        );

        assert!(matrix.equity_delta(EconomicSector::Firm).is_err());
    }

    #[test]
    fn revaluation_total_fails_closed_on_signed_overflow() {
        let mut matrix = SectorRevaluationChangeMatrix::default();
        matrix.push(
            SectorRevaluationChange::new(
                EconomicSector::Firm,
                BalanceSheetInstrument::ProductiveCapital,
                RevaluationChangeCategory::RealAssetHoldingGainLoss,
                i128::MAX,
            )
            .unwrap(),
        );
        matrix.push(
            SectorRevaluationChange::new(
                EconomicSector::Firm,
                BalanceSheetInstrument::InventoryCarryingValue,
                RevaluationChangeCategory::RealAssetHoldingGainLoss,
                1,
            )
            .unwrap(),
        );

        assert!(matrix.try_net_change(EconomicSector::Firm).is_err());
    }

    #[test]
    fn revaluation_projection_rejects_tampering() {
        let state = EconomicState::new(vec![
            ActorBalanceSheet::new("bank"),
            ActorBalanceSheet::new("firm"),
        ]);
        let transitions = vec![EconomicTransition::RealAssetRevaluation(
            RealAssetRevaluation::new(
                "firm",
                RealAssetRevaluationTarget::ProductiveCapital,
                10,
            )
            .unwrap(),
        )];

        let mut matrix =
            SectorRevaluationChangeMatrix::from_transitions(&transitions, &assignments()).unwrap();
        matrix.changes[0].amount = 11;

        assert!(matrix
            .verify_against(&state, &assignments(), &transitions)
            .is_err());
    }
}
