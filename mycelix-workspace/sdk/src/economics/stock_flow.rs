// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Stock-flow-consistent economic dynamics primitives.
//!
//! This module is deliberately an accounting substrate, not an economic policy
//! engine. It incorporates the useful part of Keen/Minsky/Godley-style thinking:
//! monetary claims, liabilities, and real stocks must evolve through explicit
//! flows rather than appearing or disappearing implicitly.
//!
//! Design goals:
//! - every financial flow has an explicit source and destination;
//! - credit creation creates a matching asset and liability;
//! - debt repayment retires both sides of the claim;
//! - real-resource stocks cannot be changed without an explicit flow;
//! - diagnostics expose leverage, debt-service pressure, and liquidity without
//!   declaring a preferred policy outcome.
//!
//! This is intentionally small enough to embed in simulations and larger
//! Mycelix economic models later.

use serde::{Deserialize, Serialize};

/// Stable identifier for an economic actor.
pub type ActorId = String;

/// A monetary stock held by an actor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct MonetaryStock {
    /// Physical/base money held by the actor.
    pub cash: i128,
    /// Bank deposits held as financial claims on a deposit issuer.
    pub deposits: i128,
    /// Other financial claims on actors (for example, loans).
    pub claims: i128,
    /// Debt liabilities owed by the actor.
    pub liabilities: i128,
    /// Deposit liabilities issued by the actor (normally a bank).
    pub deposit_liabilities: i128,
}

impl MonetaryStock {
    /// Total financial assets.
    pub fn assets(&self) -> i128 {
        self.cash + self.deposits + self.claims
    }

    /// Net financial position: financial assets minus liabilities.
    pub fn net_position(&self) -> i128 {
        self.assets() - self.liabilities - self.deposit_liabilities
    }
}

/// A real productive/resource stock.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct RealStock {
    /// Productive capital stock.
    pub productive_capital: i128,
    /// Material/resource stock available to the actor or commons.
    pub resources: i128,
}

/// Complete balance-sheet state for one actor.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ActorBalanceSheet {
    pub actor: ActorId,
    pub monetary: MonetaryStock,
    pub real: RealStock,
}

impl ActorBalanceSheet {
    pub fn new(actor: impl Into<ActorId>) -> Self {
        Self {
            actor: actor.into(),
            monetary: MonetaryStock::default(),
            real: RealStock::default(),
        }
    }

    pub fn net_worth(&self) -> i128 {
        self.monetary.net_position() + self.real.productive_capital + self.real.resources
    }
}

/// Monetary instrument used by a transfer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MonetaryInstrument {
    Cash,
    Deposit,
}

/// A transfer of money between actors.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MonetaryFlow {
    pub from: ActorId,
    pub to: ActorId,
    pub amount: i128,
    pub instrument: MonetaryInstrument,
}

impl MonetaryFlow {
    /// Create a physical/base-money transfer.
    pub fn new(from: impl Into<ActorId>, to: impl Into<ActorId>, amount: i128) -> Result<Self, String> {
        Self::with_instrument(from, to, amount, MonetaryInstrument::Cash)
    }

    /// Create a bank-deposit transfer.
    pub fn deposit_transfer(
        from: impl Into<ActorId>,
        to: impl Into<ActorId>,
        amount: i128,
    ) -> Result<Self, String> {
        Self::with_instrument(from, to, amount, MonetaryInstrument::Deposit)
    }

    pub fn with_instrument(
        from: impl Into<ActorId>,
        to: impl Into<ActorId>,
        amount: i128,
        instrument: MonetaryInstrument,
    ) -> Result<Self, String> {
        if amount <= 0 {
            return Err("monetary flow amount must be positive".into());
        }
        Ok(Self {
            from: from.into(),
            to: to.into(),
            amount,
            instrument,
        })
    }
}

/// Semantic category for a deposit-settled economic transfer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Hash, Ord, PartialOrd)]
pub enum EconomicFlowCategory {
    Wage,
    Interest,
    Tax,
    Transfer,
    Consumption,
}

impl EconomicFlowCategory {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Wage => "wage",
            Self::Interest => "interest",
            Self::Tax => "tax",
            Self::Transfer => "transfer",
            Self::Consumption => "consumption",
        }
    }
}

/// An income/expenditure transfer with an explicit economic category.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IncomeTransfer {
    pub payer: ActorId,
    pub recipient: ActorId,
    pub amount: i128,
    pub category: EconomicFlowCategory,
}

impl IncomeTransfer {
    pub fn new(
        payer: impl Into<ActorId>,
        recipient: impl Into<ActorId>,
        amount: i128,
    ) -> Result<Self, String> {
        Self::with_category(payer, recipient, amount, EconomicFlowCategory::Transfer)
    }

    pub fn with_category(
        payer: impl Into<ActorId>,
        recipient: impl Into<ActorId>,
        amount: i128,
        category: EconomicFlowCategory,
    ) -> Result<Self, String> {
        if amount <= 0 {
            return Err("income transfer amount must be positive".into());
        }
        Ok(Self {
            payer: payer.into(),
            recipient: recipient.into(),
            amount,
            category,
        })
    }
}

/// Explicit endogenous credit creation.
///
/// Credit creation increases the lender's financial asset and the borrower's
/// liability by the same amount. Aggregate net financial assets therefore
/// remain unchanged.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CreditCreation {
    pub lender: ActorId,
    pub borrower: ActorId,
    pub amount: i128,
}

impl CreditCreation {
    pub fn new(
        lender: impl Into<ActorId>,
        borrower: impl Into<ActorId>,
        amount: i128,
    ) -> Result<Self, String> {
        if amount <= 0 {
            return Err("credit amount must be positive".into());
        }
        Ok(Self {
            lender: lender.into(),
            borrower: borrower.into(),
            amount,
        })
    }
}

/// Explicit debt repayment.
///
/// Repayment retires the lender's financial asset and borrower's liability.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DebtRepayment {
    pub lender: ActorId,
    pub borrower: ActorId,
    pub amount: i128,
}

impl DebtRepayment {
    pub fn new(
        lender: impl Into<ActorId>,
        borrower: impl Into<ActorId>,
        amount: i128,
    ) -> Result<Self, String> {
        if amount <= 0 {
            return Err("repayment amount must be positive".into());
        }
        Ok(Self {
            lender: lender.into(),
            borrower: borrower.into(),
            amount,
        })
    }
}

/// Aggregate accounting state for a simulation timestep.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct EconomicState {
    pub actors: Vec<ActorBalanceSheet>,
    /// Cumulative nominal monetary flows during the current accounting period.
    pub monetary_flow_volume: i128,
    /// Cumulative newly-created credit during the current accounting period.
    pub credit_created: i128,
    /// Cumulative debt repayment during the current accounting period.
    pub debt_repaid: i128,
}

impl EconomicState {
    pub fn new(actors: Vec<ActorBalanceSheet>) -> Self {
        Self {
            actors,
            ..Self::default()
        }
    }

    fn actor_mut(&mut self, actor: &str) -> Result<&mut ActorBalanceSheet, String> {
        self.actors
            .iter_mut()
            .find(|a| a.actor == actor)
            .ok_or_else(|| format!("unknown actor: {actor}"))
    }

    fn actor_pair_mut(
        &mut self,
        left: &str,
        right: &str,
    ) -> Result<(&mut ActorBalanceSheet, &mut ActorBalanceSheet), String> {
        let left_idx = self
            .actors
            .iter()
            .position(|a| a.actor == left)
            .ok_or_else(|| format!("unknown actor: {left}"))?;
        let right_idx = self
            .actors
            .iter()
            .position(|a| a.actor == right)
            .ok_or_else(|| format!("unknown actor: {right}"))?;

        if left_idx == right_idx {
            return Err("actor pair must contain distinct actors".into());
        }

        if left_idx < right_idx {
            let (before, after) = self.actors.split_at_mut(right_idx);
            Ok((&mut before[left_idx], &mut after[0]))
        } else {
            let (before, after) = self.actors.split_at_mut(left_idx);
            Ok((&mut after[0], &mut before[right_idx]))
        }
    }

    /// Apply a monetary transfer while preserving aggregate monetary assets.
    pub fn apply_flow(&mut self, flow: &MonetaryFlow) -> Result<(), String> {
        let (sender, receiver) = self.actor_pair_mut(&flow.from, &flow.to)?;
        match flow.instrument {
            MonetaryInstrument::Cash => {
                if sender.monetary.cash < flow.amount {
                    return Err(format!(
                        "insufficient cash for {}: have {}, need {}",
                        sender.actor, sender.monetary.cash, flow.amount
                    ));
                }
                sender.monetary.cash -= flow.amount;
                receiver.monetary.cash += flow.amount;
            }
            MonetaryInstrument::Deposit => {
                if sender.monetary.deposits < flow.amount {
                    return Err(format!(
                        "insufficient deposits for {}: have {}, need {}",
                        sender.actor, sender.monetary.deposits, flow.amount
                    ));
                }
                sender.monetary.deposits -= flow.amount;
                receiver.monetary.deposits += flow.amount;
            }
        }
        self.monetary_flow_volume += flow.amount;
        Ok(())
    }

    /// Apply a deposit-settled income transfer.
    ///
    /// Deposits move between actors while the payer's equity falls and the
    /// recipient's equity rises by the same amount. This prevents income from
    /// appearing as unexplained net worth.
    pub fn apply_income_transfer(&mut self, transfer: &IncomeTransfer) -> Result<(), String> {
        let (payer, recipient) = self.actor_pair_mut(&transfer.payer, &transfer.recipient)?;
        if payer.monetary.deposits < transfer.amount {
            return Err(format!(
                "insufficient deposits for {}: have {}, need {}",
                payer.actor, payer.monetary.deposits, transfer.amount
            ));
        }
        payer.monetary.deposits -= transfer.amount;
        recipient.monetary.deposits = recipient
            .monetary
            .deposits
            .checked_add(transfer.amount)
            .ok_or_else(|| "recipient deposit overflow".to_string())?;
        self.monetary_flow_volume = self
            .monetary_flow_volume
            .checked_add(transfer.amount)
            .ok_or_else(|| "monetary flow counter overflow".to_string())?;
        Ok(())
    }

    /// Create endogenous bank credit with the SFC loan/deposit double entry.
    ///
    /// The lender records a loan claim and a matching deposit liability. The
    /// borrower records the deposit asset and the matching debt liability.
    /// This is the minimal private-money representation of loan creation.
    pub fn create_credit(&mut self, credit: &CreditCreation) -> Result<(), String> {
        let (lender, borrower) = self.actor_pair_mut(&credit.lender, &credit.borrower)?;
        lender.monetary.claims = lender
            .monetary
            .claims
            .checked_add(credit.amount)
            .ok_or_else(|| "lender claim overflow".to_string())?;
        lender.monetary.deposit_liabilities = lender
            .monetary
            .deposit_liabilities
            .checked_add(credit.amount)
            .ok_or_else(|| "deposit liability overflow".to_string())?;
        borrower.monetary.deposits = borrower
            .monetary
            .deposits
            .checked_add(credit.amount)
            .ok_or_else(|| "borrower deposit overflow".to_string())?;
        borrower.monetary.liabilities = borrower
            .monetary
            .liabilities
            .checked_add(credit.amount)
            .ok_or_else(|| "borrower liability overflow".to_string())?;
        self.credit_created = self
            .credit_created
            .checked_add(credit.amount)
            .ok_or_else(|| "credit counter overflow".to_string())?;
        Ok(())
    }

    /// Retire a debt claim after receiving repayment.
    pub fn repay_debt(&mut self, repayment: &DebtRepayment) -> Result<(), String> {
        let (lender, borrower) =
            self.actor_pair_mut(&repayment.lender, &repayment.borrower)?;

        if borrower.monetary.liabilities < repayment.amount
            || lender.monetary.claims < repayment.amount
        {
            return Err("repayment exceeds outstanding debt claim".into());
        }

        if borrower.monetary.deposits >= repayment.amount {
            if lender.monetary.deposit_liabilities < repayment.amount {
                return Err("deposit repayment requires a matching lender deposit liability".into());
            }
            borrower.monetary.deposits -= repayment.amount;
            lender.monetary.deposit_liabilities -= repayment.amount;
        } else if borrower.monetary.cash >= repayment.amount {
            borrower.monetary.cash -= repayment.amount;
            lender.monetary.cash = lender
                .monetary
                .cash
                .checked_add(repayment.amount)
                .ok_or_else(|| "lender cash overflow".to_string())?;
        } else {
            return Err(format!(
                "borrower {} cannot repay {} with deposits={} and cash={}",
                borrower.actor, repayment.amount, borrower.monetary.deposits, borrower.monetary.cash
            ));
        }

        borrower.monetary.liabilities -= repayment.amount;
        lender.monetary.claims -= repayment.amount;

        self.debt_repaid = self
            .debt_repaid
            .checked_add(repayment.amount)
            .ok_or_else(|| "repayment counter overflow".to_string())?;
        Ok(())
    }

    /// Sum of all monetary assets held by actors.
    pub fn aggregate_assets(&self) -> i128 {
        self.actors.iter().map(|a| a.monetary.assets()).sum()
    }

    /// Sum of all outstanding financial liabilities, including issued deposits.
    pub fn aggregate_liabilities(&self) -> i128 {
        self.actors
            .iter()
            .map(|a| a.monetary.liabilities + a.monetary.deposit_liabilities)
            .sum()
    }

    /// Sum of modeled internal financial claims, including deposits.
    pub fn aggregate_claims(&self) -> i128 {
        self.actors
            .iter()
            .map(|a| a.monetary.deposits + a.monetary.claims)
            .sum()
    }

    /// Aggregate net financial position.
    pub fn aggregate_net_financial_position(&self) -> i128 {
        self.actors.iter().map(|a| a.monetary.net_position()).sum()
    }

    /// Outstanding debt / monetary assets, when assets are non-zero.
    pub fn gross_leverage(&self) -> Option<f64> {
        let assets = self.aggregate_assets();
        if assets == 0 {
            None
        } else {
            Some(self.aggregate_liabilities as f64 / assets as f64)
        }
    }

    /// Credit creation minus repayment during the current period.
    pub fn net_credit_impulse(&self) -> i128 {
        self.credit_created - self.debt_repaid
    }

    /// Check the internal financial-instrument identity: every modeled claim
    /// has a matching modeled liability. This is deliberately narrower than
    /// requiring aggregate net financial assets to equal zero, because cash
    /// can be backed by an issuer or external sector not represented here.
    pub fn claims_liabilities_identity_holds(&self) -> bool {
        self.aggregate_claims() == self.aggregate_liabilities()
    }

    /// Backward-compatible accounting check. New code should prefer the
    /// instrument-level claims/liabilities identity above.
    pub fn accounting_identity_holds(&self) -> bool {
        self.claims_liabilities_identity_holds()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state() -> EconomicState {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.cash = 1_000;

        EconomicState::new(vec![
            bank,
            ActorBalanceSheet::new("household"),
            ActorBalanceSheet::new("firm"),
        ])
    }

    #[test]
    fn income_transfer_changes_net_worth_only_through_the_posted_deposit() {
        let mut s = state();
        s.create_credit(&CreditCreation::new("bank", "household", 500).unwrap())
            .unwrap();
        let before_household = s.actors[1].net_worth();
        let before_bank = s.actors[0].net_worth();

        s.apply_income_transfer(
            &IncomeTransfer::with_category(
                "household",
                "bank",
                100,
                EconomicFlowCategory::Wage,
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(s.actors[1].net_worth(), before_household - 100);
        assert_eq!(s.actors[0].net_worth(), before_bank + 100);
        assert_eq!(
            s.actors[0].net_worth() + s.actors[1].net_worth(),
            before_bank + before_household
        );
        assert!(s.claims_liabilities_identity_holds());
    }

    #[test]
    fn credit_creation_preserves_aggregate_net_financial_assets() {
        let mut s = state();
        s.create_credit(&CreditCreation::new("bank", "household", 500).unwrap())
            .unwrap();

        assert_eq!(s.aggregate_assets(), 2_000);
        assert_eq!(s.aggregate_claims(), 1_000);
        assert_eq!(s.aggregate_liabilities(), 1_000);
        assert!(s.claims_liabilities_identity_holds());
        assert_eq!(s.credit_created, 500);
    }

    #[test]
    fn repayment_retires_asset_and_liability() {
        let mut s = state();
        s.create_credit(&CreditCreation::new("bank", "household", 500).unwrap())
            .unwrap();
        s.repay_debt(&DebtRepayment::new("bank", "household", 500).unwrap())
            .unwrap();

        assert_eq!(s.aggregate_liabilities(), 0);
        assert_eq!(s.debt_repaid, 500);
        assert_eq!(s.actors[1].monetary.deposits, 0);
        assert_eq!(s.actors[0].monetary.claims, 0);
        assert_eq!(s.actors[0].monetary.deposit_liabilities, 0);
        assert_eq!(s.actors[0].monetary.cash, 1_000);
    }

    #[test]
    fn transfers_preserve_money_stock() {
        let mut s = state();
        s.apply_flow(&MonetaryFlow::new("bank", "household", 250).unwrap())
            .unwrap();

        assert_eq!(s.aggregate_assets(), 1_000);
        assert_eq!(s.actors[0].monetary.cash, 750);
        assert_eq!(s.actors[1].monetary.cash, 250);
    }

    #[test]
    fn invalid_flows_are_rejected() {
        let mut s = state();
        let err = s
            .apply_flow(&MonetaryFlow::new("household", "firm", 1).unwrap())
            .unwrap_err();
        assert!(err.contains("insufficient"));
    }

    #[test]
    fn self_pair_is_rejected() {
        let mut s = state();
        assert!(s
            .create_credit(&CreditCreation::new("bank", "bank", 1).unwrap())
            .is_err());
    }

    #[test]
    fn net_credit_impulse_is_credit_minus_repayment() {
        let mut s = state();
        s.create_credit(&CreditCreation::new("bank", "household", 200).unwrap())
            .unwrap();
        assert_eq!(s.net_credit_impulse(), 200);
    }
}
