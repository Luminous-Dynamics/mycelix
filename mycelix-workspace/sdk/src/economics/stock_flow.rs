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
    /// Trade receivables owed to the actor from deferred commercial sales.
    #[serde(default)]
    pub trade_receivables: i128,
    /// Debt liabilities owed by the actor.
    pub liabilities: i128,
    /// Deposit liabilities issued by the actor (normally a bank).
    pub deposit_liabilities: i128,
    /// Trade payables owed by the actor from deferred commercial purchases.
    #[serde(default)]
    pub trade_payables: i128,
}

impl MonetaryStock {
    /// Checked total financial assets.
    pub fn try_assets(&self) -> Result<i128, String> {
        self.cash
            .checked_add(self.deposits)
            .and_then(|value| value.checked_add(self.claims))
            .and_then(|value| value.checked_add(self.trade_receivables))
            .ok_or_else(|| "monetary asset overflow".into())
    }

    /// Total financial assets.
    pub fn assets(&self) -> i128 {
        self.try_assets().expect("monetary asset overflow")
    }

    /// Checked net financial position.
    pub fn try_net_position(&self) -> Result<i128, String> {
        self.try_assets()?
            .checked_sub(self.liabilities)
            .and_then(|value| value.checked_sub(self.deposit_liabilities))
            .and_then(|value| value.checked_sub(self.trade_payables))
            .ok_or_else(|| "net financial position overflow".into())
    }

    /// Net financial position: financial assets minus liabilities.
    pub fn net_position(&self) -> i128 {
        self.try_net_position()
            .expect("net financial position overflow")
    }

    /// Checked trade-credit position used by the working-capital layer.
    pub fn try_net_trade_position(&self) -> Result<i128, String> {
        self.trade_receivables
            .checked_sub(self.trade_payables)
            .ok_or_else(|| "net trade position overflow".into())
    }

    /// Trade-credit position used by the working-capital layer.
    pub fn net_trade_position(&self) -> i128 {
        self.try_net_trade_position()
            .expect("net trade position overflow")
    }
}

/// A real productive/resource stock.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct RealStock {
    /// Productive capital carrying amount in monetary units.
    ///
    /// The physical wear/depreciation process remains separately modeled.
    pub productive_capital: i128,
    /// Finished-goods inventory quantity in physical model units.
    ///
    /// Monetary carrying value is stored separately on ActorBalanceSheet.
    pub inventories: i128,
    /// Material/resource quantity in physical model units.
    ///
    /// This quantity is intentionally not added to monetary net worth.
    pub resources: i128,
}

/// Complete balance-sheet state for one actor.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ActorBalanceSheet {
    pub actor: ActorId,
    pub monetary: MonetaryStock,
    pub real: RealStock,
    /// Monetary carrying amount assigned to the actor's physical inventory.
    ///
    /// This is deliberately separate from real.inventories, which is a
    /// physical quantity. The value changes only through explicit inventory
    /// accounting transitions.
    #[serde(default)]
    pub inventory_carrying_value: i128,
}

impl ActorBalanceSheet {
    pub fn new(actor: impl Into<ActorId>) -> Self {
        Self {
            actor: actor.into(),
            monetary: MonetaryStock::default(),
            real: RealStock::default(),
            inventory_carrying_value: 0,
        }
    }

    /// Checked monetary net worth using only monetary-valued assets and liabilities.
    pub fn try_net_worth(&self) -> Result<i128, String> {
        self.monetary
            .try_net_position()?
            .checked_add(self.real.productive_capital)
            .and_then(|value| value.checked_add(self.inventory_carrying_value))
            .ok_or_else(|| "net worth overflow".into())
    }

    /// Monetary net worth using only monetary-valued assets and liabilities.
    ///
    /// Physical quantities such as resource units and inventory units remain
    /// in a separate dimensional accounting domain.
    pub fn net_worth(&self) -> i128 {
        self.try_net_worth().expect("net worth overflow")
    }

    /// Net operating working capital, excluding cash, deposits, and long-term
    /// financial claims. Physical inventory quantity remains outside this
    /// monetary measure.
    pub fn try_net_working_capital(&self) -> Result<i128, String> {
        self.inventory_carrying_value
            .checked_add(self.monetary.trade_receivables)
            .and_then(|value| value.checked_sub(self.monetary.trade_payables))
            .ok_or_else(|| "working-capital overflow".into())
    }

    pub fn net_working_capital(&self) -> i128 {
        self.try_net_working_capital()
            .expect("working-capital overflow")
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
    Investment,
}

impl EconomicFlowCategory {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Wage => "wage",
            Self::Interest => "interest",
            Self::Tax => "tax",
            Self::Transfer => "transfer",
            Self::Consumption => "consumption",
            Self::Investment => "investment",
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

/// Capital formation paid for through a deposit transfer.
///
/// The buyer exchanges deposits for newly-formed productive capital. The
/// producer receives the deposits; the buyer's net worth is unchanged because
/// one asset is exchanged for another, while the producer's net worth rises
/// through the sale proceeds. This is the minimal real/financial bridge for
/// investment before inventories and production functions are introduced.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CapitalInvestment {
    pub buyer: ActorId,
    pub producer: ActorId,
    pub amount: i128,
}

impl CapitalInvestment {
    pub fn new(
        buyer: impl Into<ActorId>,
        producer: impl Into<ActorId>,
        amount: i128,
    ) -> Result<Self, String> {
        if amount <= 0 {
            return Err("capital investment amount must be positive".into());
        }
        Ok(Self {
            buyer: buyer.into(),
            producer: producer.into(),
            amount,
        })
    }
}

/// Explicit production event.
///
/// Production consumes an input quantity from the producer's resource stock and
/// creates an explicit finished-goods inventory stock. Monetary consequences
/// (wages, sales, financing) remain separate transitions so the accounting
/// substrate never hides transfers inside a production primitive.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProductionEvent {
    pub producer: ActorId,
    pub resource_input: i128,
    pub output: i128,
}

impl ProductionEvent {
    pub fn new(
        producer: impl Into<ActorId>,
        resource_input: i128,
        output: i128,
    ) -> Result<Self, String> {
        if resource_input <= 0 || output <= 0 {
            return Err("production quantities must be positive".into());
        }
        Ok(Self { producer: producer.into(), resource_input, output })
    }
}

/// Transfer finished-goods inventory between actors.
///
/// This is deliberately a physical transition. Payment is a separate monetary
/// transition, so price formation and physical goods movement cannot become an
/// implicit side effect of one another.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct InventoryTransfer {
    pub from: ActorId,
    pub to: ActorId,
    pub quantity: i128,
}

impl InventoryTransfer {
    pub fn new(
        from: impl Into<ActorId>,
        to: impl Into<ActorId>,
        quantity: i128,
    ) -> Result<Self, String> {
        if quantity <= 0 {
            return Err("inventory transfer quantity must be positive".into());
        }
        Ok(Self {
            from: from.into(),
            to: to.into(),
            quantity,
        })
    }
}

/// Consume/draw down finished-goods inventory.
///
/// This represents final use, spoilage, destruction, or another explicit
/// inventory sink. Monetary consumption remains a separate transition.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct InventoryConsumption {
    pub consumer: ActorId,
    pub quantity: i128,
}

impl InventoryConsumption {
    pub fn new(consumer: impl Into<ActorId>, quantity: i128) -> Result<Self, String> {
        if quantity <= 0 {
            return Err("inventory consumption quantity must be positive".into());
        }
        Ok(Self {
            consumer: consumer.into(),
            quantity,
        })
    }
}

/// Explicit sale of finished goods at a stated monetary amount.
///
/// Physical quantity and monetary consideration are intentionally separate
/// fields. The transition transfers inventory and deposits atomically, while
/// the carrying-value/COGS layer remains a future explicit accounting policy.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GoodsSale {
    pub seller: ActorId,
    pub buyer: ActorId,
    pub quantity: i128,
    pub consideration: i128,
}

impl GoodsSale {
    pub fn new(
        seller: impl Into<ActorId>,
        buyer: impl Into<ActorId>,
        quantity: i128,
        consideration: i128,
    ) -> Result<Self, String> {
        if quantity <= 0 || consideration <= 0 {
            return Err("sale quantity and consideration must be positive".into());
        }
        Ok(Self {
            seller: seller.into(),
            buyer: buyer.into(),
            quantity,
            consideration,
        })
    }
}

/// Add an explicit monetary carrying amount to inventory that is already
/// physically held.
///
/// This records the result of an external cost-allocation/accounting
/// calculation. It does not move money or invent a financing source.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct InventoryCostAddition {
    pub actor: ActorId,
    pub quantity: i128,
    pub carrying_value: i128,
}

impl InventoryCostAddition {
    pub fn new(
        actor: impl Into<ActorId>,
        quantity: i128,
        carrying_value: i128,
    ) -> Result<Self, String> {
        if quantity <= 0 || carrying_value <= 0 {
            return Err("inventory cost addition quantity and carrying value must be positive".into());
        }
        Ok(Self {
            actor: actor.into(),
            quantity,
            carrying_value,
        })
    }
}

/// Relieve an explicit monetary carrying amount from inventory.
///
/// In the sale path this is the COGS posting. The physical sale and monetary
/// consideration remain a separate GoodsSale transition.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct InventoryCostRelief {
    pub actor: ActorId,
    pub quantity: i128,
    pub carrying_value: i128,
}

impl InventoryCostRelief {
    pub fn new(
        actor: impl Into<ActorId>,
        quantity: i128,
        carrying_value: i128,
    ) -> Result<Self, String> {
        if quantity <= 0 || carrying_value <= 0 {
            return Err("inventory cost relief quantity and carrying value must be positive".into());
        }
        Ok(Self {
            actor: actor.into(),
            quantity,
            carrying_value,
        })
    }
}

/// Explicit straight-line/current-period depreciation of productive capital.
///
/// Depreciation reduces the monetary carrying amount of productive capital and
/// therefore reduces the actor's derived equity. No physical wear or residual
/// material flow is inferred here; that belongs to the ecological layer.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Depreciation {
    pub actor: ActorId,
    pub amount: i128,
}

impl Depreciation {
    pub fn new(actor: impl Into<ActorId>, amount: i128) -> Result<Self, String> {
        if amount <= 0 {
            return Err("depreciation amount must be positive".into());
        }
        Ok(Self {
            actor: actor.into(),
            amount,
        })
    }
}

/// Deferred commercial sale that transfers goods now and settles the monetary
/// consideration later through trade credit.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TradeCreditSale {
    pub seller: ActorId,
    pub buyer: ActorId,
    pub quantity: i128,
    pub consideration: i128,
}

impl TradeCreditSale {
    pub fn new(
        seller: impl Into<ActorId>,
        buyer: impl Into<ActorId>,
        quantity: i128,
        consideration: i128,
    ) -> Result<Self, String> {
        if quantity <= 0 || consideration <= 0 {
            return Err("trade-credit sale quantity and consideration must be positive".into());
        }
        Ok(Self {
            seller: seller.into(),
            buyer: buyer.into(),
            quantity,
            consideration,
        })
    }
}

/// Settlement of an outstanding trade receivable/payable through deposits.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TradeCreditSettlement {
    pub seller: ActorId,
    pub buyer: ActorId,
    pub amount: i128,
}

impl TradeCreditSettlement {
    pub fn new(
        seller: impl Into<ActorId>,
        buyer: impl Into<ActorId>,
        amount: i128,
    ) -> Result<Self, String> {
        if amount <= 0 {
            return Err("trade-credit settlement amount must be positive".into());
        }
        Ok(Self {
            seller: seller.into(),
            buyer: buyer.into(),
            amount,
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

/// Explicit unilateral debt write-off outside a settlement transaction.
///
/// A write-off extinguishes an outstanding lender claim and the matching
/// borrower liability without moving cash or deposits. The lender's derived
/// equity falls while the borrower's derived equity rises by the same amount.
///
/// Semantic boundary: this primitive represents creditor-initiated write-off
/// or write-down without a bilateral debt-forgiveness agreement. Negotiated
/// debt forgiveness is a distinct accounting event: it requires a capital
/// transfer together with financial-claim extinction and must not be encoded
/// as this other-volume primitive.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DebtWriteOff {
    pub lender: ActorId,
    pub borrower: ActorId,
    pub amount: i128,
}

impl DebtWriteOff {
    pub fn new(
        lender: impl Into<ActorId>,
        borrower: impl Into<ActorId>,
        amount: i128,
    ) -> Result<Self, String> {
        if amount <= 0 {
            return Err("debt write-off amount must be positive".into());
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

/// Which monetary-valued real asset receives an explicit non-cash revaluation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RealAssetRevaluationTarget {
    ProductiveCapital,
    InventoryCarryingValue,
}

/// Explicit non-cash revaluation of a monetary-valued real asset.
///
/// A positive amount is a holding gain; a negative amount is a holding loss.
/// No cash, financial claim, or physical quantity is created. The balancing
/// change is therefore carried by the actor's derived equity/net worth.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RealAssetRevaluation {
    pub actor: ActorId,
    pub target: RealAssetRevaluationTarget,
    /// Signed change in the asset's monetary carrying amount.
    pub amount: i128,
}

impl RealAssetRevaluation {
    pub fn new(
        actor: impl Into<ActorId>,
        target: RealAssetRevaluationTarget,
        amount: i128,
    ) -> Result<Self, String> {
        if amount == 0 {
            return Err("real-asset revaluation amount must be non-zero".into());
        }
        Ok(Self {
            actor: actor.into(),
            target,
            amount,
        })
    }
}

/// Aggregate accounting state for a simulation timestep.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct EconomicState {
    pub actors: Vec<ActorBalanceSheet>,
    /// Cumulative nominal monetary-flow volume across the lifetime of this state.
    /// Period-specific flow totals are derived from `EconomicPeriodLedger`.
    pub monetary_flow_volume: i128,
    /// Cumulative newly-created credit across the lifetime of this state.
    /// Period-specific credit creation is derived from `EconomicPeriodLedger`.
    pub credit_created: i128,
    /// Cumulative debt repayment across the lifetime of this state.
    /// Period-specific debt repayment is derived from `EconomicPeriodLedger`.
    pub debt_repaid: i128,
    /// Cumulative debt write-offs across the lifetime of this state.
    /// Period-specific debt write-offs are derived from `EconomicPeriodLedger`.
    #[serde(default)]
    pub debt_written_off: i128,
}

impl EconomicState {
    pub fn new(actors: Vec<ActorBalanceSheet>) -> Self {
        Self {
            actors,
            ..Self::default()
        }
    }

    fn require_positive(amount: i128, label: &str) -> Result<(), String> {
        if amount <= 0 {
            Err(format!("{label} amount must be positive"))
        } else {
            Ok(())
        }
    }

    /// Validate the domain of a complete economic state before it enters a timestep.
    ///
    /// Stock and cumulative-counter fields represent non-negative quantities.
    /// Financial identities are checked separately because some valid closed
    /// models intentionally contain issuer-backed cash outside the modeled
    /// claim/liability set.
    pub fn validate(&self) -> Result<(), String> {
        let mut actor_ids = std::collections::BTreeSet::new();
        for actor in &self.actors {
            if actor.actor.is_empty() {
                return Err("economic state contains an empty actor id".into());
            }
            if !actor_ids.insert(actor.actor.as_str()) {
                return Err(format!("duplicate economic actor id: {}", actor.actor));
            }

            let monetary = &actor.monetary;
            for (label, value) in [
                ("cash", monetary.cash),
                ("deposits", monetary.deposits),
                ("claims", monetary.claims),
                ("trade receivables", monetary.trade_receivables),
                ("liabilities", monetary.liabilities),
                ("deposit liabilities", monetary.deposit_liabilities),
                ("trade payables", monetary.trade_payables),
            ] {
                if value < 0 {
                    return Err(format!(
                        "economic state contains negative {label} for actor {}",
                        actor.actor
                    ));
                }
            }

            for (label, value) in [
                ("productive capital", actor.real.productive_capital),
                ("inventories", actor.real.inventories),
                ("resources", actor.real.resources),
                ("inventory carrying value", actor.inventory_carrying_value),
            ] {
                if value < 0 {
                    return Err(format!(
                        "economic state contains negative {label} for actor {}",
                        actor.actor
                    ));
                }
            }
        }

        for (label, value) in [
            ("monetary flow volume", self.monetary_flow_volume),
            ("credit created", self.credit_created),
            ("debt repaid", self.debt_repaid),
            ("debt written off", self.debt_written_off),
        ] {
            if value < 0 {
                return Err(format!("economic state contains negative {label}"));
            }
        }

        let claims = self
            .try_aggregate_claims()
            .map_err(|error| format!("economic state claim aggregation failed: {error}"))?;
        let liabilities = self
            .try_aggregate_liabilities()
            .map_err(|error| {
                format!("economic state liability aggregation failed: {error}")
            })?;
        if claims != liabilities {
            return Err(format!(
                "economic state financial claims/liabilities do not reconcile: claims={claims}, liabilities={liabilities}"
            ));
        }

        Ok(())
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
        Self::require_positive(flow.amount, "monetary flow")?;

        let next_flow_volume = self
            .monetary_flow_volume
            .checked_add(flow.amount)
            .ok_or_else(|| "monetary flow counter overflow".to_string())?;
        let (sender, receiver) = self.actor_pair_mut(&flow.from, &flow.to)?;
        match flow.instrument {
            MonetaryInstrument::Cash => {
                if sender.monetary.cash < flow.amount {
                    return Err(format!(
                        "insufficient cash for {}: have {}, need {}",
                        sender.actor, sender.monetary.cash, flow.amount
                    ));
                }
                let receiver_cash = receiver
                    .monetary
                    .cash
                    .checked_add(flow.amount)
                    .ok_or_else(|| "receiver cash overflow".to_string())?;
                sender.monetary.cash -= flow.amount;
                receiver.monetary.cash = receiver_cash;
            }
            MonetaryInstrument::Deposit => {
                if sender.monetary.deposits < flow.amount {
                    return Err(format!(
                        "insufficient deposits for {}: have {}, need {}",
                        sender.actor, sender.monetary.deposits, flow.amount
                    ));
                }
                let receiver_deposits = receiver
                    .monetary
                    .deposits
                    .checked_add(flow.amount)
                    .ok_or_else(|| "receiver deposit overflow".to_string())?;
                sender.monetary.deposits -= flow.amount;
                receiver.monetary.deposits = receiver_deposits;
            }
        }
        self.monetary_flow_volume = next_flow_volume;
        Ok(())
    }

    /// Apply a deposit-settled income transfer.
    ///
    /// Deposits move between actors. Net worth changes are represented by the
    /// derived balance-sheet equity residual; there is intentionally no mutable
    /// equity field to update separately, preventing double counting.
    pub fn apply_income_transfer(&mut self, transfer: &IncomeTransfer) -> Result<(), String> {
        Self::require_positive(transfer.amount, "income transfer")?;

        let next_flow_volume = self
            .monetary_flow_volume
            .checked_add(transfer.amount)
            .ok_or_else(|| "monetary flow counter overflow".to_string())?;
        let (payer, recipient) = self.actor_pair_mut(&transfer.payer, &transfer.recipient)?;
        if payer.monetary.deposits < transfer.amount {
            return Err(format!(
                "insufficient deposits for {}: have {}, need {}",
                payer.actor, payer.monetary.deposits, transfer.amount
            ));
        }
        let recipient_deposits = recipient
            .monetary
            .deposits
            .checked_add(transfer.amount)
            .ok_or_else(|| "recipient deposit overflow".to_string())?;
        payer.monetary.deposits -= transfer.amount;
        recipient.monetary.deposits = recipient_deposits;
        self.monetary_flow_volume = next_flow_volume;
        Ok(())
    }

    /// Settle a capital investment: deposits move from buyer to producer and
    /// newly-formed productive capital is recorded by the buyer.
    pub fn apply_capital_investment(
        &mut self,
        investment: &CapitalInvestment,
    ) -> Result<(), String> {
        Self::require_positive(investment.amount, "capital investment")?;

        let next_flow_volume = self
            .monetary_flow_volume
            .checked_add(investment.amount)
            .ok_or_else(|| "monetary flow counter overflow".to_string())?;
        let (buyer, producer) = self.actor_pair_mut(&investment.buyer, &investment.producer)?;
        if buyer.monetary.deposits < investment.amount {
            return Err(format!(
                "insufficient deposits for {}: have {}, need {}",
                buyer.actor, buyer.monetary.deposits, investment.amount
            ));
        }
        let buyer_capital = buyer
            .real
            .productive_capital
            .checked_add(investment.amount)
            .ok_or_else(|| "productive capital overflow".to_string())?;
        let producer_deposits = producer
            .monetary
            .deposits
            .checked_add(investment.amount)
            .ok_or_else(|| "producer deposit overflow".to_string())?;
        buyer.monetary.deposits -= investment.amount;
        buyer.real.productive_capital = buyer_capital;
        producer.monetary.deposits = producer_deposits;
        self.monetary_flow_volume = next_flow_volume;
        Ok(())
    }

    /// Apply production as an explicit real-stock transformation.
    pub fn apply_production(&mut self, production: &ProductionEvent) -> Result<(), String> {
        Self::require_positive(production.resource_input, "production resource input")?;
        Self::require_positive(production.output, "production output")?;

        let producer = self.actor_mut(&production.producer)?;
        if producer.real.resources < production.resource_input {
            return Err(format!(
                "insufficient resources for {}: have {}, need {}",
                producer.actor, producer.real.resources, production.resource_input
            ));
        }
        let inventories = producer
            .real
            .inventories
            .checked_add(production.output)
            .ok_or_else(|| "inventory overflow".to_string())?;
        producer.real.resources -= production.resource_input;
        producer.real.inventories = inventories;
        Ok(())
    }

    /// Transfer finished-goods inventory without introducing an implicit
    /// monetary payment.
    pub fn apply_inventory_transfer(
        &mut self,
        transfer: &InventoryTransfer,
    ) -> Result<(), String> {
        Self::require_positive(transfer.quantity, "inventory transfer")?;

        let (sender, receiver) = self.actor_pair_mut(&transfer.from, &transfer.to)?;
        if sender.real.inventories < transfer.quantity {
            return Err(format!(
                "insufficient inventory for {}: have {}, need {}",
                sender.actor, sender.real.inventories, transfer.quantity
            ));
        }
        let receiver_inventories = receiver
            .real
            .inventories
            .checked_add(transfer.quantity)
            .ok_or_else(|| "inventory transfer overflow".to_string())?;
        sender.real.inventories -= transfer.quantity;
        receiver.real.inventories = receiver_inventories;
        Ok(())
    }

    /// Draw down finished-goods inventory without creating an implicit
    /// monetary flow.
    pub fn apply_inventory_consumption(
        &mut self,
        consumption: &InventoryConsumption,
    ) -> Result<(), String> {
        Self::require_positive(consumption.quantity, "inventory consumption")?;

        let consumer = self.actor_mut(&consumption.consumer)?;
        if consumer.real.inventories < consumption.quantity {
            return Err(format!(
                "insufficient inventory for {}: have {}, need {}",
                consumer.actor, consumer.real.inventories, consumption.quantity
            ));
        }
        consumer.real.inventories -= consumption.quantity;
        Ok(())
    }

    /// Settle a goods sale: inventory quantity moves physically while deposits
    /// move monetarily. Inventory carrying value is unchanged; explicit
    /// cost-relief and buyer-side cost-addition transitions carry the accounting.
    pub fn apply_goods_sale(&mut self, sale: &GoodsSale) -> Result<(), String> {
        Self::require_positive(sale.quantity, "goods sale quantity")?;
        Self::require_positive(sale.consideration, "goods sale consideration")?;

        let next_flow_volume = self
            .monetary_flow_volume
            .checked_add(sale.consideration)
            .ok_or_else(|| "monetary flow counter overflow".to_string())?;
        let (seller, buyer) = self.actor_pair_mut(&sale.seller, &sale.buyer)?;
        if seller.real.inventories < sale.quantity {
            return Err(format!(
                "insufficient inventory for {}: have {}, need {}",
                seller.actor, seller.real.inventories, sale.quantity
            ));
        }
        if buyer.monetary.deposits < sale.consideration {
            return Err(format!(
                "insufficient deposits for {}: have {}, need {}",
                buyer.actor, buyer.monetary.deposits, sale.consideration
            ));
        }
        let buyer_inventories = buyer
            .real
            .inventories
            .checked_add(sale.quantity)
            .ok_or_else(|| "buyer inventory overflow".to_string())?;
        let seller_deposits = seller
            .monetary
            .deposits
            .checked_add(sale.consideration)
            .ok_or_else(|| "seller deposit overflow".to_string())?;
        seller.real.inventories -= sale.quantity;
        buyer.real.inventories = buyer_inventories;
        buyer.monetary.deposits -= sale.consideration;
        seller.monetary.deposits = seller_deposits;
        self.monetary_flow_volume = next_flow_volume;
        Ok(())
    }

    /// Add an explicit carrying amount to physically-held inventory.
    pub fn apply_inventory_cost_addition(
        &mut self,
        addition: &InventoryCostAddition,
    ) -> Result<(), String> {
        Self::require_positive(addition.quantity, "inventory cost addition quantity")?;
        Self::require_positive(addition.carrying_value, "inventory cost addition carrying value")?;

        let actor = self.actor_mut(&addition.actor)?;
        if actor.real.inventories < addition.quantity {
            return Err(format!(
                "inventory cost addition covers {} units for {}, but only {} are held",
                addition.quantity, actor.actor, actor.real.inventories
            ));
        }
        let carrying_value = actor
            .inventory_carrying_value
            .checked_add(addition.carrying_value)
            .ok_or_else(|| "inventory carrying value overflow".to_string())?;
        actor.inventory_carrying_value = carrying_value;
        Ok(())
    }

    /// Relieve an explicit carrying amount from physically-held inventory.
    ///
    /// In a normal sale sequence this is the COGS posting and should precede
    /// the corresponding GoodsSale quantity reduction.
    pub fn apply_inventory_cost_relief(
        &mut self,
        relief: &InventoryCostRelief,
    ) -> Result<(), String> {
        Self::require_positive(relief.quantity, "inventory cost relief quantity")?;
        Self::require_positive(relief.carrying_value, "inventory cost relief carrying value")?;

        let actor = self.actor_mut(&relief.actor)?;
        if actor.real.inventories < relief.quantity {
            return Err(format!(
                "inventory cost relief covers {} units for {}, but only {} are held",
                relief.quantity, actor.actor, actor.real.inventories
            ));
        }
        if actor.inventory_carrying_value < relief.carrying_value {
            return Err(format!(
                "inventory carrying value for {} is {}, need {}",
                actor.actor, actor.inventory_carrying_value, relief.carrying_value
            ));
        }
        actor.inventory_carrying_value -= relief.carrying_value;
        Ok(())
    }

    /// Apply an explicit non-cash revaluation without changing physical stocks or liquidity.
    pub fn apply_real_asset_revaluation(
        &mut self,
        revaluation: &RealAssetRevaluation,
    ) -> Result<(), String> {
        if revaluation.amount == 0 {
            return Err("real-asset revaluation amount must be non-zero".into());
        }

        let actor = self.actor_mut(&revaluation.actor)?;
        if matches!(
            revaluation.target,
            RealAssetRevaluationTarget::InventoryCarryingValue
        ) && actor.real.inventories == 0
        {
            return Err(format!(
                "cannot revalue inventory carrying value for {} without physical inventory",
                actor.actor
            ));
        }

        let current = match revaluation.target {
            RealAssetRevaluationTarget::ProductiveCapital => actor.real.productive_capital,
            RealAssetRevaluationTarget::InventoryCarryingValue => actor.inventory_carrying_value,
        };
        let next = current
            .checked_add(revaluation.amount)
            .ok_or_else(|| "real-asset revaluation overflow".to_string())?;
        if next < 0 {
            return Err(format!(
                "real-asset revaluation would make {:?} negative for {}",
                revaluation.target, actor.actor
            ));
        }

        match revaluation.target {
            RealAssetRevaluationTarget::ProductiveCapital => actor.real.productive_capital = next,
            RealAssetRevaluationTarget::InventoryCarryingValue => {
                actor.inventory_carrying_value = next
            }
        }
        Ok(())
    }

    /// Recognize an explicit depreciation charge against productive capital.
    pub fn apply_depreciation(&mut self, depreciation: &Depreciation) -> Result<(), String> {
        Self::require_positive(depreciation.amount, "depreciation")?;

        let actor = self.actor_mut(&depreciation.actor)?;
        if actor.real.productive_capital < depreciation.amount {
            return Err(format!(
                "productive capital for {} is {}, need {}",
                actor.actor, actor.real.productive_capital, depreciation.amount
            ));
        }
        actor.real.productive_capital -= depreciation.amount;
        Ok(())
    }

    /// Record a deferred commercial sale. Inventory moves physically while
    /// the seller records a receivable and the buyer a matching payable.
    /// No deposit transfer occurs and no inventory carrying value is inferred.
    pub fn apply_trade_credit_sale(&mut self, sale: &TradeCreditSale) -> Result<(), String> {
        Self::require_positive(sale.quantity, "trade-credit sale quantity")?;
        Self::require_positive(sale.consideration, "trade-credit sale consideration")?;

        let (seller, buyer) = self.actor_pair_mut(&sale.seller, &sale.buyer)?;
        if seller.real.inventories < sale.quantity {
            return Err(format!(
                "insufficient inventory for {}: have {}, need {}",
                seller.actor, seller.real.inventories, sale.quantity
            ));
        }
        let buyer_inventories = buyer
            .real
            .inventories
            .checked_add(sale.quantity)
            .ok_or_else(|| "buyer inventory overflow".to_string())?;
        let seller_receivables = seller
            .monetary
            .trade_receivables
            .checked_add(sale.consideration)
            .ok_or_else(|| "trade receivable overflow".to_string())?;
        let buyer_payables = buyer
            .monetary
            .trade_payables
            .checked_add(sale.consideration)
            .ok_or_else(|| "trade payable overflow".to_string())?;
        seller.real.inventories -= sale.quantity;
        buyer.real.inventories = buyer_inventories;
        seller.monetary.trade_receivables = seller_receivables;
        buyer.monetary.trade_payables = buyer_payables;
        Ok(())
    }

    /// Settle trade credit through deposits. This retires the matching
    /// receivable/payable without changing aggregate deposit volume.
    pub fn apply_trade_credit_settlement(
        &mut self,
        settlement: &TradeCreditSettlement,
    ) -> Result<(), String> {
        Self::require_positive(settlement.amount, "trade-credit settlement")?;

        let next_flow_volume = self
            .monetary_flow_volume
            .checked_add(settlement.amount)
            .ok_or_else(|| "monetary flow counter overflow".to_string())?;
        let (seller, buyer) = self.actor_pair_mut(&settlement.seller, &settlement.buyer)?;
        if seller.monetary.trade_receivables < settlement.amount
            || buyer.monetary.trade_payables < settlement.amount
        {
            return Err("trade-credit settlement exceeds outstanding receivable/payable".into());
        }
        if buyer.monetary.deposits < settlement.amount {
            return Err(format!(
                "buyer {} cannot settle {} with deposits={}",
                buyer.actor, settlement.amount, buyer.monetary.deposits
            ));
        }
        let seller_deposits = seller
            .monetary
            .deposits
            .checked_add(settlement.amount)
            .ok_or_else(|| "seller deposit overflow".to_string())?;
        buyer.monetary.deposits -= settlement.amount;
        seller.monetary.deposits = seller_deposits;
        seller.monetary.trade_receivables -= settlement.amount;
        buyer.monetary.trade_payables -= settlement.amount;
        self.monetary_flow_volume = next_flow_volume;
        Ok(())
    }

    /// Create endogenous bank credit with the SFC loan/deposit double entry.
    ///
    /// The lender records a loan claim and a matching deposit liability. The
    /// borrower records the deposit asset and the matching debt liability.
    /// This is the minimal private-money representation of loan creation.
    pub fn create_credit(&mut self, credit: &CreditCreation) -> Result<(), String> {
        Self::require_positive(credit.amount, "credit creation")?;

        let next_credit_created = self
            .credit_created
            .checked_add(credit.amount)
            .ok_or_else(|| "credit counter overflow".to_string())?;
        let (lender, borrower) = self.actor_pair_mut(&credit.lender, &credit.borrower)?;
        let lender_claims = lender
            .monetary
            .claims
            .checked_add(credit.amount)
            .ok_or_else(|| "lender claim overflow".to_string())?;
        let lender_deposit_liabilities = lender
            .monetary
            .deposit_liabilities
            .checked_add(credit.amount)
            .ok_or_else(|| "deposit liability overflow".to_string())?;
        let borrower_deposits = borrower
            .monetary
            .deposits
            .checked_add(credit.amount)
            .ok_or_else(|| "borrower deposit overflow".to_string())?;
        let borrower_liabilities = borrower
            .monetary
            .liabilities
            .checked_add(credit.amount)
            .ok_or_else(|| "borrower liability overflow".to_string())?;
        lender.monetary.claims = lender_claims;
        lender.monetary.deposit_liabilities = lender_deposit_liabilities;
        borrower.monetary.deposits = borrower_deposits;
        borrower.monetary.liabilities = borrower_liabilities;
        self.credit_created = next_credit_created;
        Ok(())
    }

    /// Extinguish a debt claim and matching liability without settlement.
    pub fn write_off_debt(&mut self, write_off: &DebtWriteOff) -> Result<(), String> {
        Self::require_positive(write_off.amount, "debt write-off")?;
        let next_debt_written_off = self
            .debt_written_off
            .checked_add(write_off.amount)
            .ok_or_else(|| "write-off counter overflow".to_string())?;

        let (lender, borrower) =
            self.actor_pair_mut(&write_off.lender, &write_off.borrower)?;
        if borrower.monetary.liabilities < write_off.amount
            || lender.monetary.claims < write_off.amount
        {
            return Err("debt write-off exceeds outstanding debt claim".into());
        }

        borrower.monetary.liabilities -= write_off.amount;
        lender.monetary.claims -= write_off.amount;
        self.debt_written_off = next_debt_written_off;
        Ok(())
    }

    /// Retire a debt claim after receiving repayment.
    pub fn repay_debt(&mut self, repayment: &DebtRepayment) -> Result<(), String> {
        Self::require_positive(repayment.amount, "debt repayment")?;
        let next_flow_volume = self
            .monetary_flow_volume
            .checked_add(repayment.amount)
            .ok_or_else(|| "monetary flow counter overflow".to_string())?;
        Self::require_positive(repayment.amount, "debt repayment")?;

        let next_debt_repaid = self
            .debt_repaid
            .checked_add(repayment.amount)
            .ok_or_else(|| "repayment counter overflow".to_string())?;
        let (lender, borrower) =
            self.actor_pair_mut(&repayment.lender, &repayment.borrower)?;
        if borrower.monetary.liabilities < repayment.amount
            || lender.monetary.claims < repayment.amount
        {
            return Err("repayment exceeds outstanding debt claim".into());
        }

        let lender_cash = if borrower.monetary.deposits >= repayment.amount {
            if lender.monetary.deposit_liabilities < repayment.amount {
                return Err("deposit repayment requires a matching lender deposit liability".into());
            }
            None
        } else if borrower.monetary.cash >= repayment.amount {
            Some(
                lender
                    .monetary
                    .cash
                    .checked_add(repayment.amount)
                    .ok_or_else(|| "lender cash overflow".to_string())?,
            )
        } else {
            return Err(format!(
                "borrower {} cannot repay {} with deposits={} and cash={}",
                borrower.actor, repayment.amount, borrower.monetary.deposits, borrower.monetary.cash
            ));
        };

        if lender_cash.is_none() {
            borrower.monetary.deposits -= repayment.amount;
            lender.monetary.deposit_liabilities -= repayment.amount;
        } else {
            borrower.monetary.cash -= repayment.amount;
            lender.monetary.cash = lender_cash.unwrap();
        }
        borrower.monetary.liabilities -= repayment.amount;
        lender.monetary.claims -= repayment.amount;
        self.debt_repaid = next_debt_repaid;
        self.monetary_flow_volume = next_flow_volume;
        Ok(())
    }

    /// Checked sum of all monetary assets held by actors.
    pub fn try_aggregate_assets(&self) -> Result<i128, String> {
        self.actors.iter().try_fold(0i128, |sum, actor| {
            sum.checked_add(actor.monetary.try_assets()?)
                .ok_or_else(|| "aggregate assets overflow".to_string())
        })
    }

    /// Sum of all monetary assets held by actors.
    pub fn aggregate_assets(&self) -> i128 {
        self.try_aggregate_assets()
            .expect("aggregate assets overflow")
    }

    /// Checked sum of all outstanding financial liabilities, including issued deposits.
    pub fn try_aggregate_liabilities(&self) -> Result<i128, String> {
        self.actors.iter().try_fold(0i128, |sum, actor| {
            let liabilities = actor
                .monetary
                .liabilities
                .checked_add(actor.monetary.deposit_liabilities)
                .and_then(|value| value.checked_add(actor.monetary.trade_payables))
                .ok_or_else(|| "actor liabilities overflow".to_string())?;
            sum.checked_add(liabilities)
                .ok_or_else(|| "aggregate liabilities overflow".to_string())
        })
    }

    /// Sum of all outstanding financial liabilities, including issued deposits.
    pub fn aggregate_liabilities(&self) -> i128 {
        self.try_aggregate_liabilities()
            .expect("aggregate liabilities overflow")
    }

    /// Checked sum of modeled internal financial claims, including deposits.
    pub fn try_aggregate_claims(&self) -> Result<i128, String> {
        self.actors.iter().try_fold(0i128, |sum, actor| {
            let claims = actor
                .monetary
                .deposits
                .checked_add(actor.monetary.claims)
                .and_then(|value| value.checked_add(actor.monetary.trade_receivables))
                .ok_or_else(|| "actor claims overflow".to_string())?;
            sum.checked_add(claims)
                .ok_or_else(|| "aggregate claims overflow".to_string())
        })
    }

    /// Sum of modeled internal financial claims, including deposits.
    pub fn aggregate_claims(&self) -> i128 {
        self.try_aggregate_claims()
            .expect("aggregate claims overflow")
    }

    /// Checked aggregate net financial position.
    pub fn try_aggregate_net_financial_position(&self) -> Result<i128, String> {
        self.actors.iter().try_fold(0i128, |sum, actor| {
            sum.checked_add(actor.monetary.try_net_position()?)
                .ok_or_else(|| "aggregate net financial position overflow".to_string())
        })
    }

    /// Checked validation of the closed-model financial claim identities.
    ///
    /// Each modeled deposit, loan, and trade-credit claim must have a matching
    /// counterpart liability somewhere in the same EconomicState. This is a
    /// closed-system assertion; models with intentionally external counterparties
    /// should represent them explicitly as actors/sectors.
    pub fn closed_financial_rows_clear(&self) -> bool {
        self.actors.iter().try_fold(
            (
                0i128, // deposits
                0i128, // deposit liabilities
                0i128, // loan claims
                0i128, // debt
                0i128, // trade receivables
                0i128, // trade payables
            ),
            |(deposits, deposit_liabilities, loans, debt, receivables, payables), actor| {
                Some((
                    deposits.checked_add(actor.monetary.deposits)?,
                    deposit_liabilities.checked_add(actor.monetary.deposit_liabilities)?,
                    loans.checked_add(actor.monetary.claims)?,
                    debt.checked_add(actor.monetary.liabilities)?,
                    receivables.checked_add(actor.monetary.trade_receivables)?,
                    payables.checked_add(actor.monetary.trade_payables)?,
                ))
            },
        )
        .map(|(deposits, deposit_liabilities, loans, debt, receivables, payables)| {
            deposits == deposit_liabilities
                && loans == debt
                && receivables == payables
        })
        .unwrap_or(false)
    }

    /// Aggregate net financial position.
    pub fn aggregate_net_financial_position(&self) -> i128 {
        self.try_aggregate_net_financial_position()
            .expect("aggregate net financial position overflow")
    }

    /// Outstanding debt / monetary assets, when assets are non-zero.
    pub fn gross_leverage(&self) -> Option<f64> {
        let assets = self.aggregate_assets();
        if assets == 0 {
            None
        } else {
            Some(self.aggregate_liabilities() as f64 / assets as f64)
        }
    }

    /// Credit creation minus repayment during the current period.
    pub fn try_net_credit_impulse(&self) -> Result<i128, String> {
        self.credit_created
            .checked_sub(self.debt_repaid)
            .ok_or_else(|| "net credit impulse overflow".into())
    }

    pub fn net_credit_impulse(&self) -> i128 {
        self.try_net_credit_impulse()
            .expect("net credit impulse overflow")
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

    #[test]
    fn legacy_state_without_write_off_counter_defaults_to_zero() {
        let json = r#"{
            "actors": [],
            "monetary_flow_volume": 0,
            "credit_created": 0,
            "debt_repaid": 0
        }"#;
        let state: EconomicState = serde_json::from_str(json).unwrap();
        assert_eq!(state.debt_written_off, 0);
    }

    #[test]
    fn inventory_revaluation_requires_physical_inventory() {
        let mut state = EconomicState::new(vec![ActorBalanceSheet::new("firm")]);
        state.actors[0].inventory_carrying_value = 50;

        let gain = RealAssetRevaluation::new(
            "firm",
            RealAssetRevaluationTarget::InventoryCarryingValue,
            10,
        )
        .unwrap();

        assert!(state.apply_real_asset_revaluation(&gain).is_err());
        assert_eq!(state.actors[0].inventory_carrying_value, 50);
    }

    #[test]
    fn debt_write_off_extinguishes_claim_and_liability_without_cash() {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.claims = 100;
        let mut firm = ActorBalanceSheet::new("firm");
        firm.monetary.liabilities = 100;
        let mut state = EconomicState::new(vec![bank, firm]);

        let write_off = DebtWriteOff::new("bank", "firm", 40).unwrap();
        let before_liquidity = state.actors[0].monetary.cash
            + state.actors[0].monetary.deposits
            + state.actors[1].monetary.cash
            + state.actors[1].monetary.deposits;
        state.write_off_debt(&write_off).unwrap();

        assert_eq!(state.actors[0].monetary.claims, 60);
        assert_eq!(state.actors[1].monetary.liabilities, 60);
        assert_eq!(state.debt_written_off, 40);
        let after_liquidity = state.actors[0].monetary.cash
            + state.actors[0].monetary.deposits
            + state.actors[1].monetary.cash
            + state.actors[1].monetary.deposits;
        assert_eq!(after_liquidity, before_liquidity);
    }

    #[test]
    fn debt_write_off_rejects_excessive_extinguishment_atomically() {
        let mut bank = ActorBalanceSheet::new("bank");
        bank.monetary.claims = 10;
        let mut firm = ActorBalanceSheet::new("firm");
        firm.monetary.liabilities = 10;
        let mut state = EconomicState::new(vec![bank, firm]);

        let write_off = DebtWriteOff::new("bank", "firm", 11).unwrap();
        assert!(state.write_off_debt(&write_off).is_err());
        assert_eq!(state.actors[0].monetary.claims, 10);
        assert_eq!(state.actors[1].monetary.liabilities, 10);
        assert_eq!(state.debt_written_off, 0);
    }

    #[test]
    fn real_asset_revaluation_loss_cannot_create_negative_asset_stock() {
        let mut state = EconomicState::new(vec![ActorBalanceSheet::new("firm")]);
        state.actors[0].real.productive_capital = 10;

        let loss = RealAssetRevaluation::new(
            "firm",
            RealAssetRevaluationTarget::ProductiveCapital,
            -11,
        )
        .unwrap();

        assert!(state.apply_real_asset_revaluation(&loss).is_err());
        assert_eq!(state.actors[0].real.productive_capital, 10);
    }

    #[test]
    fn real_asset_revaluation_is_non_cash_and_preserves_physical_quantity() {
        let mut state = EconomicState::new(vec![ActorBalanceSheet::new("firm")]);
        state.actors[0].real.productive_capital = 100;
        state.actors[0].real.inventories = 5;
        state.actors[0].inventory_carrying_value = 50;

        let gain = RealAssetRevaluation::new(
            "firm",
            RealAssetRevaluationTarget::ProductiveCapital,
            25,
        )
        .unwrap();
        state.apply_real_asset_revaluation(&gain).unwrap();

        assert_eq!(state.actors[0].real.productive_capital, 125);
        assert_eq!(state.actors[0].real.inventories, 5);
        assert_eq!(state.actors[0].inventory_carrying_value, 50);
        assert_eq!(state.monetary_flow_volume, 0);
    }

    #[test]
    fn state_domain_validation_rejects_unmatched_financial_claims_and_empty_ids() {
        let mut invalid = state();
        invalid.actors[0].monetary.deposit_liabilities = 10;
        assert!(invalid.validate().is_err());

        let empty_id = EconomicState::new(vec![ActorBalanceSheet::new("")]);
        assert!(empty_id.validate().is_err());
    }

    #[test]
    fn state_domain_validation_rejects_negative_stocks_and_duplicate_ids() {
        let mut invalid = state();
        invalid.actors[1].monetary.deposits = -1;
        assert!(invalid.validate().is_err());

        let duplicate = EconomicState::new(vec![
            ActorBalanceSheet::new("same"),
            ActorBalanceSheet::new("same"),
        ]);
        assert!(duplicate.validate().is_err());

        let mut negative_counter = state();
        negative_counter.credit_created = -1;
        assert!(negative_counter.validate().is_err());
    }

    #[test]
    fn monetary_and_balance_sheet_checked_arithmetic_fails_closed() {
        let stock = MonetaryStock {
            cash: i128::MAX,
            deposits: 1,
            ..MonetaryStock::default()
        };
        assert!(stock.try_assets().is_err());
        assert!(stock.try_net_position().is_err());

        let actor = ActorBalanceSheet {
            actor: "overflow".into(),
            monetary: MonetaryStock::default(),
            real: RealStock {
                productive_capital: i128::MAX,
                ..RealStock::default()
            },
            inventory_carrying_value: 1,
        };
        assert!(actor.try_net_worth().is_err());

        let state = EconomicState::new(vec![
            ActorBalanceSheet {
                actor: "a".into(),
                monetary: MonetaryStock {
                    cash: i128::MAX,
                    ..MonetaryStock::default()
                },
                real: RealStock::default(),
                inventory_carrying_value: 0,
            },
            ActorBalanceSheet {
                actor: "b".into(),
                monetary: MonetaryStock {
                    cash: 1,
                    ..MonetaryStock::default()
                },
                real: RealStock::default(),
                inventory_carrying_value: 0,
            },
        ]);
        assert!(state.try_aggregate_assets().is_err());
        assert!(state.try_aggregate_net_financial_position().is_err());
    }
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
    fn capital_investment_converts_deposits_into_real_capital() {
        let mut s = state();
        s.actors[2].monetary.deposits = 500;

        let before_firm = s.actors[2].net_worth();
        let before_household = s.actors[1].net_worth();

        s.apply_capital_investment(
            &CapitalInvestment::new("firm", "household", 200).unwrap(),
        )
        .unwrap();

        assert_eq!(s.actors[2].monetary.deposits, 300);
        assert_eq!(s.actors[1].monetary.deposits, 200);
        assert_eq!(s.actors[2].real.productive_capital, 0);
        assert_eq!(s.actors[1].real.productive_capital, 200);
        assert_eq!(s.actors[2].net_worth(), before_firm);
        assert_eq!(s.actors[1].net_worth(), before_household + 200);
        assert!(s.claims_liabilities_identity_holds());
    }

    #[test]
    fn working_capital_is_monetary_and_excludes_physical_quantities() {
        let mut firm = ActorBalanceSheet::new("firm");
        firm.inventory_carrying_value = 50;
        firm.monetary.trade_receivables = 30;
        firm.monetary.trade_payables = 20;
        firm.real.inventories = 100;
        assert_eq!(firm.net_working_capital(), 60);
    }

    #[test]
    fn physical_resources_do_not_change_monetary_net_worth() {
        let mut s = state();
        s.actors.iter_mut().find(|a| a.actor == "firm").unwrap().real.resources = 10;
        assert_eq!(s.actors[2].net_worth(), 0);
    }

    #[test]
    fn trade_credit_creates_matching_receivable_and_payable() {
        let mut s = state();
        s.actors.iter_mut().find(|a| a.actor == "firm").unwrap().real.inventories = 10;
        s.apply_trade_credit_sale(
            &TradeCreditSale::new("firm", "household", 4, 80).unwrap(),
        )
        .unwrap();
        let firm = s.actors.iter().find(|a| a.actor == "firm").unwrap();
        let household = s.actors.iter().find(|a| a.actor == "household").unwrap();
        assert_eq!(firm.real.inventories, 6);
        assert_eq!(household.real.inventories, 4);
        assert_eq!(firm.monetary.trade_receivables, 80);
        assert_eq!(household.monetary.trade_payables, 80);
        assert!(s.claims_liabilities_identity_holds());
    }

    #[test]
    fn trade_credit_settlement_moves_deposits_and_retires_working_capital() {
        let mut s = state();
        s.actors.iter_mut().find(|a| a.actor == "firm").unwrap().real.inventories = 10;
        s.actors.iter_mut().find(|a| a.actor == "household").unwrap().monetary.deposits = 100;
        s.apply_trade_credit_sale(
            &TradeCreditSale::new("firm", "household", 4, 80).unwrap(),
        )
        .unwrap();
        s.apply_trade_credit_settlement(
            &TradeCreditSettlement::new("firm", "household", 80).unwrap(),
        )
        .unwrap();
        let firm = s.actors.iter().find(|a| a.actor == "firm").unwrap();
        let household = s.actors.iter().find(|a| a.actor == "household").unwrap();
        assert_eq!(firm.monetary.trade_receivables, 0);
        assert_eq!(household.monetary.trade_payables, 0);
        assert_eq!(firm.monetary.deposits, 80);
        assert_eq!(household.monetary.deposits, 20);
        assert!(s.claims_liabilities_identity_holds());
    }

    #[test]
    fn depreciation_reduces_productive_capital_explicitly() {
        let mut s = state();
        s.actors.iter_mut().find(|a| a.actor == "firm").unwrap().real.productive_capital = 100;

        s.apply_depreciation(&Depreciation::new("firm", 25).unwrap()).unwrap();

        assert_eq!(
            s.actors.iter().find(|a| a.actor == "firm").unwrap().real.productive_capital,
            75
        );
    }

    #[test]
    fn physical_inventory_quantity_is_not_monetary_value() {
        let mut s = state();
        s.actors.iter_mut().find(|a| a.actor == "firm").unwrap().real.inventories = 25;
        let before = s.actors[2].net_worth();

        s.apply_inventory_transfer(
            &InventoryTransfer::new("firm", "household", 10).unwrap(),
        ).unwrap();

        assert_eq!(s.actors[2].net_worth(), before);
        assert_eq!(s.actors[1].net_worth(), 0);
    }

    #[test]
    fn inventory_cost_accounting_is_explicit_and_dimensionally_separate() {
        let mut s = state();
        s.actors.iter_mut().find(|a| a.actor == "firm").unwrap().real.inventories = 10;

        s.apply_inventory_cost_addition(
            &InventoryCostAddition::new("firm", 10, 100).unwrap(),
        ).unwrap();
        assert_eq!(s.actors[2].inventory_carrying_value, 100);

        s.apply_inventory_cost_relief(
            &InventoryCostRelief::new("firm", 4, 40).unwrap(),
        ).unwrap();
        assert_eq!(s.actors[2].inventory_carrying_value, 60);
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
    fn malformed_public_transitions_are_rejected_without_mutation() {
        let mut s = state();
        s.actors.iter_mut().find(|a| a.actor == "bank").unwrap().monetary.cash = 10;
        let before = s.clone();

        assert!(s.apply_flow(&MonetaryFlow {
            from: "bank".into(),
            to: "household".into(),
            amount: -1,
            instrument: MonetaryInstrument::Cash,
        }).is_err());
        assert_eq!(s, before);

        assert!(s.create_credit(&CreditCreation {
            lender: "bank".into(),
            borrower: "household".into(),
            amount: 0,
        }).is_err());
        assert_eq!(s, before);

        assert!(s.repay_debt(&DebtRepayment {
            lender: "bank".into(),
            borrower: "household".into(),
            amount: -5,
        }).is_err());
        assert_eq!(s, before);
    }

    #[test]
    fn financial_flow_overflow_is_atomic() {
        let mut s = state();
        s.actors.iter_mut().find(|a| a.actor == "household").unwrap().monetary.cash = i128::MAX;
        s.actors.iter_mut().find(|a| a.actor == "bank").unwrap().monetary.cash = 1;
        let before = s.clone();
        assert!(s.apply_flow(&MonetaryFlow::new("bank", "household", 1).unwrap()).is_err());
        assert_eq!(s, before);
    }

    #[test]
    fn credit_creation_overflow_is_atomic() {
        let mut s = state();
        let lender = s.actors.iter_mut().find(|a| a.actor == "bank").unwrap();
        lender.monetary.claims = i128::MAX - 1;
        lender.monetary.deposit_liabilities = i128::MAX;
        let before = s.clone();
        assert!(s.create_credit(&CreditCreation::new("bank", "firm", 1).unwrap()).is_err());
        assert_eq!(s, before);
    }

    #[test]
    fn debt_repayment_counts_as_monetary_flow() {
        let mut s = state();
        s.create_credit(&CreditCreation::new("bank", "household", 10).unwrap())
            .unwrap();
        let before = s.monetary_flow_volume;

        s.repay_debt(&DebtRepayment::new("bank", "household", 4).unwrap())
            .unwrap();

        assert_eq!(s.monetary_flow_volume, before + 4);
        assert_eq!(s.debt_repaid, 4);
    }

    #[test]
    fn debt_repayment_counter_overflow_is_atomic() {
        let mut s = state();
        s.create_credit(&CreditCreation::new("bank", "household", 10).unwrap()).unwrap();
        s.debt_repaid = i128::MAX;
        let before = s.clone();
        assert!(s.repay_debt(&DebtRepayment::new("bank", "household", 1).unwrap()).is_err());
        assert_eq!(s, before);
    }

    #[test]
    fn trade_credit_settlement_overflow_is_atomic() {
        let mut s = state();
        s.actors.iter_mut().find(|a| a.actor == "firm").unwrap().monetary.deposits = i128::MAX;
        s.actors.iter_mut().find(|a| a.actor == "firm").unwrap().monetary.trade_receivables = 1;
        s.actors.iter_mut().find(|a| a.actor == "household").unwrap().monetary.deposits = 1;
        s.actors.iter_mut().find(|a| a.actor == "household").unwrap().monetary.trade_payables = 1;
        let before = s.clone();
        assert!(s.apply_trade_credit_settlement(&TradeCreditSettlement::new("firm", "household", 1).unwrap()).is_err());
        assert_eq!(s, before);
    }

    #[test]
    fn income_transfer_overflow_is_atomic() {
        let mut s = state();
        s.actors.iter_mut().find(|a| a.actor == "household").unwrap().monetary.deposits = 1;
        s.actors.iter_mut().find(|a| a.actor == "bank").unwrap().monetary.deposits = i128::MAX;
        let before = s.clone();
        assert!(s.apply_income_transfer(&IncomeTransfer::new("household", "bank", 1).unwrap()).is_err());
        assert_eq!(s, before);
    }

    #[test]
    fn capital_investment_overflow_is_atomic() {
        let mut s = state();
        s.actors.iter_mut().find(|a| a.actor == "firm").unwrap().monetary.deposits = 1;
        s.actors.iter_mut().find(|a| a.actor == "household").unwrap().real.productive_capital = i128::MAX;
        let before = s.clone();
        assert!(s.apply_capital_investment(&CapitalInvestment::new("firm", "household", 1).unwrap()).is_err());
        assert_eq!(s, before);
    }

    #[test]
    fn production_overflow_is_atomic() {
        let mut s = state();
        let producer = s.actors.iter_mut().find(|a| a.actor == "firm").unwrap();
        producer.real.resources = 1;
        producer.real.inventories = i128::MAX;
        let before = s.clone();
        assert!(s.apply_production(&ProductionEvent::new("firm", 1, 1).unwrap()).is_err());
        assert_eq!(s, before);
    }

    #[test]
    fn inventory_transfer_overflow_is_atomic() {
        let mut s = state();
        s.actors.iter_mut().find(|a| a.actor == "firm").unwrap().real.inventories = 1;
        s.actors.iter_mut().find(|a| a.actor == "household").unwrap().real.inventories = i128::MAX;
        let before = s.clone();
        assert!(s.apply_inventory_transfer(&InventoryTransfer::new("firm", "household", 1).unwrap()).is_err());
        assert_eq!(s, before);
    }

    #[test]
    fn goods_sale_overflow_is_atomic() {
        let mut s = state();
        s.actors.iter_mut().find(|a| a.actor == "firm").unwrap().real.inventories = 1;
        s.actors.iter_mut().find(|a| a.actor == "household").unwrap().monetary.deposits = 1;
        s.actors.iter_mut().find(|a| a.actor == "household").unwrap().real.inventories = i128::MAX;
        let before = s.clone();
        assert!(s.apply_goods_sale(&GoodsSale::new("firm", "household", 1, 1).unwrap()).is_err());
        assert_eq!(s, before);
    }

    #[test]
    fn trade_credit_sale_overflow_is_atomic() {
        let mut s = state();
        s.actors.iter_mut().find(|a| a.actor == "firm").unwrap().real.inventories = 1;
        s.actors.iter_mut().find(|a| a.actor == "firm").unwrap().monetary.trade_receivables = i128::MAX;
        let before = s.clone();
        assert!(s.apply_trade_credit_sale(&TradeCreditSale::new("firm", "household", 1, 1).unwrap()).is_err());
        assert_eq!(s, before);
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
    fn checked_net_credit_impulse_fails_closed_on_overflow() {
        let mut state = EconomicState::new(Vec::new());
        state.credit_created = i128::MIN;
        state.debt_repaid = 1;
        assert!(state.try_net_credit_impulse().is_err());
    }

    #[test]
    fn net_credit_impulse_is_credit_minus_repayment() {
        let mut s = state();
        s.create_credit(&CreditCreation::new("bank", "household", 200).unwrap())
            .unwrap();
        assert_eq!(s.net_credit_impulse(), 200);
    }

    #[test]
    fn closed_financial_rows_require_matching_counterparts() {
        let mut bank = ActorBalanceSheet::new("bank");
        let mut firm = ActorBalanceSheet::new("firm");
        bank.monetary.deposit_liabilities = 100;
        firm.monetary.deposits = 100;
        bank.monetary.claims = 50;
        firm.monetary.liabilities = 50;
        bank.monetary.trade_receivables = 30;
        firm.monetary.trade_payables = 30;
        let state = EconomicState::new(vec![bank, firm]);
        assert!(state.closed_financial_rows_clear());

        let mut broken = state.clone();
        broken.actors[1].monetary.trade_payables = 29;
        assert!(!broken.closed_financial_rows_clear());
    }

    #[test]
    fn aggregate_checked_sums_fail_closed_on_overflow() {
        let mut max_actor = ActorBalanceSheet::new("max");
        max_actor.monetary.deposits = i128::MAX;
        let mut overflow_actor = ActorBalanceSheet::new("overflow");
        overflow_actor.monetary.deposits = 1;
        let state = EconomicState::new(vec![max_actor, overflow_actor]);

        assert!(state.try_aggregate_claims().is_err());
        assert!(state.try_aggregate_assets().is_err());
    }
}