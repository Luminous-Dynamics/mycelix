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
    /// Total financial assets.
    pub fn assets(&self) -> i128 {
        self.cash + self.deposits + self.claims + self.trade_receivables
    }

    /// Net financial position: financial assets minus liabilities.
    pub fn net_position(&self) -> i128 {
        self.assets() - self.liabilities - self.deposit_liabilities - self.trade_payables
    }

    /// Trade-credit position used by the working-capital layer.
    pub fn net_trade_position(&self) -> i128 {
        self.trade_receivables - self.trade_payables
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

    /// Monetary net worth using only monetary-valued assets and liabilities.
    ///
    /// Physical quantities such as resource units and inventory units remain
    /// in a separate dimensional accounting domain.
    pub fn net_worth(&self) -> i128 {
        self.monetary.net_position()
            + self.real.productive_capital
            + self.inventory_carrying_value
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
    /// Deposits move between actors. Net worth changes are represented by the
    /// derived balance-sheet equity residual; there is intentionally no mutable
    /// equity field to update separately, preventing double counting.
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

    /// Settle a capital investment: deposits move from buyer to producer and
    /// newly-formed productive capital is recorded by the buyer.
    pub fn apply_capital_investment(
        &mut self,
        investment: &CapitalInvestment,
    ) -> Result<(), String> {
        let (buyer, producer) = self.actor_pair_mut(&investment.buyer, &investment.producer)?;
        if buyer.monetary.deposits < investment.amount {
            return Err(format!(
                "insufficient deposits for {}: have {}, need {}",
                buyer.actor, buyer.monetary.deposits, investment.amount
            ));
        }
        buyer.monetary.deposits -= investment.amount;
        buyer.real.productive_capital = buyer
            .real
            .productive_capital
            .checked_add(investment.amount)
            .ok_or_else(|| "productive capital overflow".to_string())?;
        producer.monetary.deposits = producer
            .monetary
            .deposits
            .checked_add(investment.amount)
            .ok_or_else(|| "producer deposit overflow".to_string())?;
        self.monetary_flow_volume = self
            .monetary_flow_volume
            .checked_add(investment.amount)
            .ok_or_else(|| "monetary flow counter overflow".to_string())?;
        Ok(())
    }

    /// Apply production as an explicit real-stock transformation.
    pub fn apply_production(&mut self, production: &ProductionEvent) -> Result<(), String> {
        let producer = self.actor_mut(&production.producer)?;
        if producer.real.resources < production.resource_input {
            return Err(format!(
                "insufficient resources for {}: have {}, need {}",
                producer.actor, producer.real.resources, production.resource_input
            ));
        }
        producer.real.resources -= production.resource_input;
        producer.real.inventories = producer
            .real
            .inventories
            .checked_add(production.output)
            .ok_or_else(|| "inventory overflow".to_string())?;
        Ok(())
    }

    /// Transfer finished-goods inventory without introducing an implicit
    /// monetary payment.
    pub fn apply_inventory_transfer(
        &mut self,
        transfer: &InventoryTransfer,
    ) -> Result<(), String> {
        let (sender, receiver) = self.actor_pair_mut(&transfer.from, &transfer.to)?;
        if sender.real.inventories < transfer.quantity {
            return Err(format!(
                "insufficient inventory for {}: have {}, need {}",
                sender.actor, sender.real.inventories, transfer.quantity
            ));
        }
        sender.real.inventories -= transfer.quantity;
        receiver.real.inventories = receiver
            .real
            .inventories
            .checked_add(transfer.quantity)
            .ok_or_else(|| "inventory transfer overflow".to_string())?;
        Ok(())
    }

    /// Draw down finished-goods inventory without creating an implicit
    /// monetary flow.
    pub fn apply_inventory_consumption(
        &mut self,
        consumption: &InventoryConsumption,
    ) -> Result<(), String> {
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
        seller.real.inventories -= sale.quantity;
        buyer.real.inventories = buyer.real.inventories
            .checked_add(sale.quantity)
            .ok_or_else(|| "buyer inventory overflow".to_string())?;
        buyer.monetary.deposits -= sale.consideration;
        seller.monetary.deposits = seller.monetary.deposits
            .checked_add(sale.consideration)
            .ok_or_else(|| "seller deposit overflow".to_string())?;
        self.monetary_flow_volume = self.monetary_flow_volume
            .checked_add(sale.consideration)
            .ok_or_else(|| "monetary flow counter overflow".to_string())?;
        Ok(())
    }

    /// Add an explicit carrying amount to physically-held inventory.
    pub fn apply_inventory_cost_addition(
        &mut self,
        addition: &InventoryCostAddition,
    ) -> Result<(), String> {
        let actor = self.actor_mut(&addition.actor)?;
        if actor.real.inventories < addition.quantity {
            return Err(format!(
                "inventory cost addition covers {} units for {}, but only {} are held",
                addition.quantity, actor.actor, actor.real.inventories
            ));
        }
        actor.inventory_carrying_value = actor
            .inventory_carrying_value
            .checked_add(addition.carrying_value)
            .ok_or_else(|| "inventory carrying value overflow".to_string())?;
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

    /// Recognize an explicit depreciation charge against productive capital.
    pub fn apply_depreciation(&mut self, depreciation: &Depreciation) -> Result<(), String> {
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
        let (seller, buyer) = self.actor_pair_mut(&sale.seller, &sale.buyer)?;
        if seller.real.inventories < sale.quantity {
            return Err(format!(
                "insufficient inventory for {}: have {}, need {}",
                seller.actor, seller.real.inventories, sale.quantity
            ));
        }
        seller.real.inventories -= sale.quantity;
        buyer.real.inventories = buyer
            .real
            .inventories
            .checked_add(sale.quantity)
            .ok_or_else(|| "buyer inventory overflow".to_string())?;
        seller.monetary.trade_receivables = seller
            .monetary
            .trade_receivables
            .checked_add(sale.consideration)
            .ok_or_else(|| "trade receivable overflow".to_string())?;
        buyer.monetary.trade_payables = buyer
            .monetary
            .trade_payables
            .checked_add(sale.consideration)
            .ok_or_else(|| "trade payable overflow".to_string())?;
        Ok(())
    }

    /// Settle trade credit through deposits. This retires the matching
    /// receivable/payable without changing aggregate deposit volume.
    pub fn apply_trade_credit_settlement(
        &mut self,
        settlement: &TradeCreditSettlement,
    ) -> Result<(), String> {
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
        buyer.monetary.deposits -= settlement.amount;
        seller.monetary.deposits = seller
            .monetary
            .deposits
            .checked_add(settlement.amount)
            .ok_or_else(|| "seller deposit overflow".to_string())?;
        seller.monetary.trade_receivables -= settlement.amount;
        buyer.monetary.trade_payables -= settlement.amount;
        self.monetary_flow_volume = self
            .monetary_flow_volume
            .checked_add(settlement.amount)
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
            .map(|a| {
                a.monetary.liabilities
                    + a.monetary.deposit_liabilities
                    + a.monetary.trade_payables
            })
            .sum()
    }

    /// Sum of modeled internal financial claims, including deposits.
    pub fn aggregate_claims(&self) -> i128 {
        self.actors
            .iter()
            .map(|a| a.monetary.deposits + a.monetary.claims + a.monetary.trade_receivables)
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
            Some(self.aggregate_liabilities() as f64 / assets as f64)
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
