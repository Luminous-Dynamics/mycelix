// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Finances Integrity Zome
//! Entry types and validation for charges, payments, reserves, and budgets.

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// A monthly charge for a member
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MonthlyCharge {
    pub member: AgentPubKey,
    pub unit_hash: ActionHash,
    pub period_year: u16,
    pub period_month: u8,
    pub base_rent_cents: u64,
    pub maintenance_fee_cents: u64,
    pub utilities_cents: u64,
    pub reserve_contribution_cents: u64,
    pub total_cents: u64,
}

/// Method of payment
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum PaymentMethod {
    BankTransfer,
    MutualCredit,
    Cash,
    Check,
    TimeBankCredit,
}

/// A payment record
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Payment {
    pub member: AgentPubKey,
    pub charge_hash: Option<ActionHash>,
    pub amount_cents: u64,
    pub payment_method: PaymentMethod,
    pub paid_at: Timestamp,
    pub reference: Option<String>,
}

/// Type of reserve fund
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum FundType {
    CapitalReserve,
    OperatingReserve,
    EmergencyFund,
    ImprovementFund,
}

/// A reserve fund
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ReserveFund {
    pub name: String,
    pub fund_type: FundType,
    pub balance_cents: u64,
    pub target_cents: u64,
    pub description: String,
}

/// A budget category line item
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct BudgetCategory {
    pub name: String,
    pub allocated_cents: u64,
    pub spent_cents: u64,
}

/// An annual budget
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Budget {
    pub fiscal_year: u16,
    pub income_projected_cents: u64,
    pub expenses_projected_cents: u64,
    pub categories: Vec<BudgetCategory>,
    pub approved: bool,
    pub approved_at: Option<Timestamp>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    MonthlyCharge(MonthlyCharge),
    Payment(Payment),
    ReserveFund(ReserveFund),
    Budget(Budget),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Member to their charges
    MemberToCharge,
    /// Charge to payments
    ChargeToPayment,
    /// Member to their payments
    MemberToPayment,
    /// All reserve funds
    AllReserveFunds,
    /// Fiscal year to budget
    YearToBudget,
    /// Period anchor to charges
    PeriodToCharge,
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::MonthlyCharge(charge) => validate_create_charge(action, charge),
                EntryTypes::Payment(payment) => validate_create_payment(action, payment),
                EntryTypes::ReserveFund(fund) => validate_create_fund(action, fund),
                EntryTypes::Budget(budget) => validate_create_budget(action, budget),
            },
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash: _,
                original_entry_hash: _,
            } => validate_update_entry_type(action, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => match link_type {
            LinkTypes::MemberToCharge => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ChargeToPayment => Ok(ValidateCallbackResult::Valid),
            LinkTypes::MemberToPayment => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AllReserveFunds => Ok(ValidateCallbackResult::Valid),
            LinkTypes::YearToBudget => Ok(ValidateCallbackResult::Valid),
            LinkTypes::PeriodToCharge => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDeleteLink {
            link_type: _,
            original_action: _,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // This DHT op was previously left fully permissive (`Ok(Valid)`
            // unconditionally) -- the 22nd confirmed instance of this exact
            // bug pattern this pass. Found + fixed 2026-07-09 during the P0
            // author-binding pass. Route through the same per-type
            // validators as the StoreEntry perspective.
            OpUpdate::Entry { app_entry, action } => validate_update_entry_type(action, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(OpDelete { action }) => {
            // Also previously fully permissive. The coordinator never
            // calls delete_entry here, so this is pure hardening, zero
            // functional impact.
            let original = must_get_action(action.deletes_address.clone())?;
            if action.author != *original.action().author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete an entry".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

/// Shared per-entry-type update validation, called from BOTH the
/// StoreEntry (OpEntry::UpdateEntry) and RegisterUpdate DHT-op
/// perspectives so they agree.
fn validate_update_entry_type(
    action: Update,
    app_entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match app_entry {
        EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
        // No live update_entry call for MonthlyCharge (confirmed via
        // grep) -- previously silently accepted any field change,
        // including member/amounts. Made explicitly immutable.
        EntryTypes::MonthlyCharge(_) => Ok(ValidateCallbackResult::Invalid(
            "Monthly charges are immutable".into(),
        )),
        EntryTypes::Payment(_) => Ok(ValidateCallbackResult::Invalid(
            "Payments cannot be modified after creation".into(),
        )),
        EntryTypes::ReserveFund(fund) => validate_update_fund(action, fund),
        EntryTypes::Budget(budget) => validate_update_budget(action, budget),
    }
}

/// No author requirement: deposit_to_reserve has zero caller-identity
/// check in the coordinator (this zome has no steward/treasurer concept
/// at all to bind against, unlike mycelix-housing/clt's
/// stewardship_board) -- case (c), no established authority model.
/// Content is restricted to balance_cents -- this closes the wide-open
/// bug that previously let name/fund_type/target_cents/description
/// change unconditionally on update too.
fn validate_update_fund(action: Update, fund: ReserveFund) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: ReserveFund = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original fund not found".into()
        )))?;

    if fund.name != original.name
        || fund.fund_type != original.fund_type
        || fund.target_cents != original.target_cents
        || fund.description != original.description
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only balance_cents can change on a reserve fund update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// No author requirement: approve_budget likewise has zero
/// caller-identity check -- case (c). Content restricted to
/// approved/approved_at.
fn validate_update_budget(action: Update, budget: Budget) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: Budget = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original budget not found".into()
        )))?;

    if budget.fiscal_year != original.fiscal_year
        || budget.income_projected_cents != original.income_projected_cents
        || budget.expenses_projected_cents != original.expenses_projected_cents
        || budget.categories != original.categories
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only approved/approved_at can change on a budget update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// MonthlyCharge.member is deliberately NOT bound to the committer: the
/// coordinator's generate_monthly_charges is explicitly a batch-admin
/// operation that creates charges for a LIST of OTHER members at once
/// (no agent_info() call at all, by design) -- a third-party field.
/// Reviewed 2026-07-09 during the P0 author-binding pass; case (b). WHO
/// may call generate_monthly_charges at all is unchecked (no
/// steward/treasurer concept exists in this zome to bind against) --
/// case (c), a real but separate gap, not fixed here.
fn validate_create_charge(
    _action: Create,
    charge: MonthlyCharge,
) -> ExternResult<ValidateCallbackResult> {
    if charge.period_month < 1 || charge.period_month > 12 {
        return Ok(ValidateCallbackResult::Invalid(
            "Month must be between 1 and 12".into(),
        ));
    }
    if charge.period_year < 2020 || charge.period_year > 2100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Year must be between 2020 and 2100".into(),
        ));
    }
    let computed_total = charge.base_rent_cents
        + charge.maintenance_fee_cents
        + charge.utilities_cents
        + charge.reserve_contribution_cents;
    if charge.total_cents != computed_total {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Total ({}) must equal sum of components ({})",
            charge.total_cents, computed_total
        )));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Payment.member deliberately NOT bound to the committer, but this is a
/// disclosed, NOT-fixed gap rather than a clean case (b)/(c): the
/// coordinator's record_payment takes the full Payment struct with ZERO
/// caller-identity derivation, and there's no authorization model to
/// bind against either way. It's genuinely ambiguous whether this
/// should be self-report (a member recording their own bank transfer)
/// or third-party admin entry (a treasurer recording a cash/check
/// payment received in person, who cannot self-attest since the payer
/// may have no chain identity at all). Binding member == author would
/// break the legitimate admin-entry case; leaving it fully open permits
/// a self-serving forged payment record (falsely claiming to have paid
/// via BankTransfer/MutualCredit/TimeBankCredit) OR a malicious agent
/// falsely marking another member's charge as paid. Reviewed 2026-07-09
/// during the P0 author-binding pass -- same class of gap as
/// mycelix-identity/bridge's report_reputation and
/// mycelix-knowledge/inference's author_reputation (needs a real
/// treasurer/capability-grant authorization model, not simple
/// author-binding). Flagged for dedicated follow-up, not fixed here.
fn validate_create_payment(
    _action: Create,
    payment: Payment,
) -> ExternResult<ValidateCallbackResult> {
    if payment.amount_cents == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Payment amount must be greater than 0".into(),
        ));
    }
    if let Some(ref reference) = payment.reference {
        if reference.len() > 256 {
            return Ok(ValidateCallbackResult::Invalid(
                "Payment reference must be at most 256 characters".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_fund(
    _action: Create,
    fund: ReserveFund,
) -> ExternResult<ValidateCallbackResult> {
    if fund.name.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Fund name cannot be empty".into(),
        ));
    }
    if fund.target_cents == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Fund target must be greater than 0".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_budget(_action: Create, budget: Budget) -> ExternResult<ValidateCallbackResult> {
    if budget.fiscal_year < 2020 || budget.fiscal_year > 2100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Fiscal year must be between 2020 and 2100".into(),
        ));
    }
    if budget.categories.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Budget must have at least one category".into(),
        ));
    }
    for cat in &budget.categories {
        if cat.name.is_empty() {
            return Ok(ValidateCallbackResult::Invalid(
                "Budget category name cannot be empty".into(),
            ));
        }
    }
    // Verify category allocations sum to projected expenses
    let total_allocated: u64 = budget.categories.iter().map(|c| c.allocated_cents).sum();
    if total_allocated != budget.expenses_projected_cents {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Category allocations ({}) must equal projected expenses ({})",
            total_allocated, budget.expenses_projected_cents
        )));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod content_integrity_tests {
    use super::*;

    fn update_action(author: AgentPubKey) -> Update {
        Update {
            author,
            timestamp: Timestamp::from_micros(1),
            action_seq: 1,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            original_action_address: ActionHash::from_raw_36(vec![9u8; 36]),
            original_entry_address: EntryHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        }
    }

    fn me() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }

    #[test]
    fn update_entry_type_rejects_monthly_charge_update() {
        // No live update_entry call exists for MonthlyCharge --
        // dead-path immutability, testable without must_get_valid_record.
        let charge = MonthlyCharge {
            member: me(),
            unit_hash: ActionHash::from_raw_36(vec![2u8; 36]),
            period_year: 2026,
            period_month: 7,
            base_rent_cents: 100_000,
            maintenance_fee_cents: 5_000,
            utilities_cents: 3_000,
            reserve_contribution_cents: 2_000,
            total_cents: 110_000,
        };
        let result =
            validate_update_entry_type(update_action(me()), EntryTypes::MonthlyCharge(charge))
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_payment_update() {
        let payment = Payment {
            member: me(),
            charge_hash: None,
            amount_cents: 110_000,
            payment_method: PaymentMethod::BankTransfer,
            paid_at: Timestamp::from_micros(0),
            reference: None,
        };
        let result =
            validate_update_entry_type(update_action(me()), EntryTypes::Payment(payment)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
