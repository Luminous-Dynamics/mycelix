// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Lending Integrity Zome
//! Updated to use HDI 0.7 patterns with FlatOp validation
use hdi::prelude::*;
use mycelix_bridge_entry_types::{did_for_author, require_did_is_author};

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Loan {
    pub id: String,
    pub borrower_did: String,
    pub lender_did: String,
    pub principal: f64,
    pub currency: String,
    pub interest_rate: f64,
    pub term_days: u32,
    pub collateral_ids: Vec<String>,
    pub status: LoanStatus,
    pub created: Timestamp,
    pub funded: Option<Timestamp>,
    pub maturity: Option<Timestamp>,
    pub repaid: Option<Timestamp>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum LoanStatus {
    Requested,
    Offered,
    Funded,
    Active,
    Repaid,
    Defaulted,
    Cancelled,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct LoanOffer {
    pub id: String,
    pub lender_did: String,
    pub max_amount: f64,
    pub min_amount: f64,
    pub currency: String,
    pub base_interest_rate: f64,
    pub min_credit_score: f64,
    pub max_term_days: u32,
    pub collateral_required: bool,
    pub active: bool,
    pub created: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct PaymentSchedule {
    pub loan_id: String,
    pub payments: Vec<ScheduledPayment>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ScheduledPayment {
    pub payment_number: u32,
    pub due_date: Timestamp,
    pub principal_amount: f64,
    pub interest_amount: f64,
    pub total_amount: f64,
    pub paid: bool,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Loan(Loan),
    LoanOffer(LoanOffer),
    PaymentSchedule(PaymentSchedule),
}

#[hdk_link_types]
pub enum LinkTypes {
    BorrowerToLoans,
    LenderToLoans,
    LenderToOffers,
    LoanToSchedule,
    CollateralToLoan,
    ActiveOffers,
}

/// Genesis self-check
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Validate a public app-entry create identically for entry and record authorities.
///
/// `must_get_valid_record` reports StoreRecord validity. Keeping the create/update
/// rules shared here ensures an update that depends on a predecessor gets the same
/// semantic theorem from StoreRecord authorities that StoreEntry authorities apply.
fn validate_app_entry_create(
    action: Create,
    app_entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match app_entry {
        EntryTypes::Loan(loan) => {
            validate_create_loan(EntryCreationAction::Create(action), loan)
        }
        EntryTypes::LoanOffer(offer) => {
            validate_create_loan_offer(EntryCreationAction::Create(action), offer)
        }
        EntryTypes::PaymentSchedule(schedule) => {
            validate_create_payment_schedule(EntryCreationAction::Create(action), schedule)
        }
    }
}

/// Validate a public app-entry update identically for entry and record authorities.
fn validate_app_entry_update(
    action: Update,
    app_entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match app_entry {
        EntryTypes::Loan(loan) => validate_update_loan(action, loan),
        EntryTypes::LoanOffer(offer) => validate_update_loan_offer(action, offer),
        EntryTypes::PaymentSchedule(_) => Ok(ValidateCallbackResult::Invalid(
            "Payment schedules cannot be updated".into(),
        )),
    }
}

/// Main validation callback using FlatOp pattern
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => {
                validate_app_entry_create(action, app_entry)
            }
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => validate_app_entry_update(action, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::StoreRecord(store_record) => match store_record {
            OpRecord::CreateEntry { app_entry, action } => {
                validate_app_entry_create(action, app_entry)
            }
            OpRecord::UpdateEntry {
                app_entry, action, ..
            } => validate_app_entry_update(action, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address,
            target_address,
            ..
        } => {
            // Validate hash lengths (39 bytes for Holochain hashes)
            let base_valid = base_address.as_ref().len() == 39;
            let target_valid = target_address.as_ref().len() == 39;

            match link_type {
                LinkTypes::BorrowerToLoans
                | LinkTypes::LenderToLoans
                | LinkTypes::LenderToOffers => {
                    // Agent to entry links
                    if !base_valid || !target_valid {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Link must connect valid agent and entry hashes".into(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
                LinkTypes::LoanToSchedule => {
                    // Entry to entry link
                    if !base_valid || !target_valid {
                        return Ok(ValidateCallbackResult::Invalid(
                            "LoanToSchedule must connect valid entry hashes".into(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
                LinkTypes::CollateralToLoan => {
                    // Entry to entry link - collateral to loan
                    if !base_valid || !target_valid {
                        return Ok(ValidateCallbackResult::Invalid(
                            "CollateralToLoan must connect valid entry hashes".into(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
                LinkTypes::ActiveOffers => {
                    // Anchor to offer entries
                    if !target_valid {
                        return Ok(ValidateCallbackResult::Invalid(
                            "ActiveOffers target must be a valid entry hash".into(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
            }
        }
        FlatOp::RegisterDeleteLink { link_type, .. } => {
            match link_type {
                // Loan to schedule links are immutable once created
                LinkTypes::LoanToSchedule => Ok(ValidateCallbackResult::Invalid(
                    "LoanToSchedule links cannot be deleted - payment schedules are immutable"
                        .into(),
                )),
                // Collateral links cannot be deleted while loan is active
                LinkTypes::CollateralToLoan => Ok(ValidateCallbackResult::Invalid(
                    "CollateralToLoan links cannot be deleted - collateral is locked".into(),
                )),
                _ => Ok(ValidateCallbackResult::Valid),
            }
        }
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn loan_validation_error(loan: &Loan) -> Option<String> {
    if !loan.borrower_did.starts_with("did:") {
        return Some("Borrower must be a valid DID".into());
    }
    if !loan.lender_did.is_empty() && !loan.lender_did.starts_with("did:") {
        return Some("Lender must be a valid DID when assigned".into());
    }
    if !loan.principal.is_finite() || loan.principal <= 0.0 {
        return Some("Principal must be finite and positive".into());
    }
    if !loan.interest_rate.is_finite() || !(0.0..=1.0).contains(&loan.interest_rate) {
        return Some("Interest rate must be finite and between 0 and 100%".into());
    }
    if loan.currency.is_empty() {
        return Some("Currency must not be empty".into());
    }
    if loan.term_days == 0 {
        return Some("Loan term must be at least one day".into());
    }
    if !loan.lender_did.is_empty() && loan.borrower_did == loan.lender_did {
        return Some("Cannot lend to yourself".into());
    }
    None
}

fn validate_create_loan(
    action: EntryCreationAction,
    loan: Loan,
) -> ExternResult<ValidateCallbackResult> {
    if let Some(msg) = loan_validation_error(&loan) {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    // Legacy loans are frozen to one canonical creation state. Later financial
    // states must be reached through predecessor-checked updates, never by
    // creating an already-funded/defaulted/repaid entry directly.
    if loan.status != LoanStatus::Requested {
        return Ok(ValidateCallbackResult::Invalid(
            "New legacy loans must begin in Requested state".into(),
        ));
    }
    if !loan.lender_did.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Requested loans cannot pre-assign a lender".into(),
        ));
    }
    if loan.funded.is_some() || loan.maturity.is_some() || loan.repaid.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "Requested loans cannot carry funded, maturity, or repaid timestamps".into(),
        ));
    }

    // `request_loan` accepts borrower_did from the client, so bind the request
    // to the action author at integrity validation. This prevents a caller from
    // publishing a debt request in somebody else's identity.
    let author_did = did_for_author(action.author());
    if let ValidateCallbackResult::Invalid(msg) =
        require_did_is_author("Loan", "borrower_did", &loan.borrower_did, &author_did)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn is_allowed_loan_transition(previous: &LoanStatus, next: &LoanStatus) -> bool {
    matches!(
        (previous, next),
        (LoanStatus::Requested, LoanStatus::Funded)
            | (LoanStatus::Requested, LoanStatus::Cancelled)
            // `Offered` is retained only for already-existing legacy entries.
            | (LoanStatus::Offered, LoanStatus::Funded)
            | (LoanStatus::Offered, LoanStatus::Cancelled)
            | (LoanStatus::Funded, LoanStatus::Active)
            | (LoanStatus::Funded, LoanStatus::Repaid)
            | (LoanStatus::Active, LoanStatus::Repaid)
            | (LoanStatus::Active, LoanStatus::Defaulted)
    )
}

fn validate_loan_update_against_previous(
    actor_did: &str,
    previous: &Loan,
    updated: &Loan,
) -> ValidateCallbackResult {
    if let Some(msg) = loan_validation_error(updated) {
        return ValidateCallbackResult::Invalid(msg);
    }

    if previous.id != updated.id {
        return ValidateCallbackResult::Invalid("Loan id is immutable".into());
    }
    if previous.borrower_did != updated.borrower_did {
        return ValidateCallbackResult::Invalid("Loan borrower is immutable".into());
    }
    if previous.principal != updated.principal {
        return ValidateCallbackResult::Invalid("Loan principal is immutable".into());
    }
    if previous.currency != updated.currency {
        return ValidateCallbackResult::Invalid("Loan currency is immutable".into());
    }
    if previous.term_days != updated.term_days {
        return ValidateCallbackResult::Invalid("Loan term is immutable".into());
    }
    if previous.collateral_ids != updated.collateral_ids {
        return ValidateCallbackResult::Invalid("Loan collateral set is immutable".into());
    }
    if previous.created != updated.created {
        return ValidateCallbackResult::Invalid("Loan creation timestamp is immutable".into());
    }

    if !is_allowed_loan_transition(&previous.status, &updated.status) {
        return ValidateCallbackResult::Invalid(format!(
            "Invalid legacy loan transition: {:?} -> {:?}",
            previous.status, updated.status
        ));
    }

    let funding_transition = matches!(
        (&previous.status, &updated.status),
        (LoanStatus::Requested, LoanStatus::Funded)
            | (LoanStatus::Offered, LoanStatus::Funded)
    );

    if funding_transition {
        if updated.lender_did.is_empty() {
            return ValidateCallbackResult::Invalid(
                "Funding must assign a lender DID".into(),
            );
        }
        if !previous.lender_did.is_empty() && previous.lender_did != updated.lender_did {
            return ValidateCallbackResult::Invalid(
                "An already-assigned lender cannot be replaced during funding".into(),
            );
        }
        if actor_did != updated.lender_did {
            return ValidateCallbackResult::Invalid(
                "Only the funding lender may fund a legacy loan".into(),
            );
        }
        if updated.funded.is_none() || updated.maturity.is_none() {
            return ValidateCallbackResult::Invalid(
                "Funding must record funded and maturity timestamps".into(),
            );
        }
        if updated.repaid.is_some() {
            return ValidateCallbackResult::Invalid(
                "A newly funded loan cannot already be repaid".into(),
            );
        }
        if let (Some(funded), Some(maturity)) = (updated.funded, updated.maturity) {
            if maturity <= funded {
                return ValidateCallbackResult::Invalid(
                    "Loan maturity must be after the funded timestamp".into(),
                );
            }
        }
    } else {
        if previous.lender_did != updated.lender_did {
            return ValidateCallbackResult::Invalid(
                "Loan lender is immutable after funding".into(),
            );
        }
        if previous.interest_rate != updated.interest_rate {
            return ValidateCallbackResult::Invalid(
                "Loan interest rate is immutable after funding".into(),
            );
        }
        if previous.funded != updated.funded || previous.maturity != updated.maturity {
            return ValidateCallbackResult::Invalid(
                "Loan funding timestamps are immutable after funding".into(),
            );
        }
    }

    match updated.status {
        LoanStatus::Cancelled => {
            if actor_did != previous.borrower_did {
                return ValidateCallbackResult::Invalid(
                    "Only the borrower may cancel a legacy loan request".into(),
                );
            }
            if updated.repaid != previous.repaid {
                return ValidateCallbackResult::Invalid(
                    "Cancellation cannot alter repayment evidence".into(),
                );
            }
        }
        LoanStatus::Defaulted => {
            if previous.lender_did.is_empty() || actor_did != previous.lender_did {
                return ValidateCallbackResult::Invalid(
                    "Only the lender may mark an active legacy loan defaulted".into(),
                );
            }
            if updated.repaid != previous.repaid {
                return ValidateCallbackResult::Invalid(
                    "Default cannot alter repayment evidence".into(),
                );
            }
        }
        LoanStatus::Repaid => {
            let actor_is_participant = actor_did == previous.borrower_did
                || (!previous.lender_did.is_empty() && actor_did == previous.lender_did);
            if !actor_is_participant {
                return ValidateCallbackResult::Invalid(
                    "Only a loan participant may record legacy repayment".into(),
                );
            }
            if updated.repaid.is_none() {
                return ValidateCallbackResult::Invalid(
                    "Repaid loans must record a repayment timestamp".into(),
                );
            }
            if let (Some(funded), Some(repaid)) = (updated.funded, updated.repaid) {
                if repaid < funded {
                    return ValidateCallbackResult::Invalid(
                        "Repayment timestamp cannot precede funding".into(),
                    );
                }
            }
        }
        LoanStatus::Active => {
            let actor_is_participant = actor_did == previous.borrower_did
                || (!previous.lender_did.is_empty() && actor_did == previous.lender_did);
            if !actor_is_participant {
                return ValidateCallbackResult::Invalid(
                    "Only a loan participant may activate a funded legacy loan".into(),
                );
            }
            if updated.repaid != previous.repaid {
                return ValidateCallbackResult::Invalid(
                    "Activation cannot alter repayment evidence".into(),
                );
            }
        }
        LoanStatus::Funded => {
            // Funding authority and timestamp checks are handled above.
        }
        LoanStatus::Requested | LoanStatus::Offered => {
            return ValidateCallbackResult::Invalid(
                "Legacy loan updates cannot move back into pre-funding states".into(),
            );
        }
    }

    ValidateCallbackResult::Valid
}

fn validate_update_loan(action: Update, loan: Loan) -> ExternResult<ValidateCallbackResult> {
    let previous_record = must_get_valid_record(action.original_action_address.clone())?;
    let previous_loan = match previous_record.entry().to_app_option::<Loan>() {
        Ok(Some(previous)) => previous,
        _ => {
            return Ok(ValidateCallbackResult::Invalid(
                "Loan update must reference a valid predecessor Loan entry".into(),
            ))
        }
    };

    let actor_did = did_for_author(&action.author);
    Ok(validate_loan_update_against_previous(
        &actor_did,
        &previous_loan,
        &loan,
    ))
}

fn validate_create_loan_offer(
    action: EntryCreationAction,
    offer: LoanOffer,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the offer to its committer. `create_loan_offer`
    // (lending/coordinator/src/lib.rs:105) takes `input.lender_did` from the client
    // and never checks the caller, so before this any agent could publish a loan
    // offer in someone else's name (MYCELIX_AUTHOR_BINDING_TRIAGE_2026-07-09.md,
    // finance Class-A, `lending:234`).
    //
    // Safe to bind: exactly one coordinator creation path, no on-behalf-of flow
    // (verified 2026-07-28).
    let author_did = did_for_author(action.author());
    if let ValidateCallbackResult::Invalid(msg) =
        require_did_is_author("LoanOffer", "lender_did", &offer.lender_did, &author_did)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    if !offer.lender_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Lender must be a valid DID".into(),
        ));
    }
    if !offer.min_amount.is_finite()
        || !offer.max_amount.is_finite()
        || offer.min_amount <= 0.0
        || offer.max_amount <= 0.0
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Offer amounts must be finite and positive".into(),
        ));
    }
    if offer.min_amount > offer.max_amount {
        return Ok(ValidateCallbackResult::Invalid(
            "Min amount cannot exceed max amount".into(),
        ));
    }
    if !offer.base_interest_rate.is_finite()
        || !(0.0..=1.0).contains(&offer.base_interest_rate)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Interest rate must be finite and between 0 and 100%".into(),
        ));
    }
    if !offer.min_credit_score.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Minimum credit score must be finite".into(),
        ));
    }
    if offer.max_term_days == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Maximum term must be at least one day".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_loan_offer(
    action: Update,
    offer: LoanOffer,
) -> ExternResult<ValidateCallbackResult> {
    // Bind updates too, so a bound create cannot simply be overwritten by another
    // agent. Safe: the only coordinator update path (`deactivate_offer`:415)
    // locates the offer with `query()`, which reads the CALLER'S OWN source chain
    // only — it can already only touch offers the caller authored.
    //
    // NOTE: `Update.author` is a field; `EntryCreationAction::author()` is a
    // method. Hence the asymmetry with the create validator above.
    let author_did = did_for_author(&action.author);
    if let ValidateCallbackResult::Invalid(msg) =
        require_did_is_author("LoanOffer", "lender_did", &offer.lender_did, &author_did)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    if !offer.min_amount.is_finite()
        || !offer.max_amount.is_finite()
        || offer.min_amount <= 0.0
        || offer.max_amount <= 0.0
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Offer amounts must be finite and positive".into(),
        ));
    }
    if offer.min_amount > offer.max_amount {
        return Ok(ValidateCallbackResult::Invalid(
            "Min amount cannot exceed max amount".into(),
        ));
    }
    if !offer.base_interest_rate.is_finite()
        || !(0.0..=1.0).contains(&offer.base_interest_rate)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Interest rate must be finite and between 0 and 100%".into(),
        ));
    }
    if !offer.min_credit_score.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Minimum credit score must be finite".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_payment_schedule(
    _action: EntryCreationAction,
    schedule: PaymentSchedule,
) -> ExternResult<ValidateCallbackResult> {
    if schedule.payments.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Schedule must have at least one payment".into(),
        ));
    }
    for payment in &schedule.payments {
        if !payment.principal_amount.is_finite()
            || !payment.interest_amount.is_finite()
            || !payment.total_amount.is_finite()
            || payment.principal_amount < 0.0
            || payment.interest_amount < 0.0
            || payment.total_amount < 0.0
        {
            return Ok(ValidateCallbackResult::Invalid(
                "Scheduled payment amounts must be finite and non-negative".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ts(micros: i64) -> Timestamp {
        Timestamp::from_micros(micros)
    }

    fn make_create() -> Create {
        Create {
            author: AgentPubKey::from_raw_36(vec![0; 36]),
            timestamp: ts(1_000_000),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::CapClaim,
            entry_hash: EntryHash::from_raw_36(vec![0; 36]),
            weight: Default::default(),
        }
    }

    fn make_update() -> Update {
        Update {
            author: AgentPubKey::from_raw_36(vec![0; 36]),
            timestamp: ts(1_000_000),
            action_seq: 1,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            original_action_address: ActionHash::from_raw_36(vec![0; 36]),
            original_entry_address: EntryHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::CapClaim,
            entry_hash: EntryHash::from_raw_36(vec![0; 36]),
            weight: Default::default(),
        }
    }

    /// DID of the agent `make_create()`/`make_update()` attribute actions to.
    fn test_author_did() -> String {
        format!("did:mycelix:{}", AgentPubKey::from_raw_36(vec![0; 36]))
    }

    fn valid_requested_loan() -> Loan {
        Loan {
            id: "loan:test:001".into(),
            borrower_did: test_author_did(),
            lender_did: String::new(),
            principal: 1000.0,
            currency: "SAP".into(),
            interest_rate: 0.0,
            term_days: 30,
            collateral_ids: vec!["collateral:test:001".into()],
            status: LoanStatus::Requested,
            created: ts(1_000_000),
            funded: None,
            maturity: None,
            repaid: None,
        }
    }

    fn active_loan() -> Loan {
        Loan {
            lender_did: "did:mycelix:lender".into(),
            interest_rate: 0.05,
            status: LoanStatus::Active,
            funded: Some(ts(2_000_000)),
            maturity: Some(ts(20_000_000)),
            ..valid_requested_loan()
        }
    }

    fn valid_offer() -> LoanOffer {
        LoanOffer {
            id: "offer:test:001".into(),
            lender_did: test_author_did(),
            max_amount: 1000.0,
            min_amount: 100.0,
            currency: "SAP".into(),
            base_interest_rate: 0.05,
            min_credit_score: 500.0,
            max_term_days: 90,
            collateral_required: false,
            active: true,
            created: ts(1_000_000),
        }
    }

    #[test]
    fn test_requested_loan_from_committing_borrower_is_accepted() {
        let result = validate_create_loan(
            EntryCreationAction::Create(make_create()),
            valid_requested_loan(),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_requested_loan_forged_for_another_borrower_is_rejected() {
        let mut forged = valid_requested_loan();
        forged.borrower_did = "did:mycelix:uhCAkSomeoneElse".into();
        let result =
            validate_create_loan(EntryCreationAction::Create(make_create()), forged).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_non_finite_legacy_money_is_rejected() {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut loan = valid_requested_loan();
            loan.principal = bad;
            let result =
                validate_create_loan(EntryCreationAction::Create(make_create()), loan).unwrap();
            assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
        }
    }

    #[test]
    fn test_loan_principal_cannot_change_on_update() {
        let previous = active_loan();
        let mut updated = previous.clone();
        updated.status = LoanStatus::Repaid;
        updated.repaid = Some(ts(3_000_000));
        updated.principal += 1.0;
        let result = validate_loan_update_against_previous(
            &previous.borrower_did,
            &previous,
            &updated,
        );
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_only_lender_can_default_active_loan() {
        let previous = active_loan();
        let mut updated = previous.clone();
        updated.status = LoanStatus::Defaulted;

        let rejected = validate_loan_update_against_previous(
            &previous.borrower_did,
            &previous,
            &updated,
        );
        assert!(matches!(rejected, ValidateCallbackResult::Invalid(_)));

        let accepted = validate_loan_update_against_previous(
            &previous.lender_did,
            &previous,
            &updated,
        );
        assert!(matches!(accepted, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_only_borrower_can_cancel_requested_loan() {
        let previous = valid_requested_loan();
        let mut updated = previous.clone();
        updated.status = LoanStatus::Cancelled;

        let rejected = validate_loan_update_against_previous(
            "did:mycelix:someone-else",
            &previous,
            &updated,
        );
        assert!(matches!(rejected, ValidateCallbackResult::Invalid(_)));

        let accepted = validate_loan_update_against_previous(
            &previous.borrower_did,
            &previous,
            &updated,
        );
        assert!(matches!(accepted, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_funding_binds_new_lender_to_update_author() {
        let previous = valid_requested_loan();
        let mut updated = previous.clone();
        updated.status = LoanStatus::Funded;
        updated.lender_did = "did:mycelix:lender".into();
        updated.interest_rate = 0.05;
        updated.funded = Some(ts(2_000_000));
        updated.maturity = Some(ts(20_000_000));

        let rejected = validate_loan_update_against_previous(
            "did:mycelix:someone-else",
            &previous,
            &updated,
        );
        assert!(matches!(rejected, ValidateCallbackResult::Invalid(_)));

        let accepted = validate_loan_update_against_previous(
            &updated.lender_did,
            &previous,
            &updated,
        );
        assert!(matches!(accepted, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_terminal_loan_states_cannot_be_resurrected() {
        for terminal in [
            LoanStatus::Repaid,
            LoanStatus::Defaulted,
            LoanStatus::Cancelled,
        ] {
            let mut previous = active_loan();
            previous.status = terminal;
            let mut updated = previous.clone();
            updated.status = LoanStatus::Active;
            let result = validate_loan_update_against_previous(
                &previous.borrower_did,
                &previous,
                &updated,
            );
            assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
        }
    }

    #[test]
    fn test_offer_from_the_committing_agent_is_accepted() {
        let result =
            validate_create_loan_offer(EntryCreationAction::Create(make_create()), valid_offer())
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_offer_forged_for_another_lender_is_rejected() {
        let mut forged = valid_offer();
        forged.lender_did = "did:mycelix:uhCAkSomeoneElse".into();
        let result =
            validate_create_loan_offer(EntryCreationAction::Create(make_create()), forged).unwrap();
        match result {
            ValidateCallbackResult::Invalid(msg) => {
                assert!(
                    msg.contains("LoanOffer")
                        && msg.contains("lender_did")
                        && msg.contains("forgery"),
                    "got: {msg}"
                )
            }
            other => panic!("forged lender_did must be rejected, got {other:?}"),
        }
    }

    #[test]
    fn test_update_own_offer_is_accepted() {
        let result = validate_update_loan_offer(make_update(), valid_offer()).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_update_another_lenders_offer_is_rejected() {
        let mut forged = valid_offer();
        forged.lender_did = "did:mycelix:uhCAkSomeoneElse".into();
        let result = validate_update_loan_offer(make_update(), forged).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
