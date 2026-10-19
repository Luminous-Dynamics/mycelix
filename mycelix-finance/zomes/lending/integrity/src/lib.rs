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

/// Main validation callback using FlatOp pattern
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Loan(loan) => {
                    validate_create_loan(EntryCreationAction::Create(action), loan)
                }
                EntryTypes::LoanOffer(offer) => {
                    validate_create_loan_offer(EntryCreationAction::Create(action), offer)
                }
                EntryTypes::PaymentSchedule(schedule) => {
                    validate_create_payment_schedule(EntryCreationAction::Create(action), schedule)
                }
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::Loan(loan) => validate_update_loan(action, loan),
                EntryTypes::LoanOffer(offer) => validate_update_loan_offer(action, offer),
                EntryTypes::PaymentSchedule(_) => Ok(ValidateCallbackResult::Invalid(
                    "Payment schedules cannot be updated".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address,
            target_address,
            ..
        } => {
            let base_valid = base_address.as_ref().len() == 39;
            let target_valid = target_address.as_ref().len() == 39;

            match link_type {
                LinkTypes::BorrowerToLoans
                | LinkTypes::LenderToLoans
                | LinkTypes::LenderToOffers => {
                    if !base_valid || !target_valid {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Link must connect valid agent and entry hashes".into(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
                LinkTypes::LoanToSchedule => {
                    if !base_valid || !target_valid {
                        return Ok(ValidateCallbackResult::Invalid(
                            "LoanToSchedule must connect valid entry hashes".into(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
                LinkTypes::CollateralToLoan => {
                    if !base_valid || !target_valid {
                        return Ok(ValidateCallbackResult::Invalid(
                            "CollateralToLoan must connect valid entry hashes".into(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
                LinkTypes::ActiveOffers => {
                    if !target_valid {
                        return Ok(ValidateCallbackResult::Invalid(
                            "ActiveOffers target must be a valid entry hash".into(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
            }
        }
        FlatOp::RegisterDeleteLink { link_type, .. } => match link_type {
            LinkTypes::LoanToSchedule => Ok(ValidateCallbackResult::Invalid(
                "LoanToSchedule links cannot be deleted - payment schedules are immutable".into(),
            )),
            LinkTypes::CollateralToLoan => Ok(ValidateCallbackResult::Invalid(
                "CollateralToLoan links cannot be deleted - collateral is locked".into(),
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_loan_numeric_fields(loan: &Loan) -> Result<(), String> {
    if !loan.principal.is_finite() || loan.principal <= 0.0 {
        return Err("Principal must be finite and positive".into());
    }
    if !loan.interest_rate.is_finite() || !(0.0..=1.0).contains(&loan.interest_rate) {
        return Err("Interest rate must be finite and between 0 and 100%".into());
    }
    Ok(())
}

fn require_author(role: &str, field: &str, expected_did: &str, author_did: &str) -> Result<(), String> {
    match require_did_is_author(role, field, expected_did, author_did) {
        ValidateCallbackResult::Valid => Ok(()),
        ValidateCallbackResult::Invalid(msg) => Err(msg),
        other => Err(format!("Unexpected author-binding result: {other:?}")),
    }
}

/// Pure theorem for the frozen legacy loan state machine.
///
/// This does not add new lending semantics. It only makes the existing mutation
/// surface explicit and fail-closed so direct DHT commits cannot bypass the
/// coordinator's intended borrower/lender authority.
fn validate_loan_transition(old: &Loan, new: &Loan, author_did: &str) -> Result<(), String> {
    validate_loan_numeric_fields(new)?;

    if old.id != new.id {
        return Err("Loan id is immutable".into());
    }
    if old.borrower_did != new.borrower_did {
        return Err("Borrower is immutable".into());
    }
    if old.principal != new.principal {
        return Err("Principal is immutable".into());
    }
    if old.currency != new.currency {
        return Err("Currency is immutable".into());
    }
    if old.term_days != new.term_days {
        return Err("Term is immutable".into());
    }
    if old.collateral_ids != new.collateral_ids {
        return Err("Collateral set is immutable in the legacy loan model".into());
    }
    if old.created != new.created {
        return Err("Loan creation timestamp is immutable".into());
    }

    match (&old.status, &new.status) {
        (LoanStatus::Requested, LoanStatus::Funded)
        | (LoanStatus::Offered, LoanStatus::Funded) => {
            if !old.lender_did.is_empty() && old.lender_did != new.lender_did {
                return Err("Existing lender cannot be replaced".into());
            }
            if !new.lender_did.starts_with("did:") {
                return Err("Funded loan must bind a valid lender DID".into());
            }
            require_author("LoanFunding", "lender_did", &new.lender_did, author_did)?;
            if new.funded.is_none() || new.maturity.is_none() {
                return Err("Funding must set funded and maturity timestamps".into());
            }
            if new.repaid.is_some() {
                return Err("A newly funded loan cannot already be repaid".into());
            }
        }
        (LoanStatus::Funded, LoanStatus::Active) => {
            if old.lender_did != new.lender_did || old.interest_rate != new.interest_rate {
                return Err("Lender and interest rate are immutable after funding".into());
            }
            if old.funded != new.funded || old.maturity != new.maturity || new.repaid.is_some() {
                return Err("Funding timestamps are immutable after funding".into());
            }
            require_author("LoanActivation", "lender_did", &new.lender_did, author_did)?;
        }
        (LoanStatus::Funded, LoanStatus::Repaid)
        | (LoanStatus::Active, LoanStatus::Repaid) => {
            if old.lender_did != new.lender_did || old.interest_rate != new.interest_rate {
                return Err("Lender and interest rate are immutable after funding".into());
            }
            if old.funded != new.funded || old.maturity != new.maturity {
                return Err("Funding timestamps are immutable after funding".into());
            }
            if new.repaid.is_none() {
                return Err("Repaid transition must set repaid timestamp".into());
            }
            require_author("LoanRepayment", "borrower_did", &new.borrower_did, author_did)?;
        }
        (LoanStatus::Active, LoanStatus::Defaulted) => {
            if old.lender_did != new.lender_did || old.interest_rate != new.interest_rate {
                return Err("Lender and interest rate are immutable after funding".into());
            }
            if old.funded != new.funded || old.maturity != new.maturity || old.repaid != new.repaid {
                return Err("Default cannot rewrite funding or repayment timestamps".into());
            }
            require_author("LoanDefault", "lender_did", &new.lender_did, author_did)?;
        }
        (LoanStatus::Requested, LoanStatus::Cancelled) => {
            if old.lender_did != new.lender_did
                || old.interest_rate != new.interest_rate
                || old.funded != new.funded
                || old.maturity != new.maturity
                || old.repaid != new.repaid
            {
                return Err("Cancellation may only change loan status".into());
            }
            require_author("LoanCancellation", "borrower_did", &new.borrower_did, author_did)?;
        }
        _ => {
            return Err(format!(
                "Invalid legacy loan transition: {:?} -> {:?}",
                old.status, new.status
            ));
        }
    }

    Ok(())
}

fn validate_create_loan(
    action: EntryCreationAction,
    loan: Loan,
) -> ExternResult<ValidateCallbackResult> {
    if !loan.borrower_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Borrower must be a valid DID".into(),
        ));
    }

    let author_did = did_for_author(action.author());
    if let ValidateCallbackResult::Invalid(msg) =
        require_did_is_author("Loan", "borrower_did", &loan.borrower_did, &author_did)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    if let Err(msg) = validate_loan_numeric_fields(&loan) {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }
    if loan.borrower_did == loan.lender_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot lend to yourself".into(),
        ));
    }
    if loan.status != LoanStatus::Requested {
        return Ok(ValidateCallbackResult::Invalid(
            "Legacy loans must be created in Requested state".into(),
        ));
    }
    if !loan.lender_did.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Requested loan must not pre-bind a lender".into(),
        ));
    }
    if loan.interest_rate != 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Requested loan must start with zero interest until funding".into(),
        ));
    }
    if loan.funded.is_some() || loan.maturity.is_some() || loan.repaid.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "Requested loan cannot contain funding, maturity, or repayment timestamps".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_loan(action: Update, loan: Loan) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original_loan = match original_record.entry().to_app_option::<Loan>()? {
        Some(original_loan) => original_loan,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "Loan update must reference a valid prior Loan entry".into(),
            ));
        }
    };

    let author_did = did_for_author(&action.author);
    match validate_loan_transition(&original_loan, &loan, &author_did) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(msg) => Ok(ValidateCallbackResult::Invalid(msg)),
    }
}

fn validate_create_loan_offer(
    action: EntryCreationAction,
    offer: LoanOffer,
) -> ExternResult<ValidateCallbackResult> {
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
        || offer.min_amount > offer.max_amount
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Offer amounts must be finite, positive, and min <= max".into(),
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

fn validate_update_loan_offer(
    action: Update,
    offer: LoanOffer,
) -> ExternResult<ValidateCallbackResult> {
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
        || offer.min_amount > offer.max_amount
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Offer amounts must be finite, positive, and min <= max".into(),
        ));
    }
    if !offer.base_interest_rate.is_finite()
        || !(0.0..=1.0).contains(&offer.base_interest_rate)
        || !offer.min_credit_score.is_finite()
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Offer rate and credit score must be finite and valid".into(),
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
                "Payment schedule amounts must be finite and non-negative".into(),
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

    fn test_author_did() -> String {
        format!("did:mycelix:{}", AgentPubKey::from_raw_36(vec![0; 36]))
    }

    fn borrower_loan() -> Loan {
        Loan {
            id: "loan:test:001".into(),
            borrower_did: test_author_did(),
            lender_did: String::new(),
            principal: 1_000.0,
            currency: "SAP".into(),
            interest_rate: 0.0,
            term_days: 30,
            collateral_ids: vec!["collateral:1".into()],
            status: LoanStatus::Requested,
            created: ts(1_000_000),
            funded: None,
            maturity: None,
            repaid: None,
        }
    }

    fn funded_loan(lender_did: String) -> Loan {
        Loan {
            lender_did,
            interest_rate: 0.05,
            status: LoanStatus::Funded,
            funded: Some(ts(2_000_000)),
            maturity: Some(ts(3_000_000)),
            ..borrower_loan()
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
    fn test_requested_loan_from_borrower_is_accepted() {
        let result = validate_create_loan(
            EntryCreationAction::Create(make_create()),
            borrower_loan(),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_requested_loan_forged_for_another_borrower_is_rejected() {
        let mut loan = borrower_loan();
        loan.borrower_did = "did:mycelix:someone-else".into();
        let result = validate_create_loan(EntryCreationAction::Create(make_create()), loan).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_requested_loan_rejects_non_finite_money_like_values() {
        let mut loan = borrower_loan();
        loan.principal = f64::NAN;
        assert!(matches!(
            validate_create_loan(EntryCreationAction::Create(make_create()), loan).unwrap(),
            ValidateCallbackResult::Invalid(_)
        ));

        let mut loan = borrower_loan();
        loan.interest_rate = f64::INFINITY;
        assert!(matches!(
            validate_create_loan(EntryCreationAction::Create(make_create()), loan).unwrap(),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn test_transition_rejects_principal_or_party_mutation() {
        let old = funded_loan("did:mycelix:lender".into());
        let mut new = old.clone();
        new.status = LoanStatus::Active;
        new.principal += 1.0;
        assert!(validate_loan_transition(&old, &new, "did:mycelix:lender").is_err());

        let mut new = old.clone();
        new.status = LoanStatus::Active;
        new.borrower_did = "did:mycelix:other".into();
        assert!(validate_loan_transition(&old, &new, "did:mycelix:lender").is_err());
    }

    #[test]
    fn test_default_is_lender_only() {
        let mut old = funded_loan("did:mycelix:lender".into());
        old.status = LoanStatus::Active;
        let mut new = old.clone();
        new.status = LoanStatus::Defaulted;
        assert!(validate_loan_transition(&old, &new, "did:mycelix:lender").is_ok());
        assert!(validate_loan_transition(&old, &new, &old.borrower_did).is_err());
    }

    #[test]
    fn test_cancel_is_borrower_only() {
        let old = borrower_loan();
        let mut new = old.clone();
        new.status = LoanStatus::Cancelled;
        assert!(validate_loan_transition(&old, &new, &old.borrower_did).is_ok());
        assert!(validate_loan_transition(&old, &new, "did:mycelix:not-borrower").is_err());
    }

    #[test]
    fn test_invalid_transition_is_rejected() {
        let old = borrower_loan();
        let mut new = old.clone();
        new.status = LoanStatus::Defaulted;
        assert!(validate_loan_transition(&old, &new, &old.borrower_did).is_err());
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
    fn test_offer_rejects_nan_amount_or_rate() {
        let mut offer = valid_offer();
        offer.max_amount = f64::NAN;
        assert!(matches!(
            validate_create_loan_offer(EntryCreationAction::Create(make_create()), offer).unwrap(),
            ValidateCallbackResult::Invalid(_)
        ));

        let mut offer = valid_offer();
        offer.base_interest_rate = f64::NAN;
        assert!(matches!(
            validate_create_loan_offer(EntryCreationAction::Create(make_create()), offer).unwrap(),
            ValidateCallbackResult::Invalid(_)
        ));
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
