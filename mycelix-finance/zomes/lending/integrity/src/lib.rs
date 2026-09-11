// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Lending Integrity Zome
//! Updated to use HDI 0.7 patterns with FlatOp validation
//!
//! This is a deprecated compatibility surface. Validation is intentionally
//! conservative: preserve existing commitments, but do not let legacy lending
//! become a second source of financial truth.
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
            // Validate hash lengths (39 bytes for Holochain hashes)
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

fn validate_loan_numbers(loan: &Loan) -> Result<(), String> {
    if !loan.principal.is_finite() || loan.principal <= 0.0 {
        return Err("Principal must be finite and positive".into());
    }
    if !loan.interest_rate.is_finite() || !(0.0..=1.0).contains(&loan.interest_rate) {
        return Err("Interest rate must be finite and between 0 and 100%".into());
    }
    if loan.term_days == 0 {
        return Err("Term must be at least one day".into());
    }
    Ok(())
}

fn require_role(role: &str, expected_did: &str, author_did: &str) -> Result<(), String> {
    if expected_did == author_did {
        Ok(())
    } else {
        Err(format!(
            "Legacy loan {role} transition requires {expected_did}; action author was {author_did}"
        ))
    }
}

fn immutable_loan_identity_changed(old: &Loan, new: &Loan) -> bool {
    old.id != new.id
        || old.borrower_did != new.borrower_did
        || old.principal != new.principal
        || old.currency != new.currency
        || old.term_days != new.term_days
        || old.collateral_ids != new.collateral_ids
        || old.created != new.created
}

fn funded_terms_changed(old: &Loan, new: &Loan) -> bool {
    old.lender_did != new.lender_did
        || old.interest_rate != new.interest_rate
        || old.funded != new.funded
        || old.maturity != new.maturity
}

/// Pure legacy state-machine theorem. The coordinator is not authority for these
/// transitions; DHT validation must reject forged parties, rewritten economics,
/// impossible transitions, and lifecycle timestamps that do not match the state.
fn validate_loan_transition(old: &Loan, new: &Loan, author_did: &str) -> Result<(), String> {
    validate_loan_numbers(new)?;

    if immutable_loan_identity_changed(old, new) {
        return Err(
            "Legacy loan identity/economic terms (id, borrower, principal, currency, term, collateral, created) are immutable"
                .into(),
        );
    }

    match (&old.status, &new.status) {
        (LoanStatus::Requested, LoanStatus::Funded) => {
            if !old.lender_did.is_empty()
                || old.interest_rate != 0.0
                || old.funded.is_some()
                || old.maturity.is_some()
                || old.repaid.is_some()
            {
                return Err("Requested predecessor contains funded-state data".into());
            }
            if !new.lender_did.starts_with("did:") {
                return Err("Funded loan must bind a valid lender DID".into());
            }
            require_role("fund", &new.lender_did, author_did)?;
            let funded = new
                .funded
                .ok_or_else(|| "Funded transition requires funded timestamp".to_string())?;
            let maturity = new
                .maturity
                .ok_or_else(|| "Funded transition requires maturity timestamp".to_string())?;
            if maturity <= funded {
                return Err("Loan maturity must be after funding".into());
            }
            if new.repaid.is_some() {
                return Err("Newly funded loan cannot already be repaid".into());
            }
        }
        (LoanStatus::Requested, LoanStatus::Cancelled) => {
            require_role("cancel", &old.borrower_did, author_did)?;
            if old.lender_did != new.lender_did
                || old.interest_rate != new.interest_rate
                || old.funded != new.funded
                || old.maturity != new.maturity
                || old.repaid != new.repaid
            {
                return Err("Cancellation cannot rewrite loan terms or timestamps".into());
            }
        }
        (LoanStatus::Funded, LoanStatus::Active) => {
            require_role("activate", &old.lender_did, author_did)?;
            if funded_terms_changed(old, new) || old.repaid != new.repaid {
                return Err("Activation cannot rewrite funded loan terms".into());
            }
        }
        (LoanStatus::Funded, LoanStatus::Repaid)
        | (LoanStatus::Active, LoanStatus::Repaid) => {
            require_role("repay acknowledgement", &old.lender_did, author_did)?;
            if funded_terms_changed(old, new) {
                return Err("Repayment cannot rewrite funded loan terms".into());
            }
            if old.repaid.is_some() || new.repaid.is_none() {
                return Err("Repaid transition requires a newly recorded repayment timestamp".into());
            }
        }
        (LoanStatus::Active, LoanStatus::Defaulted) => {
            require_role("default", &old.lender_did, author_did)?;
            if funded_terms_changed(old, new) || old.repaid != new.repaid {
                return Err("Default cannot rewrite funded loan terms or repayment evidence".into());
            }
        }
        _ => {
            return Err(format!(
                "Invalid legacy loan status transition: {:?} -> {:?}",
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
    let author_did = did_for_author(action.author());
    if let ValidateCallbackResult::Invalid(msg) =
        require_did_is_author("Loan", "borrower_did", &loan.borrower_did, &author_did)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    if !loan.borrower_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Borrower must be a valid DID".into(),
        ));
    }
    if let Err(msg) = validate_loan_numbers(&loan) {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }
    if loan.status != LoanStatus::Requested {
        return Ok(ValidateCallbackResult::Invalid(
            "Legacy loans must be created in Requested state".into(),
        ));
    }
    if !loan.lender_did.is_empty() || loan.interest_rate != 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Requested loan cannot pre-bind lender or interest terms".into(),
        ));
    }
    if loan.funded.is_some() || loan.maturity.is_some() || loan.repaid.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "Requested loan cannot contain lifecycle completion timestamps".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_loan(action: Update, loan: Loan) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original_loan = match original_record.entry().to_app_option::<Loan>()? {
        Some(original) => original,
        None => {
            return Ok(ValidateCallbackResult::Invalid(
                "Loan update predecessor must contain a Loan entry".into(),
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
            "Offer amounts must be finite, positive, and ordered".into(),
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
            || payment.total_amount <= 0.0
        {
            return Ok(ValidateCallbackResult::Invalid(
                "Scheduled payment amounts must be finite and non-negative, with positive total"
                    .into(),
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

    fn valid_requested_loan() -> Loan {
        Loan {
            id: "loan:test:001".into(),
            borrower_did: test_author_did(),
            lender_did: String::new(),
            principal: 1_000.0,
            currency: "SAP".into(),
            interest_rate: 0.0,
            term_days: 90,
            collateral_ids: vec!["collateral:1".into()],
            status: LoanStatus::Requested,
            created: ts(1_000_000),
            funded: None,
            maturity: None,
            repaid: None,
        }
    }

    fn funded_from(requested: &Loan, lender: &str) -> Loan {
        Loan {
            lender_did: lender.into(),
            interest_rate: 0.05,
            status: LoanStatus::Funded,
            funded: Some(ts(2_000_000)),
            maturity: Some(ts(3_000_000)),
            ..requested.clone()
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
    fn requested_loan_from_borrower_is_accepted() {
        let result = validate_create_loan(
            EntryCreationAction::Create(make_create()),
            valid_requested_loan(),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn requested_loan_forged_for_another_borrower_is_rejected() {
        let mut forged = valid_requested_loan();
        forged.borrower_did = "did:mycelix:someone-else".into();
        let result = validate_create_loan(EntryCreationAction::Create(make_create()), forged).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn requested_loan_rejects_non_finite_principal() {
        let mut loan = valid_requested_loan();
        loan.principal = f64::NAN;
        let result = validate_create_loan(EntryCreationAction::Create(make_create()), loan).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn lender_can_fund_requested_loan() {
        let old = valid_requested_loan();
        let lender = "did:mycelix:lender";
        let new = funded_from(&old, lender);
        assert!(validate_loan_transition(&old, &new, lender).is_ok());
    }

    #[test]
    fn third_party_cannot_fund_for_lender() {
        let old = valid_requested_loan();
        let new = funded_from(&old, "did:mycelix:lender");
        assert!(validate_loan_transition(&old, &new, "did:mycelix:attacker").is_err());
    }

    #[test]
    fn borrower_can_cancel_only_requested_loan() {
        let old = valid_requested_loan();
        let mut cancelled = old.clone();
        cancelled.status = LoanStatus::Cancelled;
        assert!(validate_loan_transition(&old, &cancelled, &old.borrower_did).is_ok());
        assert!(validate_loan_transition(&old, &cancelled, "did:mycelix:attacker").is_err());
    }

    #[test]
    fn transition_cannot_rewrite_principal_or_parties() {
        let old = valid_requested_loan();
        let lender = "did:mycelix:lender";
        let mut funded = funded_from(&old, lender);
        funded.principal = old.principal + 1.0;
        assert!(validate_loan_transition(&old, &funded, lender).is_err());

        let funded = funded_from(&old, lender);
        let mut active = funded.clone();
        active.status = LoanStatus::Active;
        active.lender_did = "did:mycelix:replacement".into();
        assert!(validate_loan_transition(&funded, &active, lender).is_err());
    }

    #[test]
    fn lender_controls_default_and_repayment_acknowledgement() {
        let requested = valid_requested_loan();
        let lender = "did:mycelix:lender";
        let funded = funded_from(&requested, lender);
        let mut active = funded.clone();
        active.status = LoanStatus::Active;
        assert!(validate_loan_transition(&funded, &active, lender).is_ok());

        let mut defaulted = active.clone();
        defaulted.status = LoanStatus::Defaulted;
        assert!(validate_loan_transition(&active, &defaulted, lender).is_ok());
        assert!(validate_loan_transition(&active, &defaulted, &requested.borrower_did).is_err());

        let mut repaid = funded.clone();
        repaid.status = LoanStatus::Repaid;
        repaid.repaid = Some(ts(2_500_000));
        assert!(validate_loan_transition(&funded, &repaid, lender).is_ok());
        assert!(validate_loan_transition(&funded, &repaid, &requested.borrower_did).is_err());
    }

    #[test]
    fn invalid_status_shortcuts_are_rejected() {
        let old = valid_requested_loan();
        let mut defaulted = old.clone();
        defaulted.status = LoanStatus::Defaulted;
        assert!(validate_loan_transition(&old, &defaulted, &old.borrower_did).is_err());
    }

    #[test]
    fn offer_from_the_committing_agent_is_accepted() {
        let result =
            validate_create_loan_offer(EntryCreationAction::Create(make_create()), valid_offer())
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn offer_forged_for_another_lender_is_rejected() {
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
    fn offer_rejects_non_finite_values() {
        let mut offer = valid_offer();
        offer.max_amount = f64::INFINITY;
        let result =
            validate_create_loan_offer(EntryCreationAction::Create(make_create()), offer).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_own_offer_is_accepted() {
        let result = validate_update_loan_offer(make_update(), valid_offer()).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn update_another_lenders_offer_is_rejected() {
        let mut forged = valid_offer();
        forged.lender_did = "did:mycelix:uhCAkSomeoneElse".into();
        let result = validate_update_loan_offer(make_update(), forged).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
