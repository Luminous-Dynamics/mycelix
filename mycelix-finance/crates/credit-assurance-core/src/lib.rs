//! Pure loan lifecycle and repayment-assurance kernel.
//!
//! This crate deliberately has no Holochain, networking, bank, credit-bureau,
//! or machine-learning dependency. It encodes arithmetic and state transitions;
//! accountable institutions must authenticate evidence and enforce these rules
//! at their authoritative mutation boundary.

#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

pub const BASIS_POINTS: u128 = 10_000;
pub const ACTUAL_365_DAY_COUNT: u128 = 365;
pub const INTEREST_DENOMINATOR: u128 = BASIS_POINTS * ACTUAL_365_DAY_COUNT;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CreditError {
    MissingReference,
    InvalidCurrency,
    InvalidPrincipal,
    InvalidApr,
    InvalidMaturity,
    InvalidStateTransition,
    TimeMovedBackwards,
    InterestArithmeticOverflow,
    BalanceArithmeticOverflow,
    PaymentAmountInvalid,
    PaymentNotFinal,
    PaymentOutcomeUnknown,
    PaymentReceiptMismatch,
    PaymentDayNotAccrued,
    Overpayment,
    IdempotencyConflict,
    DuplicateExternalReceipt,
    LoanNotFound,
    LoanAlreadyExists,
    HardshipAgreementRequired,
    InvalidAffordabilityInput,
    InvalidRiskInput,
    InvalidRestructureTerms,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoanState {
    Draft,
    Offered,
    Accepted,
    Funded,
    Active,
    InArrears,
    Hardship,
    Forbearance,
    Restructured,
    Disputed,
    Defaulted,
    Settled,
    WrittenOff,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LoanPolicy {
    pub max_apr_bps: u32,
    pub max_term_days: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LoanTerms {
    pub loan_id: String,
    pub borrower_id: String,
    pub lender_id: String,
    pub currency: String,
    pub original_principal_minor: u128,
    pub apr_bps: u32,
    pub maturity_day: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterestTreatment {
    Contractual,
    PauseByAgreement,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InterestAccrualEvent {
    pub from_day: u64,
    pub to_day: u64,
    pub treatment: InterestTreatment,
    pub agreement_ref: Option<String>,
    pub accrued_minor: u128,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LoanEvent {
    pub event_type: String,
    pub evidence_ref: String,
    pub day: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Loan {
    terms: LoanTerms,
    policy: LoanPolicy,
    state: LoanState,
    principal_outstanding_minor: u128,
    accrued_interest_minor: u128,
    interest_fraction_remainder: u128,
    last_accrual_day: Option<u64>,
    disputed_from: Option<LoanState>,
    interest_events: Vec<InterestAccrualEvent>,
    events: Vec<LoanEvent>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RepaymentAllocation {
    pub loan_id: String,
    pub total_paid_minor: u128,
    pub interest_paid_minor: u128,
    pub principal_paid_minor: u128,
    pub principal_remaining_minor: u128,
    pub interest_remaining_minor: u128,
    pub resulting_state: LoanState,
    pub external_receipt_ref: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PaymentOutcome {
    PaidFinal {
        external_receipt_ref: String,
        finality_policy_ref: String,
    },
    Rejected {
        external_receipt_ref: String,
        reason_ref: String,
    },
    Indeterminate {
        reconciliation_ref: Option<String>,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RepaymentReceipt {
    pub loan_id: String,
    pub borrower_id: String,
    pub lender_id: String,
    pub currency: String,
    pub amount_minor: u128,
    pub payment_day: u64,
    pub idempotency_key: String,
    pub request_digest: [u8; 32],
    pub outcome: PaymentOutcome,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RepaymentDisposition {
    Applied(RepaymentAllocation),
    ExactReplay(RepaymentAllocation),
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ConsumedPayment {
    request_digest: [u8; 32],
    allocation: RepaymentAllocation,
}

#[derive(Debug, Clone, Default)]
pub struct CreditPortfolio {
    loans: BTreeMap<String, Loan>,
    consumed_idempotency: BTreeMap<String, ConsumedPayment>,
    consumed_external_receipts: BTreeSet<String>,
}

impl Loan {
    pub fn new(terms: LoanTerms, policy: LoanPolicy) -> Result<Self, CreditError> {
        if terms.loan_id.is_empty()
            || terms.borrower_id.is_empty()
            || terms.lender_id.is_empty()
        {
            return Err(CreditError::MissingReference);
        }
        if terms.borrower_id == terms.lender_id {
            return Err(CreditError::MissingReference);
        }
        if terms.currency.len() != 3
            || !terms.currency.bytes().all(|b| b.is_ascii_uppercase())
        {
            return Err(CreditError::InvalidCurrency);
        }
        if terms.original_principal_minor == 0 {
            return Err(CreditError::InvalidPrincipal);
        }
        if terms.apr_bps > policy.max_apr_bps {
            return Err(CreditError::InvalidApr);
        }
        if terms.maturity_day == 0 || policy.max_term_days == 0 {
            return Err(CreditError::InvalidMaturity);
        }

        Ok(Self {
            principal_outstanding_minor: terms.original_principal_minor,
            terms,
            policy,
            state: LoanState::Draft,
            accrued_interest_minor: 0,
            interest_fraction_remainder: 0,
            last_accrual_day: None,
            disputed_from: None,
            interest_events: Vec::new(),
            events: Vec::new(),
        })
    }

    pub fn state(&self) -> LoanState {
        self.state
    }

    pub fn loan_id(&self) -> &str {
        &self.terms.loan_id
    }

    pub fn borrower_id(&self) -> &str {
        &self.terms.borrower_id
    }

    pub fn lender_id(&self) -> &str {
        &self.terms.lender_id
    }

    pub fn currency(&self) -> &str {
        &self.terms.currency
    }

    pub fn principal_outstanding_minor(&self) -> u128 {
        self.principal_outstanding_minor
    }

    pub fn accrued_interest_minor(&self) -> u128 {
        self.accrued_interest_minor
    }

    pub fn last_accrual_day(&self) -> Option<u64> {
        self.last_accrual_day
    }

    pub fn interest_events(&self) -> &[InterestAccrualEvent] {
        &self.interest_events
    }

    pub fn events(&self) -> &[LoanEvent] {
        &self.events
    }

    pub fn apr_bps(&self) -> u32 {
        self.terms.apr_bps
    }

    pub fn offer(&mut self, offer_ref: &str, day: u64) -> Result<(), CreditError> {
        self.require_state(LoanState::Draft)?;
        self.push_event("offered", offer_ref, day)?;
        self.state = LoanState::Offered;
        Ok(())
    }

    pub fn accept(&mut self, acceptance_ref: &str, day: u64) -> Result<(), CreditError> {
        self.require_state(LoanState::Offered)?;
        self.push_event("accepted", acceptance_ref, day)?;
        self.state = LoanState::Accepted;
        Ok(())
    }

    pub fn fund(&mut self, funding_receipt_ref: &str, day: u64) -> Result<(), CreditError> {
        self.require_state(LoanState::Accepted)?;
        if funding_receipt_ref.is_empty() {
            return Err(CreditError::MissingReference);
        }
        if day >= self.terms.maturity_day
            || self.terms.maturity_day - day > self.policy.max_term_days
        {
            return Err(CreditError::InvalidMaturity);
        }
        self.push_event("funded", funding_receipt_ref, day)?;
        self.last_accrual_day = Some(day);
        self.state = LoanState::Funded;
        Ok(())
    }

    pub fn activate(&mut self, servicing_ref: &str, day: u64) -> Result<(), CreditError> {
        self.require_state(LoanState::Funded)?;
        self.push_event("activated", servicing_ref, day)?;
        self.state = LoanState::Active;
        Ok(())
    }

    pub fn mark_in_arrears(&mut self, arrears_ref: &str, day: u64) -> Result<(), CreditError> {
        self.require_state(LoanState::Active)?;
        self.push_event("arrears", arrears_ref, day)?;
        self.state = LoanState::InArrears;
        Ok(())
    }

    pub fn enter_hardship(&mut self, case_ref: &str, day: u64) -> Result<(), CreditError> {
        if !matches!(self.state, LoanState::Active | LoanState::InArrears) {
            return Err(CreditError::InvalidStateTransition);
        }
        self.push_event("hardship_opened", case_ref, day)?;
        self.state = LoanState::Hardship;
        Ok(())
    }

    pub fn grant_forbearance(
        &mut self,
        agreement_ref: &str,
        day: u64,
    ) -> Result<(), CreditError> {
        self.require_state(LoanState::Hardship)?;
        self.push_event("forbearance_agreed", agreement_ref, day)?;
        self.state = LoanState::Forbearance;
        Ok(())
    }

    /// Reprices and/or extends the loan under a new, evidenced agreement.
    /// It never silently changes principal or capitalizes accrued interest.
    pub fn restructure(
        &mut self,
        new_apr_bps: u32,
        new_maturity_day: u64,
        agreement_ref: &str,
        approval_ref: &str,
        day: u64,
    ) -> Result<(), CreditError> {
        if !matches!(
            self.state,
            LoanState::Hardship | LoanState::Forbearance | LoanState::InArrears
        ) {
            return Err(CreditError::InvalidStateTransition);
        }
        if agreement_ref.is_empty() || approval_ref.is_empty() {
            return Err(CreditError::MissingReference);
        }
        if new_apr_bps > self.policy.max_apr_bps
            || new_maturity_day <= day
            || new_maturity_day - day > self.policy.max_term_days
        {
            return Err(CreditError::InvalidRestructureTerms);
        }
        self.terms.apr_bps = new_apr_bps;
        self.terms.maturity_day = new_maturity_day;
        self.push_event(
            "restructured",
            &format!("{agreement_ref}:{approval_ref}"),
            day,
        )?;
        self.state = LoanState::Restructured;
        Ok(())
    }

    pub fn resume(&mut self, approval_ref: &str, day: u64) -> Result<(), CreditError> {
        if !matches!(
            self.state,
            LoanState::Hardship | LoanState::Forbearance | LoanState::Restructured
        ) {
            return Err(CreditError::InvalidStateTransition);
        }
        self.push_event("resumed", approval_ref, day)?;
        self.state = LoanState::Active;
        Ok(())
    }

    pub fn open_dispute(&mut self, dispute_ref: &str, day: u64) -> Result<(), CreditError> {
        if matches!(
            self.state,
            LoanState::Draft
                | LoanState::Settled
                | LoanState::Defaulted
                | LoanState::WrittenOff
                | LoanState::Disputed
        ) {
            return Err(CreditError::InvalidStateTransition);
        }
        self.push_event("disputed", dispute_ref, day)?;
        self.disputed_from = Some(self.state);
        self.state = LoanState::Disputed;
        Ok(())
    }

    pub fn resolve_dispute(
        &mut self,
        resolution_ref: &str,
        day: u64,
    ) -> Result<(), CreditError> {
        self.require_state(LoanState::Disputed)?;
        let prior = self
            .disputed_from
            .take()
            .ok_or(CreditError::InvalidStateTransition)?;
        self.push_event("dispute_resolved", resolution_ref, day)?;
        self.state = prior;
        Ok(())
    }

    pub fn declare_default(
        &mut self,
        evidence_ref: &str,
        authority_ref: &str,
        day: u64,
    ) -> Result<(), CreditError> {
        if !matches!(
            self.state,
            LoanState::Active
                | LoanState::InArrears
                | LoanState::Hardship
                | LoanState::Forbearance
                | LoanState::Restructured
        ) {
            return Err(CreditError::InvalidStateTransition);
        }
        if evidence_ref.is_empty() || authority_ref.is_empty() {
            return Err(CreditError::MissingReference);
        }
        self.push_event(
            "default_declared",
            &format!("{evidence_ref}:{authority_ref}"),
            day,
        )?;
        self.state = LoanState::Defaulted;
        Ok(())
    }

    /// Accounting write-off is a separately authorized action; it does not,
    /// by itself, forgive the borrower's legal obligation or erase the history.
    pub fn write_off(
        &mut self,
        accounting_ref: &str,
        authority_ref: &str,
        day: u64,
    ) -> Result<(), CreditError> {
        self.require_state(LoanState::Defaulted)?;
        if accounting_ref.is_empty() || authority_ref.is_empty() {
            return Err(CreditError::MissingReference);
        }
        self.push_event(
            "written_off",
            &format!("{accounting_ref}:{authority_ref}"),
            day,
        )?;
        self.state = LoanState::WrittenOff;
        Ok(())
    }

    pub fn accrue_interest_to_day(
        &mut self,
        day: u64,
        treatment: InterestTreatment,
        agreement_ref: Option<&str>,
    ) -> Result<u128, CreditError> {
        if !matches!(
            self.state,
            LoanState::Active
                | LoanState::InArrears
                | LoanState::Hardship
                | LoanState::Forbearance
                | LoanState::Restructured
        ) {
            return Err(CreditError::InvalidStateTransition);
        }
        let from_day = self
            .last_accrual_day
            .ok_or(CreditError::InvalidStateTransition)?;
        if day < from_day {
            return Err(CreditError::TimeMovedBackwards);
        }
        if day > self.terms.maturity_day {
            return Err(CreditError::InvalidMaturity);
        }
        if day == from_day {
            return Ok(0);
        }

        let delta_days = (day - from_day) as u128;
        let accrued_minor = match treatment {
            InterestTreatment::Contractual => {
                let numerator = self
                    .principal_outstanding_minor
                    .checked_mul(self.terms.apr_bps as u128)
                    .and_then(|v| v.checked_mul(delta_days))
                    .and_then(|v| v.checked_add(self.interest_fraction_remainder))
                    .ok_or(CreditError::InterestArithmeticOverflow)?;
                let accrued = numerator / INTEREST_DENOMINATOR;
                let remainder = numerator % INTEREST_DENOMINATOR;
                self.accrued_interest_minor = self
                    .accrued_interest_minor
                    .checked_add(accrued)
                    .ok_or(CreditError::InterestArithmeticOverflow)?;
                self.interest_fraction_remainder = remainder;
                accrued
            }
            InterestTreatment::PauseByAgreement => {
                if !matches!(
                    self.state,
                    LoanState::Hardship | LoanState::Forbearance | LoanState::Restructured
                ) || agreement_ref.is_none_or(str::is_empty)
                {
                    return Err(CreditError::HardshipAgreementRequired);
                }
                0
            }
        };

        let agreement = agreement_ref.map(str::to_string);
        self.interest_events.push(InterestAccrualEvent {
            from_day,
            to_day: day,
            treatment,
            agreement_ref: agreement,
            accrued_minor,
        });
        self.last_accrual_day = Some(day);
        Ok(accrued_minor)
    }

    fn calculate_repayment(
        &self,
        receipt: &RepaymentReceipt,
    ) -> Result<RepaymentAllocation, CreditError> {
        if !matches!(
            self.state,
            LoanState::Active
                | LoanState::InArrears
                | LoanState::Hardship
                | LoanState::Forbearance
                | LoanState::Restructured
        ) {
            return Err(CreditError::InvalidStateTransition);
        }
        if receipt.loan_id != self.terms.loan_id
            || receipt.borrower_id != self.terms.borrower_id
            || receipt.lender_id != self.terms.lender_id
            || receipt.currency != self.terms.currency
        {
            return Err(CreditError::PaymentReceiptMismatch);
        }
        if receipt.amount_minor == 0 {
            return Err(CreditError::PaymentAmountInvalid);
        }
        if self.last_accrual_day != Some(receipt.payment_day) {
            return Err(CreditError::PaymentDayNotAccrued);
        }
        let external_receipt_ref = match &receipt.outcome {
            PaymentOutcome::PaidFinal {
                external_receipt_ref,
                finality_policy_ref,
            } => {
                if external_receipt_ref.is_empty() || finality_policy_ref.is_empty() {
                    return Err(CreditError::PaymentNotFinal);
                }
                external_receipt_ref.clone()
            }
            PaymentOutcome::Rejected { .. } => return Err(CreditError::PaymentNotFinal),
            PaymentOutcome::Indeterminate { .. } => return Err(CreditError::PaymentOutcomeUnknown),
        };

        let total_due = self
            .principal_outstanding_minor
            .checked_add(self.accrued_interest_minor)
            .ok_or(CreditError::BalanceArithmeticOverflow)?;
        if receipt.amount_minor > total_due {
            return Err(CreditError::Overpayment);
        }
        let interest_paid = receipt.amount_minor.min(self.accrued_interest_minor);
        let principal_paid = receipt.amount_minor - interest_paid;
        let interest_remaining = self.accrued_interest_minor - interest_paid;
        let principal_remaining = self.principal_outstanding_minor - principal_paid;
        let resulting_state = if interest_remaining == 0 && principal_remaining == 0 {
            LoanState::Settled
        } else {
            self.state
        };

        Ok(RepaymentAllocation {
            loan_id: self.terms.loan_id.clone(),
            total_paid_minor: receipt.amount_minor,
            interest_paid_minor: interest_paid,
            principal_paid_minor: principal_paid,
            principal_remaining_minor: principal_remaining,
            interest_remaining_minor: interest_remaining,
            resulting_state,
            external_receipt_ref,
        })
    }

    fn commit_repayment(&mut self, allocation: &RepaymentAllocation) {
        self.principal_outstanding_minor = allocation.principal_remaining_minor;
        self.accrued_interest_minor = allocation.interest_remaining_minor;
        self.state = allocation.resulting_state;
        self.events.push(LoanEvent {
            event_type: "repayment_applied".to_string(),
            evidence_ref: allocation.external_receipt_ref.clone(),
            day: self.last_accrual_day.unwrap_or_default(),
        });
    }

    fn require_state(&self, expected: LoanState) -> Result<(), CreditError> {
        if self.state != expected {
            return Err(CreditError::InvalidStateTransition);
        }
        Ok(())
    }

    fn push_event(
        &mut self,
        event_type: &str,
        evidence_ref: &str,
        day: u64,
    ) -> Result<(), CreditError> {
        if evidence_ref.is_empty() {
            return Err(CreditError::MissingReference);
        }
        self.events.push(LoanEvent {
            event_type: event_type.to_string(),
            evidence_ref: evidence_ref.to_string(),
            day,
        });
        Ok(())
    }
}

impl CreditPortfolio {
    pub fn insert_loan(&mut self, loan: Loan) -> Result<(), CreditError> {
        let id = loan.loan_id().to_string();
        if self.loans.contains_key(&id) {
            return Err(CreditError::LoanAlreadyExists);
        }
        self.loans.insert(id, loan);
        Ok(())
    }

    pub fn loan(&self, loan_id: &str) -> Option<&Loan> {
        self.loans.get(loan_id)
    }

    /// Applies a repayment only after an external rail reports finality.
    /// All validations happen before either the loan or global replay ledgers mutate.
    pub fn apply_repayment(
        &mut self,
        receipt: RepaymentReceipt,
    ) -> Result<RepaymentDisposition, CreditError> {
        if receipt.idempotency_key.is_empty() {
            return Err(CreditError::MissingReference);
        }
        if let Some(consumed) = self.consumed_idempotency.get(&receipt.idempotency_key) {
            if consumed.request_digest == receipt.request_digest {
                return Ok(RepaymentDisposition::ExactReplay(
                    consumed.allocation.clone(),
                ));
            }
            return Err(CreditError::IdempotencyConflict);
        }
        let allocation = {
            let loan = self
                .loans
                .get(&receipt.loan_id)
                .ok_or(CreditError::LoanNotFound)?;
            loan.calculate_repayment(&receipt)?
        };
        if self
            .consumed_external_receipts
            .contains(&allocation.external_receipt_ref)
        {
            return Err(CreditError::DuplicateExternalReceipt);
        }

        let loan = self
            .loans
            .get_mut(&receipt.loan_id)
            .ok_or(CreditError::LoanNotFound)?;
        loan.commit_repayment(&allocation);
        self.consumed_external_receipts
            .insert(allocation.external_receipt_ref.clone());
        self.consumed_idempotency.insert(
            receipt.idempotency_key,
            ConsumedPayment {
                request_digest: receipt.request_digest,
                allocation: allocation.clone(),
            },
        );
        Ok(RepaymentDisposition::Applied(allocation))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AffordabilityAssessment {
    pub monthly_surplus_minor: u128,
    pub total_debt_service_minor: u128,
    pub debt_service_ratio_bps: u128,
    pub within_policy: bool,
}

pub fn assess_affordability(
    monthly_net_income_minor: u128,
    essential_expenses_minor: u128,
    existing_debt_service_minor: u128,
    proposed_debt_service_minor: u128,
    minimum_surplus_minor: u128,
    maximum_dsr_bps: u16,
) -> Result<AffordabilityAssessment, CreditError> {
    if monthly_net_income_minor == 0
        || maximum_dsr_bps as u128 > BASIS_POINTS
    {
        return Err(CreditError::InvalidAffordabilityInput);
    }
    let outflows = essential_expenses_minor
        .checked_add(existing_debt_service_minor)
        .and_then(|v| v.checked_add(proposed_debt_service_minor))
        .ok_or(CreditError::BalanceArithmeticOverflow)?;
    if outflows > monthly_net_income_minor {
        return Err(CreditError::InvalidAffordabilityInput);
    }
    let monthly_surplus_minor = monthly_net_income_minor - outflows;
    let total_debt_service_minor = existing_debt_service_minor
        .checked_add(proposed_debt_service_minor)
        .ok_or(CreditError::BalanceArithmeticOverflow)?;
    let debt_service_ratio_bps = total_debt_service_minor
        .checked_mul(BASIS_POINTS)
        .ok_or(CreditError::BalanceArithmeticOverflow)?
        / monthly_net_income_minor;
    Ok(AffordabilityAssessment {
        monthly_surplus_minor,
        total_debt_service_minor,
        debt_service_ratio_bps,
        within_policy: debt_service_ratio_bps <= maximum_dsr_bps as u128
            && monthly_surplus_minor >= minimum_surplus_minor,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedCreditLoss {
    pub exposure_at_default_minor: u128,
    pub probability_of_default_bps: u16,
    pub loss_given_default_bps: u16,
    pub expected_loss_minor: u128,
}

/// PD and LGD are supplied by an independently validated model/policy.
/// This is only the arithmetic ECL calculation, not an estimator.
pub fn expected_credit_loss(
    exposure_at_default_minor: u128,
    probability_of_default_bps: u16,
    loss_given_default_bps: u16,
) -> Result<ExpectedCreditLoss, CreditError> {
    if exposure_at_default_minor == 0
        || probability_of_default_bps as u128 > BASIS_POINTS
        || loss_given_default_bps as u128 > BASIS_POINTS
    {
        return Err(CreditError::InvalidRiskInput);
    }
    let numerator = exposure_at_default_minor
        .checked_mul(probability_of_default_bps as u128)
        .and_then(|v| v.checked_mul(loss_given_default_bps as u128))
        .ok_or(CreditError::BalanceArithmeticOverflow)?;
    let expected_loss_minor = numerator / (BASIS_POINTS * BASIS_POINTS);
    Ok(ExpectedCreditLoss {
        exposure_at_default_minor,
        probability_of_default_bps,
        loss_given_default_bps,
        expected_loss_minor,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn terms() -> LoanTerms {
        LoanTerms {
            loan_id: "loan-001".into(),
            borrower_id: "borrower-001".into(),
            lender_id: "lender-001".into(),
            currency: "ZAR".into(),
            original_principal_minor: 100_000,
            apr_bps: 1_000,
            maturity_day: 500,
        }
    }

    fn policy() -> LoanPolicy {
        LoanPolicy {
            max_apr_bps: 3_000,
            max_term_days: 1_000,
        }
    }

    fn active_loan() -> Loan {
        let mut loan = Loan::new(terms(), policy()).unwrap();
        loan.offer("offer", 0).unwrap();
        loan.accept("accept", 0).unwrap();
        loan.fund("funding-final", 0).unwrap();
        loan.activate("service-start", 0).unwrap();
        loan
    }

    fn final_receipt(
        loan: &Loan,
        amount: u128,
        key: &str,
        digest: [u8; 32],
        external_ref: &str,
        day: u64,
    ) -> RepaymentReceipt {
        RepaymentReceipt {
            loan_id: loan.terms.loan_id.clone(),
            borrower_id: loan.terms.borrower_id.clone(),
            lender_id: loan.terms.lender_id.clone(),
            currency: loan.terms.currency.clone(),
            amount_minor: amount,
            payment_day: day,
            idempotency_key: key.into(),
            request_digest: digest,
            outcome: PaymentOutcome::PaidFinal {
                external_receipt_ref: external_ref.into(),
                finality_policy_ref: "bank-rail-finality/1".into(),
            },
        }
    }

    #[test]
    fn loan_requires_valid_principal_currency_rate_and_maturity() {
        let mut invalid = terms();
        invalid.original_principal_minor = 0;
        assert_eq!(
            Loan::new(invalid, policy()).err(),
            Some(CreditError::InvalidPrincipal)
        );
        let mut invalid = terms();
        invalid.currency = "Zar".into();
        assert_eq!(
            Loan::new(invalid, policy()).err(),
            Some(CreditError::InvalidCurrency)
        );
        let mut invalid = terms();
        invalid.apr_bps = 3_001;
        assert_eq!(
            Loan::new(invalid, policy()).err(),
            Some(CreditError::InvalidApr)
        );
    }

    #[test]
    fn loan_lifecycle_requires_offer_acceptance_and_funding() {
        let mut loan = Loan::new(terms(), policy()).unwrap();
        assert_eq!(loan.activate("x", 0), Err(CreditError::InvalidStateTransition));
        loan.offer("offer", 0).unwrap();
        loan.accept("accept", 1).unwrap();
        loan.fund("bank-funding-receipt", 2).unwrap();
        loan.activate("service-start", 2).unwrap();
        assert_eq!(loan.state(), LoanState::Active);
    }

    #[test]
    fn funding_rejects_terms_beyond_policy_maximum() {
        let mut loan = Loan::new(terms(), LoanPolicy {
            max_apr_bps: 3_000,
            max_term_days: 100,
        }).unwrap();
        loan.offer("offer", 0).unwrap();
        loan.accept("accept", 0).unwrap();
        assert_eq!(loan.fund("fund", 0), Err(CreditError::InvalidMaturity));
        assert_eq!(loan.state(), LoanState::Accepted);
    }

    #[test]
    fn interest_cannot_accrue_beyond_contractual_maturity() {
        let mut loan = active_loan();
        assert_eq!(
            loan.accrue_interest_to_day(501, InterestTreatment::Contractual, None),
            Err(CreditError::InvalidMaturity)
        );
    }

    #[test]
    fn simple_interest_accrues_using_actual_365_day_count() {
        let mut loan = active_loan();
        assert_eq!(
            loan.accrue_interest_to_day(365, InterestTreatment::Contractual, None)
                .unwrap(),
            10_000
        );
        assert_eq!(loan.accrued_interest_minor(), 10_000);
    }

    #[test]
    fn splitting_interest_accrual_does_not_lose_fractional_remainder() {
        let mut one_step = active_loan();
        let mut split = active_loan();
        one_step
            .accrue_interest_to_day(365, InterestTreatment::Contractual, None)
            .unwrap();
        split
            .accrue_interest_to_day(100, InterestTreatment::Contractual, None)
            .unwrap();
        split
            .accrue_interest_to_day(365, InterestTreatment::Contractual, None)
            .unwrap();
        assert_eq!(
            one_step.accrued_interest_minor(),
            split.accrued_interest_minor()
        );
        assert_eq!(
            one_step.interest_fraction_remainder,
            split.interest_fraction_remainder
        );
    }

    #[test]
    fn interest_cannot_be_accrued_backwards_or_after_settlement() {
        let mut loan = active_loan();
        loan.accrue_interest_to_day(10, InterestTreatment::Contractual, None)
            .unwrap();
        assert_eq!(
            loan.accrue_interest_to_day(9, InterestTreatment::Contractual, None),
            Err(CreditError::TimeMovedBackwards)
        );
    }

    #[test]
    fn interest_pause_requires_explicit_hardship_agreement() {
        let mut loan = active_loan();
        assert_eq!(
            loan.accrue_interest_to_day(30, InterestTreatment::PauseByAgreement, None),
            Err(CreditError::HardshipAgreementRequired)
        );
        loan.enter_hardship("hardship-case", 1).unwrap();
        assert_eq!(
            loan.accrue_interest_to_day(30, InterestTreatment::PauseByAgreement, Some("pause-agreement"))
                .unwrap(),
            0
        );
    }

    #[test]
    fn payment_unknown_does_not_change_loan_balance() {
        let mut portfolio = CreditPortfolio::default();
        let mut loan = active_loan();
        loan.accrue_interest_to_day(30, InterestTreatment::Contractual, None)
            .unwrap();
        portfolio.insert_loan(loan.clone()).unwrap();
        let mut receipt = final_receipt(&loan, 5_000, "key-1", [1; 32], "rail-1", 30);
        receipt.outcome = PaymentOutcome::Indeterminate {
            reconciliation_ref: Some("query-pending".into()),
        };
        assert_eq!(
            portfolio.apply_repayment(receipt),
            Err(CreditError::PaymentOutcomeUnknown)
        );
        let after = portfolio.loan("loan-001").unwrap();
        assert_eq!(after.principal_outstanding_minor(), 100_000);
    }

    #[test]
    fn repayment_allocates_interest_before_principal() {
        let mut portfolio = CreditPortfolio::default();
        let mut loan = active_loan();
        loan.accrue_interest_to_day(365, InterestTreatment::Contractual, None)
            .unwrap();
        portfolio.insert_loan(loan.clone()).unwrap();
        let disposition = portfolio
            .apply_repayment(final_receipt(&loan, 15_000, "key-1", [1; 32], "rail-1", 365))
            .unwrap();
        let RepaymentDisposition::Applied(allocation) = disposition else {
            panic!("payment should be newly applied");
        };
        assert_eq!(allocation.interest_paid_minor, 10_000);
        assert_eq!(allocation.principal_paid_minor, 5_000);
        assert_eq!(allocation.principal_remaining_minor, 95_000);
        assert_eq!(portfolio.loan("loan-001").unwrap().state(), LoanState::Active);
    }

    #[test]
    fn exact_payment_retry_returns_original_allocation_without_double_apply() {
        let mut portfolio = CreditPortfolio::default();
        let mut loan = active_loan();
        loan.accrue_interest_to_day(365, InterestTreatment::Contractual, None)
            .unwrap();
        portfolio.insert_loan(loan.clone()).unwrap();
        let receipt = final_receipt(&loan, 15_000, "key-1", [1; 32], "rail-1", 365);
        let first = portfolio.apply_repayment(receipt.clone()).unwrap();
        let retry = portfolio.apply_repayment(receipt).unwrap();
        assert!(matches!(first, RepaymentDisposition::Applied(_)));
        assert!(matches!(retry, RepaymentDisposition::ExactReplay(_)));
        assert_eq!(
            portfolio.loan("loan-001").unwrap().principal_outstanding_minor(),
            95_000
        );
    }

    #[test]
    fn same_idempotency_key_with_changed_request_digest_is_rejected() {
        let mut portfolio = CreditPortfolio::default();
        let mut loan = active_loan();
        loan.accrue_interest_to_day(365, InterestTreatment::Contractual, None)
            .unwrap();
        portfolio.insert_loan(loan.clone()).unwrap();
        portfolio
            .apply_repayment(final_receipt(&loan, 15_000, "key-1", [1; 32], "rail-1", 365))
            .unwrap();
        assert_eq!(
            portfolio.apply_repayment(final_receipt(&loan, 15_001, "key-1", [2; 32], "rail-2", 365)),
            Err(CreditError::IdempotencyConflict)
        );
    }

    #[test]
    fn external_payment_receipt_cannot_be_consumed_by_two_loans() {
        let mut portfolio = CreditPortfolio::default();
        let mut loan_a = active_loan();
        loan_a.accrue_interest_to_day(30, InterestTreatment::Contractual, None)
            .unwrap();
        let mut terms_b = terms();
        terms_b.loan_id = "loan-002".into();
        let mut loan_b = Loan::new(terms_b, policy()).unwrap();
        loan_b.offer("offer-b", 0).unwrap();
        loan_b.accept("accept-b", 0).unwrap();
        loan_b.fund("fund-b", 0).unwrap();
        loan_b.activate("start-b", 0).unwrap();
        loan_b.accrue_interest_to_day(30, InterestTreatment::Contractual, None)
            .unwrap();
        portfolio.insert_loan(loan_a.clone()).unwrap();
        portfolio.insert_loan(loan_b.clone()).unwrap();
        portfolio
            .apply_repayment(final_receipt(&loan_a, 1_000, "key-a", [1; 32], "same-rail-ref", 30))
            .unwrap();
        let mut duplicate = final_receipt(&loan_b, 1_000, "key-b", [2; 32], "same-rail-ref", 30);
        duplicate.loan_id = "loan-002".into();
        assert_eq!(
            portfolio.apply_repayment(duplicate),
            Err(CreditError::DuplicateExternalReceipt)
        );
    }

    #[test]
    fn overpayment_is_rejected_without_mutating_the_loan() {
        let mut portfolio = CreditPortfolio::default();
        let mut loan = active_loan();
        loan.accrue_interest_to_day(1, InterestTreatment::Contractual, None)
            .unwrap();
        portfolio.insert_loan(loan.clone()).unwrap();
        let before = portfolio.loan("loan-001").unwrap().clone();
        assert_eq!(
            portfolio.apply_repayment(final_receipt(&loan, 200_000, "key-1", [1; 32], "rail-1", 1)),
            Err(CreditError::Overpayment)
        );
        assert_eq!(portfolio.loan("loan-001").unwrap(), &before);
    }

    #[test]
    fn hardship_forbearance_and_restructure_are_explicit_and_non_destructive() {
        let mut loan = active_loan();
        loan.enter_hardship("hardship-case", 10).unwrap();
        loan.grant_forbearance("forbearance-agreement", 11).unwrap();
        let principal_before = loan.principal_outstanding_minor();
        loan.restructure(500, 900, "restructure-agreement", "committee-approval", 12)
            .unwrap();
        assert_eq!(loan.principal_outstanding_minor(), principal_before);
        assert_eq!(loan.apr_bps(), 500);
        assert_eq!(loan.state(), LoanState::Restructured);
    }

    #[test]
    fn dispute_freezes_repayment_until_explicit_resolution() {
        let mut portfolio = CreditPortfolio::default();
        let mut loan = active_loan();
        loan.accrue_interest_to_day(30, InterestTreatment::Contractual, None)
            .unwrap();
        loan.open_dispute("repayment-dispute", 30).unwrap();
        portfolio.insert_loan(loan.clone()).unwrap();
        let receipt = final_receipt(&loan, 1_000, "key-1", [1; 32], "rail-1", 30);
        assert_eq!(
            portfolio.apply_repayment(receipt),
            Err(CreditError::InvalidStateTransition)
        );
        portfolio
            .loans
            .get_mut("loan-001")
            .unwrap()
            .resolve_dispute("resolution-record", 31)
            .unwrap();
        let receipt = final_receipt(
            portfolio.loan("loan-001").unwrap(),
            1_000,
            "key-1",
            [1; 32],
            "rail-1",
            30,
        );
        assert!(portfolio.apply_repayment(receipt).is_ok());
    }

    #[test]
    fn default_and_writeoff_require_separate_authority_evidence() {
        let mut loan = active_loan();
        assert_eq!(
            loan.declare_default("", "authority", 100),
            Err(CreditError::MissingReference)
        );
        loan.mark_in_arrears("arrears", 100).unwrap();
        loan.declare_default("default-evidence", "default-authority", 101)
            .unwrap();
        assert_eq!(loan.state(), LoanState::Defaulted);
        assert_eq!(loan.write_off("", "authority", 102), Err(CreditError::MissingReference));
        loan.write_off("accounting-entry", "writeoff-authority", 103)
            .unwrap();
        assert_eq!(loan.state(), LoanState::WrittenOff);
        assert_eq!(loan.principal_outstanding_minor(), 100_000);
    }

    #[test]
    fn affordability_reports_ratio_and_minimum_surplus() {
        let result = assess_affordability(100_000, 40_000, 10_000, 20_000, 20_000, 3_500).unwrap();
        assert_eq!(result.debt_service_ratio_bps, 3_000);
        assert!(result.within_policy);
        let denied = assess_affordability(100_000, 40_000, 20_000, 20_000, 30_000, 3_500).unwrap();
        assert!(!denied.within_policy);
    }

    #[test]
    fn expected_credit_loss_uses_pd_lgd_ead_not_reputation_scores() {
        let result = expected_credit_loss(1_000_000, 500, 4_000).unwrap();
        assert_eq!(result.expected_loss_minor, 20_000);
    }

    #[test]
    fn zero_or_out_of_range_risk_inputs_are_rejected() {
        assert_eq!(
            expected_credit_loss(0, 100, 100),
            Err(CreditError::InvalidRiskInput)
        );
        assert_eq!(
            expected_credit_loss(1_000, 10_001, 100),
            Err(CreditError::InvalidRiskInput)
        );
    }
}
