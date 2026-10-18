//! Creator royalty statement-to-cash reconciliation.
//!
//! This module keeps reported royalty entitlements distinct from actual cash
//! receipts. Statement lines are claims made by a reporting source; cash is
//! evidenced by an existing Ledger MoneyEvent of kind Receipt. The module
//! records creator-approved allocations between them, but does not prove the
//! underlying source truth or provide bank-account integration.

use std::collections::BTreeMap;
use std::error::Error;
use std::fmt;

use crate::{Ledger, MoneyEvent, MoneyKind};

const MAX_FIELD_LENGTH: usize = 256;

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SettlementError {
    EmptyField(&'static str),
    FieldTooLong(&'static str),
    InvalidFieldCharacter(&'static str),
    InvalidCurrency(String),
    NonPositiveAmount,
    InvalidReportingPeriod,
    ConflictingReplay {
        record_type: &'static str,
        id: String,
    },
    StatementLineNotFound(String),
    ReceiptNotFound(String),
    ReceiptNotActualCashReceipt(String),
    CurrencyMismatch {
        statement_currency: String,
        receipt_currency: String,
    },
    AllocationExceedsStatement {
        statement_line_id: String,
        requested_minor_units: i64,
        remaining_minor_units: i64,
    },
    AllocationExceedsReceipt {
        receipt_event_id: String,
        requested_minor_units: i64,
        remaining_minor_units: i64,
    },
    AllocationNotFound(String),
    AllocationAlreadyReversed(String),
    ArithmeticOverflow,
}

impl fmt::Display for SettlementError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyField(name) => write!(f, "{name} must not be empty"),
            Self::FieldTooLong(name) => write!(f, "{name} exceeds the maximum length"),
            Self::InvalidFieldCharacter(name) => {
                write!(f, "{name} contains a control character")
            }
            Self::InvalidCurrency(value) => write!(
                f,
                "currency must be three uppercase ASCII letters: {value:?}"
            ),
            Self::NonPositiveAmount => write!(f, "amount must be positive minor units"),
            Self::InvalidReportingPeriod => {
                write!(f, "reporting period end must be after period start")
            }
            Self::ConflictingReplay { record_type, id } => {
                write!(f, "conflicting {record_type} replay for identifier {id:?}")
            }
            Self::StatementLineNotFound(id) => {
                write!(f, "royalty statement line {id:?} was not found")
            }
            Self::ReceiptNotFound(id) => write!(f, "cash receipt {id:?} was not found"),
            Self::ReceiptNotActualCashReceipt(id) => {
                write!(f, "ledger event {id:?} is not an actual cash receipt")
            }
            Self::CurrencyMismatch {
                statement_currency,
                receipt_currency,
            } => write!(
                f,
                "currency mismatch: statement {statement_currency} vs receipt {receipt_currency}"
            ),
            Self::AllocationExceedsStatement {
                statement_line_id,
                requested_minor_units,
                remaining_minor_units,
            } => write!(
                f,
                "allocation {requested_minor_units} exceeds statement line {statement_line_id:?} remainder {remaining_minor_units}"
            ),
            Self::AllocationExceedsReceipt {
                receipt_event_id,
                requested_minor_units,
                remaining_minor_units,
            } => write!(
                f,
                "allocation {requested_minor_units} exceeds receipt {receipt_event_id:?} remainder {remaining_minor_units}"
            ),
            Self::AllocationNotFound(id) => write!(f, "allocation {id:?} was not found"),
            Self::AllocationAlreadyReversed(id) => {
                write!(f, "allocation {id:?} has already been reversed")
            }
            Self::ArithmeticOverflow => write!(f, "checked arithmetic overflow"),
        }
    }
}

impl Error for SettlementError {}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SettlementInsertOutcome {
    Inserted,
    IdenticalReplay,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RoyaltyStatementLine {
    line_id: String,
    statement_id: String,
    reporting_source_id: String,
    /// Versioned rule that established this line as a payable assertion.
    semantic_mapping_rule_id: String,
    semantic_mapping_version: String,
    /// Reference to the source bytes or creator-reviewed evidence supporting the interpretation.
    source_evidence_ref: String,
    period_start_utc_seconds: i64,
    period_end_utc_seconds: i64,
    currency: String,
    /// Positive integer minor units reported as payable for this line.
    /// A statement assertion is not a receipt and must not be added to cash totals.
    amount_minor_units: i64,
}

impl RoyaltyStatementLine {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        line_id: impl AsRef<str>,
        statement_id: impl AsRef<str>,
        reporting_source_id: impl AsRef<str>,
        semantic_mapping_rule_id: impl AsRef<str>,
        semantic_mapping_version: impl AsRef<str>,
        source_evidence_ref: impl AsRef<str>,
        period_start_utc_seconds: i64,
        period_end_utc_seconds: i64,
        currency: impl AsRef<str>,
        amount_minor_units: i64,
    ) -> Result<Self, SettlementError> {
        if period_end_utc_seconds <= period_start_utc_seconds {
            return Err(SettlementError::InvalidReportingPeriod);
        }
        if amount_minor_units <= 0 {
            return Err(SettlementError::NonPositiveAmount);
        }
        let currency = currency.as_ref().to_owned();
        if !valid_currency(&currency) {
            return Err(SettlementError::InvalidCurrency(currency));
        }

        Ok(Self {
            line_id: validate_field(line_id.as_ref(), "line_id")?,
            statement_id: validate_field(statement_id.as_ref(), "statement_id")?,
            reporting_source_id: validate_field(
                reporting_source_id.as_ref(),
                "reporting_source_id",
            )?,
            semantic_mapping_rule_id: validate_field(
                semantic_mapping_rule_id.as_ref(),
                "semantic_mapping_rule_id",
            )?,
            semantic_mapping_version: validate_field(
                semantic_mapping_version.as_ref(),
                "semantic_mapping_version",
            )?,
            source_evidence_ref: validate_field(
                source_evidence_ref.as_ref(),
                "source_evidence_ref",
            )?,
            period_start_utc_seconds,
            period_end_utc_seconds,
            currency,
            amount_minor_units,
        })
    }

    pub fn line_id(&self) -> &str {
        &self.line_id
    }

    pub fn statement_id(&self) -> &str {
        &self.statement_id
    }

    pub fn reporting_source_id(&self) -> &str {
        &self.reporting_source_id
    }

    pub fn semantic_mapping_rule_id(&self) -> &str {
        &self.semantic_mapping_rule_id
    }

    pub fn semantic_mapping_version(&self) -> &str {
        &self.semantic_mapping_version
    }

    pub fn source_evidence_ref(&self) -> &str {
        &self.source_evidence_ref
    }

    pub fn period_start_utc_seconds(&self) -> i64 {
        self.period_start_utc_seconds
    }

    pub fn period_end_utc_seconds(&self) -> i64 {
        self.period_end_utc_seconds
    }

    pub fn currency(&self) -> &str {
        &self.currency
    }

    pub fn amount_minor_units(&self) -> i64 {
        self.amount_minor_units
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ReconciliationAllocation {
    allocation_id: String,
    statement_line_id: String,
    receipt_event_id: String,
    amount_minor_units: i64,
    approved_by: String,
    evidence_ref: String,
}

impl ReconciliationAllocation {
    pub fn new(
        allocation_id: impl AsRef<str>,
        statement_line_id: impl AsRef<str>,
        receipt_event_id: impl AsRef<str>,
        amount_minor_units: i64,
        approved_by: impl AsRef<str>,
        evidence_ref: impl AsRef<str>,
    ) -> Result<Self, SettlementError> {
        if amount_minor_units <= 0 {
            return Err(SettlementError::NonPositiveAmount);
        }
        Ok(Self {
            allocation_id: validate_field(allocation_id.as_ref(), "allocation_id")?,
            statement_line_id: validate_field(statement_line_id.as_ref(), "statement_line_id")?,
            receipt_event_id: validate_field(receipt_event_id.as_ref(), "receipt_event_id")?,
            amount_minor_units,
            approved_by: validate_field(approved_by.as_ref(), "approved_by")?,
            evidence_ref: validate_field(evidence_ref.as_ref(), "evidence_ref")?,
        })
    }

    pub fn allocation_id(&self) -> &str {
        &self.allocation_id
    }

    pub fn statement_line_id(&self) -> &str {
        &self.statement_line_id
    }

    pub fn receipt_event_id(&self) -> &str {
        &self.receipt_event_id
    }

    pub fn amount_minor_units(&self) -> i64 {
        self.amount_minor_units
    }

    pub fn approved_by(&self) -> &str {
        &self.approved_by
    }

    pub fn evidence_ref(&self) -> &str {
        &self.evidence_ref
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AllocationReversal {
    reversal_id: String,
    allocation_id: String,
    approved_by: String,
    reason: String,
    evidence_ref: String,
}

impl AllocationReversal {
    pub fn new(
        reversal_id: impl AsRef<str>,
        allocation_id: impl AsRef<str>,
        approved_by: impl AsRef<str>,
        reason: impl AsRef<str>,
        evidence_ref: impl AsRef<str>,
    ) -> Result<Self, SettlementError> {
        let reason = validate_field(reason.as_ref(), "reason")?;
        Ok(Self {
            reversal_id: validate_field(reversal_id.as_ref(), "reversal_id")?,
            allocation_id: validate_field(allocation_id.as_ref(), "allocation_id")?,
            approved_by: validate_field(approved_by.as_ref(), "approved_by")?,
            reason,
            evidence_ref: validate_field(evidence_ref.as_ref(), "evidence_ref")?,
        })
    }

    pub fn reversal_id(&self) -> &str {
        &self.reversal_id
    }

    pub fn allocation_id(&self) -> &str {
        &self.allocation_id
    }

    pub fn approved_by(&self) -> &str {
        &self.approved_by
    }

    pub fn reason(&self) -> &str {
        &self.reason
    }

    pub fn evidence_ref(&self) -> &str {
        &self.evidence_ref
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReconciliationState {
    Unmatched,
    PartiallyMatched,
    FullyMatched,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct StatementLineReconciliation {
    pub line_id: String,
    pub currency: String,
    pub reported_minor_units: i64,
    pub allocated_minor_units: i64,
    pub outstanding_minor_units: i64,
    pub state: ReconciliationState,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ReceiptReconciliation {
    pub receipt_event_id: String,
    pub currency: String,
    pub receipt_minor_units: i64,
    pub allocated_minor_units: i64,
    pub unallocated_minor_units: i64,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct SettlementBook {
    statement_lines: BTreeMap<String, RoyaltyStatementLine>,
    allocations: BTreeMap<String, ReconciliationAllocation>,
    reversals: BTreeMap<String, AllocationReversal>,
    reversed_allocations: BTreeMap<String, String>,
}

impl SettlementBook {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn statement_lines(&self) -> impl Iterator<Item = &RoyaltyStatementLine> {
        self.statement_lines.values()
    }

    /// All allocation events, including reversed ones, remain available for audit.
    pub fn allocations(&self) -> impl Iterator<Item = &ReconciliationAllocation> {
        self.allocations.values()
    }

    pub fn reversals(&self) -> impl Iterator<Item = &AllocationReversal> {
        self.reversals.values()
    }

    pub fn insert_statement_line(
        &mut self,
        line: RoyaltyStatementLine,
    ) -> Result<SettlementInsertOutcome, SettlementError> {
        if let Some(existing) = self.statement_lines.get(line.line_id()) {
            return if existing == &line {
                Ok(SettlementInsertOutcome::IdenticalReplay)
            } else {
                Err(SettlementError::ConflictingReplay {
                    record_type: "royalty statement line",
                    id: line.line_id,
                })
            };
        }

        self.statement_lines.insert(line.line_id.clone(), line);
        Ok(SettlementInsertOutcome::Inserted)
    }

    /// Record an explicit approved allocation from a reported royalty line to
    /// an existing cash receipt. No amount or currency is inferred.
    pub fn allocate(
        &mut self,
        ledger: &Ledger,
        allocation: ReconciliationAllocation,
    ) -> Result<SettlementInsertOutcome, SettlementError> {
        if let Some(existing) = self.allocations.get(allocation.allocation_id()) {
            return if existing == &allocation {
                Ok(SettlementInsertOutcome::IdenticalReplay)
            } else {
                Err(SettlementError::ConflictingReplay {
                    record_type: "reconciliation allocation",
                    id: allocation.allocation_id,
                })
            };
        }

        let line = self
            .statement_lines
            .get(allocation.statement_line_id())
            .ok_or_else(|| {
                SettlementError::StatementLineNotFound(
                    allocation.statement_line_id().to_owned(),
                )
            })?;
        let receipt = find_receipt(ledger, allocation.receipt_event_id())?;

        if receipt.currency != line.currency {
            return Err(SettlementError::CurrencyMismatch {
                statement_currency: line.currency.clone(),
                receipt_currency: receipt.currency.clone(),
            });
        }

        let line_allocated = self.active_allocated_for_line(line.line_id())?;
        let line_remaining = i128::from(line.amount_minor_units()) - line_allocated;
        let requested = i128::from(allocation.amount_minor_units());
        if requested > line_remaining {
            return Err(SettlementError::AllocationExceedsStatement {
                statement_line_id: line.line_id().to_owned(),
                requested_minor_units: allocation.amount_minor_units(),
                remaining_minor_units: to_i64(line_remaining)?,
            });
        }

        let receipt_allocated = self.active_allocated_for_receipt(receipt.event_id.as_str())?;
        let receipt_remaining = i128::from(receipt.amount_minor_units) - receipt_allocated;
        if requested > receipt_remaining {
            return Err(SettlementError::AllocationExceedsReceipt {
                receipt_event_id: receipt.event_id.clone(),
                requested_minor_units: allocation.amount_minor_units(),
                remaining_minor_units: to_i64(receipt_remaining)?,
            });
        }

        self.allocations
            .insert(allocation.allocation_id.clone(), allocation);
        Ok(SettlementInsertOutcome::Inserted)
    }

    /// Reverse an allocation without erasing its history; capacity is released
    /// only after the reversal event has been validated and recorded.
    pub fn reverse_allocation(
        &mut self,
        reversal: AllocationReversal,
    ) -> Result<SettlementInsertOutcome, SettlementError> {
        if let Some(existing) = self.reversals.get(reversal.reversal_id()) {
            return if existing == &reversal {
                Ok(SettlementInsertOutcome::IdenticalReplay)
            } else {
                Err(SettlementError::ConflictingReplay {
                    record_type: "allocation reversal",
                    id: reversal.reversal_id,
                })
            };
        }

        if !self.allocations.contains_key(reversal.allocation_id()) {
            return Err(SettlementError::AllocationNotFound(
                reversal.allocation_id().to_owned(),
            ));
        }
        if self
            .reversed_allocations
            .contains_key(reversal.allocation_id())
        {
            return Err(SettlementError::AllocationAlreadyReversed(
                reversal.allocation_id().to_owned(),
            ));
        }

        self.reversed_allocations.insert(
            reversal.allocation_id.clone(),
            reversal.reversal_id.clone(),
        );
        self.reversals.insert(reversal.reversal_id.clone(), reversal);
        Ok(SettlementInsertOutcome::Inserted)
    }

    pub fn statement_line_status(
        &self,
        line_id: &str,
    ) -> Result<StatementLineReconciliation, SettlementError> {
        let line = self
            .statement_lines
            .get(line_id)
            .ok_or_else(|| SettlementError::StatementLineNotFound(line_id.to_owned()))?;
        let allocated_minor_units = to_i64(self.active_allocated_for_line(line_id)?)?;
        let outstanding_minor_units = line
            .amount_minor_units()
            .checked_sub(allocated_minor_units)
            .ok_or(SettlementError::ArithmeticOverflow)?;
        let state = if allocated_minor_units == 0 {
            ReconciliationState::Unmatched
        } else if outstanding_minor_units == 0 {
            ReconciliationState::FullyMatched
        } else {
            ReconciliationState::PartiallyMatched
        };

        Ok(StatementLineReconciliation {
            line_id: line.line_id().to_owned(),
            currency: line.currency().to_owned(),
            reported_minor_units: line.amount_minor_units(),
            allocated_minor_units,
            outstanding_minor_units,
            state,
        })
    }

    pub fn receipt_status(
        &self,
        ledger: &Ledger,
        receipt_event_id: &str,
    ) -> Result<ReceiptReconciliation, SettlementError> {
        let receipt = find_receipt(ledger, receipt_event_id)?;
        let allocated_minor_units = to_i64(
            self.active_allocated_for_receipt(receipt_event_id)?,
        )?;
        let unallocated_minor_units = receipt
            .amount_minor_units
            .checked_sub(allocated_minor_units)
            .ok_or(SettlementError::ArithmeticOverflow)?;
        Ok(ReceiptReconciliation {
            receipt_event_id: receipt.event_id.clone(),
            currency: receipt.currency.clone(),
            receipt_minor_units: receipt.amount_minor_units,
            allocated_minor_units,
            unallocated_minor_units,
        })
    }

    pub fn statement_line_count(&self) -> usize {
        self.statement_lines.len()
    }

    pub fn allocation_count(&self) -> usize {
        self.allocations.len()
    }

    pub fn reversal_count(&self) -> usize {
        self.reversals.len()
    }

    fn active_allocated_for_line(&self, line_id: &str) -> Result<i128, SettlementError> {
        self.allocations
            .values()
            .filter(|allocation| {
                allocation.statement_line_id() == line_id
                    && !self
                        .reversed_allocations
                        .contains_key(allocation.allocation_id())
            })
            .try_fold(0_i128, |total, allocation| {
                total
                    .checked_add(i128::from(allocation.amount_minor_units()))
                    .ok_or(SettlementError::ArithmeticOverflow)
            })
    }

    fn active_allocated_for_receipt(
        &self,
        receipt_event_id: &str,
    ) -> Result<i128, SettlementError> {
        self.allocations
            .values()
            .filter(|allocation| {
                allocation.receipt_event_id() == receipt_event_id
                    && !self
                        .reversed_allocations
                        .contains_key(allocation.allocation_id())
            })
            .try_fold(0_i128, |total, allocation| {
                total
                    .checked_add(i128::from(allocation.amount_minor_units()))
                    .ok_or(SettlementError::ArithmeticOverflow)
            })
    }
}

fn find_receipt<'a>(
    ledger: &'a Ledger,
    receipt_event_id: &str,
) -> Result<&'a MoneyEvent, SettlementError> {
    let event = ledger
        .money_events()
        .find(|event| event.event_id == receipt_event_id)
        .ok_or_else(|| SettlementError::ReceiptNotFound(receipt_event_id.to_owned()))?;
    if event.kind != MoneyKind::Receipt {
        return Err(SettlementError::ReceiptNotActualCashReceipt(
            receipt_event_id.to_owned(),
        ));
    }
    Ok(event)
}

fn validate_field(value: &str, name: &'static str) -> Result<String, SettlementError> {
    let value = value.trim();
    if value.is_empty() {
        return Err(SettlementError::EmptyField(name));
    }
    if value.len() > MAX_FIELD_LENGTH {
        return Err(SettlementError::FieldTooLong(name));
    }
    if value.chars().any(char::is_control) {
        return Err(SettlementError::InvalidFieldCharacter(name));
    }
    Ok(value.to_owned())
}

fn valid_currency(value: &str) -> bool {
    value.len() == 3 && value.bytes().all(|byte| byte.is_ascii_uppercase())
}

fn to_i64(value: i128) -> Result<i64, SettlementError> {
    i64::try_from(value).map_err(|_| SettlementError::ArithmeticOverflow)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MoneyEvent, WorkSession};

    fn line(id: &str, amount: i64, currency: &str) -> RoyaltyStatementLine {
        RoyaltyStatementLine::new(
            id,
            "statement-1",
            "distributor-1",
            "fixture-payable-normalization",
            "1",
            "evidence://source/statement-line",
            100,
            200,
            currency,
            amount,
        )
        .unwrap()
    }

    fn receipt(id: &str, amount: i64, currency: &str, kind: MoneyKind) -> MoneyEvent {
        MoneyEvent {
            event_id: id.to_owned(),
            source_id: "creator-bank-export".to_owned(),
            occurred_at_utc_seconds: 150,
            currency: currency.to_owned(),
            amount_minor_units: amount,
            kind,
        }
    }

    fn allocation(
        id: &str,
        line_id: &str,
        receipt_id: &str,
        amount: i64,
    ) -> ReconciliationAllocation {
        ReconciliationAllocation::new(
            id,
            line_id,
            receipt_id,
            amount,
            "creator-confirmed",
            "evidence://review/1",
        )
        .unwrap()
    }

    fn seeded_ledger() -> Ledger {
        let mut ledger = Ledger::new();
        ledger
            .insert_money_event(receipt("cash-1", 7_500, "USD", MoneyKind::Receipt))
            .unwrap();
        ledger
            .insert_money_event(receipt("fee-1", 200, "USD", MoneyKind::Fee))
            .unwrap();
        ledger
    }

    #[test]
    fn reported_statement_line_does_not_create_cash() {
        let mut ledger = Ledger::new();
        ledger
            .insert_work_session(WorkSession {
                session_id: "work-1".to_owned(),
                start_utc_seconds: 1,
                end_utc_seconds: 2,
            })
            .unwrap();
        let before = ledger.summarize().unwrap();
        let mut book = SettlementBook::new();
        book.insert_statement_line(line("line-1", 7_500, "USD"))
            .unwrap();

        assert_eq!(ledger.summarize().unwrap(), before);
        assert_eq!(ledger.summarize().unwrap().money_event_count, 0);
        assert_eq!(book.statement_line_count(), 1);
    }

    #[test]
    fn partial_cash_allocation_can_reconcile_a_statement_line_incrementally() {
        let ledger = seeded_ledger();
        let mut book = SettlementBook::new();
        book.insert_statement_line(line("line-1", 10_000, "USD"))
            .unwrap();

        book.allocate(&ledger, allocation("a-1", "line-1", "cash-1", 4_000))
            .unwrap();
        let partial = book.statement_line_status("line-1").unwrap();
        assert_eq!(partial.allocated_minor_units, 4_000);
        assert_eq!(partial.outstanding_minor_units, 6_000);
        assert_eq!(partial.state, ReconciliationState::PartiallyMatched);

        book.allocate(&ledger, allocation("a-2", "line-1", "cash-1", 3_500))
            .unwrap();
        let complete = book.statement_line_status("line-1").unwrap();
        assert_eq!(complete.allocated_minor_units, 7_500);
        assert_eq!(complete.outstanding_minor_units, 2_500);
        assert_eq!(complete.state, ReconciliationState::PartiallyMatched);
    }

    #[test]
    fn receipt_can_be_allocated_across_lines_but_never_overallocated() {
        let ledger = seeded_ledger();
        let mut book = SettlementBook::new();
        book.insert_statement_line(line("line-a", 4_000, "USD"))
            .unwrap();
        book.insert_statement_line(line("line-b", 5_000, "USD"))
            .unwrap();
        book.allocate(&ledger, allocation("a-1", "line-a", "cash-1", 4_000))
            .unwrap();
        book.allocate(&ledger, allocation("a-2", "line-b", "cash-1", 3_500))
            .unwrap();

        assert_eq!(
            book.allocate(&ledger, allocation("a-3", "line-b", "cash-1", 1))
                .unwrap_err(),
            SettlementError::AllocationExceedsReceipt {
                receipt_event_id: "cash-1".to_owned(),
                requested_minor_units: 1,
                remaining_minor_units: 0,
            }
        );
        let status = book.receipt_status(&ledger, "cash-1").unwrap();
        assert_eq!(status.allocated_minor_units, 7_500);
        assert_eq!(status.unallocated_minor_units, 0);
        assert_eq!(book.allocation_count(), 2);
    }

    #[test]
    fn rejects_fees_as_cash_and_rejects_currency_mismatch() {
        let ledger = seeded_ledger();
        let mut book = SettlementBook::new();
        book.insert_statement_line(line("line-1", 100, "USD"))
            .unwrap();

        assert_eq!(
            book.allocate(&ledger, allocation("a-fee", "line-1", "fee-1", 100)),
            Err(SettlementError::ReceiptNotActualCashReceipt(
                "fee-1".to_owned()
            ))
        );

        let mut other_ledger = Ledger::new();
        other_ledger
            .insert_money_event(receipt("cash-eur", 100, "EUR", MoneyKind::Receipt))
            .unwrap();
        assert_eq!(
            book.allocate(
                &other_ledger,
                allocation("a-eur", "line-1", "cash-eur", 100)
            ),
            Err(SettlementError::CurrencyMismatch {
                statement_currency: "USD".to_owned(),
                receipt_currency: "EUR".to_owned(),
            })
        );
        assert_eq!(book.allocation_count(), 0);
    }

    #[test]
    fn exact_replays_are_idempotent_and_conflicting_ids_fail_closed() {
        let ledger = seeded_ledger();
        let mut book = SettlementBook::new();
        book.insert_statement_line(line("line-1", 5_000, "USD"))
            .unwrap();
        assert_eq!(
            book.insert_statement_line(line("line-1", 5_000, "USD")),
            Ok(SettlementInsertOutcome::IdenticalReplay)
        );
        book.allocate(&ledger, allocation("a-1", "line-1", "cash-1", 2_000))
            .unwrap();
        assert_eq!(
            book.allocate(&ledger, allocation("a-1", "line-1", "cash-1", 2_000)),
            Ok(SettlementInsertOutcome::IdenticalReplay)
        );
        assert!(matches!(
            book.allocate(&ledger, allocation("a-1", "line-1", "cash-1", 1_000)),
            Err(SettlementError::ConflictingReplay { .. })
        ));
        assert_eq!(book.allocation_count(), 1);
    }

    #[test]
    fn failed_overallocation_leaves_book_unchanged() {
        let ledger = seeded_ledger();
        let mut book = SettlementBook::new();
        book.insert_statement_line(line("line-1", 1_000, "USD"))
            .unwrap();
        assert!(matches!(
            book.allocate(&ledger, allocation("too-large", "line-1", "cash-1", 1_001)),
            Err(SettlementError::AllocationExceedsStatement { .. })
        ));
        assert_eq!(book.allocation_count(), 0);
        assert_eq!(
            book.statement_line_status("line-1").unwrap().allocated_minor_units,
            0
        );
    }

    #[test]
    fn reversal_releases_capacity_without_erasing_allocation_history() {
        let ledger = seeded_ledger();
        let mut book = SettlementBook::new();
        book.insert_statement_line(line("line-1", 5_000, "USD"))
            .unwrap();
        book.allocate(&ledger, allocation("a-1", "line-1", "cash-1", 5_000))
            .unwrap();

        let reversal = AllocationReversal::new(
            "rev-1",
            "a-1",
            "creator-confirmed",
            "Wrong statement-to-receipt association",
            "evidence://correction/1",
        )
        .unwrap();
        book.reverse_allocation(reversal.clone()).unwrap();
        assert_eq!(
            book.reverse_allocation(reversal),
            Ok(SettlementInsertOutcome::IdenticalReplay)
        );
        assert_eq!(book.allocation_count(), 1);
        assert_eq!(book.reversal_count(), 1);
        assert_eq!(
            book.statement_line_status("line-1").unwrap().state,
            ReconciliationState::Unmatched
        );

        book.allocate(&ledger, allocation("a-2", "line-1", "cash-1", 5_000))
            .unwrap();
        assert_eq!(
            book.statement_line_status("line-1").unwrap().state,
            ReconciliationState::FullyMatched
        );
    }

    #[test]
    fn one_allocation_cannot_be_reversed_twice_by_different_events() {
        let mut book = SettlementBook::new();
        book.insert_statement_line(line("line-1", 5_000, "USD"))
            .unwrap();
        let ledger = seeded_ledger();
        book.allocate(&ledger, allocation("a-1", "line-1", "cash-1", 1_000))
            .unwrap();
        book.reverse_allocation(
            AllocationReversal::new(
                "rev-1",
                "a-1",
                "creator",
                "First correction",
                "evidence://reversal/1",
            )
            .unwrap(),
        )
        .unwrap();
        assert_eq!(
            book.reverse_allocation(
                AllocationReversal::new(
                    "rev-2",
                    "a-1",
                    "creator",
                    "Second correction",
                    "evidence://reversal/2",
                )
                .unwrap(),
            ),
            Err(SettlementError::AllocationAlreadyReversed(
                "a-1".to_owned()
            ))
        );
        assert_eq!(book.reversal_count(), 1);
    }
}
