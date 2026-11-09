use creator_sovereignty_core::settlement::{
    AllocationReversal, ReconciliationAllocation, ReconciliationState, RoyaltyStatementLine,
    SettlementBook, SettlementError, SettlementInsertOutcome,
};
use creator_sovereignty_core::{InsertOutcome, Ledger, LedgerError, MoneyEvent, MoneyKind};

fn statement_line(
    line_id: &str,
    statement_id: &str,
    amount_minor_units: i64,
    currency: &str,
) -> RoyaltyStatementLine {
    RoyaltyStatementLine::new(
        line_id,
        statement_id,
        "distributor-a",
        "fixture-payable-normalization",
        "1",
        "evidence://source/statement-line",
        1_700_000_000,
        1_702_000_000,
        currency,
        amount_minor_units,
    )
    .unwrap()
}

fn event(id: &str, amount_minor_units: i64, currency: &str, kind: MoneyKind) -> MoneyEvent {
    MoneyEvent {
        event_id: id.to_owned(),
        source_id: "creator-bank-export".to_owned(),
        occurred_at_utc_seconds: 1_702_000_100,
        currency: currency.to_owned(),
        amount_minor_units,
        kind,
    }
}

fn allocation(
    id: &str,
    line_id: &str,
    receipt_id: &str,
    amount_minor_units: i64,
) -> ReconciliationAllocation {
    ReconciliationAllocation::new(
        id,
        line_id,
        receipt_id,
        amount_minor_units,
        "creator-approved",
        "evidence://manual-review/receipt-match",
    )
    .unwrap()
}

fn ledger_with_receipt(id: &str, amount_minor_units: i64, currency: &str) -> Ledger {
    let mut ledger = Ledger::new();
    ledger
        .insert_money_event(event(
            id,
            amount_minor_units,
            currency,
            MoneyKind::Receipt,
        ))
        .unwrap();
    ledger
}

#[test]
fn equal_amounts_do_not_create_an_automatic_match() {
    let ledger = ledger_with_receipt("bank-tx-1", 5_000, "USD");
    let mut book = SettlementBook::new();
    book.insert_statement_line(statement_line("royalty-1", "statement-1", 5_000, "USD"))
        .unwrap();

    // Equality of amount/currency alone is ambiguous. No matching algorithm runs
    // until an explicit allocation with approver and evidence reference is added.
    let status = book.statement_line_status("royalty-1").unwrap();
    assert_eq!(status.state, ReconciliationState::Unmatched);
    assert_eq!(status.outstanding_minor_units, 5_000);
    assert_eq!(book.allocation_count(), 0);
    assert_eq!(ledger.summarize().unwrap().money_event_count, 1);
}

#[test]
fn reporting_a_royalty_does_not_increase_observed_cash() {
    let ledger = Ledger::new();
    let before = ledger.summarize().unwrap();
    let mut book = SettlementBook::new();
    book.insert_statement_line(statement_line("royalty-1", "statement-1", 9_999, "USD"))
        .unwrap();

    assert_eq!(ledger.summarize().unwrap(), before);
    assert_eq!(ledger.summarize().unwrap().money_event_count, 0);
    assert_eq!(book.statement_line_count(), 1);
}

#[test]
fn statement_amount_can_be_matched_in_parts_without_floating_point() {
    let ledger = ledger_with_receipt("bank-tx-1", 6_250, "USD");
    let mut book = SettlementBook::new();
    book.insert_statement_line(statement_line("royalty-1", "statement-1", 10_000, "USD"))
        .unwrap();

    book.allocate(&ledger, allocation("alloc-1", "royalty-1", "bank-tx-1", 2_500))
        .unwrap();
    book.allocate(&ledger, allocation("alloc-2", "royalty-1", "bank-tx-1", 3_750))
        .unwrap();

    let status = book.statement_line_status("royalty-1").unwrap();
    assert_eq!(status.allocated_minor_units, 6_250);
    assert_eq!(status.outstanding_minor_units, 3_750);
    assert_eq!(status.state, ReconciliationState::PartiallyMatched);
    assert_eq!(
        book.receipt_status(&ledger, "bank-tx-1")
            .unwrap()
            .unallocated_minor_units,
        0
    );
}

#[test]
fn statement_line_and_receipt_caps_are_enforced_atomically() {
    let ledger = ledger_with_receipt("bank-tx-1", 4_000, "USD");
    let mut book = SettlementBook::new();
    book.insert_statement_line(statement_line("royalty-1", "statement-1", 3_000, "USD"))
        .unwrap();

    book.allocate(&ledger, allocation("alloc-1", "royalty-1", "bank-tx-1", 2_000))
        .unwrap();
    let before = book.clone();
    assert!(matches!(
        book.allocate(&ledger, allocation("alloc-too-much", "royalty-1", "bank-tx-1", 1_001)),
        Err(SettlementError::AllocationExceedsStatement { .. })
    ));
    assert_eq!(book, before);
}

#[test]
fn one_cash_receipt_cannot_be_allocated_more_than_once_in_total() {
    let ledger = ledger_with_receipt("bank-tx-1", 5_000, "USD");
    let mut book = SettlementBook::new();
    book.insert_statement_line(statement_line("royalty-a", "statement-a", 3_000, "USD"))
        .unwrap();
    book.insert_statement_line(statement_line("royalty-b", "statement-b", 3_000, "USD"))
        .unwrap();

    book.allocate(&ledger, allocation("alloc-a", "royalty-a", "bank-tx-1", 3_000))
        .unwrap();
    let before = book.clone();
    assert_eq!(
        book.allocate(&ledger, allocation("alloc-b", "royalty-b", "bank-tx-1", 2_001)),
        Err(SettlementError::AllocationExceedsReceipt {
            receipt_event_id: "bank-tx-1".to_owned(),
            requested_minor_units: 2_001,
            remaining_minor_units: 2_000,
        })
    );
    assert_eq!(book, before);
}

#[test]
fn cross_currency_matches_are_never_converted_or_accepted() {
    let ledger = ledger_with_receipt("bank-tx-zar", 10_000, "ZAR");
    let mut book = SettlementBook::new();
    book.insert_statement_line(statement_line("royalty-usd", "statement-1", 10_000, "USD"))
        .unwrap();

    assert_eq!(
        book.allocate(
            &ledger,
            allocation("alloc-1", "royalty-usd", "bank-tx-zar", 10_000)
        ),
        Err(SettlementError::CurrencyMismatch {
            statement_currency: "USD".to_owned(),
            receipt_currency: "ZAR".to_owned(),
        })
    );
    assert_eq!(book.allocation_count(), 0);
}

#[test]
fn only_money_events_classified_as_receipts_can_back_a_match() {
    let mut ledger = Ledger::new();
    ledger
        .insert_money_event(event("royalty-fee", 500, "USD", MoneyKind::Fee))
        .unwrap();
    ledger
        .insert_money_event(event(
            "royalty-cost",
            700,
            "USD",
            MoneyKind::OperatingCost,
        ))
        .unwrap();

    let mut book = SettlementBook::new();
    book.insert_statement_line(statement_line("royalty-1", "statement-1", 500, "USD"))
        .unwrap();
    assert_eq!(
        book.allocate(
            &ledger,
            allocation("alloc-1", "royalty-1", "royalty-fee", 500)
        ),
        Err(SettlementError::ReceiptNotActualCashReceipt(
            "royalty-fee".to_owned()
        ))
    );
    assert_eq!(
        book.allocate(
            &ledger,
            allocation("alloc-2", "royalty-1", "royalty-cost", 500)
        ),
        Err(SettlementError::ReceiptNotActualCashReceipt(
            "royalty-cost".to_owned()
        ))
    );
    assert_eq!(book.allocation_count(), 0);
}

#[test]
fn replayed_statement_and_allocation_ids_are_idempotent_but_not_mutable() {
    let ledger = ledger_with_receipt("bank-tx-1", 10_000, "USD");
    let original_line = statement_line("royalty-1", "statement-1", 7_000, "USD");
    let mut book = SettlementBook::new();

    assert_eq!(
        book.insert_statement_line(original_line.clone()),
        Ok(SettlementInsertOutcome::Inserted)
    );
    assert_eq!(
        book.insert_statement_line(original_line),
        Ok(SettlementInsertOutcome::IdenticalReplay)
    );
    assert!(matches!(
        book.insert_statement_line(statement_line("royalty-1", "statement-2", 7_000, "USD")),
        Err(SettlementError::ConflictingReplay { .. })
    ));

    let original_allocation = allocation("alloc-1", "royalty-1", "bank-tx-1", 4_000);
    book.allocate(&ledger, original_allocation.clone()).unwrap();
    assert_eq!(
        book.allocate(&ledger, original_allocation),
        Ok(SettlementInsertOutcome::IdenticalReplay)
    );
    assert!(matches!(
        book.allocate(&ledger, allocation("alloc-1", "royalty-1", "bank-tx-1", 3_000)),
        Err(SettlementError::ConflictingReplay { .. })
    ));
    assert_eq!(book.allocation_count(), 1);
}

#[test]
fn reversal_keeps_audit_history_and_releases_the_allocation_capacity() {
    let ledger = ledger_with_receipt("bank-tx-1", 6_000, "USD");
    let mut book = SettlementBook::new();
    book.insert_statement_line(statement_line("royalty-1", "statement-1", 6_000, "USD"))
        .unwrap();
    book.allocate(&ledger, allocation("alloc-1", "royalty-1", "bank-tx-1", 6_000))
        .unwrap();
    let reversal = AllocationReversal::new(
        "reverse-1",
        "alloc-1",
        "creator-approved",
        "Incorrect link selected during manual review",
        "evidence://correction/1",
    )
    .unwrap();

    book.reverse_allocation(reversal.clone()).unwrap();
    assert_eq!(
        book.reverse_allocation(reversal),
        Ok(SettlementInsertOutcome::IdenticalReplay)
    );
    assert_eq!(book.allocations().count(), 1);
    assert_eq!(book.reversals().count(), 1);
    assert_eq!(
        book.statement_line_status("royalty-1").unwrap().state,
        ReconciliationState::Unmatched
    );
    book.allocate(&ledger, allocation("alloc-2", "royalty-1", "bank-tx-1", 6_000))
        .unwrap();
    assert_eq!(
        book.statement_line_status("royalty-1").unwrap().state,
        ReconciliationState::FullyMatched
    );
}

#[test]
fn normalized_payable_line_retains_semantic_mapping_and_source_evidence() {
    let line = statement_line("royalty-1", "statement-1", 1_250, "USD");
    assert_eq!(
        line.semantic_mapping_rule_id(),
        "fixture-payable-normalization"
    );
    assert_eq!(line.semantic_mapping_version(), "1");
    assert_eq!(
        line.source_evidence_ref(),
        "evidence://source/statement-line"
    );
}

#[test]
fn payable_line_requires_mapping_version_and_source_evidence() {
    assert_eq!(
        RoyaltyStatementLine::new(
            "x",
            "statement",
            "source",
            "payable-normalization",
            " ",
            "evidence://source/line",
            100,
            200,
            "USD",
            100,
        ),
        Err(SettlementError::EmptyField("semantic_mapping_version"))
    );
    assert_eq!(
        RoyaltyStatementLine::new(
            "x",
            "statement",
            "source",
            "payable-normalization",
            "1",
            " ",
            100,
            200,
            "USD",
            100,
        ),
        Err(SettlementError::EmptyField("source_evidence_ref"))
    );
}

#[test]
fn invalid_report_fields_and_missing_cash_fail_closed() {
    assert_eq!(
        RoyaltyStatementLine::new(
            "x",
            "statement",
            "source",
            "manual-entry",
            "1",
            "evidence://creator-input",
            100,
            100,
            "USD",
            1,
        ),
        Err(SettlementError::InvalidReportingPeriod)
    );
    assert!(matches!(
        RoyaltyStatementLine::new(
            "x",
            "statement",
            "source",
            "manual-entry",
            "1",
            "evidence://creator-input",
            100,
            200,
            "usd",
            1,
        ),
        Err(SettlementError::InvalidCurrency(_))
    ));
    assert_eq!(
        ReconciliationAllocation::new("alloc", "line", "receipt", 1, "creator", "  "),
        Err(SettlementError::EmptyField("evidence_ref"))
    );

    let ledger = Ledger::new();
    let mut book = SettlementBook::new();
    book.insert_statement_line(statement_line("royalty-1", "statement-1", 100, "USD"))
        .unwrap();
    assert_eq!(
        book.allocate(&ledger, allocation("alloc-1", "royalty-1", "missing-cash", 100)),
        Err(SettlementError::ReceiptNotFound("missing-cash".to_owned()))
    );
    assert_eq!(book.allocation_count(), 0);
}
