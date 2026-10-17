use creator_sovereignty_core::{
    InsertOutcome, Ledger, LedgerError, LedgerSummary, MoneyEvent, MoneyKind, RateUnavailableReason,
    RationalRate, VerificationError, WorkSession, verify_summary,
};

fn event(id: &str, currency: &str, amount: i64, kind: MoneyKind) -> MoneyEvent {
    MoneyEvent {
        event_id: id.into(),
        source_id: "synthetic-fixture".into(),
        occurred_at_utc_seconds: 1_700_000_000,
        currency: currency.into(),
        amount_minor_units: amount,
        kind,
    }
}

fn session(id: &str, start: i64, end: i64) -> WorkSession {
    WorkSession {
        session_id: id.into(),
        start_utc_seconds: start,
        end_utc_seconds: end,
    }
}

fn standard_ledger() -> Ledger {
    let mut ledger = Ledger::new();
    ledger
        .insert_money_event(event("receipt", "USD", 10_000, MoneyKind::Receipt))
        .unwrap();
    ledger
        .insert_money_event(event("fee", "USD", 500, MoneyKind::Fee))
        .unwrap();
    ledger
        .insert_money_event(event("cost", "USD", 1_000, MoneyKind::OperatingCost))
        .unwrap();
    ledger.insert_work_session(session("work", 0, 3_600)).unwrap();
    ledger
}

fn rate_is_lower(left: &RationalRate, right: &RationalRate) -> bool {
    // Compare exact rational values without floating-point rounding. These test
    // fixtures are intentionally small enough for cross products to fit i128.
    (left.numerator_minor_units_per_hour * (right.denominator_work_seconds as i128))
        < (right.numerator_minor_units_per_hour * (left.denominator_work_seconds as i128))
}

#[test]
fn higher_gross_revenue_can_still_mean_worse_economics_per_hour() {
    let mut baseline = Ledger::new();
    baseline
        .insert_money_event(event("receipt", "USD", 10_000, MoneyKind::Receipt))
        .unwrap();
    baseline
        .insert_money_event(event("cost", "USD", 2_000, MoneyKind::OperatingCost))
        .unwrap();
    baseline.insert_work_session(session("work", 0, 3_600)).unwrap();

    let mut growth_scenario = Ledger::new();
    growth_scenario
        .insert_money_event(event("receipt", "USD", 12_000, MoneyKind::Receipt))
        .unwrap();
    growth_scenario
        .insert_money_event(event("cost", "USD", 7_000, MoneyKind::OperatingCost))
        .unwrap();
    growth_scenario
        .insert_work_session(session("work", 0, 7_200))
        .unwrap();

    let baseline_report = baseline.summarize().unwrap();
    let scenario_report = growth_scenario.summarize().unwrap();

    assert_eq!(baseline_report.currencies[0].receipts_minor_units, 10_000);
    assert_eq!(scenario_report.currencies[0].receipts_minor_units, 12_000);
    assert_eq!(baseline_report.currencies[0].net_contribution_minor_units, 8_000);
    assert_eq!(scenario_report.currencies[0].net_contribution_minor_units, 5_000);

    let baseline_rate = baseline_report.currencies[0].net_per_work_hour.as_ref().unwrap();
    let scenario_rate = scenario_report.currencies[0].net_per_work_hour.as_ref().unwrap();
    assert!(rate_is_lower(scenario_rate, baseline_rate));
}

#[test]
fn insertion_order_does_not_change_canonical_summary() {
    let events = [
        event("receipt-b", "USD", 200, MoneyKind::Receipt),
        event("receipt-a", "USD", 500, MoneyKind::Receipt),
        event("refund", "USD", 100, MoneyKind::Refund),
        event("fee", "USD", 25, MoneyKind::Fee),
        event("cost", "EUR", 50, MoneyKind::OperatingCost),
    ];
    let mut first = Ledger::new();
    for item in events.clone() {
        first.insert_money_event(item).unwrap();
    }
    let mut second = Ledger::new();
    for item in events.into_iter().rev() {
        second.insert_money_event(item).unwrap();
    }

    first.insert_work_session(session("w1", 0, 1_000)).unwrap();
    first.insert_work_session(session("w2", 1_000, 2_000)).unwrap();
    second.insert_work_session(session("w2", 1_000, 2_000)).unwrap();
    second.insert_work_session(session("w1", 0, 1_000)).unwrap();

    assert_eq!(first.summarize().unwrap(), second.summarize().unwrap());
}

#[test]
fn touching_work_intervals_are_not_overlapping() {
    let mut ledger = Ledger::new();
    ledger.insert_work_session(session("morning", 0, 3_600)).unwrap();
    ledger
        .insert_work_session(session("afternoon", 3_600, 7_200))
        .unwrap();
    assert_eq!(ledger.recorded_work_seconds().unwrap(), 7_200);
}

#[test]
fn end_to_end_overflowing_interval_fails_closed() {
    let mut ledger = Ledger::new();
    assert_eq!(
        ledger.insert_work_session(session("overflow", i64::MIN, i64::MAX)),
        Err(LedgerError::ArithmeticOverflow)
    );
}

#[test]
fn conflicting_event_source_cannot_be_silently_rewritten() {
    let mut ledger = Ledger::new();
    let original = event("same-id", "USD", 500, MoneyKind::Receipt);
    ledger.insert_money_event(original).unwrap();

    let mut conflicting = event("same-id", "USD", 500, MoneyKind::Receipt);
    conflicting.source_id = "different-source".into();
    assert!(matches!(
        ledger.insert_money_event(conflicting),
        Err(LedgerError::ConflictingReplay { .. })
    ));
    assert_eq!(ledger.money_events().count(), 1);
}

fn assert_tampered_summary_rejected(
    ledger: &Ledger,
    mutate: impl FnOnce(&mut LedgerSummary),
) {
    let mut report = ledger.summarize().unwrap();
    mutate(&mut report);
    assert!(
        verify_summary(ledger, &report).is_err(),
        "independent checker accepted a tampered summary"
    );
}

#[test]
fn independent_checker_rejects_each_material_currency_summary_tamper() {
    let ledger = standard_ledger();

    assert_tampered_summary_rejected(&ledger, |r| r.currencies[0].currency = "EUR".into());
    assert_tampered_summary_rejected(&ledger, |r| r.currencies[0].receipts_minor_units += 1);
    assert_tampered_summary_rejected(&ledger, |r| r.currencies[0].refunds_minor_units += 1);
    assert_tampered_summary_rejected(&ledger, |r| r.currencies[0].fees_minor_units += 1);
    assert_tampered_summary_rejected(&ledger, |r| r.currencies[0].operating_costs_minor_units += 1);
    assert_tampered_summary_rejected(&ledger, |r| r.currencies[0].net_contribution_minor_units += 1);
    assert_tampered_summary_rejected(&ledger, |r| {
        r.currencies[0]
            .net_per_work_hour
            .as_mut()
            .unwrap()
            .numerator_minor_units_per_hour += 1
    });
    assert_tampered_summary_rejected(&ledger, |r| {
        r.currencies[0]
            .net_per_work_hour
            .as_mut()
            .unwrap()
            .denominator_work_seconds += 1
    });
    assert_tampered_summary_rejected(&ledger, |r| r.schema_version += 1);
}

#[test]
fn identical_replays_never_create_extra_gross_revenue() {
    let mut ledger = Ledger::new();
    let receipt = event("receipt", "USD", 900, MoneyKind::Receipt);
    assert_eq!(
        ledger.insert_money_event(receipt.clone()).unwrap(),
        InsertOutcome::Inserted
    );
    for _ in 0..100 {
        assert_eq!(
            ledger.insert_money_event(receipt.clone()).unwrap(),
            InsertOutcome::IdenticalReplay
        );
    }
    assert_eq!(ledger.money_events().count(), 1);
    assert_eq!(
        ledger.summarize().unwrap().currencies[0].receipts_minor_units,
        900
    );
}

#[test]
fn different_currencies_are_not_collapsed_to_one_total() {
    let mut ledger = Ledger::new();
    ledger
        .insert_money_event(event("usd", "USD", 1_000, MoneyKind::Receipt))
        .unwrap();
    ledger
        .insert_money_event(event("eur", "EUR", 1_000, MoneyKind::Receipt))
        .unwrap();
    ledger.insert_work_session(session("work", 0, 3_600)).unwrap();

    let report = ledger.summarize().unwrap();
    assert_eq!(report.currencies.len(), 2);
    assert_eq!(report.currencies[0].currency, "EUR");
    assert_eq!(report.currencies[1].currency, "USD");
    assert_ne!(
        report.currencies[0].currency,
        report.currencies[1].currency
    );
    assert!(report.currencies.iter().all(|summary| summary.net_per_work_hour.is_none()));
    assert!(report.currencies.iter().all(|summary| {
        summary.rate_unavailable_reasons
            == vec![RateUnavailableReason::MultipleCurrencies]
    }));
    verify_summary(&ledger, &report).unwrap();
}

#[test]
fn independent_checker_rejects_unjustified_cross_currency_hourly_rates() {
    let mut ledger = Ledger::new();
    ledger
        .insert_money_event(event("usd", "USD", 1_000, MoneyKind::Receipt))
        .unwrap();
    ledger
        .insert_money_event(event("eur", "EUR", 1_000, MoneyKind::Receipt))
        .unwrap();
    ledger.insert_work_session(session("work", 0, 3_600)).unwrap();

    let mut report = ledger.summarize().unwrap();
    let recorded_work_seconds = report.recorded_work_seconds;
    for summary in &mut report.currencies {
        summary.net_per_work_hour = Some(RationalRate {
            numerator_minor_units_per_hour: summary.net_contribution_minor_units * 3_600,
            denominator_work_seconds: recorded_work_seconds,
        });
        summary.rate_unavailable_reasons.clear();
    }

    assert!(matches!(
        verify_summary(&ledger, &report),
        Err(VerificationError::TotalsMismatch { .. })
    ));
}
