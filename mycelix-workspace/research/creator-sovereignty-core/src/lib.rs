//! Experimental core for a creator-owned career ledger.
//!
//! This crate establishes deterministic accounting semantics before CSV/JSON
//! adapters, platform connectors, databases, or AI recommendations are added.
//! It deliberately has no network or third-party dependencies.

pub mod scenario;

use std::collections::BTreeMap;
use std::error::Error;
use std::fmt;

pub const SCHEMA_VERSION: u16 = 1;
const SECONDS_PER_HOUR: i128 = 3_600;

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LedgerError {
    EmptyField(&'static str),
    InvalidCurrency(String),
    NonPositiveAmount,
    InvalidWorkInterval,
    OverlappingWorkSessions { existing_id: String, new_id: String },
    ConflictingReplay { record_type: &'static str, id: String },
    ArithmeticOverflow,
}

impl fmt::Display for LedgerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyField(name) => write!(f, "{name} must not be empty"),
            Self::InvalidCurrency(value) => write!(
                f,
                "currency must be three uppercase ASCII letters: {value:?}"
            ),
            Self::NonPositiveAmount => {
                write!(f, "amount must be a positive number of minor units")
            }
            Self::InvalidWorkInterval => write!(f, "work session end must be after start"),
            Self::OverlappingWorkSessions { existing_id, new_id } => write!(
                f,
                "work sessions {existing_id:?} and {new_id:?} overlap; the creator-hour denominator would double-count time"
            ),
            Self::ConflictingReplay { record_type, id } => {
                write!(f, "conflicting {record_type} replay for identifier {id:?}")
            }
            Self::ArithmeticOverflow => write!(f, "checked arithmetic overflow"),
        }
    }
}

impl Error for LedgerError {}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InsertOutcome {
    Inserted,
    IdenticalReplay,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MoneyKind {
    Receipt,
    Refund,
    Fee,
    OperatingCost,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MoneyEvent {
    pub event_id: String,
    pub source_id: String,
    /// Unix timestamp in seconds (UTC); source timezone is not inferred.
    pub occurred_at_utc_seconds: i64,
    /// Three-letter uppercase currency code. This validates syntax, not ISO registry membership.
    pub currency: String,
    /// Positive integer minor units; kind supplies the accounting sign.
    pub amount_minor_units: i64,
    pub kind: MoneyKind,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct WorkSession {
    pub session_id: String,
    /// Unix timestamp in seconds (UTC).
    pub start_utc_seconds: i64,
    /// Unix timestamp in seconds (UTC).
    pub end_utc_seconds: i64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RationalRate {
    /// Exact numerator for minor currency units per hour; never silently rounded.
    pub numerator_minor_units_per_hour: i128,
    /// Exact positive denominator in seconds of creator-recorded work.
    pub denominator_work_seconds: u128,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RateUnavailableReason {
    NoRecordedWork,
    MultipleCurrencies,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CurrencySummary {
    pub currency: String,
    pub receipts_minor_units: i128,
    pub refunds_minor_units: i128,
    pub fees_minor_units: i128,
    pub operating_costs_minor_units: i128,
    pub net_contribution_minor_units: i128,
    /// None when the rate is not justified by the recorded denominator.
    pub net_per_work_hour: Option<RationalRate>,
    /// Explicit reasons help consumers avoid treating an unavailable rate as zero.
    pub rate_unavailable_reasons: Vec<RateUnavailableReason>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LedgerSummary {
    pub schema_version: u16,
    pub money_event_count: usize,
    pub work_session_count: usize,
    pub recorded_work_seconds: u128,
    /// Sorted lexicographically by currency. Different currencies are never combined.
    pub currencies: Vec<CurrencySummary>,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct Ledger {
    money_events: BTreeMap<String, MoneyEvent>,
    work_sessions: BTreeMap<String, WorkSession>,
}

impl Ledger {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn money_events(&self) -> impl Iterator<Item = &MoneyEvent> {
        self.money_events.values()
    }

    pub fn work_sessions(&self) -> impl Iterator<Item = &WorkSession> {
        self.work_sessions.values()
    }

    pub fn insert_money_event(&mut self, event: MoneyEvent) -> Result<InsertOutcome, LedgerError> {
        require_non_empty(&event.event_id, "event_id")?;
        require_non_empty(&event.source_id, "source_id")?;
        if !valid_currency(&event.currency) {
            return Err(LedgerError::InvalidCurrency(event.currency));
        }
        if event.amount_minor_units <= 0 {
            return Err(LedgerError::NonPositiveAmount);
        }

        if let Some(existing) = self.money_events.get(&event.event_id) {
            return if existing == &event {
                Ok(InsertOutcome::IdenticalReplay)
            } else {
                Err(LedgerError::ConflictingReplay {
                    record_type: "money event",
                    id: event.event_id,
                })
            };
        }
        self.money_events.insert(event.event_id.clone(), event);
        Ok(InsertOutcome::Inserted)
    }

    pub fn insert_work_session(
        &mut self,
        session: WorkSession,
    ) -> Result<InsertOutcome, LedgerError> {
        require_non_empty(&session.session_id, "session_id")?;
        if session.end_utc_seconds <= session.start_utc_seconds {
            return Err(LedgerError::InvalidWorkInterval);
        }
        session
            .end_utc_seconds
            .checked_sub(session.start_utc_seconds)
            .ok_or(LedgerError::ArithmeticOverflow)?;

        if let Some(existing) = self.work_sessions.get(&session.session_id) {
            return if existing == &session {
                Ok(InsertOutcome::IdenticalReplay)
            } else {
                Err(LedgerError::ConflictingReplay {
                    record_type: "work session",
                    id: session.session_id,
                })
            };
        }

        // Time is the creator's scarce resource. Overlapping intervals must not
        // inflate the denominator. Concurrent activities should be labels on one
        // session rather than separate intervals for this single-creator model.
        for existing in self.work_sessions.values() {
            let overlaps = session.start_utc_seconds < existing.end_utc_seconds
                && existing.start_utc_seconds < session.end_utc_seconds;
            if overlaps {
                return Err(LedgerError::OverlappingWorkSessions {
                    existing_id: existing.session_id.clone(),
                    new_id: session.session_id,
                });
            }
        }

        self.work_sessions.insert(session.session_id.clone(), session);
        Ok(InsertOutcome::Inserted)
    }

    pub fn summarize(&self) -> Result<LedgerSummary, LedgerError> {
        // Fixed slots: receipts, refunds, fees, operating costs.
        let mut grouped: BTreeMap<String, [i128; 4]> = BTreeMap::new();
        for event in self.money_events.values() {
            let totals = grouped.entry(event.currency.clone()).or_insert([0; 4]);
            let slot = match event.kind {
                MoneyKind::Receipt => 0,
                MoneyKind::Refund => 1,
                MoneyKind::Fee => 2,
                MoneyKind::OperatingCost => 3,
            };
            totals[slot] = totals[slot]
                .checked_add(i128::from(event.amount_minor_units))
                .ok_or(LedgerError::ArithmeticOverflow)?;
        }

        let recorded_work_seconds = self.recorded_work_seconds()?;
        let currency_count = grouped.len();
        let mut rate_unavailable_reasons = Vec::new();
        if currency_count > 1 {
            rate_unavailable_reasons.push(RateUnavailableReason::MultipleCurrencies);
        }
        if recorded_work_seconds == 0 {
            rate_unavailable_reasons.push(RateUnavailableReason::NoRecordedWork);
        }

        let mut currencies = Vec::with_capacity(grouped.len());
        for (currency, totals) in grouped {
            let net = totals[0]
                .checked_sub(totals[1])
                .and_then(|value| value.checked_sub(totals[2]))
                .and_then(|value| value.checked_sub(totals[3]))
                .ok_or(LedgerError::ArithmeticOverflow)?;
            let net_per_work_hour = if rate_unavailable_reasons.is_empty() {
                Some(RationalRate {
                    numerator_minor_units_per_hour: net
                        .checked_mul(SECONDS_PER_HOUR)
                        .ok_or(LedgerError::ArithmeticOverflow)?,
                    denominator_work_seconds: recorded_work_seconds,
                })
            } else {
                None
            };
            currencies.push(CurrencySummary {
                currency,
                receipts_minor_units: totals[0],
                refunds_minor_units: totals[1],
                fees_minor_units: totals[2],
                operating_costs_minor_units: totals[3],
                net_contribution_minor_units: net,
                net_per_work_hour,
                rate_unavailable_reasons: rate_unavailable_reasons.clone(),
            });
        }

        Ok(LedgerSummary {
            schema_version: SCHEMA_VERSION,
            money_event_count: self.money_events.len(),
            work_session_count: self.work_sessions.len(),
            recorded_work_seconds,
            currencies,
        })
    }

    pub fn recorded_work_seconds(&self) -> Result<u128, LedgerError> {
        self.work_sessions.values().try_fold(0_u128, |sum, session| {
            let duration = session
                .end_utc_seconds
                .checked_sub(session.start_utc_seconds)
                .ok_or(LedgerError::ArithmeticOverflow)?;
            sum.checked_add(duration as u128)
                .ok_or(LedgerError::ArithmeticOverflow)
        })
    }
}

/// Recompute a claimed summary without calling Ledger::summarize.
///
/// This checker deliberately uses a different aggregation representation and
/// code path. It is an in-process verifier for the prototype, not yet a
/// standalone verifier over a serialized export or cryptographic manifest.
pub fn verify_summary(ledger: &Ledger, claimed: &LedgerSummary) -> Result<(), VerificationError> {
    if claimed.schema_version != SCHEMA_VERSION {
        return Err(VerificationError::SchemaVersion);
    }
    if claimed.money_event_count != ledger.money_events.len()
        || claimed.work_session_count != ledger.work_sessions.len()
    {
        return Err(VerificationError::RecordCount);
    }

    #[derive(Default)]
    struct CheckTotals {
        receipts: i128,
        refunds: i128,
        fees: i128,
        costs: i128,
    }

    let mut by_currency: BTreeMap<String, CheckTotals> = BTreeMap::new();
    for event in ledger.money_events.values() {
        let total = by_currency.entry(event.currency.clone()).or_default();
        let destination = match event.kind {
            MoneyKind::Receipt => &mut total.receipts,
            MoneyKind::Refund => &mut total.refunds,
            MoneyKind::Fee => &mut total.fees,
            MoneyKind::OperatingCost => &mut total.costs,
        };
        *destination = (*destination)
            .checked_add(i128::from(event.amount_minor_units))
            .ok_or(VerificationError::ArithmeticOverflow)?;
    }

    let work_seconds = ledger.work_sessions.values().try_fold(
        0_u128,
        |sum, session| {
            let elapsed = session
                .end_utc_seconds
                .checked_sub(session.start_utc_seconds)
                .ok_or(VerificationError::ArithmeticOverflow)?;
            sum.checked_add(elapsed as u128)
                .ok_or(VerificationError::ArithmeticOverflow)
        },
    )?;
    if claimed.recorded_work_seconds != work_seconds {
        return Err(VerificationError::WorkDuration);
    }
    if claimed.currencies.len() != by_currency.len() {
        return Err(VerificationError::CurrencySet);
    }

    for (reported, (currency, total)) in claimed.currencies.iter().zip(by_currency.iter()) {
        if reported.currency.as_str() != currency.as_str() {
            return Err(VerificationError::CurrencySet);
        }
        let net = total
            .receipts
            .checked_sub(total.refunds)
            .and_then(|v| v.checked_sub(total.fees))
            .and_then(|v| v.checked_sub(total.costs))
            .ok_or(VerificationError::ArithmeticOverflow)?;
        let mut expected_unavailable_reasons = Vec::new();
        if by_currency.len() > 1 {
            expected_unavailable_reasons.push(RateUnavailableReason::MultipleCurrencies);
        }
        if work_seconds == 0 {
            expected_unavailable_reasons.push(RateUnavailableReason::NoRecordedWork);
        }
        let expected_rate = if expected_unavailable_reasons.is_empty() {
            Some(RationalRate {
                numerator_minor_units_per_hour: net
                    .checked_mul(SECONDS_PER_HOUR)
                    .ok_or(VerificationError::ArithmeticOverflow)?,
                denominator_work_seconds: work_seconds,
            })
        } else {
            None
        };

        if reported.receipts_minor_units != total.receipts
            || reported.refunds_minor_units != total.refunds
            || reported.fees_minor_units != total.fees
            || reported.operating_costs_minor_units != total.costs
            || reported.net_contribution_minor_units != net
            || reported.net_per_work_hour != expected_rate
            || reported.rate_unavailable_reasons != expected_unavailable_reasons
        {
            return Err(VerificationError::TotalsMismatch {
                currency: currency.clone(),
            });
        }
    }

    Ok(())
}

fn require_non_empty(value: &str, field: &'static str) -> Result<(), LedgerError> {
    if value.trim().is_empty() {
        Err(LedgerError::EmptyField(field))
    } else {
        Ok(())
    }
}

fn valid_currency(value: &str) -> bool {
    value.len() == 3 && value.bytes().all(|byte| byte.is_ascii_uppercase())
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum VerificationError {
    SchemaVersion,
    RecordCount,
    WorkDuration,
    CurrencySet,
    TotalsMismatch { currency: String },
    ArithmeticOverflow,
}

impl fmt::Display for VerificationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SchemaVersion => write!(f, "unsupported schema version"),
            Self::RecordCount => write!(f, "record count mismatch"),
            Self::WorkDuration => write!(f, "recorded work duration mismatch"),
            Self::CurrencySet => write!(f, "currency set or ordering mismatch"),
            Self::TotalsMismatch { currency } => {
                write!(f, "accounting totals mismatch for {currency}")
            }
            Self::ArithmeticOverflow => {
                write!(f, "checked arithmetic overflow during verification")
            }
        }
    }
}

impl Error for VerificationError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn event(
        id: &str,
        source: &str,
        currency: &str,
        amount: i64,
        kind: MoneyKind,
    ) -> MoneyEvent {
        MoneyEvent {
            event_id: id.into(),
            source_id: source.into(),
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

    #[test]
    fn computes_exact_net_and_exact_work_hour_rate() {
        let mut ledger = Ledger::new();
        ledger.insert_money_event(event("r1", "store", "USD", 10_000, MoneyKind::Receipt)).unwrap();
        ledger.insert_money_event(event("r2", "subs", "USD", 2_000, MoneyKind::Receipt)).unwrap();
        ledger.insert_money_event(event("rf1", "store", "USD", 500, MoneyKind::Refund)).unwrap();
        ledger.insert_money_event(event("f1", "processor", "USD", 500, MoneyKind::Fee)).unwrap();
        ledger.insert_money_event(event("c1", "studio", "USD", 1_000, MoneyKind::OperatingCost)).unwrap();
        ledger.insert_work_session(session("w1", 100, 1_900)).unwrap();
        ledger.insert_work_session(session("w2", 2_000, 3_800)).unwrap();

        let report = ledger.summarize().unwrap();
        let usd = &report.currencies[0];
        assert_eq!(usd.receipts_minor_units, 12_000);
        assert_eq!(usd.refunds_minor_units, 500);
        assert_eq!(usd.fees_minor_units, 500);
        assert_eq!(usd.operating_costs_minor_units, 1_000);
        assert_eq!(usd.net_contribution_minor_units, 10_000);
        assert_eq!(report.recorded_work_seconds, 3_600);
        assert_eq!(usd.net_per_work_hour, Some(RationalRate {
            numerator_minor_units_per_hour: 36_000_000,
            denominator_work_seconds: 3_600,
        }));
        verify_summary(&ledger, &report).unwrap();
    }

    #[test]
    fn different_currencies_are_never_mixed() {
        let mut ledger = Ledger::new();
        ledger.insert_money_event(event("usd", "a", "USD", 10_000, MoneyKind::Receipt)).unwrap();
        ledger.insert_money_event(event("eur", "b", "EUR", 8_000, MoneyKind::Receipt)).unwrap();
        ledger.insert_work_session(session("work", 0, 3_600)).unwrap();
        let report = ledger.summarize().unwrap();
        assert_eq!(
            report.currencies.iter().map(|item| item.currency.as_str()).collect::<Vec<_>>(),
            vec!["EUR", "USD"]
        );
        assert_eq!(report.currencies[0].net_contribution_minor_units, 8_000);
        assert_eq!(report.currencies[1].net_contribution_minor_units, 10_000);
        assert!(report.currencies.iter().all(|summary| summary.net_per_work_hour.is_none()));
        assert!(report.currencies.iter().all(|summary| {
            summary.rate_unavailable_reasons == vec![RateUnavailableReason::MultipleCurrencies]
        }));
        verify_summary(&ledger, &report).unwrap();
    }

    #[test]
    fn missing_work_time_is_unknown_not_zero_productivity() {
        let mut ledger = Ledger::new();
        ledger.insert_money_event(event("r1", "store", "USD", 500, MoneyKind::Receipt)).unwrap();
        let report = ledger.summarize().unwrap();
        assert_eq!(report.recorded_work_seconds, 0);
        assert_eq!(report.currencies[0].net_per_work_hour, None);
        assert_eq!(
            report.currencies[0].rate_unavailable_reasons,
            vec![RateUnavailableReason::NoRecordedWork]
        );
        verify_summary(&ledger, &report).unwrap();
    }

    #[test]
    fn identical_replay_is_idempotent_but_conflicting_replay_fails_closed() {
        let mut ledger = Ledger::new();
        let original = event("r1", "store", "USD", 500, MoneyKind::Receipt);
        assert_eq!(ledger.insert_money_event(original.clone()).unwrap(), InsertOutcome::Inserted);
        assert_eq!(ledger.insert_money_event(original).unwrap(), InsertOutcome::IdenticalReplay);
        assert_eq!(ledger.money_events().count(), 1);
        assert!(matches!(
            ledger.insert_money_event(event("r1", "store", "USD", 700, MoneyKind::Receipt)),
            Err(LedgerError::ConflictingReplay { .. })
        ));
        assert_eq!(ledger.money_events().count(), 1);
    }

    #[test]
    fn rejects_invalid_amount_currency_and_intervals() {
        let mut ledger = Ledger::new();
        assert_eq!(
            ledger.insert_money_event(event("zero", "store", "USD", 0, MoneyKind::Receipt)),
            Err(LedgerError::NonPositiveAmount)
        );
        assert!(matches!(
            ledger.insert_money_event(event("bad-currency", "store", "usd", 100, MoneyKind::Receipt)),
            Err(LedgerError::InvalidCurrency(_))
        ));
        assert_eq!(
            ledger.insert_work_session(session("zero", 10, 10)),
            Err(LedgerError::InvalidWorkInterval)
        );
        assert_eq!(
            ledger.insert_work_session(session("reverse", 11, 10)),
            Err(LedgerError::InvalidWorkInterval)
        );
    }

    #[test]
    fn overlapping_sessions_do_not_double_count_creator_hours() {
        let mut ledger = Ledger::new();
        ledger.insert_work_session(session("w1", 10, 100)).unwrap();
        assert!(matches!(
            ledger.insert_work_session(session("w2", 99, 120)),
            Err(LedgerError::OverlappingWorkSessions { .. })
        ));
        assert_eq!(ledger.recorded_work_seconds().unwrap(), 90);
    }

    #[test]
    fn identical_work_session_replay_is_safe() {
        let mut ledger = Ledger::new();
        let original = session("w1", 10, 100);
        assert_eq!(ledger.insert_work_session(original.clone()).unwrap(), InsertOutcome::Inserted);
        assert_eq!(ledger.insert_work_session(original).unwrap(), InsertOutcome::IdenticalReplay);
        assert_eq!(ledger.recorded_work_seconds().unwrap(), 90);
    }

    #[test]
    fn independent_checker_rejects_tampered_totals_and_record_counts() {
        let mut ledger = Ledger::new();
        ledger.insert_money_event(event("r1", "store", "USD", 500, MoneyKind::Receipt)).unwrap();
        let mut report = ledger.summarize().unwrap();
        report.currencies[0].net_contribution_minor_units += 1;
        assert_eq!(
            verify_summary(&ledger, &report),
            Err(VerificationError::TotalsMismatch { currency: "USD".into() })
        );
        let mut report = ledger.summarize().unwrap();
        report.money_event_count += 1;
        assert_eq!(verify_summary(&ledger, &report), Err(VerificationError::RecordCount));
    }

    #[test]
    fn rejects_report_with_wrong_work_seconds() {
        let mut ledger = Ledger::new();
        ledger.insert_money_event(event("r1", "store", "USD", 360, MoneyKind::Receipt)).unwrap();
        ledger.insert_work_session(session("w1", 0, 3_600)).unwrap();
        let mut report = ledger.summarize().unwrap();
        report.recorded_work_seconds += 1;
        assert_eq!(verify_summary(&ledger, &report), Err(VerificationError::WorkDuration));
    }

    #[test]
    fn preserves_negative_net_without_saturation() {
        let mut ledger = Ledger::new();
        ledger.insert_money_event(event("r1", "store", "USD", 100, MoneyKind::Receipt)).unwrap();
        ledger.insert_money_event(event("c1", "studio", "USD", 250, MoneyKind::OperatingCost)).unwrap();
        let report = ledger.summarize().unwrap();
        assert_eq!(report.currencies[0].net_contribution_minor_units, -150);
        verify_summary(&ledger, &report).unwrap();
    }
}
