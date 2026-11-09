//! Deterministic creator-revenue stress scenarios.
//!
//! This module calculates user-specified arithmetic "what if" scenarios. It
//! does not assign probabilities, forecast behavior, or mutate a ledger. Costs,
//! refunds, and fees are frozen in v1; results state that assumption explicitly.

use std::collections::BTreeMap;
use std::error::Error;
use std::fmt;

pub const SCENARIO_SCHEMA_VERSION: u16 = 1;
const BASIS_POINTS: i128 = 10_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvidenceCoverage {
    CreatorDeclaredCompleteForWindow,
    PartialOrUnknown,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RevenueSourceSnapshot {
    pub source_id: String,
    pub currency: String,
    /// Non-negative gross receipts observed for the declared window, in minor units.
    pub gross_receipts_minor_units: i128,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CurrencyDeductions {
    pub currency: String,
    /// Refunds/chargebacks assumed to remain unchanged in this scenario.
    pub refunds_minor_units: i128,
    /// Fees assumed to remain unchanged in this scenario.
    pub fees_minor_units: i128,
    /// Operating costs assumed to remain unchanged in this scenario.
    pub operating_costs_minor_units: i128,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SourceDecline {
    pub source_id: String,
    /// Requested decline in gross receipts: 0 = 0%, 10_000 = 100%.
    pub decline_basis_points: u16,
    /// Human-readable rationale, e.g. "primary sponsor pauses for this period".
    pub rationale: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ScenarioInput {
    pub window_start_utc_seconds: i64,
    pub window_end_utc_seconds: i64,
    pub evidence_coverage: EvidenceCoverage,
    pub revenue_sources: Vec<RevenueSourceSnapshot>,
    /// Exactly one explicit deduction record is required for every currency used by a source.
    /// A currency with deductions but no receipts is also permitted.
    pub deductions: Vec<CurrencyDeductions>,
    pub source_declines: Vec<SourceDecline>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ScenarioSemantics {
    /// Deterministic arithmetic using explicit assumptions; not a probability forecast.
    UserSpecifiedWhatIfNotPrediction,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CurrencyScenarioResult {
    pub currency: String,
    pub baseline_gross_receipts_minor_units: i128,
    pub scenario_gross_receipts_minor_units: i128,
    pub assumed_lost_gross_receipts_minor_units: i128,
    pub refunds_minor_units_held_constant: i128,
    pub fees_minor_units_held_constant: i128,
    pub operating_costs_minor_units_held_constant: i128,
    /// Called "observed" because partial input is not silently upgraded to a whole-career total.
    pub baseline_observed_contribution_minor_units: i128,
    pub scenario_observed_contribution_minor_units: i128,
    pub evidence_coverage: EvidenceCoverage,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ScenarioReport {
    pub schema_version: u16,
    pub semantics: ScenarioSemantics,
    pub window_start_utc_seconds: i64,
    pub window_end_utc_seconds: i64,
    pub evidence_coverage: EvidenceCoverage,
    /// True in this deliberately simple v1 model. Variable costs are not inferred.
    pub deductions_held_constant: bool,
    /// Fractional minor-unit loss is rounded up, conservatively, rather than understating impact.
    pub conservative_minor_unit_rounding: bool,
    /// Sorted by source ID so equivalent inputs have a canonical assumption order.
    pub applied_source_declines: Vec<SourceDecline>,
    /// Sorted lexicographically by currency; currencies are never combined.
    pub currencies: Vec<CurrencyScenarioResult>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ScenarioError {
    EmptyBaseline,
    EmptyField(&'static str),
    InvalidCurrency(String),
    NegativeAmount(&'static str),
    InvalidWindow,
    DuplicateSourceId(String),
    DuplicateCurrencyDeductions(String),
    MissingCurrencyDeductions(String),
    DuplicateSourceDecline(String),
    UnknownSourceDecline(String),
    DeclineOutOfRange { source_id: String, basis_points: u16 },
    ArithmeticOverflow,
}

impl fmt::Display for ScenarioError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyBaseline => write!(f, "scenario requires at least one source or deductions record"),
            Self::EmptyField(field) => write!(f, "{field} must not be empty"),
            Self::InvalidCurrency(currency) => write!(
                f,
                "currency must be three uppercase ASCII letters: {currency:?}"
            ),
            Self::NegativeAmount(field) => write!(f, "{field} must not be negative"),
            Self::InvalidWindow => write!(f, "scenario window end must be after its start"),
            Self::DuplicateSourceId(id) => write!(f, "duplicate revenue source identifier: {id:?}"),
            Self::DuplicateCurrencyDeductions(currency) => {
                write!(f, "duplicate deductions record for currency {currency}")
            }
            Self::MissingCurrencyDeductions(currency) => {
                write!(f, "no explicit deductions record for currency {currency}")
            }
            Self::DuplicateSourceDecline(id) => write!(f, "duplicate source decline for {id:?}"),
            Self::UnknownSourceDecline(id) => {
                write!(f, "source decline references unknown source {id:?}")
            }
            Self::DeclineOutOfRange { source_id, basis_points } => write!(
                f,
                "decline for {source_id:?} must be between 0 and {BASIS_POINTS} basis points, got {basis_points}"
            ),
            Self::ArithmeticOverflow => write!(f, "checked scenario arithmetic overflow"),
        }
    }
}

impl Error for ScenarioError {}

#[derive(Default)]
struct CurrencyTotals {
    baseline_gross: i128,
    lost_gross: i128,
}

#[derive(Default)]
struct CurrencyDeductionsOwned {
    refunds: i128,
    fees: i128,
    operating_costs: i128,
}

pub fn run_scenario(input: &ScenarioInput) -> Result<ScenarioReport, ScenarioError> {
    if input.revenue_sources.is_empty() && input.deductions.is_empty() {
        return Err(ScenarioError::EmptyBaseline);
    }
    if input.window_end_utc_seconds <= input.window_start_utc_seconds {
        return Err(ScenarioError::InvalidWindow);
    }

    let mut sources: BTreeMap<String, &RevenueSourceSnapshot> = BTreeMap::new();
    for source in &input.revenue_sources {
        require_non_empty(&source.source_id, "source_id")?;
        if !valid_currency(&source.currency) {
            return Err(ScenarioError::InvalidCurrency(source.currency.clone()));
        }
        if source.gross_receipts_minor_units < 0 {
            return Err(ScenarioError::NegativeAmount("gross_receipts_minor_units"));
        }
        if sources.insert(source.source_id.clone(), source).is_some() {
            return Err(ScenarioError::DuplicateSourceId(source.source_id.clone()));
        }
    }

    let mut deductions: BTreeMap<String, CurrencyDeductionsOwned> = BTreeMap::new();
    for item in &input.deductions {
        if !valid_currency(&item.currency) {
            return Err(ScenarioError::InvalidCurrency(item.currency.clone()));
        }
        if item.refunds_minor_units < 0 {
            return Err(ScenarioError::NegativeAmount("refunds_minor_units"));
        }
        if item.fees_minor_units < 0 {
            return Err(ScenarioError::NegativeAmount("fees_minor_units"));
        }
        if item.operating_costs_minor_units < 0 {
            return Err(ScenarioError::NegativeAmount("operating_costs_minor_units"));
        }
        let currency = item.currency.clone();
        let owned = CurrencyDeductionsOwned {
            refunds: item.refunds_minor_units,
            fees: item.fees_minor_units,
            operating_costs: item.operating_costs_minor_units,
        };
        if deductions.insert(currency.clone(), owned).is_some() {
            return Err(ScenarioError::DuplicateCurrencyDeductions(currency));
        }
    }

    for source in input.revenue_sources.iter() {
        if !deductions.contains_key(&source.currency) {
            return Err(ScenarioError::MissingCurrencyDeductions(source.currency.clone()));
        }
    }

    let mut declines: BTreeMap<String, SourceDecline> = BTreeMap::new();
    for decline in &input.source_declines {
        require_non_empty(&decline.source_id, "decline.source_id")?;
        require_non_empty(&decline.rationale, "decline.rationale")?;
        if decline.decline_basis_points > BASIS_POINTS as u16 {
            return Err(ScenarioError::DeclineOutOfRange {
                source_id: decline.source_id.clone(),
                basis_points: decline.decline_basis_points,
            });
        }
        if !sources.contains_key(&decline.source_id) {
            return Err(ScenarioError::UnknownSourceDecline(decline.source_id.clone()));
        }
        if declines.insert(decline.source_id.clone(), decline.clone()).is_some() {
            return Err(ScenarioError::DuplicateSourceDecline(decline.source_id.clone()));
        }
    }

    let mut totals: BTreeMap<String, CurrencyTotals> = BTreeMap::new();
    for item in &input.deductions {
        totals.entry(item.currency.clone()).or_default();
    }
    for source in &input.revenue_sources {
        let currency_totals = totals.entry(source.currency.clone()).or_default();
        currency_totals.baseline_gross = currency_totals
            .baseline_gross
            .checked_add(source.gross_receipts_minor_units)
            .ok_or(ScenarioError::ArithmeticOverflow)?;
        let basis_points = declines
            .get(&source.source_id)
            .map(|decline| decline.decline_basis_points)
            .unwrap_or(0);
        let lost = conservative_decline_amount(
            source.gross_receipts_minor_units,
            basis_points,
        )?;
        currency_totals.lost_gross = currency_totals
            .lost_gross
            .checked_add(lost)
            .ok_or(ScenarioError::ArithmeticOverflow)?;
    }

    let mut currency_results = Vec::with_capacity(totals.len());
    for (currency, totals_for_currency) in totals {
        let item = deductions
            .get(&currency)
            .ok_or_else(|| ScenarioError::MissingCurrencyDeductions(currency.clone()))?;
        let scenario_gross = totals_for_currency
            .baseline_gross
            .checked_sub(totals_for_currency.lost_gross)
            .ok_or(ScenarioError::ArithmeticOverflow)?;
        let baseline_contribution = contribution(
            totals_for_currency.baseline_gross,
            item.refunds,
            item.fees,
            item.operating_costs,
        )?;
        let scenario_contribution = contribution(
            scenario_gross,
            item.refunds,
            item.fees,
            item.operating_costs,
        )?;
        currency_results.push(CurrencyScenarioResult {
            currency,
            baseline_gross_receipts_minor_units: totals_for_currency.baseline_gross,
            scenario_gross_receipts_minor_units: scenario_gross,
            assumed_lost_gross_receipts_minor_units: totals_for_currency.lost_gross,
            refunds_minor_units_held_constant: item.refunds,
            fees_minor_units_held_constant: item.fees,
            operating_costs_minor_units_held_constant: item.operating_costs,
            baseline_observed_contribution_minor_units: baseline_contribution,
            scenario_observed_contribution_minor_units: scenario_contribution,
            evidence_coverage: input.evidence_coverage,
        });
    }

    let mut applied_source_declines: Vec<_> = declines.into_values().collect();
    applied_source_declines.sort_by(|left, right| left.source_id.cmp(&right.source_id));

    Ok(ScenarioReport {
        schema_version: SCENARIO_SCHEMA_VERSION,
        semantics: ScenarioSemantics::UserSpecifiedWhatIfNotPrediction,
        window_start_utc_seconds: input.window_start_utc_seconds,
        window_end_utc_seconds: input.window_end_utc_seconds,
        evidence_coverage: input.evidence_coverage,
        deductions_held_constant: true,
        conservative_minor_unit_rounding: true,
        applied_source_declines,
        currencies: currency_results,
    })
}

fn contribution(
    gross: i128,
    refunds: i128,
    fees: i128,
    operating_costs: i128,
) -> Result<i128, ScenarioError> {
    gross
        .checked_sub(refunds)
        .and_then(|value| value.checked_sub(fees))
        .and_then(|value| value.checked_sub(operating_costs))
        .ok_or(ScenarioError::ArithmeticOverflow)
}

/// Computes ceil(gross * basis_points / 10_000) without multiplying gross by
/// 10_000 directly. Rounding up prevents fractional minor-unit impact from
/// understating the stressed loss. basis_points must be in 0..=10_000.
fn conservative_decline_amount(gross: i128, basis_points: u16) -> Result<i128, ScenarioError> {
    if gross < 0 || basis_points > BASIS_POINTS as u16 {
        return Err(ScenarioError::NegativeAmount("scenario input"));
    }
    if gross == 0 || basis_points == 0 {
        return Ok(0);
    }
    let whole = gross / BASIS_POINTS;
    let remainder = gross % BASIS_POINTS;
    let whole_loss = whole
        .checked_mul(i128::from(basis_points))
        .ok_or(ScenarioError::ArithmeticOverflow)?;
    let remainder_product = remainder
        .checked_mul(i128::from(basis_points))
        .ok_or(ScenarioError::ArithmeticOverflow)?;
    let remainder_floor = remainder_product / BASIS_POINTS;
    let round_up = if remainder_product % BASIS_POINTS != 0 { 1 } else { 0 };
    let loss = whole_loss
        .checked_add(remainder_floor)
        .and_then(|value| value.checked_add(round_up))
        .ok_or(ScenarioError::ArithmeticOverflow)?;
    Ok(loss.min(gross))
}

fn require_non_empty(value: &str, field: &'static str) -> Result<(), ScenarioError> {
    if value.trim().is_empty() {
        Err(ScenarioError::EmptyField(field))
    } else {
        Ok(())
    }
}

fn valid_currency(value: &str) -> bool {
    value.len() == 3 && value.bytes().all(|byte| byte.is_ascii_uppercase())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn source(id: &str, currency: &str, gross: i128) -> RevenueSourceSnapshot {
        RevenueSourceSnapshot {
            source_id: id.into(),
            currency: currency.into(),
            gross_receipts_minor_units: gross,
        }
    }

    fn deductions(currency: &str, refunds: i128, fees: i128, costs: i128) -> CurrencyDeductions {
        CurrencyDeductions {
            currency: currency.into(),
            refunds_minor_units: refunds,
            fees_minor_units: fees,
            operating_costs_minor_units: costs,
        }
    }

    fn decline(id: &str, bps: u16) -> SourceDecline {
        SourceDecline {
            source_id: id.into(),
            decline_basis_points: bps,
            rationale: "explicit synthetic stress assumption".into(),
        }
    }

    fn input() -> ScenarioInput {
        ScenarioInput {
            window_start_utc_seconds: 1_700_000_000,
            window_end_utc_seconds: 1_702_592_000,
            evidence_coverage: EvidenceCoverage::CreatorDeclaredCompleteForWindow,
            revenue_sources: vec![source("sponsor", "USD", 7_000), source("direct", "USD", 3_000)],
            deductions: vec![deductions("USD", 100, 500, 2_000)],
            source_declines: vec![decline("sponsor", 10_000)],
        }
    }

    #[test]
    fn full_sponsor_loss_is_deterministic_and_currency_scoped() {
        let report = run_scenario(&input()).unwrap();
        assert_eq!(report.semantics, ScenarioSemantics::UserSpecifiedWhatIfNotPrediction);
        assert!(report.deductions_held_constant);
        assert!(report.conservative_minor_unit_rounding);
        let usd = &report.currencies[0];
        assert_eq!(usd.currency, "USD");
        assert_eq!(usd.baseline_gross_receipts_minor_units, 10_000);
        assert_eq!(usd.scenario_gross_receipts_minor_units, 3_000);
        assert_eq!(usd.assumed_lost_gross_receipts_minor_units, 7_000);
        assert_eq!(usd.baseline_observed_contribution_minor_units, 7_400);
        assert_eq!(usd.scenario_observed_contribution_minor_units, 400);
    }

    #[test]
    fn partial_evidence_remains_marked_partial() {
        let mut scenario = input();
        scenario.evidence_coverage = EvidenceCoverage::PartialOrUnknown;
        let report = run_scenario(&scenario).unwrap();
        assert_eq!(report.evidence_coverage, EvidenceCoverage::PartialOrUnknown);
        assert!(report.currencies.iter().all(|item| {
            item.evidence_coverage == EvidenceCoverage::PartialOrUnknown
        }));
    }

    #[test]
    fn mixed_currencies_are_never_aggregated() {
        let mut scenario = input();
        scenario.revenue_sources.push(source("euro-store", "EUR", 5_000));
        scenario.deductions.push(deductions("EUR", 0, 0, 500));
        let report = run_scenario(&scenario).unwrap();
        assert_eq!(report.currencies.len(), 2);
        assert_eq!(report.currencies[0].currency, "EUR");
        assert_eq!(report.currencies[1].currency, "USD");
        assert_eq!(report.currencies[0].baseline_gross_receipts_minor_units, 5_000);
        assert_eq!(report.currencies[1].baseline_gross_receipts_minor_units, 10_000);
    }

    #[test]
    fn higher_gross_receipts_do_not_hide_negative_contribution() {
        let mut scenario = input();
        scenario.deductions = vec![deductions("USD", 500, 500, 15_000)];
        let report = run_scenario(&scenario).unwrap();
        assert!(report.currencies[0].baseline_observed_contribution_minor_units < 0);
        assert!(report.currencies[0].scenario_observed_contribution_minor_units < 0);
    }

    #[test]
    fn unchanged_source_has_no_assumed_loss() {
        let mut scenario = input();
        scenario.source_declines.clear();
        let report = run_scenario(&scenario).unwrap();
        let usd = &report.currencies[0];
        assert_eq!(usd.assumed_lost_gross_receipts_minor_units, 0);
        assert_eq!(
            usd.baseline_observed_contribution_minor_units,
            usd.scenario_observed_contribution_minor_units
        );
    }

    #[test]
    fn conservative_minor_unit_rounding_does_not_understate_declines() {
        assert_eq!(conservative_decline_amount(1, 1).unwrap(), 1);
        assert_eq!(conservative_decline_amount(10_000, 1).unwrap(), 1);
        assert_eq!(conservative_decline_amount(10_000, 5_000).unwrap(), 5_000);
        assert_eq!(conservative_decline_amount(10_000, 10_000).unwrap(), 10_000);
    }

    #[test]
    fn requires_explicit_currency_deductions_even_if_zero() {
        let mut scenario = input();
        scenario.deductions.clear();
        assert_eq!(
            run_scenario(&scenario),
            Err(ScenarioError::MissingCurrencyDeductions("USD".into()))
        );
    }

    #[test]
    fn rejects_duplicate_sources_unknown_sources_and_duplicate_shocks() {
        let mut scenario = input();
        scenario.revenue_sources.push(source("sponsor", "USD", 1));
        assert_eq!(
            run_scenario(&scenario),
            Err(ScenarioError::DuplicateSourceId("sponsor".into()))
        );

        let mut scenario = input();
        scenario.source_declines = vec![decline("missing", 100)];
        assert_eq!(
            run_scenario(&scenario),
            Err(ScenarioError::UnknownSourceDecline("missing".into()))
        );

        let mut scenario = input();
        scenario.source_declines = vec![decline("sponsor", 100), decline("sponsor", 200)];
        assert_eq!(
            run_scenario(&scenario),
            Err(ScenarioError::DuplicateSourceDecline("sponsor".into()))
        );
    }

    #[test]
    fn rejects_bad_decline_rates_currency_and_negative_amounts() {
        let mut scenario = input();
        scenario.source_declines = vec![decline("sponsor", 10_001)];
        assert!(matches!(
            run_scenario(&scenario),
            Err(ScenarioError::DeclineOutOfRange { .. })
        ));

        let mut scenario = input();
        scenario.revenue_sources[0].currency = "usd".into();
        assert!(matches!(
            run_scenario(&scenario),
            Err(ScenarioError::InvalidCurrency(_))
        ));

        let mut scenario = input();
        scenario.revenue_sources[0].gross_receipts_minor_units = -1;
        assert_eq!(
            run_scenario(&scenario),
            Err(ScenarioError::NegativeAmount("gross_receipts_minor_units"))
        );
    }

    #[test]
    fn rejects_ambiguous_cost_snapshots_and_bad_windows() {
        let mut scenario = input();
        scenario.deductions.push(deductions("USD", 0, 0, 0));
        assert_eq!(
            run_scenario(&scenario),
            Err(ScenarioError::DuplicateCurrencyDeductions("USD".into()))
        );

        let mut scenario = input();
        scenario.window_end_utc_seconds = scenario.window_start_utc_seconds;
        assert_eq!(run_scenario(&scenario), Err(ScenarioError::InvalidWindow));
    }

    #[test]
    fn rejects_overflow_in_gross_aggregation_and_contribution() {
        let mut scenario = input();
        scenario.revenue_sources = vec![
            source("a", "USD", i128::MAX),
            source("b", "USD", 1),
        ];
        scenario.deductions = vec![deductions("USD", 0, 0, 0)];
        scenario.source_declines.clear();
        assert_eq!(run_scenario(&scenario), Err(ScenarioError::ArithmeticOverflow));

        let mut scenario = input();
        scenario.deductions = vec![deductions("USD", i128::MAX, i128::MAX, 0)];
        assert_eq!(run_scenario(&scenario), Err(ScenarioError::ArithmeticOverflow));
    }

    #[test]
    fn cost_only_currency_is_reported_without_inventing_receipts() {
        let scenario = ScenarioInput {
            window_start_utc_seconds: 0,
            window_end_utc_seconds: 1,
            evidence_coverage: EvidenceCoverage::PartialOrUnknown,
            revenue_sources: vec![],
            deductions: vec![deductions("USD", 0, 0, 500)],
            source_declines: vec![],
        };
        let report = run_scenario(&scenario).unwrap();
        assert_eq!(report.currencies[0].baseline_gross_receipts_minor_units, 0);
        assert_eq!(report.currencies[0].scenario_gross_receipts_minor_units, 0);
        assert_eq!(report.currencies[0].baseline_observed_contribution_minor_units, -500);
    }
}
