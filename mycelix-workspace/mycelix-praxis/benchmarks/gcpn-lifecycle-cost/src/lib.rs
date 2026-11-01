//! Synthetic lifecycle cost and accessible-reach evaluation.
//! All cost amounts are invented units, not currency, procurement estimates or budgets.

use std::collections::{BTreeMap, HashSet};

pub const COST_FIXTURE: &str = include_str!("../fixtures/lifecycle-cost-profile-v0.1.csv");
pub const OUTCOME_FIXTURE: &str = include_str!("../fixtures/service-outcomes-v0.1.csv");

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Architecture {
    Centralized,
    Regional,
    LocalFirst,
}

impl Architecture {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "centralized" => Ok(Self::Centralized),
            "regional" => Ok(Self::Regional),
            "local-first" => Ok(Self::LocalFirst),
            _ => Err(format!("invalid architecture: {value}")),
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Centralized => "centralized",
            Self::Regional => "regional",
            Self::LocalFirst => "local-first",
        }
    }

    pub const ALL: [Self; 3] = [Self::Centralized, Self::Regional, Self::LocalFirst];
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CostPattern {
    OneTime,
    Annual,
    Scheduled,
    ScenarioRestoration,
}

impl CostPattern {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "one_time" => Ok(Self::OneTime),
            "annual" => Ok(Self::Annual),
            "scheduled" => Ok(Self::Scheduled),
            "scenario_restoration" => Ok(Self::ScenarioRestoration),
            _ => Err(format!("invalid cost_pattern: {value}")),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct CostRow {
    pub cost_id: String,
    pub architecture: Architecture,
    /// "all" applies to each scenario for this architecture.
    pub scenario_id: String,
    pub component: String,
    pub pattern: CostPattern,
    pub cost_year: u32,
    pub service_horizon_years: u32,
    pub discount_rate: f64,
    pub low_cost: f64,
    pub base_cost: f64,
    pub high_cost: f64,
    pub annual_staff_hours: f64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ServiceOutcome {
    pub architecture: Architecture,
    pub scenario_id: String,
    pub intended_receivers: u64,
    pub unique_receivers_reached: u64,
    pub accessible_receivers_reached: u64,
    pub duplicate_channel_receipts: u64,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct ComponentCost {
    pub component: String,
    pub low_present_value: f64,
    pub base_present_value: f64,
    pub high_present_value: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CostEvaluation {
    pub architecture: Architecture,
    pub scenario_id: String,
    pub service_horizon_years: u32,
    pub discount_rate: f64,
    pub intended_receivers: u64,
    pub unique_receivers_reached: u64,
    pub accessible_receivers_reached: u64,
    pub duplicate_channel_receipts: u64,
    pub low_lifecycle_present_value: f64,
    pub base_lifecycle_present_value: f64,
    pub high_lifecycle_present_value: f64,
    pub base_annualized_cost: f64,
    pub base_cost_per_intended_receiver: Option<f64>,
    pub base_cost_per_unique_receiver_reached: Option<f64>,
    pub base_cost_per_accessible_receiver_reached: Option<f64>,
    pub annual_staff_hours: f64,
    pub components: Vec<ComponentCost>,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct CostReport {
    pub cost_record_count: u32,
    pub service_outcome_count: u32,
    pub scenario_count: u32,
    pub architecture_count: u32,
    pub cost_ranges_monotonic: bool,
    pub service_denominators_consistent: bool,
    pub outcome_counts_valid: bool,
    pub all_architectures_have_profiles: bool,
    pub evaluations: Vec<CostEvaluation>,
}

impl CostReport {
    pub fn fixture_oracle_matches(&self) -> bool {
        if self.cost_record_count != 24
            || self.service_outcome_count != 21
            || self.scenario_count != 7
            || self.architecture_count != 3
            || !self.cost_ranges_monotonic
            || !self.service_denominators_consistent
            || !self.outcome_counts_valid
            || !self.all_architectures_have_profiles
            || self.evaluations.len() != 21
        {
            return false;
        }

        let base = |architecture: Architecture, scenario: &str| {
            self.evaluations.iter().find(|evaluation| {
                evaluation.architecture == architecture && evaluation.scenario_id == scenario
            })
        };

        match (
            base(Architecture::Centralized, "all-available"),
            base(Architecture::Regional, "all-available"),
            base(Architecture::LocalFirst, "all-available"),
            base(Architecture::Centralized, "origin-authority-unavailable"),
            base(Architecture::Centralized, "interregional-partition"),
            base(Architecture::Regional, "interregional-partition"),
            base(Architecture::LocalFirst, "interregional-partition"),
        ) {
            (Some(c), Some(r), Some(l), Some(zero), Some(cp), Some(rp), Some(lp)) => {
                (c.base_lifecycle_present_value - 28_026.378583683407).abs() < 1e-6
                    && (r.base_lifecycle_present_value - 31_443.190239418658).abs() < 1e-6
                    && (l.base_lifecycle_present_value - 38_847.468928902585).abs() < 1e-6
                    && c.accessible_receivers_reached == 1_800
                    && r.accessible_receivers_reached == 1_850
                    && l.accessible_receivers_reached == 1_900
                    && zero.accessible_receivers_reached == 0
                    && zero.base_cost_per_accessible_receiver_reached.is_none()
                    && cp.accessible_receivers_reached == 1_800
                    && rp.accessible_receivers_reached == 900
                    && lp.accessible_receivers_reached == 950
                    && c.low_lifecycle_present_value <= c.base_lifecycle_present_value
                    && c.base_lifecycle_present_value <= c.high_lifecycle_present_value
            }
            _ => false,
        }
    }

    pub fn evaluation(
        &self,
        architecture: Architecture,
        scenario_id: &str,
    ) -> Option<&CostEvaluation> {
        self.evaluations.iter().find(|evaluation| {
            evaluation.architecture == architecture && evaluation.scenario_id == scenario_id
        })
    }
}

fn parse_u32(value: &str, field: &str) -> Result<u32, String> {
    value
        .parse()
        .map_err(|_| format!("{field} must be an unsigned 32-bit integer, got {value}"))
}

fn parse_u64(value: &str, field: &str) -> Result<u64, String> {
    value
        .parse()
        .map_err(|_| format!("{field} must be an unsigned 64-bit integer, got {value}"))
}

fn parse_f64(value: &str, field: &str) -> Result<f64, String> {
    let parsed = value
        .parse::<f64>()
        .map_err(|_| format!("{field} must be numeric, got {value}"))?;
    if !parsed.is_finite() {
        return Err(format!("{field} must be finite"));
    }
    Ok(parsed)
}

pub fn parse_cost_fixture(input: &str) -> Result<Vec<CostRow>, String> {
    const HEADER: &str = "cost_id,architecture,scenario_id,component,cost_pattern,cost_year,service_horizon_years,discount_rate,low_cost,base_cost,high_cost,annual_staff_hours";
    let mut lines = input.lines();
    if lines.next() != Some(HEADER) {
        return Err("lifecycle cost fixture header does not match frozen v0.1 contract".into());
    }

    let mut rows = Vec::new();
    let mut ids = HashSet::new();
    let mut profiles: BTreeMap<Architecture, (u32, f64)> = BTreeMap::new();

    for (line_idx, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        if f.len() != 12 {
            return Err(format!("line {}: expected 12 fields, got {}", line_idx + 2, f.len()));
        }

        let row = CostRow {
            cost_id: f[0].to_owned(),
            architecture: Architecture::parse(f[1])?,
            scenario_id: f[2].to_owned(),
            component: f[3].to_owned(),
            pattern: CostPattern::parse(f[4])?,
            cost_year: parse_u32(f[5], "cost_year")?,
            service_horizon_years: parse_u32(f[6], "service_horizon_years")?,
            discount_rate: parse_f64(f[7], "discount_rate")?,
            low_cost: parse_f64(f[8], "low_cost")?,
            base_cost: parse_f64(f[9], "base_cost")?,
            high_cost: parse_f64(f[10], "high_cost")?,
            annual_staff_hours: parse_f64(f[11], "annual_staff_hours")?,
        };

        if row.cost_id.trim().is_empty()
            || row.scenario_id.trim().is_empty()
            || row.component.trim().is_empty()
        {
            return Err(format!("line {}: cost ID, scenario ID and component are required", line_idx + 2));
        }
        if !ids.insert(row.cost_id.clone()) {
            return Err(format!("line {}: duplicate cost_id", line_idx + 2));
        }
        if row.service_horizon_years == 0 {
            return Err(format!("line {}: service horizon must be non-zero", line_idx + 2));
        }
        if !(0.0..=1.0).contains(&row.discount_rate) {
            return Err(format!("line {}: discount rate must be within [0,1]", line_idx + 2));
        }
        if row.low_cost < 0.0
            || row.base_cost < 0.0
            || row.high_cost < 0.0
            || row.low_cost > row.base_cost
            || row.base_cost > row.high_cost
        {
            return Err(format!("line {}: costs must be non-negative and low <= base <= high", line_idx + 2));
        }
        if row.annual_staff_hours < 0.0 {
            return Err(format!("line {}: staff hours must be non-negative", line_idx + 2));
        }

        match row.pattern {
            CostPattern::OneTime if row.cost_year != 0 => {
                return Err(format!("line {}: one_time cost must be at year zero", line_idx + 2));
            }
            CostPattern::Annual if row.cost_year != 0 => {
                return Err(format!("line {}: annual cost uses cost_year=0 as recurrence marker", line_idx + 2));
            }
            CostPattern::Scheduled | CostPattern::ScenarioRestoration if row.cost_year == 0 => {
                return Err(format!("line {}: scheduled/restoration costs must be after year zero", line_idx + 2));
            }
            _ => {}
        }
        if row.pattern != CostPattern::Annual && row.annual_staff_hours != 0.0 {
            return Err(format!("line {}: staff hours must be attached to annual cost rows", line_idx + 2));
        }
        if row.pattern == CostPattern::ScenarioRestoration && row.scenario_id == "all" {
            return Err(format!("line {}: scenario restoration must bind a named scenario", line_idx + 2));
        }
        if row.pattern != CostPattern::ScenarioRestoration && row.scenario_id != "all" {
            return Err(format!("line {}: only restoration rows may be scenario-specific", line_idx + 2));
        }

        match profiles.get(&row.architecture) {
            Some((horizon, rate)) if *horizon != row.service_horizon_years || (*rate - row.discount_rate).abs() > 1e-12 => {
                return Err(format!("line {}: architecture profile uses inconsistent horizon/discount assumptions", line_idx + 2));
            }
            None => {
                profiles.insert(row.architecture, (row.service_horizon_years, row.discount_rate));
            }
            _ => {}
        }

        rows.push(row);
    }

    if rows.is_empty() {
        return Err("lifecycle cost fixture contains no rows".into());
    }
    Ok(rows)
}

pub fn parse_service_outcomes(input: &str) -> Result<Vec<ServiceOutcome>, String> {
    const HEADER: &str = "architecture,scenario_id,intended_receivers,unique_receivers_reached,accessible_receivers_reached,duplicate_channel_receipts";
    let mut lines = input.lines();
    if lines.next() != Some(HEADER) {
        return Err("service outcome fixture header does not match frozen v0.1 contract".into());
    }

    let mut rows = Vec::new();
    let mut ids = HashSet::new();
    for (line_idx, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        if f.len() != 6 {
            return Err(format!("line {}: expected 6 fields, got {}", line_idx + 2, f.len()));
        }

        let row = ServiceOutcome {
            architecture: Architecture::parse(f[0])?,
            scenario_id: f[1].to_owned(),
            intended_receivers: parse_u64(f[2], "intended_receivers")?,
            unique_receivers_reached: parse_u64(f[3], "unique_receivers_reached")?,
            accessible_receivers_reached: parse_u64(f[4], "accessible_receivers_reached")?,
            duplicate_channel_receipts: parse_u64(f[5], "duplicate_channel_receipts")?,
        };

        if row.scenario_id.trim().is_empty() || row.intended_receivers == 0 {
            return Err(format!("line {}: scenario ID and intended receiver count are required", line_idx + 2));
        }
        if row.unique_receivers_reached > row.intended_receivers
            || row.accessible_receivers_reached > row.unique_receivers_reached
        {
            return Err(format!("line {}: delivered counts exceed their denominators", line_idx + 2));
        }
        if !ids.insert((row.architecture, row.scenario_id.clone())) {
            return Err(format!("line {}: duplicate architecture/scenario outcome", line_idx + 2));
        }
        rows.push(row);
    }

    if rows.is_empty() {
        return Err("service outcome fixture contains no rows".into());
    }
    Ok(rows)
}

fn discounted(amount: f64, year: u32, rate: f64) -> f64 {
    amount / (1.0 + rate).powf(f64::from(year))
}

fn present_value(pattern: CostPattern, year: u32, amount: f64, horizon: u32, rate: f64) -> f64 {
    match pattern {
        CostPattern::OneTime => discounted(amount, year, rate),
        CostPattern::Annual => (1..=horizon)
            .map(|period| discounted(amount, period, rate))
            .sum(),
        CostPattern::Scheduled | CostPattern::ScenarioRestoration => {
            if year <= horizon {
                discounted(amount, year, rate)
            } else {
                0.0
            }
        }
    }
}

fn annualized_cost(present_value: f64, horizon: u32, rate: f64) -> f64 {
    if horizon == 0 {
        return 0.0;
    }
    if rate == 0.0 {
        present_value / f64::from(horizon)
    } else {
        present_value * rate / (1.0 - (1.0 + rate).powf(-f64::from(horizon)))
    }
}

fn unit_cost(cost: f64, denominator: u64) -> Option<f64> {
    (denominator > 0).then_some(cost / denominator as f64)
}

pub fn evaluate_at(
    cost_rows: &[CostRow],
    outcomes: &[ServiceOutcome],
    service_horizon_years: u32,
    discount_rate: f64,
) -> Result<Vec<CostEvaluation>, String> {
    if service_horizon_years == 0 || !(0.0..=1.0).contains(&discount_rate) || !discount_rate.is_finite() {
        return Err("horizon must be non-zero and discount rate finite within [0,1]".into());
    }

    let mut evaluations = Vec::new();
    for outcome in outcomes {
        let applicable: Vec<_> = cost_rows
            .iter()
            .filter(|row| {
                row.architecture == outcome.architecture
                    && (row.scenario_id == "all" || row.scenario_id == outcome.scenario_id)
            })
            .collect();

        if !applicable.iter().any(|row| row.scenario_id == "all") {
            return Err(format!("missing common cost profile for {}", outcome.architecture.label()));
        }

        let mut component_totals: BTreeMap<String, ComponentCost> = BTreeMap::new();
        let mut annual_staff_hours = 0.0;

        for row in applicable {
            let low = present_value(row.pattern, row.cost_year, row.low_cost, service_horizon_years, discount_rate);
            let base = present_value(row.pattern, row.cost_year, row.base_cost, service_horizon_years, discount_rate);
            let high = present_value(row.pattern, row.cost_year, row.high_cost, service_horizon_years, discount_rate);
            let component = component_totals.entry(row.component.clone()).or_insert_with(|| ComponentCost {
                component: row.component.clone(),
                ..ComponentCost::default()
            });
            component.low_present_value += low;
            component.base_present_value += base;
            component.high_present_value += high;

            if row.component == "annual_staffing" && row.pattern == CostPattern::Annual {
                annual_staff_hours += row.annual_staff_hours * f64::from(service_horizon_years);
            }
        }

        let components: Vec<_> = component_totals.into_values().collect();
        let low_total = components.iter().map(|item| item.low_present_value).sum();
        let base_total = components.iter().map(|item| item.base_present_value).sum();
        let high_total = components.iter().map(|item| item.high_present_value).sum();

        if !(low_total <= base_total && base_total <= high_total) {
            return Err(format!("low/base/high lifecycle cost ordering failed for {}", outcome.scenario_id));
        }

        evaluations.push(CostEvaluation {
            architecture: outcome.architecture,
            scenario_id: outcome.scenario_id.clone(),
            service_horizon_years,
            discount_rate,
            intended_receivers: outcome.intended_receivers,
            unique_receivers_reached: outcome.unique_receivers_reached,
            accessible_receivers_reached: outcome.accessible_receivers_reached,
            duplicate_channel_receipts: outcome.duplicate_channel_receipts,
            low_lifecycle_present_value: low_total,
            base_lifecycle_present_value: base_total,
            high_lifecycle_present_value: high_total,
            base_annualized_cost: annualized_cost(base_total, service_horizon_years, discount_rate),
            base_cost_per_intended_receiver: unit_cost(base_total, outcome.intended_receivers),
            base_cost_per_unique_receiver_reached: unit_cost(base_total, outcome.unique_receivers_reached),
            base_cost_per_accessible_receiver_reached: unit_cost(base_total, outcome.accessible_receivers_reached),
            annual_staff_hours,
            components,
        });
    }

    Ok(evaluations)
}

pub fn fixture_report() -> Result<CostReport, String> {
    let costs = parse_cost_fixture(COST_FIXTURE)?;
    let outcomes = parse_service_outcomes(OUTCOME_FIXTURE)?;

    let scenarios: HashSet<_> = outcomes.iter().map(|row| row.scenario_id.as_str()).collect();
    let architectures: HashSet<_> = outcomes.iter().map(|row| row.architecture).collect();

    let service_denominators_consistent = scenarios.iter().all(|scenario| {
        let group: Vec<_> = outcomes.iter().filter(|row| row.scenario_id.as_str() == *scenario).collect();
        group.iter().map(|row| row.intended_receivers).collect::<HashSet<_>>().len() == 1
    });

    let outcome_counts_valid = outcomes.iter().all(|row| {
        row.accessible_receivers_reached <= row.unique_receivers_reached
            && row.unique_receivers_reached <= row.intended_receivers
    });

    let profiles: HashSet<_> = costs.iter().map(|row| row.architecture).collect();
    let all_architectures_have_profiles = Architecture::ALL.iter().all(|architecture| profiles.contains(architecture));

    let mut cost_ranges_monotonic = costs.iter().all(|row| row.low_cost <= row.base_cost && row.base_cost <= row.high_cost);
    let mut evaluations = evaluate_at(&costs, &outcomes, 5, 0.05)?;
    cost_ranges_monotonic &= evaluations.iter().all(|row| {
        row.low_lifecycle_present_value <= row.base_lifecycle_present_value
            && row.base_lifecycle_present_value <= row.high_lifecycle_present_value
    });

    // Stable ordering makes CLI and test output deterministic.
    evaluations.sort_by_key(|row| (row.scenario_id.clone(), row.architecture));

    Ok(CostReport {
        cost_record_count: costs.len() as u32,
        service_outcome_count: outcomes.len() as u32,
        scenario_count: scenarios.len() as u32,
        architecture_count: architectures.len() as u32,
        cost_ranges_monotonic,
        service_denominators_consistent,
        outcome_counts_valid,
        all_architectures_have_profiles,
        evaluations,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fnv1a64(bytes: &[u8]) -> u64 {
        bytes.iter().fold(0xcbf29ce484222325u64, |mut hash, byte| {
            hash ^= u64::from(*byte);
            hash.wrapping_mul(0x100000001b3)
        })
    }

    #[test]
    fn fixtures_are_frozen_and_well_formed() {
        assert_eq!(fnv1a64(COST_FIXTURE.as_bytes()), 0x52072da4d6fb1ba6);
        assert_eq!(fnv1a64(OUTCOME_FIXTURE.as_bytes()), 0x376c727b26d887aa);
        let costs = parse_cost_fixture(COST_FIXTURE).unwrap();
        let outcomes = parse_service_outcomes(OUTCOME_FIXTURE).unwrap();
        assert_eq!(costs.len(), 24);
        assert_eq!(outcomes.len(), 21);
        assert_eq!(outcomes.iter().map(|row| row.scenario_id.as_str()).collect::<HashSet<_>>().len(), 7);
    }

    #[test]
    fn lifecycle_cost_is_discounted_by_component_and_scenario() {
        let report = fixture_report().unwrap();
        let central = report.evaluation(Architecture::Centralized, "all-available").unwrap();
        let operations = central.components.iter().find(|component| component.component == "annual_operations").unwrap();
        let factor = (1..=5).map(|year| 1.0 / 1.05f64.powi(year)).sum::<f64>();
        assert!((operations.base_present_value - 1_200.0 * factor).abs() < 1e-9);
    }

    #[test]
    fn low_base_high_ranges_are_monotonic_and_cost_components_sum_to_total() {
        let report = fixture_report().unwrap();
        assert!(report.cost_ranges_monotonic);
        for evaluation in &report.evaluations {
            assert!((evaluation.components.iter().map(|c| c.base_present_value).sum::<f64>()
                - evaluation.base_lifecycle_present_value).abs() < 1e-8);
            assert!(evaluation.low_lifecycle_present_value <= evaluation.base_lifecycle_present_value);
            assert!(evaluation.base_lifecycle_present_value <= evaluation.high_lifecycle_present_value);
        }
    }

    #[test]
    fn zero_accessible_reach_yields_undefined_unit_cost_not_zero_or_infinity() {
        let report = fixture_report().unwrap();
        let zero = report.evaluation(Architecture::Centralized, "origin-authority-unavailable").unwrap();
        assert_eq!(zero.accessible_receivers_reached, 0);
        assert_eq!(zero.base_cost_per_accessible_receiver_reached, None);
        assert_eq!(zero.base_cost_per_unique_receiver_reached, None);
        assert!(zero.base_cost_per_intended_receiver.is_some());
    }

    #[test]
    fn cost_per_accessible_receiver_uses_accessible_unique_count_not_dispatches() {
        let report = fixture_report().unwrap();
        let all = report.evaluation(Architecture::Centralized, "all-available").unwrap();
        assert_eq!(all.unique_receivers_reached, 2_000);
        assert_eq!(all.accessible_receivers_reached, 1_800);
        assert_eq!(all.duplicate_channel_receipts, 2_500);
        assert!((all.base_cost_per_accessible_receiver_reached.unwrap()
            - all.base_lifecycle_present_value / 1_800.0).abs() < 1e-9);
        assert_ne!(
            all.base_cost_per_accessible_receiver_reached,
            Some(all.base_lifecycle_present_value / all.duplicate_channel_receipts as f64)
        );
    }

    #[test]
    fn restoration_costs_apply_only_to_the_named_fault_scenario() {
        let report = fixture_report().unwrap();
        let all = report.evaluation(Architecture::Centralized, "all-available").unwrap();
        let fault = report.evaluation(Architecture::Centralized, "central-hub-outage").unwrap();
        assert!(fault.base_lifecycle_present_value > all.base_lifecycle_present_value);
        assert_eq!(all.components.iter().find(|c| c.component == "scenario_restoration"), None);
        assert!(fault.components.iter().any(|c| c.component == "scenario_restoration"));
    }

    #[test]
    fn horizon_and_discount_sensitivity_are_explicit_and_reproducible() {
        let costs = parse_cost_fixture(COST_FIXTURE).unwrap();
        let outcomes = parse_service_outcomes(OUTCOME_FIXTURE).unwrap();
        let five_year = evaluate_at(&costs, &outcomes, 5, 0.05).unwrap();
        let ten_year = evaluate_at(&costs, &outcomes, 10, 0.05).unwrap();
        let five_year_zero_discount = evaluate_at(&costs, &outcomes, 5, 0.0).unwrap();
        let base5 = five_year.iter().find(|row| row.architecture == Architecture::Centralized && row.scenario_id == "all-available").unwrap();
        let base10 = ten_year.iter().find(|row| row.architecture == Architecture::Centralized && row.scenario_id == "all-available").unwrap();
        let zero_rate = five_year_zero_discount.iter().find(|row| row.architecture == Architecture::Centralized && row.scenario_id == "all-available").unwrap();
        assert!(base10.base_lifecycle_present_value > base5.base_lifecycle_present_value);
        assert!(zero_rate.base_lifecycle_present_value > base5.base_lifecycle_present_value);
    }

    #[test]
    fn invalid_service_denominators_and_cost_ranges_fail_closed() {
        let invalid_outcome = OUTCOME_FIXTURE.replace(
            "centralized,all-available,2000,2000,1800,2500",
            "centralized,all-available,2000,2000,2001,2500",
        );
        assert_ne!(invalid_outcome, OUTCOME_FIXTURE);
        assert!(parse_service_outcomes(&invalid_outcome).is_err());

        let invalid_cost = COST_FIXTURE.replace(
            "C001,centralized,all,capital_setup,one_time,0,5,0.05,6500,8000,10500,0",
            "C001,centralized,all,capital_setup,one_time,0,5,0.05,9000,8000,10500,0",
        );
        assert_ne!(invalid_cost, COST_FIXTURE);
        assert!(parse_cost_fixture(&invalid_cost).is_err());
    }
}
