#![forbid(unsafe_code)]

// Composes the existing per-interval portfolio and stock-flow references.
// This is not a scheduler, optimiser, evidence authenticator or controller.

use std::collections::{BTreeMap, BTreeSet};

const HEADER: &str = "record_type|record_id|option_id|service_id|cohort_id|scenario_id|interval_id|subject_id|quantity|unit|status|source_owner|evidence_ref";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InputError(pub String);

impl std::fmt::Display for InputError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for InputError {}

#[derive(Clone, Debug)]
struct Interval {
    id: String,
    start: u64,
    end: u64,
    records: Vec<String>,
    floors: BTreeSet<(String, String)>,
}

#[derive(Clone, Debug)]
struct Plan {
    portfolio_id: String,
    scenario_id: String,
    snapshot_id: String,
    decision_at: u64,
    clock_basis_id: String,
    stock_resources: Vec<String>,
    cohort_coverage_complete: bool,
    intervals: Vec<Interval>,
}

#[derive(Clone, Debug)]
struct StockInterval {
    id: String,
    start: u64,
    end: u64,
    opening: Option<u64>,
    replenishment: Option<u64>,
    consumption: Option<u64>,
    loss: Option<u64>,
    closing: Option<u64>,
    state: String,
    owner: String,
    source_ref: String,
}

#[derive(Clone, Debug)]
struct StockSeries {
    scenario_id: String,
    resource_id: String,
    unit: String,
    time_basis_id: String,
    disposition: String,
    intervals: Vec<StockInterval>,
}

#[derive(Clone, Debug)]
struct PortfolioInterval {
    disposition: String,
    service_floors: Vec<String>,
    resources: BTreeMap<String, ResourceResult>,
    evidence: Vec<String>,
}

#[derive(Clone, Debug)]
struct ResourceResult {
    demand: Option<u64>,
    capacity: Option<u64>,
    unit: String,
    state: String,
}

fn valid_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 160
        && value
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'_' | b'-' | b'.' | b':' | b'/'))
}

fn metadata_value(metadata: &BTreeMap<String, String>, key: &str) -> Result<String, InputError> {
    let value = metadata
        .get(key)
        .ok_or_else(|| InputError(format!("missing metadata key {key}")))?;
    if value.is_empty() {
        return Err(InputError(format!("empty metadata key {key}")));
    }
    Ok(value.clone())
}

fn parse_plan(source: &str) -> Result<Plan, InputError> {
    let mut metadata = BTreeMap::<String, String>::new();
    let mut interval_specs = Vec::<(String, u64, u64)>::new();
    let mut raw_records = Vec::<(usize, String, Vec<String>)>::new();
    let mut header_seen = false;
    let mut record_ids = BTreeSet::new();

    for (offset, raw) in source.lines().enumerate() {
        let line_number = offset + 1;
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            if let Some((key, value)) = comment.trim().split_once('=') {
                let key = key.trim();
                let value = value.trim();
                if key == "interval" {
                    let parts: Vec<&str> = value.split('|').map(str::trim).collect();
                    if parts.len() != 3 || !valid_id(parts[0]) {
                        return Err(InputError(format!(
                            "line {line_number}: interval metadata must be id|start_seconds|end_seconds"
                        )));
                    }
                    let start = parts[1].parse::<u64>().map_err(|error| {
                        InputError(format!("line {line_number}: invalid interval start: {error}"))
                    })?;
                    let end = parts[2].parse::<u64>().map_err(|error| {
                        InputError(format!("line {line_number}: invalid interval end: {error}"))
                    })?;
                    interval_specs.push((parts[0].to_owned(), start, end));
                } else if metadata.insert(key.to_owned(), value.to_owned()).is_some() {
                    return Err(InputError(format!(
                        "line {line_number}: duplicate metadata key {key}"
                    )));
                }
            }
            continue;
        }
        if !header_seen {
            if line != HEADER {
                return Err(InputError(format!("line {line_number}: unexpected plan header")));
            }
            header_seen = true;
            continue;
        }
        let fields: Vec<&str> = line.split('|').map(str::trim).collect();
        if fields.len() != 13 {
            return Err(InputError(format!(
                "line {line_number}: expected 13 plan columns, received {}",
                fields.len()
            )));
        }
        if !valid_id(fields[1]) || !record_ids.insert(fields[1].to_owned()) {
            return Err(InputError(format!(
                "line {line_number}: invalid or duplicate record_id {}",
                fields[1]
            )));
        }
        raw_records.push((
            line_number,
            line.to_owned(),
            fields.iter().map(|field| (*field).to_owned()).collect(),
        ));
    }

    if !header_seen || raw_records.is_empty() || interval_specs.is_empty() {
        return Err(InputError("plan requires a header, records and interval metadata".to_owned()));
    }

    let portfolio_id = metadata_value(&metadata, "portfolio_id")?;
    let scenario_id = metadata_value(&metadata, "scenario_id")?;
    let snapshot_id = metadata_value(&metadata, "snapshot_id")?;
    let clock_basis_id = metadata_value(&metadata, "clock_basis_id")?;
    let decision_at = metadata_value(&metadata, "decision_at")?
        .parse::<u64>()
        .map_err(|error| InputError(format!("invalid decision_at: {error}")))?;
    let resource_list = metadata_value(&metadata, "stock_resource_ids")?;
    if let Some(key) = metadata.keys().find(|key| {
        !matches!(
            key.as_str(),
            "portfolio_id"
                | "scenario_id"
                | "snapshot_id"
                | "clock_basis_id"
                | "decision_at"
                | "stock_resource_ids"
        )
    }) {
        return Err(InputError(format!("unknown plan metadata key {key}")));
    }

    for (label, value) in [
        ("portfolio_id", portfolio_id.as_str()),
        ("scenario_id", scenario_id.as_str()),
        ("snapshot_id", snapshot_id.as_str()),
        ("clock_basis_id", clock_basis_id.as_str()),
    ] {
        if !valid_id(value) {
            return Err(InputError(format!("invalid metadata value for {label}")));
        }
    }

    let mut stock_resources: Vec<String> = resource_list
        .split(',')
        .map(str::trim)
        .map(str::to_owned)
        .collect();
    if stock_resources.is_empty() || stock_resources.iter().any(|id| !valid_id(id)) {
        return Err(InputError("stock_resource_ids must be a non-empty comma-separated ID list".to_owned()));
    }
    stock_resources.sort();
    if stock_resources.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(InputError("duplicate stock resource ID".to_owned()));
    }

    interval_specs.sort_by_key(|(_, start, _)| *start);
    let mut interval_ids = BTreeSet::new();
    for (index, (id, start, end)) in interval_specs.iter().enumerate() {
        if !interval_ids.insert(id.clone()) || start >= end {
            return Err(InputError(format!("invalid or duplicate interval metadata for {id}")));
        }
        if index > 0 && interval_specs[index - 1].2 != *start {
            return Err(InputError(format!(
                "plan interval gap or overlap between {} and {id}",
                interval_specs[index - 1].0
            )));
        }
    }
    if decision_at > interval_specs[0].1 {
        return Err(InputError(
            "decision_at must not be later than the first interval start".to_owned(),
        ));
    }

    let mut intervals: Vec<Interval> = interval_specs
        .into_iter()
        .map(|(id, start, end)| Interval {
            id,
            start,
            end,
            records: Vec::new(),
            floors: BTreeSet::new(),
        })
        .collect();

    for (line_number, raw, fields) in raw_records {
        if fields[5] != scenario_id {
            return Err(InputError(format!(
                "line {line_number}: record {} crosses the frozen scenario boundary",
                fields[1]
            )));
        }
        let interval = intervals
            .iter_mut()
            .find(|interval| interval.id == fields[6])
            .ok_or_else(|| {
                InputError(format!(
                    "line {line_number}: record {} references undeclared interval {}",
                    fields[1], fields[6]
                ))
            })?;
        if fields[0] == "SERVICE_FLOOR" {
            if fields[3].is_empty() || fields[4].is_empty() {
                return Err(InputError(format!("line {line_number}: malformed service floor")));
            }
            interval
                .floors
                .insert((fields[3].clone(), fields[4].clone()));
        }
        interval.records.push(raw);
    }

    if intervals.iter().any(|interval| interval.records.is_empty()) {
        return Err(InputError("every declared interval must contain portfolio records".to_owned()));
    }
    let first_floor_set = intervals[0].floors.clone();
    let cohort_coverage_complete = !first_floor_set.is_empty()
        && intervals
            .iter()
            .all(|interval| interval.floors == first_floor_set);

    Ok(Plan {
        portfolio_id,
        scenario_id,
        snapshot_id,
        decision_at,
        clock_basis_id,
        stock_resources,
        cohort_coverage_complete,
        intervals,
    })
}

fn parse_u64_or_unknown(value: &str, label: &str) -> Result<Option<u64>, InputError> {
    if value == "UNKNOWN" || value.is_empty() {
        Ok(None)
    } else {
        value
            .parse::<u64>()
            .map(Some)
            .map_err(|error| InputError(format!("invalid {label}: {error}")))
    }
}

fn parse_stock_flow(source: &str) -> Result<StockSeries, InputError> {
    let report = fsre_resource_stock_flow::validate_and_report(source)
        .map_err(|error| InputError(format!("stock-flow reference rejected input: {error}")))?;
    let mut values = BTreeMap::<String, String>::new();
    let mut intervals = Vec::new();
    for line in report.lines() {
        if let Some((key, value)) = line.split_once('=') {
            if key == "interval" {
                let fields: Vec<&str> = value.split('|').collect();
                if fields.len() != 11 {
                    return Err(InputError("malformed stock-flow reference interval report".to_owned()));
                }
                intervals.push(StockInterval {
                    id: fields[0].to_owned(),
                    start: fields[1].parse().map_err(|error| {
                        InputError(format!("invalid stock-flow interval start: {error}"))
                    })?,
                    end: fields[2].parse().map_err(|error| {
                        InputError(format!("invalid stock-flow interval end: {error}"))
                    })?,
                    opening: parse_u64_or_unknown(fields[3], "stock opening")?,
                    replenishment: parse_u64_or_unknown(fields[4], "stock replenishment")?,
                    consumption: parse_u64_or_unknown(fields[5], "stock consumption")?,
                    loss: parse_u64_or_unknown(fields[6], "stock loss")?,
                    closing: parse_u64_or_unknown(fields[7], "stock closing")?,
                    state: fields[8].to_owned(),
                    owner: fields[9].to_owned(),
                    source_ref: fields[10].to_owned(),
                });
            } else {
                values.insert(key.to_owned(), value.to_owned());
            }
        }
    }
    let get = |key: &str| {
        values
            .get(key)
            .cloned()
            .ok_or_else(|| InputError(format!("stock-flow report missing {key}")))
    };
    Ok(StockSeries {
        scenario_id: get("scenario_id")?,
        resource_id: get("resource_id")?,
        unit: get("resource_unit")?,
        time_basis_id: get("time_basis_id")?,
        disposition: get("stock_flow_disposition")?,
        intervals,
    })
}

fn parse_portfolio_report(report: &str) -> Result<PortfolioInterval, InputError> {
    let mut disposition = None;
    let mut service_floors = Vec::new();
    let mut resources = BTreeMap::new();
    let mut evidence = Vec::new();
    for line in report.lines() {
        if let Some(value) = line.strip_prefix("portfolio_disposition=") {
            disposition = Some(value.to_owned());
        } else if let Some(value) = line.strip_prefix("service_floor=") {
            let fields: Vec<&str> = value.split('|').collect();
            if fields.len() != 6 {
                return Err(InputError("malformed per-interval service-floor report".to_owned()));
            }
            service_floors.push(value.to_owned());
        } else if let Some(value) = line.strip_prefix("resource=") {
            let fields: Vec<&str> = value.split('|').collect();
            if fields.len() != 5 {
                return Err(InputError("malformed per-interval resource report".to_owned()));
            }
            let result = ResourceResult {
                demand: parse_u64_or_unknown(fields[1], "portfolio resource demand")?,
                capacity: parse_u64_or_unknown(fields[2], "portfolio resource capacity")?,
                unit: fields[3].to_owned(),
                state: fields[4].to_owned(),
            };
            if resources.insert(fields[0].to_owned(), result).is_some() {
                return Err(InputError(format!("duplicate resource result {}", fields[0])));
            }
        } else if let Some(value) = line.strip_prefix("evidence=") {
            evidence.push(value.to_owned());
        }
    }
    Ok(PortfolioInterval {
        disposition: disposition
            .ok_or_else(|| InputError("portfolio reference report has no disposition".to_owned()))?,
        service_floors,
        resources,
        evidence,
    })
}

fn build_interval_plan(plan: &Plan, interval: &Interval) -> String {
    let mut source = format!(
        "# portfolio_id={}\n# scenario_id={}\n# interval_id={}\n# snapshot_id={}\n# decision_at={}\n# clock_basis_id={}\n{}\n",
        plan.portfolio_id,
        plan.scenario_id,
        interval.id,
        plan.snapshot_id,
        plan.decision_at,
        plan.clock_basis_id,
        HEADER
    );
    source.push_str(&interval.records.join("\n"));
    source.push('\n');
    source
}

fn compose(plan_source: &str, stock_sources: &[String]) -> Result<String, InputError> {
    let plan = parse_plan(plan_source)?;
    let mut stock_series = BTreeMap::<String, StockSeries>::new();
    for source in stock_sources {
        let series = parse_stock_flow(source)?;
        let id = series.resource_id.clone();
        if stock_series.insert(id.clone(), series).is_some() {
            return Err(InputError(format!("duplicate stock-flow resource series {id}")));
        }
    }
    let supplied: Vec<String> = stock_series.keys().cloned().collect();
    if supplied != plan.stock_resources {
        return Err(InputError(format!(
            "stock-flow resource files {:?} do not match declared stock_resource_ids {:?}",
            supplied, plan.stock_resources
        )));
    }

    let mut not_estimable = !plan.cohort_coverage_complete;
    let mut not_reconciled = false;
    let mut not_feasible = false;
    let mut portfolio_results = Vec::new();

    for interval in &plan.intervals {
        let source = build_interval_plan(&plan, interval);
        let report = fsre_portfolio_reference::validate_and_report(&source).map_err(|error| {
            InputError(format!(
                "portfolio reference rejected interval {}: {error}",
                interval.id
            ))
        })?;
        let result = parse_portfolio_report(&report)?;
        match result.disposition.as_str() {
            "PORTFOLIO_FEASIBLE_FOR_DECLARED_SCENARIO" => {}
            "NOT_ESTIMABLE" => not_estimable = true,
            "NOT_FEASIBLE" => not_feasible = true,
            other => {
                return Err(InputError(format!(
                    "unexpected portfolio disposition {other} in {}",
                    interval.id
                )))
            }
        }
        for floor in &result.service_floors {
            let outcome = floor.split('|').nth(5).unwrap_or("NOT_ESTIMABLE");
            match outcome {
                "MET" => {}
                "MISSED" => not_feasible = true,
                _ => not_estimable = true,
            }
        }
        portfolio_results.push(result);
    }

    for (resource_id, series) in &stock_series {
        if series.scenario_id != plan.scenario_id || series.time_basis_id != plan.clock_basis_id {
            not_reconciled = true;
        }
        if series.intervals.len() != plan.intervals.len() {
            not_reconciled = true;
        }
        match series.disposition.as_str() {
            "BALANCED_FOR_DECLARED_HORIZON" => {}
            "NOT_ESTIMABLE" => not_estimable = true,
            "NOT_BALANCED" => not_reconciled = true,
            other => {
                return Err(InputError(format!("unexpected stock-flow disposition {other}")))
            }
        }
        for (index, interval) in plan.intervals.iter().enumerate() {
            let Some(stock) = series.intervals.iter().find(|row| row.id == interval.id) else {
                not_reconciled = true;
                continue;
            };
            if stock.start != interval.start || stock.end != interval.end {
                not_reconciled = true;
            }
            let portfolio = &portfolio_results[index];
            let Some(resource) = portfolio.resources.get(resource_id) else {
                not_estimable = true;
                continue;
            };
            if resource.unit != series.unit {
                not_reconciled = true;
            }
            if resource.state == "NOT_ESTIMABLE" || stock.state.starts_with("NOT_ESTIMABLE") {
                not_estimable = true;
                continue;
            }
            if resource.state != "WITHIN_CAPACITY" {
                not_feasible = true;
            }
            match (resource.demand, stock.consumption) {
                (Some(demand), Some(consumption)) if demand == consumption => {}
                (Some(_), Some(_)) => not_reconciled = true,
                _ => not_estimable = true,
            }
            match (resource.capacity, stock.opening) {
                (Some(capacity), Some(opening)) if capacity <= opening => {}
                (Some(_), Some(_)) => not_reconciled = true,
                _ => not_estimable = true,
            }
        }
    }

    let disposition = if not_estimable {
        "NOT_ESTIMABLE"
    } else if not_reconciled {
        "NOT_RECONCILED"
    } else if not_feasible {
        "NOT_FEASIBLE"
    } else {
        "PORTFOLIO_FEASIBLE_FOR_DECLARED_HORIZON"
    };

    let mut lines = vec![
        "report_schema=fsre-time-expanded-portfolio-composition-report.v0.1".to_owned(),
        format!("portfolio_id={}", plan.portfolio_id),
        format!("scenario_id={}", plan.scenario_id),
        format!("evidence_snapshot_id={}", plan.snapshot_id),
        format!("decision_at={}", plan.decision_at),
        format!("clock_basis_id={}", plan.clock_basis_id),
        format!("intervals={}", plan.intervals.len()),
        format!("stock_resources={}", plan.stock_resources.join(",")),
        format!("time_expanded_disposition={disposition}"),
    ];

    for (index, interval) in plan.intervals.iter().enumerate() {
        let portfolio = &portfolio_results[index];
        lines.push(format!(
            "interval={}|{}|{}|{}",
            interval.id, interval.start, interval.end, portfolio.disposition
        ));
        let mut floors = portfolio.service_floors.clone();
        floors.sort();
        for floor in floors {
            lines.push(format!("service_floor={}|{}", interval.id, floor));
        }
        let mut evidence = portfolio.evidence.clone();
        evidence.sort();
        for row in evidence {
            lines.push(format!("portfolio_evidence={interval}|{row}"));
        }
    }

    for (resource_id, series) in &stock_series {
        for (index, interval) in plan.intervals.iter().enumerate() {
            let stock = series.intervals.iter().find(|row| row.id == interval.id);
            let portfolio = &portfolio_results[index];
            match (stock, portfolio.resources.get(resource_id)) {
                (Some(stock), Some(resource)) => {
                    let matched = match (
                        resource.demand,
                        stock.consumption,
                        resource.capacity,
                        stock.opening,
                    ) {
                        (Some(demand), Some(consumption), Some(capacity), Some(opening)) => {
                            demand == consumption && capacity <= opening && resource.unit == series.unit
                        }
                        _ => false,
                    };
                    lines.push(format!(
                        "resource_reconciliation={}|{}|{}|{}|{}|{}|{}",
                        interval.id,
                        resource_id,
                        resource.demand.map(|v| v.to_string()).unwrap_or_else(|| "UNKNOWN".to_owned()),
                        stock.consumption.map(|v| v.to_string()).unwrap_or_else(|| "UNKNOWN".to_owned()),
                        resource.capacity.map(|v| v.to_string()).unwrap_or_else(|| "UNKNOWN".to_owned()),
                        stock.opening.map(|v| v.to_string()).unwrap_or_else(|| "UNKNOWN".to_owned()),
                        if matched { "MATCHED" } else { "NOT_CONFIRMED" }
                    ));
                    lines.push(format!(
                        "stock_flow={}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}",
                        resource_id,
                        interval.id,
                        stock.start,
                        stock.end,
                        stock.opening.map(|v| v.to_string()).unwrap_or_else(|| "UNKNOWN".to_owned()),
                        stock.replenishment.map(|v| v.to_string()).unwrap_or_else(|| "UNKNOWN".to_owned()),
                        stock.consumption.map(|v| v.to_string()).unwrap_or_else(|| "UNKNOWN".to_owned()),
                        stock.loss.map(|v| v.to_string()).unwrap_or_else(|| "UNKNOWN".to_owned()),
                        stock.closing.map(|v| v.to_string()).unwrap_or_else(|| "UNKNOWN".to_owned()),
                        stock.state,
                        stock.owner,
                        stock.source_ref,
                        series.unit
                    ));
                }
                _ => lines.push(format!(
                    "resource_reconciliation={}|{}|UNKNOWN|UNKNOWN|UNKNOWN|UNKNOWN|NOT_CONFIRMED",
                    interval.id, resource_id
                )),
            }
        }
    }
    lines.push("claim_scope=DECLARED_HORIZON_AND_FROZEN_EVIDENCE_ONLY".to_owned());
    lines.push("service_quality=NOT_EVALUATED_BY_THIS_COMPOSER".to_owned());
    lines.push("as_known_at_cutoff=NOT_INDEPENDENTLY_REPLAYED_BY_THIS_COMPOSER".to_owned());
    lines.push("source_authenticity=NOT_AUTHENTICATED_BY_THIS_TOOL".to_owned());
    lines.push("execution_authority=NONE".to_owned());
    lines.push("operational_actuation=NOT_AUTHORISED".to_owned());
    Ok(lines.join("\n") + "\n")
}

/// Compose per-interval portfolio evaluations with owner-qualified stock-flow series.
///
/// stock_flow_sources must contain exactly one series for each resource ID declared in
/// the plan. Inputs are frozen; no orders, schedules or operational actions are issued.
pub fn validate_and_report(
    plan_source: &str,
    stock_flow_sources: &[String],
) -> Result<String, InputError> {
    compose(plan_source, stock_flow_sources)
}

#[cfg(test)]
mod tests {
    use super::*;

    const PLAN: &str = include_str!("../fixtures/synthetic-plan.psv");
    const STOCK: &str = include_str!("../fixtures/synthetic-diesel.psv");
    const EXPECTED: &str = include_str!("../fixtures/synthetic-composition.expected.txt");

    fn run(plan: &str, stock: &str) -> String {
        validate_and_report(plan, &[stock.to_owned()]).expect("composition")
    }

    #[test]
    fn three_interval_composition_matches_golden_report() {
        assert_eq!(run(PLAN, STOCK), EXPECTED);
    }

    #[test]
    fn a_cohort_floor_miss_cannot_be_offset_by_another_cohort_surplus() {
        let changed = PLAN
            .replace(
                "ALLOCATION|allocation-households-01|backup_plant|water|households|shock|hour-01||60|m3|PROPOSED|operator|plan:households:01",
                "ALLOCATION|allocation-households-01|backup_plant|water|households|shock|hour-01||40|m3|PROPOSED|operator|plan:households:01",
            )
            .replace(
                "ALLOCATION|allocation-hospital-01|backup_plant|water|hospital|shock|hour-01||40|m3|PROPOSED|operator|plan:hospital:01",
                "ALLOCATION|allocation-hospital-01|backup_plant|water|hospital|shock|hour-01||60|m3|PROPOSED|operator|plan:hospital:01",
            );
        let report = run(&changed, STOCK);
        assert!(report.contains("time_expanded_disposition=NOT_FEASIBLE"));
        assert!(report.contains("service_floor=hour-01|water|households|40|55|m3|MISSED"));
    }

    #[test]
    fn portfolio_consumption_must_match_resource_stock_flow() {
        let changed = PLAN.replace(
            "RESOURCE_USE|use-diesel-01|backup_plant|water||shock|hour-01|diesel|20|L|QUALIFIED|operator|use:diesel:01",
            "RESOURCE_USE|use-diesel-01|backup_plant|water||shock|hour-01|diesel|19|L|QUALIFIED|operator|use:diesel:01",
        );
        let report = run(&changed, STOCK);
        assert!(report.contains("time_expanded_disposition=NOT_RECONCILED"));
        assert!(report.contains("resource_reconciliation=hour-01|diesel|19|20|80|80|NOT_CONFIRMED"));
    }

    #[test]
    fn portfolio_capacity_must_not_exceed_opening_inventory() {
        let changed = PLAN.replace(
            "RESOURCE_CAPACITY|cap-diesel-01||||shock|hour-01|diesel|80|L|QUALIFIED|operator|capacity:diesel:01",
            "RESOURCE_CAPACITY|cap-diesel-01||||shock|hour-01|diesel|81|L|QUALIFIED|operator|capacity:diesel:01",
        );
        let report = run(&changed, STOCK);
        assert!(report.contains("time_expanded_disposition=NOT_RECONCILED"));
        assert!(report.contains("resource_reconciliation=hour-01|diesel|20|20|81|80|NOT_CONFIRMED"));
    }

    #[test]
    fn unknown_stock_at_the_end_of_the_horizon_abstains() {
        let changed = STOCK.replace(
            "stock-02|2|hour-02|7200|10800|70|0|20|0|50|operator|stock:diesel:02|KNOWN",
            "stock-02|2|hour-02|7200|10800|70||20|0||operator|stock:diesel:02|UNKNOWN",
        );
        let report = run(PLAN, &changed);
        assert!(report.contains("time_expanded_disposition=NOT_ESTIMABLE"));
    }

    #[test]
    fn scenario_and_clock_basis_mismatch_are_not_reconciled() {
        let changed = STOCK
            .replace("# scenario_id=shock", "# scenario_id=other-shock")
            .replace("synthetic-utc-v1", "other-clock");
        let report = run(PLAN, &changed);
        assert!(report.contains("time_expanded_disposition=NOT_RECONCILED"));
    }

    #[test]
    fn interval_boundary_mismatch_is_not_reconciled() {
        let changed = PLAN
            .replace("# interval=hour-00|0|3600", "# interval=hour-00|0|3601")
            .replace("# interval=hour-01|3600|7200", "# interval=hour-01|3601|7201")
            .replace("# interval=hour-02|7200|10800", "# interval=hour-02|7201|10801");
        let report = run(&changed, STOCK);
        assert!(report.contains("time_expanded_disposition=NOT_RECONCILED"));
    }

    #[test]
    fn every_service_cohort_must_have_a_floor_in_every_interval() {
        let changed = PLAN.replace(
            "SERVICE_FLOOR|floor-households-02||water|households|shock|hour-02||55|m3|REQUIRED|service-owner|floor:households:02\n",
            "",
        );
        let report = run(&changed, STOCK);
        assert!(report.contains("time_expanded_disposition=NOT_ESTIMABLE"));
    }

    #[test]
    fn plan_record_order_does_not_change_the_composition() {
        let lines: Vec<&str> = PLAN.lines().collect();
        let header = lines.iter().position(|line| *line == HEADER).unwrap();
        let metadata = lines[..header].to_vec();
        let mut rows: Vec<&str> = lines[header + 1..].to_vec();
        rows.reverse();
        let reordered = format!("{}\n{}\n{}\n", metadata.join("\n"), HEADER, rows.join("\n"));
        assert_eq!(run(&reordered, STOCK), EXPECTED);
    }
}
