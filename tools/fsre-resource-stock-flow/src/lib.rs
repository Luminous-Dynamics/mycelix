#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

const HEADER: &str = "record_id|interval_index|interval_id|start_seconds|end_seconds|opening_quantity|replenishment_quantity|consumption_quantity|loss_quantity|closing_quantity|source_owner|source_ref|state";

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
    record_id: String,
    index: u64,
    id: String,
    start: u64,
    end: u64,
    opening: Option<u64>,
    replenishment: Option<u64>,
    consumption: Option<u64>,
    loss: Option<u64>,
    closing: Option<u64>,
    owner: String,
    source_ref: String,
    state: String,
}

#[derive(Clone, Debug)]
struct Series {
    scenario_id: String,
    resource_id: String,
    unit: String,
    time_basis_id: String,
    intervals: Vec<Interval>,
}

fn valid_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 160
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.' | b':' | b'/')
        })
}

fn parse_u64(value: &str, label: &str, line: usize) -> Result<u64, InputError> {
    value
        .parse::<u64>()
        .map_err(|error| InputError(format!("line {line}: invalid {label}: {error}")))
}

fn optional_u64(value: &str, label: &str, line: usize) -> Result<Option<u64>, InputError> {
    if value.is_empty() {
        Ok(None)
    } else {
        parse_u64(value, label, line).map(Some)
    }
}

fn parse_interval(line: &str, line_number: usize) -> Result<Interval, InputError> {
    let fields: Vec<&str> = line.split('|').map(str::trim).collect();
    if fields.len() != 13 {
        return Err(InputError(format!(
            "line {line_number}: expected 13 columns, received {}",
            fields.len()
        )));
    }
    let interval = Interval {
        record_id: fields[0].to_owned(),
        index: parse_u64(fields[1], "interval_index", line_number)?,
        id: fields[2].to_owned(),
        start: parse_u64(fields[3], "start_seconds", line_number)?,
        end: parse_u64(fields[4], "end_seconds", line_number)?,
        opening: optional_u64(fields[5], "opening_quantity", line_number)?,
        replenishment: optional_u64(fields[6], "replenishment_quantity", line_number)?,
        consumption: optional_u64(fields[7], "consumption_quantity", line_number)?,
        loss: optional_u64(fields[8], "loss_quantity", line_number)?,
        closing: optional_u64(fields[9], "closing_quantity", line_number)?,
        owner: fields[10].to_owned(),
        source_ref: fields[11].to_owned(),
        state: fields[12].to_owned(),
    };
    for (label, value) in [
        ("record_id", interval.record_id.as_str()),
        ("interval_id", interval.id.as_str()),
        ("source_owner", interval.owner.as_str()),
    ] {
        if !valid_id(value) {
            return Err(InputError(format!("line {line_number}: invalid {label}")));
        }
    }
    if interval.start >= interval.end {
        return Err(InputError(format!(
            "line {line_number}: interval must have positive duration"
        )));
    }
    if interval.source_ref.is_empty() || interval.source_ref.len() > 512 {
        return Err(InputError(format!(
            "line {line_number}: missing or invalid source_ref"
        )));
    }
    match interval.state.as_str() {
        "KNOWN" => {
            if [
                interval.opening,
                interval.replenishment,
                interval.consumption,
                interval.loss,
                interval.closing,
            ]
            .iter()
            .any(Option::is_none)
            {
                return Err(InputError(format!(
                    "line {line_number}: KNOWN interval requires all five quantities"
                )));
            }
        }
        "UNKNOWN" => {
            if interval.closing.is_some() {
                return Err(InputError(format!(
                    "line {line_number}: UNKNOWN interval cannot assert a closing balance"
                )));
            }
            if [
                interval.opening,
                interval.replenishment,
                interval.consumption,
                interval.loss,
            ]
            .iter()
            .all(Option::is_some)
            {
                return Err(InputError(format!(
                    "line {line_number}: UNKNOWN interval must retain at least one unknown term"
                )));
            }
        }
        _ => {
            return Err(InputError(format!(
                "line {line_number}: unsupported state {}",
                interval.state
            )));
        }
    }
    Ok(interval)
}

fn parse_series(source: &str) -> Result<Series, InputError> {
    let mut metadata = BTreeMap::<String, String>::new();
    let mut intervals = Vec::new();
    let mut ids = BTreeSet::new();
    let mut header_seen = false;

    for (index, raw) in source.lines().enumerate() {
        let line_number = index + 1;
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            if let Some((key, value)) = comment.trim().split_once('=') {
                if metadata
                    .insert(key.trim().to_owned(), value.trim().to_owned())
                    .is_some()
                {
                    return Err(InputError(format!(
                        "line {line_number}: duplicate metadata key"
                    )));
                }
            }
            continue;
        }
        if !header_seen {
            if line != HEADER {
                return Err(InputError(format!(
                    "line {line_number}: unexpected stock-flow header"
                )));
            }
            header_seen = true;
            continue;
        }
        let row = parse_interval(line, line_number)?;
        if !ids.insert(row.record_id.clone()) {
            return Err(InputError(format!(
                "line {line_number}: duplicate record_id {}",
                row.record_id
            )));
        }
        intervals.push(row);
    }
    if !header_seen || intervals.is_empty() {
        return Err(InputError("missing stock-flow header or rows".to_owned()));
    }
    let mut take = |key: &str| {
        let value = metadata
            .remove(key)
            .ok_or_else(|| InputError(format!("missing metadata key {key}")))?;
        if !valid_id(&value) {
            return Err(InputError(format!("invalid metadata key {key}")));
        }
        Ok(value)
    };
    let scenario_id = take("scenario_id")?;
    let resource_id = take("resource_id")?;
    let unit = take("resource_unit")?;
    let time_basis_id = take("time_basis_id")?;
    if let Some(key) = metadata.keys().next() {
        return Err(InputError(format!("unknown metadata key {key}")));
    }
    intervals.sort_by_key(|row| row.index);
    for (expected, row) in intervals.iter().enumerate() {
        if row.index != expected as u64 {
            return Err(InputError(format!(
                "interval indices must be contiguous from zero; found {} at position {expected}",
                row.index
            )));
        }
        if expected > 0 {
            let previous = &intervals[expected - 1];
            if previous.end != row.start {
                return Err(InputError(format!(
                    "time gap or overlap between {} and {}",
                    previous.id, row.id
                )));
            }
        }
    }
    Ok(Series {
        scenario_id,
        resource_id,
        unit,
        time_basis_id,
        intervals,
    })
}

#[derive(Clone, Debug)]
struct IntervalResult {
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

fn evaluate(series: &Series) -> Result<String, InputError> {
    let mut results = Vec::new();
    let mut previous_closing: Option<u64> = None;
    let mut continuity_lost = false;
    let mut any_unknown = false;
    let mut any_unbalanced = false;
    let mut first_opening: Option<u64> = None;
    let mut final_closing: Option<u64> = None;
    let mut unreconciled = Vec::new();

    for (index, interval) in series.intervals.iter().enumerate() {
        let mut status = "BALANCED";
        if interval.state == "UNKNOWN" {
            any_unknown = true;
            if let (Some(previous), Some(opening)) = (previous_closing, interval.opening) {
                if previous != opening {
                    any_unbalanced = true;
                    status = "NOT_ESTIMABLE_CONTINUITY_MISMATCH";
                } else {
                    status = "NOT_ESTIMABLE";
                }
            } else {
                status = "NOT_ESTIMABLE";
            }
            continuity_lost = true;
            unreconciled.push(interval.id.clone());
            previous_closing = None;
            final_closing = None;
        } else if continuity_lost {
            return Err(InputError(format!(
                "known interval {} follows an unknown stock-flow interval without a new series boundary",
                interval.id
            )));
        } else {
            let opening = interval.opening.expect("KNOWN interval has opening stock");
            let replenishment = interval
                .replenishment
                .expect("KNOWN interval has replenishment");
            let consumption = interval
                .consumption
                .expect("KNOWN interval has consumption");
            let loss = interval.loss.expect("KNOWN interval has loss");
            let closing = interval.closing.expect("KNOWN interval has closing stock");

            if index == 0 {
                first_opening = Some(opening);
            } else if previous_closing != Some(opening) {
                status = "CONTINUITY_MISMATCH";
                any_unbalanced = true;
            }

            let available = opening.checked_add(replenishment).ok_or_else(|| {
                InputError(format!("available stock overflow in {}", interval.id))
            })?;
            let required = consumption.checked_add(loss).ok_or_else(|| {
                InputError(format!("resource outflow overflow in {}", interval.id))
            })?;
            match available.checked_sub(required) {
                Some(expected_closing) if expected_closing == closing => {}
                _ => {
                    if status == "BALANCED" {
                        status = "FLOW_MISMATCH";
                    }
                    any_unbalanced = true;
                }
            }
            previous_closing = Some(closing);
            final_closing = Some(closing);
        }

        results.push(IntervalResult {
            id: interval.id.clone(),
            start: interval.start,
            end: interval.end,
            opening: interval.opening,
            replenishment: interval.replenishment,
            consumption: interval.consumption,
            loss: interval.loss,
            closing: interval.closing,
            state: status.to_owned(),
            owner: interval.owner.clone(),
            source_ref: interval.source_ref.clone(),
        });
    }

    let disposition = if any_unknown {
        "NOT_ESTIMABLE"
    } else if any_unbalanced {
        "NOT_BALANCED"
    } else {
        "BALANCED_FOR_DECLARED_HORIZON"
    };
    let mut out = vec![
        "report_schema=fsre-resource-stock-flow-report.v0.1".to_owned(),
        format!("scenario_id={}", series.scenario_id),
        format!("resource_id={}", series.resource_id),
        format!("resource_unit={}", series.unit),
        format!("time_basis_id={}", series.time_basis_id),
        format!("intervals={}", results.len()),
        format!("stock_flow_disposition={disposition}"),
        format!(
            "opening_stock={}",
            first_opening
                .map(|v| v.to_string())
                .unwrap_or_else(|| "UNKNOWN".to_owned())
        ),
        format!(
            "closing_stock={}",
            final_closing
                .map(|v| v.to_string())
                .unwrap_or_else(|| "UNKNOWN".to_owned())
        ),
    ];
    for result in &results {
        out.push(format!(
            "interval={}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}",
            result.id,
            result.start,
            result.end,
            result
                .opening
                .map(|v| v.to_string())
                .unwrap_or_else(|| "UNKNOWN".to_owned()),
            result
                .replenishment
                .map(|v| v.to_string())
                .unwrap_or_else(|| "UNKNOWN".to_owned()),
            result
                .consumption
                .map(|v| v.to_string())
                .unwrap_or_else(|| "UNKNOWN".to_owned()),
            result
                .loss
                .map(|v| v.to_string())
                .unwrap_or_else(|| "UNKNOWN".to_owned()),
            result
                .closing
                .map(|v| v.to_string())
                .unwrap_or_else(|| "UNKNOWN".to_owned()),
            result.state,
            result.owner,
            result.source_ref
        ));
    }
    out.push(format!(
        "unreconciled_intervals={}",
        if unreconciled.is_empty() {
            "NONE".to_owned()
        } else {
            unreconciled.join(",")
        }
    ));
    out.push("source_authenticity=NOT_AUTHENTICATED_BY_THIS_TOOL".to_owned());
    out.push("execution_authority=NONE".to_owned());
    out.push("operational_actuation=NOT_AUTHORISED".to_owned());
    Ok(out.join("\n") + "\n")
}

pub fn validate_and_report(source: &str) -> Result<String, InputError> {
    evaluate(&parse_series(source)?)
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../fixtures/synthetic-balanced.psv");

    #[test]
    fn three_interval_stock_flow_conserves_inventory_with_replenishment() {
        let report = validate_and_report(FIXTURE).expect("balanced fixture");
        assert!(report.contains("stock_flow_disposition=BALANCED_FOR_DECLARED_HORIZON"));
        assert!(report.contains("opening_stock=100"));
        assert!(report.contains("closing_stock=50"));
        assert!(report.contains("interval=hour-01|3600|7200|70|50|60|0|60|BALANCED"));
    }

    #[test]
    fn flow_mismatch_is_not_misreported_as_balanced() {
        let changed = FIXTURE.replace(
            "stock-001|1|hour-01|3600|7200|70|50|60|0|60",
            "stock-001|1|hour-01|3600|7200|70|50|60|0|55",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("mismatch is a reportable result");
        assert!(report.contains("stock_flow_disposition=NOT_BALANCED"));
        assert!(report.contains("FLOW_MISMATCH"));
    }

    #[test]
    fn continuity_mismatch_between_intervals_is_detected() {
        let changed = FIXTURE.replace(
            "stock-002|2|hour-02|7200|10800|60|0|10|0|50",
            "stock-002|2|hour-02|7200|10800|61|0|10|0|51",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("continuity mismatch is reportable");
        assert!(report.contains("stock_flow_disposition=NOT_BALANCED"));
        assert!(report.contains("CONTINUITY_MISMATCH"));
    }

    #[test]
    fn stock_cannot_become_negative() {
        let changed = FIXTURE.replace(
            "stock-002|2|hour-02|7200|10800|60|0|10|0|50",
            "stock-002|2|hour-02|7200|10800|60|0|70|0|0",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("negative balance is a mismatch");
        assert!(report.contains("stock_flow_disposition=NOT_BALANCED"));
        assert!(report.contains("FLOW_MISMATCH"));
    }

    #[test]
    fn unknown_interval_breaks_stock_continuity_and_abstains() {
        let changed = FIXTURE
            .replace(
                "stock-001|1|hour-01|3600|7200|70|50|60|0|60|operator|synthetic:stock:hour-01|KNOWN",
                "stock-001|1|hour-01|3600|7200|70||60|0||operator|synthetic:stock:hour-01|UNKNOWN",
            )
            .replace(
                "stock-002|2|hour-02|7200|10800|60|0|10|0|50|operator|synthetic:stock:hour-02|KNOWN",
                "stock-002|2|hour-02|7200|10800||||||operator|synthetic:stock:hour-02|UNKNOWN",
            );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("unknown interval is explicit");
        assert!(report.contains("stock_flow_disposition=NOT_ESTIMABLE"));
        assert!(report.contains("unreconciled_intervals=hour-01,hour-02"));
    }

    #[test]
    fn known_interval_after_unknown_is_rejected_without_reconciliation() {
        let changed = FIXTURE.replace(
            "stock-001|1|hour-01|3600|7200|70|50|60|0|60|operator|synthetic:stock:hour-01|KNOWN",
            "stock-001|1|hour-01|3600|7200|70||60|0||operator|synthetic:stock:hour-01|UNKNOWN",
        );
        assert_ne!(changed, FIXTURE);
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("known interval hour-02 follows an unknown"));
    }

    #[test]
    fn time_gaps_and_overlaps_are_rejected() {
        let changed = FIXTURE.replace(
            "stock-002|2|hour-02|7200|10800",
            "stock-002|2|hour-02|7300|10800",
        );
        assert_ne!(changed, FIXTURE);
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("time gap or overlap"));
    }

    #[test]
    fn row_order_does_not_change_stock_flow_report() {
        let metadata: Vec<&str> = FIXTURE
            .lines()
            .filter(|line| line.starts_with('#'))
            .collect();
        let rows: Vec<&str> = FIXTURE
            .lines()
            .filter(|line| !line.starts_with('#') && !line.is_empty())
            .collect();
        let reordered = format!(
            "{}\n{}\n{}\n",
            metadata.join("\n"),
            HEADER,
            rows.into_iter().rev().collect::<Vec<_>>().join("\n")
        );
        assert_eq!(
            validate_and_report(FIXTURE).expect("original"),
            validate_and_report(&reordered).expect("reordered")
        );
    }

    #[test]
    fn arithmetic_overflow_is_rejected() {
        let changed = FIXTURE.replace(
            "stock-000|0|hour-00|0|3600|100|0|30|0|70",
            "stock-000|0|hour-00|0|3600|18446744073709551615|1|30|0|70",
        );
        assert_ne!(changed, FIXTURE);
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("available stock overflow"));
    }

    #[test]
    fn source_evidence_is_preserved_and_authority_is_absent() {
        let report = validate_and_report(FIXTURE).expect("fixture");
        assert!(report.contains("operator|synthetic:stock:hour-01"));
        assert!(report.contains("operational_actuation=NOT_AUTHORISED"));
    }

    #[test]
    fn generated_stock_flow_arithmetic_sweep_rejects_one_unit_errors() {
        let mut checked = 0_u64;
        for opening in 0..=8_u64 {
            for replenishment in 0..=8_u64 {
                for consumption in 0..=8_u64 {
                    for loss in 0..=8_u64 {
                        let available = opening + replenishment;
                        let outgoing = consumption + loss;
                        if outgoing > available {
                            continue;
                        }
                        let closing = available - outgoing;
                        let source = format!(
                            "# scenario_id=sweep-scenario\n# resource_id=sweep-resource\n# resource_unit=unit\n# time_basis_id=sweep-clock\n{HEADER}\nsweep-row|0|sweep-interval|0|60|{opening}|{replenishment}|{consumption}|{loss}|{closing}|operator|synthetic:sweep|KNOWN\n"
                        );
                        let report = validate_and_report(&source).expect("balanced sweep case");
                        assert!(
                            report.contains("stock_flow_disposition=BALANCED_FOR_DECLARED_HORIZON")
                        );
                        let wrong_closing = closing + 1;
                        let invalid = source.replace(
                            &format!("|{loss}|{closing}|operator|synthetic:sweep|KNOWN"),
                            &format!("|{loss}|{wrong_closing}|operator|synthetic:sweep|KNOWN"),
                        );
                        let invalid_report =
                            validate_and_report(&invalid).expect("mismatch report");
                        assert!(invalid_report.contains("stock_flow_disposition=NOT_BALANCED"));
                        checked += 1;
                    }
                }
            }
        }
        assert!(
            checked >= 3_000,
            "expected broad arithmetic sweep, got {checked}"
        );
    }
}
