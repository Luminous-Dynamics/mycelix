#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

const HEADER: &str = "record_id|interval_index|interval_id|start_seconds|end_seconds|opening_quantity|replenishment_quantity|consumption_quantity|loss_quantity|closing_quantity|source_owner|source_ref|state";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReferenceError(pub String);

impl std::fmt::Display for ReferenceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for ReferenceError {}

#[derive(Clone, Debug)]
struct Row {
    record: String,
    position: u64,
    label: String,
    begin: u64,
    finish: u64,
    opening: Option<u64>,
    receipt: Option<u64>,
    use_amount: Option<u64>,
    removal: Option<u64>,
    closing: Option<u64>,
    owner: String,
    reference: String,
    classification: String,
}

#[derive(Clone, Debug)]
struct Dataset {
    scenario: String,
    resource: String,
    measure: String,
    clock: String,
    rows: Vec<Row>,
}

fn identifier(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 160
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.' | b':' | b'/')
        })
}

fn number(value: &str, name: &str, line: usize) -> Result<u64, ReferenceError> {
    value.parse::<u64>().map_err(|error| {
        ReferenceError(format!("line {line}: invalid {name}: {error}"))
    })
}

fn maybe_number(value: &str, name: &str, line: usize) -> Result<Option<u64>, ReferenceError> {
    if value.is_empty() {
        Ok(None)
    } else {
        number(value, name, line).map(Some)
    }
}

fn decode_row(source: &str, line: usize) -> Result<Row, ReferenceError> {
    let cells: Vec<&str> = source.split('|').map(str::trim).collect();
    if cells.len() != 13 {
        return Err(ReferenceError(format!(
            "line {line}: expected 13 columns"
        )));
    }
    let row = Row {
        record: cells[0].to_owned(),
        position: number(cells[1], "interval index", line)?,
        label: cells[2].to_owned(),
        begin: number(cells[3], "interval start", line)?,
        finish: number(cells[4], "interval end", line)?,
        opening: maybe_number(cells[5], "opening stock", line)?,
        receipt: maybe_number(cells[6], "replenishment", line)?,
        use_amount: maybe_number(cells[7], "consumption", line)?,
        removal: maybe_number(cells[8], "loss", line)?,
        closing: maybe_number(cells[9], "closing stock", line)?,
        owner: cells[10].to_owned(),
        reference: cells[11].to_owned(),
        classification: cells[12].to_owned(),
    };
    for (name, value) in [
        ("record id", row.record.as_str()),
        ("interval id", row.label.as_str()),
        ("owner", row.owner.as_str()),
    ] {
        if !identifier(value) {
            return Err(ReferenceError(format!("line {line}: invalid {name}")));
        }
    }
    if row.begin >= row.finish {
        return Err(ReferenceError(format!(
            "line {line}: interval duration must be positive"
        )));
    }
    if row.reference.is_empty() || row.reference.len() > 512 {
        return Err(ReferenceError(format!(
            "line {line}: invalid source reference"
        )));
    }
    match row.classification.as_str() {
        "KNOWN" => {
            if row.opening.is_none()
                || row.receipt.is_none()
                || row.use_amount.is_none()
                || row.removal.is_none()
                || row.closing.is_none()
            {
                return Err(ReferenceError(format!(
                    "line {line}: known row contains an unknown term"
                )));
            }
        }
        "UNKNOWN" => {
            if row.closing.is_some() {
                return Err(ReferenceError(format!(
                    "line {line}: unknown row cannot assert closing stock"
                )));
            }
            if row.opening.is_some()
                && row.receipt.is_some()
                && row.use_amount.is_some()
                && row.removal.is_some()
            {
                return Err(ReferenceError(format!(
                    "line {line}: unknown row has no unknown flow term"
                )));
            }
        }
        _ => {
            return Err(ReferenceError(format!(
                "line {line}: unrecognised evidence classification"
            )));
        }
    }
    Ok(row)
}

fn decode_dataset(source: &str) -> Result<Dataset, ReferenceError> {
    let mut properties = BTreeMap::<String, String>::new();
    let mut rows = Vec::new();
    let mut seen = BTreeSet::new();
    let mut header = false;

    for (index, raw) in source.lines().enumerate() {
        let line_no = index + 1;
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            if let Some((key, value)) = comment.trim().split_once('=') {
                if properties.insert(key.trim().to_owned(), value.trim().to_owned()).is_some() {
                    return Err(ReferenceError(format!("line {line_no}: repeated metadata")));
                }
            }
            continue;
        }
        if !header {
            if line != HEADER {
                return Err(ReferenceError(format!("line {line_no}: unexpected header")));
            }
            header = true;
            continue;
        }
        let row = decode_row(line, line_no)?;
        if !seen.insert(row.record.clone()) {
            return Err(ReferenceError(format!("line {line_no}: duplicate record id")));
        }
        rows.push(row);
    }
    if !header || rows.is_empty() {
        return Err(ReferenceError("missing data header or rows".to_owned()));
    }
    let mut extract = |key: &str| {
        let value = properties
            .remove(key)
            .ok_or_else(|| ReferenceError(format!("missing metadata {key}")))?;
        if !identifier(&value) {
            return Err(ReferenceError(format!("invalid metadata {key}")));
        }
        Ok(value)
    };
    let scenario = extract("scenario_id")?;
    let resource = extract("resource_id")?;
    let measure = extract("resource_unit")?;
    let clock = extract("time_basis_id")?;
    if let Some(extra) = properties.keys().next() {
        return Err(ReferenceError(format!("unexpected metadata {extra}")));
    }
    rows.sort_by_key(|row| row.position);
    for (offset, row) in rows.iter().enumerate() {
        if row.position != offset as u64 {
            return Err(ReferenceError(format!(
                "expected consecutive interval index {offset}, found {}",
                row.position
            )));
        }
        if offset > 0 && rows[offset - 1].finish != row.begin {
            return Err(ReferenceError(format!(
                "temporal boundary discontinuity: {} -> {}",
                rows[offset - 1].label, row.label
            )));
        }
    }
    Ok(Dataset {
        scenario,
        resource,
        measure,
        clock,
        rows,
    })
}

#[derive(Clone, Debug)]
struct Assessment {
    id: String,
    begin: u64,
    finish: u64,
    opening: Option<u64>,
    receipt: Option<u64>,
    use_amount: Option<u64>,
    removal: Option<u64>,
    closing: Option<u64>,
    state: String,
    owner: String,
    reference: String,
}

fn plus(left: u64, right: u64, label: &str, interval: &str) -> Result<u64, ReferenceError> {
    left.checked_add(right).ok_or_else(|| {
        ReferenceError(format!("{label} overflow in interval {interval}"))
    })
}

fn assess(dataset: &Dataset) -> Result<String, ReferenceError> {
    let mut reports = Vec::new();
    let mut preceding_balance: Option<u64> = None;
    let mut after_unknown = false;
    let mut has_unknown = false;
    let mut has_mismatch = false;
    let mut initial_balance: Option<u64> = None;
    let mut terminal_balance: Option<u64> = None;
    let mut uncertain_ids = Vec::new();

    for (position, row) in dataset.rows.iter().enumerate() {
        let mut assessment = "BALANCED";
        if row.classification == "UNKNOWN" {
            has_unknown = true;
            if let (Some(previous), Some(current_opening)) = (preceding_balance, row.opening) {
                if previous != current_opening {
                    has_mismatch = true;
                    assessment = "NOT_ESTIMABLE_CONTINUITY_MISMATCH";
                } else {
                    assessment = "NOT_ESTIMABLE";
                }
            } else {
                assessment = "NOT_ESTIMABLE";
            }
            after_unknown = true;
            uncertain_ids.push(row.label.clone());
            preceding_balance = None;
            terminal_balance = None;
        } else {
            if after_unknown {
                return Err(ReferenceError(format!(
                    "known interval {} follows an unresolved interval",
                    row.label
                )));
            }
            let opening = row.opening.expect("known opening quantity");
            let receipt = row.receipt.expect("known receipt quantity");
            let consumed = row.use_amount.expect("known consumption quantity");
            let removed = row.removal.expect("known loss quantity");
            let closing = row.closing.expect("known closing quantity");

            if position == 0 {
                initial_balance = Some(opening);
            } else if preceding_balance != Some(opening) {
                assessment = "CONTINUITY_MISMATCH";
                has_mismatch = true;
            }

            let available = plus(opening, receipt, "available inventory", &row.label)?;
            let outgoing = plus(consumed, removed, "outgoing inventory", &row.label)?;
            if available.checked_sub(outgoing) != Some(closing) {
                assessment = "FLOW_MISMATCH";
                has_mismatch = true;
            }
            preceding_balance = Some(closing);
            terminal_balance = Some(closing);
        }
        reports.push(Assessment {
            id: row.label.clone(),
            begin: row.begin,
            finish: row.finish,
            opening: row.opening,
            receipt: row.receipt,
            use_amount: row.use_amount,
            removal: row.removal,
            closing: row.closing,
            state: assessment.to_owned(),
            owner: row.owner.clone(),
            reference: row.reference.clone(),
        });
    }

    let disposition = if has_unknown {
        "NOT_ESTIMABLE"
    } else if has_mismatch {
        "NOT_BALANCED"
    } else {
        "BALANCED_FOR_DECLARED_HORIZON"
    };
    let mut result = vec![
        "report_schema=fsre-resource-stock-flow-report.v0.1".to_owned(),
        format!("scenario_id={}", dataset.scenario),
        format!("resource_id={}", dataset.resource),
        format!("resource_unit={}", dataset.measure),
        format!("time_basis_id={}", dataset.clock),
        format!("intervals={}", reports.len()),
        format!("stock_flow_disposition={disposition}"),
        format!(
            "opening_stock={}",
            initial_balance.map(|n| n.to_string()).unwrap_or_else(|| "UNKNOWN".to_owned())
        ),
        format!(
            "closing_stock={}",
            terminal_balance.map(|n| n.to_string()).unwrap_or_else(|| "UNKNOWN".to_owned())
        ),
    ];
    for row in &reports {
        result.push(format!(
            "interval={}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}",
            row.id,
            row.begin,
            row.finish,
            row.opening.map(|n| n.to_string()).unwrap_or_else(|| "UNKNOWN".to_owned()),
            row.receipt.map(|n| n.to_string()).unwrap_or_else(|| "UNKNOWN".to_owned()),
            row.use_amount.map(|n| n.to_string()).unwrap_or_else(|| "UNKNOWN".to_owned()),
            row.removal.map(|n| n.to_string()).unwrap_or_else(|| "UNKNOWN".to_owned()),
            row.closing.map(|n| n.to_string()).unwrap_or_else(|| "UNKNOWN".to_owned()),
            row.state,
            row.owner,
            row.reference
        ));
    }
    result.push(format!(
        "unreconciled_intervals={}",
        if uncertain_ids.is_empty() {
            "NONE".to_owned()
        } else {
            uncertain_ids.join(",")
        }
    ));
    result.push("source_authenticity=NOT_AUTHENTICATED_BY_THIS_TOOL".to_owned());
    result.push("execution_authority=NONE".to_owned());
    result.push("operational_actuation=NOT_AUTHORISED".to_owned());
    Ok(result.join("\n") + "\n")
}

pub fn validate_and_report(source: &str) -> Result<String, ReferenceError> {
    assess(&decode_dataset(source)?)
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str =
        include_str!("../../fsre-resource-stock-flow/fixtures/synthetic-balanced.psv");

    #[test]
    fn independent_stock_flow_reference_matches_three_period_series() {
        let report = validate_and_report(FIXTURE).expect("fixture");
        assert!(report.contains("stock_flow_disposition=BALANCED_FOR_DECLARED_HORIZON"));
        assert!(report.contains("opening_stock=100"));
        assert!(report.contains("closing_stock=50"));
    }

    #[test]
    fn consumption_overdraw_is_not_a_valid_balance() {
        let changed = FIXTURE.replace(
            "stock-002|2|hour-02|7200|10800|60|0|10|0|50",
            "stock-002|2|hour-02|7200|10800|60|0|70|0|0",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("overdraw is a reportable mismatch");
        assert!(report.contains("stock_flow_disposition=NOT_BALANCED"));
        assert!(report.contains("FLOW_MISMATCH"));
    }

    #[test]
    fn unknown_stock_breaks_future_reconciliation() {
        let changed = FIXTURE.replace(
            "stock-001|1|hour-01|3600|7200|70|50|60|0|60|operator|synthetic:stock:hour-01|KNOWN",
            "stock-001|1|hour-01|3600|7200|70||60|0||operator|synthetic:stock:hour-01|UNKNOWN",
        );
        assert_ne!(changed, FIXTURE);
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("known interval hour-02 follows an unresolved interval"));
    }

    #[test]
    fn unknown_row_cannot_claim_closing_stock() {
        let changed = FIXTURE.replace(
            "stock-001|1|hour-01|3600|7200|70|50|60|0|60|operator|synthetic:stock:hour-01|KNOWN",
            "stock-001|1|hour-01|3600|7200|70||60|0|60|operator|synthetic:stock:hour-01|UNKNOWN",
        );
        assert_ne!(changed, FIXTURE);
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("unknown row cannot assert closing stock"));
    }

    #[test]
    fn row_permutation_does_not_change_the_report() {
        let mut data = Vec::new();
        let mut metadata = Vec::new();
        for line in FIXTURE.lines() {
            if line.starts_with('#') {
                metadata.push(line);
            } else if line == HEADER {
                data.push(line);
            } else if !line.is_empty() {
                data.push(line);
            }
        }
        let heading = data.remove(0);
        data.reverse();
        let reordered = format!(
            "{}\n{}\n{}\n",
            metadata.join("\n"),
            heading,
            data.join("\n")
        );
        assert_eq!(
            validate_and_report(FIXTURE).expect("original"),
            validate_and_report(&reordered).expect("reordered")
        );
    }

    #[test]
    fn independently_generated_stock_flow_arithmetic_sweep() {
        let mut checked = 0_u64;
        for opening in 0..=8_u64 {
            for receipt in 0..=8_u64 {
                for used in 0..=8_u64 {
                    for removed in 0..=8_u64 {
                        let available = opening + receipt;
                        let outgoing = used + removed;
                        if outgoing > available {
                            continue;
                        }
                        let closing = available - outgoing;
                        let input = format!(
                            "# scenario_id=oracle-sweep\n# resource_id=oracle-resource\n# resource_unit=unit\n# time_basis_id=oracle-clock\n{HEADER}\nrow-0|0|interval-0|0|60|{opening}|{receipt}|{used}|{removed}|{closing}|operator|synthetic:oracle-sweep|KNOWN\n"
                        );
                        let output = validate_and_report(&input).expect("balanced oracle case");
                        assert!(output.contains("stock_flow_disposition=BALANCED_FOR_DECLARED_HORIZON"));
                        let wrong = closing + 1;
                        let corrupt = input.replace(
                            &format!("|{removed}|{closing}|operator|synthetic:oracle-sweep|KNOWN"),
                            &format!("|{removed}|{wrong}|operator|synthetic:oracle-sweep|KNOWN"),
                        );
                        let rejected = validate_and_report(&corrupt).expect("oracle mismatch report");
                        assert!(rejected.contains("stock_flow_disposition=NOT_BALANCED"));
                        checked += 1;
                    }
                }
            }
        }
        assert!(checked >= 3_000, "expected broad oracle sweep, got {checked}");
    }


}
