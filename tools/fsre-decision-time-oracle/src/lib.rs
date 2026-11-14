#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

const COLUMN_LINE: &str = "record_id|evidence_id|revision|claim_id|subject_id|predicate|valid_from|valid_until|observed_at|recorded_at|revoked_at|state|source_owner|evidence_ref";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReferenceError(pub String);

impl std::fmt::Display for ReferenceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for ReferenceError {}

#[derive(Clone, Debug)]
struct Version {
    row_id: String,
    evidence: String,
    version: u64,
    claim: String,
    subject: String,
    predicate: String,
    begins: u64,
    ends: u64,
    source_time: u64,
    database_time: u64,
    withdrawal: Option<u64>,
    status: String,
    owner: String,
    reference: String,
}

#[derive(Clone, Debug)]
struct Replay {
    scenario: String,
    decision: String,
    cutoff: u64,
    snapshot: String,
    clock: String,
    rows: BTreeMap<String, Vec<Version>>,
}

fn identifier(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 160
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric()
                || matches!(byte, b'_' | b'-' | b'.' | b':' | b'/')
        })
}

fn integer(value: &str, field: &str, line: usize) -> Result<u64, ReferenceError> {
    value.parse::<u64>().map_err(|error| {
        ReferenceError(format!("line {line}: invalid {field}: {error}"))
    })
}

fn maybe_integer(value: &str, field: &str, line: usize) -> Result<Option<u64>, ReferenceError> {
    if value.is_empty() {
        Ok(None)
    } else {
        integer(value, field, line).map(Some)
    }
}

fn read_version(line: &str, line_number: usize) -> Result<Version, ReferenceError> {
    let cells: Vec<&str> = line.split('|').map(str::trim).collect();
    if cells.len() != 14 {
        return Err(ReferenceError(format!(
            "line {line_number}: expected 14 evidence-version columns"
        )));
    }
    let row = Version {
        row_id: cells[0].to_owned(),
        evidence: cells[1].to_owned(),
        version: integer(cells[2], "revision", line_number)?,
        claim: cells[3].to_owned(),
        subject: cells[4].to_owned(),
        predicate: cells[5].to_owned(),
        begins: integer(cells[6], "valid_from", line_number)?,
        ends: integer(cells[7], "valid_until", line_number)?,
        source_time: integer(cells[8], "observed_at", line_number)?,
        database_time: integer(cells[9], "recorded_at", line_number)?,
        withdrawal: maybe_integer(cells[10], "revoked_at", line_number)?,
        status: cells[11].to_owned(),
        owner: cells[12].to_owned(),
        reference: cells[13].to_owned(),
    };
    for (field, value) in [
        ("record_id", row.row_id.as_str()),
        ("evidence_id", row.evidence.as_str()),
        ("claim_id", row.claim.as_str()),
        ("subject_id", row.subject.as_str()),
        ("predicate", row.predicate.as_str()),
        ("source_owner", row.owner.as_str()),
    ] {
        if !identifier(value) {
            return Err(ReferenceError(format!(
                "line {line_number}: invalid {field}"
            )));
        }
    }
    if row.version == 0 {
        return Err(ReferenceError(format!(
            "line {line_number}: revision must be positive"
        )));
    }
    if row.begins >= row.ends {
        return Err(ReferenceError(format!(
            "line {line_number}: valid-time interval must have positive duration"
        )));
    }
    if row.source_time > row.database_time {
        return Err(ReferenceError(format!(
            "line {line_number}: evidence cannot be recorded before it is observed"
        )));
    }
    if row.withdrawal.is_some_and(|time| time < row.begins) {
        return Err(ReferenceError(format!(
            "line {line_number}: revocation precedes the evidence valid-time interval"
        )));
    }
    if row.reference.is_empty() || row.reference.len() > 512 {
        return Err(ReferenceError(format!(
            "line {line_number}: invalid evidence reference"
        )));
    }
    if !matches!(row.status.as_str(), "ASSERTED" | "UNKNOWN") {
        return Err(ReferenceError(format!(
            "line {line_number}: unsupported evidence state"
        )));
    }
    Ok(row)
}

fn metadata_value(metadata: &mut BTreeMap<String, String>, key: &str) -> Result<String, ReferenceError> {
    let value = metadata
        .remove(key)
        .ok_or_else(|| ReferenceError(format!("missing metadata key {key}")))?;
    if !identifier(&value) {
        return Err(ReferenceError(format!("invalid metadata key {key}")));
    }
    Ok(value)
}

fn decode(source: &str) -> Result<Replay, ReferenceError> {
    let mut metadata = BTreeMap::<String, String>::new();
    let mut grouped = BTreeMap::<String, Vec<Version>>::new();
    let mut record_ids = BTreeSet::new();
    let mut header = false;

    for (offset, raw) in source.lines().enumerate() {
        let line_number = offset + 1;
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            if let Some((name, value)) = comment.trim().split_once('=') {
                if metadata.insert(name.trim().to_owned(), value.trim().to_owned()).is_some() {
                    return Err(ReferenceError(format!(
                        "line {line_number}: duplicate metadata key"
                    )));
                }
            }
            continue;
        }
        if !header {
            if line != COLUMN_LINE {
                return Err(ReferenceError(format!(
                    "line {line_number}: unexpected evidence header"
                )));
            }
            header = true;
            continue;
        }
        let row = read_version(line, line_number)?;
        if !record_ids.insert(row.row_id.clone()) {
            return Err(ReferenceError(format!(
                "line {line_number}: duplicate record_id {}",
                row.row_id
            )));
        }
        grouped.entry(row.evidence.clone()).or_default().push(row);
    }
    if !header || grouped.is_empty() {
        return Err(ReferenceError("missing evidence header or data".to_owned()));
    }

    let scenario = metadata_value(&mut metadata, "scenario_id")?;
    let decision = metadata_value(&mut metadata, "decision_id")?;
    let cutoff = metadata_value(&mut metadata, "decision_time")?
        .parse::<u64>()
        .map_err(|error| ReferenceError(format!("invalid decision_time: {error}")))?;
    let snapshot = metadata_value(&mut metadata, "snapshot_id")?;
    let clock = metadata_value(&mut metadata, "time_basis_id")?;
    if let Some(extra) = metadata.keys().next() {
        return Err(ReferenceError(format!("unknown metadata key {extra}")));
    }

    for (evidence_id, versions) in &mut grouped {
        versions.sort_by_key(|row| row.version);
        let first = &versions[0];
        let mut seen_versions = BTreeSet::new();
        let mut transaction_cursor = 0_u64;
        for row in versions.iter() {
            if !seen_versions.insert(row.version) {
                return Err(ReferenceError(format!(
                    "evidence {evidence_id} has duplicate revision {}",
                    row.version
                )));
            }
            if row.claim != first.claim
                || row.subject != first.subject
                || row.predicate != first.predicate
                || row.owner != first.owner
            {
                return Err(ReferenceError(format!(
                    "evidence {evidence_id} changes claim identity, subject, predicate or owner across revisions"
                )));
            }
            if row.database_time < transaction_cursor {
                return Err(ReferenceError(format!(
                    "evidence {evidence_id} transaction time moves backwards across revisions"
                )));
            }
            transaction_cursor = row.database_time;
        }
    }

    Ok(Replay {
        scenario,
        decision,
        cutoff,
        snapshot,
        clock,
        rows: grouped,
    })
}

fn disposition(row: &Version, cutoff: u64) -> &'static str {
    if cutoff < row.begins {
        "NOT_YET_VALID"
    } else if cutoff >= row.ends {
        "EXPIRED"
    } else if row.withdrawal.is_some_and(|revoked| revoked <= cutoff) {
        "REVOKED"
    } else if row.status == "UNKNOWN" {
        "SOURCE_UNKNOWN"
    } else {
        "KNOWN_VALID"
    }
}

fn replay(input: &Replay) -> Result<String, ReferenceError> {
    let mut reports = Vec::new();
    let mut future_total = 0_usize;
    let mut missing_as_of = false;
    let mut unknown_source = false;
    let mut invalid_time = false;

    for (evidence_id, versions) in &input.rows {
        let future_count = versions
            .iter()
            .filter(|row| row.source_time > input.cutoff || row.database_time > input.cutoff)
            .count();
        future_total += future_count;

        let current = versions
            .iter()
            .filter(|row| row.source_time <= input.cutoff && row.database_time <= input.cutoff)
            .max_by_key(|row| (row.version, row.database_time));

        match current {
            None => {
                missing_as_of = true;
                let row = versions.iter().min_by_key(|version| version.version).expect("evidence has at least one version");
                reports.push((
                    evidence_id.clone(),
                    None,
                    row.claim.clone(),
                    row.subject.clone(),
                    row.predicate.clone(),
                    "NO_AS_OF_VERSION".to_owned(),
                    row.owner.clone(),
                    row.reference.clone(),
                    future_count,
                ));
            }
            Some(row) => {
                let state = disposition(row, input.cutoff);
                unknown_source |= state == "SOURCE_UNKNOWN";
                invalid_time |= matches!(state, "NOT_YET_VALID" | "EXPIRED" | "REVOKED");
                reports.push((
                    evidence_id.clone(),
                    Some(row.version),
                    row.claim.clone(),
                    row.subject.clone(),
                    row.predicate.clone(),
                    state.to_owned(),
                    row.owner.clone(),
                    row.reference.clone(),
                    future_count,
                ));
            }
        }
    }

    let overall = if missing_as_of || unknown_source {
        "NOT_ESTIMABLE"
    } else if invalid_time {
        "EVIDENCE_NOT_VALID_AT_DECISION"
    } else {
        "ALL_SUBMITTED_EVIDENCE_VALID_AT_DECISION"
    };
    let selected = reports.iter().filter(|row| row.1.is_some()).count();
    let mut lines = vec![
        "report_schema=fsre-decision-time-replay-report.v0.1".to_owned(),
        format!("scenario_id={}", input.scenario),
        format!("decision_id={}", input.decision),
        format!("decision_time={}", input.cutoff),
        format!("time_basis_id={}", input.clock),
        format!("evidence_snapshot_id={}", input.snapshot),
        format!("submitted_evidence_ids={}", reports.len()),
        format!("selected_as_of_versions={selected}"),
        format!("future_revisions_excluded={future_total}"),
        format!("decision_evidence_disposition={overall}"),
    ];
    for row in reports {
        let revision = row.1.map(|value| value.to_string()).unwrap_or_else(|| "NONE".to_owned());
        lines.push(format!(
            "evidence={}|{}|{}|{}|{}|{}|{}|{}|future_revisions_excluded={}",
            row.0, revision, row.2, row.3, row.4, row.5, row.6, row.7, row.8
        ));
    }
    lines.push("information_frontier=OBSERVED_AT_AND_RECORDED_AT_LE_DECISION_TIME".to_owned());
    lines.push("claim_completeness=NOT_ESTABLISHED_BY_THIS_TOOL".to_owned());
    lines.push("source_authenticity=NOT_AUTHENTICATED_BY_THIS_TOOL".to_owned());
    lines.push("execution_authority=NONE".to_owned());
    lines.push("operational_actuation=NOT_AUTHORISED".to_owned());
    Ok(lines.join("\n") + "\n")
}

pub fn validate_and_report(source: &str) -> Result<String, ReferenceError> {
    replay(&decode(source)?)
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str =
        include_str!("../../fsre-decision-time-replay/fixtures/synthetic-decision.psv");

    #[test]
    fn independent_decision_time_reference_reproduces_as_of_fixture() {
        let report = validate_and_report(FIXTURE).expect("fixture");
        assert!(report.contains("decision_evidence_disposition=ALL_SUBMITTED_EVIDENCE_VALID_AT_DECISION"));
        assert!(report.contains("selected_as_of_versions=3"));
        assert!(report.contains("future_revisions_excluded=1"));
        assert!(report.contains("evidence=stock_receipt|1|claim-diesel-stock|diesel-tank-01|available_stock|KNOWN_VALID|operator|synthetic:stock:r1|future_revisions_excluded=1"));
    }

    #[test]
    fn late_transaction_version_is_excluded() {
        let changed = FIXTURE.replace(
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|1050|960|970||ASSERTED|transport-owner",
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|1050|960|1010||ASSERTED|transport-owner",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("late ingestion");
        assert!(report.contains("NO_AS_OF_VERSION"));
        assert!(report.contains("decision_evidence_disposition=NOT_ESTIMABLE"));
    }

    #[test]
    fn expiry_and_revocation_are_distinct_negative_states() {
        let expired = FIXTURE.replace(
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|1050|960|970||ASSERTED",
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|990|960|970||ASSERTED",
        );
        assert!(validate_and_report(&expired)
            .expect("expired")
            .contains("route_access|1|claim-road-access|road-east|passable|EXPIRED"));

        let revoked = FIXTURE.replace(
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|1050|960|970||ASSERTED",
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|1050|960|970|990|ASSERTED",
        );
        assert!(validate_and_report(&revoked)
            .expect("revoked")
            .contains("route_access|1|claim-road-access|road-east|passable|REVOKED"));
    }

    #[test]
    fn later_revocation_does_not_rewrite_the_prior_decision() {
        let future = "route-r2|route_access|2|claim-road-access|road-east|passable|950|1050|980|1210|990|ASSERTED|transport-owner|synthetic:route:r2";
        let changed = format!("{FIXTURE}{future}\n");
        let report = validate_and_report(&changed).expect("future revocation");
        assert!(report.contains("ALL_SUBMITTED_EVIDENCE_VALID_AT_DECISION"));
        assert!(report.contains("route_access|1|claim-road-access|road-east|passable|KNOWN_VALID|transport-owner|synthetic:route:r1|future_revisions_excluded=1"));
    }

    #[test]
    fn revision_identity_changes_are_rejected() {
        let changed = FIXTURE.replace(
            "stock-r2|stock_receipt|2|claim-diesel-stock|diesel-tank-01",
            "stock-r2|stock_receipt|2|different-claim|diesel-tank-01",
        );
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("changes claim identity"));
    }

    #[test]
    fn record_before_observation_is_rejected() {
        let changed = FIXTURE.replace(
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|1050|960|970",
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|1050|960|950",
        );
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("recorded before it is observed"));
    }

    #[test]
    fn all_future_versions_keep_a_stable_no_as_of_representative() {
        let changed = FIXTURE.replace(
            "stock-r1|stock_receipt|1|claim-diesel-stock|diesel-tank-01|available_stock|800|1100|820|830||ASSERTED",
            "stock-r1|stock_receipt|1|claim-diesel-stock|diesel-tank-01|available_stock|800|1100|1200|1205||ASSERTED",
        );
        assert_ne!(changed, FIXTURE);
        let mut meta = Vec::new();
        let mut rows = Vec::new();
        for line in changed.lines() {
            if line.starts_with('#') { meta.push(line); }
            else if !line.is_empty() { rows.push(line); }
        }
        let header = rows.remove(0);
        rows.reverse();
        let permuted = format!("{}\n{}\n{}\n", meta.join("\n"), header, rows.join("\n"));
        let first = validate_and_report(&changed).expect("original");
        let second = validate_and_report(&permuted).expect("permuted");
        assert_eq!(first, second);
        assert!(first.contains("stock_receipt|NONE|claim-diesel-stock|diesel-tank-01|available_stock|NO_AS_OF_VERSION|operator|synthetic:stock:r1"));
    }

    #[test]
    fn row_permutation_has_identical_replay() {
        let mut metadata = Vec::new();
        let mut records = Vec::new();
        for line in FIXTURE.lines() {
            if line.starts_with('#') { metadata.push(line); }
            else if !line.is_empty() { records.push(line); }
        }
        let header = records.remove(0);
        records.reverse();
        let changed = format!("{}\n{}\n{}\n", metadata.join("\n"), header, records.join("\n"));
        assert_eq!(
            validate_and_report(FIXTURE).expect("original"),
            validate_and_report(&changed).expect("permuted")
        );
    }
}
