#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

const HEADER: &str = "record_id|evidence_id|revision|claim_id|subject_id|predicate|valid_from|valid_until|observed_at|recorded_at|revoked_at|state|source_owner|evidence_ref";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InputError(pub String);

impl std::fmt::Display for InputError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for InputError {}

#[derive(Clone, Debug)]
struct EvidenceVersion {
    record_id: String,
    evidence_id: String,
    revision: u64,
    claim_id: String,
    subject_id: String,
    predicate: String,
    valid_from: u64,
    valid_until: u64,
    observed_at: u64,
    recorded_at: u64,
    revoked_at: Option<u64>,
    state: String,
    source_owner: String,
    evidence_ref: String,
}

#[derive(Clone, Debug)]
struct ReplayInput {
    scenario_id: String,
    decision_id: String,
    decision_time: u64,
    snapshot_id: String,
    time_basis_id: String,
    versions: Vec<EvidenceVersion>,
}

fn valid_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 160
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric()
                || matches!(byte, b'_' | b'-' | b'.' | b':' | b'/')
        })
}

fn parse_u64(value: &str, label: &str, line: usize) -> Result<u64, InputError> {
    value.parse::<u64>().map_err(|error| {
        InputError(format!("line {line}: invalid {label}: {error}"))
    })
}

fn parse_optional_u64(value: &str, label: &str, line: usize) -> Result<Option<u64>, InputError> {
    if value.is_empty() {
        Ok(None)
    } else {
        parse_u64(value, label, line).map(Some)
    }
}

fn parse_version(line: &str, line_number: usize) -> Result<EvidenceVersion, InputError> {
    let fields: Vec<&str> = line.split('|').map(str::trim).collect();
    if fields.len() != 14 {
        return Err(InputError(format!(
            "line {line_number}: expected 14 columns, received {}",
            fields.len()
        )));
    }
    let version = EvidenceVersion {
        record_id: fields[0].to_owned(),
        evidence_id: fields[1].to_owned(),
        revision: parse_u64(fields[2], "revision", line_number)?,
        claim_id: fields[3].to_owned(),
        subject_id: fields[4].to_owned(),
        predicate: fields[5].to_owned(),
        valid_from: parse_u64(fields[6], "valid_from", line_number)?,
        valid_until: parse_u64(fields[7], "valid_until", line_number)?,
        observed_at: parse_u64(fields[8], "observed_at", line_number)?,
        recorded_at: parse_u64(fields[9], "recorded_at", line_number)?,
        revoked_at: parse_optional_u64(fields[10], "revoked_at", line_number)?,
        state: fields[11].to_owned(),
        source_owner: fields[12].to_owned(),
        evidence_ref: fields[13].to_owned(),
    };
    for (label, value) in [
        ("record_id", version.record_id.as_str()),
        ("evidence_id", version.evidence_id.as_str()),
        ("claim_id", version.claim_id.as_str()),
        ("subject_id", version.subject_id.as_str()),
        ("predicate", version.predicate.as_str()),
        ("source_owner", version.source_owner.as_str()),
    ] {
        if !valid_id(value) {
            return Err(InputError(format!(
                "line {line_number}: invalid {label}"
            )));
        }
    }
    if version.revision == 0 {
        return Err(InputError(format!("line {line_number}: revision must be positive")));
    }
    if version.valid_from >= version.valid_until {
        return Err(InputError(format!(
            "line {line_number}: valid-time interval must have positive duration"
        )));
    }
    if version.observed_at > version.recorded_at {
        return Err(InputError(format!(
            "line {line_number}: evidence cannot be recorded before it is observed"
        )));
    }
    if version.revoked_at.is_some_and(|revoked| revoked < version.valid_from) {
        return Err(InputError(format!(
            "line {line_number}: revocation precedes the evidence valid-time interval"
        )));
    }
    if version.evidence_ref.is_empty() || version.evidence_ref.len() > 512 {
        return Err(InputError(format!(
            "line {line_number}: missing or invalid evidence_ref"
        )));
    }
    if !matches!(version.state.as_str(), "ASSERTED" | "UNKNOWN") {
        return Err(InputError(format!(
            "line {line_number}: unsupported evidence state {}",
            version.state
        )));
    }
    Ok(version)
}

fn parse_input(source: &str) -> Result<ReplayInput, InputError> {
    let mut metadata = BTreeMap::<String, String>::new();
    let mut versions = Vec::new();
    let mut record_ids = BTreeSet::new();
    let mut header_seen = false;

    for (index, raw) in source.lines().enumerate() {
        let line_number = index + 1;
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            if let Some((key, value)) = comment.trim().split_once('=') {
                if metadata.insert(key.trim().to_owned(), value.trim().to_owned()).is_some() {
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
                    "line {line_number}: unexpected evidence-version header"
                )));
            }
            header_seen = true;
            continue;
        }
        let version = parse_version(line, line_number)?;
        if !record_ids.insert(version.record_id.clone()) {
            return Err(InputError(format!(
                "line {line_number}: duplicate record_id {}",
                version.record_id
            )));
        }
        versions.push(version);
    }
    if !header_seen || versions.is_empty() {
        return Err(InputError("missing evidence header or versions".to_owned()));
    }
    let mut take_id = |key: &str| {
        let value = metadata
            .remove(key)
            .ok_or_else(|| InputError(format!("missing metadata key {key}")))?;
        if !valid_id(&value) {
            return Err(InputError(format!("invalid metadata key {key}")));
        }
        Ok(value)
    };
    let scenario_id = take_id("scenario_id")?;
    let decision_id = take_id("decision_id")?;
    let decision_time = take_id("decision_time")?
        .parse::<u64>()
        .map_err(|error| InputError(format!("invalid decision_time: {error}")))?;
    let snapshot_id = take_id("snapshot_id")?;
    let time_basis_id = take_id("time_basis_id")?;
    if let Some(extra) = metadata.keys().next() {
        return Err(InputError(format!("unknown metadata key {extra}")));
    }

    let mut by_evidence = BTreeMap::<String, Vec<&EvidenceVersion>>::new();
    for version in &versions {
        by_evidence
            .entry(version.evidence_id.clone())
            .or_default()
            .push(version);
    }
    for (evidence_id, rows) in by_evidence {
        let first = rows[0];
        let mut ordered = rows;
        ordered.sort_by_key(|row| row.revision);
        let mut prior_revision = 0_u64;
        let mut prior_recorded_at = 0_u64;
        for row in ordered {
            if row.claim_id != first.claim_id
                || row.subject_id != first.subject_id
                || row.predicate != first.predicate
                || row.source_owner != first.source_owner
            {
                return Err(InputError(format!(
                    "evidence {evidence_id} changes claim identity, subject, predicate or owner across revisions"
                )));
            }
            if row.revision == prior_revision {
                return Err(InputError(format!(
                    "evidence {evidence_id} has duplicate revision {}",
                    row.revision
                )));
            }
            if row.recorded_at < prior_recorded_at {
                return Err(InputError(format!(
                    "evidence {evidence_id} transaction time moves backwards across revisions"
                )));
            }
            prior_revision = row.revision;
            prior_recorded_at = row.recorded_at;
        }
    }

    Ok(ReplayInput {
        scenario_id,
        decision_id,
        decision_time,
        snapshot_id,
        time_basis_id,
        versions,
    })
}

#[derive(Clone, Debug)]
struct EvidenceResult {
    evidence_id: String,
    revision: Option<u64>,
    claim_id: String,
    subject_id: String,
    predicate: String,
    disposition: String,
    owner: String,
    reference: String,
    future_revisions_excluded: usize,
}

fn classify(version: &EvidenceVersion, decision_time: u64) -> &'static str {
    if version.valid_from > decision_time {
        "NOT_YET_VALID"
    } else if version.valid_until <= decision_time {
        "EXPIRED"
    } else if version.revoked_at.is_some_and(|revoked| revoked <= decision_time) {
        "REVOKED"
    } else if version.state == "UNKNOWN" {
        "SOURCE_UNKNOWN"
    } else {
        "KNOWN_VALID"
    }
}

fn evaluate(input: &ReplayInput) -> Result<String, InputError> {
    let mut by_evidence = BTreeMap::<String, Vec<&EvidenceVersion>>::new();
    for version in &input.versions {
        by_evidence
            .entry(version.evidence_id.clone())
            .or_default()
            .push(version);
    }

    let mut results = Vec::<EvidenceResult>::new();
    let mut future_revisions_total = 0_usize;
    let mut no_as_of_version = false;
    let mut source_unknown = false;
    let mut temporally_invalid = false;

    for (evidence_id, versions) in by_evidence {
        let future_revisions_excluded = versions
            .iter()
            .filter(|version| {
                version.observed_at > input.decision_time
                    || version.recorded_at > input.decision_time
            })
            .count();
        future_revisions_total += future_revisions_excluded;

        let selected = versions
            .iter()
            .copied()
            .filter(|version| {
                version.observed_at <= input.decision_time
                    && version.recorded_at <= input.decision_time
            })
            .max_by_key(|version| (version.revision, version.recorded_at));

        match selected {
            None => {
                no_as_of_version = true;
                let example = versions.iter().min_by_key(|version| version.revision).expect("evidence has at least one version");
                results.push(EvidenceResult {
                    evidence_id,
                    revision: None,
                    claim_id: example.claim_id.clone(),
                    subject_id: example.subject_id.clone(),
                    predicate: example.predicate.clone(),
                    disposition: "NO_AS_OF_VERSION".to_owned(),
                    owner: example.source_owner.clone(),
                    reference: example.evidence_ref.clone(),
                    future_revisions_excluded,
                });
            }
            Some(version) => {
                let disposition = classify(version, input.decision_time);
                source_unknown |= disposition == "SOURCE_UNKNOWN";
                temporally_invalid |= matches!(
                    disposition,
                    "NOT_YET_VALID" | "EXPIRED" | "REVOKED"
                );
                results.push(EvidenceResult {
                    evidence_id,
                    revision: Some(version.revision),
                    claim_id: version.claim_id.clone(),
                    subject_id: version.subject_id.clone(),
                    predicate: version.predicate.clone(),
                    disposition: disposition.to_owned(),
                    owner: version.source_owner.clone(),
                    reference: version.evidence_ref.clone(),
                    future_revisions_excluded,
                });
            }
        }
    }

    let disposition = if no_as_of_version || source_unknown {
        "NOT_ESTIMABLE"
    } else if temporally_invalid {
        "EVIDENCE_NOT_VALID_AT_DECISION"
    } else {
        "ALL_SUBMITTED_EVIDENCE_VALID_AT_DECISION"
    };
    let selected_count = results.iter().filter(|result| result.revision.is_some()).count();

    let mut out = vec![
        "report_schema=fsre-decision-time-replay-report.v0.1".to_owned(),
        format!("scenario_id={}", input.scenario_id),
        format!("decision_id={}", input.decision_id),
        format!("decision_time={}", input.decision_time),
        format!("time_basis_id={}", input.time_basis_id),
        format!("evidence_snapshot_id={}", input.snapshot_id),
        format!("submitted_evidence_ids={}", results.len()),
        format!("selected_as_of_versions={selected_count}"),
        format!("future_revisions_excluded={future_revisions_total}"),
        format!("decision_evidence_disposition={disposition}"),
    ];
    for result in &results {
        let revision = result
            .revision
            .map(|value| value.to_string())
            .unwrap_or_else(|| "NONE".to_owned());
        out.push(format!(
            "evidence={}|{}|{}|{}|{}|{}|{}|{}|future_revisions_excluded={}",
            result.evidence_id,
            revision,
            result.claim_id,
            result.subject_id,
            result.predicate,
            result.disposition,
            result.owner,
            result.reference,
            result.future_revisions_excluded
        ));
    }
    out.push("information_frontier=OBSERVED_AT_AND_RECORDED_AT_LE_DECISION_TIME".to_owned());
    out.push("claim_completeness=NOT_ESTABLISHED_BY_THIS_TOOL".to_owned());
    out.push("source_authenticity=NOT_AUTHENTICATED_BY_THIS_TOOL".to_owned());
    out.push("execution_authority=NONE".to_owned());
    out.push("operational_actuation=NOT_AUTHORISED".to_owned());
    Ok(out.join("\n") + "\n")
}

pub fn validate_and_report(source: &str) -> Result<String, InputError> {
    evaluate(&parse_input(source)?)
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../fixtures/synthetic-decision.psv");

    #[test]
    fn replay_selects_only_evidence_known_and_valid_at_decision_time() {
        let report = validate_and_report(FIXTURE).expect("fixture");
        assert!(report.contains("decision_evidence_disposition=ALL_SUBMITTED_EVIDENCE_VALID_AT_DECISION"));
        assert!(report.contains("selected_as_of_versions=3"));
        assert!(report.contains("future_revisions_excluded=1"));
        assert!(report.contains("evidence=stock_receipt|1|claim-diesel-stock|diesel-tank-01|available_stock|KNOWN_VALID|operator|synthetic:stock:r1|future_revisions_excluded=1"));
        assert!(report.contains("claim_completeness=NOT_ESTABLISHED_BY_THIS_TOOL"));
    }

    #[test]
    fn later_revision_does_not_rewrite_the_as_known_at_decision_state() {
        let report = validate_and_report(FIXTURE).expect("fixture");
        assert!(report.contains("evidence=stock_receipt|1|claim-diesel-stock"));
        assert!(!report.contains("evidence=stock_receipt|2|claim-diesel-stock"));
    }

    #[test]
    fn evidence_recorded_after_decision_is_not_admissible_at_that_time() {
        let changed = FIXTURE.replace(
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|1050|960|970||ASSERTED",
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|1050|960|1010||ASSERTED",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("late ingestion is replayable");
        assert!(report.contains("decision_evidence_disposition=NOT_ESTIMABLE"));
        assert!(report.contains("evidence=route_access|NONE|claim-road-access|road-east|passable|NO_AS_OF_VERSION"));
    }

    #[test]
    fn evidence_observed_after_decision_is_not_admissible_even_if_recorded_early() {
        let changed = FIXTURE.replace(
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|1050|960|970||ASSERTED",
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|1050|1010|1020||ASSERTED",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("future observation is replayable");
        assert!(report.contains("NO_AS_OF_VERSION"));
    }

    #[test]
    fn expired_evidence_is_not_current_at_decision_time() {
        let changed = FIXTURE.replace(
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|1050|960|970||ASSERTED",
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|990|960|970||ASSERTED",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("expired evidence");
        assert!(report.contains("decision_evidence_disposition=EVIDENCE_NOT_VALID_AT_DECISION"));
        assert!(report.contains("route_access|1|claim-road-access|road-east|passable|EXPIRED"));
    }

    #[test]
    fn revocation_known_before_decision_invalidates_evidence() {
        let changed = FIXTURE.replace(
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|1050|960|970||ASSERTED",
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|1050|960|970|990|ASSERTED",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("revoked evidence");
        assert!(report.contains("EVIDENCE_NOT_VALID_AT_DECISION"));
        assert!(report.contains("route_access|1|claim-road-access|road-east|passable|REVOKED"));
    }

    #[test]
    fn evidence_not_yet_valid_is_not_accepted() {
        let changed = FIXTURE.replace(
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|1050|960|970||ASSERTED",
            "route-r1|route_access|1|claim-road-access|road-east|passable|1010|1050|960|970||ASSERTED",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("not-yet-valid evidence");
        assert!(report.contains("NOT_YET_VALID"));
    }

    #[test]
    fn transaction_time_cannot_precede_source_observation() {
        let changed = FIXTURE.replace(
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|1050|960|970||ASSERTED",
            "route-r1|route_access|1|claim-road-access|road-east|passable|950|1050|960|950||ASSERTED",
        );
        assert_ne!(changed, FIXTURE);
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("recorded before it is observed"));
    }

    #[test]
    fn duplicate_revision_is_rejected() {
        let duplicate = "stock-r3|stock_receipt|2|claim-diesel-stock|diesel-tank-01|available_stock|800|1200|900|910||ASSERTED|operator|synthetic:stock:r3";
        let changed = format!("{FIXTURE}{duplicate}\n");
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("duplicate revision 2"));
    }

    #[test]
    fn revocation_recorded_after_decision_does_not_rewrite_prior_knowledge() {
        let future_revocation = "route-r2|route_access|2|claim-road-access|road-east|passable|950|1050|980|1210|990|ASSERTED|transport-owner|synthetic:route:r2";
        let changed = format!("{FIXTURE}{future_revocation}\n");
        let report = validate_and_report(&changed).expect("late revocation revision");
        assert!(report.contains("decision_evidence_disposition=ALL_SUBMITTED_EVIDENCE_VALID_AT_DECISION"));
        assert!(report.contains("evidence=route_access|1|claim-road-access|road-east|passable|KNOWN_VALID|transport-owner|synthetic:route:r1|future_revisions_excluded=1"));
    }

    #[test]
    fn evidence_revision_cannot_silently_change_claim_identity() {
        let changed = FIXTURE.replace(
            "stock-r2|stock_receipt|2|claim-diesel-stock|diesel-tank-01",
            "stock-r2|stock_receipt|2|claim-other-stock|diesel-tank-01",
        );
        assert_ne!(changed, FIXTURE);
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("changes claim identity"));
    }

    #[test]
    fn all_future_revisions_have_row_order_invariant_no_as_of_output() {
        let changed = FIXTURE.replace(
            "stock-r1|stock_receipt|1|claim-diesel-stock|diesel-tank-01|available_stock|800|1100|820|830||ASSERTED",
            "stock-r1|stock_receipt|1|claim-diesel-stock|diesel-tank-01|available_stock|800|1100|1200|1205||ASSERTED",
        );
        assert_ne!(changed, FIXTURE);
        let mut metadata = Vec::new();
        let mut rows = Vec::new();
        for line in changed.lines() {
            if line.starts_with('#') {
                metadata.push(line);
            } else if !line.is_empty() {
                rows.push(line);
            }
        }
        let header = rows.remove(0);
        rows.reverse();
        let reordered = format!(
            "{}\n{}\n{}\n",
            metadata.join("\n"),
            header,
            rows.join("\n")
        );
        let original_report = validate_and_report(&changed).expect("all-future revisions");
        let reordered_report = validate_and_report(&reordered).expect("permuted all-future revisions");
        assert_eq!(original_report, reordered_report);
        assert!(original_report.contains(
            "evidence=stock_receipt|NONE|claim-diesel-stock|diesel-tank-01|available_stock|NO_AS_OF_VERSION|operator|synthetic:stock:r1|future_revisions_excluded=2"
        ));
    }

    #[test]
    fn row_order_does_not_change_as_of_replay() {
        let mut metadata = Vec::new();
        let mut rows = Vec::new();
        for line in FIXTURE.lines() {
            if line.starts_with('#') {
                metadata.push(line);
            } else if !line.is_empty() {
                rows.push(line);
            }
        }
        let header = rows.remove(0);
        rows.reverse();
        let reordered = format!(
            "{}\n{}\n{}\n",
            metadata.join("\n"),
            header,
            rows.join("\n")
        );
        assert_eq!(
            validate_and_report(FIXTURE).expect("original"),
            validate_and_report(&reordered).expect("reordered")
        );
    }
}
