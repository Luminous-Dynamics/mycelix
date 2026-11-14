#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

const HEADER: &str = "record_type|record_id|claim_id|scenario_id|observed_at|available_at|valid_from|valid_until|revoked_effective_at|revocation_known_at|status|clock_basis_id|source_owner|source_ref|required_evidence_count|max_observation_age_seconds";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InputError(pub String);

impl std::fmt::Display for InputError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for InputError {}

#[derive(Clone, Debug)]
struct Claim {
    id: String,
    owner: String,
    source_ref: String,
    required_count: u64,
    max_age_override: Option<u64>,
}

#[derive(Clone, Debug)]
struct Evidence {
    id: String,
    claim_id: String,
    observed_at: Option<u64>,
    available_at: Option<u64>,
    valid_from: Option<u64>,
    valid_until: Option<u64>,
    revoked_effective_at: Option<u64>,
    revocation_known_at: Option<u64>,
    status: String,
    owner: String,
    source_ref: String,
}

#[derive(Clone, Debug)]
struct Input {
    scenario_id: String,
    evidence_snapshot_id: String,
    decision_at: u64,
    max_age: u64,
    clock_basis_id: String,
    claims: Vec<Claim>,
    evidence: Vec<Evidence>,
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

fn require_id(value: &str, label: &str, line: usize) -> Result<(), InputError> {
    if !valid_id(value) {
        return Err(InputError(format!("line {line}: invalid {label}")));
    }
    Ok(())
}

fn parse_input(source: &str) -> Result<Input, InputError> {
    let mut metadata = BTreeMap::<String, String>::new();
    let mut claims = Vec::new();
    let mut evidence = Vec::new();
    let mut record_ids = BTreeSet::new();
    let mut evidence_refs = BTreeSet::new();
    let mut row_scenarios = Vec::new();
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
                    "line {line_number}: unexpected evidence header"
                )));
            }
            header_seen = true;
            continue;
        }

        let fields: Vec<&str> = line.split('|').map(str::trim).collect();
        if fields.len() != 16 {
            return Err(InputError(format!(
                "line {line_number}: expected 16 columns, received {}",
                fields.len()
            )));
        }
        let record_type = fields[0];
        let record_id = fields[1];
        let claim_id = fields[2];
        let row_scenario = fields[3];
        let clock_basis = fields[11];
        let owner = fields[12];
        row_scenarios.push(row_scenario.to_owned());
        let source_ref = fields[13];

        require_id(record_id, "record_id", line_number)?;
        require_id(claim_id, "claim_id", line_number)?;
        require_id(row_scenario, "scenario_id", line_number)?;
        require_id(clock_basis, "clock_basis_id", line_number)?;
        require_id(owner, "source_owner", line_number)?;
        if source_ref.is_empty() || source_ref.len() > 512 {
            return Err(InputError(format!(
                "line {line_number}: invalid source_ref"
            )));
        }
        if !record_ids.insert(record_id.to_owned()) {
            return Err(InputError(format!(
                "line {line_number}: duplicate record_id"
            )));
        }

        let observed = optional_u64(fields[4], "observed_at", line_number)?;
        let available = optional_u64(fields[5], "available_at", line_number)?;
        let valid_from = optional_u64(fields[6], "valid_from", line_number)?;
        let valid_until = optional_u64(fields[7], "valid_until", line_number)?;
        let revoked_effective = optional_u64(fields[8], "revoked_effective_at", line_number)?;
        let revocation_known = optional_u64(fields[9], "revocation_known_at", line_number)?;

        match record_type {
            "CLAIM" => {
                if row_scenario.is_empty()
                    || observed.is_some()
                    || available.is_some()
                    || valid_from.is_some()
                    || valid_until.is_some()
                    || revoked_effective.is_some()
                    || revocation_known.is_some()
                    || fields[10] != "REQUIRED"
                {
                    return Err(InputError(format!(
                        "line {line_number}: malformed CLAIM row"
                    )));
                }
                if clock_basis != "UTC_UNIX_SECONDS_V1" {
                    return Err(InputError(format!(
                        "line {line_number}: unsupported clock basis"
                    )));
                }
                let required_count = parse_u64(fields[14], "required_evidence_count", line_number)?;
                if required_count == 0 {
                    return Err(InputError(format!(
                        "line {line_number}: required evidence count must be positive"
                    )));
                }
                let max_age_override =
                    optional_u64(fields[15], "max_observation_age_seconds", line_number)?;
                claims.push(Claim {
                    id: claim_id.to_owned(),
                    owner: owner.to_owned(),
                    source_ref: source_ref.to_owned(),
                    required_count,
                    max_age_override,
                });
            }
            "EVIDENCE" => {
                if !fields[14].is_empty() || !fields[15].is_empty() {
                    return Err(InputError(format!(
                        "line {line_number}: EVIDENCE row must not set claim policy fields"
                    )));
                }
                if clock_basis != "UTC_UNIX_SECONDS_V1" {
                    return Err(InputError(format!(
                        "line {line_number}: incompatible evidence clock basis"
                    )));
                }
                if !evidence_refs.insert((claim_id.to_owned(), source_ref.to_owned())) {
                    return Err(InputError(format!(
                        "line {line_number}: duplicate source_ref for claim {claim_id}"
                    )));
                }
                let status = fields[10];
                match status {
                    "ASSERTED" => {
                        if observed.is_none() || available.is_none() || valid_from.is_none() {
                            return Err(InputError(format!("line {line_number}: ASSERTED evidence requires observed_at, available_at and valid_from")));
                        }
                        let observed_value = observed.expect("checked above");
                        let available_value = available.expect("checked above");
                        let valid_from_value = valid_from.expect("checked above");
                        if observed_value > available_value {
                            return Err(InputError(format!(
                                "line {line_number}: observed_at must not be after available_at"
                            )));
                        }
                        // A document may be available before its future validity begins,
                        // or may describe a historical validity interval. The as-of
                        // evaluator classifies those cases; the parser must preserve them.
                        if valid_until.is_some_and(|end| end <= valid_from_value) {
                            return Err(InputError(format!(
                                "line {line_number}: validity interval must be non-empty"
                            )));
                        }
                        if revoked_effective.is_some() != revocation_known.is_some() {
                            return Err(InputError(format!(
                                "line {line_number}: revocation timestamps must be paired"
                            )));
                        }
                        if let Some(known_at) = revocation_known {
                            if known_at < available_value {
                                return Err(InputError(format!("line {line_number}: revocation cannot be known before source evidence was available")));
                            }
                        }
                    }
                    "UNKNOWN" => {
                        if observed.is_some()
                            || available.is_some()
                            || valid_from.is_some()
                            || valid_until.is_some()
                            || revoked_effective.is_some()
                            || revocation_known.is_some()
                        {
                            return Err(InputError(format!("line {line_number}: UNKNOWN evidence must not carry temporal assertions")));
                        }
                    }
                    _ => {
                        return Err(InputError(format!(
                            "line {line_number}: unsupported evidence status"
                        )))
                    }
                }
                evidence.push(Evidence {
                    id: record_id.to_owned(),
                    claim_id: claim_id.to_owned(),
                    observed_at: observed,
                    available_at: available,
                    valid_from,
                    valid_until,
                    revoked_effective_at: revoked_effective,
                    revocation_known_at: revocation_known,
                    status: status.to_owned(),
                    owner: owner.to_owned(),
                    source_ref: source_ref.to_owned(),
                });
            }
            _ => {
                return Err(InputError(format!(
                    "line {line_number}: unsupported record_type {record_type}"
                )))
            }
        }
    }

    if !header_seen || claims.is_empty() {
        return Err(InputError(
            "missing header or claim requirements".to_owned(),
        ));
    }
    let mut take = |key: &str| {
        metadata
            .remove(key)
            .ok_or_else(|| InputError(format!("missing metadata key {key}")))
    };
    let scenario_id = take("scenario_id")?;
    let evidence_snapshot_id = take("evidence_snapshot_id")?;
    if row_scenarios.iter().any(|row_scenario| row_scenario != &scenario_id) {
        return Err(InputError("record scenario_id does not match the frozen scenario".to_owned()));
    }
    let decision_at = parse_u64(&take("decision_at")?, "decision_at", 0)?;
    let max_age = parse_u64(&take("max_observation_age_seconds")?, "max_observation_age_seconds", 0)?;
    let clock_basis_id = take("clock_basis_id")?;
    require_id(&scenario_id, "scenario_id", 0)?;
    require_id(&evidence_snapshot_id, "evidence_snapshot_id", 0)?;
    require_id(&clock_basis_id, "clock_basis_id", 0)?;
    if clock_basis_id != "UTC_UNIX_SECONDS_V1" {
        return Err(InputError("unsupported evaluation clock basis".to_owned()));
    }
    if let Some(extra) = metadata.keys().next() {
        return Err(InputError(format!("unknown metadata key {extra}")));
    }

    let mut claim_ids = BTreeSet::new();
    for claim in &claims {
        if !claim_ids.insert(claim.id.clone()) {
            return Err(InputError(format!(
                "duplicate claim requirement {}",
                claim.id
            )));
        }
    }
    for row in &evidence {
        if row.claim_id.is_empty() || !claim_ids.contains(&row.claim_id) {
            return Err(InputError(format!(
                "evidence {} references an undeclared claim",
                row.id
            )));
        }
    }

    Ok(Input {
        scenario_id,
        evidence_snapshot_id,
        decision_at,
        max_age,
        clock_basis_id,
        claims,
        evidence,
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Assessment {
    Unknown,
    FutureObservation,
    NotYetAvailable,
    NotYetValid,
    Expired,
    Stale,
    RevokedAsKnown,
    Admissible,
    RevocationPending,
    HindsightRevocation,
}

impl Assessment {
    fn label(self) -> &'static str {
        match self {
            Self::Unknown => "UNKNOWN",
            Self::FutureObservation => "FUTURE_OBSERVATION",
            Self::NotYetAvailable => "NOT_YET_AVAILABLE",
            Self::NotYetValid => "NOT_YET_VALID",
            Self::Expired => "EXPIRED",
            Self::Stale => "STALE",
            Self::RevokedAsKnown => "REVOKED_AS_KNOWN",
            Self::Admissible => "ADMISSIBLE",
            Self::RevocationPending => "REVOCATION_PENDING",
            Self::HindsightRevocation => "ADMISSIBLE_WITH_HINDSIGHT_REVOCATION",
        }
    }

    fn admissible(self) -> bool {
        matches!(
            self,
            Self::Admissible | Self::RevocationPending | Self::HindsightRevocation
        )
    }
}

fn assess(evidence: &Evidence, input: &Input, max_age: u64) -> Assessment {
    if evidence.status == "UNKNOWN" {
        return Assessment::Unknown;
    }
    let observed = evidence
        .observed_at
        .expect("ASSERTED evidence has observed_at");
    let available = evidence
        .available_at
        .expect("ASSERTED evidence has available_at");
    let valid_from = evidence
        .valid_from
        .expect("ASSERTED evidence has valid_from");

    if observed > input.decision_at {
        return Assessment::FutureObservation;
    }
    if available > input.decision_at {
        return Assessment::NotYetAvailable;
    }
    if valid_from > input.decision_at {
        return Assessment::NotYetValid;
    }
    if evidence
        .valid_until
        .is_some_and(|valid_until| input.decision_at >= valid_until)
    {
        return Assessment::Expired;
    }
    if input.decision_at - observed > max_age {
        return Assessment::Stale;
    }

    match (evidence.revoked_effective_at, evidence.revocation_known_at) {
        (Some(effective_at), Some(known_at))
            if known_at <= input.decision_at && effective_at <= input.decision_at =>
        {
            Assessment::RevokedAsKnown
        }
        (Some(effective_at), Some(known_at))
            if known_at <= input.decision_at && effective_at > input.decision_at =>
        {
            Assessment::RevocationPending
        }
        (Some(effective_at), Some(known_at))
            if effective_at <= input.decision_at && known_at > input.decision_at =>
        {
            Assessment::HindsightRevocation
        }
        _ => Assessment::Admissible,
    }
}

#[derive(Default)]
struct Counts {
    admissible: u64,
    unknown: u64,
    future_observation: u64,
    not_yet_available: u64,
    not_yet_valid: u64,
    expired: u64,
    stale: u64,
    revoked: u64,
    revocation_pending: u64,
    hindsight_revocation: u64,
}

fn add_assessment(counts: &mut Counts, assessment: Assessment) {
    match assessment {
        Assessment::Unknown => counts.unknown += 1,
        Assessment::FutureObservation => counts.future_observation += 1,
        Assessment::NotYetAvailable => counts.not_yet_available += 1,
        Assessment::NotYetValid => counts.not_yet_valid += 1,
        Assessment::Expired => counts.expired += 1,
        Assessment::Stale => counts.stale += 1,
        Assessment::RevokedAsKnown => counts.revoked += 1,
        Assessment::Admissible => counts.admissible += 1,
        Assessment::RevocationPending => {
            counts.admissible += 1;
            counts.revocation_pending += 1;
        }
        Assessment::HindsightRevocation => {
            counts.admissible += 1;
            counts.hindsight_revocation += 1;
        }
    }
}

fn evaluate(input: &Input) -> String {
    let mut claims = input.claims.clone();
    claims.sort_by(|left, right| left.id.cmp(&right.id));
    let mut evidence = input.evidence.clone();
    evidence.sort_by(|left, right| left.id.cmp(&right.id));

    let max_age_by_claim: BTreeMap<String, u64> = claims
        .iter()
        .map(|claim| {
            (
                claim.id.clone(),
                claim.max_age_override.unwrap_or(input.max_age),
            )
        })
        .collect();
    let mut assessments = BTreeMap::<String, Assessment>::new();
    let mut counts_by_claim = BTreeMap::<String, Counts>::new();
    for row in &evidence {
        let max_age = max_age_by_claim[&row.claim_id];
        let assessment = assess(row, input, max_age);
        assessments.insert(row.id.clone(), assessment);
        add_assessment(
            counts_by_claim.entry(row.claim_id.clone()).or_default(),
            assessment,
        );
    }

    let mut out = vec![
        "report_schema=fsre-asof-evidence-report.v0.1".to_owned(),
        format!("scenario_id={}", input.scenario_id),
        format!("evidence_snapshot_id={}", input.evidence_snapshot_id),
        format!("decision_at={}", input.decision_at),
        format!("max_observation_age_seconds={}", input.max_age),
        format!("clock_basis_id={}", input.clock_basis_id),
        format!("claims={}", claims.len()),
        format!("evidence_records={}", evidence.len()),
    ];

    let mut total_hindsight = 0_u64;
    for claim in &claims {
        let counts = counts_by_claim.get(&claim.id);
        let admissible = counts.map(|c| c.admissible).unwrap_or(0);
        let missing = claim.required_count.saturating_sub(admissible);
        let disposition = if missing == 0 {
            "SUPPORTED_AS_KNOWN_AT_CUTOFF"
        } else {
            "NOT_ESTIMABLE"
        };
        let hindsight = counts.map(|c| c.hindsight_revocation).unwrap_or(0);
        let freshness_basis = if claim.max_age_override.is_some() {
            "CLAIM_OWNER_DECLARED"
        } else {
            "STUDY_DEFAULT"
        };
        total_hindsight += hindsight;
        out.push(format!(
            "claim={}|{}|admissible={}|required={}|missing={}|max_age_seconds={}|freshness_policy_basis={}|hindsight_revocations={}|owner={}|ref={}",
            claim.id,
            disposition,
            admissible,
            claim.required_count,
            missing,
            claim.max_age_override.unwrap_or(input.max_age),
            freshness_basis,
            hindsight,
            claim.owner,
            claim.source_ref
        ));
    }

    for row in &evidence {
        let assessment = assessments[&row.id];
        out.push(format!(
            "evidence={}|{}|{}|admissible={}|owner={}|ref={}",
            row.id,
            row.claim_id,
            assessment.label(),
            if assessment.admissible() { "YES" } else { "NO" },
            row.owner,
            row.source_ref
        ));
    }
    out.push(format!("hindsight_revocation_records={total_hindsight}"));
    out.push("replay_semantics=AS_KNOWN_AT_DECISION_CUTOFF".to_owned());
    out.push("future_information_used=NO".to_owned());
    out.push("source_authenticity=NOT_AUTHENTICATED_BY_THIS_TOOL".to_owned());
    out.push("operational_authority=NONE".to_owned());
    Ok(out.join("\n") + "\n")
}

pub fn validate_and_report(source: &str) -> Result<String, InputError> {
    let input = parse_input(source)?;
    Ok(evaluate(&input))
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../fixtures/synthetic-asof.psv");

    #[test]
    fn evidence_snapshot_id_is_required_and_echoed_in_report() {
        let report = validate_and_report(FIXTURE).expect("snapshot-bound fixture");
        assert!(report.contains("evidence_snapshot_id=synthetic-decision-snapshot-01"));
        let changed = FIXTURE
            .lines()
            .filter(|line| !line.starts_with("# evidence_snapshot_id="))
            .collect::<Vec<_>>()
            .join("\n");
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("missing metadata key evidence_snapshot_id"));
    }

    #[test]
    fn frozen_fixture_separates_world_validity_from_knowledge_time() {
        let report = validate_and_report(FIXTURE).expect("fixture is valid");
        assert!(report.contains(
            "claim=pump_capacity|SUPPORTED_AS_KNOWN_AT_CUTOFF|admissible=2|required=2|missing=0"
        ));
        assert!(
            report.contains("claim=crew_authority|NOT_ESTIMABLE|admissible=0|required=1|missing=1")
        );
        assert!(report.contains("e-future|future_sensor|FUTURE_OBSERVATION|admissible=NO"));
        assert!(report.contains("e-road|road_access|EXPIRED|admissible=NO"));
        assert!(report
            .contains("e-telemetry|telemetry|ADMISSIBLE_WITH_HINDSIGHT_REVOCATION|admissible=YES"));
        assert!(report.contains("future_information_used=NO"));
    }

    #[test]
    fn availability_at_cutoff_is_inclusive_but_after_cutoff_is_not() {
        let source = FIXTURE
            .replace(
                "e-pump-1|pump_capacity|synthetic-water-shock|6000|6100",
                "e-pump-1|pump_capacity|synthetic-water-shock|6000|7200",
            )
            .replace(
                "e-pump-2|pump_capacity|synthetic-water-shock|3600|3700",
                "e-pump-2|pump_capacity|synthetic-water-shock|3600|7201",
            );
        let report = validate_and_report(&source).expect("temporal boundary fixture");
        assert!(report.contains("e-pump-1|pump_capacity|ADMISSIBLE|admissible=YES"));
        assert!(report.contains("e-pump-2|pump_capacity|NOT_YET_AVAILABLE|admissible=NO"));
    }

    #[test]
    fn validity_interval_end_is_exclusive() {
        let source = FIXTURE.replace(
            "e-pump-1|pump_capacity|synthetic-water-shock|6000|6100|5900|9000",
            "e-pump-1|pump_capacity|synthetic-water-shock|6000|6100|5900|7200",
        );
        let report = validate_and_report(&source).expect("validity boundary");
        assert!(report.contains("e-pump-1|pump_capacity|EXPIRED|admissible=NO"));
    }

    #[test]
    fn observation_age_threshold_is_inclusive() {
        let source = FIXTURE
            .replace(
                "e-pump-1|pump_capacity|synthetic-water-shock|6000|6100",
                "e-pump-1|pump_capacity|synthetic-water-shock|3600|6100",
            )
            .replace(
                "e-pump-1|pump_capacity|synthetic-water-shock|3600|6100|5900",
                "e-pump-1|pump_capacity|synthetic-water-shock|3600|6100|3500",
            );
        let report = validate_and_report(&source).expect("freshness boundary");
        assert!(report.contains("e-pump-1|pump_capacity|ADMISSIBLE|admissible=YES"));
    }

    #[test]
    fn revocation_known_before_cutoff_blocks_evidence() {
        let report = validate_and_report(FIXTURE).expect("fixture");
        assert!(report.contains("e-crew|crew_authority|REVOKED_AS_KNOWN|admissible=NO"));
    }

    #[test]
    fn later_discovered_revocation_does_not_rewrite_as_known_decision() {
        let report = validate_and_report(FIXTURE).expect("fixture");
        assert!(report.contains("claim=telemetry|SUPPORTED_AS_KNOWN_AT_CUTOFF|admissible=1|required=1|missing=0|hindsight_revocations=1"));
        assert!(report.contains("hindsight_revocation_records=1"));
    }

    #[test]
    fn announced_future_revocation_remains_valid_until_effective_time() {
        let report = validate_and_report(FIXTURE).expect("fixture");
        assert!(report.contains("e-pending|pending_permit|REVOCATION_PENDING|admissible=YES"));
    }

    #[test]
    fn unknown_evidence_cannot_carry_temporal_assertions() {
        let source = FIXTURE.replace(
            "EVIDENCE|e-water|water_quality|synthetic-water-shock|||||||UNKNOWN",
            "EVIDENCE|e-water|water_quality|synthetic-water-shock|7100||||||UNKNOWN",
        );
        assert_ne!(source, FIXTURE);
        assert!(validate_and_report(&source)
            .unwrap_err()
            .0
            .contains("UNKNOWN evidence must not carry temporal assertions"));
    }

    #[test]
    fn mismatched_clock_basis_is_rejected() {
        let source = FIXTURE.replace(
            "EVIDENCE|e-pump-1|pump_capacity|synthetic-water-shock|6000|6100|5900|9000|||ASSERTED|UTC_UNIX_SECONDS_V1",
            "EVIDENCE|e-pump-1|pump_capacity|synthetic-water-shock|6000|6100|5900|9000|||ASSERTED|LOCAL_WALL_CLOCK",
        );
        assert_ne!(source, FIXTURE);
        assert!(validate_and_report(&source)
            .unwrap_err()
            .0
            .contains("incompatible evidence clock basis"));
    }

    #[test]
    fn future_observation_is_not_used_at_the_cutoff() {
        let source = FIXTURE.replace(
            "EVIDENCE|e-pump-1|pump_capacity|synthetic-water-shock|6000|6100|5900|9000|||ASSERTED",
            "EVIDENCE|e-pump-1|pump_capacity|synthetic-water-shock|7300|7400|7000|9000|||ASSERTED",
        );
        assert_ne!(source, FIXTURE);
        let report = validate_and_report(&source).expect("future evidence is classifiable");
        assert!(report.contains("e-pump-1|pump_capacity|FUTURE_OBSERVATION|admissible=NO"));
    }

    #[test]
    fn evidence_reference_cannot_be_counted_twice_for_one_claim() {
        let line = "EVIDENCE|e-pump-2|pump_capacity|synthetic-water-shock|3600|3700|3500|8500|||ASSERTED|UTC_UNIX_SECONDS_V1|symtropy|synthetic:evidence:pump:2||";
        let duplicate = line.replace("e-pump-2", "e-pump-duplicate");
        let source = format!("{FIXTURE}{duplicate}\n");
        assert!(validate_and_report(&source)
            .unwrap_err()
            .0
            .contains("duplicate source_ref for claim pump_capacity"));
    }

    #[test]
    fn input_row_order_does_not_change_report() {
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
        let reordered = format!("{}\n{}\n{}\n", metadata.join("\n"), header, rows.join("\n"));
        assert_eq!(
            validate_and_report(FIXTURE).expect("original"),
            validate_and_report(&reordered).expect("reordered")
        );
    }

    #[test]
    fn scenario_mismatch_is_rejected_instead_of_crossing_evidence_boundaries() {
        let source = FIXTURE.replace(
            "EVIDENCE|e-pump-1|pump_capacity|synthetic-water-shock",
            "EVIDENCE|e-pump-1|pump_capacity|different-scenario",
        );
        assert_ne!(source, FIXTURE);
        assert!(validate_and_report(&source)
            .unwrap_err()
            .0
            .contains("does not match the frozen scenario"));
    }

    #[test]
    fn validity_start_is_inclusive_and_future_start_is_not_yet_valid() {
        let source = FIXTURE.replace(
            "EVIDENCE|e-pump-1|pump_capacity|synthetic-water-shock|6000|6100|5900|9000",
            "EVIDENCE|e-pump-1|pump_capacity|synthetic-water-shock|7200|7200|7200|9000",
        );
        let report = validate_and_report(&source).expect("valid_from boundary");
        assert!(report.contains("e-pump-1|pump_capacity|ADMISSIBLE|admissible=YES"));

        let future = FIXTURE.replace(
            "EVIDENCE|e-pump-1|pump_capacity|synthetic-water-shock|6000|6100|5900|9000",
            "EVIDENCE|e-pump-1|pump_capacity|synthetic-water-shock|7000|7100|7300|9000",
        );
        let report = validate_and_report(&future).expect("future validity start");
        assert!(report.contains("e-pump-1|pump_capacity|NOT_YET_VALID|admissible=NO"));
    }

    #[test]
    fn evidence_for_a_future_validity_window_is_not_yet_valid() {
        let source = FIXTURE.replace(
            "EVIDENCE|e-pump-1|pump_capacity|synthetic-water-shock|6000|6100|5900|9000",
            "EVIDENCE|e-pump-1|pump_capacity|synthetic-water-shock|6000|6100|7300|9000",
        );
        let report =
            validate_and_report(&source).expect("future-effective evidence is valid input");
        assert!(report.contains("e-pump-1|pump_capacity|NOT_YET_VALID|admissible=NO"));
    }

    #[test]
    fn claim_specific_freshness_override_can_be_stricter_than_run_default() {
        let source = FIXTURE.replace(
            "CLAIM|claim-fuel|fuel_stock|synthetic-water-shock|||||||REQUIRED|UTC_UNIX_SECONDS_V1|operator|synthetic:claim:fuel|1|",
            "CLAIM|claim-fuel|fuel_stock|synthetic-water-shock|||||||REQUIRED|UTC_UNIX_SECONDS_V1|operator|synthetic:claim:fuel|1|3000",
        );
        let report = validate_and_report(&source).expect("claim-specific freshness policy");
        assert!(report.contains("claim=fuel_stock|NOT_ESTIMABLE|admissible=0|required=1|missing=1|max_age_seconds=3000|freshness_policy_basis=CLAIM_OWNER_DECLARED"));
        assert!(report.contains("e-fuel|fuel_stock|STALE|admissible=NO"));
    }

    #[test]
    fn claim_specific_freshness_override_can_be_more_permissive_than_run_default() {
        let source = FIXTURE.replace(
            "CLAIM|claim-fuel|fuel_stock|synthetic-water-shock|||||||REQUIRED|UTC_UNIX_SECONDS_V1|operator|synthetic:claim:fuel|1|",
            "CLAIM|claim-fuel|fuel_stock|synthetic-water-shock|||||||REQUIRED|UTC_UNIX_SECONDS_V1|operator|synthetic:claim:fuel|1|4000",
        );
        let report = validate_and_report(&source).expect("claim-specific freshness policy");
        assert!(report.contains("claim=fuel_stock|SUPPORTED_AS_KNOWN_AT_CUTOFF|admissible=1|required=1|missing=0|max_age_seconds=4000|freshness_policy_basis=CLAIM_OWNER_DECLARED"));
        assert!(report.contains("e-fuel|fuel_stock|ADMISSIBLE|admissible=YES"));
    }

    #[test]
    fn evidence_rows_cannot_override_claim_freshness_policy() {
        let source = FIXTURE.replace(
            "EVIDENCE|e-fuel|fuel_stock|synthetic-water-shock|3500|3600|3400|9000|||ASSERTED|UTC_UNIX_SECONDS_V1|operator|synthetic:evidence:fuel||",
            "EVIDENCE|e-fuel|fuel_stock|synthetic-water-shock|3500|3600|3400|9000|||ASSERTED|UTC_UNIX_SECONDS_V1|operator|synthetic:evidence:fuel||4000",
        );
        assert_ne!(source, FIXTURE);
        assert!(validate_and_report(&source)
            .unwrap_err()
            .0
            .contains("EVIDENCE row must not set claim policy fields"));
    }

    #[test]
    fn declared_required_evidence_without_rows_is_not_estimable() {
        let source = FIXTURE
            .lines()
            .filter(|line| !line.starts_with("EVIDENCE|e-water|"))
            .collect::<Vec<_>>()
            .join("\n");
        let report = validate_and_report(&source).expect("missing evidence is an explicit state");
        assert!(
            report.contains("claim=water_quality|NOT_ESTIMABLE|admissible=0|required=1|missing=1")
        );
    }

    #[test]
    fn generated_cutoff_sweep_checks_availability_validity_and_freshness_boundaries() {
        for available_at in [7199_u64, 7200, 7201] {
            let source = FIXTURE.replace(
                "EVIDENCE|e-pump-1|pump_capacity|synthetic-water-shock|6000|6100|5900|9000|||ASSERTED",
                &format!("EVIDENCE|e-pump-1|pump_capacity|synthetic-water-shock|6000|{available_at}|5900|9000|||ASSERTED"),
            );
            let report = validate_and_report(&source).expect("availability sweep");
            let expected = if available_at <= 7200 {
                "ADMISSIBLE"
            } else {
                "NOT_YET_AVAILABLE"
            };
            assert!(report.contains(&format!("e-pump-1|pump_capacity|{expected}")));
        }
        for valid_until in [7199_u64, 7200, 7201] {
            let source = FIXTURE.replace(
                "EVIDENCE|e-pump-1|pump_capacity|synthetic-water-shock|6000|6100|5900|9000|||ASSERTED",
                &format!("EVIDENCE|e-pump-1|pump_capacity|synthetic-water-shock|6000|6100|5900|{valid_until}|||ASSERTED"),
            );
            let report = validate_and_report(&source).expect("validity sweep");
            let expected = if valid_until > 7200 {
                "ADMISSIBLE"
            } else {
                "EXPIRED"
            };
            assert!(report.contains(&format!("e-pump-1|pump_capacity|{expected}")));
        }
        for effective_at in [7199_u64, 7200, 7201] {
            let source = FIXTURE.replace(
                "EVIDENCE|e-pending|pending_permit|synthetic-water-shock|6800|6900|6700|10000|7600|7000",
                &format!(
                    "EVIDENCE|e-pending|pending_permit|synthetic-water-shock|6800|6900|6700|10000|{effective_at}|7000"
                ),
            );
            let report = validate_and_report(&source).expect("revocation effective-time sweep");
            let expected = if effective_at <= 7200 {
                "REVOKED_AS_KNOWN"
            } else {
                "REVOCATION_PENDING"
            };
            assert!(
                report.contains(&format!("e-pending|pending_permit|{expected}")),
                "revocation effective-time boundary {effective_at}"
            );
        }
        for known_at in [7199_u64, 7200, 7201] {
            let source = FIXTURE.replace(
                "EVIDENCE|e-telemetry|telemetry|synthetic-water-shock|6800|6900|6000|9500|6500|7600",
                &format!(
                    "EVIDENCE|e-telemetry|telemetry|synthetic-water-shock|6800|6900|6000|9500|6500|{known_at}"
                ),
            );
            let report = validate_and_report(&source).expect("revocation knowledge-time sweep");
            let expected = if known_at <= 7200 {
                "REVOKED_AS_KNOWN"
            } else {
                "ADMISSIBLE_WITH_HINDSIGHT_REVOCATION"
            };
            assert!(
                report.contains(&format!("e-telemetry|telemetry|{expected}")),
                "revocation knowledge-time boundary {known_at}"
            );
        }

        for observed_at in [3599_u64, 3600, 3601] {
            let source = FIXTURE.replace(
                "EVIDENCE|e-pump-1|pump_capacity|synthetic-water-shock|6000|6100|5900|9000|||ASSERTED",
                &format!("EVIDENCE|e-pump-1|pump_capacity|synthetic-water-shock|{observed_at}|6100|3500|9000|||ASSERTED"),
            );
            let report = validate_and_report(&source).expect("freshness sweep");
            let expected = if 7200 - observed_at <= 3600 {
                "ADMISSIBLE"
            } else {
                "STALE"
            };
            assert!(report.contains(&format!("e-pump-1|pump_capacity|{expected}")));
        }
    }
}
