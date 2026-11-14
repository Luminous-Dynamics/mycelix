#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

const COLUMNS: &str = "record_type|record_id|claim_id|scenario_id|observed_at|available_at|valid_from|valid_until|revoked_effective_at|revocation_known_at|status|clock_basis_id|source_owner|source_ref|required_evidence_count|max_observation_age_seconds";

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReferenceError(pub String);

impl std::fmt::Display for ReferenceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}
impl std::error::Error for ReferenceError {}

#[derive(Debug, Clone)]
struct Requirement {
    key: String,
    owner: String,
    reference: String,
    minimum: u64,
    freshness_limit: Option<u64>,
}

#[derive(Debug, Clone)]
struct Observation {
    key: String,
    requirement: String,
    observed: Option<u64>,
    released: Option<u64>,
    begins: Option<u64>,
    ends: Option<u64>,
    revoked_at: Option<u64>,
    revocation_learned_at: Option<u64>,
    state: String,
    owner: String,
    reference: String,
}

#[derive(Debug, Clone)]
struct Replay {
    scenario: String,
    cutoff: u64,
    standard_freshness: u64,
    clock: String,
    requirements: Vec<Requirement>,
    observations: Vec<Observation>,
}

fn identifier(text: &str) -> bool {
    !text.is_empty()
        && text.len() <= 160
        && text.bytes().all(|value| {
            value.is_ascii_alphanumeric()
                || value == b'_'
                || value == b'-'
                || value == b'.'
                || value == b':'
                || value == b'/'
        })
}

fn integer(text: &str, field: &str, row: usize) -> Result<u64, ReferenceError> {
    text.parse::<u64>().map_err(|error| {
        ReferenceError(format!("row {row}: bad {field}: {error}"))
    })
}

fn maybe_integer(text: &str, field: &str, row: usize) -> Result<Option<u64>, ReferenceError> {
    match text.is_empty() {
        true => Ok(None),
        false => integer(text, field, row).map(Some),
    }
}

fn check_identifier(text: &str, field: &str, row: usize) -> Result<(), ReferenceError> {
    if identifier(text) {
        Ok(())
    } else {
        Err(ReferenceError(format!("row {row}: invalid {field}")))
    }
}

fn split_row(line: &str, row: usize) -> Result<Vec<&str>, ReferenceError> {
    let fields: Vec<&str> = line.split('|').map(str::trim).collect();
    if fields.len() != 16 {
        return Err(ReferenceError(format!(
            "row {row}: expected 16 columns, got {}",
            fields.len()
        )));
    }
    Ok(fields)
}

fn parse_replay(text: &str) -> Result<Replay, ReferenceError> {
    let mut header = false;
    let mut properties = BTreeMap::<String, String>::new();
    let mut requirements = Vec::new();
    let mut observations = Vec::new();
    let mut record_keys = BTreeSet::new();
    let mut reference_keys = BTreeSet::new();
    let mut row_scenarios = Vec::new();

    for (index, raw) in text.lines().enumerate() {
        let row_number = index + 1;
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            if let Some((name, value)) = comment.trim().split_once('=') {
                if properties.insert(name.trim().to_owned(), value.trim().to_owned()).is_some() {
                    return Err(ReferenceError(format!("row {row_number}: repeated metadata key")));
                }
            }
            continue;
        }
        if !header {
            if line != COLUMNS {
                return Err(ReferenceError(format!("row {row_number}: unexpected header")));
            }
            header = true;
            continue;
        }

        let fields = split_row(line, row_number)?;
        let kind = fields[0];
        let record_id = fields[1];
        let requirement_id = fields[2];
        let scenario = fields[3];
        let state = fields[10];
        let clock = fields[11];
        let owner = fields[12];
        let reference = fields[13];

        for (field, value) in [
            ("record_id", record_id),
            ("claim_id", requirement_id),
            ("scenario_id", scenario),
            ("clock_basis_id", clock),
            ("source_owner", owner),
        ] {
            check_identifier(value, field, row_number)?;
        }
        if reference.is_empty() || reference.len() > 512 {
            return Err(ReferenceError(format!("row {row_number}: bad source reference")));
        }
        if !record_keys.insert(record_id.to_owned()) {
            return Err(ReferenceError(format!("row {row_number}: reused record ID")));
        }
        row_scenarios.push(scenario.to_owned());

        let observed = maybe_integer(fields[4], "observed_at", row_number)?;
        let released = maybe_integer(fields[5], "available_at", row_number)?;
        let begins = maybe_integer(fields[6], "valid_from", row_number)?;
        let ends = maybe_integer(fields[7], "valid_until", row_number)?;
        let revoked_at = maybe_integer(fields[8], "revoked_effective_at", row_number)?;
        let revocation_learned_at = maybe_integer(fields[9], "revocation_known_at", row_number)?;

        if clock != "UTC_UNIX_SECONDS_V1" {
            return Err(ReferenceError(format!("row {row_number}: clock basis mismatch")));
        }

        match kind {
            "CLAIM" => {
                if observed.is_some()
                    || released.is_some()
                    || begins.is_some()
                    || ends.is_some()
                    || revoked_at.is_some()
                    || revocation_learned_at.is_some()
                    || state != "REQUIRED"
                {
                    return Err(ReferenceError(format!("row {row_number}: malformed claim requirement")));
                }
                let minimum = integer(fields[14], "required evidence count", row_number)?;
                if minimum == 0 {
                    return Err(ReferenceError(format!("row {row_number}: zero required evidence count")));
                }
                let freshness_limit = maybe_integer(fields[15], "claim freshness limit", row_number)?;
                requirements.push(Requirement {
                    key: requirement_id.to_owned(),
                    owner: owner.to_owned(),
                    reference: reference.to_owned(),
                    minimum,
                    freshness_limit,
                });
            }
            "EVIDENCE" => {
                if !fields[14].is_empty() || !fields[15].is_empty() {
                    return Err(ReferenceError(format!("row {row_number}: evidence row contains claim policy")));
                }
                if !reference_keys.insert((requirement_id.to_owned(), reference.to_owned())) {
                    return Err(ReferenceError(format!("row {row_number}: duplicated evidence reference")));
                }
                match state {
                    "UNKNOWN" => {
                        if [
                            observed,
                            released,
                            begins,
                            ends,
                            revoked_at,
                            revocation_learned_at,
                        ]
                        .iter()
                        .any(Option::is_some)
                        {
                            return Err(ReferenceError(format!(
                                "row {row_number}: unknown row contains a time assertion"
                            )));
                        }
                    }
                    "ASSERTED" => {
                        let measured = observed.ok_or_else(|| {
                            ReferenceError(format!("row {row_number}: asserted row has no observation time"))
                        })?;
                        let available = released.ok_or_else(|| {
                            ReferenceError(format!("row {row_number}: asserted row has no availability time"))
                        })?;
                        let start = begins.ok_or_else(|| {
                            ReferenceError(format!("row {row_number}: asserted row has no validity start"))
                        })?;
                        if measured > available {
                            return Err(ReferenceError(format!(
                                "row {row_number}: observation occurs after publication/availability"
                            )));
                        }
                        if let Some(end) = ends {
                            if end <= start {
                                return Err(ReferenceError(format!(
                                    "row {row_number}: empty validity period"
                                )));
                            }
                        }
                        if revoked_at.is_some() != revocation_learned_at.is_some() {
                            return Err(ReferenceError(format!(
                                "row {row_number}: incomplete revocation pair"
                            )));
                        }
                        if revocation_learned_at.is_some_and(|known| known < available) {
                            return Err(ReferenceError(format!(
                                "row {row_number}: revocation known before evidence became available"
                            )));
                        }
                    }
                    _ => return Err(ReferenceError(format!("row {row_number}: unknown evidence state"))),
                }
                observations.push(Observation {
                    key: record_id.to_owned(),
                    requirement: requirement_id.to_owned(),
                    observed,
                    released,
                    begins,
                    ends,
                    revoked_at,
                    revocation_learned_at,
                    state: state.to_owned(),
                    owner: owner.to_owned(),
                    reference: reference.to_owned(),
                });
            }
            _ => return Err(ReferenceError(format!("row {row_number}: unrecognised row kind"))),
        }
    }

    if !header || requirements.is_empty() {
        return Err(ReferenceError("no claim requirements supplied".to_owned()));
    }
    let mut take = |key: &str| {
        properties
            .remove(key)
            .ok_or_else(|| ReferenceError(format!("missing metadata {key}")))
    };
    let scenario = take("scenario_id")?;
    let cutoff = integer(&take("decision_at")?, "decision_at", 0)?;
    let standard_freshness = integer(&take("max_observation_age_seconds")?, "default freshness", 0)?;
    let clock = take("clock_basis_id")?;
    if !identifier(&scenario) || clock != "UTC_UNIX_SECONDS_V1" {
        return Err(ReferenceError("unsupported scenario or replay clock".to_owned()));
    }
    if properties.keys().next().is_some() {
        return Err(ReferenceError("unexpected metadata field".to_owned()));
    }
    if row_scenarios.iter().any(|value| value != &scenario) {
        return Err(ReferenceError("row crosses frozen scenario boundary".to_owned()));
    }

    let mut required = BTreeSet::new();
    for claim in &requirements {
        if !required.insert(claim.key.clone()) {
            return Err(ReferenceError(format!("duplicate requirement {}", claim.key)));
        }
    }
    for item in &observations {
        if !required.contains(&item.requirement) {
            return Err(ReferenceError(format!(
                "evidence {} has no declared requirement",
                item.key
            )));
        }
    }

    Ok(Replay {
        scenario,
        cutoff,
        standard_freshness,
        clock,
        requirements,
        observations,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ResultClass {
    Unknown,
    FutureObservation,
    DelayedPublication,
    FutureValidity,
    EndedValidity,
    TooOld,
    Revoked,
    Usable,
    RevocationNotice,
    RetrospectiveRevocation,
}

impl ResultClass {
    fn text(self) -> &'static str {
        match self {
            Self::Unknown => "UNKNOWN",
            Self::FutureObservation => "FUTURE_OBSERVATION",
            Self::DelayedPublication => "NOT_YET_AVAILABLE",
            Self::FutureValidity => "NOT_YET_VALID",
            Self::EndedValidity => "EXPIRED",
            Self::TooOld => "STALE",
            Self::Revoked => "REVOKED_AS_KNOWN",
            Self::Usable => "ADMISSIBLE",
            Self::RevocationNotice => "REVOCATION_PENDING",
            Self::RetrospectiveRevocation => "ADMISSIBLE_WITH_HINDSIGHT_REVOCATION",
        }
    }

    fn counts(self) -> bool {
        matches!(
            self,
            Self::Usable | Self::RevocationNotice | Self::RetrospectiveRevocation
        )
    }
}

fn classify(row: &Observation, replay: &Replay, allowed_age: u64) -> ResultClass {
    if row.state == "UNKNOWN" {
        return ResultClass::Unknown;
    }
    let observed = row.observed.expect("validated asserted observation");
    let available = row.released.expect("validated asserted availability");
    let valid_from = row.begins.expect("validated asserted validity start");

    if observed > replay.cutoff {
        return ResultClass::FutureObservation;
    }
    if available > replay.cutoff {
        return ResultClass::DelayedPublication;
    }
    if valid_from > replay.cutoff {
        return ResultClass::FutureValidity;
    }
    if row.ends.is_some_and(|end| replay.cutoff >= end) {
        return ResultClass::EndedValidity;
    }
    if replay.cutoff - observed > allowed_age {
        return ResultClass::TooOld;
    }

    match (row.revoked_at, row.revocation_learned_at) {
        (Some(effective), Some(known))
            if effective <= replay.cutoff && known <= replay.cutoff =>
        {
            ResultClass::Revoked
        }
        (Some(effective), Some(known))
            if effective > replay.cutoff && known <= replay.cutoff =>
        {
            ResultClass::RevocationNotice
        }
        (Some(effective), Some(known))
            if effective <= replay.cutoff && known > replay.cutoff =>
        {
            ResultClass::RetrospectiveRevocation
        }
        _ => ResultClass::Usable,
    }
}

#[derive(Default, Debug)]
struct Tally {
    accepted: u64,
    retrospective: u64,
}

fn reproduce(replay: &Replay) -> String {
    let mut requirements = replay.requirements.clone();
    requirements.sort_by(|a, b| a.key.cmp(&b.key));
    let mut observations = replay.observations.clone();
    observations.sort_by(|a, b| a.key.cmp(&b.key));

    let ages: BTreeMap<String, u64> = requirements
        .iter()
        .map(|claim| {
            (
                claim.key.clone(),
                claim.freshness_limit.unwrap_or(replay.standard_freshness),
            )
        })
        .collect();
    let classes: BTreeMap<String, ResultClass> = observations
        .iter()
        .map(|row| {
            (
                row.key.clone(),
                classify(row, replay, ages[&row.requirement]),
            )
        })
        .collect();

    let mut tallies = BTreeMap::<String, Tally>::new();
    for row in &observations {
        let state = classes[&row.key];
        let tally = tallies.entry(row.requirement.clone()).or_default();
        if state.counts() {
            tally.accepted += 1;
        }
        if state == ResultClass::RetrospectiveRevocation {
            tally.retrospective += 1;
        }
    }

    let mut report = vec![
        "report_schema=fsre-asof-evidence-report.v0.1".to_owned(),
        format!("scenario_id={}", replay.scenario),
        format!("decision_at={}", replay.cutoff),
        format!("max_observation_age_seconds={}", replay.standard_freshness),
        format!("clock_basis_id={}", replay.clock),
        format!("claims={}", requirements.len()),
        format!("evidence_records={}", observations.len()),
    ];

    for requirement in &requirements {
        let tally = tallies.get(&requirement.key);
        let accepted = tally.map(|value| value.accepted).unwrap_or(0);
        let retrospective = tally.map(|value| value.retrospective).unwrap_or(0);
        let missing = requirement.minimum.saturating_sub(accepted);
        let state = if missing == 0 {
            "SUPPORTED_AS_KNOWN_AT_CUTOFF"
        } else {
            "NOT_ESTIMABLE"
        };
        report.push(format!(
            "claim={}|{}|admissible={}|required={}|missing={}|max_age_seconds={}|hindsight_revocations={}|owner={}|ref={}",
            requirement.key,
            state,
            accepted,
            requirement.minimum,
            missing,
            ages[&requirement.key],
            retrospective,
            requirement.owner,
            requirement.reference
        ));
    }

    for row in &observations {
        let state = classes[&row.key];
        report.push(format!(
            "evidence={}|{}|{}|admissible={}|owner={}|ref={}",
            row.key,
            row.requirement,
            state.text(),
            if state.counts() { "YES" } else { "NO" },
            row.owner,
            row.reference
        ));
    }

    let retrospective_total: u64 = tallies.values().map(|value| value.retrospective).sum();
    report.push(format!("hindsight_revocation_records={retrospective_total}"));
    report.push("replay_semantics=AS_KNOWN_AT_DECISION_CUTOFF".to_owned());
    report.push("future_information_used=NO".to_owned());
    report.push("source_authenticity=NOT_AUTHENTICATED_BY_THIS_TOOL".to_owned());
    report.push("operational_authority=NONE".to_owned());
    report.join("\n") + "\n"
}

pub fn validate_and_report(source: &str) -> Result<String, ReferenceError> {
    Ok(reproduce(&parse_replay(source)?))
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str =
        include_str!("../../fsre-asof-evidence/fixtures/synthetic-asof.psv");

    #[test]
    fn independent_oracle_matches_point_in_time_fixture() {
        let report = validate_and_report(FIXTURE).expect("fixture");
        assert!(report.contains("claim=pump_capacity|SUPPORTED_AS_KNOWN_AT_CUTOFF|admissible=2"));
        assert!(report.contains("e-telemetry|telemetry|ADMISSIBLE_WITH_HINDSIGHT_REVOCATION"));
        assert!(report.contains("e-crew|crew_authority|REVOKED_AS_KNOWN"));
    }

    #[test]
    fn row_permutation_does_not_change_the_oracle_report() {
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
        let reversed = format!(
            "{}\n{}\n{}\n",
            metadata.join("\n"),
            header,
            rows.join("\n")
        );
        assert_eq!(
            validate_and_report(FIXTURE).expect("original"),
            validate_and_report(&reversed).expect("reversed")
        );
    }

    #[test]
    fn a_late_revocation_is_visible_without_rewriting_history() {
        let report = validate_and_report(FIXTURE).expect("fixture");
        assert!(report.contains("claim=telemetry|SUPPORTED_AS_KNOWN_AT_CUTOFF"));
        assert!(report.contains("hindsight_revocation_records=1"));
    }

    #[test]
    fn claim_freshness_override_is_applied_independently_of_run_default() {
        let changed = FIXTURE.replace(
            "CLAIM|claim-fuel|fuel_stock|synthetic-water-shock|||||||REQUIRED|UTC_UNIX_SECONDS_V1|operator|synthetic:claim:fuel|1|",
            "CLAIM|claim-fuel|fuel_stock|synthetic-water-shock|||||||REQUIRED|UTC_UNIX_SECONDS_V1|operator|synthetic:claim:fuel|1|4000",
        );
        let report = validate_and_report(&changed).expect("claim freshness override");
        assert!(report.contains("claim=fuel_stock|SUPPORTED_AS_KNOWN_AT_CUTOFF|admissible=1|required=1"));
        assert!(report.contains("e-fuel|fuel_stock|ADMISSIBLE|admissible=YES"));
    }

    #[test]
    fn cross_scenario_evidence_is_rejected() {
        let changed = FIXTURE.replace(
            "EVIDENCE|e-pump-1|pump_capacity|synthetic-water-shock",
            "EVIDENCE|e-pump-1|pump_capacity|another-scenario",
        );
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("row crosses frozen scenario boundary"));
    }

    #[test]
    fn source_cannot_be_counted_twice_for_a_claim() {
        let original = FIXTURE
            .lines()
            .find(|line| line.starts_with("EVIDENCE|e-pump-2|"))
            .expect("pump evidence");
        let duplicate = original.replace("e-pump-2", "e-pump-duplicate");
        let changed = format!("{FIXTURE}{duplicate}\n");
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("duplicated evidence reference"));
    }
}
