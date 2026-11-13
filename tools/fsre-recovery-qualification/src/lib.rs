#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const REQUIRED_PREDICATES: [PredicateKind; 7] = [
    PredicateKind::PhysicalFeasibility,
    PredicateKind::SpatialAccess,
    PredicateKind::DependencyIndependence,
    PredicateKind::Staffing,
    PredicateKind::Resources,
    PredicateKind::Authority,
    PredicateKind::Safety,
];

const DEBT_STATUSES: [&str; 9] = [
    "QUALIFIED_DEFICIENCY",
    "STALE_EVIDENCE",
    "UNKNOWN",
    "EXPIRED_AUTHORITY",
    "OUT_OF_DOMAIN",
    "INCOMPLETE_INVENTORY",
    "UNVERIFIED_COMPLETION",
    "CONFLICTING_EVIDENCE",
    "RESOLVED_PENDING_VERIFICATION",
];

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InputError(pub String);

impl fmt::Display for InputError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for InputError {}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum PredicateKind {
    PhysicalFeasibility,
    SpatialAccess,
    DependencyIndependence,
    Staffing,
    Resources,
    Authority,
    Safety,
}

impl PredicateKind {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::PhysicalFeasibility => "physical_feasibility",
            Self::SpatialAccess => "spatial_access",
            Self::DependencyIndependence => "dependency_independence",
            Self::Staffing => "staffing",
            Self::Resources => "resources",
            Self::Authority => "authority",
            Self::Safety => "safety",
        }
    }

    pub fn expected_owner(self) -> &'static str {
        match self {
            Self::PhysicalFeasibility => "symtropy",
            Self::SpatialAccess => "sol-atlas",
            Self::DependencyIndependence => "mycelix",
            Self::Staffing | Self::Resources | Self::Safety => "operator",
            Self::Authority => "mycelix",
        }
    }

    fn parse(value: &str) -> Result<Self, InputError> {
        match value {
            "physical_feasibility" => Ok(Self::PhysicalFeasibility),
            "spatial_access" => Ok(Self::SpatialAccess),
            "dependency_independence" => Ok(Self::DependencyIndependence),
            "staffing" => Ok(Self::Staffing),
            "resources" => Ok(Self::Resources),
            "authority" => Ok(Self::Authority),
            "safety" => Ok(Self::Safety),
            _ => Err(InputError(format!("unknown predicate: {value}"))),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Disposition {
    Qualified,
    Unknown,
    NotIndependent,
    Unavailable,
    Expired,
    Revoked,
    WrongScope,
    OutOfDomain,
    Conflict,
}

impl Disposition {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Qualified => "QUALIFIED",
            Self::Unknown => "UNKNOWN",
            Self::NotIndependent => "NOT_INDEPENDENT",
            Self::Unavailable => "UNAVAILABLE",
            Self::Expired => "EXPIRED",
            Self::Revoked => "REVOKED",
            Self::WrongScope => "WRONG_SCOPE",
            Self::OutOfDomain => "OUT_OF_DOMAIN",
            Self::Conflict => "CONFLICT",
        }
    }

    fn parse(value: &str) -> Result<Self, InputError> {
        match value {
            "QUALIFIED" => Ok(Self::Qualified),
            "UNKNOWN" => Ok(Self::Unknown),
            "NOT_INDEPENDENT" => Ok(Self::NotIndependent),
            "UNAVAILABLE" => Ok(Self::Unavailable),
            "EXPIRED" => Ok(Self::Expired),
            "REVOKED" => Ok(Self::Revoked),
            "WRONG_SCOPE" => Ok(Self::WrongScope),
            "OUT_OF_DOMAIN" => Ok(Self::OutOfDomain),
            "CONFLICT" => Ok(Self::Conflict),
            _ => Err(InputError(format!("unknown disposition: {value}"))),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Scenario {
    pub scenario_id: String,
    pub scenario_digest: String,
    pub time_basis_id: String,
    pub evaluation_time_seconds: u64,
    pub clock_uncertainty_seconds: u64,
    pub horizon_seconds: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RecoveryOption {
    pub option_id: String,
    pub service_id: String,
    pub mobilisation_seconds: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PredicateEvidence {
    pub option_id: String,
    pub service_id: String,
    pub scenario_id: String,
    pub scenario_digest: String,
    pub source_schema_version: String,
    pub predicate: PredicateKind,
    pub disposition: Disposition,
    pub source_owner: String,
    pub evidence_ref: String,
    pub time_basis_id: String,
    pub observed_at_seconds: u64,
    pub valid_until_seconds: u64,
    pub clock_uncertainty_seconds: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DebtItem {
    pub debt_id: String,
    pub service_id: String,
    pub dimension: String,
    pub status: String,
    pub source_owner: String,
    pub evidence_ref: String,
    pub time_basis_id: String,
    pub observed_at_seconds: u64,
    pub valid_until_seconds: u64,
    pub clock_uncertainty_seconds: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DebtGroup {
    pub dimension: String,
    pub status: String,
    pub freshness: String,
    pub item_count: usize,
    pub service_count: usize,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OptionResult {
    pub option_id: String,
    pub service_id: String,
    pub disposition: String,
    pub mobilisation_seconds: u64,
    pub reason_codes: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EvaluationReport {
    pub scenario_id: String,
    pub debt_item_count: usize,
    pub debt_groups: Vec<DebtGroup>,
    pub options: Vec<OptionResult>,
}

impl EvaluationReport {
    pub fn render(&self) -> String {
        let mut output = String::new();
        output.push_str("report_schema=fsre-recovery-qualification-report.v0.1\n");
        output.push_str(&format!("scenario_id={}\n", self.scenario_id));
        output.push_str("claim_scope=DECLARED_SCENARIO_ONLY\n");
        output.push_str("execution_authority=NONE\n");
        output.push_str("operational_actuation=NOT_AUTHORISED\n");
        output.push_str(&format!("debt_items={}\n", self.debt_item_count));
        output.push_str(&format!("debt_groups={}\n", self.debt_groups.len()));
        for group in &self.debt_groups {
            output.push_str(&format!(
                "debt_group={}|{}|{}|{}|{}\n",
                group.dimension,
                group.status,
                group.freshness,
                group.item_count,
                group.service_count
            ));
        }
        output.push_str(&format!("recovery_options={}\n", self.options.len()));
        for option in &self.options {
            let reasons = if option.reason_codes.is_empty() {
                "NONE".to_owned()
            } else {
                option.reason_codes.join(",")
            };
            output.push_str(&format!(
                "option={}|{}|{}|{}|{}\n",
                option.option_id,
                option.service_id,
                option.disposition,
                option.mobilisation_seconds,
                reasons
            ));
        }
        output
    }
}

fn required_metadata(
    metadata: &mut BTreeMap<String, String>,
    key: &str,
) -> Result<String, InputError> {
    metadata
        .remove(key)
        .ok_or_else(|| InputError(format!("missing metadata key: {key}")))
}

fn parse_metadata(source: &str) -> Result<BTreeMap<String, String>, InputError> {
    let mut metadata = BTreeMap::new();
    for (index, raw_line) in source.lines().enumerate() {
        let line = raw_line.trim();
        if let Some(comment) = line.strip_prefix('#') {
            if let Some((key, value)) = comment.trim().split_once('=') {
                let key = key.trim().to_owned();
                let value = value.trim().to_owned();
                if key.is_empty() || value.is_empty() {
                    return Err(InputError(format!(
                        "line {}: metadata key/value must not be empty",
                        index + 1
                    )));
                }
                if metadata.insert(key.clone(), value).is_some() {
                    return Err(InputError(format!(
                        "line {}: duplicate metadata key {key}",
                        index + 1
                    )));
                }
            }
        }
    }
    Ok(metadata)
}

fn parse_rows(
    source: &str,
    expected_header: &str,
    expected_columns: usize,
) -> Result<Vec<Vec<String>>, InputError> {
    let mut header_seen = false;
    let mut rows = Vec::new();
    for (index, raw_line) in source.lines().enumerate() {
        let line = raw_line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if !header_seen {
            if line != expected_header {
                return Err(InputError(format!(
                    "line {}: expected header {expected_header}",
                    index + 1
                )));
            }
            header_seen = true;
            continue;
        }
        let columns: Vec<String> = line.split('|').map(|part| part.trim().to_owned()).collect();
        if columns.len() != expected_columns || columns.iter().any(|column| column.is_empty()) {
            return Err(InputError(format!(
                "line {}: expected {expected_columns} non-empty pipe-delimited fields",
                index + 1
            )));
        }
        rows.push(columns);
    }
    if !header_seen {
        return Err(InputError("missing input header".to_owned()));
    }
    Ok(rows)
}

fn parse_u64(value: &str, field: &str) -> Result<u64, InputError> {
    value
        .parse::<u64>()
        .map_err(|error| InputError(format!("invalid {field} {value}: {error}")))
}

pub fn parse_scenario_and_options(
    source: &str,
) -> Result<(Scenario, Vec<RecoveryOption>), InputError> {
    let mut metadata = parse_metadata(source)?;
    let scenario_id = required_metadata(&mut metadata, "scenario_id")?;
    let scenario_digest = required_metadata(&mut metadata, "scenario_digest")?;
    let time_basis_id = required_metadata(&mut metadata, "time_basis_id")?;
    let evaluation_time_seconds = parse_u64(
        &required_metadata(&mut metadata, "evaluation_time_seconds")?,
        "evaluation_time_seconds",
    )?;
    let clock_uncertainty_seconds = parse_u64(
        &required_metadata(&mut metadata, "clock_uncertainty_seconds")?,
        "clock_uncertainty_seconds",
    )?;
    let horizon_seconds = parse_u64(
        &required_metadata(&mut metadata, "horizon_seconds")?,
        "horizon_seconds",
    )?;
    if !metadata.is_empty() {
        return Err(InputError(format!(
            "unknown metadata key: {}",
            metadata.keys().next().expect("metadata is not empty")
        )));
    }
    if horizon_seconds == 0 {
        return Err(InputError(
            "horizon_seconds must be greater than zero".to_owned(),
        ));
    }

    let rows = parse_rows(source, "option_id|service_id|mobilisation_seconds", 3)?;
    let mut seen = BTreeSet::new();
    let mut options = Vec::with_capacity(rows.len());
    for row in rows {
        if !seen.insert(row[0].clone()) {
            return Err(InputError(format!("duplicate option_id: {}", row[0])));
        }
        options.push(RecoveryOption {
            option_id: row[0].clone(),
            service_id: row[1].clone(),
            mobilisation_seconds: parse_u64(&row[2], "mobilisation_seconds")?,
        });
    }

    Ok((
        Scenario {
            scenario_id,
            scenario_digest,
            time_basis_id,
            evaluation_time_seconds,
            clock_uncertainty_seconds,
            horizon_seconds,
        },
        options,
    ))
}

pub fn parse_debt_items(source: &str) -> Result<Vec<DebtItem>, InputError> {
    let rows = parse_rows(
        source,
        "debt_id|service_id|dimension|status|source_owner|evidence_ref|time_basis_id|observed_at_seconds|valid_until_seconds|clock_uncertainty_seconds",
        10,
    )?;
    let mut seen = BTreeSet::new();
    let mut items = Vec::with_capacity(rows.len());
    for row in rows {
        if !seen.insert(row[0].clone()) {
            return Err(InputError(format!("duplicate debt_id: {}", row[0])));
        }
        if !DEBT_STATUSES.contains(&row[3].as_str()) {
            return Err(InputError(format!("unknown debt status: {}", row[3])));
        }
        items.push(DebtItem {
            debt_id: row[0].clone(),
            service_id: row[1].clone(),
            dimension: row[2].clone(),
            status: row[3].clone(),
            source_owner: row[4].clone(),
            evidence_ref: row[5].clone(),
            time_basis_id: row[6].clone(),
            observed_at_seconds: parse_u64(&row[7], "observed_at_seconds")?,
            valid_until_seconds: parse_u64(&row[8], "valid_until_seconds")?,
            clock_uncertainty_seconds: parse_u64(&row[9], "clock_uncertainty_seconds")?,
        });
    }
    Ok(items)
}

pub fn parse_predicate_evidence(source: &str) -> Result<Vec<PredicateEvidence>, InputError> {
    let rows = parse_rows(
        source,
        "option_id|service_id|scenario_id|scenario_digest|source_schema_version|predicate|disposition|source_owner|evidence_ref|time_basis_id|observed_at_seconds|valid_until_seconds|clock_uncertainty_seconds",
        13,
    )?;
    let mut seen = BTreeSet::new();
    let mut evidence = Vec::with_capacity(rows.len());
    for row in rows {
        let predicate = PredicateKind::parse(&row[5])?;
        if !seen.insert((row[0].clone(), predicate)) {
            return Err(InputError(format!(
                "duplicate predicate {} for option {}",
                predicate.as_str(),
                row[0]
            )));
        }
        evidence.push(PredicateEvidence {
            option_id: row[0].clone(),
            service_id: row[1].clone(),
            scenario_id: row[2].clone(),
            scenario_digest: row[3].clone(),
            source_schema_version: row[4].clone(),
            predicate,
            disposition: Disposition::parse(&row[6])?,
            source_owner: row[7].clone(),
            evidence_ref: row[8].clone(),
            time_basis_id: row[9].clone(),
            observed_at_seconds: parse_u64(&row[10], "observed_at_seconds")?,
            valid_until_seconds: parse_u64(&row[11], "valid_until_seconds")?,
            clock_uncertainty_seconds: parse_u64(&row[12], "clock_uncertainty_seconds")?,
        });
    }
    Ok(evidence)
}

fn evidence_is_current(
    scenario: &Scenario,
    time_basis_id: &str,
    observed_at_seconds: u64,
    valid_until_seconds: u64,
    clock_uncertainty_seconds: u64,
    record_id: &str,
) -> Result<bool, InputError> {
    if time_basis_id != scenario.time_basis_id {
        return Err(InputError(format!(
            "time-basis mismatch for {record_id}: expected {}, received {time_basis_id}",
            scenario.time_basis_id
        )));
    }
    let earliest_evaluation = scenario
        .evaluation_time_seconds
        .checked_sub(scenario.clock_uncertainty_seconds)
        .ok_or_else(|| {
            InputError("scenario clock uncertainty exceeds evaluation time".to_owned())
        })?;
    let latest_evaluation = scenario
        .evaluation_time_seconds
        .checked_add(scenario.clock_uncertainty_seconds)
        .ok_or_else(|| InputError("scenario evaluation-time bound overflow".to_owned()))?;
    let latest_observation = observed_at_seconds
        .checked_add(clock_uncertainty_seconds)
        .ok_or_else(|| InputError(format!("observation-time bound overflow for {record_id}")))?;
    let earliest_valid_until = valid_until_seconds
        .checked_sub(clock_uncertainty_seconds)
        .ok_or_else(|| InputError(format!("validity-time bound underflow for {record_id}")))?;
    if latest_observation >= earliest_evaluation {
        return Err(InputError(format!(
            "evidence is not proven to predate evaluation for {record_id}"
        )));
    }
    if earliest_valid_until <= latest_observation {
        return Err(InputError(format!(
            "evidence validity interval is empty under clock uncertainty for {record_id}"
        )));
    }
    Ok(earliest_valid_until > latest_evaluation)
}

pub fn evaluate(
    scenario: &Scenario,
    options: &[RecoveryOption],
    debt_items: &[DebtItem],
    evidence: &[PredicateEvidence],
) -> Result<EvaluationReport, InputError> {
    if scenario.scenario_id.trim().is_empty()
        || scenario.scenario_digest.trim().is_empty()
        || scenario.time_basis_id.trim().is_empty()
        || scenario.horizon_seconds == 0
    {
        return Err(InputError("invalid scenario contract".to_owned()));
    }
    if scenario.clock_uncertainty_seconds > scenario.evaluation_time_seconds {
        return Err(InputError(
            "scenario clock uncertainty exceeds evaluation time".to_owned(),
        ));
    }

    if options
        .iter()
        .any(|option| option.option_id.trim().is_empty() || option.service_id.trim().is_empty())
    {
        return Err(InputError(
            "option_id and service_id must not be empty".to_owned(),
        ));
    }
    let option_ids: BTreeSet<&str> = options
        .iter()
        .map(|option| option.option_id.as_str())
        .collect();
    if option_ids.len() != options.len() {
        return Err(InputError(
            "duplicate option_id in evaluation input".to_owned(),
        ));
    }

    let mut evidence_by_option =
        BTreeMap::<String, BTreeMap<PredicateKind, &PredicateEvidence>>::new();
    for record in evidence {
        if !option_ids.contains(record.option_id.as_str()) {
            return Err(InputError(format!(
                "evidence references unknown option: {}",
                record.option_id
            )));
        }
        let option = options
            .iter()
            .find(|option| option.option_id == record.option_id)
            .expect("option membership was checked above");
        if record.service_id != option.service_id {
            return Err(InputError(format!(
                "service binding mismatch for {}: expected {}, received {}",
                record.option_id, option.service_id, record.service_id
            )));
        }
        if record.scenario_id != scenario.scenario_id
            || record.scenario_digest != scenario.scenario_digest
        {
            return Err(InputError(format!(
                "scenario binding mismatch for {}",
                record.option_id
            )));
        }
        if record.source_schema_version.trim().is_empty() {
            return Err(InputError(format!(
                "missing source_schema_version for {}:{}",
                record.option_id,
                record.predicate.as_str()
            )));
        }
        let schema_owner_prefix = format!("{}.", record.predicate.expected_owner());
        if !record
            .source_schema_version
            .starts_with(&schema_owner_prefix)
        {
            return Err(InputError(format!(
                "source schema owner mismatch for {}:{}",
                record.option_id,
                record.predicate.as_str()
            )));
        }
        if record.source_owner != record.predicate.expected_owner() {
            return Err(InputError(format!(
                "wrong owner for {} on {}: expected {}, received {}",
                record.predicate.as_str(),
                record.option_id,
                record.predicate.expected_owner(),
                record.source_owner
            )));
        }
        if record.evidence_ref.trim().is_empty() {
            return Err(InputError(format!(
                "empty evidence_ref for {} on {}",
                record.predicate.as_str(),
                record.option_id
            )));
        }
        evidence_is_current(
            scenario,
            &record.time_basis_id,
            record.observed_at_seconds,
            record.valid_until_seconds,
            record.clock_uncertainty_seconds,
            &format!("{}:{}", record.option_id, record.predicate.as_str()),
        )?;
        let option_records = evidence_by_option
            .entry(record.option_id.clone())
            .or_default();
        if option_records.insert(record.predicate, record).is_some() {
            return Err(InputError(format!(
                "duplicate predicate {} for option {}",
                record.predicate.as_str(),
                record.option_id
            )));
        }
    }

    let mut debt_ids = BTreeSet::new();
    let mut debt_groups = BTreeMap::<(String, String, String), BTreeSet<String>>::new();
    for item in debt_items {
        if !debt_ids.insert(item.debt_id.as_str()) {
            return Err(InputError(format!("duplicate debt_id: {}", item.debt_id)));
        }
        if item.service_id.trim().is_empty()
            || item.dimension.trim().is_empty()
            || item.source_owner.trim().is_empty()
            || item.evidence_ref.trim().is_empty()
        {
            return Err(InputError(format!(
                "debt item {} has an empty service, dimension, owner or evidence reference",
                item.debt_id
            )));
        }
        let current = evidence_is_current(
            scenario,
            &item.time_basis_id,
            item.observed_at_seconds,
            item.valid_until_seconds,
            item.clock_uncertainty_seconds,
            &item.debt_id,
        )?;
        let freshness = if current { "CURRENT" } else { "STALE" };
        debt_groups
            .entry((
                item.dimension.clone(),
                item.status.clone(),
                freshness.to_owned(),
            ))
            .or_default()
            .insert(item.service_id.clone());
    }

    let mut grouped_counts = BTreeMap::<(String, String, String), usize>::new();
    for item in debt_items {
        let current = evidence_is_current(
            scenario,
            &item.time_basis_id,
            item.observed_at_seconds,
            item.valid_until_seconds,
            item.clock_uncertainty_seconds,
            &item.debt_id,
        )?;
        let freshness = if current { "CURRENT" } else { "STALE" };
        *grouped_counts
            .entry((
                item.dimension.clone(),
                item.status.clone(),
                freshness.to_owned(),
            ))
            .or_default() += 1;
    }

    let mut report_groups = Vec::with_capacity(grouped_counts.len());
    for (key, item_count) in grouped_counts {
        let service_count = debt_groups.get(&key).map(BTreeSet::len).unwrap_or_default();
        report_groups.push(DebtGroup {
            dimension: key.0,
            status: key.1,
            freshness: key.2,
            item_count,
            service_count,
        });
    }

    let mut results = Vec::with_capacity(options.len());
    for option in options {
        let mut reasons = BTreeSet::new();
        if option.mobilisation_seconds > scenario.horizon_seconds {
            reasons.insert("MOBILISATION_EXCEEDS_SCENARIO_HORIZON".to_owned());
        }
        let records = evidence_by_option.get(&option.option_id);
        for predicate in REQUIRED_PREDICATES {
            let Some(record) = records.and_then(|map| map.get(&predicate)) else {
                reasons.insert(format!("MISSING_PREDICATE:{}", predicate.as_str()));
                continue;
            };
            if record.disposition != Disposition::Qualified {
                reasons.insert(format!(
                    "PREDICATE_NOT_QUALIFIED:{}:{}",
                    predicate.as_str(),
                    record.disposition.as_str()
                ));
            }
            let current = evidence_is_current(
                scenario,
                &record.time_basis_id,
                record.observed_at_seconds,
                record.valid_until_seconds,
                record.clock_uncertainty_seconds,
                &format!("{}:{}", option.option_id, predicate.as_str()),
            )?;
            if !current {
                reasons.insert(format!("STALE_EVIDENCE:{}", predicate.as_str()));
            }
        }
        let reason_codes: Vec<String> = reasons.into_iter().collect();
        let disposition = if reason_codes.is_empty() {
            "QUALIFIED_FOR_DECLARED_SCENARIO"
        } else {
            "NOT_QUALIFIED"
        };
        results.push(OptionResult {
            option_id: option.option_id.clone(),
            service_id: option.service_id.clone(),
            disposition: disposition.to_owned(),
            mobilisation_seconds: option.mobilisation_seconds,
            reason_codes,
        });
    }
    results.sort_by(|left, right| left.option_id.cmp(&right.option_id));

    Ok(EvaluationReport {
        scenario_id: scenario.scenario_id.clone(),
        debt_item_count: debt_items.len(),
        debt_groups: report_groups,
        options: results,
    })
}

#[cfg(test)]
mod tests {
    use super::{
        evaluate, parse_debt_items, parse_predicate_evidence, parse_scenario_and_options, DebtItem,
        Disposition, PredicateEvidence, PredicateKind, RecoveryOption, Scenario,
        REQUIRED_PREDICATES,
    };

    fn scenario() -> Scenario {
        Scenario {
            scenario_id: "synthetic-test".to_owned(),
            scenario_digest:
                "sha256:f77fcc28f2a68ef2bdbbc16b36d85d475a5bae3293e1c944b44734630328e579"
                    .to_owned(),
            time_basis_id: "synthetic-clock-v1".to_owned(),
            evaluation_time_seconds: 100,
            clock_uncertainty_seconds: 0,
            horizon_seconds: 1_000,
        }
    }

    fn option(mobilisation_seconds: u64) -> RecoveryOption {
        RecoveryOption {
            option_id: "backup-a".to_owned(),
            service_id: "water-service-a".to_owned(),
            mobilisation_seconds,
        }
    }

    fn evidence(
        predicate: PredicateKind,
        disposition: Disposition,
        valid_until_seconds: u64,
    ) -> PredicateEvidence {
        PredicateEvidence {
            option_id: "backup-a".to_owned(),
            service_id: "water-service-a".to_owned(),
            scenario_id: "synthetic-test".to_owned(),
            scenario_digest: "sha256:f77fcc28f2a68ef2bdbbc16b36d85d475a5bae3293e1c944b44734630328e579".to_owned(),
            source_schema_version: format!("{}.predicate.v0.1", predicate.expected_owner()),
            predicate,
            disposition,
            source_owner: predicate.expected_owner().to_owned(),
            evidence_ref: format!("synthetic:{}:backup-a", predicate.as_str()),
            time_basis_id: "synthetic-clock-v1".to_owned(),
            observed_at_seconds: 10,
            valid_until_seconds,
            clock_uncertainty_seconds: 0,
        }
    }

    fn all_qualified() -> Vec<PredicateEvidence> {
        REQUIRED_PREDICATES
            .iter()
            .map(|predicate| evidence(*predicate, Disposition::Qualified, 500))
            .collect()
    }

    #[test]
    fn rendered_report_never_grants_execution_authority() {
        let result = evaluate(&scenario(), &[option(100)], &[], &all_qualified()).unwrap();
        let rendered = result.render();
        assert!(rendered.contains("claim_scope=DECLARED_SCENARIO_ONLY"));
        assert!(rendered.contains("execution_authority=NONE"));
        assert!(rendered.contains("operational_actuation=NOT_AUTHORISED"));
    }

    #[test]
    fn fully_qualified_option_is_accepted_for_declared_scenario() {
        let result = evaluate(&scenario(), &[option(100)], &[], &all_qualified()).unwrap();
        assert_eq!(
            result.options[0].disposition,
            "QUALIFIED_FOR_DECLARED_SCENARIO"
        );
        assert!(result.options[0].reason_codes.is_empty());
    }

    #[test]
    fn unknown_dependency_disposition_blocks_qualification() {
        let mut records = all_qualified();
        records[2] = evidence(
            PredicateKind::DependencyIndependence,
            Disposition::Unknown,
            500,
        );
        let result = evaluate(&scenario(), &[option(100)], &[], &records).unwrap();
        assert_eq!(result.options[0].disposition, "NOT_QUALIFIED");
        assert!(result.options[0]
            .reason_codes
            .contains(&"PREDICATE_NOT_QUALIFIED:dependency_independence:UNKNOWN".to_owned()));
    }

    #[test]
    fn expired_or_revoked_authority_blocks_option() {
        let mut records = all_qualified();
        records[5] = evidence(PredicateKind::Authority, Disposition::Expired, 500);
        let result = evaluate(&scenario(), &[option(100)], &[], &records).unwrap();
        assert!(result.options[0]
            .reason_codes
            .contains(&"PREDICATE_NOT_QUALIFIED:authority:EXPIRED".to_owned()));
    }

    #[test]
    fn stale_qualified_evidence_is_not_accepted() {
        let mut records = all_qualified();
        records[0] = evidence(
            PredicateKind::PhysicalFeasibility,
            Disposition::Qualified,
            100,
        );
        let result = evaluate(&scenario(), &[option(100)], &[], &records).unwrap();
        assert!(result.options[0]
            .reason_codes
            .contains(&"STALE_EVIDENCE:physical_feasibility".to_owned()));
    }

    #[test]
    fn missing_predicates_are_explicitly_reported() {
        let result = evaluate(&scenario(), &[option(100)], &[], &[]).unwrap();
        assert_eq!(result.options[0].reason_codes.len(), 7);
        assert!(result.options[0]
            .reason_codes
            .contains(&"MISSING_PREDICATE:authority".to_owned()));
    }

    #[test]
    fn mobilization_beyond_window_blocks_option() {
        let result = evaluate(&scenario(), &[option(1_001)], &[], &all_qualified()).unwrap();
        assert!(result.options[0]
            .reason_codes
            .contains(&"MOBILISATION_EXCEEDS_SCENARIO_HORIZON".to_owned()));
    }

    #[test]
    fn wrong_owner_is_rejected_instead_of_relabelled() {
        let mut records = all_qualified();
        records[1].source_owner = "mycelix".to_owned();
        assert!(evaluate(&scenario(), &[option(100)], &[], &records).is_err());
    }

    #[test]
    fn future_dated_evidence_is_rejected() {
        let mut records = all_qualified();
        records[0].observed_at_seconds = 101;
        assert!(evaluate(&scenario(), &[option(100)], &[], &records).is_err());
    }

    #[test]
    fn duplicate_predicate_is_rejected() {
        let mut records = all_qualified();
        records.push(records[0].clone());
        assert!(evaluate(&scenario(), &[option(100)], &[], &records).is_err());
    }

    #[test]
    fn debt_summary_preserves_dimension_status_and_freshness() {
        let debt = vec![
            DebtItem {
                debt_id: "d1".to_owned(),
                service_id: "water-a".to_owned(),
                dimension: "workforce_competence".to_owned(),
                status: "QUALIFIED_DEFICIENCY".to_owned(),
                source_owner: "operator".to_owned(),
                evidence_ref: "synthetic:roster-1".to_owned(),
                time_basis_id: "synthetic-clock-v1".to_owned(),
                observed_at_seconds: 10,
                valid_until_seconds: 500,
                clock_uncertainty_seconds: 0,
            },
            DebtItem {
                debt_id: "d2".to_owned(),
                service_id: "water-b".to_owned(),
                dimension: "workforce_competence".to_owned(),
                status: "QUALIFIED_DEFICIENCY".to_owned(),
                source_owner: "operator".to_owned(),
                evidence_ref: "synthetic:roster-2".to_owned(),
                time_basis_id: "synthetic-clock-v1".to_owned(),
                observed_at_seconds: 10,
                valid_until_seconds: 100,
                clock_uncertainty_seconds: 0,
            },
        ];
        let report = evaluate(&scenario(), &[], &debt, &[]).unwrap();
        assert_eq!(report.debt_item_count, 2);
        assert_eq!(report.debt_groups.len(), 2);
        assert_eq!(report.debt_groups[0].freshness, "CURRENT");
        assert_eq!(report.debt_groups[1].freshness, "STALE");
    }

    #[test]
    fn scenario_and_options_parser_requires_explicit_metadata() {
        let source = "# scenario_id=fixture\n# scenario_digest=sha256:0984038d21f0a1a80d86f74d6776f8f2defba3e997d29eb67a732943004fc0b6\n# time_basis_id=synthetic-clock-v1\n# evaluation_time_seconds=100\n# clock_uncertainty_seconds=0\n# horizon_seconds=1000\noption_id|service_id|mobilisation_seconds\no1|water-a|10\n";
        let (parsed_scenario, options) = parse_scenario_and_options(source).unwrap();
        assert_eq!(parsed_scenario.scenario_id, "fixture");
        assert_eq!(options.len(), 1);
    }

    #[test]
    fn predicate_parser_rejects_duplicate_semantics() {
        let header = "option_id|service_id|scenario_id|scenario_digest|source_schema_version|predicate|disposition|source_owner|evidence_ref|time_basis_id|observed_at_seconds|valid_until_seconds|clock_uncertainty_seconds";
        let row = "o1|water-a|fixture|sha256:0984038d21f0a1a80d86f74d6776f8f2defba3e997d29eb67a732943004fc0b6|mycelix.authority.v0.1|authority|QUALIFIED|mycelix|synthetic:grant|synthetic-clock-v1|1|100|0";
        let source = format!("{header}\n{row}\n{row}\n");
        assert!(parse_predicate_evidence(&source).is_err());
    }

    #[test]
    fn mismatched_time_basis_is_rejected() {
        let mut records = all_qualified();
        records[0].time_basis_id = "unrelated-clock".to_owned();
        assert!(evaluate(&scenario(), &[option(100)], &[], &records).is_err());
    }

    #[test]
    fn clock_uncertainty_can_make_nominally_fresh_evidence_stale() {
        let mut bounded_scenario = scenario();
        bounded_scenario.clock_uncertainty_seconds = 5;
        let mut records = all_qualified();
        records[0].valid_until_seconds = 108;
        records[0].clock_uncertainty_seconds = 5;
        let result = evaluate(&bounded_scenario, &[option(100)], &[], &records).unwrap();
        assert!(result.options[0]
            .reason_codes
            .contains(&"STALE_EVIDENCE:physical_feasibility".to_owned()));
    }

    #[test]
    fn clock_uncertainty_that_overlaps_observation_and_evaluation_is_rejected() {
        let mut bounded_scenario = scenario();
        bounded_scenario.clock_uncertainty_seconds = 5;
        let mut records = all_qualified();
        records[0].observed_at_seconds = 96;
        records[0].clock_uncertainty_seconds = 5;
        assert!(evaluate(&bounded_scenario, &[option(100)], &[], &records).is_err());
    }

    #[test]
    fn service_binding_mismatch_is_rejected() {
        let mut records = all_qualified();
        records[0].service_id = "different-service".to_owned();
        assert!(evaluate(&scenario(), &[option(100)], &[], &records).is_err());
    }

    #[test]
    fn scenario_revision_mismatch_is_rejected() {
        let mut records = all_qualified();
        records[0].scenario_digest = "synthetic:scenario-digest:older-v0".to_owned();
        assert!(evaluate(&scenario(), &[option(100)], &[], &records).is_err());
    }

    #[test]
    fn source_schema_namespace_must_match_predicate_owner() {
        let mut records = all_qualified();
        records[0].source_schema_version = "sol-atlas.physical-feasibility.v0.1".to_owned();
        assert!(evaluate(&scenario(), &[option(100)], &[], &records).is_err());
    }

    #[test]
    fn debt_items_require_nonempty_provenance_fields() {
        let mut debt = vec![DebtItem {
            debt_id: "d1".to_owned(),
            service_id: "water-a".to_owned(),
            dimension: "maintenance".to_owned(),
            status: "QUALIFIED_DEFICIENCY".to_owned(),
            source_owner: "operator".to_owned(),
            evidence_ref: "synthetic:work-order".to_owned(),
            time_basis_id: "synthetic-clock-v1".to_owned(),
            observed_at_seconds: 10,
            valid_until_seconds: 500,
            clock_uncertainty_seconds: 0,
        }];
        debt[0].evidence_ref.clear();
        assert!(evaluate(&scenario(), &[], &debt, &[]).is_err());
    }

    #[test]
    fn empty_source_schema_version_is_rejected() {
        let mut records = all_qualified();
        records[0].source_schema_version.clear();
        assert!(evaluate(&scenario(), &[option(100)], &[], &records).is_err());
    }

    #[test]
    fn debt_parser_rejects_unknown_states() {
        let source = "debt_id|service_id|dimension|status|source_owner|evidence_ref|time_basis_id|observed_at_seconds|valid_until_seconds|clock_uncertainty_seconds\nd1|water-a|maintenance|MAGICALLY_FIXED|operator|synthetic:wo|synthetic-clock-v1|1|100|0\n";
        assert!(parse_debt_items(source).is_err());
    }
}
