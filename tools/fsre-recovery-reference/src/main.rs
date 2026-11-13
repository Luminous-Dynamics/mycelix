#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs;

#[derive(Clone, Debug)]
struct Scenario {
    id: String,
    digest: String,
    time_basis: String,
    evaluation: u64,
    uncertainty: u64,
    horizon: u64,
}

#[derive(Clone, Debug)]
struct OptionCandidate {
    id: String,
    service: String,
    mobilisation: u64,
}

#[derive(Clone, Debug)]
struct Predicate {
    option: String,
    service: String,
    scenario: String,
    digest: String,
    schema: String,
    kind: String,
    disposition: String,
    owner: String,
    evidence_ref: String,
    time_basis: String,
    observed: u64,
    valid_until: u64,
    uncertainty: u64,
}

const REQUIRED: [&str; 7] = [
    "physical_feasibility",
    "spatial_access",
    "dependency_independence",
    "staffing",
    "resources",
    "authority",
    "safety",
];

fn owner(kind: &str) -> Option<&'static str> {
    match kind {
        "physical_feasibility" => Some("symtropy"),
        "spatial_access" => Some("sol-atlas"),
        "dependency_independence" | "authority" => Some("mycelix"),
        "staffing" | "resources" | "safety" => Some("operator"),
        _ => None,
    }
}

fn fields(line: &str, expected: usize, line_number: usize) -> Result<Vec<String>, String> {
    let values: Vec<String> = line.split('|').map(|part| part.trim().to_owned()).collect();
    if values.len() != expected || values.iter().any(|value| value.is_empty()) {
        return Err(format!("line {line_number}: expected {expected} non-empty fields"));
    }
    Ok(values)
}

fn integer(value: &str, label: &str) -> Result<u64, String> {
    value
        .parse::<u64>()
        .map_err(|error| format!("invalid {label} {value}: {error}"))
}

fn read_scenario(source: &str) -> Result<(Scenario, Vec<OptionCandidate>), String> {
    let mut metadata = BTreeMap::new();
    let mut header_seen = false;
    let mut options = Vec::new();
    let mut ids = BTreeSet::new();

    for (index, raw) in source.lines().enumerate() {
        let line_number = index + 1;
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            if let Some((key, value)) = comment.trim().split_once('=') {
                if metadata.insert(key.trim().to_owned(), value.trim().to_owned()).is_some() {
                    return Err(format!("line {line_number}: duplicate metadata key"));
                }
            }
            continue;
        }
        if !header_seen {
            if line != "option_id|service_id|mobilisation_seconds" {
                return Err(format!("line {line_number}: unexpected scenario header"));
            }
            header_seen = true;
            continue;
        }
        let row = fields(line, 3, line_number)?;
        if !ids.insert(row[0].clone()) {
            return Err(format!("duplicate option ID {}", row[0]));
        }
        options.push(OptionCandidate {
            id: row[0].clone(),
            service: row[1].clone(),
            mobilisation: integer(&row[2], "mobilisation_seconds")?,
        });
    }
    if !header_seen || options.is_empty() {
        return Err("scenario is missing its option header or options".to_owned());
    }

    let mut take = |key: &str| {
        metadata
            .remove(key)
            .ok_or_else(|| format!("missing scenario metadata {key}"))
    };
    let scenario = Scenario {
        id: take("scenario_id")?,
        digest: take("scenario_digest")?,
        time_basis: take("time_basis_id")?,
        evaluation: integer(&take("evaluation_time_seconds")?, "evaluation_time_seconds")?,
        uncertainty: integer(
            &take("clock_uncertainty_seconds")?,
            "clock_uncertainty_seconds",
        )?,
        horizon: integer(&take("horizon_seconds")?, "horizon_seconds")?,
    };
    if !metadata.is_empty() {
        return Err(format!(
            "unknown scenario metadata {}",
            metadata.keys().next().expect("metadata is nonempty")
        ));
    }
    if scenario.id.is_empty()
        || scenario.time_basis.is_empty()
        || scenario.horizon == 0
        || !valid_digest(&scenario.digest)
        || scenario.uncertainty > scenario.evaluation
    {
        return Err("invalid scenario identity, digest, clock or horizon".to_owned());
    }
    Ok((scenario, options))
}

fn valid_digest(value: &str) -> bool {
    let Some(hex) = value.strip_prefix("sha256:") else {
        return false;
    };
    hex.len() == 64
        && hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn read_predicates(source: &str) -> Result<Vec<Predicate>, String> {
    let mut records = Vec::new();
    let mut seen = BTreeSet::new();
    let mut header_seen = false;
    for (index, raw) in source.lines().enumerate() {
        let line_number = index + 1;
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if !header_seen {
            let expected = "option_id|service_id|scenario_id|scenario_digest|source_schema_version|predicate|disposition|source_owner|evidence_ref|time_basis_id|observed_at_seconds|valid_until_seconds|clock_uncertainty_seconds";
            if line != expected {
                return Err(format!("line {line_number}: unexpected predicate header"));
            }
            header_seen = true;
            continue;
        }
        let row = fields(line, 13, line_number)?;
        if owner(&row[5]).is_none() {
            return Err(format!("line {line_number}: unknown predicate {}", row[5]));
        }
        if !seen.insert((row[0].clone(), row[5].clone())) {
            return Err(format!(
                "duplicate predicate {} for option {}",
                row[5], row[0]
            ));
        }
        records.push(Predicate {
            option: row[0].clone(),
            service: row[1].clone(),
            scenario: row[2].clone(),
            digest: row[3].clone(),
            schema: row[4].clone(),
            kind: row[5].clone(),
            disposition: row[6].clone(),
            owner: row[7].clone(),
            evidence_ref: row[8].clone(),
            time_basis: row[9].clone(),
            observed: integer(&row[10], "observed_at_seconds")?,
            valid_until: integer(&row[11], "valid_until_seconds")?,
            uncertainty: integer(&row[12], "clock_uncertainty_seconds")?,
        });
    }
    if !header_seen {
        return Err("predicate header is missing".to_owned());
    }
    Ok(records)
}

fn record_is_current(scenario: &Scenario, record: &Predicate) -> Result<bool, String> {
    if record.time_basis != scenario.time_basis {
        return Err(format!("time basis mismatch for {}:{}", record.option, record.kind));
    }
    let earliest_evaluation = scenario
        .evaluation
        .checked_sub(scenario.uncertainty)
        .ok_or_else(|| "evaluation time lower bound underflow".to_owned())?;
    let latest_evaluation = scenario
        .evaluation
        .checked_add(scenario.uncertainty)
        .ok_or_else(|| "evaluation time upper bound overflow".to_owned())?;
    let latest_observation = record
        .observed
        .checked_add(record.uncertainty)
        .ok_or_else(|| "observation time upper bound overflow".to_owned())?;
    let earliest_validity_end = record
        .valid_until
        .checked_sub(record.uncertainty)
        .ok_or_else(|| "validity lower bound underflow".to_owned())?;
    if latest_observation >= earliest_evaluation {
        return Err(format!("observation not proven prior for {}:{}", record.option, record.kind));
    }
    if earliest_validity_end <= latest_observation {
        return Err(format!("empty conservative validity interval for {}:{}", record.option, record.kind));
    }
    Ok(earliest_validity_end > latest_evaluation)
}

fn assess(
    scenario: &Scenario,
    options: &[OptionCandidate],
    predicates: &[Predicate],
) -> Result<String, String> {
    let known_options: BTreeSet<&str> = options.iter().map(|option| option.id.as_str()).collect();
    let mut by_option = BTreeMap::<&str, BTreeMap<&str, &Predicate>>::new();
    for record in predicates {
        if !known_options.contains(record.option.as_str()) {
            return Err(format!("predicate references unknown option {}", record.option));
        }
        let candidate = options.iter().find(|option| option.id == record.option).unwrap();
        if candidate.service != record.service
            || scenario.id != record.scenario
            || scenario.digest != record.digest
        {
            return Err(format!("scope mismatch for {}:{}", record.option, record.kind));
        }
        let expected_owner = owner(&record.kind).unwrap();
        if record.owner != expected_owner || !record.schema.starts_with(&format!("{expected_owner}.")) {
            return Err(format!("owner/schema mismatch for {}:{}", record.option, record.kind));
        }
        if record.evidence_ref.trim().is_empty() {
            return Err(format!("missing evidence reference for {}:{}", record.option, record.kind));
        }
        by_option.entry(record.option.as_str()).or_default().insert(record.kind.as_str(), record);
    }

    let mut lines = Vec::new();
    for option in options {
        let mut reasons = BTreeSet::new();
        if option.mobilisation > scenario.horizon {
            reasons.insert("MOBILISATION_EXCEEDS_SCENARIO_HORIZON".to_owned());
        }
        for kind in REQUIRED {
            match by_option.get(option.id.as_str()).and_then(|records| records.get(kind)) {
                None => {
                    reasons.insert(format!("MISSING_PREDICATE:{kind}"));
                }
                Some(record) => {
                    if record.disposition != "QUALIFIED" {
                        reasons.insert(format!(
                            "PREDICATE_NOT_QUALIFIED:{kind}:{}",
                            record.disposition
                        ));
                    }
                    if !record_is_current(scenario, record)? {
                        reasons.insert(format!("STALE_EVIDENCE:{kind}"));
                    }
                }
            }
        }
        let reason_text = if reasons.is_empty() {
            "NONE".to_owned()
        } else {
            reasons.into_iter().collect::<Vec<_>>().join(",")
        };
        let disposition = if reason_text == "NONE" {
            "QUALIFIED_FOR_DECLARED_SCENARIO"
        } else {
            "NOT_QUALIFIED"
        };
        lines.push(format!(
            "option={}|{}|{}|{}",
            option.id, option.service, disposition, reason_text
        ));
    }
    lines.sort();
    Ok(format!(
        "reference_schema=fsre-independent-recovery-reference.v0.1\nscenario_id={}\nscenario_digest={}\noptions={}\n{}\n",
        scenario.id,
        scenario.digest,
        lines.len(),
        lines.join("\n")
    ))
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() != 2 {
        return Err("expected scenario-options.psv and predicate-evidence.psv".to_owned());
    }
    let scenario_source = fs::read_to_string(&args[0])
        .map_err(|error| format!("could not read {}: {error}", args[0]))?;
    let predicate_source = fs::read_to_string(&args[1])
        .map_err(|error| format!("could not read {}: {error}", args[1]))?;
    let (scenario, options) = read_scenario(&scenario_source)?;
    let predicates = read_predicates(&predicate_source)?;
    print!("{}", assess(&scenario, &options, &predicates)?);
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("error: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::{record_is_current, valid_digest, Scenario, Predicate};

    fn scenario() -> Scenario {
        Scenario {
            id: "scenario-a".to_owned(),
            digest: "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".to_owned(),
            time_basis: "clock-a".to_owned(),
            evaluation: 100,
            uncertainty: 5,
            horizon: 300,
        }
    }

    fn record() -> Predicate {
        Predicate {
            option: "option-a".to_owned(),
            service: "service-a".to_owned(),
            scenario: "scenario-a".to_owned(),
            digest: "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".to_owned(),
            schema: "mycelix.authority.v0.1".to_owned(),
            kind: "authority".to_owned(),
            disposition: "QUALIFIED".to_owned(),
            owner: "mycelix".to_owned(),
            evidence_ref: "synthetic:grant".to_owned(),
            time_basis: "clock-a".to_owned(),
            observed: 50,
            valid_until: 120,
            uncertainty: 2,
        }
    }

    #[test]
    fn canonical_digest_shape_is_exact() {
        assert!(valid_digest("sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"));
        assert!(!valid_digest("sha256:AAAA"));
        assert!(!valid_digest("sha256:gggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggg"));
    }

    #[test]
    fn conservative_clock_interval_detects_staleness() {
        let mut evidence = record();
        evidence.valid_until = 106;
        evidence.uncertainty = 2;
        assert!(!record_is_current(&scenario(), &evidence).unwrap());
    }

    #[test]
    fn clock_overlap_is_rejected() {
        let mut evidence = record();
        evidence.observed = 96;
        evidence.uncertainty = 5;
        assert!(record_is_current(&scenario(), &evidence).is_err());
    }

    #[test]
    fn wrong_time_basis_is_rejected() {
        let mut evidence = record();
        evidence.time_basis = "other-clock".to_owned();
        assert!(record_is_current(&scenario(), &evidence).is_err());
    }
}
