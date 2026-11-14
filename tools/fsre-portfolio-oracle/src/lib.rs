#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

const HEADER: &str = "record_type|record_id|option_id|service_id|cohort_id|scenario_id|interval_id|subject_id|quantity|unit|status|source_owner|evidence_ref";

type OptionKey = (String, String);
type CohortKey = (String, String);

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReferenceError(pub String);

impl std::fmt::Display for ReferenceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for ReferenceError {}

#[derive(Clone, Debug)]
struct Fact {
    kind: String,
    id: String,
    option: String,
    service: String,
    cohort: String,
    scenario: String,
    interval: String,
    subject: String,
    quantity: Option<u64>,
    unit: Option<String>,
    status: String,
    owner: String,
    evidence_ref: String,
}

impl Fact {
    fn option_key(&self) -> OptionKey {
        (self.option.clone(), self.service.clone())
    }
}

#[derive(Clone, Debug)]
struct Snapshot {
    portfolio: String,
    scenario: String,
    interval: String,
    snapshot: String,
    facts: Vec<Fact>,
}

fn safe_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 160
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric()
                || matches!(byte, b'_' | b'-' | b'.' | b':' | b'/')
        })
}

fn quantity(value: &str, line: usize) -> Result<Option<u64>, ReferenceError> {
    if value.is_empty() {
        return Ok(None);
    }
    value.parse::<u64>().map(Some).map_err(|error| {
        ReferenceError(format!(
            "line {line}: invalid non-negative integer quantity: {error}"
        ))
    })
}

fn parse_fact(line: &str, line_number: usize) -> Result<Fact, ReferenceError> {
    let fields: Vec<&str> = line.split('|').map(str::trim).collect();
    if fields.len() != 13 {
        return Err(ReferenceError(format!(
            "line {line_number}: expected 13 pipe-delimited columns"
        )));
    }
    let fact = Fact {
        kind: fields[0].to_owned(),
        id: fields[1].to_owned(),
        option: fields[2].to_owned(),
        service: fields[3].to_owned(),
        cohort: fields[4].to_owned(),
        scenario: fields[5].to_owned(),
        interval: fields[6].to_owned(),
        subject: fields[7].to_owned(),
        quantity: quantity(fields[8], line_number)?,
        unit: (!fields[9].is_empty()).then(|| fields[9].to_owned()),
        status: fields[10].to_owned(),
        owner: fields[11].to_owned(),
        evidence_ref: fields[12].to_owned(),
    };
    for (label, value) in [
        ("record_type", fact.kind.as_str()),
        ("record_id", fact.id.as_str()),
        ("scenario_id", fact.scenario.as_str()),
        ("interval_id", fact.interval.as_str()),
        ("source_owner", fact.owner.as_str()),
    ] {
        if !safe_id(value) {
            return Err(ReferenceError(format!(
                "line {line_number}: invalid {label}"
            )));
        }
    }
    if fact.evidence_ref.is_empty() || fact.evidence_ref.len() > 512 {
        return Err(ReferenceError(format!(
            "line {line_number}: invalid evidence reference"
        )));
    }

    match fact.kind.as_str() {
        "OPTION_QUALIFICATION" => {
            if !safe_id(&fact.option)
                || !safe_id(&fact.service)
                || !fact.cohort.is_empty()
                || !fact.subject.is_empty()
                || fact.quantity.is_some()
                || fact.unit.is_some()
                || !matches!(
                    fact.status.as_str(),
                    "QUALIFIED" | "UNKNOWN" | "NOT_QUALIFIED" | "REVOKED" | "EXPIRED" | "OUT_OF_DOMAIN"
                )
            {
                return Err(ReferenceError(format!(
                    "line {line_number}: malformed option qualification"
                )));
            }
        }
        "OPTION_CAPACITY" => {
            if !safe_id(&fact.option)
                || !safe_id(&fact.service)
                || !fact.cohort.is_empty()
                || !fact.subject.is_empty()
            {
                return Err(ReferenceError(format!(
                    "line {line_number}: malformed option-capacity scope"
                )));
            }
            match fact.status.as_str() {
                "QUALIFIED" if fact.quantity.is_some() && fact.unit.is_some() => {}
                "UNKNOWN" if fact.quantity.is_none() && fact.unit.is_none() => {}
                _ => {
                    return Err(ReferenceError(format!(
                        "line {line_number}: option capacity status/measurement mismatch"
                    )));
                }
            }
        }
        "ALLOCATION" => {
            if !safe_id(&fact.option)
                || !safe_id(&fact.service)
                || !safe_id(&fact.cohort)
                || !fact.subject.is_empty()
                || fact.quantity.is_none()
                || fact.unit.is_none()
                || fact.status != "PROPOSED"
            {
                return Err(ReferenceError(format!(
                    "line {line_number}: malformed proposed allocation"
                )));
            }
        }
        "SERVICE_FLOOR" => {
            if !fact.option.is_empty()
                || !safe_id(&fact.service)
                || !safe_id(&fact.cohort)
                || !fact.subject.is_empty()
                || fact.quantity.is_none()
                || fact.unit.is_none()
                || fact.status != "REQUIRED"
            {
                return Err(ReferenceError(format!(
                    "line {line_number}: malformed service floor"
                )));
            }
        }
        "RESOURCE_PROFILE" | "DOMAIN_PROFILE" => {
            if !safe_id(&fact.option)
                || !safe_id(&fact.service)
                || !fact.cohort.is_empty()
                || !fact.subject.is_empty()
                || fact.quantity.is_some()
                || fact.unit.is_some()
                || !matches!(fact.status.as_str(), "COMPLETE" | "UNKNOWN")
            {
                return Err(ReferenceError(format!(
                    "line {line_number}: malformed completeness profile"
                )));
            }
        }
        "RESOURCE_USE" => {
            if !safe_id(&fact.option)
                || !safe_id(&fact.service)
                || !fact.cohort.is_empty()
                || !safe_id(&fact.subject)
            {
                return Err(ReferenceError(format!(
                    "line {line_number}: malformed resource-use scope"
                )));
            }
            match fact.status.as_str() {
                "QUALIFIED" if fact.quantity.is_some() && fact.unit.is_some() => {}
                "UNKNOWN" if fact.quantity.is_none() && fact.unit.is_none() => {}
                _ => {
                    return Err(ReferenceError(format!(
                        "line {line_number}: resource-use status/measurement mismatch"
                    )));
                }
            }
        }
        "RESOURCE_CAPACITY" => {
            if !fact.option.is_empty()
                || !fact.service.is_empty()
                || !fact.cohort.is_empty()
                || !safe_id(&fact.subject)
            {
                return Err(ReferenceError(format!(
                    "line {line_number}: malformed resource-capacity scope"
                )));
            }
            match fact.status.as_str() {
                "QUALIFIED" if fact.quantity.is_some() && fact.unit.is_some() => {}
                "UNKNOWN" if fact.quantity.is_none() && fact.unit.is_none() => {}
                _ => {
                    return Err(ReferenceError(format!(
                        "line {line_number}: resource-capacity status/measurement mismatch"
                    )));
                }
            }
        }
        "OPTION_DOMAIN" => {
            if !safe_id(&fact.option)
                || !safe_id(&fact.service)
                || !fact.cohort.is_empty()
                || !safe_id(&fact.subject)
                || fact.quantity.is_some()
                || fact.unit.is_some()
                || fact.status != "MEMBER"
            {
                return Err(ReferenceError(format!(
                    "line {line_number}: malformed option/domain membership"
                )));
            }
        }
        "DOMAIN_STATE" => {
            if !fact.option.is_empty()
                || !fact.service.is_empty()
                || !fact.cohort.is_empty()
                || !safe_id(&fact.subject)
                || fact.quantity.is_some()
                || fact.unit.is_some()
                || !matches!(fact.status.as_str(), "CLEAR" | "FAILED" | "UNKNOWN")
            {
                return Err(ReferenceError(format!(
                    "line {line_number}: malformed domain state"
                )));
            }
        }
        _ => {
            return Err(ReferenceError(format!(
                "line {line_number}: unsupported record type {}",
                fact.kind
            )));
        }
    }
    Ok(fact)
}

fn parse_snapshot(source: &str) -> Result<Snapshot, ReferenceError> {
    let mut meta = BTreeMap::<String, String>::new();
    let mut facts = Vec::new();
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
                if meta.insert(key.trim().to_owned(), value.trim().to_owned()).is_some() {
                    return Err(ReferenceError(format!(
                        "line {line_number}: duplicate metadata field"
                    )));
                }
            }
            continue;
        }
        if !header_seen {
            if line != HEADER {
                return Err(ReferenceError(format!(
                    "line {line_number}: unexpected header"
                )));
            }
            header_seen = true;
            continue;
        }
        let fact = parse_fact(line, line_number)?;
        if !ids.insert(fact.id.clone()) {
            return Err(ReferenceError(format!(
                "line {line_number}: duplicate record ID {}",
                fact.id
            )));
        }
        facts.push(fact);
    }
    if !header_seen || facts.is_empty() {
        return Err(ReferenceError("missing header or evidence rows".to_owned()));
    }

    let mut take = |key: &str| {
        let value = meta
            .remove(key)
            .ok_or_else(|| ReferenceError(format!("missing metadata field {key}")))?;
        if !safe_id(&value) {
            return Err(ReferenceError(format!("invalid metadata field {key}")));
        }
        Ok(value)
    };
    let snapshot = Snapshot {
        portfolio: take("portfolio_id")?,
        scenario: take("scenario_id")?,
        interval: take("interval_id")?,
        snapshot: take("snapshot_id")?,
        facts,
    };
    if let Some(extra) = meta.keys().next() {
        return Err(ReferenceError(format!("unsupported metadata field {extra}")));
    }
    for fact in &snapshot.facts {
        if fact.scenario != snapshot.scenario || fact.interval != snapshot.interval {
            return Err(ReferenceError(format!(
                "record {} does not belong to the frozen scenario/interval",
                fact.id
            )));
        }
    }
    let mut option_service = BTreeMap::<String, String>::new();
    for fact in &snapshot.facts {
        if fact.option.is_empty() {
            continue;
        }
        if let Some(previous) = option_service.insert(fact.option.clone(), fact.service.clone()) {
            if previous != fact.service {
                return Err(ReferenceError(format!(
                    "option {} crosses service lanes",
                    fact.option
                )));
            }
        }
    }
    Ok(snapshot)
}

#[derive(Clone, Debug, Default)]
struct Candidate {
    amount: u64,
    unit: Option<String>,
    qualified: bool,
    blocked: bool,
    unknown: bool,
    reasons: BTreeSet<String>,
}

#[derive(Clone, Debug, Default)]
struct Cohort {
    amount: u64,
    floor: Option<u64>,
    unit: Option<String>,
    unknown: bool,
}

#[derive(Clone, Debug, Default)]
struct SharedResource {
    demand: u64,
    capacity: Option<u64>,
    unit: Option<String>,
    unknown: bool,
}

fn checked_add(total: &mut u64, addend: u64, label: &str) -> Result<(), ReferenceError> {
    *total = total
        .checked_add(addend)
        .ok_or_else(|| ReferenceError(format!("overflow in {label}")))?;
    Ok(())
}

fn insert_once<K: Ord, V>(
    map: &mut BTreeMap<K, V>,
    key: K,
    value: V,
    label: &str,
) -> Result<(), ReferenceError> {
    if let std::collections::btree_map::Entry::Vacant(entry) = map.entry(key) {
        entry.insert(value);
        Ok(())
    } else {
        Err(ReferenceError(format!("duplicate {label}")))
    }
}

fn reference_evaluate(snapshot: &Snapshot) -> Result<String, ReferenceError> {
    let mut qualifications = BTreeMap::<OptionKey, &Fact>::new();
    let mut option_capacities = BTreeMap::<OptionKey, &Fact>::new();
    let mut resource_profiles = BTreeMap::<OptionKey, &Fact>::new();
    let mut domain_profiles = BTreeMap::<OptionKey, &Fact>::new();
    let mut allocations = Vec::<&Fact>::new();
    let mut floors = BTreeMap::<CohortKey, &Fact>::new();
    let mut resource_uses = Vec::<&Fact>::new();
    let mut resource_capacities = BTreeMap::<String, &Fact>::new();
    let mut memberships = BTreeMap::<OptionKey, Vec<&Fact>>::new();
    let mut domain_states = BTreeMap::<String, &Fact>::new();

    for fact in &snapshot.facts {
        match fact.kind.as_str() {
            "OPTION_QUALIFICATION" => insert_once(&mut qualifications, fact.option_key(), fact, "option qualification")?,
            "OPTION_CAPACITY" => insert_once(&mut option_capacities, fact.option_key(), fact, "option capacity")?,
            "RESOURCE_PROFILE" => insert_once(&mut resource_profiles, fact.option_key(), fact, "resource profile")?,
            "DOMAIN_PROFILE" => insert_once(&mut domain_profiles, fact.option_key(), fact, "domain profile")?,
            "ALLOCATION" => allocations.push(fact),
            "SERVICE_FLOOR" => insert_once(&mut floors, (fact.service.clone(), fact.cohort.clone()), fact, "service floor")?,
            "RESOURCE_USE" => resource_uses.push(fact),
            "RESOURCE_CAPACITY" => insert_once(&mut resource_capacities, fact.subject.clone(), fact, "resource capacity")?,
            "OPTION_DOMAIN" => memberships.entry(fact.option_key()).or_default().push(fact),
            "DOMAIN_STATE" => insert_once(&mut domain_states, fact.subject.clone(), fact, "failure-domain state")?,
            _ => unreachable!(),
        }
    }

    let mut totals = BTreeMap::<OptionKey, (u64, Option<String>)>::new();
    for allocation in &allocations {
        let key = allocation.option_key();
        let value = totals.entry(key).or_insert((0, allocation.unit.clone()));
        if value.1 != allocation.unit {
            return Err(ReferenceError(format!(
                "allocation unit mismatch for {}",
                allocation.option
            )));
        }
        checked_add(
            &mut value.0,
            allocation.quantity.expect("validated allocation amount"),
            "option allocation",
        )?;
    }

    let mut candidates = BTreeMap::<OptionKey, Candidate>::new();
    let mut unknown_evidence = false;
    let mut infeasible = false;
    for (key, (amount, unit)) in &totals {
        let candidate = candidates.entry(key.clone()).or_default();
        candidate.amount = *amount;
        candidate.unit = unit.clone();
        match qualifications.get(key) {
            None => {
                candidate.unknown = true;
                candidate.reasons.insert("MISSING_OPTION_QUALIFICATION".to_owned());
            }
            Some(fact) if fact.status == "QUALIFIED" => candidate.qualified = true,
            Some(fact) if fact.status == "UNKNOWN" => {
                candidate.unknown = true;
                candidate.reasons.insert("OPTION_QUALIFICATION_UNKNOWN".to_owned());
            }
            Some(fact) => {
                candidate.blocked = true;
                candidate.reasons.insert(format!("OPTION_{}", fact.status));
            }
        }
        match option_capacities.get(key) {
            None => {
                candidate.unknown = true;
                candidate.reasons.insert("MISSING_OPTION_CAPACITY".to_owned());
            }
            Some(fact) if fact.status == "UNKNOWN" => {
                candidate.unknown = true;
                candidate.reasons.insert("OPTION_CAPACITY_UNKNOWN".to_owned());
            }
            Some(fact) => {
                if fact.unit != *unit {
                    return Err(ReferenceError(format!(
                        "option capacity unit mismatch for {}",
                        key.0
                    )));
                }
                if *amount > fact.quantity.expect("qualified capacity has quantity") {
                    candidate.blocked = true;
                    candidate.reasons.insert("OPTION_CAPACITY_EXCEEDED".to_owned());
                    infeasible = true;
                }
            }
        }
        match resource_profiles.get(key) {
            None => {
                candidate.unknown = true;
                candidate.reasons.insert("MISSING_RESOURCE_PROFILE".to_owned());
            }
            Some(fact) if fact.status != "COMPLETE" => {
                candidate.unknown = true;
                candidate.reasons.insert("RESOURCE_PROFILE_INCOMPLETE".to_owned());
            }
            Some(_) => {}
        }
        match domain_profiles.get(key) {
            None => {
                candidate.unknown = true;
                candidate.reasons.insert("MISSING_DOMAIN_PROFILE".to_owned());
            }
            Some(fact) if fact.status != "COMPLETE" => {
                candidate.unknown = true;
                candidate.reasons.insert("DOMAIN_PROFILE_UNKNOWN".to_owned());
            }
            Some(_) => {}
        }
        if domain_profiles.get(key).is_some_and(|fact| fact.status == "COMPLETE") {
            if let Some(domains) = memberships.get(key) {
                for domain in domains {
                    match domain_states.get(&domain.subject) {
                        None => {
                            candidate.unknown = true;
                            candidate.reasons.insert(format!("MISSING_DOMAIN_STATE:{}", domain.subject));
                        }
                        Some(state) if state.status == "UNKNOWN" => {
                            candidate.unknown = true;
                            candidate.reasons.insert(format!("DOMAIN_STATE_UNKNOWN:{}", domain.subject));
                        }
                        Some(state) if state.status == "FAILED" => {
                            candidate.blocked = true;
                            candidate.reasons.insert(format!("COMMON_CAUSE_FAILED:{}", domain.subject));
                        }
                        Some(_) => {}
                    }
                }
            }
        }
        unknown_evidence |= candidate.unknown;
        infeasible |= candidate.blocked;
    }

    let mut cohorts = BTreeMap::<CohortKey, Cohort>::new();
    let mut blocked_allocations = 0_u64;
    for allocation in &allocations {
        let candidate = candidates.get(&allocation.option_key()).expect("allocated candidate exists");
        let eligible = candidate.qualified && !candidate.blocked && !candidate.unknown;
        let key = (allocation.service.clone(), allocation.cohort.clone());
        let cohort = cohorts.entry(key).or_default();
        if cohort.unit.as_ref().is_some_and(|unit| Some(unit) != allocation.unit.as_ref()) {
            return Err(ReferenceError(format!(
                "cohort unit mismatch for {}",
                allocation.service
            )));
        }
        cohort.unit = allocation.unit.clone();
        if eligible {
            checked_add(
                &mut cohort.amount,
                allocation.quantity.expect("validated allocation amount"),
                "cohort delivery",
            )?;
        } else {
            blocked_allocations = blocked_allocations.saturating_add(1);
            cohort.unknown |= candidate.unknown;
        }
    }
    for (key, cohort) in &mut cohorts {
        match floors.get(key) {
            None => {
                cohort.unknown = true;
                unknown_evidence = true;
            }
            Some(fact) => {
                if fact.unit != cohort.unit {
                    return Err(ReferenceError(format!(
                        "service-floor unit mismatch for {}/{}",
                        key.0, key.1
                    )));
                }
                cohort.floor = fact.quantity;
                if cohort.amount < fact.quantity.expect("validated floor") && !cohort.unknown {
                    infeasible = true;
                }
            }
        }
    }
    for (key, floor) in &floors {
        if let std::collections::btree_map::Entry::Vacant(entry) = cohorts.entry(key.clone()) {
            let value = floor.quantity.expect("validated service floor");
            entry.insert(Cohort {
                amount: 0,
                floor: Some(value),
                unit: floor.unit.clone(),
                unknown: false,
            });
            infeasible |= value > 0;
        }
    }

    let allocated: BTreeSet<OptionKey> = totals.keys().cloned().collect();
    let mut resources = BTreeMap::<String, SharedResource>::new();
    for usage in &resource_uses {
        if !allocated.contains(&usage.option_key()) {
            return Err(ReferenceError(format!(
                "resource use {} has no allocation",
                usage.id
            )));
        }
        if !resource_profiles
            .get(&usage.option_key())
            .is_some_and(|fact| fact.status == "COMPLETE")
        {
            unknown_evidence = true;
        }
        let resource = resources.entry(usage.subject.clone()).or_default();
        if resource.unit.as_ref().is_some_and(|unit| Some(unit) != usage.unit.as_ref()) {
            return Err(ReferenceError(format!(
                "resource unit mismatch for {}",
                usage.subject
            )));
        }
        resource.unit = usage.unit.clone();
        if usage.status == "UNKNOWN" {
            resource.unknown = true;
            unknown_evidence = true;
        } else {
            checked_add(
                &mut resource.demand,
                usage.quantity.expect("qualified resource use has quantity"),
                "shared resource demand",
            )?;
        }
    }
    for (id, resource) in &mut resources {
        match resource_capacities.get(id) {
            None => {
                resource.unknown = true;
                unknown_evidence = true;
            }
            Some(fact) if fact.status != "QUALIFIED" => {
                resource.unknown = true;
                unknown_evidence = true;
            }
            Some(fact) => {
                if fact.unit != resource.unit {
                    return Err(ReferenceError(format!("resource capacity unit mismatch for {id}")));
                }
                resource.capacity = fact.quantity;
                if resource.demand > fact.quantity.expect("qualified capacity has quantity") {
                    infeasible = true;
                }
            }
        }
    }

    let mut domains = BTreeMap::<String, BTreeSet<String>>::new();
    for key in domain_profiles.keys() {
        if allocated.contains(key) {
            if let Some(links) = memberships.get(key) {
                for link in links {
                    domains
                        .entry(link.subject.clone())
                        .or_default()
                        .insert(key.0.clone());
                }
            }
        }
    }
    let common_domains: Vec<(String, Vec<String>)> = domains
        .into_iter()
        .filter(|(_, members)| members.len() > 1)
        .map(|(domain, members)| (domain, members.into_iter().collect()))
        .collect();

    let disposition = if unknown_evidence {
        "NOT_ESTIMABLE"
    } else if infeasible {
        "NOT_FEASIBLE"
    } else {
        "PORTFOLIO_FEASIBLE_FOR_DECLARED_SCENARIO"
    };
    let mut out = vec![
        format!("portfolio_id={}", snapshot.portfolio),
        format!("scenario_id={}", snapshot.scenario),
        format!("interval_id={}", snapshot.interval),
        format!("evidence_snapshot_id={}", snapshot.snapshot),
        format!("portfolio_disposition={disposition}"),
    ];
    for (key, candidate) in &candidates {
        let status = if candidate.unknown {
            "NOT_ESTIMABLE"
        } else if candidate.blocked {
            "BLOCKED"
        } else {
            "ELIGIBLE"
        };
        let reasons = if candidate.reasons.is_empty() {
            "NONE".to_owned()
        } else {
            candidate.reasons.iter().cloned().collect::<Vec<_>>().join(",")
        };
        out.push(format!(
            "option={}|{}|{}|{}|{}",
            key.0, key.1, status, candidate.amount, reasons
        ));
    }
    out.push(format!("service_cohorts={}", cohorts.len()));
    for (key, cohort) in &cohorts {
        let unit = cohort.unit.as_deref().unwrap_or("UNKNOWN");
        let floor = cohort
            .floor
            .map(|value| value.to_string())
            .unwrap_or_else(|| "UNKNOWN".to_owned());
        let status = if cohort.unknown {
            "NOT_ESTIMABLE"
        } else if cohort.amount >= cohort.floor.unwrap_or(0) {
            "MET"
        } else {
            "MISSED"
        };
        out.push(format!(
            "service_floor={}|{}|{}|{}|{}|{}",
            key.0, key.1, cohort.amount, floor, unit, status
        ));
    }
    out.push(format!("resource_classes={}", resources.len()));
    for (id, resource) in &resources {
        let capacity = resource
            .capacity
            .map(|value| value.to_string())
            .unwrap_or_else(|| "UNKNOWN".to_owned());
        let unit = resource.unit.as_deref().unwrap_or("UNKNOWN");
        let status = if resource.unknown {
            "NOT_ESTIMABLE"
        } else if resource.demand <= resource.capacity.unwrap_or(0) {
            "WITHIN_CAPACITY"
        } else {
            "CAPACITY_EXCEEDED"
        };
        out.push(format!(
            "resource={id}|{}|{capacity}|{unit}|{status}",
            resource.demand
        ));
    }
    out.push(format!("shared_failure_domains={}", common_domains.len()));
    for (domain, members) in &common_domains {
        out.push(format!(
            "shared_domain={domain}|{}|{}",
            members.len(),
            members.join(",")
        ));
    }
    out.push(format!("blocked_allocations={blocked_allocations}"));
    out.push("claim_scope=DECLARED_SCENARIO_AND_INTERVAL_ONLY".to_owned());
    out.push("option_independence_claim=NOT_MADE;SHARED_DOMAINS_REPORTED".to_owned());
    out.push("source_authenticity=NOT_AUTHENTICATED_BY_THIS_TOOL".to_owned());
    out.push("execution_authority=NONE".to_owned());
    out.push("operational_actuation=NOT_AUTHORISED".to_owned());
    Ok(out.join("\n") + "\n")
}

pub fn validate_and_report(source: &str) -> Result<String, ReferenceError> {
    reference_evaluate(&parse_snapshot(source)?)
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str =
        include_str!("../../fsre-portfolio-reference/fixtures/synthetic-feasible.psv");

    #[test]
    fn independent_reference_matches_declared_feasible_portfolio() {
        let report = validate_and_report(FIXTURE).expect("fixture should parse");
        assert!(report.contains("portfolio_disposition=PORTFOLIO_FEASIBLE_FOR_DECLARED_SCENARIO"));
        assert!(report.contains("service_floor=water|households|100|70|m3|MET"));
        assert!(report.contains("service_floor=water|hospital|60|50|m3|MET"));
        assert!(report.contains("resource=diesel|50|60|L|WITHIN_CAPACITY"));
        assert!(report.contains("shared_domain=power_feed_east|2|backup_pump,mobile_treatment"));
        assert!(report.contains("operational_actuation=NOT_AUTHORISED"));
    }

    #[test]
    fn common_cause_failure_blocks_dependent_option() {
        let changed = FIXTURE.replace(
            "DOMAIN_STATE|domain-state-power||||synthetic-water-shock|hour-01|power_feed_east|||CLEAR|mycelix|synthetic:domain:power",
            "DOMAIN_STATE|domain-state-power||||synthetic-water-shock|hour-01|power_feed_east|||FAILED|mycelix|synthetic:domain:power",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("domain failure is valid evidence");
        assert!(report.contains("portfolio_disposition=NOT_FEASIBLE"));
        assert!(report.contains("service_floor=water|hospital|0|50|m3|MISSED"));
    }

    #[test]
    fn shared_resource_overcommit_blocks_portfolio() {
        let changed = FIXTURE.replace(
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|60|L|QUALIFIED|operator|synthetic:stock:diesel",
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|40|L|QUALIFIED|operator|synthetic:stock:diesel",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("capacity reduction is valid evidence");
        assert!(report.contains("portfolio_disposition=NOT_FEASIBLE"));
        assert!(report.contains("resource=diesel|50|40|L|CAPACITY_EXCEEDED"));
    }

    #[test]
    fn missing_required_capacity_is_not_zero() {
        let changed = FIXTURE.replace(
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|60|L|QUALIFIED|operator|synthetic:stock:diesel\n",
            "",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("missing capacity is a valid evidence state");
        assert!(report.contains("portfolio_disposition=NOT_ESTIMABLE"));
    }

    #[test]
    fn cohort_floor_is_not_compensated_by_another_cohorts_surplus() {
        let changed = FIXTURE.replace(
            "ALLOCATION|allocation-households|backup_pump|water|households|synthetic-water-shock|hour-01||80|m3|PROPOSED|operator|synthetic:plan:pump",
            "ALLOCATION|allocation-households|backup_pump|water|households|synthetic-water-shock|hour-01||40|m3|PROPOSED|operator|synthetic:plan:pump",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("cohort shortfall is valid evidence");
        assert!(report.contains("portfolio_disposition=NOT_FEASIBLE"));
        assert!(report.contains("service_floor=water|households|60|70|m3|MISSED"));
        assert!(report.contains("service_floor=water|hospital|60|50|m3|MET"));
    }

    #[test]
    fn unknown_resource_capacity_cannot_include_a_numeric_value() {
        let changed = FIXTURE.replace(
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|60|L|QUALIFIED|operator|synthetic:stock:diesel",
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|60|L|UNKNOWN|operator|synthetic:stock:diesel",
        );
        assert_ne!(changed, FIXTURE);
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("status/measurement mismatch"));
    }

    #[test]
    fn row_order_does_not_change_reference_output() {
        let mut metadata = Vec::new();
        let mut rows = Vec::new();
        for line in FIXTURE.lines() {
            if line.starts_with('#') || line == HEADER {
                metadata.push(line);
            } else if !line.is_empty() {
                rows.push(line);
            }
        }
        rows.reverse();
        let reordered = format!("{}\n{}\n{}\n", metadata.join("\n"), HEADER, rows.join("\n"));
        assert_eq!(
            validate_and_report(FIXTURE).expect("original fixture"),
            validate_and_report(&reordered).expect("reordered fixture")
        );
    }
}
