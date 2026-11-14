#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InputError(pub String);

impl std::fmt::Display for InputError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for InputError {}

type OptionKey = (String, String);
type CohortKey = (String, String);

#[derive(Clone, Debug)]
struct Record {
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
impl Record {
    fn option_key(&self) -> OptionKey {
        (self.option.clone(), self.service.clone())
    }
}

#[derive(Clone, Debug)]
struct Input {
    portfolio_id: String,
    scenario_id: String,
    interval_id: String,
    snapshot_id: String,
    records: Vec<Record>,
}

fn valid_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 160
        && value
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'_' | b'-' | b'.' | b':' | b'/'))
}

fn required_id(value: &str, label: &str, line: usize) -> Result<(), InputError> {
    if !valid_id(value) {
        return Err(InputError(format!(
            "line {line}: missing or invalid {label}"
        )));
    }
    Ok(())
}

fn parse_quantity(value: &str, line: usize) -> Result<Option<u64>, InputError> {
    if value.is_empty() {
        return Ok(None);
    }
    value.parse::<u64>().map(Some).map_err(|error| {
        InputError(format!(
            "line {line}: invalid non-negative integer quantity: {error}"
        ))
    })
}

fn require_measure(record: &Record, line: usize) -> Result<(), InputError> {
    if record.quantity.is_none() || record.unit.is_none() {
        return Err(InputError(format!(
            "line {line}: {} requires a non-negative quantity and unit",
            record.kind
        )));
    }
    Ok(())
}

fn require_no_measure(record: &Record, line: usize) -> Result<(), InputError> {
    if record.quantity.is_some() || record.unit.is_some() {
        return Err(InputError(format!(
            "line {line}: {} must not contain a quantity or unit",
            record.kind
        )));
    }
    Ok(())
}

fn parse_record(line: &str, line_number: usize) -> Result<Record, InputError> {
    let fields: Vec<&str> = line.split('|').map(str::trim).collect();
    if fields.len() != 13 {
        return Err(InputError(format!(
            "line {line_number}: expected 13 fields, received {}",
            fields.len()
        )));
    }
    let record = Record {
        kind: fields[0].to_owned(),
        id: fields[1].to_owned(),
        option: fields[2].to_owned(),
        service: fields[3].to_owned(),
        cohort: fields[4].to_owned(),
        scenario: fields[5].to_owned(),
        interval: fields[6].to_owned(),
        subject: fields[7].to_owned(),
        quantity: parse_quantity(fields[8], line_number)?,
        unit: (!fields[9].is_empty()).then(|| fields[9].to_owned()),
        status: fields[10].to_owned(),
        owner: fields[11].to_owned(),
        evidence_ref: fields[12].to_owned(),
    };
    required_id(&record.kind, "record_type", line_number)?;
    required_id(&record.id, "record_id", line_number)?;
    required_id(&record.scenario, "scenario_id", line_number)?;
    required_id(&record.interval, "interval_id", line_number)?;
    required_id(&record.owner, "source_owner", line_number)?;
    if record.evidence_ref.is_empty() || record.evidence_ref.len() > 512 {
        return Err(InputError(format!(
            "line {line_number}: invalid evidence_ref"
        )));
    }
    Ok(record)
}

fn validate_shape(record: &Record, line: usize) -> Result<(), InputError> {
    match record.kind.as_str() {
        "OPTION_QUALIFICATION" => {
            required_id(&record.option, "option_id", line)?;
            required_id(&record.service, "service_id", line)?;
            if !record.cohort.is_empty() || !record.subject.is_empty() {
                return Err(InputError(format!(
                    "line {line}: invalid option-qualification scope"
                )));
            }
            require_no_measure(record, line)?;
            if !matches!(
                record.status.as_str(),
                "QUALIFIED" | "UNKNOWN" | "NOT_QUALIFIED" | "REVOKED" | "EXPIRED" | "OUT_OF_DOMAIN"
            ) {
                return Err(InputError(format!(
                    "line {line}: invalid qualification status"
                )));
            }
        }
        "OPTION_CAPACITY" => {
            required_id(&record.option, "option_id", line)?;
            required_id(&record.service, "service_id", line)?;
            if !record.cohort.is_empty() || !record.subject.is_empty() {
                return Err(InputError(format!(
                    "line {line}: invalid option-capacity scope"
                )));
            }
            match record.status.as_str() {
                "QUALIFIED" => require_measure(record, line)?,
                "UNKNOWN" => require_no_measure(record, line)?,
                _ => {
                    return Err(InputError(format!(
                        "line {line}: invalid option-capacity status"
                    )));
                }
            }
        }
        "ALLOCATION" => {
            required_id(&record.option, "option_id", line)?;
            required_id(&record.service, "service_id", line)?;
            required_id(&record.cohort, "cohort_id", line)?;
            if !record.subject.is_empty() {
                return Err(InputError(format!(
                    "line {line}: invalid allocation subject"
                )));
            }
            require_measure(record, line)?;
            if record.status != "PROPOSED" {
                return Err(InputError(format!(
                    "line {line}: allocation status must be PROPOSED"
                )));
            }
        }
        "SERVICE_FLOOR" => {
            if !record.option.is_empty() {
                return Err(InputError(format!(
                    "line {line}: service floor cannot name an option"
                )));
            }
            required_id(&record.service, "service_id", line)?;
            required_id(&record.cohort, "cohort_id", line)?;
            if !record.subject.is_empty() {
                return Err(InputError(format!(
                    "line {line}: invalid service-floor subject"
                )));
            }
            require_measure(record, line)?;
            if record.status != "REQUIRED" {
                return Err(InputError(format!(
                    "line {line}: service-floor status must be REQUIRED"
                )));
            }
        }
        "RESOURCE_PROFILE" | "DOMAIN_PROFILE" => {
            required_id(&record.option, "option_id", line)?;
            required_id(&record.service, "service_id", line)?;
            if !record.cohort.is_empty() || !record.subject.is_empty() {
                return Err(InputError(format!("line {line}: invalid profile scope")));
            }
            require_no_measure(record, line)?;
            if !matches!(record.status.as_str(), "COMPLETE" | "UNKNOWN") {
                return Err(InputError(format!("line {line}: invalid profile status")));
            }
        }
        "RESOURCE_USE" => {
            required_id(&record.option, "option_id", line)?;
            required_id(&record.service, "service_id", line)?;
            if !record.cohort.is_empty() {
                return Err(InputError(format!(
                    "line {line}: invalid resource-use cohort"
                )));
            }
            required_id(&record.subject, "resource_id", line)?;
            match record.status.as_str() {
                "QUALIFIED" => require_measure(record, line)?,
                "UNKNOWN" => require_no_measure(record, line)?,
                _ => {
                    return Err(InputError(format!(
                        "line {line}: invalid resource-use status"
                    )));
                }
            }
        }
        "RESOURCE_CAPACITY" => {
            if !record.option.is_empty() || !record.service.is_empty() || !record.cohort.is_empty()
            {
                return Err(InputError(format!(
                    "line {line}: invalid resource-capacity scope"
                )));
            }
            required_id(&record.subject, "resource_id", line)?;
            match record.status.as_str() {
                "QUALIFIED" => require_measure(record, line)?,
                "UNKNOWN" => require_no_measure(record, line)?,
                _ => {
                    return Err(InputError(format!(
                        "line {line}: invalid resource-capacity status"
                    )));
                }
            }
        }
        "OPTION_DOMAIN" => {
            required_id(&record.option, "option_id", line)?;
            required_id(&record.service, "service_id", line)?;
            if !record.cohort.is_empty() {
                return Err(InputError(format!(
                    "line {line}: invalid option-domain cohort"
                )));
            }
            required_id(&record.subject, "failure_domain_id", line)?;
            require_no_measure(record, line)?;
            if record.status != "MEMBER" {
                return Err(InputError(format!(
                    "line {line}: option-domain status must be MEMBER"
                )));
            }
        }
        "DOMAIN_STATE" => {
            if !record.option.is_empty() || !record.service.is_empty() || !record.cohort.is_empty()
            {
                return Err(InputError(format!(
                    "line {line}: invalid domain-state scope"
                )));
            }
            required_id(&record.subject, "failure_domain_id", line)?;
            require_no_measure(record, line)?;
            if !matches!(record.status.as_str(), "CLEAR" | "FAILED" | "UNKNOWN") {
                return Err(InputError(format!("line {line}: invalid domain state")));
            }
        }
        _ => {
            return Err(InputError(format!(
                "line {line}: unsupported record type {}",
                record.kind
            )))
        }
    }
    Ok(())
}

fn parse_input(source: &str) -> Result<Input, InputError> {
    const HEADER: &str = "record_type|record_id|option_id|service_id|cohort_id|scenario_id|interval_id|subject_id|quantity|unit|status|source_owner|evidence_ref";
    let mut metadata = BTreeMap::<String, String>::new();
    let mut records = Vec::new();
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
                    "line {line_number}: unexpected input header"
                )));
            }
            header_seen = true;
            continue;
        }
        let record = parse_record(line, line_number)?;
        validate_shape(&record, line_number)?;
        if !ids.insert(record.id.clone()) {
            return Err(InputError(format!(
                "line {line_number}: duplicate record_id {}",
                record.id
            )));
        }
        records.push(record);
    }
    if !header_seen || records.is_empty() {
        return Err(InputError("missing input header or records".to_owned()));
    }
    let mut take = |key: &str| {
        metadata
            .remove(key)
            .filter(|value| valid_id(value))
            .ok_or_else(|| InputError(format!("missing or invalid metadata key {key}")))
    };
    let input = Input {
        portfolio_id: take("portfolio_id")?,
        scenario_id: take("scenario_id")?,
        interval_id: take("interval_id")?,
        snapshot_id: take("snapshot_id")?,
        records,
    };
    if let Some(key) = metadata.keys().next() {
        return Err(InputError(format!("unknown metadata key {key}")));
    }
    for record in &input.records {
        if record.scenario != input.scenario_id || record.interval != input.interval_id {
            return Err(InputError(format!(
                "record {} crosses the frozen scenario or interval boundary",
                record.id
            )));
        }
    }
    Ok(input)
}

fn unique_insert<K: Ord, V>(
    map: &mut BTreeMap<K, V>,
    key: K,
    value: V,
    label: &str,
) -> Result<(), InputError> {
    if map.insert(key, value).is_some() {
        return Err(InputError(format!("duplicate {label}")));
    }
    Ok(())
}

fn add_checked(target: &mut u64, amount: u64, label: &str) -> Result<(), InputError> {
    *target = target
        .checked_add(amount)
        .ok_or_else(|| InputError(format!("integer overflow while summing {label}")))?;
    Ok(())
}

#[derive(Clone, Debug, Default)]
struct OptionEval {
    allocated: u64,
    unit: Option<String>,
    qualified: bool,
    blocked: bool,
    unknown: bool,
    reasons: BTreeSet<String>,
}

#[derive(Clone, Debug, Default)]
struct ServiceEval {
    allocated: u64,
    floor: Option<u64>,
    unit: Option<String>,
    unknown: bool,
}

#[derive(Clone, Debug, Default)]
struct ResourceEval {
    demand: u64,
    capacity: Option<u64>,
    unit: Option<String>,
    unknown: bool,
}

fn evaluate(input: &Input) -> Result<String, InputError> {
    let mut qualifications = BTreeMap::<OptionKey, &Record>::new();
    let mut capacities = BTreeMap::<OptionKey, &Record>::new();
    let mut resource_profiles = BTreeMap::<OptionKey, &Record>::new();
    let mut domain_profiles = BTreeMap::<OptionKey, &Record>::new();
    let mut floors = BTreeMap::<CohortKey, &Record>::new();
    let mut resource_capacities = BTreeMap::<String, &Record>::new();
    let mut domain_states = BTreeMap::<String, &Record>::new();
    let mut allocations = Vec::<&Record>::new();
    let mut resource_uses = Vec::<&Record>::new();
    let mut option_domains = BTreeMap::<OptionKey, Vec<&Record>>::new();

    let mut option_services = BTreeMap::<String, String>::new();
    for record in &input.records {
        if !record.option.is_empty() {
            if let Some(previous_service) =
                option_services.insert(record.option.clone(), record.service.clone())
            {
                if previous_service != record.service {
                    return Err(InputError(format!(
                        "option {} is assigned to multiple service lanes; v0.1 requires one service per option ID",
                        record.option
                    )));
                }
            }
        }
        match record.kind.as_str() {
            "OPTION_QUALIFICATION" => unique_insert(
                &mut qualifications,
                record.option_key(),
                record,
                "option qualification",
            )?,
            "OPTION_CAPACITY" => unique_insert(
                &mut capacities,
                record.option_key(),
                record,
                "option capacity",
            )?,
            "ALLOCATION" => allocations.push(record),
            "SERVICE_FLOOR" => unique_insert(
                &mut floors,
                (record.service.clone(), record.cohort.clone()),
                record,
                "service floor",
            )?,
            "RESOURCE_PROFILE" => unique_insert(
                &mut resource_profiles,
                record.option_key(),
                record,
                "resource profile",
            )?,
            "RESOURCE_USE" => resource_uses.push(record),
            "RESOURCE_CAPACITY" => unique_insert(
                &mut resource_capacities,
                record.subject.clone(),
                record,
                "resource capacity",
            )?,
            "DOMAIN_PROFILE" => unique_insert(
                &mut domain_profiles,
                record.option_key(),
                record,
                "domain profile",
            )?,
            "OPTION_DOMAIN" => option_domains
                .entry(record.option_key())
                .or_default()
                .push(record),
            "DOMAIN_STATE" => unique_insert(
                &mut domain_states,
                record.subject.clone(),
                record,
                "failure-domain state",
            )?,
            _ => unreachable!(),
        }
    }

    let mut allocation_totals = BTreeMap::<OptionKey, (u64, Option<String>)>::new();
    for allocation in &allocations {
        let entry = allocation_totals
            .entry(allocation.option_key())
            .or_insert((0, allocation.unit.clone()));
        if entry.1 != allocation.unit {
            return Err(InputError(format!(
                "allocation unit mismatch for option {}",
                allocation.option
            )));
        }
        add_checked(
            &mut entry.0,
            allocation.quantity.expect("validated allocation quantity"),
            "option allocation",
        )?;
    }

    let mut option_eval = BTreeMap::<OptionKey, OptionEval>::new();
    let mut not_estimable = false;
    let mut infeasible = false;

    for (key, (allocated, unit)) in &allocation_totals {
        let eval = option_eval.entry(key.clone()).or_default();
        eval.allocated = *allocated;
        eval.unit = unit.clone();

        match qualifications.get(key) {
            None => {
                eval.unknown = true;
                eval.reasons
                    .insert("MISSING_OPTION_QUALIFICATION".to_owned());
            }
            Some(record) if record.status == "QUALIFIED" => eval.qualified = true,
            Some(record) if record.status == "UNKNOWN" => {
                eval.unknown = true;
                eval.reasons
                    .insert("OPTION_QUALIFICATION_UNKNOWN".to_owned());
            }
            Some(record) => {
                eval.blocked = true;
                eval.reasons.insert(format!("OPTION_{}", record.status));
            }
        }
        match capacities.get(key) {
            None => {
                eval.unknown = true;
                eval.reasons.insert("MISSING_OPTION_CAPACITY".to_owned());
            }
            Some(record) if record.status != "QUALIFIED" => {
                eval.unknown = true;
                eval.reasons.insert("OPTION_CAPACITY_UNKNOWN".to_owned());
            }
            Some(record) => {
                if record.unit.as_deref() != unit.as_deref() {
                    return Err(InputError(format!(
                        "option capacity unit mismatch for {}",
                        key.0
                    )));
                }
                if *allocated > record.quantity.expect("validated option capacity") {
                    eval.blocked = true;
                    eval.reasons.insert("OPTION_CAPACITY_EXCEEDED".to_owned());
                }
            }
        }
        match resource_profiles.get(key) {
            None => {
                eval.unknown = true;
                eval.reasons.insert("MISSING_RESOURCE_PROFILE".to_owned());
            }
            Some(record) if record.status != "COMPLETE" => {
                eval.unknown = true;
                eval.reasons
                    .insert("RESOURCE_PROFILE_INCOMPLETE".to_owned());
            }
            Some(_) => {}
        }
        match domain_profiles.get(key) {
            None => {
                eval.unknown = true;
                eval.reasons.insert("MISSING_DOMAIN_PROFILE".to_owned());
            }
            Some(record) if record.status != "COMPLETE" => {
                eval.unknown = true;
                eval.reasons.insert("DOMAIN_PROFILE_UNKNOWN".to_owned());
            }
            Some(_) => {}
        }
        if domain_profiles
            .get(key)
            .is_some_and(|profile| profile.status == "COMPLETE")
        {
            if let Some(domains) = option_domains.get(key) {
                for domain in domains {
                    match domain_states.get(&domain.subject) {
                        None => {
                            eval.unknown = true;
                            eval.reasons
                                .insert(format!("MISSING_DOMAIN_STATE:{}", domain.subject));
                        }
                        Some(state) if state.status == "UNKNOWN" => {
                            eval.unknown = true;
                            eval.reasons
                                .insert(format!("DOMAIN_STATE_UNKNOWN:{}", domain.subject));
                        }
                        Some(state) if state.status == "FAILED" => {
                            eval.blocked = true;
                            eval.reasons
                                .insert(format!("COMMON_CAUSE_FAILED:{}", domain.subject));
                        }
                        Some(_) => {}
                    }
                }
            }
        }
        if eval.unknown {
            not_estimable = true;
        }
        if eval.blocked {
            infeasible = true;
        }
    }

    let mut service_eval = BTreeMap::<CohortKey, ServiceEval>::new();
    let mut blocked_allocations = 0_u64;
    for allocation in &allocations {
        let key = allocation.option_key();
        let option = option_eval
            .get(&key)
            .expect("allocation created option result");
        let eligible = option.qualified && !option.blocked && !option.unknown;
        let service_key = (allocation.service.clone(), allocation.cohort.clone());
        let result = service_eval.entry(service_key).or_default();
        if result
            .unit
            .as_ref()
            .is_some_and(|unit| Some(unit) != allocation.unit.as_ref())
        {
            return Err(InputError(format!(
                "service unit mismatch for {}",
                allocation.service
            )));
        }
        result.unit = allocation.unit.clone();
        if eligible {
            add_checked(
                &mut result.allocated,
                allocation.quantity.expect("validated allocation"),
                "service cohort delivery",
            )?;
        } else {
            blocked_allocations = blocked_allocations.saturating_add(1);
            if option.unknown {
                result.unknown = true;
            }
        }
    }

    for ((service, cohort), result) in &mut service_eval {
        match floors.get(&(service.clone(), cohort.clone())) {
            None => {
                result.unknown = true;
                not_estimable = true;
            }
            Some(floor) => {
                if floor.unit != result.unit {
                    return Err(InputError(format!(
                        "service floor unit mismatch for {service}/{cohort}"
                    )));
                }
                result.floor = floor.quantity;
                if result.allocated < floor.quantity.expect("validated service floor")
                    && !result.unknown
                {
                    infeasible = true;
                }
            }
        }
    }
    for ((service, cohort), floor) in &floors {
        if !service_eval.contains_key(&(service.clone(), cohort.clone())) {
            let floor_quantity = floor.quantity.expect("validated service floor");
            service_eval.insert(
                (service.clone(), cohort.clone()),
                ServiceEval {
                    allocated: 0,
                    floor: Some(floor_quantity),
                    unit: floor.unit.clone(),
                    unknown: false,
                },
            );
            if floor_quantity > 0 {
                infeasible = true;
            }
        }
    }

    let allocated_options: BTreeSet<OptionKey> = allocation_totals.keys().cloned().collect();
    let mut resource_eval = BTreeMap::<String, ResourceEval>::new();
    for usage in &resource_uses {
        let key = usage.option_key();
        if !allocated_options.contains(&key) {
            return Err(InputError(format!(
                "resource use {} has no proposed allocation",
                usage.id
            )));
        }
        if !resource_profiles
            .get(&key)
            .is_some_and(|profile| profile.status == "COMPLETE")
        {
            not_estimable = true;
        }
        let result = resource_eval.entry(usage.subject.clone()).or_default();
        if result
            .unit
            .as_ref()
            .is_some_and(|unit| Some(unit) != usage.unit.as_ref())
        {
            return Err(InputError(format!(
                "resource unit mismatch for {}",
                usage.subject
            )));
        }
        result.unit = usage.unit.clone();
        if usage.status == "UNKNOWN" {
            result.unknown = true;
            not_estimable = true;
        } else {
            add_checked(
                &mut result.demand,
                usage.quantity.expect("validated resource use"),
                "shared resource demand",
            )?;
        }
    }

    for (resource_id, result) in &mut resource_eval {
        match resource_capacities.get(resource_id) {
            None => {
                result.unknown = true;
                not_estimable = true;
            }
            Some(capacity) if capacity.status != "QUALIFIED" => {
                result.unknown = true;
                not_estimable = true;
            }
            Some(capacity) => {
                if capacity.unit != result.unit {
                    return Err(InputError(format!(
                        "resource capacity unit mismatch for {resource_id}"
                    )));
                }
                result.capacity = capacity.quantity;
                if result.demand > capacity.quantity.expect("validated resource capacity") {
                    infeasible = true;
                }
            }
        }
    }

    let mut domain_members = BTreeMap::<String, BTreeSet<String>>::new();
    for (key, profile) in &domain_profiles {
        if allocated_options.contains(key) {
            // Even partial mappings are retained as observed common-cause links.
            // An incomplete profile still forces NOT_ESTIMABLE above.
            if let Some(domains) = option_domains.get(key) {
                for domain in domains {
                    domain_members
                        .entry(domain.subject.clone())
                        .or_default()
                        .insert(key.0.clone());
                }
            }
        }
    }
    let shared_domains: Vec<(String, Vec<String>)> = domain_members
        .into_iter()
        .filter(|(_, members)| members.len() > 1)
        .map(|(domain, members)| (domain, members.into_iter().collect()))
        .collect();

    let disposition = if not_estimable {
        "NOT_ESTIMABLE"
    } else if infeasible {
        "NOT_FEASIBLE"
    } else {
        "PORTFOLIO_FEASIBLE_FOR_DECLARED_SCENARIO"
    };

    let mut lines = vec![
        "report_schema=fsre-portfolio-reference-report.v0.1".to_owned(),
        format!("portfolio_id={}", input.portfolio_id),
        format!("scenario_id={}", input.scenario_id),
        format!("interval_id={}", input.interval_id),
        format!("evidence_snapshot_id={}", input.snapshot_id),
        format!("portfolio_disposition={disposition}"),
        format!("option_service_pairs={}", option_eval.len()),
    ];
    for (key, result) in &option_eval {
        let option_disposition = if result.unknown {
            "NOT_ESTIMABLE"
        } else if result.blocked {
            "BLOCKED"
        } else {
            "ELIGIBLE"
        };
        lines.push(format!(
            "option={}|{}|{}|{}|{}",
            key.0,
            key.1,
            option_disposition,
            result.allocated,
            if result.reasons.is_empty() {
                "NONE".to_owned()
            } else {
                result.reasons.iter().cloned().collect::<Vec<_>>().join(",")
            }
        ));
    }
    lines.push(format!("service_cohorts={}", service_eval.len()));
    for ((service, cohort), result) in &service_eval {
        let unit = result.unit.as_deref().unwrap_or("UNKNOWN");
        let floor = result
            .floor
            .map(|value| value.to_string())
            .unwrap_or_else(|| "UNKNOWN".to_owned());
        let outcome = if result.unknown {
            "NOT_ESTIMABLE"
        } else if result.allocated >= result.floor.unwrap_or(0) {
            "MET"
        } else {
            "MISSED"
        };
        lines.push(format!(
            "service_floor={service}|{cohort}|{}|{floor}|{unit}|{outcome}",
            result.allocated
        ));
    }
    lines.push(format!("resource_classes={}", resource_eval.len()));
    for (resource, result) in &resource_eval {
        let capacity = result
            .capacity
            .map(|value| value.to_string())
            .unwrap_or_else(|| "UNKNOWN".to_owned());
        let outcome = if result.unknown {
            "NOT_ESTIMABLE"
        } else if result.demand <= result.capacity.unwrap_or(0) {
            "WITHIN_CAPACITY"
        } else {
            "CAPACITY_EXCEEDED"
        };
        lines.push(format!(
            "resource={resource}|{}|{capacity}|{}|{outcome}",
            result.demand,
            result.unit.as_deref().unwrap_or("UNKNOWN")
        ));
    }
    lines.push(format!("shared_failure_domains={}", shared_domains.len()));
    for (domain, members) in &shared_domains {
        lines.push(format!(
            "shared_domain={domain}|{}|{}",
            members.len(),
            members.join(",")
        ));
    }
    lines.push(format!("blocked_allocations={blocked_allocations}"));
    lines.push(format!("evidence_records={}", input.records.len()));
    let mut evidence_records: Vec<&Record> = input.records.iter().collect();
    evidence_records.sort_by(|left, right| left.id.cmp(&right.id));
    for record in evidence_records {
        lines.push(format!(
            "evidence={}|{}|{}|{}",
            record.id, record.kind, record.owner, record.evidence_ref
        ));
    }
    lines.push("claim_scope=DECLARED_SCENARIO_AND_INTERVAL_ONLY".to_owned());
    lines.push("option_independence_claim=NOT_MADE;SHARED_DOMAINS_REPORTED".to_owned());
    lines.push("source_authenticity=NOT_AUTHENTICATED_BY_THIS_TOOL".to_owned());
    lines.push("execution_authority=NONE".to_owned());
    lines.push("operational_actuation=NOT_AUTHORISED".to_owned());
    Ok(lines.join("\n") + "\n")
}

pub fn validate_and_report(source: &str) -> Result<String, InputError> {
    evaluate(&parse_input(source)?)
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../fixtures/synthetic-feasible.psv");

    #[test]
    fn feasible_plan_respects_cohort_floors_and_shared_resource_caps() {
        let report = validate_and_report(FIXTURE).expect("fixture should validate");
        assert!(report.contains("portfolio_disposition=PORTFOLIO_FEASIBLE_FOR_DECLARED_SCENARIO"));
        assert!(report.contains("service_floor=water|households|100|70|m3|MET"));
        assert!(report.contains("service_floor=water|hospital|60|50|m3|MET"));
        assert!(report.contains("resource=diesel|50|60|L|WITHIN_CAPACITY"));
        assert!(report.contains("resource=qualified_crew_hours|10|12|h|WITHIN_CAPACITY"));
        assert!(report.contains("shared_failure_domains=1"));
        assert!(report.contains("shared_domain=power_feed_east|2|backup_pump,mobile_treatment"));
        assert!(report.contains("operational_actuation=NOT_AUTHORISED"));
    }

    #[test]
    fn failed_common_cause_removes_affected_capacity() {
        let changed = FIXTURE.replace(
            "DOMAIN_STATE|domain-state-power||||synthetic-water-shock|hour-01|power_feed_east|||CLEAR|mycelix|synthetic:domain:power",
            "DOMAIN_STATE|domain-state-power||||synthetic-water-shock|hour-01|power_feed_east|||FAILED|mycelix|synthetic:domain:power",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("failed domain is a valid observation");
        assert!(report.contains("portfolio_disposition=NOT_FEASIBLE"));
        assert!(report.contains("COMMON_CAUSE_FAILED:power_feed_east"));
        assert!(report.contains("service_floor=water|hospital|0|50|m3|MISSED"));
    }

    #[test]
    fn shared_resource_overcommit_is_rejected() {
        let changed = FIXTURE.replace(
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|60|L|QUALIFIED|operator|synthetic:stock:diesel",
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|40|L|QUALIFIED|operator|synthetic:stock:diesel",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("overcommit is a valid plan to reject");
        assert!(report.contains("portfolio_disposition=NOT_FEASIBLE"));
        assert!(report.contains("resource=diesel|50|40|L|CAPACITY_EXCEEDED"));
    }

    #[test]
    fn missing_resource_capacity_abstains_instead_of_assuming_unlimited_supply() {
        let changed = FIXTURE.replace(
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|60|L|QUALIFIED|operator|synthetic:stock:diesel\n",
            "",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("unknown capacity is a valid state");
        assert!(report.contains("portfolio_disposition=NOT_ESTIMABLE"));
        assert!(report.contains("resource=diesel|50|UNKNOWN|L|NOT_ESTIMABLE"));
    }

    #[test]
    fn unknown_domain_state_abstains() {
        let changed = FIXTURE.replace(
            "DOMAIN_STATE|domain-state-power||||synthetic-water-shock|hour-01|power_feed_east|||CLEAR|mycelix|synthetic:domain:power",
            "DOMAIN_STATE|domain-state-power||||synthetic-water-shock|hour-01|power_feed_east|||UNKNOWN|mycelix|synthetic:domain:power",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("unknown domain is explicit");
        assert!(report.contains("portfolio_disposition=NOT_ESTIMABLE"));
        assert!(report.contains("DOMAIN_STATE_UNKNOWN:power_feed_east"));
    }

    #[test]
    fn cohort_floor_failure_cannot_be_hidden_by_aggregate_total() {
        let changed = FIXTURE.replace(
            "ALLOCATION|allocation-households|backup_pump|water|households|synthetic-water-shock|hour-01||80|m3|PROPOSED|operator|synthetic:plan:pump",
            "ALLOCATION|allocation-households|backup_pump|water|households|synthetic-water-shock|hour-01||40|m3|PROPOSED|operator|synthetic:plan:pump",
        );
        assert_ne!(changed, FIXTURE);
        let report =
            validate_and_report(&changed).expect("cohort under-delivery is a valid proposal");
        assert!(report.contains("portfolio_disposition=NOT_FEASIBLE"));
        assert!(report.contains("service_floor=water|households|60|70|m3|MISSED"));
        assert!(report.contains("service_floor=water|hospital|60|50|m3|MET"));
    }

    #[test]
    fn missing_resource_profile_is_not_assumed_zero() {
        let changed = FIXTURE.replace(
            "RESOURCE_PROFILE|resource-profile-mobile|mobile_treatment|water||synthetic-water-shock|hour-01||||COMPLETE|operator|synthetic:resource-profile:mobile_treatment\n",
            "",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("missing profile is explicit");
        assert!(report.contains("portfolio_disposition=NOT_ESTIMABLE"));
        assert!(report.contains("MISSING_RESOURCE_PROFILE"));
    }

    #[test]
    fn unit_mismatch_is_rejected_not_converted_implicitly() {
        let changed = FIXTURE.replace(
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|60|L|QUALIFIED|operator|synthetic:stock:diesel",
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|60|m3|QUALIFIED|operator|synthetic:stock:diesel",
        );
        assert_ne!(changed, FIXTURE);
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("resource capacity unit mismatch"));
    }

    #[test]
    fn integer_overflow_is_rejected() {
        let changed = FIXTURE.replace(
            "ALLOCATION|allocation-households|backup_pump|water|households|synthetic-water-shock|hour-01||80|m3|PROPOSED|operator|synthetic:plan:pump",
            "ALLOCATION|allocation-households|backup_pump|water|households|synthetic-water-shock|hour-01||18446744073709551615|m3|PROPOSED|operator|synthetic:plan:pump",
        );
        let changed = format!(
            "{changed}ALLOCATION|allocation-overflow|backup_pump|water|households|synthetic-water-shock|hour-01||1|m3|PROPOSED|operator|synthetic:plan:overflow\n"
        );
        assert_ne!(changed, FIXTURE);
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("integer overflow"));
    }

    #[test]
    fn scenario_and_interval_mismatch_is_rejected() {
        let changed = FIXTURE.replace(
            "ALLOCATION|allocation-households|backup_pump|water|households|synthetic-water-shock|hour-01",
            "ALLOCATION|allocation-households|backup_pump|water|households|other-scenario|hour-01",
        );
        assert_ne!(changed, FIXTURE);
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("frozen scenario or interval"));
    }

    #[test]
    fn one_option_id_cannot_be_double_counted_across_service_lanes() {
        let changed = FIXTURE.replace(
            "OPTION_QUALIFICATION|qual-tanker|tanker_delivery|water",
            "OPTION_QUALIFICATION|qual-tanker|tanker_delivery|wastewater",
        );
        assert_ne!(changed, FIXTURE);
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("multiple service lanes"));
    }

    #[test]
    fn report_preserves_owner_and_evidence_references() {
        let report = validate_and_report(FIXTURE).expect("fixture should validate");
        assert!(report.contains("evidence_records=28"));
        assert!(report.contains(
            "evidence=qual-pump|OPTION_QUALIFICATION|operator|synthetic:qualification:pump"
        ));
        assert!(report.contains(
            "evidence=resource-cap-diesel|RESOURCE_CAPACITY|operator|synthetic:stock:diesel"
        ));
    }

    #[test]
    fn generated_capacity_floor_and_allocation_sweeps_check_exact_boundaries() {
        let resource_old = "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|60|L|QUALIFIED|operator|synthetic:stock:diesel";
        let option_old = "OPTION_CAPACITY|cap-pump|backup_pump|water||synthetic-water-shock|hour-01||80|m3|QUALIFIED|symtropy|synthetic:capacity:pump";
        let floor_old = "SERVICE_FLOOR|floor-households||water|households|synthetic-water-shock|hour-01||70|m3|REQUIRED|service-owner|synthetic:floor:households";

        for capacity in 0..=70_u64 {
            let replacement = format!(
                "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|{capacity}|L|QUALIFIED|operator|synthetic:stock:diesel"
            );
            let changed = FIXTURE.replace(resource_old, &replacement);
            assert_eq!(changed == FIXTURE, capacity == 60);
            let report = validate_and_report(&changed).expect("resource capacity sweep input");
            let expected = if capacity < 50 {
                "portfolio_disposition=NOT_FEASIBLE"
            } else {
                "portfolio_disposition=PORTFOLIO_FEASIBLE_FOR_DECLARED_SCENARIO"
            };
            assert!(
                report.contains(expected),
                "resource capacity boundary {capacity}"
            );
        }

        for capacity in 0..=100_u64 {
            let replacement = format!(
                "OPTION_CAPACITY|cap-pump|backup_pump|water||synthetic-water-shock|hour-01||{capacity}|m3|QUALIFIED|symtropy|synthetic:capacity:pump"
            );
            let changed = FIXTURE.replace(option_old, &replacement);
            assert_eq!(changed == FIXTURE, capacity == 80);
            let report = validate_and_report(&changed).expect("option capacity sweep input");
            let expected = if capacity < 80 {
                "portfolio_disposition=NOT_FEASIBLE"
            } else {
                "portfolio_disposition=PORTFOLIO_FEASIBLE_FOR_DECLARED_SCENARIO"
            };
            assert!(
                report.contains(expected),
                "option capacity boundary {capacity}"
            );
        }

        for floor in 0..=110_u64 {
            let replacement = format!(
                "SERVICE_FLOOR|floor-households||water|households|synthetic-water-shock|hour-01||{floor}|m3|REQUIRED|service-owner|synthetic:floor:households"
            );
            let changed = FIXTURE.replace(floor_old, &replacement);
            assert_eq!(changed == FIXTURE, floor == 70);
            let report = validate_and_report(&changed).expect("cohort floor sweep input");
            let expected = if floor > 100 {
                "portfolio_disposition=NOT_FEASIBLE"
            } else {
                "portfolio_disposition=PORTFOLIO_FEASIBLE_FOR_DECLARED_SCENARIO"
            };
            assert!(report.contains(expected), "service floor boundary {floor}");
        }
    }

    #[test]
    fn unknown_capacity_cannot_smuggle_a_numeric_bound() {
        let changed = FIXTURE.replace(
            "OPTION_CAPACITY|cap-pump|backup_pump|water||synthetic-water-shock|hour-01||80|m3|QUALIFIED|symtropy|synthetic:capacity:pump",
            "OPTION_CAPACITY|cap-pump|backup_pump|water||synthetic-water-shock|hour-01||80|m3|UNKNOWN|symtropy|synthetic:capacity:pump",
        );
        assert_ne!(changed, FIXTURE);
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("must not contain a quantity or unit"));
    }

    #[test]
    fn unknown_resource_use_cannot_smuggle_a_numeric_demand() {
        let changed = FIXTURE.replace(
            "RESOURCE_USE|resource-use-pump|backup_pump|water||synthetic-water-shock|hour-01|qualified_crew_hours|6|h|QUALIFIED|operator|synthetic:resource:pump-crew",
            "RESOURCE_USE|resource-use-pump|backup_pump|water||synthetic-water-shock|hour-01|qualified_crew_hours|6|h|UNKNOWN|operator|synthetic:resource:pump-crew",
        );
        assert_ne!(changed, FIXTURE);
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("must not contain a quantity or unit"));
    }

    #[test]
    fn unknown_resource_capacity_cannot_smuggle_a_numeric_bound() {
        let changed = FIXTURE.replace(
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|60|L|QUALIFIED|operator|synthetic:stock:diesel",
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|60|L|UNKNOWN|operator|synthetic:stock:diesel",
        );
        assert_ne!(changed, FIXTURE);
        assert!(validate_and_report(&changed)
            .unwrap_err()
            .0
            .contains("must not contain a quantity or unit"));
    }

    #[test]
    fn unknown_resource_capacity_without_numeric_bounds_abstains() {
        let changed = FIXTURE.replace(
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|60|L|QUALIFIED|operator|synthetic:stock:diesel",
            "RESOURCE_CAPACITY|resource-cap-diesel||||synthetic-water-shock|hour-01|diesel|||UNKNOWN|operator|synthetic:stock:diesel",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("unknown resource capacity is explicit");
        assert!(report.contains("portfolio_disposition=NOT_ESTIMABLE"));
        assert!(report.contains("resource=diesel|50|UNKNOWN|L|NOT_ESTIMABLE"));
    }

    #[test]
    fn unknown_resource_use_without_numeric_demand_abstains() {
        let changed = FIXTURE.replace(
            "RESOURCE_USE|resource-use-pump|backup_pump|water||synthetic-water-shock|hour-01|qualified_crew_hours|6|h|QUALIFIED|operator|synthetic:resource:pump-crew",
            "RESOURCE_USE|resource-use-pump|backup_pump|water||synthetic-water-shock|hour-01|qualified_crew_hours|||UNKNOWN|operator|synthetic:resource:pump-crew",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("unknown resource demand is explicit");
        assert!(report.contains("portfolio_disposition=NOT_ESTIMABLE"));
        assert!(report.contains("resource=qualified_crew_hours|4|12|h|NOT_ESTIMABLE"));
    }

    #[test]
    fn missing_service_floor_abstains() {
        let changed = FIXTURE.replace(
            "SERVICE_FLOOR|floor-hospital||water|hospital|synthetic-water-shock|hour-01||50|m3|REQUIRED|service-owner|synthetic:floor:hospital\n",
            "",
        );
        assert_ne!(changed, FIXTURE);
        let report = validate_and_report(&changed).expect("missing floor is explicit");
        assert!(report.contains("portfolio_disposition=NOT_ESTIMABLE"));
        assert!(report.contains("service_floor=water|hospital|60|UNKNOWN|m3|NOT_ESTIMABLE"));
    }
}
