use std::collections::{BTreeMap, BTreeSet};

const EPSILON: f64 = 1e-9;
#[cfg(test)]
const FIXTURE: &str = include_str!("../../../docs/research/maritime-productive-capability/fixtures/maritime-capability-tiny-v0.tsv");

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct ResourceKey {
    resource_id: String,
    site_id: String,
    period_id: String,
}

impl ResourceKey {
    fn render(&self) -> String {
        format!("{}|{}|{}", self.resource_id, self.site_id, self.period_id)
    }
}

#[derive(Clone, Debug)]
struct Resource {
    capacity: f64,
    unit: String,
}

#[derive(Clone, Debug)]
struct WorkPackage {
    kind: String,
    required_period: String,
}

#[derive(Clone, Debug)]
struct ResourceUse {
    key: ResourceKey,
    quantity: f64,
    unit: String,
}

#[derive(Clone, Debug)]
struct SourcingOption {
    work_package_id: String,
    option_id: String,
    source_region: String,
    qualified: bool,
    cost_units: f64,
    lead_days: f64,
    embodied_co2_units: f64,
    regional_value_units: f64,
    reservations: Vec<ResourceUse>,
}

#[derive(Clone, Debug)]
struct Instance {
    id: String,
    version: String,
    resources: BTreeMap<ResourceKey, Resource>,
    work_packages: BTreeMap<String, WorkPackage>,
    options: BTreeMap<String, Vec<SourcingOption>>,
}

#[derive(Clone, Debug, PartialEq)]
struct Metrics {
    cost_units: f64,
    max_lead_days: f64,
    embodied_co2_units: f64,
    regional_value_units: f64,
}

#[derive(Clone, Debug)]
struct Candidate {
    plan_id: String,
    selected_options: Vec<String>,
    feasible: bool,
    blocking_reasons: Vec<String>,
    metrics: Metrics,
}

#[derive(Clone, Debug)]
struct Audit {
    instance_id: String,
    version: String,
    resources: BTreeMap<ResourceKey, Resource>,
    work_packages: BTreeMap<String, WorkPackage>,
    candidates: Vec<Candidate>,
    feasible_count: usize,
    infeasible_count: usize,
    pareto_plan_ids: Vec<String>,
}

fn parse_number(value: &str, label: &str, line_number: usize) -> Result<f64, String> {
    let number = value
        .parse::<f64>()
        .map_err(|error| format!("line {line_number}: invalid {label} '{value}': {error}"))?;
    if !number.is_finite() || number < 0.0 {
        return Err(format!(
            "line {line_number}: {label} must be finite and non-negative"
        ));
    }
    Ok(number)
}

fn parse_resource_key(parts: &[&str], line_number: usize) -> Result<ResourceKey, String> {
    if parts.len() != 3 || parts.iter().any(|part| part.trim().is_empty()) {
        return Err(format!(
            "line {line_number}: resource key must be resource_id:site_id:period_id"
        ));
    }
    Ok(ResourceKey {
        resource_id: parts[0].to_owned(),
        site_id: parts[1].to_owned(),
        period_id: parts[2].to_owned(),
    })
}

fn parse_fixture(input: &str) -> Result<Instance, String> {
    let mut identity: Option<(String, String)> = None;
    let mut resources = BTreeMap::new();
    let mut work_packages = BTreeMap::new();
    let mut options: BTreeMap<String, Vec<SourcingOption>> = BTreeMap::new();

    for (index, raw_line) in input.lines().enumerate() {
        let line_number = index + 1;
        let line = raw_line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let fields: Vec<&str> = line.split('\t').collect();
        match fields.first().copied().unwrap_or_default() {
            "INSTANCE" if fields.len() == 3 => {
                if identity.is_some() {
                    return Err(format!("line {line_number}: duplicate INSTANCE"));
                }
                if fields[1].is_empty() || fields[2].is_empty() {
                    return Err(format!(
                        "line {line_number}: empty instance identity or version"
                    ));
                }
                identity = Some((fields[1].to_owned(), fields[2].to_owned()));
            }
            "RESOURCE" if fields.len() == 6 => {
                let key = ResourceKey {
                    resource_id: fields[1].to_owned(),
                    site_id: fields[2].to_owned(),
                    period_id: fields[3].to_owned(),
                };
                if key.resource_id.is_empty()
                    || key.site_id.is_empty()
                    || key.period_id.is_empty()
                    || fields[5].is_empty()
                {
                    return Err(format!(
                        "line {line_number}: resource identity and unit must be non-empty"
                    ));
                }
                let resource = Resource {
                    capacity: parse_number(fields[4], "resource capacity", line_number)?,
                    unit: fields[5].to_owned(),
                };
                if resources.insert(key.clone(), resource).is_some() {
                    return Err(format!(
                        "line {line_number}: duplicate resource {}",
                        key.render()
                    ));
                }
            }
            "WORK_PACKAGE" if fields.len() == 4 => {
                if fields[1].is_empty() || fields[2].is_empty() || fields[3].is_empty() {
                    return Err(format!(
                        "line {line_number}: work-package fields must be non-empty"
                    ));
                }
                let package = WorkPackage {
                    kind: fields[2].to_owned(),
                    required_period: fields[3].to_owned(),
                };
                if work_packages.insert(fields[1].to_owned(), package).is_some() {
                    return Err(format!(
                        "line {line_number}: duplicate work package {}",
                        fields[1]
                    ));
                }
            }
            "OPTION" if fields.len() == 10 => {
                if fields[1].is_empty() || fields[2].is_empty() || fields[3].is_empty() {
                    return Err(format!(
                        "line {line_number}: option identity fields must be non-empty"
                    ));
                }
                let qualified = match fields[4] {
                    "true" => true,
                    "false" => false,
                    other => {
                        return Err(format!(
                            "line {line_number}: invalid qualification flag '{other}'"
                        ));
                    }
                };
                let reservations = if fields[9] == "-" {
                    Vec::new()
                } else {
                    fields[9]
                        .split(';')
                        .map(|reservation| {
                            let parts: Vec<&str> = reservation.split(':').collect();
                            if parts.len() != 5 {
                                return Err(format!(
                                    "line {line_number}: reservation must be resource_id:site_id:period_id:quantity:unit"
                                ));
                            }
                            let key = parse_resource_key(&parts[..3], line_number)?;
                            let quantity =
                                parse_number(parts[3], "reservation quantity", line_number)?;
                            if parts[4].is_empty() {
                                return Err(format!(
                                    "line {line_number}: reservation unit must be non-empty"
                                ));
                            }
                            Ok(ResourceUse {
                                key,
                                quantity,
                                unit: parts[4].to_owned(),
                            })
                        })
                        .collect::<Result<Vec<_>, String>>()?
                };
                let option = SourcingOption {
                    work_package_id: fields[1].to_owned(),
                    option_id: fields[2].to_owned(),
                    source_region: fields[3].to_owned(),
                    qualified,
                    cost_units: parse_number(fields[5], "cost", line_number)?,
                    lead_days: parse_number(fields[6], "lead time", line_number)?,
                    embodied_co2_units: parse_number(fields[7], "embodied emissions", line_number)?,
                    regional_value_units: parse_number(fields[8], "regional value", line_number)?,
                    reservations,
                };
                options.entry(fields[1].to_owned()).or_default().push(option);
            }
            kind => {
                return Err(format!(
                    "line {line_number}: unknown record or wrong field count for '{kind}'"
                ));
            }
        }
    }

    let (id, version) = identity.ok_or_else(|| "missing INSTANCE record".to_owned())?;
    if resources.is_empty() || work_packages.is_empty() {
        return Err("fixture requires at least one resource and work package".to_owned());
    }
    for package_id in work_packages.keys() {
        let package_options = options
            .get_mut(package_id)
            .ok_or_else(|| format!("work package {package_id} has no sourcing options"))?;
        package_options.sort_by(|left, right| left.option_id.cmp(&right.option_id));
        let mut seen = BTreeSet::new();
        for option in package_options.iter() {
            if !seen.insert(option.option_id.clone()) {
                return Err(format!(
                    "duplicate option {} for {package_id}",
                    option.option_id
                ));
            }
        }
    }
    for package_id in options.keys() {
        if !work_packages.contains_key(package_id) {
            return Err(format!(
                "option references unknown work package {package_id}"
            ));
        }
    }

    Ok(Instance {
        id,
        version,
        resources,
        work_packages,
        options,
    })
}

fn evaluate_selection(instance: &Instance, selected: &[&SourcingOption]) -> Candidate {
    let mut reservations: BTreeMap<ResourceKey, f64> = BTreeMap::new();
    let mut reasons = BTreeSet::new();
    let mut metrics = Metrics {
        cost_units: 0.0,
        max_lead_days: 0.0,
        embodied_co2_units: 0.0,
        regional_value_units: 0.0,
    };
    let mut selected_options = Vec::new();

    for option in selected {
        selected_options.push(format!(
            "{}={}@{}",
            option.work_package_id, option.option_id, option.source_region
        ));
        metrics.cost_units += option.cost_units;
        metrics.max_lead_days = metrics.max_lead_days.max(option.lead_days);
        metrics.embodied_co2_units += option.embodied_co2_units;
        metrics.regional_value_units += option.regional_value_units;

        if !option.qualified {
            reasons.insert(format!("UNQUALIFIED_OPTION:{}", option.option_id));
        }
        let package = instance
            .work_packages
            .get(&option.work_package_id)
            .expect("parser guarantees a declared work package");
        for usage in &option.reservations {
            if usage.key.period_id != package.required_period {
                reasons.insert(format!(
                    "RESERVATION_PERIOD_MISMATCH:{}:required={}:observed={}",
                    usage.key.render(),
                    package.required_period,
                    usage.key.period_id
                ));
            }
            let Some(resource) = instance.resources.get(&usage.key) else {
                reasons.insert(format!("MISSING_RESOURCE:{}", usage.key.render()));
                continue;
            };
            if resource.unit != usage.unit {
                reasons.insert(format!(
                    "RESOURCE_UNIT_MISMATCH:{}:expected={}:observed={}",
                    usage.key.render(),
                    resource.unit,
                    usage.unit
                ));
                continue;
            }
            *reservations.entry(usage.key.clone()).or_default() += usage.quantity;
        }
    }

    for (key, quantity) in reservations {
        if let Some(resource) = instance.resources.get(&key) {
            if quantity > resource.capacity + EPSILON {
                reasons.insert(format!(
                    "CAPACITY_EXCEEDED:{}:used={:.3}:capacity={:.3}:unit={}",
                    key.render(),
                    quantity,
                    resource.capacity,
                    resource.unit
                ));
            }
        }
    }

    selected_options.sort();
    let plan_id = selected_options.join("+");
    Candidate {
        plan_id,
        selected_options,
        feasible: reasons.is_empty(),
        blocking_reasons: reasons.into_iter().collect(),
        metrics,
    }
}

fn enumerate_recursive<'a>(
    instance: &'a Instance,
    package_ids: &[String],
    depth: usize,
    selected: &mut Vec<&'a SourcingOption>,
    output: &mut Vec<Candidate>,
) {
    if depth == package_ids.len() {
        output.push(evaluate_selection(instance, selected));
        return;
    }
    let package_id = &package_ids[depth];
    if let Some(package_options) = instance.options.get(package_id) {
        for option in package_options {
            selected.push(option);
            enumerate_recursive(instance, package_ids, depth + 1, selected, output);
            selected.pop();
        }
    }
}

fn dominates(left: &Candidate, right: &Candidate) -> bool {
    let no_worse = left.metrics.cost_units <= right.metrics.cost_units + EPSILON
        && left.metrics.max_lead_days <= right.metrics.max_lead_days + EPSILON
        && left.metrics.embodied_co2_units <= right.metrics.embodied_co2_units + EPSILON
        && left.metrics.regional_value_units + EPSILON >= right.metrics.regional_value_units;
    let strictly_better = left.metrics.cost_units + EPSILON < right.metrics.cost_units
        || left.metrics.max_lead_days + EPSILON < right.metrics.max_lead_days
        || left.metrics.embodied_co2_units + EPSILON < right.metrics.embodied_co2_units
        || left.metrics.regional_value_units > right.metrics.regional_value_units + EPSILON;
    no_worse && strictly_better
}

fn evaluate(instance: &Instance) -> Audit {
    let package_ids: Vec<String> = instance.work_packages.keys().cloned().collect();
    let mut candidates = Vec::new();
    enumerate_recursive(
        instance,
        &package_ids,
        0,
        &mut Vec::new(),
        &mut candidates,
    );
    candidates.sort_by(|left, right| left.plan_id.cmp(&right.plan_id));

    let feasible: Vec<&Candidate> = candidates
        .iter()
        .filter(|candidate| candidate.feasible)
        .collect();
    let pareto_plan_ids: Vec<String> = feasible
        .iter()
        .filter(|candidate| {
            !feasible
                .iter()
                .any(|other| other.plan_id != candidate.plan_id && dominates(other, candidate))
        })
        .map(|candidate| candidate.plan_id.clone())
        .collect();

    let feasible_count = feasible.len();
    let infeasible_count = candidates.len() - feasible_count;
    Audit {
        instance_id: instance.id.clone(),
        version: instance.version.clone(),
        resources: instance.resources.clone(),
        work_packages: instance.work_packages.clone(),
        candidates,
        feasible_count,
        infeasible_count,
        pareto_plan_ids,
    }
}

fn render_receipt(audit: &Audit) -> String {
    let pareto: BTreeSet<&str> = audit.pareto_plan_ids.iter().map(String::as_str).collect();
    let mut lines = vec![
        "MPCB_RECEIPT\t0.1".to_owned(),
        format!("INSTANCE\t{}\t{}", audit.instance_id, audit.version),
    ];
    for (package_id, package) in &audit.work_packages {
        lines.push(format!(
            "WORK_PACKAGE\t{}\t{}\t{}",
            package_id, package.kind, package.required_period
        ));
    }
    for (key, resource) in &audit.resources {
        lines.push(format!(
            "RESOURCE\t{}\t{}\t{}\t{:.3}\t{}",
            key.resource_id, key.site_id, key.period_id, resource.capacity, resource.unit
        ));
    }
    for candidate in &audit.candidates {
        let status = if candidate.feasible { "FEASIBLE" } else { "INFEASIBLE" };
        let frontier = if pareto.contains(candidate.plan_id.as_str()) {
            "PARETO"
        } else {
            "DOMINATED_OR_BLOCKED"
        };
        lines.push(format!(
            "PLAN\t{}\t{}\t{}\t{:.3}\t{:.3}\t{:.3}\t{:.3}",
            candidate.plan_id,
            status,
            frontier,
            candidate.metrics.cost_units,
            candidate.metrics.max_lead_days,
            candidate.metrics.embodied_co2_units,
            candidate.metrics.regional_value_units,
        ));
        for option in &candidate.selected_options {
            lines.push(format!("SELECTED\t{}\t{}", candidate.plan_id, option));
        }
        for reason in &candidate.blocking_reasons {
            lines.push(format!("BLOCK\t{}\t{}", candidate.plan_id, reason));
        }
    }
    lines.push(format!(
        "SUMMARY\t{}\t{}\t{}",
        audit.candidates.len(),
        audit.feasible_count,
        audit.infeasible_count
    ));
    lines.push(format!("PARETO\t{}", audit.pareto_plan_ids.join(";")));
    lines.join("\n") + "\n"
}

fn audit_fixture(input: &str) -> Result<String, String> {
    let instance = parse_fixture(input)?;
    Ok(render_receipt(&evaluate(&instance)))
}

pub fn run_audit(input: &str) -> Result<String, String> {
    audit_fixture(input)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tiny_fixture_has_expected_feasible_and_pareto_sets() {
        let instance = parse_fixture(FIXTURE).expect("fixture parses");
        let audit = evaluate(&instance);
        assert_eq!(audit.candidates.len(), 4);
        assert_eq!(audit.feasible_count, 3);
        assert_eq!(audit.infeasible_count, 1);
        assert_eq!(audit.pareto_plan_ids.len(), 3);

        let blocked = audit.candidates.iter().find(|candidate| !candidate.feasible).unwrap();
        assert!(blocked.plan_id.contains("A-LOCAL") && blocked.plan_id.contains("B-LOCAL"));
        assert!(blocked.blocking_reasons.iter().any(|reason| reason.starts_with("CAPACITY_EXCEEDED:REGIONAL-PLATE")));
        assert!(blocked.blocking_reasons.iter().any(|reason| reason.starts_with("CAPACITY_EXCEEDED:REGIONAL-WELD")));
        assert!(!blocked.blocking_reasons.iter().any(|reason| reason.starts_with("CAPACITY_EXCEEDED:REGIONAL-NDT")));
    }

    #[test]
    fn each_local_option_is_feasible_alone() {
        let audit = evaluate(&parse_fixture(FIXTURE).expect("fixture parses"));
        let local_a = audit.candidates.iter().find(|candidate| {
            candidate.plan_id.contains("WP-A=A-LOCAL") && candidate.plan_id.contains("WP-B=B-IMPORT")
        }).unwrap();
        let local_b = audit.candidates.iter().find(|candidate| {
            candidate.plan_id.contains("WP-A=A-IMPORT") && candidate.plan_id.contains("WP-B=B-LOCAL")
        }).unwrap();
        assert!(local_a.feasible);
        assert!(local_b.feasible);
    }

    #[test]
    fn resource_unit_mismatch_fails_closed() {
        let altered = FIXTURE.replace(
            "REGIONAL-PLATE:YARD-A:P1:2:t",
            "REGIONAL-PLATE:YARD-A:P1:2:kg",
        );
        let audit = evaluate(&parse_fixture(&altered).expect("mutated fixture parses"));
        assert_eq!(audit.feasible_count, 1);
        assert!(audit.candidates.iter().any(|candidate| {
            candidate.blocking_reasons.iter().any(|reason| reason.contains("RESOURCE_UNIT_MISMATCH"))
        }));
    }

    #[test]
    fn reservation_period_mismatch_fails_closed() {
        let altered = FIXTURE.replace(
            "REGIONAL-PLATE:YARD-A:P1:2:t",
            "REGIONAL-PLATE:YARD-A:P2:2:t",
        );
        let audit = evaluate(&parse_fixture(&altered).expect("mutated fixture parses"));
        assert_eq!(audit.feasible_count, 1);
        assert!(audit.candidates.iter().any(|candidate| {
            candidate.blocking_reasons.iter().any(|reason| reason.contains("RESERVATION_PERIOD_MISMATCH"))
        }));
    }

    #[test]
    fn canonical_receipt_is_input_order_invariant() {
        let reversed = FIXTURE.lines().rev().collect::<Vec<_>>().join("\n");
        assert_eq!(audit_fixture(FIXTURE).unwrap(), audit_fixture(&reversed).unwrap());
    }
}
