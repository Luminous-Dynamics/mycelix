#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

const HEADER: &str = "term_id|status|lower|upper|source_owner|source_ref|source_schema_version|admissibility|time_basis_id|geography_id|period_start_seconds|period_end_seconds|correlation_group|method_id|reason";
const REQUIRED_TERMS: [&str; 6] = [
    "generation",
    "other_inflows",
    "treated",
    "bypass",
    "reuse",
    "storage_change",
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TermStatus {
    ObservedInterval,
    EstimatedInterval,
    Unknown,
    NotApplicable,
}

impl TermStatus {
    fn parse(value: &str, line: usize) -> Result<Self, String> {
        match value {
            "OBSERVED_INTERVAL" => Ok(Self::ObservedInterval),
            "ESTIMATED_INTERVAL" => Ok(Self::EstimatedInterval),
            "UNKNOWN" => Ok(Self::Unknown),
            "NOT_APPLICABLE" => Ok(Self::NotApplicable),
            _ => Err(format!("line {line}: unsupported term status {value}")),
        }
    }

    fn as_str(self) -> &'static str {
        match self {
            Self::ObservedInterval => "OBSERVED_INTERVAL",
            Self::EstimatedInterval => "ESTIMATED_INTERVAL",
            Self::Unknown => "UNKNOWN",
            Self::NotApplicable => "NOT_APPLICABLE",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Metadata {
    balance_id: String,
    service_id: String,
    scenario_id: String,
    scope_ref: String,
    time_basis_id: String,
    geography_id: String,
    period_start: u64,
    period_end: u64,
    unit: String,
    boundary_description: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct FlowTerm {
    term_id: String,
    status: TermStatus,
    lower: Option<i64>,
    upper: Option<i64>,
    source_owner: String,
    source_ref: String,
    source_schema: String,
    admissibility: String,
    time_basis_id: String,
    geography_id: String,
    period_start: u64,
    period_end: u64,
    correlation_group: String,
    method_id: String,
    reason: String,
}

fn take_required(metadata: &mut BTreeMap<String, String>, key: &str) -> Result<String, String> {
    metadata
        .remove(key)
        .ok_or_else(|| format!("missing metadata key: {key}"))
}

fn parse_u64(value: &str, label: &str) -> Result<u64, String> {
    value
        .parse::<u64>()
        .map_err(|error| format!("invalid {label}: {error}"))
}

fn parse_i64(value: &str, label: &str, line: usize) -> Result<Option<i64>, String> {
    if value.is_empty() {
        return Ok(None);
    }
    value
        .parse::<i64>()
        .map(Some)
        .map_err(|error| format!("line {line}: invalid {label}: {error}"))
}

fn parse_metadata(metadata: &mut BTreeMap<String, String>) -> Result<Metadata, String> {
    let parsed = Metadata {
        balance_id: take_required(metadata, "balance_id")?,
        service_id: take_required(metadata, "service_id")?,
        scenario_id: take_required(metadata, "scenario_id")?,
        scope_ref: take_required(metadata, "scope_ref")?,
        time_basis_id: take_required(metadata, "time_basis_id")?,
        geography_id: take_required(metadata, "geography_id")?,
        period_start: parse_u64(&take_required(metadata, "period_start_seconds")?, "period_start_seconds")?,
        period_end: parse_u64(&take_required(metadata, "period_end_seconds")?, "period_end_seconds")?,
        unit: take_required(metadata, "unit")?,
        boundary_description: take_required(metadata, "boundary_description")?,
    };
    if let Some(key) = metadata.keys().next() {
        return Err(format!("unknown metadata key: {key}"));
    }
    if parsed.balance_id.is_empty()
        || parsed.service_id.is_empty()
        || parsed.scenario_id.is_empty()
        || parsed.scope_ref.is_empty()
        || parsed.time_basis_id.is_empty()
        || parsed.geography_id.is_empty()
        || parsed.unit != "m3"
        || parsed.boundary_description.is_empty()
        || parsed.period_start >= parsed.period_end
    {
        return Err("invalid balance identity, scope, unit or observation period".to_owned());
    }
    Ok(parsed)
}

fn parse_term(line: &str, line_number: usize) -> Result<FlowTerm, String> {
    let columns: Vec<&str> = line.split('|').map(str::trim).collect();
    if columns.len() != 15
        || columns
            .iter()
            .enumerate()
            .any(|(index, value)| value.is_empty() && !matches!(index, 2 | 3 | 12))
    {
        return Err(format!(
            "line {line_number}: expected 15 fields; lower, upper and correlation_group may be empty"
        ));
    }

    let status = TermStatus::parse(columns[1], line_number)?;
    let lower = parse_i64(columns[2], "lower", line_number)?;
    let upper = parse_i64(columns[3], "upper", line_number)?;
    let period_start = parse_u64(columns[10], "period_start_seconds")?;
    let period_end = parse_u64(columns[11], "period_end_seconds")?;

    match status {
        TermStatus::ObservedInterval | TermStatus::EstimatedInterval => {
            let (Some(lower), Some(upper)) = (lower, upper) else {
                return Err(format!("line {line_number}: known interval requires both lower and upper"));
            };
            if lower > upper {
                return Err(format!("line {line_number}: lower bound exceeds upper bound"));
            }
            if columns[0] != "storage_change" && lower < 0 {
                return Err(format!("line {line_number}: physical flow interval cannot be negative"));
            }
        }
        TermStatus::Unknown => {
            if lower.is_some() || upper.is_some() || columns[14].is_empty() {
                return Err(format!("line {line_number}: unknown term requires empty bounds and a reason"));
            }
        }
        TermStatus::NotApplicable => {
            if lower != Some(0) || upper != Some(0) || columns[14].is_empty() {
                return Err(format!("line {line_number}: not-applicable term requires explicit [0,0] and a reason"));
            }
        }
    }

    if !REQUIRED_TERMS.contains(&columns[0]) {
        return Err(format!("line {line_number}: unknown flow term {}", columns[0]));
    }
    if period_start >= period_end {
        return Err(format!("line {line_number}: invalid term observation period"));
    }

    Ok(FlowTerm {
        term_id: columns[0].to_owned(),
        status,
        lower,
        upper,
        source_owner: columns[4].to_owned(),
        source_ref: columns[5].to_owned(),
        source_schema: columns[6].to_owned(),
        admissibility: columns[7].to_owned(),
        time_basis_id: columns[8].to_owned(),
        geography_id: columns[9].to_owned(),
        period_start,
        period_end,
        correlation_group: columns[12].to_owned(),
        method_id: columns[13].to_owned(),
        reason: columns[14].to_owned(),
    })
}

fn parse(source: &str) -> Result<(Metadata, Vec<FlowTerm>), String> {
    let mut metadata = BTreeMap::new();
    let mut terms = Vec::new();
    let mut header_seen = false;
    let mut seen = BTreeSet::new();

    for (index, raw) in source.lines().enumerate() {
        let line_number = index + 1;
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            if let Some((key, value)) = comment.trim().split_once('=') {
                let key = key.trim().to_owned();
                if metadata.insert(key.clone(), value.trim().to_owned()).is_some() {
                    return Err(format!("line {line_number}: duplicate metadata key {key}"));
                }
            }
            continue;
        }
        if !header_seen {
            if line != HEADER {
                return Err(format!("line {line_number}: unexpected flow-term header"));
            }
            header_seen = true;
            continue;
        }

        let term = parse_term(line, line_number)?;
        if !seen.insert(term.term_id.clone()) {
            return Err(format!("line {line_number}: duplicate term {}", term.term_id));
        }
        terms.push(term);
    }

    if !header_seen {
        return Err("missing flow-term header".to_owned());
    }
    let parsed_metadata = parse_metadata(&mut metadata)?;
    for required in REQUIRED_TERMS {
        if !seen.contains(required) {
            return Err(format!("missing required flow term {required}"));
        }
    }
    if terms.len() != REQUIRED_TERMS.len() {
        return Err("unexpected flow term count".to_owned());
    }

    for term in &terms {
        if term.source_owner.is_empty()
            || term.source_ref.is_empty()
            || term.source_schema.is_empty()
            || term.method_id.is_empty()
            || term.admissibility.is_empty()
        {
            return Err(format!("missing source or method lineage for {}", term.term_id));
        }
        if term.time_basis_id != parsed_metadata.time_basis_id
            || term.geography_id != parsed_metadata.geography_id
            || term.period_start != parsed_metadata.period_start
            || term.period_end != parsed_metadata.period_end
        {
            return Err(format!(
                "scope/time mismatch for flow term {}",
                term.term_id
            ));
        }
        if term.status == TermStatus::NotApplicable && !term.reason.contains("boundary") {
            return Err(format!(
                "not-applicable term {} must cite its boundary rationale",
                term.term_id
            ));
        }
    }
    Ok((parsed_metadata, terms))
}

fn interval(terms: &[FlowTerm], id: &str) -> (i128, i128) {
    let term = terms.iter().find(|term| term.term_id == id).expect("required term is present");
    (
        i128::from(term.lower.expect("known term has lower bound")),
        i128::from(term.upper.expect("known term has upper bound")),
    )
}

fn render(metadata: &Metadata, terms: &[FlowTerm]) -> String {
    let mut output = String::new();
    output.push_str("report_schema=fsre-flow-balance-report.v0.1\n");
    output.push_str(&format!("balance_id={}\n", metadata.balance_id));
    output.push_str(&format!("service_id={}\n", metadata.service_id));
    output.push_str(&format!("scenario_id={}\n", metadata.scenario_id));
    output.push_str(&format!("scope_ref={}\n", metadata.scope_ref));
    output.push_str(&format!("time_basis_id={}\n", metadata.time_basis_id));
    output.push_str(&format!("geography_id={}\n", metadata.geography_id));
    output.push_str(&format!("period_start_seconds={}\n", metadata.period_start));
    output.push_str(&format!("period_end_seconds={}\n", metadata.period_end));
    output.push_str(&format!("unit={}\n", metadata.unit));
    output.push_str(&format!("boundary_description={}\n", metadata.boundary_description));

    let mut ordered = terms.to_vec();
    ordered.sort_by(|left, right| left.term_id.cmp(&right.term_id));
    let unknown: Vec<&str> = ordered
        .iter()
        .filter(|term| term.status == TermStatus::Unknown)
        .map(|term| term.term_id.as_str())
        .collect();
    let mut correlations: BTreeSet<&str> = BTreeSet::new();

    for term in &ordered {
        output.push_str(&format!("term.{}.status={}\n", term.term_id, term.status.as_str()));
        output.push_str(&format!("term.{}.source_owner={}\n", term.term_id, term.source_owner));
        output.push_str(&format!("term.{}.source_ref={}\n", term.term_id, term.source_ref));
        output.push_str(&format!("term.{}.source_schema={}\n", term.term_id, term.source_schema));
        output.push_str(&format!("term.{}.admissibility={}\n", term.term_id, term.admissibility));
        output.push_str(&format!("term.{}.method_id={}\n", term.term_id, term.method_id));
        if !term.correlation_group.is_empty() {
            correlations.insert(term.correlation_group.as_str());
        }
        match (term.lower, term.upper) {
            (Some(lower), Some(upper)) => output.push_str(&format!(
                "term.{}.interval=[{},{}] {}\n",
                term.term_id, lower, upper, metadata.unit
            )),
            _ => output.push_str(&format!("term.{}.interval=UNKNOWN\n", term.term_id)),
        }
        if !term.reason.is_empty() {
            output.push_str(&format!("term.{}.reason={}\n", term.term_id, term.reason));
        }
    }

    output.push_str(&format!("distinct_correlation_groups={}\n", correlations.len()));
    output.push_str("interval_method=CONSERVATIVE_OUTER_BOUND;DEPENDENCE_NOT_MODELLED\n");

    if unknown.is_empty() {
        let (g_l, g_u) = interval(terms, "generation");
        let (i_l, i_u) = interval(terms, "other_inflows");
        let (t_l, t_u) = interval(terms, "treated");
        let (b_l, b_u) = interval(terms, "bypass");
        let (r_l, r_u) = interval(terms, "reuse");
        let (s_l, s_u) = interval(terms, "storage_change");

        let residual_lower = g_l + i_l - t_u - b_u - r_u - s_u;
        let residual_upper = g_u + i_u - t_l - b_l - r_l - s_l;
        output.push_str(&format!("unaccounted_residual_interval=[{residual_lower},{residual_upper}] {}\n", metadata.unit));

        let interpretation = if residual_lower > 0 {
            "POSITIVE_UNACCOUNTED_RESIDUAL"
        } else if residual_upper < 0 {
            "NEGATIVE_ACCOUNTING_MISMATCH"
        } else if residual_lower == 0 && residual_upper == 0 {
            "ZERO_RESIDUAL_WITHIN_DECLARED_INTERVALS"
        } else {
            "SIGN_OR_ZERO_NOT_IDENTIFIABLE"
        };
        output.push_str(&format!("residual_interpretation={interpretation}\n"));
        output.push_str("residual_causal_attribution=NONE\n");
    } else {
        output.push_str(&format!("unknown_required_terms={}\n", unknown.join(",")));
        output.push_str("unaccounted_residual_interval=NOT_ESTIMABLE\n");
        output.push_str("residual_interpretation=INCOMPLETE_REQUIRED_EVIDENCE\n");
        output.push_str("residual_causal_attribution=NONE\n");
    }

    output.push_str("source_authenticity=NOT_INDEPENDENTLY_AUTHENTICATED\n");
    output.push_str("physical_model_qualification=NOT_ASSESSED\n");
    output.push_str("intervention_authority=NONE\n");
    output.push_str("operational_actuation=NOT_AUTHORISED\n");
    output
}

pub fn validate_and_report(source: &str) -> Result<String, String> {
    let (metadata, terms) = parse(source)?;
    Ok(render(&metadata, &terms))
}

#[cfg(test)]
mod tests {
    use super::*;

    const BALANCED: &str = include_str!("../fixtures/synthetic-balanced.psv");
    const UNKNOWN_BYPASS: &str = include_str!("../fixtures/synthetic-unknown-bypass.psv");

    #[test]
    fn balanced_fixture_has_exact_zero_residual() {
        let report = validate_and_report(BALANCED).expect("balanced fixture should validate");
        assert!(report.contains("unaccounted_residual_interval=[0,0] m3"));
        assert!(report.contains("residual_interpretation=ZERO_RESIDUAL_WITHIN_DECLARED_INTERVALS"));
    }

    #[test]
    fn unknown_bypass_blocks_residual_calculation() {
        let report = validate_and_report(UNKNOWN_BYPASS).expect("unknown fixture should validate");
        assert!(report.contains("unknown_required_terms=bypass"));
        assert!(report.contains("unaccounted_residual_interval=NOT_ESTIMABLE"));
    }

    #[test]
    fn negative_residual_is_preserved_not_clipped() {
        let source = BALANCED.replace("treated|OBSERVED_INTERVAL|80|80", "treated|OBSERVED_INTERVAL|95|95");
        let report = validate_and_report(&source).expect("negative residual case should validate");
        assert!(report.contains("unaccounted_residual_interval=[-15,-15] m3"));
        assert!(report.contains("residual_interpretation=NEGATIVE_ACCOUNTING_MISMATCH"));
    }

    #[test]
    fn positive_residual_is_bounded_not_attributed() {
        let source = BALANCED.replace("treated|OBSERVED_INTERVAL|80|80", "treated|OBSERVED_INTERVAL|60|60");
        let report = validate_and_report(&source).expect("positive residual case should validate");
        assert!(report.contains("unaccounted_residual_interval=[20,20] m3"));
        assert!(report.contains("residual_causal_attribution=NONE"));
    }

    #[test]
    fn duplicate_terms_are_rejected() {
        let row = BALANCED
            .lines()
            .find(|line| line.starts_with("generation|"))
            .expect("generation term exists");
        assert!(validate_and_report(&format!("{BALANCED}{row}\n")).is_err());
    }

    #[test]
    fn mismatched_period_is_rejected() {
        let mutated = BALANCED.replace("treated|OBSERVED_INTERVAL|80|80|operator", "treated|OBSERVED_INTERVAL|80|80|operator");
        let mutated = mutated.replace("treated|OBSERVED_INTERVAL|80|80|operator|", "treated|OBSERVED_INTERVAL|80|80|operator|");
        let mutated = mutated.replace("|100|200||meter-volume-v1|", "|100|201||meter-volume-v1|");
        assert!(validate_and_report(&mutated).is_err());
    }

    #[test]
    fn impossible_negative_physical_flow_is_rejected() {
        let mutated = BALANCED.replace("treated|OBSERVED_INTERVAL|80|80", "treated|OBSERVED_INTERVAL|-1|80");
        assert!(validate_and_report(&mutated).is_err());
    }

    #[test]
    fn not_applicable_is_an_explicit_boundary_scoped_zero() {
        let report = validate_and_report(UNKNOWN_BYPASS).expect("unknown fixture should validate");
        assert!(report.contains("term.other_inflows.interval=[0,0] m3"));
        assert!(report.contains("term.other_inflows.reason=excluded by boundary"));
    }

    #[test]
    fn report_is_invariant_to_term_order() {
        let mut lines: Vec<&str> = BALANCED.lines().filter(|line| !line.starts_with('#') && !line.starts_with("term_id|")).collect();
        lines.reverse();
        let metadata: Vec<&str> = BALANCED.lines().filter(|line| line.starts_with('#')).collect();
        let reordered = format!("{}\n{}\n{}\n", metadata.join("\n"), HEADER, lines.join("\n"));
        assert_eq!(
            validate_and_report(BALANCED).expect("original"),
            validate_and_report(&reordered).expect("reordered")
        );
    }
}
