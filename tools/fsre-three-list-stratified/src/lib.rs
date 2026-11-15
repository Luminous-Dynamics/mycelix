#![forbid(unsafe_code)]

// Stratified three-list capture-recapture composition.
//
// Each stratum is evaluated by the exact single-stratum reference crate.
// This wrapper sums independent stratum sensitivity intervals, but only
// emits target-population totals when the full stratum universe and frame
// are declared complete. A pooled table is emitted as a diagnostic only.

use fsre_three_list_capture::{
    analyze as analyze_single, Problem as SingleProblem, Rational,
};
use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

const STRATA_HEADER: &str = "stratum_id|theta_abc_min|theta_abc_max";
const PATTERNS_HEADER: &str = "stratum_id|capture_pattern|count";
const PATTERNS: [&str; 7] = ["001", "010", "011", "100", "101", "110", "111"];
const MAX_INPUT_BYTES: usize = 4 * 1024 * 1024;
const MAX_STRATA: usize = 128;
const MAX_CELL_COUNT: u64 = 1_000_000_000_000;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Stratum {
    pub id: String,
    pub theta_abc_min: Rational,
    pub theta_abc_max: Option<Rational>,
    pub counts: BTreeMap<String, u64>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Problem {
    pub analysis_id: String,
    pub assumption_set_id: String,
    pub source_ids: [String; 3],
    pub source_ancestry: [String; 3],
    pub population_frame_status: String,
    pub pattern_table_status: String,
    pub strata_universe_status: String,
    pub three_way_interaction_assumption: String,
    pub strata: Vec<Stratum>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StratumResult {
    pub stratum_id: String,
    pub status: String,
    pub observed_union: Option<u128>,
    pub cross_product_ratio: Option<Rational>,
    pub hidden_min: Option<Rational>,
    pub hidden_max: Option<Rational>,
    pub population_min: Option<Rational>,
    pub population_max: Option<Rational>,
    pub no_three_way_hidden_baseline: Option<Rational>,
    pub no_three_way_population_baseline: Option<Rational>,
    pub zero_patterns: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Analysis {
    pub problem: Problem,
    pub source_ancestry_relation: String,
    pub strata_results: Vec<StratumResult>,
    pub aggregate_status: String,
    pub within_declared_strata_observed_union: Option<u128>,
    pub within_declared_strata_hidden_min: Option<Rational>,
    pub within_declared_strata_hidden_max: Option<Rational>,
    pub target_hidden_min: Option<Rational>,
    pub target_hidden_max: Option<Rational>,
    pub target_population_min: Option<Rational>,
    pub target_population_max: Option<Rational>,
    pub no_three_way_hidden_baseline: Option<Rational>,
    pub no_three_way_population_baseline: Option<Rational>,
    pub pooled_cross_product_ratio_diagnostic: Option<Rational>,
    pub aggregate_numeric_overflow: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StratifiedError(pub String);

impl std::fmt::Display for StratifiedError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for StratifiedError {}

fn fail(message: impl Into<String>) -> StratifiedError {
    StratifiedError(message.into())
}

fn valid_token(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 200
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.' | b':')
        })
}

fn valid_ancestry_path(value: &str) -> bool {
    value == "UNKNOWN"
        || (!value.is_empty()
            && value
                .split('/')
                .all(|segment| !segment.is_empty() && valid_token(segment)))
}

fn ancestry_relation(paths: &[String; 3]) -> &'static str {
    if paths.iter().any(|path| path == "UNKNOWN") {
        return "UPSTREAM_ANCESTRY_UNKNOWN_INDEPENDENCE_WITHHELD";
    }
    let segments = paths
        .iter()
        .map(|path| path.split('/').collect::<BTreeSet<_>>())
        .collect::<Vec<_>>();
    for left in 0..segments.len() {
        for right in (left + 1)..segments.len() {
            if !segments[left].is_disjoint(&segments[right]) {
                return "SHARED_DECLARED_UPSTREAM_ANCESTRY";
            }
        }
    }
    "DISTINCT_DECLARED_ANCESTRY_INDEPENDENCE_NOT_ESTABLISHED"
}

fn required<'a>(
    metadata: &'a BTreeMap<String, String>,
    key: &str,
) -> Result<&'a str, StratifiedError> {
    metadata
        .get(key)
        .map(String::as_str)
        .ok_or_else(|| fail(format!("missing metadata key {key}")))
}

fn parse_flag(value: &str, field: &str) -> Result<String, StratifiedError> {
    match value {
        "DECLARED_COMPLETE" => Ok("DECLARED_COMPLETE".to_owned()),
        "NOT_ESTABLISHED" | "DECLARED_INCOMPLETE" => Ok("NOT_ESTABLISHED".to_owned()),
        _ => Err(fail(format!("unknown {field} state {value}"))),
    }
}

fn parse_theta_max(value: &str) -> Result<Option<Rational>, StratifiedError> {
    if value == "UNBOUNDED" {
        return Ok(None);
    }
    let rational = Rational::parse(value, "theta_abc_max")
        .map_err(|error| fail(error.to_string()))?;
    if rational.numerator == 0 {
        return Err(fail("theta_abc_max must be strictly positive or UNBOUNDED"));
    }
    Ok(Some(rational))
}

fn parse_section<'a>(
    sections: &'a BTreeMap<String, Vec<(usize, String)>>,
    name: &str,
    expected_header: &str,
) -> Result<Vec<(usize, Vec<String>)>, StratifiedError> {
    let lines = sections
        .get(name)
        .ok_or_else(|| fail(format!("missing [{name}] section")))?;
    if lines.first().map(|(_, line)| line.as_str()) != Some(expected_header) {
        return Err(fail(format!("expected exact {name} section header")));
    }
    let mut rows = Vec::new();
    for (line_no, line) in lines.iter().skip(1) {
        let fields = line.split('|').map(str::trim).collect::<Vec<_>>();
        rows.push((
            *line_no,
            fields.iter().map(|field| (*field).to_owned()).collect(),
        ));
    }
    Ok(rows)
}

pub fn parse_input(source: &str) -> Result<Problem, StratifiedError> {
    if source.len() > MAX_INPUT_BYTES {
        return Err(fail("input exceeds 4 MiB"));
    }

    let allowed_metadata = [
        "analysis_id",
        "assumption_set_id",
        "source_a_id",
        "source_b_id",
        "source_c_id",
        "source_a_ancestry",
        "source_b_ancestry",
        "source_c_ancestry",
        "population_frame_status",
        "pattern_table_status",
        "strata_universe_status",
        "three_way_interaction_assumption",
        "max_strata",
    ];
    let allowed_sections = ["STRATA", "PATTERNS"];
    let mut metadata = BTreeMap::<String, String>::new();
    let mut sections = BTreeMap::<String, Vec<(usize, String)>>::new();
    let mut current_section: Option<String> = None;

    for (index, raw) in source.lines().enumerate() {
        let line_no = index + 1;
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            if current_section.is_some() {
                return Err(fail(format!(
                    "line {line_no}: metadata must precede data sections"
                )));
            }
            let (key, value) = comment
                .trim()
                .split_once('=')
                .ok_or_else(|| fail(format!("line {line_no}: malformed metadata")))?;
            let key = key.trim().to_owned();
            let value = value.trim().to_owned();
            if !allowed_metadata.contains(&key.as_str()) {
                return Err(fail(format!("line {line_no}: unknown metadata key {key}")));
            }
            if value.is_empty() || metadata.insert(key.clone(), value).is_some() {
                return Err(fail(format!(
                    "line {line_no}: empty or duplicate metadata key {key}"
                )));
            }
            continue;
        }
        if line.starts_with('[') && line.ends_with(']') {
            let section = line[1..line.len() - 1].to_owned();
            if !allowed_sections.contains(&section.as_str()) {
                return Err(fail(format!("line {line_no}: unknown section {section}")));
            }
            if sections.contains_key(&section) {
                return Err(fail(format!("line {line_no}: duplicate section {section}")));
            }
            sections.insert(section.clone(), Vec::new());
            current_section = Some(section);
            continue;
        }
        let section = current_section
            .as_ref()
            .ok_or_else(|| fail(format!("line {line_no}: row precedes first section")))?;
        sections
            .get_mut(section)
            .expect("section inserted")
            .push((line_no, line.to_owned()));
    }

    let analysis_id = required(&metadata, "analysis_id")?.to_owned();
    let assumption_set_id = required(&metadata, "assumption_set_id")?.to_owned();
    if !valid_token(&analysis_id) || !valid_token(&assumption_set_id) {
        return Err(fail("analysis_id and assumption_set_id must be valid tokens"));
    }

    let source_ids = [
        required(&metadata, "source_a_id")?.to_owned(),
        required(&metadata, "source_b_id")?.to_owned(),
        required(&metadata, "source_c_id")?.to_owned(),
    ];
    if source_ids.iter().any(|id| !valid_token(id))
        || source_ids.iter().collect::<BTreeSet<_>>().len() != 3
    {
        return Err(fail("three distinct valid source IDs are required"));
    }

    let source_ancestry = [
        required(&metadata, "source_a_ancestry")?.to_owned(),
        required(&metadata, "source_b_ancestry")?.to_owned(),
        required(&metadata, "source_c_ancestry")?.to_owned(),
    ];
    if source_ancestry
        .iter()
        .any(|path| !valid_ancestry_path(path))
    {
        return Err(fail(
            "source ancestry must be UNKNOWN or a valid slash-separated path",
        ));
    }

    let population_frame_status =
        parse_flag(required(&metadata, "population_frame_status")?, "population_frame_status")?;
    let pattern_table_status =
        parse_flag(required(&metadata, "pattern_table_status")?, "pattern_table_status")?;
    let strata_universe_status =
        parse_flag(required(&metadata, "strata_universe_status")?, "strata_universe_status")?;
    let three_way_interaction_assumption =
        match required(&metadata, "three_way_interaction_assumption")? {
            "DECLARED_NO_THREE_WAY_INTERACTION" => "DECLARED_NO_THREE_WAY_INTERACTION",
            "NOT_DECLARED" => "NOT_DECLARED",
            value => return Err(fail(format!("unknown three_way_interaction_assumption {value}"))),
        }
        .to_owned();
    let max_strata = required(&metadata, "max_strata")?
        .parse::<usize>()
        .map_err(|error| fail(format!("invalid max_strata: {error}")))?;
    if max_strata == 0 || max_strata > MAX_STRATA {
        return Err(fail(format!("max_strata must be in 1..={MAX_STRATA}")));
    }

    let strata_rows = parse_section(&sections, "STRATA", STRATA_HEADER)?;
    let pattern_rows = parse_section(&sections, "PATTERNS", PATTERNS_HEADER)?;

    let mut strata = BTreeMap::<String, Stratum>::new();
    for (line_no, fields) in strata_rows {
        if fields.len() != 3 || !valid_token(&fields[0]) {
            return Err(fail(format!("line {line_no}: invalid stratum row")));
        }
        let id = fields[0].clone();
        let theta_abc_min = Rational::parse(&fields[1], "theta_abc_min")
            .map_err(|error| fail(error.to_string()))?;
        if theta_abc_min.numerator == 0 {
            return Err(fail(format!(
                "line {line_no}: theta_abc_min must be strictly positive"
            )));
        }
        let theta_abc_max = parse_theta_max(&fields[2])?;
        if let Some(maximum) = theta_abc_max {
            if theta_abc_min
                .checked_cmp(maximum)
                .map_or(true, |ordering| ordering == Ordering::Greater)
            {
                return Err(fail(format!(
                    "line {line_no}: theta_abc_min exceeds theta_abc_max"
                )));
            }
        }
        if three_way_interaction_assumption == "DECLARED_NO_THREE_WAY_INTERACTION" {
            let one = Rational::one();
            let includes_one = theta_abc_min
                .checked_cmp(one)
                .is_some_and(|ordering| ordering != Ordering::Greater)
                && theta_abc_max.map_or(true, |maximum| {
                    maximum
                        .checked_cmp(one)
                        .is_some_and(|ordering| ordering != Ordering::Less)
                });
            if !includes_one {
                return Err(fail(format!(
                    "line {line_no}: no-three-way baseline conflicts with theta interval"
                )));
            }
        }
        if strata
            .insert(
                id.clone(),
                Stratum {
                    id,
                    theta_abc_min,
                    theta_abc_max,
                    counts: BTreeMap::new(),
                },
            )
            .is_some()
        {
            return Err(fail(format!("line {line_no}: duplicate stratum ID")));
        }
        if strata.len() > max_strata {
            return Err(fail("stratum count exceeds declared max_strata"));
        }
    }
    if strata.is_empty() {
        return Err(fail("at least one stratum is required"));
    }

    for (line_no, fields) in pattern_rows {
        if fields.len() != 3 || !valid_token(&fields[0]) {
            return Err(fail(format!("line {line_no}: invalid pattern row")));
        }
        let stratum = strata
            .get_mut(&fields[0])
            .ok_or_else(|| fail(format!("line {line_no}: unknown stratum {}", fields[0])))?;
        if !PATTERNS.contains(&fields[1].as_str()) {
            return Err(fail(format!(
                "line {line_no}: unknown capture pattern {}",
                fields[1]
            )));
        }
        let count = fields[2]
            .parse::<u64>()
            .map_err(|error| fail(format!("line {line_no}: invalid count: {error}")))?;
        if count > MAX_CELL_COUNT {
            return Err(fail(format!("line {line_no}: count exceeds {MAX_CELL_COUNT}")));
        }
        if stratum.counts.insert(fields[1].clone(), count).is_some() {
            return Err(fail(format!(
                "line {line_no}: duplicate pattern {} in stratum {}",
                fields[1], fields[0]
            )));
        }
    }

    for stratum in strata.values() {
        if stratum.counts.len() != PATTERNS.len() {
            return Err(fail(format!(
                "stratum {} must have all seven observed patterns exactly once",
                stratum.id
            )));
        }
    }

    Ok(Problem {
        analysis_id,
        assumption_set_id,
        source_ids,
        source_ancestry,
        population_frame_status,
        pattern_table_status,
        strata_universe_status,
        three_way_interaction_assumption,
        strata: strata.into_values().collect(),
    })
}

fn single_problem(problem: &Problem, stratum: &Stratum) -> SingleProblem {
    SingleProblem {
        analysis_id: format!("{}-{}", problem.analysis_id, stratum.id),
        assumption_set_id: problem.assumption_set_id.clone(),
        source_ids: problem.source_ids.clone(),
        source_ancestry: problem.source_ancestry.clone(),
        population_frame_status: problem.population_frame_status.clone(),
        pattern_table_status: problem.pattern_table_status.clone(),
        three_way_interaction_assumption: problem.three_way_interaction_assumption.clone(),
        theta_abc_min: stratum.theta_abc_min,
        theta_abc_max: stratum.theta_abc_max,
        counts: stratum.counts.clone(),
    }
}

fn sum_rationals<I>(values: I) -> Option<Rational>
where
    I: IntoIterator<Item = Rational>,
{
    values
        .into_iter()
        .try_fold(Rational::zero(), |sum, value| sum.checked_add(value))
}

fn pooled_ratio_diagnostic(strata: &[Stratum]) -> Option<Rational> {
    let mut pooled = BTreeMap::<String, u128>::new();
    for pattern in PATTERNS {
        let total = strata.iter().try_fold(0_u128, |sum, stratum| {
            sum.checked_add(u128::from(*stratum.counts.get(pattern)?))
        })?;
        pooled.insert(pattern.to_owned(), total);
    }
    let numerator = pooled["111"]
        .checked_mul(pooled["100"])?
        .checked_mul(pooled["010"])?
        .checked_mul(pooled["001"])?;
    let denominator = pooled["110"]
        .checked_mul(pooled["101"])?
        .checked_mul(pooled["011"])?;
    Rational::new(numerator, denominator)
}

fn source_relation_for(problem: &Problem) -> String {
    if problem
        .source_ancestry
        .iter()
        .any(|path| path == "UNKNOWN")
    {
        return "UPSTREAM_ANCESTRY_UNKNOWN_INDEPENDENCE_WITHHELD".to_owned();
    }
    let ancestry = problem
        .source_ancestry
        .iter()
        .map(|path| path.split('/').collect::<BTreeSet<_>>())
        .collect::<Vec<_>>();
    for left in 0..ancestry.len() {
        for right in (left + 1)..ancestry.len() {
            if !ancestry[left].is_disjoint(&ancestry[right]) {
                return "SHARED_DECLARED_UPSTREAM_ANCESTRY".to_owned();
            }
        }
    }
    "DISTINCT_DECLARED_ANCESTRY_INDEPENDENCE_NOT_ESTABLISHED".to_owned()
}

pub fn analyze(problem: &Problem) -> Analysis {
    let source_ancestry_relation = source_relation_for(problem);
    if problem.strata.is_empty() {
        return Analysis {
            problem: problem.clone(),
            source_ancestry_relation,
            strata_results: Vec::new(),
            aggregate_status: "AGGREGATE_NOT_ESTABLISHED_NO_STRATA".to_owned(),
            within_declared_strata_observed_union: None,
            within_declared_strata_hidden_min: None,
            within_declared_strata_hidden_max: None,
            target_hidden_min: None,
            target_hidden_max: None,
            target_population_min: None,
            target_population_max: None,
            no_three_way_hidden_baseline: None,
            no_three_way_population_baseline: None,
            pooled_cross_product_ratio_diagnostic: None,
            aggregate_numeric_overflow: false,
        };
    }
    let strata_results = problem
        .strata
        .iter()
        .map(|stratum| {
            let result = analyze_single(&single_problem(problem, stratum));
            StratumResult {
                stratum_id: stratum.id.clone(),
                status: result.status,
                observed_union: result.observed_union,
                cross_product_ratio: result.cross_product_ratio,
                hidden_min: result.hidden_min,
                hidden_max: result.hidden_max,
                population_min: result.population_min,
                population_max: result.population_max,
                no_three_way_hidden_baseline: result.no_three_way_hidden_baseline,
                no_three_way_population_baseline: result.no_three_way_population_baseline,
                zero_patterns: result.zero_patterns,
            }
        })
        .collect::<Vec<_>>();

    let within_declared_strata_observed_union = strata_results
        .iter()
        .try_fold(0_u128, |sum, result| sum.checked_add(result.observed_union?));
    let all_strata_have_bounds = strata_results
        .iter()
        .all(|result| result.hidden_min.is_some() && result.hidden_max.is_some());
    let within_declared_strata_hidden_min = if all_strata_have_bounds {
        sum_rationals(
            strata_results
                .iter()
                .filter_map(|result| result.hidden_min),
        )
    } else {
        None
    };
    let within_declared_strata_hidden_max = if all_strata_have_bounds {
        sum_rationals(
            strata_results
                .iter()
                .filter_map(|result| result.hidden_max),
        )
    } else {
        None
    };

    let strata_universe_complete = problem.strata_universe_status == "DECLARED_COMPLETE";
    let pattern_table_complete = problem.pattern_table_status == "DECLARED_COMPLETE";
    let frame_complete = problem.population_frame_status == "DECLARED_COMPLETE";
    let target_hidden_min = (strata_universe_complete && pattern_table_complete)
        .then_some(within_declared_strata_hidden_min)
        .flatten();
    let target_hidden_max = (strata_universe_complete && pattern_table_complete)
        .then_some(within_declared_strata_hidden_max)
        .flatten();

    let target_population_min = if frame_complete && strata_universe_complete && pattern_table_complete {
        match (within_declared_strata_observed_union, target_hidden_min) {
            (Some(observed), Some(hidden)) => Rational::integer(observed).checked_add(hidden),
            _ => None,
        }
    } else {
        None
    };
    let target_population_max = if frame_complete && strata_universe_complete && pattern_table_complete {
        match (within_declared_strata_observed_union, target_hidden_max) {
            (Some(observed), Some(hidden)) => Rational::integer(observed).checked_add(hidden),
            _ => None,
        }
    } else {
        None
    };

    let all_baselines = strata_results
        .iter()
        .all(|result| result.no_three_way_hidden_baseline.is_some());
    let no_three_way_hidden_baseline =
        if strata_universe_complete && pattern_table_complete && all_baselines {
            sum_rationals(
                strata_results
                    .iter()
                    .filter_map(|result| result.no_three_way_hidden_baseline),
            )
        } else {
            None
        };
    let no_three_way_population_baseline =
        if frame_complete && strata_universe_complete && pattern_table_complete {
            match (
                within_declared_strata_observed_union,
                no_three_way_hidden_baseline,
            ) {
                (Some(observed), Some(hidden)) => Rational::integer(observed).checked_add(hidden),
                _ => None,
            }
        } else {
            None
        };

    let pooled_cross_product_ratio_diagnostic = if pattern_table_complete {
        pooled_ratio_diagnostic(&problem.strata)
    } else {
        None
    };

    let aggregate_numeric_overflow = all_strata_have_bounds
        && (within_declared_strata_hidden_min.is_none()
            || within_declared_strata_hidden_max.is_none()
            || within_declared_strata_observed_union.is_none());

    let aggregate_status = if !strata_universe_complete {
        "AGGREGATE_NOT_ESTABLISHED_STRATA_UNIVERSE_INCOMPLETE"
    } else if !pattern_table_complete {
        "AGGREGATE_NOT_ESTABLISHED_PATTERN_TABLE_INCOMPLETE"
    } else if !all_strata_have_bounds {
        "AGGREGATE_NOT_ESTABLISHED_STRATUM_UNRESOLVED"
    } else if aggregate_numeric_overflow
        || target_hidden_min.is_none()
        || target_hidden_max.is_none()
    {
        "AGGREGATE_ARITHMETIC_OR_BOUND_FAILURE"
    } else if !frame_complete {
        "COMPUTED_HIDDEN_BOUNDS_POPULATION_TOTAL_WITHHELD_FRAME_INCOMPLETE"
    } else {
        "COMPUTED_STRATIFIED_THREE_LIST_INTERVAL"
    };

    Analysis {
        problem: problem.clone(),
        source_ancestry_relation,
        strata_results,
        aggregate_status: aggregate_status.to_owned(),
        within_declared_strata_observed_union,
        within_declared_strata_hidden_min,
        within_declared_strata_hidden_max,
        target_hidden_min,
        target_hidden_max,
        target_population_min,
        target_population_max,
        no_three_way_hidden_baseline,
        no_three_way_population_baseline,
        pooled_cross_product_ratio_diagnostic,
        aggregate_numeric_overflow,
    }
}

fn write_bound(out: &mut String, key: &str, value: Option<Rational>) {
    match value {
        Some(value) => writeln!(out, "{key}={}", value.display()).expect("String write"),
        None => writeln!(out, "{key}=NOT_ESTABLISHED").expect("String write"),
    }
}

impl Analysis {
    pub fn render(&self) -> String {
        let mut out = String::new();
        writeln!(out, "schema=fsre.three-list-stratified-e0").expect("String write");
        writeln!(out, "version=0.1.0").expect("String write");
        writeln!(out, "analysis_id={}", self.problem.analysis_id).expect("String write");
        writeln!(
            out,
            "assumption_set_id={}",
            self.problem.assumption_set_id
        )
        .expect("String write");
        for index in 0..3 {
            let label = ["a", "b", "c"][index];
            writeln!(out, "source_{label}_id={}", self.problem.source_ids[index])
                .expect("String write");
            writeln!(
                out,
                "source_{label}_ancestry={}",
                self.problem.source_ancestry[index]
            )
            .expect("String write");
        }
        writeln!(
            out,
            "source_ancestry_relation={}",
            self.source_ancestry_relation
        )
        .expect("String write");
        writeln!(
            out,
            "population_frame_status={}",
            self.problem.population_frame_status
        )
        .expect("String write");
        writeln!(
            out,
            "pattern_table_status={}",
            self.problem.pattern_table_status
        )
        .expect("String write");
        writeln!(
            out,
            "strata_universe_status={}",
            self.problem.strata_universe_status
        )
        .expect("String write");
        writeln!(
            out,
            "three_way_interaction_assumption={}",
            self.problem.three_way_interaction_assumption
        )
        .expect("String write");
        writeln!(out, "stratum_count={}", self.problem.strata.len()).expect("String write");

        for result in &self.strata_results {
            let prefix = format!("stratum.{}", result.stratum_id);
            writeln!(out, "{prefix}.status={}", result.status).expect("String write");
            writeln!(
                out,
                "{prefix}.observed_union={}",
                result
                    .observed_union
                    .map(|value| value.to_string())
                    .unwrap_or_else(|| "NOT_ESTABLISHED".to_owned())
            )
            .expect("String write");
            write_bound(
                &mut out,
                &format!("{prefix}.cross_product_ratio"),
                result.cross_product_ratio,
            );
            write_bound(&mut out, &format!("{prefix}.hidden_lower"), result.hidden_min);
            write_bound(&mut out, &format!("{prefix}.hidden_upper"), result.hidden_max);
            write_bound(
                &mut out,
                &format!("{prefix}.population_lower"),
                result.population_min,
            );
            write_bound(
                &mut out,
                &format!("{prefix}.population_upper"),
                result.population_max,
            );
            write_bound(
                &mut out,
                &format!("{prefix}.no_three_way_hidden_baseline"),
                result.no_three_way_hidden_baseline,
            );
            write_bound(
                &mut out,
                &format!("{prefix}.no_three_way_population_baseline"),
                result.no_three_way_population_baseline,
            );
            writeln!(
                out,
                "{prefix}.zero_patterns={}",
                result.zero_patterns.join(",")
            )
            .expect("String write");
        }

        writeln!(out, "aggregate_status={}", self.aggregate_status).expect("String write");
        writeln!(
            out,
            "within_declared_strata_observed_union={}",
            self.within_declared_strata_observed_union
                .map(|value| value.to_string())
                .unwrap_or_else(|| "NOT_ESTABLISHED".to_owned())
        )
        .expect("String write");
        write_bound(
            &mut out,
            "within_declared_strata_hidden_lower",
            self.within_declared_strata_hidden_min,
        );
        write_bound(
            &mut out,
            "within_declared_strata_hidden_upper",
            self.within_declared_strata_hidden_max,
        );
        write_bound(&mut out, "target_hidden_lower", self.target_hidden_min);
        write_bound(&mut out, "target_hidden_upper", self.target_hidden_max);
        write_bound(&mut out, "target_population_lower", self.target_population_min);
        write_bound(&mut out, "target_population_upper", self.target_population_max);
        write_bound(
            &mut out,
            "stratified_no_three_way_hidden_baseline",
            self.no_three_way_hidden_baseline,
        );
        write_bound(
            &mut out,
            "stratified_no_three_way_population_baseline",
            self.no_three_way_population_baseline,
        );
        write_bound(
            &mut out,
            "pooled_cross_product_ratio_diagnostic_only",
            self.pooled_cross_product_ratio_diagnostic,
        );
        writeln!(
            out,
            "pooled_ratio_use=DIAGNOSTIC_ONLY_NOT_USED_FOR_STRATIFIED_BOUNDS"
        )
        .expect("String write");
        writeln!(
            out,
            "aggregate_numeric_overflow={}",
            self.aggregate_numeric_overflow
        )
        .expect("String write");
        out.push_str("stratum_theta_intervals=SEPARATE_DECLARED_SENSITIVITY_INTERVALS\n");
        out.push_str("cross_stratum_restrictions=NONE_DECLARED_CARTESIAN_PRODUCT_ASSUMPTION\n");
        out.push_str("stratified_interval=SUM_OF_PER_STRATUM_INTERVAL_ENDPOINTS\n");
        out.push_str("population_total_requires_complete_frame_and_strata_universe=true\n");
        out.push_str("three_way_theta_interval=DECLARED_SENSITIVITY_NOT_SAMPLING_CI\n");
        out.push_str("no_three_way_baseline=CONDITIONAL_MODEL_BASELINE_NOT_CONFIDENCE_INTERVAL\n");
        out.push_str("source_independence=NOT_ESTABLISHED_BY_ANCESTRY_LABELS\n");
        out.push_str("population_frame_authentication=NOT_ESTABLISHED_BY_E0\n");
        out.push_str("strata_universe_authentication=NOT_ESTABLISHED_BY_E0\n");
        out.push_str("source_authentication=NOT_ESTABLISHED_BY_E0\n");
        out.push_str("field_outcome=NOT_ESTABLISHED_BY_E0\n");
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../fixtures/two-strata.psv");

    fn fixture() -> Problem {
        parse_input(FIXTURE).expect("fixture parses")
    }

    #[test]
    fn exact_stratum_and_aggregate_bounds_match_known_answer() {
        let report = analyze(&fixture());
        assert_eq!(
            report.aggregate_status,
            "COMPUTED_STRATIFIED_THREE_LIST_INTERVAL"
        );
        assert_eq!(report.strata_results.len(), 2);
        assert_eq!(report.strata_results[0].stratum_id, "rural");
        assert_eq!(report.strata_results[0].cross_product_ratio.unwrap().display(), "90");
        assert_eq!(report.strata_results[0].hidden_min.unwrap().display(), "30");
        assert_eq!(report.strata_results[0].hidden_max.unwrap().display(), "90");
        assert_eq!(report.strata_results[1].stratum_id, "urban");
        assert_eq!(report.strata_results[1].cross_product_ratio.unwrap().display(), "240");
        assert_eq!(report.strata_results[1].hidden_min.unwrap().display(), "120");
        assert_eq!(report.strata_results[1].hidden_max.unwrap().display(), "480");
        assert_eq!(report.within_declared_strata_observed_union, Some(233));
        assert_eq!(report.target_hidden_min.unwrap().display(), "150");
        assert_eq!(report.target_hidden_max.unwrap().display(), "570");
        assert_eq!(report.target_population_min.unwrap().display(), "383");
        assert_eq!(report.target_population_max.unwrap().display(), "803");
        assert_eq!(
            report.no_three_way_hidden_baseline.unwrap().display(),
            "330"
        );
        assert_eq!(
            report.no_three_way_population_baseline.unwrap().display(),
            "563"
        );
    }

    #[test]
    fn pooled_ratio_is_diagnostic_and_not_substituted_for_stratified_sum() {
        let report = analyze(&fixture());
        assert_eq!(
            report.pooled_cross_product_ratio_diagnostic.unwrap().display(),
            "315"
        );
        assert_eq!(
            report.no_three_way_hidden_baseline.unwrap().display(),
            "330"
        );
        assert_ne!(
            report.pooled_cross_product_ratio_diagnostic,
            report.no_three_way_hidden_baseline
        );
    }

    #[test]
    fn incomplete_strata_universe_preserves_within_declared_bounds_but_withholds_target() {
        let mut problem = fixture();
        problem.strata_universe_status = "NOT_ESTABLISHED".to_owned();
        let report = analyze(&problem);
        assert_eq!(
            report.aggregate_status,
            "AGGREGATE_NOT_ESTABLISHED_STRATA_UNIVERSE_INCOMPLETE"
        );
        assert_eq!(
            report.within_declared_strata_hidden_min.unwrap().display(),
            "150"
        );
        assert_eq!(
            report.within_declared_strata_hidden_max.unwrap().display(),
            "570"
        );
        assert!(report.target_hidden_min.is_none());
        assert!(report.target_hidden_max.is_none());
        assert!(report.target_population_min.is_none());
        assert!(report.target_population_max.is_none());
    }

    #[test]
    fn incomplete_frame_withholds_population_totals_but_retains_complete_hidden_bounds() {
        let mut problem = fixture();
        problem.population_frame_status = "NOT_ESTABLISHED".to_owned();
        let report = analyze(&problem);
        assert_eq!(
            report.aggregate_status,
            "COMPUTED_HIDDEN_BOUNDS_POPULATION_TOTAL_WITHHELD_FRAME_INCOMPLETE"
        );
        assert_eq!(report.target_hidden_min.unwrap().display(), "150");
        assert_eq!(report.target_hidden_max.unwrap().display(), "570");
        assert!(report.target_population_min.is_none());
        assert!(report.target_population_max.is_none());
    }

    #[test]
    fn a_zero_cell_in_one_stratum_withholds_aggregate_but_preserves_other_stratum() {
        let mut problem = fixture();
        problem
            .strata
            .iter_mut()
            .find(|stratum| stratum.id == "urban")
            .unwrap()
            .counts
            .insert("101".to_owned(), 0);
        let report = analyze(&problem);
        assert_eq!(
            report.aggregate_status,
            "AGGREGATE_NOT_ESTABLISHED_STRATUM_UNRESOLVED"
        );
        let rural = report
            .strata_results
            .iter()
            .find(|result| result.stratum_id == "rural")
            .unwrap();
        let urban = report
            .strata_results
            .iter()
            .find(|result| result.stratum_id == "urban")
            .unwrap();
        assert!(rural.hidden_min.is_some());
        assert!(rural.hidden_max.is_some());
        assert!(urban.hidden_min.is_none());
        assert!(urban.hidden_max.is_none());
        assert!(report.target_population_min.is_none());
    }

    #[test]
    fn shared_ancestry_suppresses_every_point_baseline_but_not_sensitivity() {
        let mut problem = fixture();
        problem.source_ancestry[0] = "shared-root/utility".to_owned();
        problem.source_ancestry[2] = "shared-root/benefits".to_owned();
        let report = analyze(&problem);
        assert_eq!(
            report.source_ancestry_relation,
            "SHARED_DECLARED_UPSTREAM_ANCESTRY"
        );
        assert!(report
            .strata_results
            .iter()
            .all(|result| result.no_three_way_hidden_baseline.is_none()));
        assert!(report.no_three_way_hidden_baseline.is_none());
        assert!(report.target_hidden_min.is_some());
        assert!(report.target_hidden_max.is_some());
    }

    #[test]
    fn incomplete_pattern_universe_withholds_all_bounds() {
        let mut problem = fixture();
        problem.pattern_table_status = "NOT_ESTABLISHED".to_owned();
        let report = analyze(&problem);
        assert_eq!(
            report.aggregate_status,
            "AGGREGATE_NOT_ESTABLISHED_PATTERN_TABLE_INCOMPLETE"
        );
        assert!(report.within_declared_strata_hidden_min.is_none());
        assert!(report.target_hidden_min.is_none());
    }

    #[test]
    fn deterministic_under_stratum_and_pattern_row_permutation() {
        let original = analyze(&fixture()).render();
        let mut metadata = Vec::new();
        let mut sections = BTreeMap::<String, Vec<String>>::new();
        let mut section = String::new();
        for raw in FIXTURE.lines() {
            let line = raw.trim();
            if line.starts_with('#') {
                metadata.push(line.to_owned());
            } else if line.starts_with('[') && line.ends_with(']') {
                section = line.to_owned();
                sections.entry(section.clone()).or_default();
            } else if !line.is_empty() {
                sections
                    .get_mut(&section)
                    .expect("section exists")
                    .push(line.to_owned());
            }
        }
        for rows in sections.values_mut() {
            rows[1..].reverse();
        }
        let mut input = String::new();
        for line in metadata {
            writeln!(input, "{line}").unwrap();
        }
        for (name, rows) in sections {
            writeln!(input, "{name}").unwrap();
            for row in rows {
                writeln!(input, "{row}").unwrap();
            }
        }
        assert_eq!(original, analyze(&parse_input(&input).unwrap()).render());
    }

    #[test]
    fn missing_duplicate_stratum_and_pattern_rows_fail_closed() {
        let missing_pattern = FIXTURE.replace("urban|101|20\n", "");
        assert!(parse_input(&missing_pattern).is_err());
        let duplicate_stratum = FIXTURE.replace("urban|1/2|2\n", "urban|1/2|2\nurban|1/2|2\n");
        assert!(parse_input(&duplicate_stratum).is_err());
        let duplicate_pattern = FIXTURE.replace("urban|101|20\n", "urban|101|20\nurban|101|20\n");
        assert!(parse_input(&duplicate_pattern).is_err());
        let unknown_stratum = FIXTURE.replace("urban|100|40\n", "unknown|100|40\n");
        assert!(parse_input(&unknown_stratum).is_err());
    }

    #[test]
    fn rational_aggregate_overflow_is_not_silently_accepted() {
        let maximum = Rational::integer(u128::MAX);
        assert!(maximum.checked_add(Rational::one()).is_none());
    }
}
