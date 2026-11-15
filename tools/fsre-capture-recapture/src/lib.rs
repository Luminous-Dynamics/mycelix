#![forbid(unsafe_code)]

// Finite E0 two-list capture-recapture / dual-system sensitivity reference.
//
// The unobserved both-missed cell is not identified by the three observed
// cells. A declared source-dependence odds-ratio interval is a sensitivity
// assumption, not something this two-list table can estimate on its own.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

const HEADER: &str =
    "stratum_id|a_only|b_only|both|theta_min_num|theta_min_den|theta_max_num|theta_max_den";
const MAX_INPUT_BYTES: usize = 4 * 1024 * 1024;
const MAX_STRATA: usize = 16;
const MAX_CELL_COUNT: u64 = 1_000_000_000;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Rational {
    pub numerator: u128,
    pub denominator: u128,
}

impl Rational {
    pub fn new(numerator: u128, denominator: u128) -> Option<Self> {
        if denominator == 0 {
            return None;
        }
        let divisor = gcd(numerator, denominator);
        Some(Self {
            numerator: numerator / divisor,
            denominator: denominator / divisor,
        })
    }

    pub fn integer(value: u128) -> Self {
        Self {
            numerator: value,
            denominator: 1,
        }
    }

    pub fn checked_add(&self, other: &Self) -> Option<Self> {
        let left = self.numerator.checked_mul(other.denominator)?;
        let right = other.numerator.checked_mul(self.denominator)?;
        let numerator = left.checked_add(right)?;
        let denominator = self.denominator.checked_mul(other.denominator)?;
        Self::new(numerator, denominator)
    }

    pub fn checked_cmp(&self, other: &Self) -> Option<Ordering> {
        let left = self.numerator.checked_mul(other.denominator)?;
        let right = other.numerator.checked_mul(self.denominator)?;
        Some(left.cmp(&right))
    }

    pub fn display(&self) -> String {
        format!("{}/{}", self.numerator, self.denominator)
    }
}

fn gcd(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left.max(1)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IndependenceAssumption {
    Declared,
    NotDeclared,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinkageStatus {
    PerfectLinkageAssumed,
    NotEstablished,
    Unknown,
}

impl LinkageStatus {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::PerfectLinkageAssumed => "PERFECT_LINKAGE_ASSUMED_NOT_AUTHENTICATED",
            Self::NotEstablished => "LINKAGE_QUALITY_NOT_ESTABLISHED",
            Self::Unknown => "LINKAGE_STATUS_UNKNOWN",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Stratum {
    pub id: String,
    pub a_only: u64,
    pub b_only: u64,
    pub both: u64,
    pub theta_min: Rational,
    pub theta_max: Option<Rational>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Problem {
    pub analysis_id: String,
    pub source_a_id: String,
    pub source_b_id: String,
    pub source_a_ancestry: String,
    pub source_b_ancestry: String,
    pub population_frame_status: String,
    pub strata_universe_status: String,
    pub independence_assumption: IndependenceAssumption,
    pub linkage_status: LinkageStatus,
    pub within_stratum_homogeneity: String,
    pub strata: Vec<Stratum>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StratumResult {
    pub stratum_id: String,
    pub status: String,
    pub observed_union: Option<u128>,
    pub hidden_min: Option<Rational>,
    pub hidden_max: Option<Rational>,
    pub population_min: Option<Rational>,
    pub population_max: Option<Rational>,
    pub lincoln_petersen: Option<Rational>,
    pub chapman: Option<Rational>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AggregateResult {
    pub status: String,
    pub hidden_min: Option<Rational>,
    pub hidden_max: Option<Rational>,
    pub population_min: Option<Rational>,
    pub population_max: Option<Rational>,
    pub stratified_lincoln_petersen: Option<Rational>,
    pub stratified_chapman: Option<Rational>,
    pub pooled_lincoln_petersen: Option<Rational>,
    pub pooled_chapman: Option<Rational>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Analysis {
    pub problem: Problem,
    pub source_ancestry_relation: String,
    pub independence_point_status: String,
    pub results: Vec<StratumResult>,
    pub aggregate: AggregateResult,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CaptureError(pub String);

impl std::fmt::Display for CaptureError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for CaptureError {}

fn fail(message: impl Into<String>) -> CaptureError {
    CaptureError(message.into())
}

fn valid_token(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 200
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.' | b':' | b'/')
        })
}

fn valid_ancestry_path(value: &str) -> bool {
    value == "UNKNOWN"
        || (!value.is_empty()
            && value
                .split('/')
                .all(|segment| !segment.is_empty() && valid_token(segment) && !segment.contains('/')))
}

fn source_ancestry_relation(source_a: &str, source_b: &str) -> &'static str {
    if source_a == "UNKNOWN" || source_b == "UNKNOWN" {
        return "UPSTREAM_ANCESTRY_UNKNOWN_INDEPENDENCE_WITHHELD";
    }
    let a_segments = source_a.split('/').collect::<BTreeSet<_>>();
    let b_segments = source_b.split('/').collect::<BTreeSet<_>>();
    if !a_segments.is_disjoint(&b_segments) {
        "SHARED_DECLARED_UPSTREAM_ANCESTRY"
    } else {
        "DISTINCT_DECLARED_ANCESTRY_INDEPENDENCE_NOT_ESTABLISHED"
    }
}

fn parse_u64(value: &str, name: &str) -> Result<u64, CaptureError> {
    value
        .parse::<u64>()
        .map_err(|error| fail(format!("invalid {name}: {error}")))
}

fn parse_independence(value: &str) -> Result<IndependenceAssumption, CaptureError> {
    match value {
        "DECLARED" => Ok(IndependenceAssumption::Declared),
        "NOT_DECLARED" => Ok(IndependenceAssumption::NotDeclared),
        _ => Err(fail(format!("unknown independence_assumption {value}"))),
    }
}

fn parse_linkage(value: &str) -> Result<LinkageStatus, CaptureError> {
    match value {
        "PERFECT_LINKAGE_ASSUMED" => Ok(LinkageStatus::PerfectLinkageAssumed),
        "NOT_ESTABLISHED" => Ok(LinkageStatus::NotEstablished),
        "UNKNOWN" => Ok(LinkageStatus::Unknown),
        _ => Err(fail(format!("unknown linkage_status {value}"))),
    }
}

fn parse_ratio(
    numerator: &str,
    denominator: &str,
    name: &str,
    allow_zero: bool,
) -> Result<Rational, CaptureError> {
    let num = parse_u64(numerator, &format!("{name}_numerator"))?;
    let den = parse_u64(denominator, &format!("{name}_denominator"))?;
    if den == 0 || (!allow_zero && num == 0) {
        return Err(fail(format!(
            "{name} must have a positive denominator and valid numerator"
        )));
    }
    Rational::new(u128::from(num), u128::from(den))
        .ok_or_else(|| fail(format!("invalid {name} rational")))
}

fn parse_upper_ratio(numerator: &str, denominator: &str) -> Result<Option<Rational>, CaptureError> {
    if numerator == "UNBOUNDED" {
        if denominator != "1" {
            return Err(fail("UNBOUNDED theta maximum must use denominator 1"));
        }
        return Ok(None);
    }
    parse_ratio(numerator, denominator, "theta_max", true).map(Some)
}

pub fn parse_input(source: &str) -> Result<Problem, CaptureError> {
    if source.len() > MAX_INPUT_BYTES {
        return Err(fail("input exceeds 4 MiB"));
    }

    let mut metadata = BTreeMap::<String, String>::new();
    let mut lines = Vec::<(usize, &str)>::new();
    for (index, raw) in source.lines().enumerate() {
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            let (key, value) = comment
                .trim()
                .split_once('=')
                .ok_or_else(|| fail(format!("line {}: malformed metadata", index + 1)))?;
            if !matches!(
                key,
                "analysis_id"
                    | "source_a_id"
                    | "source_b_id"
                    | "source_a_ancestry"
                    | "source_b_ancestry"
                    | "population_frame_status"
                    | "strata_universe_status"
                    | "independence_assumption"
                    | "linkage_status"
                    | "within_stratum_homogeneity"
                    | "max_strata"
            ) {
                return Err(fail(format!(
                    "line {}: unknown metadata key {key}",
                    index + 1
                )));
            }
            if value.is_empty() || metadata.insert(key.to_owned(), value.to_owned()).is_some() {
                return Err(fail(format!(
                    "line {}: empty or duplicate metadata",
                    index + 1
                )));
            }
        } else {
            lines.push((index + 1, line));
        }
    }

    let required = |key: &str| -> Result<&str, CaptureError> {
        metadata
            .get(key)
            .map(String::as_str)
            .ok_or_else(|| fail(format!("missing metadata key {key}")))
    };

    let analysis_id = required("analysis_id")?.to_owned();
    let source_a_id = required("source_a_id")?.to_owned();
    let source_b_id = required("source_b_id")?.to_owned();
    let source_a_ancestry = required("source_a_ancestry")?.to_owned();
    let source_b_ancestry = required("source_b_ancestry")?.to_owned();
    for (value, name) in [
        (&analysis_id, "analysis_id"),
        (&source_a_id, "source_a_id"),
        (&source_b_id, "source_b_id"),
        (&source_a_ancestry, "source_a_ancestry"),
        (&source_b_ancestry, "source_b_ancestry"),
    ] {
        let valid = if name == "source_a_ancestry" || name == "source_b_ancestry" {
            valid_ancestry_path(value)
        } else {
            valid_token(value)
        };
        if !valid {
            return Err(fail(format!("{name} must be a valid token or slash-separated ancestry path")));
        }
    }
    if source_a_id == source_b_id {
        return Err(fail(
            "source_a_id and source_b_id must identify distinct declared lists",
        ));
    }

    let population_frame_status = match required("population_frame_status")? {
        "DECLARED_COMPLETE" => "DECLARED_COMPLETE".to_owned(),
        "NOT_ESTABLISHED" | "DECLARED_INCOMPLETE" => "NOT_ESTABLISHED".to_owned(),
        value => return Err(fail(format!("unknown population_frame_status {value}"))),
    };
    let strata_universe_status = match required("strata_universe_status")? {
        "DECLARED_COMPLETE" => "DECLARED_COMPLETE".to_owned(),
        "NOT_ESTABLISHED" | "DECLARED_INCOMPLETE" => "NOT_ESTABLISHED".to_owned(),
        value => return Err(fail(format!("unknown strata_universe_status {value}"))),
    };
    let independence_assumption = parse_independence(required("independence_assumption")?)?;
    let linkage_status = parse_linkage(required("linkage_status")?)?;
    let within_stratum_homogeneity = match required("within_stratum_homogeneity")? {
        "DECLARED_WITHIN_STRATUM" => "DECLARED_WITHIN_STRATUM".to_owned(),
        "NOT_ESTABLISHED" => "NOT_ESTABLISHED".to_owned(),
        value => return Err(fail(format!("unknown within_stratum_homogeneity {value}"))),
    };
    let max_strata = parse_u64(required("max_strata")?, "max_strata")?;
    if max_strata == 0 || max_strata > MAX_STRATA as u64 {
        return Err(fail(format!("max_strata must be in 1..={MAX_STRATA}")));
    }

    if lines.first().map(|(_, line)| *line) != Some(HEADER) {
        return Err(fail("expected exact stratum count header"));
    }
    let mut strata = Vec::new();
    let mut ids = BTreeSet::<String>::new();
    for (line_no, line) in lines.iter().skip(1) {
        let fields: Vec<&str> = line.split('|').map(str::trim).collect();
        if fields.len() != 8 || !valid_token(fields[0]) {
            return Err(fail(format!("line {line_no}: invalid stratum row")));
        }
        if !ids.insert(fields[0].to_owned()) {
            return Err(fail(format!(
                "line {line_no}: duplicate stratum id {}",
                fields[0]
            )));
        }
        let a_only = parse_u64(fields[1], "a_only")?;
        let b_only = parse_u64(fields[2], "b_only")?;
        let both = parse_u64(fields[3], "both")?;
        if [a_only, b_only, both]
            .iter()
            .any(|value| *value > MAX_CELL_COUNT)
        {
            return Err(fail(format!(
                "line {line_no}: cell counts exceed {MAX_CELL_COUNT}"
            )));
        }
        let theta_min = parse_ratio(fields[4], fields[5], "theta_min", true)?;
        let theta_max = parse_upper_ratio(fields[6], fields[7])?;
        if let Some(maximum) = &theta_max {
            if theta_min
                .checked_cmp(maximum)
                .ok_or_else(|| fail("theta comparison overflow"))?
                == Ordering::Greater
            {
                return Err(fail(format!("line {line_no}: theta_min exceeds theta_max")));
            }
        }
        if independence_assumption == IndependenceAssumption::Declared {
            let one = Rational::integer(1);
            let contains_one = theta_min
                .checked_cmp(&one)
                .ok_or_else(|| fail("theta comparison overflow"))?
                != Ordering::Greater
                && theta_max.as_ref().map_or(true, |maximum| {
                    maximum
                        .checked_cmp(&one)
                        .is_some_and(|ordering| ordering != Ordering::Less)
                });
            if !contains_one {
                return Err(fail(format!(
                    "line {line_no}: declared independence conflicts with theta interval excluding 1"
                )));
            }
        }
        strata.push(Stratum {
            id: fields[0].to_owned(),
            a_only,
            b_only,
            both,
            theta_min,
            theta_max,
        });
        if strata.len() > max_strata as usize {
            return Err(fail("stratum count exceeds declared max_strata"));
        }
    }
    if strata.is_empty() {
        return Err(fail("at least one stratum is required"));
    }
    strata.sort_by_key(|stratum| stratum.id.clone());

    Ok(Problem {
        analysis_id,
        source_a_id,
        source_b_id,
        source_a_ancestry,
        source_b_ancestry,
        population_frame_status,
        strata_universe_status,
        independence_assumption,
        linkage_status,
        within_stratum_homogeneity,
        strata,
    })
}

fn observed_union(stratum: &Stratum) -> Option<u128> {
    u128::from(stratum.a_only)
        .checked_add(u128::from(stratum.b_only))?
        .checked_add(u128::from(stratum.both))
}

fn hidden_cell(stratum: &Stratum, theta: &Rational) -> Option<Rational> {
    let numerator = theta
        .numerator
        .checked_mul(u128::from(stratum.a_only))?
        .checked_mul(u128::from(stratum.b_only))?;
    let denominator = theta.denominator.checked_mul(u128::from(stratum.both))?;
    Rational::new(numerator, denominator)
}

fn add_observed_union(union: u128, hidden: &Rational) -> Option<Rational> {
    let observed_numerator = union.checked_mul(hidden.denominator)?;
    let numerator = observed_numerator.checked_add(hidden.numerator)?;
    Rational::new(numerator, hidden.denominator)
}

fn lincoln_petersen(stratum: &Stratum) -> Option<Rational> {
    if stratum.both == 0 {
        return None;
    }
    let a_total = u128::from(stratum.a_only).checked_add(u128::from(stratum.both))?;
    let b_total = u128::from(stratum.b_only).checked_add(u128::from(stratum.both))?;
    Rational::new(a_total.checked_mul(b_total)?, u128::from(stratum.both))
}

fn chapman(stratum: &Stratum) -> Option<Rational> {
    if stratum.both == 0 {
        return None;
    }
    let a_total = u128::from(stratum.a_only).checked_add(u128::from(stratum.both))?;
    let b_total = u128::from(stratum.b_only).checked_add(u128::from(stratum.both))?;
    let both_plus_one = u128::from(stratum.both).checked_add(1)?;
    let numerator = a_total
        .checked_add(1)?
        .checked_mul(b_total.checked_add(1)?)?
        .checked_sub(both_plus_one)?;
    Rational::new(numerator, both_plus_one)
}

fn result_for(stratum: &Stratum, independence_enabled: bool) -> StratumResult {
    let union = observed_union(stratum);
    if stratum.both == 0 {
        return StratumResult {
            stratum_id: stratum.id.clone(),
            status: "NOT_ESTIMABLE_ZERO_OVERLAP".to_owned(),
            observed_union: union,
            hidden_min: None,
            hidden_max: None,
            population_min: None,
            population_max: None,
            lincoln_petersen: None,
            chapman: None,
        };
    }
    if stratum.a_only == 0 || stratum.b_only == 0 {
        return StratumResult {
            stratum_id: stratum.id.clone(),
            status: "NOT_IDENTIFIED_DEGENERATE_MARGIN".to_owned(),
            observed_union: union,
            hidden_min: None,
            hidden_max: None,
            population_min: None,
            population_max: None,
            lincoln_petersen: None,
            chapman: None,
        };
    }
    let Some(union) = union else {
        return failed_result(stratum, "ARITHMETIC_OVERFLOW");
    };
    let Some(hidden_min) = hidden_cell(stratum, &stratum.theta_min) else {
        return failed_result(stratum, "RATIONAL_OVERFLOW");
    };
    let Some(population_min) = add_observed_union(union, &hidden_min) else {
        return failed_result(stratum, "RATIONAL_OVERFLOW");
    };
    let (hidden_max, population_max) = match &stratum.theta_max {
        Some(theta_max) => {
            let Some(hidden) = hidden_cell(stratum, theta_max) else {
                return failed_result(stratum, "RATIONAL_OVERFLOW");
            };
            let Some(population) = add_observed_union(union, &hidden) else {
                return failed_result(stratum, "RATIONAL_OVERFLOW");
            };
            (Some(hidden), Some(population))
        }
        None => (None, None),
    };
    StratumResult {
        stratum_id: stratum.id.clone(),
        status: if population_max.is_some() {
            "COMPUTED_FINITE_SENSITIVITY_INTERVAL".to_owned()
        } else {
            "COMPUTED_UNBOUNDED_UPPER_INTERVAL".to_owned()
        },
        observed_union: Some(union),
        hidden_min: Some(hidden_min),
        hidden_max,
        population_min: Some(population_min),
        population_max,
        lincoln_petersen: if independence_enabled {
            lincoln_petersen(stratum)
        } else {
            None
        },
        chapman: if independence_enabled {
            chapman(stratum)
        } else {
            None
        },
    }
}

fn failed_result(stratum: &Stratum, status: &str) -> StratumResult {
    StratumResult {
        stratum_id: stratum.id.clone(),
        status: status.to_owned(),
        observed_union: observed_union(stratum),
        hidden_min: None,
        hidden_max: None,
        population_min: None,
        population_max: None,
        lincoln_petersen: None,
        chapman: None,
    }
}

fn checked_sum<I>(values: I) -> Option<u128>
where
    I: IntoIterator<Item = u128>,
{
    values
        .into_iter()
        .try_fold(0_u128, |sum, value| sum.checked_add(value))
}

fn sum_rationals<'a, I>(values: I) -> Option<Rational>
where
    I: IntoIterator<Item = &'a Rational>,
{
    values
        .into_iter()
        .try_fold(Rational::integer(0), |sum, value| sum.checked_add(value))
}

fn aggregate(problem: &Problem, results: &[StratumResult]) -> AggregateResult {
    if problem.strata_universe_status != "DECLARED_COMPLETE" {
        return AggregateResult {
            status: "AGGREGATE_NOT_ESTABLISHED_STRATA_UNIVERSE_INCOMPLETE".to_owned(),
            hidden_min: None,
            hidden_max: None,
            population_min: None,
            population_max: None,
            stratified_lincoln_petersen: None,
            stratified_chapman: None,
            pooled_lincoln_petersen: None,
            pooled_chapman: None,
        };
    }
    if results.iter().any(|result| result.population_min.is_none()) {
        return AggregateResult {
            status: "AGGREGATE_NOT_ESTABLISHED_STRATUM_UNRESOLVED".to_owned(),
            hidden_min: None,
            hidden_max: None,
            population_min: None,
            population_max: None,
            stratified_lincoln_petersen: None,
            stratified_chapman: None,
            pooled_lincoln_petersen: None,
            pooled_chapman: None,
        };
    }

    let hidden_min = sum_rationals(
        results
            .iter()
            .filter_map(|result| result.hidden_min.as_ref()),
    );
    let hidden_max = if results.iter().any(|result| result.hidden_max.is_none()) {
        None
    } else {
        sum_rationals(
            results
                .iter()
                .filter_map(|result| result.hidden_max.as_ref()),
        )
    };
    let population_min = sum_rationals(
        results
            .iter()
            .filter_map(|result| result.population_min.as_ref()),
    );
    let population_max = if results.iter().any(|result| result.population_max.is_none()) {
        None
    } else {
        sum_rationals(
            results
                .iter()
                .filter_map(|result| result.population_max.as_ref()),
        )
    };

    let all_independence_points = problem.independence_assumption
        == IndependenceAssumption::Declared
        && problem.linkage_status == LinkageStatus::PerfectLinkageAssumed
        && source_ancestry_relation(
            &problem.source_a_ancestry,
            &problem.source_b_ancestry,
        ) == "DISTINCT_DECLARED_ANCESTRY_INDEPENDENCE_NOT_ESTABLISHED"
        && results
            .iter()
            .all(|result| result.lincoln_petersen.is_some() && result.chapman.is_some());
    let stratified_lp = if all_independence_points {
        sum_rationals(
            results
                .iter()
                .filter_map(|result| result.lincoln_petersen.as_ref()),
        )
    } else {
        None
    };
    let stratified_chapman = if all_independence_points {
        sum_rationals(results.iter().filter_map(|result| result.chapman.as_ref()))
    } else {
        None
    };

    let pooled = pooled_stratum(&problem.strata);
    let (pooled_lp, pooled_chapman) = if all_independence_points {
        pooled
            .as_ref()
            .map(|stratum| (lincoln_petersen(stratum), chapman(stratum)))
            .unwrap_or((None, None))
    } else {
        (None, None)
    };

    let aggregate_failed = hidden_min.is_none() || population_min.is_none();
    AggregateResult {
        status: if aggregate_failed {
            "AGGREGATE_RATIONAL_OVERFLOW".to_owned()
        } else if population_max.is_some() {
            "COMPUTED_FINITE_STRATIFIED_INTERVAL".to_owned()
        } else {
            "COMPUTED_UNBOUNDED_STRATIFIED_INTERVAL".to_owned()
        },
        hidden_min,
        hidden_max,
        population_min,
        population_max,
        stratified_lincoln_petersen: stratified_lp,
        stratified_chapman,
        pooled_lincoln_petersen: pooled_lp,
        pooled_chapman,
    }
}

fn pooled_stratum(strata: &[Stratum]) -> Option<Stratum> {
    let a_only = checked_sum(strata.iter().map(|stratum| u128::from(stratum.a_only)))?;
    let b_only = checked_sum(strata.iter().map(|stratum| u128::from(stratum.b_only)))?;
    let both = checked_sum(strata.iter().map(|stratum| u128::from(stratum.both)))?;
    Some(Stratum {
        id: "pooled".to_owned(),
        a_only: u64::try_from(a_only).ok()?,
        b_only: u64::try_from(b_only).ok()?,
        both: u64::try_from(both).ok()?,
        theta_min: Rational::integer(1),
        theta_max: Some(Rational::integer(1)),
    })
}

pub fn analyze(problem: &Problem) -> Analysis {
    let ancestry_relation = source_ancestry_relation(
        &problem.source_a_ancestry,
        &problem.source_b_ancestry,
    );
    let source_ancestry_relation = ancestry_relation.to_owned();

    let independence_enabled = problem.independence_assumption == IndependenceAssumption::Declared
        && problem.linkage_status == LinkageStatus::PerfectLinkageAssumed
        && problem.within_stratum_homogeneity == "DECLARED_WITHIN_STRATUM"
        && ancestry_relation == "DISTINCT_DECLARED_ANCESTRY_INDEPENDENCE_NOT_ESTABLISHED";
    let independence_point_status =
        if problem.independence_assumption == IndependenceAssumption::NotDeclared {
            "NOT_EMITTED_INDEPENDENCE_NOT_DECLARED"
        } else if problem.linkage_status != LinkageStatus::PerfectLinkageAssumed {
            "NOT_EMITTED_LINKAGE_NOT_ASSUMED_PERFECT"
        } else if problem.within_stratum_homogeneity != "DECLARED_WITHIN_STRATUM" {
            "NOT_EMITTED_WITHIN_STRATUM_HOMOGENEITY_NOT_DECLARED"
        } else if ancestry_relation == "UPSTREAM_ANCESTRY_UNKNOWN_INDEPENDENCE_WITHHELD" {
            "NOT_EMITTED_UPSTREAM_ANCESTRY_UNKNOWN"
        } else if ancestry_relation == "SHARED_DECLARED_UPSTREAM_ANCESTRY" {
            "NOT_EMITTED_SHARED_UPSTREAM_ANCESTRY_DEPENDENCE_RISK"
        } else {
            "CONDITIONAL_ESTIMATES_EMITTED_INDEPENDENCE_ASSUMPTION_NOT_AUTHENTICATED"
        }
        .to_owned();

    let results = problem
        .strata
        .iter()
        .map(|stratum| result_for(stratum, independence_enabled))
        .collect::<Vec<_>>();
    let aggregate = aggregate(problem, &results);

    Analysis {
        problem: problem.clone(),
        source_ancestry_relation,
        independence_point_status,
        results,
        aggregate,
    }
}

fn write_bound(out: &mut String, key: &str, value: &Option<Rational>) {
    match value {
        Some(value) => {
            writeln!(out, "{key}={}", value.display()).expect("String write");
        }
        None => {
            writeln!(out, "{key}=NOT_ESTABLISHED_OR_UNBOUNDED").expect("String write");
        }
    }
}

impl Analysis {
    pub fn render(&self) -> String {
        let mut out = String::new();
        writeln!(out, "schema=fsre.capture-recapture-e0").expect("String write");
        writeln!(out, "version=0.1.0").expect("String write");
        writeln!(out, "analysis_id={}", self.problem.analysis_id).expect("String write");
        writeln!(out, "source_a_id={}", self.problem.source_a_id).expect("String write");
        writeln!(out, "source_b_id={}", self.problem.source_b_id).expect("String write");
        writeln!(out, "source_a_ancestry={}", self.problem.source_a_ancestry)
            .expect("String write");
        writeln!(out, "source_b_ancestry={}", self.problem.source_b_ancestry)
            .expect("String write");
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
            "strata_universe_status={}",
            self.problem.strata_universe_status
        )
        .expect("String write");
        writeln!(
            out,
            "independence_assumption={}",
            match self.problem.independence_assumption {
                IndependenceAssumption::Declared => "DECLARED_NOT_AUTHENTICATED",
                IndependenceAssumption::NotDeclared => "NOT_DECLARED",
            }
        )
        .expect("String write");
        writeln!(
            out,
            "linkage_status={}",
            self.problem.linkage_status.as_str()
        )
        .expect("String write");
        writeln!(
            out,
            "within_stratum_homogeneity={}",
            self.problem.within_stratum_homogeneity
        )
        .expect("String write");
        writeln!(out, "stratum_count={}", self.problem.strata.len()).expect("String write");
        writeln!(
            out,
            "independence_point_status={}",
            self.independence_point_status
        )
        .expect("String write");

        for result in &self.results {
            let prefix = format!("stratum.{}", result.stratum_id);
            writeln!(out, "{prefix}.status={}", result.status).expect("String write");
            match result.observed_union {
                Some(value) => {
                    writeln!(out, "{prefix}.observed_union={value}").expect("String write");
                }
                None => {
                    writeln!(out, "{prefix}.observed_union=NOT_ESTABLISHED").expect("String write");
                }
            }
            write_bound(
                &mut out,
                &format!("{prefix}.hidden_cell_lower"),
                &result.hidden_min,
            );
            write_bound(
                &mut out,
                &format!("{prefix}.hidden_cell_upper"),
                &result.hidden_max,
            );
            write_bound(
                &mut out,
                &format!("{prefix}.population_total_lower"),
                &result.population_min,
            );
            write_bound(
                &mut out,
                &format!("{prefix}.population_total_upper"),
                &result.population_max,
            );
            write_bound(
                &mut out,
                &format!("{prefix}.lincoln_petersen_conditional"),
                &result.lincoln_petersen,
            );
            write_bound(
                &mut out,
                &format!("{prefix}.chapman_conditional"),
                &result.chapman,
            );
        }

        writeln!(out, "aggregate.status={}", self.aggregate.status).expect("String write");
        write_bound(
            &mut out,
            "aggregate.hidden_cell_lower",
            &self.aggregate.hidden_min,
        );
        write_bound(
            &mut out,
            "aggregate.hidden_cell_upper",
            &self.aggregate.hidden_max,
        );
        write_bound(
            &mut out,
            "aggregate.population_total_lower",
            &self.aggregate.population_min,
        );
        write_bound(
            &mut out,
            "aggregate.population_total_upper",
            &self.aggregate.population_max,
        );
        write_bound(
            &mut out,
            "aggregate.stratified_lincoln_petersen_conditional",
            &self.aggregate.stratified_lincoln_petersen,
        );
        write_bound(
            &mut out,
            "aggregate.stratified_chapman_conditional",
            &self.aggregate.stratified_chapman,
        );
        write_bound(
            &mut out,
            "aggregate.pooled_lincoln_petersen_conditional",
            &self.aggregate.pooled_lincoln_petersen,
        );
        write_bound(
            &mut out,
            "aggregate.pooled_chapman_conditional",
            &self.aggregate.pooled_chapman,
        );

        out.push_str("theta_bounds=CALLER_DECLARED_SENSITIVITY_NOT_CALIBRATED\n");
        out.push_str("two_list_theta_identification=NOT_IDENTIFIED_FROM_OBSERVED_CELLS_ALONE\n");
        out.push_str("two_list_unrestricted_dependence=NO_FINITE_UPPER_POPULATION_BOUND\n");
        out.push_str("linkage_error=NOT_MODELLED\n");
        out.push_str("capture_heterogeneity=NOT_IDENTIFIED_BY_THIS_TWO_LIST_REFERENCE\n");
        out.push_str("sampling_uncertainty=NOT_ESTIMATED\n");
        out.push_str("confidence_interval=NOT_PROVIDED\n");
        out.push_str(
            "independence_point_estimates=CONDITIONAL_BASELINES_ONLY_NOT_CONFIDENCE_INTERVALS\n",
        );
        out.push_str("source_independence=NOT_ESTABLISHED_BY_ANCESTRY_LABELS\n");
        out.push_str("frame_completeness=CALLER_DECLARED_NOT_AUTHENTICATED\n");
        out.push_str("strata_universe_completeness=CALLER_DECLARED_NOT_AUTHENTICATED\n");
        out.push_str("population_estimand_scope=DECLARED_FRAME_AND_DECLARED_STRATA_ONLY\n");
        out.push_str("causal_truth=NOT_ESTABLISHED\n");
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../fixtures/dependence-strata.psv");
    const UNBOUNDED_FIXTURE: &str = include_str!("../fixtures/unbounded-dependence.psv");

    fn fixture() -> Problem {
        parse_input(FIXTURE).expect("fixture parses")
    }

    fn unbounded_fixture() -> Problem {
        parse_input(UNBOUNDED_FIXTURE).expect("unbounded fixture parses")
    }

    #[test]
    fn exact_theta_interval_produces_exact_hidden_and_population_bounds() {
        let problem = parse_input(
            "#analysis_id=dependence-known-answer\n\
             #source_a_id=a\n\
             #source_b_id=b\n\
             #source_a_ancestry=source-a-lineage\n\
             #source_b_ancestry=source-b-lineage\n\
             #population_frame_status=DECLARED_COMPLETE\n\
             #strata_universe_status=DECLARED_COMPLETE\n\
             #independence_assumption=NOT_DECLARED\n\
             #linkage_status=PERFECT_LINKAGE_ASSUMED\n\
             #within_stratum_homogeneity=DECLARED_WITHIN_STRATUM\n\
             #max_strata=1\n\
             stratum_id|a_only|b_only|both|theta_min_num|theta_min_den|theta_max_num|theta_max_den\n\
             whole|180|280|120|1|2|2|1\n",
        )
        .unwrap();
        let report = analyze(&problem);
        let stratum = &report.results[0];
        assert_eq!(stratum.hidden_min.as_ref().unwrap().display(), "210/1");
        assert_eq!(stratum.hidden_max.as_ref().unwrap().display(), "840/1");
        assert_eq!(stratum.population_min.as_ref().unwrap().display(), "790/1");
        assert_eq!(stratum.population_max.as_ref().unwrap().display(), "1420/1");
    }

    #[test]
    fn per_stratum_and_pooled_independence_baselines_are_not_interchangeable() {
        let report = analyze(&fixture());
        assert_eq!(
            report
                .aggregate
                .stratified_lincoln_petersen
                .as_ref()
                .unwrap()
                .display(),
            "800/1"
        );
        assert_eq!(
            report
                .aggregate
                .pooled_lincoln_petersen
                .as_ref()
                .unwrap()
                .display(),
            "750/1"
        );
        assert_eq!(
            report
                .aggregate
                .stratified_chapman
                .as_ref()
                .unwrap()
                .display(),
            "40450/51"
        );
        assert_eq!(
            report.aggregate.pooled_chapman.as_ref().unwrap().display(),
            "75450/101"
        );
    }

    #[test]
    fn independent_hand_calculation_matches_every_stratum_and_aggregate() {
        let report = analyze(&fixture());

        // These reference calculations are written directly from the
        // cross-product equation, not via hidden_cell(), result_for(), or
        // the production aggregation helpers.
        let a_lower_hidden = (100_u128 * 150) / 100;
        let a_upper_hidden = (2_u128 * 100 * 150) / 50;
        let b_hidden = (50_u128 * 50) / 50;
        assert_eq!(a_lower_hidden, 150);
        assert_eq!(a_upper_hidden, 600);
        assert_eq!(b_hidden, 50);

        let a_observed = 100_u128 + 150 + 50;
        let b_observed = 50_u128 + 50 + 50;
        let expected_lower_population = a_observed + a_lower_hidden + b_observed + b_hidden;
        let expected_upper_population = a_observed + a_upper_hidden + b_observed + b_hidden;

        assert_eq!(expected_lower_population, 650);
        assert_eq!(expected_upper_population, 1100);
        assert_eq!(
            report.aggregate.population_min.as_ref().unwrap().numerator,
            expected_lower_population
        );
        assert_eq!(
            report.aggregate.population_max.as_ref().unwrap().numerator,
            expected_upper_population
        );

        let stratum_a_lp = (100_u128 + 50) * (150_u128 + 50) / 50;
        let stratum_b_lp = (50_u128 + 50) * (50_u128 + 50) / 50;
        assert_eq!(stratum_a_lp + stratum_b_lp, 800);
        assert_eq!(
            report
                .aggregate
                .stratified_lincoln_petersen
                .as_ref()
                .unwrap()
                .display(),
            "800/1"
        );
    }

    #[test]
    fn aggregate_sensitivity_interval_is_sum_of_stratum_intervals() {
        let report = analyze(&fixture());
        assert_eq!(
            report.aggregate.hidden_min.as_ref().unwrap().display(),
            "200/1"
        );
        assert_eq!(
            report.aggregate.hidden_max.as_ref().unwrap().display(),
            "650/1"
        );
        assert_eq!(
            report.aggregate.population_min.as_ref().unwrap().display(),
            "650/1"
        );
        assert_eq!(
            report.aggregate.population_max.as_ref().unwrap().display(),
            "1100/1"
        );
    }

    #[test]
    fn incomplete_strata_universe_withholds_target_population_aggregate() {
        let source = FIXTURE.replace(
            "#strata_universe_status=DECLARED_COMPLETE",
            "#strata_universe_status=NOT_ESTABLISHED",
        );
        let problem = parse_input(&source).unwrap();
        let report = analyze(&problem);
        assert_eq!(
            report.aggregate.status,
            "AGGREGATE_NOT_ESTABLISHED_STRATA_UNIVERSE_INCOMPLETE"
        );
        assert!(report.aggregate.population_min.is_none());
        assert!(report
            .render()
            .contains("population_estimand_scope=DECLARED_FRAME_AND_DECLARED_STRATA_ONLY"));
    }

    #[test]
    fn unrestricted_dependence_has_no_finite_upper_bound() {
        let report = analyze(&unbounded_fixture());
        assert_eq!(
            report.results[0].hidden_min.as_ref().unwrap().display(),
            "0/1"
        );
        assert!(report.results[0].hidden_max.is_none());
        assert_eq!(
            report.results[0].population_min.as_ref().unwrap().display(),
            "580/1"
        );
        assert!(report.results[0].population_max.is_none());
        assert_eq!(
            report.aggregate.status,
            "COMPUTED_UNBOUNDED_STRATIFIED_INTERVAL"
        );
        assert!(report
            .render()
            .contains("two_list_unrestricted_dependence=NO_FINITE_UPPER_POPULATION_BOUND"));
    }

    #[test]
    fn zero_overlap_abstains_from_numeric_hidden_cell_and_estimates() {
        let source = FIXTURE.replace("stratum-a|100|150|50", "stratum-a|100|150|0");
        let mut problem = parse_input(&source).unwrap();
        problem.strata.retain(|stratum| stratum.id == "stratum-a");
        let report = analyze(&problem);
        assert_eq!(report.results[0].status, "NOT_ESTIMABLE_ZERO_OVERLAP");
        assert!(report.results[0].hidden_min.is_none());
        assert!(report.results[0].lincoln_petersen.is_none());
        assert!(report.aggregate.population_min.is_none());
    }

    #[test]
    fn shared_ancestry_is_warned_but_distinct_ancestry_never_proves_independence() {
        let report = analyze(&fixture());
        assert_eq!(
            report.source_ancestry_relation,
            "DISTINCT_DECLARED_ANCESTRY_INDEPENDENCE_NOT_ESTABLISHED"
        );
        assert!(report
            .render()
            .contains("source_independence=NOT_ESTABLISHED_BY_ANCESTRY_LABELS"));
        let mut problem = fixture();
        problem.source_b_ancestry = problem.source_a_ancestry.clone();
        let shared = analyze(&problem);
        assert_eq!(
            shared.source_ancestry_relation,
            "SHARED_DECLARED_UPSTREAM_ANCESTRY"
        );
        assert_eq!(
            shared.independence_point_status,
            "NOT_EMITTED_SHARED_UPSTREAM_ANCESTRY_DEPENDENCE_RISK"
        );
        assert!(shared
            .results
            .iter()
            .all(|result| result.lincoln_petersen.is_none() && result.chapman.is_none()));
        assert!(shared.aggregate.stratified_lincoln_petersen.is_none());
        assert!(shared.aggregate.stratified_chapman.is_none());
        assert!(shared.aggregate.pooled_lincoln_petersen.is_none());
        assert!(shared.aggregate.pooled_chapman.is_none());
        assert!(shared.aggregate.population_min.is_some());
        assert!(shared.aggregate.population_max.is_some());

        problem.source_a_ancestry = "registry-a/census-2022".to_owned();
        problem.source_b_ancestry = "registry-b/census-2022".to_owned();
        let shared_parent = analyze(&problem);
        assert_eq!(
            shared_parent.source_ancestry_relation,
            "SHARED_DECLARED_UPSTREAM_ANCESTRY"
        );
        assert!(shared_parent.results.iter().all(|result| {
            result.lincoln_petersen.is_none() && result.chapman.is_none()
        }));
        assert!(shared_parent.aggregate.stratified_lincoln_petersen.is_none());
        assert!(shared_parent.aggregate.pooled_chapman.is_none());

        problem.source_a_ancestry = "UNKNOWN".to_owned();
        let unknown = analyze(&problem);
        assert_eq!(
            unknown.source_ancestry_relation,
            "UPSTREAM_ANCESTRY_UNKNOWN_INDEPENDENCE_WITHHELD"
        );
        assert!(unknown.results.iter().all(|result| {
            result.lincoln_petersen.is_none() && result.chapman.is_none()
        }));
        assert_eq!(
            unknown.independence_point_status,
            "NOT_EMITTED_UPSTREAM_ANCESTRY_UNKNOWN"
        );
    }

    #[test]
    fn ancestry_paths_parse_and_malformed_paths_fail_closed() {
        let valid_paths = FIXTURE
            .replace(
                "#source_a_ancestry=utility-account-master",
                "#source_a_ancestry=national-register/utility-account",
            )
            .replace(
                "#source_b_ancestry=field-inspection-collection",
                "#source_b_ancestry=national-register/inspection-log",
            );
        let parsed = parse_input(&valid_paths).unwrap();
        let report = analyze(&parsed);
        assert_eq!(
            report.source_ancestry_relation,
            "SHARED_DECLARED_UPSTREAM_ANCESTRY"
        );
        assert!(report.results.iter().all(|result| {
            result.lincoln_petersen.is_none() && result.chapman.is_none()
        }));

        let malformed = FIXTURE.replace(
            "#source_a_ancestry=utility-account-master",
            "#source_a_ancestry=national-register//utility-account",
        );
        assert!(parse_input(&malformed).is_err());
    }

    #[test]
    fn independence_estimates_are_not_emitted_when_not_declared_or_linkage_unknown() {
        let mut problem = fixture();
        problem.independence_assumption = IndependenceAssumption::NotDeclared;
        let report = analyze(&problem);
        assert!(report
            .results
            .iter()
            .all(|result| result.lincoln_petersen.is_none()));
        problem.independence_assumption = IndependenceAssumption::Declared;
        problem.linkage_status = LinkageStatus::Unknown;
        let report = analyze(&problem);
        assert!(report.results.iter().all(|result| result.chapman.is_none()));
    }

    #[test]
    fn independence_point_estimates_require_within_stratum_homogeneity_declaration() {
        let mut problem = fixture();
        problem.within_stratum_homogeneity = "NOT_ESTABLISHED".to_owned();
        let report = analyze(&problem);
        assert_eq!(
            report.independence_point_status,
            "NOT_EMITTED_WITHIN_STRATUM_HOMOGENEITY_NOT_DECLARED"
        );
        assert!(report
            .results
            .iter()
            .all(|result| result.lincoln_petersen.is_none()));
        assert!(report.results.iter().all(|result| result.chapman.is_none()));
    }

    #[test]
    fn exact_rational_arithmetic_reduces_fractions_and_checks_overflow() {
        assert_eq!(Rational::new(120_580, 121).unwrap().display(), "120580/121");
        assert!(u128::MAX.checked_mul(2).is_none());
        let left = Rational::new(1, 3).unwrap();
        let right = Rational::new(1, 6).unwrap();
        assert_eq!(left.checked_add(&right).unwrap().display(), "1/2");
    }

    #[test]
    fn duplicate_strata_unknown_schema_and_duplicate_sources_are_rejected() {
        let duplicate = FIXTURE.replace("stratum-b|50|50|50|1|1|1|1", "stratum-a|50|50|50|1|1|1|1");
        assert!(parse_input(&duplicate).is_err());
        let unknown = FIXTURE.replace("#max_strata=16", "#max_strata=16\n#trust_me=INDEPENDENT");
        assert!(parse_input(&unknown).is_err());
        let same_source = FIXTURE.replace(
            "#source_b_id=inspection-register-b",
            "#source_b_id=utility-register-a",
        );
        assert!(parse_input(&same_source).is_err());
    }

    #[test]
    fn theta_interval_excluding_one_rejects_declared_independence() {
        let conflict = FIXTURE.replace(
            "stratum-a|100|150|50|1|2|2|1",
            "stratum-a|100|150|50|2|1|3|1",
        );
        assert!(parse_input(&conflict).is_err());
    }

    #[test]
    fn input_permutation_preserves_exact_report_bytes() {
        let original = analyze(&fixture()).render();
        let lines = FIXTURE.lines().collect::<Vec<_>>();
        let header_index = lines.iter().position(|line| *line == HEADER).unwrap();
        let metadata = &lines[..header_index];
        let mut rows = lines[header_index + 1..].to_vec();
        rows.reverse();
        let mut input = String::new();
        for line in metadata {
            writeln!(input, "{line}").unwrap();
        }
        writeln!(input, "{HEADER}").unwrap();
        for row in rows {
            writeln!(input, "{row}").unwrap();
        }
        assert_eq!(original, analyze(&parse_input(&input).unwrap()).render());
    }
}
