#![forbid(unsafe_code)]

// Exact E0 three-list capture-recapture sensitivity reference.
//
// Seven observed binary capture patterns are supplied. The all-zero pattern
// is unobserved. Its count is bounded through an explicit three-way
// cross-product interaction interval. The no-three-way-interaction estimate
// is a conditional baseline, not a confidence interval or authenticated truth.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

const HEADER: &str = "capture_pattern|count";
const PATTERNS: [&str; 7] = ["001", "010", "011", "100", "101", "110", "111"];
const MAX_INPUT_BYTES: usize = 1024 * 1024;
const MAX_CELL_COUNT: u64 = 1_000_000_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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

    pub fn zero() -> Self {
        Self {
            numerator: 0,
            denominator: 1,
        }
    }

    pub fn one() -> Self {
        Self {
            numerator: 1,
            denominator: 1,
        }
    }

    pub fn integer(value: u128) -> Self {
        Self {
            numerator: value,
            denominator: 1,
        }
    }

    pub fn parse(value: &str, field: &str) -> Result<Self, ThreeListError> {
        if let Some((numerator, denominator)) = value.trim().split_once('/') {
            let numerator = numerator
                .trim()
                .parse::<u128>()
                .map_err(|error| fail(format!("invalid {field} numerator: {error}")))?;
            let denominator = denominator
                .trim()
                .parse::<u128>()
                .map_err(|error| fail(format!("invalid {field} denominator: {error}")))?;
            Self::new(numerator, denominator)
                .ok_or_else(|| fail(format!("{field} denominator must be positive")))
        } else {
            let numerator = value
                .trim()
                .parse::<u128>()
                .map_err(|error| fail(format!("invalid {field}: {error}")))?;
            Ok(Self::integer(numerator))
        }
    }

    pub fn is_positive(self) -> bool {
        self.numerator > 0
    }

    pub fn checked_add(self, other: Self) -> Option<Self> {
        let common = gcd(self.denominator, other.denominator);
        let left_scale = other.denominator / common;
        let right_scale = self.denominator / common;
        let left = self.numerator.checked_mul(left_scale)?;
        let right = other.numerator.checked_mul(right_scale)?;
        let numerator = left.checked_add(right)?;
        let denominator = self.denominator.checked_mul(left_scale)?;
        Self::new(numerator, denominator)
    }

    pub fn checked_mul(self, other: Self) -> Option<Self> {
        let left_cancel = gcd(self.numerator, other.denominator);
        let right_cancel = gcd(other.numerator, self.denominator);
        let numerator = (self.numerator / left_cancel)
            .checked_mul(other.numerator / right_cancel)?;
        let denominator = (self.denominator / right_cancel)
            .checked_mul(other.denominator / left_cancel)?;
        Self::new(numerator, denominator)
    }

    pub fn checked_div(self, other: Self) -> Option<Self> {
        if other.numerator == 0 {
            return None;
        }
        let reciprocal = Self::new(other.denominator, other.numerator)?;
        self.checked_mul(reciprocal)
    }

    pub fn checked_cmp(self, other: Self) -> Option<Ordering> {
        let common_num = gcd(self.numerator, other.numerator);
        let common_den = gcd(self.denominator, other.denominator);
        let left = (self.numerator / common_num)
            .checked_mul(other.denominator / common_den)?;
        let right = (other.numerator / common_num)
            .checked_mul(self.denominator / common_den)?;
        Some(left.cmp(&right))
    }

    pub fn display(self) -> String {
        if self.denominator == 1 {
            self.numerator.to_string()
        } else {
            format!("{}/{}", self.numerator, self.denominator)
        }
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

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Problem {
    pub analysis_id: String,
    pub assumption_set_id: String,
    pub source_ids: [String; 3],
    pub source_ancestry: [String; 3],
    pub population_frame_status: String,
    pub pattern_table_status: String,
    pub three_way_interaction_assumption: String,
    pub theta_abc_min: Rational,
    pub theta_abc_max: Option<Rational>,
    pub counts: BTreeMap<String, u64>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Analysis {
    pub problem: Problem,
    pub status: String,
    pub source_ancestry_relation: String,
    pub observed_union: Option<u128>,
    pub zero_patterns: Vec<String>,
    pub cross_product_ratio: Option<Rational>,
    pub hidden_min: Option<Rational>,
    pub hidden_max: Option<Rational>,
    pub population_min: Option<Rational>,
    pub population_max: Option<Rational>,
    pub no_three_way_hidden_baseline: Option<Rational>,
    pub no_three_way_population_baseline: Option<Rational>,
    pub baseline_status: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ThreeListError(pub String);

impl std::fmt::Display for ThreeListError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for ThreeListError {}

fn fail(message: impl Into<String>) -> ThreeListError {
    ThreeListError(message.into())
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
) -> Result<&'a str, ThreeListError> {
    metadata
        .get(key)
        .map(String::as_str)
        .ok_or_else(|| fail(format!("missing metadata key {key}")))
}

fn parse_theta(value: &str, field: &str) -> Result<Option<Rational>, ThreeListError> {
    if value == "UNBOUNDED" {
        return Ok(None);
    }
    let rational = Rational::parse(value, field)?;
    if !rational.is_positive() {
        return Err(fail(format!("{field} must be strictly positive")));
    }
    Ok(Some(rational))
}

pub fn parse_input(source: &str) -> Result<Problem, ThreeListError> {
    if source.len() > MAX_INPUT_BYTES {
        return Err(fail("input exceeds 1 MiB"));
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
        "three_way_interaction_assumption",
        "theta_abc_min",
        "theta_abc_max",
    ];
    let mut metadata = BTreeMap::<String, String>::new();
    let mut rows = Vec::<(usize, String)>::new();

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
            let key = key.trim().to_owned();
            let value = value.trim().to_owned();
            if !allowed_metadata.contains(&key.as_str()) {
                return Err(fail(format!("line {}: unknown metadata key {key}", index + 1)));
            }
            if value.is_empty() || metadata.insert(key.clone(), value).is_some() {
                return Err(fail(format!("line {}: empty or duplicate metadata", index + 1)));
            }
        } else {
            rows.push((index + 1, line.to_owned()));
        }
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
        return Err(fail("source ancestry must be UNKNOWN or a valid slash-separated path"));
    }

    let population_frame_status = match required(&metadata, "population_frame_status")? {
        "DECLARED_COMPLETE" => "DECLARED_COMPLETE",
        "NOT_ESTABLISHED" | "DECLARED_INCOMPLETE" => "NOT_ESTABLISHED",
        value => return Err(fail(format!("unknown population_frame_status {value}"))),
    }
    .to_owned();
    let pattern_table_status = match required(&metadata, "pattern_table_status")? {
        "DECLARED_COMPLETE" => "DECLARED_COMPLETE",
        "NOT_ESTABLISHED" | "DECLARED_INCOMPLETE" => "NOT_ESTABLISHED",
        value => return Err(fail(format!("unknown pattern_table_status {value}"))),
    }
    .to_owned();
    let three_way_interaction_assumption =
        match required(&metadata, "three_way_interaction_assumption")? {
            "DECLARED_NO_THREE_WAY_INTERACTION" => "DECLARED_NO_THREE_WAY_INTERACTION",
            "NOT_DECLARED" => "NOT_DECLARED",
            value => return Err(fail(format!("unknown three_way_interaction_assumption {value}"))),
        }
        .to_owned();

    let theta_abc_min = Rational::parse(required(&metadata, "theta_abc_min")?, "theta_abc_min")?;
    if !theta_abc_min.is_positive() {
        return Err(fail("theta_abc_min must be strictly positive"));
    }
    let theta_abc_max = parse_theta(required(&metadata, "theta_abc_max")?, "theta_abc_max")?;
    if let Some(maximum) = theta_abc_max {
        if theta_abc_min
            .checked_cmp(maximum)
            .ok_or_else(|| fail("theta interval comparison overflow"))?
            == Ordering::Greater
        {
            return Err(fail("theta_abc_min exceeds theta_abc_max"));
        }
    }
    if three_way_interaction_assumption == "DECLARED_NO_THREE_WAY_INTERACTION" {
        let one = Rational::one();
        if theta_abc_min
            .checked_cmp(one)
            .map_or(true, |ordering| ordering == Ordering::Greater)
            || theta_abc_max.is_some_and(|maximum| {
                maximum
                    .checked_cmp(one)
                    .map_or(true, |ordering| ordering == Ordering::Less)
            })
        {
            return Err(fail(
                "declared no-three-way-interaction conflicts with theta interval excluding 1",
            ));
        }
    }

    if rows.first().map(|(_, line)| line.as_str()) != Some(HEADER) {
        return Err(fail("expected exact capture pattern header"));
    }
    let mut counts = BTreeMap::<String, u64>::new();
    for (line_no, line) in rows.iter().skip(1) {
        let fields = line.split('|').map(str::trim).collect::<Vec<_>>();
        if fields.len() != 2 || !PATTERNS.contains(&fields[0]) {
            return Err(fail(format!("line {line_no}: invalid or unknown capture pattern")));
        }
        let count = fields[1]
            .parse::<u64>()
            .map_err(|error| fail(format!("line {line_no}: invalid count: {error}")))?;
        if count > MAX_CELL_COUNT {
            return Err(fail(format!("line {line_no}: count exceeds {MAX_CELL_COUNT}")));
        }
        if counts.insert(fields[0].to_owned(), count).is_some() {
            return Err(fail(format!(
                "line {line_no}: duplicate capture pattern {}",
                fields[0]
            )));
        }
    }
    if counts.len() != PATTERNS.len() {
        return Err(fail("all seven observed capture patterns must be present exactly once"));
    }

    Ok(Problem {
        analysis_id,
        assumption_set_id,
        source_ids,
        source_ancestry,
        population_frame_status,
        pattern_table_status,
        three_way_interaction_assumption,
        theta_abc_min,
        theta_abc_max,
        counts,
    })
}

fn cross_product_ratio(problem: &Problem) -> Option<Rational> {
    let n100 = u128::from(problem.counts["100"]);
    let n010 = u128::from(problem.counts["010"]);
    let n001 = u128::from(problem.counts["001"]);
    let n110 = u128::from(problem.counts["110"]);
    let n101 = u128::from(problem.counts["101"]);
    let n011 = u128::from(problem.counts["011"]);
    let n111 = u128::from(problem.counts["111"]);

    let numerator = n111
        .checked_mul(n100)?
        .checked_mul(n010)?
        .checked_mul(n001)?;
    let denominator = n110.checked_mul(n101)?.checked_mul(n011)?;
    Rational::new(numerator, denominator)
}

fn observed_union(problem: &Problem) -> Option<u128> {
    problem
        .counts
        .values()
        .try_fold(0_u128, |sum, count| sum.checked_add(u128::from(*count)))
}

fn sum_hidden_and_observed(union: u128, hidden: Rational) -> Option<Rational> {
    Rational::integer(union).checked_add(hidden)
}

pub fn analyze(problem: &Problem) -> Analysis {
    let source_ancestry_relation = ancestry_relation(&problem.source_ancestry).to_owned();
    let union = observed_union(problem);
    let zero_patterns = PATTERNS
        .iter()
        .filter(|pattern| problem.counts[**pattern] == 0)
        .map(|pattern| (*pattern).to_owned())
        .collect::<Vec<_>>();

    let baseline_blocker = if problem.three_way_interaction_assumption != "DECLARED_NO_THREE_WAY_INTERACTION" {
        "NOT_EMITTED_NO_THREE_WAY_INTERACTION_NOT_DECLARED"
    } else if source_ancestry_relation == "UPSTREAM_ANCESTRY_UNKNOWN_INDEPENDENCE_WITHHELD" {
        "NOT_EMITTED_UPSTREAM_ANCESTRY_UNKNOWN"
    } else if source_ancestry_relation == "SHARED_DECLARED_UPSTREAM_ANCESTRY" {
        "NOT_EMITTED_SHARED_UPSTREAM_ANCESTRY_DEPENDENCE_RISK"
    } else if problem.population_frame_status != "DECLARED_COMPLETE" {
        "NOT_EMITTED_POPULATION_FRAME_NOT_ESTABLISHED"
    } else {
        "CONDITIONAL_BASELINE_ELIGIBLE"
    };

    if problem.pattern_table_status != "DECLARED_COMPLETE" {
        return Analysis {
            problem: problem.clone(),
            status: "NOT_ESTABLISHED_PATTERN_TABLE_COMPLETENESS".to_owned(),
            source_ancestry_relation,
            observed_union: union,
            zero_patterns,
            cross_product_ratio: None,
            hidden_min: None,
            hidden_max: None,
            population_min: None,
            population_max: None,
            no_three_way_hidden_baseline: None,
            no_three_way_population_baseline: None,
            baseline_status: "NOT_EMITTED_PATTERN_TABLE_NOT_ESTABLISHED".to_owned(),
        };
    }

    if !zero_patterns.is_empty() {
        return Analysis {
            problem: problem.clone(),
            status: "NOT_IDENTIFIED_ZERO_OR_SPARSE_PATTERN".to_owned(),
            source_ancestry_relation,
            observed_union: union,
            zero_patterns,
            cross_product_ratio: None,
            hidden_min: None,
            hidden_max: None,
            population_min: None,
            population_max: None,
            no_three_way_hidden_baseline: None,
            no_three_way_population_baseline: None,
            baseline_status: "NOT_EMITTED_ZERO_OR_SPARSE_PATTERN".to_owned(),
        };
    }

    let Some(union_value) = union else {
        return failed_analysis(problem, "ARITHMETIC_OVERFLOW", source_ancestry_relation);
    };
    let Some(ratio) = cross_product_ratio(problem) else {
        return failed_analysis(problem, "CROSS_PRODUCT_OVERFLOW", source_ancestry_relation);
    };

    let hidden_min = match problem.theta_abc_max {
        Some(maximum) => ratio.checked_div(maximum),
        None => Some(Rational::zero()),
    };
    let Some(hidden_min) = hidden_min else {
        return failed_analysis(problem, "HIDDEN_LOWER_BOUND_OVERFLOW", source_ancestry_relation);
    };
    let Some(hidden_max) = ratio.checked_div(problem.theta_abc_min) else {
        return failed_analysis(problem, "HIDDEN_UPPER_BOUND_OVERFLOW", source_ancestry_relation);
    };

    let (population_min, population_max) = if problem.population_frame_status == "DECLARED_COMPLETE" {
        (
            sum_hidden_and_observed(union_value, hidden_min),
            sum_hidden_and_observed(union_value, hidden_max),
        )
    } else {
        (None, None)
    };

    let baseline_allowed = baseline_blocker == "CONDITIONAL_BASELINE_ELIGIBLE"
        && problem.theta_abc_min.checked_cmp(Rational::one()).is_some_and(|order| {
            order != Ordering::Greater
        })
        && problem.theta_abc_max.map_or(true, |maximum| {
            maximum
                .checked_cmp(Rational::one())
                .is_some_and(|order| order != Ordering::Less)
        });

    let no_three_way_hidden_baseline = baseline_allowed.then_some(ratio);
    let no_three_way_population_baseline = if baseline_allowed {
        sum_hidden_and_observed(union_value, ratio)
    } else {
        None
    };
    let baseline_status = if baseline_allowed {
        "CONDITIONAL_NO_THREE_WAY_INTERACTION_BASELINE_EMITTED"
    } else {
        baseline_blocker
    };

    Analysis {
        problem: problem.clone(),
        status: if problem.population_frame_status == "DECLARED_COMPLETE" {
            "COMPUTED_FINITE_THREE_LIST_SENSITIVITY_INTERVAL".to_owned()
        } else {
            "COMPUTED_WITHIN_DECLARED_FRAME_ONLY_TARGET_TOTAL_WITHHELD".to_owned()
        },
        source_ancestry_relation,
        observed_union: Some(union_value),
        zero_patterns,
        cross_product_ratio: Some(ratio),
        hidden_min: Some(hidden_min),
        hidden_max: Some(hidden_max),
        population_min,
        population_max,
        no_three_way_hidden_baseline,
        no_three_way_population_baseline,
        baseline_status: baseline_status.to_owned(),
    }
}

fn failed_analysis(problem: &Problem, status: &str, ancestry: String) -> Analysis {
    Analysis {
        problem: problem.clone(),
        status: status.to_owned(),
        source_ancestry_relation: ancestry,
        observed_union: observed_union(problem),
        zero_patterns: Vec::new(),
        cross_product_ratio: None,
        hidden_min: None,
        hidden_max: None,
        population_min: None,
        population_max: None,
        no_three_way_hidden_baseline: None,
        no_three_way_population_baseline: None,
        baseline_status: "NOT_EMITTED_NUMERIC_FAILURE".to_owned(),
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
        writeln!(out, "schema=fsre.three-list-capture-e0").expect("String write");
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
            writeln!(
                out,
                "source_{label}_id={}",
                self.problem.source_ids[index]
            )
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
            "three_way_interaction_assumption={}",
            self.problem.three_way_interaction_assumption
        )
        .expect("String write");
        writeln!(
            out,
            "theta_abc_interval=[{},{}]",
            self.problem.theta_abc_min.display(),
            self.problem
                .theta_abc_max
                .map(Rational::display)
                .unwrap_or_else(|| "UNBOUNDED".to_owned())
        )
        .expect("String write");
        writeln!(out, "analysis_status={}", self.status).expect("String write");
        writeln!(
            out,
            "observed_union={}",
            self.observed_union
                .map(|value| value.to_string())
                .unwrap_or_else(|| "NOT_ESTABLISHED".to_owned())
        )
        .expect("String write");
        writeln!(out, "zero_pattern_count={}", self.zero_patterns.len())
            .expect("String write");
        writeln!(out, "zero_patterns={}", self.zero_patterns.join(","))
            .expect("String write");
        write_bound(&mut out, "three_way_cross_product_ratio", self.cross_product_ratio);
        write_bound(&mut out, "hidden_cell_lower", self.hidden_min);
        write_bound(&mut out, "hidden_cell_upper", self.hidden_max);
        write_bound(&mut out, "population_lower", self.population_min);
        write_bound(&mut out, "population_upper", self.population_max);
        write_bound(
            &mut out,
            "no_three_way_interaction_hidden_baseline",
            self.no_three_way_hidden_baseline,
        );
        write_bound(
            &mut out,
            "no_three_way_interaction_population_baseline",
            self.no_three_way_population_baseline,
        );
        writeln!(out, "baseline_status={}", self.baseline_status).expect("String write");
        out.push_str("three_way_theta_interval=DECLARED_SENSITIVITY_NOT_SAMPLING_CI\n");
        out.push_str("no_three_way_baseline=CONDITIONAL_MODEL_BASELINE_NOT_CONFIDENCE_INTERVAL\n");
        out.push_str("source_independence=NOT_ESTABLISHED_BY_ANCESTRY_LABELS\n");
        out.push_str("population_frame_authentication=NOT_ESTABLISHED_BY_E0\n");
        out.push_str("source_authentication=NOT_ESTABLISHED_BY_E0\n");
        out.push_str("causal_truth=NOT_ESTABLISHED_BY_E0\n");
        out.push_str("field_outcome=NOT_ESTABLISHED_BY_E0\n");
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../fixtures/known-answer.psv");

    fn fixture() -> Problem {
        parse_input(FIXTURE).expect("fixture parses")
    }

    #[test]
    fn exact_three_way_cross_product_ratio_produces_sharp_sensitivity_bounds() {
        let report = analyze(&fixture());
        assert_eq!(report.status, "COMPUTED_FINITE_THREE_LIST_SENSITIVITY_INTERVAL");
        assert_eq!(report.observed_union, Some(195));
        assert_eq!(report.cross_product_ratio.unwrap().display(), "240");
        assert_eq!(report.hidden_min.unwrap().display(), "120");
        assert_eq!(report.hidden_max.unwrap().display(), "480");
        assert_eq!(report.population_min.unwrap().display(), "315");
        assert_eq!(report.population_max.unwrap().display(), "675");
        assert_eq!(
            report.no_three_way_hidden_baseline.unwrap().display(),
            "240"
        );
        assert_eq!(
            report.no_three_way_population_baseline.unwrap().display(),
            "435"
        );
    }

    #[test]
    fn shared_ancestry_withholds_conditional_baseline_but_keeps_theta_bounds() {
        let mut problem = fixture();
        problem.source_ancestry[1] = "shared-register/inspection".to_owned();
        problem.source_ancestry[2] = "shared-register/benefits".to_owned();
        let report = analyze(&problem);
        assert_eq!(
            report.source_ancestry_relation,
            "SHARED_DECLARED_UPSTREAM_ANCESTRY"
        );
        assert!(report.no_three_way_hidden_baseline.is_none());
        assert!(report.no_three_way_population_baseline.is_none());
        assert_eq!(report.hidden_min.unwrap().display(), "120");
        assert_eq!(report.hidden_max.unwrap().display(), "480");
    }

    #[test]
    fn unknown_ancestry_withholds_conditional_baseline() {
        let mut problem = fixture();
        problem.source_ancestry[0] = "UNKNOWN".to_owned();
        let report = analyze(&problem);
        assert_eq!(
            report.source_ancestry_relation,
            "UPSTREAM_ANCESTRY_UNKNOWN_INDEPENDENCE_WITHHELD"
        );
        assert!(report.no_three_way_hidden_baseline.is_none());
        assert!(report.hidden_min.is_some());
        assert!(report.hidden_max.is_some());
    }

    #[test]
    fn incomplete_population_frame_preserves_within_frame_hidden_bounds_only() {
        let mut problem = fixture();
        problem.population_frame_status = "NOT_ESTABLISHED".to_owned();
        let report = analyze(&problem);
        assert!(report.hidden_min.is_some());
        assert!(report.hidden_max.is_some());
        assert!(report.population_min.is_none());
        assert!(report.population_max.is_none());
        assert!(report.no_three_way_population_baseline.is_none());
    }

    #[test]
    fn incomplete_pattern_table_withholds_all_numeric_results() {
        let mut problem = fixture();
        problem.pattern_table_status = "NOT_ESTABLISHED".to_owned();
        let report = analyze(&problem);
        assert_eq!(
            report.status,
            "NOT_ESTABLISHED_PATTERN_TABLE_COMPLETENESS"
        );
        assert!(report.cross_product_ratio.is_none());
        assert!(report.hidden_min.is_none());
        assert!(report.population_min.is_none());
    }

    #[test]
    fn zero_in_any_observed_pattern_withholds_numeric_ratio_and_bounds() {
        let mut problem = fixture();
        problem.counts.insert("101".to_owned(), 0);
        let report = analyze(&problem);
        assert_eq!(report.status, "NOT_IDENTIFIED_ZERO_OR_SPARSE_PATTERN");
        assert_eq!(report.zero_patterns, vec!["101"]);
        assert!(report.cross_product_ratio.is_none());
        assert!(report.hidden_min.is_none());
        assert!(report.population_max.is_none());
    }

    #[test]
    fn unbounded_theta_upper_yields_zero_hidden_lower_bound() {
        let mut problem = fixture();
        problem.theta_abc_max = None;
        let report = analyze(&problem);
        assert_eq!(report.hidden_min.unwrap().display(), "0");
        assert_eq!(report.hidden_max.unwrap().display(), "480");
    }

    #[test]
    fn malformed_theta_interval_and_declared_baseline_conflict_fail_closed() {
        let bad_order = FIXTURE.replace("#theta_abc_max=2", "#theta_abc_max=1/4");
        assert!(parse_input(&bad_order).is_err());
        let no_one = FIXTURE
            .replace("#theta_abc_min=1/2", "#theta_abc_min=2")
            .replace("#theta_abc_max=2", "#theta_abc_max=3");
        assert!(parse_input(&no_one).is_err());
        let zero_min = FIXTURE.replace("#theta_abc_min=1/2", "#theta_abc_min=0");
        assert!(parse_input(&zero_min).is_err());
    }

    #[test]
    fn source_ancestry_paths_detect_any_common_ancestor_and_reject_bad_segments() {
        let common = FIXTURE
            .replace("#source_a_ancestry=utility-master", "#source_a_ancestry=master/utility")
            .replace("#source_b_ancestry=inspection-collection", "#source_b_ancestry=master/inspection");
        let problem = parse_input(&common).unwrap();
        assert_eq!(
            ancestry_relation(&problem.source_ancestry),
            "SHARED_DECLARED_UPSTREAM_ANCESTRY"
        );
        let malformed = FIXTURE.replace(
            "#source_a_ancestry=utility-master",
            "#source_a_ancestry=master//utility",
        );
        assert!(parse_input(&malformed).is_err());
    }

    #[test]
    fn deterministic_under_pattern_row_permutation() {
        let original = analyze(&fixture()).render();
        let prefix = FIXTURE
            .lines()
            .take_while(|line| *line != HEADER)
            .collect::<Vec<_>>();
        let mut rows = FIXTURE
            .lines()
            .skip_while(|line| *line != HEADER)
            .skip(1)
            .collect::<Vec<_>>();
        rows.reverse();
        let mut input = format!("{}\n{}\n", prefix.join("\n"), HEADER);
        for row in rows {
            writeln!(input, "{row}").unwrap();
        }
        assert_eq!(original, analyze(&parse_input(&input).unwrap()).render());
    }

    #[test]
    fn rational_arithmetic_reduces_and_rejects_overflow() {
        assert_eq!(Rational::new(2, 4).unwrap().display(), "1/2");
        assert_eq!(
            Rational::new(1, 3)
                .unwrap()
                .checked_mul(Rational::new(9, 2).unwrap())
                .unwrap()
                .display(),
            "3/2"
        );
        assert!(Rational::new(u128::MAX, 1)
            .unwrap()
            .checked_mul(Rational::integer(2))
            .is_none());
    }
}
