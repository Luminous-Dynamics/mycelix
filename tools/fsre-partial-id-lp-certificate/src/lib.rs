#![forbid(unsafe_code)]

// Solver-independent E0 verifier for continuous sparse LP certificates.
// For max c^T x subject to A x <= b, x >= 0:
//   primal feasible x gives a lower bound;
//   dual feasible y (A^T y >= c, y >= 0) gives an upper bound;
//   equal exact objectives prove optimality for this declared LP.
//
// Exact duplicate-column aggregation is lossless for nonnegative continuous
// variables only. Integer/binary semantics are outside this crate's contract.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

const ROWS_HEADER: &str = "row_id|rhs";
const COLUMNS_HEADER: &str = "column_id|objective";
const COEFFICIENTS_HEADER: &str = "row_id|column_id|coefficient";
const PRIMAL_HEADER: &str = "column_id|value";
const DUAL_HEADER: &str = "row_id|value";
const MAX_INPUT_BYTES: usize = 8 * 1024 * 1024;
const MAX_ROWS: usize = 512;
const MAX_COLUMNS: usize = 512;
const MAX_NONZEROS: usize = 100_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Rational {
    pub numerator: i128,
    pub denominator: i128,
}

impl Rational {
    pub fn new(numerator: i128, denominator: i128) -> Result<Self, LpError> {
        if denominator == 0 {
            return Err(fail("rational denominator cannot be zero"));
        }
        let (numerator, denominator) = if denominator < 0 {
            (
                numerator
                    .checked_neg()
                    .ok_or_else(|| fail("rational sign normalization overflow"))?,
                denominator
                    .checked_neg()
                    .ok_or_else(|| fail("rational denominator normalization overflow"))?,
            )
        } else {
            (numerator, denominator)
        };
        let divisor = gcd(numerator.unsigned_abs(), denominator as u128);
        let divisor =
            i128::try_from(divisor).map_err(|_| fail("rational reduction divisor overflow"))?;
        Ok(Self {
            numerator: numerator / divisor,
            denominator: denominator / divisor,
        })
    }

    pub const fn zero() -> Self {
        Self {
            numerator: 0,
            denominator: 1,
        }
    }

    pub const fn one() -> Self {
        Self {
            numerator: 1,
            denominator: 1,
        }
    }

    pub fn parse(value: &str, field: &str) -> Result<Self, LpError> {
        let value = value.trim();
        if let Some((numerator, denominator)) = value.split_once('/') {
            let numerator = numerator
                .trim()
                .parse::<i128>()
                .map_err(|error| fail(format!("invalid {field} numerator: {error}")))?;
            let denominator = denominator
                .trim()
                .parse::<i128>()
                .map_err(|error| fail(format!("invalid {field} denominator: {error}")))?;
            Self::new(numerator, denominator)
        } else {
            let numerator = value
                .parse::<i128>()
                .map_err(|error| fail(format!("invalid {field}: {error}")))?;
            Ok(Self {
                numerator,
                denominator: 1,
            })
        }
    }

    pub fn checked_add(self, other: Self) -> Result<Self, LpError> {
        let left = self
            .numerator
            .checked_mul(other.denominator)
            .ok_or_else(|| fail("rational addition overflow"))?;
        let right = other
            .numerator
            .checked_mul(self.denominator)
            .ok_or_else(|| fail("rational addition overflow"))?;
        let numerator = left
            .checked_add(right)
            .ok_or_else(|| fail("rational addition overflow"))?;
        let denominator = self
            .denominator
            .checked_mul(other.denominator)
            .ok_or_else(|| fail("rational denominator overflow"))?;
        Self::new(numerator, denominator)
    }

    pub fn checked_neg(self) -> Result<Self, LpError> {
        let numerator = self
            .numerator
            .checked_neg()
            .ok_or_else(|| fail("rational negation overflow"))?;
        Self::new(numerator, self.denominator)
    }

    pub fn checked_mul(self, other: Self) -> Result<Self, LpError> {
        let numerator = self
            .numerator
            .checked_mul(other.numerator)
            .ok_or_else(|| fail("rational multiplication overflow"))?;
        let denominator = self
            .denominator
            .checked_mul(other.denominator)
            .ok_or_else(|| fail("rational denominator overflow"))?;
        Self::new(numerator, denominator)
    }

    pub fn checked_cmp(self, other: Self) -> Result<Ordering, LpError> {
        let left = self
            .numerator
            .checked_mul(other.denominator)
            .ok_or_else(|| fail("rational comparison overflow"))?;
        let right = other
            .numerator
            .checked_mul(self.denominator)
            .ok_or_else(|| fail("rational comparison overflow"))?;
        Ok(left.cmp(&right))
    }

    pub fn is_nonnegative(self) -> bool {
        self.numerator >= 0
    }

    pub fn display(self) -> String {
        if self.denominator == 1 {
            self.numerator.to_string()
        } else {
            format!("{}/{}", self.numerator, self.denominator)
        }
    }

    pub fn finite_decimal(self) -> Result<String, LpError> {
        if self.numerator == 0 {
            return Ok("0".to_owned());
        }
        let mut denominator = self.denominator;
        let mut twos = 0_u32;
        let mut fives = 0_u32;
        while denominator % 2 == 0 {
            denominator /= 2;
            twos += 1;
        }
        while denominator % 5 == 0 {
            denominator /= 5;
            fives += 1;
        }
        if denominator != 1 {
            return Err(fail(format!(
                "rational {} has no exact finite decimal representation",
                self.display()
            )));
        }
        let places = twos.max(fives);
        if places > 18 {
            return Err(fail("finite decimal requires more than 18 places"));
        }
        let scale = 10_i128
            .checked_pow(places)
            .ok_or_else(|| fail("finite decimal scale overflow"))?;
        let scaled = self
            .numerator
            .checked_mul(scale)
            .ok_or_else(|| fail("finite decimal numerator overflow"))?
            / self.denominator;
        let negative = scaled < 0;
        let magnitude = scaled.unsigned_abs();
        let scale_unsigned = scale as u128;
        let integer = magnitude / scale_unsigned;
        let fraction = magnitude % scale_unsigned;
        if places == 0 || fraction == 0 {
            return Ok(format!("{}{}", if negative { "-" } else { "" }, integer));
        }
        let mut fraction_text = format!("{:0width$}", fraction, width = places as usize);
        while fraction_text.ends_with('0') {
            fraction_text.pop();
        }
        Ok(format!(
            "{}{}.{}",
            if negative { "-" } else { "" },
            integer,
            fraction_text
        ))
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
pub struct LpRow {
    pub id: String,
    pub rhs: Rational,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LpColumn {
    pub id: String,
    pub objective: Rational,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LpModel {
    pub problem_id: String,
    pub scope_sha256: String,
    pub assumption_set_id: String,
    pub rows: Vec<LpRow>,
    pub columns: Vec<LpColumn>,
    pub coefficients: BTreeMap<(String, String), Rational>,
    pub primal: BTreeMap<String, Rational>,
    pub dual: BTreeMap<String, Rational>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ColumnClass {
    pub representative: String,
    pub members: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CertificateCheck {
    pub feasible: bool,
    pub objective: Rational,
    pub violation_count: usize,
    pub violations: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReductionCheck {
    pub original_column_count: usize,
    pub reduced_column_count: usize,
    pub equivalence_classes: Vec<ColumnClass>,
    pub primal_feasible_preserved: bool,
    pub dual_feasible_preserved: bool,
    pub primal_objective_preserved: bool,
    pub coefficient_projection_exact: bool,
    pub reduced_primal_objective: Rational,
    pub reduced_dual_objective: Rational,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FarkasCheck {
    pub valid: bool,
    pub contradiction_value: Rational,
    pub column_combinations: BTreeMap<String, Rational>,
    pub violations: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Analysis {
    pub model: LpModel,
    pub primal: CertificateCheck,
    pub dual: CertificateCheck,
    pub optimality_status: String,
    pub objective_lower_bound: Option<Rational>,
    pub objective_upper_bound: Option<Rational>,
    pub reduction: ReductionCheck,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExtremumDirection {
    Minimum,
    Maximum,
}

impl ExtremumDirection {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Minimum => "MINIMUM",
            Self::Maximum => "MAXIMUM",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FiniteWorldValue {
    pub world_id: String,
    pub value: Rational,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FiniteWorldExtremum {
    pub direction: ExtremumDirection,
    pub value: Rational,
    pub witness_world_id: String,
    pub lp_analysis: Analysis,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WorldMomentConstraint {
    pub id: String,
    pub coefficients: BTreeMap<String, Rational>,
    pub lower: Option<Rational>,
    pub upper: Option<Rational>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LpError(pub String);

impl std::fmt::Display for LpError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for LpError {}

fn fail(message: impl Into<String>) -> LpError {
    LpError(message.into())
}

fn valid_token(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 200
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.' | b':' | b'/')
        })
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn required<'a>(metadata: &'a BTreeMap<String, String>, key: &str) -> Result<&'a str, LpError> {
    metadata
        .get(key)
        .map(String::as_str)
        .ok_or_else(|| fail(format!("missing metadata key {key}")))
}

fn parse_pairs(
    lines: &[(usize, String)],
    header: &str,
    expected_fields: usize,
    section: &str,
) -> Result<Vec<Vec<String>>, LpError> {
    let Some((header_line, first)) = lines.first() else {
        return Err(fail(format!("missing {section} section header")));
    };
    if first != header {
        return Err(fail(format!(
            "line {header_line}: expected exact {section} header"
        )));
    }
    let mut rows = Vec::new();
    for (line_no, line) in lines.iter().skip(1) {
        let fields = line.split('|').map(str::trim).collect::<Vec<_>>();
        if fields.len() != expected_fields {
            return Err(fail(format!(
                "line {line_no}: expected {expected_fields} fields in {section}"
            )));
        }
        rows.push(fields.iter().map(|field| (*field).to_owned()).collect());
    }
    Ok(rows)
}

pub fn parse_input(source: &str) -> Result<LpModel, LpError> {
    if source.len() > MAX_INPUT_BYTES {
        return Err(fail("input exceeds 8 MiB"));
    }

    let allowed_metadata = [
        "problem_id",
        "scope_sha256",
        "assumption_set_id",
        "objective_sense",
    ];
    let allowed_sections = ["ROWS", "COLUMNS", "COEFFICIENTS", "PRIMAL", "DUAL"];
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
                    "line {line_no}: metadata is only allowed before sections"
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
            let name = line[1..line.len() - 1].to_owned();
            if !allowed_sections.contains(&name.as_str()) {
                return Err(fail(format!("line {line_no}: unknown section {name}")));
            }
            if sections.contains_key(&name) {
                return Err(fail(format!("line {line_no}: duplicate section {name}")));
            }
            sections.insert(name.clone(), Vec::new());
            current_section = Some(name);
            continue;
        }
        let section = current_section
            .as_ref()
            .ok_or_else(|| fail(format!("line {line_no}: data precedes first section")))?;
        sections
            .get_mut(section)
            .expect("section inserted")
            .push((line_no, line.to_owned()));
    }

    let problem_id = required(&metadata, "problem_id")?.to_owned();
    let scope_sha256 = required(&metadata, "scope_sha256")?.to_owned();
    let assumption_set_id = required(&metadata, "assumption_set_id")?.to_owned();
    let objective_sense = required(&metadata, "objective_sense")?;
    if !valid_token(&problem_id) || !valid_token(&assumption_set_id) {
        return Err(fail(
            "problem_id and assumption_set_id must be valid tokens",
        ));
    }
    if !valid_sha256(&scope_sha256) {
        return Err(fail(
            "scope_sha256 must be 64 lowercase hexadecimal characters",
        ));
    }
    if objective_sense != "MAXIMIZE" {
        return Err(fail("E0 currently supports objective_sense=MAXIMIZE only"));
    }

    for name in allowed_sections {
        if !sections.contains_key(name) {
            return Err(fail(format!("missing [{name}] section")));
        }
    }

    let row_lines = sections.get("ROWS").expect("required").clone();
    let column_lines = sections.get("COLUMNS").expect("required").clone();
    let coefficient_lines = sections.get("COEFFICIENTS").expect("required").clone();
    let primal_lines = sections.get("PRIMAL").expect("required").clone();
    let dual_lines = sections.get("DUAL").expect("required").clone();

    let mut rows = Vec::new();
    let mut row_ids = BTreeSet::<String>::new();
    for fields in parse_pairs(&row_lines, ROWS_HEADER, 2, "ROWS")? {
        if !valid_token(&fields[0]) || !row_ids.insert(fields[0].clone()) {
            return Err(fail(format!("invalid or duplicate row id {}", fields[0])));
        }
        rows.push(LpRow {
            id: fields[0].clone(),
            rhs: Rational::parse(&fields[1], "row rhs")?,
        });
    }
    if rows.is_empty() || rows.len() > MAX_ROWS {
        return Err(fail(format!("row count must be in 1..={MAX_ROWS}")));
    }
    rows.sort_by_key(|row| row.id.clone());

    let mut columns = Vec::new();
    let mut column_ids = BTreeSet::<String>::new();
    for fields in parse_pairs(&column_lines, COLUMNS_HEADER, 2, "COLUMNS")? {
        if !valid_token(&fields[0]) || !column_ids.insert(fields[0].clone()) {
            return Err(fail(format!(
                "invalid or duplicate column id {}",
                fields[0]
            )));
        }
        columns.push(LpColumn {
            id: fields[0].clone(),
            objective: Rational::parse(&fields[1], "objective coefficient")?,
        });
    }
    if columns.is_empty() || columns.len() > MAX_COLUMNS {
        return Err(fail(format!("column count must be in 1..={MAX_COLUMNS}")));
    }
    columns.sort_by_key(|column| column.id.clone());

    let mut coefficients = BTreeMap::<(String, String), Rational>::new();
    for fields in parse_pairs(&coefficient_lines, COEFFICIENTS_HEADER, 3, "COEFFICIENTS")? {
        if !row_ids.contains(&fields[0]) || !column_ids.contains(&fields[1]) {
            return Err(fail(format!(
                "coefficient references unknown row {} or column {}",
                fields[0], fields[1]
            )));
        }
        let value = Rational::parse(&fields[2], "matrix coefficient")?;
        if value == Rational::zero() {
            return Err(fail(
                "explicit zero coefficients must be omitted from sparse matrix",
            ));
        }
        let key = (fields[0].clone(), fields[1].clone());
        if coefficients.insert(key, value).is_some() {
            return Err(fail("duplicate matrix coefficient"));
        }
        if coefficients.len() > MAX_NONZEROS {
            return Err(fail(format!("nonzero count exceeds {MAX_NONZEROS}")));
        }
    }

    let mut primal = BTreeMap::<String, Rational>::new();
    for fields in parse_pairs(&primal_lines, PRIMAL_HEADER, 2, "PRIMAL")? {
        if !column_ids.contains(&fields[0]) {
            return Err(fail(format!(
                "primal certificate references unknown column {}",
                fields[0]
            )));
        }
        if primal
            .insert(
                fields[0].clone(),
                Rational::parse(&fields[1], "primal value")?,
            )
            .is_some()
        {
            return Err(fail(format!("duplicate primal value for {}", fields[0])));
        }
    }
    if primal.len() != columns.len() {
        return Err(fail(
            "primal certificate must provide exactly one value for every column",
        ));
    }

    let mut dual = BTreeMap::<String, Rational>::new();
    for fields in parse_pairs(&dual_lines, DUAL_HEADER, 2, "DUAL")? {
        if !row_ids.contains(&fields[0]) {
            return Err(fail(format!(
                "dual certificate references unknown row {}",
                fields[0]
            )));
        }
        if dual
            .insert(
                fields[0].clone(),
                Rational::parse(&fields[1], "dual value")?,
            )
            .is_some()
        {
            return Err(fail(format!("duplicate dual value for {}", fields[0])));
        }
    }
    if dual.len() != rows.len() {
        return Err(fail(
            "dual certificate must provide exactly one value for every row",
        ));
    }

    Ok(LpModel {
        problem_id,
        scope_sha256,
        assumption_set_id,
        rows,
        columns,
        coefficients,
        primal,
        dual,
    })
}

fn coefficient(model: &LpModel, row: &str, column: &str) -> Rational {
    model
        .coefficients
        .get(&(row.to_owned(), column.to_owned()))
        .copied()
        .unwrap_or_else(Rational::zero)
}

fn dot<I>(terms: I) -> Result<Rational, LpError>
where
    I: IntoIterator<Item = (Rational, Rational)>,
{
    terms
        .into_iter()
        .try_fold(Rational::zero(), |sum, (left, right)| {
            sum.checked_add(left.checked_mul(right)?)
        })
}

fn check_primal(model: &LpModel) -> Result<CertificateCheck, LpError> {
    let objective = dot(model.columns.iter().map(|column| {
        (
            column.objective,
            *model
                .primal
                .get(&column.id)
                .expect("parser checked complete primal"),
        )
    }))?;
    let mut violations = Vec::new();
    for column in &model.columns {
        let value = model.primal[&column.id];
        if !value.is_nonnegative() {
            violations.push(format!("NONNEGATIVE_VARIABLE:{}", column.id));
        }
    }
    for row in &model.rows {
        let lhs = dot(model.columns.iter().map(|column| {
            (
                coefficient(model, &row.id, &column.id),
                model.primal[&column.id],
            )
        }))?;
        if lhs.checked_cmp(row.rhs)? == Ordering::Greater {
            violations.push(format!(
                "ROW_CONSTRAINT:{}:lhs={}:rhs={}",
                row.id,
                lhs.display(),
                row.rhs.display()
            ));
        }
    }
    Ok(CertificateCheck {
        feasible: violations.is_empty(),
        objective,
        violation_count: violations.len(),
        violations,
    })
}

fn check_dual(model: &LpModel) -> Result<CertificateCheck, LpError> {
    let objective = dot(model.rows.iter().map(|row| {
        (
            row.rhs,
            *model
                .dual
                .get(&row.id)
                .expect("parser checked complete dual"),
        )
    }))?;
    let mut violations = Vec::new();
    for row in &model.rows {
        let value = model.dual[&row.id];
        if !value.is_nonnegative() {
            violations.push(format!("NONNEGATIVE_DUAL:{}", row.id));
        }
    }
    for column in &model.columns {
        let lhs = dot(model
            .rows
            .iter()
            .map(|row| (coefficient(model, &row.id, &column.id), model.dual[&row.id])))?;
        if lhs.checked_cmp(column.objective)? == Ordering::Less {
            violations.push(format!(
                "DUAL_CONSTRAINT:{}:lhs={}:rhs={}",
                column.id,
                lhs.display(),
                column.objective.display()
            ));
        }
    }
    Ok(CertificateCheck {
        feasible: violations.is_empty(),
        objective,
        violation_count: violations.len(),
        violations,
    })
}

fn column_signature(model: &LpModel, column: &LpColumn) -> String {
    let mut values = Vec::with_capacity(model.rows.len() + 1);
    values.push(column.objective.display());
    for row in &model.rows {
        values.push(coefficient(model, &row.id, &column.id).display());
    }
    values.join("|")
}

fn reduce_identical_columns(model: &LpModel) -> Result<(LpModel, Vec<ColumnClass>), LpError> {
    let mut signatures = BTreeMap::<String, Vec<String>>::new();
    for column in &model.columns {
        signatures
            .entry(column_signature(model, column))
            .or_default()
            .push(column.id.clone());
    }
    let mut classes = signatures
        .into_values()
        .map(|mut members| {
            members.sort();
            ColumnClass {
                representative: members[0].clone(),
                members,
            }
        })
        .collect::<Vec<_>>();
    classes.sort_by_key(|class| class.representative.clone());

    let mut reduced_columns = Vec::new();
    let mut reduced_coefficients = BTreeMap::new();
    let mut reduced_primal = BTreeMap::new();

    for class in &classes {
        let representative = model
            .columns
            .iter()
            .find(|column| column.id == class.representative)
            .expect("representative exists")
            .clone();
        reduced_columns.push(representative.clone());

        let aggregated_primal = class
            .members
            .iter()
            .try_fold(Rational::zero(), |sum, member| {
                sum.checked_add(model.primal[member])
            })?;
        reduced_primal.insert(representative.id.clone(), aggregated_primal);

        for row in &model.rows {
            let value = coefficient(model, &row.id, &representative.id);
            if value != Rational::zero() {
                reduced_coefficients.insert((row.id.clone(), representative.id.clone()), value);
            }
        }
    }

    reduced_columns.sort_by_key(|column| column.id.clone());
    let reduced = LpModel {
        problem_id: model.problem_id.clone(),
        scope_sha256: model.scope_sha256.clone(),
        assumption_set_id: model.assumption_set_id.clone(),
        rows: model.rows.clone(),
        columns: reduced_columns,
        coefficients: reduced_coefficients,
        primal: reduced_primal,
        dual: model.dual.clone(),
    };
    Ok((reduced, classes))
}

pub fn finite_world_extremum_lp(
    problem_id: &str,
    scope_sha256: &str,
    assumption_set_id: &str,
    values: &[FiniteWorldValue],
    direction: ExtremumDirection,
) -> Result<(LpModel, String), LpError> {
    if !valid_token(problem_id) || !valid_token(assumption_set_id) {
        return Err(fail(
            "finite-world problem and assumption IDs must be valid tokens",
        ));
    }
    if !valid_sha256(scope_sha256) {
        return Err(fail(
            "finite-world scope digest must be 64 lowercase hexadecimal characters",
        ));
    }
    if values.is_empty() || values.len() > MAX_COLUMNS {
        return Err(fail(format!(
            "finite-world value count must be in 1..={MAX_COLUMNS}"
        )));
    }

    let mut sorted = values.to_vec();
    sorted.sort_by_key(|value| value.world_id.clone());
    let mut ids = BTreeSet::new();
    for value in &sorted {
        if !valid_token(&value.world_id) || !ids.insert(value.world_id.clone()) {
            return Err(fail(format!(
                "invalid or duplicate finite-world ID {}",
                value.world_id
            )));
        }
    }

    let transformed = |value: Rational| -> Result<Rational, LpError> {
        match direction {
            ExtremumDirection::Maximum => Ok(value),
            ExtremumDirection::Minimum => value.checked_neg(),
        }
    };

    let mut objectives = BTreeMap::<String, Rational>::new();
    let mut best_world_id = sorted[0].world_id.clone();
    let mut best_objective = transformed(sorted[0].value)?;
    for value in &sorted {
        let objective = transformed(value.value)?;
        objectives.insert(value.world_id.clone(), objective);
        if objective.checked_cmp(best_objective)? == Ordering::Greater {
            best_objective = objective;
            best_world_id = value.world_id.clone();
        }
    }

    let dual_upper = if best_objective.is_nonnegative() {
        best_objective
    } else {
        Rational::zero()
    };
    let dual_lower = if best_objective.is_nonnegative() {
        Rational::zero()
    } else {
        best_objective.checked_neg()?
    };

    let rows = vec![
        LpRow {
            id: "mass-upper".to_owned(),
            rhs: Rational::one(),
        },
        LpRow {
            id: "mass-lower".to_owned(),
            rhs: Rational::new(-1, 1)?,
        },
    ];

    let columns = sorted
        .iter()
        .map(|value| LpColumn {
            id: format!("world-{}", value.world_id),
            objective: objectives[&value.world_id],
        })
        .collect::<Vec<_>>();

    let mut coefficients = BTreeMap::new();
    let mut primal = BTreeMap::new();
    for value in &sorted {
        let column_id = format!("world-{}", value.world_id);
        coefficients.insert(
            ("mass-upper".to_owned(), column_id.clone()),
            Rational::one(),
        );
        coefficients.insert(
            ("mass-lower".to_owned(), column_id.clone()),
            Rational::new(-1, 1)?,
        );
        primal.insert(
            column_id,
            if value.world_id == best_world_id {
                Rational::one()
            } else {
                Rational::zero()
            },
        );
    }

    let dual = BTreeMap::from([
        ("mass-upper".to_owned(), dual_upper),
        ("mass-lower".to_owned(), dual_lower),
    ]);

    Ok((
        LpModel {
            problem_id: problem_id.to_owned(),
            scope_sha256: scope_sha256.to_owned(),
            assumption_set_id: assumption_set_id.to_owned(),
            rows,
            columns,
            coefficients,
            primal,
            dual,
        },
        best_world_id,
    ))
}

/// Builds a partial-identification LP over a declared finite world set.
///
/// Each moment constraint represents:
///   lower <= sum_w coefficient[w] * probability[w] <= upper
/// A missing lower/upper endpoint omits that side of the interval. The
/// normalization simplex is always included. Certificates are keyed by
/// original world IDs (primal) and generated row IDs (dual).
///
/// This builder does not solve the LP. Caller-supplied certificates are
/// checked by analyze using exact rational arithmetic.
pub fn build_world_distribution_lp(
    problem_id: &str,
    scope_sha256: &str,
    assumption_set_id: &str,
    values: &[FiniteWorldValue],
    constraints: &[WorldMomentConstraint],
    direction: ExtremumDirection,
    primal_world_masses: &BTreeMap<String, Rational>,
    dual_row_values: &BTreeMap<String, Rational>,
) -> Result<LpModel, LpError> {
    if !valid_token(problem_id) || !valid_token(assumption_set_id) {
        return Err(fail("world-distribution IDs must be valid tokens"));
    }
    if !valid_sha256(scope_sha256) {
        return Err(fail("world-distribution scope digest must be 64 lowercase hexadecimal characters"));
    }
    if values.is_empty() || values.len() > MAX_COLUMNS {
        return Err(fail(format!("world count must be in 1..={MAX_COLUMNS}")));
    }

    let mut sorted_values = values.to_vec();
    sorted_values.sort_by_key(|value| value.world_id.clone());
    let mut world_ids = BTreeSet::new();
    for value in &sorted_values {
        if !valid_token(&value.world_id) || !world_ids.insert(value.world_id.clone()) {
            return Err(fail(format!("invalid or duplicate world ID {}", value.world_id)));
        }
    }
    if primal_world_masses.len() != values.len()
        || world_ids.iter().any(|world_id| !primal_world_masses.contains_key(world_id))
    {
        return Err(fail("primal world masses must provide exactly one value for every world"));
    }

    let transformed = |value: Rational| -> Result<Rational, LpError> {
        match direction {
            ExtremumDirection::Maximum => Ok(value),
            ExtremumDirection::Minimum => value.checked_neg(),
        }
    };

    let mut columns = sorted_values
        .iter()
        .map(|value| {
            Ok(LpColumn {
                id: format!("world-{}", value.world_id),
                objective: transformed(value.value)?,
            })
        })
        .collect::<Result<Vec<_>, LpError>>()?;
    columns.sort_by_key(|column| column.id.clone());

    let mut rows = vec![
        LpRow {
            id: "mass-upper".to_owned(),
            rhs: Rational::one(),
        },
        LpRow {
            id: "mass-lower".to_owned(),
            rhs: Rational::new(-1, 1)?,
        },
    ];
    let mut row_ids = BTreeSet::from(["mass-upper".to_owned(), "mass-lower".to_owned()]);
    let mut sorted_constraints = constraints.to_vec();
    sorted_constraints.sort_by_key(|constraint| constraint.id.clone());

    for constraint in &sorted_constraints {
        if !valid_token(&constraint.id) {
            return Err(fail(format!("invalid moment constraint ID {}", constraint.id)));
        }
        if constraint.lower.is_none() && constraint.upper.is_none() {
            return Err(fail(format!("moment constraint {} has no bounds", constraint.id)));
        }
        if let (Some(lower), Some(upper)) = (constraint.lower, constraint.upper) {
            if lower.checked_cmp(upper)? == Ordering::Greater {
                return Err(fail(format!("moment constraint {} has lower bound above upper bound", constraint.id)));
            }
        }
        if constraint.coefficients.keys().any(|world_id| !world_ids.contains(world_id)) {
            return Err(fail(format!("moment constraint {} references an unknown world", constraint.id)));
        }
        if let Some(upper) = constraint.upper {
            let id = format!("moment-{}-upper", constraint.id);
            if !row_ids.insert(id.clone()) {
                return Err(fail(format!("duplicate generated row ID {id}")));
            }
            rows.push(LpRow { id, rhs: upper });
        }
        if let Some(lower) = constraint.lower {
            let id = format!("moment-{}-lower", constraint.id);
            if !row_ids.insert(id.clone()) {
                return Err(fail(format!("duplicate generated row ID {id}")));
            }
            rows.push(LpRow {
                id,
                rhs: lower.checked_neg()?,
            });
        }
    }
    rows.sort_by_key(|row| row.id.clone());
    if rows.len() > MAX_ROWS {
        return Err(fail(format!("generated row count exceeds {MAX_ROWS}")));
    }

    let mut coefficients = BTreeMap::<(String, String), Rational>::new();
    for column in &columns {
        let world_id = column.id.strip_prefix("world-").expect("generated world column");
        coefficients.insert(
            ("mass-upper".to_owned(), column.id.clone()),
            Rational::one(),
        );
        coefficients.insert(
            ("mass-lower".to_owned(), column.id.clone()),
            Rational::new(-1, 1)?,
        );
        for constraint in &sorted_constraints {
            let value = constraint
                .coefficients
                .get(world_id)
                .copied()
                .unwrap_or_else(Rational::zero);
            if value == Rational::zero() {
                continue;
            }
            if constraint.upper.is_some() {
                coefficients.insert(
                    (format!("moment-{}-upper", constraint.id), column.id.clone()),
                    value,
                );
            }
            if constraint.lower.is_some() {
                coefficients.insert(
                    (
                        format!("moment-{}-lower", constraint.id),
                        column.id.clone(),
                    ),
                    value.checked_neg()?,
                );
            }
        }
    }
    if coefficients.len() > MAX_NONZEROS {
        return Err(fail(format!("generated nonzero count exceeds {MAX_NONZEROS}")));
    }

    let primal = primal_world_masses
        .iter()
        .map(|(world_id, mass)| (format!("world-{world_id}"), *mass))
        .collect::<BTreeMap<_, _>>();
    if dual_row_values.len() != rows.len()
        || rows.iter().any(|row| !dual_row_values.contains_key(&row.id))
    {
        return Err(fail("dual certificate must provide exactly one value for every generated row"));
    }

    Ok(LpModel {
        problem_id: problem_id.to_owned(),
        scope_sha256: scope_sha256.to_owned(),
        assumption_set_id: assumption_set_id.to_owned(),
        rows,
        columns,
        coefficients,
        primal,
        dual: dual_row_values.clone(),
    })
}

pub fn analyze_finite_world_extremum(
    problem_id: &str,
    scope_sha256: &str,
    assumption_set_id: &str,
    values: &[FiniteWorldValue],
    direction: ExtremumDirection,
) -> Result<FiniteWorldExtremum, LpError> {
    let (model, witness_world_id) = finite_world_extremum_lp(
        problem_id,
        scope_sha256,
        assumption_set_id,
        values,
        direction,
    )?;
    let lp_analysis = analyze(&model)?;
    if lp_analysis.optimality_status != "OPTIMALITY_PROVEN_BY_EXACT_PRIMAL_DUAL_EQUALITY" {
        return Err(fail(
            "finite-world simplex LP did not produce an exact optimality certificate",
        ));
    }
    let value = match direction {
        ExtremumDirection::Maximum => lp_analysis.primal.objective,
        ExtremumDirection::Minimum => lp_analysis.primal.objective.checked_neg()?,
    };
    Ok(FiniteWorldExtremum {
        direction,
        value,
        witness_world_id,
        lp_analysis,
    })
}

/// Verifies a Farkas infeasibility certificate for:
///   A x <= b, x >= 0
///
/// A valid certificate has y >= 0, A^T y >= 0, and b^T y < 0.
/// This proves that no nonnegative x can satisfy every declared row.
/// It proves infeasibility only for the encoded LP and supplied constraints.
pub fn verify_farkas_infeasibility(
    model: &LpModel,
    multipliers: &BTreeMap<String, Rational>,
) -> Result<FarkasCheck, LpError> {
    if multipliers.keys().any(|row_id| !model.rows.iter().any(|row| &row.id == row_id)) {
        return Err(fail("Farkas certificate references an unknown row"));
    }
    if model.rows.iter().any(|row| !multipliers.contains_key(&row.id)) {
        return Err(fail("Farkas certificate must provide exactly one multiplier for every row"));
    }

    let mut violations = Vec::new();
    for row in &model.rows {
        if !multipliers[&row.id].is_nonnegative() {
            violations.push(format!("FARKAS_MULTIPLIER_NEGATIVE:{}", row.id));
        }
    }

    let contradiction_value = dot(model.rows.iter().map(|row| {
        (row.rhs, multipliers[&row.id])
    }))?;

    let mut column_combinations = BTreeMap::new();
    for column in &model.columns {
        let combined = dot(model.rows.iter().map(|row| {
            (coefficient(model, &row.id, &column.id), multipliers[&row.id])
        }))?;
        if !combined.is_nonnegative() {
            violations.push(format!(
                "FARKAS_COLUMN_COMBINATION_NEGATIVE:{}:{}",
                column.id,
                combined.display()
            ));
        }
        column_combinations.insert(column.id.clone(), combined);
    }

    if contradiction_value.checked_cmp(Rational::zero())? != Ordering::Less {
        violations.push(format!(
            "FARKAS_CONTRADICTION_NOT_NEGATIVE:{}",
            contradiction_value.display()
        ));
    }

    Ok(FarkasCheck {
        valid: violations.is_empty(),
        contradiction_value,
        column_combinations,
        violations,
    })
}

pub fn analyze(model: &LpModel) -> Result<Analysis, LpError> {
    let primal = check_primal(model)?;
    let dual = check_dual(model)?;
    let (reduced, equivalence_classes) = reduce_identical_columns(model)?;
    let reduced_primal = check_primal(&reduced)?;
    let reduced_dual = check_dual(&reduced)?;

    let primal_objective_preserved = primal.objective == reduced_primal.objective;
    let primal_feasible_preserved = primal.feasible == reduced_primal.feasible;
    let dual_feasible_preserved = dual.feasible == reduced_dual.feasible;
    let coefficient_projection_exact = equivalence_classes.iter().all(|class| {
        let Some(representative) = model
            .columns
            .iter()
            .find(|column| column.id == class.representative)
        else {
            return false;
        };
        class.members.iter().all(|member| {
            model
                .columns
                .iter()
                .find(|column| column.id == *member)
                .is_some_and(|column| {
                    column_signature(model, column) == column_signature(model, representative)
                })
        })
    });

    if !primal_feasible_preserved
        || !dual_feasible_preserved
        || !primal_objective_preserved
        || !coefficient_projection_exact
    {
        return Err(fail(
            "canonical column reduction failed exact equivalence check",
        ));
    }

    let optimality_status = match (primal.feasible, dual.feasible) {
        (true, true) => match primal.objective.checked_cmp(dual.objective)? {
            Ordering::Equal => "OPTIMALITY_PROVEN_BY_EXACT_PRIMAL_DUAL_EQUALITY",
            Ordering::Less => "FEASIBLE_PRIMAL_DUAL_GAP_BOUNDS_ONLY",
            Ordering::Greater => "CERTIFICATE_CONTRADICTION_WEAK_DUALITY_VIOLATION",
        },
        (true, false) => "PRIMAL_FEASIBLE_LOWER_BOUND_ONLY",
        (false, true) => "DUAL_FEASIBLE_UPPER_BOUND_ONLY",
        (false, false) => "NO_VALID_BOUND_CERTIFICATE",
    }
    .to_owned();

    let objective_lower_bound = primal.feasible.then_some(primal.objective);
    let objective_upper_bound = dual.feasible.then_some(dual.objective);

    Ok(Analysis {
        model: model.clone(),
        primal,
        dual,
        optimality_status,
        objective_lower_bound,
        objective_upper_bound,
        reduction: ReductionCheck {
            original_column_count: model.columns.len(),
            reduced_column_count: reduced.columns.len(),
            equivalence_classes,
            primal_feasible_preserved,
            dual_feasible_preserved,
            primal_objective_preserved,
            coefficient_projection_exact,
            reduced_primal_objective: reduced_primal.objective,
            reduced_dual_objective: reduced_dual.objective,
        },
    })
}

fn valid_mps_name(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 200
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.')
        })
}

/// Emits deterministic free-format MPS for the exact canonical LP.
///
/// MPS decimal fields cannot represent every rational exactly. This writer
/// rejects non-terminating decimal coefficients and never rounds them.
pub fn render_mps(model: &LpModel) -> Result<String, LpError> {
    if model.rows.iter().any(|row| !valid_mps_name(&row.id))
        || model.columns.iter().any(|column| !valid_mps_name(&column.id))
    {
        return Err(fail("MPS row/column names must use ASCII alphanumeric, underscore, dash or dot"));
    }

    let mut out = String::new();
    writeln!(out, "NAME          FSRE").expect("String write");
    out.push_str("OBJSENSE\n MAX\nROWS\n N  OBJ\n");
    for row in &model.rows {
        writeln!(out, " L  {}", row.id).expect("String write");
    }
    out.push_str("COLUMNS\n");
    for column in &model.columns {
        writeln!(
            out,
            "    {:<24} {:<24} {}",
            column.id,
            "OBJ",
            column.objective.finite_decimal()?
        )
        .expect("String write");
        for row in &model.rows {
            let value = coefficient(model, &row.id, &column.id);
            if value == Rational::zero() {
                continue;
            }
            writeln!(
                out,
                "    {:<24} {:<24} {}",
                column.id,
                row.id,
                value.finite_decimal()?
            )
            .expect("String write");
        }
    }
    out.push_str("RHS\n");
    for row in &model.rows {
        writeln!(
            out,
            "    {:<24} {:<24} {}",
            "RHS1",
            row.id,
            row.rhs.finite_decimal()?
        )
        .expect("String write");
    }
    out.push_str("ENDATA\n");
    Ok(out)
}

fn optional_rational(value: Option<Rational>) -> String {
    value
        .map(Rational::display)
        .unwrap_or_else(|| "NOT_ESTABLISHED".to_owned())
}

impl Analysis {
    pub fn render(&self) -> String {
        let mut out = String::new();
        writeln!(out, "schema=fsre.lp-certificate-e0").expect("String write");
        writeln!(out, "version=0.1.0").expect("String write");
        writeln!(out, "problem_id={}", self.model.problem_id).expect("String write");
        writeln!(out, "scope_sha256={}", self.model.scope_sha256).expect("String write");
        writeln!(out, "assumption_set_id={}", self.model.assumption_set_id).expect("String write");
        writeln!(out, "objective_sense=MAXIMIZE").expect("String write");
        writeln!(out, "row_count={}", self.model.rows.len()).expect("String write");
        writeln!(
            out,
            "original_column_count={}",
            self.reduction.original_column_count
        )
        .expect("String write");
        writeln!(out, "nonzero_count={}", self.model.coefficients.len()).expect("String write");
        writeln!(
            out,
            "reduced_column_count={}",
            self.reduction.reduced_column_count
        )
        .expect("String write");
        writeln!(
            out,
            "reduction_class_count={}",
            self.reduction.equivalence_classes.len()
        )
        .expect("String write");
        writeln!(
            out,
            "primal_feasible={}",
            if self.primal.feasible { "PASS" } else { "FAIL" }
        )
        .expect("String write");
        writeln!(
            out,
            "primal_violation_count={}",
            self.primal.violation_count
        )
        .expect("String write");
        writeln!(
            out,
            "primal_objective_candidate={}",
            self.primal.objective.display()
        )
        .expect("String write");
        writeln!(
            out,
            "dual_feasible={}",
            if self.dual.feasible { "PASS" } else { "FAIL" }
        )
        .expect("String write");
        writeln!(out, "dual_violation_count={}", self.dual.violation_count).expect("String write");
        writeln!(
            out,
            "dual_objective_candidate={}",
            self.dual.objective.display()
        )
        .expect("String write");
        writeln!(
            out,
            "objective_lower_bound={}",
            optional_rational(self.objective_lower_bound)
        )
        .expect("String write");
        writeln!(
            out,
            "objective_upper_bound={}",
            optional_rational(self.objective_upper_bound)
        )
        .expect("String write");
        writeln!(out, "optimality_status={}", self.optimality_status).expect("String write");
        writeln!(
            out,
            "reduced_primal_objective={}",
            self.reduction.reduced_primal_objective.display()
        )
        .expect("String write");
        writeln!(
            out,
            "reduced_dual_objective={}",
            self.reduction.reduced_dual_objective.display()
        )
        .expect("String write");
        writeln!(
            out,
            "reduction_primal_feasibility_preserved={}",
            self.reduction.primal_feasible_preserved
        )
        .expect("String write");
        writeln!(
            out,
            "reduction_dual_feasibility_preserved={}",
            self.reduction.dual_feasible_preserved
        )
        .expect("String write");
        writeln!(
            out,
            "reduction_objective_preserved={}",
            self.reduction.primal_objective_preserved
        )
        .expect("String write");
        writeln!(
            out,
            "reduction_coefficient_projection_exact={}",
            self.reduction.coefficient_projection_exact
        )
        .expect("String write");
        for (index, class) in self.reduction.equivalence_classes.iter().enumerate() {
            writeln!(
                out,
                "column_class.{index:04}.representative={}",
                class.representative
            )
            .expect("String write");
            writeln!(
                out,
                "column_class.{index:04}.members={}",
                class.members.join(",")
            )
            .expect("String write");
        }
        for violation in &self.primal.violations {
            writeln!(out, "primal_violation={violation}").expect("String write");
        }
        for violation in &self.dual.violations {
            writeln!(out, "dual_violation={violation}").expect("String write");
        }
        out.push_str("arithmetic=EXACT_NORMALIZED_RATIONAL\n");
        out.push_str("canonical_column_reduction=EXACT_EQUAL_OBJECTIVE_AND_ALL_ROW_COEFFICIENTS\n");
        out.push_str("lp_integrality=CONTINUOUS_NONNEGATIVE_ONLY\n");
        out.push_str("solver_execution=NOT_PERFORMED\n");
        out.push_str("model_authentication=NOT_ESTABLISHED_BY_CERTIFICATE_VERIFIER\n");
        out.push_str("assumption_set_authentication=NOT_ESTABLISHED_BY_CERTIFICATE_VERIFIER\n");
        out.push_str("population_frame_completeness=NOT_ESTABLISHED_BY_CERTIFICATE_VERIFIER\n");
        out.push_str("causal_truth=NOT_ESTABLISHED_BY_CERTIFICATE_VERIFIER\n");
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../fixtures/reduced-columns-optimal.psv");

    fn fixture() -> LpModel {
        parse_input(FIXTURE).expect("fixture parses")
    }

    #[test]
    fn exact_primal_dual_certificate_proves_optimality_for_declared_lp() {
        let report = analyze(&fixture()).unwrap();
        assert!(report.primal.feasible);
        assert!(report.dual.feasible);
        assert_eq!(report.primal.objective.display(), "10");
        assert_eq!(report.dual.objective.display(), "10");
        assert_eq!(
            report.optimality_status,
            "OPTIMALITY_PROVEN_BY_EXACT_PRIMAL_DUAL_EQUALITY"
        );
        assert_eq!(
            report.objective_lower_bound,
            Some(Rational::new(10, 1).unwrap())
        );
        assert_eq!(
            report.objective_upper_bound,
            Some(Rational::new(10, 1).unwrap())
        );
    }

    #[test]
    fn exact_duplicate_column_reduction_preserves_primal_dual_certificate() {
        let report = analyze(&fixture()).unwrap();
        assert_eq!(report.reduction.original_column_count, 3);
        assert_eq!(report.reduction.reduced_column_count, 2);
        assert_eq!(report.reduction.equivalence_classes.len(), 2);
        assert!(report.reduction.primal_feasible_preserved);
        assert!(report.reduction.dual_feasible_preserved);
        assert!(report.reduction.primal_objective_preserved);
        assert!(report.reduction.coefficient_projection_exact);
        assert!(report
            .reduction
            .equivalence_classes
            .iter()
            .any(|class| class.members == vec!["x1".to_owned(), "x2".to_owned()]));
    }

    #[test]
    fn feasible_primal_and_dual_with_gap_are_bounds_not_an_optimum_claim() {
        let source = FIXTURE.replace("x-cap|1", "x-cap|2");
        let report = analyze(&parse_input(&source).unwrap()).unwrap();
        assert!(report.primal.feasible);
        assert!(report.dual.feasible);
        assert_eq!(report.objective_lower_bound.unwrap().display(), "10");
        assert_eq!(report.objective_upper_bound.unwrap().display(), "12");
        assert_eq!(
            report.optimality_status,
            "FEASIBLE_PRIMAL_DUAL_GAP_BOUNDS_ONLY"
        );
    }

    #[test]
    fn dual_feasible_only_provides_upper_bound_when_primal_is_infeasible() {
        let source = FIXTURE.replace("x1|2\nx2|0\ny|2", "x1|3\nx2|0\ny|2");
        let report = analyze(&parse_input(&source).unwrap()).unwrap();
        assert!(!report.primal.feasible);
        assert!(report.dual.feasible);
        assert_eq!(report.objective_lower_bound, None);
        assert_eq!(report.objective_upper_bound.unwrap().display(), "10");
        assert_eq!(report.optimality_status, "DUAL_FEASIBLE_UPPER_BOUND_ONLY");
    }

    #[test]
    fn primal_feasible_only_provides_lower_bound_when_dual_is_infeasible() {
        let source = FIXTURE.replace("total-cap|2", "total-cap|1");
        let report = analyze(&parse_input(&source).unwrap()).unwrap();
        assert!(report.primal.feasible);
        assert!(!report.dual.feasible);
        assert_eq!(report.objective_lower_bound.unwrap().display(), "10");
        assert_eq!(report.objective_upper_bound, None);
        assert_eq!(report.optimality_status, "PRIMAL_FEASIBLE_LOWER_BOUND_ONLY");
    }

    #[test]
    fn mps_export_is_deterministic_and_preserves_finite_decimals() {
        let exported = render_mps(&fixture()).unwrap();
        assert_eq!(exported, include_str!("../fixtures/reduced-columns-optimal.mps.expected.txt"));
        assert!(exported.contains(" MAX\n"));
        assert!(exported.contains("    x1                       OBJ                      3\n"));
        assert!(exported.contains("    x1                       total-cap                1\n"));
    }

    #[test]
    fn mps_export_rejects_non_terminating_rationals_instead_of_rounding() {
        let mut model = fixture();
        model.columns[0].objective = Rational::new(1, 3).unwrap();
        assert!(render_mps(&model).is_err());
    }

    #[test]
    fn finite_decimal_formats_negative_and_fractional_values_exactly() {
        assert_eq!(Rational::new(1, 2).unwrap().finite_decimal().unwrap(), "0.5");
        assert_eq!(Rational::new(-7, 4).unwrap().finite_decimal().unwrap(), "-1.75");
        assert_eq!(Rational::new(5, 2).unwrap().finite_decimal().unwrap(), "2.5");
        assert!(Rational::new(1, 3).unwrap().finite_decimal().is_err());
    }

    #[test]
    fn rational_parser_normalizes_and_compares_exact_values() {
        assert_eq!(Rational::parse("2/4", "test").unwrap().display(), "1/2");
        assert_eq!(Rational::parse("-6/8", "test").unwrap().display(), "-3/4");
        assert_eq!(
            Rational::parse("1/3", "test")
                .unwrap()
                .checked_add(Rational::parse("1/6", "test").unwrap())
                .unwrap()
                .display(),
            "1/2"
        );
        assert!(Rational::parse("1/0", "test").is_err());
    }

    #[test]
    fn duplicate_rows_and_unknown_sections_fail_closed() {
        let duplicate = FIXTURE.replace("total-cap|4", "total-cap|4\ntotal-cap|4");
        assert!(parse_input(&duplicate).is_err());
        let unknown = FIXTURE.replace("[DUAL]", "[TRUST_ME]");
        assert!(parse_input(&unknown).is_err());
    }

    #[test]
    fn matrix_duplicates_and_dangling_references_are_rejected() {
        let duplicate = FIXTURE.replace("y-cap|y|1", "y-cap|y|1\ny-cap|y|1");
        assert!(parse_input(&duplicate).is_err());
        let dangling = FIXTURE.replace("x-cap|x1|1", "missing-row|x1|1");
        assert!(parse_input(&dangling).is_err());
    }

    #[test]
    fn incomplete_primal_or_dual_certificate_is_rejected() {
        let missing_primal = FIXTURE.replace("x2|0\n", "");
        assert!(parse_input(&missing_primal).is_err());
        let missing_dual = FIXTURE.replace("y-cap|0\n", "");
        assert!(parse_input(&missing_dual).is_err());
    }

    #[test]
    fn deterministic_under_section_row_permutations() {
        let original = analyze(&fixture()).unwrap().render();
        let rows = FIXTURE.lines().collect::<Vec<_>>();
        let mut section = String::new();
        let mut current = String::new();
        let mut metadata = Vec::new();
        let mut sections = BTreeMap::<String, Vec<String>>::new();
        for line in rows {
            if line.starts_with('#') {
                metadata.push(line.to_owned());
            } else if line.starts_with('[') {
                current = line.to_owned();
                sections.entry(current.clone()).or_default();
            } else if !line.is_empty() {
                sections
                    .get_mut(&current)
                    .expect("section")
                    .push(line.to_owned());
            }
        }
        for line in metadata {
            writeln!(section, "{line}").unwrap();
        }
        for (name, mut section_rows) in sections {
            section_rows[1..].reverse();
            writeln!(section, "{name}").unwrap();
            for row in section_rows {
                writeln!(section, "{row}").unwrap();
            }
        }
        assert_eq!(
            original,
            analyze(&parse_input(&section).unwrap()).unwrap().render()
        );
    }

    #[test]
    fn finite_world_simplex_lp_matches_exact_enumerated_minimum_and_maximum() {
        let values = vec![
            FiniteWorldValue {
                world_id: "world-a".to_owned(),
                value: Rational::new(4, 1).unwrap(),
            },
            FiniteWorldValue {
                world_id: "world-b".to_owned(),
                value: Rational::new(7, 1).unwrap(),
            },
            FiniteWorldValue {
                world_id: "world-c".to_owned(),
                value: Rational::new(9, 1).unwrap(),
            },
        ];
        let maximum = analyze_finite_world_extremum(
            "finite-world-action-upper",
            "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            "finite-world-set-v1",
            &values,
            ExtremumDirection::Maximum,
        )
        .unwrap();
        let minimum = analyze_finite_world_extremum(
            "finite-world-action-lower",
            "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            "finite-world-set-v1",
            &values,
            ExtremumDirection::Minimum,
        )
        .unwrap();

        assert_eq!(maximum.value.display(), "9");
        assert_eq!(maximum.witness_world_id, "world-c");
        assert_eq!(minimum.value.display(), "4");
        assert_eq!(minimum.witness_world_id, "world-a");
        assert_eq!(
            maximum.lp_analysis.optimality_status,
            "OPTIMALITY_PROVEN_BY_EXACT_PRIMAL_DUAL_EQUALITY"
        );
        assert_eq!(
            minimum.lp_analysis.optimality_status,
            "OPTIMALITY_PROVEN_BY_EXACT_PRIMAL_DUAL_EQUALITY"
        );
    }

    #[test]
    fn finite_world_simplex_bridge_supports_negative_and_rational_values() {
        let values = vec![
            FiniteWorldValue {
                world_id: "world-a".to_owned(),
                value: Rational::new(-7, 3).unwrap(),
            },
            FiniteWorldValue {
                world_id: "world-b".to_owned(),
                value: Rational::new(-1, 2).unwrap(),
            },
            FiniteWorldValue {
                world_id: "world-c".to_owned(),
                value: Rational::new(-5, 4).unwrap(),
            },
        ];
        let maximum = analyze_finite_world_extremum(
            "finite-world-negative-upper",
            "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            "finite-world-set-v1",
            &values,
            ExtremumDirection::Maximum,
        )
        .unwrap();
        let minimum = analyze_finite_world_extremum(
            "finite-world-negative-lower",
            "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            "finite-world-set-v1",
            &values,
            ExtremumDirection::Minimum,
        )
        .unwrap();
        assert_eq!(maximum.value.display(), "-1/2");
        assert_eq!(maximum.witness_world_id, "world-b");
        assert_eq!(minimum.value.display(), "-7/3");
        assert_eq!(minimum.witness_world_id, "world-a");
    }

    #[test]
    fn finite_world_simplex_reduction_merges_only_equal_query_values() {
        let values = vec![
            FiniteWorldValue {
                world_id: "world-a".to_owned(),
                value: Rational::new(5, 1).unwrap(),
            },
            FiniteWorldValue {
                world_id: "world-b".to_owned(),
                value: Rational::new(5, 1).unwrap(),
            },
            FiniteWorldValue {
                world_id: "world-c".to_owned(),
                value: Rational::new(8, 1).unwrap(),
            },
        ];
        let result = analyze_finite_world_extremum(
            "finite-world-tied-states",
            "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            "finite-world-set-v1",
            &values,
            ExtremumDirection::Maximum,
        )
        .unwrap();
        assert_eq!(result.lp_analysis.reduction.original_column_count, 3);
        assert_eq!(result.lp_analysis.reduction.reduced_column_count, 2);
        assert_eq!(result.value.display(), "8");
    }

    #[test]
    fn finite_world_simplex_bridge_matches_exhaustive_enumerator_for_small_integer_domains() {
        let scope = "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";
        let alphabet = [-2_i128, -1, 0, 1, 2];

        for world_count in 1..=4_usize {
            let case_count = 5_u32.pow(world_count as u32);
            for case_index in 0..case_count {
                let mut code = case_index;
                let mut values = Vec::with_capacity(world_count);
                let mut expected_min = i128::MAX;
                let mut expected_max = i128::MIN;

                for world_index in 0..world_count {
                    let value = alphabet[(code % 5) as usize];
                    code /= 5;
                    expected_min = expected_min.min(value);
                    expected_max = expected_max.max(value);
                    values.push(FiniteWorldValue {
                        world_id: format!("world-{world_index:02}"),
                        value: Rational::new(value, 1).unwrap(),
                    });
                }

                let suffix = format!("{world_count}-{case_index}");
                let minimum = analyze_finite_world_extremum(
                    &format!("enumerated-min-{suffix}"),
                    scope,
                    "finite-world-exhaustive-v1",
                    &values,
                    ExtremumDirection::Minimum,
                )
                .unwrap();
                let maximum = analyze_finite_world_extremum(
                    &format!("enumerated-max-{suffix}"),
                    scope,
                    "finite-world-exhaustive-v1",
                    &values,
                    ExtremumDirection::Maximum,
                )
                .unwrap();

                assert_eq!(minimum.value.numerator, expected_min);
                assert_eq!(minimum.value.denominator, 1);
                assert_eq!(maximum.value.numerator, expected_max);
                assert_eq!(maximum.value.denominator, 1);
                assert_eq!(
                    minimum.lp_analysis.optimality_status,
                    "OPTIMALITY_PROVEN_BY_EXACT_PRIMAL_DUAL_EQUALITY"
                );
                assert_eq!(
                    maximum.lp_analysis.optimality_status,
                    "OPTIMALITY_PROVEN_BY_EXACT_PRIMAL_DUAL_EQUALITY"
                );
                assert_eq!(
                    minimum.lp_analysis.primal.objective,
                    minimum.lp_analysis.dual.objective
                );
                assert_eq!(
                    maximum.lp_analysis.primal.objective,
                    maximum.lp_analysis.dual.objective
                );
                assert!(values.iter().any(|value| {
                    value.world_id == minimum.witness_world_id && value.value.numerator == expected_min
                }));
                assert!(values.iter().any(|value| {
                    value.world_id == maximum.witness_world_id && value.value.numerator == expected_max
                }));
            }
        }
    }

    #[test]
    fn finite_world_bridge_rejects_duplicate_world_ids_and_invalid_scope() {
        let duplicate_ids = vec![
            FiniteWorldValue {
                world_id: "world-a".to_owned(),
                value: Rational::new(1, 1).unwrap(),
            },
            FiniteWorldValue {
                world_id: "world-a".to_owned(),
                value: Rational::new(2, 1).unwrap(),
            },
        ];
        assert!(finite_world_extremum_lp(
            "duplicate-world-test",
            "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            "finite-world-set-v1",
            &duplicate_ids,
            ExtremumDirection::Maximum,
        )
        .is_err());

        let values = vec![FiniteWorldValue {
            world_id: "world-a".to_owned(),
            value: Rational::new(1, 1).unwrap(),
        }];
        assert!(finite_world_extremum_lp(
            "invalid-scope-test",
            "not-a-digest",
            "finite-world-set-v1",
            &values,
            ExtremumDirection::Maximum,
        )
        .is_err());
    }

    fn moment_fixture_certificates() -> (
        Vec<FiniteWorldValue>,
        Vec<WorldMomentConstraint>,
        BTreeMap<String, Rational>,
        BTreeMap<String, Rational>,
    ) {
        let values = vec![
            FiniteWorldValue {
                world_id: "w0".to_owned(),
                value: Rational::new(0, 1).unwrap(),
            },
            FiniteWorldValue {
                world_id: "w1".to_owned(),
                value: Rational::new(10, 1).unwrap(),
            },
            FiniteWorldValue {
                world_id: "w2".to_owned(),
                value: Rational::new(20, 1).unwrap(),
            },
            FiniteWorldValue {
                world_id: "w3".to_owned(),
                value: Rational::new(30, 1).unwrap(),
            },
        ];
        let coefficients = BTreeMap::from([
            ("w0".to_owned(), Rational::one()),
            ("w1".to_owned(), Rational::one()),
            ("w2".to_owned(), Rational::zero()),
            ("w3".to_owned(), Rational::zero()),
        ]);
        let constraints = vec![WorldMomentConstraint {
            id: "low-group-share".to_owned(),
            coefficients,
            lower: Some(Rational::new(1, 2).unwrap()),
            upper: Some(Rational::new(1, 2).unwrap()),
        }];
        let max_primal = BTreeMap::from([
            ("w0".to_owned(), Rational::zero()),
            ("w1".to_owned(), Rational::new(1, 2).unwrap()),
            ("w2".to_owned(), Rational::zero()),
            ("w3".to_owned(), Rational::new(1, 2).unwrap()),
        ]);
        let max_dual = BTreeMap::from([
            ("mass-upper".to_owned(), Rational::new(30, 1).unwrap()),
            ("mass-lower".to_owned(), Rational::zero()),
            (
                "moment-low-group-share-upper".to_owned(),
                Rational::zero(),
            ),
            (
                "moment-low-group-share-lower".to_owned(),
                Rational::new(20, 1).unwrap(),
            ),
        ]);
        (values, constraints, max_primal, max_dual)
    }

    #[test]
    fn partial_identification_moment_constraints_prove_sharp_bounds_ten_to_twenty() {
        let (values, constraints, max_primal, max_dual) = moment_fixture_certificates();
        let maximum_model = build_world_distribution_lp(
            "water-population-upper",
            "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            "half-population-known-v1",
            &values,
            &constraints,
            ExtremumDirection::Maximum,
            &max_primal,
            &max_dual,
        )
        .unwrap();
        let maximum = analyze(&maximum_model).unwrap();
        assert!(maximum.primal.feasible);
        assert!(maximum.dual.feasible);
        assert_eq!(maximum.primal.objective.display(), "20");
        assert_eq!(maximum.dual.objective.display(), "20");
        assert_eq!(
            maximum.optimality_status,
            "OPTIMALITY_PROVEN_BY_EXACT_PRIMAL_DUAL_EQUALITY"
        );

        let min_primal = BTreeMap::from([
            ("w0".to_owned(), Rational::new(1, 2).unwrap()),
            ("w1".to_owned(), Rational::zero()),
            ("w2".to_owned(), Rational::new(1, 2).unwrap()),
            ("w3".to_owned(), Rational::zero()),
        ]);
        let min_dual = BTreeMap::from([
            ("mass-upper".to_owned(), Rational::zero()),
            ("mass-lower".to_owned(), Rational::new(20, 1).unwrap()),
            (
                "moment-low-group-share-upper".to_owned(),
                Rational::new(20, 1).unwrap(),
            ),
            (
                "moment-low-group-share-lower".to_owned(),
                Rational::zero(),
            ),
        ]);
        let minimum_model = build_world_distribution_lp(
            "water-population-lower",
            "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            "half-population-known-v1",
            &values,
            &constraints,
            ExtremumDirection::Minimum,
            &min_primal,
            &min_dual,
        )
        .unwrap();
        let minimum = analyze(&minimum_model).unwrap();
        assert!(minimum.primal.feasible);
        assert!(minimum.dual.feasible);
        assert_eq!(minimum.primal.objective.display(), "-10");
        assert_eq!(minimum.dual.objective.display(), "-10");
        assert_eq!(
            minimum.optimality_status,
            "OPTIMALITY_PROVEN_BY_EXACT_PRIMAL_DUAL_EQUALITY"
        );
        assert_eq!(minimum.primal.objective.checked_neg().unwrap().display(), "10");
        assert_eq!(maximum.primal.objective.display(), "20");
    }

    #[test]
    fn moment_constraints_narrow_bounds_relative_to_unconstrained_simplex() {
        let (values, constraints, max_primal, max_dual) = moment_fixture_certificates();
        let constrained_model = build_world_distribution_lp(
            "constrained-upper",
            "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            "half-population-known-v1",
            &values,
            &constraints,
            ExtremumDirection::Maximum,
            &max_primal,
            &max_dual,
        )
        .unwrap();
        let constrained = analyze(&constrained_model).unwrap();
        assert_eq!(constrained.primal.objective.display(), "20");

        let unconstrained_primal = BTreeMap::from([
            ("w0".to_owned(), Rational::zero()),
            ("w1".to_owned(), Rational::zero()),
            ("w2".to_owned(), Rational::zero()),
            ("w3".to_owned(), Rational::one()),
        ]);
        let unconstrained_dual = BTreeMap::from([
            ("mass-upper".to_owned(), Rational::new(30, 1).unwrap()),
            ("mass-lower".to_owned(), Rational::zero()),
        ]);
        let unconstrained_model = build_world_distribution_lp(
            "unconstrained-upper",
            "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            "no-moment-fact-v1",
            &values,
            &[],
            ExtremumDirection::Maximum,
            &unconstrained_primal,
            &unconstrained_dual,
        )
        .unwrap();
        let unconstrained = analyze(&unconstrained_model).unwrap();
        assert_eq!(unconstrained.primal.objective.display(), "30");
        assert!(constrained.primal.objective.checked_cmp(unconstrained.primal.objective).unwrap() == Ordering::Less);
    }

    #[test]
    fn moment_constraint_builder_rejects_invalid_bounds_and_unknown_worlds() {
        let (values, _, primal, _) = moment_fixture_certificates();
        let invalid = vec![WorldMomentConstraint {
            id: "invalid".to_owned(),
            coefficients: BTreeMap::from([("w0".to_owned(), Rational::one())]),
            lower: Some(Rational::new(3, 4).unwrap()),
            upper: Some(Rational::new(1, 2).unwrap()),
        }];
        let dual = BTreeMap::from([
            ("mass-upper".to_owned(), Rational::zero()),
            ("mass-lower".to_owned(), Rational::zero()),
            ("moment-invalid-upper".to_owned(), Rational::zero()),
            ("moment-invalid-lower".to_owned(), Rational::zero()),
        ]);
        assert!(build_world_distribution_lp(
            "invalid-interval",
            "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            "invalid-assumptions",
            &values,
            &invalid,
            ExtremumDirection::Maximum,
            &primal,
            &dual,
        )
        .is_err());

        let unknown = vec![WorldMomentConstraint {
            id: "unknown-world".to_owned(),
            coefficients: BTreeMap::from([("not-a-world".to_owned(), Rational::one())]),
            lower: None,
            upper: Some(Rational::one()),
        }];
        let dual = BTreeMap::from([
            ("mass-upper".to_owned(), Rational::zero()),
            ("mass-lower".to_owned(), Rational::zero()),
            ("moment-unknown-world-upper".to_owned(), Rational::zero()),
        ]);
        assert!(build_world_distribution_lp(
            "unknown-world",
            "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            "invalid-assumptions",
            &values,
            &unknown,
            ExtremumDirection::Maximum,
            &primal,
            &dual,
        )
        .is_err());
    }

    #[test]
    fn farkas_certificate_proves_conflicting_aggregate_constraints_infeasible() {
        let values = vec![
            FiniteWorldValue { world_id: "w0".to_owned(), value: Rational::new(0, 1).unwrap() },
            FiniteWorldValue { world_id: "w1".to_owned(), value: Rational::new(10, 1).unwrap() },
            FiniteWorldValue { world_id: "w2".to_owned(), value: Rational::new(20, 1).unwrap() },
            FiniteWorldValue { world_id: "w3".to_owned(), value: Rational::new(30, 1).unwrap() },
        ];
        let low_group = BTreeMap::from([
            ("w0".to_owned(), Rational::one()),
            ("w1".to_owned(), Rational::one()),
        ]);
        let constraints = vec![
            WorldMomentConstraint {
                id: "low-minimum".to_owned(),
                coefficients: low_group.clone(),
                lower: Some(Rational::new(3, 4).unwrap()),
                upper: None,
            },
            WorldMomentConstraint {
                id: "low-maximum".to_owned(),
                coefficients: low_group,
                lower: None,
                upper: Some(Rational::new(1, 2).unwrap()),
            },
        ];
        let primal = BTreeMap::from([
            ("w0".to_owned(), Rational::zero()),
            ("w1".to_owned(), Rational::zero()),
            ("w2".to_owned(), Rational::zero()),
            ("w3".to_owned(), Rational::zero()),
        ]);
        let dual = BTreeMap::from([
            ("mass-upper".to_owned(), Rational::zero()),
            ("mass-lower".to_owned(), Rational::zero()),
            ("moment-low-minimum-lower".to_owned(), Rational::zero()),
            ("moment-low-maximum-upper".to_owned(), Rational::zero()),
        ]);
        let model = build_world_distribution_lp(
            "incompatible-register-constraints",
            "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            "contradictory-aggregate-facts-v1",
            &values,
            &constraints,
            ExtremumDirection::Maximum,
            &primal,
            &dual,
        )
        .unwrap();
        let multipliers = BTreeMap::from([
            ("mass-upper".to_owned(), Rational::zero()),
            ("mass-lower".to_owned(), Rational::zero()),
            ("moment-low-minimum-lower".to_owned(), Rational::one()),
            ("moment-low-maximum-upper".to_owned(), Rational::one()),
        ]);
        let check = verify_farkas_infeasibility(&model, &multipliers).unwrap();
        assert!(check.valid);
        assert_eq!(check.contradiction_value.display(), "-1/4");
        assert!(check.column_combinations.values().all(|value| *value == Rational::zero()));
    }

    #[test]
    fn invalid_farkas_multiplier_or_unknown_row_is_rejected() {
        let (values, constraints, primal, dual) = moment_fixture_certificates();
        let model = build_world_distribution_lp(
            "farkas-negative-control",
            "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            "half-population-known-v1",
            &values,
            &constraints,
            ExtremumDirection::Maximum,
            &primal,
            &dual,
        )
        .unwrap();
        let mut negative = BTreeMap::from([
            ("mass-upper".to_owned(), Rational::zero()),
            ("mass-lower".to_owned(), Rational::zero()),
            ("moment-low-group-share-upper".to_owned(), Rational::zero()),
            ("moment-low-group-share-lower".to_owned(), Rational::new(-1, 1).unwrap()),
        ]);
        assert!(!verify_farkas_infeasibility(&model, &negative).unwrap().valid);
        negative.insert("unknown-row".to_owned(), Rational::one());
        assert!(verify_farkas_infeasibility(&model, &negative).is_err());
    }

    #[test]
    fn rational_overflow_fails_closed() {
        let large = Rational::new(i128::MAX, 1).unwrap();
        assert!(large.checked_mul(large).is_err());
    }
}
