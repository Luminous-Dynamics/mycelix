#![forbid(unsafe_code)]

// Exact finite-world reference oracle for FSRE partial-identification work.
// It computes extrema over an explicitly declared finite set of admissible
// worlds. It does not construct that set, authenticate its assumptions, infer
// causal effects, or establish real-world completeness. Use it as a small-
// instance oracle for future LP/MILP implementations, not as a scalable
// substitute for them.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

const HEADER: &str = "world_id|action_id|decision_value|safety_value";
const MAX_ROWS: usize = 1_000_000;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BoundsError(pub String);

impl std::fmt::Display for BoundsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for BoundsError {}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Cell {
    decision_value: i64,
    safety_value: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ActionBounds {
    pub action_id: String,
    pub decision_lower: i64,
    pub decision_upper: i64,
    pub safety_lower: i64,
    pub safety_upper: i64,
    pub robustly_safe: bool,
    pub best_case_regret: i128,
    pub worst_case_regret: i128,
    pub robustly_dominates: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BoundsReport {
    pub analysis_id: String,
    pub estimand: String,
    pub unit: String,
    pub decision_cutoff: u64,
    pub safety_floor: i64,
    pub assumption_set_id: String,
    pub input_snapshot_id: String,
    pub status: String,
    pub world_count: usize,
    pub action_count: usize,
    pub ranking_status: String,
    pub invariant_optimal_set: Vec<String>,
    pub robust_optimal_action: Option<String>,
    pub minimax_regret_actions: Vec<String>,
    pub actions: Vec<ActionBounds>,
}

impl BoundsReport {
    /// Stable line-oriented serialization for golden comparison and receipts.
    pub fn render(&self) -> String {
        let mut out = String::new();
        writeln!(&mut out, "analysis_id={}", self.analysis_id).expect("String write");
        writeln!(&mut out, "estimand={}", self.estimand).expect("String write");
        writeln!(&mut out, "unit={}", self.unit).expect("String write");
        writeln!(&mut out, "decision_cutoff={}", self.decision_cutoff).expect("String write");
        writeln!(&mut out, "safety_floor={}", self.safety_floor).expect("String write");
        writeln!(&mut out, "assumption_set_id={}", self.assumption_set_id).expect("String write");
        writeln!(&mut out, "input_snapshot_id={}", self.input_snapshot_id).expect("String write");
        writeln!(&mut out, "analysis_status={}", self.status).expect("String write");
        writeln!(&mut out, "world_count={}", self.world_count).expect("String write");
        writeln!(&mut out, "action_count={}", self.action_count).expect("String write");
        writeln!(&mut out, "ranking_status={}", self.ranking_status).expect("String write");
        writeln!(&mut out, "invariant_optimal_set={}", self.invariant_optimal_set.join(",")).expect("String write");
        writeln!(&mut out, "robust_optimal_action={}", self.robust_optimal_action.as_deref().unwrap_or("NONE")).expect("String write");
        writeln!(&mut out, "minimax_regret_actions={}", self.minimax_regret_actions.join(",")).expect("String write");
        for action in &self.actions {
            writeln!(&mut out, "action.{}.decision_bounds=[{},{}]", action.action_id, action.decision_lower, action.decision_upper).expect("String write");
            writeln!(
                &mut out,
                "action.{}.decision_identification={}",
                action.action_id,
                if action.decision_lower == action.decision_upper { "POINT_WITHIN_DECLARED_WORLD_SET" } else { "PARTIAL_WITHIN_DECLARED_WORLD_SET" }
            ).expect("String write");
            writeln!(&mut out, "action.{}.safety_bounds=[{},{}]", action.action_id, action.safety_lower, action.safety_upper).expect("String write");
            writeln!(&mut out, "action.{}.robust_safety={}", action.action_id, if action.robustly_safe { "PASS" } else { "FAIL" }).expect("String write");
            writeln!(&mut out, "action.{}.regret_bounds=[{},{}]", action.action_id, action.best_case_regret, action.worst_case_regret).expect("String write");
            writeln!(&mut out, "action.{}.worst_case_regret={}", action.action_id, action.worst_case_regret).expect("String write");
            writeln!(&mut out, "action.{}.robustly_dominates={}", action.action_id, action.robustly_dominates.join(",")).expect("String write");
        }
        out.push_str("world_set_completeness=DECLARED_ASSERTION_NOT_AUTHENTICATED\n");
        out.push_str("source_authentication=ASSERTED_NOT_AUTHENTICATED\n");
        out.push_str("causal_model_authentication=NOT_ESTABLISHED\n");
        out
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Qualification {
    pub analysis_id: String,
    pub estimand: String,
    pub unit: String,
    pub decision_cutoff: u64,
    pub status: String,
}

impl Qualification {
    pub fn render(&self) -> String {
        format!(
            concat!(
                "analysis_id={}\n",
                "estimand={}\n",
                "unit={}\n",
                "decision_cutoff={}\n",
                "analysis_status={}\n",
                "world_count=NOT_COMPUTED\n",
                "action_count=NOT_COMPUTED\n",
                "bounds=NOT_COMPUTED\n",
                "recommendation=NONE\n"
            ),
            self.analysis_id, self.estimand, self.unit, self.decision_cutoff, self.status
        )
    }
}

fn valid_token(value: &str) -> bool {
    !value.is_empty() && value.len() <= 200 && value.bytes().all(|byte| {
        byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.' | b':' | b'/')
    })
}

fn required<'a>(metadata: &'a BTreeMap<String, String>, key: &str) -> Result<&'a str, BoundsError> {
    let value = metadata.get(key).ok_or_else(|| BoundsError(format!("missing metadata key {key}")))?;
    if value.is_empty() {
        return Err(BoundsError(format!("empty metadata key {key}")));
    }
    Ok(value)
}

fn parse_metadata_line(line: &str) -> Result<Option<(String, String)>, BoundsError> {
    let Some(comment) = line.strip_prefix('#') else { return Ok(None); };
    let Some((key, value)) = comment.trim().split_once('=') else { return Ok(None); };
    let key = key.trim().to_owned();
    let value = value.trim().to_owned();
    if key.is_empty() || value.is_empty() {
        return Err(BoundsError("empty metadata key or value".to_owned()));
    }
    Ok(Some((key, value)))
}

fn parse_prefix(source: &str) -> Result<(BTreeMap<String, String>, Vec<&str>), BoundsError> {
    let allowed = [
        "analysis_id", "estimand", "unit", "decision_cutoff", "decision_direction",
        "safety_floor", "assumption_set_id", "input_snapshot_id", "source_use_status",
        "authority_status", "world_set_completeness",
    ];
    let mut metadata = BTreeMap::new();
    let mut rows = Vec::new();
    let mut header_seen = false;
    for (offset, raw) in source.lines().enumerate() {
        let line = raw.trim();
        if line.is_empty() { continue; }
        if !header_seen {
            if line.starts_with('#') {
                if let Some((key, value)) = parse_metadata_line(line)? {
                    if !allowed.contains(&key.as_str()) {
                        return Err(BoundsError(format!("unknown metadata key {key}")));
                    }
                    if metadata.insert(key.clone(), value).is_some() {
                        return Err(BoundsError(format!("line {}: duplicate metadata key {key}", offset + 1)));
                    }
                }
                continue;
            }
            if line != HEADER {
                return Err(BoundsError(format!("line {}: expected exact table header", offset + 1)));
            }
            header_seen = true;
        } else {
            rows.push(line);
            if rows.len() > MAX_ROWS {
                return Err(BoundsError(format!("input exceeds {MAX_ROWS} rows")));
            }
        }
    }
    if !header_seen { return Err(BoundsError("missing table header".to_owned())); }
    Ok((metadata, rows))
}

fn parse_i64(value: &str, field: &str) -> Result<i64, BoundsError> {
    value.parse::<i64>().map_err(|error| BoundsError(format!("invalid {field}: {error}")))
}
fn parse_u64(value: &str, field: &str) -> Result<u64, BoundsError> {
    value.parse::<u64>().map_err(|error| BoundsError(format!("invalid {field}: {error}")))
}

fn base_fields(metadata: &BTreeMap<String, String>) -> Result<(String, String, String, u64), BoundsError> {
    let analysis_id = required(metadata, "analysis_id")?;
    let estimand = required(metadata, "estimand")?;
    let unit = required(metadata, "unit")?;
    if !valid_token(analysis_id) || !valid_token(estimand) || !valid_token(unit) {
        return Err(BoundsError("invalid analysis_id, estimand or unit token".to_owned()));
    }
    let cutoff = parse_u64(required(metadata, "decision_cutoff")?, "decision_cutoff")?;
    Ok((analysis_id.to_owned(), estimand.to_owned(), unit.to_owned(), cutoff))
}

fn qualification(metadata: &BTreeMap<String, String>, status: &str) -> Result<Qualification, BoundsError> {
    let (analysis_id, estimand, unit, decision_cutoff) = base_fields(metadata)?;
    Ok(Qualification { analysis_id, estimand, unit, decision_cutoff, status: status.to_owned() })
}

fn gate(metadata: &BTreeMap<String, String>) -> Result<Option<Qualification>, BoundsError> {
    let _ = base_fields(metadata)?;
    let source_use = required(metadata, "source_use_status")?;
    if !matches!(source_use, "AUTHORIZED" | "BLOCKED" | "UNKNOWN") {
        return Err(BoundsError("invalid source_use_status".to_owned()));
    }
    if source_use != "AUTHORIZED" {
        return Ok(Some(qualification(metadata, "BLOCKED_SOURCE_USE")?));
    }
    let authority = required(metadata, "authority_status")?;
    if !matches!(authority, "ESTABLISHED" | "BLOCKED" | "UNKNOWN") {
        return Err(BoundsError("invalid authority_status".to_owned()));
    }
    if authority != "ESTABLISHED" {
        return Ok(Some(qualification(metadata, "BLOCKED_AUTHORITY_UNESTABLISHED")?));
    }
    let completeness = required(metadata, "world_set_completeness")?;
    if !matches!(completeness, "DECLARED_COMPLETE" | "UNKNOWN") {
        return Err(BoundsError("invalid world_set_completeness".to_owned()));
    }
    if completeness != "DECLARED_COMPLETE" {
        return Ok(Some(qualification(metadata, "NOT_IDENTIFIED_WORLD_SET_INCOMPLETE")?));
    }
    Ok(None)
}

/// Evaluates an explicit finite set of admissible counterfactual worlds.
/// Hard source-use and authority gates are checked before outcome rows are
/// parsed. Unknown/incomplete world sets yield NOT_IDENTIFIED without numeric
/// bounds. DECLARED_COMPLETE is an unauthenticated submitter assertion.
pub fn analyze(source: &str) -> Result<Result<BoundsReport, Qualification>, BoundsError> {
    let (raw_metadata, rows) = parse_prefix(source)?;
    if let Some(blocked) = gate(&raw_metadata)? { return Ok(Err(blocked)); }

    for key in ["decision_direction", "safety_floor", "assumption_set_id", "input_snapshot_id"] {
        let value = required(&raw_metadata, key)?;
        if !valid_token(value) { return Err(BoundsError(format!("invalid metadata value for {key}"))); }
    }
    if required(&raw_metadata, "decision_direction")? != "MAXIMIZE" {
        return Err(BoundsError("v0.1 supports decision_direction=MAXIMIZE only".to_owned()));
    }
    let safety_floor = parse_i64(required(&raw_metadata, "safety_floor")?, "safety_floor")?;
    let (analysis_id, estimand, unit, decision_cutoff) = base_fields(&raw_metadata)?;
    let assumption_set_id = required(&raw_metadata, "assumption_set_id")?.to_owned();
    let input_snapshot_id = required(&raw_metadata, "input_snapshot_id")?.to_owned();

    let mut cells = BTreeMap::<(String, String), Cell>::new();
    let mut worlds = BTreeSet::<String>::new();
    let mut actions = BTreeSet::<String>::new();
    for (offset, line) in rows.iter().enumerate() {
        let fields: Vec<&str> = line.split('|').map(str::trim).collect();
        if fields.len() != 4 {
            return Err(BoundsError(format!("data row {}: expected exactly four fields", offset + 1)));
        }
        if !valid_token(fields[0]) || !valid_token(fields[1]) {
            return Err(BoundsError(format!("data row {}: invalid world or action token", offset + 1)));
        }
        let cell = Cell {
            decision_value: parse_i64(fields[2], "decision_value")?,
            safety_value: parse_i64(fields[3], "safety_value")?,
        };
        let key = (fields[0].to_owned(), fields[1].to_owned());
        if cells.insert(key, cell).is_some() {
            return Err(BoundsError(format!("data row {}: duplicate world/action cell", offset + 1)));
        }
        worlds.insert(fields[0].to_owned());
        actions.insert(fields[1].to_owned());
    }
    if worlds.is_empty() || actions.is_empty() {
        return Err(BoundsError("complete declared world set has no cells".to_owned()));
    }
    let expected = worlds.len().checked_mul(actions.len())
        .ok_or_else(|| BoundsError("world/action grid size overflow".to_owned()))?;
    if expected != cells.len() {
        return Err(BoundsError(format!("incomplete world/action grid: expected {expected} cells, found {}", cells.len())));
    }

    let action_ids: Vec<String> = actions.into_iter().collect();
    let world_ids: Vec<String> = worlds.into_iter().collect();
    let mut best_by_world = BTreeMap::<String, i64>::new();
    let mut top_by_world = BTreeMap::<String, Vec<String>>::new();
    for world in &world_ids {
        let mut best: Option<i64> = None;
        let mut top = Vec::new();
        for action in &action_ids {
            let cell = cells.get(&(world.clone(), action.clone())).expect("complete grid checked");
            match best {
                None => { best = Some(cell.decision_value); top.push(action.clone()); }
                Some(value) if cell.decision_value > value => {
                    best = Some(cell.decision_value);
                    top.clear();
                    top.push(action.clone());
                }
                Some(value) if cell.decision_value == value => top.push(action.clone()),
                Some(_) => {}
            }
        }
        best_by_world.insert(world.clone(), best.expect("non-empty actions"));
        top_by_world.insert(world.clone(), top);
    }

    let invariant_set = top_by_world.get(&world_ids[0]).expect("non-empty worlds").clone();
    let ranking_invariant = world_ids.iter().all(|world| top_by_world.get(world) == Some(&invariant_set));
    let ranking_status = if ranking_invariant && invariant_set.len() == 1 {
        "INVARIANT_UNIQUE_OPTIMUM"
    } else if ranking_invariant {
        "INVARIANT_TIED_OPTIMUM"
    } else {
        "RANKING_NOT_INVARIANT"
    };

    let mut results = Vec::new();
    for action in &action_ids {
        let mut decision_lower = i64::MAX;
        let mut decision_upper = i64::MIN;
        let mut safety_lower = i64::MAX;
        let mut safety_upper = i64::MIN;
        let mut robustly_safe = true;
        let mut best_case_regret = i128::MAX;
        let mut worst_case_regret = 0_i128;
        for world in &world_ids {
            let cell = cells.get(&(world.clone(), action.clone())).expect("complete grid checked");
            decision_lower = decision_lower.min(cell.decision_value);
            decision_upper = decision_upper.max(cell.decision_value);
            safety_lower = safety_lower.min(cell.safety_value);
            safety_upper = safety_upper.max(cell.safety_value);
            robustly_safe &= cell.safety_value >= safety_floor;
            let best = *best_by_world.get(world).expect("known world");
            let regret = i128::from(best) - i128::from(cell.decision_value);
            best_case_regret = best_case_regret.min(regret);
            worst_case_regret = worst_case_regret.max(regret);
        }
        let mut robustly_dominates = Vec::new();
        if robustly_safe {
            for other in &action_ids {
                if other == action { continue; }
                let mut never_worse = true;
                let mut strictly_better_once = false;
                for world in &world_ids {
                    let lhs = cells.get(&(world.clone(), action.clone())).expect("complete grid checked").decision_value;
                    let rhs = cells.get(&(world.clone(), other.clone())).expect("complete grid checked").decision_value;
                    never_worse &= lhs >= rhs;
                    strictly_better_once |= lhs > rhs;
                }
                if never_worse && strictly_better_once { robustly_dominates.push(other.clone()); }
            }
        }
        results.push(ActionBounds {
            action_id: action.clone(),
            decision_lower,
            decision_upper,
            safety_lower,
            safety_upper,
            robustly_safe,
            best_case_regret,
            worst_case_regret,
            robustly_dominates,
        });
    }

    let minimum_safe_regret = results.iter().filter(|item| item.robustly_safe)
        .map(|item| item.worst_case_regret).min();
    let minimax_regret_actions = match minimum_safe_regret {
        Some(minimum) => results.iter()
            .filter(|item| item.robustly_safe && item.worst_case_regret == minimum)
            .map(|item| item.action_id.clone()).collect(),
        None => Vec::new(),
    };
    let robust_optimal_action = if ranking_invariant && invariant_set.len() == 1
        && results.iter().any(|item| item.action_id == invariant_set[0] && item.robustly_safe) {
        Some(invariant_set[0].clone())
    } else { None };

    Ok(Ok(BoundsReport {
        analysis_id,
        estimand,
        unit,
        decision_cutoff,
        safety_floor,
        assumption_set_id,
        input_snapshot_id,
        status: if results.iter().any(|item| item.decision_lower != item.decision_upper) {
            "PARTIALLY_IDENTIFIED_WITHIN_DECLARED_WORLD_SET".to_owned()
        } else {
            "POINT_IDENTIFIED_WITHIN_DECLARED_WORLD_SET".to_owned()
        },
        world_count: world_ids.len(),
        action_count: action_ids.len(),
        ranking_status: ranking_status.to_owned(),
        invariant_optimal_set: if ranking_invariant { invariant_set } else { Vec::new() },
        robust_optimal_action,
        minimax_regret_actions,
        actions: results,
    }))
}

#[cfg(test)]
mod unit_tests {
    use super::*;
    #[test]
    fn integer_regret_uses_wide_intermediate() {
        let delta = i128::from(i64::MAX) - i128::from(i64::MIN);
        assert_eq!(delta, 18_446_744_073_709_551_615_i128);
    }
    #[test]
    fn token_parser_rejects_delimiters() {
        assert!(valid_token("plant-001"));
        assert!(!valid_token("plant|001"));
        assert!(!valid_token(""));
    }
}
