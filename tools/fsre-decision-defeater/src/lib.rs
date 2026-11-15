#![forbid(unsafe_code)]

// Bounded E0 decision-defeater reference oracle.
//
// Claim boundary: exact only for the supplied finite world/action grid and
// admitted additive value perturbations. It does not authenticate evidence,
// prove the perturbation universe complete in reality, or establish physical
// feasibility or causal truth.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

const WORLD_HEADER: &str = "world_id|action_id|decision_value|safety_value";
const MUTATION_HEADER: &str =
    "mutation_id|mutation_kind|world_id|action_id|delta|cost|evidence_ref|admissibility";
const MAX_ROWS: usize = 1_000_000;
const MAX_PERTURBATIONS: usize = 22;
const DEFAULT_MAX_SUBSETS: u64 = 1 << MAX_PERTURBATIONS;
const DEFAULT_MAX_WITNESSES: usize = 128;

type CellKey = (String, String);
type CellDelta = (i128, i128);
type DeltaMap = BTreeMap<CellKey, CellDelta>;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DefeaterError(pub String);

impl std::fmt::Display for DefeaterError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for DefeaterError {}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Qualification {
    pub analysis_id: String,
    pub status: String,
    pub detail: String,
}

impl Qualification {
    pub fn render(&self) -> String {
        format!(
            "analysis_id={}\nanalysis_status={}\ndetail={}\n\
             decision_property=NOT_EVALUATED\n\
             evaluated_subsets=0\nrobustness_margin=NOT_COMPUTED\n\
             recommendation=NONE\nsource_authentication=NOT_ESTABLISHED\n",
            self.analysis_id, self.status, self.detail
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DecisionProperty {
    RobustSafeActionRetained,
    NonemptySafeActionSet,
    InvariantUniqueOptimum,
    MinimaxRegretSetUnchanged,
}

impl DecisionProperty {
    fn parse(value: &str) -> Result<Self, DefeaterError> {
        match value {
            "ROBUST_SAFE_ACTION_RETAINED" => Ok(Self::RobustSafeActionRetained),
            "NONEMPTY_SAFE_ACTION_SET" => Ok(Self::NonemptySafeActionSet),
            "INVARIANT_UNIQUE_OPTIMUM" => Ok(Self::InvariantUniqueOptimum),
            "MINIMAX_REGRET_SET_UNCHANGED" => Ok(Self::MinimaxRegretSetUnchanged),
            other => Err(DefeaterError(format!(
                "unsupported decision_property {other}"
            ))),
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::RobustSafeActionRetained => "ROBUST_SAFE_ACTION_RETAINED",
            Self::NonemptySafeActionSet => "NONEMPTY_SAFE_ACTION_SET",
            Self::InvariantUniqueOptimum => "INVARIANT_UNIQUE_OPTIMUM",
            Self::MinimaxRegretSetUnchanged => "MINIMAX_REGRET_SET_UNCHANGED",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MutationKind {
    DecisionValueDelta,
    SafetyValueDelta,
}

impl MutationKind {
    fn parse(value: &str) -> Option<Self> {
        match value {
            "DECISION_VALUE_DELTA" => Some(Self::DecisionValueDelta),
            "SAFETY_VALUE_DELTA" => Some(Self::SafetyValueDelta),
            _ => None,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Cell {
    decision_value: i64,
    safety_value: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Mutation {
    id: String,
    kind: MutationKind,
    world_id: String,
    action_id: String,
    delta: i64,
    cost: u64,
    evidence_ref: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DecisionSignature {
    pub robustly_safe_actions: Vec<String>,
    pub invariant_optimal_set: Vec<String>,
    pub unique_optimum: Option<String>,
    pub minimax_regret_actions: Vec<String>,
}

impl DecisionSignature {
    fn render_into(&self, out: &mut String, prefix: &str) {
        writeln!(
            out,
            "{prefix}.robustly_safe_actions={}",
            self.robustly_safe_actions.join(",")
        )
        .expect("String write");
        writeln!(
            out,
            "{prefix}.invariant_optimal_set={}",
            self.invariant_optimal_set.join(",")
        )
        .expect("String write");
        writeln!(
            out,
            "{prefix}.unique_optimum={}",
            self.unique_optimum.as_deref().unwrap_or("NONE")
        )
        .expect("String write");
        writeln!(
            out,
            "{prefix}.minimax_regret_actions={}",
            self.minimax_regret_actions.join(",")
        )
        .expect("String write");
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DefeaterWitness {
    pub mutation_ids: Vec<String>,
    pub evidence_refs: Vec<String>,
    pub total_cost: u128,
    pub baseline_signature: DecisionSignature,
    pub mutated_signature: DecisionSignature,
    pub failed_property: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReplayReport {
    pub analysis_id: String,
    pub property: String,
    pub selected_mutation_ids: Vec<String>,
    pub selected_evidence_refs: Vec<String>,
    pub total_cost: u128,
    pub is_defeater: bool,
    pub baseline_signature: DecisionSignature,
    pub replayed_signature: DecisionSignature,
}

impl ReplayReport {
    pub fn render(&self) -> String {
        let mut out = String::new();
        writeln!(out, "analysis_id={}", self.analysis_id).expect("String write");
        writeln!(out, "decision_property={}", self.property).expect("String write");
        writeln!(
            out,
            "selected_mutation_ids={}",
            self.selected_mutation_ids.join(",")
        )
        .expect("String write");
        writeln!(
            out,
            "selected_evidence_refs={}",
            self.selected_evidence_refs.join(",")
        )
        .expect("String write");
        writeln!(out, "total_cost={}", self.total_cost).expect("String write");
        writeln!(out, "is_defeater={}", self.is_defeater).expect("String write");
        self.baseline_signature.render_into(&mut out, "baseline");
        self.replayed_signature.render_into(&mut out, "replayed");
        out.push_str("replay_mode=DETERMINISTIC_SELF_REPLAY_NOT_INDEPENDENT_VERIFICATION\n");
        out
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DefeaterReport {
    pub analysis_id: String,
    pub input_snapshot_id: String,
    pub decision_cutoff: u64,
    pub property: String,
    pub target_action: Option<String>,
    pub safety_floor: i64,
    pub cost_unit: String,
    pub status: String,
    pub minimality_status: String,
    pub declared_mutation_count: usize,
    pub admissible_mutation_count: usize,
    pub blocked_mutation_count: usize,
    pub total_subsets: u64,
    pub evaluated_subsets: u64,
    pub witness_count: u64,
    pub witnesses_truncated: bool,
    pub robustness_margin: Option<u128>,
    pub best_witness_cost_upper_bound: Option<u128>,
    pub baseline_signature: DecisionSignature,
    pub witnesses: Vec<DefeaterWitness>,
}

impl DefeaterReport {
    pub fn render(&self) -> String {
        let mut out = String::new();
        writeln!(out, "analysis_id={}", self.analysis_id).expect("String write");
        writeln!(out, "input_snapshot_id={}", self.input_snapshot_id).expect("String write");
        writeln!(out, "decision_cutoff={}", self.decision_cutoff).expect("String write");
        writeln!(out, "decision_property={}", self.property).expect("String write");
        writeln!(
            out,
            "target_action={}",
            self.target_action.as_deref().unwrap_or("NONE")
        )
        .expect("String write");
        writeln!(out, "safety_floor={}", self.safety_floor).expect("String write");
        writeln!(out, "cost_unit={}", self.cost_unit).expect("String write");
        writeln!(out, "search_status={}", self.status).expect("String write");
        writeln!(out, "minimality_status={}", self.minimality_status).expect("String write");
        writeln!(
            out,
            "declared_mutation_count={}",
            self.declared_mutation_count
        )
        .expect("String write");
        writeln!(
            out,
            "admissible_mutation_count={}",
            self.admissible_mutation_count
        )
        .expect("String write");
        writeln!(
            out,
            "blocked_mutation_count={}",
            self.blocked_mutation_count
        )
        .expect("String write");
        writeln!(out, "total_subsets={}", self.total_subsets).expect("String write");
        writeln!(out, "evaluated_subsets={}", self.evaluated_subsets).expect("String write");
        writeln!(out, "minimum_witness_count={}", self.witness_count).expect("String write");
        writeln!(out, "witnesses_truncated={}", self.witnesses_truncated).expect("String write");
        writeln!(
            out,
            "robustness_margin={}",
            self.robustness_margin
                .map_or_else(|| "NOT_ESTABLISHED".to_owned(), |v| v.to_string())
        )
        .expect("String write");
        writeln!(
            out,
            "best_witness_cost_upper_bound={}",
            self.best_witness_cost_upper_bound
                .map_or_else(|| "NONE".to_owned(), |v| v.to_string())
        )
        .expect("String write");
        self.baseline_signature.render_into(&mut out, "baseline");
        for (index, witness) in self.witnesses.iter().enumerate() {
            let prefix = format!("witness.{index:04}");
            writeln!(
                out,
                "{prefix}.mutation_ids={}",
                witness.mutation_ids.join(",")
            )
            .expect("String write");
            writeln!(
                out,
                "{prefix}.evidence_refs={}",
                witness.evidence_refs.join(",")
            )
            .expect("String write");
            writeln!(out, "{prefix}.total_cost={}", witness.total_cost).expect("String write");
            writeln!(out, "{prefix}.failed_property={}", witness.failed_property)
                .expect("String write");
            witness
                .baseline_signature
                .render_into(&mut out, &format!("{prefix}.baseline"));
            witness
                .mutated_signature
                .render_into(&mut out, &format!("{prefix}.mutated"));
        }
        out.push_str("source_use=ASSERTED_NOT_AUTHENTICATED\n");
        out.push_str("authority=ASSERTED_NOT_AUTHENTICATED\n");
        out.push_str("perturbation_admissibility=ASSERTED_NOT_AUTHENTICATED\n");
        out.push_str("world_set_completeness=DECLARED_ASSERTION_NOT_AUTHENTICATED\n");
        out.push_str("perturbation_universe_completeness=DECLARED_ASSERTION_NOT_AUTHENTICATED\n");
        out.push_str("physical_realizability=NOT_ESTABLISHED\n");
        out.push_str("causal_truth=NOT_ESTABLISHED\n");
        out
    }
}

#[derive(Clone, Debug)]
struct Problem {
    analysis_id: String,
    input_snapshot_id: String,
    decision_cutoff: u64,
    safety_floor: i64,
    property: DecisionProperty,
    target_action: Option<String>,
    cost_unit: String,
    max_subsets: u64,
    max_witnesses: usize,
    worlds: Vec<String>,
    actions: Vec<String>,
    cells: BTreeMap<(String, String), Cell>,
    mutations: Vec<Mutation>,
    declared_mutation_count: usize,
    blocked_mutation_count: usize,
    baseline_signature: DecisionSignature,
}

enum Prepared {
    Qualified(Qualification),
    Ready(Box<Problem>),
}

fn valid_token(value: &str) -> bool {
    !value.is_empty()
        && value != "-"
        && value.len() <= 200
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.' | b':' | b'/')
        })
}

fn required<'a>(
    metadata: &'a BTreeMap<String, String>,
    key: &str,
) -> Result<&'a str, DefeaterError> {
    let value = metadata
        .get(key)
        .ok_or_else(|| DefeaterError(format!("missing metadata key {key}")))?;
    if value.is_empty() {
        return Err(DefeaterError(format!("empty metadata key {key}")));
    }
    Ok(value)
}

fn parse_metadata_prefix(
    lines: &[&str],
) -> Result<(BTreeMap<String, String>, usize), DefeaterError> {
    let allowed = [
        "analysis_id",
        "input_snapshot_id",
        "decision_cutoff",
        "decision_direction",
        "decision_property",
        "target_action",
        "safety_floor",
        "source_use_status",
        "authority_status",
        "world_set_completeness",
        "perturbation_universe_completeness",
        "cost_unit",
        "max_subsets",
        "max_witnesses",
    ];
    let mut metadata = BTreeMap::new();
    for (index, raw) in lines.iter().enumerate() {
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            let Some((key, value)) = comment.trim().split_once('=') else {
                continue;
            };
            let key = key.trim().to_owned();
            let value = value.trim().to_owned();
            if key.is_empty() || value.is_empty() {
                return Err(DefeaterError("empty metadata key or value".to_owned()));
            }
            if !allowed.contains(&key.as_str()) {
                return Err(DefeaterError(format!("unknown metadata key {key}")));
            }
            if metadata.insert(key.clone(), value).is_some() {
                return Err(DefeaterError(format!(
                    "line {}: duplicate metadata key {key}",
                    index + 1
                )));
            }
            continue;
        }
        if line != WORLD_HEADER {
            return Err(DefeaterError(format!(
                "line {}: expected exact world/action header",
                index + 1
            )));
        }
        return Ok((metadata, index));
    }
    Err(DefeaterError("missing world/action header".to_owned()))
}

fn qualification(
    metadata: &BTreeMap<String, String>,
    status: &str,
    detail: &str,
) -> Result<Qualification, DefeaterError> {
    let analysis_id = required(metadata, "analysis_id")?;
    if !valid_token(analysis_id) {
        return Err(DefeaterError("invalid analysis_id".to_owned()));
    }
    Ok(Qualification {
        analysis_id: analysis_id.to_owned(),
        status: status.to_owned(),
        detail: detail.to_owned(),
    })
}

fn preflight(metadata: &BTreeMap<String, String>) -> Result<Option<Qualification>, DefeaterError> {
    let analysis_id = required(metadata, "analysis_id")?;
    let input_snapshot_id = required(metadata, "input_snapshot_id")?;
    let _ = required(metadata, "decision_cutoff")?
        .parse::<u64>()
        .map_err(|error| DefeaterError(format!("invalid decision_cutoff: {error}")))?;
    if !valid_token(analysis_id) || !valid_token(input_snapshot_id) {
        return Err(DefeaterError(
            "invalid analysis or snapshot token".to_owned(),
        ));
    }

    match required(metadata, "source_use_status")? {
        "AUTHORIZED" => {}
        "BLOCKED" => {
            return Ok(Some(qualification(
                metadata,
                "BLOCKED_SOURCE_USE",
                "source-use gate blocked before outcome rows were parsed",
            )?));
        }
        "UNKNOWN" => {
            return Ok(Some(qualification(
                metadata,
                "NOT_IDENTIFIED_SOURCE_USE",
                "source-use status is unknown; no numerical evaluation",
            )?));
        }
        other => {
            return Err(DefeaterError(format!("invalid source_use_status {other}")));
        }
    }

    match required(metadata, "authority_status")? {
        "ESTABLISHED" => {}
        "BLOCKED" => {
            return Ok(Some(qualification(
                metadata,
                "BLOCKED_AUTHORITY",
                "authority gate blocked before outcome rows were parsed",
            )?));
        }
        "UNKNOWN" => {
            return Ok(Some(qualification(
                metadata,
                "NOT_IDENTIFIED_AUTHORITY",
                "authority status is unknown; no numerical evaluation",
            )?));
        }
        other => return Err(DefeaterError(format!("invalid authority_status {other}"))),
    }

    match required(metadata, "world_set_completeness")? {
        "DECLARED_COMPLETE" => {}
        "UNKNOWN" => {
            return Ok(Some(qualification(
                metadata,
                "NOT_IDENTIFIED_WORLD_SET",
                "declared admissible world set is incomplete or unknown",
            )?));
        }
        other => {
            return Err(DefeaterError(format!(
                "invalid world_set_completeness {other}"
            )));
        }
    }

    match required(metadata, "perturbation_universe_completeness")? {
        "DECLARED_COMPLETE" => {}
        "UNKNOWN" => {
            return Ok(Some(qualification(
                metadata,
                "NOT_IDENTIFIED_PERTURBATION_UNIVERSE",
                "perturbation universe is incomplete or unknown",
            )?));
        }
        other => {
            return Err(DefeaterError(format!(
                "invalid perturbation_universe_completeness {other}"
            )));
        }
    }
    Ok(None)
}

fn parse_i64(value: &str, label: &str) -> Result<i64, DefeaterError> {
    value
        .parse::<i64>()
        .map_err(|error| DefeaterError(format!("invalid {label}: {error}")))
}

fn parse_u64(value: &str, label: &str) -> Result<u64, DefeaterError> {
    value
        .parse::<u64>()
        .map_err(|error| DefeaterError(format!("invalid {label}: {error}")))
}

fn compute_signature(
    worlds: &[String],
    actions: &[String],
    cells: &BTreeMap<(String, String), Cell>,
    deltas: &DeltaMap,
    safety_floor: i64,
) -> DecisionSignature {
    let robustly_safe_actions: Vec<String> = actions
        .iter()
        .filter(|action| {
            worlds.iter().all(|world| {
                let key = (world.clone(), (*action).clone());
                let cell = cells.get(&key).expect("complete grid was checked");
                let safety_delta = deltas.get(&key).map_or(0, |pair| pair.1);
                i128::from(cell.safety_value) + safety_delta >= i128::from(safety_floor)
            })
        })
        .cloned()
        .collect();

    let mut first_world_top: Option<Vec<String>> = None;
    let mut invariant = true;
    let mut worst_regret = BTreeMap::<String, i128>::new();

    for world in worlds {
        let mut best_value: Option<i128> = None;
        let mut top = Vec::new();
        for action in &robustly_safe_actions {
            let key = (world.clone(), action.clone());
            let cell = cells.get(&key).expect("complete grid was checked");
            let decision_delta = deltas.get(&key).map_or(0, |pair| pair.0);
            let value = i128::from(cell.decision_value) + decision_delta;
            match best_value {
                None => {
                    best_value = Some(value);
                    top.push(action.clone());
                }
                Some(best) if value > best => {
                    best_value = Some(value);
                    top.clear();
                    top.push(action.clone());
                }
                Some(best) if value == best => top.push(action.clone()),
                Some(_) => {}
            }
        }

        if let Some(previous) = &first_world_top {
            if previous != &top {
                invariant = false;
            }
        } else {
            first_world_top = Some(top.clone());
        }

        if let Some(best) = best_value {
            for action in &robustly_safe_actions {
                let key = (world.clone(), action.clone());
                let cell = cells.get(&key).expect("complete grid was checked");
                let decision_delta = deltas.get(&key).map_or(0, |pair| pair.0);
                let value = i128::from(cell.decision_value) + decision_delta;
                let regret = best - value;
                let current = worst_regret.entry(action.clone()).or_insert(0);
                *current = (*current).max(regret);
            }
        }
    }

    let invariant_optimal_set = if invariant {
        first_world_top.unwrap_or_default()
    } else {
        Vec::new()
    };
    let unique_optimum = if invariant_optimal_set.len() == 1 {
        Some(invariant_optimal_set[0].clone())
    } else {
        None
    };

    let minimum_worst_regret = worst_regret.values().copied().min();
    let minimax_regret_actions = match minimum_worst_regret {
        Some(minimum) => worst_regret
            .iter()
            .filter(|(_, regret)| **regret == minimum)
            .map(|(action, _)| action.clone())
            .collect(),
        None => Vec::new(),
    };

    DecisionSignature {
        robustly_safe_actions,
        invariant_optimal_set,
        unique_optimum,
        minimax_regret_actions,
    }
}

fn property_satisfied(
    property: DecisionProperty,
    target_action: Option<&str>,
    baseline: &DecisionSignature,
    current: &DecisionSignature,
) -> bool {
    match property {
        DecisionProperty::RobustSafeActionRetained => target_action.is_some_and(|target| {
            current
                .robustly_safe_actions
                .iter()
                .any(|item| item == target)
        }),
        DecisionProperty::NonemptySafeActionSet => !current.robustly_safe_actions.is_empty(),
        DecisionProperty::InvariantUniqueOptimum => {
            target_action.is_some_and(|target| current.unique_optimum.as_deref() == Some(target))
        }
        DecisionProperty::MinimaxRegretSetUnchanged => {
            current.minimax_regret_actions == baseline.minimax_regret_actions
        }
    }
}

fn apply_mask(mutations: &[Mutation], mask: u64) -> (DeltaMap, u128, Vec<String>, Vec<String>) {
    let mut deltas = DeltaMap::new();
    let mut total_cost = 0_u128;
    let mut mutation_ids = Vec::new();
    let mut evidence_refs = Vec::new();

    for (index, mutation) in mutations.iter().enumerate() {
        if mask & (1_u64 << index) == 0 {
            continue;
        }
        let key = (mutation.world_id.clone(), mutation.action_id.clone());
        let entry = deltas.entry(key).or_insert((0, 0));
        match mutation.kind {
            MutationKind::DecisionValueDelta => entry.0 += i128::from(mutation.delta),
            MutationKind::SafetyValueDelta => entry.1 += i128::from(mutation.delta),
        }
        total_cost += u128::from(mutation.cost);
        mutation_ids.push(mutation.id.clone());
        evidence_refs.push(mutation.evidence_ref.clone());
    }

    (deltas, total_cost, mutation_ids, evidence_refs)
}

fn parse_problem(
    source: &str,
    metadata: &BTreeMap<String, String>,
    header_index: usize,
) -> Result<Result<Problem, Qualification>, DefeaterError> {
    let analysis_id = required(metadata, "analysis_id")?.to_owned();
    let input_snapshot_id = required(metadata, "input_snapshot_id")?.to_owned();
    let decision_cutoff = parse_u64(required(metadata, "decision_cutoff")?, "decision_cutoff")?;
    if required(metadata, "decision_direction")? != "MAXIMIZE" {
        return Err(DefeaterError(
            "v0.1 supports decision_direction=MAXIMIZE only".to_owned(),
        ));
    }

    let property = DecisionProperty::parse(required(metadata, "decision_property")?)?;
    let target_action = metadata.get("target_action").cloned();
    if let Some(target) = &target_action {
        if !valid_token(target) {
            return Err(DefeaterError("invalid target_action".to_owned()));
        }
    }
    match property {
        DecisionProperty::RobustSafeActionRetained | DecisionProperty::InvariantUniqueOptimum
            if target_action.is_none() =>
        {
            return Err(DefeaterError(
                "target_action is required for the selected decision property".to_owned(),
            ));
        }
        _ => {}
    }

    let safety_floor = parse_i64(required(metadata, "safety_floor")?, "safety_floor")?;
    let cost_unit = required(metadata, "cost_unit")?.to_owned();
    if !valid_token(&cost_unit) {
        return Err(DefeaterError("invalid cost_unit".to_owned()));
    }

    let max_subsets = match metadata.get("max_subsets") {
        Some(value) => parse_u64(value, "max_subsets")?,
        None => DEFAULT_MAX_SUBSETS,
    };
    if max_subsets == 0 || max_subsets > DEFAULT_MAX_SUBSETS {
        return Err(DefeaterError(format!(
            "max_subsets must be in 1..={DEFAULT_MAX_SUBSETS}"
        )));
    }
    let max_witnesses = match metadata.get("max_witnesses") {
        Some(value) => value
            .parse::<usize>()
            .map_err(|error| DefeaterError(format!("invalid max_witnesses: {error}")))?,
        None => DEFAULT_MAX_WITNESSES,
    };
    if !(1..=10_000).contains(&max_witnesses) {
        return Err(DefeaterError(
            "max_witnesses must be in 1..=10000".to_owned(),
        ));
    }

    let lines: Vec<&str> = source.lines().collect();
    let mut worlds = BTreeSet::<String>::new();
    let mut actions = BTreeSet::<String>::new();
    let mut cells = BTreeMap::<(String, String), Cell>::new();
    let mut mutations = Vec::new();
    let mut mutation_ids = BTreeSet::new();
    let mut blocked_mutation_count = 0_usize;
    let mut mutation_section = false;
    let mut mutation_header_seen = false;

    let mut index = header_index + 1;
    while index < lines.len() {
        let line = lines[index].trim();
        index += 1;
        if line.is_empty() {
            continue;
        }
        if !mutation_section && line == "MUTATIONS" {
            mutation_section = true;
            if index >= lines.len() || lines[index].trim() != MUTATION_HEADER {
                return Err(DefeaterError(
                    "MUTATIONS must be followed by the exact mutation header".to_owned(),
                ));
            }
            mutation_header_seen = true;
            index += 1;
            continue;
        }
        if line.starts_with('#') {
            return Err(DefeaterError(format!(
                "line {}: metadata is only permitted before the world/action header",
                index
            )));
        }

        if !mutation_section {
            let fields: Vec<&str> = line.split('|').map(str::trim).collect();
            if fields.len() != 4 {
                return Err(DefeaterError(format!(
                    "world row {}: expected exactly four fields",
                    index - header_index
                )));
            }
            if !valid_token(fields[0]) || !valid_token(fields[1]) {
                return Err(DefeaterError(format!(
                    "world row {}: invalid world/action token",
                    index - header_index
                )));
            }
            let cell = Cell {
                decision_value: parse_i64(fields[2], "decision_value")?,
                safety_value: parse_i64(fields[3], "safety_value")?,
            };
            let key = (fields[0].to_owned(), fields[1].to_owned());
            if cells.insert(key, cell).is_some() {
                return Err(DefeaterError(format!(
                    "world row {}: duplicate world/action cell",
                    index - header_index
                )));
            }
            worlds.insert(fields[0].to_owned());
            actions.insert(fields[1].to_owned());
            if cells.len() > MAX_ROWS {
                return Err(DefeaterError(format!(
                    "input exceeds {MAX_ROWS} world rows"
                )));
            }
        } else {
            let fields: Vec<&str> = line.split('|').map(str::trim).collect();
            if fields.len() != 8 {
                return Err(DefeaterError(format!(
                    "mutation row {}: expected exactly eight fields",
                    index
                )));
            }
            if !valid_token(fields[0])
                || !valid_token(fields[2])
                || !valid_token(fields[3])
                || !valid_token(fields[6])
            {
                return Err(DefeaterError(format!(
                    "mutation row {}: invalid identifier or evidence reference",
                    index
                )));
            }
            if !mutation_ids.insert(fields[0].to_owned()) {
                return Err(DefeaterError(format!(
                    "mutation row {}: duplicate mutation_id",
                    index
                )));
            }
            let Some(kind) = MutationKind::parse(fields[1]) else {
                return Ok(Err(Qualification {
                    analysis_id,
                    status: "UNSUPPORTED_MUTATION".to_owned(),
                    detail: format!("unsupported mutation kind {}", fields[1]),
                }));
            };
            let delta = parse_i64(fields[4], "delta")?;
            let cost = parse_u64(fields[5], "cost")?;
            match fields[7] {
                "ADMISSIBLE" => mutations.push(Mutation {
                    id: fields[0].to_owned(),
                    kind,
                    world_id: fields[2].to_owned(),
                    action_id: fields[3].to_owned(),
                    delta,
                    cost,
                    evidence_ref: fields[6].to_owned(),
                }),
                "BLOCKED" => blocked_mutation_count += 1,
                "UNKNOWN" => {
                    return Ok(Err(Qualification {
                        analysis_id,
                        status: "NOT_IDENTIFIED_PERTURBATION_UNIVERSE".to_owned(),
                        detail: format!(
                            "mutation {} has unknown admissibility; no numerical search",
                            fields[0]
                        ),
                    }));
                }
                other => {
                    return Err(DefeaterError(format!(
                        "mutation row {}: invalid admissibility {other}",
                        index
                    )));
                }
            }
            if mutations.len() + blocked_mutation_count > MAX_ROWS {
                return Err(DefeaterError(format!(
                    "input exceeds {MAX_ROWS} mutation rows"
                )));
            }
        }
    }

    if !mutation_section || !mutation_header_seen {
        return Err(DefeaterError("missing MUTATIONS section".to_owned()));
    }
    if worlds.is_empty() || actions.is_empty() {
        return Err(DefeaterError(
            "declared complete world/action grid must not be empty".to_owned(),
        ));
    }
    let expected = worlds
        .len()
        .checked_mul(actions.len())
        .ok_or_else(|| DefeaterError("world/action grid size overflow".to_owned()))?;
    if expected != cells.len() {
        return Err(DefeaterError(format!(
            "incomplete world/action grid: expected {expected} cells, found {}",
            cells.len()
        )));
    }

    for mutation in &mutations {
        if !worlds.contains(&mutation.world_id) || !actions.contains(&mutation.action_id) {
            return Err(DefeaterError(format!(
                "mutation {} references an unknown world or action",
                mutation.id
            )));
        }
    }
    if mutations.len() > MAX_PERTURBATIONS {
        return Ok(Err(Qualification {
            analysis_id,
            status: "SEARCH_LIMIT_EXCEEDED".to_owned(),
            detail: format!(
                "{} admissible mutations exceed the v0.1 cap of {MAX_PERTURBATIONS}",
                mutations.len()
            ),
        }));
    }

    mutations.sort_by_key(|mutation| mutation.id.clone());
    let worlds: Vec<String> = worlds.into_iter().collect();
    let actions: Vec<String> = actions.into_iter().collect();
    let baseline_deltas = DeltaMap::new();
    let baseline_signature =
        compute_signature(&worlds, &actions, &cells, &baseline_deltas, safety_floor);

    if matches!(
        property,
        DecisionProperty::RobustSafeActionRetained | DecisionProperty::InvariantUniqueOptimum
    ) && target_action
        .as_ref()
        .is_some_and(|target| !actions.contains(target))
    {
        return Err(DefeaterError(
            "target_action is not present in the declared action set".to_owned(),
        ));
    }
    if !property_satisfied(
        property,
        target_action.as_deref(),
        &baseline_signature,
        &baseline_signature,
    ) {
        return Err(DefeaterError(
            "baseline decision property is not satisfied; defeater search requires a qualified baseline"
                .to_owned(),
        ));
    }
    if baseline_signature.robustly_safe_actions.is_empty() {
        return Err(DefeaterError(
            "baseline has no robustly safe actions; decision-regret properties are undefined"
                .to_owned(),
        ));
    }

    Ok(Ok(Problem {
        analysis_id,
        input_snapshot_id,
        decision_cutoff,
        safety_floor,
        property,
        target_action,
        cost_unit,
        max_subsets,
        max_witnesses,
        worlds,
        actions,
        cells,
        declared_mutation_count: mutations.len() + blocked_mutation_count,
        blocked_mutation_count,
        mutations,
        baseline_signature,
    }))
}

fn prepare(source: &str) -> Result<Prepared, DefeaterError> {
    let lines: Vec<&str> = source.lines().collect();
    let (metadata, header_index) = parse_metadata_prefix(&lines)?;
    if let Some(blocked) = preflight(&metadata)? {
        return Ok(Prepared::Qualified(blocked));
    }
    match parse_problem(source, &metadata, header_index)? {
        Ok(problem) => Ok(Prepared::Ready(Box::new(problem))),
        Err(qualification) => Ok(Prepared::Qualified(qualification)),
    }
}

/// Runs an exhaustive search over every subset of the declared admissible
/// perturbations, unless the declared subset budget is reached first.
pub fn analyze(source: &str) -> Result<Result<DefeaterReport, Qualification>, DefeaterError> {
    let problem = match prepare(source)? {
        Prepared::Qualified(qualification) => return Ok(Err(qualification)),
        Prepared::Ready(problem) => *problem,
    };

    let total_subsets = 1_u64
        .checked_shl(problem.mutations.len() as u32)
        .ok_or_else(|| DefeaterError("subset count overflow".to_owned()))?;
    let search_limit = problem.max_subsets.min(total_subsets);
    let mut evaluated_subsets = 0_u64;
    let mut best_cost: Option<u128> = None;
    let mut witness_count = 0_u64;
    let mut witnesses = Vec::new();
    let mut witnesses_truncated = false;

    for mask in 0..search_limit {
        evaluated_subsets += 1;
        let (deltas, total_cost, mutation_ids, evidence_refs) =
            apply_mask(&problem.mutations, mask);
        let signature = compute_signature(
            &problem.worlds,
            &problem.actions,
            &problem.cells,
            &deltas,
            problem.safety_floor,
        );
        if property_satisfied(
            problem.property,
            problem.target_action.as_deref(),
            &problem.baseline_signature,
            &signature,
        ) {
            continue;
        }

        match best_cost {
            None => {
                best_cost = Some(total_cost);
                witness_count = 1;
                witnesses.clear();
                witnesses_truncated = false;
            }
            Some(best) if total_cost < best => {
                best_cost = Some(total_cost);
                witness_count = 1;
                witnesses.clear();
                witnesses_truncated = false;
            }
            Some(best) if total_cost == best => witness_count += 1,
            Some(_) => continue,
        }

        if witnesses.len() < problem.max_witnesses {
            witnesses.push(DefeaterWitness {
                mutation_ids,
                evidence_refs,
                total_cost,
                baseline_signature: problem.baseline_signature.clone(),
                mutated_signature: signature,
                failed_property: problem.property.label().to_owned(),
            });
        } else {
            witnesses_truncated = true;
        }
    }

    witnesses.sort_by_key(|witness| witness.mutation_ids.clone());
    let complete = evaluated_subsets == total_subsets;
    let (status, minimality_status, robustness_margin) = if complete {
        match best_cost {
            Some(cost) => ("DEFEATER_FOUND", "PROVEN_MINIMAL", Some(cost)),
            None => (
                "NO_DEFEATER_FOUND_IN_DECLARED_UNIVERSE",
                "NOT_APPLICABLE",
                None,
            ),
        }
    } else {
        match best_cost {
            Some(_) => ("SEARCH_LIMIT_EXCEEDED", "WITNESS_ONLY", None),
            None => ("SEARCH_LIMIT_EXCEEDED", "NOT_ESTABLISHED", None),
        }
    };

    Ok(Ok(DefeaterReport {
        analysis_id: problem.analysis_id,
        input_snapshot_id: problem.input_snapshot_id,
        decision_cutoff: problem.decision_cutoff,
        property: problem.property.label().to_owned(),
        target_action: problem.target_action,
        safety_floor: problem.safety_floor,
        cost_unit: problem.cost_unit,
        status: status.to_owned(),
        minimality_status: minimality_status.to_owned(),
        declared_mutation_count: problem.declared_mutation_count,
        admissible_mutation_count: problem.mutations.len(),
        blocked_mutation_count: problem.blocked_mutation_count,
        total_subsets,
        evaluated_subsets,
        witness_count,
        witnesses_truncated,
        robustness_margin,
        best_witness_cost_upper_bound: best_cost,
        baseline_signature: problem.baseline_signature,
        witnesses,
    }))
}

/// Recomputes one selected mutation subset against the original frozen input.
/// This is deterministic self-replay, not an independent implementation.
pub fn replay_witness(
    source: &str,
    selected_mutation_ids: &[String],
) -> Result<Result<ReplayReport, Qualification>, DefeaterError> {
    let problem = match prepare(source)? {
        Prepared::Qualified(qualification) => return Ok(Err(qualification)),
        Prepared::Ready(problem) => problem,
    };

    let mut requested = BTreeSet::new();
    for id in selected_mutation_ids {
        if !valid_token(id) || !requested.insert(id.as_str()) {
            return Err(DefeaterError(
                "selected mutation IDs must be valid and unique".to_owned(),
            ));
        }
    }
    let mut deltas = BTreeMap::<(String, String), (i128, i128)>::new();
    let mut total_cost = 0_u128;
    let mut canonical_ids = Vec::new();
    let mut selected_evidence_refs = Vec::new();

    for id in &requested {
        let mutation = problem
            .mutations
            .iter()
            .find(|mutation| mutation.id == *id)
            .ok_or_else(|| DefeaterError(format!("unknown or non-admissible mutation {id}")))?;
        let key = (mutation.world_id.clone(), mutation.action_id.clone());
        let entry = deltas.entry(key).or_insert((0, 0));
        match mutation.kind {
            MutationKind::DecisionValueDelta => entry.0 += i128::from(mutation.delta),
            MutationKind::SafetyValueDelta => entry.1 += i128::from(mutation.delta),
        }
        total_cost += u128::from(mutation.cost);
        canonical_ids.push(mutation.id.clone());
        selected_evidence_refs.push(mutation.evidence_ref.clone());
    }
    let replayed_signature = compute_signature(
        &problem.worlds,
        &problem.actions,
        &problem.cells,
        &deltas,
        problem.safety_floor,
    );
    let is_defeater = !property_satisfied(
        problem.property,
        problem.target_action.as_deref(),
        &problem.baseline_signature,
        &replayed_signature,
    );

    Ok(Ok(ReplayReport {
        analysis_id: problem.analysis_id,
        property: problem.property.label().to_owned(),
        selected_mutation_ids: canonical_ids,
        selected_evidence_refs,
        total_cost,
        is_defeater,
        baseline_signature: problem.baseline_signature,
        replayed_signature,
    }))
}

#[cfg(test)]
mod tests {
    use super::{analyze, replay_witness};

    const FIXTURE: &str = r#"
#analysis_id=pump-station-decision
#input_snapshot_id=snapshot-2026-11-15-a
#decision_cutoff=100
#decision_direction=MAXIMIZE
#decision_property=ROBUST_SAFE_ACTION_RETAINED
#target_action=repair
#safety_floor=60
#source_use_status=AUTHORIZED
#authority_status=ESTABLISHED
#world_set_completeness=DECLARED_COMPLETE
#perturbation_universe_completeness=DECLARED_COMPLETE
#cost_unit=inspection_points
#max_subsets=1000
#max_witnesses=10
world_id|action_id|decision_value|safety_value
w1|repair|90|80
w1|inspect|70|75
w2|repair|85|80
w2|inspect|75|75
MUTATIONS
mutation_id|mutation_kind|world_id|action_id|delta|cost|evidence_ref|admissibility
m-a|SAFETY_VALUE_DELTA|w1|repair|-30|2|evidence-a|ADMISSIBLE
m-b|SAFETY_VALUE_DELTA|w2|repair|-30|2|evidence-b|ADMISSIBLE
m-noop|DECISION_VALUE_DELTA|w1|inspect|0|1|evidence-noop|ADMISSIBLE
"#;

    #[test]
    fn exhaustive_search_finds_all_tied_minimum_witnesses() {
        let report = analyze(FIXTURE).unwrap().unwrap();
        assert_eq!(report.status, "DEFEATER_FOUND");
        assert_eq!(report.minimality_status, "PROVEN_MINIMAL");
        assert_eq!(report.total_subsets, 8);
        assert_eq!(report.evaluated_subsets, 8);
        assert_eq!(report.robustness_margin, Some(2));
        assert_eq!(report.witness_count, 2);
        assert_eq!(report.witnesses.len(), 2);
        assert_eq!(report.witnesses[0].mutation_ids, vec!["m-a".to_owned()]);
        assert_eq!(report.witnesses[1].mutation_ids, vec!["m-b".to_owned()]);
    }

    #[test]
    fn every_returned_witness_can_be_replayed() {
        let report = analyze(FIXTURE).unwrap().unwrap();
        for witness in &report.witnesses {
            let replay = replay_witness(FIXTURE, &witness.mutation_ids)
                .unwrap()
                .unwrap();
            assert!(replay.is_defeater);
            assert_eq!(replay.total_cost, witness.total_cost);
            assert_eq!(replay.replayed_signature, witness.mutated_signature);
            assert_eq!(replay.selected_evidence_refs, witness.evidence_refs);
            assert_eq!(witness.evidence_refs.len(), witness.mutation_ids.len());
        }
    }

    #[test]
    fn complete_negative_search_is_not_universal_robustness() {
        let input = FIXTURE
            .replace(
                "m-a|SAFETY_VALUE_DELTA|w1|repair|-30|2|evidence-a|ADMISSIBLE\n",
                "",
            )
            .replace(
                "m-b|SAFETY_VALUE_DELTA|w2|repair|-30|2|evidence-b|ADMISSIBLE\n",
                "",
            );
        let report = analyze(&input).unwrap().unwrap();
        assert_eq!(report.status, "NO_DEFEATER_FOUND_IN_DECLARED_UNIVERSE");
        assert_eq!(report.minimality_status, "NOT_APPLICABLE");
        assert_eq!(report.robustness_margin, None);
        assert_eq!(report.evaluated_subsets, 2);
    }

    #[test]
    fn subset_budget_never_claims_proven_minimality() {
        let input = FIXTURE.replace("max_subsets=1000", "max_subsets=2");
        let report = analyze(&input).unwrap().unwrap();
        assert_eq!(report.status, "SEARCH_LIMIT_EXCEEDED");
        assert_eq!(report.minimality_status, "WITNESS_ONLY");
        assert_eq!(report.robustness_margin, None);
        assert_eq!(report.best_witness_cost_upper_bound, Some(2));
        assert_eq!(report.evaluated_subsets, 2);
    }

    #[test]
    fn blocked_mutations_are_excluded_and_counted() {
        let input = FIXTURE.replace(
            "m-noop|DECISION_VALUE_DELTA|w1|inspect|0|1|evidence-noop|ADMISSIBLE",
            "m-noop|DECISION_VALUE_DELTA|w1|inspect|0|1|evidence-noop|BLOCKED",
        );
        let report = analyze(&input).unwrap().unwrap();
        assert_eq!(report.blocked_mutation_count, 1);
        assert_eq!(report.admissible_mutation_count, 2);
        assert_eq!(report.total_subsets, 4);
    }

    #[test]
    fn unknown_mutation_admissibility_prevents_negative_search() {
        let input = FIXTURE.replace("|ADMISSIBLE\nm-b|", "|UNKNOWN\nm-b|");
        let qualification = analyze(&input).unwrap().unwrap_err();
        assert_eq!(qualification.status, "NOT_IDENTIFIED_PERTURBATION_UNIVERSE");
    }

    #[test]
    fn source_use_gate_runs_before_world_and_mutation_rows() {
        let input = FIXTURE
            .replace("source_use_status=AUTHORIZED", "source_use_status=BLOCKED")
            .replace("w1|repair|90|80", "not-a-valid-world-row");
        let qualification = analyze(&input).unwrap().unwrap_err();
        assert_eq!(qualification.status, "BLOCKED_SOURCE_USE");
        assert!(qualification.render().contains("evaluated_subsets=0"));
    }

    #[test]
    fn input_row_and_mutation_order_do_not_change_result() {
        let original = analyze(FIXTURE).unwrap().unwrap().render();
        let reordered = FIXTURE
            .replace(
                "w1|repair|90|80\nw1|inspect|70|75\nw2|repair|85|80\nw2|inspect|75|75",
                "w2|inspect|75|75\nw2|repair|85|80\nw1|inspect|70|75\nw1|repair|90|80",
            )
            .replace(
                "m-a|SAFETY_VALUE_DELTA|w1|repair|-30|2|evidence-a|ADMISSIBLE\nm-b|SAFETY_VALUE_DELTA|w2|repair|-30|2|evidence-b|ADMISSIBLE\nm-noop|DECISION_VALUE_DELTA|w1|inspect|0|1|evidence-noop|ADMISSIBLE",
                "m-noop|DECISION_VALUE_DELTA|w1|inspect|0|1|evidence-noop|ADMISSIBLE\nm-b|SAFETY_VALUE_DELTA|w2|repair|-30|2|evidence-b|ADMISSIBLE\nm-a|SAFETY_VALUE_DELTA|w1|repair|-30|2|evidence-a|ADMISSIBLE",
            );
        assert_eq!(analyze(&reordered).unwrap().unwrap().render(), original);
    }

    #[test]
    fn duplicate_cells_and_incomplete_grids_are_rejected() {
        let duplicate = FIXTURE.replace("w2|inspect|75|75", "w2|inspect|75|75\nw2|inspect|75|75");
        assert!(analyze(&duplicate).is_err());
        let missing = FIXTURE.replace("w2|inspect|75|75\n", "");
        assert!(analyze(&missing).is_err());
    }

    #[test]
    fn baseline_property_must_already_hold() {
        let input = FIXTURE.replace("#safety_floor=60", "#safety_floor=81");
        assert!(analyze(&input).is_err());
    }

    #[test]
    fn unsupported_mutation_type_is_not_silently_ignored() {
        let input = FIXTURE.replace("m-noop|DECISION_VALUE_DELTA", "m-noop|IDENTITY_EDGE_CHANGE");
        let qualification = analyze(&input).unwrap().unwrap_err();
        assert_eq!(qualification.status, "UNSUPPORTED_MUTATION");
    }

    #[test]
    fn mutation_reference_must_resolve_inside_the_frozen_grid() {
        let input = FIXTURE.replace(
            "m-noop|DECISION_VALUE_DELTA|w1|inspect|0|1|evidence-noop|ADMISSIBLE",
            "m-noop|DECISION_VALUE_DELTA|missing-world|inspect|0|1|evidence-noop|ADMISSIBLE",
        );
        assert!(analyze(&input).is_err());
    }

    #[test]
    fn witness_cap_is_disclosed_without_false_minimality() {
        let input = FIXTURE.replace("max_witnesses=10", "max_witnesses=1");
        let report = analyze(&input).unwrap().unwrap();
        assert_eq!(report.witness_count, 2);
        assert_eq!(report.witnesses.len(), 1);
        assert!(report.witnesses_truncated);
        assert_eq!(report.minimality_status, "PROVEN_MINIMAL");
    }

    #[test]
    fn interacting_perturbations_are_found_when_no_single_change_defeats() {
        let input = FIXTURE
            .replace("repair|-30|2|evidence-a", "repair|-15|2|evidence-a")
            .replace("w2|repair|-30|2|evidence-b", "w1|repair|-15|3|evidence-b");
        let report = analyze(&input).unwrap().unwrap();
        assert_eq!(report.status, "DEFEATER_FOUND");
        assert_eq!(report.robustness_margin, Some(5));
        assert_eq!(report.witness_count, 1);
        assert_eq!(
            report.witnesses[0].mutation_ids,
            vec!["m-a".to_owned(), "m-b".to_owned()]
        );
    }

    #[test]
    fn unique_optimum_property_is_checked_across_every_world() {
        let input = FIXTURE.replace(
            "decision_property=ROBUST_SAFE_ACTION_RETAINED",
            "decision_property=INVARIANT_UNIQUE_OPTIMUM",
        );
        let report = analyze(&input).unwrap().unwrap();
        assert_eq!(report.robustness_margin, Some(2));
        assert!(report.witnesses.iter().all(|witness| {
            witness.mutated_signature.unique_optimum.as_deref() == Some("inspect")
        }));
    }

    #[test]
    fn minimax_regret_set_changes_are_detected() {
        let input = FIXTURE.replace(
            "decision_property=ROBUST_SAFE_ACTION_RETAINED",
            "decision_property=MINIMAX_REGRET_SET_UNCHANGED",
        );
        let report = analyze(&input).unwrap().unwrap();
        assert_eq!(report.robustness_margin, Some(2));
        assert!(report.witnesses.iter().all(|witness| {
            witness.mutated_signature.minimax_regret_actions == vec!["inspect".to_owned()]
        }));
    }

    #[test]
    fn nonempty_safe_action_property_requires_all_safe_actions_to_be_defeated() {
        let input = FIXTURE
            .replace(
                "decision_property=ROBUST_SAFE_ACTION_RETAINED",
                "decision_property=NONEMPTY_SAFE_ACTION_SET",
            )
            .replace(
                "m-noop|DECISION_VALUE_DELTA|w1|inspect|0|1|evidence-noop|ADMISSIBLE",
                concat!(
                    "m-noop|DECISION_VALUE_DELTA|w1|inspect|0|1|evidence-noop|ADMISSIBLE\n",
                    "m-c|SAFETY_VALUE_DELTA|w1|inspect|-30|4|evidence-c|ADMISSIBLE\n",
                    "m-d|SAFETY_VALUE_DELTA|w2|inspect|-30|5|evidence-d|ADMISSIBLE"
                ),
            );
        let report = analyze(&input).unwrap().unwrap();
        assert_eq!(report.robustness_margin, Some(6));
        assert_eq!(report.witness_count, 2);
        assert!(report
            .witnesses
            .iter()
            .all(|witness| { witness.mutated_signature.robustly_safe_actions.is_empty() }));
    }

    #[test]
    fn authority_unknown_blocks_numeric_analysis() {
        let input = FIXTURE.replace("authority_status=ESTABLISHED", "authority_status=UNKNOWN");
        let qualification = analyze(&input).unwrap().unwrap_err();
        assert_eq!(qualification.status, "NOT_IDENTIFIED_AUTHORITY");
    }
}
