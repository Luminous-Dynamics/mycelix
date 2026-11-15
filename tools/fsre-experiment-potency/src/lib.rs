#![forbid(unsafe_code)]

// Exact finite-world reference for safe, cost-constrained experiment selection.
//
// The objective is worst-case identified-set width over deterministic
// observation partitions. It is intentionally not an implementation of
// general SCM polynomial programming or the NeurIPS max-potency algorithm.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

const WORLD_HEADER: &str = "world_id|query_value";
const EXPERIMENT_HEADER: &str = "experiment_id|cost|unit_id|start|end|regime|carryover|source_use|authority|rights_ethics|safety|realizability|evidence_ref";
const OBSERVATION_HEADER: &str = "experiment_id|world_id|outcome";
const MAX_ROWS: usize = 1_000_000;
const MAX_EXPERIMENTS: usize = 22;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct World {
    pub id: String,
    pub query_value: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Experiment {
    pub id: String,
    pub cost: u64,
    pub unit_id: String,
    pub start: i64,
    pub end: i64,
    pub regime: String,
    pub carryover: String,
    pub source_use: String,
    pub authority: String,
    pub rights_ethics: String,
    pub safety: String,
    pub realizability: String,
    pub evidence_ref: String,
    pub outcomes: BTreeMap<String, String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Problem {
    pub analysis_id: String,
    pub cost_unit: String,
    pub budget: u64,
    pub max_subsets: u64,
    pub world_set_complete: bool,
    pub experiment_universe_complete: bool,
    pub worlds: Vec<World>,
    pub experiments: Vec<Experiment>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CandidateStatus {
    Selectable,
    BlockedAuthority,
    BlockedSourceUseOrRights,
    BlockedSafetyOrServiceFloor,
    NotPhysicallyRealizable,
    RealizabilityNotEstablished,
    InvalidDesign,
}

impl CandidateStatus {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Selectable => "SELECTABLE_WITHIN_DECLARED_DESIGN",
            Self::BlockedAuthority => "BLOCKED_AUTHORITY",
            Self::BlockedSourceUseOrRights => "BLOCKED_SOURCE_USE_OR_RIGHTS",
            Self::BlockedSafetyOrServiceFloor => "BLOCKED_SAFETY_OR_SERVICE_FLOOR",
            Self::NotPhysicallyRealizable => "NOT_PHYSICALLY_REALIZABLE",
            Self::RealizabilityNotEstablished => "REALIZABILITY_NOT_ESTABLISHED",
            Self::InvalidDesign => "INVALID_DESIGN",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SelectedSet {
    pub experiment_ids: Vec<String>,
    pub cost: u128,
    pub residual_width: i128,
    pub potency: i128,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SearchResult {
    pub status: String,
    pub evaluated_subsets: u64,
    pub candidate_count: usize,
    pub best: Option<SelectedSet>,
    pub tied_best_sets: Vec<SelectedSet>,
    pub baseline_width: i128,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Analysis {
    pub analysis_id: String,
    pub cost_unit: String,
    pub budget: u64,
    pub search_status: String,
    pub world_count: usize,
    pub experiment_count: usize,
    pub selectable_count: usize,
    pub pruned_inert_ids: Vec<String>,
    pub candidate_statuses: Vec<(String, CandidateStatus)>,
    pub unpruned: SearchResult,
    pub pruned: SearchResult,
    pub pruning_equivalence: bool,
    pub greedy: SelectedSet,
    pub greedy_is_optimal: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PotencyError(pub String);

impl std::fmt::Display for PotencyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for PotencyError {}

fn fail(message: impl Into<String>) -> PotencyError {
    PotencyError(message.into())
}

fn valid_token(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 200
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.' | b':' | b'/')
        })
}

fn parse_u64(value: &str, name: &str) -> Result<u64, PotencyError> {
    value.parse::<u64>().map_err(|error| fail(format!("invalid {name}: {error}")))
}

fn parse_i64(value: &str, name: &str) -> Result<i64, PotencyError> {
    value.parse::<i64>().map_err(|error| fail(format!("invalid {name}: {error}")))
}

fn parse_bool_complete(value: &str, name: &str) -> Result<bool, PotencyError> {
    match value {
        "DECLARED_COMPLETE" => Ok(true),
        "NOT_ESTABLISHED" | "DECLARED_INCOMPLETE" => Ok(false),
        _ => Err(fail(format!("invalid {name} state {value}"))),
    }
}

pub fn parse_input(source: &str) -> Result<Problem, PotencyError> {
    if source.len() > 16 * 1024 * 1024 {
        return Err(fail("input exceeds 16 MiB"));
    }
    let mut metadata = BTreeMap::<String, String>::new();
    let mut lines = Vec::<(usize, &str)>::new();
    for (index, raw) in source.lines().enumerate() {
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            let (key, value) = comment.trim().split_once('=')
                .ok_or_else(|| fail(format!("line {}: malformed metadata", index + 1)))?;
            if !matches!(key,
                "analysis_id" | "cost_unit" | "budget" | "max_subsets" |
                "world_set_completeness" | "experiment_universe_completeness")
            {
                return Err(fail(format!("line {}: unknown metadata key {key}", index + 1)));
            }
            if value.is_empty() || metadata.insert(key.to_owned(), value.to_owned()).is_some() {
                return Err(fail(format!("line {}: empty or duplicate metadata", index + 1)));
            }
        } else {
            lines.push((index + 1, line));
        }
    }
    let required = |key: &str| -> Result<&str, PotencyError> {
        metadata.get(key).map(String::as_str)
            .ok_or_else(|| fail(format!("missing metadata key {key}")))
    };
    let analysis_id = required("analysis_id")?.to_owned();
    let cost_unit = required("cost_unit")?.to_owned();
    if !valid_token(&analysis_id) || !valid_token(&cost_unit) {
        return Err(fail("analysis_id and cost_unit must be valid tokens"));
    }
    let budget = parse_u64(required("budget")?, "budget")?;
    let max_subsets = parse_u64(required("max_subsets")?, "max_subsets")?;
    if max_subsets == 0 {
        return Err(fail("max_subsets must be positive"));
    }
    let world_set_complete = parse_bool_complete(required("world_set_completeness")?, "world_set_completeness")?;
    let experiment_universe_complete = parse_bool_complete(required("experiment_universe_completeness")?, "experiment_universe_completeness")?;

    let mut cursor = 0usize;
    if lines.get(cursor).map(|(_, line)| *line) != Some(WORLD_HEADER) {
        return Err(fail("expected exact world header"));
    }
    cursor += 1;
    let mut worlds = Vec::new();
    let mut world_ids = BTreeSet::new();
    while cursor < lines.len() {
        let (line_no, line) = lines[cursor];
        if line == "EXPERIMENTS" {
            break;
        }
        let fields: Vec<&str> = line.split('|').map(str::trim).collect();
        if fields.len() != 2 || !valid_token(fields[0]) {
            return Err(fail(format!("line {line_no}: invalid world row")));
        }
        if !world_ids.insert(fields[0].to_owned()) {
            return Err(fail(format!("line {line_no}: duplicate world {}", fields[0])));
        }
        worlds.push(World {
            id: fields[0].to_owned(),
            query_value: parse_i64(fields[1], "query_value")?,
        });
        if worlds.len() > MAX_ROWS {
            return Err(fail("world count exceeds limit"));
        }
        cursor += 1;
    }
    if worlds.is_empty() {
        return Err(fail("world set is empty"));
    }
    worlds.sort_by_key(|world| world.id.clone());

    if lines.get(cursor).map(|(_, line)| *line) != Some("EXPERIMENTS") {
        return Err(fail("missing EXPERIMENTS section"));
    }
    cursor += 1;
    if lines.get(cursor).map(|(_, line)| *line) != Some(EXPERIMENT_HEADER) {
        return Err(fail("expected exact experiment header"));
    }
    cursor += 1;
    let mut experiments = Vec::<Experiment>::new();
    let mut experiment_ids = BTreeSet::new();
    while cursor < lines.len() {
        let (line_no, line) = lines[cursor];
        if line == "OBSERVATIONS" {
            break;
        }
        let fields: Vec<&str> = line.split('|').map(str::trim).collect();
        if fields.len() != 13 || !valid_token(fields[0]) ||
            !valid_token(fields[2]) || !valid_token(fields[5]) ||
            !valid_token(fields[12])
        {
            return Err(fail(format!("line {line_no}: invalid experiment row")));
        }
        if !experiment_ids.insert(fields[0].to_owned()) {
            return Err(fail(format!("line {line_no}: duplicate experiment {}", fields[0])));
        }
        let exp = Experiment {
            id: fields[0].to_owned(),
            cost: parse_u64(fields[1], "experiment cost")?,
            unit_id: fields[2].to_owned(),
            start: parse_i64(fields[3], "start")?,
            end: parse_i64(fields[4], "end")?,
            regime: fields[5].to_owned(),
            carryover: fields[6].to_owned(),
            source_use: fields[7].to_owned(),
            authority: fields[8].to_owned(),
            rights_ethics: fields[9].to_owned(),
            safety: fields[10].to_owned(),
            realizability: fields[11].to_owned(),
            evidence_ref: fields[12].to_owned(),
            outcomes: BTreeMap::new(),
        };
        if exp.start >= exp.end {
            return Err(fail(format!("line {line_no}: schedule interval must have start < end")));
        }
        for (value, field) in [
            (&exp.carryover, "carryover"),
            (&exp.source_use, "source_use"),
            (&exp.authority, "authority"),
            (&exp.rights_ethics, "rights_ethics"),
            (&exp.safety, "safety"),
            (&exp.realizability, "realizability"),
        ] {
            if !valid_token(value) {
                return Err(fail(format!("line {line_no}: invalid {field} token")));
            }
        }
        experiments.push(exp);
        if experiments.len() > MAX_ROWS {
            return Err(fail("experiment count exceeds limit"));
        }
        cursor += 1;
    }
    if experiments.is_empty() {
        return Err(fail("experiment set is empty"));
    }
    experiments.sort_by_key(|experiment| experiment.id.clone());

    if lines.get(cursor).map(|(_, line)| *line) != Some("OBSERVATIONS") {
        return Err(fail("missing OBSERVATIONS section"));
    }
    cursor += 1;
    if lines.get(cursor).map(|(_, line)| *line) != Some(OBSERVATION_HEADER) {
        return Err(fail("expected exact observation header"));
    }
    cursor += 1;
    let mut observation_count = 0usize;
    while cursor < lines.len() {
        let (line_no, line) = lines[cursor];
        let fields: Vec<&str> = line.split('|').map(str::trim).collect();
        if fields.len() != 3 || !valid_token(fields[0]) || !valid_token(fields[1]) || !valid_token(fields[2]) {
            return Err(fail(format!("line {line_no}: invalid observation row")));
        }
        let experiment = experiments.iter_mut().find(|experiment| experiment.id == fields[0])
            .ok_or_else(|| fail(format!("line {line_no}: unknown experiment {}", fields[0])))?;
        if !world_ids.contains(fields[1]) {
            return Err(fail(format!("line {line_no}: unknown world {}", fields[1])));
        }
        if experiment.outcomes.insert(fields[1].to_owned(), fields[2].to_owned()).is_some() {
            return Err(fail(format!("line {line_no}: duplicate experiment/world observation")));
        }
        observation_count += 1;
        if observation_count > MAX_ROWS {
            return Err(fail("observation count exceeds limit"));
        }
        cursor += 1;
    }
    let expected = experiments.len().checked_mul(worlds.len())
        .ok_or_else(|| fail("observation matrix size overflow"))?;
    if observation_count != expected {
        return Err(fail(format!("observation matrix incomplete: expected {expected}, got {observation_count}")));
    }
    for experiment in &experiments {
        if experiment.outcomes.len() != worlds.len() {
            return Err(fail(format!("experiment {} lacks a world outcome", experiment.id)));
        }
    }

    Ok(Problem {
        analysis_id, cost_unit, budget, max_subsets, world_set_complete,
        experiment_universe_complete, worlds, experiments,
    })
}

pub fn candidate_status(experiment: &Experiment) -> CandidateStatus {
    if experiment.start >= experiment.end {
        return CandidateStatus::InvalidDesign;
    }
    if experiment.authority == "BLOCKED" {
        return CandidateStatus::BlockedAuthority;
    }
    if experiment.authority != "ESTABLISHED" {
        return CandidateStatus::RealizabilityNotEstablished;
    }
    if experiment.source_use == "BLOCKED" || experiment.rights_ethics == "BLOCKED" {
        return CandidateStatus::BlockedSourceUseOrRights;
    }
    if experiment.source_use != "AUTHORIZED" || experiment.rights_ethics != "CLEARED" {
        return CandidateStatus::BlockedSourceUseOrRights;
    }
    if experiment.safety == "BLOCKED" {
        return CandidateStatus::BlockedSafetyOrServiceFloor;
    }
    if experiment.safety != "WITHIN_FLOOR" {
        return CandidateStatus::BlockedSafetyOrServiceFloor;
    }
    match experiment.realizability.as_str() {
        "NOT_REALIZABLE" => return CandidateStatus::NotPhysicallyRealizable,
        "ESTABLISHED" => {}
        _ => return CandidateStatus::RealizabilityNotEstablished,
    }
    if !matches!(experiment.carryover.as_str(), "NO_CARRYOVER" | "WASHOUT_VERIFIED" | "UNKNOWN") {
        return CandidateStatus::InvalidDesign;
    }
    CandidateStatus::Selectable
}

fn schedule_compatible(selected: &[&Experiment]) -> bool {
    for i in 0..selected.len() {
        for j in i + 1..selected.len() {
            let a = selected[i];
            let b = selected[j];
            if a.unit_id != b.unit_id || a.regime == b.regime {
                continue;
            }
            let overlap = a.start < b.end && b.start < a.end;
            if overlap {
                return false;
            }
            let carryover_ok = matches!(a.carryover.as_str(), "NO_CARRYOVER" | "WASHOUT_VERIFIED")
                && matches!(b.carryover.as_str(), "NO_CARRYOVER" | "WASHOUT_VERIFIED");
            if !carryover_ok {
                return false;
            }
        }
    }
    true
}

fn baseline_width(worlds: &[World]) -> i128 {
    let min = worlds.iter().map(|world| i128::from(world.query_value)).min().unwrap_or(0);
    let max = worlds.iter().map(|world| i128::from(world.query_value)).max().unwrap_or(0);
    max - min
}

pub fn residual_width(problem: &Problem, selected: &[&Experiment]) -> i128 {
    let mut groups = BTreeMap::<Vec<String>, (i128, i128)>::new();
    for world in &problem.worlds {
        let signature = selected.iter()
            .map(|experiment| experiment.outcomes.get(&world.id).expect("complete outcome matrix").clone())
            .collect::<Vec<_>>();
        let value = i128::from(world.query_value);
        groups.entry(signature)
            .and_modify(|range| {
                range.0 = range.0.min(value);
                range.1 = range.1.max(value);
            })
            .or_insert((value, value));
    }
    groups.values().map(|(min, max)| max - min).max().unwrap_or(0)
}

fn make_set(problem: &Problem, selected: &[&Experiment]) -> SelectedSet {
    let residual = residual_width(problem, selected);
    let base = baseline_width(&problem.worlds);
    SelectedSet {
        experiment_ids: selected.iter().map(|experiment| experiment.id.clone()).collect(),
        cost: selected.iter().map(|experiment| u128::from(experiment.cost)).sum(),
        residual_width: residual,
        potency: base - residual,
    }
}

fn subsets_to_search(problem: &Problem, candidates: &[&Experiment], label: &str) -> SearchResult {
    let base_width = baseline_width(&problem.worlds);
    if candidates.len() > MAX_EXPERIMENTS {
        return SearchResult {
            status: "SEARCH_LIMIT_EXCEEDED".to_owned(),
            evaluated_subsets: 0,
            candidate_count: candidates.len(),
            best: None,
            tied_best_sets: Vec::new(),
            baseline_width: base_width,
        };
    }
    let total = 1_u64 << candidates.len();
    if total > problem.max_subsets {
        return SearchResult {
            status: "SEARCH_LIMIT_EXCEEDED".to_owned(),
            evaluated_subsets: 0,
            candidate_count: candidates.len(),
            best: None,
            tied_best_sets: Vec::new(),
            baseline_width: base_width,
        };
    }
    let mut evaluated = 0u64;
    let mut best: Option<SelectedSet> = None;
    let mut ties = Vec::<SelectedSet>::new();

    for mask in 0..total {
        evaluated += 1;
        let selected: Vec<&Experiment> = candidates.iter().enumerate()
            .filter_map(|(index, experiment)| {
                if mask & (1_u64 << index) != 0 { Some(*experiment) } else { None }
            })
            .collect();
        let cost: u128 = selected.iter().map(|experiment| u128::from(experiment.cost)).sum();
        if cost > u128::from(problem.budget) || !schedule_compatible(&selected) {
            continue;
        }
        let candidate = make_set(problem, &selected);
        match &best {
            None => {
                best = Some(candidate.clone());
                ties.clear();
                ties.push(candidate);
            }
            Some(current) if candidate.potency > current.potency ||
                (candidate.potency == current.potency && candidate.cost < current.cost) => {
                best = Some(candidate.clone());
                ties.clear();
                ties.push(candidate);
            }
            Some(current) if candidate.potency == current.potency && candidate.cost == current.cost => {
                ties.push(candidate);
            }
            Some(_) => {}
        }
    }
    ties.sort_by_key(|set| set.experiment_ids.clone());
    SearchResult {
        status: "EXACT_OPTIMUM".to_owned(),
        evaluated_subsets: evaluated,
        candidate_count: candidates.len(),
        best,
        tied_best_sets: ties,
        baseline_width: base_width,
    }
}

fn inert_experiment(experiment: &Experiment, worlds: &[World]) -> bool {
    let mut outcomes = worlds.iter().map(|world| {
        experiment.outcomes.get(&world.id).expect("complete outcome matrix")
    });
    let Some(first) = outcomes.next() else { return true; };
    outcomes.all(|outcome| outcome == first)
}

fn greedy(problem: &Problem, candidates: &[&Experiment]) -> SelectedSet {
    let mut selected = Vec::<&Experiment>::new();
    let mut selected_ids = BTreeSet::<String>::new();
    let mut spent = 0u128;
    loop {
        let current = make_set(problem, &selected);
        let mut best: Option<(&Experiment, i128)> = None;
        for candidate in candidates {
            if selected_ids.contains(&candidate.id) {
                continue;
            }
            let next_cost = spent + u128::from(candidate.cost);
            if next_cost > u128::from(problem.budget) {
                continue;
            }
            let mut trial = selected.clone();
            trial.push(candidate);
            trial.sort_by_key(|experiment| experiment.id.clone());
            if !schedule_compatible(&trial) {
                continue;
            }
            let marginal = current.residual_width - residual_width(problem, &trial);
            if marginal <= 0 {
                continue;
            }
            let replace = match best {
                None => true,
                Some((old, old_gain)) => {
                    if candidate.cost == 0 && old.cost != 0 {
                        true
                    } else if candidate.cost != 0 && old.cost == 0 {
                        false
                    } else if candidate.cost == 0 && old.cost == 0 {
                        marginal > old_gain || (marginal == old_gain && candidate.id < old.id)
                    } else {
                        let lhs = marginal * i128::from(old.cost);
                        let rhs = old_gain * i128::from(candidate.cost);
                        lhs > rhs || (lhs == rhs && candidate.id < old.id)
                    }
                }
            };
            if replace {
                best = Some((candidate, marginal));
            }
        }
        let Some((chosen, _)) = best else { break; };
        selected.push(chosen);
        selected.sort_by_key(|experiment| experiment.id.clone());
        selected_ids.insert(chosen.id.clone());
        spent += u128::from(chosen.cost);
    }
    make_set(problem, &selected)
}

pub fn analyze(problem: &Problem) -> Analysis {
    let statuses: Vec<(String, CandidateStatus)> = problem.experiments.iter()
        .map(|experiment| (experiment.id.clone(), candidate_status(experiment)))
        .collect();
    let selectable: Vec<&Experiment> = problem.experiments.iter()
        .filter(|experiment| candidate_status(experiment) == CandidateStatus::Selectable)
        .collect();
    let pruned_inert_ids = selectable.iter()
        .filter(|experiment| inert_experiment(experiment, &problem.worlds))
        .map(|experiment| experiment.id.clone())
        .collect::<Vec<_>>();
    let unpruned = if !problem.world_set_complete || !problem.experiment_universe_complete {
        SearchResult {
            status: "INPUT_OR_EXPERIMENT_UNIVERSE_INCOMPLETE".to_owned(),
            evaluated_subsets: 0, candidate_count: selectable.len(), best: None,
            tied_best_sets: Vec::new(), baseline_width: baseline_width(&problem.worlds),
        }
    } else {
        subsets_to_search(problem, &selectable, "unpruned")
    };
    let pruned_candidates: Vec<&Experiment> = selectable.iter()
        .copied()
        .filter(|experiment| !inert_experiment(experiment, &problem.worlds))
        .collect();
    let pruned = if !problem.world_set_complete || !problem.experiment_universe_complete {
        SearchResult {
            status: "INPUT_OR_EXPERIMENT_UNIVERSE_INCOMPLETE".to_owned(),
            evaluated_subsets: 0, candidate_count: pruned_candidates.len(), best: None,
            tied_best_sets: Vec::new(), baseline_width: baseline_width(&problem.worlds),
        }
    } else {
        subsets_to_search(problem, &pruned_candidates, "sound-inert-pruned")
    };
    let pruning_equivalence = unpruned.status == "EXACT_OPTIMUM"
        && pruned.status == "EXACT_OPTIMUM"
        && unpruned.best == pruned.best
        && unpruned.tied_best_sets == pruned.tied_best_sets;
    let greedy_set = greedy(problem, &selectable);
    let greedy_is_optimal = unpruned.best.as_ref().is_some_and(|best| {
        best.potency == greedy_set.potency && best.cost == greedy_set.cost
    });
    let search_status = if !problem.world_set_complete || !problem.experiment_universe_complete {
        "INPUT_OR_EXPERIMENT_UNIVERSE_INCOMPLETE"
    } else if unpruned.status == "SEARCH_LIMIT_EXCEEDED" || pruned.status == "SEARCH_LIMIT_EXCEEDED" {
        "SEARCH_LIMIT_EXCEEDED"
    } else if !pruning_equivalence {
        "PRUNING_EQUIVALENCE_NOT_ESTABLISHED"
    } else {
        "EXACT_OPTIMUM"
    };
    Analysis {
        analysis_id: problem.analysis_id.clone(),
        cost_unit: problem.cost_unit.clone(),
        budget: problem.budget,
        search_status: search_status.to_owned(),
        world_count: problem.worlds.len(),
        experiment_count: problem.experiments.len(),
        selectable_count: selectable.len(),
        pruned_inert_ids,
        candidate_statuses: statuses,
        unpruned,
        pruned,
        pruning_equivalence,
        greedy: greedy_set,
        greedy_is_optimal,
    }
}

impl Analysis {
    pub fn render(&self) -> String {
        let mut out = String::new();
        writeln!(out, "schema=fsre.experiment-potency-e0").expect("String write");
        writeln!(out, "version=0.1.0").expect("String write");
        writeln!(out, "analysis_id={}", self.analysis_id).expect("String write");
        writeln!(out, "search_status={}", self.search_status).expect("String write");
        writeln!(out, "world_count={}", self.world_count).expect("String write");
        writeln!(out, "experiment_count={}", self.experiment_count).expect("String write");
        writeln!(out, "selectable_count={}", self.selectable_count).expect("String write");
        writeln!(out, "budget={}", self.budget).expect("String write");
        writeln!(out, "cost_unit={}", self.cost_unit).expect("String write");
        writeln!(out, "unpruned_evaluated_subsets={}", self.unpruned.evaluated_subsets).expect("String write");
        writeln!(out, "pruned_evaluated_subsets={}", self.pruned.evaluated_subsets).expect("String write");
        writeln!(out, "pruned_inert_count={}", self.pruned_inert_ids.len()).expect("String write");
        writeln!(out, "pruned_inert_ids={}", self.pruned_inert_ids.join(",")).expect("String write");
        writeln!(out, "pruning_equivalence={}", self.pruning_equivalence).expect("String write");
        writeln!(out, "baseline_width={}", self.unpruned.baseline_width).expect("String write");
        if let Some(best) = &self.unpruned.best {
            writeln!(out, "optimal_potency={}", best.potency).expect("String write");
            writeln!(out, "optimal_residual_width={}", best.residual_width).expect("String write");
            writeln!(out, "optimal_cost={}", best.cost).expect("String write");
            writeln!(out, "optimal_set_count={}", self.unpruned.tied_best_sets.len()).expect("String write");
            for (index, set) in self.unpruned.tied_best_sets.iter().enumerate() {
                writeln!(out, "optimal_set.{index:04}={}", set.experiment_ids.join(",")).expect("String write");
            }
        } else {
            out.push_str("optimal_potency=NOT_ESTABLISHED\noptimal_cost=NOT_ESTABLISHED\noptimal_set_count=0\n");
        }
        writeln!(out, "greedy_potency={}", self.greedy.potency).expect("String write");
        writeln!(out, "greedy_residual_width={}", self.greedy.residual_width).expect("String write");
        writeln!(out, "greedy_cost={}", self.greedy.cost).expect("String write");
        writeln!(out, "greedy_set={}", self.greedy.experiment_ids.join(",")).expect("String write");
        writeln!(out, "greedy_matches_exact={}", self.greedy_is_optimal).expect("String write");
        for (id, status) in &self.candidate_statuses {
            writeln!(out, "candidate.{id}.status={}", status.as_str()).expect("String write");
        }
        out.push_str("objective=GUARANTEED_WORST_CASE_QUERY_WIDTH_REDUCTION\n");
        out.push_str("schedule_intervals=HALF_OPEN\n");
        out.push_str("pruning_rule=OBSERVATION_CONSTANT_OVER_ALL_DECLARED_WORLDS\n");
        out.push_str("authority=ASSERTED_NOT_AUTHENTICATED\n");
        out.push_str("source_authenticity=NOT_ESTABLISHED\n");
        out.push_str("physical_realizability=DECLARED_DESIGN_CONSISTENCY_ONLY\n");
        out.push_str("causal_truth=NOT_ESTABLISHED\n");
        out.push_str("field_effectiveness=NOT_ESTABLISHED\n");
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../fixtures/complementary-probes.psv");

    fn fixture() -> Problem {
        parse_input(FIXTURE).expect("fixture parses")
    }

    fn ids(set: &SelectedSet) -> Vec<String> {
        set.experiment_ids.clone()
    }

    #[test]
    fn complementary_row_and_column_probes_double_individual_potency() {
        let problem = fixture();
        let row = problem.experiments.iter().find(|experiment| experiment.id == "row_probe").unwrap();
        let col = problem.experiments.iter().find(|experiment| experiment.id == "column_probe").unwrap();
        assert_eq!(residual_width(&problem, &[]), 2);
        assert_eq!(2 - residual_width(&problem, &[row]), 1);
        assert_eq!(2 - residual_width(&problem, &[col]), 1);
        assert_eq!(2 - residual_width(&problem, &[row, col]), 2);
    }

    #[test]
    fn exhaustive_oracle_finds_complementary_optimum_and_prunes_only_inert_probe() {
        let report = analyze(&fixture());
        assert_eq!(report.search_status, "EXACT_OPTIMUM");
        assert!(report.pruning_equivalence);
        assert_eq!(report.unpruned.evaluated_subsets, 16);
        assert_eq!(report.pruned.evaluated_subsets, 8);
        assert_eq!(report.pruned_inert_ids, vec!["inert_probe"]);
        let best = report.unpruned.best.unwrap();
        assert_eq!(best.potency, 2);
        assert_eq!(best.residual_width, 0);
        assert_eq!(best.cost, 2);
        assert_eq!(ids(&best), vec!["column_probe", "row_probe"]);
    }

    #[test]
    fn deterministic_under_world_experiment_and_observation_row_permutation() {
        let original = analyze(&fixture()).render();
        let mut lines: Vec<&str> = FIXTURE.lines().collect();
        let marker = lines.iter().position(|line| *line == "EXPERIMENTS").unwrap();
        let obs_marker = lines.iter().position(|line| *line == "OBSERVATIONS").unwrap();
        let mut world_rows = lines[1..marker].to_vec();
        world_rows.reverse();
        let mut exp_rows = lines[marker + 2..obs_marker].to_vec();
        exp_rows.reverse();
        let mut obs_rows = lines[obs_marker + 2..].to_vec();
        obs_rows.reverse();
        let metadata = lines[0];
        let mut rebuilt = String::new();
        rebuilt.push_str(metadata);
        rebuilt.push('\n');
        rebuilt.push_str(WORLD_HEADER);
        rebuilt.push('\n');
        for line in world_rows { rebuilt.push_str(line); rebuilt.push('\n'); }
        rebuilt.push_str("EXPERIMENTS\n");
        rebuilt.push_str(EXPERIMENT_HEADER);
        rebuilt.push('\n');
        for line in exp_rows { rebuilt.push_str(line); rebuilt.push('\n'); }
        rebuilt.push_str("OBSERVATIONS\n");
        rebuilt.push_str(OBSERVATION_HEADER);
        rebuilt.push('\n');
        for line in obs_rows { rebuilt.push_str(line); rebuilt.push('\n'); }
        assert_eq!(original, analyze(&parse_input(&rebuilt).unwrap()).render());
    }

    #[test]
    fn blocked_authority_source_rights_and_safety_never_enter_search() {
        let mut problem = fixture();
        let exp = problem.experiments.iter_mut().find(|experiment| experiment.id == "row_probe").unwrap();
        exp.authority = "UNKNOWN".to_owned();
        assert_eq!(candidate_status(exp), CandidateStatus::RealizabilityNotEstablished);
        exp.authority = "BLOCKED".to_owned();
        assert_eq!(candidate_status(exp), CandidateStatus::BlockedAuthority);
        exp.authority = "ESTABLISHED".to_owned();
        exp.source_use = "UNKNOWN".to_owned();
        assert_eq!(candidate_status(exp), CandidateStatus::BlockedSourceUseOrRights);
        exp.source_use = "AUTHORIZED".to_owned();
        exp.rights_ethics = "BLOCKED".to_owned();
        assert_eq!(candidate_status(exp), CandidateStatus::BlockedSourceUseOrRights);
        exp.rights_ethics = "CLEARED".to_owned();
        exp.safety = "BLOCKED".to_owned();
        assert_eq!(candidate_status(exp), CandidateStatus::BlockedSafetyOrServiceFloor);
    }

    #[test]
    fn unknown_physical_realizability_is_not_promoted() {
        let mut problem = fixture();
        let exp = problem.experiments.iter_mut().find(|experiment| experiment.id == "row_probe").unwrap();
        exp.realizability = "UNKNOWN".to_owned();
        assert_eq!(candidate_status(exp), CandidateStatus::RealizabilityNotEstablished);
        exp.realizability = "NOT_REALIZABLE".to_owned();
        assert_eq!(candidate_status(exp), CandidateStatus::NotPhysicallyRealizable);
    }

    #[test]
    fn half_open_adjacent_regimes_need_verified_carryover_or_no_carryover() {
        let mut problem = fixture();
        let mut first = problem.experiments[0].clone();
        let mut second = problem.experiments[1].clone();
        first.unit_id = "pump-7".to_owned();
        second.unit_id = "pump-7".to_owned();
        first.start = 0; first.end = 10; first.regime = "inspection".to_owned();
        second.start = 10; second.end = 20; second.regime = "flush".to_owned();
        first.carryover = "UNKNOWN".to_owned();
        second.carryover = "UNKNOWN".to_owned();
        assert!(!schedule_compatible(&[&first, &second]));
        first.carryover = "NO_CARRYOVER".to_owned();
        second.carryover = "WASHOUT_VERIFIED".to_owned();
        assert!(schedule_compatible(&[&first, &second]));
    }

    #[test]
    fn overlapping_mutually_exclusive_regimes_are_rejected() {
        let problem = fixture();
        let mut first = problem.experiments[0].clone();
        let mut second = problem.experiments[1].clone();
        first.unit_id = "pump-7".to_owned();
        second.unit_id = "pump-7".to_owned();
        first.start = 0; first.end = 15; first.regime = "inspection".to_owned();
        second.start = 10; second.end = 20; second.regime = "flush".to_owned();
        first.carryover = "NO_CARRYOVER".to_owned();
        second.carryover = "NO_CARRYOVER".to_owned();
        assert!(!schedule_compatible(&[&first, &second]));
    }

    #[test]
    fn incomplete_universe_has_no_numeric_optimum() {
        let mut problem = fixture();
        problem.experiment_universe_complete = false;
        let report = analyze(&problem);
        assert_eq!(report.search_status, "INPUT_OR_EXPERIMENT_UNIVERSE_INCOMPLETE");
        assert!(report.unpruned.best.is_none());
        assert!(report.render().contains("optimal_potency=NOT_ESTABLISHED"));
    }

    #[test]
    fn search_cap_never_returns_a_partial_optimum() {
        let mut problem = fixture();
        problem.max_subsets = 4;
        let report = analyze(&problem);
        assert_eq!(report.search_status, "SEARCH_LIMIT_EXCEEDED");
        assert!(report.unpruned.best.is_none());
    }

    #[test]
    fn incomplete_observation_matrix_is_rejected() {
        let truncated = FIXTURE.replace("inert_probe|w11|constant", "");
        assert!(parse_input(&truncated).is_err());
    }

    #[test]
    fn duplicate_observation_is_rejected() {
        let duplicated = FIXTURE.replace(
            "row_probe|w00|r0",
            "row_probe|w00|r0\nrow_probe|w00|r0",
        );
        assert!(parse_input(&duplicated).is_err());
    }

    #[test]
    fn unknown_world_reference_is_rejected() {
        let input = FIXTURE.replace("row_probe|w00|r0", "row_probe|missing|r0");
        assert!(parse_input(&input).is_err());
    }

    #[test]
    fn constant_experiment_is_zero_potency_and_prunable() {
        let problem = fixture();
        let inert = problem.experiments.iter().find(|experiment| experiment.id == "inert_probe").unwrap();
        assert_eq!(residual_width(&problem, &[]), residual_width(&problem, &[inert]));
        assert!(inert_experiment(inert, &problem.worlds));
    }
}
