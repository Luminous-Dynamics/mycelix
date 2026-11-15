#![forbid(unsafe_code)]

// Exact E0 oracle for decision stability under finite-world expansion.
// Negative results are scoped to the declared candidate pool, never global safety.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

const WORLD_HEADER: &str = "world_id|safety_value|decision_value";
const CANDIDATE_HEADER: &str = "candidate_id|safety_value|decision_value|expansion_cost|population_mass_milli|evidence_state|admissibility|evidence_ref";
const MAX_ROWS: usize = 1_000_000;
const MAX_CANDIDATES: usize = 22;
const MAX_INPUT_BYTES: usize = 16 * 1024 * 1024;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct World {
    pub id: String,
    pub safety_value: i64,
    pub decision_value: i64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EvidenceState {
    Supported,
    Hypothetical,
    Unknown,
    Blocked,
}

impl EvidenceState {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Supported => "SUPPORTED",
            Self::Hypothetical => "HYPOTHETICAL",
            Self::Unknown => "UNKNOWN",
            Self::Blocked => "BLOCKED",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Admissibility {
    Admissible,
    Blocked,
    Unknown,
}

impl Admissibility {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Admissible => "ADMISSIBLE",
            Self::Blocked => "BLOCKED",
            Self::Unknown => "UNKNOWN",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExpansionCandidate {
    pub id: String,
    pub safety_value: i64,
    pub decision_value: i64,
    pub expansion_cost: u64,
    pub population_mass_milli: u64,
    pub evidence_state: EvidenceState,
    pub admissibility: Admissibility,
    pub evidence_ref: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CandidateDisposition {
    EvidenceSupported,
    Hypothetical,
    Unknown,
    Blocked,
}

impl CandidateDisposition {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::EvidenceSupported => "DECLARED_SUPPORTED_LABEL_AND_ADMISSIBLE",
            Self::Hypothetical => "HYPOTHETICAL_ADMISSIBLE",
            Self::Unknown => "ADMISSIBILITY_OR_EVIDENCE_UNKNOWN",
            Self::Blocked => "BLOCKED",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Problem {
    pub analysis_id: String,
    pub target_action: String,
    pub safety_floor: i64,
    pub expansion_cost_unit: String,
    pub world_set_status: String,
    pub candidate_pool_status: String,
    pub max_subsets: u64,
    pub base_worlds: Vec<World>,
    pub candidates: Vec<ExpansionCandidate>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RadiusTie {
    pub candidate_ids: Vec<String>,
    pub count: usize,
    pub expansion_cost: u128,
    pub population_mass_milli: u128,
    pub resulting_min_safety: i64,
    pub resulting_max_safety: i64,
    pub resulting_min_decision: i64,
    pub resulting_max_decision: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MetricRadius {
    pub metric: String,
    pub value: Option<u128>,
    pub ties: Vec<RadiusTie>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SearchResult {
    pub status: String,
    pub disposition: String,
    pub candidate_count: usize,
    pub evaluated_subsets: u64,
    pub minimum_count: MetricRadius,
    pub minimum_cost: MetricRadius,
    pub minimum_population_mass: MetricRadius,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Analysis {
    pub problem: Problem,
    pub base_min_safety: i64,
    pub base_max_safety: i64,
    pub base_min_decision: i64,
    pub base_max_decision: i64,
    pub supported: SearchResult,
    pub hypothetical: SearchResult,
    pub candidate_dispositions: Vec<(String, CandidateDisposition)>,
    pub unknown_candidate_count: usize,
    pub blocked_candidate_count: usize,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StabilityError(pub String);

impl std::fmt::Display for StabilityError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for StabilityError {}

fn fail(message: impl Into<String>) -> StabilityError {
    StabilityError(message.into())
}

fn valid_token(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 200
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.' | b':' | b'/')
        })
}

fn parse_u64(value: &str, name: &str) -> Result<u64, StabilityError> {
    value
        .parse::<u64>()
        .map_err(|error| fail(format!("invalid {name}: {error}")))
}

fn parse_i64(value: &str, name: &str) -> Result<i64, StabilityError> {
    value
        .parse::<i64>()
        .map_err(|error| fail(format!("invalid {name}: {error}")))
}

fn parse_completeness(value: &str, name: &str) -> Result<&'static str, StabilityError> {
    match value {
        "DECLARED_COMPLETE" => Ok("DECLARED_COMPLETE"),
        "NOT_ESTABLISHED" | "DECLARED_INCOMPLETE" => Ok("NOT_ESTABLISHED"),
        _ => Err(fail(format!("invalid {name} state {value}"))),
    }
}

fn parse_evidence_state(value: &str) -> Result<EvidenceState, StabilityError> {
    match value {
        "SUPPORTED" => Ok(EvidenceState::Supported),
        "HYPOTHETICAL" => Ok(EvidenceState::Hypothetical),
        "UNKNOWN" => Ok(EvidenceState::Unknown),
        "BLOCKED" => Ok(EvidenceState::Blocked),
        _ => Err(fail(format!("unknown evidence_state {value}"))),
    }
}

fn parse_admissibility(value: &str) -> Result<Admissibility, StabilityError> {
    match value {
        "ADMISSIBLE" => Ok(Admissibility::Admissible),
        "BLOCKED" => Ok(Admissibility::Blocked),
        "UNKNOWN" => Ok(Admissibility::Unknown),
        _ => Err(fail(format!("unknown admissibility {value}"))),
    }
}

pub fn parse_input(source: &str) -> Result<Problem, StabilityError> {
    if source.len() > MAX_INPUT_BYTES {
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
            let (key, value) = comment
                .trim()
                .split_once('=')
                .ok_or_else(|| fail(format!("line {}: malformed metadata", index + 1)))?;
            if !matches!(
                key,
                "analysis_id"
                    | "target_action"
                    | "safety_floor"
                    | "expansion_cost_unit"
                    | "world_set_status"
                    | "candidate_pool_status"
                    | "max_subsets"
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

    let required = |key: &str| -> Result<&str, StabilityError> {
        metadata
            .get(key)
            .map(String::as_str)
            .ok_or_else(|| fail(format!("missing metadata key {key}")))
    };

    let analysis_id = required("analysis_id")?.to_owned();
    let target_action = required("target_action")?.to_owned();
    let expansion_cost_unit = required("expansion_cost_unit")?.to_owned();
    if !valid_token(&analysis_id)
        || !valid_token(&target_action)
        || !valid_token(&expansion_cost_unit)
    {
        return Err(fail(
            "analysis_id, target_action and expansion_cost_unit must be valid tokens",
        ));
    }
    let safety_floor = parse_i64(required("safety_floor")?, "safety_floor")?;
    let max_subsets = parse_u64(required("max_subsets")?, "max_subsets")?;
    if max_subsets == 0 {
        return Err(fail("max_subsets must be positive"));
    }
    let world_set_status =
        parse_completeness(required("world_set_status")?, "world_set_status")?.to_owned();
    let candidate_pool_status =
        parse_completeness(required("candidate_pool_status")?, "candidate_pool_status")?.to_owned();

    let mut cursor = 0usize;
    if lines.get(cursor).map(|(_, line)| *line) != Some(WORLD_HEADER) {
        return Err(fail("expected exact base-world header"));
    }
    cursor += 1;

    let mut base_worlds = Vec::new();
    let mut world_ids = BTreeSet::<String>::new();
    while cursor < lines.len() {
        let (line_no, line) = lines[cursor];
        if line == "OMITTED_CANDIDATES" {
            break;
        }
        let fields: Vec<&str> = line.split('|').map(str::trim).collect();
        if fields.len() != 3 || !valid_token(fields[0]) {
            return Err(fail(format!("line {line_no}: invalid base-world row")));
        }
        if !world_ids.insert(fields[0].to_owned()) {
            return Err(fail(format!("line {line_no}: duplicate base-world id")));
        }
        base_worlds.push(World {
            id: fields[0].to_owned(),
            safety_value: parse_i64(fields[1], "base safety_value")?,
            decision_value: parse_i64(fields[2], "base decision_value")?,
        });
        if base_worlds.len() > MAX_ROWS {
            return Err(fail("base-world count exceeds limit"));
        }
        cursor += 1;
    }
    if base_worlds.is_empty() {
        return Err(fail("base-world set is empty"));
    }
    base_worlds.sort_by_key(|world| world.id.clone());

    if lines.get(cursor).map(|(_, line)| *line) != Some("OMITTED_CANDIDATES") {
        return Err(fail("missing OMITTED_CANDIDATES section"));
    }
    cursor += 1;
    if lines.get(cursor).map(|(_, line)| *line) != Some(CANDIDATE_HEADER) {
        return Err(fail("expected exact omitted-candidate header"));
    }
    cursor += 1;

    let mut candidates = Vec::new();
    let mut candidate_ids = BTreeSet::<String>::new();
    while cursor < lines.len() {
        let (line_no, line) = lines[cursor];
        let fields: Vec<&str> = line.split('|').map(str::trim).collect();
        if fields.len() != 8 || !valid_token(fields[0]) || !valid_token(fields[7]) {
            return Err(fail(format!(
                "line {line_no}: invalid omitted-candidate row"
            )));
        }
        if world_ids.contains(fields[0]) || !candidate_ids.insert(fields[0].to_owned()) {
            return Err(fail(format!(
                "line {line_no}: candidate id duplicates a base-world or candidate id"
            )));
        }
        candidates.push(ExpansionCandidate {
            id: fields[0].to_owned(),
            safety_value: parse_i64(fields[1], "candidate safety_value")?,
            decision_value: parse_i64(fields[2], "candidate decision_value")?,
            expansion_cost: parse_u64(fields[3], "expansion_cost")?,
            population_mass_milli: parse_u64(fields[4], "population_mass_milli")?,
            evidence_state: parse_evidence_state(fields[5])?,
            admissibility: parse_admissibility(fields[6])?,
            evidence_ref: fields[7].to_owned(),
        });
        if candidates.len() > MAX_ROWS {
            return Err(fail("candidate count exceeds limit"));
        }
        cursor += 1;
    }
    candidates.sort_by_key(|candidate| candidate.id.clone());

    Ok(Problem {
        analysis_id,
        target_action,
        safety_floor,
        expansion_cost_unit,
        world_set_status,
        candidate_pool_status,
        max_subsets,
        base_worlds,
        candidates,
    })
}

fn candidate_disposition(candidate: &ExpansionCandidate) -> CandidateDisposition {
    match (candidate.evidence_state, candidate.admissibility) {
        (EvidenceState::Blocked, _) | (_, Admissibility::Blocked) => CandidateDisposition::Blocked,
        (EvidenceState::Unknown, _) | (_, Admissibility::Unknown) => CandidateDisposition::Unknown,
        (EvidenceState::Supported, Admissibility::Admissible) => {
            CandidateDisposition::EvidenceSupported
        }
        (EvidenceState::Hypothetical, Admissibility::Admissible) => {
            CandidateDisposition::Hypothetical
        }
    }
}

fn world_ranges(worlds: &[World]) -> (i64, i64, i64, i64) {
    let min_safety = worlds
        .iter()
        .map(|world| world.safety_value)
        .min()
        .unwrap_or(0);
    let max_safety = worlds
        .iter()
        .map(|world| world.safety_value)
        .max()
        .unwrap_or(0);
    let min_decision = worlds
        .iter()
        .map(|world| world.decision_value)
        .min()
        .unwrap_or(0);
    let max_decision = worlds
        .iter()
        .map(|world| world.decision_value)
        .max()
        .unwrap_or(0);
    (min_safety, max_safety, min_decision, max_decision)
}

fn empty_radius(metric_name: &str) -> MetricRadius {
    MetricRadius {
        metric: metric_name.to_owned(),
        value: None,
        ties: Vec::new(),
    }
}

fn radius_for(
    metric_name: &str,
    dimension: usize,
    subsets: &[Vec<&ExpansionCandidate>],
    base_ranges: (i64, i64, i64, i64),
) -> MetricRadius {
    let mut ranked: Vec<(u128, RadiusTie)> = subsets
        .iter()
        .map(|subset| {
            let count = subset.len();
            let expansion_cost = subset
                .iter()
                .map(|candidate| u128::from(candidate.expansion_cost))
                .sum::<u128>();
            let population_mass_milli = subset
                .iter()
                .map(|candidate| u128::from(candidate.population_mass_milli))
                .sum::<u128>();
            let mut min_safety = base_ranges.0;
            let mut max_safety = base_ranges.1;
            let mut min_decision = base_ranges.2;
            let mut max_decision = base_ranges.3;
            let mut ids = subset
                .iter()
                .map(|candidate| candidate.id.clone())
                .collect::<Vec<_>>();
            for candidate in subset {
                min_safety = min_safety.min(candidate.safety_value);
                max_safety = max_safety.max(candidate.safety_value);
                min_decision = min_decision.min(candidate.decision_value);
                max_decision = max_decision.max(candidate.decision_value);
            }
            ids.sort();
            let tie = RadiusTie {
                candidate_ids: ids,
                count,
                expansion_cost,
                population_mass_milli,
                resulting_min_safety: min_safety,
                resulting_max_safety: max_safety,
                resulting_min_decision: min_decision,
                resulting_max_decision: max_decision,
            };
            let value = match dimension {
                0 => count as u128,
                1 => expansion_cost,
                _ => population_mass_milli,
            };
            (value, tie)
        })
        .collect();
    ranked.sort_by(|left, right| {
        left.0
            .cmp(&right.0)
            .then_with(|| left.1.candidate_ids.cmp(&right.1.candidate_ids))
    });
    let minimum = ranked.first().map(|entry| entry.0);
    let ties = ranked
        .into_iter()
        .filter(|entry| Some(entry.0) == minimum)
        .map(|entry| entry.1)
        .collect();
    MetricRadius {
        metric: metric_name.to_owned(),
        value: minimum,
        ties,
    }
}

fn search_limit(candidate_count: usize) -> SearchResult {
    SearchResult {
        status: "SEARCH_LIMIT_EXCEEDED".to_owned(),
        disposition: "NO_RADIUS_OR_OPTIMALITY_CLAIM".to_owned(),
        candidate_count,
        evaluated_subsets: 0,
        minimum_count: empty_radius("ADDED_WORLD_COUNT"),
        minimum_cost: empty_radius("ADDED_EXPANSION_COST"),
        minimum_population_mass: empty_radius("ADDED_POPULATION_MASS_MILLI"),
    }
}

fn search(
    problem: &Problem,
    candidates: &[&ExpansionCandidate],
    pool_is_evidence_supported: bool,
) -> SearchResult {
    let base_ranges = world_ranges(&problem.base_worlds);
    if base_ranges.0 < problem.safety_floor {
        return SearchResult {
            status: "BASE_SET_ALREADY_UNSAFE".to_owned(),
            disposition: "BASE_SET_ALREADY_BREACHES_SERVICE_FLOOR".to_owned(),
            candidate_count: candidates.len(),
            evaluated_subsets: 0,
            minimum_count: MetricRadius {
                metric: "ADDED_WORLD_COUNT".to_owned(),
                value: Some(0),
                ties: Vec::new(),
            },
            minimum_cost: MetricRadius {
                metric: format!("ADDED_COST:{}", problem.expansion_cost_unit),
                value: Some(0),
                ties: Vec::new(),
            },
            minimum_population_mass: MetricRadius {
                metric: "ADDED_POPULATION_MASS_MILLI".to_owned(),
                value: Some(0),
                ties: Vec::new(),
            },
        };
    }
    if candidates.len() > MAX_CANDIDATES {
        return search_limit(candidates.len());
    }
    let total = 1_u64 << candidates.len();
    if total > problem.max_subsets {
        return search_limit(candidates.len());
    }

    let mut defeating_subsets = Vec::<Vec<&ExpansionCandidate>>::new();
    let mut evaluated_subsets = 0u64;
    for mask in 0..total {
        evaluated_subsets += 1;
        let selected = candidates
            .iter()
            .enumerate()
            .filter_map(|(index, candidate)| {
                if mask & (1_u64 << index) != 0 {
                    Some(*candidate)
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        if selected.is_empty() {
            continue;
        }
        let min_safety = selected
            .iter()
            .map(|candidate| candidate.safety_value)
            .chain(problem.base_worlds.iter().map(|world| world.safety_value))
            .min()
            .unwrap_or(i64::MAX);
        if min_safety < problem.safety_floor {
            defeating_subsets.push(selected);
        }
    }

    if defeating_subsets.is_empty() {
        let unknown = problem
            .candidates
            .iter()
            .filter(|candidate| candidate_disposition(candidate) == CandidateDisposition::Unknown)
            .count();
        let disposition = if !pool_is_evidence_supported {
            "HYPOTHETICAL_STRESS_ONLY_NO_DEFEATER_FOUND"
        } else if problem.candidate_pool_status != "DECLARED_COMPLETE" {
            "NO_DEFEATER_FOUND_IN_INCOMPLETE_POOL"
        } else if unknown > 0 {
            "NO_DEFEATER_FOUND_BUT_UNKNOWN_CANDIDATES_REMAIN"
        } else {
            "NO_DEFEATER_WITHIN_DECLARED_SUPPORTED_POOL_NOT_GLOBAL_SAFETY"
        };
        return SearchResult {
            status: "EXACT_SEARCH_NO_DEFEATER_FOUND".to_owned(),
            disposition: disposition.to_owned(),
            candidate_count: candidates.len(),
            evaluated_subsets,
            minimum_count: empty_radius("ADDED_WORLD_COUNT"),
            minimum_cost: empty_radius(&format!("ADDED_COST:{}", problem.expansion_cost_unit)),
            minimum_population_mass: empty_radius("ADDED_POPULATION_MASS_MILLI"),
        };
    }

    SearchResult {
        status: "EXACT_SEARCH_DEFEATER_FOUND".to_owned(),
        disposition: if pool_is_evidence_supported {
            "DECLARED_SUPPORTED_LABEL_DEFEATER_WITHIN_POOL".to_owned()
        } else {
            "HYPOTHETICAL_DEFEATER_STRESS_ONLY".to_owned()
        },
        candidate_count: candidates.len(),
        evaluated_subsets,
        minimum_count: radius_for("ADDED_WORLD_COUNT", 0, &defeating_subsets, base_ranges),
        minimum_cost: radius_for(
            &format!("ADDED_COST:{}", problem.expansion_cost_unit),
            1,
            &defeating_subsets,
            base_ranges,
        ),
        minimum_population_mass: radius_for(
            "ADDED_POPULATION_MASS_MILLI",
            2,
            &defeating_subsets,
            base_ranges,
        ),
    }
}

pub fn analyze(problem: &Problem) -> Analysis {
    let candidate_dispositions = problem
        .candidates
        .iter()
        .map(|candidate| (candidate.id.clone(), candidate_disposition(candidate)))
        .collect::<Vec<_>>();
    let supported = problem
        .candidates
        .iter()
        .filter(|candidate| {
            candidate_disposition(candidate) == CandidateDisposition::EvidenceSupported
        })
        .collect::<Vec<_>>();
    let hypothetical = problem
        .candidates
        .iter()
        .filter(|candidate| candidate_disposition(candidate) == CandidateDisposition::Hypothetical)
        .collect::<Vec<_>>();
    let unknown_candidate_count = candidate_dispositions
        .iter()
        .filter(|(_, disposition)| *disposition == CandidateDisposition::Unknown)
        .count();
    let blocked_candidate_count = candidate_dispositions
        .iter()
        .filter(|(_, disposition)| *disposition == CandidateDisposition::Blocked)
        .count();
    let ranges = world_ranges(&problem.base_worlds);
    Analysis {
        problem: problem.clone(),
        base_min_safety: ranges.0,
        base_max_safety: ranges.1,
        base_min_decision: ranges.2,
        base_max_decision: ranges.3,
        supported: search(problem, &supported, true),
        hypothetical: search(problem, &hypothetical, false),
        candidate_dispositions,
        unknown_candidate_count,
        blocked_candidate_count,
    }
}

fn render_radius(out: &mut String, prefix: &str, radius: &MetricRadius) {
    writeln!(out, "{prefix}.metric={}", radius.metric).expect("String write");
    match radius.value {
        Some(value) => writeln!(out, "{prefix}.value={value}").expect("String write"),
        None => writeln!(out, "{prefix}.value=NOT_ESTABLISHED").expect("String write"),
    }
    writeln!(out, "{prefix}.tie_count={}", radius.ties.len()).expect("String write");
    for (index, tie) in radius.ties.iter().enumerate() {
        let key = format!("{prefix}.tie.{index:04}");
        writeln!(out, "{key}.candidate_ids={}", tie.candidate_ids.join(",")).expect("String write");
        writeln!(out, "{key}.count={}", tie.count).expect("String write");
        writeln!(out, "{key}.expansion_cost={}", tie.expansion_cost).expect("String write");
        writeln!(
            out,
            "{key}.population_mass_milli={}",
            tie.population_mass_milli
        )
        .expect("String write");
        writeln!(
            out,
            "{key}.resulting_min_safety={}",
            tie.resulting_min_safety
        )
        .expect("String write");
        writeln!(
            out,
            "{key}.resulting_max_safety={}",
            tie.resulting_max_safety
        )
        .expect("String write");
        writeln!(
            out,
            "{key}.resulting_min_decision={}",
            tie.resulting_min_decision
        )
        .expect("String write");
        writeln!(
            out,
            "{key}.resulting_max_decision={}",
            tie.resulting_max_decision
        )
        .expect("String write");
    }
}

impl Analysis {
    pub fn render(&self) -> String {
        let mut out = String::new();
        writeln!(out, "schema=fsre.worldset-stability-e0").expect("String write");
        writeln!(out, "version=0.1.0").expect("String write");
        writeln!(out, "analysis_id={}", self.problem.analysis_id).expect("String write");
        writeln!(out, "target_action={}", self.problem.target_action).expect("String write");
        writeln!(out, "safety_floor={}", self.problem.safety_floor).expect("String write");
        writeln!(
            out,
            "expansion_cost_unit={}",
            self.problem.expansion_cost_unit
        )
        .expect("String write");
        writeln!(out, "world_set_status={}", self.problem.world_set_status).expect("String write");
        writeln!(
            out,
            "candidate_pool_status={}",
            self.problem.candidate_pool_status
        )
        .expect("String write");
        writeln!(out, "base_world_count={}", self.problem.base_worlds.len()).expect("String write");
        writeln!(out, "candidate_count={}", self.problem.candidates.len()).expect("String write");
        writeln!(out, "base_min_safety={}", self.base_min_safety).expect("String write");
        writeln!(out, "base_max_safety={}", self.base_max_safety).expect("String write");
        writeln!(out, "base_min_decision={}", self.base_min_decision).expect("String write");
        writeln!(out, "base_max_decision={}", self.base_max_decision).expect("String write");
        writeln!(
            out,
            "base_action_status={}",
            if self.base_min_safety >= self.problem.safety_floor {
                "SAFE_WITHIN_DECLARED_BASE_SET"
            } else {
                "BASE_SET_ALREADY_UNSAFE"
            }
        )
        .expect("String write");
        writeln!(
            out,
            "unknown_candidate_count={}",
            self.unknown_candidate_count
        )
        .expect("String write");
        writeln!(
            out,
            "blocked_candidate_count={}",
            self.blocked_candidate_count
        )
        .expect("String write");
        writeln!(out, "supported_search_status={}", self.supported.status).expect("String write");
        writeln!(
            out,
            "supported_search_disposition={}",
            self.supported.disposition
        )
        .expect("String write");
        writeln!(
            out,
            "supported_evaluated_subsets={}",
            self.supported.evaluated_subsets
        )
        .expect("String write");
        render_radius(
            &mut out,
            "supported.minimum_count",
            &self.supported.minimum_count,
        );
        render_radius(
            &mut out,
            "supported.minimum_cost",
            &self.supported.minimum_cost,
        );
        render_radius(
            &mut out,
            "supported.minimum_population_mass",
            &self.supported.minimum_population_mass,
        );
        writeln!(
            out,
            "hypothetical_search_status={}",
            self.hypothetical.status
        )
        .expect("String write");
        writeln!(
            out,
            "hypothetical_search_disposition={}",
            self.hypothetical.disposition
        )
        .expect("String write");
        writeln!(
            out,
            "hypothetical_evaluated_subsets={}",
            self.hypothetical.evaluated_subsets
        )
        .expect("String write");
        render_radius(
            &mut out,
            "hypothetical.minimum_count",
            &self.hypothetical.minimum_count,
        );
        render_radius(
            &mut out,
            "hypothetical.minimum_cost",
            &self.hypothetical.minimum_cost,
        );
        render_radius(
            &mut out,
            "hypothetical.minimum_population_mass",
            &self.hypothetical.minimum_population_mass,
        );
        for candidate in &self.problem.candidates {
            let id = &candidate.id;
            let disposition = candidate_disposition(candidate);
            writeln!(out, "candidate.{id}.disposition={}", disposition.as_str())
                .expect("String write");
            writeln!(
                out,
                "candidate.{id}.evidence_state_label={}",
                candidate.evidence_state.as_str()
            )
            .expect("String write");
            writeln!(
                out,
                "candidate.{id}.admissibility_label={}",
                candidate.admissibility.as_str()
            )
            .expect("String write");
            writeln!(
                out,
                "candidate.{id}.evidence_ref={}",
                candidate.evidence_ref
            )
            .expect("String write");
        }
        out.push_str("candidate_state_labels=CALLER_ASSERTED_NOT_AUTHENTICATED\n");
        out.push_str("negative_result_scope=DECLARED_FINITE_CANDIDATE_POOL_ONLY\n");
        out.push_str("declared_completeness_is_empirical_coverage=FALSE\n");
        out.push_str("physical_realizability=NOT_ESTABLISHED_BY_THIS_ORACLE\n");
        out.push_str("source_authenticity=NOT_ESTABLISHED_BY_THIS_ORACLE\n");
        out.push_str("institutional_authority=NOT_ESTABLISHED_BY_THIS_ORACLE\n");
        out.push_str("global_safety=NOT_ESTABLISHED\n");
        out.push_str("causal_truth=NOT_ESTABLISHED\n");
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../fixtures/omitted-worlds.psv");

    fn fixture() -> Problem {
        parse_input(FIXTURE).expect("fixture parses")
    }

    #[test]
    fn exact_fixture_reports_count_cost_and_mass_stability_radii() {
        let report = analyze(&fixture());
        assert_eq!(report.supported.status, "EXACT_SEARCH_DEFEATER_FOUND");
        assert_eq!(report.supported.evaluated_subsets, 4);
        assert_eq!(report.supported.minimum_count.value, Some(1));
        assert_eq!(report.supported.minimum_count.ties.len(), 2);
        assert_eq!(report.supported.minimum_cost.value, Some(3));
        assert_eq!(
            report.supported.minimum_cost.ties[0].candidate_ids,
            vec!["omitted-b"]
        );
        assert_eq!(report.supported.minimum_population_mass.value, Some(100));
        assert_eq!(
            report.supported.minimum_population_mass.ties[0].candidate_ids,
            vec!["omitted-a"]
        );
        assert_eq!(report.hypothetical.minimum_count.value, Some(1));
        assert_eq!(
            report.hypothetical.minimum_count.ties[0].candidate_ids,
            vec!["hypothetical-c"]
        );
        assert_eq!(report.unknown_candidate_count, 1);
        assert_eq!(report.blocked_candidate_count, 1);
        assert!(report
            .render()
            .contains("candidate_state_labels=CALLER_ASSERTED_NOT_AUTHENTICATED"));
        assert!(report
            .render()
            .contains("candidate.omitted-a.evidence_ref=evidence-a"));
    }

    #[test]
    fn independent_enumerator_matches_three_distinct_radius_metrics() {
        let base_floor = 60_i64;
        let base_safety = [80_i64, 70_i64];
        let candidates = [
            ("omitted-a", 50_i64, 9_u128, 100_u128),
            ("omitted-b", 55_i64, 3_u128, 700_u128),
        ];
        let mut defeating = Vec::<(u128, u128, u128, Vec<&str>)>::new();
        for mask in 1_u8..4 {
            let chosen = candidates
                .iter()
                .enumerate()
                .filter(|(index, _)| mask & (1_u8 << index) != 0)
                .collect::<Vec<_>>();
            let worst = base_safety
                .iter()
                .copied()
                .chain(chosen.iter().map(|(_, candidate)| candidate.1))
                .min()
                .unwrap();
            if worst < base_floor {
                defeating.push((
                    chosen.len() as u128,
                    chosen.iter().map(|(_, candidate)| candidate.2).sum(),
                    chosen.iter().map(|(_, candidate)| candidate.3).sum(),
                    chosen.iter().map(|(_, candidate)| candidate.0).collect(),
                ));
            }
        }
        let expected_count = defeating.iter().map(|item| item.0).min().unwrap();
        let expected_cost = defeating.iter().map(|item| item.1).min().unwrap();
        let expected_mass = defeating.iter().map(|item| item.2).min().unwrap();
        assert_eq!(expected_count, 1);
        assert_eq!(expected_cost, 3);
        assert_eq!(expected_mass, 100);

        let report = analyze(&fixture());
        assert_eq!(report.supported.minimum_count.value, Some(expected_count));
        assert_eq!(report.supported.minimum_cost.value, Some(expected_cost));
        assert_eq!(
            report.supported.minimum_population_mass.value,
            Some(expected_mass)
        );
    }

    #[test]
    fn incomplete_candidate_pool_does_not_turn_a_witness_into_global_claim() {
        let mut problem = fixture();
        problem.candidate_pool_status = "NOT_ESTABLISHED".to_owned();
        let report = analyze(&problem);
        assert_eq!(
            report.supported.disposition,
            "DECLARED_SUPPORTED_LABEL_DEFEATER_WITHIN_POOL"
        );
        assert!(report.render().contains("global_safety=NOT_ESTABLISHED"));
    }

    #[test]
    fn unknown_and_blocked_candidates_do_not_enter_supported_search() {
        let report = analyze(&fixture());
        assert_eq!(report.supported.candidate_count, 2);
        assert_eq!(report.hypothetical.candidate_count, 1);
        assert_eq!(report.unknown_candidate_count, 1);
        assert_eq!(report.blocked_candidate_count, 1);
    }

    #[test]
    fn base_set_already_unsafe_has_zero_stability_radius() {
        let mut problem = fixture();
        problem.base_worlds[0].safety_value = 10;
        let report = analyze(&problem);
        assert_eq!(report.supported.status, "BASE_SET_ALREADY_UNSAFE");
        assert_eq!(report.supported.minimum_count.value, Some(0));
        assert_eq!(report.supported.minimum_cost.value, Some(0));
    }

    #[test]
    fn search_cap_never_returns_a_partial_radius() {
        let mut problem = fixture();
        problem.max_subsets = 2;
        let report = analyze(&problem);
        assert_eq!(report.supported.status, "SEARCH_LIMIT_EXCEEDED");
        assert_eq!(report.supported.minimum_count.value, None);
        assert!(report
            .render()
            .contains("supported.minimum_count.value=NOT_ESTABLISHED"));
    }

    #[test]
    fn no_defeater_in_incomplete_pool_is_not_promoted_to_global_safety() {
        let mut problem = fixture();
        problem
            .candidates
            .retain(|candidate| candidate.evidence_state != EvidenceState::Supported);
        problem
            .candidates
            .retain(|candidate| candidate.evidence_state != EvidenceState::Unknown);
        problem.candidate_pool_status = "NOT_ESTABLISHED".to_owned();
        let report = analyze(&problem);
        assert_eq!(
            report.supported.disposition,
            "NO_DEFEATER_FOUND_IN_INCOMPLETE_POOL"
        );
    }

    #[test]
    fn candidate_id_collision_and_duplicate_rows_are_rejected() {
        let duplicate = FIXTURE.replace(
            "omitted-a|50|20|9|100|SUPPORTED|ADMISSIBLE|evidence-a",
            "base-a|50|20|9|100|SUPPORTED|ADMISSIBLE|evidence-a",
        );
        assert!(parse_input(&duplicate).is_err());
    }

    #[test]
    fn unknown_schema_fields_and_states_are_rejected() {
        let unknown_meta = FIXTURE.replace(
            "#max_subsets=1000",
            "#max_subsets=1000\n#pretend_coverage=COMPLETE",
        );
        assert!(parse_input(&unknown_meta).is_err());
        let unknown_state = FIXTURE.replace("SUPPORTED", "PROBABLY_SUPPORTED");
        assert!(parse_input(&unknown_state).is_err());
    }

    #[test]
    fn row_permutations_preserve_exact_output() {
        let original = analyze(&fixture()).render();
        let lines = FIXTURE.lines().collect::<Vec<_>>();
        let world_header = lines.iter().position(|line| *line == WORLD_HEADER).unwrap();
        let candidates_section = lines
            .iter()
            .position(|line| *line == "OMITTED_CANDIDATES")
            .unwrap();
        let metadata = &lines[..world_header];
        let worlds = &lines[world_header + 1..candidates_section];
        let candidate_header = candidates_section + 1;
        let mut world_rows = worlds.to_vec();
        world_rows.reverse();
        let mut candidate_rows = lines[candidate_header + 1..].to_vec();
        candidate_rows.reverse();
        let mut input = String::new();
        for line in metadata {
            writeln!(input, "{line}").unwrap();
        }
        writeln!(input, "{WORLD_HEADER}").unwrap();
        for line in world_rows {
            writeln!(input, "{line}").unwrap();
        }
        writeln!(input, "OMITTED_CANDIDATES").unwrap();
        writeln!(input, "{CANDIDATE_HEADER}").unwrap();
        for line in candidate_rows {
            writeln!(input, "{line}").unwrap();
        }
        assert_eq!(original, analyze(&parse_input(&input).unwrap()).render());
    }
}
