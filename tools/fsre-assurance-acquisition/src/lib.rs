#![forbid(unsafe_code)]

// Exact E0 set-cover planner for acquisition of missing assurance evidence.
//
// It identifies candidate plans only. Coverage of a declared obligation is
// not a guarantee that an inspection succeeds, qualifies or authorizes action.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

const HEADER: &str = "acquisition_id|acquisition_type|cost|scope_sha256|authority|rights|safety|physical|schedule|data_access|covers|prerequisites|resource_slot|source_group|source_ref";
const MAX_INPUT_BYTES: usize = 4 * 1024 * 1024;
const MAX_OBLIGATIONS: usize = 128;
const MAX_ACQUISITIONS: usize = 20;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Gate {
    Pass,
    Blocked,
    Unknown,
}

impl Gate {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Pass => "PASS",
            Self::Blocked => "BLOCKED",
            Self::Unknown => "UNKNOWN",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CandidateDisposition {
    Admissible,
    Blocked,
    Unknown,
    ScopeMismatch,
}

impl CandidateDisposition {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Admissible => "ADMISSIBLE_WITHIN_DECLARED_GATES",
            Self::Blocked => "BLOCKED",
            Self::Unknown => "UNKNOWN_GATE",
            Self::ScopeMismatch => "SCOPE_MISMATCH",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Acquisition {
    pub id: String,
    pub acquisition_type: String,
    pub cost: u64,
    pub scope_sha256: String,
    pub authority: Gate,
    pub rights: Gate,
    pub safety: Gate,
    pub physical: Gate,
    pub schedule: Gate,
    pub data_access: Gate,
    pub covers: Vec<String>,
    pub prerequisites: Vec<String>,
    pub resource_slot: String,
    pub source_group: String,
    pub source_ref: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Problem {
    pub episode_id: String,
    pub scope_sha256: String,
    pub budget: u64,
    pub cost_unit: String,
    pub max_subsets: u64,
    pub obligations: Vec<String>,
    pub acquisitions: Vec<Acquisition>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Plan {
    pub acquisition_ids: Vec<String>,
    pub cost: u128,
    pub covered_obligations: Vec<String>,
    pub uncovered_obligations: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SearchResult {
    pub status: String,
    pub candidate_count: usize,
    pub evaluated_subsets: u64,
    pub complete_plan_count: u64,
    pub minimum_cost: Option<u128>,
    pub minimum_cost_ties: Vec<Plan>,
    pub minimum_count: Option<usize>,
    pub minimum_count_ties: Vec<Plan>,
    pub best_partial: Option<Plan>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Analysis {
    pub problem: Problem,
    pub dispositions: Vec<(String, CandidateDisposition)>,
    pub blocked_count: usize,
    pub unknown_count: usize,
    pub scope_mismatch_count: usize,
    pub result: SearchResult,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AcquisitionError(pub String);

impl std::fmt::Display for AcquisitionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for AcquisitionError {}

fn fail(message: impl Into<String>) -> AcquisitionError {
    AcquisitionError(message.into())
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

fn parse_u64(value: &str, name: &str) -> Result<u64, AcquisitionError> {
    value
        .parse::<u64>()
        .map_err(|error| fail(format!("invalid {name}: {error}")))
}

fn parse_gate(value: &str, name: &str) -> Result<Gate, AcquisitionError> {
    match value {
        "PASS" => Ok(Gate::Pass),
        "BLOCKED" => Ok(Gate::Blocked),
        "UNKNOWN" => Ok(Gate::Unknown),
        _ => Err(fail(format!("unknown {name} gate {value}"))),
    }
}

fn parse_list(value: &str, name: &str, allow_empty: bool) -> Result<Vec<String>, AcquisitionError> {
    if value == "-" && allow_empty {
        return Ok(Vec::new());
    }
    let mut values = value
        .split(',')
        .map(str::trim)
        .map(str::to_owned)
        .collect::<Vec<_>>();
    if values.is_empty()
        || values.iter().any(|item| !valid_token(item))
        || values.iter().collect::<BTreeSet<_>>().len() != values.len()
    {
        return Err(fail(format!("invalid, empty or duplicate {name} list")));
    }
    values.sort();
    Ok(values)
}

pub fn parse_input(source: &str) -> Result<Problem, AcquisitionError> {
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
                "episode_id"
                    | "scope_sha256"
                    | "budget"
                    | "cost_unit"
                    | "max_subsets"
                    | "unresolved_obligations"
            ) {
                return Err(fail(format!("line {}: unknown metadata key {key}", index + 1)));
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

    let required = |key: &str| -> Result<&str, AcquisitionError> {
        metadata
            .get(key)
            .map(String::as_str)
            .ok_or_else(|| fail(format!("missing metadata key {key}")))
    };

    let episode_id = required("episode_id")?.to_owned();
    let scope_sha256 = required("scope_sha256")?.to_owned();
    let budget = parse_u64(required("budget")?, "budget")?;
    let cost_unit = required("cost_unit")?.to_owned();
    let max_subsets = parse_u64(required("max_subsets")?, "max_subsets")?;
    if !valid_token(&episode_id) || !valid_token(&cost_unit) {
        return Err(fail("episode_id and cost_unit must be valid tokens"));
    }
    if !valid_sha256(&scope_sha256) {
        return Err(fail("scope_sha256 must be 64 lowercase hexadecimal characters"));
    }
    if max_subsets == 0 {
        return Err(fail("max_subsets must be positive"));
    }
    let obligations = parse_list(required("unresolved_obligations")?, "obligation", false)?;
    if obligations.len() > MAX_OBLIGATIONS {
        return Err(fail("obligation count exceeds limit"));
    }

    if lines.first().map(|(_, line)| *line) != Some("ACQUISITIONS") {
        return Err(fail("expected ACQUISITIONS section"));
    }
    if lines.get(1).map(|(_, line)| *line) != Some(HEADER) {
        return Err(fail("expected exact acquisition header"));
    }

    let mut acquisitions = Vec::new();
    let mut ids = BTreeSet::<String>::new();
    for (line_no, line) in lines.iter().skip(2) {
        let fields = line.split('|').map(str::trim).collect::<Vec<_>>();
        if fields.len() != 15
            || !valid_token(fields[0])
            || !valid_token(fields[1])
            || !valid_sha256(fields[3])
            || !valid_token(fields[12]) && fields[12] != "-"
            || !valid_token(fields[13]) && fields[13] != "-"
            || !valid_token(fields[14])
        {
            return Err(fail(format!("line {line_no}: invalid acquisition row")));
        }
        if !ids.insert(fields[0].to_owned()) {
            return Err(fail(format!("line {line_no}: duplicate acquisition id")));
        }
        let covers = parse_list(fields[10], "coverage", true)?;
        if covers.iter().any(|item| !obligations.contains(item)) {
            return Err(fail(format!(
                "line {line_no}: acquisition covers an undeclared obligation"
            )));
        }
        let prerequisites = parse_list(fields[11], "prerequisite", true)?;
        acquisitions.push(Acquisition {
            id: fields[0].to_owned(),
            acquisition_type: fields[1].to_owned(),
            cost: parse_u64(fields[2], "acquisition cost")?,
            scope_sha256: fields[3].to_owned(),
            authority: parse_gate(fields[4], "authority")?,
            rights: parse_gate(fields[5], "rights")?,
            safety: parse_gate(fields[6], "safety")?,
            physical: parse_gate(fields[7], "physical")?,
            schedule: parse_gate(fields[8], "schedule")?,
            data_access: parse_gate(fields[9], "data_access")?,
            covers,
            prerequisites,
            resource_slot: fields[12].to_owned(),
            source_group: fields[13].to_owned(),
            source_ref: fields[14].to_owned(),
        });
        if acquisitions.len() > MAX_ACQUISITIONS {
            return Err(fail(format!("acquisition count exceeds {MAX_ACQUISITIONS}")));
        }
    }
    if acquisitions.is_empty() {
        return Err(fail("at least one acquisition candidate is required"));
    }
    acquisitions.sort_by_key(|item| item.id.clone());

    for item in &acquisitions {
        if item.prerequisites.contains(&item.id) {
            return Err(fail(format!("{} declares itself as a prerequisite", item.id)));
        }
        for prerequisite in &item.prerequisites {
            if !ids.contains(prerequisite) {
                return Err(fail(format!(
                    "{} has unknown prerequisite {}",
                    item.id, prerequisite
                )));
            }
        }
    }

    Ok(Problem {
        episode_id,
        scope_sha256,
        budget,
        cost_unit,
        max_subsets,
        obligations,
        acquisitions,
    })
}

fn disposition(problem: &Problem, acquisition: &Acquisition) -> CandidateDisposition {
    if acquisition.scope_sha256 != problem.scope_sha256 {
        return CandidateDisposition::ScopeMismatch;
    }
    let gates = [
        acquisition.authority,
        acquisition.rights,
        acquisition.safety,
        acquisition.physical,
        acquisition.schedule,
        acquisition.data_access,
    ];
    if gates.contains(&Gate::Blocked) {
        CandidateDisposition::Blocked
    } else if gates.contains(&Gate::Unknown) {
        CandidateDisposition::Unknown
    } else {
        CandidateDisposition::Admissible
    }
}

fn plan_from_mask(
    problem: &Problem,
    admissible: &[&Acquisition],
    mask: u64,
) -> Option<Plan> {
    let selected = admissible
        .iter()
        .enumerate()
        .filter_map(|(index, item)| {
            if mask & (1_u64 << index) != 0 {
                Some(*item)
            } else {
                None
            }
        })
        .collect::<Vec<_>>();

    let total_cost = selected
        .iter()
        .try_fold(0_u128, |sum, item| sum.checked_add(u128::from(item.cost)))?;
    if total_cost > u128::from(problem.budget) {
        return None;
    }

    let selected_ids = selected
        .iter()
        .map(|item| item.id.as_str())
        .collect::<BTreeSet<_>>();
    let mut resources = BTreeSet::<&str>::new();
    for item in &selected {
        if item
            .prerequisites
            .iter()
            .any(|prerequisite| !selected_ids.contains(prerequisite.as_str()))
        {
            return None;
        }
        if item.resource_slot != "-" && !resources.insert(item.resource_slot.as_str()) {
            return None;
        }
    }

    let covered = selected
        .iter()
        .flat_map(|item| item.covers.iter().cloned())
        .collect::<BTreeSet<_>>();
    let uncovered = problem
        .obligations
        .iter()
        .filter(|obligation| !covered.contains(*obligation))
        .cloned()
        .collect::<Vec<_>>();
    let acquisition_ids = selected
        .iter()
        .map(|item| item.id.clone())
        .collect::<Vec<_>>();
    let covered_obligations = covered.into_iter().collect::<Vec<_>>();

    Some(Plan {
        acquisition_ids,
        cost: total_cost,
        covered_obligations,
        uncovered_obligations: uncovered,
    })
}

fn partial_is_better(candidate: &Plan, current: &Plan) -> bool {
    candidate
        .covered_obligations
        .len()
        .cmp(&current.covered_obligations.len())
        .then_with(|| current.cost.cmp(&candidate.cost))
        .then_with(|| current.acquisition_ids.len().cmp(&candidate.acquisition_ids.len()))
        .then_with(|| current.acquisition_ids.cmp(&candidate.acquisition_ids))
        == std::cmp::Ordering::Greater
}

fn search_limit(candidate_count: usize) -> SearchResult {
    SearchResult {
        status: "SEARCH_LIMIT_EXCEEDED_NO_OPTIMALITY_CLAIM".to_owned(),
        candidate_count,
        evaluated_subsets: 0,
        complete_plan_count: 0,
        minimum_cost: None,
        minimum_cost_ties: Vec::new(),
        minimum_count: None,
        minimum_count_ties: Vec::new(),
        best_partial: None,
    }
}

fn search(problem: &Problem, admissible: &[&Acquisition]) -> SearchResult {
    if admissible.len() > MAX_ACQUISITIONS {
        return search_limit(admissible.len());
    }
    let total = 1_u64 << admissible.len();
    if total > problem.max_subsets {
        return search_limit(admissible.len());
    }

    let mut evaluated_subsets = 0u64;
    let mut complete_plan_count = 0u64;
    let mut minimum_cost: Option<u128> = None;
    let mut minimum_cost_ties = Vec::<Plan>::new();
    let mut minimum_count: Option<usize> = None;
    let mut minimum_count_ties = Vec::<Plan>::new();
    let mut best_partial: Option<Plan> = None;

    for mask in 0..total {
        evaluated_subsets += 1;
        let Some(plan) = plan_from_mask(problem, admissible, mask) else {
            continue;
        };
        let is_complete = plan.uncovered_obligations.is_empty();
        if let Some(current) = &best_partial {
            if partial_is_better(&plan, current) {
                best_partial = Some(plan.clone());
            }
        } else {
            best_partial = Some(plan.clone());
        }

        if !is_complete {
            continue;
        }
        complete_plan_count += 1;

        match minimum_cost {
            None => {
                minimum_cost = Some(plan.cost);
                minimum_cost_ties.clear();
                minimum_cost_ties.push(plan.clone());
            }
            Some(current) if plan.cost < current => {
                minimum_cost = Some(plan.cost);
                minimum_cost_ties.clear();
                minimum_cost_ties.push(plan.clone());
            }
            Some(current) if plan.cost == current => minimum_cost_ties.push(plan.clone()),
            _ => {}
        }

        let count = plan.acquisition_ids.len();
        match minimum_count {
            None => {
                minimum_count = Some(count);
                minimum_count_ties.clear();
                minimum_count_ties.push(plan);
            }
            Some(current) if count < current => {
                minimum_count = Some(count);
                minimum_count_ties.clear();
                minimum_count_ties.push(plan);
            }
            Some(current) if count == current => minimum_count_ties.push(plan),
            _ => {}
        }
    }

    let sort_plans = |plans: &mut Vec<Plan>| {
        plans.sort_by(|left, right| left.acquisition_ids.cmp(&right.acquisition_ids));
    };
    sort_plans(&mut minimum_cost_ties);
    sort_plans(&mut minimum_count_ties);

    let status = if problem.obligations.is_empty() {
        "NO_ACQUISITIONS_REQUIRED"
    } else if complete_plan_count > 0 {
        "EXACT_COMPLETE_COVER_FOUND"
    } else if best_partial
        .as_ref()
        .is_some_and(|plan| !plan.covered_obligations.is_empty())
    {
        "PARTIAL_COVERAGE_ONLY"
    } else {
        "NO_ADMISSIBLE_COVERAGE"
    };

    SearchResult {
        status: status.to_owned(),
        candidate_count: admissible.len(),
        evaluated_subsets,
        complete_plan_count,
        minimum_cost,
        minimum_cost_ties,
        minimum_count,
        minimum_count_ties,
        best_partial,
    }
}

pub fn analyze(problem: &Problem) -> Analysis {
    let dispositions = problem
        .acquisitions
        .iter()
        .map(|item| (item.id.clone(), disposition(problem, item)))
        .collect::<Vec<_>>();
    let blocked_count = dispositions
        .iter()
        .filter(|(_, disposition)| *disposition == CandidateDisposition::Blocked)
        .count();
    let unknown_count = dispositions
        .iter()
        .filter(|(_, disposition)| *disposition == CandidateDisposition::Unknown)
        .count();
    let scope_mismatch_count = dispositions
        .iter()
        .filter(|(_, disposition)| *disposition == CandidateDisposition::ScopeMismatch)
        .count();
    let admissible = problem
        .acquisitions
        .iter()
        .filter(|item| disposition(problem, item) == CandidateDisposition::Admissible)
        .collect::<Vec<_>>();
    let result = search(problem, &admissible);

    Analysis {
        problem: problem.clone(),
        dispositions,
        blocked_count,
        unknown_count,
        scope_mismatch_count,
        result,
    }
}

fn render_plan(out: &mut String, prefix: &str, plan: &Plan) {
    writeln!(
        out,
        "{prefix}.acquisition_ids={}",
        plan.acquisition_ids.join(",")
    )
    .expect("String write");
    writeln!(out, "{prefix}.cost={}", plan.cost).expect("String write");
    writeln!(
        out,
        "{prefix}.covered_obligations={}",
        plan.covered_obligations.join(",")
    )
    .expect("String write");
    writeln!(
        out,
        "{prefix}.uncovered_obligations={}",
        plan.uncovered_obligations.join(",")
    )
    .expect("String write");
}

impl Analysis {
    pub fn render(&self) -> String {
        let mut out = String::new();
        writeln!(out, "schema=fsre.assurance-acquisition-e0").expect("String write");
        writeln!(out, "version=0.1.0").expect("String write");
        writeln!(out, "episode_id={}", self.problem.episode_id).expect("String write");
        writeln!(out, "scope_sha256={}", self.problem.scope_sha256).expect("String write");
        writeln!(out, "budget={}", self.problem.budget).expect("String write");
        writeln!(out, "cost_unit={}", self.problem.cost_unit).expect("String write");
        writeln!(out, "unresolved_obligation_count={}", self.problem.obligations.len())
            .expect("String write");
        writeln!(
            out,
            "unresolved_obligations={}",
            self.problem.obligations.join(",")
        )
        .expect("String write");
        writeln!(
            out,
            "acquisition_candidate_count={}",
            self.problem.acquisitions.len()
        )
        .expect("String write");
        writeln!(
            out,
            "admissible_candidate_count={}",
            self.result.candidate_count
        )
        .expect("String write");
        writeln!(out, "blocked_candidate_count={}", self.blocked_count)
            .expect("String write");
        writeln!(out, "unknown_candidate_count={}", self.unknown_count)
            .expect("String write");
        writeln!(
            out,
            "scope_mismatch_candidate_count={}",
            self.scope_mismatch_count
        )
        .expect("String write");
        writeln!(out, "search_status={}", self.result.status).expect("String write");
        writeln!(
            out,
            "evaluated_subsets={}",
            self.result.evaluated_subsets
        )
        .expect("String write");
        writeln!(
            out,
            "complete_plan_count={}",
            self.result.complete_plan_count
        )
        .expect("String write");
        match self.result.minimum_cost {
            Some(value) => writeln!(out, "minimum_cost={value}").expect("String write"),
            None => out.push_str("minimum_cost=NOT_ESTABLISHED\n"),
        }
        writeln!(
            out,
            "minimum_cost_tie_count={}",
            self.result.minimum_cost_ties.len()
        )
        .expect("String write");
        for (index, plan) in self.result.minimum_cost_ties.iter().enumerate() {
            render_plan(&mut out, &format!("minimum_cost_tie.{index:04}"), plan);
        }
        match self.result.minimum_count {
            Some(value) => writeln!(out, "minimum_acquisition_count={value}").expect("String write"),
            None => out.push_str("minimum_acquisition_count=NOT_ESTABLISHED\n"),
        }
        writeln!(
            out,
            "minimum_count_tie_count={}",
            self.result.minimum_count_ties.len()
        )
        .expect("String write");
        for (index, plan) in self.result.minimum_count_ties.iter().enumerate() {
            render_plan(&mut out, &format!("minimum_count_tie.{index:04}"), plan);
        }
        if let Some(plan) = &self.result.best_partial {
            render_plan(&mut out, "best_partial", plan);
        } else {
            out.push_str("best_partial=NOT_ESTABLISHED\n");
        }
        for (id, disposition) in &self.dispositions {
            writeln!(out, "candidate.{id}.disposition={}", disposition.as_str())
                .expect("String write");
        }
        out.push_str("plan_coverage=DECLARED_CAPABILITY_ONLY\n");
        out.push_str("plan_result_success=NOT_GUARANTEED\n");
        out.push_str("plan_evidence_qualification=REQUIRES_INDEPENDENT_REVIEW\n");
        out.push_str("shared_source_independence=NOT_ESTABLISHED_BY_E0_PLANNER\n");
        out.push_str("stale_asof_evidence=NOT_VALIDATED_BY_E0_PLANNER\n");
        out.push_str("plan_execution=NOT_PERFORMED\n");
        out.push_str("resource_reservation=NOT_PERFORMED\n");
        out.push_str("field_authorization=NOT_GRANTED\n");
        out.push_str("decision_authorization=NOT_GRANTED\n");
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fmt::Write as _;

    const FIXTURE: &str = include_str!("../fixtures/water-obligations.psv");

    fn fixture() -> Problem {
        parse_input(FIXTURE).expect("fixture parses")
    }

    #[test]
    fn exact_fixture_finds_two_minimum_cost_ties_and_one_minimum_count_plan() {
        let report = analyze(&fixture());
        assert_eq!(report.result.status, "EXACT_COMPLETE_COVER_FOUND");
        assert_eq!(report.result.evaluated_subsets, 32);
        assert_eq!(report.result.complete_plan_count, 3);
        assert_eq!(report.result.minimum_cost, Some(9));
        assert_eq!(report.result.minimum_cost_ties.len(), 2);
        assert_eq!(
            report.result.minimum_cost_ties[0].acquisition_ids,
            vec!["authority-review", "combined-survey"]
        );
        assert_eq!(
            report.result.minimum_cost_ties[1].acquisition_ids,
            vec!["joint-audit"]
        );
        assert_eq!(report.result.minimum_count, Some(1));
        assert_eq!(report.result.minimum_count_ties.len(), 1);
        assert_eq!(
            report.result.minimum_count_ties[0].acquisition_ids,
            vec!["joint-audit"]
        );
    }

    #[test]
    fn independent_enumerator_agrees_on_minimum_cost_and_count() {
        let items = [
            ("authority-review", 2_u128, 0b001_u8),
            ("combined-survey", 7_u128, 0b110_u8),
            ("frame-survey", 4_u128, 0b010_u8),
            ("joint-audit", 9_u128, 0b111_u8),
            ("pump-test", 5_u128, 0b100_u8),
        ];
        let mut full = Vec::<(u128, usize, Vec<&str>)>::new();
        for mask in 0_u8..32 {
            let chosen = items
                .iter()
                .enumerate()
                .filter(|(index, _)| mask & (1_u8 << index) != 0)
                .collect::<Vec<_>>();
            let coverage = chosen.iter().fold(0_u8, |value, (_, item)| value | item.2);
            if coverage == 0b111 {
                let cost = chosen.iter().map(|(_, item)| item.1).sum::<u128>();
                let ids = chosen.iter().map(|(_, item)| item.0).collect::<Vec<_>>();
                full.push((cost, chosen.len(), ids));
            }
        }
        let expected_cost = full.iter().map(|item| item.0).min().unwrap();
        let expected_count = full.iter().map(|item| item.1).min().unwrap();
        assert_eq!(expected_cost, 9);
        assert_eq!(expected_count, 1);

        let report = analyze(&fixture());
        assert_eq!(report.result.minimum_cost, Some(expected_cost));
        assert_eq!(report.result.minimum_count, Some(expected_count));
    }

    #[test]
    fn blocked_unknown_and_wrong_scope_acquisitions_never_enter_search() {
        let report = analyze(&fixture());
        assert_eq!(report.result.candidate_count, 5);
        assert_eq!(report.blocked_count, 1);
        assert_eq!(report.unknown_count, 1);
        assert_eq!(report.scope_mismatch_count, 0);
        assert!(report.render().contains("field_authorization=NOT_GRANTED"));
    }

    #[test]
    fn prerequisite_must_be_selected_and_resource_slots_are_exclusive() {
        let mut problem = fixture();
        let authority = problem
            .acquisitions
            .iter_mut()
            .find(|item| item.id == "authority-review")
            .unwrap();
        authority.rights = Gate::Blocked;
        let report = analyze(&problem);
        assert!(!report.result.minimum_cost_ties.iter().any(|plan| {
            plan.acquisition_ids.contains(&"pump-test".to_owned())
                || plan.acquisition_ids.contains(&"combined-survey".to_owned())
        }));

        let mut conflict = fixture();
        let auth = conflict
            .acquisitions
            .iter_mut()
            .find(|item| item.id == "authority-review")
            .unwrap();
        auth.resource_slot = "crew-x".to_owned();
        conflict
            .acquisitions
            .iter_mut()
            .find(|item| item.id == "pump-test")
            .unwrap()
            .resource_slot = "crew-x".to_owned();
        let conflict_report = analyze(&conflict);
        assert!(!conflict_report.result.minimum_cost_ties.iter().any(|plan| {
            plan.acquisition_ids.contains(&"pump-test".to_owned())
                && plan.acquisition_ids.contains(&"authority-review".to_owned())
        }));
    }

    #[test]
    fn incomplete_budget_returns_best_partial_coverage_not_false_full_plan() {
        let mut problem = fixture();
        problem.budget = 8;
        let report = analyze(&problem);
        assert_eq!(report.result.status, "PARTIAL_COVERAGE_ONLY");
        assert!(report.result.minimum_cost.is_none());
        assert!(report.result.best_partial.is_some());
        assert!(!report.result.best_partial.unwrap().uncovered_obligations.is_empty());
    }

    #[test]
    fn search_cap_returns_no_optimality_claim() {
        let mut problem = fixture();
        problem.max_subsets = 8;
        let report = analyze(&problem);
        assert_eq!(
            report.result.status,
            "SEARCH_LIMIT_EXCEEDED_NO_OPTIMALITY_CLAIM"
        );
        assert!(report.result.minimum_cost.is_none());
    }

    #[test]
    fn missing_obligation_capability_is_reported_not_invented() {
        let mut problem = fixture();
        problem.acquisitions.retain(|item| item.id != "joint-audit");
        problem.acquisitions.retain(|item| item.id != "combined-survey");
        problem.acquisitions.retain(|item| item.id != "pump-test");
        let report = analyze(&problem);
        assert_eq!(report.result.status, "PARTIAL_COVERAGE_ONLY");
        assert!(report
            .result
            .best_partial
            .as_ref()
            .unwrap()
            .uncovered_obligations
            .contains(&"HYDRAULIC_SAFETY".to_owned()));
    }

    #[test]
    fn deterministic_under_acquisition_row_permutation() {
        let original = analyze(&fixture()).render();
        let lines = FIXTURE.lines().collect::<Vec<_>>();
        let header = lines.iter().position(|line| *line == HEADER).unwrap();
        let metadata = &lines[..lines.iter().position(|line| *line == "ACQUISITIONS").unwrap()];
        let mut rows = lines[header + 1..].to_vec();
        rows.reverse();
        let mut input = String::new();
        for line in metadata {
            writeln!(input, "{line}").unwrap();
        }
        writeln!(input, "ACQUISITIONS").unwrap();
        writeln!(input, "{HEADER}").unwrap();
        for row in rows {
            writeln!(input, "{row}").unwrap();
        }
        assert_eq!(original, analyze(&parse_input(&input).unwrap()).render());
    }

    #[test]
    fn conflicting_scope_and_dangling_prerequisite_are_rejected_or_excluded() {
        let mut problem = fixture();
        problem.acquisitions[0].scope_sha256 = "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd".to_owned();
        let report = analyze(&problem);
        assert_eq!(report.scope_mismatch_count, 1);

        let broken = FIXTURE.replace(
            "combined-survey|field-survey|7|cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc|PASS|PASS|PASS|PASS|PASS|PASS|FRAME_COVERAGE,HYDRAULIC_SAFETY|authority-review|crew-b",
            "combined-survey|field-survey|7|cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc|PASS|PASS|PASS|PASS|PASS|PASS|FRAME_COVERAGE,HYDRAULIC_SAFETY|missing-prerequisite|crew-b",
        );
        assert!(parse_input(&broken).is_err());
    }
}
