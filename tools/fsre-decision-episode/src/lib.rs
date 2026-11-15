#![forbid(unsafe_code)]

// E0 composition checker for typed FSRE decision claims.
// It checks scope references and graph structure; it does not authenticate
// sources, rehash artifact bytes, grant authority, or authorize actuation.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::fmt::Write as _;

const CLAIM_HEADER: &str =
    "claim_id|claim_type|qualification_state|scope_id|scope_sha256|artifact_id|artifact_sha256|issuer|source_ref";
const EDGE_HEADER: &str = "claim_id|depends_on_claim_id";
const MAX_INPUT_BYTES: usize = 4 * 1024 * 1024;
const MAX_CLAIMS: usize = 256;
const MAX_EDGES: usize = 4096;

const REQUIRED_CLAIMS: &[&str] = &[
    "FRAME_SCOPE_DECLARED",
    "TEMPORAL_IDENTITY_REPLAY_QUALIFIED",
    "SOURCE_DEPENDENCY_AUDIT_QUALIFIED",
    "PARTIAL_IDENTIFICATION_BOUNDS_VERIFIED",
    "WORLD_SET_ADEQUACY_QUALIFIED_WITHIN_SCOPE",
    "ROBUST_SAFETY_WITHIN_DECLARED_SET",
    "DEFEATER_SEARCH_COMPLETED_WITHIN_DECLARED_UNIVERSE",
    "SAFE_EXPERIMENT_SCHEDULE_QUALIFIED_WITHIN_DECLARED_DESIGN",
    "RIGHTS_SOURCE_USE_QUALIFIED",
    "COMPETENT_AUTHORITY_QUALIFIED",
    "INDEPENDENT_RECEIPT_VERIFIED",
];

const OPTIONAL_CLAIMS: &[&str] = &["REALIZED_SERVICE_OUTCOME_OBSERVED"];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QualificationState {
    QualifiedWithinScope,
    AssertedNotAuthenticated,
    Blocked,
    Unknown,
    NotApplicable,
}

impl QualificationState {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::QualifiedWithinScope => "QUALIFIED_WITHIN_SCOPE",
            Self::AssertedNotAuthenticated => "ASSERTED_NOT_AUTHENTICATED",
            Self::Blocked => "BLOCKED",
            Self::Unknown => "UNKNOWN",
            Self::NotApplicable => "NOT_APPLICABLE",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Scope {
    pub id: String,
    pub scope_sha256: String,
    pub population_frame_id: String,
    pub population_frame_sha256: String,
    pub service_boundary_id: String,
    pub asset_namespace_id: String,
    pub effective_time_cutoff: i64,
    pub recorded_time_cutoff: i64,
    pub source_snapshot_id: String,
    pub source_snapshot_sha256: String,
    pub contract_version: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Claim {
    pub id: String,
    pub claim_type: String,
    pub state: QualificationState,
    pub scope_id: String,
    pub scope_sha256: String,
    pub artifact_id: String,
    pub artifact_sha256: String,
    pub issuer: String,
    pub source_ref: String,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Dependency {
    pub claim_id: String,
    pub depends_on_claim_id: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Problem {
    pub episode_id: String,
    pub target_action: String,
    pub service_floor: i64,
    pub scope: Scope,
    pub claims: Vec<Claim>,
    pub dependencies: Vec<Dependency>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Analysis {
    pub problem: Problem,
    pub terminal_status: String,
    pub graph_status: String,
    pub missing_claim_types: Vec<String>,
    pub unqualified_claim_types: Vec<String>,
    pub blocked_claim_types: Vec<String>,
    pub scope_mismatches: Vec<String>,
    pub satisfied_claim_types: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EpisodeError(pub String);

impl std::fmt::Display for EpisodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for EpisodeError {}

fn fail(message: impl Into<String>) -> EpisodeError {
    EpisodeError(message.into())
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

fn parse_i64(value: &str, name: &str) -> Result<i64, EpisodeError> {
    value
        .parse::<i64>()
        .map_err(|error| fail(format!("invalid {name}: {error}")))
}

fn parse_state(value: &str) -> Result<QualificationState, EpisodeError> {
    match value {
        "QUALIFIED_WITHIN_SCOPE" => Ok(QualificationState::QualifiedWithinScope),
        "ASSERTED_NOT_AUTHENTICATED" => Ok(QualificationState::AssertedNotAuthenticated),
        "BLOCKED" => Ok(QualificationState::Blocked),
        "UNKNOWN" => Ok(QualificationState::Unknown),
        "NOT_APPLICABLE" => Ok(QualificationState::NotApplicable),
        _ => Err(fail(format!("unknown qualification_state {value}"))),
    }
}

fn known_claim_type(value: &str) -> bool {
    REQUIRED_CLAIMS.contains(&value) || OPTIONAL_CLAIMS.contains(&value)
}

pub fn parse_input(source: &str) -> Result<Problem, EpisodeError> {
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
                    | "target_action"
                    | "service_floor"
                    | "scope_id"
                    | "scope_sha256"
                    | "population_frame_id"
                    | "population_frame_sha256"
                    | "service_boundary_id"
                    | "asset_namespace_id"
                    | "effective_time_cutoff"
                    | "recorded_time_cutoff"
                    | "source_snapshot_id"
                    | "source_snapshot_sha256"
                    | "contract_version"
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

    let required = |key: &str| -> Result<&str, EpisodeError> {
        metadata
            .get(key)
            .map(String::as_str)
            .ok_or_else(|| fail(format!("missing metadata key {key}")))
    };

    let episode_id = required("episode_id")?.to_owned();
    let target_action = required("target_action")?.to_owned();
    let service_floor = parse_i64(required("service_floor")?, "service_floor")?;
    let scope_id = required("scope_id")?.to_owned();
    let scope_sha256 = required("scope_sha256")?.to_owned();
    let population_frame_id = required("population_frame_id")?.to_owned();
    let population_frame_sha256 = required("population_frame_sha256")?.to_owned();
    let service_boundary_id = required("service_boundary_id")?.to_owned();
    let asset_namespace_id = required("asset_namespace_id")?.to_owned();
    let effective_time_cutoff =
        parse_i64(required("effective_time_cutoff")?, "effective_time_cutoff")?;
    let recorded_time_cutoff =
        parse_i64(required("recorded_time_cutoff")?, "recorded_time_cutoff")?;
    let source_snapshot_id = required("source_snapshot_id")?.to_owned();
    let source_snapshot_sha256 = required("source_snapshot_sha256")?.to_owned();
    let contract_version = required("contract_version")?.to_owned();

    for (value, name) in [
        (&episode_id, "episode_id"),
        (&target_action, "target_action"),
        (&scope_id, "scope_id"),
        (&population_frame_id, "population_frame_id"),
        (&service_boundary_id, "service_boundary_id"),
        (&asset_namespace_id, "asset_namespace_id"),
        (&source_snapshot_id, "source_snapshot_id"),
        (&contract_version, "contract_version"),
    ] {
        if !valid_token(value) {
            return Err(fail(format!("{name} must be a valid token")));
        }
    }
    if !valid_sha256(&scope_sha256)
        || !valid_sha256(&population_frame_sha256)
        || !valid_sha256(&source_snapshot_sha256)
    {
        return Err(fail("scope hashes must be 64 lowercase hexadecimal characters"));
    }
    if effective_time_cutoff > recorded_time_cutoff {
        return Err(fail(
            "effective_time_cutoff cannot be later than recorded_time_cutoff",
        ));
    }

    let mut cursor = 0usize;
    if lines.get(cursor).map(|(_, line)| *line) != Some("CLAIMS") {
        return Err(fail("expected CLAIMS section"));
    }
    cursor += 1;
    if lines.get(cursor).map(|(_, line)| *line) != Some(CLAIM_HEADER) {
        return Err(fail("expected exact claim header"));
    }
    cursor += 1;

    let mut claims = Vec::<Claim>::new();
    let mut claim_ids = BTreeSet::<String>::new();
    let mut claim_types = BTreeSet::<String>::new();
    let mut artifact_digests = BTreeMap::<String, String>::new();
    while cursor < lines.len() {
        let (line_no, line) = lines[cursor];
        if line == "DEPENDENCIES" {
            break;
        }
        let fields: Vec<&str> = line.split('|').map(str::trim).collect();
        if fields.len() != 9
            || !valid_token(fields[0])
            || !valid_token(fields[1])
            || !valid_token(fields[3])
            || !valid_sha256(fields[4])
            || !valid_token(fields[5])
            || !valid_sha256(fields[6])
            || !valid_token(fields[7])
            || !valid_token(fields[8])
        {
            return Err(fail(format!("line {line_no}: invalid claim row")));
        }
        if !known_claim_type(fields[1]) {
            return Err(fail(format!("line {line_no}: unknown claim type {}", fields[1])));
        }
        if !claim_ids.insert(fields[0].to_owned()) {
            return Err(fail(format!("line {line_no}: duplicate claim id {}", fields[0])));
        }
        if !claim_types.insert(fields[1].to_owned()) {
            return Err(fail(format!(
                "line {line_no}: duplicate claim type {}",
                fields[1]
            )));
        }
        let artifact_id = fields[5].to_owned();
        let artifact_digest = fields[6].to_owned();
        if let Some(existing) = artifact_digests.get(&artifact_id) {
            if existing != &artifact_digest {
                return Err(fail(format!(
                    "line {line_no}: artifact id {artifact_id} is bound to conflicting digests"
                )));
            }
        } else {
            artifact_digests.insert(artifact_id.clone(), artifact_digest.clone());
        }
        claims.push(Claim {
            id: fields[0].to_owned(),
            claim_type: fields[1].to_owned(),
            state: parse_state(fields[2])?,
            scope_id: fields[3].to_owned(),
            scope_sha256: fields[4].to_owned(),
            artifact_id: fields[5].to_owned(),
            artifact_sha256: fields[6].to_owned(),
            issuer: fields[7].to_owned(),
            source_ref: fields[8].to_owned(),
        });
        if claims.len() > MAX_CLAIMS {
            return Err(fail("claim count exceeds limit"));
        }
        cursor += 1;
    }
    if claims.is_empty() {
        return Err(fail("claim set is empty"));
    }
    claims.sort_by_key(|claim| claim.id.clone());

    if lines.get(cursor).map(|(_, line)| *line) != Some("DEPENDENCIES") {
        return Err(fail("missing DEPENDENCIES section"));
    }
    cursor += 1;
    if lines.get(cursor).map(|(_, line)| *line) != Some(EDGE_HEADER) {
        return Err(fail("expected exact dependency header"));
    }
    cursor += 1;

    let mut dependencies = Vec::<Dependency>::new();
    let mut dependency_set = BTreeSet::<Dependency>::new();
    while cursor < lines.len() {
        let (line_no, line) = lines[cursor];
        let fields: Vec<&str> = line.split('|').map(str::trim).collect();
        if fields.len() != 2 || !valid_token(fields[0]) || !valid_token(fields[1]) {
            return Err(fail(format!("line {line_no}: invalid dependency row")));
        }
        let dependency = Dependency {
            claim_id: fields[0].to_owned(),
            depends_on_claim_id: fields[1].to_owned(),
        };
        if dependency.claim_id == dependency.depends_on_claim_id {
            return Err(fail(format!("line {line_no}: self-dependency")));
        }
        if !dependency_set.insert(dependency.clone()) {
            return Err(fail(format!("line {line_no}: duplicate dependency edge")));
        }
        if !claim_ids.contains(&dependency.claim_id)
            || !claim_ids.contains(&dependency.depends_on_claim_id)
        {
            return Err(fail(format!("line {line_no}: dangling dependency reference")));
        }
        dependencies.push(dependency);
        if dependencies.len() > MAX_EDGES {
            return Err(fail("dependency edge count exceeds limit"));
        }
        cursor += 1;
    }
    dependencies.sort();

    Ok(Problem {
        episode_id,
        target_action,
        service_floor,
        scope: Scope {
            id: scope_id,
            scope_sha256,
            population_frame_id,
            population_frame_sha256,
            service_boundary_id,
            asset_namespace_id,
            effective_time_cutoff,
            recorded_time_cutoff,
            source_snapshot_id,
            source_snapshot_sha256,
            contract_version,
        },
        claims,
        dependencies,
    })
}

fn graph_is_acyclic(problem: &Problem) -> bool {
    let mut indegree = problem
        .claims
        .iter()
        .map(|claim| (claim.id.clone(), 0usize))
        .collect::<BTreeMap<_, _>>();
    let mut dependents = BTreeMap::<String, Vec<String>>::new();

    // Edge claim -> dependency is traversed dependency -> dependent.
    for edge in &problem.dependencies {
        *indegree
            .get_mut(&edge.claim_id)
            .expect("parser checked claim id") += 1;
        dependents
            .entry(edge.depends_on_claim_id.clone())
            .or_default()
            .push(edge.claim_id.clone());
    }

    let mut queue = indegree
        .iter()
        .filter(|(_, count)| **count == 0)
        .map(|(id, _)| id.clone())
        .collect::<VecDeque<_>>();
    let mut visited = 0usize;

    while let Some(id) = queue.pop_front() {
        visited += 1;
        if let Some(children) = dependents.get(&id) {
            for child in children {
                let count = indegree.get_mut(child).expect("claim id exists");
                *count -= 1;
                if *count == 0 {
                    queue.push_back(child.clone());
                }
            }
        }
    }
    visited == problem.claims.len()
}

pub fn analyze(problem: &Problem) -> Analysis {
    let present = problem
        .claims
        .iter()
        .map(|claim| claim.claim_type.as_str())
        .collect::<BTreeSet<_>>();
    let missing_claim_types = REQUIRED_CLAIMS
        .iter()
        .filter(|claim_type| !present.contains(**claim_type))
        .map(|claim_type| (*claim_type).to_owned())
        .collect::<Vec<_>>();

    let mut unqualified_claim_types = Vec::new();
    let mut blocked_claim_types = Vec::new();
    let mut satisfied_claim_types = Vec::new();
    let mut scope_mismatches = Vec::new();

    for claim in &problem.claims {
        if claim.scope_id != problem.scope.id
            || claim.scope_sha256 != problem.scope.scope_sha256
        {
            scope_mismatches.push(claim.claim_type.clone());
        }
        if REQUIRED_CLAIMS.contains(&claim.claim_type.as_str()) {
            match claim.state {
                QualificationState::QualifiedWithinScope => {
                    satisfied_claim_types.push(claim.claim_type.clone());
                }
                QualificationState::Blocked => blocked_claim_types.push(claim.claim_type.clone()),
                _ => unqualified_claim_types.push(claim.claim_type.clone()),
            }
        }
    }

    let graph_acyclic = graph_is_acyclic(problem);
    let graph_status = if graph_acyclic {
        "ACYCLIC"
    } else {
        "CYCLE_DETECTED"
    }
    .to_owned();

    let independent_receipt_good = problem
        .claims
        .iter()
        .find(|claim| claim.claim_type == "INDEPENDENT_RECEIPT_VERIFIED")
        .is_some_and(|claim| {
            claim.state == QualificationState::QualifiedWithinScope
                && claim.scope_id == problem.scope.id
        });

    let terminal_status = if !scope_mismatches.is_empty() || !graph_acyclic {
        "INPUT_OR_SCOPE_INCONSISTENCY"
    } else if !blocked_claim_types.is_empty() {
        "BLOCKED_HARD_CONSTRAINT"
    } else if !independent_receipt_good {
        "SEARCH_OR_VERIFICATION_INCOMPLETE"
    } else if !missing_claim_types.is_empty() || !unqualified_claim_types.is_empty() {
        "ABSTAIN_MISSING_OR_UNQUALIFIED_OBLIGATIONS"
    } else {
        "DECISION_ASSURANCE_COMPLETE_WITHIN_SCOPE"
    }
    .to_owned();

    Analysis {
        problem: problem.clone(),
        terminal_status,
        graph_status,
        missing_claim_types,
        unqualified_claim_types,
        blocked_claim_types,
        scope_mismatches,
        satisfied_claim_types,
    }
}

impl Analysis {
    pub fn render(&self) -> String {
        let mut out = String::new();
        writeln!(out, "schema=fsre.decision-episode-e0").expect("String write");
        writeln!(out, "version=0.1.0").expect("String write");
        writeln!(out, "episode_id={}", self.problem.episode_id).expect("String write");
        writeln!(out, "target_action={}", self.problem.target_action).expect("String write");
        writeln!(out, "service_floor={}", self.problem.service_floor).expect("String write");
        writeln!(out, "scope_id={}", self.problem.scope.id).expect("String write");
        writeln!(out, "scope_sha256={}", self.problem.scope.scope_sha256)
            .expect("String write");
        writeln!(
            out,
            "population_frame_id={}",
            self.problem.scope.population_frame_id
        )
        .expect("String write");
        writeln!(
            out,
            "population_frame_sha256={}",
            self.problem.scope.population_frame_sha256
        )
        .expect("String write");
        writeln!(
            out,
            "service_boundary_id={}",
            self.problem.scope.service_boundary_id
        )
        .expect("String write");
        writeln!(
            out,
            "asset_namespace_id={}",
            self.problem.scope.asset_namespace_id
        )
        .expect("String write");
        writeln!(
            out,
            "effective_time_cutoff={}",
            self.problem.scope.effective_time_cutoff
        )
        .expect("String write");
        writeln!(
            out,
            "recorded_time_cutoff={}",
            self.problem.scope.recorded_time_cutoff
        )
        .expect("String write");
        writeln!(
            out,
            "source_snapshot_id={}",
            self.problem.scope.source_snapshot_id
        )
        .expect("String write");
        writeln!(
            out,
            "source_snapshot_sha256={}",
            self.problem.scope.source_snapshot_sha256
        )
        .expect("String write");
        writeln!(
            out,
            "contract_version={}",
            self.problem.scope.contract_version
        )
        .expect("String write");
        writeln!(out, "claim_count={}", self.problem.claims.len()).expect("String write");
        writeln!(
            out,
            "dependency_edge_count={}",
            self.problem.dependencies.len()
        )
        .expect("String write");
        writeln!(out, "required_claim_count={}", REQUIRED_CLAIMS.len())
            .expect("String write");
        writeln!(out, "graph_status={}", self.graph_status).expect("String write");
        writeln!(
            out,
            "scope_mismatch_count={}",
            self.scope_mismatches.len()
        )
        .expect("String write");
        writeln!(
            out,
            "scope_mismatches={}",
            self.scope_mismatches.join(",")
        )
        .expect("String write");
        writeln!(
            out,
            "missing_claim_count={}",
            self.missing_claim_types.len()
        )
        .expect("String write");
        writeln!(
            out,
            "missing_claim_types={}",
            self.missing_claim_types.join(",")
        )
        .expect("String write");
        writeln!(
            out,
            "unqualified_claim_count={}",
            self.unqualified_claim_types.len()
        )
        .expect("String write");
        writeln!(
            out,
            "unqualified_claim_types={}",
            self.unqualified_claim_types.join(",")
        )
        .expect("String write");
        writeln!(
            out,
            "blocked_claim_count={}",
            self.blocked_claim_types.len()
        )
        .expect("String write");
        writeln!(
            out,
            "blocked_claim_types={}",
            self.blocked_claim_types.join(",")
        )
        .expect("String write");
        writeln!(
            out,
            "satisfied_required_claim_count={}",
            self.satisfied_claim_types.len()
        )
        .expect("String write");
        writeln!(
            out,
            "satisfied_required_claim_types={}",
            self.satisfied_claim_types.join(",")
        )
        .expect("String write");
        writeln!(out, "terminal_status={}", self.terminal_status).expect("String write");

        for claim in &self.problem.claims {
            let prefix = format!("claim.{}", claim.id);
            writeln!(out, "{prefix}.type={}", claim.claim_type).expect("String write");
            writeln!(out, "{prefix}.state={}", claim.state.as_str()).expect("String write");
            writeln!(out, "{prefix}.scope_id={}", claim.scope_id).expect("String write");
            writeln!(out, "{prefix}.scope_sha256={}", claim.scope_sha256)
                .expect("String write");
            writeln!(out, "{prefix}.artifact_id={}", claim.artifact_id).expect("String write");
            writeln!(
                out,
                "{prefix}.artifact_sha256={}",
                claim.artifact_sha256
            )
            .expect("String write");
            writeln!(out, "{prefix}.issuer={}", claim.issuer).expect("String write");
            writeln!(out, "{prefix}.source_ref={}", claim.source_ref).expect("String write");
        }

        out.push_str("claim_qualification_states=CALLER_SUPPLIED_NOT_AUTHENTICATED\n");
        out.push_str("artifact_digest_check=FORMAT_AND_REFERENCE_CONSISTENCY_ONLY\n");
        out.push_str("artifact_content_rehash=NOT_PERFORMED\n");
        out.push_str("independent_verifier_claim=CALLER_SUPPLIED_NOT_REAUTHENTICATED_BY_COMPOSER\n");
        out.push_str("source_authenticity=NOT_ESTABLISHED_BY_COMPOSER\n");
        out.push_str("rights_authority=CLAIM_STATE_ONLY_NOT_AUTHENTICATED\n");
        out.push_str("physical_realizability=NOT_ESTABLISHED_BY_COMPOSER\n");
        out.push_str("realized_service_outcome=NOT_ESTABLISHED\n");
        out.push_str("decision_authorization=NOT_GRANTED_BY_COMPOSER\n");
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../fixtures/complete-episode.psv");

    fn fixture() -> Problem {
        parse_input(FIXTURE).expect("fixture parses")
    }

    #[test]
    fn complete_claim_graph_qualifies_only_within_declared_scope() {
        let report = analyze(&fixture());
        assert_eq!(
            report.terminal_status,
            "DECISION_ASSURANCE_COMPLETE_WITHIN_SCOPE"
        );
        assert_eq!(report.graph_status, "ACYCLIC");
        assert!(report.missing_claim_types.is_empty());
        assert!(report.unqualified_claim_types.is_empty());
        assert!(report.blocked_claim_types.is_empty());
        assert!(report.scope_mismatches.is_empty());
        assert!(report.render().contains("decision_authorization=NOT_GRANTED_BY_COMPOSER"));
        assert!(report
            .render()
            .contains("claim_qualification_states=CALLER_SUPPLIED_NOT_AUTHENTICATED"));
    }

    #[test]
    fn missing_required_claim_fails_closed() {
        let source = FIXTURE
            .lines()
             .map(|line| {
                if line.starts_with("c-source|") {
                    line.replace(
                        "SOURCE_DEPENDENCY_AUDIT_QUALIFIED",
                        "REALIZED_SERVICE_OUTCOME_OBSERVED",
                    )
                } else {
                    line.to_owned()
                }
            })
            .collect::<Vec<_>>()
            .join("\n");
        let problem = parse_input(&source).unwrap();
        let report = analyze(&problem);
        assert_eq!(
            report.terminal_status,
            "ABSTAIN_MISSING_OR_UNQUALIFIED_OBLIGATIONS"
        );
        assert_eq!(
            report.missing_claim_types,
            vec!["SOURCE_DEPENDENCY_AUDIT_QUALIFIED"]
        );
    }

    #[test]
    fn unqualified_claim_is_not_promoted_by_other_qualified_claims() {
        let source = FIXTURE.replace(
            "c-source|SOURCE_DEPENDENCY_AUDIT_QUALIFIED|QUALIFIED_WITHIN_SCOPE",
            "c-source|SOURCE_DEPENDENCY_AUDIT_QUALIFIED|ASSERTED_NOT_AUTHENTICATED",
        );
        let report = analyze(&parse_input(&source).unwrap());
        assert_eq!(
            report.terminal_status,
            "ABSTAIN_MISSING_OR_UNQUALIFIED_OBLIGATIONS"
        );
        assert_eq!(
            report.unqualified_claim_types,
            vec!["SOURCE_DEPENDENCY_AUDIT_QUALIFIED"]
        );
    }

    #[test]
    fn blocked_claim_blocks_hard_decision_gate() {
        let source = FIXTURE.replace(
            "c-authority|COMPETENT_AUTHORITY_QUALIFIED|QUALIFIED_WITHIN_SCOPE",
            "c-authority|COMPETENT_AUTHORITY_QUALIFIED|BLOCKED",
        );
        let report = analyze(&parse_input(&source).unwrap());
        assert_eq!(report.terminal_status, "BLOCKED_HARD_CONSTRAINT");
    }

    #[test]
    fn scope_mismatch_is_an_input_inconsistency() {
        let source = FIXTURE.replace(
            "c-identity|TEMPORAL_IDENTITY_REPLAY_QUALIFIED|QUALIFIED_WITHIN_SCOPE|decision-scope-v1|cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            "c-identity|TEMPORAL_IDENTITY_REPLAY_QUALIFIED|QUALIFIED_WITHIN_SCOPE|stale-scope-v0|cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
        );
        let report = analyze(&parse_input(&source).unwrap());
        assert_eq!(
            report.terminal_status,
            "INPUT_OR_SCOPE_INCONSISTENCY"
        );
        assert_eq!(
            report.scope_mismatches,
            vec!["TEMPORAL_IDENTITY_REPLAY_QUALIFIED"]
        );
    }

    #[test]
    fn scope_digest_mismatch_is_detected_even_when_scope_id_matches() {
        let source = FIXTURE.replace(
            "c-identity|TEMPORAL_IDENTITY_REPLAY_QUALIFIED|QUALIFIED_WITHIN_SCOPE|decision-scope-v1|cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            "c-identity|TEMPORAL_IDENTITY_REPLAY_QUALIFIED|QUALIFIED_WITHIN_SCOPE|decision-scope-v1|dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd",
        );
        let report = analyze(&parse_input(&source).unwrap());
        assert_eq!(
            report.terminal_status,
            "INPUT_OR_SCOPE_INCONSISTENCY"
        );
        assert_eq!(
            report.scope_mismatches,
            vec!["TEMPORAL_IDENTITY_REPLAY_QUALIFIED"]
        );
    }

    #[test]
    fn artifact_id_cannot_be_reused_for_a_different_digest() {
        let source = FIXTURE.replace(
            "c-identity|TEMPORAL_IDENTITY_REPLAY_QUALIFIED|QUALIFIED_WITHIN_SCOPE|decision-scope-v1|cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc|artifact-identity|2222222222222222222222222222222222222222222222222222222222222222",
            "c-identity|TEMPORAL_IDENTITY_REPLAY_QUALIFIED|QUALIFIED_WITHIN_SCOPE|decision-scope-v1|cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc|artifact-frame|2222222222222222222222222222222222222222222222222222222222222222",
        );
        assert!(parse_input(&source).is_err());
    }

    #[test]
    fn missing_independent_receipt_is_verification_incomplete() {
        let source = FIXTURE.replace(
            "c-verifier|INDEPENDENT_RECEIPT_VERIFIED|QUALIFIED_WITHIN_SCOPE|decision-scope-v1|cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc|artifact-verifier",
            "c-verifier|INDEPENDENT_RECEIPT_VERIFIED|UNKNOWN|decision-scope-v1|cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc|artifact-verifier",
        );
        let report = analyze(&parse_input(&source).unwrap());
        assert_eq!(
            report.terminal_status,
            "SEARCH_OR_VERIFICATION_INCOMPLETE"
        );
    }

    #[test]
    fn dependency_cycle_is_rejected_by_terminal_composer() {
        let mut problem = fixture();
        problem.dependencies.push(Dependency {
            claim_id: "c-frame".to_owned(),
            depends_on_claim_id: "c-verifier".to_owned(),
        });
        let report = analyze(&problem);
        assert_eq!(report.graph_status, "CYCLE_DETECTED");
        assert_eq!(
            report.terminal_status,
            "INPUT_OR_SCOPE_INCONSISTENCY"
        );
    }

    #[test]
    fn self_dependency_dangling_edges_and_duplicates_are_rejected() {
        let self_edge = FIXTURE.replace(
            "c-identity|c-frame",
            "c-identity|c-identity",
        );
        assert!(parse_input(&self_edge).is_err());

        let dangling = FIXTURE.replace(
            "c-identity|c-frame",
            "c-identity|missing-claim",
        );
        assert!(parse_input(&dangling).is_err());

        let duplicate = FIXTURE.replace(
            "c-identity|c-frame",
            "c-identity|c-frame\nc-identity|c-frame",
        );
        assert!(parse_input(&duplicate).is_err());
    }

    #[test]
    fn digest_shape_is_checked_but_content_is_not_claimed_rehashed() {
        let invalid = FIXTURE.replace(
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "not-a-sha256-digest",
        );
        assert!(parse_input(&invalid).is_err());

        let report = analyze(&fixture());
        assert!(report.render().contains("artifact_content_rehash=NOT_PERFORMED"));
    }

    #[test]
    fn unknown_claim_types_and_qualification_states_are_rejected() {
        let unknown_type = FIXTURE.replace(
            "SOURCE_DEPENDENCY_AUDIT_QUALIFIED",
            "MAGIC_DATA_QUALITY",
        );
        assert!(parse_input(&unknown_type).is_err());

        let unknown_state = FIXTURE.replace(
            "QUALIFIED_WITHIN_SCOPE",
            "TRUST_ME",
        );
        assert!(parse_input(&unknown_state).is_err());
    }

    #[test]
    fn permutation_of_claims_and_edges_preserves_rendered_receipt() {
        let original = analyze(&fixture()).render();
        let lines = FIXTURE.lines().collect::<Vec<_>>();
        let claims_section = lines.iter().position(|line| *line == "CLAIMS").unwrap();
        let dependencies_section = lines
            .iter()
            .position(|line| *line == "DEPENDENCIES")
            .unwrap();
        let metadata = &lines[..claims_section];
        let mut claims = lines[claims_section + 2..dependencies_section].to_vec();
        let mut edges = lines[dependencies_section + 2..].to_vec();
        claims.reverse();
        edges.reverse();

        let mut input = String::new();
        for line in metadata {
            writeln!(input, "{line}").unwrap();
        }
        writeln!(input, "CLAIMS").unwrap();
        writeln!(input, "{CLAIM_HEADER}").unwrap();
        for line in claims {
            writeln!(input, "{line}").unwrap();
        }
        writeln!(input, "DEPENDENCIES").unwrap();
        writeln!(input, "{EDGE_HEADER}").unwrap();
        for line in edges {
            writeln!(input, "{line}").unwrap();
        }

        assert_eq!(original, analyze(&parse_input(&input).unwrap()).render());
    }

    #[test]
    fn effective_time_cannot_exceed_recorded_time_cutoff() {
        let invalid = FIXTURE.replace(
            "#effective_time_cutoff=100",
            "#effective_time_cutoff=300",
        );
        assert!(parse_input(&invalid).is_err());
    }
}
