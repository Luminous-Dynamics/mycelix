#![forbid(unsafe_code)]

// Structural dependency audit for service-population source frames.
//
// This tool detects declared shared collection processes and declared
// upstream-source ancestry. Absence of a detected dependency is NOT proof of
// statistical independence. It does not estimate coverage or run capture-
// recapture / multiple-systems estimators.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::fmt::Write as _;

const HEADER: &str = "record_type|frame_id|upstream_frame_id|frame_role|owner_id|collection_process_id|snapshot_id|valid_from_seconds|valid_until_seconds|evidence_owner|evidence_ref|assessment";
const MAX_ROWS: usize = 500_000;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FrameAuditError(pub String);

impl std::fmt::Display for FrameAuditError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for FrameAuditError {}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FrameRole {
    Capture,
    Upstream,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum IndependenceAssessment {
    SeparateDesignDeclared,
    DependenceDeclared,
    Unknown,
}
impl IndependenceAssessment {
    fn parse(value: &str) -> Result<Self, FrameAuditError> {
        match value {
            "SEPARATE_DESIGN_DECLARED" => Ok(Self::SeparateDesignDeclared),
            "DEPENDENCE_DECLARED" => Ok(Self::DependenceDeclared),
            "UNKNOWN" => Ok(Self::Unknown),
            other => Err(FrameAuditError(format!("invalid independence assessment {other}"))),
        }
    }
}

#[derive(Clone, Debug)]
struct Frame {
    id: String,
    role: FrameRole,
    owner_id: String,
    collection_process_id: String,
    snapshot_id: String,
    assessment: IndependenceAssessment,
}

#[derive(Clone, Debug)]
struct Metadata {
    audit_id: String,
    target_scope_id: String,
    study_start: u64,
    study_end: u64,
    source_use_status: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FramePairResult {
    pub frame_a: String,
    pub frame_b: String,
    pub status: String,
    pub same_owner: bool,
    pub same_collection_process: bool,
    pub same_source_snapshot: bool,
    pub shared_ancestors: Vec<String>,
    pub ancestor_relationship: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FrameAuditReport {
    pub audit_id: String,
    pub target_scope_id: String,
    pub study_start: u64,
    pub study_end: u64,
    pub frame_count: usize,
    pub capture_frame_count: usize,
    pub dependency_edge_count: usize,
    pub pair_count: usize,
    pub pairs: Vec<FramePairResult>,
}

impl FrameAuditReport {
    pub fn render(&self) -> String {
        let mut out = String::new();
        writeln!(&mut out, "audit_id={}", self.audit_id).expect("String write");
        writeln!(&mut out, "target_scope_id={}", self.target_scope_id).expect("String write");
        writeln!(&mut out, "study_start={}", self.study_start).expect("String write");
        writeln!(&mut out, "study_end={}", self.study_end).expect("String write");
        writeln!(&mut out, "frame_count={}", self.frame_count).expect("String write");
        writeln!(&mut out, "capture_frame_count={}", self.capture_frame_count).expect("String write");
        writeln!(&mut out, "dependency_edge_count={}", self.dependency_edge_count).expect("String write");
        writeln!(&mut out, "pair_count={}", self.pair_count).expect("String write");
        for pair in &self.pairs {
            let prefix = format!("pair.{}.{}", pair.frame_a, pair.frame_b);
            writeln!(&mut out, "{prefix}.status={}", pair.status).expect("String write");
            writeln!(&mut out, "{prefix}.same_owner={}", pair.same_owner).expect("String write");
            writeln!(&mut out, "{prefix}.same_collection_process={}", pair.same_collection_process).expect("String write");
            writeln!(&mut out, "{prefix}.same_source_snapshot={}", pair.same_source_snapshot).expect("String write");
            writeln!(&mut out, "{prefix}.shared_ancestors={}", pair.shared_ancestors.join(",")).expect("String write");
            writeln!(&mut out, "{prefix}.ancestor_relationship={}", pair.ancestor_relationship).expect("String write");
        }
        out.push_str("independence_conclusion=NOT_ESTABLISHED_BY_STRUCTURAL_AUDIT\n");
        out.push_str("dual_system_estimator_admissibility=NOT_ESTABLISHED\n");
        out.push_str("multiple_systems_estimator_admissibility=NOT_ESTABLISHED\n");
        out.push_str("source_authentication=ASSERTED_NOT_AUTHENTICATED\n");
        out
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Qualification {
    pub audit_id: String,
    pub target_scope_id: String,
    pub status: String,
}

impl Qualification {
    pub fn render(&self) -> String {
        format!(
            concat!(
                "audit_id={}\n",
                "target_scope_id={}\n",
                "audit_status={}\n",
                "frame_count=NOT_COMPUTED\n",
                "pair_count=NOT_COMPUTED\n",
                "independence=NOT_ESTABLISHED\n",
                "estimators=NOT_ADMISSIBLE\n"
            ),
            self.audit_id, self.target_scope_id, self.status
        )
    }
}

fn valid_token(value: &str) -> bool {
    !value.is_empty()
        && value != "-"
        && value.len() <= 200
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric()
                || matches!(byte, b'_' | b'-' | b'.' | b':' | b'/')
        })
}

fn parse_u64(value: &str, field: &str) -> Result<u64, FrameAuditError> {
    value.parse::<u64>().map_err(|error| FrameAuditError(format!("invalid {field}: {error}")))
}

fn parse_prefix(source: &str) -> Result<(BTreeMap<String, String>, Vec<&str>), FrameAuditError> {
    let allowed = [
        "audit_id",
        "target_scope_id",
        "study_start_seconds",
        "study_end_seconds",
        "source_use_status",
    ];
    let mut metadata = BTreeMap::new();
    let mut rows = Vec::new();
    let mut header_seen = false;
    for (offset, raw) in source.lines().enumerate() {
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if !header_seen {
            if let Some(comment) = line.strip_prefix('#') {
                let Some((key, value)) = comment.trim().split_once('=') else {
                    continue;
                };
                let key = key.trim().to_owned();
                let value = value.trim().to_owned();
                if key.is_empty() || value.is_empty() {
                    return Err(FrameAuditError("empty metadata key or value".to_owned()));
                }
                if !allowed.contains(&key.as_str()) {
                    return Err(FrameAuditError(format!("unknown metadata key {key}")));
                }
                if metadata.insert(key.clone(), value).is_some() {
                    return Err(FrameAuditError(format!("line {}: duplicate metadata key {key}", offset + 1)));
                }
                continue;
            }
            if line != HEADER {
                return Err(FrameAuditError(format!("line {}: unexpected table header", offset + 1)));
            }
            header_seen = true;
        } else {
            rows.push(line);
            if rows.len() > MAX_ROWS {
                return Err(FrameAuditError(format!("input exceeds {MAX_ROWS} rows")));
            }
        }
    }
    if !header_seen {
        return Err(FrameAuditError("missing table header".to_owned()));
    }
    Ok((metadata, rows))
}

fn required<'a>(metadata: &'a BTreeMap<String, String>, key: &str) -> Result<&'a str, FrameAuditError> {
    let value = metadata.get(key).ok_or_else(|| FrameAuditError(format!("missing metadata key {key}")))?;
    if value.is_empty() {
        return Err(FrameAuditError(format!("empty metadata key {key}")));
    }
    Ok(value)
}

fn parse_metadata(metadata: &BTreeMap<String, String>) -> Result<Metadata, FrameAuditError> {
    let audit_id = required(metadata, "audit_id")?;
    let target_scope_id = required(metadata, "target_scope_id")?;
    if !valid_token(audit_id) || !valid_token(target_scope_id) {
        return Err(FrameAuditError("invalid audit or scope identifier".to_owned()));
    }
    let study_start = parse_u64(required(metadata, "study_start_seconds")?, "study_start_seconds")?;
    let study_end = parse_u64(required(metadata, "study_end_seconds")?, "study_end_seconds")?;
    if study_end <= study_start {
        return Err(FrameAuditError("study horizon must be a non-empty half-open interval".to_owned()));
    }
    let source_use_status = required(metadata, "source_use_status")?.to_owned();
    if !matches!(source_use_status.as_str(), "AUTHORIZED" | "BLOCKED" | "UNKNOWN") {
        return Err(FrameAuditError("invalid source_use_status".to_owned()));
    }
    Ok(Metadata {
        audit_id: audit_id.to_owned(),
        target_scope_id: target_scope_id.to_owned(),
        study_start,
        study_end,
        source_use_status,
    })
}

fn interval_covers(from: &str, until: &str, metadata: &Metadata, label: &str) -> Result<(), FrameAuditError> {
    let from = parse_u64(from, &format!("{label}_valid_from_seconds"))?;
    let until = parse_u64(until, &format!("{label}_valid_until_seconds"))?;
    if until <= from || from > metadata.study_start || until < metadata.study_end {
        return Err(FrameAuditError(format!("{label} validity does not cover the complete study horizon")));
    }
    Ok(())
}

fn ensure_acyclic(frames: &BTreeMap<String, Frame>, parents: &BTreeMap<String, BTreeSet<String>>) -> Result<(), FrameAuditError> {
    let mut indegree: BTreeMap<String, usize> = frames.keys().map(|id| (id.clone(), 0)).collect();
    for upstreams in parents.values() {
        for upstream in upstreams {
            *indegree.get_mut(upstream).expect("dependency endpoints validated") += 1;
        }
    }
    let mut queue: VecDeque<String> = indegree.iter()
        .filter(|(_, degree)| **degree == 0)
        .map(|(id, _)| id.clone())
        .collect();
    let mut visited = 0usize;
    while let Some(id) = queue.pop_front() {
        visited += 1;
        if let Some(upstreams) = parents.get(&id) {
            for upstream in upstreams {
                let degree = indegree.get_mut(upstream).expect("dependency endpoints validated");
                *degree -= 1;
                if *degree == 0 {
                    queue.push_back(upstream.clone());
                }
            }
        }
    }
    if visited != frames.len() {
        return Err(FrameAuditError("source dependency graph contains a cycle".to_owned()));
    }
    Ok(())
}

fn ancestors(start: &str, parents: &BTreeMap<String, BTreeSet<String>>) -> BTreeSet<String> {
    let mut found = BTreeSet::new();
    let mut pending = vec![start.to_owned()];
    while let Some(current) = pending.pop() {
        if let Some(upstreams) = parents.get(&current) {
            for upstream in upstreams {
                if found.insert(upstream.clone()) {
                    pending.push(upstream.clone());
                }
            }
        }
    }
    found
}

/// Audits declared source-frame provenance and reports pairwise dependency signals.
/// No outcome data are evaluated and no statistical independence is inferred.
pub fn audit(source: &str) -> Result<Result<FrameAuditReport, Qualification>, FrameAuditError> {
    let (raw_metadata, rows) = parse_prefix(source)?;
    let metadata = parse_metadata(&raw_metadata)?;
    if metadata.source_use_status != "AUTHORIZED" {
        return Ok(Err(Qualification {
            audit_id: metadata.audit_id,
            target_scope_id: metadata.target_scope_id,
            status: "BLOCKED_SOURCE_USE".to_owned(),
        }));
    }

    let mut frames = BTreeMap::<String, Frame>::new();
    let mut parents = BTreeMap::<String, BTreeSet<String>>::new();
    let mut dependency_edges = 0usize;

    for (offset, line) in rows.iter().enumerate() {
        let fields: Vec<&str> = line.split('|').map(str::trim).collect();
        if fields.len() != 12 {
            return Err(FrameAuditError(format!("data row {}: expected exactly 12 fields", offset + 1)));
        }
        let frame_id = fields[1];
        if !valid_token(frame_id) {
            return Err(FrameAuditError(format!("data row {}: invalid frame ID", offset + 1)));
        }
        interval_covers(fields[7], fields[8], &metadata, "row")?;
        match fields[0] {
            "FRAME" => {
                if fields[2] != "-" {
                    return Err(FrameAuditError(format!("data row {}: FRAME row must not name an upstream frame", offset + 1)));
                }
                let role = match fields[3] {
                    "CAPTURE" => FrameRole::Capture,
                    "UPSTREAM" => FrameRole::Upstream,
                    other => return Err(FrameAuditError(format!("invalid frame role {other}"))),
                };
                for (value, label) in [
                    (fields[4], "owner_id"),
                    (fields[5], "collection_process_id"),
                    (fields[6], "snapshot_id"),
                    (fields[9], "evidence_owner"),
                    (fields[10], "evidence_ref"),
                ] {
                    if !valid_token(value) {
                        return Err(FrameAuditError(format!("invalid {label}")));
                    }
                }
                let assessment = IndependenceAssessment::parse(fields[11])?;
                if role == FrameRole::Upstream && assessment != IndependenceAssessment::Unknown {
                    return Err(FrameAuditError("UPSTREAM frame assessment must be UNKNOWN".to_owned()));
                }
                let frame = Frame {
                    id: frame_id.to_owned(),
                    role,
                    owner_id: fields[4].to_owned(),
                    collection_process_id: fields[5].to_owned(),
                    snapshot_id: fields[6].to_owned(),
                    assessment,
                };
                if frames.insert(frame_id.to_owned(), frame).is_some() {
                    return Err(FrameAuditError(format!("duplicate frame {frame_id}")));
                }
            }
            "DEPENDENCY" => {
                if fields[3..7].iter().any(|value| *value != "-") || fields[11] != "-" {
                    return Err(FrameAuditError("DEPENDENCY row contains non-evidence frame fields".to_owned()));
                }
                let upstream = fields[2];
                if !valid_token(upstream) || upstream == frame_id {
                    return Err(FrameAuditError("invalid or self-referential upstream frame".to_owned()));
                }
                if !valid_token(fields[9]) || !valid_token(fields[10]) {
                    return Err(FrameAuditError("dependency edge requires evidence owner and reference".to_owned()));
                }
                let edge_set = parents.entry(frame_id.to_owned()).or_default();
                if !edge_set.insert(upstream.to_owned()) {
                    return Err(FrameAuditError(format!("duplicate dependency edge {frame_id}->{upstream}")));
                }
                dependency_edges += 1;
            }
            other => return Err(FrameAuditError(format!("unknown record type {other}"))),
        }
    }

    if frames.is_empty() {
        return Err(FrameAuditError("at least one FRAME row is required".to_owned()));
    }
    for (child, upstreams) in &parents {
        if !frames.contains_key(child) {
            return Err(FrameAuditError(format!("dependency child frame {child} is missing")));
        }
        for upstream in upstreams {
            if !frames.contains_key(upstream) {
                return Err(FrameAuditError(format!("upstream frame {upstream} is missing")));
            }
        }
    }
    ensure_acyclic(&frames, &parents)?;

    let capture_ids: Vec<String> = frames.values()
        .filter(|frame| frame.role == FrameRole::Capture)
        .map(|frame| frame.id.clone())
        .collect();
    let closure: BTreeMap<String, BTreeSet<String>> = capture_ids.iter()
        .map(|id| (id.clone(), ancestors(id, &parents)))
        .collect();
    let mut pairs = Vec::new();
    for left_index in 0..capture_ids.len() {
        for right_index in (left_index + 1)..capture_ids.len() {
            let left = frames.get(&capture_ids[left_index]).expect("capture frame exists");
            let right = frames.get(&capture_ids[right_index]).expect("capture frame exists");
            let left_ancestors = closure.get(&left.id).expect("closure exists");
            let right_ancestors = closure.get(&right.id).expect("closure exists");
            let shared_ancestors: Vec<String> = left_ancestors.intersection(right_ancestors).cloned().collect();
            let ancestor_relationship = left_ancestors.contains(&right.id) || right_ancestors.contains(&left.id);
            let same_owner = left.owner_id == right.owner_id;
            let same_collection_process = left.collection_process_id == right.collection_process_id;
            let same_source_snapshot = left.snapshot_id == right.snapshot_id;

            let status = if same_collection_process {
                "DECLARED_SHARED_COLLECTION_PROCESS"
            } else if same_source_snapshot {
                "DECLARED_SHARED_SOURCE_SNAPSHOT"
            } else if ancestor_relationship {
                "DECLARED_DIRECT_OR_TRANSITIVE_ANCESTRY"
            } else if !shared_ancestors.is_empty() {
                "DECLARED_SHARED_UPSTREAM_SOURCE"
            } else if left.assessment == IndependenceAssessment::DependenceDeclared
                || right.assessment == IndependenceAssessment::DependenceDeclared {
                "DEPENDENCE_DECLARED_BY_FRAME_ASSESSMENT"
            } else if same_owner {
                "POTENTIAL_COMMON_OWNER_REQUIRES_REVIEW"
            } else if left.assessment == IndependenceAssessment::SeparateDesignDeclared
                && right.assessment == IndependenceAssessment::SeparateDesignDeclared {
                "NO_DECLARED_DEPENDENCE_INDEPENDENCE_UNAUTHENTICATED"
            } else {
                "INDEPENDENCE_NOT_ESTABLISHED"
            };
            pairs.push(FramePairResult {
                frame_a: left.id.clone(),
                frame_b: right.id.clone(),
                status: status.to_owned(),
                same_owner,
                same_collection_process,
                same_source_snapshot,
                shared_ancestors,
                ancestor_relationship,
            });
        }
    }

    Ok(Ok(FrameAuditReport {
        audit_id: metadata.audit_id,
        target_scope_id: metadata.target_scope_id,
        study_start: metadata.study_start,
        study_end: metadata.study_end,
        frame_count: frames.len(),
        capture_frame_count: capture_ids.len(),
        dependency_edge_count: dependency_edges,
        pair_count: pairs.len(),
        pairs,
    }))
}

#[cfg(test)]
mod unit_tests {
    use super::*;

    #[test]
    fn tokens_reject_reserved_sentinel() {
        assert!(!valid_token("-"));
        assert!(valid_token("frame-001"));
    }

    #[test]
    fn interval_requires_full_half_open_horizon() {
        let metadata = Metadata {
            audit_id: "a".to_owned(),
            target_scope_id: "s".to_owned(),
            study_start: 10,
            study_end: 20,
            source_use_status: "AUTHORIZED".to_owned(),
        };
        assert!(interval_covers("0", "20", &metadata, "test").is_ok());
        assert!(interval_covers("11", "20", &metadata, "test").is_err());
    }
}
