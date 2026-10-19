//! Century-scale evolution primitives.
//!
//! These helpers preserve explicit artifact lineage and only propose format
//! migration paths whose edges are declared verified-lossless and carry a
//! well-formed evidence digest. They do not verify the evidence bytes, prove
//! a migration is actually lossless, or perform the migration itself.

use std::collections::{HashMap, HashSet, VecDeque};

fn nonblank(value: &str) -> bool {
    !value.trim().is_empty()
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

/// A format identity is a stable identifier plus an explicitly named version.
/// The version is intentionally opaque: not every construction method, legal
/// edition, or archival format follows semantic-versioning rules.
#[derive(Clone, Debug, Eq, PartialEq, Hash)]
pub struct FormatVersion {
    pub format_id: String,
    pub version: String,
}

impl FormatVersion {
    pub fn new(format_id: impl Into<String>, version: impl Into<String>) -> Self {
        Self {
            format_id: format_id.into(),
            version: version.into(),
        }
    }

    fn is_well_formed(&self) -> bool {
        nonblank(&self.format_id) && nonblank(&self.version)
    }
}

/// One immutable revision of a design artifact, drawing, model, specification,
/// rule pack, calculation report, or as-built record.
///
/// A revision points to its predecessor rather than replacing it. Storage must
/// enforce append-only writes; this validator can only check the supplied set.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ArtifactRevision {
    pub artifact_id: String,
    pub revision_id: String,
    pub parent_revision_id: Option<String>,
    pub format: FormatVersion,
    /// SHA-256 of the exact stored bytes. This module checks syntax only.
    pub content_sha256: String,
    /// UTC day number since 1970-01-01; the caller supplies trusted time.
    pub recorded_on_day: u32,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RevisionIssue {
    EmptyLedger,
    EmptyArtifactId(String),
    EmptyRevisionId,
    DuplicateRevisionId(String),
    InvalidFormatVersion(String),
    InvalidContentDigest(String),
    UnknownParent {
        revision_id: String,
        parent_id: String,
    },
    ParentArtifactMismatch {
        revision_id: String,
        parent_id: String,
    },
    ParentRecordedAfterChild {
        revision_id: String,
        parent_id: String,
    },
    ParentCycle(String),
}

/// Validate the supplied revision graph without mutating or rewriting records.
pub fn validate_revision_ledger(revisions: &[ArtifactRevision]) -> Vec<RevisionIssue> {
    let mut issues = Vec::new();
    if revisions.is_empty() {
        return vec![RevisionIssue::EmptyLedger];
    }

    let mut by_id: HashMap<&str, &ArtifactRevision> = HashMap::new();
    for revision in revisions {
        if !nonblank(&revision.artifact_id) {
            issues.push(RevisionIssue::EmptyArtifactId(revision.revision_id.clone()));
        }
        if !nonblank(&revision.revision_id) {
            issues.push(RevisionIssue::EmptyRevisionId);
        } else if by_id
            .insert(revision.revision_id.as_str(), revision)
            .is_some()
        {
            issues.push(RevisionIssue::DuplicateRevisionId(
                revision.revision_id.clone(),
            ));
        }
        if !revision.format.is_well_formed() {
            issues.push(RevisionIssue::InvalidFormatVersion(
                revision.revision_id.clone(),
            ));
        }
        if !is_sha256_hex(&revision.content_sha256) {
            issues.push(RevisionIssue::InvalidContentDigest(
                revision.revision_id.clone(),
            ));
        }
    }

    for revision in revisions {
        let Some(parent_id) = revision.parent_revision_id.as_deref() else {
            continue;
        };
        let Some(parent) = by_id.get(parent_id).copied() else {
            issues.push(RevisionIssue::UnknownParent {
                revision_id: revision.revision_id.clone(),
                parent_id: parent_id.to_owned(),
            });
            continue;
        };
        if parent.artifact_id != revision.artifact_id {
            issues.push(RevisionIssue::ParentArtifactMismatch {
                revision_id: revision.revision_id.clone(),
                parent_id: parent_id.to_owned(),
            });
        }
        if parent.recorded_on_day > revision.recorded_on_day {
            issues.push(RevisionIssue::ParentRecordedAfterChild {
                revision_id: revision.revision_id.clone(),
                parent_id: parent_id.to_owned(),
            });
        }
    }

    // Walk each parent chain so same-day edits cannot hide a cycle.
    for revision in revisions {
        let mut visited: HashSet<&str> = HashSet::new();
        let mut cursor: Option<&str> = Some(revision.revision_id.as_str());
        while let Some(revision_id) = cursor {
            if !visited.insert(revision_id) {
                issues.push(RevisionIssue::ParentCycle(revision.revision_id.clone()));
                break;
            }
            cursor = by_id
                .get(revision_id)
                .and_then(|entry| entry.parent_revision_id.as_deref());
        }
    }

    issues
}

/// How much trust to place in a proposed format migration.
///
/// Only VerifiedLossless edges can be traversed by the automatic planner.
/// RequiresHumanReview, Lossy, Experimental, and Retired deliberately require
/// another workflow and are never silently treated as safe.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MigrationDisposition {
    Pending,
    RequiresHumanReview,
    ApprovedForAutomaticUse,
    Lossy,
    Experimental,
    Retired,
}

/// A semantic property that must survive migration. Values are digests of
/// canonicalized property values; the adapter must compute them from actual
/// source/target artifacts before submitting this evidence.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SemanticInvariant {
    pub invariant_id: String,
    pub canonical_value_sha256: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MigrationCandidate {
    pub migration_id: String,
    pub from: FormatVersion,
    pub to: FormatVersion,
    pub tool_version: String,
    pub disposition: MigrationDisposition,
    pub source_artifact_sha256: String,
    pub target_artifact_sha256: String,
    pub verification_evidence_sha256: String,
    /// Critical semantic properties the migration contract promises to retain.
    pub required_invariants: Vec<String>,
    pub source_invariants: Vec<SemanticInvariant>,
    pub target_invariants: Vec<SemanticInvariant>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct MigrationDefinition {
    migration_id: String,
    from: FormatVersion,
    to: FormatVersion,
    tool_version: String,
    source_artifact_sha256: String,
    target_artifact_sha256: String,
    verification_evidence_sha256: String,
    required_invariants: Vec<String>,
}

/// This type can only be constructed inside this module after candidate
/// validation. It is an API-level guard against accidentally passing an
/// unverified candidate into automatic planning, not a cryptographic boundary:
/// production still needs authenticated reviewer authorization and real digest
/// recomputation from artifact bytes.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VerifiedMigration {
    definition: MigrationDefinition,
}

impl VerifiedMigration {
    pub fn migration_id(&self) -> &str {
        &self.definition.migration_id
    }

    pub fn from(&self) -> &FormatVersion {
        &self.definition.from
    }

    pub fn to(&self) -> &FormatVersion {
        &self.definition.to
    }

    pub fn tool_version(&self) -> &str {
        &self.definition.tool_version
    }

    pub fn source_artifact_sha256(&self) -> &str {
        &self.definition.source_artifact_sha256
    }

    pub fn target_artifact_sha256(&self) -> &str {
        &self.definition.target_artifact_sha256
    }

    pub fn verification_evidence_sha256(&self) -> &str {
        &self.definition.verification_evidence_sha256
    }

    pub fn required_invariants(&self) -> &[String] {
        &self.definition.required_invariants
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum MigrationVerificationIssue {
    InvalidMetadata(&'static str),
    InvalidSourceVersion,
    InvalidTargetVersion,
    NotApprovedForAutomaticUse(MigrationDisposition),
    InvalidSourceArtifactDigest,
    InvalidTargetArtifactDigest,
    InvalidVerificationEvidenceDigest,
    NoRequiredInvariants,
    EmptyRequiredInvariant,
    DuplicateRequiredInvariant(String),
    EmptyObservedInvariant {
        side: &'static str,
    },
    DuplicateObservedInvariant {
        side: &'static str,
        invariant_id: String,
    },
    InvalidInvariantDigest {
        side: &'static str,
        invariant_id: String,
    },
    MissingRequiredInvariant {
        side: &'static str,
        invariant_id: String,
    },
    CriticalInvariantChanged(String),
}

/// Validate the explicit migration contract and mint an opaque
/// VerifiedMigration only when every declared critical invariant is present
/// on both sides and its canonical value digest is unchanged.
///
/// Important limit: this compares adapter-supplied observations and verifies
/// digest syntax; it does not hash the files itself or prove the adapter
/// extracted all critical properties. Those claims require a trusted adapter,
/// authenticated review, and independent conformance fixtures.
pub fn verify_migration_candidate(
    candidate: MigrationCandidate,
) -> Result<VerifiedMigration, Vec<MigrationVerificationIssue>> {
    let mut issues = Vec::new();

    if !nonblank(&candidate.migration_id) {
        issues.push(MigrationVerificationIssue::InvalidMetadata("migration_id"));
    }
    if !nonblank(&candidate.tool_version) {
        issues.push(MigrationVerificationIssue::InvalidMetadata("tool_version"));
    }
    if !candidate.from.is_well_formed() {
        issues.push(MigrationVerificationIssue::InvalidSourceVersion);
    }
    if !candidate.to.is_well_formed() {
        issues.push(MigrationVerificationIssue::InvalidTargetVersion);
    }
    if candidate.disposition != MigrationDisposition::ApprovedForAutomaticUse {
        issues.push(MigrationVerificationIssue::NotApprovedForAutomaticUse(
            candidate.disposition,
        ));
    }
    if !is_sha256_hex(&candidate.source_artifact_sha256) {
        issues.push(MigrationVerificationIssue::InvalidSourceArtifactDigest);
    }
    if !is_sha256_hex(&candidate.target_artifact_sha256) {
        issues.push(MigrationVerificationIssue::InvalidTargetArtifactDigest);
    }
    if !is_sha256_hex(&candidate.verification_evidence_sha256) {
        issues.push(MigrationVerificationIssue::InvalidVerificationEvidenceDigest);
    }
    if candidate.required_invariants.is_empty() {
        issues.push(MigrationVerificationIssue::NoRequiredInvariants);
    }

    let mut required = HashSet::new();
    for invariant_id in &candidate.required_invariants {
        if !nonblank(invariant_id) {
            issues.push(MigrationVerificationIssue::EmptyRequiredInvariant);
        } else if !required.insert(invariant_id.as_str()) {
            issues.push(MigrationVerificationIssue::DuplicateRequiredInvariant(
                invariant_id.clone(),
            ));
        }
    }

    let collect_observations = |side: &'static str,
                                observations: &[SemanticInvariant],
                                issues: &mut Vec<MigrationVerificationIssue>|
     -> HashMap<String, String> {
        let mut values: HashMap<String, String> = HashMap::new();
        for observation in observations {
            if !nonblank(&observation.invariant_id) {
                issues.push(MigrationVerificationIssue::EmptyObservedInvariant { side });
                continue;
            }
            if !is_sha256_hex(&observation.canonical_value_sha256) {
                issues.push(MigrationVerificationIssue::InvalidInvariantDigest {
                    side,
                    invariant_id: observation.invariant_id.clone(),
                });
                continue;
            }
            if values
                .insert(
                    observation.invariant_id.clone(),
                    observation.canonical_value_sha256.clone(),
                )
                .is_some()
            {
                issues.push(MigrationVerificationIssue::DuplicateObservedInvariant {
                    side,
                    invariant_id: observation.invariant_id.clone(),
                });
            }
        }
        values
    };

    let source = collect_observations("source", &candidate.source_invariants, &mut issues);
    let target = collect_observations("target", &candidate.target_invariants, &mut issues);

    for invariant_id in &candidate.required_invariants {
        if !nonblank(invariant_id) {
            continue;
        }
        let source_value = source.get(invariant_id.as_str());
        let target_value = target.get(invariant_id.as_str());
        if source_value.is_none() {
            issues.push(MigrationVerificationIssue::MissingRequiredInvariant {
                side: "source",
                invariant_id: invariant_id.clone(),
            });
        }
        if target_value.is_none() {
            issues.push(MigrationVerificationIssue::MissingRequiredInvariant {
                side: "target",
                invariant_id: invariant_id.clone(),
            });
        }
        if let (Some(source_value), Some(target_value)) = (source_value, target_value)
            && source_value != target_value
        {
            issues.push(MigrationVerificationIssue::CriticalInvariantChanged(
                invariant_id.clone(),
            ));
        }
    }

    if !issues.is_empty() {
        return Err(issues);
    }

    Ok(VerifiedMigration {
        definition: MigrationDefinition {
            migration_id: candidate.migration_id,
            from: candidate.from,
            to: candidate.to,
            tool_version: candidate.tool_version,
            source_artifact_sha256: candidate.source_artifact_sha256,
            target_artifact_sha256: candidate.target_artifact_sha256,
            verification_evidence_sha256: candidate.verification_evidence_sha256,
            required_invariants: candidate.required_invariants,
        },
    })
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum MigrationBlocker {
    InvalidSourceVersion,
    InvalidTargetVersion,
    DuplicateVerifiedMigrationId(String),
    NoVerifiedLosslessPath {
        from: FormatVersion,
        to: FormatVersion,
        max_steps: usize,
    },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum MigrationPlanStatus {
    Ready,
    Blocked,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MigrationPlan {
    pub status: MigrationPlanStatus,
    pub steps: Vec<VerifiedMigration>,
    pub blockers: Vec<MigrationBlocker>,
}

/// Find a shortest path through opaque VerifiedMigration values only. Raw
/// MigrationCandidate values cannot be passed to this function.
pub fn plan_lossless_migration(
    from: &FormatVersion,
    to: &FormatVersion,
    migrations: &[VerifiedMigration],
    max_steps: usize,
) -> MigrationPlan {
    let mut blockers = Vec::new();
    if !from.is_well_formed() {
        blockers.push(MigrationBlocker::InvalidSourceVersion);
    }
    if !to.is_well_formed() {
        blockers.push(MigrationBlocker::InvalidTargetVersion);
    }
    if !blockers.is_empty() {
        return MigrationPlan {
            status: MigrationPlanStatus::Blocked,
            steps: Vec::new(),
            blockers,
        };
    }
    if from == to {
        return MigrationPlan {
            status: MigrationPlanStatus::Ready,
            steps: Vec::new(),
            blockers,
        };
    }

    let mut seen_ids = HashSet::new();
    let mut adjacency: HashMap<FormatVersion, Vec<usize>> = HashMap::new();
    for (index, migration) in migrations.iter().enumerate() {
        let definition = &migration.definition;
        if !seen_ids.insert(definition.migration_id.as_str()) {
            blockers.push(MigrationBlocker::DuplicateVerifiedMigrationId(
                definition.migration_id.clone(),
            ));
            continue;
        }
        adjacency
            .entry(definition.from.clone())
            .or_default()
            .push(index);
    }

    for indexes in adjacency.values_mut() {
        indexes.sort_by(|left, right| {
            migrations[*left]
                .migration_id()
                .cmp(migrations[*right].migration_id())
        });
    }

    let mut queue: VecDeque<(FormatVersion, usize)> = VecDeque::new();
    let mut visited: HashSet<FormatVersion> = HashSet::new();
    let mut parent: HashMap<FormatVersion, (FormatVersion, usize)> = HashMap::new();
    queue.push_back((from.clone(), 0));
    visited.insert(from.clone());

    while let Some((current, depth)) = queue.pop_front() {
        if &current == to {
            break;
        }
        if depth >= max_steps {
            continue;
        }
        let Some(indexes) = adjacency.get(&current) else {
            continue;
        };
        for index in indexes {
            let next = migrations[*index].to().clone();
            if visited.insert(next.clone()) {
                parent.insert(next.clone(), (current.clone(), *index));
                queue.push_back((next, depth + 1));
            }
        }
    }

    if !visited.contains(to) {
        blockers.push(MigrationBlocker::NoVerifiedLosslessPath {
            from: from.clone(),
            to: to.clone(),
            max_steps,
        });
        return MigrationPlan {
            status: MigrationPlanStatus::Blocked,
            steps: Vec::new(),
            blockers,
        };
    }

    if !blockers.is_empty() {
        return MigrationPlan {
            status: MigrationPlanStatus::Blocked,
            steps: Vec::new(),
            blockers,
        };
    }

    let mut steps = Vec::new();
    let mut cursor = to.clone();
    while &cursor != from {
        let Some((previous, index)) = parent.get(&cursor).cloned() else {
            return MigrationPlan {
                status: MigrationPlanStatus::Blocked,
                steps: Vec::new(),
                blockers: vec![MigrationBlocker::NoVerifiedLosslessPath {
                    from: from.clone(),
                    to: to.clone(),
                    max_steps,
                }],
            };
        };
        steps.push(migrations[index].clone());
        cursor = previous;
    }
    steps.reverse();

    MigrationPlan {
        status: MigrationPlanStatus::Ready,
        steps,
        blockers,
    }
}

/// A directed dependency: dependent_id was derived from input_id.
///
/// If input_id changes, the dependent output and all transitive dependents
/// become stale until re-evaluated. Identifiers should refer to stable object,
/// rule, solver, dataset, or evidence identities—not display names.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DependencyEdge {
    pub input_id: String,
    pub dependent_id: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DependencyGraphIssue {
    EmptyChangedArtifactId,
    EmptyInputId,
    EmptyDependentId,
    SelfDependency(String),
    DuplicateEdge {
        input_id: String,
        dependent_id: String,
    },
    CycleDetected,
}

/// One artifact that must be re-evaluated, plus its immediate cause in
/// the deterministic breadth-first impact forest.
///
/// A changed root has no `cause_parent_id`. For any dependent artifact, follow
/// the parent links backward and reverse the result to recover its shortest
/// cause chain. Parent pointers keep the report O(V + E) rather than storing
/// every full path, which would take quadratic space for a long dependency chain.
/// This is an invalidation explanation, not proof that a downstream evaluator ran.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AffectedArtifact {
    pub artifact_id: String,
    pub cause_parent_id: Option<String>,
}

/// Return changed nodes and all transitively dependent outputs with the
/// deterministic shortest cause path for each item.
///
/// Malformed edges and cyclic dependency graphs return an error instead of a
/// partial closure that might leave stale safety findings current. Cycles
/// anywhere in the declared graph block the report, even if disconnected from
/// the changed nodes.
///
/// This identifies what must be re-evaluated; it does not run the evaluators or
/// mark real stored findings stale. The caller must persist the invalidation.
pub fn affected_cause_tree(
    changed_artifact_ids: &[String],
    dependencies: &[DependencyEdge],
) -> Result<Vec<AffectedArtifact>, Vec<DependencyGraphIssue>> {
    let mut issues = Vec::new();
    for changed_id in changed_artifact_ids {
        if !nonblank(changed_id) {
            issues.push(DependencyGraphIssue::EmptyChangedArtifactId);
        }
    }

    let mut seen_edges: HashSet<(String, String)> = HashSet::new();
    let mut adjacency: HashMap<String, Vec<String>> = HashMap::new();
    for edge in dependencies {
        if !nonblank(&edge.input_id) {
            issues.push(DependencyGraphIssue::EmptyInputId);
            continue;
        }
        if !nonblank(&edge.dependent_id) {
            issues.push(DependencyGraphIssue::EmptyDependentId);
            continue;
        }
        if edge.input_id == edge.dependent_id {
            issues.push(DependencyGraphIssue::SelfDependency(edge.input_id.clone()));
            continue;
        }
        let pair = (edge.input_id.clone(), edge.dependent_id.clone());
        if !seen_edges.insert(pair) {
            issues.push(DependencyGraphIssue::DuplicateEdge {
                input_id: edge.input_id.clone(),
                dependent_id: edge.dependent_id.clone(),
            });
            continue;
        }
        adjacency
            .entry(edge.input_id.clone())
            .or_default()
            .push(edge.dependent_id.clone());
    }

    if !issues.is_empty() {
        return Err(issues);
    }

    for dependents in adjacency.values_mut() {
        dependents.sort();
    }

    // Reject cycles before calculating any closure. A circular derivation graph
    // cannot establish a safe recomputation order and must not yield a plausible
    // partial "affected" set.
    let mut indegree: HashMap<String, usize> = HashMap::new();
    for (source, dependents) in &adjacency {
        indegree.entry(source.clone()).or_insert(0);
        for dependent in dependents {
            *indegree.entry(dependent.clone()).or_insert(0) += 1;
        }
    }
    let mut topological_queue: VecDeque<String> = indegree
        .iter()
        .filter_map(|(node, count)| (*count == 0).then_some(node.clone()))
        .collect();
    let mut topologically_processed = 0;
    while let Some(current) = topological_queue.pop_front() {
        topologically_processed += 1;
        if let Some(dependents) = adjacency.get(&current) {
            for dependent in dependents {
                let remaining = indegree
                    .get_mut(dependent)
                    .expect("each dependency target must have an indegree");
                *remaining -= 1;
                if *remaining == 0 {
                    topological_queue.push_back(dependent.clone());
                }
            }
        }
    }
    if topologically_processed != indegree.len() {
        return Err(vec![DependencyGraphIssue::CycleDetected]);
    }

    // Seed every changed artifact first, then walk a sorted adjacency list with
    // breadth-first search. First discovery therefore gives a shortest path;
    // sorted roots and neighbors make equal-length choices reproducible.
    let mut changed = changed_artifact_ids.to_vec();
    changed.sort();
    changed.dedup();
    let mut queue: VecDeque<String> = VecDeque::new();
    let mut causes: HashMap<String, Option<String>> = HashMap::new();
    for changed_id in changed {
        causes.insert(changed_id.clone(), None);
        queue.push_back(changed_id);
    }

    while let Some(current) = queue.pop_front() {
        if let Some(dependents) = adjacency.get(&current) {
            for dependent in dependents {
                if !causes.contains_key(dependent) {
                    causes.insert(dependent.clone(), Some(current.clone()));
                    queue.push_back(dependent.clone());
                }
            }
        }
    }

    let mut affected: Vec<AffectedArtifact> = causes
        .into_iter()
        .map(|(artifact_id, cause_parent_id)| AffectedArtifact {
            artifact_id,
            cause_parent_id,
        })
        .collect();
    affected.sort_by(|left, right| left.artifact_id.cmp(&right.artifact_id));
    Ok(affected)
}

/// Return a deterministic sorted closure of changed nodes and transitive
/// dependents. Use `affected_cause_tree` when callers also need a compact,
/// human-auditable explanation of why each artifact is stale.
pub fn affected_closure(
    changed_artifact_ids: &[String],
    dependencies: &[DependencyEdge],
) -> Result<Vec<String>, Vec<DependencyGraphIssue>> {
    affected_cause_tree(changed_artifact_ids, dependencies)
        .map(|affected| affected.into_iter().map(|item| item.artifact_id).collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fmt(version: &str) -> FormatVersion {
        FormatVersion::new("ifc-example", version)
    }

    fn revision(id: &str, parent: Option<&str>, day: u32, artifact_id: &str) -> ArtifactRevision {
        ArtifactRevision {
            artifact_id: artifact_id.into(),
            revision_id: id.into(),
            parent_revision_id: parent.map(str::to_owned),
            format: fmt("1"),
            content_sha256: "a".repeat(64),
            recorded_on_day: day,
        }
    }

    fn migration_candidate(
        id: &str,
        from: &str,
        to: &str,
        disposition: MigrationDisposition,
    ) -> MigrationCandidate {
        MigrationCandidate {
            migration_id: id.into(),
            from: fmt(from),
            to: fmt(to),
            tool_version: "tool-1.0.0".into(),
            disposition,
            source_artifact_sha256: "c".repeat(64),
            target_artifact_sha256: "d".repeat(64),
            verification_evidence_sha256: "e".repeat(64),
            required_invariants: vec![
                "geometry:living-room-area".into(),
                "assembly:roof-fire-rating".into(),
            ],
            source_invariants: vec![
                SemanticInvariant {
                    invariant_id: "geometry:living-room-area".into(),
                    canonical_value_sha256: "a".repeat(64),
                },
                SemanticInvariant {
                    invariant_id: "assembly:roof-fire-rating".into(),
                    canonical_value_sha256: "b".repeat(64),
                },
            ],
            target_invariants: vec![
                SemanticInvariant {
                    invariant_id: "geometry:living-room-area".into(),
                    canonical_value_sha256: "a".repeat(64),
                },
                SemanticInvariant {
                    invariant_id: "assembly:roof-fire-rating".into(),
                    canonical_value_sha256: "b".repeat(64),
                },
            ],
        }
    }

    fn migration(id: &str, from: &str, to: &str) -> VerifiedMigration {
        verify_migration_candidate(migration_candidate(
            id,
            from,
            to,
            MigrationDisposition::ApprovedForAutomaticUse,
        ))
        .expect("test fixture should satisfy migration invariants")
    }

    #[test]
    fn valid_append_only_revision_chain_is_accepted() {
        let records = vec![
            revision("r1", None, 1, "house-model"),
            revision("r2", Some("r1"), 2, "house-model"),
            revision("r3", Some("r2"), 3, "house-model"),
        ];
        assert!(validate_revision_ledger(&records).is_empty());
    }

    #[test]
    fn missing_parent_is_rejected() {
        let records = vec![revision("r2", Some("missing"), 2, "house-model")];
        assert!(
            validate_revision_ledger(&records).contains(&RevisionIssue::UnknownParent {
                revision_id: "r2".into(),
                parent_id: "missing".into(),
            })
        );
    }

    #[test]
    fn cross_artifact_parent_is_rejected() {
        let records = vec![
            revision("r1", None, 1, "house-model"),
            revision("r2", Some("r1"), 2, "unrelated-model"),
        ];
        assert!(validate_revision_ledger(&records).contains(
            &RevisionIssue::ParentArtifactMismatch {
                revision_id: "r2".into(),
                parent_id: "r1".into(),
            }
        ));
    }

    #[test]
    fn later_record_cannot_be_parent_of_earlier_record() {
        let records = vec![
            revision("r1", Some("r2"), 1, "house-model"),
            revision("r2", None, 2, "house-model"),
        ];
        assert!(validate_revision_ledger(&records).contains(
            &RevisionIssue::ParentRecordedAfterChild {
                revision_id: "r1".into(),
                parent_id: "r2".into(),
            }
        ));
    }

    #[test]
    fn parent_cycles_are_detected_even_on_the_same_day() {
        let records = vec![
            revision("r1", Some("r2"), 10, "house-model"),
            revision("r2", Some("r1"), 10, "house-model"),
        ];
        assert!(
            validate_revision_ledger(&records)
                .iter()
                .any(|issue| matches!(issue, RevisionIssue::ParentCycle(_)))
        );
    }

    #[test]
    fn planner_finds_a_verified_lossless_path() {
        let paths = vec![migration("m1", "1", "2"), migration("m2", "2", "3")];
        let plan = plan_lossless_migration(&fmt("1"), &fmt("3"), &paths, 3);
        assert_eq!(plan.status, MigrationPlanStatus::Ready);
        assert_eq!(
            plan.steps
                .iter()
                .map(|step| step.migration_id())
                .collect::<Vec<_>>(),
            vec!["m1", "m2"]
        );
    }

    #[test]
    fn unapproved_migration_dispositions_cannot_mint_verified_type() {
        for disposition in [
            MigrationDisposition::Pending,
            MigrationDisposition::RequiresHumanReview,
            MigrationDisposition::Lossy,
            MigrationDisposition::Experimental,
            MigrationDisposition::Retired,
        ] {
            let candidate = migration_candidate("m1", "1", "2", disposition);
            assert!(verify_migration_candidate(candidate).is_err());
        }
    }

    #[test]
    fn changed_critical_invariant_cannot_mint_verified_type() {
        let mut candidate = migration_candidate(
            "m1",
            "1",
            "2",
            MigrationDisposition::ApprovedForAutomaticUse,
        );
        candidate.target_invariants[0].canonical_value_sha256 = "f".repeat(64);
        let result = verify_migration_candidate(candidate);
        assert!(result.as_ref().is_err_and(|issues| issues.contains(
            &MigrationVerificationIssue::CriticalInvariantChanged(
                "geometry:living-room-area".into()
            )
        )));
    }

    #[test]
    fn missing_critical_invariant_on_either_side_is_rejected() {
        let mut source_missing = migration_candidate(
            "m1",
            "1",
            "2",
            MigrationDisposition::ApprovedForAutomaticUse,
        );
        source_missing.source_invariants.pop();
        let issues = verify_migration_candidate(source_missing).unwrap_err();
        assert!(
            issues.contains(&MigrationVerificationIssue::MissingRequiredInvariant {
                side: "source",
                invariant_id: "assembly:roof-fire-rating".into(),
            })
        );

        let mut target_missing = migration_candidate(
            "m2",
            "1",
            "2",
            MigrationDisposition::ApprovedForAutomaticUse,
        );
        target_missing.target_invariants.pop();
        let issues = verify_migration_candidate(target_missing).unwrap_err();
        assert!(
            issues.contains(&MigrationVerificationIssue::MissingRequiredInvariant {
                side: "target",
                invariant_id: "assembly:roof-fire-rating".into(),
            })
        );
    }

    #[test]
    fn duplicate_or_empty_required_invariants_are_rejected() {
        let mut duplicate = migration_candidate(
            "m1",
            "1",
            "2",
            MigrationDisposition::ApprovedForAutomaticUse,
        );
        duplicate
            .required_invariants
            .push("geometry:living-room-area".into());
        assert!(verify_migration_candidate(duplicate).is_err());

        let mut empty = migration_candidate(
            "m2",
            "1",
            "2",
            MigrationDisposition::ApprovedForAutomaticUse,
        );
        empty.required_invariants = vec![" ".into()];
        assert!(verify_migration_candidate(empty).is_err());
    }

    #[test]
    fn malformed_migration_or_invariant_digests_are_rejected_before_planning() {
        let mut bad_evidence = migration_candidate(
            "m1",
            "1",
            "2",
            MigrationDisposition::ApprovedForAutomaticUse,
        );
        bad_evidence.verification_evidence_sha256 = "bad".into();
        assert!(
            verify_migration_candidate(bad_evidence)
                .unwrap_err()
                .contains(&MigrationVerificationIssue::InvalidVerificationEvidenceDigest)
        );

        let mut bad_invariant = migration_candidate(
            "m2",
            "1",
            "2",
            MigrationDisposition::ApprovedForAutomaticUse,
        );
        bad_invariant.target_invariants[0].canonical_value_sha256 = "bad".into();
        assert!(
            verify_migration_candidate(bad_invariant)
                .unwrap_err()
                .iter()
                .any(|issue| matches!(
                    issue,
                    MigrationVerificationIssue::InvalidInvariantDigest { side: "target", .. }
                ))
        );
    }

    #[test]
    fn migration_depth_is_bounded() {
        let paths = vec![migration("m1", "1", "2"), migration("m2", "2", "3")];
        let plan = plan_lossless_migration(&fmt("1"), &fmt("3"), &paths, 1);
        assert_eq!(plan.status, MigrationPlanStatus::Blocked);
        assert!(plan.steps.is_empty());
    }

    #[test]
    fn cycle_does_not_make_migration_search_loop_forever() {
        let paths = vec![
            migration("m1", "1", "2"),
            migration("m2", "2", "1"),
            migration("m3", "2", "3"),
        ];
        let plan = plan_lossless_migration(&fmt("1"), &fmt("3"), &paths, 3);
        assert_eq!(plan.status, MigrationPlanStatus::Ready);
        assert_eq!(plan.steps.len(), 2);
    }

    #[test]
    fn duplicate_verified_migration_ids_block_the_catalog() {
        let duplicate = migration("same-id", "1", "2");
        let plan =
            plan_lossless_migration(&fmt("1"), &fmt("2"), &[duplicate.clone(), duplicate], 2);
        assert_eq!(plan.status, MigrationPlanStatus::Blocked);
        assert!(
            plan.blockers
                .contains(&MigrationBlocker::DuplicateVerifiedMigrationId(
                    "same-id".into()
                ))
        );
        assert!(plan.steps.is_empty());
    }

    #[test]
    fn same_format_needs_no_migration() {
        let plan = plan_lossless_migration(&fmt("2"), &fmt("2"), &[], 0);
        assert_eq!(plan.status, MigrationPlanStatus::Ready);
        assert!(plan.steps.is_empty());
    }

    fn dependency(input_id: &str, dependent_id: &str) -> DependencyEdge {
        DependencyEdge {
            input_id: input_id.into(),
            dependent_id: dependent_id.into(),
        }
    }

    #[test]
    fn component_change_invalidates_transitive_outputs_but_not_unrelated_data() {
        let edges = vec![
            dependency("material-roof", "model-roof"),
            dependency("model-roof", "calc-thermal"),
            dependency("model-roof", "calc-structural"),
            dependency("calc-structural", "review-package"),
            dependency("owner-profile", "identity-receipt"),
        ];
        let affected = affected_closure(&["material-roof".into()], &edges).unwrap();
        assert_eq!(
            affected,
            vec![
                "calc-structural".to_owned(),
                "calc-thermal".to_owned(),
                "material-roof".to_owned(),
                "model-roof".to_owned(),
                "review-package".to_owned(),
            ]
        );
        assert!(!affected.contains(&"identity-receipt".to_owned()));
    }

    fn reconstructed_cause_path(items: &[AffectedArtifact], id: &str) -> Vec<String> {
        let by_id: HashMap<&str, &AffectedArtifact> = items
            .iter()
            .map(|item| (item.artifact_id.as_str(), item))
            .collect();
        let mut path = Vec::new();
        let mut cursor = Some(id);
        while let Some(current) = cursor {
            path.push(current.to_owned());
            cursor = by_id
                .get(current)
                .and_then(|item| item.cause_parent_id.as_deref());
        }
        path.reverse();
        path
    }

    #[test]
    fn cause_tree_explains_a_deterministic_shortest_cause_chain() {
        let edges = vec![
            dependency("a", "c"),
            dependency("a", "b"),
            dependency("b", "d"),
            dependency("c", "d"),
            dependency("d", "e"),
        ];
        let affected = affected_cause_tree(&["a".into()], &edges).unwrap();

        assert_eq!(reconstructed_cause_path(&affected, "a"), vec!["a"]);
        assert_eq!(
            reconstructed_cause_path(&affected, "d"),
            vec!["a", "b", "d"]
        );
        assert_eq!(
            reconstructed_cause_path(&affected, "e"),
            vec!["a", "b", "d", "e"]
        );
        assert!(
            affected
                .windows(2)
                .all(|pair| pair[0].artifact_id < pair[1].artifact_id)
        );
    }

    #[test]
    fn all_changed_roots_remain_roots_and_ties_are_deterministic() {
        let edges = vec![dependency("b", "shared"), dependency("a", "shared")];
        let affected = affected_cause_tree(&["b".into(), "a".into(), "a".into()], &edges).unwrap();
        let shared = affected
            .iter()
            .find(|item| item.artifact_id == "shared")
            .unwrap();
        assert_eq!(shared.cause_parent_id.as_deref(), Some("a"));
        assert_eq!(
            affected
                .iter()
                .filter(|item| item.artifact_id == "a")
                .count(),
            1
        );
        assert_eq!(
            affected
                .iter()
                .find(|item| item.artifact_id == "b")
                .unwrap()
                .cause_parent_id,
            None
        );
    }

    #[test]
    fn dependency_closure_is_deterministic_for_reordered_edges() {
        let first = vec![
            dependency("a", "c"),
            dependency("a", "b"),
            dependency("c", "d"),
        ];
        let second = vec![
            dependency("c", "d"),
            dependency("a", "b"),
            dependency("a", "c"),
        ];
        assert_eq!(
            affected_closure(&["a".into()], &first),
            affected_closure(&["a".into()], &second)
        );
    }

    #[test]
    fn dependency_cycles_block_closure_instead_of_returning_partial_results() {
        let edges = vec![
            dependency("a", "b"),
            dependency("b", "c"),
            dependency("c", "a"),
            dependency("c", "d"),
        ];
        assert_eq!(
            affected_closure(&["a".into()], &edges),
            Err(vec![DependencyGraphIssue::CycleDetected])
        );
    }

    #[test]
    fn malformed_dependency_graph_is_rejected_without_partial_closure() {
        let edges = vec![
            dependency("a", "b"),
            dependency("a", "b"),
            dependency("b", "b"),
        ];
        let result = affected_closure(&["a".into()], &edges);
        assert!(result.is_err());
        let issues = result.unwrap_err();
        assert!(issues.iter().any(|issue| matches!(
            issue,
            DependencyGraphIssue::DuplicateEdge { input_id, dependent_id }
                if input_id == "a" && dependent_id == "b"
        )));
        assert!(issues.contains(&DependencyGraphIssue::SelfDependency("b".into())));
    }

    #[test]
    fn blank_change_id_fails_closed() {
        let result = affected_closure(&[" ".into()], &[]);
        assert_eq!(
            result,
            Err(vec![DependencyGraphIssue::EmptyChangedArtifactId])
        );
    }
}
