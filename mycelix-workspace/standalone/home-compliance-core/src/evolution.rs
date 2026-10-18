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
    UnknownParent { revision_id: String, parent_id: String },
    ParentArtifactMismatch { revision_id: String, parent_id: String },
    ParentRecordedAfterChild { revision_id: String, parent_id: String },
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
            issues.push(RevisionIssue::EmptyArtifactId(
                revision.revision_id.clone(),
            ));
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
                issues.push(RevisionIssue::ParentCycle(
                    revision.revision_id.clone(),
                ));
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
pub enum MigrationAssurance {
    VerifiedLossless,
    RequiresHumanReview,
    Lossy,
    Experimental,
    Retired,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MigrationDefinition {
    pub migration_id: String,
    pub from: FormatVersion,
    pub to: FormatVersion,
    /// Version or immutable identifier for the migration implementation.
    pub tool_version: String,
    pub assurance: MigrationAssurance,
    /// Digest of the conformance fixture/report package. Syntax only here.
    pub verification_evidence_sha256: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum MigrationBlocker {
    InvalidSourceVersion,
    InvalidTargetVersion,
    InvalidMigrationMetadata(String),
    InvalidVerificationEvidenceDigest(String),
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
    pub steps: Vec<MigrationDefinition>,
    pub blockers: Vec<MigrationBlocker>,
}

/// Find a shortest path of verified-lossless migration edges.
///
/// This is a path planner, not a migration executor. It assumes that the
/// VerifiedLossless label has already been earned through independent
/// round-trip tests and review. A hash-shaped evidence string alone is not
/// proof that any report exists or that a migration preserves meaning.
pub fn plan_lossless_migration(
    from: &FormatVersion,
    to: &FormatVersion,
    migrations: &[MigrationDefinition],
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

    let mut adjacency: HashMap<FormatVersion, Vec<usize>> = HashMap::new();
    for (index, migration) in migrations.iter().enumerate() {
        if !nonblank(&migration.migration_id)
            || !migration.from.is_well_formed()
            || !migration.to.is_well_formed()
            || !nonblank(&migration.tool_version)
        {
            blockers.push(MigrationBlocker::InvalidMigrationMetadata(
                migration.migration_id.clone(),
            ));
            continue;
        }

        if migration.assurance != MigrationAssurance::VerifiedLossless {
            continue;
        }
        if !is_sha256_hex(&migration.verification_evidence_sha256) {
            blockers.push(MigrationBlocker::InvalidVerificationEvidenceDigest(
                migration.migration_id.clone(),
            ));
            continue;
        }
        adjacency
            .entry(migration.from.clone())
            .or_default()
            .push(index);
    }

    for indexes in adjacency.values_mut() {
        indexes.sort_by(|left, right| {
            migrations[*left]
                .migration_id
                .cmp(&migrations[*right].migration_id)
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
            let migration = &migrations[*index];
            let next = migration.to.clone();
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

    // Do not report Ready if the migration catalog contains malformed
    // metadata: ambiguity in the catalog is itself a fail-closed condition.
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
    DuplicateEdge { input_id: String, dependent_id: String },
}

/// Return a deterministic closure of changed nodes plus all transitively
/// dependent outputs. If the dependency graph is malformed, return all issues
/// instead of a partial closure that might leave stale safety findings current.
///
/// This identifies what must be re-evaluated; it does not run the evaluators or
/// mark real stored findings stale. The caller must persist the invalidation.
pub fn affected_closure(
    changed_artifact_ids: &[String],
    dependencies: &[DependencyEdge],
) -> Result<Vec<String>, Vec<DependencyGraphIssue>> {
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
            issues.push(DependencyGraphIssue::SelfDependency(
                edge.input_id.clone(),
            ));
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

    let mut queue: VecDeque<String> = VecDeque::new();
    let mut visited: HashSet<String> = HashSet::new();
    let mut changed = changed_artifact_ids.to_vec();
    changed.sort();
    changed.dedup();
    for changed_id in changed {
        if visited.insert(changed_id.clone()) {
            queue.push_back(changed_id);
        }
    }

    while let Some(current) = queue.pop_front() {
        if let Some(dependents) = adjacency.get(&current) {
            for dependent in dependents {
                if visited.insert(dependent.clone()) {
                    queue.push_back(dependent.clone());
                }
            }
        }
    }

    let mut affected: Vec<String> = visited.into_iter().collect();
    affected.sort();
    Ok(affected)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fmt(version: &str) -> FormatVersion {
        FormatVersion::new("ifc-example", version)
    }

    fn revision(
        id: &str,
        parent: Option<&str>,
        day: u32,
        artifact_id: &str,
    ) -> ArtifactRevision {
        ArtifactRevision {
            artifact_id: artifact_id.into(),
            revision_id: id.into(),
            parent_revision_id: parent.map(str::to_owned),
            format: fmt("1"),
            content_sha256: "a".repeat(64),
            recorded_on_day: day,
        }
    }

    fn migration(
        id: &str,
        from: &str,
        to: &str,
        assurance: MigrationAssurance,
    ) -> MigrationDefinition {
        MigrationDefinition {
            migration_id: id.into(),
            from: fmt(from),
            to: fmt(to),
            tool_version: "tool-1.0.0".into(),
            assurance,
            verification_evidence_sha256: "b".repeat(64),
        }
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
        assert!(validate_revision_ledger(&records).contains(
            &RevisionIssue::UnknownParent {
                revision_id: "r2".into(),
                parent_id: "missing".into(),
            }
        ));
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
        assert!(validate_revision_ledger(&records)
            .iter()
            .any(|issue| matches!(issue, RevisionIssue::ParentCycle(_))));
    }

    #[test]
    fn planner_finds_a_verified_lossless_path() {
        let paths = vec![
            migration("m1", "1", "2", MigrationAssurance::VerifiedLossless),
            migration("m2", "2", "3", MigrationAssurance::VerifiedLossless),
        ];
        let plan = plan_lossless_migration(&fmt("1"), &fmt("3"), &paths, 3);
        assert_eq!(plan.status, MigrationPlanStatus::Ready);
        assert_eq!(
            plan.steps.iter().map(|step| step.migration_id.as_str()).collect::<Vec<_>>(),
            vec!["m1", "m2"]
        );
    }

    #[test]
    fn lossy_or_review_required_edges_are_never_auto_selected() {
        for assurance in [
            MigrationAssurance::RequiresHumanReview,
            MigrationAssurance::Lossy,
            MigrationAssurance::Experimental,
            MigrationAssurance::Retired,
        ] {
            let paths = vec![migration("m1", "1", "2", assurance)];
            let plan = plan_lossless_migration(&fmt("1"), &fmt("2"), &paths, 1);
            assert_eq!(plan.status, MigrationPlanStatus::Blocked);
            assert!(plan.steps.is_empty());
        }
    }

    #[test]
    fn migration_depth_is_bounded() {
        let paths = vec![
            migration("m1", "1", "2", MigrationAssurance::VerifiedLossless),
            migration("m2", "2", "3", MigrationAssurance::VerifiedLossless),
        ];
        let plan = plan_lossless_migration(&fmt("1"), &fmt("3"), &paths, 1);
        assert_eq!(plan.status, MigrationPlanStatus::Blocked);
        assert!(plan.steps.is_empty());
    }

    #[test]
    fn cycle_does_not_make_migration_search_loop_forever() {
        let paths = vec![
            migration("m1", "1", "2", MigrationAssurance::VerifiedLossless),
            migration("m2", "2", "1", MigrationAssurance::VerifiedLossless),
            migration("m3", "2", "3", MigrationAssurance::VerifiedLossless),
        ];
        let plan = plan_lossless_migration(&fmt("1"), &fmt("3"), &paths, 3);
        assert_eq!(plan.status, MigrationPlanStatus::Ready);
        assert_eq!(plan.steps.len(), 2);
    }

    #[test]
    fn malformed_migration_evidence_digest_blocks_that_path() {
        let mut edge = migration("m1", "1", "2", MigrationAssurance::VerifiedLossless);
        edge.verification_evidence_sha256 = "bad".into();
        let plan = plan_lossless_migration(&fmt("1"), &fmt("2"), &[edge], 2);
        assert_eq!(plan.status, MigrationPlanStatus::Blocked);
        assert!(plan.blockers.iter().any(|blocker| matches!(
            blocker,
            MigrationBlocker::InvalidVerificationEvidenceDigest(id) if id == "m1"
        )));
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
                "calc-structural",
                "calc-thermal",
                "material-roof",
                "model-roof",
                "review-package",
            ]
        );
        assert!(!affected.contains(&"identity-receipt".to_owned()));
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
    fn dependency_cycles_terminate_and_return_unique_ids() {
        let edges = vec![
            dependency("a", "b"),
            dependency("b", "c"),
            dependency("c", "a"),
            dependency("c", "d"),
        ];
        let affected = affected_closure(&["a".into()], &edges).unwrap();
        assert_eq!(affected, vec!["a", "b", "c", "d"]);
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
        assert!(issues
            .contains(&DependencyGraphIssue::SelfDependency("b".into())));
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
