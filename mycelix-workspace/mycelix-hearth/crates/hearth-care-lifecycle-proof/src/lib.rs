// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Pure deterministic Care legacy/v2 lifecycle projection.
//!
//! This crate has no Holochain host calls. It receives already-observed source
//! facts and preserves the distinction between legacy revision state,
//! immutable completion evidence, and already-validated lifecycle admissions.
//! It never turns evidence presence into authority.

use std::collections::{HashMap, HashSet};

/// Opaque identity for a revision/evidence/admission fact.
///
/// Runtime adapters should populate this from canonical raw hash bytes. The
/// projection kernel intentionally does not know how a hash was fetched or
/// validated.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FactId(Vec<u8>);

impl FactId {
    pub fn new(bytes: impl Into<Vec<u8>>) -> Self {
        Self(bytes.into())
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LegacyScheduleStatus {
    Active,
    Paused,
    Completed,
}

/// One legacy CareSchedule revision in the explicit update graph.
///
/// `parent = None` means the original Create/root action. Updates name their
/// exact parent revision; no timestamp or returned-list position is used as an
/// implicit parent/canonicality rule.
///
/// `deleted` means at least one valid Delete relationship was observed for this
/// revision. This first coexistence kernel does not invent deletion semantics:
/// any observed deletion yields `LegacyDeletionConflict`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LegacyRevision {
    pub id: FactId,
    pub parent: Option<FactId>,
    pub status: LegacyScheduleStatus,
    pub deleted: bool,
}

/// Immutable completion attestation. This is evidence, not lifecycle admission.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CompletionEvidence {
    pub id: FactId,
    pub revision_id: FactId,
}

/// A completion admission whose consensus/authority validation happened in a
/// separate layer. The projection kernel does not grant admission authority.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ValidatedCompletionAdmission {
    pub id: FactId,
    pub root_id: FactId,
    pub evidence_ids: Vec<FactId>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Source<T> {
    Known(T),
    Unknown,
    Unavailable,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Availability {
    Known,
    Unknown,
    Unavailable,
}

impl<T> Source<T> {
    pub fn availability(&self) -> Availability {
        match self {
            Self::Known(_) => Availability::Known,
            Self::Unknown => Availability::Unknown,
            Self::Unavailable => Availability::Unavailable,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SourceAvailability {
    pub legacy: Availability,
    pub completion_evidence: Availability,
    pub admissions: Availability,
}

impl SourceAvailability {
    fn has_unavailable(self) -> bool {
        matches!(self.legacy, Availability::Unavailable)
            || matches!(self.completion_evidence, Availability::Unavailable)
            || matches!(self.admissions, Availability::Unavailable)
    }

    fn has_unknown(self) -> bool {
        matches!(self.legacy, Availability::Unknown)
            || matches!(self.completion_evidence, Availability::Unknown)
            || matches!(self.admissions, Availability::Unknown)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LifecycleSources {
    pub legacy: Source<Vec<LegacyRevision>>,
    pub completion_evidence: Source<Vec<CompletionEvidence>>,
    pub admissions: Source<Vec<ValidatedCompletionAdmission>>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LifecycleDisposition {
    /// At least one required source is known to be unavailable.
    Unavailable,
    /// No source is unavailable, but at least one required source is not yet known.
    Unknown,
    /// One unconflicted legacy head, no completion evidence, no admission.
    LegacyOnly { status: LegacyScheduleStatus },
    /// Attestation exists, but no validated lifecycle admission exists.
    CompletionEvidenceOnly {
        legacy_status: LegacyScheduleStatus,
    },
    /// Legacy itself says Completed, but no validated v2 admission exists.
    LegacyCompletedOnly,
    /// Legacy Completed and validated v2 admission agree on completion.
    LegacyV2Agreement,
    /// Validated v2 admission exists while the single legacy head is not Completed.
    LegacyV2Conflict {
        legacy_status: LegacyScheduleStatus,
    },
    /// Multiple terminal legacy heads exist. No winner is selected.
    LegacyForkConflict,
    /// At least one legacy revision has valid delete metadata. No deletion
    /// interpretation is invented by this kernel.
    LegacyDeletionConflict,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LifecycleProjection {
    pub sources: SourceAvailability,
    pub disposition: LifecycleDisposition,
    pub root_id: Option<FactId>,
    pub legacy_head_ids: Vec<FactId>,
    pub completion_evidence_count: usize,
    pub admission_count: usize,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProjectionError {
    NoLegacySchedule,
    DuplicateRevision(FactId),
    ExpectedSingleRoot { count: usize },
    MissingParent { revision: FactId, parent: FactId },
    LegacyCycle(FactId),
    DuplicateCompletionEvidence(FactId),
    CompletionEvidenceReferencesUnknownRevision {
        evidence: FactId,
        revision: FactId,
    },
    DuplicateAdmission(FactId),
    AdmissionRootMismatch {
        admission: FactId,
        expected_root: FactId,
        claimed_root: FactId,
    },
    AdmissionHasNoEvidence(FactId),
    DuplicateAdmissionEvidenceReference {
        admission: FactId,
        evidence: FactId,
    },
    AdmissionReferencesUnknownEvidence {
        admission: FactId,
        evidence: FactId,
    },
}

struct LegacyAnalysis {
    root_id: FactId,
    head_ids: Vec<FactId>,
    single_head_status: Option<LegacyScheduleStatus>,
    revision_ids: HashSet<FactId>,
    has_deletion: bool,
}

/// Project independently sourced legacy state, immutable attestations, and
/// already-validated admissions without strengthening one source into another.
pub fn project_lifecycle(
    sources: LifecycleSources,
) -> Result<LifecycleProjection, ProjectionError> {
    let availability = SourceAvailability {
        legacy: sources.legacy.availability(),
        completion_evidence: sources.completion_evidence.availability(),
        admissions: sources.admissions.availability(),
    };

    if availability.has_unavailable() {
        return Ok(incomplete_projection(
            availability,
            LifecycleDisposition::Unavailable,
        ));
    }
    if availability.has_unknown() {
        return Ok(incomplete_projection(
            availability,
            LifecycleDisposition::Unknown,
        ));
    }

    let Source::Known(legacy) = sources.legacy else {
        unreachable!("source availability already proved legacy Known")
    };
    let Source::Known(completion_evidence) = sources.completion_evidence else {
        unreachable!("source availability already proved completion evidence Known")
    };
    let Source::Known(admissions) = sources.admissions else {
        unreachable!("source availability already proved admissions Known")
    };

    let legacy_analysis = analyze_legacy_graph(&legacy)?;
    validate_completion_evidence(&completion_evidence, &legacy_analysis.revision_ids)?;
    validate_admissions(
        &admissions,
        &legacy_analysis.root_id,
        &completion_evidence,
    )?;

    let evidence_count = completion_evidence.len();
    let admission_count = admissions.len();

    let disposition = if legacy_analysis.has_deletion {
        LifecycleDisposition::LegacyDeletionConflict
    } else if legacy_analysis.head_ids.len() > 1 {
        LifecycleDisposition::LegacyForkConflict
    } else {
        let status = legacy_analysis
            .single_head_status
            .expect("one legacy head always has one status");

        if admission_count > 0 {
            match status {
                LegacyScheduleStatus::Completed => LifecycleDisposition::LegacyV2Agreement,
                LegacyScheduleStatus::Active | LegacyScheduleStatus::Paused => {
                    LifecycleDisposition::LegacyV2Conflict {
                        legacy_status: status,
                    }
                }
            }
        } else if status == LegacyScheduleStatus::Completed {
            LifecycleDisposition::LegacyCompletedOnly
        } else if evidence_count > 0 {
            LifecycleDisposition::CompletionEvidenceOnly {
                legacy_status: status,
            }
        } else {
            LifecycleDisposition::LegacyOnly { status }
        }
    };

    Ok(LifecycleProjection {
        sources: availability,
        disposition,
        root_id: Some(legacy_analysis.root_id),
        legacy_head_ids: legacy_analysis.head_ids,
        completion_evidence_count: evidence_count,
        admission_count,
    })
}

fn incomplete_projection(
    sources: SourceAvailability,
    disposition: LifecycleDisposition,
) -> LifecycleProjection {
    LifecycleProjection {
        sources,
        disposition,
        root_id: None,
        legacy_head_ids: Vec::new(),
        completion_evidence_count: 0,
        admission_count: 0,
    }
}

fn analyze_legacy_graph(revisions: &[LegacyRevision]) -> Result<LegacyAnalysis, ProjectionError> {
    if revisions.is_empty() {
        return Err(ProjectionError::NoLegacySchedule);
    }

    let mut by_id: HashMap<FactId, &LegacyRevision> = HashMap::with_capacity(revisions.len());
    for revision in revisions {
        if by_id.insert(revision.id.clone(), revision).is_some() {
            return Err(ProjectionError::DuplicateRevision(revision.id.clone()));
        }
    }

    for revision in revisions {
        if let Some(parent) = &revision.parent
            && !by_id.contains_key(parent)
        {
            return Err(ProjectionError::MissingParent {
                revision: revision.id.clone(),
                parent: parent.clone(),
            });
        }
    }

    let roots: Vec<&LegacyRevision> = revisions
        .iter()
        .filter(|revision| revision.parent.is_none())
        .collect();
    if roots.len() != 1 {
        return Err(ProjectionError::ExpectedSingleRoot { count: roots.len() });
    }
    let root_id = roots[0].id.clone();

    // Every node must trace to the unique root without cycling.
    for revision in revisions {
        let mut current = revision.id.clone();
        let mut seen = HashSet::new();
        loop {
            if !seen.insert(current.clone()) {
                return Err(ProjectionError::LegacyCycle(current));
            }
            let node = by_id
                .get(&current)
                .expect("all parents were proven present in the revision map");
            match &node.parent {
                Some(parent) => current = parent.clone(),
                None => {
                    if node.id != root_id {
                        return Err(ProjectionError::ExpectedSingleRoot { count: 2 });
                    }
                    break;
                }
            }
        }
    }

    let referenced_as_parent: HashSet<FactId> = revisions
        .iter()
        .filter_map(|revision| revision.parent.clone())
        .collect();
    let mut heads: Vec<&LegacyRevision> = revisions
        .iter()
        .filter(|revision| !referenced_as_parent.contains(&revision.id))
        .collect();
    heads.sort_by(|left, right| left.id.cmp(&right.id));

    let head_ids = heads.iter().map(|head| head.id.clone()).collect::<Vec<_>>();
    let single_head_status = (heads.len() == 1).then_some(heads[0].status);

    Ok(LegacyAnalysis {
        root_id,
        head_ids,
        single_head_status,
        revision_ids: by_id.into_keys().collect(),
        has_deletion: revisions.iter().any(|revision| revision.deleted),
    })
}

fn validate_completion_evidence(
    evidence: &[CompletionEvidence],
    revision_ids: &HashSet<FactId>,
) -> Result<(), ProjectionError> {
    let mut evidence_ids = HashSet::with_capacity(evidence.len());
    for item in evidence {
        if !evidence_ids.insert(item.id.clone()) {
            return Err(ProjectionError::DuplicateCompletionEvidence(item.id.clone()));
        }
        if !revision_ids.contains(&item.revision_id) {
            return Err(ProjectionError::CompletionEvidenceReferencesUnknownRevision {
                evidence: item.id.clone(),
                revision: item.revision_id.clone(),
            });
        }
    }
    Ok(())
}

fn validate_admissions(
    admissions: &[ValidatedCompletionAdmission],
    root_id: &FactId,
    evidence: &[CompletionEvidence],
) -> Result<(), ProjectionError> {
    let evidence_ids: HashSet<FactId> = evidence.iter().map(|item| item.id.clone()).collect();
    let mut admission_ids = HashSet::with_capacity(admissions.len());

    for admission in admissions {
        if !admission_ids.insert(admission.id.clone()) {
            return Err(ProjectionError::DuplicateAdmission(admission.id.clone()));
        }
        if &admission.root_id != root_id {
            return Err(ProjectionError::AdmissionRootMismatch {
                admission: admission.id.clone(),
                expected_root: root_id.clone(),
                claimed_root: admission.root_id.clone(),
            });
        }
        if admission.evidence_ids.is_empty() {
            return Err(ProjectionError::AdmissionHasNoEvidence(admission.id.clone()));
        }

        let mut seen_evidence = HashSet::with_capacity(admission.evidence_ids.len());
        for evidence_id in &admission.evidence_ids {
            if !seen_evidence.insert(evidence_id.clone()) {
                return Err(ProjectionError::DuplicateAdmissionEvidenceReference {
                    admission: admission.id.clone(),
                    evidence: evidence_id.clone(),
                });
            }
            if !evidence_ids.contains(evidence_id) {
                return Err(ProjectionError::AdmissionReferencesUnknownEvidence {
                    admission: admission.id.clone(),
                    evidence: evidence_id.clone(),
                });
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(byte: u8) -> FactId {
        FactId::new(vec![byte])
    }

    fn revision(byte: u8, parent: Option<u8>, status: LegacyScheduleStatus) -> LegacyRevision {
        LegacyRevision {
            id: id(byte),
            parent: parent.map(id),
            status,
            deleted: false,
        }
    }

    fn deleted_revision(
        byte: u8,
        parent: Option<u8>,
        status: LegacyScheduleStatus,
    ) -> LegacyRevision {
        LegacyRevision {
            deleted: true,
            ..revision(byte, parent, status)
        }
    }

    fn evidence(byte: u8, revision: u8) -> CompletionEvidence {
        CompletionEvidence {
            id: id(byte),
            revision_id: id(revision),
        }
    }

    fn admission(byte: u8, root: u8, evidence_ids: &[u8]) -> ValidatedCompletionAdmission {
        ValidatedCompletionAdmission {
            id: id(byte),
            root_id: id(root),
            evidence_ids: evidence_ids.iter().copied().map(id).collect(),
        }
    }

    fn known(
        legacy: Vec<LegacyRevision>,
        evidence: Vec<CompletionEvidence>,
        admissions: Vec<ValidatedCompletionAdmission>,
    ) -> LifecycleSources {
        LifecycleSources {
            legacy: Source::Known(legacy),
            completion_evidence: Source::Known(evidence),
            admissions: Source::Known(admissions),
        }
    }

    #[test]
    fn active_legacy_without_v2_facts_is_legacy_only() {
        let projection = project_lifecycle(known(
            vec![revision(1, None, LegacyScheduleStatus::Active)],
            vec![],
            vec![],
        ))
        .unwrap();
        assert_eq!(
            projection.disposition,
            LifecycleDisposition::LegacyOnly {
                status: LegacyScheduleStatus::Active
            }
        );
        assert_eq!(projection.root_id, Some(id(1)));
    }

    #[test]
    fn completion_attestation_never_becomes_admission() {
        let projection = project_lifecycle(known(
            vec![revision(1, None, LegacyScheduleStatus::Active)],
            vec![evidence(10, 1)],
            vec![],
        ))
        .unwrap();
        assert_eq!(
            projection.disposition,
            LifecycleDisposition::CompletionEvidenceOnly {
                legacy_status: LegacyScheduleStatus::Active
            }
        );
        assert_eq!(projection.completion_evidence_count, 1);
        assert_eq!(projection.admission_count, 0);
    }

    #[test]
    fn legacy_completed_without_v2_admission_stays_legacy_completed_only() {
        let projection = project_lifecycle(known(
            vec![revision(1, None, LegacyScheduleStatus::Completed)],
            vec![evidence(10, 1)],
            vec![],
        ))
        .unwrap();
        assert_eq!(
            projection.disposition,
            LifecycleDisposition::LegacyCompletedOnly
        );
        assert_eq!(projection.completion_evidence_count, 1);
    }

    #[test]
    fn legacy_completed_and_v2_admission_agree_once() {
        let projection = project_lifecycle(known(
            vec![revision(1, None, LegacyScheduleStatus::Completed)],
            vec![evidence(10, 1)],
            vec![admission(20, 1, &[10])],
        ))
        .unwrap();
        assert_eq!(
            projection.disposition,
            LifecycleDisposition::LegacyV2Agreement
        );
        assert_eq!(projection.admission_count, 1);
    }

    #[test]
    fn admission_does_not_silently_override_active_legacy_head() {
        let projection = project_lifecycle(known(
            vec![revision(1, None, LegacyScheduleStatus::Active)],
            vec![evidence(10, 1)],
            vec![admission(20, 1, &[10])],
        ))
        .unwrap();
        assert_eq!(
            projection.disposition,
            LifecycleDisposition::LegacyV2Conflict {
                legacy_status: LegacyScheduleStatus::Active
            }
        );
    }

    #[test]
    fn admission_does_not_silently_override_paused_legacy_head() {
        let projection = project_lifecycle(known(
            vec![revision(1, None, LegacyScheduleStatus::Paused)],
            vec![evidence(10, 1)],
            vec![admission(20, 1, &[10])],
        ))
        .unwrap();
        assert_eq!(
            projection.disposition,
            LifecycleDisposition::LegacyV2Conflict {
                legacy_status: LegacyScheduleStatus::Paused
            }
        );
    }

    #[test]
    fn competing_legacy_heads_are_never_last_write_wins() {
        let projection = project_lifecycle(known(
            vec![
                revision(1, None, LegacyScheduleStatus::Active),
                revision(2, Some(1), LegacyScheduleStatus::Completed),
                revision(3, Some(1), LegacyScheduleStatus::Paused),
            ],
            vec![evidence(10, 2)],
            vec![],
        ))
        .unwrap();
        assert_eq!(
            projection.disposition,
            LifecycleDisposition::LegacyForkConflict
        );
        assert_eq!(projection.legacy_head_ids, vec![id(2), id(3)]);
    }

    #[test]
    fn root_delete_is_explicit_conflict() {
        let projection = project_lifecycle(known(
            vec![deleted_revision(1, None, LegacyScheduleStatus::Completed)],
            vec![],
            vec![],
        ))
        .unwrap();
        assert_eq!(
            projection.disposition,
            LifecycleDisposition::LegacyDeletionConflict
        );
    }

    #[test]
    fn deleted_historical_revision_is_not_silently_ignored() {
        let projection = project_lifecycle(known(
            vec![
                deleted_revision(1, None, LegacyScheduleStatus::Active),
                revision(2, Some(1), LegacyScheduleStatus::Completed),
            ],
            vec![evidence(10, 2)],
            vec![admission(20, 1, &[10])],
        ))
        .unwrap();
        assert_eq!(
            projection.disposition,
            LifecycleDisposition::LegacyDeletionConflict
        );
    }

    #[test]
    fn update_chain_normalizes_to_original_root() {
        let projection = project_lifecycle(known(
            vec![
                revision(1, None, LegacyScheduleStatus::Active),
                revision(2, Some(1), LegacyScheduleStatus::Paused),
                revision(3, Some(2), LegacyScheduleStatus::Completed),
            ],
            vec![evidence(10, 2), evidence(11, 3)],
            vec![],
        ))
        .unwrap();
        assert_eq!(projection.root_id, Some(id(1)));
        assert_eq!(projection.legacy_head_ids, vec![id(3)]);
        assert_eq!(
            projection.disposition,
            LifecycleDisposition::LegacyCompletedOnly
        );
    }

    #[test]
    fn missing_parent_fails_closed() {
        let error = project_lifecycle(known(
            vec![revision(2, Some(99), LegacyScheduleStatus::Active)],
            vec![],
            vec![],
        ))
        .unwrap_err();
        assert_eq!(
            error,
            ProjectionError::MissingParent {
                revision: id(2),
                parent: id(99)
            }
        );
    }

    #[test]
    fn disconnected_roots_fail_closed() {
        let error = project_lifecycle(known(
            vec![
                revision(1, None, LegacyScheduleStatus::Active),
                revision(2, None, LegacyScheduleStatus::Paused),
            ],
            vec![],
            vec![],
        ))
        .unwrap_err();
        assert_eq!(error, ProjectionError::ExpectedSingleRoot { count: 2 });
    }

    #[test]
    fn cycle_fails_closed() {
        let error = project_lifecycle(known(
            vec![
                revision(1, None, LegacyScheduleStatus::Active),
                revision(2, Some(3), LegacyScheduleStatus::Paused),
                revision(3, Some(2), LegacyScheduleStatus::Completed),
            ],
            vec![],
            vec![],
        ))
        .unwrap_err();
        assert!(matches!(error, ProjectionError::LegacyCycle(_)));
    }

    #[test]
    fn unknown_revision_in_completion_evidence_fails_closed() {
        let error = project_lifecycle(known(
            vec![revision(1, None, LegacyScheduleStatus::Active)],
            vec![evidence(10, 99)],
            vec![],
        ))
        .unwrap_err();
        assert_eq!(
            error,
            ProjectionError::CompletionEvidenceReferencesUnknownRevision {
                evidence: id(10),
                revision: id(99)
            }
        );
    }

    #[test]
    fn admission_must_bind_stable_root() {
        let error = project_lifecycle(known(
            vec![revision(1, None, LegacyScheduleStatus::Completed)],
            vec![evidence(10, 1)],
            vec![admission(20, 9, &[10])],
        ))
        .unwrap_err();
        assert_eq!(
            error,
            ProjectionError::AdmissionRootMismatch {
                admission: id(20),
                expected_root: id(1),
                claimed_root: id(9)
            }
        );
    }

    #[test]
    fn admission_must_reference_known_evidence() {
        let error = project_lifecycle(known(
            vec![revision(1, None, LegacyScheduleStatus::Completed)],
            vec![evidence(10, 1)],
            vec![admission(20, 1, &[99])],
        ))
        .unwrap_err();
        assert_eq!(
            error,
            ProjectionError::AdmissionReferencesUnknownEvidence {
                admission: id(20),
                evidence: id(99)
            }
        );
    }

    #[test]
    fn equivalent_multiple_admissions_do_not_multiply_disposition() {
        let projection = project_lifecycle(known(
            vec![revision(1, None, LegacyScheduleStatus::Completed)],
            vec![evidence(10, 1)],
            vec![admission(20, 1, &[10]), admission(21, 1, &[10])],
        ))
        .unwrap();
        assert_eq!(
            projection.disposition,
            LifecycleDisposition::LegacyV2Agreement
        );
        assert_eq!(projection.admission_count, 2);
    }

    #[test]
    fn unavailable_source_blocks_positive_projection() {
        let projection = project_lifecycle(LifecycleSources {
            legacy: Source::Known(vec![revision(1, None, LegacyScheduleStatus::Completed)]),
            completion_evidence: Source::Unavailable,
            admissions: Source::Known(vec![]),
        })
        .unwrap();
        assert_eq!(projection.disposition, LifecycleDisposition::Unavailable);
        assert_eq!(
            projection.sources.completion_evidence,
            Availability::Unavailable
        );
    }

    #[test]
    fn unknown_source_blocks_positive_projection() {
        let projection = project_lifecycle(LifecycleSources {
            legacy: Source::Known(vec![revision(1, None, LegacyScheduleStatus::Completed)]),
            completion_evidence: Source::Known(vec![]),
            admissions: Source::Unknown,
        })
        .unwrap();
        assert_eq!(projection.disposition, LifecycleDisposition::Unknown);
        assert_eq!(projection.sources.admissions, Availability::Unknown);
    }
}
