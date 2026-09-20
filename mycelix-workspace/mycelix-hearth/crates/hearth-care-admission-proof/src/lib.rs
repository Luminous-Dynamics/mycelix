// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Pure structural semantics for Hearth Care completion admission.
//!
//! This crate has no Holochain host calls and authenticates no input facts.
//! Runtime adapters must independently prove canonical root, evidence, and
//! membership provenance before using a successful structural result as an
//! input to consensus validation.

pub const ADMISSION_SCHEMA_V1: u8 = 1;
pub const MAX_EVIDENCE_REFS_V1: usize = 32;

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

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum CareCompletionAdmissionProfileV1 {
    CreatorOrCurrentGuardianV1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StructuralAuthority {
    StructuralOnly,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ScheduleRootFact {
    pub root_id: FactId,
    pub hearth_id: FactId,
    pub creator_actor_id: FactId,
}

/// One already-normalized completion-evidence observation.
///
/// `attestor_actor_id` and `revision_assignee_actor_id` are deliberately
/// retained so tests and downstream diagnostics can preserve provenance. They
/// are never admission-authority inputs in v1.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CompletionEvidenceFact {
    pub id: FactId,
    pub normalized_root_id: FactId,
    pub hearth_id: FactId,
    pub attestor_actor_id: FactId,
    pub revision_assignee_actor_id: FactId,
}

/// Result of a membership-evidence layer supplied to this pure kernel.
///
/// This enum is not proof that the input is authentic. The HDI/integrity layer
/// must establish the underlying Kinship provenance and freshness theorem.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MembershipDisposition {
    FreshActive { is_guardian: bool },
    StaleOrInactive,
    Unknown,
    Unavailable,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MembershipFact {
    pub proof_id: FactId,
    pub actor_id: FactId,
    pub hearth_id: FactId,
    pub disposition: MembershipDisposition,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AdmissionCandidate {
    pub schema_version: u8,
    pub profile: CareCompletionAdmissionProfileV1,
    pub actor_id: FactId,
    pub root: ScheduleRootFact,
    /// Must be strictly sorted by `CompletionEvidenceFact.id` and unique.
    pub evidence: Vec<CompletionEvidenceFact>,
    pub membership: MembershipFact,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct LifecycleCompletionKey {
    pub profile: CareCompletionAdmissionProfileV1,
    pub root_id: FactId,
    pub hearth_id: FactId,
}

/// Structurally admissible semantics only.
///
/// This value is intentionally not named `ValidatedCompletionAdmission`: the
/// caller has not proven canonical Holochain provenance merely by constructing
/// these Rust facts.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StructuralAdmission {
    pub schema_version: u8,
    pub profile: CareCompletionAdmissionProfileV1,
    pub actor_id: FactId,
    pub root_id: FactId,
    pub hearth_id: FactId,
    pub evidence_ids: Vec<FactId>,
    pub membership_proof_id: FactId,
}

impl StructuralAdmission {
    pub fn authority(&self) -> StructuralAuthority {
        StructuralAuthority::StructuralOnly
    }

    pub fn authenticated_admission_established(&self) -> bool {
        false
    }

    pub fn universal_completion_established(&self) -> bool {
        false
    }

    pub fn lifecycle_completion_key(&self) -> LifecycleCompletionKey {
        LifecycleCompletionKey {
            profile: self.profile,
            root_id: self.root_id.clone(),
            hearth_id: self.hearth_id.clone(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AdmissionError {
    UnsupportedSchemaVersion(u8),
    NoCompletionEvidence,
    TooManyCompletionEvidence {
        count: usize,
        maximum: usize,
    },
    DuplicateEvidenceReference(FactId),
    EvidenceReferencesNotStrictlySorted {
        previous: FactId,
        current: FactId,
    },
    EvidenceRootMismatch {
        evidence: FactId,
        expected_root: FactId,
        observed_root: FactId,
    },
    EvidenceHearthMismatch {
        evidence: FactId,
        expected_hearth: FactId,
        observed_hearth: FactId,
    },
    MembershipActorMismatch {
        expected_actor: FactId,
        observed_actor: FactId,
    },
    MembershipHearthMismatch {
        expected_hearth: FactId,
        observed_hearth: FactId,
    },
    MembershipStaleOrInactive,
    MembershipUnknown,
    MembershipUnavailable,
    ActorIsNeitherCreatorNorCurrentGuardian,
}

/// Evaluate v1 admission semantics over already-normalized supplied facts.
///
/// Success means only that the facts are structurally compatible with the
/// `CreatorOrCurrentGuardianV1` contract. It does not authenticate those facts.
pub fn evaluate_admission(
    candidate: AdmissionCandidate,
) -> Result<StructuralAdmission, AdmissionError> {
    if candidate.schema_version != ADMISSION_SCHEMA_V1 {
        return Err(AdmissionError::UnsupportedSchemaVersion(
            candidate.schema_version,
        ));
    }

    if candidate.evidence.is_empty() {
        return Err(AdmissionError::NoCompletionEvidence);
    }
    if candidate.evidence.len() > MAX_EVIDENCE_REFS_V1 {
        return Err(AdmissionError::TooManyCompletionEvidence {
            count: candidate.evidence.len(),
            maximum: MAX_EVIDENCE_REFS_V1,
        });
    }

    for pair in candidate.evidence.windows(2) {
        let previous = &pair[0].id;
        let current = &pair[1].id;
        if previous == current {
            return Err(AdmissionError::DuplicateEvidenceReference(current.clone()));
        }
        if previous > current {
            return Err(AdmissionError::EvidenceReferencesNotStrictlySorted {
                previous: previous.clone(),
                current: current.clone(),
            });
        }
    }

    for evidence in &candidate.evidence {
        if evidence.normalized_root_id != candidate.root.root_id {
            return Err(AdmissionError::EvidenceRootMismatch {
                evidence: evidence.id.clone(),
                expected_root: candidate.root.root_id.clone(),
                observed_root: evidence.normalized_root_id.clone(),
            });
        }
        if evidence.hearth_id != candidate.root.hearth_id {
            return Err(AdmissionError::EvidenceHearthMismatch {
                evidence: evidence.id.clone(),
                expected_hearth: candidate.root.hearth_id.clone(),
                observed_hearth: evidence.hearth_id.clone(),
            });
        }
    }

    if candidate.membership.actor_id != candidate.actor_id {
        return Err(AdmissionError::MembershipActorMismatch {
            expected_actor: candidate.actor_id.clone(),
            observed_actor: candidate.membership.actor_id,
        });
    }
    if candidate.membership.hearth_id != candidate.root.hearth_id {
        return Err(AdmissionError::MembershipHearthMismatch {
            expected_hearth: candidate.root.hearth_id.clone(),
            observed_hearth: candidate.membership.hearth_id,
        });
    }

    let is_guardian = match candidate.membership.disposition {
        MembershipDisposition::FreshActive { is_guardian } => is_guardian,
        MembershipDisposition::StaleOrInactive => {
            return Err(AdmissionError::MembershipStaleOrInactive);
        }
        MembershipDisposition::Unknown => return Err(AdmissionError::MembershipUnknown),
        MembershipDisposition::Unavailable => return Err(AdmissionError::MembershipUnavailable),
    };

    let is_creator = candidate.actor_id == candidate.root.creator_actor_id;
    if !is_creator && !is_guardian {
        return Err(AdmissionError::ActorIsNeitherCreatorNorCurrentGuardian);
    }

    Ok(StructuralAdmission {
        schema_version: candidate.schema_version,
        profile: candidate.profile,
        actor_id: candidate.actor_id,
        root_id: candidate.root.root_id,
        hearth_id: candidate.root.hearth_id,
        evidence_ids: candidate
            .evidence
            .into_iter()
            .map(|evidence| evidence.id)
            .collect(),
        membership_proof_id: candidate.membership.proof_id,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: u8) -> FactId {
        FactId::new(vec![value])
    }

    fn evidence(
        evidence_id: u8,
        root_id: u8,
        hearth_id: u8,
        attestor: u8,
        revision_assignee: u8,
    ) -> CompletionEvidenceFact {
        CompletionEvidenceFact {
            id: id(evidence_id),
            normalized_root_id: id(root_id),
            hearth_id: id(hearth_id),
            attestor_actor_id: id(attestor),
            revision_assignee_actor_id: id(revision_assignee),
        }
    }

    fn candidate(
        actor: u8,
        creator: u8,
        is_guardian: bool,
        evidence: Vec<CompletionEvidenceFact>,
    ) -> AdmissionCandidate {
        AdmissionCandidate {
            schema_version: ADMISSION_SCHEMA_V1,
            profile: CareCompletionAdmissionProfileV1::CreatorOrCurrentGuardianV1,
            actor_id: id(actor),
            root: ScheduleRootFact {
                root_id: id(10),
                hearth_id: id(20),
                creator_actor_id: id(creator),
            },
            evidence,
            membership: MembershipFact {
                proof_id: id(30),
                actor_id: id(actor),
                hearth_id: id(20),
                disposition: MembershipDisposition::FreshActive { is_guardian },
            },
        }
    }

    #[test]
    fn original_creator_can_pass_while_active_without_guardian_role() {
        let admitted = evaluate_admission(candidate(
            1,
            1,
            false,
            vec![evidence(40, 10, 20, 9, 7)],
        ))
        .unwrap();

        assert_eq!(admitted.authority(), StructuralAuthority::StructuralOnly);
        assert!(!admitted.authenticated_admission_established());
        assert!(!admitted.universal_completion_established());
    }

    #[test]
    fn current_guardian_can_pass_without_being_original_creator() {
        assert!(evaluate_admission(candidate(
            2,
            1,
            true,
            vec![evidence(40, 10, 20, 7, 8)],
        ))
        .is_ok());
    }

    #[test]
    fn historical_assignee_or_attestor_does_not_grant_admission_authority() {
        let error = evaluate_admission(candidate(
            7,
            1,
            false,
            vec![evidence(40, 10, 20, 7, 7)],
        ))
        .unwrap_err();

        assert_eq!(
            error,
            AdmissionError::ActorIsNeitherCreatorNorCurrentGuardian
        );
    }

    #[test]
    fn stale_membership_fails_closed_even_for_original_creator() {
        let mut subject = candidate(1, 1, false, vec![evidence(40, 10, 20, 1, 1)]);
        subject.membership.disposition = MembershipDisposition::StaleOrInactive;
        assert_eq!(
            evaluate_admission(subject),
            Err(AdmissionError::MembershipStaleOrInactive)
        );
    }

    #[test]
    fn unknown_and_unavailable_membership_fail_closed() {
        for (disposition, expected) in [
            (MembershipDisposition::Unknown, AdmissionError::MembershipUnknown),
            (
                MembershipDisposition::Unavailable,
                AdmissionError::MembershipUnavailable,
            ),
        ] {
            let mut subject = candidate(1, 1, false, vec![evidence(40, 10, 20, 1, 1)]);
            subject.membership.disposition = disposition;
            assert_eq!(evaluate_admission(subject), Err(expected));
        }
    }

    #[test]
    fn evidence_set_must_be_strictly_sorted_and_unique() {
        let duplicate = candidate(
            1,
            1,
            false,
            vec![evidence(40, 10, 20, 1, 1), evidence(40, 10, 20, 1, 1)],
        );
        assert_eq!(
            evaluate_admission(duplicate),
            Err(AdmissionError::DuplicateEvidenceReference(id(40)))
        );

        let unsorted = candidate(
            1,
            1,
            false,
            vec![evidence(41, 10, 20, 1, 1), evidence(40, 10, 20, 1, 1)],
        );
        assert_eq!(
            evaluate_admission(unsorted),
            Err(AdmissionError::EvidenceReferencesNotStrictlySorted {
                previous: id(41),
                current: id(40),
            })
        );
    }

    #[test]
    fn evidence_root_and_hearth_must_match_admission_subject() {
        let wrong_root = candidate(1, 1, false, vec![evidence(40, 11, 20, 1, 1)]);
        assert!(matches!(
            evaluate_admission(wrong_root),
            Err(AdmissionError::EvidenceRootMismatch { .. })
        ));

        let wrong_hearth = candidate(1, 1, false, vec![evidence(40, 10, 21, 1, 1)]);
        assert!(matches!(
            evaluate_admission(wrong_hearth),
            Err(AdmissionError::EvidenceHearthMismatch { .. })
        ));
    }

    #[test]
    fn membership_actor_and_hearth_must_match_subject() {
        let mut wrong_actor = candidate(1, 1, false, vec![evidence(40, 10, 20, 1, 1)]);
        wrong_actor.membership.actor_id = id(2);
        assert!(matches!(
            evaluate_admission(wrong_actor),
            Err(AdmissionError::MembershipActorMismatch { .. })
        ));

        let mut wrong_hearth = candidate(1, 1, false, vec![evidence(40, 10, 20, 1, 1)]);
        wrong_hearth.membership.hearth_id = id(21);
        assert!(matches!(
            evaluate_admission(wrong_hearth),
            Err(AdmissionError::MembershipHearthMismatch { .. })
        ));
    }

    #[test]
    fn empty_and_oversized_evidence_sets_are_rejected() {
        assert_eq!(
            evaluate_admission(candidate(1, 1, false, vec![])),
            Err(AdmissionError::NoCompletionEvidence)
        );

        let too_many = (0..=MAX_EVIDENCE_REFS_V1)
            .map(|index| evidence((index + 1) as u8, 10, 20, 1, 1))
            .collect();
        assert_eq!(
            evaluate_admission(candidate(1, 1, false, too_many)),
            Err(AdmissionError::TooManyCompletionEvidence {
                count: MAX_EVIDENCE_REFS_V1 + 1,
                maximum: MAX_EVIDENCE_REFS_V1,
            })
        );
    }

    #[test]
    fn unsupported_schema_is_rejected() {
        let mut subject = candidate(1, 1, false, vec![evidence(40, 10, 20, 1, 1)]);
        subject.schema_version = 2;
        assert_eq!(
            evaluate_admission(subject),
            Err(AdmissionError::UnsupportedSchemaVersion(2))
        );
    }

    #[test]
    fn equivalent_admissions_share_one_lifecycle_completion_key() {
        let creator_admission = evaluate_admission(candidate(
            1,
            1,
            false,
            vec![evidence(40, 10, 20, 8, 8)],
        ))
        .unwrap();
        let guardian_admission = evaluate_admission(candidate(
            2,
            1,
            true,
            vec![evidence(41, 10, 20, 9, 9)],
        ))
        .unwrap();

        assert_ne!(creator_admission.actor_id, guardian_admission.actor_id);
        assert_ne!(creator_admission.evidence_ids, guardian_admission.evidence_ids);
        assert_eq!(
            creator_admission.lifecycle_completion_key(),
            guardian_admission.lifecycle_completion_key()
        );
    }
}
