// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Execution-phase profiling for STEW-010 reciprocity obligations.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_core::CanonicalIdV1;
use mycelix_stewardship_reciprocity::{ReciprocityBenefitV1, ReciprocityObligationV1};

pub const RECIPROCITY_DUTY_PHASE_PROFILE_V1: &str = "mycelix/reciprocity-duty-phase/v1";

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ReciprocityDutyExecutionPhaseV1 {
    Precondition,
    Concurrent,
    PostUse,
    Ongoing,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ReciprocityDutyPreUseHandlingV1 {
    MustBeSatisfiedBeforeUse,
    MustBeBoundAtUse,
    MustBeActivatedForTracking,
    MustBeBoundAndTracked,
}

impl ReciprocityDutyExecutionPhaseV1 {
    pub const fn pre_use_handling(self) -> ReciprocityDutyPreUseHandlingV1 {
        match self {
            Self::Precondition => ReciprocityDutyPreUseHandlingV1::MustBeSatisfiedBeforeUse,
            Self::Concurrent => ReciprocityDutyPreUseHandlingV1::MustBeBoundAtUse,
            Self::PostUse => ReciprocityDutyPreUseHandlingV1::MustBeActivatedForTracking,
            Self::Ongoing => ReciprocityDutyPreUseHandlingV1::MustBeBoundAndTracked,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReciprocityDutyAssignmentV1 {
    benefit: ReciprocityBenefitV1,
    phase: ReciprocityDutyExecutionPhaseV1,
    phase_basis_ref: CanonicalIdV1,
}

impl ReciprocityDutyAssignmentV1 {
    pub const fn new(
        benefit: ReciprocityBenefitV1,
        phase: ReciprocityDutyExecutionPhaseV1,
        phase_basis_ref: CanonicalIdV1,
    ) -> Self {
        Self {
            benefit,
            phase,
            phase_basis_ref,
        }
    }

    pub fn benefit(&self) -> &ReciprocityBenefitV1 {
        &self.benefit
    }

    pub const fn phase(&self) -> ReciprocityDutyExecutionPhaseV1 {
        self.phase
    }

    pub fn phase_basis_ref(&self) -> &CanonicalIdV1 {
        &self.phase_basis_ref
    }

    pub const fn pre_use_handling(&self) -> ReciprocityDutyPreUseHandlingV1 {
        self.phase.pre_use_handling()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReciprocityDutyProfileErrorV1 {
    AssignmentCountMismatch,
    BenefitNotInObligation,
    DuplicateBenefitAssignment,
}

impl fmt::Display for ReciprocityDutyProfileErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AssignmentCountMismatch => {
                f.write_str("duty profile must assign every reciprocity benefit exactly once")
            }
            Self::BenefitNotInObligation => {
                f.write_str("duty assignment references a benefit not present in the obligation")
            }
            Self::DuplicateBenefitAssignment => {
                f.write_str("reciprocity benefit has more than one duty-phase assignment")
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReciprocityDutyProfileV1 {
    profile_id: CanonicalIdV1,
    obligation: ReciprocityObligationV1,
    assignments: Vec<ReciprocityDutyAssignmentV1>,
}

impl ReciprocityDutyProfileV1 {
    pub fn new(
        profile_id: CanonicalIdV1,
        obligation: ReciprocityObligationV1,
        assignments: Vec<ReciprocityDutyAssignmentV1>,
    ) -> Result<Self, ReciprocityDutyProfileErrorV1> {
        if assignments.len() != obligation.benefits().len() {
            return Err(ReciprocityDutyProfileErrorV1::AssignmentCountMismatch);
        }

        for (index, assignment) in assignments.iter().enumerate() {
            if !obligation.benefits().contains(assignment.benefit()) {
                return Err(ReciprocityDutyProfileErrorV1::BenefitNotInObligation);
            }
            if assignments[..index]
                .iter()
                .any(|previous| previous.benefit() == assignment.benefit())
            {
                return Err(ReciprocityDutyProfileErrorV1::DuplicateBenefitAssignment);
            }
        }

        Ok(Self {
            profile_id,
            obligation,
            assignments,
        })
    }

    pub fn profile_id(&self) -> &CanonicalIdV1 {
        &self.profile_id
    }

    pub fn obligation(&self) -> &ReciprocityObligationV1 {
        &self.obligation
    }

    pub fn assignments(&self) -> &[ReciprocityDutyAssignmentV1] {
        &self.assignments
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_stewardship_core::{
        ContentDigestV1, DigestAlgorithmV1, RepresentationIdV1, RepresentationKindV1,
        RevisionIdV1, StewardedSubjectIdV1, StewardedSubjectIdentityV1,
    };
    use mycelix_stewardship_reciprocity::{
        ReciprocityBenefitKindV1, ReciprocityObligationIdV1,
    };

    fn id(value: &str) -> CanonicalIdV1 {
        CanonicalIdV1::new(value).unwrap()
    }

    fn target() -> StewardedSubjectIdentityV1 {
        StewardedSubjectIdentityV1 {
            subject: StewardedSubjectIdV1::new("subject:reciprocity:1").unwrap(),
            revision: RevisionIdV1::new("revision:1").unwrap(),
            representation: RepresentationIdV1::new("representation:dataset:1").unwrap(),
            kind: RepresentationKindV1::Dataset,
            content_digest: ContentDigestV1::new(DigestAlgorithmV1::Blake3_256, [7; 32]),
        }
    }

    fn benefit(kind: ReciprocityBenefitKindV1, requirement: &str) -> ReciprocityBenefitV1 {
        ReciprocityBenefitV1::new(kind, id(requirement))
    }

    fn obligation() -> ReciprocityObligationV1 {
        ReciprocityObligationV1::new(
            ReciprocityObligationIdV1::new("reciprocity:obligation:1").unwrap(),
            target(),
            id("assertor:1"),
            id("beneficiary:1"),
            id("trigger:1"),
            vec![
                benefit(ReciprocityBenefitKindV1::Attribution, "requirement:attribution:1"),
                benefit(
                    ReciprocityBenefitKindV1::ReturnDataOrResults,
                    "requirement:return-results:1",
                ),
            ],
            vec![id("basis:1")],
        )
        .unwrap()
    }

    fn assignments(obligation: &ReciprocityObligationV1) -> Vec<ReciprocityDutyAssignmentV1> {
        vec![
            ReciprocityDutyAssignmentV1::new(
                obligation.benefits()[0].clone(),
                ReciprocityDutyExecutionPhaseV1::Concurrent,
                id("phase-basis:attribution:1"),
            ),
            ReciprocityDutyAssignmentV1::new(
                obligation.benefits()[1].clone(),
                ReciprocityDutyExecutionPhaseV1::PostUse,
                id("phase-basis:return-results:1"),
            ),
        ]
    }

    #[test]
    fn exact_one_to_one_benefit_coverage_is_accepted() {
        let obligation = obligation();
        let profile = ReciprocityDutyProfileV1::new(
            id("duty-profile:1"),
            obligation.clone(),
            assignments(&obligation),
        )
        .unwrap();
        assert_eq!(profile.assignments().len(), obligation.benefits().len());
    }

    #[test]
    fn missing_benefit_assignment_is_rejected() {
        let obligation = obligation();
        let only_one = vec![ReciprocityDutyAssignmentV1::new(
            obligation.benefits()[0].clone(),
            ReciprocityDutyExecutionPhaseV1::Concurrent,
            id("phase-basis:1"),
        )];
        assert_eq!(
            ReciprocityDutyProfileV1::new(id("duty-profile:missing"), obligation, only_one),
            Err(ReciprocityDutyProfileErrorV1::AssignmentCountMismatch)
        );
    }

    #[test]
    fn invented_benefit_assignment_is_rejected() {
        let obligation = obligation();
        let invented = benefit(ReciprocityBenefitKindV1::Compensation, "requirement:invented:1");
        let assignments = vec![
            ReciprocityDutyAssignmentV1::new(
                obligation.benefits()[0].clone(),
                ReciprocityDutyExecutionPhaseV1::Concurrent,
                id("phase-basis:1"),
            ),
            ReciprocityDutyAssignmentV1::new(
                invented,
                ReciprocityDutyExecutionPhaseV1::Precondition,
                id("phase-basis:2"),
            ),
        ];
        assert_eq!(
            ReciprocityDutyProfileV1::new(id("duty-profile:invented"), obligation, assignments),
            Err(ReciprocityDutyProfileErrorV1::BenefitNotInObligation)
        );
    }

    #[test]
    fn duplicate_benefit_assignment_is_rejected() {
        let obligation = obligation();
        let duplicate = obligation.benefits()[0].clone();
        let assignments = vec![
            ReciprocityDutyAssignmentV1::new(
                duplicate.clone(),
                ReciprocityDutyExecutionPhaseV1::Concurrent,
                id("phase-basis:1"),
            ),
            ReciprocityDutyAssignmentV1::new(
                duplicate,
                ReciprocityDutyExecutionPhaseV1::PostUse,
                id("phase-basis:2"),
            ),
        ];
        assert_eq!(
            ReciprocityDutyProfileV1::new(id("duty-profile:duplicate"), obligation, assignments),
            Err(ReciprocityDutyProfileErrorV1::DuplicateBenefitAssignment)
        );
    }

    #[test]
    fn each_phase_has_distinct_pre_use_handling() {
        assert_eq!(
            ReciprocityDutyExecutionPhaseV1::Precondition.pre_use_handling(),
            ReciprocityDutyPreUseHandlingV1::MustBeSatisfiedBeforeUse
        );
        assert_eq!(
            ReciprocityDutyExecutionPhaseV1::Concurrent.pre_use_handling(),
            ReciprocityDutyPreUseHandlingV1::MustBeBoundAtUse
        );
        assert_eq!(
            ReciprocityDutyExecutionPhaseV1::PostUse.pre_use_handling(),
            ReciprocityDutyPreUseHandlingV1::MustBeActivatedForTracking
        );
        assert_eq!(
            ReciprocityDutyExecutionPhaseV1::Ongoing.pre_use_handling(),
            ReciprocityDutyPreUseHandlingV1::MustBeBoundAndTracked
        );
    }

    #[test]
    fn phase_basis_is_carried_but_not_verified() {
        let obligation = obligation();
        let profile = ReciprocityDutyProfileV1::new(
            id("duty-profile:1"),
            obligation.clone(),
            assignments(&obligation),
        )
        .unwrap();
        assert_eq!(
            profile.assignments()[0].phase_basis_ref().as_str(),
            "phase-basis:attribution:1"
        );
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(
            RECIPROCITY_DUTY_PHASE_PROFILE_V1,
            "mycelix/reciprocity-duty-phase/v1"
        );
    }
}
