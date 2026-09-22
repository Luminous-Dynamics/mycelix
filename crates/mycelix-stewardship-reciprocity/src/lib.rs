// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Authority-neutral reciprocity obligations for exact stewarded representations.
//!
//! STEW-010 represents asserted obligations. It does not decide whether the
//! asserting party has authority, whether a trigger occurred, whether a benefit
//! was delivered, or whether an obligation is satisfied.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_core::{CanonicalIdErrorV1, CanonicalIdV1, StewardedSubjectIdentityV1};

/// Stable profile identifier for this theorem.
pub const RECIPROCITY_PROFILE_V1: &str = "mycelix/reciprocity-obligation/v1";

/// Maximum number of benefit clauses on one v1 obligation.
pub const MAX_RECIPROCITY_BENEFITS_V1: usize = 32;
/// Maximum number of basis references supporting one v1 obligation.
pub const MAX_RECIPROCITY_BASIS_REFS_V1: usize = 32;

macro_rules! typed_ref {
    ($name:ident, $doc:literal) => {
        #[doc = $doc]
        #[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(CanonicalIdV1);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
                CanonicalIdV1::new(value).map(Self)
            }

            pub fn as_str(&self) -> &str {
                self.0.as_str()
            }
        }
    };
}

typed_ref!(
    ReciprocityObligationIdV1,
    "Typed identifier for one asserted reciprocity obligation."
);
typed_ref!(
    ReciprocityAssertorRefV1,
    "Opaque reference to the party asserted to have stated the obligation; not proof of authority."
);
typed_ref!(
    ReciprocityBeneficiaryRefV1,
    "Opaque reference to the asserted beneficiary; not proof of identity or acceptance authority."
);
typed_ref!(
    ReciprocityTriggerRefV1,
    "Opaque reference to the asserted trigger rule/condition; not proof that the trigger occurred."
);
typed_ref!(
    ReciprocityRequirementRefV1,
    "Opaque reference to the detailed terms of one benefit clause."
);
typed_ref!(
    ReciprocityBasisRefV1,
    "Opaque reference to evidence or protocol material asserted as the obligation basis."
);

/// Broad, non-monetary-first categories of reciprocal benefit.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ReciprocityBenefitKindV1 {
    Attribution,
    Compensation,
    RevenueShare,
    ReturnDataOrResults,
    ReturnCopy,
    CapacityBuilding,
    InfrastructureSupport,
    CommunityInvestment,
    AccessBenefit,
    EnvironmentalStewardship,
    KnowledgeReturn,
    Other,
}

/// One asserted benefit clause.
///
/// `requirement_ref` points to the detailed terms. This theorem intentionally
/// does not interpret currency, percentages, schedules, legal language, or
/// community protocol semantics.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct ReciprocityBenefitV1 {
    pub kind: ReciprocityBenefitKindV1,
    pub requirement_ref: ReciprocityRequirementRefV1,
}

impl ReciprocityBenefitV1 {
    pub const fn new(
        kind: ReciprocityBenefitKindV1,
        requirement_ref: ReciprocityRequirementRefV1,
    ) -> Self {
        Self {
            kind,
            requirement_ref,
        }
    }
}

/// Structural construction failures for v1 obligations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReciprocityObligationErrorV1 {
    NoBenefits,
    TooManyBenefits,
    DuplicateBenefit,
    NoBasisReferences,
    TooManyBasisReferences,
    DuplicateBasisReference,
}

impl fmt::Display for ReciprocityObligationErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoBenefits => f.write_str("reciprocity obligation requires at least one benefit"),
            Self::TooManyBenefits => f.write_str("too many reciprocity benefits for v1"),
            Self::DuplicateBenefit => f.write_str("duplicate reciprocity benefit"),
            Self::NoBasisReferences => {
                f.write_str("reciprocity obligation requires at least one basis reference")
            }
            Self::TooManyBasisReferences => {
                f.write_str("too many reciprocity basis references for v1")
            }
            Self::DuplicateBasisReference => f.write_str("duplicate reciprocity basis reference"),
        }
    }
}

/// Evidence-bearing asserted reciprocity obligation.
///
/// Core separation:
///
/// ```text
/// obligation recorded
/// != asserting party authoritative
/// != beneficiary identity verified
/// != trigger satisfied
/// != benefit delivered
/// != obligation satisfied
/// ```
///
/// Assertor, beneficiary, trigger, requirement and basis references use distinct
/// Rust types so accidental semantic-role substitution is a compile-time error.
/// The typed wrappers themselves establish no truth or authority.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReciprocityObligationV1 {
    obligation_id: ReciprocityObligationIdV1,
    target: StewardedSubjectIdentityV1,
    asserted_by_ref: ReciprocityAssertorRefV1,
    beneficiary_ref: ReciprocityBeneficiaryRefV1,
    trigger_ref: ReciprocityTriggerRefV1,
    benefits: Vec<ReciprocityBenefitV1>,
    basis_refs: Vec<ReciprocityBasisRefV1>,
}

impl ReciprocityObligationV1 {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        obligation_id: ReciprocityObligationIdV1,
        target: StewardedSubjectIdentityV1,
        asserted_by_ref: ReciprocityAssertorRefV1,
        beneficiary_ref: ReciprocityBeneficiaryRefV1,
        trigger_ref: ReciprocityTriggerRefV1,
        benefits: Vec<ReciprocityBenefitV1>,
        basis_refs: Vec<ReciprocityBasisRefV1>,
    ) -> Result<Self, ReciprocityObligationErrorV1> {
        validate_benefits(&benefits)?;
        validate_basis_refs(&basis_refs)?;

        Ok(Self {
            obligation_id,
            target,
            asserted_by_ref,
            beneficiary_ref,
            trigger_ref,
            benefits,
            basis_refs,
        })
    }

    pub fn obligation_id(&self) -> &ReciprocityObligationIdV1 {
        &self.obligation_id
    }

    pub fn target(&self) -> &StewardedSubjectIdentityV1 {
        &self.target
    }

    pub fn asserted_by_ref(&self) -> &ReciprocityAssertorRefV1 {
        &self.asserted_by_ref
    }

    pub fn beneficiary_ref(&self) -> &ReciprocityBeneficiaryRefV1 {
        &self.beneficiary_ref
    }

    pub fn trigger_ref(&self) -> &ReciprocityTriggerRefV1 {
        &self.trigger_ref
    }

    pub fn benefits(&self) -> &[ReciprocityBenefitV1] {
        &self.benefits
    }

    pub fn basis_refs(&self) -> &[ReciprocityBasisRefV1] {
        &self.basis_refs
    }
}

fn validate_benefits(
    benefits: &[ReciprocityBenefitV1],
) -> Result<(), ReciprocityObligationErrorV1> {
    if benefits.is_empty() {
        return Err(ReciprocityObligationErrorV1::NoBenefits);
    }
    if benefits.len() > MAX_RECIPROCITY_BENEFITS_V1 {
        return Err(ReciprocityObligationErrorV1::TooManyBenefits);
    }
    for (index, benefit) in benefits.iter().enumerate() {
        if benefits[..index].contains(benefit) {
            return Err(ReciprocityObligationErrorV1::DuplicateBenefit);
        }
    }
    Ok(())
}

fn validate_basis_refs(refs: &[ReciprocityBasisRefV1]) -> Result<(), ReciprocityObligationErrorV1> {
    if refs.is_empty() {
        return Err(ReciprocityObligationErrorV1::NoBasisReferences);
    }
    if refs.len() > MAX_RECIPROCITY_BASIS_REFS_V1 {
        return Err(ReciprocityObligationErrorV1::TooManyBasisReferences);
    }
    for (index, reference) in refs.iter().enumerate() {
        if refs[..index].contains(reference) {
            return Err(ReciprocityObligationErrorV1::DuplicateBasisReference);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_stewardship_core::{
        ContentDigestV1, DigestAlgorithmV1, RepresentationIdV1, RepresentationKindV1, RevisionIdV1,
        StewardedSubjectIdV1,
    };

    fn target(rep: &str, byte: u8) -> StewardedSubjectIdentityV1 {
        StewardedSubjectIdentityV1 {
            subject: StewardedSubjectIdV1::new("subject:reciprocity:example").unwrap(),
            revision: RevisionIdV1::new("revision:1").unwrap(),
            representation: RepresentationIdV1::new(rep).unwrap(),
            kind: RepresentationKindV1::Dataset,
            content_digest: ContentDigestV1::new(DigestAlgorithmV1::Blake3_256, [byte; 32]),
        }
    }

    fn benefit(kind: ReciprocityBenefitKindV1, requirement: &str) -> ReciprocityBenefitV1 {
        ReciprocityBenefitV1::new(kind, ReciprocityRequirementRefV1::new(requirement).unwrap())
    }

    fn obligation() -> ReciprocityObligationV1 {
        ReciprocityObligationV1::new(
            ReciprocityObligationIdV1::new("reciprocity:obligation:1").unwrap(),
            target("representation:dataset", 1),
            ReciprocityAssertorRefV1::new("assertor:community-council-claim:1").unwrap(),
            ReciprocityBeneficiaryRefV1::new("beneficiary:community:1").unwrap(),
            ReciprocityTriggerRefV1::new("trigger:use-policy-rule:1").unwrap(),
            vec![
                benefit(
                    ReciprocityBenefitKindV1::Attribution,
                    "requirement:attribution:1",
                ),
                benefit(
                    ReciprocityBenefitKindV1::ReturnDataOrResults,
                    "requirement:return-results:1",
                ),
            ],
            vec![ReciprocityBasisRefV1::new("basis:community-protocol:1").unwrap()],
        )
        .unwrap()
    }

    #[test]
    fn obligation_keeps_assertor_beneficiary_and_trigger_separate() {
        let obligation = obligation();
        assert_eq!(
            obligation.asserted_by_ref().as_str(),
            "assertor:community-council-claim:1"
        );
        assert_eq!(
            obligation.beneficiary_ref().as_str(),
            "beneficiary:community:1"
        );
        assert_eq!(
            obligation.trigger_ref().as_str(),
            "trigger:use-policy-rule:1"
        );
    }

    #[test]
    fn typed_roles_remain_assertions_not_verified_facts() {
        let obligation = obligation();
        assert_eq!(
            obligation.asserted_by_ref().as_str(),
            "assertor:community-council-claim:1"
        );
        assert_eq!(obligation.basis_refs().len(), 1);
        // Typed references prevent accidental role substitution only; the value
        // still carries no authority, identity proof, trigger truth or discharge.
    }

    #[test]
    fn monetary_and_nonmonetary_benefits_coexist_without_collapsing() {
        let obligation = ReciprocityObligationV1::new(
            ReciprocityObligationIdV1::new("reciprocity:obligation:2").unwrap(),
            target("representation:dataset", 2),
            ReciprocityAssertorRefV1::new("assertor:1").unwrap(),
            ReciprocityBeneficiaryRefV1::new("beneficiary:1").unwrap(),
            ReciprocityTriggerRefV1::new("trigger:1").unwrap(),
            vec![
                benefit(
                    ReciprocityBenefitKindV1::Compensation,
                    "requirement:payment:1",
                ),
                benefit(
                    ReciprocityBenefitKindV1::CapacityBuilding,
                    "requirement:training:1",
                ),
            ],
            vec![ReciprocityBasisRefV1::new("basis:1").unwrap()],
        )
        .unwrap();

        assert_eq!(obligation.benefits().len(), 2);
        assert_ne!(obligation.benefits()[0].kind, obligation.benefits()[1].kind);
    }

    #[test]
    fn empty_benefit_set_is_rejected() {
        let result = ReciprocityObligationV1::new(
            ReciprocityObligationIdV1::new("reciprocity:obligation:3").unwrap(),
            target("representation:dataset", 3),
            ReciprocityAssertorRefV1::new("assertor:1").unwrap(),
            ReciprocityBeneficiaryRefV1::new("beneficiary:1").unwrap(),
            ReciprocityTriggerRefV1::new("trigger:1").unwrap(),
            vec![],
            vec![ReciprocityBasisRefV1::new("basis:1").unwrap()],
        );
        assert_eq!(result, Err(ReciprocityObligationErrorV1::NoBenefits));
    }

    #[test]
    fn duplicate_benefit_is_rejected() {
        let clause = benefit(ReciprocityBenefitKindV1::Attribution, "requirement:1");
        let result = ReciprocityObligationV1::new(
            ReciprocityObligationIdV1::new("reciprocity:obligation:4").unwrap(),
            target("representation:dataset", 4),
            ReciprocityAssertorRefV1::new("assertor:1").unwrap(),
            ReciprocityBeneficiaryRefV1::new("beneficiary:1").unwrap(),
            ReciprocityTriggerRefV1::new("trigger:1").unwrap(),
            vec![clause.clone(), clause],
            vec![ReciprocityBasisRefV1::new("basis:1").unwrap()],
        );
        assert_eq!(result, Err(ReciprocityObligationErrorV1::DuplicateBenefit));
    }

    #[test]
    fn basis_is_required_but_not_treated_as_verified_authority() {
        let result = ReciprocityObligationV1::new(
            ReciprocityObligationIdV1::new("reciprocity:obligation:5").unwrap(),
            target("representation:dataset", 5),
            ReciprocityAssertorRefV1::new("assertor:1").unwrap(),
            ReciprocityBeneficiaryRefV1::new("beneficiary:1").unwrap(),
            ReciprocityTriggerRefV1::new("trigger:1").unwrap(),
            vec![benefit(
                ReciprocityBenefitKindV1::KnowledgeReturn,
                "requirement:1",
            )],
            vec![],
        );
        assert_eq!(result, Err(ReciprocityObligationErrorV1::NoBasisReferences));
    }

    #[test]
    fn duplicate_basis_reference_is_rejected() {
        let basis = ReciprocityBasisRefV1::new("basis:1").unwrap();
        let result = ReciprocityObligationV1::new(
            ReciprocityObligationIdV1::new("reciprocity:obligation:6").unwrap(),
            target("representation:dataset", 6),
            ReciprocityAssertorRefV1::new("assertor:1").unwrap(),
            ReciprocityBeneficiaryRefV1::new("beneficiary:1").unwrap(),
            ReciprocityTriggerRefV1::new("trigger:1").unwrap(),
            vec![benefit(
                ReciprocityBenefitKindV1::KnowledgeReturn,
                "requirement:1",
            )],
            vec![basis.clone(), basis],
        );
        assert_eq!(
            result,
            Err(ReciprocityObligationErrorV1::DuplicateBasisReference)
        );
    }

    #[test]
    fn exact_representation_target_prevents_silent_lineage_inheritance() {
        let a = obligation();
        let b_target = target("representation:translated", 9);
        assert_ne!(a.target(), &b_target);
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(RECIPROCITY_PROFILE_V1, "mycelix/reciprocity-obligation/v1");
    }
}
