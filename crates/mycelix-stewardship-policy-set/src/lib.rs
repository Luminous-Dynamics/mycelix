// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Cross-policy structural assessment for policy-authority candidates.
//!
//! STEW-012D describes agreement/divergence among STEW-012B candidates without
//! converting candidate classifications into verified authority, authorization,
//! or final denial.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_core::{CanonicalIdV1, StewardedSubjectIdentityV1};
use mycelix_stewardship_policy::{
    ActionDispositionV1, ConstraintRefV1, DutyRefV1, KnowledgeUseActionV1,
};
use mycelix_stewardship_policy_authority::PolicyAuthorityBindingCandidateV1;

pub const POLICY_SET_ASSESSMENT_PROFILE_V1: &str = "mycelix/policy-set-assessment/v1";
pub const MAX_POLICY_AUTHORITY_CANDIDATES_V1: usize = 64;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CandidateActionDispositionV1 {
    PermissionCandidate {
        constraints: Vec<ConstraintRefV1>,
        duties: Vec<DutyRefV1>,
    },
    ProhibitedCandidate,
    DeniedUnspecifiedCandidate,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PolicyCandidateAssessmentV1 {
    binding_id: CanonicalIdV1,
    policy_id: CanonicalIdV1,
    disposition: CandidateActionDispositionV1,
}

impl PolicyCandidateAssessmentV1 {
    pub fn binding_id(&self) -> &CanonicalIdV1 {
        &self.binding_id
    }

    pub fn policy_id(&self) -> &CanonicalIdV1 {
        &self.policy_id
    }

    pub fn disposition(&self) -> &CandidateActionDispositionV1 {
        &self.disposition
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrossPolicyCandidateStateV1 {
    NoCandidates,
    UniformPermissionCandidates,
    DivergentPermissionCandidates,
    UniformProhibitionCandidates,
    UniformUnspecifiedDenialCandidates,
    MixedNonPermissionCandidates,
    MixedCandidateDispositions,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolicySetAssessmentErrorV1 {
    TooManyCandidates,
    DuplicateBindingId,
    CandidateTargetMismatch,
}

impl fmt::Display for PolicySetAssessmentErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooManyCandidates => f.write_str("too many policy-authority candidates for v1"),
            Self::DuplicateBindingId => f.write_str("duplicate policy-authority binding id"),
            Self::CandidateTargetMismatch => {
                f.write_str("policy-authority candidate target does not match requested exact target")
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PolicySetAssessmentV1 {
    target: StewardedSubjectIdentityV1,
    action: KnowledgeUseActionV1,
    candidates: Vec<PolicyCandidateAssessmentV1>,
    state: CrossPolicyCandidateStateV1,
}

impl PolicySetAssessmentV1 {
    pub fn target(&self) -> &StewardedSubjectIdentityV1 {
        &self.target
    }

    pub const fn action(&self) -> KnowledgeUseActionV1 {
        self.action
    }

    pub fn candidates(&self) -> &[PolicyCandidateAssessmentV1] {
        &self.candidates
    }

    pub const fn state(&self) -> CrossPolicyCandidateStateV1 {
        self.state
    }
}

fn classify_candidate(
    candidate: &PolicyAuthorityBindingCandidateV1,
    action: KnowledgeUseActionV1,
) -> CandidateActionDispositionV1 {
    match candidate.policy().classify_action(action) {
        ActionDispositionV1::Prohibited => CandidateActionDispositionV1::ProhibitedCandidate,
        ActionDispositionV1::PermissionCandidate {
            constraints,
            duties,
        } => CandidateActionDispositionV1::PermissionCandidate {
            constraints: constraints.to_vec(),
            duties: duties.to_vec(),
        },
        ActionDispositionV1::DeniedUnspecified => {
            CandidateActionDispositionV1::DeniedUnspecifiedCandidate
        }
    }
}

fn permission_signature(
    disposition: &CandidateActionDispositionV1,
) -> Option<(Vec<ConstraintRefV1>, Vec<DutyRefV1>)> {
    match disposition {
        CandidateActionDispositionV1::PermissionCandidate {
            constraints,
            duties,
        } => {
            let mut constraints = constraints.clone();
            let mut duties = duties.clone();
            constraints.sort();
            duties.sort();
            Some((constraints, duties))
        }
        CandidateActionDispositionV1::ProhibitedCandidate
        | CandidateActionDispositionV1::DeniedUnspecifiedCandidate => None,
    }
}

fn aggregate_state(candidates: &[PolicyCandidateAssessmentV1]) -> CrossPolicyCandidateStateV1 {
    if candidates.is_empty() {
        return CrossPolicyCandidateStateV1::NoCandidates;
    }

    let permission_count = candidates
        .iter()
        .filter(|candidate| {
            matches!(
                &candidate.disposition,
                CandidateActionDispositionV1::PermissionCandidate { .. }
            )
        })
        .count();
    let prohibition_count = candidates
        .iter()
        .filter(|candidate| {
            matches!(
                &candidate.disposition,
                CandidateActionDispositionV1::ProhibitedCandidate
            )
        })
        .count();
    let unspecified_count = candidates.len() - permission_count - prohibition_count;

    if permission_count == candidates.len() {
        let first = permission_signature(&candidates[0].disposition)
            .expect("permission-only branch must have a permission signature");
        if candidates[1..]
            .iter()
            .all(|candidate| permission_signature(&candidate.disposition) == Some(first.clone()))
        {
            CrossPolicyCandidateStateV1::UniformPermissionCandidates
        } else {
            CrossPolicyCandidateStateV1::DivergentPermissionCandidates
        }
    } else if prohibition_count == candidates.len() {
        CrossPolicyCandidateStateV1::UniformProhibitionCandidates
    } else if unspecified_count == candidates.len() {
        CrossPolicyCandidateStateV1::UniformUnspecifiedDenialCandidates
    } else if permission_count == 0 {
        CrossPolicyCandidateStateV1::MixedNonPermissionCandidates
    } else {
        CrossPolicyCandidateStateV1::MixedCandidateDispositions
    }
}

pub fn assess_policy_candidates(
    target: &StewardedSubjectIdentityV1,
    action: KnowledgeUseActionV1,
    candidates: &[PolicyAuthorityBindingCandidateV1],
) -> Result<PolicySetAssessmentV1, PolicySetAssessmentErrorV1> {
    if candidates.len() > MAX_POLICY_AUTHORITY_CANDIDATES_V1 {
        return Err(PolicySetAssessmentErrorV1::TooManyCandidates);
    }

    for (index, candidate) in candidates.iter().enumerate() {
        if candidate.policy().target() != target {
            return Err(PolicySetAssessmentErrorV1::CandidateTargetMismatch);
        }
        if candidates[..index]
            .iter()
            .any(|previous| previous.binding_id() == candidate.binding_id())
        {
            return Err(PolicySetAssessmentErrorV1::DuplicateBindingId);
        }
    }

    let assessments: Vec<_> = candidates
        .iter()
        .map(|candidate| PolicyCandidateAssessmentV1 {
            binding_id: candidate.binding_id().clone(),
            policy_id: candidate.policy().policy_id().clone(),
            disposition: classify_candidate(candidate, action),
        })
        .collect();
    let state = aggregate_state(&assessments);

    Ok(PolicySetAssessmentV1 {
        target: target.clone(),
        action,
        candidates: assessments,
        state,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_stewardship_admission::{
        AdmissionEvidenceRefV1, StewardshipAdmissionDispositionV1, StewardshipAdmissionRecordV1,
    };
    use mycelix_stewardship_claims::{
        ClaimEvidenceRefV1, ClaimedStewardshipDomainV1, StewardshipClaimBasisV1,
        StewardshipClaimTargetV1, StewardshipClaimV1,
    };
    use mycelix_stewardship_core::{
        ContentDigestV1, DigestAlgorithmV1, RepresentationIdV1, RepresentationKindV1,
        RevisionIdV1, StewardedSubjectIdV1,
    };
    use mycelix_stewardship_policy::{KnowledgeUsePolicyV1, KnowledgeUseRuleV1};
    use mycelix_stewardship_policy_authority::{
        AuthorityCurrentnessAssertionV1, AuthorityEvidenceRefV1,
        PolicyAuthorityBindingCandidateV1,
    };

    fn id(value: impl Into<String>) -> CanonicalIdV1 {
        CanonicalIdV1::new(value).unwrap()
    }

    fn target(revision: &str, representation: &str, digest: u8) -> StewardedSubjectIdentityV1 {
        StewardedSubjectIdentityV1 {
            subject: StewardedSubjectIdV1::new("subject:tradition:1").unwrap(),
            revision: RevisionIdV1::new(revision).unwrap(),
            representation: RepresentationIdV1::new(representation).unwrap(),
            kind: RepresentationKindV1::Text,
            content_digest: ContentDigestV1::new(DigestAlgorithmV1::Blake3_256, [digest; 32]),
        }
    }

    fn binding(
        suffix: &str,
        exact: StewardedSubjectIdentityV1,
        rules: Vec<KnowledgeUseRuleV1>,
    ) -> PolicyAuthorityBindingCandidateV1 {
        let policy = KnowledgeUsePolicyV1::new(
            id(format!("policy:{suffix}")),
            exact.clone(),
            rules,
        )
        .unwrap();
        let claim = StewardshipClaimV1::new(
            id(format!("claim:{suffix}")),
            StewardshipClaimTargetV1::ExactRepresentation(exact),
            id("principal:issuer:1"),
            None,
            StewardshipClaimBasisV1::CommunityMandateAssertion,
            vec![ClaimedStewardshipDomainV1::AccessPolicyParticipation],
            vec![ClaimEvidenceRefV1::new(format!("evidence:claim:{suffix}")).unwrap()],
        )
        .unwrap();
        let admission = StewardshipAdmissionRecordV1::new(
            id(format!("admission:{suffix}")),
            claim,
            ClaimedStewardshipDomainV1::AccessPolicyParticipation,
            id("admission-profile:1"),
            id("principal:decider:1"),
            StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
            vec![AdmissionEvidenceRefV1::new(format!("evidence:admission:{suffix}")).unwrap()],
        )
        .unwrap();
        PolicyAuthorityBindingCandidateV1::new(
            id(format!("binding:{suffix}")),
            policy,
            admission,
            id("principal:issuer:1"),
            id("mandate-scope:1"),
            id("authority-profile:1"),
            AuthorityCurrentnessAssertionV1::AssertedCurrent,
            vec![AuthorityEvidenceRefV1::new(format!("evidence:mandate:{suffix}")).unwrap()],
            vec![AuthorityEvidenceRefV1::new(format!("evidence:current:{suffix}")).unwrap()],
            vec![AuthorityEvidenceRefV1::new(format!("evidence:binding:{suffix}")).unwrap()],
        )
        .unwrap()
    }

    fn permit(
        action: KnowledgeUseActionV1,
        constraints: &[&str],
        duties: &[&str],
    ) -> KnowledgeUseRuleV1 {
        KnowledgeUseRuleV1::permit(
            action,
            constraints
                .iter()
                .map(|value| ConstraintRefV1::new(*value).unwrap())
                .collect(),
            duties
                .iter()
                .map(|value| DutyRefV1::new(*value).unwrap())
                .collect(),
        )
        .unwrap()
    }

    #[test]
    fn empty_candidate_set_is_explicit() {
        let exact = target("revision:1", "representation:1", 1);
        let assessment = assess_policy_candidates(&exact, KnowledgeUseActionV1::View, &[]).unwrap();
        assert_eq!(assessment.state(), CrossPolicyCandidateStateV1::NoCandidates);
    }

    #[test]
    fn identical_permissions_are_uniform_even_if_reference_order_differs() {
        let exact = target("revision:1", "representation:1", 1);
        let a = binding(
            "a",
            exact.clone(),
            vec![permit(
                KnowledgeUseActionV1::View,
                &["constraint:a", "constraint:b"],
                &["duty:a", "duty:b"],
            )],
        );
        let b = binding(
            "b",
            exact.clone(),
            vec![permit(
                KnowledgeUseActionV1::View,
                &["constraint:b", "constraint:a"],
                &["duty:b", "duty:a"],
            )],
        );
        let assessment = assess_policy_candidates(&exact, KnowledgeUseActionV1::View, &[a, b]).unwrap();
        assert_eq!(
            assessment.state(),
            CrossPolicyCandidateStateV1::UniformPermissionCandidates
        );
    }

    #[test]
    fn different_permission_requirements_remain_divergent() {
        let exact = target("revision:1", "representation:1", 1);
        let a = binding(
            "a",
            exact.clone(),
            vec![permit(KnowledgeUseActionV1::View, &["constraint:a"], &[])],
        );
        let b = binding(
            "b",
            exact.clone(),
            vec![permit(KnowledgeUseActionV1::View, &["constraint:b"], &[])],
        );
        let assessment = assess_policy_candidates(&exact, KnowledgeUseActionV1::View, &[a, b]).unwrap();
        assert_eq!(
            assessment.state(),
            CrossPolicyCandidateStateV1::DivergentPermissionCandidates
        );
    }

    #[test]
    fn unverified_prohibitions_are_only_candidate_classifications() {
        let exact = target("revision:1", "representation:1", 1);
        let a = binding(
            "a",
            exact.clone(),
            vec![KnowledgeUseRuleV1::prohibit(KnowledgeUseActionV1::View)],
        );
        let assessment = assess_policy_candidates(&exact, KnowledgeUseActionV1::View, &[a]).unwrap();
        assert_eq!(
            assessment.state(),
            CrossPolicyCandidateStateV1::UniformProhibitionCandidates
        );
        assert!(matches!(
            assessment.candidates()[0].disposition(),
            CandidateActionDispositionV1::ProhibitedCandidate
        ));
    }

    #[test]
    fn unspecified_denial_is_not_promoted_to_final_denial() {
        let exact = target("revision:1", "representation:1", 1);
        let a = binding("a", exact.clone(), vec![]);
        let assessment = assess_policy_candidates(&exact, KnowledgeUseActionV1::View, &[a]).unwrap();
        assert_eq!(
            assessment.state(),
            CrossPolicyCandidateStateV1::UniformUnspecifiedDenialCandidates
        );
    }

    #[test]
    fn permission_and_prohibition_remain_mixed_without_precedence() {
        let exact = target("revision:1", "representation:1", 1);
        let permit_candidate = binding(
            "permit",
            exact.clone(),
            vec![permit(KnowledgeUseActionV1::View, &[], &[])],
        );
        let prohibit_candidate = binding(
            "prohibit",
            exact.clone(),
            vec![KnowledgeUseRuleV1::prohibit(KnowledgeUseActionV1::View)],
        );
        let assessment = assess_policy_candidates(
            &exact,
            KnowledgeUseActionV1::View,
            &[permit_candidate, prohibit_candidate],
        )
        .unwrap();
        assert_eq!(
            assessment.state(),
            CrossPolicyCandidateStateV1::MixedCandidateDispositions
        );
    }

    #[test]
    fn prohibition_and_unspecified_are_mixed_non_permission_candidates() {
        let exact = target("revision:1", "representation:1", 1);
        let prohibit = binding(
            "prohibit",
            exact.clone(),
            vec![KnowledgeUseRuleV1::prohibit(KnowledgeUseActionV1::View)],
        );
        let unspecified = binding("unspecified", exact.clone(), vec![]);
        let assessment = assess_policy_candidates(
            &exact,
            KnowledgeUseActionV1::View,
            &[prohibit, unspecified],
        )
        .unwrap();
        assert_eq!(
            assessment.state(),
            CrossPolicyCandidateStateV1::MixedNonPermissionCandidates
        );
    }

    #[test]
    fn view_permission_does_not_become_ai_training_permission() {
        let exact = target("revision:1", "representation:1", 1);
        let a = binding(
            "a",
            exact.clone(),
            vec![permit(KnowledgeUseActionV1::View, &[], &[])],
        );
        let assessment = assess_policy_candidates(&exact, KnowledgeUseActionV1::TrainAi, &[a]).unwrap();
        assert_eq!(
            assessment.state(),
            CrossPolicyCandidateStateV1::UniformUnspecifiedDenialCandidates
        );
    }

    #[test]
    fn target_mismatch_is_rejected_instead_of_silently_filtered() {
        let requested = target("revision:1", "representation:1", 1);
        let other = target("revision:2", "representation:2", 2);
        let candidate = binding(
            "other",
            other,
            vec![permit(KnowledgeUseActionV1::View, &[], &[])],
        );
        assert_eq!(
            assess_policy_candidates(&requested, KnowledgeUseActionV1::View, &[candidate]),
            Err(PolicySetAssessmentErrorV1::CandidateTargetMismatch)
        );
    }

    #[test]
    fn duplicate_binding_ids_are_rejected() {
        let exact = target("revision:1", "representation:1", 1);
        let candidate = binding(
            "same",
            exact.clone(),
            vec![permit(KnowledgeUseActionV1::View, &[], &[])],
        );
        assert_eq!(
            assess_policy_candidates(
                &exact,
                KnowledgeUseActionV1::View,
                &[candidate.clone(), candidate],
            ),
            Err(PolicySetAssessmentErrorV1::DuplicateBindingId)
        );
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(
            POLICY_SET_ASSESSMENT_PROFILE_V1,
            "mycelix/policy-set-assessment/v1"
        );
    }
}
