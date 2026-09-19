// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Structural policy-authority candidates for Mycelix stewardship.
//!
//! STEW-012A composes an exact STEW-003 knowledge-use policy with one or more
//! STEW-008 process-relative admissions. It verifies only structural
//! prerequisites. It does not authenticate issuers/deciders, validate mandate
//! evidence, establish currentness, resolve conflicting authorities, or grant a
//! runtime capability.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_admission::{
    StewardshipAdmissionDispositionV1, StewardshipAdmissionRecordV1,
};
use mycelix_stewardship_claims::{
    ClaimedStewardshipDomainV1, StewardshipClaimTargetV1,
};
use mycelix_stewardship_core::{CanonicalIdErrorV1, CanonicalIdV1, StewardedSubjectIdentityV1};
use mycelix_stewardship_policy::KnowledgeUsePolicyV1;

pub const POLICY_AUTHORITY_CANDIDATE_PROFILE_V1: &str =
    "mycelix/stewardship-policy-authority-candidate/v1";
pub const MAX_POLICY_AUTHORITY_ADMISSIONS_V1: usize = 16;
pub const MAX_POLICY_AUTHORITY_REFS_V1: usize = 32;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MandateOrDelegationRefV1(CanonicalIdV1);

impl MandateOrDelegationRefV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        CanonicalIdV1::new(value).map(Self)
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CurrentnessEvidenceRefV1(CanonicalIdV1);

impl CurrentnessEvidenceRefV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        CanonicalIdV1::new(value).map(Self)
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PolicyAuthorityEvidenceRefV1(CanonicalIdV1);

impl PolicyAuthorityEvidenceRefV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        CanonicalIdV1::new(value).map(Self)
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolicyAuthorityCandidateErrorV1 {
    NoAdmissions,
    TooManyAdmissions,
    DuplicateAdmissionRecord,
    AdmissionNotAdmitted,
    WrongAdmissionDomain,
    AdmissionTargetDoesNotCoverPolicy,
    NoMandateOrDelegationEvidence,
    TooManyMandateOrDelegationReferences,
    DuplicateMandateOrDelegationReference,
    NoCurrentnessEvidence,
    TooManyCurrentnessReferences,
    DuplicateCurrentnessReference,
    NoBindingEvidence,
    TooManyBindingEvidenceReferences,
    DuplicateBindingEvidenceReference,
}

impl fmt::Display for PolicyAuthorityCandidateErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoAdmissions => f.write_str("policy-authority candidate requires at least one stewardship admission"),
            Self::TooManyAdmissions => f.write_str("too many stewardship admissions for v1"),
            Self::DuplicateAdmissionRecord => f.write_str("duplicate stewardship admission record"),
            Self::AdmissionNotAdmitted => f.write_str("all supplied admissions must be AdmittedUnderProfile"),
            Self::WrongAdmissionDomain => f.write_str("policy authority requires AccessPolicyParticipation admission"),
            Self::AdmissionTargetDoesNotCoverPolicy => f.write_str("admission claim target does not structurally cover exact policy target"),
            Self::NoMandateOrDelegationEvidence => f.write_str("policy-authority candidate requires mandate or delegation evidence"),
            Self::TooManyMandateOrDelegationReferences => f.write_str("too many mandate/delegation references for v1"),
            Self::DuplicateMandateOrDelegationReference => f.write_str("duplicate mandate/delegation reference"),
            Self::NoCurrentnessEvidence => f.write_str("policy-authority candidate requires currentness evidence"),
            Self::TooManyCurrentnessReferences => f.write_str("too many currentness references for v1"),
            Self::DuplicateCurrentnessReference => f.write_str("duplicate currentness reference"),
            Self::NoBindingEvidence => f.write_str("policy-authority candidate requires binding evidence"),
            Self::TooManyBindingEvidenceReferences => f.write_str("too many binding evidence references for v1"),
            Self::DuplicateBindingEvidenceReference => f.write_str("duplicate binding evidence reference"),
        }
    }
}

fn target_structurally_covers(
    claim_target: &StewardshipClaimTargetV1,
    policy_target: &StewardedSubjectIdentityV1,
) -> bool {
    match claim_target {
        StewardshipClaimTargetV1::ExactRepresentation(identity) => identity == policy_target,
        StewardshipClaimTargetV1::Revision { subject, revision } => {
            subject == &policy_target.subject && revision == &policy_target.revision
        }
        StewardshipClaimTargetV1::Subject(subject) => subject == &policy_target.subject,
    }
}

fn validate_unique_nonempty<T: PartialEq>(
    refs: &[T],
    empty: PolicyAuthorityCandidateErrorV1,
    too_many: PolicyAuthorityCandidateErrorV1,
    duplicate: PolicyAuthorityCandidateErrorV1,
) -> Result<(), PolicyAuthorityCandidateErrorV1> {
    if refs.is_empty() {
        return Err(empty);
    }
    if refs.len() > MAX_POLICY_AUTHORITY_REFS_V1 {
        return Err(too_many);
    }
    for (index, reference) in refs.iter().enumerate() {
        if refs[..index].contains(reference) {
            return Err(duplicate);
        }
    }
    Ok(())
}

/// Structural candidate for later authority evaluation.
///
/// Construction proves only the fail-closed structural conditions checked by
/// this crate. It never represents final authorization.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PolicyAuthorityCandidateV1 {
    candidate_id: CanonicalIdV1,
    policy: KnowledgeUsePolicyV1,
    asserted_issuer_ref: CanonicalIdV1,
    admissions: Vec<StewardshipAdmissionRecordV1>,
    mandate_or_delegation_refs: Vec<MandateOrDelegationRefV1>,
    authority_evaluation_profile_ref: CanonicalIdV1,
    currentness_refs: Vec<CurrentnessEvidenceRefV1>,
    binding_evidence_refs: Vec<PolicyAuthorityEvidenceRefV1>,
}

impl PolicyAuthorityCandidateV1 {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        candidate_id: CanonicalIdV1,
        policy: KnowledgeUsePolicyV1,
        asserted_issuer_ref: CanonicalIdV1,
        admissions: Vec<StewardshipAdmissionRecordV1>,
        mandate_or_delegation_refs: Vec<MandateOrDelegationRefV1>,
        authority_evaluation_profile_ref: CanonicalIdV1,
        currentness_refs: Vec<CurrentnessEvidenceRefV1>,
        binding_evidence_refs: Vec<PolicyAuthorityEvidenceRefV1>,
    ) -> Result<Self, PolicyAuthorityCandidateErrorV1> {
        if admissions.is_empty() {
            return Err(PolicyAuthorityCandidateErrorV1::NoAdmissions);
        }
        if admissions.len() > MAX_POLICY_AUTHORITY_ADMISSIONS_V1 {
            return Err(PolicyAuthorityCandidateErrorV1::TooManyAdmissions);
        }
        for (index, admission) in admissions.iter().enumerate() {
            if admissions[..index]
                .iter()
                .any(|previous| previous.record_id() == admission.record_id())
            {
                return Err(PolicyAuthorityCandidateErrorV1::DuplicateAdmissionRecord);
            }
            if admission.disposition()
                != StewardshipAdmissionDispositionV1::AdmittedUnderProfile
            {
                return Err(PolicyAuthorityCandidateErrorV1::AdmissionNotAdmitted);
            }
            if admission.domain() != ClaimedStewardshipDomainV1::AccessPolicyParticipation {
                return Err(PolicyAuthorityCandidateErrorV1::WrongAdmissionDomain);
            }
            if !target_structurally_covers(admission.claim().target(), policy.target()) {
                return Err(PolicyAuthorityCandidateErrorV1::AdmissionTargetDoesNotCoverPolicy);
            }
        }

        validate_unique_nonempty(
            &mandate_or_delegation_refs,
            PolicyAuthorityCandidateErrorV1::NoMandateOrDelegationEvidence,
            PolicyAuthorityCandidateErrorV1::TooManyMandateOrDelegationReferences,
            PolicyAuthorityCandidateErrorV1::DuplicateMandateOrDelegationReference,
        )?;
        validate_unique_nonempty(
            &currentness_refs,
            PolicyAuthorityCandidateErrorV1::NoCurrentnessEvidence,
            PolicyAuthorityCandidateErrorV1::TooManyCurrentnessReferences,
            PolicyAuthorityCandidateErrorV1::DuplicateCurrentnessReference,
        )?;
        validate_unique_nonempty(
            &binding_evidence_refs,
            PolicyAuthorityCandidateErrorV1::NoBindingEvidence,
            PolicyAuthorityCandidateErrorV1::TooManyBindingEvidenceReferences,
            PolicyAuthorityCandidateErrorV1::DuplicateBindingEvidenceReference,
        )?;

        Ok(Self {
            candidate_id,
            policy,
            asserted_issuer_ref,
            admissions,
            mandate_or_delegation_refs,
            authority_evaluation_profile_ref,
            currentness_refs,
            binding_evidence_refs,
        })
    }

    pub fn candidate_id(&self) -> &CanonicalIdV1 {
        &self.candidate_id
    }

    pub fn policy(&self) -> &KnowledgeUsePolicyV1 {
        &self.policy
    }

    pub fn asserted_issuer_ref(&self) -> &CanonicalIdV1 {
        &self.asserted_issuer_ref
    }

    pub fn admissions(&self) -> &[StewardshipAdmissionRecordV1] {
        &self.admissions
    }

    pub fn mandate_or_delegation_refs(&self) -> &[MandateOrDelegationRefV1] {
        &self.mandate_or_delegation_refs
    }

    pub fn authority_evaluation_profile_ref(&self) -> &CanonicalIdV1 {
        &self.authority_evaluation_profile_ref
    }

    pub fn currentness_refs(&self) -> &[CurrentnessEvidenceRefV1] {
        &self.currentness_refs
    }

    pub fn binding_evidence_refs(&self) -> &[PolicyAuthorityEvidenceRefV1] {
        &self.binding_evidence_refs
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_stewardship_admission::{
        AdmissionEvidenceRefV1, StewardshipAdmissionDispositionV1,
    };
    use mycelix_stewardship_claims::{
        ClaimEvidenceRefV1, StewardshipClaimBasisV1, StewardshipClaimV1,
    };
    use mycelix_stewardship_core::{
        ContentDigestV1, DigestAlgorithmV1, RepresentationIdV1, RepresentationKindV1,
        RevisionIdV1, StewardedSubjectIdV1,
    };
    use mycelix_stewardship_policy::{KnowledgeUseActionV1, KnowledgeUseRuleV1};

    fn identity(subject: &str, revision: &str, representation: &str, byte: u8) -> StewardedSubjectIdentityV1 {
        StewardedSubjectIdentityV1 {
            subject: StewardedSubjectIdV1::new(subject).unwrap(),
            revision: RevisionIdV1::new(revision).unwrap(),
            representation: RepresentationIdV1::new(representation).unwrap(),
            kind: RepresentationKindV1::Text,
            content_digest: ContentDigestV1::new(DigestAlgorithmV1::Blake3_256, [byte; 32]),
        }
    }

    fn target() -> StewardedSubjectIdentityV1 {
        identity("subject:work:1", "revision:1", "representation:text:1", 1)
    }

    fn policy() -> KnowledgeUsePolicyV1 {
        KnowledgeUsePolicyV1::new(
            CanonicalIdV1::new("policy:1").unwrap(),
            target(),
            vec![KnowledgeUseRuleV1::permit(KnowledgeUseActionV1::View, vec![], vec![]).unwrap()],
        )
        .unwrap()
    }

    fn claim(target: StewardshipClaimTargetV1, domain: ClaimedStewardshipDomainV1) -> StewardshipClaimV1 {
        StewardshipClaimV1::new(
            CanonicalIdV1::new("claim:1").unwrap(),
            target,
            CanonicalIdV1::new("principal:claimant:1").unwrap(),
            None,
            StewardshipClaimBasisV1::DelegationAssertion,
            vec![domain],
            vec![ClaimEvidenceRefV1::new("evidence:claim:1").unwrap()],
        )
        .unwrap()
    }

    fn admission(
        claim_target: StewardshipClaimTargetV1,
        domain: ClaimedStewardshipDomainV1,
        disposition: StewardshipAdmissionDispositionV1,
        record_id: &str,
    ) -> StewardshipAdmissionRecordV1 {
        StewardshipAdmissionRecordV1::new(
            CanonicalIdV1::new(record_id).unwrap(),
            claim(claim_target, domain),
            domain,
            CanonicalIdV1::new("profile:admission:1").unwrap(),
            CanonicalIdV1::new("principal:decider:1").unwrap(),
            disposition,
            vec![AdmissionEvidenceRefV1::new("evidence:admission:1").unwrap()],
        )
        .unwrap()
    }

    fn mandate() -> Vec<MandateOrDelegationRefV1> {
        vec![MandateOrDelegationRefV1::new("evidence:mandate:1").unwrap()]
    }

    fn currentness() -> Vec<CurrentnessEvidenceRefV1> {
        vec![CurrentnessEvidenceRefV1::new("evidence:currentness:1").unwrap()]
    }

    fn binding() -> Vec<PolicyAuthorityEvidenceRefV1> {
        vec![PolicyAuthorityEvidenceRefV1::new("evidence:binding:1").unwrap()]
    }

    fn candidate_with(admissions: Vec<StewardshipAdmissionRecordV1>) -> Result<PolicyAuthorityCandidateV1, PolicyAuthorityCandidateErrorV1> {
        PolicyAuthorityCandidateV1::new(
            CanonicalIdV1::new("candidate:1").unwrap(),
            policy(),
            CanonicalIdV1::new("principal:asserted-issuer:1").unwrap(),
            admissions,
            mandate(),
            CanonicalIdV1::new("profile:authority-evaluation:1").unwrap(),
            currentness(),
            binding(),
        )
    }

    #[test]
    fn exact_representation_admission_can_form_structural_candidate() {
        let candidate = candidate_with(vec![admission(
            StewardshipClaimTargetV1::ExactRepresentation(target()),
            ClaimedStewardshipDomainV1::AccessPolicyParticipation,
            StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
            "admission:1",
        )])
        .unwrap();
        assert_eq!(candidate.policy().target(), &target());
    }

    #[test]
    fn revision_and_subject_claims_have_explicit_structural_coverage() {
        let revision = admission(
            StewardshipClaimTargetV1::Revision {
                subject: target().subject.clone(),
                revision: target().revision.clone(),
            },
            ClaimedStewardshipDomainV1::AccessPolicyParticipation,
            StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
            "admission:revision",
        );
        assert!(candidate_with(vec![revision]).is_ok());

        let subject = admission(
            StewardshipClaimTargetV1::Subject(target().subject.clone()),
            ClaimedStewardshipDomainV1::AccessPolicyParticipation,
            StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
            "admission:subject",
        );
        assert!(candidate_with(vec![subject]).is_ok());
    }

    #[test]
    fn wrong_domain_fails_closed() {
        let result = candidate_with(vec![admission(
            StewardshipClaimTargetV1::Subject(target().subject.clone()),
            ClaimedStewardshipDomainV1::Preservation,
            StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
            "admission:wrong-domain",
        )]);
        assert_eq!(result, Err(PolicyAuthorityCandidateErrorV1::WrongAdmissionDomain));
    }

    #[test]
    fn rejected_or_indeterminate_admission_fails_closed() {
        for disposition in [
            StewardshipAdmissionDispositionV1::RejectedUnderProfile,
            StewardshipAdmissionDispositionV1::Indeterminate,
        ] {
            let result = candidate_with(vec![admission(
                StewardshipClaimTargetV1::Subject(target().subject.clone()),
                ClaimedStewardshipDomainV1::AccessPolicyParticipation,
                disposition,
                "admission:not-admitted",
            )]);
            assert_eq!(result, Err(PolicyAuthorityCandidateErrorV1::AdmissionNotAdmitted));
        }
    }

    #[test]
    fn different_subject_revision_or_exact_representation_fails_closed() {
        let cases = [
            StewardshipClaimTargetV1::Subject(
                StewardedSubjectIdV1::new("subject:other").unwrap(),
            ),
            StewardshipClaimTargetV1::Revision {
                subject: target().subject.clone(),
                revision: RevisionIdV1::new("revision:other").unwrap(),
            },
            StewardshipClaimTargetV1::ExactRepresentation(identity(
                "subject:work:1",
                "revision:1",
                "representation:text:other",
                2,
            )),
        ];

        for (index, claim_target) in cases.into_iter().enumerate() {
            let result = candidate_with(vec![admission(
                claim_target,
                ClaimedStewardshipDomainV1::AccessPolicyParticipation,
                StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
                &format!("admission:mismatch:{index}"),
            )]);
            assert_eq!(
                result,
                Err(PolicyAuthorityCandidateErrorV1::AdmissionTargetDoesNotCoverPolicy)
            );
        }
    }

    #[test]
    fn mandate_currentness_and_binding_evidence_are_all_required() {
        let admitted = admission(
            StewardshipClaimTargetV1::ExactRepresentation(target()),
            ClaimedStewardshipDomainV1::AccessPolicyParticipation,
            StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
            "admission:1",
        );

        let base = |mandates, current, evidence| {
            PolicyAuthorityCandidateV1::new(
                CanonicalIdV1::new("candidate:requirements").unwrap(),
                policy(),
                CanonicalIdV1::new("principal:issuer:1").unwrap(),
                vec![admitted.clone()],
                mandates,
                CanonicalIdV1::new("profile:authority:1").unwrap(),
                current,
                evidence,
            )
        };

        assert_eq!(
            base(vec![], currentness(), binding()),
            Err(PolicyAuthorityCandidateErrorV1::NoMandateOrDelegationEvidence)
        );
        assert_eq!(
            base(mandate(), vec![], binding()),
            Err(PolicyAuthorityCandidateErrorV1::NoCurrentnessEvidence)
        );
        assert_eq!(
            base(mandate(), currentness(), vec![]),
            Err(PolicyAuthorityCandidateErrorV1::NoBindingEvidence)
        );
    }

    #[test]
    fn duplicate_admission_record_is_rejected() {
        let admission = admission(
            StewardshipClaimTargetV1::ExactRepresentation(target()),
            ClaimedStewardshipDomainV1::AccessPolicyParticipation,
            StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
            "admission:duplicate",
        );
        assert_eq!(
            candidate_with(vec![admission.clone(), admission]),
            Err(PolicyAuthorityCandidateErrorV1::DuplicateAdmissionRecord)
        );
    }

    #[test]
    fn candidate_does_not_upgrade_policy_permission_classification() {
        let candidate = candidate_with(vec![admission(
            StewardshipClaimTargetV1::ExactRepresentation(target()),
            ClaimedStewardshipDomainV1::AccessPolicyParticipation,
            StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
            "admission:1",
        )])
        .unwrap();
        assert!(matches!(
            candidate.policy().classify_action(KnowledgeUseActionV1::View),
            mycelix_stewardship_policy::ActionDispositionV1::PermissionCandidate { .. }
        ));
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(
            POLICY_AUTHORITY_CANDIDATE_PROFILE_V1,
            "mycelix/stewardship-policy-authority-candidate/v1"
        );
    }
}
