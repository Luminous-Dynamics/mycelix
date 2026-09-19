// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Structural policy-authority binding candidates.
//!
//! STEW-012B composes STEW-003 policy with STEW-008 process-relative admission.
//! It deliberately returns only a candidate and establishes no final authority.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_admission::StewardshipAdmissionRecordV1;
use mycelix_stewardship_claims::{ClaimedStewardshipDomainV1, StewardshipClaimTargetV1};
use mycelix_stewardship_core::{CanonicalIdErrorV1, CanonicalIdV1};
use mycelix_stewardship_policy::KnowledgeUsePolicyV1;

pub const POLICY_AUTHORITY_BINDING_PROFILE_V1: &str = "mycelix/policy-authority-binding/v1";
pub const MAX_AUTHORITY_EVIDENCE_REFS_V1: usize = 32;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct AuthorityEvidenceRefV1(CanonicalIdV1);

impl AuthorityEvidenceRefV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        CanonicalIdV1::new(value).map(Self)
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

/// Currentness remains an assertion carried into later evaluation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum AuthorityCurrentnessAssertionV1 {
    AssertedCurrent,
    AssertedRevoked,
    AssertedSuperseded,
    AssertedExpired,
    Indeterminate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolicyAuthorityBindingErrorV1 {
    AdmissionNotAccessPolicyDomain,
    AdmissionNotAdmittedUnderProfile,
    IssuerDoesNotMatchAdmittedClaimant,
    ClaimTargetDoesNotCoverPolicyTarget,
    CurrentnessNotAssertedCurrent,
    NoMandateEvidence,
    TooManyMandateEvidenceReferences,
    DuplicateMandateEvidenceReference,
    NoCurrentnessEvidence,
    TooManyCurrentnessEvidenceReferences,
    DuplicateCurrentnessEvidenceReference,
    NoBindingEvidence,
    TooManyBindingEvidenceReferences,
    DuplicateBindingEvidenceReference,
}

impl fmt::Display for PolicyAuthorityBindingErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::AdmissionNotAccessPolicyDomain => "admission domain is not AccessPolicyParticipation",
            Self::AdmissionNotAdmittedUnderProfile => "admission is not labelled AdmittedUnderProfile",
            Self::IssuerDoesNotMatchAdmittedClaimant => {
                "asserted policy issuer does not match admitted claimant in v1"
            }
            Self::ClaimTargetDoesNotCoverPolicyTarget => {
                "admitted claim target does not structurally cover policy target"
            }
            Self::CurrentnessNotAssertedCurrent => "authority currentness is not asserted current",
            Self::NoMandateEvidence => "policy-authority candidate requires mandate evidence",
            Self::TooManyMandateEvidenceReferences => "too many mandate evidence references",
            Self::DuplicateMandateEvidenceReference => "duplicate mandate evidence reference",
            Self::NoCurrentnessEvidence => "policy-authority candidate requires currentness evidence",
            Self::TooManyCurrentnessEvidenceReferences => "too many currentness evidence references",
            Self::DuplicateCurrentnessEvidenceReference => "duplicate currentness evidence reference",
            Self::NoBindingEvidence => "policy-authority candidate requires binding evidence",
            Self::TooManyBindingEvidenceReferences => "too many binding evidence references",
            Self::DuplicateBindingEvidenceReference => "duplicate binding evidence reference",
        };
        f.write_str(message)
    }
}

fn validate_refs(
    refs: &[AuthorityEvidenceRefV1],
    empty: PolicyAuthorityBindingErrorV1,
    too_many: PolicyAuthorityBindingErrorV1,
    duplicate: PolicyAuthorityBindingErrorV1,
) -> Result<(), PolicyAuthorityBindingErrorV1> {
    if refs.is_empty() {
        return Err(empty);
    }
    if refs.len() > MAX_AUTHORITY_EVIDENCE_REFS_V1 {
        return Err(too_many);
    }
    for (index, reference) in refs.iter().enumerate() {
        if refs[..index].contains(reference) {
            return Err(duplicate);
        }
    }
    Ok(())
}

fn claim_target_structurally_covers_policy(
    claim_target: &StewardshipClaimTargetV1,
    policy: &KnowledgeUsePolicyV1,
) -> bool {
    let target = policy.target();
    match claim_target {
        StewardshipClaimTargetV1::ExactRepresentation(identity) => identity == target,
        StewardshipClaimTargetV1::Revision { subject, revision } => {
            subject == &target.subject && revision == &target.revision
        }
        StewardshipClaimTargetV1::Subject(subject) => subject == &target.subject,
    }
}

/// Structural candidate for later policy-authority evaluation.
///
/// Construction proves only that the v1 composition prerequisites are present.
/// It does not prove that any referenced evidence, mandate, process, or currentness
/// assertion is valid.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PolicyAuthorityBindingCandidateV1 {
    binding_id: CanonicalIdV1,
    policy: KnowledgeUsePolicyV1,
    admission: StewardshipAdmissionRecordV1,
    asserted_policy_issuer: CanonicalIdV1,
    mandate_scope_ref: CanonicalIdV1,
    authority_evaluation_profile_ref: CanonicalIdV1,
    currentness: AuthorityCurrentnessAssertionV1,
    mandate_evidence_refs: Vec<AuthorityEvidenceRefV1>,
    currentness_evidence_refs: Vec<AuthorityEvidenceRefV1>,
    binding_evidence_refs: Vec<AuthorityEvidenceRefV1>,
}

impl PolicyAuthorityBindingCandidateV1 {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        binding_id: CanonicalIdV1,
        policy: KnowledgeUsePolicyV1,
        admission: StewardshipAdmissionRecordV1,
        asserted_policy_issuer: CanonicalIdV1,
        mandate_scope_ref: CanonicalIdV1,
        authority_evaluation_profile_ref: CanonicalIdV1,
        currentness: AuthorityCurrentnessAssertionV1,
        mandate_evidence_refs: Vec<AuthorityEvidenceRefV1>,
        currentness_evidence_refs: Vec<AuthorityEvidenceRefV1>,
        binding_evidence_refs: Vec<AuthorityEvidenceRefV1>,
    ) -> Result<Self, PolicyAuthorityBindingErrorV1> {
        if admission.domain() != ClaimedStewardshipDomainV1::AccessPolicyParticipation {
            return Err(PolicyAuthorityBindingErrorV1::AdmissionNotAccessPolicyDomain);
        }
        if !admission.is_admitted_under_profile() {
            return Err(PolicyAuthorityBindingErrorV1::AdmissionNotAdmittedUnderProfile);
        }
        if admission.claim().claimant() != &asserted_policy_issuer {
            return Err(PolicyAuthorityBindingErrorV1::IssuerDoesNotMatchAdmittedClaimant);
        }
        if !claim_target_structurally_covers_policy(admission.claim().target(), &policy) {
            return Err(PolicyAuthorityBindingErrorV1::ClaimTargetDoesNotCoverPolicyTarget);
        }
        if currentness != AuthorityCurrentnessAssertionV1::AssertedCurrent {
            return Err(PolicyAuthorityBindingErrorV1::CurrentnessNotAssertedCurrent);
        }

        validate_refs(
            &mandate_evidence_refs,
            PolicyAuthorityBindingErrorV1::NoMandateEvidence,
            PolicyAuthorityBindingErrorV1::TooManyMandateEvidenceReferences,
            PolicyAuthorityBindingErrorV1::DuplicateMandateEvidenceReference,
        )?;
        validate_refs(
            &currentness_evidence_refs,
            PolicyAuthorityBindingErrorV1::NoCurrentnessEvidence,
            PolicyAuthorityBindingErrorV1::TooManyCurrentnessEvidenceReferences,
            PolicyAuthorityBindingErrorV1::DuplicateCurrentnessEvidenceReference,
        )?;
        validate_refs(
            &binding_evidence_refs,
            PolicyAuthorityBindingErrorV1::NoBindingEvidence,
            PolicyAuthorityBindingErrorV1::TooManyBindingEvidenceReferences,
            PolicyAuthorityBindingErrorV1::DuplicateBindingEvidenceReference,
        )?;

        Ok(Self {
            binding_id,
            policy,
            admission,
            asserted_policy_issuer,
            mandate_scope_ref,
            authority_evaluation_profile_ref,
            currentness,
            mandate_evidence_refs,
            currentness_evidence_refs,
            binding_evidence_refs,
        })
    }

    pub fn binding_id(&self) -> &CanonicalIdV1 {
        &self.binding_id
    }

    pub fn policy(&self) -> &KnowledgeUsePolicyV1 {
        &self.policy
    }

    pub fn admission(&self) -> &StewardshipAdmissionRecordV1 {
        &self.admission
    }

    pub fn asserted_policy_issuer(&self) -> &CanonicalIdV1 {
        &self.asserted_policy_issuer
    }

    pub fn mandate_scope_ref(&self) -> &CanonicalIdV1 {
        &self.mandate_scope_ref
    }

    pub fn authority_evaluation_profile_ref(&self) -> &CanonicalIdV1 {
        &self.authority_evaluation_profile_ref
    }

    pub const fn currentness(&self) -> AuthorityCurrentnessAssertionV1 {
        self.currentness
    }

    pub fn mandate_evidence_refs(&self) -> &[AuthorityEvidenceRefV1] {
        &self.mandate_evidence_refs
    }

    pub fn currentness_evidence_refs(&self) -> &[AuthorityEvidenceRefV1] {
        &self.currentness_evidence_refs
    }

    pub fn binding_evidence_refs(&self) -> &[AuthorityEvidenceRefV1] {
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
        RevisionIdV1, StewardedSubjectIdV1, StewardedSubjectIdentityV1,
    };
    use mycelix_stewardship_policy::{KnowledgeUseActionV1, KnowledgeUseRuleV1};

    fn id(value: &str) -> CanonicalIdV1 {
        CanonicalIdV1::new(value).unwrap()
    }

    fn evidence(value: &str) -> AuthorityEvidenceRefV1 {
        AuthorityEvidenceRefV1::new(value).unwrap()
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

    fn policy(target: StewardedSubjectIdentityV1) -> KnowledgeUsePolicyV1 {
        KnowledgeUsePolicyV1::new(
            id("policy:1"),
            target,
            vec![KnowledgeUseRuleV1::permit(
                KnowledgeUseActionV1::View,
                vec![],
                vec![],
            )
            .unwrap()],
        )
        .unwrap()
    }

    fn claim(
        target: StewardshipClaimTargetV1,
        domain: ClaimedStewardshipDomainV1,
    ) -> StewardshipClaimV1 {
        StewardshipClaimV1::new(
            id("claim:1"),
            target,
            id("principal:issuer:1"),
            None,
            StewardshipClaimBasisV1::CommunityMandateAssertion,
            vec![domain],
            vec![ClaimEvidenceRefV1::new("evidence:claim:1").unwrap()],
        )
        .unwrap()
    }

    fn admission(
        claim: StewardshipClaimV1,
        domain: ClaimedStewardshipDomainV1,
        disposition: StewardshipAdmissionDispositionV1,
    ) -> StewardshipAdmissionRecordV1 {
        StewardshipAdmissionRecordV1::new(
            id("admission:1"),
            claim,
            domain,
            id("admission-profile:1"),
            id("principal:decider:1"),
            disposition,
            vec![AdmissionEvidenceRefV1::new("evidence:admission:1").unwrap()],
        )
        .unwrap()
    }

    fn candidate(
        policy: KnowledgeUsePolicyV1,
        admission: StewardshipAdmissionRecordV1,
        issuer: CanonicalIdV1,
        currentness: AuthorityCurrentnessAssertionV1,
    ) -> Result<PolicyAuthorityBindingCandidateV1, PolicyAuthorityBindingErrorV1> {
        PolicyAuthorityBindingCandidateV1::new(
            id("binding:1"),
            policy,
            admission,
            issuer,
            id("mandate-scope:1"),
            id("authority-profile:1"),
            currentness,
            vec![evidence("evidence:mandate:1")],
            vec![evidence("evidence:currentness:1")],
            vec![evidence("evidence:binding:1")],
        )
    }

    #[test]
    fn matching_exact_admission_yields_only_a_candidate() {
        let exact = target("revision:1", "representation:1", 1);
        let admitted = admission(
            claim(
                StewardshipClaimTargetV1::ExactRepresentation(exact.clone()),
                ClaimedStewardshipDomainV1::AccessPolicyParticipation,
            ),
            ClaimedStewardshipDomainV1::AccessPolicyParticipation,
            StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
        );
        let result = candidate(
            policy(exact),
            admitted,
            id("principal:issuer:1"),
            AuthorityCurrentnessAssertionV1::AssertedCurrent,
        )
        .unwrap();
        assert_eq!(result.asserted_policy_issuer().as_str(), "principal:issuer:1");
    }

    #[test]
    fn wrong_stewardship_domain_cannot_become_policy_authority() {
        let exact = target("revision:1", "representation:1", 1);
        let admitted = admission(
            claim(
                StewardshipClaimTargetV1::ExactRepresentation(exact.clone()),
                ClaimedStewardshipDomainV1::Preservation,
            ),
            ClaimedStewardshipDomainV1::Preservation,
            StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
        );
        assert_eq!(
            candidate(
                policy(exact),
                admitted,
                id("principal:issuer:1"),
                AuthorityCurrentnessAssertionV1::AssertedCurrent,
            ),
            Err(PolicyAuthorityBindingErrorV1::AdmissionNotAccessPolicyDomain)
        );
    }

    #[test]
    fn rejected_or_indeterminate_admission_fails_closed() {
        for disposition in [
            StewardshipAdmissionDispositionV1::RejectedUnderProfile,
            StewardshipAdmissionDispositionV1::Indeterminate,
        ] {
            let exact = target("revision:1", "representation:1", 1);
            let admitted = admission(
                claim(
                    StewardshipClaimTargetV1::ExactRepresentation(exact.clone()),
                    ClaimedStewardshipDomainV1::AccessPolicyParticipation,
                ),
                ClaimedStewardshipDomainV1::AccessPolicyParticipation,
                disposition,
            );
            assert_eq!(
                candidate(
                    policy(exact),
                    admitted,
                    id("principal:issuer:1"),
                    AuthorityCurrentnessAssertionV1::AssertedCurrent,
                ),
                Err(PolicyAuthorityBindingErrorV1::AdmissionNotAdmittedUnderProfile)
            );
        }
    }

    #[test]
    fn issuer_must_match_admitted_claimant_in_v1() {
        let exact = target("revision:1", "representation:1", 1);
        let admitted = admission(
            claim(
                StewardshipClaimTargetV1::ExactRepresentation(exact.clone()),
                ClaimedStewardshipDomainV1::AccessPolicyParticipation,
            ),
            ClaimedStewardshipDomainV1::AccessPolicyParticipation,
            StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
        );
        assert_eq!(
            candidate(
                policy(exact),
                admitted,
                id("principal:different:1"),
                AuthorityCurrentnessAssertionV1::AssertedCurrent,
            ),
            Err(PolicyAuthorityBindingErrorV1::IssuerDoesNotMatchAdmittedClaimant)
        );
    }

    #[test]
    fn exact_claim_does_not_propagate_to_future_revision() {
        let claimed = target("revision:1", "representation:1", 1);
        let later = target("revision:2", "representation:2", 2);
        let admitted = admission(
            claim(
                StewardshipClaimTargetV1::ExactRepresentation(claimed),
                ClaimedStewardshipDomainV1::AccessPolicyParticipation,
            ),
            ClaimedStewardshipDomainV1::AccessPolicyParticipation,
            StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
        );
        assert_eq!(
            candidate(
                policy(later),
                admitted,
                id("principal:issuer:1"),
                AuthorityCurrentnessAssertionV1::AssertedCurrent,
            ),
            Err(PolicyAuthorityBindingErrorV1::ClaimTargetDoesNotCoverPolicyTarget)
        );
    }

    #[test]
    fn revision_and_subject_targets_have_explicit_structural_coverage() {
        let exact = target("revision:1", "representation:1", 1);
        let revision_claim = claim(
            StewardshipClaimTargetV1::Revision {
                subject: exact.subject.clone(),
                revision: exact.revision.clone(),
            },
            ClaimedStewardshipDomainV1::AccessPolicyParticipation,
        );
        let subject_claim = claim(
            StewardshipClaimTargetV1::Subject(exact.subject.clone()),
            ClaimedStewardshipDomainV1::AccessPolicyParticipation,
        );

        for admitted_claim in [revision_claim, subject_claim] {
            let admitted = admission(
                admitted_claim,
                ClaimedStewardshipDomainV1::AccessPolicyParticipation,
                StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
            );
            assert!(candidate(
                policy(exact.clone()),
                admitted,
                id("principal:issuer:1"),
                AuthorityCurrentnessAssertionV1::AssertedCurrent,
            )
            .is_ok());
        }
    }

    #[test]
    fn noncurrent_authority_assertions_fail_closed() {
        for currentness in [
            AuthorityCurrentnessAssertionV1::AssertedRevoked,
            AuthorityCurrentnessAssertionV1::AssertedSuperseded,
            AuthorityCurrentnessAssertionV1::AssertedExpired,
            AuthorityCurrentnessAssertionV1::Indeterminate,
        ] {
            let exact = target("revision:1", "representation:1", 1);
            let admitted = admission(
                claim(
                    StewardshipClaimTargetV1::ExactRepresentation(exact.clone()),
                    ClaimedStewardshipDomainV1::AccessPolicyParticipation,
                ),
                ClaimedStewardshipDomainV1::AccessPolicyParticipation,
                StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
            );
            assert_eq!(
                candidate(policy(exact), admitted, id("principal:issuer:1"), currentness),
                Err(PolicyAuthorityBindingErrorV1::CurrentnessNotAssertedCurrent)
            );
        }
    }

    #[test]
    fn mandate_currentness_and_binding_evidence_are_independent_requirements() {
        let exact = target("revision:1", "representation:1", 1);
        let make_admission = || {
            admission(
                claim(
                    StewardshipClaimTargetV1::ExactRepresentation(exact.clone()),
                    ClaimedStewardshipDomainV1::AccessPolicyParticipation,
                ),
                ClaimedStewardshipDomainV1::AccessPolicyParticipation,
                StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
            )
        };

        let args = (
            id("binding:1"),
            id("principal:issuer:1"),
            id("mandate-scope:1"),
            id("authority-profile:1"),
        );

        assert_eq!(
            PolicyAuthorityBindingCandidateV1::new(
                args.0.clone(),
                policy(exact.clone()),
                make_admission(),
                args.1.clone(),
                args.2.clone(),
                args.3.clone(),
                AuthorityCurrentnessAssertionV1::AssertedCurrent,
                vec![],
                vec![evidence("evidence:currentness:1")],
                vec![evidence("evidence:binding:1")],
            ),
            Err(PolicyAuthorityBindingErrorV1::NoMandateEvidence)
        );

        assert_eq!(
            PolicyAuthorityBindingCandidateV1::new(
                args.0.clone(),
                policy(exact.clone()),
                make_admission(),
                args.1.clone(),
                args.2.clone(),
                args.3.clone(),
                AuthorityCurrentnessAssertionV1::AssertedCurrent,
                vec![evidence("evidence:mandate:1")],
                vec![],
                vec![evidence("evidence:binding:1")],
            ),
            Err(PolicyAuthorityBindingErrorV1::NoCurrentnessEvidence)
        );

        assert_eq!(
            PolicyAuthorityBindingCandidateV1::new(
                args.0,
                policy(exact.clone()),
                make_admission(),
                args.1,
                args.2,
                args.3,
                AuthorityCurrentnessAssertionV1::AssertedCurrent,
                vec![evidence("evidence:mandate:1")],
                vec![evidence("evidence:currentness:1")],
                vec![],
            ),
            Err(PolicyAuthorityBindingErrorV1::NoBindingEvidence)
        );
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(
            POLICY_AUTHORITY_BINDING_PROFILE_V1,
            "mycelix/policy-authority-binding/v1"
        );
    }
}
