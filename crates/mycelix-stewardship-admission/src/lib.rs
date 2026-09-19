// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Profile-relative admission records for stewardship claims.
//!
//! STEW-008 creates an explicit bridge between a recorded STEW-004 claim and a
//! process-relative decision about that claim. It does not create universal
//! legitimacy, runtime authority, ownership, access rights, or policy authority.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_claims::{ClaimedStewardshipDomainV1, StewardshipClaimV1};
use mycelix_stewardship_core::{CanonicalIdErrorV1, CanonicalIdV1};

pub const STEWARDSHIP_ADMISSION_PROFILE_V1: &str = "mycelix/stewardship-admission/v1";
pub const MAX_ADMISSION_EVIDENCE_REFS_V1: usize = 32;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct AdmissionEvidenceRefV1(CanonicalIdV1);

impl AdmissionEvidenceRefV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        CanonicalIdV1::new(value).map(Self)
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

/// Decision labels are intentionally profile-relative.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum StewardshipAdmissionDispositionV1 {
    AdmittedUnderProfile,
    RejectedUnderProfile,
    Indeterminate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StewardshipAdmissionErrorV1 {
    DomainNotClaimed,
    NoEvidenceReferences,
    TooManyEvidenceReferences,
    DuplicateEvidenceReference,
}

impl fmt::Display for StewardshipAdmissionErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DomainNotClaimed => f.write_str("admission domain is not asserted by the referenced claim"),
            Self::NoEvidenceReferences => f.write_str("admission record requires evidence references"),
            Self::TooManyEvidenceReferences => f.write_str("too many admission evidence references for v1"),
            Self::DuplicateEvidenceReference => f.write_str("duplicate admission evidence reference"),
        }
    }
}

fn validate_evidence(refs: &[AdmissionEvidenceRefV1]) -> Result<(), StewardshipAdmissionErrorV1> {
    if refs.is_empty() {
        return Err(StewardshipAdmissionErrorV1::NoEvidenceReferences);
    }
    if refs.len() > MAX_ADMISSION_EVIDENCE_REFS_V1 {
        return Err(StewardshipAdmissionErrorV1::TooManyEvidenceReferences);
    }
    for (index, reference) in refs.iter().enumerate() {
        if refs[..index].contains(reference) {
            return Err(StewardshipAdmissionErrorV1::DuplicateEvidenceReference);
        }
    }
    Ok(())
}

/// Evidence-bearing process-relative decision about one complete stewardship claim.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StewardshipAdmissionRecordV1 {
    record_id: CanonicalIdV1,
    claim: StewardshipClaimV1,
    domain: ClaimedStewardshipDomainV1,
    admission_profile_ref: CanonicalIdV1,
    asserted_decider_ref: CanonicalIdV1,
    disposition: StewardshipAdmissionDispositionV1,
    evidence_refs: Vec<AdmissionEvidenceRefV1>,
}

impl StewardshipAdmissionRecordV1 {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        record_id: CanonicalIdV1,
        claim: StewardshipClaimV1,
        domain: ClaimedStewardshipDomainV1,
        admission_profile_ref: CanonicalIdV1,
        asserted_decider_ref: CanonicalIdV1,
        disposition: StewardshipAdmissionDispositionV1,
        evidence_refs: Vec<AdmissionEvidenceRefV1>,
    ) -> Result<Self, StewardshipAdmissionErrorV1> {
        if !claim.claims_domain(domain) {
            return Err(StewardshipAdmissionErrorV1::DomainNotClaimed);
        }
        validate_evidence(&evidence_refs)?;
        Ok(Self {
            record_id,
            claim,
            domain,
            admission_profile_ref,
            asserted_decider_ref,
            disposition,
            evidence_refs,
        })
    }

    pub fn record_id(&self) -> &CanonicalIdV1 {
        &self.record_id
    }

    pub fn claim(&self) -> &StewardshipClaimV1 {
        &self.claim
    }

    pub const fn domain(&self) -> ClaimedStewardshipDomainV1 {
        self.domain
    }

    pub fn admission_profile_ref(&self) -> &CanonicalIdV1 {
        &self.admission_profile_ref
    }

    pub fn asserted_decider_ref(&self) -> &CanonicalIdV1 {
        &self.asserted_decider_ref
    }

    pub const fn disposition(&self) -> StewardshipAdmissionDispositionV1 {
        self.disposition
    }

    pub fn evidence_refs(&self) -> &[AdmissionEvidenceRefV1] {
        &self.evidence_refs
    }

    /// Structural convenience only: this means the record carries the
    /// `AdmittedUnderProfile` label. It does not establish universal authority.
    pub const fn is_admitted_under_profile(&self) -> bool {
        matches!(
            self.disposition,
            StewardshipAdmissionDispositionV1::AdmittedUnderProfile
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_stewardship_claims::{
        ClaimEvidenceRefV1, StewardshipClaimBasisV1, StewardshipClaimTargetV1,
    };
    use mycelix_stewardship_core::StewardedSubjectIdV1;

    fn claim(domains: Vec<ClaimedStewardshipDomainV1>) -> StewardshipClaimV1 {
        StewardshipClaimV1::new(
            CanonicalIdV1::new("claim:1").unwrap(),
            StewardshipClaimTargetV1::Subject(
                StewardedSubjectIdV1::new("subject:tradition:1").unwrap(),
            ),
            CanonicalIdV1::new("principal:claimant:1").unwrap(),
            Some(CanonicalIdV1::new("collective:community:1").unwrap()),
            StewardshipClaimBasisV1::CommunityMandateAssertion,
            domains,
            vec![ClaimEvidenceRefV1::new("evidence:claim:1").unwrap()],
        )
        .unwrap()
    }

    fn evidence(id: &str) -> AdmissionEvidenceRefV1 {
        AdmissionEvidenceRefV1::new(id).unwrap()
    }

    #[test]
    fn admission_is_bound_to_a_domain_the_claim_actually_asserts() {
        let record = StewardshipAdmissionRecordV1::new(
            CanonicalIdV1::new("admission:1").unwrap(),
            claim(vec![ClaimedStewardshipDomainV1::CulturalProtocol]),
            ClaimedStewardshipDomainV1::CulturalProtocol,
            CanonicalIdV1::new("admission-profile:community-process:v1").unwrap(),
            CanonicalIdV1::new("principal:asserted-decider:1").unwrap(),
            StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
            vec![evidence("evidence:decision:1")],
        )
        .unwrap();
        assert!(record.is_admitted_under_profile());
    }

    #[test]
    fn admission_cannot_expand_the_claim_into_an_unclaimed_domain() {
        let result = StewardshipAdmissionRecordV1::new(
            CanonicalIdV1::new("admission:2").unwrap(),
            claim(vec![ClaimedStewardshipDomainV1::Preservation]),
            ClaimedStewardshipDomainV1::AccessPolicyParticipation,
            CanonicalIdV1::new("profile:1").unwrap(),
            CanonicalIdV1::new("principal:decider:1").unwrap(),
            StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
            vec![evidence("evidence:1")],
        );
        assert_eq!(result, Err(StewardshipAdmissionErrorV1::DomainNotClaimed));
    }

    #[test]
    fn rejected_and_indeterminate_are_first_class_outcomes() {
        for disposition in [
            StewardshipAdmissionDispositionV1::RejectedUnderProfile,
            StewardshipAdmissionDispositionV1::Indeterminate,
        ] {
            let record = StewardshipAdmissionRecordV1::new(
                CanonicalIdV1::new(format!("admission:{disposition:?}")).unwrap(),
                claim(vec![ClaimedStewardshipDomainV1::CulturalProtocol]),
                ClaimedStewardshipDomainV1::CulturalProtocol,
                CanonicalIdV1::new("profile:1").unwrap(),
                CanonicalIdV1::new("principal:decider:1").unwrap(),
                disposition,
                vec![evidence("evidence:1")],
            )
            .unwrap();
            assert!(!record.is_admitted_under_profile());
        }
    }

    #[test]
    fn evidence_is_required_and_duplicates_are_rejected() {
        let no_evidence = StewardshipAdmissionRecordV1::new(
            CanonicalIdV1::new("admission:3").unwrap(),
            claim(vec![ClaimedStewardshipDomainV1::Preservation]),
            ClaimedStewardshipDomainV1::Preservation,
            CanonicalIdV1::new("profile:1").unwrap(),
            CanonicalIdV1::new("principal:decider:1").unwrap(),
            StewardshipAdmissionDispositionV1::Indeterminate,
            vec![],
        );
        assert_eq!(
            no_evidence,
            Err(StewardshipAdmissionErrorV1::NoEvidenceReferences)
        );

        let duplicate = evidence("evidence:1");
        let duplicated = StewardshipAdmissionRecordV1::new(
            CanonicalIdV1::new("admission:4").unwrap(),
            claim(vec![ClaimedStewardshipDomainV1::Preservation]),
            ClaimedStewardshipDomainV1::Preservation,
            CanonicalIdV1::new("profile:1").unwrap(),
            CanonicalIdV1::new("principal:decider:1").unwrap(),
            StewardshipAdmissionDispositionV1::Indeterminate,
            vec![duplicate.clone(), duplicate],
        );
        assert_eq!(
            duplicated,
            Err(StewardshipAdmissionErrorV1::DuplicateEvidenceReference)
        );
    }

    #[test]
    fn same_claim_can_have_different_profile_relative_decisions() {
        let claim = claim(vec![ClaimedStewardshipDomainV1::CulturalProtocol]);
        let admitted = StewardshipAdmissionRecordV1::new(
            CanonicalIdV1::new("admission:community:1").unwrap(),
            claim.clone(),
            ClaimedStewardshipDomainV1::CulturalProtocol,
            CanonicalIdV1::new("profile:community:1").unwrap(),
            CanonicalIdV1::new("principal:community-process:1").unwrap(),
            StewardshipAdmissionDispositionV1::AdmittedUnderProfile,
            vec![evidence("evidence:community:1")],
        )
        .unwrap();
        let rejected = StewardshipAdmissionRecordV1::new(
            CanonicalIdV1::new("admission:archive:1").unwrap(),
            claim,
            ClaimedStewardshipDomainV1::CulturalProtocol,
            CanonicalIdV1::new("profile:archive:1").unwrap(),
            CanonicalIdV1::new("principal:archive-process:1").unwrap(),
            StewardshipAdmissionDispositionV1::RejectedUnderProfile,
            vec![evidence("evidence:archive:1")],
        )
        .unwrap();
        assert_ne!(admitted.disposition(), rejected.disposition());
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(
            STEWARDSHIP_ADMISSION_PROFILE_V1,
            "mycelix/stewardship-admission/v1"
        );
    }
}
