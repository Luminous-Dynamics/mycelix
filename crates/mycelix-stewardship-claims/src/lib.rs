// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Evidence-bearing, authority-neutral stewardship claims.
//!
//! STEW-004 records what is claimed, by whom, for which explicit target breadth,
//! in which stewardship domains, and with which evidence references. It does not
//! verify those claims or grant authority.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_core::{
    CanonicalIdErrorV1, CanonicalIdV1, RevisionIdV1, StewardedSubjectIdV1,
    StewardedSubjectIdentityV1,
};

/// Stable profile identifier for STEW-004.
pub const STEWARDSHIP_CLAIM_PROFILE_V1: &str = "mycelix/stewardship-claim/v1";
/// Maximum evidence references admitted by one v1 claim.
pub const MAX_CLAIM_EVIDENCE_REFS_V1: usize = 32;
/// Maximum claimed stewardship domains admitted by one v1 claim.
pub const MAX_CLAIMED_DOMAINS_V1: usize = 16;

/// Explicit breadth of the subject covered by a stewardship claim.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum StewardshipClaimTargetV1 {
    /// One exact representation, including its exact content commitment.
    ExactRepresentation(StewardedSubjectIdentityV1),
    /// One revision of a logical subject, independent of representation.
    Revision {
        subject: StewardedSubjectIdV1,
        revision: RevisionIdV1,
    },
    /// A logical subject across revisions/representations.
    Subject(StewardedSubjectIdV1),
}

impl StewardshipClaimTargetV1 {
    /// Logical subject named by this target.
    pub fn subject(&self) -> &StewardedSubjectIdV1 {
        match self {
            Self::ExactRepresentation(identity) => &identity.subject,
            Self::Revision { subject, .. } | Self::Subject(subject) => subject,
        }
    }

    /// Whether this claim explicitly uses the broad logical-subject scope.
    pub const fn is_subject_wide(&self) -> bool {
        matches!(self, Self::Subject(_))
    }
}

/// Descriptive asserted basis for a claim.
///
/// Variant names intentionally include `Assertion` where a role could otherwise
/// be misread as verified status.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum StewardshipClaimBasisV1 {
    CreatorAssertion,
    InheritanceAssertion,
    CommunityMandateAssertion,
    CustodialAssertion,
    DelegationAssertion,
    InstitutionalAssertion,
    ContractualAssertion,
    LegalAssertion,
    TraditionalRelationshipAssertion,
    OtherAssertion,
}

/// Domain in which the claimant asserts a stewardship relationship.
///
/// These are not capabilities or permissions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ClaimedStewardshipDomainV1 {
    MetadataCuration,
    Provenance,
    Preservation,
    AccessPolicyParticipation,
    RightsContext,
    CulturalProtocol,
    Attribution,
    Reciprocity,
    Transmission,
    Other,
}

/// Opaque evidence reference carried by a claim.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ClaimEvidenceRefV1(CanonicalIdV1);

impl ClaimEvidenceRefV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        CanonicalIdV1::new(value).map(Self)
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

/// Validation failure while constructing a v1 stewardship claim.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StewardshipClaimErrorV1 {
    NoClaimedDomains,
    TooManyClaimedDomains,
    DuplicateClaimedDomain,
    NoEvidenceReferences,
    TooManyEvidenceReferences,
    DuplicateEvidenceReference,
    ClaimantEqualsRepresentedCollective,
}

impl fmt::Display for StewardshipClaimErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoClaimedDomains => f.write_str("stewardship claim needs at least one domain"),
            Self::TooManyClaimedDomains => f.write_str("too many stewardship domains for v1"),
            Self::DuplicateClaimedDomain => f.write_str("duplicate claimed stewardship domain"),
            Self::NoEvidenceReferences => f.write_str("stewardship claim needs evidence references"),
            Self::TooManyEvidenceReferences => f.write_str("too many evidence references for v1"),
            Self::DuplicateEvidenceReference => f.write_str("duplicate stewardship evidence reference"),
            Self::ClaimantEqualsRepresentedCollective => f.write_str(
                "represented_collective is only for representation claims and must differ from claimant",
            ),
        }
    }
}

/// Evidence-bearing stewardship claim with no embedded authority decision.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StewardshipClaimV1 {
    claim_id: CanonicalIdV1,
    target: StewardshipClaimTargetV1,
    claimant: CanonicalIdV1,
    represented_collective: Option<CanonicalIdV1>,
    basis: StewardshipClaimBasisV1,
    domains: Vec<ClaimedStewardshipDomainV1>,
    evidence_refs: Vec<ClaimEvidenceRefV1>,
}

impl StewardshipClaimV1 {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        claim_id: CanonicalIdV1,
        target: StewardshipClaimTargetV1,
        claimant: CanonicalIdV1,
        represented_collective: Option<CanonicalIdV1>,
        basis: StewardshipClaimBasisV1,
        domains: Vec<ClaimedStewardshipDomainV1>,
        evidence_refs: Vec<ClaimEvidenceRefV1>,
    ) -> Result<Self, StewardshipClaimErrorV1> {
        if domains.is_empty() {
            return Err(StewardshipClaimErrorV1::NoClaimedDomains);
        }
        if domains.len() > MAX_CLAIMED_DOMAINS_V1 {
            return Err(StewardshipClaimErrorV1::TooManyClaimedDomains);
        }
        for (index, domain) in domains.iter().enumerate() {
            if domains[..index].contains(domain) {
                return Err(StewardshipClaimErrorV1::DuplicateClaimedDomain);
            }
        }

        if evidence_refs.is_empty() {
            return Err(StewardshipClaimErrorV1::NoEvidenceReferences);
        }
        if evidence_refs.len() > MAX_CLAIM_EVIDENCE_REFS_V1 {
            return Err(StewardshipClaimErrorV1::TooManyEvidenceReferences);
        }
        for (index, reference) in evidence_refs.iter().enumerate() {
            if evidence_refs[..index].contains(reference) {
                return Err(StewardshipClaimErrorV1::DuplicateEvidenceReference);
            }
        }

        if represented_collective.as_ref() == Some(&claimant) {
            return Err(StewardshipClaimErrorV1::ClaimantEqualsRepresentedCollective);
        }

        Ok(Self {
            claim_id,
            target,
            claimant,
            represented_collective,
            basis,
            domains,
            evidence_refs,
        })
    }

    pub fn claim_id(&self) -> &CanonicalIdV1 {
        &self.claim_id
    }

    pub fn target(&self) -> &StewardshipClaimTargetV1 {
        &self.target
    }

    pub fn claimant(&self) -> &CanonicalIdV1 {
        &self.claimant
    }

    /// Collective the claimant says it represents. This is descriptive only.
    pub fn represented_collective(&self) -> Option<&CanonicalIdV1> {
        self.represented_collective.as_ref()
    }

    pub const fn basis(&self) -> StewardshipClaimBasisV1 {
        self.basis
    }

    pub fn domains(&self) -> &[ClaimedStewardshipDomainV1] {
        &self.domains
    }

    pub fn evidence_refs(&self) -> &[ClaimEvidenceRefV1] {
        &self.evidence_refs
    }

    pub fn claims_domain(&self, domain: ClaimedStewardshipDomainV1) -> bool {
        self.domains.contains(&domain)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_stewardship_core::{
        ContentDigestV1, DigestAlgorithmV1, RepresentationIdV1, RepresentationKindV1,
    };

    fn subject() -> StewardedSubjectIdV1 {
        StewardedSubjectIdV1::new("subject:tradition:example").unwrap()
    }

    fn exact_identity() -> StewardedSubjectIdentityV1 {
        StewardedSubjectIdentityV1 {
            subject: subject(),
            revision: RevisionIdV1::new("revision:transcription:1").unwrap(),
            representation: RepresentationIdV1::new("representation:text:1").unwrap(),
            kind: RepresentationKindV1::Text,
            content_digest: ContentDigestV1::new(DigestAlgorithmV1::Blake3_256, [4; 32]),
        }
    }

    fn evidence(value: &str) -> ClaimEvidenceRefV1 {
        ClaimEvidenceRefV1::new(value).unwrap()
    }

    fn base_claim(target: StewardshipClaimTargetV1) -> StewardshipClaimV1 {
        StewardshipClaimV1::new(
            CanonicalIdV1::new("claim:stewardship:example:1").unwrap(),
            target,
            CanonicalIdV1::new("principal:claimant:1").unwrap(),
            None,
            StewardshipClaimBasisV1::TraditionalRelationshipAssertion,
            vec![ClaimedStewardshipDomainV1::CulturalProtocol],
            vec![evidence("evidence:testimony:1")],
        )
        .unwrap()
    }

    #[test]
    fn exact_revision_and_subject_targets_remain_distinct() {
        let exact = StewardshipClaimTargetV1::ExactRepresentation(exact_identity());
        let revision = StewardshipClaimTargetV1::Revision {
            subject: subject(),
            revision: RevisionIdV1::new("revision:transcription:1").unwrap(),
        };
        let broad = StewardshipClaimTargetV1::Subject(subject());

        assert_ne!(exact, revision);
        assert_ne!(revision, broad);
        assert!(!exact.is_subject_wide());
        assert!(broad.is_subject_wide());
    }

    #[test]
    fn claim_requires_evidence_but_does_not_evaluate_it() {
        let result = StewardshipClaimV1::new(
            CanonicalIdV1::new("claim:1").unwrap(),
            StewardshipClaimTargetV1::Subject(subject()),
            CanonicalIdV1::new("principal:1").unwrap(),
            None,
            StewardshipClaimBasisV1::CommunityMandateAssertion,
            vec![ClaimedStewardshipDomainV1::Preservation],
            vec![],
        );
        assert_eq!(result, Err(StewardshipClaimErrorV1::NoEvidenceReferences));
    }

    #[test]
    fn duplicate_domains_and_evidence_are_rejected() {
        let target = StewardshipClaimTargetV1::Subject(subject());
        let domain = ClaimedStewardshipDomainV1::Attribution;
        let result = StewardshipClaimV1::new(
            CanonicalIdV1::new("claim:2").unwrap(),
            target.clone(),
            CanonicalIdV1::new("principal:1").unwrap(),
            None,
            StewardshipClaimBasisV1::CreatorAssertion,
            vec![domain, domain],
            vec![evidence("evidence:1")],
        );
        assert_eq!(result, Err(StewardshipClaimErrorV1::DuplicateClaimedDomain));

        let evidence_ref = evidence("evidence:1");
        let result = StewardshipClaimV1::new(
            CanonicalIdV1::new("claim:3").unwrap(),
            target,
            CanonicalIdV1::new("principal:1").unwrap(),
            None,
            StewardshipClaimBasisV1::CreatorAssertion,
            vec![domain],
            vec![evidence_ref.clone(), evidence_ref],
        );
        assert_eq!(result, Err(StewardshipClaimErrorV1::DuplicateEvidenceReference));
    }

    #[test]
    fn collective_representation_is_an_assertion_not_a_verification_result() {
        let claim = StewardshipClaimV1::new(
            CanonicalIdV1::new("claim:collective:1").unwrap(),
            StewardshipClaimTargetV1::Subject(subject()),
            CanonicalIdV1::new("principal:delegate:1").unwrap(),
            Some(CanonicalIdV1::new("collective:community:1").unwrap()),
            StewardshipClaimBasisV1::CommunityMandateAssertion,
            vec![ClaimedStewardshipDomainV1::CulturalProtocol],
            vec![evidence("evidence:mandate:1")],
        )
        .unwrap();
        assert_eq!(
            claim.represented_collective().map(CanonicalIdV1::as_str),
            Some("collective:community:1")
        );
    }

    #[test]
    fn claimant_cannot_use_itself_as_represented_collective_marker() {
        let same = CanonicalIdV1::new("principal:1").unwrap();
        let result = StewardshipClaimV1::new(
            CanonicalIdV1::new("claim:4").unwrap(),
            StewardshipClaimTargetV1::Subject(subject()),
            same.clone(),
            Some(same),
            StewardshipClaimBasisV1::CommunityMandateAssertion,
            vec![ClaimedStewardshipDomainV1::CulturalProtocol],
            vec![evidence("evidence:1")],
        );
        assert_eq!(
            result,
            Err(StewardshipClaimErrorV1::ClaimantEqualsRepresentedCollective)
        );
    }

    #[test]
    fn claimed_domains_are_not_collapsed() {
        let claim = StewardshipClaimV1::new(
            CanonicalIdV1::new("claim:5").unwrap(),
            StewardshipClaimTargetV1::ExactRepresentation(exact_identity()),
            CanonicalIdV1::new("principal:1").unwrap(),
            None,
            StewardshipClaimBasisV1::CustodialAssertion,
            vec![
                ClaimedStewardshipDomainV1::Preservation,
                ClaimedStewardshipDomainV1::MetadataCuration,
            ],
            vec![evidence("evidence:custody:1")],
        )
        .unwrap();
        assert!(claim.claims_domain(ClaimedStewardshipDomainV1::Preservation));
        assert!(claim.claims_domain(ClaimedStewardshipDomainV1::MetadataCuration));
        assert!(!claim.claims_domain(ClaimedStewardshipDomainV1::AccessPolicyParticipation));
    }

    #[test]
    fn broad_subject_target_is_explicit() {
        let claim = base_claim(StewardshipClaimTargetV1::Subject(subject()));
        assert!(claim.target().is_subject_wide());
        assert_eq!(claim.target().subject().as_str(), "subject:tradition:example");
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(
            STEWARDSHIP_CLAIM_PROFILE_V1,
            "mycelix/stewardship-claim/v1"
        );
    }
}
