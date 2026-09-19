// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Non-ranking joint and contested stewardship case theorem.
//!
//! STEW-005 groups claim references around one explicit target/domain while
//! refusing to rank claimants or select a winner.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_claims::{ClaimedStewardshipDomainV1, StewardshipClaimTargetV1};
use mycelix_stewardship_core::{CanonicalIdErrorV1, CanonicalIdV1};

pub const STEWARDSHIP_CASE_PROFILE_V1: &str = "mycelix/stewardship-case/v1";
pub const MAX_CASE_CLAIM_REFS_V1: usize = 64;
pub const MAX_STATUS_BASIS_REFS_V1: usize = 32;

/// Structural case state, deliberately without winner semantics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StewardshipCaseStateV1 {
    /// Exactly one recorded claim is associated with the case.
    SingleRecordedClaim,
    /// Multiple recorded claims are currently represented as compatible.
    CompatibleMultipleClaims,
    /// At least two recorded claims are represented as conflicting.
    Contested,
    /// One or more recorded claims are undergoing an external review process.
    UnderReview,
    /// Claims exist but no stronger structural state is asserted.
    Unresolved,
}

/// Opaque reference to a stewardship claim object.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct StewardshipClaimRefV1(CanonicalIdV1);

impl StewardshipClaimRefV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        CanonicalIdV1::new(value).map(Self)
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

/// Opaque evidence/reference supporting the recorded case state.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CaseStatusBasisRefV1(CanonicalIdV1);

impl CaseStatusBasisRefV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        CanonicalIdV1::new(value).map(Self)
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StewardshipCaseErrorV1 {
    NoClaimReferences,
    TooManyClaimReferences,
    DuplicateClaimReference,
    TooManyStatusBasisReferences,
    DuplicateStatusBasisReference,
    StatusBasisRequired,
    SingleStateRequiresExactlyOneClaim,
    MultipleStateRequiresAtLeastTwoClaims,
}

impl fmt::Display for StewardshipCaseErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoClaimReferences => f.write_str("stewardship case needs at least one claim reference"),
            Self::TooManyClaimReferences => f.write_str("too many stewardship claim references for v1"),
            Self::DuplicateClaimReference => f.write_str("duplicate stewardship claim reference"),
            Self::TooManyStatusBasisReferences => f.write_str("too many case status-basis references for v1"),
            Self::DuplicateStatusBasisReference => f.write_str("duplicate case status-basis reference"),
            Self::StatusBasisRequired => f.write_str("case state requires at least one status-basis reference"),
            Self::SingleStateRequiresExactlyOneClaim => f.write_str("SingleRecordedClaim requires exactly one claim reference"),
            Self::MultipleStateRequiresAtLeastTwoClaims => f.write_str("multiple-claim case state requires at least two claim references"),
        }
    }
}

/// Structural grouping of claim references for one target/domain.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StewardshipCaseV1 {
    case_id: CanonicalIdV1,
    target: StewardshipClaimTargetV1,
    domain: ClaimedStewardshipDomainV1,
    claim_refs: Vec<StewardshipClaimRefV1>,
    state: StewardshipCaseStateV1,
    status_basis_refs: Vec<CaseStatusBasisRefV1>,
}

impl StewardshipCaseV1 {
    pub fn new(
        case_id: CanonicalIdV1,
        target: StewardshipClaimTargetV1,
        domain: ClaimedStewardshipDomainV1,
        claim_refs: Vec<StewardshipClaimRefV1>,
        state: StewardshipCaseStateV1,
        status_basis_refs: Vec<CaseStatusBasisRefV1>,
    ) -> Result<Self, StewardshipCaseErrorV1> {
        if claim_refs.is_empty() {
            return Err(StewardshipCaseErrorV1::NoClaimReferences);
        }
        if claim_refs.len() > MAX_CASE_CLAIM_REFS_V1 {
            return Err(StewardshipCaseErrorV1::TooManyClaimReferences);
        }
        for (index, claim_ref) in claim_refs.iter().enumerate() {
            if claim_refs[..index].contains(claim_ref) {
                return Err(StewardshipCaseErrorV1::DuplicateClaimReference);
            }
        }

        if status_basis_refs.len() > MAX_STATUS_BASIS_REFS_V1 {
            return Err(StewardshipCaseErrorV1::TooManyStatusBasisReferences);
        }
        for (index, status_ref) in status_basis_refs.iter().enumerate() {
            if status_basis_refs[..index].contains(status_ref) {
                return Err(StewardshipCaseErrorV1::DuplicateStatusBasisReference);
            }
        }

        match state {
            StewardshipCaseStateV1::SingleRecordedClaim if claim_refs.len() != 1 => {
                return Err(StewardshipCaseErrorV1::SingleStateRequiresExactlyOneClaim);
            }
            StewardshipCaseStateV1::CompatibleMultipleClaims
            | StewardshipCaseStateV1::Contested
                if claim_refs.len() < 2 =>
            {
                return Err(StewardshipCaseErrorV1::MultipleStateRequiresAtLeastTwoClaims);
            }
            _ => {}
        }

        if !matches!(state, StewardshipCaseStateV1::Unresolved)
            && status_basis_refs.is_empty()
        {
            return Err(StewardshipCaseErrorV1::StatusBasisRequired);
        }

        Ok(Self {
            case_id,
            target,
            domain,
            claim_refs,
            state,
            status_basis_refs,
        })
    }

    pub fn case_id(&self) -> &CanonicalIdV1 {
        &self.case_id
    }

    pub fn target(&self) -> &StewardshipClaimTargetV1 {
        &self.target
    }

    pub const fn domain(&self) -> ClaimedStewardshipDomainV1 {
        self.domain
    }

    pub fn claim_refs(&self) -> &[StewardshipClaimRefV1] {
        &self.claim_refs
    }

    pub const fn state(&self) -> StewardshipCaseStateV1 {
        self.state
    }

    pub fn status_basis_refs(&self) -> &[CaseStatusBasisRefV1] {
        &self.status_basis_refs
    }

    /// Structural membership only; this does not mean the claim is valid.
    pub fn contains_claim(&self, claim_ref: &StewardshipClaimRefV1) -> bool {
        self.claim_refs.contains(claim_ref)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_stewardship_core::StewardedSubjectIdV1;

    fn target() -> StewardshipClaimTargetV1 {
        StewardshipClaimTargetV1::Subject(
            StewardedSubjectIdV1::new("subject:cultural-work:example").unwrap(),
        )
    }

    fn claim(id: &str) -> StewardshipClaimRefV1 {
        StewardshipClaimRefV1::new(id).unwrap()
    }

    fn basis(id: &str) -> CaseStatusBasisRefV1 {
        CaseStatusBasisRefV1::new(id).unwrap()
    }

    #[test]
    fn unresolved_case_needs_claims_but_no_status_basis() {
        let case = StewardshipCaseV1::new(
            CanonicalIdV1::new("case:1").unwrap(),
            target(),
            ClaimedStewardshipDomainV1::CulturalProtocol,
            vec![claim("claim:1")],
            StewardshipCaseStateV1::Unresolved,
            vec![],
        )
        .unwrap();
        assert_eq!(case.state(), StewardshipCaseStateV1::Unresolved);
    }

    #[test]
    fn single_recorded_claim_is_not_called_uncontested() {
        let case = StewardshipCaseV1::new(
            CanonicalIdV1::new("case:single").unwrap(),
            target(),
            ClaimedStewardshipDomainV1::Preservation,
            vec![claim("claim:1")],
            StewardshipCaseStateV1::SingleRecordedClaim,
            vec![basis("basis:census:1")],
        )
        .unwrap();
        assert_eq!(case.claim_refs().len(), 1);
    }

    #[test]
    fn single_state_rejects_multiple_claims() {
        let result = StewardshipCaseV1::new(
            CanonicalIdV1::new("case:2").unwrap(),
            target(),
            ClaimedStewardshipDomainV1::Attribution,
            vec![claim("claim:1"), claim("claim:2")],
            StewardshipCaseStateV1::SingleRecordedClaim,
            vec![basis("basis:1")],
        );
        assert_eq!(
            result,
            Err(StewardshipCaseErrorV1::SingleStateRequiresExactlyOneClaim)
        );
    }

    #[test]
    fn contested_requires_at_least_two_claims_and_status_basis() {
        let one_claim = StewardshipCaseV1::new(
            CanonicalIdV1::new("case:3").unwrap(),
            target(),
            ClaimedStewardshipDomainV1::CulturalProtocol,
            vec![claim("claim:1")],
            StewardshipCaseStateV1::Contested,
            vec![basis("basis:conflict:1")],
        );
        assert_eq!(
            one_claim,
            Err(StewardshipCaseErrorV1::MultipleStateRequiresAtLeastTwoClaims)
        );

        let no_basis = StewardshipCaseV1::new(
            CanonicalIdV1::new("case:4").unwrap(),
            target(),
            ClaimedStewardshipDomainV1::CulturalProtocol,
            vec![claim("claim:1"), claim("claim:2")],
            StewardshipCaseStateV1::Contested,
            vec![],
        );
        assert_eq!(no_basis, Err(StewardshipCaseErrorV1::StatusBasisRequired));
    }

    #[test]
    fn compatible_multiple_claims_is_not_joint_authority() {
        let case = StewardshipCaseV1::new(
            CanonicalIdV1::new("case:5").unwrap(),
            target(),
            ClaimedStewardshipDomainV1::Transmission,
            vec![claim("claim:1"), claim("claim:2")],
            StewardshipCaseStateV1::CompatibleMultipleClaims,
            vec![basis("agreement-reference:1")],
        )
        .unwrap();
        assert_eq!(
            case.state(),
            StewardshipCaseStateV1::CompatibleMultipleClaims
        );
        assert_eq!(case.claim_refs().len(), 2);
    }

    #[test]
    fn duplicate_claim_and_basis_references_are_rejected() {
        let c = claim("claim:1");
        let result = StewardshipCaseV1::new(
            CanonicalIdV1::new("case:6").unwrap(),
            target(),
            ClaimedStewardshipDomainV1::Preservation,
            vec![c.clone(), c],
            StewardshipCaseStateV1::Contested,
            vec![basis("basis:1")],
        );
        assert_eq!(result, Err(StewardshipCaseErrorV1::DuplicateClaimReference));

        let b = basis("basis:1");
        let result = StewardshipCaseV1::new(
            CanonicalIdV1::new("case:7").unwrap(),
            target(),
            ClaimedStewardshipDomainV1::Preservation,
            vec![claim("claim:1"), claim("claim:2")],
            StewardshipCaseStateV1::Contested,
            vec![b.clone(), b],
        );
        assert_eq!(
            result,
            Err(StewardshipCaseErrorV1::DuplicateStatusBasisReference)
        );
    }

    #[test]
    fn case_has_no_selected_winner_semantics() {
        let case = StewardshipCaseV1::new(
            CanonicalIdV1::new("case:8").unwrap(),
            target(),
            ClaimedStewardshipDomainV1::RightsContext,
            vec![claim("claim:a"), claim("claim:b")],
            StewardshipCaseStateV1::Contested,
            vec![basis("basis:dispute:1")],
        )
        .unwrap();
        assert!(case.contains_claim(&claim("claim:a")));
        assert!(case.contains_claim(&claim("claim:b")));
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(
            STEWARDSHIP_CASE_PROFILE_V1,
            "mycelix/stewardship-case/v1"
        );
    }
}
