// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Constitutional authority taxonomy and conformance rules for Mycelix.
//!
//! This crate is deliberately pure: it has no HDK/HDI dependency and performs
//! no host calls. It defines the constitutional vocabulary that governance,
//! civic, justice, integrity, and stewardship components can share without
//! coupling their execution environments.
//!
//! Design rules encoded here:
//! - constituent sovereignty is not a sixth branch;
//! - Mycelix has five constituted branches;
//! - guardians are constitutionally protected but are not sovereign branches;
//! - automated agents do not hold constitutional sovereignty;
//! - foundational civic decisions require equal civic vote weight;
//! - branch powers are allow-listed rather than inferred from role names;
//! - extraordinary emergency powers require explicit expiry.

use serde::{Deserialize, Serialize};

/// The five constituted branches of Mycelix governance.
///
/// `ConstituentSovereignty` intentionally does not appear here. The people and
/// constituent communities are the source of delegated authority, not another
/// office inside the government they constitute.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Branch {
    Deliberative,
    Stewardship,
    Justice,
    Integrity,
    CivicMandate,
}

impl Branch {
    pub const ALL: [Self; 5] = [
        Self::Deliberative,
        Self::Stewardship,
        Self::Justice,
        Self::Integrity,
        Self::CivicMandate,
    ];
}

/// Constitutionally protected independent institutions that support or check
/// the branches without becoming additional sovereign branches.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Guardian {
    RightsDefender,
    PublicEvidence,
    FutureGenerations,
    FiscalObservatory,
    PublicService,
    ProsecutionService,
}

impl Guardian {
    pub const ALL: [Self; 6] = [
        Self::RightsDefender,
        Self::PublicEvidence,
        Self::FutureGenerations,
        Self::FiscalObservatory,
        Self::PublicService,
        Self::ProsecutionService,
    ];
}

/// A principal that may appear in a constitutional authorization record.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AuthorityPrincipal {
    /// Citizens and constituent communities acting through a valid constituent
    /// process. This is the source of the constitution, not a branch.
    ConstituentSovereignty,
    /// One of the five constituted branches.
    Branch(Branch),
    /// An independent guardian with narrowly enumerated authority.
    Guardian(Guardian),
    /// Software/AI acting as an automated principal.
    ///
    /// Automated agents may receive bounded operational capabilities elsewhere,
    /// but they do not directly hold any constitutional power enumerated here.
    AutomatedAgent,
}

/// Constitutional powers that require explicit allocation.
///
/// This is intentionally narrower than all actions an implementation may take.
/// Routine internal administration should remain ordinary application logic.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ConstitutionalPower {
    // Deliberative branch
    ProposeOrdinaryLaw,
    EnactOrdinaryLaw,
    AppropriatePublicFunds,
    RatifyTreaty,
    AuthorizeEmergency,
    ConductLegislativeOversight,

    // Stewardship branch
    ExecuteLaw,
    ExecuteAppropriation,
    AdministerPublicService,
    DirectPublicAdministration,
    DeclareProvisionalEmergency,

    // Justice branch
    AdjudicateDispute,
    ConductConstitutionalReview,
    IssueJudicialRemedy,

    // Integrity branch
    AuditPublicExpenditure,
    AuditAuthorityUse,
    InvestigatePublicIntegrity,
    PublishIntegrityFinding,
    ReferForProsecution,

    // Civic Mandate branch
    AdministerElection,
    CertifyMandate,
    AdministerRecall,
    AdministerInitiative,
    AdministerSortition,
    VerifyCivicEligibility,

    // Constituent sovereignty
    CallConstitutionalConvention,
    RatifyStructuralConstitution,
    RatifyFoundationalCovenant,
    WithdrawConstituentDelegation,

    // Guardian powers
    InitiateRightsChallenge,
    RequestProtectedPublicRecord,
    PublishEvidenceAssessment,
    InitiateFutureGenerationsReview,
    PublishFiscalAssessment,
    CertifyPublicServiceQualification,
    InitiatePublicProsecution,
}

impl ConstitutionalPower {
    /// Exhaustive list used by conformance tests.
    pub const ALL: [Self; 36] = [
        Self::ProposeOrdinaryLaw,
        Self::EnactOrdinaryLaw,
        Self::AppropriatePublicFunds,
        Self::RatifyTreaty,
        Self::AuthorizeEmergency,
        Self::ConductLegislativeOversight,
        Self::ExecuteLaw,
        Self::ExecuteAppropriation,
        Self::AdministerPublicService,
        Self::DirectPublicAdministration,
        Self::DeclareProvisionalEmergency,
        Self::AdjudicateDispute,
        Self::ConductConstitutionalReview,
        Self::IssueJudicialRemedy,
        Self::AuditPublicExpenditure,
        Self::AuditAuthorityUse,
        Self::InvestigatePublicIntegrity,
        Self::PublishIntegrityFinding,
        Self::ReferForProsecution,
        Self::AdministerElection,
        Self::CertifyMandate,
        Self::AdministerRecall,
        Self::AdministerInitiative,
        Self::AdministerSortition,
        Self::VerifyCivicEligibility,
        Self::CallConstitutionalConvention,
        Self::RatifyStructuralConstitution,
        Self::RatifyFoundationalCovenant,
        Self::WithdrawConstituentDelegation,
        Self::InitiateRightsChallenge,
        Self::RequestProtectedPublicRecord,
        Self::PublishEvidenceAssessment,
        Self::InitiateFutureGenerationsReview,
        Self::PublishFiscalAssessment,
        Self::CertifyPublicServiceQualification,
        Self::InitiatePublicProsecution,
    ];

    /// Powers that represent extraordinary emergency authority must never be
    /// issued without a hard expiry.
    pub fn requires_hard_expiry(self) -> bool {
        matches!(
            self,
            Self::AuthorizeEmergency | Self::DeclareProvisionalEmergency
        )
    }
}

/// Return whether a branch may directly exercise a constitutional power.
pub fn branch_can_exercise(branch: Branch, power: ConstitutionalPower) -> bool {
    match branch {
        Branch::Deliberative => matches!(
            power,
            ConstitutionalPower::ProposeOrdinaryLaw
                | ConstitutionalPower::EnactOrdinaryLaw
                | ConstitutionalPower::AppropriatePublicFunds
                | ConstitutionalPower::RatifyTreaty
                | ConstitutionalPower::AuthorizeEmergency
                | ConstitutionalPower::ConductLegislativeOversight
        ),
        Branch::Stewardship => matches!(
            power,
            ConstitutionalPower::ExecuteLaw
                | ConstitutionalPower::ExecuteAppropriation
                | ConstitutionalPower::AdministerPublicService
                | ConstitutionalPower::DirectPublicAdministration
                | ConstitutionalPower::DeclareProvisionalEmergency
        ),
        Branch::Justice => matches!(
            power,
            ConstitutionalPower::AdjudicateDispute
                | ConstitutionalPower::ConductConstitutionalReview
                | ConstitutionalPower::IssueJudicialRemedy
        ),
        Branch::Integrity => matches!(
            power,
            ConstitutionalPower::AuditPublicExpenditure
                | ConstitutionalPower::AuditAuthorityUse
                | ConstitutionalPower::InvestigatePublicIntegrity
                | ConstitutionalPower::PublishIntegrityFinding
                | ConstitutionalPower::ReferForProsecution
        ),
        Branch::CivicMandate => matches!(
            power,
            ConstitutionalPower::AdministerElection
                | ConstitutionalPower::CertifyMandate
                | ConstitutionalPower::AdministerRecall
                | ConstitutionalPower::AdministerInitiative
                | ConstitutionalPower::AdministerSortition
                | ConstitutionalPower::VerifyCivicEligibility
        ),
    }
}

/// Return whether constituent sovereignty may exercise a constitutional power.
pub fn constituent_can_exercise(power: ConstitutionalPower) -> bool {
    matches!(
        power,
        ConstitutionalPower::CallConstitutionalConvention
            | ConstitutionalPower::RatifyStructuralConstitution
            | ConstitutionalPower::RatifyFoundationalCovenant
            | ConstitutionalPower::WithdrawConstituentDelegation
    )
}

/// Return whether a guardian may directly exercise a constitutional power.
pub fn guardian_can_exercise(guardian: Guardian, power: ConstitutionalPower) -> bool {
    match guardian {
        Guardian::RightsDefender => matches!(
            power,
            ConstitutionalPower::InitiateRightsChallenge
                | ConstitutionalPower::RequestProtectedPublicRecord
        ),
        Guardian::PublicEvidence => {
            matches!(power, ConstitutionalPower::PublishEvidenceAssessment)
        }
        Guardian::FutureGenerations => {
            matches!(power, ConstitutionalPower::InitiateFutureGenerationsReview)
        }
        Guardian::FiscalObservatory => {
            matches!(power, ConstitutionalPower::PublishFiscalAssessment)
        }
        Guardian::PublicService => {
            matches!(
                power,
                ConstitutionalPower::CertifyPublicServiceQualification
            )
        }
        Guardian::ProsecutionService => {
            matches!(power, ConstitutionalPower::InitiatePublicProsecution)
        }
    }
}

/// Deny-by-default constitutional authorization check.
pub fn principal_can_exercise(principal: AuthorityPrincipal, power: ConstitutionalPower) -> bool {
    match principal {
        AuthorityPrincipal::ConstituentSovereignty => constituent_can_exercise(power),
        AuthorityPrincipal::Branch(branch) => branch_can_exercise(branch, power),
        AuthorityPrincipal::Guardian(guardian) => guardian_can_exercise(guardian, power),
        AuthorityPrincipal::AutomatedAgent => false,
    }
}

/// Source from which a capability derives its authority.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum CapabilitySource {
    Charter { charter_id: String, version: u32 },
    ConstituentRatification { event_id: String },
    Statute { proposal_id: String },
    JudicialOrder { case_id: String },
    EmergencyProtocol { declaration_id: String },
    Delegation { parent_capability_id: String },
}

/// A typed constitutional capability.
///
/// This structure is transport-neutral. Individual zomes may wrap it in their
/// own Holochain entry helpers once the authority model is integrated.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ConstitutionalCapability {
    pub id: String,
    pub holder: AuthorityPrincipal,
    pub power: ConstitutionalPower,
    pub jurisdiction: String,
    pub source: CapabilitySource,
    /// Microseconds since Unix epoch. Kept as a primitive to avoid HDK coupling.
    pub valid_from_us: i64,
    /// Hard expiry when applicable.
    pub expires_at_us: Option<i64>,
    pub delegable: bool,
    pub delegation_depth_remaining: u8,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ConformanceError {
    EmptyCapabilityId,
    EmptyJurisdiction,
    UnauthorizedPrincipalPower,
    InvalidExpiry,
    MissingRequiredExpiry,
    NonDelegableCapabilityHasDelegationDepth,
}

impl ConstitutionalCapability {
    /// Validate constitutional shape and branch/guardian authority.
    pub fn validate(&self) -> Result<(), ConformanceError> {
        if self.id.trim().is_empty() {
            return Err(ConformanceError::EmptyCapabilityId);
        }
        if self.jurisdiction.trim().is_empty() {
            return Err(ConformanceError::EmptyJurisdiction);
        }
        if !principal_can_exercise(self.holder, self.power) {
            return Err(ConformanceError::UnauthorizedPrincipalPower);
        }
        if let Some(expires_at) = self.expires_at_us
            && expires_at <= self.valid_from_us
        {
            return Err(ConformanceError::InvalidExpiry);
        }
        if self.power.requires_hard_expiry() && self.expires_at_us.is_none() {
            return Err(ConformanceError::MissingRequiredExpiry);
        }
        if !self.delegable && self.delegation_depth_remaining != 0 {
            return Err(ConformanceError::NonDelegableCapabilityHasDelegationDepth);
        }
        Ok(())
    }
}

/// Constitutional class of a collective decision.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DecisionClass {
    /// Fundamental dignity/rights guarantees.
    FundamentalRights,
    /// Foundational covenant or replacement constitution.
    FoundationalCovenant,
    /// Branch structure, election architecture, amendment rules, etc.
    StructuralConstitution,
    /// Elections, recalls, initiatives, and other civic mandate decisions.
    CivicMandate,
    /// Ordinary public policy and governance.
    OrdinaryGovernance,
    /// Cooperative or organizational economic decisions where participants may
    /// explicitly choose bounded economic weighting.
    CooperativeEconomic,
    /// Narrow technical/operational decisions delegated by a lawful authority.
    TechnicalOperation,
}

impl DecisionClass {
    /// Whether the decision must preserve equal base civic standing.
    pub fn requires_equal_civic_weight(self) -> bool {
        matches!(
            self,
            Self::FundamentalRights
                | Self::FoundationalCovenant
                | Self::StructuralConstitution
                | Self::CivicMandate
        )
    }
}

/// Basis used to weight an eligible participant's vote.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VoteWeightBasis {
    /// One equal base vote per eligible civic participant.
    EqualCivic,
    Matl,
    Stake,
    Phi,
    Participation,
    DomainReputation,
    QuadraticCredits,
    CompositeMerit,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum VoteWeightError {
    UnequalWeightForbidden {
        decision_class: DecisionClass,
        attempted_basis: VoteWeightBasis,
    },
}

/// Validate only the constitutional equality constraint on vote weighting.
///
/// This does not decide whether a weighting system is otherwise wise or valid.
/// It establishes a narrow invariant: MATL, stake, Phi, reputation,
/// participation, quadratic credits, or composite merit must not change the
/// base weight of a person's vote in fundamental civic/constituent decisions.
pub fn validate_vote_weight_basis(
    decision_class: DecisionClass,
    basis: VoteWeightBasis,
) -> Result<(), VoteWeightError> {
    if decision_class.requires_equal_civic_weight() && basis != VoteWeightBasis::EqualCivic {
        return Err(VoteWeightError::UnequalWeightForbidden {
            decision_class,
            attempted_basis: basis,
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn capability(
        holder: AuthorityPrincipal,
        power: ConstitutionalPower,
    ) -> ConstitutionalCapability {
        ConstitutionalCapability {
            id: "cap-1".into(),
            holder,
            power,
            jurisdiction: "test-jurisdiction".into(),
            source: CapabilitySource::Charter {
                charter_id: "charter-1".into(),
                version: 1,
            },
            valid_from_us: 1,
            expires_at_us: None,
            delegable: false,
            delegation_depth_remaining: 0,
        }
    }

    #[test]
    fn constitutional_model_has_exactly_five_named_branches() {
        assert_eq!(
            Branch::ALL,
            [
                Branch::Deliberative,
                Branch::Stewardship,
                Branch::Justice,
                Branch::Integrity,
                Branch::CivicMandate,
            ]
        );
    }

    #[test]
    fn constituent_sovereignty_is_not_a_branch() {
        let constituent = AuthorityPrincipal::ConstituentSovereignty;
        assert!(!matches!(constituent, AuthorityPrincipal::Branch(_)));
        assert!(constituent_can_exercise(
            ConstitutionalPower::RatifyFoundationalCovenant
        ));
    }

    #[test]
    fn stewardship_cannot_certify_its_own_mandate() {
        assert!(!branch_can_exercise(
            Branch::Stewardship,
            ConstitutionalPower::CertifyMandate
        ));
    }

    #[test]
    fn civic_mandate_cannot_enact_substantive_law() {
        assert!(!branch_can_exercise(
            Branch::CivicMandate,
            ConstitutionalPower::EnactOrdinaryLaw
        ));
    }

    #[test]
    fn integrity_can_refer_but_cannot_adjudicate() {
        assert!(branch_can_exercise(
            Branch::Integrity,
            ConstitutionalPower::ReferForProsecution
        ));
        assert!(!branch_can_exercise(
            Branch::Integrity,
            ConstitutionalPower::AdjudicateDispute
        ));
    }

    #[test]
    fn justice_cannot_appropriate_or_execute_public_funds() {
        assert!(!branch_can_exercise(
            Branch::Justice,
            ConstitutionalPower::AppropriatePublicFunds
        ));
        assert!(!branch_can_exercise(
            Branch::Justice,
            ConstitutionalPower::ExecuteAppropriation
        ));
    }

    #[test]
    fn constituted_branches_cannot_ratify_foundational_covenant() {
        for branch in Branch::ALL {
            assert!(!branch_can_exercise(
                branch,
                ConstitutionalPower::RatifyFoundationalCovenant
            ));
        }
    }

    #[test]
    fn automated_agents_hold_no_constitutional_power() {
        for power in ConstitutionalPower::ALL {
            assert!(!principal_can_exercise(
                AuthorityPrincipal::AutomatedAgent,
                power
            ));
        }
    }

    #[test]
    fn fundamental_civic_decisions_require_equal_civic_weight() {
        let protected_classes = [
            DecisionClass::FundamentalRights,
            DecisionClass::FoundationalCovenant,
            DecisionClass::StructuralConstitution,
            DecisionClass::CivicMandate,
        ];
        let forbidden = [
            VoteWeightBasis::Matl,
            VoteWeightBasis::Stake,
            VoteWeightBasis::Phi,
            VoteWeightBasis::Participation,
            VoteWeightBasis::DomainReputation,
            VoteWeightBasis::QuadraticCredits,
            VoteWeightBasis::CompositeMerit,
        ];

        for class in protected_classes {
            assert!(validate_vote_weight_basis(class, VoteWeightBasis::EqualCivic).is_ok());
            for basis in forbidden {
                assert!(validate_vote_weight_basis(class, basis).is_err());
            }
        }
    }

    #[test]
    fn ordinary_governance_may_choose_non_constituent_weighting() {
        assert!(
            validate_vote_weight_basis(
                DecisionClass::OrdinaryGovernance,
                VoteWeightBasis::CompositeMerit
            )
            .is_ok()
        );
    }

    #[test]
    fn emergency_authority_requires_hard_expiry() {
        let mut cap = capability(
            AuthorityPrincipal::Branch(Branch::Stewardship),
            ConstitutionalPower::DeclareProvisionalEmergency,
        );
        assert_eq!(cap.validate(), Err(ConformanceError::MissingRequiredExpiry));

        cap.expires_at_us = Some(10);
        assert!(cap.validate().is_ok());
    }

    #[test]
    fn capability_rejects_cross_branch_power_escalation() {
        let cap = capability(
            AuthorityPrincipal::Branch(Branch::Stewardship),
            ConstitutionalPower::AdministerElection,
        );
        assert_eq!(
            cap.validate(),
            Err(ConformanceError::UnauthorizedPrincipalPower)
        );
    }

    #[test]
    fn nondelegable_capability_cannot_claim_delegation_depth() {
        let mut cap = capability(
            AuthorityPrincipal::Branch(Branch::Justice),
            ConstitutionalPower::AdjudicateDispute,
        );
        cap.delegation_depth_remaining = 1;
        assert_eq!(
            cap.validate(),
            Err(ConformanceError::NonDelegableCapabilityHasDelegationDepth)
        );
    }
}
