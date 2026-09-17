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
//! - sovereign powers are allow-listed and exclusive by constitutional owner;
//! - due-process and oversight entitlements are separate from sovereign power;
//! - extraordinary emergency powers require explicit expiry;
//! - delegated constitutional authority may narrow but never amplify its parent;
//! - delegated authority fails closed unless validated with its parent;
//! - issued authority is bound to a concrete holder identity, not only a class.

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

/// Constitutional class of a concrete authority holder.
///
/// This answers "what kind of constitutional actor is this?". Actual issued
/// capabilities and entitlement grants additionally carry a non-empty
/// `holder_id` so signatures, revocation, conflicts, and audit trails can bind
/// authority to a specific office/institution/constituent process rather than
/// to every member of the class.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AuthorityPrincipal {
    /// Citizens and constituent communities acting through a valid constituent
    /// process. This is the source of the constitution, not a branch.
    ConstituentSovereignty,
    /// One of the five constituted branches.
    Branch(Branch),
    /// An independent guardian with narrowly enumerated authority.
    Guardian(Guardian),
    /// Software/AI acting as an automated principal class.
    ///
    /// Automated agents may receive bounded operational capabilities elsewhere,
    /// but they do not directly hold constitutional powers or entitlements here.
    AutomatedAgent,
}

/// Sovereign or constituted powers that require explicit constitutional
/// allocation to one owner class.
///
/// Due-process and oversight access rights are intentionally represented by
/// [`ConstitutionalEntitlement`] instead of being hidden inside this enum.
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
    PublishEvidenceAssessment,
    InitiateFutureGenerationsReview,
    PublishFiscalAssessment,
    CertifyPublicServiceQualification,
    InitiatePublicProsecution,
}

impl ConstitutionalPower {
    /// Exhaustive list used by conformance tests.
    pub const ALL: [Self; 35] = [
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
        Self::PublishEvidenceAssessment,
        Self::InitiateFutureGenerationsReview,
        Self::PublishFiscalAssessment,
        Self::CertifyPublicServiceQualification,
        Self::InitiatePublicProsecution,
    ];

    pub fn requires_hard_expiry(self) -> bool {
        matches!(
            self,
            Self::AuthorizeEmergency | Self::DeclareProvisionalEmergency
        )
    }

    /// Powers whose constitutional owner must exercise them directly rather
    /// than creating a delegated constitutional chain.
    ///
    /// This does not prohibit ordinary staff from assisting the lawful owner;
    /// it prohibits transforming the constitutional source of authority into a
    /// transferable sovereign capability.
    pub fn is_intrinsically_nondelegable(self) -> bool {
        matches!(
            self,
            Self::AuthorizeEmergency
                | Self::DeclareProvisionalEmergency
                | Self::ConductConstitutionalReview
                | Self::IssueJudicialRemedy
                | Self::CertifyMandate
                | Self::CallConstitutionalConvention
                | Self::RatifyStructuralConstitution
                | Self::RatifyFoundationalCovenant
                | Self::WithdrawConstituentDelegation
        )
    }
}

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

pub fn constituent_can_exercise(power: ConstitutionalPower) -> bool {
    matches!(
        power,
        ConstitutionalPower::CallConstitutionalConvention
            | ConstitutionalPower::RatifyStructuralConstitution
            | ConstitutionalPower::RatifyFoundationalCovenant
            | ConstitutionalPower::WithdrawConstituentDelegation
    )
}

pub fn guardian_can_exercise(guardian: Guardian, power: ConstitutionalPower) -> bool {
    match guardian {
        Guardian::RightsDefender => {
            matches!(power, ConstitutionalPower::InitiateRightsChallenge)
        }
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
            matches!(power, ConstitutionalPower::CertifyPublicServiceQualification)
        }
        Guardian::ProsecutionService => {
            matches!(power, ConstitutionalPower::InitiatePublicProsecution)
        }
    }
}

pub fn principal_can_exercise(
    principal: AuthorityPrincipal,
    power: ConstitutionalPower,
) -> bool {
    match principal {
        AuthorityPrincipal::ConstituentSovereignty => constituent_can_exercise(power),
        AuthorityPrincipal::Branch(branch) => branch_can_exercise(branch, power),
        AuthorityPrincipal::Guardian(guardian) => guardian_can_exercise(guardian, power),
        AuthorityPrincipal::AutomatedAgent => false,
    }
}

/// Shared constitutional entitlements used for due process, oversight, and
/// contestability. Unlike sovereign powers, an entitlement may lawfully be
/// granted to more than one independent principal.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ConstitutionalEntitlement {
    RequestLawfulRecord,
    AccessSubmittedEvidence,
    ReceiveDecisionNotice,
    ObtainDecisionReasons,
    SubmitEvidence,
    ChallengePublicAction,
    SeekJudicialReview,
    PublishProtectedOversightReport,
    ReceiveProtectedDisclosure,
}

impl ConstitutionalEntitlement {
    pub const ALL: [Self; 9] = [
        Self::RequestLawfulRecord,
        Self::AccessSubmittedEvidence,
        Self::ReceiveDecisionNotice,
        Self::ObtainDecisionReasons,
        Self::SubmitEvidence,
        Self::ChallengePublicAction,
        Self::SeekJudicialReview,
        Self::PublishProtectedOversightReport,
        Self::ReceiveProtectedDisclosure,
    ];
}

/// Source from which a constitutional power or entitlement derives authority.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum CapabilitySource {
    Charter {
        charter_id: String,
        version: u32,
    },
    ConstituentRatification {
        event_id: String,
    },
    Statute {
        proposal_id: String,
    },
    JudicialOrder {
        case_id: String,
    },
    EmergencyProtocol {
        declaration_id: String,
    },
    Delegation {
        parent_capability_id: String,
    },
}

impl CapabilitySource {
    pub fn is_well_formed(&self) -> bool {
        match self {
            Self::Charter { charter_id, .. } => !charter_id.trim().is_empty(),
            Self::ConstituentRatification { event_id } => !event_id.trim().is_empty(),
            Self::Statute { proposal_id } => !proposal_id.trim().is_empty(),
            Self::JudicialOrder { case_id } => !case_id.trim().is_empty(),
            Self::EmergencyProtocol { declaration_id } => !declaration_id.trim().is_empty(),
            Self::Delegation {
                parent_capability_id,
            } => !parent_capability_id.trim().is_empty(),
        }
    }

    pub fn is_delegation(&self) -> bool {
        matches!(self, Self::Delegation { .. })
    }
}

/// A typed sovereign constitutional capability.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ConstitutionalCapability {
    pub id: String,
    /// Stable identifier of the concrete institution/office/constituent process
    /// that holds this capability.
    pub holder_id: String,
    /// Constitutional class of `holder_id`.
    pub holder: AuthorityPrincipal,
    pub power: ConstitutionalPower,
    pub jurisdiction: String,
    pub source: CapabilitySource,
    pub valid_from_us: i64,
    pub expires_at_us: Option<i64>,
    pub delegable: bool,
    pub delegation_depth_remaining: u8,
}

pub type SovereignCapability = ConstitutionalCapability;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ConformanceError {
    EmptyCapabilityId,
    EmptyHolderId,
    EmptyJurisdiction,
    InvalidSource,
    UnauthorizedPrincipalPower,
    InvalidExpiry,
    MissingRequiredExpiry,
    DelegationRequiresParentValidation,
    IntrinsicPowerCannotBeDelegated,
    NonDelegableCapabilityHasDelegationDepth,
    DelegableCapabilityHasNoDelegationDepth,
}

impl ConstitutionalCapability {
    fn validate_inner(&self, allow_delegation_source: bool) -> Result<(), ConformanceError> {
        if self.id.trim().is_empty() {
            return Err(ConformanceError::EmptyCapabilityId);
        }
        if self.holder_id.trim().is_empty() {
            return Err(ConformanceError::EmptyHolderId);
        }
        if self.jurisdiction.trim().is_empty() {
            return Err(ConformanceError::EmptyJurisdiction);
        }
        if !self.source.is_well_formed() {
            return Err(ConformanceError::InvalidSource);
        }
        if self.source.is_delegation() && !allow_delegation_source {
            return Err(ConformanceError::DelegationRequiresParentValidation);
        }
        if !principal_can_exercise(self.holder, self.power) {
            return Err(ConformanceError::UnauthorizedPrincipalPower);
        }
        if let Some(expires_at) = self.expires_at_us {
            if expires_at <= self.valid_from_us {
                return Err(ConformanceError::InvalidExpiry);
            }
        }
        if self.power.requires_hard_expiry() && self.expires_at_us.is_none() {
            return Err(ConformanceError::MissingRequiredExpiry);
        }
        if self.power.is_intrinsically_nondelegable()
            && (self.delegable
                || self.delegation_depth_remaining != 0
                || self.source.is_delegation())
        {
            return Err(ConformanceError::IntrinsicPowerCannotBeDelegated);
        }
        if !self.delegable && self.delegation_depth_remaining != 0 {
            return Err(ConformanceError::NonDelegableCapabilityHasDelegationDepth);
        }
        if self.delegable && self.delegation_depth_remaining == 0 {
            return Err(ConformanceError::DelegableCapabilityHasNoDelegationDepth);
        }
        Ok(())
    }

    /// Validate a root/non-delegated constitutional capability.
    /// A delegated source fails closed here and must use [`validate_delegation`].
    pub fn validate(&self) -> Result<(), ConformanceError> {
        self.validate_inner(false)
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InformationSensitivity {
    Public,
    Protected,
    Confidential,
    Restricted,
}

/// A scoped grant of a due-process or oversight entitlement.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ConstitutionalEntitlementGrant {
    pub id: String,
    pub holder_id: String,
    pub holder: AuthorityPrincipal,
    pub entitlement: ConstitutionalEntitlement,
    pub jurisdiction: String,
    pub source: CapabilitySource,
    pub purpose: String,
    pub scope: String,
    pub sensitivity: InformationSensitivity,
    pub valid_from_us: i64,
    pub expires_at_us: Option<i64>,
    pub review_path: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum EntitlementError {
    EmptyGrantId,
    EmptyHolderId,
    EmptyJurisdiction,
    EmptyPurpose,
    EmptyScope,
    EmptyReviewPath,
    InvalidSource,
    DelegatedEntitlementUnsupported,
    AutomatedAgentCannotHoldConstitutionalEntitlement,
    InvalidExpiry,
}

impl ConstitutionalEntitlementGrant {
    pub fn validate(&self) -> Result<(), EntitlementError> {
        if self.id.trim().is_empty() {
            return Err(EntitlementError::EmptyGrantId);
        }
        if self.holder_id.trim().is_empty() {
            return Err(EntitlementError::EmptyHolderId);
        }
        if self.jurisdiction.trim().is_empty() {
            return Err(EntitlementError::EmptyJurisdiction);
        }
        if self.purpose.trim().is_empty() {
            return Err(EntitlementError::EmptyPurpose);
        }
        if self.scope.trim().is_empty() {
            return Err(EntitlementError::EmptyScope);
        }
        if self.review_path.trim().is_empty() {
            return Err(EntitlementError::EmptyReviewPath);
        }
        if !self.source.is_well_formed() {
            return Err(EntitlementError::InvalidSource);
        }
        if self.source.is_delegation() {
            return Err(EntitlementError::DelegatedEntitlementUnsupported);
        }
        if matches!(self.holder, AuthorityPrincipal::AutomatedAgent) {
            return Err(EntitlementError::AutomatedAgentCannotHoldConstitutionalEntitlement);
        }
        if let Some(expires_at) = self.expires_at_us {
            if expires_at <= self.valid_from_us {
                return Err(EntitlementError::InvalidExpiry);
            }
        }
        Ok(())
    }
}

/// Result from the trusted jurisdiction resolver used by delegation validation.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum JurisdictionRelation {
    Same,
    ChildWithinParent,
    BroaderOrUnrelated,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ParentCapabilityState {
    Active,
    Expired,
    Revoked,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum DelegationError {
    InvalidParent(ConformanceError),
    InvalidChild(ConformanceError),
    ParentNotActive(ParentCapabilityState),
    ParentNotDelegable,
    ChildReusesParentCapabilityId,
    WrongParentReference,
    PowerChanged,
    JurisdictionExpandedOrUnresolved,
    ChildStartsBeforeParent,
    ChildOutlivesParent,
    DelegationDepthNotReduced,
}

/// Validate that `child` is a strict attenuation of `parent`.
///
/// The caller supplies the jurisdiction relation from a trusted jurisdiction
/// graph. If `parent` is itself delegated, its own edge must also be validated;
/// pairwise validation composes from delegation root to leaf.
pub fn validate_delegation(
    parent: &ConstitutionalCapability,
    child: &ConstitutionalCapability,
    jurisdiction_relation: JurisdictionRelation,
    parent_state: ParentCapabilityState,
) -> Result<(), DelegationError> {
    parent
        .validate_inner(true)
        .map_err(DelegationError::InvalidParent)?;
    child
        .validate_inner(true)
        .map_err(DelegationError::InvalidChild)?;

    if parent_state != ParentCapabilityState::Active {
        return Err(DelegationError::ParentNotActive(parent_state));
    }
    if !parent.delegable || parent.delegation_depth_remaining == 0 {
        return Err(DelegationError::ParentNotDelegable);
    }
    if child.id == parent.id {
        return Err(DelegationError::ChildReusesParentCapabilityId);
    }
    match &child.source {
        CapabilitySource::Delegation {
            parent_capability_id,
        } if parent_capability_id == &parent.id => {}
        _ => return Err(DelegationError::WrongParentReference),
    }
    if child.power != parent.power {
        return Err(DelegationError::PowerChanged);
    }
    match jurisdiction_relation {
        JurisdictionRelation::Same if child.jurisdiction != parent.jurisdiction => {
            return Err(DelegationError::JurisdictionExpandedOrUnresolved);
        }
        JurisdictionRelation::BroaderOrUnrelated => {
            return Err(DelegationError::JurisdictionExpandedOrUnresolved);
        }
        JurisdictionRelation::Same | JurisdictionRelation::ChildWithinParent => {}
    }
    if child.valid_from_us < parent.valid_from_us {
        return Err(DelegationError::ChildStartsBeforeParent);
    }
    if let Some(parent_expiry) = parent.expires_at_us {
        match child.expires_at_us {
            Some(child_expiry) if child_expiry <= parent_expiry => {}
            _ => return Err(DelegationError::ChildOutlivesParent),
        }
    }
    if child.delegation_depth_remaining >= parent.delegation_depth_remaining {
        return Err(DelegationError::DelegationDepthNotReduced);
    }

    Ok(())
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DecisionClass {
    FundamentalRights,
    FoundationalCovenant,
    StructuralConstitution,
    CivicMandate,
    OrdinaryGovernance,
    CooperativeEconomic,
    TechnicalOperation,
}

impl DecisionClass {
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

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VoteWeightBasis {
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
            holder_id: "subject-1".into(),
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
    fn automated_agents_hold_no_sovereign_constitutional_power() {
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
        assert!(validate_vote_weight_basis(
            DecisionClass::OrdinaryGovernance,
            VoteWeightBasis::CompositeMerit
        )
        .is_ok());
    }

    #[test]
    fn emergency_authority_requires_hard_expiry_and_is_nondelegable() {
        let mut cap = capability(
            AuthorityPrincipal::Branch(Branch::Stewardship),
            ConstitutionalPower::DeclareProvisionalEmergency,
        );
        assert_eq!(cap.validate(), Err(ConformanceError::MissingRequiredExpiry));

        cap.expires_at_us = Some(10);
        assert!(cap.validate().is_ok());

        cap.delegable = true;
        cap.delegation_depth_remaining = 1;
        assert_eq!(
            cap.validate(),
            Err(ConformanceError::IntrinsicPowerCannotBeDelegated)
        );
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
            AuthorityPrincipal::Branch(Branch::Integrity),
            ConstitutionalPower::AuditAuthorityUse,
        );
        cap.delegation_depth_remaining = 1;
        assert_eq!(
            cap.validate(),
            Err(ConformanceError::NonDelegableCapabilityHasDelegationDepth)
        );
    }

    #[test]
    fn empty_provenance_identifier_is_rejected() {
        let mut cap = capability(
            AuthorityPrincipal::Branch(Branch::Integrity),
            ConstitutionalPower::AuditAuthorityUse,
        );
        cap.source = CapabilitySource::Charter {
            charter_id: "".into(),
            version: 1,
        };
        assert_eq!(cap.validate(), Err(ConformanceError::InvalidSource));
    }

    #[test]
    fn concrete_holder_identity_is_required() {
        let mut cap = capability(
            AuthorityPrincipal::Branch(Branch::Integrity),
            ConstitutionalPower::AuditAuthorityUse,
        );
        cap.holder_id.clear();
        assert_eq!(cap.validate(), Err(ConformanceError::EmptyHolderId));
    }
}
