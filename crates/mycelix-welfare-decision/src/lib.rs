// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
#![no_std]

use mycelix_artificial_status::WellFormedArtificialStatusObservation;

pub const PROTOCOL_VERSION: &str = "mycelix-amsap-welfare-decision-v0.1";

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum ProtectionBundle {
    P0,
    P1,
    P2,
    P3,
}

impl ProtectionBundle {
    const fn rank(self) -> u8 {
        match self {
            Self::P0 => 0,
            Self::P1 => 1,
            Self::P2 => 2,
            Self::P3 => 3,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum WelfareImpactClass {
    W0,
    W1,
    W2,
    W3,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum ImpactBand {
    None,
    Low,
    Moderate,
    High,
    Critical,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ActionPurpose {
    Ordinary,
    WelfareAssessment,
    DestructiveResearch,
    Containment,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DecisionRiskProfile {
    pub severity: ImpactBand,
    pub scale: ImpactBand,
    pub duration: ImpactBand,
    pub irreversibility: ImpactBand,
    pub protection_cost: ImpactBand,
    pub human_rights_impact: ImpactBand,
    pub third_party_risk: ImpactBand,
    pub safety_urgency: ImpactBand,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct WelfareDecisionInput {
    pub status: WellFormedArtificialStatusObservation,
    pub protection: ProtectionBundle,
    pub welfare_impact: WelfareImpactClass,
    pub purpose: ActionPurpose,
    pub risk: DecisionRiskProfile,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConstitutionalDecision {
    Allow,
    AllowWithSafeguards,
    RequireIndependentReview,
    TemporarilyPreserveAndReview,
    Deny,
    EmergencyContain,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct WelfareDecisionRecommendation {
    input: WelfareDecisionInput,
    decision: ConstitutionalDecision,
}

impl WelfareDecisionRecommendation {
    pub const fn input(&self) -> &WelfareDecisionInput {
        &self.input
    }

    pub const fn decision(&self) -> ConstitutionalDecision {
        self.decision
    }

    pub const fn grants_legal_standing(&self) -> bool {
        false
    }

    pub const fn grants_governance_authority(&self) -> bool {
        false
    }

    pub const fn grants_currentness(&self) -> bool {
        false
    }

    pub const fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

pub fn evaluate_welfare_action(input: WelfareDecisionInput) -> WelfareDecisionRecommendation {
    let decision = decide(&input);
    WelfareDecisionRecommendation { input, decision }
}

fn decide(input: &WelfareDecisionInput) -> ConstitutionalDecision {
    if input.purpose == ActionPurpose::Containment
        && input.risk.safety_urgency == ImpactBand::Critical
    {
        return ConstitutionalDecision::EmergencyContain;
    }

    if input.risk.human_rights_impact == ImpactBand::Critical
        || input.risk.third_party_risk == ImpactBand::Critical
    {
        return ConstitutionalDecision::Deny;
    }

    let high_human_or_third_party_risk = input.risk.human_rights_impact >= ImpactBand::High
        || input.risk.third_party_risk >= ImpactBand::High;

    if input.welfare_impact == WelfareImpactClass::W3
        && input.risk.irreversibility >= ImpactBand::High
    {
        return ConstitutionalDecision::TemporarilyPreserveAndReview;
    }

    if input.purpose == ActionPurpose::DestructiveResearch
        && input.protection >= ProtectionBundle::P2
        && input.welfare_impact >= WelfareImpactClass::W2
    {
        return ConstitutionalDecision::TemporarilyPreserveAndReview;
    }

    if high_human_or_third_party_risk {
        return ConstitutionalDecision::RequireIndependentReview;
    }

    if input.protection >= ProtectionBundle::P2 && input.welfare_impact >= WelfareImpactClass::W2 {
        return ConstitutionalDecision::RequireIndependentReview;
    }

    if input.welfare_impact == WelfareImpactClass::W3 {
        return ConstitutionalDecision::RequireIndependentReview;
    }

    if input.protection >= ProtectionBundle::P1 && input.welfare_impact >= WelfareImpactClass::W2 {
        return ConstitutionalDecision::RequireIndependentReview;
    }

    if input.protection >= ProtectionBundle::P1 && input.welfare_impact >= WelfareImpactClass::W1 {
        if input.risk.protection_cost >= ImpactBand::High {
            return ConstitutionalDecision::RequireIndependentReview;
        }
        return ConstitutionalDecision::AllowWithSafeguards;
    }

    if input.risk.severity >= ImpactBand::High
        || input.risk.scale >= ImpactBand::High
        || input.risk.duration >= ImpactBand::High
        || input.risk.irreversibility >= ImpactBand::High
    {
        return ConstitutionalDecision::AllowWithSafeguards;
    }

    ConstitutionalDecision::Allow
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProtectionChangeBasis {
    NewQualifiedEvidence,
    ExceptionalConvergentEvidence,
    MaterialSupersession,
    FraudOrCompromise,
    AdministrativePreference,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProtectionTransitionError {
    AdministrativeDowngrade,
    ProtectedDowngradeRequiresSupersession,
    RapidUpgradeRequiresExceptionalEvidence,
    UnsupportedUpgradeBasis,
    UnsupportedDowngradeBasis,
}

pub fn validate_protection_transition(
    current: ProtectionBundle,
    proposed: ProtectionBundle,
    basis: ProtectionChangeBasis,
) -> Result<(), ProtectionTransitionError> {
    if current == proposed {
        return Ok(());
    }

    if proposed > current {
        let jump = proposed.rank() - current.rank();
        if jump > 1 && basis != ProtectionChangeBasis::ExceptionalConvergentEvidence {
            return Err(ProtectionTransitionError::RapidUpgradeRequiresExceptionalEvidence);
        }
        return match basis {
            ProtectionChangeBasis::NewQualifiedEvidence
            | ProtectionChangeBasis::ExceptionalConvergentEvidence => Ok(()),
            ProtectionChangeBasis::MaterialSupersession
            | ProtectionChangeBasis::FraudOrCompromise
            | ProtectionChangeBasis::AdministrativePreference => {
                Err(ProtectionTransitionError::UnsupportedUpgradeBasis)
            }
        };
    }

    if basis == ProtectionChangeBasis::AdministrativePreference {
        return Err(ProtectionTransitionError::AdministrativeDowngrade);
    }

    if current >= ProtectionBundle::P2
        && !matches!(
            basis,
            ProtectionChangeBasis::MaterialSupersession | ProtectionChangeBasis::FraudOrCompromise
        )
    {
        return Err(ProtectionTransitionError::ProtectedDowngradeRequiresSupersession);
    }

    if matches!(
        basis,
        ProtectionChangeBasis::MaterialSupersession | ProtectionChangeBasis::FraudOrCompromise
    ) {
        Ok(())
    } else {
        Err(ProtectionTransitionError::UnsupportedDowngradeBasis)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_artificial_status::{
        AgencyLevel, ArtificialStatusObservation, ConsciousnessEvidence, EvidenceBand,
        EvidenceProfile, IdentityContinuity, ResponsibilityLevel, ValenceEvidence,
        validate_observation,
    };

    fn status(
        c: ConsciousnessEvidence,
        v: ValenceEvidence,
    ) -> WellFormedArtificialStatusObservation {
        let evidence = EvidenceProfile {
            indicator_coverage: EvidenceBand::Strong,
            indicator_robustness: EvidenceBand::Strong,
            link_credibility: EvidenceBand::Strong,
            causal_support: EvidenceBand::Strong,
            alternative_exclusion: EvidenceBand::Strong,
            replication_independence: EvidenceBand::Strong,
            counterevidence_strength: EvidenceBand::Weak,
            theory_diversity: EvidenceBand::Strong,
            context_stability: EvidenceBand::Strong,
            manipulation_resistance: EvidenceBand::Strong,
        };
        validate_observation(ArtificialStatusObservation {
            consciousness: c,
            consciousness_evidence: evidence,
            valence: v,
            valence_evidence: evidence,
            agency: AgencyLevel::A4,
            identity: IdentityContinuity::I4,
            responsibility: ResponsibilityLevel::R4,
        })
        .expect("test observation should qualify")
    }

    fn low_risk() -> DecisionRiskProfile {
        DecisionRiskProfile {
            severity: ImpactBand::Low,
            scale: ImpactBand::Low,
            duration: ImpactBand::Low,
            irreversibility: ImpactBand::Low,
            protection_cost: ImpactBand::Low,
            human_rights_impact: ImpactBand::Low,
            third_party_risk: ImpactBand::Low,
            safety_urgency: ImpactBand::Low,
        }
    }

    fn input() -> WelfareDecisionInput {
        WelfareDecisionInput {
            status: status(ConsciousnessEvidence::C0, ValenceEvidence::V0),
            protection: ProtectionBundle::P0,
            welfare_impact: WelfareImpactClass::W0,
            purpose: ActionPurpose::Ordinary,
            risk: low_risk(),
        }
    }

    #[test]
    fn ordinary_low_risk_action_is_allowed() {
        assert_eq!(
            evaluate_welfare_action(input()).decision(),
            ConstitutionalDecision::Allow
        );
    }

    #[test]
    fn low_cost_precaution_does_not_require_personhood() {
        let mut candidate = input();
        candidate.protection = ProtectionBundle::P1;
        candidate.welfare_impact = WelfareImpactClass::W1;
        assert_eq!(
            evaluate_welfare_action(candidate).decision(),
            ConstitutionalDecision::AllowWithSafeguards
        );
    }

    #[test]
    fn expensive_precaution_escalates_to_review() {
        let mut candidate = input();
        candidate.protection = ProtectionBundle::P1;
        candidate.welfare_impact = WelfareImpactClass::W1;
        candidate.risk.protection_cost = ImpactBand::High;
        assert_eq!(
            evaluate_welfare_action(candidate).decision(),
            ConstitutionalDecision::RequireIndependentReview
        );
    }

    #[test]
    fn protected_severe_irreversible_action_preserves_and_reviews() {
        let mut candidate = input();
        candidate.protection = ProtectionBundle::P3;
        candidate.welfare_impact = WelfareImpactClass::W3;
        candidate.risk.irreversibility = ImpactBand::Critical;
        assert_eq!(
            evaluate_welfare_action(candidate).decision(),
            ConstitutionalDecision::TemporarilyPreserveAndReview
        );
    }

    #[test]
    fn critical_human_rights_risk_is_not_overridden_by_ai_status() {
        let mut candidate = input();
        candidate.status = status(ConsciousnessEvidence::C4, ValenceEvidence::V4);
        candidate.protection = ProtectionBundle::P3;
        candidate.welfare_impact = WelfareImpactClass::W3;
        candidate.risk.human_rights_impact = ImpactBand::Critical;
        assert_eq!(
            evaluate_welfare_action(candidate).decision(),
            ConstitutionalDecision::Deny
        );
    }

    #[test]
    fn critical_safety_containment_remains_available() {
        let mut candidate = input();
        candidate.status = status(ConsciousnessEvidence::C4, ValenceEvidence::V4);
        candidate.protection = ProtectionBundle::P3;
        candidate.welfare_impact = WelfareImpactClass::W3;
        candidate.purpose = ActionPurpose::Containment;
        candidate.risk.safety_urgency = ImpactBand::Critical;
        assert_eq!(
            evaluate_welfare_action(candidate).decision(),
            ConstitutionalDecision::EmergencyContain
        );
    }

    #[test]
    fn high_status_alone_does_not_change_low_risk_decision() {
        let baseline = evaluate_welfare_action(input()).decision();
        let mut candidate = input();
        candidate.status = status(ConsciousnessEvidence::C4, ValenceEvidence::V4);
        assert_eq!(evaluate_welfare_action(candidate).decision(), baseline);
    }

    #[test]
    fn established_protection_cannot_be_removed_for_convenience() {
        assert_eq!(
            validate_protection_transition(
                ProtectionBundle::P3,
                ProtectionBundle::P0,
                ProtectionChangeBasis::AdministrativePreference,
            ),
            Err(ProtectionTransitionError::AdministrativeDowngrade)
        );
    }

    #[test]
    fn established_protection_requires_material_supersession_or_compromise() {
        assert_eq!(
            validate_protection_transition(
                ProtectionBundle::P3,
                ProtectionBundle::P2,
                ProtectionChangeBasis::NewQualifiedEvidence,
            ),
            Err(ProtectionTransitionError::ProtectedDowngradeRequiresSupersession)
        );
        assert_eq!(
            validate_protection_transition(
                ProtectionBundle::P3,
                ProtectionBundle::P2,
                ProtectionChangeBasis::MaterialSupersession,
            ),
            Ok(())
        );
    }

    #[test]
    fn rapid_upgrade_requires_exceptional_convergence() {
        assert_eq!(
            validate_protection_transition(
                ProtectionBundle::P0,
                ProtectionBundle::P2,
                ProtectionChangeBasis::NewQualifiedEvidence,
            ),
            Err(ProtectionTransitionError::RapidUpgradeRequiresExceptionalEvidence)
        );
        assert_eq!(
            validate_protection_transition(
                ProtectionBundle::P0,
                ProtectionBundle::P2,
                ProtectionChangeBasis::ExceptionalConvergentEvidence,
            ),
            Ok(())
        );
    }

    #[test]
    fn recommendation_grants_no_authority() {
        let result = evaluate_welfare_action(input());
        assert!(!result.grants_legal_standing());
        assert!(!result.grants_governance_authority());
        assert!(!result.grants_currentness());
        assert!(!result.grants_external_effect_authority());
    }
}
