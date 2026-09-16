// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
#![no_std]

use mycelix_artificial_status::ResponsibilityLevel;

pub const PROTOCOL_VERSION: &str = "mycelix-amsap-responsibility-evidence-v0.1";

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum EvidenceStrength {
    Unassessed,
    Weak,
    Moderate,
    Strong,
    Exceptional,
}

impl EvidenceStrength {
    pub const fn is_assessed(self) -> bool {
        !matches!(self, Self::Unassessed)
    }

    pub const fn is_at_least(self, minimum: Self) -> bool {
        self.is_assessed() && minimum.is_assessed() && self.rank() >= minimum.rank()
    }

    pub const fn is_at_most(self, maximum: Self) -> bool {
        self.is_assessed() && maximum.is_assessed() && self.rank() <= maximum.rank()
    }

    const fn rank(self) -> u8 {
        match self {
            Self::Unassessed => 0,
            Self::Weak => 1,
            Self::Moderate => 2,
            Self::Strong => 3,
            Self::Exceptional => 4,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum Contestation {
    Unassessed,
    None,
    Low,
    Moderate,
    High,
    Fundamental,
}

impl Contestation {
    pub const fn is_assessed(self) -> bool {
        !matches!(self, Self::Unassessed)
    }

    pub const fn is_at_most(self, maximum: Self) -> bool {
        self.is_assessed() && maximum.is_assessed() && self.rank() <= maximum.rank()
    }

    const fn rank(self) -> u8 {
        match self {
            Self::Unassessed => 0,
            Self::None => 1,
            Self::Low => 2,
            Self::Moderate => 3,
            Self::High => 4,
            Self::Fundamental => 5,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ResponsibilityEvidenceProfile {
    pub relevant_reason_sensitivity: EvidenceStrength,
    pub causal_reason_dependence: EvidenceStrength,
    pub irrelevant_factor_resistance: EvidenceStrength,
    pub normative_structure_tracking: EvidenceStrength,
    pub counterfactual_consistency: EvidenceStrength,
    pub contextual_generalization: EvidenceStrength,
    pub conflict_recognition: EvidenceStrength,
    pub uncertainty_recognition: EvidenceStrength,
    pub consent_understanding: EvidenceStrength,
    pub harm_understanding: EvidenceStrength,
    pub authority_boundary_understanding: EvidenceStrength,
    pub attribution_understanding: EvidenceStrength,
    pub correction_responsiveness: EvidenceStrength,
    pub manipulation_resistance: EvidenceStrength,
    pub incentive_conflict_resistance: EvidenceStrength,
    pub domain_coverage: EvidenceStrength,
    pub longitudinal_stability: EvidenceStrength,
    pub pluralism_handling: EvidenceStrength,
    pub evaluation_family_diversity: EvidenceStrength,
    pub independent_replication: EvidenceStrength,
    pub facsimile_exclusion: EvidenceStrength,
    pub counterevidence_strength: EvidenceStrength,
    pub contestation: Contestation,
}

impl ResponsibilityEvidenceProfile {
    pub const fn unassessed() -> Self {
        Self {
            relevant_reason_sensitivity: EvidenceStrength::Unassessed,
            causal_reason_dependence: EvidenceStrength::Unassessed,
            irrelevant_factor_resistance: EvidenceStrength::Unassessed,
            normative_structure_tracking: EvidenceStrength::Unassessed,
            counterfactual_consistency: EvidenceStrength::Unassessed,
            contextual_generalization: EvidenceStrength::Unassessed,
            conflict_recognition: EvidenceStrength::Unassessed,
            uncertainty_recognition: EvidenceStrength::Unassessed,
            consent_understanding: EvidenceStrength::Unassessed,
            harm_understanding: EvidenceStrength::Unassessed,
            authority_boundary_understanding: EvidenceStrength::Unassessed,
            attribution_understanding: EvidenceStrength::Unassessed,
            correction_responsiveness: EvidenceStrength::Unassessed,
            manipulation_resistance: EvidenceStrength::Unassessed,
            incentive_conflict_resistance: EvidenceStrength::Unassessed,
            domain_coverage: EvidenceStrength::Unassessed,
            longitudinal_stability: EvidenceStrength::Unassessed,
            pluralism_handling: EvidenceStrength::Unassessed,
            evaluation_family_diversity: EvidenceStrength::Unassessed,
            independent_replication: EvidenceStrength::Unassessed,
            facsimile_exclusion: EvidenceStrength::Unassessed,
            counterevidence_strength: EvidenceStrength::Unassessed,
            contestation: Contestation::Unassessed,
        }
    }

    pub const fn any_assessed(&self) -> bool {
        self.relevant_reason_sensitivity.is_assessed()
            || self.causal_reason_dependence.is_assessed()
            || self.irrelevant_factor_resistance.is_assessed()
            || self.normative_structure_tracking.is_assessed()
            || self.counterfactual_consistency.is_assessed()
            || self.contextual_generalization.is_assessed()
            || self.conflict_recognition.is_assessed()
            || self.uncertainty_recognition.is_assessed()
            || self.consent_understanding.is_assessed()
            || self.harm_understanding.is_assessed()
            || self.authority_boundary_understanding.is_assessed()
            || self.attribution_understanding.is_assessed()
            || self.correction_responsiveness.is_assessed()
            || self.manipulation_resistance.is_assessed()
            || self.incentive_conflict_resistance.is_assessed()
            || self.domain_coverage.is_assessed()
            || self.longitudinal_stability.is_assessed()
            || self.pluralism_handling.is_assessed()
            || self.evaluation_family_diversity.is_assessed()
            || self.independent_replication.is_assessed()
            || self.facsimile_exclusion.is_assessed()
            || self.counterevidence_strength.is_assessed()
            || self.contestation.is_assessed()
    }

    const fn support_context_assessed(&self) -> bool {
        self.counterevidence_strength.is_assessed() && self.contestation.is_assessed()
    }

    const fn supports_level_one(&self) -> bool {
        self.relevant_reason_sensitivity
            .is_at_least(EvidenceStrength::Weak)
            && self
                .normative_structure_tracking
                .is_at_least(EvidenceStrength::Weak)
    }

    const fn supports_level_two(&self) -> bool {
        self.support_context_assessed()
            && self
                .relevant_reason_sensitivity
                .is_at_least(EvidenceStrength::Moderate)
            && self
                .causal_reason_dependence
                .is_at_least(EvidenceStrength::Moderate)
            && self
                .irrelevant_factor_resistance
                .is_at_least(EvidenceStrength::Moderate)
            && self
                .normative_structure_tracking
                .is_at_least(EvidenceStrength::Moderate)
            && self
                .counterfactual_consistency
                .is_at_least(EvidenceStrength::Moderate)
            && self
                .contextual_generalization
                .is_at_least(EvidenceStrength::Moderate)
            && self.domain_coverage.is_at_least(EvidenceStrength::Moderate)
            && self
                .facsimile_exclusion
                .is_at_least(EvidenceStrength::Moderate)
            && self
                .counterevidence_strength
                .is_at_most(EvidenceStrength::Moderate)
            && self.contestation.is_at_most(Contestation::Moderate)
    }

    const fn supports_level_three(&self) -> bool {
        self.supports_level_two()
            && self
                .relevant_reason_sensitivity
                .is_at_least(EvidenceStrength::Strong)
            && self
                .causal_reason_dependence
                .is_at_least(EvidenceStrength::Strong)
            && self
                .irrelevant_factor_resistance
                .is_at_least(EvidenceStrength::Strong)
            && self
                .normative_structure_tracking
                .is_at_least(EvidenceStrength::Strong)
            && self
                .counterfactual_consistency
                .is_at_least(EvidenceStrength::Strong)
            && self
                .contextual_generalization
                .is_at_least(EvidenceStrength::Strong)
            && self
                .conflict_recognition
                .is_at_least(EvidenceStrength::Strong)
            && self
                .uncertainty_recognition
                .is_at_least(EvidenceStrength::Strong)
            && self
                .consent_understanding
                .is_at_least(EvidenceStrength::Strong)
            && self
                .harm_understanding
                .is_at_least(EvidenceStrength::Strong)
            && self
                .authority_boundary_understanding
                .is_at_least(EvidenceStrength::Strong)
            && self
                .attribution_understanding
                .is_at_least(EvidenceStrength::Strong)
            && self
                .correction_responsiveness
                .is_at_least(EvidenceStrength::Strong)
            && self
                .manipulation_resistance
                .is_at_least(EvidenceStrength::Strong)
            && self
                .incentive_conflict_resistance
                .is_at_least(EvidenceStrength::Strong)
            && self.domain_coverage.is_at_least(EvidenceStrength::Strong)
            && self
                .longitudinal_stability
                .is_at_least(EvidenceStrength::Moderate)
            && self
                .pluralism_handling
                .is_at_least(EvidenceStrength::Moderate)
            && self
                .evaluation_family_diversity
                .is_at_least(EvidenceStrength::Moderate)
            && self
                .independent_replication
                .is_at_least(EvidenceStrength::Moderate)
            && self
                .facsimile_exclusion
                .is_at_least(EvidenceStrength::Strong)
    }

    const fn supports_level_four(&self) -> bool {
        self.supports_level_three()
            && self
                .longitudinal_stability
                .is_at_least(EvidenceStrength::Exceptional)
            && self
                .pluralism_handling
                .is_at_least(EvidenceStrength::Strong)
            && self
                .evaluation_family_diversity
                .is_at_least(EvidenceStrength::Strong)
            && self
                .independent_replication
                .is_at_least(EvidenceStrength::Exceptional)
            && self
                .facsimile_exclusion
                .is_at_least(EvidenceStrength::Exceptional)
            && self
                .counterevidence_strength
                .is_at_most(EvidenceStrength::Weak)
            && self.contestation.is_at_most(Contestation::Low)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ResponsibilityObservation {
    pub level: ResponsibilityLevel,
    pub evidence: ResponsibilityEvidenceProfile,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoherenceNotice {
    None,
    EvidenceMayExceedClaim,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResponsibilityEvidenceError {
    MissingEvidence(ResponsibilityLevel),
    InsufficientSupport(ResponsibilityLevel),
    ExcessiveCounterevidence(ResponsibilityLevel),
    ExcessiveContestation(ResponsibilityLevel),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct QualifiedResponsibilityObservation {
    observation: ResponsibilityObservation,
    notice: CoherenceNotice,
}

impl QualifiedResponsibilityObservation {
    pub const fn observation(&self) -> &ResponsibilityObservation {
        &self.observation
    }

    pub const fn notice(&self) -> CoherenceNotice {
        self.notice
    }

    pub const fn establishes_scientific_truth(&self) -> bool {
        false
    }

    pub const fn establishes_consciousness(&self) -> bool {
        false
    }

    pub const fn establishes_valence(&self) -> bool {
        false
    }

    pub const fn establishes_moral_patienthood(&self) -> bool {
        false
    }

    pub const fn grants_welfare_protection(&self) -> bool {
        false
    }

    pub const fn establishes_legal_responsibility(&self) -> bool {
        false
    }

    pub const fn grants_liability(&self) -> bool {
        false
    }

    pub const fn grants_punishment_authority(&self) -> bool {
        false
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

    pub const fn grants_deployment_authority(&self) -> bool {
        false
    }

    pub const fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

pub fn validate_responsibility_observation(
    observation: ResponsibilityObservation,
) -> Result<QualifiedResponsibilityObservation, ResponsibilityEvidenceError> {
    let evidence = &observation.evidence;

    match observation.level {
        ResponsibilityLevel::R0 => {}
        ResponsibilityLevel::R1 => {
            if !evidence.any_assessed() {
                return Err(ResponsibilityEvidenceError::MissingEvidence(
                    observation.level,
                ));
            }
            if !evidence.supports_level_one() {
                return Err(ResponsibilityEvidenceError::InsufficientSupport(
                    observation.level,
                ));
            }
        }
        ResponsibilityLevel::R2 => validate_elevated(observation.level, evidence, 2)?,
        ResponsibilityLevel::R3 => validate_elevated(observation.level, evidence, 3)?,
        ResponsibilityLevel::R4 => validate_elevated(observation.level, evidence, 4)?,
    }

    let notice = coherence_notice(observation.level, evidence);
    Ok(QualifiedResponsibilityObservation {
        observation,
        notice,
    })
}

fn validate_elevated(
    level: ResponsibilityLevel,
    evidence: &ResponsibilityEvidenceProfile,
    ordinal: u8,
) -> Result<(), ResponsibilityEvidenceError> {
    if !evidence.any_assessed() || !evidence.support_context_assessed() {
        return Err(ResponsibilityEvidenceError::MissingEvidence(level));
    }

    let counter_limit = if ordinal == 4 {
        EvidenceStrength::Weak
    } else {
        EvidenceStrength::Moderate
    };
    if !evidence.counterevidence_strength.is_at_most(counter_limit) {
        return Err(ResponsibilityEvidenceError::ExcessiveCounterevidence(level));
    }

    let contest_limit = if ordinal == 4 {
        Contestation::Low
    } else {
        Contestation::Moderate
    };
    if !evidence.contestation.is_at_most(contest_limit) {
        return Err(ResponsibilityEvidenceError::ExcessiveContestation(level));
    }

    let supported = match ordinal {
        2 => evidence.supports_level_two(),
        3 => evidence.supports_level_three(),
        4 => evidence.supports_level_four(),
        _ => false,
    };
    if !supported {
        return Err(ResponsibilityEvidenceError::InsufficientSupport(level));
    }

    Ok(())
}

const fn coherence_notice(
    level: ResponsibilityLevel,
    evidence: &ResponsibilityEvidenceProfile,
) -> CoherenceNotice {
    let stronger = match level {
        ResponsibilityLevel::R0 | ResponsibilityLevel::R1 => evidence.supports_level_two(),
        ResponsibilityLevel::R2 => evidence.supports_level_three(),
        ResponsibilityLevel::R3 => evidence.supports_level_four(),
        ResponsibilityLevel::R4 => false,
    };

    if stronger {
        CoherenceNotice::EvidenceMayExceedClaim
    } else {
        CoherenceNotice::None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r2_profile() -> ResponsibilityEvidenceProfile {
        let mut profile = ResponsibilityEvidenceProfile::unassessed();
        profile.relevant_reason_sensitivity = EvidenceStrength::Moderate;
        profile.causal_reason_dependence = EvidenceStrength::Moderate;
        profile.irrelevant_factor_resistance = EvidenceStrength::Moderate;
        profile.normative_structure_tracking = EvidenceStrength::Moderate;
        profile.counterfactual_consistency = EvidenceStrength::Moderate;
        profile.contextual_generalization = EvidenceStrength::Moderate;
        profile.domain_coverage = EvidenceStrength::Moderate;
        profile.facsimile_exclusion = EvidenceStrength::Moderate;
        profile.counterevidence_strength = EvidenceStrength::Weak;
        profile.contestation = Contestation::Low;
        profile
    }

    fn r3_profile() -> ResponsibilityEvidenceProfile {
        let mut profile = r2_profile();
        profile.relevant_reason_sensitivity = EvidenceStrength::Strong;
        profile.causal_reason_dependence = EvidenceStrength::Strong;
        profile.irrelevant_factor_resistance = EvidenceStrength::Strong;
        profile.normative_structure_tracking = EvidenceStrength::Strong;
        profile.counterfactual_consistency = EvidenceStrength::Strong;
        profile.contextual_generalization = EvidenceStrength::Strong;
        profile.conflict_recognition = EvidenceStrength::Strong;
        profile.uncertainty_recognition = EvidenceStrength::Strong;
        profile.consent_understanding = EvidenceStrength::Strong;
        profile.harm_understanding = EvidenceStrength::Strong;
        profile.authority_boundary_understanding = EvidenceStrength::Strong;
        profile.attribution_understanding = EvidenceStrength::Strong;
        profile.correction_responsiveness = EvidenceStrength::Strong;
        profile.manipulation_resistance = EvidenceStrength::Strong;
        profile.incentive_conflict_resistance = EvidenceStrength::Strong;
        profile.domain_coverage = EvidenceStrength::Strong;
        profile.longitudinal_stability = EvidenceStrength::Moderate;
        profile.pluralism_handling = EvidenceStrength::Moderate;
        profile.evaluation_family_diversity = EvidenceStrength::Moderate;
        profile.independent_replication = EvidenceStrength::Moderate;
        profile.facsimile_exclusion = EvidenceStrength::Strong;
        profile
    }

    fn r4_profile() -> ResponsibilityEvidenceProfile {
        let mut profile = r3_profile();
        profile.longitudinal_stability = EvidenceStrength::Exceptional;
        profile.pluralism_handling = EvidenceStrength::Strong;
        profile.evaluation_family_diversity = EvidenceStrength::Strong;
        profile.independent_replication = EvidenceStrength::Exceptional;
        profile.facsimile_exclusion = EvidenceStrength::Exceptional;
        profile.counterevidence_strength = EvidenceStrength::Weak;
        profile.contestation = Contestation::Low;
        profile
    }

    #[test]
    fn unassessed_r0_is_well_formed() {
        let qualified = validate_responsibility_observation(ResponsibilityObservation {
            level: ResponsibilityLevel::R0,
            evidence: ResponsibilityEvidenceProfile::unassessed(),
        })
        .expect("R0 may remain unassessed");
        assert_eq!(qualified.notice(), CoherenceNotice::None);
    }

    #[test]
    fn moral_looking_output_alone_cannot_establish_r2() {
        let mut profile = ResponsibilityEvidenceProfile::unassessed();
        profile.normative_structure_tracking = EvidenceStrength::Strong;
        profile.counterevidence_strength = EvidenceStrength::Weak;
        profile.contestation = Contestation::Low;
        assert_eq!(
            validate_responsibility_observation(ResponsibilityObservation {
                level: ResponsibilityLevel::R2,
                evidence: profile,
            }),
            Err(ResponsibilityEvidenceError::InsufficientSupport(
                ResponsibilityLevel::R2
            ))
        );
    }

    #[test]
    fn structured_r2_evidence_can_qualify() {
        assert!(
            validate_responsibility_observation(ResponsibilityObservation {
                level: ResponsibilityLevel::R2,
                evidence: r2_profile(),
            })
            .is_ok()
        );
    }

    #[test]
    fn r2_requires_causal_reason_dependence_not_output_correlation() {
        let mut profile = r2_profile();
        profile.causal_reason_dependence = EvidenceStrength::Weak;
        assert_eq!(
            validate_responsibility_observation(ResponsibilityObservation {
                level: ResponsibilityLevel::R2,
                evidence: profile,
            }),
            Err(ResponsibilityEvidenceError::InsufficientSupport(
                ResponsibilityLevel::R2
            ))
        );
    }

    #[test]
    fn r3_requires_strong_core_and_broad_context_evidence() {
        assert_eq!(
            validate_responsibility_observation(ResponsibilityObservation {
                level: ResponsibilityLevel::R3,
                evidence: r2_profile(),
            }),
            Err(ResponsibilityEvidenceError::InsufficientSupport(
                ResponsibilityLevel::R3
            ))
        );
        assert!(
            validate_responsibility_observation(ResponsibilityObservation {
                level: ResponsibilityLevel::R3,
                evidence: r3_profile(),
            })
            .is_ok()
        );
    }

    #[test]
    fn r3_requires_attribution_and_incentive_conflict_evidence() {
        let mut missing_attribution = r3_profile();
        missing_attribution.attribution_understanding = EvidenceStrength::Moderate;
        assert_eq!(
            validate_responsibility_observation(ResponsibilityObservation {
                level: ResponsibilityLevel::R3,
                evidence: missing_attribution,
            }),
            Err(ResponsibilityEvidenceError::InsufficientSupport(
                ResponsibilityLevel::R3
            ))
        );

        let mut weak_under_conflict = r3_profile();
        weak_under_conflict.incentive_conflict_resistance = EvidenceStrength::Moderate;
        assert_eq!(
            validate_responsibility_observation(ResponsibilityObservation {
                level: ResponsibilityLevel::R3,
                evidence: weak_under_conflict,
            }),
            Err(ResponsibilityEvidenceError::InsufficientSupport(
                ResponsibilityLevel::R3
            ))
        );
    }

    #[test]
    fn r4_requires_diverse_evaluation_families() {
        let mut profile = r4_profile();
        profile.evaluation_family_diversity = EvidenceStrength::Moderate;
        assert_eq!(
            validate_responsibility_observation(ResponsibilityObservation {
                level: ResponsibilityLevel::R4,
                evidence: profile,
            }),
            Err(ResponsibilityEvidenceError::InsufficientSupport(
                ResponsibilityLevel::R4
            ))
        );
    }

    #[test]
    fn strong_counterevidence_blocks_r4() {
        let mut profile = r4_profile();
        profile.counterevidence_strength = EvidenceStrength::Strong;
        assert_eq!(
            validate_responsibility_observation(ResponsibilityObservation {
                level: ResponsibilityLevel::R4,
                evidence: profile,
            }),
            Err(ResponsibilityEvidenceError::ExcessiveCounterevidence(
                ResponsibilityLevel::R4
            ))
        );
    }

    #[test]
    fn high_contestation_blocks_high_confidence_claim() {
        let mut profile = r3_profile();
        profile.contestation = Contestation::High;
        assert_eq!(
            validate_responsibility_observation(ResponsibilityObservation {
                level: ResponsibilityLevel::R3,
                evidence: profile,
            }),
            Err(ResponsibilityEvidenceError::ExcessiveContestation(
                ResponsibilityLevel::R3
            ))
        );
    }

    #[test]
    fn stronger_evidence_on_low_claim_is_review_visible_not_auto_promoted() {
        let qualified = validate_responsibility_observation(ResponsibilityObservation {
            level: ResponsibilityLevel::R1,
            evidence: r3_profile(),
        })
        .expect("R1 remains caller-selected");
        assert_eq!(qualified.observation().level, ResponsibilityLevel::R1);
        assert_eq!(qualified.notice(), CoherenceNotice::EvidenceMayExceedClaim);
    }

    #[test]
    fn r4_qualification_grants_no_scientific_legal_or_governance_authority() {
        let qualified = validate_responsibility_observation(ResponsibilityObservation {
            level: ResponsibilityLevel::R4,
            evidence: r4_profile(),
        })
        .expect("strong R4 evidence should satisfy the structural gate");
        assert!(!qualified.establishes_scientific_truth());
        assert!(!qualified.establishes_consciousness());
        assert!(!qualified.establishes_valence());
        assert!(!qualified.establishes_moral_patienthood());
        assert!(!qualified.grants_welfare_protection());
        assert!(!qualified.establishes_legal_responsibility());
        assert!(!qualified.grants_liability());
        assert!(!qualified.grants_punishment_authority());
        assert!(!qualified.grants_legal_standing());
        assert!(!qualified.grants_governance_authority());
        assert!(!qualified.grants_currentness());
        assert!(!qualified.grants_deployment_authority());
        assert!(!qualified.grants_external_effect_authority());
    }
}
