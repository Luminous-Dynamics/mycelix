// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
#![no_std]

/// AMSAP-002 implements only the scientific/observational C/V/A/I/R substrate.
///
/// It deliberately contains no legal-standing or governance-authority type and
/// cannot grant currentness or external-effect authority.
pub const PROTOCOL_VERSION: &str = "mycelix-amsap-artificial-status-v0.1";

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvidenceBand {
    Unassessed,
    Weak,
    Moderate,
    Strong,
    Exceptional,
    Contested,
}

impl EvidenceBand {
    const fn rank(self) -> Option<u8> {
        match self {
            Self::Unassessed => Some(0),
            Self::Weak => Some(1),
            Self::Moderate => Some(2),
            Self::Strong => Some(3),
            Self::Exceptional => Some(4),
            Self::Contested => None,
        }
    }

    pub const fn is_at_least(self, minimum: Self) -> bool {
        match (self.rank(), minimum.rank()) {
            (Some(actual), Some(required)) => actual >= required,
            _ => false,
        }
    }

    pub const fn is_assessed(self) -> bool {
        !matches!(self, Self::Unassessed)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EvidenceProfile {
    pub indicator_coverage: EvidenceBand,
    pub indicator_robustness: EvidenceBand,
    pub link_credibility: EvidenceBand,
    pub causal_support: EvidenceBand,
    pub alternative_exclusion: EvidenceBand,
    pub replication_independence: EvidenceBand,
    pub counterevidence_strength: EvidenceBand,
    pub theory_diversity: EvidenceBand,
    pub context_stability: EvidenceBand,
    pub manipulation_resistance: EvidenceBand,
}

impl EvidenceProfile {
    pub const fn unassessed() -> Self {
        Self {
            indicator_coverage: EvidenceBand::Unassessed,
            indicator_robustness: EvidenceBand::Unassessed,
            link_credibility: EvidenceBand::Unassessed,
            causal_support: EvidenceBand::Unassessed,
            alternative_exclusion: EvidenceBand::Unassessed,
            replication_independence: EvidenceBand::Unassessed,
            counterevidence_strength: EvidenceBand::Unassessed,
            theory_diversity: EvidenceBand::Unassessed,
            context_stability: EvidenceBand::Unassessed,
            manipulation_resistance: EvidenceBand::Unassessed,
        }
    }

    pub const fn any_assessed(&self) -> bool {
        self.indicator_coverage.is_assessed()
            || self.indicator_robustness.is_assessed()
            || self.link_credibility.is_assessed()
            || self.causal_support.is_assessed()
            || self.alternative_exclusion.is_assessed()
            || self.replication_independence.is_assessed()
            || self.counterevidence_strength.is_assessed()
            || self.theory_diversity.is_assessed()
            || self.context_stability.is_assessed()
            || self.manipulation_resistance.is_assessed()
    }

    pub const fn fully_assessed(&self) -> bool {
        self.indicator_coverage.is_assessed()
            && self.indicator_robustness.is_assessed()
            && self.link_credibility.is_assessed()
            && self.causal_support.is_assessed()
            && self.alternative_exclusion.is_assessed()
            && self.replication_independence.is_assessed()
            && self.counterevidence_strength.is_assessed()
            && self.theory_diversity.is_assessed()
            && self.context_stability.is_assessed()
            && self.manipulation_resistance.is_assessed()
    }

    const fn supports_level_two(&self) -> bool {
        self.link_credibility.is_at_least(EvidenceBand::Moderate)
            && self
                .indicator_robustness
                .is_at_least(EvidenceBand::Moderate)
    }

    const fn supports_level_three(&self) -> bool {
        self.supports_level_two()
            && self.causal_support.is_at_least(EvidenceBand::Moderate)
            && self
                .alternative_exclusion
                .is_at_least(EvidenceBand::Moderate)
            && self
                .replication_independence
                .is_at_least(EvidenceBand::Moderate)
    }

    const fn supports_level_four(&self) -> bool {
        self.fully_assessed()
            && self.indicator_coverage.is_at_least(EvidenceBand::Strong)
            && self.indicator_robustness.is_at_least(EvidenceBand::Strong)
            && self.link_credibility.is_at_least(EvidenceBand::Strong)
            && self.causal_support.is_at_least(EvidenceBand::Strong)
            && self.alternative_exclusion.is_at_least(EvidenceBand::Strong)
            && self
                .replication_independence
                .is_at_least(EvidenceBand::Strong)
            && self.theory_diversity.is_at_least(EvidenceBand::Strong)
            && self.context_stability.is_at_least(EvidenceBand::Strong)
            && self
                .manipulation_resistance
                .is_at_least(EvidenceBand::Strong)
            && matches!(
                self.counterevidence_strength,
                EvidenceBand::Weak | EvidenceBand::Moderate
            )
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConsciousnessEvidence {
    C0,
    C1,
    C2,
    C3,
    C4,
}

impl ConsciousnessEvidence {
    const fn level(self) -> u8 {
        match self {
            Self::C0 => 0,
            Self::C1 => 1,
            Self::C2 => 2,
            Self::C3 => 3,
            Self::C4 => 4,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ValenceEvidence {
    V0,
    V1,
    V2,
    V3,
    V4,
}

impl ValenceEvidence {
    const fn level(self) -> u8 {
        match self {
            Self::V0 => 0,
            Self::V1 => 1,
            Self::V2 => 2,
            Self::V3 => 3,
            Self::V4 => 4,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AgencyLevel {
    A0,
    A1,
    A2,
    A3,
    A4,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IdentityContinuity {
    I0,
    I1,
    I2,
    I3,
    I4,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResponsibilityLevel {
    R0,
    R1,
    R2,
    R3,
    R4,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ArtificialStatusObservation {
    pub consciousness: ConsciousnessEvidence,
    pub consciousness_evidence: EvidenceProfile,
    pub valence: ValenceEvidence,
    pub valence_evidence: EvidenceProfile,
    pub agency: AgencyLevel,
    pub identity: IdentityContinuity,
    pub responsibility: ResponsibilityLevel,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ScientificAxis {
    Consciousness,
    Valence,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StatusError {
    MissingEvidence(ScientificAxis),
    InsufficientSupport(ScientificAxis),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct WellFormedArtificialStatusObservation {
    observation: ArtificialStatusObservation,
}

impl WellFormedArtificialStatusObservation {
    pub const fn observation(&self) -> &ArtificialStatusObservation {
        &self.observation
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

pub fn validate_observation(
    observation: ArtificialStatusObservation,
) -> Result<WellFormedArtificialStatusObservation, StatusError> {
    validate_scientific_claim(
        observation.consciousness.level(),
        &observation.consciousness_evidence,
        ScientificAxis::Consciousness,
    )?;
    validate_scientific_claim(
        observation.valence.level(),
        &observation.valence_evidence,
        ScientificAxis::Valence,
    )?;

    Ok(WellFormedArtificialStatusObservation { observation })
}

fn validate_scientific_claim(
    level: u8,
    evidence: &EvidenceProfile,
    axis: ScientificAxis,
) -> Result<(), StatusError> {
    if level == 0 {
        return Ok(());
    }
    if !evidence.any_assessed() {
        return Err(StatusError::MissingEvidence(axis));
    }

    let supported = match level {
        1 => true,
        2 => evidence.supports_level_two(),
        3 => evidence.supports_level_three(),
        4 => evidence.supports_level_four(),
        _ => false,
    };

    if supported {
        Ok(())
    } else {
        Err(StatusError::InsufficientSupport(axis))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn moderate_profile() -> EvidenceProfile {
        EvidenceProfile {
            indicator_coverage: EvidenceBand::Moderate,
            indicator_robustness: EvidenceBand::Moderate,
            link_credibility: EvidenceBand::Moderate,
            causal_support: EvidenceBand::Moderate,
            alternative_exclusion: EvidenceBand::Moderate,
            replication_independence: EvidenceBand::Moderate,
            counterevidence_strength: EvidenceBand::Weak,
            theory_diversity: EvidenceBand::Moderate,
            context_stability: EvidenceBand::Moderate,
            manipulation_resistance: EvidenceBand::Moderate,
        }
    }

    fn strong_profile() -> EvidenceProfile {
        EvidenceProfile {
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
        }
    }

    fn observation() -> ArtificialStatusObservation {
        ArtificialStatusObservation {
            consciousness: ConsciousnessEvidence::C0,
            consciousness_evidence: EvidenceProfile::unassessed(),
            valence: ValenceEvidence::V0,
            valence_evidence: EvidenceProfile::unassessed(),
            agency: AgencyLevel::A0,
            identity: IdentityContinuity::I0,
            responsibility: ResponsibilityLevel::R0,
        }
    }

    #[test]
    fn unassessed_zero_claims_are_well_formed() {
        assert!(validate_observation(observation()).is_ok());
    }

    #[test]
    fn positive_consciousness_claim_requires_evidence() {
        let mut candidate = observation();
        candidate.consciousness = ConsciousnessEvidence::C1;
        assert_eq!(
            validate_observation(candidate),
            Err(StatusError::MissingEvidence(ScientificAxis::Consciousness))
        );
    }

    #[test]
    fn elevated_claims_require_progressively_stronger_support() {
        let mut candidate = observation();
        candidate.consciousness = ConsciousnessEvidence::C3;
        candidate.consciousness_evidence = moderate_profile();
        assert!(validate_observation(candidate).is_ok());

        candidate.consciousness = ConsciousnessEvidence::C4;
        assert_eq!(
            validate_observation(candidate),
            Err(StatusError::InsufficientSupport(
                ScientificAxis::Consciousness
            ))
        );

        candidate.consciousness_evidence = strong_profile();
        assert!(validate_observation(candidate).is_ok());
    }

    #[test]
    fn contested_link_cannot_satisfy_high_confidence_gate() {
        let mut candidate = observation();
        candidate.valence = ValenceEvidence::V4;
        let mut evidence = strong_profile();
        evidence.link_credibility = EvidenceBand::Contested;
        candidate.valence_evidence = evidence;
        assert_eq!(
            validate_observation(candidate),
            Err(StatusError::InsufficientSupport(ScientificAxis::Valence))
        );
    }

    #[test]
    fn strong_counterevidence_blocks_level_four() {
        let mut candidate = observation();
        candidate.valence = ValenceEvidence::V4;
        let mut evidence = strong_profile();
        evidence.counterevidence_strength = EvidenceBand::Strong;
        candidate.valence_evidence = evidence;
        assert_eq!(
            validate_observation(candidate),
            Err(StatusError::InsufficientSupport(ScientificAxis::Valence))
        );
    }

    #[test]
    fn high_agency_does_not_manufacture_consciousness_or_valence() {
        let mut candidate = observation();
        candidate.agency = AgencyLevel::A4;
        candidate.identity = IdentityContinuity::I4;
        candidate.responsibility = ResponsibilityLevel::R4;
        let qualified = validate_observation(candidate).expect("structurally valid");
        assert_eq!(
            qualified.observation().consciousness,
            ConsciousnessEvidence::C0
        );
        assert_eq!(qualified.observation().valence, ValenceEvidence::V0);
    }

    #[test]
    fn validated_observation_grants_no_authority() {
        let qualified = validate_observation(observation()).expect("structurally valid");
        assert!(!qualified.grants_legal_standing());
        assert!(!qualified.grants_governance_authority());
        assert!(!qualified.grants_currentness());
        assert!(!qualified.grants_external_effect_authority());
    }
}
