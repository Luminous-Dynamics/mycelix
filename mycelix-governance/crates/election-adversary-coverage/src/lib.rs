//! ADV-002 threat-class coverage for the Mycelix public-election assurance program.
//!
//! This crate does not claim that represented adversaries are mitigated. It proves
//! that every adversary required by ELECT-001 is mechanically routed to executable
//! adversarial evidence, an explicit protocol blocker, a residual research gap, or
//! a combination of those routes.

use election_adversarial_corpus::{
    AttackCaseId, PROTOCOL_DEPENDENT_GAPS, ProtocolGapId, REQUIRED_EXECUTABLE_CASES,
};
use election_integrity_types::{AdversaryClass, REQUIRED_ADVERSARY_CLASSES};

pub const ADVERSARY_COVERAGE_PROFILE_ID: &str = "mycelix-public-election-adversary-coverage-v1";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResidualAdversaryGapId {
    CompositeNationStateCampaigns,
    TrusteeThresholdRobustness,
    EndpointCompromiseResistance,
    VotingDeviceSupplyChainIntegrity,
    ElectionLivenessUnderPartition,
    CredentialTheftRecovery,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdversaryCoverageSpecV1 {
    pub adversary: AdversaryClass,
    pub primary_executable_attack: Option<AttackCaseId>,
    pub primary_protocol_gap: Option<ProtocolGapId>,
    pub residual_gap: Option<ResidualAdversaryGapId>,
}

pub const REQUIRED_ADVERSARY_COVERAGE: [AdversaryCoverageSpecV1; 12] = [
    AdversaryCoverageSpecV1 {
        adversary: AdversaryClass::NationState,
        primary_executable_attack: Some(AttackCaseId::SplitViewEquivocation),
        primary_protocol_gap: Some(ProtocolGapId::BallotSecrecyAgainstConcreteTranscript),
        residual_gap: Some(ResidualAdversaryGapId::CompositeNationStateCampaigns),
    },
    AdversaryCoverageSpecV1 {
        adversary: AdversaryClass::MaliciousElectionOfficial,
        primary_executable_attack: Some(AttackCaseId::WitnessControlDomainCollapse),
        primary_protocol_gap: None,
        residual_gap: None,
    },
    AdversaryCoverageSpecV1 {
        adversary: AdversaryClass::ColludingTrusteeSubset,
        primary_executable_attack: None,
        primary_protocol_gap: Some(ProtocolGapId::TalliedAsRecorded),
        residual_gap: Some(ResidualAdversaryGapId::TrusteeThresholdRobustness),
    },
    AdversaryCoverageSpecV1 {
        adversary: AdversaryClass::MaliciousVoter,
        primary_executable_attack: Some(AttackCaseId::ConflictingNullifier),
        primary_protocol_gap: Some(ProtocolGapId::CryptographicEligibilitySoundness),
        residual_gap: None,
    },
    AdversaryCoverageSpecV1 {
        adversary: AdversaryClass::CompromisedVotingDevice,
        primary_executable_attack: None,
        primary_protocol_gap: Some(ProtocolGapId::CastAsIntended),
        residual_gap: Some(ResidualAdversaryGapId::EndpointCompromiseResistance),
    },
    AdversaryCoverageSpecV1 {
        adversary: AdversaryClass::CompromisedScanner,
        primary_executable_attack: Some(AttackCaseId::CvrDuplicateOrMissing),
        primary_protocol_gap: None,
        residual_gap: None,
    },
    AdversaryCoverageSpecV1 {
        adversary: AdversaryClass::SupplyChainCompromise,
        primary_executable_attack: Some(AttackCaseId::VerifierBuilderCollapse),
        primary_protocol_gap: None,
        residual_gap: Some(ResidualAdversaryGapId::VotingDeviceSupplyChainIntegrity),
    },
    AdversaryCoverageSpecV1 {
        adversary: AdversaryClass::CompromisedMycelixNodes,
        primary_executable_attack: Some(AttackCaseId::SplitViewEquivocation),
        primary_protocol_gap: None,
        residual_gap: None,
    },
    AdversaryCoverageSpecV1 {
        adversary: AdversaryClass::MaliciousVerifier,
        primary_executable_attack: Some(AttackCaseId::VerifierLineageCollapse),
        primary_protocol_gap: None,
        residual_gap: None,
    },
    AdversaryCoverageSpecV1 {
        adversary: AdversaryClass::NetworkPartition,
        primary_executable_attack: Some(AttackCaseId::OfflineNetworkDependency),
        primary_protocol_gap: None,
        residual_gap: Some(ResidualAdversaryGapId::ElectionLivenessUnderPartition),
    },
    AdversaryCoverageSpecV1 {
        adversary: AdversaryClass::Coercer,
        primary_executable_attack: None,
        primary_protocol_gap: Some(ProtocolGapId::CoercionResistance),
        residual_gap: None,
    },
    AdversaryCoverageSpecV1 {
        adversary: AdversaryClass::StolenCredential,
        primary_executable_attack: None,
        primary_protocol_gap: Some(ProtocolGapId::CryptographicEligibilitySoundness),
        residual_gap: Some(ResidualAdversaryGapId::CredentialTheftRecovery),
    },
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AdversaryCoverageDisposition {
    ExecutableCoveragePresent,
    ExplicitlyBlocked,
    ExecutableCoverageAndBlocker,
    Unaccounted,
}

pub fn classify_adversary_coverage(spec: &AdversaryCoverageSpecV1) -> AdversaryCoverageDisposition {
    let executable = spec.primary_executable_attack.is_some();
    let blocked = spec.primary_protocol_gap.is_some() || spec.residual_gap.is_some();
    match (executable, blocked) {
        (true, false) => AdversaryCoverageDisposition::ExecutableCoveragePresent,
        (false, true) => AdversaryCoverageDisposition::ExplicitlyBlocked,
        (true, true) => AdversaryCoverageDisposition::ExecutableCoverageAndBlocker,
        (false, false) => AdversaryCoverageDisposition::Unaccounted,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AdversaryCoverageViolation {
    MissingAdversary(AdversaryClass),
    DuplicateAdversary(AdversaryClass),
    UnroutedAdversary(AdversaryClass),
    UnknownExecutableAttack(AttackCaseId),
    UnknownProtocolGap(ProtocolGapId),
}

pub fn validate_adversary_coverage_registry() -> Result<(), AdversaryCoverageViolation> {
    for required in REQUIRED_ADVERSARY_CLASSES {
        let count = REQUIRED_ADVERSARY_COVERAGE
            .iter()
            .filter(|spec| spec.adversary == required)
            .count();
        if count == 0 {
            return Err(AdversaryCoverageViolation::MissingAdversary(required));
        }
        if count > 1 {
            return Err(AdversaryCoverageViolation::DuplicateAdversary(required));
        }
    }

    for spec in REQUIRED_ADVERSARY_COVERAGE {
        if classify_adversary_coverage(&spec) == AdversaryCoverageDisposition::Unaccounted {
            return Err(AdversaryCoverageViolation::UnroutedAdversary(
                spec.adversary,
            ));
        }
        if let Some(attack) = spec.primary_executable_attack {
            let registered = REQUIRED_EXECUTABLE_CASES
                .iter()
                .any(|case| case.id == attack);
            if !registered {
                return Err(AdversaryCoverageViolation::UnknownExecutableAttack(attack));
            }
        }
        if let Some(gap) = spec.primary_protocol_gap {
            let registered = PROTOCOL_DEPENDENT_GAPS
                .iter()
                .any(|item| item.id == gap && item.blocks_protocol_security_claim);
            if !registered {
                return Err(AdversaryCoverageViolation::UnknownProtocolGap(gap));
            }
        }
    }

    Ok(())
}

pub fn full_threat_model_security_claim_is_blocked() -> bool {
    REQUIRED_ADVERSARY_COVERAGE
        .iter()
        .any(|spec| spec.primary_protocol_gap.is_some() || spec.residual_gap.is_some())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec_for(adversary: AdversaryClass) -> &'static AdversaryCoverageSpecV1 {
        REQUIRED_ADVERSARY_COVERAGE
            .iter()
            .find(|spec| spec.adversary == adversary)
            .expect("every ELECT-001 adversary must have exactly one ADV-002 route")
    }

    #[test]
    fn all_twelve_required_adversaries_are_accounted_for_exactly_once() {
        assert_eq!(REQUIRED_ADVERSARY_COVERAGE.len(), 12);
        assert_eq!(validate_adversary_coverage_registry(), Ok(()));
    }

    #[test]
    fn colluding_trustees_remain_explicitly_blocked_on_real_threshold_protocol() {
        let spec = spec_for(AdversaryClass::ColludingTrusteeSubset);
        assert_eq!(spec.primary_executable_attack, None);
        assert_eq!(
            spec.primary_protocol_gap,
            Some(ProtocolGapId::TalliedAsRecorded)
        );
        assert_eq!(
            spec.residual_gap,
            Some(ResidualAdversaryGapId::TrusteeThresholdRobustness)
        );
    }

    #[test]
    fn compromised_voting_device_is_not_misrepresented_as_structurally_solved() {
        let spec = spec_for(AdversaryClass::CompromisedVotingDevice);
        assert_eq!(spec.primary_executable_attack, None);
        assert_eq!(
            spec.primary_protocol_gap,
            Some(ProtocolGapId::CastAsIntended)
        );
        assert_eq!(
            spec.residual_gap,
            Some(ResidualAdversaryGapId::EndpointCompromiseResistance)
        );
    }

    #[test]
    fn coercion_and_stolen_credentials_remain_fail_visible() {
        let coercer = spec_for(AdversaryClass::Coercer);
        assert_eq!(
            coercer.primary_protocol_gap,
            Some(ProtocolGapId::CoercionResistance)
        );

        let stolen = spec_for(AdversaryClass::StolenCredential);
        assert_eq!(
            stolen.residual_gap,
            Some(ResidualAdversaryGapId::CredentialTheftRecovery)
        );
    }

    #[test]
    fn current_program_cannot_claim_full_threat_model_security() {
        assert!(full_threat_model_security_claim_is_blocked());
    }
}
