#![allow(deprecated)]

use mycelix_bridge_common::{
    CivicRequirement, CivicTier, ConsciousnessTier, DimensionWeights, GovernanceRequirement,
    SovereignDimension, SovereignProfile,
    sovereign_gate::governance_requirement_from_civic,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ConversionError {
    InvalidIdentityMinimum,
    InvalidCommunityMinimum,
}

fn checked_threshold(
    value: Option<f64>,
    error: ConversionError,
) -> Result<Option<f64>, ConversionError> {
    match value {
        None => Ok(None),
        Some(value) if value.is_finite() && (0.0..=1.0).contains(&value) => Ok(Some(value)),
        Some(_) => Err(error),
    }
}

fn civic_tier_from_legacy(tier: &ConsciousnessTier) -> CivicTier {
    match tier {
        ConsciousnessTier::Observer => CivicTier::Observer,
        ConsciousnessTier::Participant => CivicTier::Participant,
        ConsciousnessTier::Citizen => CivicTier::Citizen,
        ConsciousnessTier::Steward => CivicTier::Steward,
        ConsciousnessTier::Guardian => CivicTier::Guardian,
    }
}

/// Candidate FIN-SAFE-028 migration rule.
///
/// A legacy scalar minimum is duplicated across both 8D dimensions that map
/// back to that scalar. Native 8D evaluation may therefore be stricter, but
/// can never become more permissive than the represented legacy minimum.
fn try_civic_requirement_from_governance(
    legacy: &GovernanceRequirement,
) -> Result<CivicRequirement, ConversionError> {
    let min_identity = checked_threshold(
        legacy.min_identity,
        ConversionError::InvalidIdentityMinimum,
    )?;
    let min_community = checked_threshold(
        legacy.min_community,
        ConversionError::InvalidCommunityMinimum,
    )?;

    let mut min_dimensions = Vec::with_capacity(4);
    if let Some(minimum) = min_identity {
        min_dimensions.push((SovereignDimension::EpistemicIntegrity, minimum));
        min_dimensions.push((SovereignDimension::NetworkResilience, minimum));
    }
    if let Some(minimum) = min_community {
        min_dimensions.push((SovereignDimension::CivicParticipation, minimum));
        min_dimensions.push((SovereignDimension::SemanticResonance, minimum));
    }

    Ok(CivicRequirement {
        min_tier: civic_tier_from_legacy(&legacy.min_tier),
        min_dimensions,
    })
}

#[test]
fn valid_legacy_requirements_round_trip_exactly_through_existing_fallback_mapping() {
    let tiers = [
        ConsciousnessTier::Observer,
        ConsciousnessTier::Participant,
        ConsciousnessTier::Citizen,
        ConsciousnessTier::Steward,
        ConsciousnessTier::Guardian,
    ];
    let minima = [None, Some(0.0), Some(0.25), Some(0.5), Some(0.75), Some(1.0)];

    for tier in tiers {
        for min_identity in minima {
            for min_community in minima {
                let legacy = GovernanceRequirement {
                    min_tier: tier.clone(),
                    min_identity,
                    min_community,
                };
                let civic = try_civic_requirement_from_governance(&legacy)
                    .expect("valid legacy requirement must convert");
                let round_trip = governance_requirement_from_civic(&civic);

                assert_eq!(round_trip.min_tier, legacy.min_tier);
                assert_eq!(round_trip.min_identity, legacy.min_identity);
                assert_eq!(round_trip.min_community, legacy.min_community);
            }
        }
    }
}

#[test]
fn identity_minimum_is_required_on_both_corresponding_8d_dimensions() {
    let legacy = GovernanceRequirement {
        min_tier: ConsciousnessTier::Observer,
        min_identity: Some(0.5),
        min_community: None,
    };
    let civic = try_civic_requirement_from_governance(&legacy).unwrap();
    let weights = DimensionWeights::governance();

    let mut profile = SovereignProfile {
        epistemic_integrity: 1.0,
        thermodynamic_yield: 1.0,
        network_resilience: 0.49,
        economic_velocity: 1.0,
        civic_participation: 1.0,
        stewardship_care: 1.0,
        semantic_resonance: 1.0,
        domain_competence: 1.0,
    };
    assert!(!profile.meets_requirement(&civic, &weights));

    profile.network_resilience = 1.0;
    profile.epistemic_integrity = 0.49;
    assert!(!profile.meets_requirement(&civic, &weights));

    profile.epistemic_integrity = 0.5;
    assert!(profile.meets_requirement(&civic, &weights));
}

#[test]
fn community_minimum_is_required_on_both_corresponding_8d_dimensions() {
    let legacy = GovernanceRequirement {
        min_tier: ConsciousnessTier::Observer,
        min_identity: None,
        min_community: Some(0.4),
    };
    let civic = try_civic_requirement_from_governance(&legacy).unwrap();
    let weights = DimensionWeights::governance();

    let mut profile = SovereignProfile {
        epistemic_integrity: 1.0,
        thermodynamic_yield: 1.0,
        network_resilience: 1.0,
        economic_velocity: 1.0,
        civic_participation: 1.0,
        stewardship_care: 1.0,
        semantic_resonance: 0.39,
        domain_competence: 1.0,
    };
    assert!(!profile.meets_requirement(&civic, &weights));

    profile.semantic_resonance = 1.0;
    profile.civic_participation = 0.39;
    assert!(!profile.meets_requirement(&civic, &weights));

    profile.civic_participation = 0.4;
    assert!(profile.meets_requirement(&civic, &weights));
}

#[test]
fn malformed_legacy_thresholds_fail_closed_instead_of_becoming_weaker_requirements() {
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -0.01, 1.01] {
        let invalid_identity = GovernanceRequirement {
            min_tier: ConsciousnessTier::Observer,
            min_identity: Some(invalid),
            min_community: None,
        };
        assert_eq!(
            try_civic_requirement_from_governance(&invalid_identity),
            Err(ConversionError::InvalidIdentityMinimum)
        );

        let invalid_community = GovernanceRequirement {
            min_tier: ConsciousnessTier::Observer,
            min_identity: None,
            min_community: Some(invalid),
        };
        assert_eq!(
            try_civic_requirement_from_governance(&invalid_community),
            Err(ConversionError::InvalidCommunityMinimum)
        );
    }
}
