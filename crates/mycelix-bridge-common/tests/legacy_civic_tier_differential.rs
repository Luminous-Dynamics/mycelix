#![allow(deprecated)]

use mycelix_bridge_common::sovereign_gate::LegacyProfile;
use mycelix_bridge_common::{
    CivicTier, ConsciousnessProfile, ConsciousnessTier, DimensionWeights, SovereignProfile,
};

const LEVELS: [f64; 5] = [0.0, 0.25, 0.5, 0.75, 1.0];
const EXPECTED_PROFILES: u64 = 390_625;

fn civic_rank(tier: CivicTier) -> u8 {
    match tier {
        CivicTier::Observer => 0,
        CivicTier::Participant => 1,
        CivicTier::Citizen => 2,
        CivicTier::Steward => 3,
        CivicTier::Guardian => 4,
    }
}

fn legacy_rank(tier: ConsciousnessTier) -> u8 {
    match tier {
        ConsciousnessTier::Observer => 0,
        ConsciousnessTier::Participant => 1,
        ConsciousnessTier::Citizen => 2,
        ConsciousnessTier::Steward => 3,
        ConsciousnessTier::Guardian => 4,
    }
}

fn tier_name(rank: usize) -> &'static str {
    match rank {
        1 => "Participant",
        2 => "Citizen",
        3 => "Steward",
        4 => "Guardian",
        _ => "Observer",
    }
}

#[test]
fn finite_grid_reports_native_vs_projected_legacy_authority_differential() {
    let weights = DimensionWeights::governance();

    let mut total = 0_u64;
    let mut native_higher = 0_u64;
    let mut equal = 0_u64;
    let mut native_lower = 0_u64;

    let mut broadening = [0_u64; 5];
    let mut narrowing = [0_u64; 5];
    let mut first_broadening: [Option<[f64; 8]>; 5] = [None; 5];
    let mut first_narrowing: [Option<[f64; 8]>; 5] = [None; 5];

    for &d0 in &LEVELS {
        for &d1 in &LEVELS {
            for &d2 in &LEVELS {
                for &d3 in &LEVELS {
                    for &d4 in &LEVELS {
                        for &d5 in &LEVELS {
                            for &d6 in &LEVELS {
                                for &d7 in &LEVELS {
                                    let values = [d0, d1, d2, d3, d4, d5, d6, d7];
                                    let profile = SovereignProfile::from_array(values);
                                    let native_tier = profile.tier(&weights);

                                    let projected = LegacyProfile::from(profile.clone());
                                    for value in [
                                        projected.identity,
                                        projected.reputation,
                                        projected.community,
                                        projected.engagement,
                                    ] {
                                        assert!(value.is_finite());
                                        assert!((0.0..=1.0).contains(&value));
                                    }

                                    let legacy_profile = ConsciousnessProfile {
                                        identity: projected.identity,
                                        reputation: projected.reputation,
                                        community: projected.community,
                                        engagement: projected.engagement,
                                    };
                                    let legacy_tier = legacy_profile.tier();

                                    let native_rank = civic_rank(native_tier);
                                    let projected_legacy_rank = legacy_rank(legacy_tier);

                                    match native_rank.cmp(&projected_legacy_rank) {
                                        core::cmp::Ordering::Greater => native_higher += 1,
                                        core::cmp::Ordering::Equal => equal += 1,
                                        core::cmp::Ordering::Less => native_lower += 1,
                                    }

                                    for required in 1_usize..=4 {
                                        let required_rank = required as u8;
                                        if native_rank >= required_rank
                                            && projected_legacy_rank < required_rank
                                        {
                                            broadening[required] += 1;
                                            first_broadening[required].get_or_insert(values);
                                        }
                                        if projected_legacy_rank >= required_rank
                                            && native_rank < required_rank
                                        {
                                            narrowing[required] += 1;
                                            first_narrowing[required].get_or_insert(values);
                                        }
                                    }

                                    total += 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    assert_eq!(total, EXPECTED_PROFILES);
    assert_eq!(native_higher + equal + native_lower, total);

    println!(
        "FIN_SAFE_028_TIER_SUMMARY total={total} native_higher={native_higher} equal={equal} native_lower={native_lower}"
    );

    for required in 1_usize..=4 {
        println!(
            "FIN_SAFE_028_AUTHORITY threshold={} native_pass_legacy_fail={} legacy_pass_native_fail={} first_native_pass_legacy_fail={:?} first_legacy_pass_native_fail={:?}",
            tier_name(required),
            broadening[required],
            narrowing[required],
            first_broadening[required],
            first_narrowing[required],
        );
    }
}
