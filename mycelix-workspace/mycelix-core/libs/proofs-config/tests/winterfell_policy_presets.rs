// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

#![cfg(feature = "winterfell")]

use proofs_config::{SecurityLevel, winterfell_options::proof_options_for_level};
use winterfell::BatchingMethod;

fn assert_linear_batching(level: SecurityLevel) {
    let options = proof_options_for_level(level);
    assert_eq!(
        options.constraint_batching_method(),
        BatchingMethod::Linear,
        "constraint batching must remain explicit Linear for {level:?}"
    );
    assert_eq!(
        options.deep_poly_batching_method(),
        BatchingMethod::Linear,
        "DEEP batching must remain explicit Linear for {level:?}"
    );
}

#[test]
fn historical_presets_use_explicit_linear_batching() {
    for level in [
        SecurityLevel::Fast,
        SecurityLevel::Optimized,
        SecurityLevel::Standard,
        SecurityLevel::High,
    ] {
        assert_linear_batching(level);
    }
}

#[test]
fn historical_query_blowup_and_grinding_values_are_preserved() {
    let fast = proof_options_for_level(SecurityLevel::Fast);
    assert_eq!(fast.num_queries(), 28);
    assert_eq!(fast.blowup_factor(), 8);
    assert_eq!(fast.grinding_factor(), 0);

    let optimized = proof_options_for_level(SecurityLevel::Optimized);
    assert_eq!(optimized.num_queries(), 40);
    assert_eq!(optimized.blowup_factor(), 8);
    assert_eq!(optimized.grinding_factor(), 16);

    let standard = proof_options_for_level(SecurityLevel::Standard);
    assert_eq!(standard.num_queries(), 50);
    assert_eq!(standard.blowup_factor(), 8);
    assert_eq!(standard.grinding_factor(), 20);

    let high = proof_options_for_level(SecurityLevel::High);
    assert_eq!(high.num_queries(), 100);
    assert_eq!(high.blowup_factor(), 16);
    assert_eq!(high.grinding_factor(), 24);
}
