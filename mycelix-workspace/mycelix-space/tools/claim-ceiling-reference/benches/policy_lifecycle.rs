// Copyright (C) 2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Warm-path microbenchmarks for the frozen policy lifecycle corpus.
//!
//! This is a measurement harness, not a correctness proof or a claim of superiority.
//! It warms the embedded-schema validator cache before timing and measures the complete
//! lifecycle evaluator for every frozen case. Compare identical harnesses/toolchains and
//! hosts across commits before attributing gains to the cache.

use claim_ceiling_reference::policy_lifecycle::{
    PolicyLifecycleTrustedContextV1, evaluate_policy_release_bytes,
};
use criterion::{BenchmarkId, Criterion, black_box, criterion_group, criterion_main};
use serde_json::Value;
use std::time::Duration;

const CORPUS: &str = include_str!("../fixtures/claim-policy-lifecycle-golden-v1.json");

struct BenchCase {
    id: String,
    release_bytes: Vec<u8>,
    profile_bytes: Vec<u8>,
    trusted_context: PolicyLifecycleTrustedContextV1,
}

fn load_cases() -> Vec<BenchCase> {
    let corpus: Value = serde_json::from_str(CORPUS).expect("frozen lifecycle corpus parses");
    corpus["cases"]
        .as_array()
        .expect("corpus contains cases")
        .iter()
        .map(|case| BenchCase {
            id: case["id"].as_str().expect("case id is a string").to_owned(),
            release_bytes: serde_json::to_vec(&case["release"]).expect("serialize release"),
            profile_bytes: case["profile_bytes"]
                .as_str()
                .expect("profile bytes are represented as a JSON string")
                .as_bytes()
                .to_vec(),
            trusted_context: serde_json::from_value(case["trusted_context"].clone())
                .expect("trusted context matches the lifecycle type"),
        })
        .collect()
}

fn policy_lifecycle_warm_path(c: &mut Criterion) {
    let cases = load_cases();

    // Compile embedded validators before measurement. This is deliberately a warm-path
    // benchmark; cold-start compilation is a separate measurement and is not inferred here.
    for case in &cases {
        evaluate_policy_release_bytes(
            &case.release_bytes,
            &case.profile_bytes,
            &case.trusted_context,
        )
        .expect("frozen case should produce a reference receipt");
    }

    let mut group = c.benchmark_group("policy_lifecycle_warm_path");
    group.sample_size(30);
    group.warm_up_time(Duration::from_millis(200));
    group.measurement_time(Duration::from_secs(1));

    for case in &cases {
        group.bench_with_input(BenchmarkId::new("evaluate", &case.id), case, |b, case| {
            b.iter(|| {
                let receipt = evaluate_policy_release_bytes(
                    black_box(&case.release_bytes),
                    black_box(&case.profile_bytes),
                    black_box(&case.trusted_context),
                )
                .expect("frozen case should produce a reference receipt");
                black_box(receipt)
            });
        });
    }

    group.finish();
}

criterion_group!(benches, policy_lifecycle_warm_path);
criterion_main!(benches);
