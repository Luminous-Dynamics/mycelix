// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Dependency-free native benchmark harness for the capability-plan compiler.
//!
//! Run from mycelix-workspace/mycelix-praxis with:
//!   cargo bench -p praxis-core --bench capability_plan
//!
//! This measures planner cost only. It does not measure learning efficacy, catalogue
//! truth, network or UI latency. Record CPU, OS, Rust version, optimization settings,
//! exact source SHA and raw output with any published comparison. Do not compare
//! different hardware/toolchains as if they were controlled experiments.
//!
//! The malformed-reference workload times a known rejection path. It is not intended
//! to measure every failure mode; cycle, budget, and preference failures remain
//! correctness-test responsibilities until separately instrumented.

use praxis_core::{
    compile_goal_plan, CapabilityGraph, CapabilityId, CapabilityNode, DeliveryMode,
    GoalPlanConstraints, GoalPlanError, GoalPlanRequest, PracticeOption,
};
use std::collections::BTreeSet;
use std::hint::black_box;
use std::time::Instant;

const GENERATED_AT: i64 = 1_800_000_000;
const MAX_TASK_MINUTES: u32 = 60;
const ITERATIONS: usize = 30;

#[derive(Clone, Copy, Debug)]
enum Shape {
    Chain,
    WideFanIn,
    BalancedTree,
    SharedPrerequisites,
    IsolatedComponents,
}

impl Shape {
    fn as_str(self) -> &'static str {
        match self {
            Self::Chain => "chain",
            Self::WideFanIn => "wide_fan_in",
            Self::BalancedTree => "balanced_tree",
            Self::SharedPrerequisites => "shared_prerequisites",
            Self::IsolatedComponents => "isolated_components",
        }
    }
}

fn cap(index: usize) -> CapabilityId {
    CapabilityId(format!("cap-{index:05}"))
}

fn make_graph(size: usize, option_count: usize, shape: Shape) -> CapabilityGraph {
    assert!(size > 0, "benchmark graph size must be nonzero");
    assert!(option_count > 0, "benchmark needs at least one option");
    let mut nodes = Vec::with_capacity(size);

    for index in 0..size {
        let prerequisites = match shape {
            Shape::Chain if index > 0 => vec![cap(index - 1)],
            Shape::Chain => Vec::new(),
            Shape::WideFanIn if index == 0 => Vec::new(),
            Shape::WideFanIn if index == size - 1 => (0..size - 1).map(cap).collect(),
            Shape::WideFanIn => vec![cap(0)],
            Shape::BalancedTree if index == 0 => Vec::new(),
            Shape::BalancedTree => vec![cap((index - 1) / 2)],
            Shape::SharedPrerequisites if index < 2 => Vec::new(),
            Shape::SharedPrerequisites => vec![cap(0), cap(1)],
            Shape::IsolatedComponents if index % 8 == 0 => Vec::new(),
            Shape::IsolatedComponents => vec![cap(index - 1)],
        };

        let practice_options = (0..option_count)
            .map(|option_index| PracticeOption {
                task_id: format!("task-{index:05}-{option_index:02}"),
                task_version: "1".into(),
                rubric_id: format!("rubric-{index:05}-{option_index:02}"),
                rubric_version: "1".into(),
                route_priority: u16::try_from(option_index).unwrap_or(u16::MAX),
                rationale: "Synthetic benchmark fixture; not measured pedagogical efficacy."
                    .into(),
                estimated_minutes: 1 + ((option_index * 7) % MAX_TASK_MINUTES as usize) as u32,
                delivery_mode: match option_index % 3 {
                    0 => DeliveryMode::RetrievalPractice,
                    1 => DeliveryMode::GuidedPractice,
                    _ => DeliveryMode::Simulation,
                },
            })
            .collect();

        nodes.push(CapabilityNode {
            capability_id: cap(index),
            version: "1".into(),
            title: format!("Synthetic capability {index}"),
            prerequisites,
            practice_options,
        });
    }

    CapabilityGraph {
        graph_id: format!("synthetic-{size}-{option_count}-{}", shape.as_str()),
        revision: "benchmark-v2".into(),
        nodes,
    }
}

fn make_request(size: usize) -> GoalPlanRequest {
    GoalPlanRequest {
        request_id: format!("bench-{size}"),
        target_capability: cap(size - 1),
        constraints: GoalPlanConstraints {
            allowed_delivery_modes: [
                DeliveryMode::RetrievalPractice,
                DeliveryMode::GuidedPractice,
                DeliveryMode::Simulation,
            ]
            .into_iter()
            .collect::<BTreeSet<_>>(),
            max_task_minutes: MAX_TASK_MINUTES,
            max_total_minutes: u32::try_from(size)
                .unwrap_or(u32::MAX)
                .saturating_mul(MAX_TASK_MINUTES),
        },
        preferred_practice: Default::default(),
    }
}

// Nearest-rank percentile: rank = ceil(p * n), converted to a zero-based index.
// At 30 samples, p95 necessarily has coarse resolution; retain all raw samples by
// capturing the full run output rather than over-interpreting the percentile.
fn percentile(sorted: &[u128], numerator: usize, denominator: usize) -> u128 {
    assert!(!sorted.is_empty(), "percentile needs at least one sample");
    assert!(denominator > 0 && numerator > 0 && numerator <= denominator);
    let rank = sorted
        .len()
        .saturating_mul(numerator)
        .div_ceil(denominator)
        .clamp(1, sorted.len());
    sorted[rank - 1]
}

fn print_row(
    case_name: &str,
    shape: &str,
    size: usize,
    options: usize,
    iterations: usize,
    steps: usize,
    outcome: &str,
    samples: &mut [u128],
    digest: &str,
) {
    assert!(!samples.is_empty(), "benchmark needs at least one sample");
    samples.sort_unstable();
    let min_ns = samples[0];
    let p50_ns = percentile(samples, 50, 100);
    let p95_ns = percentile(samples, 95, 100);
    let max_ns = *samples.last().expect("samples are nonempty");

    println!(
        "{case_name},{shape},{size},{options},{iterations},{steps},{outcome},{min_ns},{p50_ns},{p95_ns},{max_ns},{digest}"
    );
}

fn benchmark_success(size: usize, options: usize, shape: Shape, iterations: usize) {
    let graph = make_graph(size, options, shape);
    let request = make_request(size);

    // Exclude one warm-up call from measurements.
    let warmup = compile_goal_plan(&graph, &request, GENERATED_AT)
        .unwrap_or_else(|error| panic!("valid fixture failed validation: {error}"));
    black_box(warmup.input_digest);

    let mut samples = Vec::with_capacity(iterations);
    let mut final_digest = String::new();
    let mut final_steps = 0;

    for _ in 0..iterations {
        let started = Instant::now();
        let plan = compile_goal_plan(black_box(&graph), black_box(&request), GENERATED_AT)
            .unwrap_or_else(|error| panic!("planner failed valid fixture: {error}"));
        samples.push(started.elapsed().as_nanos());
        final_digest = plan.input_digest;
        final_steps = plan.steps.len();
        black_box(final_steps);
        black_box(final_digest.as_bytes());
    }

    print_row(
        "valid",
        shape.as_str(),
        size,
        options,
        iterations,
        final_steps,
        "success",
        &mut samples,
        &final_digest,
    );
}

fn benchmark_missing_reference(size: usize, options: usize, iterations: usize) {
    let mut graph = make_graph(size, options, Shape::Chain);
    graph.graph_id = format!("negative-missing-ref-{size}-{options}");
    graph
        .nodes
        .last_mut()
        .expect("size is nonzero")
        .prerequisites
        .push(CapabilityId("cap-negative-control-missing".into()));
    let request = make_request(size);

    // The fixture must reject for the expected structural reason before timing.
    match compile_goal_plan(&graph, &request, GENERATED_AT) {
        Err(GoalPlanError::UnknownPrerequisite { .. }) => {}
        Err(other) => panic!("negative fixture rejected for unexpected reason: {other}"),
        Ok(_) => panic!("negative fixture was accepted"),
    }

    let mut samples = Vec::with_capacity(iterations);
    for _ in 0..iterations {
        let started = Instant::now();
        let result = compile_goal_plan(black_box(&graph), black_box(&request), GENERATED_AT);
        samples.push(started.elapsed().as_nanos());

        match result {
            Err(GoalPlanError::UnknownPrerequisite { .. }) => {}
            Err(other) => panic!("negative fixture changed failure class: {other}"),
            Ok(_) => panic!("malformed graph unexpectedly compiled"),
        }
    }

    print_row(
        "negative_control",
        "missing_prerequisite",
        size,
        options,
        iterations,
        0,
        "expected_rejection_unknown_prerequisite",
        &mut samples,
        "not_applicable",
    );
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.iter().any(|arg| arg == "--help" || arg == "-h") {
        println!("Usage: cargo bench -p praxis-core --bench capability_plan");
        println!("CSV columns: case,shape,nodes,options_per_node,iterations,steps,outcome,min_ns,p50_ns,p95_ns,max_ns,input_digest");
        return;
    }

    println!("case,shape,nodes,options_per_node,iterations,steps,outcome,min_ns,p50_ns,p95_ns,max_ns,input_digest");
    for size in [10_usize, 100, 512, 4_096] {
        let options: &[usize] = if size <= 512 { &[1, 8, 64] } else { &[1, 8] };

        for &option_count in options {
            for shape in [
                Shape::Chain,
                Shape::WideFanIn,
                Shape::BalancedTree,
                Shape::SharedPrerequisites,
                Shape::IsolatedComponents,
            ] {
                benchmark_success(size, option_count, shape, ITERATIONS);
            }

            // One fail-closed structural error path at each size/option count.
            benchmark_missing_reference(size, option_count, ITERATIONS);
        }
    }
}
