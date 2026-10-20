// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Dependency-free, native benchmark harness for the capability-plan compiler.
//!
//! Run from mycelix-workspace/mycelix-praxis with:
//!   cargo bench -p praxis-core --bench capability_plan
//!
//! This measures planner cost only. It does not measure learning efficacy, catalogue
//! truth, network or UI latency. Record CPU, OS, Rust version, optimization settings,
//! exact source SHA, and output with any published comparison. Do not compare results
//! from different hardware/toolchains as if they were controlled experiments.

use praxis_core::{
    compile_goal_plan, CapabilityGraph, CapabilityId, CapabilityNode, DeliveryMode,
    GoalPlanConstraints, GoalPlanRequest, PracticeOption,
};
use std::collections::BTreeSet;
use std::hint::black_box;
use std::time::Instant;

const GENERATED_AT: i64 = 1_800_000_000;
const MAX_TASK_MINUTES: u32 = 60;

#[derive(Clone, Copy, Debug)]
enum Shape {
    Chain,
    WideFanIn,
}

impl Shape {
    fn as_str(self) -> &'static str {
        match self {
            Self::Chain => "chain",
            Self::WideFanIn => "wide_fan_in",
        }
    }
}

fn cap(index: usize) -> CapabilityId {
    CapabilityId(format!("cap-{index:05}"))
}

fn make_graph(size: usize, option_count: usize, shape: Shape) -> CapabilityGraph {
    let mut nodes = Vec::with_capacity(size);

    for index in 0..size {
        let prerequisites = match shape {
            Shape::Chain if index > 0 => vec![cap(index - 1)],
            Shape::Chain => Vec::new(),
            Shape::WideFanIn if index == 0 => Vec::new(),
            Shape::WideFanIn if index == size - 1 => (0..size - 1).map(cap).collect(),
            Shape::WideFanIn => vec![cap(0)],
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
        revision: "benchmark-v1".into(),
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

fn percentile(sorted: &[u128], numerator: usize, denominator: usize) -> u128 {
    let index = ((sorted.len() - 1) * numerator) / denominator;
    sorted[index]
}

fn benchmark(size: usize, options: usize, shape: Shape, iterations: usize) {
    let graph = make_graph(size, options, shape);
    let request = make_request(size);

    // Exclude one warm-up call from measurements.
    let warmup = compile_goal_plan(&graph, &request, GENERATED_AT)
        .unwrap_or_else(|error| panic!("fixture failed validation: {error}"));
    black_box(warmup.input_digest);

    let mut samples = Vec::with_capacity(iterations);
    let mut final_digest = String::new();
    let mut final_steps = 0;

    for _ in 0..iterations {
        let started = Instant::now();
        let plan = compile_goal_plan(black_box(&graph), black_box(&request), GENERATED_AT)
            .unwrap_or_else(|error| panic!("planner failed benchmark fixture: {error}"));
        samples.push(started.elapsed().as_nanos());
        final_digest = plan.input_digest;
        final_steps = plan.steps.len();
        black_box(final_steps);
    }

    samples.sort_unstable();
    let min_ns = samples[0];
    let p50_ns = percentile(&samples, 50, 100);
    let p95_ns = percentile(&samples, 95, 100);
    let max_ns = *samples.last().expect("at least one benchmark sample");

    println!(
        "{},{},{},{},{},{},{},{},{},{}",
        shape.as_str(),
        size,
        options,
        iterations,
        final_steps,
        min_ns,
        p50_ns,
        p95_ns,
        max_ns,
        final_digest
    );
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.iter().any(|arg| arg == "--help" || arg == "-h") {
        println!("Usage: cargo bench -p praxis-core --bench capability_plan");
        println!("CSV columns: shape,nodes,options_per_node,iterations,steps,min_ns,p50_ns,p95_ns,max_ns,input_digest");
        return;
    }

    println!("shape,nodes,options_per_node,iterations,steps,min_ns,p50_ns,p95_ns,max_ns,input_digest");
    for size in [10_usize, 100, 512, 4_096] {
        let options: &[usize] = if size <= 512 { &[1, 8, 64] } else { &[1, 8] };
        // Thirty samples keeps p95 interpretable across all workloads; the
        // 64-option stress case is intentionally omitted at 4,096 nodes to keep
        // the baseline run bounded. Override repetitions only in a future
        // explicitly parameterized harness rather than silently shrinking samples. 
        let iterations = 30;

        for &option_count in options {
            for shape in [Shape::Chain, Shape::WideFanIn] {
                benchmark(size, option_count, shape, iterations);
            }
        }
    }
}
