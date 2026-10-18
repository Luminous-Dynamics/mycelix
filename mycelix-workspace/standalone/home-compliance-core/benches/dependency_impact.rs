//! Reproducible, dependency-free benchmark for dependency invalidation topologies.
//!
//! Run with:
//!   cargo bench --offline --bench dependency_impact
//!
//! Timings are machine- and toolchain-specific observations, not correctness
//! proofs and not a CI pass/fail threshold. Record the machine and Rust version
//! when comparing runs.

use home_compliance_core::evolution::{DependencyEdge, affected_cause_tree, affected_closure};
use std::hint::black_box;
use std::time::Instant;

const SAMPLES: usize = 5;

fn edge(input_id: impl Into<String>, dependent_id: impl Into<String>) -> DependencyEdge {
    DependencyEdge {
        input_id: input_id.into(),
        dependent_id: dependent_id.into(),
    }
}

fn chain_graph(edge_count: usize) -> Vec<DependencyEdge> {
    (0..edge_count)
        .map(|index| {
            edge(
                format!("artifact:{index}"),
                format!("artifact:{}", index + 1),
            )
        })
        .collect()
}

fn fanout_graph(edge_count: usize) -> Vec<DependencyEdge> {
    (0..edge_count)
        .map(|index| edge("artifact:root", format!("artifact:{index}")))
        .collect()
}

fn binary_tree_graph(edge_count: usize) -> Vec<DependencyEdge> {
    (0..edge_count)
        .map(|index| {
            let parent = if index < 2 {
                "artifact:root".to_owned()
            } else {
                format!("artifact:{}", (index - 2) / 2)
            };
            edge(parent, format!("artifact:{index}"))
        })
        .collect()
}

fn median(samples: &[f64]) -> f64 {
    samples[samples.len() / 2]
}

fn measure_ms(mut operation: impl FnMut()) -> Vec<f64> {
    let mut samples = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        let started = Instant::now();
        operation();
        samples.push(started.elapsed().as_secs_f64() * 1_000.0);
    }
    samples.sort_by(f64::total_cmp);
    samples
}

fn report_shape(
    topology: &str,
    dependencies: &[DependencyEdge],
    changed_id: &str,
    expected_nodes: usize,
    tail_id: &str,
    expected_tail_parent: Option<&str>,
) {
    let changed = vec![changed_id.to_owned()];
    let closure_samples = measure_ms(|| {
        let closure = affected_closure(black_box(changed.as_slice()), black_box(dependencies))
            .expect("generated dependency graph must be valid");
        assert_eq!(closure.len(), expected_nodes);
        assert!(closure.binary_search(&changed_id.to_owned()).is_ok());
    });

    let cause_tree_samples = measure_ms(|| {
        let cause_tree =
            affected_cause_tree(black_box(changed.as_slice()), black_box(dependencies))
                .expect("generated dependency graph must produce a cause tree");
        assert_eq!(cause_tree.len(), expected_nodes);
        let tail = cause_tree
            .iter()
            .find(|item| item.artifact_id == tail_id)
            .expect("the expected tail node must be reported");
        assert_eq!(tail.cause_parent_id.as_deref(), expected_tail_parent);
        assert!(
            cause_tree
                .windows(2)
                .all(|pair| pair[0].artifact_id < pair[1].artifact_id)
        );
    });

    println!(
        "topology={topology} nodes={expected_nodes} edges={} samples={SAMPLES} \
         closure_median_ms={:.3} closure_min_ms={:.3} closure_max_ms={:.3} \
         cause_tree_median_ms={:.3} cause_tree_min_ms={:.3} cause_tree_max_ms={:.3}",
        dependencies.len(),
        median(&closure_samples),
        closure_samples[0],
        closure_samples[SAMPLES - 1],
        median(&cause_tree_samples),
        cause_tree_samples[0],
        cause_tree_samples[SAMPLES - 1],
    );
}

fn benchmark_size(edge_count: usize) {
    let nodes = edge_count + 1;

    let chain = chain_graph(edge_count);
    let chain_tail_id = format!("artifact:{edge_count}");
    let chain_tail_parent = format!("artifact:{}", edge_count - 1);
    report_shape(
        "chain",
        &chain,
        "artifact:0",
        nodes,
        &chain_tail_id,
        Some(chain_tail_parent.as_str()),
    );

    let fanout = fanout_graph(edge_count);
    let fanout_tail_id = format!("artifact:{}", edge_count - 1);
    report_shape(
        "fanout",
        &fanout,
        "artifact:root",
        nodes,
        &fanout_tail_id,
        Some("artifact:root"),
    );

    let tree = binary_tree_graph(edge_count);
    let tree_tail_id = format!("artifact:{}", edge_count - 1);
    let tail_parent = if edge_count - 1 < 2 {
        "artifact:root".to_owned()
    } else {
        format!("artifact:{}", (edge_count - 1 - 2) / 2)
    };
    report_shape(
        "binary_tree",
        &tree,
        "artifact:root",
        nodes,
        &tree_tail_id,
        Some(tail_parent.as_str()),
    );
}

fn main() {
    println!("benchmark=home-compliance-dependency-impact");
    println!("timing_summary=median,min,max; correctness_assertions=enabled");
    println!("timings_are_observations_not_acceptance_proofs");
    for edge_count in [1_000, 10_000, 100_000] {
        benchmark_size(edge_count);
    }
}
