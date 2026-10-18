//! Reproducible, dependency-free scale benchmark for dependency invalidation.
//!
//! Run with:
//!   cargo bench --offline --bench dependency_impact
//!
//! Timings are machine- and toolchain-specific observations, not correctness
//! proofs and not a CI pass/fail threshold. Record the machine and Rust version
//! when comparing runs.

use home_compliance_core::evolution::{
    affected_cause_tree, affected_closure, DependencyEdge,
};
use std::hint::black_box;
use std::time::Instant;

fn dependency_chain(edge_count: usize) -> Vec<DependencyEdge> {
    (0..edge_count)
        .map(|index| DependencyEdge {
            input_id: format!("artifact:{index}"),
            dependent_id: format!("artifact:{}", index + 1),
        })
        .collect()
}

fn benchmark_size(edge_count: usize) {
    let dependencies = dependency_chain(edge_count);
    let changed = vec!["artifact:0".to_owned()];
    let expected_nodes = edge_count + 1;

    let started = Instant::now();
    let closure = affected_closure(
        black_box(changed.as_slice()),
        black_box(dependencies.as_slice()),
    )
    .expect("generated dependency chain must be valid");
    let closure_elapsed = started.elapsed();
    assert_eq!(closure.len(), expected_nodes);
    assert!(closure.binary_search(&"artifact:0".to_owned()).is_ok());

    let final_id = format!("artifact:{edge_count}");
    let expected_parent = format!("artifact:{}", edge_count - 1);
    let started = Instant::now();
    let cause_tree = affected_cause_tree(
        black_box(changed.as_slice()),
        black_box(dependencies.as_slice()),
    )
    .expect("the same generated chain must produce a cause tree");
    let cause_tree_elapsed = started.elapsed();
    assert_eq!(cause_tree.len(), expected_nodes);
    let final_node = cause_tree
        .iter()
        .find(|item| item.artifact_id == final_id)
        .expect("the last chain node must be reported");
    assert_eq!(final_node.cause_parent_id.as_deref(), Some(expected_parent.as_str()));

    println!(
        "graph=chain nodes={} edges={} closure_ms={:.3} cause_tree_ms={:.3}",
        expected_nodes,
        edge_count,
        closure_elapsed.as_secs_f64() * 1_000.0,
        cause_tree_elapsed.as_secs_f64() * 1_000.0,
    );
}

fn main() {
    println!("benchmark=home-compliance-dependency-impact");
    println!("timings_are_observations_not_acceptance_proofs");
    for edge_count in [1_000, 10_000, 100_000] {
        benchmark_size(edge_count);
    }
}
