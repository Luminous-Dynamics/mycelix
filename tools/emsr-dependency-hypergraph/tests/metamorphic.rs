use emsr_dependency_hypergraph::{
    AssertionState, DependencyDomain, DependencyHyperedge, DependencyQuery, Disposition,
    HypergraphInput, INPUT_CONTRACT, InventorySnapshot, Node, NodeKind, evaluate_bytes,
    independently_verify,
};

fn fixture() -> HypergraphInput {
    HypergraphInput {
        contract: INPUT_CONTRACT.to_owned(),
        snapshot: InventorySnapshot {
            snapshot_id: "snapshot-meta-001".to_owned(),
            as_of: 100,
            evidence_frontier: 120,
            complete: true,
            scoped_source_ids: vec!["rx-a".to_owned(), "rx-b".to_owned()],
            covered_node_ids: vec![
                "clock-root".to_owned(),
                "provider-a".to_owned(),
                "provider-b".to_owned(),
                "rx-a".to_owned(),
                "rx-b".to_owned(),
            ],
        },
        query: DependencyQuery {
            source_ids: vec!["rx-a".to_owned(), "rx-b".to_owned()],
            domain: DependencyDomain::Clock,
            event_time: 90,
            expected_snapshot_id: "snapshot-meta-001".to_owned(),
            expected_evidence_frontier: 120,
        },
        nodes: vec![
            Node {
                id: "clock-root".to_owned(),
                kind: NodeKind::Dependency,
            },
            Node {
                id: "provider-a".to_owned(),
                kind: NodeKind::Dependency,
            },
            Node {
                id: "provider-b".to_owned(),
                kind: NodeKind::Dependency,
            },
            Node {
                id: "rx-a".to_owned(),
                kind: NodeKind::Source,
            },
            Node {
                id: "rx-b".to_owned(),
                kind: NodeKind::Source,
            },
        ],
        hyperedges: vec![
            DependencyHyperedge {
                id: "edge-a".to_owned(),
                subject_ids: vec!["rx-a".to_owned()],
                dependency_id: "provider-a".to_owned(),
                domain: DependencyDomain::Clock,
                valid_from: Some(0),
                valid_until: None,
                recorded_at: 80,
                evidence_ref: "evidence:a".to_owned(),
                state: AssertionState::Qualified,
            },
            DependencyHyperedge {
                id: "edge-b".to_owned(),
                subject_ids: vec!["rx-b".to_owned()],
                dependency_id: "provider-b".to_owned(),
                domain: DependencyDomain::Clock,
                valid_from: Some(0),
                valid_until: None,
                recorded_at: 80,
                evidence_ref: "evidence:b".to_owned(),
                state: AssertionState::Qualified,
            },
            DependencyHyperedge {
                id: "shared-root".to_owned(),
                subject_ids: vec!["provider-a".to_owned(), "provider-b".to_owned()],
                dependency_id: "clock-root".to_owned(),
                domain: DependencyDomain::Clock,
                valid_from: Some(0),
                valid_until: None,
                recorded_at: 80,
                evidence_ref: "evidence:shared-root".to_owned(),
                state: AssertionState::Qualified,
            },
        ],
    }
}

fn evaluate(input: &HypergraphInput) -> emsr_dependency_hypergraph::EvaluationReport {
    let bytes = serde_json::to_vec(input).expect("serialize input");
    evaluate_bytes(&bytes)
}

fn no_shared_clock(input: &HypergraphInput) -> HypergraphInput {
    let mut output = input.clone();
    output.hyperedges.retain(|edge| edge.id != "shared-root");
    output
}

fn add_power_only_common_edge(input: &mut HypergraphInput) {
    input.nodes.push(Node {
        id: "power-root".to_owned(),
        kind: NodeKind::Dependency,
    });
    input.snapshot.covered_node_ids.push("power-root".to_owned());
    input.snapshot.covered_node_ids.sort();
    input.hyperedges.push(DependencyHyperedge {
        id: "shared-power".to_owned(),
        subject_ids: vec!["rx-a".to_owned(), "rx-b".to_owned()],
        dependency_id: "power-root".to_owned(),
        domain: DependencyDomain::Power,
        valid_from: Some(0),
        valid_until: None,
        recorded_at: 80,
        evidence_ref: "evidence:shared-power".to_owned(),
        state: AssertionState::Qualified,
    });
}

#[test]
fn node_and_edge_order_permutations_preserve_semantics_and_verify() {
    let original = fixture();
    let expected = evaluate(&original);
    assert_eq!(expected.disposition, Disposition::SharedDependency);

    let mut permuted = original.clone();
    permuted.nodes.reverse();
    permuted.hyperedges.reverse();
    let bytes = serde_json::to_vec(&permuted).expect("serialize permuted input");
    let report = evaluate_bytes(&bytes);

    assert_eq!(report.disposition, expected.disposition);
    assert_eq!(report.source_ancestors, expected.source_ancestors);
    assert_eq!(report.common_dependency_ids, expected.common_dependency_ids);
    assert_eq!(report.reason_codes, expected.reason_codes);

    let report_bytes = serde_json::to_vec(&report).expect("serialize report");
    let verification = independently_verify(&bytes, &report_bytes);
    assert_eq!(verification.status, "verified");
}

#[test]
fn a_shared_power_edge_cannot_change_a_clock_domain_answer() {
    let input = no_shared_clock(&fixture());
    assert_eq!(evaluate(&input).disposition, Disposition::NoDeclaredCommonDomain);

    let mut with_power = input.clone();
    add_power_only_common_edge(&mut with_power);
    let report = evaluate(&with_power);

    assert_eq!(report.disposition, Disposition::NoDeclaredCommonDomain);
    assert!(report.common_dependency_ids.is_empty());
}

#[test]
fn valid_time_is_half_open_at_the_upper_boundary() {
    let mut input = fixture();
    input
        .hyperedges
        .iter_mut()
        .find(|edge| edge.id == "shared-root")
        .expect("shared edge")
        .valid_until = Some(input.query.event_time);

    assert_eq!(evaluate(&input).disposition, Disposition::NoDeclaredCommonDomain);
}

#[test]
fn valid_time_includes_the_lower_boundary() {
    let mut input = fixture();
    input
        .hyperedges
        .iter_mut()
        .find(|edge| edge.id == "shared-root")
        .expect("shared edge")
        .valid_from = Some(input.query.event_time);

    assert_eq!(evaluate(&input).disposition, Disposition::SharedDependency);
}

#[test]
fn incomplete_inventory_cannot_upgrade_no_common_evidence_to_independent() {
    let complete = no_shared_clock(&fixture());
    assert_eq!(
        evaluate(&complete).disposition,
        Disposition::NoDeclaredCommonDomain
    );

    let mut incomplete = complete;
    incomplete.snapshot.complete = false;
    assert_eq!(
        evaluate(&incomplete).disposition,
        Disposition::UnknownDependency
    );
}

#[test]
fn known_common_evidence_remains_visible_when_other_lineage_is_unknown() {
    let mut input = fixture();
    input.snapshot.complete = false;

    let report = evaluate(&input);
    assert_eq!(report.disposition, Disposition::UnknownDependency);
    assert_eq!(report.common_dependency_ids, vec!["clock-root"]);
}

#[test]
fn adding_a_qualified_common_ancestor_cannot_make_a_pair_more_independent() {
    let independent = no_shared_clock(&fixture());
    assert_eq!(
        evaluate(&independent).disposition,
        Disposition::NoDeclaredCommonDomain
    );

    let common = fixture();
    assert_eq!(
        evaluate(&common).disposition,
        Disposition::SharedDependency
    );
}

#[test]
fn stale_common_edge_cannot_be_reported_as_no_declared_common_domain() {
    let mut input = fixture();
    let shared = input
        .hyperedges
        .iter_mut()
        .find(|edge| edge.id == "shared-root")
        .expect("shared edge");
    shared.state = AssertionState::Stale;

    assert_eq!(evaluate(&input).disposition, Disposition::UnknownDependency);
}
