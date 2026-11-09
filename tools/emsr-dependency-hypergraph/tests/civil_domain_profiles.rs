use emsr_dependency_hypergraph::{
    evaluate_bytes, independently_verify, AssertionState, DependencyDomain, DependencyHyperedge,
    DependencyQuery, Disposition, HypergraphInput, InventorySnapshot, Node, NodeKind,
    INPUT_CONTRACT,
};
use serde_json::to_vec;

fn node(id: &str, kind: NodeKind) -> Node {
    Node {
        id: id.to_owned(),
        kind,
    }
}

fn edge(
    id: &str,
    subjects: &[&str],
    dependency: &str,
    domain: DependencyDomain,
    state: AssertionState,
) -> DependencyHyperedge {
    let mut subject_ids: Vec<String> = subjects.iter().map(|item| (*item).to_owned()).collect();
    subject_ids.sort();
    DependencyHyperedge {
        id: id.to_owned(),
        subject_ids,
        dependency_id: dependency.to_owned(),
        domain,
        valid_from: Some(0),
        valid_until: None,
        recorded_at: 80,
        evidence_ref: format!("synthetic:{id}"),
        state,
    }
}

fn input(
    domain: DependencyDomain,
    sources: &[&str],
    nodes: Vec<Node>,
    hyperedges: Vec<DependencyHyperedge>,
) -> HypergraphInput {
    let mut source_ids: Vec<String> = sources.iter().map(|item| (*item).to_owned()).collect();
    source_ids.sort();
    let mut covered_node_ids: Vec<String> = nodes.iter().map(|item| item.id.clone()).collect();
    covered_node_ids.sort();
    HypergraphInput {
        contract: INPUT_CONTRACT.to_owned(),
        snapshot: InventorySnapshot {
            snapshot_id: "civil-domain-snapshot-v0.1".to_owned(),
            as_of: 100,
            evidence_frontier: 120,
            complete: true,
            scoped_source_ids: source_ids.clone(),
            covered_node_ids,
        },
        query: DependencyQuery {
            source_ids,
            domain,
            event_time: 90,
            expected_snapshot_id: "civil-domain-snapshot-v0.1".to_owned(),
            expected_evidence_frontier: 120,
        },
        nodes,
        hyperedges,
    }
}

fn evaluate(value: &HypergraphInput) -> emsr_dependency_hypergraph::EvaluationReport {
    let input_bytes = to_vec(value).expect("serialize synthetic civil profile");
    let report = evaluate_bytes(&input_bytes);
    let report_bytes = to_vec(&report).expect("serialize synthetic civil report");
    let verification = independently_verify(&input_bytes, &report_bytes);
    assert_eq!(verification.status, "verified");
    report
}

#[test]
fn uas_vendor_feeds_sharing_a_collector_are_not_independent_confirmations() {
    let value = input(
        DependencyDomain::Collector,
        &["rf-feed-a", "rf-feed-b"],
        vec![
            node("collector-a", NodeKind::Dependency),
            node("collector-b", NodeKind::Dependency),
            node("rf-feed-a", NodeKind::Source),
            node("rf-feed-b", NodeKind::Source),
            node("shared-collector", NodeKind::Dependency),
        ],
        vec![
            edge("a-to-collector", &["rf-feed-a"], "collector-a", DependencyDomain::Collector, AssertionState::Qualified),
            edge("b-to-collector", &["rf-feed-b"], "collector-b", DependencyDomain::Collector, AssertionState::Qualified),
            edge("collector-common-root", &["collector-a", "collector-b"], "shared-collector", DependencyDomain::Collector, AssertionState::Qualified),
        ],
    );
    let report = evaluate(&value);
    assert_eq!(report.disposition, Disposition::SharedDependency);
    assert_eq!(report.common_dependency_ids, vec!["shared-collector"]);
}

#[test]
fn maritime_ais_and_radar_paths_sharing_a_port_pipeline_are_correlated() {
    let value = input(
        DependencyDomain::DataPipeline,
        &["ais-provider", "radar-processor"],
        vec![
            node("ais-aggregation", NodeKind::Dependency),
            node("ais-provider", NodeKind::Source),
            node("port-correlation-platform", NodeKind::Dependency),
            node("radar-processor", NodeKind::Source),
            node("radar-track-pipeline", NodeKind::Dependency),
        ],
        vec![
            edge("ais-to-platform", &["ais-provider"], "ais-aggregation", DependencyDomain::DataPipeline, AssertionState::Qualified),
            edge("radar-to-pipeline", &["radar-processor"], "radar-track-pipeline", DependencyDomain::DataPipeline, AssertionState::Qualified),
            edge("shared-port-platform", &["ais-aggregation", "radar-track-pipeline"], "port-correlation-platform", DependencyDomain::DataPipeline, AssertionState::Qualified),
        ],
    );
    let report = evaluate(&value);
    assert_eq!(report.disposition, Disposition::SharedDependency);
    assert_eq!(report.common_dependency_ids, vec!["port-correlation-platform"]);
}

#[test]
fn civil_aviation_shared_power_does_not_imply_shared_clock() {
    let mut value = input(
        DependencyDomain::Clock,
        &["adsb-feed", "mlat-feed"],
        vec![
            node("adsb-clock", NodeKind::Dependency),
            node("adsb-feed", NodeKind::Source),
            node("airport-power-bus", NodeKind::Dependency),
            node("mlat-clock", NodeKind::Dependency),
            node("mlat-feed", NodeKind::Source),
        ],
        vec![
            edge("adsb-clock-edge", &["adsb-feed"], "adsb-clock", DependencyDomain::Clock, AssertionState::Qualified),
            edge("mlat-clock-edge", &["mlat-feed"], "mlat-clock", DependencyDomain::Clock, AssertionState::Qualified),
            edge("shared-power", &["adsb-feed", "mlat-feed"], "airport-power-bus", DependencyDomain::Power, AssertionState::Qualified),
        ],
    );
    let clock_report = evaluate(&value);
    assert_eq!(clock_report.disposition, Disposition::NoDeclaredCommonDomain);
    assert!(clock_report.common_dependency_ids.is_empty());

    value.query.domain = DependencyDomain::Power;
    let power_report = evaluate(&value);
    assert_eq!(power_report.disposition, Disposition::SharedDependency);
    assert_eq!(power_report.common_dependency_ids, vec!["airport-power-bus"]);
}

#[test]
fn complete_positive_control_uses_no_declared_common_domain_not_physical_independence() {
    let value = input(
        DependencyDomain::DataPipeline,
        &["source-a", "source-b"],
        vec![
            node("pipeline-a", NodeKind::Dependency),
            node("pipeline-b", NodeKind::Dependency),
            node("source-a", NodeKind::Source),
            node("source-b", NodeKind::Source),
        ],
        vec![
            edge("source-a-pipeline", &["source-a"], "pipeline-a", DependencyDomain::DataPipeline, AssertionState::Qualified),
            edge("source-b-pipeline", &["source-b"], "pipeline-b", DependencyDomain::DataPipeline, AssertionState::Qualified),
        ],
    );
    let report = evaluate(&value);
    assert_eq!(report.disposition, Disposition::NoDeclaredCommonDomain);
    assert!(report.claim_ceiling.contains("not proof of physical or statistical independence"));
}

#[test]
fn unknown_maritime_lineage_is_not_promoted_to_no_common_domain() {
    let mut value = input(
        DependencyDomain::DataPipeline,
        &["ais-provider", "sar-processor"],
        vec![
            node("ais-provider", NodeKind::Source),
            node("sar-processor", NodeKind::Source),
        ],
        vec![edge(
            "unknown-sar-lineage",
            &["sar-processor"],
            "missing-source-registry-entry",
            DependencyDomain::DataPipeline,
            AssertionState::Unknown,
        )],
    );
    // The edge references a missing node and is structurally invalid. This
    // must not be misreported as an independent-source finding.
    let report = evaluate(&value);
    assert_eq!(report.disposition, Disposition::InvalidInput);

    // When the dependency node exists but its edge is unqualified, disposition
    // is UNKNOWN_DEPENDENCY rather than NO_DECLARED_COMMON_DOMAIN.
    value.nodes.push(node("missing-source-registry-entry", NodeKind::Dependency));
    value.nodes.sort_by(|left, right| left.id.cmp(&right.id));
    value.snapshot.covered_node_ids.push("missing-source-registry-entry".to_owned());
    value.snapshot.covered_node_ids.sort();
    let report = evaluate(&value);
    assert_eq!(report.disposition, Disposition::UnknownDependency);
}

#[test]
fn aviation_clock_and_antenna_site_are_distinct_dependency_dimensions() {
    let value = input(
        DependencyDomain::AntennaSite,
        &["adsb-feed", "mlat-feed"],
        vec![
            node("adsb-feed", NodeKind::Source),
            node("airport-antenna-site", NodeKind::Dependency),
            node("mlat-feed", NodeKind::Source),
        ],
        vec![edge(
            "shared-antenna-site",
            &["adsb-feed", "mlat-feed"],
            "airport-antenna-site",
            DependencyDomain::AntennaSite,
            AssertionState::Qualified,
        )],
    );
    let report = evaluate(&value);
    assert_eq!(report.disposition, Disposition::SharedDependency);
    assert_eq!(report.common_dependency_ids, vec!["airport-antenna-site"]);
}


#[test]
fn long_maritime_dependency_chains_are_stack_safe_and_reproducible() {
    const DEPTH: usize = 10_000;
    let mut nodes = vec![
        node("ais-observation-source", NodeKind::Source),
        node("radar-observation-source", NodeKind::Source),
        node("shared-port-core", NodeKind::Dependency),
    ];
    let mut edges = Vec::with_capacity(2 * DEPTH + 1);

    for (prefix, source) in [
        ("ais-dependency", "ais-observation-source"),
        ("radar-dependency", "radar-observation-source"),
    ] {
        let mut previous = source.to_owned();
        for index in 0..DEPTH {
            let current = format!("{prefix}-{index:05}");
            nodes.push(node(&current, NodeKind::Dependency));
            edges.push(edge(
                &format!("{prefix}-edge-{index:05}"),
                &[&previous],
                &current,
                DependencyDomain::DataPipeline,
                AssertionState::Qualified,
            ));
            previous = current;
        }
        edges.push(edge(
            &format!("{prefix}-to-shared-port-core"),
            &[&previous],
            "shared-port-core",
            DependencyDomain::DataPipeline,
            AssertionState::Qualified,
        ));
    }
    nodes.sort_by(|left, right| left.id.cmp(&right.id));

    let value = input(
        DependencyDomain::DataPipeline,
        &["ais-observation-source", "radar-observation-source"],
        nodes,
        edges,
    );
    let report = evaluate(&value);
    assert_eq!(report.disposition, Disposition::SharedDependency);
    assert_eq!(report.common_dependency_ids, vec!["shared-port-core"]);
}
