use emsr_dependency_hypergraph::{
    evaluate_bytes, independently_verify, AssertionState, DependencyDomain, DependencyHyperedge,
    DependencyQuery, Disposition, HypergraphInput, InventorySnapshot, Node, NodeKind,
    INPUT_CONTRACT,
};
use serde::Deserialize;

const CHALLENGE_PACK: &str =
    include_str!("../fixtures/dependency-hypergraph-challenges-0.1.json");

#[derive(Debug, Deserialize)]
struct ChallengePack {
    schema: String,
    version: String,
    state: String,
    claim_ceiling: String,
    evaluator_truth: String,
    case_count: usize,
    cases: Vec<ChallengeCase>,
}

#[derive(Debug, Deserialize)]
struct ChallengeCase {
    id: String,
    mutation: String,
    expected_disposition: String,
    expected_reason_codes: Vec<String>,
}

fn base_input() -> HypergraphInput {
    HypergraphInput {
        contract: INPUT_CONTRACT.to_owned(),
        snapshot: InventorySnapshot {
            snapshot_id: "snapshot-dh-001".to_owned(),
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
            expected_snapshot_id: "snapshot-dh-001".to_owned(),
            expected_evidence_frontier: 120,
        },
        nodes: vec![
            Node { id: "clock-root".to_owned(), kind: NodeKind::Dependency },
            Node { id: "provider-a".to_owned(), kind: NodeKind::Dependency },
            Node { id: "provider-b".to_owned(), kind: NodeKind::Dependency },
            Node { id: "rx-a".to_owned(), kind: NodeKind::Source },
            Node { id: "rx-b".to_owned(), kind: NodeKind::Source },
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
                evidence_ref: "evidence:root".to_owned(),
                state: AssertionState::Qualified,
            },
        ],
    }
}

fn mutate(input: &mut HypergraphInput, mutation: &str) {
    match mutation {
        "transitive_shared_ancestor" => {}
        "direct_multi_subject_hyperedge" => {
            input.hyperedges.truncate(2);
            input.hyperedges.push(DependencyHyperedge {
                id: "one-clock-for-both".to_owned(),
                subject_ids: vec!["rx-a".to_owned(), "rx-b".to_owned()],
                dependency_id: "clock-root".to_owned(),
                domain: DependencyDomain::Clock,
                valid_from: None,
                valid_until: None,
                recorded_at: 80,
                evidence_ref: "evidence:one-clock".to_owned(),
                state: AssertionState::Qualified,
            });
        }
        "shared_power_does_not_imply_clock" => {
            input.nodes.push(Node { id: "power-root".to_owned(), kind: NodeKind::Dependency });
            input.nodes.sort_by(|left, right| left.id.cmp(&right.id));
            input.snapshot.covered_node_ids.push("power-root".to_owned());
            input.snapshot.covered_node_ids.sort();
            input.hyperedges = vec![DependencyHyperedge {
                id: "power-only".to_owned(),
                subject_ids: vec!["rx-a".to_owned(), "rx-b".to_owned()],
                dependency_id: "power-root".to_owned(),
                domain: DependencyDomain::Power,
                valid_from: Some(0),
                valid_until: None,
                recorded_at: 80,
                evidence_ref: "evidence:power-only".to_owned(),
                state: AssertionState::Qualified,
            }];
            input.query.domain = DependencyDomain::Clock;
        }
        "shared_edge_expired_at_event_time" => {
            input.hyperedges[2].valid_until = Some(50);
        }
        "stale_shared_edge" => {
            input.hyperedges[2].state = AssertionState::Stale;
        }
        "source_missing_from_inventory_scope" => {
            input.snapshot.scoped_source_ids.pop();
        }
        "incomplete_inventory_snapshot" => {
            input.snapshot.complete = false;
        }
        "source_missing_from_snapshot_coverage" => {
            input.snapshot.covered_node_ids.pop();
        }
        "dependency_cycle" => {
            input.hyperedges.push(DependencyHyperedge {
                id: "cycle".to_owned(),
                subject_ids: vec!["clock-root".to_owned()],
                dependency_id: "provider-a".to_owned(),
                domain: DependencyDomain::Clock,
                valid_from: Some(0),
                valid_until: None,
                recorded_at: 80,
                evidence_ref: "evidence:cycle".to_owned(),
                state: AssertionState::Qualified,
            });
        }
        "tampered_closure_report" => {}
        "noncanonical_source_order" => {
            input.query.source_ids.reverse();
        }
        "complete_inventory_no_common_ancestor" => {
            input.hyperedges.truncate(2);
        }
        other => panic!("unknown frozen challenge mutation: {other}"),
    }
}

fn expected_disposition(name: &str) -> Disposition {
    match name {
        "SHARED_DEPENDENCY" => Disposition::SharedDependency,
        "UNKNOWN_DEPENDENCY" => Disposition::UnknownDependency,
        "NO_DECLARED_COMMON_DOMAIN" => Disposition::NoDeclaredCommonDomain,
        "INVALID_INPUT" => Disposition::InvalidInput,
        other => panic!("unknown expected disposition: {other}"),
    }
}

#[test]
fn challenge_pack_is_frozen_and_case_discovery_is_exact() {
    let pack: ChallengePack = serde_json::from_str(CHALLENGE_PACK).expect("challenge pack JSON");
    assert_eq!(pack.schema, "emsr-dependency-hypergraph-challenge-pack/0.1");
    assert_eq!(pack.version, "0.1.0");
    assert_eq!(pack.state, "FROZEN_PUBLIC_SYNTHETIC_CONFORMANCE");
    assert!(!pack.claim_ceiling.trim().is_empty());
    assert!(!pack.evaluator_truth.trim().is_empty());
    assert_eq!(pack.case_count, pack.cases.len());
    let expected_ids = [
        "DH-001", "DH-002", "DH-003", "DH-004", "DH-005", "DH-006",
        "DH-007", "DH-008", "DH-009", "DH-010", "DH-011", "DH-012",
    ];
    let actual_ids: Vec<&str> = pack.cases.iter().map(|case| case.id.as_str()).collect();
    assert_eq!(actual_ids, expected_ids);
}

#[test]
fn every_frozen_challenge_matches_the_expected_disposition_and_verifier() {
    let pack: ChallengePack = serde_json::from_str(CHALLENGE_PACK).expect("challenge pack JSON");

    for case in &pack.cases {
        let mut input = base_input();
        mutate(&mut input, &case.mutation);
        let input_bytes = serde_json::to_vec(&input).expect("serialize challenge input");

        if case.expected_disposition == "VERIFIER_REJECT" {
            let mut report = evaluate_bytes(&input_bytes);
            report.common_dependency_ids.clear();
            let report_bytes = serde_json::to_vec(&report).expect("serialize tampered report");
            let verification = independently_verify(&input_bytes, &report_bytes);
            assert_eq!(verification.status, "rejected", "{}", case.id);
            continue;
        }

        let report = evaluate_bytes(&input_bytes);
        assert_eq!(
            report.disposition,
            expected_disposition(&case.expected_disposition),
            "{}",
            case.id
        );
        for reason in &case.expected_reason_codes {
            assert!(report.reason_codes.contains(reason), "{} missing reason {reason}", case.id);
        }

        if case.id == "DH-001" {
            assert_eq!(report.common_dependency_ids, vec!["clock-root"]);
            assert_eq!(report.source_ancestors["rx-a"], vec!["clock-root", "provider-a"]);
            assert_eq!(report.source_ancestors["rx-b"], vec!["clock-root", "provider-b"]);
        }

        let report_bytes = serde_json::to_vec(&report).expect("serialize report");
        let verification = independently_verify(&input_bytes, &report_bytes);
        assert_eq!(verification.status, "verified", "{}", case.id);
    }
}
