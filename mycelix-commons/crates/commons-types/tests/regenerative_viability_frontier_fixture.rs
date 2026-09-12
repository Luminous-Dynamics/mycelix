use commons_types::{
    RegenerativeViabilityFrontierEvidenceV1, REGENERATIVE_VIABILITY_FRONTIER_SCHEMA_V1,
};

const FIXTURE: &str = include_str!("../fixtures/manta-forge-viability-frontier-v1.txt");

fn evidence() -> RegenerativeViabilityFrontierEvidenceV1 {
    RegenerativeViabilityFrontierEvidenceV1 {
        schema_version: REGENERATIVE_VIABILITY_FRONTIER_SCHEMA_V1,
        frontier_id: "manta-forge-viability-frontier-v1".into(),
        fixture_content_binding:
            "git-blob:a6348dd1d94de454114e3de4bad6086d080c4157".into(),
        symtropy_frontier_binding:
            "symtropy-pr:779:c3c7f09853075457f430f61eba7dee0a603f29bf".into(),
        symthaea_frontier_binding:
            "symthaea-pr:1873:552830efa215e49dca3cb341c91b1e6fa062ec6c".into(),
        bootstrap_policy_binding: "bootstrap-policy:manta-forge-frontier-v1".into(),
        varied_dimension_refs: vec![
            "dimension:forge-tooling-stock".into(),
            "dimension:metrology-recovery-tick".into(),
        ],
        fixed_context_refs: vec![
            "fixed:metrology-initial-stock:1".into(),
            "fixed:reactor-service-stock:8".into(),
            "fixed:structural-stock:18".into(),
        ],
        total_cases: 42,
        role_overlap_cases: 25,
        reproduction_ready_cases: 18,
        role_overlap_without_bootstrap_cases: 7,
        no_role_overlap_cases: 17,
    }
}

#[test]
fn exact_frontier_fixture_matches_carried_summary_counts() {
    let mut total = 0u32;
    let mut role_overlap = 0u32;
    let mut reproduction_ready = 0u32;

    for line in FIXTURE.lines().filter_map(|line| line.strip_prefix("case=")) {
        total += 1;
        let mut role_window = None;
        let mut handoff_window = None;
        for field in line.split('|') {
            let (key, value) = field.split_once(':').unwrap();
            match key {
                "expected_role_window" => role_window = Some(value),
                "expected_handoff_window" => handoff_window = Some(value),
                _ => {}
            }
        }
        if role_window.unwrap() != "none" {
            role_overlap += 1;
        }
        if handoff_window.unwrap() != "none" {
            reproduction_ready += 1;
        }
    }

    let carried = evidence();
    carried.validate().unwrap();
    assert_eq!(total, carried.total_cases);
    assert_eq!(role_overlap, carried.role_overlap_cases);
    assert_eq!(reproduction_ready, carried.reproduction_ready_cases);
    assert_eq!(
        role_overlap - reproduction_ready,
        carried.role_overlap_without_bootstrap_cases
    );
    assert_eq!(total - role_overlap, carried.no_role_overlap_cases);
    assert!(carried.bootstrap_frontier_is_strictly_narrower());
}

#[test]
fn shared_fixture_identity_and_upstream_subjects_are_pinned() {
    let carried = evidence();
    assert_eq!(
        carried.fixture_content_binding,
        "git-blob:a6348dd1d94de454114e3de4bad6086d080c4157"
    );
    assert_eq!(
        carried.symtropy_frontier_binding,
        "symtropy-pr:779:c3c7f09853075457f430f61eba7dee0a603f29bf"
    );
    assert_eq!(
        carried.symthaea_frontier_binding,
        "symthaea-pr:1873:552830efa215e49dca3cb341c91b1e6fa062ec6c"
    );
}
