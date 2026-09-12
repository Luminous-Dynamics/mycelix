use commons_types::{
    verify_regenerative_successor_depth_evidence, RegenerativeSuccessorDepthBucketV1,
    RegenerativeSuccessorDepthEvidenceV1, RegenerativeViabilityFrontierEvidenceV1,
    REGENERATIVE_SUCCESSOR_DEPTH_SCHEMA_V1, REGENERATIVE_VIABILITY_FRONTIER_SCHEMA_V1,
};

const DEPTH_FIXTURE: &str = include_str!("../fixtures/manta-forge-successor-depth-frontier-v1.txt");

fn parent_frontier() -> RegenerativeViabilityFrontierEvidenceV1 {
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

fn depth_evidence() -> RegenerativeSuccessorDepthEvidenceV1 {
    let parent = parent_frontier();
    RegenerativeSuccessorDepthEvidenceV1 {
        schema_version: REGENERATIVE_SUCCESSOR_DEPTH_SCHEMA_V1,
        depth_frontier_id: "manta-forge-successor-depth-frontier-v1".into(),
        parent_frontier_content_digest: parent.content_digest().unwrap(),
        depth_fixture_content_binding:
            "git-blob:1e143e30acc9d0d89af97a4c07e31d7c8563c2bb".into(),
        symtropy_depth_binding:
            "symtropy-pr:785:a6a7b18bae74040e5c0c41d91d7ea1f248e663a3".into(),
        symthaea_depth_binding:
            "symthaea-pr:1884:2013bc796251c2bd940b2f617d689aa778f5d43f".into(),
        total_cases: 42,
        reproduction_ready_cases: 18,
        no_successor_cases: 24,
        depth_buckets: vec![
            RegenerativeSuccessorDepthBucketV1 {
                complete_periods: 1,
                case_count: 6,
            },
            RegenerativeSuccessorDepthBucketV1 {
                complete_periods: 2,
                case_count: 5,
            },
            RegenerativeSuccessorDepthBucketV1 {
                complete_periods: 3,
                case_count: 4,
            },
            RegenerativeSuccessorDepthBucketV1 {
                complete_periods: 4,
                case_count: 3,
            },
        ],
    }
}

#[test]
fn exact_depth_fixture_matches_carried_histogram() {
    let mut counts = [0u32; 5];
    let mut no_successor = 0u32;
    let mut total = 0u32;

    for line in DEPTH_FIXTURE
        .lines()
        .filter_map(|line| line.strip_prefix("case="))
    {
        total += 1;
        let horizon = line
            .split('|')
            .find_map(|field| {
                let (key, value) = field.split_once(':').unwrap();
                (key == "expected_successor_horizon").then_some(value)
            })
            .unwrap();
        if horizon == "none" {
            no_successor += 1;
        } else {
            counts[horizon.parse::<usize>().unwrap()] += 1;
        }
    }

    let depth = depth_evidence();
    verify_regenerative_successor_depth_evidence(&parent_frontier(), &depth).unwrap();
    assert_eq!(total, depth.total_cases);
    assert_eq!(no_successor, depth.no_successor_cases);
    assert_eq!(counts[1], 6);
    assert_eq!(counts[2], 5);
    assert_eq!(counts[3], 4);
    assert_eq!(counts[4], 3);
    assert_eq!(depth.cases_at_least(1), 18);
    assert_eq!(depth.cases_at_least(2), 12);
    assert_eq!(depth.cases_at_least(3), 7);
    assert_eq!(depth.cases_at_least(4), 3);
    assert_eq!(depth.max_observed_depth(), 4);
}

#[test]
fn parent_digest_and_exact_upstream_subjects_are_bound() {
    let depth = depth_evidence();
    assert_eq!(
        depth.parent_frontier_content_digest,
        parent_frontier().content_digest().unwrap()
    );
    assert_eq!(
        depth.depth_fixture_content_binding,
        "git-blob:1e143e30acc9d0d89af97a4c07e31d7c8563c2bb"
    );
    assert_eq!(
        depth.symtropy_depth_binding,
        "symtropy-pr:785:a6a7b18bae74040e5c0c41d91d7ea1f248e663a3"
    );
    assert_eq!(
        depth.symthaea_depth_binding,
        "symthaea-pr:1884:2013bc796251c2bd940b2f617d689aa778f5d43f"
    );
}
