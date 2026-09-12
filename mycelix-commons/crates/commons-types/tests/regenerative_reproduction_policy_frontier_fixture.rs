include!("regenerative_multigeneration_depth_fixture.rs");

const POLICY_FIXTURE: &str =
    include_str!("../fixtures/manta-forge-reproduction-policy-frontier-v1.txt");

fn bucket(
    complete_periods: u64,
    case_count: u32,
) -> commons_types::RegenerativeSuccessorDepthBucketV1 {
    commons_types::RegenerativeSuccessorDepthBucketV1 {
        complete_periods,
        case_count,
    }
}

fn policy_outcome(
    maturity_periods: u64,
    total_successful_handoff_count: u32,
    generation_depth_buckets: Vec<commons_types::RegenerativeSuccessorDepthBucketV1>,
) -> commons_types::RegenerativeReproductionPolicyOutcomeV1 {
    commons_types::RegenerativeReproductionPolicyOutcomeV1 {
        maturity_periods,
        total_successful_handoff_count,
        generation_depth_buckets,
    }
}

fn policy_frontier() -> commons_types::RegenerativeReproductionPolicyFrontierEvidenceV1 {
    let parent = depth_evidence();
    let baseline = multigeneration_evidence();
    commons_types::RegenerativeReproductionPolicyFrontierEvidenceV1 {
        schema_version: commons_types::REGENERATIVE_REPRODUCTION_POLICY_FRONTIER_SCHEMA_V1,
        policy_frontier_id: "manta-forge-reproduction-policy-frontier-v1".into(),
        parent_successor_depth_content_digest: parent.content_digest().unwrap(),
        baseline_multigeneration_content_digest: baseline.content_digest().unwrap(),
        fixture_content_binding:
            "git-blob:d39ea79abe3dce26d3f8cc589b1d223a79eaf321".into(),
        symtropy_policy_frontier_binding:
            "symtropy-pr:794:e49453c04dd56226bd53bd9b21f7e3c43703a667".into(),
        symthaea_policy_frontier_binding:
            "symthaea-pr:1934:b1bee0c6eb13fd8ab0f3bf055fec0c6ecee36fac".into(),
        policy_dimension_binding:
            "policy-dimension:minimum-complete-descendant-periods-before-reproduction".into(),
        reproductive_case_count: 18,
        outcomes: vec![
            policy_outcome(
                1,
                40,
                vec![bucket(1, 6), bucket(2, 5), bucket(3, 4), bucket(4, 3)],
            ),
            policy_outcome(2, 25, vec![bucket(1, 11), bucket(2, 7)]),
            policy_outcome(3, 21, vec![bucket(1, 15), bucket(2, 3)]),
            policy_outcome(4, 18, vec![bucket(1, 18)]),
        ],
        longer_maturity_strictly_reduces_total_handoffs: true,
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ParsedPolicyRow {
    maturity_periods: u64,
    depth_counts: [u32; 5],
    total_handoffs: u32,
}

fn parsed_policy_rows() -> Vec<ParsedPolicyRow> {
    POLICY_FIXTURE
        .lines()
        .filter_map(|line| line.strip_prefix("policy="))
        .map(|line| {
            let mut maturity_periods = None;
            let mut depth_counts = [0u32; 5];
            let mut total_handoffs = None;
            for field in line.split('|') {
                let (key, value) = field.split_once(':').unwrap();
                match key {
                    "maturity_periods" => maturity_periods = Some(value.parse().unwrap()),
                    "depth1_cases" => depth_counts[1] = value.parse().unwrap(),
                    "depth2_cases" => depth_counts[2] = value.parse().unwrap(),
                    "depth3_cases" => depth_counts[3] = value.parse().unwrap(),
                    "depth4_cases" => depth_counts[4] = value.parse().unwrap(),
                    "expected_total_handoffs" => total_handoffs = Some(value.parse().unwrap()),
                    other => panic!("unknown policy fixture field {other}"),
                }
            }
            ParsedPolicyRow {
                maturity_periods: maturity_periods.unwrap(),
                depth_counts,
                total_handoffs: total_handoffs.unwrap(),
            }
        })
        .collect()
}

#[test]
fn exact_policy_fixture_matches_carried_frontier() {
    let parent = depth_evidence();
    let baseline = multigeneration_evidence();
    let frontier = policy_frontier();
    commons_types::verify_regenerative_reproduction_policy_frontier_evidence(
        &parent,
        &baseline,
        &frontier,
    )
    .unwrap();

    let parsed = parsed_policy_rows();
    assert_eq!(parsed.len(), frontier.outcomes.len());
    for (row, outcome) in parsed.iter().zip(&frontier.outcomes) {
        assert_eq!(row.maturity_periods, outcome.maturity_periods);
        assert_eq!(row.total_handoffs, outcome.total_successful_handoff_count);
        for depth in 1..=4usize {
            let carried = outcome
                .generation_depth_buckets
                .iter()
                .find(|bucket| bucket.complete_periods == depth as u64)
                .map(|bucket| bucket.case_count)
                .unwrap_or(0);
            assert_eq!(row.depth_counts[depth], carried);
        }
    }
    assert_eq!(
        frontier.outcomes.iter().map(|row| row.total_successful_handoff_count).collect::<Vec<_>>(),
        vec![40, 25, 21, 18]
    );
}

#[test]
fn one_period_policy_is_exactly_the_bound_multigeneration_baseline() {
    let frontier = policy_frontier();
    let baseline = multigeneration_evidence();
    let one_period = frontier.outcome_for_maturity(1).unwrap();
    assert_eq!(
        one_period.total_successful_handoff_count,
        baseline.total_successful_handoff_count
    );
    assert_eq!(
        one_period.generation_depth_buckets,
        baseline.generation_depth_buckets
    );
}

#[test]
fn maturity_effect_claims_fail_closed_when_frontier_is_inconsistent() {
    let parent = depth_evidence();
    let baseline = multigeneration_evidence();

    let mut non_monotonic = policy_frontier();
    non_monotonic.outcomes[1].total_successful_handoff_count = 41;
    assert!(non_monotonic.validate().is_err());

    let mut wrong_baseline = policy_frontier();
    wrong_baseline.outcomes[0].generation_depth_buckets[0].case_count = 5;
    wrong_baseline.outcomes[0].generation_depth_buckets[1].case_count = 6;
    assert!(commons_types::verify_regenerative_reproduction_policy_frontier_evidence(
        &parent,
        &baseline,
        &wrong_baseline,
    )
    .is_err());
}

#[test]
fn policy_frontier_provenance_fits_existing_maritime_transport() {
    let frontier = policy_frontier();
    let envelope = frontier
        .to_maritime_envelope(
            "manta-lineage-frontier",
            3,
            1,
            1_789_230_100_000_000,
            "evidence:reproduction-policy-frontier-event-v1",
        )
        .unwrap();
    let decoded: commons_types::RegenerativeReproductionPolicyFrontierEvidenceV1 =
        serde_json::from_str(&envelope.payload_json).unwrap();
    assert_eq!(decoded, frontier);
}
