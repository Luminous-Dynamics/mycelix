include!("regenerative_reproduction_policy_frontier_fixture.rs");

const POLICY_METRICS_FIXTURE: &str =
    include_str!("../fixtures/manta-forge-generation-policy-metrics-v1.txt");

fn metric_row(
    maturity_periods: u64,
    founded_descendants: u32,
    maturity_completed_descendants: u32,
    descendant_reproduction_transitions: u32,
) -> commons_types::RegenerativeGenerationPolicyMetricRowV1 {
    commons_types::RegenerativeGenerationPolicyMetricRowV1 {
        maturity_periods,
        founded_descendants,
        maturity_completed_descendants,
        descendant_reproduction_transitions,
    }
}

fn policy_metrics() -> commons_types::RegenerativeGenerationPolicyMetricsEvidenceV1 {
    let parent = policy_frontier();
    commons_types::RegenerativeGenerationPolicyMetricsEvidenceV1 {
        schema_version: commons_types::REGENERATIVE_GENERATION_POLICY_METRICS_SCHEMA_V1,
        metrics_id: "manta-forge-generation-policy-metrics-v1".into(),
        parent_policy_frontier_content_digest: parent.content_digest().unwrap(),
        metrics_fixture_binding:
            "git-blob:9700c5b4e3d18a50b7f2d412169fcc831096dd58".into(),
        symtropy_dynamic_metrics_binding:
            "symtropy-pr:799:a2101e4a1f8306dcaf018eb5da291b90aceb3304".into(),
        symthaea_projection_binding:
            "symthaea-pr:1947:26dacc728073445b8f1eaa21a77439f5fc50a753".into(),
        symthaea_metrics_binding:
            "symthaea-pr:1951:ef38a17ce32ce8018609e08396913f77b1309c0c".into(),
        reproductive_case_count: 18,
        rows: vec![
            metric_row(1, 40, 40, 22),
            metric_row(2, 25, 15, 7),
            metric_row(3, 21, 7, 3),
            metric_row(4, 18, 3, 0),
        ],
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ParsedMetricRow {
    maturity_periods: u64,
    founded_descendants: u32,
    maturity_completed_descendants: u32,
    descendant_reproduction_transitions: u32,
}

fn parsed_metric_rows() -> Vec<ParsedMetricRow> {
    POLICY_METRICS_FIXTURE
        .lines()
        .filter_map(|line| line.strip_prefix("policy="))
        .map(|line| {
            let mut maturity_periods = None;
            let mut founded_descendants = None;
            let mut maturity_completed_descendants = None;
            let mut descendant_reproduction_transitions = None;
            for field in line.split('|') {
                let (key, value) = field.split_once(':').unwrap();
                match key {
                    "maturity_periods" => maturity_periods = Some(value.parse().unwrap()),
                    "founded_descendants" => founded_descendants = Some(value.parse().unwrap()),
                    "maturity_completed_descendants" => {
                        maturity_completed_descendants = Some(value.parse().unwrap())
                    }
                    "descendant_reproduction_transitions" => {
                        descendant_reproduction_transitions = Some(value.parse().unwrap())
                    }
                    other => panic!("unknown policy metric fixture field {other}"),
                }
            }
            ParsedMetricRow {
                maturity_periods: maturity_periods.unwrap(),
                founded_descendants: founded_descendants.unwrap(),
                maturity_completed_descendants: maturity_completed_descendants.unwrap(),
                descendant_reproduction_transitions: descendant_reproduction_transitions.unwrap(),
            }
        })
        .collect()
}

#[test]
fn exact_metric_fixture_matches_carried_generation_semantics() {
    let parent = policy_frontier();
    let metrics = policy_metrics();
    commons_types::verify_regenerative_generation_policy_metrics_evidence(&parent, &metrics)
        .unwrap();

    let parsed = parsed_metric_rows();
    assert_eq!(parsed.len(), metrics.rows.len());
    for (fixture, carried) in parsed.iter().zip(&metrics.rows) {
        assert_eq!(fixture.maturity_periods, carried.maturity_periods);
        assert_eq!(fixture.founded_descendants, carried.founded_descendants);
        assert_eq!(
            fixture.maturity_completed_descendants,
            carried.maturity_completed_descendants
        );
        assert_eq!(
            fixture.descendant_reproduction_transitions,
            carried.descendant_reproduction_transitions
        );
    }
}

#[test]
fn founded_counts_are_exactly_composed_with_policy_frontier() {
    let parent = policy_frontier();
    let metrics = policy_metrics();
    for (row, outcome) in metrics.rows.iter().zip(&parent.outcomes) {
        assert_eq!(row.maturity_periods, outcome.maturity_periods);
        assert_eq!(row.founded_descendants, outcome.total_successful_handoff_count);
        assert_eq!(
            row.founded_descendants,
            metrics.reproductive_case_count + row.descendant_reproduction_transitions
        );
        assert!(row.maturity_completed_descendants <= row.founded_descendants);
        assert!(row.descendant_reproduction_transitions <= row.maturity_completed_descendants);
    }
}

#[test]
fn ambiguous_or_inconsistent_metric_claims_fail_closed() {
    let parent = policy_frontier();

    let mut impossible_maturity = policy_metrics();
    impossible_maturity.rows[3].maturity_completed_descendants = 19;
    assert!(impossible_maturity.validate().is_err());

    let mut wrong_founded = policy_metrics();
    wrong_founded.rows[1].founded_descendants = 26;
    assert!(commons_types::verify_regenerative_generation_policy_metrics_evidence(
        &parent,
        &wrong_founded,
    )
    .is_err());
}

#[test]
fn metric_provenance_fits_existing_maritime_transport() {
    let metrics = policy_metrics();
    let envelope = metrics
        .to_maritime_envelope(
            "manta-lineage-frontier",
            3,
            2,
            1_789_230_200_000_000,
            "evidence:generation-policy-metrics-event-v1",
        )
        .unwrap();
    let decoded: commons_types::RegenerativeGenerationPolicyMetricsEvidenceV1 =
        serde_json::from_str(&envelope.payload_json).unwrap();
    assert_eq!(decoded, metrics);
}
