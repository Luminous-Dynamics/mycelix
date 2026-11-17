use lise_temporal_institution_reference::{
    replay_trace, TraceOracle, TraceSuite,
};

const SCENARIOS: &str = include_str!("../fixtures/scenarios.json");
const ORACLE: &str = include_str!("../fixtures/oracle.json");

#[test]
fn temporal_trace_fixtures_match_the_separate_oracle() {
    let suite: TraceSuite = serde_json::from_str(SCENARIOS).expect("scenario suite parses");
    let oracle: TraceOracle = serde_json::from_str(ORACLE).expect("oracle parses");
    assert_eq!(suite.case_count, suite.cases.len());
    assert_eq!(oracle.case_count, oracle.rows.len());
    assert_eq!(suite.suite_id, oracle.suite_id);
    assert_eq!(suite.cases.len(), 12);

    let scenario_ids: std::collections::BTreeSet<_> =
        suite.cases.iter().map(|case| case.id.as_str()).collect();
    let oracle_rows: std::collections::BTreeMap<_, _> =
        oracle.rows.iter().map(|row| (row.case_id.as_str(), row)).collect();
    assert_eq!(scenario_ids.len(), suite.cases.len());
    assert_eq!(oracle_rows.len(), oracle.rows.len());
    assert_eq!(scenario_ids, oracle_rows.keys().copied().collect());

    for scenario in &suite.cases {
        let (_, actual) = replay_trace(&scenario.actions);
        let expected = oracle_rows[scenario.id.as_str()];
        assert_eq!(
            actual.disposition.as_str(),
            expected.expected_disposition,
            "{} — {}",
            scenario.id,
            scenario.title
        );
        assert_eq!(
            actual.reason_code,
            expected.reason_code,
            "{} reason — {}",
            scenario.id,
            scenario.title
        );
    }
}

#[test]
fn bounded_search_reports_findings_and_respects_its_state_budget() {
    let report = lise_temporal_institution_reference::bounded_explore(4, 10_000);
    assert!(report.visited_states > 1);
    assert!(!report.hard_counterexamples.is_empty());
    assert!(!report.challenge_witnesses.is_empty());
    assert!(report
        .hard_counterexamples
        .iter()
        .all(|witness| !witness.trace.is_empty()));
}
