use larp_contracts_reference::{evaluate, Disposition, OracleSuite, ScenarioSuite};
use std::collections::{BTreeMap, BTreeSet};

const SCENARIOS: &str = include_str!("../fixtures/scenarios.json");
const ORACLE: &str = include_str!("../fixtures/oracle.json");

#[test]
fn all_scenarios_match_the_separate_reference_oracle() {
    let scenarios: ScenarioSuite = serde_json::from_str(SCENARIOS).expect("scenario JSON must parse");
    let oracle: OracleSuite = serde_json::from_str(ORACLE).expect("oracle JSON must parse");

    assert_eq!(scenarios.case_count, scenarios.cases.len());
    assert_eq!(oracle.case_count, oracle.rows.len());
    assert_eq!(scenarios.suite_id, oracle.suite_id);
    assert_eq!(scenarios.cases.len(), 29);

    let ids: BTreeSet<_> = scenarios.cases.iter().map(|case| case.id.as_str()).collect();
    assert_eq!(ids.len(), scenarios.cases.len(), "scenario IDs must be unique");

    let rows: BTreeMap<_, _> = oracle.rows.iter().map(|row| (row.case_id.as_str(), row)).collect();
    assert_eq!(rows.len(), oracle.rows.len(), "oracle IDs must be unique");
    assert_eq!(ids, rows.keys().copied().collect(), "each scenario must have exactly one oracle row");

    for case in &scenarios.cases {
        let expected = rows[case.id.as_str()].expected_disposition.as_str();
        let actual = evaluate(&case.facts, case.facts.evaluated_at_unix);
        assert_eq!(
            actual.disposition.as_str(),
            expected,
            "{} — {}",
            case.id,
            case.title
        );
        assert!(!actual.reason_codes.is_empty(), "{} must emit a stable reason code", case.id);
    }
}

#[test]
fn scenario_parser_rejects_unknown_top_level_fields() {
    let invalid = r#"{"schema_version":"x","suite_id":"x","status":"x","contract_version":"x","case_count":0,"fixture_rules":[],"fact_schema_version":"x","cases":[],"surprise":true}"#;
    assert!(serde_json::from_str::<ScenarioSuite>(invalid).is_err());
}

#[test]
fn facts_parser_rejects_unknown_fields_in_nested_authority() {
    let invalid = r#"{
      "mode":"protected_action",
      "evaluated_at_unix":1,
      "authority":{"unexpected_admin":true},
      "scope":{},
      "rights":{},
      "evidence":{},
      "institution":{},
      "execution":{"outcome":"not_started"}
    }"#;
    assert!(serde_json::from_str::<larp_contracts_reference::EvaluationFacts>(invalid).is_err());
}

#[test]
fn stale_evidence_is_held_for_current_use_but_not_erased_from_history() {
    let scenarios: ScenarioSuite = serde_json::from_str(SCENARIOS).unwrap();
    let historical = scenarios.cases.iter().find(|case| case.id == "A14").unwrap();
    let historical_decision = evaluate(&historical.facts, historical.facts.evaluated_at_unix);
    assert_eq!(historical_decision.disposition, Disposition::Permit);

    let current = scenarios.cases.iter().find(|case| case.id == "P03").unwrap();
    let mut current_facts = current.facts.clone();
    current_facts.evidence.fresh_for_current_time = Some(false);
    let current_decision = evaluate(&current_facts, current_facts.evaluated_at_unix);
    assert_eq!(current_decision.disposition, Disposition::Hold);
}

#[test]
fn evaluation_is_deterministic_for_pinned_inputs_and_time() {
    let scenarios: ScenarioSuite = serde_json::from_str(SCENARIOS).unwrap();
    for case in &scenarios.cases {
        let first = evaluate(&case.facts, case.facts.evaluated_at_unix);
        let second = evaluate(&case.facts, case.facts.evaluated_at_unix);
        assert_eq!(first, second, "{}", case.id);
    }
}

#[test]
fn evaluator_has_no_mutating_or_provider_capability_in_its_interface() {
    // This reference API accepts only immutable facts and returns a value.
    // There is intentionally no executor, signer, treasury, network client,
    // credential issuer, or mutable authority-store handle in its API.
    let source = include_str!("../src/lib.rs");
    assert!(source.contains("pub fn evaluate(facts: &EvaluationFacts, now_unix: i64) -> Decision"));
    assert!(!source.contains("pub async fn execute"));
    assert!(!source.contains("pub fn transfer_funds"));
    assert!(!source.contains("pub fn grant_authority"));
}
