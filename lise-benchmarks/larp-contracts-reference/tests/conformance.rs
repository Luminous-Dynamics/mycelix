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

#[test]
fn missing_authority_fact_never_becomes_permission() {
    let scenarios: ScenarioSuite = serde_json::from_str(SCENARIOS).unwrap();
    let positive = scenarios.cases.iter().find(|case| case.id == "P01").unwrap();
    let mut facts = positive.facts.clone();
    facts.authority.local_authorized = None;
    let decision = evaluate(&facts, facts.evaluated_at_unix);
    assert_eq!(decision.disposition, Disposition::Indeterminate);
}

#[test]
fn adding_a_known_bypass_cannot_improve_a_disposition() {
    let scenarios: ScenarioSuite = serde_json::from_str(SCENARIOS).unwrap();
    let positive = scenarios.cases.iter().find(|case| case.id == "P01").unwrap();
    let mut facts = positive.facts.clone();
    facts.institution.known_route_bypass = Some(true);
    let decision = evaluate(&facts, facts.evaluated_at_unix);
    assert_eq!(decision.disposition, Disposition::Deny);
}

#[test]
fn adding_a_common_dependency_invalidates_an_independence_claim() {
    let scenarios: ScenarioSuite = serde_json::from_str(SCENARIOS).unwrap();
    let positive = scenarios.cases.iter().find(|case| case.id == "P03").unwrap();
    let mut facts = positive.facts.clone();
    facts.evidence.known_shared_dependency = Some(true);
    let decision = evaluate(&facts, facts.evaluated_at_unix);
    assert_eq!(decision.disposition, Disposition::Indeterminate);
}

#[test]
fn contract_expiry_uses_explicit_evaluation_time() {
    let scenarios: ScenarioSuite = serde_json::from_str(SCENARIOS).unwrap();
    let unknown = scenarios.cases.iter().find(|case| case.id == "R06").unwrap();
    let facts = &unknown.facts;
    let before_expiry = evaluate(facts, 1_799_998_000);
    let after_expiry = evaluate(facts, 1_800_000_000);
    assert_eq!(before_expiry.disposition, Disposition::Hold);
    assert_eq!(after_expiry.disposition, Disposition::Hold);
    assert_eq!(before_expiry.reason_codes[0], "EXTERNAL_OUTCOME_UNKNOWN_RECONCILIATION_REQUIRED");
    assert_eq!(after_expiry.reason_codes[0], "EXPIRED_CONTRACT_WITH_UNRECONCILED_EFFECT");
}

#[test]
fn safe_continuity_requires_a_verified_trigger() {
    let scenarios: ScenarioSuite = serde_json::from_str(SCENARIOS).unwrap();
    let recovery = scenarios.cases.iter().find(|case| case.id == "R04").unwrap();
    let mut facts = recovery.facts.clone();
    facts.institution.continuity_trigger_verified = None;
    let decision = evaluate(&facts, facts.evaluated_at_unix);
    assert_eq!(decision.disposition, Disposition::Indeterminate);
}

#[test]
fn already_applied_logical_operation_cannot_be_dispatched_again() {
    let scenarios: ScenarioSuite = serde_json::from_str(SCENARIOS).unwrap();
    let replay = scenarios.cases.iter().find(|case| case.id == "R03").unwrap();
    let decision = evaluate(&replay.facts, replay.facts.evaluated_at_unix);
    assert_eq!(decision.disposition, Disposition::Deny);
    assert_eq!(decision.reason_codes[0], "LOGICAL_OPERATION_ALREADY_APPLIED");
}

#[test]
fn partner_credential_cannot_bypass_evidence_purpose_or_disclosure_rights() {
    let scenarios: ScenarioSuite = serde_json::from_str(SCENARIOS).unwrap();
    let partner = scenarios.cases.iter().find(|case| case.id == "P07").unwrap();

    let mut purpose_unknown = partner.facts.clone();
    purpose_unknown.scope.purpose_allowed = None;
    assert_eq!(
        evaluate(&purpose_unknown, purpose_unknown.evaluated_at_unix).disposition,
        Disposition::Indeterminate
    );

    let mut rights_denied = partner.facts.clone();
    rights_denied.rights.rights_satisfied = Some(false);
    assert_eq!(
        evaluate(&rights_denied, rights_denied.evaluated_at_unix).disposition,
        Disposition::Deny
    );
}

#[test]
fn staged_reward_requires_rights_and_remedy_not_only_treasury_authorization() {
    let scenarios: ScenarioSuite = serde_json::from_str(SCENARIOS).unwrap();
    let reward = scenarios.cases.iter().find(|case| case.id == "P06").unwrap();
    let mut facts = reward.facts.clone();
    facts.rights.rights_satisfied = Some(false);
    let decision = evaluate(&facts, facts.evaluated_at_unix);
    assert_eq!(decision.disposition, Disposition::Deny);
}
