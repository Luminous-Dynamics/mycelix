use larp_contracts_reference::{
    evaluate, evaluate_epistemic_bounty, evaluate_restoration_contract, Disposition,
    DomainOracleSuite, DomainScenarioSuite, EpistemicBountyFacts, OracleSuite, RestorationFacts,
    ScenarioSuite,
};
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

const EPISTEMIC_SCENARIOS: &str = include_str!("../fixtures/epistemic_scenarios.json");
const EPISTEMIC_ORACLE: &str = include_str!("../fixtures/epistemic_oracle.json");
const RESTORATION_SCENARIOS: &str = include_str!("../fixtures/restoration_scenarios.json");
const RESTORATION_ORACLE: &str = include_str!("../fixtures/restoration_oracle.json");

#[test]
fn epistemic_bounty_domain_cases_match_separate_oracle() {
    let suite: DomainScenarioSuite<EpistemicBountyFacts> =
        serde_json::from_str(EPISTEMIC_SCENARIOS).expect("epistemic cases must parse");
    let oracle: DomainOracleSuite =
        serde_json::from_str(EPISTEMIC_ORACLE).expect("epistemic oracle must parse");
    assert_eq!(suite.case_count, suite.cases.len());
    assert_eq!(oracle.case_count, oracle.rows.len());
    assert_eq!(suite.suite_id, oracle.suite_id);
    assert_eq!(suite.cases.len(), 13);

    let ids: BTreeSet<_> = suite.cases.iter().map(|case| case.id.as_str()).collect();
    let rows: BTreeMap<_, _> = oracle.rows.iter().map(|row| (row.case_id.as_str(), row)).collect();
    assert_eq!(ids.len(), suite.cases.len());
    assert_eq!(rows.len(), oracle.rows.len());
    assert_eq!(ids, rows.keys().copied().collect());

    for case in &suite.cases {
        let expected = rows[case.id.as_str()];
        let actual = evaluate_epistemic_bounty(&case.facts);
        assert_eq!(actual.disposition.as_str(), expected.expected_disposition, "{}: {}", case.id, case.title);
        assert_eq!(actual.reason_codes.first().map(String::as_str), Some(expected.reason_code.as_str()), "{} reason", case.id);
    }
}

#[test]
fn restoration_domain_cases_match_separate_oracle() {
    let suite: DomainScenarioSuite<RestorationFacts> =
        serde_json::from_str(RESTORATION_SCENARIOS).expect("restoration cases must parse");
    let oracle: DomainOracleSuite =
        serde_json::from_str(RESTORATION_ORACLE).expect("restoration oracle must parse");
    assert_eq!(suite.case_count, suite.cases.len());
    assert_eq!(oracle.case_count, oracle.rows.len());
    assert_eq!(suite.suite_id, oracle.suite_id);
    assert_eq!(suite.cases.len(), 12);

    let ids: BTreeSet<_> = suite.cases.iter().map(|case| case.id.as_str()).collect();
    let rows: BTreeMap<_, _> = oracle.rows.iter().map(|row| (row.case_id.as_str(), row)).collect();
    assert_eq!(ids.len(), suite.cases.len());
    assert_eq!(rows.len(), oracle.rows.len());
    assert_eq!(ids, rows.keys().copied().collect());

    for case in &suite.cases {
        let expected = rows[case.id.as_str()];
        let actual = evaluate_restoration_contract(&case.facts);
        assert_eq!(actual.disposition.as_str(), expected.expected_disposition, "{}: {}", case.id, case.title);
        assert_eq!(actual.reason_codes.first().map(String::as_str), Some(expected.reason_code.as_str()), "{} reason", case.id);
    }
}

#[test]
fn epistemic_similarity_never_becomes_a_scientific_misconduct_verdict() {
    let suite: DomainScenarioSuite<EpistemicBountyFacts> =
        serde_json::from_str(EPISTEMIC_SCENARIOS).unwrap();
    let benign = suite.cases.iter().find(|case| case.id == "EB01").unwrap();
    let mut facts = benign.facts.clone();
    facts.request_sanction = Some(true);
    facts.similarity_high = Some(true);
    facts.coordination_corroborated = Some(false);
    assert_eq!(evaluate_epistemic_bounty(&facts).disposition, Disposition::Deny);
}

#[test]
fn restoration_does_not_trade_biodiversity_for_canopy_or_local_gain_for_downstream_harm() {
    let suite: DomainScenarioSuite<RestorationFacts> =
        serde_json::from_str(RESTORATION_SCENARIOS).unwrap();
    let biodiversity = suite.cases.iter().find(|case| case.id == "RST02").unwrap();
    let downstream = suite.cases.iter().find(|case| case.id == "RST03").unwrap();
    assert_eq!(evaluate_restoration_contract(&biodiversity.facts).disposition, Disposition::Deny);
    assert_eq!(evaluate_restoration_contract(&downstream.facts).disposition, Disposition::Deny);
}

#[test]
fn epistemic_reward_cannot_bypass_local_authority_recipient_or_rights() {
    let suite: DomainScenarioSuite<EpistemicBountyFacts> =
        serde_json::from_str(EPISTEMIC_SCENARIOS).unwrap();
    let reward = suite.cases.iter().find(|case| case.id == "EB09").unwrap();

    let mut no_authority = reward.facts.clone();
    no_authority.local_authority_valid = Some(false);
    assert_eq!(evaluate_epistemic_bounty(&no_authority).disposition, Disposition::Deny);

    let mut no_recipient = reward.facts.clone();
    no_recipient.recipient_eligible = Some(false);
    assert_eq!(evaluate_epistemic_bounty(&no_recipient).disposition, Disposition::Deny);

    let mut unknown_constitution = reward.facts.clone();
    unknown_constitution.constitution_current = None;
    assert_eq!(evaluate_epistemic_bounty(&unknown_constitution).disposition, Disposition::Indeterminate);
}

#[test]
fn restoration_tranche_requires_affected_party_rights_and_remedy() {
    let suite: DomainScenarioSuite<RestorationFacts> =
        serde_json::from_str(RESTORATION_SCENARIOS).unwrap();
    let tranche = suite.cases.iter().find(|case| case.id == "RST01").unwrap();

    let mut rights_denied = tranche.facts.clone();
    rights_denied.affected_party_rights_satisfied = Some(false);
    assert_eq!(evaluate_restoration_contract(&rights_denied).disposition, Disposition::Deny);

    let mut remedy_missing = tranche.facts.clone();
    remedy_missing.remedy_path_defined = Some(false);
    assert_eq!(evaluate_restoration_contract(&remedy_missing).disposition, Disposition::Deny);
}

#[test]
fn epistemic_reward_amount_must_fit_the_exact_ceiling_and_currency() {
    let suite: DomainScenarioSuite<EpistemicBountyFacts> =
        serde_json::from_str(EPISTEMIC_SCENARIOS).unwrap();
    let reward = suite.cases.iter().find(|case| case.id == "EB09").unwrap();

    let mut over_ceiling = reward.facts.clone();
    over_ceiling.amount_units = Some(1_001);
    assert_eq!(evaluate_epistemic_bounty(&over_ceiling).disposition, Disposition::Deny);

    let mut unknown_amount = reward.facts.clone();
    unknown_amount.amount_ceiling_units = None;
    assert_eq!(evaluate_epistemic_bounty(&unknown_amount).disposition, Disposition::Indeterminate);

    let mut invalid_funding = reward.facts.clone();
    invalid_funding.funding_source_valid = Some(false);
    assert_eq!(evaluate_epistemic_bounty(&invalid_funding).disposition, Disposition::Deny);
}

#[test]
fn restoration_tranche_must_fit_remaining_budget_and_valid_funding_source() {
    let suite: DomainScenarioSuite<RestorationFacts> =
        serde_json::from_str(RESTORATION_SCENARIOS).unwrap();
    let tranche = suite.cases.iter().find(|case| case.id == "RST01").unwrap();

    let mut over_budget = tranche.facts.clone();
    over_budget.tranche_amount_units = Some(1_001);
    assert_eq!(evaluate_restoration_contract(&over_budget).disposition, Disposition::Deny);

    let mut unknown_budget = tranche.facts.clone();
    unknown_budget.remaining_budget_units = None;
    assert_eq!(evaluate_restoration_contract(&unknown_budget).disposition, Disposition::Indeterminate);

    let mut invalid_funding = tranche.facts.clone();
    invalid_funding.funding_source_valid = Some(false);
    assert_eq!(evaluate_restoration_contract(&invalid_funding).disposition, Disposition::Deny);
}
