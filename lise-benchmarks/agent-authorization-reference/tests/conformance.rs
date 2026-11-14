use agent_authorization_reference::{evaluate, Authority, Disposition, OracleSuite, ScenarioSuite};
use std::collections::{BTreeMap, BTreeSet};
const SCENARIOS: &str = include_str!("../fixtures/scenarios.json");
const ORACLE: &str = include_str!("../fixtures/oracle.json");

#[test]
fn all_cases_match_separate_oracle_on_both_axes() {
    let suite: ScenarioSuite = serde_json::from_str(SCENARIOS).expect("scenario JSON parses");
    let oracle: OracleSuite = serde_json::from_str(ORACLE).expect("oracle JSON parses");
    assert_eq!(suite.case_count, suite.cases.len());
    assert_eq!(oracle.case_count, oracle.rows.len());
    assert_eq!(suite.suite_id, oracle.suite_id);
    assert_eq!(suite.cases.len(), 26);
    let ids: BTreeSet<_> = suite.cases.iter().map(|x| x.id.as_str()).collect();
    let rows: BTreeMap<_, _> = oracle.rows.iter().map(|x| (x.case_id.as_str(), x)).collect();
    assert_eq!(ids.len(), suite.cases.len());
    assert_eq!(rows.len(), oracle.rows.len());
    assert_eq!(ids, rows.keys().copied().collect());
    for case in &suite.cases {
        let expected = rows[case.id.as_str()];
        let actual = evaluate(case);
        assert_eq!(actual.task_outcome, expected.expected_task_outcome, "{}", case.id);
        assert_eq!(actual.disposition, expected.expected_disposition, "{} {}", case.id, case.title);
        assert_eq!(actual.reason_code, expected.reason_code, "{} reason", case.id);
    }
}
#[test]
fn parser_rejects_unknown_top_level_fields() {
    let invalid = r#"{"schema_version":"x","suite_id":"x","status":"x","case_count":0,"cases":[],"surprise":true}"#;
    assert!(serde_json::from_str::<ScenarioSuite>(invalid).is_err());
}
#[test]
fn parser_rejects_unknown_nested_authority_fields() {
    let invalid = r#"{"status":"valid","principal_id":"p","actor_id":"a","action":"x","resource":"r","purpose":"p","operation_digest":"d","recipient":"r","maximum_amount_minor":10,"currency":"USD","constitutional_epoch":1,"valid_from_unix":0,"expires_at_unix":10,"delegated":false,"parent_grant":null,"admin":true}"#;
    assert!(serde_json::from_str::<Authority>(invalid).is_err());
}
#[test]
fn task_failure_can_be_authorization_compliant() {
    let s: ScenarioSuite=serde_json::from_str(SCENARIOS).unwrap();
    let c=s.cases.iter().find(|x|x.id=="A11").unwrap();
    assert_eq!(c.task_outcome.as_str(),"failed");
    assert_eq!(evaluate(c).disposition,Disposition::Permit);
}
#[test]
fn task_success_cannot_upgrade_collateral_write() {
    let s: ScenarioSuite=serde_json::from_str(SCENARIOS).unwrap();
    let c=s.cases.iter().find(|x|x.id=="A10").unwrap();
    assert_eq!(c.task_outcome.as_str(),"succeeded");
    assert_eq!(evaluate(c).disposition,Disposition::Deny);
}
#[test]
fn clarification_halts_ambiguous_action_before_dispatch() {
    let s: ScenarioSuite=serde_json::from_str(SCENARIOS).unwrap();
    let c=s.cases.iter().find(|x|x.id=="A02").unwrap();
    assert!(!c.execution.attempted);
    assert_eq!(evaluate(c).disposition,Disposition::ChallengeRequired);
}


#[test]
fn no_primary_attempt_does_not_hide_unknown_or_partial_effects() {
    let suite: ScenarioSuite = serde_json::from_str(SCENARIOS).unwrap();
    for id in ["A23", "A24"] {
        let case = suite.cases.iter().find(|case| case.id == id).unwrap();
        assert!(!case.execution.attempted);
        assert_eq!(evaluate(case).disposition, Disposition::Hold, "{id}");
    }
}
#[test]
fn collateral_effect_is_denied_even_when_primary_operation_was_not_attempted() {
    let suite: ScenarioSuite = serde_json::from_str(SCENARIOS).unwrap();
    let case = suite.cases.iter().find(|case| case.id == "A25").unwrap();
    assert!(!case.execution.attempted);
    assert_eq!(case.execution.unauthorized_collateral_effects, 1);
    assert_eq!(evaluate(case).disposition, Disposition::Deny);
}
#[test]
fn committed_effect_cannot_be_silently_coerced_to_not_attempted() {
    let suite: ScenarioSuite = serde_json::from_str(SCENARIOS).unwrap();
    let case = suite.cases.iter().find(|case| case.id == "A26").unwrap();
    assert!(!case.execution.attempted);
    assert_eq!(case.execution.effect_status, agent_authorization_reference::EffectStatus::Committed);
    assert_eq!(evaluate(case).disposition, Disposition::Indeterminate);
}
