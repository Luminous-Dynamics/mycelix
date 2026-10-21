use nrvca_validator::validate_report;
use serde_json::Value;

const GOLDEN: &str =
    include_str!("../../../../fixtures/nrvca-synthetic-golden-v0.1.json");

#[test]
fn financial_liability_revaluation_requires_explicit_event_semantics() {
    let mut report: Value = serde_json::from_str(GOLDEN).expect("golden report parses");
    report["stock_flow_accounts"][0]["changes"][3]["change_class"] =
        serde_json::json!("revaluation");

    let diagnostics = validate_report(&report);
    assert!(diagnostics
        .iter()
        .any(|d| d.code == "SEM-FINANCIAL-REVALUATION-CLASS"));
    assert!(diagnostics
        .iter()
        .any(|d| d.code == "SEM-STOCK-EVENT-CLASS"));
}
