use mycelix_finance_sync_graph::SettlementGraphInputV1;

fn fixture_input() -> serde_json::Value {
    let raw = include_str!("../test-vectors/settlement-graph-v1.json");
    let fixture: serde_json::Value = serde_json::from_str(raw).expect("valid frozen fixture");
    fixture["input"].clone()
}

#[test]
fn dependency_unknown_fields_fail_closed() {
    let mut input = fixture_input();
    input["dependencies"][0]["settled"] = serde_json::Value::Bool(true);

    let encoded = serde_json::to_string(&input).expect("encode mutated fixture");
    let error = serde_json::from_str::<SettlementGraphInputV1>(&encoded)
        .expect_err("unknown dependency field must be rejected");

    assert!(
        error.to_string().contains("unknown field"),
        "unexpected serde error: {error}"
    );
}

#[test]
fn dependency_unknown_fields_cannot_hide_authority_claims() {
    let mut input = fixture_input();
    input["dependencies"][0]["authorized"] = serde_json::Value::Bool(true);
    input["dependencies"][0]["provider_receipt"] =
        serde_json::Value::String("opaque-provider-receipt".to_owned());

    let encoded = serde_json::to_string(&input).expect("encode mutated fixture");
    assert!(
        serde_json::from_str::<SettlementGraphInputV1>(&encoded).is_err(),
        "runtime/evidence fields must not be silently discarded from dependency input"
    );
}
