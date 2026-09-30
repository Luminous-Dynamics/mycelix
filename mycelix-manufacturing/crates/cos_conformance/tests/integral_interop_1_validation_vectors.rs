use serde_json::Value;

use cos_conformance::integral_interop::{
    selected_oad_design_semantic_commitment_checked, validate_selected_oad_design_semantics,
};

fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_design.json"
    ))
    .expect("Integral fixture must be valid JSON")
}

#[test]
fn validation_corpus_is_machine_readable_and_self_describing() {
    let corpus: Value = serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_validation_vectors.json"
    ))
    .expect("validation corpus must be valid JSON");

    assert_eq!(corpus["profile"], "integral-interop-1");
    assert_eq!(
        corpus["projection_version"],
        "integral-interop-1-design-semantic-v1"
    );
    assert!(corpus["vectors"].is_array());
    assert!(!corpus["vectors"].as_array().unwrap().is_empty());
}

#[test]
fn baseline_vector_accepts() {
    let value = fixture();
    assert!(validate_selected_oad_design_semantics(&value).is_ok());
    assert!(selected_oad_design_semantic_commitment_checked(&value).is_ok());
}

#[test]
fn adversarial_validation_vectors_match_the_declared_boundary() {
    let mut value = fixture();

    value["design_version"]["materials"] = Value::String("wrong".into());
    assert!(validate_selected_oad_design_semantics(&value).is_err());

    let mut value = fixture();
    value["design_version"]["parameters"]["bill_of_materials_kg"]["silicone"] =
        serde_json::json!(0.25);
    assert!(validate_selected_oad_design_semantics(&value).is_err());

    let mut value = fixture();
    value["design_version"]["parameters"]["production_steps"][0]["estimated_hours"] =
        serde_json::json!(1.5);
    assert!(validate_selected_oad_design_semantics(&value).is_err());

    let mut value = fixture();
    value["design_version"]["parameters"]["production_steps"][0]["tools_required"] =
        Value::String("press".into());
    assert!(validate_selected_oad_design_semantics(&value).is_err());

    let mut value = fixture();
    value["design_version"]["parameters"]["production_steps"][0]["sequence_index"] =
        serde_json::json!(0.5);
    assert!(validate_selected_oad_design_semantics(&value).is_err());
}
