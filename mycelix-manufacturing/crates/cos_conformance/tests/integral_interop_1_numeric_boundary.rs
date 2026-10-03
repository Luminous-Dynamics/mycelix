use cos_conformance::canonical_derivation_receipt::canonical_bytes;
use cos_conformance::integral_interop::{
    selected_oad_design_semantic_projection,
    selected_oad_design_semantic_commitment,
};
use serde_json::Value;

fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_design.json"
    ))
    .expect("Integral fixture must be valid JSON")
}

#[test]
fn selected_integral_decimal_semantics_are_fail_closed_under_d6s_canon_1() {
    let mut changed = fixture();
    changed["design_version"]["parameters"]["bill_of_materials_kg"]["silicone"] =
        Value::from(0.25);

    let projection = selected_oad_design_semantic_projection(&changed);

    assert!(
        canonical_bytes(&projection).is_err(),
        "D6S-CANON-1 must not silently serialize fractional Integral semantics"
    );
}

#[test]
fn selected_integral_integer_semantics_remain_canonicalizable() {
    let baseline = fixture();
    let projection = selected_oad_design_semantic_projection(&baseline);

    assert!(canonical_bytes(&projection).is_ok());
}

#[test]
fn production_step_array_order_is_semantic() {
    let baseline = fixture();
    let mut changed = baseline.clone();

    let steps = changed["design_version"]["parameters"]["production_steps"]
        .as_array_mut()
        .expect("production_steps must be an array");
    steps.swap(0, 1);

    assert_ne!(
        selected_oad_design_semantic_commitment(&baseline),
        selected_oad_design_semantic_commitment(&changed),
        "ordered production steps must not be normalized as an unordered set"
    );
}
