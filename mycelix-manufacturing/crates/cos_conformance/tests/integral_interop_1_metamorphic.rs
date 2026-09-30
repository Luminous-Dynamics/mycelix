use serde_json::{Map, Value};

use cos_conformance::canonical_derivation_receipt::canonical_sha256;
use cos_conformance::integral_interop::{
    selected_oad_design_semantic_commitment_checked, selected_oad_design_semantic_projection,
};

fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_design.json"
    ))
    .expect("Integral fixture must be valid JSON")
}

#[test]
fn object_order_does_not_change_semantic_commitment() {
    let baseline = fixture();
    let mut reordered = baseline.clone();

    let original = reordered.as_object().unwrap().clone();
    let mut reversed = Map::new();
    for (key, value) in original.into_iter().rev() {
        reversed.insert(key, value);
    }
    reordered = Value::Object(reversed);

    assert_eq!(
        selected_oad_design_semantic_commitment_checked(&baseline).unwrap(),
        selected_oad_design_semantic_commitment_checked(&reordered).unwrap()
    );
}

#[test]
fn irrelevant_metadata_and_unknown_top_level_fields_do_not_change_commitment() {
    let baseline = fixture();

    let mut metadata = baseline.clone();
    metadata["design_version"]["change_log"] = Value::String("unrelated revision".into());
    metadata["certification"]["documentation_bundle_uri"] =
        Value::String("urn:integral:bundle:unrelated".into());

    let mut unknown = baseline.clone();
    unknown["future_top_level_field"] = serde_json::json!({
        "must_not": "enter the frozen semantic projection"
    });

    let expected = selected_oad_design_semantic_commitment_checked(&baseline).unwrap();
    assert_eq!(
        expected,
        selected_oad_design_semantic_commitment_checked(&metadata).unwrap()
    );
    assert_eq!(
        expected,
        selected_oad_design_semantic_commitment_checked(&unknown).unwrap()
    );
}

#[test]
fn production_step_object_order_does_not_change_commitment() {
    let baseline = fixture();
    let mut reordered = baseline.clone();

    let step = reordered["design_version"]["parameters"]["production_steps"][0]
        .as_object()
        .unwrap()
        .clone();
    let mut reverse_order = Map::new();
    for (key, value) in step.into_iter().rev() {
        reverse_order.insert(key, value);
    }
    reordered["design_version"]["parameters"]["production_steps"][0] =
        Value::Object(reverse_order);

    assert_eq!(
        selected_oad_design_semantic_commitment_checked(&baseline).unwrap(),
        selected_oad_design_semantic_commitment_checked(&reordered).unwrap()
    );
}

#[test]
fn production_step_array_order_is_semantic() {
    let baseline = fixture();
    let mut changed = baseline.clone();
    changed["design_version"]["parameters"]["production_steps"]
        .as_array_mut()
        .unwrap()
        .swap(0, 1);

    assert_ne!(
        selected_oad_design_semantic_commitment_checked(&baseline).unwrap(),
        selected_oad_design_semantic_commitment_checked(&changed).unwrap()
    );
}

#[test]
fn selected_semantic_mutation_changes_commitment() {
    let baseline = fixture();
    let mut changed = baseline.clone();
    changed["design_version"]["parameters"]["production_steps"][1]["estimated_hours"] =
        Value::from(3);

    assert_ne!(
        selected_oad_design_semantic_commitment_checked(&baseline).unwrap(),
        selected_oad_design_semantic_commitment_checked(&changed).unwrap()
    );
}

#[test]
fn projection_version_is_a_commitment_domain_boundary() {
    let baseline = fixture();
    let projection = selected_oad_design_semantic_projection(&baseline);

    assert_ne!(
        canonical_sha256("integral-interop-1-design-semantic-v1", &projection),
        canonical_sha256("integral-interop-1-design-semantic-v2", &projection)
    );
}

#[test]
fn invalid_selected_input_produces_no_commitment() {
    let mut fractional = fixture();
    fractional["design_version"]["parameters"]["bill_of_materials_kg"]["silicone"] =
        serde_json::json!(0.25);
    assert!(selected_oad_design_semantic_commitment_checked(&fractional).is_err());

    let mut unknown_step_field = fixture();
    unknown_step_field["design_version"]["parameters"]["production_steps"][0]["future_field"] =
        Value::String("not in the frozen projection".into());
    assert!(selected_oad_design_semantic_commitment_checked(&unknown_step_field).is_err());
}
