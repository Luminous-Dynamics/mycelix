use cos_conformance::integral_interop::{
    selected_oad_design_semantic_projection, OAD_SEMANTIC_PROJECTION_VERSION,
};
use serde_json::{json, Value};

fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_design.json"
    ))
    .expect("fixture must be valid JSON")
}

#[test]
fn selected_projection_has_an_explicit_top_level_field_set() {
    let projection = selected_oad_design_semantic_projection(&fixture());
    let object = projection.as_object().expect("projection must be an object");

    let keys = object.keys().cloned().collect::<Vec<_>>();
    assert_eq!(
        keys,
        vec![
            "bill_of_materials_kg",
            "design_version_id",
            "materials",
            "production_steps",
            "spec_id",
        ]
    );
}

#[test]
fn selected_projection_has_an_explicit_production_step_field_set() {
    let projection = selected_oad_design_semantic_projection(&fixture());
    let steps = projection["production_steps"]
        .as_array()
        .expect("production_steps must be an array");

    for step in steps {
        let object = step.as_object().expect("step must be an object");
        let keys = object.keys().cloned().collect::<Vec<_>>();
        assert_eq!(
            keys,
            vec![
                "estimated_hours",
                "name",
                "safety_notes",
                "sequence_index",
                "skill_tier",
                "tools_required",
            ]
        );
    }
}

#[test]
fn projection_version_is_explicit_and_stable() {
    assert_eq!(
        OAD_SEMANTIC_PROJECTION_VERSION,
        "integral-interop-1-design-semantic-v1"
    );
}

#[test]
fn unselected_fixture_metadata_cannot_leak_into_projection() {
    let mut changed = fixture();
    changed["certification"]["status"] = Value::from("revoked");
    changed["source"]["model_basis"] = Value::from("changed");
    changed["design_version"]["change_log"] = Value::from("changed");

    assert_eq!(
        selected_oad_design_semantic_projection(&fixture()),
        selected_oad_design_semantic_projection(&changed),
        "metadata-only changes must remain outside selected D6X semantics"
    );
}

#[test]
fn selected_material_and_production_changes_do_leak_into_projection() {
    let baseline = selected_oad_design_semantic_projection(&fixture());
    let mut changed = fixture();
    changed["design_version"]["parameters"]["production_steps"][0]["estimated_hours"] =
        Value::from(9);
    changed["design_version"]["parameters"]["bill_of_materials_kg"]["stainless-steel"] =
        Value::from(3);

    let selected = selected_oad_design_semantic_projection(&changed);
    assert_ne!(baseline, selected);

    assert_eq!(
        selected["production_steps"][0]["estimated_hours"],
        json!(9)
    );
    assert_eq!(
        selected["bill_of_materials_kg"]["stainless-steel"],
        json!(3)
    );
}
