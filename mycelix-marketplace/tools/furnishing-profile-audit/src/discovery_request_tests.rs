use super::validate_discovery_request;
use serde_json::Value;
use std::path::PathBuf;

fn baseline() -> Value {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../contracts/fixtures/furnishing-adapter-request-v0.1.json");
    let bytes = std::fs::read(&path).expect("request fixture should exist");
    serde_json::from_slice(&bytes).expect("request fixture should be valid JSON")
}

fn errors_after(mut document: Value, mutate: impl FnOnce(&mut Value)) -> Vec<String> {
    mutate(&mut document);
    validate_discovery_request(&document)
}

#[test]
fn frozen_discovery_request_is_effects_disabled() {
    let document = baseline();
    let errors = validate_discovery_request(&document);
    assert!(errors.is_empty(), "unexpected request errors: {errors:#?}");
}

#[test]
fn enabling_reservation_is_rejected() {
    let errors = errors_after(baseline(), |doc| {
        doc["effects"]["reserve_stock"] = Value::Bool(true);
    });
    assert!(errors
        .iter()
        .any(|error| error.contains("effect reserve_stock must be false")));
}

#[test]
fn enabling_order_creation_is_rejected() {
    let errors = errors_after(baseline(), |doc| {
        doc["effects"]["create_order"] = Value::Bool(true);
    });
    assert!(errors
        .iter()
        .any(|error| error.contains("effect create_order must be false")));
}

#[test]
fn transmitting_exact_address_is_rejected_in_both_policy_layers() {
    let errors = errors_after(baseline(), |doc| {
        doc["data_minimization"]["exact_address_transmitted"] = Value::Bool(true);
        doc["effects"]["transmit_exact_address"] = Value::Bool(true);
    });
    assert!(errors
        .iter()
        .any(|error| error.contains("exact_address_transmitted must be false")));
    assert!(errors
        .iter()
        .any(|error| error.contains("effect transmit_exact_address must be false")));
}

#[test]
fn transmitting_floor_plan_is_rejected() {
    let errors = errors_after(baseline(), |doc| {
        doc["design_snapshot"]["full_geometry_transmission"] = Value::Bool(true);
        doc["data_minimization"]["floor_plan_transmitted"] = Value::Bool(true);
        doc["effects"]["upload_floor_plan"] = Value::Bool(true);
    });
    assert!(errors
        .iter()
        .any(|error| error.contains("full geometry transmission must be false")));
    assert!(errors
        .iter()
        .any(|error| error.contains("floor_plan_transmitted must be false")));
    assert!(errors
        .iter()
        .any(|error| error.contains("effect upload_floor_plan must be false")));
}

#[test]
fn purchase_authorization_cannot_be_smuggled_into_discovery() {
    let errors = errors_after(baseline(), |doc| {
        doc["authorization"]["purchase_authorization_state"] =
            Value::String("authorized".to_owned());
    });
    assert!(errors
        .iter()
        .any(|error| error.contains("purchase authorization must not be requested")));
}

#[test]
fn unknown_query_fields_fail_closed() {
    let errors = errors_after(baseline(), |doc| {
        doc["data_minimization"]["fields_permitted_for_query"]
            .as_array_mut()
            .unwrap()
            .push(Value::String("exact_residential_address".to_owned()));
    });
    assert!(errors
        .iter()
        .any(|error| error.contains("exceeds the allowlist")));
}

#[test]
fn design_and_query_digests_must_remain_local() {
    let errors = errors_after(baseline(), |doc| {
        doc["data_minimization"]["design_context_digest_transmitted"] = Value::Bool(true);
        doc["data_minimization"]["query_digest_transmitted"] = Value::Bool(true);
    });
    assert!(errors
        .iter()
        .any(|error| error.contains("design_context_digest_transmitted must be false")));
    assert!(errors
        .iter()
        .any(|error| error.contains("query_digest_transmitted must be false")));
}

#[test]
fn malformed_design_digest_is_rejected() {
    let errors = errors_after(baseline(), |doc| {
        doc["design_snapshot"]["design_context_digest_sha256"] =
            Value::String("short".to_owned());
    });
    assert!(errors
        .iter()
        .any(|error| error.contains("malformed design_context_digest_sha256")));
}

#[test]
fn non_metric_dimensions_are_rejected() {
    let errors = errors_after(baseline(), |doc| {
        doc["design_snapshot"]["dimensions_unit"] = Value::String("feet".to_owned());
    });
    assert!(errors
        .iter()
        .any(|error| error.contains("dimensions must be expressed in metres")));
}

#[test]
fn seller_contact_is_not_a_discovery_side_effect() {
    let errors = errors_after(baseline(), |doc| {
        doc["effects"]["contact_seller"] = Value::Bool(true);
    });
    assert!(errors
        .iter()
        .any(|error| error.contains("effect contact_seller must be false")));
}
