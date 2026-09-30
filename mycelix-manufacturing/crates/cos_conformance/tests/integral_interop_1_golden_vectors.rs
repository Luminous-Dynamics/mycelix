use cos_conformance::canonical_derivation_receipt::{canonical_bytes, canonical_sha256, D6S_REFERENCE_CANONICALIZATION_VERSION};
use serde_json::Value;

#[derive(Debug, serde::Deserialize)]
struct Expected {
    fixture_id: String,
    canonicalization_version: String,
    hash_domain: String,
    canonical_utf8: String,
    domain_separated_sha256: String,
}

#[test]
fn integral_interop_1_design_vector_is_cross_runtime_stable() {
    let value: Value = serde_json::from_str(include_str!("../testdata/integral_interop_1_design.json"))
        .expect("Integral interop fixture must be valid JSON");
    let expected: Expected =
        serde_json::from_str(include_str!("../testdata/integral_interop_1_design.expected.json"))
            .expect("Integral interop expected vector must be valid JSON");

    assert_eq!(expected.fixture_id, "Integral-Interop-1");
    assert_eq!(expected.canonicalization_version, D6S_REFERENCE_CANONICALIZATION_VERSION);

    let bytes = canonical_bytes(&value).expect("Integral fixture must be canonicalizable");
    assert_eq!(
        String::from_utf8(bytes.clone()).expect("D6S canonical bytes must be UTF-8"),
        expected.canonical_utf8
    );
    assert_eq!(
        canonical_sha256(&expected.hash_domain, &value),
        expected.domain_separated_sha256
    );
}

#[test]
fn integral_interop_1_object_member_order_is_semantically_irrelevant() {
    let original: Value =
        serde_json::from_str(include_str!("../testdata/integral_interop_1_design.json"))
            .expect("fixture must be valid JSON");

    let mut reordered = original.clone();
    let object = reordered.as_object_mut().expect("fixture root must be an object");

    let source = object.remove("source").expect("source field must exist");
    let certification = object.remove("certification").expect("certification field must exist");
    let design_version = object.remove("design_version").expect("design_version field must exist");
    let status = object.remove("status").expect("status field must exist");
    let fixture_version = object.remove("fixture_version").expect("fixture_version field must exist");
    let fixture_id = object.remove("fixture_id").expect("fixture_id field must exist");

    object.insert("fixture_id".into(), fixture_id);
    object.insert("fixture_version".into(), fixture_version);
    object.insert("status".into(), status);
    object.insert("design_version".into(), design_version);
    object.insert("certification".into(), certification);
    object.insert("source".into(), source);

    assert_eq!(
        canonical_bytes(&original).expect("original must canonicalize"),
        canonical_bytes(&reordered).expect("reordered object must canonicalize")
    );
}

#[test]
fn integral_interop_1_production_step_mutation_changes_identity() {
    let mut value: Value =
        serde_json::from_str(include_str!("../testdata/integral_interop_1_design.json"))
            .expect("fixture must be valid JSON");

    let baseline = canonical_sha256("integral-interop-1", &value);

    value["design_version"]["parameters"]["production_steps"][1]["estimated_hours"] = Value::from(3);

    let mutated = canonical_sha256("integral-interop-1", &value);
    assert_ne!(baseline, mutated, "selected production-step mutation must change identity");
}
