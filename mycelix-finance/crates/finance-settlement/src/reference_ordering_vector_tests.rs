use std::collections::BTreeSet;

use mycelix_business_core::ReferenceId;
use serde_json::Value;

use crate::{FinalityProfile, ReversalModel, canonical_finality_profile_bytes};

fn reference(value: &str) -> ReferenceId {
    ReferenceId::new(value).expect("fixture reference")
}

fn fixture() -> Value {
    serde_json::from_str(include_str!("../test-vectors/reference-ordering-v1.json"))
        .expect("checked-in reference-ordering vector must parse")
}

fn bytes_hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

#[test]
fn reference_sets_sort_raw_utf8_before_length_encoding() {
    let fixture = fixture();
    let physical = fixture["physical_input_references"]
        .as_array()
        .expect("physical input references");

    let mut evidence_kinds = BTreeSet::new();
    for value in physical {
        evidence_kinds.insert(reference(value.as_str().expect("reference string")));
    }

    let expected_order: Vec<&str> = fixture["expected_unique_order"]
        .as_array()
        .expect("expected order")
        .iter()
        .map(|value| value.as_str().expect("ordered reference string"))
        .collect();
    let actual_order: Vec<&str> = evidence_kinds.iter().map(ReferenceId::as_str).collect();
    assert_eq!(actual_order, expected_order);

    let profile_fixture = &fixture["profile"];
    let profile = FinalityProfile::new(
        reference(profile_fixture["id"].as_str().expect("profile id")),
        profile_fixture["revision"].as_u64().expect("profile revision"),
        reference(profile_fixture["rail"].as_str().expect("rail")),
        reference(profile_fixture["network"].as_str().expect("network")),
        evidence_kinds,
        u16::try_from(
            profile_fixture["min_distinct_sources"]
                .as_u64()
                .expect("minimum distinct sources"),
        )
        .expect("u16 minimum distinct sources"),
        profile_fixture["max_observation_age_ms"]
            .as_u64()
            .expect("maximum observation age"),
        ReversalModel::MayReverse,
    )
    .expect("ordering profile");

    let canonical = canonical_finality_profile_bytes(&profile).expect("canonical ordering profile");
    assert_eq!(
        canonical.len(),
        usize::try_from(
            profile_fixture["canonical_length"]
                .as_u64()
                .expect("canonical length"),
        )
        .expect("usize canonical length")
    );
    assert_eq!(
        bytes_hex(&canonical),
        profile_fixture["canonical_hex"]
            .as_str()
            .expect("canonical hex")
    );
    assert_eq!(
        bytes_hex(&profile.profile_ref().digest.0),
        profile_fixture["commitment_hex"]
            .as_str()
            .expect("commitment hex")
    );
}
