use cos_conformance::canonical_derivation_receipt::{canonical_bytes, canonical_sha256, D6S_HASH_DOMAIN, D6S_REFERENCE_CANONICALIZATION_VERSION};
use serde::Deserialize;
use serde_json::Value;

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

#[derive(Debug, Deserialize)]
struct Fixture {
    canonicalization_version: String,
    hash_domain_prefix_hex: String,
    vectors: Vec<Vector>,
}

#[derive(Debug, Deserialize)]
struct Vector {
    id: String,
    domain: String,
    value: Value,
    canonical_json: String,
    sha256: String,
}

#[test]
fn machine_readable_d6s_canon_1_vectors_match_reference_implementation() {
    let fixture: Fixture = serde_json::from_str(include_str!("../testdata/d6s_canon_1_golden_vectors.json"))
        .expect("golden vector fixture must be valid JSON");

    assert_eq!(fixture.canonicalization_version, D6S_REFERENCE_CANONICALIZATION_VERSION);
    assert_eq!(
        fixture.hash_domain_prefix_hex,
        hex(D6S_HASH_DOMAIN),
        "fixture must bind the exact hash-domain prefix"
    );

    for vector in fixture.vectors {
        let bytes = canonical_bytes(&vector.value).expect("fixture value must be canonicalizable");
        assert_eq!(
            String::from_utf8(bytes.clone()).expect("canonical bytes must be UTF-8"),
            vector.canonical_json,
            "{} canonical bytes drifted",
            vector.id
        );
        assert_eq!(
            canonical_sha256(&vector.domain, &vector.value),
            vector.sha256,
            "{} commitment drifted",
            vector.id
        );
    }
}
