use cos_conformance::{canonical_bytes, d6s_raw_json::parse_d6s_canon_json, D6S_HASH_DOMAIN, D6S_REFERENCE_CANONICALIZATION_VERSION};
use sha2::{Digest, Sha256};
use serde_json::Value;
use std::fs;
use std::path::PathBuf;

#[derive(Debug, serde::Deserialize)]
struct GoldenCorpus {
    profile: String,
    hash_domain: String,
    cases: Vec<GoldenCase>,
    rejections: Vec<GoldenRejection>,
}

#[derive(Debug, serde::Deserialize)]
struct GoldenCase {
    name: String,
    value: Value,
    canonical: String,
    commitment: String,
}

#[derive(Debug, serde::Deserialize)]
struct GoldenRejection {
    name: String,
    json: String,
}

fn corpus_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../docs/integral/d6s-canon-1-golden-vectors.json")
}

fn commitment(canonical: &[u8]) -> String {
    let mut input = Vec::with_capacity(D6S_HASH_DOMAIN.len() + canonical.len());
    input.extend_from_slice(D6S_HASH_DOMAIN);
    input.extend_from_slice(canonical);
    format!("{:x}", Sha256::digest(input))
}

#[test]
fn rust_canonicalizer_matches_frozen_d6s_golden_vectors() {
    let bytes = fs::read(corpus_path()).expect("D6S golden corpus must exist");
    let corpus: GoldenCorpus =
        serde_json::from_slice(&bytes).expect("D6S golden corpus must parse");

    assert_eq!(corpus.profile, D6S_REFERENCE_CANONICALIZATION_VERSION);
    assert_eq!(
        corpus.hash_domain.as_bytes(),
        D6S_HASH_DOMAIN,
        "golden corpus hash domain must match the Rust reference"
    );
    assert_eq!(corpus.cases.len(), 8);
    assert_eq!(corpus.rejections.len(), 9);

    for case in corpus.cases {
        let actual = canonical_bytes(&case.value).expect("golden value must canonicalize");
        assert_eq!(
            actual,
            case.canonical.as_bytes(),
            "{}: Rust canonical bytes diverged from frozen vector",
            case.name
        );
        assert_eq!(
            commitment(&actual),
            case.commitment,
            "{}: Rust commitment diverged from frozen vector",
            case.name
        );
    }
}

#[test]
fn rust_raw_parser_rejects_every_frozen_d6s_rejection_vector() {
    let bytes = fs::read(corpus_path()).expect("D6S golden corpus must exist");
    let corpus: GoldenCorpus =
        serde_json::from_slice(&bytes).expect("D6S golden corpus must parse");

    for case in corpus.rejections {
        assert!(
            parse_d6s_canon_json(case.json.as_bytes()).is_err(),
            "{}: raw D6S input was unexpectedly accepted",
            case.name
        );
    }
}

#[test]
fn typed_canonicalizer_remains_distinct_from_raw_lexical_validation() {
    let typed = serde_json::json!({"value": 1});
    assert!(canonical_bytes(&typed).is_ok());

    // serde_json::Value has already erased the lexical distinction between
    // inputs such as "1" and "1e0". The raw-input gate owns that distinction.
    let exponent = r#"1e0"#;
    assert!(parse_d6s_canon_json(exponent.as_bytes()).is_err());
}
