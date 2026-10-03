use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::fs;
use std::path::PathBuf;

fn fixture_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../docs/integral/d6s-canon-2-authority-boundary-fixture.json")
}

fn fixture_manifest_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../docs/integral/d6s-canon-2-manifest.json")
}

fn canon1_manifest_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../docs/integral/d6s-canon-1-manifest.json")
}

fn canon1_corpus_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../docs/integral/d6s-canon-1-golden-vectors.json")
}

fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

fn expected_cases() -> BTreeMap<&'static str, (&'static str, &'static str)> {
    [
        ("author-grant", ("capability-authorization", "authorized")),
        ("authorized-semantic-rejection", ("zome-semantic-validation", "authorized")),
        ("blocked-provenance", ("capability-authorization", "not-authorized")),
        ("canonical-payload-accepted", ("capability-authorization", "authorized")),
        ("expired-invocation", ("invocation-expiry", "not-authorized")),
        ("nonce-replay", ("nonce-replay-protection", "not-authorized")),
        ("nonce-stale", ("nonce-replay-protection", "not-authorized")),
        ("payload-mutation", ("d6s-integrity", "authenticated-not-authorized")),
        ("provenance-mismatch", ("capability-authorization", "not-authorized")),
        ("revoked-capability", ("capability-authorization", "not-authorized")),
        ("valid-capability", ("capability-authorization", "authorized")),
        ("wire-signature-invalid", ("wire-authentication", "unauthenticated")),
        ("wire-signature-valid", ("wire-authentication", "authenticated-not-authorized")),
        ("wrong-capability", ("capability-authorization", "not-authorized")),
        ("wrong-cell", ("invocation-routing-or-binding", "not-authorized")),
        ("wrong-function", ("invocation-binding-or-authorization", "not-authorized")),
        ("wrong-zome", ("invocation-binding-or-authorization", "not-authorized")),
    ]
    .into_iter()
    .collect()
}

#[test]
fn d6s_canon_2_fixture_is_complete_and_claim_bounded() {
    let bytes = fs::read(fixture_path()).expect("D6S-CANON-2 fixture must exist");
    let fixture: Value =
        serde_json::from_slice(&bytes).expect("D6S-CANON-2 fixture must parse");

    let manifest_bytes =
        fs::read(fixture_manifest_path()).expect("D6S-CANON-2 manifest must exist");
    let manifest: Value =
        serde_json::from_slice(&manifest_bytes).expect("D6S-CANON-2 manifest must parse");

    let canon1_manifest_bytes =
        fs::read(canon1_manifest_path()).expect("D6S-CANON-1 manifest must exist");
    let canon1_manifest: Value =
        serde_json::from_slice(&canon1_manifest_bytes).expect("D6S-CANON-1 manifest must parse");

    let canon1_corpus_bytes =
        fs::read(canon1_corpus_path()).expect("D6S-CANON-1 corpus must exist");

    assert_eq!(fixture["profile"], "D6S-CANON-2");
    assert_eq!(fixture["kind"], "authority-boundary-reference-fixture");
    assert_eq!(fixture["version"], 1);
    assert_eq!(
        fixture["depends_on"]["canonicalization_profile"],
        "D6S-CANON-1"
    );
    assert_eq!(
        fixture["depends_on"]["claim_ceiling"],
        "ReferenceModelOnly"
    );

    assert_eq!(manifest["profile"], "D6S-CANON-2");
    assert_eq!(
        manifest["fixture_blob_sha"],
        "0b4c5b924bc3b807c556ef92cc869d66f7cc97a1"
    );
    assert_eq!(
        manifest["dependencies"]["d6s_canon_1_manifest_blob_sha"],
        "53d88a67c48c8adbf783d1df290b3c591cd15566"
    );
    assert_eq!(
        manifest["dependencies"]["d6s_canon_1_corpus_blob_sha"],
        "5e359549623101fe5997d0cfe8c7acb9c50cd57b"
    );
    assert_eq!(
        sha256_hex(&canon1_corpus_bytes),
        "9d61cdb2e625c13c5813fffb7cceea4af2f93ed60d7d64c068dc6d3f6f6b614d"
    );
    assert_eq!(canon1_manifest["profile"], "D6S-CANON-1");
    assert_eq!(
        canon1_manifest["corpus_sha256"],
        sha256_hex(&canon1_corpus_bytes)
    );

    let cases = fixture["boundary"]
        .as_array()
        .expect("boundary must be an array");
    assert_eq!(cases.len(), 17);

    let actual: BTreeMap<&str, (&str, &str)> = cases
        .iter()
        .map(|case| {
            (
                case["case_id"].as_str().expect("case_id must be a string"),
                (
                    case["terminal_gate"]
                        .as_str()
                        .expect("terminal_gate must be a string"),
                    case["authority_state"]
                        .as_str()
                        .expect("authority_state must be a string"),
                ),
            )
        })
        .collect();
    assert_eq!(actual, expected_cases());

    for case in cases {
        assert_eq!(case["semantic_result"].is_null(), false);
        assert!(case["terminal_gate"].is_string());
        assert!(case["authority_state"].is_string());

        if case["zome_reached"] == Value::Bool(true) {
            assert_eq!(case["boundary_result"], "authorized");
            assert_eq!(case["authority_state"], "authorized");
            assert!(matches!(
                case["semantic_result"].as_str(),
                Some("subject-to-zome-validation") | Some("rejected-by-zome")
            ));
        } else {
            assert_ne!(case["authority_state"], "authorized");
        }
    }

    let signature_invalid = cases
        .iter()
        .find(|c| c["case_id"] == "wire-signature-invalid")
        .expect("wire-signature-invalid must exist");
    assert_eq!(
        signature_invalid["boundary_result"],
        "holochain-signature-authentication-rejection"
    );
    assert_eq!(signature_invalid["terminal_gate"], "wire-authentication");
    assert_eq!(signature_invalid["authority_state"], "unauthenticated");
    assert_eq!(signature_invalid["zome_reached"], false);
    assert_eq!(signature_invalid["semantic_result"], "not-reached");

    let signature_valid = cases
        .iter()
        .find(|c| c["case_id"] == "wire-signature-valid")
        .expect("wire-signature-valid must exist");
    assert_eq!(signature_valid["boundary_result"], "authenticated");
    assert_eq!(signature_valid["terminal_gate"], "wire-authentication");
    assert_eq!(signature_valid["authority_state"], "authenticated-not-authorized");
    assert_eq!(signature_valid["zome_reached"], false);

    let mutation = cases
        .iter()
        .find(|c| c["case_id"] == "payload-mutation")
        .expect("payload-mutation must exist");
    assert_eq!(
        mutation["payload_state"],
        "authenticated-but-d6s-commitment-inconsistent"
    );
    assert_eq!(mutation["terminal_gate"], "d6s-integrity");
    assert_eq!(mutation["authority_state"], "authenticated-not-authorized");

    let author_grant = cases
        .iter()
        .find(|c| c["case_id"] == "author-grant")
        .expect("author-grant must exist");
    assert_eq!(author_grant["boundary_result"], "authorized");
    assert_eq!(author_grant["zome_reached"], true);
    assert_eq!(author_grant["capability_state"], "author-grant");

    assert_eq!(fixture["depends_on"]["claim_ceiling"], "ReferenceModelOnly");
}
