use serde_json::Value;
use std::collections::BTreeSet;
use std::fs;
use std::path::PathBuf;

fn fixture_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../docs/integral/d6s-canon-2-authority-boundary-fixture.json")
}

fn case_ids() -> BTreeSet<&'static str> {
    [
        "author-grant",
        "authorized-semantic-rejection",
        "blocked-provenance",
        "canonical-payload-accepted",
        "expired-invocation",
        "nonce-replay",
        "nonce-stale",
        "payload-mutation",
        "provenance-mismatch",
        "revoked-capability",
        "valid-capability",
        "wire-signature-invalid",
        "wire-signature-valid",
        "wrong-capability",
        "wrong-cell",
        "wrong-function",
        "wrong-zome",
    ]
    .into_iter()
    .collect()
}

#[test]
fn d6s_canon_2_fixture_is_complete_and_claim_bounded() {
    let bytes = fs::read(fixture_path()).expect("D6S-CANON-2 fixture must exist");
    let fixture: Value =
        serde_json::from_slice(&bytes).expect("D6S-CANON-2 fixture must parse");

    assert_eq!(fixture["profile"], "D6S-CANON-2");
    assert_eq!(fixture["kind"], "authority-boundary-reference-fixture");
    assert_eq!(
        fixture["depends_on"]["canonicalization_profile"],
        "D6S-CANON-1"
    );
    assert_eq!(
        fixture["depends_on"]["claim_ceiling"],
        "ReferenceModelOnly"
    );

    let cases = fixture["boundary"]
        .as_array()
        .expect("boundary must be an array");
    assert_eq!(cases.len(), 17);

    let actual: BTreeSet<&str> = cases
        .iter()
        .map(|case| case["case_id"].as_str().expect("case_id must be a string"))
        .collect();
    assert_eq!(actual, case_ids());

    for case in cases {
        assert_eq!(case["semantic_result"].is_null(), false);
        if case["zome_reached"] == Value::Bool(true) {
            assert_eq!(case["boundary_result"], "authorized");
            assert!(matches!(
                case["semantic_result"].as_str(),
                Some("subject-to-zome-validation") | Some("rejected-by-zome")
            ));
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
    assert_eq!(signature_invalid["zome_reached"], false);
    assert_eq!(signature_invalid["semantic_result"], "not-reached");

    let signature_valid = cases
        .iter()
        .find(|c| c["case_id"] == "wire-signature-valid")
        .expect("wire-signature-valid must exist");
    assert_eq!(signature_valid["boundary_result"], "authenticated");
    assert_eq!(signature_valid["zome_reached"], false);

    let author_grant = cases
        .iter()
        .find(|c| c["case_id"] == "author-grant")
        .expect("author-grant must exist");
    assert_eq!(author_grant["boundary_result"], "authorized");
    assert_eq!(author_grant["zome_reached"], true);
    assert_eq!(author_grant["capability_state"], "author-grant");

    assert!(fixture["depends_on"]["claim_ceiling"]
        .as_str()
        .is_some_and(|v| v == "ReferenceModelOnly"));
}
