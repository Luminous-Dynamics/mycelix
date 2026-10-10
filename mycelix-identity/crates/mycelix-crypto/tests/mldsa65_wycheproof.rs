//! Pinned published and proposed-upstream ML-DSA-65 verification corpora.
//!
//! This executes the exact published C2SP snapshot plus the separately pinned,
//! signed-but-not-yet-merged C2SP PR #278 candidate. The latter regenerates
//! edge-case vectors under the final FIPS 204 key-expansion derivation.
//! This remains RustCrypto regression evidence, not cross-implementation qualification.

use mycelix_crypto::mldsa65_verify::{
    verify_with_empty_context, ML_DSA_65_PUBLIC_KEY_BYTES, ML_DSA_65_SIGNATURE_BYTES,
};
use serde_json::Value;
use sha2::{Digest, Sha256};

const PUBLISHED_CORPUS: &str = include_str!("data/mldsa_65_verify_test.json");
const PUBLISHED_SHA256: &str =
    "49ac366d76115eab56b7116f10d06e288e6f23fe6cfb90b26bfb2d731a8d1e02";

const PR278_CORPUS: &str = include_str!("data/mldsa_65_verify_pr278_test.json");
const PR278_SHA256: &str =
    "1ca235f61928421a173a171780beb00264ae9155dc33c52a11e1766cc7979034";

fn decode_hex(input: &str) -> Vec<u8> {
    assert_eq!(input.len() % 2, 0, "hex input has an odd number of digits");
    let bytes = input.as_bytes();
    let mut output = Vec::with_capacity(bytes.len() / 2);
    for (index, pair) in bytes.chunks_exact(2).enumerate() {
        let high = hex_nibble(pair[0])
            .unwrap_or_else(|| panic!("invalid hex digit at byte {}", index * 2));
        let low = hex_nibble(pair[1])
            .unwrap_or_else(|| panic!("invalid hex digit at byte {}", index * 2 + 1));
        output.push((high << 4) | low);
    }
    output
}

fn hex_nibble(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        b'A'..=b'F' => Some(byte - b'A' + 10),
        _ => None,
    }
}

fn exercise_corpus(source: &str, corpus_text: &str, expected_sha256: &str) {
    let actual_sha256 = format!("{:x}", Sha256::digest(corpus_text.as_bytes()));
    assert_eq!(
        actual_sha256, expected_sha256,
        "{source}: pinned fixture SHA-256 mismatch"
    );

    let corpus: Value =
        serde_json::from_str(corpus_text).expect("pinned Wycheproof corpus must be valid JSON");
    assert_eq!(corpus["algorithm"].as_str(), Some("ML-DSA-65"));
    assert_eq!(corpus["numberOfTests"].as_u64(), Some(210));

    let groups = corpus["testGroups"]
        .as_array()
        .expect("testGroups must be an array");
    let mut total = 0usize;
    let mut valid = 0usize;
    let mut invalid = 0usize;
    let mut tc19_seen = false;
    let mut tc61_seen = false;

    for (group_index, group) in groups.iter().enumerate() {
        assert_eq!(
            group["type"].as_str(),
            Some("MlDsaVerify"),
            "{source}: unexpected test group type at index {group_index}"
        );
        let public_key = decode_hex(
            group["publicKey"]
                .as_str()
                .expect("test group must contain a raw publicKey"),
        );
        assert_eq!(
            public_key.len(),
            ML_DSA_65_PUBLIC_KEY_BYTES,
            "{source}: wrong public-key size in group {group_index}"
        );
        let tests = group["tests"]
            .as_array()
            .expect("each test group must contain an array of tests");

        for case in tests {
            let tc_id = case["tcId"]
                .as_u64()
                .expect("each test case must have a numeric tcId");
            let message = decode_hex(
                case["msg"]
                    .as_str()
                    .expect("each test case must contain a hex message"),
            );
            let signature = decode_hex(
                case["sig"]
                    .as_str()
                    .expect("each test case must contain a hex signature"),
            );
            assert_eq!(
                signature.len(),
                ML_DSA_65_SIGNATURE_BYTES,
                "{source}: wrong signature size for tcId {tc_id}"
            );

            let result = verify_with_empty_context(&public_key, &message, &signature);
            let expected = case["result"]
                .as_str()
                .expect("each test case must declare its expected result");
            match expected {
                "valid" => {
                    assert!(
                        result.is_ok(),
                        "{source}: tcId {tc_id} expected valid but was rejected: {result:?}"
                    );
                    valid += 1;
                }
                "invalid" => {
                    assert!(
                        result.is_err(),
                        "{source}: tcId {tc_id} expected invalid but was accepted"
                    );
                    invalid += 1;
                }
                "acceptable" => {
                    panic!("{source}: tcId {tc_id} is flagged acceptable; no explicit policy is defined");
                }
                other => panic!("{source}: tcId {tc_id} has unknown expected result {other:?}"),
            }

            match tc_id {
                19 => {
                    assert_eq!(expected, "invalid", "{source}: tcId 19 must reject repeated hint indices");
                    let flags = case["flags"].as_array().expect("tcId 19 must declare flags");
                    assert!(flags.iter().any(|flag| flag.as_str() == Some("InvalidHintsEncoding")));
                    tc19_seen = true;
                }
                61 => {
                    assert_eq!(expected, "valid", "{source}: tcId 61 must accept the valid near-boundary signature");
                    let flags = case["flags"].as_array().expect("tcId 61 must declare flags");
                    assert!(flags.iter().any(|flag| flag.as_str() == Some("BoundaryCondition")));
                    tc61_seen = true;
                }
                _ => {}
            }
            total += 1;
        }
    }

    assert_eq!(total, 210, "{source}: not every corpus case was exercised");
    assert_eq!(valid, 79, "{source}: unexpected number of expected-valid cases");
    assert_eq!(invalid, 131, "{source}: unexpected number of expected-invalid cases");
    assert!(tc19_seen, "{source}: required repeated-hint sentinel tcId 19 was not exercised");
    assert!(tc61_seen, "{source}: required valid-boundary sentinel tcId 61 was not exercised");
}

#[test]
fn published_c2sp_snapshot_matches_all_expected_verdicts() {
    exercise_corpus("published C2SP snapshot at 12fd3aaf", PUBLISHED_CORPUS, PUBLISHED_SHA256);
}

#[test]
fn regenerated_c2sp_pr278_candidate_matches_all_expected_verdicts() {
    exercise_corpus("C2SP PR #278 candidate at 8f654b7f", PR278_CORPUS, PR278_SHA256);
}
