//! Complete, pinned C2SP Wycheproof ML-DSA-65 verification corpus.
//!
//! This is corpus regression evidence for the RustCrypto candidate only. It is
//! not independent cross-implementation qualification; the separate Libcrux
//! oracle and mutation gates remain mandatory in ML_DSA_65_QUALIFICATION_PLAN.md.

use mycelix_crypto::mldsa65_verify::{
    verify_with_empty_context, ML_DSA_65_PUBLIC_KEY_BYTES, ML_DSA_65_SIGNATURE_BYTES,
};
use serde_json::Value;
use sha2::{Digest, Sha256};

const CORPUS: &str = include_str!("data/mldsa_65_verify_test.json");
const EXPECTED_SHA256: &str =
    "49ac366d76115eab56b7116f10d06e288e6f23fe6cfb90b26bfb2d731a8d1e02";

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

#[test]
fn fixture_sha256_matches_the_pinned_corpus() {
    let actual = format!("{:x}", Sha256::digest(CORPUS.as_bytes()));
    assert_eq!(actual, EXPECTED_SHA256, "pinned Wycheproof fixture changed");
}

#[test]
fn complete_wycheproof_mldsa65_verification_corpus() {
    let corpus: Value =
        serde_json::from_str(CORPUS).expect("pinned Wycheproof corpus must be valid JSON");
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
            "unexpected test group type at index {group_index}"
        );
        let public_key = decode_hex(
            group["publicKey"]
                .as_str()
                .expect("test group must contain a raw publicKey"),
        );
        assert_eq!(
            public_key.len(),
            ML_DSA_65_PUBLIC_KEY_BYTES,
            "wrong public-key size in test group {group_index}"
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
                "wrong signature size for tcId {tc_id}"
            );

            let result = verify_with_empty_context(&public_key, &message, &signature);
            let expected = case["result"]
                .as_str()
                .expect("each test case must declare its expected result");
            match expected {
                "valid" => {
                    assert!(
                        result.is_ok(),
                        "tcId {tc_id} was expected valid but was rejected: {result:?}"
                    );
                    valid += 1;
                }
                "invalid" => {
                    assert!(
                        result.is_err(),
                        "tcId {tc_id} was expected invalid but was accepted"
                    );
                    invalid += 1;
                }
                "acceptable" => {
                    panic!(
                        "tcId {tc_id} is flagged acceptable; add an explicit reviewed policy                          instead of silently classifying it"
                    );
                }
                other => panic!("tcId {tc_id} has unknown expected result {other:?}"),
            }

            match tc_id {
                19 => {
                    assert_eq!(expected, "invalid", "tcId 19 must reject repeated hint indices");
                    let flags = case["flags"]
                        .as_array()
                        .expect("tcId 19 must declare flags");
                    assert!(flags.iter().any(|flag| flag.as_str() == Some("InvalidHintsEncoding")));
                    tc19_seen = true;
                }
                61 => {
                    assert_eq!(expected, "valid", "tcId 61 must accept the valid near-boundary signature");
                    let flags = case["flags"]
                        .as_array()
                        .expect("tcId 61 must declare flags");
                    assert!(flags.iter().any(|flag| flag.as_str() == Some("BoundaryCondition")));
                    tc61_seen = true;
                }
                _ => {}
            }
            total += 1;
        }
    }

    assert_eq!(total, 210, "not every pinned corpus case was exercised");
    assert_eq!(valid, 79, "unexpected count of expected-valid cases");
    assert_eq!(invalid, 131, "unexpected count of expected-invalid cases");
    assert!(tc19_seen, "required repeated-hint sentinel tcId 19 was not exercised");
    assert!(tc61_seen, "required valid-boundary sentinel tcId 61 was not exercised");
}
