// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Deterministic public client contract tests.
//!
//! Real-conductor compatibility must exercise this transport and an authorized
//! signer together. Connectivity through the official client does not validate
//! this crate's wire implementation.

use mycelix_leptos_client::{ClientError, SignedZomeCall, decode, encode};

#[test]
fn encode_decode_roundtrip() {
    let data = serde_json::json!({"id": "MIP-042", "title": "Solar garden"});
    let encoded = encode(&data).unwrap();
    let decoded: serde_json::Value = decode(&encoded).unwrap();
    assert_eq!(decoded, data);
}

#[test]
fn signed_call_rejects_empty_bytes() {
    let result = SignedZomeCall::new(Vec::new(), vec![7; 64]);
    assert!(matches!(result, Err(ClientError::InvalidSignature(_))));
}

#[test]
fn signed_call_rejects_wrong_signature_length() {
    let result = SignedZomeCall::new(vec![1, 2, 3], vec![7; 63]);
    assert!(matches!(result, Err(ClientError::InvalidSignature(_))));
}

#[test]
fn signed_call_rejects_zero_signature() {
    let result = SignedZomeCall::new(vec![1, 2, 3], vec![0; 64]);
    assert!(matches!(result, Err(ClientError::InvalidSignature(_))));
}

#[test]
fn signed_call_accepts_nonzero_ed25519_shape() {
    let call = SignedZomeCall::new(vec![1, 2, 3], vec![7; 64]).unwrap();
    assert_eq!(call.bytes(), &[1, 2, 3]);
    assert_eq!(call.signature(), &[7; 64]);
}

// ---------------------------------------------------------------------------
// Host-signer payload decoding — characterization of a real, currently-unfixed
// defect. See browser.rs `zome_call_to_js`.
//
// That function converts an already-MessagePack-encoded zome-call payload back
// into a JS value, because the official host hook takes an *unsigned*
// CallZomeRequest and performs the ExternIO encoding itself. It does the
// conversion by decoding into `serde_json::Value`:
//
//     let payload: serde_json::Value = rmp_serde::from_slice(&request.payload)?;
//
// `serde_json::Value` has no representation for a byte string. Holochain
// serializes every hash type — ActionHash, EntryHash, AgentPubKey, DnaHash —
// as MessagePack `bin`. So any zome call that takes a hash argument (which is
// most of them: get_record, create_link, get_links, …) fails to convert.
//
// This matters more than its one call site suggests: `zome_call_to_js` is
// reached only from `HostZomeCallSigner`, and that is precisely the path where
// the *host* holds the keys and signs on our behalf — i.e. the path a Kangaroo
// or Tauri native shell needs, and the path a Moss/Weave Tool needs. Both
// packaging routes depend on the one code path this breaks.
//
// It fails loudly (Err, not silent corruption), which is the right failure
// mode. The tests below pin the behaviour so it cannot change unnoticed.
//
// WHEN THIS IS FIXED, `hash_bearing_payload_cannot_decode_to_json_value` will
// start failing. That is the intended signal — replace it with the positive
// assertion given in its body.
// ---------------------------------------------------------------------------

#[derive(serde::Serialize)]
struct CallWithHash {
    /// Holochain hashes are msgpack `bin`, which `serde_bytes` models.
    #[serde(with = "serde_bytes")]
    action_hash: Vec<u8>,
    note: String,
}

/// A 39-byte ActionHash: 3-byte multihash prefix + 32 core bytes + 4 loc bytes.
fn sample_action_hash() -> Vec<u8> {
    let mut hash = vec![0x84, 0x21, 0x24];
    hash.extend_from_slice(&[0xAB; 32]);
    hash.extend_from_slice(&[0x01, 0x02, 0x03, 0x04]);
    hash
}

#[test]
fn hash_bearing_payload_cannot_decode_to_json_value() {
    let encoded = encode(&CallWithHash {
        action_hash: sample_action_hash(),
        note: "get_record".to_string(),
    })
    .expect("encoding a hash-bearing payload must succeed");

    let decoded: Result<serde_json::Value, _> = decode(&encoded);

    // When the defect is fixed, this becomes:
    //     let decoded = decoded.expect("hash-bearing payload must decode");
    //     assert!(decoded.get("action_hash").is_some());
    let error = decoded.expect_err(
        "EXPECTED FAILURE: if this now decodes, zome_call_to_js has been fixed \
         — invert this test (see the note above it)",
    );
    let message = format!("{error}");
    assert!(
        message.contains("byte array"),
        "expected the msgpack `bin` -> serde_json::Value type error, got: {message}"
    );
}

#[test]
fn payload_without_hash_decodes_normally() {
    // Positive control: proves the failure above is specific to byte strings
    // and not a general decode problem.
    #[derive(serde::Serialize)]
    struct CallWithoutHash {
        note: String,
    }

    let encoded = encode(&CallWithoutHash {
        note: "ping".to_string(),
    })
    .expect("encoding must succeed");
    let decoded: serde_json::Value = decode(&encoded).expect("no-hash payload must decode");
    assert_eq!(decoded["note"], "ping");
}
