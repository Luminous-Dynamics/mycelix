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
