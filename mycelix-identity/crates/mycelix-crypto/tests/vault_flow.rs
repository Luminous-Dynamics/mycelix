// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Integration test: the end-to-end pattern a cluster uses to store an
//! encrypted record *at rest* with hybrid PQC — the turnkey reference for
//! Phase 4 (encrypt the `mycelix-personal` vaults, which store plaintext PHI
//! today). See `PQC_ROADMAP_2026-07-07.md`.
//!
//! Unlike the per-module unit tests, this exercises the serialization boundary
//! a real cluster crosses: a structured record → JSON bytes → authenticated
//! hybrid ciphertext (the opaque bytes that replace a plaintext `data: String`
//! entry field) → verify sender → decrypt → deserialize.

#![cfg(feature = "hybrid-rc")]

use mycelix_crypto::hybrid_kem::HybridKemKeyPair;
use mycelix_crypto::hybrid_sig::HybridSigner;
use mycelix_crypto::sealed_box::{open_verified, seal_signed};
use serde::{Deserialize, Serialize};

/// A stand-in for `mycelix-personal`'s `HealthRecord` whose `data` field is
/// plaintext JSON on the DHT today.
#[derive(Serialize, Deserialize, PartialEq, Debug, Clone)]
struct HealthRecord {
    record_type: String,
    data: serde_json::Value,
    source: String,
}

#[test]
fn health_record_encrypted_at_rest_round_trip() {
    // Recipient = the patient's vault keypair; sender = the writing device/clinician.
    let patient = HybridKemKeyPair::generate();
    let clinician = HybridSigner::generate();

    let record = HealthRecord {
        record_type: "lab_result".into(),
        data: serde_json::json!({ "hba1c": 5.4, "unit": "%" }),
        source: "did:mycelix:clinic-42".into(),
    };

    // WRITE PATH: serialize → authenticated hybrid-encrypt.
    let plaintext = serde_json::to_vec(&record).expect("serialize record");
    let sealed = seal_signed(&patient.public_keys(), &clinician, &plaintext).expect("seal");

    // What the zome stores is opaque bytes (ciphertext + KEM ct + nonce + sig);
    // no plaintext PHI ever reaches the DHT.
    assert_ne!(
        sealed.ciphertext.ciphertext, plaintext,
        "stored bytes must be ciphertext, not plaintext"
    );

    // READ PATH: verify sender → decrypt → deserialize.
    let recovered_bytes =
        open_verified(&patient, &clinician.verifying_keys(), &sealed).expect("open");
    let recovered: HealthRecord =
        serde_json::from_slice(&recovered_bytes).expect("deserialize record");
    assert_eq!(recovered, record);
}

#[test]
fn plaintext_phi_never_appears_in_stored_ciphertext() {
    let patient = HybridKemKeyPair::generate();
    let clinician = HybridSigner::generate();

    let secret = b"diagnosis: confidential condition";
    let sealed = seal_signed(&patient.public_keys(), &clinician, secret).expect("seal");

    // The sensitive bytes must not appear anywhere in what gets stored.
    assert!(
        !contains_subslice(&sealed.ciphertext.ciphertext, secret),
        "PHI leaked into stored ciphertext"
    );
}

#[test]
fn a_different_patient_key_cannot_read_the_record() {
    let patient = HybridKemKeyPair::generate();
    let attacker = HybridKemKeyPair::generate();
    let clinician = HybridSigner::generate();

    let sealed = seal_signed(&patient.public_keys(), &clinician, b"private").expect("seal");
    // Even a valid, correctly-signed record cannot be read with the wrong vault key.
    assert!(open_verified(&attacker, &clinician.verifying_keys(), &sealed).is_err());
}

fn contains_subslice(haystack: &[u8], needle: &[u8]) -> bool {
    if needle.is_empty() || haystack.len() < needle.len() {
        return false;
    }
    haystack.windows(needle.len()).any(|w| w == needle)
}
