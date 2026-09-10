// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_identity_provenance::{
    KeyAssociationPurpose, KeyDidBindingArtifact, XENIA_ARTIFACT_DOMAIN,
    XENIA_ARTIFACT_SCHEMA,
};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct Fixture {
    fixture_schema: String,
    did: String,
    xenia_key_fingerprint_hex: String,
    xenia_signature_suite: String,
    purpose: String,
    scope: String,
    xenia_artifact_domain: String,
    xenia_artifact_schema: String,
    subject_ref: String,
    canonical_bytes_len: usize,
    canonical_bytes_hex: String,
}

fn decode_hex(input: &str) -> Vec<u8> {
    assert_eq!(input.len() % 2, 0, "fixture hex must have an even length");
    input
        .as_bytes()
        .chunks_exact(2)
        .map(|pair| {
            let high = (pair[0] as char).to_digit(16).expect("valid fixture hex");
            let low = (pair[1] as char).to_digit(16).expect("valid fixture hex");
            ((high << 4) | low) as u8
        })
        .collect()
}

#[test]
fn key_did_binding_v1_matches_pinned_xenia_conformance_vector() {
    let fixture: Fixture = serde_json::from_str(include_str!(
        "../interop-fixtures/key-did-binding-v1.json"
    ))
    .expect("valid conformance fixture");

    assert_eq!(
        fixture.fixture_schema,
        "mycelix-xenia-key-binding-conformance-v1"
    );

    let fingerprint = decode_hex(&fixture.xenia_key_fingerprint_hex);
    let fingerprint: [u8; 32] = fingerprint
        .try_into()
        .expect("fixture fingerprint must be exactly 32 bytes");

    let purpose = match fixture.purpose.as_str() {
        "EvidenceAttestor" => KeyAssociationPurpose::EvidenceAttestor,
        other => panic!("unsupported fixture purpose: {other}"),
    };

    let artifact = KeyDidBindingArtifact::new(
        fixture.did.clone(),
        fingerprint,
        fixture.xenia_signature_suite.clone(),
        purpose,
        fixture.scope.clone(),
    )
    .expect("fixture binding must satisfy Mycelix v1 invariants");

    let canonical = artifact
        .canonical_bytes()
        .expect("fixture binding must canonicalize");
    assert_eq!(canonical.len(), fixture.canonical_bytes_len);
    assert_eq!(canonical, decode_hex(&fixture.canonical_bytes_hex));

    assert_eq!(fixture.xenia_artifact_domain, XENIA_ARTIFACT_DOMAIN);
    assert_eq!(fixture.xenia_artifact_schema, XENIA_ARTIFACT_SCHEMA);
    assert_eq!(fixture.subject_ref, artifact.xenia_subject_ref());
}
