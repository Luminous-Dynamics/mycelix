// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use curve25519_dalek::constants::EIGHT_TORSION;
use mycelix_constitutional_root::{
    AuthorizedPolicyScope, ConstitutionalRoot, QualifiedConstitutionalRootA, Rulebook, qualify_root,
};
use mycelix_constitutional_transition_verifier::{
    AuthorizationMaterial, AuthorizationProof, MATERIAL_PROFILE, MATERIAL_SUITE,
    NORMAL_ROTATION_PROFILE, PROOF_SUITE, TRANSITION_PROFILE, TransitionCandidate, TransitionError,
    authorization_material_commitment, canonical_candidate_bytes, verify_transition,
};
use sha2::{Digest, Sha256};

fn hex_bytes(value: &str) -> Vec<u8> {
    assert_eq!(value.len() % 2, 0);
    value
        .as_bytes()
        .as_chunks::<2>()
        .0
        .iter()
        .map(|pair| (nibble(pair[0]) << 4) | nibble(pair[1]))
        .collect()
}

fn hex32(value: &str) -> [u8; 32] {
    hex_bytes(value).try_into().expect("32-byte fixture")
}

fn hex64(value: &str) -> [u8; 64] {
    hex_bytes(value).try_into().expect("64-byte fixture")
}

fn nibble(value: u8) -> u8 {
    match value {
        b'0'..=b'9' => value - b'0',
        b'a'..=b'f' => value - b'a' + 10,
        b'A'..=b'F' => value - b'A' + 10,
        _ => panic!("invalid hex fixture"),
    }
}

fn sha256(bytes: &[u8]) -> [u8; 32] {
    let digest = Sha256::digest(bytes);
    let mut out = [0u8; 32];
    out.copy_from_slice(&digest);
    out
}

fn rulebook(id: &str, version: &str, digest: [u8; 32]) -> Rulebook {
    Rulebook {
        id: id.into(),
        version: version.into(),
        digest,
    }
}

fn scopes() -> Vec<AuthorizedPolicyScope> {
    vec![
        AuthorizedPolicyScope {
            policy_identity_profile: "mycelix-review-policy-v1-blake3-framed-semantic".into(),
            policy_registry_namespace: "registry:review-policy:example-city".into(),
            provider_authority_institution_id: "institution:city-clerk".into(),
            provider_authority_jurisdiction_id: Some("jurisdiction:example-city".into()),
            provider_authority_rulebook: rulebook("rulebook:city-clerk:v1", "1.0.0", [0x22; 32]),
            required_provider_capability: "administration.review-policy.currentness.attest".into(),
        },
        AuthorizedPolicyScope {
            policy_identity_profile:
                "mycelix-procedure-policy-currentness-provider-v1-blake3-framed-semantic".into(),
            policy_registry_namespace: "registry:procedure-policy:example-city".into(),
            provider_authority_institution_id: "institution:city-clerk".into(),
            provider_authority_jurisdiction_id: Some("jurisdiction:example-city".into()),
            provider_authority_rulebook: rulebook("rulebook:city-clerk:v1", "1.0.0", [0x22; 32]),
            required_provider_capability: "administration.policy.currentness.attest".into(),
        },
    ]
}

fn predecessor_root() -> ConstitutionalRoot {
    ConstitutionalRoot {
        protocol_version: "mycelix-constitutional-trust-root-v0.1".into(),
        institution_id: "institution:city-of-example".into(),
        jurisdiction_id: Some("jurisdiction:example-city".into()),
        constitutional_rulebook: rulebook("rulebook:city-charter:v1", "1.0.0", [0x11; 32]),
        generation: 0,
        predecessor_root_digest: None,
        bootstrap_mode: "pinned-constitutional-commitment".into(),
        bootstrap_profile: "deployment-pinned-root-digest-v1".into(),
        authoritative_root_source_ref: "registry:constitutional-root:example-city".into(),
        root_coverage_profile: "mycelix-constitutional-root-covered-head-v1".into(),
        root_source_verification_profile: "mycelix-constitutional-root-source-verification-v1"
            .into(),
        root_source_anchor_digest: [0x33; 32],
        authorized_policy_scopes: scopes(),
        valid_from_ms: 1_800_000_000_000,
        expires_at_ms: Some(1_800_000_200_000),
        rotation_mode: "predecessor-authorized".into(),
        rotation_profile: Some(NORMAL_ROTATION_PROFILE.into()),
        rotation_authority_anchor_digest: Some(hex32(
            "32d29fa9f5a28f5ef5ca3f298902add00f1ddd79007b729d0c12eae77358367f",
        )),
    }
}

fn successor_root() -> ConstitutionalRoot {
    ConstitutionalRoot {
        protocol_version: "mycelix-constitutional-trust-root-v0.1".into(),
        institution_id: "institution:city-of-example".into(),
        jurisdiction_id: Some("jurisdiction:example-city".into()),
        constitutional_rulebook: rulebook("rulebook:city-charter:v2", "2.0.0", [0xaa; 32]),
        generation: 1,
        predecessor_root_digest: Some(hex32(
            "b7a0c7cb28f182d06367d4bf1d3cc4f7d82094701d955eeaa7ec53153d2080a6",
        )),
        bootstrap_mode: "pinned-constitutional-commitment".into(),
        bootstrap_profile: "deployment-pinned-root-digest-v1".into(),
        authoritative_root_source_ref: "registry:constitutional-root:example-city".into(),
        root_coverage_profile: "mycelix-constitutional-root-covered-head-v1".into(),
        root_source_verification_profile: "mycelix-constitutional-root-source-verification-v1"
            .into(),
        root_source_anchor_digest: [0x55; 32],
        authorized_policy_scopes: scopes(),
        valid_from_ms: 1_800_000_100_000,
        expires_at_ms: None,
        rotation_mode: "predecessor-authorized".into(),
        rotation_profile: Some(NORMAL_ROTATION_PROFILE.into()),
        rotation_authority_anchor_digest: Some(hex32(
            "f6833e11f4317ff680813e6e1d9e1189b4cbb1abb44a67f4b7fec3897427435f",
        )),
    }
}

fn qualified_pair() -> (QualifiedConstitutionalRootA, QualifiedConstitutionalRootA) {
    (
        qualify_root(predecessor_root()).expect("qualified predecessor"),
        qualify_root(successor_root()).expect("qualified successor"),
    )
}

fn candidate() -> TransitionCandidate {
    TransitionCandidate {
        profile: TRANSITION_PROFILE.into(),
        predecessor_root_identity_profile:
            "mycelix-constitutional-trust-root-v1-sha256-framed-semantic".into(),
        predecessor_root_digest: hex32(
            "b7a0c7cb28f182d06367d4bf1d3cc4f7d82094701d955eeaa7ec53153d2080a6",
        ),
        predecessor_generation: 0,
        predecessor_source_descriptor_profile:
            "mycelix-constitutional-root-source-descriptor-v1-sha256-framed-semantic".into(),
        predecessor_source_descriptor_digest: hex32(
            "f97a96e20ce6dd0c86c67e1590252abc678dfd6401128ab44ced4a66ec088126",
        ),
        predecessor_rotation_authority_profile:
            "mycelix-constitutional-root-rotation-authority-v1-sha256-framed-semantic".into(),
        predecessor_rotation_authority_digest: hex32(
            "db3bf05b04063ea2e9626e9ea63def4920f205cb36c43d64a046fd04e87b8304",
        ),
        successor_root_identity_profile:
            "mycelix-constitutional-trust-root-v1-sha256-framed-semantic".into(),
        successor_root_digest: hex32(
            "c970bfc0957efc00946d08a957d7617b85815f47d98fcc98aa43f6f6a6ced963",
        ),
        successor_generation: 1,
        successor_source_descriptor_profile:
            "mycelix-constitutional-root-source-descriptor-v1-sha256-framed-semantic".into(),
        successor_source_descriptor_digest: hex32(
            "3f867aab08093b08f8f4088a540e8cffa85dc25ab7c7cfb34ac65c8bc071a3fe",
        ),
        successor_rotation_authority_profile: Some(
            "mycelix-constitutional-root-rotation-authority-v1-sha256-framed-semantic".into(),
        ),
        successor_rotation_authority_digest: Some(hex32(
            "ee35844f067306e66ec8fcb40442b3564e9f7f5bd824a3e2a71265ea318aebea",
        )),
        authorized_at_ms: 1_800_000_050_000,
        effective_at_ms: 1_800_000_100_000,
        replay_nonce: vec![0x9a; 32],
    }
}

fn material() -> AuthorizationMaterial {
    AuthorizationMaterial {
        profile: MATERIAL_PROFILE.into(),
        suite: MATERIAL_SUITE.into(),
        public_key_spki_der: hex_bytes(
            "302a300506032b657003210059e958de703e469ebab72fc99f64d7f715cb1fd723f571575e74833cbc4bedee",
        ),
    }
}

fn proof() -> AuthorizationProof {
    AuthorizationProof {
        suite: PROOF_SUITE.into(),
        signature: hex64(
            "1bd78cbe9f303f8b258c928bef0c2173dc581c22c699cf1d2c43c7565bd3a4f6576def1a1c06aa8e107556ac964ef1862d0ea2fb48b2d3c88f27f5019aab7601",
        ),
    }
}

#[test]
fn exact_qualified_839_vector_verifies_in_rust() {
    let (predecessor, successor) = qualified_pair();
    assert_eq!(
        predecessor.root_identity().digest,
        candidate().predecessor_root_digest
    );
    assert_eq!(
        successor.root_identity().digest,
        candidate().successor_root_digest
    );

    let material_commitment = authorization_material_commitment(&material()).unwrap();
    assert_eq!(
        material_commitment,
        hex32("32d29fa9f5a28f5ef5ca3f298902add00f1ddd79007b729d0c12eae77358367f")
    );

    let bytes = canonical_candidate_bytes(&candidate()).unwrap();
    assert_eq!(
        sha256(&bytes),
        hex32("d71e66425effe3668790f552409aeba2005cca3c50ccb23cbd1d5a32dd060a24")
    );

    let verified = verify_transition(
        &predecessor,
        &successor,
        &candidate(),
        &material(),
        &proof(),
    )
    .unwrap();
    assert_eq!(
        verified.transition_identity().digest,
        hex32("d71e66425effe3668790f552409aeba2005cca3c50ccb23cbd1d5a32dd060a24")
    );
    assert!(!verified.grants_currentness());
    assert!(!verified.grants_effect_authority());
}

#[test]
fn signature_and_candidate_mutation_deny() {
    let (predecessor, successor) = qualified_pair();
    let mut bad_proof = proof();
    bad_proof.signature[32] ^= 1;
    assert_eq!(
        verify_transition(
            &predecessor,
            &successor,
            &candidate(),
            &material(),
            &bad_proof,
        ),
        Err(TransitionError::InvalidSignature)
    );

    let mut bad_candidate = candidate();
    bad_candidate.successor_root_digest[0] ^= 1;
    assert_eq!(
        verify_transition(
            &predecessor,
            &successor,
            &bad_candidate,
            &material(),
            &proof(),
        ),
        Err(TransitionError::CandidateRebindingMismatch)
    );
}

#[test]
fn successor_key_cannot_authorize_its_own_admission() {
    let (predecessor, successor) = qualified_pair();
    let successor_material = AuthorizationMaterial {
        profile: MATERIAL_PROFILE.into(),
        suite: MATERIAL_SUITE.into(),
        public_key_spki_der: hex_bytes(
            "302a300506032b6570032100b3aeb3ef6cd8f7957cc868ecf66e2ba21d3dfd2417a43354749d684633380333",
        ),
    };
    assert_eq!(
        verify_transition(
            &predecessor,
            &successor,
            &candidate(),
            &successor_material,
            &proof(),
        ),
        Err(TransitionError::MaterialCommitmentMismatch)
    );
}

#[test]
fn noncanonical_nonce_and_signature_point_deny() {
    let (predecessor, successor) = qualified_pair();
    let mut zero_nonce = candidate();
    zero_nonce.replay_nonce = vec![0; 32];
    assert_eq!(
        verify_transition(&predecessor, &successor, &zero_nonce, &material(), &proof(),),
        Err(TransitionError::InvalidReplayNonce)
    );

    let mut torsion_proof = proof();
    torsion_proof.signature[..32].copy_from_slice(EIGHT_TORSION[1].compress().as_bytes());
    assert_eq!(
        verify_transition(
            &predecessor,
            &successor,
            &candidate(),
            &material(),
            &torsion_proof,
        ),
        Err(TransitionError::NonPrimeOrderSignaturePoint)
    );
}
