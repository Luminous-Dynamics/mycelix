// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_constitutional_root::{
    AuthorizedPolicyScope, ConstitutionalRoot, PROTOCOL_VERSION, Rulebook, qualify_root,
};
use std::fmt::Write;

fn digest(byte: u8) -> [u8; 32] {
    [byte; 32]
}

fn hex32(value: &str) -> [u8; 32] {
    assert_eq!(value.len(), 64);
    let mut output = [0_u8; 32];
    for (index, slot) in output.iter_mut().enumerate() {
        let raw = value.as_bytes();
        *slot = (nibble(raw[index * 2]) << 4) | nibble(raw[index * 2 + 1]);
    }
    output
}

fn nibble(value: u8) -> u8 {
    match value {
        b'0'..=b'9' => value - b'0',
        b'a'..=b'f' => value - b'a' + 10,
        b'A'..=b'F' => value - b'A' + 10,
        _ => panic!("invalid hex fixture"),
    }
}

fn hex(value: &[u8; 32]) -> String {
    let mut output = String::with_capacity(64);
    for byte in value {
        write!(&mut output, "{byte:02x}").expect("write to String");
    }
    output
}

fn provider_rulebook() -> Rulebook {
    Rulebook {
        id: "rulebook:city-clerk:v1".into(),
        version: "1.0.0".into(),
        digest: digest(0x22),
    }
}

fn scopes() -> Vec<AuthorizedPolicyScope> {
    vec![
        AuthorizedPolicyScope {
            policy_identity_profile: "mycelix-review-policy-v1-blake3-framed-semantic".into(),
            policy_registry_namespace: "registry:review-policy:example-city".into(),
            provider_authority_institution_id: "institution:city-clerk".into(),
            provider_authority_jurisdiction_id: Some("jurisdiction:example-city".into()),
            provider_authority_rulebook: provider_rulebook(),
            required_provider_capability: "administration.review-policy.currentness.attest".into(),
        },
        AuthorizedPolicyScope {
            policy_identity_profile:
                "mycelix-procedure-policy-currentness-provider-v1-blake3-framed-semantic".into(),
            policy_registry_namespace: "registry:procedure-policy:example-city".into(),
            provider_authority_institution_id: "institution:city-clerk".into(),
            provider_authority_jurisdiction_id: Some("jurisdiction:example-city".into()),
            provider_authority_rulebook: provider_rulebook(),
            required_provider_capability: "administration.policy.currentness.attest".into(),
        },
    ]
}

fn root() -> ConstitutionalRoot {
    ConstitutionalRoot {
        protocol_version: PROTOCOL_VERSION.into(),
        institution_id: "institution:city-of-example".into(),
        jurisdiction_id: Some("jurisdiction:example-city".into()),
        constitutional_rulebook: Rulebook {
            id: "rulebook:city-charter:v1".into(),
            version: "1.0.0".into(),
            digest: digest(0x11),
        },
        generation: 0,
        predecessor_root_digest: None,
        bootstrap_mode: "pinned-constitutional-commitment".into(),
        bootstrap_profile: "deployment-pinned-root-digest-v1".into(),
        authoritative_root_source_ref: "registry:constitutional-root:example-city".into(),
        root_coverage_profile: "mycelix-constitutional-root-covered-head-v1".into(),
        root_source_verification_profile: "mycelix-constitutional-root-source-verification-v1"
            .into(),
        root_source_anchor_digest: digest(0x33),
        authorized_policy_scopes: scopes(),
        valid_from_ms: 1_800_000_000_000,
        expires_at_ms: None,
        rotation_mode: "predecessor-authorized".into(),
        rotation_profile: Some("constitutional-root-rotation-v1".into()),
        rotation_authority_anchor_digest: Some(digest(0x44)),
    }
}

fn transition_predecessor() -> ConstitutionalRoot {
    let mut root = root();
    root.expires_at_ms = Some(1_800_000_200_000);
    root.rotation_authority_anchor_digest = Some(hex32(
        "32d29fa9f5a28f5ef5ca3f298902add00f1ddd79007b729d0c12eae77358367f",
    ));
    root
}

fn transition_successor() -> ConstitutionalRoot {
    let mut root = root();
    root.constitutional_rulebook = Rulebook {
        id: "rulebook:city-charter:v2".into(),
        version: "2.0.0".into(),
        digest: digest(0xaa),
    };
    root.generation = 1;
    root.predecessor_root_digest = Some(hex32(
        "b7a0c7cb28f182d06367d4bf1d3cc4f7d82094701d955eeaa7ec53153d2080a6",
    ));
    root.root_source_anchor_digest = digest(0x55);
    root.valid_from_ms = 1_800_000_100_000;
    root.rotation_authority_anchor_digest = Some(hex32(
        "f6833e11f4317ff680813e6e1d9e1189b4cbb1abb44a67f4b7fec3897427435f",
    ));
    root
}

fn print_qualified(label: &str, candidate: ConstitutionalRoot) {
    let qualified = qualify_root(candidate).expect("conformance fixture must qualify");
    println!("{label}.root={}", hex(&qualified.root_identity().digest));
    println!(
        "{label}.source={}",
        hex(&qualified.source_descriptor_identity().digest)
    );
    println!(
        "{label}.rotation={}",
        hex(&qualified
            .rotation_authority_identity()
            .expect("fixture is rotatable")
            .digest)
    );
}

fn main() {
    print_qualified("golden", root());
    print_qualified("predecessor", transition_predecessor());
    print_qualified("successor", transition_successor());
}
