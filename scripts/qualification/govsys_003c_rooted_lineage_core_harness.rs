// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Qualification-only executable bridge into the exact CORE-LINEAGE Stage 1 crate.
// This is not a production constitutional verifier.

use mycelix_core_lineage::{
    LineageError, ProfiledDigest32, RootAnchorFacts, TransitionFacts, project_rooted_lineage,
};

const DOMAIN_PROFILE: &str = "mycelix-constitutional-root-lineage-domain-v1-sha256-framed-semantic";
const ROOT_PROFILE: &str = "mycelix-constitutional-trust-root-v1-sha256-framed-semantic";
const SOURCE_PROFILE: &str =
    "mycelix-constitutional-root-source-descriptor-v1-sha256-framed-semantic";
const TRANSITION_PROFILE: &str = "mycelix-constitutional-root-transition-v1-sha256-framed-semantic";

fn hex32(value: &str) -> [u8; 32] {
    assert_eq!(value.len(), 64);
    let mut out = [0_u8; 32];
    let bytes = value.as_bytes();
    for index in 0..32 {
        let high = nibble(bytes[index * 2]);
        let low = nibble(bytes[index * 2 + 1]);
        out[index] = (high << 4) | low;
    }
    out
}

fn nibble(value: u8) -> u8 {
    match value {
        b'0'..=b'9' => value - b'0',
        b'a'..=b'f' => value - b'a' + 10,
        b'A'..=b'F' => value - b'A' + 10,
        _ => panic!("non-hex qualification fixture"),
    }
}

fn id(profile: &str, digest: &str) -> ProfiledDigest32 {
    ProfiledDigest32::try_new(profile, hex32(digest)).expect("valid qualification identity")
}

fn edge_domain() -> ProfiledDigest32 {
    id(
        DOMAIN_PROFILE,
        "6067a895dfec8c8a6f5650e5f2031cf981c116f5b22fc1e5f316893f6dcb7f7a",
    )
}

fn main() {
    let domain = edge_domain();
    let predecessor = id(
        ROOT_PROFILE,
        "b7a0c7cb28f182d06367d4bf1d3cc4f7d82094701d955eeaa7ec53153d2080a6",
    );
    let predecessor_source = id(
        SOURCE_PROFILE,
        "f97a96e20ce6dd0c86c67e1590252abc678dfd6401128ab44ced4a66ec088126",
    );
    let successor = id(
        ROOT_PROFILE,
        "c970bfc0957efc00946d08a957d7617b85815f47d98fcc98aa43f6f6a6ced963",
    );
    let successor_source = id(
        SOURCE_PROFILE,
        "3f867aab08093b08f8f4088a540e8cffa85dc25ab7c7cfb34ac65c8bc071a3fe",
    );
    let transition_identity = id(
        TRANSITION_PROFILE,
        "d71e66425effe3668790f552409aeba2005cca3c50ccb23cbd1d5a32dd060a24",
    );

    let root = RootAnchorFacts::new(
        domain.clone(),
        0,
        predecessor.clone(),
        1_800_000_000_000,
        predecessor_source.clone(),
    );
    let edge = TransitionFacts::new(
        domain,
        0,
        predecessor,
        predecessor_source,
        1,
        successor.clone(),
        successor_source.clone(),
        transition_identity,
        1_800_000_100_000,
    );

    let projected = project_rooted_lineage(root.clone(), core::slice::from_ref(&edge))
        .expect("qualified one-edge lineage");
    assert_eq!(projected.endpoint_generation(), 1);
    assert_eq!(projected.endpoint_node_identity(), &successor);
    assert_eq!(
        projected.endpoint_source_descriptor_identity(),
        &successor_source
    );
    assert_eq!(
        projected.stable_commitment(),
        &hex32("e039378be03e597f6d3c117890e8395cb112efbe6f19e2c7e0a2ccdf37eb1b74")
    );
    assert_eq!(
        projected.stable_commitment_profile(),
        "mycelix-core-lineage-v1-sha256-framed-semantic"
    );
    assert!(!projected.grants_currentness());
    assert!(!projected.grants_effect_authority());

    let duplicate = project_rooted_lineage(root.clone(), &[edge.clone(), edge.clone()])
        .expect("exact semantic duplicate must normalize");
    assert_eq!(duplicate.stable_commitment(), projected.stable_commitment());

    let parallel = TransitionFacts::new(
        edge_domain(),
        0,
        id(
            ROOT_PROFILE,
            "b7a0c7cb28f182d06367d4bf1d3cc4f7d82094701d955eeaa7ec53153d2080a6",
        ),
        id(
            SOURCE_PROFILE,
            "f97a96e20ce6dd0c86c67e1590252abc678dfd6401128ab44ced4a66ec088126",
        ),
        1,
        successor,
        successor_source,
        id(
            TRANSITION_PROFILE,
            "7777777777777777777777777777777777777777777777777777777777777777",
        ),
        1_800_000_100_000,
    );
    let conflict = project_rooted_lineage(root, &[edge, parallel]);
    assert_eq!(conflict, Err(LineageError::ParallelTransitionConflict));

    println!(
        "GOVSYS-003C-A CORE-LINEAGE bridge PASS: e039378be03e597f6d3c117890e8395cb112efbe6f19e2c7e0a2ccdf37eb1b74"
    );
}
