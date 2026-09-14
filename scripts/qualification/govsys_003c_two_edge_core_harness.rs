// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Qualification-only two-edge bridge into exact CORE-LINEAGE Stage 1.

use mycelix_core_lineage::{
    LineageError, ProfiledDigest32, RootAnchorFacts, TransitionFacts, project_rooted_lineage,
};

const DOMAIN_PROFILE: &str = "mycelix-constitutional-root-lineage-domain-v1-sha256-framed-semantic";
const ROOT_PROFILE: &str = "mycelix-constitutional-trust-root-v1-sha256-framed-semantic";
const SOURCE_PROFILE: &str =
    "mycelix-constitutional-root-source-descriptor-v1-sha256-framed-semantic";
const TRANSITION_PROFILE: &str = "mycelix-constitutional-root-transition-v1-sha256-framed-semantic";

fn nibble(value: u8) -> u8 {
    match value {
        b'0'..=b'9' => value - b'0',
        b'a'..=b'f' => value - b'a' + 10,
        b'A'..=b'F' => value - b'A' + 10,
        _ => panic!("non-hex qualification fixture"),
    }
}

fn hex32(value: &str) -> [u8; 32] {
    assert_eq!(value.len(), 64);
    let bytes = value.as_bytes();
    let mut out = [0_u8; 32];
    for index in 0..32 {
        out[index] = (nibble(bytes[index * 2]) << 4) | nibble(bytes[index * 2 + 1]);
    }
    out
}

fn id(profile: &str, digest: &str) -> ProfiledDigest32 {
    ProfiledDigest32::try_new(profile, hex32(digest)).expect("valid qualification identity")
}

fn domain() -> ProfiledDigest32 {
    id(
        DOMAIN_PROFILE,
        "6067a895dfec8c8a6f5650e5f2031cf981c116f5b22fc1e5f316893f6dcb7f7a",
    )
}

fn edge01() -> TransitionFacts {
    TransitionFacts::new(
        domain(),
        0,
        id(
            ROOT_PROFILE,
            "978bd949775e927e55dc6b7c5e7ef2ca18ebadbfc71b8782950af0c22644ed8b",
        ),
        id(
            SOURCE_PROFILE,
            "a5d9e84877dc89e83462e65fdc1cccc14af0e9438a6f7b67ccdf4a4c51baecec",
        ),
        1,
        id(
            ROOT_PROFILE,
            "35b89e7c6ad5ac7c8e7eccd80182f2f4c047778b538068fa4e63ce2ab61ac526",
        ),
        id(
            SOURCE_PROFILE,
            "c20eb54d468d2878acf6966fd8efd61674bb1b0412c28fd7952be0830bd3bef8",
        ),
        id(
            TRANSITION_PROFILE,
            "3144c9cbbdc7e40bcce842a26e52a78839a88f900713551c081ffe9ebf1343ed",
        ),
        1_900_000_100_000,
    )
}

fn edge12() -> TransitionFacts {
    TransitionFacts::new(
        domain(),
        1,
        id(
            ROOT_PROFILE,
            "35b89e7c6ad5ac7c8e7eccd80182f2f4c047778b538068fa4e63ce2ab61ac526",
        ),
        id(
            SOURCE_PROFILE,
            "c20eb54d468d2878acf6966fd8efd61674bb1b0412c28fd7952be0830bd3bef8",
        ),
        2,
        id(
            ROOT_PROFILE,
            "9d9b2901ba66fa3bbdd53b72f2897c05aab0f6fd5d1e3e3fe01da7d6be9ccbd4",
        ),
        id(
            SOURCE_PROFILE,
            "3c6aa2aea5a138e1da33f0cd96225c6b26adb5e4651151779ab7617311bae99c",
        ),
        id(
            TRANSITION_PROFILE,
            "8b5fab72940e3f6280abbe7b5481c7775cd872caffa33609a38b233bf3f15f19",
        ),
        1_900_000_200_000,
    )
}

fn root() -> RootAnchorFacts {
    RootAnchorFacts::new(
        domain(),
        0,
        id(
            ROOT_PROFILE,
            "978bd949775e927e55dc6b7c5e7ef2ca18ebadbfc71b8782950af0c22644ed8b",
        ),
        1_900_000_000_000,
        id(
            SOURCE_PROFILE,
            "a5d9e84877dc89e83462e65fdc1cccc14af0e9438a6f7b67ccdf4a4c51baecec",
        ),
    )
}

fn main() {
    let first = edge01();
    let second = edge12();
    let expected = hex32("3802a7e337444b8252dde31d619c14d2c443f3ab35dea825801cb6eb265169e4");

    let projected = project_rooted_lineage(root(), &[first.clone(), second.clone()])
        .expect("qualified two-edge lineage");
    assert_eq!(projected.endpoint_generation(), 2);
    assert_eq!(
        projected.endpoint_node_identity(),
        &id(
            ROOT_PROFILE,
            "9d9b2901ba66fa3bbdd53b72f2897c05aab0f6fd5d1e3e3fe01da7d6be9ccbd4"
        )
    );
    assert_eq!(
        projected.endpoint_source_descriptor_identity(),
        &id(
            SOURCE_PROFILE,
            "3c6aa2aea5a138e1da33f0cd96225c6b26adb5e4651151779ab7617311bae99c"
        )
    );
    assert_eq!(projected.stable_commitment(), &expected);
    assert!(!projected.grants_currentness());
    assert!(!projected.grants_effect_authority());

    let reversed = project_rooted_lineage(root(), &[second.clone(), first.clone()])
        .expect("arrival order must not affect stable history");
    assert_eq!(reversed.stable_commitment(), &expected);

    let duplicated = project_rooted_lineage(
        root(),
        &[first.clone(), second.clone(), first.clone(), second.clone()],
    )
    .expect("exact semantic duplicates normalize");
    assert_eq!(duplicated.stable_commitment(), &expected);

    let parallel_second = TransitionFacts::new(
        domain(),
        1,
        id(
            ROOT_PROFILE,
            "35b89e7c6ad5ac7c8e7eccd80182f2f4c047778b538068fa4e63ce2ab61ac526",
        ),
        id(
            SOURCE_PROFILE,
            "c20eb54d468d2878acf6966fd8efd61674bb1b0412c28fd7952be0830bd3bef8",
        ),
        2,
        id(
            ROOT_PROFILE,
            "9d9b2901ba66fa3bbdd53b72f2897c05aab0f6fd5d1e3e3fe01da7d6be9ccbd4",
        ),
        id(
            SOURCE_PROFILE,
            "3c6aa2aea5a138e1da33f0cd96225c6b26adb5e4651151779ab7617311bae99c",
        ),
        id(TRANSITION_PROFILE, &"77".repeat(32)),
        1_900_000_200_000,
    );
    assert_eq!(
        project_rooted_lineage(root(), &[first.clone(), second.clone(), parallel_second]),
        Err(LineageError::ParallelTransitionConflict)
    );

    let fork_second = TransitionFacts::new(
        domain(),
        1,
        id(
            ROOT_PROFILE,
            "35b89e7c6ad5ac7c8e7eccd80182f2f4c047778b538068fa4e63ce2ab61ac526",
        ),
        id(
            SOURCE_PROFILE,
            "c20eb54d468d2878acf6966fd8efd61674bb1b0412c28fd7952be0830bd3bef8",
        ),
        2,
        id(ROOT_PROFILE, &"88".repeat(32)),
        id(SOURCE_PROFILE, &"99".repeat(32)),
        id(TRANSITION_PROFILE, &"aa".repeat(32)),
        1_900_000_200_000,
    );
    assert_eq!(
        project_rooted_lineage(root(), &[first.clone(), second, fork_second]),
        Err(LineageError::ForkConflict)
    );

    assert_eq!(
        project_rooted_lineage(root(), &[edge12()]),
        Err(LineageError::DiscontinuousGeneration)
    );

    println!(
        "GOVSYS-003C two-edge CORE-LINEAGE PASS: 3802a7e337444b8252dde31d619c14d2c443f3ab35dea825801cb6eb265169e4"
    );
}
