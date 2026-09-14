// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_core_lineage::{
    LineageError, ProfiledDigest32, RootAnchorFacts, TransitionFacts, project_rooted_lineage,
};

const DOMAIN_PROFILE: &str =
    "mycelix-constitutional-root-lineage-domain-v1-sha256-framed-semantic";
const ROOT_PROFILE: &str =
    "mycelix-constitutional-trust-root-v1-sha256-framed-semantic";
const SOURCE_PROFILE: &str =
    "mycelix-constitutional-root-source-descriptor-v1-sha256-framed-semantic";
const TRANSITION_PROFILE: &str =
    "mycelix-constitutional-root-transition-v1-sha256-framed-semantic";

const ROOT0: &str = "2684cc6c8b8f13e2e564b0bb7682dde0d333124d4a5db4ed04eaae1ad99023c9";
const ROOT1: &str = "8f868c8c33698e6734ead90df9383f81141b6ff52d8ab765aa2aa5d6a028810c";
const ROOT2: &str = "49be9b61e1c3f2b8f016e3b4e9c416dc83c689760a83cfb28ec9f6ff1f7d8825";
const SOURCE0: &str = "aa78b5370cb1def51be692498ef6a5bc6c51612155fb9167421d8b10cd01cb84";
const SOURCE1: &str = "9c4db710b045aee584019e6ff1ec1d0721df11fa1da0c8b2d4172e21fc96a1e1";
const SOURCE2: &str = "ab5a3de2df73dc4b22ea438795ceb4be851338615106067f524d080e13ad873e";
const EDGE0: &str = "19e565417b53d57e0e73250dae14d33665d9461839083ce6260dc296d03cf2e1";
const EDGE1: &str = "4d41a29c0c3ad8a51bde874d2fb25a709b1a9b7099fa82da855eabb4374111f7";
const DOMAIN: &str = "d95e2f1af546480ebfcc1218f7ad671ac38ac3898cdce89a91328407dc3bc005";
const CORE: &str = "579c8810162e88958117b085fad6d01b599d8222a37fcd6e540f787c4fcdd5b1";

const T0: u64 = 1_810_000_000_000;
const T1: u64 = 1_810_000_100_000;
const T2: u64 = 1_810_000_200_000;

fn decode_hex32(value: &str) -> [u8; 32] {
    assert_eq!(value.len(), 64);
    let mut output = [0_u8; 32];
    for (index, slot) in output.iter_mut().enumerate() {
        *slot = u8::from_str_radix(&value[index * 2..index * 2 + 2], 16)
            .expect("frozen vector hex");
    }
    output
}

fn id(profile: &str, value: &str) -> ProfiledDigest32 {
    ProfiledDigest32::try_new(profile, decode_hex32(value)).expect("valid frozen identity")
}

fn domain() -> ProfiledDigest32 {
    id(DOMAIN_PROFILE, DOMAIN)
}

fn root_anchor() -> RootAnchorFacts {
    RootAnchorFacts::new(
        domain(),
        0,
        id(ROOT_PROFILE, ROOT0),
        T0,
        id(SOURCE_PROFILE, SOURCE0),
    )
}

fn edge0() -> TransitionFacts {
    TransitionFacts::new(
        domain(),
        0,
        id(ROOT_PROFILE, ROOT0),
        id(SOURCE_PROFILE, SOURCE0),
        1,
        id(ROOT_PROFILE, ROOT1),
        id(SOURCE_PROFILE, SOURCE1),
        id(TRANSITION_PROFILE, EDGE0),
        T1,
    )
}

fn edge1() -> TransitionFacts {
    TransitionFacts::new(
        domain(),
        1,
        id(ROOT_PROFILE, ROOT1),
        id(SOURCE_PROFILE, SOURCE1),
        2,
        id(ROOT_PROFILE, ROOT2),
        id(SOURCE_PROFILE, SOURCE2),
        id(TRANSITION_PROFILE, EDGE1),
        T2,
    )
}

fn main() {
    let first = edge0();
    let second = edge1();
    let projected = project_rooted_lineage(root_anchor(), &[second.clone(), first.clone()])
        .expect("fresh two-edge lineage must project");

    assert_eq!(projected.endpoint_generation(), 2);
    assert_eq!(projected.endpoint_node_identity(), &id(ROOT_PROFILE, ROOT2));
    assert_eq!(
        projected.endpoint_source_descriptor_identity(),
        &id(SOURCE_PROFILE, SOURCE2)
    );
    assert_eq!(projected.stable_commitment(), &decode_hex32(CORE));
    assert!(!projected.grants_currentness());
    assert!(!projected.grants_effect_authority());

    let duplicated = project_rooted_lineage(
        root_anchor(),
        &[second.clone(), first.clone(), first.clone()],
    )
    .expect("exact duplicate must normalize");
    assert_eq!(duplicated.stable_commitment(), projected.stable_commitment());

    let wrong_middle = TransitionFacts::new(
        domain(),
        1,
        id(ROOT_PROFILE, &"77".repeat(32)),
        id(SOURCE_PROFILE, SOURCE1),
        2,
        id(ROOT_PROFILE, ROOT2),
        id(SOURCE_PROFILE, SOURCE2),
        id(TRANSITION_PROFILE, &"78".repeat(32)),
        T2,
    );
    assert_eq!(
        project_rooted_lineage(root_anchor(), &[first.clone(), wrong_middle]),
        Err(LineageError::PredecessorNodeMismatch)
    );

    let parallel = TransitionFacts::new(
        domain(),
        0,
        id(ROOT_PROFILE, ROOT0),
        id(SOURCE_PROFILE, SOURCE0),
        1,
        id(ROOT_PROFILE, ROOT1),
        id(SOURCE_PROFILE, SOURCE1),
        id(TRANSITION_PROFILE, &"79".repeat(32)),
        T1,
    );
    assert_eq!(
        project_rooted_lineage(root_anchor(), &[first.clone(), parallel]),
        Err(LineageError::ParallelTransitionConflict)
    );

    let fork = TransitionFacts::new(
        domain(),
        0,
        id(ROOT_PROFILE, ROOT0),
        id(SOURCE_PROFILE, SOURCE0),
        1,
        id(ROOT_PROFILE, &"7a".repeat(32)),
        id(SOURCE_PROFILE, &"7b".repeat(32)),
        id(TRANSITION_PROFILE, &"7c".repeat(32)),
        T1,
    );
    assert_eq!(
        project_rooted_lineage(root_anchor(), &[first.clone(), fork]),
        Err(LineageError::ForkConflict)
    );

    let gap = TransitionFacts::new(
        domain(),
        0,
        id(ROOT_PROFILE, ROOT0),
        id(SOURCE_PROFILE, SOURCE0),
        2,
        id(ROOT_PROFILE, ROOT2),
        id(SOURCE_PROFILE, SOURCE2),
        id(TRANSITION_PROFILE, &"7d".repeat(32)),
        T2,
    );
    assert_eq!(
        project_rooted_lineage(root_anchor(), &[gap]),
        Err(LineageError::DiscontinuousGeneration)
    );

    let time_regression = TransitionFacts::new(
        domain(),
        1,
        id(ROOT_PROFILE, ROOT1),
        id(SOURCE_PROFILE, SOURCE1),
        2,
        id(ROOT_PROFILE, ROOT2),
        id(SOURCE_PROFILE, SOURCE2),
        id(TRANSITION_PROFILE, &"7e".repeat(32)),
        T1 - 1,
    );
    assert_eq!(
        project_rooted_lineage(root_anchor(), &[first.clone(), time_regression]),
        Err(LineageError::EffectiveTimeRegression)
    );

    let wrong_domain = TransitionFacts::new(
        id(DOMAIN_PROFILE, &"7f".repeat(32)),
        0,
        id(ROOT_PROFILE, ROOT0),
        id(SOURCE_PROFILE, SOURCE0),
        1,
        id(ROOT_PROFILE, ROOT1),
        id(SOURCE_PROFILE, SOURCE1),
        id(TRANSITION_PROFILE, &"80".repeat(32)),
        T1,
    );
    assert_eq!(
        project_rooted_lineage(root_anchor(), &[wrong_domain]),
        Err(LineageError::DomainMismatch)
    );

    println!("GOVSYS-003C-B CORE-LINEAGE two-edge bridge PASS: {CORE}");
}
