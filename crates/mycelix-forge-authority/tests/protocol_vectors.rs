// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_forge_authority::{
    BoundGenesisAuthority, Capability, CapabilityRule, Digest, DigestAlgorithm, PrincipalGrant,
    PrincipalId, ProjectIdentity, ProjectIdentitySeed, RootAuthorityPolicy, GENESIS_NONCE_LEN,
};

fn digest(byte: u8) -> Digest {
    Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
}

fn principal(byte: u8) -> PrincipalId {
    PrincipalId::new(digest(byte))
}

fn grant(principal: PrincipalId, capabilities: &[Capability]) -> PrincipalGrant {
    PrincipalGrant::new(principal, capabilities.iter().copied()).unwrap()
}

#[test]
fn frozen_v1_root_to_genesis_vector() {
    let root_policy = RootAuthorityPolicy::new(
        vec![
            grant(
                principal(1),
                &[Capability::ManageAuthority, Capability::ReviewSource],
            ),
            grant(principal(2), &[Capability::ManageAuthority]),
        ],
        vec![
            CapabilityRule::new(Capability::ManageAuthority, 2).unwrap(),
            CapabilityRule::new(Capability::ReviewSource, 1).unwrap(),
        ],
    )
    .unwrap();

    let root_commitment = root_policy.commitment(DigestAlgorithm::Sha256).unwrap();
    assert_eq!(
        root_commitment.to_string(),
        "sha256:43526f6aa2a4246c829b11445a8045c629c724e1577d722c2a9e832d43a99a05"
    );

    let seed = ProjectIdentitySeed::new([0x44; GENESIS_NONCE_LEN], root_commitment);
    let project = ProjectIdentity::derive(&seed, DigestAlgorithm::Sha256).unwrap();
    assert_eq!(
        project.to_string(),
        "mycelix:forge:project:v1:sha256:dbb70e8c84ee62a55c7eece90f5eb0be180cfdd9bd2d92751075fcf5ed04e694"
    );

    let bound = BoundGenesisAuthority::new(&seed, project, root_policy, 1_000, None).unwrap();
    assert_eq!(
        bound
            .epoch()
            .digest(DigestAlgorithm::Sha256)
            .unwrap()
            .to_string(),
        "sha256:9bf18008ac319f79336437a774695e28e26c22c62aa5b7914b34a8639f9c2628"
    );
}
