// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
#![no_std]

extern crate alloc;

use alloc::collections::BTreeSet;
use alloc::string::String;
use alloc::vec::Vec;
use sha2::{Digest, Sha256};

pub const PROTOCOL_VERSION: &str = "mycelix-constitutional-trust-root-v0.1";
pub const IDENTITY_PROFILE: &str = "mycelix-constitutional-trust-root-v1-sha256-framed-semantic";
pub const SOURCE_DESCRIPTOR_PROFILE: &str =
    "mycelix-constitutional-root-source-descriptor-v1-sha256-framed-semantic";
pub const ROTATION_AUTHORITY_PROFILE: &str =
    "mycelix-constitutional-root-rotation-authority-v1-sha256-framed-semantic";

const DOMAIN: &[u8] = b"mycelix/public-institution/constitutional-trust-root/v1";
const SOURCE_DESCRIPTOR_DOMAIN: &[u8] =
    b"mycelix/public-institution/constitutional-root-source-descriptor/v1";
const ROTATION_AUTHORITY_DOMAIN: &[u8] =
    b"mycelix/public-institution/constitutional-root-rotation-authority/v1";

const MAX_ID_BYTES: usize = 512;
const MAX_PROFILE_BYTES: usize = 256;
const MAX_NAMESPACE_BYTES: usize = 1024;
const MAX_RULEBOOK_VERSION_BYTES: usize = 128;
const MAX_AUTHORIZED_POLICY_SCOPES: usize = 1024;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Rulebook {
    pub id: String,
    pub version: String,
    pub digest: [u8; 32],
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AuthorizedPolicyScope {
    pub policy_identity_profile: String,
    pub policy_registry_namespace: String,
    pub provider_authority_institution_id: String,
    pub provider_authority_jurisdiction_id: Option<String>,
    pub provider_authority_rulebook: Rulebook,
    pub required_provider_capability: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConstitutionalRoot {
    pub protocol_version: String,
    pub institution_id: String,
    pub jurisdiction_id: Option<String>,
    pub constitutional_rulebook: Rulebook,
    pub generation: u64,
    pub predecessor_root_digest: Option<[u8; 32]>,
    pub bootstrap_mode: String,
    pub bootstrap_profile: String,
    pub authoritative_root_source_ref: String,
    pub root_coverage_profile: String,
    pub root_source_verification_profile: String,
    pub root_source_anchor_digest: [u8; 32],
    pub authorized_policy_scopes: Vec<AuthorizedPolicyScope>,
    pub valid_from_ms: u64,
    pub expires_at_ms: Option<u64>,
    pub rotation_mode: String,
    pub rotation_profile: Option<String>,
    pub rotation_authority_anchor_digest: Option<[u8; 32]>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RootError {
    WrongProtocolVersion,
    InvalidText,
    ZeroDigest,
    DuplicatePolicyScopeKey,
    DuplicatePolicyScope,
    SelfAuthorizingPolicyProfile,
    TooManyPolicyScopes,
    InvalidGenerationPredecessorRelation,
    InvalidValidityInterval,
    UnsupportedBootstrapMode,
    UnsupportedRotationMode,
    InvalidRotationShape,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ProfiledDigest32 {
    pub profile: &'static str,
    pub digest: [u8; 32],
}

#[derive(Debug, Eq, PartialEq)]
pub struct QualifiedConstitutionalRootA {
    root: ConstitutionalRoot,
    root_identity: ProfiledDigest32,
    source_descriptor_identity: ProfiledDigest32,
    rotation_authority_identity: Option<ProfiledDigest32>,
}

impl QualifiedConstitutionalRootA {
    pub fn root(&self) -> &ConstitutionalRoot {
        &self.root
    }

    pub const fn root_identity(&self) -> &ProfiledDigest32 {
        &self.root_identity
    }

    pub const fn source_descriptor_identity(&self) -> &ProfiledDigest32 {
        &self.source_descriptor_identity
    }

    pub const fn rotation_authority_identity(&self) -> Option<&ProfiledDigest32> {
        self.rotation_authority_identity.as_ref()
    }

    pub const fn grants_currentness(&self) -> bool {
        false
    }

    pub const fn grants_effect_authority(&self) -> bool {
        false
    }
}

pub fn qualify_root(root: ConstitutionalRoot) -> Result<QualifiedConstitutionalRootA, RootError> {
    validate_root(&root)?;
    let root_digest = sha256(&canonical_bytes_validated(&root));
    let source_digest = sha256(&source_descriptor_bytes_validated(&root));
    let rotation_digest = if root.rotation_mode == "immutable" {
        None
    } else {
        Some(sha256(&rotation_authority_bytes_validated(&root)))
    };

    Ok(QualifiedConstitutionalRootA {
        root,
        root_identity: ProfiledDigest32 {
            profile: IDENTITY_PROFILE,
            digest: root_digest,
        },
        source_descriptor_identity: ProfiledDigest32 {
            profile: SOURCE_DESCRIPTOR_PROFILE,
            digest: source_digest,
        },
        rotation_authority_identity: rotation_digest.map(|digest| ProfiledDigest32 {
            profile: ROTATION_AUTHORITY_PROFILE,
            digest,
        }),
    })
}

pub fn validate_root(root: &ConstitutionalRoot) -> Result<(), RootError> {
    if root.protocol_version != PROTOCOL_VERSION {
        return Err(RootError::WrongProtocolVersion);
    }

    validate_text(&root.institution_id, MAX_ID_BYTES)?;
    validate_optional_text(root.jurisdiction_id.as_deref(), MAX_ID_BYTES)?;
    validate_rulebook(&root.constitutional_rulebook)?;

    match (root.generation, root.predecessor_root_digest) {
        (0, Some(_)) | (1.., None) => {
            return Err(RootError::InvalidGenerationPredecessorRelation);
        }
        _ => {}
    }
    if let Some(predecessor) = root.predecessor_root_digest {
        validate_digest(predecessor)?;
    }

    match root.bootstrap_mode.as_str() {
        "pinned-constitutional-commitment"
        | "genesis-governance-decision"
        | "external-institutional-credential" => {}
        _ => return Err(RootError::UnsupportedBootstrapMode),
    }
    validate_text(&root.bootstrap_profile, MAX_PROFILE_BYTES)?;
    validate_text(&root.authoritative_root_source_ref, MAX_NAMESPACE_BYTES)?;
    validate_text(&root.root_coverage_profile, MAX_PROFILE_BYTES)?;
    validate_text(&root.root_source_verification_profile, MAX_PROFILE_BYTES)?;
    validate_digest(root.root_source_anchor_digest)?;

    if root.authorized_policy_scopes.len() > MAX_AUTHORIZED_POLICY_SCOPES {
        return Err(RootError::TooManyPolicyScopes);
    }
    let mut keys = BTreeSet::<(String, String)>::new();
    let mut encoded_scopes = Vec::<Vec<u8>>::with_capacity(root.authorized_policy_scopes.len());
    for scope in &root.authorized_policy_scopes {
        validate_scope(scope)?;
        let key = (
            scope.policy_identity_profile.clone(),
            scope.policy_registry_namespace.clone(),
        );
        if !keys.insert(key) {
            return Err(RootError::DuplicatePolicyScopeKey);
        }
        encoded_scopes.push(scope_bytes(scope));
    }
    encoded_scopes.sort();
    if encoded_scopes.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(RootError::DuplicatePolicyScope);
    }

    if let Some(expires_at_ms) = root.expires_at_ms
        && expires_at_ms <= root.valid_from_ms
    {
        return Err(RootError::InvalidValidityInterval);
    }

    match root.rotation_mode.as_str() {
        "immutable" => {
            if root.rotation_profile.is_some() || root.rotation_authority_anchor_digest.is_some() {
                return Err(RootError::InvalidRotationShape);
            }
        }
        "predecessor-authorized" => {
            let profile = root
                .rotation_profile
                .as_deref()
                .ok_or(RootError::InvalidRotationShape)?;
            validate_text(profile, MAX_PROFILE_BYTES)?;
            let anchor = root
                .rotation_authority_anchor_digest
                .ok_or(RootError::InvalidRotationShape)?;
            validate_digest(anchor)?;
        }
        _ => return Err(RootError::UnsupportedRotationMode),
    }

    Ok(())
}

pub fn canonical_bytes(root: &ConstitutionalRoot) -> Result<Vec<u8>, RootError> {
    validate_root(root)?;
    Ok(canonical_bytes_validated(root))
}

pub fn root_identity(root: &ConstitutionalRoot) -> Result<ProfiledDigest32, RootError> {
    Ok(ProfiledDigest32 {
        profile: IDENTITY_PROFILE,
        digest: sha256(&canonical_bytes(root)?),
    })
}

pub fn source_descriptor_identity(
    root: &ConstitutionalRoot,
) -> Result<ProfiledDigest32, RootError> {
    validate_root(root)?;
    Ok(ProfiledDigest32 {
        profile: SOURCE_DESCRIPTOR_PROFILE,
        digest: sha256(&source_descriptor_bytes_validated(root)),
    })
}

pub fn rotation_authority_identity(
    root: &ConstitutionalRoot,
) -> Result<Option<ProfiledDigest32>, RootError> {
    validate_root(root)?;
    if root.rotation_mode == "immutable" {
        return Ok(None);
    }
    Ok(Some(ProfiledDigest32 {
        profile: ROTATION_AUTHORITY_PROFILE,
        digest: sha256(&rotation_authority_bytes_validated(root)),
    }))
}

fn validate_scope(scope: &AuthorizedPolicyScope) -> Result<(), RootError> {
    validate_text(&scope.policy_identity_profile, MAX_PROFILE_BYTES)?;
    if scope.policy_identity_profile == IDENTITY_PROFILE {
        return Err(RootError::SelfAuthorizingPolicyProfile);
    }
    validate_text(&scope.policy_registry_namespace, MAX_NAMESPACE_BYTES)?;
    validate_text(&scope.provider_authority_institution_id, MAX_ID_BYTES)?;
    validate_optional_text(
        scope.provider_authority_jurisdiction_id.as_deref(),
        MAX_ID_BYTES,
    )?;
    validate_rulebook(&scope.provider_authority_rulebook)?;
    validate_text(&scope.required_provider_capability, MAX_ID_BYTES)?;
    Ok(())
}

fn validate_rulebook(rulebook: &Rulebook) -> Result<(), RootError> {
    validate_text(&rulebook.id, MAX_ID_BYTES)?;
    validate_text(&rulebook.version, MAX_RULEBOOK_VERSION_BYTES)?;
    validate_digest(rulebook.digest)
}

fn validate_optional_text(value: Option<&str>, max_bytes: usize) -> Result<(), RootError> {
    if let Some(value) = value {
        validate_text(value, max_bytes)?;
    }
    Ok(())
}

fn validate_text(value: &str, max_bytes: usize) -> Result<(), RootError> {
    let raw = value.as_bytes();
    if raw.is_empty() || raw.len() > max_bytes {
        return Err(RootError::InvalidText);
    }
    if raw.first() == Some(&b' ') || raw.last() == Some(&b' ') {
        return Err(RootError::InvalidText);
    }
    if raw.iter().any(|byte| *byte < 0x20 || *byte == 0x7f) {
        return Err(RootError::InvalidText);
    }
    Ok(())
}

fn validate_digest(digest: [u8; 32]) -> Result<(), RootError> {
    if digest == [0; 32] {
        return Err(RootError::ZeroDigest);
    }
    Ok(())
}

fn canonical_bytes_validated(root: &ConstitutionalRoot) -> Vec<u8> {
    let mut output = Vec::new();
    output.extend_from_slice(DOMAIN);
    frame_text(&mut output, IDENTITY_PROFILE);
    frame_text(&mut output, &root.protocol_version);
    frame_text(&mut output, &root.institution_id);
    frame_optional_text(&mut output, root.jurisdiction_id.as_deref());
    frame_rulebook(&mut output, &root.constitutional_rulebook);
    frame_u64(&mut output, root.generation);
    frame_optional_digest(&mut output, root.predecessor_root_digest.as_ref());
    frame_text(&mut output, &root.bootstrap_mode);
    frame_text(&mut output, &root.bootstrap_profile);
    frame_text(&mut output, &root.authoritative_root_source_ref);
    frame_text(&mut output, &root.root_coverage_profile);
    frame_text(&mut output, &root.root_source_verification_profile);
    frame(&mut output, &root.root_source_anchor_digest);

    let mut scopes: Vec<Vec<u8>> = root
        .authorized_policy_scopes
        .iter()
        .map(scope_bytes)
        .collect();
    scopes.sort();
    frame_u64(&mut output, scopes.len() as u64);
    for scope in scopes {
        output.extend_from_slice(&scope);
    }

    frame_u64(&mut output, root.valid_from_ms);
    frame_optional_u64(&mut output, root.expires_at_ms);
    frame_text(&mut output, &root.rotation_mode);
    frame_optional_text(&mut output, root.rotation_profile.as_deref());
    frame_optional_digest(&mut output, root.rotation_authority_anchor_digest.as_ref());
    output
}

fn source_descriptor_bytes_validated(root: &ConstitutionalRoot) -> Vec<u8> {
    let mut output = Vec::new();
    output.extend_from_slice(SOURCE_DESCRIPTOR_DOMAIN);
    frame_text(&mut output, SOURCE_DESCRIPTOR_PROFILE);
    frame_text(&mut output, &root.authoritative_root_source_ref);
    frame_text(&mut output, &root.root_coverage_profile);
    frame_text(&mut output, &root.root_source_verification_profile);
    frame(&mut output, &root.root_source_anchor_digest);
    output
}

fn rotation_authority_bytes_validated(root: &ConstitutionalRoot) -> Vec<u8> {
    let mut output = Vec::new();
    output.extend_from_slice(ROTATION_AUTHORITY_DOMAIN);
    frame_text(&mut output, ROTATION_AUTHORITY_PROFILE);
    frame_text(
        &mut output,
        root.rotation_profile
            .as_deref()
            .expect("validated rotatable root carries rotation profile"),
    );
    frame(
        &mut output,
        &root
            .rotation_authority_anchor_digest
            .expect("validated rotatable root carries rotation anchor"),
    );
    output
}

fn scope_bytes(scope: &AuthorizedPolicyScope) -> Vec<u8> {
    let mut output = Vec::new();
    frame_text(&mut output, &scope.policy_identity_profile);
    frame_text(&mut output, &scope.policy_registry_namespace);
    frame_text(&mut output, &scope.provider_authority_institution_id);
    frame_optional_text(
        &mut output,
        scope.provider_authority_jurisdiction_id.as_deref(),
    );
    frame_rulebook(&mut output, &scope.provider_authority_rulebook);
    frame_text(&mut output, &scope.required_provider_capability);
    output
}

fn frame_rulebook(output: &mut Vec<u8>, rulebook: &Rulebook) {
    frame_text(output, &rulebook.id);
    frame_text(output, &rulebook.version);
    frame(output, &rulebook.digest);
}

fn frame_optional_text(output: &mut Vec<u8>, value: Option<&str>) {
    match value {
        None => frame(output, &[0]),
        Some(value) => {
            frame(output, &[1]);
            frame_text(output, value);
        }
    }
}

fn frame_optional_digest(output: &mut Vec<u8>, value: Option<&[u8; 32]>) {
    match value {
        None => frame(output, &[0]),
        Some(value) => {
            frame(output, &[1]);
            frame(output, value);
        }
    }
}

fn frame_optional_u64(output: &mut Vec<u8>, value: Option<u64>) {
    match value {
        None => frame(output, &[0]),
        Some(value) => {
            frame(output, &[1]);
            frame_u64(output, value);
        }
    }
}

fn frame_text(output: &mut Vec<u8>, value: &str) {
    frame(output, value.as_bytes());
}

fn frame_u64(output: &mut Vec<u8>, value: u64) {
    frame(output, &value.to_le_bytes());
}

fn frame(output: &mut Vec<u8>, raw: &[u8]) {
    output.extend_from_slice(&(raw.len() as u64).to_le_bytes());
    output.extend_from_slice(raw);
}

fn sha256(raw: &[u8]) -> [u8; 32] {
    let digest = Sha256::digest(raw);
    let mut output = [0_u8; 32];
    output.copy_from_slice(&digest);
    output
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;
    use alloc::vec;

    fn digest(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn hex32(value: &str) -> [u8; 32] {
        assert_eq!(value.len(), 64);
        let mut output = [0_u8; 32];
        for (index, slot) in output.iter_mut().enumerate() {
            let pair = &value.as_bytes()[index * 2..index * 2 + 2];
            *slot = (hex_nibble(pair[0]) << 4) | hex_nibble(pair[1]);
        }
        output
    }

    fn hex_nibble(value: u8) -> u8 {
        match value {
            b'0'..=b'9' => value - b'0',
            b'a'..=b'f' => value - b'a' + 10,
            b'A'..=b'F' => value - b'A' + 10,
            _ => panic!("invalid hex fixture"),
        }
    }

    fn provider_rulebook() -> Rulebook {
        Rulebook {
            id: "rulebook:city-clerk:v1".to_string(),
            version: "1.0.0".to_string(),
            digest: digest(0x22),
        }
    }

    fn scopes() -> Vec<AuthorizedPolicyScope> {
        vec![
            AuthorizedPolicyScope {
                policy_identity_profile: "mycelix-review-policy-v1-blake3-framed-semantic"
                    .to_string(),
                policy_registry_namespace: "registry:review-policy:example-city".to_string(),
                provider_authority_institution_id: "institution:city-clerk".to_string(),
                provider_authority_jurisdiction_id: Some("jurisdiction:example-city".to_string()),
                provider_authority_rulebook: provider_rulebook(),
                required_provider_capability: "administration.review-policy.currentness.attest"
                    .to_string(),
            },
            AuthorizedPolicyScope {
                policy_identity_profile:
                    "mycelix-procedure-policy-currentness-provider-v1-blake3-framed-semantic"
                        .to_string(),
                policy_registry_namespace: "registry:procedure-policy:example-city".to_string(),
                provider_authority_institution_id: "institution:city-clerk".to_string(),
                provider_authority_jurisdiction_id: Some("jurisdiction:example-city".to_string()),
                provider_authority_rulebook: provider_rulebook(),
                required_provider_capability: "administration.policy.currentness.attest"
                    .to_string(),
            },
        ]
    }

    fn golden_root() -> ConstitutionalRoot {
        ConstitutionalRoot {
            protocol_version: PROTOCOL_VERSION.to_string(),
            institution_id: "institution:city-of-example".to_string(),
            jurisdiction_id: Some("jurisdiction:example-city".to_string()),
            constitutional_rulebook: Rulebook {
                id: "rulebook:city-charter:v1".to_string(),
                version: "1.0.0".to_string(),
                digest: digest(0x11),
            },
            generation: 0,
            predecessor_root_digest: None,
            bootstrap_mode: "pinned-constitutional-commitment".to_string(),
            bootstrap_profile: "deployment-pinned-root-digest-v1".to_string(),
            authoritative_root_source_ref: "registry:constitutional-root:example-city".to_string(),
            root_coverage_profile: "mycelix-constitutional-root-covered-head-v1".to_string(),
            root_source_verification_profile: "mycelix-constitutional-root-source-verification-v1"
                .to_string(),
            root_source_anchor_digest: digest(0x33),
            authorized_policy_scopes: scopes(),
            valid_from_ms: 1_800_000_000_000,
            expires_at_ms: None,
            rotation_mode: "predecessor-authorized".to_string(),
            rotation_profile: Some("constitutional-root-rotation-v1".to_string()),
            rotation_authority_anchor_digest: Some(digest(0x44)),
        }
    }

    fn transition_predecessor() -> ConstitutionalRoot {
        let mut root = golden_root();
        root.expires_at_ms = Some(1_800_000_200_000);
        root.rotation_authority_anchor_digest = Some(hex32(
            "32d29fa9f5a28f5ef5ca3f298902add00f1ddd79007b729d0c12eae77358367f",
        ));
        root
    }

    fn transition_successor() -> ConstitutionalRoot {
        let mut root = golden_root();
        root.constitutional_rulebook = Rulebook {
            id: "rulebook:city-charter:v2".to_string(),
            version: "2.0.0".to_string(),
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

    #[test]
    fn golden_identity_matches_qualified_python() {
        let qualified = qualify_root(golden_root()).expect("golden Root-A must qualify");
        assert_eq!(
            qualified.root_identity().digest,
            hex32("c3f9ba9b323f20c2d2ebd597e424857e8f3459d31393886fd6a7f835a6a93d6d")
        );
        assert_eq!(
            qualified.source_descriptor_identity().digest,
            hex32("f97a96e20ce6dd0c86c67e1590252abc678dfd6401128ab44ced4a66ec088126")
        );
        assert_eq!(
            qualified
                .rotation_authority_identity()
                .expect("rotatable")
                .digest,
            hex32("cf86718a53410b1e5e38cf5c552b3e4448772da97533fbd08c7b9bbfbee909ae")
        );
        assert!(!qualified.grants_currentness());
        assert!(!qualified.grants_effect_authority());
    }

    #[test]
    fn transition_pair_matches_qualified_python() {
        let predecessor = qualify_root(transition_predecessor()).expect("predecessor qualifies");
        assert_eq!(
            predecessor.root_identity().digest,
            hex32("b7a0c7cb28f182d06367d4bf1d3cc4f7d82094701d955eeaa7ec53153d2080a6")
        );
        assert_eq!(
            predecessor.source_descriptor_identity().digest,
            hex32("f97a96e20ce6dd0c86c67e1590252abc678dfd6401128ab44ced4a66ec088126")
        );
        assert_eq!(
            predecessor
                .rotation_authority_identity()
                .expect("rotatable")
                .digest,
            hex32("db3bf05b04063ea2e9626e9ea63def4920f205cb36c43d64a046fd04e87b8304")
        );

        let successor = qualify_root(transition_successor()).expect("successor qualifies");
        assert_eq!(
            successor.root_identity().digest,
            hex32("c970bfc0957efc00946d08a957d7617b85815f47d98fcc98aa43f6f6a6ced963")
        );
        assert_eq!(
            successor.source_descriptor_identity().digest,
            hex32("3f867aab08093b08f8f4088a540e8cffa85dc25ab7c7cfb34ac65c8bc071a3fe")
        );
        assert_eq!(
            successor
                .rotation_authority_identity()
                .expect("rotatable")
                .digest,
            hex32("ee35844f067306e66ec8fcb40442b3564e9f7f5bd824a3e2a71265ea318aebea")
        );
    }

    #[test]
    fn policy_scope_order_is_semantically_irrelevant() {
        let root = golden_root();
        let expected = root_identity(&root).expect("valid").digest;
        let mut reordered = root;
        reordered.authorized_policy_scopes.reverse();
        assert_eq!(root_identity(&reordered).expect("valid").digest, expected);
    }

    #[test]
    fn duplicate_and_ambiguous_scopes_fail_closed() {
        let mut duplicate = golden_root();
        duplicate
            .authorized_policy_scopes
            .push(duplicate.authorized_policy_scopes[0].clone());
        assert!(matches!(
            validate_root(&duplicate),
            Err(RootError::DuplicatePolicyScopeKey | RootError::DuplicatePolicyScope)
        ));

        let mut ambiguous = golden_root();
        let mut alternate = ambiguous.authorized_policy_scopes[0].clone();
        alternate.provider_authority_institution_id = "institution:alternate-clerk".to_string();
        ambiguous.authorized_policy_scopes.push(alternate);
        assert_eq!(
            validate_root(&ambiguous),
            Err(RootError::DuplicatePolicyScopeKey)
        );
    }

    #[test]
    fn self_authorizing_policy_profile_fails_closed() {
        let mut root = golden_root();
        root.authorized_policy_scopes[0].policy_identity_profile = IDENTITY_PROFILE.to_string();
        assert_eq!(
            validate_root(&root),
            Err(RootError::SelfAuthorizingPolicyProfile)
        );
    }

    #[test]
    fn generation_predecessor_and_validity_rules_match_v1() {
        let mut genesis_with_predecessor = golden_root();
        genesis_with_predecessor.predecessor_root_digest = Some(digest(0x77));
        assert_eq!(
            validate_root(&genesis_with_predecessor),
            Err(RootError::InvalidGenerationPredecessorRelation)
        );

        let mut successor_without_predecessor = golden_root();
        successor_without_predecessor.generation = 1;
        assert_eq!(
            validate_root(&successor_without_predecessor),
            Err(RootError::InvalidGenerationPredecessorRelation)
        );

        let mut expired = golden_root();
        expired.expires_at_ms = Some(expired.valid_from_ms);
        assert_eq!(
            validate_root(&expired),
            Err(RootError::InvalidValidityInterval)
        );
    }

    #[test]
    fn zero_digests_fail_closed() {
        let mut source = golden_root();
        source.root_source_anchor_digest = [0; 32];
        assert_eq!(validate_root(&source), Err(RootError::ZeroDigest));

        let mut rotation = golden_root();
        rotation.rotation_authority_anchor_digest = Some([0; 32]);
        assert_eq!(validate_root(&rotation), Err(RootError::ZeroDigest));

        let mut rulebook = golden_root();
        rulebook.constitutional_rulebook.digest = [0; 32];
        assert_eq!(validate_root(&rulebook), Err(RootError::ZeroDigest));
    }

    #[test]
    fn immutable_and_rotatable_shapes_are_exact() {
        let mut immutable_with_profile = golden_root();
        immutable_with_profile.rotation_mode = "immutable".to_string();
        assert_eq!(
            validate_root(&immutable_with_profile),
            Err(RootError::InvalidRotationShape)
        );

        let mut immutable = golden_root();
        immutable.rotation_mode = "immutable".to_string();
        immutable.rotation_profile = None;
        immutable.rotation_authority_anchor_digest = None;
        let qualified = qualify_root(immutable).expect("exact immutable shape qualifies");
        assert!(qualified.rotation_authority_identity().is_none());

        let mut missing_profile = golden_root();
        missing_profile.rotation_profile = None;
        assert_eq!(
            validate_root(&missing_profile),
            Err(RootError::InvalidRotationShape)
        );
    }

    #[test]
    fn source_and_rotation_identities_are_independent() {
        let root = golden_root();
        let source = source_descriptor_identity(&root).expect("valid").digest;
        let rotation = rotation_authority_identity(&root)
            .expect("valid")
            .expect("rotatable")
            .digest;

        let mut source_changed = root.clone();
        source_changed.root_source_anchor_digest = digest(0x99);
        assert_ne!(
            source_descriptor_identity(&source_changed)
                .expect("valid")
                .digest,
            source
        );
        assert_eq!(
            rotation_authority_identity(&source_changed)
                .expect("valid")
                .expect("rotatable")
                .digest,
            rotation
        );

        let mut rotation_changed = root;
        rotation_changed.rotation_authority_anchor_digest = Some(digest(0x88));
        assert_eq!(
            source_descriptor_identity(&rotation_changed)
                .expect("valid")
                .digest,
            source
        );
        assert_ne!(
            rotation_authority_identity(&rotation_changed)
                .expect("valid")
                .expect("rotatable")
                .digest,
            rotation
        );
    }

    #[test]
    fn text_constraints_match_v1() {
        let mut control = golden_root();
        control.institution_id = "institution:city\nsmuggled".to_string();
        assert_eq!(validate_root(&control), Err(RootError::InvalidText));

        let mut leading = golden_root();
        leading.institution_id = " institution:city".to_string();
        assert_eq!(validate_root(&leading), Err(RootError::InvalidText));

        let mut trailing = golden_root();
        trailing.authorized_policy_scopes[0]
            .policy_registry_namespace
            .push(' ');
        assert_eq!(validate_root(&trailing), Err(RootError::InvalidText));
    }
}
