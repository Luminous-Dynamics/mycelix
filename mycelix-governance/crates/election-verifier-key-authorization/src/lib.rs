//! ELECT-017 immutable verifier-key authorization roots for Mycelix public elections.
//!
//! This crate owns election authorization semantics only. It freezes which verifier releases,
//! implementation lineages, builder-control domains, and Xenia signer-key identities may later
//! authenticate ELECT-016 receipt envelopes. It performs no signature verification and defines
//! no key-rotation authority; those are deliberately separate later tranches.

use election_integrity_types::{Digest32, PUBLIC_ELECTION_PROFILE_ID};
use sha2::{Digest as ShaDigest, Sha256};
use std::collections::HashSet;

pub const VERIFIER_KEY_AUTHORIZATION_PROFILE_ID: &str =
    "mycelix-public-election-verifier-key-authorization-v1";
pub const VERIFIER_KEY_AUTHORIZATION_HASH_ID: &str = "sha-256";
pub const XENIA_ED25519_AUTHENTICATION_SUITE_ID: u16 = 1;
pub const XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID: u16 = 2;
pub const XENIA_AUTHENTICATION_SUITE_REGISTRY_V1_SHA256: Digest32 = [
    0x02, 0x55, 0xc2, 0xb3, 0x07, 0x0e, 0x57, 0x9d, 0x52, 0xe4, 0x1a, 0xb6, 0xa9, 0xd7, 0x67, 0xd1,
    0x70, 0x0b, 0x61, 0xfd, 0x68, 0x78, 0x7b, 0xba, 0xe7, 0x8b, 0xd4, 0x1a, 0xa8, 0x68, 0x94, 0x5f,
];

pub const MIN_AUTHORIZED_VERIFIERS: u16 = 3;
pub const MIN_DISTINCT_IMPLEMENTATION_LINEAGES: u16 = 3;
pub const MIN_DISTINCT_BUILDER_CONTROL_DOMAINS: u16 = 2;
pub const MAX_AUTHORIZED_VERIFIERS: usize = 64;
pub const MAX_CANONICAL_STRING_BYTES: usize = 256;

const POLICY_DOMAIN: &[u8] = b"MYCELIX:PUBLIC-ELECTION:VERIFIER-KEY-AUTHORIZATION-POLICY:V1\0";
const ROOT_DOMAIN: &[u8] = b"MYCELIX:PUBLIC-ELECTION:VERIFIER-KEY-AUTHORIZATION-ROOT:V1\0";
const REQUIREMENTS_DOMAIN: &[u8] =
    b"MYCELIX:PUBLIC-ELECTION:CERTIFICATION-REQUIREMENTS:VERIFIER-KEY-AUTHORIZATION:V1\0";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifierKeyAuthorizationPolicyV1 {
    pub public_election_profile_id: String,
    pub verifier_key_authorization_profile_id: String,
    pub authentication_suite_registry_digest: Digest32,
    pub minimum_total_verifiers: u16,
    pub minimum_distinct_implementation_lineages: u16,
    pub minimum_distinct_builder_control_domains: u16,
    pub require_ed25519: bool,
    pub require_ml_dsa_65: bool,
}

impl Default for VerifierKeyAuthorizationPolicyV1 {
    fn default() -> Self {
        Self {
            public_election_profile_id: PUBLIC_ELECTION_PROFILE_ID.to_owned(),
            verifier_key_authorization_profile_id: VERIFIER_KEY_AUTHORIZATION_PROFILE_ID.to_owned(),
            authentication_suite_registry_digest: XENIA_AUTHENTICATION_SUITE_REGISTRY_V1_SHA256,
            minimum_total_verifiers: MIN_AUTHORIZED_VERIFIERS,
            minimum_distinct_implementation_lineages: MIN_DISTINCT_IMPLEMENTATION_LINEAGES,
            minimum_distinct_builder_control_domains: MIN_DISTINCT_BUILDER_CONTROL_DOMAINS,
            require_ed25519: true,
            require_ml_dsa_65: true,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VerifierKeyAuthorizationPolicyViolation {
    WrongPublicElectionProfile,
    WrongAuthorizationProfile,
    WrongXeniaAuthenticationSuiteRegistry,
    TooFewVerifiers,
    TooManyRequiredVerifiers,
    TooFewImplementationLineages,
    TooFewBuilderControlDomains,
    ImplementationLineagesExceedTotalVerifiers,
    BuilderControlDomainsExceedTotalVerifiers,
    Ed25519NotRequired,
    MlDsa65NotRequired,
    CanonicalStringTooLong,
}

fn sha256(bytes: &[u8]) -> Digest32 {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    hasher.finalize().into()
}

fn append_len_prefixed_utf8(bytes: &mut Vec<u8>, value: &str) -> Result<(), ()> {
    let raw = value.as_bytes();
    if raw.len() > MAX_CANONICAL_STRING_BYTES {
        return Err(());
    }
    let length = u32::try_from(raw.len()).map_err(|_| ())?;
    bytes.extend_from_slice(&length.to_be_bytes());
    bytes.extend_from_slice(raw);
    Ok(())
}

pub fn validate_verifier_key_authorization_policy(
    policy: &VerifierKeyAuthorizationPolicyV1,
) -> Result<(), VerifierKeyAuthorizationPolicyViolation> {
    if policy.public_election_profile_id != PUBLIC_ELECTION_PROFILE_ID {
        return Err(VerifierKeyAuthorizationPolicyViolation::WrongPublicElectionProfile);
    }
    if policy.verifier_key_authorization_profile_id != VERIFIER_KEY_AUTHORIZATION_PROFILE_ID {
        return Err(VerifierKeyAuthorizationPolicyViolation::WrongAuthorizationProfile);
    }
    if policy.public_election_profile_id.len() > MAX_CANONICAL_STRING_BYTES
        || policy.verifier_key_authorization_profile_id.len() > MAX_CANONICAL_STRING_BYTES
    {
        return Err(VerifierKeyAuthorizationPolicyViolation::CanonicalStringTooLong);
    }
    if policy.authentication_suite_registry_digest != XENIA_AUTHENTICATION_SUITE_REGISTRY_V1_SHA256
    {
        return Err(VerifierKeyAuthorizationPolicyViolation::WrongXeniaAuthenticationSuiteRegistry);
    }
    if policy.minimum_total_verifiers < MIN_AUTHORIZED_VERIFIERS {
        return Err(VerifierKeyAuthorizationPolicyViolation::TooFewVerifiers);
    }
    if usize::from(policy.minimum_total_verifiers) > MAX_AUTHORIZED_VERIFIERS {
        return Err(VerifierKeyAuthorizationPolicyViolation::TooManyRequiredVerifiers);
    }
    if policy.minimum_distinct_implementation_lineages < MIN_DISTINCT_IMPLEMENTATION_LINEAGES {
        return Err(VerifierKeyAuthorizationPolicyViolation::TooFewImplementationLineages);
    }
    if policy.minimum_distinct_builder_control_domains < MIN_DISTINCT_BUILDER_CONTROL_DOMAINS {
        return Err(VerifierKeyAuthorizationPolicyViolation::TooFewBuilderControlDomains);
    }
    if policy.minimum_distinct_implementation_lineages > policy.minimum_total_verifiers {
        return Err(
            VerifierKeyAuthorizationPolicyViolation::ImplementationLineagesExceedTotalVerifiers,
        );
    }
    if policy.minimum_distinct_builder_control_domains > policy.minimum_total_verifiers {
        return Err(
            VerifierKeyAuthorizationPolicyViolation::BuilderControlDomainsExceedTotalVerifiers,
        );
    }
    if !policy.require_ed25519 {
        return Err(VerifierKeyAuthorizationPolicyViolation::Ed25519NotRequired);
    }
    if !policy.require_ml_dsa_65 {
        return Err(VerifierKeyAuthorizationPolicyViolation::MlDsa65NotRequired);
    }
    Ok(())
}

pub fn canonical_verifier_key_authorization_policy_bytes(
    policy: &VerifierKeyAuthorizationPolicyV1,
) -> Result<Vec<u8>, VerifierKeyAuthorizationPolicyViolation> {
    validate_verifier_key_authorization_policy(policy)?;
    let mut bytes = Vec::with_capacity(256);
    bytes.extend_from_slice(POLICY_DOMAIN);
    append_len_prefixed_utf8(&mut bytes, &policy.public_election_profile_id)
        .map_err(|_| VerifierKeyAuthorizationPolicyViolation::CanonicalStringTooLong)?;
    append_len_prefixed_utf8(&mut bytes, &policy.verifier_key_authorization_profile_id)
        .map_err(|_| VerifierKeyAuthorizationPolicyViolation::CanonicalStringTooLong)?;
    bytes.extend_from_slice(&policy.authentication_suite_registry_digest);
    bytes.extend_from_slice(&policy.minimum_total_verifiers.to_be_bytes());
    bytes.extend_from_slice(
        &policy
            .minimum_distinct_implementation_lineages
            .to_be_bytes(),
    );
    bytes.extend_from_slice(
        &policy
            .minimum_distinct_builder_control_domains
            .to_be_bytes(),
    );
    bytes.push(u8::from(policy.require_ed25519));
    bytes.push(u8::from(policy.require_ml_dsa_65));
    Ok(bytes)
}

pub fn verifier_key_authorization_policy_digest(
    policy: &VerifierKeyAuthorizationPolicyV1,
) -> Result<Digest32, VerifierKeyAuthorizationPolicyViolation> {
    Ok(sha256(&canonical_verifier_key_authorization_policy_bytes(
        policy,
    )?))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AuthorizedVerifierKeysetV1 {
    pub verifier_release_digest: Digest32,
    pub verifier_lineage_digest: Digest32,
    pub builder_control_domain_digest: Digest32,
    pub ed25519_signer_key_id: Digest32,
    pub ml_dsa_65_signer_key_id: Digest32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifierKeyAuthorizationRootV1 {
    pub verifier_key_authorization_profile_id: String,
    pub election_definition_digest: Digest32,
    pub jurisdiction_snapshot_digest: Digest32,
    pub authorization_policy_digest: Digest32,
    pub authorized_verifiers: Vec<AuthorizedVerifierKeysetV1>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VerifierKeyAuthorizationRootViolation {
    Policy(VerifierKeyAuthorizationPolicyViolation),
    WrongAuthorizationProfile,
    CanonicalStringTooLong,
    ZeroElectionDefinitionDigest,
    ZeroJurisdictionSnapshotDigest,
    ZeroAuthorizationPolicyDigest,
    AuthorizationPolicyDigestMismatch,
    TooFewAuthorizedVerifiers,
    TooManyAuthorizedVerifiers,
    ZeroVerifierReleaseDigest,
    ZeroVerifierLineageDigest,
    ZeroBuilderControlDomainDigest,
    ZeroEd25519SignerKeyId,
    ZeroMlDsa65SignerKeyId,
    DuplicateVerifierRelease,
    ReusedEd25519SignerKeyId,
    ReusedMlDsa65SignerKeyId,
    TooFewDistinctImplementationLineages,
    TooFewDistinctBuilderControlDomains,
}

pub fn validate_verifier_key_authorization_root(
    root: &VerifierKeyAuthorizationRootV1,
    policy: &VerifierKeyAuthorizationPolicyV1,
) -> Result<(), VerifierKeyAuthorizationRootViolation> {
    validate_verifier_key_authorization_policy(policy)
        .map_err(VerifierKeyAuthorizationRootViolation::Policy)?;

    if root.verifier_key_authorization_profile_id != VERIFIER_KEY_AUTHORIZATION_PROFILE_ID {
        return Err(VerifierKeyAuthorizationRootViolation::WrongAuthorizationProfile);
    }
    if root.verifier_key_authorization_profile_id.len() > MAX_CANONICAL_STRING_BYTES {
        return Err(VerifierKeyAuthorizationRootViolation::CanonicalStringTooLong);
    }

    let zero = [0_u8; 32];
    if root.election_definition_digest == zero {
        return Err(VerifierKeyAuthorizationRootViolation::ZeroElectionDefinitionDigest);
    }
    if root.jurisdiction_snapshot_digest == zero {
        return Err(VerifierKeyAuthorizationRootViolation::ZeroJurisdictionSnapshotDigest);
    }
    if root.authorization_policy_digest == zero {
        return Err(VerifierKeyAuthorizationRootViolation::ZeroAuthorizationPolicyDigest);
    }

    let expected_policy_digest = verifier_key_authorization_policy_digest(policy)
        .map_err(VerifierKeyAuthorizationRootViolation::Policy)?;
    if root.authorization_policy_digest != expected_policy_digest {
        return Err(VerifierKeyAuthorizationRootViolation::AuthorizationPolicyDigestMismatch);
    }

    if root.authorized_verifiers.len() < usize::from(policy.minimum_total_verifiers) {
        return Err(VerifierKeyAuthorizationRootViolation::TooFewAuthorizedVerifiers);
    }
    if root.authorized_verifiers.len() > MAX_AUTHORIZED_VERIFIERS {
        return Err(VerifierKeyAuthorizationRootViolation::TooManyAuthorizedVerifiers);
    }

    let mut releases = HashSet::with_capacity(root.authorized_verifiers.len());
    let mut lineages = HashSet::with_capacity(root.authorized_verifiers.len());
    let mut builders = HashSet::with_capacity(root.authorized_verifiers.len());
    let mut ed25519_keys = HashSet::with_capacity(root.authorized_verifiers.len());
    let mut ml_dsa_65_keys = HashSet::with_capacity(root.authorized_verifiers.len());

    for verifier in &root.authorized_verifiers {
        if verifier.verifier_release_digest == zero {
            return Err(VerifierKeyAuthorizationRootViolation::ZeroVerifierReleaseDigest);
        }
        if verifier.verifier_lineage_digest == zero {
            return Err(VerifierKeyAuthorizationRootViolation::ZeroVerifierLineageDigest);
        }
        if verifier.builder_control_domain_digest == zero {
            return Err(VerifierKeyAuthorizationRootViolation::ZeroBuilderControlDomainDigest);
        }
        if verifier.ed25519_signer_key_id == zero {
            return Err(VerifierKeyAuthorizationRootViolation::ZeroEd25519SignerKeyId);
        }
        if verifier.ml_dsa_65_signer_key_id == zero {
            return Err(VerifierKeyAuthorizationRootViolation::ZeroMlDsa65SignerKeyId);
        }

        if !releases.insert(verifier.verifier_release_digest) {
            return Err(VerifierKeyAuthorizationRootViolation::DuplicateVerifierRelease);
        }
        if !ed25519_keys.insert(verifier.ed25519_signer_key_id) {
            return Err(VerifierKeyAuthorizationRootViolation::ReusedEd25519SignerKeyId);
        }
        if !ml_dsa_65_keys.insert(verifier.ml_dsa_65_signer_key_id) {
            return Err(VerifierKeyAuthorizationRootViolation::ReusedMlDsa65SignerKeyId);
        }

        lineages.insert(verifier.verifier_lineage_digest);
        builders.insert(verifier.builder_control_domain_digest);
    }

    if lineages.len() < usize::from(policy.minimum_distinct_implementation_lineages) {
        return Err(VerifierKeyAuthorizationRootViolation::TooFewDistinctImplementationLineages);
    }
    if builders.len() < usize::from(policy.minimum_distinct_builder_control_domains) {
        return Err(VerifierKeyAuthorizationRootViolation::TooFewDistinctBuilderControlDomains);
    }

    Ok(())
}

pub fn canonical_verifier_key_authorization_root_bytes(
    root: &VerifierKeyAuthorizationRootV1,
    policy: &VerifierKeyAuthorizationPolicyV1,
) -> Result<Vec<u8>, VerifierKeyAuthorizationRootViolation> {
    validate_verifier_key_authorization_root(root, policy)?;

    let mut ordered = root.authorized_verifiers.clone();
    ordered.sort_by_key(|verifier| {
        (
            verifier.verifier_release_digest,
            verifier.verifier_lineage_digest,
            verifier.builder_control_domain_digest,
            verifier.ed25519_signer_key_id,
            verifier.ml_dsa_65_signer_key_id,
        )
    });

    let count = u16::try_from(ordered.len())
        .map_err(|_| VerifierKeyAuthorizationRootViolation::TooManyAuthorizedVerifiers)?;

    let mut bytes = Vec::with_capacity(768);
    bytes.extend_from_slice(ROOT_DOMAIN);
    append_len_prefixed_utf8(&mut bytes, &root.verifier_key_authorization_profile_id)
        .map_err(|_| VerifierKeyAuthorizationRootViolation::CanonicalStringTooLong)?;
    bytes.extend_from_slice(&root.election_definition_digest);
    bytes.extend_from_slice(&root.jurisdiction_snapshot_digest);
    bytes.extend_from_slice(&root.authorization_policy_digest);
    bytes.extend_from_slice(&count.to_be_bytes());

    for verifier in ordered {
        bytes.extend_from_slice(&verifier.verifier_release_digest);
        bytes.extend_from_slice(&verifier.verifier_lineage_digest);
        bytes.extend_from_slice(&verifier.builder_control_domain_digest);
        bytes.extend_from_slice(&verifier.ed25519_signer_key_id);
        bytes.extend_from_slice(&verifier.ml_dsa_65_signer_key_id);
    }
    Ok(bytes)
}

pub fn verifier_key_authorization_root_digest(
    root: &VerifierKeyAuthorizationRootV1,
    policy: &VerifierKeyAuthorizationPolicyV1,
) -> Result<Digest32, VerifierKeyAuthorizationRootViolation> {
    Ok(sha256(&canonical_verifier_key_authorization_root_bytes(
        root, policy,
    )?))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VerifierKeyAuthorizationRequirementsBindingV1 {
    pub previous_certification_requirements_digest: Digest32,
    pub verifier_key_authorization_root_digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VerifierKeyAuthorizationRequirementsViolation {
    ZeroPreviousCertificationRequirementsDigest,
    ZeroVerifierKeyAuthorizationRootDigest,
}

pub fn verifier_key_authorization_requirements_digest(
    binding: &VerifierKeyAuthorizationRequirementsBindingV1,
) -> Result<Digest32, VerifierKeyAuthorizationRequirementsViolation> {
    let zero = [0_u8; 32];
    if binding.previous_certification_requirements_digest == zero {
        return Err(
            VerifierKeyAuthorizationRequirementsViolation::ZeroPreviousCertificationRequirementsDigest,
        );
    }
    if binding.verifier_key_authorization_root_digest == zero {
        return Err(
            VerifierKeyAuthorizationRequirementsViolation::ZeroVerifierKeyAuthorizationRootDigest,
        );
    }

    let mut bytes = Vec::with_capacity(REQUIREMENTS_DOMAIN.len() + 64);
    bytes.extend_from_slice(REQUIREMENTS_DOMAIN);
    bytes.extend_from_slice(&binding.previous_certification_requirements_digest);
    bytes.extend_from_slice(&binding.verifier_key_authorization_root_digest);
    Ok(sha256(&bytes))
}

#[cfg(test)]
mod tests {
    use super::*;

    const GOLDEN_POLICY_DIGEST: Digest32 = [
        0x91, 0x53, 0x6d, 0xd7, 0x64, 0x59, 0xb2, 0xbe, 0x14, 0x88, 0x22, 0x39, 0xb2, 0x19, 0x25,
        0x10, 0xef, 0x94, 0x0e, 0x94, 0xc4, 0x73, 0x71, 0xbe, 0xb3, 0xae, 0x22, 0xd9, 0x1a, 0x44,
        0xfd, 0x98,
    ];
    const GOLDEN_ROOT_DIGEST: Digest32 = [
        0xb0, 0x91, 0xac, 0xf3, 0xf9, 0x1c, 0xa7, 0x8b, 0xaf, 0x02, 0xe1, 0xdf, 0x52, 0x96, 0x5d,
        0x73, 0xeb, 0x86, 0x19, 0xb4, 0x9a, 0x0f, 0x0a, 0x8c, 0x60, 0xb8, 0x79, 0xe4, 0xfe, 0x70,
        0xaf, 0xb4,
    ];
    const GOLDEN_REQUIREMENTS_DIGEST: Digest32 = [
        0x95, 0x7e, 0x05, 0x4b, 0xf7, 0x2a, 0xef, 0xf6, 0x89, 0x6f, 0x75, 0x6e, 0xc3, 0x5d, 0x55,
        0x62, 0x09, 0x61, 0x62, 0xf0, 0xe6, 0x4c, 0xa0, 0x86, 0x5c, 0xbd, 0x27, 0xf8, 0xad, 0xab,
        0xa1, 0xa1,
    ];

    fn digest(byte: u8) -> Digest32 {
        [byte; 32]
    }

    fn keyset(
        release: u8,
        lineage: u8,
        builder: u8,
        ed25519: u8,
        ml_dsa_65: u8,
    ) -> AuthorizedVerifierKeysetV1 {
        AuthorizedVerifierKeysetV1 {
            verifier_release_digest: digest(release),
            verifier_lineage_digest: digest(lineage),
            builder_control_domain_digest: digest(builder),
            ed25519_signer_key_id: digest(ed25519),
            ml_dsa_65_signer_key_id: digest(ml_dsa_65),
        }
    }

    fn policy() -> VerifierKeyAuthorizationPolicyV1 {
        VerifierKeyAuthorizationPolicyV1::default()
    }

    fn root() -> VerifierKeyAuthorizationRootV1 {
        VerifierKeyAuthorizationRootV1 {
            verifier_key_authorization_profile_id: VERIFIER_KEY_AUTHORIZATION_PROFILE_ID.to_owned(),
            election_definition_digest: digest(0x11),
            jurisdiction_snapshot_digest: digest(0x12),
            authorization_policy_digest: GOLDEN_POLICY_DIGEST,
            authorized_verifiers: vec![
                keyset(0x21, 0x31, 0x41, 0x51, 0x61),
                keyset(0x22, 0x32, 0x42, 0x52, 0x62),
                keyset(0x23, 0x33, 0x41, 0x53, 0x63),
            ],
        }
    }

    #[test]
    fn policy_pins_xenia_v1_registry_and_both_authentication_suites() {
        let policy = policy();
        assert_eq!(
            verifier_key_authorization_policy_digest(&policy).unwrap(),
            GOLDEN_POLICY_DIGEST
        );
        assert!(policy.require_ed25519);
        assert!(policy.require_ml_dsa_65);
        assert_eq!(XENIA_ED25519_AUTHENTICATION_SUITE_ID, 1);
        assert_eq!(XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID, 2);
    }

    #[test]
    fn wrong_xenia_registry_fails_closed() {
        let mut policy = policy();
        policy.authentication_suite_registry_digest = digest(0x99);
        assert_eq!(
            validate_verifier_key_authorization_policy(&policy),
            Err(VerifierKeyAuthorizationPolicyViolation::WrongXeniaAuthenticationSuiteRegistry)
        );
    }

    #[test]
    fn weakening_required_hybrid_authentication_fails_closed() {
        let mut policy = policy();
        policy.require_ml_dsa_65 = false;
        assert_eq!(
            validate_verifier_key_authorization_policy(&policy),
            Err(VerifierKeyAuthorizationPolicyViolation::MlDsa65NotRequired)
        );
    }

    #[test]
    fn root_digest_is_order_invariant_and_matches_golden_vector() {
        let policy = policy();
        let root = root();
        assert_eq!(
            verifier_key_authorization_root_digest(&root, &policy).unwrap(),
            GOLDEN_ROOT_DIGEST
        );

        let mut reordered = root.clone();
        reordered.authorized_verifiers.reverse();
        assert_eq!(
            verifier_key_authorization_root_digest(&reordered, &policy).unwrap(),
            GOLDEN_ROOT_DIGEST
        );
    }

    #[test]
    fn duplicate_release_and_reused_signer_ids_fail_closed() {
        let policy = policy();

        let mut duplicate_release = root();
        duplicate_release.authorized_verifiers[1].verifier_release_digest =
            duplicate_release.authorized_verifiers[0].verifier_release_digest;
        assert_eq!(
            validate_verifier_key_authorization_root(&duplicate_release, &policy),
            Err(VerifierKeyAuthorizationRootViolation::DuplicateVerifierRelease)
        );

        let mut reused_key = root();
        reused_key.authorized_verifiers[1].ed25519_signer_key_id =
            reused_key.authorized_verifiers[0].ed25519_signer_key_id;
        assert_eq!(
            validate_verifier_key_authorization_root(&reused_key, &policy),
            Err(VerifierKeyAuthorizationRootViolation::ReusedEd25519SignerKeyId)
        );
    }

    #[test]
    fn raw_key_count_cannot_fake_independence_quorum() {
        let policy = policy();

        let mut same_lineage = root();
        for verifier in &mut same_lineage.authorized_verifiers {
            verifier.verifier_lineage_digest = digest(0x31);
        }
        assert_eq!(
            validate_verifier_key_authorization_root(&same_lineage, &policy),
            Err(VerifierKeyAuthorizationRootViolation::TooFewDistinctImplementationLineages)
        );

        let mut same_builder = root();
        for verifier in &mut same_builder.authorized_verifiers {
            verifier.builder_control_domain_digest = digest(0x41);
        }
        assert_eq!(
            validate_verifier_key_authorization_root(&same_builder, &policy),
            Err(VerifierKeyAuthorizationRootViolation::TooFewDistinctBuilderControlDomains)
        );
    }

    #[test]
    fn policy_digest_is_recomputed_not_self_asserted() {
        let policy = policy();
        let mut root = root();
        root.authorization_policy_digest = digest(0x44);
        assert_eq!(
            validate_verifier_key_authorization_root(&root, &policy),
            Err(VerifierKeyAuthorizationRootViolation::AuthorizationPolicyDigestMismatch)
        );
    }

    #[test]
    fn zero_identity_fields_fail_closed() {
        let policy = policy();
        let mut root = root();
        root.authorized_verifiers[0].ml_dsa_65_signer_key_id = [0_u8; 32];
        assert_eq!(
            validate_verifier_key_authorization_root(&root, &policy),
            Err(VerifierKeyAuthorizationRootViolation::ZeroMlDsa65SignerKeyId)
        );
    }

    #[test]
    fn requirements_binding_preserves_preexisting_certification_requirements() {
        let policy = policy();
        let root_digest = verifier_key_authorization_root_digest(&root(), &policy).unwrap();
        let binding = VerifierKeyAuthorizationRequirementsBindingV1 {
            previous_certification_requirements_digest: digest(0x70),
            verifier_key_authorization_root_digest: root_digest,
        };
        assert_eq!(
            verifier_key_authorization_requirements_digest(&binding).unwrap(),
            GOLDEN_REQUIREMENTS_DIGEST
        );

        let mut changed_previous = binding;
        changed_previous.previous_certification_requirements_digest = digest(0x71);
        assert_ne!(
            verifier_key_authorization_requirements_digest(&changed_previous).unwrap(),
            GOLDEN_REQUIREMENTS_DIGEST
        );
    }
}
