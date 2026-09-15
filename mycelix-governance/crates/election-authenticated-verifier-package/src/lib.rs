//! ELECT-017C authenticated-verifier offline package extension for Mycelix public elections.
//!
//! ELECT-011's qualified V1 manifest remains unchanged. This crate composes that legacy manifest
//! with exactly two public extension artifacts: the canonical ELECT-017B verifier public-key bundle
//! and the exact qualified Xenia authentication-profile bytes. It hashes the actual carried bytes,
//! derives a separate extension root, and derives a V2 package identity without reinterpreting the
//! legacy V1 package root.
//!
//! This crate does not claim that ELECT-011's legacy package-root field has been recomputed from
//! archive bytes. Legacy package-content integrity remains the responsibility of the existing
//! PackageIntegrity verification stage.

use election_integrity_types::Digest32;
use election_verifier_contract::{
    ArtifactDisclosureClass, ElectionEvidencePackageManifestV1, EvidencePackageViolation,
    MAX_CANONICAL_PATH_BYTES, validate_evidence_package_manifest,
};
use election_verifier_key_authorization::{
    VerifierKeyAuthorizationPolicyV1, VerifierKeyAuthorizationRootV1,
    VerifierKeyAuthorizationRootViolation, verifier_key_authorization_root_digest,
};
use election_verifier_public_key_binding::{
    VerifierAuthenticationPublicKeyBundleV1, VerifierPublicKeyBundleViolation,
    VerifierPublicKeyRequirementsBindingV1, VerifierPublicKeyRequirementsViolation,
    XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
    canonical_verifier_authentication_public_key_bundle_bytes,
    verifier_authentication_public_key_bundle_digest, verifier_public_key_requirements_digest,
};
use sha2::{Digest as ShaDigest, Sha256};
use std::collections::HashSet;

pub const AUTHENTICATED_VERIFIER_PACKAGE_PROFILE_ID: &str =
    "mycelix-public-election-evidence-package-authenticated-verifier-v2";
pub const VERIFIER_PUBLIC_KEY_BUNDLE_PATH: &str =
    "authentication/verifier-public-key-bundle-v1.bin";
pub const XENIA_AUTHENTICATION_PROFILE_PATH: &str =
    "authentication/xenia-authentication-profile-v1.txt";
pub const XENIA_AUTHENTICATION_PROFILE_V1_BYTES: usize = 1120;

/// ELECT-017B admits at most 128 public-key evidence records. Even if every record used the
/// larger ML-DSA-65 key encoding, its canonical bundle is below 264 KiB. A 512 KiB package
/// preflight ceiling leaves substantial framing headroom while preventing unbounded allocation.
pub const MAX_VERIFIER_PUBLIC_KEY_BUNDLE_BYTES_V2: usize = 512 * 1024;
pub const REQUIRED_EXTENSION_ARTIFACTS_V2: usize = 2;
pub const MAX_CANONICAL_PROFILE_ID_BYTES: usize = 256;

const EXTENSION_ROOT_DOMAIN: &[u8] =
    b"MYCELIX:PUBLIC-ELECTION:AUTHENTICATED-VERIFIER-EXTENSION-ROOT:V1\0";
const PACKAGE_IDENTITY_DOMAIN: &[u8] =
    b"MYCELIX:PUBLIC-ELECTION:AUTHENTICATED-VERIFIER-EVIDENCE-PACKAGE:V2\0";

fn sha256(bytes: &[u8]) -> Digest32 {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    hasher.finalize().into()
}

fn append_len_prefixed_utf8(bytes: &mut Vec<u8>, value: &str) -> Result<(), ()> {
    let raw = value.as_bytes();
    if raw.len() > MAX_CANONICAL_PROFILE_ID_BYTES {
        return Err(());
    }
    let length = u32::try_from(raw.len()).map_err(|_| ())?;
    bytes.extend_from_slice(&length.to_be_bytes());
    bytes.extend_from_slice(raw);
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum AuthenticatedVerifierExtensionRoleV1 {
    VerifierPublicKeyBundle,
    XeniaAuthenticationProfile,
}

impl AuthenticatedVerifierExtensionRoleV1 {
    fn tag(self) -> u8 {
        match self {
            Self::VerifierPublicKeyBundle => 1,
            Self::XeniaAuthenticationProfile => 2,
        }
    }

    fn canonical_path(self) -> &'static str {
        match self {
            Self::VerifierPublicKeyBundle => VERIFIER_PUBLIC_KEY_BUNDLE_PATH,
            Self::XeniaAuthenticationProfile => XENIA_AUTHENTICATION_PROFILE_PATH,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuthenticatedVerifierExtensionArtifactRefV1 {
    pub role: AuthenticatedVerifierExtensionRoleV1,
    pub canonical_path: String,
    pub content_digest: Digest32,
    pub byte_length: u64,
    pub disclosure: ArtifactDisclosureClass,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuthenticatedVerifierEvidencePackageManifestV2 {
    pub authenticated_verifier_package_profile_id: String,
    pub legacy_v1_manifest: ElectionEvidencePackageManifestV1,
    pub elect017_authorization_root_digest: Digest32,
    pub elect017b_requirements_digest: Digest32,
    pub xenia_authentication_profile_digest: Digest32,
    pub verifier_public_key_bundle_digest: Digest32,
    pub extensions: Vec<AuthenticatedVerifierExtensionArtifactRefV1>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AuthenticatedVerifierPackageViolation {
    LegacyManifest(EvidencePackageViolation),
    AuthorizationRoot(VerifierKeyAuthorizationRootViolation),
    PublicKeyBundle(VerifierPublicKeyBundleViolation),
    PublicKeyRequirements(VerifierPublicKeyRequirementsViolation),
    WrongPackageProfile,
    PackageProfileTooLong,
    ZeroLegacyPackageRoot,
    Elect017AuthorizationRootMismatch,
    Elect017bRequirementsDigestMismatch,
    WrongXeniaAuthenticationProfile,
    VerifierPublicKeyBundleDigestMismatch,
    WrongExtensionCount,
    DuplicateExtensionRole,
    WrongExtensionPath,
    ExtensionPathTooLong,
    ExtensionPathCollidesWithLegacyArtifact,
    ExtensionMustBePublic,
    ZeroExtensionDigest,
    EmptyExtensionArtifact,
    ExtensionArtifactLengthOverflow,
    VerifierPublicKeyBundleTooLarge,
    VerifierPublicKeyBundleRefDigestMismatch,
    VerifierPublicKeyBundleRefLengthMismatch,
    VerifierPublicKeyBundleBytesMismatch,
    XeniaProfileRefDigestMismatch,
    XeniaProfileRefLengthMismatch,
    XeniaProfileLengthMismatch,
    XeniaProfileDigestMismatch,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QualifiedAuthenticatedVerifierEvidencePackageV2 {
    legacy_v1_package_root_digest: Digest32,
    authenticated_verifier_extension_root_digest: Digest32,
    authenticated_verifier_package_identity_digest: Digest32,
    elect017_authorization_root_digest: Digest32,
    elect017b_requirements_digest: Digest32,
    verifier_public_key_bundle_digest: Digest32,
    xenia_authentication_profile_digest: Digest32,
}

impl QualifiedAuthenticatedVerifierEvidencePackageV2 {
    pub fn legacy_v1_package_root_digest(&self) -> Digest32 {
        self.legacy_v1_package_root_digest
    }

    pub fn authenticated_verifier_extension_root_digest(&self) -> Digest32 {
        self.authenticated_verifier_extension_root_digest
    }

    pub fn authenticated_verifier_package_identity_digest(&self) -> Digest32 {
        self.authenticated_verifier_package_identity_digest
    }

    pub fn elect017_authorization_root_digest(&self) -> Digest32 {
        self.elect017_authorization_root_digest
    }

    pub fn elect017b_requirements_digest(&self) -> Digest32 {
        self.elect017b_requirements_digest
    }

    pub fn verifier_public_key_bundle_digest(&self) -> Digest32 {
        self.verifier_public_key_bundle_digest
    }

    pub fn xenia_authentication_profile_digest(&self) -> Digest32 {
        self.xenia_authentication_profile_digest
    }
}

fn validate_extension_ref(
    extension: &AuthenticatedVerifierExtensionArtifactRefV1,
    legacy_manifest: &ElectionEvidencePackageManifestV1,
) -> Result<(), AuthenticatedVerifierPackageViolation> {
    if extension.canonical_path != extension.role.canonical_path() {
        return Err(AuthenticatedVerifierPackageViolation::WrongExtensionPath);
    }
    if extension.canonical_path.len() > MAX_CANONICAL_PATH_BYTES {
        return Err(AuthenticatedVerifierPackageViolation::ExtensionPathTooLong);
    }
    if legacy_manifest
        .artifacts
        .iter()
        .any(|artifact| artifact.canonical_path == extension.canonical_path)
    {
        return Err(AuthenticatedVerifierPackageViolation::ExtensionPathCollidesWithLegacyArtifact);
    }
    if extension.disclosure != ArtifactDisclosureClass::PublicVerificationEvidence {
        return Err(AuthenticatedVerifierPackageViolation::ExtensionMustBePublic);
    }
    if extension.content_digest == [0_u8; 32] {
        return Err(AuthenticatedVerifierPackageViolation::ZeroExtensionDigest);
    }
    if extension.byte_length == 0 {
        return Err(AuthenticatedVerifierPackageViolation::EmptyExtensionArtifact);
    }
    Ok(())
}

fn extension_root_digest(
    manifest: &AuthenticatedVerifierEvidencePackageManifestV2,
) -> Result<Digest32, AuthenticatedVerifierPackageViolation> {
    if manifest.extensions.len() != REQUIRED_EXTENSION_ARTIFACTS_V2 {
        return Err(AuthenticatedVerifierPackageViolation::WrongExtensionCount);
    }

    let mut ordered = manifest.extensions.clone();
    ordered.sort_by_key(|extension| extension.role);
    let mut seen = HashSet::with_capacity(ordered.len());
    for extension in &ordered {
        if !seen.insert(extension.role) {
            return Err(AuthenticatedVerifierPackageViolation::DuplicateExtensionRole);
        }
        validate_extension_ref(extension, &manifest.legacy_v1_manifest)?;
    }

    let count = u16::try_from(ordered.len())
        .map_err(|_| AuthenticatedVerifierPackageViolation::WrongExtensionCount)?;
    let mut bytes = Vec::with_capacity(320);
    bytes.extend_from_slice(EXTENSION_ROOT_DOMAIN);
    bytes.extend_from_slice(&count.to_be_bytes());
    for extension in ordered {
        let path = extension.canonical_path.as_bytes();
        let path_length = u32::try_from(path.len())
            .map_err(|_| AuthenticatedVerifierPackageViolation::ExtensionPathTooLong)?;
        bytes.push(extension.role.tag());
        bytes.extend_from_slice(&path_length.to_be_bytes());
        bytes.extend_from_slice(path);
        bytes.extend_from_slice(&extension.content_digest);
        bytes.extend_from_slice(&extension.byte_length.to_be_bytes());
    }
    Ok(sha256(&bytes))
}

fn package_identity_digest(
    manifest: &AuthenticatedVerifierEvidencePackageManifestV2,
    authenticated_verifier_extension_root_digest: Digest32,
) -> Result<Digest32, AuthenticatedVerifierPackageViolation> {
    let mut bytes = Vec::with_capacity(256);
    bytes.extend_from_slice(PACKAGE_IDENTITY_DOMAIN);
    append_len_prefixed_utf8(
        &mut bytes,
        &manifest.authenticated_verifier_package_profile_id,
    )
    .map_err(|_| AuthenticatedVerifierPackageViolation::PackageProfileTooLong)?;
    bytes.extend_from_slice(&manifest.legacy_v1_manifest.package_root_digest);
    bytes.extend_from_slice(&authenticated_verifier_extension_root_digest);
    bytes.extend_from_slice(&manifest.elect017b_requirements_digest);
    Ok(sha256(&bytes))
}

fn extension_for_role(
    manifest: &AuthenticatedVerifierEvidencePackageManifestV2,
    role: AuthenticatedVerifierExtensionRoleV1,
) -> Result<&AuthenticatedVerifierExtensionArtifactRefV1, AuthenticatedVerifierPackageViolation> {
    let mut matches = manifest
        .extensions
        .iter()
        .filter(|extension| extension.role == role);
    let first = matches
        .next()
        .ok_or(AuthenticatedVerifierPackageViolation::WrongExtensionCount)?;
    if matches.next().is_some() {
        return Err(AuthenticatedVerifierPackageViolation::DuplicateExtensionRole);
    }
    Ok(first)
}

pub fn qualify_authenticated_verifier_evidence_package_v2(
    manifest: &AuthenticatedVerifierEvidencePackageManifestV2,
    authorization_root: &VerifierKeyAuthorizationRootV1,
    authorization_policy: &VerifierKeyAuthorizationPolicyV1,
    public_key_bundle: &VerifierAuthenticationPublicKeyBundleV1,
    public_key_requirements_binding: &VerifierPublicKeyRequirementsBindingV1,
    carried_public_key_bundle_bytes: &[u8],
    carried_xenia_authentication_profile_bytes: &[u8],
) -> Result<QualifiedAuthenticatedVerifierEvidencePackageV2, AuthenticatedVerifierPackageViolation>
{
    validate_evidence_package_manifest(&manifest.legacy_v1_manifest)
        .map_err(AuthenticatedVerifierPackageViolation::LegacyManifest)?;

    if manifest.authenticated_verifier_package_profile_id
        != AUTHENTICATED_VERIFIER_PACKAGE_PROFILE_ID
    {
        return Err(AuthenticatedVerifierPackageViolation::WrongPackageProfile);
    }
    if manifest.authenticated_verifier_package_profile_id.len() > MAX_CANONICAL_PROFILE_ID_BYTES {
        return Err(AuthenticatedVerifierPackageViolation::PackageProfileTooLong);
    }
    if manifest.legacy_v1_manifest.package_root_digest == [0_u8; 32] {
        return Err(AuthenticatedVerifierPackageViolation::ZeroLegacyPackageRoot);
    }

    let expected_authorization_root_digest =
        verifier_key_authorization_root_digest(authorization_root, authorization_policy)
            .map_err(AuthenticatedVerifierPackageViolation::AuthorizationRoot)?;
    if manifest.elect017_authorization_root_digest != expected_authorization_root_digest {
        return Err(AuthenticatedVerifierPackageViolation::Elect017AuthorizationRootMismatch);
    }

    let expected_public_key_bundle_bytes =
        canonical_verifier_authentication_public_key_bundle_bytes(
            public_key_bundle,
            authorization_root,
            authorization_policy,
        )
        .map_err(AuthenticatedVerifierPackageViolation::PublicKeyBundle)?;
    if expected_public_key_bundle_bytes.len() > MAX_VERIFIER_PUBLIC_KEY_BUNDLE_BYTES_V2
        || carried_public_key_bundle_bytes.len() > MAX_VERIFIER_PUBLIC_KEY_BUNDLE_BYTES_V2
    {
        return Err(AuthenticatedVerifierPackageViolation::VerifierPublicKeyBundleTooLarge);
    }
    if carried_public_key_bundle_bytes != expected_public_key_bundle_bytes {
        return Err(AuthenticatedVerifierPackageViolation::VerifierPublicKeyBundleBytesMismatch);
    }

    let expected_public_key_bundle_digest = verifier_authentication_public_key_bundle_digest(
        public_key_bundle,
        authorization_root,
        authorization_policy,
    )
    .map_err(AuthenticatedVerifierPackageViolation::PublicKeyBundle)?;
    if manifest.verifier_public_key_bundle_digest != expected_public_key_bundle_digest {
        return Err(AuthenticatedVerifierPackageViolation::VerifierPublicKeyBundleDigestMismatch);
    }

    let expected_requirements_digest = verifier_public_key_requirements_digest(
        public_key_requirements_binding,
        public_key_bundle,
        authorization_root,
        authorization_policy,
    )
    .map_err(AuthenticatedVerifierPackageViolation::PublicKeyRequirements)?;
    if manifest.elect017b_requirements_digest != expected_requirements_digest {
        return Err(AuthenticatedVerifierPackageViolation::Elect017bRequirementsDigestMismatch);
    }

    if manifest.xenia_authentication_profile_digest != XENIA_AUTHENTICATION_PROFILE_V1_SHA256 {
        return Err(AuthenticatedVerifierPackageViolation::WrongXeniaAuthenticationProfile);
    }
    if carried_xenia_authentication_profile_bytes.len() != XENIA_AUTHENTICATION_PROFILE_V1_BYTES {
        return Err(AuthenticatedVerifierPackageViolation::XeniaProfileLengthMismatch);
    }
    if sha256(carried_xenia_authentication_profile_bytes) != XENIA_AUTHENTICATION_PROFILE_V1_SHA256
    {
        return Err(AuthenticatedVerifierPackageViolation::XeniaProfileDigestMismatch);
    }

    let extension_root = extension_root_digest(manifest)?;
    let public_key_ref = extension_for_role(
        manifest,
        AuthenticatedVerifierExtensionRoleV1::VerifierPublicKeyBundle,
    )?;
    let xenia_profile_ref = extension_for_role(
        manifest,
        AuthenticatedVerifierExtensionRoleV1::XeniaAuthenticationProfile,
    )?;

    if public_key_ref.content_digest != expected_public_key_bundle_digest {
        return Err(
            AuthenticatedVerifierPackageViolation::VerifierPublicKeyBundleRefDigestMismatch,
        );
    }
    let public_key_bundle_length = u64::try_from(carried_public_key_bundle_bytes.len())
        .map_err(|_| AuthenticatedVerifierPackageViolation::ExtensionArtifactLengthOverflow)?;
    if public_key_ref.byte_length != public_key_bundle_length {
        return Err(
            AuthenticatedVerifierPackageViolation::VerifierPublicKeyBundleRefLengthMismatch,
        );
    }

    if xenia_profile_ref.content_digest != XENIA_AUTHENTICATION_PROFILE_V1_SHA256 {
        return Err(AuthenticatedVerifierPackageViolation::XeniaProfileRefDigestMismatch);
    }
    let xenia_profile_length = u64::try_from(carried_xenia_authentication_profile_bytes.len())
        .map_err(|_| AuthenticatedVerifierPackageViolation::ExtensionArtifactLengthOverflow)?;
    if xenia_profile_ref.byte_length != xenia_profile_length {
        return Err(AuthenticatedVerifierPackageViolation::XeniaProfileRefLengthMismatch);
    }

    let package_identity = package_identity_digest(manifest, extension_root)?;
    Ok(QualifiedAuthenticatedVerifierEvidencePackageV2 {
        legacy_v1_package_root_digest: manifest.legacy_v1_manifest.package_root_digest,
        authenticated_verifier_extension_root_digest: extension_root,
        authenticated_verifier_package_identity_digest: package_identity,
        elect017_authorization_root_digest: expected_authorization_root_digest,
        elect017b_requirements_digest: expected_requirements_digest,
        verifier_public_key_bundle_digest: expected_public_key_bundle_digest,
        xenia_authentication_profile_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use election_integrity_types::PUBLIC_ELECTION_PROFILE_ID;
    use election_verifier_contract::{
        ArtifactDisclosureClass, EVIDENCE_PACKAGE_PROFILE_ID, EvidenceArtifactKind,
        EvidenceArtifactRefV1, InteroperabilityProfileRefV1,
    };
    use election_verifier_key_authorization::{
        AuthorizedVerifierKeysetV1, VERIFIER_KEY_AUTHORIZATION_PROFILE_ID,
        VerifierKeyAuthorizationRequirementsBindingV1,
        XENIA_AUTHENTICATION_SUITE_REGISTRY_V1_SHA256, XENIA_ED25519_AUTHENTICATION_SUITE_ID,
        XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID, verifier_key_authorization_policy_digest,
    };
    use election_verifier_public_key_binding::{
        VERIFIER_PUBLIC_KEY_BINDING_PROFILE_ID, VerifierAuthenticationPublicKeyEvidenceV1,
        xenia_authentication_signer_key_id_v1,
    };

    const XENIA_PROFILE_BYTES: &[u8] = b"XENIA:AuthenticationProfile:v1\nhash=sha-256\nsuite-registry-sha256=0255c2b3070e579d52e41ab6a9d767d1700b61fd68787bbae78bd41aa868945f\nsuite.1=ed25519\nsuite.1.public-key=32-bytes\nsuite.1.signature=64-bytes\nsuite.1.verification=ed25519-verify-strict\nsuite.2=ml-dsa-65-fips204\nsuite.2.public-key=1952-bytes\nsuite.2.signature=3309-bytes\nsuite.2.verification=fips204-ml-dsa-65\ncontext.component.charset=ascii-alnum-dash-underscore\ncontext.component.length=1..64-bytes\ncontext.version=u32-le-nonzero\nsigner-key-id.domain=XENIA:AuthenticationSignerKeyId:v1\nsigner-key-id.suite=u16-le-nonzero\nsigner-key-id.public-key-length=u64-le\nsigner-key-id.public-key=1..4096-bytes\nsigner-key-id.transcript=domain||suite||public-key-length||public-key\nsubject-auth.domain=XENIA:AuthenticatedSubject:v1\nsubject-auth.context-components=u64-le-length-prefixed-utf8\nsubject-auth.subject-digest=32-bytes-nonzero\nsubject-auth.suite=u16-le-nonzero\nsubject-auth.signer-key-id=32-bytes-nonzero\nsubject-auth.transcript=domain||ecosystem||application||purpose||context-version||subject-digest||suite||signer-key-id\ndetached-auth.signature=1..4096-bytes\n";

    const GOLDEN_EXTENSION_ROOT_DIGEST: Digest32 = [
        0xf3, 0x98, 0x19, 0xfd, 0x42, 0xdf, 0xc9, 0xc9, 0xda, 0x4b, 0x9c, 0x8b, 0xb9, 0x8a, 0x10,
        0xba, 0xa3, 0x7e, 0x4b, 0x84, 0x0b, 0x0c, 0x5e, 0x74, 0xc6, 0xff, 0x4c, 0x74, 0xb8, 0xb0,
        0xf2, 0xa9,
    ];
    const GOLDEN_PACKAGE_IDENTITY_DIGEST: Digest32 = [
        0x37, 0x64, 0x6d, 0x82, 0x40, 0xaf, 0xdc, 0xae, 0xca, 0xaf, 0x8f, 0x81, 0x02, 0x54, 0xa7,
        0x20, 0xb5, 0xbb, 0xaf, 0xde, 0x32, 0x9c, 0xb3, 0xef, 0x2f, 0x27, 0x4c, 0xa0, 0x20, 0x5b,
        0xd3, 0x3d,
    ];

    fn digest(byte: u8) -> Digest32 {
        [byte; 32]
    }

    fn artifact(kind: EvidenceArtifactKind, path: &str, byte: u8) -> EvidenceArtifactRefV1 {
        EvidenceArtifactRefV1 {
            kind,
            canonical_path: path.to_owned(),
            content_digest: digest(byte),
            byte_length: 10,
            disclosure: ArtifactDisclosureClass::PublicVerificationEvidence,
        }
    }

    fn legacy_manifest() -> ElectionEvidencePackageManifestV1 {
        ElectionEvidencePackageManifestV1 {
            public_election_profile_id: PUBLIC_ELECTION_PROFILE_ID.to_owned(),
            evidence_package_profile_id: EVIDENCE_PACKAGE_PROFILE_ID.to_owned(),
            election_constitution_digest: digest(1),
            package_canonicalization_profile_digest: digest(2),
            artifact_index_digest: digest(3),
            package_root_digest: digest(4),
            final_transparency_checkpoint_digest: digest(5),
            artifacts: vec![
                artifact(
                    EvidenceArtifactKind::ElectionConstitution,
                    "election/constitution.bin",
                    10,
                ),
                artifact(
                    EvidenceArtifactKind::TransparencyCheckpointChain,
                    "transparency/checkpoints.bin",
                    11,
                ),
                artifact(
                    EvidenceArtifactKind::WitnessAttestations,
                    "transparency/witnesses.bin",
                    12,
                ),
                artifact(
                    EvidenceArtifactKind::AnonymousAuthorityCensus,
                    "authority/census.bin",
                    13,
                ),
                artifact(
                    EvidenceArtifactKind::TallyEvidence,
                    "tally/evidence.bin",
                    14,
                ),
                artifact(
                    EvidenceArtifactKind::PhysicalAuditEvidence,
                    "physical/audit.bin",
                    15,
                ),
                artifact(
                    EvidenceArtifactKind::ChallengeLedger,
                    "challenges/ledger.bin",
                    16,
                ),
                artifact(
                    EvidenceArtifactKind::CertificationPolicy,
                    "certification/policy.bin",
                    17,
                ),
            ],
            interoperability_profiles: Vec::<InteroperabilityProfileRefV1>::new(),
        }
    }

    fn ed25519_keys() -> [[u8; 32]; 3] {
        [
            [
                0x21, 0x52, 0xf8, 0xd1, 0x9b, 0x79, 0x1d, 0x24, 0x45, 0x32, 0x42, 0xe1, 0x5f, 0x2e,
                0xab, 0x6c, 0xb7, 0xcf, 0xfa, 0x7b, 0x6a, 0x5e, 0xd3, 0x00, 0x97, 0x96, 0x0e, 0x06,
                0x98, 0x81, 0xdb, 0x12,
            ],
            [
                0x22, 0xfc, 0x29, 0x77, 0x92, 0xf0, 0xb6, 0xff, 0xc0, 0xbf, 0xcf, 0xdb, 0x7e, 0xdb,
                0x0c, 0x0a, 0xa1, 0x4e, 0x02, 0x5a, 0x36, 0x5e, 0xc0, 0xe3, 0x42, 0xe8, 0x6e, 0x38,
                0x29, 0xcb, 0x74, 0xb6,
            ],
            [
                0xd7, 0x59, 0x79, 0x3b, 0xbc, 0x13, 0xa2, 0x81, 0x9a, 0x82, 0x7c, 0x76, 0xad, 0xb6,
                0xfb, 0xa8, 0xa4, 0x9a, 0xee, 0x00, 0x7f, 0x49, 0xf2, 0xd0, 0x99, 0x2d, 0x99, 0xb8,
                0x25, 0xad, 0x2c, 0x48,
            ],
        ]
    }

    fn ml_dsa_key(index: usize) -> Vec<u8> {
        vec![0x42 + u8::try_from(index).unwrap(); 1952]
    }

    fn parent_fixture() -> (
        VerifierKeyAuthorizationPolicyV1,
        VerifierKeyAuthorizationRootV1,
        VerifierAuthenticationPublicKeyBundleV1,
        VerifierPublicKeyRequirementsBindingV1,
    ) {
        let policy = VerifierKeyAuthorizationPolicyV1::default();
        let policy_digest = verifier_key_authorization_policy_digest(&policy).unwrap();
        let ed = ed25519_keys();

        let mut keysets = Vec::new();
        let mut evidence = Vec::new();
        for i in 0..3 {
            let release = digest(0x21 + u8::try_from(i).unwrap());
            let ed_key = ed[i].to_vec();
            let ml_key = ml_dsa_key(i);
            let ed_id = xenia_authentication_signer_key_id_v1(
                XENIA_ED25519_AUTHENTICATION_SUITE_ID,
                &ed_key,
            )
            .unwrap();
            let ml_id = xenia_authentication_signer_key_id_v1(
                XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID,
                &ml_key,
            )
            .unwrap();
            keysets.push(AuthorizedVerifierKeysetV1 {
                verifier_release_digest: release,
                verifier_lineage_digest: digest(0x31 + u8::try_from(i).unwrap()),
                builder_control_domain_digest: if i == 1 { digest(0x42) } else { digest(0x41) },
                ed25519_signer_key_id: ed_id,
                ml_dsa_65_signer_key_id: ml_id,
            });
            evidence.push(VerifierAuthenticationPublicKeyEvidenceV1 {
                verifier_release_digest: release,
                authentication_suite_id: XENIA_ED25519_AUTHENTICATION_SUITE_ID,
                xenia_authentication_profile_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
                public_key_bytes: ed_key,
                signer_key_id: ed_id,
            });
            evidence.push(VerifierAuthenticationPublicKeyEvidenceV1 {
                verifier_release_digest: release,
                authentication_suite_id: XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID,
                xenia_authentication_profile_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
                public_key_bytes: ml_key,
                signer_key_id: ml_id,
            });
        }

        let root = VerifierKeyAuthorizationRootV1 {
            verifier_key_authorization_profile_id: VERIFIER_KEY_AUTHORIZATION_PROFILE_ID.to_owned(),
            election_definition_digest: digest(0x11),
            jurisdiction_snapshot_digest: digest(0x12),
            authorization_policy_digest: policy_digest,
            authorized_verifiers: keysets,
        };
        let root_digest = verifier_key_authorization_root_digest(&root, &policy).unwrap();
        let bundle = VerifierAuthenticationPublicKeyBundleV1 {
            verifier_public_key_binding_profile_id: VERIFIER_PUBLIC_KEY_BINDING_PROFILE_ID
                .to_owned(),
            verifier_key_authorization_root_digest: root_digest,
            election_definition_digest: digest(0x11),
            jurisdiction_snapshot_digest: digest(0x12),
            xenia_authentication_profile_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
            xenia_authentication_suite_registry_digest:
                XENIA_AUTHENTICATION_SUITE_REGISTRY_V1_SHA256,
            evidence,
        };
        let bundle_digest =
            verifier_authentication_public_key_bundle_digest(&bundle, &root, &policy).unwrap();
        let requirements = VerifierPublicKeyRequirementsBindingV1 {
            elect017_requirements_binding: VerifierKeyAuthorizationRequirementsBindingV1 {
                previous_certification_requirements_digest: digest(0x71),
                verifier_key_authorization_root_digest: root_digest,
            },
            xenia_authentication_profile_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
            verifier_public_key_bundle_digest: bundle_digest,
        };
        (policy, root, bundle, requirements)
    }

    fn valid_manifest(
        policy: &VerifierKeyAuthorizationPolicyV1,
        root: &VerifierKeyAuthorizationRootV1,
        bundle: &VerifierAuthenticationPublicKeyBundleV1,
        requirements: &VerifierPublicKeyRequirementsBindingV1,
    ) -> (AuthenticatedVerifierEvidencePackageManifestV2, Vec<u8>) {
        let root_digest = verifier_key_authorization_root_digest(root, policy).unwrap();
        let bundle_bytes =
            canonical_verifier_authentication_public_key_bundle_bytes(bundle, root, policy)
                .unwrap();
        let bundle_digest =
            verifier_authentication_public_key_bundle_digest(bundle, root, policy).unwrap();
        let requirements_digest =
            verifier_public_key_requirements_digest(requirements, bundle, root, policy).unwrap();
        let manifest = AuthenticatedVerifierEvidencePackageManifestV2 {
            authenticated_verifier_package_profile_id: AUTHENTICATED_VERIFIER_PACKAGE_PROFILE_ID
                .to_owned(),
            legacy_v1_manifest: legacy_manifest(),
            elect017_authorization_root_digest: root_digest,
            elect017b_requirements_digest: requirements_digest,
            xenia_authentication_profile_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
            verifier_public_key_bundle_digest: bundle_digest,
            extensions: vec![
                AuthenticatedVerifierExtensionArtifactRefV1 {
                    role: AuthenticatedVerifierExtensionRoleV1::VerifierPublicKeyBundle,
                    canonical_path: VERIFIER_PUBLIC_KEY_BUNDLE_PATH.to_owned(),
                    content_digest: bundle_digest,
                    byte_length: u64::try_from(bundle_bytes.len()).unwrap(),
                    disclosure: ArtifactDisclosureClass::PublicVerificationEvidence,
                },
                AuthenticatedVerifierExtensionArtifactRefV1 {
                    role: AuthenticatedVerifierExtensionRoleV1::XeniaAuthenticationProfile,
                    canonical_path: XENIA_AUTHENTICATION_PROFILE_PATH.to_owned(),
                    content_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
                    byte_length: u64::try_from(XENIA_PROFILE_BYTES.len()).unwrap(),
                    disclosure: ArtifactDisclosureClass::PublicVerificationEvidence,
                },
            ],
        };
        (manifest, bundle_bytes)
    }

    #[test]
    fn exact_extension_bytes_qualify_and_match_language_neutral_vectors() {
        assert_eq!(
            XENIA_PROFILE_BYTES.len(),
            XENIA_AUTHENTICATION_PROFILE_V1_BYTES
        );
        assert_eq!(
            sha256(XENIA_PROFILE_BYTES),
            XENIA_AUTHENTICATION_PROFILE_V1_SHA256
        );

        let (policy, root, bundle, requirements) = parent_fixture();
        let (manifest, bundle_bytes) = valid_manifest(&policy, &root, &bundle, &requirements);
        assert_eq!(bundle_bytes.len(), 6838);
        let qualified = qualify_authenticated_verifier_evidence_package_v2(
            &manifest,
            &root,
            &policy,
            &bundle,
            &requirements,
            &bundle_bytes,
            XENIA_PROFILE_BYTES,
        )
        .unwrap();
        assert_eq!(
            qualified.authenticated_verifier_extension_root_digest(),
            GOLDEN_EXTENSION_ROOT_DIGEST
        );
        assert_eq!(
            qualified.authenticated_verifier_package_identity_digest(),
            GOLDEN_PACKAGE_IDENTITY_DIGEST
        );
        assert_eq!(qualified.legacy_v1_package_root_digest(), digest(4));
    }

    #[test]
    fn extension_order_is_not_semantic() {
        let (policy, root, bundle, requirements) = parent_fixture();
        let (manifest, bundle_bytes) = valid_manifest(&policy, &root, &bundle, &requirements);
        let mut reversed = manifest.clone();
        reversed.extensions.reverse();
        let a = qualify_authenticated_verifier_evidence_package_v2(
            &manifest,
            &root,
            &policy,
            &bundle,
            &requirements,
            &bundle_bytes,
            XENIA_PROFILE_BYTES,
        )
        .unwrap();
        let b = qualify_authenticated_verifier_evidence_package_v2(
            &reversed,
            &root,
            &policy,
            &bundle,
            &requirements,
            &bundle_bytes,
            XENIA_PROFILE_BYTES,
        )
        .unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn duplicate_or_missing_extension_role_fails_closed() {
        let (policy, root, bundle, requirements) = parent_fixture();
        let (manifest, bundle_bytes) = valid_manifest(&policy, &root, &bundle, &requirements);

        let mut missing = manifest.clone();
        missing.extensions.pop();
        assert_eq!(
            qualify_authenticated_verifier_evidence_package_v2(
                &missing,
                &root,
                &policy,
                &bundle,
                &requirements,
                &bundle_bytes,
                XENIA_PROFILE_BYTES,
            ),
            Err(AuthenticatedVerifierPackageViolation::WrongExtensionCount)
        );

        let mut duplicate = manifest.clone();
        duplicate.extensions[1] = duplicate.extensions[0].clone();
        assert_eq!(
            qualify_authenticated_verifier_evidence_package_v2(
                &duplicate,
                &root,
                &policy,
                &bundle,
                &requirements,
                &bundle_bytes,
                XENIA_PROFILE_BYTES,
            ),
            Err(AuthenticatedVerifierPackageViolation::DuplicateExtensionRole)
        );
    }

    #[test]
    fn changed_carried_bundle_bytes_fail_even_with_correct_manifest_digest() {
        let (policy, root, bundle, requirements) = parent_fixture();
        let (manifest, mut bundle_bytes) = valid_manifest(&policy, &root, &bundle, &requirements);
        bundle_bytes[0] ^= 0x01;
        assert_eq!(
            qualify_authenticated_verifier_evidence_package_v2(
                &manifest,
                &root,
                &policy,
                &bundle,
                &requirements,
                &bundle_bytes,
                XENIA_PROFILE_BYTES,
            ),
            Err(AuthenticatedVerifierPackageViolation::VerifierPublicKeyBundleBytesMismatch)
        );
    }

    #[test]
    fn changed_profile_bytes_fail_even_with_correct_manifest_digest() {
        let (policy, root, bundle, requirements) = parent_fixture();
        let (manifest, bundle_bytes) = valid_manifest(&policy, &root, &bundle, &requirements);
        let mut profile_bytes = XENIA_PROFILE_BYTES.to_vec();
        profile_bytes[0] ^= 0x01;
        assert_eq!(
            qualify_authenticated_verifier_evidence_package_v2(
                &manifest,
                &root,
                &policy,
                &bundle,
                &requirements,
                &bundle_bytes,
                &profile_bytes,
            ),
            Err(AuthenticatedVerifierPackageViolation::XeniaProfileDigestMismatch)
        );
    }

    #[test]
    fn truncated_profile_fails_before_digest_acceptance() {
        let (policy, root, bundle, requirements) = parent_fixture();
        let (manifest, bundle_bytes) = valid_manifest(&policy, &root, &bundle, &requirements);
        assert_eq!(
            qualify_authenticated_verifier_evidence_package_v2(
                &manifest,
                &root,
                &policy,
                &bundle,
                &requirements,
                &bundle_bytes,
                &XENIA_PROFILE_BYTES[..XENIA_PROFILE_BYTES.len() - 1],
            ),
            Err(AuthenticatedVerifierPackageViolation::XeniaProfileLengthMismatch)
        );
    }

    #[test]
    fn stale_or_spliced_parent_digests_fail_closed() {
        let (policy, root, bundle, requirements) = parent_fixture();
        let (manifest, bundle_bytes) = valid_manifest(&policy, &root, &bundle, &requirements);

        let mut wrong_root = manifest.clone();
        wrong_root.elect017_authorization_root_digest = digest(0x99);
        assert_eq!(
            qualify_authenticated_verifier_evidence_package_v2(
                &wrong_root,
                &root,
                &policy,
                &bundle,
                &requirements,
                &bundle_bytes,
                XENIA_PROFILE_BYTES,
            ),
            Err(AuthenticatedVerifierPackageViolation::Elect017AuthorizationRootMismatch)
        );

        let mut wrong_requirements = manifest.clone();
        wrong_requirements.elect017b_requirements_digest = digest(0x98);
        assert_eq!(
            qualify_authenticated_verifier_evidence_package_v2(
                &wrong_requirements,
                &root,
                &policy,
                &bundle,
                &requirements,
                &bundle_bytes,
                XENIA_PROFILE_BYTES,
            ),
            Err(AuthenticatedVerifierPackageViolation::Elect017bRequirementsDigestMismatch)
        );
    }

    #[test]
    fn extension_refs_must_match_actual_bytes_and_qualified_profile() {
        let (policy, root, bundle, requirements) = parent_fixture();
        let (manifest, bundle_bytes) = valid_manifest(&policy, &root, &bundle, &requirements);

        let mut wrong_bundle_ref = manifest.clone();
        wrong_bundle_ref.extensions[0].content_digest = digest(0x90);
        assert_eq!(
            qualify_authenticated_verifier_evidence_package_v2(
                &wrong_bundle_ref,
                &root,
                &policy,
                &bundle,
                &requirements,
                &bundle_bytes,
                XENIA_PROFILE_BYTES,
            ),
            Err(AuthenticatedVerifierPackageViolation::VerifierPublicKeyBundleRefDigestMismatch)
        );

        let mut wrong_profile_ref = manifest.clone();
        wrong_profile_ref.extensions[1].content_digest = digest(0x91);
        assert_eq!(
            qualify_authenticated_verifier_evidence_package_v2(
                &wrong_profile_ref,
                &root,
                &policy,
                &bundle,
                &requirements,
                &bundle_bytes,
                XENIA_PROFILE_BYTES,
            ),
            Err(AuthenticatedVerifierPackageViolation::XeniaProfileRefDigestMismatch)
        );
    }

    #[test]
    fn extension_paths_are_exact_public_and_cannot_collide_with_v1() {
        let (policy, root, bundle, requirements) = parent_fixture();
        let (manifest, bundle_bytes) = valid_manifest(&policy, &root, &bundle, &requirements);

        let mut wrong_path = manifest.clone();
        wrong_path.extensions[0].canonical_path = "other/key-bundle.bin".to_owned();
        assert_eq!(
            qualify_authenticated_verifier_evidence_package_v2(
                &wrong_path,
                &root,
                &policy,
                &bundle,
                &requirements,
                &bundle_bytes,
                XENIA_PROFILE_BYTES,
            ),
            Err(AuthenticatedVerifierPackageViolation::WrongExtensionPath)
        );

        let mut secret = manifest.clone();
        secret.extensions[0].disclosure = ArtifactDisclosureClass::NonPublicOrSecret;
        assert_eq!(
            qualify_authenticated_verifier_evidence_package_v2(
                &secret,
                &root,
                &policy,
                &bundle,
                &requirements,
                &bundle_bytes,
                XENIA_PROFILE_BYTES,
            ),
            Err(AuthenticatedVerifierPackageViolation::ExtensionMustBePublic)
        );

        let mut collision = manifest.clone();
        collision.legacy_v1_manifest.artifacts[0].canonical_path =
            VERIFIER_PUBLIC_KEY_BUNDLE_PATH.to_owned();
        assert_eq!(
            qualify_authenticated_verifier_evidence_package_v2(
                &collision,
                &root,
                &policy,
                &bundle,
                &requirements,
                &bundle_bytes,
                XENIA_PROFILE_BYTES,
            ),
            Err(AuthenticatedVerifierPackageViolation::ExtensionPathCollidesWithLegacyArtifact)
        );
    }

    #[test]
    fn legacy_v1_manifest_must_still_pass_its_original_contract() {
        let (policy, root, bundle, requirements) = parent_fixture();
        let (mut manifest, bundle_bytes) = valid_manifest(&policy, &root, &bundle, &requirements);
        manifest.legacy_v1_manifest.artifacts.pop();
        assert!(matches!(
            qualify_authenticated_verifier_evidence_package_v2(
                &manifest,
                &root,
                &policy,
                &bundle,
                &requirements,
                &bundle_bytes,
                XENIA_PROFILE_BYTES,
            ),
            Err(AuthenticatedVerifierPackageViolation::LegacyManifest(_))
        ));
    }

    #[test]
    fn v2_identity_extends_but_does_not_rewrite_legacy_package_root() {
        let (policy, root, bundle, requirements) = parent_fixture();
        let (manifest, bundle_bytes) = valid_manifest(&policy, &root, &bundle, &requirements);
        let qualified = qualify_authenticated_verifier_evidence_package_v2(
            &manifest,
            &root,
            &policy,
            &bundle,
            &requirements,
            &bundle_bytes,
            XENIA_PROFILE_BYTES,
        )
        .unwrap();
        assert_eq!(
            qualified.legacy_v1_package_root_digest(),
            manifest.legacy_v1_manifest.package_root_digest
        );
        assert_ne!(
            qualified.authenticated_verifier_package_identity_digest(),
            manifest.legacy_v1_manifest.package_root_digest
        );
    }

    #[test]
    fn bundle_resource_ceiling_is_preflight_and_fail_closed() {
        let (policy, root, bundle, requirements) = parent_fixture();
        let (manifest, _) = valid_manifest(&policy, &root, &bundle, &requirements);
        let oversized = vec![0_u8; MAX_VERIFIER_PUBLIC_KEY_BUNDLE_BYTES_V2 + 1];
        assert_eq!(
            qualify_authenticated_verifier_evidence_package_v2(
                &manifest,
                &root,
                &policy,
                &bundle,
                &requirements,
                &oversized,
                XENIA_PROFILE_BYTES,
            ),
            Err(AuthenticatedVerifierPackageViolation::VerifierPublicKeyBundleTooLarge)
        );
    }
}
