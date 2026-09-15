//! ELECT-017B public-key/profile binding for Mycelix public-election verifier authentication.
//!
//! This crate consumes the hosted-qualified ELECT-017 authorization root and the hosted-qualified
//! Xenia generic authentication profile. It proves that complete public-key evidence recomputes to
//! the exact suite-specific signer identities already frozen by ELECT-017.
//!
//! It performs no signature verification, accepts no private-key material, and defines no
//! key-lifecycle authority.

use election_integrity_types::Digest32;
use election_verifier_key_authorization::{
    AuthorizedVerifierKeysetV1, VerifierKeyAuthorizationPolicyV1,
    VerifierKeyAuthorizationRequirementsBindingV1, VerifierKeyAuthorizationRequirementsViolation,
    VerifierKeyAuthorizationRootV1, VerifierKeyAuthorizationRootViolation,
    XENIA_AUTHENTICATION_SUITE_REGISTRY_V1_SHA256, XENIA_ED25519_AUTHENTICATION_SUITE_ID,
    XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID, validate_verifier_key_authorization_root,
    verifier_key_authorization_requirements_digest, verifier_key_authorization_root_digest,
};
use sha2::{Digest as ShaDigest, Sha256};
use std::collections::HashSet;

pub const VERIFIER_PUBLIC_KEY_BINDING_PROFILE_ID: &str =
    "mycelix-public-election-verifier-public-key-binding-v1";
pub const XENIA_AUTHENTICATION_PROFILE_V1_SHA256: Digest32 = [
    0xe8, 0xb6, 0xac, 0x90, 0x02, 0x8c, 0x30, 0x64, 0x88, 0x0b, 0xfb, 0x6b, 0x59, 0xac, 0x7f, 0xbd,
    0x1a, 0x1d, 0xb0, 0x41, 0x82, 0xbd, 0x38, 0x41, 0xe6, 0x56, 0xbb, 0xa2, 0x73, 0x6d, 0x2b, 0x90,
];
pub const XENIA_ED25519_PUBLIC_KEY_BYTES: usize = 32;
pub const XENIA_ML_DSA_65_PUBLIC_KEY_BYTES: usize = 1952;
pub const MAX_PUBLIC_KEY_EVIDENCE_RECORDS_V1: usize = 128;
pub const MAX_CANONICAL_STRING_BYTES: usize = 256;

const XENIA_SIGNER_KEY_ID_DOMAIN: &[u8] = b"XENIA:AuthenticationSignerKeyId:v1";
const BUNDLE_DOMAIN: &[u8] = b"MYCELIX:PUBLIC-ELECTION:VERIFIER-PUBLIC-KEY-BUNDLE:V1\0";
const REQUIREMENTS_DOMAIN: &[u8] =
    b"MYCELIX:PUBLIC-ELECTION:CERTIFICATION-REQUIREMENTS:VERIFIER-PUBLIC-KEY-BINDING:V1\0";

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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum XeniaSignerKeyIdViolation {
    UnknownAuthenticationSuite,
    WrongEd25519PublicKeyLength { actual: usize },
    WrongMlDsa65PublicKeyLength { actual: usize },
    PublicKeyLengthOverflow,
}

pub fn xenia_authentication_signer_key_id_v1(
    authentication_suite_id: u16,
    public_key_bytes: &[u8],
) -> Result<Digest32, XeniaSignerKeyIdViolation> {
    match authentication_suite_id {
        XENIA_ED25519_AUTHENTICATION_SUITE_ID => {
            if public_key_bytes.len() != XENIA_ED25519_PUBLIC_KEY_BYTES {
                return Err(XeniaSignerKeyIdViolation::WrongEd25519PublicKeyLength {
                    actual: public_key_bytes.len(),
                });
            }
        }
        XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID => {
            if public_key_bytes.len() != XENIA_ML_DSA_65_PUBLIC_KEY_BYTES {
                return Err(XeniaSignerKeyIdViolation::WrongMlDsa65PublicKeyLength {
                    actual: public_key_bytes.len(),
                });
            }
        }
        _ => return Err(XeniaSignerKeyIdViolation::UnknownAuthenticationSuite),
    }

    let public_key_length = u64::try_from(public_key_bytes.len())
        .map_err(|_| XeniaSignerKeyIdViolation::PublicKeyLengthOverflow)?;
    let mut bytes =
        Vec::with_capacity(XENIA_SIGNER_KEY_ID_DOMAIN.len() + 2 + 8 + public_key_bytes.len());
    bytes.extend_from_slice(XENIA_SIGNER_KEY_ID_DOMAIN);
    bytes.extend_from_slice(&authentication_suite_id.to_le_bytes());
    bytes.extend_from_slice(&public_key_length.to_le_bytes());
    bytes.extend_from_slice(public_key_bytes);
    Ok(sha256(&bytes))
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifierAuthenticationPublicKeyEvidenceV1 {
    pub verifier_release_digest: Digest32,
    pub authentication_suite_id: u16,
    pub xenia_authentication_profile_digest: Digest32,
    pub public_key_bytes: Vec<u8>,
    pub signer_key_id: Digest32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifierAuthenticationPublicKeyBundleV1 {
    pub verifier_public_key_binding_profile_id: String,
    pub verifier_key_authorization_root_digest: Digest32,
    pub election_definition_digest: Digest32,
    pub jurisdiction_snapshot_digest: Digest32,
    pub xenia_authentication_profile_digest: Digest32,
    pub xenia_authentication_suite_registry_digest: Digest32,
    pub evidence: Vec<VerifierAuthenticationPublicKeyEvidenceV1>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VerifierPublicKeyBundleViolation {
    ParentAuthorizationRoot(VerifierKeyAuthorizationRootViolation),
    WrongBindingProfile,
    CanonicalStringTooLong,
    WrongXeniaAuthenticationProfile,
    WrongXeniaAuthenticationSuiteRegistry,
    AuthorizationRootDigestMismatch,
    ElectionDefinitionDigestMismatch,
    JurisdictionSnapshotDigestMismatch,
    TooManyEvidenceRecords,
    PublicKeyLengthOverflow,
    UnexpectedVerifierRelease,
    UnknownAuthenticationSuite,
    EvidenceProfileMismatch,
    WrongEd25519PublicKeyLength,
    WrongMlDsa65PublicKeyLength,
    ZeroSignerKeyId,
    SignerKeyIdRecomputationMismatch,
    SignerKeyIdNotAuthorized,
    DuplicateVerifierSuiteEvidence,
    MissingEd25519PublicKey,
    MissingMlDsa65PublicKey,
}

fn expected_authorized_signer_key_id(
    verifier: &AuthorizedVerifierKeysetV1,
    suite: u16,
) -> Result<Digest32, VerifierPublicKeyBundleViolation> {
    match suite {
        XENIA_ED25519_AUTHENTICATION_SUITE_ID => Ok(verifier.ed25519_signer_key_id),
        XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID => Ok(verifier.ml_dsa_65_signer_key_id),
        _ => Err(VerifierPublicKeyBundleViolation::UnknownAuthenticationSuite),
    }
}

pub fn validate_verifier_authentication_public_key_bundle(
    bundle: &VerifierAuthenticationPublicKeyBundleV1,
    authorization_root: &VerifierKeyAuthorizationRootV1,
    authorization_policy: &VerifierKeyAuthorizationPolicyV1,
) -> Result<(), VerifierPublicKeyBundleViolation> {
    validate_verifier_key_authorization_root(authorization_root, authorization_policy)
        .map_err(VerifierPublicKeyBundleViolation::ParentAuthorizationRoot)?;

    if bundle.verifier_public_key_binding_profile_id != VERIFIER_PUBLIC_KEY_BINDING_PROFILE_ID {
        return Err(VerifierPublicKeyBundleViolation::WrongBindingProfile);
    }
    if bundle.verifier_public_key_binding_profile_id.len() > MAX_CANONICAL_STRING_BYTES {
        return Err(VerifierPublicKeyBundleViolation::CanonicalStringTooLong);
    }
    if bundle.xenia_authentication_profile_digest != XENIA_AUTHENTICATION_PROFILE_V1_SHA256 {
        return Err(VerifierPublicKeyBundleViolation::WrongXeniaAuthenticationProfile);
    }
    if bundle.xenia_authentication_suite_registry_digest
        != XENIA_AUTHENTICATION_SUITE_REGISTRY_V1_SHA256
    {
        return Err(VerifierPublicKeyBundleViolation::WrongXeniaAuthenticationSuiteRegistry);
    }
    if bundle.election_definition_digest != authorization_root.election_definition_digest {
        return Err(VerifierPublicKeyBundleViolation::ElectionDefinitionDigestMismatch);
    }
    if bundle.jurisdiction_snapshot_digest != authorization_root.jurisdiction_snapshot_digest {
        return Err(VerifierPublicKeyBundleViolation::JurisdictionSnapshotDigestMismatch);
    }

    let expected_root_digest =
        verifier_key_authorization_root_digest(authorization_root, authorization_policy)
            .map_err(VerifierPublicKeyBundleViolation::ParentAuthorizationRoot)?;
    if bundle.verifier_key_authorization_root_digest != expected_root_digest {
        return Err(VerifierPublicKeyBundleViolation::AuthorizationRootDigestMismatch);
    }

    if bundle.evidence.len() > MAX_PUBLIC_KEY_EVIDENCE_RECORDS_V1 {
        return Err(VerifierPublicKeyBundleViolation::TooManyEvidenceRecords);
    }

    let mut observed = HashSet::with_capacity(bundle.evidence.len());

    for record in &bundle.evidence {
        if record.xenia_authentication_profile_digest != XENIA_AUTHENTICATION_PROFILE_V1_SHA256 {
            return Err(VerifierPublicKeyBundleViolation::EvidenceProfileMismatch);
        }
        if record.signer_key_id == [0_u8; 32] {
            return Err(VerifierPublicKeyBundleViolation::ZeroSignerKeyId);
        }

        let authorized_verifier = authorization_root
            .authorized_verifiers
            .iter()
            .find(|verifier| verifier.verifier_release_digest == record.verifier_release_digest)
            .ok_or(VerifierPublicKeyBundleViolation::UnexpectedVerifierRelease)?;

        if !observed.insert((
            record.verifier_release_digest,
            record.authentication_suite_id,
        )) {
            return Err(VerifierPublicKeyBundleViolation::DuplicateVerifierSuiteEvidence);
        }

        let recomputed = xenia_authentication_signer_key_id_v1(
            record.authentication_suite_id,
            &record.public_key_bytes,
        )
        .map_err(|violation| match violation {
            XeniaSignerKeyIdViolation::UnknownAuthenticationSuite => {
                VerifierPublicKeyBundleViolation::UnknownAuthenticationSuite
            }
            XeniaSignerKeyIdViolation::WrongEd25519PublicKeyLength { .. } => {
                VerifierPublicKeyBundleViolation::WrongEd25519PublicKeyLength
            }
            XeniaSignerKeyIdViolation::WrongMlDsa65PublicKeyLength { .. } => {
                VerifierPublicKeyBundleViolation::WrongMlDsa65PublicKeyLength
            }
            XeniaSignerKeyIdViolation::PublicKeyLengthOverflow => {
                VerifierPublicKeyBundleViolation::PublicKeyLengthOverflow
            }
        })?;

        if record.signer_key_id != recomputed {
            return Err(VerifierPublicKeyBundleViolation::SignerKeyIdRecomputationMismatch);
        }

        let authorized =
            expected_authorized_signer_key_id(authorized_verifier, record.authentication_suite_id)?;
        if recomputed != authorized {
            return Err(VerifierPublicKeyBundleViolation::SignerKeyIdNotAuthorized);
        }
    }

    for verifier in &authorization_root.authorized_verifiers {
        if !observed.contains(&(
            verifier.verifier_release_digest,
            XENIA_ED25519_AUTHENTICATION_SUITE_ID,
        )) {
            return Err(VerifierPublicKeyBundleViolation::MissingEd25519PublicKey);
        }
        if !observed.contains(&(
            verifier.verifier_release_digest,
            XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID,
        )) {
            return Err(VerifierPublicKeyBundleViolation::MissingMlDsa65PublicKey);
        }
    }

    Ok(())
}

pub fn canonical_verifier_authentication_public_key_bundle_bytes(
    bundle: &VerifierAuthenticationPublicKeyBundleV1,
    authorization_root: &VerifierKeyAuthorizationRootV1,
    authorization_policy: &VerifierKeyAuthorizationPolicyV1,
) -> Result<Vec<u8>, VerifierPublicKeyBundleViolation> {
    validate_verifier_authentication_public_key_bundle(
        bundle,
        authorization_root,
        authorization_policy,
    )?;

    let mut ordered = bundle.evidence.clone();
    ordered.sort_by_key(|record| {
        (
            record.verifier_release_digest,
            record.authentication_suite_id,
            record.signer_key_id,
        )
    });

    let count = u16::try_from(ordered.len())
        .map_err(|_| VerifierPublicKeyBundleViolation::TooManyEvidenceRecords)?;
    let mut bytes = Vec::with_capacity(8192);
    bytes.extend_from_slice(BUNDLE_DOMAIN);
    append_len_prefixed_utf8(&mut bytes, &bundle.verifier_public_key_binding_profile_id)
        .map_err(|_| VerifierPublicKeyBundleViolation::CanonicalStringTooLong)?;
    bytes.extend_from_slice(&bundle.verifier_key_authorization_root_digest);
    bytes.extend_from_slice(&bundle.election_definition_digest);
    bytes.extend_from_slice(&bundle.jurisdiction_snapshot_digest);
    bytes.extend_from_slice(&bundle.xenia_authentication_profile_digest);
    bytes.extend_from_slice(&bundle.xenia_authentication_suite_registry_digest);
    bytes.extend_from_slice(&count.to_be_bytes());

    for record in ordered {
        let public_key_length = u32::try_from(record.public_key_bytes.len())
            .map_err(|_| VerifierPublicKeyBundleViolation::PublicKeyLengthOverflow)?;
        bytes.extend_from_slice(&record.verifier_release_digest);
        bytes.extend_from_slice(&record.authentication_suite_id.to_be_bytes());
        bytes.extend_from_slice(&record.xenia_authentication_profile_digest);
        bytes.extend_from_slice(&record.signer_key_id);
        bytes.extend_from_slice(&public_key_length.to_be_bytes());
        bytes.extend_from_slice(&record.public_key_bytes);
    }
    Ok(bytes)
}

pub fn verifier_authentication_public_key_bundle_digest(
    bundle: &VerifierAuthenticationPublicKeyBundleV1,
    authorization_root: &VerifierKeyAuthorizationRootV1,
    authorization_policy: &VerifierKeyAuthorizationPolicyV1,
) -> Result<Digest32, VerifierPublicKeyBundleViolation> {
    Ok(sha256(
        &canonical_verifier_authentication_public_key_bundle_bytes(
            bundle,
            authorization_root,
            authorization_policy,
        )?,
    ))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VerifierPublicKeyRequirementsBindingV1 {
    pub elect017_requirements_binding: VerifierKeyAuthorizationRequirementsBindingV1,
    pub xenia_authentication_profile_digest: Digest32,
    pub verifier_public_key_bundle_digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VerifierPublicKeyRequirementsViolation {
    ParentRequirements(VerifierKeyAuthorizationRequirementsViolation),
    ParentAuthorizationRoot(VerifierKeyAuthorizationRootViolation),
    PublicKeyBundle(VerifierPublicKeyBundleViolation),
    Elect017AuthorizationRootMismatch,
    WrongXeniaAuthenticationProfile,
    VerifierPublicKeyBundleDigestMismatch,
}

pub fn verifier_public_key_requirements_digest(
    binding: &VerifierPublicKeyRequirementsBindingV1,
    bundle: &VerifierAuthenticationPublicKeyBundleV1,
    authorization_root: &VerifierKeyAuthorizationRootV1,
    authorization_policy: &VerifierKeyAuthorizationPolicyV1,
) -> Result<Digest32, VerifierPublicKeyRequirementsViolation> {
    let elect017_certification_requirements_digest =
        verifier_key_authorization_requirements_digest(&binding.elect017_requirements_binding)
            .map_err(VerifierPublicKeyRequirementsViolation::ParentRequirements)?;

    let expected_authorization_root_digest =
        verifier_key_authorization_root_digest(authorization_root, authorization_policy)
            .map_err(VerifierPublicKeyRequirementsViolation::ParentAuthorizationRoot)?;
    if binding
        .elect017_requirements_binding
        .verifier_key_authorization_root_digest
        != expected_authorization_root_digest
    {
        return Err(VerifierPublicKeyRequirementsViolation::Elect017AuthorizationRootMismatch);
    }

    if binding.xenia_authentication_profile_digest != XENIA_AUTHENTICATION_PROFILE_V1_SHA256 {
        return Err(VerifierPublicKeyRequirementsViolation::WrongXeniaAuthenticationProfile);
    }

    let expected_bundle_digest = verifier_authentication_public_key_bundle_digest(
        bundle,
        authorization_root,
        authorization_policy,
    )
    .map_err(VerifierPublicKeyRequirementsViolation::PublicKeyBundle)?;
    if binding.verifier_public_key_bundle_digest != expected_bundle_digest {
        return Err(VerifierPublicKeyRequirementsViolation::VerifierPublicKeyBundleDigestMismatch);
    }

    let mut bytes = Vec::with_capacity(REQUIREMENTS_DOMAIN.len() + 128);
    bytes.extend_from_slice(REQUIREMENTS_DOMAIN);
    bytes.extend_from_slice(&elect017_certification_requirements_digest);
    bytes.extend_from_slice(&expected_authorization_root_digest);
    bytes.extend_from_slice(&binding.xenia_authentication_profile_digest);
    bytes.extend_from_slice(&expected_bundle_digest);
    Ok(sha256(&bytes))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VerifierPublicKeyEvidenceArtifactRefV1 {
    pub verifier_public_key_bundle_digest: Digest32,
    pub xenia_authentication_profile_digest: Digest32,
    pub evidence_record_count: u16,
}

#[cfg(test)]
mod tests {
    use super::*;
    use election_verifier_key_authorization::{
        VERIFIER_KEY_AUTHORIZATION_PROFILE_ID, verifier_key_authorization_policy_digest,
    };

    const GOLDEN_ROOT_DIGEST: Digest32 = [
        0x4e, 0x52, 0xe8, 0xb4, 0x27, 0x61, 0x1e, 0x11, 0xb3, 0x6b, 0x0a, 0x22, 0xb8, 0xc8, 0xab,
        0x7b, 0x62, 0xbf, 0x80, 0x8d, 0x42, 0x23, 0xdb, 0x19, 0x55, 0xf5, 0x0c, 0xa9, 0xb0, 0x2e,
        0x65, 0xb5,
    ];
    const GOLDEN_ELECT017_REQUIREMENTS_DIGEST: Digest32 = [
        0x6a, 0xa1, 0x7c, 0x36, 0xfa, 0x6f, 0xda, 0x24, 0x75, 0x61, 0x57, 0xc3, 0xc5, 0x0a, 0x7b,
        0x74, 0x68, 0x52, 0x77, 0x1a, 0x26, 0xe4, 0xcb, 0xdf, 0xc9, 0x5b, 0x07, 0x82, 0xda, 0x18,
        0xd9, 0xc4,
    ];
    const GOLDEN_BUNDLE_DIGEST: Digest32 = [
        0x97, 0x56, 0x5d, 0x55, 0x4f, 0xfa, 0xdd, 0xbc, 0xc3, 0x5d, 0x2a, 0x22, 0xd2, 0x09, 0xec,
        0x1f, 0xb2, 0x64, 0x6c, 0x0f, 0x43, 0x03, 0x14, 0x6b, 0xb7, 0x3c, 0x3d, 0x96, 0x40, 0xfd,
        0xbe, 0xfc,
    ];
    const GOLDEN_REQUIREMENTS_DIGEST: Digest32 = [
        0x58, 0x85, 0xc0, 0x78, 0x2e, 0x5f, 0xcd, 0xdc, 0x7d, 0x96, 0xf4, 0xde, 0xa0, 0x4a, 0x08,
        0xae, 0xec, 0x53, 0xe3, 0xf7, 0x89, 0x34, 0xa1, 0xac, 0x39, 0x0a, 0x74, 0x67, 0xed, 0x0c,
        0xb2, 0x93,
    ];
    const GOLDEN_ED25519_KEY_ID: Digest32 = [
        0x3c, 0x0e, 0x8b, 0xd0, 0x16, 0x25, 0x3b, 0xfe, 0xd7, 0x34, 0xee, 0x6a, 0x24, 0xd4, 0xb4,
        0xa4, 0x7c, 0x2e, 0x15, 0x46, 0xbc, 0x04, 0xfa, 0x00, 0x2b, 0x43, 0x22, 0x9a, 0xd2, 0x95,
        0xe0, 0x55,
    ];
    const GOLDEN_ML_DSA_65_KEY_ID: Digest32 = [
        0xeb, 0xcf, 0x9a, 0xda, 0x05, 0xd7, 0x34, 0xec, 0x98, 0x91, 0x67, 0xee, 0x61, 0x27, 0x89,
        0x04, 0x7f, 0xa3, 0x29, 0x92, 0xe8, 0x1b, 0x26, 0x8e, 0xde, 0xcf, 0x4e, 0x80, 0x3e, 0x64,
        0x3f, 0x07,
    ];

    fn digest(byte: u8) -> Digest32 {
        [byte; 32]
    }

    const ED25519_PUBLIC_KEYS: [[u8; 32]; 3] = [
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
    ];

    fn ed25519_public_key(index: usize) -> Vec<u8> {
        ED25519_PUBLIC_KEYS[index].to_vec()
    }

    fn ml_dsa_public_key(index: usize) -> Vec<u8> {
        vec![0x42 + u8::try_from(index).unwrap(); XENIA_ML_DSA_65_PUBLIC_KEY_BYTES]
    }

    fn policy() -> VerifierKeyAuthorizationPolicyV1 {
        VerifierKeyAuthorizationPolicyV1::default()
    }

    fn authorization_root() -> VerifierKeyAuthorizationRootV1 {
        let policy = policy();
        let mut authorized_verifiers = Vec::new();
        for index in 0..3 {
            let ed = ed25519_public_key(index);
            let ml = ml_dsa_public_key(index);
            authorized_verifiers.push(AuthorizedVerifierKeysetV1 {
                verifier_release_digest: digest(0x21 + u8::try_from(index).unwrap()),
                verifier_lineage_digest: digest(0x31 + u8::try_from(index).unwrap()),
                builder_control_domain_digest: digest(if index == 1 { 0x42 } else { 0x41 }),
                ed25519_signer_key_id: xenia_authentication_signer_key_id_v1(
                    XENIA_ED25519_AUTHENTICATION_SUITE_ID,
                    &ed,
                )
                .unwrap(),
                ml_dsa_65_signer_key_id: xenia_authentication_signer_key_id_v1(
                    XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID,
                    &ml,
                )
                .unwrap(),
            });
        }
        VerifierKeyAuthorizationRootV1 {
            verifier_key_authorization_profile_id: VERIFIER_KEY_AUTHORIZATION_PROFILE_ID.to_owned(),
            election_definition_digest: digest(0x11),
            jurisdiction_snapshot_digest: digest(0x12),
            authorization_policy_digest: verifier_key_authorization_policy_digest(&policy).unwrap(),
            authorized_verifiers,
        }
    }

    fn bundle() -> VerifierAuthenticationPublicKeyBundleV1 {
        let policy = policy();
        let root = authorization_root();
        let mut evidence = Vec::new();
        for (index, verifier) in root.authorized_verifiers.iter().enumerate() {
            let ed = ed25519_public_key(index);
            let ml = ml_dsa_public_key(index);
            evidence.push(VerifierAuthenticationPublicKeyEvidenceV1 {
                verifier_release_digest: verifier.verifier_release_digest,
                authentication_suite_id: XENIA_ED25519_AUTHENTICATION_SUITE_ID,
                xenia_authentication_profile_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
                signer_key_id: verifier.ed25519_signer_key_id,
                public_key_bytes: ed,
            });
            evidence.push(VerifierAuthenticationPublicKeyEvidenceV1 {
                verifier_release_digest: verifier.verifier_release_digest,
                authentication_suite_id: XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID,
                xenia_authentication_profile_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
                signer_key_id: verifier.ml_dsa_65_signer_key_id,
                public_key_bytes: ml,
            });
        }
        VerifierAuthenticationPublicKeyBundleV1 {
            verifier_public_key_binding_profile_id: VERIFIER_PUBLIC_KEY_BINDING_PROFILE_ID
                .to_owned(),
            verifier_key_authorization_root_digest: verifier_key_authorization_root_digest(
                &root, &policy,
            )
            .unwrap(),
            election_definition_digest: root.election_definition_digest,
            jurisdiction_snapshot_digest: root.jurisdiction_snapshot_digest,
            xenia_authentication_profile_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
            xenia_authentication_suite_registry_digest:
                XENIA_AUTHENTICATION_SUITE_REGISTRY_V1_SHA256,
            evidence,
        }
    }

    #[test]
    fn xenia_signer_id_recomputation_matches_qualified_cross_language_vectors() {
        assert_eq!(
            xenia_authentication_signer_key_id_v1(
                XENIA_ED25519_AUTHENTICATION_SUITE_ID,
                &ed25519_public_key(0),
            )
            .unwrap(),
            GOLDEN_ED25519_KEY_ID
        );
        assert_eq!(
            xenia_authentication_signer_key_id_v1(
                XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID,
                &ml_dsa_public_key(0),
            )
            .unwrap(),
            GOLDEN_ML_DSA_65_KEY_ID
        );
    }

    #[test]
    fn qualified_xenia_profile_and_elect017_root_are_exactly_bound() {
        let policy = policy();
        let root = authorization_root();
        assert_eq!(
            verifier_key_authorization_root_digest(&root, &policy).unwrap(),
            GOLDEN_ROOT_DIGEST
        );
        let previous = digest(0x71);
        let elect017 = verifier_key_authorization_requirements_digest(
            &VerifierKeyAuthorizationRequirementsBindingV1 {
                previous_certification_requirements_digest: previous,
                verifier_key_authorization_root_digest: GOLDEN_ROOT_DIGEST,
            },
        )
        .unwrap();
        assert_eq!(elect017, GOLDEN_ELECT017_REQUIREMENTS_DIGEST);
    }

    #[test]
    fn canonical_bundle_is_order_invariant_and_matches_golden_vector() {
        let policy = policy();
        let root = authorization_root();
        let bundle = bundle();
        assert_eq!(
            verifier_authentication_public_key_bundle_digest(&bundle, &root, &policy).unwrap(),
            GOLDEN_BUNDLE_DIGEST
        );

        let mut reordered = bundle.clone();
        reordered.evidence.reverse();
        assert_eq!(
            verifier_authentication_public_key_bundle_digest(&reordered, &root, &policy).unwrap(),
            GOLDEN_BUNDLE_DIGEST
        );
    }

    #[test]
    fn changed_public_key_with_copied_old_id_fails_closed() {
        let policy = policy();
        let root = authorization_root();
        let mut bundle = bundle();
        bundle.evidence[0].public_key_bytes[0] ^= 1;
        assert_eq!(
            validate_verifier_authentication_public_key_bundle(&bundle, &root, &policy),
            Err(VerifierPublicKeyBundleViolation::SignerKeyIdRecomputationMismatch)
        );
    }

    #[test]
    fn recomputed_but_unauthorized_signer_id_fails_closed() {
        let policy = policy();
        let root = authorization_root();
        let mut bundle = bundle();
        bundle.evidence[0].public_key_bytes[0] ^= 1;
        bundle.evidence[0].signer_key_id = xenia_authentication_signer_key_id_v1(
            bundle.evidence[0].authentication_suite_id,
            &bundle.evidence[0].public_key_bytes,
        )
        .unwrap();
        assert_eq!(
            validate_verifier_authentication_public_key_bundle(&bundle, &root, &policy),
            Err(VerifierPublicKeyBundleViolation::SignerKeyIdNotAuthorized)
        );
    }

    #[test]
    fn wrong_suite_and_malformed_key_lengths_fail_closed() {
        let policy = policy();
        let root = authorization_root();

        let mut wrong_suite = bundle();
        wrong_suite.evidence[0].authentication_suite_id = XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID;
        assert_eq!(
            validate_verifier_authentication_public_key_bundle(&wrong_suite, &root, &policy),
            Err(VerifierPublicKeyBundleViolation::WrongMlDsa65PublicKeyLength)
        );

        let mut truncated = bundle();
        truncated.evidence[1].public_key_bytes.pop();
        assert_eq!(
            validate_verifier_authentication_public_key_bundle(&truncated, &root, &policy),
            Err(VerifierPublicKeyBundleViolation::WrongMlDsa65PublicKeyLength)
        );
    }

    #[test]
    fn profile_substitution_fails_at_bundle_and_record_boundaries() {
        let policy = policy();
        let root = authorization_root();

        let mut wrong_bundle_profile = bundle();
        wrong_bundle_profile.xenia_authentication_profile_digest = digest(0x99);
        assert_eq!(
            validate_verifier_authentication_public_key_bundle(
                &wrong_bundle_profile,
                &root,
                &policy
            ),
            Err(VerifierPublicKeyBundleViolation::WrongXeniaAuthenticationProfile)
        );

        let mut wrong_record_profile = bundle();
        wrong_record_profile.evidence[0].xenia_authentication_profile_digest = digest(0x99);
        assert_eq!(
            validate_verifier_authentication_public_key_bundle(
                &wrong_record_profile,
                &root,
                &policy
            ),
            Err(VerifierPublicKeyBundleViolation::EvidenceProfileMismatch)
        );
    }

    #[test]
    fn missing_suite_evidence_fails_closed() {
        let policy = policy();
        let root = authorization_root();

        let mut missing_ed = bundle();
        missing_ed.evidence.remove(0);
        assert_eq!(
            validate_verifier_authentication_public_key_bundle(&missing_ed, &root, &policy),
            Err(VerifierPublicKeyBundleViolation::MissingEd25519PublicKey)
        );

        let mut missing_ml = bundle();
        missing_ml.evidence.remove(1);
        assert_eq!(
            validate_verifier_authentication_public_key_bundle(&missing_ml, &root, &policy),
            Err(VerifierPublicKeyBundleViolation::MissingMlDsa65PublicKey)
        );
    }

    #[test]
    fn duplicate_or_unexpected_release_evidence_fails_closed() {
        let policy = policy();
        let root = authorization_root();

        let mut duplicate = bundle();
        duplicate.evidence.push(duplicate.evidence[0].clone());
        assert_eq!(
            validate_verifier_authentication_public_key_bundle(&duplicate, &root, &policy),
            Err(VerifierPublicKeyBundleViolation::DuplicateVerifierSuiteEvidence)
        );

        let mut unexpected = bundle();
        unexpected.evidence[0].verifier_release_digest = digest(0xee);
        assert_eq!(
            validate_verifier_authentication_public_key_bundle(&unexpected, &root, &policy),
            Err(VerifierPublicKeyBundleViolation::UnexpectedVerifierRelease)
        );
    }

    #[test]
    fn changed_authorization_root_or_authorized_signer_fails_closed() {
        let policy = policy();
        let root = authorization_root();

        let mut wrong_root = bundle();
        wrong_root.verifier_key_authorization_root_digest = digest(0x99);
        assert_eq!(
            validate_verifier_authentication_public_key_bundle(&wrong_root, &root, &policy),
            Err(VerifierPublicKeyBundleViolation::AuthorizationRootDigestMismatch)
        );

        let mut wrong_signer = bundle();
        wrong_signer.evidence[0].signer_key_id = digest(0x88);
        assert_eq!(
            validate_verifier_authentication_public_key_bundle(&wrong_signer, &root, &policy),
            Err(VerifierPublicKeyBundleViolation::SignerKeyIdRecomputationMismatch)
        );
    }

    #[test]
    fn certification_requirements_reject_root_or_bundle_splicing() {
        let policy = policy();
        let root = authorization_root();
        let bundle = bundle();
        let bundle_digest =
            verifier_authentication_public_key_bundle_digest(&bundle, &root, &policy).unwrap();

        let wrong_root = VerifierPublicKeyRequirementsBindingV1 {
            elect017_requirements_binding: VerifierKeyAuthorizationRequirementsBindingV1 {
                previous_certification_requirements_digest: digest(0x71),
                verifier_key_authorization_root_digest: digest(0x99),
            },
            xenia_authentication_profile_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
            verifier_public_key_bundle_digest: bundle_digest,
        };
        assert_eq!(
            verifier_public_key_requirements_digest(&wrong_root, &bundle, &root, &policy),
            Err(VerifierPublicKeyRequirementsViolation::Elect017AuthorizationRootMismatch)
        );

        let wrong_bundle = VerifierPublicKeyRequirementsBindingV1 {
            elect017_requirements_binding: VerifierKeyAuthorizationRequirementsBindingV1 {
                previous_certification_requirements_digest: digest(0x71),
                verifier_key_authorization_root_digest: GOLDEN_ROOT_DIGEST,
            },
            xenia_authentication_profile_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
            verifier_public_key_bundle_digest: digest(0x98),
        };
        assert_eq!(
            verifier_public_key_requirements_digest(&wrong_bundle, &bundle, &root, &policy),
            Err(VerifierPublicKeyRequirementsViolation::VerifierPublicKeyBundleDigestMismatch)
        );
    }

    #[test]
    fn certification_requirements_preserve_elect017_and_bind_public_key_bundle() {
        let policy = policy();
        let root = authorization_root();
        let bundle = bundle();
        let bundle_digest =
            verifier_authentication_public_key_bundle_digest(&bundle, &root, &policy).unwrap();
        assert_eq!(bundle_digest, GOLDEN_BUNDLE_DIGEST);

        let digest = verifier_public_key_requirements_digest(
            &VerifierPublicKeyRequirementsBindingV1 {
                elect017_requirements_binding: VerifierKeyAuthorizationRequirementsBindingV1 {
                    previous_certification_requirements_digest: digest(0x71),
                    verifier_key_authorization_root_digest: GOLDEN_ROOT_DIGEST,
                },
                xenia_authentication_profile_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
                verifier_public_key_bundle_digest: bundle_digest,
            },
            &bundle,
            &root,
            &policy,
        )
        .unwrap();
        assert_eq!(digest, GOLDEN_REQUIREMENTS_DIGEST);
    }
}
