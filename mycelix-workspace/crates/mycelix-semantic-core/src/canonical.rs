use sha2::{Digest, Sha256};

use crate::{
    Commitment32, SemanticEnvironmentV1, SemanticProfileRefV1, SemanticSubjectRefV1,
};

/// Stable identifier for the language-neutral outer commitment profile.
pub const SEMANTIC_COMMITMENT_PROFILE_ID: &str =
    "mycelix-semantic-commitment/sha256-length-prefixed-be-v1";

/// Stable revision of the outer commitment profile.
pub const SEMANTIC_COMMITMENT_PROFILE_REVISION: u16 = 1;

/// Marker for the frozen MYC-SEM-001C outer commitment profile.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SemanticCommitmentProfileV1;

impl SemanticCommitmentProfileV1 {
    /// Stable profile identifier.
    pub const fn id() -> &'static str {
        SEMANTIC_COMMITMENT_PROFILE_ID
    }

    /// Stable profile revision.
    pub const fn revision() -> u16 {
        SEMANTIC_COMMITMENT_PROFILE_REVISION
    }

    /// Hash algorithm frozen by this profile.
    pub const fn hash_algorithm() -> &'static str {
        "SHA-256"
    }
}

const ENVIRONMENT_DOMAIN_SEPARATOR: &[u8] = b"MYCELIX_SEMANTIC_ENVIRONMENT_V1\0";
const SUBJECT_DOMAIN_SEPARATOR: &[u8] = b"MYCELIX_SEMANTIC_SUBJECT_V1\0";

pub(crate) fn derive_environment_commitment(
    environment: &SemanticEnvironmentV1,
) -> Commitment32 {
    let mut canonical = Vec::with_capacity(512);
    canonical.extend_from_slice(ENVIRONMENT_DOMAIN_SEPARATOR);
    canonical.extend_from_slice(&SEMANTIC_COMMITMENT_PROFILE_REVISION.to_be_bytes());

    put_profile(&mut canonical, environment.schema().profile());
    put_profile(&mut canonical, environment.interpretation_profile());
    put_profile(&mut canonical, environment.identity_profile());
    put_profile(&mut canonical, environment.authority_profile());
    put_profile(&mut canonical, environment.temporal_profile());
    put_profile(&mut canonical, environment.canonicalization_profile());

    sha256_commitment(&canonical)
}

pub(crate) fn derive_subject_commitment(subject: &SemanticSubjectRefV1) -> Commitment32 {
    let mut canonical = Vec::with_capacity(384);
    canonical.extend_from_slice(SUBJECT_DOMAIN_SEPARATOR);
    canonical.extend_from_slice(&SEMANTIC_COMMITMENT_PROFILE_REVISION.to_be_bytes());

    canonical.extend_from_slice(
        &subject
            .environment()
            .commitment_profile_revision()
            .to_be_bytes(),
    );
    canonical.extend_from_slice(subject.environment().commitment().as_bytes());
    put_text(&mut canonical, subject.domain().as_str());
    put_profile(&mut canonical, subject.schema().profile());
    put_text(&mut canonical, subject.subject_id().as_str());

    sha256_commitment(&canonical)
}

fn sha256_commitment(canonical: &[u8]) -> Commitment32 {
    let digest: [u8; 32] = Sha256::digest(canonical).into();
    Commitment32::from_bytes(digest)
}

fn put_profile(out: &mut Vec<u8>, profile: &SemanticProfileRefV1) {
    put_text(out, profile.id().as_str());
    out.extend_from_slice(&profile.revision().to_be_bytes());
    out.extend_from_slice(profile.digest().as_bytes());
}

fn put_text(out: &mut Vec<u8>, value: &str) {
    let bytes = value.as_bytes();
    let len = u32::try_from(bytes.len()).expect("semantic text bound is below u32::MAX");
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
}
