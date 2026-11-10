//! Canonical owner-scoped checkpoint subjects for the SAP payment journal.
//!
//! A checkpoint is an attestation about one owner-specific journal prefix.
//! In a Holochain DNA, authorship is provided by the signed Create action that
//! commits this entry; this subject codec does not add a detached signature.
//! Runtime validation must bind the action author to the issuer DID, verify the
//! owner-authority grant, and resolve the exact owner-native source action.

use sha2::{Digest, Sha256};
use std::fmt;

const CHECKPOINT_DOMAIN: &[u8] = b"MXSAP-OWNER-CHECKPOINT-SUBJECT-V1\0";
const MAX_TEXT_BYTES: usize = 4 * 1024;
const MAX_SOURCE_ACTION_HASH_BYTES: usize = 256;
const KEY_PROFILE_HOLOCHAIN_ACTION_ED25519_V1: u8 = 1;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum CheckpointOwnerDomain {
    PaymentsLocalTransaction = 1,
    TreasuryFeeEffect = 2,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OwnerCheckpointSubjectV1 {
    pub operation_id: String,
    pub intent_digest: String,
    pub owner_domain: CheckpointOwnerDomain,
    pub event_log_digest: String,
    pub event_count: u64,
    pub tail_digest: String,
    pub checkpoint_sequence: u64,
    pub prior_checkpoint_digest: Option<String>,
    pub source_action_hash: Vec<u8>,
    pub issuer_did: String,
    pub issuer_key_fingerprint: String,
    pub owner_authority_digest: String,
    pub issued_at_micros: i64,
    pub supersedes_checkpoint_digest: Option<String>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CheckpointError {
    EmptyRequiredField,
    TextFieldTooLarge,
    TextFieldNotAscii,
    InvalidDigest,
    InvalidIssuerDid,
    EmptyEventFrontier,
    InvalidCheckpointSequence,
    InvalidSourceActionHash,
}

impl fmt::Display for CheckpointError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{self:?}")
    }
}

impl std::error::Error for CheckpointError {}

impl OwnerCheckpointSubjectV1 {
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, CheckpointError> {
        self.validate()?;

        let mut bytes = Vec::with_capacity(512 + self.source_action_hash.len());
        bytes.extend_from_slice(CHECKPOINT_DOMAIN);
        bytes.push(self.owner_domain as u8);
        append_text(&mut bytes, &self.operation_id)?;
        append_digest(&mut bytes, &self.intent_digest)?;
        append_digest(&mut bytes, &self.event_log_digest)?;
        bytes.extend_from_slice(&self.event_count.to_be_bytes());
        append_digest(&mut bytes, &self.tail_digest)?;
        bytes.extend_from_slice(&self.checkpoint_sequence.to_be_bytes());
        append_optional_digest(&mut bytes, self.prior_checkpoint_digest.as_deref())?;
        append_bytes(&mut bytes, &self.source_action_hash)?;
        append_text(&mut bytes, &self.issuer_did)?;
        bytes.push(KEY_PROFILE_HOLOCHAIN_ACTION_ED25519_V1);
        append_digest(&mut bytes, &self.issuer_key_fingerprint)?;
        append_digest(&mut bytes, &self.owner_authority_digest)?;
        bytes.extend_from_slice(&self.issued_at_micros.to_be_bytes());
        append_optional_digest(
            &mut bytes,
            self.supersedes_checkpoint_digest.as_deref(),
        )?;
        Ok(bytes)
    }

    pub fn digest(&self) -> Result<String, CheckpointError> {
        let bytes = self.canonical_bytes()?;
        let hash = Sha256::digest(bytes);
        Ok(format!("sha256:{}", hex_lower(&hash)))
    }

    pub fn validate(&self) -> Result<(), CheckpointError> {
        require_text(&self.operation_id)?;
        validate_digest(&self.intent_digest)?;
        validate_digest(&self.event_log_digest)?;
        validate_digest(&self.tail_digest)?;
        validate_digest(&self.issuer_key_fingerprint)?;
        validate_digest(&self.owner_authority_digest)?;

        if self.event_count == 0 {
            return Err(CheckpointError::EmptyEventFrontier);
        }
        if self.checkpoint_sequence == 0 {
            return Err(CheckpointError::InvalidCheckpointSequence);
        }
        match (self.checkpoint_sequence, self.prior_checkpoint_digest.as_deref()) {
            (1, None) => {}
            (1, Some(_)) | (_, None) => {
                return Err(CheckpointError::InvalidCheckpointSequence);
            }
            (_, Some(digest)) => validate_digest(digest)?,
        }

        if self.source_action_hash.is_empty()
            || self.source_action_hash.len() > MAX_SOURCE_ACTION_HASH_BYTES
        {
            return Err(CheckpointError::InvalidSourceActionHash);
        }
        if !self.issuer_did.starts_with("did:mycelix:")
            || self.issuer_did.trim() != self.issuer_did
            || self.issuer_did.len() > MAX_TEXT_BYTES
        {
            return Err(CheckpointError::InvalidIssuerDid);
        }
        if let Some(digest) = self.supersedes_checkpoint_digest.as_deref() {
            validate_digest(digest)?;
        }
        Ok(())
    }
}

fn require_text(value: &str) -> Result<(), CheckpointError> {
    if value.is_empty() {
        return Err(CheckpointError::EmptyRequiredField);
    }
    if value.len() > MAX_TEXT_BYTES || !value.is_ascii() {
        return Err(CheckpointError::TextFieldTooLarge);
    }
    Ok(())
}

fn validate_digest(value: &str) -> Result<(), CheckpointError> {
    let Some(hex) = value.strip_prefix("sha256:") else {
        return Err(CheckpointError::InvalidDigest);
    };
    if hex.len() != 64
        || !hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(CheckpointError::InvalidDigest);
    }
    Ok(())
}

fn append_text(output: &mut Vec<u8>, value: &str) -> Result<(), CheckpointError> {
    require_text(value)?;
    let length = u32::try_from(value.len()).map_err(|_| CheckpointError::TextFieldTooLarge)?;
    output.extend_from_slice(&length.to_be_bytes());
    output.extend_from_slice(value.as_bytes());
    Ok(())
}

fn append_digest(output: &mut Vec<u8>, value: &str) -> Result<(), CheckpointError> {
    validate_digest(value)?;
    append_text(output, value)
}

fn append_optional_digest(
    output: &mut Vec<u8>,
    value: Option<&str>,
) -> Result<(), CheckpointError> {
    match value {
        None => output.push(0),
        Some(digest) => {
            output.push(1);
            append_digest(output, digest)?;
        }
    }
    Ok(())
}

fn append_bytes(output: &mut Vec<u8>, value: &[u8]) -> Result<(), CheckpointError> {
    if value.len() > MAX_SOURCE_ACTION_HASH_BYTES {
        return Err(CheckpointError::InvalidSourceActionHash);
    }
    let length = u32::try_from(value.len()).map_err(|_| CheckpointError::InvalidSourceActionHash)?;
    output.extend_from_slice(&length.to_be_bytes());
    output.extend_from_slice(value);
    Ok(())
}

fn hex_lower(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_subject() -> OwnerCheckpointSubjectV1 {
        OwnerCheckpointSubjectV1 {
            operation_id: "payment-op:v1:alice-01".into(),
            intent_digest: format!("sha256:{}", "a".repeat(64)),
            owner_domain: CheckpointOwnerDomain::PaymentsLocalTransaction,
            event_log_digest: format!("sha256:{}", "b".repeat(64)),
            event_count: 8,
            tail_digest: format!("sha256:{}", "e".repeat(64)),
            checkpoint_sequence: 1,
            prior_checkpoint_digest: None,
            source_action_hash: (0_u8..39).collect(),
            issuer_did: "did:mycelix:uhCAkAlice".into(),
            issuer_key_fingerprint: format!("sha256:{}", "c".repeat(64)),
            owner_authority_digest: format!("sha256:{}", "d".repeat(64)),
            issued_at_micros: 1_794_300_000_000_000,
            supersedes_checkpoint_digest: None,
        }
    }

    #[test]
    fn canonical_checkpoint_subject_is_stable_and_domain_separated() {
        let subject = valid_subject();
        let first = subject.canonical_bytes().unwrap();
        let second = subject.canonical_bytes().unwrap();
        assert_eq!(first, second);
        assert!(first.starts_with(CHECKPOINT_DOMAIN));
        assert_eq!(
            subject.digest().unwrap(),
            "sha256:69e8cd918ccdda9a080af4a48bfd4d0e65b684d8ab584f710926afd673a86e60"
        );
    }

    #[test]
    fn owner_domain_changes_the_canonical_subject() {
        let first = valid_subject();
        let mut second = first.clone();
        second.owner_domain = CheckpointOwnerDomain::TreasuryFeeEffect;
        assert_ne!(first.canonical_bytes().unwrap(), second.canonical_bytes().unwrap());
        assert_ne!(first.digest().unwrap(), second.digest().unwrap());
    }

    #[test]
    fn later_checkpoint_requires_a_prior_owner_checkpoint_digest() {
        let mut subject = valid_subject();
        subject.checkpoint_sequence = 2;
        assert_eq!(
            subject.validate(),
            Err(CheckpointError::InvalidCheckpointSequence)
        );
        subject.prior_checkpoint_digest = Some(format!("sha256:{}", "f".repeat(64)));
        assert!(subject.validate().is_ok());
    }

    #[test]
    fn genesis_checkpoint_cannot_claim_a_prior_checkpoint() {
        let mut subject = valid_subject();
        subject.prior_checkpoint_digest = Some(format!("sha256:{}", "f".repeat(64)));
        assert_eq!(
            subject.validate(),
            Err(CheckpointError::InvalidCheckpointSequence)
        );
    }

    #[test]
    fn malformed_digest_and_untrusted_issuer_form_are_rejected() {
        let mut subject = valid_subject();
        subject.intent_digest = format!("sha256:{}", "A".repeat(64));
        assert_eq!(subject.validate(), Err(CheckpointError::InvalidDigest));

        let mut subject = valid_subject();
        subject.issuer_did = "did:example:alice".into();
        assert_eq!(subject.validate(), Err(CheckpointError::InvalidIssuerDid));
    }

    #[test]
    fn checkpoint_requires_a_bounded_source_action_hash() {
        let mut subject = valid_subject();
        subject.source_action_hash.clear();
        assert_eq!(
            subject.validate(),
            Err(CheckpointError::InvalidSourceActionHash)
        );
        subject.source_action_hash = vec![0; MAX_SOURCE_ACTION_HASH_BYTES + 1];
        assert_eq!(
            subject.validate(),
            Err(CheckpointError::InvalidSourceActionHash)
        );
    }

    #[test]
    fn correction_reference_must_be_a_canonical_digest() {
        let mut subject = valid_subject();
        subject.supersedes_checkpoint_digest = Some("not-a-digest".into());
        assert_eq!(subject.validate(), Err(CheckpointError::InvalidDigest));
    }
}
