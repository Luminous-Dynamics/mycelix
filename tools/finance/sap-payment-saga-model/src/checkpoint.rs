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
    pub owner_authority_action_hash: Vec<u8>,
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
    InvalidJournalIdentity,
    InvalidIssuerDid,
    EmptyEventFrontier,
    InvalidCheckpointSequence,
    InvalidSourceActionHash,
    InvalidAuthorityActionHash,
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
        append_action_hash(&mut bytes, &self.owner_authority_action_hash)?;
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
        if self.event_log_digest
            != journal_identity_digest(&self.operation_id, &self.intent_digest)?
        {
            return Err(CheckpointError::InvalidJournalIdentity);
        }
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
        if self.owner_authority_action_hash.is_empty()
            || self.owner_authority_action_hash.len() > MAX_SOURCE_ACTION_HASH_BYTES
        {
            return Err(CheckpointError::InvalidAuthorityActionHash);
        }
        if require_text(&self.issuer_did).is_err()
            || !self.issuer_did.starts_with("did:mycelix:")
            || self.issuer_did.trim() != self.issuer_did
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
    if value.len() > MAX_TEXT_BYTES {
        return Err(CheckpointError::TextFieldTooLarge);
    }
    if !value.is_ascii() {
        return Err(CheckpointError::TextFieldNotAscii);
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

fn journal_identity_digest(
    operation_id: &str,
    intent_digest: &str,
) -> Result<String, CheckpointError> {
    let mut bytes = b"MXSAP-JOURNAL-IDENTITY-V1\0".to_vec();
    append_text(&mut bytes, operation_id)?;
    append_digest(&mut bytes, intent_digest)?;
    let digest = Sha256::digest(bytes);
    Ok(format!("sha256:{}", hex_lower(&digest)))
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

fn append_action_hash(output: &mut Vec<u8>, value: &[u8]) -> Result<(), CheckpointError> {
    if value.is_empty() || value.len() > MAX_SOURCE_ACTION_HASH_BYTES {
        return Err(CheckpointError::InvalidAuthorityActionHash);
    }
    let length =
        u32::try_from(value.len()).map_err(|_| CheckpointError::InvalidAuthorityActionHash)?;
    output.extend_from_slice(&length.to_be_bytes());
    output.extend_from_slice(value);
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

    const CHECKPOINT_FIXTURE: &str =
        include_str!("../fixtures/sap-payment-owner-checkpoint-v1.kat");

    fn fixture_field(name: &str) -> &str {
        CHECKPOINT_FIXTURE
            .lines()
            .find_map(|line| {
                let (key, value) = line.split_once('=')?;
                (key == name).then_some(value)
            })
            .unwrap_or_else(|| panic!("missing checkpoint fixture field: {name}"))
    }

    fn decode_hex(value: &str) -> Vec<u8> {
        assert_eq!(value.len() % 2, 0);
        value
            .as_bytes()
            .chunks_exact(2)
            .map(|pair| {
                let hex = std::str::from_utf8(pair).unwrap();
                u8::from_str_radix(hex, 16).unwrap()
            })
            .collect()
    }

    fn optional_digest(field: &str) -> Option<String> {
        match fixture_field(field) {
            "-" => None,
            value => Some(value.to_owned()),
        }
    }

    fn valid_subject() -> OwnerCheckpointSubjectV1 {
        let owner_domain = match fixture_field("owner_domain") {
            "payments_local_transaction" => CheckpointOwnerDomain::PaymentsLocalTransaction,
            "treasury_fee_effect" => CheckpointOwnerDomain::TreasuryFeeEffect,
            other => panic!("unknown checkpoint owner domain: {other}"),
        };
        OwnerCheckpointSubjectV1 {
            operation_id: fixture_field("operation_id").into(),
            intent_digest: fixture_field("intent_digest").into(),
            owner_domain,
            event_log_digest: fixture_field("event_log_digest").into(),
            event_count: fixture_field("event_count").parse().unwrap(),
            tail_digest: fixture_field("tail_digest").into(),
            checkpoint_sequence: fixture_field("checkpoint_sequence").parse().unwrap(),
            prior_checkpoint_digest: optional_digest("prior_checkpoint_digest"),
            source_action_hash: decode_hex(fixture_field("source_action_hash_hex")),
            owner_authority_action_hash: decode_hex(fixture_field("owner_authority_action_hash_hex")),
            issuer_did: fixture_field("issuer_did").into(),
            issuer_key_fingerprint: fixture_field("issuer_key_fingerprint").into(),
            owner_authority_digest: fixture_field("owner_authority_digest").into(),
            issued_at_micros: fixture_field("issued_at_micros").parse().unwrap(),
            supersedes_checkpoint_digest: optional_digest("supersedes_checkpoint_digest"),
        }
    }

    #[test]
    fn canonical_checkpoint_subject_is_stable_and_domain_separated() {
        let subject = valid_subject();
        let first = subject.canonical_bytes().unwrap();
        let second = subject.canonical_bytes().unwrap();
        assert_eq!(first, second);
        assert!(first.starts_with(CHECKPOINT_DOMAIN));
        assert_eq!(hex_lower(&first), fixture_field("canonical_hex"));
        assert_eq!(subject.digest().unwrap(), fixture_field("digest"));
    }

    #[test]
    fn journal_identity_is_derived_from_operation_and_intent() {
        let subject = valid_subject();
        let mut changed = subject.clone();
        changed.operation_id.push_str("-other");
        assert_eq!(
            changed.validate(),
            Err(CheckpointError::InvalidJournalIdentity)
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
    fn checkpoint_requires_a_bounded_owner_authority_action_hash() {
        let mut subject = valid_subject();
        subject.owner_authority_action_hash.clear();
        assert_eq!(
            subject.validate(),
            Err(CheckpointError::InvalidAuthorityActionHash)
        );
        subject.owner_authority_action_hash = vec![0; MAX_SOURCE_ACTION_HASH_BYTES + 1];
        assert_eq!(
            subject.validate(),
            Err(CheckpointError::InvalidAuthorityActionHash)
        );
    }

    #[test]
    fn correction_reference_must_be_a_canonical_digest() {
        let mut subject = valid_subject();
        subject.supersedes_checkpoint_digest = Some("not-a-digest".into());
        assert_eq!(subject.validate(), Err(CheckpointError::InvalidDigest));
    }
}
