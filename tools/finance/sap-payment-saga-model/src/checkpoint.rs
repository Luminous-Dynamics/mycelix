//! Canonical owner-scoped checkpoint subjects for SAP payment journal replay.
//!
//! This is a deterministic reference profile. The source action hash, issuer and
//! authority-profile digest are bound as subject data; this module does not
//! resolve Holochain actions, verify the action author, or authorize the issuer.
//! A production Holochain Create action is the native authorship attestation.

use super::{PaymentIntent, PaymentJournal};
use sha2::{Digest, Sha256};

const INTENT_DOMAIN: &[u8] = b"MXSAP-PAYMENT-INTENT-V1\0";
const CHECKPOINT_DOMAIN: &[u8] = b"MXSAP-JOURNAL-CHECKPOINT-V1\0";
const MAX_TEXT_BYTES: usize = 16 * 1024;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CheckpointOwner {
    Payments,
    Treasury,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CheckpointLifecycle {
    Current,
    Supersedes,
    Retracts,
}

/// One owner’s attestation subject over a specific prefix of its own journal.
/// Payments and Treasury must publish separate subjects; this type intentionally
/// does not invent a single total order across their source chains.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct JournalCheckpointSubjectV1 {
    pub owner: CheckpointOwner,
    pub lifecycle: CheckpointLifecycle,
    pub operation_id: String,
    pub intent_digest: String,
    pub event_log_id: String,
    pub event_count: u64,
    pub tail_digest: String,
    pub checkpoint_sequence: u64,
    pub previous_checkpoint_digest: Option<String>,
    pub source_action_hash: String,
    pub issuer_did: String,
    pub verification_method: String,
    pub authority_profile_digest: String,
    pub issued_at_unix_micros: u64,
    pub supersedes_checkpoint_digest: Option<String>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CheckpointError {
    InvalidIntent,
    OperationIdentityMismatch,
    IntentDigestMismatch,
    InvalidOwner,
    InvalidLifecycle,
    InvalidSequence,
    InvalidDigest,
    InvalidField,
    MissingPreviousCheckpoint,
    UnexpectedPreviousCheckpoint,
    MissingSupersededCheckpoint,
    UnexpectedSupersededCheckpoint,
}

impl JournalCheckpointSubjectV1 {
    /// Computes the exact versioned subject digest after binding it to the
    /// supplied immutable intent. The returned digest is not an authorship proof.
    pub fn canonical_digest(
        &self,
        intent: &PaymentIntent,
    ) -> Result<String, CheckpointError> {
        let bytes = self.canonical_bytes(intent)?;
        Ok(digest_hex(&bytes))
    }

    /// Exposed for independent implementations and frozen known-answer tests.
    pub fn canonical_bytes(
        &self,
        intent: &PaymentIntent,
    ) -> Result<Vec<u8>, CheckpointError> {
        intent.validate().map_err(|_| CheckpointError::InvalidIntent)?;
        let expected_operation = PaymentJournal::new(intent.clone()).operation_id().to_owned();
        if self.operation_id != expected_operation {
            return Err(CheckpointError::OperationIdentityMismatch);
        }
        let expected_intent_digest = canonical_intent_digest(intent)?;
        if self.intent_digest != expected_intent_digest {
            return Err(CheckpointError::IntentDigestMismatch);
        }
        if !is_sha256_digest(&self.tail_digest)
            || !is_sha256_digest(&self.intent_digest)
            || !is_sha256_digest(&self.authority_profile_digest)
        {
            return Err(CheckpointError::InvalidDigest);
        }
        if self.event_count == 0 || self.checkpoint_sequence == 0 {
            return Err(CheckpointError::InvalidSequence);
        }
        if self.checkpoint_sequence == 1 {
            if self.previous_checkpoint_digest.is_some() {
                return Err(CheckpointError::UnexpectedPreviousCheckpoint);
            }
        } else {
            let previous = self
                .previous_checkpoint_digest
                .as_deref()
                .ok_or(CheckpointError::MissingPreviousCheckpoint)?;
            if !is_sha256_digest(previous) {
                return Err(CheckpointError::InvalidDigest);
            }
        }
        match self.lifecycle {
            CheckpointLifecycle::Current => {
                if self.supersedes_checkpoint_digest.is_some() {
                    return Err(CheckpointError::UnexpectedSupersededCheckpoint);
                }
            }
            CheckpointLifecycle::Supersedes | CheckpointLifecycle::Retracts => {
                let superseded = self
                    .supersedes_checkpoint_digest
                    .as_deref()
                    .ok_or(CheckpointError::MissingSupersededCheckpoint)?;
                if !is_sha256_digest(superseded) {
                    return Err(CheckpointError::InvalidDigest);
                }
            }
        }
        if self.issued_at_unix_micros == 0 {
            return Err(CheckpointError::InvalidField);
        }

        let mut output = Vec::with_capacity(512);
        output.extend_from_slice(CHECKPOINT_DOMAIN);
        output.push(1); // profile version
        output.push(owner_tag(self.owner));
        output.push(lifecycle_tag(self.lifecycle));
        append_text(&mut output, &self.operation_id)?;
        append_text(&mut output, &self.intent_digest)?;
        append_text(&mut output, &self.event_log_id)?;
        output.extend_from_slice(&self.event_count.to_be_bytes());
        append_text(&mut output, &self.tail_digest)?;
        output.extend_from_slice(&self.checkpoint_sequence.to_be_bytes());
        append_optional_text(&mut output, self.previous_checkpoint_digest.as_deref())?;
        append_text(&mut output, &self.source_action_hash)?;
        append_text(&mut output, &self.issuer_did)?;
        append_text(&mut output, &self.verification_method)?;
        append_text(&mut output, &self.authority_profile_digest)?;
        output.extend_from_slice(&self.issued_at_unix_micros.to_be_bytes());
        append_optional_text(&mut output, self.supersedes_checkpoint_digest.as_deref())?;
        Ok(output)
    }
}

/// Canonical digest of the immutable operation intent. Amounts are u64
/// big-endian minor units; strings are exact UTF-8 with u32 big-endian lengths.
pub fn canonical_intent_digest(intent: &PaymentIntent) -> Result<String, CheckpointError> {
    intent.validate().map_err(|_| CheckpointError::InvalidIntent)?;
    let mut output = Vec::with_capacity(256);
    output.extend_from_slice(INTENT_DOMAIN);
    output.push(1);
    append_text(&mut output, &intent.sender_did)?;
    append_text(&mut output, &intent.recipient_did)?;
    append_text(&mut output, &intent.client_idempotency_key)?;
    append_text(&mut output, &intent.payload_fingerprint)?;
    output.extend_from_slice(&intent.amount_minor.to_be_bytes());
    output.extend_from_slice(&intent.fee_minor.to_be_bytes());
    append_text(&mut output, &intent.currency)?;
    Ok(digest_hex(&output))
}

fn append_text(output: &mut Vec<u8>, value: &str) -> Result<(), CheckpointError> {
    if value.is_empty() || value.len() > MAX_TEXT_BYTES {
        return Err(CheckpointError::InvalidField);
    }
    let length = u32::try_from(value.len()).map_err(|_| CheckpointError::InvalidField)?;
    output.extend_from_slice(&length.to_be_bytes());
    output.extend_from_slice(value.as_bytes());
    Ok(())
}

fn append_optional_text(
    output: &mut Vec<u8>,
    value: Option<&str>,
) -> Result<(), CheckpointError> {
    match value {
        None => output.push(0),
        Some(value) => {
            output.push(1);
            append_text(output, value)?;
        }
    }
    Ok(())
}

fn owner_tag(owner: CheckpointOwner) -> u8 {
    match owner {
        CheckpointOwner::Payments => 1,
        CheckpointOwner::Treasury => 2,
    }
}

fn lifecycle_tag(lifecycle: CheckpointLifecycle) -> u8 {
    match lifecycle {
        CheckpointLifecycle::Current => 1,
        CheckpointLifecycle::Supersedes => 2,
        CheckpointLifecycle::Retracts => 3,
    }
}

fn is_sha256_digest(value: &str) -> bool {
    let Some(hex) = value.strip_prefix("sha256:") else {
        return false;
    };
    hex.len() == 64
        && hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn digest_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(71);
    output.push_str("sha256:");
    for byte in digest {
        use std::fmt::Write as _;
        write!(&mut output, "{byte:02x}").expect("writing to String cannot fail");
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    fn intent() -> PaymentIntent {
        PaymentIntent::new(
            "did:sender",
            "did:recipient",
            "client-key-001",
            format!("sha256:{}", "a".repeat(64)),
            100,
            3,
            "SAP",
        )
    }

    fn subject() -> JournalCheckpointSubjectV1 {
        JournalCheckpointSubjectV1 {
            owner: CheckpointOwner::Payments,
            lifecycle: CheckpointLifecycle::Current,
            operation_id: "sap-payment:10:did:sender:14:client-key-001".into(),
            intent_digest: canonical_intent_digest(&intent()).expect("intent digest"),
            event_log_id: "sap-payment-journal:payments:operation-1".into(),
            event_count: 4,
            tail_digest: "sha256:5c34988af074f1a2f8a387a16e58d1121bc888426b2d5989cd8904f7f279684f".into(),
            checkpoint_sequence: 1,
            previous_checkpoint_digest: None,
            source_action_hash: "uhCkkPaymentsActionHashExample".into(),
            issuer_did: "did:mycelix:payments-operator".into(),
            verification_method: "did:mycelix:payments-operator#agent-key-1".into(),
            authority_profile_digest: format!("sha256:{}", "c".repeat(64)),
            issued_at_unix_micros: 1_794_326_400_000_000,
            supersedes_checkpoint_digest: None,
        }
    }

    fn vector(name: &str) -> &'static str {
        include_str!("../../sap-payment-saga-go-verifier/fixtures/sap-payment-journal-checkpoint-v1.kat")
            .lines()
            .find_map(|line| line.strip_prefix(name).and_then(|value| value.strip_prefix('=')))
            .expect("checkpoint known-answer vector exists")
    }

    #[test]
    fn canonical_intent_and_checkpoint_match_cross_language_vectors() {
        let intent = intent();
        assert_eq!(
            canonical_intent_digest(&intent).expect("intent digest"),
            vector("intent-digest")
        );
        assert_eq!(
            subject().canonical_digest(&intent).expect("checkpoint digest"),
            vector("payments-checkpoint-digest")
        );
    }

    #[test]
    fn owner_domain_is_bound_into_checkpoint_digest() {
        let intent = intent();
        let original = subject().canonical_digest(&intent).expect("digest");
        let mut treasury = subject();
        treasury.owner = CheckpointOwner::Treasury;
        assert_ne!(treasury.canonical_digest(&intent).expect("digest"), original);
    }

    #[test]
    fn rejects_wrong_operation_or_intent_binding() {
        let intent = intent();
        let mut changed = subject();
        changed.operation_id.push_str("-other");
        assert_eq!(
            changed.canonical_digest(&intent),
            Err(CheckpointError::OperationIdentityMismatch)
        );
        let mut changed = subject();
        changed.intent_digest = format!("sha256:{}", "d".repeat(64));
        assert_eq!(
            changed.canonical_digest(&intent),
            Err(CheckpointError::IntentDigestMismatch)
        );
    }

    #[test]
    fn enforces_owner_checkpoint_sequence_and_previous_digest() {
        let intent = intent();
        let mut second = subject();
        second.checkpoint_sequence = 2;
        assert_eq!(
            second.canonical_digest(&intent),
            Err(CheckpointError::MissingPreviousCheckpoint)
        );
        second.previous_checkpoint_digest = Some(format!("sha256:{}", "e".repeat(64)));
        assert!(second.canonical_digest(&intent).is_ok());
        second.previous_checkpoint_digest = Some("sha256:bad".into());
        assert_eq!(
            second.canonical_digest(&intent),
            Err(CheckpointError::InvalidDigest)
        );
    }

    #[test]
    fn enforces_correction_and_retraction_links() {
        let intent = intent();
        let mut superseding = subject();
        superseding.lifecycle = CheckpointLifecycle::Supersedes;
        assert_eq!(
            superseding.canonical_digest(&intent),
            Err(CheckpointError::MissingSupersededCheckpoint)
        );
        superseding.supersedes_checkpoint_digest = Some(format!("sha256:{}", "f".repeat(64)));
        assert!(superseding.canonical_digest(&intent).is_ok());
        superseding.lifecycle = CheckpointLifecycle::Current;
        assert_eq!(
            superseding.canonical_digest(&intent),
            Err(CheckpointError::UnexpectedSupersededCheckpoint)
        );
    }

    #[test]
    fn binds_each_checkpoint_subject_field_into_digest() {
        let intent = intent();
        let original = subject().canonical_digest(&intent).expect("digest");
        let mut changed = subject();
        changed.issuer_did.push_str("-changed");
        assert_ne!(changed.canonical_digest(&intent).expect("digest"), original);
        let mut changed = subject();
        changed.source_action_hash.push_str("-changed");
        assert_ne!(changed.canonical_digest(&intent).expect("digest"), original);
        let mut changed = subject();
        changed.event_count += 1;
        assert_ne!(changed.canonical_digest(&intent).expect("digest"), original);
    }
}
