//! Canonical, owner-scoped journal checkpoint subjects.
//!
//! This module is an effects-disabled reference profile. A checkpoint stored in
//! the same Holochain DNA should be committed by a native Create action. This
//! digest identifies the exact subject; it is not an authorship signature,
//! owner authorization proof, source-action proof, or settlement receipt.

use sha2::{Digest, Sha256};

const DOMAIN: &[u8] = b"MXSAP-JOURNAL-CHECKPOINT-SUBJECT-V1\0";
const MAX_FIELD_BYTES: usize = 4096;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum CheckpointOwner {
    Payments = 1,
    Treasury = 2,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CheckpointSubjectV1 {
    pub operation_id: String,
    pub intent_digest: String,
    pub owner: CheckpointOwner,
    pub event_log_id: String,
    pub event_count: u64,
    pub tail_digest: String,
    pub checkpoint_sequence: u64,
    pub prior_checkpoint_digest: Option<String>,
    pub source_action_hash: String,
    pub source_chain_frontier: String,
    pub issuer_did: String,
    pub verification_method: String,
    pub authority_grant_ref: String,
    pub issued_at_unix_ms: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CheckpointError {
    MissingField,
    InvalidDigest,
    InvalidSequence,
    InvalidTimestamp,
    OversizedField,
}

impl CheckpointSubjectV1 {
    pub fn validate(&self) -> Result<(), CheckpointError> {
        for value in [
            self.operation_id.as_str(),
            self.event_log_id.as_str(),
            self.source_action_hash.as_str(),
            self.source_chain_frontier.as_str(),
            self.issuer_did.as_str(),
            self.verification_method.as_str(),
            self.authority_grant_ref.as_str(),
        ] {
            if value.is_empty() {
                return Err(CheckpointError::MissingField);
            }
            if value.len() > MAX_FIELD_BYTES {
                return Err(CheckpointError::OversizedField);
            }
        }

        if !valid_digest(&self.intent_digest) || !valid_digest(&self.tail_digest) {
            return Err(CheckpointError::InvalidDigest);
        }
        if let Some(prior) = &self.prior_checkpoint_digest {
            if !valid_digest(prior) {
                return Err(CheckpointError::InvalidDigest);
            }
        }

        if self.event_count == 0 || self.checkpoint_sequence == 0 {
            return Err(CheckpointError::InvalidSequence);
        }
        match (self.checkpoint_sequence, self.prior_checkpoint_digest.is_some()) {
            (1, false) => {}
            (1, true) | (_, false) => return Err(CheckpointError::InvalidSequence),
            (_, true) => {}
        }
        if self.issued_at_unix_ms == 0 {
            return Err(CheckpointError::InvalidTimestamp);
        }
        Ok(())
    }

    /// Returns the exact versioned subject bytes used by the shared Rust/Go
    /// known-answer profile. No generic serializer is involved.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, CheckpointError> {
        self.validate()?;
        let mut bytes = Vec::with_capacity(512);
        bytes.extend_from_slice(DOMAIN);
        append_text(&mut bytes, &self.operation_id)?;
        append_text(&mut bytes, &self.intent_digest)?;
        bytes.push(self.owner as u8);
        append_text(&mut bytes, &self.event_log_id)?;
        bytes.extend_from_slice(&self.event_count.to_be_bytes());
        append_text(&mut bytes, &self.tail_digest)?;
        bytes.extend_from_slice(&self.checkpoint_sequence.to_be_bytes());
        match &self.prior_checkpoint_digest {
            Some(prior) => {
                bytes.push(1);
                append_text(&mut bytes, prior)?;
            }
            None => bytes.push(0),
        }
        append_text(&mut bytes, &self.source_action_hash)?;
        append_text(&mut bytes, &self.source_chain_frontier)?;
        append_text(&mut bytes, &self.issuer_did)?;
        append_text(&mut bytes, &self.verification_method)?;
        append_text(&mut bytes, &self.authority_grant_ref)?;
        bytes.extend_from_slice(&self.issued_at_unix_ms.to_be_bytes());
        Ok(bytes)
    }

    pub fn canonical_digest(&self) -> Result<String, CheckpointError> {
        let bytes = self.canonical_bytes()?;
        let digest = Sha256::digest(&bytes);
        let mut output = String::with_capacity(71);
        output.push_str("sha256:");
        for byte in digest {
            use std::fmt::Write as _;
            write!(&mut output, "{byte:02x}").expect("writing to String cannot fail");
        }
        Ok(output)
    }
}

fn append_text(output: &mut Vec<u8>, value: &str) -> Result<(), CheckpointError> {
    let encoded = value.as_bytes();
    if encoded.len() > MAX_FIELD_BYTES {
        return Err(CheckpointError::OversizedField);
    }
    let length =
        u32::try_from(encoded.len()).map_err(|_| CheckpointError::OversizedField)?;
    output.extend_from_slice(&length.to_be_bytes());
    output.extend_from_slice(encoded);
    Ok(())
}

fn valid_digest(value: &str) -> bool {
    value.len() == 71
        && value.starts_with("sha256:")
        && value[7..]
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    fn subject() -> CheckpointSubjectV1 {
        CheckpointSubjectV1 {
            operation_id: "sap-payment:10:did:sender:14:client-key-001".into(),
            intent_digest: format!("sha256:{}", "11".repeat(32)),
            owner: CheckpointOwner::Payments,
            event_log_id: "journal:did:sender:payment:001".into(),
            event_count: 8,
            tail_digest: "sha256:a1e5b966be2b6aab453c65370639d702e6d83b3097c9a542a0d85d28bf6ed899"
                .into(),
            checkpoint_sequence: 1,
            prior_checkpoint_digest: None,
            source_action_hash: "uhCkk-source-payment-action".into(),
            source_chain_frontier: "uhCkk-source-chain-frontier".into(),
            issuer_did: "did:mycelix:agent:sender".into(),
            verification_method: "did:mycelix:agent:sender#holochain-key-1".into(),
            authority_grant_ref: "uhCkk-payments-authority-grant".into(),
            issued_at_unix_ms: 1_794_316_800_000,
        }
    }

    fn fixture() -> String {
        let root = env!("CARGO_MANIFEST_DIR");
        let path = std::path::Path::new(root)
            .join("../sap-payment-saga-go-verifier/fixtures/sap-payment-journal-checkpoint-v1.kat");
        fs::read_to_string(path).expect("shared checkpoint fixture must be readable")
    }

    #[test]
    fn canonical_subject_matches_shared_known_answer_bytes_and_digest() {
        let mut vectors = std::collections::BTreeMap::new();
        for line in fixture().lines() {
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let (key, value) = line.split_once('=').expect("fixture line must contain =");
            assert!(vectors.insert(key, value).is_none(), "duplicate fixture key: {key}");
        }
        assert_eq!(vectors.len(), 3, "fixture must contain exactly three vectors");
        assert_eq!(
            vectors.get("profile"),
            Some(&"MXSAP-JOURNAL-CHECKPOINT-SUBJECT-V1")
        );

        let subject = subject();
        let bytes_hex: String = subject
            .canonical_bytes()
            .expect("canonical bytes")
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect();
        assert_eq!(
            Some(bytes_hex.as_str()),
            vectors.get("subject-bytes-hex").copied()
        );
        assert_eq!(
            Some(subject.canonical_digest().expect("canonical digest").as_str()),
            vectors.get("subject-digest").copied()
        );
    }

    #[test]
    fn every_subject_dimension_changes_the_digest() {
        let original = subject().canonical_digest().expect("valid digest");
        let mut cases = Vec::new();

        let mut value = subject();
        value.operation_id.push('x');
        cases.push(value);
        let mut value = subject();
        value.intent_digest = format!("sha256:{}", "22".repeat(32));
        cases.push(value);
        let mut value = subject();
        value.owner = CheckpointOwner::Treasury;
        cases.push(value);
        let mut value = subject();
        value.event_log_id.push('x');
        cases.push(value);
        let mut value = subject();
        value.event_count += 1;
        cases.push(value);
        let mut value = subject();
        value.tail_digest = format!("sha256:{}", "22".repeat(32));
        cases.push(value);
        let mut value = subject();
        value.checkpoint_sequence = 2;
        value.prior_checkpoint_digest = Some(format!("sha256:{}", "33".repeat(32)));
        cases.push(value);
        let mut value = subject();
        value.checkpoint_sequence = 2;
        value.prior_checkpoint_digest = Some(format!("sha256:{}", "44".repeat(32)));
        cases.push(value);
        let mut value = subject();
        value.source_action_hash.push('x');
        cases.push(value);
        let mut value = subject();
        value.source_chain_frontier.push('x');
        cases.push(value);
        let mut value = subject();
        value.issuer_did.push('x');
        cases.push(value);
        let mut value = subject();
        value.verification_method.push('x');
        cases.push(value);
        let mut value = subject();
        value.authority_grant_ref.push('x');
        cases.push(value);
        let mut value = subject();
        value.issued_at_unix_ms += 1;
        cases.push(value);

        for (index, changed) in cases.iter().enumerate() {
            assert_ne!(
                changed.canonical_digest().expect("mutated valid subject"),
                original,
                "subject mutation {index} must change its digest"
            );
        }
    }

    #[test]
    fn invalid_subjects_fail_closed() {
        let mut value = subject();
        value.intent_digest = "sha256:ABC".into();
        assert_eq!(value.validate(), Err(CheckpointError::InvalidDigest));

        let mut value = subject();
        value.checkpoint_sequence = 2;
        assert_eq!(value.validate(), Err(CheckpointError::InvalidSequence));

        let mut value = subject();
        value.prior_checkpoint_digest = Some(format!("sha256:{}", "22".repeat(32)));
        assert_eq!(value.validate(), Err(CheckpointError::InvalidSequence));

        let mut value = subject();
        value.event_count = 0;
        assert_eq!(value.validate(), Err(CheckpointError::InvalidSequence));

        let mut value = subject();
        value.issuer_did.clear();
        assert_eq!(value.validate(), Err(CheckpointError::MissingField));

        let mut value = subject();
        value.source_action_hash = "x".repeat(MAX_FIELD_BYTES + 1);
        assert_eq!(value.validate(), Err(CheckpointError::OversizedField));
    }
}
