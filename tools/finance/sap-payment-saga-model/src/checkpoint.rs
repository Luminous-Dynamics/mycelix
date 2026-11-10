//! Canonical, owner-scoped checkpoint subjects for SAP payment journal replay.
//!
//! This is an effects-disabled reference profile. A valid digest proves only that
//! these exact bytes were encoded under this profile. It does not authenticate
//! the Holochain action, establish owner authorization, or prove settlement.

use sha2::{Digest, Sha256};

use super::{PaymentIntent, PaymentJournal};

const CHECKPOINT_DOMAIN: &[u8] = b"MXSAP-JOURNAL-CHECKPOINT-V1\0";
const INTENT_DOMAIN: &[u8] = b"MXSAP-PAYMENT-INTENT-V1\0";
const MAX_CHECKPOINT_TEXT_BYTES: usize = 4096;
const ACTION_HASH_BYTES: usize = 39;
const ACTION_HASH_PREFIX: [u8; 3] = [0x84, 0x29, 0x24];

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum CheckpointOwnerDomain {
    Payments = 1,
    Treasury = 2,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct JournalCheckpointSubject {
    pub owner_domain: CheckpointOwnerDomain,
    pub operation_id: String,
    pub intent_digest: String,
    pub owner_did: String,
    pub event_log_id: String,
    pub event_count: u64,
    pub tail_digest: String,
    pub checkpoint_sequence: u64,
    pub previous_checkpoint_digest: Option<String>,
    /// Raw 39-byte Holochain ActionHash (type prefix + core + DHT location).
    pub source_action_hash: Vec<u8>,
    pub source_action_sequence: u32,
    /// The source action's predecessor. None only for source-chain sequence zero.
    pub source_previous_action_hash: Option<Vec<u8>>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CheckpointError {
    InvalidIntent,
    InvalidSubject,
    InvalidDigest,
    InvalidActionHash,
    InvalidCheckpointSequence,
    InvalidSourceChainFrontier,
}

impl JournalCheckpointSubject {
    /// Returns the fixed-field canonical preimage for MXSAP-JOURNAL-CHECKPOINT-V1.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, CheckpointError> {
        self.validate()?;
        let mut output = Vec::with_capacity(512);
        output.extend_from_slice(CHECKPOINT_DOMAIN);
        output.push(self.owner_domain as u8);
        append_text(&mut output, &self.operation_id)?;
        append_text(&mut output, &self.intent_digest)?;
        append_text(&mut output, &self.owner_did)?;
        append_text(&mut output, &self.event_log_id)?;
        append_u64(&mut output, self.event_count);
        append_text(&mut output, &self.tail_digest)?;
        append_u64(&mut output, self.checkpoint_sequence);

        match &self.previous_checkpoint_digest {
            Some(digest) => {
                output.push(1);
                append_text(&mut output, digest)?;
            }
            None => output.push(0),
        }

        output.extend_from_slice(&self.source_action_hash);
        output.extend_from_slice(&self.source_action_sequence.to_be_bytes());
        match &self.source_previous_action_hash {
            Some(hash) => {
                output.push(1);
                output.extend_from_slice(hash);
            }
            None => output.push(0),
        }
        Ok(output)
    }

    pub fn canonical_digest(&self) -> Result<String, CheckpointError> {
        let digest = Sha256::digest(self.canonical_bytes()?);
        Ok(format_digest(&digest))
    }

    pub fn validate(&self) -> Result<(), CheckpointError> {
        for value in [&self.operation_id, &self.owner_did, &self.event_log_id] {
            if value.trim().is_empty()
                || value.contains('\0')
                || value.as_bytes().len() > MAX_CHECKPOINT_TEXT_BYTES
            {
                return Err(CheckpointError::InvalidSubject);
            }
        }
        if !self.owner_did.starts_with("did:") {
            return Err(CheckpointError::InvalidSubject);
        }
        validate_digest(&self.intent_digest)?;
        validate_digest(&self.tail_digest)?;

        if self.event_count == 0 || self.checkpoint_sequence == 0 {
            return Err(CheckpointError::InvalidCheckpointSequence);
        }
        match (&self.previous_checkpoint_digest, self.checkpoint_sequence) {
            (None, 1) => {}
            (Some(digest), sequence) if sequence > 1 => validate_digest(digest)?,
            _ => return Err(CheckpointError::InvalidCheckpointSequence),
        }

        validate_action_hash(&self.source_action_hash)?;
        match (&self.source_previous_action_hash, self.source_action_sequence) {
            (None, 0) => {}
            (Some(hash), sequence) if sequence > 0 => validate_action_hash(hash)
                .map_err(|_| CheckpointError::InvalidSourceChainFrontier)?,
            _ => return Err(CheckpointError::InvalidSourceChainFrontier),
        }
        Ok(())
    }
}

/// Hashes the complete immutable intent under a separate versioned profile.
/// The operation identity is bound separately in the checkpoint subject.
pub fn canonical_intent_digest(intent: &PaymentIntent) -> Result<String, CheckpointError> {
    intent.validate().map_err(|_| CheckpointError::InvalidIntent)?;
    let mut output = Vec::with_capacity(256);
    output.extend_from_slice(INTENT_DOMAIN);
    append_text(&mut output, &intent.sender_did)?;
    append_text(&mut output, &intent.recipient_did)?;
    append_text(&mut output, &intent.client_idempotency_key)?;
    append_text(&mut output, &intent.payload_fingerprint)?;
    append_u64(&mut output, intent.amount_minor);
    append_u64(&mut output, intent.fee_minor);
    append_text(&mut output, &intent.currency)?;
    let digest = Sha256::digest(output);
    Ok(format_digest(&digest))
}

fn append_text(output: &mut Vec<u8>, value: &str) -> Result<(), CheckpointError> {
    let bytes = value.as_bytes();
    if bytes.len() > MAX_CHECKPOINT_TEXT_BYTES {
        return Err(CheckpointError::InvalidSubject);
    }
    let length = u32::try_from(bytes.len()).map_err(|_| CheckpointError::InvalidSubject)?;
    output.extend_from_slice(&length.to_be_bytes());
    output.extend_from_slice(bytes);
    Ok(())
}

fn append_u64(output: &mut Vec<u8>, value: u64) {
    output.extend_from_slice(&value.to_be_bytes());
}

fn validate_digest(value: &str) -> Result<(), CheckpointError> {
    let bytes = value.as_bytes();
    if bytes.len() != 71
        || !bytes.starts_with(b"sha256:")
        || !bytes[7..].iter().all(u8::is_ascii_hexdigit)
        || bytes[7..].iter().any(u8::is_ascii_uppercase)
    {
        return Err(CheckpointError::InvalidDigest);
    }
    Ok(())
}

fn validate_action_hash(value: &[u8]) -> Result<(), CheckpointError> {
    if value.len() != ACTION_HASH_BYTES || value[..3] != ACTION_HASH_PREFIX {
        return Err(CheckpointError::InvalidActionHash);
    }
    Ok(())
}

fn format_digest(bytes: &[u8]) -> String {
    let mut output = String::with_capacity(71);
    output.push_str("sha256:");
    for byte in bytes {
        use std::fmt::Write as _;
        write!(&mut output, "{byte:02x}").expect("writing to String cannot fail");
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    fn test_intent() -> PaymentIntent {
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

    fn action_hash(core: u8, location: u8) -> Vec<u8> {
        let mut hash = ACTION_HASH_PREFIX.to_vec();
        hash.extend([core; 32]);
        hash.extend([location; 4]);
        hash
    }

    fn vectors() -> HashMap<String, String> {
        include_str!("../fixtures/sap-payment-journal-checkpoint-v1.kat")
            .lines()
            .filter(|line| !line.is_empty() && !line.starts_with('#'))
            .map(|line| {
                let (key, value) = line.split_once('=').expect("fixture key=value");
                (key.to_owned(), value.to_owned())
            })
            .collect()
    }

    fn payments_subject(intent: &PaymentIntent) -> JournalCheckpointSubject {
        JournalCheckpointSubject {
            owner_domain: CheckpointOwnerDomain::Payments,
            operation_id: PaymentJournal::new(intent.clone()).operation_id().to_owned(),
            intent_digest: canonical_intent_digest(intent).expect("valid intent"),
            owner_did: "did:mycelix:payments:alice".into(),
            event_log_id: "sap-journal:payments:op-001".into(),
            event_count: 8,
            tail_digest: format!(
                "sha256:{}",
                "a1e5b966be2b6aab453c65370639d702e6d83b3097c9a542a0d85d28bf6ed899",
            ),
            checkpoint_sequence: 1,
            previous_checkpoint_digest: None,
            source_action_hash: action_hash(0x11, 0x22),
            source_action_sequence: 42,
            source_previous_action_hash: Some(action_hash(0x33, 0x44)),
        }
    }

    fn treasury_subject(intent: &PaymentIntent) -> JournalCheckpointSubject {
        JournalCheckpointSubject {
            owner_domain: CheckpointOwnerDomain::Treasury,
            operation_id: PaymentJournal::new(intent.clone()).operation_id().to_owned(),
            intent_digest: canonical_intent_digest(intent).expect("valid intent"),
            owner_did: "did:mycelix:treasury:community".into(),
            event_log_id: "sap-journal:treasury:op-001".into(),
            event_count: 2,
            tail_digest: format!(
                "sha256:{}",
                "98bea9e9428e50e2fb522a6d8e7800d3cbc9e32ea068c5b23313f459d9e13f27",
            ),
            checkpoint_sequence: 1,
            previous_checkpoint_digest: None,
            source_action_hash: action_hash(0x55, 0x66),
            source_action_sequence: 101,
            source_previous_action_hash: Some(action_hash(0x77, 0x88)),
        }
    }

    fn hex(bytes: &[u8]) -> String {
        const DIGITS: &[u8; 16] = b"0123456789abcdef";
        let mut output = String::with_capacity(bytes.len() * 2);
        for byte in bytes {
            output.push(DIGITS[(byte >> 4) as usize] as char);
            output.push(DIGITS[(byte & 0x0f) as usize] as char);
        }
        output
    }

    #[test]
    fn shared_known_answer_vectors_pin_intent_and_owner_scoped_checkpoint_bytes() {
        let vectors = vectors();
        let intent = test_intent();
        let intent_digest = canonical_intent_digest(&intent).expect("intent digest");
        assert_eq!(intent_digest, vectors["intent-digest"]);

        for (name, subject) in [
            ("payments", payments_subject(&intent)),
            ("treasury", treasury_subject(&intent)),
        ] {
            let bytes = subject.canonical_bytes().expect("canonical subject");
            assert_eq!(hex(&bytes), vectors[&format!("{name}-subject-hex")]);
            assert_eq!(
                subject.canonical_digest().expect("subject digest"),
                vectors[&format!("{name}-subject-digest")]
            );
        }
    }

    #[test]
    fn owner_domain_and_subject_mutations_change_the_commitment() {
        let intent = test_intent();
        let original = payments_subject(&intent);
        let baseline = original.canonical_digest().expect("valid checkpoint");

        let mut changed = original.clone();
        changed.owner_domain = CheckpointOwnerDomain::Treasury;
        assert_ne!(changed.canonical_digest().expect("valid changed checkpoint"), baseline);

        let mut changed = original.clone();
        changed.owner_did.push_str(":substituted");
        assert_ne!(changed.canonical_digest().expect("valid changed checkpoint"), baseline);

        let mut changed = original.clone();
        changed.event_count += 1;
        assert_ne!(changed.canonical_digest().expect("valid changed checkpoint"), baseline);

        let mut changed = original.clone();
        changed.tail_digest.replace_range(7..8, "b");
        assert_ne!(changed.canonical_digest().expect("valid changed checkpoint"), baseline);

        let mut changed = original.clone();
        changed.source_action_hash[10] ^= 1;
        assert_ne!(changed.canonical_digest().expect("valid changed checkpoint"), baseline);
    }

    #[test]
    fn checkpoint_sequence_requires_a_prior_digest_after_genesis() {
        let intent = test_intent();
        let mut subject = payments_subject(&intent);
        subject.checkpoint_sequence = 2;
        assert_eq!(
            subject.validate(),
            Err(CheckpointError::InvalidCheckpointSequence)
        );
        subject.previous_checkpoint_digest = Some(
            format!("sha256:{}", "b".repeat(64)),
        );
        assert_eq!(subject.validate(), Ok(()));
    }

    #[test]
    fn source_chain_frontier_requires_a_typed_predecessor_after_genesis() {
        let intent = test_intent();
        let mut subject = payments_subject(&intent);
        subject.source_action_sequence = 0;
        assert_eq!(
            subject.validate(),
            Err(CheckpointError::InvalidSourceChainFrontier)
        );
        subject.source_previous_action_hash = None;
        assert_eq!(subject.validate(), Ok(()));
    }

    #[test]
    fn rejects_wrong_hash_types_malformed_digests_and_unbounded_text() {
        let intent = test_intent();
        let mut subject = payments_subject(&intent);
        subject.source_action_hash[0] = 0;
        assert_eq!(subject.validate(), Err(CheckpointError::InvalidActionHash));

        let mut subject = payments_subject(&intent);
        subject.intent_digest = format!("SHA256:{}", "a".repeat(64));
        assert_eq!(subject.validate(), Err(CheckpointError::InvalidDigest));

        let mut subject = payments_subject(&intent);
        subject.event_log_id = "x".repeat(MAX_CHECKPOINT_TEXT_BYTES + 1);
        assert_eq!(subject.validate(), Err(CheckpointError::InvalidSubject));
    }


    #[test]
    fn treasury_checkpoints_extend_their_own_chain_and_source_frontier() {
        let intent = test_intent();
        let first = treasury_subject(&intent);
        let vectors = vectors();
        assert_eq!(
            first.canonical_digest().expect("first Treasury checkpoint"),
            vectors["treasury-subject-digest"]
        );

        let mut next = first.clone();
        next.event_count = 3;
        next.checkpoint_sequence = 2;
        next.previous_checkpoint_digest = Some(
            first.canonical_digest().expect("prior Treasury checkpoint"),
        );
        next.source_action_hash = action_hash(0x99, 0xaa);
        next.source_action_sequence = 102;
        next.source_previous_action_hash = Some(first.source_action_hash.clone());

        let bytes = next.canonical_bytes().expect("continued Treasury checkpoint");
        assert_eq!(hex(&bytes), vectors["treasury-next-subject-hex"]);
        assert_eq!(
            next.canonical_digest().expect("continued Treasury digest"),
            vectors["treasury-next-subject-digest"]
        );
    }

    #[test]
    fn treasury_checkpoint_has_an_independent_owner_domain() {
        let intent = test_intent();
        let payments = payments_subject(&intent);
        let treasury = treasury_subject(&intent);
        assert_ne!(
            payments.canonical_digest().expect("payments digest"),
            treasury.canonical_digest().expect("treasury digest")
        );
    }
}
