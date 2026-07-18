//! Versioned wire contracts for the Pulse working-alpha message lifecycle.
//!
//! The canonical encoders are deliberately independent of Serde formats. They
//! use fixed-width integers and length-prefixed byte strings so signatures do
//! not depend on map ordering or serializer implementation details.

use serde::{Deserialize, Serialize};

pub const ENVELOPE_V1: u16 = 1;
pub const ENVELOPE_V2_HYBRID_PQC: u16 = 2;
pub const SUITE_X25519_AES_256_GCM_ED25519: &str = "x25519-hkdf-sha256-aes256gcm-ed25519";
pub const SUITE_X25519_MLKEM768_AES_256_GCM_AGENT_MLDSA65: &str =
    "x25519+ml-kem-768-hkdf-sha256-aes256gcm-agent-ed25519+ml-dsa-65";
const ENVELOPE_DOMAIN: &[u8] = b"mycelix-pulse/envelope/v1\0";
const ENVELOPE_V2_DOMAIN: &[u8] = b"mycelix-pulse/envelope/v2-hybrid-pqc\0";
const RECEIPT_DOMAIN: &[u8] = b"mycelix-pulse/receipt/v1\0";
pub const ML_KEM_768_ENCAPSULATION_KEY_BYTES: usize = 1184;
pub const ML_KEM_768_CIPHERTEXT_BYTES: usize = 1088;
pub const ML_DSA_65_VERIFYING_KEY_BYTES: usize = 1952;
pub const ML_DSA_65_SIGNATURE_BYTES: usize = 3309;
const PLAINTEXT_V2_DOMAIN: &[u8] = b"mycelix-pulse/plaintext/v2\0";

/// The single payload protected by one AES-256-GCM invocation. Keeping the
/// subject and body in one ciphertext prevents nonce reuse across fields and
/// makes authentication/decryption atomic.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MessagePlaintextV2 {
    pub subject: String,
    pub body: String,
    pub content_type: String,
}

impl MessagePlaintextV2 {
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ProtocolError> {
        let mut out = Vec::with_capacity(
            PLAINTEXT_V2_DOMAIN.len()
                + self.subject.len()
                + self.body.len()
                + self.content_type.len()
                + 12,
        );
        out.extend_from_slice(PLAINTEXT_V2_DOMAIN);
        put_bytes(&mut out, self.subject.as_bytes())?;
        put_bytes(&mut out, self.body.as_bytes())?;
        put_bytes(&mut out, self.content_type.as_bytes())?;
        Ok(out)
    }

    pub fn from_canonical_bytes(bytes: &[u8]) -> Result<Self, ProtocolError> {
        let Some(mut rest) = bytes.strip_prefix(PLAINTEXT_V2_DOMAIN) else {
            return Err(ProtocolError::InvalidPlaintextEncoding);
        };
        let subject = take_text(&mut rest)?;
        let body = take_text(&mut rest)?;
        let content_type = take_text(&mut rest)?;
        if !rest.is_empty() {
            return Err(ProtocolError::InvalidPlaintextEncoding);
        }
        Ok(Self {
            subject,
            body,
            content_type,
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct MessageId(pub [u8; 32]);

#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct EncryptionKeyId(pub [u8; 32]);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EncryptionKeyState {
    Active,
    Retired,
    RevokedCompromised,
    Lost,
}

impl EncryptionKeyState {
    pub fn accepts_new_messages(self) -> bool {
        self == Self::Active
    }

    pub fn permits_historical_decryption(self) -> bool {
        matches!(
            self,
            Self::Active | Self::Retired | Self::RevokedCompromised
        )
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthenticatedMetadataV1 {
    pub in_reply_to: Option<MessageId>,
    pub thread_id: Option<[u8; 32]>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EncryptedEnvelopeV1 {
    pub version: u16,
    pub cipher_suite: String,
    pub message_id: MessageId,
    pub sender_agent: Vec<u8>,
    pub recipient_agent: Vec<u8>,
    pub sender_key_id: EncryptionKeyId,
    pub recipient_key_id: EncryptionKeyId,
    pub ephemeral_public_key: [u8; 32],
    pub nonce: [u8; 24],
    pub encrypted_subject: Vec<u8>,
    pub encrypted_body: Vec<u8>,
    pub metadata: AuthenticatedMetadataV1,
    pub created_at_micros: i64,
    pub agent_signature: Vec<u8>,
}

/// Default new-message profile once the hybrid implementation-evidence gate
/// passes. Both signatures cover exactly [`canonical_signing_bytes`](Self::canonical_signing_bytes).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EncryptedEnvelopeV2HybridPqc {
    pub version: u16,
    pub cipher_suite: String,
    pub message_id: MessageId,
    pub sender_agent: Vec<u8>,
    pub recipient_agent: Vec<u8>,
    pub sender_mldsa_key_id: EncryptionKeyId,
    pub recipient_hybrid_key_id: EncryptionKeyId,
    pub x25519_ephemeral_public_key: [u8; 32],
    pub ml_kem_ciphertext: Vec<u8>,
    pub nonce: [u8; 12],
    /// AES-256-GCM output for one canonical [`MessagePlaintextV2`] value,
    /// including the 16-byte authentication tag.
    pub ciphertext: Vec<u8>,
    pub metadata: AuthenticatedMetadataV1,
    pub created_at_micros: i64,
    pub agent_signature: Vec<u8>,
    pub ml_dsa_signature: Vec<u8>,
}

impl EncryptedEnvelopeV2HybridPqc {
    pub fn validate_structure(&self) -> Result<(), ProtocolError> {
        if self.version != ENVELOPE_V2_HYBRID_PQC {
            return Err(ProtocolError::UnsupportedEnvelopeVersion(self.version));
        }
        if self.cipher_suite != SUITE_X25519_MLKEM768_AES_256_GCM_AGENT_MLDSA65 {
            return Err(ProtocolError::UnsupportedCipherSuite(
                self.cipher_suite.clone(),
            ));
        }
        if self.ml_kem_ciphertext.len() != ML_KEM_768_CIPHERTEXT_BYTES {
            return Err(ProtocolError::InvalidComponentLength {
                component: "ml_kem_ciphertext",
                expected: ML_KEM_768_CIPHERTEXT_BYTES,
                actual: self.ml_kem_ciphertext.len(),
            });
        }
        if !self.ml_dsa_signature.is_empty()
            && self.ml_dsa_signature.len() != ML_DSA_65_SIGNATURE_BYTES
        {
            return Err(ProtocolError::InvalidComponentLength {
                component: "ml_dsa_signature",
                expected: ML_DSA_65_SIGNATURE_BYTES,
                actual: self.ml_dsa_signature.len(),
            });
        }
        Ok(())
    }

    /// Canonical transcript signed independently by the Holochain agent key
    /// and the advertised ML-DSA-65 key. Verification requires both.
    pub fn canonical_signing_bytes(&self) -> Result<Vec<u8>, ProtocolError> {
        self.validate_structure()?;
        let mut out =
            Vec::with_capacity(self.ml_kem_ciphertext.len() + self.ciphertext.len() + 512);
        self.append_authenticated_header(&mut out)?;
        put_bytes(&mut out, &self.ciphertext)?;
        Ok(out)
    }

    /// AES-GCM AAD. Ciphertexts are AEAD outputs, while every routing, suite,
    /// key-binding, KEM, nonce, and immutable metadata field is included.
    pub fn canonical_aad(&self) -> Result<Vec<u8>, ProtocolError> {
        self.validate_structure()?;
        let mut out = Vec::with_capacity(self.ml_kem_ciphertext.len() + 512);
        self.append_authenticated_header(&mut out)?;
        Ok(out)
    }

    fn append_authenticated_header(&self, out: &mut Vec<u8>) -> Result<(), ProtocolError> {
        out.extend_from_slice(ENVELOPE_V2_DOMAIN);
        put_u16(out, self.version);
        put_bytes(out, self.cipher_suite.as_bytes())?;
        out.extend_from_slice(&self.message_id.0);
        put_bytes(out, &self.sender_agent)?;
        put_bytes(out, &self.recipient_agent)?;
        out.extend_from_slice(&self.sender_mldsa_key_id.0);
        out.extend_from_slice(&self.recipient_hybrid_key_id.0);
        out.extend_from_slice(&self.x25519_ephemeral_public_key);
        put_bytes(out, &self.ml_kem_ciphertext)?;
        out.extend_from_slice(&self.nonce);
        put_option_32(out, self.metadata.in_reply_to.as_ref().map(|id| &id.0));
        put_option_32(out, self.metadata.thread_id.as_ref());
        out.extend_from_slice(&self.created_at_micros.to_be_bytes());
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NewMessageProfile {
    V2HybridPqc,
    V1ClassicalDiagnostic,
}

/// Select a new-message profile without permitting an implicit downgrade.
pub fn select_new_message_profile(
    request: NewMessageProfile,
    recipient_advertises_hybrid: bool,
) -> Result<NewMessageProfile, ProtocolError> {
    match (request, recipient_advertises_hybrid) {
        (NewMessageProfile::V2HybridPqc, true) => Ok(NewMessageProfile::V2HybridPqc),
        (NewMessageProfile::V2HybridPqc, false) => Err(ProtocolError::HybridProfileUnavailable),
        (NewMessageProfile::V1ClassicalDiagnostic, _) => {
            Ok(NewMessageProfile::V1ClassicalDiagnostic)
        }
    }
}

impl EncryptedEnvelopeV1 {
    pub fn validate_version(&self) -> Result<(), ProtocolError> {
        if self.version != ENVELOPE_V1 {
            return Err(ProtocolError::UnsupportedEnvelopeVersion(self.version));
        }
        if self.cipher_suite != SUITE_X25519_AES_256_GCM_ED25519 {
            return Err(ProtocolError::UnsupportedCipherSuite(
                self.cipher_suite.clone(),
            ));
        }
        Ok(())
    }

    /// Bytes covered by the Holochain agent signature.
    pub fn canonical_signing_bytes(&self) -> Result<Vec<u8>, ProtocolError> {
        self.validate_version()?;
        let mut out =
            Vec::with_capacity(self.encrypted_subject.len() + self.encrypted_body.len() + 512);
        out.extend_from_slice(ENVELOPE_DOMAIN);
        put_u16(&mut out, self.version);
        put_bytes(&mut out, self.cipher_suite.as_bytes())?;
        out.extend_from_slice(&self.message_id.0);
        put_bytes(&mut out, &self.sender_agent)?;
        put_bytes(&mut out, &self.recipient_agent)?;
        out.extend_from_slice(&self.sender_key_id.0);
        out.extend_from_slice(&self.recipient_key_id.0);
        out.extend_from_slice(&self.ephemeral_public_key);
        out.extend_from_slice(&self.nonce);
        put_bytes(&mut out, &self.encrypted_subject)?;
        put_bytes(&mut out, &self.encrypted_body)?;
        put_option_32(&mut out, self.metadata.in_reply_to.as_ref().map(|id| &id.0));
        put_option_32(&mut out, self.metadata.thread_id.as_ref());
        out.extend_from_slice(&self.created_at_micros.to_be_bytes());
        Ok(out)
    }

    /// Immutable routing and identity bytes bound as AES-GCM AAD.
    /// Ciphertexts are excluded because they are the AEAD output.
    pub fn canonical_aad(&self) -> Result<Vec<u8>, ProtocolError> {
        self.validate_version()?;
        let mut out = Vec::with_capacity(320);
        out.extend_from_slice(ENVELOPE_DOMAIN);
        put_u16(&mut out, self.version);
        put_bytes(&mut out, self.cipher_suite.as_bytes())?;
        out.extend_from_slice(&self.message_id.0);
        put_bytes(&mut out, &self.sender_agent)?;
        put_bytes(&mut out, &self.recipient_agent)?;
        out.extend_from_slice(&self.sender_key_id.0);
        out.extend_from_slice(&self.recipient_key_id.0);
        out.extend_from_slice(&self.ephemeral_public_key);
        out.extend_from_slice(&self.nonce);
        put_option_32(&mut out, self.metadata.in_reply_to.as_ref().map(|id| &id.0));
        put_option_32(&mut out, self.metadata.thread_id.as_ref());
        out.extend_from_slice(&self.created_at_micros.to_be_bytes());
        Ok(out)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReceiptKind {
    Observed,
    Delivered,
    Read,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecipientReceipt {
    pub message_id: MessageId,
    pub recipient_agent: Vec<u8>,
    pub kind: ReceiptKind,
    pub occurred_at_micros: i64,
    pub recipient_signature: Vec<u8>,
}

impl RecipientReceipt {
    pub fn canonical_signing_bytes(&self) -> Result<Vec<u8>, ProtocolError> {
        let mut out = Vec::with_capacity(128 + self.recipient_agent.len());
        out.extend_from_slice(RECEIPT_DOMAIN);
        out.extend_from_slice(&self.message_id.0);
        put_bytes(&mut out, &self.recipient_agent)?;
        out.push(match self.kind {
            ReceiptKind::Observed => 1,
            ReceiptKind::Delivered => 2,
            ReceiptKind::Read => 3,
        });
        out.extend_from_slice(&self.occurred_at_micros.to_be_bytes());
        Ok(out)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DerivedDeliveryState {
    Committed,
    Observed,
    Delivered,
    Read,
}

pub fn derive_delivery_state(
    message_id: &MessageId,
    recipient_agent: &[u8],
    receipts: &[RecipientReceipt],
) -> DerivedDeliveryState {
    receipts
        .iter()
        .filter(|receipt| {
            &receipt.message_id == message_id && receipt.recipient_agent == recipient_agent
        })
        .map(|receipt| match receipt.kind {
            ReceiptKind::Observed => DerivedDeliveryState::Observed,
            ReceiptKind::Delivered => DerivedDeliveryState::Delivered,
            ReceiptKind::Read => DerivedDeliveryState::Read,
        })
        .max()
        .unwrap_or(DerivedDeliveryState::Committed)
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProtocolError {
    UnsupportedEnvelopeVersion(u16),
    UnsupportedCipherSuite(String),
    FieldTooLarge,
    HybridProfileUnavailable,
    InvalidComponentLength {
        component: &'static str,
        expected: usize,
        actual: usize,
    },
    InvalidPlaintextEncoding,
}

fn take_text(input: &mut &[u8]) -> Result<String, ProtocolError> {
    if input.len() < 4 {
        return Err(ProtocolError::InvalidPlaintextEncoding);
    }
    let len = u32::from_be_bytes(input[..4].try_into().expect("four-byte length")) as usize;
    *input = &input[4..];
    if input.len() < len {
        return Err(ProtocolError::InvalidPlaintextEncoding);
    }
    let value = String::from_utf8(input[..len].to_vec())
        .map_err(|_| ProtocolError::InvalidPlaintextEncoding)?;
    *input = &input[len..];
    Ok(value)
}

fn put_u16(out: &mut Vec<u8>, value: u16) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn put_bytes(out: &mut Vec<u8>, value: &[u8]) -> Result<(), ProtocolError> {
    let len = u32::try_from(value.len()).map_err(|_| ProtocolError::FieldTooLarge)?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(value);
    Ok(())
}

fn put_option_32(out: &mut Vec<u8>, value: Option<&[u8; 32]>) {
    match value {
        Some(value) => {
            out.push(1);
            out.extend_from_slice(value);
        }
        None => out.push(0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn envelope() -> EncryptedEnvelopeV1 {
        EncryptedEnvelopeV1 {
            version: ENVELOPE_V1,
            cipher_suite: SUITE_X25519_AES_256_GCM_ED25519.into(),
            message_id: MessageId([1; 32]),
            sender_agent: vec![2; 39],
            recipient_agent: vec![3; 39],
            sender_key_id: EncryptionKeyId([4; 32]),
            recipient_key_id: EncryptionKeyId([5; 32]),
            ephemeral_public_key: [6; 32],
            nonce: [7; 24],
            encrypted_subject: b"subject ciphertext".to_vec(),
            encrypted_body: b"body ciphertext".to_vec(),
            metadata: AuthenticatedMetadataV1 {
                in_reply_to: Some(MessageId([8; 32])),
                thread_id: Some([9; 32]),
            },
            created_at_micros: 42,
            agent_signature: vec![10; 64],
        }
    }

    fn hybrid_envelope() -> EncryptedEnvelopeV2HybridPqc {
        EncryptedEnvelopeV2HybridPqc {
            version: ENVELOPE_V2_HYBRID_PQC,
            cipher_suite: SUITE_X25519_MLKEM768_AES_256_GCM_AGENT_MLDSA65.into(),
            message_id: MessageId([11; 32]),
            sender_agent: vec![12; 39],
            recipient_agent: vec![13; 39],
            sender_mldsa_key_id: EncryptionKeyId([14; 32]),
            recipient_hybrid_key_id: EncryptionKeyId([15; 32]),
            x25519_ephemeral_public_key: [16; 32],
            ml_kem_ciphertext: vec![17; ML_KEM_768_CIPHERTEXT_BYTES],
            nonce: [18; 12],
            ciphertext: vec![19; 144],
            metadata: AuthenticatedMetadataV1 {
                in_reply_to: None,
                thread_id: Some([21; 32]),
            },
            created_at_micros: 84,
            agent_signature: vec![22; 64],
            ml_dsa_signature: vec![23; ML_DSA_65_SIGNATURE_BYTES],
        }
    }

    #[test]
    fn canonical_bytes_are_stable_and_signature_independent() {
        let first = envelope();
        let mut second = first.clone();
        second.agent_signature = vec![99; 64];
        assert_eq!(
            first.canonical_signing_bytes().unwrap(),
            second.canonical_signing_bytes().unwrap()
        );
        assert_eq!(
            first.canonical_aad().unwrap(),
            second.canonical_aad().unwrap()
        );
    }

    #[test]
    fn immutable_field_changes_change_signing_bytes() {
        let first = envelope();
        let mut second = first.clone();
        second.recipient_key_id = EncryptionKeyId([44; 32]);
        assert_ne!(
            first.canonical_signing_bytes().unwrap(),
            second.canonical_signing_bytes().unwrap()
        );
        assert_ne!(
            first.canonical_aad().unwrap(),
            second.canonical_aad().unwrap()
        );
    }

    #[test]
    fn unknown_versions_and_suites_fail_closed() {
        let mut value = envelope();
        value.version = 2;
        assert_eq!(
            value.canonical_signing_bytes(),
            Err(ProtocolError::UnsupportedEnvelopeVersion(2))
        );
        value.version = ENVELOPE_V1;
        value.cipher_suite = "future-suite".into();
        assert_eq!(
            value.canonical_aad(),
            Err(ProtocolError::UnsupportedCipherSuite("future-suite".into()))
        );
    }

    #[test]
    fn receipts_are_recipient_scoped_and_monotonic() {
        let message = MessageId([1; 32]);
        let alice = vec![2; 39];
        let bob = vec![3; 39];
        let receipts = vec![
            RecipientReceipt {
                message_id: message.clone(),
                recipient_agent: bob.clone(),
                kind: ReceiptKind::Read,
                occurred_at_micros: 3,
                recipient_signature: vec![],
            },
            RecipientReceipt {
                message_id: message.clone(),
                recipient_agent: bob.clone(),
                kind: ReceiptKind::Observed,
                occurred_at_micros: 1,
                recipient_signature: vec![],
            },
        ];
        assert_eq!(
            derive_delivery_state(&message, &bob, &receipts),
            DerivedDeliveryState::Read
        );
        assert_eq!(
            derive_delivery_state(&message, &alice, &receipts),
            DerivedDeliveryState::Committed
        );
    }

    #[test]
    fn key_states_enforce_new_send_policy_without_erasing_history() {
        assert!(EncryptionKeyState::Active.accepts_new_messages());
        assert!(!EncryptionKeyState::Retired.accepts_new_messages());
        assert!(EncryptionKeyState::Retired.permits_historical_decryption());
        assert!(EncryptionKeyState::RevokedCompromised.permits_historical_decryption());
        assert!(!EncryptionKeyState::Lost.permits_historical_decryption());
    }

    #[test]
    fn hybrid_transcript_binds_both_kem_components_and_ciphertexts() {
        let original = hybrid_envelope();
        let transcript = original.canonical_signing_bytes().unwrap();

        let mut changed = original.clone();
        changed.x25519_ephemeral_public_key[0] ^= 1;
        assert_ne!(transcript, changed.canonical_signing_bytes().unwrap());

        let mut changed = original.clone();
        changed.ml_kem_ciphertext[0] ^= 1;
        assert_ne!(transcript, changed.canonical_signing_bytes().unwrap());

        let mut changed = original;
        changed.ciphertext[0] ^= 1;
        assert_ne!(transcript, changed.canonical_signing_bytes().unwrap());
    }

    #[test]
    fn v2_plaintext_encoding_is_unambiguous() {
        let first = MessagePlaintextV2 {
            subject: "ab".into(),
            body: "c".into(),
            content_type: "text/plain".into(),
        };
        let second = MessagePlaintextV2 {
            subject: "a".into(),
            body: "bc".into(),
            content_type: "text/plain".into(),
        };
        assert_ne!(
            first.canonical_bytes().unwrap(),
            second.canonical_bytes().unwrap()
        );
        assert_eq!(
            MessagePlaintextV2::from_canonical_bytes(&first.canonical_bytes().unwrap()).unwrap(),
            first
        );
        let mut malformed = second.canonical_bytes().unwrap();
        malformed.push(0);
        assert_eq!(
            MessagePlaintextV2::from_canonical_bytes(&malformed),
            Err(ProtocolError::InvalidPlaintextEncoding)
        );
    }

    #[test]
    fn hybrid_structure_rejects_malformed_encapsulation_and_signature() {
        let mut value = hybrid_envelope();
        value.ml_kem_ciphertext.pop();
        assert!(matches!(
            value.validate_structure(),
            Err(ProtocolError::InvalidComponentLength {
                component: "ml_kem_ciphertext",
                ..
            })
        ));

        let mut value = hybrid_envelope();
        value.ml_dsa_signature.pop();
        assert!(matches!(
            value.validate_structure(),
            Err(ProtocolError::InvalidComponentLength {
                component: "ml_dsa_signature",
                ..
            })
        ));
    }

    #[test]
    fn hybrid_profile_never_silently_downgrades() {
        assert_eq!(
            select_new_message_profile(NewMessageProfile::V2HybridPqc, true),
            Ok(NewMessageProfile::V2HybridPqc)
        );
        assert_eq!(
            select_new_message_profile(NewMessageProfile::V2HybridPqc, false),
            Err(ProtocolError::HybridProfileUnavailable)
        );
        assert_eq!(
            select_new_message_profile(NewMessageProfile::V1ClassicalDiagnostic, true),
            Ok(NewMessageProfile::V1ClassicalDiagnostic)
        );
    }
}
