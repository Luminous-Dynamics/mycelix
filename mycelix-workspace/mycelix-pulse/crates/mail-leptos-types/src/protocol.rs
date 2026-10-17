//! Versioned wire contracts for the Pulse working-alpha message lifecycle.
//!
//! The canonical encoders are deliberately independent of Serde formats. They
//! use fixed-width integers and length-prefixed byte strings so signatures do
//! not depend on map ordering or serializer implementation details.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

pub const ENVELOPE_V1: u16 = 1;
pub const ENVELOPE_V2_HYBRID_PQC: u16 = 2;
pub const SUITE_X25519_AES_256_GCM_ED25519: &str = "x25519-hkdf-sha256-aes256gcm-ed25519";
pub const SUITE_X25519_MLKEM768_AES_256_GCM_AGENT_MLDSA65: &str =
    "x25519+ml-kem-768-hkdf-sha256-aes256gcm-agent-ed25519+ml-dsa-65";
const ENVELOPE_DOMAIN: &[u8] = b"mycelix-pulse/envelope/v1\0";
const ENVELOPE_V2_DOMAIN: &[u8] = b"mycelix-pulse/envelope/v2-hybrid-pqc\0";
const RECEIPT_DOMAIN: &[u8] = b"mycelix-pulse/receipt/v1\0";
const RECEIPT_V2_DOMAIN: &[u8] = b"mycelix-pulse/receipt/v2\0";
const RECEIPT_EVENT_ID_DOMAIN: &[u8] = b"mycelix-pulse/receipt-event-id/v2\0";
const ENVELOPE_COMMITMENT_DOMAIN: &[u8] = b"mycelix-pulse/envelope-commitment/v1\0";
pub const RECEIPT_V2_PROTOCOL_VERSION: u16 = 2;
pub const ML_KEM_768_ENCAPSULATION_KEY_BYTES: usize = 1184;
pub const ML_KEM_768_CIPHERTEXT_BYTES: usize = 1088;
pub const ML_DSA_65_VERIFYING_KEY_BYTES: usize = 1952;
pub const ML_DSA_65_SIGNATURE_BYTES: usize = 3309;
pub const RECEIPT_V2_AGENT_SIGNATURE_BYTES: usize = 64;
pub const HOLOCHAIN_AGENT_PUBKEY_BYTES: usize = 39;
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
    /// Raw 39-byte `ActionHash` of the sender's published `HybridKeyBundleV2`
    /// entry — a DHT-lookup pointer only, not itself authenticated. A
    /// validator uses it to fetch a candidate bundle, then independently
    /// checks that bundle's own content hash matches `sender_mldsa_key_id`
    /// (which the signatures below already cover) before trusting it. Not
    /// part of the signed transcript/AAD for exactly that reason — its
    /// correctness is verified transitively, not by signing it directly.
    pub sender_mldsa_bundle_hash: Vec<u8>,
    /// Raw 39-byte `ActionHash` of the recipient's published `HybridKeyBundleV2`
    /// entry, at the time this envelope was sealed — same lookup-pointer-only
    /// contract as `sender_mldsa_bundle_hash` above. Lets a validator check
    /// that the recipient's key was still `Active` (not revoked/expired) when
    /// the sender committed to `recipient_hybrid_key_id`, closing the
    /// asymmetry where only the sender's key state was enforced on-chain.
    /// Not part of the signed transcript/AAD for the same reason as the
    /// sender pointer: a wrong or forged hash here only breaks the lookup,
    /// since the validator independently re-derives the fetched bundle's
    /// content hash and checks it against the already-signed
    /// `recipient_hybrid_key_id`.
    pub recipient_bundle_hash: Vec<u8>,
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

/// V2 receipt contract. This is additive: V1 signing bytes remain unchanged.
/// A V2 receipt can bind only a V2 hybrid-PQC envelope; V1 receipt semantics remain separate.
///
/// `receipt_event_id` is generated once for one logical receipt event and
/// MUST be reused across retries. Reusing that identity with altered signed
/// content is an idempotency conflict, not a new receipt. Durable conflict
/// detection remains the responsibility of the receipt store/coordinator.
///
/// `envelope_commitment` is SHA-256 over
/// [`canonical_envelope_commitment_preimage`] applied to the exact envelope's
/// canonical signing bytes. A verifier MUST recompute it from the envelope
/// it actually received; it must not trust a caller-supplied commitment.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecipientReceiptV2 {
    pub protocol_version: u16,
    /// Stable 256-bit idempotency identity, persisted or deterministically derived
    /// before first dispatch.
    pub receipt_event_id: [u8; 32],
    pub message_id: MessageId,
    pub recipient_agent: Vec<u8>,
    /// The exact envelope version/profile the recipient observed.
    pub envelope_version: u16,
    pub cipher_suite: String,
    /// Domain-separated commitment to the exact canonical envelope transcript.
    pub envelope_commitment: [u8; 32],
    pub kind: ReceiptKind,
    pub occurred_at_micros: i64,
    /// Agent signature over the canonical V2 transcript (64-byte Ed25519).
    pub agent_signature: Vec<u8>,
    /// Application-level post-quantum signature over the same transcript.
    pub ml_dsa_signature: Vec<u8>,
}

impl RecipientReceiptV2 {
    /// Reject structurally incomplete receipt signatures before cryptographic verification.
    ///
    /// This is only a length check. The caller/integrity validator MUST verify
    /// both signatures against the recipient agent key and the ML-DSA-65
    /// verifying key from the exact envelope-bound recipient bundle.
    pub fn validate_signature_shape(&self) -> Result<(), ProtocolError> {
        if self.agent_signature.len() != RECEIPT_V2_AGENT_SIGNATURE_BYTES {
            return Err(ProtocolError::InvalidComponentLength {
                component: "receipt_agent_signature",
                expected: RECEIPT_V2_AGENT_SIGNATURE_BYTES,
                actual: self.agent_signature.len(),
            });
        }
        if self.ml_dsa_signature.len() != ML_DSA_65_SIGNATURE_BYTES {
            return Err(ProtocolError::InvalidComponentLength {
                component: "receipt_ml_dsa_signature",
                expected: ML_DSA_65_SIGNATURE_BYTES,
                actual: self.ml_dsa_signature.len(),
            });
        }
        Ok(())
    }

    /// Verify that this receipt is bound to the exact V2 envelope transcript.
    ///
    /// This checks envelope/profile consistency and independently recomputes the
    /// envelope commitment. It does NOT verify either receipt signature.
    pub fn validate_envelope_binding(
        &self,
        envelope: &EncryptedEnvelopeV2HybridPqc,
    ) -> Result<(), ProtocolError> {
        // Also enforces the deterministic event identity and V2-only profile.
        self.canonical_signing_bytes()?;
        let envelope_transcript = envelope.canonical_signing_bytes()?;
        let expected_commitment =
            envelope_commitment_from_canonical_signing_bytes(&envelope_transcript)?;

        if self.message_id != envelope.message_id
            || self.recipient_agent != envelope.recipient_agent
            || self.envelope_version != envelope.version
            || self.cipher_suite != envelope.cipher_suite
            || self.envelope_commitment != expected_commitment
        {
            return Err(ProtocolError::ReceiptContextMismatch);
        }
        Ok(())
    }

    /// Exact transcript covered by the recipient's V2 receipt signatures.
    /// Both signatures are intentionally excluded. V2 requires the recipient's
    /// Holochain-agent Ed25519 signature AND ML-DSA-65 signature; neither alone
    /// satisfies this profile.
    pub fn canonical_signing_bytes(&self) -> Result<Vec<u8>, ProtocolError> {
        if self.protocol_version != RECEIPT_V2_PROTOCOL_VERSION {
            return Err(ProtocolError::UnsupportedReceiptVersion(self.protocol_version));
        }

        // Enforce one event identity for a given logical message, recipient,
        // and receipt kind. Without this check callers could bypass replay
        // classification by supplying arbitrary IDs for the same event.
        let expected_event_id =
            derive_receipt_event_id(&self.message_id, &self.recipient_agent, self.kind)?;
        if self.receipt_event_id != expected_event_id {
            return Err(ProtocolError::ReceiptEventIdentityMismatch);
        }

        let expected_suite = match self.envelope_version {
            ENVELOPE_V2_HYBRID_PQC => SUITE_X25519_MLKEM768_AES_256_GCM_AGENT_MLDSA65,
            ENVELOPE_V1 => return Err(ProtocolError::ReceiptContextMismatch),
            version => return Err(ProtocolError::UnsupportedEnvelopeVersion(version)),
        };
        if self.cipher_suite != expected_suite {
            return Err(ProtocolError::ReceiptContextMismatch);
        }

        let mut out = Vec::with_capacity(
            192 + self.recipient_agent.len() + self.cipher_suite.len(),
        );
        out.extend_from_slice(RECEIPT_V2_DOMAIN);
        put_u16(&mut out, self.protocol_version);
        out.extend_from_slice(&self.receipt_event_id);
        out.extend_from_slice(&self.message_id.0);
        put_bytes(&mut out, &self.recipient_agent)?;
        put_u16(&mut out, self.envelope_version);
        put_bytes(&mut out, self.cipher_suite.as_bytes())?;
        out.extend_from_slice(&self.envelope_commitment);
        out.push(match self.kind {
            ReceiptKind::Observed => 1,
            ReceiptKind::Delivered => 2,
            ReceiptKind::Read => 3,
        });
        out.extend_from_slice(&self.occurred_at_micros.to_be_bytes());
        Ok(out)
    }
}

/// Canonical input to the SHA-256 receipt-event identity derivation.
///
/// The ID is deliberately independent of timestamp, route, attempt number,
/// and envelope commitment so retries remain stable and conflicting reuse of
/// the same logical message/recipient/kind can be detected.
pub fn canonical_receipt_event_id_preimage(
    message_id: &MessageId,
    recipient_agent: &[u8],
    kind: ReceiptKind,
) -> Result<Vec<u8>, ProtocolError> {
    if recipient_agent.len() != HOLOCHAIN_AGENT_PUBKEY_BYTES {
        return Err(ProtocolError::InvalidComponentLength {
            component: "recipient_agent",
            expected: HOLOCHAIN_AGENT_PUBKEY_BYTES,
            actual: recipient_agent.len(),
        });
    }
    let mut out = Vec::with_capacity(
        RECEIPT_EVENT_ID_DOMAIN.len() + 32 + 4 + recipient_agent.len() + 1,
    );
    out.extend_from_slice(RECEIPT_EVENT_ID_DOMAIN);
    out.extend_from_slice(&message_id.0);
    put_bytes(&mut out, recipient_agent)?;
    out.push(match kind {
        ReceiptKind::Observed => 1,
        ReceiptKind::Delivered => 2,
        ReceiptKind::Read => 3,
    });
    Ok(out)
}

/// Construct the domain-separated preimage for the envelope-commitment SHA-256.
pub fn canonical_envelope_commitment_preimage(
    canonical_signing_bytes: &[u8],
) -> Result<Vec<u8>, ProtocolError> {
    let mut out = Vec::with_capacity(
        ENVELOPE_COMMITMENT_DOMAIN.len() + canonical_signing_bytes.len() + 4,
    );
    out.extend_from_slice(ENVELOPE_COMMITMENT_DOMAIN);
    put_bytes(&mut out, canonical_signing_bytes)?;
    Ok(out)
}

/// Result of comparing a candidate receipt with a stored receipt found by event ID.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReceiptReplayClassification {
    /// The stable event ID and all signed transcript bytes are identical.
    ExactReplay,
    /// The stable event ID is reused with different signed transcript bytes.
    ConflictingReuse,
}

/// Classify a candidate against the prior record already indexed by its event ID.
///
/// The caller MUST authenticate the incoming and stored recipient signatures
/// before using the result to affect delivery state. The durable store must
/// perform lookup/reservation atomically; this pure comparison does not prevent
/// concurrent duplicate commits or provide durable persistence.
pub fn classify_receipt_v2_replay(
    existing: &RecipientReceiptV2,
    incoming: &RecipientReceiptV2,
) -> Result<ReceiptReplayClassification, ProtocolError> {
    if existing.receipt_event_id != incoming.receipt_event_id {
        return Err(ProtocolError::ReceiptEventIdentityMismatch);
    }

    let existing_transcript = existing.canonical_signing_bytes()?;
    let incoming_transcript = incoming.canonical_signing_bytes()?;
    if existing_transcript == incoming_transcript {
        Ok(ReceiptReplayClassification::ExactReplay)
    } else {
        Ok(ReceiptReplayClassification::ConflictingReuse)
    }
}

/// Hash canonical envelope signing bytes into the V2 receipt commitment.
///
/// The argument must be the exact, validated envelope transcript; callers
/// must not hash an arbitrary body or caller-supplied display representation.
pub fn envelope_commitment_from_canonical_signing_bytes(
    canonical_signing_bytes: &[u8],
) -> Result<[u8; 32], ProtocolError> {
    let preimage = canonical_envelope_commitment_preimage(canonical_signing_bytes)?;
    Ok(Sha256::digest(preimage).into())
}

/// Derive the deterministic logical receipt identity for a message, recipient,
/// and receipt kind. Timestamp, route, attempt number, and envelope commitment
/// are intentionally excluded so retries/conflicts converge on the same key.
pub fn derive_receipt_event_id(
    message_id: &MessageId,
    recipient_agent: &[u8],
    kind: ReceiptKind,
) -> Result<[u8; 32], ProtocolError> {
    let preimage = canonical_receipt_event_id_preimage(message_id, recipient_agent, kind)?;
    Ok(Sha256::digest(preimage).into())
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
    UnsupportedReceiptVersion(u16),
    ReceiptContextMismatch,
    ReceiptEventIdentityMismatch,
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
            sender_mldsa_bundle_hash: vec![20; 39],
            recipient_bundle_hash: vec![21; 39],
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

    fn receipt_v2_for_envelope(envelope: &EncryptedEnvelopeV2HybridPqc) -> RecipientReceiptV2 {
        let transcript = envelope.canonical_signing_bytes().unwrap();
        let envelope_commitment =
            envelope_commitment_from_canonical_signing_bytes(&transcript).unwrap();
        RecipientReceiptV2 {
            protocol_version: RECEIPT_V2_PROTOCOL_VERSION,
            receipt_event_id: derive_receipt_event_id(
                &envelope.message_id,
                &envelope.recipient_agent,
                ReceiptKind::Delivered,
            )
            .unwrap(),
            message_id: envelope.message_id.clone(),
            recipient_agent: envelope.recipient_agent.clone(),
            envelope_version: envelope.version,
            cipher_suite: envelope.cipher_suite.clone(),
            envelope_commitment,
            kind: ReceiptKind::Delivered,
            occurred_at_micros: 35,
            agent_signature: vec![36; RECEIPT_V2_AGENT_SIGNATURE_BYTES],
            ml_dsa_signature: vec![37; ML_DSA_65_SIGNATURE_BYTES],
        }
    }

    fn receipt_v2() -> RecipientReceiptV2 {
        RecipientReceiptV2 {
            protocol_version: RECEIPT_V2_PROTOCOL_VERSION,
            receipt_event_id: derive_receipt_event_id(
                &MessageId([32; 32]),
                &[33; 39],
                ReceiptKind::Delivered,
            )
            .unwrap(),
            message_id: MessageId([32; 32]),
            recipient_agent: vec![33; 39],
            envelope_version: ENVELOPE_V2_HYBRID_PQC,
            cipher_suite: SUITE_X25519_MLKEM768_AES_256_GCM_AGENT_MLDSA65.into(),
            envelope_commitment: [34; 32],
            kind: ReceiptKind::Delivered,
            occurred_at_micros: 35,
            agent_signature: vec![36; RECEIPT_V2_AGENT_SIGNATURE_BYTES],
            ml_dsa_signature: vec![37; ML_DSA_65_SIGNATURE_BYTES],
        }
    }

    #[test]
    fn receipt_event_id_derivation_matches_frozen_sha256_vector() {
        let actual = derive_receipt_event_id(
            &MessageId([32; 32]),
            &[33; 39],
            ReceiptKind::Delivered,
        )
        .unwrap();
        let actual_hex = actual
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        assert_eq!(
            actual_hex,
            include_str!("../test-vectors/receipt-v2-event-id-delivered.sha256").trim()
        );
    }

    #[test]
    fn envelope_commitment_hash_matches_frozen_sha256_vector() {
        let actual = envelope_commitment_from_canonical_signing_bytes(b"abc").unwrap();
        let actual_hex = actual
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        assert_eq!(
            actual_hex,
            include_str!("../test-vectors/envelope-commitment-abc.sha256").trim()
        );
    }

    #[test]
    fn receipt_event_id_preimage_matches_frozen_vector() {
        let message_id = MessageId([32; 32]);
        let recipient = vec![33; 39];
        let preimage = canonical_receipt_event_id_preimage(
            &message_id,
            &recipient,
            ReceiptKind::Delivered,
        )
        .unwrap();
        let actual_hex = preimage
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        assert_eq!(
            actual_hex,
            include_str!("../test-vectors/receipt-v2-event-id-preimage-delivered.hex").trim()
        );
    }

    #[test]
    fn receipt_event_id_preimage_is_stable_but_scoped_to_message_recipient_and_kind() {
        let message_id = MessageId([32; 32]);
        let recipient = vec![33; 39];
        let first = canonical_receipt_event_id_preimage(
            &message_id,
            &recipient,
            ReceiptKind::Delivered,
        )
        .unwrap();
        assert_eq!(
            first,
            canonical_receipt_event_id_preimage(
                &message_id,
                &recipient,
                ReceiptKind::Delivered
            )
            .unwrap()
        );
        assert_ne!(
            first,
            canonical_receipt_event_id_preimage(
                &message_id,
                &recipient,
                ReceiptKind::Read
            )
            .unwrap()
        );
        assert_ne!(
            first,
            canonical_receipt_event_id_preimage(
                &message_id,
                &[34; 39],
                ReceiptKind::Delivered
            )
            .unwrap()
        );
        assert_ne!(
            first,
            canonical_receipt_event_id_preimage(
                &MessageId([35; 32]),
                &recipient,
                ReceiptKind::Delivered
            )
            .unwrap()
        );
    }

    #[test]
    fn v2_receipt_matches_frozen_canonical_byte_vector() {
        let actual = receipt_v2().canonical_signing_bytes().unwrap();
        let actual_hex = actual
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        assert_eq!(
            actual_hex,
            include_str!("../test-vectors/receipt-v2-delivered.hex").trim()
        );
    }

    #[test]
    fn v2_receipt_binds_event_message_recipient_envelope_and_state() {
        let first = receipt_v2();
        let canonical = first.canonical_signing_bytes().unwrap();

        let mut changed = first.clone();
        changed.receipt_event_id[0] ^= 1;
        assert_eq!(
            changed.canonical_signing_bytes(),
            Err(ProtocolError::ReceiptEventIdentityMismatch)
        );

        let mut changed = first.clone();
        changed.message_id = MessageId([41; 32]);
        assert_eq!(
            changed.canonical_signing_bytes(),
            Err(ProtocolError::ReceiptEventIdentityMismatch)
        );

        let mut changed = first.clone();
        changed.recipient_agent[0] ^= 1;
        assert_eq!(
            changed.canonical_signing_bytes(),
            Err(ProtocolError::ReceiptEventIdentityMismatch)
        );

        let mut changed = first.clone();
        changed.envelope_commitment[0] ^= 1;
        assert_ne!(canonical, changed.canonical_signing_bytes().unwrap());

        let mut changed = first.clone();
        changed.envelope_version = ENVELOPE_V1;
        changed.cipher_suite = SUITE_X25519_AES_256_GCM_ED25519.into();
        assert_eq!(
            changed.canonical_signing_bytes(),
            Err(ProtocolError::ReceiptContextMismatch)
        );

        let mut changed = first.clone();
        changed.kind = ReceiptKind::Read;
        assert_eq!(
            changed.canonical_signing_bytes(),
            Err(ProtocolError::ReceiptEventIdentityMismatch)
        );

        let mut changed = first.clone();
        changed.occurred_at_micros += 1;
        assert_ne!(canonical, changed.canonical_signing_bytes().unwrap());

        let mut changed = first.clone();
        changed.agent_signature = vec![99; RECEIPT_V2_AGENT_SIGNATURE_BYTES];
        assert_eq!(canonical, changed.canonical_signing_bytes().unwrap());

        let mut changed = first.clone();
        changed.ml_dsa_signature = vec![98; ML_DSA_65_SIGNATURE_BYTES];
        assert_eq!(canonical, changed.canonical_signing_bytes().unwrap());
    }

    #[test]
    fn v2_receipt_replay_classifier_distinguishes_duplicate_and_conflict() {
        let existing = receipt_v2();
        assert_eq!(
            classify_receipt_v2_replay(&existing, &existing).unwrap(),
            ReceiptReplayClassification::ExactReplay
        );

        // Signature bytes are not signed by themselves. Verify both signatures
        // before classification; alternate valid signature encodings for the
        // same transcript still refer to one logical receipt event.
        let mut alternate_signature = existing.clone();
        alternate_signature.recipient_signature = vec![99; 64];
        assert_eq!(
            classify_receipt_v2_replay(&existing, &alternate_signature).unwrap(),
            ReceiptReplayClassification::ExactReplay
        );

        let mut conflicting_payload = existing.clone();
        conflicting_payload.occurred_at_micros += 1;
        assert_eq!(
            classify_receipt_v2_replay(&existing, &conflicting_payload).unwrap(),
            ReceiptReplayClassification::ConflictingReuse
        );

        let mut conflicting_commitment = existing.clone();
        conflicting_commitment.envelope_commitment[0] ^= 1;
        assert_eq!(
            classify_receipt_v2_replay(&existing, &conflicting_commitment).unwrap(),
            ReceiptReplayClassification::ConflictingReuse
        );

        let mut different_identity = existing.clone();
        different_identity.receipt_event_id[0] ^= 1;
        assert_eq!(
            classify_receipt_v2_replay(&existing, &different_identity),
            Err(ProtocolError::ReceiptEventIdentityMismatch)
        );
    }

    #[test]
    fn v2_receipt_requires_both_ed25519_and_ml_dsa_signature_shapes() {
        let valid = receipt_v2();
        assert_eq!(valid.validate_signature_shape(), Ok(()));

        let mut invalid_agent = valid.clone();
        invalid_agent.agent_signature.pop();
        assert_eq!(
            invalid_agent.validate_signature_shape(),
            Err(ProtocolError::InvalidComponentLength {
                component: "receipt_agent_signature",
                expected: RECEIPT_V2_AGENT_SIGNATURE_BYTES,
                actual: RECEIPT_V2_AGENT_SIGNATURE_BYTES - 1,
            })
        );

        let mut invalid_ml_dsa = valid;
        invalid_ml_dsa.ml_dsa_signature.pop();
        assert_eq!(
            invalid_ml_dsa.validate_signature_shape(),
            Err(ProtocolError::InvalidComponentLength {
                component: "receipt_ml_dsa_signature",
                expected: ML_DSA_65_SIGNATURE_BYTES,
                actual: ML_DSA_65_SIGNATURE_BYTES - 1,
            })
        );
    }

    #[test]
    fn v2_receipt_validates_binding_to_exact_envelope_transcript() {
        let envelope = hybrid_envelope();
        let receipt = receipt_v2_for_envelope(&envelope);
        assert_eq!(receipt.validate_envelope_binding(&envelope), Ok(()));

        let mut changed_ciphertext = envelope.clone();
        changed_ciphertext.ciphertext[0] ^= 1;
        assert_eq!(
            receipt.validate_envelope_binding(&changed_ciphertext),
            Err(ProtocolError::ReceiptContextMismatch)
        );

        let mut changed_recipient = envelope.clone();
        changed_recipient.recipient_agent[0] ^= 1;
        assert_eq!(
            receipt.validate_envelope_binding(&changed_recipient),
            Err(ProtocolError::ReceiptContextMismatch)
        );

        let mut changed_message_id = envelope.clone();
        changed_message_id.message_id.0[0] ^= 1;
        assert_eq!(
            receipt.validate_envelope_binding(&changed_message_id),
            Err(ProtocolError::ReceiptContextMismatch)
        );

        let mut changed_suite = envelope;
        changed_suite.cipher_suite = SUITE_X25519_AES_256_GCM_ED25519.into();
        assert_eq!(
            receipt.validate_envelope_binding(&changed_suite),
            Err(ProtocolError::UnsupportedCipherSuite(
                SUITE_X25519_AES_256_GCM_ED25519.into()
            ))
        );
    }

    #[test]
    fn receipt_event_id_rejects_non_39_byte_recipient_identity() {
        assert_eq!(
            derive_receipt_event_id(
                &MessageId([32; 32]),
                &[33; 38],
                ReceiptKind::Delivered,
            ),
            Err(ProtocolError::InvalidComponentLength {
                component: "recipient_agent",
                expected: HOLOCHAIN_AGENT_PUBKEY_BYTES,
                actual: 38,
            })
        );
    }

    #[test]
    fn v2_receipt_rejects_arbitrary_event_ids_before_transcript_signing() {
        let mut value = receipt_v2();
        value.receipt_event_id = [0xA5; 32];
        assert_eq!(
            value.canonical_signing_bytes(),
            Err(ProtocolError::ReceiptEventIdentityMismatch)
        );
    }

    #[test]
    fn v2_receipt_rejects_unknown_version_and_profile_downgrade() {
        let mut value = receipt_v2();
        value.protocol_version = 3;
        assert_eq!(
            value.canonical_signing_bytes(),
            Err(ProtocolError::UnsupportedReceiptVersion(3))
        );

        let mut value = receipt_v2();
        value.cipher_suite = SUITE_X25519_AES_256_GCM_ED25519.into();
        assert_eq!(
            value.canonical_signing_bytes(),
            Err(ProtocolError::ReceiptContextMismatch)
        );

        let mut value = receipt_v2();
        value.envelope_version = ENVELOPE_V1;
        value.cipher_suite = SUITE_X25519_AES_256_GCM_ED25519.into();
        assert_eq!(
            value.canonical_signing_bytes(),
            Err(ProtocolError::ReceiptContextMismatch)
        );

        let mut value = receipt_v2();
        value.envelope_version = 99;
        assert_eq!(
            value.canonical_signing_bytes(),
            Err(ProtocolError::UnsupportedEnvelopeVersion(99))
        );
    }

    #[test]
    fn envelope_commitment_preimage_is_domain_separated_and_length_prefixed() {
        let first = canonical_envelope_commitment_preimage(b"abc").unwrap();
        let second = canonical_envelope_commitment_preimage(b"abd").unwrap();
        assert_ne!(first, second);
        assert!(first.starts_with(ENVELOPE_COMMITMENT_DOMAIN));
        assert!(first.ends_with(&[0, 0, 0, 3, b'a', b'b', b'c']));
    }

    #[test]
    fn v1_receipt_transcript_remains_separate_from_v2() {
        let legacy = RecipientReceipt {
            message_id: MessageId([1; 32]),
            recipient_agent: vec![2; 39],
            kind: ReceiptKind::Delivered,
            occurred_at_micros: 3,
            recipient_signature: vec![4; 64],
        };
        let current = receipt_v2();
        assert!(legacy.canonical_signing_bytes().unwrap().starts_with(RECEIPT_DOMAIN));
        assert!(current.canonical_signing_bytes().unwrap().starts_with(RECEIPT_V2_DOMAIN));
    }

}
