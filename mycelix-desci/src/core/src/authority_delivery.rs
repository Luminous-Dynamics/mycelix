// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Signed delivery envelopes for durable authority outbox publication.
//!
//! Database leases and HTTP idempotency make publication operationally safe,
//! but they do not prove that a payload was emitted by an authorized Mycelix
//! authority service. This module binds the immutable payload hash, aggregate
//! position, server creation time, and delivery identifier under a dedicated
//! Ed25519 key that is distinct from actor and authority-receipt keys.

use crate::authority_signing::AuthoritySigner;
use crate::scientific_events::ContentHash;
use crate::{Error, Result};
use chrono::{DateTime, Utc};
use ed25519_dalek::{Signature, SigningKey, Verifier, VerifyingKey};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

pub const AUTHORITY_DELIVERY_PROTOCOL: &str = "mycelix-desci-authority-delivery";
pub const AUTHORITY_DELIVERY_PROTOCOL_VERSION: u16 = 1;
pub const AUTHORITY_DELIVERY_SCHEMA_VERSION: u16 = 1;
pub const AUTHORITY_DELIVERY_CODEC: &str = "mycelix-canonical-binary-v1";
const MAX_TOPIC_BYTES: usize = 128;
const MAX_AGGREGATE_ID_BYTES: usize = 256;
const MAX_PAYLOAD_BYTES: usize = 4 * 1024 * 1024;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AuthorityDeliveryEnvelope {
    pub protocol: String,
    pub protocol_version: u16,
    pub codec: String,
    pub schema_version: u16,
    pub delivery_id: Uuid,
    pub topic: String,
    pub aggregate_id: String,
    pub aggregate_sequence: i64,
    pub created_at: DateTime<Utc>,
    pub payload_hash: ContentHash,
    pub payload: serde_json::Value,
}

impl AuthorityDeliveryEnvelope {
    pub fn new(
        delivery_id: Uuid,
        topic: impl Into<String>,
        aggregate_id: impl Into<String>,
        aggregate_sequence: i64,
        created_at: DateTime<Utc>,
        payload: serde_json::Value,
    ) -> Result<Self> {
        let payload_bytes = serde_json::to_vec(&payload)?;
        let envelope = Self {
            protocol: AUTHORITY_DELIVERY_PROTOCOL.to_string(),
            protocol_version: AUTHORITY_DELIVERY_PROTOCOL_VERSION,
            codec: AUTHORITY_DELIVERY_CODEC.to_string(),
            schema_version: AUTHORITY_DELIVERY_SCHEMA_VERSION,
            delivery_id,
            topic: topic.into(),
            aggregate_id: aggregate_id.into(),
            aggregate_sequence,
            created_at,
            payload_hash: ContentHash::digest(&payload_bytes),
            payload,
        };
        envelope.validate()?;
        Ok(envelope)
    }

    pub fn validate(&self) -> Result<()> {
        if self.protocol != AUTHORITY_DELIVERY_PROTOCOL
            || self.protocol_version != AUTHORITY_DELIVERY_PROTOCOL_VERSION
            || self.codec != AUTHORITY_DELIVERY_CODEC
            || self.schema_version != AUTHORITY_DELIVERY_SCHEMA_VERSION
        {
            return Err(Error::Validation(
                "unsupported authority delivery protocol".to_string(),
            ));
        }
        validate_text(&self.topic, MAX_TOPIC_BYTES, "delivery topic")?;
        validate_text(
            &self.aggregate_id,
            MAX_AGGREGATE_ID_BYTES,
            "delivery aggregate id",
        )?;
        if self.aggregate_sequence < 0 {
            return Err(Error::Validation(
                "delivery aggregate sequence cannot be negative".to_string(),
            ));
        }
        let payload_bytes = serde_json::to_vec(&self.payload)?;
        if payload_bytes.len() > MAX_PAYLOAD_BYTES {
            return Err(Error::Validation(format!(
                "delivery payload exceeds {MAX_PAYLOAD_BYTES} bytes"
            )));
        }
        if ContentHash::digest(&payload_bytes) != self.payload_hash {
            return Err(Error::VerificationFailed(
                "delivery payload does not match its committed hash".to_string(),
            ));
        }
        Ok(())
    }

    pub fn signing_bytes(&self) -> Result<Vec<u8>> {
        self.validate()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-AUTHORITY-DELIVERY\0");
        push_text(&mut bytes, &self.protocol)?;
        bytes.extend_from_slice(&self.protocol_version.to_be_bytes());
        push_text(&mut bytes, &self.codec)?;
        bytes.extend_from_slice(&self.schema_version.to_be_bytes());
        bytes.extend_from_slice(self.delivery_id.as_bytes());
        push_text(&mut bytes, &self.topic)?;
        push_text(&mut bytes, &self.aggregate_id)?;
        bytes.extend_from_slice(&self.aggregate_sequence.to_be_bytes());
        bytes.extend_from_slice(&self.created_at.timestamp().to_be_bytes());
        bytes.extend_from_slice(&self.created_at.timestamp_subsec_nanos().to_be_bytes());
        bytes.extend_from_slice(&self.payload_hash.0);
        Ok(bytes)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignedAuthorityDeliveryEnvelope {
    pub envelope: AuthorityDeliveryEnvelope,
    pub signer_public_key: [u8; 32],
    pub signature: Vec<u8>,
}

impl SignedAuthorityDeliveryEnvelope {
    pub fn sign(envelope: AuthorityDeliveryEnvelope, key: &SigningKey) -> Result<Self> {
        Self::sign_with(envelope, key)
    }

    pub fn sign_with(
        envelope: AuthorityDeliveryEnvelope,
        signer: &dyn AuthoritySigner,
    ) -> Result<Self> {
        envelope.validate()?;
        let signing_bytes = envelope.signing_bytes()?;
        let signature = signer.sign_message(&signing_bytes)?;
        let signed = Self {
            signer_public_key: signer.verifying_key().to_bytes(),
            signature,
            envelope,
        };
        // Hardware and remote signers are untrusted protocol participants until
        // their returned signature is independently checked.
        signed.verify()?;
        Ok(signed)
    }

    pub fn verify(&self) -> Result<()> {
        self.envelope.validate()?;
        let key = VerifyingKey::from_bytes(&self.signer_public_key)
            .map_err(|error| Error::Crypto(error.to_string()))?;
        let signature = Signature::try_from(self.signature.as_slice())
            .map_err(|error| Error::Crypto(error.to_string()))?;
        key.verify(&self.envelope.signing_bytes()?, &signature)
            .map_err(|error| Error::VerificationFailed(error.to_string()))
    }

    pub fn delivery_hash(&self) -> Result<ContentHash> {
        self.verify()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-SIGNED-AUTHORITY-DELIVERY\0");
        let signing_bytes = self.envelope.signing_bytes()?;
        push_bytes(&mut bytes, &signing_bytes)?;
        bytes.extend_from_slice(&self.signer_public_key);
        push_bytes(&mut bytes, &self.signature)?;
        Ok(ContentHash::digest(&bytes))
    }
}

fn validate_text(value: &str, max_bytes: usize, label: &str) -> Result<()> {
    let trimmed = value.trim();
    if trimmed.is_empty() || trimmed.len() > max_bytes || trimmed.chars().any(char::is_control) {
        return Err(Error::Validation(format!(
            "{label} must contain 1-{max_bytes} printable bytes"
        )));
    }
    Ok(())
}

fn push_text(bytes: &mut Vec<u8>, value: &str) -> Result<()> {
    push_bytes(bytes, value.as_bytes())
}

fn push_bytes(bytes: &mut Vec<u8>, value: &[u8]) -> Result<()> {
    let len = u32::try_from(value.len()).map_err(|_| {
        Error::Validation("authority delivery field exceeds canonical codec limits".to_string())
    })?;
    bytes.extend_from_slice(&len.to_be_bytes());
    bytes.extend_from_slice(value);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn delivery_signature_binds_payload_and_position() {
        let key = SigningKey::from_bytes(&[7; 32]);
        let envelope = AuthorityDeliveryEnvelope::new(
            Uuid::nil(),
            "scientific.event.committed",
            Uuid::nil().to_string(),
            0,
            DateTime::from_timestamp(1_700_000_000, 0).unwrap(),
            serde_json::json!({"event": "example"}),
        )
        .unwrap();
        let mut signed = SignedAuthorityDeliveryEnvelope::sign(envelope, &key).unwrap();
        signed.verify().unwrap();
        signed.envelope.aggregate_sequence = 1;
        assert!(signed.verify().is_err());
    }

    struct InvalidRemoteSigner {
        key: SigningKey,
    }

    impl AuthoritySigner for InvalidRemoteSigner {
        fn key_id(&self) -> &str {
            "invalid-remote-signer"
        }

        fn verifying_key(&self) -> VerifyingKey {
            self.key.verifying_key()
        }

        fn sign_message(&self, _message: &[u8]) -> Result<Vec<u8>> {
            Ok(vec![0; 64])
        }
    }

    #[test]
    fn delivery_rejects_an_invalid_remote_signer_response() {
        let signer = InvalidRemoteSigner {
            key: SigningKey::from_bytes(&[9; 32]),
        };
        let envelope = AuthorityDeliveryEnvelope::new(
            Uuid::new_v4(),
            "authority.database-epoch.recorded.v1",
            "deployment:test",
            1,
            DateTime::from_timestamp(1_700_000_000, 0).unwrap(),
            serde_json::json!({"epoch": 1}),
        )
        .unwrap();
        assert!(SignedAuthorityDeliveryEnvelope::sign_with(envelope, &signer).is_err());
    }

    #[test]
    fn delivery_rejects_payload_hash_mismatch() {
        let key = SigningKey::from_bytes(&[8; 32]);
        let envelope = AuthorityDeliveryEnvelope::new(
            Uuid::nil(),
            "credential.recorded",
            "credential-registry",
            0,
            DateTime::from_timestamp(1_700_000_000, 0).unwrap(),
            serde_json::json!({"record": 1}),
        )
        .unwrap();
        let mut signed = SignedAuthorityDeliveryEnvelope::sign(envelope, &key).unwrap();
        signed.envelope.payload = serde_json::json!({"record": 2});
        assert!(signed.verify().is_err());
    }
}
