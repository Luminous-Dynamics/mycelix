// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Externally signed authority-write leases.
//!
//! PostgreSQL advisory locks serialize cooperating writers, but they cannot
//! fence an isolated former primary. A short-lived authority-write lease adds
//! an external, cryptographically verifiable prerequisite to every canonical
//! mutation. The lease binds one deployment, database identity, PostgreSQL
//! timeline, monotonic generation, governed database epoch, and operation set.
//! Once the lease expires, a disconnected former primary cannot continue
//! writing unless the independent lease issuer deliberately renews it.

use crate::authority_signing::AuthoritySigner;
use crate::scientific_events::ContentHash;
use crate::{Error, Result};
use async_trait::async_trait;
use chrono::{DateTime, Duration as ChronoDuration, Utc};
use ed25519_dalek::{Signature, Signer, SigningKey, Verifier, VerifyingKey};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use tokio::sync::RwLock;
use uuid::Uuid;

pub const AUTHORITY_WRITE_LEASE_PROTOCOL: &str = "mycelix-desci-authority-write-lease";
pub const AUTHORITY_WRITE_LEASE_PROTOCOL_VERSION: u16 = 1;
pub const AUTHORITY_WRITE_LEASE_SCHEMA_VERSION: u16 = 1;
pub const AUTHORITY_WRITE_LEASE_CODEC: &str = "mycelix-canonical-binary-v1";
pub const MAX_AUTHORITY_WRITE_LEASE_FILE_BYTES: u64 = 1024 * 1024;
pub const MAX_AUTHORITY_WRITE_LEASE_DURATION_SECONDS: i64 = 15 * 60;
pub const DEFAULT_AUTHORITY_WRITE_LEASE_CLOCK_SKEW_SECONDS: i64 = 30;

const MAX_IDENTIFIER_BYTES: usize = 256;
const MAX_REASON_BYTES: usize = 4_096;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthorityWriteScope {
    ScientificEvent,
    CredentialRegistry,
    CredentialGovernance,
    DatabaseEpochPromotion,
    RecoveryReconciliation,
    DeliveryAcknowledgement,
    CheckpointMirror,
    OutboxDelivery,
    SchemaMigration,
}

impl AuthorityWriteScope {
    fn code(self) -> u8 {
        match self {
            Self::ScientificEvent => 1,
            Self::CredentialRegistry => 2,
            Self::CredentialGovernance => 3,
            Self::DatabaseEpochPromotion => 4,
            Self::RecoveryReconciliation => 5,
            Self::DeliveryAcknowledgement => 6,
            Self::CheckpointMirror => 7,
            Self::OutboxDelivery => 8,
            Self::SchemaMigration => 9,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthorityWriteLeasePhase {
    /// Used only before the first governed database epoch exists.
    Bootstrap,
    /// Bound to an exact governed database epoch.
    Epoch,
}

impl AuthorityWriteLeasePhase {
    fn code(self) -> u8 {
        match self {
            Self::Bootstrap => 1,
            Self::Epoch => 2,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AuthorityWriteLease {
    pub lease_id: Uuid,
    pub deployment_id: String,
    pub primary_id: String,
    pub database_system_identifier: String,
    pub postgres_timeline: u64,
    pub generation: u64,
    pub phase: AuthorityWriteLeasePhase,
    pub epoch_number: Option<u64>,
    pub epoch_hash: Option<ContentHash>,
    pub allowed_scopes: BTreeSet<AuthorityWriteScope>,
    pub issued_at: DateTime<Utc>,
    pub not_before: DateTime<Utc>,
    pub expires_at: DateTime<Utc>,
    pub reason: String,
}

impl AuthorityWriteLease {
    pub fn validate(&self) -> Result<()> {
        validate_text(&self.deployment_id, MAX_IDENTIFIER_BYTES, "deployment id")?;
        validate_text(&self.primary_id, MAX_IDENTIFIER_BYTES, "primary id")?;
        validate_text(
            &self.database_system_identifier,
            MAX_IDENTIFIER_BYTES,
            "database system identifier",
        )?;
        validate_text(&self.reason, MAX_REASON_BYTES, "write lease reason")?;
        if self.postgres_timeline == 0 || self.generation == 0 {
            return Err(Error::Validation(
                "authority write lease timeline and generation must be positive".to_string(),
            ));
        }
        if self.allowed_scopes.is_empty() {
            return Err(Error::Validation(
                "authority write lease must authorize at least one write scope".to_string(),
            ));
        }
        if self.not_before < self.issued_at {
            return Err(Error::Validation(
                "authority write lease cannot become active before it was issued".to_string(),
            ));
        }
        if self.expires_at <= self.not_before
            || self.expires_at - self.not_before
                > ChronoDuration::seconds(MAX_AUTHORITY_WRITE_LEASE_DURATION_SECONDS)
        {
            return Err(Error::Validation(format!(
                "authority write lease must expire within {MAX_AUTHORITY_WRITE_LEASE_DURATION_SECONDS} seconds of activation"
            )));
        }
        match self.phase {
            AuthorityWriteLeasePhase::Bootstrap => {
                if self.epoch_number.is_some() || self.epoch_hash.is_some() {
                    return Err(Error::Validation(
                        "bootstrap write lease must not name a database epoch".to_string(),
                    ));
                }
            }
            AuthorityWriteLeasePhase::Epoch => {
                if self.epoch_number.is_none_or(|number| number == 0) || self.epoch_hash.is_none() {
                    return Err(Error::Validation(
                        "epoch write lease must name a positive epoch number and hash".to_string(),
                    ));
                }
            }
        }
        Ok(())
    }

    pub fn validate_at(&self, now: DateTime<Utc>, clock_skew: ChronoDuration) -> Result<()> {
        self.validate()?;
        if clock_skew < ChronoDuration::zero() || clock_skew > ChronoDuration::minutes(5) {
            return Err(Error::Validation(
                "authority write lease clock skew must be between zero and five minutes"
                    .to_string(),
            ));
        }
        if now + clock_skew < self.not_before {
            return Err(Error::VerificationFailed(
                "authority write lease is not active yet".to_string(),
            ));
        }
        // Expiration is deliberately strict. Clock skew may admit a lease a
        // little before `not_before`, but it must never extend the period in
        // which an isolated former primary can continue writing.
        if now >= self.expires_at {
            return Err(Error::VerificationFailed(
                "authority write lease has expired".to_string(),
            ));
        }
        Ok(())
    }

    pub fn permits(&self, scope: AuthorityWriteScope) -> Result<()> {
        if self.allowed_scopes.contains(&scope) {
            Ok(())
        } else {
            Err(Error::VerificationFailed(format!(
                "authority write lease does not permit {scope:?} mutations"
            )))
        }
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>> {
        self.validate()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-AUTHORITY-WRITE-LEASE\0");
        bytes.extend_from_slice(self.lease_id.as_bytes());
        push_text(&mut bytes, &self.deployment_id)?;
        push_text(&mut bytes, &self.primary_id)?;
        push_text(&mut bytes, &self.database_system_identifier)?;
        push_u64(&mut bytes, self.postgres_timeline);
        push_u64(&mut bytes, self.generation);
        bytes.push(self.phase.code());
        push_option_u64(&mut bytes, self.epoch_number);
        push_option_hash(&mut bytes, self.epoch_hash);
        push_len(&mut bytes, self.allowed_scopes.len())?;
        for scope in &self.allowed_scopes {
            bytes.push(scope.code());
        }
        push_datetime(&mut bytes, &self.issued_at);
        push_datetime(&mut bytes, &self.not_before);
        push_datetime(&mut bytes, &self.expires_at);
        push_text(&mut bytes, &self.reason)?;
        Ok(bytes)
    }

    pub fn lease_hash(&self) -> Result<ContentHash> {
        Ok(ContentHash::digest(&self.canonical_bytes()?))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignedAuthorityWriteLease {
    pub protocol: String,
    pub protocol_version: u16,
    pub codec: String,
    pub schema_version: u16,
    pub lease: AuthorityWriteLease,
    pub signer_key_id: String,
    pub signer_public_key: [u8; 32],
    pub signature: Vec<u8>,
}

impl SignedAuthorityWriteLease {
    pub fn sign(lease: AuthorityWriteLease, key: &SigningKey) -> Result<Self> {
        let signer_public_key = key.verifying_key().to_bytes();
        let mut signed = Self {
            protocol: AUTHORITY_WRITE_LEASE_PROTOCOL.to_string(),
            protocol_version: AUTHORITY_WRITE_LEASE_PROTOCOL_VERSION,
            codec: AUTHORITY_WRITE_LEASE_CODEC.to_string(),
            schema_version: AUTHORITY_WRITE_LEASE_SCHEMA_VERSION,
            lease,
            signer_key_id: "software-ed25519".to_string(),
            signer_public_key,
            signature: Vec::new(),
        };
        signed.signature = key.sign(&signed.signing_bytes()?).to_bytes().to_vec();
        signed.verify_signature()?;
        Ok(signed)
    }

    pub fn sign_with(lease: AuthorityWriteLease, signer: &dyn AuthoritySigner) -> Result<Self> {
        let mut signed = Self {
            protocol: AUTHORITY_WRITE_LEASE_PROTOCOL.to_string(),
            protocol_version: AUTHORITY_WRITE_LEASE_PROTOCOL_VERSION,
            codec: AUTHORITY_WRITE_LEASE_CODEC.to_string(),
            schema_version: AUTHORITY_WRITE_LEASE_SCHEMA_VERSION,
            lease,
            signer_key_id: signer.key_id().to_string(),
            signer_public_key: signer.verifying_key().to_bytes(),
            signature: Vec::new(),
        };
        signed.signature = signer.sign_message(&signed.signing_bytes()?)?;
        signed.verify_signature()?;
        Ok(signed)
    }

    pub fn signing_bytes(&self) -> Result<Vec<u8>> {
        validate_text(
            &self.signer_key_id,
            MAX_IDENTIFIER_BYTES,
            "lease signer key id",
        )?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-SIGNED-AUTHORITY-WRITE-LEASE\0");
        push_text(&mut bytes, &self.protocol)?;
        push_u16(&mut bytes, self.protocol_version);
        push_text(&mut bytes, &self.codec)?;
        push_u16(&mut bytes, self.schema_version);
        push_bytes(&mut bytes, &self.lease.canonical_bytes()?)?;
        push_text(&mut bytes, &self.signer_key_id)?;
        bytes.extend_from_slice(&self.signer_public_key);
        Ok(bytes)
    }

    pub fn verify_signature(&self) -> Result<()> {
        if self.protocol != AUTHORITY_WRITE_LEASE_PROTOCOL
            || self.protocol_version != AUTHORITY_WRITE_LEASE_PROTOCOL_VERSION
            || self.codec != AUTHORITY_WRITE_LEASE_CODEC
            || self.schema_version != AUTHORITY_WRITE_LEASE_SCHEMA_VERSION
        {
            return Err(Error::VerificationFailed(
                "unsupported authority write lease protocol envelope".to_string(),
            ));
        }
        self.lease.validate()?;
        let key = VerifyingKey::from_bytes(&self.signer_public_key)
            .map_err(|error| Error::Crypto(error.to_string()))?;
        let signature = Signature::try_from(self.signature.as_slice())
            .map_err(|error| Error::Crypto(error.to_string()))?;
        key.verify(&self.signing_bytes()?, &signature)
            .map_err(|error| Error::VerificationFailed(error.to_string()))
    }

    pub fn verify_at(
        &self,
        trusted_keys: &BTreeSet<[u8; 32]>,
        now: DateTime<Utc>,
        clock_skew: ChronoDuration,
    ) -> Result<()> {
        self.verify_signature()?;
        if !trusted_keys.contains(&self.signer_public_key) {
            return Err(Error::VerificationFailed(
                "authority write lease signer is not trusted".to_string(),
            ));
        }
        self.lease.validate_at(now, clock_skew)
    }

    pub fn signed_lease_hash(&self) -> Result<ContentHash> {
        self.verify_signature()?;
        let mut bytes = self.signing_bytes()?;
        push_bytes(&mut bytes, &self.signature)?;
        Ok(ContentHash::digest(&bytes))
    }
}

#[async_trait]
pub trait AuthorityWriteLeaseProvider: Send + Sync {
    async fn current_lease(&self) -> Result<SignedAuthorityWriteLease>;
}

#[derive(Clone)]
pub struct StaticAuthorityWriteLeaseProvider {
    lease: Arc<RwLock<SignedAuthorityWriteLease>>,
}

impl StaticAuthorityWriteLeaseProvider {
    pub fn new(lease: SignedAuthorityWriteLease) -> Self {
        Self {
            lease: Arc::new(RwLock::new(lease)),
        }
    }

    pub async fn replace(&self, lease: SignedAuthorityWriteLease) {
        *self.lease.write().await = lease;
    }
}

#[async_trait]
impl AuthorityWriteLeaseProvider for StaticAuthorityWriteLeaseProvider {
    async fn current_lease(&self) -> Result<SignedAuthorityWriteLease> {
        Ok(self.lease.read().await.clone())
    }
}

#[derive(Debug, Clone)]
pub struct FileAuthorityWriteLeaseProvider {
    path: PathBuf,
}

impl FileAuthorityWriteLeaseProvider {
    pub fn new(path: impl Into<PathBuf>) -> Result<Self> {
        let path = path.into();
        if path.as_os_str().is_empty() {
            return Err(Error::Validation(
                "authority write lease path cannot be empty".to_string(),
            ));
        }
        Ok(Self { path })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }
}

#[async_trait]
impl AuthorityWriteLeaseProvider for FileAuthorityWriteLeaseProvider {
    async fn current_lease(&self) -> Result<SignedAuthorityWriteLease> {
        let metadata = tokio::fs::symlink_metadata(&self.path)
            .await
            .map_err(|error| Error::Storage(error.to_string()))?;
        if metadata.file_type().is_symlink() || !metadata.is_file() {
            return Err(Error::Storage(format!(
                "authority write lease must be a regular non-symbolic-link file: {}",
                self.path.display()
            )));
        }
        if metadata.len() == 0 || metadata.len() > MAX_AUTHORITY_WRITE_LEASE_FILE_BYTES {
            return Err(Error::Storage(format!(
                "authority write lease file must contain 1-{MAX_AUTHORITY_WRITE_LEASE_FILE_BYTES} bytes"
            )));
        }
        let bytes = tokio::fs::read(&self.path)
            .await
            .map_err(|error| Error::Storage(error.to_string()))?;
        serde_json::from_slice(&bytes).map_err(Error::from)
    }
}

fn validate_text(value: &str, max_bytes: usize, label: &str) -> Result<()> {
    let value = value.trim();
    if value.is_empty() || value.len() > max_bytes || value.chars().any(char::is_control) {
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
    push_len(bytes, value.len())?;
    bytes.extend_from_slice(value);
    Ok(())
}

fn push_len(bytes: &mut Vec<u8>, value: usize) -> Result<()> {
    let value = u32::try_from(value).map_err(|_| {
        Error::Validation("canonical authority-write lease field exceeds u32 length".to_string())
    })?;
    bytes.extend_from_slice(&value.to_be_bytes());
    Ok(())
}

fn push_u16(bytes: &mut Vec<u8>, value: u16) {
    bytes.extend_from_slice(&value.to_be_bytes());
}

fn push_u64(bytes: &mut Vec<u8>, value: u64) {
    bytes.extend_from_slice(&value.to_be_bytes());
}

fn push_option_u64(bytes: &mut Vec<u8>, value: Option<u64>) {
    match value {
        Some(value) => {
            bytes.push(1);
            push_u64(bytes, value);
        }
        None => bytes.push(0),
    }
}

fn push_option_hash(bytes: &mut Vec<u8>, value: Option<ContentHash>) {
    match value {
        Some(value) => {
            bytes.push(1);
            bytes.extend_from_slice(&value.0);
        }
        None => bytes.push(0),
    }
}

fn push_datetime(bytes: &mut Vec<u8>, value: &DateTime<Utc>) {
    bytes.extend_from_slice(&value.timestamp().to_be_bytes());
    bytes.extend_from_slice(&value.timestamp_subsec_nanos().to_be_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lease(key: &SigningKey) -> SignedAuthorityWriteLease {
        SignedAuthorityWriteLease::sign(
            AuthorityWriteLease {
                lease_id: Uuid::nil(),
                deployment_id: "deployment:test".to_string(),
                primary_id: "postgres-a".to_string(),
                database_system_identifier: "731245".to_string(),
                postgres_timeline: 2,
                generation: 7,
                phase: AuthorityWriteLeasePhase::Epoch,
                epoch_number: Some(3),
                epoch_hash: Some(ContentHash([3; 32])),
                allowed_scopes: [
                    AuthorityWriteScope::ScientificEvent,
                    AuthorityWriteScope::CredentialGovernance,
                ]
                .into_iter()
                .collect(),
                issued_at: DateTime::from_timestamp(1_700_000_000, 0).unwrap(),
                not_before: DateTime::from_timestamp(1_700_000_001, 0).unwrap(),
                expires_at: DateTime::from_timestamp(1_700_000_301, 0).unwrap(),
                reason: "planned primary lease".to_string(),
            },
            key,
        )
        .unwrap()
    }

    #[test]
    fn signed_lease_binds_epoch_scope_and_expiry() {
        let key = SigningKey::from_bytes(&[81; 32]);
        let mut signed = lease(&key);
        let trusted = [key.verifying_key().to_bytes()].into_iter().collect();
        signed
            .verify_at(
                &trusted,
                DateTime::from_timestamp(1_700_000_100, 0).unwrap(),
                ChronoDuration::seconds(0),
            )
            .unwrap();
        signed
            .lease
            .permits(AuthorityWriteScope::ScientificEvent)
            .unwrap();
        assert!(
            signed
                .lease
                .permits(AuthorityWriteScope::RecoveryReconciliation)
                .is_err()
        );
        assert!(
            signed
                .verify_at(
                    &trusted,
                    DateTime::from_timestamp(1_700_000_301, 0).unwrap(),
                    ChronoDuration::seconds(0),
                )
                .is_err()
        );
        assert!(
            signed
                .verify_at(
                    &trusted,
                    DateTime::from_timestamp(1_700_000_301, 0).unwrap(),
                    ChronoDuration::seconds(30),
                )
                .is_err()
        );
        signed.lease.generation += 1;
        assert!(signed.verify_signature().is_err());
    }

    #[tokio::test]
    async fn static_provider_rotates_atomically() {
        let first_key = SigningKey::from_bytes(&[82; 32]);
        let second_key = SigningKey::from_bytes(&[83; 32]);
        let provider = StaticAuthorityWriteLeaseProvider::new(lease(&first_key));
        assert_eq!(provider.current_lease().await.unwrap().lease.generation, 7);
        let mut replacement = lease(&second_key);
        replacement.lease.generation = 8;
        replacement = SignedAuthorityWriteLease::sign(replacement.lease, &second_key).unwrap();
        provider.replace(replacement).await;
        assert_eq!(provider.current_lease().await.unwrap().lease.generation, 8);
    }
}
