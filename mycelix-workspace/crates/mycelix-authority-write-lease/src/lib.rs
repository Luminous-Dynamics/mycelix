//! Minimal, wire-compatible verification for Mycelix signed PostgreSQL write leases.
//!
//! This crate intentionally contains no database client, async runtime, CLI,
//! HTTP client, or application domain dependencies. It preserves the v1
//! canonical byte format used by mycelix-desci-core so independent effect
//! owners can authenticate the same signed lease without depending on the
//! complete DeSci application crate.

use chrono::{DateTime, Duration, Utc};
use ed25519_dalek::{Signature, Verifier, VerifyingKey};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use uuid::Uuid;

pub const AUTHORITY_WRITE_LEASE_PROTOCOL: &str = "mycelix-desci-authority-write-lease";
pub const AUTHORITY_WRITE_LEASE_PROTOCOL_VERSION: u16 = 1;
pub const AUTHORITY_WRITE_LEASE_SCHEMA_VERSION: u16 = 1;
pub const AUTHORITY_WRITE_LEASE_CODEC: &str = "mycelix-canonical-binary-v1";
pub const MAX_AUTHORITY_WRITE_LEASE_DURATION_SECONDS: i64 = 15 * 60;
pub const DEFAULT_AUTHORITY_WRITE_LEASE_CLOCK_SKEW_SECONDS: i64 = 30;

const MAX_IDENTIFIER_BYTES: usize = 256;
const MAX_REASON_BYTES: usize = 4_096;

pub type LeaseResult<T> = Result<T, String>;

/// BLAKE3 commitment used by the existing authority-write lease wire format.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ContentHash(pub [u8; 32]);

impl ContentHash {
    pub fn digest(bytes: &[u8]) -> Self {
        Self(*blake3::hash(bytes).as_bytes())
    }

    pub fn to_hex(self) -> String {
        hex_encode(&self.0)
    }
}

/// Scopes have stable numeric codes in addition to their serde names.
/// Never renumber an existing variant or reuse a retired code.
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
    DirectoryFabricEffect,
}

impl AuthorityWriteScope {
    pub const fn stable_code(self) -> u8 {
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
            Self::DirectoryFabricEffect => 10,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthorityWriteLeasePhase {
    Bootstrap,
    Epoch,
}

impl AuthorityWriteLeasePhase {
    const fn stable_code(self) -> u8 {
        match self {
            Self::Bootstrap => 1,
            Self::Epoch => 2,
        }
    }
}

/// v1 lease payload. Field names, serde representation and canonical byte
/// order intentionally match mycelix-desci-core::AuthorityWriteLease.
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
    pub fn validate(&self) -> LeaseResult<()> {
        validate_text(&self.deployment_id, MAX_IDENTIFIER_BYTES, "deployment id")?;
        validate_text(&self.primary_id, MAX_IDENTIFIER_BYTES, "primary id")?;
        validate_text(
            &self.database_system_identifier,
            MAX_IDENTIFIER_BYTES,
            "database system identifier",
        )?;
        validate_text(&self.reason, MAX_REASON_BYTES, "write lease reason")?;

        if self.postgres_timeline == 0 || self.generation == 0 {
            return Err("authority write lease timeline and generation must be positive".into());
        }
        if self.allowed_scopes.is_empty() {
            return Err("authority write lease must authorize at least one write scope".into());
        }
        if self.not_before < self.issued_at {
            return Err("authority write lease cannot become active before it was issued".into());
        }
        if self.expires_at <= self.not_before
            || self.expires_at - self.not_before
                > Duration::seconds(MAX_AUTHORITY_WRITE_LEASE_DURATION_SECONDS)
        {
            return Err(format!(
                "authority write lease must expire within {MAX_AUTHORITY_WRITE_LEASE_DURATION_SECONDS} seconds of activation"
            ));
        }

        match self.phase {
            AuthorityWriteLeasePhase::Bootstrap => {
                if self.epoch_number.is_some() || self.epoch_hash.is_some() {
                    return Err("bootstrap write lease must not name a database epoch".into());
                }
            }
            AuthorityWriteLeasePhase::Epoch => {
                if self.epoch_number.is_none_or(|number| number == 0)
                    || self.epoch_hash.is_none()
                {
                    return Err(
                        "epoch write lease must name a positive epoch number and hash".into(),
                    );
                }
            }
        }
        Ok(())
    }

    pub fn validate_at(
        &self,
        now: DateTime<Utc>,
        clock_skew: Duration,
    ) -> LeaseResult<()> {
        self.validate()?;
        if clock_skew < Duration::zero() || clock_skew > Duration::minutes(5) {
            return Err("authority write lease clock skew must be between zero and five minutes".into());
        }
        if now + clock_skew < self.not_before {
            return Err("authority write lease is not active yet".into());
        }
        // Clock skew never extends expiry: an isolated former primary must
        // stop writing at the signed expires_at boundary.
        if now >= self.expires_at {
            return Err("authority write lease has expired".into());
        }
        Ok(())
    }

    pub fn permits(&self, scope: AuthorityWriteScope) -> LeaseResult<()> {
        if self.allowed_scopes.contains(&scope) {
            Ok(())
        } else {
            Err(format!("authority write lease does not permit {scope:?} mutations"))
        }
    }

    /// Exact v1 canonical payload bytes. Do not change without a new codec
    /// version and golden compatibility vectors.
    pub fn canonical_bytes(&self) -> LeaseResult<Vec<u8>> {
        self.validate()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-AUTHORITY-WRITE-LEASE\0");
        bytes.extend_from_slice(self.lease_id.as_bytes());
        push_text(&mut bytes, &self.deployment_id)?;
        push_text(&mut bytes, &self.primary_id)?;
        push_text(&mut bytes, &self.database_system_identifier)?;
        push_u64(&mut bytes, self.postgres_timeline);
        push_u64(&mut bytes, self.generation);
        bytes.push(self.phase.stable_code());
        push_option_u64(&mut bytes, self.epoch_number);
        push_option_hash(&mut bytes, self.epoch_hash);
        push_len(&mut bytes, self.allowed_scopes.len())?;
        for scope in &self.allowed_scopes {
            bytes.push(scope.stable_code());
        }
        push_datetime(&mut bytes, &self.issued_at);
        push_datetime(&mut bytes, &self.not_before);
        push_datetime(&mut bytes, &self.expires_at);
        push_text(&mut bytes, &self.reason)?;
        Ok(bytes)
    }

    pub fn lease_hash(&self) -> LeaseResult<ContentHash> {
        Ok(ContentHash::digest(&self.canonical_bytes()?))
    }
}

/// Signed envelope kept JSON-compatible with the existing v1 DeSci lease.
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

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthorityWriteLeaseTarget {
    pub deployment_id: String,
    pub primary_id: String,
    pub database_system_identifier: String,
    pub postgres_timeline: u64,
    pub phase: AuthorityWriteLeasePhase,
    pub epoch_number: Option<u64>,
    pub epoch_hash: Option<ContentHash>,
}

impl SignedAuthorityWriteLease {
    /// Bytes covered by the Ed25519 signature, using the existing v1 envelope.
    pub fn signing_bytes(&self) -> LeaseResult<Vec<u8>> {
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

    pub fn verify_signature(&self) -> LeaseResult<()> {
        if self.protocol != AUTHORITY_WRITE_LEASE_PROTOCOL
            || self.protocol_version != AUTHORITY_WRITE_LEASE_PROTOCOL_VERSION
            || self.codec != AUTHORITY_WRITE_LEASE_CODEC
            || self.schema_version != AUTHORITY_WRITE_LEASE_SCHEMA_VERSION
        {
            return Err("unsupported authority write lease protocol envelope".into());
        }
        self.lease.validate()?;
        let key = VerifyingKey::from_bytes(&self.signer_public_key)
            .map_err(|error| format!("invalid authority lease public key: {error}"))?;
        let signature = Signature::try_from(self.signature.as_slice())
            .map_err(|error| format!("invalid authority lease signature encoding: {error}"))?;
        key.verify(&self.signing_bytes()?, &signature)
            .map_err(|error| format!("authority lease signature verification failed: {error}"))
    }

    pub fn verify_at(
        &self,
        trusted_keys: &BTreeSet<[u8; 32]>,
        now: DateTime<Utc>,
        clock_skew: Duration,
    ) -> LeaseResult<()> {
        self.verify_signature()?;
        if !trusted_keys.contains(&self.signer_public_key) {
            return Err("authority write lease signer is not trusted".into());
        }
        self.lease.validate_at(now, clock_skew)
    }

    pub fn verify_for_target(
        &self,
        trusted_keys: &BTreeSet<[u8; 32]>,
        now: DateTime<Utc>,
        clock_skew: Duration,
        required_scope: AuthorityWriteScope,
        target: &AuthorityWriteLeaseTarget,
    ) -> LeaseResult<()> {
        self.verify_at(trusted_keys, now, clock_skew)?;
        self.lease.permits(required_scope)?;
        if self.lease.deployment_id != target.deployment_id
            || self.lease.primary_id != target.primary_id
            || self.lease.database_system_identifier != target.database_system_identifier
            || self.lease.postgres_timeline != target.postgres_timeline
            || self.lease.phase != target.phase
            || self.lease.epoch_number != target.epoch_number
            || self.lease.epoch_hash != target.epoch_hash
        {
            return Err(
                "authority write lease does not bind the exact deployment, primary, database identity, timeline and epoch"
                    .into(),
            );
        }
        Ok(())
    }

    pub fn signed_lease_hash(&self) -> LeaseResult<ContentHash> {
        self.verify_signature()?;
        let mut bytes = self.signing_bytes()?;
        push_bytes(&mut bytes, &self.signature)?;
        Ok(ContentHash::digest(&bytes))
    }
}

fn validate_text(value: &str, max_bytes: usize, label: &str) -> LeaseResult<()> {
    let trimmed = value.trim();
    if trimmed.is_empty() || trimmed.len() > max_bytes || trimmed.chars().any(char::is_control) {
        return Err(format!("{label} must contain 1-{max_bytes} printable bytes"));
    }
    Ok(())
}

fn push_text(bytes: &mut Vec<u8>, value: &str) -> LeaseResult<()> {
    push_bytes(bytes, value.as_bytes())
}

fn push_bytes(bytes: &mut Vec<u8>, value: &[u8]) -> LeaseResult<()> {
    push_len(bytes, value.len())?;
    bytes.extend_from_slice(value);
    Ok(())
}

fn push_len(bytes: &mut Vec<u8>, value: usize) -> LeaseResult<()> {
    let value = u32::try_from(value)
        .map_err(|_| "canonical authority-write lease field exceeds u32 length".to_owned())?;
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

fn hex_encode(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer, SigningKey};

    fn signed_fixture() -> SignedAuthorityWriteLease {
        let key = SigningKey::from_bytes(&[84; 32]);
        let lease = AuthorityWriteLease {
            lease_id: Uuid::from_bytes([0x11; 16]),
            deployment_id: "deployment:test".into(),
            primary_id: "postgres-a".into(),
            database_system_identifier: "731245".into(),
            postgres_timeline: 2,
            generation: 7,
            phase: AuthorityWriteLeasePhase::Epoch,
            epoch_number: Some(3),
            epoch_hash: Some(ContentHash([3; 32])),
            allowed_scopes: [AuthorityWriteScope::DirectoryFabricEffect]
                .into_iter()
                .collect(),
            issued_at: DateTime::from_timestamp(1_700_000_000, 0).unwrap(),
            not_before: DateTime::from_timestamp(1_700_000_001, 0).unwrap(),
            expires_at: DateTime::from_timestamp(1_700_000_301, 0).unwrap(),
            reason: "test lease".into(),
        };
        let mut signed = SignedAuthorityWriteLease {
            protocol: AUTHORITY_WRITE_LEASE_PROTOCOL.into(),
            protocol_version: AUTHORITY_WRITE_LEASE_PROTOCOL_VERSION,
            codec: AUTHORITY_WRITE_LEASE_CODEC.into(),
            schema_version: AUTHORITY_WRITE_LEASE_SCHEMA_VERSION,
            lease,
            signer_key_id: "fixture-ed25519".into(),
            signer_public_key: key.verifying_key().to_bytes(),
            signature: Vec::new(),
        };
        signed.signature = key.sign(&signed.signing_bytes().unwrap()).to_bytes().to_vec();
        signed
    }

    #[test]
    fn v1_scope_codes_and_names_are_stable() {
        assert_eq!(AuthorityWriteScope::SchemaMigration.stable_code(), 9);
        assert_eq!(AuthorityWriteScope::DirectoryFabricEffect.stable_code(), 10);
        assert_eq!(
            serde_json::to_string(&AuthorityWriteScope::DirectoryFabricEffect).unwrap(),
            "\"directory_fabric_effect\""
        );
    }

    #[test]
    fn directory_fabric_scope_v1_matches_canonical_golden_vector() {
        let signed = signed_fixture();
        assert_eq!(
            hex_encode(&signed.lease.canonical_bytes().unwrap()),
            "4d5943454c49582d44455343492d415554484f524954592d57524954452d4c4541534500111111111111111111111111111111110000000f6465706c6f796d656e743a746573740000000a706f7374677265732d61000000063733313234350000000000000002000000000000000702010000000000000003010303030303030303030303030303030303030303030303030303030303030303000000010a000000006553f10000000000000000006553f10100000000000000006553f22d000000000000000a74657374206c65617365"
        );
    }

    #[test]
    fn signed_v1_lease_roundtrips_json_and_canonical_bytes() {
        let signed = signed_fixture();
        let bytes = signed.signing_bytes().unwrap();
        signed.verify_signature().unwrap();
        let json = serde_json::to_vec(&signed).unwrap();
        let decoded: SignedAuthorityWriteLease = serde_json::from_slice(&json).unwrap();
        assert_eq!(decoded, signed);
        assert_eq!(decoded.signing_bytes().unwrap(), bytes);
        assert_eq!(decoded.signed_lease_hash().unwrap(), signed.signed_lease_hash().unwrap());
    }

    #[test]
    fn verification_requires_trusted_key_scope_and_exact_target() {
        let signed = signed_fixture();
        let key = [signed.signer_public_key].into_iter().collect();
        let target = AuthorityWriteLeaseTarget {
            deployment_id: "deployment:test".into(),
            primary_id: "postgres-a".into(),
            database_system_identifier: "731245".into(),
            postgres_timeline: 2,
            phase: AuthorityWriteLeasePhase::Epoch,
            epoch_number: Some(3),
            epoch_hash: Some(ContentHash([3; 32])),
        };
        signed
            .verify_for_target(
                &key,
                DateTime::from_timestamp(1_700_000_100, 0).unwrap(),
                Duration::zero(),
                AuthorityWriteScope::DirectoryFabricEffect,
                &target,
            )
            .unwrap();

        let mut wrong_target = target.clone();
        wrong_target.postgres_timeline = 3;
        assert!(signed
            .verify_for_target(
                &key,
                DateTime::from_timestamp(1_700_000_100, 0).unwrap(),
                Duration::zero(),
                AuthorityWriteScope::DirectoryFabricEffect,
                &wrong_target,
            )
            .is_err());
        assert!(signed
            .verify_for_target(
                &BTreeSet::new(),
                DateTime::from_timestamp(1_700_000_100, 0).unwrap(),
                Duration::zero(),
                AuthorityWriteScope::DirectoryFabricEffect,
                &target,
            )
            .is_err());
        assert!(signed
            .verify_for_target(
                &key,
                DateTime::from_timestamp(1_700_000_100, 0).unwrap(),
                Duration::zero(),
                AuthorityWriteScope::DatabaseEpochPromotion,
                &target,
            )
            .is_err());
    }

    #[test]
    fn tampered_bytes_expired_lease_and_unsupported_protocol_reject() {
        let mut signed = signed_fixture();
        signed.lease.generation += 1;
        assert!(signed.verify_signature().is_err());

        let mut signed = signed_fixture();
        let keys = [signed.signer_public_key].into_iter().collect();
        assert!(signed
            .verify_at(
                &keys,
                DateTime::from_timestamp(1_700_000_301, 0).unwrap(),
                Duration::zero(),
            )
            .is_err());

        let mut signed = signed_fixture();
        signed.protocol_version += 1;
        assert!(signed.verify_signature().is_err());
    }
}
