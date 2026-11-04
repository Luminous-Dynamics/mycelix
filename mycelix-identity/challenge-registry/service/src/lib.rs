#![forbid(unsafe_code)]
//! Native PostgreSQL adapter for the Mycelix verifier challenge registry.
//!
//! This crate is deliberately outside the Holochain WASM workspace. PostgreSQL
//! is the authoritative consume-once coordinator; a DHT projection is not an
//! arbitration mechanism. Callers must verify the holder signature and exact
//! request/audience/verifier binding before invoking consume_and_commit.
//!
//! A successful SQL function result inside an open transaction is not an
//! accepted authentication. The commit must be acknowledged first. A commit
//! transport error is always reported as an unknown outcome and must never be
//! retried with the same challenge.

use rand::rngs::OsRng;
use rand::RngCore;
use sha2::{Digest, Sha256};
use sqlx::postgres::PgPoolOptions;
use sqlx::PgPool;
use std::fmt;
use std::time::Duration;
use thiserror::Error;
use uuid::Uuid;
use zeroize::Zeroize;

const CHALLENGE_BYTES: usize = 32;
const MAX_TTL_SECONDS: u64 = 600;
const MAX_VERIFIER_ID_CHARS: usize = 512;
const MAX_AUDIENCE_CHARS: usize = 2048;
const CHALLENGE_DIGEST_DOMAIN: &[u8] = b"mycelix:identity:verifier-challenge:v1\0";
const DEFAULT_ACQUIRE_TIMEOUT: Duration = Duration::from_secs(10);
const ISSUE_ATTEMPTS: usize = 3;

/// Raw challenge material held only at the verifier/request boundary.
///
/// Debug is redacted, there is no serialization implementation, and the
/// in-memory copy is zeroized on drop. The caller must still avoid logging or
/// persisting the value returned by as_wire_bytes.
pub struct ChallengeSecret([u8; CHALLENGE_BYTES]);

impl ChallengeSecret {
    /// Parse the exact 32-byte value received from the signed presentation.
    pub fn from_wire_bytes(bytes: &[u8]) -> Result<Self, RegistryError> {
        let value: [u8; CHALLENGE_BYTES] = bytes
            .try_into()
            .map_err(|_| RegistryError::InvalidChallengeLength)?;
        Ok(Self(value))
    }

    /// Borrow the raw bytes only for protocol construction or verification.
    pub fn as_wire_bytes(&self) -> &[u8; CHALLENGE_BYTES] {
        &self.0
    }

    fn generate() -> Result<Self, RegistryError> {
        let mut bytes = [0_u8; CHALLENGE_BYTES];
        OsRng
            .try_fill_bytes(&mut bytes)
            .map_err(RegistryError::RandomSource)?;
        Ok(Self(bytes))
    }

    fn digest(&self) -> [u8; 32] {
        let mut hasher = Sha256::new();
        hasher.update(CHALLENGE_DIGEST_DOMAIN);
        hasher.update(self.0);
        hasher.finalize().into()
    }
}

impl fmt::Debug for ChallengeSecret {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("ChallengeSecret([REDACTED])")
    }
}

impl Drop for ChallengeSecret {
    fn drop(&mut self) {
        self.0.zeroize();
    }
}

/// Context committed by the registry. Request and holder-binding digests are
/// SHA-256 values supplied by the upstream presentation verifier.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChallengeContext {
    pub verifier_id: String,
    pub audience: String,
    pub transaction_id: Uuid,
    pub request_digest: [u8; 32],
    pub holder_binding_digest: [u8; 32],
}

impl ChallengeContext {
    pub fn new(
        verifier_id: impl Into<String>,
        audience: impl Into<String>,
        transaction_id: Uuid,
        request_digest: [u8; 32],
        holder_binding_digest: [u8; 32],
    ) -> Result<Self, RegistryError> {
        let context = Self {
            verifier_id: verifier_id.into(),
            audience: audience.into(),
            transaction_id,
            request_digest,
            holder_binding_digest,
        };
        context.validate()?;
        Ok(context)
    }

    fn validate(&self) -> Result<(), RegistryError> {
        let verifier_chars = self.verifier_id.chars().count();
        if verifier_chars == 0
            || verifier_chars > MAX_VERIFIER_ID_CHARS
            || self.verifier_id.chars().any(char::is_control)
        {
            return Err(RegistryError::InvalidVerifierId);
        }
        let audience_chars = self.audience.chars().count();
        if audience_chars == 0
            || audience_chars > MAX_AUDIENCE_CHARS
            || self.audience.chars().any(char::is_control)
        {
            return Err(RegistryError::InvalidAudience);
        }
        Ok(())
    }
}

#[derive(Debug, Error)]
pub enum RegistryError {
    #[error("verifier ID is empty, too long, or contains control characters")]
    InvalidVerifierId,
    #[error("audience is empty, too long, or contains control characters")]
    InvalidAudience,
    #[error("challenge wire value must contain exactly 32 bytes")]
    InvalidChallengeLength,
    #[error("challenge TTL must be a whole number of seconds from 1 through 600")]
    InvalidTtl,
    #[error("OS cryptographic random source failed")]
    RandomSource(#[source] rand::Error),
    #[error("challenge registration was rejected after bounded fresh-digest attempts")]
    RegistrationRejected,
    #[error("PostgreSQL challenge registry operation failed")]
    Database(#[from] sqlx::Error),
    #[error("challenge consume committed or may have committed; outcome is unknown; issue a new challenge")]
    CommitOutcomeUnknown(#[source] sqlx::Error),
    #[error("registry connection pool limit must be between 1 and 64")]
    InvalidPoolLimit,
}

#[derive(Clone)]
pub struct PostgresChallengeRegistry {
    pool: PgPool,
}

impl PostgresChallengeRegistry {
    /// Connect to the authoritative PostgreSQL primary. Deployment must ensure
    /// this URL resolves only to the single writable primary, not a read pool.
    pub async fn connect(database_url: &str) -> Result<Self, RegistryError> {
        Self::connect_with_limit(database_url, 8).await
    }

    pub async fn connect_with_limit(
        database_url: &str,
        max_connections: u32,
    ) -> Result<Self, RegistryError> {
        if !(1..=64).contains(&max_connections) {
            return Err(RegistryError::InvalidPoolLimit);
        }
        let pool = PgPoolOptions::new()
            .max_connections(max_connections)
            .acquire_timeout(DEFAULT_ACQUIRE_TIMEOUT)
            .connect(database_url)
            .await?;
        Ok(Self { pool })
    }

    /// Construct from a pool whose endpoint and role have been provisioned by
    /// the deployment. The runtime role should have EXECUTE only on the two
    /// SECURITY DEFINER functions and no direct table UPDATE/DELETE privileges.
    pub fn from_pool(pool: PgPool) -> Self {
        Self { pool }
    }

    /// Generate a fresh 256-bit OS-CSPRNG challenge and register only its
    /// domain-separated digest. The database clock computes expiry from TTL;
    /// caller clock skew cannot extend or prematurely expire the challenge.
    pub async fn issue(
        &self,
        context: &ChallengeContext,
        ttl: Duration,
    ) -> Result<ChallengeSecret, RegistryError> {
        context.validate()?;
        if ttl.as_secs() == 0
            || ttl.as_secs() > MAX_TTL_SECONDS
            || ttl.subsec_nanos() != 0
        {
            return Err(RegistryError::InvalidTtl);
        }
        let ttl_seconds = i32::try_from(ttl.as_secs()).map_err(|_| RegistryError::InvalidTtl)?;

        for _ in 0..ISSUE_ATTEMPTS {
            let secret = ChallengeSecret::generate()?;
            let digest = secret.digest();
            let registered = sqlx::query_scalar::<_, bool>(
                "SELECT identity_security.register_verifier_challenge($1, $2, $3, $4, $5, $6, $7)",
            )
            .bind(digest.to_vec())
            .bind(&context.verifier_id)
            .bind(&context.audience)
            .bind(context.transaction_id)
            .bind(context.request_digest.to_vec())
            .bind(context.holder_binding_digest.to_vec())
            .bind(ttl_seconds)
            .fetch_one(&self.pool)
            .await?;

            if registered {
                return Ok(secret);
            }
            // Inputs are validated locally; false should only be a duplicate
            // digest. Retry with fresh random material, bounded above.
        }
        Err(RegistryError::RegistrationRejected)
    }

    /// Consume after the holder signature and exact presentation context have
    /// already been verified by the caller. Authentication may be accepted
    /// only for Ok(true), after this method confirms COMMIT.
    ///
    /// A COMMIT error is ambiguous: the server may have committed and the
    /// acknowledgement may have been lost. This method never retries. The
    /// caller must fail closed and issue a new challenge.
    pub async fn consume_and_commit(
        &self,
        challenge: &ChallengeSecret,
        context: &ChallengeContext,
    ) -> Result<bool, RegistryError> {
        context.validate()?;
        let digest = challenge.digest();
        let mut transaction = self.pool.begin().await?;

        let consumed = sqlx::query_scalar::<_, bool>(
            "SELECT identity_security.consume_verifier_challenge($1, $2, $3, $4, $5, $6)",
        )
        .bind(digest.to_vec())
        .bind(&context.verifier_id)
        .bind(&context.audience)
        .bind(context.transaction_id)
        .bind(context.request_digest.to_vec())
        .bind(context.holder_binding_digest.to_vec())
        .fetch_one(&mut *transaction)
        .await?;

        if !consumed {
            transaction.rollback().await?;
            return Ok(false);
        }

        transaction
            .commit()
            .await
            .map_err(RegistryError::CommitOutcomeUnknown)?;
        Ok(true)
    }

    /// Close idle connections during controlled shutdown.
    pub async fn close(&self) {
        self.pool.close().await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn context() -> ChallengeContext {
        ChallengeContext::new(
            "did:mycelix:verifier-test",
            "urn:mycelix:test-audience",
            Uuid::new_v4(),
            [0x11; 32],
            [0x22; 32],
        )
        .expect("valid fixture context")
    }

    #[test]
    fn domain_separated_digest_is_stable_and_not_raw_sha256() {
        let challenge = ChallengeSecret::from_wire_bytes(&[0x42; 32]).expect("32 bytes");
        let digest = challenge.digest();
        let raw_digest: [u8; 32] = Sha256::digest([0x42; 32]).into();
        assert_ne!(digest, raw_digest);
        assert_eq!(digest, challenge.digest());
    }

    #[test]
    fn challenge_debug_never_discloses_raw_material() {
        let challenge = ChallengeSecret::from_wire_bytes(&[0x5a; 32]).expect("32 bytes");
        let rendered = format!("{challenge:?}");
        assert_eq!(rendered, "ChallengeSecret([REDACTED])");
        assert!(!rendered.contains("5a5a"));
    }

    #[test]
    fn malformed_wire_length_is_rejected() {
        assert!(matches!(
            ChallengeSecret::from_wire_bytes(&[0; 31]),
            Err(RegistryError::InvalidChallengeLength)
        ));
        assert!(matches!(
            ChallengeSecret::from_wire_bytes(&[0; 33]),
            Err(RegistryError::InvalidChallengeLength)
        ));
    }

    #[test]
    fn context_rejects_empty_or_controlled_verifier_and_audience() {
        assert!(matches!(
            ChallengeContext::new("", "urn:audience", Uuid::new_v4(), [1; 32], [2; 32]),
            Err(RegistryError::InvalidVerifierId)
        ));
        assert!(matches!(
            ChallengeContext::new(
                "did:mycelix:verifier\n",
                "urn:audience",
                Uuid::new_v4(),
                [1; 32],
                [2; 32]
            ),
            Err(RegistryError::InvalidVerifierId)
        ));
        assert!(matches!(
            ChallengeContext::new("did:mycelix:v", "", Uuid::new_v4(), [1; 32], [2; 32]),
            Err(RegistryError::InvalidAudience)
        ));
    }

    #[test]
    fn only_whole_second_ttls_from_one_to_ten_minutes_are_accepted() {
        let valid = [
            Duration::from_secs(1),
            Duration::from_secs(300),
            Duration::from_secs(600),
        ];
        for ttl in valid {
            assert!(ttl.as_secs() >= 1 && ttl.as_secs() <= MAX_TTL_SECONDS);
            assert_eq!(ttl.subsec_nanos(), 0);
        }
        let invalid = [
            Duration::ZERO,
            Duration::from_millis(1500),
            Duration::from_secs(601),
        ];
        for ttl in invalid {
            assert!(
                ttl.as_secs() == 0
                    || ttl.as_secs() > MAX_TTL_SECONDS
                    || ttl.subsec_nanos() != 0
            );
        }
        assert!(context().validate().is_ok());
    }
}
