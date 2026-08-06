// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! PostgreSQL authority backend for canonical scientific events.
//!
//! The file backend remains a deterministic reference implementation. This
//! backend provides the stronger multi-process boundary: one serializable SQL
//! transaction commits the scientific event, its independently signed
//! authority receipt, and a durable publication-outbox record.

use crate::authority_delivery::{AuthorityDeliveryEnvelope, SignedAuthorityDeliveryEnvelope};
use crate::authority_epoch::SignedAuthorityDatabaseEpochCertificate;
use crate::authority_fencing::{
    AuthorityWriteLeasePhase, AuthorityWriteLeaseProvider, AuthorityWriteScope,
    DEFAULT_AUTHORITY_WRITE_LEASE_CLOCK_SKEW_SECONDS, SignedAuthorityWriteLease,
};
use crate::authority_signing::AuthoritySigner;
use crate::checkpoint_mirror::SignedCheckpointMirrorObservation;
use crate::scientific_authority_audit::{
    AuthorityAttestationStatus, AuthorityAuditSummary, ScientificAuthorityAuditStore,
    SignedScientificAuthorityReceipt,
};
use crate::scientific_credential_governance::CredentialGovernanceProjection;
use crate::scientific_events::{
    ActorId, AppendReceipt, ClaimId, ClaimProjection, ContentHash, EventPage, ScientificEventId,
    ScientificEventLog, SignedScientificEvent, StreamHead,
};
use crate::scientific_governance::AtomicScientificCommitStore;
use crate::{Error, Result};
use async_trait::async_trait;
use chrono::{DateTime, Duration as ChronoDuration, Utc};
use ed25519_dalek::VerifyingKey;
use serde::{Deserialize, Serialize};
use sqlx::Row;
use sqlx::postgres::{PgPool, PgPoolOptions, PgRow};
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::sync::Arc;
use std::time::Duration;
use uuid::Uuid;

const DEFAULT_MAX_CONNECTIONS: u32 = 16;
const DEFAULT_ACQUIRE_TIMEOUT_SECONDS: u64 = 10;
const MAX_PAGE_SIZE: usize = 10_000;
const MAX_MIRROR_FUTURE_SKEW_SECONDS: i64 = 300;
const LEGACY_AUTHORITY_SCHEMA_VERSION: i64 = 1;
pub(crate) const AUTHORITY_SCHEMA_VERSION: i64 = 4;
const AUTHORITY_SCHEMA_MIGRATION_ID: &str = "mycelix-desci-authority-v4-2026-08-05";

#[derive(Debug, Clone)]
pub struct PostgresAuthorityConfig {
    pub database_url: String,
    pub max_connections: u32,
    pub acquire_timeout: Duration,
    pub run_migrations: bool,
}

impl PostgresAuthorityConfig {
    pub fn from_database_url(database_url: impl Into<String>) -> Self {
        Self {
            database_url: database_url.into(),
            max_connections: DEFAULT_MAX_CONNECTIONS,
            acquire_timeout: Duration::from_secs(DEFAULT_ACQUIRE_TIMEOUT_SECONDS),
            run_migrations: true,
        }
    }

    pub fn validate(&self) -> Result<()> {
        if self.database_url.trim().is_empty() {
            return Err(Error::Validation(
                "PostgreSQL database URL cannot be empty".to_string(),
            ));
        }
        if self.max_connections == 0 || self.max_connections > 256 {
            return Err(Error::Validation(
                "PostgreSQL max_connections must be between 1 and 256".to_string(),
            ));
        }
        if self.acquire_timeout.is_zero() || self.acquire_timeout > Duration::from_secs(120) {
            return Err(Error::Validation(
                "PostgreSQL acquire timeout must be between 1 and 120 seconds".to_string(),
            ));
        }
        Ok(())
    }
}

#[derive(Clone)]
pub struct PostgresAuthorityFencingConfig {
    pub deployment_id: String,
    pub trusted_lease_keys: BTreeSet<[u8; 32]>,
    pub lease_provider: Arc<dyn AuthorityWriteLeaseProvider>,
    pub clock_skew: ChronoDuration,
}

impl PostgresAuthorityFencingConfig {
    pub fn new(
        deployment_id: impl Into<String>,
        trusted_lease_keys: BTreeSet<[u8; 32]>,
        lease_provider: Arc<dyn AuthorityWriteLeaseProvider>,
    ) -> Self {
        Self {
            deployment_id: deployment_id.into(),
            trusted_lease_keys,
            lease_provider,
            clock_skew: ChronoDuration::seconds(DEFAULT_AUTHORITY_WRITE_LEASE_CLOCK_SKEW_SECONDS),
        }
    }

    pub fn validate(&self) -> Result<()> {
        let deployment_id = self.deployment_id.trim();
        if deployment_id.is_empty()
            || deployment_id.len() > 256
            || deployment_id.chars().any(char::is_control)
        {
            return Err(Error::Validation(
                "authority fencing deployment id must contain 1-256 printable bytes".to_string(),
            ));
        }
        if self.trusted_lease_keys.is_empty() {
            return Err(Error::Validation(
                "authority fencing requires at least one trusted lease key".to_string(),
            ));
        }
        for key in &self.trusted_lease_keys {
            VerifyingKey::from_bytes(key).map_err(|error| Error::Crypto(error.to_string()))?;
        }
        if self.clock_skew < ChronoDuration::zero() || self.clock_skew > ChronoDuration::minutes(5)
        {
            return Err(Error::Validation(
                "authority fencing clock skew must be between zero and five minutes".to_string(),
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityWriteFencingStatus {
    pub configured: bool,
    pub active: bool,
    pub deployment_id: Option<String>,
    pub primary_id: Option<String>,
    pub generation: Option<u64>,
    pub lease_id: Option<Uuid>,
    pub phase: Option<AuthorityWriteLeasePhase>,
    pub epoch_number: Option<u64>,
    pub epoch_hash: Option<ContentHash>,
    pub database_system_identifier: Option<String>,
    pub postgres_timeline: Option<u64>,
    pub expires_at: Option<DateTime<Utc>>,
    pub signer_public_key: Option<String>,
    pub error: Option<String>,
}

#[derive(Clone)]
struct PostgresAuthorityFencingRuntime {
    deployment_id: String,
    trusted_lease_keys: Arc<BTreeSet<[u8; 32]>>,
    lease_provider: Arc<dyn AuthorityWriteLeaseProvider>,
    clock_skew: ChronoDuration,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityOutboxSummary {
    pub pending: i64,
    pub leased: i64,
    pub delivery_attempts: i64,
    pub maximum_attempts: i32,
    pub oldest_pending_at: Option<DateTime<Utc>>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityOutboxMessage {
    pub id: Uuid,
    pub topic: String,
    pub aggregate_id: String,
    pub aggregate_sequence: i64,
    pub delivery: SignedAuthorityDeliveryEnvelope,
    pub delivery_hash: ContentHash,
    pub created_at: DateTime<Utc>,
    pub attempts: i32,
    pub lease_owner: Option<String>,
    pub lease_until: Option<DateTime<Utc>>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CheckpointMirrorRecord {
    pub observation: SignedCheckpointMirrorObservation,
    pub observation_hash: ContentHash,
    pub accepted_at: DateTime<Utc>,
}

#[derive(Clone)]
pub struct PostgresAuthorityStore {
    pool: PgPool,
    trusted_receipt_keys: Arc<BTreeSet<[u8; 32]>>,
    delivery_signer: Arc<dyn AuthoritySigner>,
    write_fencing: Option<Arc<PostgresAuthorityFencingRuntime>>,
}

impl PostgresAuthorityStore {
    pub async fn connect(
        config: PostgresAuthorityConfig,
        trusted_receipt_keys: BTreeSet<[u8; 32]>,
        delivery_signer: Arc<dyn AuthoritySigner>,
    ) -> Result<Self> {
        Self::connect_internal(config, trusted_receipt_keys, delivery_signer, None).await
    }

    pub async fn connect_with_fencing(
        config: PostgresAuthorityConfig,
        trusted_receipt_keys: BTreeSet<[u8; 32]>,
        delivery_signer: Arc<dyn AuthoritySigner>,
        fencing: PostgresAuthorityFencingConfig,
    ) -> Result<Self> {
        fencing.validate()?;
        Self::connect_internal(config, trusted_receipt_keys, delivery_signer, Some(fencing)).await
    }

    async fn connect_internal(
        config: PostgresAuthorityConfig,
        trusted_receipt_keys: BTreeSet<[u8; 32]>,
        delivery_signer: Arc<dyn AuthoritySigner>,
        fencing: Option<PostgresAuthorityFencingConfig>,
    ) -> Result<Self> {
        config.validate()?;
        if trusted_receipt_keys.is_empty() {
            return Err(Error::Validation(
                "PostgreSQL authority backend requires at least one trusted receipt key"
                    .to_string(),
            ));
        }
        for key in &trusted_receipt_keys {
            VerifyingKey::from_bytes(key).map_err(|error| Error::Crypto(error.to_string()))?;
        }
        if trusted_receipt_keys.contains(&delivery_signer.verifying_key().to_bytes()) {
            return Err(Error::VerificationFailed(
                "authority delivery signing key must differ from every authority receipt key"
                    .to_string(),
            ));
        }
        if let Some(fencing) = fencing.as_ref() {
            if fencing
                .trusted_lease_keys
                .contains(&delivery_signer.verifying_key().to_bytes())
                || fencing
                    .trusted_lease_keys
                    .iter()
                    .any(|key| trusted_receipt_keys.contains(key))
            {
                return Err(Error::VerificationFailed(
                    "authority write-lease keys must differ from receipt and delivery service keys"
                        .to_string(),
                ));
            }
        }
        let pool = PgPoolOptions::new()
            .max_connections(config.max_connections)
            .acquire_timeout(config.acquire_timeout)
            .connect(&config.database_url)
            .await
            .map_err(sql_error)?;
        let write_fencing = fencing.map(|fencing| {
            Arc::new(PostgresAuthorityFencingRuntime {
                deployment_id: fencing.deployment_id.trim().to_string(),
                trusted_lease_keys: Arc::new(fencing.trusted_lease_keys),
                lease_provider: fencing.lease_provider,
                clock_skew: fencing.clock_skew,
            })
        });
        let store = Self {
            pool,
            trusted_receipt_keys: Arc::new(trusted_receipt_keys),
            delivery_signer,
            write_fencing,
        };
        if config.run_migrations {
            store.verify_schema_migration_fence().await?;
            store.migrate().await?;
        } else {
            store.verify_schema().await?;
        }
        store.health_check().await?;
        Ok(store)
    }

    pub fn pool(&self) -> &PgPool {
        &self.pool
    }

    pub(crate) fn trusted_receipt_keys(&self) -> &BTreeSet<[u8; 32]> {
        self.trusted_receipt_keys.as_ref()
    }

    pub(crate) fn delivery_public_key(&self) -> [u8; 32] {
        self.delivery_signer.verifying_key().to_bytes()
    }

    pub fn write_fencing_configured(&self) -> bool {
        self.write_fencing.is_some()
    }

    pub fn trusted_write_lease_keys(&self) -> BTreeSet<[u8; 32]> {
        self.write_fencing
            .as_ref()
            .map(|fencing| fencing.trusted_lease_keys.as_ref().clone())
            .unwrap_or_default()
    }

    pub async fn write_fencing_status(&self) -> AuthorityWriteFencingStatus {
        let Some(fencing) = self.write_fencing.as_ref() else {
            return AuthorityWriteFencingStatus {
                configured: false,
                active: false,
                deployment_id: None,
                primary_id: None,
                generation: None,
                lease_id: None,
                phase: None,
                epoch_number: None,
                epoch_hash: None,
                database_system_identifier: None,
                postgres_timeline: None,
                expires_at: None,
                signer_public_key: None,
                error: Some("authority write fencing is not configured".to_string()),
            };
        };
        let result = async {
            let signed = fencing.lease_provider.current_lease().await?;
            signed.verify_at(&fencing.trusted_lease_keys, Utc::now(), fencing.clock_skew)?;
            if signed.lease.deployment_id != fencing.deployment_id {
                return Err(Error::VerificationFailed(
                    "authority write lease deployment does not match server configuration"
                        .to_string(),
                ));
            }
            let mut transaction = self.pool.begin().await.map_err(sql_error)?;
            begin_read_only_repeatable_read(&mut transaction).await?;
            verify_write_lease_target(
                &mut transaction,
                &signed,
                AuthorityWriteFenceTarget::CurrentEpoch,
            )
            .await?;
            transaction.rollback().await.map_err(sql_error)?;
            Ok::<_, Error>(signed)
        }
        .await;
        match result {
            Ok(signed) => AuthorityWriteFencingStatus {
                configured: true,
                active: true,
                deployment_id: Some(signed.lease.deployment_id),
                primary_id: Some(signed.lease.primary_id),
                generation: Some(signed.lease.generation),
                lease_id: Some(signed.lease.lease_id),
                phase: Some(signed.lease.phase),
                epoch_number: signed.lease.epoch_number,
                epoch_hash: signed.lease.epoch_hash,
                database_system_identifier: Some(signed.lease.database_system_identifier),
                postgres_timeline: Some(signed.lease.postgres_timeline),
                expires_at: Some(signed.lease.expires_at),
                signer_public_key: Some(hex::encode(signed.signer_public_key)),
                error: None,
            },
            Err(error) => AuthorityWriteFencingStatus {
                configured: true,
                active: false,
                deployment_id: Some(fencing.deployment_id.clone()),
                primary_id: None,
                generation: None,
                lease_id: None,
                phase: None,
                epoch_number: None,
                epoch_hash: None,
                database_system_identifier: None,
                postgres_timeline: None,
                expires_at: None,
                signer_public_key: None,
                error: Some(error.to_string()),
            },
        }
    }

    pub(crate) async fn assert_authority_write_fence(
        &self,
        transaction: &mut sqlx::Transaction<'_, sqlx::Postgres>,
        scope: AuthorityWriteScope,
    ) -> Result<Option<SignedAuthorityWriteLease>> {
        self.assert_authority_write_fence_for_target(
            transaction,
            scope,
            AuthorityWriteFenceTarget::CurrentEpoch,
        )
        .await
    }

    pub(crate) async fn assert_authority_write_fence_for_epoch(
        &self,
        transaction: &mut sqlx::Transaction<'_, sqlx::Postgres>,
        scope: AuthorityWriteScope,
        certificate: &SignedAuthorityDatabaseEpochCertificate,
        epoch_hash: ContentHash,
    ) -> Result<Option<SignedAuthorityWriteLease>> {
        self.assert_authority_write_fence_for_target(
            transaction,
            scope,
            AuthorityWriteFenceTarget::CandidateEpoch {
                deployment_id: &certificate.certificate.intent.deployment_id,
                primary_id: &certificate.certificate.intent.candidate_primary_id,
                database_system_identifier: &certificate
                    .certificate
                    .intent
                    .database_system_identifier,
                postgres_timeline: certificate.certificate.intent.postgres_timeline,
                epoch_number: certificate.certificate.intent.epoch_number,
                epoch_hash,
            },
        )
        .await
    }

    async fn assert_authority_write_fence_for_target(
        &self,
        transaction: &mut sqlx::Transaction<'_, sqlx::Postgres>,
        scope: AuthorityWriteScope,
        target: AuthorityWriteFenceTarget<'_>,
    ) -> Result<Option<SignedAuthorityWriteLease>> {
        let Some(fencing) = self.write_fencing.as_ref() else {
            return Ok(None);
        };
        let signed = fencing.lease_provider.current_lease().await?;
        signed.verify_at(&fencing.trusted_lease_keys, Utc::now(), fencing.clock_skew)?;
        signed.lease.permits(scope)?;
        if signed.lease.deployment_id != fencing.deployment_id {
            return Err(Error::VerificationFailed(
                "authority write lease deployment does not match server configuration".to_string(),
            ));
        }
        verify_write_lease_target(transaction, &signed, target).await?;
        persist_authority_write_fence(transaction, &signed, scope).await?;
        Ok(Some(signed))
    }

    async fn verify_schema_migration_fence(&self) -> Result<()> {
        let Some(fencing) = self.write_fencing.as_ref() else {
            return Ok(());
        };
        let signed = fencing.lease_provider.current_lease().await?;
        signed.verify_at(&fencing.trusted_lease_keys, Utc::now(), fencing.clock_skew)?;
        signed.lease.permits(AuthorityWriteScope::SchemaMigration)?;
        if signed.lease.deployment_id != fencing.deployment_id {
            return Err(Error::VerificationFailed(
                "authority write lease deployment does not match server configuration".to_string(),
            ));
        }
        let mut transaction = self.pool.begin().await.map_err(sql_error)?;
        begin_read_only_repeatable_read(&mut transaction).await?;
        let epoch_table_exists: bool =
            sqlx::query_scalar("SELECT to_regclass('public.desci_database_epochs') IS NOT NULL")
                .fetch_one(&mut *transaction)
                .await
                .map_err(sql_error)?;
        if epoch_table_exists {
            verify_write_lease_target(
                &mut transaction,
                &signed,
                AuthorityWriteFenceTarget::CurrentEpoch,
            )
            .await?;
        } else {
            let (system_identifier, timeline_id) = postgres_identity(&mut transaction).await?;
            if signed.lease.phase != AuthorityWriteLeasePhase::Bootstrap
                || signed.lease.epoch_number.is_some()
                || signed.lease.epoch_hash.is_some()
                || signed.lease.database_system_identifier != system_identifier
                || signed.lease.postgres_timeline != timeline_id
            {
                return Err(Error::VerificationFailed(
                    "initial authority schema migration requires a bootstrap lease bound to the connected PostgreSQL system and timeline"
                        .to_string(),
                ));
            }
        }
        transaction.rollback().await.map_err(sql_error)?;
        Ok(())
    }

    pub async fn health_check(&self) -> Result<()> {
        sqlx::query("SELECT 1")
            .execute(&self.pool)
            .await
            .map_err(sql_error)?;
        if self.write_fencing.is_some() {
            let status = self.write_fencing_status().await;
            if !status.active {
                return Err(Error::VerificationFailed(format!(
                    "PostgreSQL authority write fencing is inactive: {}",
                    status.error.as_deref().unwrap_or("unknown fencing error")
                )));
            }
        }
        let unsigned_pending: i64 = sqlx::query_scalar(
            r#"
            SELECT COUNT(*)
            FROM desci_authority_outbox
            WHERE published_at IS NULL
              AND NOT (
                  payload ? 'envelope'
                  AND payload ? 'signer_public_key'
                  AND payload ? 'signature'
              )
            "#,
        )
        .fetch_one(&self.pool)
        .await
        .map_err(sql_error)?;
        if unsigned_pending > 0 {
            return Err(Error::VerificationFailed(format!(
                "PostgreSQL authority outbox contains {unsigned_pending} pending unsigned legacy rows"
            )));
        }
        Ok(())
    }

    pub async fn migrate(&self) -> Result<()> {
        let mut transaction = self.pool.begin().await.map_err(sql_error)?;
        sqlx::query("SELECT pg_advisory_xact_lock(730391904221)")
            .execute(&mut *transaction)
            .await
            .map_err(sql_error)?;

        // Establish only the migration metadata table before inspecting the
        // marker. If a different migration already owns this schema version,
        // fail before applying any of this tranche's domain DDL.
        sqlx::query(POSTGRES_AUTHORITY_MIGRATION[0])
            .execute(&mut *transaction)
            .await
            .map_err(sql_error)?;
        let existing_migration_id: Option<String> = sqlx::query_scalar(
            "SELECT migration_id FROM desci_authority_schema_migrations WHERE version = $1 FOR UPDATE",
        )
        .bind(AUTHORITY_SCHEMA_VERSION)
        .fetch_optional(&mut *transaction)
        .await
        .map_err(sql_error)?;
        let upgrading_from_legacy_schema: bool = sqlx::query_scalar(
            "SELECT EXISTS (SELECT 1 FROM desci_authority_schema_migrations WHERE version = $1)",
        )
        .bind(LEGACY_AUTHORITY_SCHEMA_VERSION)
        .fetch_one(&mut *transaction)
        .await
        .map_err(sql_error)?;
        if let Some(existing_migration_id) = existing_migration_id.as_deref() {
            if existing_migration_id != AUTHORITY_SCHEMA_MIGRATION_ID {
                return Err(Error::Storage(format!(
                    "PostgreSQL authority schema version {AUTHORITY_SCHEMA_VERSION} is already owned by unexpected migration id {existing_migration_id}"
                )));
            }
        }

        if upgrading_from_legacy_schema && existing_migration_id.is_none() {
            let legacy_pending: i64 = sqlx::query_scalar(
                r#"
                SELECT COUNT(*)
                FROM desci_authority_outbox
                WHERE published_at IS NULL
                  AND NOT (
                      payload ? 'envelope'
                      AND payload ? 'signer_public_key'
                      AND payload ? 'signature'
                  )
                "#,
            )
            .fetch_one(&mut *transaction)
            .await
            .map_err(sql_error)?;
            if legacy_pending > 0 {
                return Err(Error::Storage(format!(
                    "PostgreSQL authority schema v4 requires draining {legacy_pending} pending unsigned v1 outbox rows before migration"
                )));
            }
        }
        for statement in &POSTGRES_AUTHORITY_MIGRATION[1..] {
            sqlx::query(statement)
                .execute(&mut *transaction)
                .await
                .map_err(sql_error)?;
        }
        sqlx::query(POSTGRES_AUTHORITY_MIGRATION_MARKER_SQL)
            .bind(AUTHORITY_SCHEMA_VERSION)
            .bind(AUTHORITY_SCHEMA_MIGRATION_ID)
            .execute(&mut *transaction)
            .await
            .map_err(sql_error)?;
        transaction.commit().await.map_err(sql_error)?;
        self.verify_schema().await
    }

    pub async fn verify_schema(&self) -> Result<()> {
        let migration_id: Option<String> = sqlx::query_scalar(
            "SELECT migration_id FROM desci_authority_schema_migrations WHERE version = $1",
        )
        .bind(AUTHORITY_SCHEMA_VERSION)
        .fetch_optional(&self.pool)
        .await
        .map_err(sql_error)?;
        if migration_id.as_deref() != Some(AUTHORITY_SCHEMA_MIGRATION_ID) {
            return Err(Error::Storage(format!(
                "PostgreSQL authority schema version {AUTHORITY_SCHEMA_VERSION} is absent or has an unexpected migration id"
            )));
        }
        Ok(())
    }

    pub(crate) fn sign_delivery(
        &self,
        delivery_id: Uuid,
        topic: &str,
        aggregate_id: &str,
        aggregate_sequence: i64,
        created_at: DateTime<Utc>,
        payload: serde_json::Value,
    ) -> Result<(serde_json::Value, ContentHash)> {
        let envelope = AuthorityDeliveryEnvelope::new(
            delivery_id,
            topic,
            aggregate_id,
            aggregate_sequence,
            created_at,
            payload,
        )?;
        let signed =
            SignedAuthorityDeliveryEnvelope::sign_with(envelope, self.delivery_signer.as_ref())?;
        let delivery_hash = signed.delivery_hash()?;
        Ok((serde_json::to_value(signed)?, delivery_hash))
    }

    /// Claim unpublished outbox rows using `FOR UPDATE SKIP LOCKED`. A worker
    /// must later call `mark_outbox_published` or `mark_outbox_failed` with the
    /// same owner token.
    pub async fn outbox_summary(&self) -> Result<AuthorityOutboxSummary> {
        let row = sqlx::query(
            r#"
            SELECT
                COUNT(*) FILTER (WHERE published_at IS NULL) AS pending,
                COUNT(*) FILTER (
                    WHERE published_at IS NULL
                      AND lease_owner IS NOT NULL
                      AND lease_until >= clock_timestamp()
                ) AS leased,
                COALESCE(SUM(attempts) FILTER (WHERE published_at IS NULL), 0) AS delivery_attempts,
                COALESCE(MAX(attempts) FILTER (WHERE published_at IS NULL), 0) AS maximum_attempts,
                MIN(created_at) FILTER (WHERE published_at IS NULL) AS oldest_pending_at
            FROM desci_authority_outbox
            "#,
        )
        .fetch_one(&self.pool)
        .await
        .map_err(sql_error)?;
        Ok(AuthorityOutboxSummary {
            pending: row.try_get("pending").map_err(sql_error)?,
            leased: row.try_get("leased").map_err(sql_error)?,
            delivery_attempts: row.try_get("delivery_attempts").map_err(sql_error)?,
            maximum_attempts: row.try_get("maximum_attempts").map_err(sql_error)?,
            oldest_pending_at: row.try_get("oldest_pending_at").map_err(sql_error)?,
        })
    }

    pub async fn claim_outbox_batch(
        &self,
        worker_id: &str,
        limit: usize,
        lease_seconds: u64,
    ) -> Result<Vec<AuthorityOutboxMessage>> {
        validate_worker_id(worker_id)?;
        if limit == 0 || limit > 1_000 {
            return Err(Error::Validation(
                "outbox batch limit must be between 1 and 1000".to_string(),
            ));
        }
        if lease_seconds == 0 || lease_seconds > 3_600 {
            return Err(Error::Validation(
                "outbox lease must be between 1 and 3600 seconds".to_string(),
            ));
        }
        let mut transaction = self.pool.begin().await.map_err(sql_error)?;
        sqlx::query("SET TRANSACTION ISOLATION LEVEL SERIALIZABLE")
            .execute(&mut *transaction)
            .await
            .map_err(sql_error)?;
        lock_authority_epoch_shared(&mut transaction).await?;
        self.assert_authority_write_fence(&mut transaction, AuthorityWriteScope::OutboxDelivery)
            .await?;
        let rows = sqlx::query(
            r#"
            WITH candidates AS (
                SELECT id
                FROM desci_authority_outbox
                WHERE published_at IS NULL
                  AND (lease_until IS NULL OR lease_until < clock_timestamp())
                ORDER BY created_at, id
                FOR UPDATE SKIP LOCKED
                LIMIT $1
            )
            UPDATE desci_authority_outbox AS outbox
            SET lease_owner = $2,
                lease_until = clock_timestamp() + ($3::bigint * interval '1 second'),
                attempts = attempts + 1
            FROM candidates
            WHERE outbox.id = candidates.id
            RETURNING outbox.id, outbox.topic, outbox.aggregate_id,
                      outbox.aggregate_sequence, outbox.payload, outbox.payload_hash,
                      outbox.created_at, outbox.attempts, outbox.lease_owner,
                      outbox.lease_until
            "#,
        )
        .bind(limit as i64)
        .bind(worker_id)
        .bind(lease_seconds as i64)
        .fetch_all(&mut *transaction)
        .await
        .map_err(sql_error)?;
        transaction.commit().await.map_err(sql_error)?;
        rows.into_iter().map(outbox_from_row).collect()
    }

    pub async fn mark_outbox_published(
        &self,
        id: Uuid,
        worker_id: &str,
        published_at: DateTime<Utc>,
    ) -> Result<()> {
        validate_worker_id(worker_id)?;
        let mut transaction = self.pool.begin().await.map_err(sql_error)?;
        sqlx::query("SET TRANSACTION ISOLATION LEVEL SERIALIZABLE")
            .execute(&mut *transaction)
            .await
            .map_err(sql_error)?;
        lock_authority_epoch_shared(&mut transaction).await?;
        self.assert_authority_write_fence(&mut transaction, AuthorityWriteScope::OutboxDelivery)
            .await?;
        let result = sqlx::query(
            r#"
            UPDATE desci_authority_outbox
            SET published_at = $3,
                lease_owner = NULL,
                lease_until = NULL,
                last_error = NULL
            WHERE id = $1
              AND lease_owner = $2
              AND published_at IS NULL
            "#,
        )
        .bind(id)
        .bind(worker_id)
        .bind(published_at)
        .execute(&mut *transaction)
        .await
        .map_err(sql_error)?;
        if result.rows_affected() != 1 {
            return Err(Error::Storage(
                "outbox publication acknowledgement lost its lease or row".to_string(),
            ));
        }
        transaction.commit().await.map_err(sql_error)?;
        Ok(())
    }

    pub async fn mark_outbox_failed(&self, id: Uuid, worker_id: &str, error: &str) -> Result<()> {
        validate_worker_id(worker_id)?;
        let error = error.trim();
        if error.is_empty() || error.len() > 4_096 {
            return Err(Error::Validation(
                "outbox failure text must be between 1 and 4096 bytes".to_string(),
            ));
        }
        let mut transaction = self.pool.begin().await.map_err(sql_error)?;
        sqlx::query("SET TRANSACTION ISOLATION LEVEL SERIALIZABLE")
            .execute(&mut *transaction)
            .await
            .map_err(sql_error)?;
        lock_authority_epoch_shared(&mut transaction).await?;
        self.assert_authority_write_fence(&mut transaction, AuthorityWriteScope::OutboxDelivery)
            .await?;
        let result = sqlx::query(
            r#"
            UPDATE desci_authority_outbox
            SET lease_owner = NULL,
                lease_until = clock_timestamp()
                    + (LEAST(3600, GREATEST(1, LEAST(attempts, 60) * LEAST(attempts, 60))) * interval '1 second'),
                last_error = $3
            WHERE id = $1
              AND lease_owner = $2
              AND published_at IS NULL
            "#,
        )
        .bind(id)
        .bind(worker_id)
        .bind(error)
        .execute(&mut *transaction)
        .await
        .map_err(sql_error)?;
        if result.rows_affected() != 1 {
            return Err(Error::Storage(
                "outbox failure acknowledgement lost its lease or row".to_string(),
            ));
        }
        transaction.commit().await.map_err(sql_error)?;
        Ok(())
    }

    pub async fn record_checkpoint_mirror_observation(
        &self,
        observation: SignedCheckpointMirrorObservation,
        governance: &CredentialGovernanceProjection,
    ) -> Result<CheckpointMirrorRecord> {
        observation.verify_against_governance(governance)?;
        if observation.observed_at
            > Utc::now() + chrono::Duration::seconds(MAX_MIRROR_FUTURE_SKEW_SECONDS)
        {
            return Err(Error::VerificationFailed(
                "checkpoint mirror observation exceeds the allowed future clock skew".to_string(),
            ));
        }
        let recorded_at = Utc::now();
        let observation_hash = observation.observation_hash()?;
        let observation_json = serde_json::to_value(&observation)?;
        let payload = serde_json::json!({
            "schema": "mycelix-desci.checkpoint-mirror-observed.v1",
            "observation": &observation,
            "observation_hash": observation_hash,
            "accepted_at": recorded_at,
        });
        let delivery_id = Uuid::new_v4();
        let (delivery_json, delivery_hash) = self.sign_delivery(
            delivery_id,
            "transparency.checkpoint.mirrored.v1",
            &observation.observation_id.0.to_string(),
            0,
            recorded_at,
            payload,
        )?;
        let mut transaction = self.pool.begin().await.map_err(sql_error)?;
        sqlx::query("SET TRANSACTION ISOLATION LEVEL SERIALIZABLE")
            .execute(&mut *transaction)
            .await
            .map_err(sql_error)?;
        lock_authority_epoch_shared(&mut transaction).await?;
        self.assert_authority_write_fence(&mut transaction, AuthorityWriteScope::CheckpointMirror)
            .await?;
        let existing = sqlx::query(
            r#"
            SELECT observation_id, checkpoint_hash, mirror_actor,
                   mirror_organization, mirror_uri, observed_at, accepted_at,
                   observation_json
            FROM desci_checkpoint_mirror_observations
            WHERE checkpoint_hash = $1 AND mirror_actor = $2
            FOR UPDATE
            "#,
        )
        .bind(observation.checkpoint_hash.0.to_vec())
        .bind(observation.actor.as_str())
        .fetch_optional(&mut *transaction)
        .await
        .map_err(sql_error)?;
        if let Some(row) = existing {
            let persisted = checkpoint_mirror_record_from_row(row)?;
            if persisted.observation == observation {
                transaction.commit().await.map_err(sql_error)?;
                return Ok(persisted);
            }
            return Err(Error::Storage(
                "checkpoint mirror actor already recorded a different observation for this checkpoint"
                    .to_string(),
            ));
        }
        sqlx::query(
            r#"
            INSERT INTO desci_checkpoint_mirror_observations (
                observation_id, checkpoint_hash, mirror_actor, mirror_organization,
                mirror_uri, observed_at, accepted_at, observation_json
            ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
            "#,
        )
        .bind(observation.observation_id.0)
        .bind(observation.checkpoint_hash.0.to_vec())
        .bind(observation.actor.as_str())
        .bind(observation.organization.as_str())
        .bind(&observation.mirror_uri)
        .bind(observation.observed_at)
        .bind(recorded_at)
        .bind(observation_json)
        .execute(&mut *transaction)
        .await
        .map_err(sql_error)?;
        sqlx::query(
            r#"
            INSERT INTO desci_authority_outbox (
                id, topic, aggregate_id, aggregate_sequence,
                payload, payload_hash, created_at
            ) VALUES ($1, $2, $3, 0, $4, $5, $6)
            "#,
        )
        .bind(delivery_id)
        .bind("transparency.checkpoint.mirrored.v1")
        .bind(observation.observation_id.0.to_string())
        .bind(delivery_json)
        .bind(delivery_hash.0.to_vec())
        .bind(recorded_at)
        .execute(&mut *transaction)
        .await
        .map_err(sql_error)?;
        transaction.commit().await.map_err(sql_error)?;
        Ok(CheckpointMirrorRecord {
            observation,
            observation_hash,
            accepted_at: recorded_at,
        })
    }

    pub async fn checkpoint_mirror_observations(
        &self,
        checkpoint_hash: ContentHash,
    ) -> Result<Vec<CheckpointMirrorRecord>> {
        let rows = sqlx::query(
            r#"
            SELECT observation_id, checkpoint_hash, mirror_actor,
                   mirror_organization, mirror_uri, observed_at, accepted_at,
                   observation_json
            FROM desci_checkpoint_mirror_observations
            WHERE checkpoint_hash = $1
            ORDER BY observed_at, observation_id
            "#,
        )
        .bind(checkpoint_hash.0.to_vec())
        .fetch_all(&self.pool)
        .await
        .map_err(sql_error)?;
        rows.into_iter()
            .map(checkpoint_mirror_record_from_row)
            .collect()
    }

    async fn verify_persisted_receipt(
        &self,
        receipt: SignedScientificAuthorityReceipt,
    ) -> Result<SignedScientificAuthorityReceipt> {
        let event = <Self as ScientificEventLog>::event_by_id(self, receipt.receipt.event_id)
            .await?
            .ok_or_else(|| {
                Error::VerificationFailed(
                    "PostgreSQL authority receipt has no corresponding scientific event"
                        .to_string(),
                )
            })?;
        receipt.verify_for_event(&event, self.trusted_receipt_keys.as_ref())?;
        Ok(receipt)
    }

    async fn all_events(&self) -> Result<Vec<(SignedScientificEvent, DateTime<Utc>)>> {
        let rows = sqlx::query(
            r#"
            SELECT stream_id, sequence, event_id, event_hash, actor_id,
                   idempotency_key, received_at, event_json
            FROM desci_scientific_events
            ORDER BY stream_id, sequence
            "#,
        )
        .fetch_all(&self.pool)
        .await
        .map_err(sql_error)?;
        rows.into_iter()
            .map(event_with_received_at_from_row)
            .collect()
    }
}

#[async_trait]
impl AtomicScientificCommitStore for PostgresAuthorityStore {
    async fn commit_scientific_event(
        &self,
        expected_sequence: u64,
        event: SignedScientificEvent,
        receipt: SignedScientificAuthorityReceipt,
        received_at: DateTime<Utc>,
    ) -> Result<AppendReceipt> {
        event.verify()?;
        receipt.verify_for_event(&event, self.trusted_receipt_keys.as_ref())?;
        if receipt.receipt.received_at != received_at {
            return Err(Error::VerificationFailed(
                "atomic authority receipt time does not match append time".to_string(),
            ));
        }
        if event.envelope.sequence != expected_sequence {
            return Err(Error::Storage(
                "atomic append expected sequence does not match event sequence".to_string(),
            ));
        }
        let sql_sequence = sequence_to_i64(expected_sequence)?;
        let mut candidate_stream = self.stream(event.envelope.stream_id).await?;
        if candidate_stream.len() as u64 != expected_sequence {
            return Err(Error::Storage(
                "atomic append caller is stale relative to the durable stream".to_string(),
            ));
        }
        candidate_stream.push(event.clone());
        ClaimProjection::rebuild(&candidate_stream)?;

        let event_hash = event.event_hash()?;
        let receipt_hash = receipt.receipt_hash()?;
        let event_json = serde_json::to_value(&event)?;
        let receipt_json = serde_json::to_value(&receipt)?;
        let append_receipt = AppendReceipt {
            stream_id: event.envelope.stream_id,
            sequence: event.envelope.sequence,
            event_id: event.envelope.event_id,
            event_hash,
            received_at,
        };
        let outbox_payload = serde_json::json!({
            "schema": "mycelix-desci.scientific-event-committed.v1",
            "append_receipt": &append_receipt,
            "event": &event,
            "authority_receipt": &receipt,
        });
        let outbox_id = Uuid::new_v4();
        let (outbox_delivery, outbox_delivery_hash) = self.sign_delivery(
            outbox_id,
            "scientific.event.committed.v1",
            &event.envelope.stream_id.to_string(),
            sql_sequence,
            received_at,
            outbox_payload,
        )?;

        let mut transaction = self.pool.begin().await.map_err(sql_error)?;
        sqlx::query("SET TRANSACTION ISOLATION LEVEL SERIALIZABLE")
            .execute(&mut *transaction)
            .await
            .map_err(sql_error)?;
        lock_authority_epoch_shared(&mut transaction).await?;
        self.assert_authority_write_fence(&mut transaction, AuthorityWriteScope::ScientificEvent)
            .await?;
        sqlx::query("SELECT pg_advisory_xact_lock(hashtextextended($1, 0))")
            .bind(event.envelope.stream_id.to_string())
            .execute(&mut *transaction)
            .await
            .map_err(sql_error)?;

        let current = sqlx::query(
            r#"
            SELECT sequence, event_hash
            FROM desci_scientific_events
            WHERE stream_id = $1
            ORDER BY sequence DESC
            LIMIT 1
            FOR UPDATE
            "#,
        )
        .bind(event.envelope.stream_id.0)
        .fetch_optional(&mut *transaction)
        .await
        .map_err(sql_error)?;
        match current {
            None if expected_sequence == 0 => {
                if event.envelope.previous_hash.is_some() {
                    return Err(Error::Storage(
                        "genesis scientific event unexpectedly has a previous hash".to_string(),
                    ));
                }
            }
            Some(row) => {
                let sequence: i64 = row.try_get("sequence").map_err(sql_error)?;
                let hash = content_hash_from_bytes(
                    row.try_get::<Vec<u8>, _>("event_hash").map_err(sql_error)?,
                    "event hash",
                )?;
                if sequence.checked_add(1) != Some(sql_sequence)
                    || event.envelope.previous_hash != Some(hash)
                {
                    return Err(Error::Storage(format!(
                        "serializable append conflict: stream head is {}, caller expected {}",
                        sequence.saturating_add(1),
                        expected_sequence
                    )));
                }
            }
            None => {
                return Err(Error::Storage(format!(
                    "serializable append conflict: empty stream cannot accept sequence {expected_sequence}"
                )));
            }
        }

        let previous_receipt = sqlx::query(
            r#"
            SELECT sequence, receipt_hash
            FROM desci_authority_receipts
            WHERE stream_id = $1 AND status = 'committed'
            ORDER BY sequence DESC
            LIMIT 1
            FOR UPDATE
            "#,
        )
        .bind(event.envelope.stream_id.0)
        .fetch_optional(&mut *transaction)
        .await
        .map_err(sql_error)?
        .map(|row| {
            let sequence: i64 = row.try_get("sequence").map_err(sql_error)?;
            let hash = content_hash_from_bytes(
                row.try_get::<Vec<u8>, _>("receipt_hash")
                    .map_err(sql_error)?,
                "receipt hash",
            )?;
            Ok::<_, Error>((sequence, hash))
        })
        .transpose()?;
        match previous_receipt {
            Some((prior_sequence, prior_hash)) => {
                if prior_sequence.checked_add(1) != Some(sql_sequence)
                    || receipt.receipt.previous_receipt_hash != Some(prior_hash)
                    || receipt.receipt.legacy_cutover_anchor.is_some()
                {
                    return Err(Error::Storage(
                        "authority receipt does not immediately extend the committed SQL receipt chain"
                            .to_string(),
                    ));
                }
            }
            None if sql_sequence == 0 => {
                if receipt.receipt.previous_receipt_hash.is_some()
                    || receipt.receipt.legacy_cutover_anchor.is_some()
                {
                    return Err(Error::Storage(
                        "genesis authority receipt unexpectedly has a predecessor".to_string(),
                    ));
                }
            }
            None => {
                if receipt.receipt.previous_receipt_hash.is_some()
                    || receipt.receipt.legacy_cutover_anchor != event.envelope.previous_hash
                {
                    return Err(Error::Storage(
                        "first authority receipt does not bind the legacy SQL cutover".to_string(),
                    ));
                }
            }
        }

        sqlx::query(
            r#"
            INSERT INTO desci_scientific_events (
                stream_id, sequence, event_id, event_hash, actor_id,
                idempotency_key, received_at, event_json
            ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
            "#,
        )
        .bind(event.envelope.stream_id.0)
        .bind(sql_sequence)
        .bind(event.envelope.event_id.0)
        .bind(event_hash.0.to_vec())
        .bind(event.envelope.actor.as_str())
        .bind(event.envelope.idempotency_key.as_deref())
        .bind(received_at)
        .bind(event_json)
        .execute(&mut *transaction)
        .await
        .map_err(sql_error)?;

        sqlx::query(
            r#"
            INSERT INTO desci_authority_receipts (
                event_id, stream_id, sequence, receipt_hash,
                previous_receipt_hash, status, receipt_json, prepared_at, committed_at
            ) VALUES ($1, $2, $3, $4, $5, 'committed', $6, $7, $7)
            "#,
        )
        .bind(event.envelope.event_id.0)
        .bind(event.envelope.stream_id.0)
        .bind(sql_sequence)
        .bind(receipt_hash.0.to_vec())
        .bind(
            receipt
                .receipt
                .previous_receipt_hash
                .map(|hash| hash.0.to_vec()),
        )
        .bind(receipt_json)
        .bind(received_at)
        .execute(&mut *transaction)
        .await
        .map_err(sql_error)?;

        sqlx::query(
            r#"
            INSERT INTO desci_authority_outbox (
                id, topic, aggregate_id, aggregate_sequence,
                payload, payload_hash, created_at
            ) VALUES ($1, $2, $3, $4, $5, $6, $7)
            "#,
        )
        .bind(outbox_id)
        .bind("scientific.event.committed.v1")
        .bind(event.envelope.stream_id.to_string())
        .bind(sql_sequence)
        .bind(outbox_delivery)
        .bind(outbox_delivery_hash.0.to_vec())
        .bind(received_at)
        .execute(&mut *transaction)
        .await
        .map_err(sql_error)?;

        transaction.commit().await.map_err(sql_error)?;
        Ok(append_receipt)
    }
}

#[async_trait]
impl ScientificEventLog for PostgresAuthorityStore {
    async fn append_at(
        &self,
        _expected_sequence: u64,
        _event: SignedScientificEvent,
        _received_at: DateTime<Utc>,
    ) -> Result<AppendReceipt> {
        Err(Error::Storage(
            "PostgreSQL scientific events must be appended through the governed atomic commit path"
                .to_string(),
        ))
    }

    async fn head(&self, claim_id: ClaimId) -> Result<Option<StreamHead>> {
        let row = sqlx::query(
            r#"
            SELECT sequence, event_id, event_hash
            FROM desci_scientific_events
            WHERE stream_id = $1
            ORDER BY sequence DESC
            LIMIT 1
            "#,
        )
        .bind(claim_id.0)
        .fetch_optional(&self.pool)
        .await
        .map_err(sql_error)?;
        row.map(|row| {
            Ok(StreamHead {
                sequence: u64::try_from(row.try_get::<i64, _>("sequence").map_err(sql_error)?)
                    .map_err(|_| Error::Storage("negative PostgreSQL sequence".to_string()))?,
                event_id: ScientificEventId(row.try_get("event_id").map_err(sql_error)?),
                event_hash: content_hash_from_bytes(
                    row.try_get("event_hash").map_err(sql_error)?,
                    "event hash",
                )?,
            })
        })
        .transpose()
    }

    async fn read(&self, claim_id: ClaimId, from_sequence: u64, limit: usize) -> Result<EventPage> {
        let limit = limit.clamp(1, MAX_PAGE_SIZE);
        let rows = sqlx::query(
            r#"
            SELECT stream_id, sequence, event_id, event_hash, actor_id,
                   idempotency_key, received_at, event_json
            FROM desci_scientific_events
            WHERE stream_id = $1 AND sequence >= $2
            ORDER BY sequence
            LIMIT $3
            "#,
        )
        .bind(claim_id.0)
        .bind(sequence_to_i64(from_sequence)?)
        .bind(i64::try_from(limit + 1).map_err(|_| {
            Error::Validation("event page limit exceeds PostgreSQL range".to_string())
        })?)
        .fetch_all(&self.pool)
        .await
        .map_err(sql_error)?;
        let mut events = rows
            .into_iter()
            .map(event_from_row)
            .collect::<Result<Vec<_>>>()?;
        let next_sequence = if events.len() > limit {
            events.pop();
            Some(from_sequence + events.len() as u64)
        } else {
            None
        };
        Ok(EventPage {
            events,
            next_sequence,
        })
    }

    async fn stream(&self, claim_id: ClaimId) -> Result<Vec<SignedScientificEvent>> {
        let rows = sqlx::query(
            r#"
            SELECT stream_id, sequence, event_id, event_hash, actor_id,
                   idempotency_key, received_at, event_json
            FROM desci_scientific_events
            WHERE stream_id = $1
            ORDER BY sequence
            "#,
        )
        .bind(claim_id.0)
        .fetch_all(&self.pool)
        .await
        .map_err(sql_error)?;
        rows.into_iter().map(event_from_row).collect()
    }

    async fn stream_ids(&self) -> Result<Vec<ClaimId>> {
        let rows = sqlx::query(
            "SELECT DISTINCT stream_id FROM desci_scientific_events ORDER BY stream_id",
        )
        .fetch_all(&self.pool)
        .await
        .map_err(sql_error)?;
        rows.into_iter()
            .map(|row| {
                Ok(ClaimId(
                    row.try_get::<Uuid, _>("stream_id").map_err(sql_error)?,
                ))
            })
            .collect()
    }

    async fn event_by_id(
        &self,
        event_id: ScientificEventId,
    ) -> Result<Option<SignedScientificEvent>> {
        let row = sqlx::query(
            r#"
            SELECT stream_id, sequence, event_id, event_hash, actor_id,
                   idempotency_key, received_at, event_json
            FROM desci_scientific_events
            WHERE event_id = $1
            "#,
        )
        .bind(event_id.0)
        .fetch_optional(&self.pool)
        .await
        .map_err(sql_error)?;
        row.map(event_from_row).transpose()
    }

    async fn event_by_hash(&self, hash: ContentHash) -> Result<Option<SignedScientificEvent>> {
        let row = sqlx::query(
            r#"
            SELECT stream_id, sequence, event_id, event_hash, actor_id,
                   idempotency_key, received_at, event_json
            FROM desci_scientific_events
            WHERE event_hash = $1
            "#,
        )
        .bind(hash.0.to_vec())
        .fetch_optional(&self.pool)
        .await
        .map_err(sql_error)?;
        row.map(event_from_row).transpose()
    }
}

#[async_trait]
impl ScientificAuthorityAuditStore for PostgresAuthorityStore {
    async fn prepare(&self, _receipt: SignedScientificAuthorityReceipt) -> Result<()> {
        Err(Error::Storage(
            "PostgreSQL authority receipts must be committed with their scientific event through the atomic governed path"
                .to_string(),
        ))
    }

    async fn commit(&self, _event_id: ScientificEventId) -> Result<()> {
        Err(Error::Storage(
            "PostgreSQL authority receipts do not support separate finalization".to_string(),
        ))
    }

    async fn abort(&self, _event_id: ScientificEventId) -> Result<()> {
        Err(Error::Storage(
            "PostgreSQL atomic scientific commits roll back as one transaction".to_string(),
        ))
    }

    async fn receipt(
        &self,
        event_id: ScientificEventId,
    ) -> Result<Option<SignedScientificAuthorityReceipt>> {
        let row = sqlx::query(
            r#"
            SELECT event_id, stream_id, sequence, receipt_hash,
                   previous_receipt_hash, status, prepared_at, committed_at,
                   receipt_json
            FROM desci_authority_receipts
            WHERE event_id = $1
            "#,
        )
        .bind(event_id.0)
        .fetch_optional(&self.pool)
        .await
        .map_err(sql_error)?;
        let receipt = row.map(receipt_from_row).transpose()?;
        match receipt {
            Some(receipt) => Ok(Some(self.verify_persisted_receipt(receipt).await?)),
            None => Ok(None),
        }
    }

    async fn status(&self, event_id: ScientificEventId) -> Result<AuthorityAttestationStatus> {
        let row = sqlx::query("SELECT status FROM desci_authority_receipts WHERE event_id = $1")
            .bind(event_id.0)
            .fetch_optional(&self.pool)
            .await
            .map_err(sql_error)?;
        if let Some(row) = row {
            let status: String = row.try_get("status").map_err(sql_error)?;
            return match status.as_str() {
                "committed" => Ok(AuthorityAttestationStatus::ReceiptAttested),
                "pending" => Ok(AuthorityAttestationStatus::PendingReconciliation),
                other => Err(Error::Storage(format!(
                    "unknown PostgreSQL authority receipt status: {other}"
                ))),
            };
        }
        let event = sqlx::query(
            "SELECT stream_id, sequence FROM desci_scientific_events WHERE event_id = $1",
        )
        .bind(event_id.0)
        .fetch_optional(&self.pool)
        .await
        .map_err(sql_error)?;
        let Some(event) = event else {
            return Ok(AuthorityAttestationStatus::Unknown);
        };
        let stream_id: Uuid = event.try_get("stream_id").map_err(sql_error)?;
        let sequence: i64 = event.try_get("sequence").map_err(sql_error)?;
        let first_receipt: Option<i64> = sqlx::query_scalar(
            r#"
            SELECT MIN(sequence)
            FROM desci_authority_receipts
            WHERE stream_id = $1 AND status = 'committed'
            "#,
        )
        .bind(stream_id)
        .fetch_one(&self.pool)
        .await
        .map_err(sql_error)?;
        Ok(match first_receipt {
            None => AuthorityAttestationStatus::LegacyUnattested,
            Some(first) if sequence < first => AuthorityAttestationStatus::LegacyUnattested,
            Some(_) => AuthorityAttestationStatus::UnsafeUnattested,
        })
    }

    async fn stream_head(
        &self,
        claim_id: ClaimId,
    ) -> Result<Option<SignedScientificAuthorityReceipt>> {
        let row = sqlx::query(
            r#"
            SELECT event_id, stream_id, sequence, receipt_hash,
                   previous_receipt_hash, status, prepared_at, committed_at,
                   receipt_json
            FROM desci_authority_receipts
            WHERE stream_id = $1 AND status = 'committed'
            ORDER BY sequence DESC
            LIMIT 1
            "#,
        )
        .bind(claim_id.0)
        .fetch_optional(&self.pool)
        .await
        .map_err(sql_error)?;
        let receipt = row.map(receipt_from_row).transpose()?;
        match receipt {
            Some(receipt) => Ok(Some(self.verify_persisted_receipt(receipt).await?)),
            None => Ok(None),
        }
    }

    async fn stream_receipts(
        &self,
        claim_id: ClaimId,
    ) -> Result<Vec<SignedScientificAuthorityReceipt>> {
        let rows = sqlx::query(
            r#"
            SELECT event_id, stream_id, sequence, receipt_hash,
                   previous_receipt_hash, status, prepared_at, committed_at,
                   receipt_json
            FROM desci_authority_receipts
            WHERE stream_id = $1 AND status = 'committed'
            ORDER BY sequence
            "#,
        )
        .bind(claim_id.0)
        .fetch_all(&self.pool)
        .await
        .map_err(sql_error)?;
        let receipts = rows
            .into_iter()
            .map(receipt_from_row)
            .collect::<Result<Vec<_>>>()?;
        let mut verified = Vec::with_capacity(receipts.len());
        for receipt in receipts {
            verified.push(self.verify_persisted_receipt(receipt).await?);
        }
        Ok(verified)
    }

    async fn reconcile(
        &self,
        _event_log: &dyn ScientificEventLog,
    ) -> Result<AuthorityAuditSummary> {
        let events = self.all_events().await?;
        let mut event_map = HashMap::new();
        let mut event_received_at = HashMap::new();
        let mut event_streams: BTreeMap<ClaimId, Vec<SignedScientificEvent>> = BTreeMap::new();
        for (event, received_at) in events {
            event.verify()?;
            event_streams
                .entry(event.envelope.stream_id)
                .or_default()
                .push(event.clone());
            event_received_at.insert(event.envelope.event_id, received_at);
            if event_map.insert(event.envelope.event_id, event).is_some() {
                return Err(Error::VerificationFailed(
                    "duplicate scientific event id in PostgreSQL replay".to_string(),
                ));
            }
        }
        for stream in event_streams.values() {
            ClaimProjection::rebuild(stream)?;
        }

        let receipt_rows = sqlx::query(
            r#"
            SELECT event_id, stream_id, sequence, receipt_hash,
                   previous_receipt_hash, status, prepared_at, committed_at,
                   receipt_json
            FROM desci_authority_receipts
            ORDER BY stream_id, sequence
            "#,
        )
        .fetch_all(&self.pool)
        .await
        .map_err(sql_error)?;
        let mut committed = 0usize;
        let mut pending = 0usize;
        let mut receipt_streams = BTreeSet::new();
        let mut committed_ids = BTreeSet::new();
        let mut previous_receipts: BTreeMap<ClaimId, ContentHash> = BTreeMap::new();
        let mut previous_receipt_sequences: BTreeMap<ClaimId, u64> = BTreeMap::new();
        let mut first_receipt_sequences: BTreeMap<ClaimId, u64> = BTreeMap::new();
        for row in receipt_rows {
            let status: String = row.try_get("status").map_err(sql_error)?;
            let receipt = receipt_from_row(row)?;
            if status == "pending" {
                pending += 1;
                let event = event_map.get(&receipt.receipt.event_id).ok_or_else(|| {
                    Error::VerificationFailed(
                        "PostgreSQL pending authority receipt has no corresponding scientific event"
                            .to_string(),
                    )
                })?;
                receipt.verify_for_event(event, self.trusted_receipt_keys.as_ref())?;
                if event_received_at.get(&receipt.receipt.event_id)
                    != Some(&receipt.receipt.received_at)
                {
                    return Err(Error::VerificationFailed(
                        "PostgreSQL pending receipt disagrees with server receipt time".to_string(),
                    ));
                }
                continue;
            }
            if status != "committed" {
                return Err(Error::Storage(format!(
                    "unknown PostgreSQL authority receipt status: {status}"
                )));
            }
            let event = event_map.get(&receipt.receipt.event_id).ok_or_else(|| {
                Error::VerificationFailed(
                    "PostgreSQL committed authority receipt has no corresponding scientific event"
                        .to_string(),
                )
            })?;
            receipt.verify_for_event(event, self.trusted_receipt_keys.as_ref())?;
            if event_received_at.get(&receipt.receipt.event_id)
                != Some(&receipt.receipt.received_at)
            {
                return Err(Error::VerificationFailed(
                    "PostgreSQL scientific event and authority receipt disagree on server receipt time"
                        .to_string(),
                ));
            }
            let previous = previous_receipts.get(&receipt.receipt.stream_id).copied();
            if let Some(previous) = previous {
                let previous_sequence = previous_receipt_sequences
                    .get(&receipt.receipt.stream_id)
                    .copied()
                    .ok_or_else(|| {
                        Error::VerificationFailed(
                            "PostgreSQL receipt sequence state is incomplete".to_string(),
                        )
                    })?;
                if previous_sequence.checked_add(1) != Some(receipt.receipt.sequence)
                    || receipt.receipt.previous_receipt_hash != Some(previous)
                    || receipt.receipt.legacy_cutover_anchor.is_some()
                {
                    return Err(Error::VerificationFailed(
                        "PostgreSQL authority receipt chain is discontinuous".to_string(),
                    ));
                }
            } else if receipt.receipt.sequence == 0 {
                if receipt.receipt.previous_receipt_hash.is_some()
                    || receipt.receipt.legacy_cutover_anchor.is_some()
                {
                    return Err(Error::VerificationFailed(
                        "genesis PostgreSQL authority receipt has a predecessor".to_string(),
                    ));
                }
            } else if receipt.receipt.previous_receipt_hash.is_some()
                || receipt.receipt.legacy_cutover_anchor != event.envelope.previous_hash
            {
                return Err(Error::VerificationFailed(
                    "first PostgreSQL authority receipt does not bind the legacy cutover"
                        .to_string(),
                ));
            }
            first_receipt_sequences
                .entry(receipt.receipt.stream_id)
                .or_insert(receipt.receipt.sequence);
            previous_receipt_sequences.insert(receipt.receipt.stream_id, receipt.receipt.sequence);
            previous_receipts.insert(receipt.receipt.stream_id, receipt.receipt_hash()?);
            committed += 1;
            committed_ids.insert(receipt.receipt.event_id);
            receipt_streams.insert(receipt.receipt.stream_id);
        }
        let mut legacy_unattested = 0usize;
        let mut unsafe_unattested = 0usize;
        for event in event_map.values() {
            if committed_ids.contains(&event.envelope.event_id) {
                continue;
            }
            match first_receipt_sequences.get(&event.envelope.stream_id) {
                None => legacy_unattested += 1,
                Some(first) if event.envelope.sequence < *first => legacy_unattested += 1,
                Some(_) => unsafe_unattested += 1,
            }
        }
        let unattested = legacy_unattested + unsafe_unattested;
        Ok(AuthorityAuditSummary {
            committed_receipts: committed,
            pending_receipts: pending,
            unattested_events: unattested,
            legacy_unattested_events: legacy_unattested,
            unsafe_unattested_events: unsafe_unattested,
            receipt_chains: receipt_streams.len(),
            last_reconciled_at: Some(Utc::now()),
        })
    }

    async fn summary(&self) -> Result<AuthorityAuditSummary> {
        self.reconcile(self).await
    }
}

fn event_with_received_at_from_row(row: PgRow) -> Result<(SignedScientificEvent, DateTime<Utc>)> {
    let received_at: DateTime<Utc> = row.try_get("received_at").map_err(sql_error)?;
    Ok((event_from_row(row)?, received_at))
}

fn event_from_row(row: PgRow) -> Result<SignedScientificEvent> {
    let value: serde_json::Value = row.try_get("event_json").map_err(sql_error)?;
    let event: SignedScientificEvent = serde_json::from_value(value)?;
    event.verify()?;
    let stream_id: Uuid = row.try_get("stream_id").map_err(sql_error)?;
    let sequence = u64::try_from(row.try_get::<i64, _>("sequence").map_err(sql_error)?)
        .map_err(|_| Error::Storage("negative PostgreSQL event sequence".to_string()))?;
    let event_id: Uuid = row.try_get("event_id").map_err(sql_error)?;
    let event_hash =
        content_hash_from_bytes(row.try_get("event_hash").map_err(sql_error)?, "event hash")?;
    let actor_id: String = row.try_get("actor_id").map_err(sql_error)?;
    let idempotency_key: Option<String> = row.try_get("idempotency_key").map_err(sql_error)?;
    if event.envelope.stream_id.0 != stream_id
        || event.envelope.sequence != sequence
        || event.envelope.event_id.0 != event_id
        || event.event_hash()? != event_hash
        || event.envelope.actor.as_str() != actor_id
        || event.envelope.idempotency_key != idempotency_key
    {
        return Err(Error::VerificationFailed(
            "PostgreSQL scientific event JSON does not match its indexed columns".to_string(),
        ));
    }
    Ok(event)
}

fn receipt_from_row(row: PgRow) -> Result<SignedScientificAuthorityReceipt> {
    let value: serde_json::Value = row.try_get("receipt_json").map_err(sql_error)?;
    let receipt: SignedScientificAuthorityReceipt = serde_json::from_value(value)?;
    let event_id: Uuid = row.try_get("event_id").map_err(sql_error)?;
    let stream_id: Uuid = row.try_get("stream_id").map_err(sql_error)?;
    let sequence = u64::try_from(row.try_get::<i64, _>("sequence").map_err(sql_error)?)
        .map_err(|_| Error::Storage("negative PostgreSQL receipt sequence".to_string()))?;
    let receipt_hash = content_hash_from_bytes(
        row.try_get("receipt_hash").map_err(sql_error)?,
        "receipt hash",
    )?;
    let previous_receipt_hash = row
        .try_get::<Option<Vec<u8>>, _>("previous_receipt_hash")
        .map_err(sql_error)?
        .map(|bytes| content_hash_from_bytes(bytes, "previous receipt hash"))
        .transpose()?;
    let status: String = row.try_get("status").map_err(sql_error)?;
    let prepared_at: DateTime<Utc> = row.try_get("prepared_at").map_err(sql_error)?;
    let committed_at: Option<DateTime<Utc>> = row.try_get("committed_at").map_err(sql_error)?;
    let lifecycle_matches = match status.as_str() {
        "pending" => committed_at.is_none(),
        "committed" => committed_at.as_ref() == Some(&receipt.receipt.received_at),
        _ => false,
    };
    if receipt.receipt.event_id.0 != event_id
        || receipt.receipt.stream_id.0 != stream_id
        || receipt.receipt.sequence != sequence
        || receipt.receipt_hash()? != receipt_hash
        || receipt.receipt.previous_receipt_hash != previous_receipt_hash
        || prepared_at != receipt.receipt.received_at
        || !lifecycle_matches
    {
        return Err(Error::VerificationFailed(
            "PostgreSQL authority receipt JSON does not match its indexed columns".to_string(),
        ));
    }
    Ok(receipt)
}

fn checkpoint_mirror_record_from_row(row: PgRow) -> Result<CheckpointMirrorRecord> {
    let value: serde_json::Value = row.try_get("observation_json").map_err(sql_error)?;
    let observation: SignedCheckpointMirrorObservation = serde_json::from_value(value)?;
    observation.verify()?;
    let observation_id: Uuid = row.try_get("observation_id").map_err(sql_error)?;
    let checkpoint_hash = content_hash_from_bytes(
        row.try_get("checkpoint_hash").map_err(sql_error)?,
        "checkpoint mirror hash",
    )?;
    let actor: String = row.try_get("mirror_actor").map_err(sql_error)?;
    let organization: String = row.try_get("mirror_organization").map_err(sql_error)?;
    let mirror_uri: String = row.try_get("mirror_uri").map_err(sql_error)?;
    let observed_at: DateTime<Utc> = row.try_get("observed_at").map_err(sql_error)?;
    let accepted_at: DateTime<Utc> = row.try_get("accepted_at").map_err(sql_error)?;
    if observation.observation_id.0 != observation_id
        || observation.checkpoint_hash != checkpoint_hash
        || observation.actor.as_str() != actor
        || observation.organization.as_str() != organization
        || observation.mirror_uri != mirror_uri
        || observation.observed_at != observed_at
    {
        return Err(Error::VerificationFailed(
            "PostgreSQL mirror observation JSON does not match its indexed columns".to_string(),
        ));
    }
    if observation.observed_at
        > accepted_at + chrono::Duration::seconds(MAX_MIRROR_FUTURE_SKEW_SECONDS)
    {
        return Err(Error::VerificationFailed(
            "PostgreSQL mirror observation exceeds acceptance-time clock skew".to_string(),
        ));
    }
    let observation_hash = observation.observation_hash()?;
    Ok(CheckpointMirrorRecord {
        observation,
        observation_hash,
        accepted_at,
    })
}

fn outbox_from_row(row: PgRow) -> Result<AuthorityOutboxMessage> {
    let value: serde_json::Value = row.try_get("payload").map_err(sql_error)?;
    let delivery: SignedAuthorityDeliveryEnvelope = serde_json::from_value(value)?;
    delivery.verify()?;
    let delivery_hash = content_hash_from_bytes(
        row.try_get("payload_hash").map_err(sql_error)?,
        "outbox delivery hash",
    )?;
    if delivery.delivery_hash()? != delivery_hash {
        return Err(Error::VerificationFailed(
            "PostgreSQL outbox delivery does not match its stored hash".to_string(),
        ));
    }
    let id: Uuid = row.try_get("id").map_err(sql_error)?;
    let topic: String = row.try_get("topic").map_err(sql_error)?;
    let aggregate_id: String = row.try_get("aggregate_id").map_err(sql_error)?;
    let aggregate_sequence: i64 = row.try_get("aggregate_sequence").map_err(sql_error)?;
    let created_at: DateTime<Utc> = row.try_get("created_at").map_err(sql_error)?;
    if delivery.envelope.delivery_id != id
        || delivery.envelope.topic != topic
        || delivery.envelope.aggregate_id != aggregate_id
        || delivery.envelope.aggregate_sequence != aggregate_sequence
        || delivery.envelope.created_at != created_at
    {
        return Err(Error::VerificationFailed(
            "PostgreSQL outbox delivery JSON does not match its indexed columns".to_string(),
        ));
    }
    Ok(AuthorityOutboxMessage {
        id,
        topic,
        aggregate_id,
        aggregate_sequence,
        delivery,
        delivery_hash,
        created_at,
        attempts: row.try_get("attempts").map_err(sql_error)?,
        lease_owner: row.try_get("lease_owner").map_err(sql_error)?,
        lease_until: row.try_get("lease_until").map_err(sql_error)?,
    })
}

fn content_hash_from_bytes(bytes: Vec<u8>, label: &str) -> Result<ContentHash> {
    let bytes: [u8; 32] = bytes
        .try_into()
        .map_err(|_| Error::Storage(format!("PostgreSQL {label} must contain exactly 32 bytes")))?;
    Ok(ContentHash(bytes))
}

fn sequence_to_i64(sequence: u64) -> Result<i64> {
    i64::try_from(sequence).map_err(|_| {
        Error::Validation("scientific event sequence exceeds PostgreSQL BIGINT".to_string())
    })
}

fn validate_worker_id(worker_id: &str) -> Result<()> {
    let worker_id = worker_id.trim();
    if worker_id.is_empty() || worker_id.len() > 256 || worker_id.chars().any(char::is_control) {
        return Err(Error::Validation(
            "outbox worker id must be 1-256 printable bytes".to_string(),
        ));
    }
    Ok(())
}

pub(crate) const AUTHORITY_EPOCH_BARRIER_LOCK_KEY: i64 = 730_391_904_224;

pub(crate) async fn lock_authority_epoch_shared(
    transaction: &mut sqlx::Transaction<'_, sqlx::Postgres>,
) -> Result<()> {
    sqlx::query("SELECT pg_advisory_xact_lock_shared($1)")
        .bind(AUTHORITY_EPOCH_BARRIER_LOCK_KEY)
        .execute(&mut **transaction)
        .await
        .map_err(sql_error)?;
    Ok(())
}

pub(crate) async fn lock_authority_epoch_exclusive(
    transaction: &mut sqlx::Transaction<'_, sqlx::Postgres>,
) -> Result<()> {
    sqlx::query("SELECT pg_advisory_xact_lock($1)")
        .bind(AUTHORITY_EPOCH_BARRIER_LOCK_KEY)
        .execute(&mut **transaction)
        .await
        .map_err(sql_error)?;
    Ok(())
}

#[derive(Clone, Copy)]
enum AuthorityWriteFenceTarget<'a> {
    CurrentEpoch,
    CandidateEpoch {
        deployment_id: &'a str,
        primary_id: &'a str,
        database_system_identifier: &'a str,
        postgres_timeline: u64,
        epoch_number: u64,
        epoch_hash: ContentHash,
    },
}

async fn begin_read_only_repeatable_read(
    transaction: &mut sqlx::Transaction<'_, sqlx::Postgres>,
) -> Result<()> {
    sqlx::query("SET TRANSACTION ISOLATION LEVEL REPEATABLE READ, READ ONLY")
        .execute(&mut **transaction)
        .await
        .map_err(sql_error)?;
    Ok(())
}

async fn postgres_identity(
    transaction: &mut sqlx::Transaction<'_, sqlx::Postgres>,
) -> Result<(String, u64)> {
    let identity = sqlx::query(
        r#"
        SELECT system.system_identifier::text AS system_identifier,
               checkpoint.timeline_id::bigint AS timeline_id
        FROM pg_control_system() AS system
        CROSS JOIN pg_control_checkpoint() AS checkpoint
        "#,
    )
    .fetch_one(&mut **transaction)
    .await
    .map_err(sql_error)?;
    let system_identifier: String = identity.try_get("system_identifier").map_err(sql_error)?;
    let timeline_id: i64 = identity.try_get("timeline_id").map_err(sql_error)?;
    let timeline_id = u64::try_from(timeline_id).map_err(|_| {
        Error::Storage("PostgreSQL reported a negative timeline identifier".to_string())
    })?;
    Ok((system_identifier, timeline_id))
}

async fn verify_write_lease_target(
    transaction: &mut sqlx::Transaction<'_, sqlx::Postgres>,
    signed: &SignedAuthorityWriteLease,
    target: AuthorityWriteFenceTarget<'_>,
) -> Result<()> {
    let (system_identifier, timeline_id) = postgres_identity(transaction).await?;
    if signed.lease.database_system_identifier != system_identifier
        || signed.lease.postgres_timeline != timeline_id
    {
        return Err(Error::VerificationFailed(format!(
            "authority write lease targets PostgreSQL system {}/timeline {}, but the connected database reports {system_identifier}/timeline {timeline_id}",
            signed.lease.database_system_identifier, signed.lease.postgres_timeline,
        )));
    }

    match target {
        AuthorityWriteFenceTarget::CurrentEpoch => {
            let row = sqlx::query(
                r#"
                SELECT epoch_number, epoch_hash, deployment_id, primary_id, certificate_json
                FROM desci_database_epochs
                ORDER BY epoch_number DESC
                LIMIT 1
                "#,
            )
            .fetch_optional(&mut **transaction)
            .await
            .map_err(sql_error)?;
            match row {
                None => {
                    if signed.lease.phase != AuthorityWriteLeasePhase::Bootstrap
                        || signed.lease.epoch_number.is_some()
                        || signed.lease.epoch_hash.is_some()
                    {
                        return Err(Error::VerificationFailed(
                            "database has no governed epoch; only a bootstrap write lease is valid"
                                .to_string(),
                        ));
                    }
                }
                Some(row) => {
                    let epoch_number: i64 = row.try_get("epoch_number").map_err(sql_error)?;
                    let epoch_number = u64::try_from(epoch_number).map_err(|_| {
                        Error::Storage("database epoch number is negative".to_string())
                    })?;
                    let epoch_hash = content_hash_from_bytes(
                        row.try_get("epoch_hash").map_err(sql_error)?,
                        "database epoch hash",
                    )?;
                    let deployment_id: String = row.try_get("deployment_id").map_err(sql_error)?;
                    let primary_id: String = row.try_get("primary_id").map_err(sql_error)?;
                    let certificate_value: serde_json::Value =
                        row.try_get("certificate_json").map_err(sql_error)?;
                    let certificate: SignedAuthorityDatabaseEpochCertificate =
                        serde_json::from_value(certificate_value)?;
                    certificate.verify()?;
                    if certificate.certificate.intent.epoch_number != epoch_number
                        || certificate.epoch_hash()? != epoch_hash
                        || certificate.certificate.intent.deployment_id != deployment_id
                        || certificate.certificate.intent.candidate_primary_id != primary_id
                        || certificate.certificate.intent.database_system_identifier
                            != system_identifier
                        || certificate.certificate.intent.postgres_timeline != timeline_id
                    {
                        return Err(Error::VerificationFailed(
                            "durable database epoch indexes disagree with the signed certificate"
                                .to_string(),
                        ));
                    }
                    verify_epoch_bound_lease(
                        &signed.lease,
                        &deployment_id,
                        &primary_id,
                        &system_identifier,
                        timeline_id,
                        epoch_number,
                        epoch_hash,
                    )?;
                }
            }
        }
        AuthorityWriteFenceTarget::CandidateEpoch {
            deployment_id,
            primary_id,
            database_system_identifier,
            postgres_timeline,
            epoch_number,
            epoch_hash,
        } => {
            verify_epoch_bound_lease(
                &signed.lease,
                deployment_id,
                primary_id,
                database_system_identifier,
                postgres_timeline,
                epoch_number,
                epoch_hash,
            )?;
        }
    }
    Ok(())
}

fn verify_epoch_bound_lease(
    lease: &crate::authority_fencing::AuthorityWriteLease,
    deployment_id: &str,
    primary_id: &str,
    database_system_identifier: &str,
    postgres_timeline: u64,
    epoch_number: u64,
    epoch_hash: ContentHash,
) -> Result<()> {
    if lease.phase != AuthorityWriteLeasePhase::Epoch
        || lease.deployment_id != deployment_id
        || lease.primary_id != primary_id
        || lease.database_system_identifier != database_system_identifier
        || lease.postgres_timeline != postgres_timeline
        || lease.epoch_number != Some(epoch_number)
        || lease.epoch_hash != Some(epoch_hash)
    {
        return Err(Error::VerificationFailed(
            "authority write lease does not bind the exact governed database epoch and primary"
                .to_string(),
        ));
    }
    Ok(())
}

async fn persist_authority_write_fence(
    transaction: &mut sqlx::Transaction<'_, sqlx::Postgres>,
    signed: &SignedAuthorityWriteLease,
    scope: AuthorityWriteScope,
) -> Result<()> {
    let lease_hash = signed.signed_lease_hash()?;
    let generation = sequence_to_i64(signed.lease.generation)?;
    let epoch_number = signed.lease.epoch_number.map(sequence_to_i64).transpose()?;
    let postgres_timeline = sequence_to_i64(signed.lease.postgres_timeline)?;
    let existing = sqlx::query(
        r#"
        SELECT generation, lease_id, lease_hash
        FROM desci_authority_write_fencing_state
        WHERE deployment_id = $1
        FOR UPDATE
        "#,
    )
    .bind(&signed.lease.deployment_id)
    .fetch_optional(&mut **transaction)
    .await
    .map_err(sql_error)?;
    if let Some(row) = existing {
        let durable_generation: i64 = row.try_get("generation").map_err(sql_error)?;
        let durable_lease_id: Uuid = row.try_get("lease_id").map_err(sql_error)?;
        let durable_lease_hash = content_hash_from_bytes(
            row.try_get("lease_hash").map_err(sql_error)?,
            "durable authority write lease hash",
        )?;
        if durable_generation > generation {
            return Err(Error::VerificationFailed(format!(
                "stale authority write lease generation {generation}; durable generation is {durable_generation}"
            )));
        }
        if durable_generation == generation
            && (durable_lease_id != signed.lease.lease_id || durable_lease_hash != lease_hash)
        {
            return Err(Error::VerificationFailed(
                "authority write lease generation is already bound to different signed bytes"
                    .to_string(),
            ));
        }
    }

    sqlx::query(
        r#"
        INSERT INTO desci_authority_write_fencing_state (
            deployment_id, generation, lease_id, lease_hash, signer_public_key,
            phase, epoch_number, epoch_hash, primary_id, database_system_identifier,
            postgres_timeline, expires_at, last_scope, last_seen_at, lease_json
        ) VALUES (
            $1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, $13,
            clock_timestamp(), $14
        )
        ON CONFLICT (deployment_id) DO UPDATE SET
            generation = EXCLUDED.generation,
            lease_id = EXCLUDED.lease_id,
            lease_hash = EXCLUDED.lease_hash,
            signer_public_key = EXCLUDED.signer_public_key,
            phase = EXCLUDED.phase,
            epoch_number = EXCLUDED.epoch_number,
            epoch_hash = EXCLUDED.epoch_hash,
            primary_id = EXCLUDED.primary_id,
            database_system_identifier = EXCLUDED.database_system_identifier,
            postgres_timeline = EXCLUDED.postgres_timeline,
            expires_at = EXCLUDED.expires_at,
            last_scope = EXCLUDED.last_scope,
            last_seen_at = clock_timestamp(),
            lease_json = EXCLUDED.lease_json
        "#,
    )
    .bind(&signed.lease.deployment_id)
    .bind(generation)
    .bind(signed.lease.lease_id)
    .bind(lease_hash.0.to_vec())
    .bind(signed.signer_public_key.to_vec())
    .bind(match signed.lease.phase {
        AuthorityWriteLeasePhase::Bootstrap => "bootstrap",
        AuthorityWriteLeasePhase::Epoch => "epoch",
    })
    .bind(epoch_number)
    .bind(signed.lease.epoch_hash.map(|hash| hash.0.to_vec()))
    .bind(&signed.lease.primary_id)
    .bind(&signed.lease.database_system_identifier)
    .bind(postgres_timeline)
    .bind(signed.lease.expires_at)
    .bind(format!("{scope:?}").to_ascii_lowercase())
    .bind(serde_json::to_value(signed)?)
    .execute(&mut **transaction)
    .await
    .map_err(sql_error)?;
    Ok(())
}

fn sql_error(error: impl std::fmt::Display) -> Error {
    Error::Storage(format!("PostgreSQL authority backend: {error}"))
}

const POSTGRES_AUTHORITY_MIGRATION_MARKER_SQL: &str = r#"
    INSERT INTO desci_authority_schema_migrations (version, migration_id)
    VALUES ($1, $2)
    ON CONFLICT (version) DO NOTHING
"#;

const POSTGRES_AUTHORITY_MIGRATION: &[&str] = &[
    r#"
    CREATE TABLE IF NOT EXISTS desci_authority_schema_migrations (
        version BIGINT PRIMARY KEY CHECK (version > 0),
        migration_id TEXT NOT NULL UNIQUE,
        applied_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp()
    )
    "#,
    r#"
    CREATE TABLE IF NOT EXISTS desci_scientific_events (
        stream_id UUID NOT NULL,
        sequence BIGINT NOT NULL CHECK (sequence >= 0),
        event_id UUID NOT NULL UNIQUE,
        event_hash BYTEA NOT NULL UNIQUE CHECK (octet_length(event_hash) = 32),
        actor_id TEXT NOT NULL,
        idempotency_key TEXT,
        received_at TIMESTAMPTZ NOT NULL,
        event_json JSONB NOT NULL,
        PRIMARY KEY (stream_id, sequence)
    )
    "#,
    r#"
    CREATE UNIQUE INDEX IF NOT EXISTS desci_scientific_event_idempotency_idx
    ON desci_scientific_events (actor_id, idempotency_key)
    WHERE idempotency_key IS NOT NULL
    "#,
    r#"
    CREATE TABLE IF NOT EXISTS desci_authority_receipts (
        event_id UUID PRIMARY KEY REFERENCES desci_scientific_events(event_id)
            DEFERRABLE INITIALLY DEFERRED,
        stream_id UUID NOT NULL,
        sequence BIGINT NOT NULL CHECK (sequence >= 0),
        receipt_hash BYTEA NOT NULL UNIQUE CHECK (octet_length(receipt_hash) = 32),
        previous_receipt_hash BYTEA CHECK (
            previous_receipt_hash IS NULL OR octet_length(previous_receipt_hash) = 32
        ),
        status TEXT NOT NULL CHECK (status IN ('pending', 'committed')),
        receipt_json JSONB NOT NULL,
        prepared_at TIMESTAMPTZ NOT NULL,
        committed_at TIMESTAMPTZ,
        CHECK (
            (status = 'pending' AND committed_at IS NULL)
            OR (status = 'committed' AND committed_at IS NOT NULL)
        ),
        UNIQUE (stream_id, sequence),
        FOREIGN KEY (stream_id, sequence)
            REFERENCES desci_scientific_events(stream_id, sequence)
            DEFERRABLE INITIALLY DEFERRED
    )
    "#,
    r#"
    CREATE TABLE IF NOT EXISTS desci_authority_outbox (
        id UUID PRIMARY KEY,
        topic TEXT NOT NULL,
        aggregate_id TEXT NOT NULL,
        aggregate_sequence BIGINT NOT NULL,
        payload JSONB NOT NULL,
        payload_hash BYTEA NOT NULL CHECK (octet_length(payload_hash) = 32),
        created_at TIMESTAMPTZ NOT NULL,
        published_at TIMESTAMPTZ,
        attempts INTEGER NOT NULL DEFAULT 0 CHECK (attempts >= 0),
        lease_owner TEXT,
        lease_until TIMESTAMPTZ,
        last_error TEXT,
        UNIQUE (topic, aggregate_id, aggregate_sequence)
    )
    "#,
    r#"
    CREATE INDEX IF NOT EXISTS desci_authority_outbox_pending_idx
    ON desci_authority_outbox (created_at, id)
    WHERE published_at IS NULL
    "#,
    r#"
    CREATE TABLE IF NOT EXISTS desci_credential_records (
        sequence BIGINT PRIMARY KEY CHECK (sequence >= 0),
        record_hash BYTEA NOT NULL UNIQUE CHECK (octet_length(record_hash) = 32),
        received_at TIMESTAMPTZ NOT NULL,
        record_json JSONB NOT NULL
    )
    "#,
    r#"
    CREATE TABLE IF NOT EXISTS desci_governance_records (
        sequence BIGINT PRIMARY KEY CHECK (sequence >= 0),
        record_hash BYTEA NOT NULL UNIQUE CHECK (octet_length(record_hash) = 32),
        received_at TIMESTAMPTZ NOT NULL,
        record_json JSONB NOT NULL
    )
    "#,
    r#"
    CREATE TABLE IF NOT EXISTS desci_credential_events (
        sequence BIGINT PRIMARY KEY CHECK (sequence >= 0),
        event_id UUID NOT NULL UNIQUE,
        event_hash BYTEA NOT NULL UNIQUE CHECK (octet_length(event_hash) = 32),
        record_hash BYTEA NOT NULL UNIQUE CHECK (octet_length(record_hash) = 32),
        previous_record_hash BYTEA CHECK (
            previous_record_hash IS NULL OR octet_length(previous_record_hash) = 32
        ),
        actor_id TEXT NOT NULL,
        idempotency_key TEXT,
        received_at TIMESTAMPTZ NOT NULL,
        record_json JSONB NOT NULL
    )
    "#,
    r#"
    CREATE UNIQUE INDEX IF NOT EXISTS desci_credential_idempotency_idx
    ON desci_credential_events (actor_id, idempotency_key)
    WHERE idempotency_key IS NOT NULL
    "#,
    r#"
    CREATE TABLE IF NOT EXISTS desci_governance_events (
        sequence BIGINT PRIMARY KEY CHECK (sequence >= 0),
        event_id UUID NOT NULL UNIQUE,
        event_hash BYTEA NOT NULL UNIQUE CHECK (octet_length(event_hash) = 32),
        record_hash BYTEA NOT NULL UNIQUE CHECK (octet_length(record_hash) = 32),
        previous_record_hash BYTEA CHECK (
            previous_record_hash IS NULL OR octet_length(previous_record_hash) = 32
        ),
        actor_id TEXT NOT NULL,
        idempotency_key TEXT NOT NULL,
        received_at TIMESTAMPTZ NOT NULL,
        record_json JSONB NOT NULL,
        UNIQUE (actor_id, idempotency_key)
    )
    "#,
    r#"
    CREATE TABLE IF NOT EXISTS desci_checkpoint_mirror_observations (
        observation_id UUID PRIMARY KEY,
        checkpoint_hash BYTEA NOT NULL CHECK (octet_length(checkpoint_hash) = 32),
        mirror_actor TEXT NOT NULL,
        mirror_organization TEXT NOT NULL,
        mirror_uri TEXT NOT NULL,
        observed_at TIMESTAMPTZ NOT NULL,
        accepted_at TIMESTAMPTZ NOT NULL,
        observation_json JSONB NOT NULL,
        UNIQUE (checkpoint_hash, mirror_actor)
    )
    "#,
    r#"
    CREATE TABLE IF NOT EXISTS desci_database_epochs (
        epoch_number BIGINT PRIMARY KEY CHECK (epoch_number > 0),
        epoch_id UUID NOT NULL UNIQUE,
        epoch_hash BYTEA NOT NULL UNIQUE CHECK (octet_length(epoch_hash) = 32),
        previous_epoch_hash BYTEA CHECK (
            previous_epoch_hash IS NULL OR octet_length(previous_epoch_hash) = 32
        ),
        deployment_id TEXT NOT NULL,
        primary_id TEXT NOT NULL,
        promoted_at TIMESTAMPTZ NOT NULL,
        accepted_at TIMESTAMPTZ NOT NULL CHECK (accepted_at >= promoted_at),
        state_commitment_hash BYTEA NOT NULL CHECK (
            octet_length(state_commitment_hash) = 32
        ),
        publication_delivery_id UUID NOT NULL UNIQUE REFERENCES desci_authority_outbox(id)
            DEFERRABLE INITIALLY DEFERRED,
        certificate_json JSONB NOT NULL,
        UNIQUE (deployment_id, epoch_number)
    )
    "#,
    r#"
    CREATE TABLE IF NOT EXISTS desci_recovery_reconciliations (
        reconciliation_id UUID PRIMARY KEY,
        epoch_hash BYTEA NOT NULL UNIQUE REFERENCES desci_database_epochs(epoch_hash),
        reconciliation_hash BYTEA NOT NULL UNIQUE CHECK (
            octet_length(reconciliation_hash) = 32
        ),
        verified_at TIMESTAMPTZ NOT NULL,
        accepted_at TIMESTAMPTZ NOT NULL CHECK (accepted_at >= verified_at),
        publication_delivery_id UUID NOT NULL UNIQUE REFERENCES desci_authority_outbox(id)
            DEFERRABLE INITIALLY DEFERRED,
        reconciliation_json JSONB NOT NULL
    )
    "#,
    r#"
    CREATE TABLE IF NOT EXISTS desci_authority_delivery_acknowledgements (
        acknowledgement_id UUID PRIMARY KEY,
        delivery_id UUID NOT NULL REFERENCES desci_authority_outbox(id),
        delivery_hash BYTEA NOT NULL CHECK (octet_length(delivery_hash) = 32),
        witness_actor TEXT NOT NULL,
        witness_organization TEXT NOT NULL,
        acknowledged_at TIMESTAMPTZ NOT NULL,
        accepted_at TIMESTAMPTZ NOT NULL CHECK (accepted_at >= acknowledged_at),
        acknowledgement_hash BYTEA NOT NULL UNIQUE CHECK (
            octet_length(acknowledgement_hash) = 32
        ),
        acknowledgement_json JSONB NOT NULL,
        UNIQUE (delivery_id, witness_actor)
    )
    "#,
    r#"
    CREATE INDEX IF NOT EXISTS desci_delivery_ack_delivery_idx
    ON desci_authority_delivery_acknowledgements (delivery_id, accepted_at)
    "#,
    r#"
    CREATE TABLE IF NOT EXISTS desci_authority_write_fencing_state (
        deployment_id TEXT PRIMARY KEY,
        generation BIGINT NOT NULL CHECK (generation > 0),
        lease_id UUID NOT NULL,
        lease_hash BYTEA NOT NULL CHECK (octet_length(lease_hash) = 32),
        signer_public_key BYTEA NOT NULL CHECK (octet_length(signer_public_key) = 32),
        phase TEXT NOT NULL CHECK (phase IN ('bootstrap', 'epoch')),
        epoch_number BIGINT CHECK (epoch_number IS NULL OR epoch_number > 0),
        epoch_hash BYTEA CHECK (epoch_hash IS NULL OR octet_length(epoch_hash) = 32),
        primary_id TEXT NOT NULL,
        database_system_identifier TEXT NOT NULL,
        postgres_timeline BIGINT NOT NULL CHECK (postgres_timeline > 0),
        expires_at TIMESTAMPTZ NOT NULL,
        last_scope TEXT NOT NULL,
        last_seen_at TIMESTAMPTZ NOT NULL,
        lease_json JSONB NOT NULL,
        CHECK (
            (phase = 'bootstrap' AND epoch_number IS NULL AND epoch_hash IS NULL)
            OR (phase = 'epoch' AND epoch_number IS NOT NULL AND epoch_hash IS NOT NULL)
        )
    )
    "#,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn postgres_configuration_fails_closed() {
        let mut config = PostgresAuthorityConfig::from_database_url("");
        assert!(config.validate().is_err());
        config.database_url = "postgres://localhost/mycelix".to_string();
        config.max_connections = 0;
        assert!(config.validate().is_err());
        config.max_connections = 8;
        config.acquire_timeout = Duration::ZERO;
        assert!(config.validate().is_err());
    }

    #[test]
    fn content_hash_requires_exact_width() {
        assert!(content_hash_from_bytes(vec![0; 31], "test").is_err());
        assert_eq!(
            content_hash_from_bytes(vec![7; 32], "test").unwrap(),
            ContentHash([7; 32])
        );
    }

    #[test]
    fn sequence_conversion_rejects_bigint_overflow() {
        assert!(sequence_to_i64(u64::MAX).is_err());
    }
}
