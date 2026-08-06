// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Transactional PostgreSQL persistence for credential and threshold governance.
//!
//! The credential registry and governance journal have separate projections,
//! but proposal execution is one authority transition. This module commits the
//! optional credential record, governance execution record, and signed outbox
//! publications in one serializable database transaction.

use crate::authority_fencing::AuthorityWriteScope;
use crate::postgres_authority::{PostgresAuthorityStore, lock_authority_epoch_shared};
use crate::scientific_credential_governance::RecordedCredentialGovernanceEvent;
use crate::scientific_credentials::RecordedScientificCredentialEvent;
use crate::scientific_events::ContentHash;
use crate::{Error, Result};
use chrono::{DateTime, Utc};
use sqlx::{Postgres, Row, Transaction};
use uuid::Uuid;

const CREDENTIAL_LOCK_KEY: i64 = 730_391_904_222;
const GOVERNANCE_LOCK_KEY: i64 = 730_391_904_223;

impl PostgresAuthorityStore {
    pub async fn credential_records(&self) -> Result<Vec<RecordedScientificCredentialEvent>> {
        let rows = sqlx::query(
            r#"
            SELECT sequence, event_id, event_hash, record_hash, previous_record_hash,
                   actor_id, idempotency_key, received_at, record_json
            FROM desci_credential_events
            ORDER BY sequence
            "#,
        )
        .fetch_all(self.pool())
        .await
        .map_err(sql_error)?;
        rows.into_iter().map(credential_record_from_row).collect()
    }

    pub async fn governance_records(&self) -> Result<Vec<RecordedCredentialGovernanceEvent>> {
        let rows = sqlx::query(
            r#"
            SELECT sequence, event_id, event_hash, record_hash, previous_record_hash,
                   actor_id, idempotency_key, received_at, record_json
            FROM desci_governance_events
            ORDER BY sequence
            "#,
        )
        .fetch_all(self.pool())
        .await
        .map_err(sql_error)?;
        rows.into_iter().map(governance_record_from_row).collect()
    }

    pub async fn append_credential_record(
        &self,
        expected_sequence: u64,
        record: &RecordedScientificCredentialEvent,
    ) -> Result<()> {
        let mut transaction = self.pool().begin().await.map_err(sql_error)?;
        begin_serializable(&mut transaction).await?;
        lock_authority_journals(&mut transaction).await?;
        self.assert_authority_write_fence(
            &mut transaction,
            AuthorityWriteScope::CredentialRegistry,
        )
        .await?;
        verify_credential_head(&mut transaction, expected_sequence, record).await?;
        insert_credential_record(&mut transaction, record).await?;
        insert_credential_outbox(self, &mut transaction, record).await?;
        transaction.commit().await.map_err(sql_error)
    }

    pub async fn append_governance_record(
        &self,
        expected_sequence: u64,
        record: &RecordedCredentialGovernanceEvent,
    ) -> Result<()> {
        let mut transaction = self.pool().begin().await.map_err(sql_error)?;
        begin_serializable(&mut transaction).await?;
        lock_authority_journals(&mut transaction).await?;
        self.assert_authority_write_fence(
            &mut transaction,
            AuthorityWriteScope::CredentialGovernance,
        )
        .await?;
        verify_governance_head(&mut transaction, expected_sequence, record).await?;
        insert_governance_record(&mut transaction, record).await?;
        insert_governance_outbox(self, &mut transaction, record).await?;
        transaction.commit().await.map_err(sql_error)
    }

    /// Import a fully validated file-backed credential and governance history
    /// into an empty PostgreSQL authority schema. Both journals and all signed
    /// publication envelopes commit in one serializable transaction.
    pub async fn import_credential_authority_history(
        &self,
        credential_records: &[RecordedScientificCredentialEvent],
        governance_records: &[RecordedCredentialGovernanceEvent],
    ) -> Result<()> {
        let mut transaction = self.pool().begin().await.map_err(sql_error)?;
        begin_serializable(&mut transaction).await?;
        lock_authority_journals(&mut transaction).await?;
        self.assert_authority_write_fence(
            &mut transaction,
            AuthorityWriteScope::CredentialRegistry,
        )
        .await?;
        self.assert_authority_write_fence(
            &mut transaction,
            AuthorityWriteScope::CredentialGovernance,
        )
        .await?;
        verify_credential_count(&mut transaction, 0).await?;
        verify_governance_count(&mut transaction, 0).await?;

        for (sequence, record) in credential_records.iter().enumerate() {
            let expected = u64::try_from(sequence).map_err(|_| {
                Error::Validation("credential import sequence exceeds u64".to_string())
            })?;
            verify_credential_head(&mut transaction, expected, record).await?;
            insert_credential_record(&mut transaction, record).await?;
            insert_credential_outbox(self, &mut transaction, record).await?;
        }
        for (sequence, record) in governance_records.iter().enumerate() {
            let expected = u64::try_from(sequence).map_err(|_| {
                Error::Validation("governance import sequence exceeds u64".to_string())
            })?;
            verify_governance_head(&mut transaction, expected, record).await?;
            insert_governance_record(&mut transaction, record).await?;
            insert_governance_outbox(self, &mut transaction, record).await?;
        }
        transaction.commit().await.map_err(sql_error)
    }

    pub async fn commit_governance_execution(
        &self,
        expected_credential_sequence: u64,
        credential_record: Option<&RecordedScientificCredentialEvent>,
        expected_governance_sequence: u64,
        governance_record: &RecordedCredentialGovernanceEvent,
    ) -> Result<()> {
        let mut transaction = self.pool().begin().await.map_err(sql_error)?;
        begin_serializable(&mut transaction).await?;
        lock_authority_journals(&mut transaction).await?;
        self.assert_authority_write_fence(
            &mut transaction,
            AuthorityWriteScope::CredentialGovernance,
        )
        .await?;
        if credential_record.is_some() {
            self.assert_authority_write_fence(
                &mut transaction,
                AuthorityWriteScope::CredentialRegistry,
            )
            .await?;
        }
        verify_governance_head(
            &mut transaction,
            expected_governance_sequence,
            governance_record,
        )
        .await?;
        if let Some(record) = credential_record {
            verify_credential_head(&mut transaction, expected_credential_sequence, record).await?;
            insert_credential_record(&mut transaction, record).await?;
            insert_credential_outbox(self, &mut transaction, record).await?;
        } else {
            verify_credential_count(&mut transaction, expected_credential_sequence).await?;
        }
        insert_governance_record(&mut transaction, governance_record).await?;
        insert_governance_outbox(self, &mut transaction, governance_record).await?;
        transaction.commit().await.map_err(sql_error)
    }
}

async fn begin_serializable(transaction: &mut Transaction<'_, Postgres>) -> Result<()> {
    sqlx::query("SET TRANSACTION ISOLATION LEVEL SERIALIZABLE")
        .execute(&mut **transaction)
        .await
        .map_err(sql_error)?;
    Ok(())
}

pub(crate) async fn lock_authority_journals(
    transaction: &mut Transaction<'_, Postgres>,
) -> Result<()> {
    lock_authority_epoch_shared(transaction).await?;
    sqlx::query("SELECT pg_advisory_xact_lock($1), pg_advisory_xact_lock($2)")
        .bind(CREDENTIAL_LOCK_KEY)
        .bind(GOVERNANCE_LOCK_KEY)
        .execute(&mut **transaction)
        .await
        .map_err(sql_error)?;
    Ok(())
}

async fn verify_credential_count(
    transaction: &mut Transaction<'_, Postgres>,
    expected_sequence: u64,
) -> Result<()> {
    let count: i64 = sqlx::query_scalar("SELECT COUNT(*) FROM desci_credential_events")
        .fetch_one(&mut **transaction)
        .await
        .map_err(sql_error)?;
    if count != sequence_to_i64(expected_sequence, "credential")? {
        return Err(Error::Storage(format!(
            "serializable credential conflict: durable count is {count}, caller expected {expected_sequence}"
        )));
    }
    Ok(())
}

async fn verify_governance_count(
    transaction: &mut Transaction<'_, Postgres>,
    expected_sequence: u64,
) -> Result<()> {
    let count: i64 = sqlx::query_scalar("SELECT COUNT(*) FROM desci_governance_events")
        .fetch_one(&mut **transaction)
        .await
        .map_err(sql_error)?;
    if count != sequence_to_i64(expected_sequence, "governance")? {
        return Err(Error::Storage(format!(
            "serializable governance conflict: durable count is {count}, caller expected {expected_sequence}"
        )));
    }
    Ok(())
}

async fn verify_credential_head(
    transaction: &mut Transaction<'_, Postgres>,
    expected_sequence: u64,
    record: &RecordedScientificCredentialEvent,
) -> Result<()> {
    record.event.verify()?;
    if record.event.envelope.sequence != expected_sequence {
        return Err(Error::Storage(
            "credential record sequence does not match expected sequence".to_string(),
        ));
    }
    let current = sqlx::query(
        "SELECT sequence, record_hash FROM desci_credential_events ORDER BY sequence DESC LIMIT 1 FOR UPDATE",
    )
    .fetch_optional(&mut **transaction)
    .await
    .map_err(sql_error)?;
    verify_head(
        current,
        expected_sequence,
        record.event.envelope.previous_hash,
        "credential",
    )
}

async fn verify_governance_head(
    transaction: &mut Transaction<'_, Postgres>,
    expected_sequence: u64,
    record: &RecordedCredentialGovernanceEvent,
) -> Result<()> {
    record.event.verify()?;
    if record.event.envelope.sequence != expected_sequence {
        return Err(Error::Storage(
            "governance record sequence does not match expected sequence".to_string(),
        ));
    }
    let current = sqlx::query(
        "SELECT sequence, record_hash FROM desci_governance_events ORDER BY sequence DESC LIMIT 1 FOR UPDATE",
    )
    .fetch_optional(&mut **transaction)
    .await
    .map_err(sql_error)?;
    verify_head(
        current,
        expected_sequence,
        record.event.envelope.previous_hash,
        "governance",
    )
}

fn verify_head(
    current: Option<sqlx::postgres::PgRow>,
    expected_sequence: u64,
    previous_hash: Option<ContentHash>,
    label: &str,
) -> Result<()> {
    match current {
        None if expected_sequence == 0 && previous_hash.is_none() => Ok(()),
        Some(row) => {
            let sequence: i64 = row.try_get("sequence").map_err(sql_error)?;
            let record_hash = content_hash(
                row.try_get("record_hash").map_err(sql_error)?,
                &format!("{label} record hash"),
            )?;
            if sequence.checked_add(1) != Some(sequence_to_i64(expected_sequence, label)?)
                || previous_hash != Some(record_hash)
            {
                return Err(Error::Storage(format!(
                    "serializable {label} conflict: durable head is {}, caller expected {expected_sequence}",
                    sequence.saturating_add(1)
                )));
            }
            Ok(())
        }
        None => Err(Error::Storage(format!(
            "empty {label} journal cannot accept sequence {expected_sequence}"
        ))),
    }
}

async fn insert_credential_record(
    transaction: &mut Transaction<'_, Postgres>,
    record: &RecordedScientificCredentialEvent,
) -> Result<()> {
    let event = &record.event;
    let sequence = sequence_to_i64(event.envelope.sequence, "credential")?;
    let event_hash = event.event_hash()?;
    let record_hash = record.record_hash()?;
    sqlx::query(
        r#"
        INSERT INTO desci_credential_events (
            sequence, event_id, event_hash, record_hash, previous_record_hash,
            actor_id, idempotency_key, received_at, record_json
        ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9)
        "#,
    )
    .bind(sequence)
    .bind(event.envelope.event_id.0)
    .bind(event_hash.0.to_vec())
    .bind(record_hash.0.to_vec())
    .bind(event.envelope.previous_hash.map(|hash| hash.0.to_vec()))
    .bind(event.envelope.actor.as_str())
    .bind(event.envelope.idempotency_key.as_deref())
    .bind(record.received_at.clone())
    .bind(serde_json::to_value(record)?)
    .execute(&mut **transaction)
    .await
    .map_err(sql_error)?;
    Ok(())
}

async fn insert_governance_record(
    transaction: &mut Transaction<'_, Postgres>,
    record: &RecordedCredentialGovernanceEvent,
) -> Result<()> {
    let event = &record.event;
    let sequence = sequence_to_i64(event.envelope.sequence, "governance")?;
    let event_hash = event.event_hash()?;
    let record_hash = record.record_hash()?;
    sqlx::query(
        r#"
        INSERT INTO desci_governance_events (
            sequence, event_id, event_hash, record_hash, previous_record_hash,
            actor_id, idempotency_key, received_at, record_json
        ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9)
        "#,
    )
    .bind(sequence)
    .bind(event.envelope.event_id.0)
    .bind(event_hash.0.to_vec())
    .bind(record_hash.0.to_vec())
    .bind(event.envelope.previous_hash.map(|hash| hash.0.to_vec()))
    .bind(event.envelope.actor.as_str())
    .bind(&event.envelope.idempotency_key)
    .bind(record.received_at.clone())
    .bind(serde_json::to_value(record)?)
    .execute(&mut **transaction)
    .await
    .map_err(sql_error)?;
    Ok(())
}

async fn insert_credential_outbox(
    store: &PostgresAuthorityStore,
    transaction: &mut Transaction<'_, Postgres>,
    record: &RecordedScientificCredentialEvent,
) -> Result<()> {
    let payload = serde_json::json!({
        "schema": "mycelix-desci.credential-recorded.v1",
        "record_hash": record.record_hash()?,
        "recorded_event": record,
    });
    insert_outbox(
        store,
        transaction,
        "credential.recorded.v1",
        "credential-registry",
        sequence_to_i64(record.event.envelope.sequence, "credential")?,
        record.received_at.clone(),
        payload,
    )
    .await
}

async fn insert_governance_outbox(
    store: &PostgresAuthorityStore,
    transaction: &mut Transaction<'_, Postgres>,
    record: &RecordedCredentialGovernanceEvent,
) -> Result<()> {
    let payload = serde_json::json!({
        "schema": "mycelix-desci.credential-governance-recorded.v1",
        "record_hash": record.record_hash()?,
        "recorded_event": record,
    });
    insert_outbox(
        store,
        transaction,
        "credential.governance.recorded.v1",
        "credential-governance",
        sequence_to_i64(record.event.envelope.sequence, "governance")?,
        record.received_at.clone(),
        payload,
    )
    .await
}

async fn insert_outbox(
    store: &PostgresAuthorityStore,
    transaction: &mut Transaction<'_, Postgres>,
    topic: &str,
    aggregate_id: &str,
    aggregate_sequence: i64,
    created_at: DateTime<Utc>,
    payload: serde_json::Value,
) -> Result<()> {
    let delivery_id = Uuid::new_v4();
    let (delivery_json, delivery_hash) = store.sign_delivery(
        delivery_id,
        topic,
        aggregate_id,
        aggregate_sequence,
        created_at,
        payload,
    )?;
    sqlx::query(
        r#"
        INSERT INTO desci_authority_outbox (
            id, topic, aggregate_id, aggregate_sequence,
            payload, payload_hash, created_at
        ) VALUES ($1, $2, $3, $4, $5, $6, $7)
        "#,
    )
    .bind(delivery_id)
    .bind(topic)
    .bind(aggregate_id)
    .bind(aggregate_sequence)
    .bind(delivery_json)
    .bind(delivery_hash.0.to_vec())
    .bind(created_at)
    .execute(&mut **transaction)
    .await
    .map_err(sql_error)?;
    Ok(())
}

fn credential_record_from_row(
    row: sqlx::postgres::PgRow,
) -> Result<RecordedScientificCredentialEvent> {
    let value: serde_json::Value = row.try_get("record_json").map_err(sql_error)?;
    let record: RecordedScientificCredentialEvent = serde_json::from_value(value)?;
    record.event.verify()?;
    verify_indexed_record(
        &row,
        record.event.envelope.sequence,
        record.event.envelope.event_id.0,
        record.event.event_hash()?,
        record.record_hash()?,
        record.event.envelope.previous_hash,
        record.event.envelope.actor.as_str(),
        record.event.envelope.idempotency_key.as_deref(),
        record.received_at.clone(),
        "credential",
    )?;
    Ok(record)
}

fn governance_record_from_row(
    row: sqlx::postgres::PgRow,
) -> Result<RecordedCredentialGovernanceEvent> {
    let value: serde_json::Value = row.try_get("record_json").map_err(sql_error)?;
    let record: RecordedCredentialGovernanceEvent = serde_json::from_value(value)?;
    record.event.verify()?;
    verify_indexed_record(
        &row,
        record.event.envelope.sequence,
        record.event.envelope.event_id.0,
        record.event.event_hash()?,
        record.record_hash()?,
        record.event.envelope.previous_hash,
        record.event.envelope.actor.as_str(),
        Some(&record.event.envelope.idempotency_key),
        record.received_at.clone(),
        "governance",
    )?;
    Ok(record)
}

#[allow(clippy::too_many_arguments)]
fn verify_indexed_record(
    row: &sqlx::postgres::PgRow,
    sequence: u64,
    event_id: Uuid,
    event_hash: ContentHash,
    record_hash: ContentHash,
    previous_record_hash: Option<ContentHash>,
    actor_id: &str,
    idempotency_key: Option<&str>,
    received_at: DateTime<Utc>,
    label: &str,
) -> Result<()> {
    let indexed_sequence: i64 = row.try_get("sequence").map_err(sql_error)?;
    let indexed_event_id: Uuid = row.try_get("event_id").map_err(sql_error)?;
    let indexed_event_hash = content_hash(
        row.try_get("event_hash").map_err(sql_error)?,
        &format!("{label} event hash"),
    )?;
    let indexed_record_hash = content_hash(
        row.try_get("record_hash").map_err(sql_error)?,
        &format!("{label} record hash"),
    )?;
    let indexed_previous = row
        .try_get::<Option<Vec<u8>>, _>("previous_record_hash")
        .map_err(sql_error)?
        .map(|bytes| content_hash(bytes, &format!("{label} previous record hash")))
        .transpose()?;
    let indexed_actor: String = row.try_get("actor_id").map_err(sql_error)?;
    let indexed_idempotency: Option<String> = row.try_get("idempotency_key").map_err(sql_error)?;
    let indexed_received_at: DateTime<Utc> = row.try_get("received_at").map_err(sql_error)?;
    if indexed_sequence != sequence_to_i64(sequence, label)?
        || indexed_event_id != event_id
        || indexed_event_hash != event_hash
        || indexed_record_hash != record_hash
        || indexed_previous != previous_record_hash
        || indexed_actor != actor_id
        || indexed_idempotency.as_deref() != idempotency_key
        || indexed_received_at != received_at
    {
        return Err(Error::VerificationFailed(format!(
            "PostgreSQL {label} record JSON does not match its indexed columns"
        )));
    }
    Ok(())
}

fn content_hash(bytes: Vec<u8>, label: &str) -> Result<ContentHash> {
    let bytes: [u8; 32] = bytes
        .try_into()
        .map_err(|_| Error::Storage(format!("PostgreSQL {label} must contain exactly 32 bytes")))?;
    Ok(ContentHash(bytes))
}

fn sequence_to_i64(sequence: u64, label: &str) -> Result<i64> {
    i64::try_from(sequence)
        .map_err(|_| Error::Validation(format!("{label} sequence exceeds PostgreSQL BIGINT")))
}

fn sql_error(error: impl std::fmt::Display) -> Error {
    Error::Storage(format!("PostgreSQL credential authority backend: {error}"))
}
