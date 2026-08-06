// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! PostgreSQL persistence for governed database epochs and recovery evidence.

use crate::authority_delivery::SignedAuthorityDeliveryEnvelope;
use crate::authority_epoch::{
    AuthorityDatabaseStateCommitment, DatabaseEpochPromotionIntent,
    SignedAuthorityDatabaseEpochCertificate, SignedAuthorityDeliveryAcknowledgement,
    SignedAuthorityRecoveryReconciliation,
};
use crate::authority_fencing::AuthorityWriteScope;
use crate::postgres_authority::{
    AUTHORITY_SCHEMA_VERSION, PostgresAuthorityStore, lock_authority_epoch_exclusive,
};
use crate::postgres_credential_authority::lock_authority_journals;
use crate::scientific_credential_governance::{
    CredentialGovernanceAction, CredentialGovernancePayload, CredentialGovernanceProjection,
    RecordedCredentialGovernanceEvent,
};
use crate::scientific_credentials::{
    RecordedScientificCredentialEvent, ScientificCredentialRegistryProjection,
};
use crate::scientific_events::ContentHash;
use crate::{Error, Result};
use chrono::{DateTime, Duration, Utc};
use serde::{Deserialize, Serialize};
use sqlx::{Postgres, Row, Transaction};
use std::collections::BTreeSet;
use uuid::Uuid;

const MAX_EPOCH_FUTURE_SKEW_SECONDS: i64 = 300;

const DATABASE_EPOCH_SELECT_COLUMNS: &str = r#"
    SELECT epoch_number, epoch_id, epoch_hash, previous_epoch_hash,
           deployment_id, primary_id, promoted_at, accepted_at,
           state_commitment_hash, publication_delivery_id, certificate_json
    FROM desci_database_epochs
"#;
const DATABASE_EPOCH_SELECT_BY_ID_FOR_UPDATE: &str = r#"
    SELECT epoch_number, epoch_id, epoch_hash, previous_epoch_hash,
           deployment_id, primary_id, promoted_at, accepted_at,
           state_commitment_hash, publication_delivery_id, certificate_json
    FROM desci_database_epochs
    WHERE epoch_id = $1
    FOR UPDATE
"#;
const RECOVERY_RECONCILIATION_SELECT_COLUMNS: &str = r#"
    SELECT reconciliation_id, epoch_hash, reconciliation_hash,
           verified_at, accepted_at, publication_delivery_id, reconciliation_json
    FROM desci_recovery_reconciliations
"#;
const RECOVERY_RECONCILIATION_SELECT_BY_ID_FOR_UPDATE: &str = r#"
    SELECT reconciliation_id, epoch_hash, reconciliation_hash,
           verified_at, accepted_at, publication_delivery_id, reconciliation_json
    FROM desci_recovery_reconciliations
    WHERE reconciliation_id = $1
    FOR UPDATE
"#;
const DELIVERY_ACK_SELECT_COLUMNS: &str = r#"
    SELECT acknowledgement_id, delivery_id, delivery_hash,
           witness_actor, witness_organization, acknowledged_at,
           accepted_at, acknowledgement_hash, acknowledgement_json
    FROM desci_authority_delivery_acknowledgements
"#;
const DELIVERY_ACK_SELECT_BY_ID_FOR_UPDATE: &str = r#"
    SELECT acknowledgement_id, delivery_id, delivery_hash,
           witness_actor, witness_organization, acknowledged_at,
           accepted_at, acknowledgement_hash, acknowledgement_json
    FROM desci_authority_delivery_acknowledgements
    WHERE acknowledgement_id = $1
    FOR UPDATE
"#;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityDatabaseEpochRecord {
    pub certificate: SignedAuthorityDatabaseEpochCertificate,
    pub epoch_hash: ContentHash,
    pub publication_delivery_id: Uuid,
    pub accepted_at: DateTime<Utc>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityRecoveryReconciliationRecord {
    pub reconciliation: SignedAuthorityRecoveryReconciliation,
    pub reconciliation_hash: ContentHash,
    pub publication_delivery_id: Uuid,
    pub accepted_at: DateTime<Utc>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityDeliveryAcknowledgementRecord {
    pub acknowledgement: SignedAuthorityDeliveryAcknowledgement,
    pub acknowledgement_hash: ContentHash,
    pub accepted_at: DateTime<Utc>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityDatabaseEpochSummary {
    pub epoch_count: u64,
    pub latest_epoch_number: Option<u64>,
    pub latest_epoch_hash: Option<ContentHash>,
    pub latest_primary_id: Option<String>,
    pub latest_promotion_mode: Option<crate::authority_epoch::DatabasePromotionMode>,
    pub latest_epoch_reconciled: bool,
    pub recovery_reconciliation_count: u64,
    pub latest_published_delivery_id: Option<Uuid>,
    pub latest_delivery_acknowledgement_organizations: usize,
}

struct PersistedAuthorityDelivery {
    signed: SignedAuthorityDeliveryEnvelope,
}

impl PostgresAuthorityStore {
    /// Compute one stable authority-state commitment while all cooperating
    /// authority writers are excluded by the database epoch barrier.
    pub async fn authority_state_commitment(
        &self,
        anchor_checkpoint_hash: ContentHash,
    ) -> Result<AuthorityDatabaseStateCommitment> {
        let mut transaction = self.pool().begin().await.map_err(sql_error)?;
        begin_serializable(&mut transaction).await?;
        lock_authority_epoch_exclusive(&mut transaction).await?;
        let commitment =
            authority_state_commitment_in_transaction(&mut transaction, anchor_checkpoint_hash)
                .await?;
        transaction.commit().await.map_err(sql_error)?;
        Ok(commitment)
    }

    pub async fn record_database_epoch(
        &self,
        certificate: SignedAuthorityDatabaseEpochCertificate,
        _registry: &ScientificCredentialRegistryProjection,
        governance: &CredentialGovernanceProjection,
    ) -> Result<AuthorityDatabaseEpochRecord> {
        certificate.verify()?;
        let epoch_hash = certificate.epoch_hash()?;
        let mut transaction = self.pool().begin().await.map_err(sql_error)?;
        begin_serializable(&mut transaction).await?;
        lock_authority_epoch_exclusive(&mut transaction).await?;

        if let Some(row) = sqlx::query(DATABASE_EPOCH_SELECT_BY_ID_FOR_UPDATE)
            .bind(certificate.certificate.intent.promotion_id)
            .fetch_optional(&mut *transaction)
            .await
            .map_err(sql_error)?
        {
            let existing = database_epoch_from_row(row)?;
            if existing.certificate == certificate {
                let epoch_number = existing.certificate.certificate.intent.epoch_number;
                transaction.commit().await.map_err(sql_error)?;
                return self.database_epoch(epoch_number).await?.ok_or_else(|| {
                    Error::Storage(
                        "database epoch disappeared after exact retry commit".to_string(),
                    )
                });
            }
            return Err(Error::Storage(
                "database epoch promotion id is already bound to different signed bytes"
                    .to_string(),
            ));
        }

        self.assert_authority_write_fence_for_epoch(
            &mut transaction,
            AuthorityWriteScope::DatabaseEpochPromotion,
            &certificate,
            epoch_hash,
        )
        .await?;
        certificate.verify_governance_authorization(governance)?;
        let request_time = Utc::now();
        if certificate.certificate.promoted_at
            > request_time + Duration::seconds(MAX_EPOCH_FUTURE_SKEW_SECONDS)
        {
            return Err(Error::VerificationFailed(
                "database epoch promotion time exceeds allowed future clock skew".to_string(),
            ));
        }
        let proposal = governance
            .proposals
            .get(&certificate.certificate.governance_proposal_id)
            .ok_or_else(|| {
                Error::NotFound("database epoch governance proposal not found".to_string())
            })?;
        if proposal
            .executed_at
            .as_ref()
            .is_none_or(|executed_at| &certificate.certificate.promoted_at < executed_at)
        {
            return Err(Error::VerificationFailed(
                "database epoch promotion predates governance execution".to_string(),
            ));
        }

        let durable_registry = durable_credential_projection_in_transaction(
            &mut transaction,
            self.trusted_receipt_keys(),
        )
        .await?;
        let durable_governance_keys =
            durable_governance_service_keys_in_transaction(&mut transaction).await?;
        if durable_registry.owns_signing_key(&certificate.signer_public_key)
            || self
                .trusted_receipt_keys()
                .contains(&certificate.signer_public_key)
            || self.delivery_public_key() == certificate.signer_public_key
            || durable_governance_keys.contains(&certificate.signer_public_key)
        {
            return Err(Error::VerificationFailed(
                "database epoch signer overlaps a durable actor or service authority key"
                    .to_string(),
            ));
        }

        verify_durable_database_epoch_authorization(
            &mut transaction,
            certificate.certificate.governance_proposal_id.0,
            &certificate.certificate.intent,
            certificate
                .certificate
                .intent
                .anchor_checkpoint
                .checkpoint_hash(),
            certificate.certificate.governance_execution_record_hash,
        )
        .await?;

        let durable_state = authority_state_commitment_in_transaction(
            &mut transaction,
            certificate
                .certificate
                .intent
                .anchor_checkpoint
                .checkpoint_hash(),
        )
        .await?;
        if durable_state != certificate.certificate.state_commitment {
            return Err(Error::Storage(
                "database authority state changed before epoch commit".to_string(),
            ));
        }

        let previous = sqlx::query(
            r#"
            SELECT epoch_number, epoch_hash, certificate_json
            FROM desci_database_epochs
            ORDER BY epoch_number DESC
            LIMIT 1
            FOR UPDATE
            "#,
        )
        .fetch_optional(&mut *transaction)
        .await
        .map_err(sql_error)?;
        match previous {
            None => {
                if certificate.certificate.intent.epoch_number != 1
                    || certificate
                        .certificate
                        .intent
                        .expected_previous_epoch_hash
                        .is_some()
                {
                    return Err(Error::Storage(
                        "empty database epoch journal accepts only epoch one".to_string(),
                    ));
                }
            }
            Some(row) => {
                let previous_number = i64_to_u64(
                    row.try_get("epoch_number").map_err(sql_error)?,
                    "database epoch number",
                )?;
                let previous_hash = content_hash(
                    row.try_get("epoch_hash").map_err(sql_error)?,
                    "database epoch hash",
                )?;
                let previous_json: serde_json::Value =
                    row.try_get("certificate_json").map_err(sql_error)?;
                let previous_certificate: SignedAuthorityDatabaseEpochCertificate =
                    serde_json::from_value(previous_json)?;
                previous_certificate.verify()?;
                if certificate.certificate.intent.epoch_number != previous_number + 1
                    || certificate.certificate.intent.expected_previous_epoch_hash
                        != Some(previous_hash)
                    || certificate.certificate.intent.deployment_id
                        != previous_certificate.certificate.intent.deployment_id
                {
                    return Err(Error::Storage(
                        "database epoch certificate does not extend the durable epoch chain"
                            .to_string(),
                    ));
                }
            }
        }

        let accepted_at = Utc::now();
        let delivery_id = Uuid::new_v4();
        let payload = serde_json::json!({
            "schema": "mycelix-desci.database-epoch-recorded.v1",
            "epoch_hash": epoch_hash,
            "certificate": &certificate,
        });
        let epoch_sequence = u64_to_i64(
            certificate.certificate.intent.epoch_number,
            "database epoch number",
        )?;
        let (delivery, delivery_hash) = self.sign_delivery(
            delivery_id,
            "authority.database-epoch.recorded.v1",
            &certificate.certificate.intent.deployment_id,
            epoch_sequence,
            accepted_at,
            payload,
        )?;
        insert_outbox(
            &mut transaction,
            delivery_id,
            "authority.database-epoch.recorded.v1",
            &certificate.certificate.intent.deployment_id,
            epoch_sequence,
            delivery,
            delivery_hash,
            accepted_at,
        )
        .await?;

        sqlx::query(
            r#"
            INSERT INTO desci_database_epochs (
                epoch_number, epoch_id, epoch_hash, previous_epoch_hash,
                deployment_id, primary_id, promoted_at, accepted_at,
                state_commitment_hash, publication_delivery_id, certificate_json
            ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11)
            "#,
        )
        .bind(epoch_sequence)
        .bind(certificate.certificate.intent.promotion_id)
        .bind(epoch_hash.0.to_vec())
        .bind(
            certificate
                .certificate
                .intent
                .expected_previous_epoch_hash
                .map(|hash| hash.0.to_vec()),
        )
        .bind(&certificate.certificate.intent.deployment_id)
        .bind(&certificate.certificate.intent.candidate_primary_id)
        .bind(certificate.certificate.promoted_at)
        .bind(accepted_at)
        .bind(
            certificate
                .certificate
                .state_commitment
                .commitment_hash()?
                .0
                .to_vec(),
        )
        .bind(delivery_id)
        .bind(serde_json::to_value(&certificate)?)
        .execute(&mut *transaction)
        .await
        .map_err(sql_error)?;

        transaction.commit().await.map_err(sql_error)?;
        Ok(AuthorityDatabaseEpochRecord {
            certificate,
            epoch_hash,
            publication_delivery_id: delivery_id,
            accepted_at,
        })
    }

    pub async fn record_recovery_reconciliation(
        &self,
        reconciliation: SignedAuthorityRecoveryReconciliation,
    ) -> Result<AuthorityRecoveryReconciliationRecord> {
        reconciliation.verify()?;
        let request_time = Utc::now();
        if reconciliation.reconciliation.verified_at
            > request_time + Duration::seconds(MAX_EPOCH_FUTURE_SKEW_SECONDS)
        {
            return Err(Error::VerificationFailed(
                "recovery reconciliation time exceeds allowed future clock skew".to_string(),
            ));
        }
        let reconciliation_hash = reconciliation.reconciliation_hash()?;
        let mut transaction = self.pool().begin().await.map_err(sql_error)?;
        begin_serializable(&mut transaction).await?;
        lock_authority_epoch_exclusive(&mut transaction).await?;

        if let Some(row) = sqlx::query(RECOVERY_RECONCILIATION_SELECT_BY_ID_FOR_UPDATE)
            .bind(reconciliation.reconciliation.reconciliation_id)
            .fetch_optional(&mut *transaction)
            .await
            .map_err(sql_error)?
        {
            let existing = recovery_reconciliation_from_row(row)?;
            if existing.reconciliation == reconciliation {
                let epoch_hash = existing.reconciliation.reconciliation.epoch_hash;
                let reconciliation_id = existing.reconciliation.reconciliation.reconciliation_id;
                transaction.commit().await.map_err(sql_error)?;
                return self
                    .recovery_reconciliations(epoch_hash)
                    .await?
                    .into_iter()
                    .find(|record| {
                        record.reconciliation.reconciliation.reconciliation_id == reconciliation_id
                    })
                    .ok_or_else(|| {
                        Error::Storage(
                            "recovery reconciliation disappeared after exact retry commit"
                                .to_string(),
                        )
                    });
            }
            return Err(Error::Storage(
                "recovery reconciliation id is already bound to different signed bytes".to_string(),
            ));
        }

        self.assert_authority_write_fence(
            &mut transaction,
            AuthorityWriteScope::RecoveryReconciliation,
        )
        .await?;
        let already_reconciled: bool = sqlx::query_scalar(
            "SELECT EXISTS (SELECT 1 FROM desci_recovery_reconciliations WHERE epoch_hash = $1)",
        )
        .bind(reconciliation.reconciliation.epoch_hash.0.to_vec())
        .fetch_one(&mut *transaction)
        .await
        .map_err(sql_error)?;
        if already_reconciled {
            return Err(Error::Storage(
                "database epoch already has a signed recovery reconciliation".to_string(),
            ));
        }

        let row = sqlx::query(
            "SELECT epoch_number, certificate_json FROM desci_database_epochs WHERE epoch_hash = $1 FOR UPDATE",
        )
        .bind(reconciliation.reconciliation.epoch_hash.0.to_vec())
        .fetch_optional(&mut *transaction)
        .await
        .map_err(sql_error)?
        .ok_or_else(|| Error::NotFound("database epoch certificate not found".to_string()))?;
        let indexed_epoch_number: i64 = row.try_get("epoch_number").map_err(sql_error)?;
        let latest_epoch_number: Option<i64> =
            sqlx::query_scalar("SELECT MAX(epoch_number) FROM desci_database_epochs")
                .fetch_one(&mut *transaction)
                .await
                .map_err(sql_error)?;
        if latest_epoch_number != Some(indexed_epoch_number) {
            return Err(Error::VerificationFailed(
                "recovery reconciliation must target the current database epoch".to_string(),
            ));
        }
        let value: serde_json::Value = row.try_get("certificate_json").map_err(sql_error)?;
        let epoch: SignedAuthorityDatabaseEpochCertificate = serde_json::from_value(value)?;
        reconciliation.verify_against_epoch(&epoch)?;
        let durable_state = authority_state_commitment_in_transaction(
            &mut transaction,
            epoch.certificate.state_commitment.anchor_checkpoint_hash,
        )
        .await?;
        if durable_state != reconciliation.reconciliation.observed_state {
            return Err(Error::Storage(
                "recovered database does not match the signed epoch commitment".to_string(),
            ));
        }

        let accepted_at = Utc::now();
        let delivery_id = Uuid::new_v4();
        let payload = serde_json::json!({
            "schema": "mycelix-desci.recovery-reconciliation-recorded.v1",
            "reconciliation_hash": reconciliation_hash,
            "reconciliation": &reconciliation,
        });
        let aggregate_id = reconciliation.reconciliation.epoch_hash.to_string();
        let aggregate_sequence = u64_to_i64(
            reconciliation.reconciliation.epoch_number,
            "recovery epoch number",
        )?;
        let (delivery, delivery_hash) = self.sign_delivery(
            delivery_id,
            "authority.recovery-reconciliation.recorded.v1",
            &aggregate_id,
            aggregate_sequence,
            accepted_at,
            payload,
        )?;
        insert_outbox(
            &mut transaction,
            delivery_id,
            "authority.recovery-reconciliation.recorded.v1",
            &aggregate_id,
            aggregate_sequence,
            delivery,
            delivery_hash,
            accepted_at,
        )
        .await?;

        sqlx::query(
            r#"
            INSERT INTO desci_recovery_reconciliations (
                reconciliation_id, epoch_hash, reconciliation_hash,
                verified_at, accepted_at, publication_delivery_id, reconciliation_json
            ) VALUES ($1, $2, $3, $4, $5, $6, $7)
            "#,
        )
        .bind(reconciliation.reconciliation.reconciliation_id)
        .bind(reconciliation.reconciliation.epoch_hash.0.to_vec())
        .bind(reconciliation_hash.0.to_vec())
        .bind(reconciliation.reconciliation.verified_at)
        .bind(accepted_at)
        .bind(delivery_id)
        .bind(serde_json::to_value(&reconciliation)?)
        .execute(&mut *transaction)
        .await
        .map_err(sql_error)?;

        transaction.commit().await.map_err(sql_error)?;
        Ok(AuthorityRecoveryReconciliationRecord {
            reconciliation,
            reconciliation_hash,
            publication_delivery_id: delivery_id,
            accepted_at,
        })
    }

    pub async fn record_delivery_acknowledgement(
        &self,
        acknowledgement: SignedAuthorityDeliveryAcknowledgement,
        governance: &CredentialGovernanceProjection,
    ) -> Result<AuthorityDeliveryAcknowledgementRecord> {
        acknowledgement.verify()?;
        let request_time = Utc::now();
        if acknowledgement.acknowledgement.acknowledged_at
            > request_time + Duration::seconds(MAX_EPOCH_FUTURE_SKEW_SECONDS)
        {
            return Err(Error::VerificationFailed(
                "delivery acknowledgement exceeds allowed future clock skew".to_string(),
            ));
        }
        let acknowledgement_hash = acknowledgement.acknowledgement_hash()?;
        let mut transaction = self.pool().begin().await.map_err(sql_error)?;
        begin_serializable(&mut transaction).await?;
        lock_authority_journals(&mut transaction).await?;
        if let Some(row) = sqlx::query(DELIVERY_ACK_SELECT_BY_ID_FOR_UPDATE)
            .bind(acknowledgement.acknowledgement.acknowledgement_id)
            .fetch_optional(&mut *transaction)
            .await
            .map_err(sql_error)?
        {
            let existing = delivery_acknowledgement_from_row(row)?;
            if existing.acknowledgement == acknowledgement {
                transaction.commit().await.map_err(sql_error)?;
                return Ok(existing);
            }
            return Err(Error::Storage(
                "delivery acknowledgement id is already bound to different signed bytes"
                    .to_string(),
            ));
        }
        self.assert_authority_write_fence(
            &mut transaction,
            AuthorityWriteScope::DeliveryAcknowledgement,
        )
        .await?;
        verify_governance_projection_head(&mut transaction, governance.head_hash).await?;
        acknowledgement.verify_against_governance(governance)?;
        let row = sqlx::query(
            r#"
            SELECT id, topic, aggregate_id, aggregate_sequence, payload,
                   payload_hash, created_at, published_at
            FROM desci_authority_outbox
            WHERE id = $1
            FOR UPDATE
            "#,
        )
        .bind(acknowledgement.acknowledgement.delivery_id)
        .fetch_optional(&mut *transaction)
        .await
        .map_err(sql_error)?
        .ok_or_else(|| Error::NotFound("authority delivery not found".to_string()))?;
        let signed_delivery: SignedAuthorityDeliveryEnvelope =
            serde_json::from_value(row.try_get("payload").map_err(sql_error)?)?;
        signed_delivery.verify()?;
        let topic: String = row.try_get("topic").map_err(sql_error)?;
        let delivery_hash = content_hash(
            row.try_get("payload_hash").map_err(sql_error)?,
            "authority delivery hash",
        )?;
        let indexed_id: Uuid = row.try_get("id").map_err(sql_error)?;
        let indexed_aggregate_id: String = row.try_get("aggregate_id").map_err(sql_error)?;
        let indexed_sequence: i64 = row.try_get("aggregate_sequence").map_err(sql_error)?;
        let indexed_created_at: DateTime<Utc> = row.try_get("created_at").map_err(sql_error)?;
        let published_at: Option<DateTime<Utc>> = row.try_get("published_at").map_err(sql_error)?;
        if indexed_id != signed_delivery.envelope.delivery_id
            || topic != signed_delivery.envelope.topic
            || indexed_aggregate_id != signed_delivery.envelope.aggregate_id
            || indexed_sequence != signed_delivery.envelope.aggregate_sequence
            || indexed_created_at != signed_delivery.envelope.created_at
            || delivery_hash != signed_delivery.delivery_hash()?
            || signed_delivery.signer_public_key != self.delivery_public_key()
            || topic != acknowledgement.acknowledgement.topic
            || delivery_hash != acknowledgement.acknowledgement.delivery_hash
            || published_at.is_none()
            || published_at.is_some_and(|published_at| {
                acknowledgement.acknowledgement.acknowledged_at < published_at
            })
        {
            return Err(Error::VerificationFailed(
                "delivery acknowledgement does not bind a published authority delivery".to_string(),
            ));
        }
        let accepted_at = Utc::now();
        sqlx::query(
            r#"
            INSERT INTO desci_authority_delivery_acknowledgements (
                acknowledgement_id, delivery_id, delivery_hash, witness_actor,
                witness_organization, acknowledged_at, accepted_at,
                acknowledgement_hash, acknowledgement_json
            ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9)
            "#,
        )
        .bind(acknowledgement.acknowledgement.acknowledgement_id)
        .bind(acknowledgement.acknowledgement.delivery_id)
        .bind(acknowledgement.acknowledgement.delivery_hash.0.to_vec())
        .bind(acknowledgement.acknowledgement.actor.as_str())
        .bind(acknowledgement.acknowledgement.organization.as_str())
        .bind(acknowledgement.acknowledgement.acknowledged_at)
        .bind(accepted_at)
        .bind(acknowledgement_hash.0.to_vec())
        .bind(serde_json::to_value(&acknowledgement)?)
        .execute(&mut *transaction)
        .await
        .map_err(sql_error)?;
        transaction.commit().await.map_err(sql_error)?;
        Ok(AuthorityDeliveryAcknowledgementRecord {
            acknowledgement,
            acknowledgement_hash,
            accepted_at,
        })
    }

    async fn verified_authority_delivery(
        &self,
        delivery_id: Uuid,
    ) -> Result<PersistedAuthorityDelivery> {
        let row = sqlx::query(
            r#"
            SELECT id, topic, aggregate_id, aggregate_sequence, payload,
                   payload_hash, created_at, published_at
            FROM desci_authority_outbox
            WHERE id = $1
            "#,
        )
        .bind(delivery_id)
        .fetch_optional(self.pool())
        .await
        .map_err(sql_error)?
        .ok_or_else(|| {
            Error::VerificationFailed(
                "database authority record references a missing publication delivery".to_string(),
            )
        })?;
        let signed: SignedAuthorityDeliveryEnvelope =
            serde_json::from_value(row.try_get("payload").map_err(sql_error)?)?;
        signed.verify()?;
        let delivery_hash = content_hash(
            row.try_get("payload_hash").map_err(sql_error)?,
            "authority delivery hash",
        )?;
        let indexed_id: Uuid = row.try_get("id").map_err(sql_error)?;
        let indexed_topic: String = row.try_get("topic").map_err(sql_error)?;
        let indexed_aggregate_id: String = row.try_get("aggregate_id").map_err(sql_error)?;
        let indexed_sequence: i64 = row.try_get("aggregate_sequence").map_err(sql_error)?;
        let indexed_created_at: DateTime<Utc> = row.try_get("created_at").map_err(sql_error)?;
        if indexed_id != signed.envelope.delivery_id
            || indexed_topic != signed.envelope.topic
            || indexed_aggregate_id != signed.envelope.aggregate_id
            || indexed_sequence != signed.envelope.aggregate_sequence
            || indexed_created_at != signed.envelope.created_at
            || delivery_hash != signed.delivery_hash()?
            || signed.signer_public_key != self.delivery_public_key()
        {
            return Err(Error::VerificationFailed(
                "indexed authority delivery metadata disagrees with its signed envelope"
                    .to_string(),
            ));
        }
        let _: Option<DateTime<Utc>> = row.try_get("published_at").map_err(sql_error)?;
        Ok(PersistedAuthorityDelivery { signed })
    }

    async fn verify_epoch_publication(&self, record: &AuthorityDatabaseEpochRecord) -> Result<()> {
        let delivery = self
            .verified_authority_delivery(record.publication_delivery_id)
            .await?;
        let expected_payload = serde_json::json!({
            "schema": "mycelix-desci.database-epoch-recorded.v1",
            "epoch_hash": record.epoch_hash,
            "certificate": &record.certificate,
        });
        let expected_sequence = u64_to_i64(
            record.certificate.certificate.intent.epoch_number,
            "database epoch number",
        )?;
        if delivery.signed.envelope.topic != "authority.database-epoch.recorded.v1"
            || delivery.signed.envelope.aggregate_id
                != record.certificate.certificate.intent.deployment_id
            || delivery.signed.envelope.aggregate_sequence != expected_sequence
            || delivery.signed.envelope.created_at != record.accepted_at
            || delivery.signed.envelope.payload != expected_payload
        {
            return Err(Error::VerificationFailed(
                "database epoch publication does not bind the persisted certificate".to_string(),
            ));
        }
        Ok(())
    }

    async fn verify_reconciliation_publication(
        &self,
        record: &AuthorityRecoveryReconciliationRecord,
    ) -> Result<()> {
        let delivery = self
            .verified_authority_delivery(record.publication_delivery_id)
            .await?;
        let expected_payload = serde_json::json!({
            "schema": "mycelix-desci.recovery-reconciliation-recorded.v1",
            "reconciliation_hash": record.reconciliation_hash,
            "reconciliation": &record.reconciliation,
        });
        let expected_sequence = u64_to_i64(
            record.reconciliation.reconciliation.epoch_number,
            "recovery epoch number",
        )?;
        if delivery.signed.envelope.topic != "authority.recovery-reconciliation.recorded.v1"
            || delivery.signed.envelope.aggregate_id
                != record.reconciliation.reconciliation.epoch_hash.to_string()
            || delivery.signed.envelope.aggregate_sequence != expected_sequence
            || delivery.signed.envelope.created_at != record.accepted_at
            || delivery.signed.envelope.payload != expected_payload
        {
            return Err(Error::VerificationFailed(
                "recovery publication does not bind the persisted reconciliation".to_string(),
            ));
        }
        Ok(())
    }

    pub async fn database_epoch(
        &self,
        epoch_number: u64,
    ) -> Result<Option<AuthorityDatabaseEpochRecord>> {
        let query = format!("{DATABASE_EPOCH_SELECT_COLUMNS} WHERE epoch_number = $1");
        let row = sqlx::query(&query)
            .bind(u64_to_i64(epoch_number, "database epoch number")?)
            .fetch_optional(self.pool())
            .await
            .map_err(sql_error)?;
        let Some(row) = row else {
            return Ok(None);
        };
        let record = database_epoch_from_row(row)?;
        self.verify_epoch_publication(&record).await?;
        Ok(Some(record))
    }

    /// Replay and verify the complete database-epoch chain, including the
    /// signed publication delivery linked to every certificate.
    pub async fn database_epochs(&self) -> Result<Vec<AuthorityDatabaseEpochRecord>> {
        let query = format!("{DATABASE_EPOCH_SELECT_COLUMNS} ORDER BY epoch_number");
        let rows = sqlx::query(&query)
            .fetch_all(self.pool())
            .await
            .map_err(sql_error)?;
        let mut records: Vec<AuthorityDatabaseEpochRecord> = Vec::with_capacity(rows.len());
        for row in rows {
            let record = database_epoch_from_row(row)?;
            self.verify_epoch_publication(&record).await?;
            if let Some(previous) = records.last() {
                let expected_number = previous
                    .certificate
                    .certificate
                    .intent
                    .epoch_number
                    .checked_add(1)
                    .ok_or_else(|| Error::Storage("database epoch number overflow".to_string()))?;
                if record.certificate.certificate.intent.epoch_number != expected_number
                    || record
                        .certificate
                        .certificate
                        .intent
                        .expected_previous_epoch_hash
                        != Some(previous.epoch_hash)
                    || record.certificate.certificate.intent.deployment_id
                        != previous.certificate.certificate.intent.deployment_id
                {
                    return Err(Error::VerificationFailed(
                        "database epoch journal does not form one contiguous signed chain"
                            .to_string(),
                    ));
                }
            } else if record.certificate.certificate.intent.epoch_number != 1
                || record
                    .certificate
                    .certificate
                    .intent
                    .expected_previous_epoch_hash
                    .is_some()
            {
                return Err(Error::VerificationFailed(
                    "database epoch journal does not begin with epoch one".to_string(),
                ));
            }
            records.push(record);
        }
        Ok(records)
    }

    pub async fn latest_database_epoch(&self) -> Result<Option<AuthorityDatabaseEpochRecord>> {
        Ok(self.database_epochs().await?.pop())
    }

    pub async fn recovery_reconciliations(
        &self,
        epoch_hash: ContentHash,
    ) -> Result<Vec<AuthorityRecoveryReconciliationRecord>> {
        let query = format!(
            "{RECOVERY_RECONCILIATION_SELECT_COLUMNS} WHERE epoch_hash = $1 ORDER BY accepted_at, reconciliation_id"
        );
        let rows = sqlx::query(&query)
            .bind(epoch_hash.0.to_vec())
            .fetch_all(self.pool())
            .await
            .map_err(sql_error)?;
        let mut records = Vec::with_capacity(rows.len());
        for row in rows {
            let record = recovery_reconciliation_from_row(row)?;
            self.verify_reconciliation_publication(&record).await?;
            records.push(record);
        }
        Ok(records)
    }

    pub async fn all_recovery_reconciliations(
        &self,
    ) -> Result<Vec<AuthorityRecoveryReconciliationRecord>> {
        let query = format!(
            "{RECOVERY_RECONCILIATION_SELECT_COLUMNS} ORDER BY accepted_at, reconciliation_id"
        );
        let rows = sqlx::query(&query)
            .fetch_all(self.pool())
            .await
            .map_err(sql_error)?;
        let mut records = Vec::with_capacity(rows.len());
        for row in rows {
            let record = recovery_reconciliation_from_row(row)?;
            self.verify_reconciliation_publication(&record).await?;
            records.push(record);
        }
        Ok(records)
    }

    pub async fn delivery_acknowledgements(
        &self,
        delivery_id: Uuid,
    ) -> Result<Vec<AuthorityDeliveryAcknowledgementRecord>> {
        let query = format!(
            "{DELIVERY_ACK_SELECT_COLUMNS} WHERE delivery_id = $1 ORDER BY accepted_at, acknowledgement_id"
        );
        let rows = sqlx::query(&query)
            .bind(delivery_id)
            .fetch_all(self.pool())
            .await
            .map_err(sql_error)?;
        rows.into_iter()
            .map(delivery_acknowledgement_from_row)
            .collect()
    }

    pub async fn database_epoch_summary(
        &self,
        governance: &CredentialGovernanceProjection,
    ) -> Result<AuthorityDatabaseEpochSummary> {
        let epochs = self.database_epochs().await?;
        let epoch_count = u64::try_from(epochs.len()).map_err(|_| {
            Error::Storage("database epoch count exceeds protocol limits".to_string())
        })?;
        let reconciliations = self.all_recovery_reconciliations().await?;
        let reconciliation_count = u64::try_from(reconciliations.len()).map_err(|_| {
            Error::Storage("recovery reconciliation count exceeds protocol limits".to_string())
        })?;
        let latest = epochs.last().cloned();
        let latest_reconciliation_delivery = latest.as_ref().and_then(|record| {
            reconciliations
                .iter()
                .rev()
                .find(|reconciliation| {
                    reconciliation.reconciliation.reconciliation.epoch_hash == record.epoch_hash
                })
                .map(|reconciliation| reconciliation.publication_delivery_id)
        });
        let latest_reconciled = match latest_reconciliation_delivery {
            Some(delivery_id) => sqlx::query_scalar::<_, bool>(
                "SELECT published_at IS NOT NULL FROM desci_authority_outbox WHERE id = $1",
            )
            .bind(delivery_id)
            .fetch_one(self.pool())
            .await
            .map_err(sql_error)?,
            None => false,
        };
        let latest_delivery_id = match &latest {
            Some(record) => {
                let published: bool = sqlx::query_scalar(
                    "SELECT published_at IS NOT NULL FROM desci_authority_outbox WHERE id = $1",
                )
                .bind(record.publication_delivery_id)
                .fetch_one(self.pool())
                .await
                .map_err(sql_error)?;
                published.then_some(record.publication_delivery_id)
            }
            None => None,
        };
        let acknowledgement_organizations = match latest_delivery_id {
            Some(delivery_id) => self
                .delivery_acknowledgements(delivery_id)
                .await?
                .into_iter()
                .filter(|record| {
                    record
                        .acknowledgement
                        .verify_against_governance(governance)
                        .is_ok()
                })
                .map(|record| record.acknowledgement.acknowledgement.organization)
                .collect::<BTreeSet<_>>()
                .len(),
            None => 0,
        };
        Ok(AuthorityDatabaseEpochSummary {
            epoch_count,
            latest_epoch_number: latest
                .as_ref()
                .map(|record| record.certificate.certificate.intent.epoch_number),
            latest_epoch_hash: latest.as_ref().map(|record| record.epoch_hash),
            latest_primary_id: latest.as_ref().map(|record| {
                record
                    .certificate
                    .certificate
                    .intent
                    .candidate_primary_id
                    .clone()
            }),
            latest_promotion_mode: latest
                .as_ref()
                .map(|record| record.certificate.certificate.intent.mode),
            latest_epoch_reconciled: latest_reconciled,
            recovery_reconciliation_count: reconciliation_count,
            latest_published_delivery_id: latest_delivery_id,
            latest_delivery_acknowledgement_organizations: acknowledgement_organizations,
        })
    }
}

async fn begin_serializable(transaction: &mut Transaction<'_, Postgres>) -> Result<()> {
    sqlx::query("SET TRANSACTION ISOLATION LEVEL SERIALIZABLE")
        .execute(&mut **transaction)
        .await
        .map_err(sql_error)?;
    Ok(())
}

async fn authority_state_commitment_in_transaction(
    transaction: &mut Transaction<'_, Postgres>,
    anchor_checkpoint_hash: ContentHash,
) -> Result<AuthorityDatabaseStateCommitment> {
    let credential_count: i64 = sqlx::query_scalar("SELECT COUNT(*) FROM desci_credential_events")
        .fetch_one(&mut **transaction)
        .await
        .map_err(sql_error)?;
    let credential_head = latest_hash(
        transaction,
        "SELECT record_hash FROM desci_credential_events ORDER BY sequence DESC LIMIT 1",
        "credential head",
    )
    .await?;
    let governance_count: i64 = sqlx::query_scalar("SELECT COUNT(*) FROM desci_governance_events")
        .fetch_one(&mut **transaction)
        .await
        .map_err(sql_error)?;
    let governance_head = latest_hash(
        transaction,
        "SELECT record_hash FROM desci_governance_events ORDER BY sequence DESC LIMIT 1",
        "governance head",
    )
    .await?;
    let scientific_event_count: i64 =
        sqlx::query_scalar("SELECT COUNT(*) FROM desci_scientific_events")
            .fetch_one(&mut **transaction)
            .await
            .map_err(sql_error)?;
    let scientific_heads = sqlx::query(
        r#"
        SELECT DISTINCT ON (stream_id) stream_id, sequence, event_hash
        FROM desci_scientific_events
        ORDER BY stream_id, sequence DESC
        "#,
    )
    .fetch_all(&mut **transaction)
    .await
    .map_err(sql_error)?;
    let scientific_stream_heads_root = head_rows_root(
        b"MYCELIX-DESCI-SCIENTIFIC-STREAM-HEADS\0",
        &scientific_heads,
        "event_hash",
    )?;
    let receipt_count: i64 = sqlx::query_scalar(
        "SELECT COUNT(*) FROM desci_authority_receipts WHERE status = 'committed'",
    )
    .fetch_one(&mut **transaction)
    .await
    .map_err(sql_error)?;
    let receipt_heads = sqlx::query(
        r#"
        SELECT DISTINCT ON (stream_id) stream_id, sequence, receipt_hash
        FROM desci_authority_receipts
        WHERE status = 'committed'
        ORDER BY stream_id, sequence DESC
        "#,
    )
    .fetch_all(&mut **transaction)
    .await
    .map_err(sql_error)?;
    let authority_receipt_heads_root = head_rows_root(
        b"MYCELIX-DESCI-AUTHORITY-RECEIPT-HEADS\0",
        &receipt_heads,
        "receipt_hash",
    )?;
    let commitment = AuthorityDatabaseStateCommitment {
        authority_schema_version: u16::try_from(AUTHORITY_SCHEMA_VERSION).map_err(|_| {
            Error::Storage("authority schema version exceeds protocol width".to_string())
        })?,
        credential_event_count: i64_to_u64(credential_count, "credential event count")?,
        credential_head,
        governance_event_count: i64_to_u64(governance_count, "governance event count")?,
        governance_head,
        scientific_event_count: i64_to_u64(scientific_event_count, "scientific event count")?,
        scientific_stream_count: u64::try_from(scientific_heads.len())
            .map_err(|_| Error::Storage("scientific stream count exceeds u64".to_string()))?,
        scientific_stream_heads_root,
        authority_receipt_count: i64_to_u64(receipt_count, "authority receipt count")?,
        authority_receipt_stream_count: u64::try_from(receipt_heads.len()).map_err(|_| {
            Error::Storage("authority receipt stream count exceeds u64".to_string())
        })?,
        authority_receipt_heads_root,
        anchor_checkpoint_hash,
    };
    commitment.validate()?;
    Ok(commitment)
}

async fn latest_hash(
    transaction: &mut Transaction<'_, Postgres>,
    query: &str,
    label: &str,
) -> Result<ContentHash> {
    let row = sqlx::query(query)
        .fetch_optional(&mut **transaction)
        .await
        .map_err(sql_error)?
        .ok_or_else(|| Error::Storage(format!("PostgreSQL {label} is absent")))?;
    content_hash(row.try_get("record_hash").map_err(sql_error)?, label)
}

fn head_rows_root(
    domain: &[u8],
    rows: &[sqlx::postgres::PgRow],
    hash_column: &str,
) -> Result<ContentHash> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(domain);
    for row in rows {
        let stream_id: Uuid = row.try_get("stream_id").map_err(sql_error)?;
        let sequence: i64 = row.try_get("sequence").map_err(sql_error)?;
        let hash = content_hash(row.try_get(hash_column).map_err(sql_error)?, hash_column)?;
        bytes.extend_from_slice(stream_id.as_bytes());
        bytes.extend_from_slice(&sequence.to_be_bytes());
        bytes.extend_from_slice(&hash.0);
    }
    Ok(ContentHash::digest(&bytes))
}

async fn durable_credential_projection_in_transaction(
    transaction: &mut Transaction<'_, Postgres>,
    trusted_acceptance_keys: &BTreeSet<[u8; 32]>,
) -> Result<ScientificCredentialRegistryProjection> {
    let rows = sqlx::query(
        "SELECT sequence, record_hash, record_json FROM desci_credential_events ORDER BY sequence",
    )
    .fetch_all(&mut **transaction)
    .await
    .map_err(sql_error)?;
    let mut records = Vec::with_capacity(rows.len());
    for row in rows {
        let sequence: i64 = row.try_get("sequence").map_err(sql_error)?;
        let value: serde_json::Value = row.try_get("record_json").map_err(sql_error)?;
        let record: RecordedScientificCredentialEvent = serde_json::from_value(value)?;
        let indexed_hash = content_hash(
            row.try_get("record_hash").map_err(sql_error)?,
            "credential record hash",
        )?;
        if indexed_hash != record.record_hash()?
            || sequence != u64_to_i64(record.event.envelope.sequence, "credential record sequence")?
        {
            return Err(Error::VerificationFailed(
                "indexed credential metadata disagrees with its signed record".to_string(),
            ));
        }
        records.push(record);
    }
    ScientificCredentialRegistryProjection::rebuild(&records, trusted_acceptance_keys)
}

async fn durable_governance_service_keys_in_transaction(
    transaction: &mut Transaction<'_, Postgres>,
) -> Result<BTreeSet<[u8; 32]>> {
    let rows = sqlx::query(
        "SELECT sequence, record_hash, record_json FROM desci_governance_events ORDER BY sequence",
    )
    .fetch_all(&mut **transaction)
    .await
    .map_err(sql_error)?;
    let mut opened = std::collections::BTreeMap::new();
    let mut keys = BTreeSet::new();
    for row in rows {
        let sequence: i64 = row.try_get("sequence").map_err(sql_error)?;
        let value: serde_json::Value = row.try_get("record_json").map_err(sql_error)?;
        let record: RecordedCredentialGovernanceEvent = serde_json::from_value(value)?;
        let indexed_hash = content_hash(
            row.try_get("record_hash").map_err(sql_error)?,
            "governance record hash",
        )?;
        if indexed_hash != record.record_hash()?
            || sequence != u64_to_i64(record.event.envelope.sequence, "governance record sequence")?
        {
            return Err(Error::VerificationFailed(
                "indexed governance metadata disagrees with its signed record".to_string(),
            ));
        }
        match &record.event.envelope.payload {
            CredentialGovernancePayload::PolicyInitialized {
                initial_acceptance_service_keys,
                ..
            } => {
                keys.extend(
                    initial_acceptance_service_keys
                        .iter()
                        .map(|key| key.public_key),
                );
            }
            CredentialGovernancePayload::ProposalOpened {
                proposal_id,
                action,
                ..
            } => {
                if opened.insert(*proposal_id, action.clone()).is_some() {
                    return Err(Error::VerificationFailed(
                        "governance proposal id was opened more than once".to_string(),
                    ));
                }
            }
            CredentialGovernancePayload::ProposalExecuted { proposal_id } => {
                let action = opened.get(proposal_id).ok_or_else(|| {
                    Error::VerificationFailed(
                        "governance proposal executed before its durable opening".to_string(),
                    )
                })?;
                match action {
                    CredentialGovernanceAction::AuthorizeAcceptanceServiceKey { key } => {
                        keys.insert(key.public_key);
                    }
                    CredentialGovernanceAction::AuthorizeTransparencyWitness { witness } => {
                        keys.insert(witness.public_key);
                    }
                    _ => {}
                }
            }
            _ => {}
        }
    }
    Ok(keys)
}

async fn verify_durable_database_epoch_authorization(
    transaction: &mut Transaction<'_, Postgres>,
    proposal_id: Uuid,
    expected_intent: &DatabaseEpochPromotionIntent,
    expected_checkpoint_hash: ContentHash,
    expected_execution_hash: ContentHash,
) -> Result<()> {
    let rows = sqlx::query(
        "SELECT sequence, record_hash, record_json FROM desci_governance_events ORDER BY sequence",
    )
    .fetch_all(&mut **transaction)
    .await
    .map_err(sql_error)?;
    let mut opened_sequence = None;
    let mut approvals = BTreeSet::new();
    let mut executed = false;
    let mut checkpoint_present = false;
    for row in rows {
        let sequence: i64 = row.try_get("sequence").map_err(sql_error)?;
        let value: serde_json::Value = row.try_get("record_json").map_err(sql_error)?;
        let record: RecordedCredentialGovernanceEvent = serde_json::from_value(value)?;
        let indexed = content_hash(
            row.try_get("record_hash").map_err(sql_error)?,
            "governance record hash",
        )?;
        if indexed != record.record_hash()?
            || sequence != u64_to_i64(record.event.envelope.sequence, "governance record sequence")?
        {
            return Err(Error::VerificationFailed(
                "indexed governance metadata disagrees with its signed record".to_string(),
            ));
        }
        match &record.event.envelope.payload {
            CredentialGovernancePayload::ProposalOpened {
                proposal_id: candidate,
                action,
                ..
            } if candidate.0 == proposal_id => {
                if opened_sequence.is_some() {
                    return Err(Error::VerificationFailed(
                        "database epoch proposal id was opened more than once".to_string(),
                    ));
                }
                let CredentialGovernanceAction::AuthorizeDatabaseEpochPromotion { intent } = action
                else {
                    return Err(Error::VerificationFailed(
                        "durable proposal does not authorize a database epoch".to_string(),
                    ));
                };
                if intent.intent_hash()? != expected_intent.intent_hash()? {
                    return Err(Error::VerificationFailed(
                        "durable database epoch proposal differs from the certificate intent"
                            .to_string(),
                    ));
                }
                opened_sequence = Some(sequence);
            }
            CredentialGovernancePayload::ProposalApproved {
                proposal_id: candidate,
            } if candidate.0 == proposal_id => {
                if opened_sequence.is_none() || executed {
                    return Err(Error::VerificationFailed(
                        "database epoch proposal approval has invalid durable ordering".to_string(),
                    ));
                }
                approvals.insert(record.event.envelope.actor.clone());
            }
            CredentialGovernancePayload::ProposalExecuted {
                proposal_id: candidate,
            } if candidate.0 == proposal_id => {
                let Some(opened) = opened_sequence else {
                    return Err(Error::VerificationFailed(
                        "database epoch proposal executed before its durable opening".to_string(),
                    ));
                };
                if sequence <= opened || indexed != expected_execution_hash || executed {
                    return Err(Error::VerificationFailed(
                        "database epoch certificate references an invalid governance execution"
                            .to_string(),
                    ));
                }
                executed = true;
            }
            CredentialGovernancePayload::TransparencyCheckpointPublished { checkpoint }
                if checkpoint.checkpoint_hash() == expected_checkpoint_hash =>
            {
                checkpoint_present = true;
            }
            _ => {}
        }
    }
    if opened_sequence.is_none() || !executed || !checkpoint_present {
        return Err(Error::VerificationFailed(
            "database epoch authorization is incomplete in durable governance history".to_string(),
        ));
    }
    if let Some(ceremony) = &expected_intent.emergency_ceremony {
        let approving_participants = ceremony
            .named_participants
            .iter()
            .filter(|participant| approvals.contains(*participant))
            .count();
        if approving_participants < 2 {
            return Err(Error::VerificationFailed(
                "durable disaster-recovery authorization lacks two named ceremony approvers"
                    .to_string(),
            ));
        }
    }
    Ok(())
}

async fn verify_governance_projection_head(
    transaction: &mut Transaction<'_, Postgres>,
    expected_head: Option<ContentHash>,
) -> Result<()> {
    let durable: Option<Vec<u8>> = sqlx::query_scalar(
        "SELECT record_hash FROM desci_governance_events ORDER BY sequence DESC LIMIT 1",
    )
    .fetch_optional(&mut **transaction)
    .await
    .map_err(sql_error)?;
    let durable = durable
        .map(|bytes| content_hash(bytes, "governance projection head"))
        .transpose()?;
    if durable != expected_head {
        return Err(Error::Storage(
            "governance state changed before delivery acknowledgement commit; synchronize and retry"
                .to_string(),
        ));
    }
    Ok(())
}

async fn insert_outbox(
    transaction: &mut Transaction<'_, Postgres>,
    id: Uuid,
    topic: &str,
    aggregate_id: &str,
    aggregate_sequence: i64,
    delivery: serde_json::Value,
    delivery_hash: ContentHash,
    created_at: DateTime<Utc>,
) -> Result<()> {
    sqlx::query(
        r#"
        INSERT INTO desci_authority_outbox (
            id, topic, aggregate_id, aggregate_sequence,
            payload, payload_hash, created_at
        ) VALUES ($1, $2, $3, $4, $5, $6, $7)
        "#,
    )
    .bind(id)
    .bind(topic)
    .bind(aggregate_id)
    .bind(aggregate_sequence)
    .bind(delivery)
    .bind(delivery_hash.0.to_vec())
    .bind(created_at)
    .execute(&mut **transaction)
    .await
    .map_err(sql_error)?;
    Ok(())
}

fn database_epoch_from_row(row: sqlx::postgres::PgRow) -> Result<AuthorityDatabaseEpochRecord> {
    let value: serde_json::Value = row.try_get("certificate_json").map_err(sql_error)?;
    let certificate: SignedAuthorityDatabaseEpochCertificate = serde_json::from_value(value)?;
    certificate.verify()?;
    let epoch_hash = content_hash(
        row.try_get("epoch_hash").map_err(sql_error)?,
        "database epoch hash",
    )?;
    let previous_bytes: Option<Vec<u8>> = row.try_get("previous_epoch_hash").map_err(sql_error)?;
    let previous_hash = previous_bytes
        .map(|bytes| content_hash(bytes, "previous database epoch hash"))
        .transpose()?;
    let state_commitment_hash = content_hash(
        row.try_get("state_commitment_hash").map_err(sql_error)?,
        "database state commitment hash",
    )?;
    let indexed_number = i64_to_u64(
        row.try_get("epoch_number").map_err(sql_error)?,
        "database epoch number",
    )?;
    let indexed_id: Uuid = row.try_get("epoch_id").map_err(sql_error)?;
    let indexed_deployment: String = row.try_get("deployment_id").map_err(sql_error)?;
    let indexed_primary: String = row.try_get("primary_id").map_err(sql_error)?;
    let indexed_promoted_at: DateTime<Utc> = row.try_get("promoted_at").map_err(sql_error)?;
    let indexed_accepted_at: DateTime<Utc> = row.try_get("accepted_at").map_err(sql_error)?;
    if epoch_hash != certificate.epoch_hash()?
        || previous_hash != certificate.certificate.intent.expected_previous_epoch_hash
        || state_commitment_hash != certificate.certificate.state_commitment.commitment_hash()?
        || indexed_number != certificate.certificate.intent.epoch_number
        || indexed_id != certificate.certificate.intent.promotion_id
        || indexed_deployment != certificate.certificate.intent.deployment_id
        || indexed_primary != certificate.certificate.intent.candidate_primary_id
        || indexed_promoted_at != certificate.certificate.promoted_at
        || indexed_accepted_at < indexed_promoted_at
    {
        return Err(Error::VerificationFailed(
            "indexed database epoch metadata disagrees with its signed certificate".to_string(),
        ));
    }
    Ok(AuthorityDatabaseEpochRecord {
        certificate,
        epoch_hash,
        publication_delivery_id: row.try_get("publication_delivery_id").map_err(sql_error)?,
        accepted_at: indexed_accepted_at,
    })
}

fn recovery_reconciliation_from_row(
    row: sqlx::postgres::PgRow,
) -> Result<AuthorityRecoveryReconciliationRecord> {
    let value: serde_json::Value = row.try_get("reconciliation_json").map_err(sql_error)?;
    let reconciliation: SignedAuthorityRecoveryReconciliation = serde_json::from_value(value)?;
    reconciliation.verify()?;
    let reconciliation_hash = content_hash(
        row.try_get("reconciliation_hash").map_err(sql_error)?,
        "recovery reconciliation hash",
    )?;
    let indexed_id: Uuid = row.try_get("reconciliation_id").map_err(sql_error)?;
    let indexed_epoch_hash = content_hash(
        row.try_get("epoch_hash").map_err(sql_error)?,
        "recovery epoch hash",
    )?;
    let indexed_verified_at: DateTime<Utc> = row.try_get("verified_at").map_err(sql_error)?;
    let indexed_accepted_at: DateTime<Utc> = row.try_get("accepted_at").map_err(sql_error)?;
    if reconciliation_hash != reconciliation.reconciliation_hash()?
        || indexed_id != reconciliation.reconciliation.reconciliation_id
        || indexed_epoch_hash != reconciliation.reconciliation.epoch_hash
        || indexed_verified_at != reconciliation.reconciliation.verified_at
        || indexed_accepted_at < indexed_verified_at
    {
        return Err(Error::VerificationFailed(
            "indexed recovery reconciliation metadata disagrees with its signed record".to_string(),
        ));
    }
    Ok(AuthorityRecoveryReconciliationRecord {
        reconciliation,
        reconciliation_hash,
        publication_delivery_id: row.try_get("publication_delivery_id").map_err(sql_error)?,
        accepted_at: indexed_accepted_at,
    })
}

fn delivery_acknowledgement_from_row(
    row: sqlx::postgres::PgRow,
) -> Result<AuthorityDeliveryAcknowledgementRecord> {
    let value: serde_json::Value = row.try_get("acknowledgement_json").map_err(sql_error)?;
    let acknowledgement: SignedAuthorityDeliveryAcknowledgement = serde_json::from_value(value)?;
    acknowledgement.verify()?;
    let acknowledgement_hash = content_hash(
        row.try_get("acknowledgement_hash").map_err(sql_error)?,
        "delivery acknowledgement hash",
    )?;
    let indexed_id: Uuid = row.try_get("acknowledgement_id").map_err(sql_error)?;
    let indexed_delivery_id: Uuid = row.try_get("delivery_id").map_err(sql_error)?;
    let indexed_delivery_hash = content_hash(
        row.try_get("delivery_hash").map_err(sql_error)?,
        "authority delivery hash",
    )?;
    let indexed_actor: String = row.try_get("witness_actor").map_err(sql_error)?;
    let indexed_organization: String = row.try_get("witness_organization").map_err(sql_error)?;
    let indexed_acknowledged_at: DateTime<Utc> =
        row.try_get("acknowledged_at").map_err(sql_error)?;
    let indexed_accepted_at: DateTime<Utc> = row.try_get("accepted_at").map_err(sql_error)?;
    if acknowledgement_hash != acknowledgement.acknowledgement_hash()?
        || indexed_id != acknowledgement.acknowledgement.acknowledgement_id
        || indexed_delivery_id != acknowledgement.acknowledgement.delivery_id
        || indexed_delivery_hash != acknowledgement.acknowledgement.delivery_hash
        || indexed_actor != acknowledgement.acknowledgement.actor.as_str()
        || indexed_organization != acknowledgement.acknowledgement.organization.as_str()
        || indexed_acknowledged_at != acknowledgement.acknowledgement.acknowledged_at
        || indexed_accepted_at < indexed_acknowledged_at
    {
        return Err(Error::VerificationFailed(
            "indexed delivery acknowledgement metadata disagrees with its signed record"
                .to_string(),
        ));
    }
    Ok(AuthorityDeliveryAcknowledgementRecord {
        acknowledgement,
        acknowledgement_hash,
        accepted_at: indexed_accepted_at,
    })
}

fn content_hash(bytes: Vec<u8>, label: &str) -> Result<ContentHash> {
    let bytes: [u8; 32] = bytes
        .try_into()
        .map_err(|_| Error::Storage(format!("PostgreSQL {label} must contain exactly 32 bytes")))?;
    Ok(ContentHash(bytes))
}

fn u64_to_i64(value: u64, label: &str) -> Result<i64> {
    i64::try_from(value)
        .map_err(|_| Error::Validation(format!("{label} exceeds PostgreSQL BIGINT")))
}

fn i64_to_u64(value: i64, label: &str) -> Result<u64> {
    u64::try_from(value).map_err(|_| Error::Storage(format!("negative PostgreSQL {label}")))
}

fn sql_error(error: impl std::fmt::Display) -> Error {
    Error::Storage(format!("PostgreSQL authority epoch backend: {error}"))
}
