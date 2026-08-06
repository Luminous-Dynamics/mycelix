// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Governed database epochs, recovery reconciliation, and delivery acknowledgements.
//!
//! A PostgreSQL primary promotion is an authority transition, not merely an
//! infrastructure event. This module provides deterministic protocol objects
//! that bind a threshold-governed promotion intent to the exact authority state
//! observed at promotion time. Recovery reconciliation separately proves that a
//! restored database matches the state commitment authorized for an epoch.

use crate::authority_signing::AuthoritySigner;
use crate::scientific_credential_governance::{
    CredentialGovernanceProjection, CredentialGovernanceProposalId,
    CredentialTransparencyCheckpoint,
};
use crate::scientific_events::{ActorId, ContentHash, OrganizationId};
use crate::{Error, Result};
use chrono::{DateTime, Duration, Utc};
use ed25519_dalek::{Signature, SigningKey, Verifier, VerifyingKey};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use url::Url;
use uuid::Uuid;

pub const AUTHORITY_DATABASE_EPOCH_PROTOCOL: &str = "mycelix-desci-authority-database-epoch";
pub const AUTHORITY_DATABASE_EPOCH_PROTOCOL_VERSION: u16 = 1;
pub const AUTHORITY_DATABASE_EPOCH_SCHEMA_VERSION: u16 = 1;
pub const AUTHORITY_DATABASE_EPOCH_CODEC: &str = "mycelix-canonical-binary-v1";
pub const AUTHORITY_RECOVERY_RECONCILIATION_PROTOCOL: &str =
    "mycelix-desci-authority-recovery-reconciliation";
pub const AUTHORITY_DELIVERY_ACK_PROTOCOL: &str = "mycelix-desci-authority-delivery-ack";

const MAX_IDENTIFIER_BYTES: usize = 256;
const MAX_REASON_BYTES: usize = 4_096;
const MAX_REFERENCE_BYTES: usize = 2_048;
const MAX_EMERGENCY_DURATION_SECONDS: i64 = 7 * 24 * 60 * 60;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DatabasePromotionMode {
    InitialActivation,
    PlannedFailover,
    DisasterRecovery,
}

impl DatabasePromotionMode {
    fn code(self) -> u8 {
        match self {
            Self::InitialActivation => 1,
            Self::PlannedFailover => 2,
            Self::DisasterRecovery => 3,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EmergencyRecoveryCeremony {
    pub incident_id: Uuid,
    pub declared_at: DateTime<Utc>,
    pub expires_at: DateTime<Utc>,
    pub incident_summary: String,
    pub recovery_objective: String,
    pub acknowledged_data_loss_window: String,
    pub coordination_channel: String,
    pub named_participants: Vec<ActorId>,
}

impl EmergencyRecoveryCeremony {
    pub fn validate(&self) -> Result<()> {
        validate_text(&self.incident_summary, MAX_REASON_BYTES, "incident summary")?;
        validate_text(
            &self.recovery_objective,
            MAX_REASON_BYTES,
            "recovery objective",
        )?;
        validate_text(
            &self.acknowledged_data_loss_window,
            MAX_REASON_BYTES,
            "acknowledged data-loss window",
        )?;
        validate_text(
            &self.coordination_channel,
            MAX_REFERENCE_BYTES,
            "coordination channel",
        )?;
        if self.expires_at <= self.declared_at
            || self.expires_at - self.declared_at
                > Duration::seconds(MAX_EMERGENCY_DURATION_SECONDS)
        {
            return Err(Error::Validation(
                "emergency ceremony must expire within seven days of declaration".to_string(),
            ));
        }
        let participants = self
            .named_participants
            .iter()
            .map(|participant| {
                participant.validate()?;
                Ok(participant.clone())
            })
            .collect::<Result<BTreeSet<_>>>()?;
        if participants.len() < 2 || participants.len() != self.named_participants.len() {
            return Err(Error::Validation(
                "emergency ceremony requires at least two unique named participants".to_string(),
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AuthorityDatabaseStateCommitment {
    pub authority_schema_version: u16,
    pub credential_event_count: u64,
    pub credential_head: ContentHash,
    pub governance_event_count: u64,
    pub governance_head: ContentHash,
    pub scientific_event_count: u64,
    pub scientific_stream_count: u64,
    pub scientific_stream_heads_root: ContentHash,
    pub authority_receipt_count: u64,
    pub authority_receipt_stream_count: u64,
    pub authority_receipt_heads_root: ContentHash,
    pub anchor_checkpoint_hash: ContentHash,
}

impl AuthorityDatabaseStateCommitment {
    pub fn validate(&self) -> Result<()> {
        if self.authority_schema_version == 0
            || self.credential_event_count == 0
            || self.governance_event_count == 0
            || self.authority_receipt_count > self.scientific_event_count
            || self.scientific_stream_count > self.scientific_event_count
            || self.authority_receipt_stream_count > self.authority_receipt_count
        {
            return Err(Error::Validation(
                "authority database state commitment contains impossible counts".to_string(),
            ));
        }
        Ok(())
    }

    pub fn commitment_hash(&self) -> Result<ContentHash> {
        self.validate()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-AUTHORITY-DATABASE-STATE\0");
        push_u16(&mut bytes, self.authority_schema_version);
        push_u64(&mut bytes, self.credential_event_count);
        push_hash(&mut bytes, self.credential_head);
        push_u64(&mut bytes, self.governance_event_count);
        push_hash(&mut bytes, self.governance_head);
        push_u64(&mut bytes, self.scientific_event_count);
        push_u64(&mut bytes, self.scientific_stream_count);
        push_hash(&mut bytes, self.scientific_stream_heads_root);
        push_u64(&mut bytes, self.authority_receipt_count);
        push_u64(&mut bytes, self.authority_receipt_stream_count);
        push_hash(&mut bytes, self.authority_receipt_heads_root);
        push_hash(&mut bytes, self.anchor_checkpoint_hash);
        Ok(ContentHash::digest(&bytes))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatabaseEpochPromotionIntent {
    pub promotion_id: Uuid,
    pub deployment_id: String,
    pub operator_actor: ActorId,
    pub candidate_primary_id: String,
    pub database_system_identifier: String,
    pub postgres_timeline: u64,
    pub recovery_lsn: String,
    pub epoch_number: u64,
    pub expected_previous_epoch_hash: Option<ContentHash>,
    pub anchor_checkpoint: CredentialTransparencyCheckpoint,
    pub epoch_signer_public_key: [u8; 32],
    pub mode: DatabasePromotionMode,
    pub requested_recovery_target: Option<DateTime<Utc>>,
    pub emergency_ceremony: Option<EmergencyRecoveryCeremony>,
    pub reason: String,
}

impl DatabaseEpochPromotionIntent {
    pub fn validate(&self) -> Result<()> {
        validate_text(&self.deployment_id, MAX_IDENTIFIER_BYTES, "deployment id")?;
        self.operator_actor.validate()?;
        validate_text(
            &self.candidate_primary_id,
            MAX_IDENTIFIER_BYTES,
            "candidate primary id",
        )?;
        validate_text(
            &self.database_system_identifier,
            MAX_IDENTIFIER_BYTES,
            "database system identifier",
        )?;
        validate_lsn(&self.recovery_lsn)?;
        validate_text(&self.reason, MAX_REASON_BYTES, "promotion reason")?;
        VerifyingKey::from_bytes(&self.epoch_signer_public_key)
            .map_err(|error| Error::Crypto(error.to_string()))?;
        if self.postgres_timeline == 0 || self.epoch_number == 0 {
            return Err(Error::Validation(
                "database epoch number and PostgreSQL timeline must be positive".to_string(),
            ));
        }
        if (self.epoch_number == 1) != self.expected_previous_epoch_hash.is_none() {
            return Err(Error::Validation(
                "epoch one must have no predecessor and later epochs must name one".to_string(),
            ));
        }
        if self.anchor_checkpoint.registry_event_count == 0
            || self.anchor_checkpoint.governance_event_count == 0
        {
            return Err(Error::Validation(
                "database epoch promotion requires a non-empty transparency checkpoint".to_string(),
            ));
        }
        match self.mode {
            DatabasePromotionMode::InitialActivation => {
                if self.epoch_number != 1
                    || self.requested_recovery_target.is_some()
                    || self.emergency_ceremony.is_some()
                {
                    return Err(Error::Validation(
                        "initial activation is valid only for epoch one without recovery metadata"
                            .to_string(),
                    ));
                }
            }
            DatabasePromotionMode::PlannedFailover => {
                if self.epoch_number == 1
                    || self.requested_recovery_target.is_some()
                    || self.emergency_ceremony.is_some()
                {
                    return Err(Error::Validation(
                        "planned failover requires a predecessor and cannot carry disaster-recovery metadata"
                            .to_string(),
                    ));
                }
            }
            DatabasePromotionMode::DisasterRecovery => {
                if self.epoch_number == 1 {
                    return Err(Error::Validation(
                        "disaster recovery requires a previously governed database epoch"
                            .to_string(),
                    ));
                }
                let recovery_target = self.requested_recovery_target.as_ref().ok_or_else(|| {
                    Error::Validation(
                        "disaster-recovery promotion requires an explicit recovery target"
                            .to_string(),
                    )
                })?;
                let ceremony = self.emergency_ceremony.as_ref().ok_or_else(|| {
                    Error::Validation(
                        "disaster-recovery promotion requires an emergency ceremony".to_string(),
                    )
                })?;
                ceremony.validate()?;
                if recovery_target > &ceremony.declared_at {
                    return Err(Error::Validation(
                        "disaster-recovery target must not postdate the emergency declaration"
                            .to_string(),
                    ));
                }
            }
        }
        Ok(())
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>> {
        self.validate()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-DATABASE-EPOCH-PROMOTION-INTENT\0");
        bytes.extend_from_slice(self.promotion_id.as_bytes());
        push_text(&mut bytes, &self.deployment_id)?;
        push_text(&mut bytes, self.operator_actor.as_str())?;
        push_text(&mut bytes, &self.candidate_primary_id)?;
        push_text(&mut bytes, &self.database_system_identifier)?;
        bytes.extend_from_slice(&self.postgres_timeline.to_be_bytes());
        push_text(&mut bytes, &self.recovery_lsn)?;
        push_u64(&mut bytes, self.epoch_number);
        push_option_hash(&mut bytes, self.expected_previous_epoch_hash);
        push_checkpoint(&mut bytes, &self.anchor_checkpoint);
        bytes.extend_from_slice(&self.epoch_signer_public_key);
        bytes.push(self.mode.code());
        push_option_datetime(&mut bytes, self.requested_recovery_target.as_ref());
        match &self.emergency_ceremony {
            Some(ceremony) => {
                bytes.push(1);
                bytes.extend_from_slice(ceremony.incident_id.as_bytes());
                push_datetime(&mut bytes, &ceremony.declared_at);
                push_datetime(&mut bytes, &ceremony.expires_at);
                push_text(&mut bytes, &ceremony.incident_summary)?;
                push_text(&mut bytes, &ceremony.recovery_objective)?;
                push_text(&mut bytes, &ceremony.acknowledged_data_loss_window)?;
                push_text(&mut bytes, &ceremony.coordination_channel)?;
                push_len(&mut bytes, ceremony.named_participants.len())?;
                for participant in &ceremony.named_participants {
                    push_text(&mut bytes, participant.as_str())?;
                }
            }
            None => bytes.push(0),
        }
        push_text(&mut bytes, &self.reason)?;
        Ok(bytes)
    }

    pub fn intent_hash(&self) -> Result<ContentHash> {
        Ok(ContentHash::digest(&self.canonical_bytes()?))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AuthorityDatabaseEpochCertificate {
    pub protocol: String,
    pub protocol_version: u16,
    pub codec: String,
    pub schema_version: u16,
    pub intent: DatabaseEpochPromotionIntent,
    pub governance_proposal_id: CredentialGovernanceProposalId,
    pub governance_execution_record_hash: ContentHash,
    pub promoted_at: DateTime<Utc>,
    pub state_commitment: AuthorityDatabaseStateCommitment,
}

impl AuthorityDatabaseEpochCertificate {
    pub fn new(
        intent: DatabaseEpochPromotionIntent,
        governance_proposal_id: CredentialGovernanceProposalId,
        governance_execution_record_hash: ContentHash,
        promoted_at: DateTime<Utc>,
        state_commitment: AuthorityDatabaseStateCommitment,
    ) -> Result<Self> {
        let certificate = Self {
            protocol: AUTHORITY_DATABASE_EPOCH_PROTOCOL.to_string(),
            protocol_version: AUTHORITY_DATABASE_EPOCH_PROTOCOL_VERSION,
            codec: AUTHORITY_DATABASE_EPOCH_CODEC.to_string(),
            schema_version: AUTHORITY_DATABASE_EPOCH_SCHEMA_VERSION,
            intent,
            governance_proposal_id,
            governance_execution_record_hash,
            promoted_at,
            state_commitment,
        };
        certificate.validate()?;
        Ok(certificate)
    }

    pub fn validate(&self) -> Result<()> {
        if self.protocol != AUTHORITY_DATABASE_EPOCH_PROTOCOL
            || self.protocol_version != AUTHORITY_DATABASE_EPOCH_PROTOCOL_VERSION
            || self.codec != AUTHORITY_DATABASE_EPOCH_CODEC
            || self.schema_version != AUTHORITY_DATABASE_EPOCH_SCHEMA_VERSION
        {
            return Err(Error::Validation(
                "unsupported authority database epoch protocol".to_string(),
            ));
        }
        self.intent.validate()?;
        self.state_commitment.validate()?;
        if let Some(ceremony) = &self.intent.emergency_ceremony {
            if self.promoted_at < ceremony.declared_at || self.promoted_at > ceremony.expires_at {
                return Err(Error::VerificationFailed(
                    "disaster-recovery promotion occurred outside its emergency ceremony window"
                        .to_string(),
                ));
            }
        }
        if self.state_commitment.anchor_checkpoint_hash
            != self.intent.anchor_checkpoint.checkpoint_hash()
            || self.state_commitment.credential_event_count
                < self.intent.anchor_checkpoint.registry_event_count
            || self.state_commitment.governance_event_count
                < self.intent.anchor_checkpoint.governance_event_count
        {
            return Err(Error::VerificationFailed(
                "database epoch state does not extend its governed checkpoint anchor".to_string(),
            ));
        }
        Ok(())
    }

    pub fn signing_bytes(&self) -> Result<Vec<u8>> {
        self.validate()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-AUTHORITY-DATABASE-EPOCH-CERTIFICATE\0");
        push_text(&mut bytes, &self.protocol)?;
        push_u16(&mut bytes, self.protocol_version);
        push_text(&mut bytes, &self.codec)?;
        push_u16(&mut bytes, self.schema_version);
        push_hash(&mut bytes, self.intent.intent_hash()?);
        bytes.extend_from_slice(self.governance_proposal_id.0.as_bytes());
        push_hash(&mut bytes, self.governance_execution_record_hash);
        push_datetime(&mut bytes, &self.promoted_at);
        push_hash(&mut bytes, self.state_commitment.commitment_hash()?);
        Ok(bytes)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignedAuthorityDatabaseEpochCertificate {
    pub certificate: AuthorityDatabaseEpochCertificate,
    pub signer_public_key: [u8; 32],
    pub signature: Vec<u8>,
}

impl SignedAuthorityDatabaseEpochCertificate {
    pub fn sign(certificate: AuthorityDatabaseEpochCertificate, key: &SigningKey) -> Result<Self> {
        Self::sign_with(certificate, key)
    }

    pub fn sign_with(
        certificate: AuthorityDatabaseEpochCertificate,
        signer: &dyn AuthoritySigner,
    ) -> Result<Self> {
        certificate.validate()?;
        if signer.verifying_key().to_bytes() != certificate.intent.epoch_signer_public_key {
            return Err(Error::VerificationFailed(
                "database epoch signer does not match the governed promotion intent".to_string(),
            ));
        }
        let signed = Self {
            signature: signer.sign_message(&certificate.signing_bytes()?)?,
            signer_public_key: signer.verifying_key().to_bytes(),
            certificate,
        };
        signed.verify()?;
        Ok(signed)
    }

    pub fn verify(&self) -> Result<()> {
        self.certificate.validate()?;
        if self.signer_public_key != self.certificate.intent.epoch_signer_public_key {
            return Err(Error::VerificationFailed(
                "database epoch certificate uses an unapproved signer".to_string(),
            ));
        }
        verify_signature(
            self.signer_public_key,
            &self.certificate.signing_bytes()?,
            &self.signature,
        )
    }

    pub fn epoch_hash(&self) -> Result<ContentHash> {
        self.verify()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-SIGNED-AUTHORITY-DATABASE-EPOCH\0");
        push_bytes(&mut bytes, &self.certificate.signing_bytes()?)?;
        bytes.extend_from_slice(&self.signer_public_key);
        push_bytes(&mut bytes, &self.signature)?;
        Ok(ContentHash::digest(&bytes))
    }

    pub fn verify_governance_authorization(
        &self,
        governance: &CredentialGovernanceProjection,
    ) -> Result<()> {
        self.verify()?;
        let proposal = governance
            .proposals
            .get(&self.certificate.governance_proposal_id)
            .ok_or_else(|| {
                Error::NotFound("database epoch governance proposal not found".to_string())
            })?;
        if proposal.status
            != crate::scientific_credential_governance::CredentialGovernanceProposalStatus::Executed
        {
            return Err(Error::VerificationFailed(
                "database epoch governance proposal has not executed".to_string(),
            ));
        }
        let crate::scientific_credential_governance::CredentialGovernanceAction::AuthorizeDatabaseEpochPromotion { intent } = &proposal.action else {
            return Err(Error::VerificationFailed(
                "referenced governance proposal does not authorize a database epoch".to_string(),
            ));
        };
        if intent.intent_hash()? != self.certificate.intent.intent_hash()? {
            return Err(Error::VerificationFailed(
                "database epoch certificate differs from its governed intent".to_string(),
            ));
        }
        let checkpoint_hash = self.certificate.intent.anchor_checkpoint.checkpoint_hash();
        if !governance
            .checkpoints
            .iter()
            .any(|checkpoint| checkpoint.checkpoint_hash() == checkpoint_hash)
        {
            return Err(Error::VerificationFailed(
                "database epoch checkpoint anchor is absent from governed history".to_string(),
            ));
        }
        if let Some(ceremony) = &self.certificate.intent.emergency_ceremony {
            let approving_participants = ceremony
                .named_participants
                .iter()
                .filter(|participant| proposal.approvals.contains(*participant))
                .count();
            if approving_participants < 2 {
                return Err(Error::VerificationFailed(
                    "disaster-recovery ceremony requires at least two named governance approvers"
                        .to_string(),
                ));
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RecoveryReconciliationStatus {
    ExactMatch,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AuthorityRecoveryReconciliation {
    pub protocol: String,
    pub protocol_version: u16,
    pub codec: String,
    pub schema_version: u16,
    pub reconciliation_id: Uuid,
    pub operator_actor: ActorId,
    pub epoch_hash: ContentHash,
    pub epoch_number: u64,
    pub primary_id: String,
    pub database_system_identifier: String,
    pub postgres_timeline: u64,
    pub replayed_through_lsn: String,
    pub recovery_target_at: Option<DateTime<Utc>>,
    pub verified_at: DateTime<Utc>,
    pub observed_state: AuthorityDatabaseStateCommitment,
    pub status: RecoveryReconciliationStatus,
    pub notes: String,
}

impl AuthorityRecoveryReconciliation {
    pub fn validate(&self) -> Result<()> {
        if self.protocol != AUTHORITY_RECOVERY_RECONCILIATION_PROTOCOL
            || self.protocol_version != 1
            || self.codec != AUTHORITY_DATABASE_EPOCH_CODEC
            || self.schema_version != 1
        {
            return Err(Error::Validation(
                "unsupported authority recovery reconciliation protocol".to_string(),
            ));
        }
        self.operator_actor.validate()?;
        validate_text(&self.primary_id, MAX_IDENTIFIER_BYTES, "primary id")?;
        validate_text(
            &self.database_system_identifier,
            MAX_IDENTIFIER_BYTES,
            "database system identifier",
        )?;
        validate_lsn(&self.replayed_through_lsn)?;
        validate_text(&self.notes, MAX_REASON_BYTES, "reconciliation notes")?;
        if self.epoch_number == 0 || self.postgres_timeline == 0 {
            return Err(Error::Validation(
                "reconciliation epoch and PostgreSQL timeline must be positive".to_string(),
            ));
        }
        self.observed_state.validate()
    }

    pub fn signing_bytes(&self) -> Result<Vec<u8>> {
        self.validate()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-AUTHORITY-RECOVERY-RECONCILIATION\0");
        bytes.extend_from_slice(self.reconciliation_id.as_bytes());
        push_text(&mut bytes, self.operator_actor.as_str())?;
        push_hash(&mut bytes, self.epoch_hash);
        push_u64(&mut bytes, self.epoch_number);
        push_text(&mut bytes, &self.primary_id)?;
        push_text(&mut bytes, &self.database_system_identifier)?;
        bytes.extend_from_slice(&self.postgres_timeline.to_be_bytes());
        push_text(&mut bytes, &self.replayed_through_lsn)?;
        push_option_datetime(&mut bytes, self.recovery_target_at.as_ref());
        push_datetime(&mut bytes, &self.verified_at);
        push_hash(&mut bytes, self.observed_state.commitment_hash()?);
        bytes.push(1);
        push_text(&mut bytes, &self.notes)?;
        Ok(bytes)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignedAuthorityRecoveryReconciliation {
    pub reconciliation: AuthorityRecoveryReconciliation,
    pub signer_public_key: [u8; 32],
    pub signature: Vec<u8>,
}

impl SignedAuthorityRecoveryReconciliation {
    pub fn sign_with(
        reconciliation: AuthorityRecoveryReconciliation,
        signer: &dyn AuthoritySigner,
    ) -> Result<Self> {
        reconciliation.validate()?;
        let signed = Self {
            signature: signer.sign_message(&reconciliation.signing_bytes()?)?,
            signer_public_key: signer.verifying_key().to_bytes(),
            reconciliation,
        };
        signed.verify()?;
        Ok(signed)
    }

    pub fn verify(&self) -> Result<()> {
        verify_signature(
            self.signer_public_key,
            &self.reconciliation.signing_bytes()?,
            &self.signature,
        )
    }

    pub fn verify_against_epoch(
        &self,
        epoch: &SignedAuthorityDatabaseEpochCertificate,
    ) -> Result<()> {
        self.verify()?;
        epoch.verify()?;
        if self.reconciliation.epoch_hash != epoch.epoch_hash()?
            || self.reconciliation.epoch_number != epoch.certificate.intent.epoch_number
            || self.signer_public_key != epoch.signer_public_key
            || self.reconciliation.operator_actor != epoch.certificate.intent.operator_actor
            || self.reconciliation.primary_id != epoch.certificate.intent.candidate_primary_id
            || self.reconciliation.database_system_identifier
                != epoch.certificate.intent.database_system_identifier
            || self.reconciliation.postgres_timeline != epoch.certificate.intent.postgres_timeline
            || self.reconciliation.replayed_through_lsn != epoch.certificate.intent.recovery_lsn
            || self.reconciliation.recovery_target_at
                != epoch.certificate.intent.requested_recovery_target
            || self.reconciliation.verified_at < epoch.certificate.promoted_at
            || self.reconciliation.observed_state != epoch.certificate.state_commitment
        {
            return Err(Error::VerificationFailed(
                "recovery reconciliation does not exactly match its database epoch".to_string(),
            ));
        }
        Ok(())
    }

    pub fn reconciliation_hash(&self) -> Result<ContentHash> {
        self.verify()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-SIGNED-RECOVERY-RECONCILIATION\0");
        push_bytes(&mut bytes, &self.reconciliation.signing_bytes()?)?;
        bytes.extend_from_slice(&self.signer_public_key);
        push_bytes(&mut bytes, &self.signature)?;
        Ok(ContentHash::digest(&bytes))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AuthorityDeliveryAcknowledgement {
    pub protocol: String,
    pub protocol_version: u16,
    pub codec: String,
    pub schema_version: u16,
    pub acknowledgement_id: Uuid,
    pub delivery_id: Uuid,
    pub delivery_hash: ContentHash,
    pub topic: String,
    pub actor: ActorId,
    pub organization: OrganizationId,
    pub acknowledged_at: DateTime<Utc>,
    pub durable_reference: String,
}

impl AuthorityDeliveryAcknowledgement {
    pub fn new(
        acknowledgement_id: Uuid,
        delivery_id: Uuid,
        delivery_hash: ContentHash,
        topic: impl Into<String>,
        actor: ActorId,
        organization: OrganizationId,
        acknowledged_at: DateTime<Utc>,
        durable_reference: impl Into<String>,
    ) -> Result<Self> {
        let acknowledgement = Self {
            protocol: AUTHORITY_DELIVERY_ACK_PROTOCOL.to_string(),
            protocol_version: 1,
            codec: AUTHORITY_DATABASE_EPOCH_CODEC.to_string(),
            schema_version: 1,
            acknowledgement_id,
            delivery_id,
            delivery_hash,
            topic: topic.into(),
            actor,
            organization,
            acknowledged_at,
            durable_reference: durable_reference.into(),
        };
        acknowledgement.validate()?;
        Ok(acknowledgement)
    }

    pub fn validate(&self) -> Result<()> {
        if self.protocol != AUTHORITY_DELIVERY_ACK_PROTOCOL
            || self.protocol_version != 1
            || self.codec != AUTHORITY_DATABASE_EPOCH_CODEC
            || self.schema_version != 1
        {
            return Err(Error::Validation(
                "unsupported authority delivery acknowledgement protocol".to_string(),
            ));
        }
        self.actor.validate()?;
        self.organization.validate()?;
        validate_text(&self.topic, 128, "delivery topic")?;
        validate_text(
            &self.durable_reference,
            MAX_REFERENCE_BYTES,
            "durable delivery reference",
        )?;
        let reference = Url::parse(&self.durable_reference).map_err(|error| {
            Error::Validation(format!("invalid durable delivery reference: {error}"))
        })?;
        if reference.scheme() != "https"
            || reference.host_str().is_none()
            || reference.username() != ""
            || reference.password().is_some()
            || reference.query().is_some()
            || reference.fragment().is_some()
            || !reference
                .path()
                .to_ascii_lowercase()
                .contains(&self.delivery_hash.to_hex())
        {
            return Err(Error::Validation(
                "durable delivery reference must be an immutable HTTPS URL containing the delivery hash"
                    .to_string(),
            ));
        }
        Ok(())
    }

    pub fn signing_bytes(&self) -> Result<Vec<u8>> {
        self.validate()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-AUTHORITY-DELIVERY-ACKNOWLEDGEMENT\0");
        bytes.extend_from_slice(self.acknowledgement_id.as_bytes());
        bytes.extend_from_slice(self.delivery_id.as_bytes());
        push_hash(&mut bytes, self.delivery_hash);
        push_text(&mut bytes, &self.topic)?;
        push_text(&mut bytes, self.actor.as_str())?;
        push_text(&mut bytes, self.organization.as_str())?;
        push_datetime(&mut bytes, &self.acknowledged_at);
        push_text(&mut bytes, &self.durable_reference)?;
        Ok(bytes)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignedAuthorityDeliveryAcknowledgement {
    pub acknowledgement: AuthorityDeliveryAcknowledgement,
    pub signer_public_key: [u8; 32],
    pub signature: Vec<u8>,
}

impl SignedAuthorityDeliveryAcknowledgement {
    pub fn sign(
        acknowledgement: AuthorityDeliveryAcknowledgement,
        key: &SigningKey,
    ) -> Result<Self> {
        Self::sign_with(acknowledgement, key)
    }

    pub fn sign_with(
        acknowledgement: AuthorityDeliveryAcknowledgement,
        signer: &dyn AuthoritySigner,
    ) -> Result<Self> {
        acknowledgement.validate()?;
        let signed = Self {
            signature: signer.sign_message(&acknowledgement.signing_bytes()?)?,
            signer_public_key: signer.verifying_key().to_bytes(),
            acknowledgement,
        };
        signed.verify()?;
        Ok(signed)
    }

    pub fn verify(&self) -> Result<()> {
        verify_signature(
            self.signer_public_key,
            &self.acknowledgement.signing_bytes()?,
            &self.signature,
        )
    }

    pub fn verify_against_governance(
        &self,
        governance: &CredentialGovernanceProjection,
    ) -> Result<()> {
        self.verify()?;
        let authorized = governance
            .active_transparency_witness(
                &self.acknowledgement.actor,
                &self.signer_public_key,
                &self.acknowledgement.acknowledged_at,
            )
            .ok_or_else(|| {
                Error::VerificationFailed(
                    "delivery acknowledgement signer is not an active governed witness".to_string(),
                )
            })?;
        if authorized.organization != self.acknowledgement.organization {
            return Err(Error::VerificationFailed(
                "delivery acknowledgement organization does not match governed witness authority"
                    .to_string(),
            ));
        }
        Ok(())
    }

    pub fn acknowledgement_hash(&self) -> Result<ContentHash> {
        self.verify()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-SIGNED-AUTHORITY-DELIVERY-ACK\0");
        push_bytes(&mut bytes, &self.acknowledgement.signing_bytes()?)?;
        bytes.extend_from_slice(&self.signer_public_key);
        push_bytes(&mut bytes, &self.signature)?;
        Ok(ContentHash::digest(&bytes))
    }
}

fn verify_signature(public_key: [u8; 32], message: &[u8], signature: &[u8]) -> Result<()> {
    let key =
        VerifyingKey::from_bytes(&public_key).map_err(|error| Error::Crypto(error.to_string()))?;
    let signature =
        Signature::try_from(signature).map_err(|error| Error::Crypto(error.to_string()))?;
    key.verify(message, &signature)
        .map_err(|error| Error::VerificationFailed(error.to_string()))
}

fn validate_text(value: &str, max_bytes: usize, label: &str) -> Result<()> {
    let trimmed = value.trim();
    if trimmed.is_empty()
        || trimmed.len() > max_bytes
        || trimmed.chars().any(char::is_control)
        || trimmed != value
    {
        return Err(Error::Validation(format!(
            "{label} must contain 1-{max_bytes} canonical printable bytes"
        )));
    }
    Ok(())
}

fn validate_lsn(value: &str) -> Result<()> {
    validate_text(value, 64, "PostgreSQL LSN")?;
    let Some((left, right)) = value.split_once('/') else {
        return Err(Error::Validation(
            "PostgreSQL LSN must use hexadecimal X/Y form".to_string(),
        ));
    };
    if left.is_empty()
        || right.is_empty()
        || !left.chars().all(|character| character.is_ascii_hexdigit())
        || !right.chars().all(|character| character.is_ascii_hexdigit())
    {
        return Err(Error::Validation(
            "PostgreSQL LSN must use hexadecimal X/Y form".to_string(),
        ));
    }
    Ok(())
}

fn push_checkpoint(bytes: &mut Vec<u8>, checkpoint: &CredentialTransparencyCheckpoint) {
    push_u64(bytes, checkpoint.registry_event_count);
    push_hash(bytes, checkpoint.registry_head);
    push_hash(bytes, checkpoint.registry_merkle_root);
    push_u64(bytes, checkpoint.governance_event_count);
    push_hash(bytes, checkpoint.governance_head);
    push_hash(bytes, checkpoint.governance_merkle_root);
    push_datetime(bytes, &checkpoint.issued_at);
}

fn push_text(bytes: &mut Vec<u8>, value: &str) -> Result<()> {
    push_bytes(bytes, value.as_bytes())
}

fn push_bytes(bytes: &mut Vec<u8>, value: &[u8]) -> Result<()> {
    push_len(bytes, value.len())?;
    bytes.extend_from_slice(value);
    Ok(())
}

fn push_len(bytes: &mut Vec<u8>, len: usize) -> Result<()> {
    let len = u32::try_from(len).map_err(|_| {
        Error::Validation("canonical authority field exceeds u32 length".to_string())
    })?;
    bytes.extend_from_slice(&len.to_be_bytes());
    Ok(())
}

fn push_u16(bytes: &mut Vec<u8>, value: u16) {
    bytes.extend_from_slice(&value.to_be_bytes());
}

fn push_u64(bytes: &mut Vec<u8>, value: u64) {
    bytes.extend_from_slice(&value.to_be_bytes());
}

fn push_hash(bytes: &mut Vec<u8>, value: ContentHash) {
    bytes.extend_from_slice(&value.0);
}

fn push_option_hash(bytes: &mut Vec<u8>, value: Option<ContentHash>) {
    match value {
        Some(value) => {
            bytes.push(1);
            push_hash(bytes, value);
        }
        None => bytes.push(0),
    }
}

fn push_datetime(bytes: &mut Vec<u8>, value: &DateTime<Utc>) {
    bytes.extend_from_slice(&value.timestamp().to_be_bytes());
    bytes.extend_from_slice(&value.timestamp_subsec_nanos().to_be_bytes());
}

fn push_option_datetime(bytes: &mut Vec<u8>, value: Option<&DateTime<Utc>>) {
    match value {
        Some(value) => {
            bytes.push(1);
            push_datetime(bytes, value);
        }
        None => bytes.push(0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn checkpoint() -> CredentialTransparencyCheckpoint {
        CredentialTransparencyCheckpoint {
            registry_event_count: 3,
            registry_head: ContentHash([1; 32]),
            registry_merkle_root: ContentHash([2; 32]),
            governance_event_count: 5,
            governance_head: ContentHash([3; 32]),
            governance_merkle_root: ContentHash([4; 32]),
            issued_at: DateTime::from_timestamp(1_700_000_000, 0).unwrap(),
        }
    }

    fn state() -> AuthorityDatabaseStateCommitment {
        AuthorityDatabaseStateCommitment {
            authority_schema_version: 3,
            credential_event_count: 3,
            credential_head: ContentHash([1; 32]),
            governance_event_count: 6,
            governance_head: ContentHash([5; 32]),
            scientific_event_count: 2,
            scientific_stream_count: 1,
            scientific_stream_heads_root: ContentHash([6; 32]),
            authority_receipt_count: 2,
            authority_receipt_stream_count: 1,
            authority_receipt_heads_root: ContentHash([7; 32]),
            anchor_checkpoint_hash: checkpoint().checkpoint_hash(),
        }
    }

    #[test]
    fn epoch_signature_binds_governed_intent_and_state() {
        let key = SigningKey::from_bytes(&[71; 32]);
        let intent = DatabaseEpochPromotionIntent {
            promotion_id: Uuid::nil(),
            deployment_id: "deployment:test".to_string(),
            operator_actor: ActorId::new("did:key:operator").unwrap(),
            candidate_primary_id: "postgres-primary-b".to_string(),
            database_system_identifier: "cluster-123".to_string(),
            postgres_timeline: 2,
            recovery_lsn: "0/16B6C50".to_string(),
            epoch_number: 2,
            expected_previous_epoch_hash: Some(ContentHash([8; 32])),
            anchor_checkpoint: checkpoint(),
            epoch_signer_public_key: key.verifying_key().to_bytes(),
            mode: DatabasePromotionMode::PlannedFailover,
            requested_recovery_target: None,
            emergency_ceremony: None,
            reason: "planned primary maintenance".to_string(),
        };
        let certificate = AuthorityDatabaseEpochCertificate::new(
            intent,
            CredentialGovernanceProposalId(Uuid::nil()),
            ContentHash([9; 32]),
            DateTime::from_timestamp(1_700_000_100, 0).unwrap(),
            state(),
        )
        .unwrap();
        let mut signed = SignedAuthorityDatabaseEpochCertificate::sign(certificate, &key).unwrap();
        signed.verify().unwrap();
        signed.certificate.state_commitment.governance_event_count += 1;
        assert!(signed.verify().is_err());
    }

    #[test]
    fn delivery_acknowledgement_requires_immutable_hash_bound_https_reference() {
        let delivery_hash = ContentHash([42; 32]);
        let acknowledgement = AuthorityDeliveryAcknowledgement::new(
            Uuid::new_v4(),
            Uuid::new_v4(),
            delivery_hash,
            "authority.database-epoch.recorded.v1",
            ActorId::new("did:key:witness-a").unwrap(),
            OrganizationId::new("org:independent-a").unwrap(),
            DateTime::from_timestamp(1_700_000_000, 0).unwrap(),
            format!(
                "https://mirror.example/authority-deliveries/{}.json",
                delivery_hash.to_hex()
            ),
        )
        .unwrap();
        acknowledgement.validate().unwrap();

        let mut mutable_reference = acknowledgement.clone();
        mutable_reference.durable_reference =
            "https://mirror.example/authority-deliveries/latest.json".to_string();
        assert!(mutable_reference.validate().is_err());

        let mut credentialed_reference = acknowledgement;
        credentialed_reference.durable_reference = format!(
            "https://user:secret@mirror.example/authority-deliveries/{}.json",
            delivery_hash.to_hex()
        );
        assert!(credentialed_reference.validate().is_err());
    }

    #[test]
    fn disaster_recovery_requires_bounded_multi_person_ceremony() {
        let key = SigningKey::from_bytes(&[72; 32]);
        let mut intent = DatabaseEpochPromotionIntent {
            promotion_id: Uuid::new_v4(),
            deployment_id: "deployment:test".to_string(),
            operator_actor: ActorId::new("did:key:operator").unwrap(),
            candidate_primary_id: "postgres-recovered".to_string(),
            database_system_identifier: "cluster-123".to_string(),
            postgres_timeline: 3,
            recovery_lsn: "0/16B6C50".to_string(),
            epoch_number: 2,
            expected_previous_epoch_hash: Some(ContentHash([8; 32])),
            anchor_checkpoint: checkpoint(),
            epoch_signer_public_key: key.verifying_key().to_bytes(),
            mode: DatabasePromotionMode::DisasterRecovery,
            requested_recovery_target: Some(DateTime::from_timestamp(1_700_000_000, 0).unwrap()),
            emergency_ceremony: None,
            reason: "loss of primary region".to_string(),
        };
        assert!(intent.validate().is_err());
        intent.emergency_ceremony = Some(EmergencyRecoveryCeremony {
            incident_id: Uuid::new_v4(),
            declared_at: DateTime::from_timestamp(1_700_000_000, 0).unwrap(),
            expires_at: DateTime::from_timestamp(1_700_003_600, 0).unwrap(),
            incident_summary: "primary region unavailable".to_string(),
            recovery_objective: "restore canonical authority".to_string(),
            acknowledged_data_loss_window: "at most 30 seconds".to_string(),
            coordination_channel: "incident:2026-08-05".to_string(),
            named_participants: vec![
                ActorId::new("did:key:admin-a").unwrap(),
                ActorId::new("did:key:admin-b").unwrap(),
            ],
        });
        assert!(intent.validate().is_ok());
    }
}
