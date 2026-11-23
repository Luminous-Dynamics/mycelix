//! Domain-separated, signed authority proof receipts.
//!
//! This crate models the external proofs that sit beside a signed write lease:
//! deployment high-water advancement, physical fencing and promotion eligibility.
//! It deliberately contains no database client, network client, async runtime or
//! application dependency. Trust-root selection and minimum-sequence state are
//! supplied by the verifier from outside the database rollback domain.

use chrono::{DateTime, Duration, Utc};
use ed25519_dalek::{Signature, Verifier, VerifyingKey};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use uuid::Uuid;

pub const AUTHORITY_PROOF_PROTOCOL: &str = "mycelix-authority-proof";
pub const AUTHORITY_PROOF_VERSION: u16 = 1;
pub const MAX_PROOF_LIFETIME_SECONDS: i64 = 10 * 60;
pub type ProofResult<T> = Result<T, String>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ProofDigest(pub [u8; 32]);

impl ProofDigest {
    pub fn digest(bytes: &[u8]) -> Self {
        Self(*blake3::hash(bytes).as_bytes())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthorityProofPurpose {
    LeaseHighWater,
    PhysicalFencing,
    PromotionEligibility,
}

impl AuthorityProofPurpose {
    pub const fn code(self) -> u8 {
        match self {
            Self::LeaseHighWater => 1,
            Self::PhysicalFencing => 2,
            Self::PromotionEligibility => 3,
        }
    }

    const fn domain(self) -> &'static [u8] {
        match self {
            Self::LeaseHighWater => b"MYCELIX-AUTHORITY-PROOF-LEASE-HIGHWATER\0V1",
            Self::PhysicalFencing => b"MYCELIX-AUTHORITY-PROOF-PHYSICAL-FENCING\0V1",
            Self::PromotionEligibility => b"MYCELIX-AUTHORITY-PROOF-PROMOTION-ELIGIBILITY\0V1",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthorityProofIssuerRole {
    LeaseHighWaterAuthority,
    PhysicalFencingAttestor,
    PromotionController,
}

impl AuthorityProofIssuerRole {
    pub const fn code(self) -> u8 {
        match self {
            Self::LeaseHighWaterAuthority => 1,
            Self::PhysicalFencingAttestor => 2,
            Self::PromotionController => 3,
        }
    }

    pub const fn permitted_purpose(self) -> AuthorityProofPurpose {
        match self {
            Self::LeaseHighWaterAuthority => AuthorityProofPurpose::LeaseHighWater,
            Self::PhysicalFencingAttestor => AuthorityProofPurpose::PhysicalFencing,
            Self::PromotionController => AuthorityProofPurpose::PromotionEligibility,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FencingMechanism {
    /// Network isolation alone is explicitly not a physical fence.
    NetworkPartitionOnly,
    PowerOff,
    HypervisorFence,
    CloudInstanceDeallocation,
    StorageLeaseRevocation,
    QuorumLeaseRevocation,
    HardwareWatchdog,
    OtherIndependentlyAttested,
}

impl FencingMechanism {
    const fn code(self) -> u8 {
        match self {
            Self::NetworkPartitionOnly => 0,
            Self::PowerOff => 1,
            Self::HypervisorFence => 2,
            Self::CloudInstanceDeallocation => 3,
            Self::StorageLeaseRevocation => 4,
            Self::QuorumLeaseRevocation => 5,
            Self::HardwareWatchdog => 6,
            Self::OtherIndependentlyAttested => 255,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum AuthorityProofPayload {
    LeaseHighWater {
        previous_generation: u64,
        previous_lease_hash: ProofDigest,
        new_generation: u64,
        new_lease_id: Uuid,
        new_lease_hash: ProofDigest,
        authority_policy_epoch: u64,
    },
    PhysicalFencing {
        old_primary_id: String,
        old_system_identifier: String,
        old_timeline: u64,
        owner_epoch: u64,
        fence_sequence: u64,
        mechanism: FencingMechanism,
        actuator_id: String,
        operation_id: String,
        successor_primary_id: String,
        observed_at: DateTime<Utc>,
        actuator_evidence_hash: ProofDigest,
    },
    PromotionEligibility {
        candidate_primary_id: String,
        candidate_system_identifier: String,
        candidate_timeline: u64,
        target_transaction_end_lsn: u64,
        candidate_flush_lsn: u64,
        candidate_replay_lsn: u64,
        timeline_ancestry_hash: ProofDigest,
        promotion_policy_epoch: u64,
        lease_highwater_proof_hash: ProofDigest,
        physical_fencing_proof_hash: ProofDigest,
    },
}

impl AuthorityProofPayload {
    pub const fn purpose(&self) -> AuthorityProofPurpose {
        match self {
            Self::LeaseHighWater { .. } => AuthorityProofPurpose::LeaseHighWater,
            Self::PhysicalFencing { .. } => AuthorityProofPurpose::PhysicalFencing,
            Self::PromotionEligibility { .. } => AuthorityProofPurpose::PromotionEligibility,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AuthorityProofReceipt {
    pub proof_id: Uuid,
    pub purpose: AuthorityProofPurpose,
    pub issuer_role: AuthorityProofIssuerRole,
    pub deployment_id: String,
    pub database_system_identifier: String,
    pub primary_id: String,
    pub postgres_timeline: u64,
    pub owner_epoch: u64,
    pub sequence: u64,
    pub issued_at: DateTime<Utc>,
    pub not_before: DateTime<Utc>,
    pub expires_at: DateTime<Utc>,
    pub payload: AuthorityProofPayload,
}

impl AuthorityProofReceipt {
    pub fn validate(&self) -> ProofResult<()> {
        validate_text(&self.deployment_id, "deployment ID")?;
        validate_text(
            &self.database_system_identifier,
            "database system identifier",
        )?;
        validate_text(&self.primary_id, "primary ID")?;

        if self.postgres_timeline == 0 || self.owner_epoch == 0 || self.sequence == 0 {
            return Err(
                "authority proof timeline, owner epoch and sequence must be positive".into(),
            );
        }
        if self.purpose != self.payload.purpose()
            || self.issuer_role.permitted_purpose() != self.purpose
        {
            return Err("authority proof purpose and issuer role are not compatible".into());
        }
        if self.not_before < self.issued_at
            || self.expires_at <= self.not_before
            || self.expires_at - self.not_before > Duration::seconds(MAX_PROOF_LIFETIME_SECONDS)
        {
            return Err(
                "authority proof validity interval is invalid or exceeds ten minutes".into(),
            );
        }

        match &self.payload {
            AuthorityProofPayload::LeaseHighWater {
                previous_generation,
                previous_lease_hash,
                new_generation,
                new_lease_hash,
                authority_policy_epoch,
                ..
            } => {
                let predecessor_invalid = if *previous_generation == 0 {
                    !is_zero(&previous_lease_hash.0)
                } else {
                    is_zero(&previous_lease_hash.0)
                };
                if new_generation <= previous_generation
                    || *new_generation == 0
                    || *authority_policy_epoch == 0
                    || predecessor_invalid
                    || is_zero(&new_lease_hash.0)
                {
                    return Err(
                        "lease high-water proof does not advance a valid generation/hash".into(),
                    );
                }
            }
            AuthorityProofPayload::PhysicalFencing {
                old_primary_id,
                old_system_identifier,
                old_timeline,
                owner_epoch,
                fence_sequence,
                actuator_id,
                operation_id,
                successor_primary_id,
                observed_at,
                actuator_evidence_hash,
                ..
            } => {
                validate_text(old_primary_id, "old primary ID")?;
                validate_text(old_system_identifier, "old system identifier")?;
                validate_text(actuator_id, "fencing actuator ID")?;
                validate_text(operation_id, "fencing operation ID")?;
                validate_text(successor_primary_id, "successor primary ID")?;
                if *old_timeline == 0
                    || *owner_epoch == 0
                    || *fence_sequence == 0
                    || *old_primary_id != self.primary_id
                    || *old_system_identifier != self.database_system_identifier
                    || *old_timeline != self.postgres_timeline
                    || *owner_epoch != self.owner_epoch
                    || old_primary_id == successor_primary_id
                    || *observed_at < self.issued_at
                    || *observed_at > self.expires_at
                    || *mechanism == FencingMechanism::NetworkPartitionOnly
                    || is_zero(&actuator_evidence_hash.0)
                {
                    return Err(
                        "physical fencing proof has invalid target, sequence or actuator evidence"
                            .into(),
                    );
                }
            }
            AuthorityProofPayload::PromotionEligibility {
                candidate_primary_id,
                candidate_system_identifier,
                candidate_timeline,
                target_transaction_end_lsn,
                candidate_flush_lsn,
                candidate_replay_lsn,
                timeline_ancestry_hash,
                promotion_policy_epoch,
                lease_highwater_proof_hash,
                physical_fencing_proof_hash,
            } => {
                validate_text(candidate_primary_id, "candidate primary ID")?;
                validate_text(candidate_system_identifier, "candidate system identifier")?;
                if *candidate_timeline == 0
                    || *candidate_primary_id != self.primary_id
                    || *candidate_system_identifier != self.database_system_identifier
                    || *candidate_timeline != self.postgres_timeline
                    || *target_transaction_end_lsn == 0
                    || *candidate_flush_lsn < *target_transaction_end_lsn
                    || *candidate_replay_lsn < *target_transaction_end_lsn
                    || *promotion_policy_epoch == 0
                    || is_zero(&timeline_ancestry_hash.0)
                    || is_zero(&lease_highwater_proof_hash.0)
                    || is_zero(&physical_fencing_proof_hash.0)
                {
                    return Err(
                        "promotion eligibility proof lacks WAL, timeline or authority coverage"
                            .into(),
                    );
                }
            }
        }
        Ok(())
    }

    pub fn canonical_bytes(&self) -> ProofResult<Vec<u8>> {
        self.validate()?;
        let mut bytes = self.purpose.domain().to_vec();
        bytes.extend_from_slice(self.proof_id.as_bytes());
        bytes.push(self.purpose.code());
        bytes.push(self.issuer_role.code());
        push_text(&mut bytes, &self.deployment_id)?;
        push_text(&mut bytes, &self.database_system_identifier)?;
        push_text(&mut bytes, &self.primary_id)?;
        push_u64(&mut bytes, self.postgres_timeline);
        push_u64(&mut bytes, self.owner_epoch);
        push_u64(&mut bytes, self.sequence);
        push_time(&mut bytes, &self.issued_at);
        push_time(&mut bytes, &self.not_before);
        push_time(&mut bytes, &self.expires_at);
        encode_payload(&mut bytes, &self.payload)?;
        Ok(bytes)
    }

    pub fn digest(&self) -> ProofResult<ProofDigest> {
        Ok(ProofDigest::digest(&self.canonical_bytes()?))
    }

    pub fn validate_at(&self, now: DateTime<Utc>, clock_skew: Duration) -> ProofResult<()> {
        self.validate()?;
        if clock_skew < Duration::zero() || clock_skew > Duration::minutes(5) {
            return Err("authority proof clock skew must be between zero and five minutes".into());
        }
        if now + clock_skew < self.not_before {
            return Err("authority proof is not active yet".into());
        }
        if now >= self.expires_at {
            return Err("authority proof has expired".into());
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignedAuthorityProofReceipt {
    pub protocol: String,
    pub version: u16,
    pub receipt: AuthorityProofReceipt,
    pub issuer_key_id: String,
    pub issuer_public_key: [u8; 32],
    pub signature: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthorityProofTarget {
    pub deployment_id: String,
    pub database_system_identifier: String,
    pub primary_id: String,
    pub postgres_timeline: u64,
    pub owner_epoch: u64,
    /// Required for physical fencing: the exact successor the old primary
    /// must be fenced away from.
    pub successor_primary_id: Option<String>,
    /// If set, a fencing attestor must be different from the database writer.
    pub database_writer_public_key: Option<[u8; 32]>,
    /// Optional exact predecessor state for a high-water proof.
    pub expected_previous_highwater: Option<(u64, ProofDigest)>,
    /// Optional exact successor lease being admitted by a high-water proof.
    pub expected_new_lease: Option<(u64, Uuid, ProofDigest)>,
    /// Optional exact transaction target for promotion eligibility.
    pub expected_transaction_end_lsn: Option<u64>,
    pub expected_timeline_ancestry_hash: Option<ProofDigest>,
    pub expected_highwater_proof_hash: Option<ProofDigest>,
    pub expected_physical_fencing_proof_hash: Option<ProofDigest>,
}

#[derive(Debug, Clone)]
pub struct AuthorityProofTrustPolicy {
    /// Trust is keyed by the signed key ID; public keys are not learned from SQL.
    pub trusted_issuer_keys: std::collections::BTreeMap<String, [u8; 32]>,
    pub required_issuer_role: AuthorityProofIssuerRole,
    /// Last accepted sequence for this issuer/purpose/deployment domain.
    /// New proofs must strictly advance it.
    pub last_accepted_sequence: u64,
    pub clock_skew: Duration,
}

impl SignedAuthorityProofReceipt {
    pub fn signing_bytes(&self) -> ProofResult<Vec<u8>> {
        validate_text(&self.issuer_key_id, "proof issuer key ID")?;
        let mut bytes = b"MYCELIX-SIGNED-AUTHORITY-PROOF-RECEIPT\0V1".to_vec();
        push_text(&mut bytes, &self.protocol)?;
        push_u16(&mut bytes, self.version);
        push_bytes(&mut bytes, &self.receipt.canonical_bytes()?)?;
        push_text(&mut bytes, &self.issuer_key_id)?;
        bytes.extend_from_slice(&self.issuer_public_key);
        Ok(bytes)
    }

    pub fn verify_signature(&self) -> ProofResult<()> {
        if self.protocol != AUTHORITY_PROOF_PROTOCOL || self.version != AUTHORITY_PROOF_VERSION {
            return Err("unsupported authority proof envelope".into());
        }
        self.receipt.validate()?;
        let key = VerifyingKey::from_bytes(&self.issuer_public_key)
            .map_err(|error| format!("invalid authority proof public key: {error}"))?;
        let signature = Signature::try_from(self.signature.as_slice())
            .map_err(|error| format!("invalid authority proof signature encoding: {error}"))?;
        key.verify(&self.signing_bytes()?, &signature)
            .map_err(|error| format!("authority proof signature verification failed: {error}"))
    }

    pub fn verify_for_target(
        &self,
        policy: &AuthorityProofTrustPolicy,
        required_purpose: AuthorityProofPurpose,
        target: &AuthorityProofTarget,
        now: DateTime<Utc>,
    ) -> ProofResult<ProofDigest> {
        self.verify_signature()?;
        self.receipt.validate_at(now, policy.clock_skew)?;
        if self.receipt.purpose != required_purpose
            || self.receipt.issuer_role != policy.required_issuer_role
            || self.receipt.issuer_role.permitted_purpose() != required_purpose
        {
            return Err("authority proof purpose or issuer role does not satisfy policy".into());
        }
        if policy.trusted_issuer_keys.is_empty()
            || policy.trusted_issuer_keys.get(&self.issuer_key_id) != Some(&self.issuer_public_key)
        {
            return Err("authority proof key ID/public key is not in the configured trust set".into());
        }
        if self.receipt.sequence <= policy.last_accepted_sequence {
            return Err("authority proof sequence does not advance the external anti-replay high-water".into());
        }
        if self.receipt.deployment_id != target.deployment_id
            || self.receipt.database_system_identifier != target.database_system_identifier
            || self.receipt.primary_id != target.primary_id
            || self.receipt.postgres_timeline != target.postgres_timeline
            || self.receipt.owner_epoch != target.owner_epoch
        {
            return Err("authority proof does not bind the exact deployment, database, primary, timeline and owner epoch".into());
        }

        match &self.receipt.payload {
            AuthorityProofPayload::LeaseHighWater {
                previous_generation,
                previous_lease_hash,
                new_generation,
                new_lease_id,
                new_lease_hash,
                ..
            } => {
                let (expected_generation, expected_hash) = target
                    .expected_previous_highwater
                    .ok_or_else(|| "lease high-water verification requires an externally supplied predecessor".to_owned())?;
                if *previous_generation != expected_generation
                    || *previous_lease_hash != expected_hash
                {
                    return Err("lease high-water proof does not extend the verifier's exact external predecessor".into());
                }
                let (expected_generation, expected_id, expected_hash) = target
                    .expected_new_lease
                    .ok_or_else(|| "lease high-water verification requires the exact pending successor lease".to_owned())?;
                if *new_generation != expected_generation
                    || *new_lease_id != expected_id
                    || *new_lease_hash != expected_hash
                {
                    return Err("lease high-water proof does not authorize the exact successor lease".into());
                }
            }
            AuthorityProofPayload::PhysicalFencing {
                old_primary_id,
                old_system_identifier,
                old_timeline,
                owner_epoch,
                successor_primary_id,
                ..
            } => {
                if old_primary_id != &self.receipt.primary_id
                    || old_system_identifier != &self.receipt.database_system_identifier
                    || *old_timeline != self.receipt.postgres_timeline
                    || *owner_epoch != self.receipt.owner_epoch
                {
                    return Err("physical fencing proof payload does not match its signed outer target".into());
                }
                let expected_successor = target
                    .successor_primary_id
                    .as_deref()
                    .ok_or_else(|| "physical fencing verification requires an exact successor primary".to_owned())?;
                if expected_successor != successor_primary_id {
                    return Err("physical fencing proof does not bind the exact expected successor primary".into());
                }
                let database_writer_key = target
                    .database_writer_public_key
                    .ok_or_else(|| "physical fencing verification requires the database writer key for independence checks".to_owned())?;
                if database_writer_key == self.issuer_public_key {
                    return Err("physical fencing attestor must be independent from the database writer key".into());
                }
            }
            AuthorityProofPayload::PromotionEligibility {
                candidate_primary_id,
                candidate_system_identifier,
                candidate_timeline,
                target_transaction_end_lsn,
                timeline_ancestry_hash,
                lease_highwater_proof_hash,
                physical_fencing_proof_hash,
                ..
            } => {
                if candidate_primary_id != &self.receipt.primary_id
                    || candidate_system_identifier != &self.receipt.database_system_identifier
                    || *candidate_timeline != self.receipt.postgres_timeline
                {
                    return Err("promotion eligibility payload does not match its signed candidate identity".into());
                }
                let expected_lsn = target
                    .expected_transaction_end_lsn
                    .ok_or_else(|| "promotion proof verification requires the exact acknowledged transaction end-LSN".to_owned())?;
                let expected_ancestry = target
                    .expected_timeline_ancestry_hash
                    .ok_or_else(|| "promotion proof verification requires the exact timeline ancestry commitment".to_owned())?;
                let expected_highwater = target
                    .expected_highwater_proof_hash
                    .ok_or_else(|| "promotion proof verification requires the exact verified high-water proof".to_owned())?;
                let expected_fencing = target
                    .expected_physical_fencing_proof_hash
                    .ok_or_else(|| "promotion proof verification requires the exact verified physical-fencing proof".to_owned())?;
                if expected_lsn != *target_transaction_end_lsn
                    || expected_ancestry != *timeline_ancestry_hash
                    || expected_highwater != *lease_highwater_proof_hash
                    || expected_fencing != *physical_fencing_proof_hash
                {
                    return Err("promotion eligibility proof does not bind the expected effect/high-water/fencing evidence".into());
                }
            }
        }
        Ok(self.receipt.digest()?)
    }

    pub fn signed_receipt_hash(&self) -> ProofResult<ProofDigest> {
        self.verify_signature()?;
        let mut bytes = self.signing_bytes()?;
        push_bytes(&mut bytes, &self.signature)?;
        Ok(ProofDigest::digest(&bytes))
    }
}

fn encode_payload(bytes: &mut Vec<u8>, payload: &AuthorityProofPayload) -> ProofResult<()> {
    match payload {
        AuthorityProofPayload::LeaseHighWater {
            previous_generation,
            previous_lease_hash,
            new_generation,
            new_lease_id,
            new_lease_hash,
            authority_policy_epoch,
        } => {
            bytes.push(1);
            push_u64(bytes, *previous_generation);
            bytes.extend_from_slice(&previous_lease_hash.0);
            push_u64(bytes, *new_generation);
            bytes.extend_from_slice(new_lease_id.as_bytes());
            bytes.extend_from_slice(&new_lease_hash.0);
            push_u64(bytes, *authority_policy_epoch);
        }
        AuthorityProofPayload::PhysicalFencing {
            old_primary_id,
            old_system_identifier,
            old_timeline,
            owner_epoch,
            fence_sequence,
            mechanism,
            actuator_id,
            operation_id,
            successor_primary_id,
            observed_at,
            actuator_evidence_hash,
        } => {
            bytes.push(2);
            push_text(bytes, old_primary_id)?;
            push_text(bytes, old_system_identifier)?;
            push_u64(bytes, *old_timeline);
            push_u64(bytes, *owner_epoch);
            push_u64(bytes, *fence_sequence);
            bytes.push(mechanism.code());
            push_text(bytes, actuator_id)?;
            push_text(bytes, operation_id)?;
            push_text(bytes, successor_primary_id)?;
            push_time(bytes, observed_at);
            bytes.extend_from_slice(&actuator_evidence_hash.0);
        }
        AuthorityProofPayload::PromotionEligibility {
            candidate_primary_id,
            candidate_system_identifier,
            candidate_timeline,
            target_transaction_end_lsn,
            candidate_flush_lsn,
            candidate_replay_lsn,
            timeline_ancestry_hash,
            promotion_policy_epoch,
            lease_highwater_proof_hash,
            physical_fencing_proof_hash,
        } => {
            bytes.push(3);
            push_text(bytes, candidate_primary_id)?;
            push_text(bytes, candidate_system_identifier)?;
            push_u64(bytes, *candidate_timeline);
            push_u64(bytes, *target_transaction_end_lsn);
            push_u64(bytes, *candidate_flush_lsn);
            push_u64(bytes, *candidate_replay_lsn);
            bytes.extend_from_slice(&timeline_ancestry_hash.0);
            push_u64(bytes, *promotion_policy_epoch);
            bytes.extend_from_slice(&lease_highwater_proof_hash.0);
            bytes.extend_from_slice(&physical_fencing_proof_hash.0);
        }
    }
    Ok(())
}

fn validate_text(value: &str, label: &str) -> ProofResult<()> {
    if value.trim().is_empty() || value.len() > 512 || value.chars().any(char::is_control) {
        return Err(format!("{label} must contain 1-512 printable bytes"));
    }
    Ok(())
}

fn is_zero(value: &[u8; 32]) -> bool {
    value.iter().all(|byte| *byte == 0)
}

fn push_text(bytes: &mut Vec<u8>, value: &str) -> ProofResult<()> {
    push_bytes(bytes, value.as_bytes())
}

fn push_bytes(bytes: &mut Vec<u8>, value: &[u8]) -> ProofResult<()> {
    let length = u32::try_from(value.len())
        .map_err(|_| "authority proof field exceeds canonical u32 length".to_owned())?;
    bytes.extend_from_slice(&length.to_be_bytes());
    bytes.extend_from_slice(value);
    Ok(())
}

fn push_u16(bytes: &mut Vec<u8>, value: u16) {
    bytes.extend_from_slice(&value.to_be_bytes());
}

fn push_u64(bytes: &mut Vec<u8>, value: u64) {
    bytes.extend_from_slice(&value.to_be_bytes());
}

fn push_time(bytes: &mut Vec<u8>, value: &DateTime<Utc>) {
    bytes.extend_from_slice(&value.timestamp().to_be_bytes());
    bytes.extend_from_slice(&value.timestamp_subsec_nanos().to_be_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::TimeZone;
    use ed25519_dalek::{Signer, SigningKey};

    fn common(
        purpose: AuthorityProofPurpose,
        role: AuthorityProofIssuerRole,
        payload: AuthorityProofPayload,
    ) -> AuthorityProofReceipt {
        let now = Utc.with_ymd_and_hms(2026, 11, 23, 10, 0, 0).unwrap();
        AuthorityProofReceipt {
            proof_id: Uuid::from_bytes([0x11; 16]),
            purpose,
            issuer_role: role,
            deployment_id: "deployment:test".into(),
            database_system_identifier: "731245".into(),
            primary_id: "postgres-a".into(),
            postgres_timeline: 2,
            owner_epoch: 3,
            sequence: 9,
            issued_at: now,
            not_before: now + Duration::seconds(1),
            expires_at: now + Duration::minutes(5),
            payload,
        }
    }

    fn sign(receipt: AuthorityProofReceipt, key: &SigningKey) -> SignedAuthorityProofReceipt {
        let mut signed = SignedAuthorityProofReceipt {
            protocol: AUTHORITY_PROOF_PROTOCOL.into(),
            version: AUTHORITY_PROOF_VERSION,
            receipt,
            issuer_key_id: "fixture-proof-key".into(),
            issuer_public_key: key.verifying_key().to_bytes(),
            signature: Vec::new(),
        };
        signed.signature = key
            .sign(&signed.signing_bytes().unwrap())
            .to_bytes()
            .to_vec();
        signed
    }

    #[test]
    fn authority_proof_purposes_and_issuer_roles_are_domain_separated() {
        let key = SigningKey::from_bytes(&[84; 32]);
        let mut receipt = common(
            AuthorityProofPurpose::PhysicalFencing,
            AuthorityProofIssuerRole::PhysicalFencingAttestor,
            AuthorityProofPayload::PhysicalFencing {
                old_primary_id: "postgres-old".into(),
                old_system_identifier: "731245".into(),
                old_timeline: 2,
                owner_epoch: 3,
                fence_sequence: 9,
                mechanism: FencingMechanism::HypervisorFence,
                actuator_id: "hypervisor-controller-a".into(),
                operation_id: "fence-op-9".into(),
                successor_primary_id: "postgres-new".into(),
                observed_at: Utc.with_ymd_and_hms(2026, 11, 23, 10, 0, 2).unwrap(),
                actuator_evidence_hash: ProofDigest([7; 32]),
            },
        );
        receipt.primary_id = "postgres-old".into();
        let signed = sign(receipt, &key);
        signed.verify_signature().unwrap();

        let policy = AuthorityProofTrustPolicy {
            trusted_issuer_keys: [("fixture-proof-key".to_owned(), key.verifying_key().to_bytes())]
                .into_iter()
                .collect(),
            required_issuer_role: AuthorityProofIssuerRole::PhysicalFencingAttestor,
            last_accepted_sequence: 8,
            clock_skew: Duration::zero(),
        };
        let target = AuthorityProofTarget {
            deployment_id: "deployment:test".into(),
            database_system_identifier: "731245".into(),
            primary_id: "postgres-old".into(),
            postgres_timeline: 2,
            owner_epoch: 3,
            successor_primary_id: Some("postgres-new".into()),
            database_writer_public_key: Some([85; 32]),
            expected_previous_highwater: None,
            expected_new_lease: None,
            expected_transaction_end_lsn: None,
            expected_timeline_ancestry_hash: None,
            expected_highwater_proof_hash: None,
            expected_physical_fencing_proof_hash: None,
        };
        signed
            .verify_for_target(
                &policy,
                AuthorityProofPurpose::PhysicalFencing,
                &target,
                Utc.with_ymd_and_hms(2026, 11, 23, 10, 0, 3).unwrap(),
            )
            .unwrap();
        assert!(signed
            .verify_for_target(
                &policy,
                AuthorityProofPurpose::LeaseHighWater,
                &target,
                Utc.with_ymd_and_hms(2026, 11, 23, 10, 0, 3).unwrap(),
            )
            .is_err());

        let wrong_role = AuthorityProofTrustPolicy {
            required_issuer_role: AuthorityProofIssuerRole::LeaseHighWaterAuthority,
            ..policy
        };
        assert!(signed
            .verify_for_target(
                &wrong_role,
                AuthorityProofPurpose::PhysicalFencing,
                &target,
                Utc.with_ymd_and_hms(2026, 11, 23, 10, 0, 3).unwrap(),
            )
            .is_err());
    }

    #[test]
    fn highwater_proof_must_extend_exact_external_predecessor_and_lease() {
        let key = SigningKey::from_bytes(&[86; 32]);
        let new_lease_id = Uuid::from_bytes([9; 16]);
        let new_lease_hash = ProofDigest([9; 32]);
        let receipt = common(
            AuthorityProofPurpose::LeaseHighWater,
            AuthorityProofIssuerRole::LeaseHighWaterAuthority,
            AuthorityProofPayload::LeaseHighWater {
                previous_generation: 8,
                previous_lease_hash: ProofDigest([8; 32]),
                new_generation: 9,
                new_lease_id,
                new_lease_hash,
                authority_policy_epoch: 2,
            },
        );
        let signed = sign(receipt, &key);
        let policy = AuthorityProofTrustPolicy {
            trusted_issuer_keys: [("fixture-proof-key".to_owned(), key.verifying_key().to_bytes())]
                .into_iter()
                .collect(),
            required_issuer_role: AuthorityProofIssuerRole::LeaseHighWaterAuthority,
            last_accepted_sequence: 8,
            clock_skew: Duration::zero(),
        };
        let target = AuthorityProofTarget {
            deployment_id: "deployment:test".into(),
            database_system_identifier: "731245".into(),
            primary_id: "postgres-a".into(),
            postgres_timeline: 2,
            owner_epoch: 3,
            successor_primary_id: None,
            database_writer_public_key: None,
            expected_previous_highwater: Some((8, ProofDigest([8; 32]))),
            expected_new_lease: Some((9, new_lease_id, new_lease_hash)),
            expected_transaction_end_lsn: None,
            expected_highwater_proof_hash: None,
            expected_physical_fencing_proof_hash: None,
        };
        signed
            .verify_for_target(
                &policy,
                AuthorityProofPurpose::LeaseHighWater,
                &target,
                Utc.with_ymd_and_hms(2026, 11, 23, 10, 0, 2).unwrap(),
            )
            .unwrap();

        let wrong_predecessor = AuthorityProofTarget {
            expected_previous_highwater: Some((7, ProofDigest([7; 32]))),
            ..target
        };
        assert!(signed
            .verify_for_target(
                &policy,
                AuthorityProofPurpose::LeaseHighWater,
                &wrong_predecessor,
                Utc.with_ymd_and_hms(2026, 11, 23, 10, 0, 2).unwrap(),
            )
            .is_err());
    }

    #[test]
    fn physical_fencing_requires_independent_attestor_and_exact_successor() {
        let key = SigningKey::from_bytes(&[87; 32]);
        let receipt = common(
            AuthorityProofPurpose::PhysicalFencing,
            AuthorityProofIssuerRole::PhysicalFencingAttestor,
            AuthorityProofPayload::PhysicalFencing {
                old_primary_id: "postgres-a".into(),
                old_system_identifier: "731245".into(),
                old_timeline: 2,
                owner_epoch: 3,
                fence_sequence: 12,
                mechanism: FencingMechanism::StorageLeaseRevocation,
                actuator_id: "storage-controller-1".into(),
                operation_id: "storage-fence-12".into(),
                successor_primary_id: "postgres-b".into(),
                observed_at: Utc.with_ymd_and_hms(2026, 11, 23, 10, 0, 2).unwrap(),
                actuator_evidence_hash: ProofDigest([12; 32]),
            },
        );
        let signed = sign(receipt, &key);
        let policy = AuthorityProofTrustPolicy {
            trusted_issuer_keys: [("fixture-proof-key".to_owned(), key.verifying_key().to_bytes())]
                .into_iter()
                .collect(),
            required_issuer_role: AuthorityProofIssuerRole::PhysicalFencingAttestor,
            last_accepted_sequence: 11,
            clock_skew: Duration::zero(),
        };
        let target = AuthorityProofTarget {
            deployment_id: "deployment:test".into(),
            database_system_identifier: "731245".into(),
            primary_id: "postgres-a".into(),
            postgres_timeline: 2,
            owner_epoch: 3,
            successor_primary_id: Some("postgres-b".into()),
            database_writer_public_key: Some([88; 32]),
            expected_previous_highwater: None,
            expected_new_lease: None,
            expected_transaction_end_lsn: None,
            expected_highwater_proof_hash: None,
            expected_physical_fencing_proof_hash: None,
        };
        signed
            .verify_for_target(
                &policy,
                AuthorityProofPurpose::PhysicalFencing,
                &target,
                Utc.with_ymd_and_hms(2026, 11, 23, 10, 0, 3).unwrap(),
            )
            .unwrap();

        let same_writer_key = AuthorityProofTarget {
            database_writer_public_key: Some(key.verifying_key().to_bytes()),
            ..target.clone()
        };
        assert!(signed
            .verify_for_target(
                &policy,
                AuthorityProofPurpose::PhysicalFencing,
                &same_writer_key,
                Utc.with_ymd_and_hms(2026, 11, 23, 10, 0, 3).unwrap(),
            )
            .is_err());

        let wrong_successor = AuthorityProofTarget {
            successor_primary_id: Some("postgres-c".into()),
            ..target
        };
        assert!(signed
            .verify_for_target(
                &policy,
                AuthorityProofPurpose::PhysicalFencing,
                &wrong_successor,
                Utc.with_ymd_and_hms(2026, 11, 23, 10, 0, 3).unwrap(),
            )
            .is_err());
    }

    #[test]
    fn promotion_eligibility_binds_exact_end_lsn_and_proof_chain() {
        let key = SigningKey::from_bytes(&[89; 32]);
        let highwater_hash = ProofDigest([41; 32]);
        let fence_hash = ProofDigest([42; 32]);
        let receipt = common(
            AuthorityProofPurpose::PromotionEligibility,
            AuthorityProofIssuerRole::PromotionController,
            AuthorityProofPayload::PromotionEligibility {
                candidate_primary_id: "postgres-b".into(),
                candidate_system_identifier: "731245".into(),
                candidate_timeline: 3,
                target_transaction_end_lsn: 4096,
                candidate_flush_lsn: 8192,
                candidate_replay_lsn: 8192,
                timeline_ancestry_hash: ProofDigest([43; 32]),
                promotion_policy_epoch: 7,
                lease_highwater_proof_hash: highwater_hash,
                physical_fencing_proof_hash: fence_hash,
            },
        );
        let mut receipt = receipt;
        receipt.primary_id = "postgres-b".into();
        receipt.postgres_timeline = 3;
        let signed = sign(receipt, &key);
        let policy = AuthorityProofTrustPolicy {
            trusted_issuer_keys: [("fixture-proof-key".to_owned(), key.verifying_key().to_bytes())]
                .into_iter()
                .collect(),
            required_issuer_role: AuthorityProofIssuerRole::PromotionController,
            last_accepted_sequence: 8,
            clock_skew: Duration::zero(),
        };
        let target = AuthorityProofTarget {
            deployment_id: "deployment:test".into(),
            database_system_identifier: "731245".into(),
            primary_id: "postgres-b".into(),
            postgres_timeline: 3,
            owner_epoch: 3,
            successor_primary_id: None,
            database_writer_public_key: None,
            expected_previous_highwater: None,
            expected_new_lease: None,
            expected_transaction_end_lsn: Some(4096),
            expected_timeline_ancestry_hash: Some(ProofDigest([43; 32])),
            expected_highwater_proof_hash: Some(highwater_hash),
            expected_physical_fencing_proof_hash: Some(fence_hash),
        };
        signed
            .verify_for_target(
                &policy,
                AuthorityProofPurpose::PromotionEligibility,
                &target,
                Utc.with_ymd_and_hms(2026, 11, 23, 10, 0, 3).unwrap(),
            )
            .unwrap();

        let wrong_lsn = AuthorityProofTarget {
            expected_transaction_end_lsn: Some(4097),
            ..target
        };
        assert!(signed
            .verify_for_target(
                &policy,
                AuthorityProofPurpose::PromotionEligibility,
                &wrong_lsn,
                Utc.with_ymd_and_hms(2026, 11, 23, 10, 0, 3).unwrap(),
            )
            .is_err());
    }

    #[test]
    fn network_partition_is_not_accepted_as_physical_fencing() {
        let receipt = common(
            AuthorityProofPurpose::PhysicalFencing,
            AuthorityProofIssuerRole::PhysicalFencingAttestor,
            AuthorityProofPayload::PhysicalFencing {
                old_primary_id: "postgres-old".into(),
                old_system_identifier: "731245".into(),
                old_timeline: 2,
                owner_epoch: 3,
                fence_sequence: 10,
                mechanism: FencingMechanism::NetworkPartitionOnly,
                actuator_id: "network-monitor".into(),
                operation_id: "partition-detected-10".into(),
                successor_primary_id: "postgres-new".into(),
                observed_at: Utc.with_ymd_and_hms(2026, 11, 23, 10, 0, 2).unwrap(),
                actuator_evidence_hash: ProofDigest([10; 32]),
            },
        );
        assert!(receipt.validate().is_err());
    }

    #[test]
    fn stale_sequence_wrong_target_and_expiry_reject() {
        let key = SigningKey::from_bytes(&[84; 32]);
        let now = Utc.with_ymd_and_hms(2026, 11, 23, 10, 0, 0).unwrap();
        let receipt = common(
            AuthorityProofPurpose::LeaseHighWater,
            AuthorityProofIssuerRole::LeaseHighWaterAuthority,
            AuthorityProofPayload::LeaseHighWater {
                previous_generation: 8,
                previous_lease_hash: ProofDigest([8; 32]),
                new_generation: 9,
                new_lease_id: Uuid::from_bytes([9; 16]),
                new_lease_hash: ProofDigest([9; 32]),
                authority_policy_epoch: 2,
            },
        );
        let signed = sign(receipt, &key);
        let policy = AuthorityProofTrustPolicy {
            trusted_issuer_keys: [("fixture-proof-key".to_owned(), key.verifying_key().to_bytes())]
                .into_iter()
                .collect(),
            required_issuer_role: AuthorityProofIssuerRole::LeaseHighWaterAuthority,
            last_accepted_sequence: 9,
            clock_skew: Duration::zero(),
        };
        let target = AuthorityProofTarget {
            deployment_id: "deployment:test".into(),
            database_system_identifier: "731245".into(),
            primary_id: "postgres-a".into(),
            postgres_timeline: 2,
            owner_epoch: 3,
            successor_primary_id: None,
            database_writer_public_key: None,
            expected_previous_highwater: None,
            expected_new_lease: None,
            expected_transaction_end_lsn: None,
            expected_highwater_proof_hash: None,
            expected_physical_fencing_proof_hash: None,
        };
        assert!(signed
            .verify_for_target(
                &policy,
                AuthorityProofPurpose::LeaseHighWater,
                &target,
                now + Duration::seconds(2),
            )
            .is_err());
    }
}
