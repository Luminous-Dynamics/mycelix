//! PostgreSQL-backed verifier for Directory Fabric effect and execution receipts.
//!
//! Reads the append-only effect_commits and effect_execution_attestations rows.
//! Verification proves that the exact historical effect/worker/owner coordinates
//! are visible on the connected writable primary. It does not prove that a
//! split-brain former primary is physically fenced or that asynchronous replicas
//! contain the commit.

use std::path::Path;

use myc_dir_fabric_nats_reference::{Digest, DurableOwnerDisposition, NatsCommandBinding};
use rustls_tokio_postgres::{config_from_ca_cert, MakeRustlsConnect};
use tokio_postgres::{Client, Row};

use crate::{sha256, AdapterResult, OwnerDispositionVerifier, VerifiedOwnerDisposition};

const RECEIPT_DOMAIN: &[u8] = b"MYC-DIR-POSTGRES-EFFECT-EXECUTION-RECEIPT\0\x02";

const SELECT_EFFECT_COMMIT: &str = r#"
SELECT
    pg_catalog.pg_is_in_recovery(),
    current_setting('transaction_read_only')::boolean,
    e.effect_id::text,
    e.realm_id,
    e.resource_id,
    e.action_id,
    e.effect_owner_id,
    e.object_id,
    e.currency_code,
    e.idempotency_key_digest,
    e.logical_operation_id,
    e.request_digest,
    e.decision_receipt_digest,
    e.payload_digest,
    e.accepted_generation,
    e.delta_minor,
    e.balance_after_minor,
    a.attestation_id::text,
    a.owner_epoch,
    a.accepted_generation,
    a.worker_id,
    a.claim_epoch,
    a.claim_proof_digest,
    a.claim_clock_profile_id,
    to_char(a.claim_issued_at AT TIME ZONE 'UTC', 'YYYY-MM-DD"T"HH24:MI:SS.US'),
    to_char(a.claim_expires_at AT TIME ZONE 'UTC', 'YYYY-MM-DD"T"HH24:MI:SS.US'),
    a.database_xact_id,
    t.previous_owner_epoch,
    t.new_owner_epoch,
    t.previous_generation,
    t.new_generation,
    t.transition_xact_id,
    t.epoch_authority_receipt_digest,
    t.physical_fencing_receipt_digest
FROM myc_effect_owner.effect_commits e
JOIN myc_effect_owner.effect_execution_attestations a
  ON a.effect_id = e.effect_id
LEFT JOIN myc_effect_owner.effect_owner_epoch_transitions t
  ON t.realm_id = a.realm_id
 AND t.resource_id = a.resource_id
 AND t.action_id = a.action_id
 AND t.effect_owner_id = a.effect_owner_id
 AND t.new_owner_epoch = a.owner_epoch
WHERE e.realm_id = $1
  AND e.resource_id = $2
  AND e.action_id = $3
  AND e.effect_owner_id = $4
  AND e.object_id = $5
  AND e.currency_code = $6
  AND e.idempotency_key_digest = $7
  AND e.logical_operation_id = $8
"#;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VerifiedOwnerEpochTransition {
    pub previous_owner_epoch: u64,
    pub new_owner_epoch: u64,
    pub previous_generation: u64,
    pub new_generation: u64,
    pub transition_xact_id: String,
    pub epoch_authority_receipt_digest: Digest,
    pub physical_fencing_receipt_digest: Digest,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VerifiedPostgresEffectReceipt {
    pub effect_id: String,
    pub attestation_id: String,
    pub realm_id: String,
    pub resource_id: String,
    pub action_id: String,
    pub effect_owner_id: String,
    pub object_id: String,
    pub currency_code: String,
    pub logical_operation_id: String,
    pub owner_epoch: u64,
    pub accepted_generation: u64,
    pub worker_id: String,
    pub claim_epoch: u64,
    pub claim_proof_digest: Digest,
    pub claim_clock_profile_id: String,
    pub claim_issued_at: String,
    pub claim_expires_at: String,
    pub database_xact_id: String,
    pub epoch_transition: Option<VerifiedOwnerEpochTransition>,
    pub delta_minor: i64,
    pub balance_after_minor: i64,
    pub idempotency_digest: Digest,
    pub request_digest: Digest,
    pub decision_receipt_digest: Digest,
    pub payload_digest: Digest,
    pub effect_receipt_digest: Digest,
}

pub struct PostgresEffectCommitVerifier {
    client: Client,
}

impl PostgresEffectCommitVerifier {
    /// Connect using a caller-provided CA bundle. Certificate chain and host
    /// verification remain enabled by rustls-tokio-postgres.
    ///
    /// The DSN must use the DNS name present in the PostgreSQL server
    /// certificate and a writable primary endpoint. Do not use sslmode=disable
    /// or an unverified TLS connector in production.
    pub async fn connect_with_ca(
        dsn: &str,
        ca_certificate: impl AsRef<Path>,
    ) -> Result<Self, String> {
        let tls_config = config_from_ca_cert(ca_certificate)
            .map_err(|error| format!("load PostgreSQL CA certificate: {error}"))?;
        let tls = MakeRustlsConnect::new(tls_config);
        let (client, connection) = tokio_postgres::connect(dsn, tls)
            .await
            .map_err(|error| format!("connect to PostgreSQL effect owner: {error}"))?;

        tokio::spawn(async move {
            if let Err(error) = connection.await {
                eprintln!("PostgreSQL effect-owner connection terminated: {error}");
            }
        });

        Ok(Self { client })
    }

    /// Query the append-only effect and execution-attestation ledgers and
    /// return a verified receipt capability. The digest is recomputed from
    /// actual database fields, including owner epoch and historical worker claim.
    pub async fn verify_effect_commit(
        &self,
        binding: &NatsCommandBinding,
    ) -> AdapterResult<VerifiedOwnerDisposition> {
        let receipt = self
            .load_effect_receipt(binding)
            .await
            .map_err(crate::AdapterError::OwnerDispositionVerification)?;

        let disposition = DurableOwnerDisposition::EffectCommitted {
            logical_operation_id: binding.logical_operation_id().to_owned(),
            effect_owner_id: binding.effect_owner_id().to_owned(),
            idempotency_digest: *binding.idempotency_digest(),
            effect_receipt_digest: receipt.effect_receipt_digest,
        };

        VerifiedOwnerDisposition::from_verified_binding(binding, disposition)
    }

    /// Checks primary/read-only state and fetches the exact effect and
    /// execution-attestation rows in one SQL statement snapshot.
    pub async fn load_effect_receipt(
        &self,
        binding: &NatsCommandBinding,
    ) -> Result<VerifiedPostgresEffectReceipt, String> {
        let idempotency_bytes: &[u8] = binding.idempotency_digest();
        let row = self
            .client
            .query_opt(
                SELECT_EFFECT_COMMIT,
                &[
                    &binding.realm_id(),
                    &binding.resource_id(),
                    &binding.action_id(),
                    &binding.effect_owner_id(),
                    &binding.object_id(),
                    &binding.currency_code(),
                    &idempotency_bytes,
                    &binding.logical_operation_id(),
                ],
            )
            .await
            .map_err(|error| format!("query append-only PostgreSQL effect and attestation ledgers: {error}"))?
            .ok_or_else(|| {
                "no committed PostgreSQL effect plus execution attestation matches the exact command scope".to_owned()
            })?;

        let in_recovery: bool = row.get(0);
        let transaction_read_only: bool = row.get(1);
        if in_recovery {
            return Err("PostgreSQL receipt verifier connected to a recovery/read replica".into());
        }
        if transaction_read_only {
            return Err("PostgreSQL receipt verifier session is read-only".into());
        }

        let receipt = decode_effect_receipt(&row)?;
        validate_receipt_binding(binding, &receipt)?;
        Ok(receipt)
    }
}

#[async_trait::async_trait]
impl OwnerDispositionVerifier for PostgresEffectCommitVerifier {
    async fn verify_durable_disposition(
        &self,
        binding: &NatsCommandBinding,
        disposition: &DurableOwnerDisposition,
    ) -> Result<(), String> {
        let DurableOwnerDisposition::EffectCommitted {
            logical_operation_id,
            effect_owner_id,
            idempotency_digest,
            effect_receipt_digest,
        } = disposition
        else {
            return Err(
                "the PostgreSQL reference ledger currently proves committed effects only; \
                 terminal rejection/no-effect and reconciliation require separate durable ledgers"
                    .into(),
            );
        };

        if logical_operation_id != binding.logical_operation_id()
            || effect_owner_id != binding.effect_owner_id()
            || idempotency_digest != binding.idempotency_digest()
        {
            return Err(
                "effect disposition operation, owner or idempotency binding differs from command"
                    .into(),
            );
        }

        let receipt = self.load_effect_receipt(binding).await?;
        if effect_receipt_digest != &receipt.effect_receipt_digest {
            return Err(
                "effect receipt digest differs from the canonical PostgreSQL effect and execution-attestation rows"
                    .into(),
            );
        }
        Ok(())
    }
}

fn decode_effect_receipt(row: &Row) -> Result<VerifiedPostgresEffectReceipt, String> {
    let effect_id: String = row.get(2);
    let realm_id: String = row.get(3);
    let resource_id: String = row.get(4);
    let action_id: String = row.get(5);
    let effect_owner_id: String = row.get(6);
    let object_id: String = row.get(7);
    let currency_code: String = row.get(8);
    let idempotency_digest = digest_column(row, 9, "idempotency_key_digest")?;
    let logical_operation_id: String = row.get(10);
    let request_digest = digest_column(row, 11, "request_digest")?;
    let decision_receipt_digest = digest_column(row, 12, "decision_receipt_digest")?;
    let payload_digest = digest_column(row, 13, "payload_digest")?;
    let effect_generation_i64: i64 = row.get(14);
    let delta_minor: i64 = row.get(15);
    let balance_after_minor: i64 = row.get(16);
    let attestation_id: String = row.get(17);
    let owner_epoch_i64: i64 = row.get(18);
    let attestation_generation_i64: i64 = row.get(19);
    let worker_id: String = row.get(20);
    let claim_epoch_i64: i64 = row.get(21);
    let claim_proof_digest = digest_column(row, 22, "claim_proof_digest")?;
    let claim_clock_profile_id: String = row.get(23);
    let claim_issued_at: String = row.get(24);
    let claim_expires_at: String = row.get(25);
    let database_xact_id: String = row.get(26);
    let transition_previous_owner_epoch: Option<i64> = row.get(27);
    let transition_new_owner_epoch: Option<i64> = row.get(28);
    let transition_previous_generation: Option<i64> = row.get(29);
    let transition_new_generation: Option<i64> = row.get(30);
    let transition_xact_id: Option<String> = row.get(31);
    let epoch_authority_receipt_digest =
        optional_digest_column(row, 32, "epoch_authority_receipt_digest")?;
    let physical_fencing_receipt_digest =
        optional_digest_column(row, 33, "physical_fencing_receipt_digest")?;

    let accepted_generation = u64::try_from(effect_generation_i64)
        .map_err(|_| "PostgreSQL effect has a nonpositive accepted generation".to_owned())?;
    let attestation_generation = u64::try_from(attestation_generation_i64)
        .map_err(|_| "PostgreSQL execution attestation has a nonpositive generation".to_owned())?;
    let owner_epoch = u64::try_from(owner_epoch_i64)
        .map_err(|_| "PostgreSQL execution attestation has a nonpositive owner epoch".to_owned())?;
    let claim_epoch = u64::try_from(claim_epoch_i64).map_err(|_| {
        "PostgreSQL execution attestation has a nonpositive worker claim epoch".to_owned()
    })?;

    if accepted_generation != attestation_generation {
        return Err(
            "effect_commits and effect_execution_attestations disagree on accepted generation"
                .into(),
        );
    }
    let transition_columns = (
        transition_previous_owner_epoch,
        transition_new_owner_epoch,
        transition_previous_generation,
        transition_new_generation,
        transition_xact_id,
        epoch_authority_receipt_digest,
        physical_fencing_receipt_digest,
    );
    let epoch_transition =
        decode_epoch_transition(owner_epoch, accepted_generation, transition_columns)?;
    if worker_id.trim().is_empty()
        || claim_clock_profile_id.trim().is_empty()
        || database_xact_id.trim().is_empty()
        || claim_issued_at >= claim_expires_at
    {
        return Err("PostgreSQL execution attestation has invalid worker, clock, transaction or claim validity fields".into());
    }

    let effect_receipt_digest = canonical_effect_receipt_digest(&EffectReceiptDigestFields {
        effect_id: &effect_id,
        attestation_id: &attestation_id,
        realm_id: &realm_id,
        resource_id: &resource_id,
        action_id: &action_id,
        effect_owner_id: &effect_owner_id,
        object_id: &object_id,
        currency_code: &currency_code,
        logical_operation_id: &logical_operation_id,
        owner_epoch,
        accepted_generation,
        worker_id: &worker_id,
        claim_epoch,
        claim_proof_digest: &claim_proof_digest,
        claim_clock_profile_id: &claim_clock_profile_id,
        claim_issued_at: &claim_issued_at,
        claim_expires_at: &claim_expires_at,
        database_xact_id: &database_xact_id,
        epoch_transition: &epoch_transition,
        delta_minor,
        balance_after_minor,
        idempotency_digest: &idempotency_digest,
        request_digest: &request_digest,
        decision_receipt_digest: &decision_receipt_digest,
        payload_digest: &payload_digest,
    })?;

    Ok(VerifiedPostgresEffectReceipt {
        effect_id,
        attestation_id,
        realm_id,
        resource_id,
        action_id,
        effect_owner_id,
        object_id,
        currency_code,
        logical_operation_id,
        owner_epoch,
        accepted_generation,
        worker_id,
        claim_epoch,
        claim_proof_digest,
        claim_clock_profile_id,
        claim_issued_at,
        claim_expires_at,
        database_xact_id,
        epoch_transition,
        delta_minor,
        balance_after_minor,
        idempotency_digest,
        request_digest,
        decision_receipt_digest,
        payload_digest,
        effect_receipt_digest,
    })
}

fn decode_epoch_transition(
    owner_epoch: u64,
    accepted_generation: u64,
    columns: (
        Option<i64>,
        Option<i64>,
        Option<i64>,
        Option<i64>,
        Option<String>,
        Option<Digest>,
        Option<Digest>,
    ),
) -> Result<Option<VerifiedOwnerEpochTransition>, String> {
    let (
        previous_owner_epoch,
        new_owner_epoch,
        previous_generation,
        new_generation,
        transition_xact_id,
        epoch_authority_receipt_digest,
        physical_fencing_receipt_digest,
    ) = columns;

    let all_absent = previous_owner_epoch.is_none()
        && new_owner_epoch.is_none()
        && previous_generation.is_none()
        && new_generation.is_none()
        && transition_xact_id.is_none()
        && epoch_authority_receipt_digest.is_none()
        && physical_fencing_receipt_digest.is_none();

    if owner_epoch == 1 {
        return if all_absent {
            Ok(None)
        } else {
            Err("initial owner epoch must not have a transition row".into())
        };
    }

    let transition = VerifiedOwnerEpochTransition {
        previous_owner_epoch: positive_position(
            previous_owner_epoch,
            "transition previous owner epoch",
        )?,
        new_owner_epoch: positive_position(
            new_owner_epoch,
            "transition new owner epoch",
        )?,
        previous_generation: positive_position(
            previous_generation,
            "transition previous generation",
        )?,
        new_generation: positive_position(
            new_generation,
            "transition new generation",
        )?,
        transition_xact_id: transition_xact_id
            .ok_or_else(|| "owner epoch transition transaction ID is missing".to_owned())?,
        epoch_authority_receipt_digest: epoch_authority_receipt_digest
            .ok_or_else(|| "owner epoch authority receipt digest is missing".to_owned())?,
        physical_fencing_receipt_digest: physical_fencing_receipt_digest
            .ok_or_else(|| "physical fencing receipt digest is missing".to_owned())?,
    };
    validate_epoch_transition(&transition, owner_epoch, accepted_generation)?;
    Ok(Some(transition))
}

fn positive_position(value: Option<i64>, name: &str) -> Result<u64, String> {
    let value = value.ok_or_else(|| format!("PostgreSQL {name} is missing"))?;
    u64::try_from(value).map_err(|_| format!("PostgreSQL {name} is nonpositive"))
}

fn validate_epoch_transition(
    transition: &VerifiedOwnerEpochTransition,
    attested_owner_epoch: u64,
    accepted_generation: u64,
) -> Result<(), String> {
    if transition.previous_owner_epoch >= transition.new_owner_epoch
        || transition.new_owner_epoch != attested_owner_epoch
        || transition.previous_generation >= transition.new_generation
        || transition.new_generation > accepted_generation
        || transition.transition_xact_id.trim().is_empty()
        || transition.epoch_authority_receipt_digest.iter().all(|byte| *byte == 0)
        || transition.physical_fencing_receipt_digest.iter().all(|byte| *byte == 0)
    {
        return Err(
            "owner epoch transition chronology or authority/fencing evidence is invalid".into(),
        );
    }
    Ok(())
}

fn digest_column(row: &Row, index: usize, name: &str) -> Result<Digest, String> {
    let bytes: Vec<u8> = row.get(index);
    digest_from_bytes(bytes, name)
}

fn optional_digest_column(row: &Row, index: usize, name: &str) -> Result<Option<Digest>, String> {
    let bytes: Option<Vec<u8>> = row.get(index);
    bytes
        .map(|value| digest_from_bytes(value, name))
        .transpose()
}

fn digest_from_bytes(bytes: Vec<u8>, name: &str) -> Result<Digest, String> {
    let digest: Digest = bytes
        .try_into()
        .map_err(|_| format!("PostgreSQL {name} is not exactly 32 bytes"))?;
    if digest.iter().all(|byte| *byte == 0) {
        return Err(format!("PostgreSQL {name} is the zero digest"));
    }
    Ok(digest)
}

impl VerifiedPostgresEffectReceipt {
    fn recompute_digest(&self) -> Result<Digest, String> {
        canonical_effect_receipt_digest(&EffectReceiptDigestFields {
            effect_id: &self.effect_id,
            attestation_id: &self.attestation_id,
            realm_id: &self.realm_id,
            resource_id: &self.resource_id,
            action_id: &self.action_id,
            effect_owner_id: &self.effect_owner_id,
            object_id: &self.object_id,
            currency_code: &self.currency_code,
            logical_operation_id: &self.logical_operation_id,
            owner_epoch: self.owner_epoch,
            accepted_generation: self.accepted_generation,
            worker_id: &self.worker_id,
            claim_epoch: self.claim_epoch,
            claim_proof_digest: &self.claim_proof_digest,
            claim_clock_profile_id: &self.claim_clock_profile_id,
            claim_issued_at: &self.claim_issued_at,
            claim_expires_at: &self.claim_expires_at,
            database_xact_id: &self.database_xact_id,
            epoch_transition: &self.epoch_transition,
            delta_minor: self.delta_minor,
            balance_after_minor: self.balance_after_minor,
            idempotency_digest: &self.idempotency_digest,
            request_digest: &self.request_digest,
            decision_receipt_digest: &self.decision_receipt_digest,
            payload_digest: &self.payload_digest,
        })
    }
}

fn validate_receipt_binding(
    binding: &NatsCommandBinding,
    receipt: &VerifiedPostgresEffectReceipt,
) -> Result<(), String> {
    let (command_owner_epoch, command_generation) = binding.owner_position();
    // Owner epoch and resource generation are both monotonic coordinates.
    // A receipt is never allowed to be ahead in either dimension, even when
    // its other coordinate is from an older epoch.
    if receipt.worker_id.trim().is_empty()
        || receipt.claim_epoch == 0
        || receipt.claim_proof_digest.iter().all(|byte| *byte == 0)
        || receipt.claim_clock_profile_id.trim().is_empty()
        || receipt.claim_issued_at >= receipt.claim_expires_at
        || receipt.database_xact_id.trim().is_empty()
    {
        return Err(
            "PostgreSQL execution attestation has incomplete worker-claim coordinates".into(),
        );
    }

    if receipt.recompute_digest()? != receipt.effect_receipt_digest {
        return Err(
            "canonical PostgreSQL effect/execution receipt digest does not match its fields".into(),
        );
    }

    if receipt.realm_id != binding.realm_id()
        || receipt.resource_id != binding.resource_id()
        || receipt.action_id != binding.action_id()
        || receipt.effect_owner_id != binding.effect_owner_id()
        || receipt.object_id != binding.object_id()
        || receipt.currency_code != binding.currency_code()
        || receipt.logical_operation_id != binding.logical_operation_id()
        || receipt.idempotency_digest != *binding.idempotency_digest()
        || receipt.request_digest != *binding.request_digest()
        || receipt.decision_receipt_digest != *binding.decision_receipt_digest()
        || receipt.payload_digest != *binding.payload_digest()
        || receipt.owner_epoch > command_owner_epoch
        || receipt.accepted_generation > command_generation
    {
        return Err(
            "PostgreSQL effect and execution attestation do not match the exact command scope, digests or non-future owner position"
                .into(),
        );
    }
    Ok(())
}

/// Versioned canonical receipt bytes. All strings are length-prefixed UTF-8;
/// fixed-size digests and signed integer fields use network byte order. The
/// commitment includes the actual worker-claim and owner-epoch evidence.
struct EffectReceiptDigestFields<'a> {
    effect_id: &'a str,
    attestation_id: &'a str,
    realm_id: &'a str,
    resource_id: &'a str,
    action_id: &'a str,
    effect_owner_id: &'a str,
    object_id: &'a str,
    currency_code: &'a str,
    logical_operation_id: &'a str,
    owner_epoch: u64,
    accepted_generation: u64,
    worker_id: &'a str,
    claim_epoch: u64,
    claim_proof_digest: &'a Digest,
    claim_clock_profile_id: &'a str,
    claim_issued_at: &'a str,
    claim_expires_at: &'a str,
    database_xact_id: &'a str,
    epoch_transition: &'a Option<VerifiedOwnerEpochTransition>,
    delta_minor: i64,
    balance_after_minor: i64,
    idempotency_digest: &'a Digest,
    request_digest: &'a Digest,
    decision_receipt_digest: &'a Digest,
    payload_digest: &'a Digest,
}

fn canonical_effect_receipt_digest(
    fields: &EffectReceiptDigestFields<'_>,
) -> Result<Digest, String> {
    let mut bytes = RECEIPT_DOMAIN.to_vec();
    for value in [
        fields.effect_id,
        fields.attestation_id,
        fields.realm_id,
        fields.resource_id,
        fields.action_id,
        fields.effect_owner_id,
        fields.object_id,
        fields.currency_code,
        fields.logical_operation_id,
        fields.worker_id,
        fields.claim_clock_profile_id,
        fields.claim_issued_at,
        fields.claim_expires_at,
        fields.database_xact_id,
    ] {
        append_string(&mut bytes, value)?;
    }
    bytes.extend_from_slice(&fields.owner_epoch.to_be_bytes());
    bytes.extend_from_slice(&fields.accepted_generation.to_be_bytes());
    bytes.extend_from_slice(&fields.claim_epoch.to_be_bytes());
    bytes.extend_from_slice(&fields.delta_minor.to_be_bytes());
    bytes.extend_from_slice(&fields.balance_after_minor.to_be_bytes());
    bytes.extend_from_slice(fields.claim_proof_digest);
    match fields.epoch_transition {
        Some(transition) => {
            bytes.push(1);
            bytes.extend_from_slice(&transition.previous_owner_epoch.to_be_bytes());
            bytes.extend_from_slice(&transition.new_owner_epoch.to_be_bytes());
            bytes.extend_from_slice(&transition.previous_generation.to_be_bytes());
            bytes.extend_from_slice(&transition.new_generation.to_be_bytes());
            append_string(&mut bytes, &transition.transition_xact_id)?;
            bytes.extend_from_slice(&transition.epoch_authority_receipt_digest);
            bytes.extend_from_slice(&transition.physical_fencing_receipt_digest);
        }
        None => bytes.push(0),
    }
    bytes.extend_from_slice(fields.idempotency_digest);
    bytes.extend_from_slice(fields.request_digest);
    bytes.extend_from_slice(fields.decision_receipt_digest);
    bytes.extend_from_slice(fields.payload_digest);
    Ok(sha256(&bytes))
}

fn append_string(bytes: &mut Vec<u8>, value: &str) -> Result<(), String> {
    let length = u32::try_from(value.len())
        .map_err(|_| "effect receipt field exceeds canonical encoding limit".to_owned())?;
    bytes.extend_from_slice(&length.to_be_bytes());
    bytes.extend_from_slice(value.as_bytes());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest {
        [byte; 32]
    }

    fn binding() -> NatsCommandBinding {
        binding_with_position(5, 7)
    }

    fn binding_with_position(owner_epoch: u64, resource_generation: u64) -> NatsCommandBinding {
        NatsCommandBinding::new(myc_dir_fabric_nats_reference::NatsCommandBindingInput {
            account_id: "test-account".into(),
            realm_id: "realm-a".into(),
            resource_id: "resource-a".into(),
            action_id: "balance-adjust".into(),
            object_id: "account-a".into(),
            currency_code: "USD".into(),
            stream_name: "MYCELIX_COMMANDS".into(),
            subject: "mycelix.command.resource-a".into(),
            consumer_name: "effect-workers".into(),
            nats_message_id: "publication-a".into(),
            payload_digest: digest(4),
            request_digest: digest(2),
            decision_receipt_digest: digest(3),
            logical_operation_id: "operation-a".into(),
            idempotency_digest: digest(1),
            effect_owner_id: "owner-a".into(),
            owner_epoch,
            resource_generation,
        })
        .unwrap()
    }

    fn recommit_receipt(receipt: &mut VerifiedPostgresEffectReceipt) {
        receipt.effect_receipt_digest = receipt.recompute_digest().unwrap();
    }

    fn receipt_for(binding: &NatsCommandBinding) -> VerifiedPostgresEffectReceipt {
        let effect_id = "7a1d28d8-5e9b-4a65-95dd-329816bfc33a".to_owned();
        let attestation_id = "af5ec518-655c-4539-aae3-890e35034329".to_owned();
        let realm_id = binding.realm_id().to_owned();
        let resource_id = binding.resource_id().to_owned();
        let action_id = binding.action_id().to_owned();
        let effect_owner_id = binding.effect_owner_id().to_owned();
        let object_id = binding.object_id().to_owned();
        let currency_code = binding.currency_code().to_owned();
        let logical_operation_id = binding.logical_operation_id().to_owned();
        let owner_epoch = 4;
        let accepted_generation = 7;
        let worker_id = "worker-a".to_owned();
        let claim_epoch = 3;
        let claim_proof_digest = digest(5);
        let claim_clock_profile_id = "postgres-db-clock-v1".to_owned();
        let claim_issued_at = "2026-11-22T10:00:00.000000".to_owned();
        let claim_expires_at = "2026-11-22T11:00:00.000000".to_owned();
        let database_xact_id = "73421".to_owned();
        let epoch_transition = Some(VerifiedOwnerEpochTransition {
            previous_owner_epoch: 2,
            new_owner_epoch: owner_epoch,
            previous_generation: accepted_generation - 1,
            new_generation: accepted_generation,
            transition_xact_id: "73420".to_owned(),
            epoch_authority_receipt_digest: digest(6),
            physical_fencing_receipt_digest: digest(7),
        });
        let delta_minor = 100;
        let balance_after_minor = 1100;
        let idempotency_digest = *binding.idempotency_digest();
        let request_digest = *binding.request_digest();
        let decision_receipt_digest = *binding.decision_receipt_digest();
        let payload_digest = *binding.payload_digest();
        let mut receipt = VerifiedPostgresEffectReceipt {
            effect_id,
            attestation_id,
            realm_id,
            resource_id,
            action_id,
            effect_owner_id,
            object_id,
            currency_code,
            logical_operation_id,
            owner_epoch,
            accepted_generation,
            worker_id,
            claim_epoch,
            claim_proof_digest,
            claim_clock_profile_id,
            claim_issued_at,
            claim_expires_at,
            database_xact_id,
            epoch_transition,
            delta_minor,
            balance_after_minor,
            idempotency_digest,
            request_digest,
            decision_receipt_digest,
            payload_digest,
            effect_receipt_digest: [0; 32],
        };
        recommit_receipt(&mut receipt);
        receipt
    }

    #[test]
    fn exact_database_effect_and_execution_attestation_match_command() {
        let binding = binding();
        let receipt = receipt_for(&binding);
        assert_eq!(validate_receipt_binding(&binding, &receipt), Ok(()));
    }

    #[test]
    fn receipt_must_not_be_from_a_future_owner_position() {
        let binding = binding();
        let mut receipt = receipt_for(&binding);
        receipt.owner_epoch = 6;
        recommit_receipt(&mut receipt);
        assert!(validate_receipt_binding(&binding, &receipt).is_err());

        let older_generation_binding = binding_with_position(5, 6);
        let mut future_generation_receipt = receipt_for(&older_generation_binding);
        future_generation_receipt.accepted_generation = 7;
        recommit_receipt(&mut future_generation_receipt);
        assert!(
            validate_receipt_binding(&older_generation_binding, &future_generation_receipt)
                .is_err()
        );
    }

    #[test]
    fn receipt_request_decision_payload_owner_generation_and_claim_substitution_reject() {
        let binding = binding();
        let original = receipt_for(&binding);

        let mut receipt = original.clone();
        receipt.request_digest = digest(21);
        assert!(validate_receipt_binding(&binding, &receipt).is_err());

        let mut receipt = original.clone();
        receipt.decision_receipt_digest = digest(22);
        assert!(validate_receipt_binding(&binding, &receipt).is_err());

        let mut receipt = original.clone();
        receipt.payload_digest = digest(23);
        assert!(validate_receipt_binding(&binding, &receipt).is_err());

        let mut receipt = original.clone();
        receipt.effect_owner_id = "owner-b".into();
        assert!(validate_receipt_binding(&binding, &receipt).is_err());

        let mut receipt = original.clone();
        receipt.accepted_generation += 1;
        assert!(validate_receipt_binding(&binding, &receipt).is_err());

        let mut receipt = original;
        receipt.claim_proof_digest = digest(24);
        assert!(validate_receipt_binding(&binding, &receipt).is_err());
    }

    #[test]
    fn epoch_transition_requires_complete_monotonic_chronology() {
        let valid = VerifiedOwnerEpochTransition {
            previous_owner_epoch: 1,
            new_owner_epoch: 2,
            previous_generation: 1,
            new_generation: 2,
            transition_xact_id: "100".into(),
            epoch_authority_receipt_digest: digest(6),
            physical_fencing_receipt_digest: digest(7),
        };
        assert_eq!(validate_epoch_transition(&valid, 2, 2), Ok(()));
        assert!(validate_epoch_transition(&valid, 3, 2).is_err());

        let mut wrong_owner = valid.clone();
        wrong_owner.previous_owner_epoch = 2;
        assert!(validate_epoch_transition(&wrong_owner, 2, 2).is_err());

        let mut generation_after_effect = valid.clone();
        generation_after_effect.new_generation = 3;
        assert!(validate_epoch_transition(&generation_after_effect, 2, 2).is_err());

        let mut blank_xact = valid.clone();
        blank_xact.transition_xact_id = "  ".into();
        assert!(validate_epoch_transition(&blank_xact, 2, 2).is_err());
    }

    #[test]
    fn initial_epoch_has_no_transition_and_later_epoch_requires_one() {
        let no_transition = (None, None, None, None, None, None, None);
        assert_eq!(decode_epoch_transition(1, 1, no_transition), Ok(None));

        let partial_transition = (
            Some(1),
            Some(2),
            Some(1),
            Some(2),
            Some("200".into()),
            Some(digest(6)),
            None,
        );
        assert!(decode_epoch_transition(2, 2, partial_transition).is_err());
        assert!(decode_epoch_transition(2, 2, no_transition).is_err());
    }

    #[test]
    fn canonical_execution_receipt_digest_is_deterministic_and_binds_claim() {
        let binding = binding();
        let receipt = receipt_for(&binding);
        let unchanged = receipt.recompute_digest().unwrap();
        assert_eq!(unchanged, receipt.effect_receipt_digest);
    }
}
