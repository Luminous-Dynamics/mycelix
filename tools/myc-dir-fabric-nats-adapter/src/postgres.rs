//! PostgreSQL-backed verifier for Directory Fabric effect receipts.
//!
//! This verifier reads the existing append-only myc_effect_owner.effect_commits
//! ledger. A successful lookup is proof of visibility on the connected
//! writable primary; durability across promotion still depends on the database
//! synchronous-commit/replication and promotion policy.

use std::path::Path;

use myc_dir_fabric_nats_reference::{
    DurableOwnerDisposition, NatsCommandBinding,
};
use rustls_tokio_postgres::{config_from_ca_cert, MakeRustlsConnect};
use tokio_postgres::{Client, Row};

use crate::{
    sha256, AdapterResult, OwnerDispositionVerifier, VerifiedOwnerDisposition,
};

const RECEIPT_DOMAIN: &[u8] = b"MYC-DIR-POSTGRES-EFFECT-RECEIPT\0\x01";

const SELECT_EFFECT_COMMIT: &str = r#"
SELECT
    effect_id::text,
    realm_id,
    resource_id,
    action_id,
    effect_owner_id,
    object_id,
    currency_code,
    idempotency_key_digest,
    logical_operation_id,
    request_digest,
    decision_receipt_digest,
    payload_digest,
    accepted_generation,
    delta_minor,
    balance_after_minor
FROM myc_effect_owner.effect_commits
WHERE realm_id = $1
  AND resource_id = $2
  AND action_id = $3
  AND effect_owner_id = $4
  AND object_id = $5
  AND currency_code = $6
  AND idempotency_key_digest = $7
  AND logical_operation_id = $8
"#;

const CHECK_PRIMARY: &str = r#"
SELECT pg_catalog.pg_is_in_recovery(),
       current_setting('transaction_read_only')::boolean
"#;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VerifiedPostgresEffectReceipt {
    pub effect_id: String,
    pub realm_id: String,
    pub resource_id: String,
    pub action_id: String,
    pub effect_owner_id: String,
    pub object_id: String,
    pub currency_code: String,
    pub logical_operation_id: String,
    pub accepted_generation: u64,
    pub delta_minor: i64,
    pub balance_after_minor: i64,
    pub idempotency_digest: [u8; 32],
    pub request_digest: [u8; 32],
    pub decision_receipt_digest: [u8; 32],
    pub payload_digest: [u8; 32],
    pub effect_receipt_digest: [u8; 32],
}

#[derive(Clone)]
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

    /// Query the append-only effect ledger and return a verified receipt
    /// capability ready for the NATS ACK gate. The caller does not provide the
    /// receipt digest; it is recomputed from the selected database row.
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

    /// A writable-primary check followed by an exact-scope ledger lookup.
    pub async fn load_effect_receipt(
        &self,
        binding: &NatsCommandBinding,
    ) -> Result<VerifiedPostgresEffectReceipt, String> {
        let server_state = self
            .client
            .query_one(CHECK_PRIMARY, &[])
            .await
            .map_err(|error| format!("check PostgreSQL primary state: {error}"))?;
        let in_recovery: bool = server_state.get(0);
        let transaction_read_only: bool = server_state.get(1);
        if in_recovery {
            return Err("PostgreSQL receipt verifier connected to a recovery/read replica".into());
        }
        if transaction_read_only {
            return Err("PostgreSQL receipt verifier session is read-only".into());
        }

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
            .map_err(|error| format!("query append-only PostgreSQL effect ledger: {error}"))?
            .ok_or_else(|| {
                "no committed PostgreSQL effect receipt matches the exact command scope".to_owned()
            })?;

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
                "effect receipt digest differs from the canonical append-only PostgreSQL row"
                    .into(),
            );
        }
        Ok(())
    }
}

fn decode_effect_receipt(row: &Row) -> Result<VerifiedPostgresEffectReceipt, String> {
    let effect_id: String = row.get(0);
    let realm_id: String = row.get(1);
    let resource_id: String = row.get(2);
    let action_id: String = row.get(3);
    let effect_owner_id: String = row.get(4);
    let object_id: String = row.get(5);
    let currency_code: String = row.get(6);
    let idempotency_digest = digest_column(row, 7, "idempotency_key_digest")?;
    let logical_operation_id: String = row.get(8);
    let request_digest = digest_column(row, 9, "request_digest")?;
    let decision_receipt_digest = digest_column(row, 10, "decision_receipt_digest")?;
    let payload_digest = digest_column(row, 11, "payload_digest")?;
    let accepted_generation_i64: i64 = row.get(12);
    let delta_minor: i64 = row.get(13);
    let balance_after_minor: i64 = row.get(14);

    let accepted_generation = u64::try_from(accepted_generation_i64)
        .map_err(|_| "PostgreSQL receipt has a nonpositive accepted generation".to_owned())?;

    let effect_receipt_digest = canonical_effect_receipt_digest(
        &effect_id,
        &realm_id,
        &resource_id,
        &action_id,
        &effect_owner_id,
        &object_id,
        &currency_code,
        &logical_operation_id,
        accepted_generation,
        delta_minor,
        balance_after_minor,
        &idempotency_digest,
        &request_digest,
        &decision_receipt_digest,
        &payload_digest,
    )?;

    Ok(VerifiedPostgresEffectReceipt {
        effect_id,
        realm_id,
        resource_id,
        action_id,
        effect_owner_id,
        object_id,
        currency_code,
        logical_operation_id,
        accepted_generation,
        delta_minor,
        balance_after_minor,
        idempotency_digest,
        request_digest,
        decision_receipt_digest,
        payload_digest,
        effect_receipt_digest,
    })
}

fn digest_column(row: &Row, index: usize, name: &str) -> Result<[u8; 32], String> {
    let bytes: Vec<u8> = row.get(index);
    let digest: [u8; 32] = bytes
        .try_into()
        .map_err(|_| format!("PostgreSQL {name} is not exactly 32 bytes"))?;
    if digest.iter().all(|byte| *byte == 0) {
        return Err(format!("PostgreSQL {name} is the zero digest"));
    }
    Ok(digest)
}

fn validate_receipt_binding(
    binding: &NatsCommandBinding,
    receipt: &VerifiedPostgresEffectReceipt,
) -> Result<(), String> {
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
        || receipt.accepted_generation != binding.owner_position().1
    {
        return Err(
            "append-only PostgreSQL effect row does not match every command scope and digest field"
                .into(),
        );
    }
    Ok(())
}

/// Versioned canonical receipt bytes. All strings are length-prefixed UTF-8;
/// fixed-size digests and signed integer fields use network byte order.
fn canonical_effect_receipt_digest(
    effect_id: &str,
    realm_id: &str,
    resource_id: &str,
    action_id: &str,
    effect_owner_id: &str,
    object_id: &str,
    currency_code: &str,
    logical_operation_id: &str,
    accepted_generation: u64,
    delta_minor: i64,
    balance_after_minor: i64,
    idempotency_digest: &Digest,
    request_digest: &Digest,
    decision_receipt_digest: &Digest,
    payload_digest: &Digest,
) -> Result<Digest, String> {
    let mut bytes = RECEIPT_DOMAIN.to_vec();
    for value in [
        effect_id,
        realm_id,
        resource_id,
        action_id,
        effect_owner_id,
        object_id,
        currency_code,
        logical_operation_id,
    ] {
        append_string(&mut bytes, value)?;
    }
    bytes.extend_from_slice(&accepted_generation.to_be_bytes());
    bytes.extend_from_slice(&delta_minor.to_be_bytes());
    bytes.extend_from_slice(&balance_after_minor.to_be_bytes());
    bytes.extend_from_slice(idempotency_digest);
    bytes.extend_from_slice(request_digest);
    bytes.extend_from_slice(decision_receipt_digest);
    bytes.extend_from_slice(payload_digest);
    Ok(sha256(&bytes))
}

fn append_string(bytes: &mut Vec<u8>, value: &str) -> Result<(), String> {
    let length = u32::try_from(value.len())
        .map_err(|_| "effect receipt field exceeds canonical encoding limit".to_owned())?;
    bytes.extend_from_slice(&length.to_be_bytes());
    bytes.extend_from_slice(value.as_bytes());
    Ok(())
}
