// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! PSI-002B3B2A r2 — nonce-bound SQLite WAL/FULL atomic spend backend.

#![forbid(unsafe_code)]

use psi_query_credit_spend_core::QueryCreditSpendKeyV2;
use rusqlite::{params, Connection, TransactionBehavior};
use serde::Serialize;
use std::{path::Path, sync::Mutex, time::Duration};

pub const SQLITE_SPEND_PROFILE_V2: &str = "sqlite-wal-full-query-token-spend-v2-nonce-bound";
pub const SQLITE_SCHEMA_VERSION_V2: i64 = 2;
pub const SQLITE_BUSY_TIMEOUT_MS: u64 = 5_000;
pub const SQLITE_SYNCHRONOUS_FULL_CODE: i64 = 2;

const SCHEMA_SQL: &str = r#"
CREATE TABLE IF NOT EXISTS query_credit_store_meta_v2 (
    key TEXT PRIMARY KEY NOT NULL,
    value TEXT NOT NULL
) STRICT;
CREATE TABLE IF NOT EXISTS query_token_spends_v2 (
    spend_key_sha256 TEXT PRIMARY KEY NOT NULL CHECK(length(spend_key_sha256) = 64),
    policy_sha256 TEXT NOT NULL CHECK(length(policy_sha256) = 64),
    token_nonce_sha256 TEXT NOT NULL CHECK(length(token_nonce_sha256) = 64),
    requested_identifier_count INTEGER NOT NULL CHECK(requested_identifier_count > 0)
) STRICT;
"#;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SqliteSpendError {
    InvalidStoreInstanceId,
    Database(String),
    JournalModeMismatch(String),
    SynchronousModeMismatch(i64),
    BusyTimeoutMismatch(i64),
    SchemaVersionMismatch(i64),
    StoreInstanceIdentityMismatch,
    Replay,
    LockPoisoned,
}

impl From<rusqlite::Error> for SqliteSpendError {
    fn from(value: rusqlite::Error) -> Self {
        Self::Database(value.to_string())
    }
}

#[derive(Debug)]
pub struct SqliteFullSyncSpendStoreV2 {
    connection: Mutex<Connection>,
    store_instance_id: String,
    sqlite_runtime_version: String,
}

impl SqliteFullSyncSpendStoreV2 {
    pub fn open(
        path: impl AsRef<Path>,
        store_instance_id: impl Into<String>,
    ) -> Result<Self, SqliteSpendError> {
        let store_instance_id = store_instance_id.into();
        validate_store_instance_id(&store_instance_id)?;

        let mut connection = Connection::open(path)?;
        connection.busy_timeout(Duration::from_millis(SQLITE_BUSY_TIMEOUT_MS))?;

        let journal_mode: String =
            connection.query_row("PRAGMA journal_mode=WAL", [], |row| row.get(0))?;
        if !journal_mode.eq_ignore_ascii_case("wal") {
            return Err(SqliteSpendError::JournalModeMismatch(journal_mode));
        }
        connection.pragma_update(None, "synchronous", SQLITE_SYNCHRONOUS_FULL_CODE)?;
        verify_sqlite_runtime_mode(&connection)?;

        let existing_version: i64 =
            connection.pragma_query_value(None, "user_version", |row| row.get(0))?;
        if existing_version != 0 && existing_version != SQLITE_SCHEMA_VERSION_V2 {
            return Err(SqliteSpendError::SchemaVersionMismatch(existing_version));
        }

        connection.execute_batch(SCHEMA_SQL)?;
        if existing_version == 0 {
            connection.pragma_update(None, "user_version", SQLITE_SCHEMA_VERSION_V2)?;
        }
        verify_full_profile(&connection)?;

        let transaction = connection.transaction_with_behavior(TransactionBehavior::Immediate)?;
        transaction.execute(
            "INSERT OR IGNORE INTO query_credit_store_meta_v2(key, value) VALUES('store_instance_id', ?1)",
            params![&store_instance_id],
        )?;
        let persisted: String = transaction.query_row(
            "SELECT value FROM query_credit_store_meta_v2 WHERE key='store_instance_id'",
            [],
            |row| row.get(0),
        )?;
        if persisted != store_instance_id {
            transaction.rollback()?;
            return Err(SqliteSpendError::StoreInstanceIdentityMismatch);
        }
        transaction.commit()?;

        Ok(Self {
            connection: Mutex::new(connection),
            store_instance_id,
            sqlite_runtime_version: rusqlite::version().to_owned(),
        })
    }

    pub fn store_instance_id(&self) -> &str {
        &self.store_instance_id
    }

    pub fn sqlite_runtime_version(&self) -> &str {
        &self.sqlite_runtime_version
    }

    pub fn consume_once(
        &self,
        key: &QueryCreditSpendKeyV2,
    ) -> Result<SqliteFullSyncConsumedQueryTokenV2, SqliteSpendError> {
        let mut connection = self
            .connection
            .lock()
            .map_err(|_| SqliteSpendError::LockPoisoned)?;
        let transaction = connection.transaction_with_behavior(TransactionBehavior::Immediate)?;
        verify_full_profile(&transaction)?;

        let changed = transaction.execute(
            "INSERT OR IGNORE INTO query_token_spends_v2(
                spend_key_sha256, policy_sha256, token_nonce_sha256, requested_identifier_count
             ) VALUES(?1, ?2, ?3, ?4)",
            params![
                key.commitment_sha256(),
                key.policy_sha256(),
                key.token_nonce_sha256(),
                i64::from(key.requested_identifier_count()),
            ],
        )?;

        if changed != 1 {
            transaction.rollback()?;
            return Err(SqliteSpendError::Replay);
        }

        transaction.commit()?;

        Ok(SqliteFullSyncConsumedQueryTokenV2 {
            spend_key_sha256: key.commitment_sha256().to_owned(),
            policy_sha256: key.policy_sha256().to_owned(),
            token_nonce_sha256: key.token_nonce_sha256().to_owned(),
            requested_identifier_count: key.requested_identifier_count(),
            store_instance_id: self.store_instance_id.clone(),
            sqlite_profile: SQLITE_SPEND_PROFILE_V2.to_owned(),
            sqlite_runtime_version: self.sqlite_runtime_version.clone(),
        })
    }

    pub fn consumed_count(&self) -> Result<u64, SqliteSpendError> {
        let connection = self
            .connection
            .lock()
            .map_err(|_| SqliteSpendError::LockPoisoned)?;
        let count: i64 = connection.query_row(
            "SELECT COUNT(*) FROM query_token_spends_v2",
            [],
            |row| row.get(0),
        )?;
        u64::try_from(count).map_err(|_| SqliteSpendError::Database("negative row count".into()))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct SqliteFullSyncConsumedQueryTokenV2 {
    spend_key_sha256: String,
    policy_sha256: String,
    token_nonce_sha256: String,
    requested_identifier_count: u32,
    store_instance_id: String,
    sqlite_profile: String,
    sqlite_runtime_version: String,
}

impl SqliteFullSyncConsumedQueryTokenV2 {
    pub fn spend_key_sha256(&self) -> &str { &self.spend_key_sha256 }
    pub fn token_nonce_sha256(&self) -> &str { &self.token_nonce_sha256 }
    pub fn store_instance_id(&self) -> &str { &self.store_instance_id }
    pub const fn sqlite_atomic_single_use_established(&self) -> bool { true }
    pub const fn sqlite_full_sync_profile_established(&self) -> bool { true }
    pub const fn token_nonce_cryptographically_bound(&self) -> bool { false }
    pub const fn privacy_pass_token_cryptographically_verified(&self) -> bool { false }
    pub const fn global_store_uniqueness_established(&self) -> bool { false }
    pub const fn hardware_power_loss_durability_established(&self) -> bool { false }
    pub const fn safe_compaction_established(&self) -> bool { false }
    pub const fn query_credit_granted(&self) -> bool { false }
    pub const fn anonymous_rate_limit_established(&self) -> bool { false }
    pub const fn enumeration_resistance_established(&self) -> bool { false }
    pub const fn application_authority_granted(&self) -> bool { false }
}

fn validate_store_instance_id(value: &str) -> Result<(), SqliteSpendError> {
    if value.is_empty()
        || value.len() > 128
        || !value.is_ascii()
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_' | b'.' | b':'))
    {
        return Err(SqliteSpendError::InvalidStoreInstanceId);
    }
    Ok(())
}

fn verify_sqlite_runtime_mode(connection: &Connection) -> Result<(), SqliteSpendError> {
    let journal_mode: String =
        connection.pragma_query_value(None, "journal_mode", |row| row.get(0))?;
    if !journal_mode.eq_ignore_ascii_case("wal") {
        return Err(SqliteSpendError::JournalModeMismatch(journal_mode));
    }
    let synchronous: i64 =
        connection.pragma_query_value(None, "synchronous", |row| row.get(0))?;
    if synchronous != SQLITE_SYNCHRONOUS_FULL_CODE {
        return Err(SqliteSpendError::SynchronousModeMismatch(synchronous));
    }
    let busy_timeout: i64 =
        connection.pragma_query_value(None, "busy_timeout", |row| row.get(0))?;
    if busy_timeout != SQLITE_BUSY_TIMEOUT_MS as i64 {
        return Err(SqliteSpendError::BusyTimeoutMismatch(busy_timeout));
    }
    Ok(())
}

fn verify_full_profile(connection: &Connection) -> Result<(), SqliteSpendError> {
    verify_sqlite_runtime_mode(connection)?;
    let schema_version: i64 =
        connection.pragma_query_value(None, "user_version", |row| row.get(0))?;
    if schema_version != SQLITE_SCHEMA_VERSION_V2 {
        return Err(SqliteSpendError::SchemaVersionMismatch(schema_version));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use psi_privacy_pass_credit_core::{
        PrivacyPassRedemptionObservationV1, QueryCreditPolicyV1, ReplayPolicyV1,
        Rfc9578TokenType,
    };
    use psi_query_credit_spend_core::derive_spend_key_v2;
    use std::sync::{Arc, Barrier};
    use std::thread;
    use tempfile::tempdir;

    fn policy() -> QueryCreditPolicyV1 {
        QueryCreditPolicyV1 {
            service_domain: "contacts.mycelix.test".into(),
            issuer_name: "issuer.mycelix.test".into(),
            issuer_configuration_sha256: "11".repeat(32),
            token_type: Rfc9578TokenType::PrivateVoprfP384Sha384,
            token_key_id_sha256: "22".repeat(32),
            budget_epoch: "epoch-a".into(),
            max_identifiers_per_credit: 256,
            replay_policy: ReplayPolicyV1::AtomicSingleUseRequired,
        }
    }

    fn key(nonce_byte: &str, token_byte: &str) -> QueryCreditSpendKeyV2 {
        let policy = policy();
        let challenge = policy.challenge_binding().unwrap();
        let observation = PrivacyPassRedemptionObservationV1 {
            token_type: policy.token_type,
            issuer_name: policy.issuer_name.clone(),
            token_key_id_sha256: policy.token_key_id_sha256.clone(),
            service_domain: policy.service_domain.clone(),
            budget_epoch: policy.budget_epoch.clone(),
            challenge_binding_sha256: challenge.commitment_sha256().unwrap(),
            token_challenge_digest_sha256: "33".repeat(32),
            token_nonce_sha256: nonce_byte.repeat(32),
            token_sha256: token_byte.repeat(32),
            backend_profile: "future-rfc9578-backend-v1".into(),
            backend_receipt_sha256: "66".repeat(32),
            requested_identifier_count: 128,
        };
        derive_spend_key_v2(&policy, &challenge, &observation).unwrap()
    }

    #[test]
    fn exact_wal_full_profile_and_first_commit() {
        let directory = tempdir().unwrap();
        let path = directory.path().join("spends.sqlite3");
        let store = SqliteFullSyncSpendStoreV2::open(&path, "contacts-primary").unwrap();
        let positive = store.consume_once(&key("44", "55")).unwrap();
        assert!(positive.sqlite_atomic_single_use_established());
        assert!(positive.sqlite_full_sync_profile_established());
        assert_eq!(store.consumed_count().unwrap(), 1);
    }

    #[test]
    fn replay_fails_after_close_and_reopen() {
        let directory = tempdir().unwrap();
        let path = directory.path().join("spends.sqlite3");
        let spend_key = key("44", "55");
        {
            let store = SqliteFullSyncSpendStoreV2::open(&path, "contacts-primary").unwrap();
            store.consume_once(&spend_key).unwrap();
        }
        let reopened = SqliteFullSyncSpendStoreV2::open(&path, "contacts-primary").unwrap();
        assert_eq!(reopened.consume_once(&spend_key), Err(SqliteSpendError::Replay));
    }

    #[test]
    fn same_nonce_different_token_artifact_is_still_replay() {
        let directory = tempdir().unwrap();
        let path = directory.path().join("spends.sqlite3");
        let store = SqliteFullSyncSpendStoreV2::open(&path, "contacts-primary").unwrap();
        let first = key("44", "55");
        let second = key("44", "77");
        assert_eq!(first.commitment_sha256(), second.commitment_sha256());
        store.consume_once(&first).unwrap();
        assert_eq!(store.consume_once(&second), Err(SqliteSpendError::Replay));
    }

    #[test]
    fn two_connections_racing_one_database_have_one_winner() {
        let directory = tempdir().unwrap();
        let path = directory.path().join("spends.sqlite3");
        let left = Arc::new(SqliteFullSyncSpendStoreV2::open(&path, "contacts-primary").unwrap());
        let right = Arc::new(SqliteFullSyncSpendStoreV2::open(&path, "contacts-primary").unwrap());
        let barrier = Arc::new(Barrier::new(2));
        let spend_key = key("44", "55");
        let mut joins = Vec::new();
        for store in [left, right] {
            let barrier = Arc::clone(&barrier);
            let spend_key = spend_key.clone();
            joins.push(thread::spawn(move || {
                barrier.wait();
                store.consume_once(&spend_key).is_ok()
            }));
        }
        assert_eq!(joins.into_iter().filter(|join| join.join().unwrap()).count(), 1);
    }

    #[test]
    fn distinct_nonces_each_commit_once() {
        let directory = tempdir().unwrap();
        let path = directory.path().join("spends.sqlite3");
        let store = SqliteFullSyncSpendStoreV2::open(&path, "contacts-primary").unwrap();
        assert!(store.consume_once(&key("44", "55")).is_ok());
        assert!(store.consume_once(&key("88", "55")).is_ok());
        assert_eq!(store.consumed_count().unwrap(), 2);
    }

    #[test]
    fn wrong_store_instance_identity_is_rejected() {
        let directory = tempdir().unwrap();
        let path = directory.path().join("spends.sqlite3");
        drop(SqliteFullSyncSpendStoreV2::open(&path, "contacts-primary").unwrap());
        assert_eq!(
            SqliteFullSyncSpendStoreV2::open(&path, "other-store").unwrap_err(),
            SqliteSpendError::StoreInstanceIdentityMismatch
        );
    }

    #[test]
    fn schema_version_mismatch_is_rejected() {
        let directory = tempdir().unwrap();
        let path = directory.path().join("spends.sqlite3");
        drop(SqliteFullSyncSpendStoreV2::open(&path, "contacts-primary").unwrap());
        let connection = Connection::open(&path).unwrap();
        connection.pragma_update(None, "user_version", 99_i64).unwrap();
        drop(connection);
        assert_eq!(
            SqliteFullSyncSpendStoreV2::open(&path, "contacts-primary").unwrap_err(),
            SqliteSpendError::SchemaVersionMismatch(99)
        );
    }

    #[test]
    fn persisted_schema_is_minimized_to_nonce_replay_evidence() {
        let directory = tempdir().unwrap();
        let path = directory.path().join("spends.sqlite3");
        drop(SqliteFullSyncSpendStoreV2::open(&path, "contacts-primary").unwrap());
        let connection = Connection::open(&path).unwrap();
        let mut statement = connection
            .prepare("SELECT name FROM pragma_table_info('query_token_spends_v2') ORDER BY cid")
            .unwrap();
        let columns: Vec<String> = statement
            .query_map([], |row| row.get(0))
            .unwrap()
            .map(|row| row.expect("column query must succeed"))
            .collect();
        assert_eq!(
            columns,
            vec![
                "spend_key_sha256",
                "policy_sha256",
                "token_nonce_sha256",
                "requested_identifier_count"
            ]
        );
    }

    #[test]
    fn positive_keeps_higher_authority_false() {
        let directory = tempdir().unwrap();
        let path = directory.path().join("spends.sqlite3");
        let positive = SqliteFullSyncSpendStoreV2::open(&path, "contacts-primary")
            .unwrap()
            .consume_once(&key("44", "55"))
            .unwrap();
        assert!(!positive.token_nonce_cryptographically_bound());
        assert!(!positive.privacy_pass_token_cryptographically_verified());
        assert!(!positive.global_store_uniqueness_established());
        assert!(!positive.hardware_power_loss_durability_established());
        assert!(!positive.safe_compaction_established());
        assert!(!positive.query_credit_granted());
        assert!(!positive.anonymous_rate_limit_established());
        assert!(!positive.enumeration_resistance_established());
        assert!(!positive.application_authority_granted());
        let json = serde_json::to_string(&positive).unwrap();
        assert!(json.contains(SQLITE_SPEND_PROFILE_V2));
        assert!(!json.contains("token_sha256"));
    }
}
