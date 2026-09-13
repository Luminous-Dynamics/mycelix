//! Fail-closed classified transaction owner for runtime claim + expired-lease recovery.
//!
//! This crate replaces the legacy unclassified `claim_outbox()` transition at the
//! integration security boundary. It opens the already-provisioned exact SQLite
//! store without running schema migration, takes an `IMMEDIATE` transaction,
//! captures the exact predecessor state for every row the transaction can touch,
//! performs expired-lease recovery plus deterministic claims, captures the exact
//! candidate successor before COMMIT, and classifies any returned commit error
//! only by independently reopening the exact store.
//!
//! A commit error never reconstructs `ExecutionClaim`s. Even when the candidate
//! successor is later proved durable, the result is recovery-only.

#[cfg(not(unix))]
compile_error!("runtime claim commit classification v0.1 requires Unix file identity semantics");

use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, DigestAlgorithm, ExecutionAttemptId,
    ExternalExecutionOutcome, ExternalOperationRef, IdempotencyKey, ReconciliationHint,
    ReconciliationStrategy, SideEffectClass,
};
use mycelix_integration_runtime::{ExecutionClaim, RecoverySummary};
use mycelix_integration_runtime_store_binding::{
    QualifiedRuntimeStoreBinding, RuntimeStoreBindingError,
};
use mycelix_integration_sqlite_commit_classification::{
    CommitClassification, classify_exact_durable_transition,
};
use rusqlite::{
    Connection, OpenFlags, OptionalExtension, Transaction, TransactionBehavior, params,
};
use std::collections::BTreeMap;
use std::fs;
use std::path::Path;
use std::time::Duration;
use thiserror::Error;

pub const RUNTIME_CLAIM_COMMIT_CLASSIFICATION_PROFILE: &str =
    "mycelix-integration-runtime-claim-commit-classification-v1";

const RUNTIME_SCHEMA_VERSION: i64 = 2;
const OUTBOX_TABLE: &str = "integration_outbox";
const OBSERVATION_TABLE: &str = "integration_execution_observation";
const OUTBOX_COMMITTED: i64 = 3;
const ATTEMPT_PREPARED: i64 = 4;
const DISPATCH_STARTED: i64 = 5;
const AMBIGUOUS: i64 = 8;
const MAX_WORKER_ID_BYTES: usize = 256;
const MAX_OUTBOX_CLAIM: usize = 1024;
const MAX_EXECUTION_ATTEMPTS_PER_ENTRY: i64 = 1024;
const MAX_EXECUTION_OBSERVATIONS_PER_ENTRY: i64 = 4096;
const MAX_RECOVERY_ROWS_PER_CLAIM: usize = 4096;
const MAX_SERIALIZED_OUTCOME_BYTES: usize = 256 * 1024;

include!("types.rs");
include!("transaction.rs");
include!("storage.rs");
include!("errors.rs");
include!("tests.rs");
