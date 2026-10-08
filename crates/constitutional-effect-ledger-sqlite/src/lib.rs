#![deny(unsafe_code)]

//! SQLite-backed durable implementation of the constitutional same-action fence.
//!
//! This crate is host-side infrastructure. It is deliberately not a Holochain
//! WASM dependency and does not claim that DHT links provide a linearizable lock.
//! The SQLite database itself is the shared collision domain: every boundary
//! instance that can cross the same effecting target MUST use the same durable
//! database (or a storage system with equivalent transactional semantics).
//!
//! Every mutating operation uses one BEGIN IMMEDIATE transaction. SQLite permits
//! only one simultaneous write transaction per database; primary-key/UNIQUE
//! constraints provide the durable collision namespaces.

pub mod boundary;

use constitutional_effect_ledger::{
    ActionFenceMutationError, ActionFenceRecordV1, ActionFenceState, ActionKeyV1,
    AtomicAdmissionDecision, AttemptIdentityV1, AttemptRecordState, AttemptRecordV1,
    AuthorizationAdmissionProofV1, DurableActionFenceStore, NativeReplayBindingV1,
    ProviderEntryClaimV1, TerminalEvidenceV1, TerminalOutcomeV1,
    ACTION_FENCE_SCHEMA_VERSION, ATTEMPT_RECORD_SCHEMA_VERSION,
    NATIVE_REPLAY_BINDING_SCHEMA_VERSION,
};
use rusqlite::{params, Connection, OptionalExtension, Transaction, TransactionBehavior};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::time::Duration;

pub const SQLITE_FENCE_STORE_SCHEMA_VERSION: i64 = 6;
pub const SQLITE_FENCE_STORE_PROFILE: &str =
    "constitutional-effect-ledger/sqlite-fence-store-v6";
pub const SQLITE_FENCE_BUSY_TIMEOUT: Duration = Duration::from_secs(5);

const META_TABLE: &str = "effect_fence_store_meta";
const ATTEMPT_TABLE: &str = "effect_attempts";
const FENCE_TABLE: &str = "effect_action_fences";
const REPLAY_TABLE: &str = "effect_native_replay_bindings";
const ENTRY_CLAIM_TABLE: &str = "effect_provider_entry_claims";

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SqliteActionFenceStore {
    path: PathBuf,
}

impl SqliteActionFenceStore {
    /// Open or initialize a dedicated durable fence database.
    ///
    /// Existing databases are fail-closed unless their exact store profile is
    /// present. We do not infer that an unversioned database has compatible
    /// historical semantics.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, String> {
        let path = path.as_ref().to_path_buf();
        let mut conn = open_connection(&path)?;
        ensure_schema(&mut conn)?;
        validate_persisted_state(&conn)?;
        Ok(Self { path })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Re-scan durable state and fail closed on cross-record inconsistency.
    ///
    /// This is intentionally separate from the hot-path local transition checks:
    /// operators can run it after restart, before promotion, and during periodic
    /// storage qualification without silently trusting cached projections.
    pub fn audit_integrity(&self) -> Result<(), String> {
        let mut conn = open_connection(&self.path)?;
        ensure_schema(&mut conn)?;
        validate_persisted_state(&conn)
    }

    fn with_transaction<F, T>(&self, f: F) -> Result<T, ActionFenceMutationError>
    where
        F: FnOnce(&Transaction<'_>) -> Result<T, ActionFenceMutationError>,
    {
        let mut conn = open_connection(&self.path).map_err(storage_error)?;
        let tx = conn
            .transaction_with_behavior(TransactionBehavior::Immediate)
            .map_err(storage_error)?;
        let value = f(&tx)?;
        tx.commit().map_err(storage_error)?;
        Ok(value)
    }

    fn transition_state(
        &self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        next_state: AttemptRecordState,
    ) -> Result<(), ActionFenceMutationError> {
        self.with_transaction(|tx| {
            let current = load_attempt_tx(tx, attempt_identity.digest())
                .map_err(storage_error)?
                .ok_or(ActionFenceMutationError::NotOwner)?;

            verify_owner_and_key(
                &current,
                action_key,
                attempt_identity,
                owner_token_digest,
            )?;

            if current.state.is_terminal() {
                return Err(ActionFenceMutationError::AlreadyClosed);
            }
            if !current.state.allows_transition_to(next_state) {
                return Err(ActionFenceMutationError::InvalidTransition);
            }
            if matches!(
                next_state,
                AttemptRecordState::DispatchPending
                    | AttemptRecordState::Invoked
                    | AttemptRecordState::Indeterminate
            ) && (current.provider_reference_seed_digest.is_none()
                || current.provider_reference_descriptor_digest.is_none())
            {
                return Err(ActionFenceMutationError::InvalidTransition);
            }

            let fence = load_fence_tx(tx, action_key.digest())
                .map_err(storage_error)?
                .ok_or(ActionFenceMutationError::NotOccupied)?;
            verify_fence_owner(&fence, attempt_identity, owner_token_digest)?;

            if next_state == AttemptRecordState::Indeterminate
                && load_provider_entry_claim_tx(tx, attempt_identity.digest())
                    .map_err(storage_error)?
                    .is_some()
            {
                return Err(ActionFenceMutationError::ProviderEntryClaimed);
            }

            let mut updated = current.clone();
            updated.state = next_state;
            if next_state == AttemptRecordState::Indeterminate {
                updated.reconciliation_token_digest =
                    Some(updated.reconciliation_token_digest());
            } else {
                updated.reconciliation_token_digest = None;
            }
            update_attempt_tx(tx, &current, &updated)?;
            Ok(())
        })
    }

    fn terminal_transition(
        &self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        terminal_evidence: &TerminalEvidenceV1,
        next_state: AttemptRecordState,
    ) -> Result<(), ActionFenceMutationError> {
        self.with_transaction(|tx| {
            let current = load_attempt_tx(tx, attempt_identity.digest())
                .map_err(storage_error)?
                .ok_or(ActionFenceMutationError::NotOwner)?;

            if current.state == AttemptRecordState::Indeterminate {
                if current.reconciliation_token_digest.as_deref() != Some(owner_token_digest) {
                    return Err(ActionFenceMutationError::OwnershipTokenMismatch);
                }
            } else {
                verify_owner_and_key(
                    &current,
                    action_key,
                    attempt_identity,
                    owner_token_digest,
                )?;
            }

            if current.state.is_terminal() {
                return Err(ActionFenceMutationError::AlreadyClosed);
            }
            if !matches!(
                current.state,
                AttemptRecordState::Invoked | AttemptRecordState::Indeterminate
            ) {
                return Err(ActionFenceMutationError::InvalidTransition);
            }

            let expected_outcome = match next_state {
                AttemptRecordState::Executed => TerminalOutcomeV1::Executed,
                AttemptRecordState::Failed => TerminalOutcomeV1::Failed,
                _ => return Err(ActionFenceMutationError::InvalidTransition),
            };
            if terminal_evidence.outcome() != expected_outcome
                || terminal_evidence.action_key_digest() != action_key.digest()
                || terminal_evidence.attempt_identity() != attempt_identity.digest()
                || terminal_evidence.operation_id() != current.operation_id
                || terminal_evidence.native_replay_identity() != current.native_replay_identity
                || terminal_evidence.effecting_target_identity()
                    != current.effecting_target_identity
                || terminal_evidence.provider_environment() != current.provider_environment
                || terminal_evidence.provider_audience() != current.provider_audience
                || terminal_evidence.adapter_identity() != current.adapter_identity
            {
                return Err(ActionFenceMutationError::TerminalEvidenceMismatch);
            }

            let fence = load_fence_tx(tx, action_key.digest())
                .map_err(storage_error)?
                .ok_or(ActionFenceMutationError::NotOccupied)?;
            if current.state == AttemptRecordState::Indeterminate {
                if fence.owner_attempt_identity != attempt_identity.digest()
                    || fence.state.is_closed()
                {
                    return Err(ActionFenceMutationError::NotOwner);
                }
            } else {
                verify_fence_owner(&fence, attempt_identity, owner_token_digest)?;
            }

            let mut updated = current.clone();
            updated.state = next_state;
            updated.reconciliation_token_digest = None;
            updated.terminal_evidence_digest = Some(terminal_evidence.digest().to_owned());
            update_attempt_tx(tx, &current, &updated)?;

            if next_state == AttemptRecordState::Executed {
                let changed = tx
                    .execute(
                        "UPDATE effect_action_fences
                         SET state = ?1,
                             record_digest = ?6
                         WHERE action_key_digest = ?2
                           AND state = ?3
                           AND owner_attempt_identity = ?4
                           AND owner_token_digest = ?5",
                        params![
                            ActionFenceState::Closed.storage_tag(),
                            action_key.digest(),
                            ActionFenceState::Occupied.storage_tag(),
                            attempt_identity.digest(),
                            fence.owner_token_digest,
                            {
                                let mut updated_fence = fence.clone();
                                updated_fence.state = ActionFenceState::Closed;
                                updated_fence.record_digest()
                            }
                        ],
                    )
                    .map_err(storage_error)?;
                if changed != 1 {
                    return Err(ActionFenceMutationError::NotOccupied);
                }
            } else {
                let changed = tx
                    .execute(
                        "DELETE FROM effect_action_fences
                         WHERE action_key_digest = ?1
                           AND state = ?2
                           AND owner_attempt_identity = ?3
                           AND owner_token_digest = ?4",
                        params![
                            action_key.digest(),
                            ActionFenceState::Occupied.storage_tag(),
                            attempt_identity.digest(),
                            fence.owner_token_digest
                        ],
                    )
                    .map_err(storage_error)?;
                if changed != 1 {
                    return Err(ActionFenceMutationError::NotOccupied);
                }
            }
            Ok(())
        })
    }
}

impl DurableActionFenceStore for SqliteActionFenceStore {
    fn atomically_admit(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        mut record: AttemptRecordV1,
        authorization_proof: AuthorizationAdmissionProofV1,
    ) -> Result<AtomicAdmissionDecision, String> {
        if record.authorization_admission_proof_digest.is_some() {
            return Err(
                "attempt record must not supply its own authorization admission proof digest"
                    .into(),
            );
        }
        if !authorization_proof.matches(&record, action_key) {
            return Err("authorization admission proof does not match exact attempt/action".into());
        }
        record.authorization_admission_proof_digest = Some(authorization_proof.digest().to_owned());
        record.validate()?;
        if record.action_key_digest != action_key.digest()
            || record.action_digest != action_key.material_action_digest()
            || record.effecting_target_identity != action_key.effecting_target_identity()
        {
            return Err("attempt record does not match typed ActionKeyV1".into());
        }
        if record.attempt_identity != attempt_identity.digest() {
            return Err("attempt record does not match typed AttemptIdentityV1".into());
        }
        if !record.state.occupies_action_fence() {
            return Err("admitted attempt must occupy the action fence".into());
        }

        let mut conn = open_connection(&self.path)?;
        let tx = conn
            .transaction_with_behavior(TransactionBehavior::Immediate)
            .map_err(|e| e.to_string())?;

        if let Some(existing) = load_attempt_tx(&tx, attempt_identity.digest())? {
            if existing.record_digest() == record.record_digest() {
                if existing.state.occupies_action_fence() {
                    match load_fence_tx(&tx, action_key.digest())? {
                        Some(fence)
                            if fence.state == ActionFenceState::Occupied
                                && fence.owner_attempt_identity == record.attempt_identity
                                && fence.owner_token_digest == record.ownership_token_digest => {}
                        _ => return Ok(AtomicAdmissionDecision::AttemptOwnershipConflict),
                    }
                }
                return Ok(AtomicAdmissionDecision::DuplicateAttempt);
            }
            return Ok(AtomicAdmissionDecision::AttemptOwnershipConflict);
        }

        if let Some(existing) = load_replay_tx(&tx, &record.native_replay_identity)? {
            if existing.operation_id != record.operation_id
                || existing.action_key_digest != record.action_key_digest
            {
                return Ok(AtomicAdmissionDecision::NativeReplayConflict {
                    existing_operation_id: existing.operation_id,
                    existing_action_key_digest: existing.action_key_digest,
                });
            }
        }

        if let Some(existing) = load_fence_tx(&tx, action_key.digest())? {
            if existing.state.is_closed() {
                return Ok(AtomicAdmissionDecision::ActionAlreadyExecuted);
            }
            if existing.owner_attempt_identity != record.attempt_identity {
                return Ok(AtomicAdmissionDecision::ActionInFlight);
            }
            return Ok(AtomicAdmissionDecision::AttemptOwnershipConflict);
        }

        let replay = NativeReplayBindingV1 {
            schema_version: NATIVE_REPLAY_BINDING_SCHEMA_VERSION,
            native_replay_identity: record.native_replay_identity.clone(),
            operation_id: record.operation_id.clone(),
            action_key_digest: record.action_key_digest.clone(),
        };
        replay.validate()?;

        tx.execute(
            "INSERT INTO effect_native_replay_bindings
             (native_replay_identity, operation_id, action_key_digest, record_digest)
             VALUES (?1, ?2, ?3, ?4)",
            params![
                replay.native_replay_identity,
                replay.operation_id,
                replay.action_key_digest,
                replay.digest()
            ],
        )
        .map_err(|e| e.to_string())?;

        insert_attempt_tx(&tx, &record)?;

        let fence = ActionFenceRecordV1 {
            schema_version: ACTION_FENCE_SCHEMA_VERSION,
            action_key_digest: record.action_key_digest.clone(),
            owner_attempt_identity: record.attempt_identity.clone(),
            owner_token_digest: record.ownership_token_digest.clone(),
            state: ActionFenceState::Occupied,
        };
        fence.validate()?;

        tx.execute(
            "INSERT INTO effect_action_fences
             (action_key_digest, owner_attempt_identity, owner_token_digest, state, record_digest)
             VALUES (?1, ?2, ?3, ?4, ?5)",
            params![
                fence.action_key_digest,
                fence.owner_attempt_identity,
                fence.owner_token_digest,
                fence.state.storage_tag(),
                fence.record_digest()
            ],
        )
        .map_err(|e| e.to_string())?;

        tx.commit().map_err(|e| e.to_string())?;
        Ok(AtomicAdmissionDecision::Admitted)
    }

    fn atomically_record_provider_entry_proof(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        proof_digest: String,
    ) -> Result<(), ActionFenceMutationError> {
        self.with_transaction(|tx| {
            if proof_digest.trim().is_empty() || proof_digest.len() > 512 {
                return Err(ActionFenceMutationError::InvalidTransition);
            }
            let current = load_attempt_tx(tx, attempt_identity.digest())
                .map_err(storage_error)?
                .ok_or(ActionFenceMutationError::NotOwner)?;
            verify_owner_and_key(&current, action_key, attempt_identity, owner_token_digest)?;
            if current.state != AttemptRecordState::DispatchPending {
                return Err(ActionFenceMutationError::InvalidTransition);
            }
            if current.entry_admission_proof_digest.is_some() {
                return Err(ActionFenceMutationError::ProviderEntryProofAlreadyRecorded);
            }
            let claim = load_provider_entry_claim_tx(tx, attempt_identity.digest())
                .map_err(storage_error)?
                .ok_or(ActionFenceMutationError::ProviderEntryClaimMismatch)?;
            if claim.action_key_digest != action_key.digest()
                || claim.owner_token_digest != owner_token_digest
            {
                return Err(ActionFenceMutationError::ProviderEntryClaimMismatch);
            }

            let mut updated = current.clone();
            updated.entry_admission_proof_digest = Some(proof_digest);
            update_attempt_tx(tx, &current, &updated)?;
            Ok(())
        })
    }

    fn atomically_mark_dispatch_pending(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
    ) -> Result<(), ActionFenceMutationError> {
        self.transition_state(
            action_key,
            attempt_identity,
            owner_token_digest,
            AttemptRecordState::DispatchPending,
        )
    }

    fn atomically_claim_provider_entry(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        claim_token_digest: &str,
    ) -> Result<ProviderEntryClaimV1, ActionFenceMutationError> {
        self.with_transaction(|tx| {
            let current = load_attempt_tx(tx, attempt_identity.digest())
                .map_err(storage_error)?
                .ok_or(ActionFenceMutationError::NotOwner)?;
            verify_owner_and_key(&current, action_key, attempt_identity, owner_token_digest)?;
            if current.state != AttemptRecordState::DispatchPending {
                return Err(ActionFenceMutationError::InvalidTransition);
            }
            if load_provider_entry_claim_tx(tx, attempt_identity.digest())
                .map_err(storage_error)?.is_some()
            {
                return Err(ActionFenceMutationError::ProviderEntryClaimed);
            }
            let fence = load_fence_tx(tx, action_key.digest())
                .map_err(storage_error)?
                .ok_or(ActionFenceMutationError::NotOccupied)?;
            verify_fence_owner(&fence, attempt_identity, owner_token_digest)?;
            let claim = ProviderEntryClaimV1::new(
                attempt_identity.digest(), action_key.digest(), owner_token_digest, claim_token_digest,
            ).map_err(storage_error)?;
            tx.execute(
                "INSERT INTO effect_provider_entry_claims
                 (attempt_identity, action_key_digest, owner_token_digest, claim_token_digest, record_digest)
                 VALUES (?1, ?2, ?3, ?4, ?5)",
                params![claim.attempt_identity, claim.action_key_digest, claim.owner_token_digest,
                        claim.claim_token_digest, claim.record_digest()],
            ).map_err(storage_error)?;
            Ok(claim)
        })
    }

    fn atomically_mark_invoked(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        claim_token_digest: &str,
    ) -> Result<(), ActionFenceMutationError> {
        self.with_transaction(|tx| {
            let current = load_attempt_tx(tx, attempt_identity.digest())
                .map_err(storage_error)?
                .ok_or(ActionFenceMutationError::NotOwner)?;
            verify_owner_and_key(&current, action_key, attempt_identity, owner_token_digest)?;
            if current.state != AttemptRecordState::DispatchPending {
                return Err(ActionFenceMutationError::InvalidTransition);
            }
            let claim = load_provider_entry_claim_tx(tx, attempt_identity.digest())
                .map_err(storage_error)?
                .ok_or(ActionFenceMutationError::ProviderEntryClaimMismatch)?;
            if claim.action_key_digest != action_key.digest()
                || claim.owner_token_digest != owner_token_digest
                || claim.claim_token_digest != claim_token_digest
            {
                return Err(ActionFenceMutationError::ProviderEntryClaimMismatch);
            }
            let mut updated = current.clone();
            updated.state = AttemptRecordState::Invoked;
            updated.reconciliation_token_digest = None;
            update_attempt_tx(tx, &current, &updated)?;
            let changed = tx.execute(
                "DELETE FROM effect_provider_entry_claims
                 WHERE attempt_identity = ?1 AND action_key_digest = ?2
                   AND owner_token_digest = ?3 AND claim_token_digest = ?4
                   AND record_digest = ?5",
                params![attempt_identity.digest(), action_key.digest(), owner_token_digest,
                        claim_token_digest, claim.record_digest()],
            ).map_err(storage_error)?;
            if changed != 1 {
                return Err(ActionFenceMutationError::ProviderEntryClaimMismatch);
            }
            Ok(())
        })
    }

    fn atomically_release_provider_entry_claim_not_entered(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        claim_token_digest: &str,
        marker: String,
    ) -> Result<(), ActionFenceMutationError> {
        if marker.trim().is_empty() || marker.len() > 512 {
            return Err(ActionFenceMutationError::InvalidTransition);
        }

        self.with_transaction(|tx| {
            let current = load_attempt_tx(tx, attempt_identity.digest())
                .map_err(storage_error)?
                .ok_or(ActionFenceMutationError::NotOwner)?;
            verify_owner_and_key(&current, action_key, attempt_identity, owner_token_digest)?;
            if current.state != AttemptRecordState::DispatchPending {
                return Err(ActionFenceMutationError::InvalidTransition);
            }

            let claim = load_provider_entry_claim_tx(tx, attempt_identity.digest())
                .map_err(storage_error)?
                .ok_or(ActionFenceMutationError::ProviderEntryClaimMismatch)?;
            if claim.action_key_digest != action_key.digest()
                || claim.owner_token_digest != owner_token_digest
                || claim.claim_token_digest != claim_token_digest
            {
                return Err(ActionFenceMutationError::ProviderEntryClaimMismatch);
            }

            let fence = load_fence_tx(tx, action_key.digest())
                .map_err(storage_error)?
                .ok_or(ActionFenceMutationError::NotOccupied)?;
            verify_fence_owner(&fence, attempt_identity, owner_token_digest)?;

            let mut updated = current.clone();
            updated.state = AttemptRecordState::NotEntered;
            updated.not_entered_marker = Some(marker);
            updated.entry_admission_proof_digest = None;
            updated.validate().map_err(storage_error)?;
            update_attempt_tx(tx, &current, &updated)?;

            let claim_deleted = tx
                .execute(
                    "DELETE FROM effect_provider_entry_claims
                     WHERE attempt_identity = ?1
                       AND action_key_digest = ?2
                       AND owner_token_digest = ?3
                       AND claim_token_digest = ?4
                       AND record_digest = ?5",
                    params![
                        attempt_identity.digest(),
                        action_key.digest(),
                        owner_token_digest,
                        claim_token_digest,
                        claim.record_digest()
                    ],
                )
                .map_err(storage_error)?;
            if claim_deleted != 1 {
                return Err(ActionFenceMutationError::ProviderEntryClaimMismatch);
            }

            let fence_deleted = tx
                .execute(
                    "DELETE FROM effect_action_fences
                     WHERE action_key_digest = ?1
                       AND state = ?2
                       AND owner_attempt_identity = ?3
                       AND owner_token_digest = ?4",
                    params![
                        action_key.digest(),
                        ActionFenceState::Occupied.storage_tag(),
                        attempt_identity.digest(),
                        owner_token_digest
                    ],
                )
                .map_err(storage_error)?;
            if fence_deleted != 1 {
                return Err(ActionFenceMutationError::NotOccupied);
            }
            Ok(())
        })
    }

    fn atomically_recover_provider_entry_claim(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        claim_token_digest: &str,
    ) -> Result<String, ActionFenceMutationError> {
        self.with_transaction(|tx| {
            let current = load_attempt_tx(tx, attempt_identity.digest())
                .map_err(storage_error)?
                .ok_or(ActionFenceMutationError::NotOwner)?;
            verify_owner_and_key(&current, action_key, attempt_identity, owner_token_digest)?;
            if current.state != AttemptRecordState::DispatchPending {
                return Err(ActionFenceMutationError::InvalidTransition);
            }
            let claim = load_provider_entry_claim_tx(tx, attempt_identity.digest())
                .map_err(storage_error)?
                .ok_or(ActionFenceMutationError::ProviderEntryClaimMismatch)?;
            if claim.action_key_digest != action_key.digest()
                || claim.owner_token_digest != owner_token_digest
                || claim.claim_token_digest != claim_token_digest
            {
                return Err(ActionFenceMutationError::ProviderEntryClaimMismatch);
            }

            let mut updated = current.clone();
            updated.state = AttemptRecordState::Indeterminate;
            updated.reconciliation_token_digest = Some(updated.reconciliation_token_digest());
            let reconciliation_token = updated
                .reconciliation_token_digest
                .clone()
                .ok_or(ActionFenceMutationError::InvalidTransition)?;
            update_attempt_tx(tx, &current, &updated)?;

            let changed = tx
                .execute(
                    "DELETE FROM effect_provider_entry_claims
                     WHERE attempt_identity = ?1
                       AND action_key_digest = ?2
                       AND owner_token_digest = ?3
                       AND claim_token_digest = ?4
                       AND record_digest = ?5",
                    params![
                        attempt_identity.digest(),
                        action_key.digest(),
                        owner_token_digest,
                        claim_token_digest,
                        claim.record_digest()
                    ],
                )
                .map_err(storage_error)?;
            if changed != 1 {
                return Err(ActionFenceMutationError::ProviderEntryClaimMismatch);
            }
            Ok(reconciliation_token)
        })
    }

    fn atomically_mark_indeterminate(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
    ) -> Result<String, ActionFenceMutationError> {
        self.transition_state(
            action_key,
            attempt_identity,
            owner_token_digest,
            AttemptRecordState::Indeterminate,
        )?;
        self.durably_read_attempt(attempt_identity)
            .map_err(storage_error)?
            .and_then(|attempt| attempt.reconciliation_token_digest)
            .ok_or(ActionFenceMutationError::InvalidTransition)
    }

    fn atomically_release_after_failed(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        terminal_evidence: &TerminalEvidenceV1,
    ) -> Result<(), ActionFenceMutationError> {
        self.terminal_transition(
            action_key,
            attempt_identity,
            owner_token_digest,
            terminal_evidence,
            AttemptRecordState::Failed,
        )
    }

    fn atomically_close_executed(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        terminal_evidence: &TerminalEvidenceV1,
    ) -> Result<(), ActionFenceMutationError> {
        self.terminal_transition(
            action_key,
            attempt_identity,
            owner_token_digest,
            terminal_evidence,
            AttemptRecordState::Executed,
        )
    }

    fn atomically_release_not_entered(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        marker: String,
    ) -> Result<(), ActionFenceMutationError> {
        if marker.trim().is_empty() || marker.len() > 512 {
            return Err(ActionFenceMutationError::InvalidTransition);
        }

        self.with_transaction(|tx| {
            let current = load_attempt_tx(tx, attempt_identity.digest())
                .map_err(storage_error)?
                .ok_or(ActionFenceMutationError::NotOwner)?;
            verify_owner_and_key(
                &current,
                action_key,
                attempt_identity,
                owner_token_digest,
            )?;
            if !current.state.allows_not_entered_release() {
                return Err(ActionFenceMutationError::NotOccupied);
            }
            let fence = load_fence_tx(tx, action_key.digest())
                .map_err(storage_error)?
                .ok_or(ActionFenceMutationError::NotOccupied)?;
            verify_fence_owner(&fence, attempt_identity, owner_token_digest)?;

            let mut updated = current.clone();
            updated.state = AttemptRecordState::NotEntered;
            updated.not_entered_marker = Some(marker);
            update_attempt_tx(tx, &current, &updated)?;

            let changed = tx
                .execute(
                    "DELETE FROM effect_action_fences
                     WHERE action_key_digest = ?1
                       AND state = ?2
                       AND owner_attempt_identity = ?3
                       AND owner_token_digest = ?4",
                    params![
                        action_key.digest(),
                        ActionFenceState::Occupied.storage_tag(),
                        attempt_identity.digest(),
                        owner_token_digest
                    ],
                )
                .map_err(storage_error)?;
            if changed != 1 {
                return Err(ActionFenceMutationError::NotOccupied);
            }
            Ok(())
        })
    }

    fn durably_read_attempt(
        &self,
        attempt_identity: &AttemptIdentityV1,
    ) -> Result<Option<AttemptRecordV1>, String> {
        let conn = open_connection(&self.path)?;
        load_attempt_txless(&conn, attempt_identity.digest())
    }

    fn durably_read_fence(
        &self,
        action_key: &ActionKeyV1,
    ) -> Result<Option<ActionFenceRecordV1>, String> {
        let conn = open_connection(&self.path)?;
        load_fence_txless(&conn, action_key.digest())
    }

    fn durably_read_replay_binding(
        &self,
        native_replay_identity: &str,
    ) -> Result<Option<NativeReplayBindingV1>, String> {
        let conn = open_connection(&self.path)?;
        load_replay_txless(&conn, native_replay_identity)
    }

    fn durably_read_provider_entry_claim(
        &self,
        attempt_identity: &AttemptIdentityV1,
    ) -> Result<Option<ProviderEntryClaimV1>, String> {
        let conn = open_connection(&self.path)?;
        load_provider_entry_claim_txless(&conn, attempt_identity.digest())
    }
}

fn storage_error(error: impl ToString) -> ActionFenceMutationError {
    ActionFenceMutationError::StorageFailure(error.to_string())
}

fn verify_owner_and_key(
    current: &AttemptRecordV1,
    action_key: &ActionKeyV1,
    attempt_identity: &AttemptIdentityV1,
    owner_token_digest: &str,
) -> Result<(), ActionFenceMutationError> {
    if current.ownership_token_digest != owner_token_digest {
        return Err(ActionFenceMutationError::OwnershipTokenMismatch);
    }
    if current.attempt_identity != attempt_identity.digest()
        || current.action_key_digest != action_key.digest()
    {
        return Err(ActionFenceMutationError::NotOwner);
    }
    Ok(())
}

fn verify_fence_owner(
    fence: &ActionFenceRecordV1,
    attempt_identity: &AttemptIdentityV1,
    owner_token_digest: &str,
) -> Result<(), ActionFenceMutationError> {
    if fence.owner_attempt_identity != attempt_identity.digest() {
        return Err(ActionFenceMutationError::NotOwner);
    }
    if fence.owner_token_digest != owner_token_digest {
        return Err(ActionFenceMutationError::OwnershipTokenMismatch);
    }
    if fence.state.is_closed() {
        return Err(ActionFenceMutationError::AlreadyClosed);
    }
    Ok(())
}

fn open_connection(path: &Path) -> Result<Connection, String> {
    let conn = Connection::open(path).map_err(|e| e.to_string())?;
    conn.busy_timeout(SQLITE_FENCE_BUSY_TIMEOUT)
        .map_err(|e| e.to_string())?;
    conn.pragma_update(None, "foreign_keys", "ON")
        .map_err(|e| e.to_string())?;
    conn.pragma_update(None, "synchronous", "FULL")
        .map_err(|e| e.to_string())?;
    Ok(conn)
}

fn ensure_schema(conn: &mut Connection) -> Result<(), String> {
    let meta_exists = {
        let mut stmt = conn
            .prepare(
                "SELECT EXISTS(
                    SELECT 1 FROM sqlite_master
                    WHERE type = 'table' AND name = ?1
                )",
            )
            .map_err(|e| e.to_string())?;
        stmt.query_row(params![META_TABLE], |row| row.get::<_, bool>(0))
            .map_err(|e| e.to_string())?
    };

    if !meta_exists {
        conn.pragma_update(None, "journal_mode", "WAL")
            .map_err(|e| e.to_string())?;

        // Bootstrap under the same write lock used by every durable mutation.
        // The metadata check is repeated inside the transaction so two boundary
        // instances opening an empty database cannot both create the schema.
        let tx = conn
            .transaction_with_behavior(TransactionBehavior::Immediate)
            .map_err(|e| e.to_string())?;

        let tx_meta_exists: bool = tx
            .query_row(
                "SELECT EXISTS(
                    SELECT 1 FROM sqlite_master
                    WHERE type = 'table' AND name = ?1
                )",
                params![META_TABLE],
                |row| row.get(0),
            )
            .map_err(|e| e.to_string())?;

        if !tx_meta_exists {
            let user_table_exists: bool = tx
                .query_row(
                    "SELECT EXISTS(
                        SELECT 1 FROM sqlite_master
                        WHERE type = 'table'
                          AND name NOT LIKE 'sqlite_%'
                    )",
                    [],
                    |row| row.get(0),
                )
                .map_err(|e| e.to_string())?;

            if user_table_exists {
                return Err(
                    "unversioned or foreign SQLite state cannot be adopted as an effect fence store"
                        .into(),
                );
            }

            tx.execute_batch(SCHEMA_SQL).map_err(|e| e.to_string())?;
            tx.execute_batch(&format!(
                "PRAGMA user_version = {};",
                SQLITE_FENCE_STORE_SCHEMA_VERSION
            ))
            .map_err(|e| e.to_string())?;
            tx.execute(
                "INSERT INTO effect_fence_store_meta
                 (singleton, schema_version, profile)
                 VALUES (1, ?1, ?2)",
                params![
                    SQLITE_FENCE_STORE_SCHEMA_VERSION,
                    SQLITE_FENCE_STORE_PROFILE
                ],
            )
            .map_err(|e| e.to_string())?;
        }

        tx.commit().map_err(|e| e.to_string())?;
    }

    let (version, profile): (i64, String) = conn
        .query_row(
            "SELECT schema_version, profile
             FROM effect_fence_store_meta
             WHERE singleton = 1",
            [],
            |row| Ok((row.get(0)?, row.get(1)?)),
        )
        .map_err(|e| e.to_string())?;

    if version != SQLITE_FENCE_STORE_SCHEMA_VERSION
        || profile != SQLITE_FENCE_STORE_PROFILE
    {
        return Err(format!(
            "unsupported effect fence store profile: version={version} profile={profile}"
        ));
    }

    let user_version: i64 = conn
        .pragma_query_value(None, "user_version", |row| row.get(0))
        .map_err(|e| e.to_string())?;
    if user_version != SQLITE_FENCE_STORE_SCHEMA_VERSION {
        return Err(format!(
            "SQLite user_version {user_version} does not match durable fence schema {}",
            SQLITE_FENCE_STORE_SCHEMA_VERSION
        ));
    }

    validate_schema_columns(conn)?;
    validate_no_managed_triggers(conn)?;
    validate_no_explicit_managed_indexes(conn)?;
    validate_foreign_keys(conn)?;
    Ok(())
}

fn validate_schema_columns(conn: &Connection) -> Result<(), String> {
    let expectations: [(&str, &[(&str, &str, i64, i64)]); 5] = [
        (
            META_TABLE,
            &[
                ("singleton", "INTEGER", 1, 1),
                ("schema_version", "INTEGER", 1, 0),
                ("profile", "TEXT", 1, 0),
            ],
        ),
        (
            ATTEMPT_TABLE,
            &[
                ("attempt_identity", "TEXT", 1, 1),
                ("schema_version", "INTEGER", 1, 0),
                ("operation_id", "TEXT", 1, 0),
                ("native_replay_identity", "TEXT", 1, 0),
                ("action_digest", "TEXT", 1, 0),
                ("action_key_digest", "TEXT", 1, 0),
                ("effecting_target_identity", "TEXT", 1, 0),
                ("provider_reference_seed_digest", "TEXT", 0, 0),
                ("provider_reference_descriptor_digest", "TEXT", 0, 0),
                ("provider_environment", "TEXT", 1, 0),
                ("provider_audience", "TEXT", 1, 0),
                ("adapter_identity", "TEXT", 1, 0),
                ("provider_idempotency_key", "TEXT", 1, 0),
                ("authorization_admission_proof_digest", "TEXT", 0, 0),
                ("entry_admission_proof_digest", "TEXT", 0, 0),
                ("ownership_token_digest", "TEXT", 1, 0),
                ("reconciliation_token_digest", "TEXT", 0, 0),
                ("terminal_evidence_digest", "TEXT", 0, 0),
                ("state", "INTEGER", 1, 0),
                ("not_entered_marker", "TEXT", 0, 0),
                ("record_digest", "TEXT", 1, 0),
            ],
        ),
        (
            FENCE_TABLE,
            &[
                ("action_key_digest", "TEXT", 1, 1),
                ("owner_attempt_identity", "TEXT", 1, 0),
                ("owner_token_digest", "TEXT", 1, 0),
                ("state", "INTEGER", 1, 0),
                ("record_digest", "TEXT", 1, 0),
            ],
        ),
        (
            REPLAY_TABLE,
            &[
                ("native_replay_identity", "TEXT", 1, 1),
                ("operation_id", "TEXT", 1, 0),
                ("action_key_digest", "TEXT", 1, 0),
                ("record_digest", "TEXT", 1, 0),
            ],
        ),
        (
            ENTRY_CLAIM_TABLE,
            &[
                ("attempt_identity", "TEXT", 1, 1),
                ("action_key_digest", "TEXT", 1, 0),
                ("owner_token_digest", "TEXT", 1, 0),
                ("claim_token_digest", "TEXT", 1, 0),
                ("record_digest", "TEXT", 1, 0),
            ],
        ),
    ];

    for (table, expected) in expectations {
        let mut stmt = conn
            .prepare(&format!("PRAGMA table_info({table})"))
            .map_err(|e| e.to_string())?;
        let rows = stmt
            .query_map([], |row| {
                Ok((
                    row.get::<_, String>(1)?,
                    row.get::<_, String>(2)?,
                    row.get::<_, i64>(3)?,
                    row.get::<_, i64>(5)?,
                ))
            })
            .map_err(|e| e.to_string())?;
        let actual = rows
            .collect::<Result<Vec<_>, _>>()
            .map_err(|e| e.to_string())?;

        if actual.len() != expected.len() {
            return Err(format!(
                "{table} column count mismatch: expected {} got {}",
                expected.len(),
                actual.len()
            ));
        }
        for (idx, (name, kind, not_null, pk)) in expected.iter().enumerate() {
            if actual[idx]
                != (name.to_string(), kind.to_string(), *not_null, *pk)
            {
                return Err(format!(
                    "{table} column {} mismatch: expected {:?} got {:?}",
                    idx, (name, kind, not_null, pk), actual[idx]
                ));
            }
        }
    }
    Ok(())
}

fn validate_no_managed_triggers(conn: &Connection) -> Result<(), String> {
    let exists: bool = conn
        .query_row(
            "SELECT EXISTS(
                SELECT 1 FROM sqlite_master
                WHERE type = 'trigger'
                  AND tbl_name IN (?1, ?2, ?3, ?4, ?5)
            )",
            params![META_TABLE, ATTEMPT_TABLE, FENCE_TABLE, REPLAY_TABLE, ENTRY_CLAIM_TABLE],
            |row| row.get(0),
        )
        .map_err(|e| e.to_string())?;
    if exists {
        return Err("managed effect-fence tables contain an unqualified trigger".into());
    }
    Ok(())
}

fn validate_no_explicit_managed_indexes(conn: &Connection) -> Result<(), String> {
    let exists: bool = conn
        .query_row(
            "SELECT EXISTS(
                SELECT 1 FROM sqlite_master
                WHERE type = 'index'
                  AND sql IS NOT NULL
                  AND tbl_name IN (?1, ?2, ?3, ?4, ?5)
            )",
            params![META_TABLE, ATTEMPT_TABLE, FENCE_TABLE, REPLAY_TABLE, ENTRY_CLAIM_TABLE],
            |row| row.get(0),
        )
        .map_err(|e| e.to_string())?;
    if exists {
        return Err(
            "managed effect-fence tables contain an unexpected explicit index".into(),
        );
    }
    Ok(())
}

fn validate_persisted_state(conn: &Connection) -> Result<(), String> {
    let integrity: String = conn
        .query_row("PRAGMA integrity_check", [], |row| row.get(0))
        .map_err(|e| e.to_string())?;
    if integrity != "ok" {
        return Err(format!("SQLite integrity_check failed: {integrity}"));
    }

    let mut attempts = HashMap::new();
    let mut stmt = conn
        .prepare(&format!(
            "SELECT attempt_identity, schema_version, operation_id, native_replay_identity,
                    action_digest, action_key_digest, effecting_target_identity,
                    provider_reference_seed_digest, provider_reference_descriptor_digest,
                    provider_environment, provider_audience, adapter_identity,
                    provider_idempotency_key, authorization_admission_proof_digest,
                    entry_admission_proof_digest, ownership_token_digest,
                    reconciliation_token_digest,
                    terminal_evidence_digest, state,
                    not_entered_marker, record_digest
             FROM {ATTEMPT_TABLE}
             ORDER BY attempt_identity"
        ))
        .map_err(|e| e.to_string())?;
    let rows = stmt
        .query_map([], map_attempt_row)
        .map_err(|e| e.to_string())?;
    for row in rows {
        let record = row.map_err(|e| e.to_string())?;
        attempts.insert(record.attempt_identity.clone(), record);
    }

    let mut fences = HashMap::new();
    let mut stmt = conn
        .prepare(&format!(
            "SELECT action_key_digest, owner_attempt_identity, owner_token_digest, state, record_digest
             FROM {FENCE_TABLE}
             ORDER BY action_key_digest"
        ))
        .map_err(|e| e.to_string())?;
    let rows = stmt
        .query_map([], map_fence_row)
        .map_err(|e| e.to_string())?;
    for row in rows {
        let fence = row.map_err(|e| e.to_string())?;
        fences.insert(fence.action_key_digest.clone(), fence);
    }

    let mut replays = HashMap::new();
    let mut stmt = conn
        .prepare(&format!(
            "SELECT native_replay_identity, operation_id, action_key_digest, record_digest
             FROM {REPLAY_TABLE}
             ORDER BY native_replay_identity"
        ))
        .map_err(|e| e.to_string())?;
    let rows = stmt
        .query_map([], map_replay_row)
        .map_err(|e| e.to_string())?;
    for row in rows {
        let replay = row.map_err(|e| e.to_string())?;
        replays.insert(replay.native_replay_identity.clone(), replay);
    }

    let mut claims = HashMap::new();
    let mut stmt = conn
        .prepare(&format!(
            "SELECT attempt_identity, action_key_digest, owner_token_digest,
                    claim_token_digest, record_digest
             FROM {ENTRY_CLAIM_TABLE}
             ORDER BY attempt_identity"
        ))
        .map_err(|e| e.to_string())?;
    let rows = stmt
        .query_map([], map_provider_entry_claim_row)
        .map_err(|e| e.to_string())?;
    for row in rows {
        let claim = row.map_err(|e| e.to_string())?;
        claims.insert(claim.attempt_identity.clone(), claim);
    }

    for record in attempts.values() {
        if let Some(proof_digest) = record.entry_admission_proof_digest.as_deref() {
            if proof_digest.trim().is_empty() {
                return Err(format!(
                    "attempt {} has an empty entry admission proof digest",
                    record.attempt_identity
                ));
            }
            if record.state == AttemptRecordState::NotEntered
                || record.state == AttemptRecordState::Consumed
                || record.state == AttemptRecordState::Reserved
            {
                return Err(format!(
                    "attempt {} retains an entry admission proof before provider entry",
                    record.attempt_identity
                ));
            }
            if record.state == AttemptRecordState::DispatchPending
                && !claims.contains_key(&record.attempt_identity)
            {
                return Err(format!(
                    "DISPATCH_PENDING attempt {} has an entry proof but no active provider entry claim",
                    record.attempt_identity
                ));
            }
        }
    }

    for claim in claims.values() {
        let attempt = attempts.get(&claim.attempt_identity).ok_or_else(|| {
            format!(
                "provider entry claim {} references unknown attempt",
                claim.attempt_identity
            )
        })?;
        if attempt.state != AttemptRecordState::DispatchPending {
            return Err(format!(
                "provider entry claim {} requires DISPATCH_PENDING attempt",
                claim.attempt_identity
            ));
        }
        if attempt.action_key_digest != claim.action_key_digest
            || attempt.ownership_token_digest != claim.owner_token_digest
        {
            return Err(format!(
                "provider entry claim {} does not match attempt owner/action",
                claim.attempt_identity
            ));
        }
        let fence = fences.get(&claim.action_key_digest).ok_or_else(|| {
            format!(
                "provider entry claim {} is missing its action fence",
                claim.attempt_identity
            )
        })?;
        if fence.owner_attempt_identity != claim.attempt_identity
            || fence.owner_token_digest != claim.owner_token_digest
            || fence.state != ActionFenceState::Occupied
        {
            return Err(format!(
                "provider entry claim {} does not match occupied action fence",
                claim.attempt_identity
            ));
        }
    }

    for (attempt_id, claim) in &claims {
        if attempt_id != &claim.attempt_identity {
            return Err("provider entry claim map key mismatch".into());
        }
    }

    for record in attempts.values() {
        let replay = replays.get(&record.native_replay_identity).ok_or_else(|| {
            format!(
                "attempt {} has no native replay binding",
                record.attempt_identity
            )
        })?;
        if replay.operation_id != record.operation_id
            || replay.action_key_digest != record.action_key_digest
        {
            return Err(format!(
                "attempt {} conflicts with native replay binding",
                record.attempt_identity
            ));
        }

        match record.state {
            AttemptRecordState::Executed => {
                let fence = fences.get(&record.action_key_digest).ok_or_else(|| {
                    format!(
                        "executed attempt {} is missing its closed action fence",
                        record.attempt_identity
                    )
                })?;
                if fence.state != ActionFenceState::Closed
                    || fence.owner_attempt_identity != record.attempt_identity
                    || fence.owner_token_digest != record.ownership_token_digest
                {
                    return Err(format!(
                        "executed attempt {} has an invalid closed fence",
                        record.attempt_identity
                    ));
                }
            }
            AttemptRecordState::Failed | AttemptRecordState::NotEntered => {
                if fences.contains_key(&record.action_key_digest) {
                    return Err(format!(
                        "terminal non-executed attempt {} still owns an action fence",
                        record.attempt_identity
                    ));
                }
            }
            _ => {
                let fence = fences.get(&record.action_key_digest).ok_or_else(|| {
                    format!(
                        "live attempt {} is missing its occupied action fence",
                        record.attempt_identity
                    )
                })?;
                if fence.state != ActionFenceState::Occupied
                    || fence.owner_attempt_identity != record.attempt_identity
                    || fence.owner_token_digest != record.ownership_token_digest
                {
                    return Err(format!(
                        "live attempt {} does not own its occupied action fence",
                        record.attempt_identity
                    ));
                }
            }
        }
    }

    for replay in replays.values() {
        if !attempts.values().any(|attempt| {
            attempt.native_replay_identity == replay.native_replay_identity
                && attempt.operation_id == replay.operation_id
                && attempt.action_key_digest == replay.action_key_digest
        }) {
            return Err(format!(
                "native replay binding {} has no matching attempt history",
                replay.native_replay_identity
            ));
        }
    }

    for fence in fences.values() {
        let attempt = attempts.get(&fence.owner_attempt_identity).ok_or_else(|| {
            format!(
                "fence {} references missing attempt {}",
                fence.action_key_digest, fence.owner_attempt_identity
            )
        })?;
        if attempt.action_key_digest != fence.action_key_digest
            || attempt.ownership_token_digest != fence.owner_token_digest
        {
            return Err(format!(
                "fence {} disagrees with owner attempt {}",
                fence.action_key_digest, fence.owner_attempt_identity
            ));
        }
        if fence.state == ActionFenceState::Occupied && !attempt.state.occupies_action_fence() {
            return Err(format!(
                "occupied fence {} belongs to non-live attempt {}",
                fence.action_key_digest, fence.owner_attempt_identity
            ));
        }
        if fence.state == ActionFenceState::Closed
            && attempt.state != AttemptRecordState::Executed
        {
            return Err(format!(
                "closed fence {} does not belong to an executed attempt",
                fence.action_key_digest
            ));
        }
    }

    Ok(())
}

fn validate_foreign_keys(conn: &Connection) -> Result<(), String> {
    let mut stmt = conn
        .prepare(&format!("PRAGMA foreign_key_list({FENCE_TABLE})"))
        .map_err(|e| e.to_string())?;
    let rows = stmt
        .query_map([], |row| {
            Ok((
                row.get::<_, String>(2)?,
                row.get::<_, String>(3)?,
                row.get::<_, String>(4)?,
                row.get::<_, String>(5)?,
            ))
        })
        .map_err(|e| e.to_string())?;
    let actual = rows
        .collect::<Result<Vec<_>, _>>()
        .map_err(|e| e.to_string())?;
    if actual
        != vec![(
            ATTEMPT_TABLE.to_owned(),
            "owner_attempt_identity".to_owned(),
            "attempt_identity".to_owned(),
            "NO ACTION".to_owned(),
        )]
    {
        return Err(format!(
            "unexpected {FENCE_TABLE} foreign keys: {actual:?}"
        ));
    }

    let mut stmt = conn
        .prepare(&format!("PRAGMA foreign_key_list({ENTRY_CLAIM_TABLE})"))
        .map_err(|e| e.to_string())?;
    let rows = stmt
        .query_map([], |row| {
            Ok((
                row.get::<_, String>(2)?,
                row.get::<_, String>(3)?,
                row.get::<_, String>(4)?,
                row.get::<_, String>(5)?,
            ))
        })
        .map_err(|e| e.to_string())?;
    let actual = rows
        .collect::<Result<Vec<_>, _>>()
        .map_err(|e| e.to_string())?;
    let expected = vec![
        (
            ATTEMPT_TABLE.to_owned(),
            "attempt_identity".to_owned(),
            "attempt_identity".to_owned(),
            "NO ACTION".to_owned(),
        ),
        (
            FENCE_TABLE.to_owned(),
            "action_key_digest".to_owned(),
            "action_key_digest".to_owned(),
            "NO ACTION".to_owned(),
        ),
    ];
    if actual != expected {
        return Err(format!(
            "unexpected {ENTRY_CLAIM_TABLE} foreign keys: {actual:?}"
        ));
    }
    Ok(())
}

const SCHEMA_SQL: &str = r#"
CREATE TABLE effect_fence_store_meta (
    singleton INTEGER PRIMARY KEY NOT NULL CHECK(singleton = 1),
    schema_version INTEGER NOT NULL,
    profile TEXT NOT NULL
);

CREATE TABLE effect_attempts (
    attempt_identity TEXT PRIMARY KEY NOT NULL,
    schema_version INTEGER NOT NULL,
    operation_id TEXT NOT NULL,
    native_replay_identity TEXT NOT NULL,
    action_digest TEXT NOT NULL,
    action_key_digest TEXT NOT NULL,
    effecting_target_identity TEXT NOT NULL,
    provider_reference_seed_digest TEXT,
    provider_reference_descriptor_digest TEXT,
    provider_environment TEXT NOT NULL,
    provider_audience TEXT NOT NULL,
    adapter_identity TEXT NOT NULL,
    provider_idempotency_key TEXT NOT NULL,
    authorization_admission_proof_digest TEXT,
    entry_admission_proof_digest TEXT,
    ownership_token_digest TEXT NOT NULL,
    reconciliation_token_digest TEXT,
    terminal_evidence_digest TEXT,
    state INTEGER NOT NULL CHECK(state IN (1,2,3,4,5,6,7,8)),
    not_entered_marker TEXT,
    record_digest TEXT NOT NULL,
    CHECK((state IN (5,6)) OR terminal_evidence_digest IS NULL),
    CHECK((state = 7) OR reconciliation_token_digest IS NULL),
    CHECK((state != 7) OR reconciliation_token_digest IS NOT NULL),
    CHECK((state = 8) OR not_entered_marker IS NULL),
    CHECK((state != 8) OR not_entered_marker IS NOT NULL),
    CHECK(
        state NOT IN (3,4,5,6,7)
        OR (provider_reference_seed_digest IS NOT NULL
            AND provider_reference_descriptor_digest IS NOT NULL)
    )
);

CREATE TABLE effect_action_fences (
    action_key_digest TEXT PRIMARY KEY NOT NULL,
    owner_attempt_identity TEXT NOT NULL UNIQUE,
    owner_token_digest TEXT NOT NULL,
    state INTEGER NOT NULL CHECK(state IN (1,2)),
    record_digest TEXT NOT NULL,
    FOREIGN KEY(owner_attempt_identity)
        REFERENCES effect_attempts(attempt_identity)
);

CREATE TABLE effect_native_replay_bindings (
    native_replay_identity TEXT PRIMARY KEY NOT NULL,
    operation_id TEXT NOT NULL,
    action_key_digest TEXT NOT NULL,
    record_digest TEXT NOT NULL
);

CREATE TABLE effect_provider_entry_claims (
    attempt_identity TEXT PRIMARY KEY NOT NULL,
    action_key_digest TEXT NOT NULL UNIQUE,
    owner_token_digest TEXT NOT NULL,
    claim_token_digest TEXT NOT NULL UNIQUE,
    record_digest TEXT NOT NULL,
    FOREIGN KEY(attempt_identity) REFERENCES effect_attempts(attempt_identity),
    FOREIGN KEY(action_key_digest) REFERENCES effect_action_fences(action_key_digest)
);
"#;

fn insert_attempt_tx(tx: &Transaction<'_>, record: &AttemptRecordV1) -> Result<(), String> {
    tx.execute(
        "INSERT INTO effect_attempts (
            attempt_identity, schema_version, operation_id, native_replay_identity,
            action_digest, action_key_digest, effecting_target_identity,
            provider_reference_seed_digest, provider_reference_descriptor_digest,
            provider_environment, provider_audience, adapter_identity,
            provider_idempotency_key, authorization_admission_proof_digest,
            entry_admission_proof_digest, ownership_token_digest,
            reconciliation_token_digest, terminal_evidence_digest, state, not_entered_marker, record_digest
        ) VALUES (
            ?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12, ?13, ?14,
            ?15, ?16, ?17, ?18, ?19, ?20, ?21
        )",
        params![
            record.attempt_identity,
            record.schema_version,
            record.operation_id,
            record.native_replay_identity,
            record.action_digest,
            record.action_key_digest,
            record.effecting_target_identity,
            record.provider_reference_seed_digest,
            record.provider_reference_descriptor_digest,
            record.provider_environment,
            record.provider_audience,
            record.adapter_identity,
            record.provider_idempotency_key,
            record.authorization_admission_proof_digest,
            record.entry_admission_proof_digest,
            record.ownership_token_digest,
            record.reconciliation_token_digest,
            record.terminal_evidence_digest,
            record.state.storage_tag(),
            record.not_entered_marker,
            record.record_digest()
        ],
    )
    .map_err(|e| e.to_string())?;
    Ok(())
}

fn update_attempt_tx(
    tx: &Transaction<'_>,
    current: &AttemptRecordV1,
    updated: &AttemptRecordV1,
) -> Result<(), ActionFenceMutationError> {
    updated.validate().map_err(storage_error)?;
    let changed = tx
        .execute(
            "UPDATE effect_attempts
             SET state = ?1,
                 authorization_admission_proof_digest = ?2,
                 entry_admission_proof_digest = ?3,
                 reconciliation_token_digest = ?4,
                 terminal_evidence_digest = ?5,
                 not_entered_marker = ?6,
                 record_digest = ?7
             WHERE attempt_identity = ?8
               AND state = ?9
               AND action_key_digest = ?10
               AND ownership_token_digest = ?11
               AND record_digest = ?12",
            params![
                updated.state.storage_tag(),
                updated.authorization_admission_proof_digest,
                updated.entry_admission_proof_digest,
                updated.reconciliation_token_digest,
                updated.terminal_evidence_digest,
                updated.not_entered_marker,
                updated.record_digest(),
                updated.attempt_identity,
                current.state.storage_tag(),
                current.action_key_digest,
                current.ownership_token_digest,
                current.record_digest()
            ],
        )
        .map_err(storage_error)?;
    if changed != 1 {
        return Err(ActionFenceMutationError::NotOwner);
    }
    Ok(())
}

fn load_attempt_tx(
    tx: &Transaction<'_>,
    attempt_identity: &str,
) -> Result<Option<AttemptRecordV1>, String> {
    tx.query_row(
        &format!(
            "SELECT attempt_identity, schema_version, operation_id, native_replay_identity,
                    action_digest, action_key_digest, effecting_target_identity,
                    provider_reference_seed_digest, provider_reference_descriptor_digest,
                    provider_environment, provider_audience, adapter_identity,
                    provider_idempotency_key, authorization_admission_proof_digest,
                    entry_admission_proof_digest, ownership_token_digest,
                    reconciliation_token_digest,
                    terminal_evidence_digest, state,
                    not_entered_marker, record_digest
             FROM {ATTEMPT_TABLE}
             WHERE attempt_identity = ?1"
        ),
        params![attempt_identity],
        map_attempt_row,
    )
    .optional()
    .map_err(|e| e.to_string())?
    .transpose()
}

fn load_attempt_txless(
    conn: &Connection,
    attempt_identity: &str,
) -> Result<Option<AttemptRecordV1>, String> {
    conn.query_row(
        &format!(
            "SELECT attempt_identity, schema_version, operation_id, native_replay_identity,
                    action_digest, action_key_digest, effecting_target_identity,
                    provider_reference_seed_digest, provider_reference_descriptor_digest,
                    provider_environment, provider_audience, adapter_identity,
                    provider_idempotency_key, authorization_admission_proof_digest,
                    entry_admission_proof_digest, ownership_token_digest,
                    reconciliation_token_digest,
                    terminal_evidence_digest, state,
                    not_entered_marker, record_digest
             FROM {ATTEMPT_TABLE}
             WHERE attempt_identity = ?1"
        ),
        params![attempt_identity],
        map_attempt_row,
    )
    .optional()
    .map_err(|e| e.to_string())?
    .transpose()
}

fn map_attempt_row(row: &rusqlite::Row<'_>) -> Result<AttemptRecordV1, rusqlite::Error> {
    let state_tag: i64 = row.get(18)?;
    let state = AttemptRecordState::from_storage_tag(state_tag).ok_or_else(|| {
        rusqlite::Error::InvalidParameterName("invalid attempt state tag".to_owned())
    })?;

    let record = AttemptRecordV1 {
        attempt_identity: row.get(0)?,
        schema_version: row.get(1)?,
        operation_id: row.get(2)?,
        native_replay_identity: row.get(3)?,
        action_digest: row.get(4)?,
        action_key_digest: row.get(5)?,
        effecting_target_identity: row.get(6)?,
        provider_reference_seed_digest: row.get(7)?,
        provider_reference_descriptor_digest: row.get(8)?,
        provider_environment: row.get(9)?,
        provider_audience: row.get(10)?,
        adapter_identity: row.get(11)?,
        provider_idempotency_key: row.get(12)?,
        authorization_admission_proof_digest: row.get(13)?,
        entry_admission_proof_digest: row.get(14)?,
        ownership_token_digest: row.get(15)?,
        reconciliation_token_digest: row.get(16)?,
        terminal_evidence_digest: row.get(17)?,
        state,
        not_entered_marker: row.get(19)?,
    };

    let stored_digest: String = row.get(20)?;
    record.validate().map_err(|e| {
        rusqlite::Error::InvalidParameterName(format!("invalid persisted attempt: {e}"))
    })?;
    if record.record_digest() != stored_digest {
        return Err(rusqlite::Error::InvalidParameterName(
            "persisted attempt record digest mismatch".to_owned(),
        ));
    }
    Ok(record)
}

fn load_fence_tx(
    tx: &Transaction<'_>,
    action_key_digest: &str,
) -> Result<Option<ActionFenceRecordV1>, rusqlite::Error> {
    tx.query_row(
        &format!(
            "SELECT action_key_digest, owner_attempt_identity, owner_token_digest, state, record_digest
             FROM {FENCE_TABLE}
             WHERE action_key_digest = ?1"
        ),
        params![action_key_digest],
        map_fence_row,
    )
    .optional()
}

fn load_fence_txless(
    conn: &Connection,
    action_key_digest: &str,
) -> Result<Option<ActionFenceRecordV1>, String> {
    conn.query_row(
        &format!(
            "SELECT action_key_digest, owner_attempt_identity, owner_token_digest, state
             FROM {FENCE_TABLE}
             WHERE action_key_digest = ?1"
        ),
        params![action_key_digest],
        map_fence_row,
    )
    .optional()
    .map_err(|e| e.to_string())?
    .transpose()
}

fn map_fence_row(row: &rusqlite::Row<'_>) -> Result<ActionFenceRecordV1, rusqlite::Error> {
    let state = ActionFenceState::from_storage_tag(row.get::<_, i64>(3)?).ok_or_else(|| {
        rusqlite::Error::InvalidParameterName("invalid fence state tag".to_owned())
    })?;
    let record = ActionFenceRecordV1 {
        schema_version: ACTION_FENCE_SCHEMA_VERSION,
        action_key_digest: row.get(0)?,
        owner_attempt_identity: row.get(1)?,
        owner_token_digest: row.get(2)?,
        state,
    };
    let stored_digest: String = row.get(4)?;
    record.validate().map_err(|e| {
        rusqlite::Error::InvalidParameterName(format!("invalid persisted fence: {e}"))
    })?;
    if record.record_digest() != stored_digest {
        return Err(rusqlite::Error::InvalidParameterName(
            "persisted fence record digest mismatch".to_owned(),
        ));
    }
    Ok(record)
}

fn load_provider_entry_claim_tx(
    tx: &Transaction<'_>,
    attempt_identity: &str,
) -> Result<Option<ProviderEntryClaimV1>, String> {
    tx.query_row(
        &format!(
            "SELECT attempt_identity, action_key_digest, owner_token_digest,
                    claim_token_digest, record_digest
             FROM {ENTRY_CLAIM_TABLE}
             WHERE attempt_identity = ?1"
        ),
        params![attempt_identity],
        map_provider_entry_claim_row,
    )
    .optional()
    .map_err(|e| e.to_string())
}

fn load_provider_entry_claim_txless(
    conn: &Connection,
    attempt_identity: &str,
) -> Result<Option<ProviderEntryClaimV1>, String> {
    conn.query_row(
        &format!(
            "SELECT attempt_identity, action_key_digest, owner_token_digest,
                    claim_token_digest, record_digest
             FROM {ENTRY_CLAIM_TABLE}
             WHERE attempt_identity = ?1"
        ),
        params![attempt_identity],
        map_provider_entry_claim_row,
    )
    .optional()
    .map_err(|e| e.to_string())
}

fn map_provider_entry_claim_row(
    row: &rusqlite::Row<'_>,
) -> rusqlite::Result<ProviderEntryClaimV1> {
    let attempt_identity: String = row.get(0)?;
    let action_key_digest: String = row.get(1)?;
    let owner_token_digest: String = row.get(2)?;
    let claim_token_digest: String = row.get(3)?;
    let record_digest: String = row.get(4)?;
    let claim = ProviderEntryClaimV1::new(
        attempt_identity,
        action_key_digest,
        owner_token_digest,
        claim_token_digest,
    )
    .map_err(|error| rusqlite::Error::FromSqlConversionFailure(
        4,
        rusqlite::types::Type::Text,
        Box::new(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            error,
        )),
    ))?;
    if claim.record_digest() != record_digest {
        return Err(rusqlite::Error::FromSqlConversionFailure(
            4,
            rusqlite::types::Type::Text,
            Box::new(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "provider entry claim record digest mismatch",
            )),
        ));
    }
    Ok(claim)
}

fn load_replay_tx(
    tx: &Transaction<'_>,
    native_replay_identity: &str,
) -> Result<Option<NativeReplayBindingV1>, String> {
    tx.query_row(
        &format!(
            "SELECT native_replay_identity, operation_id, action_key_digest, record_digest
             FROM {REPLAY_TABLE}
             WHERE native_replay_identity = ?1"
        ),
        params![native_replay_identity],
        map_replay_row,
    )
    .optional()
    .map_err(|e| e.to_string())?
    .transpose()
}

fn load_replay_txless(
    conn: &Connection,
    native_replay_identity: &str,
) -> Result<Option<NativeReplayBindingV1>, String> {
    conn.query_row(
        &format!(
            "SELECT native_replay_identity, operation_id, action_key_digest
             FROM {REPLAY_TABLE}
             WHERE native_replay_identity = ?1"
        ),
        params![native_replay_identity],
        map_replay_row,
    )
    .optional()
    .map_err(|e| e.to_string())?
    .transpose()
}

fn map_replay_row(
    row: &rusqlite::Row<'_>,
) -> Result<NativeReplayBindingV1, rusqlite::Error> {
    let record = NativeReplayBindingV1 {
        schema_version: NATIVE_REPLAY_BINDING_SCHEMA_VERSION,
        native_replay_identity: row.get(0)?,
        operation_id: row.get(1)?,
        action_key_digest: row.get(2)?,
    };
    let stored_digest: String = row.get(3)?;
    record.validate().map_err(|e| {
        rusqlite::Error::InvalidParameterName(format!("invalid persisted replay binding: {e}"))
    })?;
    if record.digest() != stored_digest {
        return Err(rusqlite::Error::InvalidParameterName(
            "persisted replay binding digest mismatch".to_owned(),
        ));
    }
    Ok(record)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key(action: &str) -> ActionKeyV1 {
        ActionKeyV1::new("rp-test", "target-test", action).unwrap()
    }

    fn attempt(id: &str) -> AttemptIdentityV1 {
        AttemptIdentityV1::new("governance-test", "boundary-test", id).unwrap()
    }

    fn record(
        id: &str,
        operation: &str,
        native_replay: &str,
        key: &ActionKeyV1,
        state: AttemptRecordState,
    ) -> AttemptRecordV1 {
        AttemptRecordV1::new(
            &attempt(id),
            operation,
            native_replay,
            key.material_action_digest(),
            key,
            Some("provider-seed-test".into()),
            Some("provider-descriptor-test".into()),
            "provider-test",
            "audience-test",
            "adapter-test",
            format!("owner-token-{id}"),
            state,
        )
        .unwrap()
    }

    fn authorized_admit(
        store: &mut SqliteActionFenceStore,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        record: AttemptRecordV1,
    ) -> Result<AtomicAdmissionDecision, String> {
        let proof = AuthorizationAdmissionProofV1::new(
            &record,
            action_key,
            0,
            u64::MAX,
            "test-admission-verifier-v1",
            "test-policy-snapshot-v1",
            "test-status-snapshot-v1",
            "test-admission-verifier-v1",
        )?;
        authorized_admit(&mut store, action_key, attempt_identity, record, proof)
    }

    fn evidence(
        action_key: &ActionKeyV1,
        attempt: &AttemptIdentityV1,
        operation: &str,
        native_replay: &str,
        outcome: TerminalOutcomeV1,
    ) -> TerminalEvidenceV1 {
        TerminalEvidenceV1::from_attempt(
            action_key,
            &record(
                attempt.attempt_id(),
                operation,
                native_replay,
                action_key,
                AttemptRecordState::Invoked,
            ),
            outcome,
            "provider-evidence-commitment",
            "verifier-test",
        )
        .unwrap()
    }


    #[test]
    fn admission_survives_close_and_reopen() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("fence.db");
        let mut store = SqliteActionFenceStore::open(&path).unwrap();
        let action = key("action-1");
        let owner = attempt("attempt-1");
        let row = record(
            "attempt-1",
            "operation-1",
            "native-replay-1",
            &action,
            AttemptRecordState::Consumed,
        );
        assert_eq!(
            authorized_admit(&mut store, &action, &owner, row).unwrap(),
            AtomicAdmissionDecision::Admitted
        );
        drop(store);

        let reopened = SqliteActionFenceStore::open(&path).unwrap();
        let restored = reopened.durably_read_attempt(&owner).unwrap().unwrap();
        assert_eq!(restored.state, AttemptRecordState::Consumed);
        assert_eq!(
            reopened.durably_read_fence(&action).unwrap().unwrap().state,
            ActionFenceState::Occupied
        );
        assert_eq!(
            reopened
                .durably_read_replay_binding("native-replay-1")
                .unwrap()
                .unwrap()
                .operation_id,
            "operation-1"
        );
    }

    #[test]
    fn concurrent_first_open_initializes_one_valid_store() {
        use std::sync::{Arc, Barrier};
        use std::thread;

        let dir = tempfile::tempdir().unwrap();
        let path = Arc::new(dir.path().join("first-open.db"));
        let barrier = Arc::new(Barrier::new(3));

        let mut handles = Vec::new();
        for _ in 0..2 {
            let path = Arc::clone(&path);
            let barrier = Arc::clone(&barrier);
            handles.push(thread::spawn(move || {
                barrier.wait();
                SqliteActionFenceStore::open(&*path).map(|store| store.audit_integrity())
            }));
        }

        barrier.wait();
        let results: Vec<_> = handles.into_iter().map(|h| h.join().unwrap()).collect();
        assert!(results.iter().all(|result| matches!(result, Ok(Ok(())))));
    }

    #[test]
    fn same_action_conflicts_across_independent_connections() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("fence.db");
        let mut first = SqliteActionFenceStore::open(&path).unwrap();
        let mut second = SqliteActionFenceStore::open(&path).unwrap();
        let action = key("action-1");
        let a = attempt("attempt-1");
        let b = attempt("attempt-2");

        authorized_admit(&mut first, 
                &action,
                &a,
                record(
                    "attempt-1",
                    "operation-1",
                    "native-replay-1",
                    &action,
                    AttemptRecordState::Consumed,
                ),
            )
            .unwrap();

        assert_eq!(
            authorized_admit(&mut second, 
                    &action,
                    &b,
                    record(
                        "attempt-2",
                        "operation-2",
                        "native-replay-2",
                        &action,
                        AttemptRecordState::Consumed,
                    ),
                )
                .unwrap(),
            AtomicAdmissionDecision::ActionInFlight
        );
        assert!(second.durably_read_attempt(&b).unwrap().is_none());
    }

    #[test]
    fn simultaneous_admission_has_one_winner() {
        use std::sync::{Arc, Barrier};
        use std::thread;

        let dir = tempfile::tempdir().unwrap();
        let path = Arc::new(dir.path().join("race.db"));
        SqliteActionFenceStore::open(&*path).unwrap();

        let barrier = Arc::new(Barrier::new(3));
        let mut handles = Vec::new();
        for id in ["attempt-a", "attempt-b"] {
            let path = Arc::clone(&path);
            let barrier = Arc::clone(&barrier);
            handles.push(thread::spawn(move || {
                let mut store = SqliteActionFenceStore::open(&*path).unwrap();
                let action = key("race-action");
                let owner = attempt(id);
                barrier.wait();
                authorized_admit(&mut store, 
                        &action,
                        &owner,
                        record(
                            id,
                            id,
                            &format!("native-{id}"),
                            &action,
                            AttemptRecordState::Consumed,
                        ),
                    )
                    .unwrap()
            }));
        }

        barrier.wait();
        let results: Vec<_> = handles.into_iter().map(|h| h.join().unwrap()).collect();
        assert_eq!(
            results
                .iter()
                .filter(|r| **r == AtomicAdmissionDecision::Admitted)
                .count(),
            1
        );
        assert_eq!(
            results
                .iter()
                .filter(|r| **r == AtomicAdmissionDecision::ActionInFlight)
                .count(),
            1
        );
    }

    #[test]
    fn conflicting_replay_does_not_partially_install_new_attempt() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("fence.db");
        let mut store = SqliteActionFenceStore::open(&path).unwrap();
        let first_key = key("action-1");
        let second_key = key("action-2");
        let first = attempt("attempt-1");
        let second = attempt("attempt-2");

        authorized_admit(&mut store, 
                &first_key,
                &first,
                record(
                    "attempt-1",
                    "operation-1",
                    "shared-native",
                    &first_key,
                    AttemptRecordState::Consumed,
                ),
            )
            .unwrap();

        assert!(matches!(
            authorized_admit(&mut store, 
                    &second_key,
                    &second,
                    record(
                        "attempt-2",
                        "operation-2",
                        "shared-native",
                        &second_key,
                        AttemptRecordState::Consumed,
                    ),
                )
                .unwrap(),
            AtomicAdmissionDecision::NativeReplayConflict { .. }
        ));
        assert!(store.durably_read_attempt(&second).unwrap().is_none());
        assert!(store.durably_read_fence(&second_key).unwrap().is_none());
    }

    #[test]
    fn injected_late_insert_failure_rolls_back_all_admission_roots() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("rollback.db");
        let mut store = SqliteActionFenceStore::open(&path).unwrap();
        {
            let conn = Connection::open(&path).unwrap();
            conn.execute_batch(
                "CREATE TRIGGER fail_fence_insert
                 BEFORE INSERT ON effect_action_fences
                 BEGIN
                    SELECT RAISE(ABORT, 'injected fence failure');
                 END;",
            )
            .unwrap();
        }

        let action = key("rollback-action");
        let owner = attempt("rollback-attempt");
        assert!(authorized_admit(&mut store, 
                &action,
                &owner,
                record(
                    "rollback-attempt",
                    "rollback-operation",
                    "rollback-native",
                    &action,
                    AttemptRecordState::Consumed,
                ),
            )
            .is_err());

        let conn = Connection::open(&path).unwrap();
        let counts: (i64, i64, i64) = conn
            .query_row(
                "SELECT
                    (SELECT COUNT(*) FROM effect_native_replay_bindings),
                    (SELECT COUNT(*) FROM effect_attempts),
                    (SELECT COUNT(*) FROM effect_action_fences)",
                [],
                |row| Ok((row.get(0)?, row.get(1)?, row.get(2)?)),
            )
            .unwrap();
        assert_eq!(counts, (0, 0, 0));
    }

    #[test]
    fn executed_close_survives_restart_and_blocks_new_attempt() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("terminal.db");
        let mut store = SqliteActionFenceStore::open(&path).unwrap();
        let action = key("terminal-action");
        let owner = attempt("attempt-1");

        authorized_admit(&mut store, 
                &action,
                &owner,
                record(
                    "attempt-1",
                    "operation-1",
                    "native-1",
                    &action,
                    AttemptRecordState::Consumed,
                ),
            )
            .unwrap();
        store
            .atomically_mark_dispatch_pending(&action, &owner, "owner-token-attempt-1")
            .unwrap();
        let claim = store
            .atomically_claim_provider_entry(
                &action,
                &owner,
                "owner-token-attempt-1",
                "constitutional-provider-entry-claim-token-v1:test",
            )
            .unwrap();
        store
            .atomically_mark_invoked(
                &action,
                &owner,
                "owner-token-attempt-1",
                &claim.claim_token_digest,
            )
            .unwrap();
        let reconciliation_token = store
            .atomically_mark_indeterminate(&action, &owner, "owner-token-attempt-1")
            .unwrap();
        store
            .atomically_close_executed(
                &action,
                &owner,
                &reconciliation_token,
                &evidence(&action, &owner, "operation-1", "native-1", TerminalOutcomeV1::Executed),
            )
            .unwrap();

        drop(store);
        let mut reopened = SqliteActionFenceStore::open(&path).unwrap();
        let closed_fence = reopened.durably_read_fence(&action).unwrap().unwrap();
        assert_eq!(closed_fence.state, ActionFenceState::Closed);
        let expected_closed = ActionFenceRecordV1 {
            schema_version: ACTION_FENCE_SCHEMA_VERSION,
            action_key_digest: action.digest().to_owned(),
            owner_attempt_identity: owner.digest().to_owned(),
            owner_token_digest: "owner-token-attempt-1".to_owned(),
            state: ActionFenceState::Closed,
        };
        assert_eq!(closed_fence.record_digest(), expected_closed.record_digest());
        assert_eq!(
            authorized_admit(&mut reopened, 
                    &action,
                    &attempt("attempt-2"),
                    record(
                        "attempt-2",
                        "operation-2",
                        "native-2",
                        &action,
                        AttemptRecordState::Consumed,
                    ),
                )
                .unwrap(),
            AtomicAdmissionDecision::ActionAlreadyExecuted
        );
    }

    #[test]
    fn failed_release_deletes_only_action_fence_and_preserves_replay_binding() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("failed.db");
        let mut store = SqliteActionFenceStore::open(&path).unwrap();
        let action = key("failed-action");
        let owner = attempt("attempt-1");

        authorized_admit(&mut store, 
                &action,
                &owner,
                record(
                    "attempt-1",
                    "operation-1",
                    "native-shared",
                    &action,
                    AttemptRecordState::Consumed,
                ),
            )
            .unwrap();
        store
            .atomically_mark_dispatch_pending(&action, &owner, "owner-token-attempt-1")
            .unwrap();
        let claim = store
            .atomically_claim_provider_entry(
                &action,
                &owner,
                "owner-token-attempt-1",
                "constitutional-provider-entry-claim-token-v1:test",
            )
            .unwrap();
        store
            .atomically_mark_invoked(
                &action,
                &owner,
                "owner-token-attempt-1",
                &claim.claim_token_digest,
            )
            .unwrap();
        store
            .atomically_release_after_failed(
                &action,
                &owner,
                "owner-token-attempt-1",
                &evidence(&action, &owner, "operation-1", "native-shared", TerminalOutcomeV1::Failed),
            )
            .unwrap();

        assert!(store.durably_read_fence(&action).unwrap().is_none());
        assert_eq!(
            store.durably_read_attempt(&owner).unwrap().unwrap().state,
            AttemptRecordState::Failed
        );
        assert_eq!(
            store
                .durably_read_replay_binding("native-shared")
                .unwrap()
                .unwrap()
                .action_key_digest,
            action.digest()
        );

        assert_eq!(
            authorized_admit(&mut store, 
                    &action,
                    &attempt("attempt-2"),
                    record(
                        "attempt-2",
                        "operation-2",
                        "native-shared",
                        &action,
                        AttemptRecordState::Consumed,
                    ),
                )
                .unwrap(),
            AtomicAdmissionDecision::Admitted
        );
    }

    #[test]
    fn wrong_owner_and_wrong_outcome_do_not_mutate() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("owner.db");
        let mut store = SqliteActionFenceStore::open(&path).unwrap();
        let action = key("owner-action");
        let owner = attempt("attempt-1");

        authorized_admit(&mut store, 
                &action,
                &owner,
                record(
                    "attempt-1",
                    "operation-1",
                    "native-1",
                    &action,
                    AttemptRecordState::Consumed,
                ),
            )
            .unwrap();
        store
            .atomically_mark_dispatch_pending(&action, &owner, "owner-token-attempt-1")
            .unwrap();
        let claim = store
            .atomically_claim_provider_entry(
                &action,
                &owner,
                "owner-token-attempt-1",
                "constitutional-provider-entry-claim-token-v1:test",
            )
            .unwrap();
        store
            .atomically_mark_invoked(
                &action,
                &owner,
                "owner-token-attempt-1",
                &claim.claim_token_digest,
            )
            .unwrap();

        assert_eq!(
            store
                .atomically_close_executed(
                    &action,
                    &owner,
                    "wrong-owner",
                    &evidence(&action, &owner, "operation-1", "native-1", TerminalOutcomeV1::Executed),
                )
                .unwrap_err(),
            ActionFenceMutationError::OwnershipTokenMismatch
        );
        assert_eq!(
            store
                .atomically_close_executed(
                    &action,
                    &owner,
                    "owner-token-attempt-1",
                    &evidence(&action, &owner, TerminalOutcomeV1::Failed),
                )
                .unwrap_err(),
            ActionFenceMutationError::TerminalEvidenceMismatch
        );
        assert_eq!(
            store.durably_read_attempt(&owner).unwrap().unwrap().state,
            AttemptRecordState::Invoked
        );
        assert_eq!(
            store.durably_read_fence(&action).unwrap().state,
            ActionFenceState::Occupied
        );
    }

    #[test]
    fn not_entered_release_is_durable_and_requires_pre_dispatch_state() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("not-entered.db");
        let mut store = SqliteActionFenceStore::open(&path).unwrap();
        let action = key("not-entered-action");
        let owner = attempt("attempt-1");

        authorized_admit(&mut store, 
                &action,
                &owner,
                record(
                    "attempt-1",
                    "operation-1",
                    "native-1",
                    &action,
                    AttemptRecordState::Reserved,
                ),
            )
            .unwrap();
        store
            .atomically_release_not_entered(
                &action,
                &owner,
                "owner-token-attempt-1",
                "not-entered-proof",
            )
            .unwrap();

        drop(store);
        let reopened = SqliteActionFenceStore::open(&path).unwrap();
        assert!(reopened.durably_read_fence(&action).unwrap().is_none());
        let restored = reopened.durably_read_attempt(&owner).unwrap().unwrap();
        assert_eq!(restored.state, AttemptRecordState::NotEntered);
        assert_eq!(
            restored.not_entered_marker.as_deref(),
            Some("not-entered-proof")
        );
    }

    #[test]
    fn corrupted_fence_digest_fails_closed_on_restart() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("fence-digest-tamper.db");
        let mut store = SqliteActionFenceStore::open(&path).unwrap();
        let action = key("fence-digest-action");
        let owner = attempt("attempt-1");
        authorized_admit(&mut store, 
                &action,
                &owner,
                record(
                    "attempt-1",
                    "operation-1",
                    "native-1",
                    &action,
                    AttemptRecordState::Consumed,
                ),
            )
            .unwrap();
        drop(store);

        let conn = Connection::open(&path).unwrap();
        conn.execute(
            "UPDATE effect_action_fences
             SET record_digest = 'corrupted'
             WHERE action_key_digest = ?1",
            params![action.digest()],
        )
        .unwrap();
        drop(conn);

        assert!(SqliteActionFenceStore::open(&path).is_err());
    }

    #[test]
    fn corrupted_replay_digest_fails_closed_on_restart() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("replay-digest-tamper.db");
        let mut store = SqliteActionFenceStore::open(&path).unwrap();
        let action = key("replay-digest-action");
        let owner = attempt("attempt-1");
        authorized_admit(&mut store, 
                &action,
                &owner,
                record(
                    "attempt-1",
                    "operation-1",
                    "native-1",
                    &action,
                    AttemptRecordState::Consumed,
                ),
            )
            .unwrap();
        drop(store);

        let conn = Connection::open(&path).unwrap();
        conn.execute(
            "UPDATE effect_native_replay_bindings
             SET record_digest = 'corrupted'
             WHERE native_replay_identity = ?1",
            params!["native-1"],
        )
        .unwrap();
        drop(conn);

        assert!(SqliteActionFenceStore::open(&path).is_err());
    }

    #[test]
    fn provider_entry_claim_survives_restart_and_tamper_is_fail_closed() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("claim-restart.db");
        let action = key("claim-restart-action");
        let owner = attempt("attempt-claim-restart");

        {
            let mut store = SqliteActionFenceStore::open(&path).unwrap();
            authorized_admit(&mut store, 
                    &action,
                    &owner,
                    record(
                        "attempt-claim-restart",
                        "operation-claim-restart",
                        "native-claim-restart",
                        &action,
                        AttemptRecordState::Consumed,
                    ),
                )
                .unwrap();
            store
                .atomically_mark_dispatch_pending(
                    &action,
                    &owner,
                    "owner-token-claim-restart",
                )
                .unwrap();
            store
                .atomically_claim_provider_entry(
                    &action,
                    &owner,
                    "owner-token-claim-restart",
                    "constitutional-provider-entry-claim-token-v1:restart",
                )
                .unwrap();
        }

        {
            let reopened = SqliteActionFenceStore::open(&path).unwrap();
            assert!(reopened
                .durably_read_provider_entry_claim(&owner)
                .unwrap()
                .is_some());
        }

        let conn = Connection::open(&path).unwrap();
        conn.execute(
            "UPDATE effect_provider_entry_claims
             SET record_digest = 'corrupted'
             WHERE attempt_identity = ?1",
            params![owner.digest()],
        )
        .unwrap();
        drop(conn);

        assert!(SqliteActionFenceStore::open(&path).is_err());
    }

    #[test]
    fn entry_admission_proof_survives_restart() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("entry-proof-restart.db");
        let action = key("entry-proof-restart");
        let owner = attempt("attempt-entry-proof-restart");

        {
            let mut store = SqliteActionFenceStore::open(&path).unwrap();
            authorized_admit(&mut store, 
                    &action,
                    &owner,
                    record(
                        "attempt-entry-proof-restart",
                        "operation-entry-proof-restart",
                        "native-entry-proof-restart",
                        &action,
                        AttemptRecordState::Consumed,
                    ),
                )
                .unwrap();
            store
                .atomically_mark_dispatch_pending(
                    &action,
                    &owner,
                    "owner-token-entry-proof-restart",
                )
                .unwrap();
            store
                .atomically_claim_provider_entry(
                    &action,
                    &owner,
                    "owner-token-entry-proof-restart",
                    "constitutional-provider-entry-claim-token-v1:entry-proof-restart",
                )
                .unwrap();
            store
                .atomically_record_provider_entry_proof(
                    &action,
                    &owner,
                    "owner-token-entry-proof-restart",
                    "constitutional-final-provider-entry-proof-v1:test-proof",
                )
                .unwrap();
        }

        let reopened = SqliteActionFenceStore::open(&path).unwrap();
        let attempt = reopened.durably_read_attempt(&owner).unwrap().unwrap();
        assert_eq!(
            attempt.entry_admission_proof_digest(),
            Some("constitutional-final-provider-entry-proof-v1:test-proof")
        );
        assert!(reopened
            .durably_read_provider_entry_claim(&owner)
            .unwrap()
            .is_some());
    }

    #[test]
    fn persisted_entry_proof_without_claim_fails_closed_on_restart() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("entry-proof-orphan.db");
        let action = key("entry-proof-orphan");
        let owner = attempt("attempt-entry-proof-orphan");

        {
            let mut store = SqliteActionFenceStore::open(&path).unwrap();
            authorized_admit(&mut store, 
                    &action,
                    &owner,
                    record(
                        "attempt-entry-proof-orphan",
                        "operation-entry-proof-orphan",
                        "native-entry-proof-orphan",
                        &action,
                        AttemptRecordState::Consumed,
                    ),
                )
                .unwrap();
            store
                .atomically_mark_dispatch_pending(
                    &action,
                    &owner,
                    "owner-token-entry-proof-orphan",
                )
                .unwrap();
            store
                .atomically_claim_provider_entry(
                    &action,
                    &owner,
                    "owner-token-entry-proof-orphan",
                    "constitutional-provider-entry-claim-token-v1:entry-proof-orphan",
                )
                .unwrap();
            store
                .atomically_record_provider_entry_proof(
                    &action,
                    &owner,
                    "owner-token-entry-proof-orphan",
                    "constitutional-final-provider-entry-proof-v1:orphan",
                )
                .unwrap();
        }

        let conn = Connection::open(&path).unwrap();
        conn.execute(
            "DELETE FROM effect_provider_entry_claims WHERE attempt_identity = ?1",
            params![owner.digest()],
        )
        .unwrap();
        drop(conn);

        assert!(SqliteActionFenceStore::open(&path).is_err());
    }

    #[test]
    fn second_boundary_cannot_claim_an_already_claimed_provider_entry() {    #[test]
    fn second_boundary_cannot_claim_an_already_claimed_provider_entry() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("claim-collision.db");
        let mut first = SqliteActionFenceStore::open(&path).unwrap();
        let mut second = SqliteActionFenceStore::open(&path).unwrap();
        let action = key("claim-collision-action");
        let owner = attempt("attempt-claim-collision");

        authorized_admit(&mut first, 
                &action,
                &owner,
                record(
                    "attempt-claim-collision",
                    "operation-claim-collision",
                    "native-claim-collision",
                    &action,
                    AttemptRecordState::Consumed,
                ),
            )
            .unwrap();
        first
            .atomically_mark_dispatch_pending(
                &action,
                &owner,
                "owner-token-claim-collision",
            )
            .unwrap();

        first
            .atomically_claim_provider_entry(
                &action,
                &owner,
                "owner-token-claim-collision",
                "constitutional-provider-entry-claim-token-v1:first",
            )
            .unwrap();

        assert_eq!(
            second
                .atomically_claim_provider_entry(
                    &action,
                    &owner,
                    "owner-token-claim-collision",
                    "constitutional-provider-entry-claim-token-v1:second",
                )
                .unwrap_err(),
            ActionFenceMutationError::ProviderEntryClaimed
        );
    }

    #[test]
    fn durable_adapter_matches_reference_model_for_terminal_trace() {
        use constitutional_effect_ledger::AtomicActionFenceModelV1;

        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("model-differential.db");
        let mut store = SqliteActionFenceStore::open(&path).unwrap();
        let mut model = AtomicActionFenceModelV1::new();
        let action = key("differential-action");
        let owner = attempt("attempt-1");
        let row = record(
            "attempt-1",
            "operation-1",
            "native-1",
            &action,
            AttemptRecordState::Consumed,
        );

        assert_eq!(
            model.admit(&action, &owner, row.clone()).unwrap(),
            authorized_admit(&mut store, &action, &owner, row).unwrap()
        );
        assert_eq!(
            model.attempt(owner.digest()).unwrap().record_digest(),
            store.durably_read_attempt(&owner).unwrap().unwrap().record_digest()
        );
        assert_eq!(
            model.fence(action.digest()).unwrap().record_digest(),
            store.durably_read_fence(&action).unwrap().unwrap().record_digest()
        );

        model
            .mark_dispatch_pending(&action, &owner, "owner-token-attempt-1")
            .unwrap();
        store
            .atomically_mark_dispatch_pending(&action, &owner, "owner-token-attempt-1")
            .unwrap();

        let model_claim = model
            .claim_provider_entry(
                &action,
                &owner,
                "owner-token-attempt-1",
                "constitutional-provider-entry-claim-token-v1:differential",
            )
            .unwrap();
        let store_claim = store
            .atomically_claim_provider_entry(
                &action,
                &owner,
                "owner-token-attempt-1",
                "constitutional-provider-entry-claim-token-v1:differential",
            )
            .unwrap();
        assert_eq!(model_claim.record_digest(), store_claim.record_digest());

        model
            .mark_invoked_with_claim(
                &action,
                &owner,
                "owner-token-attempt-1",
                &model_claim.claim_token_digest,
            )
            .unwrap();
        store
            .atomically_mark_invoked(
                &action,
                &owner,
                "owner-token-attempt-1",
                &store_claim.claim_token_digest,
            )
            .unwrap();

        let model_reconciliation_token = model
            .mark_indeterminate(&action, &owner, "owner-token-attempt-1")
            .unwrap();
        let store_reconciliation_token = store
            .atomically_mark_indeterminate(&action, &owner, "owner-token-attempt-1")
            .unwrap();
        assert_eq!(model_reconciliation_token, store_reconciliation_token);

        let proof = evidence(
            &action,
            &owner,
            "operation-1",
            "native-1",
            TerminalOutcomeV1::Executed,
        );
        model
            .close_executed(
                &action,
                &owner,
                &model_reconciliation_token,
                &proof,
            )
            .unwrap();
        store
            .atomically_close_executed(
                &action,
                &owner,
                &store_reconciliation_token,
                &proof,
            )
            .unwrap();

        assert_eq!(
            model.attempt(owner.digest()).unwrap().record_digest(),
            store.durably_read_attempt(&owner).unwrap().unwrap().record_digest()
        );
        assert_eq!(
            model.fence(action.digest()).unwrap().record_digest(),
            store.durably_read_fence(&action).unwrap().unwrap().record_digest()
        );
    }

    #[test]
    fn corrupted_persisted_digest_fails_closed_on_restart() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("tamper.db");
        let mut store = SqliteActionFenceStore::open(&path).unwrap();
        let action = key("tamper-action");
        let owner = attempt("attempt-1");
        authorized_admit(&mut store, 
                &action,
                &owner,
                record(
                    "attempt-1",
                    "operation-1",
                    "native-1",
                    &action,
                    AttemptRecordState::Consumed,
                ),
            )
            .unwrap();
        drop(store);

        let conn = Connection::open(&path).unwrap();
        conn.execute(
            "UPDATE effect_attempts SET record_digest = 'corrupted' WHERE attempt_identity = ?1",
            params![owner.digest()],
        )
        .unwrap();
        drop(conn);

        assert!(SqliteActionFenceStore::open(&path).is_err());
    }

    #[test]
    fn fence_owner_corruption_fails_closed_on_restart() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("fence-tamper.db");
        let mut store = SqliteActionFenceStore::open(&path).unwrap();
        let action = key("fence-tamper-action");
        let owner = attempt("attempt-1");
        authorized_admit(&mut store, 
                &action,
                &owner,
                record(
                    "attempt-1",
                    "operation-1",
                    "native-1",
                    &action,
                    AttemptRecordState::Consumed,
                ),
            )
            .unwrap();
        drop(store);

        let conn = Connection::open(&path).unwrap();
        conn.execute(
            "UPDATE effect_action_fences SET owner_attempt_identity = ?1 WHERE action_key_digest = ?2",
            params![attempt("attempt-2").digest(), action.digest()],
        )
        .unwrap();
        drop(conn);

        assert!(SqliteActionFenceStore::open(&path).is_err());
    }

    #[test]
    fn corrupted_schema_profile_fails_closed() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("schema.db");
        let conn = Connection::open(&path).unwrap();
        conn.execute_batch(
            "CREATE TABLE effect_fence_store_meta (
                singleton INTEGER PRIMARY KEY,
                schema_version INTEGER NOT NULL,
                profile TEXT NOT NULL
            );
            INSERT INTO effect_fence_store_meta VALUES (1, 99, 'wrong-profile');",
        )
        .unwrap();
        drop(conn);
        assert!(SqliteActionFenceStore::open(&path).is_err());
    }
}
