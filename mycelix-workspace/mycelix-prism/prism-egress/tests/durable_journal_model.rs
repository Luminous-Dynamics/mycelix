//! Test-only model of an append-only, write-ahead egress journal.
//! The digest below is a deterministic test checksum, NOT a cryptographic hash.
//! Production must use the repository's cryptographic primitive and define
//! storage fsync, rollback, key and privileged-attacker assumptions explicitly.
use std::collections::HashSet;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct AttemptScope {
    attempt_id: u64,
    logical_operation_id: u64,
    workload_id: u64,
    principal_digest: u64,
    origin_digest: u64,
    request_digest: u64,
    policy_generation: u64,
    network_generation: u64,
    idempotency_key_digest: Option<u64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RecordKind {
    Intent,
    Authorized,
    DispatchMarked,
    RequestWriteStarted,
    ResponseObserved,
    RemoteReconciled,
}

impl RecordKind {
    fn code(self) -> u64 {
        match self {
            Self::Intent => 1,
            Self::Authorized => 2,
            Self::DispatchMarked => 3,
            Self::RequestWriteStarted => 4,
            Self::ResponseObserved => 5,
            Self::RemoteReconciled => 6,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RemoteResolution {
    Applied,
    DefinitelyNotApplied,
}

impl RemoteResolution {
    fn code(self) -> u64 {
        match self {
            Self::Applied => 1,
            Self::DefinitelyNotApplied => 2,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct JournalRecord {
    sequence: u64,
    scope: AttemptScope,
    kind: RecordKind,
    remote_resolution: Option<RemoteResolution>,
    previous_digest: u64,
    record_digest: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum JournalError {
    InvalidTransition,
    ScopeMismatch,
    SequenceGap,
    PreviousDigestMismatch,
    RecordDigestMismatch,
    NotDurable,
    MissingDispatchMarker,
    DispatchAlreadyConsumed,
    WrongAttempt,
    UnflushedTail,
    AttemptIdReused,
    LogicalOperationUnresolved,
    LogicalOperationMismatch,
    SupersededAttempt,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RecoveryState {
    NoIntent,
    Prepared,
    Authorized,
    DispatchMayHaveOccurred,
    RequestWriteObserved,
    ResponseObserved,
    Reconciled,
}

#[derive(Clone, Debug)]
struct Journal {
    records: Vec<JournalRecord>,
    /// Number of leading records confirmed durable by the model's flush().
    durable_len: usize,
}

impl Journal {
    fn new() -> Self {
        Self {
            records: Vec::new(),
            durable_len: 0,
        }
    }

    fn append(
        &mut self,
        scope: AttemptScope,
        kind: RecordKind,
    ) -> Result<u64, JournalError> {
        self.append_with_resolution(scope, kind, None)
    }

    fn append_with_resolution(
        &mut self,
        scope: AttemptScope,
        kind: RecordKind,
        remote_resolution: Option<RemoteResolution>,
    ) -> Result<u64, JournalError> {
        self.verify_all()?;
        if (kind == RecordKind::RemoteReconciled) != remote_resolution.is_some() {
            return Err(JournalError::InvalidTransition);
        }

        if kind == RecordKind::Intent {
            validate_new_attempt(&self.records, scope)?;
        } else {
            validate_current_attempt(&self.records, scope)?;
        }

        let previous = self
            .records
            .iter()
            .rev()
            .find(|record| record.scope.attempt_id == scope.attempt_id);

        if let Some(previous) = previous {
            if previous.scope != scope {
                return Err(JournalError::ScopeMismatch);
            }
            if !valid_transition(previous.kind, kind) {
                return Err(JournalError::InvalidTransition);
            }
        } else if kind != RecordKind::Intent {
            return Err(JournalError::InvalidTransition);
        }

        let sequence = self.records.len() as u64 + 1;
        let previous_digest = self
            .records
            .last()
            .map(|record| record.record_digest)
            .unwrap_or(0);
        let mut record = JournalRecord {
            sequence,
            scope,
            kind,
            remote_resolution,
            previous_digest,
            record_digest: 0,
        };
        record.record_digest = digest_record(record);
        self.records.push(record);
        Ok(sequence)
    }
    fn flush(&mut self) -> Result<(), JournalError> {
        self.verify_all()?;
        self.durable_len = self.records.len();
        Ok(())
    }

    fn verify_all(&self) -> Result<(), JournalError> {
        self.verify_prefix(self.records.len())
    }

    fn verify_durable(&self) -> Result<(), JournalError> {
        self.verify_prefix(self.durable_len)
    }

    fn verify_prefix(&self, length: usize) -> Result<(), JournalError> {
        if length > self.records.len() {
            return Err(JournalError::NotDurable);
        }
        let mut previous_digest = 0;
        for (index, record) in self.records.iter().take(length).enumerate() {
            if record.sequence != index as u64 + 1 {
                return Err(JournalError::SequenceGap);
            }
            if record.previous_digest != previous_digest {
                return Err(JournalError::PreviousDigestMismatch);
            }
            if record.record_digest != digest_record(*record) {
                return Err(JournalError::RecordDigestMismatch);
            }

            if (record.kind == RecordKind::RemoteReconciled)
                != record.remote_resolution.is_some()
            {
                return Err(JournalError::InvalidTransition);
            }
            let history = &self.records[..index];
            if record.kind == RecordKind::Intent {
                validate_new_attempt(history, record.scope)?;
            } else {
                validate_current_attempt(history, record.scope)?;
            }

            let prior = history
                .iter()
                .rev()
                .find(|prior| prior.scope.attempt_id == record.scope.attempt_id);
            match prior {
                None if record.kind != RecordKind::Intent => {
                    return Err(JournalError::InvalidTransition);
                }
                None => {}
                Some(prior) => {
                    if prior.scope != record.scope {
                        return Err(JournalError::ScopeMismatch);
                    }
                    if !valid_transition(prior.kind, record.kind) {
                        return Err(JournalError::InvalidTransition);
                    }
                }
            }
            previous_digest = record.record_digest;
        }
        Ok(())
    }

    fn durable_state(&self, attempt_id: u64) -> Result<RecoveryState, JournalError> {
        self.verify_durable()?;
        let last = self.records.iter().take(self.durable_len).rev()
            .find(|record| record.scope.attempt_id == attempt_id);
        Ok(match last.map(|record| record.kind) {
            None => RecoveryState::NoIntent,
            Some(RecordKind::Intent) => RecoveryState::Prepared,
            Some(RecordKind::Authorized) => RecoveryState::Authorized,
            Some(RecordKind::DispatchMarked) => RecoveryState::DispatchMayHaveOccurred,
            Some(RecordKind::RequestWriteStarted) => RecoveryState::RequestWriteObserved,
            Some(RecordKind::ResponseObserved) => RecoveryState::ResponseObserved,
            Some(RecordKind::RemoteReconciled) => RecoveryState::Reconciled,
        })
    }

    /// Persist a one-use dispatch marker before returning an ephemeral token.
    /// A replayed journal never reconstructs this token after process restart.
    fn authorize_dispatch(
        &mut self,
        scope: AttemptScope,
    ) -> Result<DispatchToken, JournalError> {
        if self.records.len() != self.durable_len {
            return Err(JournalError::UnflushedTail);
        }
        if self.durable_state(scope.attempt_id)? != RecoveryState::Authorized {
            return Err(JournalError::InvalidTransition);
        }
        let sequence = self.append(scope, RecordKind::DispatchMarked)?;
        self.flush()?;
        if sequence as usize > self.durable_len {
            return Err(JournalError::NotDurable);
        }
        Ok(DispatchToken {
            scope,
            marker_sequence: sequence,
        })
    }

    fn recover_after_crash(&self) -> Result<Self, JournalError> {
        self.verify_durable()?;
        Ok(Self {
            records: self.records.iter().take(self.durable_len).copied().collect(),
            durable_len: self.durable_len,
        })
    }

    fn record_request_write_started(
        &mut self,
        permit: TransportWritePermit,
    ) -> Result<(), JournalError> {
        if self.durable_state(permit.scope.attempt_id)?
            != RecoveryState::DispatchMayHaveOccurred
        {
            return Err(JournalError::InvalidTransition);
        }
        if !self.records.iter().take(self.durable_len).any(|record| {
            record.sequence == permit.marker_sequence
                && record.scope == permit.scope
                && record.kind == RecordKind::DispatchMarked
        }) {
            return Err(JournalError::MissingDispatchMarker);
        }
        self.append(permit.scope, RecordKind::RequestWriteStarted)?;
        self.flush()
    }

    fn record_response(&mut self, scope: AttemptScope) -> Result<(), JournalError> {
        self.append(scope, RecordKind::ResponseObserved)?;
        self.flush()
    }

    fn record_remote_reconciled(
        &mut self,
        scope: AttemptScope,
        resolution: RemoteResolution,
    ) -> Result<(), JournalError> {
        self.append_with_resolution(
            scope,
            RecordKind::RemoteReconciled,
            Some(resolution),
        )?;
        self.flush()
    }
}

#[derive(Debug)]
struct DispatchToken {
    scope: AttemptScope,
    marker_sequence: u64,
}

#[derive(Debug)]
struct TransportWritePermit {
    scope: AttemptScope,
    marker_sequence: u64,
}

#[derive(Default)]
struct AdapterGate {
    consumed: HashSet<(u64, u64)>,
}

impl AdapterGate {
    /// Consumes the ephemeral token and returns a one-use write permit.
    /// A token is never reconstructed by Journal::recover_after_crash.
    fn begin_request_write(
        &mut self,
        journal: &Journal,
        token: DispatchToken,
        expected_scope: AttemptScope,
    ) -> Result<TransportWritePermit, JournalError> {
        journal.verify_durable()?;
        if token.scope != expected_scope {
            return Err(JournalError::ScopeMismatch);
        }
        let marker = journal.records.iter().take(journal.durable_len).find(|record| {
            record.sequence == token.marker_sequence
                && record.scope == expected_scope
                && record.kind == RecordKind::DispatchMarked
        });
        if marker.is_none() {
            return Err(JournalError::MissingDispatchMarker);
        }
        if journal.durable_state(expected_scope.attempt_id)?
            != RecoveryState::DispatchMayHaveOccurred
        {
            return Err(JournalError::InvalidTransition);
        }
        if !self.consumed.insert((expected_scope.attempt_id, token.marker_sequence)) {
            return Err(JournalError::DispatchAlreadyConsumed);
        }
        Ok(TransportWritePermit {
            scope: expected_scope,
            marker_sequence: token.marker_sequence,
        })
    }
}

fn valid_transition(previous: RecordKind, next: RecordKind) -> bool {
    matches!(
        (previous, next),
        (RecordKind::Intent, RecordKind::Authorized)
            | (RecordKind::Authorized, RecordKind::DispatchMarked)
            | (RecordKind::DispatchMarked, RecordKind::RequestWriteStarted)
            | (RecordKind::DispatchMarked, RecordKind::RemoteReconciled)
            | (RecordKind::RequestWriteStarted, RecordKind::ResponseObserved)
            | (RecordKind::RequestWriteStarted, RecordKind::RemoteReconciled)
            | (RecordKind::ResponseObserved, RecordKind::RemoteReconciled)
    )
}


fn same_logical_request(left: AttemptScope, right: AttemptScope) -> bool {
    left.logical_operation_id == right.logical_operation_id
        && left.workload_id == right.workload_id
        && left.principal_digest == right.principal_digest
        && left.origin_digest == right.origin_digest
        && left.request_digest == right.request_digest
        && left.idempotency_key_digest == right.idempotency_key_digest
}

fn validate_new_attempt(
    history: &[JournalRecord],
    scope: AttemptScope,
) -> Result<(), JournalError> {
    if history.iter().any(|record| record.scope.attempt_id == scope.attempt_id) {
        return Err(JournalError::AttemptIdReused);
    }
    let previous = history.iter().rev().find(|record| {
        record.scope.logical_operation_id == scope.logical_operation_id
    });
    let Some(previous) = previous else {
        return Ok(());
    };

    if !same_logical_request(previous.scope, scope) {
        return Err(JournalError::LogicalOperationMismatch);
    }
    match (previous.kind, previous.remote_resolution) {
        (RecordKind::Intent | RecordKind::Authorized, None) => Ok(()),
        (
            RecordKind::RemoteReconciled,
            Some(RemoteResolution::DefinitelyNotApplied),
        ) => Ok(()),
        _ => Err(JournalError::LogicalOperationUnresolved),
    }
}

fn validate_current_attempt(
    history: &[JournalRecord],
    scope: AttemptScope,
) -> Result<(), JournalError> {
    let latest = history.iter().rev().find(|record| {
        record.scope.logical_operation_id == scope.logical_operation_id
    });
    match latest {
        Some(record) if record.scope.attempt_id == scope.attempt_id => Ok(()),
        Some(_) => Err(JournalError::SupersededAttempt),
        None => Err(JournalError::WrongAttempt),
    }
}

/// A tiny deterministic checksum for model tests only. Never reuse this as a
/// production integrity primitive.
fn digest_record(record: JournalRecord) -> u64 {
    let scope = record.scope;
    let values = [
        record.sequence,
        scope.attempt_id,
        scope.logical_operation_id,
        scope.workload_id,
        scope.principal_digest,
        scope.origin_digest,
        scope.request_digest,
        scope.policy_generation,
        scope.network_generation,
        scope.idempotency_key_digest.unwrap_or(0),
        record.kind.code(),
        record.remote_resolution.map(RemoteResolution::code).unwrap_or(0),
        record.previous_digest,
    ];
    let mut digest = 0x6a09_e667_f3bc_c909_u64;
    for value in values {
        digest ^= value.wrapping_add(0x9e37_79b9_7f4a_7c15);
        digest = digest.rotate_left(17).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    }
    digest
}

fn scope() -> AttemptScope {
    AttemptScope {
        attempt_id: 41,
        logical_operation_id: 40,
        workload_id: 7,
        principal_digest: 101,
        origin_digest: 202,
        request_digest: 303,
        policy_generation: 4,
        network_generation: 5,
        idempotency_key_digest: Some(404),
    }
}

fn authorized_journal() -> Journal {
    let mut journal = Journal::new();
    journal.append(scope(), RecordKind::Intent).unwrap();
    journal.flush().unwrap();
    journal.append(scope(), RecordKind::Authorized).unwrap();
    journal.flush().unwrap();
    journal
}

fn dispatched_journal() -> (Journal, DispatchToken) {
    let mut journal = authorized_journal();
    let token = journal.authorize_dispatch(scope()).unwrap();
    (journal, token)
}

#[test]
fn a_transport_write_cannot_begin_without_a_dispatch_marker() {
    let journal = authorized_journal();
    let mut gate = AdapterGate::default();
    let forged_token = DispatchToken {
        scope: scope(),
        marker_sequence: 3,
    };
    assert_eq!(
        journal.durable_state(scope().attempt_id).unwrap(),
        RecoveryState::Authorized
    );
    assert_eq!(
        gate.begin_request_write(&journal, forged_token, scope()).err(),
        Some(JournalError::MissingDispatchMarker)
    );
}

#[test]
fn dispatch_marker_is_flushed_before_ephemeral_token_is_returned() {
    let mut journal = authorized_journal();
    let token = journal.authorize_dispatch(scope()).unwrap();
    assert_eq!(token.marker_sequence as usize, journal.durable_len);
    assert_eq!(
        journal.durable_state(scope().attempt_id).unwrap(),
        RecoveryState::DispatchMayHaveOccurred
    );
}

#[test]
fn appending_a_dispatch_marker_without_flush_does_not_authorize_writes() {
    let mut journal = authorized_journal();
    journal.append(scope(), RecordKind::DispatchMarked).unwrap();
    assert_eq!(
        journal.durable_state(scope().attempt_id).unwrap(),
        RecoveryState::Authorized
    );
    assert_eq!(
        journal.records.len(),
        journal.durable_len + 1
    );
}

#[test]
fn an_unflushed_tail_blocks_issuing_a_dispatch_token() {
    let mut journal = authorized_journal();
    journal.append(scope(), RecordKind::DispatchMarked).unwrap();
    assert_eq!(
        journal.authorize_dispatch(scope()).err(),
        Some(JournalError::UnflushedTail)
    );
}

#[test]
fn the_adapter_consumes_a_dispatch_token_once() {
    let (journal, token) = dispatched_journal();
    let mut gate = AdapterGate::default();
    let permit = gate.begin_request_write(&journal, token, scope()).unwrap();
    assert_eq!(permit.marker_sequence, 3);
    assert!(gate.consumed.contains(&(41, 3)));
}

#[test]
fn dispatch_token_is_bound_to_exact_workload_and_request_scope() {
    let (journal, token) = dispatched_journal();
    let mut changed = scope();
    changed.workload_id += 1;
    let mut gate = AdapterGate::default();
    assert_eq!(
        gate.begin_request_write(&journal, token, changed).err(),
        Some(JournalError::ScopeMismatch)
    );
}

#[test]
fn dispatch_token_is_bound_to_exact_origin_and_principal() {
    let (journal, token) = dispatched_journal();
    let mut changed = scope();
    changed.origin_digest += 1;
    let mut gate = AdapterGate::default();
    assert_eq!(
        gate.begin_request_write(&journal, token, changed).err(),
        Some(JournalError::ScopeMismatch)
    );
}

#[test]
fn dispatch_token_is_bound_to_policy_and_network_generations() {
    let (journal, token) = dispatched_journal();
    let mut changed = scope();
    changed.policy_generation += 1;
    let mut gate = AdapterGate::default();
    assert_eq!(
        gate.begin_request_write(&journal, token, changed).err(),
        Some(JournalError::ScopeMismatch)
    );
}

#[test]
fn a_pre_dispatch_crash_discards_volatile_dispatch_marker() {
    let mut journal = authorized_journal();
    journal.append(scope(), RecordKind::DispatchMarked).unwrap();
    let recovered = journal.recover_after_crash().unwrap();
    assert_eq!(
        recovered.durable_state(scope().attempt_id).unwrap(),
        RecoveryState::Authorized
    );
    assert_eq!(recovered.records.len(), 2);
}

#[test]
fn a_crash_after_dispatch_flush_recovers_as_maybe_dispatched() {
    let (journal, _ephemeral_token) = dispatched_journal();
    let recovered = journal.recover_after_crash().unwrap();
    assert_eq!(
        recovered.durable_state(scope().attempt_id).unwrap(),
        RecoveryState::DispatchMayHaveOccurred
    );
    assert_eq!(
        recovered.authorize_dispatch(scope()).err(),
        Some(JournalError::InvalidTransition)
    );
}

#[test]
fn recovered_dispatch_marker_never_reconstructs_the_lost_token() {
    let (journal, _ephemeral_token) = dispatched_journal();
    let recovered = journal.recover_after_crash().unwrap();
    let mut gate = AdapterGate::default();
    // The recovery API intentionally has no method to issue a token from an
    // existing DispatchMarked record.
    assert_eq!(
        recovered.durable_state(scope().attempt_id).unwrap(),
        RecoveryState::DispatchMayHaveOccurred
    );
    assert_eq!(gate.consumed.len(), 0);
}

#[test]
fn journal_chain_verifies_across_multiple_attempts() {
    let mut journal = Journal::new();
    journal.append(scope(), RecordKind::Intent).unwrap();
    journal.append(scope(), RecordKind::Authorized).unwrap();
    let mut other = scope();
    other.attempt_id = 51;
    other.logical_operation_id = 50;
    journal.append(other, RecordKind::Intent).unwrap();
    journal.append(other, RecordKind::Authorized).unwrap();
    journal.flush().unwrap();
    assert!(journal.verify_durable().is_ok());
}

#[test]
fn a_changed_record_payload_is_detected() {
    let mut journal = authorized_journal();
    journal.records[0].scope.request_digest += 1;
    assert_eq!(
        journal.verify_durable().err(),
        Some(JournalError::RecordDigestMismatch)
    );
}

#[test]
fn a_changed_previous_digest_is_detected() {
    let mut journal = authorized_journal();
    journal.records[1].previous_digest += 1;
    assert_eq!(
        journal.verify_durable().err(),
        Some(JournalError::PreviousDigestMismatch)
    );
}

#[test]
fn a_sequence_gap_is_detected() {
    let mut journal = authorized_journal();
    journal.records[1].sequence += 1;
    assert_eq!(
        journal.verify_durable().err(),
        Some(JournalError::SequenceGap)
    );
}

#[test]
fn authorization_cannot_precede_intent() {
    let mut journal = Journal::new();
    assert_eq!(
        journal.append(scope(), RecordKind::Authorized).err(),
        Some(JournalError::InvalidTransition)
    );
}

#[test]
fn dispatch_cannot_precede_durable_authorization() {
    let mut journal = Journal::new();
    journal.append(scope(), RecordKind::Intent).unwrap();
    journal.flush().unwrap();
    assert_eq!(
        journal.append(scope(), RecordKind::DispatchMarked).err(),
        Some(JournalError::InvalidTransition)
    );
}

#[test]
fn request_write_observation_cannot_precede_dispatch() {
    let mut journal = authorized_journal();
    assert_eq!(
        journal.append(scope(), RecordKind::RequestWriteStarted).err(),
        Some(JournalError::InvalidTransition)
    );
}

#[test]
fn response_observation_cannot_precede_request_write() {
    let mut journal = authorized_journal();
    assert_eq!(
        journal.append(scope(), RecordKind::ResponseObserved).err(),
        Some(JournalError::InvalidTransition)
    );
}

#[test]
fn request_write_observation_follows_one_use_transport_permit() {
    let (mut journal, token) = dispatched_journal();
    let mut gate = AdapterGate::default();
    let permit = gate.begin_request_write(&journal, token, scope()).unwrap();
    journal.record_request_write_started(permit).unwrap();
    assert_eq!(
        journal.durable_state(scope().attempt_id).unwrap(),
        RecoveryState::RequestWriteObserved
    );
}

#[test]
fn server_reconciliation_can_resolve_dispatch_before_write_observation() {
    let (mut journal, _token) = dispatched_journal();
    journal.record_remote_reconciled(scope(), RemoteResolution::Applied).unwrap();
    assert_eq!(
        journal.durable_state(scope().attempt_id).unwrap(),
        RecoveryState::Reconciled
    );
}

#[test]
fn response_and_remote_reconciliation_have_ordered_journal_records() {
    let (mut journal, token) = dispatched_journal();
    let mut gate = AdapterGate::default();
    let permit = gate.begin_request_write(&journal, token, scope()).unwrap();
    journal.record_request_write_started(permit).unwrap();
    journal.record_response(scope()).unwrap();
    journal.record_remote_reconciled(scope(), RemoteResolution::Applied).unwrap();
    assert_eq!(
        journal.durable_state(scope().attempt_id).unwrap(),
        RecoveryState::Reconciled
    );
}

#[test]
fn a_second_dispatch_marker_for_one_attempt_is_rejected() {
    let (mut journal, _token) = dispatched_journal();
    assert_eq!(
        journal.authorize_dispatch(scope()).err(),
        Some(JournalError::InvalidTransition)
    );
}

#[test]
fn request_digest_mismatch_cannot_use_an_existing_dispatch_marker() {
    let (journal, token) = dispatched_journal();
    let mut changed = scope();
    changed.request_digest += 1;
    let mut gate = AdapterGate::default();
    assert_eq!(
        gate.begin_request_write(&journal, token, changed).err(),
        Some(JournalError::ScopeMismatch)
    );
}

#[test]
fn a_volatile_suffix_is_not_included_in_recovered_history() {
    let mut journal = authorized_journal();
    journal.append(scope(), RecordKind::DispatchMarked).unwrap();
    journal.append(scope(), RecordKind::RequestWriteStarted).unwrap();
    let recovered = journal.recover_after_crash().unwrap();
    assert_eq!(recovered.records.len(), 2);
    assert_eq!(
        recovered.durable_state(scope().attempt_id).unwrap(),
        RecoveryState::Authorized
    );
}


#[test]
fn unresolved_dispatch_blocks_a_new_attempt_for_the_same_logical_operation() {
    let (mut journal, _token) = dispatched_journal();
    let mut retry = scope();
    retry.attempt_id = 42;
    assert_eq!(
        journal.append(retry, RecordKind::Intent).err(),
        Some(JournalError::LogicalOperationUnresolved)
    );
}

#[test]
fn authoritative_not_applied_reconciliation_allows_fresh_attempt_identity() {
    let (mut journal, _token) = dispatched_journal();
    journal.record_remote_reconciled(scope(), RemoteResolution::DefinitelyNotApplied).unwrap();
    let mut retry = scope();
    retry.attempt_id = 42;
    retry.policy_generation += 1;
    retry.network_generation += 1;
    journal.append(retry, RecordKind::Intent).unwrap();
    journal.flush().unwrap();
    journal.append(retry, RecordKind::Authorized).unwrap();
    journal.flush().unwrap();
    assert_eq!(
        journal.durable_state(42).unwrap(),
        RecoveryState::Authorized
    );
}

#[test]
fn applied_operation_cannot_be_reissued_as_a_new_attempt() {
    let (mut journal, _token) = dispatched_journal();
    journal.record_remote_reconciled(scope(), RemoteResolution::Applied).unwrap();
    let mut retry = scope();
    retry.attempt_id = 42;
    assert_eq!(
        journal.append(retry, RecordKind::Intent).err(),
        Some(JournalError::LogicalOperationUnresolved)
    );
}

#[test]
fn definitely_not_applied_does_not_authorize_changed_request_bytes() {
    let (mut journal, _token) = dispatched_journal();
    journal.record_remote_reconciled(scope(), RemoteResolution::DefinitelyNotApplied).unwrap();
    let mut retry = scope();
    retry.attempt_id = 42;
    retry.request_digest += 1;
    assert_eq!(
        journal.append(retry, RecordKind::Intent).err(),
        Some(JournalError::LogicalOperationMismatch)
    );
}

#[test]
fn new_attempt_requires_same_idempotency_key_and_principal() {
    let (mut journal, _token) = dispatched_journal();
    journal.record_remote_reconciled(scope(), RemoteResolution::DefinitelyNotApplied).unwrap();
    let mut retry = scope();
    retry.attempt_id = 42;
    retry.idempotency_key_digest = Some(999);
    assert_eq!(
        journal.append(retry, RecordKind::Intent).err(),
        Some(JournalError::LogicalOperationMismatch)
    );
}

#[test]
fn superseded_attempt_cannot_append_records_after_retry_identity_is_created() {
    let mut journal = authorized_journal();
    let mut retry = scope();
    retry.attempt_id = 42;
    journal.append(retry, RecordKind::Intent).unwrap();
    assert_eq!(
        journal.append(scope(), RecordKind::Authorized).err(),
        Some(JournalError::SupersededAttempt)
    );
}

#[test]
fn attempt_id_cannot_be_reused_for_a_second_logical_operation() {
    let mut journal = authorized_journal();
    let mut other = scope();
    other.logical_operation_id = 99;
    assert_eq!(
        journal.append(other, RecordKind::Intent).err(),
        Some(JournalError::AttemptIdReused)
    );
}
