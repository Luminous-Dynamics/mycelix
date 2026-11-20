//! Dependency-free retry/hedging qualification model.
//! This model refuses to infer application side-effect semantics from an
//! HTTP method string or the presence of an Idempotency-Key header. A retry
//! requires an exact endpoint contract, an exact remote idempotency contract,
//! or an attempt-bound proof that no request bytes were written.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RetryMode {
    SequentialRetry,
    Hedge,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ConcurrentDuplicateBehavior {
    CoalesceWithOriginal,
    RejectWhileInProgress,
    Unspecified,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Operation {
    logical_operation_id: u64,
    attempt_id: u64,
    workload_id: u64,
    principal_digest: u64,
    origin_digest: u64,
    method: &'static str,
    request_digest: u64,
    semantics: OperationSemantics,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct BoundSemantics {
    contract_digest: u64,
    logical_operation_id: u64,
    workload_id: u64,
    principal_digest: u64,
    origin_digest: u64,
    method: &'static str,
    request_digest: u64,
    expires_at_ns: u64,
    permits_hedging: bool,
}

impl BoundSemantics {
    fn matches(self, operation: Operation, now_ns: u64) -> bool {
        self.contract_digest != 0
            && self.logical_operation_id == operation.logical_operation_id
            && self.workload_id == operation.workload_id
            && self.principal_digest == operation.principal_digest
            && self.origin_digest == operation.origin_digest
            && self.method == operation.method
            && self.request_digest == operation.request_digest
            && now_ns < self.expires_at_ns
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct KeyedIdempotencyContract {
    contract_digest: u64,
    key_digest: u64,
    request_fingerprint_digest: u64,
    logical_operation_id: u64,
    workload_id: u64,
    principal_digest: u64,
    origin_digest: u64,
    method: &'static str,
    request_digest: u64,
    retention_until_ns: u64,
    concurrent_behavior: ConcurrentDuplicateBehavior,
    atomically_deduplicates: bool,
    durable_across_server_failover: bool,
    rejects_same_key_different_fingerprint: bool,
    returns_completion_consistent_result: bool,
}

impl KeyedIdempotencyContract {
    fn matches(self, operation: Operation, now_ns: u64) -> bool {
        self.contract_digest != 0
            && self.key_digest != 0
            && self.request_fingerprint_digest == operation.request_digest
            && self.logical_operation_id == operation.logical_operation_id
            && self.workload_id == operation.workload_id
            && self.principal_digest == operation.principal_digest
            && self.origin_digest == operation.origin_digest
            && self.method == operation.method
            && self.request_digest == operation.request_digest
            && now_ns < self.retention_until_ns
            && self.concurrent_behavior != ConcurrentDuplicateBehavior::Unspecified
            && self.atomically_deduplicates
            && self.durable_across_server_failover
            && self.rejects_same_key_different_fingerprint
            && self.returns_completion_consistent_result
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum OperationSemantics {
    ReadOnly(BoundSemantics),
    IdempotentEffect(BoundSemantics),
    KeyedDeduplication(KeyedIdempotencyContract),
    NonIdempotent,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct NoWriteProof {
    logical_operation_id: u64,
    attempt_id: u64,
    workload_id: u64,
    principal_digest: u64,
    origin_digest: u64,
    method: &'static str,
    request_digest: u64,
    adapter_generation: u64,
    proof_digest: u64,
}

impl NoWriteProof {
    fn matches(self, operation: Operation) -> bool {
        self.logical_operation_id == operation.logical_operation_id
            && self.attempt_id == operation.attempt_id
            && self.workload_id == operation.workload_id
            && self.principal_digest == operation.principal_digest
            && self.origin_digest == operation.origin_digest
            && self.method == operation.method
            && self.request_digest == operation.request_digest
            && self.adapter_generation != 0
            && self.proof_digest != 0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RemoteOperationState {
    Applied,
    DefinitelyNotApplied,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct RemoteOperationProof {
    logical_operation_id: u64,
    attempt_id: u64,
    workload_id: u64,
    principal_digest: u64,
    origin_digest: u64,
    method: &'static str,
    request_digest: u64,
    state: RemoteOperationState,
    linearizable: bool,
    server_revision: u64,
    proof_digest: u64,
}

impl RemoteOperationProof {
    fn matches(self, operation: Operation) -> bool {
        self.logical_operation_id == operation.logical_operation_id
            && self.attempt_id == operation.attempt_id
            && self.workload_id == operation.workload_id
            && self.principal_digest == operation.principal_digest
            && self.origin_digest == operation.origin_digest
            && self.method == operation.method
            && self.request_digest == operation.request_digest
            && self.server_revision != 0
            && self.proof_digest != 0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FailureEvidence {
    ProvenNoWrite(NoWriteProof),
    DispatchedOutcomeUnknown {
        logical_operation_id: u64,
        attempt_id: u64,
        dispatch_marker_sequence: u64,
    },
    RemoteStatus(RemoteOperationProof),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RetryDecision {
    PermitFreshAttempt,
    PermitKeyedReplay { key_digest: u64 },
    ReconciledApplied,
    HoldUnknown,
    DenyHedge,
    ContradictoryEvidence,
}

fn decide_retry(
    operation: Operation,
    mode: RetryMode,
    now_ns: u64,
    evidence: FailureEvidence,
) -> RetryDecision {
    match evidence {
        FailureEvidence::ProvenNoWrite(proof) => {
            if proof.matches(operation) {
                RetryDecision::PermitFreshAttempt
            } else {
                RetryDecision::ContradictoryEvidence
            }
        }
        FailureEvidence::DispatchedOutcomeUnknown {
            logical_operation_id,
            attempt_id,
            dispatch_marker_sequence,
        } => {
            if logical_operation_id != operation.logical_operation_id
                || attempt_id != operation.attempt_id
                || dispatch_marker_sequence == 0
            {
                return RetryDecision::ContradictoryEvidence;
            }

            match operation.semantics {
                OperationSemantics::ReadOnly(contract)
                | OperationSemantics::IdempotentEffect(contract) => {
                    if !contract.matches(operation, now_ns) {
                        return RetryDecision::HoldUnknown;
                    }
                    if mode == RetryMode::Hedge && !contract.permits_hedging {
                        return RetryDecision::DenyHedge;
                    }
                    RetryDecision::PermitFreshAttempt
                }
                OperationSemantics::KeyedDeduplication(contract) => {
                    if !contract.matches(operation, now_ns) {
                        return RetryDecision::HoldUnknown;
                    }
                    if mode == RetryMode::Hedge
                        && contract.concurrent_behavior
                            != ConcurrentDuplicateBehavior::CoalesceWithOriginal
                    {
                        return RetryDecision::DenyHedge;
                    }
                    RetryDecision::PermitKeyedReplay {
                        key_digest: contract.key_digest,
                    }
                }
                OperationSemantics::NonIdempotent => RetryDecision::HoldUnknown,
            }
        }
        FailureEvidence::RemoteStatus(proof) => {
            if !proof.matches(operation) {
                return RetryDecision::ContradictoryEvidence;
            }
            if !proof.linearizable {
                return RetryDecision::HoldUnknown;
            }
            match proof.state {
                RemoteOperationState::Applied => RetryDecision::ReconciledApplied,
                RemoteOperationState::DefinitelyNotApplied => match operation.semantics {
                    OperationSemantics::KeyedDeduplication(contract)
                        if contract.matches(operation, now_ns) =>
                    {
                        RetryDecision::PermitKeyedReplay {
                            key_digest: contract.key_digest,
                        }
                    }
                    _ => RetryDecision::PermitFreshAttempt,
                },
            }
        }
    }
}

fn operation(semantics: OperationSemantics) -> Operation {
    Operation {
        logical_operation_id: 10,
        attempt_id: 11,
        workload_id: 7,
        principal_digest: 101,
        origin_digest: 202,
        method: "POST",
        request_digest: 303,
        semantics,
    }
}

fn bound(permits_hedging: bool) -> BoundSemantics {
    BoundSemantics {
        contract_digest: 909,
        logical_operation_id: 10,
        workload_id: 7,
        principal_digest: 101,
        origin_digest: 202,
        method: "POST",
        request_digest: 303,
        expires_at_ns: 10_000,
        permits_hedging,
    }
}

fn keyed(behavior: ConcurrentDuplicateBehavior) -> KeyedIdempotencyContract {
    KeyedIdempotencyContract {
        contract_digest: 910,
        key_digest: 404,
        request_fingerprint_digest: 303,
        logical_operation_id: 10,
        workload_id: 7,
        principal_digest: 101,
        origin_digest: 202,
        method: "POST",
        request_digest: 303,
        retention_until_ns: 10_000,
        concurrent_behavior: behavior,
        atomically_deduplicates: true,
        durable_across_server_failover: true,
        rejects_same_key_different_fingerprint: true,
        returns_completion_consistent_result: true,
    }
}

fn unknown() -> FailureEvidence {
    FailureEvidence::DispatchedOutcomeUnknown {
        logical_operation_id: 10,
        attempt_id: 11,
        dispatch_marker_sequence: 3,
    }
}

fn no_write() -> FailureEvidence {
    FailureEvidence::ProvenNoWrite(NoWriteProof {
        logical_operation_id: 10,
        attempt_id: 11,
        workload_id: 7,
        principal_digest: 101,
        origin_digest: 202,
        method: "POST",
        request_digest: 303,
        adapter_generation: 4,
        proof_digest: 505,
    })
}

fn remote(state: RemoteOperationState) -> FailureEvidence {
    FailureEvidence::RemoteStatus(RemoteOperationProof {
        logical_operation_id: 10,
        attempt_id: 11,
        workload_id: 7,
        principal_digest: 101,
        origin_digest: 202,
        method: "POST",
        request_digest: 303,
        state,
        linearizable: true,
        server_revision: 12,
        proof_digest: 606,
    })
}

#[test]
fn non_idempotent_post_with_unknown_outcome_is_held() {
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::NonIdempotent),
            RetryMode::SequentialRetry,
            100,
            unknown(),
        ),
        RetryDecision::HoldUnknown
    );
}

#[test]
fn a_bound_read_only_contract_permits_a_sequential_retry_after_unknown() {
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::ReadOnly(bound(false))),
            RetryMode::SequentialRetry,
            100,
            unknown(),
        ),
        RetryDecision::PermitFreshAttempt
    );
}

#[test]
fn a_bound_idempotent_post_contract_permits_retry() {
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::IdempotentEffect(bound(false))),
            RetryMode::SequentialRetry,
            100,
            unknown(),
        ),
        RetryDecision::PermitFreshAttempt
    );
}

#[test]
fn method_name_alone_does_not_authorize_a_get_retry() {
    let mut op = operation(OperationSemantics::NonIdempotent);
    op.method = "GET";
    assert_eq!(
        decide_retry(op, RetryMode::SequentialRetry, 100, unknown()),
        RetryDecision::HoldUnknown
    );
}

#[test]
fn method_name_alone_does_not_authorize_a_put_retry_without_route_contract() {
    let mut op = operation(OperationSemantics::NonIdempotent);
    op.method = "PUT";
    assert_eq!(
        decide_retry(op, RetryMode::SequentialRetry, 100, unknown()),
        RetryDecision::HoldUnknown
    );
}

#[test]
fn documented_resource_semantics_can_make_post_idempotent() {
    let op = operation(OperationSemantics::IdempotentEffect(bound(false)));
    assert_eq!(
        decide_retry(op, RetryMode::SequentialRetry, 100, unknown()),
        RetryDecision::PermitFreshAttempt
    );
}

#[test]
fn expired_read_only_contract_does_not_authorize_retry() {
    let mut contract = bound(false);
    contract.expires_at_ns = 100;
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::ReadOnly(contract)),
            RetryMode::SequentialRetry,
            100,
            unknown(),
        ),
        RetryDecision::HoldUnknown
    );
}

#[test]
fn changed_request_digest_invalidates_idempotent_contract() {
    let mut op = operation(OperationSemantics::IdempotentEffect(bound(false)));
    op.request_digest += 1;
    assert_eq!(
        decide_retry(op, RetryMode::SequentialRetry, 100, unknown()),
        RetryDecision::HoldUnknown
    );
}

#[test]
fn changed_origin_invalidates_read_only_contract() {
    let mut op = operation(OperationSemantics::ReadOnly(bound(false)));
    op.origin_digest += 1;
    assert_eq!(
        decide_retry(op, RetryMode::SequentialRetry, 100, unknown()),
        RetryDecision::HoldUnknown
    );
}

#[test]
fn changed_principal_invalidates_idempotent_contract() {
    let mut op = operation(OperationSemantics::IdempotentEffect(bound(false)));
    op.principal_digest += 1;
    assert_eq!(
        decide_retry(op, RetryMode::SequentialRetry, 100, unknown()),
        RetryDecision::HoldUnknown
    );
}

#[test]
fn a_valid_atomic_keyed_contract_permits_same_key_replay() {
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::KeyedDeduplication(keyed(
                ConcurrentDuplicateBehavior::CoalesceWithOriginal
            ))),
            RetryMode::SequentialRetry,
            100,
            unknown(),
        ),
        RetryDecision::PermitKeyedReplay { key_digest: 404 }
    );
}

#[test]
fn keyed_contract_rejects_a_changed_request_fingerprint() {
    let mut contract = keyed(ConcurrentDuplicateBehavior::CoalesceWithOriginal);
    contract.request_fingerprint_digest += 1;
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::KeyedDeduplication(contract)),
            RetryMode::SequentialRetry,
            100,
            unknown(),
        ),
        RetryDecision::HoldUnknown
    );
}

#[test]
fn zero_idempotency_key_digest_is_not_authority() {
    let mut contract = keyed(ConcurrentDuplicateBehavior::CoalesceWithOriginal);
    contract.key_digest = 0;
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::KeyedDeduplication(contract)),
            RetryMode::SequentialRetry,
            100,
            unknown(),
        ),
        RetryDecision::HoldUnknown
    );
}

#[test]
fn expired_idempotency_retention_does_not_authorize_same_key_replay() {
    let mut contract = keyed(ConcurrentDuplicateBehavior::CoalesceWithOriginal);
    contract.retention_until_ns = 100;
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::KeyedDeduplication(contract)),
            RetryMode::SequentialRetry,
            100,
            unknown(),
        ),
        RetryDecision::HoldUnknown
    );
}

#[test]
fn missing_key_payload_mismatch_rejection_is_not_a_valid_contract() {
    let mut contract = keyed(ConcurrentDuplicateBehavior::CoalesceWithOriginal);
    contract.rejects_same_key_different_fingerprint = false;
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::KeyedDeduplication(contract)),
            RetryMode::SequentialRetry,
            100,
            unknown(),
        ),
        RetryDecision::HoldUnknown
    );
}

#[test]
fn missing_original_result_replay_semantics_is_not_a_valid_contract() {
    let mut contract = keyed(ConcurrentDuplicateBehavior::CoalesceWithOriginal);
    contract.returns_completion_consistent_result = false;
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::KeyedDeduplication(contract)),
            RetryMode::SequentialRetry,
            100,
            unknown(),
        ),
        RetryDecision::HoldUnknown
    );
}

#[test]
fn unspecified_concurrent_duplicate_behavior_is_not_a_valid_contract() {
    let contract = keyed(ConcurrentDuplicateBehavior::Unspecified);
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::KeyedDeduplication(contract)),
            RetryMode::SequentialRetry,
            100,
            unknown(),
        ),
        RetryDecision::HoldUnknown
    );
}

#[test]
fn reject_while_in_progress_allows_sequential_same_key_replay() {
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::KeyedDeduplication(keyed(
                ConcurrentDuplicateBehavior::RejectWhileInProgress
            ))),
            RetryMode::SequentialRetry,
            100,
            unknown(),
        ),
        RetryDecision::PermitKeyedReplay { key_digest: 404 }
    );
}

#[test]
fn reject_while_in_progress_does_not_authorize_a_hedge() {
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::KeyedDeduplication(keyed(
                ConcurrentDuplicateBehavior::RejectWhileInProgress
            ))),
            RetryMode::Hedge,
            100,
            unknown(),
        ),
        RetryDecision::DenyHedge
    );
}

#[test]
fn unspecified_concurrent_behavior_does_not_authorize_a_hedge() {
    let contract = keyed(ConcurrentDuplicateBehavior::Unspecified);
    // Even if other contract flags are populated, concurrent behavior is unknown.
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::KeyedDeduplication(contract)),
            RetryMode::Hedge,
            100,
            unknown(),
        ),
        RetryDecision::HoldUnknown
    );
}

#[test]
fn coalescing_keyed_contract_permits_a_hedge_with_the_same_key() {
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::KeyedDeduplication(keyed(
                ConcurrentDuplicateBehavior::CoalesceWithOriginal
            ))),
            RetryMode::Hedge,
            100,
            unknown(),
        ),
        RetryDecision::PermitKeyedReplay { key_digest: 404 }
    );
}

#[test]
fn read_only_hedging_requires_explicit_resource_permission() {
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::ReadOnly(bound(false))),
            RetryMode::Hedge,
            100,
            unknown(),
        ),
        RetryDecision::DenyHedge
    );
}

#[test]
fn read_only_contract_can_explicitly_allow_hedging() {
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::ReadOnly(bound(true))),
            RetryMode::Hedge,
            100,
            unknown(),
        ),
        RetryDecision::PermitFreshAttempt
    );
}

#[test]
fn idempotent_effect_hedging_requires_explicit_resource_permission() {
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::IdempotentEffect(bound(false))),
            RetryMode::Hedge,
            100,
            unknown(),
        ),
        RetryDecision::DenyHedge
    );
}

#[test]
fn non_idempotent_operation_never_hedges_from_unknown_state() {
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::NonIdempotent),
            RetryMode::Hedge,
            100,
            unknown(),
        ),
        RetryDecision::HoldUnknown
    );
}

#[test]
fn exact_attempt_bound_no_write_proof_allows_fresh_attempt() {
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::NonIdempotent),
            RetryMode::SequentialRetry,
            100,
            no_write(),
        ),
        RetryDecision::PermitFreshAttempt
    );
}

#[test]
fn no_write_proof_for_another_attempt_is_contradictory() {
    let mut evidence = no_write();
    if let FailureEvidence::ProvenNoWrite(mut proof) = evidence {
        proof.attempt_id += 1;
        evidence = FailureEvidence::ProvenNoWrite(proof);
    }
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::NonIdempotent),
            RetryMode::SequentialRetry,
            100,
            evidence,
        ),
        RetryDecision::ContradictoryEvidence
    );
}

#[test]
fn no_write_proof_for_another_request_digest_is_contradictory() {
    let mut evidence = no_write();
    if let FailureEvidence::ProvenNoWrite(mut proof) = evidence {
        proof.request_digest += 1;
        evidence = FailureEvidence::ProvenNoWrite(proof);
    }
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::NonIdempotent),
            RetryMode::SequentialRetry,
            100,
            evidence,
        ),
        RetryDecision::ContradictoryEvidence
    );
}

#[test]
fn linearizable_applied_proof_reconciles_without_retry() {
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::NonIdempotent),
            RetryMode::SequentialRetry,
            100,
            remote(RemoteOperationState::Applied),
        ),
        RetryDecision::ReconciledApplied
    );
}

#[test]
fn linearizable_not_applied_proof_allows_non_idempotent_fresh_attempt() {
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::NonIdempotent),
            RetryMode::SequentialRetry,
            100,
            remote(RemoteOperationState::DefinitelyNotApplied),
        ),
        RetryDecision::PermitFreshAttempt
    );
}

#[test]
fn keyed_operation_reuses_same_key_after_authoritative_not_applied_proof() {
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::KeyedDeduplication(keyed(
                ConcurrentDuplicateBehavior::CoalesceWithOriginal
            ))),
            RetryMode::SequentialRetry,
            100,
            remote(RemoteOperationState::DefinitelyNotApplied),
        ),
        RetryDecision::PermitKeyedReplay { key_digest: 404 }
    );
}

#[test]
fn applied_remote_proof_for_another_logical_operation_is_contradictory() {
    let mut evidence = remote(RemoteOperationState::Applied);
    if let FailureEvidence::RemoteStatus(mut proof) = evidence {
        proof.logical_operation_id += 1;
        evidence = FailureEvidence::RemoteStatus(proof);
    }
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::NonIdempotent),
            RetryMode::SequentialRetry,
            100,
            evidence,
        ),
        RetryDecision::ContradictoryEvidence
    );
}

#[test]
fn remote_proof_for_another_origin_is_contradictory() {
    let mut evidence = remote(RemoteOperationState::DefinitelyNotApplied);
    if let FailureEvidence::RemoteStatus(mut proof) = evidence {
        proof.origin_digest += 1;
        evidence = FailureEvidence::RemoteStatus(proof);
    }
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::NonIdempotent),
            RetryMode::SequentialRetry,
            100,
            evidence,
        ),
        RetryDecision::ContradictoryEvidence
    );
}

#[test]
fn eventually_consistent_remote_absence_remains_unknown() {
    let mut evidence = remote(RemoteOperationState::DefinitelyNotApplied);
    if let FailureEvidence::RemoteStatus(mut proof) = evidence {
        proof.linearizable = false;
        evidence = FailureEvidence::RemoteStatus(proof);
    }
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::NonIdempotent),
            RetryMode::SequentialRetry,
            100,
            evidence,
        ),
        RetryDecision::HoldUnknown
    );
}

#[test]
fn missing_remote_proof_digest_does_not_authorize_retry() {
    let mut evidence = remote(RemoteOperationState::DefinitelyNotApplied);
    if let FailureEvidence::RemoteStatus(mut proof) = evidence {
        proof.proof_digest = 0;
        evidence = FailureEvidence::RemoteStatus(proof);
    }
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::NonIdempotent),
            RetryMode::SequentialRetry,
            100,
            evidence,
        ),
        RetryDecision::ContradictoryEvidence
    );
}

#[test]
fn dispatched_unknown_evidence_must_match_exact_attempt_and_marker() {
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::ReadOnly(bound(false))),
            RetryMode::SequentialRetry,
            100,
            FailureEvidence::DispatchedOutcomeUnknown {
                logical_operation_id: 10,
                attempt_id: 12,
                dispatch_marker_sequence: 3,
            },
        ),
        RetryDecision::ContradictoryEvidence
    );
}

#[test]
fn generated_non_idempotent_replay_sequences_never_gain_authority() {
    for attempt_id in 1..=128 {
        let mut op = operation(OperationSemantics::NonIdempotent);
        op.attempt_id = attempt_id;
        let evidence = FailureEvidence::DispatchedOutcomeUnknown {
            logical_operation_id: op.logical_operation_id,
            attempt_id,
            dispatch_marker_sequence: attempt_id + 10,
        };
        assert_eq!(
            decide_retry(op, RetryMode::SequentialRetry, 100, evidence),
            RetryDecision::HoldUnknown,
            "non-idempotent operation attempt {attempt_id} must remain unknown"
        );
    }
}


#[test]
fn keyed_replay_requires_atomic_server_deduplication() {
    let mut contract = keyed(ConcurrentDuplicateBehavior::CoalesceWithOriginal);
    contract.atomically_deduplicates = false;
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::KeyedDeduplication(contract)),
            RetryMode::SequentialRetry,
            100,
            unknown(),
        ),
        RetryDecision::HoldUnknown
    );
}

#[test]
fn keyed_replay_requires_durable_deduplication_across_server_failover() {
    let mut contract = keyed(ConcurrentDuplicateBehavior::CoalesceWithOriginal);
    contract.durable_across_server_failover = false;
    assert_eq!(
        decide_retry(
            operation(OperationSemantics::KeyedDeduplication(contract)),
            RetryMode::SequentialRetry,
            100,
            unknown(),
        ),
        RetryDecision::HoldUnknown
    );
}
