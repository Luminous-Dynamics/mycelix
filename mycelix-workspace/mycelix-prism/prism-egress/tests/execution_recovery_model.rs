//! Dependency-free model for crash recovery around outbound request side effects.
//! This is test-only contract work. It is not a durable journal, transport adapter,
//! server idempotency implementation, or production recovery mechanism.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum JournalPhase {
    Prepared,
    Authorized,
    DispatchMarked,
}

#[derive(Clone, Copy, Debug)]
struct Attempt {
    attempt_id: u64,
    workload_id: u64,
    origin_digest: u64,
    principal_digest: u64,
    request_digest: u64,
    idempotency_key: Option<u64>,
    phase: JournalPhase,
    journal_durable: bool,
}

#[derive(Clone, Copy, Debug)]
struct IdempotencyContract {
    origin_digest: u64,
    principal_digest: u64,
    request_digest: u64,
    key_digest: u64,
    deduplicates_concurrent_requests: bool,
    lookup_is_linearizable: bool,
    retention_until_ns: u64,
}

impl IdempotencyContract {
    fn matches(&self, attempt: Attempt, now_ns: u64) -> bool {
        attempt.idempotency_key == Some(self.key_digest)
            && self.origin_digest == attempt.origin_digest
            && self.principal_digest == attempt.principal_digest
            && self.request_digest == attempt.request_digest
            && self.deduplicates_concurrent_requests
            && self.lookup_is_linearizable
            && now_ns < self.retention_until_ns
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RemoteState {
    Applied,
    DefinitelyNotApplied,
    Unknown,
}

#[derive(Clone, Copy, Debug)]
struct RemoteProof {
    attempt_id: u64,
    origin_digest: u64,
    principal_digest: u64,
    request_digest: u64,
    idempotency_key: Option<u64>,
    state: RemoteState,
    linearizable: bool,
    server_revision: u64,
    proof_digest: u64,
}

impl RemoteProof {
    fn matches(&self, attempt: Attempt, contract: IdempotencyContract, now_ns: u64) -> bool {
        self.attempt_id == attempt.attempt_id
            && self.origin_digest == attempt.origin_digest
            && self.principal_digest == attempt.principal_digest
            && self.request_digest == attempt.request_digest
            && self.idempotency_key == attempt.idempotency_key
            && contract.matches(attempt, now_ns)
            && self.linearizable
            && self.server_revision != 0
            && self.proof_digest != 0
    }
}

#[derive(Clone, Copy, Debug)]
enum TransportObservation {
    Unknown,
    /// A transport adapter's positive proof that no request bytes were handed
    /// to the transport for this exact attempt. A missing socket event is not
    /// a substitute for this proof.
    ProvenNoWrite {
        attempt_id: u64,
        request_digest: u64,
        adapter_generation: u64,
        proof_digest: u64,
    },
    ResponseReceived {
        connection_id: u64,
        peer_digest: u64,
        response_digest: u64,
    },
}

#[derive(Clone, Copy, Debug)]
struct KernelConnectEvent {
    generation: u64,
    connection_id: u64,
    workload_id: u64,
    peer_digest: u64,
    allowed: bool,
}

#[derive(Clone, Debug)]
enum KernelObserver {
    Complete {
        generation: u64,
        events: Vec<KernelConnectEvent>,
    },
    EventLoss,
    CoverageGap,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RecoveryDecision {
    /// A new request attempt may be created, but it needs fresh authority.
    SafeToAuthorizeFreshAttempt,
    /// A non-mutating authoritative status lookup is the next allowed action.
    QueryRemoteIdempotencyState,
    /// An authoritative absence proof permits a new attempt using the same
    /// logical idempotency key and a fresh attempt/capability identity.
    SafeToAuthorizeFreshAttemptWithSameKey,
    ReconciledApplied,
    /// The application response and socket identity/peer corroborate at the
    /// transport layer only. This is not proof of HTTP business effect.
    ResponseAndSocketCorroborated,
    /// A response exists, but independent socket corroboration is incomplete.
    ResponseObservedWithoutIndependentSocketCorroboration,
    HoldUnknown,
    Contradiction,
}

fn response_socket_assessment(
    attempt: Attempt,
    transport: TransportObservation,
    observer: &KernelObserver,
) -> RecoveryDecision {
    let TransportObservation::ResponseReceived {
        connection_id,
        peer_digest,
        response_digest,
    } = transport
    else {
        return RecoveryDecision::HoldUnknown;
    };

    if response_digest == 0 || connection_id == 0 || peer_digest == 0 {
        return RecoveryDecision::Contradiction;
    }

    match observer {
        KernelObserver::Complete { generation, events } => {
            let matching_connection: Vec<&KernelConnectEvent> = events
                .iter()
                .filter(|event| event.connection_id == connection_id)
                .collect();

            if matching_connection.iter().any(|event| {
                event.generation != *generation
                    || event.workload_id != attempt.workload_id
                    || event.peer_digest != peer_digest
                    || !event.allowed
            }) {
                return RecoveryDecision::Contradiction;
            }

            if matching_connection.len() == 1 {
                RecoveryDecision::ResponseAndSocketCorroborated
            } else {
                // No connect event may simply mean this request reused a
                // previously established HTTP/1 or HTTP/2 connection.
                RecoveryDecision::ResponseObservedWithoutIndependentSocketCorroboration
            }
        }
        KernelObserver::EventLoss | KernelObserver::CoverageGap => {
            RecoveryDecision::ResponseObservedWithoutIndependentSocketCorroboration
        }
    }
}

fn decide_recovery(
    attempt: Attempt,
    now_ns: u64,
    contract: Option<IdempotencyContract>,
    remote: Option<RemoteProof>,
    transport: TransportObservation,
    observer: &KernelObserver,
) -> RecoveryDecision {
    if let Some(proof) = remote {
        let Some(contract) = contract else {
            return RecoveryDecision::HoldUnknown;
        };
        if !contract.matches(attempt, now_ns) {
            return RecoveryDecision::HoldUnknown;
        }
        if !proof.matches(attempt, contract, now_ns) {
            return RecoveryDecision::Contradiction;
        }

        let no_write_claim = matches!(
            transport,
            TransportObservation::ProvenNoWrite { .. }
        );
        if no_write_claim && proof.state == RemoteState::Applied {
            return RecoveryDecision::Contradiction;
        }

        if let TransportObservation::ResponseReceived { .. } = transport {
            if response_socket_assessment(attempt, transport, observer)
                == RecoveryDecision::Contradiction
            {
                return RecoveryDecision::Contradiction;
            }
        }

        return match (attempt.phase, proof.state) {
            (JournalPhase::Prepared | JournalPhase::Authorized, RemoteState::Applied) => {
                RecoveryDecision::Contradiction
            }
            (JournalPhase::Prepared | JournalPhase::Authorized, RemoteState::DefinitelyNotApplied) => {
                RecoveryDecision::SafeToAuthorizeFreshAttempt
            }
            (JournalPhase::DispatchMarked, RemoteState::Applied) => {
                RecoveryDecision::ReconciledApplied
            }
            (_, RemoteState::DefinitelyNotApplied) => {
                RecoveryDecision::SafeToAuthorizeFreshAttemptWithSameKey
            }
            (_, RemoteState::Unknown) => RecoveryDecision::HoldUnknown,
        };
    }

    if let TransportObservation::ResponseReceived { .. } = transport {
        // A response is evidence that transport activity occurred. It does
        // not by itself establish the business effect or authorize a retry.
        return response_socket_assessment(attempt, transport, observer);
    }

    if let TransportObservation::ProvenNoWrite {
        attempt_id,
        request_digest,
        adapter_generation,
        proof_digest,
    } = transport
    {
        if attempt_id != attempt.attempt_id
            || request_digest != attempt.request_digest
            || adapter_generation == 0
            || proof_digest == 0
        {
            return RecoveryDecision::Contradiction;
        }
        return RecoveryDecision::SafeToAuthorizeFreshAttempt;
    }

    match attempt.phase {
        JournalPhase::Prepared | JournalPhase::Authorized if attempt.journal_durable => {
            // Safe only if production enforces write-ahead ordering:
            // durable DispatchMarked must precede entry into the transport.
            RecoveryDecision::SafeToAuthorizeFreshAttempt
        }
        JournalPhase::Prepared | JournalPhase::Authorized => RecoveryDecision::HoldUnknown,
        JournalPhase::DispatchMarked => match contract {
            Some(contract) if contract.matches(attempt, now_ns) => {
                // Query only. Never infer absence from a timeout or cache miss.
                RecoveryDecision::QueryRemoteIdempotencyState
            }
            _ => RecoveryDecision::HoldUnknown,
        },
    }
}

fn attempt(phase: JournalPhase, durable: bool) -> Attempt {
    Attempt {
        attempt_id: 41,
        workload_id: 7,
        origin_digest: 101,
        principal_digest: 202,
        request_digest: 303,
        idempotency_key: Some(404),
        phase,
        journal_durable: durable,
    }
}

fn contract() -> IdempotencyContract {
    IdempotencyContract {
        origin_digest: 101,
        principal_digest: 202,
        request_digest: 303,
        key_digest: 404,
        deduplicates_concurrent_requests: true,
        lookup_is_linearizable: true,
        retention_until_ns: 10_000,
    }
}

fn proof(state: RemoteState) -> RemoteProof {
    RemoteProof {
        attempt_id: 41,
        origin_digest: 101,
        principal_digest: 202,
        request_digest: 303,
        idempotency_key: Some(404),
        state,
        linearizable: true,
        server_revision: 9,
        proof_digest: 505,
    }
}

fn complete_empty_observer() -> KernelObserver {
    KernelObserver::Complete {
        generation: 4,
        events: Vec::new(),
    }
}

fn response() -> TransportObservation {
    TransportObservation::ResponseReceived {
        connection_id: 77,
        peer_digest: 88,
        response_digest: 99,
    }
}

#[test]
fn durable_pre_dispatch_preparation_allows_fresh_attempt() {
    assert_eq!(
        decide_recovery(
            attempt(JournalPhase::Prepared, true),
            100,
            None,
            None,
            TransportObservation::Unknown,
            &complete_empty_observer(),
        ),
        RecoveryDecision::SafeToAuthorizeFreshAttempt
    );
}

#[test]
fn durable_authorization_without_dispatch_marker_allows_fresh_attempt() {
    assert_eq!(
        decide_recovery(
            attempt(JournalPhase::Authorized, true),
            100,
            None,
            None,
            TransportObservation::Unknown,
            &complete_empty_observer(),
        ),
        RecoveryDecision::SafeToAuthorizeFreshAttempt
    );
}

#[test]
fn non_durable_pre_dispatch_journal_is_not_a_no_dispatch_proof() {
    assert_eq!(
        decide_recovery(
            attempt(JournalPhase::Authorized, false),
            100,
            None,
            None,
            TransportObservation::Unknown,
            &complete_empty_observer(),
        ),
        RecoveryDecision::HoldUnknown
    );
}

#[test]
fn dispatch_marker_without_remote_reconciliation_holds_unknown() {
    assert_eq!(
        decide_recovery(
            attempt(JournalPhase::DispatchMarked, true),
            100,
            None,
            None,
            TransportObservation::Unknown,
            &complete_empty_observer(),
        ),
        RecoveryDecision::HoldUnknown
    );
}

#[test]
fn complete_observer_with_no_connect_event_does_not_prove_no_http_dispatch() {
    let decision = decide_recovery(
        attempt(JournalPhase::DispatchMarked, true),
        100,
        Some(contract()),
        None,
        TransportObservation::Unknown,
        &complete_empty_observer(),
    );
    assert_eq!(decision, RecoveryDecision::QueryRemoteIdempotencyState);
}

#[test]
fn observer_event_loss_never_authorizes_a_retry() {
    let decision = decide_recovery(
        attempt(JournalPhase::DispatchMarked, true),
        100,
        None,
        None,
        TransportObservation::Unknown,
        &KernelObserver::EventLoss,
    );
    assert_eq!(decision, RecoveryDecision::HoldUnknown);
}

#[test]
fn observer_coverage_gap_never_authorizes_a_retry() {
    let decision = decide_recovery(
        attempt(JournalPhase::DispatchMarked, true),
        100,
        None,
        None,
        TransportObservation::Unknown,
        &KernelObserver::CoverageGap,
    );
    assert_eq!(decision, RecoveryDecision::HoldUnknown);
}

#[test]
fn exact_no_write_transport_proof_allows_a_fresh_attempt() {
    let decision = decide_recovery(
        attempt(JournalPhase::DispatchMarked, true),
        100,
        None,
        None,
        TransportObservation::ProvenNoWrite {
            attempt_id: 41,
            request_digest: 303,
            adapter_generation: 6,
            proof_digest: 707,
        },
        &complete_empty_observer(),
    );
    assert_eq!(decision, RecoveryDecision::SafeToAuthorizeFreshAttempt);
}

#[test]
fn no_write_proof_for_another_attempt_is_a_contradiction() {
    let decision = decide_recovery(
        attempt(JournalPhase::DispatchMarked, true),
        100,
        None,
        None,
        TransportObservation::ProvenNoWrite {
            attempt_id: 42,
            request_digest: 303,
            adapter_generation: 6,
            proof_digest: 707,
        },
        &complete_empty_observer(),
    );
    assert_eq!(decision, RecoveryDecision::Contradiction);
}

#[test]
fn dispatch_with_valid_contract_requires_non_mutating_remote_lookup() {
    assert_eq!(
        decide_recovery(
            attempt(JournalPhase::DispatchMarked, true),
            100,
            Some(contract()),
            None,
            TransportObservation::Unknown,
            &complete_empty_observer(),
        ),
        RecoveryDecision::QueryRemoteIdempotencyState
    );
}

#[test]
fn linearizable_applied_proof_reconciles_original_attempt() {
    assert_eq!(
        decide_recovery(
            attempt(JournalPhase::DispatchMarked, true),
            100,
            Some(contract()),
            Some(proof(RemoteState::Applied)),
            TransportObservation::Unknown,
            &complete_empty_observer(),
        ),
        RecoveryDecision::ReconciledApplied
    );
}

#[test]
fn linearizable_absence_proof_allows_same_logical_key_with_fresh_attempt() {
    assert_eq!(
        decide_recovery(
            attempt(JournalPhase::DispatchMarked, true),
            100,
            Some(contract()),
            Some(proof(RemoteState::DefinitelyNotApplied)),
            TransportObservation::Unknown,
            &complete_empty_observer(),
        ),
        RecoveryDecision::SafeToAuthorizeFreshAttemptWithSameKey
    );
}

#[test]
fn inconclusive_remote_lookup_remains_unknown() {
    assert_eq!(
        decide_recovery(
            attempt(JournalPhase::DispatchMarked, true),
            100,
            Some(contract()),
            Some(proof(RemoteState::Unknown)),
            TransportObservation::Unknown,
            &complete_empty_observer(),
        ),
        RecoveryDecision::HoldUnknown
    );
}

#[test]
fn eventually_consistent_remote_proof_cannot_authorize_replay() {
    let mut weak = proof(RemoteState::DefinitelyNotApplied);
    weak.linearizable = false;
    assert_eq!(
        decide_recovery(
            attempt(JournalPhase::DispatchMarked, true),
            100,
            Some(contract()),
            Some(weak),
            TransportObservation::Unknown,
            &complete_empty_observer(),
        ),
        RecoveryDecision::Contradiction
    );
}

#[test]
fn expired_idempotency_contract_cannot_authorize_lookup_or_replay() {
    assert_eq!(
        decide_recovery(
            attempt(JournalPhase::DispatchMarked, true),
            10_000,
            Some(contract()),
            Some(proof(RemoteState::DefinitelyNotApplied)),
            TransportObservation::Unknown,
            &complete_empty_observer(),
        ),
        RecoveryDecision::HoldUnknown
    );
}

#[test]
fn changed_request_digest_cannot_reuse_previous_idempotency_contract() {
    let mut changed = attempt(JournalPhase::DispatchMarked, true);
    changed.request_digest += 1;
    assert_eq!(
        decide_recovery(
            changed,
            100,
            Some(contract()),
            None,
            TransportObservation::Unknown,
            &complete_empty_observer(),
        ),
        RecoveryDecision::HoldUnknown
    );
}

#[test]
fn changed_origin_cannot_reuse_previous_idempotency_contract() {
    let mut changed = attempt(JournalPhase::DispatchMarked, true);
    changed.origin_digest += 1;
    assert_eq!(
        decide_recovery(
            changed,
            100,
            Some(contract()),
            None,
            TransportObservation::Unknown,
            &complete_empty_observer(),
        ),
        RecoveryDecision::HoldUnknown
    );
}

#[test]
fn changed_principal_cannot_reuse_previous_idempotency_contract() {
    let mut changed = attempt(JournalPhase::DispatchMarked, true);
    changed.principal_digest += 1;
    assert_eq!(
        decide_recovery(
            changed,
            100,
            Some(contract()),
            None,
            TransportObservation::Unknown,
            &complete_empty_observer(),
        ),
        RecoveryDecision::HoldUnknown
    );
}

#[test]
fn mismatched_remote_proof_attempt_identity_is_a_contradiction() {
    let mut wrong = proof(RemoteState::Applied);
    wrong.attempt_id += 1;
    assert_eq!(
        decide_recovery(
            attempt(JournalPhase::DispatchMarked, true),
            100,
            Some(contract()),
            Some(wrong),
            TransportObservation::Unknown,
            &complete_empty_observer(),
        ),
        RecoveryDecision::Contradiction
    );
}

#[test]
fn application_response_and_matching_kernel_socket_are_transport_corroborated() {
    let observer = KernelObserver::Complete {
        generation: 4,
        events: vec![KernelConnectEvent {
            generation: 4,
            connection_id: 77,
            workload_id: 7,
            peer_digest: 88,
            allowed: true,
        }],
    };
    assert_eq!(
        response_socket_assessment(
            attempt(JournalPhase::DispatchMarked, true),
            response(),
            &observer,
        ),
        RecoveryDecision::ResponseAndSocketCorroborated
    );
}

#[test]
fn response_over_reused_connection_without_new_connect_event_is_not_no_dispatch() {
    assert_eq!(
        response_socket_assessment(
            attempt(JournalPhase::DispatchMarked, true),
            response(),
            &complete_empty_observer(),
        ),
        RecoveryDecision::ResponseObservedWithoutIndependentSocketCorroboration
    );
}

#[test]
fn response_peer_mismatch_is_a_contradiction() {
    let observer = KernelObserver::Complete {
        generation: 4,
        events: vec![KernelConnectEvent {
            generation: 4,
            connection_id: 77,
            workload_id: 7,
            peer_digest: 999,
            allowed: true,
        }],
    };
    assert_eq!(
        response_socket_assessment(
            attempt(JournalPhase::DispatchMarked, true),
            response(),
            &observer,
        ),
        RecoveryDecision::Contradiction
    );
}

#[test]
fn response_on_another_workload_is_a_contradiction() {
    let observer = KernelObserver::Complete {
        generation: 4,
        events: vec![KernelConnectEvent {
            generation: 4,
            connection_id: 77,
            workload_id: 8,
            peer_digest: 88,
            allowed: true,
        }],
    };
    assert_eq!(
        response_socket_assessment(
            attempt(JournalPhase::DispatchMarked, true),
            response(),
            &observer,
        ),
        RecoveryDecision::Contradiction
    );
}

#[test]
fn response_with_observer_loss_is_not_independently_socket_corroborated() {
    assert_eq!(
        response_socket_assessment(
            attempt(JournalPhase::DispatchMarked, true),
            response(),
            &KernelObserver::EventLoss,
        ),
        RecoveryDecision::ResponseObservedWithoutIndependentSocketCorroboration
    );
}

#[test]
fn transport_response_without_remote_effect_proof_is_not_reconciled_applied() {
    let decision = decide_recovery(
        attempt(JournalPhase::DispatchMarked, true),
        100,
        Some(contract()),
        None,
        response(),
        &complete_empty_observer(),
    );
    assert_eq!(
        decision,
        RecoveryDecision::ResponseObservedWithoutIndependentSocketCorroboration
    );
    assert_ne!(decision, RecoveryDecision::ReconciledApplied);
}


#[test]
fn no_write_proof_without_adapter_generation_is_rejected() {
    let decision = decide_recovery(
        attempt(JournalPhase::DispatchMarked, true),
        100,
        None,
        None,
        TransportObservation::ProvenNoWrite {
            attempt_id: 41,
            request_digest: 303,
            adapter_generation: 0,
            proof_digest: 707,
        },
        &complete_empty_observer(),
    );
    assert_eq!(decision, RecoveryDecision::Contradiction);
}

#[test]
fn idempotency_contract_without_concurrent_deduplication_is_not_usable() {
    let mut weak = contract();
    weak.deduplicates_concurrent_requests = false;
    assert_eq!(
        decide_recovery(
            attempt(JournalPhase::DispatchMarked, true),
            100,
            Some(weak),
            None,
            TransportObservation::Unknown,
            &complete_empty_observer(),
        ),
        RecoveryDecision::HoldUnknown
    );
}

#[test]
fn changed_request_digest_in_remote_absence_proof_is_a_contradiction() {
    let mut changed = proof(RemoteState::DefinitelyNotApplied);
    changed.request_digest += 1;
    assert_eq!(
        decide_recovery(
            attempt(JournalPhase::DispatchMarked, true),
            100,
            Some(contract()),
            Some(changed),
            TransportObservation::Unknown,
            &complete_empty_observer(),
        ),
        RecoveryDecision::Contradiction
    );
}

#[test]
fn missing_remote_proof_digest_is_a_contradiction() {
    let mut invalid = proof(RemoteState::Applied);
    invalid.proof_digest = 0;
    assert_eq!(
        decide_recovery(
            attempt(JournalPhase::DispatchMarked, true),
            100,
            Some(contract()),
            Some(invalid),
            TransportObservation::Unknown,
            &complete_empty_observer(),
        ),
        RecoveryDecision::Contradiction
    );
}

#[test]
fn duplicate_connect_events_do_not_become_unique_socket_corroboration() {
    let event = KernelConnectEvent {
        generation: 4,
        connection_id: 77,
        workload_id: 7,
        peer_digest: 88,
        allowed: true,
    };
    let observer = KernelObserver::Complete {
        generation: 4,
        events: vec![event, event],
    };
    assert_eq!(
        response_socket_assessment(
            attempt(JournalPhase::DispatchMarked, true),
            response(),
            &observer,
        ),
        RecoveryDecision::ResponseObservedWithoutIndependentSocketCorroboration
    );
}


#[test]
fn remote_applied_proof_conflicts_with_transport_no_write_proof() {
    let decision = decide_recovery(
        attempt(JournalPhase::DispatchMarked, true),
        100,
        Some(contract()),
        Some(proof(RemoteState::Applied)),
        TransportObservation::ProvenNoWrite {
            attempt_id: 41,
            request_digest: 303,
            adapter_generation: 6,
            proof_digest: 707,
        },
        &complete_empty_observer(),
    );
    assert_eq!(decision, RecoveryDecision::Contradiction);
}

#[test]
fn remote_applied_proof_conflicts_with_pre_dispatch_journal_state() {
    let decision = decide_recovery(
        attempt(JournalPhase::Authorized, true),
        100,
        Some(contract()),
        Some(proof(RemoteState::Applied)),
        TransportObservation::Unknown,
        &complete_empty_observer(),
    );
    assert_eq!(decision, RecoveryDecision::Contradiction);
}

#[test]
fn authoritative_absence_can_resolve_a_response_without_repeating_the_effect() {
    let decision = decide_recovery(
        attempt(JournalPhase::DispatchMarked, true),
        100,
        Some(contract()),
        Some(proof(RemoteState::DefinitelyNotApplied)),
        response(),
        &complete_empty_observer(),
    );
    assert_eq!(
        decision,
        RecoveryDecision::SafeToAuthorizeFreshAttemptWithSameKey
    );
}
