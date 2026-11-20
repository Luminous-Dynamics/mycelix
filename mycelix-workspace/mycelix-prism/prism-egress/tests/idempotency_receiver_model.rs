//! Test-only model of a receiver-side idempotency contract.
//! The commit() operation models one atomic database transaction that commits
//! both the business effect and the durable idempotency result. It is not a
//! real database, network server, consensus protocol or failover test.
use std::collections::HashMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct KeyScope {
    workload_id: u64,
    principal_digest: u64,
    origin_digest: u64,
    key_digest: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ReceiverRequest {
    logical_operation_id: u64,
    attempt_id: u64,
    workload_id: u64,
    principal_digest: u64,
    origin_digest: u64,
    method: &'static str,
    request_digest: u64,
    idempotency_key_digest: u64,
}

impl ReceiverRequest {
    fn key(self) -> KeyScope {
        KeyScope {
            workload_id: self.workload_id,
            principal_digest: self.principal_digest,
            origin_digest: self.origin_digest,
            key_digest: self.idempotency_key_digest,
        }
    }

    fn same_operation(self, other: Self) -> bool {
        self.logical_operation_id == other.logical_operation_id
            && self.workload_id == other.workload_id
            && self.principal_digest == other.principal_digest
            && self.origin_digest == other.origin_digest
            && self.method == other.method
            && self.request_digest == other.request_digest
            && self.idempotency_key_digest == other.idempotency_key_digest
    }
}

#[derive(Debug, PartialEq, Eq)]
struct OwnerToken {
    key: KeyScope,
    logical_operation_id: u64,
    attempt_id: u64,
    request_digest: u64,
    nonce: u64,
}

#[derive(Debug, PartialEq, Eq)]
enum BeginDecision {
    Owner(OwnerToken),
    Completed {
        result_digest: u64,
        revision: u64,
    },
    ConflictInProgress,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ReceiverError {
    InvalidRequest,
    KeyExpired,
    KeyReuseMismatch,
    InvalidToken,
    UnknownReservation,
    InvalidResult,
    CounterOverflow,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct InFlight {
    request: ReceiverRequest,
    nonce: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Completed {
    request: ReceiverRequest,
    result_digest: u64,
    expires_at_ns: u64,
    revision: u64,
}

#[derive(Clone, Debug, Default)]
struct Receiver {
    in_flight: HashMap<KeyScope, InFlight>,
    completed: HashMap<KeyScope, Completed>,
    applied_effects: HashMap<u64, u64>,
    next_nonce: u64,
    revision: u64,
    key_retention_ns: u64,
}

impl Receiver {
    fn new() -> Self {
        Self {
            next_nonce: 1,
            key_retention_ns: 10_000,
            ..Self::default()
        }
    }

    fn with_retention(key_retention_ns: u64) -> Result<Self, ReceiverError> {
        if key_retention_ns == 0 {
            return Err(ReceiverError::InvalidRequest);
        }
        Ok(Self {
            next_nonce: 1,
            key_retention_ns,
            ..Self::default()
        })
    }

    fn begin(
        &mut self,
        request: ReceiverRequest,
        now_ns: u64,
    ) -> Result<BeginDecision, ReceiverError> {
        if request.logical_operation_id == 0
            || request.attempt_id == 0
            || request.workload_id == 0
            || request.principal_digest == 0
            || request.origin_digest == 0
            || request.request_digest == 0
            || request.idempotency_key_digest == 0
            || request.method.is_empty()
        {
            return Err(ReceiverError::InvalidRequest);
        }

        let expires_at_ns = now_ns
            .checked_add(self.key_retention_ns)
            .ok_or(ReceiverError::CounterOverflow)?;
        let key = request.key();
        if let Some(completed) = self.completed.get(&key) {
            if now_ns >= completed.expires_at_ns {
                return Err(ReceiverError::KeyExpired);
            }
            if !completed.request.same_operation(request) {
                return Err(ReceiverError::KeyReuseMismatch);
            }
            return Ok(BeginDecision::Completed {
                result_digest: completed.result_digest,
                revision: completed.revision,
            });
        }

        if let Some(in_flight) = self.in_flight.get(&key) {
            if !in_flight.request.same_operation(request) {
                return Err(ReceiverError::KeyReuseMismatch);
            }
            // This mirrors the IETF draft's conflict behavior for a duplicate
            // arriving before the original request has completed.
            return Ok(BeginDecision::ConflictInProgress);
        }

        let nonce = self.next_nonce;
        self.next_nonce = self
            .next_nonce
            .checked_add(1)
            .ok_or(ReceiverError::CounterOverflow)?;

        self.in_flight.insert(key, InFlight { request, nonce });
        Ok(BeginDecision::Owner(OwnerToken {
            key,
            logical_operation_id: request.logical_operation_id,
            attempt_id: request.attempt_id,
            request_digest: request.request_digest,
            nonce,
        }))
    }

    /// Atomically persists the business effect and its replayable idempotency
    /// result. No partial effect is visible in this reference model.
    fn commit(
        &mut self,
        token: &OwnerToken,
        result_digest: u64,
    ) -> Result<u64, ReceiverError> {
        if result_digest == 0 {
            return Err(ReceiverError::InvalidResult);
        }

        let in_flight = self
            .in_flight
            .get(&token.key)
            .copied()
            .ok_or(ReceiverError::UnknownReservation)?;

        if in_flight.nonce != token.nonce
            || in_flight.request.logical_operation_id != token.logical_operation_id
            || in_flight.request.attempt_id != token.attempt_id
            || in_flight.request.request_digest != token.request_digest
        {
            return Err(ReceiverError::InvalidToken);
        }

        let next_effect_count = self
            .applied_effects
            .get(&token.logical_operation_id)
            .copied()
            .unwrap_or(0)
            .checked_add(1)
            .ok_or(ReceiverError::CounterOverflow)?;
        let next_revision = self
            .revision
            .checked_add(1)
            .ok_or(ReceiverError::CounterOverflow)?;

        // Treat these mutations as one transaction in the model. A real
        // receiver must commit the effect and idempotency row atomically.
        self.in_flight.remove(&token.key);
        self.completed.insert(
            token.key,
            Completed {
                request: in_flight.request,
                result_digest,
                expires_at_ns: expires_at_ns,
                revision: next_revision,
            },
        );
        self.applied_effects
            .insert(token.logical_operation_id, next_effect_count);
        self.revision = next_revision;
        Ok(next_revision)
    }

    fn abort(&mut self, token: &OwnerToken) -> Result<(), ReceiverError> {
        let in_flight = self
            .in_flight
            .get(&token.key)
            .copied()
            .ok_or(ReceiverError::UnknownReservation)?;

        if in_flight.nonce != token.nonce
            || in_flight.request.logical_operation_id != token.logical_operation_id
            || in_flight.request.attempt_id != token.attempt_id
            || in_flight.request.request_digest != token.request_digest
        {
            return Err(ReceiverError::InvalidToken);
        }

        self.in_flight.remove(&token.key);
        Ok(())
    }

    /// A crash rolls back uncommitted in-flight reservations. Committed
    /// results and applied effects survive because they are one durable unit.
    fn recover_after_crash(&self) -> Self {
        Self {
            in_flight: HashMap::new(),
            completed: self.completed.clone(),
            applied_effects: self.applied_effects.clone(),
            next_nonce: self.next_nonce,
            revision: self.revision,
            key_retention_ns: self.key_retention_ns,
        }
    }

    /// Model a failover that preserves the idempotency table and side effects.
    fn failover_durable(&self) -> Self {
        self.recover_after_crash()
    }

    /// Deliberately unsafe control: the business effect survives but the
    /// idempotency table is lost on failover.
    fn failover_losing_dedup_state(&self) -> Self {
        Self {
            in_flight: HashMap::new(),
            completed: HashMap::new(),
            applied_effects: self.applied_effects.clone(),
            next_nonce: self.next_nonce,
            revision: self.revision,
            key_retention_ns: self.key_retention_ns,
        }
    }

    fn effect_count(&self, logical_operation_id: u64) -> u64 {
        self.applied_effects
            .get(&logical_operation_id)
            .copied()
            .unwrap_or(0)
    }

    fn completed_count(&self) -> usize {
        self.completed.len()
    }

    fn in_flight_count(&self) -> usize {
        self.in_flight.len()
    }
}

fn request() -> ReceiverRequest {
    ReceiverRequest {
        logical_operation_id: 10,
        attempt_id: 11,
        workload_id: 7,
        principal_digest: 101,
        origin_digest: 202,
        method: "POST",
        request_digest: 303,
        idempotency_key_digest: 404,
    }
}

fn acquire(receiver: &mut Receiver, request: ReceiverRequest, now_ns: u64) -> OwnerToken {
    match receiver.begin(request, now_ns).unwrap() {
        BeginDecision::Owner(token) => token,
        other => panic!("expected owner reservation, got {other:?}"),
    }
}

#[test]
fn first_request_atomically_commits_effect_and_idempotency_result() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    assert_eq!(receiver.effect_count(10), 0);
    assert_eq!(receiver.commit(&token, 505), Ok(1));
    assert_eq!(receiver.effect_count(10), 1);
    assert_eq!(receiver.completed_count(), 1);
    assert_eq!(receiver.in_flight_count(), 0);
}

#[test]
fn completed_duplicate_returns_cached_result_without_second_effect() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    receiver.commit(&token, 505).unwrap();
    let duplicate = receiver.begin(request(), 101).unwrap();
    assert_eq!(
        duplicate,
        BeginDecision::Completed {
            result_digest: 505,
            revision: 1
        }
    );
    assert_eq!(receiver.effect_count(10), 1);
}

#[test]
fn concurrent_same_key_duplicate_is_rejected_while_original_is_in_progress() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    assert_eq!(
        receiver.begin(request(), 101),
        Ok(BeginDecision::ConflictInProgress)
    );
    assert_eq!(receiver.effect_count(10), 0);
    receiver.commit(&token, 505).unwrap();
    assert_eq!(receiver.effect_count(10), 1);
}

#[test]
fn same_key_with_different_request_fingerprint_is_rejected() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    let mut changed = request();
    changed.request_digest += 1;
    assert_eq!(
        receiver.begin(changed, 101).err(),
        Some(ReceiverError::KeyReuseMismatch)
    );
    receiver.commit(&token, 505).unwrap();
    assert_eq!(receiver.effect_count(10), 1);
}

#[test]
fn crash_before_atomic_commit_discards_reservation_without_effect() {
    let mut receiver = Receiver::new();
    let _token = acquire(&mut receiver, request(), 100);
    let mut recovered = receiver.recover_after_crash();
    assert_eq!(recovered.in_flight_count(), 0);
    assert_eq!(recovered.effect_count(10), 0);
    assert_eq!(recovered.completed_count(), 0);
    let retry = acquire(&mut recovered, request(), 101);
    recovered.commit(&retry, 505).unwrap();
    assert_eq!(recovered.effect_count(10), 1);
}
#[test]
fn crash_after_commit_before_response_replays_completion_without_duplicate_effect() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    receiver.commit(&token, 505).unwrap();
    let recovered = receiver.recover_after_crash();
    assert_eq!(
        recovered.begin(request(), 101),
        Ok(BeginDecision::Completed {
            result_digest: 505,
            revision: 1
        })
    );
    assert_eq!(recovered.effect_count(10), 1);
}

#[test]
fn durable_failover_preserves_deduplication_and_effect() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    receiver.commit(&token, 505).unwrap();
    let replica = receiver.failover_durable();
    assert_eq!(
        replica.begin(request(), 101),
        Ok(BeginDecision::Completed {
            result_digest: 505,
            revision: 1
        })
    );
    assert_eq!(replica.effect_count(10), 1);
}

#[test]
fn losing_idempotency_state_on_failover_is_an_explicit_unsafe_control() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    receiver.commit(&token, 505).unwrap();
    let mut unsafe_replica = receiver.failover_losing_dedup_state();
    let duplicate_token = acquire(&mut unsafe_replica, request(), 101);
    unsafe_replica.commit(&duplicate_token, 606).unwrap();
    assert_eq!(unsafe_replica.effect_count(10), 2);
}

#[test]
fn abort_releases_reservation_without_applying_effect() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    receiver.abort(&token).unwrap();
    assert_eq!(receiver.in_flight_count(), 0);
    assert_eq!(receiver.effect_count(10), 0);
    let retry = acquire(&mut receiver, request(), 101);
    receiver.commit(&retry, 505).unwrap();
    assert_eq!(receiver.effect_count(10), 1);
}

#[test]
fn commit_token_for_another_attempt_is_rejected() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    let mut changed = token;
    changed.attempt_id += 1;
    assert_eq!(
        receiver.commit(&changed, 505),
        Err(ReceiverError::InvalidToken)
    );
    assert_eq!(receiver.effect_count(10), 0);
}

#[test]
fn commit_token_for_another_request_digest_is_rejected() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    let mut changed = token;
    changed.request_digest += 1;
    assert_eq!(
        receiver.commit(&changed, 505),
        Err(ReceiverError::InvalidToken)
    );
    assert_eq!(receiver.effect_count(10), 0);
}

#[test]
fn a_committed_owner_token_cannot_be_used_twice() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    receiver.commit(&token, 505).unwrap();
    assert_eq!(
        receiver.commit(&token, 606),
        Err(ReceiverError::UnknownReservation)
    );
    assert_eq!(receiver.effect_count(10), 1);
}
#[test]
fn an_aborted_owner_token_cannot_be_used_to_commit_later() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    receiver.abort(&token).unwrap();
    assert_eq!(
        receiver.commit(&token, 505),
        Err(ReceiverError::UnknownReservation)
    );
    assert_eq!(receiver.effect_count(10), 0);
}
#[test]
fn duplicate_at_exact_key_expiry_is_rejected() {
    let mut receiver = Receiver::with_retention(100).unwrap();
    let token = acquire(&mut receiver, request(), 100);
    receiver.commit(&token, 505).unwrap();
    assert_eq!(
        receiver.begin(request(), 200),
        Err(ReceiverError::KeyExpired)
    );
    assert_eq!(receiver.effect_count(10), 1);
}

#[test]
fn same_raw_key_in_another_principal_namespace_is_independent() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    receiver.commit(&token, 505).unwrap();
    let mut other_principal = request();
    other_principal.logical_operation_id = 20;
    other_principal.attempt_id = 21;
    other_principal.principal_digest = 999;
    let other = acquire(&mut receiver, other_principal, 101);
    receiver.commit(&other, 606).unwrap();
    assert_eq!(receiver.effect_count(10), 1);
    assert_eq!(receiver.effect_count(20), 1);
}

#[test]
fn same_raw_key_in_another_workload_namespace_is_independent() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    receiver.commit(&token, 505).unwrap();
    let mut other_workload = request();
    other_workload.logical_operation_id = 20;
    other_workload.attempt_id = 21;
    other_workload.workload_id = 8;
    let other = acquire(&mut receiver, other_workload, 101);
    receiver.commit(&other, 606).unwrap();
    assert_eq!(receiver.effect_count(10), 1);
    assert_eq!(receiver.effect_count(20), 1);
}

#[test]
fn same_raw_key_at_another_origin_namespace_is_independent() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    receiver.commit(&token, 505).unwrap();
    let mut other_origin = request();
    other_origin.logical_operation_id = 20;
    other_origin.attempt_id = 21;
    other_origin.origin_digest = 999;
    let other = acquire(&mut receiver, other_origin, 101);
    receiver.commit(&other, 606).unwrap();
    assert_eq!(receiver.effect_count(10), 1);
    assert_eq!(receiver.effect_count(20), 1);
}

#[test]
fn same_key_with_different_http_method_is_rejected() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    let mut changed = request();
    changed.method = "PATCH";
    assert_eq!(
        receiver.begin(changed, 101).err(),
        Some(ReceiverError::KeyReuseMismatch)
    );
    receiver.commit(&token, 505).unwrap();
}

#[test]
fn same_key_cannot_be_reassigned_to_another_logical_operation() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    let mut changed = request();
    changed.logical_operation_id += 1;
    changed.attempt_id += 1;
    assert_eq!(
        receiver.begin(changed, 101).err(),
        Some(ReceiverError::KeyReuseMismatch)
    );
    receiver.commit(&token, 505).unwrap();
}

#[test]
fn retry_with_new_attempt_id_returns_the_original_completion() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    receiver.commit(&token, 505).unwrap();
    let mut retry = request();
    retry.attempt_id = 99;
    assert_eq!(
        receiver.begin(retry, 101),
        Ok(BeginDecision::Completed {
            result_digest: 505,
            revision: 1
        })
    );
    assert_eq!(receiver.effect_count(10), 1);
}

#[test]
fn many_concurrent_duplicates_do_not_apply_effect_before_owner_commit() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    for _ in 0..128 {
        assert_eq!(
            receiver.begin(request(), 101),
            Ok(BeginDecision::ConflictInProgress)
        );
    }
    assert_eq!(receiver.effect_count(10), 0);
    receiver.commit(&token, 505).unwrap();
    assert_eq!(receiver.effect_count(10), 1);
    assert_eq!(receiver.completed_count(), 1);
}

#[test]
fn generated_sequential_replays_always_return_one_cached_result() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    receiver.commit(&token, 505).unwrap();
    for attempt_id in 12..=139 {
        let mut retry = request();
        retry.attempt_id = attempt_id;
        assert_eq!(
            receiver.begin(retry, 200),
            Ok(BeginDecision::Completed {
                result_digest: 505,
                revision: 1
            })
        );
    }
    assert_eq!(receiver.effect_count(10), 1);
}

#[test]
fn zero_result_digest_does_not_commit_or_release_reservation() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    assert_eq!(
        receiver.commit(&token, 0),
        Err(ReceiverError::InvalidResult)
    );
    assert_eq!(receiver.effect_count(10), 0);
    assert_eq!(receiver.in_flight_count(), 1);
    receiver.commit(&token, 505).unwrap();
    assert_eq!(receiver.effect_count(10), 1);
}
#[test]
fn forged_owner_nonce_cannot_commit_effect() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    let mut forged = token;
    forged.nonce += 1;
    assert_eq!(
        receiver.commit(&forged, 505),
        Err(ReceiverError::InvalidToken)
    );
    assert_eq!(receiver.effect_count(10), 0);
}

#[test]
fn crash_clears_uncommitted_in_flight_owner_without_effect() {
    let mut receiver = Receiver::new();
    let _token = acquire(&mut receiver, request(), 100);
    let recovered = receiver.recover_after_crash();
    assert_eq!(recovered.in_flight_count(), 0);
    assert_eq!(recovered.effect_count(10), 0);
    assert_eq!(recovered.completed_count(), 0);
}

#[test]
fn failover_preserves_the_cached_result_digest_and_revision() {
    let mut receiver = Receiver::new();
    let token = acquire(&mut receiver, request(), 100);
    receiver.commit(&token, 0xabc).unwrap();
    let replica = receiver.failover_durable();
    assert_eq!(
        replica.begin(request(), 101),
        Ok(BeginDecision::Completed {
            result_digest: 0xabc,
            revision: 1
        })
    );
}

#[test]
fn different_key_same_payload_is_a_distinct_logical_operation() {
    let mut receiver = Receiver::new();
    let first = acquire(&mut receiver, request(), 100);
    receiver.commit(&first, 505).unwrap();
    let mut second_request = request();
    second_request.logical_operation_id = 20;
    second_request.attempt_id = 21;
    second_request.idempotency_key_digest = 405;
    let second = acquire(&mut receiver, second_request, 101);
    receiver.commit(&second, 606).unwrap();
    assert_eq!(receiver.effect_count(10), 1);
    assert_eq!(receiver.effect_count(20), 1);
}

#[test]
fn the_receiver_rejects_zero_retention_configuration() {
    assert_eq!(
        Receiver::with_retention(0).err(),
        Some(ReceiverError::InvalidRequest)
    );
}

#[test]
fn expired_key_is_not_silently_extended_by_a_later_duplicate() {
    let mut receiver = Receiver::with_retention(100).unwrap();
    let token = acquire(&mut receiver, request(), 100);
    receiver.commit(&token, 505).unwrap();
    assert_eq!(
        receiver.begin(request(), 200),
        Err(ReceiverError::KeyExpired)
    );
    assert_eq!(receiver.effect_count(10), 1);
}


#[test]
fn receiver_owns_key_retention_instead_of_trusting_the_request() {
    let mut receiver = Receiver::with_retention(100).unwrap();
    let token = acquire(&mut receiver, request(), 100);
    receiver.commit(&token, 505).unwrap();
    assert_eq!(
        receiver.begin(request(), 199),
        Ok(BeginDecision::Completed {
            result_digest: 505,
            revision: 1
        })
    );
    assert_eq!(
        receiver.begin(request(), 200),
        Err(ReceiverError::KeyExpired)
    );
}

#[test]
fn key_expiry_deadline_overflow_fails_closed() {
    let mut receiver = Receiver::new();
    assert_eq!(
        receiver.begin(request(), u64::MAX),
        Err(ReceiverError::CounterOverflow)
    );
    assert_eq!(receiver.effect_count(10), 0);
}
