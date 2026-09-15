#[test]
fn valid_consume_advances_sequence_and_state_commitment() {
    let descriptor = descriptor();
    let initial = ReservationState::initial(&descriptor).expect("initial state");
    let initial_commitment = initial.state_commitment();

    let outcome = apply_reservation_transition(
        &descriptor,
        &initial,
        initial.expected(),
        consume("attempt:1", "payment:idem:1", 400),
    )
    .expect("consume should apply");

    assert!(!outcome.is_idempotent_replay());
    let consumed = outcome.state();
    assert_eq!(consumed.sequence(), 2);
    assert_ne!(consumed.state_commitment(), initial_commitment);
    assert!(matches!(
        consumed.lifecycle(),
        ReservationLifecycle::Consumed {
            consumed_at_unix_ms: 400,
            ..
        }
    ));
}

#[test]
fn same_attempt_and_idempotency_replay_is_explicitly_idempotent() {
    let descriptor = descriptor();
    let initial = ReservationState::initial(&descriptor).expect("initial state");
    let consumed = apply_reservation_transition(
        &descriptor,
        &initial,
        initial.expected(),
        consume("attempt:1", "payment:idem:1", 400),
    )
    .expect("consume should apply")
    .into_state();

    let replay = apply_reservation_transition(
        &descriptor,
        &consumed,
        consumed.expected(),
        consume("attempt:1", "payment:idem:1", 450),
    )
    .expect("same attempt replay should be idempotent");

    assert!(replay.is_idempotent_replay());
    assert_eq!(replay.state().sequence(), consumed.sequence());
    assert_eq!(
        replay.state().state_commitment(),
        consumed.state_commitment()
    );
}

#[test]
fn replay_cannot_claim_a_time_before_original_consumption() {
    let descriptor = descriptor();
    let initial = ReservationState::initial(&descriptor).expect("initial state");
    let consumed = apply_reservation_transition(
        &descriptor,
        &initial,
        initial.expected(),
        consume("attempt:1", "payment:idem:1", 400),
    )
    .expect("consume should apply")
    .into_state();

    assert_eq!(
        apply_reservation_transition(
            &descriptor,
            &consumed,
            consumed.expected(),
            consume("attempt:1", "payment:idem:1", 399),
        ),
        Err(ReservationLifecycleError::ReplayBeforeOriginalConsumption)
    );
}

#[test]
fn another_attempt_or_idempotency_identity_cannot_reconsume() {
    let descriptor = descriptor();
    let initial = ReservationState::initial(&descriptor).expect("initial state");
    let consumed = apply_reservation_transition(
        &descriptor,
        &initial,
        initial.expected(),
        consume("attempt:1", "payment:idem:1", 400),
    )
    .expect("consume should apply")
    .into_state();

    assert_eq!(
        apply_reservation_transition(
            &descriptor,
            &consumed,
            consumed.expected(),
            consume("attempt:2", "payment:idem:1", 450),
        ),
        Err(ReservationLifecycleError::AlreadyConsumed)
    );
    assert_eq!(
        apply_reservation_transition(
            &descriptor,
            &consumed,
            consumed.expected(),
            consume("attempt:1", "payment:idem:other", 450),
        ),
        Err(ReservationLifecycleError::AlreadyConsumed)
    );
}

#[test]
fn consumed_capacity_cannot_be_silently_restored_by_release() {
    let descriptor = descriptor();
    let initial = ReservationState::initial(&descriptor).expect("initial state");
    let consumed = apply_reservation_transition(
        &descriptor,
        &initial,
        initial.expected(),
        consume("attempt:1", "payment:idem:1", 400),
    )
    .expect("consume should apply")
    .into_state();

    assert_eq!(
        apply_reservation_transition(
            &descriptor,
            &consumed,
            consumed.expected(),
            ReservationTransition::Release {
                evidence: reference("evidence:settlement-failed"),
                at_unix_ms: 500,
            },
        ),
        Err(ReservationLifecycleError::AlreadyConsumed)
    );
}

#[test]
fn release_revoke_and_expire_are_distinct_terminal_states() {
    let descriptor = descriptor();
    let initial = ReservationState::initial(&descriptor).expect("initial state");

    let released = apply_reservation_transition(
        &descriptor,
        &initial,
        initial.expected(),
        ReservationTransition::Release {
            evidence: reference("evidence:release"),
            at_unix_ms: 400,
        },
    )
    .expect("release should apply")
    .into_state();
    assert!(matches!(
        released.lifecycle(),
        ReservationLifecycle::Released { .. }
    ));
    assert_ne!(released.state_commitment(), initial.state_commitment());

    let revoked = apply_reservation_transition(
        &descriptor,
        &initial,
        initial.expected(),
        ReservationTransition::Revoke {
            evidence: reference("evidence:revoke"),
            at_unix_ms: 400,
        },
    )
    .expect("revoke should apply")
    .into_state();
    assert!(matches!(
        revoked.lifecycle(),
        ReservationLifecycle::Revoked { .. }
    ));
    assert_ne!(revoked.state_commitment(), initial.state_commitment());
    assert_ne!(revoked.state_commitment(), released.state_commitment());

    let expired = apply_reservation_transition(
        &descriptor,
        &initial,
        initial.expected(),
        ReservationTransition::Expire { at_unix_ms: 1_000 },
    )
    .expect("expiry boundary should apply")
    .into_state();
    assert!(matches!(
        expired.lifecycle(),
        ReservationLifecycle::Expired {
            expired_at_unix_ms: 1_000
        }
    ));
    assert_ne!(expired.state_commitment(), initial.state_commitment());
    assert_ne!(expired.state_commitment(), released.state_commitment());
    assert_ne!(expired.state_commitment(), revoked.state_commitment());
}

#[test]
fn terminal_non_consumed_state_cannot_transition_again() {
    let descriptor = descriptor();
    let initial = ReservationState::initial(&descriptor).expect("initial state");
    let released = apply_reservation_transition(
        &descriptor,
        &initial,
        initial.expected(),
        ReservationTransition::Release {
            evidence: reference("evidence:release"),
            at_unix_ms: 400,
        },
    )
    .expect("release should apply")
    .into_state();

    assert_eq!(
        apply_reservation_transition(
            &descriptor,
            &released,
            released.expected(),
            consume("attempt:1", "payment:idem:1", 500),
        ),
        Err(ReservationLifecycleError::TerminalState)
    );
}

#[test]
fn transition_time_bounds_fail_closed() {
    let descriptor = descriptor();
    let initial = ReservationState::initial(&descriptor).expect("initial state");

    assert_eq!(
        apply_reservation_transition(
            &descriptor,
            &initial,
            initial.expected(),
            consume("attempt:1", "payment:idem:1", 99),
        ),
        Err(ReservationLifecycleError::TransitionBeforeIssuance)
    );
    assert_eq!(
        apply_reservation_transition(
            &descriptor,
            &initial,
            initial.expected(),
            consume("attempt:1", "payment:idem:1", 1_000),
        ),
        Err(ReservationLifecycleError::TransitionAfterExpiry)
    );
    assert_eq!(
        apply_reservation_transition(
            &descriptor,
            &initial,
            initial.expected(),
            ReservationTransition::Expire { at_unix_ms: 999 },
        ),
        Err(ReservationLifecycleError::EarlyExpiry)
    );
}

#[test]
fn stale_sequence_and_state_frontier_fail_closed() {
    let base = descriptor();
    let initial = ReservationState::initial(&base).expect("initial state");

    let stale_sequence = ExpectedReservationState::new(
        0,
        initial.state_commitment(),
    );
    assert_eq!(
        apply_reservation_transition(
            &base,
            &initial,
            stale_sequence,
            consume("attempt:1", "payment:idem:1", 400),
        ),
        Err(ReservationLifecycleError::StaleSequence {
            expected: 0,
            actual: 1,
        })
    );

    let mut other_descriptor = descriptor();
    other_descriptor.reservation_id = ReservationId(reference("reservation:other"));
    let other = ReservationState::initial(&other_descriptor).expect("other initial state");
    let stale_frontier = ExpectedReservationState::new(
        initial.sequence(),
        other.state_commitment(),
    );
    assert_eq!(
        apply_reservation_transition(
            &base,
            &initial,
            stale_frontier,
            consume("attempt:1", "payment:idem:1", 400),
        ),
        Err(ReservationLifecycleError::StaleStateCommitment)
    );
}

#[test]
fn state_cannot_be_reinterpreted_under_a_changed_descriptor() {
    let descriptor = descriptor();
    let initial = ReservationState::initial(&descriptor).expect("initial state");
    let mut changed = descriptor.clone();
    changed.amount = AssetAmount::new(
        101,
        AssetId::new("USD.micro").expect("static asset identifier"),
    );

    assert_eq!(
        apply_reservation_transition(
            &changed,
            &initial,
            initial.expected(),
            consume("attempt:1", "payment:idem:1", 400),
        ),
        Err(ReservationLifecycleError::DescriptorCommitmentMismatch)
    );
}
