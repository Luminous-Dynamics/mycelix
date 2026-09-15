#[test]
fn verified_reconstruction_accepts_canonical_active_and_consumed_claims() {
    let descriptor = descriptor();
    let initial = ReservationState::initial(&descriptor).expect("initial state");

    let reconstructed_active = ReservationState::verified_from_claimed(
        &descriptor,
        initial.sequence(),
        initial.lifecycle().clone(),
        initial.state_commitment().into_bytes(),
    )
    .expect("canonical active claim should reconstruct");
    assert_eq!(reconstructed_active, initial);

    let consumed = apply_reservation_transition(
        &descriptor,
        &initial,
        initial.expected(),
        consume("attempt:1", "payment:idem:1", 400),
    )
    .expect("consume should apply")
    .into_state();

    let reconstructed_consumed = ReservationState::verified_from_claimed(
        &descriptor,
        consumed.sequence(),
        consumed.lifecycle().clone(),
        consumed.state_commitment().into_bytes(),
    )
    .expect("canonical consumed claim should reconstruct");
    assert_eq!(reconstructed_consumed, consumed);
}

#[test]
fn verified_reconstruction_rejects_forged_state_commitment() {
    let descriptor = descriptor();
    let initial = ReservationState::initial(&descriptor).expect("initial state");
    let mut forged = initial.state_commitment().into_bytes();
    forged[0] ^= 0x80;

    assert_eq!(
        ReservationState::verified_from_claimed(
            &descriptor,
            1,
            ReservationLifecycle::Active,
            forged,
        ),
        Err(ReservationLifecycleError::StateCommitmentMismatch)
    );
}

#[test]
fn verified_reconstruction_rejects_impossible_v1_sequences() {
    let descriptor = descriptor();
    let initial = ReservationState::initial(&descriptor).expect("initial state");

    assert_eq!(
        ReservationState::verified_from_claimed(
            &descriptor,
            2,
            ReservationLifecycle::Active,
            initial.state_commitment().into_bytes(),
        ),
        Err(ReservationLifecycleError::InvalidSequenceForLifecycle {
            expected: 1,
            actual: 2,
        })
    );

    assert_eq!(
        ReservationState::verified_from_claimed(
            &descriptor,
            1,
            ReservationLifecycle::Consumed {
                attempt: attempt("attempt:1"),
                idempotency_key: reference("payment:idem:1"),
                consumed_at_unix_ms: 400,
            },
            [0; 32],
        ),
        Err(ReservationLifecycleError::InvalidSequenceForLifecycle {
            expected: 2,
            actual: 1,
        })
    );
}

#[test]
fn verified_reconstruction_rejects_impossible_terminal_times_before_hash_check() {
    let descriptor = descriptor();

    assert_eq!(
        ReservationState::verified_from_claimed(
            &descriptor,
            2,
            ReservationLifecycle::Consumed {
                attempt: attempt("attempt:1"),
                idempotency_key: reference("payment:idem:1"),
                consumed_at_unix_ms: 99,
            },
            [0; 32],
        ),
        Err(ReservationLifecycleError::TransitionBeforeIssuance)
    );

    assert_eq!(
        ReservationState::verified_from_claimed(
            &descriptor,
            2,
            ReservationLifecycle::Released {
                evidence: reference("evidence:release"),
                released_at_unix_ms: 1_000,
            },
            [0; 32],
        ),
        Err(ReservationLifecycleError::TransitionAfterExpiry)
    );

    assert_eq!(
        ReservationState::verified_from_claimed(
            &descriptor,
            2,
            ReservationLifecycle::Expired {
                expired_at_unix_ms: 999,
            },
            [0; 32],
        ),
        Err(ReservationLifecycleError::EarlyExpiry)
    );
}