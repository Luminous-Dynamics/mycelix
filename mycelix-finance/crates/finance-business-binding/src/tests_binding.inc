#[test]
fn exact_binding_succeeds_at_attempt_boundary() {
    let f = fixture();
    let bound = bind(&f).expect("fixture should bind");

    assert_eq!(bound.attempt(), &f.attempt.attempt);
    assert_eq!(bound.reservation_id(), &f.reservation.reservation_id);
    assert_eq!(bound.amount(), &f.request.amount);
    assert_eq!(bound.required_finality_profile(), &f.request.required_finality_profile);
    assert_eq!(bound.bound_at_unix_ms(), ATTEMPT_AT);
}

#[test]
fn post_hoc_binding_time_is_rejected() {
    let f = fixture();
    let result = bind_finance_for_attempt(
        &f.decision,
        &f.coordination,
        &f.authorization,
        &f.attempt,
        &f.request,
        &f.reservation,
        &f.revalidation_contract,
        &f.revalidation_evidence,
        AUTHORITY_EPOCH,
        ATTEMPT_AT + 1,
    );
    assert_eq!(result, Err(FinanceBindingError::BindingTimeMismatch));
}

#[test]
fn reservation_issued_after_preparation_is_rejected() {
    let mut f = fixture();
    f.reservation.issued_at_unix_ms = f.coordination.prepared.prepared_at_unix_ms + 1;
    assert_eq!(
        bind(&f),
        Err(FinanceBindingError::ReservationIssuedAfterPreparation)
    );
}

#[test]
fn optional_finance_reference_cannot_be_promoted_to_required_authority() {
    let mut f = fixture();
    f.coordination.contract.required_reservation_domains.clear();
    assert_eq!(bind(&f), Err(FinanceBindingError::FinanceDomainNotRequired));
}

#[test]
fn duplicate_exact_reservation_references_fail_closed() {
    let mut f = fixture();
    let duplicate = f.coordination.prepared.reservations[0].clone();
    f.coordination.prepared.reservations.push(duplicate);
    assert_eq!(
        bind(&f),
        Err(FinanceBindingError::DuplicateReservationReference)
    );
}

#[test]
fn wrong_reservation_identity_is_not_substituted_by_same_domain() {
    let mut f = fixture();
    f.coordination.prepared.reservations[0].reservation_id =
        ReservationId(reference("reservation:finance:other"));
    assert_eq!(
        bind(&f),
        Err(FinanceBindingError::ReservationReferenceMissing)
    );
}

#[test]
fn reservation_digest_mismatch_fails_closed() {
    let mut f = fixture();
    f.coordination.prepared.reservations[0].digest = Digest32::repeat(99);
    assert_eq!(
        bind(&f),
        Err(FinanceBindingError::ReservationReferenceMismatch)
    );
}

#[test]
fn aggregate_policy_omission_cannot_bypass_business_contract() {
    let mut f = fixture();
    f.request.aggregate_policy_keys.clear();
    f.reservation.aggregate_policy_keys.clear();
    assert_eq!(bind(&f), Err(FinanceBindingError::AggregatePolicyMismatch));
}

#[test]
fn idempotency_identity_cannot_change_at_finance_boundary() {
    let mut f = fixture();
    f.request.idempotency_key = reference("payment:idem:other");
    f.reservation.idempotency_key = f.request.idempotency_key.clone();
    assert_eq!(bind(&f), Err(FinanceBindingError::IdempotencyMismatch));
}

#[test]
fn policy_frontier_drift_is_caught_by_real_business_revalidation() {
    let mut f = fixture();
    f.revalidation_evidence = vec![RevalidationEvidence::ExactFrontier {
        class: FrontierClass::Policy,
        current: FrontierRef {
            domain: reference("policy:finance-spend"),
            sequence: 1,
            digest: Digest32::repeat(98),
        },
        checked_at_unix_ms: 390,
    }];

    assert!(matches!(
        bind(&f),
        Err(FinanceBindingError::Revalidation(
            mycelix_business_revalidation::RevalidationError::ExactFrontierChanged { .. }
        ))
    ));
}

#[test]
fn authority_epoch_change_fails_before_finance_binding() {
    let f = fixture();
    let result = bind_finance_for_attempt(
        &f.decision,
        &f.coordination,
        &f.authorization,
        &f.attempt,
        &f.request,
        &f.reservation,
        &f.revalidation_contract,
        &f.revalidation_evidence,
        AUTHORITY_EPOCH + 1,
        ATTEMPT_AT,
    );

    assert!(matches!(
        result,
        Err(FinanceBindingError::InvalidPreparedAction(_))
    ));
}
