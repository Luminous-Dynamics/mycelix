fn assert_descriptor_mutation_changes(
    mutate: fn(&mut ReservationDescriptor),
) {
    let base = descriptor();
    let base_commitment = reservation_commitment(&base).expect("valid base descriptor");
    let mut changed = base;
    mutate(&mut changed);
    let changed_commitment = reservation_commitment(&changed).expect("valid changed descriptor");
    assert_ne!(base_commitment, changed_commitment);
}

#[test]
fn every_authority_bearing_descriptor_field_changes_commitment() {
    assert_descriptor_mutation_changes(|d| d.reservation_id = ReservationId(reference("reservation:2")));
    assert_descriptor_mutation_changes(|d| d.request_id = reference("request:2"));
    assert_descriptor_mutation_changes(|d| d.finance_domain = reference("finance:other"));
    assert_descriptor_mutation_changes(|d| d.issuer_authority = reference("finance-authority:beta"));
    assert_descriptor_mutation_changes(|d| d.issuer_profile = reference("issuer:finance:v2"));
    assert_descriptor_mutation_changes(|d| d.issuer_profile_revision = 2);
    assert_descriptor_mutation_changes(|d| d.issuer_profile_digest = Digest32::repeat(7));
    assert_descriptor_mutation_changes(|d| d.finance_policy_sequence = 10);
    assert_descriptor_mutation_changes(|d| d.finance_policy_digest = Digest32::repeat(8));
    assert_descriptor_mutation_changes(|d| d.subject = SubjectRef(reference("treasury:other")));
    assert_descriptor_mutation_changes(|d| {
        d.amount = AssetAmount::new(
            101,
            AssetId::new("USD.micro").expect("static asset identifier"),
        )
    });
    assert_descriptor_mutation_changes(|d| {
        d.amount = AssetAmount::new(
            100,
            AssetId::new("EUR.micro").expect("static asset identifier"),
        )
    });
    assert_descriptor_mutation_changes(|d| d.effect_class = reference("finance-effect:credit-draw"));
    assert_descriptor_mutation_changes(|d| d.action_contract.semantic_id = reference("mycelix.finance.refund.v1"));
    assert_descriptor_mutation_changes(|d| d.action_contract.digest = Digest32::repeat(9));
    assert_descriptor_mutation_changes(|d| d.decision.id = reference("decision:2"));
    assert_descriptor_mutation_changes(|d| d.decision.digest = Digest32::repeat(10));
    assert_descriptor_mutation_changes(|d| d.authorized_intent = AuthorizedIntentRef(reference("intent:2")));
    assert_descriptor_mutation_changes(|d| d.intent_digest = Digest32::repeat(11));
    assert_descriptor_mutation_changes(|d| d.authority_lease_id = reference("authority:lease:2"));
    assert_descriptor_mutation_changes(|d| d.authority_epoch = 8);
    assert_descriptor_mutation_changes(|d| d.fencing_token = 12);
    assert_descriptor_mutation_changes(|d| {
        d.aggregate_policy_keys.insert(reference("aggregate:project:24h"));
    });
    assert_descriptor_mutation_changes(|d| d.idempotency_key = reference("payment:idem:2"));
    assert_descriptor_mutation_changes(|d| d.required_finality_profile.id = reference("finality:bank:v2"));
    assert_descriptor_mutation_changes(|d| d.required_finality_profile.revision = 2);
    assert_descriptor_mutation_changes(|d| d.required_finality_profile.digest = Digest32::repeat(12));
    assert_descriptor_mutation_changes(|d| d.issued_at_unix_ms = 101);
    assert_descriptor_mutation_changes(|d| d.expires_at_unix_ms = 1_001);
}

#[test]
fn aggregate_policy_key_insertion_order_is_canonical() {
    let first = descriptor();
    let mut second = descriptor();
    second.aggregate_policy_keys.clear();
    second
        .aggregate_policy_keys
        .insert(reference("aggregate:treasury:24h"));
    second
        .aggregate_policy_keys
        .insert(reference("aggregate:counterparty:24h"));

    assert_eq!(
        canonical_descriptor_bytes(&first).expect("valid descriptor"),
        canonical_descriptor_bytes(&second).expect("valid descriptor")
    );
    assert_eq!(
        reservation_commitment(&first).expect("valid descriptor"),
        reservation_commitment(&second).expect("valid descriptor")
    );
}

#[test]
fn unsupported_profile_and_invalid_authority_fields_fail_closed() {
    let mut invalid = descriptor();
    invalid.commitment_profile_revision = RESERVATION_COMMITMENT_PROFILE_REVISION + 1;
    assert_eq!(
        reservation_commitment(&invalid),
        Err(ReservationDescriptorError::UnsupportedCommitmentProfile {
            expected: RESERVATION_COMMITMENT_PROFILE_REVISION,
            actual: RESERVATION_COMMITMENT_PROFILE_REVISION + 1,
        })
    );

    let mut zero_amount = descriptor();
    zero_amount.amount = AssetAmount::new(
        0,
        AssetId::new("USD.micro").expect("static asset identifier"),
    );
    assert_eq!(
        reservation_commitment(&zero_amount),
        Err(ReservationDescriptorError::ZeroAmount)
    );

    let mut zero_issuer_profile_revision = descriptor();
    zero_issuer_profile_revision.issuer_profile_revision = 0;
    assert_eq!(
        reservation_commitment(&zero_issuer_profile_revision),
        Err(ReservationDescriptorError::ZeroIssuerProfileRevision)
    );

    let mut zero_finance_policy_sequence = descriptor();
    zero_finance_policy_sequence.finance_policy_sequence = 0;
    assert_eq!(
        reservation_commitment(&zero_finance_policy_sequence),
        Err(ReservationDescriptorError::ZeroFinancePolicySequence)
    );

    let mut zero_epoch = descriptor();
    zero_epoch.authority_epoch = 0;
    assert_eq!(
        reservation_commitment(&zero_epoch),
        Err(ReservationDescriptorError::ZeroAuthorityEpoch)
    );

    let mut zero_fence = descriptor();
    zero_fence.fencing_token = 0;
    assert_eq!(
        reservation_commitment(&zero_fence),
        Err(ReservationDescriptorError::ZeroFencingToken)
    );

    let mut zero_profile_revision = descriptor();
    zero_profile_revision.required_finality_profile.revision = 0;
    assert_eq!(
        reservation_commitment(&zero_profile_revision),
        Err(ReservationDescriptorError::ZeroFinalityProfileRevision)
    );

    let mut invalid_window = descriptor();
    invalid_window.expires_at_unix_ms = invalid_window.issued_at_unix_ms;
    assert_eq!(
        reservation_commitment(&invalid_window),
        Err(ReservationDescriptorError::InvalidWindow)
    );
}
