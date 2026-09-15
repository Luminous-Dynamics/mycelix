#[test]
fn multiple_operations_can_exactly_satisfy_subject() {
    qualify_settlement(
        &subject(),
        &profile(),
        &[
            observation("op:1", 1, 40, ObservedSettlementState::Applied, 1_000),
            observation("op:2", 1, 60, ObservedSettlementState::Applied, 1_010),
        ],
        1_100,
        Digest32::repeat(3),
    ).expect("two applied operations should compose exactly");
}

#[test]
fn stale_and_future_observations_fail_closed() {
    let mut stale_profile = profile();
    stale_profile.max_observation_age_ms = 10;
    assert_eq!(
        qualify_settlement(
            &subject(),
            &stale_profile,
            &[observation("op:1", 1, 100, ObservedSettlementState::Applied, 1_000)],
            1_100,
            Digest32::repeat(3),
        ),
        Err(SettlementQualificationError::StaleObservation)
    );
    assert_eq!(
        qualify_settlement(
            &subject(),
            &profile(),
            &[observation("op:1", 1, 100, ObservedSettlementState::Applied, 1_200)],
            1_100,
            Digest32::repeat(3),
        ),
        Err(SettlementQualificationError::FutureObservation)
    );
}

#[test]
fn wrong_network_and_asset_fail_closed() {
    let mut wrong_network = observation("op:1", 1, 100, ObservedSettlementState::Applied, 1_000);
    wrong_network.network = reference("network:wrong");
    assert_eq!(
        qualify_settlement(&subject(), &profile(), &[wrong_network], 1_100, Digest32::repeat(3)),
        Err(SettlementQualificationError::ObservationNetworkMismatch)
    );

    let mut wrong_asset = observation("op:1", 1, 100, ObservedSettlementState::Applied, 1_000);
    wrong_asset.amount = AssetAmount::new(100, AssetId::new("EUR.micro").expect("static asset"));
    assert_eq!(
        qualify_settlement(&subject(), &profile(), &[wrong_asset], 1_100, Digest32::repeat(3)),
        Err(SettlementQualificationError::AssetMismatch)
    );
}

#[test]
fn replayed_evidence_for_another_subject_is_rejected() {
    let mut replay = observation("op:1", 1, 100, ObservedSettlementState::Applied, 1_000);
    replay.evidence[0].subject = reference("settlement:subject:other");
    assert_eq!(
        qualify_settlement(&subject(), &profile(), &[replay], 1_100, Digest32::repeat(3)),
        Err(SettlementQualificationError::EvidenceSubjectMismatch)
    );
}

#[test]
fn profile_requires_declared_evidence_kinds_and_source_diversity() {
    let mut strict_profile = profile();
    strict_profile.required_evidence_kinds.insert(reference("evidence:independent-witness"));
    assert_eq!(
        qualify_settlement(
            &subject(),
            &strict_profile,
            &[observation("op:1", 1, 100, ObservedSettlementState::Applied, 1_000)],
            1_100,
            Digest32::repeat(3),
        ),
        Err(SettlementQualificationError::MissingEvidenceKind)
    );

    let mut diverse_profile = profile();
    diverse_profile.min_distinct_sources = 2;
    assert_eq!(
        qualify_settlement(
            &subject(),
            &diverse_profile,
            &[observation("op:1", 1, 100, ObservedSettlementState::Applied, 1_000)],
            1_100,
            Digest32::repeat(3),
        ),
        Err(SettlementQualificationError::InsufficientDistinctSources)
    );
}

#[test]
fn later_reversal_derives_explicit_invalidation_lineage() {
    let prior = qualify_settlement(
        &subject(),
        &profile(),
        &[observation("op:1", 1, 100, ObservedSettlementState::Applied, 1_000)],
        1_100,
        Digest32::repeat(3),
    ).expect("initial qualification");
    let reversal = observation("op:1", 2, 100, ObservedSettlementState::Reversed, 1_200);
    let invalidation = derive_invalidation(&prior, &reversal)
        .expect("later reversal should invalidate historical qualification");
    assert_eq!(invalidation.prior_revision(), 1);
    assert_eq!(invalidation.operation_id(), &reference("op:1"));
}

#[test]
fn same_applied_state_at_higher_revision_does_not_invalidate() {
    let prior = qualify_settlement(
        &subject(),
        &profile(),
        &[observation("op:1", 1, 100, ObservedSettlementState::Applied, 1_000)],
        1_100,
        Digest32::repeat(3),
    ).expect("initial qualification");
    let same = observation("op:1", 2, 100, ObservedSettlementState::Applied, 1_200);
    assert_eq!(
        derive_invalidation(&prior, &same),
        Err(SettlementInvalidationError::ObservationDoesNotInvalidate)
    );
}
