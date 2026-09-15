#[test]
fn exact_applied_settlement_qualifies() {
    let result = qualify_settlement(
        &subject(),
        &profile(),
        &[observation("op:1", 1, 100, ObservedSettlementState::Applied, 1_000)],
        1_100,
        Digest32::repeat(3),
    ).expect("qualification should succeed");
    assert_eq!(result.amount().atomic_units(), 100);
    assert_eq!(result.operations().len(), 1);
}

#[test]
fn duplicate_consistent_observations_do_not_double_count() {
    let mut second = observation("op:1", 1, 100, ObservedSettlementState::Applied, 1_010);
    second.observation_id = reference("observation:op:1:duplicate");
    second.evidence = vec![evidence("op:1", "provider-b")];
    let result = qualify_settlement(
        &subject(),
        &profile(),
        &[
            observation("op:1", 1, 100, ObservedSettlementState::Applied, 1_000),
            second,
        ],
        1_100,
        Digest32::repeat(3),
    ).expect("consistent duplicate should merge evidence");
    assert_eq!(result.amount().atomic_units(), 100);
    assert_eq!(result.operations().len(), 1);
}

#[test]
fn conflicting_same_revision_fails_closed() {
    let observations = vec![
        observation("op:1", 1, 100, ObservedSettlementState::Applied, 1_000),
        observation("op:1", 1, 100, ObservedSettlementState::Reversed, 1_001),
    ];
    assert_eq!(
        qualify_settlement(&subject(), &profile(), &observations, 1_100, Digest32::repeat(3)),
        Err(SettlementQualificationError::ConflictingCurrentRevision)
    );
}

#[test]
fn latest_revision_controls_current_state() {
    let observations = vec![
        observation("op:1", 1, 100, ObservedSettlementState::Unknown, 900),
        observation("op:1", 2, 100, ObservedSettlementState::Applied, 1_000),
    ];
    qualify_settlement(&subject(), &profile(), &observations, 1_100, Digest32::repeat(3))
        .expect("later applied revision should resolve old unknown state");
}

#[test]
fn current_unknown_blocks_qualification() {
    assert_eq!(
        qualify_settlement(
            &subject(),
            &profile(),
            &[observation("op:1", 1, 100, ObservedSettlementState::Unknown, 1_000)],
            1_100,
            Digest32::repeat(3),
        ),
        Err(SettlementQualificationError::OutstandingUncertainty)
    );
}

#[test]
fn partial_and_over_settlement_are_distinct() {
    assert_eq!(
        qualify_settlement(
            &subject(),
            &profile(),
            &[observation("op:1", 1, 80, ObservedSettlementState::Applied, 1_000)],
            1_100,
            Digest32::repeat(3),
        ),
        Err(SettlementQualificationError::PartialSettlement)
    );
    assert_eq!(
        qualify_settlement(
            &subject(),
            &profile(),
            &[observation("op:1", 1, 120, ObservedSettlementState::Applied, 1_000)],
            1_100,
            Digest32::repeat(3),
        ),
        Err(SettlementQualificationError::OverSettlement)
    );
}
