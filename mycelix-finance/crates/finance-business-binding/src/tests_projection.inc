fn settlement_profile(profile: FinalityProfileRef) -> FinalityProfile {
    FinalityProfile {
        profile_ref: profile,
        rail: reference("rail:bank"),
        network: reference("network:test"),
        required_evidence_kinds: BTreeSet::from([reference("evidence:provider")]),
        min_distinct_sources: 1,
        max_observation_age_ms: 1_000,
        reversal_model: ReversalModel::MayReverse,
    }
}

fn qualified_settlement_for(
    attempt: ExecutionAttemptRef,
    amount: AssetAmount,
    profile_ref: FinalityProfileRef,
    observed_at_unix_ms: u64,
    qualified_at_unix_ms: u64,
) -> mycelix_finance_settlement::QualifiedSettlement {
    let profile = settlement_profile(profile_ref.clone());
    let subject = SettlementSubject {
        id: reference("settlement:subject:1"),
        attempt: attempt.clone(),
        rail: profile.rail.clone(),
        network: profile.network.clone(),
        amount: amount.clone(),
        required_profile: profile_ref,
    };
    let operation_id = reference("operation:bank:1");
    let observation = SettlementObservation {
        observation_id: reference("settlement-observation:1"),
        subject: subject.id.clone(),
        attempt,
        rail: profile.rail.clone(),
        network: profile.network.clone(),
        operation_id: operation_id.clone(),
        revision: 1,
        amount,
        state: ObservedSettlementState::Applied,
        observed_at_unix_ms,
        evidence: vec![FinalityEvidence {
            evidence_id: reference("settlement-evidence:1"),
            subject: subject.id.clone(),
            operation_id,
            kind: reference("evidence:provider"),
            source: reference("source:bank"),
            digest: Digest32::repeat(41),
        }],
    };

    qualify_settlement(
        &subject,
        &profile,
        &[observation],
        qualified_at_unix_ms,
        Digest32::repeat(42),
    )
    .expect("settlement fixture should qualify")
}

#[test]
fn sealed_qualified_settlement_projects_without_asserting_business_success() {
    let f = fixture();
    let bound = bind(&f).expect("fixture should bind");
    let settlement = qualified_settlement_for(
        f.attempt.attempt.clone(),
        f.request.amount.clone(),
        f.request.required_finality_profile.clone(),
        450,
        500,
    );

    let projection = project_qualified_settlement(&bound, &settlement, 550)
        .expect("matching sealed settlement should project");

    assert_eq!(projection.attempt(), bound.attempt());
    assert_eq!(projection.amount(), bound.amount());
    assert!(matches!(
        projection.state(),
        FinanceProjectionState::QualifiedSettlement { .. }
    ));
}

#[test]
fn qualified_settlement_from_another_attempt_is_rejected() {
    let f = fixture();
    let bound = bind(&f).expect("fixture should bind");
    let settlement = qualified_settlement_for(
        attempt_ref("attempt:other"),
        f.request.amount.clone(),
        f.request.required_finality_profile.clone(),
        450,
        500,
    );

    assert_eq!(
        project_qualified_settlement(&bound, &settlement, 550),
        Err(FinanceProjectionError::SettlementAttemptMismatch)
    );
}

#[test]
fn qualified_settlement_amount_or_profile_cannot_drift() {
    let f = fixture();
    let bound = bind(&f).expect("fixture should bind");

    let wrong_amount = qualified_settlement_for(
        f.attempt.attempt.clone(),
        usd(99),
        f.request.required_finality_profile.clone(),
        450,
        500,
    );
    assert_eq!(
        project_qualified_settlement(&bound, &wrong_amount, 550),
        Err(FinanceProjectionError::SettlementAmountMismatch)
    );

    let wrong_profile = qualified_settlement_for(
        f.attempt.attempt.clone(),
        f.request.amount.clone(),
        profile_ref("finality:other:v1"),
        450,
        500,
    );
    assert_eq!(
        project_qualified_settlement(&bound, &wrong_profile, 550),
        Err(FinanceProjectionError::FinalityProfileMismatch)
    );
}

#[test]
fn settlement_cannot_precede_the_bound_execution_attempt() {
    let f = fixture();
    let bound = bind(&f).expect("fixture should bind");
    let settlement = qualified_settlement_for(
        f.attempt.attempt.clone(),
        f.request.amount.clone(),
        f.request.required_finality_profile.clone(),
        340,
        350,
    );

    assert_eq!(
        project_qualified_settlement(&bound, &settlement, 550),
        Err(FinanceProjectionError::SettlementPredatesBinding)
    );
}

#[test]
fn projection_cannot_claim_to_exist_before_settlement_qualification() {
    let f = fixture();
    let bound = bind(&f).expect("fixture should bind");
    let settlement = qualified_settlement_for(
        f.attempt.attempt.clone(),
        f.request.amount.clone(),
        f.request.required_finality_profile.clone(),
        450,
        500,
    );

    assert_eq!(
        project_qualified_settlement(&bound, &settlement, 499),
        Err(FinanceProjectionError::ProjectionPredatesSettlement)
    );
}

#[test]
fn unknown_finance_outcome_remains_explicitly_unknown() {
    let f = fixture();
    let bound = bind(&f).expect("fixture should bind");
    let unknown = UnknownFinanceOutcome {
        attempt: f.attempt.attempt.clone(),
        observation: reference("finance-outcome:unknown:1"),
        evidence_frontier: Digest32::repeat(43),
        observed_at_unix_ms: 450,
    };

    let projection = project_unknown_finance_outcome(&bound, &unknown, 500)
        .expect("unknown outcome should be preserved");
    assert!(matches!(
        projection.state(),
        FinanceProjectionState::Unknown { .. }
    ));
}

#[test]
fn unknown_outcome_cannot_be_replayed_from_another_attempt_or_before_binding() {
    let f = fixture();
    let bound = bind(&f).expect("fixture should bind");

    let wrong_attempt = UnknownFinanceOutcome {
        attempt: attempt_ref("attempt:other"),
        observation: reference("finance-outcome:unknown:other"),
        evidence_frontier: Digest32::repeat(44),
        observed_at_unix_ms: 450,
    };
    assert_eq!(
        project_unknown_finance_outcome(&bound, &wrong_attempt, 500),
        Err(FinanceProjectionError::UnknownAttemptMismatch)
    );

    let too_early = UnknownFinanceOutcome {
        attempt: f.attempt.attempt.clone(),
        observation: reference("finance-outcome:unknown:early"),
        evidence_frontier: Digest32::repeat(45),
        observed_at_unix_ms: ATTEMPT_AT - 1,
    };
    assert_eq!(
        project_unknown_finance_outcome(&bound, &too_early, 500),
        Err(FinanceProjectionError::UnknownPredatesBinding)
    );
}
