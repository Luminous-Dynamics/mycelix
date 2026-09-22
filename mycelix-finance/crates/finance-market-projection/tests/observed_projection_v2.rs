use mycelix_finance_exact::{AssetAmount, AssetId};
use mycelix_finance_market_core::{
    canonicalize_order_intent_v1, CanonicalMarketOrderIntentV1, Digest32,
    MarketExternalIdV1, MarketIdempotencyRefV1, MarketInstrumentRefV1,
    MarketOrderIntentInputV1, MarketPriceV1, MarketProfileRefV1, MarketProtocolIdV1,
    MarketQuantityV1, MarketSideV1, MarketSubjectRefV1, OrderTermsV1, QuantitySpecV1,
    TimeInForceV1,
};
use mycelix_finance_market_execution_frontier::{
    resolve_observed_execution_frontier_v1, ExecutionFrontierScopeV1,
};
use mycelix_finance_market_observation::{
    canonicalize_event_observation_v1, canonicalize_fill_adjustment_v1,
    canonicalize_fill_observation_v1, CanonicalMarketEventObservationV1,
    FillAdjustmentKindV1, FillAdjustmentObservationInputV1, FillObservationInputV1,
    MarketEventKindV1, MarketEventObservationInputV1, MarketOrderObservationSubjectV1,
    ProviderChronologyV1, ProviderObservationRefV1,
};
use mycelix_finance_market_projection::{
    project_observed_market_order_v2, LatestObservedControlPostureV2,
    ObservedExecutionPostureV2, ObservedTargetProgressV2, ObservedUnitTargetComparisonV2,
    ProjectionScopeV2,
};

fn profile(name: &str, byte: u8) -> MarketProfileRefV1 {
    MarketProfileRefV1 {
        profile_id: MarketProtocolIdV1::new(name).expect("profile id"),
        revision: 1,
        digest: Digest32::from_bytes([byte; 32]),
    }
}

fn intent(notional: bool) -> CanonicalMarketOrderIntentV1 {
    let quantity = if notional {
        QuantitySpecV1::Notional(MarketQuantityV1 {
            unit_profile: profile("unit:notional-usd", 1),
            amount: AssetAmount::new(100_000, AssetId::new("USD:micro").expect("asset")),
        })
    } else {
        QuantitySpecV1::Units(MarketQuantityV1 {
            unit_profile: profile("unit:share", 2),
            amount: AssetAmount::new(100, AssetId::new("share:XYZ").expect("asset")),
        })
    };

    canonicalize_order_intent_v1(MarketOrderIntentInputV1 {
        intent_subject: MarketSubjectRefV1 {
            subject_profile: profile("subject:intent", 3),
            subject_id: MarketExternalIdV1::new("intent-1").expect("id"),
        },
        account_subject: MarketSubjectRefV1 {
            subject_profile: profile("subject:account", 4),
            subject_id: MarketExternalIdV1::new("account-1").expect("id"),
        },
        instrument: MarketInstrumentRefV1 {
            instrument_profile: profile("instrument:equity", 5),
            instrument_id: MarketExternalIdV1::new("XYZ").expect("id"),
        },
        side: MarketSideV1::AcquireLong,
        quantity,
        order_terms: OrderTermsV1::Market,
        time_in_force: TimeInForceV1::Day,
        execution_profile: profile("execution:test", 6),
        semantic_idempotency: MarketIdempotencyRefV1 {
            idempotency_profile: profile("idempotency:test", 7),
            semantic_id: MarketProtocolIdV1::new("order-1").expect("semantic id"),
        },
        upstream_economic_effect_commitment: None,
    })
    .expect("intent")
}

fn execution_scope() -> ExecutionFrontierScopeV1 {
    ExecutionFrontierScopeV1 {
        provider_profile: profile("provider:test", 8),
        fill_observation_profile: profile("obs:fill", 9),
        adjustment_observation_profile: profile("obs:adjust", 10),
    }
}

fn projection_scope() -> ProjectionScopeV2 {
    ProjectionScopeV2 {
        event_observation_profile: profile("obs:event", 11),
        execution_frontier_scope: execution_scope(),
    }
}

fn subject(intent: &CanonicalMarketOrderIntentV1) -> MarketOrderObservationSubjectV1 {
    MarketOrderObservationSubjectV1 {
        intent_commitment: intent.commitment(),
        account_subject: intent.input().account_subject.clone(),
        instrument: intent.input().instrument.clone(),
        provider_profile: profile("provider:test", 8),
        provider_order_ref: Some(MarketExternalIdV1::new("provider-order-1").expect("id")),
    }
}

fn fill(
    intent: &CanonicalMarketOrderIntentV1,
    observation_id: &str,
    execution_id: &str,
    quantity: u64,
) -> mycelix_finance_market_observation::CanonicalFillObservationV1 {
    canonicalize_fill_observation_v1(FillObservationInputV1 {
        subject: subject(intent),
        observation_ref: ProviderObservationRefV1 {
            observation_profile: profile("obs:fill", 9),
            observation_id: MarketExternalIdV1::new(observation_id).expect("id"),
        },
        provider_execution_ref: MarketExternalIdV1::new(execution_id).expect("id"),
        executed_quantity: MarketQuantityV1 {
            unit_profile: profile("unit:share", 2),
            amount: AssetAmount::new(quantity, AssetId::new("share:XYZ").expect("asset")),
        },
        execution_price: MarketPriceV1 {
            pricing_profile: profile("price:usd", 12),
            quote_amount: AssetAmount::new(12_500, AssetId::new("USD:micro").expect("asset")),
        },
        venue_ref: None,
        chronology: ProviderChronologyV1 {
            chronology_profile: None,
            sequence: None,
            provider_time: None,
        },
        source_evidence_commitment: Digest32::from_bytes([13; 32]),
    })
    .expect("fill")
}

fn event(
    intent: &CanonicalMarketOrderIntentV1,
    observation_id: &str,
    kind: MarketEventKindV1,
    sequence: Option<u64>,
) -> CanonicalMarketEventObservationV1 {
    canonicalize_event_observation_v1(MarketEventObservationInputV1 {
        subject: subject(intent),
        observation_ref: ProviderObservationRefV1 {
            observation_profile: profile("obs:event", 11),
            observation_id: MarketExternalIdV1::new(observation_id).expect("id"),
        },
        event_kind: kind,
        chronology: match sequence {
            Some(sequence) => ProviderChronologyV1 {
                chronology_profile: Some(profile("chronology:test", 14)),
                sequence: Some(sequence),
                provider_time: None,
            },
            None => ProviderChronologyV1 {
                chronology_profile: None,
                sequence: None,
                provider_time: None,
            },
        },
        source_evidence_commitment: Digest32::from_bytes([15; 32]),
    })
    .expect("event")
}

#[test]
fn duplicate_fill_envelopes_project_one_economic_execution() {
    let intent = intent(false);
    let fills = vec![
        fill(&intent, "fill-envelope-a", "exec-1", 40),
        fill(&intent, "fill-envelope-b", "exec-1", 40),
    ];
    let frontier = resolve_observed_execution_frontier_v1(
        &intent,
        &execution_scope(),
        &fills,
        &[],
    )
    .expect("frontier");

    let projection = project_observed_market_order_v2(
        &intent,
        &projection_scope(),
        &[],
        &frontier,
    )
    .expect("projection");

    assert_eq!(projection.unique_fill_observation_count(), 2);
    assert_eq!(projection.unique_execution_identity_count(), 1);
    assert_eq!(projection.execution_posture(), ObservedExecutionPostureV2::EffectiveExecutionObserved);
    match projection.target_progress() {
        ObservedTargetProgressV2::Unit {
            effective_observed_atomic_units,
            comparison,
            ..
        } => {
            assert_eq!(*effective_observed_atomic_units, 40);
            assert_eq!(*comparison, ObservedUnitTargetComparisonV2::ObservedBelowTarget);
        }
        other => panic!("unexpected progress: {other:?}"),
    }
}

#[test]
fn distinct_execution_ids_with_identical_economics_remain_additive() {
    let intent = intent(false);
    let fills = vec![
        fill(&intent, "fill-a", "exec-1", 40),
        fill(&intent, "fill-b", "exec-2", 40),
    ];
    let frontier = resolve_observed_execution_frontier_v1(
        &intent,
        &execution_scope(),
        &fills,
        &[],
    )
    .expect("frontier");
    let projection = project_observed_market_order_v2(
        &intent,
        &projection_scope(),
        &[],
        &frontier,
    )
    .expect("projection");

    assert_eq!(projection.unique_execution_identity_count(), 2);
    match projection.target_progress() {
        ObservedTargetProgressV2::Unit {
            effective_observed_atomic_units,
            ..
        } => assert_eq!(*effective_observed_atomic_units, 80),
        other => panic!("unexpected progress: {other:?}"),
    }
}

#[test]
fn busted_execution_remains_historical_execution_evidence() {
    let intent = intent(false);
    let fill = fill(&intent, "fill-a", "exec-1", 40);
    let bust = canonicalize_fill_adjustment_v1(FillAdjustmentObservationInputV1 {
        subject: subject(&intent),
        observation_ref: ProviderObservationRefV1 {
            observation_profile: profile("obs:adjust", 10),
            observation_id: MarketExternalIdV1::new("bust-a").expect("id"),
        },
        adjustment_kind: FillAdjustmentKindV1::Bust {
            prior_fill_commitment: fill.semantic_commitment(),
        },
        chronology: ProviderChronologyV1 {
            chronology_profile: None,
            sequence: None,
            provider_time: None,
        },
        source_evidence_commitment: Digest32::from_bytes([16; 32]),
    })
    .expect("bust");
    let frontier = resolve_observed_execution_frontier_v1(
        &intent,
        &execution_scope(),
        &[fill],
        &[bust],
    )
    .expect("frontier");
    let projection = project_observed_market_order_v2(
        &intent,
        &projection_scope(),
        &[],
        &frontier,
    )
    .expect("projection");

    assert_eq!(projection.unique_execution_identity_count(), 1);
    assert_eq!(
        projection.execution_posture(),
        ObservedExecutionPostureV2::HistoricalExecutionOnlyObserved
    );
    match projection.target_progress() {
        ObservedTargetProgressV2::Unit {
            effective_observed_atomic_units,
            comparison,
            ..
        } => {
            assert_eq!(*effective_observed_atomic_units, 0);
            assert_eq!(*comparison, ObservedUnitTargetComparisonV2::ObservedNone);
        }
        other => panic!("unexpected progress: {other:?}"),
    }
}

#[test]
fn chronology_projection_is_input_order_independent() {
    let intent = intent(false);
    let frontier = resolve_observed_execution_frontier_v1(
        &intent,
        &execution_scope(),
        &[],
        &[],
    )
    .expect("frontier");
    let working = event(&intent, "evt-working", MarketEventKindV1::WorkingObserved, Some(10));
    let cancel = event(
        &intent,
        "evt-cancel",
        MarketEventKindV1::CancelRequestedObserved,
        Some(20),
    );

    let first = project_observed_market_order_v2(
        &intent,
        &projection_scope(),
        &[working.clone(), cancel.clone()],
        &frontier,
    )
    .expect("projection");
    let reversed = project_observed_market_order_v2(
        &intent,
        &projection_scope(),
        &[cancel, working],
        &frontier,
    )
    .expect("projection");

    assert_eq!(first.latest_observed_control(), reversed.latest_observed_control());
    assert_eq!(
        first.latest_observed_control(),
        &LatestObservedControlPostureV2::PendingCancel
    );
}

#[test]
fn multiple_unsequenced_control_events_remain_indeterminate() {
    let intent = intent(false);
    let frontier = resolve_observed_execution_frontier_v1(
        &intent,
        &execution_scope(),
        &[],
        &[],
    )
    .expect("frontier");
    let events = vec![
        event(&intent, "evt-working", MarketEventKindV1::WorkingObserved, None),
        event(&intent, "evt-done", MarketEventKindV1::DoneForDayObserved, None),
    ];
    let projection = project_observed_market_order_v2(
        &intent,
        &projection_scope(),
        &events,
        &frontier,
    )
    .expect("projection");
    assert_eq!(
        projection.latest_observed_control(),
        &LatestObservedControlPostureV2::ChronologyIndeterminate
    );
}

#[test]
fn cancel_acknowledgement_is_not_terminal_cancellation() {
    let intent = intent(false);
    let frontier = resolve_observed_execution_frontier_v1(
        &intent,
        &execution_scope(),
        &[],
        &[],
    )
    .expect("frontier");
    let projection = project_observed_market_order_v2(
        &intent,
        &projection_scope(),
        &[event(
            &intent,
            "evt-cancel-ack",
            MarketEventKindV1::CancelAcceptedObserved,
            None,
        )],
        &frontier,
    )
    .expect("projection");
    assert_eq!(
        projection.latest_observed_control(),
        &LatestObservedControlPostureV2::CancelAcknowledged
    );
}

#[test]
fn replace_acceptance_keeps_successor_unresolved() {
    let intent = intent(false);
    let frontier = resolve_observed_execution_frontier_v1(
        &intent,
        &execution_scope(),
        &[],
        &[],
    )
    .expect("frontier");
    let projection = project_observed_market_order_v2(
        &intent,
        &projection_scope(),
        &[event(
            &intent,
            "evt-replace-ack",
            MarketEventKindV1::ReplaceAcceptedObserved,
            None,
        )],
        &frontier,
    )
    .expect("projection");
    assert_eq!(
        projection.latest_observed_control(),
        &LatestObservedControlPostureV2::ReplacementAcceptedSuccessorUnresolved
    );
}

#[test]
fn notional_progress_never_derives_quantity_times_execution_price() {
    let intent = intent(true);
    let fills = vec![fill(&intent, "fill-a", "exec-1", 3)];
    let frontier = resolve_observed_execution_frontier_v1(
        &intent,
        &execution_scope(),
        &fills,
        &[],
    )
    .expect("frontier");
    let projection = project_observed_market_order_v2(
        &intent,
        &projection_scope(),
        &[],
        &frontier,
    )
    .expect("projection");
    assert_eq!(
        projection.target_progress(),
        &ObservedTargetProgressV2::NotionalTargetProgressIndeterminate {
            effective_observed_execution_count: 1,
        }
    );
}
