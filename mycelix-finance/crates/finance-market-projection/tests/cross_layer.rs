use mycelix_finance_exact::{AssetAmount, AssetId};
use mycelix_finance_market_core::{
    canonicalize_order_intent_v1, CanonicalMarketOrderIntentV1, Digest32, MarketExternalIdV1,
    MarketIdempotencyRefV1, MarketInstrumentRefV1, MarketOrderIntentInputV1, MarketPriceV1,
    MarketProfileRefV1, MarketProtocolIdV1, MarketQuantityV1, MarketSideV1, MarketSubjectRefV1,
    OrderTermsV1, QuantitySpecV1, TimeInForceV1,
};
use mycelix_finance_market_execution_frontier::{
    resolve_observed_execution_frontier_v1, ExecutionFrontierScopeV1,
};
use mycelix_finance_market_observation::{
    canonicalize_event_observation_v1, canonicalize_fill_adjustment_v1,
    canonicalize_fill_observation_v1, CanonicalFillObservationV1, FillAdjustmentKindV1,
    FillAdjustmentObservationInputV1, FillObservationInputV1, MarketEventKindV1,
    MarketEventObservationInputV1, MarketOrderObservationSubjectV1, ProviderChronologyV1,
    ProviderObservationRefV1,
};
use mycelix_finance_market_projection::{
    project_observed_market_order_v2, LatestObservedControlPostureV2,
    ObservedExecutionPostureV2, ObservedTargetProgressV2, ObservedUnitTargetComparisonV2,
    ProjectionScopeV2,
};

fn digest(byte: u8) -> Digest32 {
    Digest32::from_bytes([byte; 32])
}

fn profile(id: &str, byte: u8) -> MarketProfileRefV1 {
    MarketProfileRefV1 {
        profile_id: MarketProtocolIdV1::new(id).unwrap(),
        revision: 1,
        digest: digest(byte),
    }
}

fn subject(profile_id: &str, id: &str, byte: u8) -> MarketSubjectRefV1 {
    MarketSubjectRefV1 {
        subject_profile: profile(profile_id, byte),
        subject_id: MarketExternalIdV1::new(id).unwrap(),
    }
}

fn instrument() -> MarketInstrumentRefV1 {
    MarketInstrumentRefV1 {
        instrument_profile: profile("instrument.listed-equity.us", 0x22),
        instrument_id: MarketExternalIdV1::new("instrument:demo:AAPL").unwrap(),
    }
}

fn unit_quantity(value: u64) -> MarketQuantityV1 {
    MarketQuantityV1 {
        unit_profile: profile("quantity.micro-share-demo", 0x33),
        amount: AssetAmount::new(value, AssetId::new("AAPL.share.micro.demo").unwrap()),
    }
}

fn intent_with_quantity(quantity: QuantitySpecV1) -> CanonicalMarketOrderIntentV1 {
    canonicalize_order_intent_v1(MarketOrderIntentInputV1 {
        intent_subject: subject("mycelix.market.intent", "intent:demo:001", 0x10),
        account_subject: subject("mycelix.finance.account", "account:demo:001", 0x11),
        instrument: instrument(),
        side: MarketSideV1::AcquireLong,
        quantity,
        order_terms: OrderTermsV1::Market,
        time_in_force: TimeInForceV1::Day,
        execution_profile: profile("execution.market-order-demo", 0x55),
        semantic_idempotency: MarketIdempotencyRefV1 {
            idempotency_profile: profile("idempotency.market-order-v1", 0x66),
            semantic_id: MarketProtocolIdV1::new("idem:demo:001").unwrap(),
        },
        upstream_economic_effect_commitment: None,
    })
    .unwrap()
}

fn execution_scope() -> ExecutionFrontierScopeV1 {
    ExecutionFrontierScopeV1 {
        provider_profile: profile("provider.demo", 0x70),
        fill_observation_profile: profile("provider.demo.fill-observation-v1", 0x72),
        adjustment_observation_profile: profile("provider.demo.adjustment-observation-v1", 0x73),
    }
}

fn projection_scope() -> ProjectionScopeV2 {
    ProjectionScopeV2 {
        event_observation_profile: profile("provider.demo.event-observation-v1", 0x71),
        execution_frontier_scope: execution_scope(),
    }
}

fn observation_subject(intent: &CanonicalMarketOrderIntentV1) -> MarketOrderObservationSubjectV1 {
    MarketOrderObservationSubjectV1 {
        intent_commitment: intent.commitment(),
        account_subject: intent.input().account_subject.clone(),
        instrument: intent.input().instrument.clone(),
        provider_profile: execution_scope().provider_profile,
        provider_order_ref: Some(MarketExternalIdV1::new("provider-order:001").unwrap()),
    }
}

fn chronology(sequence: Option<u64>) -> ProviderChronologyV1 {
    match sequence {
        None => ProviderChronologyV1 {
            chronology_profile: None,
            sequence: None,
            provider_time: None,
        },
        Some(value) => ProviderChronologyV1 {
            chronology_profile: Some(profile("provider.demo.sequence-v1", 0x74)),
            sequence: Some(value),
            provider_time: None,
        },
    }
}

fn fill(
    intent: &CanonicalMarketOrderIntentV1,
    id: &str,
    execution_ref: &str,
    quantity: u64,
    evidence_byte: u8,
) -> CanonicalFillObservationV1 {
    canonicalize_fill_observation_v1(FillObservationInputV1 {
        subject: observation_subject(intent),
        observation_ref: ProviderObservationRefV1 {
            observation_profile: execution_scope().fill_observation_profile,
            observation_id: MarketExternalIdV1::new(id).unwrap(),
        },
        provider_execution_ref: MarketExternalIdV1::new(execution_ref).unwrap(),
        executed_quantity: unit_quantity(quantity),
        execution_price: MarketPriceV1 {
            pricing_profile: profile("price.usd-cents-per-share-demo", 0x44),
            quote_amount: AssetAmount::new(18_750, AssetId::new("USD.cent.demo").unwrap()),
        },
        venue_ref: None,
        chronology: chronology(Some(10)),
        source_evidence_commitment: digest(evidence_byte),
    })
    .unwrap()
}

fn event(
    intent: &CanonicalMarketOrderIntentV1,
    id: &str,
    kind: MarketEventKindV1,
    sequence: Option<u64>,
    evidence_byte: u8,
) -> mycelix_finance_market_observation::CanonicalMarketEventObservationV1 {
    canonicalize_event_observation_v1(MarketEventObservationInputV1 {
        subject: observation_subject(intent),
        observation_ref: ProviderObservationRefV1 {
            observation_profile: projection_scope().event_observation_profile,
            observation_id: MarketExternalIdV1::new(id).unwrap(),
        },
        event_kind: kind,
        chronology: chronology(sequence),
        source_evidence_commitment: digest(evidence_byte),
    })
    .unwrap()
}

#[test]
fn duplicate_observation_envelopes_for_one_execution_project_40_not_80() {
    let intent = intent_with_quantity(QuantitySpecV1::Units(unit_quantity(100)));
    let first = fill(&intent, "fill-envelope:1", "exec:1", 40, 0x91);
    let second = fill(&intent, "fill-envelope:2", "exec:1", 40, 0x92);

    let frontier = resolve_observed_execution_frontier_v1(
        &intent,
        &execution_scope(),
        &[first, second],
        &[],
    )
    .unwrap();
    assert_eq!(frontier.unique_fill_observation_count(), 2);
    assert_eq!(frontier.unique_execution_count(), 1);

    let projected = project_observed_market_order_v2(&intent, &projection_scope(), &[], &frontier)
        .unwrap();

    assert_eq!(
        projected.execution_posture(),
        ObservedExecutionPostureV2::EffectiveExecutionObserved
    );
    assert_eq!(
        projected.target_progress(),
        &ObservedTargetProgressV2::Unit {
            target_atomic_units: 100,
            effective_observed_atomic_units: 40,
            unobserved_target_gap_under_this_frontier: 60,
            comparison: ObservedUnitTargetComparisonV2::ObservedBelowTarget,
        }
    );
}

#[test]
fn bust_preserves_historical_execution_while_effective_progress_returns_to_zero() {
    let intent = intent_with_quantity(QuantitySpecV1::Units(unit_quantity(100)));
    let first = fill(&intent, "fill-envelope:1", "exec:1", 40, 0x91);
    let adjustment = canonicalize_fill_adjustment_v1(FillAdjustmentObservationInputV1 {
        subject: observation_subject(&intent),
        observation_ref: ProviderObservationRefV1 {
            observation_profile: execution_scope().adjustment_observation_profile,
            observation_id: MarketExternalIdV1::new("adjust:bust:1").unwrap(),
        },
        adjustment_kind: FillAdjustmentKindV1::Bust {
            prior_fill_commitment: first.semantic_commitment(),
        },
        chronology: chronology(Some(20)),
        source_evidence_commitment: digest(0x93),
    })
    .unwrap();

    let frontier = resolve_observed_execution_frontier_v1(
        &intent,
        &execution_scope(),
        &[first],
        &[adjustment],
    )
    .unwrap();
    assert_eq!(frontier.unique_execution_count(), 1);

    let projected = project_observed_market_order_v2(&intent, &projection_scope(), &[], &frontier)
        .unwrap();

    assert_eq!(
        projected.execution_posture(),
        ObservedExecutionPostureV2::HistoricalExecutionOnlyObserved
    );
    assert_eq!(
        projected.target_progress(),
        &ObservedTargetProgressV2::Unit {
            target_atomic_units: 100,
            effective_observed_atomic_units: 0,
            unobserved_target_gap_under_this_frontier: 100,
            comparison: ObservedUnitTargetComparisonV2::ObservedNone,
        }
    );
}

#[test]
fn chronology_projection_is_invariant_to_input_vector_order() {
    let intent = intent_with_quantity(QuantitySpecV1::Units(unit_quantity(100)));
    let frontier = resolve_observed_execution_frontier_v1(&intent, &execution_scope(), &[], &[])
        .unwrap();
    let working = event(
        &intent,
        "event:working",
        MarketEventKindV1::WorkingObserved,
        Some(10),
        0x81,
    );
    let cancel = event(
        &intent,
        "event:cancel",
        MarketEventKindV1::CancelRequestedObserved,
        Some(11),
        0x82,
    );

    let first = project_observed_market_order_v2(
        &intent,
        &projection_scope(),
        &[working.clone(), cancel.clone()],
        &frontier,
    )
    .unwrap();
    let second = project_observed_market_order_v2(
        &intent,
        &projection_scope(),
        &[cancel, working],
        &frontier,
    )
    .unwrap();

    assert_eq!(first.latest_observed_control(), second.latest_observed_control());
    assert_eq!(
        first.latest_observed_control(),
        &LatestObservedControlPostureV2::PendingCancel
    );
}

#[test]
fn notional_progress_remains_indeterminate_after_effective_execution() {
    let notional = MarketQuantityV1 {
        unit_profile: profile("notional.usd-cents-demo", 0x35),
        amount: AssetAmount::new(100_000, AssetId::new("USD.cent.demo").unwrap()),
    };
    let intent = intent_with_quantity(QuantitySpecV1::Notional(notional));
    let first = fill(&intent, "fill-envelope:1", "exec:1", 40, 0x91);
    let frontier = resolve_observed_execution_frontier_v1(
        &intent,
        &execution_scope(),
        &[first],
        &[],
    )
    .unwrap();

    let projected = project_observed_market_order_v2(&intent, &projection_scope(), &[], &frontier)
        .unwrap();

    assert_eq!(
        projected.target_progress(),
        &ObservedTargetProgressV2::NotionalTargetProgressIndeterminate {
            effective_observed_execution_count: 1,
        }
    );
}
