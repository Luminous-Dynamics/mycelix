#![forbid(unsafe_code)]

//! FIN-MKT-002B: deterministic projection over exact supplied control evidence and one
//! non-forgeable FIN-MKT-002B0 observed execution frontier.
//!
//! This crate never aggregates raw fill envelopes. FIN-MKT-002B0 owns provider-scoped economic
//! execution de-duplication and adjustment resolution. This layer combines that positive frontier
//! with control-event observations to produce an evidence-local projection only.

use mycelix_finance_market_core::{
    CanonicalMarketOrderIntentV1, Digest32, MarketExternalIdV1, MarketProfileRefV1,
    QuantitySpecV1,
};
use mycelix_finance_market_execution_frontier::{
    ExecutionFrontierResolutionV1, ObservedExecutionFrontierV1,
};
use mycelix_finance_market_observation::{
    validate_subject_against_intent_v1, CanonicalMarketEventObservationV1, MarketEventKindV1,
    MarketOrderObservationSubjectV1,
};
use serde::Serialize;
use std::collections::BTreeMap;
use std::fmt;

pub const MAX_CONTROL_EVENTS_V1: usize = 4096;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProjectionError {
    TooManyControlEvents,
    ExecutionFrontierIntentMismatch,
    ExecutionFrontierProviderMismatch,
    ObservationSubjectMismatch,
    EventProviderProfileMismatch,
    EventObservationProfileMismatch,
    ProviderOrderRefConflict,
    ObservationIdentityConflict,
    IncompatibleExecutionUnits,
    ArithmeticOverflow,
}

impl fmt::Display for ProjectionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::TooManyControlEvents => "market projection exceeds the V1 control-event bound",
            Self::ExecutionFrontierIntentMismatch => {
                "execution frontier belongs to a different immutable market-order intent"
            }
            Self::ExecutionFrontierProviderMismatch => {
                "execution frontier provider profile does not match the projection scope"
            }
            Self::ObservationSubjectMismatch => {
                "control observation is outside the projected immutable order lineage"
            }
            Self::EventProviderProfileMismatch => {
                "control observation provider profile does not match the projection scope"
            }
            Self::EventObservationProfileMismatch => {
                "control observation profile does not match the projection event role"
            }
            Self::ProviderOrderRefConflict => {
                "incompatible provider-order references occur across supplied evidence"
            }
            Self::ObservationIdentityConflict => {
                "one control-observation identity is reused for different semantics"
            }
            Self::IncompatibleExecutionUnits => {
                "effective execution units are not exactly comparable with the unit target"
            }
            Self::ArithmeticOverflow => "observed execution aggregation overflowed",
        };
        f.write_str(message)
    }
}

impl std::error::Error for ProjectionError {}

/// Caller-selected coordinates for control-event interpretation. This is not a positive theorem.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProjectionScopeV1 {
    pub provider_profile: MarketProfileRefV1,
    pub event_observation_profile: MarketProfileRefV1,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub enum LatestObservedControlPostureV1 {
    NoControlObservation,
    ChronologyIndeterminate,
    Submitted,
    Accepted,
    PendingNew,
    Working,
    PendingCancel,
    CancelAcknowledged,
    CancelRejected,
    PendingReplace,
    ReplacementAcceptedSuccessorUnresolved,
    ReplaceRejected,
    DoneForDay,
    Expired,
    SuspendedOrHalted,
    ProviderRejected,
    SubmissionOutcomeUnknown,
    OpaqueProviderStatus(MarketExternalIdV1),
    Conflicted,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum ObservedExecutionPostureV1 {
    NoExecutionObserved,
    ExecutionObserved,
    AdjustedExecutionObserved,
    AdjustmentResolutionIndeterminate,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum ObservedUnitTargetComparisonV1 {
    ObservedNone,
    ObservedBelowTarget,
    ObservedAtTarget,
    ObservedOverTargetConflict,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub enum ObservedTargetProgressV1 {
    Unit {
        target_atomic_units: u64,
        effective_observed_atomic_units: u64,
        /// Arithmetic gap inside this supplied evidence frontier only. It is not an
        /// authoritative provider residual unless a separate completeness theorem exists.
        unobserved_target_gap_under_this_frontier: u64,
        comparison: ObservedUnitTargetComparisonV1,
    },
    NotionalNoEffectiveExecutionObserved,
    NotionalTargetProgressIndeterminate {
        effective_observed_execution_count: u32,
    },
    AdjustmentResolutionIndeterminate,
}

/// Positive evidence-local projection. Fields are private and the type is not deserializable.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct MarketOrderObservedProjectionV1 {
    intent_commitment: Digest32,
    scope: ProjectionScopeV1,
    provider_order_ref_observed: Option<MarketExternalIdV1>,
    latest_observed_control: LatestObservedControlPostureV1,
    execution_posture: ObservedExecutionPostureV1,
    target_progress: ObservedTargetProgressV1,
    unique_event_count: u32,
    execution_frontier: ObservedExecutionFrontierV1,
}

impl MarketOrderObservedProjectionV1 {
    pub const fn intent_commitment(&self) -> Digest32 {
        self.intent_commitment
    }

    pub fn scope(&self) -> &ProjectionScopeV1 {
        &self.scope
    }

    pub fn provider_order_ref_observed(&self) -> Option<&MarketExternalIdV1> {
        self.provider_order_ref_observed.as_ref()
    }

    pub fn latest_observed_control(&self) -> &LatestObservedControlPostureV1 {
        &self.latest_observed_control
    }

    pub const fn execution_posture(&self) -> ObservedExecutionPostureV1 {
        self.execution_posture
    }

    pub fn target_progress(&self) -> &ObservedTargetProgressV1 {
        &self.target_progress
    }

    pub const fn unique_event_count(&self) -> u32 {
        self.unique_event_count
    }

    /// Preserve the exact positive execution theorem consumed by this projection.
    pub fn execution_frontier(&self) -> &ObservedExecutionFrontierV1 {
        &self.execution_frontier
    }
}

pub fn project_observed_market_order_v1(
    intent: &CanonicalMarketOrderIntentV1,
    scope: &ProjectionScopeV1,
    events: &[CanonicalMarketEventObservationV1],
    execution_frontier: &ObservedExecutionFrontierV1,
) -> Result<MarketOrderObservedProjectionV1, ProjectionError> {
    if events.len() > MAX_CONTROL_EVENTS_V1 {
        return Err(ProjectionError::TooManyControlEvents);
    }

    if execution_frontier.intent_commitment() != intent.commitment() {
        return Err(ProjectionError::ExecutionFrontierIntentMismatch);
    }
    if execution_frontier.scope().provider_profile != scope.provider_profile {
        return Err(ProjectionError::ExecutionFrontierProviderMismatch);
    }

    let provider_order_ref_observed = validate_event_scope(
        intent,
        scope,
        events,
        execution_frontier.provider_order_ref_observed().cloned(),
    )?;
    let unique_events = deduplicate_events(events)?;
    let latest_observed_control = project_control_posture(&unique_events);
    let (execution_posture, target_progress) =
        project_execution_progress(intent, execution_frontier)?;

    Ok(MarketOrderObservedProjectionV1 {
        intent_commitment: intent.commitment(),
        scope: scope.clone(),
        provider_order_ref_observed,
        latest_observed_control,
        execution_posture,
        target_progress,
        unique_event_count: u32::try_from(unique_events.len())
            .map_err(|_| ProjectionError::TooManyControlEvents)?,
        execution_frontier: execution_frontier.clone(),
    })
}

fn validate_event_scope(
    intent: &CanonicalMarketOrderIntentV1,
    scope: &ProjectionScopeV1,
    events: &[CanonicalMarketEventObservationV1],
    mut provider_order_ref: Option<MarketExternalIdV1>,
) -> Result<Option<MarketExternalIdV1>, ProjectionError> {
    for event in events {
        validate_subject_against_intent_v1(&event.input().subject, intent)
            .map_err(|_| ProjectionError::ObservationSubjectMismatch)?;

        if event.input().subject.provider_profile != scope.provider_profile {
            return Err(ProjectionError::EventProviderProfileMismatch);
        }
        if event.input().observation_ref.observation_profile != scope.event_observation_profile {
            return Err(ProjectionError::EventObservationProfileMismatch);
        }

        absorb_provider_order_ref(
            &event.input().subject,
            &mut provider_order_ref,
        )?;
    }

    Ok(provider_order_ref)
}

fn absorb_provider_order_ref(
    subject: &MarketOrderObservationSubjectV1,
    provider_order_ref: &mut Option<MarketExternalIdV1>,
) -> Result<(), ProjectionError> {
    if let Some(candidate) = subject.provider_order_ref.as_ref() {
        match provider_order_ref.as_ref() {
            None => *provider_order_ref = Some(candidate.clone()),
            Some(existing) if existing == candidate => {}
            Some(_) => return Err(ProjectionError::ProviderOrderRefConflict),
        }
    }
    Ok(())
}

fn deduplicate_events<'a>(
    events: &'a [CanonicalMarketEventObservationV1],
) -> Result<Vec<&'a CanonicalMarketEventObservationV1>, ProjectionError> {
    let mut by_identity: BTreeMap<Digest32, &'a CanonicalMarketEventObservationV1> =
        BTreeMap::new();

    for event in events {
        match by_identity.get(&event.identity_commitment()) {
            None => {
                by_identity.insert(event.identity_commitment(), event);
            }
            Some(existing) if existing.semantic_commitment() == event.semantic_commitment() => {}
            Some(_) => return Err(ProjectionError::ObservationIdentityConflict),
        }
    }

    Ok(by_identity.into_values().collect())
}

fn project_control_posture(
    events: &[&CanonicalMarketEventObservationV1],
) -> LatestObservedControlPostureV1 {
    if events.is_empty() {
        return LatestObservedControlPostureV1::NoControlObservation;
    }

    if events.len() == 1 {
        return map_event_kind(&events[0].input().event_kind);
    }

    let first_profile = match events[0].input().chronology.chronology_profile.as_ref() {
        Some(value) => value,
        None => return LatestObservedControlPostureV1::ChronologyIndeterminate,
    };

    let mut by_sequence: BTreeMap<u64, &CanonicalMarketEventObservationV1> = BTreeMap::new();

    for event in events {
        let chronology = &event.input().chronology;
        if chronology.chronology_profile.as_ref() != Some(first_profile) {
            return LatestObservedControlPostureV1::ChronologyIndeterminate;
        }
        let sequence = match chronology.sequence {
            Some(value) => value,
            None => return LatestObservedControlPostureV1::ChronologyIndeterminate,
        };
        if by_sequence.insert(sequence, event).is_some() {
            return LatestObservedControlPostureV1::Conflicted;
        }
    }

    let Some((_, latest)) = by_sequence.last_key_value() else {
        return LatestObservedControlPostureV1::NoControlObservation;
    };
    map_event_kind(&latest.input().event_kind)
}

fn map_event_kind(value: &MarketEventKindV1) -> LatestObservedControlPostureV1 {
    match value {
        MarketEventKindV1::SubmitAttemptObserved => LatestObservedControlPostureV1::Submitted,
        MarketEventKindV1::ProviderAcceptedObserved => LatestObservedControlPostureV1::Accepted,
        MarketEventKindV1::PendingNewObserved => LatestObservedControlPostureV1::PendingNew,
        MarketEventKindV1::WorkingObserved => LatestObservedControlPostureV1::Working,
        MarketEventKindV1::CancelRequestedObserved => LatestObservedControlPostureV1::PendingCancel,
        MarketEventKindV1::CancelAcceptedObserved => {
            LatestObservedControlPostureV1::CancelAcknowledged
        }
        MarketEventKindV1::CancelRejectedObserved => LatestObservedControlPostureV1::CancelRejected,
        MarketEventKindV1::ReplaceRequestedObserved => LatestObservedControlPostureV1::PendingReplace,
        MarketEventKindV1::ReplaceAcceptedObserved => {
            LatestObservedControlPostureV1::ReplacementAcceptedSuccessorUnresolved
        }
        MarketEventKindV1::ReplaceRejectedObserved => LatestObservedControlPostureV1::ReplaceRejected,
        MarketEventKindV1::DoneForDayObserved => LatestObservedControlPostureV1::DoneForDay,
        MarketEventKindV1::ExpiredObserved => LatestObservedControlPostureV1::Expired,
        MarketEventKindV1::SuspendedOrHaltedObserved => {
            LatestObservedControlPostureV1::SuspendedOrHalted
        }
        MarketEventKindV1::ProviderRejectedObserved => LatestObservedControlPostureV1::ProviderRejected,
        MarketEventKindV1::OpaqueProviderStatus(status) => {
            LatestObservedControlPostureV1::OpaqueProviderStatus(status.clone())
        }
        MarketEventKindV1::SubmissionOutcomeUnknownObserved => {
            LatestObservedControlPostureV1::SubmissionOutcomeUnknown
        }
    }
}

fn project_execution_progress(
    intent: &CanonicalMarketOrderIntentV1,
    execution_frontier: &ObservedExecutionFrontierV1,
) -> Result<(ObservedExecutionPostureV1, ObservedTargetProgressV1), ProjectionError> {
    match execution_frontier.resolution() {
        ExecutionFrontierResolutionV1::AdjustmentResolutionIndeterminate => Ok((
            ObservedExecutionPostureV1::AdjustmentResolutionIndeterminate,
            ObservedTargetProgressV1::AdjustmentResolutionIndeterminate,
        )),
        ExecutionFrontierResolutionV1::Resolved {
            had_adjustments,
            effective_executions,
        } => {
            let execution_posture = if *had_adjustments {
                ObservedExecutionPostureV1::AdjustedExecutionObserved
            } else if effective_executions.is_empty() {
                ObservedExecutionPostureV1::NoExecutionObserved
            } else {
                ObservedExecutionPostureV1::ExecutionObserved
            };

            let target_progress = match &intent.input().quantity {
                QuantitySpecV1::Units(target) => {
                    let mut observed = 0_u64;
                    for execution in effective_executions {
                        let quantity = execution.executed_quantity();
                        if quantity.unit_profile != target.unit_profile
                            || quantity.amount.asset() != target.amount.asset()
                        {
                            return Err(ProjectionError::IncompatibleExecutionUnits);
                        }
                        observed = observed
                            .checked_add(quantity.amount.atomic_units())
                            .ok_or(ProjectionError::ArithmeticOverflow)?;
                    }

                    let target_units = target.amount.atomic_units();
                    let (gap, comparison) = if observed == 0 {
                        (target_units, ObservedUnitTargetComparisonV1::ObservedNone)
                    } else if observed < target_units {
                        (
                            target_units - observed,
                            ObservedUnitTargetComparisonV1::ObservedBelowTarget,
                        )
                    } else if observed == target_units {
                        (0, ObservedUnitTargetComparisonV1::ObservedAtTarget)
                    } else {
                        (0, ObservedUnitTargetComparisonV1::ObservedOverTargetConflict)
                    };

                    ObservedTargetProgressV1::Unit {
                        target_atomic_units: target_units,
                        effective_observed_atomic_units: observed,
                        unobserved_target_gap_under_this_frontier: gap,
                        comparison,
                    }
                }
                QuantitySpecV1::Notional(_) => {
                    if effective_executions.is_empty() {
                        ObservedTargetProgressV1::NotionalNoEffectiveExecutionObserved
                    } else {
                        ObservedTargetProgressV1::NotionalTargetProgressIndeterminate {
                            effective_observed_execution_count: u32::try_from(
                                effective_executions.len(),
                            )
                            .map_err(|_| ProjectionError::ArithmeticOverflow)?,
                        }
                    }
                }
            };

            Ok((execution_posture, target_progress))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_finance_exact::{AssetAmount, AssetId};
    use mycelix_finance_market_core::{
        canonicalize_order_intent_v1, MarketIdempotencyRefV1, MarketInstrumentRefV1,
        MarketOrderIntentInputV1, MarketProtocolIdV1, MarketQuantityV1, MarketSideV1,
        MarketSubjectRefV1, OrderTermsV1, TimeInForceV1,
    };
    use mycelix_finance_market_execution_frontier::{
        resolve_observed_execution_frontier_v1, ExecutionFrontierScopeV1,
    };
    use mycelix_finance_market_observation::{
        canonicalize_event_observation_v1, canonicalize_fill_observation_v1,
        FillObservationInputV1, MarketEventObservationInputV1, ProviderChronologyV1,
        ProviderObservationRefV1,
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

    fn projection_scope() -> ProjectionScopeV1 {
        ProjectionScopeV1 {
            provider_profile: profile("provider.demo", 0x70),
            event_observation_profile: profile("provider.demo.event-observation-v1", 0x71),
        }
    }

    fn execution_scope() -> ExecutionFrontierScopeV1 {
        ExecutionFrontierScopeV1 {
            provider_profile: profile("provider.demo", 0x70),
            fill_observation_profile: profile("provider.demo.fill-observation-v1", 0x72),
            adjustment_observation_profile: profile("provider.demo.adjustment-observation-v1", 0x73),
        }
    }

    fn observation_subject(intent: &CanonicalMarketOrderIntentV1) -> MarketOrderObservationSubjectV1 {
        MarketOrderObservationSubjectV1 {
            intent_commitment: intent.commitment(),
            account_subject: intent.input().account_subject.clone(),
            instrument: intent.input().instrument.clone(),
            provider_profile: projection_scope().provider_profile,
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

    fn event(
        intent: &CanonicalMarketOrderIntentV1,
        id: &str,
        kind: MarketEventKindV1,
        sequence: Option<u64>,
        evidence_byte: u8,
    ) -> CanonicalMarketEventObservationV1 {
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

    fn fill(
        intent: &CanonicalMarketOrderIntentV1,
        observation_id: &str,
        execution_ref: &str,
        quantity: u64,
        evidence_byte: u8,
    ) -> mycelix_finance_market_observation::CanonicalFillObservationV1 {
        canonicalize_fill_observation_v1(FillObservationInputV1 {
            subject: observation_subject(intent),
            observation_ref: ProviderObservationRefV1 {
                observation_profile: execution_scope().fill_observation_profile,
                observation_id: MarketExternalIdV1::new(observation_id).unwrap(),
            },
            provider_execution_ref: MarketExternalIdV1::new(execution_ref).unwrap(),
            executed_quantity: unit_quantity(quantity),
            execution_price: mycelix_finance_market_core::MarketPriceV1 {
                pricing_profile: profile("price.usd-cents-per-share-demo", 0x44),
                quote_amount: AssetAmount::new(18_750, AssetId::new("USD.cent.demo").unwrap()),
            },
            venue_ref: None,
            chronology: chronology(Some(10)),
            source_evidence_commitment: digest(evidence_byte),
        })
        .unwrap()
    }

    fn frontier(
        intent: &CanonicalMarketOrderIntentV1,
        fills: &[mycelix_finance_market_observation::CanonicalFillObservationV1],
    ) -> ObservedExecutionFrontierV1 {
        resolve_observed_execution_frontier_v1(intent, &execution_scope(), fills, &[]).unwrap()
    }

    #[test]
    fn duplicate_envelopes_for_one_execution_project_once() {
        let intent = intent_with_quantity(QuantitySpecV1::Units(unit_quantity(100)));
        let fill_a = fill(&intent, "fill-envelope:a", "exec:one", 40, 0x91);
        let fill_b = fill(&intent, "fill-envelope:b", "exec:one", 40, 0x92);
        let execution_frontier = frontier(&intent, &[fill_a, fill_b]);

        assert_eq!(execution_frontier.unique_fill_observation_count(), 2);
        assert_eq!(execution_frontier.unique_execution_count(), 1);

        let projected = project_observed_market_order_v1(
            &intent,
            &projection_scope(),
            &[],
            &execution_frontier,
        )
        .unwrap();

        assert_eq!(
            projected.target_progress(),
            &ObservedTargetProgressV1::Unit {
                target_atomic_units: 100,
                effective_observed_atomic_units: 40,
                unobserved_target_gap_under_this_frontier: 60,
                comparison: ObservedUnitTargetComparisonV1::ObservedBelowTarget,
            }
        );
        assert_eq!(projected.execution_frontier().unique_execution_count(), 1);
    }

    #[test]
    fn distinct_execution_ids_with_same_economics_count_separately() {
        let intent = intent_with_quantity(QuantitySpecV1::Units(unit_quantity(100)));
        let fill_a = fill(&intent, "fill:a", "exec:a", 40, 0x91);
        let fill_b = fill(&intent, "fill:b", "exec:b", 40, 0x92);
        let execution_frontier = frontier(&intent, &[fill_a, fill_b]);

        let projected = project_observed_market_order_v1(
            &intent,
            &projection_scope(),
            &[],
            &execution_frontier,
        )
        .unwrap();

        assert_eq!(
            projected.target_progress(),
            &ObservedTargetProgressV1::Unit {
                target_atomic_units: 100,
                effective_observed_atomic_units: 80,
                unobserved_target_gap_under_this_frontier: 20,
                comparison: ObservedUnitTargetComparisonV1::ObservedBelowTarget,
            }
        );
    }

    #[test]
    fn notional_target_never_uses_generic_quantity_times_price_math() {
        let notional = MarketQuantityV1 {
            unit_profile: profile("notional.usd-cents-demo", 0x35),
            amount: AssetAmount::new(100_000, AssetId::new("USD.cent.demo").unwrap()),
        };
        let intent = intent_with_quantity(QuantitySpecV1::Notional(notional));
        let first_fill = fill(&intent, "fill:a", "exec:a", 40, 0x91);
        let execution_frontier = frontier(&intent, &[first_fill]);

        let projected = project_observed_market_order_v1(
            &intent,
            &projection_scope(),
            &[],
            &execution_frontier,
        )
        .unwrap();

        assert_eq!(
            projected.target_progress(),
            &ObservedTargetProgressV1::NotionalTargetProgressIndeterminate {
                effective_observed_execution_count: 1,
            }
        );
    }

    #[test]
    fn reversed_control_input_order_projects_same_latest_observed_event() {
        let intent = intent_with_quantity(QuantitySpecV1::Units(unit_quantity(100)));
        let execution_frontier = frontier(&intent, &[]);
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

        let first = project_observed_market_order_v1(
            &intent,
            &projection_scope(),
            &[working.clone(), cancel.clone()],
            &execution_frontier,
        )
        .unwrap();
        let second = project_observed_market_order_v1(
            &intent,
            &projection_scope(),
            &[cancel, working],
            &execution_frontier,
        )
        .unwrap();

        assert_eq!(first, second);
        assert_eq!(
            first.latest_observed_control(),
            &LatestObservedControlPostureV1::PendingCancel
        );
    }

    #[test]
    fn multiple_unsequenced_events_are_chronology_indeterminate() {
        let intent = intent_with_quantity(QuantitySpecV1::Units(unit_quantity(100)));
        let execution_frontier = frontier(&intent, &[]);
        let accepted = event(
            &intent,
            "event:accepted",
            MarketEventKindV1::ProviderAcceptedObserved,
            None,
            0x81,
        );
        let working = event(
            &intent,
            "event:working",
            MarketEventKindV1::WorkingObserved,
            None,
            0x82,
        );

        let projected = project_observed_market_order_v1(
            &intent,
            &projection_scope(),
            &[accepted, working],
            &execution_frontier,
        )
        .unwrap();

        assert_eq!(
            projected.latest_observed_control(),
            &LatestObservedControlPostureV1::ChronologyIndeterminate
        );
    }

    #[test]
    fn alternate_evidence_for_same_event_does_not_duplicate_control_transition() {
        let intent = intent_with_quantity(QuantitySpecV1::Units(unit_quantity(100)));
        let execution_frontier = frontier(&intent, &[]);
        let first = event(
            &intent,
            "event:working",
            MarketEventKindV1::WorkingObserved,
            None,
            0x81,
        );
        let second = event(
            &intent,
            "event:working",
            MarketEventKindV1::WorkingObserved,
            None,
            0x82,
        );

        let projected = project_observed_market_order_v1(
            &intent,
            &projection_scope(),
            &[first, second],
            &execution_frontier,
        )
        .unwrap();

        assert_eq!(projected.unique_event_count(), 1);
        assert_eq!(
            projected.latest_observed_control(),
            &LatestObservedControlPostureV1::Working
        );
    }

    #[test]
    fn projection_rejects_execution_frontier_from_another_provider_profile() {
        let intent = intent_with_quantity(QuantitySpecV1::Units(unit_quantity(100)));
        let execution_frontier = frontier(&intent, &[]);
        let mut scope = projection_scope();
        scope.provider_profile = profile("provider.other", 0x79);

        let error = project_observed_market_order_v1(
            &intent,
            &scope,
            &[],
            &execution_frontier,
        )
        .unwrap_err();

        assert_eq!(error, ProjectionError::ExecutionFrontierProviderMismatch);
    }
}
