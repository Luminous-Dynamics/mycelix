#![forbid(unsafe_code)]

//! FIN-MKT-002B V2: deterministic observed-frontier market-order projection.
//!
//! This crate consumes control-event observations plus a positive FIN-MKT-002B0 execution
//! frontier. It deliberately does not consume raw fill observations or adjustments.

use mycelix_finance_market_core::{
    CanonicalMarketOrderIntentV1, Digest32, MarketExternalIdV1, QuantitySpecV1,
};
use mycelix_finance_market_execution_frontier::{
    ExecutionFrontierResolutionV1, ExecutionFrontierScopeV1, ObservedExecutionFrontierV1,
};
use mycelix_finance_market_observation::{
    validate_subject_against_intent_v1, CanonicalMarketEventObservationV1, MarketEventKindV1,
};
use serde::Serialize;
use std::collections::BTreeMap;
use std::fmt;

pub const MAX_CONTROL_FRONTIER_ITEMS_V2: usize = 4096;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProjectionErrorV2 {
    ControlFrontierTooLarge,
    ObservationSubjectMismatch,
    ProviderProfileMismatch,
    EventObservationProfileMismatch,
    ExecutionFrontierIntentMismatch,
    ExecutionFrontierScopeMismatch,
    ProviderOrderRefConflict,
    EventObservationIdentityConflict,
    IncompatibleExecutionUnits,
    ArithmeticOverflow,
}

impl fmt::Display for ProjectionErrorV2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::ControlFrontierTooLarge => "market control-event frontier exceeds the V2 bound",
            Self::ObservationSubjectMismatch => {
                "control observation is outside the selected immutable order lineage"
            }
            Self::ProviderProfileMismatch => {
                "control observation provider profile does not match the projection scope"
            }
            Self::EventObservationProfileMismatch => {
                "control observation profile does not match the projection scope"
            }
            Self::ExecutionFrontierIntentMismatch => {
                "execution frontier is bound to a different immutable order intent"
            }
            Self::ExecutionFrontierScopeMismatch => {
                "execution frontier scope does not exactly match the projection scope"
            }
            Self::ProviderOrderRefConflict => {
                "control and execution evidence contain incompatible provider-order references"
            }
            Self::EventObservationIdentityConflict => {
                "one control observation identity is reused for different normalized semantics"
            }
            Self::IncompatibleExecutionUnits => {
                "effective execution units are not exactly comparable with the unit-target order"
            }
            Self::ArithmeticOverflow => "observed execution aggregation overflowed",
        };
        f.write_str(message)
    }
}

impl std::error::Error for ProjectionErrorV2 {}

/// Caller-supplied coordinates. This is not itself a positive theorem.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProjectionScopeV2 {
    pub event_observation_profile: mycelix_finance_market_core::MarketProfileRefV1,
    pub execution_frontier_scope: ExecutionFrontierScopeV1,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub enum LatestObservedControlPostureV2 {
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
pub enum ObservedExecutionPostureV2 {
    NoExecutionObserved,
    HistoricalExecutionOnlyObserved,
    EffectiveExecutionObserved,
    AdjustedEffectiveExecutionObserved,
    AdjustmentResolutionIndeterminate,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum ObservedUnitTargetComparisonV2 {
    ObservedNone,
    ObservedBelowTarget,
    ObservedAtTarget,
    ObservedOverTargetConflict,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub enum ObservedTargetProgressV2 {
    Unit {
        target_atomic_units: u64,
        effective_observed_atomic_units: u64,
        /// Arithmetic gap inside this supplied frontier only. This is not an authoritative
        /// residual quantity unless a separate completeness/currentness theorem exists.
        unobserved_target_gap_under_this_frontier: u64,
        comparison: ObservedUnitTargetComparisonV2,
    },
    NotionalNoEffectiveExecutionObserved,
    NotionalTargetProgressIndeterminate {
        effective_observed_execution_count: u32,
    },
    AdjustmentResolutionIndeterminate,
}

/// Positive projection result. Fields are private and this type is intentionally not
/// deserializable or publicly constructible.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct MarketOrderObservedProjectionV2 {
    intent_commitment: Digest32,
    scope: ProjectionScopeV2,
    provider_order_ref_observed: Option<MarketExternalIdV1>,
    latest_observed_control: LatestObservedControlPostureV2,
    execution_posture: ObservedExecutionPostureV2,
    target_progress: ObservedTargetProgressV2,
    unique_control_event_count: u32,
    unique_execution_identity_count: u32,
    unique_fill_observation_count: u32,
    unique_adjustment_count: u32,
}

impl MarketOrderObservedProjectionV2 {
    pub const fn intent_commitment(&self) -> Digest32 {
        self.intent_commitment
    }

    pub fn scope(&self) -> &ProjectionScopeV2 {
        &self.scope
    }

    pub fn provider_order_ref_observed(&self) -> Option<&MarketExternalIdV1> {
        self.provider_order_ref_observed.as_ref()
    }

    pub fn latest_observed_control(&self) -> &LatestObservedControlPostureV2 {
        &self.latest_observed_control
    }

    pub const fn execution_posture(&self) -> ObservedExecutionPostureV2 {
        self.execution_posture
    }

    pub fn target_progress(&self) -> &ObservedTargetProgressV2 {
        &self.target_progress
    }

    pub const fn unique_control_event_count(&self) -> u32 {
        self.unique_control_event_count
    }

    pub const fn unique_execution_identity_count(&self) -> u32 {
        self.unique_execution_identity_count
    }

    pub const fn unique_fill_observation_count(&self) -> u32 {
        self.unique_fill_observation_count
    }

    pub const fn unique_adjustment_count(&self) -> u32 {
        self.unique_adjustment_count
    }
}

pub fn project_observed_market_order_v2(
    intent: &CanonicalMarketOrderIntentV1,
    scope: &ProjectionScopeV2,
    events: &[CanonicalMarketEventObservationV1],
    execution_frontier: &ObservedExecutionFrontierV1,
) -> Result<MarketOrderObservedProjectionV2, ProjectionErrorV2> {
    if events.len() > MAX_CONTROL_FRONTIER_ITEMS_V2 {
        return Err(ProjectionErrorV2::ControlFrontierTooLarge);
    }
    if execution_frontier.intent_commitment() != intent.commitment() {
        return Err(ProjectionErrorV2::ExecutionFrontierIntentMismatch);
    }
    if execution_frontier.scope() != &scope.execution_frontier_scope {
        return Err(ProjectionErrorV2::ExecutionFrontierScopeMismatch);
    }

    let mut provider_order_ref = execution_frontier.provider_order_ref_observed().cloned();
    validate_control_scope(intent, scope, events, &mut provider_order_ref)?;
    let unique_events = deduplicate_events(events)?;
    let latest_observed_control = project_control_posture(&unique_events);
    let (execution_posture, target_progress) =
        project_execution_progress(intent, execution_frontier)?;

    Ok(MarketOrderObservedProjectionV2 {
        intent_commitment: intent.commitment(),
        scope: scope.clone(),
        provider_order_ref_observed: provider_order_ref,
        latest_observed_control,
        execution_posture,
        target_progress,
        unique_control_event_count: u32::try_from(unique_events.len())
            .map_err(|_| ProjectionErrorV2::ControlFrontierTooLarge)?,
        unique_execution_identity_count: execution_frontier.unique_execution_count(),
        unique_fill_observation_count: execution_frontier.unique_fill_observation_count(),
        unique_adjustment_count: execution_frontier.unique_adjustment_count(),
    })
}

fn validate_control_scope(
    intent: &CanonicalMarketOrderIntentV1,
    scope: &ProjectionScopeV2,
    events: &[CanonicalMarketEventObservationV1],
    provider_order_ref: &mut Option<MarketExternalIdV1>,
) -> Result<(), ProjectionErrorV2> {
    for event in events {
        validate_subject_against_intent_v1(&event.input().subject, intent)
            .map_err(|_| ProjectionErrorV2::ObservationSubjectMismatch)?;
        if event.input().subject.provider_profile
            != scope.execution_frontier_scope.provider_profile
        {
            return Err(ProjectionErrorV2::ProviderProfileMismatch);
        }
        if event.input().observation_ref.observation_profile != scope.event_observation_profile {
            return Err(ProjectionErrorV2::EventObservationProfileMismatch);
        }
        absorb_provider_order_ref(
            provider_order_ref,
            event.input().subject.provider_order_ref.as_ref(),
        )?;
    }
    Ok(())
}

fn absorb_provider_order_ref(
    accumulated: &mut Option<MarketExternalIdV1>,
    candidate: Option<&MarketExternalIdV1>,
) -> Result<(), ProjectionErrorV2> {
    let Some(candidate) = candidate else {
        return Ok(());
    };
    match accumulated.as_ref() {
        None => {
            *accumulated = Some(candidate.clone());
            Ok(())
        }
        Some(existing) if existing == candidate => Ok(()),
        Some(_) => Err(ProjectionErrorV2::ProviderOrderRefConflict),
    }
}

fn deduplicate_events(
    events: &[CanonicalMarketEventObservationV1],
) -> Result<Vec<&CanonicalMarketEventObservationV1>, ProjectionErrorV2> {
    let mut by_identity = BTreeMap::<Digest32, &CanonicalMarketEventObservationV1>::new();
    for event in events {
        match by_identity.get(&event.identity_commitment()) {
            None => {
                by_identity.insert(event.identity_commitment(), event);
            }
            Some(existing) if existing.semantic_commitment() == event.semantic_commitment() => {}
            Some(_) => return Err(ProjectionErrorV2::EventObservationIdentityConflict),
        }
    }
    Ok(by_identity.into_values().collect())
}

fn project_control_posture(
    events: &[&CanonicalMarketEventObservationV1],
) -> LatestObservedControlPostureV2 {
    if events.is_empty() {
        return LatestObservedControlPostureV2::NoControlObservation;
    }
    if events.len() == 1 {
        return map_event_kind(&events[0].input().event_kind);
    }

    let Some(first_profile) = events[0].input().chronology.chronology_profile.as_ref() else {
        return LatestObservedControlPostureV2::ChronologyIndeterminate;
    };
    let mut by_sequence = BTreeMap::<u64, &CanonicalMarketEventObservationV1>::new();
    for event in events {
        let chronology = &event.input().chronology;
        if chronology.chronology_profile.as_ref() != Some(first_profile) {
            return LatestObservedControlPostureV2::ChronologyIndeterminate;
        }
        let Some(sequence) = chronology.sequence else {
            return LatestObservedControlPostureV2::ChronologyIndeterminate;
        };
        if by_sequence.insert(sequence, event).is_some() {
            return LatestObservedControlPostureV2::Conflicted;
        }
    }

    match by_sequence.last_key_value() {
        Some((_, latest)) => map_event_kind(&latest.input().event_kind),
        None => LatestObservedControlPostureV2::NoControlObservation,
    }
}

fn map_event_kind(value: &MarketEventKindV1) -> LatestObservedControlPostureV2 {
    match value {
        MarketEventKindV1::SubmitAttemptObserved => LatestObservedControlPostureV2::Submitted,
        MarketEventKindV1::ProviderAcceptedObserved => LatestObservedControlPostureV2::Accepted,
        MarketEventKindV1::PendingNewObserved => LatestObservedControlPostureV2::PendingNew,
        MarketEventKindV1::WorkingObserved => LatestObservedControlPostureV2::Working,
        MarketEventKindV1::CancelRequestedObserved => LatestObservedControlPostureV2::PendingCancel,
        MarketEventKindV1::CancelAcceptedObserved => {
            LatestObservedControlPostureV2::CancelAcknowledged
        }
        MarketEventKindV1::CancelRejectedObserved => LatestObservedControlPostureV2::CancelRejected,
        MarketEventKindV1::ReplaceRequestedObserved => LatestObservedControlPostureV2::PendingReplace,
        MarketEventKindV1::ReplaceAcceptedObserved => {
            LatestObservedControlPostureV2::ReplacementAcceptedSuccessorUnresolved
        }
        MarketEventKindV1::ReplaceRejectedObserved => LatestObservedControlPostureV2::ReplaceRejected,
        MarketEventKindV1::DoneForDayObserved => LatestObservedControlPostureV2::DoneForDay,
        MarketEventKindV1::ExpiredObserved => LatestObservedControlPostureV2::Expired,
        MarketEventKindV1::SuspendedOrHaltedObserved => {
            LatestObservedControlPostureV2::SuspendedOrHalted
        }
        MarketEventKindV1::ProviderRejectedObserved => LatestObservedControlPostureV2::ProviderRejected,
        MarketEventKindV1::SubmissionOutcomeUnknownObserved => {
            LatestObservedControlPostureV2::SubmissionOutcomeUnknown
        }
        MarketEventKindV1::OpaqueProviderStatus(status) => {
            LatestObservedControlPostureV2::OpaqueProviderStatus(status.clone())
        }
    }
}

fn project_execution_progress(
    intent: &CanonicalMarketOrderIntentV1,
    execution_frontier: &ObservedExecutionFrontierV1,
) -> Result<(ObservedExecutionPostureV2, ObservedTargetProgressV2), ProjectionErrorV2> {
    let (had_adjustments, effective_executions) = match execution_frontier.resolution() {
        ExecutionFrontierResolutionV1::AdjustmentResolutionIndeterminate => {
            return Ok((
                ObservedExecutionPostureV2::AdjustmentResolutionIndeterminate,
                ObservedTargetProgressV2::AdjustmentResolutionIndeterminate,
            ));
        }
        ExecutionFrontierResolutionV1::Resolved {
            had_adjustments,
            effective_executions,
        } => (*had_adjustments, effective_executions.as_slice()),
    };

    let execution_posture = if effective_executions.is_empty() {
        if execution_frontier.unique_execution_count() == 0 {
            ObservedExecutionPostureV2::NoExecutionObserved
        } else {
            ObservedExecutionPostureV2::HistoricalExecutionOnlyObserved
        }
    } else if had_adjustments {
        ObservedExecutionPostureV2::AdjustedEffectiveExecutionObserved
    } else {
        ObservedExecutionPostureV2::EffectiveExecutionObserved
    };

    let progress = match &intent.input().quantity {
        QuantitySpecV1::Units(target) => {
            let mut observed = 0_u64;
            for execution in effective_executions {
                let quantity = execution.executed_quantity();
                if quantity.unit_profile != target.unit_profile
                    || quantity.amount.asset() != target.amount.asset()
                {
                    return Err(ProjectionErrorV2::IncompatibleExecutionUnits);
                }
                observed = observed
                    .checked_add(quantity.amount.atomic_units())
                    .ok_or(ProjectionErrorV2::ArithmeticOverflow)?;
            }

            let target_units = target.amount.atomic_units();
            let comparison = if observed == 0 {
                ObservedUnitTargetComparisonV2::ObservedNone
            } else if observed < target_units {
                ObservedUnitTargetComparisonV2::ObservedBelowTarget
            } else if observed == target_units {
                ObservedUnitTargetComparisonV2::ObservedAtTarget
            } else {
                ObservedUnitTargetComparisonV2::ObservedOverTargetConflict
            };
            ObservedTargetProgressV2::Unit {
                target_atomic_units: target_units,
                effective_observed_atomic_units: observed,
                unobserved_target_gap_under_this_frontier: target_units.saturating_sub(observed),
                comparison,
            }
        }
        QuantitySpecV1::Notional(_) => {
            if effective_executions.is_empty() {
                ObservedTargetProgressV2::NotionalNoEffectiveExecutionObserved
            } else {
                ObservedTargetProgressV2::NotionalTargetProgressIndeterminate {
                    effective_observed_execution_count: u32::try_from(effective_executions.len())
                        .map_err(|_| ProjectionErrorV2::ArithmeticOverflow)?,
                }
            }
        }
    };

    Ok((execution_posture, progress))
}
