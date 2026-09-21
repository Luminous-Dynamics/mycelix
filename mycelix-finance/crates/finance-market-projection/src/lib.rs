#![forbid(unsafe_code)]

//! FIN-MKT-002B: deterministic projection over an exact supplied observation frontier.
//!
//! This crate projects only what is observed inside the caller-supplied frontier.
//! It does not establish that the frontier is complete/current, that the provider's
//! state is current, that an order settled, or that any financial action was authorized.

use mycelix_finance_market_core::{
    CanonicalMarketOrderIntentV1, Digest32, MarketExternalIdV1, MarketProfileRefV1,
    QuantitySpecV1,
};
use mycelix_finance_market_observation::{
    validate_subject_against_intent_v1, CanonicalFillAdjustmentObservationV1,
    CanonicalFillObservationV1, CanonicalMarketEventObservationV1, FillAdjustmentKindV1,
    MarketEventKindV1, MarketOrderObservationSubjectV1,
};
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const MAX_FRONTIER_ITEMS_V1: usize = 4096;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProjectionError {
    FrontierTooLarge,
    ObservationSubjectMismatch,
    ProviderProfileMismatch,
    ObservationProfileMismatch,
    ProviderOrderRefConflict,
    ObservationIdentityConflict,
    FillSemanticCommitmentConflict,
    AdjustmentIdentityConflict,
    AdjustmentTopologyConflict,
    AdjustmentCycle,
    IncompatibleFillUnits,
    ArithmeticOverflow,
}

impl fmt::Display for ProjectionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::FrontierTooLarge => "market observation frontier exceeds the V1 bound",
            Self::ObservationSubjectMismatch => {
                "observation subject is outside the projected market-order lineage"
            }
            Self::ProviderProfileMismatch => {
                "observation provider profile does not match the projection scope"
            }
            Self::ObservationProfileMismatch => {
                "observation profile does not match the projection scope role"
            }
            Self::ProviderOrderRefConflict => {
                "multiple incompatible provider-order references occur in one immutable intent frontier"
            }
            Self::ObservationIdentityConflict => {
                "one observation identity is reused for different normalized semantics"
            }
            Self::FillSemanticCommitmentConflict => {
                "one fill semantic commitment resolves to incompatible fill observations"
            }
            Self::AdjustmentIdentityConflict => {
                "one adjustment identity is reused for different normalized semantics"
            }
            Self::AdjustmentTopologyConflict => {
                "fill correction/bust topology is conflicting or references an incompatible relation"
            }
            Self::AdjustmentCycle => "fill correction topology contains a cycle",
            Self::IncompatibleFillUnits => {
                "effective fill units are not exactly comparable with the unit-target order"
            }
            Self::ArithmeticOverflow => "observed execution aggregation overflowed",
        };
        f.write_str(message)
    }
}

impl std::error::Error for ProjectionError {}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProjectionScopeV1 {
    pub provider_profile: MarketProfileRefV1,
    pub event_observation_profile: MarketProfileRefV1,
    pub fill_observation_profile: MarketProfileRefV1,
    pub adjustment_observation_profile: MarketProfileRefV1,
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
        /// Arithmetic gap inside this supplied frontier only. This is NOT an
        /// authoritative residual quantity unless a separate completeness theorem exists.
        unobserved_target_gap_under_this_frontier: u64,
        comparison: ObservedUnitTargetComparisonV1,
    },
    NotionalNoEffectiveFillObserved,
    NotionalTargetProgressIndeterminate {
        effective_observed_fill_count: u32,
    },
    AdjustmentResolutionIndeterminate,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct MarketOrderObservedProjectionV1 {
    pub provider_order_ref_observed: Option<MarketExternalIdV1>,
    pub latest_observed_control: LatestObservedControlPostureV1,
    pub execution_posture: ObservedExecutionPostureV1,
    pub target_progress: ObservedTargetProgressV1,
    pub unique_event_count: u32,
    pub unique_fill_count: u32,
    pub unique_adjustment_count: u32,
}

#[derive(Clone, Copy)]
enum AdjustmentEdge {
    Correction(Digest32),
    Bust,
}

pub fn project_observed_market_order_v1(
    intent: &CanonicalMarketOrderIntentV1,
    scope: &ProjectionScopeV1,
    events: &[CanonicalMarketEventObservationV1],
    fills: &[CanonicalFillObservationV1],
    adjustments: &[CanonicalFillAdjustmentObservationV1],
) -> Result<MarketOrderObservedProjectionV1, ProjectionError> {
    let total = events
        .len()
        .checked_add(fills.len())
        .and_then(|value| value.checked_add(adjustments.len()))
        .ok_or(ProjectionError::FrontierTooLarge)?;
    if total > MAX_FRONTIER_ITEMS_V1 {
        return Err(ProjectionError::FrontierTooLarge);
    }

    let provider_order_ref_observed =
        validate_frontier_scope(intent, scope, events, fills, adjustments)?;

    let unique_events = deduplicate_events(events)?;
    let unique_fills = deduplicate_fills(fills)?;
    let unique_adjustments = deduplicate_adjustments(adjustments)?;

    let latest_observed_control = project_control_posture(&unique_events);
    let (execution_posture, target_progress) =
        project_execution_progress(intent, &unique_fills, &unique_adjustments)?;

    Ok(MarketOrderObservedProjectionV1 {
        provider_order_ref_observed,
        latest_observed_control,
        execution_posture,
        target_progress,
        unique_event_count: u32::try_from(unique_events.len())
            .map_err(|_| ProjectionError::FrontierTooLarge)?,
        unique_fill_count: u32::try_from(unique_fills.len())
            .map_err(|_| ProjectionError::FrontierTooLarge)?,
        unique_adjustment_count: u32::try_from(unique_adjustments.len())
            .map_err(|_| ProjectionError::FrontierTooLarge)?,
    })
}

fn validate_frontier_scope(
    intent: &CanonicalMarketOrderIntentV1,
    scope: &ProjectionScopeV1,
    events: &[CanonicalMarketEventObservationV1],
    fills: &[CanonicalFillObservationV1],
    adjustments: &[CanonicalFillAdjustmentObservationV1],
) -> Result<Option<MarketExternalIdV1>, ProjectionError> {
    let mut provider_order_ref: Option<MarketExternalIdV1> = None;

    for event in events {
        validate_subject_scope(
            &event.input().subject,
            intent,
            &scope.provider_profile,
            &mut provider_order_ref,
        )?;
        if event.input().observation_ref.observation_profile != scope.event_observation_profile {
            return Err(ProjectionError::ObservationProfileMismatch);
        }
    }

    for fill in fills {
        validate_subject_scope(
            &fill.input().subject,
            intent,
            &scope.provider_profile,
            &mut provider_order_ref,
        )?;
        if fill.input().observation_ref.observation_profile != scope.fill_observation_profile {
            return Err(ProjectionError::ObservationProfileMismatch);
        }
    }

    for adjustment in adjustments {
        validate_subject_scope(
            &adjustment.input().subject,
            intent,
            &scope.provider_profile,
            &mut provider_order_ref,
        )?;
        if adjustment.input().observation_ref.observation_profile
            != scope.adjustment_observation_profile
        {
            return Err(ProjectionError::ObservationProfileMismatch);
        }
    }

    Ok(provider_order_ref)
}

fn validate_subject_scope(
    subject: &MarketOrderObservationSubjectV1,
    intent: &CanonicalMarketOrderIntentV1,
    provider_profile: &MarketProfileRefV1,
    provider_order_ref: &mut Option<MarketExternalIdV1>,
) -> Result<(), ProjectionError> {
    validate_subject_against_intent_v1(subject, intent)
        .map_err(|_| ProjectionError::ObservationSubjectMismatch)?;

    if &subject.provider_profile != provider_profile {
        return Err(ProjectionError::ProviderProfileMismatch);
    }

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

fn deduplicate_fills<'a>(
    fills: &'a [CanonicalFillObservationV1],
) -> Result<Vec<&'a CanonicalFillObservationV1>, ProjectionError> {
    let mut by_identity: BTreeMap<Digest32, &'a CanonicalFillObservationV1> = BTreeMap::new();
    let mut by_semantic: BTreeMap<Digest32, Digest32> = BTreeMap::new();

    for fill in fills {
        match by_identity.get(&fill.identity_commitment()) {
            None => {
                by_identity.insert(fill.identity_commitment(), fill);
            }
            Some(existing) if existing.semantic_commitment() == fill.semantic_commitment() => {}
            Some(_) => return Err(ProjectionError::ObservationIdentityConflict),
        }

        match by_semantic.get(&fill.semantic_commitment()) {
            None => {
                by_semantic.insert(fill.semantic_commitment(), fill.identity_commitment());
            }
            Some(existing_identity) if *existing_identity == fill.identity_commitment() => {}
            Some(_) => return Err(ProjectionError::FillSemanticCommitmentConflict),
        }
    }

    Ok(by_identity.into_values().collect())
}

fn deduplicate_adjustments<'a>(
    adjustments: &'a [CanonicalFillAdjustmentObservationV1],
) -> Result<Vec<&'a CanonicalFillAdjustmentObservationV1>, ProjectionError> {
    let mut by_identity: BTreeMap<Digest32, &'a CanonicalFillAdjustmentObservationV1> =
        BTreeMap::new();

    for adjustment in adjustments {
        match by_identity.get(&adjustment.identity_commitment()) {
            None => {
                by_identity.insert(adjustment.identity_commitment(), adjustment);
            }
            Some(existing)
                if existing.semantic_commitment() == adjustment.semantic_commitment() => {}
            Some(_) => return Err(ProjectionError::AdjustmentIdentityConflict),
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
        MarketEventKindV1::CancelRejectedObserved => {
            LatestObservedControlPostureV1::CancelRejected
        }
        MarketEventKindV1::ReplaceRequestedObserved => {
            LatestObservedControlPostureV1::PendingReplace
        }
        MarketEventKindV1::ReplaceAcceptedObserved => {
            LatestObservedControlPostureV1::ReplacementAcceptedSuccessorUnresolved
        }
        MarketEventKindV1::ReplaceRejectedObserved => {
            LatestObservedControlPostureV1::ReplaceRejected
        }
        MarketEventKindV1::DoneForDayObserved => LatestObservedControlPostureV1::DoneForDay,
        MarketEventKindV1::ExpiredObserved => LatestObservedControlPostureV1::Expired,
        MarketEventKindV1::SuspendedOrHaltedObserved => {
            LatestObservedControlPostureV1::SuspendedOrHalted
        }
        MarketEventKindV1::ProviderRejectedObserved => {
            LatestObservedControlPostureV1::ProviderRejected
        }
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
    fills: &[&CanonicalFillObservationV1],
    adjustments: &[&CanonicalFillAdjustmentObservationV1],
) -> Result<(ObservedExecutionPostureV1, ObservedTargetProgressV1), ProjectionError> {
    let mut fill_by_semantic: BTreeMap<Digest32, &CanonicalFillObservationV1> = BTreeMap::new();
    for fill in fills {
        fill_by_semantic.insert(fill.semantic_commitment(), fill);
    }

    let mut outgoing: BTreeMap<Digest32, AdjustmentEdge> = BTreeMap::new();
    let mut incoming: BTreeMap<Digest32, Digest32> = BTreeMap::new();
    let mut unresolved_adjustment = false;

    for adjustment in adjustments {
        match &adjustment.input().adjustment_kind {
            FillAdjustmentKindV1::Correction {
                prior_fill_commitment,
                replacement_fill_commitment,
            } => {
                if !fill_by_semantic.contains_key(prior_fill_commitment)
                    || !fill_by_semantic.contains_key(replacement_fill_commitment)
                {
                    unresolved_adjustment = true;
                    continue;
                }

                match outgoing.get(prior_fill_commitment) {
                    None => {
                        outgoing.insert(
                            *prior_fill_commitment,
                            AdjustmentEdge::Correction(*replacement_fill_commitment),
                        );
                    }
                    Some(AdjustmentEdge::Correction(existing))
                        if existing == replacement_fill_commitment => {}
                    Some(_) => return Err(ProjectionError::AdjustmentTopologyConflict),
                }

                match incoming.get(replacement_fill_commitment) {
                    None => {
                        incoming.insert(*replacement_fill_commitment, *prior_fill_commitment);
                    }
                    Some(existing) if existing == prior_fill_commitment => {}
                    Some(_) => return Err(ProjectionError::AdjustmentTopologyConflict),
                }
            }
            FillAdjustmentKindV1::Bust {
                prior_fill_commitment,
            } => {
                if !fill_by_semantic.contains_key(prior_fill_commitment) {
                    unresolved_adjustment = true;
                    continue;
                }

                match outgoing.get(prior_fill_commitment) {
                    None => {
                        outgoing.insert(*prior_fill_commitment, AdjustmentEdge::Bust);
                    }
                    Some(AdjustmentEdge::Bust) => {}
                    Some(_) => return Err(ProjectionError::AdjustmentTopologyConflict),
                }
            }
        }
    }

    if has_adjustment_cycle(&outgoing) {
        return Err(ProjectionError::AdjustmentCycle);
    }

    if unresolved_adjustment {
        return Ok((
            ObservedExecutionPostureV1::AdjustmentResolutionIndeterminate,
            ObservedTargetProgressV1::AdjustmentResolutionIndeterminate,
        ));
    }

    let effective_fills: Vec<&CanonicalFillObservationV1> = fill_by_semantic
        .iter()
        .filter_map(|(semantic, fill)| {
            if outgoing.contains_key(semantic) {
                None
            } else {
                Some(*fill)
            }
        })
        .collect();

    let execution_posture = if fills.is_empty() && adjustments.is_empty() {
        ObservedExecutionPostureV1::NoExecutionObserved
    } else if adjustments.is_empty() {
        ObservedExecutionPostureV1::ExecutionObserved
    } else {
        ObservedExecutionPostureV1::AdjustedExecutionObserved
    };

    let target_progress = match &intent.input().quantity {
        QuantitySpecV1::Units(target) => {
            let mut observed = 0_u64;

            for fill in &effective_fills {
                let quantity = &fill.input().executed_quantity;
                if quantity.unit_profile != target.unit_profile
                    || quantity.amount.asset() != target.amount.asset()
                {
                    return Err(ProjectionError::IncompatibleFillUnits);
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
            if effective_fills.is_empty() {
                ObservedTargetProgressV1::NotionalNoEffectiveFillObserved
            } else {
                ObservedTargetProgressV1::NotionalTargetProgressIndeterminate {
                    effective_observed_fill_count: u32::try_from(effective_fills.len())
                        .map_err(|_| ProjectionError::FrontierTooLarge)?,
                }
            }
        }
    };

    Ok((execution_posture, target_progress))
}

fn has_adjustment_cycle(outgoing: &BTreeMap<Digest32, AdjustmentEdge>) -> bool {
    for start in outgoing.keys() {
        let mut seen = BTreeSet::new();
        let mut current = *start;

        loop {
            if !seen.insert(current) {
                return true;
            }
            match outgoing.get(&current) {
                Some(AdjustmentEdge::Correction(next)) => current = *next,
                Some(AdjustmentEdge::Bust) | None => break,
            }
        }
    }
    false
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
    use mycelix_finance_market_observation::{
        canonicalize_event_observation_v1, canonicalize_fill_adjustment_v1,
        canonicalize_fill_observation_v1, FillAdjustmentObservationInputV1,
        FillObservationInputV1, MarketEventObservationInputV1,
        ProviderChronologyV1, ProviderObservationRefV1,
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
        id: &str,
        execution_ref: &str,
        quantity: u64,
        evidence_byte: u8,
    ) -> CanonicalFillObservationV1 {
        canonicalize_fill_observation_v1(FillObservationInputV1 {
            subject: observation_subject(intent),
            observation_ref: ProviderObservationRefV1 {
                observation_profile: projection_scope().fill_observation_profile,
                observation_id: MarketExternalIdV1::new(id).unwrap(),
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

    #[test]
    fn reversed_input_order_projects_same_latest_observed_event() {
        let intent = intent_with_quantity(QuantitySpecV1::Units(unit_quantity(100)));
        let working = event(&intent, "event:working", MarketEventKindV1::WorkingObserved, Some(10), 0x81);
        let cancel = event(&intent, "event:cancel", MarketEventKindV1::CancelRequestedObserved, Some(11), 0x82);

        let first = project_observed_market_order_v1(
            &intent,
            &projection_scope(),
            &[working.clone(), cancel.clone()],
            &[],
            &[],
        )
        .unwrap();
        let second = project_observed_market_order_v1(
            &intent,
            &projection_scope(),
            &[cancel, working],
            &[],
            &[],
        )
        .unwrap();

        assert_eq!(first, second);
        assert_eq!(first.latest_observed_control, LatestObservedControlPostureV1::PendingCancel);
    }

    #[test]
    fn multiple_unsequenced_events_are_chronology_indeterminate() {
        let intent = intent_with_quantity(QuantitySpecV1::Units(unit_quantity(100)));
        let accepted = event(&intent, "event:accepted", MarketEventKindV1::ProviderAcceptedObserved, None, 0x81);
        let working = event(&intent, "event:working", MarketEventKindV1::WorkingObserved, None, 0x82);

        let projected = project_observed_market_order_v1(
            &intent,
            &projection_scope(),
            &[accepted, working],
            &[],
            &[],
        )
        .unwrap();

        assert_eq!(
            projected.latest_observed_control,
            LatestObservedControlPostureV1::ChronologyIndeterminate
        );
    }

    #[test]
    fn alternate_evidence_for_same_event_does_not_duplicate_transition() {
        let intent = intent_with_quantity(QuantitySpecV1::Units(unit_quantity(100)));
        let first = event(&intent, "event:working", MarketEventKindV1::WorkingObserved, None, 0x81);
        let second = event(&intent, "event:working", MarketEventKindV1::WorkingObserved, None, 0x82);

        let projected = project_observed_market_order_v1(
            &intent,
            &projection_scope(),
            &[first, second],
            &[],
            &[],
        )
        .unwrap();

        assert_eq!(projected.unique_event_count, 1);
        assert_eq!(projected.latest_observed_control, LatestObservedControlPostureV1::Working);
    }

    #[test]
    fn unit_target_progress_is_observed_frontier_only() {
        let intent = intent_with_quantity(QuantitySpecV1::Units(unit_quantity(100)));
        let first_fill = fill(&intent, "fill:1", "exec:1", 40, 0x91);

        let projected = project_observed_market_order_v1(
            &intent,
            &projection_scope(),
            &[],
            &[first_fill],
            &[],
        )
        .unwrap();

        assert_eq!(
            projected.target_progress,
            ObservedTargetProgressV1::Unit {
                target_atomic_units: 100,
                effective_observed_atomic_units: 40,
                unobserved_target_gap_under_this_frontier: 60,
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
        let first_fill = fill(&intent, "fill:1", "exec:1", 40, 0x91);

        let projected = project_observed_market_order_v1(
            &intent,
            &projection_scope(),
            &[],
            &[first_fill],
            &[],
        )
        .unwrap();

        assert_eq!(
            projected.target_progress,
            ObservedTargetProgressV1::NotionalTargetProgressIndeterminate {
                effective_observed_fill_count: 1,
            }
        );
    }

    #[test]
    fn unresolved_adjustment_keeps_execution_projection_indeterminate() {
        let intent = intent_with_quantity(QuantitySpecV1::Units(unit_quantity(100)));
        let adjustment = canonicalize_fill_adjustment_v1(FillAdjustmentObservationInputV1 {
            subject: observation_subject(&intent),
            observation_ref: ProviderObservationRefV1 {
                observation_profile: projection_scope().adjustment_observation_profile,
                observation_id: MarketExternalIdV1::new("adjust:1").unwrap(),
            },
            adjustment_kind: FillAdjustmentKindV1::Bust {
                prior_fill_commitment: digest(0xaa),
            },
            chronology: chronology(Some(20)),
            source_evidence_commitment: digest(0x92),
        })
        .unwrap();

        let projected = project_observed_market_order_v1(
            &intent,
            &projection_scope(),
            &[],
            &[],
            &[adjustment],
        )
        .unwrap();

        assert_eq!(
            projected.execution_posture,
            ObservedExecutionPostureV1::AdjustmentResolutionIndeterminate
        );
        assert_eq!(
            projected.target_progress,
            ObservedTargetProgressV1::AdjustmentResolutionIndeterminate
        );
    }

    #[test]
    fn replace_acceptance_does_not_guess_successor() {
        let intent = intent_with_quantity(QuantitySpecV1::Units(unit_quantity(100)));
        let replace = event(
            &intent,
            "event:replace-accepted",
            MarketEventKindV1::ReplaceAcceptedObserved,
            None,
            0x81,
        );

        let projected = project_observed_market_order_v1(
            &intent,
            &projection_scope(),
            &[replace],
            &[],
            &[],
        )
        .unwrap();

        assert_eq!(
            projected.latest_observed_control,
            LatestObservedControlPostureV1::ReplacementAcceptedSuccessorUnresolved
        );
    }

    #[test]
    fn cancel_acceptance_is_not_terminal_cancellation() {
        let intent = intent_with_quantity(QuantitySpecV1::Units(unit_quantity(100)));
        let cancel_ack = event(
            &intent,
            "event:cancel-accepted",
            MarketEventKindV1::CancelAcceptedObserved,
            None,
            0x81,
        );

        let projected = project_observed_market_order_v1(
            &intent,
            &projection_scope(),
            &[cancel_ack],
            &[],
            &[],
        )
        .unwrap();

        assert_eq!(
            projected.latest_observed_control,
            LatestObservedControlPostureV1::CancelAcknowledged
        );
    }
}
