use std::collections::{BTreeMap, BTreeSet};

use mycelix_finance_sync_graph::{BoundedText, Commitment32, SettlementGraphV1};

use crate::canonical::{derive_observation_commitment, derive_receipt_commitment};
use crate::{
    DerivedLegObservationV1, EvalError, LegObservationInputV1, LegObservedDispositionV1,
    ObservationClassV1, ObservationEvaluationInputV1, ObservedGraphDispositionV1,
    ObservedGraphReceiptV1, SelectedObservationStreamV1,
    MAX_OBSERVATIONS_PER_LEG, MAX_OBSERVATION_STREAMS_PER_LEG, MAX_TOTAL_OBSERVATIONS,
};

#[derive(Clone)]
struct CanonicalObservation {
    input: LegObservationInputV1,
    commitment: Commitment32,
}

pub fn evaluate_observed_graph_v1(
    graph: &SettlementGraphV1,
    input: ObservationEvaluationInputV1,
) -> Result<ObservedGraphReceiptV1, EvalError> {
    if input.graph_commitment != graph.graph_commitment() {
        return Err(EvalError::GraphCommitmentMismatch);
    }
    if input.observations.len() > MAX_TOTAL_OBSERVATIONS {
        return Err(EvalError::TooManyObservations);
    }

    let graph_leg_ids: BTreeSet<_> = graph.legs().iter().map(|leg| leg.leg_id()).collect();
    let mut raw_counts = BTreeMap::<Commitment32, usize>::new();
    let mut evidence_identities =
        BTreeMap::<BoundedText, (Commitment32, Commitment32)>::new();
    let mut conflicting_legs = BTreeSet::<Commitment32>::new();
    let mut observations_by_leg =
        BTreeMap::<Commitment32, Vec<CanonicalObservation>>::new();
    let mut all_observation_commitments = BTreeSet::<Commitment32>::new();

    for observation in input.observations {
        if !graph_leg_ids.contains(&observation.leg_id) {
            return Err(EvalError::UnknownLeg);
        }

        let count = raw_counts.entry(observation.leg_id).or_default();
        *count = count
            .checked_add(1)
            .ok_or(EvalError::TooManyObservationsForLeg)?;
        if *count > MAX_OBSERVATIONS_PER_LEG {
            return Err(EvalError::TooManyObservationsForLeg);
        }

        let commitment = derive_observation_commitment(graph.graph_commitment(), &observation)?;
        all_observation_commitments.insert(commitment);

        if let Some((prior_commitment, prior_leg_id)) =
            evidence_identities.get(&observation.evidence_id).copied()
        {
            if prior_commitment == commitment {
                // Exact replay of one physical evidence identity is idempotent.
                continue;
            }
            conflicting_legs.insert(prior_leg_id);
            conflicting_legs.insert(observation.leg_id);
        } else {
            evidence_identities.insert(
                observation.evidence_id.clone(),
                (commitment, observation.leg_id),
            );
        }

        observations_by_leg
            .entry(observation.leg_id)
            .or_default()
            .push(CanonicalObservation {
                input: observation,
                commitment,
            });
    }

    let mut leg_dispositions = Vec::with_capacity(graph.legs().len());
    for leg in graph.legs() {
        let leg_id = leg.leg_id();
        let observations = observations_by_leg.get(&leg_id);
        leg_dispositions.push(derive_leg_disposition(
            leg_id,
            observations.map(Vec::as_slice).unwrap_or(&[]),
            conflicting_legs.contains(&leg_id),
        )?);
    }
    leg_dispositions.sort_by_key(LegObservedDispositionV1::leg_id);

    let disposition = derive_graph_disposition(&leg_dispositions);
    let mut receipt = ObservedGraphReceiptV1 {
        graph_commitment: graph.graph_commitment(),
        evaluation_profile: input.evaluation_profile,
        evaluation_context_commitment: input.evaluation_context_commitment,
        observation_commitments: all_observation_commitments.into_iter().collect(),
        leg_dispositions,
        disposition,
        receipt_commitment: Commitment32::from_bytes([0_u8; 32]),
    };
    receipt.receipt_commitment = derive_receipt_commitment(&receipt)?;
    Ok(receipt)
}

fn derive_leg_disposition(
    leg_id: Commitment32,
    observations: &[CanonicalObservation],
    evidence_identity_conflict: bool,
) -> Result<LegObservedDispositionV1, EvalError> {
    if observations.is_empty() {
        return Ok(LegObservedDispositionV1 {
            leg_id,
            state: if evidence_identity_conflict {
                DerivedLegObservationV1::Conflicted
            } else {
                DerivedLegObservationV1::NoObservation
            },
            selected_streams: Vec::new(),
        });
    }

    let mut streams = BTreeMap::<BoundedText, Vec<&CanonicalObservation>>::new();
    for observation in observations {
        streams
            .entry(observation.input.observation_stream_ref.clone())
            .or_default()
            .push(observation);
    }
    if streams.len() > MAX_OBSERVATION_STREAMS_PER_LEG {
        return Err(EvalError::TooManyObservationStreamsForLeg);
    }

    let mut selected_streams = Vec::with_capacity(streams.len());
    let mut stream_conflict = false;

    for (stream_ref, stream_observations) in streams {
        let selected_revision = stream_observations
            .iter()
            .map(|observation| observation.input.observation_revision)
            .max()
            .ok_or(EvalError::InternalInvariant)?;

        let selected: Vec<_> = stream_observations
            .iter()
            .copied()
            .filter(|observation| observation.input.observation_revision == selected_revision)
            .collect();

        let semantics: BTreeSet<_> = selected
            .iter()
            .map(|observation| {
                (
                    observation.input.class,
                    observation.input.observation_profile.clone(),
                )
            })
            .collect();

        let mut selected_observation_commitments: Vec<_> =
            selected.iter().map(|observation| observation.commitment).collect();
        selected_observation_commitments.sort_unstable();
        selected_observation_commitments.dedup();

        if semantics.len() != 1 {
            stream_conflict = true;
            continue;
        }

        let (class, observation_profile) = semantics
            .into_iter()
            .next()
            .ok_or(EvalError::InternalInvariant)?;

        selected_streams.push(SelectedObservationStreamV1 {
            observation_stream_ref: stream_ref,
            selected_revision,
            class,
            observation_profile,
            selected_observation_commitments,
        });
    }

    selected_streams.sort_by(|left, right| {
        left.observation_stream_ref
            .cmp(&right.observation_stream_ref)
    });

    let state = derive_leg_state(
        &selected_streams,
        evidence_identity_conflict || stream_conflict,
    );

    Ok(LegObservedDispositionV1 {
        leg_id,
        state,
        selected_streams,
    })
}

fn derive_leg_state(
    streams: &[SelectedObservationStreamV1],
    has_conflict: bool,
) -> DerivedLegObservationV1 {
    if has_conflict {
        return DerivedLegObservationV1::Conflicted;
    }
    if streams.is_empty() {
        return DerivedLegObservationV1::NoObservation;
    }

    let classes: BTreeSet<_> = streams.iter().map(|stream| stream.class).collect();
    let has_applied = classes.contains(&ObservationClassV1::ReportedAppliedUnqualified);
    let has_reversed = classes.contains(&ObservationClassV1::ReportedReversedUnqualified);
    let has_not_dispatched = classes.contains(&ObservationClassV1::ReportedNotDispatched);
    let has_definitely_rejected =
        classes.contains(&ObservationClassV1::ReportedDefinitelyRejectedBeforeEffect);
    let has_rejected = classes.contains(&ObservationClassV1::ReportedRejectedUnqualified);
    let has_unknown = classes.contains(&ObservationClassV1::ReportedOutcomeUnknown);
    let has_pending = classes.contains(&ObservationClassV1::ReportedPending);
    let has_indeterminate = classes.contains(&ObservationClassV1::ReportedIndeterminate);

    let has_effect_evidence = has_applied || has_reversed;
    let has_no_effect_evidence = has_not_dispatched || has_definitely_rejected || has_rejected;
    let has_uncertainty = has_unknown || has_pending || has_indeterminate;

    // Applied/reversed observations and explicit no-effect/rejection observations
    // from independent current stream frontiers are semantically contradictory.
    if has_effect_evidence && has_no_effect_evidence {
        return DerivedLegObservationV1::Conflicted;
    }

    // Reversal implies an applied effect was observed at some point. An
    // additional applied report is not itself contradictory, but any remaining
    // uncertainty stays visible at graph level through the reversal posture.
    if has_reversed {
        return DerivedLegObservationV1::ObservedReversedUnqualified;
    }

    if has_applied {
        return if has_uncertainty {
            DerivedLegObservationV1::ObservedAppliedWithUncertainty
        } else {
            DerivedLegObservationV1::ObservedAppliedUnqualified
        };
    }

    if has_uncertainty {
        if has_indeterminate && !has_unknown && !has_pending && !has_no_effect_evidence {
            return DerivedLegObservationV1::Indeterminate;
        }
        if has_pending && !has_unknown && !has_indeterminate && !has_no_effect_evidence {
            return DerivedLegObservationV1::ReportedPending;
        }
        return DerivedLegObservationV1::ReportedOutcomeUnknown;
    }

    if classes.len() == 1 {
        let class = classes.iter().next().copied();
        return match class {
            Some(ObservationClassV1::ReportedNotDispatched) => {
                DerivedLegObservationV1::ReportedNotDispatched
            }
            Some(ObservationClassV1::ReportedDefinitelyRejectedBeforeEffect) => {
                DerivedLegObservationV1::ReportedDefinitelyRejectedBeforeEffect
            }
            Some(ObservationClassV1::ReportedRejectedUnqualified) => {
                DerivedLegObservationV1::ObservedRejectedUnqualified
            }
            _ => DerivedLegObservationV1::Indeterminate,
        };
    }

    // Multiple independent streams may agree only at the coarser fact that no
    // applied effect is currently reported (e.g. NotDispatched + Rejected).
    DerivedLegObservationV1::NoKnownAppliedEffect
}

fn derive_graph_disposition(
    dispositions: &[LegObservedDispositionV1],
) -> ObservedGraphDispositionV1 {
    if dispositions
        .iter()
        .any(|leg| leg.state == DerivedLegObservationV1::Conflicted)
    {
        return ObservedGraphDispositionV1::Conflicted;
    }

    let has_reversal = dispositions
        .iter()
        .any(|leg| leg.state == DerivedLegObservationV1::ObservedReversedUnqualified);
    if has_reversal {
        let only_effect_states = dispositions.iter().all(|leg| {
            matches!(
                leg.state,
                DerivedLegObservationV1::ObservedAppliedUnqualified
                    | DerivedLegObservationV1::ObservedReversedUnqualified
            )
        });
        return if only_effect_states {
            ObservedGraphDispositionV1::ReversalObserved
        } else {
            ObservedGraphDispositionV1::ReconciliationRequired
        };
    }

    let all_applied = !dispositions.is_empty()
        && dispositions
            .iter()
            .all(|leg| leg.state == DerivedLegObservationV1::ObservedAppliedUnqualified);
    if all_applied {
        return ObservedGraphDispositionV1::AllRequiredEffectsObservedAppliedButUnqualified;
    }

    let has_applied = dispositions.iter().any(|leg| {
        matches!(
            leg.state,
            DerivedLegObservationV1::ObservedAppliedUnqualified
                | DerivedLegObservationV1::ObservedAppliedWithUncertainty
        )
    });

    if has_applied {
        let has_definite_non_applied = dispositions.iter().any(|leg| {
            matches!(
                leg.state,
                DerivedLegObservationV1::ReportedNotDispatched
                    | DerivedLegObservationV1::ReportedDefinitelyRejectedBeforeEffect
                    | DerivedLegObservationV1::ObservedRejectedUnqualified
                    | DerivedLegObservationV1::NoKnownAppliedEffect
            )
        });
        if has_definite_non_applied {
            return ObservedGraphDispositionV1::PartialEffectObserved;
        }

        // An applied observation combined with a missing/pending/unknown stream
        // is not an established partial failure. The unresolved leg may also
        // have applied, so uncertainty must remain explicit.
        return ObservedGraphDispositionV1::UnknownEffectPossible;
    }

    let has_indeterminate = dispositions
        .iter()
        .any(|leg| leg.state == DerivedLegObservationV1::Indeterminate);
    let has_unknown = dispositions.iter().any(|leg| {
        matches!(
            leg.state,
            DerivedLegObservationV1::NoObservation
                | DerivedLegObservationV1::ReportedOutcomeUnknown
                | DerivedLegObservationV1::ReportedPending
                | DerivedLegObservationV1::ObservedAppliedWithUncertainty
        )
    });

    if has_unknown {
        return ObservedGraphDispositionV1::UnknownEffectPossible;
    }
    if has_indeterminate {
        return ObservedGraphDispositionV1::Indeterminate;
    }

    ObservedGraphDispositionV1::NoKnownAppliedEffect
}
