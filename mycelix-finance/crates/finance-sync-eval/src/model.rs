use std::fmt;

use mycelix_finance_sync_graph::{BoundedText, Commitment32, SemanticProfileRefV1};
use serde::{Deserialize, Serialize};

pub const MAX_TOTAL_OBSERVATIONS: usize = 1024;
pub const MAX_OBSERVATIONS_PER_LEG: usize = 32;
pub const MAX_OBSERVATION_STREAMS_PER_LEG: usize = 16;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EvalError {
    GraphCommitmentMismatch,
    TooManyObservations,
    TooManyObservationsForLeg,
    TooManyObservationStreamsForLeg,
    UnknownLeg,
    CanonicalLengthOverflow,
    InternalInvariant,
}

impl fmt::Display for EvalError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::GraphCommitmentMismatch => {
                "evaluation input does not bind the supplied settlement graph"
            }
            Self::TooManyObservations => "evaluation exceeds the total observation bound",
            Self::TooManyObservationsForLeg => {
                "evaluation exceeds the per-leg observation bound"
            }
            Self::TooManyObservationStreamsForLeg => {
                "evaluation exceeds the per-leg observation-stream bound"
            }
            Self::UnknownLeg => "observation references a leg outside the settlement graph",
            Self::CanonicalLengthOverflow => "canonical encoding length exceeds u32",
            Self::InternalInvariant => "internal observation-lattice invariant failed",
        };
        f.write_str(message)
    }
}

impl std::error::Error for EvalError {}

/// An explicitly unqualified report about one settlement leg.
///
/// These classes are descriptive inputs only. None constitutes FIN-ECO
/// settlement qualification or a synchronization proof.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ObservationClassV1 {
    ReportedNotDispatched,
    ReportedDefinitelyRejectedBeforeEffect,
    ReportedOutcomeUnknown,
    ReportedPending,
    ReportedAppliedUnqualified,
    ReportedRejectedUnqualified,
    ReportedReversedUnqualified,
    ReportedIndeterminate,
}

impl ObservationClassV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::ReportedNotDispatched => 1,
            Self::ReportedDefinitelyRejectedBeforeEffect => 2,
            Self::ReportedOutcomeUnknown => 3,
            Self::ReportedPending => 4,
            Self::ReportedAppliedUnqualified => 5,
            Self::ReportedRejectedUnqualified => 6,
            Self::ReportedReversedUnqualified => 7,
            Self::ReportedIndeterminate => 8,
        }
    }
}

/// One explicitly unqualified observation from one independently revisioned
/// observation stream.
///
/// `observation_stream_ref` is part of the semantic identity. Revisions are
/// compared only within that exact stream; a high revision from one stream can
/// never erase another stream's current frontier.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LegObservationInputV1 {
    pub leg_id: Commitment32,
    pub observation_stream_ref: BoundedText,
    pub evidence_id: BoundedText,
    pub evidence_commitment: Commitment32,
    pub observation_revision: u64,
    pub class: ObservationClassV1,
    pub observation_profile: SemanticProfileRefV1,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ObservationEvaluationInputV1 {
    pub graph_commitment: Commitment32,
    pub evaluation_profile: SemanticProfileRefV1,
    pub evaluation_context_commitment: Commitment32,
    pub observations: Vec<LegObservationInputV1>,
}

/// Deterministic per-leg state derived from the selected frontier of every
/// independent observation stream.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum DerivedLegObservationV1 {
    NoObservation,
    ReportedNotDispatched,
    ReportedDefinitelyRejectedBeforeEffect,
    ReportedOutcomeUnknown,
    ReportedPending,
    ObservedAppliedUnqualified,
    ObservedAppliedWithUncertainty,
    ObservedRejectedUnqualified,
    ObservedReversedUnqualified,
    NoKnownAppliedEffect,
    Conflicted,
    Indeterminate,
}

impl DerivedLegObservationV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::NoObservation => 0,
            Self::ReportedNotDispatched => 1,
            Self::ReportedDefinitelyRejectedBeforeEffect => 2,
            Self::ReportedOutcomeUnknown => 3,
            Self::ReportedPending => 4,
            Self::ObservedAppliedUnqualified => 5,
            Self::ObservedAppliedWithUncertainty => 6,
            Self::ObservedRejectedUnqualified => 7,
            Self::ObservedReversedUnqualified => 8,
            Self::NoKnownAppliedEffect => 9,
            Self::Conflicted => 10,
            Self::Indeterminate => 11,
        }
    }
}

/// Observation-level graph disposition. There is deliberately no `Completed`
/// or `Qualified*` variant in FIN-SYNC-002A.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ObservedGraphDispositionV1 {
    NoKnownAppliedEffect,
    UnknownEffectPossible,
    PartialEffectObserved,
    AllRequiredEffectsObservedAppliedButUnqualified,
    ReversalObserved,
    ReconciliationRequired,
    Conflicted,
    Indeterminate,
}

impl ObservedGraphDispositionV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::NoKnownAppliedEffect => 1,
            Self::UnknownEffectPossible => 2,
            Self::PartialEffectObserved => 3,
            Self::AllRequiredEffectsObservedAppliedButUnqualified => 4,
            Self::ReversalObserved => 5,
            Self::ReconciliationRequired => 6,
            Self::Conflicted => 7,
            Self::Indeterminate => 8,
        }
    }
}

/// The selected frontier for one observation stream. This is a derived positive
/// output and intentionally is not deserializable from caller data.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct SelectedObservationStreamV1 {
    pub(crate) observation_stream_ref: BoundedText,
    pub(crate) selected_revision: u64,
    pub(crate) class: ObservationClassV1,
    pub(crate) observation_profile: SemanticProfileRefV1,
    pub(crate) selected_observation_commitments: Vec<Commitment32>,
}

impl SelectedObservationStreamV1 {
    pub fn observation_stream_ref(&self) -> &BoundedText {
        &self.observation_stream_ref
    }

    pub fn selected_revision(&self) -> u64 {
        self.selected_revision
    }

    pub fn class(&self) -> ObservationClassV1 {
        self.class
    }

    pub fn observation_profile(&self) -> &SemanticProfileRefV1 {
        &self.observation_profile
    }

    pub fn selected_observation_commitments(&self) -> &[Commitment32] {
        &self.selected_observation_commitments
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct LegObservedDispositionV1 {
    pub(crate) leg_id: Commitment32,
    pub(crate) state: DerivedLegObservationV1,
    pub(crate) selected_streams: Vec<SelectedObservationStreamV1>,
}

impl LegObservedDispositionV1 {
    pub fn leg_id(&self) -> Commitment32 {
        self.leg_id
    }

    pub fn state(&self) -> DerivedLegObservationV1 {
        self.state
    }

    pub fn selected_streams(&self) -> &[SelectedObservationStreamV1] {
        &self.selected_streams
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ObservedGraphReceiptV1 {
    pub(crate) graph_commitment: Commitment32,
    pub(crate) evaluation_profile: SemanticProfileRefV1,
    pub(crate) evaluation_context_commitment: Commitment32,
    /// Canonical commitments of every admitted non-duplicate physical input,
    /// including lower revisions. This keeps the receipt evidence-set complete
    /// even when only the stream frontier affects the derived state.
    pub(crate) observation_commitments: Vec<Commitment32>,
    pub(crate) leg_dispositions: Vec<LegObservedDispositionV1>,
    pub(crate) disposition: ObservedGraphDispositionV1,
    pub(crate) receipt_commitment: Commitment32,
}

impl ObservedGraphReceiptV1 {
    pub fn graph_commitment(&self) -> Commitment32 {
        self.graph_commitment
    }

    pub fn evaluation_profile(&self) -> &SemanticProfileRefV1 {
        &self.evaluation_profile
    }

    pub fn evaluation_context_commitment(&self) -> Commitment32 {
        self.evaluation_context_commitment
    }

    pub fn observation_commitments(&self) -> &[Commitment32] {
        &self.observation_commitments
    }

    pub fn leg_dispositions(&self) -> &[LegObservedDispositionV1] {
        &self.leg_dispositions
    }

    pub fn disposition(&self) -> ObservedGraphDispositionV1 {
        self.disposition
    }

    pub fn receipt_commitment(&self) -> Commitment32 {
        self.receipt_commitment
    }
}
