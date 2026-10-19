// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Causal identity and retry semantics.

#[cfg(not(test))]
use core::fmt;

use crate::ids::{
    AttemptRef, LogicalIntentRef, OperationCommitment, SemanticProfileId, SourceEventRef,
    TransportDeliveryRef, WorkflowRef,
};
#[cfg(not(test))]
use crate::qualification::{CutError, QualificationCut, QualifiedInputRef};

/// One logical institutional effect bound to the owning domain/profile's exact
/// material-operation commitment.
///
/// The generic Business core does not compute or validate `semantic_commitment`.
/// The owning domain/profile defines canonical material fields and produces the
/// commitment. Business only preserves the binding so a caller cannot silently
/// reuse the same logical ID after changing material operation semantics.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CommittedIntent {
    pub intent_ref: LogicalIntentRef,
    pub operation_profile: SemanticProfileId,
    pub semantic_commitment: OperationCommitment,
}

impl CommittedIntent {
    /// Bind one logical intent ID to an exact operation profile and semantic
    /// commitment supplied by the owning domain/profile.
    #[must_use]
    pub fn new(
        intent_ref: LogicalIntentRef,
        operation_profile: SemanticProfileId,
        semantic_commitment: OperationCommitment,
    ) -> Self {
        Self {
            intent_ref,
            operation_profile,
            semantic_commitment,
        }
    }
}

/// Test-only migration from pre-commitment logical-intent fixtures.
///
/// Production callers never receive this conversion: they must provide the
/// owning-domain/profile semantic commitment explicitly.
#[cfg(test)]
impl From<LogicalIntentRef> for CommittedIntent {
    fn from(intent_ref: LogicalIntentRef) -> Self {
        let operation_profile =
            SemanticProfileId::new("test.legacy-intent", 1).expect("static test profile is valid");
        let semantic_commitment = OperationCommitment::new(format!(
            "test-fixture:{}",
            intent_ref.as_str()
        ))
        .expect("test-only commitment derived from valid logical intent");
        Self::new(intent_ref, operation_profile, semantic_commitment)
    }
}

/// Outcome of one concrete attempt to realize a logical intent.
///
/// In production this enum is only an adapter-supplied interpretation. Retry
/// permission is available only through `AttemptOutcomeBinding`, which binds the
/// interpreted outcome to an exact attempt, committed intent, and current result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AttemptOutcome {
    Prepared,
    Dispatched,
    Accepted,
    Rejected,
    ProvenNotApplied,
    Applied,
    OutcomeUnknown,
}

/// Production binding of an interpreted attempt outcome to exact provenance.
#[cfg(not(test))]
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AttemptOutcomeBinding {
    attempt: AttemptRef,
    logical_intent: CommittedIntent,
    outcome: AttemptOutcome,
    source: QualifiedInputRef,
}

/// Structural attempt-outcome provenance errors.
#[cfg(not(test))]
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AttemptOutcomeError {
    InvalidQualification(CutError),
    SourceMissingFromCut {
        attempt: AttemptRef,
        source: QualifiedInputRef,
    },
}

#[cfg(not(test))]
impl fmt::Display for AttemptOutcomeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidQualification(err) => {
                write!(f, "invalid attempt-outcome qualification cut: {err}")
            }
            Self::SourceMissingFromCut { attempt, source } => write!(
                f,
                "attempt-outcome source {}/{}@{} for attempt {attempt} is absent from the exact qualification cut",
                source.domain, source.record, source.version
            ),
        }
    }
}

#[cfg(not(test))]
impl std::error::Error for AttemptOutcomeError {}

#[cfg(not(test))]
impl From<CutError> for AttemptOutcomeError {
    fn from(value: CutError) -> Self {
        Self::InvalidQualification(value)
    }
}

#[cfg(not(test))]
impl AttemptOutcomeBinding {
    /// Bind one owning-adapter interpretation of an attempt outcome to exact,
    /// current provenance in the qualification cut.
    pub fn bind(
        attempt: AttemptRef,
        logical_intent: CommittedIntent,
        outcome: AttemptOutcome,
        source: QualifiedInputRef,
        qualification_cut: &QualificationCut,
    ) -> Result<Self, AttemptOutcomeError> {
        let binding = Self {
            attempt,
            logical_intent,
            outcome,
            source,
        };
        binding.validate_against(qualification_cut)?;
        Ok(binding)
    }

    /// Exact execution attempt whose outcome was interpreted.
    #[must_use]
    pub fn attempt(&self) -> &AttemptRef {
        &self.attempt
    }

    /// Exact committed logical effect associated with the attempt.
    #[must_use]
    pub fn logical_intent(&self) -> &CommittedIntent {
        &self.logical_intent
    }

    /// Owning-adapter interpretation of the exact attempt result.
    #[must_use]
    pub const fn outcome(&self) -> AttemptOutcome {
        self.outcome
    }

    /// Exact domain-owned result supporting the interpreted outcome.
    #[must_use]
    pub fn source(&self) -> &QualifiedInputRef {
        &self.source
    }

    fn validate_against(
        &self,
        qualification_cut: &QualificationCut,
    ) -> Result<(), AttemptOutcomeError> {
        if !qualification_cut.inputs().contains(&self.source) {
            return Err(AttemptOutcomeError::SourceMissingFromCut {
                attempt: self.attempt.clone(),
                source: self.source.clone(),
            });
        }

        qualification_cut.validate_at_qualification_time()?;
        Ok(())
    }

    /// Retry permission requiring no additional duplicate-safety claim.
    ///
    /// Only a current, provenance-bound `ProvenNotApplied` result qualifies.
    #[must_use]
    pub fn retry_permitted_without_additional_safety(
        &self,
        qualification_cut: &QualificationCut,
    ) -> bool {
        matches!(self.outcome, AttemptOutcome::ProvenNotApplied)
            && self.validate_against(qualification_cut).is_ok()
    }

    /// Whether an exact unknown attempt outcome may be retried under one exact,
    /// current duplicate-safety result for the same committed operation.
    #[must_use]
    pub fn retry_permitted_with_safety(
        &self,
        safety: &RetrySafety,
        qualification_cut: &QualificationCut,
    ) -> bool {
        matches!(self.outcome, AttemptOutcome::OutcomeUnknown)
            && self.validate_against(qualification_cut).is_ok()
            && safety
                .validate_against(&self.logical_intent, qualification_cut)
                .is_ok()
    }
}

/// Production retry-safety proof for one exact committed logical effect.
///
/// A positive safety claim is not a freely constructible enum. The owning
/// domain/profile supplies an exact result into the qualification cut; Business
/// preserves that provenance and revalidates it when deciding whether an unknown
/// outcome may be retried.
#[cfg(not(test))]
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RetrySafety {
    logical_intent: CommittedIntent,
    source: QualifiedInputRef,
}

/// Structural retry-safety errors.
#[cfg(not(test))]
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RetrySafetyError {
    InvalidQualification(CutError),
    SourceMissingFromCut {
        logical_intent: CommittedIntent,
        source: QualifiedInputRef,
    },
}

#[cfg(not(test))]
impl fmt::Display for RetrySafetyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidQualification(err) => {
                write!(f, "invalid retry-safety qualification cut: {err}")
            }
            Self::SourceMissingFromCut {
                logical_intent,
                source,
            } => write!(
                f,
                "retry-safety source {}/{}@{} for committed intent {} is absent from the exact qualification cut",
                source.domain, source.record, source.version, logical_intent.intent_ref
            ),
        }
    }
}

#[cfg(not(test))]
impl std::error::Error for RetrySafetyError {}

#[cfg(not(test))]
impl From<CutError> for RetrySafetyError {
    fn from(value: CutError) -> Self {
        Self::InvalidQualification(value)
    }
}

#[cfg(not(test))]
impl RetrySafety {
    /// Bind duplicate-safety semantics for one exact committed intent to an
    /// exact current domain-owned result in the same qualification cut.
    pub fn bind(
        logical_intent: CommittedIntent,
        source: QualifiedInputRef,
        qualification_cut: &QualificationCut,
    ) -> Result<Self, RetrySafetyError> {
        let binding = Self {
            logical_intent,
            source,
        };
        binding.validate_against(binding.logical_intent(), qualification_cut)?;
        Ok(binding)
    }

    /// Exact committed logical effect for which duplicate safety was qualified.
    #[must_use]
    pub fn logical_intent(&self) -> &CommittedIntent {
        &self.logical_intent
    }

    /// Exact domain-owned result supplied as the retry-safety basis.
    #[must_use]
    pub fn source(&self) -> &QualifiedInputRef {
        &self.source
    }

    fn validate_against(
        &self,
        logical_intent: &CommittedIntent,
        qualification_cut: &QualificationCut,
    ) -> Result<(), RetrySafetyError> {
        if &self.logical_intent != logical_intent
            || !qualification_cut.inputs().contains(&self.source)
        {
            return Err(RetrySafetyError::SourceMissingFromCut {
                logical_intent: logical_intent.clone(),
                source: self.source.clone(),
            });
        }

        qualification_cut.validate_at_qualification_time()?;
        Ok(())
    }
}

/// Legacy retry-safety shape for this crate's pre-provenance unit fixtures only.
/// Production callers never receive a freely constructible positive safety enum.
#[cfg(test)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RetrySafety {
    Default,
    DuplicateApplicationImpossibleOrAcceptable,
}

impl AttemptOutcome {
    /// Legacy unit-fixture compatibility. Production retry decisions use
    /// `AttemptOutcomeBinding` instead of a naked outcome enum.
    #[cfg(test)]
    #[must_use]
    pub const fn retry_permitted(self, safety: RetrySafety) -> bool {
        match self {
            Self::ProvenNotApplied => true,
            Self::OutcomeUnknown => matches!(
                safety,
                RetrySafety::DuplicateApplicationImpossibleOrAcceptable
            ),
            Self::Prepared
            | Self::Dispatched
            | Self::Accepted
            | Self::Rejected
            | Self::Applied => false,
        }
    }
}

/// Distinct causal identities associated with an institutional operation.
///
/// None of the optional identities substitutes for `logical_intent`. Rust's
/// type system enforces that distinction; an attempt ID cannot be passed where
/// a committed logical intent is required:
///
/// ```compile_fail
/// use mycelix_business_core::causal::CausalIdentity;
/// use mycelix_business_core::AttemptRef;
///
/// let attempt = AttemptRef::new("attempt:1").unwrap();
/// let _ = CausalIdentity::new(attempt);
/// ```
///
/// A transport-delivery ID likewise cannot masquerade as a source event:
///
/// ```compile_fail
/// use mycelix_business_core::causal::{CausalIdentity, CommittedIntent};
/// use mycelix_business_core::{
///     LogicalIntentRef, OperationCommitment, SemanticProfileId, TransportDeliveryRef,
/// };
///
/// let intent = CommittedIntent::new(
///     LogicalIntentRef::new("intent:1").unwrap(),
///     SemanticProfileId::new("finance.pay", 1).unwrap(),
///     OperationCommitment::new("commitment:pay:500:alice").unwrap(),
/// );
/// let identity = CausalIdentity::new(intent);
/// let delivery = TransportDeliveryRef::new("delivery:1").unwrap();
/// let _ = identity.with_source_event(delivery);
/// ```
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CausalIdentity {
    pub logical_intent: CommittedIntent,
    pub attempt: Option<AttemptRef>,
    pub source_event: Option<SourceEventRef>,
    pub transport_delivery: Option<TransportDeliveryRef>,
    pub caused_by: Option<CommittedIntent>,
    pub workflow: Option<WorkflowRef>,
}

impl CausalIdentity {
    /// Start a causal identity anchored to one semantically committed logical
    /// institutional effect.
    #[must_use]
    pub fn new(logical_intent: CommittedIntent) -> Self {
        Self {
            logical_intent,
            attempt: None,
            source_event: None,
            transport_delivery: None,
            caused_by: None,
            workflow: None,
        }
    }

    /// Bind a concrete execution-attempt identity.
    #[must_use]
    pub fn with_attempt(mut self, attempt: AttemptRef) -> Self {
        self.attempt = Some(attempt);
        self
    }

    /// Bind an external/domain source-event identity.
    #[must_use]
    pub fn with_source_event(mut self, source_event: SourceEventRef) -> Self {
        self.source_event = Some(source_event);
        self
    }

    /// Bind a transport-delivery identity.
    #[must_use]
    pub fn with_transport_delivery(mut self, delivery: TransportDeliveryRef) -> Self {
        self.transport_delivery = Some(delivery);
        self
    }

    /// Link a compensating/follow-on effect to the exact committed logical
    /// effect that caused it.
    #[must_use]
    pub fn caused_by(mut self, logical_intent: CommittedIntent) -> Self {
        self.caused_by = Some(logical_intent);
        self
    }

    /// Bind the Business workflow carrying the causal relation.
    #[must_use]
    pub fn in_workflow(mut self, workflow: WorkflowRef) -> Self {
        self.workflow = Some(workflow);
        self
    }
}
