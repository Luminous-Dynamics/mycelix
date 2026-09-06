#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

//! Transport-neutral institutional contracts for Mycelix Business.
//!
//! This crate intentionally does **not** perform I/O, persistence, hashing,
//! signature verification, policy discovery, clock reads, Holochain calls, or
//! external effects. It freezes only semantic non-substitutability and pure
//! structural rules from the Business v0.1 normative corpus.
//!
//! In particular:
//!
//! - logical intent identity != execution-attempt identity != source-event
//!   identity;
//! - evidence/reference possession != authority;
//! - `OutcomeUnknown` != `ProvenNotApplied`;
//! - obligation satisfaction != waiver/breach/termination;
//! - `PowerGrant` state != current per-operation authorization;
//! - observation validation != reconciliation;
//! - workflow progress != closure disposition.

macro_rules! opaque_ref {
    ($name:ident, $doc:literal) => {
        #[doc = $doc]
        #[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
        pub struct $name([u8; 32]);

        impl $name {
            /// Constructs an opaque semantic reference from already-established
            /// identity bytes. This constructor does not confer authority.
            pub const fn new(bytes: [u8; 32]) -> Self {
                Self(bytes)
            }

            /// Returns the opaque identity bytes.
            pub const fn as_bytes(&self) -> &[u8; 32] {
                &self.0
            }
        }
    };
}

opaque_ref!(PartyRef, "Reference to a party owned by an identity/domain boundary.");
opaque_ref!(OrganizationContextRef, "Reference to an institutional organization context.");
opaque_ref!(SubjectRef, "Reference to the subject of an institutional assertion or effect.");
opaque_ref!(NormativeSourceRef, "Reference to the normative source of an obligation or rule.");
opaque_ref!(CommitmentRef, "Reference to a normative performance commitment.");
opaque_ref!(ObligationRef, "Reference to an active institutional obligation.");
opaque_ref!(PowerGrantRef, "Reference to an institutional power grant.");
opaque_ref!(LogicalIntentRef, "Semantic identity of one requested institutional effect.");
opaque_ref!(AttemptRef, "Identity of one concrete attempt to realize a logical intent.");
opaque_ref!(SourceEventRef, "Identity of one external or domain-generated source event.");
opaque_ref!(WorkflowRef, "Reference to one Business orchestration workflow.");
opaque_ref!(EvidenceRef, "Reference to evidence; possession does not imply authority or dereference access.");
opaque_ref!(ReconciliationRef, "Reference to an owning-domain reconciliation result.");
opaque_ref!(AuthorizationDecisionRef, "Reference to an owning-domain per-operation authorization decision.");
opaque_ref!(DomainOutcomeRef, "Reference to an owning-domain operational outcome.");
opaque_ref!(PolicyRef, "Reference to an exact policy identity.");
opaque_ref!(SemanticProfileRef, "Reference to a registered semantic profile.");
opaque_ref!(PredicateSetCommitment, "Commitment to an exact closure predicate set.");
opaque_ref!(DomainOutcomeSetCommitment, "Commitment to an exact set of domain-owned outcomes.");
opaque_ref!(EvidenceSetCommitment, "Commitment to an exact evidence set.");
opaque_ref!(ObligationSetCommitment, "Commitment to an exact obligation/disposition set.");
opaque_ref!(AuthorizationSetCommitment, "Commitment to an exact authorization-decision set.");
opaque_ref!(ExceptionSetCommitment, "Commitment to an exact closure exception set.");

/// Exact semantic profile and version under which a shared value is interpreted.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct SemanticProfileV1 {
    profile: SemanticProfileRef,
    version: u32,
}

impl SemanticProfileV1 {
    pub const fn new(profile: SemanticProfileRef, version: u32) -> Self {
        Self { profile, version }
    }

    pub const fn profile(&self) -> SemanticProfileRef {
        self.profile
    }

    pub const fn version(&self) -> u32 {
        self.version
    }
}

/// Time/freshness boundary supplied by an upstream prerequisite.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ValidityBoundary {
    /// The prerequisite is valid through this inclusive logical time value.
    KnownUntil(u64),
    /// This prerequisite does not impose a validity horizon.
    NotApplicable,
    /// A required freshness boundary is unknown and must fail closed.
    Unknown,
}

/// Effective validity after conservative attenuation of prerequisites.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EffectiveValidity {
    KnownUntil(u64),
    NotApplicable,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ValidityError {
    UnknownRequiredBoundary,
}

/// Returns the minimum known upstream boundary.
///
/// Any `Unknown` input denies rather than inventing a lifetime. If every input
/// is `NotApplicable`, the result is `NotApplicable`.
pub const fn derive_effective_validity<const N: usize>(
    boundaries: &[ValidityBoundary; N],
) -> Result<EffectiveValidity, ValidityError> {
    let mut index = 0;
    let mut minimum: Option<u64> = None;

    while index < N {
        match boundaries[index] {
            ValidityBoundary::Unknown => return Err(ValidityError::UnknownRequiredBoundary),
            ValidityBoundary::NotApplicable => {}
            ValidityBoundary::KnownUntil(value) => {
                minimum = Some(match minimum {
                    Some(current) if current <= value => current,
                    _ => value,
                });
            }
        }
        index += 1;
    }

    match minimum {
        Some(value) => Ok(EffectiveValidity::KnownUntil(value)),
        None => Ok(EffectiveValidity::NotApplicable),
    }
}

/// Causal identity carried by an operation without deciding its semantic hash.
///
/// The owning domain/profile is responsible for establishing the
/// `LogicalIntentRef`; Business does not hash arbitrary payloads to decide
/// semantic equality.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct OperationCausalityV1 {
    logical_intent: LogicalIntentRef,
    attempt: AttemptRef,
    source_event: Option<SourceEventRef>,
    workflow: Option<WorkflowRef>,
    profile: SemanticProfileV1,
}

impl OperationCausalityV1 {
    pub const fn new(
        logical_intent: LogicalIntentRef,
        attempt: AttemptRef,
        source_event: Option<SourceEventRef>,
        workflow: Option<WorkflowRef>,
        profile: SemanticProfileV1,
    ) -> Self {
        Self {
            logical_intent,
            attempt,
            source_event,
            workflow,
            profile,
        }
    }

    pub const fn logical_intent(&self) -> LogicalIntentRef {
        self.logical_intent
    }

    pub const fn attempt(&self) -> AttemptRef {
        self.attempt
    }

    pub const fn source_event(&self) -> Option<SourceEventRef> {
        self.source_event
    }

    pub const fn workflow(&self) -> Option<WorkflowRef> {
        self.workflow
    }

    pub const fn profile(&self) -> SemanticProfileV1 {
        self.profile
    }
}

/// Closed-world classification of whether an attempted effect is known to have
/// been applied.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EffectApplicationStatus {
    Confirmed,
    ProvenNotApplied,
    OutcomeUnknown,
}

/// Structural retry guidance only; never an authorization to execute.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RetryAdmissibility {
    /// The same logical effect is already confirmed; retrying it as the same
    /// effect is denied.
    DeniedAlreadyConfirmed,
    /// A fresh attempt may be considered, but still requires current domain
    /// authority/policy qualification.
    FreshAttemptMayBeEvaluated,
    /// Blind retry is forbidden. The owning domain must establish an exact
    /// idempotency/reconciliation guarantee before another attempt is safe.
    RequiresExactIdempotencyGuarantee,
}

pub const fn classify_retry(status: EffectApplicationStatus) -> RetryAdmissibility {
    match status {
        EffectApplicationStatus::Confirmed => RetryAdmissibility::DeniedAlreadyConfirmed,
        EffectApplicationStatus::ProvenNotApplied => {
            RetryAdmissibility::FreshAttemptMayBeEvaluated
        }
        EffectApplicationStatus::OutcomeUnknown => {
            RetryAdmissibility::RequiresExactIdempotencyGuarantee
        }
    }
}

/// Semantic disposition of an active institutional duty.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ObligationDisposition {
    Active,
    PartiallySatisfied,
    Satisfied,
    Breached,
    Disputed,
    Waived,
    Superseded,
    Terminated,
}

impl ObligationDisposition {
    /// True only when performance itself is accepted as satisfying the duty.
    /// Waiver, termination, or supersession are deliberately not performance.
    pub const fn performance_satisfied(self) -> bool {
        matches!(self, Self::Satisfied)
    }

    /// Returns a non-performance discharge kind when one is explicit.
    pub const fn nonperformance_discharge(self) -> Option<NonPerformanceDischarge> {
        match self {
            Self::Waived => Some(NonPerformanceDischarge::Waiver),
            Self::Superseded => Some(NonPerformanceDischarge::Supersession),
            Self::Terminated => Some(NonPerformanceDischarge::Termination),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NonPerformanceDischarge {
    Waiver,
    Supersession,
    Termination,
}

/// Lifecycle state of a power grant. Active grant state is merely eligible for
/// an owning-domain authorization evaluation; it is not positive authorization.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PowerGrantState {
    Proposed,
    Granted,
    Active,
    Suspended,
    Expired,
    Revoked,
    Superseded,
}

impl PowerGrantState {
    pub const fn eligible_for_authorization_evaluation(self) -> bool {
        matches!(self, Self::Active)
    }
}

/// Lifecycle state of a source-attributed observation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ObservationState {
    Observed,
    Validated,
    Rejected,
    Conflicted,
    Reconciled,
    Superseded,
}

impl ObservationState {
    pub const fn reconciled(self) -> bool {
        matches!(self, Self::Reconciled)
    }
}

/// Progress state of a Business workflow, separate from its closure reason.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WorkflowProgress {
    Proposed,
    Authorized,
    Executing,
    PartiallyPerformed,
    AwaitingEvidence,
    Disputed,
    CompensationRequired,
    Compensating,
    ClosureCandidate,
    Closed,
}

/// Why a consequential workflow became terminal under its exact closure policy.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClosureClass {
    Satisfied,
    Waived,
    Compensated,
    ResolvedWithExceptions,
    Terminated,
}

/// Exact exception-set presence without requiring allocation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExceptionSetV1 {
    None,
    Present(ExceptionSetCommitment),
}

impl ExceptionSetV1 {
    pub const fn is_present(self) -> bool {
        matches!(self, Self::Present(_))
    }
}

/// Structural evidence required to distinguish terminal closure dispositions.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DispositionEvidenceV1 {
    None,
    Waiver(AuthorizationDecisionRef),
    Compensation(DomainOutcomeRef),
    Termination(AuthorizationDecisionRef),
}

/// Exact structural basis for one closure claim.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ClosureBasisV1 {
    workflow: WorkflowRef,
    organization: OrganizationContextRef,
    workflow_profile: SemanticProfileV1,
    closure_policy: PolicyRef,
    closure_policy_version: u32,
    predicate_set: PredicateSetCommitment,
    obligation_set: ObligationSetCommitment,
    outcome_set: DomainOutcomeSetCommitment,
    authorization_set: AuthorizationSetCommitment,
    evidence_set: EvidenceSetCommitment,
    exceptions: ExceptionSetV1,
    disposition_evidence: DispositionEvidenceV1,
}

impl ClosureBasisV1 {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        workflow: WorkflowRef,
        organization: OrganizationContextRef,
        workflow_profile: SemanticProfileV1,
        closure_policy: PolicyRef,
        closure_policy_version: u32,
        predicate_set: PredicateSetCommitment,
        obligation_set: ObligationSetCommitment,
        outcome_set: DomainOutcomeSetCommitment,
        authorization_set: AuthorizationSetCommitment,
        evidence_set: EvidenceSetCommitment,
        exceptions: ExceptionSetV1,
        disposition_evidence: DispositionEvidenceV1,
    ) -> Self {
        Self {
            workflow,
            organization,
            workflow_profile,
            closure_policy,
            closure_policy_version,
            predicate_set,
            obligation_set,
            outcome_set,
            authorization_set,
            evidence_set,
            exceptions,
            disposition_evidence,
        }
    }

    pub const fn workflow(&self) -> WorkflowRef {
        self.workflow
    }

    pub const fn organization(&self) -> OrganizationContextRef {
        self.organization
    }

    pub const fn workflow_profile(&self) -> SemanticProfileV1 {
        self.workflow_profile
    }

    pub const fn closure_policy(&self) -> PolicyRef {
        self.closure_policy
    }

    pub const fn closure_policy_version(&self) -> u32 {
        self.closure_policy_version
    }

    pub const fn predicate_set(&self) -> PredicateSetCommitment {
        self.predicate_set
    }

    pub const fn obligation_set(&self) -> ObligationSetCommitment {
        self.obligation_set
    }

    pub const fn outcome_set(&self) -> DomainOutcomeSetCommitment {
        self.outcome_set
    }

    pub const fn authorization_set(&self) -> AuthorizationSetCommitment {
        self.authorization_set
    }

    pub const fn evidence_set(&self) -> EvidenceSetCommitment {
        self.evidence_set
    }

    pub const fn exceptions(&self) -> ExceptionSetV1 {
        self.exceptions
    }

    pub const fn disposition_evidence(&self) -> DispositionEvidenceV1 {
        self.disposition_evidence
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClosureShapeError {
    NotClosureCandidate,
    SatisfiedWithExceptions,
    SatisfiedWithDispositionOverride,
    WaiverAuthorizationRequired,
    CompensationOutcomeRequired,
    ExceptionsRequired,
    TerminationAuthorizationRequired,
    UnexpectedExceptionsForClosureClass,
}

/// Opaque positive result of **structural** closure validation only.
///
/// This object deliberately does not claim that referenced domain outcomes,
/// authorizations, evidence, or policy were cryptographically/currently
/// verified. A later adapter must establish those facts before institutional
/// closure can be relied upon.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ShapeValidatedClosureClaimV1 {
    closure_class: ClosureClass,
    basis: ClosureBasisV1,
}

impl ShapeValidatedClosureClaimV1 {
    pub const fn closure_class(&self) -> ClosureClass {
        self.closure_class
    }

    pub const fn basis(&self) -> ClosureBasisV1 {
        self.basis
    }
}

/// Validates only non-substitutability and closure-shape rules.
pub const fn validate_closure_shape(
    progress: WorkflowProgress,
    closure_class: ClosureClass,
    basis: ClosureBasisV1,
) -> Result<ShapeValidatedClosureClaimV1, ClosureShapeError> {
    if !matches!(progress, WorkflowProgress::ClosureCandidate) {
        return Err(ClosureShapeError::NotClosureCandidate);
    }

    match closure_class {
        ClosureClass::Satisfied => {
            if basis.exceptions.is_present() {
                return Err(ClosureShapeError::SatisfiedWithExceptions);
            }
            if !matches!(basis.disposition_evidence, DispositionEvidenceV1::None) {
                return Err(ClosureShapeError::SatisfiedWithDispositionOverride);
            }
        }
        ClosureClass::Waived => {
            if basis.exceptions.is_present() {
                return Err(ClosureShapeError::UnexpectedExceptionsForClosureClass);
            }
            if !matches!(
                basis.disposition_evidence,
                DispositionEvidenceV1::Waiver(_)
            ) {
                return Err(ClosureShapeError::WaiverAuthorizationRequired);
            }
        }
        ClosureClass::Compensated => {
            if basis.exceptions.is_present() {
                return Err(ClosureShapeError::UnexpectedExceptionsForClosureClass);
            }
            if !matches!(
                basis.disposition_evidence,
                DispositionEvidenceV1::Compensation(_)
            ) {
                return Err(ClosureShapeError::CompensationOutcomeRequired);
            }
        }
        ClosureClass::ResolvedWithExceptions => {
            if !basis.exceptions.is_present() {
                return Err(ClosureShapeError::ExceptionsRequired);
            }
        }
        ClosureClass::Terminated => {
            if basis.exceptions.is_present() {
                return Err(ClosureShapeError::UnexpectedExceptionsForClosureClass);
            }
            if !matches!(
                basis.disposition_evidence,
                DispositionEvidenceV1::Termination(_)
            ) {
                return Err(ClosureShapeError::TerminationAuthorizationRequired);
            }
        }
    }

    Ok(ShapeValidatedClosureClaimV1 {
        closure_class,
        basis,
    })
}

/// Relationship between an old and new representation/profile interpretation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MigrationKind {
    /// Representation changes while institutional meaning is preserved exactly.
    RepresentationEquivalent,
    /// Institutional meaning changes and explicit lineage/requalification is
    /// therefore required.
    SemanticChange,
}

impl MigrationKind {
    pub const fn may_preserve_logical_identity(self) -> bool {
        matches!(self, Self::RepresentationEquivalent)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn id(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn profile() -> SemanticProfileV1 {
        SemanticProfileV1::new(SemanticProfileRef::new(id(1)), 1)
    }

    fn basis(
        exceptions: ExceptionSetV1,
        disposition_evidence: DispositionEvidenceV1,
    ) -> ClosureBasisV1 {
        ClosureBasisV1::new(
            WorkflowRef::new(id(2)),
            OrganizationContextRef::new(id(3)),
            profile(),
            PolicyRef::new(id(4)),
            1,
            PredicateSetCommitment::new(id(5)),
            ObligationSetCommitment::new(id(6)),
            DomainOutcomeSetCommitment::new(id(7)),
            AuthorizationSetCommitment::new(id(8)),
            EvidenceSetCommitment::new(id(9)),
            exceptions,
            disposition_evidence,
        )
    }

    #[test]
    fn causal_identity_keeps_logical_intent_and_attempt_distinct() {
        let logical = LogicalIntentRef::new(id(10));
        let first = OperationCausalityV1::new(
            logical,
            AttemptRef::new(id(11)),
            None,
            Some(WorkflowRef::new(id(12))),
            profile(),
        );
        let second = OperationCausalityV1::new(
            logical,
            AttemptRef::new(id(13)),
            None,
            Some(WorkflowRef::new(id(12))),
            profile(),
        );

        assert_eq!(first.logical_intent(), second.logical_intent());
        assert_ne!(first.attempt(), second.attempt());
    }

    #[test]
    fn duplicate_source_event_identity_remains_exactly_equal() {
        let source = SourceEventRef::new(id(20));
        let first = OperationCausalityV1::new(
            LogicalIntentRef::new(id(21)),
            AttemptRef::new(id(22)),
            Some(source),
            None,
            profile(),
        );
        let redelivery = OperationCausalityV1::new(
            LogicalIntentRef::new(id(21)),
            AttemptRef::new(id(22)),
            Some(source),
            None,
            profile(),
        );

        assert_eq!(first.source_event(), redelivery.source_event());
    }

    #[test]
    fn effective_validity_is_minimum_known_boundary() {
        let bounds = [
            ValidityBoundary::KnownUntil(500),
            ValidityBoundary::NotApplicable,
            ValidityBoundary::KnownUntil(300),
            ValidityBoundary::KnownUntil(450),
        ];

        assert_eq!(
            derive_effective_validity(&bounds),
            Ok(EffectiveValidity::KnownUntil(300))
        );
    }

    #[test]
    fn unknown_required_validity_fails_closed() {
        let bounds = [
            ValidityBoundary::KnownUntil(500),
            ValidityBoundary::Unknown,
        ];

        assert_eq!(
            derive_effective_validity(&bounds),
            Err(ValidityError::UnknownRequiredBoundary)
        );
    }

    #[test]
    fn unknown_outcome_is_not_retryable_as_proven_non_application() {
        assert_eq!(
            classify_retry(EffectApplicationStatus::OutcomeUnknown),
            RetryAdmissibility::RequiresExactIdempotencyGuarantee
        );
        assert_eq!(
            classify_retry(EffectApplicationStatus::ProvenNotApplied),
            RetryAdmissibility::FreshAttemptMayBeEvaluated
        );
    }

    #[test]
    fn confirmed_effect_denies_same_effect_retry() {
        assert_eq!(
            classify_retry(EffectApplicationStatus::Confirmed),
            RetryAdmissibility::DeniedAlreadyConfirmed
        );
    }

    #[test]
    fn waiver_is_not_performance_satisfaction() {
        assert!(!ObligationDisposition::Waived.performance_satisfied());
        assert_eq!(
            ObligationDisposition::Waived.nonperformance_discharge(),
            Some(NonPerformanceDischarge::Waiver)
        );
        assert!(ObligationDisposition::Satisfied.performance_satisfied());
    }

    #[test]
    fn breach_is_not_implicitly_a_discharge() {
        assert_eq!(ObligationDisposition::Breached.nonperformance_discharge(), None);
        assert!(!ObligationDisposition::Breached.performance_satisfied());
    }

    #[test]
    fn only_active_power_grant_is_eligible_for_further_authorization_evaluation() {
        assert!(PowerGrantState::Active.eligible_for_authorization_evaluation());
        assert!(!PowerGrantState::Granted.eligible_for_authorization_evaluation());
        assert!(!PowerGrantState::Revoked.eligible_for_authorization_evaluation());
        assert!(!PowerGrantState::Expired.eligible_for_authorization_evaluation());
    }

    #[test]
    fn validated_observation_is_not_reconciled() {
        assert!(!ObservationState::Validated.reconciled());
        assert!(ObservationState::Reconciled.reconciled());
    }

    #[test]
    fn satisfied_closure_rejects_exceptions() {
        let result = validate_closure_shape(
            WorkflowProgress::ClosureCandidate,
            ClosureClass::Satisfied,
            basis(
                ExceptionSetV1::Present(ExceptionSetCommitment::new(id(30))),
                DispositionEvidenceV1::None,
            ),
        );

        assert_eq!(result, Err(ClosureShapeError::SatisfiedWithExceptions));
    }

    #[test]
    fn satisfied_closure_rejects_waiver_substitution() {
        let result = validate_closure_shape(
            WorkflowProgress::ClosureCandidate,
            ClosureClass::Satisfied,
            basis(
                ExceptionSetV1::None,
                DispositionEvidenceV1::Waiver(AuthorizationDecisionRef::new(id(31))),
            ),
        );

        assert_eq!(
            result,
            Err(ClosureShapeError::SatisfiedWithDispositionOverride)
        );
    }

    #[test]
    fn waived_closure_requires_waiver_authorization_reference() {
        let denied = validate_closure_shape(
            WorkflowProgress::ClosureCandidate,
            ClosureClass::Waived,
            basis(ExceptionSetV1::None, DispositionEvidenceV1::None),
        );
        assert_eq!(denied, Err(ClosureShapeError::WaiverAuthorizationRequired));

        let accepted = validate_closure_shape(
            WorkflowProgress::ClosureCandidate,
            ClosureClass::Waived,
            basis(
                ExceptionSetV1::None,
                DispositionEvidenceV1::Waiver(AuthorizationDecisionRef::new(id(32))),
            ),
        );
        assert_eq!(accepted.unwrap().closure_class(), ClosureClass::Waived);
    }

    #[test]
    fn compensated_closure_requires_compensation_outcome_reference() {
        let denied = validate_closure_shape(
            WorkflowProgress::ClosureCandidate,
            ClosureClass::Compensated,
            basis(ExceptionSetV1::None, DispositionEvidenceV1::None),
        );
        assert_eq!(denied, Err(ClosureShapeError::CompensationOutcomeRequired));

        let accepted = validate_closure_shape(
            WorkflowProgress::ClosureCandidate,
            ClosureClass::Compensated,
            basis(
                ExceptionSetV1::None,
                DispositionEvidenceV1::Compensation(DomainOutcomeRef::new(id(33))),
            ),
        );
        assert_eq!(accepted.unwrap().closure_class(), ClosureClass::Compensated);
    }

    #[test]
    fn exception_closure_requires_explicit_exception_set() {
        let denied = validate_closure_shape(
            WorkflowProgress::ClosureCandidate,
            ClosureClass::ResolvedWithExceptions,
            basis(ExceptionSetV1::None, DispositionEvidenceV1::None),
        );
        assert_eq!(denied, Err(ClosureShapeError::ExceptionsRequired));

        let accepted = validate_closure_shape(
            WorkflowProgress::ClosureCandidate,
            ClosureClass::ResolvedWithExceptions,
            basis(
                ExceptionSetV1::Present(ExceptionSetCommitment::new(id(34))),
                DispositionEvidenceV1::None,
            ),
        );
        assert_eq!(
            accepted.unwrap().closure_class(),
            ClosureClass::ResolvedWithExceptions
        );
    }

    #[test]
    fn termination_requires_authorization_reference() {
        let denied = validate_closure_shape(
            WorkflowProgress::ClosureCandidate,
            ClosureClass::Terminated,
            basis(ExceptionSetV1::None, DispositionEvidenceV1::None),
        );
        assert_eq!(
            denied,
            Err(ClosureShapeError::TerminationAuthorizationRequired)
        );
    }

    #[test]
    fn workflow_must_reach_closure_candidate_before_structural_closure() {
        let result = validate_closure_shape(
            WorkflowProgress::Executing,
            ClosureClass::Satisfied,
            basis(ExceptionSetV1::None, DispositionEvidenceV1::None),
        );

        assert_eq!(result, Err(ClosureShapeError::NotClosureCandidate));
    }

    #[test]
    fn semantic_change_cannot_preserve_logical_identity_by_default() {
        assert!(MigrationKind::RepresentationEquivalent.may_preserve_logical_identity());
        assert!(!MigrationKind::SemanticChange.may_preserve_logical_identity());
    }
}
