// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact qualification cuts, acyclic derivations, and authorization binding.

use core::fmt;
use std::collections::{BTreeMap, BTreeSet};

use crate::causal::CommittedIntent;
use crate::ids::{
    DerivationNodeRef, DomainAllocationRef, DomainAuthorizationDecisionRef, DomainExceptionRef,
    DomainRef, DomainReconciliationRef, GenerationNamespace, OrganizationContextRef, RecordRef,
    SemanticProfileId, SourceEventRef,
};
#[cfg(test)]
use crate::ids::{ExceptionRef, LogicalIntentRef, OperationCommitment};
use crate::time::{required_horizon, TimestampMs, ValidityEnd, ValidityError, ValidityWindow};

/// Explicit generation binding used when a semantic profile requires one
/// coherent generation for a namespace.
///
/// In v0.1 the namespace is local to the authoritative domain of the enclosing
/// `QualifiedInputRef`. Two different domains may therefore both use a local
/// namespace such as `policy` without creating a false cross-domain conflict.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct GenerationBinding {
    pub namespace: GenerationNamespace,
    pub generation: u64,
}

/// Exact reference to one domain-owned prerequisite record/version.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct QualifiedInputRef {
    pub domain: DomainRef,
    pub record: RecordRef,
    pub version: u64,
    pub semantic_profile: SemanticProfileId,
    pub generation: Option<GenerationBinding>,
    pub validity: ValidityWindow,
}

/// Structural qualification-cut errors.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CutError {
    GenerationConflict {
        domain: DomainRef,
        namespace: GenerationNamespace,
        existing: u64,
        incoming: u64,
    },
    InputNotValidAtQualification {
        domain: DomainRef,
        record: RecordRef,
    },
    UnknownInputFreshness {
        domain: DomainRef,
        record: RecordRef,
    },
}

impl fmt::Display for CutError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::GenerationConflict {
                domain,
                namespace,
                existing,
                incoming,
            } => write!(
                f,
                "generation conflict for {domain}/{namespace}: existing {existing}, incoming {incoming}"
            ),
            Self::InputNotValidAtQualification { domain, record } => {
                write!(f, "input {domain}/{record} is not valid at qualification time")
            }
            Self::UnknownInputFreshness { domain, record } => write!(
                f,
                "input {domain}/{record} has unknown required freshness; qualification fails closed"
            ),
        }
    }
}

impl std::error::Error for CutError {}

/// One exact, deterministic set of inputs used for consequential qualification.
///
/// Unordered collections use `BTreeSet`, making semantic equality independent of
/// insertion/arrival order while preserving exact identities and versions.
/// Domain-owned decision/reconciliation/allocation/exception references are
/// domain-scoped so equal local IDs from different domains cannot collapse.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualificationCut {
    pub organization_context: OrganizationContextRef,
    pub semantic_profile: SemanticProfileId,
    pub qualification_time: TimestampMs,
    inputs: BTreeSet<QualifiedInputRef>,
    source_events: BTreeSet<SourceEventRef>,
    authorization_decisions: BTreeSet<DomainAuthorizationDecisionRef>,
    reconciliations: BTreeSet<DomainReconciliationRef>,
    allocations: BTreeSet<DomainAllocationRef>,
    exceptions: BTreeSet<DomainExceptionRef>,
}

impl QualificationCut {
    /// Construct an empty exact qualification cut.
    #[must_use]
    pub fn new(
        organization_context: OrganizationContextRef,
        semantic_profile: SemanticProfileId,
        qualification_time: TimestampMs,
    ) -> Self {
        Self {
            organization_context,
            semantic_profile,
            qualification_time,
            inputs: BTreeSet::new(),
            source_events: BTreeSet::new(),
            authorization_decisions: BTreeSet::new(),
            reconciliations: BTreeSet::new(),
            allocations: BTreeSet::new(),
            exceptions: BTreeSet::new(),
        }
    }

    /// Insert one exact domain input.
    ///
    /// Conflicting generations in the same domain-local namespace are rejected.
    /// Equal local namespace names from different domains remain independent.
    pub fn insert_input(&mut self, input: QualifiedInputRef) -> Result<bool, CutError> {
        if let Some(incoming) = &input.generation {
            for existing in &self.inputs {
                if let Some(existing_generation) = &existing.generation {
                    if existing.domain == input.domain
                        && existing_generation.namespace == incoming.namespace
                        && existing_generation.generation != incoming.generation
                    {
                        return Err(CutError::GenerationConflict {
                            domain: input.domain.clone(),
                            namespace: incoming.namespace.clone(),
                            existing: existing_generation.generation,
                            incoming: incoming.generation,
                        });
                    }
                }
            }
        }

        Ok(self.inputs.insert(input))
    }

    /// Insert a source-event identity. Duplicate delivery adds no second event.
    #[must_use]
    pub fn insert_source_event(&mut self, source_event: SourceEventRef) -> bool {
        self.source_events.insert(source_event)
    }

    /// Insert a domain-scoped authorization decision reference.
    #[must_use]
    pub fn insert_authorization_decision(
        &mut self,
        decision: DomainAuthorizationDecisionRef,
    ) -> bool {
        self.authorization_decisions.insert(decision)
    }

    /// Insert a domain-scoped reconciliation reference.
    #[must_use]
    pub fn insert_reconciliation(&mut self, reconciliation: DomainReconciliationRef) -> bool {
        self.reconciliations.insert(reconciliation)
    }

    /// Insert a domain-scoped allocation reference.
    #[must_use]
    pub fn insert_allocation(&mut self, allocation: DomainAllocationRef) -> bool {
        self.allocations.insert(allocation)
    }

    /// Insert a domain-scoped retained exception/dispute reference.
    #[cfg(not(test))]
    #[must_use]
    pub fn insert_exception(&mut self, exception: DomainExceptionRef) -> bool {
        self.exceptions.insert(exception)
    }

    /// Legacy in-crate test insertion path for pre-scoping exception fixtures.
    #[cfg(test)]
    #[must_use]
    pub fn insert_exception(&mut self, exception: ExceptionRef) -> bool {
        self.exceptions.insert(exception.into())
    }

    #[must_use]
    pub fn inputs(&self) -> &BTreeSet<QualifiedInputRef> {
        &self.inputs
    }

    #[must_use]
    pub fn source_events(&self) -> &BTreeSet<SourceEventRef> {
        &self.source_events
    }

    #[must_use]
    pub fn authorization_decisions(&self) -> &BTreeSet<DomainAuthorizationDecisionRef> {
        &self.authorization_decisions
    }

    #[must_use]
    pub fn reconciliations(&self) -> &BTreeSet<DomainReconciliationRef> {
        &self.reconciliations
    }

    #[must_use]
    pub fn allocations(&self) -> &BTreeSet<DomainAllocationRef> {
        &self.allocations
    }

    #[must_use]
    pub fn exceptions(&self) -> &BTreeSet<DomainExceptionRef> {
        &self.exceptions
    }

    /// Verify that every declared prerequisite is positively valid at the
    /// exact qualification time.
    pub fn validate_at_qualification_time(&self) -> Result<(), CutError> {
        for input in &self.inputs {
            if matches!(input.validity.valid_until(), ValidityEnd::Unknown) {
                return Err(CutError::UnknownInputFreshness {
                    domain: input.domain.clone(),
                    record: input.record.clone(),
                });
            }

            if !input.validity.contains(self.qualification_time) {
                return Err(CutError::InputNotValidAtQualification {
                    domain: input.domain.clone(),
                    record: input.record.clone(),
                });
            }
        }

        Ok(())
    }

    /// Earliest validity horizon across all exact prerequisite inputs.
    ///
    /// Unknown required freshness fails closed.
    pub fn required_validity_horizon(&self) -> Result<ValidityEnd, ValidityError> {
        required_horizon(self.inputs.iter().map(|input| input.validity.valid_until()))
    }
}

/// Authorization binding errors.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AuthorizationError {
    InvalidCut(CutError),
    InvalidValidity(ValidityError),
    DecisionMissingFromCut {
        decision: DomainAuthorizationDecisionRef,
    },
    DecisionSourceDomainMismatch {
        decision: DomainAuthorizationDecisionRef,
        source_domain: DomainRef,
    },
    DecisionSourceMissingFromCut {
        decision: DomainAuthorizationDecisionRef,
        source: QualifiedInputRef,
    },
    DecisionNotValidAtQualification,
}

impl fmt::Display for AuthorizationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidCut(err) => write!(f, "invalid authorization qualification cut: {err}"),
            Self::InvalidValidity(err) => write!(f, "invalid authorization validity: {err}"),
            Self::DecisionMissingFromCut { decision } => write!(
                f,
                "authorization decision {decision} is not present in the exact qualification cut"
            ),
            Self::DecisionSourceDomainMismatch {
                decision,
                source_domain,
            } => write!(
                f,
                "authorization decision {decision} cannot use source owned by domain {source_domain}"
            ),
            Self::DecisionSourceMissingFromCut { decision, source } => write!(
                f,
                "authorization source {}/{}@{} for decision {decision} is absent from the exact qualification cut",
                source.domain, source.record, source.version
            ),
            Self::DecisionNotValidAtQualification => f.write_str(
                "domain authorization decision is not valid at the qualification time",
            ),
        }
    }
}

impl std::error::Error for AuthorizationError {}

impl From<CutError> for AuthorizationError {
    fn from(value: CutError) -> Self {
        Self::InvalidCut(value)
    }
}

impl From<ValidityError> for AuthorizationError {
    fn from(value: ValidityError) -> Self {
        Self::InvalidValidity(value)
    }
}

/// Domain-owned authorization result bound to one exact committed intent and
/// qualification cut.
///
/// The authorization decision's exact domain-owned source record is retained
/// explicitly. Business does not supply the decision lease; its validity is
/// derived from the exact source result and then attenuated by every prerequisite
/// in the cut.
///
/// Qualified positive objects are sealed in production: callers may inspect
/// fields through accessors but cannot construct or mutate the binding directly.
///
/// ```compile_fail
/// use mycelix_business_core::AuthorizationBinding;
///
/// fn cannot_mutate_basis(binding: &AuthorizationBinding) {
///     let _ = &binding.decision_source;
/// }
/// ```
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuthorizationBinding {
    pub(crate) decision_ref: DomainAuthorizationDecisionRef,
    pub(crate) decision_source: QualifiedInputRef,
    pub(crate) committed_intent: CommittedIntent,
    pub(crate) qualification_cut: QualificationCut,
    pub(crate) effective_valid_until: ValidityEnd,
}

impl AuthorizationBinding {
    /// Domain-owned authorization-decision identity.
    #[must_use]
    pub fn decision_ref(&self) -> &DomainAuthorizationDecisionRef {
        &self.decision_ref
    }

    /// Exact domain-owned source result for the authorization decision.
    #[must_use]
    pub fn decision_source(&self) -> &QualifiedInputRef {
        &self.decision_source
    }

    /// Exact semantically committed operation authorized by this binding.
    #[must_use]
    pub fn committed_intent(&self) -> &CommittedIntent {
        &self.committed_intent
    }

    /// Exact qualification basis retained by the binding.
    #[must_use]
    pub fn qualification_cut(&self) -> &QualificationCut {
        &self.qualification_cut
    }

    /// Decision validity is derived directly from the exact decision source.
    #[must_use]
    pub const fn decision_validity(&self) -> ValidityWindow {
        self.decision_source.validity
    }

    /// Attenuated validity horizon for this exact authorization binding.
    #[must_use]
    pub const fn effective_valid_until(&self) -> ValidityEnd {
        self.effective_valid_until
    }

    /// Public production constructor. The decision source must be an exact
    /// domain-owned input in the same cut. Its validity is authoritative for the
    /// generic composition layer; callers cannot provide a separate lease.
    pub fn bind(
        decision_ref: DomainAuthorizationDecisionRef,
        committed_intent: CommittedIntent,
        decision_source: QualifiedInputRef,
        qualification_cut: QualificationCut,
    ) -> Result<Self, AuthorizationError> {
        Self::bind_inner(
            decision_ref,
            committed_intent,
            decision_source,
            qualification_cut,
        )
    }

    /// Compatibility constructor for this crate's older in-crate unit fixtures.
    /// It is absent from the production API. The supplied legacy validity is
    /// converted into a synthetic exact domain input before strict binding.
    #[cfg(test)]
    pub(crate) fn new(
        decision_ref: DomainAuthorizationDecisionRef,
        logical_intent: LogicalIntentRef,
        operation_profile: SemanticProfileId,
        decision_validity: ValidityWindow,
        mut qualification_cut: QualificationCut,
    ) -> Result<Self, AuthorizationError> {
        if matches!(decision_validity.valid_until(), ValidityEnd::Unknown) {
            return Err(ValidityError::UnknownRequiredEnd.into());
        }

        let _ = qualification_cut.insert_authorization_decision(decision_ref.clone());

        let decision_source = QualifiedInputRef {
            domain: decision_ref.domain().clone(),
            record: RecordRef::new(format!(
                "test-authorization:{}",
                decision_ref.local().as_str()
            ))
            .expect("test-only authorization source id is valid"),
            version: 1,
            semantic_profile: SemanticProfileId::new("test.authorization-decision", 1)
                .expect("static authorization profile is valid"),
            generation: None,
            validity: decision_validity,
        };
        let inserted = qualification_cut
            .insert_input(decision_source.clone())
            .expect("test-only authorization source is generation-compatible");
        debug_assert!(inserted, "test-only authorization source must be unique");

        let semantic_commitment = OperationCommitment::new(format!(
            "test-fixture:{}@{}:{}",
            operation_profile.name(),
            operation_profile.version(),
            logical_intent.as_str()
        ))
        .expect("test-only semantic commitment derived from valid IDs");

        Self::bind_inner(
            decision_ref,
            CommittedIntent::new(logical_intent, operation_profile, semantic_commitment),
            decision_source,
            qualification_cut,
        )
    }

    fn bind_inner(
        decision_ref: DomainAuthorizationDecisionRef,
        committed_intent: CommittedIntent,
        decision_source: QualifiedInputRef,
        qualification_cut: QualificationCut,
    ) -> Result<Self, AuthorizationError> {
        if !qualification_cut
            .authorization_decisions()
            .contains(&decision_ref)
        {
            return Err(AuthorizationError::DecisionMissingFromCut {
                decision: decision_ref,
            });
        }

        if decision_ref.domain() != &decision_source.domain {
            return Err(AuthorizationError::DecisionSourceDomainMismatch {
                decision: decision_ref,
                source_domain: decision_source.domain.clone(),
            });
        }

        if !qualification_cut.inputs().contains(&decision_source) {
            return Err(AuthorizationError::DecisionSourceMissingFromCut {
                decision: decision_ref,
                source: decision_source,
            });
        }

        let decision_validity = decision_source.validity;
        if matches!(decision_validity.valid_until(), ValidityEnd::Unknown) {
            return Err(ValidityError::UnknownRequiredEnd.into());
        }

        if !decision_validity.contains(qualification_cut.qualification_time) {
            return Err(AuthorizationError::DecisionNotValidAtQualification);
        }

        qualification_cut.validate_at_qualification_time()?;

        let prerequisite_horizon = qualification_cut.required_validity_horizon()?;
        let effective_valid_until =
            required_horizon([decision_validity.valid_until(), prerequisite_horizon])?;

        Ok(Self {
            decision_ref,
            decision_source,
            committed_intent,
            qualification_cut,
            effective_valid_until,
        })
    }

    /// Production applicability check. A domain authorization is applicable only
    /// to the exact committed operation it qualified.
    #[must_use]
    pub fn is_applicable_to_committed(
        &self,
        committed_intent: &CommittedIntent,
        at: TimestampMs,
    ) -> bool {
        if &self.committed_intent != committed_intent {
            return false;
        }

        self.is_within_effective_validity(at)
    }

    /// Compatibility applicability check for pre-commitment in-crate unit
    /// fixtures only. This method is absent from the production library API.
    #[cfg(test)]
    #[must_use]
    pub(crate) fn is_applicable_to(
        &self,
        logical_intent: &LogicalIntentRef,
        at: TimestampMs,
    ) -> bool {
        if &self.committed_intent.intent_ref != logical_intent {
            return false;
        }

        self.is_within_effective_validity(at)
    }

    fn is_within_effective_validity(&self, at: TimestampMs) -> bool {
        match ValidityWindow::new(
            self.qualification_cut.qualification_time,
            self.effective_valid_until,
        ) {
            Ok(window) => window.contains(at),
            Err(_) => false,
        }
    }
}

/// Directed qualification dependency graph.
///
/// An edge `conclusion -> prerequisite` means the conclusion depends on that
/// prerequisite. Cycles are forbidden even when nodes live in different
/// services/domains.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DependencyGraph {
    edges: BTreeMap<DerivationNodeRef, BTreeSet<DerivationNodeRef>>,
}

/// Dependency graph errors.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GraphError {
    CycleDetected(DerivationNodeRef),
}

impl fmt::Display for GraphError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::CycleDetected(node) => {
                write!(f, "qualification dependency cycle detected at {node}")
            }
        }
    }
}

impl std::error::Error for GraphError {}

impl DependencyGraph {
    /// Add a node even if it has no dependencies.
    pub fn add_node(&mut self, node: DerivationNodeRef) {
        self.edges.entry(node).or_default();
    }

    /// Add `conclusion -> prerequisite`.
    pub fn add_dependency(
        &mut self,
        conclusion: DerivationNodeRef,
        prerequisite: DerivationNodeRef,
    ) {
        self.edges
            .entry(conclusion)
            .or_default()
            .insert(prerequisite.clone());
        self.edges.entry(prerequisite).or_default();
    }

    /// Validate that no conclusion directly or indirectly justifies itself.
    pub fn validate_acyclic(&self) -> Result<(), GraphError> {
        let mut visiting = BTreeSet::new();
        let mut visited = BTreeSet::new();

        for node in self.edges.keys() {
            if let Some(cycle) = visit(node, &self.edges, &mut visiting, &mut visited) {
                return Err(GraphError::CycleDetected(cycle));
            }
        }

        Ok(())
    }
}

fn visit(
    node: &DerivationNodeRef,
    edges: &BTreeMap<DerivationNodeRef, BTreeSet<DerivationNodeRef>>,
    visiting: &mut BTreeSet<DerivationNodeRef>,
    visited: &mut BTreeSet<DerivationNodeRef>,
) -> Option<DerivationNodeRef> {
    if visited.contains(node) {
        return None;
    }

    if !visiting.insert(node.clone()) {
        return Some(node.clone());
    }

    if let Some(prerequisites) = edges.get(node) {
        for prerequisite in prerequisites {
            if let Some(cycle) = visit(prerequisite, edges, visiting, visited) {
                return Some(cycle);
            }
        }
    }

    visiting.remove(node);
    visited.insert(node.clone());
    None
}
