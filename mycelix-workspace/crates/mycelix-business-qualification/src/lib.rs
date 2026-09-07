// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

#![forbid(unsafe_code)]

//! Reusable proof-carrying closure qualification above `mycelix-business-core`.
//!
//! This crate does not interpret domain truth. It composes exact domain-owned
//! boundary results into one auditable closure proof skeleton while preserving
//! organization/profile scope, closure-class-specific semantics, acyclicity,
//! exact cut membership, policy provenance, and the non-strengthening guarantees
//! enforced by the core `WorkflowClosureReceipt`.

use core::fmt;
use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    ClosureClass, ClosureError, ClosurePolicyRef, ClosureRequirements, CompensationBinding,
    DependencyGraph, DerivationNodeRef, DomainReconciliationRef, ExceptionBinding, GraphError,
    ObligationDispositionBinding, OrganizationContextRef, QualificationCut, QualifiedInputRef,
    SemanticProfileId, WorkflowClosureReceipt, WorkflowRef,
};

/// One exact boundary result that can ground a leaf of a closure dependency DAG.
///
/// The enum is intentionally narrow. These are the boundary kinds independently
/// observed in GP-002, GP-003, and GP-006. New variants should be added only when
/// another Golden Path demonstrates that a distinct cut-level reference kind is
/// materially required.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum QualifiedBoundaryRef {
    /// Exact domain-owned record/version/profile result in the qualification cut.
    Input(QualifiedInputRef),
    /// Exact domain-scoped reconciliation identity in the qualification cut.
    Reconciliation(DomainReconciliationRef),
}

/// Closure dependency DAG with enough structure to bind graph leaves to exact
/// qualified boundary results.
///
/// Cycle validation is delegated to `mycelix-business-core::DependencyGraph`.
/// This wrapper adds reachability/leaf structure needed to prove that the graph
/// actually describes the closure basis rather than being unrelated metadata.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ClosureDependencyGraph {
    edges: BTreeMap<DerivationNodeRef, BTreeSet<DerivationNodeRef>>,
}

impl ClosureDependencyGraph {
    /// Add one node even when it has no prerequisites.
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

    /// Number of explicitly represented nodes.
    #[must_use]
    pub fn node_count(&self) -> usize {
        self.edges.len()
    }

    /// Validate acyclicity using the institutional core's graph checker.
    pub fn validate_acyclic(&self) -> Result<(), GraphError> {
        let mut graph = DependencyGraph::default();
        for (conclusion, prerequisites) in &self.edges {
            graph.add_node(conclusion.clone());
            for prerequisite in prerequisites {
                graph.add_dependency(conclusion.clone(), prerequisite.clone());
            }
        }
        graph.validate_acyclic()
    }

    /// Reachable nodes from one conclusion, including the conclusion itself.
    #[must_use]
    pub fn reachable_nodes_from(
        &self,
        root: &DerivationNodeRef,
    ) -> Option<BTreeSet<DerivationNodeRef>> {
        if !self.edges.contains_key(root) {
            return None;
        }

        let mut reachable = BTreeSet::new();
        let mut pending = vec![root.clone()];
        while let Some(node) = pending.pop() {
            if !reachable.insert(node.clone()) {
                continue;
            }
            if let Some(prerequisites) = self.edges.get(&node) {
                pending.extend(prerequisites.iter().cloned());
            }
        }
        Some(reachable)
    }

    /// Reachable leaf nodes whose proof must be grounded by exact boundary refs.
    #[must_use]
    pub fn reachable_leaves_from(
        &self,
        root: &DerivationNodeRef,
    ) -> Option<BTreeSet<DerivationNodeRef>> {
        let reachable = self.reachable_nodes_from(root)?;
        Some(
            reachable
                .into_iter()
                .filter(|node| self.edges.get(node).is_some_and(BTreeSet::is_empty))
                .collect(),
        )
    }
}

/// Reusable structural semantics for one organization/policy/class closure path.
///
/// A profile does not contain transaction-specific business results, but it is
/// anchored to one exact policy/profile source supplied by the owning policy
/// adapter/domain. The generic qualification layer does not interpret that
/// source; it only requires that the source is present and current in the exact
/// qualification cut so profile selection cannot float free of provenance.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ClosureQualificationProfile {
    organization_context: OrganizationContextRef,
    qualification_profile: SemanticProfileId,
    closure_policy: ClosurePolicyRef,
    closure_profile: SemanticProfileId,
    closure_class: ClosureClass,
    policy_source: QualifiedInputRef,
    root_node: DerivationNodeRef,
}

impl ClosureQualificationProfile {
    /// Define one reusable organization/policy/class-specific closure profile.
    ///
    /// `policy_source` is the exact owning-domain result that the policy adapter
    /// asserts corresponds to this policy/profile/class definition. Business
    /// checks provenance/currentness but does not interpret that correspondence.
    #[must_use]
    pub fn new(
        organization_context: OrganizationContextRef,
        qualification_profile: SemanticProfileId,
        closure_policy: ClosurePolicyRef,
        closure_profile: SemanticProfileId,
        closure_class: ClosureClass,
        policy_source: QualifiedInputRef,
        root_node: DerivationNodeRef,
    ) -> Self {
        Self {
            organization_context,
            qualification_profile,
            closure_policy,
            closure_profile,
            closure_class,
            policy_source,
            root_node,
        }
    }

    #[must_use]
    pub fn organization_context(&self) -> &OrganizationContextRef {
        &self.organization_context
    }

    #[must_use]
    pub fn qualification_profile(&self) -> &SemanticProfileId {
        &self.qualification_profile
    }

    #[must_use]
    pub fn closure_policy(&self) -> &ClosurePolicyRef {
        &self.closure_policy
    }

    #[must_use]
    pub fn closure_profile(&self) -> &SemanticProfileId {
        &self.closure_profile
    }

    #[must_use]
    pub const fn closure_class(&self) -> ClosureClass {
        self.closure_class
    }

    /// Exact domain-owned result grounding this policy/profile selection.
    #[must_use]
    pub fn policy_source(&self) -> &QualifiedInputRef {
        &self.policy_source
    }

    #[must_use]
    pub fn root_node(&self) -> &DerivationNodeRef {
        &self.root_node
    }

    /// Qualify one exact closure basis into a sealed core receipt.
    ///
    /// The generic qualification layer proves only structure and provenance. It
    /// does not interpret the substantive meaning of any boundary or policy input.
    pub fn qualify(
        &self,
        workflow: WorkflowRef,
        basis: ClosureQualificationBasis,
    ) -> Result<WorkflowClosureReceipt, ClosureQualificationError> {
        if basis.qualification_cut.organization_context != self.organization_context {
            return Err(ClosureQualificationError::WrongOrganization {
                expected: self.organization_context.clone(),
                actual: basis.qualification_cut.organization_context.clone(),
            });
        }

        if basis.qualification_cut.semantic_profile != self.qualification_profile {
            return Err(ClosureQualificationError::WrongQualificationProfile {
                expected: self.qualification_profile.clone(),
                actual: basis.qualification_cut.semantic_profile.clone(),
            });
        }

        if !basis.qualification_cut.inputs().contains(&self.policy_source) {
            return Err(ClosureQualificationError::PolicySourceMissingFromCut {
                source: self.policy_source.clone(),
            });
        }

        basis.qualification_cut.validate_at_qualification_time()?;
        basis.dependency_graph.validate_acyclic()?;

        let reachable = basis
            .dependency_graph
            .reachable_nodes_from(&self.root_node)
            .ok_or_else(|| ClosureQualificationError::MissingRootNode {
                root: self.root_node.clone(),
            })?;

        if reachable.len() != basis.dependency_graph.node_count() {
            return Err(ClosureQualificationError::DisconnectedDependencyGraph {
                root: self.root_node.clone(),
                reachable: reachable.len(),
                total: basis.dependency_graph.node_count(),
            });
        }

        let leaves = basis
            .dependency_graph
            .reachable_leaves_from(&self.root_node)
            .expect("root presence checked above");
        let bound_nodes = basis.boundary_results.keys().cloned().collect::<BTreeSet<_>>();
        if leaves != bound_nodes {
            return Err(ClosureQualificationError::BoundaryLeafMismatch {
                graph_leaves: leaves,
                bound_nodes,
            });
        }

        for (node, boundary) in &basis.boundary_results {
            match boundary {
                QualifiedBoundaryRef::Input(input) => {
                    if !basis.qualification_cut.inputs().contains(input) {
                        return Err(ClosureQualificationError::BoundaryInputMissing {
                            node: node.clone(),
                            input: input.clone(),
                        });
                    }
                }
                QualifiedBoundaryRef::Reconciliation(reconciliation) => {
                    if !basis
                        .qualification_cut
                        .reconciliations()
                        .contains(reconciliation)
                    {
                        return Err(ClosureQualificationError::BoundaryReconciliationMissing {
                            node: node.clone(),
                            reconciliation: reconciliation.clone(),
                        });
                    }
                }
            }
        }

        WorkflowClosureReceipt::new(
            workflow,
            self.closure_policy.clone(),
            self.closure_profile.clone(),
            basis.qualification_cut,
            basis.closure_requirements,
            self.closure_class,
            basis.disposition_bindings,
            basis.exception_bindings,
            basis.compensating_intents,
        )
        .map_err(Into::into)
    }
}

/// Exact instance-specific basis consumed by one closure qualification.
///
/// This is candidate input, not a positive proof object. It is intentionally
/// constructible by policy code and consumed by `ClosureQualificationProfile`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ClosureQualificationBasis {
    pub qualification_cut: QualificationCut,
    pub dependency_graph: ClosureDependencyGraph,
    pub boundary_results: BTreeMap<DerivationNodeRef, QualifiedBoundaryRef>,
    pub closure_requirements: ClosureRequirements,
    pub disposition_bindings: Vec<ObligationDispositionBinding>,
    pub exception_bindings: Vec<ExceptionBinding>,
    pub compensating_intents: BTreeSet<CompensationBinding>,
}

/// Structural failures before the core closure receipt can be minted.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ClosureQualificationError {
    WrongOrganization {
        expected: OrganizationContextRef,
        actual: OrganizationContextRef,
    },
    WrongQualificationProfile {
        expected: SemanticProfileId,
        actual: SemanticProfileId,
    },
    PolicySourceMissingFromCut {
        source: QualifiedInputRef,
    },
    InvalidCut(mycelix_business_core::CutError),
    Dependency(GraphError),
    MissingRootNode {
        root: DerivationNodeRef,
    },
    DisconnectedDependencyGraph {
        root: DerivationNodeRef,
        reachable: usize,
        total: usize,
    },
    BoundaryLeafMismatch {
        graph_leaves: BTreeSet<DerivationNodeRef>,
        bound_nodes: BTreeSet<DerivationNodeRef>,
    },
    BoundaryInputMissing {
        node: DerivationNodeRef,
        input: QualifiedInputRef,
    },
    BoundaryReconciliationMissing {
        node: DerivationNodeRef,
        reconciliation: DomainReconciliationRef,
    },
    Closure(ClosureError),
}

impl fmt::Display for ClosureQualificationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongOrganization { expected, actual } => write!(
                f,
                "closure qualification expected organization {expected} but cut is scoped to {actual}"
            ),
            Self::WrongQualificationProfile { expected, actual } => write!(
                f,
                "closure qualification expected semantic profile {expected} but cut uses {actual}"
            ),
            Self::PolicySourceMissingFromCut { source } => write!(
                f,
                "closure profile source {}/{}@{} is absent from the exact qualification cut",
                source.domain, source.record, source.version
            ),
            Self::InvalidCut(err) => write!(f, "invalid exact closure qualification cut: {err}"),
            Self::Dependency(err) => write!(f, "invalid closure dependency graph: {err}"),
            Self::MissingRootNode { root } => {
                write!(f, "closure dependency graph does not contain root node {root}")
            }
            Self::DisconnectedDependencyGraph {
                root,
                reachable,
                total,
            } => write!(
                f,
                "closure dependency graph rooted at {root} reaches {reachable} of {total} nodes"
            ),
            Self::BoundaryLeafMismatch {
                graph_leaves,
                bound_nodes,
            } => write!(
                f,
                "closure graph leaves {graph_leaves:?} do not exactly match bound boundary nodes {bound_nodes:?}"
            ),
            Self::BoundaryInputMissing { node, input } => write!(
                f,
                "closure boundary node {node} references missing exact input {}/{}@{}",
                input.domain, input.record, input.version
            ),
            Self::BoundaryReconciliationMissing {
                node,
                reconciliation,
            } => write!(
                f,
                "closure boundary node {node} references missing reconciliation {reconciliation}"
            ),
            Self::Closure(err) => write!(f, "core closure qualification failed: {err}"),
        }
    }
}

impl std::error::Error for ClosureQualificationError {}

impl From<mycelix_business_core::CutError> for ClosureQualificationError {
    fn from(value: mycelix_business_core::CutError) -> Self {
        Self::InvalidCut(value)
    }
}

impl From<GraphError> for ClosureQualificationError {
    fn from(value: GraphError) -> Self {
        Self::Dependency(value)
    }
}

impl From<ClosureError> for ClosureQualificationError {
    fn from(value: ClosureError) -> Self {
        Self::Closure(value)
    }
}
