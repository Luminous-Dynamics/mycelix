// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

#![forbid(unsafe_code)]

//! Reusable proof-carrying closure qualification above `mycelix-business-core`.
//!
//! This crate does not interpret domain truth. It composes exact domain-owned
//! boundary results into one auditable closure proof skeleton while preserving
//! organization/profile scope, closure-class-specific semantics, acyclicity,
//! exact cut membership, policy provenance, proof-schema provenance, required
//! obligation completeness, and the non-strengthening guarantees enforced by the
//! core `WorkflowClosureReceipt`.

use core::fmt;
use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    ClosureClass, ClosureError, ClosurePolicyRef, ClosureRequirements, CompensationBinding,
    DependencyGraph, DerivationNodeRef, DomainObligationRef, DomainReconciliationRef, DomainRef,
    ExceptionBinding, GraphError, ObligationDispositionBinding, OrganizationContextRef,
    QualificationCut, QualifiedInputRef, SemanticProfileId, WorkflowClosureReceipt, WorkflowRef,
};

/// One exact boundary result that can ground a node of a closure dependency DAG.
///
/// Boundary results may appear at leaves or at reachable internal nodes. An
/// accepted domain result can itself depend on another accepted result without
/// ceasing to be an exact cross-domain boundary. The enum is intentionally
/// narrow: these are the reference kinds independently required by GP-002,
/// GP-003, and GP-006.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum QualifiedBoundaryRef {
    /// Exact domain-owned record/version/profile result in the qualification cut.
    Input(QualifiedInputRef),
    /// Exact domain-scoped reconciliation identity in the qualification cut.
    Reconciliation(DomainReconciliationRef),
}

/// Reusable structural requirement for one boundary role in a closure profile.
///
/// This does not interpret the referenced record. It fixes only the reference
/// kind, authoritative-domain namespace, and (for exact inputs) semantic profile
/// that a policy adapter says may occupy the role. Exact record IDs and versions
/// remain instance-specific in `ClosureQualificationBasis`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum QualifiedBoundaryRequirement {
    /// Require an exact input from one domain under one exact semantic profile.
    Input {
        domain: DomainRef,
        semantic_profile: SemanticProfileId,
    },
    /// Require a reconciliation identity owned by one domain.
    Reconciliation { domain: DomainRef },
}

impl QualifiedBoundaryRequirement {
    #[must_use]
    pub fn input(domain: DomainRef, semantic_profile: SemanticProfileId) -> Self {
        Self::Input {
            domain,
            semantic_profile,
        }
    }

    #[must_use]
    pub fn reconciliation(domain: DomainRef) -> Self {
        Self::Reconciliation { domain }
    }

    #[must_use]
    pub fn domain(&self) -> &DomainRef {
        match self {
            Self::Input { domain, .. } | Self::Reconciliation { domain } => domain,
        }
    }

    #[must_use]
    pub fn matches(&self, boundary: &QualifiedBoundaryRef) -> bool {
        match (self, boundary) {
            (
                Self::Input {
                    domain,
                    semantic_profile,
                },
                QualifiedBoundaryRef::Input(input),
            ) => &input.domain == domain && &input.semantic_profile == semantic_profile,
            (
                Self::Reconciliation { domain },
                QualifiedBoundaryRef::Reconciliation(reconciliation),
            ) => reconciliation.domain() == domain,
            _ => false,
        }
    }
}

/// Closure dependency DAG for one reusable closure proof schema.
///
/// Cycle validation is delegated to `mycelix-business-core::DependencyGraph`.
/// This wrapper adds reachability/grounding structure needed to prove that a
/// profile's graph actually describes one closure proof rather than unrelated
/// decorative metadata.
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

    /// Reachable leaf nodes. Every one must be declared as a required boundary
    /// role by the reusable profile, while required boundaries may additionally
    /// appear at reachable internal nodes.
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
/// The profile owns proof topology, boundary roles, and required-obligation roles.
/// A transaction basis therefore cannot weaken the proof by deleting an edge,
/// omitting an accepted internal result, or deciding that a policy-required duty
/// does not count. Exact records, versions, and obligation IDs remain
/// instance-specific.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ClosureQualificationProfile {
    organization_context: OrganizationContextRef,
    qualification_profile: SemanticProfileId,
    closure_policy: ClosurePolicyRef,
    closure_profile: SemanticProfileId,
    closure_class: ClosureClass,
    policy_source: QualifiedInputRef,
    root_node: DerivationNodeRef,
    dependency_graph: ClosureDependencyGraph,
    boundary_requirements: BTreeMap<DerivationNodeRef, QualifiedBoundaryRequirement>,
    required_obligation_roles: BTreeMap<DerivationNodeRef, DomainRef>,
}

impl ClosureQualificationProfile {
    /// Define one reusable organization/policy/class-specific closure profile.
    ///
    /// `required_obligation_roles` maps disposition-boundary nodes to the
    /// authoritative domain of the exact obligation that must occupy that role.
    /// The corresponding basis disposition binding must use the same exact input
    /// as the boundary result at that node.
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        organization_context: OrganizationContextRef,
        qualification_profile: SemanticProfileId,
        closure_policy: ClosurePolicyRef,
        closure_profile: SemanticProfileId,
        closure_class: ClosureClass,
        policy_source: QualifiedInputRef,
        root_node: DerivationNodeRef,
        dependency_graph: ClosureDependencyGraph,
        boundary_requirements: BTreeMap<DerivationNodeRef, QualifiedBoundaryRequirement>,
        required_obligation_roles: BTreeMap<DerivationNodeRef, DomainRef>,
    ) -> Self {
        Self {
            organization_context,
            qualification_profile,
            closure_policy,
            closure_profile,
            closure_class,
            policy_source,
            root_node,
            dependency_graph,
            boundary_requirements,
            required_obligation_roles,
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

    /// Reusable dependency topology owned by this policy/profile/class.
    #[must_use]
    pub fn dependency_graph(&self) -> &ClosureDependencyGraph {
        &self.dependency_graph
    }

    /// Exact set of proof roles that must carry instance-specific provenance.
    #[must_use]
    pub fn boundary_requirements(
        &self,
    ) -> &BTreeMap<DerivationNodeRef, QualifiedBoundaryRequirement> {
        &self.boundary_requirements
    }

    /// Exact set of policy-required obligation roles and owning domains.
    #[must_use]
    pub fn required_obligation_roles(&self) -> &BTreeMap<DerivationNodeRef, DomainRef> {
        &self.required_obligation_roles
    }

    /// Qualify one exact closure basis into a sealed core receipt.
    ///
    /// The generic qualification layer proves only structure, scope, and
    /// provenance. It does not interpret the substantive meaning of any boundary,
    /// obligation disposition, or policy input.
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
        self.dependency_graph.validate_acyclic()?;

        let reachable = self
            .dependency_graph
            .reachable_nodes_from(&self.root_node)
            .ok_or_else(|| ClosureQualificationError::MissingRootNode {
                root: self.root_node.clone(),
            })?;

        if reachable.len() != self.dependency_graph.node_count() {
            return Err(ClosureQualificationError::DisconnectedDependencyGraph {
                root: self.root_node.clone(),
                reachable: reachable.len(),
                total: self.dependency_graph.node_count(),
            });
        }

        let required_nodes = self
            .boundary_requirements
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>();
        let leaves = self
            .dependency_graph
            .reachable_leaves_from(&self.root_node)
            .expect("root presence checked above");

        let ungrounded_leaves = leaves
            .difference(&required_nodes)
            .cloned()
            .collect::<BTreeSet<_>>();
        if !ungrounded_leaves.is_empty() {
            return Err(ClosureQualificationError::ProfileUngroundedLeaves {
                leaves: ungrounded_leaves,
            });
        }

        let unreachable_required = required_nodes
            .difference(&reachable)
            .cloned()
            .collect::<BTreeSet<_>>();
        if !unreachable_required.is_empty() {
            return Err(ClosureQualificationError::ProfileUnreachableBoundaryNodes {
                nodes: unreachable_required,
            });
        }

        for (role, expected_domain) in &self.required_obligation_roles {
            let Some(boundary_requirement) = self.boundary_requirements.get(role) else {
                return Err(ClosureQualificationError::ProfileObligationRoleMissingBoundary {
                    role: role.clone(),
                });
            };
            if boundary_requirement.domain() != expected_domain {
                return Err(ClosureQualificationError::ProfileObligationRoleDomainMismatch {
                    role: role.clone(),
                    expected: expected_domain.clone(),
                    boundary_domain: boundary_requirement.domain().clone(),
                });
            }
        }

        let provided_nodes = basis
            .boundary_results
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>();
        if provided_nodes != required_nodes {
            return Err(ClosureQualificationError::BoundaryRoleSetMismatch {
                required: required_nodes,
                provided: provided_nodes,
            });
        }

        for (node, requirement) in &self.boundary_requirements {
            let boundary = basis
                .boundary_results
                .get(node)
                .expect("exact boundary role-set equality checked above");

            if !requirement.matches(boundary) {
                return Err(ClosureQualificationError::BoundaryRequirementMismatch {
                    node: node.clone(),
                    requirement: requirement.clone(),
                    boundary: boundary.clone(),
                });
            }

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

        let required_obligation_role_nodes = self
            .required_obligation_roles
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>();
        let provided_obligation_role_nodes = basis
            .obligations
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>();
        if provided_obligation_role_nodes != required_obligation_role_nodes {
            return Err(ClosureQualificationError::ObligationRoleSetMismatch {
                required: required_obligation_role_nodes,
                provided: provided_obligation_role_nodes,
            });
        }

        let mut role_by_obligation = BTreeMap::new();
        for (role, obligation) in &basis.obligations {
            if let Some(first_role) = role_by_obligation.insert(obligation.clone(), role.clone()) {
                return Err(ClosureQualificationError::ObligationRoleAliasing {
                    obligation: obligation.clone(),
                    first_role,
                    second_role: role.clone(),
                });
            }
        }

        let required_obligations = basis
            .obligations
            .values()
            .cloned()
            .collect::<BTreeSet<_>>();
        let provided_dispositions = basis
            .disposition_bindings
            .iter()
            .map(|binding| binding.obligation().clone())
            .collect::<BTreeSet<_>>();
        if provided_dispositions != required_obligations {
            return Err(ClosureQualificationError::ObligationDispositionSetMismatch {
                required: required_obligations,
                provided: provided_dispositions,
            });
        }

        let mut closure_required_obligations = BTreeSet::new();
        for (role, expected_domain) in &self.required_obligation_roles {
            let obligation = basis
                .obligations
                .get(role)
                .expect("exact obligation role-set equality checked above");
            if obligation.domain() != expected_domain {
                return Err(ClosureQualificationError::ObligationRoleDomainMismatch {
                    role: role.clone(),
                    expected: expected_domain.clone(),
                    actual: obligation.domain().clone(),
                });
            }

            let boundary = basis
                .boundary_results
                .get(role)
                .expect("obligation role must also be an exact boundary role");
            let QualifiedBoundaryRef::Input(boundary_source) = boundary else {
                return Err(ClosureQualificationError::ObligationDispositionBoundaryNotInput {
                    role: role.clone(),
                });
            };

            let binding = basis
                .disposition_bindings
                .iter()
                .find(|binding| binding.obligation() == obligation)
                .expect("exact disposition obligation-set equality checked above");
            if binding.source() != boundary_source {
                return Err(ClosureQualificationError::ObligationDispositionBoundaryMismatch {
                    role: role.clone(),
                    disposition_source: binding.source().clone(),
                    boundary_source: boundary_source.clone(),
                });
            }

            closure_required_obligations.insert(obligation.clone());
        }

        WorkflowClosureReceipt::new(
            workflow,
            self.closure_policy.clone(),
            self.closure_profile.clone(),
            basis.qualification_cut,
            ClosureRequirements::new(closure_required_obligations),
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
/// The basis cannot choose or rewrite the reusable dependency topology, boundary
/// roles, or which obligation roles count. It supplies exact transaction-specific
/// records/versions and exact obligation identities for those declared roles.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ClosureQualificationBasis {
    pub qualification_cut: QualificationCut,
    pub boundary_results: BTreeMap<DerivationNodeRef, QualifiedBoundaryRef>,
    pub obligations: BTreeMap<DerivationNodeRef, DomainObligationRef>,
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
    ProfileUngroundedLeaves {
        leaves: BTreeSet<DerivationNodeRef>,
    },
    ProfileUnreachableBoundaryNodes {
        nodes: BTreeSet<DerivationNodeRef>,
    },
    ProfileObligationRoleMissingBoundary {
        role: DerivationNodeRef,
    },
    ProfileObligationRoleDomainMismatch {
        role: DerivationNodeRef,
        expected: DomainRef,
        boundary_domain: DomainRef,
    },
    BoundaryRoleSetMismatch {
        required: BTreeSet<DerivationNodeRef>,
        provided: BTreeSet<DerivationNodeRef>,
    },
    BoundaryRequirementMismatch {
        node: DerivationNodeRef,
        requirement: QualifiedBoundaryRequirement,
        boundary: QualifiedBoundaryRef,
    },
    BoundaryInputMissing {
        node: DerivationNodeRef,
        input: QualifiedInputRef,
    },
    BoundaryReconciliationMissing {
        node: DerivationNodeRef,
        reconciliation: DomainReconciliationRef,
    },
    ObligationRoleSetMismatch {
        required: BTreeSet<DerivationNodeRef>,
        provided: BTreeSet<DerivationNodeRef>,
    },
    ObligationRoleAliasing {
        obligation: DomainObligationRef,
        first_role: DerivationNodeRef,
        second_role: DerivationNodeRef,
    },
    ObligationRoleDomainMismatch {
        role: DerivationNodeRef,
        expected: DomainRef,
        actual: DomainRef,
    },
    ObligationDispositionSetMismatch {
        required: BTreeSet<DomainObligationRef>,
        provided: BTreeSet<DomainObligationRef>,
    },
    ObligationDispositionBoundaryNotInput {
        role: DerivationNodeRef,
    },
    ObligationDispositionBoundaryMismatch {
        role: DerivationNodeRef,
        disposition_source: QualifiedInputRef,
        boundary_source: QualifiedInputRef,
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
            Self::ProfileUngroundedLeaves { leaves } => write!(
                f,
                "closure profile has reachable leaf nodes without declared exact boundary roles: {leaves:?}"
            ),
            Self::ProfileUnreachableBoundaryNodes { nodes } => write!(
                f,
                "closure profile declares exact boundary roles on nodes that are not reachable from the closure root: {nodes:?}"
            ),
            Self::ProfileObligationRoleMissingBoundary { role } => write!(
                f,
                "closure profile obligation role {role} is not also a required exact boundary role"
            ),
            Self::ProfileObligationRoleDomainMismatch {
                role,
                expected,
                boundary_domain,
            } => write!(
                f,
                "closure profile obligation role {role} expects domain {expected} but its boundary role expects {boundary_domain}"
            ),
            Self::BoundaryRoleSetMismatch { required, provided } => write!(
                f,
                "closure basis boundary roles {provided:?} do not exactly match profile-required roles {required:?}"
            ),
            Self::BoundaryRequirementMismatch {
                node,
                requirement,
                boundary,
            } => write!(
                f,
                "closure boundary node {node} received {boundary:?}, which does not satisfy profile requirement {requirement:?}"
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
            Self::ObligationRoleSetMismatch { required, provided } => write!(
                f,
                "closure basis obligation roles {provided:?} do not exactly match profile-required roles {required:?}"
            ),
            Self::ObligationRoleAliasing {
                obligation,
                first_role,
                second_role,
            } => write!(
                f,
                "closure obligation roles {first_role} and {second_role} both map to exact obligation {obligation}; implicit normative-duty aliasing is forbidden"
            ),
            Self::ObligationRoleDomainMismatch {
                role,
                expected,
                actual,
            } => write!(
                f,
                "closure obligation role {role} expects domain {expected} but basis supplied {actual}"
            ),
            Self::ObligationDispositionSetMismatch { required, provided } => write!(
                f,
                "closure disposition obligations {provided:?} do not exactly match profile-required exact obligations {required:?}"
            ),
            Self::ObligationDispositionBoundaryNotInput { role } => write!(
                f,
                "closure obligation role {role} must be grounded by an exact input boundary"
            ),
            Self::ObligationDispositionBoundaryMismatch {
                role,
                disposition_source,
                boundary_source,
            } => write!(
                f,
                "closure obligation role {role} disposition source {}/{}@{} does not equal boundary source {}/{}@{}",
                disposition_source.domain,
                disposition_source.record,
                disposition_source.version,
                boundary_source.domain,
                boundary_source.record,
                boundary_source.version
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
