// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

#![forbid(unsafe_code)]

//! Policy-declared terminal-role correspondence above the reusable Business
//! closure qualification algebra.
//!
//! This crate does not interpret refunds, exceptions, disputes, or other domain
//! truth. It proves only that terminal bindings occupy the exact reusable roles
//! declared by a closure policy/profile before delegating final closure semantics
//! to `mycelix-business-qualification` and `mycelix-business-core`.

use core::fmt;
use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    CommittedIntent, CompensationBinding, DerivationNodeRef, DomainExceptionRef,
    DomainObligationRef, ExceptionBinding, ObligationDispositionBinding, QualificationCut,
    QualifiedInputRef, SemanticProfileId, WorkflowClosureReceipt, WorkflowRef,
};
use mycelix_business_qualification::{
    ClosureQualificationBasis, ClosureQualificationError, ClosureQualificationProfile,
    QualifiedBoundaryRef, QualifiedBoundaryRequirement,
};

/// Reusable requirement for one compensating terminal role.
///
/// Source domain and source semantic profile are inherited from the already
/// declared exact input boundary role. This requirement adds the missing
/// operation-semantic constraint without duplicating boundary schema.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CompensationRoleRequirement {
    operation_profile: SemanticProfileId,
}

impl CompensationRoleRequirement {
    #[must_use]
    pub const fn new(operation_profile: SemanticProfileId) -> Self {
        Self { operation_profile }
    }

    #[must_use]
    pub const fn operation_profile(&self) -> &SemanticProfileId {
        &self.operation_profile
    }
}

/// Reusable closure profile with explicit terminal-role correspondence.
///
/// The wrapped profile continues to own organization, policy/profile source,
/// closure class, proof DAG, exact boundary schema, and required obligations.
/// This layer adds only which declared input-boundary roles must be occupied by
/// compensation or retained-exception bindings.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TerminalClosureQualificationProfile {
    base: ClosureQualificationProfile,
    compensation_roles: BTreeMap<DerivationNodeRef, CompensationRoleRequirement>,
    exception_roles: BTreeSet<DerivationNodeRef>,
}

impl TerminalClosureQualificationProfile {
    #[must_use]
    pub fn new(
        base: ClosureQualificationProfile,
        compensation_roles: BTreeMap<DerivationNodeRef, CompensationRoleRequirement>,
        exception_roles: BTreeSet<DerivationNodeRef>,
    ) -> Self {
        Self {
            base,
            compensation_roles,
            exception_roles,
        }
    }

    #[must_use]
    pub const fn base(&self) -> &ClosureQualificationProfile {
        &self.base
    }

    #[must_use]
    pub const fn compensation_roles(
        &self,
    ) -> &BTreeMap<DerivationNodeRef, CompensationRoleRequirement> {
        &self.compensation_roles
    }

    #[must_use]
    pub const fn exception_roles(&self) -> &BTreeSet<DerivationNodeRef> {
        &self.exception_roles
    }

    /// Qualify terminal-role correspondence, then delegate the exact proof basis
    /// to the reusable closure qualifier.
    pub fn qualify(
        &self,
        workflow: WorkflowRef,
        basis: TerminalClosureQualificationBasis,
    ) -> Result<WorkflowClosureReceipt, TerminalClosureQualificationError> {
        self.validate_profile_terminal_roles()?;

        let required_compensation_roles = self
            .compensation_roles
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>();
        let provided_compensation_roles = basis
            .compensation_bindings
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>();
        if provided_compensation_roles != required_compensation_roles {
            return Err(TerminalClosureQualificationError::CompensationRoleSetMismatch {
                required: required_compensation_roles,
                provided: provided_compensation_roles,
            });
        }

        let provided_exception_roles = basis
            .exception_bindings
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>();
        if provided_exception_roles != self.exception_roles {
            return Err(TerminalClosureQualificationError::ExceptionRoleSetMismatch {
                required: self.exception_roles.clone(),
                provided: provided_exception_roles,
            });
        }

        let mut compensation_role_by_intent = BTreeMap::<CommittedIntent, DerivationNodeRef>::new();
        for (role, requirement) in &self.compensation_roles {
            let boundary_source = input_boundary_source(
                &basis.boundary_results,
                role,
                TerminalRoleKind::Compensation,
            )?;
            let binding = basis
                .compensation_bindings
                .get(role)
                .expect("exact compensation role-set equality checked above");

            if binding.source() != boundary_source {
                return Err(
                    TerminalClosureQualificationError::CompensationBoundaryMismatch {
                        role: role.clone(),
                        compensation_source: binding.source().clone(),
                        boundary_source: boundary_source.clone(),
                    },
                );
            }

            if &binding.intent().operation_profile != requirement.operation_profile() {
                return Err(
                    TerminalClosureQualificationError::CompensationOperationProfileMismatch {
                        role: role.clone(),
                        expected: requirement.operation_profile().clone(),
                        actual: binding.intent().operation_profile.clone(),
                    },
                );
            }

            if let Some(first_role) =
                compensation_role_by_intent.insert(binding.intent().clone(), role.clone())
            {
                return Err(TerminalClosureQualificationError::CompensationRoleAliasing {
                    intent: binding.intent().clone(),
                    first_role,
                    second_role: role.clone(),
                });
            }
        }

        let mut exception_role_by_identity =
            BTreeMap::<DomainExceptionRef, DerivationNodeRef>::new();
        for role in &self.exception_roles {
            let boundary_source = input_boundary_source(
                &basis.boundary_results,
                role,
                TerminalRoleKind::Exception,
            )?;
            let binding = basis
                .exception_bindings
                .get(role)
                .expect("exact exception role-set equality checked above");

            if binding.source() != boundary_source {
                return Err(TerminalClosureQualificationError::ExceptionBoundaryMismatch {
                    role: role.clone(),
                    exception_source: binding.source().clone(),
                    boundary_source: boundary_source.clone(),
                });
            }

            if let Some(first_role) = exception_role_by_identity
                .insert(binding.exception().clone(), role.clone())
            {
                return Err(TerminalClosureQualificationError::ExceptionRoleAliasing {
                    exception: binding.exception().clone(),
                    first_role,
                    second_role: role.clone(),
                });
            }
        }

        let closure_basis = ClosureQualificationBasis {
            qualification_cut: basis.qualification_cut,
            boundary_results: basis.boundary_results,
            obligations: basis.obligations,
            disposition_bindings: basis.disposition_bindings,
            exception_bindings: basis.exception_bindings.into_values().collect(),
            compensating_intents: basis.compensation_bindings.into_values().collect(),
        };

        self.base
            .qualify(workflow, closure_basis)
            .map_err(TerminalClosureQualificationError::ClosureQualification)
    }

    fn validate_profile_terminal_roles(
        &self,
    ) -> Result<(), TerminalClosureQualificationError> {
        for role in self.compensation_roles.keys() {
            require_input_boundary(&self.base, role, TerminalRoleKind::Compensation)?;
        }
        for role in &self.exception_roles {
            require_input_boundary(&self.base, role, TerminalRoleKind::Exception)?;
        }
        Ok(())
    }
}

/// Exact transaction-specific terminal closure basis.
///
/// Terminal bindings are keyed by reusable role. The transaction may choose the
/// exact committed operation / exception identity, but it cannot choose which
/// declared boundary is allowed to prove that role.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TerminalClosureQualificationBasis {
    pub qualification_cut: QualificationCut,
    pub boundary_results: BTreeMap<DerivationNodeRef, QualifiedBoundaryRef>,
    pub obligations: BTreeMap<DerivationNodeRef, DomainObligationRef>,
    pub disposition_bindings: Vec<ObligationDispositionBinding>,
    pub compensation_bindings: BTreeMap<DerivationNodeRef, CompensationBinding>,
    pub exception_bindings: BTreeMap<DerivationNodeRef, ExceptionBinding>,
}

/// Terminal-role category used in structural profile diagnostics.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TerminalRoleKind {
    Compensation,
    Exception,
}

fn require_input_boundary(
    profile: &ClosureQualificationProfile,
    role: &DerivationNodeRef,
    kind: TerminalRoleKind,
) -> Result<(), TerminalClosureQualificationError> {
    let Some(requirement) = profile.boundary_requirements().get(role) else {
        return Err(TerminalClosureQualificationError::TerminalRoleMissingBoundary {
            role: role.clone(),
            kind,
        });
    };

    if !matches!(requirement, QualifiedBoundaryRequirement::Input { .. }) {
        return Err(TerminalClosureQualificationError::TerminalRoleBoundaryNotInput {
            role: role.clone(),
            kind,
        });
    }

    Ok(())
}

fn input_boundary_source<'a>(
    boundary_results: &'a BTreeMap<DerivationNodeRef, QualifiedBoundaryRef>,
    role: &DerivationNodeRef,
    kind: TerminalRoleKind,
) -> Result<&'a QualifiedInputRef, TerminalClosureQualificationError> {
    let Some(boundary) = boundary_results.get(role) else {
        return Err(TerminalClosureQualificationError::TerminalRoleBoundaryMissingFromBasis {
            role: role.clone(),
            kind,
        });
    };

    let QualifiedBoundaryRef::Input(input) = boundary else {
        return Err(TerminalClosureQualificationError::TerminalRoleBoundaryNotInput {
            role: role.clone(),
            kind,
        });
    };
    Ok(input)
}

/// Structural terminal-role failures before reusable closure qualification.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TerminalClosureQualificationError {
    TerminalRoleMissingBoundary {
        role: DerivationNodeRef,
        kind: TerminalRoleKind,
    },
    TerminalRoleBoundaryNotInput {
        role: DerivationNodeRef,
        kind: TerminalRoleKind,
    },
    TerminalRoleBoundaryMissingFromBasis {
        role: DerivationNodeRef,
        kind: TerminalRoleKind,
    },
    CompensationRoleSetMismatch {
        required: BTreeSet<DerivationNodeRef>,
        provided: BTreeSet<DerivationNodeRef>,
    },
    ExceptionRoleSetMismatch {
        required: BTreeSet<DerivationNodeRef>,
        provided: BTreeSet<DerivationNodeRef>,
    },
    CompensationBoundaryMismatch {
        role: DerivationNodeRef,
        compensation_source: QualifiedInputRef,
        boundary_source: QualifiedInputRef,
    },
    CompensationOperationProfileMismatch {
        role: DerivationNodeRef,
        expected: SemanticProfileId,
        actual: SemanticProfileId,
    },
    CompensationRoleAliasing {
        intent: CommittedIntent,
        first_role: DerivationNodeRef,
        second_role: DerivationNodeRef,
    },
    ExceptionBoundaryMismatch {
        role: DerivationNodeRef,
        exception_source: QualifiedInputRef,
        boundary_source: QualifiedInputRef,
    },
    ExceptionRoleAliasing {
        exception: DomainExceptionRef,
        first_role: DerivationNodeRef,
        second_role: DerivationNodeRef,
    },
    ClosureQualification(ClosureQualificationError),
}

impl fmt::Display for TerminalClosureQualificationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TerminalRoleMissingBoundary { role, kind } => write!(
                f,
                "{kind:?} terminal role {role} is not a declared reusable boundary role"
            ),
            Self::TerminalRoleBoundaryNotInput { role, kind } => write!(
                f,
                "{kind:?} terminal role {role} must be grounded by an exact input boundary"
            ),
            Self::TerminalRoleBoundaryMissingFromBasis { role, kind } => write!(
                f,
                "{kind:?} terminal role {role} has no exact boundary result in the transaction basis"
            ),
            Self::CompensationRoleSetMismatch { required, provided } => write!(
                f,
                "compensation role set {provided:?} does not exactly match policy-required roles {required:?}"
            ),
            Self::ExceptionRoleSetMismatch { required, provided } => write!(
                f,
                "exception role set {provided:?} does not exactly match policy-required roles {required:?}"
            ),
            Self::CompensationBoundaryMismatch {
                role,
                compensation_source,
                boundary_source,
            } => write!(
                f,
                "compensation role {role} uses source {}/{}@{} but its declared boundary source is {}/{}@{}",
                compensation_source.domain,
                compensation_source.record,
                compensation_source.version,
                boundary_source.domain,
                boundary_source.record,
                boundary_source.version
            ),
            Self::CompensationOperationProfileMismatch {
                role,
                expected,
                actual,
            } => write!(
                f,
                "compensation role {role} requires operation profile {expected} but binding uses {actual}"
            ),
            Self::CompensationRoleAliasing {
                intent,
                first_role,
                second_role,
            } => write!(
                f,
                "compensation roles {first_role} and {second_role} both map to committed intent {}",
                intent.intent_ref
            ),
            Self::ExceptionBoundaryMismatch {
                role,
                exception_source,
                boundary_source,
            } => write!(
                f,
                "exception role {role} uses source {}/{}@{} but its declared boundary source is {}/{}@{}",
                exception_source.domain,
                exception_source.record,
                exception_source.version,
                boundary_source.domain,
                boundary_source.record,
                boundary_source.version
            ),
            Self::ExceptionRoleAliasing {
                exception,
                first_role,
                second_role,
            } => write!(
                f,
                "exception roles {first_role} and {second_role} both map to exact exception {exception}"
            ),
            Self::ClosureQualification(err) => {
                write!(f, "closure qualification failed after terminal-role validation: {err}")
            }
        }
    }
}

impl std::error::Error for TerminalClosureQualificationError {}
