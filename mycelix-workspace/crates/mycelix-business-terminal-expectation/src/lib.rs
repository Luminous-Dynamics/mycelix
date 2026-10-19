// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

#![forbid(unsafe_code)]

//! Instance-specific terminal expectation provenance above the reusable Business
//! terminal-role qualification layer.
//!
//! A terminal result may occupy the correct policy-declared source role and use
//! the correct operation semantic profile while still representing the wrong
//! transaction-specific effect. This layer binds the expected terminal identity
//! to an independent exact authoritative boundary result and requires the actual
//! terminal binding to match it exactly.
//!
//! The owning domain/profile remains responsible for interpreting an expectation
//! source into the expected `CommittedIntent` or `DomainExceptionRef`. This crate
//! proves exact provenance, independence, and equality; it does not interpret
//! refund adequacy, dispute legitimacy, or domain truth.

use core::fmt;
use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    CommittedIntent, DerivationNodeRef, DomainExceptionRef, QualificationCut, QualifiedInputRef,
    WorkflowClosureReceipt, WorkflowRef,
};
use mycelix_business_qualification::{QualifiedBoundaryRef, QualifiedBoundaryRequirement};
use mycelix_business_terminal_qualification::{
    TerminalClosureQualificationBasis, TerminalClosureQualificationError,
    TerminalClosureQualificationProfile,
};

/// Adapter-supplied interpretation of one exact authoritative result as the
/// expected instance-specific compensating operation.
///
/// The source is structural provenance only. The generic Business layer does not
/// interpret the source record and therefore cannot establish by itself that the
/// record substantively means `expected_intent`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CompensationExpectationBinding {
    expected_intent: CommittedIntent,
    source: QualifiedInputRef,
}

impl CompensationExpectationBinding {
    pub fn bind(
        expected_intent: CommittedIntent,
        source: QualifiedInputRef,
        qualification_cut: &QualificationCut,
    ) -> Result<Self, TerminalExpectationBindingError> {
        if !qualification_cut.inputs().contains(&source) {
            return Err(TerminalExpectationBindingError::SourceMissingFromCut {
                source,
            });
        }
        Ok(Self {
            expected_intent,
            source,
        })
    }

    #[must_use]
    pub const fn expected_intent(&self) -> &CommittedIntent {
        &self.expected_intent
    }

    #[must_use]
    pub const fn source(&self) -> &QualifiedInputRef {
        &self.source
    }
}

/// Adapter-supplied interpretation of one exact authoritative result as the
/// expected instance-specific retained exception identity.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExceptionExpectationBinding {
    expected_exception: DomainExceptionRef,
    source: QualifiedInputRef,
}

impl ExceptionExpectationBinding {
    pub fn bind(
        expected_exception: DomainExceptionRef,
        source: QualifiedInputRef,
        qualification_cut: &QualificationCut,
    ) -> Result<Self, TerminalExpectationBindingError> {
        if !qualification_cut.inputs().contains(&source) {
            return Err(TerminalExpectationBindingError::SourceMissingFromCut {
                source,
            });
        }
        Ok(Self {
            expected_exception,
            source,
        })
    }

    #[must_use]
    pub const fn expected_exception(&self) -> &DomainExceptionRef {
        &self.expected_exception
    }

    #[must_use]
    pub const fn source(&self) -> &QualifiedInputRef {
        &self.source
    }
}

/// Structural expectation-binding construction failures.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TerminalExpectationBindingError {
    SourceMissingFromCut { source: QualifiedInputRef },
}

impl fmt::Display for TerminalExpectationBindingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SourceMissingFromCut { source } => write!(
                f,
                "terminal expectation source {}/{}@{} is absent from the exact qualification cut",
                source.domain, source.record, source.version
            ),
        }
    }
}

impl std::error::Error for TerminalExpectationBindingError {}

/// Reusable requirement linking one compensation terminal role to the exact
/// boundary role whose owning adapter supplies the instance-specific expectation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CompensationExpectationRequirement {
    expectation_role: DerivationNodeRef,
}

impl CompensationExpectationRequirement {
    #[must_use]
    pub const fn new(expectation_role: DerivationNodeRef) -> Self {
        Self { expectation_role }
    }

    #[must_use]
    pub const fn expectation_role(&self) -> &DerivationNodeRef {
        &self.expectation_role
    }
}

/// Reusable requirement linking one exception terminal role to the exact
/// boundary role whose owning adapter supplies the expected exception identity.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExceptionExpectationRequirement {
    expectation_role: DerivationNodeRef,
}

impl ExceptionExpectationRequirement {
    #[must_use]
    pub const fn new(expectation_role: DerivationNodeRef) -> Self {
        Self { expectation_role }
    }

    #[must_use]
    pub const fn expectation_role(&self) -> &DerivationNodeRef {
        &self.expectation_role
    }
}

/// Terminal qualification profile with independent instance-expectation roles.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExpectedTerminalClosureProfile {
    base: TerminalClosureQualificationProfile,
    compensation_expectations:
        BTreeMap<DerivationNodeRef, CompensationExpectationRequirement>,
    exception_expectations: BTreeMap<DerivationNodeRef, ExceptionExpectationRequirement>,
}

impl ExpectedTerminalClosureProfile {
    #[must_use]
    pub fn new(
        base: TerminalClosureQualificationProfile,
        compensation_expectations: BTreeMap<
            DerivationNodeRef,
            CompensationExpectationRequirement,
        >,
        exception_expectations: BTreeMap<DerivationNodeRef, ExceptionExpectationRequirement>,
    ) -> Self {
        Self {
            base,
            compensation_expectations,
            exception_expectations,
        }
    }

    #[must_use]
    pub const fn base(&self) -> &TerminalClosureQualificationProfile {
        &self.base
    }

    #[must_use]
    pub const fn compensation_expectations(
        &self,
    ) -> &BTreeMap<DerivationNodeRef, CompensationExpectationRequirement> {
        &self.compensation_expectations
    }

    #[must_use]
    pub const fn exception_expectations(
        &self,
    ) -> &BTreeMap<DerivationNodeRef, ExceptionExpectationRequirement> {
        &self.exception_expectations
    }

    pub fn qualify(
        &self,
        workflow: WorkflowRef,
        basis: ExpectedTerminalClosureBasis,
    ) -> Result<WorkflowClosureReceipt, TerminalExpectationError> {
        self.validate_profile_expectations()?;

        let required_compensation_roles = self
            .base
            .compensation_roles()
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>();
        let declared_compensation_expectation_roles = self
            .compensation_expectations
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>();
        if declared_compensation_expectation_roles != required_compensation_roles {
            return Err(TerminalExpectationError::CompensationExpectationProfileSetMismatch {
                terminal_roles: required_compensation_roles,
                expectation_roles: declared_compensation_expectation_roles,
            });
        }

        let required_exception_roles = self.base.exception_roles().clone();
        let declared_exception_expectation_roles = self
            .exception_expectations
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>();
        if declared_exception_expectation_roles != required_exception_roles {
            return Err(TerminalExpectationError::ExceptionExpectationProfileSetMismatch {
                terminal_roles: required_exception_roles,
                expectation_roles: declared_exception_expectation_roles,
            });
        }

        let provided_compensation_expectations = basis
            .compensation_expectations
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>();
        let required_compensation_expectations = self
            .compensation_expectations
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>();
        if provided_compensation_expectations != required_compensation_expectations {
            return Err(TerminalExpectationError::CompensationExpectationSetMismatch {
                required: required_compensation_expectations,
                provided: provided_compensation_expectations,
            });
        }

        let provided_exception_expectations = basis
            .exception_expectations
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>();
        let required_exception_expectations = self
            .exception_expectations
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>();
        if provided_exception_expectations != required_exception_expectations {
            return Err(TerminalExpectationError::ExceptionExpectationSetMismatch {
                required: required_exception_expectations,
                provided: provided_exception_expectations,
            });
        }

        for (terminal_role, requirement) in &self.compensation_expectations {
            let expectation = basis
                .compensation_expectations
                .get(terminal_role)
                .expect("exact compensation expectation role-set equality checked above");
            let expected_source = exact_input_at_role(
                &basis.terminal_basis.boundary_results,
                requirement.expectation_role(),
            )?;
            if expectation.source() != expected_source {
                return Err(TerminalExpectationError::CompensationExpectationSourceMismatch {
                    terminal_role: terminal_role.clone(),
                    expectation_source: expectation.source().clone(),
                    boundary_source: expected_source.clone(),
                });
            }

            let actual = basis
                .terminal_basis
                .compensation_bindings
                .get(terminal_role)
                .ok_or_else(|| TerminalExpectationError::MissingCompensationBinding {
                    terminal_role: terminal_role.clone(),
                })?;
            if actual.intent() != expectation.expected_intent() {
                return Err(TerminalExpectationError::CompensationExpectationMismatch {
                    terminal_role: terminal_role.clone(),
                    expected: expectation.expected_intent().clone(),
                    actual: actual.intent().clone(),
                });
            }
        }

        for (terminal_role, requirement) in &self.exception_expectations {
            let expectation = basis
                .exception_expectations
                .get(terminal_role)
                .expect("exact exception expectation role-set equality checked above");
            let expected_source = exact_input_at_role(
                &basis.terminal_basis.boundary_results,
                requirement.expectation_role(),
            )?;
            if expectation.source() != expected_source {
                return Err(TerminalExpectationError::ExceptionExpectationSourceMismatch {
                    terminal_role: terminal_role.clone(),
                    expectation_source: expectation.source().clone(),
                    boundary_source: expected_source.clone(),
                });
            }

            let actual = basis
                .terminal_basis
                .exception_bindings
                .get(terminal_role)
                .ok_or_else(|| TerminalExpectationError::MissingExceptionBinding {
                    terminal_role: terminal_role.clone(),
                })?;
            if actual.exception() != expectation.expected_exception() {
                return Err(TerminalExpectationError::ExceptionExpectationMismatch {
                    terminal_role: terminal_role.clone(),
                    expected: expectation.expected_exception().clone(),
                    actual: actual.exception().clone(),
                });
            }
        }

        self.base
            .qualify(workflow, basis.terminal_basis)
            .map_err(TerminalExpectationError::TerminalQualification)
    }

    fn validate_profile_expectations(&self) -> Result<(), TerminalExpectationError> {
        let all_terminal_roles = self
            .base
            .compensation_roles()
            .keys()
            .cloned()
            .chain(self.base.exception_roles().iter().cloned())
            .collect::<BTreeSet<_>>();

        for (terminal_role, requirement) in &self.compensation_expectations {
            validate_expectation_role(
                &self.base,
                terminal_role,
                requirement.expectation_role(),
                &all_terminal_roles,
            )?;
        }
        for (terminal_role, requirement) in &self.exception_expectations {
            validate_expectation_role(
                &self.base,
                terminal_role,
                requirement.expectation_role(),
                &all_terminal_roles,
            )?;
        }
        Ok(())
    }
}

/// Exact instance-specific expectation basis layered over one terminal basis.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExpectedTerminalClosureBasis {
    pub terminal_basis: TerminalClosureQualificationBasis,
    pub compensation_expectations:
        BTreeMap<DerivationNodeRef, CompensationExpectationBinding>,
    pub exception_expectations: BTreeMap<DerivationNodeRef, ExceptionExpectationBinding>,
}

fn validate_expectation_role(
    profile: &TerminalClosureQualificationProfile,
    terminal_role: &DerivationNodeRef,
    expectation_role: &DerivationNodeRef,
    all_terminal_roles: &BTreeSet<DerivationNodeRef>,
) -> Result<(), TerminalExpectationError> {
    if terminal_role == expectation_role || all_terminal_roles.contains(expectation_role) {
        return Err(TerminalExpectationError::ExpectationRoleIsTerminal {
            terminal_role: terminal_role.clone(),
            expectation_role: expectation_role.clone(),
        });
    }

    let Some(requirement) = profile.base().boundary_requirements().get(expectation_role) else {
        return Err(TerminalExpectationError::ExpectationRoleMissingBoundary {
            terminal_role: terminal_role.clone(),
            expectation_role: expectation_role.clone(),
        });
    };
    if !matches!(requirement, QualifiedBoundaryRequirement::Input { .. }) {
        return Err(TerminalExpectationError::ExpectationRoleBoundaryNotInput {
            terminal_role: terminal_role.clone(),
            expectation_role: expectation_role.clone(),
        });
    }

    let dependencies = profile
        .base()
        .dependency_graph()
        .reachable_nodes_from(expectation_role)
        .ok_or_else(|| TerminalExpectationError::ExpectationRoleMissingFromGraph {
            terminal_role: terminal_role.clone(),
            expectation_role: expectation_role.clone(),
        })?;
    let terminal_dependencies = dependencies
        .intersection(all_terminal_roles)
        .cloned()
        .collect::<BTreeSet<_>>();
    if !terminal_dependencies.is_empty() {
        return Err(TerminalExpectationError::ExpectationDependsOnTerminalRole {
            terminal_role: terminal_role.clone(),
            expectation_role: expectation_role.clone(),
            terminal_dependencies,
        });
    }

    Ok(())
}

fn exact_input_at_role<'a>(
    boundary_results: &'a BTreeMap<DerivationNodeRef, QualifiedBoundaryRef>,
    role: &DerivationNodeRef,
) -> Result<&'a QualifiedInputRef, TerminalExpectationError> {
    let Some(boundary) = boundary_results.get(role) else {
        return Err(TerminalExpectationError::ExpectationBoundaryMissingFromBasis {
            expectation_role: role.clone(),
        });
    };
    let QualifiedBoundaryRef::Input(input) = boundary else {
        return Err(TerminalExpectationError::ExpectationBoundaryNotInput {
            expectation_role: role.clone(),
        });
    };
    Ok(input)
}

/// Structural failures in instance-specific terminal expectation qualification.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TerminalExpectationError {
    CompensationExpectationProfileSetMismatch {
        terminal_roles: BTreeSet<DerivationNodeRef>,
        expectation_roles: BTreeSet<DerivationNodeRef>,
    },
    ExceptionExpectationProfileSetMismatch {
        terminal_roles: BTreeSet<DerivationNodeRef>,
        expectation_roles: BTreeSet<DerivationNodeRef>,
    },
    CompensationExpectationSetMismatch {
        required: BTreeSet<DerivationNodeRef>,
        provided: BTreeSet<DerivationNodeRef>,
    },
    ExceptionExpectationSetMismatch {
        required: BTreeSet<DerivationNodeRef>,
        provided: BTreeSet<DerivationNodeRef>,
    },
    ExpectationRoleIsTerminal {
        terminal_role: DerivationNodeRef,
        expectation_role: DerivationNodeRef,
    },
    ExpectationRoleMissingBoundary {
        terminal_role: DerivationNodeRef,
        expectation_role: DerivationNodeRef,
    },
    ExpectationRoleBoundaryNotInput {
        terminal_role: DerivationNodeRef,
        expectation_role: DerivationNodeRef,
    },
    ExpectationRoleMissingFromGraph {
        terminal_role: DerivationNodeRef,
        expectation_role: DerivationNodeRef,
    },
    ExpectationDependsOnTerminalRole {
        terminal_role: DerivationNodeRef,
        expectation_role: DerivationNodeRef,
        terminal_dependencies: BTreeSet<DerivationNodeRef>,
    },
    ExpectationBoundaryMissingFromBasis {
        expectation_role: DerivationNodeRef,
    },
    ExpectationBoundaryNotInput {
        expectation_role: DerivationNodeRef,
    },
    CompensationExpectationSourceMismatch {
        terminal_role: DerivationNodeRef,
        expectation_source: QualifiedInputRef,
        boundary_source: QualifiedInputRef,
    },
    ExceptionExpectationSourceMismatch {
        terminal_role: DerivationNodeRef,
        expectation_source: QualifiedInputRef,
        boundary_source: QualifiedInputRef,
    },
    MissingCompensationBinding {
        terminal_role: DerivationNodeRef,
    },
    MissingExceptionBinding {
        terminal_role: DerivationNodeRef,
    },
    CompensationExpectationMismatch {
        terminal_role: DerivationNodeRef,
        expected: CommittedIntent,
        actual: CommittedIntent,
    },
    ExceptionExpectationMismatch {
        terminal_role: DerivationNodeRef,
        expected: DomainExceptionRef,
        actual: DomainExceptionRef,
    },
    TerminalQualification(TerminalClosureQualificationError),
}

impl fmt::Display for TerminalExpectationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::CompensationExpectationProfileSetMismatch {
                terminal_roles,
                expectation_roles,
            } => write!(
                f,
                "compensation expectation profile roles {expectation_roles:?} do not exactly match terminal roles {terminal_roles:?}"
            ),
            Self::ExceptionExpectationProfileSetMismatch {
                terminal_roles,
                expectation_roles,
            } => write!(
                f,
                "exception expectation profile roles {expectation_roles:?} do not exactly match terminal roles {terminal_roles:?}"
            ),
            Self::CompensationExpectationSetMismatch { required, provided } => write!(
                f,
                "compensation expectation bindings {provided:?} do not exactly match required roles {required:?}"
            ),
            Self::ExceptionExpectationSetMismatch { required, provided } => write!(
                f,
                "exception expectation bindings {provided:?} do not exactly match required roles {required:?}"
            ),
            Self::ExpectationRoleIsTerminal {
                terminal_role,
                expectation_role,
            } => write!(
                f,
                "terminal role {terminal_role} cannot use terminal role {expectation_role} as its authoritative expectation source"
            ),
            Self::ExpectationRoleMissingBoundary {
                terminal_role,
                expectation_role,
            } => write!(
                f,
                "terminal role {terminal_role} expectation role {expectation_role} is not a declared boundary role"
            ),
            Self::ExpectationRoleBoundaryNotInput {
                terminal_role,
                expectation_role,
            } => write!(
                f,
                "terminal role {terminal_role} expectation role {expectation_role} must be an exact input boundary"
            ),
            Self::ExpectationRoleMissingFromGraph {
                terminal_role,
                expectation_role,
            } => write!(
                f,
                "terminal role {terminal_role} expectation role {expectation_role} is absent from the policy dependency graph"
            ),
            Self::ExpectationDependsOnTerminalRole {
                terminal_role,
                expectation_role,
                terminal_dependencies,
            } => write!(
                f,
                "terminal role {terminal_role} expectation role {expectation_role} depends on terminal result nodes {terminal_dependencies:?}; expectations must be independently grounded"
            ),
            Self::ExpectationBoundaryMissingFromBasis { expectation_role } => write!(
                f,
                "expectation role {expectation_role} has no exact boundary result in the transaction basis"
            ),
            Self::ExpectationBoundaryNotInput { expectation_role } => write!(
                f,
                "expectation role {expectation_role} is not grounded by an exact input result"
            ),
            Self::CompensationExpectationSourceMismatch {
                terminal_role,
                expectation_source,
                boundary_source,
            } => write!(
                f,
                "compensation expectation for role {terminal_role} uses source {}/{}@{} but its declared expectation boundary is {}/{}@{}",
                expectation_source.domain,
                expectation_source.record,
                expectation_source.version,
                boundary_source.domain,
                boundary_source.record,
                boundary_source.version
            ),
            Self::ExceptionExpectationSourceMismatch {
                terminal_role,
                expectation_source,
                boundary_source,
            } => write!(
                f,
                "exception expectation for role {terminal_role} uses source {}/{}@{} but its declared expectation boundary is {}/{}@{}",
                expectation_source.domain,
                expectation_source.record,
                expectation_source.version,
                boundary_source.domain,
                boundary_source.record,
                boundary_source.version
            ),
            Self::MissingCompensationBinding { terminal_role } => write!(
                f,
                "compensation expectation exists for role {terminal_role} but no actual compensation binding was supplied"
            ),
            Self::MissingExceptionBinding { terminal_role } => write!(
                f,
                "exception expectation exists for role {terminal_role} but no actual exception binding was supplied"
            ),
            Self::CompensationExpectationMismatch {
                terminal_role,
                expected,
                actual,
            } => write!(
                f,
                "compensation role {terminal_role} actual committed intent {} does not equal authoritative expected committed intent {}",
                actual.intent_ref, expected.intent_ref
            ),
            Self::ExceptionExpectationMismatch {
                terminal_role,
                expected,
                actual,
            } => write!(
                f,
                "exception role {terminal_role} actual exception {actual} does not equal authoritative expected exception {expected}"
            ),
            Self::TerminalQualification(err) => write!(
                f,
                "terminal source-role qualification failed after expectation validation: {err}"
            ),
        }
    }
}

impl std::error::Error for TerminalExpectationError {}
