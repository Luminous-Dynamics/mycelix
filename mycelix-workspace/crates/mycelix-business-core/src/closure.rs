// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Obligation dispositions and explicit workflow closure classes.

use core::fmt;
use std::collections::{BTreeMap, BTreeSet};

use crate::causal::CommittedIntent;
use crate::ids::{
    ClosurePolicyRef, DomainExceptionRef, DomainObligationRef, SemanticProfileId, WorkflowRef,
};
#[cfg(test)]
use crate::ids::{DomainRef, ExceptionRef, LogicalIntentRef, ObligationRef, RecordRef};
use crate::qualification::{CutError, QualificationCut, QualifiedInputRef};
#[cfg(test)]
use crate::time::{ValidityEnd, ValidityWindow};

/// Institutional disposition of an obligation.
///
/// Performance and non-performance discharge reasons remain non-substitutable.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
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
    /// Only actual accepted satisfaction counts as performance satisfaction.
    #[must_use]
    pub const fn performance_satisfied(self) -> bool {
        matches!(self, Self::Satisfied)
    }

    /// Whether the disposition is normally no longer an active duty.
    ///
    /// `Breached` and `Disputed` are deliberately not terminal here because
    /// breach/dispute alone need not discharge the duty.
    #[must_use]
    pub const fn is_terminal_disposition(self) -> bool {
        matches!(
            self,
            Self::Satisfied | Self::Waived | Self::Superseded | Self::Terminated
        )
    }
}

/// Exact domain-owned result supporting one obligation disposition used by a
/// Business closure.
///
/// This binding is deliberately structural. The generic Business core does not
/// interpret the source record or decide that it substantively means
/// `Satisfied`, `Waived`, or another disposition. The owning domain/profile must
/// establish that mapping. Business enforces only that closure cannot consume a
/// free-standing disposition detached from an exact source result in the same
/// qualification cut.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct ObligationDispositionBinding {
    obligation: DomainObligationRef,
    disposition: ObligationDisposition,
    source: QualifiedInputRef,
}

impl ObligationDispositionBinding {
    /// Bind a claimed disposition to one exact domain-owned source result that
    /// is present in the same qualification cut.
    pub fn bind(
        obligation: DomainObligationRef,
        disposition: ObligationDisposition,
        source: QualifiedInputRef,
        qualification_cut: &QualificationCut,
    ) -> Result<Self, ClosureError> {
        let binding = Self {
            obligation,
            disposition,
            source,
        };
        binding.validate_against(qualification_cut)?;
        Ok(binding)
    }

    /// Domain-scoped obligation whose disposition is represented.
    #[must_use]
    pub fn obligation(&self) -> &DomainObligationRef {
        &self.obligation
    }

    /// Claimed institutional disposition supplied by the owning adapter/profile.
    #[must_use]
    pub const fn disposition(&self) -> ObligationDisposition {
        self.disposition
    }

    /// Exact domain-owned result from which the owning adapter/profile derived
    /// this disposition.
    #[must_use]
    pub fn source(&self) -> &QualifiedInputRef {
        &self.source
    }

    fn validate_against(&self, qualification_cut: &QualificationCut) -> Result<(), ClosureError> {
        if self.obligation.domain() != &self.source.domain {
            return Err(ClosureError::DispositionSourceDomainMismatch {
                obligation: self.obligation.clone(),
                source_domain: self.source.domain.clone(),
            });
        }

        if !qualification_cut.inputs().contains(&self.source) {
            return Err(ClosureError::DispositionSourceMissingFromCut {
                obligation: self.obligation.clone(),
                source: self.source.clone(),
            });
        }

        Ok(())
    }
}

/// Exact result supporting one compensating institutional effect.
///
/// The full committed intent is retained so amount/destination/purpose/profile
/// semantics cannot collapse back to a reusable logical ID at closure. The
/// supporting result must be part of the same exact qualification cut. The
/// owning adapter/profile remains responsible for proving that the result
/// substantively corresponds to this compensation and that compensation is
/// institutionally adequate.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct CompensationBinding {
    intent: CommittedIntent,
    source: QualifiedInputRef,
}

impl CompensationBinding {
    /// Bind one exact compensating operation to an exact supporting domain result
    /// already present in the qualification cut.
    pub fn bind(
        intent: CommittedIntent,
        source: QualifiedInputRef,
        qualification_cut: &QualificationCut,
    ) -> Result<Self, ClosureError> {
        let binding = Self { intent, source };
        binding.validate_against(qualification_cut)?;
        Ok(binding)
    }

    /// Exact semantically committed compensating operation.
    #[must_use]
    pub fn intent(&self) -> &CommittedIntent {
        &self.intent
    }

    /// Exact domain-owned result supporting the compensation.
    #[must_use]
    pub fn source(&self) -> &QualifiedInputRef {
        &self.source
    }

    fn validate_against(&self, qualification_cut: &QualificationCut) -> Result<(), ClosureError> {
        if !qualification_cut.inputs().contains(&self.source) {
            return Err(ClosureError::CompensationSourceMissingFromCut {
                intent: self.intent.clone(),
                source: self.source.clone(),
            });
        }

        Ok(())
    }
}

/// Exact domain-owned result supporting one retained closure/dispute exception.
///
/// The generic Business core does not decide what the exception substantively
/// means or whether policy permits closure with it. The owning domain/profile
/// supplies that interpretation. Business enforces domain identity, exact source
/// membership, and exact retention in the final receipt.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct ExceptionBinding {
    exception: DomainExceptionRef,
    source: QualifiedInputRef,
}

impl ExceptionBinding {
    /// Bind one domain-scoped exception to an exact domain-owned source result in
    /// the same qualification cut.
    pub fn bind(
        exception: DomainExceptionRef,
        source: QualifiedInputRef,
        qualification_cut: &QualificationCut,
    ) -> Result<Self, ClosureError> {
        let binding = Self { exception, source };
        binding.validate_against(qualification_cut)?;
        Ok(binding)
    }

    /// Exact domain-scoped retained exception identity.
    #[must_use]
    pub fn exception(&self) -> &DomainExceptionRef {
        &self.exception
    }

    /// Exact domain-owned result from which the owning adapter/profile derived
    /// this retained exception.
    #[must_use]
    pub fn source(&self) -> &QualifiedInputRef {
        &self.source
    }

    fn validate_against(&self, qualification_cut: &QualificationCut) -> Result<(), ClosureError> {
        if self.exception.domain() != &self.source.domain {
            return Err(ClosureError::ExceptionSourceDomainMismatch {
                exception: self.exception.clone(),
                source_domain: self.source.domain.clone(),
            });
        }

        if !qualification_cut.inputs().contains(&self.source) {
            return Err(ClosureError::ExceptionSourceMissingFromCut {
                exception: self.exception.clone(),
                source: self.source.clone(),
            });
        }

        Ok(())
    }
}

/// Exact obligation set declared by the named closure policy/profile as
/// required for this closure qualification.
///
/// Obligation identity is domain-scoped. Two authoritative domains may use the
/// same local obligation identifier without one duty disappearing from the
/// closure requirement set.
///
/// This type does not decide which obligations *should* be required. It makes
/// the policy-supplied requirement set explicit so a receipt cannot obtain a
/// stronger class merely by omitting an inconvenient required obligation.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ClosureRequirements {
    required_obligations: BTreeSet<DomainObligationRef>,
}

impl ClosureRequirements {
    /// Production constructor: every required obligation must name its owning
    /// authoritative domain explicitly.
    #[cfg(not(test))]
    #[must_use]
    pub fn new(required_obligations: BTreeSet<DomainObligationRef>) -> Self {
        Self {
            required_obligations,
        }
    }

    /// Compatibility constructor for this crate's pre-scoping unit fixtures.
    /// It is absent from the production library API.
    #[cfg(test)]
    #[must_use]
    pub fn new(required_obligations: BTreeSet<ObligationRef>) -> Self {
        Self {
            required_obligations: required_obligations
                .into_iter()
                .map(Into::into)
                .collect(),
        }
    }

    /// Obligations whose disposition must be represented in the receipt.
    #[must_use]
    pub fn required_obligations(&self) -> &BTreeSet<DomainObligationRef> {
        &self.required_obligations
    }
}

/// Explicit reason a workflow is terminal.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ClosureClass {
    Satisfied,
    Waived,
    Compensated,
    ResolvedWithExceptions,
    Terminated,
}

impl ClosureClass {
    /// Whether the closure class itself claims full performance satisfaction.
    #[must_use]
    pub const fn claims_full_satisfaction(self) -> bool {
        matches!(self, Self::Satisfied)
    }
}

/// Structural closure-receipt validation errors.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ClosureError {
    InvalidQualification(CutError),
    DispositionSourceDomainMismatch {
        obligation: DomainObligationRef,
        source_domain: crate::ids::DomainRef,
    },
    DispositionSourceMissingFromCut {
        obligation: DomainObligationRef,
        source: QualifiedInputRef,
    },
    CompensationSourceMissingFromCut {
        intent: CommittedIntent,
        source: QualifiedInputRef,
    },
    ExceptionSourceDomainMismatch {
        exception: DomainExceptionRef,
        source_domain: crate::ids::DomainRef,
    },
    ExceptionSourceMissingFromCut {
        exception: DomainExceptionRef,
        source: QualifiedInputRef,
    },
    ExceptionSetMismatch {
        qualification_cut: BTreeSet<DomainExceptionRef>,
        receipt: BTreeSet<DomainExceptionRef>,
    },
    DuplicateExceptionBinding {
        exception: DomainExceptionRef,
    },
    DuplicateObligationDisposition {
        obligation: DomainObligationRef,
    },
    MissingRequiredObligation {
        obligation: DomainObligationRef,
    },
    SatisfactionStrengthening {
        obligation: DomainObligationRef,
        disposition: ObligationDisposition,
    },
    NonTerminalRequiredObligation {
        obligation: DomainObligationRef,
        disposition: ObligationDisposition,
        class: ClosureClass,
    },
    SatisfiedWithExceptions,
    WaivedWithoutWaiver,
    CompensatedWithoutCompensation,
    ResolvedWithoutExceptions,
    TerminatedWithoutTermination,
}

impl fmt::Display for ClosureError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidQualification(err) => {
                write!(f, "invalid closure qualification cut: {err}")
            }
            Self::DispositionSourceDomainMismatch {
                obligation,
                source_domain,
            } => write!(
                f,
                "obligation {obligation} cannot use disposition source owned by domain {source_domain}"
            ),
            Self::DispositionSourceMissingFromCut { obligation, source } => write!(
                f,
                "disposition source {}/{}@{} for obligation {obligation} is absent from the exact qualification cut",
                source.domain, source.record, source.version
            ),
            Self::CompensationSourceMissingFromCut { intent, source } => write!(
                f,
                "compensation source {}/{}@{} for committed intent {} is absent from the exact qualification cut",
                source.domain, source.record, source.version, intent.intent_ref
            ),
            Self::ExceptionSourceDomainMismatch {
                exception,
                source_domain,
            } => write!(
                f,
                "exception {exception} cannot use source owned by domain {source_domain}"
            ),
            Self::ExceptionSourceMissingFromCut { exception, source } => write!(
                f,
                "exception source {}/{}@{} for {exception} is absent from the exact qualification cut",
                source.domain, source.record, source.version
            ),
            Self::ExceptionSetMismatch { .. } => f.write_str(
                "closure exception bindings must exactly match the domain-scoped exception set retained by the qualification cut",
            ),
            Self::DuplicateExceptionBinding { exception } => write!(
                f,
                "closure receipt contains more than one exception binding for {exception}"
            ),
            Self::DuplicateObligationDisposition { obligation } => write!(
                f,
                "closure receipt contains more than one disposition binding for obligation {obligation}"
            ),
            Self::MissingRequiredObligation { obligation } => write!(
                f,
                "closure receipt omits policy-required obligation {obligation}"
            ),
            Self::SatisfactionStrengthening {
                obligation,
                disposition,
            } => write!(
                f,
                "closure class Satisfied would strengthen obligation {obligation} from {disposition:?}"
            ),
            Self::NonTerminalRequiredObligation {
                obligation,
                disposition,
                class,
            } => write!(
                f,
                "closure class {class:?} cannot leave required obligation {obligation} in non-terminal disposition {disposition:?}"
            ),
            Self::SatisfiedWithExceptions => {
                f.write_str("closure class Satisfied cannot retain unresolved exceptions")
            }
            Self::WaivedWithoutWaiver => {
                f.write_str("closure class Waived requires at least one waived obligation")
            }
            Self::CompensatedWithoutCompensation => f.write_str(
                "closure class Compensated requires at least one exact compensation binding",
            ),
            Self::ResolvedWithoutExceptions => {
                f.write_str("ResolvedWithExceptions requires an explicit retained exception set")
            }
            Self::TerminatedWithoutTermination => {
                f.write_str("closure class Terminated requires at least one terminated obligation")
            }
        }
    }
}

impl std::error::Error for ClosureError {}

impl From<CutError> for ClosureError {
    fn from(value: CutError) -> Self {
        Self::InvalidQualification(value)
    }
}

/// Auditable workflow-closure receipt.
///
/// The receipt structurally preserves exact required obligations, exact
/// domain-owned disposition/compensation/exception sources, committed
/// compensation semantics, disposition, and qualification context. Domain-specific
/// closure predicates and interpretation of source records remain owned by the
/// named policy and authoritative domains.
///
/// Qualified positive closure objects are sealed in production. Callers can
/// inspect them through accessors but cannot mutate the class or evidence basis
/// after construction:
///
/// ```compile_fail
/// use mycelix_business_core::{ClosureClass, WorkflowClosureReceipt};
///
/// fn cannot_strengthen_after_validation(receipt: &mut WorkflowClosureReceipt) {
///     receipt.class = ClosureClass::Satisfied;
/// }
/// ```
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WorkflowClosureReceipt {
    pub(crate) workflow: WorkflowRef,
    pub(crate) closure_policy: ClosurePolicyRef,
    pub(crate) closure_profile: SemanticProfileId,
    pub(crate) qualification_cut: QualificationCut,
    pub(crate) requirements: ClosureRequirements,
    pub(crate) class: ClosureClass,
    pub(crate) obligations: BTreeMap<DomainObligationRef, ObligationDispositionBinding>,
    pub(crate) exceptions: BTreeMap<DomainExceptionRef, ExceptionBinding>,
    /// Exact compensating effects. The historical field name is retained inside
    /// the crate, but entries preserve committed semantics and exact result
    /// provenance rather than bare logical IDs.
    pub(crate) compensating_intents: BTreeSet<CompensationBinding>,
}

impl WorkflowClosureReceipt {
    /// Workflow qualified by this receipt.
    #[must_use]
    pub fn workflow(&self) -> &WorkflowRef {
        &self.workflow
    }

    /// Closure policy used for qualification.
    #[must_use]
    pub fn closure_policy(&self) -> &ClosurePolicyRef {
        &self.closure_policy
    }

    /// Exact closure semantic profile/version.
    #[must_use]
    pub fn closure_profile(&self) -> &SemanticProfileId {
        &self.closure_profile
    }

    /// Exact immutable qualification basis retained by the receipt.
    #[must_use]
    pub fn qualification_cut(&self) -> &QualificationCut {
        &self.qualification_cut
    }

    /// Policy-declared required obligation set.
    #[must_use]
    pub fn requirements(&self) -> &ClosureRequirements {
        &self.requirements
    }

    /// Explicit reason for terminality.
    #[must_use]
    pub const fn class(&self) -> ClosureClass {
        self.class
    }

    /// Exact obligation-disposition bindings consumed by closure.
    #[must_use]
    pub fn obligations(&self) -> &BTreeMap<DomainObligationRef, ObligationDispositionBinding> {
        &self.obligations
    }

    /// Exact retained exception bindings consumed by closure.
    #[must_use]
    pub fn exceptions(&self) -> &BTreeMap<DomainExceptionRef, ExceptionBinding> {
        &self.exceptions
    }

    /// Exact compensation bindings consumed by a compensated closure.
    #[must_use]
    pub fn compensating_intents(&self) -> &BTreeSet<CompensationBinding> {
        &self.compensating_intents
    }

    /// Production constructor: every consequential obligation disposition,
    /// retained exception, and compensating effect must be bound to exact results
    /// in the same cut.
    #[cfg(not(test))]
    pub fn new(
        workflow: WorkflowRef,
        closure_policy: ClosurePolicyRef,
        closure_profile: SemanticProfileId,
        qualification_cut: QualificationCut,
        requirements: ClosureRequirements,
        class: ClosureClass,
        disposition_bindings: Vec<ObligationDispositionBinding>,
        exception_bindings: Vec<ExceptionBinding>,
        compensating_intents: BTreeSet<CompensationBinding>,
    ) -> Result<Self, ClosureError> {
        Self::new_inner(
            workflow,
            closure_policy,
            closure_profile,
            qualification_cut,
            requirements,
            class,
            disposition_bindings,
            exception_bindings,
            compensating_intents,
        )
    }

    /// Compatibility constructor for this crate's pre-binding unit fixtures.
    /// It is absent from the production library API. Legacy dispositions,
    /// exceptions, and compensating logical IDs are converted into synthetic
    /// scoped/exact inputs before the strict inner constructor is used.
    #[cfg(test)]
    pub fn new(
        workflow: WorkflowRef,
        closure_policy: ClosurePolicyRef,
        closure_profile: SemanticProfileId,
        mut qualification_cut: QualificationCut,
        requirements: ClosureRequirements,
        class: ClosureClass,
        obligations: BTreeMap<ObligationRef, ObligationDisposition>,
        exceptions: BTreeSet<ExceptionRef>,
        compensating_intents: BTreeSet<LogicalIntentRef>,
    ) -> Result<Self, ClosureError> {
        let mut bindings = Vec::with_capacity(obligations.len());

        for (legacy_obligation, disposition) in obligations {
            let obligation: DomainObligationRef = legacy_obligation.into();
            let source = QualifiedInputRef {
                domain: obligation.domain().clone(),
                record: RecordRef::new(format!(
                    "test-disposition:{}",
                    obligation.local().as_str()
                ))
                .expect("test-only disposition record is valid"),
                version: 1,
                semantic_profile: SemanticProfileId::new("test.obligation-disposition", 1)
                    .expect("static test disposition profile is valid"),
                generation: None,
                validity: ValidityWindow::new(
                    qualification_cut.qualification_time,
                    ValidityEnd::Unbounded,
                )
                .expect("test-only disposition validity is valid"),
            };
            let inserted = qualification_cut
                .insert_input(source.clone())
                .expect("test-only disposition input is generation-compatible");
            debug_assert!(inserted, "legacy test disposition inputs must be unique");
            bindings.push(ObligationDispositionBinding::bind(
                obligation,
                disposition,
                source,
                &qualification_cut,
            )?);
        }

        let mut exception_bindings = Vec::with_capacity(exceptions.len());
        for exception in exceptions {
            let scoped: DomainExceptionRef = exception.clone().into();
            let source = QualifiedInputRef {
                domain: scoped.domain().clone(),
                record: RecordRef::new(format!(
                    "test-exception:{}",
                    scoped.local().as_str()
                ))
                .expect("test-only exception record is valid"),
                version: 1,
                semantic_profile: SemanticProfileId::new("test.exception", 1)
                    .expect("static test exception profile is valid"),
                generation: None,
                validity: ValidityWindow::new(
                    qualification_cut.qualification_time,
                    ValidityEnd::Unbounded,
                )
                .expect("test-only exception validity is valid"),
            };
            let _ = qualification_cut
                .insert_input(source.clone())
                .expect("test-only exception input is generation-compatible");
            let _ = qualification_cut.insert_exception(exception);
            exception_bindings.push(ExceptionBinding::bind(
                scoped,
                source,
                &qualification_cut,
            )?);
        }

        let mut compensation_bindings = BTreeSet::new();
        for logical_intent in compensating_intents {
            let committed_intent: CommittedIntent = logical_intent.into();
            let source = QualifiedInputRef {
                domain: DomainRef::new("test-fixture")
                    .expect("static test compensation domain is valid"),
                record: RecordRef::new(format!(
                    "test-compensation:{}",
                    committed_intent.intent_ref.as_str()
                ))
                .expect("test-only compensation record is valid"),
                version: 1,
                semantic_profile: SemanticProfileId::new("test.compensation-result", 1)
                    .expect("static test compensation profile is valid"),
                generation: None,
                validity: ValidityWindow::new(
                    qualification_cut.qualification_time,
                    ValidityEnd::Unbounded,
                )
                .expect("test-only compensation validity is valid"),
            };
            let _ = qualification_cut
                .insert_input(source.clone())
                .expect("test-only compensation input is generation-compatible");
            compensation_bindings.insert(CompensationBinding::bind(
                committed_intent,
                source,
                &qualification_cut,
            )?);
        }

        Self::new_inner(
            workflow,
            closure_policy,
            closure_profile,
            qualification_cut,
            requirements,
            class,
            bindings,
            exception_bindings,
            compensation_bindings,
        )
    }

    fn new_inner(
        workflow: WorkflowRef,
        closure_policy: ClosurePolicyRef,
        closure_profile: SemanticProfileId,
        qualification_cut: QualificationCut,
        requirements: ClosureRequirements,
        class: ClosureClass,
        disposition_bindings: Vec<ObligationDispositionBinding>,
        exception_bindings: Vec<ExceptionBinding>,
        compensating_intents: BTreeSet<CompensationBinding>,
    ) -> Result<Self, ClosureError> {
        qualification_cut.validate_at_qualification_time()?;

        let mut obligations = BTreeMap::new();
        for binding in disposition_bindings {
            binding.validate_against(&qualification_cut)?;
            let obligation = binding.obligation().clone();
            if obligations.insert(obligation.clone(), binding).is_some() {
                return Err(ClosureError::DuplicateObligationDisposition { obligation });
            }
        }

        let mut exceptions = BTreeMap::new();
        for binding in exception_bindings {
            binding.validate_against(&qualification_cut)?;
            let exception = binding.exception().clone();
            if exceptions.insert(exception.clone(), binding).is_some() {
                return Err(ClosureError::DuplicateExceptionBinding { exception });
            }
        }

        let receipt_exception_set = exceptions.keys().cloned().collect::<BTreeSet<_>>();
        if qualification_cut.exceptions() != &receipt_exception_set {
            return Err(ClosureError::ExceptionSetMismatch {
                qualification_cut: qualification_cut.exceptions().clone(),
                receipt: receipt_exception_set,
            });
        }

        for compensation in &compensating_intents {
            compensation.validate_against(&qualification_cut)?;
        }

        for required in requirements.required_obligations() {
            if !obligations.contains_key(required) {
                return Err(ClosureError::MissingRequiredObligation {
                    obligation: required.clone(),
                });
            }
        }

        match class {
            ClosureClass::Satisfied => {
                if !exceptions.is_empty() {
                    return Err(ClosureError::SatisfiedWithExceptions);
                }

                for required in requirements.required_obligations() {
                    let binding = obligations
                        .get(required)
                        .expect("required obligation presence checked above");
                    let disposition = binding.disposition();
                    if !disposition.performance_satisfied() {
                        return Err(ClosureError::SatisfactionStrengthening {
                            obligation: required.clone(),
                            disposition,
                        });
                    }
                }
            }
            ClosureClass::Waived => {
                for required in requirements.required_obligations() {
                    let binding = obligations
                        .get(required)
                        .expect("required obligation presence checked above");
                    let disposition = binding.disposition();
                    if !disposition.is_terminal_disposition() {
                        return Err(ClosureError::NonTerminalRequiredObligation {
                            obligation: required.clone(),
                            disposition,
                            class,
                        });
                    }
                }

                if !requirements.required_obligations().iter().any(|required| {
                    obligations
                        .get(required)
                        .is_some_and(|binding| binding.disposition() == ObligationDisposition::Waived)
                }) {
                    return Err(ClosureError::WaivedWithoutWaiver);
                }
            }
            ClosureClass::Compensated => {
                if compensating_intents.is_empty() {
                    return Err(ClosureError::CompensatedWithoutCompensation);
                }
            }
            ClosureClass::ResolvedWithExceptions => {
                if exceptions.is_empty() {
                    return Err(ClosureError::ResolvedWithoutExceptions);
                }
            }
            ClosureClass::Terminated => {
                for required in requirements.required_obligations() {
                    let binding = obligations
                        .get(required)
                        .expect("required obligation presence checked above");
                    let disposition = binding.disposition();
                    if !disposition.is_terminal_disposition() {
                        return Err(ClosureError::NonTerminalRequiredObligation {
                            obligation: required.clone(),
                            disposition,
                            class,
                        });
                    }
                }

                if !requirements.required_obligations().iter().any(|required| {
                    obligations.get(required).is_some_and(|binding| {
                        binding.disposition() == ObligationDisposition::Terminated
                    })
                }) {
                    return Err(ClosureError::TerminatedWithoutTermination);
                }
            }
        }

        Ok(Self {
            workflow,
            closure_policy,
            closure_profile,
            qualification_cut,
            requirements,
            class,
            obligations,
            exceptions,
            compensating_intents,
        })
    }
}
