// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! GP-005 owning-domain expectation adapter diagnostic.
//!
//! `mycelix-business-terminal-expectation` deliberately does not interpret an
//! authoritative Governance resolution. This fixture models that missing owning
//! boundary as a versioned, immutable registry keyed by the exact Governance
//! `QualifiedInputRef`, then feeds the deterministically derived expectation into
//! the generic terminal-expectation qualifier.

use std::collections::{btree_map::Entry, BTreeMap, BTreeSet};

use mycelix_business_core::{
    ClosureClass, ClosurePolicyRef, CommittedIntent, CompensationBinding, DerivationNodeRef,
    DomainExceptionRef, DomainRef, DomainScopedRef, ExceptionBinding, ExceptionRef,
    LogicalIntentRef, OperationCommitment, OrganizationContextRef, QualificationCut,
    QualifiedInputRef, RecordRef, SemanticProfileId, TimestampMs, ValidityEnd, ValidityWindow,
    WorkflowRef,
};
use mycelix_business_qualification::{
    ClosureDependencyGraph, ClosureQualificationProfile, QualifiedBoundaryRef,
    QualifiedBoundaryRequirement,
};
use mycelix_business_terminal_expectation::{
    CompensationExpectationBinding, CompensationExpectationRequirement,
    ExceptionExpectationBinding, ExceptionExpectationRequirement, ExpectedTerminalClosureBasis,
    ExpectedTerminalClosureProfile,
};
use mycelix_business_terminal_qualification::{
    CompensationRoleRequirement, TerminalClosureQualificationBasis,
    TerminalClosureQualificationProfile,
};

fn semantic(name: &str, version: u32) -> SemanticProfileId {
    SemanticProfileId::new(name, version).expect("valid semantic profile")
}

fn domain(name: &str) -> DomainRef {
    DomainRef::new(name).expect("valid domain")
}

fn node(name: &str) -> DerivationNodeRef {
    DerivationNodeRef::new(name).expect("valid node")
}

fn input(
    domain_name: &str,
    record_name: &str,
    semantic_name: &str,
    semantic_version: u32,
) -> QualifiedInputRef {
    QualifiedInputRef {
        domain: domain(domain_name),
        record: RecordRef::new(record_name).expect("valid record"),
        version: 1,
        semantic_profile: semantic(semantic_name, semantic_version),
        generation: None,
        validity: ValidityWindow::new(
            TimestampMs::new(0),
            ValidityEnd::At(TimestampMs::new(100)),
        )
        .expect("valid validity"),
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum ResolutionPayloadV1 {
    Refund {
        original_order: String,
        beneficiary: String,
        currency: String,
        amount_minor: u64,
    },
    RetainException {
        exception_domain: DomainRef,
        exception_id: String,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum GovernanceAdapterError {
    WrongDomain,
    WrongResolutionProfile,
    SourceAlreadyRegistered,
    ResolutionNotRegistered,
    SourceMissingFromCut,
    WrongResolutionKind,
    BindingFailed,
}

#[derive(Clone, Debug, Default)]
struct GovernanceResolutionRegistryV1 {
    resolutions: BTreeMap<QualifiedInputRef, ResolutionPayloadV1>,
}

impl GovernanceResolutionRegistryV1 {
    fn register(
        &mut self,
        source: QualifiedInputRef,
        payload: ResolutionPayloadV1,
    ) -> Result<(), GovernanceAdapterError> {
        if source.domain != domain("governance") {
            return Err(GovernanceAdapterError::WrongDomain);
        }

        let expected_profile = match &payload {
            ResolutionPayloadV1::Refund { .. } => {
                semantic("governance.authorized-refund-resolution", 1)
            }
            ResolutionPayloadV1::RetainException { .. } => {
                semantic("governance.authorized-exception-resolution", 1)
            }
        };
        if source.semantic_profile != expected_profile {
            return Err(GovernanceAdapterError::WrongResolutionProfile);
        }

        match self.resolutions.entry(source) {
            Entry::Vacant(entry) => {
                entry.insert(payload);
                Ok(())
            }
            Entry::Occupied(_) => Err(GovernanceAdapterError::SourceAlreadyRegistered),
        }
    }

    fn compensation_expectation(
        &self,
        source: &QualifiedInputRef,
        cut: &QualificationCut,
    ) -> Result<CompensationExpectationBinding, GovernanceAdapterError> {
        if !cut.inputs().contains(source) {
            return Err(GovernanceAdapterError::SourceMissingFromCut);
        }
        let Some(payload) = self.resolutions.get(source) else {
            return Err(GovernanceAdapterError::ResolutionNotRegistered);
        };
        let ResolutionPayloadV1::Refund {
            original_order,
            beneficiary,
            currency,
            amount_minor,
        } = payload
        else {
            return Err(GovernanceAdapterError::WrongResolutionKind);
        };

        let intent = CommittedIntent::new(
            LogicalIntentRef::new(format!("refund:{original_order}:{amount_minor}"))
                .expect("adapter-generated logical intent is non-empty"),
            semantic("finance.refund", 1),
            OperationCommitment::new(format!(
                "{currency}-minor:{amount_minor}:{beneficiary}:{original_order}"
            ))
            .expect("adapter-generated operation commitment is non-empty"),
        );

        CompensationExpectationBinding::bind(intent, source.clone(), cut)
            .map_err(|_| GovernanceAdapterError::BindingFailed)
    }

    fn exception_expectation(
        &self,
        source: &QualifiedInputRef,
        cut: &QualificationCut,
    ) -> Result<ExceptionExpectationBinding, GovernanceAdapterError> {
        if !cut.inputs().contains(source) {
            return Err(GovernanceAdapterError::SourceMissingFromCut);
        }
        let Some(payload) = self.resolutions.get(source) else {
            return Err(GovernanceAdapterError::ResolutionNotRegistered);
        };
        let ResolutionPayloadV1::RetainException {
            exception_domain,
            exception_id,
        } = payload
        else {
            return Err(GovernanceAdapterError::WrongResolutionKind);
        };

        let expected_exception = DomainScopedRef::new(
            exception_domain.clone(),
            ExceptionRef::new(exception_id.clone())
                .expect("adapter-provided exception identifier is non-empty"),
        );
        ExceptionExpectationBinding::bind(expected_exception, source.clone(), cut)
            .map_err(|_| GovernanceAdapterError::BindingFailed)
    }
}

fn refund_payload(amount_minor: u64) -> ResolutionPayloadV1 {
    ResolutionPayloadV1::Refund {
        original_order: "order-1".to_owned(),
        beneficiary: "customer-a".to_owned(),
        currency: "USD".to_owned(),
        amount_minor,
    }
}

fn expected_refund_intent(amount_minor: u64) -> CommittedIntent {
    CommittedIntent::new(
        LogicalIntentRef::new(format!("refund:order-1:{amount_minor}")).unwrap(),
        semantic("finance.refund", 1),
        OperationCommitment::new(format!(
            "USD-minor:{amount_minor}:customer-a:order-1"
        ))
        .unwrap(),
    )
}

fn retained_exception(local_id: &str) -> DomainExceptionRef {
    DomainScopedRef::new(
        domain("finance"),
        ExceptionRef::new(local_id).expect("valid exception"),
    )
}

struct RefundFixture {
    profile: ExpectedTerminalClosureProfile,
    cut: QualificationCut,
    resolution_role: DerivationNodeRef,
    refund_role: DerivationNodeRef,
    resolution: QualifiedInputRef,
    refund_source: QualifiedInputRef,
}

fn refund_fixture() -> RefundFixture {
    let organization = OrganizationContextRef::new("org:acme").unwrap();
    let qualification_profile = semantic("business.gp005.adapter.compensated.qualification", 1);
    let policy_source = input(
        "governance",
        "closure-policy:gp005:adapter:compensated:v1",
        "governance.closure-policy-active",
        1,
    );
    let resolution = input(
        "governance",
        "resolution:refund-order-1",
        "governance.authorized-refund-resolution",
        1,
    );
    let refund_source = input(
        "finance",
        "refund-effect:order-1",
        "finance.accepted-refund-effect",
        1,
    );

    let root = node("business:gp005-adapter-compensated-close");
    let resolution_role = node("governance:authorized-refund-resolution");
    let refund_role = node("finance:accepted-refund-effect");
    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), resolution_role.clone());
    graph.add_dependency(root.clone(), refund_role.clone());
    graph.add_dependency(refund_role.clone(), resolution_role.clone());

    let base = ClosureQualificationProfile::new(
        organization.clone(),
        qualification_profile.clone(),
        ClosurePolicyRef::new("closure:gp005:adapter:compensated").unwrap(),
        semantic("business.gp005.adapter.compensated.close", 1),
        ClosureClass::Compensated,
        policy_source.clone(),
        root,
        graph,
        BTreeMap::from([
            (
                resolution_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("governance"),
                    semantic("governance.authorized-refund-resolution", 1),
                ),
            ),
            (
                refund_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("finance"),
                    semantic("finance.accepted-refund-effect", 1),
                ),
            ),
        ]),
        BTreeMap::new(),
    );
    let terminal = TerminalClosureQualificationProfile::new(
        base,
        BTreeMap::from([(
            refund_role.clone(),
            CompensationRoleRequirement::new(semantic("finance.refund", 1)),
        )]),
        BTreeSet::new(),
    );
    let profile = ExpectedTerminalClosureProfile::new(
        terminal,
        BTreeMap::from([(
            refund_role.clone(),
            CompensationExpectationRequirement::new(resolution_role.clone()),
        )]),
        BTreeMap::new(),
    );

    let mut cut = QualificationCut::new(
        organization,
        qualification_profile,
        TimestampMs::new(80),
    );
    assert!(cut.insert_input(policy_source).unwrap());
    assert!(cut.insert_input(resolution.clone()).unwrap());
    assert!(cut.insert_input(refund_source.clone()).unwrap());

    RefundFixture {
        profile,
        cut,
        resolution_role,
        refund_role,
        resolution,
        refund_source,
    }
}

fn qualify_refund(
    fixture: &RefundFixture,
    expectation: CompensationExpectationBinding,
    actual: CommittedIntent,
) -> bool {
    let compensation = CompensationBinding::bind(
        actual,
        fixture.refund_source.clone(),
        &fixture.cut,
    )
    .unwrap();
    fixture
        .profile
        .qualify(
            WorkflowRef::new("workflow:gp005:adapter-refund:1").unwrap(),
            ExpectedTerminalClosureBasis {
                terminal_basis: TerminalClosureQualificationBasis {
                    qualification_cut: fixture.cut.clone(),
                    boundary_results: BTreeMap::from([
                        (
                            fixture.resolution_role.clone(),
                            QualifiedBoundaryRef::Input(fixture.resolution.clone()),
                        ),
                        (
                            fixture.refund_role.clone(),
                            QualifiedBoundaryRef::Input(fixture.refund_source.clone()),
                        ),
                    ]),
                    obligations: BTreeMap::new(),
                    disposition_bindings: Vec::new(),
                    compensation_bindings: BTreeMap::from([(
                        fixture.refund_role.clone(),
                        compensation,
                    )]),
                    exception_bindings: BTreeMap::new(),
                },
                compensation_expectations: BTreeMap::from([(
                    fixture.refund_role.clone(),
                    expectation,
                )]),
                exception_expectations: BTreeMap::new(),
            },
        )
        .is_ok()
}

#[test]
fn exact_governance_refund_resolution_derives_deterministic_expectation_consumed_by_business() {
    let fixture = refund_fixture();
    let mut registry = GovernanceResolutionRegistryV1::default();
    registry
        .register(fixture.resolution.clone(), refund_payload(5_000))
        .unwrap();

    let first = registry
        .compensation_expectation(&fixture.resolution, &fixture.cut)
        .unwrap();
    let second = registry
        .compensation_expectation(&fixture.resolution, &fixture.cut)
        .unwrap();
    assert_eq!(first, second);
    assert_eq!(first.expected_intent(), &expected_refund_intent(5_000));
    assert!(qualify_refund(
        &fixture,
        first,
        expected_refund_intent(5_000)
    ));
}

#[test]
fn immutable_exact_resolution_cannot_be_reinterpreted_with_changed_refund_terms() {
    let fixture = refund_fixture();
    let mut registry = GovernanceResolutionRegistryV1::default();
    registry
        .register(fixture.resolution.clone(), refund_payload(5_000))
        .unwrap();

    assert_eq!(
        registry.register(fixture.resolution.clone(), refund_payload(6_000)),
        Err(GovernanceAdapterError::SourceAlreadyRegistered)
    );
}

#[test]
fn changed_actual_refund_is_denied_against_adapter_derived_expectation() {
    let fixture = refund_fixture();
    let mut registry = GovernanceResolutionRegistryV1::default();
    registry
        .register(fixture.resolution.clone(), refund_payload(5_000))
        .unwrap();
    let expectation = registry
        .compensation_expectation(&fixture.resolution, &fixture.cut)
        .unwrap();

    assert!(!qualify_refund(
        &fixture,
        expectation,
        expected_refund_intent(6_000)
    ));
}

#[test]
fn governance_adapter_v1_rejects_wrong_resolution_profile_or_version() {
    let fixture = refund_fixture();
    let mut registry = GovernanceResolutionRegistryV1::default();
    let wrong_version = input(
        "governance",
        "resolution:refund-order-1:v2",
        "governance.authorized-refund-resolution",
        2,
    );

    assert_eq!(
        registry.register(wrong_version, refund_payload(5_000)),
        Err(GovernanceAdapterError::WrongResolutionProfile)
    );
    assert_eq!(
        registry.register(fixture.refund_source.clone(), refund_payload(5_000)),
        Err(GovernanceAdapterError::WrongDomain)
    );
}

#[test]
fn registered_resolution_absent_from_current_cut_cannot_produce_expectation() {
    let fixture = refund_fixture();
    let mut registry = GovernanceResolutionRegistryV1::default();
    registry
        .register(fixture.resolution.clone(), refund_payload(5_000))
        .unwrap();
    let cut_without_resolution = QualificationCut::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        semantic("business.gp005.adapter.compensated.qualification", 1),
        TimestampMs::new(80),
    );

    assert_eq!(
        registry.compensation_expectation(&fixture.resolution, &cut_without_resolution),
        Err(GovernanceAdapterError::SourceMissingFromCut)
    );
}

struct ExceptionFixture {
    profile: ExpectedTerminalClosureProfile,
    cut: QualificationCut,
    resolution_role: DerivationNodeRef,
    exception_role: DerivationNodeRef,
    resolution: QualifiedInputRef,
    exception_source: QualifiedInputRef,
}

fn exception_fixture() -> ExceptionFixture {
    let organization = OrganizationContextRef::new("org:acme").unwrap();
    let qualification_profile = semantic("business.gp005.adapter.exception.qualification", 1);
    let policy_source = input(
        "governance",
        "closure-policy:gp005:adapter:exception:v1",
        "governance.closure-policy-active",
        1,
    );
    let resolution = input(
        "governance",
        "resolution:retain-chargeback-order-1",
        "governance.authorized-exception-resolution",
        1,
    );
    let exception_source = input(
        "finance",
        "chargeback:pending:order-1",
        "finance.chargeback-pending",
        1,
    );

    let root = node("business:gp005-adapter-exception-close");
    let resolution_role = node("governance:authorized-exception-resolution");
    let exception_role = node("finance:required-retained-exception");
    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), resolution_role.clone());
    graph.add_dependency(root.clone(), exception_role.clone());
    graph.add_dependency(exception_role.clone(), resolution_role.clone());

    let base = ClosureQualificationProfile::new(
        organization.clone(),
        qualification_profile.clone(),
        ClosurePolicyRef::new("closure:gp005:adapter:exception").unwrap(),
        semantic("business.gp005.adapter.exception.close", 1),
        ClosureClass::ResolvedWithExceptions,
        policy_source.clone(),
        root,
        graph,
        BTreeMap::from([
            (
                resolution_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("governance"),
                    semantic("governance.authorized-exception-resolution", 1),
                ),
            ),
            (
                exception_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("finance"),
                    semantic("finance.chargeback-pending", 1),
                ),
            ),
        ]),
        BTreeMap::new(),
    );
    let terminal = TerminalClosureQualificationProfile::new(
        base,
        BTreeMap::new(),
        BTreeSet::from([exception_role.clone()]),
    );
    let profile = ExpectedTerminalClosureProfile::new(
        terminal,
        BTreeMap::new(),
        BTreeMap::from([(
            exception_role.clone(),
            ExceptionExpectationRequirement::new(resolution_role.clone()),
        )]),
    );

    let expected_exception = retained_exception("chargeback:pending:1");
    let mut cut = QualificationCut::new(
        organization,
        qualification_profile,
        TimestampMs::new(80),
    );
    assert!(cut.insert_input(policy_source).unwrap());
    assert!(cut.insert_input(resolution.clone()).unwrap());
    assert!(cut.insert_input(exception_source.clone()).unwrap());
    assert!(cut.insert_exception(expected_exception));

    ExceptionFixture {
        profile,
        cut,
        resolution_role,
        exception_role,
        resolution,
        exception_source,
    }
}

fn qualify_exception(
    fixture: &ExceptionFixture,
    expectation: ExceptionExpectationBinding,
    actual: DomainExceptionRef,
) -> bool {
    let mut cut = fixture.cut.clone();
    if !cut.exceptions().contains(&actual) {
        assert!(cut.insert_exception(actual.clone()));
    }
    let binding = ExceptionBinding::bind(actual, fixture.exception_source.clone(), &cut).unwrap();
    fixture
        .profile
        .qualify(
            WorkflowRef::new("workflow:gp005:adapter-exception:1").unwrap(),
            ExpectedTerminalClosureBasis {
                terminal_basis: TerminalClosureQualificationBasis {
                    qualification_cut: cut,
                    boundary_results: BTreeMap::from([
                        (
                            fixture.resolution_role.clone(),
                            QualifiedBoundaryRef::Input(fixture.resolution.clone()),
                        ),
                        (
                            fixture.exception_role.clone(),
                            QualifiedBoundaryRef::Input(fixture.exception_source.clone()),
                        ),
                    ]),
                    obligations: BTreeMap::new(),
                    disposition_bindings: Vec::new(),
                    compensation_bindings: BTreeMap::new(),
                    exception_bindings: BTreeMap::from([(
                        fixture.exception_role.clone(),
                        binding,
                    )]),
                },
                compensation_expectations: BTreeMap::new(),
                exception_expectations: BTreeMap::from([(
                    fixture.exception_role.clone(),
                    expectation,
                )]),
            },
        )
        .is_ok()
}

#[test]
fn exact_governance_exception_resolution_derives_identity_consumed_by_business() {
    let fixture = exception_fixture();
    let mut registry = GovernanceResolutionRegistryV1::default();
    registry
        .register(
            fixture.resolution.clone(),
            ResolutionPayloadV1::RetainException {
                exception_domain: domain("finance"),
                exception_id: "chargeback:pending:1".to_owned(),
            },
        )
        .unwrap();
    let expectation = registry
        .exception_expectation(&fixture.resolution, &fixture.cut)
        .unwrap();

    assert_eq!(
        expectation.expected_exception(),
        &retained_exception("chargeback:pending:1")
    );
    assert!(qualify_exception(
        &fixture,
        expectation,
        retained_exception("chargeback:pending:1")
    ));
}

#[test]
fn substituted_exception_identity_is_denied_against_adapter_derived_expectation() {
    let fixture = exception_fixture();
    let mut registry = GovernanceResolutionRegistryV1::default();
    registry
        .register(
            fixture.resolution.clone(),
            ResolutionPayloadV1::RetainException {
                exception_domain: domain("finance"),
                exception_id: "chargeback:pending:1".to_owned(),
            },
        )
        .unwrap();
    let expectation = registry
        .exception_expectation(&fixture.resolution, &fixture.cut)
        .unwrap();

    assert!(!qualify_exception(
        &fixture,
        expectation,
        retained_exception("chargeback:pending:2")
    ));
}
