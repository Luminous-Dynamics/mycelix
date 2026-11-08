// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_business_core::causal::{CausalIdentity, CommittedIntent};
use mycelix_business_core::{
    AuthorizationBinding, AuthorizationDecisionRef, AuthorizationError,
    DomainAuthorizationDecisionRef, DomainRef, DomainScopedRef, LogicalIntentRef,
    OperationCommitment, OrganizationContextRef, QualificationCut, QualifiedInputRef, RecordRef,
    SemanticProfileId, TimestampMs, ValidityEnd, ValidityWindow,
};

fn profile(name: &str, version: u32) -> SemanticProfileId {
    SemanticProfileId::new(name, version).expect("valid profile")
}

fn domain(name: &str) -> DomainRef {
    DomainRef::new(name).expect("valid domain")
}

fn committed(
    intent_id: &str,
    profile_name: &str,
    version: u32,
    material_commitment: &str,
) -> CommittedIntent {
    CommittedIntent::new(
        LogicalIntentRef::new(intent_id).expect("valid intent"),
        profile(profile_name, version),
        OperationCommitment::new(material_commitment).expect("valid material commitment"),
    )
}

fn authorization_ref() -> DomainAuthorizationDecisionRef {
    DomainScopedRef::new(
        domain("finance"),
        AuthorizationDecisionRef::new("authz:1").expect("valid decision"),
    )
}

fn authorization_source(domain_name: &str, valid_until: i64) -> QualifiedInputRef {
    QualifiedInputRef {
        domain: domain(domain_name),
        record: RecordRef::new("authorization-result:1").expect("valid record"),
        version: 1,
        semantic_profile: profile("finance.authorization-decision", 1),
        generation: None,
        validity: ValidityWindow::new(
            TimestampMs::new(0),
            ValidityEnd::At(TimestampMs::new(valid_until)),
        )
        .expect("valid decision window"),
    }
}

fn cut(at: i64) -> QualificationCut {
    QualificationCut::new(
        OrganizationContextRef::new("org:acme").expect("valid context"),
        profile("business-qualification", 1),
        TimestampMs::new(at),
    )
}

fn binding(intent: CommittedIntent) -> AuthorizationBinding {
    let decision = authorization_ref();
    let source = authorization_source("finance", 100);
    let mut qualification_cut = cut(50);
    assert!(qualification_cut.insert_authorization_decision(decision.clone()));
    assert!(qualification_cut.insert_input(source.clone()).unwrap());

    AuthorizationBinding::bind(decision, intent, source, qualification_cut)
        .expect("valid binding")
}

#[test]
fn authorization_decision_must_be_present_in_exact_cut() {
    let decision = authorization_ref();
    let source = authorization_source("finance", 100);
    let mut qualification_cut = cut(50);
    assert!(qualification_cut.insert_input(source.clone()).unwrap());

    let result = AuthorizationBinding::bind(
        decision.clone(),
        committed(
            "pay:invoice:1",
            "finance.pay",
            1,
            "material:invoice-1:alice:usd:500",
        ),
        source,
        qualification_cut,
    );

    assert!(matches!(
        result,
        Err(AuthorizationError::DecisionMissingFromCut { decision: missing })
            if missing == decision
    ));
}

#[test]
fn authorization_source_must_be_owned_by_decision_domain() {
    let decision = authorization_ref();
    let source = authorization_source("governance", 100);
    let mut qualification_cut = cut(50);
    assert!(qualification_cut.insert_authorization_decision(decision.clone()));
    assert!(qualification_cut.insert_input(source.clone()).unwrap());

    let result = AuthorizationBinding::bind(
        decision.clone(),
        committed(
            "pay:invoice:1",
            "finance.pay",
            1,
            "material:invoice-1:alice:usd:500",
        ),
        source,
        qualification_cut,
    );

    assert!(matches!(
        result,
        Err(AuthorizationError::DecisionSourceDomainMismatch {
            decision: mismatch,
            source_domain,
        }) if mismatch == decision && source_domain == domain("governance")
    ));
}

#[test]
fn authorization_source_must_be_present_in_exact_cut() {
    let decision = authorization_ref();
    let source = authorization_source("finance", 100);
    let mut qualification_cut = cut(50);
    assert!(qualification_cut.insert_authorization_decision(decision.clone()));

    let result = AuthorizationBinding::bind(
        decision.clone(),
        committed(
            "pay:invoice:1",
            "finance.pay",
            1,
            "material:invoice-1:alice:usd:500",
        ),
        source.clone(),
        qualification_cut,
    );

    assert!(matches!(
        result,
        Err(AuthorizationError::DecisionSourceMissingFromCut {
            decision: missing,
            source: missing_source,
        }) if missing == decision && missing_source == source
    ));
}

#[test]
fn authorization_validity_is_derived_from_exact_source() {
    let authorized = committed(
        "pay:invoice:1",
        "finance.pay",
        1,
        "material:invoice-1:alice:usd:500",
    );
    let decision = authorization_ref();
    let source = authorization_source("finance", 73);
    let mut qualification_cut = cut(50);
    assert!(qualification_cut.insert_authorization_decision(decision.clone()));
    assert!(qualification_cut.insert_input(source.clone()).unwrap());

    let binding = AuthorizationBinding::bind(
        decision,
        authorized.clone(),
        source.clone(),
        qualification_cut,
    )
    .expect("valid source-derived binding");

    assert_eq!(binding.decision_source(), &source);
    assert_eq!(
        binding.effective_valid_until(),
        ValidityEnd::At(TimestampMs::new(73))
    );
    assert!(binding.is_applicable_to_committed(&authorized, TimestampMs::new(73)));
    assert!(!binding.is_applicable_to_committed(&authorized, TimestampMs::new(74)));
}

#[test]
fn same_id_and_commitment_remain_same_committed_intent() {
    let first = committed(
        "pay:invoice:1",
        "finance.pay",
        1,
        "material:invoice-1:alice:usd:500",
    );
    let second = committed(
        "pay:invoice:1",
        "finance.pay",
        1,
        "material:invoice-1:alice:usd:500",
    );

    assert_eq!(first, second);
}

#[test]
fn same_logical_id_with_changed_material_commitment_is_not_authorized() {
    let authorized = committed(
        "pay:invoice:1",
        "finance.pay",
        1,
        "material:invoice-1:alice:usd:500",
    );
    let binding = binding(authorized.clone());

    let amount_changed = committed(
        "pay:invoice:1",
        "finance.pay",
        1,
        "material:invoice-1:alice:usd:600",
    );

    assert!(binding.is_applicable_to_committed(&authorized, TimestampMs::new(80)));
    assert!(!binding.is_applicable_to_committed(&amount_changed, TimestampMs::new(80)));
}

#[test]
fn same_logical_id_with_changed_operation_profile_is_not_authorized() {
    let authorized = committed(
        "pay:invoice:1",
        "finance.pay",
        1,
        "material:invoice-1:alice:usd:500",
    );
    let binding = binding(authorized.clone());

    let reinterpreted = committed(
        "pay:invoice:1",
        "finance.pay",
        2,
        "material:invoice-1:alice:usd:500",
    );

    assert!(binding.is_applicable_to_committed(&authorized, TimestampMs::new(80)));
    assert!(!binding.is_applicable_to_committed(&reinterpreted, TimestampMs::new(80)));
}

#[test]
fn different_logical_ids_remain_distinct_even_with_same_material_commitment() {
    let first = committed(
        "pay:attempted-business-effect:1",
        "finance.pay",
        1,
        "material:invoice-1:alice:usd:500",
    );
    let second = committed(
        "pay:attempted-business-effect:2",
        "finance.pay",
        1,
        "material:invoice-1:alice:usd:500",
    );

    assert_ne!(first, second);
}

#[test]
fn causal_identity_retains_semantic_commitment() {
    let intent = committed(
        "pay:invoice:1",
        "finance.pay",
        1,
        "material:invoice-1:alice:usd:500",
    );
    let causal = CausalIdentity::new(intent.clone());

    assert_eq!(causal.logical_intent, intent);
}

#[test]
fn causal_parent_retains_exact_committed_operation() {
    let payment = committed(
        "pay:invoice:1",
        "finance.pay",
        1,
        "material:invoice-1:alice:usd:500",
    );
    let refund = committed(
        "refund:payment:1",
        "finance.refund",
        1,
        "material:payment-1:alice:usd:500",
    );

    let causal = CausalIdentity::new(refund).caused_by(payment.clone());
    assert_eq!(causal.caused_by, Some(payment));
}
