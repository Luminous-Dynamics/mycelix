// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use constitutional_authority::{
    AuthorityPrincipal, Branch, CapabilitySource, ConstitutionalEntitlement, ConstitutionalPower,
    Guardian,
};
use constitutional_envelope::*;

fn digest(value: &str) -> DigestRef {
    DigestRef {
        algorithm: "sha256".into(),
        value: value.into(),
    }
}

fn matter(id: &str) -> MatterId {
    MatterId {
        namespace: "appropriation".into(),
        stable_id: id.into(),
    }
}

fn base_envelope() -> AuthorizationEnvelope {
    AuthorizationEnvelope {
        schema_version: ENVELOPE_SCHEMA_VERSION,
        holder_class: AuthorityPrincipal::Branch(Branch::Deliberative),
        holder_id: "assembly:region-a".into(),
        actor_id: "member:key-1".into(),
        action: AuthorizationAction::Sovereign(ConstitutionalPower::AppropriatePublicFunds),
        jurisdiction: "region-a".into(),
        source: CapabilitySource::Charter {
            charter_id: "charter:region-a".into(),
            version: 7,
        },
        source_digest: Some(digest("source-1")),
        matter: matter("budget-42"),
        purpose: "authorize appropriation 42".into(),
        resource: ResourceBinding {
            kind: "appropriation".into(),
            id: "appropriation:42".into(),
            payload_digest: digest("payload-1"),
        },
        issued_at_us: 10,
        valid_from_us: 20,
        expires_at_us: Some(100),
        nonce: "nonce-1".into(),
        use_policy: UsePolicy::OneShot,
        delegation_chain_digest: None,
        concurrence: None,
        review_path: "justice:administrative-review".into(),
    }
}

#[test]
fn canonical_bytes_are_deterministic() {
    let envelope = base_envelope();
    let first = envelope.canonical_bytes().unwrap();
    let second = envelope.canonical_bytes().unwrap();
    assert_eq!(first, second);
    assert!(first.starts_with(ENVELOPE_DOMAIN_SEPARATOR));
}

#[test]
fn every_security_relevant_mutation_changes_canonical_bytes() {
    let base = base_envelope();
    let original = base.canonical_bytes().unwrap();
    let mut variants = Vec::new();

    let mut v = base.clone();
    v.holder_id = "assembly:region-b".into();
    variants.push(v);

    let mut v = base.clone();
    v.actor_id = "member:key-2".into();
    variants.push(v);

    let mut v = base.clone();
    v.action = AuthorizationAction::Sovereign(ConstitutionalPower::ProposeOrdinaryLaw);
    variants.push(v);

    let mut v = base.clone();
    v.jurisdiction = "region-b".into();
    variants.push(v);

    let mut v = base.clone();
    v.source = CapabilitySource::Charter {
        charter_id: "charter:region-a".into(),
        version: 8,
    };
    variants.push(v);

    let mut v = base.clone();
    v.source_digest = Some(digest("source-2"));
    variants.push(v);

    let mut v = base.clone();
    v.matter.stable_id = "budget-43".into();
    variants.push(v);

    let mut v = base.clone();
    v.purpose = "authorize appropriation 43".into();
    variants.push(v);

    let mut v = base.clone();
    v.resource.id = "appropriation:43".into();
    variants.push(v);

    let mut v = base.clone();
    v.resource.payload_digest = digest("payload-2");
    variants.push(v);

    let mut v = base.clone();
    v.issued_at_us = 11;
    variants.push(v);

    let mut v = base.clone();
    v.valid_from_us = 21;
    variants.push(v);

    let mut v = base.clone();
    v.expires_at_us = Some(101);
    variants.push(v);

    let mut v = base.clone();
    v.nonce = "nonce-2".into();
    variants.push(v);

    let mut v = base.clone();
    v.use_policy = UsePolicy::Bounded { max_uses: 2 };
    variants.push(v);

    let mut v = base.clone();
    v.review_path = "justice:constitutional-review".into();
    variants.push(v);

    for variant in variants {
        let bytes = variant.canonical_bytes().unwrap();
        assert_ne!(original, bytes);
    }
}

#[test]
fn canonical_concurrence_treats_set_order_as_semantically_irrelevant() {
    let mut a = base_envelope();
    a.concurrence = Some(ConcurrenceRequirement {
        min_approvals: 2,
        min_distinct_domains: 2,
        required_domains: vec![
            AuthorityDomain::Branch(Branch::Integrity),
            AuthorityDomain::Branch(Branch::Deliberative),
        ],
        require_unique_holders: true,
        require_unique_actors: true,
        excluded_holder_ids: vec!["holder:z".into(), "holder:a".into()],
        excluded_actor_ids: vec!["actor:z".into(), "actor:a".into()],
    });

    let mut b = a.clone();
    let req = b.concurrence.as_mut().unwrap();
    req.required_domains.reverse();
    req.excluded_holder_ids.reverse();
    req.excluded_actor_ids.reverse();

    assert_eq!(a.canonical_bytes().unwrap(), b.canonical_bytes().unwrap());
}

#[test]
fn malformed_envelopes_fail_closed() {
    let mut envelope = base_envelope();
    envelope.holder_id.clear();
    assert_eq!(envelope.validate(), Err(EnvelopeError::EmptyHolderId));

    let mut envelope = base_envelope();
    envelope.actor_id.clear();
    assert_eq!(envelope.validate(), Err(EnvelopeError::EmptyActorId));

    let mut envelope = base_envelope();
    envelope.matter.namespace = "Election / Unsafe".into();
    assert_eq!(envelope.validate(), Err(EnvelopeError::InvalidMatterNamespace));

    let mut envelope = base_envelope();
    envelope.nonce.clear();
    assert_eq!(envelope.validate(), Err(EnvelopeError::EmptyNonce));

    let mut envelope = base_envelope();
    envelope.use_policy = UsePolicy::Bounded { max_uses: 0 };
    assert_eq!(envelope.validate(), Err(EnvelopeError::InvalidUsePolicy));
}

#[test]
fn automated_agent_may_be_runtime_actor_but_not_constitutional_holder_class() {
    let mut envelope = base_envelope();
    envelope.actor_id = "service:symthaea-analysis-agent".into();
    assert!(envelope.validate().is_ok());

    envelope.holder_class = AuthorityPrincipal::AutomatedAgent;
    assert_eq!(
        envelope.validate(),
        Err(EnvelopeError::AutomatedAgentCannotHoldConstitutionalAuthority)
    );
}

#[test]
fn delegated_source_requires_chain_commitment() {
    let mut envelope = base_envelope();
    envelope.action = AuthorizationAction::Sovereign(ConstitutionalPower::ProposeOrdinaryLaw);
    envelope.source = CapabilitySource::Delegation {
        parent_capability_id: "cap-parent".into(),
    };
    envelope.source_digest = None;
    assert_eq!(
        envelope.validate(),
        Err(EnvelopeError::MissingDelegationCommitment)
    );

    envelope.delegation_chain_digest = Some(digest("chain-root"));
    assert!(envelope.validate().is_ok());
}

#[test]
fn intrinsically_nondelegable_power_cannot_use_delegated_source() {
    let mut envelope = base_envelope();
    envelope.holder_class = AuthorityPrincipal::Branch(Branch::CivicMandate);
    envelope.holder_id = "civic:region-a".into();
    envelope.action = AuthorizationAction::Sovereign(ConstitutionalPower::CertifyMandate);
    envelope.source = CapabilitySource::Delegation {
        parent_capability_id: "cap-parent".into(),
    };
    envelope.delegation_chain_digest = Some(digest("chain-root"));
    assert_eq!(
        envelope.validate(),
        Err(EnvelopeError::IntrinsicPowerCannotUseDelegatedSource)
    );
}

#[test]
fn entitlement_is_distinct_from_sovereign_power() {
    let mut envelope = base_envelope();
    envelope.holder_class = AuthorityPrincipal::Guardian(Guardian::RightsDefender);
    envelope.holder_id = "ombuds:region-a".into();
    envelope.action = AuthorizationAction::Entitlement(ConstitutionalEntitlement::RequestLawfulRecord);
    assert!(envelope.validate().is_ok());
}

#[test]
fn concurrence_requires_real_holder_and_domain_diversity() {
    let requirement = ConcurrenceRequirement {
        min_approvals: 2,
        min_distinct_domains: 2,
        required_domains: vec![
            AuthorityDomain::Branch(Branch::Deliberative),
            AuthorityDomain::Branch(Branch::Integrity),
        ],
        require_unique_holders: true,
        require_unique_actors: true,
        excluded_holder_ids: vec![],
        excluded_actor_ids: vec![],
    };
    let expected = digest("envelope-1");
    let good = vec![
        ConcurrenceApproval {
            holder_id: "assembly:a".into(),
            actor_id: "actor:1".into(),
            domain: AuthorityDomain::Branch(Branch::Deliberative),
            envelope_digest: expected.clone(),
        },
        ConcurrenceApproval {
            holder_id: "integrity:a".into(),
            actor_id: "actor:2".into(),
            domain: AuthorityDomain::Branch(Branch::Integrity),
            envelope_digest: expected.clone(),
        },
    ];
    assert!(validate_concurrence(&requirement, &good, &expected).is_ok());

    let mut same_domain = good.clone();
    same_domain[1].domain = AuthorityDomain::Branch(Branch::Deliberative);
    assert_eq!(
        validate_concurrence(&requirement, &same_domain, &expected),
        Err(ConcurrenceError::InsufficientDistinctDomains)
    );

    let mut duplicate_holder = good.clone();
    duplicate_holder[1].holder_id = duplicate_holder[0].holder_id.clone();
    assert_eq!(
        validate_concurrence(&requirement, &duplicate_holder, &expected),
        Err(ConcurrenceError::DuplicateHolder)
    );

    let mut wrong_digest = good.clone();
    wrong_digest[1].envelope_digest = digest("different-envelope");
    assert_eq!(
        validate_concurrence(&requirement, &wrong_digest, &expected),
        Err(ConcurrenceError::EnvelopeDigestMismatch)
    );
}

#[test]
fn dynamic_separation_is_matter_bound() {
    let rule = SeparationRule {
        left: AuthorizationAction::Sovereign(ConstitutionalPower::ExecuteAppropriation),
        right: AuthorizationAction::Sovereign(ConstitutionalPower::AuditPublicExpenditure),
        subject: ConflictSubject::ExecutingActor,
        scope: SeparationScope::SameMatter,
    };
    let first = AuthorityActivation {
        holder_id: "executive:a".into(),
        actor_id: "person:alice".into(),
        domain: AuthorityDomain::Branch(Branch::Stewardship),
        action: rule.left,
        matter: matter("budget-42"),
        activated_at_us: 10,
    };
    let mut second = AuthorityActivation {
        holder_id: "integrity:a".into(),
        actor_id: "person:alice".into(),
        domain: AuthorityDomain::Branch(Branch::Integrity),
        action: rule.right,
        matter: matter("budget-42"),
        activated_at_us: 20,
    };
    assert!(violates_separation(&rule, &first, &second));

    second.matter = matter("budget-99");
    assert!(!violates_separation(&rule, &first, &second));
}

#[test]
fn holder_and_actor_conflicts_are_distinct() {
    let first = AuthorityActivation {
        holder_id: "office:shared".into(),
        actor_id: "person:alice".into(),
        domain: AuthorityDomain::Branch(Branch::Stewardship),
        action: AuthorizationAction::Sovereign(ConstitutionalPower::ExecuteLaw),
        matter: MatterId { namespace: "case".into(), stable_id: "1".into() },
        activated_at_us: 10,
    };
    let second = AuthorityActivation {
        holder_id: "office:shared".into(),
        actor_id: "person:bob".into(),
        domain: AuthorityDomain::Branch(Branch::Justice),
        action: AuthorizationAction::Sovereign(ConstitutionalPower::AdjudicateDispute),
        matter: first.matter.clone(),
        activated_at_us: 11,
    };

    let actor_rule = SeparationRule {
        left: first.action,
        right: second.action,
        subject: ConflictSubject::ExecutingActor,
        scope: SeparationScope::Static,
    };
    assert!(!violates_separation(&actor_rule, &first, &second));

    let holder_rule = SeparationRule {
        subject: ConflictSubject::ConstitutionalHolder,
        ..actor_rule
    };
    assert!(violates_separation(&holder_rule, &first, &second));
}

#[test]
fn receipt_shape_is_explicitly_validated() {
    let receipt = AuthorizationReceipt {
        envelope_digest: digest("envelope-1"),
        decision: ReceiptDecision::Consumed,
        verifier_id: "runtime:governance-boundary".into(),
        verifier_version: "1".into(),
        evidence_refs: vec![digest("evidence-1")],
        approval_refs: vec![digest("approval-1")],
        timestamp_us: 100,
        action_output: Some(digest("output-1")),
        review_ref: Some("review:42".into()),
    };
    assert!(receipt.validate().is_ok());
}
