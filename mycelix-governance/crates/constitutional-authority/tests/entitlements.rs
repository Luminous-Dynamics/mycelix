// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use constitutional_authority::{
    branch_can_exercise, guardian_can_exercise, AuthorityPrincipal, Branch, CapabilitySource,
    ConstitutionalEntitlement, ConstitutionalEntitlementGrant, ConstitutionalPower,
    EntitlementError, Guardian, InformationSensitivity,
};

fn grant(holder_id: &str, holder: AuthorityPrincipal) -> ConstitutionalEntitlementGrant {
    ConstitutionalEntitlementGrant {
        id: "grant-1".into(),
        holder_id: holder_id.into(),
        holder,
        entitlement: ConstitutionalEntitlement::RequestLawfulRecord,
        jurisdiction: "region-a".into(),
        source: CapabilitySource::Charter {
            charter_id: "charter".into(),
            version: 1,
        },
        purpose: "independent constitutional oversight".into(),
        scope: "records material to matter-42".into(),
        sensitivity: InformationSensitivity::Protected,
        valid_from_us: 1,
        expires_at_us: Some(100),
        review_path: "justice:administrative-review".into(),
    }
}

#[test]
fn one_entitlement_can_be_granted_to_multiple_independent_owner_classes() {
    let integrity = grant(
        "integrity:region-a",
        AuthorityPrincipal::Branch(Branch::Integrity),
    );
    let defender = grant(
        "rights-defender:region-a",
        AuthorityPrincipal::Guardian(Guardian::RightsDefender),
    );

    assert!(integrity.validate().is_ok());
    assert!(defender.validate().is_ok());
    assert_eq!(integrity.entitlement, defender.entitlement);
    assert_ne!(integrity.holder_id, defender.holder_id);
}

#[test]
fn record_access_does_not_confer_adjudication_power() {
    let access = grant(
        "integrity:region-a",
        AuthorityPrincipal::Branch(Branch::Integrity),
    );
    assert!(access.validate().is_ok());

    assert!(!branch_can_exercise(
        Branch::Integrity,
        ConstitutionalPower::AdjudicateDispute
    ));
}

#[test]
fn evidence_access_does_not_confer_prosecution_power() {
    let mut access = grant(
        "evidence:region-a",
        AuthorityPrincipal::Guardian(Guardian::PublicEvidence),
    );
    access.entitlement = ConstitutionalEntitlement::AccessSubmittedEvidence;
    assert!(access.validate().is_ok());

    assert!(!guardian_can_exercise(
        Guardian::PublicEvidence,
        ConstitutionalPower::InitiatePublicProsecution
    ));
}

#[test]
fn constitutional_entitlement_requires_declared_purpose_scope_and_review() {
    let mut access = grant(
        "integrity:region-a",
        AuthorityPrincipal::Branch(Branch::Integrity),
    );
    access.purpose.clear();
    assert_eq!(access.validate(), Err(EntitlementError::EmptyPurpose));

    let mut access = grant(
        "integrity:region-a",
        AuthorityPrincipal::Branch(Branch::Integrity),
    );
    access.scope.clear();
    assert_eq!(access.validate(), Err(EntitlementError::EmptyScope));

    let mut access = grant(
        "integrity:region-a",
        AuthorityPrincipal::Branch(Branch::Integrity),
    );
    access.review_path.clear();
    assert_eq!(access.validate(), Err(EntitlementError::EmptyReviewPath));
}

#[test]
fn entitlement_rejects_empty_provenance_identifier() {
    let mut access = grant(
        "integrity:region-a",
        AuthorityPrincipal::Branch(Branch::Integrity),
    );
    access.source = CapabilitySource::Statute {
        proposal_id: "".into(),
    };
    assert_eq!(access.validate(), Err(EntitlementError::InvalidSource));
}

#[test]
fn entitlement_delegation_fails_closed_until_attenuation_is_defined() {
    let mut access = grant(
        "rights-defender:region-a",
        AuthorityPrincipal::Guardian(Guardian::RightsDefender),
    );
    access.source = CapabilitySource::Delegation {
        parent_capability_id: "parent-grant".into(),
    };
    assert_eq!(
        access.validate(),
        Err(EntitlementError::DelegatedEntitlementUnsupported)
    );
}

#[test]
fn automated_agent_cannot_hold_constitutional_entitlement_directly() {
    let access = grant("agent:1", AuthorityPrincipal::AutomatedAgent);
    assert_eq!(
        access.validate(),
        Err(EntitlementError::AutomatedAgentCannotHoldConstitutionalEntitlement)
    );
}

#[test]
fn concrete_holder_identity_is_required_for_entitlement() {
    let access = grant("", AuthorityPrincipal::Guardian(Guardian::RightsDefender));
    assert_eq!(access.validate(), Err(EntitlementError::EmptyHolderId));
}

#[test]
fn entitlement_expiry_must_follow_activation() {
    let mut access = grant(
        "rights-defender:region-a",
        AuthorityPrincipal::Guardian(Guardian::RightsDefender),
    );
    access.expires_at_us = Some(access.valid_from_us);
    assert_eq!(access.validate(), Err(EntitlementError::InvalidExpiry));
}
