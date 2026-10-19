// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use constitutional_authority::{
    validate_delegation, AuthorityPrincipal, Branch, CapabilitySource, ConformanceError,
    ConstitutionalCapability, ConstitutionalPower, DelegationError, JurisdictionRelation,
    ParentCapabilityState,
};

fn parent() -> ConstitutionalCapability {
    ConstitutionalCapability {
        id: "parent-cap".into(),
        holder_id: "integrity:federal".into(),
        holder: AuthorityPrincipal::Branch(Branch::Integrity),
        power: ConstitutionalPower::AuditAuthorityUse,
        jurisdiction: "federation".into(),
        source: CapabilitySource::Charter {
            charter_id: "charter".into(),
            version: 1,
        },
        valid_from_us: 10,
        expires_at_us: Some(100),
        delegable: true,
        delegation_depth_remaining: 2,
    }
}

fn child() -> ConstitutionalCapability {
    ConstitutionalCapability {
        id: "child-cap".into(),
        holder_id: "integrity:region-a".into(),
        holder: AuthorityPrincipal::Branch(Branch::Integrity),
        power: ConstitutionalPower::AuditAuthorityUse,
        jurisdiction: "region-a".into(),
        source: CapabilitySource::Delegation {
            parent_capability_id: "parent-cap".into(),
        },
        valid_from_us: 20,
        expires_at_us: Some(90),
        delegable: true,
        delegation_depth_remaining: 1,
    }
}

#[test]
fn delegated_capability_fails_closed_without_parent_validation() {
    assert_eq!(
        child().validate(),
        Err(ConformanceError::DelegationRequiresParentValidation)
    );
}

#[test]
fn strict_attenuation_is_valid() {
    assert!(validate_delegation(
        &parent(),
        &child(),
        JurisdictionRelation::ChildWithinParent,
        ParentCapabilityState::Active,
    )
    .is_ok());
}

#[test]
fn delegated_child_cannot_change_power() {
    let mut child = child();
    child.power = ConstitutionalPower::AuditPublicExpenditure;

    assert_eq!(
        validate_delegation(
            &parent(),
            &child,
            JurisdictionRelation::ChildWithinParent,
            ParentCapabilityState::Active,
        ),
        Err(DelegationError::PowerChanged)
    );
}

#[test]
fn delegated_child_cannot_broaden_jurisdiction() {
    let mut child = child();
    child.jurisdiction = "global".into();

    assert_eq!(
        validate_delegation(
            &parent(),
            &child,
            JurisdictionRelation::BroaderOrUnrelated,
            ParentCapabilityState::Active,
        ),
        Err(DelegationError::JurisdictionExpandedOrUnresolved)
    );
}

#[test]
fn same_relation_requires_same_jurisdiction_identifier() {
    assert_eq!(
        validate_delegation(
            &parent(),
            &child(),
            JurisdictionRelation::Same,
            ParentCapabilityState::Active,
        ),
        Err(DelegationError::JurisdictionExpandedOrUnresolved)
    );
}

#[test]
fn delegated_child_cannot_start_before_parent() {
    let mut child = child();
    child.valid_from_us = 9;

    assert_eq!(
        validate_delegation(
            &parent(),
            &child,
            JurisdictionRelation::ChildWithinParent,
            ParentCapabilityState::Active,
        ),
        Err(DelegationError::ChildStartsBeforeParent)
    );
}

#[test]
fn delegated_child_cannot_outlive_parent() {
    let mut child = child();
    child.expires_at_us = Some(101);

    assert_eq!(
        validate_delegation(
            &parent(),
            &child,
            JurisdictionRelation::ChildWithinParent,
            ParentCapabilityState::Active,
        ),
        Err(DelegationError::ChildOutlivesParent)
    );

    child.expires_at_us = None;
    assert_eq!(
        validate_delegation(
            &parent(),
            &child,
            JurisdictionRelation::ChildWithinParent,
            ParentCapabilityState::Active,
        ),
        Err(DelegationError::ChildOutlivesParent)
    );
}

#[test]
fn delegated_child_must_reduce_delegation_depth() {
    let mut child = child();
    child.delegation_depth_remaining = 2;

    assert_eq!(
        validate_delegation(
            &parent(),
            &child,
            JurisdictionRelation::ChildWithinParent,
            ParentCapabilityState::Active,
        ),
        Err(DelegationError::DelegationDepthNotReduced)
    );
}

#[test]
fn delegation_must_bind_exact_parent_id() {
    let mut child = child();
    child.source = CapabilitySource::Delegation {
        parent_capability_id: "other-parent".into(),
    };

    assert_eq!(
        validate_delegation(
            &parent(),
            &child,
            JurisdictionRelation::ChildWithinParent,
            ParentCapabilityState::Active,
        ),
        Err(DelegationError::WrongParentReference)
    );
}

#[test]
fn child_cannot_reuse_parent_capability_id() {
    let mut child = child();
    child.id = "parent-cap".into();

    assert_eq!(
        validate_delegation(
            &parent(),
            &child,
            JurisdictionRelation::ChildWithinParent,
            ParentCapabilityState::Active,
        ),
        Err(DelegationError::ChildReusesParentCapabilityId)
    );
}

#[test]
fn expired_or_revoked_parent_invalidates_delegation() {
    for state in [ParentCapabilityState::Expired, ParentCapabilityState::Revoked] {
        assert_eq!(
            validate_delegation(
                &parent(),
                &child(),
                JurisdictionRelation::ChildWithinParent,
                state,
            ),
            Err(DelegationError::ParentNotActive(state))
        );
    }
}

#[test]
fn nondelegable_parent_cannot_produce_child() {
    let mut parent = parent();
    parent.delegable = false;
    parent.delegation_depth_remaining = 0;

    assert_eq!(
        validate_delegation(
            &parent,
            &child(),
            JurisdictionRelation::ChildWithinParent,
            ParentCapabilityState::Active,
        ),
        Err(DelegationError::ParentNotDelegable)
    );
}

#[test]
fn delegation_cannot_cross_into_forbidden_principal_class() {
    let mut child = child();
    child.holder = AuthorityPrincipal::AutomatedAgent;

    assert_eq!(
        validate_delegation(
            &parent(),
            &child,
            JurisdictionRelation::ChildWithinParent,
            ParentCapabilityState::Active,
        ),
        Err(DelegationError::InvalidChild(
            ConformanceError::UnauthorizedPrincipalPower
        ))
    );
}

#[test]
fn intrinsically_nondelegable_power_rejects_delegable_shape() {
    let cap = ConstitutionalCapability {
        id: "mandate-cap".into(),
        holder_id: "civic:commission-a".into(),
        holder: AuthorityPrincipal::Branch(Branch::CivicMandate),
        power: ConstitutionalPower::CertifyMandate,
        jurisdiction: "region-a".into(),
        source: CapabilitySource::Charter {
            charter_id: "charter".into(),
            version: 1,
        },
        valid_from_us: 1,
        expires_at_us: None,
        delegable: true,
        delegation_depth_remaining: 1,
    };

    assert_eq!(
        cap.validate(),
        Err(ConformanceError::IntrinsicPowerCannotBeDelegated)
    );
}
