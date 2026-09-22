use mycelix_authority_evidence_lease::{
    EvidenceLease, PROTOCOL_VERSION as EVIDENCE_LEASE_PROTOCOL_VERSION,
};
use mycelix_capability_core::*;

macro_rules! r {
    ($kind:ident, $value:expr) => {
        $kind::new($value).unwrap()
    };
}

fn scope() -> CapabilityScopeV1 {
    CapabilityScopeV1::new(
        r!(CapabilitySubjectRefV1, "subject:alpha"),
        r!(CapabilityPurposeRefV1, "purpose:bounded-use"),
        r!(CapabilityResourceRefV1, "resource:42"),
        r!(CapabilityActionRefV1, "action:read"),
    )
    .unwrap()
}

fn presenter() -> PresenterBindingV1 {
    PresenterBindingV1::new(
        PresenterBindingKindV1::Key,
        r!(PresenterBindingCommitmentRefV1, "key:abc"),
    )
    .unwrap()
}

fn request() -> CapabilityMintRequestV1 {
    CapabilityMintRequestV1 {
        mint_basis: r!(CapabilityMintBasisRefV1, "basis:1"),
        issuer_authorization: r!(CapabilityIssuerAuthorizationRefV1, "issuer-auth:1"),
        local_authority_domain: r!(LocalAuthorityDomainRefV1, "domain:local"),
        scope: scope(),
        presenter: presenter(),
        replay_domain: r!(ReplayDomainRefV1, "replay:1"),
        revocation_handle: r!(RevocationHandleRefV1, "revocation:1"),
        policy_epoch: r!(PolicyEpochRefV1, "epoch:1"),
        assurance_profile: Some(r!(AssuranceProfileRefV1, "assurance:1")),
        basis_evidence_lease: EvidenceLease::new(100, 1_000, 200).unwrap(),
        issuer_authorization_lease: EvidenceLease::new(150, 900, 200).unwrap(),
        valid_from_ms: 200,
        current_until_ms: 800,
        issuer_max_use_budget: 4,
        requested_use_budget: 2,
    }
}

fn context() -> CapabilityUseContextV1 {
    CapabilityUseContextV1 {
        local_authority_domain: r!(LocalAuthorityDomainRefV1, "domain:local"),
        scope: scope(),
        presenter: presenter(),
        revocation_handle: r!(RevocationHandleRefV1, "revocation:1"),
        policy_epoch: r!(PolicyEpochRefV1, "epoch:1"),
        assurance_profile: Some(r!(AssuranceProfileRefV1, "assurance:1")),
    }
}

fn wire_from_value(value: serde_json::Value) -> BoundedCapabilityLeaseWireV1 {
    serde_json::from_value(value).unwrap()
}

#[test]
fn exact_current_structural_lease_validates() {
    let lease = BoundedCapabilityLeaseV1::mint(request(), 200).unwrap();
    lease.validate_at(300, &context()).unwrap();
    assert_eq!(lease.use_budget(), 2);
    assert_eq!(lease.issuer_max_use_budget(), 4);
    assert!(!lease.transferable());
    assert!(!lease.redelegation_allowed());
    assert_eq!(lease.delegation_depth(), 0);
}

#[test]
fn support_horizon_is_exact_conservative_intersection() {
    let lease = BoundedCapabilityLeaseV1::mint(request(), 200).unwrap();
    assert_eq!(lease.support_lease().verified_at_ms, 150);
    assert_eq!(lease.support_lease().valid_until_ms, 900);
}

#[test]
fn golden_commitment_is_stable() {
    let lease = BoundedCapabilityLeaseV1::mint(request(), 200).unwrap();
    assert_eq!(
        lease.commitment(),
        [
            173, 238, 0, 147, 66, 8, 227, 103, 82, 102, 209, 131, 14, 87, 123, 252, 33, 229, 149,
            201, 188, 92, 10, 39, 209, 100, 36, 218, 198, 226, 14, 85,
        ]
    );
}

#[test]
fn wire_roundtrip_requires_explicit_requalification() {
    let lease = BoundedCapabilityLeaseV1::mint(request(), 200).unwrap();
    let encoded = serde_json::to_vec(&lease).unwrap();
    let wire: BoundedCapabilityLeaseWireV1 = serde_json::from_slice(&encoded).unwrap();
    let requalified = BoundedCapabilityLeaseV1::try_from_wire(wire).unwrap();
    assert_eq!(lease.commitment(), requalified.commitment());
}

#[test]
fn requested_expiry_cannot_exceed_evidence_support() {
    let mut input = request();
    input.current_until_ms = 901;
    assert_eq!(
        BoundedCapabilityLeaseV1::mint(input, 200).unwrap_err(),
        CapabilityLeaseError::LifetimeViolation
    );
}

#[test]
fn future_support_cannot_be_minted() {
    let mut input = request();
    input.basis_evidence_lease = EvidenceLease {
        protocol_version: EVIDENCE_LEASE_PROTOCOL_VERSION.into(),
        verified_at_ms: 300,
        valid_until_ms: 500,
    };
    assert!(matches!(
        BoundedCapabilityLeaseV1::mint(input, 200),
        Err(CapabilityLeaseError::EvidenceLease(_))
    ));
}

#[test]
fn transport_cannot_claim_mint_before_support_verification() {
    let lease = BoundedCapabilityLeaseV1::mint(request(), 200).unwrap();
    let mut value = serde_json::to_value(&lease).unwrap();
    value["minted_at_ms"] = serde_json::Value::from(149_u64);
    let wire = wire_from_value(value);
    assert_eq!(
        BoundedCapabilityLeaseV1::try_from_wire(wire).unwrap_err(),
        CapabilityLeaseError::LifetimeViolation
    );
}

#[test]
fn use_budget_must_attenuate_issuer_maximum() {
    let mut input = request();
    input.requested_use_budget = 5;
    assert_eq!(
        BoundedCapabilityLeaseV1::mint(input, 200).unwrap_err(),
        CapabilityLeaseError::BudgetViolation
    );
}

#[test]
fn v1_rejects_transfer_and_redelegation_from_transport() {
    let lease = BoundedCapabilityLeaseV1::mint(request(), 200).unwrap();

    let mut transfer = serde_json::to_value(&lease).unwrap();
    transfer["transferable"] = serde_json::Value::Bool(true);
    let wire = wire_from_value(transfer);
    assert_eq!(
        BoundedCapabilityLeaseV1::try_from_wire(wire).unwrap_err(),
        CapabilityLeaseError::TransferDenied
    );

    let mut redelegation = serde_json::to_value(&lease).unwrap();
    redelegation["delegation_depth"] = serde_json::Value::from(1_u64);
    let wire = wire_from_value(redelegation);
    assert_eq!(
        BoundedCapabilityLeaseV1::try_from_wire(wire).unwrap_err(),
        CapabilityLeaseError::RedelegationDenied
    );
}

#[test]
fn commitment_tamper_is_detected() {
    let lease = BoundedCapabilityLeaseV1::mint(request(), 200).unwrap();
    let mut value = serde_json::to_value(&lease).unwrap();
    value["current_until_ms"] = serde_json::Value::from(700_u64);
    let wire = wire_from_value(value);
    assert_eq!(
        BoundedCapabilityLeaseV1::try_from_wire(wire).unwrap_err(),
        CapabilityLeaseError::CommitmentMismatch
    );
}

#[test]
fn support_lease_substitution_is_detected_before_commitment() {
    let lease = BoundedCapabilityLeaseV1::mint(request(), 200).unwrap();
    let mut value = serde_json::to_value(&lease).unwrap();
    value["support_lease"]["valid_until_ms"] = serde_json::Value::from(899_u64);
    let wire = wire_from_value(value);
    assert_eq!(
        BoundedCapabilityLeaseV1::try_from_wire(wire).unwrap_err(),
        CapabilityLeaseError::SupportLeaseMismatch
    );
}

#[test]
fn mint_basis_substitution_changes_structural_identity() {
    let left = BoundedCapabilityLeaseV1::mint(request(), 200).unwrap();
    let mut right_input = request();
    right_input.mint_basis = r!(CapabilityMintBasisRefV1, "basis:2");
    let right = BoundedCapabilityLeaseV1::mint(right_input, 200).unwrap();
    assert_ne!(left.commitment(), right.commitment());
}

#[test]
fn exact_context_substitution_fails_closed() {
    let lease = BoundedCapabilityLeaseV1::mint(request(), 200).unwrap();
    let mut wrong = context();
    wrong.policy_epoch = r!(PolicyEpochRefV1, "epoch:2");
    assert_eq!(
        lease.validate_at(300, &wrong).unwrap_err(),
        CapabilityLeaseError::PolicyEpochMismatch
    );
}

#[test]
fn assurance_profile_is_exact_when_present() {
    let lease = BoundedCapabilityLeaseV1::mint(request(), 200).unwrap();
    let mut wrong = context();
    wrong.assurance_profile = Some(r!(AssuranceProfileRefV1, "assurance:2"));
    assert_eq!(
        lease.validate_at(300, &wrong).unwrap_err(),
        CapabilityLeaseError::AssuranceProfileMismatch
    );
}

#[test]
fn noncanonical_or_control_character_references_reject() {
    assert_eq!(
        CapabilitySubjectRefV1::new(" subject:alpha").unwrap_err(),
        CapabilityLeaseError::InvalidReference
    );
    assert_eq!(
        CapabilitySubjectRefV1::new("subject:alpha\n").unwrap_err(),
        CapabilityLeaseError::InvalidReference
    );
}

#[test]
fn unknown_top_level_structural_field_is_rejected() {
    let lease = BoundedCapabilityLeaseV1::mint(request(), 200).unwrap();
    let mut value = serde_json::to_value(&lease).unwrap();
    value.as_object_mut().unwrap().insert(
        "future_authority_field".into(),
        serde_json::Value::Bool(true),
    );
    assert!(serde_json::from_value::<BoundedCapabilityLeaseWireV1>(value).is_err());
}

#[test]
fn unknown_nested_evidence_lease_field_is_rejected() {
    let input = request();
    let mut value = serde_json::to_value(&input).unwrap();
    value["basis_evidence_lease"]
        .as_object_mut()
        .unwrap()
        .insert("future_lease_field".into(), serde_json::Value::Bool(true));
    assert!(serde_json::from_value::<CapabilityMintRequestV1>(value).is_err());
}

#[test]
fn wire_deserialization_does_not_skip_structural_validation() {
    let lease = BoundedCapabilityLeaseV1::mint(request(), 200).unwrap();
    let mut value = serde_json::to_value(&lease).unwrap();
    value["commitment"] = serde_json::to_value(vec![0_u8; 32]).unwrap();
    let wire = wire_from_value(value);
    assert_eq!(
        BoundedCapabilityLeaseV1::try_from_wire(wire).unwrap_err(),
        CapabilityLeaseError::CommitmentMismatch
    );
}
