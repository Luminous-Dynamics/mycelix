// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Domain-neutral bounded capability lease structure.
//!
//! CORE-CAP-001 separates structural lease mechanics from the domain theorem
//! that establishes mint-basis legitimacy and issuer authority.

use mycelix_authority_evidence_lease::{
    EvidenceLease, EvidenceLeaseError, PROTOCOL_VERSION as EVIDENCE_LEASE_PROTOCOL_VERSION,
};
use serde::{Deserialize, Deserializer, Serialize};
use sha2::{Digest, Sha256};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-core-capability-lease-v1";
pub const COMMITMENT_DOMAIN: &[u8] = b"MYCELIX/CORE-CAP/LEASE/v1\0";
pub const MAX_REF_UTF8_BYTES_V1: usize = 256;
pub const MAX_USE_BUDGET_V1: u32 = 64;

macro_rules! exact_ref_type {
    ($name:ident) => {
        #[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
        #[serde(transparent)]
        pub struct $name(String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, CapabilityLeaseError> {
                let value = value.into();
                validate_ref(&value)?;
                Ok(Self(value))
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }

            fn validate(&self) -> Result<(), CapabilityLeaseError> {
                validate_ref(&self.0)
            }
        }
    };
}

exact_ref_type!(CapabilityMintBasisRefV1);
exact_ref_type!(CapabilityIssuerAuthorizationRefV1);
exact_ref_type!(LocalAuthorityDomainRefV1);
exact_ref_type!(CapabilitySubjectRefV1);
exact_ref_type!(CapabilityPurposeRefV1);
exact_ref_type!(CapabilityResourceRefV1);
exact_ref_type!(CapabilityActionRefV1);
exact_ref_type!(PresenterBindingCommitmentRefV1);
exact_ref_type!(ReplayDomainRefV1);
exact_ref_type!(RevocationHandleRefV1);
exact_ref_type!(PolicyEpochRefV1);
exact_ref_type!(AssuranceProfileRefV1);

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CapabilityScopeV1 {
    subject: CapabilitySubjectRefV1,
    purpose: CapabilityPurposeRefV1,
    resource: CapabilityResourceRefV1,
    action: CapabilityActionRefV1,
}

impl CapabilityScopeV1 {
    pub fn new(
        subject: CapabilitySubjectRefV1,
        purpose: CapabilityPurposeRefV1,
        resource: CapabilityResourceRefV1,
        action: CapabilityActionRefV1,
    ) -> Result<Self, CapabilityLeaseError> {
        let value = Self {
            subject,
            purpose,
            resource,
            action,
        };
        value.validate()?;
        Ok(value)
    }

    pub fn subject(&self) -> &CapabilitySubjectRefV1 {
        &self.subject
    }

    pub fn purpose(&self) -> &CapabilityPurposeRefV1 {
        &self.purpose
    }

    pub fn resource(&self) -> &CapabilityResourceRefV1 {
        &self.resource
    }

    pub fn action(&self) -> &CapabilityActionRefV1 {
        &self.action
    }

    fn validate(&self) -> Result<(), CapabilityLeaseError> {
        self.subject.validate()?;
        self.purpose.validate()?;
        self.resource.validate()?;
        self.action.validate()?;
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PresenterBindingKindV1 {
    Key,
    Session,
    Principal,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PresenterBindingV1 {
    kind: PresenterBindingKindV1,
    commitment: PresenterBindingCommitmentRefV1,
}

impl PresenterBindingV1 {
    pub fn new(
        kind: PresenterBindingKindV1,
        commitment: PresenterBindingCommitmentRefV1,
    ) -> Result<Self, CapabilityLeaseError> {
        commitment.validate()?;
        Ok(Self { kind, commitment })
    }

    pub fn kind(&self) -> PresenterBindingKindV1 {
        self.kind
    }

    pub fn commitment(&self) -> &PresenterBindingCommitmentRefV1 {
        &self.commitment
    }

    fn validate(&self) -> Result<(), CapabilityLeaseError> {
        self.commitment.validate()
    }
}

/// Named structural mint inputs. This is configuration/evidence-shaped data,
/// not a positive authority type.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CapabilityMintRequestV1 {
    pub mint_basis: CapabilityMintBasisRefV1,
    pub issuer_authorization: CapabilityIssuerAuthorizationRefV1,
    pub local_authority_domain: LocalAuthorityDomainRefV1,
    pub scope: CapabilityScopeV1,
    pub presenter: PresenterBindingV1,
    pub replay_domain: ReplayDomainRefV1,
    pub revocation_handle: RevocationHandleRefV1,
    pub policy_epoch: PolicyEpochRefV1,
    pub assurance_profile: Option<AssuranceProfileRefV1>,
    #[serde(deserialize_with = "deserialize_evidence_lease_strict")]
    pub basis_evidence_lease: EvidenceLease,
    #[serde(deserialize_with = "deserialize_evidence_lease_strict")]
    pub issuer_authorization_lease: EvidenceLease,
    pub valid_from_ms: u64,
    pub current_until_ms: u64,
    pub issuer_max_use_budget: u32,
    pub requested_use_budget: u32,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CapabilityUseContextV1 {
    pub local_authority_domain: LocalAuthorityDomainRefV1,
    pub scope: CapabilityScopeV1,
    pub presenter: PresenterBindingV1,
    pub revocation_handle: RevocationHandleRefV1,
    pub policy_epoch: PolicyEpochRefV1,
    pub assurance_profile: Option<AssuranceProfileRefV1>,
}

/// Untrusted transport representation. Deserialization creates only this wire
/// type; callers must explicitly requalify it before obtaining a validated
/// `BoundedCapabilityLeaseV1`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BoundedCapabilityLeaseWireV1 {
    protocol_version: String,
    mint_basis: CapabilityMintBasisRefV1,
    issuer_authorization: CapabilityIssuerAuthorizationRefV1,
    local_authority_domain: LocalAuthorityDomainRefV1,
    scope: CapabilityScopeV1,
    presenter: PresenterBindingV1,
    replay_domain: ReplayDomainRefV1,
    revocation_handle: RevocationHandleRefV1,
    policy_epoch: PolicyEpochRefV1,
    assurance_profile: Option<AssuranceProfileRefV1>,
    #[serde(deserialize_with = "deserialize_evidence_lease_strict")]
    basis_evidence_lease: EvidenceLease,
    #[serde(deserialize_with = "deserialize_evidence_lease_strict")]
    issuer_authorization_lease: EvidenceLease,
    #[serde(deserialize_with = "deserialize_evidence_lease_strict")]
    support_lease: EvidenceLease,
    minted_at_ms: u64,
    valid_from_ms: u64,
    current_until_ms: u64,
    issuer_max_use_budget: u32,
    use_budget: u32,
    transferable: bool,
    redelegation_allowed: bool,
    delegation_depth: u32,
    commitment: [u8; 32],
}

/// Constructor- or validator-produced structural lease.
///
/// This type intentionally implements `Serialize` but **not** `Deserialize`.
/// A serialized lease must first become `BoundedCapabilityLeaseWireV1`, then
/// pass `try_from_wire`. Even this validated structural type is not domain
/// authority; consuming domains must separately retain/qualify mint provenance.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(transparent)]
pub struct BoundedCapabilityLeaseV1(BoundedCapabilityLeaseWireV1);

impl BoundedCapabilityLeaseV1 {
    pub fn mint(
        request: CapabilityMintRequestV1,
        now_ms: u64,
    ) -> Result<Self, CapabilityLeaseError> {
        validate_mint_request_shape(&request)?;

        let support_lease = request
            .basis_evidence_lease
            .intersect(&request.issuer_authorization_lease, now_ms)
            .map_err(CapabilityLeaseError::EvidenceLease)?;

        if request.valid_from_ms < now_ms
            || request.valid_from_ms < support_lease.verified_at_ms
            || request.current_until_ms <= request.valid_from_ms
            || request.current_until_ms > support_lease.valid_until_ms
        {
            return Err(CapabilityLeaseError::LifetimeViolation);
        }

        validate_budget(request.issuer_max_use_budget, request.requested_use_budget)?;

        let mut wire = BoundedCapabilityLeaseWireV1 {
            protocol_version: PROTOCOL_VERSION.into(),
            mint_basis: request.mint_basis,
            issuer_authorization: request.issuer_authorization,
            local_authority_domain: request.local_authority_domain,
            scope: request.scope,
            presenter: request.presenter,
            replay_domain: request.replay_domain,
            revocation_handle: request.revocation_handle,
            policy_epoch: request.policy_epoch,
            assurance_profile: request.assurance_profile,
            basis_evidence_lease: request.basis_evidence_lease,
            issuer_authorization_lease: request.issuer_authorization_lease,
            support_lease,
            minted_at_ms: now_ms,
            valid_from_ms: request.valid_from_ms,
            current_until_ms: request.current_until_ms,
            issuer_max_use_budget: request.issuer_max_use_budget,
            use_budget: request.requested_use_budget,
            transferable: false,
            redelegation_allowed: false,
            delegation_depth: 0,
            commitment: [0; 32],
        };
        wire.commitment = recompute_commitment(&wire);
        Self::try_from_wire(wire)
    }

    pub fn try_from_wire(wire: BoundedCapabilityLeaseWireV1) -> Result<Self, CapabilityLeaseError> {
        validate_wire_structure(&wire)?;
        Ok(Self(wire))
    }

    pub fn to_wire(&self) -> BoundedCapabilityLeaseWireV1 {
        self.0.clone()
    }

    pub fn validate_structure(&self) -> Result<(), CapabilityLeaseError> {
        validate_wire_structure(&self.0)
    }

    pub fn validate_at(
        &self,
        now_ms: u64,
        context: &CapabilityUseContextV1,
    ) -> Result<(), CapabilityLeaseError> {
        self.validate_structure()?;
        self.0
            .support_lease
            .validate_at(now_ms)
            .map_err(CapabilityLeaseError::EvidenceLease)?;

        if now_ms < self.0.valid_from_ms {
            return Err(CapabilityLeaseError::NotYetValid);
        }
        if now_ms >= self.0.current_until_ms {
            return Err(CapabilityLeaseError::Expired);
        }

        context.local_authority_domain.validate()?;
        context.scope.validate()?;
        context.presenter.validate()?;
        context.revocation_handle.validate()?;
        context.policy_epoch.validate()?;
        if let Some(profile) = &context.assurance_profile {
            profile.validate()?;
        }

        if self.0.local_authority_domain != context.local_authority_domain {
            return Err(CapabilityLeaseError::DomainMismatch);
        }
        if self.0.scope != context.scope {
            return Err(CapabilityLeaseError::ScopeMismatch);
        }
        if self.0.presenter != context.presenter {
            return Err(CapabilityLeaseError::PresenterBindingMismatch);
        }
        if self.0.revocation_handle != context.revocation_handle {
            return Err(CapabilityLeaseError::RevocationMismatch);
        }
        if self.0.policy_epoch != context.policy_epoch {
            return Err(CapabilityLeaseError::PolicyEpochMismatch);
        }
        match &self.0.assurance_profile {
            Some(expected) if context.assurance_profile.as_ref() != Some(expected) => {
                Err(CapabilityLeaseError::AssuranceProfileMismatch)
            }
            _ => Ok(()),
        }
    }

    pub fn commitment(&self) -> [u8; 32] {
        self.0.commitment
    }

    pub fn mint_basis(&self) -> &CapabilityMintBasisRefV1 {
        &self.0.mint_basis
    }

    pub fn issuer_authorization(&self) -> &CapabilityIssuerAuthorizationRefV1 {
        &self.0.issuer_authorization
    }

    pub fn local_authority_domain(&self) -> &LocalAuthorityDomainRefV1 {
        &self.0.local_authority_domain
    }

    pub fn scope(&self) -> &CapabilityScopeV1 {
        &self.0.scope
    }

    pub fn presenter(&self) -> &PresenterBindingV1 {
        &self.0.presenter
    }

    pub fn replay_domain(&self) -> &ReplayDomainRefV1 {
        &self.0.replay_domain
    }

    pub fn revocation_handle(&self) -> &RevocationHandleRefV1 {
        &self.0.revocation_handle
    }

    pub fn policy_epoch(&self) -> &PolicyEpochRefV1 {
        &self.0.policy_epoch
    }

    pub fn assurance_profile(&self) -> Option<&AssuranceProfileRefV1> {
        self.0.assurance_profile.as_ref()
    }

    pub fn basis_evidence_lease(&self) -> &EvidenceLease {
        &self.0.basis_evidence_lease
    }

    pub fn issuer_authorization_lease(&self) -> &EvidenceLease {
        &self.0.issuer_authorization_lease
    }

    pub fn support_lease(&self) -> &EvidenceLease {
        &self.0.support_lease
    }

    pub fn minted_at_ms(&self) -> u64 {
        self.0.minted_at_ms
    }

    pub fn valid_from_ms(&self) -> u64 {
        self.0.valid_from_ms
    }

    pub fn current_until_ms(&self) -> u64 {
        self.0.current_until_ms
    }

    pub fn issuer_max_use_budget(&self) -> u32 {
        self.0.issuer_max_use_budget
    }

    pub fn use_budget(&self) -> u32 {
        self.0.use_budget
    }

    pub fn transferable(&self) -> bool {
        self.0.transferable
    }

    pub fn redelegation_allowed(&self) -> bool {
        self.0.redelegation_allowed
    }

    pub fn delegation_depth(&self) -> u32 {
        self.0.delegation_depth
    }
}

impl TryFrom<BoundedCapabilityLeaseWireV1> for BoundedCapabilityLeaseV1 {
    type Error = CapabilityLeaseError;

    fn try_from(value: BoundedCapabilityLeaseWireV1) -> Result<Self, Self::Error> {
        Self::try_from_wire(value)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CapabilityLeaseError {
    InvalidReference,
    WrongProtocolVersion,
    EvidenceLease(EvidenceLeaseError),
    SupportLeaseMismatch,
    LifetimeViolation,
    BudgetViolation,
    TransferDenied,
    RedelegationDenied,
    CommitmentMismatch,
    NotYetValid,
    Expired,
    DomainMismatch,
    ScopeMismatch,
    PresenterBindingMismatch,
    RevocationMismatch,
    PolicyEpochMismatch,
    AssuranceProfileMismatch,
}

impl fmt::Display for CapabilityLeaseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidReference => {
                "capability reference must be canonical, non-empty, and bounded"
            }
            Self::WrongProtocolVersion => "wrong capability lease protocol version",
            Self::EvidenceLease(_) => "evidence lease validation failed",
            Self::SupportLeaseMismatch => {
                "support lease is not the exact conservative intersection"
            }
            Self::LifetimeViolation => "capability lifetime violates the bounded support horizon",
            Self::BudgetViolation => {
                "capability use budget violates the v1 bound or issuer maximum"
            }
            Self::TransferDenied => "capability transfer is denied by v1",
            Self::RedelegationDenied => "capability redelegation is denied by v1",
            Self::CommitmentMismatch => "capability structural commitment mismatch",
            Self::NotYetValid => "capability is not yet structurally valid for use",
            Self::Expired => "capability structural lifetime has expired",
            Self::DomainMismatch => "local authority domain mismatch",
            Self::ScopeMismatch => "capability scope mismatch",
            Self::PresenterBindingMismatch => "presenter binding mismatch",
            Self::RevocationMismatch => "revocation handle mismatch",
            Self::PolicyEpochMismatch => "policy epoch mismatch",
            Self::AssuranceProfileMismatch => "assurance profile mismatch",
        };
        if let Self::EvidenceLease(source) = self {
            write!(f, "{message}: {source}")
        } else {
            write!(f, "{message}")
        }
    }
}

impl std::error::Error for CapabilityLeaseError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::EvidenceLease(source) => Some(source),
            _ => None,
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct StrictEvidenceLeaseV1 {
    protocol_version: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

fn deserialize_evidence_lease_strict<'de, D>(deserializer: D) -> Result<EvidenceLease, D::Error>
where
    D: Deserializer<'de>,
{
    let value = StrictEvidenceLeaseV1::deserialize(deserializer)?;
    Ok(EvidenceLease {
        protocol_version: value.protocol_version,
        verified_at_ms: value.verified_at_ms,
        valid_until_ms: value.valid_until_ms,
    })
}

fn validate_mint_request_shape(
    request: &CapabilityMintRequestV1,
) -> Result<(), CapabilityLeaseError> {
    request.mint_basis.validate()?;
    request.issuer_authorization.validate()?;
    request.local_authority_domain.validate()?;
    request.scope.validate()?;
    request.presenter.validate()?;
    request.replay_domain.validate()?;
    request.revocation_handle.validate()?;
    request.policy_epoch.validate()?;
    if let Some(profile) = &request.assurance_profile {
        profile.validate()?;
    }
    Ok(())
}

fn validate_wire_structure(
    wire: &BoundedCapabilityLeaseWireV1,
) -> Result<(), CapabilityLeaseError> {
    if wire.protocol_version != PROTOCOL_VERSION {
        return Err(CapabilityLeaseError::WrongProtocolVersion);
    }
    wire.mint_basis.validate()?;
    wire.issuer_authorization.validate()?;
    wire.local_authority_domain.validate()?;
    wire.scope.validate()?;
    wire.presenter.validate()?;
    wire.replay_domain.validate()?;
    wire.revocation_handle.validate()?;
    wire.policy_epoch.validate()?;
    if let Some(profile) = &wire.assurance_profile {
        profile.validate()?;
    }

    validate_lease_shape(&wire.basis_evidence_lease)?;
    validate_lease_shape(&wire.issuer_authorization_lease)?;
    validate_lease_shape(&wire.support_lease)?;

    let expected_support = EvidenceLease {
        protocol_version: EVIDENCE_LEASE_PROTOCOL_VERSION.into(),
        verified_at_ms: wire
            .basis_evidence_lease
            .verified_at_ms
            .max(wire.issuer_authorization_lease.verified_at_ms),
        valid_until_ms: wire
            .basis_evidence_lease
            .valid_until_ms
            .min(wire.issuer_authorization_lease.valid_until_ms),
    };
    if wire.support_lease != expected_support {
        return Err(CapabilityLeaseError::SupportLeaseMismatch);
    }

    if wire.minted_at_ms == 0
        || wire.minted_at_ms < wire.support_lease.verified_at_ms
        || wire.valid_from_ms < wire.minted_at_ms
        || wire.current_until_ms <= wire.valid_from_ms
        || wire.current_until_ms > wire.support_lease.valid_until_ms
    {
        return Err(CapabilityLeaseError::LifetimeViolation);
    }

    validate_budget(wire.issuer_max_use_budget, wire.use_budget)?;

    if wire.transferable {
        return Err(CapabilityLeaseError::TransferDenied);
    }
    if wire.redelegation_allowed || wire.delegation_depth != 0 {
        return Err(CapabilityLeaseError::RedelegationDenied);
    }
    if wire.commitment != recompute_commitment(wire) {
        return Err(CapabilityLeaseError::CommitmentMismatch);
    }
    Ok(())
}

fn validate_ref(value: &str) -> Result<(), CapabilityLeaseError> {
    if value.is_empty()
        || value.len() > MAX_REF_UTF8_BYTES_V1
        || value.trim() != value
        || value.chars().any(char::is_control)
    {
        return Err(CapabilityLeaseError::InvalidReference);
    }
    Ok(())
}

fn validate_lease_shape(lease: &EvidenceLease) -> Result<(), CapabilityLeaseError> {
    if lease.protocol_version != EVIDENCE_LEASE_PROTOCOL_VERSION {
        return Err(CapabilityLeaseError::EvidenceLease(
            EvidenceLeaseError::WrongProtocolVersion,
        ));
    }
    if lease.verified_at_ms == 0 {
        return Err(CapabilityLeaseError::EvidenceLease(
            EvidenceLeaseError::InvalidVerificationTime,
        ));
    }
    if lease.valid_until_ms <= lease.verified_at_ms {
        return Err(CapabilityLeaseError::EvidenceLease(
            EvidenceLeaseError::InvertedLease,
        ));
    }
    Ok(())
}

fn validate_budget(issuer_max: u32, requested: u32) -> Result<(), CapabilityLeaseError> {
    if issuer_max == 0
        || requested == 0
        || issuer_max > MAX_USE_BUDGET_V1
        || requested > MAX_USE_BUDGET_V1
        || requested > issuer_max
    {
        return Err(CapabilityLeaseError::BudgetViolation);
    }
    Ok(())
}

fn recompute_commitment(wire: &BoundedCapabilityLeaseWireV1) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hasher.update(COMMITMENT_DOMAIN);
    put_str(&mut hasher, &wire.protocol_version);
    put_str(&mut hasher, wire.mint_basis.as_str());
    put_str(&mut hasher, wire.issuer_authorization.as_str());
    put_str(&mut hasher, wire.local_authority_domain.as_str());
    put_scope(&mut hasher, &wire.scope);
    put_presenter(&mut hasher, &wire.presenter);
    put_str(&mut hasher, wire.replay_domain.as_str());
    put_str(&mut hasher, wire.revocation_handle.as_str());
    put_str(&mut hasher, wire.policy_epoch.as_str());
    put_optional_ref(&mut hasher, wire.assurance_profile.as_ref());
    put_lease(&mut hasher, &wire.basis_evidence_lease);
    put_lease(&mut hasher, &wire.issuer_authorization_lease);
    put_lease(&mut hasher, &wire.support_lease);
    put_u64(&mut hasher, wire.minted_at_ms);
    put_u64(&mut hasher, wire.valid_from_ms);
    put_u64(&mut hasher, wire.current_until_ms);
    put_u32(&mut hasher, wire.issuer_max_use_budget);
    put_u32(&mut hasher, wire.use_budget);
    put_bool(&mut hasher, wire.transferable);
    put_bool(&mut hasher, wire.redelegation_allowed);
    put_u32(&mut hasher, wire.delegation_depth);
    let digest = hasher.finalize();
    let mut output = [0_u8; 32];
    output.copy_from_slice(&digest);
    output
}

fn put_str(hasher: &mut Sha256, value: &str) {
    let bytes = value.as_bytes();
    hasher.update((bytes.len() as u64).to_be_bytes());
    hasher.update(bytes);
}

fn put_u64(hasher: &mut Sha256, value: u64) {
    hasher.update(value.to_be_bytes());
}

fn put_u32(hasher: &mut Sha256, value: u32) {
    hasher.update(value.to_be_bytes());
}

fn put_bool(hasher: &mut Sha256, value: bool) {
    hasher.update([u8::from(value)]);
}

fn put_scope(hasher: &mut Sha256, scope: &CapabilityScopeV1) {
    put_str(hasher, scope.subject.as_str());
    put_str(hasher, scope.purpose.as_str());
    put_str(hasher, scope.resource.as_str());
    put_str(hasher, scope.action.as_str());
}

fn put_presenter(hasher: &mut Sha256, presenter: &PresenterBindingV1) {
    let tag = match presenter.kind {
        PresenterBindingKindV1::Key => 1_u8,
        PresenterBindingKindV1::Session => 2_u8,
        PresenterBindingKindV1::Principal => 3_u8,
    };
    hasher.update([tag]);
    put_str(hasher, presenter.commitment.as_str());
}

fn put_optional_ref(hasher: &mut Sha256, value: Option<&AssuranceProfileRefV1>) {
    match value {
        Some(value) => {
            hasher.update([1_u8]);
            put_str(hasher, value.as_str());
        }
        None => hasher.update([0_u8]),
    }
}

fn put_lease(hasher: &mut Sha256, lease: &EvidenceLease) {
    put_str(hasher, &lease.protocol_version);
    put_u64(hasher, lease.verified_at_ms);
    put_u64(hasher, lease.valid_until_ms);
}
