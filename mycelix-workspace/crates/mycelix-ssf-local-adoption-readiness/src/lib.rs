// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Local adoption readiness for the Mycelix Solar-System Federation profile.
//!
//! This layer binds exact remote-path authorization coverage to independently
//! qualified local evidence. It proves only that remote adoption is ready for
//! a later local policy decision. It does not decide that policy, install
//! state, mint effect authority, or execute anything.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_authorization::VerifierDescriptorV1;
use mycelix_ssf_authorization_coverage::RemotePathAuthorizationCoverageV1;
use mycelix_ssf_contracts::{
    derive_valid_until, EvidenceCommitment, ValidityBoundary, ValidityInputsV1, SSF_SCHEMA_V1,
};
use mycelix_ssf_reconciliation::CausalRelationV1;
use mycelix_ssf_reconciliation_safety::RemotePromotionGateV1;
use mycelix_ssf_snapshots::FederationStateHeadV1;

macro_rules! digest_type {
    ($name:ident) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        #[repr(transparent)]
        pub struct $name([u8; 32]);

        impl $name {
            pub const fn from_bytes(bytes: [u8; 32]) -> Self {
                Self(bytes)
            }

            pub const fn as_bytes(&self) -> &[u8; 32] {
                &self.0
            }
        }
    };
}

digest_type!(LocalQualificationReceiptCommitment);

/// Exact evidence lineages underlying every mandatory local freshness input.
///
/// These are evidence identifiers only. This crate does not choose the digest
/// algorithm or prove that the referenced evidence is correct.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LocalPrerequisiteEvidenceV1 {
    pub membership: EvidenceCommitment,
    pub identity_key: EvidenceCommitment,
    pub revocation_snapshot: EvidenceCommitment,
    pub policy: EvidenceCommitment,
    pub machine_health: EvidenceCommitment,
    pub scientific_qualification: EvidenceCommitment,
}

/// Untrusted receipt asserting that one exact common ancestor was qualified by
/// a concrete local verifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommonAncestorQualificationReceiptV1 {
    pub schema_version: u16,
    pub verifier: VerifierDescriptorV1,
    pub ancestor: FederationStateHeadV1,
    pub evidence_root: EvidenceCommitment,
    pub valid_until: u64,
    pub receipt_commitment: LocalQualificationReceiptCommitment,
}

impl CommonAncestorQualificationReceiptV1 {
    pub const fn new(
        verifier: VerifierDescriptorV1,
        ancestor: FederationStateHeadV1,
        evidence_root: EvidenceCommitment,
        valid_until: u64,
        receipt_commitment: LocalQualificationReceiptCommitment,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            verifier,
            ancestor,
            evidence_root,
            valid_until,
            receipt_commitment,
        }
    }
}

/// Untrusted receipt asserting that one exact state head is the current local
/// security context and carrying all mandatory prerequisite evidence/freshness.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurrentLocalContextReceiptV1 {
    pub schema_version: u16,
    pub verifier: VerifierDescriptorV1,
    pub local_state: FederationStateHeadV1,
    pub prerequisite_evidence: LocalPrerequisiteEvidenceV1,
    pub prerequisite_validity: ValidityInputsV1,
    pub valid_until: u64,
    pub receipt_commitment: LocalQualificationReceiptCommitment,
}

impl CurrentLocalContextReceiptV1 {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        verifier: VerifierDescriptorV1,
        local_state: FederationStateHeadV1,
        prerequisite_evidence: LocalPrerequisiteEvidenceV1,
        prerequisite_validity: ValidityInputsV1,
        valid_until: u64,
        receipt_commitment: LocalQualificationReceiptCommitment,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            verifier,
            local_state,
            prerequisite_evidence,
            prerequisite_validity,
            valid_until,
            receipt_commitment,
        }
    }
}

/// Concrete verifier capability for qualifying the exact common ancestor.
pub trait CommonAncestorQualifierV1 {
    type Error;

    fn descriptor(&self) -> VerifierDescriptorV1;

    fn qualify_common_ancestor(
        &self,
        ancestor: &FederationStateHeadV1,
    ) -> Result<CommonAncestorQualificationReceiptV1, Self::Error>;
}

/// Concrete verifier capability for establishing the exact current local
/// security context and its mandatory prerequisite evidence/freshness.
pub trait CurrentLocalContextQualifierV1 {
    type Error;

    fn descriptor(&self) -> VerifierDescriptorV1;

    fn qualify_current_local_context(
        &self,
        local_state: &FederationStateHeadV1,
    ) -> Result<CurrentLocalContextReceiptV1, Self::Error>;
}

/// Independently supplied expected verifier profile for ancestor qualification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedCommonAncestorQualifierProfileV1 {
    descriptor: VerifierDescriptorV1,
}

impl ExpectedCommonAncestorQualifierProfileV1 {
    pub const fn from_trusted_configuration(descriptor: VerifierDescriptorV1) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> VerifierDescriptorV1 {
        self.descriptor
    }
}

/// Independently supplied expected verifier profile for current-context
/// qualification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedCurrentLocalContextQualifierProfileV1 {
    descriptor: VerifierDescriptorV1,
}

impl ExpectedCurrentLocalContextQualifierProfileV1 {
    pub const fn from_trusted_configuration(descriptor: VerifierDescriptorV1) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> VerifierDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocalQualificationBindingError<E> {
    Verifier(E),
    UnsupportedReceiptSchema,
    VerifierDescriptorMismatchBefore,
    VerifierDescriptorMismatchAfter,
    ReceiptVerifierMismatch,
    StateHeadMismatch,
    ReceiptOutlivesVerifier,
}

/// Exact common-ancestor qualification bound to concrete qualifier type `Q`.
pub struct QualifiedCommonAncestorV1<Q> {
    expected: ExpectedCommonAncestorQualifierProfileV1,
    receipt: CommonAncestorQualificationReceiptV1,
    _qualifier: PhantomData<fn() -> Q>,
}

impl<Q> QualifiedCommonAncestorV1<Q> {
    pub const fn ancestor(&self) -> FederationStateHeadV1 {
        self.receipt.ancestor
    }

    pub const fn evidence_root(&self) -> EvidenceCommitment {
        self.receipt.evidence_root
    }

    pub const fn valid_until(&self) -> u64 {
        self.receipt.valid_until
    }

    pub const fn receipt_commitment(&self) -> LocalQualificationReceiptCommitment {
        self.receipt.receipt_commitment
    }

    pub const fn expected_profile(&self) -> ExpectedCommonAncestorQualifierProfileV1 {
        self.expected
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

/// Exact current-local-context qualification bound to concrete qualifier type
/// `Q`.
pub struct QualifiedCurrentLocalContextV1<Q> {
    expected: ExpectedCurrentLocalContextQualifierProfileV1,
    receipt: CurrentLocalContextReceiptV1,
    _qualifier: PhantomData<fn() -> Q>,
}

impl<Q> QualifiedCurrentLocalContextV1<Q> {
    pub const fn local_state(&self) -> FederationStateHeadV1 {
        self.receipt.local_state
    }

    pub const fn prerequisite_evidence(&self) -> LocalPrerequisiteEvidenceV1 {
        self.receipt.prerequisite_evidence
    }

    pub const fn prerequisite_validity(&self) -> ValidityInputsV1 {
        self.receipt.prerequisite_validity
    }

    pub const fn valid_until(&self) -> u64 {
        self.receipt.valid_until
    }

    pub const fn receipt_commitment(&self) -> LocalQualificationReceiptCommitment {
        self.receipt.receipt_commitment
    }

    pub const fn expected_profile(&self) -> ExpectedCurrentLocalContextQualifierProfileV1 {
        self.expected
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

/// Bind one common-ancestor qualification to an independently expected
/// concrete verifier descriptor, including a before/after generation check.
pub fn qualify_common_ancestor<Q: CommonAncestorQualifierV1>(
    expected: ExpectedCommonAncestorQualifierProfileV1,
    qualifier: &Q,
    ancestor: FederationStateHeadV1,
) -> Result<QualifiedCommonAncestorV1<Q>, LocalQualificationBindingError<Q::Error>> {
    if qualifier.descriptor() != expected.descriptor {
        return Err(LocalQualificationBindingError::VerifierDescriptorMismatchBefore);
    }

    let receipt = qualifier
        .qualify_common_ancestor(&ancestor)
        .map_err(LocalQualificationBindingError::Verifier)?;

    if qualifier.descriptor() != expected.descriptor {
        return Err(LocalQualificationBindingError::VerifierDescriptorMismatchAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(LocalQualificationBindingError::UnsupportedReceiptSchema);
    }
    if receipt.verifier != expected.descriptor {
        return Err(LocalQualificationBindingError::ReceiptVerifierMismatch);
    }
    if receipt.ancestor != ancestor {
        return Err(LocalQualificationBindingError::StateHeadMismatch);
    }
    if receipt.valid_until > expected.descriptor.valid_until {
        return Err(LocalQualificationBindingError::ReceiptOutlivesVerifier);
    }

    Ok(QualifiedCommonAncestorV1 {
        expected,
        receipt,
        _qualifier: PhantomData,
    })
}

/// Bind one current-local-context qualification to an independently expected
/// concrete verifier descriptor, including a before/after generation check.
pub fn qualify_current_local_context<Q: CurrentLocalContextQualifierV1>(
    expected: ExpectedCurrentLocalContextQualifierProfileV1,
    qualifier: &Q,
    local_state: FederationStateHeadV1,
) -> Result<QualifiedCurrentLocalContextV1<Q>, LocalQualificationBindingError<Q::Error>> {
    if qualifier.descriptor() != expected.descriptor {
        return Err(LocalQualificationBindingError::VerifierDescriptorMismatchBefore);
    }

    let receipt = qualifier
        .qualify_current_local_context(&local_state)
        .map_err(LocalQualificationBindingError::Verifier)?;

    if qualifier.descriptor() != expected.descriptor {
        return Err(LocalQualificationBindingError::VerifierDescriptorMismatchAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(LocalQualificationBindingError::UnsupportedReceiptSchema);
    }
    if receipt.verifier != expected.descriptor {
        return Err(LocalQualificationBindingError::ReceiptVerifierMismatch);
    }
    if receipt.local_state != local_state {
        return Err(LocalQualificationBindingError::StateHeadMismatch);
    }
    if receipt.valid_until > expected.descriptor.valid_until {
        return Err(LocalQualificationBindingError::ReceiptOutlivesVerifier);
    }

    Ok(QualifiedCurrentLocalContextV1 {
        expected,
        receipt,
        _qualifier: PhantomData,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdoptionReadinessError {
    PromotionGateNotEligible,
    RelationNotRemoteStrictExtension,
    SafetyFloorMissingCommonAncestorRequirement,
    SafetyFloorMissingLocalPolicyRequirement,
    CommonAncestorMismatch,
    CurrentLocalStateMismatch,
    MembershipFreshnessRequired,
    IdentityKeyFreshnessRequired,
    RevocationFreshnessRequired,
    PolicyFreshnessRequired,
    MachineHealthFreshnessRequired,
    ScientificQualificationFreshnessRequired,
    LocalFreshnessDerivationFailed,
}

fn require_known(
    boundary: ValidityBoundary,
    error: AdoptionReadinessError,
) -> Result<u64, AdoptionReadinessError> {
    match boundary {
        ValidityBoundary::KnownUntil(value) => Ok(value),
        ValidityBoundary::NotApplicable | ValidityBoundary::Unknown => Err(error),
    }
}

/// Opaque evidence that exact remote-path authorization coverage is ready to be
/// considered by a later approved local adoption policy evaluator.
///
/// This is deliberately not local adoption approval and contains no effect
/// authority.
pub struct LocalAdoptionReadyEvidenceV1<EV, RV, AQ, LQ, const L: usize, const R: usize> {
    coverage: RemotePathAuthorizationCoverageV1<EV, RV, L, R>,
    common_ancestor: QualifiedCommonAncestorV1<AQ>,
    current_local_context: QualifiedCurrentLocalContextV1<LQ>,
    valid_until: u64,
}

impl<EV, RV, AQ, LQ, const L: usize, const R: usize>
    LocalAdoptionReadyEvidenceV1<EV, RV, AQ, LQ, L, R>
{
    pub const fn coverage(&self) -> &RemotePathAuthorizationCoverageV1<EV, RV, L, R> {
        &self.coverage
    }

    pub const fn common_ancestor(&self) -> &QualifiedCommonAncestorV1<AQ> {
        &self.common_ancestor
    }

    pub const fn current_local_context(&self) -> &QualifiedCurrentLocalContextV1<LQ> {
        &self.current_local_context
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn contains_local_adoption_policy_qualification(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }

    pub const fn remote_state_is_still_evidence_only(&self) -> bool {
        true
    }
}

/// Derive readiness for one exact remote strict-extension candidate.
///
/// All six local prerequisites are mandatory and bounded for this profile.
/// `NotApplicable` is therefore treated as failure rather than omission.
pub fn derive_local_adoption_readiness<EV, RV, AQ, LQ, const L: usize, const R: usize>(
    coverage: RemotePathAuthorizationCoverageV1<EV, RV, L, R>,
    common_ancestor: QualifiedCommonAncestorV1<AQ>,
    current_local_context: QualifiedCurrentLocalContextV1<LQ>,
) -> Result<LocalAdoptionReadyEvidenceV1<EV, RV, AQ, LQ, L, R>, AdoptionReadinessError> {
    let floor = coverage.safety_floor();
    let assessment = floor.assessment();

    if floor.remote_promotion_gate() != RemotePromotionGateV1::EligibleOnlyAfterLocalQualification {
        return Err(AdoptionReadinessError::PromotionGateNotEligible);
    }
    if assessment.relation() != CausalRelationV1::RemoteStrictExtension {
        return Err(AdoptionReadinessError::RelationNotRemoteStrictExtension);
    }
    if !floor.require_common_ancestor_qualification() {
        return Err(AdoptionReadinessError::SafetyFloorMissingCommonAncestorRequirement);
    }
    if !floor.require_local_policy_qualification() {
        return Err(AdoptionReadinessError::SafetyFloorMissingLocalPolicyRequirement);
    }
    if common_ancestor.ancestor() != assessment.common_ancestor() {
        return Err(AdoptionReadinessError::CommonAncestorMismatch);
    }
    if current_local_context.local_state() != assessment.local_tip() {
        return Err(AdoptionReadinessError::CurrentLocalStateMismatch);
    }

    let local = current_local_context.prerequisite_validity();
    require_known(local.membership, AdoptionReadinessError::MembershipFreshnessRequired)?;
    require_known(
        local.identity_key,
        AdoptionReadinessError::IdentityKeyFreshnessRequired,
    )?;
    require_known(
        local.revocation_snapshot,
        AdoptionReadinessError::RevocationFreshnessRequired,
    )?;
    require_known(local.policy, AdoptionReadinessError::PolicyFreshnessRequired)?;
    require_known(
        local.machine_health,
        AdoptionReadinessError::MachineHealthFreshnessRequired,
    )?;
    require_known(
        local.scientific_qualification,
        AdoptionReadinessError::ScientificQualificationFreshnessRequired,
    )?;

    let local_prerequisite_ceiling = derive_valid_until(local)
        .map_err(|_| AdoptionReadinessError::LocalFreshnessDerivationFailed)?
        .ok_or(AdoptionReadinessError::LocalFreshnessDerivationFailed)?;

    let valid_until = coverage
        .valid_until()
        .min(common_ancestor.valid_until())
        .min(current_local_context.valid_until())
        .min(local_prerequisite_ceiling);

    Ok(LocalAdoptionReadyEvidenceV1 {
        coverage,
        common_ancestor,
        current_local_context,
        valid_until,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn not_applicable_is_not_accepted_as_required_freshness() {
        assert_eq!(
            require_known(
                ValidityBoundary::NotApplicable,
                AdoptionReadinessError::PolicyFreshnessRequired,
            ),
            Err(AdoptionReadinessError::PolicyFreshnessRequired)
        );
    }

    #[test]
    fn unknown_is_not_accepted_as_required_freshness() {
        assert_eq!(
            require_known(
                ValidityBoundary::Unknown,
                AdoptionReadinessError::MachineHealthFreshnessRequired,
            ),
            Err(AdoptionReadinessError::MachineHealthFreshnessRequired)
        );
    }

    #[test]
    fn known_boundary_is_preserved() {
        assert_eq!(
            require_known(
                ValidityBoundary::KnownUntil(42),
                AdoptionReadinessError::MembershipFreshnessRequired,
            ),
            Ok(42)
        );
    }
}
