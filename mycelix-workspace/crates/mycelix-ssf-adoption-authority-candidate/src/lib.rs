// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Non-executable authority candidates derived only from locally approved SSF
//! adoption evidence.
//!
//! This crate deliberately stops before authority issuance. A candidate must
//! be durably registered and rechecked for current freshness by a later layer
//! before it can become usable authority.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_contracts::ConsequenceClass;
use mycelix_ssf_local_adoption_policy::{
    ApprovedLocalAdoptionV1, LocalAdoptionScopeCommitment, PolicyEvaluationReceiptCommitment,
};
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

digest_type!(AdoptionAuthorityCandidateId);
digest_type!(AdoptionAuthorityNonce);

/// v1 intentionally authorizes only one semantic operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AdoptionAuthorityOperationV1 {
    InstallQualifiedRemoteState,
}

/// Requested authority axes. These are untrusted until checked against the
/// exact policy approval consumed by `derive_adoption_authority_candidate`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdoptionAuthorityLeaseRequestV1 {
    pub candidate_id: AdoptionAuthorityCandidateId,
    pub nonce: AdoptionAuthorityNonce,
    pub scope: LocalAdoptionScopeCommitment,
    pub consequence: ConsequenceClass,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdoptionAuthorityCandidateError {
    ScopeMismatch,
    ConsequenceExpanded,
    LeaseOutlivesPolicyApproval,
}

fn validate_request_axes(
    approved_scope: LocalAdoptionScopeCommitment,
    approved_consequence: ConsequenceClass,
    approved_valid_until: u64,
    request: &AdoptionAuthorityLeaseRequestV1,
) -> Result<(), AdoptionAuthorityCandidateError> {
    // v1 has no scope-containment proof, so equality is the only safe rule.
    if request.scope != approved_scope {
        return Err(AdoptionAuthorityCandidateError::ScopeMismatch);
    }
    if request.consequence > approved_consequence {
        return Err(AdoptionAuthorityCandidateError::ConsequenceExpanded);
    }
    if request.valid_until > approved_valid_until {
        return Err(AdoptionAuthorityCandidateError::LeaseOutlivesPolicyApproval);
    }
    Ok(())
}

/// Exact non-executable authority candidate.
///
/// The full typed approval is retained by value, so a later registration layer
/// cannot detach the requested authority from the evidence/policy chain that
/// produced it.
pub struct AdoptionAuthorityCandidateV1<
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
> {
    approved: ApprovedLocalAdoptionV1<P, EV, RV, AQ, LQ, L, R>,
    operation: AdoptionAuthorityOperationV1,
    request: AdoptionAuthorityLeaseRequestV1,
}

impl<P, EV, RV, AQ, LQ, const L: usize, const R: usize>
    AdoptionAuthorityCandidateV1<P, EV, RV, AQ, LQ, L, R>
{
    pub const fn approved(
        &self,
    ) -> &ApprovedLocalAdoptionV1<P, EV, RV, AQ, LQ, L, R> {
        &self.approved
    }

    pub const fn operation(&self) -> AdoptionAuthorityOperationV1 {
        self.operation
    }

    pub const fn candidate_id(&self) -> AdoptionAuthorityCandidateId {
        self.request.candidate_id
    }

    pub const fn nonce(&self) -> AdoptionAuthorityNonce {
        self.request.nonce
    }

    pub const fn scope(&self) -> LocalAdoptionScopeCommitment {
        self.request.scope
    }

    pub const fn consequence(&self) -> ConsequenceClass {
        self.request.consequence
    }

    pub const fn valid_until(&self) -> u64 {
        self.request.valid_until
    }

    pub const fn source_local_state(&self) -> FederationStateHeadV1 {
        self.approved.receipt().subject.local_state
    }

    pub const fn remote_target(&self) -> FederationStateHeadV1 {
        self.approved.receipt().subject.remote_target
    }

    pub const fn policy_receipt_commitment(&self) -> PolicyEvaluationReceiptCommitment {
        self.approved.receipt().receipt_commitment
    }

    /// v1 authority candidates cannot delegate.
    pub const fn delegation_allowed(&self) -> bool {
        false
    }

    /// Candidate identity/nonce are intended to be consumed exactly once by a
    /// future durable-registration layer.
    pub const fn single_use_required(&self) -> bool {
        true
    }

    pub const fn is_durably_registered(&self) -> bool {
        false
    }

    pub const fn requires_durable_registration(&self) -> bool {
        true
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }

    pub const fn contains_state_install_authority(&self) -> bool {
        false
    }
}

/// Consume exact local approval to produce a non-executable authority
/// candidate. Reject/Defer tokens cannot type-check at this boundary.
pub fn derive_adoption_authority_candidate<
    P,
    EV,
    RV,
    AQ,
    LQ,
    const L: usize,
    const R: usize,
>(
    approved: ApprovedLocalAdoptionV1<P, EV, RV, AQ, LQ, L, R>,
    request: AdoptionAuthorityLeaseRequestV1,
) -> Result<
    AdoptionAuthorityCandidateV1<P, EV, RV, AQ, LQ, L, R>,
    AdoptionAuthorityCandidateError,
> {
    validate_request_axes(
        approved.adoption_scope(),
        approved.maximum_follow_on_consequence(),
        approved.valid_until(),
        &request,
    )?;

    Ok(AdoptionAuthorityCandidateV1 {
        approved,
        operation: AdoptionAuthorityOperationV1::InstallQualifiedRemoteState,
        request,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn request(
        scope_byte: u8,
        consequence: ConsequenceClass,
        valid_until: u64,
    ) -> AdoptionAuthorityLeaseRequestV1 {
        AdoptionAuthorityLeaseRequestV1 {
            candidate_id: AdoptionAuthorityCandidateId::from_bytes(bytes(1)),
            nonce: AdoptionAuthorityNonce::from_bytes(bytes(2)),
            scope: LocalAdoptionScopeCommitment::from_bytes(bytes(scope_byte)),
            consequence,
            valid_until,
        }
    }

    #[test]
    fn opaque_scope_requires_exact_match_in_v1() {
        assert_eq!(
            validate_request_axes(
                LocalAdoptionScopeCommitment::from_bytes(bytes(3)),
                ConsequenceClass::C5MachineIsolation,
                100,
                &request(4, ConsequenceClass::C3ScopedRestriction, 90),
            ),
            Err(AdoptionAuthorityCandidateError::ScopeMismatch)
        );
    }

    #[test]
    fn consequence_can_attenuate_but_not_expand() {
        assert_eq!(
            validate_request_axes(
                LocalAdoptionScopeCommitment::from_bytes(bytes(3)),
                ConsequenceClass::C5MachineIsolation,
                100,
                &request(3, ConsequenceClass::C3ScopedRestriction, 90),
            ),
            Ok(())
        );
        assert_eq!(
            validate_request_axes(
                LocalAdoptionScopeCommitment::from_bytes(bytes(3)),
                ConsequenceClass::C3ScopedRestriction,
                100,
                &request(3, ConsequenceClass::C5MachineIsolation, 90),
            ),
            Err(AdoptionAuthorityCandidateError::ConsequenceExpanded)
        );
    }

    #[test]
    fn candidate_cannot_outlive_policy_approval() {
        assert_eq!(
            validate_request_axes(
                LocalAdoptionScopeCommitment::from_bytes(bytes(3)),
                ConsequenceClass::C5MachineIsolation,
                100,
                &request(3, ConsequenceClass::C3ScopedRestriction, 101),
            ),
            Err(AdoptionAuthorityCandidateError::LeaseOutlivesPolicyApproval)
        );
    }
}
