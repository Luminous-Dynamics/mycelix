// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Non-authoritative bounded authority-lease candidates for SSF adoption.
//!
//! Current revalidation proves that one exact single-use reserved issuance
//! lineage is eligible to ask for authority now. This crate still does not mint
//! that authority. It consumes the current-revalidation typestate and produces
//! one bounded lease candidate that must be durably issued by a later layer.
//!
//! v0.1 has no scope-containment proof, so lease scope must equal the exact
//! policy/candidate scope. Consequence and lifetime may attenuate only.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_adoption_authority_candidate::AdoptionAuthorityOperationV1;
use mycelix_ssf_contracts::ConsequenceClass;
use mycelix_ssf_current_authority_revalidation::CurrentlyRevalidatedReservedIssuanceV1;
use mycelix_ssf_local_adoption_policy::LocalAdoptionScopeCommitment;
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

digest_type!(AuthorityLeaseCandidateId);
digest_type!(AuthorityLeaseNonce);

/// Requested lease axes. These remain untrusted until checked against the
/// exact current-revalidated issuance lineage consumed by the constructor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AuthorityLeaseCandidateRequestV1 {
    pub lease_candidate_id: AuthorityLeaseCandidateId,
    pub nonce: AuthorityLeaseNonce,
    pub scope: LocalAdoptionScopeCommitment,
    pub consequence: ConsequenceClass,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorityLeaseCandidateError {
    ScopeMismatch,
    ConsequenceExpanded,
    LeaseOutlivesCurrentRevalidation,
    LeaseAlreadyExpired,
}

fn validate_request_axes(
    approved_scope: LocalAdoptionScopeCommitment,
    maximum_consequence: ConsequenceClass,
    maximum_valid_until: u64,
    latest_possible_now: u64,
    request: &AuthorityLeaseCandidateRequestV1,
) -> Result<(), AuthorityLeaseCandidateError> {
    // There is no independently verified scope lattice in v0.1. Equality is
    // therefore the only safe scope relation; "narrower" is not guessed.
    if request.scope != approved_scope {
        return Err(AuthorityLeaseCandidateError::ScopeMismatch);
    }
    if request.consequence > maximum_consequence {
        return Err(AuthorityLeaseCandidateError::ConsequenceExpanded);
    }
    if request.valid_until > maximum_valid_until {
        return Err(AuthorityLeaseCandidateError::LeaseOutlivesCurrentRevalidation);
    }
    if request.valid_until < latest_possible_now {
        return Err(AuthorityLeaseCandidateError::LeaseAlreadyExpired);
    }
    Ok(())
}

/// Exact non-authoritative lease candidate.
///
/// The complete current-revalidated lineage is retained by value. A later
/// durable issuance layer cannot detach requested scope/consequence/lifetime
/// from the evidence and single-use reservation that justified them.
pub struct BoundedAuthorityLeaseCandidateV1<
    IS,
    RS,
    P,
    EV,
    RV,
    AQ,
    LQ,
    CQ,
    BQ,
    TQ,
    const L: usize,
    const R: usize,
> {
    revalidated:
        CurrentlyRevalidatedReservedIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    operation: AdoptionAuthorityOperationV1,
    request: AuthorityLeaseCandidateRequestV1,
}

impl<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, const L: usize, const R: usize>
    BoundedAuthorityLeaseCandidateV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>
{
    pub const fn revalidated(
        &self,
    ) -> &CurrentlyRevalidatedReservedIssuanceV1<
        IS,
        RS,
        P,
        EV,
        RV,
        AQ,
        LQ,
        CQ,
        BQ,
        TQ,
        L,
        R,
    > {
        &self.revalidated
    }

    pub const fn lease_candidate_id(&self) -> AuthorityLeaseCandidateId {
        self.request.lease_candidate_id
    }

    pub const fn nonce(&self) -> AuthorityLeaseNonce {
        self.request.nonce
    }

    pub const fn operation(&self) -> AdoptionAuthorityOperationV1 {
        self.operation
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

    pub fn source_local_state(&self) -> FederationStateHeadV1 {
        self.revalidated
            .reserved()
            .rebound_registration()
            .candidate()
            .source_local_state()
    }

    pub fn remote_target(&self) -> FederationStateHeadV1 {
        self.revalidated
            .reserved()
            .rebound_registration()
            .candidate()
            .remote_target()
    }

    /// Candidate identity and nonce are opaque values only. Their uniqueness
    /// and durable single-use issuance are the responsibility of the next
    /// boundary.
    pub const fn requires_durable_lease_issuance(&self) -> bool {
        true
    }

    pub const fn is_durably_issued(&self) -> bool {
        false
    }

    pub const fn delegation_allowed(&self) -> bool {
        false
    }

    pub const fn contains_state_install_authority(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

/// Consume current-revalidated evidence to create one bounded, non-authority
/// lease candidate. No older readiness/policy/registration type can enter this
/// boundary directly.
pub fn derive_bounded_authority_lease_candidate<
    IS,
    RS,
    P,
    EV,
    RV,
    AQ,
    LQ,
    CQ,
    BQ,
    TQ,
    const L: usize,
    const R: usize,
>(
    revalidated:
        CurrentlyRevalidatedReservedIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    request: AuthorityLeaseCandidateRequestV1,
) -> Result<
    BoundedAuthorityLeaseCandidateV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    AuthorityLeaseCandidateError,
> {
    let approved_candidate = revalidated
        .reserved()
        .rebound_registration()
        .candidate();

    validate_request_axes(
        approved_candidate.scope(),
        approved_candidate.consequence(),
        revalidated.valid_until(),
        revalidated.evidence().latest_possible_unix_ms,
        &request,
    )?;

    Ok(BoundedAuthorityLeaseCandidateV1 {
        operation: approved_candidate.operation(),
        revalidated,
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
    ) -> AuthorityLeaseCandidateRequestV1 {
        AuthorityLeaseCandidateRequestV1 {
            lease_candidate_id: AuthorityLeaseCandidateId::from_bytes(bytes(1)),
            nonce: AuthorityLeaseNonce::from_bytes(bytes(2)),
            scope: LocalAdoptionScopeCommitment::from_bytes(bytes(scope_byte)),
            consequence,
            valid_until,
        }
    }

    #[test]
    fn scope_requires_exact_match_without_containment_proof() {
        assert_eq!(
            validate_request_axes(
                LocalAdoptionScopeCommitment::from_bytes(bytes(3)),
                ConsequenceClass::C5MachineIsolation,
                200,
                100,
                &request(4, ConsequenceClass::C3ScopedRestriction, 150),
            ),
            Err(AuthorityLeaseCandidateError::ScopeMismatch)
        );
    }

    #[test]
    fn consequence_can_only_attenuate() {
        assert_eq!(
            validate_request_axes(
                LocalAdoptionScopeCommitment::from_bytes(bytes(3)),
                ConsequenceClass::C5MachineIsolation,
                200,
                100,
                &request(3, ConsequenceClass::C3ScopedRestriction, 150),
            ),
            Ok(())
        );
        assert_eq!(
            validate_request_axes(
                LocalAdoptionScopeCommitment::from_bytes(bytes(3)),
                ConsequenceClass::C3ScopedRestriction,
                200,
                100,
                &request(3, ConsequenceClass::C5MachineIsolation, 150),
            ),
            Err(AuthorityLeaseCandidateError::ConsequenceExpanded)
        );
    }

    #[test]
    fn lease_lifetime_can_only_shorten_and_must_still_be_live() {
        assert_eq!(
            validate_request_axes(
                LocalAdoptionScopeCommitment::from_bytes(bytes(3)),
                ConsequenceClass::C5MachineIsolation,
                200,
                100,
                &request(3, ConsequenceClass::C3ScopedRestriction, 201),
            ),
            Err(AuthorityLeaseCandidateError::LeaseOutlivesCurrentRevalidation)
        );
        assert_eq!(
            validate_request_axes(
                LocalAdoptionScopeCommitment::from_bytes(bytes(3)),
                ConsequenceClass::C5MachineIsolation,
                200,
                100,
                &request(3, ConsequenceClass::C3ScopedRestriction, 99),
            ),
            Err(AuthorityLeaseCandidateError::LeaseAlreadyExpired)
        );
    }
}
