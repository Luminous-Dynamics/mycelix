// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Use-time revalidation for one exact durably issued SSF semantic lease.
//!
//! Durable issuance proves that a bounded semantic lease lineage exists. It
//! does not prove that the lease is still safe to exercise now. This crate
//! requires fresh local security context, a fresh time-basis attestation bound
//! to the exact issued lease, and uncertainty-aware trusted current time before
//! a later execution-surface admission layer may consider the lease.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_authorization::VerifierDescriptorV1;
use mycelix_ssf_contracts::{
    derive_valid_until, ValidityBoundary, ValidityInputsV1, SSF_SCHEMA_V1,
};
use mycelix_ssf_current_authority_revalidation::{
    AuthorityTimeBasisV1, CurrentTimeQualifierV1, QualifiedCurrentTimeV1,
};
use mycelix_ssf_issued_authority_lease_rebind::{
    IssuedAuthorityLeaseBindingV1, ReboundIssuedAuthorityLeaseV1,
};
use mycelix_ssf_local_adoption_readiness::{
    LocalPrerequisiteEvidenceV1, LocalQualificationReceiptCommitment,
    QualifiedCurrentLocalContextV1,
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

digest_type!(UseTimeLeaseTimeBasisReceiptCommitment);

/// Untrusted receipt asserting that the exact issued semantic lease and the
/// exact fresh local-context qualification use Unix-millisecond authority
/// freshness semantics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UseTimeLeaseTimeBasisReceiptV1 {
    pub schema_version: u16,
    pub verifier: VerifierDescriptorV1,
    pub issued_lease: IssuedAuthorityLeaseBindingV1,
    pub local_state: FederationStateHeadV1,
    pub local_context_receipt: LocalQualificationReceiptCommitment,
    pub basis: AuthorityTimeBasisV1,
    pub valid_until: u64,
    pub receipt_commitment: UseTimeLeaseTimeBasisReceiptCommitment,
}

impl UseTimeLeaseTimeBasisReceiptV1 {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        verifier: VerifierDescriptorV1,
        issued_lease: IssuedAuthorityLeaseBindingV1,
        local_state: FederationStateHeadV1,
        local_context_receipt: LocalQualificationReceiptCommitment,
        basis: AuthorityTimeBasisV1,
        valid_until: u64,
        receipt_commitment: UseTimeLeaseTimeBasisReceiptCommitment,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            verifier,
            issued_lease,
            local_state,
            local_context_receipt,
            basis,
            valid_until,
            receipt_commitment,
        }
    }
}

pub trait UseTimeLeaseTimeBasisQualifierV1 {
    type Error;

    fn descriptor(&self) -> VerifierDescriptorV1;

    fn qualify_use_time_lease_time_basis(
        &self,
        issued_lease: &IssuedAuthorityLeaseBindingV1,
        local_state: &FederationStateHeadV1,
        local_context_receipt: LocalQualificationReceiptCommitment,
    ) -> Result<UseTimeLeaseTimeBasisReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedUseTimeLeaseTimeBasisQualifierProfileV1 {
    descriptor: VerifierDescriptorV1,
}

impl ExpectedUseTimeLeaseTimeBasisQualifierProfileV1 {
    pub const fn from_trusted_configuration(descriptor: VerifierDescriptorV1) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> VerifierDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UseTimeLeaseTimeBasisError<E> {
    VerifierDescriptorMismatchBefore,
    Verifier(E),
    VerifierDescriptorMismatchAfter,
    UnsupportedReceiptSchema,
    ReceiptVerifierMismatch,
    IssuedLeaseMismatch,
    LocalStateMismatch,
    LocalContextReceiptMismatch,
    UnsupportedTimeBasis,
    ReceiptOutlivesVerifier,
}

pub struct QualifiedUseTimeLeaseTimeBasisV1<Q> {
    expected: ExpectedUseTimeLeaseTimeBasisQualifierProfileV1,
    receipt: UseTimeLeaseTimeBasisReceiptV1,
    _qualifier: PhantomData<Q>,
}

impl<Q> QualifiedUseTimeLeaseTimeBasisV1<Q> {
    pub const fn expected(&self) -> ExpectedUseTimeLeaseTimeBasisQualifierProfileV1 {
        self.expected
    }

    pub const fn receipt(&self) -> UseTimeLeaseTimeBasisReceiptV1 {
        self.receipt
    }

    pub const fn valid_until(&self) -> u64 {
        self.receipt.valid_until
    }

    pub const fn receipt_commitment(&self) -> UseTimeLeaseTimeBasisReceiptCommitment {
        self.receipt.receipt_commitment
    }
}

pub fn qualify_use_time_lease_time_basis<Q: UseTimeLeaseTimeBasisQualifierV1>(
    expected: ExpectedUseTimeLeaseTimeBasisQualifierProfileV1,
    qualifier: &Q,
    issued_lease: IssuedAuthorityLeaseBindingV1,
    local_state: FederationStateHeadV1,
    local_context_receipt: LocalQualificationReceiptCommitment,
) -> Result<QualifiedUseTimeLeaseTimeBasisV1<Q>, UseTimeLeaseTimeBasisError<Q::Error>> {
    if qualifier.descriptor() != expected.descriptor {
        return Err(UseTimeLeaseTimeBasisError::VerifierDescriptorMismatchBefore);
    }

    let receipt = qualifier
        .qualify_use_time_lease_time_basis(&issued_lease, &local_state, local_context_receipt)
        .map_err(UseTimeLeaseTimeBasisError::Verifier)?;

    if qualifier.descriptor() != expected.descriptor {
        return Err(UseTimeLeaseTimeBasisError::VerifierDescriptorMismatchAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(UseTimeLeaseTimeBasisError::UnsupportedReceiptSchema);
    }
    if receipt.verifier != expected.descriptor {
        return Err(UseTimeLeaseTimeBasisError::ReceiptVerifierMismatch);
    }
    if receipt.issued_lease != issued_lease {
        return Err(UseTimeLeaseTimeBasisError::IssuedLeaseMismatch);
    }
    if receipt.local_state != local_state {
        return Err(UseTimeLeaseTimeBasisError::LocalStateMismatch);
    }
    if receipt.local_context_receipt != local_context_receipt {
        return Err(UseTimeLeaseTimeBasisError::LocalContextReceiptMismatch);
    }
    if receipt.basis != AuthorityTimeBasisV1::UnixMillisecondsUtc {
        return Err(UseTimeLeaseTimeBasisError::UnsupportedTimeBasis);
    }
    if receipt.valid_until > expected.descriptor.valid_until {
        return Err(UseTimeLeaseTimeBasisError::ReceiptOutlivesVerifier);
    }

    Ok(QualifiedUseTimeLeaseTimeBasisV1 {
        expected,
        receipt,
        _qualifier: PhantomData,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UseTimeLeaseRevalidationError {
    CurrentLocalStateMismatch,
    TimeBasisIssuedLeaseMismatch,
    TimeBasisLocalStateMismatch,
    TimeBasisLocalContextReceiptMismatch,
    MembershipFreshnessRequired,
    IdentityKeyFreshnessRequired,
    RevocationFreshnessRequired,
    PolicyFreshnessRequired,
    MachineHealthFreshnessRequired,
    ScientificQualificationFreshnessRequired,
    LocalFreshnessDerivationFailed,
    LeaseAlreadyExpired,
}

fn require_known(
    boundary: ValidityBoundary,
    error: UseTimeLeaseRevalidationError,
) -> Result<u64, UseTimeLeaseRevalidationError> {
    match boundary {
        ValidityBoundary::KnownUntil(value) => Ok(value),
        ValidityBoundary::NotApplicable | ValidityBoundary::Unknown => Err(error),
    }
}

fn required_local_valid_until(
    validity: ValidityInputsV1,
) -> Result<u64, UseTimeLeaseRevalidationError> {
    require_known(
        validity.membership,
        UseTimeLeaseRevalidationError::MembershipFreshnessRequired,
    )?;
    require_known(
        validity.identity_key,
        UseTimeLeaseRevalidationError::IdentityKeyFreshnessRequired,
    )?;
    require_known(
        validity.revocation_snapshot,
        UseTimeLeaseRevalidationError::RevocationFreshnessRequired,
    )?;
    require_known(
        validity.policy,
        UseTimeLeaseRevalidationError::PolicyFreshnessRequired,
    )?;
    require_known(
        validity.machine_health,
        UseTimeLeaseRevalidationError::MachineHealthFreshnessRequired,
    )?;
    require_known(
        validity.scientific_qualification,
        UseTimeLeaseRevalidationError::ScientificQualificationFreshnessRequired,
    )?;

    derive_valid_until(validity)
        .map_err(|_| UseTimeLeaseRevalidationError::LocalFreshnessDerivationFailed)?
        .ok_or(UseTimeLeaseRevalidationError::LocalFreshnessDerivationFailed)
}

fn derive_use_time_valid_until(
    issued_lease_valid_until: u64,
    current_context_valid_until: u64,
    prerequisite_validity: ValidityInputsV1,
    time_basis_valid_until: u64,
    current_time_valid_until: u64,
    latest_possible_now: u64,
) -> Result<u64, UseTimeLeaseRevalidationError> {
    let local_valid_until = required_local_valid_until(prerequisite_validity)?;
    let valid_until = issued_lease_valid_until
        .min(current_context_valid_until)
        .min(local_valid_until)
        .min(time_basis_valid_until)
        .min(current_time_valid_until);

    if latest_possible_now > valid_until {
        return Err(UseTimeLeaseRevalidationError::LeaseAlreadyExpired);
    }

    Ok(valid_until)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UseTimeLeaseEvidenceV1 {
    pub issued_lease: IssuedAuthorityLeaseBindingV1,
    pub local_state: FederationStateHeadV1,
    pub prerequisite_evidence: LocalPrerequisiteEvidenceV1,
    pub prerequisite_validity: ValidityInputsV1,
    pub local_context_receipt: LocalQualificationReceiptCommitment,
    pub local_context_valid_until: u64,
    pub time_basis_verifier: VerifierDescriptorV1,
    pub time_basis_receipt: UseTimeLeaseTimeBasisReceiptCommitment,
    pub time_basis_valid_until: u64,
    pub current_time_receipt: mycelix_ssf_current_authority_revalidation::CurrentTimeReceiptCommitment,
    pub latest_possible_unix_ms: u64,
    pub current_time_valid_until: u64,
    pub valid_until: u64,
}

/// Compact exact subject for the next execution-surface admission boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UseTimeRevalidatedLeaseBindingV1 {
    pub evidence: UseTimeLeaseEvidenceV1,
}

pub struct UseTimeRevalidatedAuthorityLeaseV1<
    S,
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
    UCQ,
    UBQ,
    UTQ,
    const L: usize,
    const R: usize,
> {
    lease: ReboundIssuedAuthorityLeaseV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    current_context: QualifiedCurrentLocalContextV1<UCQ>,
    time_basis: QualifiedUseTimeLeaseTimeBasisV1<UBQ>,
    current_time: QualifiedCurrentTimeV1<UTQ>,
    evidence: UseTimeLeaseEvidenceV1,
}

impl<
        S,
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
        UCQ,
        UBQ,
        UTQ,
        const L: usize,
        const R: usize,
    > UseTimeRevalidatedAuthorityLeaseV1<
        S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, UCQ, UBQ, UTQ, L, R,
    >
{
    pub const fn lease(
        &self,
    ) -> &ReboundIssuedAuthorityLeaseV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R> {
        &self.lease
    }

    pub const fn current_context(&self) -> &QualifiedCurrentLocalContextV1<UCQ> {
        &self.current_context
    }

    pub const fn time_basis(&self) -> &QualifiedUseTimeLeaseTimeBasisV1<UBQ> {
        &self.time_basis
    }

    pub const fn current_time(&self) -> &QualifiedCurrentTimeV1<UTQ> {
        &self.current_time
    }

    pub const fn evidence(&self) -> UseTimeLeaseEvidenceV1 {
        self.evidence
    }

    pub const fn binding(&self) -> UseTimeRevalidatedLeaseBindingV1 {
        UseTimeRevalidatedLeaseBindingV1 {
            evidence: self.evidence,
        }
    }

    pub const fn valid_until(&self) -> u64 {
        self.evidence.valid_until
    }

    pub const fn eligible_for_execution_surface_admission(&self) -> bool {
        true
    }

    pub const fn contains_execution_surface_binding(&self) -> bool {
        false
    }

    pub const fn contains_state_install_authority(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

pub fn revalidate_issued_authority_lease_use_time<
    S,
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
    UCQ,
    UBQ,
    UTQ,
    const L: usize,
    const R: usize,
>(
    lease: ReboundIssuedAuthorityLeaseV1<S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    current_context: QualifiedCurrentLocalContextV1<UCQ>,
    time_basis: QualifiedUseTimeLeaseTimeBasisV1<UBQ>,
    current_time: QualifiedCurrentTimeV1<UTQ>,
) -> Result<
    UseTimeRevalidatedAuthorityLeaseV1<
        S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, UCQ, UBQ, UTQ, L, R,
    >,
    UseTimeLeaseRevalidationError,
>
where
    UTQ: CurrentTimeQualifierV1,
{
    let approved_source_state = lease.candidate().source_local_state();
    if current_context.local_state() != approved_source_state {
        return Err(UseTimeLeaseRevalidationError::CurrentLocalStateMismatch);
    }

    let time_basis_receipt = time_basis.receipt();
    if time_basis_receipt.issued_lease != lease.binding() {
        return Err(UseTimeLeaseRevalidationError::TimeBasisIssuedLeaseMismatch);
    }
    if time_basis_receipt.local_state != current_context.local_state() {
        return Err(UseTimeLeaseRevalidationError::TimeBasisLocalStateMismatch);
    }
    if time_basis_receipt.local_context_receipt != current_context.receipt_commitment() {
        return Err(UseTimeLeaseRevalidationError::TimeBasisLocalContextReceiptMismatch);
    }

    let prerequisite_validity = current_context.prerequisite_validity();
    let valid_until = derive_use_time_valid_until(
        lease.authority_valid_until(),
        current_context.valid_until(),
        prerequisite_validity,
        time_basis.valid_until(),
        current_time.valid_until(),
        current_time.latest_possible_unix_ms(),
    )?;

    let evidence = UseTimeLeaseEvidenceV1 {
        issued_lease: lease.binding(),
        local_state: current_context.local_state(),
        prerequisite_evidence: current_context.prerequisite_evidence(),
        prerequisite_validity,
        local_context_receipt: current_context.receipt_commitment(),
        local_context_valid_until: current_context.valid_until(),
        time_basis_verifier: time_basis.expected().descriptor(),
        time_basis_receipt: time_basis.receipt_commitment(),
        time_basis_valid_until: time_basis.valid_until(),
        current_time_receipt: current_time.receipt_commitment(),
        latest_possible_unix_ms: current_time.latest_possible_unix_ms(),
        current_time_valid_until: current_time.valid_until(),
        valid_until,
    };

    Ok(UseTimeRevalidatedAuthorityLeaseV1 {
        lease,
        current_context,
        time_basis,
        current_time,
        evidence,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn validity(value: ValidityBoundary) -> ValidityInputsV1 {
        ValidityInputsV1 {
            membership: value,
            identity_key: value,
            revocation_snapshot: value,
            policy: value,
            machine_health: value,
            scientific_qualification: value,
        }
    }

    #[test]
    fn all_six_prerequisites_remain_mandatory_at_use_time() {
        let mut inputs = validity(ValidityBoundary::KnownUntil(200));
        inputs.machine_health = ValidityBoundary::Unknown;
        assert_eq!(
            required_local_valid_until(inputs),
            Err(UseTimeLeaseRevalidationError::MachineHealthFreshnessRequired)
        );

        let mut inputs = validity(ValidityBoundary::KnownUntil(200));
        inputs.policy = ValidityBoundary::NotApplicable;
        assert_eq!(
            required_local_valid_until(inputs),
            Err(UseTimeLeaseRevalidationError::PolicyFreshnessRequired)
        );
    }

    #[test]
    fn use_time_natural_expiry_selects_oldest_boundary() {
        let mut inputs = validity(ValidityBoundary::KnownUntil(190));
        inputs.revocation_snapshot = ValidityBoundary::KnownUntil(140);
        assert_eq!(
            derive_use_time_valid_until(180, 175, inputs, 170, 165, 100),
            Ok(140)
        );
    }

    #[test]
    fn latest_plausible_time_must_fit_inside_final_ceiling() {
        assert_eq!(
            derive_use_time_valid_until(
                180,
                175,
                validity(ValidityBoundary::KnownUntil(170)),
                165,
                160,
                161,
            ),
            Err(UseTimeLeaseRevalidationError::LeaseAlreadyExpired)
        );
    }

    #[test]
    fn fresh_evidence_cannot_extend_issued_lease_lifetime() {
        assert_eq!(
            derive_use_time_valid_until(
                120,
                500,
                validity(ValidityBoundary::KnownUntil(500)),
                500,
                500,
                100,
            ),
            Ok(120)
        );
    }
}
