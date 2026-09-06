// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Current authority revalidation for a single-use reserved SSF issuance lineage.
//!
//! Reservation proves historical single-use consumption. It does not prove that
//! authority is safe *now*. This crate requires a freshly qualified exact local
//! security context, an independently verified common time-basis attestation,
//! and trusted absolute time with explicit uncertainty before a reserved lineage
//! may become eligible for a later authority-issuance boundary.
//!
//! v0.1 deliberately requires the exact current local state to equal the state
//! that the approved adoption decision originally authorized. A changed local
//! state requires a new qualification/policy lineage; this crate does not infer
//! compatibility from "newer" generations.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_authorization::VerifierDescriptorV1;
use mycelix_ssf_contracts::{
    derive_valid_until, ValidityBoundary, ValidityInputsV1, SSF_SCHEMA_V1,
};
use mycelix_ssf_local_adoption_readiness::{
    LocalPrerequisiteEvidenceV1, LocalQualificationReceiptCommitment,
    QualifiedCurrentLocalContextV1,
};
use mycelix_ssf_reserved_issuance_rebind::{
    ReboundReservedIssuanceV1, ReservedIssuanceBindingV1,
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

digest_type!(TimeSourceCommitment);
digest_type!(CurrentTimeReceiptCommitment);
digest_type!(AuthorityTimeBasisReceiptCommitment);

/// Closed v0.1 canonical time basis for authority freshness comparisons.
///
/// Other time evidence kinds remain valid evidence elsewhere in SSF, but this
/// issuance profile does not silently coerce causal or monotonic time into Unix
/// milliseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AuthorityTimeBasisV1 {
    UnixMillisecondsUtc,
}

/// Untrusted receipt asserting that the exact reserved authority ceiling and
/// exact fresh local-context freshness boundaries use one canonical time basis.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AuthorityTimeBasisReceiptV1 {
    pub schema_version: u16,
    pub verifier: VerifierDescriptorV1,
    pub reserved_issuance: ReservedIssuanceBindingV1,
    pub local_state: FederationStateHeadV1,
    pub local_context_receipt: LocalQualificationReceiptCommitment,
    pub basis: AuthorityTimeBasisV1,
    pub valid_until: u64,
    pub receipt_commitment: AuthorityTimeBasisReceiptCommitment,
}

impl AuthorityTimeBasisReceiptV1 {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        verifier: VerifierDescriptorV1,
        reserved_issuance: ReservedIssuanceBindingV1,
        local_state: FederationStateHeadV1,
        local_context_receipt: LocalQualificationReceiptCommitment,
        basis: AuthorityTimeBasisV1,
        valid_until: u64,
        receipt_commitment: AuthorityTimeBasisReceiptCommitment,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            verifier,
            reserved_issuance,
            local_state,
            local_context_receipt,
            basis,
            valid_until,
            receipt_commitment,
        }
    }
}

/// Concrete verifier for the canonical time-basis relation between the exact
/// reserved lineage and exact current-local-context receipt.
pub trait AuthorityTimeBasisQualifierV1 {
    type Error;

    fn descriptor(&self) -> VerifierDescriptorV1;

    fn qualify_authority_time_basis(
        &self,
        reserved_issuance: &ReservedIssuanceBindingV1,
        local_state: &FederationStateHeadV1,
        local_context_receipt: LocalQualificationReceiptCommitment,
    ) -> Result<AuthorityTimeBasisReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedAuthorityTimeBasisQualifierProfileV1 {
    descriptor: VerifierDescriptorV1,
}

impl ExpectedAuthorityTimeBasisQualifierProfileV1 {
    pub const fn from_trusted_configuration(descriptor: VerifierDescriptorV1) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> VerifierDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorityTimeBasisBindingError<E> {
    Verifier(E),
    VerifierDescriptorMismatchBefore,
    VerifierDescriptorMismatchAfter,
    UnsupportedReceiptSchema,
    ReceiptVerifierMismatch,
    ReservedIssuanceMismatch,
    LocalStateMismatch,
    LocalContextReceiptMismatch,
    UnsupportedTimeBasis,
    ReceiptOutlivesVerifier,
}

/// Exact common time-basis evidence bound to concrete qualifier type `BQ`.
pub struct QualifiedAuthorityTimeBasisV1<BQ> {
    expected: ExpectedAuthorityTimeBasisQualifierProfileV1,
    receipt: AuthorityTimeBasisReceiptV1,
    _qualifier: PhantomData<fn() -> BQ>,
}

impl<BQ> QualifiedAuthorityTimeBasisV1<BQ> {
    pub const fn expected_profile(&self) -> ExpectedAuthorityTimeBasisQualifierProfileV1 {
        self.expected
    }

    pub const fn reserved_issuance(&self) -> ReservedIssuanceBindingV1 {
        self.receipt.reserved_issuance
    }

    pub const fn local_state(&self) -> FederationStateHeadV1 {
        self.receipt.local_state
    }

    pub const fn local_context_receipt(&self) -> LocalQualificationReceiptCommitment {
        self.receipt.local_context_receipt
    }

    pub const fn basis(&self) -> AuthorityTimeBasisV1 {
        self.receipt.basis
    }

    pub const fn valid_until(&self) -> u64 {
        self.receipt.valid_until
    }

    pub const fn receipt_commitment(&self) -> AuthorityTimeBasisReceiptCommitment {
        self.receipt.receipt_commitment
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

/// Bind the exact reserved lineage and exact current local-context evidence to
/// the only v0.1 authority time basis.
pub fn qualify_authority_time_basis<BQ: AuthorityTimeBasisQualifierV1>(
    expected: ExpectedAuthorityTimeBasisQualifierProfileV1,
    qualifier: &BQ,
    reserved_issuance: ReservedIssuanceBindingV1,
    local_state: FederationStateHeadV1,
    local_context_receipt: LocalQualificationReceiptCommitment,
) -> Result<QualifiedAuthorityTimeBasisV1<BQ>, AuthorityTimeBasisBindingError<BQ::Error>> {
    if qualifier.descriptor() != expected.descriptor {
        return Err(AuthorityTimeBasisBindingError::VerifierDescriptorMismatchBefore);
    }

    let receipt = qualifier
        .qualify_authority_time_basis(
            &reserved_issuance,
            &local_state,
            local_context_receipt,
        )
        .map_err(AuthorityTimeBasisBindingError::Verifier)?;

    if qualifier.descriptor() != expected.descriptor {
        return Err(AuthorityTimeBasisBindingError::VerifierDescriptorMismatchAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(AuthorityTimeBasisBindingError::UnsupportedReceiptSchema);
    }
    if receipt.verifier != expected.descriptor {
        return Err(AuthorityTimeBasisBindingError::ReceiptVerifierMismatch);
    }
    if receipt.reserved_issuance != reserved_issuance {
        return Err(AuthorityTimeBasisBindingError::ReservedIssuanceMismatch);
    }
    if receipt.local_state != local_state {
        return Err(AuthorityTimeBasisBindingError::LocalStateMismatch);
    }
    if receipt.local_context_receipt != local_context_receipt {
        return Err(AuthorityTimeBasisBindingError::LocalContextReceiptMismatch);
    }
    if receipt.basis != AuthorityTimeBasisV1::UnixMillisecondsUtc {
        return Err(AuthorityTimeBasisBindingError::UnsupportedTimeBasis);
    }
    if receipt.valid_until > expected.descriptor.valid_until {
        return Err(AuthorityTimeBasisBindingError::ReceiptOutlivesVerifier);
    }

    Ok(QualifiedAuthorityTimeBasisV1 {
        expected,
        receipt,
        _qualifier: PhantomData,
    })
}

/// Untrusted absolute-time receipt. `uncertainty_ms` is a symmetric conservative
/// uncertainty radius around `observed_unix_ms`; authority checks use the latest
/// plausible time (`observed + uncertainty`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurrentTimeReceiptV1 {
    pub schema_version: u16,
    pub verifier: VerifierDescriptorV1,
    pub source: TimeSourceCommitment,
    pub observed_unix_ms: u64,
    pub uncertainty_ms: u64,
    pub valid_until: u64,
    pub receipt_commitment: CurrentTimeReceiptCommitment,
}

impl CurrentTimeReceiptV1 {
    pub const fn new(
        verifier: VerifierDescriptorV1,
        source: TimeSourceCommitment,
        observed_unix_ms: u64,
        uncertainty_ms: u64,
        valid_until: u64,
        receipt_commitment: CurrentTimeReceiptCommitment,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            verifier,
            source,
            observed_unix_ms,
            uncertainty_ms,
            valid_until,
            receipt_commitment,
        }
    }
}

/// Concrete trusted-time provider. v0.1 supports trusted absolute time with
/// uncertainty only; causal/local-monotonic profiles remain separate future
/// contracts rather than being silently converted into Unix time.
pub trait CurrentTimeQualifierV1 {
    type Error;

    fn descriptor(&self) -> VerifierDescriptorV1;

    fn qualify_current_time(&self) -> Result<CurrentTimeReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedCurrentTimeQualifierProfileV1 {
    descriptor: VerifierDescriptorV1,
}

impl ExpectedCurrentTimeQualifierProfileV1 {
    pub const fn from_trusted_configuration(descriptor: VerifierDescriptorV1) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> VerifierDescriptorV1 {
        self.descriptor
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurrentTimeBindingError<E> {
    Verifier(E),
    VerifierDescriptorMismatchBefore,
    VerifierDescriptorMismatchAfter,
    UnsupportedReceiptSchema,
    ReceiptVerifierMismatch,
    ReceiptOutlivesVerifier,
    LatestPossibleTimeOverflow,
    ReceiptAlreadyExpired,
}

/// Exact current-time evidence bound to concrete qualifier type `TQ` and an
/// independently expected verifier descriptor.
pub struct QualifiedCurrentTimeV1<TQ> {
    expected: ExpectedCurrentTimeQualifierProfileV1,
    receipt: CurrentTimeReceiptV1,
    latest_possible_unix_ms: u64,
    _qualifier: PhantomData<fn() -> TQ>,
}

impl<TQ> QualifiedCurrentTimeV1<TQ> {
    pub const fn expected_profile(&self) -> ExpectedCurrentTimeQualifierProfileV1 {
        self.expected
    }

    pub const fn source(&self) -> TimeSourceCommitment {
        self.receipt.source
    }

    pub const fn observed_unix_ms(&self) -> u64 {
        self.receipt.observed_unix_ms
    }

    pub const fn uncertainty_ms(&self) -> u64 {
        self.receipt.uncertainty_ms
    }

    pub const fn latest_possible_unix_ms(&self) -> u64 {
        self.latest_possible_unix_ms
    }

    pub const fn valid_until(&self) -> u64 {
        self.receipt.valid_until
    }

    pub const fn receipt_commitment(&self) -> CurrentTimeReceiptCommitment {
        self.receipt.receipt_commitment
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

/// Bind current-time evidence to an independently expected concrete verifier
/// generation, including before/after descriptor checks.
pub fn qualify_current_time<TQ: CurrentTimeQualifierV1>(
    expected: ExpectedCurrentTimeQualifierProfileV1,
    qualifier: &TQ,
) -> Result<QualifiedCurrentTimeV1<TQ>, CurrentTimeBindingError<TQ::Error>> {
    if qualifier.descriptor() != expected.descriptor {
        return Err(CurrentTimeBindingError::VerifierDescriptorMismatchBefore);
    }

    let receipt = qualifier
        .qualify_current_time()
        .map_err(CurrentTimeBindingError::Verifier)?;

    if qualifier.descriptor() != expected.descriptor {
        return Err(CurrentTimeBindingError::VerifierDescriptorMismatchAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(CurrentTimeBindingError::UnsupportedReceiptSchema);
    }
    if receipt.verifier != expected.descriptor {
        return Err(CurrentTimeBindingError::ReceiptVerifierMismatch);
    }
    if receipt.valid_until > expected.descriptor.valid_until {
        return Err(CurrentTimeBindingError::ReceiptOutlivesVerifier);
    }

    let latest_possible_unix_ms = receipt
        .observed_unix_ms
        .checked_add(receipt.uncertainty_ms)
        .ok_or(CurrentTimeBindingError::LatestPossibleTimeOverflow)?;
    if latest_possible_unix_ms > receipt.valid_until {
        return Err(CurrentTimeBindingError::ReceiptAlreadyExpired);
    }

    Ok(QualifiedCurrentTimeV1 {
        expected,
        receipt,
        latest_possible_unix_ms,
        _qualifier: PhantomData,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurrentAuthorityRevalidationError {
    CurrentLocalStateMismatch,
    TimeBasisReservedIssuanceMismatch,
    TimeBasisLocalStateMismatch,
    TimeBasisLocalContextReceiptMismatch,
    MembershipFreshnessRequired,
    IdentityKeyFreshnessRequired,
    RevocationFreshnessRequired,
    PolicyFreshnessRequired,
    MachineHealthFreshnessRequired,
    ScientificQualificationFreshnessRequired,
    LocalFreshnessDerivationFailed,
    AuthorityAlreadyExpired,
}

fn require_known(
    boundary: ValidityBoundary,
    error: CurrentAuthorityRevalidationError,
) -> Result<u64, CurrentAuthorityRevalidationError> {
    match boundary {
        ValidityBoundary::KnownUntil(value) => Ok(value),
        ValidityBoundary::NotApplicable | ValidityBoundary::Unknown => Err(error),
    }
}

fn required_local_valid_until(
    validity: ValidityInputsV1,
) -> Result<u64, CurrentAuthorityRevalidationError> {
    require_known(
        validity.membership,
        CurrentAuthorityRevalidationError::MembershipFreshnessRequired,
    )?;
    require_known(
        validity.identity_key,
        CurrentAuthorityRevalidationError::IdentityKeyFreshnessRequired,
    )?;
    require_known(
        validity.revocation_snapshot,
        CurrentAuthorityRevalidationError::RevocationFreshnessRequired,
    )?;
    require_known(
        validity.policy,
        CurrentAuthorityRevalidationError::PolicyFreshnessRequired,
    )?;
    require_known(
        validity.machine_health,
        CurrentAuthorityRevalidationError::MachineHealthFreshnessRequired,
    )?;
    require_known(
        validity.scientific_qualification,
        CurrentAuthorityRevalidationError::ScientificQualificationFreshnessRequired,
    )?;

    derive_valid_until(validity)
        .map_err(|_| CurrentAuthorityRevalidationError::LocalFreshnessDerivationFailed)?
        .ok_or(CurrentAuthorityRevalidationError::LocalFreshnessDerivationFailed)
}

/// Audit-facing exact current evidence retained by the revalidation token.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurrentAuthorityEvidenceV1 {
    pub local_state: FederationStateHeadV1,
    pub prerequisite_evidence: LocalPrerequisiteEvidenceV1,
    pub prerequisite_validity: ValidityInputsV1,
    pub local_context_valid_until: u64,
    pub time_basis: AuthorityTimeBasisV1,
    pub time_basis_receipt: AuthorityTimeBasisReceiptCommitment,
    pub time_basis_valid_until: u64,
    pub time_source: TimeSourceCommitment,
    pub current_time_receipt: CurrentTimeReceiptCommitment,
    pub latest_possible_unix_ms: u64,
    pub current_time_valid_until: u64,
    pub valid_until: u64,
}

/// Exact reserved issuance lineage revalidated against current local state,
/// explicit common time basis, and trusted absolute time. This remains
/// *pre-authority*: it is eligible for a later bounded lease-issuance boundary
/// but carries no install/effect power.
pub struct CurrentlyRevalidatedReservedIssuanceV1<
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
    reserved: ReboundReservedIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, L, R>,
    current_context: QualifiedCurrentLocalContextV1<CQ>,
    time_basis: QualifiedAuthorityTimeBasisV1<BQ>,
    current_time: QualifiedCurrentTimeV1<TQ>,
    evidence: CurrentAuthorityEvidenceV1,
}

impl<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, const L: usize, const R: usize>
    CurrentlyRevalidatedReservedIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>
{
    pub const fn reserved(
        &self,
    ) -> &ReboundReservedIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, L, R> {
        &self.reserved
    }

    pub const fn current_context(&self) -> &QualifiedCurrentLocalContextV1<CQ> {
        &self.current_context
    }

    pub const fn time_basis(&self) -> &QualifiedAuthorityTimeBasisV1<BQ> {
        &self.time_basis
    }

    pub const fn current_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.current_time
    }

    pub const fn evidence(&self) -> CurrentAuthorityEvidenceV1 {
        self.evidence
    }

    pub const fn valid_until(&self) -> u64 {
        self.evidence.valid_until
    }

    pub const fn contains_current_authority_revalidation(&self) -> bool {
        true
    }

    pub const fn eligible_for_bounded_authority_issuance(&self) -> bool {
        true
    }

    pub const fn contains_state_install_authority(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

/// Revalidate one exact single-use reserved lineage against current local
/// security state, an explicit shared time basis, and trusted absolute time.
pub fn revalidate_current_authority<
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
    reserved: ReboundReservedIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, L, R>,
    current_context: QualifiedCurrentLocalContextV1<CQ>,
    time_basis: QualifiedAuthorityTimeBasisV1<BQ>,
    current_time: QualifiedCurrentTimeV1<TQ>,
) -> Result<
    CurrentlyRevalidatedReservedIssuanceV1<IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, L, R>,
    CurrentAuthorityRevalidationError,
> {
    let approved_source_state = reserved
        .rebound_registration()
        .candidate()
        .source_local_state();

    if current_context.local_state() != approved_source_state {
        return Err(CurrentAuthorityRevalidationError::CurrentLocalStateMismatch);
    }
    if time_basis.reserved_issuance() != reserved.binding() {
        return Err(CurrentAuthorityRevalidationError::TimeBasisReservedIssuanceMismatch);
    }
    if time_basis.local_state() != current_context.local_state() {
        return Err(CurrentAuthorityRevalidationError::TimeBasisLocalStateMismatch);
    }
    if time_basis.local_context_receipt() != current_context.receipt_commitment() {
        return Err(CurrentAuthorityRevalidationError::TimeBasisLocalContextReceiptMismatch);
    }

    let prerequisite_validity = current_context.prerequisite_validity();
    let local_valid_until = required_local_valid_until(prerequisite_validity)?;

    let valid_until = reserved
        .authority_eligibility_valid_until()
        .min(current_context.valid_until())
        .min(local_valid_until)
        .min(time_basis.valid_until())
        .min(current_time.valid_until());

    if current_time.latest_possible_unix_ms() > valid_until {
        return Err(CurrentAuthorityRevalidationError::AuthorityAlreadyExpired);
    }

    let evidence = CurrentAuthorityEvidenceV1 {
        local_state: current_context.local_state(),
        prerequisite_evidence: current_context.prerequisite_evidence(),
        prerequisite_validity,
        local_context_valid_until: current_context.valid_until(),
        time_basis: time_basis.basis(),
        time_basis_receipt: time_basis.receipt_commitment(),
        time_basis_valid_until: time_basis.valid_until(),
        time_source: current_time.source(),
        current_time_receipt: current_time.receipt_commitment(),
        latest_possible_unix_ms: current_time.latest_possible_unix_ms(),
        current_time_valid_until: current_time.valid_until(),
        valid_until,
    };

    Ok(CurrentlyRevalidatedReservedIssuanceV1 {
        reserved,
        current_context,
        time_basis,
        current_time,
        evidence,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn all_known(value: u64) -> ValidityInputsV1 {
        ValidityInputsV1 {
            membership: ValidityBoundary::KnownUntil(value),
            identity_key: ValidityBoundary::KnownUntil(value + 1),
            revocation_snapshot: ValidityBoundary::KnownUntil(value + 2),
            policy: ValidityBoundary::KnownUntil(value + 3),
            machine_health: ValidityBoundary::KnownUntil(value + 4),
            scientific_qualification: ValidityBoundary::KnownUntil(value + 5),
        }
    }

    #[test]
    fn every_current_prerequisite_is_mandatory() {
        assert_eq!(required_local_valid_until(all_known(100)), Ok(100));

        let mut missing = all_known(100);
        missing.machine_health = ValidityBoundary::NotApplicable;
        assert_eq!(
            required_local_valid_until(missing),
            Err(CurrentAuthorityRevalidationError::MachineHealthFreshnessRequired)
        );
    }

    #[test]
    fn unknown_current_prerequisite_fails_closed() {
        let mut unknown = all_known(100);
        unknown.revocation_snapshot = ValidityBoundary::Unknown;
        assert_eq!(
            required_local_valid_until(unknown),
            Err(CurrentAuthorityRevalidationError::RevocationFreshnessRequired)
        );
    }

    #[test]
    fn uncertainty_uses_latest_plausible_time() {
        assert_eq!(1000u64.checked_add(25), Some(1025));
    }

    #[test]
    fn natural_expiry_uses_oldest_boundary() {
        let reserved = 150u64;
        let local_context = 140u64;
        let local_prerequisites = 130u64;
        let basis = 170u64;
        let time = 160u64;
        assert_eq!(
            reserved
                .min(local_context)
                .min(local_prerequisites)
                .min(basis)
                .min(time),
            130
        );
    }

    #[test]
    fn only_unix_millisecond_basis_exists_in_v1() {
        assert_eq!(
            AuthorityTimeBasisV1::UnixMillisecondsUtc,
            AuthorityTimeBasisV1::UnixMillisecondsUtc
        );
    }
}
