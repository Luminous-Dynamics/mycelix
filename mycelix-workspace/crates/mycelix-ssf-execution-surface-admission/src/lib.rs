// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact execution-surface admission candidates for SSF semantic leases.
//!
//! A use-time revalidated semantic lease still has no executable capability.
//! This crate binds that exact lease to one independently expected execution
//! generation consisting of authenticated carrier state, actuator state, an
//! exact operation carrier, and an explicit authority-time basis.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_adoption_authority_candidate::AdoptionAuthorityOperationV1;
use mycelix_ssf_authorization::VerifierDescriptorV1;
use mycelix_ssf_contracts::{ConsequenceClass, SSF_SCHEMA_V1};
use mycelix_ssf_local_adoption_policy::LocalAdoptionScopeCommitment;
use mycelix_ssf_snapshots::FederationStateHeadV1;
use mycelix_ssf_use_time_lease_revalidation::{
    UseTimeRevalidatedAuthorityLeaseV1, UseTimeRevalidatedLeaseBindingV1,
};

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

digest_type!(ExecutionSurfaceIdentityCommitment);
digest_type!(LogicalSessionIdCommitment);
digest_type!(AuthenticatedSessionTranscriptCommitment);
digest_type!(TransportAuthorityGenerationCommitment);
digest_type!(ActuatorPolicyGenerationCommitment);
digest_type!(ActuatorHealthGenerationCommitment);
digest_type!(OperationCarrierCommitment);
digest_type!(ExecutionSurfaceAdmissionReceiptCommitment);

/// Exact authenticated session generation.
///
/// Equal logical session identity does not imply equal authority generation;
/// the authenticated transcript commitment is part of equality.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct AuthenticatedSessionGenerationV1 {
    pub logical_session: LogicalSessionIdCommitment,
    pub transcript: AuthenticatedSessionTranscriptCommitment,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct AuthenticatedCarrierGenerationV1 {
    pub session: AuthenticatedSessionGenerationV1,
    pub transport_authority: TransportAuthorityGenerationCommitment,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ActuatorGenerationV1 {
    pub policy: ActuatorPolicyGenerationCommitment,
    pub health: ActuatorHealthGenerationCommitment,
}

/// Closed v0.1 time basis for execution-generation freshness.
///
/// `valid_until` must never be compared with trusted absolute time unless this
/// basis has been explicitly supplied by the authenticated execution adapter.
/// Future clock domains require a new explicit variant and qualification rule;
/// they must not be silently coerced into Unix milliseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ExecutionGenerationTimeBasisV1 {
    UnixMillisecondsUtc,
}

/// Provider-neutral exact execution generation.
///
/// A Xenia adapter may prove the authenticated carrier fields from its exact
/// transcript/transport typestate. A Sovereign Ops adapter may prove actuator
/// policy/health generation. SSF deliberately does not depend on either crate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ExecutionSurfaceGenerationV1 {
    pub surface_identity: ExecutionSurfaceIdentityCommitment,
    pub carrier: AuthenticatedCarrierGenerationV1,
    pub actuator: ActuatorGenerationV1,
    pub operation_carrier: OperationCarrierCommitment,
    pub time_basis: ExecutionGenerationTimeBasisV1,
    pub valid_until: u64,
}

/// Independently supplied expected execution generation plus the verifier
/// generation trusted to attest its exact binding to the SSF lease subject.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedExecutionSurfaceProfileV1 {
    verifier: VerifierDescriptorV1,
    generation: ExecutionSurfaceGenerationV1,
}

impl ExpectedExecutionSurfaceProfileV1 {
    /// Construct from an already-authenticated execution-surface adapter.
    ///
    /// This pure crate does not itself prove the provenance of `generation`;
    /// that remains the Xenia/Sovereign adapter boundary.
    pub const fn from_authenticated_execution_surface(
        verifier: VerifierDescriptorV1,
        generation: ExecutionSurfaceGenerationV1,
    ) -> Self {
        Self {
            verifier,
            generation,
        }
    }

    pub const fn verifier(&self) -> VerifierDescriptorV1 {
        self.verifier
    }

    pub const fn generation(&self) -> ExecutionSurfaceGenerationV1 {
        self.generation
    }
}

/// Plain exact subject the execution-surface verifier must rebind.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExecutionSurfaceAdmissionSubjectV1 {
    pub use_time_lease: UseTimeRevalidatedLeaseBindingV1,
    pub operation: AdoptionAuthorityOperationV1,
    pub source_local_state: FederationStateHeadV1,
    pub remote_target: FederationStateHeadV1,
    pub scope: LocalAdoptionScopeCommitment,
    pub consequence: ConsequenceClass,
    pub lease_valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExecutionSurfaceAdmissionReceiptV1 {
    pub schema_version: u16,
    pub verifier: VerifierDescriptorV1,
    pub subject: ExecutionSurfaceAdmissionSubjectV1,
    pub generation: ExecutionSurfaceGenerationV1,
    pub valid_until: u64,
    pub receipt_commitment: ExecutionSurfaceAdmissionReceiptCommitment,
}

impl ExecutionSurfaceAdmissionReceiptV1 {
    pub const fn new(
        verifier: VerifierDescriptorV1,
        subject: ExecutionSurfaceAdmissionSubjectV1,
        generation: ExecutionSurfaceGenerationV1,
        valid_until: u64,
        receipt_commitment: ExecutionSurfaceAdmissionReceiptCommitment,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            verifier,
            subject,
            generation,
            valid_until,
            receipt_commitment,
        }
    }
}

pub trait ExecutionSurfaceAdmissionQualifierV1 {
    type Error;

    fn descriptor(&self) -> VerifierDescriptorV1;

    fn qualify_execution_surface_admission(
        &self,
        subject: &ExecutionSurfaceAdmissionSubjectV1,
        expected_generation: &ExecutionSurfaceGenerationV1,
    ) -> Result<ExecutionSurfaceAdmissionReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExecutionSurfaceAdmissionError<E> {
    UnsupportedExecutionGenerationTimeBasis,
    VerifierDescriptorMismatchBefore,
    Verifier(E),
    VerifierDescriptorMismatchAfter,
    UnsupportedReceiptSchema,
    ReceiptVerifierMismatch,
    SubjectMismatch,
    ExecutionGenerationMismatch,
    ReceiptOutlivesVerifier,
    ReceiptOutlivesExecutionGeneration,
    ReceiptOutlivesUseTimeLease,
    ExecutionGenerationAlreadyExpired,
    ReceiptAlreadyExpired,
}

fn derive_admission_valid_until(
    lease_valid_until: u64,
    verifier_valid_until: u64,
    execution_generation_valid_until: u64,
    receipt_valid_until: u64,
    latest_possible_now: u64,
) -> Result<u64, ExecutionSurfaceAdmissionError<core::convert::Infallible>> {
    if execution_generation_valid_until < latest_possible_now {
        return Err(ExecutionSurfaceAdmissionError::ExecutionGenerationAlreadyExpired);
    }
    if receipt_valid_until < latest_possible_now {
        return Err(ExecutionSurfaceAdmissionError::ReceiptAlreadyExpired);
    }

    Ok(lease_valid_until
        .min(verifier_valid_until)
        .min(execution_generation_valid_until)
        .min(receipt_valid_until))
}

pub struct ExecutionSurfaceAdmissionCandidateV1<
    Q,
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
    lease: UseTimeRevalidatedAuthorityLeaseV1<
        S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, UCQ, UBQ, UTQ, L, R,
    >,
    expected: ExpectedExecutionSurfaceProfileV1,
    receipt: ExecutionSurfaceAdmissionReceiptV1,
    valid_until: u64,
    _qualifier: PhantomData<Q>,
}

impl<
        Q,
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
    > ExecutionSurfaceAdmissionCandidateV1<
        Q, S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, UCQ, UBQ, UTQ, L, R,
    >
{
    pub const fn lease(
        &self,
    ) -> &UseTimeRevalidatedAuthorityLeaseV1<
        S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, UCQ, UBQ, UTQ, L, R,
    > {
        &self.lease
    }

    pub const fn expected(&self) -> ExpectedExecutionSurfaceProfileV1 {
        self.expected
    }

    pub const fn receipt(&self) -> ExecutionSurfaceAdmissionReceiptV1 {
        self.receipt
    }

    pub const fn generation(&self) -> ExecutionSurfaceGenerationV1 {
        self.expected.generation
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn exact_execution_generation_bound(&self) -> bool {
        true
    }

    pub const fn execution_generation_time_basis_explicit(&self) -> bool {
        true
    }

    pub const fn eligible_for_durable_effect_admission(&self) -> bool {
        true
    }

    pub const fn contains_single_use_effect_capability(&self) -> bool {
        false
    }

    pub const fn contains_state_install_authority(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

pub fn qualify_execution_surface_admission<
    Q,
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
    lease: UseTimeRevalidatedAuthorityLeaseV1<
        S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, UCQ, UBQ, UTQ, L, R,
    >,
    expected: ExpectedExecutionSurfaceProfileV1,
    qualifier: &Q,
) -> Result<
    ExecutionSurfaceAdmissionCandidateV1<
        Q, S, IS, RS, P, EV, RV, AQ, LQ, CQ, BQ, TQ, UCQ, UBQ, UTQ, L, R,
    >,
    ExecutionSurfaceAdmissionError<Q::Error>,
>
where
    Q: ExecutionSurfaceAdmissionQualifierV1,
{
    if expected.generation.time_basis != ExecutionGenerationTimeBasisV1::UnixMillisecondsUtc {
        return Err(ExecutionSurfaceAdmissionError::UnsupportedExecutionGenerationTimeBasis);
    }
    if qualifier.descriptor() != expected.verifier {
        return Err(ExecutionSurfaceAdmissionError::VerifierDescriptorMismatchBefore);
    }

    let bounded_candidate = lease.lease().candidate();
    let subject = ExecutionSurfaceAdmissionSubjectV1 {
        use_time_lease: lease.binding(),
        operation: bounded_candidate.operation(),
        source_local_state: bounded_candidate.source_local_state(),
        remote_target: bounded_candidate.remote_target(),
        scope: bounded_candidate.scope(),
        consequence: bounded_candidate.consequence(),
        lease_valid_until: lease.valid_until(),
    };

    let receipt = qualifier
        .qualify_execution_surface_admission(&subject, &expected.generation)
        .map_err(ExecutionSurfaceAdmissionError::Verifier)?;

    if qualifier.descriptor() != expected.verifier {
        return Err(ExecutionSurfaceAdmissionError::VerifierDescriptorMismatchAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(ExecutionSurfaceAdmissionError::UnsupportedReceiptSchema);
    }
    if receipt.verifier != expected.verifier {
        return Err(ExecutionSurfaceAdmissionError::ReceiptVerifierMismatch);
    }
    if receipt.subject != subject {
        return Err(ExecutionSurfaceAdmissionError::SubjectMismatch);
    }
    if receipt.generation != expected.generation {
        return Err(ExecutionSurfaceAdmissionError::ExecutionGenerationMismatch);
    }
    if receipt.valid_until > expected.verifier.valid_until {
        return Err(ExecutionSurfaceAdmissionError::ReceiptOutlivesVerifier);
    }
    if receipt.valid_until > expected.generation.valid_until {
        return Err(ExecutionSurfaceAdmissionError::ReceiptOutlivesExecutionGeneration);
    }
    if receipt.valid_until > lease.valid_until() {
        return Err(ExecutionSurfaceAdmissionError::ReceiptOutlivesUseTimeLease);
    }

    let valid_until = derive_admission_valid_until(
        lease.valid_until(),
        expected.verifier.valid_until,
        expected.generation.valid_until,
        receipt.valid_until,
        lease.evidence().latest_possible_unix_ms,
    )
    .map_err(|error| match error {
        ExecutionSurfaceAdmissionError::ExecutionGenerationAlreadyExpired => {
            ExecutionSurfaceAdmissionError::ExecutionGenerationAlreadyExpired
        }
        ExecutionSurfaceAdmissionError::ReceiptAlreadyExpired => {
            ExecutionSurfaceAdmissionError::ReceiptAlreadyExpired
        }
        _ => unreachable!(),
    })?;

    Ok(ExecutionSurfaceAdmissionCandidateV1 {
        lease,
        expected,
        receipt,
        valid_until,
        _qualifier: PhantomData,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn session(logical: u8, transcript: u8) -> AuthenticatedSessionGenerationV1 {
        AuthenticatedSessionGenerationV1 {
            logical_session: LogicalSessionIdCommitment::from_bytes(bytes(logical)),
            transcript: AuthenticatedSessionTranscriptCommitment::from_bytes(bytes(transcript)),
        }
    }

    fn generation(policy: u8, valid_until: u64) -> ExecutionSurfaceGenerationV1 {
        ExecutionSurfaceGenerationV1 {
            surface_identity: ExecutionSurfaceIdentityCommitment::from_bytes(bytes(1)),
            carrier: AuthenticatedCarrierGenerationV1 {
                session: session(2, 3),
                transport_authority: TransportAuthorityGenerationCommitment::from_bytes(bytes(4)),
            },
            actuator: ActuatorGenerationV1 {
                policy: ActuatorPolicyGenerationCommitment::from_bytes(bytes(policy)),
                health: ActuatorHealthGenerationCommitment::from_bytes(bytes(6)),
            },
            operation_carrier: OperationCarrierCommitment::from_bytes(bytes(7)),
            time_basis: ExecutionGenerationTimeBasisV1::UnixMillisecondsUtc,
            valid_until,
        }
    }

    #[test]
    fn equal_logical_session_with_different_transcript_is_different_generation() {
        assert_ne!(session(1, 2), session(1, 3));
    }

    #[test]
    fn execution_generation_changes_when_actuator_policy_changes() {
        assert_ne!(generation(5, 200), generation(8, 200));
    }

    #[test]
    fn execution_generation_carries_explicit_time_basis() {
        assert_eq!(
            generation(5, 200).time_basis,
            ExecutionGenerationTimeBasisV1::UnixMillisecondsUtc
        );
    }

    #[test]
    fn admission_natural_expiry_uses_oldest_boundary() {
        assert_eq!(
            derive_admission_valid_until(180, 170, 160, 150, 100),
            Ok(150)
        );
    }

    #[test]
    fn expired_execution_generation_fails_closed() {
        assert_eq!(
            derive_admission_valid_until(180, 170, 99, 150, 100),
            Err(ExecutionSurfaceAdmissionError::ExecutionGenerationAlreadyExpired)
        );
    }

    #[test]
    fn expired_receipt_fails_closed() {
        assert_eq!(
            derive_admission_valid_until(180, 170, 160, 99, 100),
            Err(ExecutionSurfaceAdmissionError::ReceiptAlreadyExpired)
        );
    }
}
