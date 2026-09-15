use mycelix_business_core::{ExecutionAttemptRef, ReferenceId};
use sha2::{Digest, Sha256};

use crate::descriptor::{
    push_reference, push_u16, push_u64, push_u8, reservation_commitment, ReservationCommitment,
    ReservationDescriptor, ReservationDescriptorError,
};

pub const RESERVATION_STATE_COMMITMENT_PROFILE_REVISION: u16 = 1;
pub const RESERVATION_STATE_DIGEST_PROFILE: &str = "sha256";
const STATE_DOMAIN: &[u8] = b"MYCELIX_FINANCE_RESERVATION_STATE_V1\0";

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReservationLifecycle {
    Active,
    Consumed {
        attempt: ExecutionAttemptRef,
        idempotency_key: ReferenceId,
        consumed_at_unix_ms: u64,
    },
    Released {
        evidence: ReferenceId,
        released_at_unix_ms: u64,
    },
    Revoked {
        evidence: ReferenceId,
        revoked_at_unix_ms: u64,
    },
    Expired {
        expired_at_unix_ms: u64,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ReservationStateCommitment([u8; 32]);

impl ReservationStateCommitment {
    pub fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }

    pub fn into_bytes(self) -> [u8; 32] {
        self.0
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReservationState {
    descriptor_commitment: ReservationCommitment,
    sequence: u64,
    lifecycle: ReservationLifecycle,
    state_commitment: ReservationStateCommitment,
}

impl ReservationState {
    pub fn initial(
        descriptor: &ReservationDescriptor,
    ) -> Result<Self, ReservationLifecycleError> {
        let descriptor_commitment = reservation_commitment(descriptor)
            .map_err(ReservationLifecycleError::Descriptor)?;
        let sequence = 1;
        let lifecycle = ReservationLifecycle::Active;
        let state_commitment = compute_state_commitment(
            descriptor_commitment,
            sequence,
            &lifecycle,
        )?;

        Ok(Self {
            descriptor_commitment,
            sequence,
            lifecycle,
            state_commitment,
        })
    }

    /// Reconstruct one persisted lifecycle claim only after recomputing and
    /// validating every commitment/time/sequence invariant owned by v1.
    ///
    /// This is intentionally stricter than a public field constructor. Runtime
    /// adapters can deserialize wire/DHT state into ordinary values, then call
    /// this function to obtain a sealed semantic `ReservationState` only when
    /// the stored state commitment is authentic *with respect to the supplied
    /// descriptor bytes*. It does not authenticate who authored the record;
    /// FIN-ECO-004B owns that provenance check.
    pub fn verified_from_claimed(
        descriptor: &ReservationDescriptor,
        sequence: u64,
        lifecycle: ReservationLifecycle,
        claimed_state_commitment: [u8; 32],
    ) -> Result<Self, ReservationLifecycleError> {
        descriptor
            .validate()
            .map_err(ReservationLifecycleError::Descriptor)?;
        validate_claimed_lifecycle(descriptor, sequence, &lifecycle)?;

        let descriptor_commitment = reservation_commitment(descriptor)
            .map_err(ReservationLifecycleError::Descriptor)?;
        let computed = compute_state_commitment(
            descriptor_commitment,
            sequence,
            &lifecycle,
        )?;
        if computed.0 != claimed_state_commitment {
            return Err(ReservationLifecycleError::StateCommitmentMismatch);
        }

        Ok(Self {
            descriptor_commitment,
            sequence,
            lifecycle,
            state_commitment: computed,
        })
    }

    pub fn descriptor_commitment(&self) -> ReservationCommitment {
        self.descriptor_commitment
    }

    pub fn sequence(&self) -> u64 {
        self.sequence
    }

    pub fn lifecycle(&self) -> &ReservationLifecycle {
        &self.lifecycle
    }

    pub fn state_commitment(&self) -> ReservationStateCommitment {
        self.state_commitment
    }

    pub fn expected(&self) -> ExpectedReservationState {
        ExpectedReservationState::new(self.sequence, self.state_commitment)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedReservationState {
    sequence: u64,
    state_commitment: ReservationStateCommitment,
}

impl ExpectedReservationState {
    pub fn new(sequence: u64, state_commitment: ReservationStateCommitment) -> Self {
        Self {
            sequence,
            state_commitment,
        }
    }

    pub fn sequence(&self) -> u64 {
        self.sequence
    }

    pub fn state_commitment(&self) -> ReservationStateCommitment {
        self.state_commitment
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReservationTransition {
    Consume {
        attempt: ExecutionAttemptRef,
        idempotency_key: ReferenceId,
        at_unix_ms: u64,
    },
    Release {
        evidence: ReferenceId,
        at_unix_ms: u64,
    },
    Revoke {
        evidence: ReferenceId,
        at_unix_ms: u64,
    },
    Expire {
        at_unix_ms: u64,
    },
}

impl ReservationTransition {
    fn at_unix_ms(&self) -> u64 {
        match self {
            Self::Consume { at_unix_ms, .. }
            | Self::Release { at_unix_ms, .. }
            | Self::Revoke { at_unix_ms, .. }
            | Self::Expire { at_unix_ms } => *at_unix_ms,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReservationTransitionOutcome {
    Applied(ReservationState),
    IdempotentReplay(ReservationState),
}

impl ReservationTransitionOutcome {
    pub fn state(&self) -> &ReservationState {
        match self {
            Self::Applied(state) | Self::IdempotentReplay(state) => state,
        }
    }

    pub fn into_state(self) -> ReservationState {
        match self {
            Self::Applied(state) | Self::IdempotentReplay(state) => state,
        }
    }

    pub fn is_idempotent_replay(&self) -> bool {
        matches!(self, Self::IdempotentReplay(_))
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReservationLifecycleError {
    Descriptor(ReservationDescriptorError),
    DescriptorCommitmentMismatch,
    InvalidSequenceForLifecycle { expected: u64, actual: u64 },
    StateCommitmentMismatch,
    StaleSequence { expected: u64, actual: u64 },
    StaleStateCommitment,
    SequenceOverflow,
    TransitionBeforeIssuance,
    TransitionAfterExpiry,
    EarlyExpiry,
    ReplayBeforeOriginalConsumption,
    AlreadyConsumed,
    TerminalState,
}

pub fn apply_reservation_transition(
    descriptor: &ReservationDescriptor,
    current: &ReservationState,
    expected: ExpectedReservationState,
    transition: ReservationTransition,
) -> Result<ReservationTransitionOutcome, ReservationLifecycleError> {
    descriptor
        .validate()
        .map_err(ReservationLifecycleError::Descriptor)?;

    let actual_descriptor_commitment = reservation_commitment(descriptor)
        .map_err(ReservationLifecycleError::Descriptor)?;
    if actual_descriptor_commitment != current.descriptor_commitment {
        return Err(ReservationLifecycleError::DescriptorCommitmentMismatch);
    }
    if expected.sequence != current.sequence {
        return Err(ReservationLifecycleError::StaleSequence {
            expected: expected.sequence,
            actual: current.sequence,
        });
    }
    if expected.state_commitment != current.state_commitment {
        return Err(ReservationLifecycleError::StaleStateCommitment);
    }

    if let ReservationLifecycle::Consumed {
        attempt: prior_attempt,
        idempotency_key: prior_idempotency_key,
        consumed_at_unix_ms,
    } = &current.lifecycle
    {
        if let ReservationTransition::Consume {
            attempt,
            idempotency_key,
            at_unix_ms,
        } = &transition
        {
            if attempt == prior_attempt && idempotency_key == prior_idempotency_key {
                if *at_unix_ms < *consumed_at_unix_ms {
                    return Err(ReservationLifecycleError::ReplayBeforeOriginalConsumption);
                }
                return Ok(ReservationTransitionOutcome::IdempotentReplay(
                    current.clone(),
                ));
            }
        }
        return Err(ReservationLifecycleError::AlreadyConsumed);
    }

    if !matches!(&current.lifecycle, ReservationLifecycle::Active) {
        return Err(ReservationLifecycleError::TerminalState);
    }

    let at = transition.at_unix_ms();
    if at < descriptor.issued_at_unix_ms {
        return Err(ReservationLifecycleError::TransitionBeforeIssuance);
    }

    let next_lifecycle = match transition {
        ReservationTransition::Consume {
            attempt,
            idempotency_key,
            at_unix_ms,
        } => {
            require_before_expiry(descriptor, at_unix_ms)?;
            ReservationLifecycle::Consumed {
                attempt,
                idempotency_key,
                consumed_at_unix_ms: at_unix_ms,
            }
        }
        ReservationTransition::Release {
            evidence,
            at_unix_ms,
        } => {
            require_before_expiry(descriptor, at_unix_ms)?;
            ReservationLifecycle::Released {
                evidence,
                released_at_unix_ms: at_unix_ms,
            }
        }
        ReservationTransition::Revoke {
            evidence,
            at_unix_ms,
        } => {
            require_before_expiry(descriptor, at_unix_ms)?;
            ReservationLifecycle::Revoked {
                evidence,
                revoked_at_unix_ms: at_unix_ms,
            }
        }
        ReservationTransition::Expire { at_unix_ms } => {
            if at_unix_ms < descriptor.expires_at_unix_ms {
                return Err(ReservationLifecycleError::EarlyExpiry);
            }
            ReservationLifecycle::Expired {
                expired_at_unix_ms: at_unix_ms,
            }
        }
    };

    let sequence = current
        .sequence
        .checked_add(1)
        .ok_or(ReservationLifecycleError::SequenceOverflow)?;
    let state_commitment = compute_state_commitment(
        current.descriptor_commitment,
        sequence,
        &next_lifecycle,
    )?;

    Ok(ReservationTransitionOutcome::Applied(ReservationState {
        descriptor_commitment: current.descriptor_commitment,
        sequence,
        lifecycle: next_lifecycle,
        state_commitment,
    }))
}

fn validate_claimed_lifecycle(
    descriptor: &ReservationDescriptor,
    sequence: u64,
    lifecycle: &ReservationLifecycle,
) -> Result<(), ReservationLifecycleError> {
    let expected_sequence = match lifecycle {
        ReservationLifecycle::Active => 1,
        ReservationLifecycle::Consumed { .. }
        | ReservationLifecycle::Released { .. }
        | ReservationLifecycle::Revoked { .. }
        | ReservationLifecycle::Expired { .. } => 2,
    };
    if sequence != expected_sequence {
        return Err(ReservationLifecycleError::InvalidSequenceForLifecycle {
            expected: expected_sequence,
            actual: sequence,
        });
    }

    match lifecycle {
        ReservationLifecycle::Active => Ok(()),
        ReservationLifecycle::Consumed {
            consumed_at_unix_ms,
            ..
        } => validate_pre_expiry_time(descriptor, *consumed_at_unix_ms),
        ReservationLifecycle::Released {
            released_at_unix_ms,
            ..
        } => validate_pre_expiry_time(descriptor, *released_at_unix_ms),
        ReservationLifecycle::Revoked {
            revoked_at_unix_ms,
            ..
        } => validate_pre_expiry_time(descriptor, *revoked_at_unix_ms),
        ReservationLifecycle::Expired { expired_at_unix_ms } => {
            if *expired_at_unix_ms < descriptor.expires_at_unix_ms {
                Err(ReservationLifecycleError::EarlyExpiry)
            } else {
                Ok(())
            }
        }
    }
}

fn validate_pre_expiry_time(
    descriptor: &ReservationDescriptor,
    at_unix_ms: u64,
) -> Result<(), ReservationLifecycleError> {
    if at_unix_ms < descriptor.issued_at_unix_ms {
        Err(ReservationLifecycleError::TransitionBeforeIssuance)
    } else {
        require_before_expiry(descriptor, at_unix_ms)
    }
}

fn require_before_expiry(
    descriptor: &ReservationDescriptor,
    at_unix_ms: u64,
) -> Result<(), ReservationLifecycleError> {
    if at_unix_ms >= descriptor.expires_at_unix_ms {
        Err(ReservationLifecycleError::TransitionAfterExpiry)
    } else {
        Ok(())
    }
}

fn compute_state_commitment(
    descriptor_commitment: ReservationCommitment,
    sequence: u64,
    lifecycle: &ReservationLifecycle,
) -> Result<ReservationStateCommitment, ReservationLifecycleError> {
    let mut bytes = Vec::with_capacity(256);
    bytes.extend_from_slice(STATE_DOMAIN);
    push_u16(&mut bytes, RESERVATION_STATE_COMMITMENT_PROFILE_REVISION);
    bytes.extend_from_slice(descriptor_commitment.as_bytes());
    push_u64(&mut bytes, sequence);

    match lifecycle {
        ReservationLifecycle::Active => {
            push_u8(&mut bytes, 0);
        }
        ReservationLifecycle::Consumed {
            attempt,
            idempotency_key,
            consumed_at_unix_ms,
        } => {
            push_u8(&mut bytes, 1);
            push_reference(&mut bytes, &attempt.0)
                .map_err(ReservationLifecycleError::Descriptor)?;
            push_reference(&mut bytes, idempotency_key)
                .map_err(ReservationLifecycleError::Descriptor)?;
            push_u64(&mut bytes, *consumed_at_unix_ms);
        }
        ReservationLifecycle::Released {
            evidence,
            released_at_unix_ms,
        } => {
            push_u8(&mut bytes, 2);
            push_reference(&mut bytes, evidence)
                .map_err(ReservationLifecycleError::Descriptor)?;
            push_u64(&mut bytes, *released_at_unix_ms);
        }
        ReservationLifecycle::Revoked {
            evidence,
            revoked_at_unix_ms,
        } => {
            push_u8(&mut bytes, 3);
            push_reference(&mut bytes, evidence)
                .map_err(ReservationLifecycleError::Descriptor)?;
            push_u64(&mut bytes, *revoked_at_unix_ms);
        }
        ReservationLifecycle::Expired { expired_at_unix_ms } => {
            push_u8(&mut bytes, 4);
            push_u64(&mut bytes, *expired_at_unix_ms);
        }
    }

    let digest = Sha256::digest(bytes);
    let mut out = [0_u8; 32];
    out.copy_from_slice(&digest);
    Ok(ReservationStateCommitment(out))
}
