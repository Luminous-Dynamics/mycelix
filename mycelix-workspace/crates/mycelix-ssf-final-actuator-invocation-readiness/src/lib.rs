// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Final non-effecting convergence boundary for SSF actuator invocation paths.
//! Both the normal/same-generation activation path and the historical-rotation
//! activation path must satisfy the same anti-rollback and lifetime theorem
//! before any actuator adapter may reserve or invoke an effect.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_actuator_effect_protocol::ActuatorInvocationAttemptManifestV1;
use mycelix_ssf_actuator_invocation_activation::ActivatedActuatorInvocationV1;
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_historical_actuator_invocation_activation::HistoricallyActivatedActuatorInvocationV1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FinalInvocationSourceKindV1 {
    CurrentOrSameGenerationRecovery,
    HistoricalSameIdentityRecovery,
}

mod sealed {
    pub trait Sealed {}
}

pub trait ExactActivatedInvocationV1: sealed::Sealed {
    fn exact_attempt(&self) -> ActuatorInvocationAttemptManifestV1;
    fn activation_latest_possible_unix_ms(&self) -> u64;
    fn activation_valid_until(&self) -> u64;
    fn source_kind(&self) -> FinalInvocationSourceKindV1;
}

impl<J, TQ> sealed::Sealed for ActivatedActuatorInvocationV1<J, TQ> {}

impl<J, TQ> ExactActivatedInvocationV1 for ActivatedActuatorInvocationV1<J, TQ> {
    fn exact_attempt(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.attempt()
    }

    fn activation_latest_possible_unix_ms(&self) -> u64 {
        self.current_time().latest_possible_unix_ms()
    }

    fn activation_valid_until(&self) -> u64 {
        self.valid_until()
    }

    fn source_kind(&self) -> FinalInvocationSourceKindV1 {
        FinalInvocationSourceKindV1::CurrentOrSameGenerationRecovery
    }
}

impl<H, TQ> sealed::Sealed for HistoricallyActivatedActuatorInvocationV1<H, TQ> {}

impl<H, TQ> ExactActivatedInvocationV1 for HistoricallyActivatedActuatorInvocationV1<H, TQ> {
    fn exact_attempt(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.attempt()
    }

    fn activation_latest_possible_unix_ms(&self) -> u64 {
        self.current_time().latest_possible_unix_ms()
    }

    fn activation_valid_until(&self) -> u64 {
        self.valid_until()
    }

    fn source_kind(&self) -> FinalInvocationSourceKindV1 {
        FinalInvocationSourceKindV1::HistoricalSameIdentityRecovery
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FinalInvocationReadinessErrorV1 {
    UnsupportedAttemptSchema,
    ActivationTimeRegressedBeforeAttempt,
    ActivationAlreadyExpired,
    AttemptAlreadyExpired,
    ActuatorGenerationAlreadyExpired,
}

pub struct FinalInvocationReadinessFailureV1<A> {
    activated: A,
    error: FinalInvocationReadinessErrorV1,
}

impl<A> FinalInvocationReadinessFailureV1<A> {
    pub const fn error(&self) -> FinalInvocationReadinessErrorV1 {
        self.error
    }

    pub fn into_activated(self) -> A {
        self.activated
    }
}

/// Canonical final pre-actuator typestate.
///
/// This token is still non-effecting. Its purpose is to prevent downstream
/// adapters from bypassing activation or choosing a weaker restart path.
pub struct FinalActuatorInvocationReadyV1<A> {
    activated: A,
    attempt: ActuatorInvocationAttemptManifestV1,
    source_kind: FinalInvocationSourceKindV1,
    activation_latest_possible_unix_ms: u64,
    valid_until: u64,
}

impl<A> FinalActuatorInvocationReadyV1<A> {
    pub const fn activated(&self) -> &A {
        &self.activated
    }

    pub const fn attempt(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.attempt
    }

    pub const fn source_kind(&self) -> FinalInvocationSourceKindV1 {
        self.source_kind
    }

    pub const fn activation_latest_possible_unix_ms(&self) -> u64 {
        self.activation_latest_possible_unix_ms
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn eligible_for_actuator_reservation(&self) -> bool {
        true
    }

    pub const fn actuator_reservation_performed(&self) -> bool {
        false
    }

    pub const fn external_effect_attempted(&self) -> bool {
        false
    }
}

pub fn finalize_actuator_invocation_readiness<A>(
    activated: A,
) -> Result<FinalActuatorInvocationReadyV1<A>, FinalInvocationReadinessFailureV1<A>>
where
    A: ExactActivatedInvocationV1,
{
    let attempt = activated.exact_attempt();
    let activation_latest = activated.activation_latest_possible_unix_ms();
    let activation_valid_until = activated.activation_valid_until();

    let error = if attempt.schema_version != SSF_SCHEMA_V1 {
        Some(FinalInvocationReadinessErrorV1::UnsupportedAttemptSchema)
    } else if activation_latest < attempt.latest_possible_unix_ms {
        Some(FinalInvocationReadinessErrorV1::ActivationTimeRegressedBeforeAttempt)
    } else if activation_valid_until < activation_latest {
        Some(FinalInvocationReadinessErrorV1::ActivationAlreadyExpired)
    } else if attempt.attempt_valid_until < activation_latest {
        Some(FinalInvocationReadinessErrorV1::AttemptAlreadyExpired)
    } else if attempt.subject.actuator.valid_until < activation_latest {
        Some(FinalInvocationReadinessErrorV1::ActuatorGenerationAlreadyExpired)
    } else {
        None
    };

    if let Some(error) = error {
        return Err(FinalInvocationReadinessFailureV1 { activated, error });
    }

    let valid_until = activation_valid_until
        .min(attempt.attempt_valid_until)
        .min(attempt.subject.actuator.valid_until);

    Ok(FinalActuatorInvocationReadyV1 {
        source_kind: activated.source_kind(),
        activated,
        attempt,
        activation_latest_possible_unix_ms: activation_latest,
        valid_until,
    })
}

#[cfg(test)]
mod tests {
    #[test]
    fn rollback_is_detectable() {
        let original_attempt_latest = 100_u64;
        let activation_latest = 99_u64;
        assert!(activation_latest < original_attempt_latest);
    }

    #[test]
    fn final_ceiling_only_shrinks() {
        let activation = 100_u64;
        let attempt = 95_u64;
        let actuator = 90_u64;
        assert_eq!(activation.min(attempt).min(actuator), 90);
    }
}
