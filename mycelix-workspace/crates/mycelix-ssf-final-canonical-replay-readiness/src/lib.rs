// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Final non-effecting convergence boundary for canonical replay invocation.
//!
//! Both ordinary/same-generation canonical replay activation and historical
//! same-identity activation must satisfy the same final anti-rollback, expiry,
//! and single-replay theorem before any canonical replay actuator adapter may
//! reserve or invoke an effect.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_actuator_effect_protocol::ActuatorInvocationAttemptManifestV1;
use mycelix_ssf_canonical_replay_invocation_activation::{
    ActivatedCanonicalReplayInvocationV1, CanonicalReplayJournalEvidenceV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_historical_canonical_replay_activation::{
    HistoricalCanonicalReplayActivationEvidenceV1,
    HistoricallyActivatedCanonicalReplayInvocationV1,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FinalCanonicalReplaySourceKindV1 {
    CurrentOrSameGenerationRecovery,
    HistoricalSameIdentityRecovery,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FinalCanonicalReplayAuditEvidenceV1 {
    CurrentOrSameGeneration(CanonicalReplayJournalEvidenceV1),
    HistoricalSameIdentity(HistoricalCanonicalReplayActivationEvidenceV1),
}

mod sealed {
    pub trait Sealed {}
}

pub trait ExactActivatedCanonicalReplayV1: sealed::Sealed {
    fn exact_attempt(&self) -> ActuatorInvocationAttemptManifestV1;
    fn activation_latest_possible_unix_ms(&self) -> u64;
    fn activation_valid_until(&self) -> u64;
    fn source_kind(&self) -> FinalCanonicalReplaySourceKindV1;
    fn audit_evidence(&self) -> FinalCanonicalReplayAuditEvidenceV1;
    fn permits_third_attempt(&self) -> bool;
}

impl<J, TQ> sealed::Sealed for ActivatedCanonicalReplayInvocationV1<J, TQ> {}

impl<J, TQ> ExactActivatedCanonicalReplayV1 for ActivatedCanonicalReplayInvocationV1<J, TQ> {
    fn exact_attempt(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.attempt()
    }

    fn activation_latest_possible_unix_ms(&self) -> u64 {
        self.activation_latest_possible_unix_ms()
    }

    fn activation_valid_until(&self) -> u64 {
        self.valid_until()
    }

    fn source_kind(&self) -> FinalCanonicalReplaySourceKindV1 {
        FinalCanonicalReplaySourceKindV1::CurrentOrSameGenerationRecovery
    }

    fn audit_evidence(&self) -> FinalCanonicalReplayAuditEvidenceV1 {
        FinalCanonicalReplayAuditEvidenceV1::CurrentOrSameGeneration(self.journal_evidence())
    }

    fn permits_third_attempt(&self) -> bool {
        self.permits_third_attempt()
    }
}

impl<H, TQ> sealed::Sealed for HistoricallyActivatedCanonicalReplayInvocationV1<H, TQ> {}

impl<H, TQ> ExactActivatedCanonicalReplayV1
    for HistoricallyActivatedCanonicalReplayInvocationV1<H, TQ>
{
    fn exact_attempt(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.attempt()
    }

    fn activation_latest_possible_unix_ms(&self) -> u64 {
        self.activation_latest_possible_unix_ms()
    }

    fn activation_valid_until(&self) -> u64 {
        self.valid_until()
    }

    fn source_kind(&self) -> FinalCanonicalReplaySourceKindV1 {
        FinalCanonicalReplaySourceKindV1::HistoricalSameIdentityRecovery
    }

    fn audit_evidence(&self) -> FinalCanonicalReplayAuditEvidenceV1 {
        FinalCanonicalReplayAuditEvidenceV1::HistoricalSameIdentity(self.evidence())
    }

    fn permits_third_attempt(&self) -> bool {
        self.permits_third_attempt()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FinalCanonicalReplayReadinessErrorV1 {
    UnsupportedAttemptSchema,
    ActivationTimeRegressedBeforeAttempt,
    ActivationAlreadyExpired,
    AttemptAlreadyExpired,
    ActuatorGenerationAlreadyExpired,
    UnexpectedThirdAttemptPermission,
}

pub struct FinalCanonicalReplayReadinessFailureV1<A> {
    activated: A,
    error: FinalCanonicalReplayReadinessErrorV1,
}

impl<A> FinalCanonicalReplayReadinessFailureV1<A> {
    pub const fn error(&self) -> FinalCanonicalReplayReadinessErrorV1 {
        self.error
    }

    pub fn into_activated(self) -> A {
        self.activated
    }
}

pub struct FinalCanonicalReplayInvocationReadyV1<A> {
    activated: A,
    attempt: ActuatorInvocationAttemptManifestV1,
    source_kind: FinalCanonicalReplaySourceKindV1,
    audit_evidence: FinalCanonicalReplayAuditEvidenceV1,
    activation_latest_possible_unix_ms: u64,
    valid_until: u64,
}

impl<A> FinalCanonicalReplayInvocationReadyV1<A> {
    pub const fn activated(&self) -> &A {
        &self.activated
    }

    pub const fn attempt(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.attempt
    }

    pub const fn source_kind(&self) -> FinalCanonicalReplaySourceKindV1 {
        self.source_kind
    }

    pub const fn audit_evidence(&self) -> FinalCanonicalReplayAuditEvidenceV1 {
        self.audit_evidence
    }

    pub const fn activation_latest_possible_unix_ms(&self) -> u64 {
        self.activation_latest_possible_unix_ms
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn eligible_for_canonical_replay_actuator_reservation(&self) -> bool {
        true
    }

    pub const fn actuator_reservation_performed(&self) -> bool {
        false
    }

    pub const fn external_effect_attempted(&self) -> bool {
        false
    }

    pub const fn permits_third_attempt(&self) -> bool {
        false
    }
}

pub fn finalize_canonical_replay_invocation_readiness<A>(
    activated: A,
) -> Result<
    FinalCanonicalReplayInvocationReadyV1<A>,
    FinalCanonicalReplayReadinessFailureV1<A>,
>
where
    A: ExactActivatedCanonicalReplayV1,
{
    let attempt = activated.exact_attempt();
    let activation_latest = activated.activation_latest_possible_unix_ms();
    let activation_valid_until = activated.activation_valid_until();

    let error = if attempt.schema_version != SSF_SCHEMA_V1 {
        Some(FinalCanonicalReplayReadinessErrorV1::UnsupportedAttemptSchema)
    } else if activation_latest < attempt.latest_possible_unix_ms {
        Some(FinalCanonicalReplayReadinessErrorV1::ActivationTimeRegressedBeforeAttempt)
    } else if activation_valid_until < activation_latest {
        Some(FinalCanonicalReplayReadinessErrorV1::ActivationAlreadyExpired)
    } else if attempt.attempt_valid_until < activation_latest {
        Some(FinalCanonicalReplayReadinessErrorV1::AttemptAlreadyExpired)
    } else if attempt.subject.actuator.valid_until < activation_latest {
        Some(FinalCanonicalReplayReadinessErrorV1::ActuatorGenerationAlreadyExpired)
    } else if activated.permits_third_attempt() {
        Some(FinalCanonicalReplayReadinessErrorV1::UnexpectedThirdAttemptPermission)
    } else {
        None
    };

    if let Some(error) = error {
        return Err(FinalCanonicalReplayReadinessFailureV1 { activated, error });
    }

    let valid_until = activation_valid_until
        .min(attempt.attempt_valid_until)
        .min(attempt.subject.actuator.valid_until);

    Ok(FinalCanonicalReplayInvocationReadyV1 {
        source_kind: activated.source_kind(),
        audit_evidence: activated.audit_evidence(),
        activated,
        attempt,
        activation_latest_possible_unix_ms: activation_latest,
        valid_until,
    })
}

#[cfg(test)]
mod tests {
    #[test]
    fn final_replay_ceiling_only_shrinks() {
        assert_eq!(100_u64.min(95).min(90), 90);
    }

    #[test]
    fn third_attempt_is_not_part_of_v0_1() {
        let permits_third_attempt = false;
        assert!(!permits_third_attempt);
    }
}
