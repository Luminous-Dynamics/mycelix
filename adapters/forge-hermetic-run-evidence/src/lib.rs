// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-004D3B1: same-run orchestration evidence for hermetic Forge verification.
//!
//! This contract closes a composition gap that policy-only sandbox evidence
//! cannot close: a passing isolation probe from one process must not be reusable
//! to bless a different verifier process. The positive type binds one run
//! challenge, one process instance, one exact ordered phase transcript, and
//! pre/post runtime-closure equality.

use mycelix_forge_core::{Digest, DigestAlgorithm};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use thiserror::Error;

const PLAN_DOMAIN_V1: &[u8] = b"mycelix-forge/hermetic-run-plan/v1\0";
const OBSERVATION_DOMAIN_V1: &[u8] = b"mycelix-forge/hermetic-run-observation/v1\0";
const QUALIFIED_DOMAIN_V1: &[u8] = b"mycelix-forge/hermetic-run-qualified/v1\0";
pub const RUN_CHALLENGE_SIZE: u64 = 32;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum RunPhase {
    PreRuntimeClosure,
    GitObjectValidation,
    PolicyTrustQualification,
    IsolationProbe,
    VerifierExecution,
    IsolationQualification,
    PostRuntimeClosure,
}

impl RunPhase {
    const fn code(self) -> u8 {
        match self {
            Self::PreRuntimeClosure => 1,
            Self::GitObjectValidation => 2,
            Self::PolicyTrustQualification => 3,
            Self::IsolationProbe => 4,
            Self::VerifierExecution => 5,
            Self::IsolationQualification => 6,
            Self::PostRuntimeClosure => 7,
        }
    }

    pub const fn required_sequence() -> [Self; 7] {
        [
            Self::PreRuntimeClosure,
            Self::GitObjectValidation,
            Self::PolicyTrustQualification,
            Self::IsolationProbe,
            Self::VerifierExecution,
            Self::IsolationQualification,
            Self::PostRuntimeClosure,
        ]
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct HermeticRunPlan {
    execution_spec: Digest,
    isolation_policy: Digest,
    git_object_validation_policy: Digest,
    run_challenge: Digest,
    run_challenge_size: u64,
}

impl HermeticRunPlan {
    pub fn new(
        execution_spec: Digest,
        isolation_policy: Digest,
        git_object_validation_policy: Digest,
        run_challenge: Digest,
        run_challenge_size: u64,
    ) -> Result<Self, HermeticRunError> {
        if run_challenge_size != RUN_CHALLENGE_SIZE {
            return Err(HermeticRunError::InvalidRunChallengeSize {
                expected: RUN_CHALLENGE_SIZE,
                actual: run_challenge_size,
            });
        }
        Ok(Self {
            execution_spec,
            isolation_policy,
            git_object_validation_policy,
            run_challenge,
            run_challenge_size,
        })
    }

    pub fn execution_spec(&self) -> &Digest {
        &self.execution_spec
    }

    pub fn isolation_policy(&self) -> &Digest {
        &self.isolation_policy
    }

    pub fn git_object_validation_policy(&self) -> &Digest {
        &self.git_object_validation_policy
    }

    pub fn run_challenge(&self) -> &Digest {
        &self.run_challenge
    }

    pub const fn run_challenge_size(&self) -> u64 {
        self.run_challenge_size
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, HermeticRunError> {
        let mut out = Vec::new();
        out.extend_from_slice(PLAN_DOMAIN_V1);
        push_digest(&mut out, &self.execution_spec)?;
        push_digest(&mut out, &self.isolation_policy)?;
        push_digest(&mut out, &self.git_object_validation_policy)?;
        push_digest(&mut out, &self.run_challenge)?;
        out.extend_from_slice(&self.run_challenge_size.to_be_bytes());
        Ok(out)
    }

    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, HermeticRunError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for HermeticRunPlan {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            execution_spec: Digest,
            isolation_policy: Digest,
            git_object_validation_policy: Digest,
            run_challenge: Digest,
            run_challenge_size: u64,
        }

        let wire = Wire::deserialize(deserializer)?;
        Self::new(
            wire.execution_spec,
            wire.isolation_policy,
            wire.git_object_validation_policy,
            wire.run_challenge,
            wire.run_challenge_size,
        )
        .map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RunPhaseEvidence {
    phase: RunPhase,
    evidence: Digest,
}

impl RunPhaseEvidence {
    pub fn new(phase: RunPhase, evidence: Digest) -> Self {
        Self { phase, evidence }
    }

    pub const fn phase(&self) -> RunPhase {
        self.phase
    }

    pub fn evidence(&self) -> &Digest {
        &self.evidence
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct HermeticRunObservation {
    plan_digest: Digest,
    run_challenge: Digest,
    child_pid: u32,
    bubblewrap_status_commitment: Digest,
    pidfd_bound_before_release: bool,
    child_blocked_during_parent_observation: bool,
    phases: Vec<RunPhaseEvidence>,
    child_exit_code: i32,
}

impl HermeticRunObservation {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        plan_digest: Digest,
        run_challenge: Digest,
        child_pid: u32,
        bubblewrap_status_commitment: Digest,
        pidfd_bound_before_release: bool,
        child_blocked_during_parent_observation: bool,
        phases: Vec<RunPhaseEvidence>,
        child_exit_code: i32,
    ) -> Result<Self, HermeticRunError> {
        if child_pid == 0 {
            return Err(HermeticRunError::InvalidChildPid);
        }
        validate_phase_sequence(&phases)?;
        Ok(Self {
            plan_digest,
            run_challenge,
            child_pid,
            bubblewrap_status_commitment,
            pidfd_bound_before_release,
            child_blocked_during_parent_observation,
            phases,
            child_exit_code,
        })
    }

    pub fn phases(&self) -> &[RunPhaseEvidence] {
        &self.phases
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, HermeticRunError> {
        validate_phase_sequence(&self.phases)?;
        let phase_count = u16::try_from(self.phases.len())
            .map_err(|_| HermeticRunError::CanonicalLengthOverflow)?;
        let mut out = Vec::new();
        out.extend_from_slice(OBSERVATION_DOMAIN_V1);
        push_digest(&mut out, &self.plan_digest)?;
        push_digest(&mut out, &self.run_challenge)?;
        out.extend_from_slice(&self.child_pid.to_be_bytes());
        push_digest(&mut out, &self.bubblewrap_status_commitment)?;
        out.push(u8::from(self.pidfd_bound_before_release));
        out.push(u8::from(self.child_blocked_during_parent_observation));
        out.extend_from_slice(&phase_count.to_be_bytes());
        for phase in &self.phases {
            out.push(phase.phase.code());
            push_digest(&mut out, &phase.evidence)?;
        }
        out.extend_from_slice(&self.child_exit_code.to_be_bytes());
        Ok(out)
    }

    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, HermeticRunError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for HermeticRunObservation {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            plan_digest: Digest,
            run_challenge: Digest,
            child_pid: u32,
            bubblewrap_status_commitment: Digest,
            pidfd_bound_before_release: bool,
            child_blocked_during_parent_observation: bool,
            phases: Vec<RunPhaseEvidence>,
            child_exit_code: i32,
        }

        let wire = Wire::deserialize(deserializer)?;
        Self::new(
            wire.plan_digest,
            wire.run_challenge,
            wire.child_pid,
            wire.bubblewrap_status_commitment,
            wire.pidfd_bound_before_release,
            wire.child_blocked_during_parent_observation,
            wire.phases,
            wire.child_exit_code,
        )
        .map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualifiedHermeticRunEvidence {
    plan_digest: Digest,
    run_challenge: Digest,
    pre_runtime_closure: Digest,
    git_object_validation: Digest,
    policy_trust: Digest,
    isolation_probe: Digest,
    verifier_output: Digest,
    isolation: Digest,
    post_runtime_closure: Digest,
    observation_digest: Digest,
    evidence_digest: Digest,
}

impl QualifiedHermeticRunEvidence {
    pub fn plan_digest(&self) -> &Digest {
        &self.plan_digest
    }

    pub fn run_challenge(&self) -> &Digest {
        &self.run_challenge
    }

    pub fn pre_runtime_closure(&self) -> &Digest {
        &self.pre_runtime_closure
    }

    pub fn git_object_validation(&self) -> &Digest {
        &self.git_object_validation
    }

    pub fn policy_trust(&self) -> &Digest {
        &self.policy_trust
    }

    pub fn isolation_probe(&self) -> &Digest {
        &self.isolation_probe
    }

    pub fn verifier_output(&self) -> &Digest {
        &self.verifier_output
    }

    pub fn isolation(&self) -> &Digest {
        &self.isolation
    }

    pub fn post_runtime_closure(&self) -> &Digest {
        &self.post_runtime_closure
    }

    pub fn observation_digest(&self) -> &Digest {
        &self.observation_digest
    }

    pub fn evidence_digest(&self) -> &Digest {
        &self.evidence_digest
    }
}

pub fn qualify_hermetic_run(
    plan: &HermeticRunPlan,
    observation: &HermeticRunObservation,
) -> Result<QualifiedHermeticRunEvidence, HermeticRunError> {
    let expected_plan = plan.digest(observation.plan_digest.algorithm())?;
    if expected_plan != observation.plan_digest {
        return Err(HermeticRunError::PlanDigestMismatch);
    }
    if plan.run_challenge != observation.run_challenge {
        return Err(HermeticRunError::RunChallengeMismatch);
    }
    if !observation.pidfd_bound_before_release {
        return Err(HermeticRunError::PidfdNotBoundBeforeRelease);
    }
    if !observation.child_blocked_during_parent_observation {
        return Err(HermeticRunError::ChildNotHeldForParentObservation);
    }
    if observation.child_exit_code != 0 {
        return Err(HermeticRunError::ChildFailed(observation.child_exit_code));
    }

    validate_phase_sequence(&observation.phases)?;
    let evidence = |phase: RunPhase| -> &Digest {
        observation
            .phases
            .iter()
            .find(|entry| entry.phase == phase)
            .expect("exact phase sequence validated")
            .evidence()
    };
    let pre = evidence(RunPhase::PreRuntimeClosure).clone();
    let post = evidence(RunPhase::PostRuntimeClosure).clone();
    if pre != post {
        return Err(HermeticRunError::RuntimeClosureChangedDuringRun);
    }

    let observation_digest = observation.digest(DigestAlgorithm::Sha256)?;
    let plan_digest = plan.digest(DigestAlgorithm::Sha256)?;
    let mut bytes = Vec::new();
    bytes.extend_from_slice(QUALIFIED_DOMAIN_V1);
    push_digest(&mut bytes, &plan_digest)?;
    push_digest(&mut bytes, &observation_digest)?;
    for phase in RunPhase::required_sequence() {
        push_digest(&mut bytes, evidence(phase))?;
    }
    let evidence_digest = Digest::of_bytes(DigestAlgorithm::Sha256, &bytes);

    Ok(QualifiedHermeticRunEvidence {
        plan_digest,
        run_challenge: plan.run_challenge.clone(),
        pre_runtime_closure: pre,
        git_object_validation: evidence(RunPhase::GitObjectValidation).clone(),
        policy_trust: evidence(RunPhase::PolicyTrustQualification).clone(),
        isolation_probe: evidence(RunPhase::IsolationProbe).clone(),
        verifier_output: evidence(RunPhase::VerifierExecution).clone(),
        isolation: evidence(RunPhase::IsolationQualification).clone(),
        post_runtime_closure: post,
        observation_digest,
        evidence_digest,
    })
}

fn validate_phase_sequence(phases: &[RunPhaseEvidence]) -> Result<(), HermeticRunError> {
    let expected = RunPhase::required_sequence();
    if phases.len() != expected.len()
        || phases
            .iter()
            .zip(expected)
            .any(|(observed, required)| observed.phase != required)
    {
        return Err(HermeticRunError::InvalidPhaseSequence);
    }
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), HermeticRunError> {
    let algorithm = digest.algorithm().id().as_bytes();
    let algorithm_len = u16::try_from(algorithm.len())
        .map_err(|_| HermeticRunError::CanonicalLengthOverflow)?;
    let digest_len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| HermeticRunError::CanonicalLengthOverflow)?;
    out.extend_from_slice(&algorithm_len.to_be_bytes());
    out.extend_from_slice(algorithm);
    out.extend_from_slice(&digest_len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum HermeticRunError {
    #[error("run challenge must be exactly {expected} bytes, got {actual}")]
    InvalidRunChallengeSize { expected: u64, actual: u64 },
    #[error("child PID may not be zero")]
    InvalidChildPid,
    #[error("run phases are missing, duplicated, or out of order")]
    InvalidPhaseSequence,
    #[error("run observation names a different launch plan")]
    PlanDigestMismatch,
    #[error("run observation names a different challenge")]
    RunChallengeMismatch,
    #[error("pidfd/process binding was not established before child release")]
    PidfdNotBoundBeforeRelease,
    #[error("sandbox child was not held while parent-side observations were captured")]
    ChildNotHeldForParentObservation,
    #[error("sandbox child exited unsuccessfully: {0}")]
    ChildFailed(i32),
    #[error("runtime NAR closure changed between preflight and postflight")]
    RuntimeClosureChangedDuringRun,
    #[error("canonical length overflow")]
    CanonicalLengthOverflow,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn plan() -> HermeticRunPlan {
        HermeticRunPlan::new(
            digest(1),
            digest(2),
            digest(3),
            digest(4),
            RUN_CHALLENGE_SIZE,
        )
        .unwrap()
    }

    fn phases(post: u8) -> Vec<RunPhaseEvidence> {
        vec![
            RunPhaseEvidence::new(RunPhase::PreRuntimeClosure, digest(10)),
            RunPhaseEvidence::new(RunPhase::GitObjectValidation, digest(11)),
            RunPhaseEvidence::new(RunPhase::PolicyTrustQualification, digest(12)),
            RunPhaseEvidence::new(RunPhase::IsolationProbe, digest(13)),
            RunPhaseEvidence::new(RunPhase::VerifierExecution, digest(14)),
            RunPhaseEvidence::new(RunPhase::IsolationQualification, digest(15)),
            RunPhaseEvidence::new(RunPhase::PostRuntimeClosure, digest(post)),
        ]
    }

    fn observation(phases: Vec<RunPhaseEvidence>) -> HermeticRunObservation {
        let plan = plan();
        HermeticRunObservation::new(
            plan.digest(DigestAlgorithm::Sha256).unwrap(),
            digest(4),
            4242,
            digest(5),
            true,
            true,
            phases,
            0,
        )
        .unwrap()
    }

    #[test]
    fn exact_same_run_sequence_qualifies() {
        let plan = plan();
        let qualified = qualify_hermetic_run(&plan, &observation(phases(10))).unwrap();
        assert_eq!(
            qualified.pre_runtime_closure(),
            qualified.post_runtime_closure()
        );
        assert_eq!(qualified.verifier_output(), &digest(14));
        assert_eq!(qualified.isolation(), &digest(15));
    }

    #[test]
    fn reordered_phases_fail_closed() {
        let mut entries = phases(10);
        entries.swap(3, 4);
        assert_eq!(
            HermeticRunObservation::new(
                plan().digest(DigestAlgorithm::Sha256).unwrap(),
                digest(4),
                1,
                digest(5),
                true,
                true,
                entries,
                0,
            )
            .unwrap_err(),
            HermeticRunError::InvalidPhaseSequence
        );
    }

    #[test]
    fn changed_post_runtime_closure_fails_closed() {
        let plan = plan();
        assert_eq!(
            qualify_hermetic_run(&plan, &observation(phases(16))).unwrap_err(),
            HermeticRunError::RuntimeClosureChangedDuringRun
        );
    }

    #[test]
    fn process_binding_is_required() {
        let plan = plan();
        let observation = HermeticRunObservation::new(
            plan.digest(DigestAlgorithm::Sha256).unwrap(),
            digest(4),
            1,
            digest(5),
            false,
            true,
            phases(10),
            0,
        )
        .unwrap();
        assert_eq!(
            qualify_hermetic_run(&plan, &observation).unwrap_err(),
            HermeticRunError::PidfdNotBoundBeforeRelease
        );
    }
}
