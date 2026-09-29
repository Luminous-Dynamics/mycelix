// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-009J: bind the exact transaction plan to a closed execution surface.
//!
//! The positive subject is intentionally non-authoritative:
//!
//! ExecutorConstrainedGitRefTransactionPlanV1
//! + exact principal-to-executor binding
//! + deterministic sealed interface commitment
//! + independent verifier
//!     -> SealedExecutorInterfaceBindingV1
//!
//! This does not prove host/process confinement by itself. The verifier evidence
//! is the independent claim that the concrete runtime actually enforces the
//! fixed interface represented by the commitment.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use forge_git_ref_transaction::GitRefTransactionPlanV1;
use mycelix_forge_core::{Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion};
use mycelix_forge_protected_ref_executor_confinement::{
    ExecutorConfinementBindingV1, ExecutorConfinementVerifierIdentityV1,
    ExecutorConstrainedGitRefTransactionPlanV1,
};
use serde::{Deserialize, Serialize};
use thiserror::Error;

const SEALED_INTERFACE_DOMAIN_V1: &[u8] =
    b"mycelix-forge/sealed-executor-interface/v1\0";
const OBSERVATION_DOMAIN_V1: &[u8] =
    b"mycelix-forge/sealed-executor-interface-observation/v1\0";
const EVIDENCE_DOMAIN_V1: &[u8] =
    b"mycelix-forge/sealed-executor-interface-evidence/v1\0";
const INVOCATION_DOMAIN_V1: &[u8] =
    b"mycelix-forge/sealed-executor-invocation/v1\0";
const MAX_PRINCIPAL_LEN: usize = 512;

/// Fixed execution-surface constraints that a concrete runtime must enforce.
pub const SEALED_EXECUTOR_INTERFACE_CONSTRAINTS_V1: &[&str] = &[
    "no-shell",
    "fixed-executable-by-provider",
    "fixed-repository-identity-by-provider",
    "argv-exactly-from-plan",
    "stdin-exactly-from-plan",
    "no-caller-arguments",
    "no-caller-stdin",
    "no-caller-ref-or-object",
    "no-caller-environment-overrides",
];

/// Raw provider evidence claiming that one exact executor binding exposes only
/// the sealed interface for one exact transaction plan in one exact repository.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SealedExecutorInterfaceObservationV1 {
    version: ProtocolVersion,
    verifier: ExecutorConfinementVerifierIdentityV1,
    principal_id: String,
    executor_identity: Digest,
    repository_identity: Digest,
    plan_evidence: Digest,
    interface_commitment: Digest,
    provider_evidence: Digest,
}

impl SealedExecutorInterfaceObservationV1 {
    /// Construct raw, non-authoritative provider evidence.
    pub fn new(
        verifier: ExecutorConfinementVerifierIdentityV1,
        principal_id: impl Into<String>,
        executor_identity: Digest,
        repository_identity: Digest,
        plan_evidence: Digest,
        interface_commitment: Digest,
        provider_evidence: Digest,
    ) -> Result<Self, SealedExecutorInterfaceError> {
        let principal_id = principal_id.into();
        validate_principal_id(&principal_id)?;
        Ok(Self {
            version: ProtocolVersion::CURRENT,
            verifier,
            principal_id,
            executor_identity,
            repository_identity,
            plan_evidence,
            interface_commitment,
            provider_evidence,
        })
    }

    /// Independent verifier identity claimed by the provider.
    pub fn verifier(&self) -> &ExecutorConfinementVerifierIdentityV1 {
        &self.verifier
    }

    /// Exact gittuf principal represented by this executor binding.
    pub fn principal_id(&self) -> &str {
        &self.principal_id
    }

    /// Exact executor identity commitment.
    pub fn executor_identity(&self) -> &Digest {
        &self.executor_identity
    }

    /// Exact FORGE-009F transaction-plan evidence commitment.
    pub fn plan_evidence(&self) -> &Digest {
        &self.plan_evidence
    }

    /// Deterministic commitment to the sealed execution surface.
    pub fn interface_commitment(&self) -> &Digest {
        &self.interface_commitment
    }

    /// Provider-native evidence commitment.
    pub fn provider_evidence(&self) -> &Digest {
        &self.provider_evidence
    }

    /// Canonical observation commitment.
    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, SealedExecutorInterfaceError> {
        let mut out = Vec::new();
        out.extend_from_slice(OBSERVATION_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_string(&mut out, self.verifier.name())?;
        push_string(&mut out, self.verifier.version())?;
        push_string(&mut out, &self.principal_id)?;
        push_digest(&mut out, &self.executor_identity)?;
        push_digest(&mut out, &self.repository_identity)?;
        push_digest(&mut out, &self.plan_evidence)?;
        push_digest(&mut out, &self.interface_commitment)?;
        push_digest(&mut out, &self.provider_evidence)?;
        Ok(Digest::of_bytes(algorithm, &out))
    }
}

/// Independent verifier for a concrete sealed-executor interface.
pub trait SealedExecutorInterfaceVerifierV1 {
    /// Stable verifier identity.
    fn identity(&self) -> ExecutorConfinementVerifierIdentityV1;

    /// Verify that the concrete executor exposes only the exact supplied
    /// sealed interface for the exact supplied transaction plan.
    fn verify_sealed_executor_interface(
        &self,
        constrained_plan: &ExecutorConstrainedGitRefTransactionPlanV1,
        plan: &GitRefTransactionPlanV1,
        binding: &ExecutorConfinementBindingV1,
        observation: &SealedExecutorInterfaceObservationV1,
    ) -> Result<Digest, SealedExecutorInterfaceVerifierErrorV1>;
}

/// Provider-specific sealed-interface verification failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum SealedExecutorInterfaceVerifierErrorV1 {
    /// Provider evidence did not establish the declared interface.
    #[error("sealed executor verifier rejected provider evidence")]
    Rejected,
}

/// Positive, non-deserializable binding between one exact executor, one exact
/// repository, and one exact transaction plan through the fixed sealed
/// execution surface.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct SealedExecutorInterfaceBindingV1 {
    project: ProjectIdentity,
    principal_id: String,
    executor_identity: Digest,
    repository_identity: Digest,
    plan_evidence: Digest,
    interface_commitment: Digest,
    verifier: ExecutorConfinementVerifierIdentityV1,
    provider_evidence: Digest,
    verifier_evidence: Digest,
    evidence_commitment: Digest,
}

/// Non-authoritative invocation description derived only from a qualified
/// sealed-executor binding and the exact transaction plan.
///
/// This type intentionally has no executable path, repository path, environment
/// map, shell string, or caller-controlled arguments. It is a transport object
/// for a later runtime; possessing it is not execution authorization.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SealedExecutorInvocationV1 {
    executor_identity: Digest,
    repository_identity: Digest,
    plan_evidence: Digest,
    interface_commitment: Digest,
    command_args: Vec<String>,
    stdin_bytes: Vec<u8>,
}

impl SealedExecutorInvocationV1 {
    /// Materialize the exact invocation surface from one positive binding and
    /// the exact plan it commits to.
    pub fn from_binding(
        binding: &SealedExecutorInterfaceBindingV1,
        plan: &GitRefTransactionPlanV1,
    ) -> Result<Self, SealedExecutorInterfaceError> {
        if binding.plan_evidence() != plan.evidence_commitment() {
            return Err(SealedExecutorInterfaceError::PlanMismatch);
        }

        let expected_interface = sealed_executor_interface_commitment_v1(
            plan,
            binding.repository_identity(),
            binding.plan_evidence().algorithm(),
        )?;
        if binding.interface_commitment() != &expected_interface {
            return Err(SealedExecutorInterfaceError::InterfaceCommitmentMismatch);
        }

        Ok(Self {
            executor_identity: binding.executor_identity().clone(),
            repository_identity: binding.repository_identity().clone(),
            plan_evidence: binding.plan_evidence().clone(),
            interface_commitment: expected_interface,
            command_args: plan
                .command_args()
                .iter()
                .map(|arg| (*arg).to_owned())
                .collect(),
            stdin_bytes: plan.stdin_bytes().to_vec(),
        })
    }

    /// Exact executor identity commitment.
    pub fn executor_identity(&self) -> &Digest {
        &self.executor_identity
    }

    /// Exact provider-defined repository identity commitment.
    pub fn repository_identity(&self) -> &Digest {
        &self.repository_identity
    }

    /// Exact FORGE-009F transaction-plan evidence commitment.
    pub fn plan_evidence(&self) -> &Digest {
        &self.plan_evidence
    }

    /// Exact sealed interface commitment.
    pub fn interface_commitment(&self) -> &Digest {
        &self.interface_commitment
    }

    /// Canonical invocation commitment for durable authority/journal binding.
    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, SealedExecutorInterfaceError> {
        let mut out = Vec::new();
        out.extend_from_slice(INVOCATION_DOMAIN_V1);
        push_digest(&mut out, &self.executor_identity)?;
        push_digest(&mut out, &self.repository_identity)?;
        push_digest(&mut out, &self.plan_evidence)?;
        push_digest(&mut out, &self.interface_commitment)?;
        let arg_count = u16::try_from(self.command_args.len())
            .map_err(|_| SealedExecutorInterfaceError::CanonicalFieldTooLarge("argv"))?;
        out.extend_from_slice(&arg_count.to_be_bytes());
        for arg in &self.command_args {
            push_string(&mut out, arg)?;
        }
        let stdin_len = u32::try_from(self.stdin_bytes.len())
            .map_err(|_| SealedExecutorInterfaceError::CanonicalFieldTooLarge("stdin"))?;
        out.extend_from_slice(&stdin_len.to_be_bytes());
        out.extend_from_slice(&self.stdin_bytes);
        Ok(Digest::of_bytes(algorithm, &out))
    }

    /// Exact Git argv; callers cannot replace or append arguments.
    pub fn command_args(&self) -> &[String] {
        &self.command_args
    }

    /// Exact NUL-framed Git transaction bytes.
    pub fn stdin_bytes(&self) -> &[u8] {
        &self.stdin_bytes
    }
}

impl SealedExecutorInterfaceBindingV1 {
    /// Exact project.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact authorized principal.
    pub fn principal_id(&self) -> &str {
        &self.principal_id
    }

    /// Exact executor identity.
    pub fn executor_identity(&self) -> &Digest {
        &self.executor_identity
    }

    /// Exact provider-defined repository identity commitment.
    pub fn repository_identity(&self) -> &Digest {
        &self.repository_identity
    }

    /// Exact FORGE-009F transaction-plan evidence.
    pub fn plan_evidence(&self) -> &Digest {
        &self.plan_evidence
    }

    /// Exact sealed interface commitment.
    pub fn interface_commitment(&self) -> &Digest {
        &self.interface_commitment
    }

    /// Independent verifier identity.
    pub fn verifier(&self) -> &ExecutorConfinementVerifierIdentityV1 {
        &self.verifier
    }

    /// Provider-native evidence.
    pub fn provider_evidence(&self) -> &Digest {
        &self.provider_evidence
    }

    /// Independent verifier evidence.
    pub fn verifier_evidence(&self) -> &Digest {
        &self.verifier_evidence
    }

    /// Aggregate evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Compute the deterministic execution-surface commitment for one exact plan
/// and one exact provider-defined repository identity.
pub fn sealed_executor_interface_commitment_v1(
    plan: &GitRefTransactionPlanV1,
    repository_identity: &Digest,
    algorithm: DigestAlgorithm,
) -> Result<Digest, SealedExecutorInterfaceError> {
    let mut out = Vec::new();
    out.extend_from_slice(SEALED_INTERFACE_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, plan.evidence_commitment())?;
    push_digest(&mut out, repository_identity)?;

    let args = plan.command_args();
    let arg_count = u16::try_from(args.len())
        .map_err(|_| SealedExecutorInterfaceError::CanonicalFieldTooLarge("argv"))?;
    out.extend_from_slice(&arg_count.to_be_bytes());
    for arg in args {
        push_string(&mut out, arg)?;
    }

    push_digest(&mut out, plan.stdin_digest())?;

    let constraint_count = u16::try_from(SEALED_EXECUTOR_INTERFACE_CONSTRAINTS_V1.len())
        .map_err(|_| SealedExecutorInterfaceError::CanonicalFieldTooLarge("constraints"))?;
    out.extend_from_slice(&constraint_count.to_be_bytes());
    for constraint in SEALED_EXECUTOR_INTERFACE_CONSTRAINTS_V1 {
        push_string(&mut out, constraint)?;
    }

    Ok(Digest::of_bytes(algorithm, &out))
}

/// Qualify one exact executor binding against one exact transaction plan and
/// an independently verified sealed execution surface.
pub fn qualify_sealed_executor_interface_v1<V: SealedExecutorInterfaceVerifierV1>(
    constrained_plan: &ExecutorConstrainedGitRefTransactionPlanV1,
    plan: &GitRefTransactionPlanV1,
    observation: &SealedExecutorInterfaceObservationV1,
    verifier: &V,
) -> Result<SealedExecutorInterfaceBindingV1, SealedExecutorInterfaceError> {
    let expected_plan = plan.evidence_commitment();
    if constrained_plan.plan_evidence() != expected_plan
        || observation.plan_evidence() != expected_plan
    {
        return Err(SealedExecutorInterfaceError::PlanMismatch);
    }

    let binding = constrained_plan
        .bindings()
        .iter()
        .find(|binding| binding.principal_id() == observation.principal_id())
        .ok_or(SealedExecutorInterfaceError::PrincipalBindingMissing)?;

    if binding.allowed_plan() != expected_plan {
        return Err(SealedExecutorInterfaceError::BindingPlanMismatch);
    }

    if binding.executor_identity() != observation.executor_identity() {
        return Err(SealedExecutorInterfaceError::ExecutorIdentityMismatch);
    }

    if observation.verifier() != &verifier.identity() {
        return Err(SealedExecutorInterfaceError::VerifierIdentityMismatch);
    }

    let expected_interface = sealed_executor_interface_commitment_v1(
        plan,
        observation.repository_identity(),
        expected_plan.algorithm(),
    )?;
    if observation.interface_commitment() != &expected_interface {
        return Err(SealedExecutorInterfaceError::InterfaceCommitmentMismatch);
    }

    let verifier_evidence = verifier
        .verify_sealed_executor_interface(constrained_plan, plan, binding, observation)
        .map_err(SealedExecutorInterfaceError::VerifierRejected)?;

    let algorithm = verifier_evidence.algorithm();
    let observation_digest = observation.digest(algorithm)?;

    let mut out = Vec::new();
    out.extend_from_slice(EVIDENCE_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project(&mut out, constrained_plan.project())?;
    push_string(&mut out, observation.principal_id())?;
    push_digest(&mut out, observation.executor_identity())?;
    push_digest(&mut out, observation.repository_identity())?;
    push_digest(&mut out, expected_plan)?;
    push_digest(&mut out, &expected_interface)?;
    push_digest(&mut out, constrained_plan.evidence_commitment())?;
    push_digest(&mut out, &observation_digest)?;
    push_digest(&mut out, &verifier_evidence)?;

    Ok(SealedExecutorInterfaceBindingV1 {
        project: constrained_plan.project().clone(),
        principal_id: observation.principal_id().to_owned(),
        executor_identity: observation.executor_identity().clone(),
        repository_identity: observation.repository_identity().clone(),
        plan_evidence: expected_plan.clone(),
        interface_commitment: expected_interface,
        verifier: observation.verifier().clone(),
        provider_evidence: observation.provider_evidence().clone(),
        verifier_evidence,
        evidence_commitment: Digest::of_bytes(algorithm, &out),
    })
}

fn validate_principal_id(value: &str) -> Result<(), SealedExecutorInterfaceError> {
    if value.is_empty() || value.len() > MAX_PRINCIPAL_LEN {
        return Err(SealedExecutorInterfaceError::InvalidPrincipalId);
    }
    Ok(())
}

fn push_project(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), SealedExecutorInterfaceError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(
    out: &mut Vec<u8>,
    digest: &Digest,
) -> Result<(), SealedExecutorInterfaceError> {
    push_string(out, digest.algorithm().id())?;
    let len = u32::try_from(digest.as_bytes().len())
        .map_err(|_| SealedExecutorInterfaceError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

fn push_string(
    out: &mut Vec<u8>,
    value: &str,
) -> Result<(), SealedExecutorInterfaceError> {
    let bytes = value.as_bytes();
    let len = u32::try_from(bytes.len())
        .map_err(|_| SealedExecutorInterfaceError::CanonicalFieldTooLarge("string"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// FORGE-009J qualification failures.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum SealedExecutorInterfaceError {
    /// Transaction plan identity differs from FORGE-009I.
    #[error("sealed executor interface references a different transaction plan")]
    PlanMismatch,
    /// No FORGE-009I binding exists for the observed principal.
    #[error("sealed executor interface principal has no FORGE-009I binding")]
    PrincipalBindingMissing,
    /// The principal is bound to another plan.
    #[error("sealed executor interface binding references another plan")]
    BindingPlanMismatch,
    /// The observed executor differs from the FORGE-009I binding.
    #[error("sealed executor interface executor identity mismatch")]
    ExecutorIdentityMismatch,
    /// The verifier identity differs from the supplied independent verifier.
    #[error("sealed executor interface verifier identity mismatch")]
    VerifierIdentityMismatch,
    /// Provider interface evidence does not match the deterministic interface.
    #[error("sealed executor interface commitment mismatch")]
    InterfaceCommitmentMismatch,
    /// Independent verifier rejected the provider evidence.
    #[error("sealed executor interface verifier rejected provider evidence: {0}")]
    VerifierRejected(SealedExecutorInterfaceVerifierErrorV1),
    /// Principal identifier is empty or exceeds v1 bounds.
    #[error("sealed executor interface principal identifier is invalid")]
    InvalidPrincipalId,
    /// Canonical field overflow.
    #[error("canonical field too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    #[test]
    fn invocation_constructor_requires_positive_binding() {
        let constructor: fn(
            &SealedExecutorInterfaceBindingV1,
            &GitRefTransactionPlanV1,
        ) -> Result<SealedExecutorInvocationV1, SealedExecutorInterfaceError> =
            SealedExecutorInvocationV1::from_binding;
        let _ = constructor;
    }

    #[test]
    fn interface_constraints_are_closed_and_shell_free() {
        assert!(SEALED_EXECUTOR_INTERFACE_CONSTRAINTS_V1.contains(&"no-shell"));
        assert!(SEALED_EXECUTOR_INTERFACE_CONSTRAINTS_V1.contains(&"no-caller-arguments"));
        assert!(SEALED_EXECUTOR_INTERFACE_CONSTRAINTS_V1.contains(&"no-caller-stdin"));
        assert!(!SEALED_EXECUTOR_INTERFACE_CONSTRAINTS_V1.contains(&"arbitrary-command"));
    }

    #[test]
    fn raw_observation_round_trips() {
        let observation = SealedExecutorInterfaceObservationV1::new(
            ExecutorConfinementVerifierIdentityV1::new("test-verifier", "v1").unwrap(),
            "principal-a",
            digest(1),
            digest(2),
            digest(3),
            digest(4),
            digest(5),
        )
        .unwrap();

        let bytes = serde_json::to_vec(&observation).unwrap();
        let decoded: SealedExecutorInterfaceObservationV1 =
            serde_json::from_slice(&bytes).unwrap();
        assert_eq!(observation, decoded);
        assert_eq!(observation.repository_identity(), &digest(2));
    }

    #[test]
    fn invocation_digest_domain_is_distinct() {
        let invocation = Digest::of_bytes(
            DigestAlgorithm::Sha256,
            b"mycelix-forge/sealed-executor-invocation/v1\0",
        );
        let interface = Digest::of_bytes(
            DigestAlgorithm::Sha256,
            b"mycelix-forge/sealed-executor-interface/v1\0",
        );
        assert_ne!(invocation, interface);
    }

    #[test]
    fn observation_rejects_empty_principal() {
        assert_eq!(
            SealedExecutorInterfaceObservationV1::new(
                ExecutorConfinementVerifierIdentityV1::new("test-verifier", "v1").unwrap(),
                "",
                digest(1),
                digest(2),
                digest(3),
                digest(4),
                digest(5),
            )
            .unwrap_err(),
            SealedExecutorInterfaceError::InvalidPrincipalId
        );
    }
}
