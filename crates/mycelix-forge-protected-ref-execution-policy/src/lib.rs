// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-009G: qualify the protected-ref execution profile against the exact
//! external repository-policy subject already committed by Forge.
//!
//! `RepositoryPolicyState::policy_digest()` already names the external policy
//! subject (for the current gittuf adapter, the active gittuf policy-ref tip
//! commitment). This crate deliberately preserves that meaning. The typed
//! protected-ref execution policy is a *second* subject whose enforcement must
//! be proven by an independently supplied policy verifier.
//!
//! ```text
//! RepositoryPolicyState(external policy subject)
//! + ProtectedRefExecutionPolicyV1
//! + provider observation that the external policy enforces that profile
//! + independent verifier
//!     -> QualifiedProtectedRefExecutionPolicyV1
//!
//! QualifiedProtectedRefExecutionPolicyV1
//! + GitRefTransactionPlanV1
//! + reserved-namespace invariants
//!     -> PolicyQualifiedGitRefTransactionPlanV1
//! ```

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use forge_git_ref_transaction::{
    GitRefTransactionPlanV1, GIT_UPDATE_REF_TRANSACTION_ARGS,
};
use mycelix_forge_core::{
    Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion, CURRENT_PROTOCOL_VERSION,
};
use mycelix_forge_repository::{
    RepositoryPolicyState, RepositoryRef, RepositoryVerificationError,
};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use thiserror::Error;

const EXECUTION_POLICY_DOMAIN_V1: &[u8] =
    b"mycelix-forge/protected-ref-execution-policy/v1\0";
const EXECUTION_POLICY_OBSERVATION_DOMAIN_V1: &[u8] =
    b"mycelix-forge/protected-ref-execution-policy-observation/v1\0";
const QUALIFIED_EXECUTION_POLICY_DOMAIN_V1: &[u8] =
    b"mycelix-forge/qualified-protected-ref-execution-policy/v1\0";
const POLICY_QUALIFIED_PLAN_DOMAIN_V1: &[u8] =
    b"mycelix-forge/policy-qualified-git-ref-transaction-plan/v1\0";
const MAX_VERIFIER_FIELD_LEN: usize = 128;

/// Reserved namespace containing durable one-time protected-merge consumption
/// markers.
pub const CONSUMPTION_MARKER_NAMESPACE_V1: &str = "refs/mycelix/forge/consumed/";

/// Typed v1 protected-ref execution profile.
///
/// Protocol v1 fixes the important invariants rather than accepting booleans
/// from callers:
///
/// - consumption markers live under [`CONSUMPTION_MARKER_NAMESPACE_V1`];
/// - ordinary protected targets may not be inside that namespace;
/// - marker operations are create-only;
/// - marker deletion/rewriting is forbidden by policy;
/// - target update + marker creation must be one atomic transaction;
/// - the Git realization must use no-deref compare-and-swap semantics.
///
/// This type is a policy subject, not proof that a repository host enforces it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProtectedRefExecutionPolicyV1 {
    version: ProtocolVersion,
    project: ProjectIdentity,
}

impl ProtectedRefExecutionPolicyV1 {
    /// Construct the fixed v1 execution profile for one project.
    pub fn new(project: ProjectIdentity) -> Self {
        Self {
            version: ProtocolVersion::CURRENT,
            project,
        }
    }

    /// Project governed by this execution policy.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Reserved durable-consumption namespace.
    pub const fn consumption_marker_namespace(&self) -> &'static str {
        CONSUMPTION_MARKER_NAMESPACE_V1
    }

    /// Whether marker creation must be in the same transaction as the target
    /// compare-and-swap.
    pub const fn requires_atomic_target_and_marker(&self) -> bool {
        true
    }

    /// Whether marker refs are create-only.
    pub const fn marker_create_only(&self) -> bool {
        true
    }

    /// Whether deletion of an existing consumption marker is permitted.
    pub const fn marker_deletion_allowed(&self) -> bool {
        false
    }

    /// Whether rewriting an existing consumption marker is permitted.
    pub const fn marker_rewrite_allowed(&self) -> bool {
        false
    }

    /// Whether the concrete Git realization must update the ref itself rather
    /// than following a symbolic ref.
    pub const fn requires_no_deref(&self) -> bool {
        true
    }

    /// Canonical v1 policy bytes.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ProtectedRefExecutionPolicyError> {
        ensure_v1(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(EXECUTION_POLICY_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        push_bytes(
            &mut out,
            "consumption_marker_namespace",
            CONSUMPTION_MARKER_NAMESPACE_V1.as_bytes(),
        )?;
        // Stable v1 flags: atomic, create-only, no-delete, no-rewrite, no-deref.
        out.extend_from_slice(&[1, 1, 0, 0, 1]);
        Ok(out)
    }

    /// Stable commitment to the exact v1 execution policy.
    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, ProtectedRefExecutionPolicyError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

/// Stable identity of the implementation/profile that verifies whether an
/// external repository policy subject enforces [`ProtectedRefExecutionPolicyV1`].
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct RepositoryExecutionPolicyVerifierIdentityV1 {
    name: String,
    version: String,
}

impl RepositoryExecutionPolicyVerifierIdentityV1 {
    /// Construct a validated verifier identity.
    pub fn new(
        name: impl Into<String>,
        version: impl Into<String>,
    ) -> Result<Self, ProtectedRefExecutionPolicyError> {
        let name = name.into();
        let version = version.into();
        validate_verifier_field("name", &name)?;
        validate_verifier_field("version", &version)?;
        Ok(Self { name, version })
    }

    /// Verifier name/profile.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Verifier version/profile revision.
    pub fn version(&self) -> &str {
        &self.version
    }
}

impl<'de> Deserialize<'de> for RepositoryExecutionPolicyVerifierIdentityV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireIdentity {
            name: String,
            version: String,
        }

        let wire = WireIdentity::deserialize(deserializer)?;
        Self::new(wire.name, wire.version).map_err(D::Error::custom)
    }
}

/// Raw provider observation claiming that one exact external repository policy
/// subject enforces the typed protected-ref execution profile.
///
/// This type is deserializable and therefore non-authoritative until accepted
/// by a concrete [`RepositoryExecutionPolicyVerifierV1`].
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct RepositoryExecutionPolicyObservationV1 {
    version: ProtocolVersion,
    verifier: RepositoryExecutionPolicyVerifierIdentityV1,
    repository_policy_state: Digest,
    external_policy_subject: Digest,
    execution_policy: Digest,
    provider_evidence: Digest,
}

impl RepositoryExecutionPolicyObservationV1 {
    /// Construct one raw execution-policy observation.
    pub fn new(
        verifier: RepositoryExecutionPolicyVerifierIdentityV1,
        repository_policy_state: Digest,
        external_policy_subject: Digest,
        execution_policy: Digest,
        provider_evidence: Digest,
    ) -> Self {
        Self {
            version: ProtocolVersion::CURRENT,
            verifier,
            repository_policy_state,
            external_policy_subject,
            execution_policy,
            provider_evidence,
        }
    }

    /// Claimed verifier identity.
    pub fn verifier(&self) -> &RepositoryExecutionPolicyVerifierIdentityV1 {
        &self.verifier
    }

    /// Exact monotonic repository-policy state claimed to have been inspected.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    /// External policy subject already committed by
    /// `RepositoryPolicyState::policy_digest()`.
    pub fn external_policy_subject(&self) -> &Digest {
        &self.external_policy_subject
    }

    /// Typed protected-ref execution-policy commitment claimed to be enforced.
    pub fn execution_policy(&self) -> &Digest {
        &self.execution_policy
    }

    /// Provider-specific evidence commitment for the enforcement claim.
    pub fn provider_evidence(&self) -> &Digest {
        &self.provider_evidence
    }

    /// Canonical v1 observation bytes.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ProtectedRefExecutionPolicyError> {
        ensure_v1(self.version)?;
        validate_verifier_field("name", self.verifier.name())?;
        validate_verifier_field("version", self.verifier.version())?;
        let mut out = Vec::new();
        out.extend_from_slice(EXECUTION_POLICY_OBSERVATION_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_bytes(&mut out, "verifier_name", self.verifier.name().as_bytes())?;
        push_bytes(
            &mut out,
            "verifier_version",
            self.verifier.version().as_bytes(),
        )?;
        push_digest(&mut out, &self.repository_policy_state)?;
        push_digest(&mut out, &self.external_policy_subject)?;
        push_digest(&mut out, &self.execution_policy)?;
        push_digest(&mut out, &self.provider_evidence)?;
        Ok(out)
    }

    /// Stable observation commitment.
    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, ProtectedRefExecutionPolicyError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for RepositoryExecutionPolicyObservationV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireObservation {
            version: ProtocolVersion,
            verifier: RepositoryExecutionPolicyVerifierIdentityV1,
            repository_policy_state: Digest,
            external_policy_subject: Digest,
            execution_policy: Digest,
            provider_evidence: Digest,
        }

        let wire = WireObservation::deserialize(deserializer)?;
        ensure_v1(wire.version).map_err(D::Error::custom)?;
        Ok(Self::new(
            wire.verifier,
            wire.repository_policy_state,
            wire.external_policy_subject,
            wire.execution_policy,
            wire.provider_evidence,
        ))
    }
}

/// Provider-specific repository-policy verification failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum RepositoryExecutionPolicyVerifierErrorV1 {
    /// External policy evidence did not prove the typed execution profile.
    #[error("repository execution-policy verifier rejected provider evidence")]
    Rejected,
}

/// Independent verifier for one external repository-policy implementation.
pub trait RepositoryExecutionPolicyVerifierV1 {
    /// Stable verifier identity/profile.
    fn identity(&self) -> RepositoryExecutionPolicyVerifierIdentityV1;

    /// Verify that the exact external policy subject committed by
    /// `repository_policy_state` enforces the exact typed execution profile.
    fn verify_execution_policy(
        &self,
        policy: &ProtectedRefExecutionPolicyV1,
        repository_policy_state: &RepositoryPolicyState,
        observation: &RepositoryExecutionPolicyObservationV1,
    ) -> Result<Digest, RepositoryExecutionPolicyVerifierErrorV1>;
}

/// Positive provider-verified binding between one external repository policy
/// subject and the typed protected-ref execution profile.
///
/// This type has no `Deserialize` implementation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedProtectedRefExecutionPolicyV1 {
    project: ProjectIdentity,
    repository_policy_state: Digest,
    repository_policy_sequence: u64,
    external_policy_subject: Digest,
    execution_policy: Digest,
    verifier: RepositoryExecutionPolicyVerifierIdentityV1,
    provider_evidence: Digest,
    verifier_evidence: Digest,
    evidence_commitment: Digest,
}

impl QualifiedProtectedRefExecutionPolicyV1 {
    /// Project governed by the qualified execution policy.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact repository-policy state inspected.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    /// Monotonic repository-policy sequence.
    pub const fn repository_policy_sequence(&self) -> u64 {
        self.repository_policy_sequence
    }

    /// Exact external policy subject from `RepositoryPolicyState`.
    pub fn external_policy_subject(&self) -> &Digest {
        &self.external_policy_subject
    }

    /// Exact typed protected-ref execution-policy commitment.
    pub fn execution_policy(&self) -> &Digest {
        &self.execution_policy
    }

    /// Provider/verifier profile that accepted the enforcement evidence.
    pub fn verifier(&self) -> &RepositoryExecutionPolicyVerifierIdentityV1 {
        &self.verifier
    }

    /// Provider-native enforcement evidence commitment.
    pub fn provider_evidence(&self) -> &Digest {
        &self.provider_evidence
    }

    /// Verifier-side evidence commitment.
    pub fn verifier_evidence(&self) -> &Digest {
        &self.verifier_evidence
    }

    /// Aggregate qualification evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Qualify one exact external repository-policy subject as enforcing the typed
/// protected-ref execution profile.
pub fn qualify_protected_ref_execution_policy_v1<V: RepositoryExecutionPolicyVerifierV1>(
    policy: &ProtectedRefExecutionPolicyV1,
    repository_policy_state: &RepositoryPolicyState,
    observation: &RepositoryExecutionPolicyObservationV1,
    verifier: &V,
) -> Result<QualifiedProtectedRefExecutionPolicyV1, ProtectedRefExecutionPolicyError> {
    if policy.project() != repository_policy_state.project() {
        return Err(ProtectedRefExecutionPolicyError::ProjectMismatch);
    }
    if observation.verifier() != &verifier.identity() {
        return Err(ProtectedRefExecutionPolicyError::VerifierIdentityMismatch);
    }

    let state_algorithm = observation.repository_policy_state().algorithm();
    let expected_state = repository_policy_state.digest(state_algorithm)?;
    if observation.repository_policy_state() != &expected_state {
        return Err(ProtectedRefExecutionPolicyError::RepositoryPolicyStateMismatch);
    }
    if observation.external_policy_subject() != repository_policy_state.policy_digest() {
        return Err(ProtectedRefExecutionPolicyError::ExternalPolicySubjectMismatch);
    }

    let execution_algorithm = observation.execution_policy().algorithm();
    let expected_execution_policy = policy.digest(execution_algorithm)?;
    if observation.execution_policy() != &expected_execution_policy {
        return Err(ProtectedRefExecutionPolicyError::ExecutionPolicyMismatch);
    }

    let verifier_evidence =
        verifier.verify_execution_policy(policy, repository_policy_state, observation)?;
    let algorithm = verifier_evidence.algorithm();
    let repository_policy_state_digest = repository_policy_state.digest(algorithm)?;
    let execution_policy = policy.digest(algorithm)?;
    let observation_digest = observation.digest(algorithm)?;

    let mut out = Vec::new();
    out.extend_from_slice(QUALIFIED_EXECUTION_POLICY_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, policy.project())?;
    push_digest(&mut out, &repository_policy_state_digest)?;
    out.extend_from_slice(&repository_policy_state.sequence().to_be_bytes());
    push_digest(&mut out, repository_policy_state.policy_digest())?;
    push_digest(&mut out, &execution_policy)?;
    push_digest(&mut out, &observation_digest)?;
    push_digest(&mut out, observation.provider_evidence())?;
    push_digest(&mut out, &verifier_evidence)?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(QualifiedProtectedRefExecutionPolicyV1 {
        project: policy.project().clone(),
        repository_policy_state: repository_policy_state_digest,
        repository_policy_sequence: repository_policy_state.sequence(),
        external_policy_subject: repository_policy_state.policy_digest().clone(),
        execution_policy,
        verifier: observation.verifier().clone(),
        provider_evidence: observation.provider_evidence().clone(),
        verifier_evidence,
        evidence_commitment,
    })
}

/// Positive structural qualification of one exact FORGE-009F transaction plan
/// under a provider-verified protected-ref execution policy.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct PolicyQualifiedGitRefTransactionPlanV1 {
    project: ProjectIdentity,
    repository_policy_state: Digest,
    external_policy_subject: Digest,
    execution_policy: Digest,
    qualified_policy: Digest,
    atomic_intent: Digest,
    plan_evidence: Digest,
    target_ref: RepositoryRef,
    consumption_marker_ref: RepositoryRef,
    evidence_commitment: Digest,
}

impl PolicyQualifiedGitRefTransactionPlanV1 {
    /// Project containing the transaction.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact repository-policy state under which the plan was qualified.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    /// Exact external repository policy subject proven to enforce the profile.
    pub fn external_policy_subject(&self) -> &Digest {
        &self.external_policy_subject
    }

    /// Exact typed protected-ref execution-policy commitment.
    pub fn execution_policy(&self) -> &Digest {
        &self.execution_policy
    }

    /// Exact provider-verified execution-policy evidence consumed here.
    pub fn qualified_policy(&self) -> &Digest {
        &self.qualified_policy
    }

    /// Exact FORGE-009E atomic intent committed by the Git plan.
    pub fn atomic_intent(&self) -> &Digest {
        &self.atomic_intent
    }

    /// Exact FORGE-009F plan evidence commitment.
    pub fn plan_evidence(&self) -> &Digest {
        &self.plan_evidence
    }

    /// Protected target ref, guaranteed to be outside the marker namespace.
    pub fn target_ref(&self) -> &RepositoryRef {
        &self.target_ref
    }

    /// Marker ref, guaranteed to be inside the reserved namespace.
    pub fn consumption_marker_ref(&self) -> &RepositoryRef {
        &self.consumption_marker_ref
    }

    /// Aggregate policy-qualified plan evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Qualify one exact Git transaction plan against provider-verified repository
/// execution policy.
pub fn qualify_git_ref_transaction_plan_policy_v1(
    qualified_policy: &QualifiedProtectedRefExecutionPolicyV1,
    plan: &GitRefTransactionPlanV1,
) -> Result<PolicyQualifiedGitRefTransactionPlanV1, ProtectedRefExecutionPolicyError> {
    validate_plan_surface(
        plan.target_ref(),
        plan.consumption_marker_ref(),
        plan.command_args(),
    )?;

    let algorithm = qualified_policy.execution_policy().algorithm();
    let mut out = Vec::new();
    out.extend_from_slice(POLICY_QUALIFIED_PLAN_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, qualified_policy.project())?;
    push_digest(&mut out, qualified_policy.repository_policy_state())?;
    push_digest(&mut out, qualified_policy.external_policy_subject())?;
    push_digest(&mut out, qualified_policy.execution_policy())?;
    push_digest(&mut out, qualified_policy.evidence_commitment())?;
    push_digest(&mut out, plan.atomic_intent())?;
    push_digest(&mut out, plan.evidence_commitment())?;
    push_ref(&mut out, plan.target_ref())?;
    push_ref(&mut out, plan.consumption_marker_ref())?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(PolicyQualifiedGitRefTransactionPlanV1 {
        project: qualified_policy.project().clone(),
        repository_policy_state: qualified_policy.repository_policy_state().clone(),
        external_policy_subject: qualified_policy.external_policy_subject().clone(),
        execution_policy: qualified_policy.execution_policy().clone(),
        qualified_policy: qualified_policy.evidence_commitment().clone(),
        atomic_intent: plan.atomic_intent().clone(),
        plan_evidence: plan.evidence_commitment().clone(),
        target_ref: plan.target_ref().clone(),
        consumption_marker_ref: plan.consumption_marker_ref().clone(),
        evidence_commitment,
    })
}

fn validate_plan_surface(
    target_ref: &RepositoryRef,
    marker_ref: &RepositoryRef,
    command_args: &[&str],
) -> Result<(), ProtectedRefExecutionPolicyError> {
    if target_ref.as_str().starts_with(CONSUMPTION_MARKER_NAMESPACE_V1) {
        return Err(ProtectedRefExecutionPolicyError::TargetInReservedNamespace);
    }
    if !marker_ref.as_str().starts_with(CONSUMPTION_MARKER_NAMESPACE_V1) {
        return Err(ProtectedRefExecutionPolicyError::MarkerOutsideReservedNamespace);
    }
    if target_ref == marker_ref {
        return Err(ProtectedRefExecutionPolicyError::MarkerAliasesTargetRef);
    }
    if command_args != GIT_UPDATE_REF_TRANSACTION_ARGS {
        return Err(ProtectedRefExecutionPolicyError::UnexpectedGitTransactionArgs);
    }
    Ok(())
}

fn ensure_v1(version: ProtocolVersion) -> Result<(), ProtectedRefExecutionPolicyError> {
    if version.get() == CURRENT_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ProtectedRefExecutionPolicyError::UnsupportedProtocolVersion(
            version.get(),
        ))
    }
}

fn validate_verifier_field(
    field: &'static str,
    value: &str,
) -> Result<(), ProtectedRefExecutionPolicyError> {
    let len = value.len();
    if len == 0 || len > MAX_VERIFIER_FIELD_LEN {
        return Err(ProtectedRefExecutionPolicyError::InvalidVerifierField { field, len });
    }
    Ok(())
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), ProtectedRefExecutionPolicyError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_ref(
    out: &mut Vec<u8>,
    reference: &RepositoryRef,
) -> Result<(), ProtectedRefExecutionPolicyError> {
    push_bytes(out, "repository_ref", reference.as_str().as_bytes())
}

fn push_digest(
    out: &mut Vec<u8>,
    digest: &Digest,
) -> Result<(), ProtectedRefExecutionPolicyError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ProtectedRefExecutionPolicyError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        ProtectedRefExecutionPolicyError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Protected-ref execution-policy failures.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum ProtectedRefExecutionPolicyError {
    /// Typed policy and repository-policy state belong to different projects.
    #[error("protected-ref execution policy project mismatch")]
    ProjectMismatch,
    /// Raw policy observation belongs to another repository-policy state.
    #[error("repository execution-policy observation names another policy state")]
    RepositoryPolicyStateMismatch,
    /// Raw observation names another external policy subject.
    #[error("repository execution-policy observation names another external policy subject")]
    ExternalPolicySubjectMismatch,
    /// Raw observation names another typed execution profile.
    #[error("repository execution-policy observation names another execution policy")]
    ExecutionPolicyMismatch,
    /// Raw observation verifier identity differs from supplied verifier.
    #[error("repository execution-policy verifier identity mismatch")]
    VerifierIdentityMismatch,
    /// A normal protected target attempts to live inside the durable marker namespace.
    #[error("protected target ref is inside the reserved consumption-marker namespace")]
    TargetInReservedNamespace,
    /// Consumption marker is not inside the exact reserved namespace.
    #[error("consumption marker is outside the reserved namespace")]
    MarkerOutsideReservedNamespace,
    /// Consumption marker aliases the protected target ref.
    #[error("consumption marker aliases protected target ref")]
    MarkerAliasesTargetRef,
    /// Git transaction argv differs from the fixed no-deref profile.
    #[error("Git ref transaction args differ from the protected execution profile")]
    UnexpectedGitTransactionArgs,
    /// Invalid verifier identity field.
    #[error("invalid execution-policy verifier field {field}: length {len}")]
    InvalidVerifierField {
        /// Field name.
        field: &'static str,
        /// Observed byte length.
        len: usize,
    },
    /// Unsupported protocol version.
    #[error("unsupported protected-ref execution-policy protocol version: {0}")]
    UnsupportedProtocolVersion(u16),
    /// Canonical field exceeded v1 bounds.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Actual byte length.
        len: usize,
        /// Maximum encodable length.
        max: usize,
    },
    /// Repository-policy canonicalization failed.
    #[error(transparent)]
    Repository(#[from] RepositoryVerificationError),
    /// Provider-specific execution-policy verification failed.
    #[error(transparent)]
    Verifier(#[from] RepositoryExecutionPolicyVerifierErrorV1),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn project(seed: u8) -> ProjectIdentity {
        ProjectIdentity::derive(
            &ProjectIdentitySeed::new([seed; GENESIS_NONCE_LEN], digest(seed.wrapping_add(1))),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn verifier_identity() -> RepositoryExecutionPolicyVerifierIdentityV1 {
        RepositoryExecutionPolicyVerifierIdentityV1::new("external-policy-verifier", "0.1.0")
            .unwrap()
    }

    struct ExactVerifier;

    impl RepositoryExecutionPolicyVerifierV1 for ExactVerifier {
        fn identity(&self) -> RepositoryExecutionPolicyVerifierIdentityV1 {
            verifier_identity()
        }

        fn verify_execution_policy(
            &self,
            _policy: &ProtectedRefExecutionPolicyV1,
            repository_policy_state: &RepositoryPolicyState,
            observation: &RepositoryExecutionPolicyObservationV1,
        ) -> Result<Digest, RepositoryExecutionPolicyVerifierErrorV1> {
            if observation.external_policy_subject() == repository_policy_state.policy_digest()
                && observation.provider_evidence() == &digest(0x70)
            {
                Ok(digest(0x71))
            } else {
                Err(RepositoryExecutionPolicyVerifierErrorV1::Rejected)
            }
        }
    }

    #[test]
    fn external_policy_subject_and_typed_execution_policy_remain_distinct() {
        let project = project(0x11);
        let policy = ProtectedRefExecutionPolicyV1::new(project.clone());
        let external_subject = digest(0x60);
        let state = RepositoryPolicyState::new(project, 0, None, external_subject.clone()).unwrap();
        let state_digest = state.digest(DigestAlgorithm::Sha256).unwrap();
        let execution_policy = policy.digest(DigestAlgorithm::Sha256).unwrap();
        assert_ne!(external_subject, execution_policy);

        let observation = RepositoryExecutionPolicyObservationV1::new(
            verifier_identity(),
            state_digest,
            external_subject,
            execution_policy,
            digest(0x70),
        );
        let qualified = qualify_protected_ref_execution_policy_v1(
            &policy,
            &state,
            &observation,
            &ExactVerifier,
        )
        .unwrap();
        assert_eq!(qualified.external_policy_subject(), state.policy_digest());
    }

    #[test]
    fn unrelated_execution_policy_cannot_borrow_external_policy_evidence() {
        let project = project(0x21);
        let policy = ProtectedRefExecutionPolicyV1::new(project.clone());
        let state = RepositoryPolicyState::new(project, 0, None, digest(0x61)).unwrap();
        let observation = RepositoryExecutionPolicyObservationV1::new(
            verifier_identity(),
            state.digest(DigestAlgorithm::Sha256).unwrap(),
            state.policy_digest().clone(),
            digest(0x99),
            digest(0x70),
        );
        assert_eq!(
            qualify_protected_ref_execution_policy_v1(
                &policy,
                &state,
                &observation,
                &ExactVerifier,
            )
            .unwrap_err(),
            ProtectedRefExecutionPolicyError::ExecutionPolicyMismatch
        );
    }

    #[test]
    fn target_inside_consumption_namespace_is_rejected() {
        let target = RepositoryRef::new(
            "refs/mycelix/forge/consumed/sha256/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
        )
        .unwrap();
        let marker = RepositoryRef::new(
            "refs/mycelix/forge/consumed/sha256/bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
        )
        .unwrap();
        assert_eq!(
            validate_plan_surface(&target, &marker, GIT_UPDATE_REF_TRANSACTION_ARGS).unwrap_err(),
            ProtectedRefExecutionPolicyError::TargetInReservedNamespace
        );
    }

    #[test]
    fn marker_outside_reserved_namespace_is_rejected() {
        let target = RepositoryRef::new("refs/heads/main").unwrap();
        let marker = RepositoryRef::new("refs/heads/not-a-consumption-marker").unwrap();
        assert_eq!(
            validate_plan_surface(&target, &marker, GIT_UPDATE_REF_TRANSACTION_ARGS).unwrap_err(),
            ProtectedRefExecutionPolicyError::MarkerOutsideReservedNamespace
        );
    }

    #[test]
    fn exact_git_transaction_args_are_required() {
        let target = RepositoryRef::new("refs/heads/main").unwrap();
        let marker = RepositoryRef::new(
            "refs/mycelix/forge/consumed/sha256/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
        )
        .unwrap();
        assert_eq!(
            validate_plan_surface(&target, &marker, &["update-ref", "--stdin"]).unwrap_err(),
            ProtectedRefExecutionPolicyError::UnexpectedGitTransactionArgs
        );
    }

    #[allow(dead_code)]
    fn public_plan_qualifier_requires_positive_git_plan(
        qualified: &QualifiedProtectedRefExecutionPolicyV1,
        plan: &GitRefTransactionPlanV1,
    ) {
        let _ = qualify_git_ref_transaction_plan_policy_v1(qualified, plan);
    }
}
