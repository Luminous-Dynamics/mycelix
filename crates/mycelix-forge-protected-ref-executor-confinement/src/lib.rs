// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-009I: close the executor-confinement gap between repository policy
//! authorization and the exact FORGE-009F Git transaction plan.
//!
//! The key theorem is intentionally closed-world:
//!
//! ```text
//! every principal authorized for the exact marker ref by FORGE-009H
//! == every principal bound to an executor capability
//! each executor capability
//!     == the exact FORGE-009F plan
//! independent verifier accepts every binding
//!     -> ExecutorConstrainedGitRefTransactionPlanV1
//! ```
//!
//! This proves a typed policy/verifier binding. It does not prove that an
//! operating system, host, or compromised executor cannot bypass its declared
//! interface; that requires a later sealed-executor enforcement theorem.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use forge_git_ref_transaction::GitRefTransactionPlanV1;
use forge_gittuf_marker_policy::GittufMarkerNamespacePolicyEvidenceV1;
use mycelix_forge_core::{Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion};
use mycelix_forge_protected_ref_execution_policy::PolicyQualifiedGitRefTransactionPlanV1;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use thiserror::Error;

const EXECUTOR_CONFINEMENT_DOMAIN_V1: &[u8] =
    b"mycelix-forge/protected-ref-executor-confinement/v1\0";
const OBSERVATION_DOMAIN_V1: &[u8] =
    b"mycelix-forge/protected-ref-executor-confinement-observation/v1\0";
const MAX_PRINCIPAL_LEN: usize = 512;
const MAX_EXECUTOR_BINDINGS: usize = 1024;

/// One principal-to-executor binding claimed by an external executor policy.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExecutorConfinementBindingV1 {
    principal_id: String,
    executor_identity: Digest,
    allowed_plan: Digest,
}

impl ExecutorConfinementBindingV1 {
    /// Construct one raw principal-to-executor binding.
    pub fn new(
        principal_id: impl Into<String>,
        executor_identity: Digest,
        allowed_plan: Digest,
    ) -> Result<Self, ExecutorConfinementError> {
        let principal_id = principal_id.into();
        if principal_id.is_empty() || principal_id.len() > MAX_PRINCIPAL_LEN {
            return Err(ExecutorConfinementError::InvalidPrincipalId);
        }
        Ok(Self {
            principal_id,
            executor_identity,
            allowed_plan,
        })
    }

    /// Opaque gittuf principal ID being confined.
    pub fn principal_id(&self) -> &str {
        &self.principal_id
    }

    /// Exact executor identity commitment.
    pub fn executor_identity(&self) -> &Digest {
        &self.executor_identity
    }

    /// Exact plan commitment this executor is permitted to execute.
    pub fn allowed_plan(&self) -> &Digest {
        &self.allowed_plan
    }
}

/// Stable identity of the independent executor-confinement verifier.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExecutorConfinementVerifierIdentityV1 {
    name: String,
    version: String,
}

impl ExecutorConfinementVerifierIdentityV1 {
    /// Construct a validated verifier identity.
    pub fn new(
        name: impl Into<String>,
        version: impl Into<String>,
    ) -> Result<Self, ExecutorConfinementError> {
        let name = name.into();
        let version = version.into();
        if name.is_empty()
            || name.len() > MAX_PRINCIPAL_LEN
            || version.is_empty()
            || version.len() > MAX_PRINCIPAL_LEN
        {
            return Err(ExecutorConfinementError::InvalidVerifierIdentity);
        }
        Ok(Self { name, version })
    }

    /// Verifier name/profile.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Verifier version/profile.
    pub fn version(&self) -> &str {
        &self.version
    }
}

/// Raw provider evidence claiming that every applicable marker principal is
/// confined to one exact Git transaction plan.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExecutorConfinementObservationV1 {
    version: ProtocolVersion,
    verifier: ExecutorConfinementVerifierIdentityV1,
    marker_policy_evidence: Digest,
    execution_policy: Digest,
    plan_evidence: Digest,
    bindings: Vec<ExecutorConfinementBindingV1>,
    provider_evidence: Digest,
}

impl ExecutorConfinementObservationV1 {
    /// Construct raw, non-authoritative executor-confinement evidence.
    pub fn new(
        verifier: ExecutorConfinementVerifierIdentityV1,
        marker_policy_evidence: Digest,
        execution_policy: Digest,
        plan_evidence: Digest,
        bindings: Vec<ExecutorConfinementBindingV1>,
        provider_evidence: Digest,
    ) -> Result<Self, ExecutorConfinementError> {
        validate_bindings(&bindings)?;
        Ok(Self {
            version: ProtocolVersion::CURRENT,
            verifier,
            marker_policy_evidence,
            execution_policy,
            plan_evidence,
            bindings,
            provider_evidence,
        })
    }

    /// Claimed verifier identity.
    pub fn verifier(&self) -> &ExecutorConfinementVerifierIdentityV1 {
        &self.verifier
    }

    /// Exact FORGE-009H marker-policy evidence commitment.
    pub fn marker_policy_evidence(&self) -> &Digest {
        &self.marker_policy_evidence
    }

    /// Exact typed execution-policy commitment.
    pub fn execution_policy(&self) -> &Digest {
        &self.execution_policy
    }

    /// Exact FORGE-009F plan evidence commitment.
    pub fn plan_evidence(&self) -> &Digest {
        &self.plan_evidence
    }

    /// All principal-to-executor bindings.
    pub fn bindings(&self) -> &[ExecutorConfinementBindingV1] {
        &self.bindings
    }

    /// Provider-native confinement evidence commitment.
    pub fn provider_evidence(&self) -> &Digest {
        &self.provider_evidence
    }

    /// Canonical observation commitment.
    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, ExecutorConfinementError> {
        let mut out = Vec::new();
        out.extend_from_slice(OBSERVATION_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_string(&mut out, &self.verifier.name())?;
        push_string(&mut out, &self.verifier.version())?;
        push_digest(&mut out, &self.marker_policy_evidence)?;
        push_digest(&mut out, &self.execution_policy)?;
        push_digest(&mut out, &self.plan_evidence)?;
        let count = u16::try_from(self.bindings.len())
            .map_err(|_| ExecutorConfinementError::TooManyBindings)?;
        out.extend_from_slice(&count.to_be_bytes());
        for binding in &self.bindings {
            push_string(&mut out, binding.principal_id())?;
            push_digest(&mut out, binding.executor_identity())?;
            push_digest(&mut out, binding.allowed_plan())?;
        }
        push_digest(&mut out, &self.provider_evidence)?;
        Ok(Digest::of_bytes(algorithm, &out))
    }
}

/// Independent verifier for the executor-confinement evidence.
pub trait ExecutorConfinementVerifierV1 {
    /// Stable verifier identity.
    fn identity(&self) -> ExecutorConfinementVerifierIdentityV1;

    /// Verify that every supplied binding is enforced by the named executor
    /// implementation and is restricted to the exact supplied plan.
    fn verify_executor_confinement(
        &self,
        marker_policy: &GittufMarkerNamespacePolicyEvidenceV1,
        qualified_plan: &PolicyQualifiedGitRefTransactionPlanV1,
        plan: &GitRefTransactionPlanV1,
        observation: &ExecutorConfinementObservationV1,
    ) -> Result<Digest, ExecutorConfinementVerifierErrorV1>;
}

/// Provider-specific executor-confinement verification failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ExecutorConfinementVerifierErrorV1 {
    /// Provider evidence did not prove the declared confinement.
    #[error("executor-confinement verifier rejected provider evidence")]
    Rejected,
}

/// Positive binding proving the exact gittuf-authorized principal set is
/// confined to the exact FORGE-009F Git transaction plan.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ExecutorConstrainedGitRefTransactionPlanV1 {
    project: ProjectIdentity,
    marker_policy_evidence: Digest,
    execution_policy: Digest,
    qualified_plan: Digest,
    plan_evidence: Digest,
    bindings: Vec<ExecutorConfinementBindingV1>,
    verifier: ExecutorConfinementVerifierIdentityV1,
    provider_evidence: Digest,
    verifier_evidence: Digest,
    evidence_commitment: Digest,
}

impl ExecutorConstrainedGitRefTransactionPlanV1 {
    /// Exact project.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }
    /// Exact FORGE-009H marker-policy evidence.
    pub fn marker_policy_evidence(&self) -> &Digest {
        &self.marker_policy_evidence
    }
    /// Exact typed execution-policy commitment.
    pub fn execution_policy(&self) -> &Digest {
        &self.execution_policy
    }
    /// Exact policy-qualified plan evidence.
    pub fn qualified_plan(&self) -> &Digest {
        &self.qualified_plan
    }
    /// Exact FORGE-009F plan evidence.
    pub fn plan_evidence(&self) -> &Digest {
        &self.plan_evidence
    }
    /// Closed-world principal-to-executor bindings.
    pub fn bindings(&self) -> &[ExecutorConfinementBindingV1] {
        &self.bindings
    }
    /// Independent verifier identity.
    pub fn verifier(&self) -> &ExecutorConfinementVerifierIdentityV1 {
        &self.verifier
    }
    /// Provider evidence commitment.
    pub fn provider_evidence(&self) -> &Digest {
        &self.provider_evidence
    }
    /// Verifier evidence commitment.
    pub fn verifier_evidence(&self) -> &Digest {
        &self.verifier_evidence
    }
    /// Aggregate confinement evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Qualify one exact transaction plan against the exact marker policy and an
/// independently verified closed-world executor-confinement observation.
pub fn qualify_executor_constrained_plan_v1<V: ExecutorConfinementVerifierV1>(
    marker_policy: &GittufMarkerNamespacePolicyEvidenceV1,
    qualified_plan: &PolicyQualifiedGitRefTransactionPlanV1,
    plan: &GitRefTransactionPlanV1,
    observation: &ExecutorConfinementObservationV1,
    verifier: &V,
) -> Result<ExecutorConstrainedGitRefTransactionPlanV1, ExecutorConfinementError> {
    let expected_plan = plan.evidence_commitment();
    if qualified_plan.plan_evidence() != expected_plan
        || observation.plan_evidence() != expected_plan
    {
        return Err(ExecutorConfinementError::PlanMismatch);
    }
    if qualified_plan.consumption_marker_ref() != plan.consumption_marker_ref()
        || marker_policy.marker_ref() != plan.consumption_marker_ref()
    {
        return Err(ExecutorConfinementError::MarkerMismatch);
    }
    if qualified_plan.project() != marker_policy.project() {
        return Err(ExecutorConfinementError::ProjectMismatch);
    }
    if qualified_plan.external_policy_subject() != marker_policy.external_policy_subject() {
        return Err(ExecutorConfinementError::ExternalPolicySubjectMismatch);
    }
    if qualified_plan.execution_policy() != observation.execution_policy() {
        return Err(ExecutorConfinementError::ExecutionPolicyMismatch);
    }
    if marker_policy.evidence_commitment() != observation.marker_policy_evidence() {
        return Err(ExecutorConfinementError::MarkerPolicyEvidenceMismatch);
    }
    if observation.verifier() != &verifier.identity() {
        return Err(ExecutorConfinementError::VerifierIdentityMismatch);
    }
    validate_bindings(observation.bindings())?;
    validate_closed_world_principal_set(
        marker_policy.authorized_principal_ids(),
        observation.bindings(),
    )?;
    for binding in observation.bindings() {
        if binding.allowed_plan() != expected_plan {
            return Err(ExecutorConfinementError::BindingPlanMismatch(
                binding.principal_id().to_owned(),
            ));
        }
    }
    let verifier_evidence = verifier
        .verify_executor_confinement(marker_policy, qualified_plan, plan, observation)
        .map_err(ExecutorConfinementError::VerifierRejected)?;
    let algorithm = verifier_evidence.algorithm();
    let observation_digest = observation.digest(algorithm)?;
    let mut out = Vec::new();
    out.extend_from_slice(EXECUTOR_CONFINEMENT_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project(&mut out, marker_policy.project())?;
    push_digest(&mut out, marker_policy.evidence_commitment())?;
    push_digest(&mut out, qualified_plan.execution_policy())?;
    push_digest(&mut out, qualified_plan.evidence_commitment())?;
    push_digest(&mut out, plan.evidence_commitment())?;
    push_digest(&mut out, &observation_digest)?;
    push_digest(&mut out, &verifier_evidence)?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);
    Ok(ExecutorConstrainedGitRefTransactionPlanV1 {
        project: marker_policy.project().clone(),
        marker_policy_evidence: marker_policy.evidence_commitment().clone(),
        execution_policy: qualified_plan.execution_policy().clone(),
        qualified_plan: qualified_plan.evidence_commitment().clone(),
        plan_evidence: plan.evidence_commitment().clone(),
        bindings: observation.bindings().to_vec(),
        verifier: observation.verifier().clone(),
        provider_evidence: observation.provider_evidence().clone(),
        verifier_evidence,
        evidence_commitment,
    })
}

fn validate_bindings(
    bindings: &[ExecutorConfinementBindingV1],
) -> Result<(), ExecutorConfinementError> {
    if bindings.is_empty() || bindings.len() > MAX_EXECUTOR_BINDINGS {
        return Err(ExecutorConfinementError::InvalidBindingCount);
    }
    let mut principals = BTreeSet::new();
    for binding in bindings {
        if binding.principal_id().is_empty() || binding.principal_id().len() > MAX_PRINCIPAL_LEN {
            return Err(ExecutorConfinementError::InvalidPrincipalId);
        }
        if !principals.insert(binding.principal_id()) {
            return Err(ExecutorConfinementError::DuplicatePrincipal);
        }
    }
    Ok(())
}

fn validate_closed_world_principal_set(
    authorized_principals: &[String],
    bindings: &[ExecutorConfinementBindingV1],
) -> Result<(), ExecutorConfinementError> {
    let authorized: BTreeSet<_> = authorized_principals.iter().map(String::as_str).collect();
    let bound: BTreeSet<_> = bindings.iter().map(|binding| binding.principal_id()).collect();
    if authorized != bound {
        return Err(ExecutorConfinementError::PrincipalSetMismatch);
    }
    Ok(())
}

fn push_project(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), ExecutorConfinementError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), ExecutorConfinementError> {
    push_string(out, digest.algorithm().id())?;
    let len = u32::try_from(digest.as_bytes().len())
        .map_err(|_| ExecutorConfinementError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

fn push_string(out: &mut Vec<u8>, value: &str) -> Result<(), ExecutorConfinementError> {
    let bytes = value.as_bytes();
    let len = u32::try_from(bytes.len())
        .map_err(|_| ExecutorConfinementError::CanonicalFieldTooLarge("string"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// FORGE-009I qualification failures.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ExecutorConfinementError {
    /// Plan identity was substituted.
    #[error("executor confinement references a different transaction plan")]
    PlanMismatch,
    /// Project identity was substituted.
    #[error("executor confinement project mismatch")]
    ProjectMismatch,
    /// External repository-policy subject was substituted.
    #[error("executor confinement external policy subject mismatch")]
    ExternalPolicySubjectMismatch,
    /// Marker identity was substituted.
    #[error("executor confinement references a different consumption marker")]
    MarkerMismatch,
    /// Execution policy identity was substituted.
    #[error("executor confinement references a different execution policy")]
    ExecutionPolicyMismatch,
    /// Marker-policy evidence identity was substituted.
    #[error("executor confinement references different marker-policy evidence")]
    MarkerPolicyEvidenceMismatch,
    /// Verifier identity was substituted.
    #[error("executor confinement verifier identity mismatch")]
    VerifierIdentityMismatch,
    /// Authorized and bound principal sets differ.
    #[error("executor confinement principal set is not closed over gittuf authorization")]
    PrincipalSetMismatch,
    /// A principal was bound to another plan.
    #[error("principal {0} is bound to another transaction plan")]
    BindingPlanMismatch(String),
    /// Provider verifier rejected the evidence.
    #[error("executor-confinement verifier rejected provider evidence: {0}")]
    VerifierRejected(ExecutorConfinementVerifierErrorV1),
    /// Invalid binding count.
    #[error("executor confinement binding count is invalid")]
    InvalidBindingCount,
    /// Principal ID is empty or too large.
    #[error("executor principal ID is invalid")]
    InvalidPrincipalId,
    /// Duplicate principal ID.
    #[error("executor confinement contains a duplicate principal")]
    DuplicatePrincipal,
    /// Canonical field overflow.
    #[error("canonical field too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
    /// Canonical digest failure.
    #[error("digest canonicalization failed: {0}")]
    Digest(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    struct AcceptingVerifier;

    impl ExecutorConfinementVerifierV1 for AcceptingVerifier {
        fn identity(&self) -> ExecutorConfinementVerifierIdentityV1 {
            ExecutorConfinementVerifierIdentityV1::new("test-verifier", "v1").unwrap()
        }

        fn verify_executor_confinement(
            &self,
            _marker_policy: &GittufMarkerNamespacePolicyEvidenceV1,
            _qualified_plan: &PolicyQualifiedGitRefTransactionPlanV1,
            _plan: &GitRefTransactionPlanV1,
            observation: &ExecutorConfinementObservationV1,
        ) -> Result<Digest, ExecutorConfinementVerifierErrorV1> {
            Ok(observation.provider_evidence().clone())
        }
    }

    #[test]
    fn binding_allows_shared_executor_identity_across_distinct_principals() {
        let b1 = ExecutorConfinementBindingV1::new("p1", digest(1), digest(9)).unwrap();
        let b2 = ExecutorConfinementBindingV1::new("p2", digest(1), digest(9)).unwrap();
        assert!(validate_bindings(&[b1, b2]).is_ok());
    }

    #[test]
    fn raw_observation_round_trips_without_becoming_positive() {
        let binding = ExecutorConfinementBindingV1::new("p1", digest(1), digest(2)).unwrap();
        let obs = ExecutorConfinementObservationV1::new(
            ExecutorConfinementVerifierIdentityV1::new("v", "1").unwrap(),
            digest(3), digest(4), digest(5), vec![binding], digest(6),
        ).unwrap();
        let bytes = serde_json::to_vec(&obs).unwrap();
        let decoded: ExecutorConfinementObservationV1 = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(obs, decoded);
    }

    #[test]
    fn closed_world_principal_set_rejects_missing_or_extra_principals() {
        let bindings = vec![
            ExecutorConfinementBindingV1::new("p1", digest(1), digest(9)).unwrap(),
            ExecutorConfinementBindingV1::new("p2", digest(2), digest(9)).unwrap(),
        ];
        let authorized = vec!["p1".to_owned(), "p2".to_owned()];
        assert!(validate_closed_world_principal_set(&authorized, &bindings).is_ok());

        let missing = vec!["p1".to_owned()];
        assert_eq!(
            validate_closed_world_principal_set(&missing, &bindings),
            Err(ExecutorConfinementError::PrincipalSetMismatch)
        );

        let extra = vec!["p1".to_owned(), "p2".to_owned(), "p3".to_owned()];
        assert_eq!(
            validate_closed_world_principal_set(&extra, &bindings),
            Err(ExecutorConfinementError::PrincipalSetMismatch)
        );
    }
}
