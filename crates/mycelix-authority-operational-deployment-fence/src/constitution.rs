// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Final binding-constitution deployment evidence.
//!
//! The semantic/currentness stack has already run before this layer. This module
//! turns the final binding-constitution re-observation into a non-deserializable
//! deployment context, then binds that exact dynamic context into deployment
//! evidence without changing stable deployment authority identity.

use super::*;
use mycelix_authority_evidence_lease::QualifiedEvidenceLeaseManifest;
use mycelix_authority_state_bootstrap_root::{
    QualifiedAuthorityStateBootstrapRoot, VerifiedCurrentConstitutionReceipt,
    ROOT_QUALIFICATION_PROFILE,
};
use serde::Serialize;
use std::fmt;

pub const BINDING_CONSTITUTION_CONTEXT_PROFILE: &str =
    "mycelix-binding-constitution-deployment-context-v1-blake3-framed";
pub const CONSTITUTION_BOUND_DEPLOYMENT_EVIDENCE_PROFILE: &str =
    "mycelix-authority-operational-deployment-evidence-constitution-v1-blake3-framed";

const DOMAIN_BINDING_CONSTITUTION_CONTEXT: &[u8] =
    b"mycelix/authority/binding-constitution-deployment-context/v1";
const DOMAIN_CONSTITUTION_BOUND_DEPLOYMENT_EVIDENCE: &[u8] =
    b"mycelix/authority/operational-deployment-evidence-constitution/v1";

/// Non-deserializable proof that one exact current-constitution receipt matches
/// the exact non-deserializable bootstrap root already used by the semantic
/// currentness stack and remains live for deployment qualification.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedBindingConstitutionContext {
    dna_hash: String,
    statement_digest: Digest32,
    statement_profile: String,
    root_qualification_digest: Digest32,
    root_qualification_profile: String,
    verification_ref: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
    context_digest: Digest32,
    context_profile: String,
}

impl QualifiedBindingConstitutionContext {
    pub fn dna_hash(&self) -> &str {
        &self.dna_hash
    }

    pub fn statement_digest(&self) -> Digest32 {
        self.statement_digest
    }

    pub fn statement_profile(&self) -> &str {
        &self.statement_profile
    }

    pub fn root_qualification_digest(&self) -> Digest32 {
        self.root_qualification_digest
    }

    pub fn root_qualification_profile(&self) -> &str {
        &self.root_qualification_profile
    }

    pub fn verification_ref(&self) -> &str {
        &self.verification_ref
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub fn context_digest(&self) -> Digest32 {
        self.context_digest
    }

    pub fn context_profile(&self) -> &str {
        &self.context_profile
    }

    fn validate_at(&self, now_ms: u64) -> Result<(), BindingConstitutionDeploymentError> {
        validate_dna_hash(&self.dna_hash)
            .map_err(BindingConstitutionDeploymentError::BaseDeploymentDenied)?;
        if self.statement_digest.is_zero()
            || self.statement_profile != STATEMENT_PROFILE
            || self.root_qualification_digest.is_zero()
            || self.root_qualification_profile != ROOT_QUALIFICATION_PROFILE
        {
            return Err(BindingConstitutionDeploymentError::InvalidConstitutionContext);
        }
        if self.verification_ref.trim().is_empty()
            || now_ms == 0
            || self.verified_at_ms == 0
            || self.verified_at_ms > now_ms
            || self.valid_until_ms <= now_ms
            || self.valid_until_ms <= self.verified_at_ms
        {
            return Err(BindingConstitutionDeploymentError::InvalidConstitutionContext);
        }
        let recomputed = binding_constitution_context_digest(
            &self.dna_hash,
            self.statement_digest,
            &self.statement_profile,
            self.root_qualification_digest,
            &self.root_qualification_profile,
            &self.verification_ref,
            self.verified_at_ms,
            self.valid_until_ms,
        );
        if recomputed != self.context_digest
            || self.context_profile != BINDING_CONSTITUTION_CONTEXT_PROFILE
        {
            return Err(BindingConstitutionDeploymentError::InvalidConstitutionContext);
        }
        Ok(())
    }
}

/// Locally qualify the final current-constitution receipt against the exact
/// non-deserializable bootstrap root already consumed by the currentness stack.
pub fn qualify_binding_constitution_context(
    current: &VerifiedCurrentConstitutionReceipt,
    rooted: &QualifiedAuthorityStateBootstrapRoot,
    now_ms: u64,
) -> Result<QualifiedBindingConstitutionContext, BindingConstitutionDeploymentError> {
    current
        .validate_at(now_ms)
        .map_err(|_| BindingConstitutionDeploymentError::CurrentConstitutionReceiptDenied)?;

    if rooted.current_constitution_digest().is_zero()
        || rooted.qualification_digest().is_zero()
        || rooted.qualification_profile() != ROOT_QUALIFICATION_PROFILE
        || current.statement_digest != rooted.current_constitution_digest()
        || current.statement_profile != STATEMENT_PROFILE
        || rooted.manifest().constitution_statement_digest != current.statement_digest
        || rooted.manifest().constitution_statement_profile != current.statement_profile
    {
        return Err(BindingConstitutionDeploymentError::ConstitutionEpochMismatch);
    }

    let context_digest = binding_constitution_context_digest(
        &current.dna_hash,
        current.statement_digest,
        &current.statement_profile,
        rooted.qualification_digest(),
        rooted.qualification_profile(),
        &current.verification_ref,
        current.verified_at_ms,
        current.valid_until_ms,
    );
    let context = QualifiedBindingConstitutionContext {
        dna_hash: current.dna_hash.clone(),
        statement_digest: current.statement_digest,
        statement_profile: current.statement_profile.clone(),
        root_qualification_digest: rooted.qualification_digest(),
        root_qualification_profile: rooted.qualification_profile().into(),
        verification_ref: current.verification_ref.clone(),
        verified_at_ms: current.verified_at_ms,
        valid_until_ms: current.valid_until_ms,
        context_digest,
        context_profile: BINDING_CONSTITUTION_CONTEXT_PROFILE.into(),
    };
    context.validate_at(now_ms)?;
    Ok(context)
}

/// Final deployment proof whose stable authority remains exactly #154's stable
/// deployment identity while dynamic evidence additionally commits the final
/// binding-constitution re-observation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedConstitutionBoundProvenanceDeploymentOperationalFreshness {
    base: QualifiedProvenanceBoundDeploymentOperationalFreshness,
    binding_constitution_context_digest: Digest32,
    binding_constitution_context_profile: String,
    binding_constitution_verification_ref: String,
    binding_constitution_verified_at_ms: u64,
    binding_constitution_valid_until_ms: u64,
    deployment_evidence_digest: Digest32,
    deployment_evidence_profile: String,
    current_freshness: VerifiedAuthorityFreshness,
    verification_ref: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedConstitutionBoundProvenanceDeploymentOperationalFreshness {
    pub fn subject(&self) -> &AuthoritySubjectRef {
        self.base.subject()
    }

    pub fn dna_hash(&self) -> &str {
        self.base.dna_hash()
    }

    pub fn constitution_statement_digest(&self) -> Digest32 {
        self.base.constitution_statement_digest()
    }

    pub fn constitution_statement_profile(&self) -> &str {
        self.base.constitution_statement_profile()
    }

    /// Stable deployment authority remains exactly the underlying #154 identity.
    pub fn deployment_authority_digest(&self) -> Digest32 {
        self.base.deployment_authority_digest()
    }

    pub fn deployment_authority_profile(&self) -> &str {
        self.base.deployment_authority_profile()
    }

    pub fn provenance_manifest_digest(&self) -> Digest32 {
        self.base.provenance_manifest_digest()
    }

    pub fn provenance_manifest_profile(&self) -> &str {
        self.base.provenance_manifest_profile()
    }

    pub fn provenance_contributor_count(&self) -> u32 {
        self.base.provenance_contributor_count()
    }

    pub fn composition_evidence_verified_at_ms(&self) -> u64 {
        self.base.composition_evidence_verified_at_ms()
    }

    pub fn composition_evidence_valid_until_ms(&self) -> u64 {
        self.base.composition_evidence_valid_until_ms()
    }

    pub fn binding_constitution_context_digest(&self) -> Digest32 {
        self.binding_constitution_context_digest
    }

    pub fn binding_constitution_context_profile(&self) -> &str {
        &self.binding_constitution_context_profile
    }

    pub fn binding_constitution_verification_ref(&self) -> &str {
        &self.binding_constitution_verification_ref
    }

    pub fn binding_constitution_verified_at_ms(&self) -> u64 {
        self.binding_constitution_verified_at_ms
    }

    pub fn binding_constitution_valid_until_ms(&self) -> u64 {
        self.binding_constitution_valid_until_ms
    }

    pub fn deployment_evidence_digest(&self) -> Digest32 {
        self.deployment_evidence_digest
    }

    pub fn deployment_evidence_profile(&self) -> &str {
        &self.deployment_evidence_profile
    }

    pub fn verification_ref(&self) -> &str {
        &self.verification_ref
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub fn to_verified_freshness(&self) -> VerifiedAuthorityFreshness {
        self.current_freshness.clone()
    }
}

/// Bind final current-constitution evidence and canonical composition provenance
/// into dynamic deployment evidence. Plain constitutional DNA/digest primitives
/// do not enter this active theorem: they are projected only from the qualified
/// non-deserializable context.
pub fn qualify_operational_freshness_for_deployment_with_constitution_and_provenance(
    semantic: &QualifiedOperationalSubjectFreshness,
    constitution: &QualifiedBindingConstitutionContext,
    local_dna: &HostLocalDnaContext,
    provenance: &QualifiedEvidenceLeaseManifest,
    now_ms: u64,
) -> Result<QualifiedConstitutionBoundProvenanceDeploymentOperationalFreshness, BindingConstitutionDeploymentError>
{
    constitution.validate_at(now_ms)?;
    local_dna
        .validate_at(now_ms)
        .map_err(BindingConstitutionDeploymentError::BaseDeploymentDenied)?;

    if semantic.root_qualification_digest() != constitution.root_qualification_digest()
        || semantic.root_qualification_profile() != constitution.root_qualification_profile()
    {
        return Err(BindingConstitutionDeploymentError::ConstitutionRootMismatch);
    }
    if local_dna.valid_until_ms() > constitution.valid_until_ms {
        return Err(BindingConstitutionDeploymentError::ConstitutionLeaseMismatch);
    }

    let base = qualify_operational_freshness_for_deployment_with_provenance(
        semantic,
        constitution.dna_hash(),
        constitution.statement_digest(),
        local_dna,
        provenance,
        now_ms,
    )
    .map_err(BindingConstitutionDeploymentError::ProvenanceDeploymentDenied)?;

    if base.dna_hash() != constitution.dna_hash()
        || base.constitution_statement_digest() != constitution.statement_digest()
        || base.constitution_statement_profile() != constitution.statement_profile()
        || base.valid_until_ms() > constitution.valid_until_ms()
    {
        return Err(BindingConstitutionDeploymentError::ConstitutionContextMismatch);
    }

    let deployment_evidence_digest = constitution_bound_deployment_evidence_digest(
        base.deployment_evidence_digest(),
        base.deployment_evidence_profile(),
        constitution.context_digest(),
        constitution.context_profile(),
    );
    let verification_ref = format!(
        "operational-deployment-evidence:{CONSTITUTION_BOUND_DEPLOYMENT_EVIDENCE_PROFILE}:{}",
        digest_hex(deployment_evidence_digest)
    );

    let verified_at_ms = base.verified_at_ms().max(constitution.verified_at_ms());
    let valid_until_ms = base.valid_until_ms().min(constitution.valid_until_ms());
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(BindingConstitutionDeploymentError::ConstitutionLeaseMismatch);
    }

    let mut current_freshness = base.to_verified_freshness();
    current_freshness.verification_ref = verification_ref.clone();
    current_freshness.verified_at_ms = current_freshness.verified_at_ms.max(verified_at_ms);
    current_freshness.lease_until_ms = current_freshness.lease_until_ms.min(valid_until_ms);
    current_freshness
        .validate_at(now_ms)
        .map_err(|_| BindingConstitutionDeploymentError::ConstitutionLeaseMismatch)?;

    Ok(QualifiedConstitutionBoundProvenanceDeploymentOperationalFreshness {
        base,
        binding_constitution_context_digest: constitution.context_digest(),
        binding_constitution_context_profile: constitution.context_profile().into(),
        binding_constitution_verification_ref: constitution.verification_ref().into(),
        binding_constitution_verified_at_ms: constitution.verified_at_ms(),
        binding_constitution_valid_until_ms: constitution.valid_until_ms(),
        deployment_evidence_digest,
        deployment_evidence_profile: CONSTITUTION_BOUND_DEPLOYMENT_EVIDENCE_PROFILE.into(),
        current_freshness,
        verification_ref,
        verified_at_ms,
        valid_until_ms,
    })
}

fn binding_constitution_context_digest(
    dna_hash: &str,
    statement_digest: Digest32,
    statement_profile: &str,
    root_qualification_digest: Digest32,
    root_qualification_profile: &str,
    verification_ref: &str,
    verified_at_ms: u64,
    valid_until_ms: u64,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_BINDING_CONSTITUTION_CONTEXT);
    frame(&mut hasher, BINDING_CONSTITUTION_CONTEXT_PROFILE.as_bytes());
    frame(&mut hasher, dna_hash.as_bytes());
    frame(&mut hasher, statement_profile.as_bytes());
    frame(&mut hasher, &statement_digest.0);
    frame(&mut hasher, root_qualification_profile.as_bytes());
    frame(&mut hasher, &root_qualification_digest.0);
    frame(&mut hasher, verification_ref.as_bytes());
    frame(&mut hasher, &verified_at_ms.to_le_bytes());
    frame(&mut hasher, &valid_until_ms.to_le_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

fn constitution_bound_deployment_evidence_digest(
    base_deployment_evidence_digest: Digest32,
    base_deployment_evidence_profile: &str,
    constitution_context_digest: Digest32,
    constitution_context_profile: &str,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_CONSTITUTION_BOUND_DEPLOYMENT_EVIDENCE);
    frame(
        &mut hasher,
        CONSTITUTION_BOUND_DEPLOYMENT_EVIDENCE_PROFILE.as_bytes(),
    );
    frame(&mut hasher, base_deployment_evidence_profile.as_bytes());
    frame(&mut hasher, &base_deployment_evidence_digest.0);
    frame(&mut hasher, constitution_context_profile.as_bytes());
    frame(&mut hasher, &constitution_context_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BindingConstitutionDeploymentError {
    BaseDeploymentDenied(DeploymentFenceError),
    ProvenanceDeploymentDenied(ProvenanceDeploymentFenceError),
    CurrentConstitutionReceiptDenied,
    ConstitutionEpochMismatch,
    InvalidConstitutionContext,
    ConstitutionRootMismatch,
    ConstitutionContextMismatch,
    ConstitutionLeaseMismatch,
}

impl fmt::Display for BindingConstitutionDeploymentError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BaseDeploymentDenied(error) => write!(f, "base deployment denied: {error}"),
            Self::ProvenanceDeploymentDenied(error) => {
                write!(f, "provenance deployment denied: {error}")
            }
            Self::CurrentConstitutionReceiptDenied => {
                write!(f, "current constitution receipt is invalid or stale")
            }
            Self::ConstitutionEpochMismatch => {
                write!(f, "final binding constitution does not match the qualified bootstrap root")
            }
            Self::InvalidConstitutionContext => {
                write!(f, "qualified binding constitution context is invalid or stale")
            }
            Self::ConstitutionRootMismatch => {
                write!(f, "semantic currentness and binding constitution context use different roots")
            }
            Self::ConstitutionContextMismatch => {
                write!(f, "deployment result does not match binding constitution context")
            }
            Self::ConstitutionLeaseMismatch => {
                write!(f, "deployment/host reuse exceeds binding constitution evidence")
            }
        }
    }
}

impl std::error::Error for BindingConstitutionDeploymentError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    #[test]
    fn constitution_context_observation_changes_dynamic_identity() {
        let first = binding_constitution_context_digest(
            "uhC0kA",
            d(1),
            STATEMENT_PROFILE,
            d(2),
            ROOT_QUALIFICATION_PROFILE,
            "constitution:1",
            100,
            500,
        );
        let second = binding_constitution_context_digest(
            "uhC0kA",
            d(1),
            STATEMENT_PROFILE,
            d(2),
            ROOT_QUALIFICATION_PROFILE,
            "constitution:1",
            120,
            500,
        );
        assert_ne!(first, second);
    }

    #[test]
    fn root_identity_changes_constitution_context_identity() {
        let first = binding_constitution_context_digest(
            "uhC0kA",
            d(1),
            STATEMENT_PROFILE,
            d(2),
            ROOT_QUALIFICATION_PROFILE,
            "constitution:1",
            100,
            500,
        );
        let second = binding_constitution_context_digest(
            "uhC0kA",
            d(1),
            STATEMENT_PROFILE,
            d(3),
            ROOT_QUALIFICATION_PROFILE,
            "constitution:1",
            100,
            500,
        );
        assert_ne!(first, second);
    }

    #[test]
    fn constitution_context_changes_deployment_evidence_not_base_authority_input() {
        let first = constitution_bound_deployment_evidence_digest(
            d(1),
            PROVENANCE_BOUND_DEPLOYMENT_EVIDENCE_PROFILE,
            d(2),
            BINDING_CONSTITUTION_CONTEXT_PROFILE,
        );
        let second = constitution_bound_deployment_evidence_digest(
            d(1),
            PROVENANCE_BOUND_DEPLOYMENT_EVIDENCE_PROFILE,
            d(3),
            BINDING_CONSTITUTION_CONTEXT_PROFILE,
        );
        assert_ne!(first, second);
    }
}
