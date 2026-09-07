// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Full constitutional-currentness evidence binding for final deployment.
//!
//! #111 still consumes its historical current-constitution receipt ABI during
//! bootstrap-root qualification. The final deployment path is stronger: it
//! consumes the canonical shared currentness transport directly, revalidates its
//! exact dynamic evidence identity, then wraps the already-reviewed #217 theorem.
//! Stable deployment authority is delegated unchanged; only dynamic deployment
//! evidence gains the explicit currentness-evidence commitment.

use super::*;
use mycelix_authority_evidence_lease::QualifiedEvidenceLeaseManifest;
use mycelix_authority_state_bootstrap_root::{
    QualifiedAuthorityStateBootstrapRoot, VerifiedCurrentConstitutionReceipt,
    CURRENT_CONSTITUTION_RECEIPT_PROTOCOL,
};
use mycelix_governance_constitution::STATEMENT_PROFILE;
use mycelix_governance_constitution_currentness::{
    CurrentnessContractError, LeasedVerifiedCurrentConstitution,
    CURRENTNESS_EVIDENCE_PROFILE,
};
use serde::Serialize;
use std::fmt;

pub const FULL_CURRENTNESS_CONSTITUTION_CONTEXT_PROFILE: &str =
    "mycelix-binding-constitution-currentness-deployment-context-v1-blake3-framed";
pub const FULL_CURRENTNESS_DEPLOYMENT_EVIDENCE_PROFILE: &str =
    "mycelix-authority-operational-deployment-evidence-currentness-v1-blake3-framed";

const DOMAIN_FULL_CURRENTNESS_CONSTITUTION_CONTEXT: &[u8] =
    b"mycelix/authority/binding-constitution-currentness-deployment-context/v1";
const DOMAIN_FULL_CURRENTNESS_DEPLOYMENT_EVIDENCE: &[u8] =
    b"mycelix/authority/operational-deployment-evidence-currentness/v1";

/// Non-deserializable final constitutional context that preserves the exact
/// shared currentness-evidence identity instead of projecting it away into the
/// older bootstrap receipt ABI.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedCurrentnessBindingConstitutionContext {
    base: QualifiedBindingConstitutionContext,
    currentness_evidence_digest: Digest32,
    currentness_evidence_profile: String,
    context_digest: Digest32,
    context_profile: String,
}

impl QualifiedCurrentnessBindingConstitutionContext {
    pub fn dna_hash(&self) -> &str {
        self.base.dna_hash()
    }

    pub fn statement_digest(&self) -> Digest32 {
        self.base.statement_digest()
    }

    pub fn statement_profile(&self) -> &str {
        self.base.statement_profile()
    }

    pub fn root_qualification_digest(&self) -> Digest32 {
        self.base.root_qualification_digest()
    }

    pub fn root_qualification_profile(&self) -> &str {
        self.base.root_qualification_profile()
    }

    pub fn currentness_evidence_digest(&self) -> Digest32 {
        self.currentness_evidence_digest
    }

    pub fn currentness_evidence_profile(&self) -> &str {
        &self.currentness_evidence_profile
    }

    pub fn verification_ref(&self) -> &str {
        self.base.verification_ref()
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.base.verified_at_ms()
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.base.valid_until_ms()
    }

    pub fn context_digest(&self) -> Digest32 {
        self.context_digest
    }

    pub fn context_profile(&self) -> &str {
        &self.context_profile
    }

    fn compatibility_context(&self) -> &QualifiedBindingConstitutionContext {
        &self.base
    }

    fn validate_at(
        &self,
        now_ms: u64,
    ) -> Result<(), CurrentnessBindingDeploymentError> {
        if self.currentness_evidence_digest.is_zero()
            || self.currentness_evidence_profile != CURRENTNESS_EVIDENCE_PROFILE
            || self.context_profile != FULL_CURRENTNESS_CONSTITUTION_CONTEXT_PROFILE
        {
            return Err(CurrentnessBindingDeploymentError::InvalidCurrentnessContext);
        }
        if now_ms == 0
            || self.verified_at_ms() == 0
            || self.verified_at_ms() > now_ms
            || self.valid_until_ms() <= now_ms
        {
            return Err(CurrentnessBindingDeploymentError::InvalidCurrentnessContext);
        }
        let recomputed = full_currentness_context_digest(
            self.base.context_digest(),
            self.base.context_profile(),
            self.currentness_evidence_digest,
            &self.currentness_evidence_profile,
        );
        if recomputed != self.context_digest {
            return Err(CurrentnessBindingDeploymentError::InvalidCurrentnessContext);
        }
        Ok(())
    }
}

/// Qualify the exact shared constitutional-currentness evidence against the same
/// non-deserializable #111 root already used by semantic currentness.
pub fn qualify_currentness_binding_constitution_context(
    current: &LeasedVerifiedCurrentConstitution,
    rooted: &QualifiedAuthorityStateBootstrapRoot,
    now_ms: u64,
) -> Result<QualifiedCurrentnessBindingConstitutionContext, CurrentnessBindingDeploymentError> {
    current
        .validate_at(now_ms)
        .map_err(CurrentnessBindingDeploymentError::CurrentnessContractDenied)?;

    let receipt = VerifiedCurrentConstitutionReceipt {
        protocol_version: CURRENT_CONSTITUTION_RECEIPT_PROTOCOL.into(),
        statement: current.statement.clone(),
        statement_digest: Digest32(current.statement_digest.0),
        statement_profile: STATEMENT_PROFILE.into(),
        dna_hash: current.dna_hash.clone(),
        verification_ref: current.verification_ref.clone(),
        verified_at_ms: current.verified_at_ms,
        valid_until_ms: current.valid_until_ms,
    };
    let base = qualify_binding_constitution_context(&receipt, rooted, now_ms)
        .map_err(CurrentnessBindingDeploymentError::CompatibilityContextDenied)?;

    if base.dna_hash() != current.dna_hash
        || base.statement_digest() != Digest32(current.statement_digest.0)
        || base.verification_ref() != current.verification_ref
        || base.verified_at_ms() != current.verified_at_ms
        || base.valid_until_ms() != current.valid_until_ms
    {
        return Err(CurrentnessBindingDeploymentError::CurrentnessContextMismatch);
    }

    let currentness_evidence_digest = Digest32(current.currentness_evidence_digest.0);
    let context_digest = full_currentness_context_digest(
        base.context_digest(),
        base.context_profile(),
        currentness_evidence_digest,
        &current.currentness_evidence_profile,
    );
    let context = QualifiedCurrentnessBindingConstitutionContext {
        base,
        currentness_evidence_digest,
        currentness_evidence_profile: current.currentness_evidence_profile.clone(),
        context_digest,
        context_profile: FULL_CURRENTNESS_CONSTITUTION_CONTEXT_PROFILE.into(),
    };
    context.validate_at(now_ms)?;
    Ok(context)
}

/// Final positive deployment object whose stable authority remains exactly the
/// #154/#201/#217 stable identity, while its dynamic evidence additionally commits
/// the explicit shared constitutional-currentness evidence identity.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedCurrentnessBoundProvenanceDeploymentOperationalFreshness {
    base: QualifiedConstitutionBoundProvenanceDeploymentOperationalFreshness,
    currentness_context_digest: Digest32,
    currentness_context_profile: String,
    currentness_evidence_digest: Digest32,
    currentness_evidence_profile: String,
    deployment_evidence_digest: Digest32,
    deployment_evidence_profile: String,
    current_freshness: VerifiedAuthorityFreshness,
    verification_ref: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedCurrentnessBoundProvenanceDeploymentOperationalFreshness {
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
        self.currentness_context_digest
    }

    pub fn binding_constitution_context_profile(&self) -> &str {
        &self.currentness_context_profile
    }

    pub fn binding_constitution_verification_ref(&self) -> &str {
        self.base.binding_constitution_verification_ref()
    }

    pub fn binding_constitution_verified_at_ms(&self) -> u64 {
        self.base.binding_constitution_verified_at_ms()
    }

    pub fn binding_constitution_valid_until_ms(&self) -> u64 {
        self.base.binding_constitution_valid_until_ms()
    }

    pub fn binding_constitution_currentness_evidence_digest(&self) -> Digest32 {
        self.currentness_evidence_digest
    }

    pub fn binding_constitution_currentness_evidence_profile(&self) -> &str {
        &self.currentness_evidence_profile
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

pub fn qualify_operational_freshness_for_deployment_with_currentness_and_provenance(
    semantic: &QualifiedOperationalSubjectFreshness,
    constitution: &QualifiedCurrentnessBindingConstitutionContext,
    local_dna: &HostLocalDnaContext,
    provenance: &QualifiedEvidenceLeaseManifest,
    now_ms: u64,
) -> Result<QualifiedCurrentnessBoundProvenanceDeploymentOperationalFreshness, CurrentnessBindingDeploymentError>
{
    constitution.validate_at(now_ms)?;

    let base = qualify_operational_freshness_for_deployment_with_constitution_and_provenance(
        semantic,
        constitution.compatibility_context(),
        local_dna,
        provenance,
        now_ms,
    )
    .map_err(CurrentnessBindingDeploymentError::CompatibilityDeploymentDenied)?;

    if base.binding_constitution_verification_ref() != constitution.verification_ref()
        || base.binding_constitution_verified_at_ms() != constitution.verified_at_ms()
        || base.binding_constitution_valid_until_ms() != constitution.valid_until_ms()
        || base.valid_until_ms() > constitution.valid_until_ms()
    {
        return Err(CurrentnessBindingDeploymentError::CurrentnessContextMismatch);
    }

    let deployment_evidence_digest = currentness_bound_deployment_evidence_digest(
        base.deployment_evidence_digest(),
        base.deployment_evidence_profile(),
        constitution.context_digest(),
        constitution.context_profile(),
    );
    let verification_ref = format!(
        "operational-deployment-evidence:{FULL_CURRENTNESS_DEPLOYMENT_EVIDENCE_PROFILE}:{}",
        digest_hex(deployment_evidence_digest)
    );
    let verified_at_ms = base.verified_at_ms().max(constitution.verified_at_ms());
    let valid_until_ms = base.valid_until_ms().min(constitution.valid_until_ms());
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(CurrentnessBindingDeploymentError::CurrentnessLeaseMismatch);
    }

    let mut current_freshness = base.to_verified_freshness();
    current_freshness.verification_ref = verification_ref.clone();
    current_freshness.verified_at_ms = current_freshness.verified_at_ms.max(verified_at_ms);
    current_freshness.lease_until_ms = current_freshness.lease_until_ms.min(valid_until_ms);
    current_freshness
        .validate_at(now_ms)
        .map_err(|_| CurrentnessBindingDeploymentError::CurrentnessLeaseMismatch)?;

    Ok(QualifiedCurrentnessBoundProvenanceDeploymentOperationalFreshness {
        currentness_context_digest: constitution.context_digest(),
        currentness_context_profile: constitution.context_profile().into(),
        currentness_evidence_digest: constitution.currentness_evidence_digest(),
        currentness_evidence_profile: constitution.currentness_evidence_profile().into(),
        base,
        deployment_evidence_digest,
        deployment_evidence_profile: FULL_CURRENTNESS_DEPLOYMENT_EVIDENCE_PROFILE.into(),
        current_freshness,
        verification_ref,
        verified_at_ms,
        valid_until_ms,
    })
}

fn full_currentness_context_digest(
    compatibility_context_digest: Digest32,
    compatibility_context_profile: &str,
    currentness_evidence_digest: Digest32,
    currentness_evidence_profile: &str,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_FULL_CURRENTNESS_CONSTITUTION_CONTEXT);
    frame(
        &mut hasher,
        FULL_CURRENTNESS_CONSTITUTION_CONTEXT_PROFILE.as_bytes(),
    );
    frame(&mut hasher, compatibility_context_profile.as_bytes());
    frame(&mut hasher, &compatibility_context_digest.0);
    frame(&mut hasher, currentness_evidence_profile.as_bytes());
    frame(&mut hasher, &currentness_evidence_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

fn currentness_bound_deployment_evidence_digest(
    base_deployment_evidence_digest: Digest32,
    base_deployment_evidence_profile: &str,
    currentness_context_digest: Digest32,
    currentness_context_profile: &str,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_FULL_CURRENTNESS_DEPLOYMENT_EVIDENCE);
    frame(
        &mut hasher,
        FULL_CURRENTNESS_DEPLOYMENT_EVIDENCE_PROFILE.as_bytes(),
    );
    frame(&mut hasher, base_deployment_evidence_profile.as_bytes());
    frame(&mut hasher, &base_deployment_evidence_digest.0);
    frame(&mut hasher, currentness_context_profile.as_bytes());
    frame(&mut hasher, &currentness_context_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CurrentnessBindingDeploymentError {
    CurrentnessContractDenied(CurrentnessContractError),
    CompatibilityContextDenied(BindingConstitutionDeploymentError),
    CompatibilityDeploymentDenied(BindingConstitutionDeploymentError),
    InvalidCurrentnessContext,
    CurrentnessContextMismatch,
    CurrentnessLeaseMismatch,
}

impl fmt::Display for CurrentnessBindingDeploymentError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::CurrentnessContractDenied(error) => {
                write!(f, "constitutional currentness contract denied: {error}")
            }
            Self::CompatibilityContextDenied(error) => {
                write!(f, "binding constitution compatibility context denied: {error}")
            }
            Self::CompatibilityDeploymentDenied(error) => {
                write!(f, "constitution/provenance deployment denied: {error}")
            }
            Self::InvalidCurrentnessContext => {
                write!(f, "explicit constitutional currentness context is invalid")
            }
            Self::CurrentnessContextMismatch => {
                write!(f, "explicit constitutional currentness evidence does not match compatibility deployment evidence")
            }
            Self::CurrentnessLeaseMismatch => {
                write!(f, "explicit constitutional currentness lease does not contain deployment reuse")
            }
        }
    }
}

impl std::error::Error for CurrentnessBindingDeploymentError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    #[test]
    fn explicit_currentness_changes_context_identity() {
        let a = full_currentness_context_digest(d(1), "compat", d(2), "currentness");
        let b = full_currentness_context_digest(d(1), "compat", d(3), "currentness");
        assert_ne!(a, b);
    }

    #[test]
    fn explicit_currentness_context_changes_dynamic_deployment_evidence() {
        let a = currentness_bound_deployment_evidence_digest(d(1), "base", d(2), "ctx");
        let b = currentness_bound_deployment_evidence_digest(d(1), "base", d(3), "ctx");
        assert_ne!(a, b);
    }
}
