// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Provenance-bound dynamic deployment evidence.
//!
//! This sibling theorem preserves #154 stable deployment authority identity while
//! binding one exact #192 canonical lease-provenance manifest into deployment
//! evidence identity. Provenance remains dynamic audit evidence, never authority.

use super::*;
use mycelix_authority_evidence_lease::QualifiedEvidenceLeaseManifest;
use serde::Serialize;
use std::fmt;

pub const PROVENANCE_BOUND_DEPLOYMENT_EVIDENCE_PROFILE: &str =
    "mycelix-authority-operational-deployment-evidence-provenance-v1-blake3-framed";
const DOMAIN_PROVENANCE_BOUND_DEPLOYMENT_EVIDENCE: &[u8] =
    b"mycelix/authority/operational-deployment-evidence-provenance/v1";

/// Non-deserializable proof that one exact #154 deployment qualification is
/// backed by one exact canonical #192 evidence-provenance manifest.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedProvenanceBoundDeploymentOperationalFreshness {
    base: QualifiedDeploymentOperationalFreshness,
    provenance_manifest_digest: Digest32,
    provenance_manifest_profile: String,
    provenance_contributor_count: u32,
    composition_evidence_verified_at_ms: u64,
    composition_evidence_valid_until_ms: u64,
    deployment_evidence_digest: Digest32,
    deployment_evidence_profile: String,
    current_freshness: VerifiedAuthorityFreshness,
    verification_ref: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedProvenanceBoundDeploymentOperationalFreshness {
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

    /// Stable deployment authority is exactly #154's authority identity.
    pub fn deployment_authority_digest(&self) -> Digest32 {
        self.base.deployment_authority_digest()
    }

    pub fn deployment_authority_profile(&self) -> &str {
        self.base.deployment_authority_profile()
    }

    /// Dynamic deployment evidence additionally commits the canonical provenance.
    pub fn deployment_evidence_digest(&self) -> Digest32 {
        self.deployment_evidence_digest
    }

    pub fn deployment_evidence_profile(&self) -> &str {
        &self.deployment_evidence_profile
    }

    pub fn provenance_manifest_digest(&self) -> Digest32 {
        self.provenance_manifest_digest
    }

    pub fn provenance_manifest_profile(&self) -> &str {
        &self.provenance_manifest_profile
    }

    pub fn provenance_contributor_count(&self) -> u32 {
        self.provenance_contributor_count
    }

    pub fn composition_evidence_verified_at_ms(&self) -> u64 {
        self.composition_evidence_verified_at_ms
    }

    pub fn composition_evidence_valid_until_ms(&self) -> u64 {
        self.composition_evidence_valid_until_ms
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

/// Bind an exact canonical provenance manifest into dynamic deployment evidence.
///
/// The underlying #154 stable deployment authority identity is preserved exactly.
/// The provenance manifest must be live, and the already-capped host/deployment
/// lease must neither predate nor outlive its aggregate evidence lease.
#[allow(clippy::too_many_arguments)]
pub fn qualify_operational_freshness_for_deployment_with_provenance(
    semantic: &QualifiedOperationalSubjectFreshness,
    constitutional_dna_hash: &str,
    constitution_statement_digest: Digest32,
    local_dna: &HostLocalDnaContext,
    provenance: &QualifiedEvidenceLeaseManifest,
    now_ms: u64,
) -> Result<QualifiedProvenanceBoundDeploymentOperationalFreshness, ProvenanceDeploymentFenceError>
{
    provenance
        .aggregate_lease()
        .validate_at(now_ms)
        .map_err(|_| ProvenanceDeploymentFenceError::ProvenanceManifestNotCurrent)?;

    let base = qualify_operational_freshness_for_deployment(
        semantic,
        constitutional_dna_hash,
        constitution_statement_digest,
        local_dna,
        now_ms,
    )
    .map_err(ProvenanceDeploymentFenceError::BaseDeploymentDenied)?;

    let aggregate = provenance.aggregate_lease();
    if local_dna.valid_until_ms() > aggregate.valid_until_ms
        || base.valid_until_ms() > aggregate.valid_until_ms
        || base.verified_at_ms() < aggregate.verified_at_ms
    {
        return Err(ProvenanceDeploymentFenceError::ProvenanceLeaseMismatch);
    }

    let provenance_manifest_digest = Digest32(provenance.manifest_digest());
    let deployment_evidence_digest = provenance_bound_deployment_evidence_digest(
        base.deployment_evidence_digest(),
        base.deployment_evidence_profile(),
        provenance_manifest_digest,
        provenance.manifest_profile(),
        provenance.contributor_count(),
        aggregate.verified_at_ms,
        aggregate.valid_until_ms,
    );
    let verification_ref = format!(
        "operational-deployment-evidence:{PROVENANCE_BOUND_DEPLOYMENT_EVIDENCE_PROFILE}:{}",
        digest_hex(deployment_evidence_digest)
    );

    let verified_at_ms = base.verified_at_ms().max(aggregate.verified_at_ms);
    let valid_until_ms = base.valid_until_ms().min(aggregate.valid_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(ProvenanceDeploymentFenceError::ProvenanceLeaseMismatch);
    }

    let mut current_freshness = base.to_verified_freshness();
    current_freshness.verification_ref = verification_ref.clone();
    current_freshness.verified_at_ms = current_freshness.verified_at_ms.max(verified_at_ms);
    current_freshness.lease_until_ms = current_freshness.lease_until_ms.min(valid_until_ms);
    current_freshness
        .validate_at(now_ms)
        .map_err(|_| ProvenanceDeploymentFenceError::ProvenanceLeaseMismatch)?;

    Ok(QualifiedProvenanceBoundDeploymentOperationalFreshness {
        base,
        provenance_manifest_digest,
        provenance_manifest_profile: provenance.manifest_profile().into(),
        provenance_contributor_count: provenance.contributor_count(),
        composition_evidence_verified_at_ms: aggregate.verified_at_ms,
        composition_evidence_valid_until_ms: aggregate.valid_until_ms,
        deployment_evidence_digest,
        deployment_evidence_profile: PROVENANCE_BOUND_DEPLOYMENT_EVIDENCE_PROFILE.into(),
        current_freshness,
        verification_ref,
        verified_at_ms,
        valid_until_ms,
    })
}

#[allow(clippy::too_many_arguments)]
fn provenance_bound_deployment_evidence_digest(
    base_deployment_evidence_digest: Digest32,
    base_deployment_evidence_profile: &str,
    provenance_manifest_digest: Digest32,
    provenance_manifest_profile: &str,
    provenance_contributor_count: u32,
    composition_verified_at_ms: u64,
    composition_valid_until_ms: u64,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_PROVENANCE_BOUND_DEPLOYMENT_EVIDENCE);
    frame(
        &mut hasher,
        PROVENANCE_BOUND_DEPLOYMENT_EVIDENCE_PROFILE.as_bytes(),
    );
    frame(&mut hasher, base_deployment_evidence_profile.as_bytes());
    frame(&mut hasher, &base_deployment_evidence_digest.0);
    frame(&mut hasher, provenance_manifest_profile.as_bytes());
    frame(&mut hasher, &provenance_manifest_digest.0);
    frame(
        &mut hasher,
        &provenance_contributor_count.to_le_bytes(),
    );
    frame(&mut hasher, &composition_verified_at_ms.to_le_bytes());
    frame(&mut hasher, &composition_valid_until_ms.to_le_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProvenanceDeploymentFenceError {
    BaseDeploymentDenied(DeploymentFenceError),
    ProvenanceManifestNotCurrent,
    ProvenanceLeaseMismatch,
}

impl fmt::Display for ProvenanceDeploymentFenceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BaseDeploymentDenied(error) => write!(f, "base deployment denied: {error}"),
            Self::ProvenanceManifestNotCurrent => {
                write!(f, "canonical evidence provenance manifest is not currently live")
            }
            Self::ProvenanceLeaseMismatch => write!(
                f,
                "deployment/host evidence window is inconsistent with canonical provenance lease"
            ),
        }
    }
}

impl std::error::Error for ProvenanceDeploymentFenceError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    #[test]
    fn provenance_changes_dynamic_evidence_identity() {
        let first = provenance_bound_deployment_evidence_digest(
            d(1),
            DEPLOYMENT_EVIDENCE_PROFILE,
            d(2),
            "manifest-v1-blake3",
            4,
            100,
            500,
        );
        let second = provenance_bound_deployment_evidence_digest(
            d(1),
            DEPLOYMENT_EVIDENCE_PROFILE,
            d(3),
            "manifest-v1-blake3",
            4,
            100,
            500,
        );
        assert_ne!(first, second);
    }

    #[test]
    fn provenance_horizon_changes_dynamic_evidence_identity() {
        let first = provenance_bound_deployment_evidence_digest(
            d(1),
            DEPLOYMENT_EVIDENCE_PROFILE,
            d(2),
            "manifest-v1-blake3",
            4,
            100,
            500,
        );
        let second = provenance_bound_deployment_evidence_digest(
            d(1),
            DEPLOYMENT_EVIDENCE_PROFILE,
            d(2),
            "manifest-v1-blake3",
            4,
            100,
            450,
        );
        assert_ne!(first, second);
    }
}
