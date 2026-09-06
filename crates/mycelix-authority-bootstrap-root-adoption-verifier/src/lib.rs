// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure exact-proof qualification for authority bootstrap-root adoption.
//!
//! A proof verifier may authenticate one exact adoption claim. It may not directly
//! manufacture the `VerifiedBootstrapRootAdoption` ABI consumed by #111. This
//! kernel reconstructs the claim from the exact current ConstitutionStatement and
//! exact root manifest, cross-binds the independently verified proof, and only then
//! projects the existing #111 adoption receipt.

use mycelix_authority_state_bootstrap_root::{
    AuthorityStateBootstrapRootManifest, VerifiedBootstrapRootAdoption,
    ROOT_ADOPTION_RECEIPT_PROTOCOL, ROOT_MANIFEST_PROFILE,
};
use mycelix_governance_constitution::{ConstitutionStatement, STATEMENT_PROFILE};
use mycelix_institutional_core::Digest32;
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-bootstrap-root-adoption-verifier-v0.1";
pub const ADOPTION_CLAIM_PROFILE: &str =
    "mycelix-authority-bootstrap-root-adoption-claim-v1-blake3-framed";
pub const QUALIFICATION_PROFILE: &str =
    "mycelix-authority-bootstrap-root-adoption-qualification-v1-blake3-framed";
pub const EVIDENCE_PROFILE: &str =
    "mycelix-authority-bootstrap-root-adoption-evidence-v1-blake3-framed";

const DOMAIN_CLAIM: &[u8] = b"mycelix/authority/bootstrap-root/adoption-claim/v1";
const DOMAIN_QUALIFICATION: &[u8] = b"mycelix/authority/bootstrap-root/adoption-qualification/v1";
const DOMAIN_EVIDENCE: &[u8] = b"mycelix/authority/bootstrap-root/adoption-evidence/v1";
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;

/// Exact semantic statement whose proof must be independently verified.
///
/// Runtime adapters must construct this claim from the current constitutional
/// statement and candidate root manifest rather than accepting it from the caller.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BootstrapRootAdoptionClaim {
    pub constitution_statement_digest: Digest32,
    pub constitution_statement_profile: String,
    pub rulebook_ref: String,
    pub rulebook_version: String,
    pub rulebook_digest: Digest32,
    pub rulebook_profile: String,
    pub root_manifest_digest: Digest32,
    pub root_manifest_profile: String,
    pub adoption_authority_ref: String,
    pub adoption_ref: String,
    pub adoption_proof_ref: String,
}

impl BootstrapRootAdoptionClaim {
    pub fn validate(&self) -> Result<(), AdoptionVerifierError> {
        for digest in [
            self.constitution_statement_digest,
            self.rulebook_digest,
            self.root_manifest_digest,
        ] {
            require_digest(digest)?;
        }
        if self.constitution_statement_profile != STATEMENT_PROFILE
            || self.root_manifest_profile != ROOT_MANIFEST_PROFILE
        {
            return Err(AdoptionVerifierError::WrongIdentityProfile);
        }
        for profile in [
            self.constitution_statement_profile.as_str(),
            self.rulebook_profile.as_str(),
            self.root_manifest_profile.as_str(),
        ] {
            require_profile(profile)?;
        }
        for value in [
            self.rulebook_ref.as_str(),
            self.rulebook_version.as_str(),
            self.adoption_authority_ref.as_str(),
            self.adoption_ref.as_str(),
            self.adoption_proof_ref.as_str(),
        ] {
            require_ref(value)?;
        }
        Ok(())
    }

    pub fn identity_digest(&self) -> Result<Digest32, AdoptionVerifierError> {
        self.validate()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_CLAIM);
        frame(&mut hasher, ADOPTION_CLAIM_PROFILE.as_bytes());
        frame(&mut hasher, self.constitution_statement_profile.as_bytes());
        frame(&mut hasher, &self.constitution_statement_digest.0);
        frame(&mut hasher, self.rulebook_ref.as_bytes());
        frame(&mut hasher, self.rulebook_version.as_bytes());
        frame(&mut hasher, self.rulebook_profile.as_bytes());
        frame(&mut hasher, &self.rulebook_digest.0);
        frame(&mut hasher, self.root_manifest_profile.as_bytes());
        frame(&mut hasher, &self.root_manifest_digest.0);
        frame(&mut hasher, self.adoption_authority_ref.as_bytes());
        frame(&mut hasher, self.adoption_ref.as_bytes());
        frame(&mut hasher, self.adoption_proof_ref.as_bytes());
        Ok(Digest32(*hasher.finalize().as_bytes()))
    }
}

/// Role-specific proof-verifier output.
///
/// This says only that one verifier authenticated the exact adoption claim/proof.
/// It is not itself the #111 root-adoption receipt.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedBootstrapRootAdoptionProof {
    pub protocol_version: String,
    pub adoption_claim_digest: Digest32,
    pub adoption_claim_profile: String,
    pub verified_adoption_authority_ref: String,
    pub verified_adoption_ref: String,
    pub verified_adoption_proof_ref: String,
    pub proof_verifier_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

impl VerifiedBootstrapRootAdoptionProof {
    pub fn validate_at(&self, now_ms: u64) -> Result<(), AdoptionVerifierError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(AdoptionVerifierError::WrongProtocolVersion);
        }
        require_digest(self.adoption_claim_digest)?;
        if self.adoption_claim_profile != ADOPTION_CLAIM_PROFILE {
            return Err(AdoptionVerifierError::WrongClaimProfile);
        }
        require_profile(&self.adoption_claim_profile)?;
        for value in [
            self.verified_adoption_authority_ref.as_str(),
            self.verified_adoption_ref.as_str(),
            self.verified_adoption_proof_ref.as_str(),
            self.proof_verifier_ref.as_str(),
            self.verification_ref.as_str(),
        ] {
            require_ref(value)?;
        }
        if now_ms == 0
            || self.verified_at_ms == 0
            || self.verified_at_ms > now_ms
            || self.valid_until_ms <= now_ms
            || self.valid_until_ms < self.verified_at_ms
        {
            return Err(AdoptionVerifierError::InvalidVerificationWindow);
        }
        Ok(())
    }
}

/// Non-deserializable positive qualification.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedBootstrapRootAdoption {
    adoption: VerifiedBootstrapRootAdoption,
    claim_digest: Digest32,
    claim_profile: String,
    qualification_digest: Digest32,
    qualification_profile: String,
    evidence_digest: Digest32,
    evidence_profile: String,
}

impl QualifiedBootstrapRootAdoption {
    pub fn adoption(&self) -> &VerifiedBootstrapRootAdoption {
        &self.adoption
    }

    pub fn to_verified_adoption(&self) -> VerifiedBootstrapRootAdoption {
        self.adoption.clone()
    }

    pub fn claim_digest(&self) -> Digest32 {
        self.claim_digest
    }

    pub fn claim_profile(&self) -> &str {
        &self.claim_profile
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &str {
        &self.qualification_profile
    }

    pub fn evidence_digest(&self) -> Digest32 {
        self.evidence_digest
    }

    pub fn evidence_profile(&self) -> &str {
        &self.evidence_profile
    }
}

/// Construct the only semantic claim that may be presented to the proof verifier.
pub fn build_adoption_claim(
    manifest: &AuthorityStateBootstrapRootManifest,
    current_constitution: &ConstitutionStatement,
) -> Result<BootstrapRootAdoptionClaim, AdoptionVerifierError> {
    validate_manifest_against_constitution(manifest, current_constitution)?;
    let statement_digest = constitution_digest_to_core(
        current_constitution
            .digest()
            .map_err(|_| AdoptionVerifierError::InvalidCurrentConstitution)?,
    );
    let manifest_digest = manifest
        .identity_digest()
        .map_err(|_| AdoptionVerifierError::InvalidRootManifest)?;

    let claim = BootstrapRootAdoptionClaim {
        constitution_statement_digest: statement_digest,
        constitution_statement_profile: STATEMENT_PROFILE.into(),
        rulebook_ref: manifest.rulebook_ref.clone(),
        rulebook_version: manifest.rulebook_version.clone(),
        rulebook_digest: manifest.rulebook_digest,
        rulebook_profile: manifest.rulebook_profile.clone(),
        root_manifest_digest: manifest_digest,
        root_manifest_profile: ROOT_MANIFEST_PROFILE.into(),
        adoption_authority_ref: manifest.adoption_authority_ref.clone(),
        adoption_ref: manifest.adoption_ref.clone(),
        adoption_proof_ref: manifest.adoption_proof_ref.clone(),
    };
    claim.validate()?;
    Ok(claim)
}

/// Qualify exact root adoption under one exact current constitutional statement.
pub fn qualify_bootstrap_root_adoption(
    manifest: &AuthorityStateBootstrapRootManifest,
    current_constitution: &ConstitutionStatement,
    proof: &VerifiedBootstrapRootAdoptionProof,
    now_ms: u64,
) -> Result<QualifiedBootstrapRootAdoption, AdoptionVerifierError> {
    let claim = build_adoption_claim(manifest, current_constitution)?;
    let claim_digest = claim.identity_digest()?;
    proof.validate_at(now_ms)?;

    if proof.adoption_claim_digest != claim_digest {
        return Err(AdoptionVerifierError::AdoptionClaimMismatch);
    }
    if proof.adoption_claim_profile != ADOPTION_CLAIM_PROFILE {
        return Err(AdoptionVerifierError::WrongClaimProfile);
    }
    if proof.verified_adoption_authority_ref != claim.adoption_authority_ref {
        return Err(AdoptionVerifierError::AdoptionAuthorityMismatch);
    }
    if proof.verified_adoption_ref != claim.adoption_ref {
        return Err(AdoptionVerifierError::AdoptionReferenceMismatch);
    }
    if proof.verified_adoption_proof_ref != claim.adoption_proof_ref {
        return Err(AdoptionVerifierError::AdoptionProofMismatch);
    }

    let valid_until_ms = proof.valid_until_ms.min(manifest.valid_until_ms);
    if valid_until_ms <= now_ms {
        return Err(AdoptionVerifierError::InvalidVerificationWindow);
    }

    let qualification_digest = qualification_digest(claim_digest);
    let evidence_digest = evidence_digest(qualification_digest, proof);
    let verification_ref = format!(
        "bootstrap-root-adoption:{EVIDENCE_PROFILE}:{}",
        hex_digest(evidence_digest)
    );

    let adoption = VerifiedBootstrapRootAdoption {
        protocol_version: ROOT_ADOPTION_RECEIPT_PROTOCOL.into(),
        constitution_statement_digest: claim.constitution_statement_digest,
        constitution_statement_profile: claim.constitution_statement_profile.clone(),
        rulebook_ref: claim.rulebook_ref.clone(),
        rulebook_version: claim.rulebook_version.clone(),
        rulebook_digest: claim.rulebook_digest,
        rulebook_profile: claim.rulebook_profile.clone(),
        root_manifest_digest: claim.root_manifest_digest,
        root_manifest_profile: claim.root_manifest_profile.clone(),
        adoption_authority_ref: claim.adoption_authority_ref.clone(),
        adoption_ref: claim.adoption_ref.clone(),
        adoption_proof_ref: claim.adoption_proof_ref.clone(),
        verified_adoption_proof_ref: proof.verified_adoption_proof_ref.clone(),
        verification_ref,
        verified_at_ms: proof.verified_at_ms,
        valid_until_ms,
    };
    adoption
        .validate_at(now_ms)
        .map_err(|_| AdoptionVerifierError::ProjectedAdoptionInvalid)?;

    Ok(QualifiedBootstrapRootAdoption {
        adoption,
        claim_digest,
        claim_profile: ADOPTION_CLAIM_PROFILE.into(),
        qualification_digest,
        qualification_profile: QUALIFICATION_PROFILE.into(),
        evidence_digest,
        evidence_profile: EVIDENCE_PROFILE.into(),
    })
}

fn validate_manifest_against_constitution(
    manifest: &AuthorityStateBootstrapRootManifest,
    current: &ConstitutionStatement,
) -> Result<(), AdoptionVerifierError> {
    manifest
        .validate()
        .map_err(|_| AdoptionVerifierError::InvalidRootManifest)?;
    current
        .validate()
        .map_err(|_| AdoptionVerifierError::InvalidCurrentConstitution)?;

    let statement_digest = constitution_digest_to_core(
        current
            .digest()
            .map_err(|_| AdoptionVerifierError::InvalidCurrentConstitution)?,
    );
    let rulebook_digest = constitution_digest_to_core(current.rulebook.digest);

    if manifest.network_id != current.network_id.as_str()
        || manifest.institution_id != current.institution_id.as_str()
        || manifest.constitution_id != current.constitution_id.as_str()
        || manifest.constitution_version != current.version
        || manifest.constitution_statement_digest != statement_digest
        || manifest.constitution_statement_profile != STATEMENT_PROFILE
        || manifest.rulebook_ref != current.rulebook_id.as_str()
        || manifest.rulebook_version != current.rulebook_version
        || manifest.rulebook_digest != rulebook_digest
        || manifest.rulebook_profile != current.rulebook.profile
    {
        return Err(AdoptionVerifierError::ManifestConstitutionMismatch);
    }
    Ok(())
}

fn qualification_digest(claim_digest: Digest32) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_QUALIFICATION);
    frame(&mut hasher, QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, ADOPTION_CLAIM_PROFILE.as_bytes());
    frame(&mut hasher, &claim_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

fn evidence_digest(
    qualification_digest: Digest32,
    proof: &VerifiedBootstrapRootAdoptionProof,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_EVIDENCE);
    frame(&mut hasher, EVIDENCE_PROFILE.as_bytes());
    frame(&mut hasher, &qualification_digest.0);
    frame(&mut hasher, proof.proof_verifier_ref.as_bytes());
    frame(&mut hasher, proof.verification_ref.as_bytes());
    frame(&mut hasher, &proof.verified_at_ms.to_le_bytes());
    frame(&mut hasher, &proof.valid_until_ms.to_le_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

fn constitution_digest_to_core(
    digest: mycelix_governance_constitution::Digest32,
) -> Digest32 {
    Digest32(digest.0)
}

fn require_digest(digest: Digest32) -> Result<(), AdoptionVerifierError> {
    if digest.is_zero() {
        Err(AdoptionVerifierError::ZeroDigest)
    } else {
        Ok(())
    }
}

fn require_ref(value: &str) -> Result<(), AdoptionVerifierError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(AdoptionVerifierError::InvalidReference)
    } else {
        Ok(())
    }
}

fn require_profile(value: &str) -> Result<(), AdoptionVerifierError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        Err(AdoptionVerifierError::InvalidProfile)
    } else {
        Ok(())
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn hex_digest(digest: Digest32) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(64);
    for byte in digest.0 {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AdoptionVerifierError {
    WrongProtocolVersion,
    WrongIdentityProfile,
    WrongClaimProfile,
    InvalidReference,
    InvalidProfile,
    ZeroDigest,
    InvalidVerificationWindow,
    InvalidCurrentConstitution,
    InvalidRootManifest,
    ManifestConstitutionMismatch,
    AdoptionClaimMismatch,
    AdoptionAuthorityMismatch,
    AdoptionReferenceMismatch,
    AdoptionProofMismatch,
    ProjectedAdoptionInvalid,
}

impl fmt::Display for AdoptionVerifierError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::WrongProtocolVersion => "wrong bootstrap-root adoption verifier protocol",
            Self::WrongIdentityProfile => "wrong bootstrap-root adoption semantic identity profile",
            Self::WrongClaimProfile => "wrong bootstrap-root adoption claim profile",
            Self::InvalidReference => "invalid bootstrap-root adoption verifier reference",
            Self::InvalidProfile => "invalid bootstrap-root adoption verifier profile",
            Self::ZeroDigest => "bootstrap-root adoption verifier digest must not be zero",
            Self::InvalidVerificationWindow => "bootstrap-root adoption proof is not currently usable",
            Self::InvalidCurrentConstitution => "invalid current constitution for root adoption",
            Self::InvalidRootManifest => "invalid bootstrap-root manifest for adoption",
            Self::ManifestConstitutionMismatch => "bootstrap-root manifest does not belong to the exact current constitution/rulebook",
            Self::AdoptionClaimMismatch => "verified proof belongs to another root-adoption claim",
            Self::AdoptionAuthorityMismatch => "verified proof authenticated another adoption authority",
            Self::AdoptionReferenceMismatch => "verified proof authenticated another adoption reference",
            Self::AdoptionProofMismatch => "verified proof authenticated another adoption proof reference",
            Self::ProjectedAdoptionInvalid => "qualified root-adoption projection is invalid",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for AdoptionVerifierError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{AuthoritySubjectKind, ProfiledDigest as FreshnessProfiledDigest};
    use mycelix_authority_state_coverage::{
        AuthorityCoveragePolicy, CoverageMode, POLICY_IDENTITY_PROFILE,
        PROTOCOL_VERSION as COVERAGE_PROTOCOL,
    };
    use mycelix_authority_state_coverage_context::{
        CoverageTrustContextPolicy, PROTOCOL_VERSION as CONTEXT_PROTOCOL,
    };
    use mycelix_governance_constitution::{
        ConstitutionId, Digest32 as ConstitutionDigest32, InstitutionId, NetworkId,
        ProfiledDigest as ConstitutionProfiledDigest, RulebookId,
        PROTOCOL_VERSION as CONSTITUTION_PROTOCOL,
    };

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn gd(byte: u8) -> ConstitutionDigest32 {
        ConstitutionDigest32([byte; 32])
    }

    fn current() -> ConstitutionStatement {
        ConstitutionStatement {
            protocol_version: CONSTITUTION_PROTOCOL.into(),
            network_id: NetworkId::new("network:test").unwrap(),
            institution_id: InstitutionId::new("institution:test").unwrap(),
            constitution_id: ConstitutionId::new("constitution:test").unwrap(),
            version: 2,
            parent_statement_digest: Some(gd(1)),
            rulebook_id: RulebookId::new("rulebook:test").unwrap(),
            rulebook_version: "v2".into(),
            rulebook: ConstitutionProfiledDigest {
                digest: gd(2),
                profile: "rulebook-v1-blake3".into(),
            },
            charter: ConstitutionProfiledDigest {
                digest: gd(3),
                profile: "charter-v1-blake3".into(),
            },
            parameters: ConstitutionProfiledDigest {
                digest: gd(4),
                profile: "parameters-v1-blake3".into(),
            },
            amendment_policy: ConstitutionProfiledDigest {
                digest: gd(5),
                profile: "amendment-v1-blake3".into(),
            },
            binding_vote_profile: "binding-vote-v1".into(),
            threshold_authority_profile: "threshold-v1".into(),
            effective_from_ms: 1_000,
        }
    }

    fn manifest(current: &ConstitutionStatement) -> AuthorityStateBootstrapRootManifest {
        let coverage = AuthorityCoveragePolicy {
            protocol_version: COVERAGE_PROTOCOL.into(),
            policy_id: "root-coverage:test".into(),
            namespace: "authority-control:test".into(),
            allowed_subject_kinds: vec![
                AuthoritySubjectKind::AuthorityCoveragePolicy,
                AuthoritySubjectKind::CoverageTrustContextPolicy,
            ],
            authoritative_source_ref: "authority-source:root".into(),
            source_identity: FreshnessProfiledDigest {
                digest: d(20),
                profile: "source-key-v1".into(),
            },
            mode: CoverageMode::DirectSource,
            max_source_age_ms: 500,
            max_witness_age_ms: 0,
            max_coverage_lease_ms: 500,
            valid_from_ms: 900,
            valid_until_ms: 9_000,
            authority_ref: "rulebook-adoption:authority".into(),
            policy_proof_ref: "proof:root-coverage".into(),
        };
        let coverage_digest = coverage.identity_digest().unwrap();
        let context = CoverageTrustContextPolicy {
            protocol_version: CONTEXT_PROTOCOL.into(),
            context_policy_id: "root-context:test".into(),
            institution_ref: "institution:test".into(),
            jurisdiction_ref: None,
            rulebook_ref: "rulebook:test".into(),
            coverage_policy: FreshnessProfiledDigest {
                digest: coverage_digest,
                profile: POLICY_IDENTITY_PROFILE.into(),
            },
            witness_trust_policy: None,
            witness_trust_verifier_ref: None,
            max_challenge_lifetime_ms: 250,
            valid_from_ms: 900,
            valid_until_ms: 8_500,
            authority_ref: "rulebook-adoption:authority".into(),
            policy_proof_ref: "proof:root-context".into(),
        };
        AuthorityStateBootstrapRootManifest {
            protocol_version: mycelix_authority_state_bootstrap_root::PROTOCOL_VERSION.into(),
            network_id: current.network_id.as_str().into(),
            institution_id: current.institution_id.as_str().into(),
            constitution_id: current.constitution_id.as_str().into(),
            constitution_version: current.version,
            constitution_statement_digest: constitution_digest_to_core(current.digest().unwrap()),
            constitution_statement_profile: STATEMENT_PROFILE.into(),
            rulebook_ref: current.rulebook_id.as_str().into(),
            rulebook_version: current.rulebook_version.clone(),
            rulebook_digest: constitution_digest_to_core(current.rulebook.digest),
            rulebook_profile: current.rulebook.profile.clone(),
            control_plane_namespace: "authority-control:test".into(),
            coverage_policy: coverage,
            coverage_context_policy: context,
            root_epoch: 2,
            effective_from_ms: 1_000,
            valid_until_ms: 8_000,
            adoption_authority_ref: "rulebook-adoption:authority".into(),
            adoption_ref: "rulebook-adoption:root:2".into(),
            adoption_proof_ref: "proof:root-adoption:2".into(),
        }
    }

    fn proof(
        claim: &BootstrapRootAdoptionClaim,
        verified_at_ms: u64,
        valid_until_ms: u64,
        verification_ref: &str,
    ) -> VerifiedBootstrapRootAdoptionProof {
        VerifiedBootstrapRootAdoptionProof {
            protocol_version: PROTOCOL_VERSION.into(),
            adoption_claim_digest: claim.identity_digest().unwrap(),
            adoption_claim_profile: ADOPTION_CLAIM_PROFILE.into(),
            verified_adoption_authority_ref: claim.adoption_authority_ref.clone(),
            verified_adoption_ref: claim.adoption_ref.clone(),
            verified_adoption_proof_ref: claim.adoption_proof_ref.clone(),
            proof_verifier_ref: "verifier:rulebook-adoption".into(),
            verification_ref: verification_ref.into(),
            verified_at_ms,
            valid_until_ms,
        }
    }

    #[test]
    fn qualifies_exact_current_rulebook_root_adoption() {
        let current = current();
        let manifest = manifest(&current);
        let claim = build_adoption_claim(&manifest, &current).unwrap();
        let proof = proof(&claim, 1_500, 3_000, "verify:adoption:1");
        let qualified = qualify_bootstrap_root_adoption(&manifest, &current, &proof, 2_000).unwrap();
        assert_eq!(qualified.adoption().root_manifest_digest, claim.root_manifest_digest);
        assert_eq!(qualified.adoption().valid_until_ms, 3_000);
    }

    #[test]
    fn proof_for_another_manifest_denies() {
        let current = current();
        let manifest = manifest(&current);
        let claim = build_adoption_claim(&manifest, &current).unwrap();
        let proof = proof(&claim, 1_500, 3_000, "verify:adoption:1");
        let mut changed = manifest.clone();
        changed.root_epoch = 3;
        assert_eq!(
            qualify_bootstrap_root_adoption(&changed, &current, &proof, 2_000).unwrap_err(),
            AdoptionVerifierError::AdoptionClaimMismatch
        );
    }

    #[test]
    fn wrong_adoption_authority_denies() {
        let current = current();
        let manifest = manifest(&current);
        let claim = build_adoption_claim(&manifest, &current).unwrap();
        let mut proof = proof(&claim, 1_500, 3_000, "verify:adoption:1");
        proof.verified_adoption_authority_ref = "rulebook-adoption:other".into();
        assert_eq!(
            qualify_bootstrap_root_adoption(&manifest, &current, &proof, 2_000).unwrap_err(),
            AdoptionVerifierError::AdoptionAuthorityMismatch
        );
    }

    #[test]
    fn constitutional_change_invalidates_manifest_adoption() {
        let old = current();
        let manifest = manifest(&old);
        let claim = build_adoption_claim(&manifest, &old).unwrap();
        let proof = proof(&claim, 1_500, 3_000, "verify:adoption:1");
        let mut newer = old.clone();
        newer.version = 3;
        newer.parent_statement_digest = Some(old.digest().unwrap());
        newer.effective_from_ms = 1_800;
        assert_eq!(
            qualify_bootstrap_root_adoption(&manifest, &newer, &proof, 2_000).unwrap_err(),
            AdoptionVerifierError::ManifestConstitutionMismatch
        );
    }

    #[test]
    fn refreshed_proof_preserves_semantic_qualification_but_changes_evidence() {
        let current = current();
        let manifest = manifest(&current);
        let claim = build_adoption_claim(&manifest, &current).unwrap();
        let first = proof(&claim, 1_500, 3_000, "verify:adoption:1");
        let second = proof(&claim, 1_700, 3_200, "verify:adoption:2");
        let a = qualify_bootstrap_root_adoption(&manifest, &current, &first, 2_000).unwrap();
        let b = qualify_bootstrap_root_adoption(&manifest, &current, &second, 2_000).unwrap();
        assert_eq!(a.qualification_digest(), b.qualification_digest());
        assert_ne!(a.evidence_digest(), b.evidence_digest());
    }
}
