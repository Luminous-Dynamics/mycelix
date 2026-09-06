// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Constitution-rooted non-circular bootstrap authority for authority-state coverage.
//!
//! The operational coverage plane must not prove its own currentness. This pure
//! layer roots only the control-plane policy classes in one exact, independently
//! verified current constitutional statement plus one exact independently verified
//! rulebook adoption of the root manifest.
//!
//! No `VerifiedAuthorityFreshness` is accepted here. Probe challenges remain
//! evidence collection, not authority. Operational authority subjects are forbidden.

use mycelix_authority_freshness::AuthoritySubjectKind;
use mycelix_authority_state_coverage::{
    AuthorityCoveragePolicy, CoverageMode, POLICY_IDENTITY_PROFILE,
};
use mycelix_authority_state_coverage_context::{
    CoverageTrustContextPolicy, CONTEXT_POLICY_PROFILE,
};
use mycelix_governance_constitution::{ConstitutionStatement, STATEMENT_PROFILE};
use mycelix_institutional_core::Digest32;
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-state-bootstrap-root-v0.1";
pub const ROOT_MANIFEST_PROFILE: &str =
    "mycelix-authority-state-bootstrap-root-manifest-v1-blake3-framed";
pub const ROOT_QUALIFICATION_PROFILE: &str =
    "mycelix-authority-state-bootstrap-root-qualification-v1-blake3-framed";
pub const CURRENT_CONSTITUTION_RECEIPT_PROTOCOL: &str =
    "mycelix-current-constitution-receipt-v0.1";
pub const ROOT_ADOPTION_RECEIPT_PROTOCOL: &str =
    "mycelix-authority-state-bootstrap-root-adoption-v0.1";

const DOMAIN_MANIFEST: &[u8] = b"mycelix/authority-state/bootstrap-root/manifest/v1";
const DOMAIN_QUALIFICATION: &[u8] = b"mycelix/authority-state/bootstrap-root/qualification/v1";
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;
const MAX_DNA_HASH_BYTES: usize = 1024;

/// Evidence-shaped receipt that one exact constitutional statement is current.
///
/// A runtime adapter must construct this only after independently verifying the
/// constitutional lineage/current-head result. Deserialization is not authority.
/// `dna_hash` is carried forward for the separate deployment binding performed by
/// the runtime-DNA layer; it is not folded into the substrate-neutral root identity.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedCurrentConstitutionReceipt {
    pub protocol_version: String,
    pub statement: ConstitutionStatement,
    pub statement_digest: Digest32,
    pub statement_profile: String,
    pub dna_hash: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

impl VerifiedCurrentConstitutionReceipt {
    pub fn validate_at(&self, now_ms: u64) -> Result<(), BootstrapRootError> {
        if self.protocol_version != CURRENT_CONSTITUTION_RECEIPT_PROTOCOL {
            return Err(BootstrapRootError::WrongCurrentConstitutionProtocol);
        }
        self.statement
            .validate()
            .map_err(|_| BootstrapRootError::InvalidCurrentConstitution)?;
        if self.statement_profile != STATEMENT_PROFILE {
            return Err(BootstrapRootError::LegacyConstitutionRejected);
        }
        let recomputed = constitution_digest_to_core(
            self.statement
                .digest()
                .map_err(|_| BootstrapRootError::InvalidCurrentConstitution)?,
        );
        if self.statement_digest.is_zero() || self.statement_digest != recomputed {
            return Err(BootstrapRootError::CurrentConstitutionDigestMismatch);
        }
        validate_dna_hash(&self.dna_hash)?;
        require_ref(&self.verification_ref)?;
        if now_ms == 0
            || self.verified_at_ms == 0
            || self.verified_at_ms < self.statement.effective_from_ms
            || self.verified_at_ms > now_ms
            || self.valid_until_ms <= now_ms
            || self.valid_until_ms < self.verified_at_ms
        {
            return Err(BootstrapRootError::CurrentConstitutionNotCurrent);
        }
        Ok(())
    }
}

/// Semantic constitution/rulebook-adopted bootstrap manifest.
///
/// The exact root source identity and coverage semantics are committed indirectly
/// but completely through `coverage_policy.identity_digest()`. Witness/trust
/// semantics are committed through the exact context-policy identity.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityStateBootstrapRootManifest {
    pub protocol_version: String,
    pub network_id: String,
    pub institution_id: String,
    pub constitution_id: String,
    pub constitution_version: u64,
    pub constitution_statement_digest: Digest32,
    pub constitution_statement_profile: String,
    pub rulebook_ref: String,
    pub rulebook_version: String,
    pub rulebook_digest: Digest32,
    pub rulebook_profile: String,
    pub control_plane_namespace: String,
    pub coverage_policy: AuthorityCoveragePolicy,
    pub coverage_context_policy: CoverageTrustContextPolicy,
    pub root_epoch: u64,
    pub effective_from_ms: u64,
    pub valid_until_ms: u64,
    pub adoption_authority_ref: String,
    pub adoption_ref: String,
    pub adoption_proof_ref: String,
}

impl AuthorityStateBootstrapRootManifest {
    pub fn validate(&self) -> Result<(), BootstrapRootError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(BootstrapRootError::WrongProtocolVersion);
        }
        for value in [
            self.network_id.as_str(),
            self.institution_id.as_str(),
            self.constitution_id.as_str(),
            self.rulebook_ref.as_str(),
            self.rulebook_version.as_str(),
            self.control_plane_namespace.as_str(),
            self.adoption_authority_ref.as_str(),
            self.adoption_ref.as_str(),
            self.adoption_proof_ref.as_str(),
        ] {
            require_ref(value)?;
        }
        if self.constitution_version == 0 || self.root_epoch == 0 {
            return Err(BootstrapRootError::InvalidEpoch);
        }
        require_digest(self.constitution_statement_digest)?;
        require_profile(&self.constitution_statement_profile)?;
        if self.constitution_statement_profile != STATEMENT_PROFILE {
            return Err(BootstrapRootError::LegacyConstitutionRejected);
        }
        require_digest(self.rulebook_digest)?;
        require_profile(&self.rulebook_profile)?;
        if self.effective_from_ms == 0 || self.valid_until_ms <= self.effective_from_ms {
            return Err(BootstrapRootError::InvalidRootLifetime);
        }

        self.coverage_policy
            .validate()
            .map_err(|_| BootstrapRootError::InvalidCoveragePolicy)?;
        self.coverage_context_policy
            .validate()
            .map_err(|_| BootstrapRootError::InvalidContextPolicy)?;

        validate_embedded_policy_contract(self)?;
        Ok(())
    }

    pub fn identity_digest(&self) -> Result<Digest32, BootstrapRootError> {
        self.validate()?;
        let coverage_digest = self
            .coverage_policy
            .identity_digest()
            .map_err(|_| BootstrapRootError::InvalidCoveragePolicy)?;
        let context_digest = self
            .coverage_context_policy
            .identity_digest()
            .map_err(|_| BootstrapRootError::InvalidContextPolicy)?;

        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_MANIFEST);
        frame(&mut hasher, ROOT_MANIFEST_PROFILE.as_bytes());
        frame(&mut hasher, self.protocol_version.as_bytes());
        frame(&mut hasher, self.network_id.as_bytes());
        frame(&mut hasher, self.institution_id.as_bytes());
        frame(&mut hasher, self.constitution_id.as_bytes());
        frame(&mut hasher, &self.constitution_version.to_le_bytes());
        frame(&mut hasher, self.constitution_statement_profile.as_bytes());
        frame(&mut hasher, &self.constitution_statement_digest.0);
        frame(&mut hasher, self.rulebook_ref.as_bytes());
        frame(&mut hasher, self.rulebook_version.as_bytes());
        frame(&mut hasher, self.rulebook_profile.as_bytes());
        frame(&mut hasher, &self.rulebook_digest.0);
        frame(&mut hasher, self.control_plane_namespace.as_bytes());
        frame(&mut hasher, POLICY_IDENTITY_PROFILE.as_bytes());
        frame(&mut hasher, &coverage_digest.0);
        frame(&mut hasher, CONTEXT_POLICY_PROFILE.as_bytes());
        frame(&mut hasher, &context_digest.0);
        frame(&mut hasher, &self.root_epoch.to_le_bytes());
        frame(&mut hasher, &self.effective_from_ms.to_le_bytes());
        frame(&mut hasher, &self.valid_until_ms.to_le_bytes());
        frame(&mut hasher, self.adoption_authority_ref.as_bytes());
        frame(&mut hasher, self.adoption_ref.as_bytes());
        frame(&mut hasher, self.adoption_proof_ref.as_bytes());
        Ok(Digest32(*hasher.finalize().as_bytes()))
    }
}

/// Evidence-shaped proof that the exact current rulebook adopted the exact root.
///
/// This receipt is intentionally deserializable. A runtime adapter must obtain it
/// from a verifier independent of the root-manifest bytes being qualified.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedBootstrapRootAdoption {
    pub protocol_version: String,
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
    pub verified_adoption_proof_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

impl VerifiedBootstrapRootAdoption {
    pub fn validate_at(&self, now_ms: u64) -> Result<(), BootstrapRootError> {
        if self.protocol_version != ROOT_ADOPTION_RECEIPT_PROTOCOL {
            return Err(BootstrapRootError::WrongAdoptionProtocol);
        }
        for digest in [
            self.constitution_statement_digest,
            self.rulebook_digest,
            self.root_manifest_digest,
        ] {
            require_digest(digest)?;
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
            self.verified_adoption_proof_ref.as_str(),
            self.verification_ref.as_str(),
        ] {
            require_ref(value)?;
        }
        if self.adoption_proof_ref != self.verified_adoption_proof_ref {
            return Err(BootstrapRootError::AdoptionProofMismatch);
        }
        if now_ms == 0
            || self.verified_at_ms == 0
            || self.verified_at_ms > now_ms
            || self.valid_until_ms <= now_ms
            || self.valid_until_ms < self.verified_at_ms
        {
            return Err(BootstrapRootError::AdoptionNotCurrent);
        }
        Ok(())
    }
}

/// Non-deserializable positive bootstrap authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedAuthorityStateBootstrapRoot {
    manifest: AuthorityStateBootstrapRootManifest,
    current_constitution_digest: Digest32,
    coverage_policy_digest: Digest32,
    context_policy_digest: Digest32,
    qualification_digest: Digest32,
    qualification_profile: String,
    verification_ref: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedAuthorityStateBootstrapRoot {
    pub fn manifest(&self) -> &AuthorityStateBootstrapRootManifest {
        &self.manifest
    }

    pub fn current_constitution_digest(&self) -> Digest32 {
        self.current_constitution_digest
    }

    pub fn coverage_policy_digest(&self) -> Digest32 {
        self.coverage_policy_digest
    }

    pub fn context_policy_digest(&self) -> Digest32 {
        self.context_policy_digest
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &str {
        &self.qualification_profile
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
}

/// Qualify one exact current constitution-rooted authority-state bootstrap root.
pub fn qualify_bootstrap_root(
    manifest: &AuthorityStateBootstrapRootManifest,
    current_constitution: &VerifiedCurrentConstitutionReceipt,
    adoption: &VerifiedBootstrapRootAdoption,
    now_ms: u64,
) -> Result<QualifiedAuthorityStateBootstrapRoot, BootstrapRootError> {
    manifest.validate()?;
    current_constitution.validate_at(now_ms)?;
    adoption.validate_at(now_ms)?;

    if now_ms < manifest.effective_from_ms || now_ms >= manifest.valid_until_ms {
        return Err(BootstrapRootError::RootNotCurrent);
    }
    if !manifest.coverage_policy.is_active_at(now_ms)
        || !manifest.coverage_context_policy.active_at(now_ms)
    {
        return Err(BootstrapRootError::RootPolicyNotCurrent);
    }

    let statement = &current_constitution.statement;
    let statement_digest = constitution_digest_to_core(
        statement
            .digest()
            .map_err(|_| BootstrapRootError::InvalidCurrentConstitution)?,
    );
    let statement_rulebook_digest = constitution_digest_to_core(statement.rulebook.digest);

    if manifest.network_id != statement.network_id.as_str()
        || manifest.institution_id != statement.institution_id.as_str()
        || manifest.constitution_id != statement.constitution_id.as_str()
        || manifest.constitution_version != statement.version
        || manifest.constitution_statement_digest != statement_digest
        || manifest.constitution_statement_digest != current_constitution.statement_digest
        || manifest.constitution_statement_profile != current_constitution.statement_profile
        || manifest.constitution_statement_profile != STATEMENT_PROFILE
        || manifest.rulebook_ref != statement.rulebook_id.as_str()
        || manifest.rulebook_version != statement.rulebook_version
        || manifest.rulebook_digest != statement_rulebook_digest
        || manifest.rulebook_profile != statement.rulebook.profile
    {
        return Err(BootstrapRootError::ManifestConstitutionMismatch);
    }

    let manifest_digest = manifest.identity_digest()?;
    if adoption.constitution_statement_digest != statement_digest
        || adoption.constitution_statement_profile != STATEMENT_PROFILE
        || adoption.rulebook_ref != manifest.rulebook_ref
        || adoption.rulebook_version != manifest.rulebook_version
        || adoption.rulebook_digest != manifest.rulebook_digest
        || adoption.rulebook_profile != manifest.rulebook_profile
        || adoption.root_manifest_digest != manifest_digest
        || adoption.root_manifest_profile != ROOT_MANIFEST_PROFILE
        || adoption.adoption_authority_ref != manifest.adoption_authority_ref
        || adoption.adoption_ref != manifest.adoption_ref
        || adoption.adoption_proof_ref != manifest.adoption_proof_ref
        || adoption.verified_adoption_proof_ref != manifest.adoption_proof_ref
    {
        return Err(BootstrapRootError::AdoptionBindingMismatch);
    }

    let coverage_policy_digest = manifest
        .coverage_policy
        .identity_digest()
        .map_err(|_| BootstrapRootError::InvalidCoveragePolicy)?;
    let context_policy_digest = manifest
        .coverage_context_policy
        .identity_digest()
        .map_err(|_| BootstrapRootError::InvalidContextPolicy)?;

    let verified_at_ms = current_constitution
        .verified_at_ms
        .max(adoption.verified_at_ms)
        .max(manifest.effective_from_ms);
    let valid_until_ms = current_constitution
        .valid_until_ms
        .min(adoption.valid_until_ms)
        .min(manifest.valid_until_ms)
        .min(manifest.coverage_policy.valid_until_ms)
        .min(manifest.coverage_context_policy.valid_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(BootstrapRootError::RootNotCurrent);
    }

    let qualification_digest = qualification_digest(
        manifest_digest,
        statement_digest,
        coverage_policy_digest,
        context_policy_digest,
    );
    let verification_ref = format!(
        "authority-bootstrap-root:{ROOT_QUALIFICATION_PROFILE}:{}",
        hex_digest(qualification_digest)
    );

    Ok(QualifiedAuthorityStateBootstrapRoot {
        manifest: manifest.clone(),
        current_constitution_digest: statement_digest,
        coverage_policy_digest,
        context_policy_digest,
        qualification_digest,
        qualification_profile: ROOT_QUALIFICATION_PROFILE.into(),
        verification_ref,
        verified_at_ms,
        valid_until_ms,
    })
}

fn validate_embedded_policy_contract(
    manifest: &AuthorityStateBootstrapRootManifest,
) -> Result<(), BootstrapRootError> {
    let coverage = &manifest.coverage_policy;
    let context = &manifest.coverage_context_policy;

    if coverage.namespace != manifest.control_plane_namespace {
        return Err(BootstrapRootError::ControlPlaneNamespaceMismatch);
    }
    if coverage.authority_ref != manifest.adoption_authority_ref
        || context.authority_ref != manifest.adoption_authority_ref
    {
        return Err(BootstrapRootError::RootAuthorityMismatch);
    }
    if context.institution_ref != manifest.institution_id
        || context.rulebook_ref != manifest.rulebook_ref
    {
        return Err(BootstrapRootError::InstitutionRulebookMismatch);
    }

    let coverage_digest = coverage
        .identity_digest()
        .map_err(|_| BootstrapRootError::InvalidCoveragePolicy)?;
    if context.coverage_policy.digest != coverage_digest
        || context.coverage_policy.profile != POLICY_IDENTITY_PROFILE
    {
        return Err(BootstrapRootError::ContextCoverageBindingMismatch);
    }

    if !coverage.is_active_at(manifest.effective_from_ms)
        || !context.active_at(manifest.effective_from_ms)
    {
        return Err(BootstrapRootError::RootPolicyNotEffectiveAtEpoch);
    }

    validate_control_plane_subject_set(&coverage.allowed_subject_kinds, &coverage.mode)?;

    match (
        &coverage.mode,
        &context.witness_trust_policy,
        &context.witness_trust_verifier_ref,
    ) {
        (CoverageMode::DirectSource, None, None) => {}
        (CoverageMode::DirectSource, _, _) => {
            return Err(BootstrapRootError::UnexpectedWitnessTrustPolicy);
        }
        (CoverageMode::WitnessQuorum { .. }, Some(_), Some(_)) => {}
        (CoverageMode::WitnessQuorum { .. }, _, _) => {
            return Err(BootstrapRootError::MissingWitnessTrustPolicy);
        }
    }

    Ok(())
}

fn validate_control_plane_subject_set(
    kinds: &[AuthoritySubjectKind],
    mode: &CoverageMode,
) -> Result<(), BootstrapRootError> {
    let mut coverage = false;
    let mut context = false;
    let mut witness = false;

    for kind in kinds {
        match kind {
            AuthoritySubjectKind::AuthorityCoveragePolicy => {
                if coverage {
                    return Err(BootstrapRootError::InvalidControlPlaneSubjectSet);
                }
                coverage = true;
            }
            AuthoritySubjectKind::CoverageTrustContextPolicy => {
                if context {
                    return Err(BootstrapRootError::InvalidControlPlaneSubjectSet);
                }
                context = true;
            }
            AuthoritySubjectKind::WitnessTrustPolicy => {
                if witness {
                    return Err(BootstrapRootError::InvalidControlPlaneSubjectSet);
                }
                witness = true;
            }
            _ => {
                // operational subject kind is forbidden in the bootstrap root.
                return Err(BootstrapRootError::OperationalSubjectInBootstrapRoot);
            }
        }
    }

    let requires_witness = matches!(mode, CoverageMode::WitnessQuorum { .. });
    if !coverage || !context || witness != requires_witness {
        return Err(BootstrapRootError::InvalidControlPlaneSubjectSet);
    }
    Ok(())
}

fn qualification_digest(
    manifest_digest: Digest32,
    statement_digest: Digest32,
    coverage_policy_digest: Digest32,
    context_policy_digest: Digest32,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_QUALIFICATION);
    frame(&mut hasher, ROOT_QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, ROOT_MANIFEST_PROFILE.as_bytes());
    frame(&mut hasher, &manifest_digest.0);
    frame(&mut hasher, STATEMENT_PROFILE.as_bytes());
    frame(&mut hasher, &statement_digest.0);
    frame(&mut hasher, POLICY_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, &coverage_policy_digest.0);
    frame(&mut hasher, CONTEXT_POLICY_PROFILE.as_bytes());
    frame(&mut hasher, &context_policy_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

fn constitution_digest_to_core(
    digest: mycelix_governance_constitution::Digest32,
) -> Digest32 {
    Digest32(digest.0)
}

fn require_digest(digest: Digest32) -> Result<(), BootstrapRootError> {
    if digest.is_zero() {
        Err(BootstrapRootError::ZeroDigest)
    } else {
        Ok(())
    }
}

fn require_ref(value: &str) -> Result<(), BootstrapRootError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(BootstrapRootError::InvalidReference)
    } else {
        Ok(())
    }
}

fn require_profile(value: &str) -> Result<(), BootstrapRootError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        Err(BootstrapRootError::InvalidProfile)
    } else {
        Ok(())
    }
}

fn validate_dna_hash(value: &str) -> Result<(), BootstrapRootError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_DNA_HASH_BYTES
        || bytes.iter().any(|byte| byte.is_ascii_whitespace())
    {
        Err(BootstrapRootError::InvalidDnaHash)
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
pub enum BootstrapRootError {
    WrongProtocolVersion,
    WrongCurrentConstitutionProtocol,
    WrongAdoptionProtocol,
    InvalidReference,
    InvalidProfile,
    InvalidDnaHash,
    ZeroDigest,
    InvalidEpoch,
    InvalidRootLifetime,
    InvalidCurrentConstitution,
    LegacyConstitutionRejected,
    CurrentConstitutionDigestMismatch,
    CurrentConstitutionNotCurrent,
    InvalidCoveragePolicy,
    InvalidContextPolicy,
    ControlPlaneNamespaceMismatch,
    RootAuthorityMismatch,
    InstitutionRulebookMismatch,
    ContextCoverageBindingMismatch,
    RootPolicyNotEffectiveAtEpoch,
    OperationalSubjectInBootstrapRoot,
    InvalidControlPlaneSubjectSet,
    UnexpectedWitnessTrustPolicy,
    MissingWitnessTrustPolicy,
    AdoptionProofMismatch,
    AdoptionNotCurrent,
    ManifestConstitutionMismatch,
    AdoptionBindingMismatch,
    RootPolicyNotCurrent,
    RootNotCurrent,
}

impl fmt::Display for BootstrapRootError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::WrongProtocolVersion => "wrong authority-state bootstrap-root protocol",
            Self::WrongCurrentConstitutionProtocol => "wrong current-constitution receipt protocol",
            Self::WrongAdoptionProtocol => "wrong bootstrap-root adoption receipt protocol",
            Self::InvalidReference => "invalid bootstrap-root reference",
            Self::InvalidProfile => "invalid bootstrap-root digest profile",
            Self::InvalidDnaHash => "invalid current-constitution DNA hash",
            Self::ZeroDigest => "bootstrap-root digest must not be zero",
            Self::InvalidEpoch => "bootstrap-root constitution/root epoch must be non-zero",
            Self::InvalidRootLifetime => "invalid bootstrap-root lifetime",
            Self::InvalidCurrentConstitution => "invalid current constitutional statement",
            Self::LegacyConstitutionRejected => "legacy/non-statement constitutional identity is rejected",
            Self::CurrentConstitutionDigestMismatch => "current constitutional statement digest mismatch",
            Self::CurrentConstitutionNotCurrent => "current constitutional statement receipt is stale or invalid",
            Self::InvalidCoveragePolicy => "invalid bootstrap authority coverage policy",
            Self::InvalidContextPolicy => "invalid bootstrap coverage trust-context policy",
            Self::ControlPlaneNamespaceMismatch => "bootstrap coverage namespace is not the control-plane namespace",
            Self::RootAuthorityMismatch => "embedded root policies do not share the exact adoption authority",
            Self::InstitutionRulebookMismatch => "embedded root context belongs to another institution or rulebook",
            Self::ContextCoverageBindingMismatch => "bootstrap context does not bind the exact coverage policy",
            Self::RootPolicyNotEffectiveAtEpoch => "bootstrap policy was not active at the root effective epoch",
            Self::OperationalSubjectInBootstrapRoot => "operational subject kind is forbidden in the bootstrap root",
            Self::InvalidControlPlaneSubjectSet => "bootstrap root does not contain the exact required control-plane policy classes",
            Self::UnexpectedWitnessTrustPolicy => "direct-source bootstrap root forbids witness trust policy",
            Self::MissingWitnessTrustPolicy => "witness-quorum bootstrap root requires witness trust policy and verifier",
            Self::AdoptionProofMismatch => "bootstrap-root adoption proof echo mismatch",
            Self::AdoptionNotCurrent => "bootstrap-root adoption receipt is stale or invalid",
            Self::ManifestConstitutionMismatch => "bootstrap manifest does not match the exact current constitutional statement/rulebook",
            Self::AdoptionBindingMismatch => "root adoption does not bind the exact current constitution/rulebook/manifest",
            Self::RootPolicyNotCurrent => "embedded bootstrap policy is no longer semantically current",
            Self::RootNotCurrent => "qualified bootstrap root is outside its conservative live window",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for BootstrapRootError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::ProfiledDigest as FreshnessProfiledDigest;
    use mycelix_authority_state_coverage::PROTOCOL_VERSION as COVERAGE_PROTOCOL;
    use mycelix_authority_state_coverage_context::PROTOCOL_VERSION as CONTEXT_PROTOCOL;
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

    fn constitution() -> ConstitutionStatement {
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

    fn policies() -> (AuthorityCoveragePolicy, CoverageTrustContextPolicy) {
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
        (coverage, context)
    }

    fn manifest(statement: &ConstitutionStatement) -> AuthorityStateBootstrapRootManifest {
        let (coverage_policy, coverage_context_policy) = policies();
        AuthorityStateBootstrapRootManifest {
            protocol_version: PROTOCOL_VERSION.into(),
            network_id: statement.network_id.as_str().into(),
            institution_id: statement.institution_id.as_str().into(),
            constitution_id: statement.constitution_id.as_str().into(),
            constitution_version: statement.version,
            constitution_statement_digest: constitution_digest_to_core(statement.digest().unwrap()),
            constitution_statement_profile: STATEMENT_PROFILE.into(),
            rulebook_ref: statement.rulebook_id.as_str().into(),
            rulebook_version: statement.rulebook_version.clone(),
            rulebook_digest: constitution_digest_to_core(statement.rulebook.digest),
            rulebook_profile: statement.rulebook.profile.clone(),
            control_plane_namespace: "authority-control:test".into(),
            coverage_policy,
            coverage_context_policy,
            root_epoch: 2,
            effective_from_ms: 1_000,
            valid_until_ms: 8_000,
            adoption_authority_ref: "rulebook-adoption:authority".into(),
            adoption_ref: "rulebook-adoption:root:2".into(),
            adoption_proof_ref: "proof:root-adoption:2".into(),
        }
    }

    fn current(statement: &ConstitutionStatement) -> VerifiedCurrentConstitutionReceipt {
        VerifiedCurrentConstitutionReceipt {
            protocol_version: CURRENT_CONSTITUTION_RECEIPT_PROTOCOL.into(),
            statement: statement.clone(),
            statement_digest: constitution_digest_to_core(statement.digest().unwrap()),
            statement_profile: STATEMENT_PROFILE.into(),
            dna_hash: "uhC0k-test-dna".into(),
            verification_ref: "constitution-current:2".into(),
            verified_at_ms: 1_100,
            valid_until_ms: 7_500,
        }
    }

    fn adoption(
        statement: &ConstitutionStatement,
        manifest: &AuthorityStateBootstrapRootManifest,
    ) -> VerifiedBootstrapRootAdoption {
        VerifiedBootstrapRootAdoption {
            protocol_version: ROOT_ADOPTION_RECEIPT_PROTOCOL.into(),
            constitution_statement_digest: constitution_digest_to_core(statement.digest().unwrap()),
            constitution_statement_profile: STATEMENT_PROFILE.into(),
            rulebook_ref: manifest.rulebook_ref.clone(),
            rulebook_version: manifest.rulebook_version.clone(),
            rulebook_digest: manifest.rulebook_digest,
            rulebook_profile: manifest.rulebook_profile.clone(),
            root_manifest_digest: manifest.identity_digest().unwrap(),
            root_manifest_profile: ROOT_MANIFEST_PROFILE.into(),
            adoption_authority_ref: manifest.adoption_authority_ref.clone(),
            adoption_ref: manifest.adoption_ref.clone(),
            adoption_proof_ref: manifest.adoption_proof_ref.clone(),
            verified_adoption_proof_ref: manifest.adoption_proof_ref.clone(),
            verification_ref: "verify:root-adoption:2".into(),
            verified_at_ms: 1_200,
            valid_until_ms: 7_000,
        }
    }

    #[test]
    fn exact_current_constitution_and_adoption_qualify() {
        let statement = constitution();
        let manifest = manifest(&statement);
        let current = current(&statement);
        let adoption = adoption(&statement, &manifest);
        let qualified = qualify_bootstrap_root(&manifest, &current, &adoption, 2_000).unwrap();
        assert_eq!(qualified.valid_until_ms(), 7_000);
        assert_eq!(qualified.manifest(), &manifest);
        assert_eq!(qualified.coverage_policy_digest(), manifest.coverage_policy.identity_digest().unwrap());
    }

    #[test]
    fn constitutional_advance_invalidates_old_manifest() {
        let old_statement = constitution();
        let old_manifest = manifest(&old_statement);
        let old_adoption = adoption(&old_statement, &old_manifest);

        let mut new_statement = old_statement.clone();
        new_statement.version = 3;
        new_statement.parent_statement_digest = Some(old_statement.digest().unwrap());
        new_statement.effective_from_ms = 1_500;
        let current = current(&new_statement);

        assert_eq!(
            qualify_bootstrap_root(&old_manifest, &current, &old_adoption, 2_000).unwrap_err(),
            BootstrapRootError::ManifestConstitutionMismatch
        );
    }

    #[test]
    fn operational_subject_kind_is_forbidden() {
        let statement = constitution();
        let mut manifest = manifest(&statement);
        manifest
            .coverage_policy
            .allowed_subject_kinds
            .push(AuthoritySubjectKind::AuthorityGrant);
        assert_eq!(
            manifest.validate().unwrap_err(),
            BootstrapRootError::OperationalSubjectInBootstrapRoot
        );
    }

    #[test]
    fn changed_root_manifest_requires_new_adoption() {
        let statement = constitution();
        let mut manifest = manifest(&statement);
        let current = current(&statement);
        let adoption = adoption(&statement, &manifest);
        manifest.root_epoch = 3;
        assert_eq!(
            qualify_bootstrap_root(&manifest, &current, &adoption, 2_000).unwrap_err(),
            BootstrapRootError::AdoptionBindingMismatch
        );
    }
}
