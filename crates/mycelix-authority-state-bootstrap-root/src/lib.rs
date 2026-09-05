// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Constitution-rooted bootstrap authority for authority-state freshness.
//!
//! The operational coverage plane cannot prove its own currentness without a
//! trust-bootstrap cycle. This crate provides a deliberately narrow control-plane
//! root whose currentness comes from the exact DNA-rooted constitutional state.
//! It does not perform Holochain calls, issue challenges, persist state, verify
//! signatures, or grant execution authority.

use mycelix_authority_freshness::AuthoritySubjectKind;
use mycelix_authority_state_coverage::{
    AuthorityCoveragePolicy, CoverageMode, POLICY_IDENTITY_PROFILE,
};
use mycelix_authority_state_coverage_context::{
    CoverageTrustContextPolicy, CONTEXT_POLICY_PROFILE,
};
use mycelix_governance_constitution::{
    ConstitutionStatement, Digest32 as ConstitutionDigest32, STATEMENT_PROFILE,
};
use mycelix_institutional_core::Digest32;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-state-bootstrap-root-v0.1";
pub const ROOT_IDENTITY_PROFILE: &str =
    "mycelix-authority-state-bootstrap-root-v1-blake3-framed";
pub const ROOT_QUALIFICATION_PROFILE: &str =
    "mycelix-authority-state-qualified-bootstrap-root-v1-blake3-framed";

const DOMAIN_ROOT: &[u8] = b"mycelix/authority-state/bootstrap-root/v1";
const DOMAIN_QUALIFIED: &[u8] = b"mycelix/authority-state/qualified-bootstrap-root/v1";
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;

/// Constitution-adopted control-plane root.
///
/// The embedded policies are not asked to prove their own currentness. Their
/// currentness is inherited only after this exact manifest is qualified against
/// the exact current constitutional statement and rulebook adoption evidence.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityStateBootstrapRootManifest {
    pub protocol_version: String,
    pub network_id: String,
    pub institution_id: String,
    pub constitution_id: String,
    pub constitution_version: u64,
    pub constitution_statement_digest: Digest32,
    pub constitution_statement_profile: String,
    pub rulebook_id: String,
    pub rulebook_version: String,
    pub rulebook_digest: Digest32,
    pub rulebook_profile: String,
    pub rulebook_ref: String,
    pub control_plane_namespace: String,
    pub coverage_policy: AuthorityCoveragePolicy,
    pub coverage_context_policy: CoverageTrustContextPolicy,
    pub effective_from_ms: u64,
    /// Exact institutional/rulebook decision adopting this root manifest.
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
            self.rulebook_id.as_str(),
            self.rulebook_version.as_str(),
            self.rulebook_ref.as_str(),
            self.control_plane_namespace.as_str(),
            self.adoption_ref.as_str(),
            self.adoption_proof_ref.as_str(),
        ] {
            require_ref(value)?;
        }
        if self.constitution_version == 0 || self.effective_from_ms == 0 {
            return Err(BootstrapRootError::InvalidRootEpoch);
        }
        if self.constitution_statement_digest.is_zero() || self.rulebook_digest.is_zero() {
            return Err(BootstrapRootError::ZeroDigest);
        }
        require_profile(&self.constitution_statement_profile)?;
        require_profile(&self.rulebook_profile)?;
        if self.constitution_statement_profile != STATEMENT_PROFILE {
            return Err(BootstrapRootError::WrongConstitutionProfile);
        }

        self.coverage_policy
            .validate()
            .map_err(|_| BootstrapRootError::InvalidCoveragePolicy)?;
        self.coverage_context_policy
            .validate()
            .map_err(|_| BootstrapRootError::InvalidCoverageContextPolicy)?;

        if self.coverage_policy.namespace != self.control_plane_namespace {
            return Err(BootstrapRootError::ControlPlaneNamespaceMismatch);
        }
        if self.coverage_context_policy.institution_ref != self.institution_id
            || self.coverage_context_policy.rulebook_ref != self.rulebook_ref
        {
            return Err(BootstrapRootError::InstitutionRulebookMismatch);
        }
        if self.coverage_policy.authority_ref != self.adoption_ref
            || self.coverage_context_policy.authority_ref != self.adoption_ref
        {
            return Err(BootstrapRootError::RootAdoptionAuthorityMismatch);
        }

        let coverage_digest = self
            .coverage_policy
            .identity_digest()
            .map_err(|_| BootstrapRootError::InvalidCoveragePolicy)?;
        if self.coverage_context_policy.coverage_policy.digest != coverage_digest
            || self.coverage_context_policy.coverage_policy.profile != POLICY_IDENTITY_PROFILE
        {
            return Err(BootstrapRootError::CoveragePolicyBindingMismatch);
        }

        validate_control_plane_subjects(
            &self.coverage_policy.allowed_subject_kinds,
            &self.coverage_policy.mode,
            self.coverage_context_policy.witness_trust_policy.is_some(),
        )?;

        match (
            &self.coverage_policy.mode,
            &self.coverage_context_policy.witness_trust_policy,
            &self.coverage_context_policy.witness_trust_verifier_ref,
        ) {
            (CoverageMode::DirectSource, None, None) => {}
            (CoverageMode::WitnessQuorum { .. }, Some(_), Some(_)) => {}
            _ => return Err(BootstrapRootError::WitnessRootMismatch),
        }

        if self.coverage_policy.valid_from_ms > self.effective_from_ms
            || self.coverage_policy.valid_until_ms <= self.effective_from_ms
            || self.coverage_context_policy.valid_from_ms > self.effective_from_ms
            || self.coverage_context_policy.valid_until_ms <= self.effective_from_ms
        {
            return Err(BootstrapRootError::RootPolicyEpochMismatch);
        }
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
            .map_err(|_| BootstrapRootError::InvalidCoverageContextPolicy)?;

        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_ROOT);
        frame(&mut hasher, ROOT_IDENTITY_PROFILE.as_bytes());
        frame(&mut hasher, self.protocol_version.as_bytes());
        frame(&mut hasher, self.network_id.as_bytes());
        frame(&mut hasher, self.institution_id.as_bytes());
        frame(&mut hasher, self.constitution_id.as_bytes());
        frame(&mut hasher, &self.constitution_version.to_le_bytes());
        frame(&mut hasher, self.constitution_statement_profile.as_bytes());
        frame(&mut hasher, &self.constitution_statement_digest.0);
        frame(&mut hasher, self.rulebook_id.as_bytes());
        frame(&mut hasher, self.rulebook_version.as_bytes());
        frame(&mut hasher, self.rulebook_profile.as_bytes());
        frame(&mut hasher, &self.rulebook_digest.0);
        frame(&mut hasher, self.rulebook_ref.as_bytes());
        frame(&mut hasher, self.control_plane_namespace.as_bytes());
        frame(&mut hasher, POLICY_IDENTITY_PROFILE.as_bytes());
        frame(&mut hasher, &coverage_digest.0);
        frame(&mut hasher, CONTEXT_POLICY_PROFILE.as_bytes());
        frame(&mut hasher, &context_digest.0);
        frame(&mut hasher, &self.effective_from_ms.to_le_bytes());
        frame(&mut hasher, self.adoption_ref.as_bytes());
        frame(&mut hasher, self.adoption_proof_ref.as_bytes());
        Ok(Digest32(*hasher.finalize().as_bytes()))
    }
}

/// Independently verified current constitutional state from the DNA-bound
/// constitution/transition plane.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedCurrentConstitutionReceipt {
    pub statement: ConstitutionStatement,
    pub statement_digest: ConstitutionDigest32,
    pub dna_hash: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
    pub legacy_constitution_authoritative: bool,
}

/// Host-attested proof that the exact current rulebook adopted the exact root
/// manifest. This is an adapter boundary; existence of this struct is not proof.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedBootstrapRootAdoption {
    pub root_manifest_digest: Digest32,
    pub root_manifest_profile: String,
    pub constitution_statement_digest: Digest32,
    pub constitution_statement_profile: String,
    pub rulebook_digest: Digest32,
    pub rulebook_profile: String,
    pub adoption_ref: String,
    pub adoption_proof_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

/// Non-deserializable current root. A positive value can only be constructed by
/// exact constitutional + adoption qualification.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedAuthorityStateBootstrapRoot {
    manifest: AuthorityStateBootstrapRootManifest,
    root_manifest_digest: Digest32,
    constitution_statement_digest: Digest32,
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
    pub fn root_manifest_digest(&self) -> Digest32 {
        self.root_manifest_digest
    }
    pub fn root_manifest_profile(&self) -> &str {
        ROOT_IDENTITY_PROFILE
    }
    pub fn constitution_statement_digest(&self) -> Digest32 {
        self.constitution_statement_digest
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

pub fn qualify_bootstrap_root(
    manifest: &AuthorityStateBootstrapRootManifest,
    constitution: &VerifiedCurrentConstitutionReceipt,
    adoption: &VerifiedBootstrapRootAdoption,
    now_ms: u64,
) -> Result<QualifiedAuthorityStateBootstrapRoot, BootstrapRootError> {
    if now_ms == 0 {
        return Err(BootstrapRootError::InvalidVerificationTime);
    }
    manifest.validate()?;
    verify_current_constitution(constitution, now_ms)?;

    let current_statement_digest = constitution
        .statement
        .digest()
        .map_err(|_| BootstrapRootError::InvalidCurrentConstitution)?;
    if current_statement_digest != constitution.statement_digest {
        return Err(BootstrapRootError::ConstitutionDigestMismatch);
    }
    let current_statement_core = core_digest(current_statement_digest);
    let current_rulebook_core = core_digest(constitution.statement.rulebook.digest);

    if manifest.network_id != constitution.statement.network_id.as_str()
        || manifest.institution_id != constitution.statement.institution_id.as_str()
        || manifest.constitution_id != constitution.statement.constitution_id.as_str()
        || manifest.constitution_version != constitution.statement.version
        || manifest.constitution_statement_digest != current_statement_core
        || manifest.rulebook_id != constitution.statement.rulebook_id.as_str()
        || manifest.rulebook_version != constitution.statement.rulebook_version
        || manifest.rulebook_digest != current_rulebook_core
        || manifest.rulebook_profile != constitution.statement.rulebook.profile
        || manifest.effective_from_ms < constitution.statement.effective_from_ms
    {
        return Err(BootstrapRootError::ManifestConstitutionMismatch);
    }
    if manifest.effective_from_ms > now_ms {
        return Err(BootstrapRootError::RootNotYetEffective);
    }
    if !manifest.coverage_policy.is_active_at(now_ms)
        || !manifest.coverage_context_policy.active_at(now_ms)
    {
        return Err(BootstrapRootError::RootPolicyInactive);
    }

    let root_digest = manifest.identity_digest()?;
    verify_adoption(
        adoption,
        manifest,
        root_digest,
        current_statement_core,
        current_rulebook_core,
        now_ms,
    )?;

    let coverage_digest = manifest
        .coverage_policy
        .identity_digest()
        .map_err(|_| BootstrapRootError::InvalidCoveragePolicy)?;
    let context_digest = manifest
        .coverage_context_policy
        .identity_digest()
        .map_err(|_| BootstrapRootError::InvalidCoverageContextPolicy)?;

    let verified_at_ms = constitution
        .verified_at_ms
        .max(adoption.verified_at_ms);
    let valid_until_ms = constitution
        .valid_until_ms
        .min(adoption.valid_until_ms)
        .min(manifest.coverage_policy.valid_until_ms)
        .min(manifest.coverage_context_policy.valid_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(BootstrapRootError::RootNotCurrentlyUsable);
    }

    let qualification_digest = qualification_digest(
        root_digest,
        current_statement_core,
        coverage_digest,
        context_digest,
    );
    let verification_ref = format!(
        "authority-bootstrap-root:{}:{}",
        ROOT_QUALIFICATION_PROFILE,
        hex_digest(qualification_digest)
    );

    Ok(QualifiedAuthorityStateBootstrapRoot {
        manifest: manifest.clone(),
        root_manifest_digest: root_digest,
        constitution_statement_digest: current_statement_core,
        coverage_policy_digest: coverage_digest,
        context_policy_digest: context_digest,
        qualification_digest,
        qualification_profile: ROOT_QUALIFICATION_PROFILE.into(),
        verification_ref,
        verified_at_ms,
        valid_until_ms,
    })
}

fn verify_current_constitution(
    receipt: &VerifiedCurrentConstitutionReceipt,
    now_ms: u64,
) -> Result<(), BootstrapRootError> {
    receipt
        .statement
        .validate()
        .map_err(|_| BootstrapRootError::InvalidCurrentConstitution)?;
    require_ref(&receipt.dna_hash)?;
    require_ref(&receipt.verification_ref)?;
    if receipt.legacy_constitution_authoritative {
        return Err(BootstrapRootError::LegacyConstitutionRejected);
    }
    if receipt.verified_at_ms == 0
        || receipt.verified_at_ms > now_ms
        || receipt.verified_at_ms < receipt.statement.effective_from_ms
        || receipt.valid_until_ms <= now_ms
    {
        return Err(BootstrapRootError::InvalidConstitutionVerificationWindow);
    }
    Ok(())
}

fn verify_adoption(
    receipt: &VerifiedBootstrapRootAdoption,
    manifest: &AuthorityStateBootstrapRootManifest,
    root_digest: Digest32,
    constitution_digest: Digest32,
    rulebook_digest: Digest32,
    now_ms: u64,
) -> Result<(), BootstrapRootError> {
    for value in [
        receipt.adoption_ref.as_str(),
        receipt.adoption_proof_ref.as_str(),
        receipt.verification_ref.as_str(),
    ] {
        require_ref(value)?;
    }
    require_profile(&receipt.root_manifest_profile)?;
    require_profile(&receipt.constitution_statement_profile)?;
    require_profile(&receipt.rulebook_profile)?;
    if receipt.root_manifest_digest != root_digest
        || receipt.root_manifest_profile != ROOT_IDENTITY_PROFILE
        || receipt.constitution_statement_digest != constitution_digest
        || receipt.constitution_statement_profile != STATEMENT_PROFILE
        || receipt.rulebook_digest != rulebook_digest
        || receipt.rulebook_profile != manifest.rulebook_profile
        || receipt.adoption_ref != manifest.adoption_ref
        || receipt.adoption_proof_ref != manifest.adoption_proof_ref
    {
        return Err(BootstrapRootError::RootAdoptionMismatch);
    }
    if receipt.verified_at_ms == 0
        || receipt.verified_at_ms > now_ms
        || receipt.valid_until_ms <= now_ms
        || receipt.verified_at_ms < manifest.effective_from_ms
    {
        return Err(BootstrapRootError::InvalidAdoptionVerificationWindow);
    }
    Ok(())
}

fn validate_control_plane_subjects(
    kinds: &[AuthoritySubjectKind],
    mode: &CoverageMode,
    has_witness_policy: bool,
) -> Result<(), BootstrapRootError> {
    let mut actual = BTreeSet::new();
    for kind in kinds {
        let code = control_kind_code(kind)?;
        if !actual.insert(code) {
            return Err(BootstrapRootError::DuplicateControlPlaneSubjectKind);
        }
    }

    let mut expected = BTreeSet::from([7u8, 8u8]);
    if matches!(mode, CoverageMode::WitnessQuorum { .. }) || has_witness_policy {
        expected.insert(9);
    }
    if actual != expected {
        return Err(BootstrapRootError::InvalidControlPlaneSubjectSet);
    }
    Ok(())
}

fn control_kind_code(kind: &AuthoritySubjectKind) -> Result<u8, BootstrapRootError> {
    match kind {
        AuthoritySubjectKind::AuthorityCoveragePolicy => Ok(7),
        AuthoritySubjectKind::CoverageTrustContextPolicy => Ok(8),
        AuthoritySubjectKind::WitnessTrustPolicy => Ok(9),
        _ => Err(BootstrapRootError::OperationalSubjectInBootstrapRoot),
    }
}

fn qualification_digest(
    root_digest: Digest32,
    constitution_digest: Digest32,
    coverage_digest: Digest32,
    context_digest: Digest32,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_QUALIFIED);
    frame(&mut hasher, ROOT_QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, &root_digest.0);
    frame(&mut hasher, &constitution_digest.0);
    frame(&mut hasher, &coverage_digest.0);
    frame(&mut hasher, &context_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

fn core_digest(value: ConstitutionDigest32) -> Digest32 {
    Digest32(value.0)
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

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn hex_digest(digest: Digest32) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(64);
    for byte in digest.0 {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BootstrapRootError {
    WrongProtocolVersion,
    InvalidReference,
    InvalidProfile,
    ZeroDigest,
    InvalidRootEpoch,
    WrongConstitutionProfile,
    InvalidCoveragePolicy,
    InvalidCoverageContextPolicy,
    ControlPlaneNamespaceMismatch,
    InstitutionRulebookMismatch,
    RootAdoptionAuthorityMismatch,
    CoveragePolicyBindingMismatch,
    DuplicateControlPlaneSubjectKind,
    InvalidControlPlaneSubjectSet,
    OperationalSubjectInBootstrapRoot,
    WitnessRootMismatch,
    RootPolicyEpochMismatch,
    InvalidVerificationTime,
    InvalidCurrentConstitution,
    ConstitutionDigestMismatch,
    LegacyConstitutionRejected,
    InvalidConstitutionVerificationWindow,
    ManifestConstitutionMismatch,
    RootNotYetEffective,
    RootPolicyInactive,
    RootAdoptionMismatch,
    InvalidAdoptionVerificationWindow,
    RootNotCurrentlyUsable,
}

impl fmt::Display for BootstrapRootError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::WrongProtocolVersion => "wrong authority-state bootstrap-root protocol version",
            Self::InvalidReference => "invalid authority-state bootstrap-root reference",
            Self::InvalidProfile => "invalid authority-state bootstrap-root profile",
            Self::ZeroDigest => "authority-state bootstrap-root digest must not be zero",
            Self::InvalidRootEpoch => "invalid authority-state bootstrap-root epoch",
            Self::WrongConstitutionProfile => "wrong constitutional statement profile",
            Self::InvalidCoveragePolicy => "invalid bootstrap coverage policy",
            Self::InvalidCoverageContextPolicy => "invalid bootstrap coverage-context policy",
            Self::ControlPlaneNamespaceMismatch => "bootstrap coverage namespace mismatch",
            Self::InstitutionRulebookMismatch => "bootstrap context institution/rulebook mismatch",
            Self::RootAdoptionAuthorityMismatch => "bootstrap policies do not share root adoption authority",
            Self::CoveragePolicyBindingMismatch => "bootstrap context binds another coverage policy",
            Self::DuplicateControlPlaneSubjectKind => "duplicate bootstrap control-plane subject kind",
            Self::InvalidControlPlaneSubjectSet => "bootstrap root has wrong control-plane subject set",
            Self::OperationalSubjectInBootstrapRoot => "operational subject kind is forbidden in bootstrap root",
            Self::WitnessRootMismatch => "bootstrap witness mode and trust context disagree",
            Self::RootPolicyEpochMismatch => "bootstrap policy is not valid at root effective epoch",
            Self::InvalidVerificationTime => "invalid bootstrap-root verification time",
            Self::InvalidCurrentConstitution => "invalid current constitutional receipt",
            Self::ConstitutionDigestMismatch => "current constitutional statement digest mismatch",
            Self::LegacyConstitutionRejected => "legacy constitution cannot anchor bootstrap authority",
            Self::InvalidConstitutionVerificationWindow => "current constitution verification is stale or future",
            Self::ManifestConstitutionMismatch => "bootstrap manifest does not match current constitution/rulebook",
            Self::RootNotYetEffective => "bootstrap root is not yet effective",
            Self::RootPolicyInactive => "bootstrap root policy/context is not currently active",
            Self::RootAdoptionMismatch => "bootstrap root adoption evidence mismatch",
            Self::InvalidAdoptionVerificationWindow => "bootstrap root adoption verification is stale or future",
            Self::RootNotCurrentlyUsable => "bootstrap root is not currently usable",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for BootstrapRootError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::ProfiledDigest as FreshnessProfiledDigest;
    use mycelix_governance_constitution::{
        ConstitutionId, InstitutionId, NetworkId, ProfiledDigest as ConstitutionProfiledDigest,
        RulebookId, PROTOCOL_VERSION as CONSTITUTION_PROTOCOL,
    };

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn cd(byte: u8) -> ConstitutionDigest32 {
        ConstitutionDigest32([byte; 32])
    }

    fn statement(version: u64, rulebook_byte: u8) -> ConstitutionStatement {
        ConstitutionStatement {
            protocol_version: CONSTITUTION_PROTOCOL.into(),
            network_id: NetworkId::new("network:test").unwrap(),
            institution_id: InstitutionId::new("institution:test").unwrap(),
            constitution_id: ConstitutionId::new("constitution:test").unwrap(),
            version,
            parent_statement_digest: if version == 1 { None } else { Some(cd(99)) },
            rulebook_id: RulebookId::new("rulebook:test").unwrap(),
            rulebook_version: format!("v{version}"),
            rulebook: ConstitutionProfiledDigest {
                digest: cd(rulebook_byte),
                profile: "rulebook-v1-blake3".into(),
            },
            charter: ConstitutionProfiledDigest {
                digest: cd(3),
                profile: "charter-v1-blake3".into(),
            },
            parameters: ConstitutionProfiledDigest {
                digest: cd(4),
                profile: "parameters-v1-blake3".into(),
            },
            amendment_policy: ConstitutionProfiledDigest {
                digest: cd(5),
                profile: "amendment-v1-blake3".into(),
            },
            binding_vote_profile: "binding-vote-v1".into(),
            threshold_authority_profile: "threshold-authority-v1".into(),
            effective_from_ms: 100,
        }
    }

    fn manifest_for(statement: &ConstitutionStatement, witness: bool) -> AuthorityStateBootstrapRootManifest {
        let statement_digest = core_digest(statement.digest().unwrap());
        let rulebook_digest = core_digest(statement.rulebook.digest);
        let mode = if witness {
            CoverageMode::WitnessQuorum {
                min_witnesses: 2,
                min_trust_domains: 2,
                max_per_trust_domain: 1,
            }
        } else {
            CoverageMode::DirectSource
        };
        let mut kinds = vec![
            AuthoritySubjectKind::AuthorityCoveragePolicy,
            AuthoritySubjectKind::CoverageTrustContextPolicy,
        ];
        if witness {
            kinds.push(AuthoritySubjectKind::WitnessTrustPolicy);
        }
        let coverage = AuthorityCoveragePolicy {
            protocol_version: mycelix_authority_state_coverage::PROTOCOL_VERSION.into(),
            policy_id: "bootstrap-coverage:test".into(),
            namespace: "authority-control:test".into(),
            allowed_subject_kinds: kinds,
            authoritative_source_ref: "authority-root-source:test".into(),
            source_identity: FreshnessProfiledDigest {
                digest: d(10),
                profile: "authority-root-source-v1-blake3".into(),
            },
            mode,
            max_source_age_ms: 1_000,
            max_witness_age_ms: 1_000,
            max_coverage_lease_ms: 500,
            valid_from_ms: 100,
            valid_until_ms: 10_000,
            authority_ref: "root-adoption:test".into(),
            policy_proof_ref: "coverage-proof:test".into(),
        };
        let coverage_digest = coverage.identity_digest().unwrap();
        let context = CoverageTrustContextPolicy {
            protocol_version: mycelix_authority_state_coverage_context::PROTOCOL_VERSION.into(),
            context_policy_id: "bootstrap-context:test".into(),
            institution_ref: "institution:test".into(),
            jurisdiction_ref: None,
            rulebook_ref: "rulebook:test@current".into(),
            coverage_policy: FreshnessProfiledDigest {
                digest: coverage_digest,
                profile: POLICY_IDENTITY_PROFILE.into(),
            },
            witness_trust_policy: witness.then(|| FreshnessProfiledDigest {
                digest: d(11),
                profile: "witness-trust-root-v1-blake3".into(),
            }),
            witness_trust_verifier_ref: witness.then(|| "root-trust-verifier:test".into()),
            max_challenge_lifetime_ms: 500,
            valid_from_ms: 100,
            valid_until_ms: 10_000,
            authority_ref: "root-adoption:test".into(),
            policy_proof_ref: "context-proof:test".into(),
        };
        AuthorityStateBootstrapRootManifest {
            protocol_version: PROTOCOL_VERSION.into(),
            network_id: statement.network_id.as_str().into(),
            institution_id: statement.institution_id.as_str().into(),
            constitution_id: statement.constitution_id.as_str().into(),
            constitution_version: statement.version,
            constitution_statement_digest: statement_digest,
            constitution_statement_profile: STATEMENT_PROFILE.into(),
            rulebook_id: statement.rulebook_id.as_str().into(),
            rulebook_version: statement.rulebook_version.clone(),
            rulebook_digest,
            rulebook_profile: statement.rulebook.profile.clone(),
            rulebook_ref: "rulebook:test@current".into(),
            control_plane_namespace: "authority-control:test".into(),
            coverage_policy: coverage,
            coverage_context_policy: context,
            effective_from_ms: 100,
            adoption_ref: "root-adoption:test".into(),
            adoption_proof_ref: "root-adoption-proof:test".into(),
        }
    }

    fn constitution_receipt(statement: ConstitutionStatement) -> VerifiedCurrentConstitutionReceipt {
        let digest = statement.digest().unwrap();
        VerifiedCurrentConstitutionReceipt {
            statement,
            statement_digest: digest,
            dna_hash: "dna:test".into(),
            verification_ref: "constitution-verification:test".into(),
            verified_at_ms: 900,
            valid_until_ms: 1_500,
            legacy_constitution_authoritative: false,
        }
    }

    fn adoption(manifest: &AuthorityStateBootstrapRootManifest) -> VerifiedBootstrapRootAdoption {
        VerifiedBootstrapRootAdoption {
            root_manifest_digest: manifest.identity_digest().unwrap(),
            root_manifest_profile: ROOT_IDENTITY_PROFILE.into(),
            constitution_statement_digest: manifest.constitution_statement_digest,
            constitution_statement_profile: STATEMENT_PROFILE.into(),
            rulebook_digest: manifest.rulebook_digest,
            rulebook_profile: manifest.rulebook_profile.clone(),
            adoption_ref: manifest.adoption_ref.clone(),
            adoption_proof_ref: manifest.adoption_proof_ref.clone(),
            verification_ref: "root-adoption-verification:test".into(),
            verified_at_ms: 910,
            valid_until_ms: 1_400,
        }
    }

    #[test]
    fn exact_current_constitution_qualifies_bootstrap_root() {
        let statement = statement(1, 2);
        let manifest = manifest_for(&statement, false);
        let qualified = qualify_bootstrap_root(
            &manifest,
            &constitution_receipt(statement),
            &adoption(&manifest),
            1_000,
        )
        .unwrap();
        assert_eq!(qualified.root_manifest_digest(), manifest.identity_digest().unwrap());
        assert_eq!(qualified.valid_until_ms(), 1_400);
    }

    #[test]
    fn constitutional_advance_invalidates_old_root_even_with_same_source() {
        let old_statement = statement(1, 2);
        let manifest = manifest_for(&old_statement, false);
        let new_statement = statement(2, 22);
        assert_eq!(
            qualify_bootstrap_root(
                &manifest,
                &constitution_receipt(new_statement),
                &adoption(&manifest),
                1_000,
            )
            .unwrap_err(),
            BootstrapRootError::ManifestConstitutionMismatch
        );
    }

    #[test]
    fn operational_subjects_cannot_enter_bootstrap_root() {
        let statement = statement(1, 2);
        let mut manifest = manifest_for(&statement, false);
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
    fn witness_root_requires_witness_trust_subject_and_context() {
        let statement = statement(1, 2);
        let mut manifest = manifest_for(&statement, true);
        manifest.coverage_context_policy.witness_trust_policy = None;
        manifest.coverage_context_policy.witness_trust_verifier_ref = None;
        assert!(manifest.validate().is_err());
    }

    #[test]
    fn root_qualification_has_no_authority_freshness_dependency() {
        let statement = statement(1, 2);
        let manifest = manifest_for(&statement, false);
        let root = qualify_bootstrap_root(
            &manifest,
            &constitution_receipt(statement),
            &adoption(&manifest),
            1_000,
        )
        .unwrap();
        assert!(!root.qualification_digest().is_zero());
    }
}
