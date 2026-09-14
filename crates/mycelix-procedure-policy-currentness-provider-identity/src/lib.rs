// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Canonical semantic identity for ADMIN-002 procedure-policy currentness-provider
//! selection policies.
//!
//! The qualified #804 currentness kernel deliberately accepts a caller-supplied
//! `ProcedurePolicyCurrentnessPolicy` describing which provider namespace and
//! institutional authority may attest policy currentness. This crate gives that
//! verifier-selection policy one stable semantic identity so later adoption,
//! revocation/currentness and audit layers can bind the exact trust root instead
//! of treating configuration provenance as implicit authority.
//!
//! This crate proves only semantic content -> canonical identity. It does not
//! prove institutional adoption, currentness, verifier provenance, administrative
//! decision authority, or external-effect authority.

use mycelix_institutional_core::{Digest32, EvidenceRequirement, RulebookRef};
use mycelix_procedure_policy_currentness::ProcedurePolicyCurrentnessPolicy;
use std::collections::BTreeSet;
use std::fmt;

pub const CURRENTNESS_PROVIDER_POLICY_IDENTITY_PROFILE: &str =
    "mycelix-procedure-policy-currentness-provider-v1-blake3-framed-semantic";
const DOMAIN_CURRENTNESS_PROVIDER_POLICY: &[u8] =
    b"mycelix/administrative-procedure/currentness-provider-policy/v1";
const MAX_ID_BYTES: usize = 512;
const MAX_EVIDENCE_TYPE_BYTES: usize = 256;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CanonicalCurrentnessProviderPolicyIdentity {
    pub digest: Digest32,
    pub profile: String,
}

impl CanonicalCurrentnessProviderPolicyIdentity {
    pub fn grants_authority(&self) -> bool {
        false
    }

    pub fn grants_policy_currentness(&self) -> bool {
        false
    }

    pub fn grants_administrative_decision_authority(&self) -> bool {
        false
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

/// Process-local proof that one supplied provider-selection policy has a stable
/// canonical semantic identity.
///
/// Deliberately not Clone/Serialize/Deserialize. Persisted policy configuration
/// must be requalified from exact semantic content rather than loading a positive
/// identity token.
#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedCurrentnessProviderPolicyIdentity {
    identity: CanonicalCurrentnessProviderPolicyIdentity,
    policy: ProcedurePolicyCurrentnessPolicy,
}

impl QualifiedCurrentnessProviderPolicyIdentity {
    pub fn identity(&self) -> &CanonicalCurrentnessProviderPolicyIdentity {
        &self.identity
    }

    pub fn policy(&self) -> &ProcedurePolicyCurrentnessPolicy {
        &self.policy
    }

    pub fn grants_authority(&self) -> bool {
        false
    }

    pub fn grants_policy_currentness(&self) -> bool {
        false
    }

    pub fn grants_administrative_decision_authority(&self) -> bool {
        false
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

pub fn currentness_provider_policy_semantic_identity(
    policy: &ProcedurePolicyCurrentnessPolicy,
) -> Result<CanonicalCurrentnessProviderPolicyIdentity, CurrentnessProviderPolicyIdentityError> {
    policy
        .validate()
        .map_err(|error| CurrentnessProviderPolicyIdentityError::InvalidCurrentnessPolicy(
            error.to_string(),
        ))?;
    validate_semantic_ids(policy)?;

    let roles = canonical_roles(policy)?;
    let evidence = canonical_evidence_requirements(&policy.provider_authority_evidence)?;

    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_CURRENTNESS_PROVIDER_POLICY);
    frame(
        &mut hasher,
        CURRENTNESS_PROVIDER_POLICY_IDENTITY_PROFILE.as_bytes(),
    );
    frame(&mut hasher, policy.protocol_version.as_bytes());

    frame(&mut hasher, policy.target_institution.as_str().as_bytes());
    frame_optional_id(
        &mut hasher,
        policy
            .target_jurisdiction
            .as_ref()
            .map(|value| value.as_str()),
    );
    frame_rulebook(&mut hasher, &policy.target_rulebook);
    frame(&mut hasher, policy.procedure_profile.as_str().as_bytes());

    frame(&mut hasher, policy.provider_namespace.as_bytes());
    frame(
        &mut hasher,
        policy.provider_authority_institution.as_str().as_bytes(),
    );
    frame_optional_id(
        &mut hasher,
        policy
            .provider_authority_jurisdiction
            .as_ref()
            .map(|value| value.as_str()),
    );
    frame_rulebook(&mut hasher, &policy.provider_authority_rulebook);
    frame(
        &mut hasher,
        policy.required_provider_capability.as_str().as_bytes(),
    );

    frame_count(&mut hasher, roles.len());
    for role in roles {
        frame(&mut hasher, role.as_bytes());
    }

    frame_count(&mut hasher, evidence.len());
    for requirement in evidence {
        frame(&mut hasher, &requirement);
    }

    Ok(CanonicalCurrentnessProviderPolicyIdentity {
        digest: Digest32(*hasher.finalize().as_bytes()),
        profile: CURRENTNESS_PROVIDER_POLICY_IDENTITY_PROFILE.into(),
    })
}

pub fn qualify_currentness_provider_policy_identity(
    policy: ProcedurePolicyCurrentnessPolicy,
) -> Result<QualifiedCurrentnessProviderPolicyIdentity, CurrentnessProviderPolicyIdentityError> {
    let identity = currentness_provider_policy_semantic_identity(&policy)?;
    Ok(QualifiedCurrentnessProviderPolicyIdentity { identity, policy })
}

fn validate_semantic_ids(
    policy: &ProcedurePolicyCurrentnessPolicy,
) -> Result<(), CurrentnessProviderPolicyIdentityError> {
    validate_id(policy.target_institution.as_str())?;
    if let Some(jurisdiction) = &policy.target_jurisdiction {
        validate_id(jurisdiction.as_str())?;
    }
    validate_id(policy.target_rulebook.id.as_str())?;
    validate_id(policy.procedure_profile.as_str())?;
    validate_id(policy.provider_authority_institution.as_str())?;
    if let Some(jurisdiction) = &policy.provider_authority_jurisdiction {
        validate_id(jurisdiction.as_str())?;
    }
    validate_id(policy.provider_authority_rulebook.id.as_str())?;
    validate_id(policy.required_provider_capability.as_str())?;
    for role in &policy.accepted_provider_roles {
        validate_id(role.as_str())?;
    }
    for requirement in &policy.provider_authority_evidence {
        validate_evidence_type(&requirement.evidence_type)?;
        for issuer in &requirement.accepted_issuers {
            validate_id(issuer.as_str())?;
        }
    }
    Ok(())
}

fn canonical_roles(
    policy: &ProcedurePolicyCurrentnessPolicy,
) -> Result<Vec<&str>, CurrentnessProviderPolicyIdentityError> {
    let mut roles = BTreeSet::new();
    for role in &policy.accepted_provider_roles {
        if !roles.insert(role.as_str()) {
            return Err(CurrentnessProviderPolicyIdentityError::DuplicateProviderRole);
        }
    }
    Ok(roles.into_iter().collect())
}

fn canonical_evidence_requirements(
    requirements: &[EvidenceRequirement],
) -> Result<Vec<Vec<u8>>, CurrentnessProviderPolicyIdentityError> {
    let mut encoded = Vec::with_capacity(requirements.len());
    for requirement in requirements {
        validate_evidence_type(&requirement.evidence_type)?;
        let mut issuers = BTreeSet::new();
        for issuer in &requirement.accepted_issuers {
            validate_id(issuer.as_str())?;
            if !issuers.insert(issuer.as_str()) {
                return Err(CurrentnessProviderPolicyIdentityError::DuplicateAcceptedIssuer);
            }
        }

        let mut bytes = Vec::new();
        encode_frame(&mut bytes, requirement.evidence_type.as_bytes());
        encode_count(&mut bytes, issuers.len());
        for issuer in issuers {
            encode_frame(&mut bytes, issuer.as_bytes());
        }
        encoded.push(bytes);
    }

    encoded.sort();
    if encoded.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(CurrentnessProviderPolicyIdentityError::DuplicateEvidenceRequirement);
    }
    Ok(encoded)
}

fn frame_rulebook(hasher: &mut blake3::Hasher, rulebook: &RulebookRef) {
    frame(hasher, rulebook.id.as_str().as_bytes());
    frame(hasher, rulebook.version.as_bytes());
    frame(hasher, &rulebook.digest.0);
}

fn frame_optional_id(hasher: &mut blake3::Hasher, value: Option<&str>) {
    match value {
        None => frame(hasher, &[0]),
        Some(value) => {
            frame(hasher, &[1]);
            frame(hasher, value.as_bytes());
        }
    }
}

fn frame_count(hasher: &mut blake3::Hasher, count: usize) {
    frame(hasher, &(count as u64).to_le_bytes());
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn encode_count(output: &mut Vec<u8>, count: usize) {
    encode_frame(output, &(count as u64).to_le_bytes());
}

fn encode_frame(output: &mut Vec<u8>, bytes: &[u8]) {
    output.extend_from_slice(&(bytes.len() as u64).to_le_bytes());
    output.extend_from_slice(bytes);
}

fn validate_id(value: &str) -> Result<(), CurrentnessProviderPolicyIdentityError> {
    if value.trim().is_empty() || value.len() > MAX_ID_BYTES {
        Err(CurrentnessProviderPolicyIdentityError::InvalidIdentifier)
    } else {
        Ok(())
    }
}

fn validate_evidence_type(
    value: &str,
) -> Result<(), CurrentnessProviderPolicyIdentityError> {
    if value.trim().is_empty() || value.len() > MAX_EVIDENCE_TYPE_BYTES {
        Err(CurrentnessProviderPolicyIdentityError::InvalidEvidenceType)
    } else {
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CurrentnessProviderPolicyIdentityError {
    InvalidCurrentnessPolicy(String),
    InvalidIdentifier,
    InvalidEvidenceType,
    DuplicateProviderRole,
    DuplicateAcceptedIssuer,
    DuplicateEvidenceRequirement,
}

impl fmt::Display for CurrentnessProviderPolicyIdentityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidCurrentnessPolicy(error) => {
                write!(f, "invalid procedure-policy currentness-provider policy: {error}")
            }
            Self::InvalidIdentifier => write!(f, "invalid currentness-provider policy identifier"),
            Self::InvalidEvidenceType => {
                write!(f, "invalid currentness-provider authority evidence type")
            }
            Self::DuplicateProviderRole => {
                write!(f, "duplicate accepted currentness-provider role")
            }
            Self::DuplicateAcceptedIssuer => {
                write!(f, "duplicate accepted currentness-provider evidence issuer")
            }
            Self::DuplicateEvidenceRequirement => {
                write!(f, "duplicate currentness-provider evidence requirement")
            }
        }
    }
}

impl std::error::Error for CurrentnessProviderPolicyIdentityError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_administrative_procedure::ProcedureProfileId;
    use mycelix_institutional_core::{
        CapabilityId, EvidenceRequirement, InstitutionId, JurisdictionId, PrincipalId, RoleId,
        RulebookId,
    };
    use mycelix_procedure_policy_currentness::PROTOCOL_VERSION;

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn rulebook(id: &str, byte: u8) -> RulebookRef {
        RulebookRef {
            id: RulebookId::new(id).unwrap(),
            version: "1.0.0".into(),
            digest: d(byte),
        }
    }

    fn p(value: &str) -> PrincipalId {
        PrincipalId::new(value).unwrap()
    }

    fn policy() -> ProcedurePolicyCurrentnessPolicy {
        ProcedurePolicyCurrentnessPolicy {
            protocol_version: PROTOCOL_VERSION.into(),
            target_institution: InstitutionId::new("institution:city").unwrap(),
            target_jurisdiction: Some(JurisdictionId::new("jurisdiction:city").unwrap()),
            target_rulebook: rulebook("rulebook:administration:v1", 1),
            procedure_profile: ProcedureProfileId::new("procedure:permit:v1").unwrap(),
            provider_namespace: "registry:procedure-policy:city".into(),
            provider_authority_institution: InstitutionId::new("institution:records-office").unwrap(),
            provider_authority_jurisdiction: Some(
                JurisdictionId::new("jurisdiction:city").unwrap(),
            ),
            provider_authority_rulebook: rulebook("rulebook:records-office:v1", 2),
            required_provider_capability: CapabilityId::new(
                "administration.policy.currentness.attest",
            )
            .unwrap(),
            accepted_provider_roles: vec![
                RoleId::new("role:policy-registrar").unwrap(),
                RoleId::new("role:records-officer").unwrap(),
            ],
            provider_authority_evidence: vec![
                EvidenceRequirement {
                    evidence_type: "credential:registry-operator".into(),
                    accepted_issuers: vec![p("did:example:city"), p("did:example:auditor")],
                },
                EvidenceRequirement {
                    evidence_type: "proof:training-current".into(),
                    accepted_issuers: vec![],
                },
            ],
        }
    }

    fn identity(policy: &ProcedurePolicyCurrentnessPolicy) -> Digest32 {
        currentness_provider_policy_semantic_identity(policy)
            .unwrap()
            .digest
    }

    #[test]
    fn exact_provider_policy_has_identity_without_authority_amplification() {
        let qualified = qualify_currentness_provider_policy_identity(policy()).unwrap();
        assert_eq!(qualified.identity().profile, CURRENTNESS_PROVIDER_POLICY_IDENTITY_PROFILE);
        assert!(!qualified.grants_authority());
        assert!(!qualified.grants_policy_currentness());
        assert!(!qualified.grants_administrative_decision_authority());
        assert!(!qualified.grants_external_effect_authority());
    }

    #[test]
    fn role_requirement_and_issuer_order_are_non_semantic() {
        let base = policy();
        let expected = identity(&base);

        let mut roles = base.clone();
        roles.accepted_provider_roles.reverse();
        assert_eq!(identity(&roles), expected);

        let mut requirements = base.clone();
        requirements.provider_authority_evidence.reverse();
        assert_eq!(identity(&requirements), expected);

        let mut issuers = base;
        issuers.provider_authority_evidence[0].accepted_issuers.reverse();
        assert_eq!(identity(&issuers), expected);
    }

    #[test]
    fn target_scope_and_rulebook_are_identity_bearing() {
        let base = policy();
        let expected = identity(&base);

        let mut changed = base.clone();
        changed.target_institution = InstitutionId::new("institution:county").unwrap();
        assert_ne!(identity(&changed), expected);

        let mut changed = base.clone();
        changed.target_jurisdiction = None;
        assert_ne!(identity(&changed), expected);

        let mut changed = base.clone();
        changed.target_rulebook.digest = d(99);
        assert_ne!(identity(&changed), expected);

        let mut changed = base;
        changed.procedure_profile = ProcedureProfileId::new("procedure:permit:v2").unwrap();
        assert_ne!(identity(&changed), expected);
    }

    #[test]
    fn provider_selection_semantics_are_identity_bearing() {
        let base = policy();
        let expected = identity(&base);

        let mut changed = base.clone();
        changed.provider_namespace = "registry:procedure-policy:county".into();
        assert_ne!(identity(&changed), expected);

        let mut changed = base.clone();
        changed.provider_authority_institution = InstitutionId::new("institution:clerk").unwrap();
        assert_ne!(identity(&changed), expected);

        let mut changed = base.clone();
        changed.provider_authority_rulebook.digest = d(77);
        assert_ne!(identity(&changed), expected);

        let mut changed = base;
        changed.required_provider_capability =
            CapabilityId::new("administration.policy.currentness.attest.v2").unwrap();
        assert_ne!(identity(&changed), expected);
    }

    #[test]
    fn provider_roles_and_evidence_requirements_are_identity_bearing() {
        let base = policy();
        let expected = identity(&base);

        let mut changed = base.clone();
        changed
            .accepted_provider_roles
            .push(RoleId::new("role:backup-registrar").unwrap());
        assert_ne!(identity(&changed), expected);

        let mut changed = base;
        changed.provider_authority_evidence.push(EvidenceRequirement {
            evidence_type: "proof:bonded-office".into(),
            accepted_issuers: vec![],
        });
        assert_ne!(identity(&changed), expected);
    }

    #[test]
    fn duplicate_accepted_issuer_is_not_an_alternate_encoding() {
        let mut duplicate = policy();
        let issuer = duplicate.provider_authority_evidence[0].accepted_issuers[0].clone();
        duplicate.provider_authority_evidence[0]
            .accepted_issuers
            .push(issuer);
        assert_eq!(
            currentness_provider_policy_semantic_identity(&duplicate).unwrap_err(),
            CurrentnessProviderPolicyIdentityError::DuplicateAcceptedIssuer
        );
    }

    #[test]
    fn duplicate_semantic_evidence_requirement_is_rejected() {
        let mut duplicate = policy();
        let requirement = duplicate.provider_authority_evidence[0].clone();
        duplicate.provider_authority_evidence.push(requirement);
        assert_eq!(
            currentness_provider_policy_semantic_identity(&duplicate).unwrap_err(),
            CurrentnessProviderPolicyIdentityError::DuplicateEvidenceRequirement
        );
    }

    #[test]
    fn malformed_public_id_wrapper_fails_identity_qualification() {
        let mut malformed = policy();
        malformed.required_provider_capability = CapabilityId("\n".into());
        assert_eq!(
            currentness_provider_policy_semantic_identity(&malformed).unwrap_err(),
            CurrentnessProviderPolicyIdentityError::InvalidIdentifier
        );
    }
}
