//! Institution-adopted integration operation -> capability semantics.
//!
//! This crate deliberately stops before generation-bound currentness. It proves
//! that one exact semantic policy was independently recorded and adopted, then
//! binds an exact typed integration command to the capability named by that
//! policy. A child tranche must add a dedicated shared freshness subject before
//! this mapping may be treated as current/revocable execution authority.

use mycelix_institutional_core::{
    CapabilityId, Digest32, InstitutionId, JurisdictionId, RulebookRef,
};
use mycelix_integration_core::{
    CanonicalEncodeV1, ContentCommitment, ExternalObjectType, ExternalOperationKind,
    ExternalSystemId, IntegrationCommand, SemanticProfileId, SideEffectClass,
};
use serde::{Deserialize, Serialize};
use thiserror::Error;

pub const PROTOCOL_VERSION: &str = "mycelix-integration-capability-policy-v0.1";
pub const POLICY_RECORD_PROOF_PROTOCOL: &str =
    "mycelix-integration-capability-policy-record-proof-v0.1";
pub const POLICY_ADOPTION_PROOF_PROTOCOL: &str =
    "mycelix-integration-capability-policy-adoption-proof-v0.1";
pub const POLICY_IDENTITY_PROFILE: &str =
    "mycelix-integration-capability-policy-v1-blake3-framed";
pub const QUALIFICATION_PROFILE: &str =
    "mycelix-integration-capability-policy-qualified-v1-blake3-framed";
pub const EVIDENCE_PROFILE: &str =
    "mycelix-integration-capability-policy-evidence-v1-blake3-framed";
pub const COMMAND_MAPPING_PROFILE: &str =
    "mycelix-integration-command-capability-mapping-v1-blake3-framed";

const DOMAIN_POLICY: &[u8] = b"mycelix/integration/capability-policy/v1";
const DOMAIN_QUALIFICATION: &[u8] = b"mycelix/integration/capability-policy/qualified/v1";
const DOMAIN_EVIDENCE: &[u8] = b"mycelix/integration/capability-policy/evidence/v1";
const DOMAIN_COMMAND_MAPPING: &[u8] = b"mycelix/integration/command-capability-mapping/v1";
const MAX_TEXT_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 256;

/// Institution-adopted semantic statement that one exact integration operation
/// requires one exact institutional capability.
///
/// Connector instance is intentionally absent: capability semantics are about
/// the operation domain, while provider deployment/currentness is qualified by
/// the separate INT-04 provider profile.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct IntegrationCapabilityPolicy {
    pub protocol_version: String,
    pub policy_id: String,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub system: ExternalSystemId,
    pub operation_kind: ExternalOperationKind,
    pub semantic_profile: SemanticProfileId,
    pub side_effect_class: SideEffectClass,
    /// Optional target narrowing. `None` means the operation policy is not
    /// target-object-type-specific.
    pub target_object_type: Option<ExternalObjectType>,
    pub required_capability: CapabilityId,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
    /// Exact institutional authority expected to adopt this policy.
    pub authority_ref: String,
    /// Exact immutable adoption proof identity expected by the policy.
    pub policy_proof_ref: String,
}

impl IntegrationCapabilityPolicy {
    pub fn validate(&self) -> Result<(), IntegrationCapabilityPolicyError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(IntegrationCapabilityPolicyError::WrongProtocolVersion);
        }
        require_text(&self.policy_id)?;
        require_text(self.institution.as_str())?;
        if let Some(jurisdiction) = &self.jurisdiction {
            require_text(jurisdiction.as_str())?;
        }
        self.rulebook
            .validate()
            .map_err(|_| IntegrationCapabilityPolicyError::InvalidRulebook)?;
        require_text(self.system.as_str())?;
        require_text(self.operation_kind.as_str())?;
        require_profile(self.semantic_profile.as_str())?;
        if let Some(target) = &self.target_object_type {
            require_text(target.as_str())?;
        }
        require_text(self.required_capability.as_str())?;
        require_text(&self.authority_ref)?;
        require_text(&self.policy_proof_ref)?;
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(IntegrationCapabilityPolicyError::InvalidPolicyWindow);
        }
        Ok(())
    }

    pub fn is_active_at(&self, now_ms: u64) -> bool {
        self.valid_from_ms <= now_ms && now_ms < self.valid_until_ms
    }

    pub fn identity_digest(&self) -> Result<Digest32, IntegrationCapabilityPolicyError> {
        self.validate()?;
        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_POLICY);
        frame(&mut h, POLICY_IDENTITY_PROFILE.as_bytes());
        frame(&mut h, self.protocol_version.as_bytes());
        frame(&mut h, self.policy_id.as_bytes());
        frame(&mut h, self.institution.as_str().as_bytes());
        frame_optional_text(
            &mut h,
            self.jurisdiction.as_ref().map(|value| value.as_str()),
        );
        frame(&mut h, self.rulebook.id.as_str().as_bytes());
        frame(&mut h, self.rulebook.version.as_bytes());
        frame(&mut h, &self.rulebook.digest.0);
        frame(&mut h, self.system.as_str().as_bytes());
        frame(&mut h, self.operation_kind.as_str().as_bytes());
        frame(&mut h, self.semantic_profile.as_str().as_bytes());
        frame(&mut h, &[side_effect_code(self.side_effect_class)]);
        frame_optional_text(
            &mut h,
            self.target_object_type.as_ref().map(|value| value.as_str()),
        );
        frame(&mut h, self.required_capability.as_str().as_bytes());
        frame(&mut h, &self.valid_from_ms.to_le_bytes());
        frame(&mut h, &self.valid_until_ms.to_le_bytes());
        frame(&mut h, self.authority_ref.as_bytes());
        frame(&mut h, self.policy_proof_ref.as_bytes());
        Ok(Digest32(*h.finalize().as_bytes()))
    }
}

/// Evidence-shaped authentication of one immutable policy record.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedIntegrationCapabilityPolicyRecordProof {
    pub protocol_version: String,
    pub policy_digest: Digest32,
    pub policy_profile: String,
    pub policy_record_ref: String,
    pub record_proof_ref: String,
    pub record_verifier_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

/// Evidence-shaped verification of institutional adoption of the exact policy.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedIntegrationCapabilityPolicyAdoptionProof {
    pub protocol_version: String,
    pub policy_digest: Digest32,
    pub policy_profile: String,
    pub verified_authority_ref: String,
    pub verified_policy_proof_ref: String,
    pub authority_verifier_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

/// Non-deserializable proof that exact policy semantics, immutable-record proof,
/// and institutional adoption agree. Currentness remains intentionally pending.
#[derive(Clone, Debug, Serialize)]
pub struct QualifiedIntegrationCapabilityPolicy {
    policy: IntegrationCapabilityPolicy,
    policy_record_ref: String,
    policy_digest: Digest32,
    qualification_digest: Digest32,
    evidence_digest: Digest32,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedIntegrationCapabilityPolicy {
    pub fn policy(&self) -> &IntegrationCapabilityPolicy {
        &self.policy
    }

    pub fn policy_record_ref(&self) -> &str {
        &self.policy_record_ref
    }

    pub fn policy_digest(&self) -> Digest32 {
        self.policy_digest
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &'static str {
        QUALIFICATION_PROFILE
    }

    pub fn evidence_digest(&self) -> Digest32 {
        self.evidence_digest
    }

    pub fn evidence_profile(&self) -> &'static str {
        EVIDENCE_PROFILE
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub const fn semantic_mapping_bound_here(&self) -> bool {
        true
    }

    pub const fn institutional_adoption_evidence_bound_here(&self) -> bool {
        true
    }

    pub const fn record_proof_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn adoption_proof_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn generation_currentness_verified_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

/// Bind one exact typed command to the capability semantics of a qualified
/// adopted policy. This still does not prove that the policy is currently Active.
#[derive(Clone, Debug, Serialize)]
pub struct QualifiedIntegrationCapabilityMapping {
    policy_digest: Digest32,
    required_capability: CapabilityId,
    command_commitment: ContentCommitment,
    mapping_digest: Digest32,
    valid_until_ms: u64,
}

impl QualifiedIntegrationCapabilityMapping {
    pub fn policy_digest(&self) -> Digest32 {
        self.policy_digest
    }

    pub fn required_capability(&self) -> &CapabilityId {
        &self.required_capability
    }

    pub fn command_commitment(&self) -> &ContentCommitment {
        &self.command_commitment
    }

    pub fn mapping_digest(&self) -> Digest32 {
        self.mapping_digest
    }

    pub fn mapping_profile(&self) -> &'static str {
        COMMAND_MAPPING_PROFILE
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub const fn exact_command_semantics_matched_here(&self) -> bool {
        true
    }

    pub const fn generation_currentness_verified_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub fn qualify_integration_capability_policy(
    policy: &IntegrationCapabilityPolicy,
    policy_record_ref: &str,
    record_proof: &VerifiedIntegrationCapabilityPolicyRecordProof,
    adoption_proof: &VerifiedIntegrationCapabilityPolicyAdoptionProof,
    now_ms: u64,
) -> Result<QualifiedIntegrationCapabilityPolicy, IntegrationCapabilityPolicyError> {
    policy.validate()?;
    if !policy.is_active_at(now_ms) {
        return Err(IntegrationCapabilityPolicyError::PolicyNotActive);
    }
    require_text(policy_record_ref)?;
    let policy_digest = policy.identity_digest()?;
    verify_record_proof(record_proof, policy_digest, policy_record_ref, now_ms)?;
    verify_adoption_proof(adoption_proof, policy, policy_digest, now_ms)?;

    let verified_at_ms = record_proof.verified_at_ms.max(adoption_proof.verified_at_ms);
    let valid_until_ms = policy
        .valid_until_ms
        .min(record_proof.valid_until_ms)
        .min(adoption_proof.valid_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(IntegrationCapabilityPolicyError::NoUsableQualificationWindow);
    }

    let qualification_digest = qualification_digest(policy_digest);
    let evidence_digest = evidence_digest(
        qualification_digest,
        policy_record_ref,
        record_proof,
        adoption_proof,
        verified_at_ms,
        valid_until_ms,
    );

    Ok(QualifiedIntegrationCapabilityPolicy {
        policy: policy.clone(),
        policy_record_ref: policy_record_ref.to_owned(),
        policy_digest,
        qualification_digest,
        evidence_digest,
        verified_at_ms,
        valid_until_ms,
    })
}

pub fn qualify_command_capability<C>(
    policy: &QualifiedIntegrationCapabilityPolicy,
    command: &IntegrationCommand<C>,
    now_ms: u64,
) -> Result<QualifiedIntegrationCapabilityMapping, IntegrationCapabilityPolicyError>
where
    C: CanonicalEncodeV1,
{
    if now_ms == 0 || policy.verified_at_ms > now_ms || policy.valid_until_ms <= now_ms {
        return Err(IntegrationCapabilityPolicyError::NoUsableQualificationWindow);
    }
    let expected = policy.policy();
    if command.system != expected.system {
        return Err(IntegrationCapabilityPolicyError::SystemMismatch);
    }
    if command.operation_kind != expected.operation_kind {
        return Err(IntegrationCapabilityPolicyError::OperationMismatch);
    }
    if command.semantic_profile != expected.semantic_profile {
        return Err(IntegrationCapabilityPolicyError::SemanticProfileMismatch);
    }
    if command.side_effect_class != expected.side_effect_class {
        return Err(IntegrationCapabilityPolicyError::SideEffectClassMismatch);
    }
    if let Some(required_target_type) = &expected.target_object_type {
        let actual = command.target.as_ref().map(|target| &target.object_type);
        if actual != Some(required_target_type) {
            return Err(IntegrationCapabilityPolicyError::TargetObjectTypeMismatch);
        }
    }
    if command
        .target
        .as_ref()
        .is_some_and(|target| target.system != command.system)
    {
        return Err(IntegrationCapabilityPolicyError::TargetSystemMismatch);
    }

    let command_commitment = command.canonical_commitment_v1();
    let mapping_digest = command_mapping_digest(
        policy.policy_digest,
        &expected.required_capability,
        &command_commitment,
    );
    Ok(QualifiedIntegrationCapabilityMapping {
        policy_digest: policy.policy_digest,
        required_capability: expected.required_capability.clone(),
        command_commitment,
        mapping_digest,
        valid_until_ms: policy.valid_until_ms,
    })
}

fn verify_record_proof(
    proof: &VerifiedIntegrationCapabilityPolicyRecordProof,
    policy_digest: Digest32,
    policy_record_ref: &str,
    now_ms: u64,
) -> Result<(), IntegrationCapabilityPolicyError> {
    if proof.protocol_version != POLICY_RECORD_PROOF_PROTOCOL {
        return Err(IntegrationCapabilityPolicyError::WrongRecordProofProtocol);
    }
    if proof.policy_digest != policy_digest || proof.policy_profile != POLICY_IDENTITY_PROFILE {
        return Err(IntegrationCapabilityPolicyError::RecordPolicyIdentityMismatch);
    }
    if proof.policy_record_ref != policy_record_ref {
        return Err(IntegrationCapabilityPolicyError::PolicyRecordRefMismatch);
    }
    for value in [
        proof.policy_record_ref.as_str(),
        proof.record_proof_ref.as_str(),
        proof.record_verifier_ref.as_str(),
        proof.verification_ref.as_str(),
    ] {
        require_text(value)?;
    }
    validate_window(proof.verified_at_ms, proof.valid_until_ms, now_ms)
}

fn verify_adoption_proof(
    proof: &VerifiedIntegrationCapabilityPolicyAdoptionProof,
    policy: &IntegrationCapabilityPolicy,
    policy_digest: Digest32,
    now_ms: u64,
) -> Result<(), IntegrationCapabilityPolicyError> {
    if proof.protocol_version != POLICY_ADOPTION_PROOF_PROTOCOL {
        return Err(IntegrationCapabilityPolicyError::WrongAdoptionProofProtocol);
    }
    if proof.policy_digest != policy_digest || proof.policy_profile != POLICY_IDENTITY_PROFILE {
        return Err(IntegrationCapabilityPolicyError::AdoptionPolicyIdentityMismatch);
    }
    if proof.verified_authority_ref != policy.authority_ref {
        return Err(IntegrationCapabilityPolicyError::AdoptionAuthorityMismatch);
    }
    if proof.verified_policy_proof_ref != policy.policy_proof_ref {
        return Err(IntegrationCapabilityPolicyError::AdoptionProofRefMismatch);
    }
    for value in [
        proof.verified_authority_ref.as_str(),
        proof.verified_policy_proof_ref.as_str(),
        proof.authority_verifier_ref.as_str(),
        proof.verification_ref.as_str(),
    ] {
        require_text(value)?;
    }
    validate_window(proof.verified_at_ms, proof.valid_until_ms, now_ms)
}

fn qualification_digest(policy_digest: Digest32) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_QUALIFICATION);
    frame(&mut h, QUALIFICATION_PROFILE.as_bytes());
    frame(&mut h, &policy_digest.0);
    Digest32(*h.finalize().as_bytes())
}

fn evidence_digest(
    qualification_digest: Digest32,
    policy_record_ref: &str,
    record: &VerifiedIntegrationCapabilityPolicyRecordProof,
    adoption: &VerifiedIntegrationCapabilityPolicyAdoptionProof,
    verified_at_ms: u64,
    valid_until_ms: u64,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_EVIDENCE);
    frame(&mut h, EVIDENCE_PROFILE.as_bytes());
    frame(&mut h, &qualification_digest.0);
    frame(&mut h, policy_record_ref.as_bytes());
    frame(&mut h, record.record_proof_ref.as_bytes());
    frame(&mut h, record.record_verifier_ref.as_bytes());
    frame(&mut h, record.verification_ref.as_bytes());
    frame(&mut h, adoption.authority_verifier_ref.as_bytes());
    frame(&mut h, adoption.verification_ref.as_bytes());
    frame(&mut h, &verified_at_ms.to_le_bytes());
    frame(&mut h, &valid_until_ms.to_le_bytes());
    Digest32(*h.finalize().as_bytes())
}

fn command_mapping_digest(
    policy_digest: Digest32,
    capability: &CapabilityId,
    command_commitment: &ContentCommitment,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_COMMAND_MAPPING);
    frame(&mut h, COMMAND_MAPPING_PROFILE.as_bytes());
    frame(&mut h, &policy_digest.0);
    frame(&mut h, capability.as_str().as_bytes());
    frame(&mut h, &command_commitment.digest);
    Digest32(*h.finalize().as_bytes())
}

fn validate_window(
    verified_at_ms: u64,
    valid_until_ms: u64,
    now_ms: u64,
) -> Result<(), IntegrationCapabilityPolicyError> {
    if now_ms == 0
        || verified_at_ms == 0
        || verified_at_ms > now_ms
        || valid_until_ms <= now_ms
        || valid_until_ms < verified_at_ms
    {
        Err(IntegrationCapabilityPolicyError::InvalidVerificationWindow)
    } else {
        Ok(())
    }
}

fn side_effect_code(value: SideEffectClass) -> u8 {
    match value {
        SideEffectClass::ReadOnly => 1,
        SideEffectClass::Reversible => 2,
        SideEffectClass::Compensatable => 3,
        SideEffectClass::Irreversible => 4,
    }
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

fn frame_optional_text(h: &mut blake3::Hasher, value: Option<&str>) {
    match value {
        Some(value) => {
            frame(h, &[1]);
            frame(h, value.as_bytes());
        }
        None => frame(h, &[0]),
    }
}

fn require_text(value: &str) -> Result<(), IntegrationCapabilityPolicyError> {
    if value.trim().is_empty() || value.len() > MAX_TEXT_BYTES || value.chars().any(char::is_control)
    {
        Err(IntegrationCapabilityPolicyError::InvalidText)
    } else {
        Ok(())
    }
}

fn require_profile(value: &str) -> Result<(), IntegrationCapabilityPolicyError> {
    if value.is_empty()
        || value.len() > MAX_PROFILE_BYTES
        || !value.bytes().all(|b| {
            b.is_ascii_lowercase()
                || b.is_ascii_digit()
                || matches!(b, b'.' | b'_' | b'/' | b'-' | b':' | b'@')
        })
    {
        Err(IntegrationCapabilityPolicyError::InvalidProfile)
    } else {
        Ok(())
    }
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum IntegrationCapabilityPolicyError {
    #[error("wrong integration capability policy protocol version")]
    WrongProtocolVersion,
    #[error("wrong policy record-proof protocol")]
    WrongRecordProofProtocol,
    #[error("wrong policy adoption-proof protocol")]
    WrongAdoptionProofProtocol,
    #[error("invalid text field")]
    InvalidText,
    #[error("invalid semantic profile")]
    InvalidProfile,
    #[error("invalid institutional rulebook")]
    InvalidRulebook,
    #[error("invalid policy validity window")]
    InvalidPolicyWindow,
    #[error("policy is not semantically active at the requested instant")]
    PolicyNotActive,
    #[error("invalid verifier evidence window")]
    InvalidVerificationWindow,
    #[error("record proof binds a different policy identity")]
    RecordPolicyIdentityMismatch,
    #[error("record proof binds a different policy record")]
    PolicyRecordRefMismatch,
    #[error("adoption proof binds a different policy identity")]
    AdoptionPolicyIdentityMismatch,
    #[error("adoption proof binds a different institutional authority")]
    AdoptionAuthorityMismatch,
    #[error("adoption proof binds a different policy proof reference")]
    AdoptionProofRefMismatch,
    #[error("record/adoption evidence leaves no usable qualification window")]
    NoUsableQualificationWindow,
    #[error("integration command belongs to a different external system")]
    SystemMismatch,
    #[error("integration command operation kind is not covered by the policy")]
    OperationMismatch,
    #[error("integration command semantic profile is not covered by the policy")]
    SemanticProfileMismatch,
    #[error("integration command side-effect classification is not covered by the policy")]
    SideEffectClassMismatch,
    #[error("integration command target object type is not covered by the policy")]
    TargetObjectTypeMismatch,
    #[error("integration command target belongs to a different external system")]
    TargetSystemMismatch,
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_institutional_core::{RulebookId, ValidationError};
    use mycelix_integration_core::{
        ConnectorInstanceId, ExternalObjectRef, ExternalOpaqueId, IdempotencyKey,
        IntegrationCommandId,
    };

    #[derive(Clone, Debug, PartialEq, Eq)]
    struct DemoPayload(u64);

    impl CanonicalEncodeV1 for DemoPayload {
        fn canonical_preimage_v1(&self) -> Vec<u8> {
            let mut out = b"CAPABILITY-DEMO\0V1\0".to_vec();
            out.extend_from_slice(&self.0.to_be_bytes());
            out
        }
    }

    fn iid<T>(value: &str, constructor: fn(String) -> Result<T, ValidationError>) -> T {
        constructor(value.to_owned()).unwrap()
    }

    fn policy() -> IntegrationCapabilityPolicy {
        IntegrationCapabilityPolicy {
            protocol_version: PROTOCOL_VERSION.into(),
            policy_id: "policy:payments:create-transfer:v1".into(),
            institution: iid("institution:acme", InstitutionId::new),
            jurisdiction: None,
            rulebook: RulebookRef {
                id: iid("rulebook:payments", RulebookId::new),
                version: "1".into(),
                digest: Digest32([7; 32]),
            },
            system: ExternalSystemId::new("provider-x").unwrap(),
            operation_kind: ExternalOperationKind::new("create-transfer").unwrap(),
            semantic_profile: SemanticProfileId::new("transfer@1").unwrap(),
            side_effect_class: SideEffectClass::Irreversible,
            target_object_type: Some(ExternalObjectType::new("account").unwrap()),
            required_capability: iid("payments.transfer.execute", CapabilityId::new),
            valid_from_ms: 1_000,
            valid_until_ms: 20_000,
            authority_ref: "governance:decision:cap-map-1".into(),
            policy_proof_ref: "proof:cap-map-1".into(),
        }
    }

    fn record(policy: &IntegrationCapabilityPolicy) -> VerifiedIntegrationCapabilityPolicyRecordProof {
        VerifiedIntegrationCapabilityPolicyRecordProof {
            protocol_version: POLICY_RECORD_PROOF_PROTOCOL.into(),
            policy_digest: policy.identity_digest().unwrap(),
            policy_profile: POLICY_IDENTITY_PROFILE.into(),
            policy_record_ref: "record:cap-map-1".into(),
            record_proof_ref: "record-proof:1".into(),
            record_verifier_ref: "verifier:record:1".into(),
            verification_ref: "verification:record:1".into(),
            verified_at_ms: 2_000,
            valid_until_ms: 15_000,
        }
    }

    fn adoption(policy: &IntegrationCapabilityPolicy) -> VerifiedIntegrationCapabilityPolicyAdoptionProof {
        VerifiedIntegrationCapabilityPolicyAdoptionProof {
            protocol_version: POLICY_ADOPTION_PROOF_PROTOCOL.into(),
            policy_digest: policy.identity_digest().unwrap(),
            policy_profile: POLICY_IDENTITY_PROFILE.into(),
            verified_authority_ref: policy.authority_ref.clone(),
            verified_policy_proof_ref: policy.policy_proof_ref.clone(),
            authority_verifier_ref: "verifier:adoption:1".into(),
            verification_ref: "verification:adoption:1".into(),
            verified_at_ms: 2_100,
            valid_until_ms: 14_000,
        }
    }

    fn command() -> IntegrationCommand<DemoPayload> {
        IntegrationCommand {
            command_id: IntegrationCommandId::new("cmd-1").unwrap(),
            connector_instance: ConnectorInstanceId::new("provider-prod-1").unwrap(),
            system: ExternalSystemId::new("provider-x").unwrap(),
            operation_kind: ExternalOperationKind::new("create-transfer").unwrap(),
            target: Some(ExternalObjectRef {
                system: ExternalSystemId::new("provider-x").unwrap(),
                object_type: ExternalObjectType::new("account").unwrap(),
                external_id: ExternalOpaqueId::new("acct-1").unwrap(),
            }),
            side_effect_class: SideEffectClass::Irreversible,
            idempotency_key: Some(IdempotencyKey::new("idem-1").unwrap()),
            semantic_profile: SemanticProfileId::new("transfer@1").unwrap(),
            payload: DemoPayload(5000),
        }
    }

    #[test]
    fn adopted_policy_maps_exact_command_without_claiming_currentness() {
        let semantic = policy();
        let qualified = qualify_integration_capability_policy(
            &semantic,
            "record:cap-map-1",
            &record(&semantic),
            &adoption(&semantic),
            3_000,
        )
        .unwrap();
        let mapped = qualify_command_capability(&qualified, &command(), 3_100).unwrap();
        assert_eq!(mapped.required_capability(), &semantic.required_capability);
        assert!(mapped.exact_command_semantics_matched_here());
        assert!(!mapped.generation_currentness_verified_here());
        assert!(!mapped.grants_execution_authority());
        assert!(!qualified.generation_currentness_verified_here());
    }

    #[test]
    fn operation_substitution_fails_closed() {
        let semantic = policy();
        let qualified = qualify_integration_capability_policy(
            &semantic,
            "record:cap-map-1",
            &record(&semantic),
            &adoption(&semantic),
            3_000,
        )
        .unwrap();
        let mut changed = command();
        changed.operation_kind = ExternalOperationKind::new("refund-transfer").unwrap();
        assert_eq!(
            qualify_command_capability(&qualified, &changed, 3_100).unwrap_err(),
            IntegrationCapabilityPolicyError::OperationMismatch
        );
    }

    #[test]
    fn semantic_profile_substitution_fails_closed() {
        let semantic = policy();
        let qualified = qualify_integration_capability_policy(
            &semantic,
            "record:cap-map-1",
            &record(&semantic),
            &adoption(&semantic),
            3_000,
        )
        .unwrap();
        let mut changed = command();
        changed.semantic_profile = SemanticProfileId::new("transfer@2").unwrap();
        assert_eq!(
            qualify_command_capability(&qualified, &changed, 3_100).unwrap_err(),
            IntegrationCapabilityPolicyError::SemanticProfileMismatch
        );
    }

    #[test]
    fn side_effect_downgrade_fails_closed() {
        let semantic = policy();
        let qualified = qualify_integration_capability_policy(
            &semantic,
            "record:cap-map-1",
            &record(&semantic),
            &adoption(&semantic),
            3_000,
        )
        .unwrap();
        let mut changed = command();
        changed.side_effect_class = SideEffectClass::ReadOnly;
        assert_eq!(
            qualify_command_capability(&qualified, &changed, 3_100).unwrap_err(),
            IntegrationCapabilityPolicyError::SideEffectClassMismatch
        );
    }

    #[test]
    fn alternate_adoption_authority_fails_closed() {
        let semantic = policy();
        let mut proof = adoption(&semantic);
        proof.verified_authority_ref = "governance:attacker".into();
        assert_eq!(
            qualify_integration_capability_policy(
                &semantic,
                "record:cap-map-1",
                &record(&semantic),
                &proof,
                3_000,
            )
            .unwrap_err(),
            IntegrationCapabilityPolicyError::AdoptionAuthorityMismatch
        );
    }

    #[test]
    fn target_type_scope_is_exact() {
        let semantic = policy();
        let qualified = qualify_integration_capability_policy(
            &semantic,
            "record:cap-map-1",
            &record(&semantic),
            &adoption(&semantic),
            3_000,
        )
        .unwrap();
        let mut changed = command();
        changed.target.as_mut().unwrap().object_type = ExternalObjectType::new("customer").unwrap();
        assert_eq!(
            qualify_command_capability(&qualified, &changed, 3_100).unwrap_err(),
            IntegrationCapabilityPolicyError::TargetObjectTypeMismatch
        );
    }
}
