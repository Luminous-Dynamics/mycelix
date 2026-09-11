//! Generation-bound currentness for institution-adopted integration capability semantics.
//!
//! This crate composes the adopted semantic policy from
//! `mycelix-integration-capability-policy` with the shared closed-set authority
//! freshness theorem. It does not introduce another currentness mechanism.

use mycelix_authority_freshness::{
    qualify_current_freshness, AuthoritySubjectKind, AuthoritySubjectRef, FreshnessError,
    ProfiledDigest, VerifiedAuthorityFreshness,
};
use mycelix_institutional_core::{CapabilityId, Digest32};
use mycelix_integration_capability_policy::{
    qualify_command_capability, IntegrationCapabilityPolicyError,
    QualifiedIntegrationCapabilityMapping, QualifiedIntegrationCapabilityPolicy,
    POLICY_IDENTITY_PROFILE,
};
use mycelix_integration_core::{CanonicalEncodeV1, ContentCommitment, IntegrationCommand};
use serde::Serialize;
use thiserror::Error;

pub const CURRENT_POLICY_QUALIFICATION_PROFILE: &str =
    "mycelix-integration-capability-current-policy-v1-blake3-framed";
pub const CURRENT_MAPPING_PROFILE: &str =
    "mycelix-integration-capability-current-mapping-v1-blake3-framed";

const DOMAIN_CURRENT_POLICY: &[u8] = b"mycelix/integration/capability-current-policy/v1";
const DOMAIN_CURRENT_MAPPING: &[u8] = b"mycelix/integration/capability-current-mapping/v1";

/// Non-deserializable proof that the exact institution-adopted capability policy
/// is the exact generation-bound Active authority subject at this instant.
#[derive(Clone, Debug, Serialize)]
pub struct QualifiedCurrentIntegrationCapabilityPolicy {
    adopted_policy: QualifiedIntegrationCapabilityPolicy,
    subject: AuthoritySubjectRef,
    freshness_digest: Digest32,
    freshness_profile: String,
    qualification_digest: Digest32,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedCurrentIntegrationCapabilityPolicy {
    pub fn adopted_policy(&self) -> &QualifiedIntegrationCapabilityPolicy {
        &self.adopted_policy
    }

    pub fn subject(&self) -> &AuthoritySubjectRef {
        &self.subject
    }

    pub fn required_capability(&self) -> &CapabilityId {
        &self.adopted_policy.policy().required_capability
    }

    pub fn freshness_digest(&self) -> Digest32 {
        self.freshness_digest
    }

    pub fn freshness_profile(&self) -> &str {
        &self.freshness_profile
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &'static str {
        CURRENT_POLICY_QUALIFICATION_PROFILE
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub const fn semantic_mapping_adopted_here(&self) -> bool {
        true
    }

    pub const fn generation_currentness_verified_here(&self) -> bool {
        true
    }

    pub const fn freshness_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn executor_capability_matched_here(&self) -> bool {
        false
    }

    pub const fn provider_profile_bound_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

/// Exact command mapping under one current, institution-adopted capability
/// policy. Retaining the actual non-deserializable current policy prevents a
/// mapping qualified under generation N from being rebound to generation N+1
/// merely because the semantic policy digest stayed unchanged.
#[derive(Clone, Debug, Serialize)]
pub struct QualifiedCurrentIntegrationCapabilityMapping {
    current_policy: QualifiedCurrentIntegrationCapabilityPolicy,
    mapping: QualifiedIntegrationCapabilityMapping,
    current_mapping_digest: Digest32,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedCurrentIntegrationCapabilityMapping {
    pub fn current_policy(&self) -> &QualifiedCurrentIntegrationCapabilityPolicy {
        &self.current_policy
    }

    pub fn mapping(&self) -> &QualifiedIntegrationCapabilityMapping {
        &self.mapping
    }

    pub fn required_capability(&self) -> &CapabilityId {
        self.mapping.required_capability()
    }

    pub fn command_commitment(&self) -> &ContentCommitment {
        self.mapping.command_commitment()
    }

    pub fn current_mapping_digest(&self) -> Digest32 {
        self.current_mapping_digest
    }

    pub fn current_mapping_profile(&self) -> &'static str {
        CURRENT_MAPPING_PROFILE
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub const fn exact_command_semantics_current_here(&self) -> bool {
        true
    }

    pub const fn executor_capability_matched_here(&self) -> bool {
        false
    }

    pub const fn attempt_bound_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub fn qualify_current_integration_capability_policy(
    adopted_policy: &QualifiedIntegrationCapabilityPolicy,
    freshness: &VerifiedAuthorityFreshness,
    now_ms: u64,
) -> Result<QualifiedCurrentIntegrationCapabilityPolicy, IntegrationCapabilityCurrentnessError> {
    if now_ms == 0
        || adopted_policy.verified_at_ms() > now_ms
        || adopted_policy.valid_until_ms() <= now_ms
    {
        return Err(IntegrationCapabilityCurrentnessError::AdoptedPolicyNotLive);
    }

    let policy = adopted_policy.policy();
    let subject = AuthoritySubjectRef {
        kind: AuthoritySubjectKind::IntegrationCapabilityPolicy,
        namespace: policy.institution.as_str().to_owned(),
        subject_id: policy.policy_id.clone(),
        identity: ProfiledDigest {
            digest: adopted_policy.policy_digest(),
            profile: POLICY_IDENTITY_PROFILE.to_owned(),
        },
    };

    // A current-state fact cannot predate the semantic policy generation that it
    // claims to activate/revoke/supersede.
    if freshness.snapshot.effective_at_ms < policy.valid_from_ms {
        return Err(IntegrationCapabilityCurrentnessError::FreshnessPredatesPolicy);
    }

    let current = qualify_current_freshness(
        std::slice::from_ref(&subject),
        std::slice::from_ref(freshness),
        now_ms,
    )?;

    let verified_at_ms = adopted_policy.verified_at_ms().max(current.verified_at_ms);
    let valid_until_ms = adopted_policy.valid_until_ms().min(current.lease_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(IntegrationCapabilityCurrentnessError::NoUsableCurrentWindow);
    }

    let qualification_digest = current_policy_digest(
        adopted_policy.qualification_digest(),
        current.freshness_digest,
    );

    Ok(QualifiedCurrentIntegrationCapabilityPolicy {
        adopted_policy: adopted_policy.clone(),
        subject,
        freshness_digest: current.freshness_digest,
        freshness_profile: current.freshness_profile,
        qualification_digest,
        verified_at_ms,
        valid_until_ms,
    })
}

pub fn qualify_current_command_capability<C>(
    current_policy: &QualifiedCurrentIntegrationCapabilityPolicy,
    command: &IntegrationCommand<C>,
    now_ms: u64,
) -> Result<QualifiedCurrentIntegrationCapabilityMapping, IntegrationCapabilityCurrentnessError>
where
    C: CanonicalEncodeV1,
{
    if now_ms == 0
        || current_policy.verified_at_ms > now_ms
        || current_policy.valid_until_ms <= now_ms
    {
        return Err(IntegrationCapabilityCurrentnessError::NoUsableCurrentWindow);
    }

    let mapping = qualify_command_capability(current_policy.adopted_policy(), command, now_ms)?;
    let valid_until_ms = current_policy.valid_until_ms.min(mapping.valid_until_ms());
    if valid_until_ms <= now_ms {
        return Err(IntegrationCapabilityCurrentnessError::NoUsableCurrentWindow);
    }
    let current_mapping_digest = current_mapping_digest(
        current_policy.qualification_digest,
        mapping.mapping_digest(),
    );

    Ok(QualifiedCurrentIntegrationCapabilityMapping {
        current_policy: current_policy.clone(),
        mapping,
        current_mapping_digest,
        verified_at_ms: current_policy.verified_at_ms,
        valid_until_ms,
    })
}

fn current_policy_digest(adopted_qualification: Digest32, freshness: Digest32) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_CURRENT_POLICY);
    frame(&mut h, CURRENT_POLICY_QUALIFICATION_PROFILE.as_bytes());
    frame(&mut h, &adopted_qualification.0);
    frame(&mut h, &freshness.0);
    Digest32(*h.finalize().as_bytes())
}

fn current_mapping_digest(current_policy: Digest32, mapping: Digest32) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_CURRENT_MAPPING);
    frame(&mut h, CURRENT_MAPPING_PROFILE.as_bytes());
    frame(&mut h, &current_policy.0);
    frame(&mut h, &mapping.0);
    Digest32(*h.finalize().as_bytes())
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error)]
pub enum IntegrationCapabilityCurrentnessError {
    #[error("adopted capability policy is not live at the requested instant")]
    AdoptedPolicyNotLive,
    #[error("current-state evidence predates the semantic capability policy")]
    FreshnessPredatesPolicy,
    #[error("policy/currentness evidence leaves no usable current window")]
    NoUsableCurrentWindow,
    #[error(transparent)]
    Freshness(#[from] FreshnessError),
    #[error(transparent)]
    CapabilityPolicy(#[from] IntegrationCapabilityPolicyError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{
        AuthorityFreshnessSnapshot, AuthorityFreshnessState,
        PROTOCOL_VERSION as FRESHNESS_PROTOCOL_VERSION,
    };
    use mycelix_institutional_core::{
        CapabilityId, InstitutionId, RulebookId, RulebookRef, ValidationError,
    };
    use mycelix_integration_capability_policy::{
        qualify_integration_capability_policy, IntegrationCapabilityPolicy,
        VerifiedIntegrationCapabilityPolicyAdoptionProof,
        VerifiedIntegrationCapabilityPolicyRecordProof, POLICY_ADOPTION_PROOF_PROTOCOL,
        POLICY_RECORD_PROOF_PROTOCOL, PROTOCOL_VERSION as POLICY_PROTOCOL_VERSION,
    };
    use mycelix_integration_core::{
        ConnectorInstanceId, ExternalObjectRef, ExternalObjectType, ExternalOpaqueId,
        ExternalOperationKind, ExternalSystemId, IdempotencyKey, IntegrationCommandId,
        SemanticProfileId, SideEffectClass,
    };

    #[derive(Clone, Debug, PartialEq, Eq)]
    struct DemoPayload(u64);

    impl CanonicalEncodeV1 for DemoPayload {
        fn canonical_preimage_v1(&self) -> Vec<u8> {
            let mut out = b"CAPABILITY-CURRENTNESS-DEMO\0V1\0".to_vec();
            out.extend_from_slice(&self.0.to_be_bytes());
            out
        }
    }

    fn iid<T>(value: &str, constructor: fn(String) -> Result<T, ValidationError>) -> T {
        constructor(value.to_owned()).unwrap()
    }

    fn semantic_policy() -> IntegrationCapabilityPolicy {
        IntegrationCapabilityPolicy {
            protocol_version: POLICY_PROTOCOL_VERSION.into(),
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

    fn adopted() -> QualifiedIntegrationCapabilityPolicy {
        let policy = semantic_policy();
        let digest = policy.identity_digest().unwrap();
        let record = VerifiedIntegrationCapabilityPolicyRecordProof {
            protocol_version: POLICY_RECORD_PROOF_PROTOCOL.into(),
            policy_digest: digest,
            policy_profile: POLICY_IDENTITY_PROFILE.into(),
            policy_record_ref: "record:cap-map-1".into(),
            record_proof_ref: "record-proof:1".into(),
            record_verifier_ref: "verifier:record:1".into(),
            verification_ref: "verification:record:1".into(),
            verified_at_ms: 2_000,
            valid_until_ms: 15_000,
        };
        let adoption = VerifiedIntegrationCapabilityPolicyAdoptionProof {
            protocol_version: POLICY_ADOPTION_PROOF_PROTOCOL.into(),
            policy_digest: digest,
            policy_profile: POLICY_IDENTITY_PROFILE.into(),
            verified_authority_ref: policy.authority_ref.clone(),
            verified_policy_proof_ref: policy.policy_proof_ref.clone(),
            authority_verifier_ref: "verifier:adoption:1".into(),
            verification_ref: "verification:adoption:1".into(),
            verified_at_ms: 2_100,
            valid_until_ms: 14_000,
        };
        qualify_integration_capability_policy(
            &policy,
            "record:cap-map-1",
            &record,
            &adoption,
            3_000,
        )
        .unwrap()
    }

    fn freshness(adopted: &QualifiedIntegrationCapabilityPolicy) -> VerifiedAuthorityFreshness {
        let policy = adopted.policy();
        VerifiedAuthorityFreshness {
            snapshot: AuthorityFreshnessSnapshot {
                protocol_version: FRESHNESS_PROTOCOL_VERSION.into(),
                subject: AuthoritySubjectRef {
                    kind: AuthoritySubjectKind::IntegrationCapabilityPolicy,
                    namespace: policy.institution.as_str().to_owned(),
                    subject_id: policy.policy_id.clone(),
                    identity: ProfiledDigest {
                        digest: adopted.policy_digest(),
                        profile: POLICY_IDENTITY_PROFILE.into(),
                    },
                },
                generation: 1,
                state: AuthorityFreshnessState::Active,
                effective_at_ms: 1_500,
                status_record_ref: "status:cap-policy:g1".into(),
            },
            authoritative_source_ref: "authority-status-source:1".into(),
            verification_ref: "verification:freshness:1".into(),
            verified_at_ms: 3_100,
            lease_until_ms: 8_000,
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
    fn exact_active_generation_closes_policy_currentness_only() {
        let adopted = adopted();
        let current = qualify_current_integration_capability_policy(
            &adopted,
            &freshness(&adopted),
            3_200,
        )
        .unwrap();
        assert_eq!(current.subject().kind, AuthoritySubjectKind::IntegrationCapabilityPolicy);
        assert!(current.generation_currentness_verified_here());
        assert!(!current.executor_capability_matched_here());
        assert!(!current.grants_execution_authority());
        assert_eq!(current.valid_until_ms(), 8_000);
    }

    #[test]
    fn revoked_policy_fails_closed() {
        let adopted = adopted();
        let mut receipt = freshness(&adopted);
        receipt.snapshot.state = AuthorityFreshnessState::Revoked;
        assert!(matches!(
            qualify_current_integration_capability_policy(&adopted, &receipt, 3_200),
            Err(IntegrationCapabilityCurrentnessError::Freshness(
                FreshnessError::SubjectNotActive
            ))
        ));
    }

    #[test]
    fn wrong_policy_identity_fails_closed() {
        let adopted = adopted();
        let mut receipt = freshness(&adopted);
        receipt.snapshot.subject.identity.digest = Digest32([9; 32]);
        assert!(matches!(
            qualify_current_integration_capability_policy(&adopted, &receipt, 3_200),
            Err(IntegrationCapabilityCurrentnessError::Freshness(
                FreshnessError::UnexpectedSubject
            ))
                | Err(IntegrationCapabilityCurrentnessError::Freshness(
                    FreshnessError::SubjectIdentityMismatch
                ))
        ));
    }

    #[test]
    fn freshness_cannot_predate_policy_activation() {
        let adopted = adopted();
        let mut receipt = freshness(&adopted);
        receipt.snapshot.effective_at_ms = 999;
        assert!(matches!(
            qualify_current_integration_capability_policy(&adopted, &receipt, 3_200),
            Err(IntegrationCapabilityCurrentnessError::FreshnessPredatesPolicy)
        ));
    }

    #[test]
    fn current_policy_maps_exact_command_but_not_executor_or_attempt() {
        let adopted = adopted();
        let current_policy = qualify_current_integration_capability_policy(
            &adopted,
            &freshness(&adopted),
            3_200,
        )
        .unwrap();
        let mapped = qualify_current_command_capability(&current_policy, &command(), 3_300).unwrap();
        assert_eq!(mapped.required_capability(), current_policy.required_capability());
        assert_eq!(
            mapped.current_policy().qualification_digest(),
            current_policy.qualification_digest()
        );
        assert!(mapped.exact_command_semantics_current_here());
        assert!(!mapped.executor_capability_matched_here());
        assert!(!mapped.attempt_bound_here());
        assert!(!mapped.grants_execution_authority());
    }
}
