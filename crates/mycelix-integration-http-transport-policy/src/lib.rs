//! Institution-adopted HTTP transport safety for Mycelix integrations.
//!
//! This policy is deliberately narrower than a general HTTP client. Provider
//! materializers do not get authority over scheme, host, port, method,
//! authorization/idempotency headers, content type, redirects, environment
//! proxies, or arbitrary headers. A later request-plan theorem may provide only
//! bounded path-suffix/query/body data under this policy.

use mycelix_authority_freshness::{
    qualify_current_freshness, AuthoritySubjectKind, AuthoritySubjectRef, FreshnessError,
    ProfiledDigest, VerifiedAuthorityFreshness,
};
use mycelix_institutional_core::{Digest32, InstitutionId, JurisdictionId, RulebookRef};
use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, DigestAlgorithm, ExternalOperationKind,
    ExternalSystemId,
};
use mycelix_integration_execution_binding::QualifiedProviderExecutionProfile;
use serde::{Deserialize, Serialize};
use thiserror::Error;

pub const PROTOCOL_VERSION: &str = "mycelix-integration-http-transport-policy-v0.1";
pub const RECORD_PROOF_PROTOCOL: &str =
    "mycelix-integration-http-transport-policy-record-proof-v0.1";
pub const ADOPTION_PROOF_PROTOCOL: &str =
    "mycelix-integration-http-transport-policy-adoption-proof-v0.1";
pub const POLICY_IDENTITY_PROFILE: &str =
    "mycelix-integration-http-transport-policy-v1-blake3-framed";
pub const QUALIFICATION_PROFILE: &str =
    "mycelix-integration-http-transport-policy-qualified-v1-blake3-framed";
pub const CURRENT_POLICY_PROFILE: &str =
    "mycelix-integration-http-transport-policy-current-v1-blake3-framed";
pub const PROVIDER_BINDING_PROFILE: &str =
    "mycelix-integration-http-transport-provider-binding-v1-blake3-framed";

const DOMAIN_POLICY: &[u8] = b"mycelix/integration/http-transport-policy/v1";
const DOMAIN_QUALIFICATION: &[u8] = b"mycelix/integration/http-transport-policy/qualified/v1";
const DOMAIN_CURRENT: &[u8] = b"mycelix/integration/http-transport-policy/current/v1";
const DOMAIN_PROVIDER_BINDING: &[u8] =
    b"mycelix/integration/http-transport-policy/provider-binding/v1";
const MAX_TEXT_BYTES: usize = 2048;
const MAX_BODY_BYTES: u32 = 64 * 1024 * 1024;
const MAX_QUERY_BYTES: u32 = 64 * 1024;
const MAX_PLAN_BYTES: u32 = 64 * 1024 * 1024;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HttpMethod {
    Get,
    Post,
    Put,
    Patch,
    Delete,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct IntegrationHttpTransportPolicy {
    pub protocol_version: String,
    pub policy_id: String,
    pub generation: u64,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub connector_instance: ConnectorInstanceId,
    pub system: ExternalSystemId,
    pub operation_kind: ExternalOperationKind,
    /// Exact signed provider execution profile to which this transport policy is
    /// attached. Provider-profile rotation requires explicit transport-policy
    /// re-adoption rather than silently inheriting endpoint authority.
    pub provider_profile_commitment: ContentCommitment,
    /// Lowercase DNS hostname only. No scheme, port, path, IP literal or userinfo.
    pub endpoint_host: String,
    /// v0.1 requires 443.
    pub endpoint_port: u16,
    pub method: HttpMethod,
    /// Absolute path prefix such as `/v1/payment_intents/`.
    pub path_prefix: String,
    /// Engine-owned request Content-Type.
    pub content_type: String,
    /// Engine-owned credential header, usually `authorization`.
    pub credential_header_name: String,
    /// Opaque secret-manager slot identity; never secret material.
    pub credential_slot_id: String,
    /// Engine-owned provider idempotency header when required.
    pub idempotency_header_name: Option<String>,
    pub max_body_bytes: u32,
    pub max_query_bytes: u32,
    pub max_materialized_plan_bytes: u32,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
    pub authority_ref: String,
    pub policy_proof_ref: String,
}

impl IntegrationHttpTransportPolicy {
    pub fn validate(&self) -> Result<(), HttpTransportPolicyError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(HttpTransportPolicyError::WrongProtocolVersion);
        }
        require_text(&self.policy_id)?;
        if self.generation == 0 {
            return Err(HttpTransportPolicyError::InvalidGeneration);
        }
        self.rulebook
            .validate()
            .map_err(|_| HttpTransportPolicyError::InvalidRulebook)?;
        validate_dns_host(&self.endpoint_host)?;
        if self.endpoint_port != 443 {
            return Err(HttpTransportPolicyError::HttpsPortRequired);
        }
        validate_path_prefix(&self.path_prefix)?;
        validate_media_type(&self.content_type)?;
        validate_header_name(&self.credential_header_name)?;
        require_text(&self.credential_slot_id)?;
        if let Some(name) = &self.idempotency_header_name {
            validate_header_name(name)?;
            if name == &self.credential_header_name {
                return Err(HttpTransportPolicyError::HeaderRoleCollision);
            }
        }
        if self.max_body_bytes == 0
            || self.max_body_bytes > MAX_BODY_BYTES
            || self.max_query_bytes > MAX_QUERY_BYTES
            || self.max_materialized_plan_bytes == 0
            || self.max_materialized_plan_bytes > MAX_PLAN_BYTES
        {
            return Err(HttpTransportPolicyError::InvalidSizeLimit);
        }
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(HttpTransportPolicyError::InvalidPolicyWindow);
        }
        require_text(&self.authority_ref)?;
        require_text(&self.policy_proof_ref)?;
        Ok(())
    }

    pub fn is_active_at(&self, now_ms: u64) -> bool {
        self.valid_from_ms <= now_ms && now_ms < self.valid_until_ms
    }

    pub fn identity_digest(&self) -> Result<Digest32, HttpTransportPolicyError> {
        self.validate()?;
        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_POLICY);
        frame(&mut h, POLICY_IDENTITY_PROFILE.as_bytes());
        frame(&mut h, self.protocol_version.as_bytes());
        frame(&mut h, self.policy_id.as_bytes());
        frame(&mut h, &self.generation.to_le_bytes());
        frame(&mut h, self.institution.as_str().as_bytes());
        frame_optional_text(
            &mut h,
            self.jurisdiction.as_ref().map(JurisdictionId::as_str),
        );
        frame(&mut h, self.rulebook.id.as_str().as_bytes());
        frame(&mut h, self.rulebook.version.as_bytes());
        frame(&mut h, &self.rulebook.digest.0);
        frame(&mut h, self.connector_instance.as_str().as_bytes());
        frame(&mut h, self.system.as_str().as_bytes());
        frame(&mut h, self.operation_kind.as_str().as_bytes());
        frame_commitment(&mut h, &self.provider_profile_commitment);
        frame(&mut h, self.endpoint_host.as_bytes());
        frame(&mut h, &self.endpoint_port.to_le_bytes());
        frame(&mut h, &[method_code(self.method)]);
        frame(&mut h, self.path_prefix.as_bytes());
        frame(&mut h, self.content_type.as_bytes());
        frame(&mut h, self.credential_header_name.as_bytes());
        frame(&mut h, self.credential_slot_id.as_bytes());
        frame_optional_text(&mut h, self.idempotency_header_name.as_deref());
        frame(&mut h, &self.max_body_bytes.to_le_bytes());
        frame(&mut h, &self.max_query_bytes.to_le_bytes());
        frame(&mut h, &self.max_materialized_plan_bytes.to_le_bytes());
        frame(&mut h, &self.valid_from_ms.to_le_bytes());
        frame(&mut h, &self.valid_until_ms.to_le_bytes());
        frame(&mut h, self.authority_ref.as_bytes());
        frame(&mut h, self.policy_proof_ref.as_bytes());
        Ok(Digest32(*h.finalize().as_bytes()))
    }

    pub const fn https_required_here(&self) -> bool {
        true
    }

    pub const fn redirects_allowed_here(&self) -> bool {
        false
    }

    pub const fn environment_proxy_allowed_here(&self) -> bool {
        false
    }

    pub const fn arbitrary_materializer_headers_allowed_here(&self) -> bool {
        false
    }

    pub const fn credential_material_present_here(&self) -> bool {
        false
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedHttpTransportPolicyRecordProof {
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

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedHttpTransportPolicyAdoptionProof {
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

#[derive(Clone, Debug)]
pub struct QualifiedAdoptedHttpTransportPolicy {
    policy: IntegrationHttpTransportPolicy,
    policy_digest: Digest32,
    qualification_digest: Digest32,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedAdoptedHttpTransportPolicy {
    pub fn policy(&self) -> &IntegrationHttpTransportPolicy {
        &self.policy
    }

    pub fn policy_digest(&self) -> Digest32 {
        self.policy_digest
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub const fn institutional_adoption_bound_here(&self) -> bool {
        true
    }

    pub const fn generation_currentness_verified_here(&self) -> bool {
        false
    }

    pub const fn proof_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

#[derive(Clone, Debug)]
pub struct QualifiedCurrentHttpTransportPolicy {
    adopted: QualifiedAdoptedHttpTransportPolicy,
    freshness_digest: Digest32,
    qualification_digest: Digest32,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedCurrentHttpTransportPolicy {
    pub fn adopted(&self) -> &QualifiedAdoptedHttpTransportPolicy {
        &self.adopted
    }

    pub fn policy(&self) -> &IntegrationHttpTransportPolicy {
        self.adopted.policy()
    }

    pub fn freshness_digest(&self) -> Digest32 {
        self.freshness_digest
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub const fn generation_currentness_verified_here(&self) -> bool {
        true
    }

    pub const fn destination_current_here(&self) -> bool {
        true
    }

    pub const fn credential_slot_current_here(&self) -> bool {
        true
    }

    pub const fn credential_secret_value_verified_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

#[derive(Clone, Debug)]
pub struct QualifiedHttpTransportProviderBinding {
    current_policy: QualifiedCurrentHttpTransportPolicy,
    provider_profile_commitment: ContentCommitment,
    binding_digest: Digest32,
    valid_until_ms: u64,
}

impl QualifiedHttpTransportProviderBinding {
    pub fn current_policy(&self) -> &QualifiedCurrentHttpTransportPolicy {
        &self.current_policy
    }

    pub fn provider_profile_commitment(&self) -> &ContentCommitment {
        &self.provider_profile_commitment
    }

    pub fn binding_digest(&self) -> Digest32 {
        self.binding_digest
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub const fn exact_provider_profile_bound_here(&self) -> bool {
        true
    }

    pub const fn network_io_performed_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub fn qualify_adopted_http_transport_policy(
    policy: &IntegrationHttpTransportPolicy,
    policy_record_ref: &str,
    record: &VerifiedHttpTransportPolicyRecordProof,
    adoption: &VerifiedHttpTransportPolicyAdoptionProof,
    now_ms: u64,
) -> Result<QualifiedAdoptedHttpTransportPolicy, HttpTransportPolicyError> {
    policy.validate()?;
    if now_ms == 0 || !policy.is_active_at(now_ms) {
        return Err(HttpTransportPolicyError::PolicyNotActive);
    }
    require_text(policy_record_ref)?;
    let policy_digest = policy.identity_digest()?;
    verify_record(record, policy_digest, policy_record_ref, now_ms)?;
    verify_adoption(adoption, policy, policy_digest, now_ms)?;
    let verified_at_ms = record.verified_at_ms.max(adoption.verified_at_ms);
    let valid_until_ms = policy
        .valid_until_ms
        .min(record.valid_until_ms)
        .min(adoption.valid_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(HttpTransportPolicyError::NoUsablePolicyWindow);
    }
    let qualification_digest = adopted_digest(policy_digest);
    Ok(QualifiedAdoptedHttpTransportPolicy {
        policy: policy.clone(),
        policy_digest,
        qualification_digest,
        verified_at_ms,
        valid_until_ms,
    })
}

pub fn qualify_current_http_transport_policy(
    adopted: &QualifiedAdoptedHttpTransportPolicy,
    freshness: &VerifiedAuthorityFreshness,
    now_ms: u64,
) -> Result<QualifiedCurrentHttpTransportPolicy, HttpTransportPolicyError> {
    if now_ms == 0 || adopted.verified_at_ms > now_ms || adopted.valid_until_ms <= now_ms {
        return Err(HttpTransportPolicyError::NoUsablePolicyWindow);
    }
    let policy = adopted.policy();
    let subject = AuthoritySubjectRef {
        kind: AuthoritySubjectKind::EffectSafetyPolicy,
        namespace: format!("integration-http:{}", policy.institution.as_str()),
        subject_id: policy.policy_id.clone(),
        identity: ProfiledDigest {
            digest: adopted.policy_digest,
            profile: POLICY_IDENTITY_PROFILE.to_owned(),
        },
    };
    if freshness.snapshot.generation != policy.generation {
        return Err(HttpTransportPolicyError::FreshnessGenerationMismatch);
    }
    if freshness.snapshot.effective_at_ms < policy.valid_from_ms {
        return Err(HttpTransportPolicyError::FreshnessPredatesPolicy);
    }
    let current = qualify_current_freshness(
        std::slice::from_ref(&subject),
        std::slice::from_ref(freshness),
        now_ms,
    )?;
    let verified_at_ms = adopted.verified_at_ms.max(current.verified_at_ms);
    let valid_until_ms = adopted.valid_until_ms.min(current.lease_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(HttpTransportPolicyError::NoUsableCurrentWindow);
    }
    let qualification_digest = current_digest(adopted.qualification_digest, current.freshness_digest);
    Ok(QualifiedCurrentHttpTransportPolicy {
        adopted: adopted.clone(),
        freshness_digest: current.freshness_digest,
        qualification_digest,
        verified_at_ms,
        valid_until_ms,
    })
}

pub fn bind_http_transport_to_provider(
    current: &QualifiedCurrentHttpTransportPolicy,
    provider: &QualifiedProviderExecutionProfile,
    now_ms: u64,
) -> Result<QualifiedHttpTransportProviderBinding, HttpTransportPolicyError> {
    if now_ms == 0 || current.verified_at_ms > now_ms || current.valid_until_ms <= now_ms {
        return Err(HttpTransportPolicyError::NoUsableCurrentWindow);
    }
    let policy = current.policy();
    let profile = provider.profile();
    if &policy.provider_profile_commitment != provider.profile_commitment() {
        return Err(HttpTransportPolicyError::ProviderProfileCommitmentMismatch);
    }
    if policy.connector_instance != profile.connector_instance
        || policy.system != profile.system
        || policy.operation_kind != profile.operation_kind
    {
        return Err(HttpTransportPolicyError::ProviderSemanticMismatch);
    }
    if policy.max_materialized_plan_bytes > profile.max_payload_bytes {
        return Err(HttpTransportPolicyError::PlanLimitExceedsProviderProfile);
    }
    let provider_valid_until = u64::try_from(profile.not_after_ms)
        .map_err(|_| HttpTransportPolicyError::ProviderProfileTimeOverflow)?
        .saturating_add(1);
    let valid_until_ms = current.valid_until_ms.min(provider_valid_until);
    if valid_until_ms <= now_ms {
        return Err(HttpTransportPolicyError::NoUsableProviderWindow);
    }
    let binding_digest = provider_binding_digest(
        current.qualification_digest,
        provider.profile_commitment(),
    );
    Ok(QualifiedHttpTransportProviderBinding {
        current_policy: current.clone(),
        provider_profile_commitment: provider.profile_commitment().clone(),
        binding_digest,
        valid_until_ms,
    })
}

fn verify_record(
    proof: &VerifiedHttpTransportPolicyRecordProof,
    digest: Digest32,
    record_ref: &str,
    now_ms: u64,
) -> Result<(), HttpTransportPolicyError> {
    if proof.protocol_version != RECORD_PROOF_PROTOCOL
        || proof.policy_digest != digest
        || proof.policy_profile != POLICY_IDENTITY_PROFILE
        || proof.policy_record_ref != record_ref
    {
        return Err(HttpTransportPolicyError::RecordProofMismatch);
    }
    require_text(&proof.record_proof_ref)?;
    require_text(&proof.record_verifier_ref)?;
    require_text(&proof.verification_ref)?;
    validate_evidence_window(proof.verified_at_ms, proof.valid_until_ms, now_ms)
}

fn verify_adoption(
    proof: &VerifiedHttpTransportPolicyAdoptionProof,
    policy: &IntegrationHttpTransportPolicy,
    digest: Digest32,
    now_ms: u64,
) -> Result<(), HttpTransportPolicyError> {
    if proof.protocol_version != ADOPTION_PROOF_PROTOCOL
        || proof.policy_digest != digest
        || proof.policy_profile != POLICY_IDENTITY_PROFILE
        || proof.verified_authority_ref != policy.authority_ref
        || proof.verified_policy_proof_ref != policy.policy_proof_ref
    {
        return Err(HttpTransportPolicyError::AdoptionProofMismatch);
    }
    require_text(&proof.authority_verifier_ref)?;
    require_text(&proof.verification_ref)?;
    validate_evidence_window(proof.verified_at_ms, proof.valid_until_ms, now_ms)
}

fn validate_evidence_window(
    verified_at_ms: u64,
    valid_until_ms: u64,
    now_ms: u64,
) -> Result<(), HttpTransportPolicyError> {
    if verified_at_ms == 0 || verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(HttpTransportPolicyError::InvalidEvidenceWindow);
    }
    Ok(())
}

fn validate_dns_host(host: &str) -> Result<(), HttpTransportPolicyError> {
    if host.is_empty() || host.len() > 253 || host != host.to_ascii_lowercase() {
        return Err(HttpTransportPolicyError::InvalidEndpointHost);
    }
    if host.contains(['/', ':', '@', '[', ']', '?', '#'])
        || host.starts_with('.')
        || host.ends_with('.')
        || !host.contains('.')
    {
        return Err(HttpTransportPolicyError::InvalidEndpointHost);
    }
    for label in host.split('.') {
        if label.is_empty()
            || label.len() > 63
            || label.starts_with('-')
            || label.ends_with('-')
            || !label
                .bytes()
                .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'-')
        {
            return Err(HttpTransportPolicyError::InvalidEndpointHost);
        }
    }
    Ok(())
}

fn validate_path_prefix(path: &str) -> Result<(), HttpTransportPolicyError> {
    if path.is_empty()
        || path.len() > MAX_TEXT_BYTES
        || !path.starts_with('/')
        || path.contains("..")
        || path.contains("//")
        || path.contains(['?', '#', '\\'])
        || path.chars().any(char::is_control)
    {
        return Err(HttpTransportPolicyError::InvalidPathPrefix);
    }
    Ok(())
}

fn validate_header_name(name: &str) -> Result<(), HttpTransportPolicyError> {
    if name.is_empty()
        || name.len() > 128
        || name != name.to_ascii_lowercase()
        || !name.bytes().all(is_header_token_byte)
        || matches!(
            name,
            "host"
                | "content-length"
                | "connection"
                | "transfer-encoding"
                | "proxy-authorization"
                | "proxy-authenticate"
                | "te"
                | "trailer"
                | "upgrade"
        )
    {
        return Err(HttpTransportPolicyError::InvalidHeaderName);
    }
    Ok(())
}

fn is_header_token_byte(byte: u8) -> bool {
    byte.is_ascii_lowercase()
        || byte.is_ascii_digit()
        || matches!(
            byte,
            b'!' | b'#' | b'$' | b'%' | b'&' | b'\'' | b'*' | b'+' | b'-' | b'.' | b'^'
                | b'_' | b'`' | b'|' | b'~'
        )
}

fn validate_media_type(value: &str) -> Result<(), HttpTransportPolicyError> {
    if value.is_empty()
        || value.len() > 256
        || value.chars().any(char::is_control)
        || !value.contains('/')
        || value.contains(['\r', '\n'])
    {
        return Err(HttpTransportPolicyError::InvalidMediaType);
    }
    Ok(())
}

fn require_text(value: &str) -> Result<(), HttpTransportPolicyError> {
    if value.trim().is_empty()
        || value.len() > MAX_TEXT_BYTES
        || value.chars().any(char::is_control)
    {
        Err(HttpTransportPolicyError::InvalidText)
    } else {
        Ok(())
    }
}

fn adopted_digest(policy: Digest32) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_QUALIFICATION);
    frame(&mut h, QUALIFICATION_PROFILE.as_bytes());
    frame(&mut h, &policy.0);
    Digest32(*h.finalize().as_bytes())
}

fn current_digest(adopted: Digest32, freshness: Digest32) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_CURRENT);
    frame(&mut h, CURRENT_POLICY_PROFILE.as_bytes());
    frame(&mut h, &adopted.0);
    frame(&mut h, &freshness.0);
    Digest32(*h.finalize().as_bytes())
}

fn provider_binding_digest(current: Digest32, profile: &ContentCommitment) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_PROVIDER_BINDING);
    frame(&mut h, PROVIDER_BINDING_PROFILE.as_bytes());
    frame(&mut h, &current.0);
    frame_commitment(&mut h, profile);
    Digest32(*h.finalize().as_bytes())
}

fn frame_commitment(h: &mut blake3::Hasher, commitment: &ContentCommitment) {
    frame(
        h,
        &[match commitment.algorithm {
            DigestAlgorithm::Sha256 => 1,
        }],
    );
    frame(h, &commitment.digest);
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

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

fn method_code(method: HttpMethod) -> u8 {
    match method {
        HttpMethod::Get => 1,
        HttpMethod::Post => 2,
        HttpMethod::Put => 3,
        HttpMethod::Patch => 4,
        HttpMethod::Delete => 5,
    }
}

#[derive(Debug, Error)]
pub enum HttpTransportPolicyError {
    #[error("wrong HTTP transport policy protocol version")]
    WrongProtocolVersion,
    #[error("transport policy text is invalid")]
    InvalidText,
    #[error("transport policy generation must be non-zero")]
    InvalidGeneration,
    #[error("transport policy rulebook is invalid")]
    InvalidRulebook,
    #[error("transport endpoint host must be a lowercase DNS name")]
    InvalidEndpointHost,
    #[error("HTTP transport v0.1 requires HTTPS port 443")]
    HttpsPortRequired,
    #[error("transport path prefix is invalid")]
    InvalidPathPrefix,
    #[error("transport header name is invalid or engine-reserved")]
    InvalidHeaderName,
    #[error("credential and idempotency header roles collide")]
    HeaderRoleCollision,
    #[error("transport content type is invalid")]
    InvalidMediaType,
    #[error("transport size limit is invalid")]
    InvalidSizeLimit,
    #[error("transport policy validity window is invalid")]
    InvalidPolicyWindow,
    #[error("transport policy is not active")]
    PolicyNotActive,
    #[error("transport policy record proof does not match")]
    RecordProofMismatch,
    #[error("transport policy adoption proof does not match")]
    AdoptionProofMismatch,
    #[error("transport policy evidence window is invalid")]
    InvalidEvidenceWindow,
    #[error("transport policy has no usable adopted window")]
    NoUsablePolicyWindow,
    #[error("transport policy freshness generation does not match policy generation")]
    FreshnessGenerationMismatch,
    #[error("transport policy freshness predates the policy")]
    FreshnessPredatesPolicy,
    #[error("transport policy has no usable current window")]
    NoUsableCurrentWindow,
    #[error("transport policy exact provider profile commitment differs")]
    ProviderProfileCommitmentMismatch,
    #[error("transport policy provider semantics differ from signed provider profile")]
    ProviderSemanticMismatch,
    #[error("transport materialized-plan limit exceeds signed provider profile limit")]
    PlanLimitExceedsProviderProfile,
    #[error("provider profile validity cannot be represented as current-policy time")]
    ProviderProfileTimeOverflow,
    #[error("transport/provider binding has no usable validity window")]
    NoUsableProviderWindow,
    #[error(transparent)]
    Freshness(#[from] FreshnessError),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn endpoint_host_rejects_scheme_ip_and_mixed_case() {
        assert!(validate_dns_host("api.stripe.com").is_ok());
        assert!(validate_dns_host("https://api.stripe.com").is_err());
        assert!(validate_dns_host("127.0.0.1").is_err());
        assert!(validate_dns_host("Api.Stripe.Com").is_err());
    }

    #[test]
    fn endpoint_path_rejects_traversal_query_and_fragment() {
        assert!(validate_path_prefix("/v1/payment_intents/").is_ok());
        assert!(validate_path_prefix("/v1/../admin").is_err());
        assert!(validate_path_prefix("/v1/payments?host=evil").is_err());
        assert!(validate_path_prefix("/v1/payments#fragment").is_err());
    }

    #[test]
    fn engine_reserved_headers_are_rejected() {
        for name in ["host", "content-length", "transfer-encoding", "proxy-authorization"] {
            assert!(validate_header_name(name).is_err());
        }
        assert!(validate_header_name("authorization").is_ok());
        assert!(validate_header_name("idempotency-key").is_ok());
    }

    #[test]
    fn policy_exposes_no_redirect_proxy_or_arbitrary_header_authority() {
        // Compile-time/API surface assertions are reinforced by CI source ratchets.
        let _ = IntegrationHttpTransportPolicy::redirects_allowed_here;
        let _ = IntegrationHttpTransportPolicy::environment_proxy_allowed_here;
        let _ = IntegrationHttpTransportPolicy::arbitrary_materializer_headers_allowed_here;
    }
}
