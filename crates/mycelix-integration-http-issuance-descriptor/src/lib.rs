//! Secret-free exact HTTP issuance descriptors.
//!
//! This crate derives an audit identity for the exact request shape that a native
//! transport may later issue, without containing or hashing credential material.
//! It binds current transport policy, canonical path/query/body plan, exact public
//! route, exact credential *version metadata*, the full canonical command identity
//! (which already contains idempotency semantics), and all relevant horizons.

use mycelix_institutional_core::Digest32;
use mycelix_integration_core::{
    CanonicalEncodeV1, ContentCommitment, DigestAlgorithm, IntegrationCommand,
};
use mycelix_integration_http_credential_slot::QualifiedHttpCredentialSlot;
use mycelix_integration_http_egress_route::QualifiedPublicHttpsRoute;
use mycelix_integration_http_request_plan::QualifiedHttpRequestPlan;
use mycelix_integration_http_transport_policy::{
    HttpMethod, QualifiedHttpTransportProviderBinding,
};
use thiserror::Error;

pub const ISSUANCE_DESCRIPTOR_PROFILE: &str =
    "mycelix-integration-http-issuance-descriptor-v1-blake3-framed";
const DOMAIN_DESCRIPTOR: &[u8] = b"mycelix/integration/http-issuance-descriptor/v1";
const MAX_RENDERED_PATH_BYTES: usize = 8192;
const MAX_RENDERED_QUERY_BYTES: usize = 64 * 1024;

/// Non-deserializable, secret-free identity of one exact candidate HTTP
/// issuance. Raw URL/body/idempotency values are deliberately not exposed.
#[derive(Clone, Debug)]
pub struct QualifiedHttpIssuanceDescriptor {
    issuance_digest: Digest32,
    command_commitment: ContentCommitment,
    transport_policy_binding_digest: Digest32,
    request_plan_digest: Digest32,
    route_digest: Digest32,
    credential_slot_digest: Digest32,
    rendered_path_commitment: ContentCommitment,
    rendered_query_commitment: ContentCommitment,
    body_commitment: ContentCommitment,
    rendered_path_len: u32,
    rendered_query_len: u32,
    body_len: u64,
    valid_until_ms: u64,
}

impl QualifiedHttpIssuanceDescriptor {
    pub fn issuance_digest(&self) -> Digest32 {
        self.issuance_digest
    }

    pub fn command_commitment(&self) -> &ContentCommitment {
        &self.command_commitment
    }

    pub fn transport_policy_binding_digest(&self) -> Digest32 {
        self.transport_policy_binding_digest
    }

    pub fn request_plan_digest(&self) -> Digest32 {
        self.request_plan_digest
    }

    pub fn route_digest(&self) -> Digest32 {
        self.route_digest
    }

    pub fn credential_slot_digest(&self) -> Digest32 {
        self.credential_slot_digest
    }

    pub fn rendered_path_commitment(&self) -> &ContentCommitment {
        &self.rendered_path_commitment
    }

    pub fn rendered_query_commitment(&self) -> &ContentCommitment {
        &self.rendered_query_commitment
    }

    pub fn body_commitment(&self) -> &ContentCommitment {
        &self.body_commitment
    }

    pub fn rendered_path_len(&self) -> u32 {
        self.rendered_path_len
    }

    pub fn rendered_query_len(&self) -> u32 {
        self.rendered_query_len
    }

    pub fn body_len(&self) -> u64 {
        self.body_len
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub fn is_live_at(&self, now_ms: u64) -> bool {
        now_ms < self.valid_until_ms
    }

    pub const fn idempotency_identity_bound_via_command_commitment_here(&self) -> bool {
        true
    }

    pub const fn credential_material_present_here(&self) -> bool {
        false
    }

    pub const fn credential_material_hashed_here(&self) -> bool {
        false
    }

    pub const fn standalone_idempotency_key_hashed_here(&self) -> bool {
        false
    }

    pub const fn raw_url_extractable_here(&self) -> bool {
        false
    }

    pub const fn raw_body_extractable_here(&self) -> bool {
        false
    }

    pub const fn provider_io_performed_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub fn qualify_http_issuance_descriptor<C>(
    command: &IntegrationCommand<C>,
    transport: &QualifiedHttpTransportProviderBinding,
    plan: &QualifiedHttpRequestPlan,
    route: &QualifiedPublicHttpsRoute,
    credential: &QualifiedHttpCredentialSlot,
    now_ms: u64,
) -> Result<QualifiedHttpIssuanceDescriptor, IssuanceDescriptorError>
where
    C: CanonicalEncodeV1,
{
    if now_ms == 0
        || transport.valid_until_ms() <= now_ms
        || !route.is_live_at(now_ms)
        || !credential.is_live_at(now_ms)
    {
        return Err(IssuanceDescriptorError::InputNotLive);
    }

    let transport_digest = transport.binding_digest();
    if plan.transport_policy_binding_digest() != transport_digest
        || route.transport_policy_binding_digest() != transport_digest
        || credential.transport_policy_binding_digest() != transport_digest
    {
        return Err(IssuanceDescriptorError::TransportBindingMismatch);
    }

    let policy = transport.current_policy().policy();
    if command.connector_instance != policy.connector_instance
        || command.system != policy.system
        || command.operation_kind != policy.operation_kind
    {
        return Err(IssuanceDescriptorError::CommandScopeMismatch);
    }
    if route.hostname() != policy.endpoint_host {
        return Err(IssuanceDescriptorError::RouteHostnameMismatch);
    }
    if credential.slot_id() != policy.credential_slot_id {
        return Err(IssuanceDescriptorError::CredentialSlotMismatch);
    }
    validate_idempotency_requirement(
        policy.idempotency_header_name.is_some(),
        command.idempotency_key.is_some(),
    )?;

    let rendered_path = render_path(&policy.path_prefix, plan.path_segments())?;
    let rendered_query = render_query(plan)?;
    if rendered_path.len() > MAX_RENDERED_PATH_BYTES
        || rendered_query.len() > MAX_RENDERED_QUERY_BYTES
    {
        return Err(IssuanceDescriptorError::RenderedRequestTooLarge);
    }
    let rendered_path_len = u32::try_from(rendered_path.len())
        .map_err(|_| IssuanceDescriptorError::RenderedRequestTooLarge)?;
    let rendered_query_len = u32::try_from(rendered_query.len())
        .map_err(|_| IssuanceDescriptorError::RenderedRequestTooLarge)?;
    let body_len = u64::try_from(plan.body_len())
        .map_err(|_| IssuanceDescriptorError::BodyLengthOverflow)?;

    let command_commitment = command.canonical_commitment_v1();
    let rendered_path_commitment = ContentCommitment::sha256(&rendered_path);
    let rendered_query_commitment = ContentCommitment::sha256(&rendered_query);
    let body_commitment = plan.body_commitment().clone();

    let valid_until_ms = transport
        .valid_until_ms()
        .min(route.valid_until_ms())
        .min(credential.valid_until_ms());
    if valid_until_ms <= now_ms {
        return Err(IssuanceDescriptorError::InputNotLive);
    }

    let issuance_digest = issuance_digest(
        &command_commitment,
        transport_digest,
        plan.qualification_digest(),
        route.route_digest(),
        credential.qualification_digest(),
        policy.method,
        &policy.endpoint_host,
        policy.endpoint_port,
        &policy.content_type,
        &policy.credential_header_name,
        policy.idempotency_header_name.as_deref(),
        &rendered_path_commitment,
        &rendered_query_commitment,
        &body_commitment,
        rendered_path_len,
        rendered_query_len,
        body_len,
        valid_until_ms,
    );

    Ok(QualifiedHttpIssuanceDescriptor {
        issuance_digest,
        command_commitment,
        transport_policy_binding_digest: transport_digest,
        request_plan_digest: plan.qualification_digest(),
        route_digest: route.route_digest(),
        credential_slot_digest: credential.qualification_digest(),
        rendered_path_commitment,
        rendered_query_commitment,
        body_commitment,
        rendered_path_len,
        rendered_query_len,
        body_len,
        valid_until_ms,
    })
}

fn validate_idempotency_requirement(
    header_required: bool,
    key_present: bool,
) -> Result<(), IssuanceDescriptorError> {
    if header_required && !key_present {
        Err(IssuanceDescriptorError::MissingIdempotencyKey)
    } else {
        Ok(())
    }
}

fn render_path(prefix: &str, segments: &[String]) -> Result<Vec<u8>, IssuanceDescriptorError> {
    let mut out = prefix.as_bytes().to_vec();
    if !segments.is_empty() && !prefix.ends_with('/') {
        out.push(b'/');
    }
    for (index, segment) in segments.iter().enumerate() {
        if index > 0 {
            out.push(b'/');
        }
        percent_encode_into(segment.as_bytes(), &mut out)?;
    }
    Ok(out)
}

fn render_query(plan: &QualifiedHttpRequestPlan) -> Result<Vec<u8>, IssuanceDescriptorError> {
    let mut out = Vec::new();
    for (index, pair) in plan.query().iter().enumerate() {
        if index > 0 {
            out.push(b'&');
        }
        percent_encode_into(pair.name().as_bytes(), &mut out)?;
        out.push(b'=');
        percent_encode_into(pair.value().as_bytes(), &mut out)?;
    }
    Ok(out)
}

fn percent_encode_into(bytes: &[u8], out: &mut Vec<u8>) -> Result<(), IssuanceDescriptorError> {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    for byte in bytes {
        if is_unreserved(*byte) {
            out.push(*byte);
        } else {
            out.try_reserve(3)
                .map_err(|_| IssuanceDescriptorError::EncodingAllocation)?;
            out.push(b'%');
            out.push(HEX[(byte >> 4) as usize]);
            out.push(HEX[(byte & 0x0f) as usize]);
        }
    }
    Ok(())
}

fn is_unreserved(byte: u8) -> bool {
    byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'.' | b'_' | b'~')
}

#[allow(clippy::too_many_arguments)]
fn issuance_digest(
    command: &ContentCommitment,
    transport: Digest32,
    plan: Digest32,
    route: Digest32,
    credential: Digest32,
    method: HttpMethod,
    hostname: &str,
    port: u16,
    content_type: &str,
    credential_header: &str,
    idempotency_header: Option<&str>,
    path: &ContentCommitment,
    query: &ContentCommitment,
    body: &ContentCommitment,
    path_len: u32,
    query_len: u32,
    body_len: u64,
    valid_until_ms: u64,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_DESCRIPTOR);
    frame(&mut h, ISSUANCE_DESCRIPTOR_PROFILE.as_bytes());
    frame_commitment(&mut h, command);
    frame(&mut h, &transport.0);
    frame(&mut h, &plan.0);
    frame(&mut h, &route.0);
    frame(&mut h, &credential.0);
    frame(&mut h, &[method_code(method)]);
    frame(&mut h, hostname.as_bytes());
    frame(&mut h, &port.to_be_bytes());
    frame(&mut h, content_type.as_bytes());
    frame(&mut h, credential_header.as_bytes());
    frame_optional_text(&mut h, idempotency_header);
    frame_commitment(&mut h, path);
    frame_commitment(&mut h, query);
    frame_commitment(&mut h, body);
    frame(&mut h, &path_len.to_le_bytes());
    frame(&mut h, &query_len.to_le_bytes());
    frame(&mut h, &body_len.to_le_bytes());
    frame(&mut h, &valid_until_ms.to_le_bytes());
    Digest32(*h.finalize().as_bytes())
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

fn frame_optional_text(h: &mut blake3::Hasher, value: Option<&str>) {
    match value {
        Some(value) => {
            frame(h, &[1]);
            frame(h, value.as_bytes());
        }
        None => frame(h, &[0]),
    }
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

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum IssuanceDescriptorError {
    #[error("one or more issuance inputs are not live")]
    InputNotLive,
    #[error("plan/route/credential do not share the exact transport-policy binding")]
    TransportBindingMismatch,
    #[error("command scope differs from HTTP transport policy")]
    CommandScopeMismatch,
    #[error("qualified route hostname differs from HTTP transport policy")]
    RouteHostnameMismatch,
    #[error("qualified credential slot differs from HTTP transport policy")]
    CredentialSlotMismatch,
    #[error("HTTP policy requires an idempotency header but command has no key")]
    MissingIdempotencyKey,
    #[error("rendered URL shape exceeds issuance descriptor bounds")]
    RenderedRequestTooLarge,
    #[error("body length cannot be represented")]
    BodyLengthOverflow,
    #[error("percent-encoding allocation failed")]
    EncodingAllocation,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percent_encoding_is_uppercase_and_unambiguous() {
        let mut out = Vec::new();
        percent_encode_into(b"a b/%", &mut out).unwrap();
        assert_eq!(out, b"a%20b%2F%25");
    }

    #[test]
    fn path_rendering_joins_segments_without_raw_slashes() {
        let rendered = render_path("/v1/items", &["a b".to_owned(), "c".to_owned()]).unwrap();
        assert_eq!(rendered, b"/v1/items/a%20b/c");
    }

    #[test]
    fn missing_required_idempotency_key_fails_closed() {
        assert_eq!(
            validate_idempotency_requirement(true, false),
            Err(IssuanceDescriptorError::MissingIdempotencyKey)
        );
        assert!(validate_idempotency_requirement(true, true).is_ok());
        assert!(validate_idempotency_requirement(false, false).is_ok());
    }
}
