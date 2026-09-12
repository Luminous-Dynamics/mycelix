//! Canonical path/query/body-only HTTP request plans.
//!
//! The binary format intentionally has no fields for scheme, host, port, method,
//! headers, credentials, redirects, or proxies. Those remain engine-owned under
//! the current institution-adopted HTTP transport policy.

use mycelix_institutional_core::Digest32;
use mycelix_integration_core::{ContentCommitment, DigestAlgorithm};
use mycelix_integration_http_transport_policy::QualifiedHttpTransportProviderBinding;
use mycelix_integration_materializer_runtime::MaterializedProviderRequest;
use thiserror::Error;

pub const PLAN_PROFILE: &str = "mycelix-integration-http-request-plan-v1-binary";
pub const QUALIFICATION_PROFILE: &str =
    "mycelix-integration-http-request-plan-qualified-v1-blake3-framed";
const MAGIC: &[u8] = b"MYCELIX-HTTP-PLAN\0V1\0";
const DOMAIN_QUALIFICATION: &[u8] = b"mycelix/integration/http-request-plan/qualified/v1";
const MAX_PATH_SEGMENTS: usize = 64;
const MAX_PATH_SEGMENT_BYTES: usize = 1024;
const MAX_QUERY_PAIRS: usize = 128;
const MAX_QUERY_KEY_BYTES: usize = 1024;
const MAX_QUERY_VALUE_BYTES: usize = 4096;
const MAX_RENDERED_PATH_BYTES: usize = 8192;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HttpQueryPair {
    name: String,
    value: String,
}

impl HttpQueryPair {
    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn value(&self) -> &str {
        &self.value
    }
}

/// Non-deserializable proof that one exact materializer output is a canonical
/// path/query/body-only plan accepted by one exact current transport policy.
#[derive(Clone, Debug)]
pub struct QualifiedHttpRequestPlan {
    path_segments: Vec<String>,
    query: Vec<HttpQueryPair>,
    body: Vec<u8>,
    body_commitment: ContentCommitment,
    materialization_digest: Digest32,
    transport_policy_binding_digest: Digest32,
    qualification_digest: Digest32,
}

impl QualifiedHttpRequestPlan {
    pub fn path_segments(&self) -> &[String] {
        &self.path_segments
    }

    pub fn query(&self) -> &[HttpQueryPair] {
        &self.query
    }

    pub fn body_len(&self) -> usize {
        self.body.len()
    }

    pub fn body_commitment(&self) -> &ContentCommitment {
        &self.body_commitment
    }

    pub fn materialization_digest(&self) -> Digest32 {
        self.materialization_digest
    }

    pub fn transport_policy_binding_digest(&self) -> Digest32 {
        self.transport_policy_binding_digest
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &'static str {
        QUALIFICATION_PROFILE
    }

    pub const fn host_authority_present_in_plan_here(&self) -> bool {
        false
    }

    pub const fn header_authority_present_in_plan_here(&self) -> bool {
        false
    }

    pub const fn credential_material_present_in_plan_here(&self) -> bool {
        false
    }

    pub const fn raw_body_extractable_here(&self) -> bool {
        false
    }

    pub const fn network_io_performed_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub fn qualify_http_request_plan(
    materialized: &MaterializedProviderRequest,
    transport: &QualifiedHttpTransportProviderBinding,
    now_ms: u64,
) -> Result<QualifiedHttpRequestPlan, HttpRequestPlanError> {
    if now_ms == 0 || transport.valid_until_ms() <= now_ms {
        return Err(HttpRequestPlanError::TransportPolicyNotLive);
    }
    if materialized.provider_profile_commitment() != transport.provider_profile_commitment() {
        return Err(HttpRequestPlanError::ProviderProfileMismatch);
    }

    let policy = transport.current_policy().policy();
    let bytes = materialized.bytes();
    if bytes.len() > policy.max_materialized_plan_bytes as usize {
        return Err(HttpRequestPlanError::PlanTooLarge);
    }
    let parsed = parse_plan(bytes)?;
    if parsed.body.len() > policy.max_body_bytes as usize {
        return Err(HttpRequestPlanError::BodyTooLarge);
    }
    if rendered_query_len(&parsed.query)? > policy.max_query_bytes as usize {
        return Err(HttpRequestPlanError::QueryTooLarge);
    }
    if rendered_path_len(&policy.path_prefix, &parsed.path_segments)? > MAX_RENDERED_PATH_BYTES {
        return Err(HttpRequestPlanError::RenderedPathTooLarge);
    }

    let body_commitment = ContentCommitment::sha256(&parsed.body);
    let qualification_digest = qualification_digest(
        materialized.materialization_digest(),
        transport.binding_digest(),
        &parsed.path_segments,
        &parsed.query,
        &body_commitment,
    );

    Ok(QualifiedHttpRequestPlan {
        path_segments: parsed.path_segments,
        query: parsed.query,
        body: parsed.body,
        body_commitment,
        materialization_digest: materialized.materialization_digest(),
        transport_policy_binding_digest: transport.binding_digest(),
        qualification_digest,
    })
}

/// Reference encoder for materializer conformance tests. Inputs are encoded in
/// canonical order and must already satisfy the v0.1 semantic restrictions.
pub fn encode_request_plan_v1(
    path_segments: &[&str],
    query: &[(&str, &str)],
    body: &[u8],
) -> Result<Vec<u8>, HttpRequestPlanError> {
    if path_segments.len() > MAX_PATH_SEGMENTS || query.len() > MAX_QUERY_PAIRS {
        return Err(HttpRequestPlanError::TooManyItems);
    }
    let mut normalized_query = Vec::with_capacity(query.len());
    for (name, value) in query {
        validate_query_part(name, MAX_QUERY_KEY_BYTES)?;
        validate_query_part(value, MAX_QUERY_VALUE_BYTES)?;
        normalized_query.push(((*name).to_owned(), (*value).to_owned()));
    }
    if !strictly_sorted_pairs(&normalized_query) {
        return Err(HttpRequestPlanError::QueryNotCanonical);
    }

    let mut out = MAGIC.to_vec();
    push_u16(&mut out, path_segments.len())?;
    for segment in path_segments {
        validate_path_segment(segment)?;
        push_short_bytes(&mut out, segment.as_bytes())?;
    }
    push_u16(&mut out, query.len())?;
    for (name, value) in query {
        push_short_bytes(&mut out, name.as_bytes())?;
        push_short_bytes(&mut out, value.as_bytes())?;
    }
    let body_len = u32::try_from(body.len()).map_err(|_| HttpRequestPlanError::BodyTooLarge)?;
    out.extend_from_slice(&body_len.to_be_bytes());
    out.extend_from_slice(body);
    Ok(out)
}

#[derive(Debug)]
struct ParsedPlan {
    path_segments: Vec<String>,
    query: Vec<HttpQueryPair>,
    body: Vec<u8>,
}

fn parse_plan(bytes: &[u8]) -> Result<ParsedPlan, HttpRequestPlanError> {
    let mut cursor = Cursor::new(bytes);
    if cursor.take(MAGIC.len())? != MAGIC {
        return Err(HttpRequestPlanError::WrongMagic);
    }

    let path_count = usize::from(cursor.read_u16()?);
    if path_count > MAX_PATH_SEGMENTS {
        return Err(HttpRequestPlanError::TooManyItems);
    }
    let mut path_segments = Vec::with_capacity(path_count);
    for _ in 0..path_count {
        let bytes = cursor.read_short_bytes()?;
        let segment = std::str::from_utf8(bytes).map_err(|_| HttpRequestPlanError::InvalidUtf8)?;
        validate_path_segment(segment)?;
        path_segments.push(segment.to_owned());
    }

    let query_count = usize::from(cursor.read_u16()?);
    if query_count > MAX_QUERY_PAIRS {
        return Err(HttpRequestPlanError::TooManyItems);
    }
    let mut query = Vec::with_capacity(query_count);
    let mut raw_pairs = Vec::with_capacity(query_count);
    for _ in 0..query_count {
        let name_bytes = cursor.read_short_bytes()?;
        let value_bytes = cursor.read_short_bytes()?;
        let name = std::str::from_utf8(name_bytes).map_err(|_| HttpRequestPlanError::InvalidUtf8)?;
        let value =
            std::str::from_utf8(value_bytes).map_err(|_| HttpRequestPlanError::InvalidUtf8)?;
        validate_query_part(name, MAX_QUERY_KEY_BYTES)?;
        validate_query_part(value, MAX_QUERY_VALUE_BYTES)?;
        raw_pairs.push((name.to_owned(), value.to_owned()));
        query.push(HttpQueryPair {
            name: name.to_owned(),
            value: value.to_owned(),
        });
    }
    if !strictly_sorted_pairs(&raw_pairs) {
        return Err(HttpRequestPlanError::QueryNotCanonical);
    }

    let body_len = usize::try_from(cursor.read_u32()?).map_err(|_| HttpRequestPlanError::BodyTooLarge)?;
    let body = cursor.take(body_len)?.to_vec();
    if !cursor.is_done() {
        return Err(HttpRequestPlanError::TrailingBytes);
    }

    Ok(ParsedPlan {
        path_segments,
        query,
        body,
    })
}

fn validate_path_segment(segment: &str) -> Result<(), HttpRequestPlanError> {
    if segment.is_empty()
        || segment.len() > MAX_PATH_SEGMENT_BYTES
        || matches!(segment, "." | "..")
        || segment.contains(['/', '\\', '?', '#'])
        || segment.chars().any(char::is_control)
    {
        return Err(HttpRequestPlanError::InvalidPathSegment);
    }
    Ok(())
}

fn validate_query_part(value: &str, max: usize) -> Result<(), HttpRequestPlanError> {
    if value.len() > max || value.chars().any(char::is_control) {
        return Err(HttpRequestPlanError::InvalidQueryPart);
    }
    Ok(())
}

fn strictly_sorted_pairs(pairs: &[(String, String)]) -> bool {
    pairs.windows(2).all(|pair| pair[0] < pair[1])
}

fn rendered_path_len(prefix: &str, segments: &[String]) -> Result<usize, HttpRequestPlanError> {
    let mut len = prefix.len();
    if !segments.is_empty() && !prefix.ends_with('/') {
        len = len.checked_add(1).ok_or(HttpRequestPlanError::RenderedPathTooLarge)?;
    }
    for (index, segment) in segments.iter().enumerate() {
        if index > 0 {
            len = len.checked_add(1).ok_or(HttpRequestPlanError::RenderedPathTooLarge)?;
        }
        len = len
            .checked_add(percent_encoded_len(segment.as_bytes())?)
            .ok_or(HttpRequestPlanError::RenderedPathTooLarge)?;
    }
    Ok(len)
}

fn rendered_query_len(query: &[HttpQueryPair]) -> Result<usize, HttpRequestPlanError> {
    let mut len = 0usize;
    for (index, pair) in query.iter().enumerate() {
        if index > 0 {
            len = len.checked_add(1).ok_or(HttpRequestPlanError::QueryTooLarge)?;
        }
        len = len
            .checked_add(percent_encoded_len(pair.name.as_bytes())?)
            .and_then(|value| value.checked_add(1))
            .and_then(|value| value.checked_add(percent_encoded_len(pair.value.as_bytes()).ok()?))
            .ok_or(HttpRequestPlanError::QueryTooLarge)?;
    }
    Ok(len)
}

fn percent_encoded_len(bytes: &[u8]) -> Result<usize, HttpRequestPlanError> {
    let mut len = 0usize;
    for byte in bytes {
        let add = if is_unreserved(*byte) { 1 } else { 3 };
        len = len.checked_add(add).ok_or(HttpRequestPlanError::EncodingOverflow)?;
    }
    Ok(len)
}

fn is_unreserved(byte: u8) -> bool {
    byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'.' | b'_' | b'~')
}

fn qualification_digest(
    materialization: Digest32,
    transport_binding: Digest32,
    path_segments: &[String],
    query: &[HttpQueryPair],
    body: &ContentCommitment,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_QUALIFICATION);
    frame(&mut h, QUALIFICATION_PROFILE.as_bytes());
    frame(&mut h, PLAN_PROFILE.as_bytes());
    frame(&mut h, &materialization.0);
    frame(&mut h, &transport_binding.0);
    for segment in path_segments {
        frame(&mut h, segment.as_bytes());
    }
    for pair in query {
        frame(&mut h, pair.name.as_bytes());
        frame(&mut h, pair.value.as_bytes());
    }
    frame_commitment(&mut h, body);
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

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

fn push_u16(out: &mut Vec<u8>, value: usize) -> Result<(), HttpRequestPlanError> {
    let value = u16::try_from(value).map_err(|_| HttpRequestPlanError::TooManyItems)?;
    out.extend_from_slice(&value.to_be_bytes());
    Ok(())
}

fn push_short_bytes(out: &mut Vec<u8>, bytes: &[u8]) -> Result<(), HttpRequestPlanError> {
    let len = u16::try_from(bytes.len()).map_err(|_| HttpRequestPlanError::FieldTooLarge)?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

struct Cursor<'a> {
    bytes: &'a [u8],
    offset: usize,
}

impl<'a> Cursor<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, offset: 0 }
    }

    fn take(&mut self, len: usize) -> Result<&'a [u8], HttpRequestPlanError> {
        let end = self
            .offset
            .checked_add(len)
            .ok_or(HttpRequestPlanError::Truncated)?;
        let slice = self
            .bytes
            .get(self.offset..end)
            .ok_or(HttpRequestPlanError::Truncated)?;
        self.offset = end;
        Ok(slice)
    }

    fn read_u16(&mut self) -> Result<u16, HttpRequestPlanError> {
        let bytes: [u8; 2] = self
            .take(2)?
            .try_into()
            .map_err(|_| HttpRequestPlanError::Truncated)?;
        Ok(u16::from_be_bytes(bytes))
    }

    fn read_u32(&mut self) -> Result<u32, HttpRequestPlanError> {
        let bytes: [u8; 4] = self
            .take(4)?
            .try_into()
            .map_err(|_| HttpRequestPlanError::Truncated)?;
        Ok(u32::from_be_bytes(bytes))
    }

    fn read_short_bytes(&mut self) -> Result<&'a [u8], HttpRequestPlanError> {
        let len = usize::from(self.read_u16()?);
        self.take(len)
    }

    fn is_done(&self) -> bool {
        self.offset == self.bytes.len()
    }
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum HttpRequestPlanError {
    #[error("current HTTP transport policy is not live")]
    TransportPolicyNotLive,
    #[error("materialization provider profile differs from HTTP transport policy binding")]
    ProviderProfileMismatch,
    #[error("materialized plan exceeds transport policy bound")]
    PlanTooLarge,
    #[error("request body exceeds transport policy bound")]
    BodyTooLarge,
    #[error("rendered query exceeds transport policy bound")]
    QueryTooLarge,
    #[error("rendered path exceeds v0.1 bound")]
    RenderedPathTooLarge,
    #[error("wrong request-plan magic/version")]
    WrongMagic,
    #[error("request plan is truncated")]
    Truncated,
    #[error("request plan contains trailing bytes")]
    TrailingBytes,
    #[error("request plan contains invalid UTF-8")]
    InvalidUtf8,
    #[error("request plan has too many path/query items")]
    TooManyItems,
    #[error("request plan field is too large")]
    FieldTooLarge,
    #[error("request-plan path segment is invalid")]
    InvalidPathSegment,
    #[error("request-plan query component is invalid")]
    InvalidQueryPart,
    #[error("request-plan query pairs are not strictly canonical-sorted")]
    QueryNotCanonical,
    #[error("percent-encoded size arithmetic overflow")]
    EncodingOverflow,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canonical_plan_round_trips() {
        let bytes = encode_request_plan_v1(
            &["pi_123", "confirm"],
            &[("expand", "charges"), ("include", "customer")],
            b"body",
        )
        .unwrap();
        let parsed = parse_plan(&bytes).unwrap();
        assert_eq!(parsed.path_segments, ["pi_123", "confirm"]);
        assert_eq!(parsed.query.len(), 2);
        assert_eq!(parsed.body, b"body");
    }

    #[test]
    fn traversal_and_slash_segments_fail_closed() {
        assert_eq!(
            validate_path_segment(".."),
            Err(HttpRequestPlanError::InvalidPathSegment)
        );
        assert_eq!(
            validate_path_segment("a/b"),
            Err(HttpRequestPlanError::InvalidPathSegment)
        );
    }

    #[test]
    fn unsorted_query_is_noncanonical() {
        assert_eq!(
            encode_request_plan_v1(&[], &[("z", "1"), ("a", "1")], b""),
            Err(HttpRequestPlanError::QueryNotCanonical)
        );
    }

    #[test]
    fn duplicate_query_pair_is_noncanonical() {
        assert_eq!(
            encode_request_plan_v1(&[], &[("a", "1"), ("a", "1")], b""),
            Err(HttpRequestPlanError::QueryNotCanonical)
        );
    }

    #[test]
    fn no_host_header_or_credential_fields_exist_in_plan_api() {
        let _ = QualifiedHttpRequestPlan::host_authority_present_in_plan_here;
        let _ = QualifiedHttpRequestPlan::header_authority_present_in_plan_here;
        let _ = QualifiedHttpRequestPlan::credential_material_present_in_plan_here;
        let _ = QualifiedHttpRequestPlan::raw_body_extractable_here;
    }
}
