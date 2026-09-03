use std::{
    net::{IpAddr, Ipv4Addr, SocketAddr},
    sync::Arc,
};

use axum::{
    body::Body,
    extract::{Path, State},
    http::{
        header::{
            ACCEPT_RANGES, CACHE_CONTROL, CONTENT_LENGTH, CONTENT_RANGE, CONTENT_TYPE, ETAG, RANGE,
        },
        HeaderMap, HeaderName, HeaderValue, StatusCode,
    },
    response::{IntoResponse, Response},
    routing::get,
    Json, Router,
};
use mycelix_content_core::{ContentDigestV1, DigestAlgorithmV1};
use mycelix_content_node::{CapacityV1, CasErrorV1, LocalCasV1, VerifiedBlobV1};
use serde::Serialize;
use tokio::{
    io::{AsyncReadExt, AsyncSeekExt},
    net::TcpListener,
    sync::Semaphore,
};
use tokio_util::io::ReaderStream;

use crate::{
    error::HttpFacadeErrorV1,
    range::{parse_single_range_v1, ByteRangeV1},
};

const X_CONTENT_TYPE_OPTIONS: HeaderName = HeaderName::from_static("x-content-type-options");

#[derive(Debug, Clone)]
pub struct HttpConfigV1 {
    pub bind_addr: SocketAddr,
    pub max_concurrent_verifications: usize,
}

impl HttpConfigV1 {
    pub fn loopback(port: u16) -> Self {
        Self {
            bind_addr: SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), port),
            max_concurrent_verifications: 4,
        }
    }

    pub fn validate(&self) -> Result<(), HttpFacadeErrorV1> {
        if !self.bind_addr.ip().is_loopback() {
            return Err(HttpFacadeErrorV1::NonLoopbackBind(self.bind_addr));
        }
        if self.max_concurrent_verifications == 0 {
            return Err(HttpFacadeErrorV1::ZeroVerificationConcurrency);
        }
        Ok(())
    }
}

#[derive(Clone)]
struct AppStateV1 {
    cas: Arc<LocalCasV1>,
    verification_slots: Arc<Semaphore>,
}

#[derive(Debug)]
enum ApiErrorV1 {
    BadDigest,
    NotFound,
    RangeNotSatisfiable { size_bytes: u64 },
    IntegrityFailure,
    StorageUnavailable,
    VerificationTaskFailed,
}

impl IntoResponse for ApiErrorV1 {
    fn into_response(self) -> Response {
        match self {
            Self::BadDigest => (StatusCode::BAD_REQUEST, "invalid content digest").into_response(),
            Self::NotFound => StatusCode::NOT_FOUND.into_response(),
            Self::RangeNotSatisfiable { size_bytes } => {
                let mut response = StatusCode::RANGE_NOT_SATISFIABLE.into_response();
                response.headers_mut().insert(
                    CONTENT_RANGE,
                    HeaderValue::from_str(&format!("bytes */{size_bytes}"))
                        .expect("u64 content-range is always a valid header"),
                );
                response
            }
            Self::IntegrityFailure => (
                StatusCode::INTERNAL_SERVER_ERROR,
                "local content integrity failure",
            )
                .into_response(),
            Self::StorageUnavailable => (
                StatusCode::SERVICE_UNAVAILABLE,
                "local content storage unavailable",
            )
                .into_response(),
            Self::VerificationTaskFailed => StatusCode::INTERNAL_SERVER_ERROR.into_response(),
        }
    }
}

#[derive(Serialize)]
struct HealthResponseV1 {
    status: &'static str,
    integrity: &'static str,
    quota_bytes: u64,
    used_bytes: u64,
    reserved_bytes: u64,
    available_bytes: u64,
}

pub async fn serve(
    cas: Arc<LocalCasV1>,
    config: HttpConfigV1,
) -> Result<(), HttpFacadeErrorV1> {
    config.validate()?;
    let listener = TcpListener::bind(config.bind_addr)
        .await
        .map_err(|source| HttpFacadeErrorV1::Bind {
            addr: config.bind_addr,
            source,
        })?;
    let app = build_router(cas, config.max_concurrent_verifications);
    axum::serve(listener, app)
        .await
        .map_err(HttpFacadeErrorV1::Serve)
}

fn build_router(cas: Arc<LocalCasV1>, max_concurrent_verifications: usize) -> Router {
    let state = AppStateV1 {
        cas,
        verification_slots: Arc::new(Semaphore::new(max_concurrent_verifications)),
    };
    Router::new()
        .route("/v1/health", get(health))
        .route("/v1/capacity", get(capacity))
        .route(
            "/v1/blobs/{algorithm}/{digest}",
            get(get_blob).head(head_blob),
        )
        .with_state(state)
}

async fn health(State(state): State<AppStateV1>) -> Json<HealthResponseV1> {
    Json(health_from_capacity(state.cas.capacity()))
}

async fn capacity(State(state): State<AppStateV1>) -> Json<HealthResponseV1> {
    Json(health_from_capacity(state.cas.capacity()))
}

fn health_from_capacity(capacity: CapacityV1) -> HealthResponseV1 {
    HealthResponseV1 {
        status: "ready",
        integrity: "verified-on-read",
        quota_bytes: capacity.quota_bytes,
        used_bytes: capacity.used_bytes,
        reserved_bytes: capacity.reserved_bytes,
        available_bytes: capacity.available_bytes,
    }
}

async fn head_blob(
    State(state): State<AppStateV1>,
    Path((algorithm, digest)): Path<(String, String)>,
) -> Result<Response, ApiErrorV1> {
    let digest = parse_digest(&algorithm, &digest)?;
    let verified = verify_digest(&state, digest).await?;
    Ok(full_response_headers(
        digest,
        verified.size_bytes(),
        Body::empty(),
    ))
}

async fn get_blob(
    State(state): State<AppStateV1>,
    Path((algorithm, digest)): Path<(String, String)>,
    headers: HeaderMap,
) -> Result<Response, ApiErrorV1> {
    let digest = parse_digest(&algorithm, &digest)?;
    let verified = verify_digest(&state, digest).await?;
    let size_bytes = verified.size_bytes();

    let range = match headers.get(RANGE) {
        None => None,
        Some(value) => {
            let value = value
                .to_str()
                .map_err(|_| ApiErrorV1::RangeNotSatisfiable { size_bytes })?;
            Some(
                parse_single_range_v1(value, size_bytes)
                    .map_err(|_| ApiErrorV1::RangeNotSatisfiable { size_bytes })?,
            )
        }
    };

    stream_verified_blob(digest, verified, range).await
}

async fn verify_digest(
    state: &AppStateV1,
    digest: ContentDigestV1,
) -> Result<VerifiedBlobV1, ApiErrorV1> {
    let _permit = state
        .verification_slots
        .clone()
        .acquire_owned()
        .await
        .map_err(|_| ApiErrorV1::VerificationTaskFailed)?;
    let cas = state.cas.clone();
    tokio::task::spawn_blocking(move || cas.open_verified_digest(digest))
        .await
        .map_err(|_| ApiErrorV1::VerificationTaskFailed)?
        .map_err(map_cas_error)
}

fn map_cas_error(error: CasErrorV1) -> ApiErrorV1 {
    match error {
        CasErrorV1::NotFound(_) => ApiErrorV1::NotFound,
        CasErrorV1::DigestMismatch { .. }
        | CasErrorV1::SizeMismatch { .. }
        | CasErrorV1::MutableBlobFile(_)
        | CasErrorV1::UnexpectedEntry(_)
        | CasErrorV1::InvalidBlobFilename(_)
        | CasErrorV1::InvalidDirectory(_) => ApiErrorV1::IntegrityFailure,
        CasErrorV1::Io { .. }
        | CasErrorV1::AlreadyLocked(_)
        | CasErrorV1::QuotaExceeded { .. }
        | CasErrorV1::UsageOverflow
        | CasErrorV1::InvalidDescriptor(_) => ApiErrorV1::StorageUnavailable,
    }
}

async fn stream_verified_blob(
    digest: ContentDigestV1,
    verified: VerifiedBlobV1,
    range: Option<ByteRangeV1>,
) -> Result<Response, ApiErrorV1> {
    let size_bytes = verified.size_bytes();
    let mut file = tokio::fs::File::from_std(verified.into_file());

    match range {
        None => {
            let body = Body::from_stream(ReaderStream::new(file));
            Ok(full_response_headers(digest, size_bytes, body))
        }
        Some(range) => {
            file.seek(std::io::SeekFrom::Start(range.start))
                .await
                .map_err(|_| ApiErrorV1::StorageUnavailable)?;
            let length = range.len();
            let body = Body::from_stream(ReaderStream::new(file.take(length)));
            let mut response = full_response_headers(digest, length, body);
            *response.status_mut() = StatusCode::PARTIAL_CONTENT;
            response.headers_mut().insert(
                CONTENT_RANGE,
                HeaderValue::from_str(&format!(
                    "bytes {}-{}/{size_bytes}",
                    range.start, range.end_inclusive
                ))
                .expect("validated byte ranges always form valid headers"),
            );
            Ok(response)
        }
    }
}

fn full_response_headers(digest: ContentDigestV1, content_length: u64, body: Body) -> Response {
    let mut response = Response::new(body);
    let headers = response.headers_mut();
    headers.insert(ACCEPT_RANGES, HeaderValue::from_static("bytes"));
    headers.insert(
        CACHE_CONTROL,
        HeaderValue::from_static("public, max-age=31536000, immutable"),
    );
    headers.insert(
        CONTENT_TYPE,
        HeaderValue::from_static("application/octet-stream"),
    );
    headers.insert(X_CONTENT_TYPE_OPTIONS, HeaderValue::from_static("nosniff"));
    headers.insert(
        CONTENT_LENGTH,
        HeaderValue::from_str(&content_length.to_string())
            .expect("u64 content length is always a valid header"),
    );
    headers.insert(
        ETAG,
        HeaderValue::from_str(&format!(
            "\"cf-{}-{}\"",
            digest.algorithm.tag(),
            hex::encode(digest.bytes)
        ))
        .expect("canonical digest ETag is always a valid header"),
    );
    response
}

fn parse_digest(algorithm: &str, digest: &str) -> Result<ContentDigestV1, ApiErrorV1> {
    let algorithm = match algorithm {
        "blake3-256" => DigestAlgorithmV1::Blake3_256,
        "sha256" => DigestAlgorithmV1::Sha256,
        _ => return Err(ApiErrorV1::BadDigest),
    };
    if digest.len() != 64
        || !digest
            .as_bytes()
            .iter()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(byte))
    {
        return Err(ApiErrorV1::BadDigest);
    }
    let mut bytes = [0_u8; 32];
    hex::decode_to_slice(digest, &mut bytes).map_err(|_| ApiErrorV1::BadDigest)?;
    Ok(ContentDigestV1 { algorithm, bytes })
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::http::{Method, Request};
    use http_body_util::BodyExt as _;
    use mycelix_content_core::BlobDescriptorV1;
    use tower::ServiceExt as _;

    fn test_router(bytes: &[u8]) -> (Router, ContentDigestV1, tempfile::TempDir) {
        let dir = tempfile::tempdir().unwrap();
        let cas = LocalCasV1::open(mycelix_content_node::CasConfigV1::new(dir.path(), 4096))
            .unwrap();
        let descriptor =
            BlobDescriptorV1::from_bytes(DigestAlgorithmV1::Blake3_256, bytes, None);
        cas.put(&descriptor, bytes).unwrap();
        (build_router(Arc::new(cas), 2), descriptor.digest, dir)
    }

    fn blob_uri(digest: ContentDigestV1) -> String {
        format!(
            "/v1/blobs/{}/{}",
            digest.algorithm.tag(),
            hex::encode(digest.bytes)
        )
    }

    #[tokio::test]
    async fn full_get_returns_verified_immutable_content() {
        let (app, digest, _dir) = test_router(b"abcdef");
        let response = app
            .oneshot(
                Request::builder()
                    .uri(blob_uri(digest))
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::OK);
        assert_eq!(response.headers()[ACCEPT_RANGES], "bytes");
        assert_eq!(response.headers()[CONTENT_LENGTH], "6");
        assert_eq!(response.headers()[X_CONTENT_TYPE_OPTIONS], "nosniff");
        let body = response.into_body().collect().await.unwrap().to_bytes();
        assert_eq!(&body[..], b"abcdef");
    }

    #[tokio::test]
    async fn range_get_returns_single_partial_range() {
        let (app, digest, _dir) = test_router(b"abcdef");
        let response = app
            .oneshot(
                Request::builder()
                    .uri(blob_uri(digest))
                    .header(RANGE, "bytes=1-3")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::PARTIAL_CONTENT);
        assert_eq!(response.headers()[CONTENT_RANGE], "bytes 1-3/6");
        assert_eq!(response.headers()[CONTENT_LENGTH], "3");
        let body = response.into_body().collect().await.unwrap().to_bytes();
        assert_eq!(&body[..], b"bcd");
    }

    #[tokio::test]
    async fn unsatisfiable_range_returns_416_with_object_size() {
        let (app, digest, _dir) = test_router(b"abcdef");
        let response = app
            .oneshot(
                Request::builder()
                    .uri(blob_uri(digest))
                    .header(RANGE, "bytes=99-")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::RANGE_NOT_SATISFIABLE);
        assert_eq!(response.headers()[CONTENT_RANGE], "bytes */6");
    }

    #[tokio::test]
    async fn head_verifies_but_returns_no_body() {
        let (app, digest, _dir) = test_router(b"abcdef");
        let response = app
            .oneshot(
                Request::builder()
                    .method(Method::HEAD)
                    .uri(blob_uri(digest))
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::OK);
        assert_eq!(response.headers()[CONTENT_LENGTH], "6");
        let body = response.into_body().collect().await.unwrap().to_bytes();
        assert!(body.is_empty());
    }

    #[tokio::test]
    async fn mutation_methods_are_not_exposed() {
        let (app, digest, _dir) = test_router(b"abcdef");
        let response = app
            .oneshot(
                Request::builder()
                    .method(Method::PUT)
                    .uri(blob_uri(digest))
                    .body(Body::from("replacement"))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::METHOD_NOT_ALLOWED);
    }

    #[tokio::test]
    async fn malformed_digest_is_rejected_before_storage_work() {
        let (app, _digest, _dir) = test_router(b"abcdef");
        let response = app
            .oneshot(
                Request::builder()
                    .uri("/v1/blobs/blake3-256/NOT-A-DIGEST")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
    }

    #[test]
    fn public_bind_is_fail_closed() {
        let config = HttpConfigV1 {
            bind_addr: "0.0.0.0:8080".parse().unwrap(),
            max_concurrent_verifications: 4,
        };
        assert!(matches!(
            config.validate(),
            Err(HttpFacadeErrorV1::NonLoopbackBind(_))
        ));
    }
}
