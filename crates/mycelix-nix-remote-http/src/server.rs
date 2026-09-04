use std::sync::{
    atomic::{AtomicU64, Ordering},
    Arc,
};

use axum::{
    body::Body,
    extract::{Path, State},
    http::{
        header::{CACHE_CONTROL, CONTENT_LENGTH, CONTENT_TYPE, ETAG, RETRY_AFTER},
        HeaderName, HeaderValue, StatusCode,
    },
    response::{IntoResponse, Response},
    routing::get,
    Router,
};
use mycelix_content_node::{CasErrorV1, LocalCasV1, VerifiedBlobV1};
use mycelix_nix_cache::{encode_nix_base32, NixCacheEntryV1, NixStoreHashV1, NIX_STORE_DIR_V1};
use mycelix_nix_exposure::{RemoteReaderV1, RemoteServingSnapshotV1};
use tokio::sync::Semaphore;
use tokio_util::io::ReaderStream;

use crate::{RemoteClockV1, RemoteNixRouterConfigV1, RemoteNixRouterErrorV1};

#[derive(Clone)]
struct AppStateV1 {
    cas: Arc<LocalCasV1>,
    snapshot: Arc<RemoteServingSnapshotV1>,
    clock: Arc<dyn RemoteClockV1>,
    last_observed_unix_ms: Arc<AtomicU64>,
    anonymous: RemoteReaderV1,
    priority: u32,
    verification_slots: Arc<Semaphore>,
}

#[derive(Debug)]
enum ApiErrorV1 {
    NotFound,
    MethodNotAllowed,
    AuthorityUnavailable,
    IntegrityFailure,
    StorageUnavailable,
    VerificationBusy,
    VerificationTaskFailed,
}

impl IntoResponse for ApiErrorV1 {
    fn into_response(self) -> Response {
        let (status, body, retry) = match self {
            Self::NotFound => (StatusCode::NOT_FOUND, "", false),
            Self::MethodNotAllowed => (StatusCode::METHOD_NOT_ALLOWED, "", false),
            Self::AuthorityUnavailable => (
                StatusCode::SERVICE_UNAVAILABLE,
                "remote exposure authority requires refresh",
                true,
            ),
            Self::IntegrityFailure => (
                StatusCode::INTERNAL_SERVER_ERROR,
                "local NAR integrity failure",
                false,
            ),
            Self::StorageUnavailable => (
                StatusCode::SERVICE_UNAVAILABLE,
                "Nix cache storage unavailable",
                true,
            ),
            Self::VerificationBusy => (
                StatusCode::SERVICE_UNAVAILABLE,
                "Nix cache verification capacity is busy",
                true,
            ),
            Self::VerificationTaskFailed => (
                StatusCode::INTERNAL_SERVER_ERROR,
                "Nix cache verification task failed",
                false,
            ),
        };
        let mut response = (status, body).into_response();
        response
            .headers_mut()
            .insert(CACHE_CONTROL, HeaderValue::from_static("no-store"));
        if retry {
            response
                .headers_mut()
                .insert(RETRY_AFTER, HeaderValue::from_static("1"));
        }
        response
    }
}

/// Construct the CF-07B public-audience router core.
///
/// This function deliberately returns only an Axum `Router`; it does not bind a
/// socket. It accepts no raw `NixCacheCatalogV1` and no general
/// `RemoteExposureSnapshotV1`. Restricted principal/group grants in the serving
/// snapshot remain invisible because every request is evaluated as anonymous.
pub fn public_router(
    cas: Arc<LocalCasV1>,
    snapshot: Arc<RemoteServingSnapshotV1>,
    clock: Arc<dyn RemoteClockV1>,
    config: RemoteNixRouterConfigV1,
) -> Result<Router, RemoteNixRouterErrorV1> {
    let config = config.validate()?;
    let state = AppStateV1 {
        cas,
        last_observed_unix_ms: Arc::new(AtomicU64::new(snapshot.evaluation_time_unix_ms())),
        snapshot,
        clock,
        anonymous: RemoteReaderV1::anonymous(),
        priority: config.priority,
        verification_slots: Arc::new(Semaphore::new(config.max_concurrent_verifications)),
    };
    Ok(Router::new()
        .route(
            "/nix-cache-info",
            get(get_cache_info)
                .head(head_cache_info)
                .fallback(method_not_allowed),
        )
        .route(
            "/{filename}",
            get(get_narinfo)
                .head(head_narinfo)
                .fallback(method_not_allowed),
        )
        .route(
            "/nar/{filename}",
            get(get_nar).head(head_nar).fallback(method_not_allowed),
        )
        .fallback(route_not_found)
        .with_state(state))
}

async fn method_not_allowed() -> ApiErrorV1 {
    ApiErrorV1::MethodNotAllowed
}

async fn route_not_found() -> ApiErrorV1 {
    ApiErrorV1::NotFound
}

async fn get_cache_info(State(state): State<AppStateV1>) -> Result<Response, ApiErrorV1> {
    require_current_snapshot(&state)?;
    Ok(text_response(
        nix_cache_info_text(state.priority),
        "text/plain; charset=utf-8",
    ))
}

async fn head_cache_info(State(state): State<AppStateV1>) -> Result<Response, ApiErrorV1> {
    require_current_snapshot(&state)?;
    Ok(head_text_response(
        nix_cache_info_text(state.priority),
        "text/plain; charset=utf-8",
    ))
}

fn nix_cache_info_text(priority: u32) -> String {
    format!("StoreDir: {NIX_STORE_DIR_V1}\nWantMassQuery: 0\nPriority: {priority}\n")
}

async fn get_narinfo(
    State(state): State<AppStateV1>,
    Path(filename): Path<String>,
) -> Result<Response, ApiErrorV1> {
    let entry = authorized_narinfo_entry(&state, &filename)?.clone();
    verify_entry(&state, &entry).await?;
    require_current_snapshot(&state)?;
    Ok(text_response(entry.render_narinfo(), "text/x-nix-narinfo"))
}

async fn head_narinfo(
    State(state): State<AppStateV1>,
    Path(filename): Path<String>,
) -> Result<Response, ApiErrorV1> {
    let entry = authorized_narinfo_entry(&state, &filename)?.clone();
    verify_entry(&state, &entry).await?;
    require_current_snapshot(&state)?;
    Ok(head_text_response(
        entry.render_narinfo(),
        "text/x-nix-narinfo",
    ))
}

async fn get_nar(
    State(state): State<AppStateV1>,
    Path(filename): Path<String>,
) -> Result<Response, ApiErrorV1> {
    let entry = authorized_nar_entry(&state, &filename)?.clone();
    let verified = verify_entry(&state, &entry).await?;
    require_current_snapshot(&state)?;
    Ok(stream_nar_response(&entry, verified))
}

async fn head_nar(
    State(state): State<AppStateV1>,
    Path(filename): Path<String>,
) -> Result<Response, ApiErrorV1> {
    let entry = authorized_nar_entry(&state, &filename)?.clone();
    verify_entry(&state, &entry).await?;
    require_current_snapshot(&state)?;
    Ok(nar_headers(&entry, Body::empty()))
}

fn authorized_narinfo_entry<'a>(
    state: &'a AppStateV1,
    filename: &str,
) -> Result<&'a NixCacheEntryV1, ApiErrorV1> {
    let store_hash = filename
        .strip_suffix(".narinfo")
        .ok_or(ApiErrorV1::NotFound)
        .and_then(|value| NixStoreHashV1::parse(value).map_err(|_| ApiErrorV1::NotFound))?;
    authorized_entry(state, &store_hash)
}

fn authorized_nar_entry<'a>(
    state: &'a AppStateV1,
    filename: &str,
) -> Result<&'a NixCacheEntryV1, ApiErrorV1> {
    let stem = filename
        .strip_suffix(".nar")
        .ok_or(ApiErrorV1::NotFound)?;
    let (store_hash, expected_nar_hash) = stem.split_once('-').ok_or(ApiErrorV1::NotFound)?;
    let store_hash = NixStoreHashV1::parse(store_hash).map_err(|_| ApiErrorV1::NotFound)?;
    let entry = authorized_entry(state, &store_hash)?;
    if expected_nar_hash != encode_nix_base32(&entry.nar_digest().bytes) {
        return Err(ApiErrorV1::NotFound);
    }
    Ok(entry)
}

fn authorized_entry<'a>(
    state: &'a AppStateV1,
    store_hash: &NixStoreHashV1,
) -> Result<&'a NixCacheEntryV1, ApiErrorV1> {
    let now = trusted_now(state)?;
    state
        .snapshot
        .entry_for_reader_at(store_hash, &state.anonymous, now)
        .map_err(|_| ApiErrorV1::AuthorityUnavailable)?
        .ok_or(ApiErrorV1::NotFound)
}

fn require_current_snapshot(state: &AppStateV1) -> Result<u64, ApiErrorV1> {
    let now = trusted_now(state)?;
    if state.snapshot.is_valid_at(now) {
        Ok(now)
    } else {
        Err(ApiErrorV1::AuthorityUnavailable)
    }
}

fn trusted_now(state: &AppStateV1) -> Result<u64, ApiErrorV1> {
    let now = state
        .clock
        .now_unix_ms()
        .map_err(|_| ApiErrorV1::AuthorityUnavailable)?;
    let previous = state.last_observed_unix_ms.fetch_max(now, Ordering::SeqCst);
    if now < previous {
        return Err(ApiErrorV1::AuthorityUnavailable);
    }
    Ok(now)
}

async fn verify_entry(
    state: &AppStateV1,
    entry: &NixCacheEntryV1,
) -> Result<VerifiedBlobV1, ApiErrorV1> {
    let _permit = state
        .verification_slots
        .clone()
        .try_acquire_owned()
        .map_err(|_| ApiErrorV1::VerificationBusy)?;
    let cas = state.cas.clone();
    let digest = entry.nar_digest();
    let expected_size = entry.nar_size();
    let verified = tokio::task::spawn_blocking(move || cas.open_verified_digest(digest))
        .await
        .map_err(|_| ApiErrorV1::VerificationTaskFailed)?
        .map_err(map_cas_error)?;
    if verified.size_bytes() != expected_size {
        return Err(ApiErrorV1::IntegrityFailure);
    }
    Ok(verified)
}

fn map_cas_error(error: CasErrorV1) -> ApiErrorV1 {
    match error {
        CasErrorV1::NotFound(_) => ApiErrorV1::StorageUnavailable,
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

fn stream_nar_response(entry: &NixCacheEntryV1, verified: VerifiedBlobV1) -> Response {
    let file = tokio::fs::File::from_std(verified.into_file());
    let body = Body::from_stream(ReaderStream::new(file));
    nar_headers(entry, body)
}

fn nar_headers(entry: &NixCacheEntryV1, body: Body) -> Response {
    let mut response = Response::new(body);
    let headers = response.headers_mut();
    headers.insert(
        CONTENT_TYPE,
        HeaderValue::from_static("application/octet-stream"),
    );
    headers.insert(
        CONTENT_LENGTH,
        HeaderValue::from_str(&entry.nar_size().to_string())
            .expect("u64 NAR size is always a valid header"),
    );
    headers.insert(CACHE_CONTROL, HeaderValue::from_static("no-store"));
    headers.insert(
        ETAG,
        HeaderValue::from_str(&format!(
            "\"nix-sha256-{}\"",
            encode_nix_base32(&entry.nar_digest().bytes)
        ))
        .expect("canonical Nix hash ETag is always valid"),
    );
    headers.insert(
        HeaderName::from_static("x-content-type-options"),
        HeaderValue::from_static("nosniff"),
    );
    response
}

fn text_response(text: String, content_type: &'static str) -> Response {
    let length = text.len();
    let mut response = Response::new(Body::from(text));
    insert_text_headers(&mut response, length, content_type);
    response
}

fn head_text_response(text: String, content_type: &'static str) -> Response {
    let length = text.len();
    let mut response = Response::new(Body::empty());
    insert_text_headers(&mut response, length, content_type);
    response
}

fn insert_text_headers(response: &mut Response, length: usize, content_type: &'static str) {
    let headers = response.headers_mut();
    headers.insert(CONTENT_TYPE, HeaderValue::from_static(content_type));
    headers.insert(
        CONTENT_LENGTH,
        HeaderValue::from_str(&length.to_string())
            .expect("usize content length is always a valid header"),
    );
    headers.insert(CACHE_CONTROL, HeaderValue::from_static("no-store"));
    headers.insert(
        HeaderName::from_static("x-content-type-options"),
        HeaderValue::from_static("nosniff"),
    );
}
