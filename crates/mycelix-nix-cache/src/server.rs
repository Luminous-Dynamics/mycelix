use std::{
    net::{IpAddr, Ipv4Addr, SocketAddr},
    sync::Arc,
};

use axum::{
    body::Body,
    extract::{Path, State},
    http::{
        header::{CACHE_CONTROL, CONTENT_LENGTH, CONTENT_TYPE, ETAG, RETRY_AFTER},
        HeaderValue, StatusCode,
    },
    response::{IntoResponse, Response},
    routing::get,
    Router,
};
use mycelix_content_node::{CasErrorV1, LocalCasV1, VerifiedBlobV1};
use tokio::{net::TcpListener, sync::Semaphore};
use tokio_util::io::ReaderStream;

use crate::{
    encode_nix_base32, NixCacheCatalogV1, NixCacheEntryV1, NixCacheErrorV1, NixStoreHashV1,
    NIX_STORE_DIR_V1,
};

const MAX_VERIFICATION_CONCURRENCY_V1: usize = 64;

#[derive(Debug, Clone)]
pub struct NixCacheConfigV1 {
    pub bind_addr: SocketAddr,
    pub priority: u32,
    pub max_concurrent_verifications: usize,
}

impl NixCacheConfigV1 {
    pub fn loopback(port: u16) -> Self {
        Self {
            bind_addr: SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), port),
            priority: 40,
            max_concurrent_verifications: 4,
        }
    }

    pub fn validate(&self) -> Result<(), NixCacheErrorV1> {
        if !self.bind_addr.ip().is_loopback() {
            return Err(NixCacheErrorV1::NonLoopbackBind(self.bind_addr));
        }
        if self.max_concurrent_verifications == 0 {
            return Err(NixCacheErrorV1::ZeroVerificationConcurrency);
        }
        if self.max_concurrent_verifications > MAX_VERIFICATION_CONCURRENCY_V1 {
            return Err(NixCacheErrorV1::VerificationConcurrencyTooHigh {
                requested: self.max_concurrent_verifications,
                maximum: MAX_VERIFICATION_CONCURRENCY_V1,
            });
        }
        Ok(())
    }
}

#[derive(Clone)]
struct AppStateV1 {
    cas: Arc<LocalCasV1>,
    catalog: Arc<NixCacheCatalogV1>,
    priority: u32,
    verification_slots: Arc<Semaphore>,
}

#[derive(Debug)]
enum ApiErrorV1 {
    NotFound,
    IntegrityFailure,
    StorageUnavailable,
    VerificationBusy,
    VerificationTaskFailed,
}

impl IntoResponse for ApiErrorV1 {
    fn into_response(self) -> Response {
        match self {
            Self::NotFound => StatusCode::NOT_FOUND.into_response(),
            Self::IntegrityFailure => (
                StatusCode::INTERNAL_SERVER_ERROR,
                "local NAR integrity failure",
            )
                .into_response(),
            Self::StorageUnavailable => (
                StatusCode::SERVICE_UNAVAILABLE,
                "Nix cache storage unavailable",
            )
                .into_response(),
            Self::VerificationBusy => {
                let mut response = (
                    StatusCode::SERVICE_UNAVAILABLE,
                    "Nix cache verification capacity is busy",
                )
                    .into_response();
                response
                    .headers_mut()
                    .insert(RETRY_AFTER, HeaderValue::from_static("1"));
                response
            }
            Self::VerificationTaskFailed => StatusCode::INTERNAL_SERVER_ERROR.into_response(),
        }
    }
}

pub async fn serve(
    cas: Arc<LocalCasV1>,
    catalog: Arc<NixCacheCatalogV1>,
    config: NixCacheConfigV1,
) -> Result<(), NixCacheErrorV1> {
    config.validate()?;
    let listener = TcpListener::bind(config.bind_addr)
        .await
        .map_err(|source| NixCacheErrorV1::Bind {
            addr: config.bind_addr,
            source,
        })?;
    let app = router(cas, catalog, &config)?;
    axum::serve(listener, app)
        .await
        .map_err(NixCacheErrorV1::Serve)
}

pub fn router(
    cas: Arc<LocalCasV1>,
    catalog: Arc<NixCacheCatalogV1>,
    config: &NixCacheConfigV1,
) -> Result<Router, NixCacheErrorV1> {
    config.validate()?;
    let state = AppStateV1 {
        cas,
        catalog,
        priority: config.priority,
        verification_slots: Arc::new(Semaphore::new(config.max_concurrent_verifications)),
    };
    Ok(Router::new()
        .route(
            "/nix-cache-info",
            get(get_cache_info).head(head_cache_info),
        )
        .route("/{filename}", get(get_narinfo).head(head_narinfo))
        .route("/nar/{filename}", get(get_nar).head(head_nar))
        .with_state(state))
}

async fn get_cache_info(State(state): State<AppStateV1>) -> Response {
    text_response(
        nix_cache_info_text(state.priority),
        "text/plain; charset=utf-8",
        false,
    )
}

async fn head_cache_info(State(state): State<AppStateV1>) -> Response {
    head_text_response(
        nix_cache_info_text(state.priority),
        "text/plain; charset=utf-8",
        false,
    )
}

fn nix_cache_info_text(priority: u32) -> String {
    format!("StoreDir: {NIX_STORE_DIR_V1}\nWantMassQuery: 0\nPriority: {priority}\n")
}

async fn get_narinfo(
    State(state): State<AppStateV1>,
    Path(filename): Path<String>,
) -> Result<Response, ApiErrorV1> {
    let entry = catalog_entry_for_narinfo(&state, &filename)?.clone();
    verify_entry(&state, &entry).await?;
    Ok(text_response(
        entry.render_narinfo(),
        "text/x-nix-narinfo",
        false,
    ))
}

async fn head_narinfo(
    State(state): State<AppStateV1>,
    Path(filename): Path<String>,
) -> Result<Response, ApiErrorV1> {
    let entry = catalog_entry_for_narinfo(&state, &filename)?.clone();
    verify_entry(&state, &entry).await?;
    Ok(head_text_response(
        entry.render_narinfo(),
        "text/x-nix-narinfo",
        false,
    ))
}

async fn get_nar(
    State(state): State<AppStateV1>,
    Path(filename): Path<String>,
) -> Result<Response, ApiErrorV1> {
    let entry = catalog_entry_for_nar(&state, &filename)?.clone();
    let verified = verify_entry(&state, &entry).await?;
    Ok(stream_nar_response(&entry, verified))
}

async fn head_nar(
    State(state): State<AppStateV1>,
    Path(filename): Path<String>,
) -> Result<Response, ApiErrorV1> {
    let entry = catalog_entry_for_nar(&state, &filename)?.clone();
    verify_entry(&state, &entry).await?;
    Ok(nar_headers(&entry, Body::empty()))
}

fn catalog_entry_for_narinfo<'a>(
    state: &'a AppStateV1,
    filename: &str,
) -> Result<&'a NixCacheEntryV1, ApiErrorV1> {
    let store_hash = filename
        .strip_suffix(".narinfo")
        .ok_or(ApiErrorV1::NotFound)
        .and_then(|value| NixStoreHashV1::parse(value).map_err(|_| ApiErrorV1::NotFound))?;
    state
        .catalog
        .entry(&store_hash)
        .ok_or(ApiErrorV1::NotFound)
}

fn catalog_entry_for_nar<'a>(
    state: &'a AppStateV1,
    filename: &str,
) -> Result<&'a NixCacheEntryV1, ApiErrorV1> {
    let stem = filename
        .strip_suffix(".nar")
        .ok_or(ApiErrorV1::NotFound)?;
    let (store_hash, expected_nar_hash) = stem.split_once('-').ok_or(ApiErrorV1::NotFound)?;
    let store_hash = NixStoreHashV1::parse(store_hash).map_err(|_| ApiErrorV1::NotFound)?;
    let entry = state
        .catalog
        .entry(&store_hash)
        .ok_or(ApiErrorV1::NotFound)?;
    if expected_nar_hash != encode_nix_base32(&entry.nar_digest().bytes) {
        return Err(ApiErrorV1::NotFound);
    }
    Ok(entry)
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
    headers.insert(
        CACHE_CONTROL,
        HeaderValue::from_static("public, max-age=31536000, immutable"),
    );
    headers.insert(
        ETAG,
        HeaderValue::from_str(&format!(
            "\"nix-sha256-{}\"",
            encode_nix_base32(&entry.nar_digest().bytes)
        ))
        .expect("canonical Nix hash ETag is always valid"),
    );
    response
}

fn text_response(text: String, content_type: &'static str, immutable: bool) -> Response {
    let length = text.len();
    let mut response = Response::new(Body::from(text));
    insert_text_headers(&mut response, length, content_type, immutable);
    response
}

fn head_text_response(text: String, content_type: &'static str, immutable: bool) -> Response {
    let length = text.len();
    let mut response = Response::new(Body::empty());
    insert_text_headers(&mut response, length, content_type, immutable);
    response
}

fn insert_text_headers(
    response: &mut Response,
    length: usize,
    content_type: &'static str,
    immutable: bool,
) {
    let headers = response.headers_mut();
    headers.insert(CONTENT_TYPE, HeaderValue::from_static(content_type));
    headers.insert(
        CONTENT_LENGTH,
        HeaderValue::from_str(&length.to_string())
            .expect("usize content length is always a valid header"),
    );
    headers.insert(
        CACHE_CONTROL,
        if immutable {
            HeaderValue::from_static("public, max-age=31536000, immutable")
        } else {
            HeaderValue::from_static("public, max-age=60")
        },
    );
}
