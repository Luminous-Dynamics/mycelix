// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Unified Prism server: static files + CORS proxy on a single port.
//!
//! Serves the WASM app from dist/ and proxies external API requests,
//! eliminating the need for a separate proxy service and port.

use axum::Router;
use axum::extract::{DefaultBodyLimit, Query, State};
use axum::http::StatusCode;
use axum::http::header::HeaderValue;
use axum::response::IntoResponse;
use axum::routing::{get, post};
use prism_common::ssrf::validate_proxy_url;
use prism_egress::{EgressAuthority, EgressBody, EgressError, PublicEgressClient};
use reqwest::{Method, header::HeaderMap};
use std::sync::Arc;
use std::time::Duration;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;
use tower_http::compression::CompressionLayer;
use tower_http::services::{ServeDir, ServeFile};
use tower_http::set_header::SetResponseHeaderLayer;
use url::Url;

const MAX_REQUEST_BODY_SIZE: usize = 2 * 1024 * 1024; // 2 MiB
const MAX_SYMTHAEA_RESPONSE_LINE_BYTES: usize = 64 * 1024;

async fn egress_request(
    method: Method,
    url: &str,
    headers: HeaderMap,
    body: Vec<u8>,
) -> Result<prism_egress::EgressResponse, EgressError> {
    PublicEgressClient::default()
        .request(method, url, headers, body)
        .await
}

async fn egress_provider_request(
    authority: EgressAuthority,
    method: Method,
    url: &str,
    headers: HeaderMap,
    body: Vec<u8>,
) -> Result<prism_egress::EgressResponse, EgressError> {
    PublicEgressClient::default()
        .request_with_authority(authority, method, url, headers, body)
        .await
}

fn log_egress_receipt(context: &str, receipt: &prism_egress::EgressReceipt) {
    // Receipts are deliberately metadata-only: no URL path/query, headers,
    // credentials, request body, or response body enters the audit record.
    log::info!(
        "Prism egress receipt context={} authority={} origin={} method={} peer={} authorized={:?} status={} request_bytes={} response_bytes={} response_limit={} redirects={} elapsed_ms={} policy={}",
        context,
        receipt.authority,
        receipt.origin,
        receipt.method,
        receipt.remote_peer,
        receipt.authorized_addresses,
        receipt.status,
        receipt.request_body_bytes,
        receipt.response_body_bytes,
        receipt.response_body_limit_bytes,
        receipt.redirect_hops.len(),
        receipt.elapsed.as_millis(),
        receipt.policy_version,
    );
    for (index, hop) in receipt.redirect_hops.iter().enumerate() {
        log::info!(
            "Prism egress redirect context={} hop={} from={} to={} status={} peer={} target_addresses={:?}",
            context,
            index + 1,
            hop.from_origin,
            hop.to_origin,
            hop.status,
            hop.remote_peer,
            hop.target_addresses,
        );
    }
}

fn upstream_error(context: &str, error: EgressError) -> axum::response::Response<axum::body::Body> {
    let (status, message) = match error {
        EgressError::InvalidUrl(_) => (StatusCode::BAD_REQUEST, "Invalid request URL"),
        EgressError::UrlTooLarge => (StatusCode::URI_TOO_LONG, "Request URL too large"),
        EgressError::RequestBodyTooLarge => {
            (StatusCode::PAYLOAD_TOO_LARGE, "Request body too large")
        }
        EgressError::RequestHeadersTooLarge | EgressError::ForbiddenHeader(_) => {
            (StatusCode::BAD_REQUEST, "Request headers not permitted")
        }
        EgressError::ResponseBodyTooLarge | EgressError::ResponseHeadersTooLarge => (
            StatusCode::BAD_GATEWAY,
            "Upstream response exceeded its resource budget",
        ),
        EgressError::DnsResolution { .. }
        | EgressError::DnsResolutionTimeout
        | EgressError::DnsCapacityUnavailable
        | EgressError::DnsAnswerSetTooLarge
        | EgressError::EmptyAddressSet(_)
        | EgressError::RemotePeerUnavailable
        | EgressError::RemotePeerNotAuthorized(_)
        | EgressError::RedirectLimitExceeded
        | EgressError::InvalidRedirectLocation
        | EgressError::RedirectLoop
        | EgressError::Http(_) => (StatusCode::BAD_GATEWAY, "Upstream request failed"),
        EgressError::CapabilityExpired
        | EgressError::ConcurrencyQueueTimeout
        | EgressError::ConcurrencyUnavailable
        | EgressError::ResponseMemoryBudgetTimeout
        | EgressError::ResponseMemoryBudgetUnavailable => (
            StatusCode::SERVICE_UNAVAILABLE,
            "Egress capacity is unavailable",
        ),
        EgressError::ExecutionTimeout => (
            StatusCode::GATEWAY_TIMEOUT,
            "Egress execution deadline exceeded",
        ),
        EgressError::UnsupportedScheme
        | EgressError::CredentialsNotPermitted
        | EgressError::BlockedHost
        | EgressError::PortNotPermitted
        | EgressError::MissingHost
        | EgressError::AddressNotPermitted(_)
        | EgressError::MethodNotPermitted
        | EgressError::RequestRequiresTls
        | EgressError::RedirectSchemeDowngrade
        | EgressError::AuthorityScopeViolation
        | EgressError::AuthorityHeaderViolation(_)
        | EgressError::ProviderCredentialMissing => (
            StatusCode::FORBIDDEN,
            "Destination or request not permitted",
        ),
    };
    // Do not log URLs, query strings, header values, or transport error details.
    log::warn!("Prism egress {context} returned status {}", status.as_u16());
    (status, message).into_response()
}

fn json_response(status: u16, body: EgressBody) -> axum::response::Response<axum::body::Body> {
    axum::response::Response::builder()
        .status(status)
        .header("content-type", "application/json")
        .body(axum::body::Body::from(body))
        .unwrap_or_else(|_| internal_error_response())
}

fn internal_error_response() -> axum::response::Response<axum::body::Body> {
    let mut resp = axum::response::Response::new(axum::body::Body::from("Internal error"));
    *resp.status_mut() = StatusCode::INTERNAL_SERVER_ERROR;
    resp
}

// ═══════════════════════════════════════════════════════════════
// PROXY HANDLERS
// ═══════════════════════════════════════════════════════════════

#[derive(serde::Deserialize)]
struct ProxyParams {
    url: String,
}

async fn proxy_handler(Query(params): Query<ProxyParams>) -> impl IntoResponse {
    let validated = match validate_proxy_url(&params.url) {
        Ok(url) => url,
        Err(reason) => {
            log::warn!("Blocked Prism proxy URL: {reason}");
            return (StatusCode::FORBIDDEN, reason.to_string()).into_response();
        }
    };

    match egress_request(
        Method::GET,
        validated.as_str(),
        HeaderMap::new(),
        Vec::new(),
    )
    .await
    {
        Ok(response) => {
            log_egress_receipt("proxy", &response.receipt);
            let content_type = response
                .headers
                .get(reqwest::header::CONTENT_TYPE)
                .cloned()
                .unwrap_or_else(|| HeaderValue::from_static("text/html"));
            axum::response::Response::builder()
                .status(response.status.as_u16())
                .header(axum::http::header::CONTENT_TYPE, content_type)
                .header("x-prism-proxied", "true")
                .body(axum::body::Body::from(response.body.into_bytes()))
                .unwrap_or_else(|_| internal_error_response())
                .into_response()
        }
        Err(error) => upstream_error("proxy", error).into_response(),
    }
}

#[derive(serde::Deserialize)]
struct DdgParams {
    q: String,
}

async fn ddg_handler(Query(params): Query<DdgParams>) -> impl IntoResponse {
    let mut url = Url::parse("https://api.duckduckgo.com/").expect("static DDG URL is valid");
    url.query_pairs_mut()
        .append_pair("q", &params.q)
        .append_pair("format", "json")
        .append_pair("no_html", "1")
        .append_pair("skip_disambig", "1");

    match egress_request(Method::GET, url.as_str(), HeaderMap::new(), Vec::new()).await {
        Ok(response) => {
            log_egress_receipt("DuckDuckGo", &response.receipt);
            json_response(response.status.as_u16(), response.body).into_response()
        }
        Err(error) => upstream_error("DuckDuckGo", error).into_response(),
    }
}

#[derive(serde::Deserialize)]
struct BraveParams {
    q: String,
}

async fn brave_handler(
    headers: axum::http::HeaderMap,
    Query(params): Query<BraveParams>,
) -> impl IntoResponse {
    let api_key = headers
        .get("X-Brave-Key")
        .and_then(|v| v.to_str().ok())
        .unwrap_or("");
    if api_key.is_empty() {
        return (StatusCode::UNAUTHORIZED, "Missing X-Brave-Key header").into_response();
    }
    let mut url = Url::parse("https://api.search.brave.com/res/v1/web/search")
        .expect("static Brave URL is valid");
    url.query_pairs_mut().append_pair("q", &params.q);

    let token = match HeaderValue::from_str(api_key) {
        Ok(value) => value,
        Err(_) => return (StatusCode::BAD_REQUEST, "Invalid Brave API key header").into_response(),
    };
    let mut outbound_headers = HeaderMap::new();
    outbound_headers.insert("X-Subscription-Token", token);
    outbound_headers.insert(
        reqwest::header::ACCEPT,
        HeaderValue::from_static("application/json"),
    );

    match egress_provider_request(
        EgressAuthority::BraveSearch,
        Method::GET,
        url.as_str(),
        outbound_headers,
        Vec::new(),
    )
    .await
    {
        Ok(response) => {
            log_egress_receipt("Brave", &response.receipt);
            json_response(response.status.as_u16(), response.body).into_response()
        }
        Err(error) => upstream_error("Brave", error).into_response(),
    }
}

async fn perplexity_handler(
    headers: axum::http::HeaderMap,
    body: axum::body::Bytes,
) -> impl IntoResponse {
    let api_key = headers
        .get("X-Perplexity-Key")
        .and_then(|v| v.to_str().ok())
        .unwrap_or("");
    if api_key.is_empty() {
        return (StatusCode::UNAUTHORIZED, "Missing X-Perplexity-Key header").into_response();
    }
    let authorization = match HeaderValue::from_str(&format!("Bearer {api_key}")) {
        Ok(value) => value,
        Err(_) => {
            return (StatusCode::BAD_REQUEST, "Invalid Perplexity API key header").into_response();
        }
    };
    let mut outbound_headers = HeaderMap::new();
    outbound_headers.insert(reqwest::header::AUTHORIZATION, authorization);
    outbound_headers.insert(
        reqwest::header::CONTENT_TYPE,
        HeaderValue::from_static("application/json"),
    );

    match egress_provider_request(
        EgressAuthority::Perplexity,
        Method::POST,
        "https://api.perplexity.ai/chat/completions",
        outbound_headers,
        body.to_vec(),
    )
    .await
    {
        Ok(response) => {
            log_egress_receipt("Perplexity", &response.receipt);
            json_response(response.status.as_u16(), response.body).into_response()
        }
        Err(error) => upstream_error("Perplexity", error).into_response(),
    }
}

// ═══════════════════════════════════════════════════════════════
// SYMTHAEA BRIDGE
// ═══════════════════════════════════════════════════════════════
//
// Forwards a query to the Symthaea Service Daemon (a separate systemd
// service, see /srv/luminous-dynamics/symthaea/nix/modules/
// symthaea-service.nix) over its Unix socket. This is the opt-in "Ask
// Symthaea" action — deliberately not wired into every search (see
// Part 2 of /home/tstoltz/.claude/plans/fuzzy-beaming-brook.md) — so
// volume stays low/high-signal and the daemon (which has no rate
// limiting of its own) isn't exposed to raw request volume.
//
// The daemon has none of its own rate limiting, so this route applies
// its own ConcurrencyLimitLayer + LoadShedLayer + TimeoutLayer (wired in
// build_app) sized to the daemon's single-mutex-serialized capacity.

const SYMTHAEA_MAX_QUERY_CHARS: usize = 2000;

/// Concurrency-limit + load-shed + timeout stack for the Symthaea route.
/// A macro rather than a function because the ServiceBuilder stack's type
/// is awkward to name explicitly; HandleErrorLayer is mandatory here —
/// axum's Router::layer requires an Infallible service error, but
/// LoadShed/Timeout produce a boxed error without it (compile error
/// otherwise: "the trait bound `Infallible: From<Box<dyn Error...>>` is
/// not satisfied").
macro_rules! symthaea_rate_limit_layer {
    ($timeout_duration:expr) => {
        tower::ServiceBuilder::new()
            .layer(axum::error_handling::HandleErrorLayer::new(
                |_: axum::BoxError| async {
                    (
                        StatusCode::SERVICE_UNAVAILABLE,
                        "Symthaea is busy, try again shortly",
                    )
                },
            ))
            .load_shed()
            .concurrency_limit(3)
            .timeout($timeout_duration)
    };
}

/// Config for reaching the Symthaea daemon. `None` (e.g. in tests, or a
/// deploy that hasn't set the env vars) makes the route return 503 rather
/// than panicking — this bridge is optional, additive functionality.
#[derive(Clone)]
struct SymthaeaConfig {
    socket_path: String,
    bearer_token: String,
}

impl SymthaeaConfig {
    fn from_env() -> Option<Self> {
        let socket_path = std::env::var("SYMTHAEA_SOCKET_PATH").ok()?;
        let bearer_token = std::env::var("SYMTHAEA_SERVICE_BEARER_TOKEN").ok()?;
        if bearer_token.trim().is_empty() {
            return None;
        }
        Some(Self {
            socket_path,
            bearer_token,
        })
    }
}

#[derive(serde::Deserialize)]
struct SymthaeaQueryRequest {
    content: String,
}

#[derive(serde::Serialize)]
struct SymthaeaQueryResponse {
    content: String,
    confidence: f32,
    safe: bool,
    phi: f32,
    processing_time_ms: u64,
}

async fn symthaea_query_handler(
    State(cfg): State<Option<Arc<SymthaeaConfig>>>,
    axum::Json(req): axum::Json<SymthaeaQueryRequest>,
) -> impl IntoResponse {
    let Some(cfg) = cfg else {
        return (
            StatusCode::SERVICE_UNAVAILABLE,
            "Symthaea bridge not configured",
        )
            .into_response();
    };

    if req.content.trim().is_empty() {
        return (StatusCode::BAD_REQUEST, "Empty query").into_response();
    }
    if req.content.len() > SYMTHAEA_MAX_QUERY_CHARS {
        return (StatusCode::PAYLOAD_TOO_LARGE, "Query too long").into_response();
    }

    let wire_request = serde_json::json!({
        "type": "query",
        "content": req.content,
        "authorization": format!("Bearer {}", cfg.bearer_token),
    });
    let mut line = match serde_json::to_string(&wire_request) {
        Ok(s) => s,
        Err(_) => return internal_error_response().into_response(),
    };
    line.push('\n');

    // The daemon's own translation calls have been observed taking up to
    // ~160s under real host load (measured 2026-07-04) — this timeout is
    // generous relative to that, not the route-level TimeoutLayer (which
    // bounds the whole request including connect + write).
    let result: Result<String, String> = async {
        let mut stream = UnixStream::connect(&cfg.socket_path)
            .await
            .map_err(|e| format!("connect failed: {e}"))?;
        stream
            .write_all(line.as_bytes())
            .await
            .map_err(|e| format!("write failed: {e}"))?;
        let mut reader = BufReader::new(stream);
        let mut response_line = Vec::with_capacity(1024);
        loop {
            let (chunk, consumed, newline_found, too_large) = {
                let available = reader
                    .fill_buf()
                    .await
                    .map_err(|e| format!("read failed: {e}"))?;
                if available.is_empty() {
                    return Err("Symthaea response ended before its newline terminator".to_string());
                }
                let remaining_with_sentinel = MAX_SYMTHAEA_RESPONSE_LINE_BYTES
                    .saturating_sub(response_line.len())
                    .saturating_add(1);
                let inspected = available.len().min(remaining_with_sentinel);
                let candidate = &available[..inspected];
                match candidate.iter().position(|byte| *byte == b'\n') {
                    Some(index) => {
                        let consumed = index + 1;
                        (
                            candidate[..consumed].to_vec(),
                            consumed,
                            true,
                            response_line.len().saturating_add(consumed)
                                > MAX_SYMTHAEA_RESPONSE_LINE_BYTES,
                        )
                    }
                    None => (
                        candidate.to_vec(),
                        inspected,
                        false,
                        response_line.len().saturating_add(inspected)
                            > MAX_SYMTHAEA_RESPONSE_LINE_BYTES,
                    ),
                }
            };
            reader.consume(consumed);
            response_line.extend_from_slice(&chunk);
            if too_large {
                return Err("Symthaea response line exceeded its byte limit".to_string());
            }
            if newline_found {
                break;
            }
        }
        String::from_utf8(response_line)
            .map_err(|_| "Symthaea response line was not valid UTF-8".to_string())
    }
    .await;

    let response_line = match result {
        Ok(line) => line,
        Err(e) => {
            log::warn!("Symthaea bridge error: {}", e);
            return (StatusCode::BAD_GATEWAY, "Symthaea unavailable").into_response();
        }
    };

    let parsed: serde_json::Value = match serde_json::from_str(&response_line) {
        Ok(v) => v,
        Err(_) => return (StatusCode::BAD_GATEWAY, "Malformed Symthaea response").into_response(),
    };

    if parsed.get("type").and_then(|t| t.as_str()) == Some("error") {
        let msg = parsed
            .get("message")
            .and_then(|m| m.as_str())
            .unwrap_or("Symthaea error");
        log::warn!("Symthaea returned an error response: {}", msg);
        return (StatusCode::BAD_GATEWAY, "Symthaea error").into_response();
    }

    let out = SymthaeaQueryResponse {
        content: parsed
            .get("content")
            .and_then(|v| v.as_str())
            .unwrap_or("")
            .to_string(),
        confidence: parsed
            .get("confidence")
            .and_then(|v| v.as_f64())
            .unwrap_or(0.0) as f32,
        safe: symthaea_response_is_safe(&parsed),
        phi: parsed.get("phi").and_then(|v| v.as_f64()).unwrap_or(0.0) as f32,
        processing_time_ms: parsed
            .get("processing_time_ms")
            .and_then(|v| v.as_u64())
            .unwrap_or(0),
    };

    axum::Json(out).into_response()
}

async fn health() -> &'static str {
    "Prism OK"
}

// ═══════════════════════════════════════════════════════════════
// SERVER SETUP
// ═══════════════════════════════════════════════════════════════

/// Build the unified Prism router: API proxy routes + static file serving.
///
/// `symthaea_cfg` is `None` when the bridge isn't configured (tests, or a
/// deploy that hasn't set SYMTHAEA_SOCKET_PATH/SYMTHAEA_SERVICE_BEARER_TOKEN)
/// — the route itself handles that by returning 503, so this is purely an
/// explicit opt-in rather than a hard dependency.
fn build_app(dist_path: &str, symthaea_cfg: Option<Arc<SymthaeaConfig>>) -> Router {
    let fallback = ServeFile::new(format!("{}/index.html", dist_path));

    let symthaea_route = axum::Router::new()
        .route("/api/symthaea/query", post(symthaea_query_handler))
        .with_state(symthaea_cfg)
        // Scoped tighter than the app-wide ConcurrencyLimitLayer(100) below:
        // the daemon serializes all processing behind a single mutex, so
        // admitting more concurrent requests than it can usefully serve in
        // parallel just means they queue instead of the caller getting a
        // fast, clear "busy" response. LoadShedLayer converts
        // ConcurrencyLimitLayer's default queue-on-overflow into an
        // immediate rejection instead. HandleErrorLayer is required: axum's
        // Router::layer demands an Infallible service error, but
        // LoadShed/Timeout produce a boxed error — this converts it into an
        // actual response instead of failing to compile.
        .layer(symthaea_rate_limit_layer!(Duration::from_secs(240)));

    Router::new()
        // Proxy API routes (same-origin, no CORS issues)
        .route("/proxy", get(proxy_handler))
        .route("/api/ddg", get(ddg_handler))
        .route("/api/brave", get(brave_handler))
        .route("/api/perplexity", post(perplexity_handler))
        .route("/health", get(health))
        .merge(symthaea_route)
        // Explicit cap for request-body extractors (not just upstream responses).
        .layer(DefaultBodyLimit::max(MAX_REQUEST_BODY_SIZE))
        // Static files with SPA fallback
        .fallback_service(ServeDir::new(dist_path).not_found_service(fallback))
        .layer(CompressionLayer::new())
        .layer(tower::limit::ConcurrencyLimitLayer::new(100))
        .layer(SetResponseHeaderLayer::overriding(
            axum::http::header::X_CONTENT_TYPE_OPTIONS,
            HeaderValue::from_static("nosniff"),
        ))
        .layer(SetResponseHeaderLayer::overriding(
            axum::http::header::X_FRAME_OPTIONS,
            HeaderValue::from_static("DENY"),
        ))
        .layer(SetResponseHeaderLayer::overriding(
            axum::http::header::REFERRER_POLICY,
            HeaderValue::from_static("strict-origin-when-cross-origin"),
        ))
}

#[tokio::main]
async fn main() {
    env_logger::init();

    let dist_path = std::env::var("PRISM_DIST").unwrap_or_else(|_| {
        "/srv/luminous-dynamics/mycelix-workspace/mycelix-prism/prism-ui/dist".to_string()
    });

    let symthaea_cfg = SymthaeaConfig::from_env().map(Arc::new);
    if symthaea_cfg.is_none() {
        log::info!(
            "Symthaea bridge not configured (SYMTHAEA_SOCKET_PATH/SYMTHAEA_SERVICE_BEARER_TOKEN unset) — /api/symthaea/query will return 503"
        );
    }

    let app = build_app(&dist_path, symthaea_cfg);

    let addr = std::env::var("PRISM_ADDR").unwrap_or_else(|_| "0.0.0.0:8130".to_string());

    log::info!("Prism unified server on {} (dist: {})", addr, dist_path);
    println!("Prism serving on http://{}", addr);

    let listener = tokio::net::TcpListener::bind(&addr)
        .await
        .expect("Failed to bind Prism server address");
    axum::serve(listener, app)
        .await
        .expect("Prism server error");
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::body::Body;
    use axum::http::{Request, StatusCode};
    use tower::ServiceExt;

    fn test_dist_dir() -> tempfile::TempDir {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(
            dir.path().join("index.html"),
            "<html><body>Prism</body></html>",
        )
        .unwrap();
        std::fs::create_dir_all(dir.path().join("static")).unwrap();
        std::fs::write(dir.path().join("static/test.js"), "console.log('ok')").unwrap();
        dir
    }

    #[test]
    fn missing_or_malformed_safety_metadata_fails_closed() {
        assert!(!symthaea_response_is_safe(&serde_json::json!({})));
        assert!(!symthaea_response_is_safe(
            &serde_json::json!({"safe": null})
        ));
        assert!(!symthaea_response_is_safe(
            &serde_json::json!({"safe": "true"})
        ));
        assert!(!symthaea_response_is_safe(&serde_json::json!({"safe": 1})));
    }

    #[test]
    fn explicit_safety_metadata_is_preserved() {
        assert!(symthaea_response_is_safe(
            &serde_json::json!({"safe": true})
        ));
        assert!(!symthaea_response_is_safe(
            &serde_json::json!({"safe": false})
        ));
    }

    #[tokio::test]
    async fn serves_index_html() {
        let dir = test_dist_dir();
        let app = build_app(dir.path().to_str().unwrap(), None);
        let resp = app
            .oneshot(Request::get("/").body(Body::empty()).unwrap())
            .await
            .unwrap();
        assert_eq!(resp.status(), StatusCode::OK);
    }

    #[tokio::test]
    async fn spa_fallback_returns_index() {
        let dir = test_dist_dir();
        let app = build_app(dir.path().to_str().unwrap(), None);
        let resp = app
            .oneshot(
                Request::get("/nonexistent/path")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        let status = resp.status();
        assert!(
            status == StatusCode::OK || status == StatusCode::NOT_FOUND,
            "Expected 200 or 404 for SPA fallback, got {}",
            status
        );
    }

    #[tokio::test]
    async fn serves_static_files() {
        let dir = test_dist_dir();
        let app = build_app(dir.path().to_str().unwrap(), None);
        let resp = app
            .oneshot(Request::get("/static/test.js").body(Body::empty()).unwrap())
            .await
            .unwrap();
        assert_eq!(resp.status(), StatusCode::OK);
    }

    #[tokio::test]
    async fn security_headers_present() {
        let dir = test_dist_dir();
        let app = build_app(dir.path().to_str().unwrap(), None);
        let resp = app
            .oneshot(Request::get("/").body(Body::empty()).unwrap())
            .await
            .unwrap();
        assert_eq!(
            resp.headers()
                .get("x-content-type-options")
                .map(|v| v.to_str().unwrap()),
            Some("nosniff")
        );
        assert_eq!(
            resp.headers()
                .get("x-frame-options")
                .map(|v| v.to_str().unwrap()),
            Some("DENY")
        );
        assert_eq!(
            resp.headers()
                .get("referrer-policy")
                .map(|v| v.to_str().unwrap()),
            Some("strict-origin-when-cross-origin")
        );
    }

    #[tokio::test]
    async fn health_endpoint() {
        let dir = test_dist_dir();
        let app = build_app(dir.path().to_str().unwrap(), None);
        let resp = app
            .oneshot(Request::get("/health").body(Body::empty()).unwrap())
            .await
            .unwrap();
        assert_eq!(resp.status(), StatusCode::OK);
    }

    #[test]
    fn rejects_localhost() {
        assert!(validate_proxy_url("http://localhost:8080").is_err());
        assert!(validate_proxy_url("http://127.0.0.1:5432").is_err());
        assert!(validate_proxy_url("http://[::1]:80").is_err());
    }

    #[test]
    fn rejects_private_ips() {
        assert!(validate_proxy_url("http://10.0.0.1").is_err());
        assert!(validate_proxy_url("http://192.168.1.1").is_err());
    }

    #[test]
    fn allows_external_urls() {
        assert!(validate_proxy_url("https://example.com").is_ok());
        assert!(validate_proxy_url("https://en.wikipedia.org/wiki/Rust").is_ok());
    }

    // ═══════════════════════════════════════════════════════════
    // SYMTHAEA BRIDGE
    // ═══════════════════════════════════════════════════════════

    #[tokio::test]
    async fn symthaea_query_returns_503_when_not_configured() {
        let dir = test_dist_dir();
        let app = build_app(dir.path().to_str().unwrap(), None);
        let resp = app
            .oneshot(
                Request::post("/api/symthaea/query")
                    .header("content-type", "application/json")
                    .body(Body::from(r#"{"content":"hello"}"#))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(resp.status(), StatusCode::SERVICE_UNAVAILABLE);
    }

    /// Spawns a fake Unix-socket daemon implementing just enough of the
    /// wire protocol to stand in for the real symthaea service: reads one
    /// newline-terminated JSON line, responds however `respond` says to.
    fn spawn_fake_daemon(
        socket_path: std::path::PathBuf,
        respond: impl Fn(serde_json::Value) -> FakeDaemonAction + Send + 'static,
    ) {
        tokio::spawn(async move {
            let listener = tokio::net::UnixListener::bind(&socket_path).unwrap();
            loop {
                let (stream, _) = match listener.accept().await {
                    Ok(x) => x,
                    Err(_) => return,
                };
                let mut reader = BufReader::new(stream);
                let mut line = String::new();
                if reader.read_line(&mut line).await.is_err() || line.is_empty() {
                    continue;
                }
                let req: serde_json::Value = match serde_json::from_str(&line) {
                    Ok(v) => v,
                    Err(_) => continue,
                };
                match respond(req) {
                    FakeDaemonAction::Reply(mut s) => {
                        s.push('\n');
                        let _ = reader.get_mut().write_all(s.as_bytes()).await;
                    }
                    FakeDaemonAction::HangForever => {
                        tokio::time::sleep(Duration::from_secs(3600)).await;
                    }
                }
            }
        });
    }

    enum FakeDaemonAction {
        Reply(String),
        HangForever,
    }

    fn test_symthaea_cfg(socket_path: &std::path::Path) -> Option<Arc<SymthaeaConfig>> {
        Some(Arc::new(SymthaeaConfig {
            socket_path: socket_path.to_str().unwrap().to_string(),
            bearer_token: "test-token".to_string(),
        }))
    }

    #[tokio::test]
    async fn symthaea_query_round_trips_through_fake_daemon() {
        let dir = test_dist_dir();
        let socket_path = dir.path().join("fake-symthaea.sock");
        spawn_fake_daemon(socket_path.clone(), |req| {
            assert_eq!(req["type"], "query");
            assert_eq!(req["authorization"], "Bearer test-token");
            FakeDaemonAction::Reply(
                serde_json::json!({
                    "type": "response",
                    "content": "a real answer",
                    "confidence": 0.9,
                    "safe": true,
                    "phi": 0.85,
                    "phi_dyad": 1.0,
                    "steps_to_emergence": 0,
                    "processing_time_ms": 42
                })
                .to_string(),
            )
        });
        // Give the fake daemon a moment to bind before the request fires.
        tokio::time::sleep(Duration::from_millis(50)).await;

        let app = build_app(
            dir.path().to_str().unwrap(),
            test_symthaea_cfg(&socket_path),
        );
        let resp = app
            .oneshot(
                Request::post("/api/symthaea/query")
                    .header("content-type", "application/json")
                    .body(Body::from(r#"{"content":"what is X?"}"#))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(resp.status(), StatusCode::OK);
        let body = axum::body::to_bytes(resp.into_body(), usize::MAX)
            .await
            .unwrap();
        let parsed: serde_json::Value = serde_json::from_slice(&body).unwrap();
        assert_eq!(parsed["content"], "a real answer");
        assert_eq!(parsed["processing_time_ms"], 42);
    }

    #[tokio::test]
    async fn symthaea_query_rejects_empty_and_oversized_content() {
        let dir = test_dist_dir();
        let socket_path = dir.path().join("fake-symthaea-2.sock");
        spawn_fake_daemon(socket_path.clone(), |_req| {
            FakeDaemonAction::Reply(
                serde_json::json!({"type": "response", "content": "", "confidence": 0.9,
                    "safe": true, "phi": 0.5, "phi_dyad": 1.0, "steps_to_emergence": 0,
                    "processing_time_ms": 1})
                .to_string(),
            )
        });
        tokio::time::sleep(Duration::from_millis(50)).await;

        let cfg = test_symthaea_cfg(&socket_path);

        let app = build_app(dir.path().to_str().unwrap(), cfg.clone());
        let resp = app
            .oneshot(
                Request::post("/api/symthaea/query")
                    .header("content-type", "application/json")
                    .body(Body::from(r#"{"content":""}"#))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(resp.status(), StatusCode::BAD_REQUEST);

        let too_long = "x".repeat(SYMTHAEA_MAX_QUERY_CHARS + 1);
        let app = build_app(dir.path().to_str().unwrap(), cfg);
        let resp = app
            .oneshot(
                Request::post("/api/symthaea/query")
                    .header("content-type", "application/json")
                    .body(Body::from(
                        serde_json::json!({ "content": too_long }).to_string(),
                    ))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(resp.status(), StatusCode::PAYLOAD_TOO_LARGE);
    }

    #[tokio::test]
    async fn symthaea_query_times_out_on_hung_daemon() {
        let dir = test_dist_dir();
        let socket_path = dir.path().join("fake-symthaea-hang.sock");
        spawn_fake_daemon(socket_path.clone(), |_req| FakeDaemonAction::HangForever);
        tokio::time::sleep(Duration::from_millis(50)).await;

        // Rebuild with a short timeout for this test only — the production
        // 240s default would make this test take 4 minutes.
        let symthaea_route = axum::Router::new()
            .route("/api/symthaea/query", post(symthaea_query_handler))
            .with_state(test_symthaea_cfg(&socket_path))
            .layer(symthaea_rate_limit_layer!(Duration::from_millis(200)));

        let resp = symthaea_route
            .oneshot(
                Request::post("/api/symthaea/query")
                    .header("content-type", "application/json")
                    .body(Body::from(r#"{"content":"hello"}"#))
                    .unwrap(),
            )
            .await
            .unwrap();
        // TimeoutLayer maps an elapsed timeout to 503 by default in tower.
        assert_eq!(resp.status(), StatusCode::SERVICE_UNAVAILABLE);
    }
}
