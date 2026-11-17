// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! CORS proxy for Prism.
//!
//! Fetches external URLs on behalf of the WASM app and returns
//! responses with permissive CORS headers.
//!
//! Usage: cargo run -p prism-proxy
//! Listens on http://127.0.0.1:8131
//!
//! WASM app requests: GET /proxy?url=https://example.com

use axum::{Router, extract::{DefaultBodyLimit, Query}, http::{StatusCode, header::HeaderValue}, response::IntoResponse, routing::get};
use prism_common::ssrf::validate_proxy_url;
use prism_egress::{EgressError, PublicEgressClient};
use reqwest::{Method, header::HeaderMap};
use url::Url;
use tower_http::cors::{AllowOrigin, CorsLayer};

const MAX_REQUEST_BODY_SIZE: usize = 2 * 1024 * 1024; // 2 MiB

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

fn upstream_error(context: &str, error: EgressError) -> axum::response::Response<axum::body::Body> {
    log::warn!("Prism egress {context} failed: {error}");
    (StatusCode::BAD_GATEWAY, "Upstream request failed").into_response()
}

fn internal_error_response() -> axum::response::Response<axum::body::Body> {
    let mut resp = axum::response::Response::new(axum::body::Body::from("Internal error"));
    *resp.status_mut() = StatusCode::INTERNAL_SERVER_ERROR;
    resp
}

fn json_response(
    status: u16,
    body: axum::body::Bytes,
) -> axum::response::Response<axum::body::Body> {
    axum::response::Response::builder()
        .status(status)
        .header("content-type", "application/json")
        .body(axum::body::Body::from(body))
        .unwrap_or_else(|_| internal_error_response())
}

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

    match egress_request(Method::GET, validated.as_str(), HeaderMap::new(), Vec::new()).await {
        Ok(response) => {
            let content_type = response
                .headers
                .get(reqwest::header::CONTENT_TYPE)
                .cloned()
                .unwrap_or_else(|| HeaderValue::from_static("text/html"));
            axum::response::Response::builder()
                .status(response.status.as_u16())
                .header(axum::http::header::CONTENT_TYPE, content_type)
                .header("x-prism-proxied", "true")
                .body(axum::body::Body::from(response.body))
                .unwrap_or_else(|_| internal_error_response())
                .into_response()
        }
        Err(error) => upstream_error("proxy", error).into_response(),
    }
}

/// DuckDuckGo Instant Answer proxy — bypasses CORS.
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
        Ok(response) => json_response(response.status.as_u16(), response.body).into_response(),
        Err(error) => upstream_error("DuckDuckGo", error).into_response(),
    }
}

/// Brave Search proxy — reads API key from X-Brave-Key header, forwards as auth.
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
    outbound_headers.insert(reqwest::header::ACCEPT, HeaderValue::from_static("application/json"));

    match egress_request(Method::GET, url.as_str(), outbound_headers, Vec::new()).await {
        Ok(response) => json_response(response.status.as_u16(), response.body).into_response(),
        Err(error) => upstream_error("Brave", error).into_response(),
    }
}

/// Perplexity proxy — reads API key from custom header, forwards as Bearer token.
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
        Err(_) => return (StatusCode::BAD_REQUEST, "Invalid Perplexity API key header").into_response(),
    };
    let mut outbound_headers = HeaderMap::new();
    outbound_headers.insert(reqwest::header::AUTHORIZATION, authorization);
    outbound_headers.insert(reqwest::header::CONTENT_TYPE, HeaderValue::from_static("application/json"));

    match egress_request(
        Method::POST,
        "https://api.perplexity.ai/chat/completions",
        outbound_headers,
        body.to_vec(),
    )
    .await
    {
        Ok(response) => json_response(response.status.as_u16(), response.body).into_response(),
        Err(error) => upstream_error("Perplexity", error).into_response(),
    }
}

async fn health() -> &'static str {
    "Prism proxy OK"
}

#[tokio::main]
async fn main() {
    env_logger::init();

    let allowed_origins = AllowOrigin::list([
        "http://localhost:8130"
            .parse()
            .expect("hardcoded CORS origin literal is malformed"),
        "https://prism.mycelix.net"
            .parse()
            .expect("hardcoded CORS origin literal is malformed"),
        "https://prism.luminousdynamics.io"
            .parse()
            .expect("hardcoded CORS origin literal is malformed"),
        "https://app.mycelix.net"
            .parse()
            .expect("hardcoded CORS origin literal is malformed"),
    ]);

    let cors = CorsLayer::new()
        .allow_origin(allowed_origins)
        .allow_methods([axum::http::Method::GET, axum::http::Method::POST])
        .allow_headers(tower_http::cors::Any);

    let app = Router::new()
        .route("/proxy", get(proxy_handler))
        .route("/api/ddg", get(ddg_handler))
        .route("/api/brave", get(brave_handler))
        .route("/api/perplexity", axum::routing::post(perplexity_handler))
        .route("/health", get(health))
        .layer(cors)
        .layer(DefaultBodyLimit::max(MAX_REQUEST_BODY_SIZE))
        .layer(tower::limit::ConcurrencyLimitLayer::new(50));

    let addr = "127.0.0.1:8131";
    log::info!("Prism proxy listening on http://{}", addr);
    println!("Prism CORS proxy on http://{}", addr);

    let listener = tokio::net::TcpListener::bind(addr)
        .await
        .expect("Failed to bind port 8131");
    axum::serve(listener, app).await.expect("Server error");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_localhost() {
        assert!(validate_proxy_url("http://localhost:8080").is_err());
        assert!(validate_proxy_url("http://127.0.0.1:5432").is_err());
        assert!(validate_proxy_url("http://[::1]:80").is_err());
        assert!(validate_proxy_url("http://0.0.0.0").is_err());
    }

    #[test]
    fn rejects_private_ips() {
        assert!(validate_proxy_url("http://10.0.0.1").is_err());
        assert!(validate_proxy_url("http://172.16.0.1").is_err());
        assert!(validate_proxy_url("http://192.168.1.1").is_err());
    }

    #[test]
    fn rejects_cloud_metadata() {
        assert!(validate_proxy_url("http://169.254.169.254/latest/meta-data/").is_err());
        assert!(validate_proxy_url("http://metadata.google.internal").is_err());
    }

    #[test]
    fn rejects_non_http_schemes() {
        assert!(validate_proxy_url("file:///etc/passwd").is_err());
        assert!(validate_proxy_url("ftp://example.com").is_err());
        assert!(validate_proxy_url("gopher://evil.com").is_err());
    }

    #[test]
    fn rejects_ipv4_mapped_ipv6() {
        // ::ffff:127.0.0.1 — loopback disguised as IPv6
        assert!(validate_proxy_url("http://[::ffff:127.0.0.1]").is_err());
        // ::ffff:192.168.1.1 — private IP disguised as IPv6
        assert!(validate_proxy_url("http://[::ffff:192.168.1.1]").is_err());
        // ::ffff:169.254.169.254 — cloud metadata disguised as IPv6
        assert!(validate_proxy_url("http://[::ffff:169.254.169.254]").is_err());
        // ::ffff:10.0.0.1 — private range disguised as IPv6
        assert!(validate_proxy_url("http://[::ffff:10.0.0.1]").is_err());
    }

    #[test]
    fn rejects_ipv6_link_local_and_ula() {
        // fe80:: — link-local
        assert!(validate_proxy_url("http://[fe80::1]").is_err());
        // fc00:: — unique-local address
        assert!(validate_proxy_url("http://[fc00::1]").is_err());
        // fd00:: — also unique-local
        assert!(validate_proxy_url("http://[fd12::1]").is_err());
    }

    #[test]
    fn allows_valid_external_urls() {
        assert!(validate_proxy_url("https://example.com").is_ok());
        assert!(validate_proxy_url("https://api.duckduckgo.com/?q=test").is_ok());
        assert!(validate_proxy_url("http://en.wikipedia.org/wiki/Rust").is_ok());
    }
}
