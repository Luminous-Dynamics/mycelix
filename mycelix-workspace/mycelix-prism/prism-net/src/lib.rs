// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Centralized bounded HTTP authority for Prism.
//!
//! This is a containment bridge toward the staged WEB-LOCATOR / WEB-CAPTURE
//! architecture. It is deliberately narrower than a general-purpose HTTP
//! client: arbitrary web GETs must pass the shared destination policy, DNS
//! answers are classified before connection, redirects are never followed,
//! and the admitted DNS answers are pinned into reqwest.

use prism_common::ssrf::{is_private_ipv4, is_private_ipv6, validate_proxy_url};
use prism_dom::DomTree;
use reqwest::header::HeaderMap;
use std::net::{IpAddr, SocketAddr};
use std::time::Duration;
use thiserror::Error;
use url::Url;

const PRISM_USER_AGENT: &str = "Prism/0.4 (bounded-fetch; +https://mycelix.net)";
const DEFAULT_MAX_BODY_SIZE: usize = 10 * 1024 * 1024;
const MAX_HEADER_COUNT: usize = 64;
const MAX_HEADER_BYTES: usize = 16 * 1024;

#[derive(Error, Debug)]
pub enum FetchError {
    #[error("HTTP request failed: {0}")]
    Http(#[from] reqwest::Error),
    #[error("Invalid URL: {0}")]
    InvalidUrl(#[from] url::ParseError),
    #[error("URL rejected by destination policy: {0}")]
    Policy(&'static str),
    #[error("DNS resolution returned no addresses for {host}")]
    NoDnsAnswers { host: String },
    #[error("DNS resolution failed for {host}: {reason}")]
    DnsResolution { host: String, reason: String },
    #[error("DNS resolution included a prohibited address for {host}: {addr}")]
    UnsafeDnsAnswer { host: String, addr: IpAddr },
    #[error("Response headers exceed configured bounds")]
    InvalidHeaders,
    #[error("Response body exceeds configured limit: {size} bytes (max {max})")]
    TooLarge { size: usize, max: usize },
    #[error("Response body is not valid UTF-8")]
    InvalidUtf8,
}

/// A bounded response returned by SafeFetchClient.
///
/// This is transport observation, not source authenticity or epistemic truth.
#[derive(Debug)]
pub struct SafeFetchResponse {
    pub url: Url,
    pub status: u16,
    pub headers: HeaderMap,
    pub body: Vec<u8>,
}

#[derive(Clone, Debug)]
pub struct SafeFetchClient {
    timeout: Duration,
    max_body_size: usize,
}

impl SafeFetchClient {
    pub fn new() -> Self {
        Self::with_limits(Duration::from_secs(30), DEFAULT_MAX_BODY_SIZE)
    }

    pub fn with_limits(timeout: Duration, max_body_size: usize) -> Self {
        Self {
            timeout,
            max_body_size,
        }
    }

    /// Fetch one URL with no redirect following and no ambient proxy use.
    ///
    /// Domain DNS answers are resolved once here, classified as a complete
    /// set, and then pinned into reqwest. A new redirect must therefore be
    /// treated as a new fetch and a new policy decision.
    pub async fn get(&self, raw_url: &str) -> Result<SafeFetchResponse, FetchError> {
        let url = validate_proxy_url(raw_url).map_err(FetchError::Policy)?;
        self.get_url(url).await
    }

    pub async fn get_url(&self, url: Url) -> Result<SafeFetchResponse, FetchError> {
        let host = url.host_str().unwrap_or("").to_string();
        let port = url
            .port_or_known_default()
            .ok_or(FetchError::Policy("URL has no effective port"))?;

        let mut builder = reqwest::Client::builder()
            .user_agent(PRISM_USER_AGENT)
            .redirect(reqwest::redirect::Policy::none())
            .no_proxy()
            .timeout(self.timeout)
            // Do not ask the server for compressed content in this legacy
            // bridge. Encoded-vs-decoded capture is a separate WEB-CAPTURE
            // semantic and must not be silently collapsed here.
            .header(reqwest::header::ACCEPT_ENCODING, "identity");

        if let Some(ip) = url.host().and_then(|h| match h {
            url::Host::Ipv4(ip) => Some(IpAddr::V4(ip)),
            url::Host::Ipv6(ip) => Some(IpAddr::V6(ip)),
            url::Host::Domain(_) => None,
        }) {
            if is_unsafe_ip(ip) {
                return Err(FetchError::UnsafeDnsAnswer {
                    host,
                    addr: ip,
                });
            }
        } else {
            let addrs: Vec<SocketAddr> = tokio::net::lookup_host((host.as_str(), port))
                .await
                .map_err(|e| FetchError::DnsResolution {
                    host: host.clone(),
                    reason: e.to_string(),
                })?
                .collect();

            if addrs.is_empty() {
                return Err(FetchError::NoDnsAnswers { host });
            }

            for addr in &addrs {
                if is_unsafe_ip(addr.ip()) {
                    return Err(FetchError::UnsafeDnsAnswer {
                        host: host.clone(),
                        addr: addr.ip(),
                    });
                }
            }

            builder = builder.resolve_to_addrs(&host, &addrs);
        }

        let client = builder.build()?;
        let response = client.get(url.clone()).send().await?;

        validate_headers(response.headers())?;

        if let Some(len) = response.content_length() {
            if len > self.max_body_size as u64 {
                return Err(FetchError::TooLarge {
                    size: len as usize,
                    max: self.max_body_size,
                });
            }
        }

        let status = response.status().as_u16();
        let headers = response.headers().clone();
        let final_url = response.url().clone();
        let body = response.bytes().await?.to_vec();

        if body.len() > self.max_body_size {
            return Err(FetchError::TooLarge {
                size: body.len(),
                max: self.max_body_size,
            });
        }

        Ok(SafeFetchResponse {
            url: final_url,
            status,
            headers,
            body,
        })
    }

    /// Fetch and parse HTML while preserving the bounded transport metadata.
    pub async fn fetch_page(&self, raw_url: &str) -> Result<FetchedPage, FetchError> {
        let response = self.get(raw_url).await?;
        let html = String::from_utf8(response.body).map_err(|_| FetchError::InvalidUtf8)?;
        let dom = prism_dom::parse_html(&html);

        let content_type = response
            .headers
            .get("content-type")
            .and_then(|v| v.to_str().ok())
            .map(str::to_owned);

        let metadata = FetchMetadata {
            url: response.url,
            status: response.status,
            headers: response.headers,
            content_type,
            has_auth_headers: false,
            has_cookies: false,
        };

        Ok(FetchedPage {
            dom,
            html,
            metadata,
        })
    }
}

impl Default for SafeFetchClient {
    fn default() -> Self {
        Self::new()
    }
}

fn is_unsafe_ip(ip: IpAddr) -> bool {
    match ip {
        IpAddr::V4(ip) => is_private_ipv4(&ip),
        IpAddr::V6(ip) => is_private_ipv6(&ip),
    }
}

fn validate_headers(headers: &HeaderMap) -> Result<(), FetchError> {
    if headers.len() > MAX_HEADER_COUNT {
        return Err(FetchError::InvalidHeaders);
    }

    let total_bytes = headers
        .iter()
        .map(|(name, value)| name.as_str().len() + value.as_bytes().len())
        .sum::<usize>();

    if total_bytes > MAX_HEADER_BYTES {
        return Err(FetchError::InvalidHeaders);
    }

    Ok(())
}

/// Backwards-compatible Prism page client. New callers should use
/// SafeFetchClient directly so the authority boundary is visible in APIs.
#[derive(Clone, Debug)]
pub struct PrismClient {
    inner: SafeFetchClient,
}

impl PrismClient {
    pub fn new() -> Self {
        Self {
            inner: SafeFetchClient::new(),
        }
    }

    pub fn with_max_body_size(max_body_size: u64) -> Self {
        Self {
            inner: SafeFetchClient::with_limits(
                Duration::from_secs(30),
                max_body_size.min(usize::MAX as u64) as usize,
            ),
        }
    }

    pub async fn fetch(&self, url: &str) -> Result<FetchedPage, FetchError> {
        self.inner.fetch_page(url).await
    }
}

impl Default for PrismClient {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone)]
pub struct FetchMetadata {
    pub url: Url,
    pub status: u16,
    pub headers: HeaderMap,
    pub content_type: Option<String>,
    pub has_auth_headers: bool,
    pub has_cookies: bool,
}

#[derive(Debug)]
pub struct FetchedPage {
    pub dom: DomTree,
    pub html: String,
    pub metadata: FetchMetadata,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn safe_ip_classification() {
        assert!(is_unsafe_ip("127.0.0.1".parse().unwrap()));
        assert!(is_unsafe_ip("10.0.0.1".parse().unwrap()));
        assert!(is_unsafe_ip("169.254.169.254".parse().unwrap()));
        assert!(is_unsafe_ip("::1".parse().unwrap()));
        assert!(is_unsafe_ip("fc00::1".parse().unwrap()));
        assert!(!is_unsafe_ip("1.1.1.1".parse().unwrap()));
        assert!(!is_unsafe_ip("2606:4700:4700::1111".parse().unwrap()));
    }

    #[test]
    fn header_bounds_are_bounded() {
        let mut headers = HeaderMap::new();
        headers.insert("content-type", "text/html".parse().unwrap());
        assert!(validate_headers(&headers).is_ok());
    }

    #[test]
    fn policy_rejects_credentials_before_network() {
        assert!(validate_proxy_url("https://user:secret@example.com").is_err());
    }

    #[test]
    fn default_client_has_bounded_body_limit() {
        assert_eq!(SafeFetchClient::new().max_body_size, DEFAULT_MAX_BODY_SIZE);
    }
}
