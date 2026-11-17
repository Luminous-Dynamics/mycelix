// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! HTTP networking for Prism.
//!
//! Fetches web pages via `reqwest`, handles decompression, and parses them
//! into `DomTree` via `prism-dom`.

use prism_dom::DomTree;
use reqwest::header::HeaderMap;
use thiserror::Error;
use url::Url;

const PRISM_USER_AGENT: &str = "Prism/0.1 (pure Rust; +https://luminousdynamics.org)";
const MAX_NAVIGATION_URL_BYTES: usize = 8 * 1024;

#[derive(Error, Debug)]
pub enum FetchError {
    #[error("HTTP request failed: {0}")]
    Http(#[from] reqwest::Error),
    #[error("Invalid URL: {0}")]
    InvalidUrl(#[from] url::ParseError),
    #[error("Response too large: {size} bytes (max {max})")]
    TooLarge { size: u64, max: u64 },
    #[error("only HTTP and HTTPS navigation is supported")]
    UnsupportedScheme,
    #[error("URL credentials are not permitted")]
    CredentialsNotPermitted,
    #[error("URL exceeds the configured byte limit")]
    UrlTooLarge,
}

/// Metadata about a fetched page, used by the reflex arc and privacy classifier.
#[derive(Debug, Clone)]
pub struct FetchMetadata {
    /// Final URL after redirects.
    pub url: Url,
    /// HTTP status code.
    pub status: u16,
    /// Response headers.
    pub headers: HeaderMap,
    /// Content-Type header value.
    pub content_type: Option<String>,
    /// Whether the response had authentication-related headers.
    pub has_auth_headers: bool,
    /// Whether cookies were present.
    pub has_cookies: bool,
}

/// Result of fetching and parsing a page.
#[derive(Debug)]
pub struct FetchedPage {
    /// Parsed DOM tree.
    pub dom: DomTree,
    /// Raw HTML body.
    pub html: String,
    /// Fetch metadata for reflex arc / privacy classification.
    pub metadata: FetchMetadata,
}

/// User-directed desktop navigation client.
///
/// This client intentionally retains browser-like access to local/LAN targets
/// and bounded redirects because it is used by the interactive Prism shell.
/// It is not a hosted proxy or background-fetch transport; those callers must
/// use the PublicEgressClient in prism-egress instead.
pub struct PrismClient {
    client: reqwest::Client,
    max_body_size: u64,
}

impl PrismClient {
    /// Create a new client with default settings.
    pub fn new() -> Self {
        Self::with_max_body_size(10 * 1024 * 1024) // 10MB max
    }

    /// Create a new client with a specified maximum body size.
    pub fn with_max_body_size(max_body_size: u64) -> Self {
        let client = reqwest::Client::builder()
            .user_agent(PRISM_USER_AGENT)
            .gzip(true)
            .brotli(true)
            .redirect(reqwest::redirect::Policy::limited(10))
            .timeout(std::time::Duration::from_secs(30))
            .build()
            .expect("failed to build HTTP client");

        Self {
            client,
            max_body_size,
        }
    }

    /// Fetch one user-initiated navigation target and parse its bounded body.
    pub async fn fetch(&self, url: &str) -> Result<FetchedPage, FetchError> {
        let parsed_url = validate_navigation_url(url)?;
        log::info!(
            "Fetching user navigation host={}",
            parsed_url.host_str().unwrap_or("<missing>")
        );

        let response = self.client.get(parsed_url).send().await?;

        let status = response.status().as_u16();
        let headers = response.headers().clone();

        // Check for auth/cookie signals (used by privacy classifier)
        let has_auth_headers = headers.contains_key("www-authenticate")
            || headers.contains_key("authorization")
            || headers.contains_key("x-csrf-token");
        let has_cookies = headers.contains_key("set-cookie");

        let content_type = headers
            .get("content-type")
            .and_then(|v| v.to_str().ok())
            .map(|s| s.to_string());

        // Content-Length is only an early rejection; the chunk loop is
        // authoritative for missing, false, or compressed lengths.
        if let Some(len) = response.content_length()
            && len > self.max_body_size
        {
            return Err(FetchError::TooLarge {
                size: len,
                max: self.max_body_size,
            });
        }

        let final_url = response.url().clone();
        let mut response = response;
        let mut bytes = Vec::new();
        while let Some(chunk) = response.chunk().await? {
            let next_size = (bytes.len() as u64).saturating_add(chunk.len() as u64);
            if next_size > self.max_body_size {
                return Err(FetchError::TooLarge {
                    size: next_size,
                    max: self.max_body_size,
                });
            }
            bytes.extend_from_slice(&chunk);
        }
        let html = decode_http_text(&bytes, content_type.as_deref());

        // Parse HTML into DOM
        let dom = prism_dom::parse_html(&html);

        let metadata = FetchMetadata {
            url: final_url,
            status,
            headers,
            content_type,
            has_auth_headers,
            has_cookies,
        };

        Ok(FetchedPage {
            dom,
            html,
            metadata,
        })
    }
}

fn decode_http_text(bytes: &[u8], content_type: Option<&str>) -> String {
    let charset = content_type.and_then(|content_type| {
        content_type.split(';').skip(1).find_map(|parameter| {
            let (name, value) = parameter.trim().split_once('=')?;
            if name.trim().eq_ignore_ascii_case("charset") {
                Some(value.trim().trim_matches('"').trim_matches('\''))
            } else {
                None
            }
        })
    });
    let encoding = charset
        .and_then(|label| encoding_rs::Encoding::for_label(label.as_bytes()))
        .unwrap_or(encoding_rs::UTF_8);
    let (decoded, _, _) = encoding.decode(bytes);
    decoded.into_owned()
}

fn validate_navigation_url(raw: &str) -> Result<Url, FetchError> {
    if raw.len() > MAX_NAVIGATION_URL_BYTES {
        return Err(FetchError::UrlTooLarge);
    }
    let parsed = Url::parse(raw)?;
    if !matches!(parsed.scheme(), "http" | "https") {
        return Err(FetchError::UnsupportedScheme);
    }
    if !parsed.username().is_empty() || parsed.password().is_some() {
        return Err(FetchError::CredentialsNotPermitted);
    }
    Ok(parsed)
}

impl Default for PrismClient {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bounded_navigation_preserves_declared_legacy_charsets() {
        assert_eq!(
            decode_http_text(&[0xE9], Some("text/html; charset=windows-1252")),
            "é"
        );
        assert_eq!(decode_http_text(b"hello", None), "hello");
    }

    #[test]
    fn navigation_policy_allows_explicit_local_navigation_but_only_http_schemes() {
        assert!(validate_navigation_url("http://127.0.0.1:8080/").is_ok());
        assert!(validate_navigation_url("https://example.com/").is_ok());
        assert!(matches!(
            validate_navigation_url("file:///etc/passwd"),
            Err(FetchError::UnsupportedScheme)
        ));
        assert!(matches!(
            validate_navigation_url("https://user:pass@example.com/"),
            Err(FetchError::CredentialsNotPermitted)
        ));
    }

    #[test]
    fn navigation_policy_bounds_url_length() {
        let raw = format!(
            "https://example.com/{}",
            "a".repeat(MAX_NAVIGATION_URL_BYTES)
        );
        assert!(matches!(
            validate_navigation_url(&raw),
            Err(FetchError::UrlTooLarge)
        ));
    }

    #[test]
    fn client_creates_successfully() {
        let _client = PrismClient::new();
    }

    #[tokio::test]
    async fn fetch_example_dot_com() {
        let client = PrismClient::new();
        // This test requires network access
        match client.fetch("https://example.com").await {
            Ok(page) => {
                assert_eq!(page.metadata.status, 200);
                assert!(!page.dom.is_empty());
                let text = page.dom.extract_visible_text();
                assert!(text.contains("Example Domain"));
                let title = page.dom.title();
                assert_eq!(title, Some("Example Domain".to_string()));
            }
            Err(e) => {
                // Network tests may fail in CI; that's OK
                eprintln!("Network test skipped: {}", e);
            }
        }
    }
}
