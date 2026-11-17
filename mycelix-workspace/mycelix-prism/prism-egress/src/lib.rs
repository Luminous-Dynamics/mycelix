// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Native, public-web-only egress for Prism.
//!
//! Security contract:
//! - Accept only HTTP on port 80 and HTTPS on port 443.
//! - Resolve a hostname once, reject the complete answer set if any address is
//!   not public unicast, and pin reqwest to the approved addresses.
//! - Build a fresh client per request so a connection pool cannot carry a
//!   previous request's DNS authorization into another request.
//! - Disable redirects and ambient proxy discovery. Callers must explicitly
//!   authorize each redirect as a new request.
//! - Bound URL, request headers, request body, response body, connect time,
//!   and total request time.
//!
//! This crate is intentionally native-only. Do not add it as a dependency of
//! prism-common, which is also consumed by WASM targets.

use reqwest::header::HeaderMap;
use reqwest::{Client, Method, StatusCode};
use std::future::Future;
use std::io;
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr};
use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;
use thiserror::Error;
use tokio::sync::{Semaphore, SemaphorePermit};
use url::{Host, Url};

const MAX_CONCURRENT_EGRESS_REQUESTS: usize = 64;
static EGRESS_SEMAPHORE: Semaphore = Semaphore::const_new(MAX_CONCURRENT_EGRESS_REQUESTS);

#[derive(Clone, Debug)]
pub struct EgressLimits {
    pub max_url_bytes: usize,
    pub max_request_header_bytes: usize,
    pub max_request_body_bytes: usize,
    pub max_response_body_bytes: usize,
    pub connect_timeout: Duration,
    pub request_timeout: Duration,
    pub capability_ttl: Duration,
}

impl Default for EgressLimits {
    fn default() -> Self {
        Self {
            max_url_bytes: 8 * 1024,
            max_request_header_bytes: 16 * 1024,
            max_request_body_bytes: 2 * 1024 * 1024,
            max_response_body_bytes: 10 * 1024 * 1024,
            connect_timeout: Duration::from_secs(5),
            request_timeout: Duration::from_secs(30),
            capability_ttl: Duration::from_secs(5),
        }
    }
}

#[derive(Debug, Error)]
pub enum EgressError {
    #[error("URL exceeds the configured byte limit")]
    UrlTooLarge,
    #[error("invalid URL: {0}")]
    InvalidUrl(#[from] url::ParseError),
    #[error("only HTTP and HTTPS URLs are permitted")]
    UnsupportedScheme,
    #[error("URL credentials are not permitted")]
    CredentialsNotPermitted,
    #[error("local or metadata hostnames are not permitted")]
    BlockedHost,
    #[error("only HTTP port 80 and HTTPS port 443 are permitted")]
    PortNotPermitted,
    #[error("URL has no host")]
    MissingHost,
    #[error("DNS resolution failed for {host}: {source}")]
    DnsResolution {
        host: String,
        #[source]
        source: std::io::Error,
    },
    #[error("DNS returned no addresses for {0}")]
    EmptyAddressSet(String),
    #[error("destination address is not permitted: {0}")]
    AddressNotPermitted(SocketAddr),
    #[error("only GET and POST methods are permitted")]
    MethodNotPermitted,
    #[error("request headers exceed the configured byte limit")]
    RequestHeadersTooLarge,
    #[error("request contains a transport-controlled header: {0}")]
    ForbiddenHeader(String),
    #[error("request payloads and custom headers require HTTPS")]
    RequestRequiresTls,
    #[error("request body exceeds the configured byte limit")]
    RequestBodyTooLarge,
    #[error("response body exceeds the configured byte limit")]
    ResponseBodyTooLarge,
    #[error("response headers exceed the configured byte limit")]
    ResponseHeadersTooLarge,
    #[error("egress concurrency queue exceeded its wait limit")]
    ConcurrencyQueueTimeout,
    #[error("egress concurrency semaphore is unavailable")]
    ConcurrencyUnavailable,
    #[error("egress capability expired before execution")]
    CapabilityExpired,
    #[error("request does not fit the selected egress authority")]
    AuthorityScopeViolation,
    #[error("request header is not permitted for the selected egress authority: {0}")]
    AuthorityHeaderViolation(String),
    #[error("required provider authorization header is missing or malformed")]
    ProviderCredentialMissing,
    #[error("HTTP transport failed: {0}")]
    Http(#[from] reqwest::Error),
}

/// Typed outbound authority. Provider variants are intentionally fixed to
/// reviewed service origins; callers cannot mint an arbitrary credential scope.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EgressAuthority {
    /// Unauthenticated, bodyless public-web GET only.
    PublicWeb,
    /// Brave Search API: api.search.brave.com/res/v1/web/search.
    BraveSearch,
    /// Perplexity API: api.perplexity.ai/chat/completions.
    Perplexity,
}

impl EgressAuthority {
    fn label(self) -> &'static str {
        match self {
            Self::PublicWeb => "public-web",
            Self::BraveSearch => "brave-search",
            Self::Perplexity => "perplexity",
        }
    }
}

/// A one-use, short-lived authorization to fetch one exact URL with one method
/// and one already-validated destination address set.
///
/// The fields are private and the value is not Clone. Execution consumes it,
/// making accidental reuse or widening harder at the API boundary.
pub struct EgressCapability {
    url: Url,
    host: String,
    authority: EgressAuthority,
    method: Method,
    headers: HeaderMap,
    body: Vec<u8>,
    addresses: Vec<SocketAddr>,
    is_domain: bool,
    expires_at: std::time::Instant,
    max_response_body_bytes: usize,
    connect_timeout: Duration,
    request_timeout: Duration,
    policy_version: &'static str,
}

/// A bounded response plus a privacy-minimized execution receipt.
#[derive(Debug)]
pub struct EgressResponse {
    pub status: StatusCode,
    pub headers: HeaderMap,
    pub body: Vec<u8>,
    pub receipt: EgressReceipt,
}

#[derive(Clone, Debug)]
pub struct EgressReceipt {
    /// Scheme + authority only. Path, query, credentials, and response content
    /// are deliberately excluded from the receipt.
    pub origin: String,
    pub method: String,
    pub authorized_addresses: Vec<SocketAddr>,
    pub status: u16,
    pub request_body_bytes: usize,
    pub response_body_bytes: usize,
    pub elapsed: Duration,
    pub policy_version: &'static str,
    pub authority: EgressAuthority,
}

/// Resolver interface makes DNS policy deterministic under test. Implementations
/// must return the complete A and AAAA answer set, not only the first address.
pub trait EgressResolver: Send + Sync {
    fn resolve<'a>(
        &'a self,
        host: &'a str,
        port: u16,
    ) -> Pin<Box<dyn Future<Output = io::Result<Vec<SocketAddr>>> + Send + 'a>>;
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SystemEgressResolver;

impl EgressResolver for SystemEgressResolver {
    fn resolve<'a>(
        &'a self,
        host: &'a str,
        port: u16,
    ) -> Pin<Box<dyn Future<Output = io::Result<Vec<SocketAddr>>> + Send + 'a>> {
        Box::pin(async move {
            tokio::net::lookup_host((host, port))
                .await
                .map(|addresses| addresses.collect())
        })
    }
}

/// Public-web-only HTTP client. Each consumed capability receives a fresh,
/// DNS-pinned reqwest client to prevent cross-request pool reuse.
#[derive(Clone)]
pub struct PublicEgressClient {
    limits: EgressLimits,
    resolver: Arc<dyn EgressResolver>,
}

impl std::fmt::Debug for PublicEgressClient {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("PublicEgressClient")
            .field("limits", &self.limits)
            .field("resolver", &"EgressResolver")
            .finish()
    }
}

impl Default for PublicEgressClient {
    fn default() -> Self {
        Self::new(EgressLimits::default())
    }
}

impl PublicEgressClient {
    pub fn new(limits: EgressLimits) -> Self {
        Self::with_resolver(limits, SystemEgressResolver)
    }

    pub fn with_resolver<R>(limits: EgressLimits, resolver: R) -> Self
    where
        R: EgressResolver + 'static,
    {
        Self {
            limits,
            resolver: Arc::new(resolver),
        }
    }

    pub fn limits(&self) -> &EgressLimits {
        &self.limits
    }

    /// Resolve and authorize an unauthenticated public-web request. This
    /// default context accepts only bodyless, headerless GET requests.
    pub async fn authorize(
        &self,
        method: Method,
        raw_url: &str,
        headers: HeaderMap,
        body: Vec<u8>,
    ) -> Result<EgressCapability, EgressError> {
        self.authorize_with_authority(
            EgressAuthority::PublicWeb,
            method,
            raw_url,
            headers,
            body,
        )
        .await
    }

    /// Resolve and authorize an exact request under a fixed, typed authority.
    pub async fn authorize_with_authority(
        &self,
        authority: EgressAuthority,
        method: Method,
        raw_url: &str,
        headers: HeaderMap,
        body: Vec<u8>,
    ) -> Result<EgressCapability, EgressError> {
        if raw_url.len() > self.limits.max_url_bytes {
            return Err(EgressError::UrlTooLarge);
        }
        if method != Method::GET && method != Method::POST {
            return Err(EgressError::MethodNotPermitted);
        }
        if body.len() > self.limits.max_request_body_bytes {
            return Err(EgressError::RequestBodyTooLarge);
        }
        validate_request_headers(&headers)?;
        if request_header_bytes(&headers) > self.limits.max_request_header_bytes {
            return Err(EgressError::RequestHeadersTooLarge);
        }

        let mut url = Url::parse(raw_url)?;
        validate_url_policy(&url)?;
        validate_authority_scope(authority, &url, &method, &headers, &body)?;
        // URL fragments are client-side state and must not affect the
        // authorization identity or be sent as part of the HTTP request.
        url.set_fragment(None);

        let host = url.host_str().ok_or(EgressError::MissingHost)?.to_owned();
        let port = url.port_or_known_default().ok_or(EgressError::PortNotPermitted)?;
        let (addresses, is_domain) =
            resolve_and_authorize(&url, port, self.resolver.as_ref()).await?;

        Ok(EgressCapability {
            url,
            host,
            authority,
            method,
            headers,
            body,
            addresses,
            is_domain,
            expires_at: std::time::Instant::now() + self.limits.capability_ttl,
            max_response_body_bytes: self.limits.max_response_body_bytes,
            connect_timeout: self.limits.connect_timeout,
            request_timeout: self.limits.request_timeout,
            policy_version: "prism-public-egress-v1",
        })
    }

    /// Consume one authorization and execute the exact request it contains.
    pub async fn execute(
        &self,
        capability: EgressCapability,
    ) -> Result<EgressResponse, EgressError> {
        if std::time::Instant::now() >= capability.expires_at {
            return Err(EgressError::CapabilityExpired);
        }

        let origin = format!("{}://{}", capability.url.scheme(), capability.host);
        let authority = capability.authority;
        let authorized_addresses = capability.addresses.clone();
        let method_name = capability.method.as_str().to_owned();
        let request_body_bytes = capability.body.len();
        let started = std::time::Instant::now();
        let _permit: SemaphorePermit<'static> = tokio::time::timeout(
            Duration::from_secs(2),
            EGRESS_SEMAPHORE.acquire(),
        )
        .await
        .map_err(|_| EgressError::ConcurrencyQueueTimeout)?
        .map_err(|_| EgressError::ConcurrencyUnavailable)?;

        // Queueing consumes capability lifetime. Re-check after admission so
        // an expired authorization cannot connect merely because it waited.
        if std::time::Instant::now() >= capability.expires_at {
            return Err(EgressError::CapabilityExpired);
        }

        let client = build_pinned_client(
            &capability.host,
            &capability.addresses,
            capability.is_domain,
            capability.connect_timeout,
            capability.request_timeout,
        )?;

        let response = client
            .request(capability.method, capability.url)
            .headers(capability.headers)
            .body(capability.body)
            .send()
            .await?;

        let status = response.status();
        let response_headers = response.headers().clone();
        if response_header_bytes(&response_headers) > 64 * 1024
            || response_headers.len() > 128
        {
            return Err(EgressError::ResponseHeadersTooLarge);
        }
        let mut response = response;
        let mut collected = Vec::new();

        while let Some(chunk) = response.chunk().await? {
            if collected.len().saturating_add(chunk.len())
                > capability.max_response_body_bytes
            {
                return Err(EgressError::ResponseBodyTooLarge);
            }
            collected.extend_from_slice(&chunk);
        }

        let receipt = EgressReceipt {
            origin,
            method: method_name,
            authorized_addresses,
            status: status.as_u16(),
            request_body_bytes,
            response_body_bytes: collected.len(),
            elapsed: started.elapsed(),
            policy_version: capability.policy_version,
            authority,
        };

        Ok(EgressResponse {
            status,
            headers: response_headers,
            body: collected,
            receipt,
        })
    }

    /// Execute a request under a reviewed provider authority.
    pub async fn request_with_authority(
        &self,
        authority: EgressAuthority,
        method: Method,
        raw_url: &str,
        headers: HeaderMap,
        body: Vec<u8>,
    ) -> Result<EgressResponse, EgressError> {
        let capability = self
            .authorize_with_authority(authority, method, raw_url, headers, body)
            .await?;
        self.execute(capability).await
    }

    /// Convenience wrapper; internally still issues then consumes a one-use
    /// authorization before any socket request is made.
    pub async fn request(
        &self,
        method: Method,
        raw_url: &str,
        headers: HeaderMap,
        body: Vec<u8>,
    ) -> Result<EgressResponse, EgressError> {
        let capability = self.authorize(method, raw_url, headers, body).await?;
        self.execute(capability).await
    }
}

fn build_pinned_client(
    host: &str,
    addresses: &[SocketAddr],
    is_domain: bool,
    connect_timeout: Duration,
    request_timeout: Duration,
) -> Result<Client, EgressError> {
    // Each request gets a fresh pool. The URL retains the original hostname
    // for Host/SNI/certificate checks while this explicit resolver override
    // constrains socket destinations to the authorized set.
    let mut builder = Client::builder()
        .user_agent("Prism/0.4 (capability-bound egress)")
        .redirect(reqwest::redirect::Policy::none())
        .no_proxy()
        .connect_timeout(connect_timeout)
        .timeout(request_timeout);
    if is_domain {
        builder = builder.resolve_to_addrs(host, addresses);
    }
    Ok(builder.build()?)
}

fn validate_authority_scope(
    authority: EgressAuthority,
    url: &Url,
    method: &Method,
    headers: &HeaderMap,
    body: &[u8],
) -> Result<(), EgressError> {
    let is_https = url.scheme() == "https";
    let host = url.host_str().unwrap_or_default();
    let path = url.path();

    match authority {
        EgressAuthority::PublicWeb => {
            if *method != Method::GET || !headers.is_empty() || !body.is_empty() {
                return Err(EgressError::AuthorityScopeViolation);
            }
        }
        EgressAuthority::BraveSearch => {
            if !is_https
                || host != "api.search.brave.com"
                || path != "/res/v1/web/search"
                || *method != Method::GET
                || !body.is_empty()
            {
                return Err(EgressError::AuthorityScopeViolation);
            }
            for name in headers.keys() {
                if !matches!(name.as_str(), "x-subscription-token" | "accept") {
                    return Err(EgressError::AuthorityHeaderViolation(name.as_str().to_owned()));
                }
            }
            let token = headers
                .get("x-subscription-token")
                .and_then(|value| value.to_str().ok())
                .filter(|value| !value.trim().is_empty());
            if token.is_none() {
                return Err(EgressError::ProviderCredentialMissing);
            }
        }
        EgressAuthority::Perplexity => {
            if !is_https
                || host != "api.perplexity.ai"
                || path != "/chat/completions"
                || *method != Method::POST
            {
                return Err(EgressError::AuthorityScopeViolation);
            }
            for name in headers.keys() {
                if !matches!(name.as_str(), "authorization" | "content-type") {
                    return Err(EgressError::AuthorityHeaderViolation(name.as_str().to_owned()));
                }
            }
            let authorization = headers
                .get(reqwest::header::AUTHORIZATION)
                .and_then(|value| value.to_str().ok())
                .filter(|value| value.starts_with("Bearer ") && value.len() > "Bearer ".len());
            let content_type = headers
                .get(reqwest::header::CONTENT_TYPE)
                .and_then(|value| value.to_str().ok())
                .filter(|value| value.eq_ignore_ascii_case("application/json"));
            if authorization.is_none() || content_type.is_none() {
                return Err(EgressError::ProviderCredentialMissing);
            }
        }
    }

    Ok(())
}

fn validate_request_headers(headers: &HeaderMap) -> Result<(), EgressError> {
    const FORBIDDEN: &[&str] = &[
        "host",
        "connection",
        "content-length",
        "transfer-encoding",
        "proxy-authorization",
        "proxy-connection",
        "te",
        "trailer",
        "upgrade",
    ];
    for name in headers.keys() {
        if FORBIDDEN.contains(&name.as_str()) || name.as_str().starts_with("proxy-") {
            return Err(EgressError::ForbiddenHeader(name.as_str().to_owned()));
        }
    }
    Ok(())
}

fn response_header_bytes(headers: &HeaderMap) -> usize {
    headers.iter().fold(0usize, |total, (name, value)| {
        total
            .saturating_add(name.as_str().len())
            .saturating_add(value.as_bytes().len())
            .saturating_add(4)
    })
}

fn request_header_bytes(headers: &HeaderMap) -> usize {
    headers.iter().fold(0usize, |total, (name, value)| {
        total
            .saturating_add(name.as_str().len())
            .saturating_add(value.as_bytes().len())
            .saturating_add(4) // conservative ": " + CRLF accounting
    })
}

fn validate_url_policy(url: &Url) -> Result<(), EgressError> {
    match url.scheme() {
        "http" if url.port_or_known_default() == Some(80) => {}
        "https" if url.port_or_known_default() == Some(443) => {}
        "http" | "https" => return Err(EgressError::PortNotPermitted),
        _ => return Err(EgressError::UnsupportedScheme),
    }
    if !url.username().is_empty() || url.password().is_some() {
        return Err(EgressError::CredentialsNotPermitted);
    }
    let host = url.host().ok_or(EgressError::MissingHost)?;
    if let Host::Domain(domain) = host {
        let normalized = domain.trim_end_matches('.').to_ascii_lowercase();
        if normalized == "localhost"
            || normalized.ends_with(".localhost")
            || normalized.ends_with(".local")
            || normalized.ends_with(".internal")
            || normalized == "metadata.google.internal"
        {
            return Err(EgressError::BlockedHost);
        }
    }
    Ok(())
}

async fn resolve_and_authorize(
    url: &Url,
    port: u16,
    resolver: &dyn EgressResolver,
) -> Result<(Vec<SocketAddr>, bool), EgressError> {
    let host = url.host().ok_or(EgressError::MissingHost)?;
    let (mut addresses, is_domain) = match host {
        Host::Ipv4(address) => (vec![SocketAddr::new(IpAddr::V4(address), port)], false),
        Host::Ipv6(address) => (vec![SocketAddr::new(IpAddr::V6(address), port)], false),
        Host::Domain(domain) => {
            let resolved = resolver
                .resolve(domain, port)
                .await
                .map_err(|source| EgressError::DnsResolution {
                    host: domain.to_owned(),
                    source,
                })?;
            (resolved, true)
        }
    };

    if addresses.is_empty() {
        return Err(EgressError::EmptyAddressSet(
            url.host_str().unwrap_or_default().to_owned(),
        ));
    }

    addresses.sort_unstable();
    addresses.dedup();
    validate_address_set(&addresses)?;
    Ok((addresses, is_domain))
}

/// Deny the complete DNS answer set if any returned address is unsafe.
/// A mixed public/private answer is never partially accepted.
pub fn validate_address_set(addresses: &[SocketAddr]) -> Result<(), EgressError> {
    if addresses.is_empty() {
        return Err(EgressError::EmptyAddressSet("<empty>".to_owned()));
    }
    for address in addresses {
        if !is_public_unicast_ip(&address.ip()) {
            return Err(EgressError::AddressNotPermitted(*address));
        }
    }
    Ok(())
}

/// Conservative public-unicast classifier based on the IANA IPv4/IPv6
/// special-purpose registries (last checked 2025-10-09).
///
/// For safety, entire containing special-purpose aggregates are denied even
/// where IANA lists globally reachable more-specific exceptions. This may
/// create a small, documented false-positive set; it avoids accidentally
/// authorizing transition, translation, documentation, benchmarking, or
/// protocol-assignment ranges.
pub fn is_public_unicast_ip(ip: &IpAddr) -> bool {
    match ip {
        IpAddr::V4(address) => is_public_ipv4(address),
        IpAddr::V6(address) => is_public_ipv6(address),
    }
}

fn is_public_ipv4(ip: &Ipv4Addr) -> bool {
    // Non-special public IPv4 unicast space, excluding IANA special-purpose
    // ranges and multicast/class-E space.
    const DENY: &[([u8; 4], u8)] = &[
        ([0, 0, 0, 0], 8),       // this network / "this host"
        ([10, 0, 0, 0], 8),      // RFC 1918 private
        ([100, 64, 0, 0], 10),   // RFC 6598 shared address space
        ([127, 0, 0, 0], 8),     // loopback
        ([169, 254, 0, 0], 16),  // link-local / metadata
        ([172, 16, 0, 0], 12),   // RFC 1918 private
        ([192, 0, 0, 0], 24),    // IETF protocol assignments (conservative aggregate)
        ([192, 0, 2, 0], 24),    // TEST-NET-1
        ([192, 88, 99, 0], 24),  // deprecated 6to4 relay anycast
        ([192, 168, 0, 0], 16),  // RFC 1918 private
        ([198, 18, 0, 0], 15),   // benchmarking
        ([198, 51, 100, 0], 24), // TEST-NET-2
        ([203, 0, 113, 0], 24),  // TEST-NET-3
        ([224, 0, 0, 0], 4),     // multicast
        ([240, 0, 0, 0], 4),     // reserved and limited broadcast
    ];
    !DENY.iter().any(|(network, prefix)| ipv4_in_prefix(ip, *network, *prefix))
}

fn is_public_ipv6(ip: &Ipv6Addr) -> bool {
    // Only global-unicast space is considered, and all special-purpose
    // transition/translation/reserved/documentation ranges are denied.
    if !ipv6_in_prefix(ip, [0x20, 0x00, 0, 0, 0, 0, 0, 0], 3) {
        return false;
    }

    const DENY: &[([u16; 8], u8)] = &[
        ([0x0064, 0xff9b, 0, 0, 0, 0, 0, 0], 96), // well-known NAT64
        ([0x0064, 0xff9b, 0x0001, 0, 0, 0, 0, 0], 48), // local-use NAT64
        ([0x2001, 0, 0, 0, 0, 0, 0, 0], 23), // IETF assignments incl. Teredo
        ([0x2001, 0x0002, 0, 0, 0, 0, 0, 0], 48), // benchmarking
        ([0x2001, 0x0010, 0, 0, 0, 0, 0, 0], 28), // deprecated ORCHID
        ([0x2001, 0x0020, 0, 0, 0, 0, 0, 0], 28), // ORCHIDv2
        ([0x2001, 0x0db8, 0, 0, 0, 0, 0, 0], 32), // documentation
        ([0x2002, 0, 0, 0, 0, 0, 0, 0], 16), // 6to4 (can embed private IPv4)
        ([0x3fff, 0, 0, 0, 0, 0, 0, 0], 20), // documentation
        ([0x5f00, 0, 0, 0, 0, 0, 0, 0], 16), // SRv6 segment-routing SIDs
    ];
    !DENY.iter().any(|(network, prefix)| ipv6_in_prefix(ip, *network, *prefix))
        && !ip.is_loopback()
        && !ip.is_unspecified()
        && !ip.is_multicast()
        && !ip.is_unique_local()
        && !ip.is_unicast_link_local()
        && ip.to_ipv4_mapped().is_none()
}

fn ipv4_in_prefix(ip: &Ipv4Addr, network: [u8; 4], prefix: u8) -> bool {
    let ip = u32::from_be_bytes(ip.octets());
    let network = u32::from_be_bytes(network);
    let mask = if prefix == 0 { 0 } else { u32::MAX << (32 - prefix) };
    (ip & mask) == (network & mask)
}

fn ipv6_in_prefix(ip: &Ipv6Addr, network: [u16; 8], prefix: u8) -> bool {
    let ip = u128::from_be_bytes(ip.octets());
    let network_bytes = network.into_iter().flat_map(u16::to_be_bytes).collect::<Vec<_>>();
    let network = u128::from_be_bytes(network_bytes.try_into().expect("16-byte IPv6 network"));
    let mask = if prefix == 0 { 0 } else { u128::MAX << (128 - prefix) };
    (ip & mask) == (network & mask)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ip(value: &str) -> IpAddr {
        value.parse().expect("test IP literal")
    }

    #[test]
    fn reqwest_client_builder_accepts_a_pinned_domain_address_set() {
        let addresses = ["93.184.216.34:443".parse().unwrap()];
        validate_address_set(&addresses).unwrap();
        assert!(build_pinned_client(
            "example.com",
            &addresses,
            true,
            Duration::from_secs(5),
            Duration::from_secs(30),
        ).is_ok());
    }

    #[tokio::test]
    async fn pinned_client_does_not_follow_redirects_to_unvalidated_destinations() {
        use tokio::io::{AsyncReadExt, AsyncWriteExt};

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            let mut request = Vec::new();
            let mut buffer = [0u8; 1024];
            loop {
                let count = stream.read(&mut buffer).await.unwrap();
                if count == 0 {
                    break;
                }
                request.extend_from_slice(&buffer[..count]);
                if request.windows(4).any(|window| window == b"\r\n\r\n") {
                    break;
                }
            }
            stream
                .write_all(
                    b"HTTP/1.1 302 Found\r\nLocation: http://metadata.google.internal/\r\nContent-Length: 0\r\nConnection: close\r\n\r\n",
                )
                .await
                .unwrap();
        });

        let client = build_pinned_client(
            "prism-redirect-test.invalid",
            &[address],
            true,
            Duration::from_secs(2),
            Duration::from_secs(5),
        )
        .unwrap();
        let url = format!("http://prism-redirect-test.invalid:{}/", address.port());
        let response = client.get(&url).send().await.unwrap();
        assert_eq!(response.status(), StatusCode::FOUND);
        server.await.unwrap();
    }

    #[tokio::test]
    async fn pinned_reqwest_client_connects_to_the_override_not_system_dns() {
        use tokio::io::{AsyncReadExt, AsyncWriteExt};

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let (mut stream, peer) = listener.accept().await.unwrap();
            let mut request = Vec::new();
            let mut buffer = [0u8; 1024];
            loop {
                let count = stream.read(&mut buffer).await.unwrap();
                if count == 0 {
                    break;
                }
                request.extend_from_slice(&buffer[..count]);
                if request.windows(4).any(|window| window == b"\r\n\r\n") {
                    break;
                }
            }
            stream
                .write_all(b"HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\nok")
                .await
                .unwrap();
            (peer, request)
        });

        // Exercise reqwest's resolver override with a loopback test server.
        // Production authorization rejects loopback before transport setup.
        let client = build_pinned_client(
            "prism-pinned-test.invalid",
            &[address],
            true,
            Duration::from_secs(2),
            Duration::from_secs(5),
        )
        .unwrap();
        let url = format!("http://prism-pinned-test.invalid:{}/probe", address.port());
        let response = client.get(&url).send().await.unwrap();
        assert_eq!(response.status(), StatusCode::OK);
        assert_eq!(response.bytes().await.unwrap().as_ref(), b"ok");

        let (peer, request) = server.await.unwrap();
        assert_eq!(peer.ip(), IpAddr::V4(Ipv4Addr::LOCALHOST));
        let request = String::from_utf8_lossy(&request);
        assert!(request.contains(&format!("Host: prism-pinned-test.invalid:{}", address.port())));
    }

    #[test]
    fn allows_representative_public_unicast_addresses() {
        for address in [
            "1.1.1.1",
            "8.8.8.8",
            "93.184.216.34",
            "192.31.196.1",
            "192.52.193.1",
            "192.175.48.1",
            "2606:4700:4700::1111",
            "2001:4860::1",
        ] {
            assert!(is_public_unicast_ip(&ip(address)), "unexpectedly denied {address}");
        }
    }

    #[test]
    fn denies_ipv4_special_and_non_global_ranges() {
        for address in [
            "0.1.2.3", "10.1.2.3", "100.64.0.1", "127.0.0.1",
            "169.254.169.254", "172.16.0.1", "192.0.0.1",
            "192.0.2.1", "192.88.99.1", "192.168.1.1",
            "198.18.0.1", "198.51.100.1", "203.0.113.1",
            "224.0.0.1", "240.0.0.1", "255.255.255.255",
        ] {
            assert!(!is_public_unicast_ip(&ip(address)), "unexpectedly allowed {address}");
        }
    }

    #[test]
    fn denies_ipv6_special_transition_and_non_global_ranges() {
        for address in [
            "::", "::1", "::ffff:8.8.8.8", "64:ff9b::808:808",
            "64:ff9b:1::1", "100::1", "2001::1", "2001:2::1",
            "2001:10::1", "2001:20::1", "2001:db8::1", "2002::1",
            "3fff::1", "5f00::1", "fc00::1", "fe80::1", "ff02::1",
        ] {
            assert!(!is_public_unicast_ip(&ip(address)), "unexpectedly allowed {address}");
        }
    }

    #[test]
    fn rejects_a_mixed_public_and_private_dns_answer_set() {
        let addresses = [
            "93.184.216.34:443".parse().unwrap(),
            "127.0.0.1:443".parse().unwrap(),
        ];
        assert!(matches!(
            validate_address_set(&addresses),
            Err(EgressError::AddressNotPermitted(_))
        ));
    }

    #[test]
    fn rejects_empty_dns_answer_set() {
        assert!(matches!(
            validate_address_set(&[]),
            Err(EgressError::EmptyAddressSet(_))
        ));
    }

    #[test]
    fn accepts_only_http_and_https_default_ports() {
        assert!(validate_url_policy(&Url::parse("https://example.com").unwrap()).is_ok());
        assert!(validate_url_policy(&Url::parse("http://example.com").unwrap()).is_ok());
        assert!(matches!(
            validate_url_policy(&Url::parse("http://example.com:8080").unwrap()),
            Err(EgressError::PortNotPermitted)
        ));
        assert!(matches!(
            validate_url_policy(&Url::parse("file:///etc/passwd").unwrap()),
            Err(EgressError::UnsupportedScheme)
        ));
    }

    #[test]
    fn blocks_local_and_metadata_hostnames_before_dns() {
        for raw in [
            "http://localhost/",
            "http://printer.local/",
            "http://svc.internal/",
            "http://metadata.google.internal/",
            "http://metadata.google.internal./",
        ] {
            assert!(matches!(
                validate_url_policy(&Url::parse(raw).unwrap()),
                Err(EgressError::BlockedHost)
            ), "expected blocked host: {raw}");
        }
    }

    #[derive(Clone)]
    struct StaticResolver {
        answers: Vec<SocketAddr>,
        calls: Arc<std::sync::atomic::AtomicUsize>,
    }

    impl EgressResolver for StaticResolver {
        fn resolve<'a>(
            &'a self,
            _host: &'a str,
            _port: u16,
        ) -> Pin<Box<dyn Future<Output = io::Result<Vec<SocketAddr>>> + Send + 'a>> {
            self.calls.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            let answers = self.answers.clone();
            Box::pin(async move { Ok(answers) })
        }
    }

    fn socket_addresses(values: &[&str]) -> Vec<SocketAddr> {
        values.iter().map(|value| value.parse().unwrap()).collect()
    }

    #[tokio::test]
    async fn injected_resolver_rejects_mixed_public_and_private_answers() {
        let resolver = StaticResolver {
            answers: socket_addresses(&["93.184.216.34:443", "10.0.0.8:443"]),
            calls: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
        };
        let client = PublicEgressClient::with_resolver(EgressLimits::default(), resolver.clone());
        let result = client
            .authorize(Method::GET, "https://mixed.example/", HeaderMap::new(), Vec::new())
            .await;
        assert!(matches!(result, Err(EgressError::AddressNotPermitted(_))));
        assert_eq!(resolver.calls.load(std::sync::atomic::Ordering::SeqCst), 1);
    }

    #[tokio::test]
    async fn injected_resolver_snapshot_is_taken_exactly_once_for_authorization() {
        let resolver = StaticResolver {
            answers: socket_addresses(&["93.184.216.34:443", "[2606:4700:4700::1111]:443"]),
            calls: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
        };
        let client = PublicEgressClient::with_resolver(EgressLimits::default(), resolver.clone());
        let capability = client
            .authorize(Method::GET, "https://snapshot.example/", HeaderMap::new(), Vec::new())
            .await
            .unwrap();
        assert_eq!(resolver.calls.load(std::sync::atomic::Ordering::SeqCst), 1);
        assert_eq!(capability.addresses.len(), 2);
        assert!(capability.addresses.iter().any(|address| address.ip() == ip("93.184.216.34")));
        assert!(
            capability
                .addresses
                .iter()
                .any(|address| address.ip() == ip("2606:4700:4700::1111"))
        );
    }

    #[test]
    fn public_web_authority_rejects_headers_and_request_bodies() {
        let url = Url::parse("https://attacker.example/collect").unwrap();
        let mut headers = HeaderMap::new();
        headers.insert(reqwest::header::AUTHORIZATION, "Bearer secret".parse().unwrap());
        assert!(matches!(
            validate_authority_scope(
                EgressAuthority::PublicWeb,
                &url,
                &Method::GET,
                &headers,
                &[],
            ),
            Err(EgressError::AuthorityScopeViolation)
        ));
        assert!(matches!(
            validate_authority_scope(
                EgressAuthority::PublicWeb,
                &url,
                &Method::POST,
                &HeaderMap::new(),
                b"private payload",
            ),
            Err(EgressError::AuthorityScopeViolation)
        ));
    }

    #[test]
    fn brave_authority_is_fixed_to_exact_origin_path_and_headers() {
        let url = Url::parse("https://api.search.brave.com/res/v1/web/search?q=x").unwrap();
        let mut headers = HeaderMap::new();
        headers.insert("X-Subscription-Token", "secret".parse().unwrap());
        headers.insert(reqwest::header::ACCEPT, "application/json".parse().unwrap());
        assert!(validate_authority_scope(
            EgressAuthority::BraveSearch,
            &url,
            &Method::GET,
            &headers,
            &[],
        ).is_ok());

        let attacker = Url::parse("https://attacker.example/res/v1/web/search").unwrap();
        assert!(matches!(
            validate_authority_scope(EgressAuthority::BraveSearch, &attacker, &Method::GET, &headers, &[]),
            Err(EgressError::AuthorityScopeViolation)
        ));

        let wrong_path = Url::parse("https://api.search.brave.com/other").unwrap();
        assert!(matches!(
            validate_authority_scope(EgressAuthority::BraveSearch, &wrong_path, &Method::GET, &headers, &[]),
            Err(EgressError::AuthorityScopeViolation)
        ));
    }

    #[test]
    fn perplexity_authority_is_fixed_to_exact_origin_path_and_header_set() {
        let url = Url::parse("https://api.perplexity.ai/chat/completions").unwrap();
        let mut headers = HeaderMap::new();
        headers.insert(reqwest::header::AUTHORIZATION, "Bearer secret".parse().unwrap());
        headers.insert(reqwest::header::CONTENT_TYPE, "application/json".parse().unwrap());
        assert!(validate_authority_scope(
            EgressAuthority::Perplexity,
            &url,
            &Method::POST,
            &headers,
            b"{}",
        ).is_ok());

        let attacker = Url::parse("https://attacker.example/chat/completions").unwrap();
        assert!(matches!(
            validate_authority_scope(EgressAuthority::Perplexity, &attacker, &Method::POST, &headers, b"{}"),
            Err(EgressError::AuthorityScopeViolation)
        ));

        let mut extra = headers.clone();
        extra.insert("x-forwarded-host", "attacker.example".parse().unwrap());
        assert!(matches!(
            validate_authority_scope(EgressAuthority::Perplexity, &url, &Method::POST, &extra, b"{}"),
            Err(EgressError::AuthorityHeaderViolation(_))
        ));
    }

    #[tokio::test]
    async fn capability_binds_the_exact_headers_and_body() {
        let client = PublicEgressClient::default();
        let mut headers = HeaderMap::new();
        headers.insert(reqwest::header::ACCEPT, "application/json".parse().unwrap());
        let capability = client
            .authorize(
                Method::POST,
                "https://1.1.1.1/resource",
                headers.clone(),
                b"exact-payload".to_vec(),
            )
            .await
            .unwrap();

        assert_eq!(capability.url.as_str(), "https://1.1.1.1/resource");
        assert_eq!(capability.method, Method::POST);
        assert_eq!(capability.headers, headers);
        assert_eq!(capability.body, b"exact-payload");
    }

    #[tokio::test]
    async fn rejects_oversized_request_body_before_dns_lookup() {
        let mut limits = EgressLimits::default();
        limits.max_request_body_bytes = 1;
        let client = PublicEgressClient::new(limits);
        let result = client
            .authorize(
                Method::POST,
                "https://this-name-should-not-be-resolved.invalid/",
                HeaderMap::new(),
                vec![1, 2],
            )
            .await;
        assert!(matches!(result, Err(EgressError::RequestBodyTooLarge)));
    }

    #[tokio::test]
    async fn requires_tls_for_custom_outbound_headers() {
        let client = PublicEgressClient::default();
        let mut headers = HeaderMap::new();
        headers.insert("x-api-token", "secret".parse().unwrap());
        let result = client
            .authorize(Method::GET, "http://93.184.216.34/", headers, Vec::new())
            .await;
        assert!(matches!(result, Err(EgressError::RequestRequiresTls)));
    }

    #[tokio::test]
    async fn rejects_http_post_payload_before_dns_lookup() {
        let client = PublicEgressClient::default();
        let result = client
            .authorize(
                Method::POST,
                "http://this-name-should-not-be-resolved.invalid/",
                HeaderMap::new(),
                b"payload".to_vec(),
            )
            .await;
        assert!(matches!(result, Err(EgressError::RequestRequiresTls)));
    }

    #[tokio::test]
    async fn rejects_private_ip_literals_before_transport() {
        let client = PublicEgressClient::default();
        let result = client
            .authorize(Method::GET, "http://127.0.0.1/", HeaderMap::new(), Vec::new());
        assert!(matches!(
            result.await,
            Err(EgressError::AddressNotPermitted(_))
        ));
    }

    #[tokio::test]
    async fn expires_capabilities_before_execution_when_ttl_is_zero() {
        let mut limits = EgressLimits::default();
        limits.capability_ttl = Duration::ZERO;
        let client = PublicEgressClient::new(limits);
        let capability = client
            .authorize(Method::GET, "https://1.1.1.1/", HeaderMap::new(), Vec::new())
            .await
            .unwrap();
        let result = client.execute(capability).await;
        assert!(matches!(result, Err(EgressError::CapabilityExpired)));
    }

    #[test]
    fn rejects_url_credentials() {
        assert!(matches!(
            validate_url_policy(&Url::parse("https://user:secret@example.com").unwrap()),
            Err(EgressError::CredentialsNotPermitted)
        ));
    }

    #[test]
    fn rejects_headers_that_can_override_transport_authority() {
        for name in [
            "host",
            "content-length",
            "transfer-encoding",
            "proxy-authorization",
            "connection",
        ] {
            let mut headers = HeaderMap::new();
            headers.insert(name, "attacker-controlled");
            assert!(matches!(
                validate_request_headers(&headers),
                Err(EgressError::ForbiddenHeader(_))
            ), "expected forbidden header: {name}");
        }
    }

    #[test]
    fn enforces_request_header_budget() {
        let mut headers = HeaderMap::new();
        headers.insert("x-test", "long-value");
        assert!(request_header_bytes(&headers) >= "x-test".len() + "long-value".len());
    }
}
