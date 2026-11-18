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

pub use reqwest::Method;
pub use reqwest::header::HeaderMap;
use bytes::Bytes;
use reqwest::{Client, StatusCode};
use std::collections::HashSet;
use std::future::Future;
use std::io;
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr};
use std::pin::Pin;
use std::sync::{Arc, OnceLock};
use std::time::Duration;
use thiserror::Error;
use tokio::sync::{OwnedSemaphorePermit, Semaphore, SemaphorePermit};
use url::{Host, Url};

const MAX_CONCURRENT_EGRESS_REQUESTS: usize = 64;
const MAX_CONCURRENT_DNS_LOOKUPS: usize = 16;
const MAX_DNS_ANSWER_ADDRESSES: usize = 32;
const MAX_PUBLIC_WEB_REDIRECT_HOPS: usize = 5;
const DNS_QUEUE_TIMEOUT: Duration = Duration::from_secs(2);
const DNS_LOOKUP_TIMEOUT: Duration = Duration::from_secs(5);
const REQUEST_HEADER_OVERHEAD_ALLOWANCE: usize = 512;
const MAX_RESPONSE_HEADER_COUNT: usize = 128;
const RESPONSE_MEMORY_BUDGET_BYTES: usize = 128 * 1024 * 1024;
const RESPONSE_MEMORY_PERMIT_UNIT_BYTES: usize = 64 * 1024;
const RESPONSE_MEMORY_BUDGET_PERMITS: usize =
    RESPONSE_MEMORY_BUDGET_BYTES / RESPONSE_MEMORY_PERMIT_UNIT_BYTES;
const RESPONSE_MEMORY_QUEUE_TIMEOUT: Duration = Duration::from_secs(2);
static EGRESS_SEMAPHORE: Semaphore = Semaphore::const_new(MAX_CONCURRENT_EGRESS_REQUESTS);
static DNS_SEMAPHORE: OnceLock<Arc<Semaphore>> = OnceLock::new();
static RESPONSE_MEMORY_SEMAPHORE: OnceLock<Arc<Semaphore>> = OnceLock::new();

fn response_memory_semaphore() -> &'static Arc<Semaphore> {
    RESPONSE_MEMORY_SEMAPHORE
        .get_or_init(|| Arc::new(Semaphore::new(RESPONSE_MEMORY_BUDGET_PERMITS)))
}

/// Reserve the whole logical body budget in fixed 64 KiB units before buffering.
/// Whole-budget reservations avoid deadlock from multiple streams holding partial
/// weighted permits while waiting for more memory.
fn response_memory_permits(body_limit_bytes: usize) -> u32 {
    let units = body_limit_bytes
        .max(1)
        .saturating_add(RESPONSE_MEMORY_PERMIT_UNIT_BYTES - 1)
        / RESPONSE_MEMORY_PERMIT_UNIT_BYTES;
    u32::try_from(units).expect("bounded Prism response limit fits in u32 permits")
}

fn dns_semaphore() -> &'static Arc<Semaphore> {
    DNS_SEMAPHORE.get_or_init(|| Arc::new(Semaphore::new(MAX_CONCURRENT_DNS_LOOKUPS)))
}

#[derive(Clone, Debug)]
pub struct EgressLimits {
    pub max_url_bytes: usize,
    pub max_request_header_bytes: usize,
    pub max_request_body_bytes: usize,
    pub max_response_body_bytes: usize,
    pub max_error_response_body_bytes: usize,
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
            max_error_response_body_bytes: 64 * 1024,
            connect_timeout: Duration::from_secs(5),
            request_timeout: Duration::from_secs(30),
            capability_ttl: Duration::from_secs(5),
        }
    }
}

impl EgressLimits {
    /// Apply absolute policy ceilings even when a caller supplies custom
    /// lower limits. Public configuration must never widen the egress policy.
    fn bounded(self) -> Self {
        Self {
            max_url_bytes: self.max_url_bytes.min(8 * 1024),
            max_request_header_bytes: self.max_request_header_bytes.min(16 * 1024),
            max_request_body_bytes: self.max_request_body_bytes.min(2 * 1024 * 1024),
            max_response_body_bytes: self.max_response_body_bytes.min(16 * 1024 * 1024),
            max_error_response_body_bytes: self.max_error_response_body_bytes.min(64 * 1024),
            connect_timeout: self.connect_timeout.min(Duration::from_secs(5)),
            request_timeout: self.request_timeout.min(Duration::from_secs(90)),
            capability_ttl: self.capability_ttl.min(Duration::from_secs(5)),
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
    #[error("DNS answer set exceeded the configured address count limit")]
    DnsAnswerSetTooLarge,
    #[error("DNS lookup exceeded its time limit")]
    DnsResolutionTimeout,
    #[error("DNS authorization capacity was not acquired for a hostname lookup")]
    DnsCapacityUnavailable,
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
    #[error("aggregate response-memory budget wait exceeded its limit")]
    ResponseMemoryBudgetTimeout,
    #[error("aggregate response-memory budget is unavailable")]
    ResponseMemoryBudgetUnavailable,
    #[error("transport did not expose the connected peer address")]
    RemotePeerUnavailable,
    #[error("connected peer was not in the authorized DNS snapshot: {0}")]
    RemotePeerNotAuthorized(SocketAddr),
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
    #[error("total egress execution deadline expired")]
    ExecutionTimeout,
    #[error("public-web redirect limit exceeded")]
    RedirectLimitExceeded,
    #[error("public-web redirect location is invalid")]
    InvalidRedirectLocation,
    #[error("redirect response contains multiple Location fields")]
    AmbiguousRedirectLocation,
    #[error("public-web redirect chain contains a loop")]
    RedirectLoop,
    #[error("HTTPS-to-HTTP redirect downgrade is not permitted")]
    RedirectSchemeDowngrade,
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
    /// Wikipedia REST page-summary endpoint; GET only.
    WikipediaSummary,
    /// Wikidata SPARQL endpoint with constrained query parameters; GET only.
    WikidataSparql,
}

const MAX_PUBLIC_WEB_REQUESTS: usize = 52;
const MAX_BRAVE_REQUESTS: usize = 4;
const MAX_PERPLEXITY_REQUESTS: usize = 4;
const MAX_WIKIPEDIA_REQUESTS: usize = 2;
const MAX_WIKIDATA_REQUESTS: usize = 2;
static PUBLIC_WEB_SEMAPHORE: Semaphore = Semaphore::const_new(MAX_PUBLIC_WEB_REQUESTS);
static BRAVE_SEMAPHORE: Semaphore = Semaphore::const_new(MAX_BRAVE_REQUESTS);
static PERPLEXITY_SEMAPHORE: Semaphore = Semaphore::const_new(MAX_PERPLEXITY_REQUESTS);
static WIKIPEDIA_SEMAPHORE: Semaphore = Semaphore::const_new(MAX_WIKIPEDIA_REQUESTS);
static WIKIDATA_SEMAPHORE: Semaphore = Semaphore::const_new(MAX_WIKIDATA_REQUESTS);

impl EgressAuthority {
    fn label(self) -> &'static str {
        match self {
            Self::PublicWeb => "public-web",
            Self::BraveSearch => "brave-search",
            Self::Perplexity => "perplexity",
            Self::WikipediaSummary => "wikipedia-summary",
            Self::WikidataSparql => "wikidata-sparql",
        }
    }

    fn admission_semaphore(self) -> &'static Semaphore {
        match self {
            Self::PublicWeb => &PUBLIC_WEB_SEMAPHORE,
            Self::BraveSearch => &BRAVE_SEMAPHORE,
            Self::Perplexity => &PERPLEXITY_SEMAPHORE,
            Self::WikipediaSummary => &WIKIPEDIA_SEMAPHORE,
            Self::WikidataSparql => &WIKIDATA_SEMAPHORE,
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
    max_error_response_body_bytes: usize,
    connect_timeout: Duration,
    request_timeout: Duration,
    policy_version: &'static str,
}

/// Owns buffered response bytes and the aggregate-memory lease for their
/// entire lifetime. Converting this value to Bytes preserves the lease in the
/// Bytes backing owner, including across shallow Bytes clones and Axum delivery.
pub struct EgressBody {
    bytes: Bytes,
}

struct LeasedResponseBuffer {
    bytes: Vec<u8>,
    _memory_permit: OwnedSemaphorePermit,
}

impl AsRef<[u8]> for LeasedResponseBuffer {
    fn as_ref(&self) -> &[u8] {
        &self.bytes
    }
}

impl EgressBody {
    fn new(bytes: Vec<u8>, memory_permit: OwnedSemaphorePermit) -> Self {
        Self {
            bytes: Bytes::from_owner(LeasedResponseBuffer {
                bytes,
                _memory_permit: memory_permit,
            }),
        }
    }

    pub fn as_slice(&self) -> &[u8] {
        self.bytes.as_ref()
    }

    pub fn len(&self) -> usize {
        self.bytes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }

    /// Move the lease-carrying Bytes buffer into the downstream body. The
    /// reservation is released only after every Bytes clone is dropped.
    pub fn into_bytes(self) -> Bytes {
        self.bytes
    }
}

impl std::fmt::Debug for EgressBody {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("EgressBody")
            .field("len", &self.bytes.len())
            .finish()
    }
}

impl AsRef<[u8]> for EgressBody {
    fn as_ref(&self) -> &[u8] {
        self.bytes.as_ref()
    }
}

impl std::ops::Deref for EgressBody {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.bytes.as_ref()
    }
}

/// A bounded response plus a privacy-minimized execution receipt.
#[derive(Debug)]
pub struct EgressResponse {
    pub status: StatusCode,
    pub headers: HeaderMap,
    pub body: EgressBody,
    pub receipt: EgressReceipt,
}

#[derive(Clone, Debug)]
pub struct RedirectHopReceipt {
    pub from_origin: String,
    pub to_origin: String,
    pub status: u16,
    pub remote_peer: SocketAddr,
    pub target_addresses: Vec<SocketAddr>,
}

#[derive(Clone, Debug)]
pub struct EgressReceipt {
    /// Scheme + authority only. Path, query, credentials, and response content
    /// are deliberately excluded from the receipt.
    pub origin: String,
    pub method: String,
    pub authorized_addresses: Vec<SocketAddr>,
    pub remote_peer: SocketAddr,
    pub status: u16,
    pub request_body_bytes: usize,
    pub response_body_bytes: usize,
    pub response_body_limit_bytes: usize,
    pub elapsed: Duration,
    pub policy_version: &'static str,
    pub authority: &'static str,
    pub redirect_hops: Vec<RedirectHopReceipt>,
}

/// Resolver interface makes DNS policy deterministic under test. Implementations
/// must return the complete A and AAAA answer set, not only the first address.
trait EgressResolver: Send + Sync {
    /// Resolve one host while owning the DNS admission permit. Implementations
    /// that delegate to non-cancellable blocking work must move this permit
    /// into that work so timeouts cannot release capacity early.
    fn resolve<'a>(
        &'a self,
        host: &'a str,
        port: u16,
        permit: OwnedSemaphorePermit,
    ) -> Pin<Box<dyn Future<Output = io::Result<Vec<SocketAddr>>> + Send + 'a>>;
}

#[derive(Clone, Copy, Debug, Default)]
struct SystemEgressResolver;

fn spawn_blocking_dns_with_permit<F>(
    permit: OwnedSemaphorePermit,
    work: F,
) -> impl Future<Output = io::Result<Vec<SocketAddr>>> + Send
where
    F: FnOnce() -> io::Result<Vec<SocketAddr>> + Send + 'static,
{
    async move {
        // A started spawn_blocking task cannot be aborted. Keep the owned
        // permit inside the closure so cancelling the async waiter cannot
        // release resolver capacity while the OS operation is still running.
        tokio::task::spawn_blocking(move || {
            let _permit = permit;
            work()
        })
        .await
        .map_err(|error| io::Error::other(format!("DNS worker failed: {error}")))?
    }
}

impl EgressResolver for SystemEgressResolver {
    fn resolve<'a>(
        &'a self,
        host: &'a str,
        port: u16,
        permit: OwnedSemaphorePermit,
    ) -> Pin<Box<dyn Future<Output = io::Result<Vec<SocketAddr>>> + Send + 'a>> {
        let host = host.to_owned();
        Box::pin(spawn_blocking_dns_with_permit(permit, move || {
            std::net::ToSocketAddrs::to_socket_addrs(&(host.as_str(), port))
                .map(collect_unique_addresses_bounded)
        }))
    }
}

/// Public-web-only HTTP client. Each consumed capability receives a fresh,
/// DNS-pinned reqwest client to prevent cross-request pool reuse.
#[derive(Clone)]
pub struct PublicEgressClient {
    limits: EgressLimits,
    resolver: Arc<dyn EgressResolver>,
    // Test-only exception so the real authorization -> pinned socket -> peer
    // receipt path can be exercised against loopback listeners. This field
    // does not exist in non-test builds and is never externally configurable.
    #[cfg(test)]
    allow_test_destinations: bool,
    // Trust roots are injected only by local TLS tests; production always uses
    // the platform/WebPKI trust configuration.
    #[cfg(test)]
    test_root_certificates: Vec<reqwest::Certificate>,
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

    fn with_resolver<R>(limits: EgressLimits, resolver: R) -> Self
    where
        R: EgressResolver + 'static,
    {
        Self {
            limits: limits.bounded(),
            resolver: Arc::new(resolver),
            #[cfg(test)]
            allow_test_destinations: false,
            #[cfg(test)]
            test_root_certificates: Vec::new(),
        }
    }

    #[cfg(test)]
    fn with_test_resolver<R>(limits: EgressLimits, resolver: R) -> Self
    where
        R: EgressResolver + 'static,
    {
        let mut client = Self::with_resolver(limits, resolver);
        client.allow_test_destinations = true;
        client
    }

    #[cfg(test)]
    fn with_test_root_certificates(
        mut self,
        certificates: Vec<reqwest::Certificate>,
    ) -> Self {
        self.test_root_certificates = certificates;
        self
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
        self.authorize_with_authority(EgressAuthority::PublicWeb, method, raw_url, headers, body)
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
        if request_header_bytes(&headers).saturating_add(REQUEST_HEADER_OVERHEAD_ALLOWANCE)
            > self.limits.max_request_header_bytes
        {
            return Err(EgressError::RequestHeadersTooLarge);
        }

        let mut url = Url::parse(raw_url)?;
        #[cfg(test)]
        let allow_test_destinations = self.allow_test_destinations;
        #[cfg(not(test))]
        let allow_test_destinations = false;
        validate_url_policy_with_test_port(&url, allow_test_destinations)?;
        if url.scheme() == "http"
            && (method != Method::GET || !headers.is_empty() || !body.is_empty())
        {
            return Err(EgressError::RequestRequiresTls);
        }
        validate_authority_scope(authority, &url, &method, &headers, &body)?;
        // URL fragments are client-side state and must not affect the
        // authorization identity or be sent as part of the HTTP request.
        url.set_fragment(None);

        let host = url.host_str().ok_or(EgressError::MissingHost)?.to_owned();
        let port = url
            .port_or_known_default()
            .ok_or(EgressError::PortNotPermitted)?;

        // Bound hostname resolution independently from HTTP execution.
        // Numeric IP literals do not consume DNS capacity.
        let dns_permit = match url.host().ok_or(EgressError::MissingHost)? {
            Host::Domain(_) => Some(
                tokio::time::timeout(DNS_QUEUE_TIMEOUT, dns_semaphore().clone().acquire_owned())
                    .await
                    .map_err(|_| EgressError::ConcurrencyQueueTimeout)?
                    .map_err(|_| EgressError::ConcurrencyUnavailable)?,
            ),
            Host::Ipv4(_) | Host::Ipv6(_) => None,
        };
        let (addresses, is_domain) = tokio::time::timeout(
            DNS_LOOKUP_TIMEOUT,
            resolve_and_authorize(
                &url,
                port,
                self.resolver.as_ref(),
                dns_permit,
                allow_test_destinations,
            ),
        )
        .await
        .map_err(|_| EgressError::DnsResolutionTimeout)??;

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
            max_error_response_body_bytes: self.limits.max_error_response_body_bytes,
            connect_timeout: self.limits.connect_timeout,
            request_timeout: self.limits.request_timeout,
            policy_version: "prism-public-egress-v1",
        })
    }

    /// Consume one authorization and execute the exact request it contains.
    /// PublicWeb redirects are followed manually: every hop receives a fresh
    /// DNS authorization and pinned connection. Provider scopes never follow.
    pub async fn execute(
        &self,
        capability: EgressCapability,
    ) -> Result<EgressResponse, EgressError> {
        let total_timeout = capability.request_timeout;
        tokio::time::timeout(total_timeout, self.execute_redirect_chain(capability))
            .await
            .map_err(|_| EgressError::ExecutionTimeout)?
    }

    async fn execute_redirect_chain(
        &self,
        capability: EgressCapability,
    ) -> Result<EgressResponse, EgressError> {
        let started = std::time::Instant::now();
        let mut capability = capability;
        let mut visited_urls = HashSet::new();
        visited_urls.insert(capability.url.as_str().to_owned());
        let mut redirect_hops = Vec::new();

        for redirect_count in 0..=MAX_PUBLIC_WEB_REDIRECT_HOPS {
            if std::time::Instant::now() >= capability.expires_at {
                return Err(EgressError::CapabilityExpired);
            }

            let origin = sanitized_origin(&capability.url);
            let authority = capability.authority.label();
            let authorized_addresses = capability.addresses.clone();
            let method_name = capability.method.as_str().to_owned();
            let request_body_bytes = capability.body.len();

            // Reserve authority-specific capacity before the shared total budget.
            // A burst of generic web reads cannot occupy Brave/Perplexity slots.
            let _authority_permit: SemaphorePermit<'static> = tokio::time::timeout(
                Duration::from_secs(2),
                capability.authority.admission_semaphore().acquire(),
            )
            .await
            .map_err(|_| EgressError::ConcurrencyQueueTimeout)?
            .map_err(|_| EgressError::ConcurrencyUnavailable)?;
            let _permit: SemaphorePermit<'static> =
                tokio::time::timeout(Duration::from_secs(2), EGRESS_SEMAPHORE.acquire())
                    .await
                    .map_err(|_| EgressError::ConcurrencyQueueTimeout)?
                    .map_err(|_| EgressError::ConcurrencyUnavailable)?;

            // Queueing consumes capability lifetime. Re-check after admission
            // so an expired authorization cannot connect merely because it waited.
            if std::time::Instant::now() >= capability.expires_at {
                return Err(EgressError::CapabilityExpired);
            }

            let client = build_pinned_client(
                &capability.host,
                &capability.addresses,
                capability.is_domain,
                capability.connect_timeout,
                capability.request_timeout,
                #[cfg(test)]
                &self.test_root_certificates,
            )?;

            let response = client
                .request(capability.method.clone(), capability.url.clone())
                .headers(capability.headers.clone())
                .body(capability.body.clone())
                .send()
                .await?;

            let remote_peer =
                validate_connected_peer(&capability.addresses, response.remote_addr())?;
            let status = response.status();
            let response_headers = response.headers().clone();
            if response_header_bytes(&response_headers) > 64 * 1024
                || response_headers.len() > MAX_RESPONSE_HEADER_COUNT
            {
                return Err(EgressError::ResponseHeadersTooLarge);
            }

            if authority_may_follow_redirect(capability.authority) && is_followable_redirect(status)
            {
                let location_values = response_headers
                    .get_all(reqwest::header::LOCATION)
                    .iter()
                    .count();
                if location_values > 1 {
                    return Err(EgressError::AmbiguousRedirectLocation);
                }
                if let Some(location) = response_headers.get(reqwest::header::LOCATION) {
                    if redirect_count >= MAX_PUBLIC_WEB_REDIRECT_HOPS {
                        return Err(EgressError::RedirectLimitExceeded);
                    }
                    let location = location
                        .to_str()
                        .map_err(|_| EgressError::InvalidRedirectLocation)?;
                    let next_url = resolve_public_redirect_target(&capability.url, location)?;

                    // Redirects carry no user headers or bodies in PublicWeb
                    // scope. Re-authorization validates URL syntax, scheme,
                    // port, credentials, DNS answers, and the next socket set.
                    let mut next_url = next_url;
                    next_url.set_fragment(None);
                    record_redirect_target(&mut visited_urls, &next_url)?;

                    // Release the redirecting socket and execution permits
                    // before DNS work for the next hop. Resolver capacity is
                    // independent, and no HTTP slot is held while resolving.
                    drop(response);
                    drop(_permit);
                    drop(_authority_permit);
                    let next_capability = self
                        .authorize_with_authority(
                            EgressAuthority::PublicWeb,
                            Method::GET,
                            next_url.as_str(),
                            HeaderMap::new(),
                            Vec::new(),
                        )
                        .await?;

                    let to_origin = sanitized_origin(&next_capability.url);
                    redirect_hops.push(RedirectHopReceipt {
                        from_origin: origin,
                        to_origin,
                        status: status.as_u16(),
                        remote_peer,
                        target_addresses: next_capability.addresses.clone(),
                    });

                    capability = next_capability;
                    continue;
                }
            }

            let response_body_limit_bytes = if status.is_success() {
                capability.max_response_body_bytes
            } else {
                capability.max_error_response_body_bytes
            };
            if response
                .content_length()
                .is_some_and(|length| length > response_body_limit_bytes as u64)
            {
                return Err(EgressError::ResponseBodyTooLarge);
            }

            // Reserve the complete body ceiling before allocating or buffering.
            // This is deliberately not acquired chunk-by-chunk: partial weighted
            // reservations can deadlock when streams retain memory while waiting
            // for additional permits.
            let memory_permits = response_memory_permits(response_body_limit_bytes);
            let memory_permit = tokio::time::timeout(
                RESPONSE_MEMORY_QUEUE_TIMEOUT,
                response_memory_semaphore()
                    .clone()
                    .acquire_many_owned(memory_permits),
            )
            .await
            .map_err(|_| EgressError::ResponseMemoryBudgetTimeout)?
            .map_err(|_| EgressError::ResponseMemoryBudgetUnavailable)?;

            let mut response = response;
            let mut collected = Vec::new();
            collected
                .try_reserve_exact(response_body_limit_bytes)
                .map_err(|_| EgressError::ResponseMemoryBudgetUnavailable)?;
            while let Some(chunk) = response.chunk().await? {
                if collected.len().saturating_add(chunk.len()) > response_body_limit_bytes {
                    return Err(EgressError::ResponseBodyTooLarge);
                }
                collected.extend_from_slice(&chunk);
            }

            let receipt = EgressReceipt {
                origin,
                method: method_name,
                authorized_addresses,
                remote_peer,
                status: status.as_u16(),
                request_body_bytes,
                response_body_bytes: collected.len(),
                response_body_limit_bytes,
                elapsed: started.elapsed(),
                policy_version: capability.policy_version,
                authority,
                redirect_hops,
            };

            return Ok(EgressResponse {
                status,
                headers: response_headers,
                body: EgressBody::new(collected, memory_permit),
                receipt,
            });
        }

        Err(EgressError::RedirectLimitExceeded)
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
    #[cfg(test)] test_root_certificates: &[reqwest::Certificate],
) -> Result<Client, EgressError> {
    // Each request gets a fresh pool. The URL retains the original hostname
    // for Host/SNI/certificate checks while this explicit resolver override
    // constrains socket destinations to the authorized set.
    let mut builder = Client::builder()
        .user_agent("Prism/0.4 (Luminous Dynamics; https://luminousdynamics.io)")
        .redirect(reqwest::redirect::Policy::none())
        // Reqwest 0.12.23+ retries protocol NACKs by default. Keep retry
        // decisions inside Prism so every attempt is visible, bounded, and
        // authorized by the same capability state machine.
        .retry(reqwest::retry::never())
        .no_proxy()
        .http1_max_headers(MAX_RESPONSE_HEADER_COUNT)
        .http2_max_header_list_size(64 * 1024)
        .connect_timeout(connect_timeout)
        .timeout(request_timeout);
    if is_domain {
        builder = builder.resolve_to_addrs(host, addresses);
    }
    #[cfg(test)]
    for certificate in test_root_certificates {
        builder = builder.add_root_certificate(certificate.clone());
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
                    return Err(EgressError::AuthorityHeaderViolation(
                        name.as_str().to_owned(),
                    ));
                }
            }
            let token_count = headers.get_all("x-subscription-token").iter().count();
            let accept_count = headers.get_all(reqwest::header::ACCEPT).iter().count();
            if token_count > 1 || accept_count > 1 {
                return Err(EgressError::AuthorityHeaderViolation(
                    "duplicate Brave provider header".to_owned(),
                ));
            }
            let token = headers
                .get("x-subscription-token")
                .and_then(|value| value.to_str().ok())
                .filter(|value| !value.trim().is_empty());
            if token.is_none() {
                return Err(EgressError::ProviderCredentialMissing);
            }
        }
        EgressAuthority::WikipediaSummary => {
            let title = path.strip_prefix("/api/rest_v1/page/summary/");
            if !is_https
                || host != "en.wikipedia.org"
                || title.is_none_or(|value| {
                    value.is_empty() || value.contains('/') || value == "." || value == ".."
                })
                || url.query().is_some()
                || *method != Method::GET
                || !headers.is_empty()
                || !body.is_empty()
            {
                return Err(EgressError::AuthorityScopeViolation);
            }
        }
        EgressAuthority::WikidataSparql => {
            if !is_https
                || host != "query.wikidata.org"
                || path != "/sparql"
                || *method != Method::GET
                || !headers.is_empty()
                || !body.is_empty()
            {
                return Err(EgressError::AuthorityScopeViolation);
            }
            let pairs = url.query_pairs().collect::<Vec<_>>();
            let query_values = pairs
                .iter()
                .filter(|(name, value)| name.as_ref() == "query" && !value.trim().is_empty())
                .count();
            let format_values = pairs
                .iter()
                .filter(|(name, value)| name.as_ref() == "format" && value.as_ref() == "json")
                .count();
            if pairs.len() != 2 || query_values != 1 || format_values != 1 {
                return Err(EgressError::AuthorityScopeViolation);
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
                    return Err(EgressError::AuthorityHeaderViolation(
                        name.as_str().to_owned(),
                    ));
                }
            }
            let authorization_count = headers
                .get_all(reqwest::header::AUTHORIZATION)
                .iter()
                .count();
            let content_type_count = headers
                .get_all(reqwest::header::CONTENT_TYPE)
                .iter()
                .count();
            if authorization_count > 1 || content_type_count > 1 {
                return Err(EgressError::AuthorityHeaderViolation(
                    "duplicate Perplexity provider header".to_owned(),
                ));
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

fn sanitized_origin(url: &Url) -> String {
    let scheme = url.scheme();
    match url.host() {
        Some(Host::Domain(host)) => format!("{scheme}://{host}"),
        Some(Host::Ipv4(address)) => format!("{scheme}://{address}"),
        Some(Host::Ipv6(address)) => format!("{scheme}://[{address}]"),
        None => format!("{scheme}://<missing-host>"),
    }
}

fn resolve_public_redirect_target(current_url: &Url, location: &str) -> Result<Url, EgressError> {
    let next_url = current_url
        .join(location)
        .map_err(|_| EgressError::InvalidRedirectLocation)?;
    if current_url.scheme() == "https" && next_url.scheme() == "http" {
        return Err(EgressError::RedirectSchemeDowngrade);
    }
    Ok(next_url)
}

fn record_redirect_target(
    visited_urls: &mut HashSet<String>,
    next_url: &Url,
) -> Result<(), EgressError> {
    if visited_urls.insert(next_url.as_str().to_owned()) {
        Ok(())
    } else {
        Err(EgressError::RedirectLoop)
    }
}

fn authority_may_follow_redirect(authority: EgressAuthority) -> bool {
    authority == EgressAuthority::PublicWeb
}

fn is_followable_redirect(status: StatusCode) -> bool {
    matches!(
        status,
        StatusCode::MOVED_PERMANENTLY
            | StatusCode::FOUND
            | StatusCode::SEE_OTHER
            | StatusCode::TEMPORARY_REDIRECT
            | StatusCode::PERMANENT_REDIRECT
    )
}

fn validate_connected_peer(
    authorized_addresses: &[SocketAddr],
    remote_peer: Option<SocketAddr>,
) -> Result<SocketAddr, EgressError> {
    let remote_peer = remote_peer.ok_or(EgressError::RemotePeerUnavailable)?;
    if !authorized_addresses.contains(&remote_peer) {
        return Err(EgressError::RemotePeerNotAuthorized(remote_peer));
    }
    Ok(remote_peer)
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
    validate_url_policy_with_test_port(url, false)
}

fn validate_url_policy_with_test_port(url: &Url, allow_test_port: bool) -> Result<(), EgressError> {
    match url.scheme() {
        "http" if url.port_or_known_default() == Some(80) => {}
        "https" if url.port_or_known_default() == Some(443) => {}
        "http" | "https" if allow_test_port && url.port().is_some() => {}
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
            // IANA Special-Use Domain: home.arpa is scoped to the local homenet.
            || normalized == "home.arpa"
            || normalized.ends_with(".home.arpa")
            || normalized == "metadata.google.internal"
        {
            return Err(EgressError::BlockedHost);
        }
    }
    Ok(())
}

fn collect_unique_addresses_bounded(
    addresses: impl IntoIterator<Item = SocketAddr>,
) -> Vec<SocketAddr> {
    let mut unique = HashSet::new();
    for address in addresses {
        unique.insert(address);
        // 33 unique entries are sufficient to prove the policy limit is
        // exceeded. Continue through duplicates until then; never truncate a
        // potentially unsafe answer set to 32 and accidentally accept it.
        if unique.len() > MAX_DNS_ANSWER_ADDRESSES {
            break;
        }
    }
    unique.into_iter().collect()
}

async fn resolve_and_authorize(
    url: &Url,
    port: u16,
    resolver: &dyn EgressResolver,
    dns_permit: Option<OwnedSemaphorePermit>,
    allow_test_destinations: bool,
) -> Result<(Vec<SocketAddr>, bool), EgressError> {
    let host = url.host().ok_or(EgressError::MissingHost)?;
    let (mut addresses, is_domain) = match host {
        Host::Ipv4(address) => (vec![SocketAddr::new(IpAddr::V4(address), port)], false),
        Host::Ipv6(address) => (vec![SocketAddr::new(IpAddr::V6(address), port)], false),
        Host::Domain(domain) => {
            let permit = dns_permit.ok_or(EgressError::DnsCapacityUnavailable)?;
            let resolved = resolver
                .resolve(domain, port, permit)
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
    if !allow_test_destinations {
        validate_address_set(&addresses)?;
    }
    Ok((addresses, is_domain))
}

/// Deny the complete DNS answer set if any returned address is unsafe.
/// A mixed public/private answer is never partially accepted.
pub fn validate_address_set(addresses: &[SocketAddr]) -> Result<(), EgressError> {
    if addresses.is_empty() {
        return Err(EgressError::EmptyAddressSet("<empty>".to_owned()));
    }
    if addresses.len() > MAX_DNS_ANSWER_ADDRESSES {
        return Err(EgressError::DnsAnswerSetTooLarge);
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
    !DENY
        .iter()
        .any(|(network, prefix)| ipv4_in_prefix(ip, *network, *prefix))
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
        ([0x2001, 0, 0, 0, 0, 0, 0, 0], 23),      // IETF assignments incl. Teredo
        ([0x2001, 0x0002, 0, 0, 0, 0, 0, 0], 48), // benchmarking
        ([0x2001, 0x0010, 0, 0, 0, 0, 0, 0], 28), // deprecated ORCHID
        ([0x2001, 0x0020, 0, 0, 0, 0, 0, 0], 28), // ORCHIDv2
        ([0x2001, 0x0db8, 0, 0, 0, 0, 0, 0], 32), // documentation
        ([0x2002, 0, 0, 0, 0, 0, 0, 0], 16),      // 6to4 (can embed private IPv4)
        ([0x3fff, 0, 0, 0, 0, 0, 0, 0], 20),      // documentation
        ([0x5f00, 0, 0, 0, 0, 0, 0, 0], 16),      // SRv6 segment-routing SIDs
    ];
    !DENY
        .iter()
        .any(|(network, prefix)| ipv6_in_prefix(ip, *network, *prefix))
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
    let mask = if prefix == 0 {
        0
    } else {
        u32::MAX << (32 - prefix)
    };
    (ip & mask) == (network & mask)
}

fn ipv6_in_prefix(ip: &Ipv6Addr, network: [u16; 8], prefix: u8) -> bool {
    let ip = u128::from_be_bytes(ip.octets());
    let network_bytes = network
        .into_iter()
        .flat_map(u16::to_be_bytes)
        .collect::<Vec<_>>();
    let network = u128::from_be_bytes(network_bytes.try_into().expect("16-byte IPv6 network"));
    let mask = if prefix == 0 {
        0
    } else {
        u128::MAX << (128 - prefix)
    };
    (ip & mask) == (network & mask)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ip(value: &str) -> IpAddr {
        value.parse().expect("test IP literal")
    }

    #[test]
    fn response_body_bytes_keep_the_memory_lease_until_all_clones_are_dropped() {
        let semaphore = Arc::new(Semaphore::new(2));
        let permit = semaphore.clone().try_acquire_many_owned(2).unwrap();
        let body = EgressBody::new(b"leased".to_vec(), permit);

        assert_eq!(semaphore.available_permits(), 0);
        assert_eq!(body.as_slice(), b"leased");

        let bytes = body.into_bytes();
        let clone = bytes.clone();
        drop(bytes);
        assert_eq!(
            semaphore.available_permits(),
            0,
            "a shallow Bytes clone must keep the response-memory lease alive"
        );

        drop(clone);
        assert_eq!(
            semaphore.available_permits(),
            2,
            "the lease is released only after the last downstream Bytes owner drops"
        );
    }

    #[test]
    fn response_memory_reservations_use_whole_64kib_units() {
        assert_eq!(response_memory_permits(0), 1);
        assert_eq!(response_memory_permits(1), 1);
        assert_eq!(response_memory_permits(64 * 1024), 1);
        assert_eq!(response_memory_permits(64 * 1024 + 1), 2);
        assert_eq!(response_memory_permits(16 * 1024 * 1024), 256);
    }

    #[test]
    fn reqwest_client_builder_accepts_a_pinned_domain_address_set() {
        let addresses = ["93.184.216.34:443".parse().unwrap()];
        validate_address_set(&addresses).unwrap();
        assert!(
            build_pinned_client(
                "example.com",
                &addresses,
                true,
                Duration::from_secs(5),
                Duration::from_secs(30),
                &[],
            )
            .is_ok()
        );
    }

    fn test_capability(
        url: Url,
        address: SocketAddr,
        max_response_body_bytes: usize,
    ) -> EgressCapability {
        EgressCapability {
            host: url.host_str().unwrap().to_owned(),
            url,
            authority: EgressAuthority::PublicWeb,
            method: Method::GET,
            headers: HeaderMap::new(),
            body: Vec::new(),
            addresses: vec![address],
            is_domain: true,
            expires_at: std::time::Instant::now() + Duration::from_secs(30),
            max_response_body_bytes,
            max_error_response_body_bytes: 64 * 1024,
            connect_timeout: Duration::from_secs(2),
            request_timeout: Duration::from_secs(5),
            policy_version: "prism-public-egress-v1",
        }
    }

    async fn read_test_request<S>(stream: &mut S) -> Vec<u8>
    where
        S: tokio::io::AsyncRead + Unpin,
    {
        use tokio::io::AsyncReadExt;
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
        request
    }

    #[test]
    fn receipt_origin_serialization_brackets_ipv6_and_omits_paths_and_queries() {
        let ipv6 = Url::parse("https://[2606:4700:4700::1111]/secret/path?q=private").unwrap();
        assert_eq!(sanitized_origin(&ipv6), "https://[2606:4700:4700::1111]");
        let domain = Url::parse("https://example.com:443/secret?q=private").unwrap();
        assert_eq!(sanitized_origin(&domain), "https://example.com");
    }

    #[test]
    fn public_redirect_targets_resolve_relative_locations_without_downgrade() {
        let current = Url::parse("https://public.example/a/b").unwrap();
        assert_eq!(
            resolve_public_redirect_target(&current, "../next")
                .unwrap()
                .as_str(),
            "https://public.example/next"
        );
        assert_eq!(
            resolve_public_redirect_target(&current, "https://other.example/path")
                .unwrap()
                .as_str(),
            "https://other.example/path"
        );
        assert!(matches!(
            resolve_public_redirect_target(&current, "http://other.example/path"),
            Err(EgressError::RedirectSchemeDowngrade)
        ));
        assert!(matches!(
            resolve_public_redirect_target(&current, "http://[invalid"),
            Err(EgressError::InvalidRedirectLocation)
        ));
    }

    #[test]
    fn redirect_loop_detection_uses_normalized_authorized_urls() {
        let mut visited = HashSet::new();
        let first = Url::parse("https://public.example/path").unwrap();
        let second = Url::parse("https://public.example/other").unwrap();
        record_redirect_target(&mut visited, &first).unwrap();
        record_redirect_target(&mut visited, &second).unwrap();
        assert!(matches!(
            record_redirect_target(&mut visited, &first),
            Err(EgressError::RedirectLoop)
        ));
    }

    #[test]
    fn only_public_web_authority_may_follow_redirects() {
        assert!(authority_may_follow_redirect(EgressAuthority::PublicWeb));
        assert!(!authority_may_follow_redirect(EgressAuthority::BraveSearch));
        assert!(!authority_may_follow_redirect(EgressAuthority::Perplexity));
        assert!(!authority_may_follow_redirect(
            EgressAuthority::WikipediaSummary
        ));
        assert!(!authority_may_follow_redirect(
            EgressAuthority::WikidataSparql
        ));
    }

    #[tokio::test]
    async fn duplicate_location_headers_are_rejected_before_a_second_hop() {
        use tokio::io::AsyncWriteExt;

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            read_test_request(&mut stream).await;
            stream
                .write_all(
                    b"HTTP/1.1 302 Found\r\nLocation: https://public.example/first\r\nLocation: http://metadata.google.internal/\r\nContent-Length: 0\r\nConnection: close\r\n\r\n",
                )
                .await
                .unwrap();
        });

        let client = PublicEgressClient::default();
        let url = Url::parse(&format!(
            "http://response-cap-test.invalid:{}/",
            address.port()
        ))
        .unwrap();
        let capability = test_capability(url, address, 16);
        let result = client.execute(capability).await;
        assert!(matches!(
            result,
            Err(EgressError::AmbiguousRedirectLocation)
        ));
        server.await.unwrap();
    }

    #[tokio::test]
    async fn public_web_redirect_to_metadata_is_rejected_before_second_connection() {
        use tokio::io::AsyncWriteExt;

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            read_test_request(&mut stream).await;
            stream.write_all(b"HTTP/1.1 302 Found\r\n").await.unwrap();
            stream
                .write_all(b"Location: http://metadata.google.internal/\r\n")
                .await
                .unwrap();
            stream
                .write_all(b"Content-Length: 0\r\nConnection: close\r\n\r\n")
                .await
                .unwrap();
        });

        let client = PublicEgressClient::default();
        let url = Url::parse(&format!(
            "http://response-cap-test.invalid:{}/",
            address.port()
        ))
        .unwrap();
        let capability = test_capability(url, address, 16);
        let result = client.execute(capability).await;
        assert!(matches!(result, Err(EgressError::BlockedHost)));
        server.await.unwrap();
    }

    #[tokio::test]
    async fn authorize_pins_resolver_snapshot_through_socket_peer_receipt() {
        use tokio::io::AsyncWriteExt;

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let (mut stream, peer) = listener.accept().await.unwrap();
            let request = read_test_request(&mut stream).await;
            stream
                .write_all(b"HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\nok")
                .await
                .unwrap();
            (peer, request)
        });

        let attacker_listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let attacker_address = attacker_listener.local_addr().unwrap();
        let resolver = RebindingResolver {
            first_answers: vec![address],
            // A second lookup deliberately changes to a different private peer.
            subsequent_answers: vec![attacker_address],
            calls: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
        };
        let client =
            PublicEgressClient::with_test_resolver(EgressLimits::default(), resolver.clone());
        let raw_url = format!("http://prism-e2e.test:{}/probe", address.port());

        // This is the public authorization API, with only the destination
        // classification/port exception enabled by a cfg(test)-only field.
        let capability = client
            .authorize(Method::GET, &raw_url, HeaderMap::new(), Vec::new())
            .await
            .unwrap();
        assert_eq!(capability.addresses, vec![address]);

        // Execution must use the exact injected DNS snapshot, connect to that
        // socket, preserve the original hostname in Host, and attest the peer.
        let response = client.execute(capability).await.unwrap();
        assert_eq!(response.body.as_slice(), b"ok");
        assert_eq!(response.receipt.authorized_addresses, vec![address]);
        assert_eq!(response.receipt.remote_peer, address);
        assert_eq!(
            resolver.calls.load(std::sync::atomic::Ordering::SeqCst),
            1,
            "execution must not perform a second DNS lookup"
        );

        let (peer, request) = server.await.unwrap();
        assert_eq!(peer, address);
        let unexpected_connection =
            tokio::time::timeout(Duration::from_millis(100), attacker_listener.accept()).await;
        assert!(
            unexpected_connection.is_err(),
            "a second connection reached the resolver's rebinding answer"
        );
        let request = String::from_utf8_lossy(&request);
        assert!(
            request.contains(&format!("Host: prism-e2e.test:{}", address.port())),
            "HTTP Host must retain the original hostname: {request}"
        );
    }

    #[derive(Clone, Default)]
    struct LoopbackResolver {
        calls: Arc<std::sync::Mutex<Vec<(String, u16)>>>,
    }

    impl EgressResolver for LoopbackResolver {
        fn resolve<'a>(
            &'a self,
            host: &'a str,
            port: u16,
            _permit: OwnedSemaphorePermit,
        ) -> Pin<Box<dyn Future<Output = io::Result<Vec<SocketAddr>>> + Send + 'a>> {
            let host = host.to_owned();
            let calls = Arc::clone(&self.calls);
            Box::pin(async move {
                calls.lock().expect("resolver call lock").push((host, port));
                Ok(vec![SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), port)])
            })
        }
    }

    #[tokio::test]
    async fn public_redirect_reauthorizes_and_pins_each_hop_dns_snapshot() {
        use tokio::io::AsyncWriteExt;

        let first_listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let first_address = first_listener.local_addr().unwrap();
        let second_listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let second_address = second_listener.local_addr().unwrap();

        let first_server = tokio::spawn(async move {
            let (mut stream, _) = first_listener.accept().await.unwrap();
            let request = read_test_request(&mut stream).await;
            let location = format!("http://prism-redirect.test:{}/final", second_address.port());
            let response = format!(
                "HTTP/1.1 302 Found\\r\\nLocation: {location}\\r\\nContent-Length: 0\\r\\nConnection: close\\r\\n\\r\\n"
            );
            stream.write_all(response.as_bytes()).await.unwrap();
            request
        });

        let second_server = tokio::spawn(async move {
            let (mut stream, _) = second_listener.accept().await.unwrap();
            let request = read_test_request(&mut stream).await;
            stream
                .write_all(
                    b"HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\nok",
                )
                .await
                .unwrap();
            request
        });

        let resolver = LoopbackResolver::default();
        let client =
            PublicEgressClient::with_test_resolver(EgressLimits::default(), resolver.clone());
        let first_url = format!("http://prism-redirect.test:{}/start", first_address.port());
        let capability = client
            .authorize(Method::GET, &first_url, HeaderMap::new(), Vec::new())
            .await
            .unwrap();

        let response = client.execute(capability).await.unwrap();
        assert_eq!(response.status, StatusCode::OK);
        assert_eq!(response.body.as_slice(), b"ok");
        assert_eq!(response.receipt.remote_peer, second_address);
        assert_eq!(response.receipt.authorized_addresses, vec![second_address]);
        assert_eq!(response.receipt.redirect_hops.len(), 1);

        let hop = &response.receipt.redirect_hops[0];
        assert_eq!(
            hop.from_origin,
            format!("http://prism-redirect.test:{}", first_address.port())
        );
        assert_eq!(
            hop.to_origin,
            format!("http://prism-redirect.test:{}", second_address.port())
        );
        assert_eq!(hop.status, StatusCode::FOUND.as_u16());
        assert_eq!(hop.remote_peer, first_address);
        assert_eq!(hop.target_addresses, vec![second_address]);

        let resolver_calls = resolver.calls.lock().expect("resolver call lock").clone();
        assert_eq!(
            resolver_calls,
            vec![
                ("prism-redirect.test".to_owned(), first_address.port()),
                ("prism-redirect.test".to_owned(), second_address.port()),
            ],
            "each redirect hop must receive a fresh DNS authorization"
        );

        let first_request = String::from_utf8_lossy(&first_server.await.unwrap());
        assert!(first_request.contains(&format!(
            "Host: prism-redirect.test:{}",
            first_address.port()
        )));
        let second_request = String::from_utf8_lossy(&second_server.await.unwrap());
        assert!(second_request.contains(&format!(
            "Host: prism-redirect.test:{}",
            second_address.port()
        )));
    }

    #[tokio::test]
    async fn successful_receipt_records_observed_peer_and_authority() {
        use tokio::io::AsyncWriteExt;

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            read_test_request(&mut stream).await;
            stream
                .write_all(b"HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\nok")
                .await
                .unwrap();
        });

        let client = PublicEgressClient::default();
        let url = Url::parse(&format!(
            "http://response-cap-test.invalid:{}/",
            address.port()
        ))
        .unwrap();
        let capability = test_capability(url, address, 16);
        let response = client.execute(capability).await.unwrap();
        assert_eq!(response.body.as_slice(), b"ok");
        assert_eq!(response.receipt.remote_peer, address);
        assert_eq!(response.receipt.authority, "public-web");
        assert_eq!(response.receipt.authorized_addresses, vec![address]);
        assert_eq!(response.receipt.response_body_limit_bytes, 16);
        server.await.unwrap();
    }


    #[derive(Debug)]
    struct SniRecordingResolver {
        certificates: Vec<Arc<tokio_rustls::rustls::sign::CertifiedKey>>,
        observed_sni: Arc<std::sync::Mutex<Vec<Option<String>>>>,
    }

    impl tokio_rustls::rustls::server::ResolvesServerCert for SniRecordingResolver {
        fn resolve(
            &self,
            client_hello: tokio_rustls::rustls::server::ClientHello<'_>,
        ) -> Option<Arc<tokio_rustls::rustls::sign::CertifiedKey>> {
            let server_name = client_hello.server_name().map(str::to_owned);
            let mut observed = self.observed_sni.lock().expect("SNI observation lock");
            observed.push(server_name.clone());
            if server_name.as_deref() != Some("prism-tls.test") {
                return None;
            }
            self.certificates.get(observed.len() - 1).cloned()
        }
    }

    fn test_certified_key(
        certificate: &rcgen::CertifiedKey,
    ) -> Arc<tokio_rustls::rustls::sign::CertifiedKey> {
        use tokio_rustls::rustls::pki_types::{PrivateKeyDer, PrivatePkcs8KeyDer};

        let provider = tokio_rustls::rustls::crypto::aws_lc_rs::default_provider();
        let key = PrivateKeyDer::Pkcs8(PrivatePkcs8KeyDer::from(
            certificate.signing_key.serialize_der(),
        ));
        Arc::new(
            tokio_rustls::rustls::sign::CertifiedKey::from_der(
                vec![certificate.cert.der().clone()],
                key,
                &provider,
            )
            .expect("valid local TLS test certificate and key"),
        )
    }

    #[tokio::test]
    async fn pinned_tls_uses_original_hostname_for_sni_and_certificate_verification() {
        use tokio::io::{AsyncReadExt, AsyncWriteExt};

        let valid_certificate =
            rcgen::generate_simple_self_signed(vec!["prism-tls.test".to_owned()]).unwrap();
        let wrong_name_certificate =
            rcgen::generate_simple_self_signed(vec!["different-tls.test".to_owned()]).unwrap();

        let valid_root =
            reqwest::Certificate::from_der(valid_certificate.cert.der().as_ref()).unwrap();
        // Trust the wrong-name certificate as a root too, so the second
        // handshake fails specifically on hostname verification, not trust.
        let wrong_name_root =
            reqwest::Certificate::from_der(wrong_name_certificate.cert.der().as_ref()).unwrap();

        let observed_sni = Arc::new(std::sync::Mutex::new(Vec::new()));
        let resolver = SniRecordingResolver {
            certificates: vec![
                test_certified_key(&valid_certificate),
                test_certified_key(&wrong_name_certificate),
            ],
            observed_sni: Arc::clone(&observed_sni),
        };
        let server_config = tokio_rustls::rustls::ServerConfig::builder()
            .with_no_client_auth()
            .with_cert_resolver(Arc::new(resolver));
        let acceptor = tokio_rustls::TlsAcceptor::from(Arc::new(server_config));

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let mut successful_request = None;
            for connection_index in 0..2 {
                let (stream, _) = listener.accept().await.unwrap();
                if let Ok(mut tls_stream) = acceptor.accept(stream).await {
                    if connection_index == 0 {
                        successful_request = Some(read_test_request(&mut tls_stream).await);
                        tls_stream
                            .write_all(
                                b"HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\nok",
                            )
                            .await
                            .unwrap();
                    } else {
                        // The client should reject the trusted certificate
                        // whose SAN does not match the authorized URL host.
                        let mut buffer = [0u8; 1024];
                        let _ = tls_stream.read(&mut buffer).await;
                    }
                }
            }
            successful_request
        });

        let test_resolver = LoopbackResolver::default();
        let client = PublicEgressClient::with_test_resolver(
            EgressLimits::default(),
            test_resolver,
        )
        .with_test_root_certificates(vec![valid_root, wrong_name_root]);

        let valid_url = format!("https://prism-tls.test:{}/probe", address.port());
        let valid_capability = client
            .authorize(Method::GET, &valid_url, HeaderMap::new(), Vec::new())
            .await
            .unwrap();
        let response = client.execute(valid_capability).await.unwrap();
        assert_eq!(response.status, StatusCode::OK);
        assert_eq!(response.body.as_slice(), b"ok");
        assert_eq!(response.receipt.remote_peer, address);

        let wrong_name_capability = client
            .authorize(Method::GET, &valid_url, HeaderMap::new(), Vec::new())
            .await
            .unwrap();
        let wrong_name_result = client.execute(wrong_name_capability).await;
        assert!(
            matches!(wrong_name_result, Err(EgressError::Http(_))),
            "a trusted certificate with the wrong DNS SAN must be rejected"
        );

        let request = String::from_utf8_lossy(&server.await.unwrap().unwrap());
        assert!(request.contains(&format!("Host: prism-tls.test:{}", address.port())));
        let observed_sni = observed_sni.lock().expect("SNI observation lock").clone();
        assert_eq!(
            observed_sni,
            vec![
                Some("prism-tls.test".to_owned()),
                Some("prism-tls.test".to_owned()),
            ],
            "both handshakes must use the original URL hostname as SNI"
        );
    }

    #[tokio::test]
    async fn http1_parser_rejects_response_header_count_over_transport_limit() {
        use tokio::io::AsyncWriteExt;

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            read_test_request(&mut stream).await;

            let mut response = b"HTTP/1.1 200 OK\r\n".to_vec();
            for index in 0..=MAX_RESPONSE_HEADER_COUNT {
                response.extend_from_slice(format!("X-Prism-Test-{index}: value\r\n").as_bytes());
            }
            response.extend_from_slice(b"Content-Length: 0\r\nConnection: close\r\n\r\n");
            stream.write_all(&response).await.unwrap();
        });

        let client = build_pinned_client(
            "prism-http1-header-limit.test",
            &[address],
            true,
            Duration::from_secs(2),
            Duration::from_secs(5),
            &[],
        )
        .unwrap();
        let url = format!("http://prism-http1-header-limit.test:{}/", address.port());
        let result = client.get(&url).send().await;
        assert!(
            result.is_err(),
            "HTTP/1 parser must reject a response above {MAX_RESPONSE_HEADER_COUNT} headers"
        );
        server.await.unwrap();
    }

    #[tokio::test]
    async fn declared_oversized_content_length_is_rejected_before_body_read() {
        use tokio::io::AsyncWriteExt;

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            read_test_request(&mut stream).await;
            stream
                .write_all(b"HTTP/1.1 200 OK\r\nContent-Length: 100\r\nConnection: close\r\n\r\n")
                .await
                .unwrap();
        });

        let client = PublicEgressClient::default();
        let url = Url::parse(&format!(
            "http://response-cap-test.invalid:{}/",
            address.port()
        ))
        .unwrap();
        let capability = test_capability(url, address, 16);
        let result = client.execute(capability).await;
        assert!(matches!(result, Err(EgressError::ResponseBodyTooLarge)));
        server.await.unwrap();
    }

    #[tokio::test]
    async fn non_success_responses_use_the_smaller_diagnostic_body_budget() {
        use tokio::io::AsyncWriteExt;

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            read_test_request(&mut stream).await;
            stream
                .write_all(
                    b"HTTP/1.1 500 Internal Server Error\r\nContent-Length: 5\r\nConnection: close\r\n\r\nerror",
                )
                .await
                .unwrap();
        });

        let client = PublicEgressClient::default();
        let url = Url::parse(&format!(
            "http://response-cap-test.invalid:{}/",
            address.port()
        ))
        .unwrap();
        let mut capability = test_capability(url, address, 16);
        capability.max_error_response_body_bytes = 4;
        let result = client.execute(capability).await;
        assert!(matches!(result, Err(EgressError::ResponseBodyTooLarge)));
        server.await.unwrap();
    }

    #[tokio::test]
    async fn streamed_chunked_body_stops_at_the_configured_cap() {
        use tokio::io::AsyncWriteExt;

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            read_test_request(&mut stream).await;
            stream
                .write_all(
                    b"HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\nConnection: close\r\n\r\n",
                )
                .await
                .unwrap();
            stream.write_all(b"5\r\nhello\r\n0\r\n\r\n").await.unwrap();
        });

        let client = PublicEgressClient::default();
        let url = Url::parse(&format!(
            "http://response-cap-test.invalid:{}/",
            address.port()
        ))
        .unwrap();
        let capability = test_capability(url, address, 4);
        let result = client.execute(capability).await;
        assert!(matches!(result, Err(EgressError::ResponseBodyTooLarge)));
        server.await.unwrap();
    }

    #[tokio::test]
    async fn gzip_decompression_is_subject_to_the_response_body_budget() {
        use tokio::io::AsyncWriteExt;

        // 44 compressed bytes expand to 8 KiB. The configured response budget
        // is 1 KiB, so the decoded stream must be rejected even though the
        // wire Content-Length is far below the limit.
        const GZIP_BOMB: [u8; 44] = [
            31, 139, 8, 0, 0, 0, 0, 0, 2, 255, 237, 193, 1, 13, 0, 0, 0, 194, 160, 108, 239, 95,
            202, 28, 110, 64, 1, 0, 0, 0, 0, 0, 0, 0, 239, 6, 204, 59, 37, 50, 0, 32, 0, 0,
        ];
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            read_test_request(&mut stream).await;
            stream
                .write_all(
                    b"HTTP/1.1 200 OK\r\nContent-Encoding: gzip\r\nContent-Length: 44\r\nConnection: close\r\n\r\n",
                )
                .await
                .unwrap();
            stream.write_all(&GZIP_BOMB).await.unwrap();
        });

        let client = PublicEgressClient::default();
        let url = Url::parse(&format!(
            "http://response-cap-test.invalid:{}/",
            address.port()
        ))
        .unwrap();
        let capability = test_capability(url, address, 1024);
        let result = client.execute(capability).await;
        assert!(
            matches!(result, Err(EgressError::ResponseBodyTooLarge)),
            "decoded 8 KiB response must exceed the 1 KiB cap: {result:?}"
        );
        server.await.unwrap();
    }

    #[test]
    fn peer_attestation_requires_the_actual_socket_peer_in_the_authorized_set() {
        let authorized = [
            "93.184.216.34:443".parse().unwrap(),
            "2606:4700:4700::1111:443".parse().unwrap(),
        ];
        let expected: SocketAddr = "93.184.216.34:443".parse().unwrap();
        assert_eq!(
            validate_connected_peer(&authorized, Some(expected)).unwrap(),
            expected
        );

        let unexpected: SocketAddr = "10.0.0.9:443".parse().unwrap();
        assert!(matches!(
            validate_connected_peer(&authorized, Some(unexpected)),
            Err(EgressError::RemotePeerNotAuthorized(address)) if address == unexpected
        ));
        assert!(matches!(
            validate_connected_peer(&authorized, None),
            Err(EgressError::RemotePeerUnavailable)
        ));
    }

    #[test]
    fn response_header_budget_counts_all_header_names_and_values() {
        let mut headers = HeaderMap::new();
        let oversized_value = vec![b'a'; 64 * 1024 + 1];
        headers.insert(
            "x-large",
            reqwest::header::HeaderValue::from_bytes(&oversized_value).unwrap(),
        );
        assert!(response_header_bytes(&headers) > 64 * 1024);
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
            stream.write_all(b"HTTP/1.1 302 Found\r\n").await.unwrap();
            stream
                .write_all(b"Location: http://metadata.google.internal/\r\n")
                .await
                .unwrap();
            stream
                .write_all(b"Content-Length: 0\r\nConnection: close\r\n\r\n")
                .await
                .unwrap();
        });

        let client = build_pinned_client(
            "prism-redirect-test.invalid",
            &[address],
            true,
            Duration::from_secs(2),
            Duration::from_secs(5),
            &[],
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
            &[],
        )
        .unwrap();
        let url = format!("http://prism-pinned-test.invalid:{}/probe", address.port());
        let response = client.get(&url).send().await.unwrap();
        assert_eq!(response.status(), StatusCode::OK);
        assert_eq!(response.bytes().await.unwrap().as_ref(), b"ok");

        let (peer, request) = server.await.unwrap();
        assert_eq!(peer.ip(), IpAddr::V4(Ipv4Addr::LOCALHOST));
        let request = String::from_utf8_lossy(&request);
        assert!(request.contains(&format!(
            "Host: prism-pinned-test.invalid:{}",
            address.port()
        )));
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
            assert!(
                is_public_unicast_ip(&ip(address)),
                "unexpectedly denied {address}"
            );
        }
    }

    #[test]
    fn denies_ipv4_special_and_non_global_ranges() {
        for address in [
            "0.1.2.3",
            "10.1.2.3",
            "100.64.0.1",
            "127.0.0.1",
            "169.254.169.254",
            "172.16.0.1",
            "192.0.0.1",
            "192.0.2.1",
            "192.88.99.1",
            "192.168.1.1",
            "198.18.0.1",
            "198.51.100.1",
            "203.0.113.1",
            "224.0.0.1",
            "240.0.0.1",
            "255.255.255.255",
        ] {
            assert!(
                !is_public_unicast_ip(&ip(address)),
                "unexpectedly allowed {address}"
            );
        }
    }

    #[test]
    fn denies_ipv6_special_transition_and_non_global_ranges() {
        for address in [
            "::",
            "::1",
            "::ffff:8.8.8.8",
            "64:ff9b::808:808",
            "64:ff9b:1::1",
            "100::1",
            "2001::1",
            "2001:2::1",
            "2001:10::1",
            "2001:20::1",
            "2001:db8::1",
            "2002::1",
            "3fff::1",
            "5f00::1",
            "fc00::1",
            "fe80::1",
            "ff02::1",
        ] {
            assert!(
                !is_public_unicast_ip(&ip(address)),
                "unexpectedly allowed {address}"
            );
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
            "http://home.arpa/",
            "http://router.home.arpa/",
            "http://printer.home.arpa./",
        ] {
            assert!(
                matches!(
                    validate_url_policy(&Url::parse(raw).unwrap()),
                    Err(EgressError::BlockedHost)
                ),
                "expected blocked host: {raw}"
            );
        }
    }

    #[derive(Clone)]
    struct StaticResolver {
        answers: Vec<SocketAddr>,
        calls: Arc<std::sync::atomic::AtomicUsize>,
    }

    #[derive(Clone)]
    struct RebindingResolver {
        first_answers: Vec<SocketAddr>,
        subsequent_answers: Vec<SocketAddr>,
        calls: Arc<std::sync::atomic::AtomicUsize>,
    }

    impl EgressResolver for RebindingResolver {
        fn resolve<'a>(
            &'a self,
            _host: &'a str,
            _port: u16,
            _permit: OwnedSemaphorePermit,
        ) -> Pin<Box<dyn Future<Output = io::Result<Vec<SocketAddr>>> + Send + 'a>> {
            let call = self.calls.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            let answers = if call == 0 {
                self.first_answers.clone()
            } else {
                self.subsequent_answers.clone()
            };
            Box::pin(async move { Ok(answers) })
        }
    }

    impl EgressResolver for StaticResolver {
        fn resolve<'a>(
            &'a self,
            _host: &'a str,
            _port: u16,
            _permit: OwnedSemaphorePermit,
        ) -> Pin<Box<dyn Future<Output = io::Result<Vec<SocketAddr>>> + Send + 'a>> {
            self.calls.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            let answers = self.answers.clone();
            Box::pin(async move { Ok(answers) })
        }
    }

    fn socket_addresses(values: &[&str]) -> Vec<SocketAddr> {
        values.iter().map(|value| value.parse().unwrap()).collect()
    }

    #[test]
    fn system_resolver_collection_deduplicates_but_keeps_overflow_sentinel() {
        let duplicate = "93.184.216.34:443".parse().unwrap();
        let mut inputs = vec![duplicate; 100];
        inputs.extend(
            (1..=MAX_DNS_ANSWER_ADDRESSES + 1).map(|last| {
                SocketAddr::new(IpAddr::V4(Ipv4Addr::new(93, 184, 217, last as u8)), 443)
            }),
        );
        let collected = collect_unique_addresses_bounded(inputs);
        assert_eq!(collected.len(), MAX_DNS_ANSWER_ADDRESSES + 1);
        assert!(matches!(
            validate_address_set(&collected),
            Err(EgressError::DnsAnswerSetTooLarge)
        ));
    }

    #[test]
    fn rejects_dns_answer_sets_over_the_count_limit() {
        let addresses = (1..=(MAX_DNS_ANSWER_ADDRESSES + 1))
            .map(|last| SocketAddr::new(IpAddr::V4(Ipv4Addr::new(93, 184, 216, last as u8)), 443))
            .collect::<Vec<_>>();
        assert!(matches!(
            validate_address_set(&addresses),
            Err(EgressError::DnsAnswerSetTooLarge)
        ));
    }

    #[tokio::test]
    async fn cancelled_dns_waiter_keeps_owned_permit_until_blocking_work_exits() {
        let semaphore = Arc::new(Semaphore::new(1));
        let permit = semaphore.clone().acquire_owned().await.unwrap();
        let (started_tx, started_rx) = tokio::sync::oneshot::channel();
        let (finished_tx, finished_rx) = tokio::sync::oneshot::channel();
        let mut work = Box::pin(spawn_blocking_dns_with_permit(permit, move || {
            let _ = started_tx.send(());
            std::thread::sleep(Duration::from_millis(100));
            let _ = finished_tx.send(());
            Ok(Vec::new())
        }));

        tokio::select! {
            _ = started_rx => {}
            result = &mut work => panic!("blocking worker finished before cancellation: {result:?}"),
        }

        let result = tokio::time::timeout(Duration::from_millis(10), &mut work).await;
        assert!(result.is_err(), "async resolver waiter should time out");
        drop(work);
        assert_eq!(
            semaphore.available_permits(),
            0,
            "started blocking DNS work must retain its owned admission permit after waiter cancellation"
        );

        tokio::time::timeout(Duration::from_secs(1), finished_rx)
            .await
            .expect("blocking worker should finish")
            .expect("blocking worker completion signal");
        assert_eq!(
            semaphore.available_permits(),
            1,
            "permit must return only after blocking resolver work exits"
        );
    }

    #[tokio::test]
    async fn injected_resolver_rejects_mixed_public_and_private_answers() {
        let resolver = StaticResolver {
            answers: socket_addresses(&["93.184.216.34:443", "10.0.0.8:443"]),
            calls: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
        };
        let client = PublicEgressClient::with_resolver(EgressLimits::default(), resolver.clone());
        let result = client
            .authorize(
                Method::GET,
                "https://mixed.example/",
                HeaderMap::new(),
                Vec::new(),
            )
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
            .authorize(
                Method::GET,
                "https://snapshot.example/",
                HeaderMap::new(),
                Vec::new(),
            )
            .await
            .unwrap();
        assert_eq!(resolver.calls.load(std::sync::atomic::Ordering::SeqCst), 1);
        assert_eq!(capability.addresses.len(), 2);
        assert!(
            capability
                .addresses
                .iter()
                .any(|address| address.ip() == ip("93.184.216.34"))
        );
        assert!(
            capability
                .addresses
                .iter()
                .any(|address| address.ip() == ip("2606:4700:4700::1111"))
        );
    }

    #[test]
    fn custom_limits_cannot_raise_absolute_security_ceilings() {
        let limits = EgressLimits {
            max_url_bytes: usize::MAX,
            max_request_header_bytes: usize::MAX,
            max_request_body_bytes: usize::MAX,
            max_response_body_bytes: usize::MAX,
            max_error_response_body_bytes: usize::MAX,
            connect_timeout: Duration::MAX,
            request_timeout: Duration::MAX,
            capability_ttl: Duration::MAX,
        };
        let client = PublicEgressClient::new(limits);
        let limits = client.limits();

        assert_eq!(limits.max_url_bytes, 8 * 1024);
        assert_eq!(limits.max_request_header_bytes, 16 * 1024);
        assert_eq!(limits.max_request_body_bytes, 2 * 1024 * 1024);
        assert_eq!(limits.max_response_body_bytes, 16 * 1024 * 1024);
        assert_eq!(limits.max_error_response_body_bytes, 64 * 1024);
        assert_eq!(limits.connect_timeout, Duration::from_secs(5));
        assert_eq!(limits.request_timeout, Duration::from_secs(90));
        assert_eq!(limits.capability_ttl, Duration::from_secs(5));
    }

    #[test]
    fn authority_admission_budgets_are_disjoint_and_sum_to_global_limit() {
        assert_eq!(MAX_PUBLIC_WEB_REQUESTS, 52);
        assert_eq!(MAX_BRAVE_REQUESTS, 4);
        assert_eq!(MAX_PERPLEXITY_REQUESTS, 4);
        assert_eq!(MAX_WIKIPEDIA_REQUESTS, 2);
        assert_eq!(MAX_WIKIDATA_REQUESTS, 2);
        assert_eq!(
            MAX_PUBLIC_WEB_REQUESTS
                + MAX_BRAVE_REQUESTS
                + MAX_PERPLEXITY_REQUESTS
                + MAX_WIKIPEDIA_REQUESTS
                + MAX_WIKIDATA_REQUESTS,
            MAX_CONCURRENT_EGRESS_REQUESTS
        );
        assert!(!std::ptr::eq(
            EgressAuthority::PublicWeb.admission_semaphore(),
            EgressAuthority::BraveSearch.admission_semaphore(),
        ));
        assert!(!std::ptr::eq(
            EgressAuthority::PublicWeb.admission_semaphore(),
            EgressAuthority::Perplexity.admission_semaphore(),
        ));
        assert!(!std::ptr::eq(
            EgressAuthority::PublicWeb.admission_semaphore(),
            EgressAuthority::WikipediaSummary.admission_semaphore(),
        ));
        assert!(!std::ptr::eq(
            EgressAuthority::PublicWeb.admission_semaphore(),
            EgressAuthority::WikidataSparql.admission_semaphore(),
        ));
    }

    #[test]
    fn fixed_source_authorities_are_bound_to_exact_hosts_paths_and_query_shapes() {
        let wikipedia =
            Url::parse("https://en.wikipedia.org/api/rest_v1/page/summary/Quantum_mechanics")
                .unwrap();
        assert!(
            validate_authority_scope(
                EgressAuthority::WikipediaSummary,
                &wikipedia,
                &Method::GET,
                &HeaderMap::new(),
                &[],
            )
            .is_ok()
        );

        let wikipedia_attacker =
            Url::parse("https://attacker.example/api/rest_v1/page/summary/Quantum_mechanics")
                .unwrap();
        assert!(matches!(
            validate_authority_scope(
                EgressAuthority::WikipediaSummary,
                &wikipedia_attacker,
                &Method::GET,
                &HeaderMap::new(),
                &[],
            ),
            Err(EgressError::AuthorityScopeViolation)
        ));
        let wikipedia_nested =
            Url::parse("https://en.wikipedia.org/api/rest_v1/page/summary/Quantum_mechanics/other")
                .unwrap();
        assert!(matches!(
            validate_authority_scope(
                EgressAuthority::WikipediaSummary,
                &wikipedia_nested,
                &Method::GET,
                &HeaderMap::new(),
                &[],
            ),
            Err(EgressError::AuthorityScopeViolation)
        ));

        let wikidata = Url::parse(
            "https://query.wikidata.org/sparql?query=SELECT%20%2A%20WHERE%20%7B%7D&format=json",
        )
        .unwrap();
        assert!(
            validate_authority_scope(
                EgressAuthority::WikidataSparql,
                &wikidata,
                &Method::GET,
                &HeaderMap::new(),
                &[],
            )
            .is_ok()
        );

        let wikidata_extra = Url::parse(
            "https://query.wikidata.org/sparql?query=SELECT%20%2A%20WHERE%20%7B%7D&format=json&redirect=https://attacker.example",
        )
        .unwrap();
        assert!(matches!(
            validate_authority_scope(
                EgressAuthority::WikidataSparql,
                &wikidata_extra,
                &Method::GET,
                &HeaderMap::new(),
                &[],
            ),
            Err(EgressError::AuthorityScopeViolation)
        ));
    }

    #[test]
    fn public_web_authority_rejects_headers_and_request_bodies() {
        let url = Url::parse("https://attacker.example/collect").unwrap();
        let mut headers = HeaderMap::new();
        headers.insert(
            reqwest::header::AUTHORIZATION,
            "Bearer secret".parse().unwrap(),
        );
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
        assert!(
            validate_authority_scope(
                EgressAuthority::BraveSearch,
                &url,
                &Method::GET,
                &headers,
                &[],
            )
            .is_ok()
        );

        let attacker = Url::parse("https://attacker.example/res/v1/web/search").unwrap();
        assert!(matches!(
            validate_authority_scope(
                EgressAuthority::BraveSearch,
                &attacker,
                &Method::GET,
                &headers,
                &[],
            ),
            Err(EgressError::AuthorityScopeViolation)
        ));

        let wrong_path = Url::parse("https://api.search.brave.com/other").unwrap();
        assert!(matches!(
            validate_authority_scope(
                EgressAuthority::BraveSearch,
                &wrong_path,
                &Method::GET,
                &headers,
                &[],
            ),
            Err(EgressError::AuthorityScopeViolation)
        ));
    }

    #[test]
    fn provider_authorities_reject_duplicate_credential_headers() {
        let brave_url = Url::parse("https://api.search.brave.com/res/v1/web/search").unwrap();
        let mut brave_headers = HeaderMap::new();
        brave_headers.insert("X-Subscription-Token", "secret".parse().unwrap());
        brave_headers.append("X-Subscription-Token", "second".parse().unwrap());
        assert!(matches!(
            validate_authority_scope(
                EgressAuthority::BraveSearch,
                &brave_url,
                &Method::GET,
                &brave_headers,
                &[],
            ),
            Err(EgressError::AuthorityHeaderViolation(_))
        ));

        let perplexity_url = Url::parse("https://api.perplexity.ai/chat/completions").unwrap();
        let mut perplexity_headers = HeaderMap::new();
        perplexity_headers.insert(
            reqwest::header::AUTHORIZATION,
            "Bearer secret".parse().unwrap(),
        );
        perplexity_headers.append(
            reqwest::header::AUTHORIZATION,
            "Bearer second".parse().unwrap(),
        );
        perplexity_headers.insert(
            reqwest::header::CONTENT_TYPE,
            "application/json".parse().unwrap(),
        );
        assert!(matches!(
            validate_authority_scope(
                EgressAuthority::Perplexity,
                &perplexity_url,
                &Method::POST,
                &perplexity_headers,
                b"{}",
            ),
            Err(EgressError::AuthorityHeaderViolation(_))
        ));
    }

    #[test]
    fn perplexity_authority_is_fixed_to_exact_origin_path_and_header_set() {
        let url = Url::parse("https://api.perplexity.ai/chat/completions").unwrap();
        let mut headers = HeaderMap::new();
        headers.insert(
            reqwest::header::AUTHORIZATION,
            "Bearer secret".parse().unwrap(),
        );
        headers.insert(
            reqwest::header::CONTENT_TYPE,
            "application/json".parse().unwrap(),
        );
        assert!(
            validate_authority_scope(
                EgressAuthority::Perplexity,
                &url,
                &Method::POST,
                &headers,
                b"{}",
            )
            .is_ok()
        );

        let attacker = Url::parse("https://attacker.example/chat/completions").unwrap();
        assert!(matches!(
            validate_authority_scope(
                EgressAuthority::Perplexity,
                &attacker,
                &Method::POST,
                &headers,
                b"{}",
            ),
            Err(EgressError::AuthorityScopeViolation)
        ));

        let mut extra = headers.clone();
        extra.insert("x-forwarded-host", "attacker.example".parse().unwrap());
        assert!(matches!(
            validate_authority_scope(
                EgressAuthority::Perplexity,
                &url,
                &Method::POST,
                &extra,
                b"{}",
            ),
            Err(EgressError::AuthorityHeaderViolation(_))
        ));
    }

    #[tokio::test]
    async fn capability_binds_the_exact_headers_and_body() {
        let resolver = StaticResolver {
            answers: socket_addresses(&["93.184.216.34:443"]),
            calls: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
        };
        let client = PublicEgressClient::with_resolver(EgressLimits::default(), resolver);
        let mut headers = HeaderMap::new();
        headers.insert(
            reqwest::header::AUTHORIZATION,
            "Bearer secret".parse().unwrap(),
        );
        headers.insert(
            reqwest::header::CONTENT_TYPE,
            "application/json".parse().unwrap(),
        );
        let capability = client
            .authorize_with_authority(
                EgressAuthority::Perplexity,
                Method::POST,
                "https://api.perplexity.ai/chat/completions",
                headers.clone(),
                b"exact-payload".to_vec(),
            )
            .await
            .unwrap();

        assert_eq!(
            capability.url.as_str(),
            "https://api.perplexity.ai/chat/completions"
        );
        assert_eq!(capability.authority, EgressAuthority::Perplexity);
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
    async fn rejects_alternate_ipv4_literal_encodings() {
        let client = PublicEgressClient::default();
        for raw in [
            "http://127.0.0.1/",
            "http://0x7f.0.0.1/",
            "http://0177.0.0.1/",
            "http://2130706433/",
            "http://0x7f000001/",
        ] {
            let result = client
                .authorize(Method::GET, raw, HeaderMap::new(), Vec::new())
                .await;
            assert!(
                matches!(
                    result,
                    Err(EgressError::InvalidUrl(_)) | Err(EgressError::AddressNotPermitted(_))
                ),
                "alternate loopback encoding escaped policy: {raw}"
            );
        }
    }

    #[tokio::test]
    async fn rejects_private_ip_literals_before_transport() {
        let client = PublicEgressClient::default();
        let result = client.authorize(
            Method::GET,
            "http://127.0.0.1/",
            HeaderMap::new(),
            Vec::new(),
        );
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
            .authorize(
                Method::GET,
                "https://1.1.1.1/",
                HeaderMap::new(),
                Vec::new(),
            )
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
            assert!(
                matches!(
                    validate_request_headers(&headers),
                    Err(EgressError::ForbiddenHeader(_))
                ),
                "expected forbidden header: {name}"
            );
        }
    }

    #[test]
    fn enforces_request_header_budget() {
        let mut headers = HeaderMap::new();
        headers.insert("x-test", "long-value");
        assert!(request_header_bytes(&headers) >= "x-test".len() + "long-value".len());
    }
}
