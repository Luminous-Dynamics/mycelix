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

use bytes::Bytes;
pub use reqwest::Method;
pub use reqwest::header::HeaderMap;
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

/// Total configured capacity of the process-wide retained response-body budget.
pub const fn response_memory_budget_bytes() -> usize {
    RESPONSE_MEMORY_BUDGET_BYTES
}

/// Snapshot of response-body budget capacity not currently reserved by live bodies.
pub fn response_memory_budget_available_bytes() -> usize {
    response_memory_semaphore()
        .available_permits()
        .saturating_mul(RESPONSE_MEMORY_PERMIT_UNIT_BYTES)
}

/// Snapshot of response-body budget currently reserved by in-flight or downstream bodies.
pub fn response_memory_budget_reserved_bytes() -> usize {
    RESPONSE_MEMORY_BUDGET_BYTES.saturating_sub(response_memory_budget_available_bytes())
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
    #[error("logical request ID must be 32 lowercase hexadecimal characters")]
    InvalidLogicalRequestId,
    #[error("logical request redirect-hop index exceeds the supported ceiling")]
    LogicalRequestHopOutOfRange,
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
    #[error("resolution lease expired before request authorization")]
    ResolutionLeaseExpired,
    #[error("resolution snapshot does not match requested host, port, logical ID or hop")]
    ResolutionSnapshotMismatch,
    #[error("resolution snapshot address-set digest does not match its socket set")]
    ResolutionAddressSetDigestMismatch,
    #[error("resolution lease or resolver snapshot digest is malformed")]
    InvalidResolutionDigest,
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
///
/// The non-duplicable property is enforced by the public Rust API, not merely
/// documented as a caller convention:
///
/// ```compile_fail
/// use prism_egress::EgressCapability;
///
/// fn attempt_to_duplicate(capability: EgressCapability) {
///     let _duplicate = capability.clone();
/// }
/// ```
///
/// Construction is also intentionally unavailable outside this crate:
///
/// ```compile_fail
/// use prism_egress::EgressCapability;
///
/// fn attempt_to_forge() {
///     let _capability = EgressCapability {};
/// }
/// ```
#[must_use = "an authorized egress capability should be executed or explicitly discarded"]
pub struct EgressCapability {
    url: Url,
    host: String,
    logical_request_id: String,
    redirect_hop: u8,
    authority: EgressAuthority,
    method: Method,
    headers: HeaderMap,
    body: Vec<u8>,
    addresses: Vec<SocketAddr>,
    is_domain: bool,
    resolution_lease_digest: Option<String>,
    resolver_snapshot_digest: Option<String>,
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

/// Application-observed TLS identity outcome for one HTTPS hop.
///
/// This records the URL reference identity and expected SNI name after Reqwest
/// returned a response. It does not contain the certificate chain/fingerprint
/// and is not independent kernel or cryptographic attestation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TlsIdentityReceipt {
    pub reference_identity: String,
    pub sni_hostname: Option<String>,
    /// True only because a successful HTTPS response was returned by the
    /// configured Reqwest TLS verifier for this URL reference identity.
    pub certificate_identity_verified: bool,
}

#[derive(Clone, Debug)]
pub struct RedirectHopReceipt {
    pub logical_request_id: String,
    pub redirect_hop: u8,
    pub from_origin: String,
    pub to_origin: String,
    pub status: u16,
    pub authorized_addresses: Vec<SocketAddr>,
    pub authorized_address_set_digest: String,
    /// Signed Nixward resolution-lease payload digest, when supplied by a
    /// trusted resolver adapter. Distinct from the resolver transcript digest.
    pub resolution_lease_digest: Option<String>,
    pub resolver_snapshot_digest: Option<String>,
    pub remote_peer: SocketAddr,
    pub tls_identity: Option<TlsIdentityReceipt>,
    pub http_version: String,
    pub target_addresses: Vec<SocketAddr>,
    pub target_address_set_digest: String,
    pub target_resolution_lease_digest: Option<String>,
    pub target_resolver_snapshot_digest: Option<String>,
}

#[derive(Clone, Debug)]
pub struct EgressReceipt {
    /// Opaque 16-byte lowercase-hex correlation ID, never an authorization token.
    pub logical_request_id: String,
    /// Zero-based HTTP hop within the logical request.
    pub redirect_hop: u8,
    /// Scheme + authority only. Path, query, credentials, and response content
    /// are deliberately excluded from the receipt.
    pub origin: String,
    pub method: String,
    pub authorized_addresses: Vec<SocketAddr>,
    /// Canonical BLAKE3 digest of the complete authorized IP:port set.
    /// This is a correlation digest, not a signature or resolver attestation.
    pub authorized_address_set_digest: String,
    /// Signed Nixward resolution-lease payload digest when externally verified.
    pub resolution_lease_digest: Option<String>,
    pub resolver_snapshot_digest: Option<String>,
    pub remote_peer: SocketAddr,
    pub tls_identity: Option<TlsIdentityReceipt>,
    pub http_version: String,
    pub status: u16,
    pub request_body_bytes: usize,
    pub response_body_bytes: usize,
    pub response_body_limit_bytes: usize,
    pub elapsed: Duration,
    pub policy_version: &'static str,
    pub authority: &'static str,
    pub redirect_hops: Vec<RedirectHopReceipt>,
}

/// Complete address and provenance snapshot produced for one logical request hop.
///
/// This value is a transport input, not a signature. The Nixward adapter must
/// verify the signed lease, parent capability, workload/cgroup binding, replay
/// nonce and resolver provenance before constructing it with
/// `from_verified_nixward_lease`. Prism independently checks request/hop
/// identity, hostname, port, expiry, address policy and the canonical set digest.
#[derive(Clone, Debug)]
pub struct EgressResolutionSnapshot {
    hostname: String,
    port: u16,
    logical_request_id: String,
    redirect_hop: u8,
    addresses: Vec<SocketAddr>,
    valid_until: Option<std::time::Instant>,
    address_set_digest: Option<String>,
    resolution_lease_digest: Option<String>,
    resolver_snapshot_digest: Option<String>,
}

impl EgressResolutionSnapshot {
    /// Construct a snapshot from the operating-system resolver. The OS API does
    /// not expose authoritative TTL, CNAME or DNSSEC provenance, so those fields
    /// remain absent and Prism applies its own short capability lifetime.
    pub fn from_system_resolver(
        hostname: impl Into<String>,
        port: u16,
        logical_request_id: impl Into<String>,
        redirect_hop: u8,
        addresses: Vec<SocketAddr>,
    ) -> Self {
        Self {
            hostname: hostname.into(),
            port,
            logical_request_id: logical_request_id.into(),
            redirect_hop,
            addresses,
            valid_until: None,
            address_set_digest: None,
            resolution_lease_digest: None,
            resolver_snapshot_digest: None,
        }
    }

    /// Construct from a Nixward lease that the resolver adapter has already
    /// cryptographically verified and bound to the calling workload/policy.
    ///
    /// This constructor validates field shape and digest consistency; it does
    /// not verify a signature or establish caller/cgroup identity by itself.
    #[allow(clippy::too_many_arguments)]
    pub fn from_verified_nixward_lease(
        hostname: impl Into<String>,
        port: u16,
        logical_request_id: impl Into<String>,
        redirect_hop: u8,
        addresses: Vec<SocketAddr>,
        valid_until: std::time::Instant,
        address_set_digest: impl Into<String>,
        resolution_lease_digest: impl Into<String>,
        resolver_snapshot_digest: impl Into<String>,
    ) -> Result<Self, EgressError> {
        let hostname = hostname.into();
        let logical_request_id = logical_request_id.into();
        let address_set_digest = address_set_digest.into();
        let resolution_lease_digest = resolution_lease_digest.into();
        let resolver_snapshot_digest = resolver_snapshot_digest.into();

        validate_logical_request_id(&logical_request_id)?;
        if hostname.is_empty() || port == 0 || redirect_hop > MAX_PUBLIC_WEB_REDIRECT_HOPS as u8 {
            return Err(EgressError::ResolutionSnapshotMismatch);
        }
        if addresses.is_empty() || addresses.len() > MAX_DNS_ANSWER_ADDRESSES {
            return Err(EgressError::EmptyAddressSet(hostname));
        }
        if addresses.iter().any(|address| address.port() != port) {
            return Err(EgressError::ResolutionSnapshotMismatch);
        }
        if valid_until <= std::time::Instant::now() {
            return Err(EgressError::ResolutionLeaseExpired);
        }
        for digest in [
            address_set_digest.as_str(),
            resolution_lease_digest.as_str(),
            resolver_snapshot_digest.as_str(),
        ] {
            if !is_lower_hex_digest(digest) {
                return Err(EgressError::InvalidResolutionDigest);
            }
        }
        if canonical_address_set_digest(&addresses) != address_set_digest {
            return Err(EgressError::ResolutionAddressSetDigestMismatch);
        }

        Ok(Self {
            hostname,
            port,
            logical_request_id,
            redirect_hop,
            addresses,
            valid_until: Some(valid_until),
            address_set_digest: Some(address_set_digest),
            resolution_lease_digest: Some(resolution_lease_digest),
            resolver_snapshot_digest: Some(resolver_snapshot_digest),
        })
    }
}

/// Resolver interface makes DNS policy deterministic under test and permits a
/// trusted external lease broker to supply the exact request/hop snapshot.
///
/// Implementations must return the complete A and AAAA answer set. A
/// lease-backed implementation must verify the signed Nixward lease and its
/// owner/workload scope before constructing an EgressResolutionSnapshot.
pub trait EgressResolver: Send + Sync {
    /// Resolve one host for exactly one logical request hop while owning the
    /// DNS admission permit. Implementations that delegate to non-cancellable
    /// blocking work must move this permit into that work so cancellation cannot
    /// release capacity early.
    fn resolve<'a>(
        &'a self,
        host: &'a str,
        port: u16,
        logical_request_id: &'a str,
        redirect_hop: u8,
        permit: OwnedSemaphorePermit,
    ) -> Pin<Box<dyn Future<Output = io::Result<EgressResolutionSnapshot>> + Send + 'a>>;
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
        logical_request_id: &'a str,
        redirect_hop: u8,
        permit: OwnedSemaphorePermit,
    ) -> Pin<Box<dyn Future<Output = io::Result<EgressResolutionSnapshot>> + Send + 'a>> {
        let host = host.to_owned();
        let logical_request_id = logical_request_id.to_owned();
        Box::pin(async move {
            let addresses = spawn_blocking_dns_with_permit(permit, move || {
                std::net::ToSocketAddrs::to_socket_addrs(&(host.as_str(), port))
                    .map(collect_unique_addresses_bounded)
            })
            .await?;
            Ok(EgressResolutionSnapshot::from_system_resolver(
                host,
                port,
                logical_request_id,
                redirect_hop,
                addresses,
            ))
        })
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

    /// Construct a client with an explicit resolution authority.
    ///
    /// A custom resolver is part of the trusted computing base. For Nixward
    /// integration, it must verify signed resolution leases before returning a
    /// snapshot; Prism then independently checks the exact request/hop binding,
    /// address set, expiry and digest.
    pub fn with_resolver<R>(limits: EgressLimits, resolver: R) -> Self
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
    fn with_test_root_certificates(mut self, certificates: Vec<reqwest::Certificate>) -> Self {
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
        let logical_request_id = uuid::Uuid::new_v4().simple().to_string();
        self.authorize_with_authority_and_correlation(
            authority,
            method,
            raw_url,
            headers,
            body,
            &logical_request_id,
            0,
        )
        .await
    }

    /// Authorize an exact request while preserving a caller-supplied opaque
    /// correlation ID and zero-based hop index. The ID is for joining evidence
    /// only; it grants no authority and must not be treated as a secret.
    ///
    /// This permits a future Prism↔Nixward adapter to pass the exact
    /// NetworkResolutionLeaseV1 logical_request_id/redirect_hop values.
    pub async fn authorize_with_authority_and_correlation(
        &self,
        authority: EgressAuthority,
        method: Method,
        raw_url: &str,
        headers: HeaderMap,
        body: Vec<u8>,
        logical_request_id: &str,
        redirect_hop: u8,
    ) -> Result<EgressCapability, EgressError> {
        validate_logical_request_id(logical_request_id)?;
        if redirect_hop > MAX_PUBLIC_WEB_REDIRECT_HOPS as u8 {
            return Err(EgressError::LogicalRequestHopOutOfRange);
        }
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
        let resolution = tokio::time::timeout(
            DNS_LOOKUP_TIMEOUT,
            resolve_and_authorize(
                &url,
                port,
                self.resolver.as_ref(),
                dns_permit,
                logical_request_id,
                redirect_hop,
                allow_test_destinations,
            ),
        )
        .await
        .map_err(|_| EgressError::DnsResolutionTimeout)??;
        let local_expiry = std::time::Instant::now() + self.limits.capability_ttl;
        let expires_at = resolution
            .valid_until
            .map_or(local_expiry, |valid_until| valid_until.min(local_expiry));
        if expires_at <= std::time::Instant::now() {
            return Err(EgressError::ResolutionLeaseExpired);
        }

        Ok(EgressCapability {
            url,
            host,
            logical_request_id: logical_request_id.to_owned(),
            redirect_hop,
            authority,
            method,
            headers,
            body,
            addresses: resolution.addresses,
            is_domain: resolution.is_domain,
            resolution_lease_digest: resolution.resolution_lease_digest,
            resolver_snapshot_digest: resolution.resolver_snapshot_digest,
            expires_at,
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
            let authorized_address_set_digest =
                canonical_address_set_digest(&authorized_addresses);
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
            // A successful HTTPS response means Reqwest's configured TLS
            // verifier accepted the URL reference identity. Record only that
            // application-level result; do not imply certificate attestation.
            let tls_identity = tls_identity_receipt(&capability.url);
            let http_version = format!("{:?}", response.version());
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
                        .authorize_with_authority_and_correlation(
                            EgressAuthority::PublicWeb,
                            Method::GET,
                            next_url.as_str(),
                            HeaderMap::new(),
                            Vec::new(),
                            &capability.logical_request_id,
                            capability.redirect_hop.saturating_add(1),
                        )
                        .await?;

                    let to_origin = sanitized_origin(&next_capability.url);
                    let target_addresses = next_capability.addresses.clone();
                    let target_address_set_digest =
                        canonical_address_set_digest(&target_addresses);
                    redirect_hops.push(RedirectHopReceipt {
                        logical_request_id: capability.logical_request_id.clone(),
                        redirect_hop: capability.redirect_hop,
                        from_origin: origin,
                        to_origin,
                        status: status.as_u16(),
                        authorized_addresses,
                        authorized_address_set_digest,
                        resolution_lease_digest: capability.resolution_lease_digest.clone(),
                        resolver_snapshot_digest: capability.resolver_snapshot_digest.clone(),
                        remote_peer,
                        tls_identity,
                        http_version,
                        target_addresses,
                        target_address_set_digest,
                        target_resolution_lease_digest: next_capability
                            .resolution_lease_digest
                            .clone(),
                        target_resolver_snapshot_digest: next_capability
                            .resolver_snapshot_digest
                            .clone(),
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
                logical_request_id: capability.logical_request_id.clone(),
                redirect_hop: capability.redirect_hop,
                origin,
                method: method_name,
                authorized_addresses,
                authorized_address_set_digest,
                resolution_lease_digest: capability.resolution_lease_digest.clone(),
                resolver_snapshot_digest: capability.resolver_snapshot_digest.clone(),
                remote_peer,
                tls_identity,
                http_version,
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

fn validate_logical_request_id(value: &str) -> Result<(), EgressError> {
    if value.len() != 32
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(EgressError::InvalidLogicalRequestId);
    }
    Ok(())
}

fn is_lower_hex_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn tls_identity_receipt(url: &Url) -> Option<TlsIdentityReceipt> {
    if url.scheme() != "https" {
        return None;
    }
    let reference_identity = url.host_str()?.to_owned();
    let sni_hostname = match url.host()? {
        Host::Domain(domain) => Some(domain.to_owned()),
        Host::Ipv4(_) | Host::Ipv6(_) => None,
    };
    Some(TlsIdentityReceipt {
        reference_identity,
        sni_hostname,
        certificate_identity_verified: true,
    })
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

/// Canonical socket endpoint fields used for stable cross-process hashing.
/// IPv6 flowinfo is intentionally excluded: it is not part of the destination
/// authority represented by an IP address, port, and scope ID.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
enum CanonicalSocketAddress {
    V4 { ip: [u8; 4], port: u16 },
    V6 {
        ip: [u8; 16],
        port: u16,
        scope_id: u32,
    },
}

/// Return the canonical binary encoding of a complete socket-address set.
///
/// Records sort and deduplicate by address family, raw address bytes,
/// destination port, and IPv6 scope ID. IPv6 flowinfo is intentionally excluded.
pub fn canonical_address_set_encoding(addresses: &[SocketAddr]) -> Vec<u8> {
    let mut canonical: Vec<CanonicalSocketAddress> = addresses
        .iter()
        .map(|address| match address {
            SocketAddr::V4(address) => CanonicalSocketAddress::V4 {
                ip: address.ip().octets(),
                port: address.port(),
            },
            SocketAddr::V6(address) => CanonicalSocketAddress::V6 {
                ip: address.ip().octets(),
                port: address.port(),
                scope_id: address.scope_id(),
            },
        })
        .collect();
    canonical.sort_unstable();
    canonical.dedup();

    let mut encoded = Vec::with_capacity(8 + canonical.len() * 23);
    encoded.extend_from_slice(&(canonical.len() as u64).to_be_bytes());
    for address in canonical {
        match address {
            CanonicalSocketAddress::V4 { ip, port } => {
                encoded.push(4);
                encoded.extend_from_slice(&ip);
                encoded.extend_from_slice(&port.to_be_bytes());
            }
            CanonicalSocketAddress::V6 { ip, port, scope_id } => {
                encoded.push(6);
                encoded.extend_from_slice(&ip);
                encoded.extend_from_slice(&port.to_be_bytes());
                encoded.extend_from_slice(&scope_id.to_be_bytes());
            }
        }
    }
    encoded
}

/// Compute a stable, domain-separated digest for a complete socket-address set.
///
/// This digest supports cross-plane correlation; it does not authenticate the
/// resolver, lease issuer, kernel observer, or application receipt.
pub fn canonical_address_set_digest(addresses: &[SocketAddr]) -> String {
    let encoded = canonical_address_set_encoding(addresses);
    let mut hasher = blake3::Hasher::new_derive_key("org.luminousdynamics.prism.address-set.v1");
    hasher.update(&encoded);
    hasher.finalize().to_hex().to_string()
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

#[derive(Clone, Debug)]
struct AuthorizedResolution {
    addresses: Vec<SocketAddr>,
    is_domain: bool,
    valid_until: Option<std::time::Instant>,
    resolution_lease_digest: Option<String>,
    resolver_snapshot_digest: Option<String>,
}

async fn resolve_and_authorize(
    url: &Url,
    port: u16,
    resolver: &dyn EgressResolver,
    dns_permit: Option<OwnedSemaphorePermit>,
    logical_request_id: &str,
    redirect_hop: u8,
    allow_test_destinations: bool,
) -> Result<AuthorizedResolution, EgressError> {
    let host = url.host().ok_or(EgressError::MissingHost)?;
    let hostname = url.host_str().ok_or(EgressError::MissingHost)?;
    let (snapshot, is_domain) = match host {
        Host::Ipv4(address) => (
            EgressResolutionSnapshot::from_system_resolver(
                hostname,
                port,
                logical_request_id,
                redirect_hop,
                vec![SocketAddr::new(IpAddr::V4(address), port)],
            ),
            false,
        ),
        Host::Ipv6(address) => (
            EgressResolutionSnapshot::from_system_resolver(
                hostname,
                port,
                logical_request_id,
                redirect_hop,
                vec![SocketAddr::new(IpAddr::V6(address), port)],
            ),
            false,
        ),
        Host::Domain(domain) => {
            let permit = dns_permit.ok_or(EgressError::DnsCapacityUnavailable)?;
            let snapshot = resolver
                .resolve(domain, port, logical_request_id, redirect_hop, permit)
                .await
                .map_err(|source| EgressError::DnsResolution {
                    host: domain.to_owned(),
                    source,
                })?;
            (snapshot, true)
        }
    };

    if snapshot.hostname != hostname
        || snapshot.port != port
        || snapshot.logical_request_id != logical_request_id
        || snapshot.redirect_hop != redirect_hop
    {
        return Err(EgressError::ResolutionSnapshotMismatch);
    }
    if snapshot
        .valid_until
        .is_some_and(|valid_until| valid_until <= std::time::Instant::now())
    {
        return Err(EgressError::ResolutionLeaseExpired);
    }

    let mut addresses = snapshot.addresses;
    if addresses.iter().any(|address| address.port() != port) {
        return Err(EgressError::ResolutionSnapshotMismatch);
    }
    if addresses.is_empty() {
        return Err(EgressError::EmptyAddressSet(hostname.to_owned()));
    }
    addresses.sort_unstable();
    addresses.dedup();
    if addresses.len() > MAX_DNS_ANSWER_ADDRESSES {
        return Err(EgressError::DnsAnswerSetTooLarge);
    }

    if let Some(expected_digest) = snapshot.address_set_digest.as_deref() {
        if canonical_address_set_digest(&addresses) != expected_digest {
            return Err(EgressError::ResolutionAddressSetDigestMismatch);
        }
    }
    if !allow_test_destinations {
        validate_address_set(&addresses)?;
    }

    Ok(AuthorizedResolution {
        addresses,
        is_domain,
        valid_until: snapshot.valid_until,
        resolution_lease_digest: snapshot.resolution_lease_digest,
        resolver_snapshot_digest: snapshot.resolver_snapshot_digest,
    })
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

    #[derive(Clone, Debug)]
    enum WireDnsRecord {
        A(std::net::Ipv4Addr, u32),
        Aaaa(std::net::Ipv6Addr, u32),
        Cname(String, u32),
    }

    #[derive(Clone, Debug)]
    enum WireDnsAnswer {
        Records(Vec<WireDnsRecord>),
        Rebinding {
            first: Vec<WireDnsRecord>,
            subsequent: Vec<WireDnsRecord>,
        },
        TruncatedThenTcp(Vec<WireDnsRecord>),
        NxDomain,
        ServFail,
    }

    #[derive(Default)]
    struct WireDnsFixtureState {
        answers: std::collections::HashMap<String, WireDnsAnswer>,
        observed_queries: Vec<(String, u16)>,
        query_counts: std::collections::HashMap<(String, u16), usize>,
        tcp_query_counts: std::collections::HashMap<(String, u16), usize>,
    }

    struct WireDnsFixture {
        address: SocketAddr,
        state: Arc<std::sync::Mutex<WireDnsFixtureState>>,
        udp_server: tokio::task::JoinHandle<()>,
        tcp_server: tokio::task::JoinHandle<()>,
    }

    impl WireDnsFixture {
        async fn start() -> Self {
            let socket = tokio::net::UdpSocket::bind("127.0.0.1:0")
                .await
                .expect("bind hermetic DNS fixture");
            let address = socket.local_addr().expect("DNS fixture address");
            // TCP and UDP may share the same numeric port. A truncated UDP
            // response is retried against this TCP listener by Hickory.
            let tcp_listener = tokio::net::TcpListener::bind(address)
                .await
                .expect("bind DNS TCP fallback listener on the UDP port");
            let state = Arc::new(std::sync::Mutex::new(WireDnsFixtureState::default()));

            let udp_state = Arc::clone(&state);
            let udp_server = tokio::spawn(async move {
                let mut request = [0u8; 4096];
                loop {
                    let (length, peer) = match socket.recv_from(&mut request).await {
                        Ok(received) => received,
                        Err(_) => break,
                    };
                    let response = build_wire_dns_response(&request[..length], &udp_state, false);
                    if let Some(response) = response {
                        let _ = socket.send_to(&response, peer).await;
                    }
                }
            });

            let tcp_state = Arc::clone(&state);
            let tcp_server = tokio::spawn(async move {
                use tokio::io::{AsyncReadExt, AsyncWriteExt};
                loop {
                    let (mut stream, _) = match tcp_listener.accept().await {
                        Ok(accepted) => accepted,
                        Err(_) => break,
                    };
                    let server_state = Arc::clone(&tcp_state);
                    tokio::spawn(async move {
                        let mut length_bytes = [0u8; 2];
                        if stream.read_exact(&mut length_bytes).await.is_err() {
                            return;
                        }
                        let message_length = u16::from_be_bytes(length_bytes) as usize;
                        if message_length == 0 || message_length > 4096 {
                            return;
                        }
                        let mut request = vec![0u8; message_length];
                        if stream.read_exact(&mut request).await.is_err() {
                            return;
                        }
                        if let Some(response) =
                            build_wire_dns_response(&request, &server_state, true)
                        {
                            if response.len() > u16::MAX as usize {
                                return;
                            }
                            let mut frame = Vec::with_capacity(response.len() + 2);
                            frame.extend_from_slice(&(response.len() as u16).to_be_bytes());
                            frame.extend_from_slice(&response);
                            let _ = stream.write_all(&frame).await;
                        }
                    });
                }
            });

            Self {
                address,
                state,
                udp_server,
                tcp_server,
            }
        }

        fn set_answer(&self, name: &str, answer: WireDnsAnswer) {
            let normalized = normalize_wire_dns_name(name);
            self.state
                .lock()
                .expect("DNS fixture state lock")
                .answers
                .insert(normalized, answer);
        }

        fn observed_queries(&self) -> Vec<(String, u16)> {
            self.state
                .lock()
                .expect("DNS fixture state lock")
                .observed_queries
                .clone()
        }

        fn query_count(&self, name: &str, query_type: u16) -> usize {
            let key = (normalize_wire_dns_name(name), query_type);
            self.state
                .lock()
                .expect("DNS fixture state lock")
                .query_counts
                .get(&key)
                .copied()
                .unwrap_or(0)
        }

        fn tcp_query_count(&self, name: &str, query_type: u16) -> usize {
            let key = (normalize_wire_dns_name(name), query_type);
            self.state
                .lock()
                .expect("DNS fixture state lock")
                .tcp_query_counts
                .get(&key)
                .copied()
                .unwrap_or(0)
        }
    }

    impl Drop for WireDnsFixture {
        fn drop(&mut self) {
            self.udp_server.abort();
            self.tcp_server.abort();
        }
    }

    fn normalize_wire_dns_name(name: &str) -> String {
        let trimmed = name.trim_end_matches('.');
        if trimmed.is_empty() {
            ".".to_owned()
        } else {
            format!("{}.", trimmed.to_ascii_lowercase())
        }
    }

    fn encode_wire_dns_name(name: &str, output: &mut Vec<u8>) -> io::Result<()> {
        let trimmed = name.trim_end_matches('.');
        if trimmed.is_empty() {
            output.push(0);
            return Ok(());
        }
        for label in trimmed.split('.') {
            if label.is_empty() || label.len() > 63 {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "invalid DNS label in fixture",
                ));
            }
            output.push(label.len() as u8);
            output.extend_from_slice(label.as_bytes());
        }
        output.push(0);
        Ok(())
    }

    fn decode_wire_dns_name(packet: &[u8], offset: &mut usize) -> io::Result<String> {
        let mut labels = Vec::new();
        let mut cursor = *offset;
        let mut resume_offset = None;
        for _ in 0..128 {
            let length = *packet.get(cursor).ok_or_else(|| {
                io::Error::new(io::ErrorKind::UnexpectedEof, "truncated DNS name")
            })?;
            if length & 0xc0 == 0xc0 {
                let second = *packet.get(cursor + 1).ok_or_else(|| {
                    io::Error::new(io::ErrorKind::UnexpectedEof, "truncated DNS pointer")
                })?;
                let pointer = (((length & 0x3f) as usize) << 8) | second as usize;
                if pointer >= packet.len() {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        "DNS compression pointer is outside packet",
                    ));
                }
                if resume_offset.is_none() {
                    resume_offset = Some(cursor + 2);
                }
                cursor = pointer;
                continue;
            }
            if length & 0xc0 != 0 {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "unsupported DNS label encoding",
                ));
            }
            cursor += 1;
            if length == 0 {
                *offset = resume_offset.unwrap_or(cursor);
                return Ok(if labels.is_empty() {
                    ".".to_owned()
                } else {
                    format!("{}.", labels.join("."))
                });
            }
            let end = cursor.saturating_add(length as usize);
            let label = packet.get(cursor..end).ok_or_else(|| {
                io::Error::new(io::ErrorKind::UnexpectedEof, "truncated DNS label")
            })?;
            labels.push(String::from_utf8_lossy(label).to_ascii_lowercase());
            cursor = end;
        }
        Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "DNS compression pointer loop or excessive labels",
        ))
    }

    fn build_wire_dns_response(
        packet: &[u8],
        state: &Arc<std::sync::Mutex<WireDnsFixtureState>>,
        tcp: bool,
    ) -> Option<Vec<u8>> {
        if packet.len() < 12 {
            return None;
        }
        let id = [packet[0], packet[1]];
        let request_flags = u16::from_be_bytes([packet[2], packet[3]]);
        let question_count = u16::from_be_bytes([packet[4], packet[5]]);
        if question_count != 1 {
            return None;
        }
        let mut question_offset = 12;
        let name = decode_wire_dns_name(packet, &mut question_offset).ok()?;
        let qtype_bytes = packet.get(question_offset..question_offset + 2)?;
        let query_type = u16::from_be_bytes([qtype_bytes[0], qtype_bytes[1]]);
        let qclass_bytes = packet.get(question_offset + 2..question_offset + 4)?;
        let query_class = [qclass_bytes[0], qclass_bytes[1]];
        let question_end = question_offset.checked_add(4)?;
        let original_question = packet.get(12..question_end)?;
        let normalized = normalize_wire_dns_name(&name);

        let (answer, query_count) = {
            let mut state = state.lock().ok()?;
            state
                .observed_queries
                .push((normalized.clone(), query_type));
            let key = (normalized.clone(), query_type);
            let counts = if tcp {
                &mut state.tcp_query_counts
            } else {
                &mut state.query_counts
            };
            let count = counts.entry(key).or_insert(0);
            let query_count = *count;
            *count = (*count).saturating_add(1);
            (state.answers.get(&normalized).cloned(), query_count)
        };

        let (rcode, records, truncated) = match answer.unwrap_or(WireDnsAnswer::NxDomain) {
            WireDnsAnswer::NxDomain => (3u16, Vec::new(), false),
            WireDnsAnswer::ServFail => (2u16, Vec::new(), false),
            WireDnsAnswer::Records(records) => (0u16, records, false),
            WireDnsAnswer::Rebinding { first, subsequent } => (
                0u16,
                if query_count == 0 { first } else { subsequent },
                false,
            ),
            WireDnsAnswer::TruncatedThenTcp(_records) if !tcp => (0u16, Vec::new(), true),
            WireDnsAnswer::TruncatedThenTcp(records) => (0u16, records, false),
        };

        let mut answer_bytes = Vec::new();
        let mut answer_count = 0u16;
        if rcode == 0 {
            for record in records {
                let (record_type, ttl, rdata) = match record {
                    WireDnsRecord::A(address, ttl) if query_type == 1 => {
                        (1u16, ttl, address.octets().to_vec())
                    }
                    WireDnsRecord::Aaaa(address, ttl) if query_type == 28 => {
                        (28u16, ttl, address.octets().to_vec())
                    }
                    WireDnsRecord::Cname(target, ttl) => {
                        let mut encoded_target = Vec::new();
                        if encode_wire_dns_name(&target, &mut encoded_target).is_err() {
                            continue;
                        }
                        (5u16, ttl, encoded_target)
                    }
                    _ => continue,
                };
                answer_bytes.extend_from_slice(&[0xc0, 0x0c]); // owner = question name
                answer_bytes.extend_from_slice(&record_type.to_be_bytes());
                answer_bytes.extend_from_slice(&1u16.to_be_bytes()); // IN
                answer_bytes.extend_from_slice(&ttl.to_be_bytes());
                answer_bytes.extend_from_slice(&(rdata.len() as u16).to_be_bytes());
                answer_bytes.extend_from_slice(&rdata);
                answer_count = answer_count.saturating_add(1);
            }
        }

        let mut response = Vec::with_capacity(12 + packet.len() + answer_bytes.len());
        response.extend_from_slice(&id);
        let recursion_desired = request_flags & 0x0100;
        let truncated_flag = if truncated { 0x0200 } else { 0 };
        let flags = 0x8000u16 | 0x0400 | 0x0080 | recursion_desired | truncated_flag | rcode;
        response.extend_from_slice(&flags.to_be_bytes());
        response.extend_from_slice(&1u16.to_be_bytes()); // QDCOUNT
        response.extend_from_slice(&answer_count.to_be_bytes());
        response.extend_from_slice(&0u16.to_be_bytes()); // NSCOUNT
        response.extend_from_slice(&0u16.to_be_bytes()); // ARCOUNT
        // Echo the exact original question bytes, including 0x20 case randomization.
        response.extend_from_slice(original_question);
        let _ = (query_type, query_class);
        response.extend_from_slice(&answer_bytes);
        Some(response)
    }

    #[derive(Clone)]
    struct HickoryWireResolver {
        server: SocketAddr,
    }

    impl HickoryWireResolver {
        fn new(server: SocketAddr) -> Self {
            Self { server }
        }

        fn build(&self) -> io::Result<hickory_resolver::TokioResolver> {
            use hickory_resolver::config::{
                ConnectionConfig, LookupIpStrategy, NameServerConfig, ResolveHosts, ResolverConfig,
                ResolverOpts,
            };
            use hickory_resolver::net::runtime::TokioRuntimeProvider;

            let mut udp_connection = ConnectionConfig::udp();
            udp_connection.port = self.server.port();
            let mut tcp_connection = ConnectionConfig::tcp();
            tcp_connection.port = self.server.port();
            let name_server =
                NameServerConfig::new(self.server.ip(), true, vec![udp_connection, tcp_connection]);
            let config = ResolverConfig::from_name_servers(vec![name_server]);
            let mut options = ResolverOpts::default();
            options.use_hosts_file = ResolveHosts::Never;
            options.ip_strategy = LookupIpStrategy::Ipv4AndIpv6;
            options.timeout = Duration::from_secs(1);
            options.attempts = 1;
            options.num_concurrent_reqs = 1;
            options.try_tcp_on_error = true;
            hickory_resolver::TokioResolver::builder_with_config(
                config,
                TokioRuntimeProvider::default(),
            )
            .with_options(options)
            .build()
            .map_err(|error| io::Error::other(format!("test resolver build: {error}")))
        }
    }

    impl EgressResolver for HickoryWireResolver {
        fn resolve<'a>(
            &'a self,
            host: &'a str,
            port: u16,
            logical_request_id: &'a str,
            redirect_hop: u8,
            permit: OwnedSemaphorePermit,
        ) -> Pin<Box<dyn Future<Output = io::Result<EgressResolutionSnapshot>> + Send + 'a>> {
            Box::pin(async move {
                // A fresh resolver per authorization prevents Hickory's test cache
                // from hiding a second wire lookup in the rebinding regression.
                // The production resolver remains the system ToSocketAddrs path.
                let _permit = permit;
                let resolver = self.build()?;
                let fqdn = format!("{}.", host.trim_end_matches('.'));
                let lookup = resolver
                    .lookup_ip(fqdn)
                    .await
                    .map_err(|error| io::Error::other(format!("loopback DNS fixture: {error}")))?;
                let addresses = lookup
                    .iter()
                    .map(|address| SocketAddr::new(address, port))
                    .collect();
                Ok(EgressResolutionSnapshot::from_system_resolver(
                    host,
                    port,
                    logical_request_id,
                    redirect_hop,
                    addresses,
                ))
            })
        }
    }

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

    #[tokio::test]
    async fn hermetic_dns_wire_fixture_exercises_dual_stack_cname_and_mixed_answer_policy() {
        use std::net::{Ipv4Addr, Ipv6Addr};
        let dns = WireDnsFixture::start().await;
        dns.set_answer(
            "alias.test",
            WireDnsAnswer::Records(vec![WireDnsRecord::Cname("middle.test.".to_owned(), 30)]),
        );
        dns.set_answer(
            "middle.test",
            WireDnsAnswer::Records(vec![WireDnsRecord::Cname("target.test.".to_owned(), 20)]),
        );
        dns.set_answer(
            "target.test",
            WireDnsAnswer::Records(vec![
                WireDnsRecord::A(Ipv4Addr::new(93, 184, 216, 34), 15),
                WireDnsRecord::Aaaa("2606:4700:4700::1111".parse::<Ipv6Addr>().unwrap(), 15),
            ]),
        );
        dns.set_answer(
            "mixed.test",
            WireDnsAnswer::Records(vec![
                WireDnsRecord::A(Ipv4Addr::new(93, 184, 216, 34), 30),
                WireDnsRecord::Aaaa("fd00::1".parse::<Ipv6Addr>().unwrap(), 30),
            ]),
        );
        dns.set_answer("missing.test", WireDnsAnswer::NxDomain);
        dns.set_answer("servfail.test", WireDnsAnswer::ServFail);

        let resolver = HickoryWireResolver::new(dns.address);
        let client = PublicEgressClient::with_resolver(EgressLimits::default(), resolver);

        let capability = client
            .authorize(
                Method::GET,
                "https://alias.test/document",
                HeaderMap::new(),
                Vec::new(),
            )
            .await
            .expect("real A/AAAA and chained CNAME wire responses should authorize");
        assert_eq!(
            capability.addresses,
            vec![
                "93.184.216.34:443".parse().unwrap(),
                "[2606:4700:4700::1111]:443".parse().unwrap(),
            ]
        );

        let queries = dns.observed_queries();
        for name in ["alias.test.", "middle.test.", "target.test."] {
            assert!(
                queries.contains(&(name.to_owned(), 1)),
                "fixture did not receive an A query for {name}: {queries:?}"
            );
            assert!(
                queries.contains(&(name.to_owned(), 28)),
                "fixture did not receive an AAAA query for {name}: {queries:?}"
            );
        }

        let mixed = client
            .authorize(
                Method::GET,
                "https://mixed.test/document",
                HeaderMap::new(),
                Vec::new(),
            )
            .await;
        assert!(
            matches!(mixed, Err(EgressError::AddressNotPermitted(_))),
            "one prohibited AAAA answer must reject the complete mixed DNS set: {mixed:?}"
        );
        let mixed_queries = dns.observed_queries();
        assert!(mixed_queries.contains(&("mixed.test.".to_owned(), 1)));
        assert!(mixed_queries.contains(&("mixed.test.".to_owned(), 28)));

        let missing = client
            .authorize(
                Method::GET,
                "https://missing.test/document",
                HeaderMap::new(),
                Vec::new(),
            )
            .await;
        assert!(
            matches!(missing, Err(EgressError::DnsResolution { .. })),
            "NXDOMAIN must fail closed instead of producing an empty or partial grant"
        );

        let servfail = client
            .authorize(
                Method::GET,
                "https://servfail.test/document",
                HeaderMap::new(),
                Vec::new(),
            )
            .await;
        assert!(
            matches!(servfail, Err(EgressError::DnsResolution { .. })),
            "SERVFAIL must fail closed"
        );
    }

    #[tokio::test]
    async fn hermetic_dns_lookup_valid_until_tracks_wire_ttl() {
        let dns = WireDnsFixture::start().await;
        dns.set_answer(
            "ttl.test",
            WireDnsAnswer::Records(vec![
                WireDnsRecord::A("93.184.216.34".parse().unwrap(), 5),
                WireDnsRecord::Aaaa("2606:4700:4700::1111".parse().unwrap(), 5),
            ]),
        );

        let resolver = HickoryWireResolver::new(dns.address).build().unwrap();
        let started = std::time::Instant::now();
        let lookup = resolver.lookup_ip("ttl.test.").await.unwrap();
        let valid_until = lookup.valid_until();
        let remaining = valid_until.saturating_duration_since(started);

        assert!(
            remaining > Duration::ZERO,
            "five-second A/AAAA TTL should not be immediately expired"
        );
        assert!(
            remaining <= Duration::from_secs(5),
            "lookup validity must not exceed the DNS record TTL: {remaining:?}"
        );
        assert_eq!(
            lookup.iter().count(),
            2,
            "the dual-stack TTL fixture should return both A and AAAA"
        );
    }

    #[tokio::test]
    async fn hermetic_dns_truncation_retries_over_tcp_and_preserves_address_policy() {
        let dns = WireDnsFixture::start().await;
        dns.set_answer(
            "tcp-fallback.test",
            WireDnsAnswer::TruncatedThenTcp(vec![WireDnsRecord::A(
                "93.184.216.34".parse().unwrap(),
                30,
            )]),
        );

        let resolver = HickoryWireResolver::new(dns.address);
        let client = PublicEgressClient::with_resolver(EgressLimits::default(), resolver);
        let capability = client
            .authorize(
                Method::GET,
                "https://tcp-fallback.test/document",
                HeaderMap::new(),
                Vec::new(),
            )
            .await
            .expect("Hickory must retry a truncated UDP reply over TCP");

        assert_eq!(
            capability.addresses,
            vec!["93.184.216.34:443".parse().unwrap()]
        );
        for query_type in [1, 28] {
            assert_eq!(
                dns.query_count("tcp-fallback.test", query_type),
                1,
                "the resolver should first query UDP for DNS type {query_type}"
            );
            assert_eq!(
                dns.tcp_query_count("tcp-fallback.test", query_type),
                1,
                "a truncated UDP response must retry DNS type {query_type} over TCP"
            );
        }
    }

    #[tokio::test]
    async fn wire_dns_rebinding_after_authorization_cannot_change_the_connected_peer() {
        use std::net::{IpAddr, Ipv6Addr, SocketAddr};
        use tokio::io::AsyncWriteExt;

        let authorized_listener = tokio::net::TcpListener::bind("127.0.0.1:0")
            .await
            .expect("bind authorized IPv4 loopback");
        let authorized_address = authorized_listener.local_addr().unwrap();
        let rebound_address =
            SocketAddr::new(IpAddr::V6(Ipv6Addr::LOCALHOST), authorized_address.port());
        let attacker_listener = tokio::net::TcpListener::bind(rebound_address)
            .await
            .expect("bind IPv6 rebinding target on same port");
        let authorized_server = tokio::spawn(async move {
            let (mut stream, peer) = authorized_listener.accept().await.unwrap();
            let request = read_test_request(&mut stream).await;
            stream
                .write_all(b"HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\nok")
                .await
                .unwrap();
            (peer, request)
        });

        let dns = WireDnsFixture::start().await;
        dns.set_answer(
            "rebind.test",
            WireDnsAnswer::Rebinding {
                first: vec![WireDnsRecord::A(std::net::Ipv4Addr::LOCALHOST, 0)],
                // The first lookup of AAAA returns NODATA; a second resolution
                // changes the host to an IPv6 listener on the same port.
                subsequent: vec![WireDnsRecord::Aaaa(Ipv6Addr::LOCALHOST, 0)],
            },
        );
        let resolver = HickoryWireResolver::new(dns.address);
        let client = PublicEgressClient::with_test_resolver(EgressLimits::default(), resolver);
        let url = format!("http://rebind.test:{}/probe", authorized_address.port());

        let capability = client
            .authorize(Method::GET, &url, HeaderMap::new(), Vec::new())
            .await
            .unwrap();
        assert_eq!(capability.addresses, vec![authorized_address]);
        assert_eq!(dns.query_count("rebind.test", 1), 1);
        assert_eq!(dns.query_count("rebind.test", 28), 1);

        let response = client.execute(capability).await.unwrap();
        assert_eq!(response.body.as_slice(), b"ok");
        assert_eq!(response.receipt.remote_peer, authorized_address);
        assert_eq!(
            response.receipt.authorized_addresses,
            vec![authorized_address]
        );

        let (observed_peer, request) = authorized_server.await.unwrap();
        assert_eq!(observed_peer, authorized_address);
        let request = String::from_utf8_lossy(&request);
        assert!(request.contains(&format!("Host: rebind.test:{}", authorized_address.port())));

        assert_eq!(
            dns.query_count("rebind.test", 1),
            1,
            "execution must not re-resolve A after capability authorization"
        );
        assert_eq!(
            dns.query_count("rebind.test", 28),
            1,
            "execution must not re-resolve AAAA after capability authorization"
        );
        assert!(
            tokio::time::timeout(Duration::from_millis(100), attacker_listener.accept())
                .await
                .is_err(),
            "the attacker at the changed DNS address must receive no connection"
        );
    }

    #[tokio::test]
    async fn hermetic_dns_wire_fixture_drives_pinned_http_socket_and_peer_receipt() {
        use tokio::io::AsyncWriteExt;

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let peer_address = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let (mut stream, peer) = listener.accept().await.unwrap();
            let request = read_test_request(&mut stream).await;
            stream
                .write_all(b"HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\nok")
                .await
                .unwrap();
            (peer, request)
        });

        let dns = WireDnsFixture::start().await;
        dns.set_answer(
            "pinned.test",
            WireDnsAnswer::Records(vec![WireDnsRecord::A(std::net::Ipv4Addr::LOCALHOST, 30)]),
        );
        let resolver = HickoryWireResolver::new(dns.address);
        // The loopback/non-default-port exception is available only in cfg(test).
        // DNS itself is still resolved over real UDP and pinned through Reqwest.
        let client = PublicEgressClient::with_test_resolver(EgressLimits::default(), resolver);
        let url = format!("http://pinned.test:{}/probe", peer_address.port());
        let capability = client
            .authorize(Method::GET, &url, HeaderMap::new(), Vec::new())
            .await
            .unwrap();
        assert_eq!(capability.addresses, vec![peer_address]);

        let response = client.execute(capability).await.unwrap();
        assert_eq!(response.body.as_slice(), b"ok");
        assert_eq!(response.receipt.authorized_addresses, vec![peer_address]);
        assert_eq!(response.receipt.remote_peer, peer_address);

        let (observed_peer, request) = server.await.unwrap();
        assert_eq!(observed_peer, peer_address);
        let request = String::from_utf8_lossy(&request);
        assert!(request.contains(&format!("Host: pinned.test:{}", peer_address.port())));
        let queries = dns.observed_queries();
        assert!(queries.contains(&("pinned.test.".to_owned(), 1)));
        assert!(queries.contains(&("pinned.test.".to_owned(), 28)));
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
            logical_request_id: uuid::Uuid::new_v4().simple().to_string(),
            redirect_hop: 0,
            host: url.host_str().unwrap().to_owned(),
            url,
            authority: EgressAuthority::PublicWeb,
            method: Method::GET,
            headers: HeaderMap::new(),
            body: Vec::new(),
            addresses: vec![address],
            is_domain: true,
            resolution_lease_digest: None,
            resolver_snapshot_digest: None,
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
    fn canonical_address_set_encoding_matches_cross_language_golden_vector() {
        let ipv4: SocketAddr = "93.184.216.34:443".parse().unwrap();
        let ipv6: SocketAddr = "[2606:4700:4700::1111]:443".parse().unwrap();
        assert_eq!(
            canonical_address_set_encoding(&[ipv6, ipv4]),
            vec![
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, // count = 2
                0x04, 0x5d, 0xb8, 0xd8, 0x22, 0x01, 0xbb, // IPv4 + 443
                0x06, 0x26, 0x06, 0x47, 0x00, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x11, 0x11, 0x01, 0xbb, 0x00, 0x00, 0x00, 0x00, // IPv6 + 443 + scope 0
            ]
        );
    }

    #[test]
    fn canonical_address_set_digest_is_order_independent_deduplicated_and_port_bound() {
        let first: SocketAddr = "93.184.216.34:443".parse().unwrap();
        let second: SocketAddr = "[2606:4700:4700::1111]:443".parse().unwrap();
        let duplicate_and_reordered = [second, first, first];
        let canonical = [first, second];

        assert_eq!(
            canonical_address_set_digest(&duplicate_and_reordered),
            canonical_address_set_digest(&canonical)
        );

        let different_port: SocketAddr = "93.184.216.34:8443".parse().unwrap();
        assert_ne!(
            canonical_address_set_digest(&[first]),
            canonical_address_set_digest(&[different_port])
        );
        assert_ne!(
            canonical_address_set_digest(&[first]),
            canonical_address_set_digest(&["[::ffff:93.184.216.34]:443".parse().unwrap()])
        );

        let scoped_one = SocketAddr::V6(std::net::SocketAddrV6::new(
            std::net::Ipv6Addr::LOCALHOST,
            443,
            0,
            1,
        ));
        let same_authority_different_flowinfo = SocketAddr::V6(std::net::SocketAddrV6::new(
            std::net::Ipv6Addr::LOCALHOST,
            443,
            7,
            1,
        ));
        let scoped_two = SocketAddr::V6(std::net::SocketAddrV6::new(
            std::net::Ipv6Addr::LOCALHOST,
            443,
            0,
            2,
        ));
        assert_eq!(
            canonical_address_set_digest(&[scoped_one]),
            canonical_address_set_digest(&[same_authority_different_flowinfo])
        );
        assert_ne!(
            canonical_address_set_digest(&[scoped_one]),
            canonical_address_set_digest(&[scoped_two])
        );
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
        let logical_request_id = capability.logical_request_id.clone();
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
        assert_eq!(
            response.receipt.authorized_address_set_digest,
            canonical_address_set_digest(&[address])
        );
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
            logical_request_id: &'a str,
            redirect_hop: u8,
            _permit: OwnedSemaphorePermit,
        ) -> Pin<Box<dyn Future<Output = io::Result<EgressResolutionSnapshot>> + Send + 'a>> {
            let host = host.to_owned();
            let logical_request_id = logical_request_id.to_owned();
            let calls = Arc::clone(&self.calls);
            Box::pin(async move {
                calls.lock().expect("resolver call lock").push((host.clone(), port));
                Ok(EgressResolutionSnapshot::from_system_resolver(
                    host,
                    port,
                    logical_request_id,
                    redirect_hop,
                    vec![SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), port)],
                ))
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
                .write_all(b"HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\nok")
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
        assert_eq!(response.receipt.logical_request_id.len(), 32);
        assert_eq!(response.receipt.redirect_hop, 1);

        let hop = &response.receipt.redirect_hops[0];
        assert_eq!(hop.logical_request_id, response.receipt.logical_request_id);
        assert_eq!(hop.redirect_hop, 0);
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
        assert_eq!(hop.authorized_addresses, vec![first_address]);
        assert_eq!(
            hop.authorized_address_set_digest,
            canonical_address_set_digest(&[first_address])
        );
        assert_eq!(hop.target_addresses, vec![second_address]);
        assert_eq!(
            hop.target_address_set_digest,
            canonical_address_set_digest(&[second_address])
        );
        assert_eq!(
            response.receipt.authorized_address_set_digest,
            canonical_address_set_digest(&[second_address])
        );
        assert!(hop.tls_identity.is_none(), "plain HTTP redirect has no TLS identity");
        assert!(!hop.http_version.is_empty());
        assert!(response.receipt.tls_identity.is_none());

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
        assert!(!first_request.contains(&response.receipt.logical_request_id));
        assert!(!second_request.contains(&response.receipt.logical_request_id));
    }

    #[tokio::test]
    async fn public_redirect_to_metadata_host_is_rejected_before_second_dns_or_socket() {
        use tokio::io::AsyncWriteExt;

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            let request = read_test_request(&mut stream).await;
            stream
                .write_all(
                    b"HTTP/1.1 302 Found\r\nLocation: http://metadata.google.internal/latest/meta-data\r\nContent-Length: 0\r\nConnection: close\r\n\r\n",
                )
                .await
                .unwrap();
            request
        });

        let resolver = LoopbackResolver::default();
        let client =
            PublicEgressClient::with_test_resolver(EgressLimits::default(), resolver.clone());
        let url = format!("http://prism-metadata-redirect.test:{}/start", address.port());
        let capability = client
            .authorize(Method::GET, &url, HeaderMap::new(), Vec::new())
            .await
            .unwrap();

        let result = client.execute(capability).await;
        assert!(matches!(result, Err(EgressError::BlockedHost)));

        let resolver_calls = resolver.calls.lock().expect("resolver call lock").clone();
        assert_eq!(
            resolver_calls,
            vec![("prism-metadata-redirect.test".to_owned(), address.port())],
            "metadata redirect must be rejected before a second resolver call"
        );

        let first_request = String::from_utf8_lossy(&server.await.unwrap());
        assert!(first_request.contains(&format!(
            "Host: prism-metadata-redirect.test:{}",
            address.port()
        )));
        assert!(!first_request.contains(&logical_request_id));
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
        assert_eq!(
            response.receipt.authorized_address_set_digest,
            canonical_address_set_digest(&[address])
        );
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
        let client = PublicEgressClient::with_test_resolver(EgressLimits::default(), test_resolver)
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
        let tls_receipt = response
            .receipt
            .tls_identity
            .as_ref()
            .expect("HTTPS response records verified TLS identity");
        assert_eq!(tls_receipt.reference_identity, "prism-tls.test");
        assert_eq!(tls_receipt.sni_hostname.as_deref(), Some("prism-tls.test"));
        assert!(tls_receipt.certificate_identity_verified);
        assert!(!response.receipt.http_version.is_empty());

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

    async fn hyper_client_accepts_response_header_block(max_header_size: Option<usize>) -> bool {
        use hyper::client::conn::http1::Builder;
        use hyper_util::rt::TokioIo;
        use tokio::io::AsyncWriteExt;

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let response = {
            let mut bytes = b"HTTP/1.1 200 OK\r\nX-Prism-Large: ".to_vec();
            bytes.extend(std::iter::repeat_n(b'v', 4096));
            bytes.extend_from_slice(b"\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
            bytes
        };

        let server = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            read_test_request(&mut stream).await;
            stream.write_all(&response).await.unwrap();
        });

        let stream = tokio::net::TcpStream::connect(address).await.unwrap();
        let mut builder = Builder::new();
        // Keep the field-count setting identical so the control isolates
        // the aggregate response-header-byte limit.
        builder.max_headers(MAX_RESPONSE_HEADER_COUNT);
        if let Some(limit) = max_header_size {
            builder.max_header_size(limit);
        }

        let (mut sender, connection) = builder
            .handshake(TokioIo::new(stream))
            .await
            .expect("Hyper HTTP/1 client handshake");
        let driver = tokio::spawn(async move {
            let _ = connection.await;
        });
        let request = hyper::Request::builder()
            .method("GET")
            .uri("http://prism-hyper-header-limit.test/probe")
            .header(hyper::header::HOST, "prism-hyper-header-limit.test")
            .body(http_body_util::Empty::<Bytes>::new())
            .expect("valid Hyper test request");
        let accepted = tokio::time::timeout(Duration::from_secs(2), sender.send_request(request))
            .await
            .is_ok_and(|result| result.is_ok());

        drop(sender);
        let _ = driver.await;
        server.await.unwrap();
        accepted
    }

    #[tokio::test]
    async fn hyper_preparse_header_byte_limit_rejects_large_single_header_with_negative_control() {
        // Negative control: the same >4 KiB response header is accepted when
        // no aggregate header-byte cap is configured.
        assert!(
            hyper_client_accepts_response_header_block(None).await,
            "the negative control should accept the oversized response header"
        );

        // Hyper documents this as a parser-level limit that includes the
        // HTTP status line. The over-limit response must fail before a
        // Response<Incoming> reaches application code.
        assert!(
            !hyper_client_accepts_response_header_block(Some(1024)).await,
            "Hyper must reject a response header block above 1 KiB"
        );
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
            host: &'a str,
            port: u16,
            logical_request_id: &'a str,
            redirect_hop: u8,
            _permit: OwnedSemaphorePermit,
        ) -> Pin<Box<dyn Future<Output = io::Result<EgressResolutionSnapshot>> + Send + 'a>> {
            let call = self.calls.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            let answers = if call == 0 {
                self.first_answers.clone()
            } else {
                self.subsequent_answers.clone()
            };
            let hostname = host.to_owned();
            let logical_request_id = logical_request_id.to_owned();
            Box::pin(async move {
                Ok(EgressResolutionSnapshot::from_system_resolver(
                    hostname,
                    port,
                    logical_request_id,
                    redirect_hop,
                    answers,
                ))
            })
        }
    }

    impl EgressResolver for StaticResolver {
        fn resolve<'a>(
            &'a self,
            host: &'a str,
            port: u16,
            logical_request_id: &'a str,
            redirect_hop: u8,
            _permit: OwnedSemaphorePermit,
        ) -> Pin<Box<dyn Future<Output = io::Result<EgressResolutionSnapshot>> + Send + 'a>> {
            self.calls.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            let answers = self.answers.clone();
            let hostname = host.to_owned();
            let logical_request_id = logical_request_id.to_owned();
            Box::pin(async move {
                Ok(EgressResolutionSnapshot::from_system_resolver(
                    hostname,
                    port,
                    logical_request_id,
                    redirect_hop,
                    answers,
                ))
            })
        }
    }

    #[derive(Clone)]
    struct LeaseSnapshotResolver {
        snapshot: EgressResolutionSnapshot,
        requests: Arc<std::sync::Mutex<Vec<(String, u16, String, u8)>>>,
    }

    impl EgressResolver for LeaseSnapshotResolver {
        fn resolve<'a>(
            &'a self,
            host: &'a str,
            port: u16,
            logical_request_id: &'a str,
            redirect_hop: u8,
            permit: OwnedSemaphorePermit,
        ) -> Pin<Box<dyn Future<Output = io::Result<EgressResolutionSnapshot>> + Send + 'a>> {
            self.requests
                .lock()
                .expect("lease snapshot request lock")
                .push((
                    host.to_owned(),
                    port,
                    logical_request_id.to_owned(),
                    redirect_hop,
                ));
            let snapshot = self.snapshot.clone();
            Box::pin(async move {
                let _permit = permit;
                Ok(snapshot)
            })
        }
    }

    fn socket_addresses(values: &[&str]) -> Vec<SocketAddr> {
        values.iter().map(|value| value.parse().unwrap()).collect()
    }

    #[tokio::test]
    async fn nixward_resolution_snapshot_binds_request_and_clamps_capability_expiry() {
        let logical_request_id = "0123456789abcdef0123456789abcdef";
        let addresses = socket_addresses(&[
            "93.184.216.34:443",
            "[2606:4700:4700::1111]:443",
        ]);
        let address_set_digest = canonical_address_set_digest(&addresses);
        let resolution_lease_digest = "11".repeat(32);
        let resolver_snapshot_digest = "22".repeat(32);
        let valid_until = std::time::Instant::now() + Duration::from_secs(4);

        let snapshot = EgressResolutionSnapshot::from_verified_nixward_lease(
            "prism-lease.test",
            443,
            logical_request_id,
            2,
            addresses.clone(),
            valid_until,
            address_set_digest.clone(),
            resolution_lease_digest.clone(),
            resolver_snapshot_digest.clone(),
        )
        .unwrap();
        let requests = Arc::new(std::sync::Mutex::new(Vec::new()));
        let resolver = LeaseSnapshotResolver {
            snapshot,
            requests: Arc::clone(&requests),
        };
        let client = PublicEgressClient::with_resolver(EgressLimits::default(), resolver);

        let capability = client
            .authorize_with_authority_and_correlation(
                EgressAuthority::PublicWeb,
                Method::GET,
                "https://prism-lease.test/",
                HeaderMap::new(),
                Vec::new(),
                logical_request_id,
                2,
            )
            .await
            .unwrap();

        assert_eq!(capability.logical_request_id, logical_request_id);
        assert_eq!(capability.redirect_hop, 2);
        assert_eq!(capability.addresses, addresses);
        assert_eq!(
            capability.resolution_lease_digest.as_deref(),
            Some(resolution_lease_digest.as_str())
        );
        assert_eq!(
            capability.resolver_snapshot_digest.as_deref(),
            Some(resolver_snapshot_digest.as_str())
        );
        assert!(
            capability.expires_at <= valid_until,
            "capability lifetime must never outlive the resolution lease"
        );
        assert_eq!(
            requests.lock().expect("lease snapshot request lock").as_slice(),
            &[(
                "prism-lease.test".to_owned(),
                443,
                logical_request_id.to_owned(),
                2,
            )]
        );
    }

    #[tokio::test]
    async fn nixward_resolution_snapshot_rejects_mismatched_scope_before_capability() {
        let logical_request_id = "0123456789abcdef0123456789abcdef";
        let addresses = socket_addresses(&["93.184.216.34:443"]);
        let address_set_digest = canonical_address_set_digest(&addresses);

        let wrong_digest = EgressResolutionSnapshot::from_verified_nixward_lease(
            "prism-lease.test",
            443,
            logical_request_id,
            0,
            addresses.clone(),
            std::time::Instant::now() + Duration::from_secs(5),
            "ff".repeat(32),
            "11".repeat(32),
            "22".repeat(32),
        );
        assert!(matches!(
            wrong_digest,
            Err(EgressError::ResolutionAddressSetDigestMismatch)
        ));

        let wrong_port = EgressResolutionSnapshot::from_verified_nixward_lease(
            "prism-lease.test",
            8443,
            logical_request_id,
            0,
            addresses.clone(),
            std::time::Instant::now() + Duration::from_secs(5),
            address_set_digest.clone(),
            "11".repeat(32),
            "22".repeat(32),
        );
        assert!(matches!(
            wrong_port,
            Err(EgressError::ResolutionSnapshotMismatch)
        ));

        let expired = EgressResolutionSnapshot::from_verified_nixward_lease(
            "prism-lease.test",
            443,
            logical_request_id,
            0,
            addresses.clone(),
            std::time::Instant::now() - Duration::from_secs(1),
            address_set_digest.clone(),
            "11".repeat(32),
            "22".repeat(32),
        );
        assert!(matches!(
            expired,
            Err(EgressError::ResolutionLeaseExpired)
        ));

        let wrong_request = EgressResolutionSnapshot::from_verified_nixward_lease(
            "other-prism-lease.test",
            443,
            "fedcba9876543210fedcba9876543210",
            0,
            addresses,
            std::time::Instant::now() + Duration::from_secs(5),
            address_set_digest,
            "11".repeat(32),
            "22".repeat(32),
        )
        .unwrap();
        let resolver = LeaseSnapshotResolver {
            snapshot: wrong_request,
            requests: Arc::new(std::sync::Mutex::new(Vec::new())),
        };
        let client = PublicEgressClient::with_resolver(EgressLimits::default(), resolver);
        let result = client
            .authorize_with_authority_and_correlation(
                EgressAuthority::PublicWeb,
                Method::GET,
                "https://prism-lease.test/",
                HeaderMap::new(),
                Vec::new(),
                logical_request_id,
                0,
            )
            .await;
        assert!(matches!(
            result,
            Err(EgressError::ResolutionSnapshotMismatch)
        ));
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

    #[tokio::test]
    async fn caller_correlation_id_is_preserved_and_hop_ceiling_fails_before_dns() {
        let calls = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let resolver = StaticResolver {
            answers: socket_addresses(&["93.184.216.34:443"]),
            calls: Arc::clone(&calls),
        };
        let client = PublicEgressClient::with_resolver(EgressLimits::default(), resolver);
        let logical_request_id = "0123456789abcdef0123456789abcdef";

        let capability = client
            .authorize_with_authority_and_correlation(
                EgressAuthority::PublicWeb,
                Method::GET,
                "https://correlation-prism.test/",
                HeaderMap::new(),
                Vec::new(),
                logical_request_id,
                2,
            )
            .await
            .unwrap();
        assert_eq!(capability.logical_request_id, logical_request_id);
        assert_eq!(capability.redirect_hop, 2);
        assert_eq!(calls.load(std::sync::atomic::Ordering::SeqCst), 1);

        let rejected = client
            .authorize_with_authority_and_correlation(
                EgressAuthority::PublicWeb,
                Method::GET,
                "https://correlation-prism.test/",
                HeaderMap::new(),
                Vec::new(),
                logical_request_id,
                (MAX_PUBLIC_WEB_REDIRECT_HOPS + 1) as u8,
            )
            .await;
        assert!(matches!(
            rejected,
            Err(EgressError::LogicalRequestHopOutOfRange)
        ));
        assert_eq!(
            calls.load(std::sync::atomic::Ordering::SeqCst),
            1,
            "invalid hop metadata must be rejected before DNS"
        );
    }

    #[test]
    fn logical_request_id_requires_exact_lowercase_hex_and_valid_hop() {
        assert!(validate_logical_request_id("0123456789abcdef0123456789abcdef").is_ok());
        assert!(matches!(
            validate_logical_request_id("0123456789ABCDEF0123456789ABCDEF"),
            Err(EgressError::InvalidLogicalRequestId)
        ));
        assert!(matches!(
            validate_logical_request_id("short"),
            Err(EgressError::InvalidLogicalRequestId)
        ));
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

    #[test]
    fn peab_manifest_contract_is_enforced_by_the_rust_test_suite() {
        const MANIFEST: &str = include_str!("../../docs/peab-scenarios-v0.1.json");
        let manifest: serde_json::Value =
            serde_json::from_str(MANIFEST).expect("PEAB scenario manifest must be valid JSON");

        assert_eq!(
            manifest["schema"],
            "org.luminousdynamics.peab.scenario-manifest.v1"
        );
        assert_eq!(manifest["version"], "0.1.0");
        assert_eq!(manifest["status"], "specification_only");

        let scenarios = manifest["scenarios"]
            .as_array()
            .expect("PEAB scenarios must be an array");
        assert!(
            scenarios.len() >= 31,
            "PEAB manifest must retain its minimum scenario corpus"
        );

        let mut ids = std::collections::HashSet::new();
        for scenario in scenarios {
            let id = scenario["id"]
                .as_str()
                .filter(|id| !id.trim().is_empty())
                .expect("every PEAB scenario needs a non-empty stable ID");
            assert!(ids.insert(id), "duplicate PEAB scenario ID: {id}");
            assert_eq!(
                scenario["negative_control_required"],
                true,
                "scenario {id} must declare a negative control"
            );

            let coverage = scenario["source_coverage"]
                .as_str()
                .expect("scenario coverage must be a string");
            assert!(
                matches!(coverage, "present" | "partial" | "planned"),
                "scenario {id} has invalid source coverage {coverage:?}"
            );

            match coverage {
                "planned" => assert!(
                    scenario["source_test"].is_null(),
                    "planned scenario {id} cannot claim a source test"
                ),
                "present" => assert!(
                    scenario["source_test"]
                        .as_str()
                        .is_some_and(|name| !name.trim().is_empty()),
                    "source-present scenario {id} must name its source test"
                ),
                "partial" => {
                    assert!(
                        scenario["source_test"].is_null()
                            || scenario["source_test"]
                                .as_str()
                                .is_some_and(|name| !name.trim().is_empty()),
                        "partial scenario {id} must use null or a non-empty source-test list"
                    );
                }
                _ => unreachable!("coverage vocabulary was checked above"),
            }

            let execution_status = scenario["execution_status"]
                .as_str()
                .expect("scenario execution status must be a string");
            if execution_status == "not_run_exact_head" {
                assert!(
                    scenario["execution_evidence"].is_null(),
                    "unrun scenario {id} cannot carry execution evidence"
                );
            } else {
                assert!(
                    matches!(execution_status, "passed" | "failed" | "skipped"),
                    "scenario {id} has unsupported execution status {execution_status:?}"
                );
                let evidence = &scenario["execution_evidence"];
                assert!(evidence.is_object(), "scenario {id} needs an evidence object");
                assert!(
                    evidence["head_sha"].as_str().is_some_and(|value| !value.is_empty()),
                    "scenario {id} evidence must bind an exact head SHA"
                );
                assert!(
                    evidence["run_id"].as_u64().is_some(),
                    "scenario {id} evidence must bind a workflow run ID"
                );
                assert!(
                    evidence["job_id"].as_u64().is_some(),
                    "scenario {id} evidence must bind a workflow job ID"
                );
            }
        }
    }
}
