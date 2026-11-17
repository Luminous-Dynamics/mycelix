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
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr};
use std::time::Duration;
use thiserror::Error;
use url::{Host, Url};

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
    #[error("request body exceeds the configured byte limit")]
    RequestBodyTooLarge,
    #[error("response body exceeds the configured byte limit")]
    ResponseBodyTooLarge,
    #[error("egress capability expired before execution")]
    CapabilityExpired,
    #[error("HTTP transport failed: {0}")]
    Http(#[from] reqwest::Error),
}

/// A one-use, short-lived authorization to fetch one exact URL with one method
/// and one already-validated destination address set.
///
/// The fields are private and the value is not Clone. Execution consumes it,
/// making accidental reuse or widening harder at the API boundary.
pub struct EgressCapability {
    url: Url,
    host: String,
    method: Method,
    addresses: Vec<SocketAddr>,
    is_domain: bool,
    expires_at: std::time::Instant,
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
}

/// Public-web-only HTTP client. It creates a fresh, DNS-pinned reqwest client
/// for each consumed capability to prevent cross-request connection-pool reuse.
#[derive(Clone, Debug, Default)]
pub struct PublicEgressClient {
    limits: EgressLimits,
}

impl PublicEgressClient {
    pub fn new(limits: EgressLimits) -> Self {
        Self { limits }
    }

    pub fn limits(&self) -> &EgressLimits {
        &self.limits
    }

    /// Resolve and authorize an exact request target. The returned capability
    /// expires quickly and can only be executed once.
    pub async fn authorize(
        &self,
        method: Method,
        raw_url: &str,
    ) -> Result<EgressCapability, EgressError> {
        if raw_url.len() > self.limits.max_url_bytes {
            return Err(EgressError::UrlTooLarge);
        }
        if method != Method::GET && method != Method::POST {
            return Err(EgressError::MethodNotPermitted);
        }

        let mut url = Url::parse(raw_url)?;
        validate_url_policy(&url)?;
        // URL fragments are client-side state and must not affect the
        // authorization identity or be sent as part of the HTTP request.
        url.set_fragment(None);

        let host = url.host_str().ok_or(EgressError::MissingHost)?.to_owned();
        let port = url.port_or_known_default().ok_or(EgressError::PortNotPermitted)?;
        let (addresses, is_domain) = resolve_and_authorize(&url, port).await?;

        Ok(EgressCapability {
            url,
            host,
            method,
            addresses,
            is_domain,
            expires_at: std::time::Instant::now() + self.limits.capability_ttl,
        })
    }

    /// Consume one authorization and perform the exact bounded request.
    pub async fn execute(
        &self,
        capability: EgressCapability,
        headers: HeaderMap,
        body: Vec<u8>,
    ) -> Result<EgressResponse, EgressError> {
        if std::time::Instant::now() > capability.expires_at {
            return Err(EgressError::CapabilityExpired);
        }
        if body.len() > self.limits.max_request_body_bytes {
            return Err(EgressError::RequestBodyTooLarge);
        }
        if request_header_bytes(&headers) > self.limits.max_request_header_bytes {
            return Err(EgressError::RequestHeadersTooLarge);
        }

        let origin = format!("{}://{}", capability.url.scheme(), capability.host);
        let authorized_addresses = capability.addresses.clone();
        let method_name = capability.method.as_str().to_owned();
        let request_body_bytes = body.len();
        let started = std::time::Instant::now();

        // A fresh client per request deliberately trades a little setup cost
        // for a simple connection-pool isolation boundary. The original URL
        // hostname remains in the URL, preserving Host, TLS SNI, and
        // certificate verification while reqwest uses only the approved IPs.
        let mut builder = Client::builder()
            .user_agent("Prism/0.4 (capability-bound egress)")
            .redirect(reqwest::redirect::Policy::none())
            .no_proxy()
            .connect_timeout(self.limits.connect_timeout)
            .timeout(self.limits.request_timeout);
        if capability.is_domain {
            builder = builder.resolve_to_addrs(&capability.host, &capability.addresses);
        }
        let client = builder.build()?;

        let response = client
            .request(capability.method, capability.url)
            .headers(headers)
            .body(body)
            .send()
            .await?;

        let status = response.status();
        let response_headers = response.headers().clone();
        let mut response = response;
        let mut collected = Vec::new();

        while let Some(chunk) = response.chunk().await? {
            if collected.len().saturating_add(chunk.len())
                > self.limits.max_response_body_bytes
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
            policy_version: "prism-public-egress-v1",
        };

        Ok(EgressResponse {
            status,
            headers: response_headers,
            body: collected,
            receipt,
        })
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
        let capability = self.authorize(method, raw_url).await?;
        self.execute(capability, headers, body).await
    }
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
    if url.host().is_none() {
        return Err(EgressError::MissingHost);
    }
    Ok(())
}

async fn resolve_and_authorize(
    url: &Url,
    port: u16,
) -> Result<(Vec<SocketAddr>, bool), EgressError> {
    let host = url.host().ok_or(EgressError::MissingHost)?;
    let (mut addresses, is_domain) = match host {
        Host::Ipv4(address) => (vec![SocketAddr::new(IpAddr::V4(address), port)], false),
        Host::Ipv6(address) => (vec![SocketAddr::new(IpAddr::V6(address), port)], false),
        Host::Domain(domain) => {
            let resolved = tokio::net::lookup_host((domain, port))
                .await
                .map_err(|source| EgressError::DnsResolution {
                    host: domain.to_owned(),
                    source,
                })?;
            (resolved.collect::<Vec<_>>(), true)
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
    fn allows_representative_public_unicast_addresses() {
        for address in ["1.1.1.1", "8.8.8.8", "93.184.216.34", "192.31.196.1", "192.52.193.1", "192.175.48.1", "2606:4700:4700::1111", "2001:4860::1"] {
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
    fn rejects_url_credentials() {
        assert!(matches!(
            validate_url_policy(&Url::parse("https://user:secret@example.com").unwrap()),
            Err(EgressError::CredentialsNotPermitted)
        ));
    }

    #[test]
    fn enforces_request_header_budget() {
        let mut headers = HeaderMap::new();
        headers.insert("x-test", "long-value");
        assert!(request_header_bytes(&headers) >= "x-test".len() + "long-value".len());
    }
}
