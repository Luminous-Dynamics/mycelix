//! Pure qualification of resolved public HTTPS routes for Mycelix integrations.
//!
//! This crate performs no DNS or network I/O. It takes a resolution observation,
//! binds it to the exact current HTTP transport/provider policy, rejects
//! special/private/non-public address space, and produces a short-lived route
//! proof. The eventual native HTTPS engine must perform resolution itself (or
//! consume independently verified resolver evidence), connect only to an address
//! in this exact qualified set, and verify WebPKI DNS identity for the policy
//! hostname. A hostname must never be re-resolved after qualification and used
//! without repeating this theorem.

use mycelix_institutional_core::Digest32;
use mycelix_integration_http_transport_policy::QualifiedHttpTransportProviderBinding;
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr};
use thiserror::Error;

pub const PUBLIC_HTTPS_ROUTE_PROFILE: &str =
    "mycelix-integration-http-public-egress-route-v1-blake3-framed";
const DOMAIN_ROUTE: &[u8] = b"mycelix/integration/http-public-egress-route/v1";
const MAX_RESOLVED_ADDRESSES: usize = 16;
const MAX_TEXT_BYTES: usize = 1024;

/// Untrusted-by-itself observation supplied to the pure qualifier. A future
/// native transport should construct this from its own pinned resolver path or
/// independently authenticated resolver evidence.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DnsResolutionObservation {
    pub hostname: String,
    pub addresses: Vec<IpAddr>,
    pub resolver_ref: String,
    pub observed_at_ms: u64,
    pub valid_until_ms: u64,
}

/// Non-deserializable proof that one exact DNS observation is compatible with
/// one exact current public-HTTPS transport policy and contains only admissible
/// public addresses.
#[derive(Clone, Debug)]
pub struct QualifiedPublicHttpsRoute {
    hostname: String,
    addresses: Vec<IpAddr>,
    resolver_ref: String,
    transport_policy_binding_digest: Digest32,
    route_digest: Digest32,
    qualified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedPublicHttpsRoute {
    pub fn hostname(&self) -> &str {
        &self.hostname
    }

    pub fn addresses(&self) -> &[IpAddr] {
        &self.addresses
    }

    pub fn resolver_ref(&self) -> &str {
        &self.resolver_ref
    }

    pub fn transport_policy_binding_digest(&self) -> Digest32 {
        self.transport_policy_binding_digest
    }

    pub fn route_digest(&self) -> Digest32 {
        self.route_digest
    }

    pub fn qualified_at_ms(&self) -> u64 {
        self.qualified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub fn is_live_at(&self, now_ms: u64) -> bool {
        now_ms >= self.qualified_at_ms && now_ms < self.valid_until_ms
    }

    pub const fn connect_only_to_qualified_addresses_here(&self) -> bool {
        true
    }

    pub const fn webpki_dns_name_verification_required_here(&self) -> bool {
        true
    }

    pub const fn sni_equals_policy_hostname_here(&self) -> bool {
        true
    }

    pub const fn redirects_allowed_here(&self) -> bool {
        false
    }

    pub const fn environment_proxy_allowed_here(&self) -> bool {
        false
    }

    pub const fn custom_trust_roots_allowed_here(&self) -> bool {
        false
    }

    pub const fn resolution_evidence_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn network_io_performed_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub fn qualify_public_https_route(
    transport: &QualifiedHttpTransportProviderBinding,
    observation: DnsResolutionObservation,
    now_ms: u64,
) -> Result<QualifiedPublicHttpsRoute, HttpEgressRouteError> {
    if now_ms == 0 || transport.valid_until_ms() <= now_ms {
        return Err(HttpEgressRouteError::TransportPolicyNotLive);
    }
    let policy = transport.current_policy().policy();
    if observation.hostname != policy.endpoint_host {
        return Err(HttpEgressRouteError::HostnameMismatch);
    }
    validate_text(&observation.resolver_ref)?;
    if observation.observed_at_ms == 0
        || observation.observed_at_ms > now_ms
        || observation.valid_until_ms <= now_ms
        || observation.valid_until_ms <= observation.observed_at_ms
    {
        return Err(HttpEgressRouteError::InvalidResolutionWindow);
    }
    if observation.addresses.is_empty()
        || observation.addresses.len() > MAX_RESOLVED_ADDRESSES
    {
        return Err(HttpEgressRouteError::InvalidAddressCount);
    }
    if !strictly_sorted_unique(&observation.addresses) {
        return Err(HttpEgressRouteError::NonCanonicalAddressSet);
    }
    if observation.addresses.iter().any(|address| !is_public_unicast(*address)) {
        return Err(HttpEgressRouteError::NonPublicAddress);
    }

    let valid_until_ms = observation.valid_until_ms.min(transport.valid_until_ms());
    if valid_until_ms <= now_ms {
        return Err(HttpEgressRouteError::NoUsableRouteWindow);
    }
    let route_digest = route_digest(
        transport.binding_digest(),
        &observation.hostname,
        policy.endpoint_port,
        &observation.addresses,
        &observation.resolver_ref,
        observation.observed_at_ms,
        valid_until_ms,
    );

    Ok(QualifiedPublicHttpsRoute {
        hostname: observation.hostname,
        addresses: observation.addresses,
        resolver_ref: observation.resolver_ref,
        transport_policy_binding_digest: transport.binding_digest(),
        route_digest,
        qualified_at_ms: now_ms,
        valid_until_ms,
    })
}

fn strictly_sorted_unique(addresses: &[IpAddr]) -> bool {
    addresses.windows(2).all(|pair| pair[0] < pair[1])
}

fn is_public_unicast(address: IpAddr) -> bool {
    match address {
        IpAddr::V4(address) => ipv4_is_public_unicast(address),
        IpAddr::V6(address) => ipv6_is_public_unicast(address),
    }
}

fn ipv4_is_public_unicast(address: Ipv4Addr) -> bool {
    let [a, b, c, _d] = address.octets();
    if address.is_unspecified()
        || address.is_loopback()
        || address.is_private()
        || address.is_link_local()
        || address.is_multicast()
        || address.is_broadcast()
    {
        return false;
    }

    // Conservative special-use deny set. Public-provider v0.1 does not permit
    // protocol-assignment, shared, benchmark, documentation, transition, or
    // future/reserved space even when the host OS might route it.
    if a == 0
        || (a == 100 && (64..=127).contains(&b)) // RFC 6598 shared/CGNAT
        || (a == 192 && b == 0 && c == 0) // IETF protocol assignments
        || (a == 192 && b == 0 && c == 2) // TEST-NET-1
        || (a == 192 && b == 88 && c == 99) // deprecated 6to4 relay anycast
        || (a == 198 && (b == 18 || b == 19)) // benchmarking
        || (a == 198 && b == 51 && c == 100) // TEST-NET-2
        || (a == 203 && b == 0 && c == 113) // TEST-NET-3
        || a >= 240 // reserved/future + limited broadcast
    {
        return false;
    }
    true
}

fn ipv6_is_public_unicast(address: Ipv6Addr) -> bool {
    let s = address.segments();
    // Require global-unicast 2000::/3 as the outer envelope.
    if s[0] & 0xe000 != 0x2000 {
        return false;
    }

    // Conservative deny set inside 2000::/3.
    if (s[0] == 0x2001 && s[1] == 0x0000) // Teredo 2001:0000::/32
        || (s[0] == 0x2001 && s[1] == 0x0002 && s[2] == 0) // benchmarking 2001:2::/48
        || (s[0] == 0x2001 && (s[1] & 0xfff0) == 0x0020) // ORCHIDv2 2001:20::/28
        || (s[0] == 0x2001 && s[1] == 0x0db8) // documentation
        || s[0] == 0x2002 // deprecated 6to4
        || (s[0] == 0x3fff && (s[1] & 0xf000) == 0) // documentation 3fff::/20
    {
        return false;
    }
    true
}

fn route_digest(
    transport_binding: Digest32,
    hostname: &str,
    port: u16,
    addresses: &[IpAddr],
    resolver_ref: &str,
    observed_at_ms: u64,
    valid_until_ms: u64,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_ROUTE);
    frame(&mut h, PUBLIC_HTTPS_ROUTE_PROFILE.as_bytes());
    frame(&mut h, &transport_binding.0);
    frame(&mut h, hostname.as_bytes());
    frame(&mut h, &port.to_be_bytes());
    for address in addresses {
        match address {
            IpAddr::V4(address) => {
                frame(&mut h, &[4]);
                frame(&mut h, &address.octets());
            }
            IpAddr::V6(address) => {
                frame(&mut h, &[6]);
                frame(&mut h, &address.octets());
            }
        }
    }
    frame(&mut h, resolver_ref.as_bytes());
    frame(&mut h, &observed_at_ms.to_le_bytes());
    frame(&mut h, &valid_until_ms.to_le_bytes());
    Digest32(*h.finalize().as_bytes())
}

fn validate_text(value: &str) -> Result<(), HttpEgressRouteError> {
    if value.trim().is_empty()
        || value.len() > MAX_TEXT_BYTES
        || value.chars().any(char::is_control)
    {
        Err(HttpEgressRouteError::InvalidResolverRef)
    } else {
        Ok(())
    }
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum HttpEgressRouteError {
    #[error("current HTTP transport policy is not live")]
    TransportPolicyNotLive,
    #[error("DNS observation hostname differs from transport policy hostname")]
    HostnameMismatch,
    #[error("resolver evidence reference is invalid")]
    InvalidResolverRef,
    #[error("DNS resolution evidence window is invalid")]
    InvalidResolutionWindow,
    #[error("DNS resolution address count is invalid")]
    InvalidAddressCount,
    #[error("DNS address set must be strictly sorted and duplicate-free")]
    NonCanonicalAddressSet,
    #[error("DNS resolution contains a non-public/special-use address")]
    NonPublicAddress,
    #[error("no usable route window remains")]
    NoUsableRouteWindow,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn public_ipv4_examples_are_admitted() {
        assert!(ipv4_is_public_unicast(Ipv4Addr::new(1, 1, 1, 1)));
        assert!(ipv4_is_public_unicast(Ipv4Addr::new(8, 8, 8, 8)));
    }

    #[test]
    fn sensitive_ipv4_ranges_fail_closed() {
        for address in [
            Ipv4Addr::new(0, 1, 2, 3),
            Ipv4Addr::new(10, 0, 0, 1),
            Ipv4Addr::new(100, 64, 0, 1),
            Ipv4Addr::new(127, 0, 0, 1),
            Ipv4Addr::new(169, 254, 1, 1),
            Ipv4Addr::new(172, 16, 0, 1),
            Ipv4Addr::new(192, 0, 0, 1),
            Ipv4Addr::new(192, 0, 2, 1),
            Ipv4Addr::new(192, 168, 0, 1),
            Ipv4Addr::new(198, 18, 0, 1),
            Ipv4Addr::new(198, 51, 100, 1),
            Ipv4Addr::new(203, 0, 113, 1),
            Ipv4Addr::new(224, 0, 0, 1),
            Ipv4Addr::new(240, 0, 0, 1),
        ] {
            assert!(!ipv4_is_public_unicast(address), "{address} should fail");
        }
    }

    #[test]
    fn public_ipv6_and_special_ranges_are_distinguished() {
        assert!(ipv6_is_public_unicast("2606:4700:4700::1111".parse().unwrap()));
        for address in [
            "::1",
            "fe80::1",
            "fc00::1",
            "2001:db8::1",
            "2001:0000::1",
            "2001:2::1",
            "2001:20::1",
            "2002::1",
            "3fff::1",
            "ff02::1",
        ] {
            let address: Ipv6Addr = address.parse().unwrap();
            assert!(!ipv6_is_public_unicast(address), "{address} should fail");
        }
    }

    #[test]
    fn address_set_must_be_sorted_and_unique() {
        let a: IpAddr = "1.1.1.1".parse().unwrap();
        let b: IpAddr = "8.8.8.8".parse().unwrap();
        assert!(strictly_sorted_unique(&[a, b]));
        assert!(!strictly_sorted_unique(&[b, a]));
        assert!(!strictly_sorted_unique(&[a, a]));
    }
}
