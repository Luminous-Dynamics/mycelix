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
//!
//! Public-SaaS v2 is deliberately stricter than "globally reachable": every
//! prefix present in the pinned IANA IPv4/IPv6 Special-Purpose Address registries
//! is denied, including protocol anycast/AS112/AMT entries that IANA marks as
//! globally reachable. Private/self-hosted deployments require a separate
//! institution-adopted egress theorem rather than an `allow_private` escape hatch.

use mycelix_institutional_core::Digest32;
use mycelix_integration_http_transport_policy::QualifiedHttpTransportProviderBinding;
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr};
use thiserror::Error;

pub const PUBLIC_HTTPS_ROUTE_PROFILE: &str =
    "mycelix-integration-http-public-egress-route-v2-blake3-framed";
pub const PUBLIC_ADDRESS_POLICY_PROFILE: &str =
    "mycelix-public-saas-egress-iana-special-purpose-deny-2025-10-09-v1";
pub const IANA_SPECIAL_PURPOSE_REGISTRY_SNAPSHOT: &str = "2025-10-09";
const DOMAIN_ROUTE: &[u8] = b"mycelix/integration/http-public-egress-route/v2";
const MAX_RESOLVED_ADDRESSES: usize = 16;
const MAX_TEXT_BYTES: usize = 1024;

// Exact IPv4 special-purpose registry prefixes as of 2025-10-09. More-specific
// entries inside 192.0.0.0/24 need no separate rows because public-SaaS policy
// rejects the whole parent special-purpose block. 255.255.255.255/32 is covered
// by 240.0.0.0/4.
const IANA_IPV4_SPECIAL_PREFIXES: &[([u8; 4], u8)] = &[
    ([0, 0, 0, 0], 8),
    ([10, 0, 0, 0], 8),
    ([100, 64, 0, 0], 10),
    ([127, 0, 0, 0], 8),
    ([169, 254, 0, 0], 16),
    ([172, 16, 0, 0], 12),
    ([192, 0, 0, 0], 24),
    ([192, 0, 2, 0], 24),
    ([192, 31, 196, 0], 24),
    ([192, 52, 193, 0], 24),
    ([192, 88, 99, 0], 24),
    ([192, 168, 0, 0], 16),
    ([192, 175, 48, 0], 24),
    ([198, 18, 0, 0], 15),
    ([198, 51, 100, 0], 24),
    ([203, 0, 113, 0], 24),
    ([240, 0, 0, 0], 4),
];

// Exact IPv6 special-purpose registry prefixes that intersect 2000::/3 as of
// 2025-10-09. Registry entries outside 2000::/3 already fail the outer GUA gate.
// The entire 2001::/23 parent is denied, including its globally-reachable
// protocol anycast/AMT/AS112/ORCHIDv2/DETs children.
const IANA_IPV6_SPECIAL_GUA_PREFIXES: &[([u8; 16], u8)] = &[
    ([0x20, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 23),
    ([0x20, 0x01, 0x0d, 0xb8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 32),
    ([0x20, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 16),
    ([0x26, 0x20, 0x00, 0x4f, 0x80, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 48),
    ([0x3f, 0xff, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 20),
];

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
/// public addresses under the pinned public-SaaS address policy.
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

    pub fn address_policy_profile(&self) -> &'static str {
        PUBLIC_ADDRESS_POLICY_PROFILE
    }

    pub fn registry_snapshot(&self) -> &'static str {
        IANA_SPECIAL_PURPOSE_REGISTRY_SNAPSHOT
    }

    pub const fn all_iana_special_purpose_prefixes_denied_here(&self) -> bool {
        true
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
    if observation
        .addresses
        .iter()
        .any(|address| !is_public_saas_unicast(*address))
    {
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

fn is_public_saas_unicast(address: IpAddr) -> bool {
    match address {
        IpAddr::V4(address) => ipv4_is_public_saas_unicast(address),
        IpAddr::V6(address) => ipv6_is_public_saas_unicast(address),
    }
}

fn ipv4_is_public_saas_unicast(address: Ipv4Addr) -> bool {
    if address.is_multicast() {
        return false;
    }
    !IANA_IPV4_SPECIAL_PREFIXES
        .iter()
        .any(|(network, prefix)| ipv4_matches_prefix(address, *network, *prefix))
}

fn ipv6_is_public_saas_unicast(address: Ipv6Addr) -> bool {
    let bytes = address.octets();
    // IANA's assignable IPv6 GUA envelope remains 2000::/3. This also denies
    // loopback/unspecified/mapped/NAT64/discard/ULA/link-local/SRv6 special-use
    // ranges before registry-specific GUA exclusions are evaluated.
    if bytes[0] & 0xe0 != 0x20 {
        return false;
    }
    !IANA_IPV6_SPECIAL_GUA_PREFIXES
        .iter()
        .any(|(network, prefix)| bytes_match_prefix(&bytes, network, *prefix))
}

fn ipv4_matches_prefix(address: Ipv4Addr, network: [u8; 4], prefix_len: u8) -> bool {
    bytes_match_prefix(&address.octets(), &network, prefix_len)
}

fn bytes_match_prefix(address: &[u8], network: &[u8], prefix_len: u8) -> bool {
    let full_bytes = usize::from(prefix_len / 8);
    let remaining_bits = prefix_len % 8;
    if address.len() != network.len()
        || full_bytes > address.len()
        || (full_bytes == address.len() && remaining_bits != 0)
    {
        return false;
    }
    if address[..full_bytes] != network[..full_bytes] {
        return false;
    }
    if remaining_bits == 0 {
        return true;
    }
    let mask = 0xff_u8 << (8 - remaining_bits);
    address[full_bytes] & mask == network[full_bytes] & mask
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
    frame(&mut h, PUBLIC_ADDRESS_POLICY_PROFILE.as_bytes());
    frame(
        &mut h,
        IANA_SPECIAL_PURPOSE_REGISTRY_SNAPSHOT.as_bytes(),
    );
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
    #[error("DNS resolution contains an address forbidden by the public-SaaS egress policy")]
    NonPublicAddress,
    #[error("no usable route window remains")]
    NoUsableRouteWindow,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn public_ipv4_examples_are_admitted() {
        assert!(ipv4_is_public_saas_unicast(Ipv4Addr::new(1, 1, 1, 1)));
        assert!(ipv4_is_public_saas_unicast(Ipv4Addr::new(8, 8, 8, 8)));
    }

    #[test]
    fn every_ipv4_registry_prefix_sample_fails_closed() {
        for (network, prefix) in IANA_IPV4_SPECIAL_PREFIXES {
            let address = Ipv4Addr::from(*network);
            assert!(
                !ipv4_is_public_saas_unicast(address),
                "{address}/{prefix} should fail"
            );
        }
    }

    #[test]
    fn globally_reachable_ipv4_special_purpose_ranges_still_fail() {
        for address in [
            Ipv4Addr::new(192, 0, 0, 9),
            Ipv4Addr::new(192, 0, 0, 10),
            Ipv4Addr::new(192, 31, 196, 1),
            Ipv4Addr::new(192, 52, 193, 1),
            Ipv4Addr::new(192, 175, 48, 1),
        ] {
            assert!(!ipv4_is_public_saas_unicast(address), "{address} should fail");
        }
    }

    #[test]
    fn multicast_ipv4_fails_even_though_not_in_special_registry() {
        assert!(!ipv4_is_public_saas_unicast(Ipv4Addr::new(224, 0, 0, 1)));
    }

    #[test]
    fn public_ipv6_example_is_admitted() {
        assert!(ipv6_is_public_saas_unicast(
            "2606:4700:4700::1111".parse().unwrap()
        ));
    }

    #[test]
    fn every_ipv6_special_gua_prefix_sample_fails_closed() {
        for (network, prefix) in IANA_IPV6_SPECIAL_GUA_PREFIXES {
            let address = Ipv6Addr::from(*network);
            assert!(
                !ipv6_is_public_saas_unicast(address),
                "{address}/{prefix} should fail"
            );
        }
    }

    #[test]
    fn globally_reachable_ipv6_special_purpose_ranges_still_fail() {
        for address in [
            "2001:1::1",
            "2001:1::2",
            "2001:1::3",
            "2001:3::1",
            "2001:4:112::1",
            "2001:20::1",
            "2001:30::1",
            "2620:4f:8000::1",
        ] {
            let address: Ipv6Addr = address.parse().unwrap();
            assert!(!ipv6_is_public_saas_unicast(address), "{address} should fail");
        }
    }

    #[test]
    fn non_gua_special_ranges_fail_at_outer_gate() {
        for address in [
            "::1",
            "64:ff9b::1",
            "64:ff9b:1::1",
            "100::1",
            "100:0:0:1::1",
            "5f00::1",
            "fc00::1",
            "fe80::1",
            "ff02::1",
        ] {
            let address: Ipv6Addr = address.parse().unwrap();
            assert!(!ipv6_is_public_saas_unicast(address), "{address} should fail");
        }
    }

    #[test]
    fn prefix_matcher_handles_partial_octets() {
        assert!(bytes_match_prefix(&[100, 64, 1, 1], &[100, 64, 0, 0], 10));
        assert!(bytes_match_prefix(&[172, 31, 255, 255], &[172, 16, 0, 0], 12));
        assert!(!bytes_match_prefix(&[100, 128, 0, 1], &[100, 64, 0, 0], 10));
        assert!(!bytes_match_prefix(&[172, 32, 0, 1], &[172, 16, 0, 0], 12));
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
