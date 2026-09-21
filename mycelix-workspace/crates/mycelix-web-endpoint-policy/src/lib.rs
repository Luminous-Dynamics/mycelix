#![forbid(unsafe_code)]
//! Zero-network IP endpoint classification for public-web acquisition.
//!
//! Classification only: no DNS, sockets, TLS, HTTP, EPI admission, or target authority.

use std::fmt;
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr};

pub const ORDINARY_PUBLIC_ENDPOINT_POLICY_V1: &str =
    "mycelix:web-endpoint-policy:ordinary-public-endpoint:v1";
pub const IANA_SPECIAL_PURPOSE_REGISTRY_PROFILE_V1: &str =
    "mycelix:iana-ip-special-purpose:2025-10-09:v1";
pub const IANA_IPV6_ADDRESS_SPACE_PROFILE_V1: &str =
    "mycelix:iana-ipv6-address-space:2025-10-23:v1";
pub const IPV6_CURRENT_IANA_GLOBAL_UNICAST_ENVELOPE_V1: &str = "2000::/3";

pub const IANA_IPV4_PREFIX_COUNT_V1: usize = 26;
pub const IANA_IPV6_PREFIX_COUNT_V1: usize = 25;
pub const EXTRA_POLICY_PREFIX_COUNT_V1: usize = 2;
pub const TOTAL_POLICY_PREFIX_COUNT_V1: usize =
    IANA_IPV4_PREFIX_COUNT_V1 + IANA_IPV6_PREFIX_COUNT_V1 + EXTRA_POLICY_PREFIX_COUNT_V1;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EndpointPolicyAuthorityScopeV1 {
    ClassificationOnly,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegistryBooleanV1 {
    True,
    False,
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EndpointPolicySourceV1 {
    IanaSpecialPurposeRegistry,
    MycelixExtraPolicy,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EndpointRefusalReasonV1 {
    SpecialPurposeRegistry,
    Multicast,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EndpointPolicyMatchV1 {
    pub prefix: &'static str,
    pub name: &'static str,
    pub prefix_len: u8,
    pub globally_reachable: RegistryBooleanV1,
    pub reserved_by_protocol: RegistryBooleanV1,
    pub source: EndpointPolicySourceV1,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EndpointPolicyDispositionV1 {
    OrdinaryPublicEndpointEligible,
    RefusedByPrefix {
        reason: EndpointRefusalReasonV1,
        matched: EndpointPolicyMatchV1,
    },
    RefusedOutsideCurrentIanaGlobalUnicastEnvelope {
        required_envelope: &'static str,
    },
}

impl EndpointPolicyDispositionV1 {
    pub const fn authority_scope(self) -> EndpointPolicyAuthorityScopeV1 {
        EndpointPolicyAuthorityScopeV1::ClassificationOnly
    }

    pub const fn is_ordinary_public_endpoint_eligible(self) -> bool {
        matches!(self, Self::OrdinaryPublicEndpointEligible)
    }
}

#[derive(Clone, Copy, Eq, PartialEq)]
pub struct EmbeddedIpv4AssessmentV1 {
    address: Ipv4Addr,
    disposition: EndpointPolicyDispositionV1,
}

impl EmbeddedIpv4AssessmentV1 {
    pub const fn address(self) -> Ipv4Addr {
        self.address
    }

    pub const fn disposition(self) -> EndpointPolicyDispositionV1 {
        self.disposition
    }
}

impl fmt::Debug for EmbeddedIpv4AssessmentV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("EmbeddedIpv4AssessmentV1")
            .field("address", &"<redacted>")
            .field("disposition", &self.disposition)
            .finish()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EndpointPolicyAssessmentV1 {
    disposition: EndpointPolicyDispositionV1,
    embedded_ipv4: Option<EmbeddedIpv4AssessmentV1>,
}

impl EndpointPolicyAssessmentV1 {
    pub const fn disposition(self) -> EndpointPolicyDispositionV1 {
        self.disposition
    }

    pub const fn embedded_ipv4(self) -> Option<EmbeddedIpv4AssessmentV1> {
        self.embedded_ipv4
    }

    pub const fn authority_scope(self) -> EndpointPolicyAuthorityScopeV1 {
        EndpointPolicyAuthorityScopeV1::ClassificationOnly
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum EndpointPolicyError {
    MalformedFrozenPrefix(&'static str),
    InvalidFrozenPrefixLength {
        prefix: &'static str,
        length: u8,
        max: u8,
    },
}

impl fmt::Display for EndpointPolicyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MalformedFrozenPrefix(prefix) => {
                write!(f, "frozen endpoint-policy prefix is malformed: {prefix}")
            }
            Self::InvalidFrozenPrefixLength { prefix, length, max } => write!(
                f,
                "frozen endpoint-policy prefix {prefix} has length {length}; maximum is {max}"
            ),
        }
    }
}

impl std::error::Error for EndpointPolicyError {}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum IpFamilyV1 {
    V4,
    V6,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct PolicyRowV1 {
    family: IpFamilyV1,
    prefix: &'static str,
    name: &'static str,
    globally_reachable: RegistryBooleanV1,
    reserved_by_protocol: RegistryBooleanV1,
    source: EndpointPolicySourceV1,
    refusal_reason: EndpointRefusalReasonV1,
}

macro_rules! rb {
    (T) => { RegistryBooleanV1::True };
    (F) => { RegistryBooleanV1::False };
    (U) => { RegistryBooleanV1::Unknown };
}

macro_rules! row4 {
    ($prefix:literal, $name:literal, $global:tt, $reserved:tt) => {
        PolicyRowV1 {
            family: IpFamilyV1::V4,
            prefix: $prefix,
            name: $name,
            globally_reachable: rb!($global),
            reserved_by_protocol: rb!($reserved),
            source: EndpointPolicySourceV1::IanaSpecialPurposeRegistry,
            refusal_reason: EndpointRefusalReasonV1::SpecialPurposeRegistry,
        }
    };
}

macro_rules! row6 {
    ($prefix:literal, $name:literal, $global:tt, $reserved:tt) => {
        PolicyRowV1 {
            family: IpFamilyV1::V6,
            prefix: $prefix,
            name: $name,
            globally_reachable: rb!($global),
            reserved_by_protocol: rb!($reserved),
            source: EndpointPolicySourceV1::IanaSpecialPurposeRegistry,
            refusal_reason: EndpointRefusalReasonV1::SpecialPurposeRegistry,
        }
    };
}

macro_rules! extra4 {
    ($prefix:literal, $name:literal, $reason:ident) => {
        PolicyRowV1 {
            family: IpFamilyV1::V4,
            prefix: $prefix,
            name: $name,
            globally_reachable: RegistryBooleanV1::Unknown,
            reserved_by_protocol: RegistryBooleanV1::Unknown,
            source: EndpointPolicySourceV1::MycelixExtraPolicy,
            refusal_reason: EndpointRefusalReasonV1::$reason,
        }
    };
}

macro_rules! extra6 {
    ($prefix:literal, $name:literal, $reason:ident) => {
        PolicyRowV1 {
            family: IpFamilyV1::V6,
            prefix: $prefix,
            name: $name,
            globally_reachable: RegistryBooleanV1::Unknown,
            reserved_by_protocol: RegistryBooleanV1::Unknown,
            source: EndpointPolicySourceV1::MycelixExtraPolicy,
            refusal_reason: EndpointRefusalReasonV1::$reason,
        }
    };
}

const POLICY_ROWS_V1: &[PolicyRowV1] = &[
    row4!("0.0.0.0/8", "This network", F, T),
    row4!("0.0.0.0/32", "This host on this network", F, T),
    row4!("10.0.0.0/8", "Private-Use", F, F),
    row4!("100.64.0.0/10", "Shared Address Space", F, F),
    row4!("127.0.0.0/8", "Loopback", F, T),
    row4!("169.254.0.0/16", "Link Local", F, T),
    row4!("172.16.0.0/12", "Private-Use", F, F),
    row4!("192.0.0.0/24", "IETF Protocol Assignments", F, F),
    row4!("192.0.0.0/29", "IPv4 Service Continuity Prefix", F, F),
    row4!("192.0.0.8/32", "IPv4 dummy address", F, F),
    row4!("192.0.0.9/32", "Port Control Protocol Anycast", T, F),
    row4!("192.0.0.10/32", "Traversal Using Relays around NAT Anycast", T, F),
    row4!("192.0.0.170/32", "NAT64/DNS64 Discovery", F, T),
    row4!("192.0.0.171/32", "NAT64/DNS64 Discovery", F, T),
    row4!("192.0.2.0/24", "Documentation (TEST-NET-1)", F, F),
    row4!("192.31.196.0/24", "AS112-v4", T, F),
    row4!("192.52.193.0/24", "AMT", T, F),
    row4!("192.88.99.0/24", "Deprecated (6to4 Relay Anycast)", U, U),
    row4!("192.88.99.2/32", "6a44-relay anycast address", F, F),
    row4!("192.168.0.0/16", "Private-Use", F, F),
    row4!("192.175.48.0/24", "Direct Delegation AS112 Service", T, F),
    row4!("198.18.0.0/15", "Benchmarking", F, F),
    row4!("198.51.100.0/24", "Documentation (TEST-NET-2)", F, F),
    row4!("203.0.113.0/24", "Documentation (TEST-NET-3)", F, F),
    row4!("240.0.0.0/4", "Reserved", F, T),
    row4!("255.255.255.255/32", "Limited Broadcast", F, T),
    row6!("::1/128", "Loopback Address", F, T),
    row6!("::/128", "Unspecified Address", F, T),
    row6!("::ffff:0:0/96", "IPv4-mapped Address", F, T),
    row6!("64:ff9b::/96", "IPv4-IPv6 Translation", T, F),
    row6!("64:ff9b:1::/48", "IPv4-IPv6 Translation", F, F),
    row6!("100::/64", "Discard-Only Address Block", F, F),
    row6!("100:0:0:1::/64", "Dummy IPv6 Prefix", F, F),
    row6!("2001::/23", "IETF Protocol Assignments", F, F),
    row6!("2001::/32", "TEREDO", U, F),
    row6!("2001:1::1/128", "Port Control Protocol Anycast", T, F),
    row6!("2001:1::2/128", "Traversal Using Relays around NAT Anycast", T, F),
    row6!("2001:1::3/128", "DNS-SD Service Registration Protocol Anycast", T, F),
    row6!("2001:2::/48", "Benchmarking", F, F),
    row6!("2001:3::/32", "AMT", T, F),
    row6!("2001:4:112::/48", "AS112-v6", T, F),
    row6!("2001:10::/28", "Deprecated ORCHID", U, U),
    row6!("2001:20::/28", "ORCHIDv2", T, F),
    row6!("2001:30::/28", "Drone Remote ID Protocol Entity Tags (DETs) Prefix", T, F),
    row6!("2001:db8::/32", "Documentation", F, F),
    row6!("2002::/16", "6to4", U, F),
    row6!("2620:4f:8000::/48", "Direct Delegation AS112 Service", T, F),
    row6!("3fff::/20", "Documentation", F, F),
    row6!("5f00::/16", "Segment Routing (SRv6) SIDs", F, F),
    row6!("fc00::/7", "Unique-Local", F, F),
    row6!("fe80::/10", "Link-Local Unicast", F, T),
    extra4!("224.0.0.0/4", "IPv4 multicast", Multicast),
    extra6!("ff00::/8", "IPv6 multicast", Multicast),
];

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct OrdinaryPublicEndpointPolicyV1;

impl OrdinaryPublicEndpointPolicyV1 {
    pub const fn profile_id(self) -> &'static str {
        ORDINARY_PUBLIC_ENDPOINT_POLICY_V1
    }

    pub const fn special_purpose_registry_profile_id(self) -> &'static str {
        IANA_SPECIAL_PURPOSE_REGISTRY_PROFILE_V1
    }

    pub const fn ipv6_address_space_profile_id(self) -> &'static str {
        IANA_IPV6_ADDRESS_SPACE_PROFILE_V1
    }

    pub const fn authority_scope(self) -> EndpointPolicyAuthorityScopeV1 {
        EndpointPolicyAuthorityScopeV1::ClassificationOnly
    }

    pub fn classify(
        self,
        address: IpAddr,
    ) -> Result<EndpointPolicyAssessmentV1, EndpointPolicyError> {
        match address {
            IpAddr::V4(address) => Ok(EndpointPolicyAssessmentV1 {
                disposition: classify_v4(address)?,
                embedded_ipv4: None,
            }),
            IpAddr::V6(address) => {
                let disposition = classify_v6(address)?;
                let embedded_ipv4 = match address.to_ipv4_mapped() {
                    Some(embedded) => Some(EmbeddedIpv4AssessmentV1 {
                        address: embedded,
                        disposition: classify_v4(embedded)?,
                    }),
                    None => None,
                };
                Ok(EndpointPolicyAssessmentV1 {
                    disposition,
                    embedded_ipv4,
                })
            }
        }
    }
}

fn classify_v4(address: Ipv4Addr) -> Result<EndpointPolicyDispositionV1, EndpointPolicyError> {
    match longest_match_v4(address)? {
        Some((row, prefix_len)) => Ok(prefix_refusal(row, prefix_len)),
        None => Ok(EndpointPolicyDispositionV1::OrdinaryPublicEndpointEligible),
    }
}

fn classify_v6(address: Ipv6Addr) -> Result<EndpointPolicyDispositionV1, EndpointPolicyError> {
    if let Some((row, prefix_len)) = longest_match_v6(address)? {
        return Ok(prefix_refusal(row, prefix_len));
    }

    let (envelope, prefix_len) = parse_v6_prefix(IPV6_CURRENT_IANA_GLOBAL_UNICAST_ENVELOPE_V1)?;
    if contains_v6(envelope, prefix_len, address) {
        Ok(EndpointPolicyDispositionV1::OrdinaryPublicEndpointEligible)
    } else {
        Ok(
            EndpointPolicyDispositionV1::RefusedOutsideCurrentIanaGlobalUnicastEnvelope {
                required_envelope: IPV6_CURRENT_IANA_GLOBAL_UNICAST_ENVELOPE_V1,
            },
        )
    }
}

fn prefix_refusal(row: PolicyRowV1, prefix_len: u8) -> EndpointPolicyDispositionV1 {
    EndpointPolicyDispositionV1::RefusedByPrefix {
        reason: row.refusal_reason,
        matched: EndpointPolicyMatchV1 {
            prefix: row.prefix,
            name: row.name,
            prefix_len,
            globally_reachable: row.globally_reachable,
            reserved_by_protocol: row.reserved_by_protocol,
            source: row.source,
        },
    }
}

fn longest_match_v4(
    address: Ipv4Addr,
) -> Result<Option<(PolicyRowV1, u8)>, EndpointPolicyError> {
    let mut best = None;
    for row in POLICY_ROWS_V1 {
        if row.family != IpFamilyV1::V4 {
            continue;
        }
        let (network, prefix_len) = parse_v4_prefix(row.prefix)?;
        let replace = match best {
            None => true,
            Some((_, best_len)) => prefix_len > best_len,
        };
        if contains_v4(network, prefix_len, address) && replace {
            best = Some((*row, prefix_len));
        }
    }
    Ok(best)
}

fn longest_match_v6(
    address: Ipv6Addr,
) -> Result<Option<(PolicyRowV1, u8)>, EndpointPolicyError> {
    let mut best = None;
    for row in POLICY_ROWS_V1 {
        if row.family != IpFamilyV1::V6 {
            continue;
        }
        let (network, prefix_len) = parse_v6_prefix(row.prefix)?;
        let replace = match best {
            None => true,
            Some((_, best_len)) => prefix_len > best_len,
        };
        if contains_v6(network, prefix_len, address) && replace {
            best = Some((*row, prefix_len));
        }
    }
    Ok(best)
}

fn parse_v4_prefix(prefix: &'static str) -> Result<(Ipv4Addr, u8), EndpointPolicyError> {
    let (network, length) = prefix
        .rsplit_once('/')
        .ok_or(EndpointPolicyError::MalformedFrozenPrefix(prefix))?;
    let network = network
        .parse::<Ipv4Addr>()
        .map_err(|_| EndpointPolicyError::MalformedFrozenPrefix(prefix))?;
    let length = length
        .parse::<u8>()
        .map_err(|_| EndpointPolicyError::MalformedFrozenPrefix(prefix))?;
    if length > 32 {
        return Err(EndpointPolicyError::InvalidFrozenPrefixLength {
            prefix,
            length,
            max: 32,
        });
    }
    Ok((network, length))
}

fn parse_v6_prefix(prefix: &'static str) -> Result<(Ipv6Addr, u8), EndpointPolicyError> {
    let (network, length) = prefix
        .rsplit_once('/')
        .ok_or(EndpointPolicyError::MalformedFrozenPrefix(prefix))?;
    let network = network
        .parse::<Ipv6Addr>()
        .map_err(|_| EndpointPolicyError::MalformedFrozenPrefix(prefix))?;
    let length = length
        .parse::<u8>()
        .map_err(|_| EndpointPolicyError::MalformedFrozenPrefix(prefix))?;
    if length > 128 {
        return Err(EndpointPolicyError::InvalidFrozenPrefixLength {
            prefix,
            length,
            max: 128,
        });
    }
    Ok((network, length))
}

fn contains_v4(network: Ipv4Addr, prefix_len: u8, address: Ipv4Addr) -> bool {
    let mask = if prefix_len == 0 {
        0
    } else {
        u32::MAX << (32 - prefix_len)
    };
    (u32::from(network) & mask) == (u32::from(address) & mask)
}

fn contains_v6(network: Ipv6Addr, prefix_len: u8, address: Ipv6Addr) -> bool {
    let mask = if prefix_len == 0 {
        0
    } else {
        u128::MAX << (128 - prefix_len)
    };
    (u128::from(network) & mask) == (u128::from(address) & mask)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;
    use std::collections::HashSet;

    const REGISTRY_SEED: &str = include_str!(
        "../../../docs/epistemic/fixtures/WEB_NET_REGISTRY_TEST_001_SEED_V0_1.json"
    );
    const ALLOCATION_SEED: &str = include_str!(
        "../../../docs/epistemic/fixtures/WEB_NET_ALLOC_TEST_001_IPV6_ENVELOPE_V0_1.json"
    );

    #[test]
    fn compiled_prefix_set_exactly_matches_frozen_registry_fixture() {
        let root: Value = serde_json::from_str(REGISTRY_SEED).expect("registry fixture JSON");
        let mut fixture = HashSet::new();
        for key in ["ipv4_entries", "ipv6_entries", "extra_policy_prefixes"] {
            for entry in root[key].as_array().expect("fixture prefix array") {
                fixture.insert(entry["prefix"].as_str().expect("prefix"));
            }
        }
        let compiled: HashSet<&str> = POLICY_ROWS_V1.iter().map(|row| row.prefix).collect();
        assert_eq!(fixture.len(), TOTAL_POLICY_PREFIX_COUNT_V1);
        assert_eq!(compiled.len(), TOTAL_POLICY_PREFIX_COUNT_V1);
        assert_eq!(fixture, compiled);
    }

    #[test]
    fn registry_metadata_matches_frozen_fixture() {
        let root: Value = serde_json::from_str(REGISTRY_SEED).expect("registry fixture JSON");
        for key in ["ipv4_entries", "ipv6_entries"] {
            for entry in root[key].as_array().expect("registry entries") {
                let prefix = entry["prefix"].as_str().expect("prefix");
                let row = POLICY_ROWS_V1
                    .iter()
                    .find(|row| row.prefix == prefix)
                    .expect("compiled row");
                assert_eq!(row.name, entry["name"].as_str().expect("name"));
                assert_eq!(
                    row.globally_reachable,
                    json_registry_bool(&entry["iana_globally_reachable"])
                );
                assert_eq!(
                    row.reserved_by_protocol,
                    json_registry_bool(&entry["iana_reserved_by_protocol"])
                );
            }
        }
    }

    fn json_registry_bool(value: &Value) -> RegistryBooleanV1 {
        match value.as_bool() {
            Some(true) => RegistryBooleanV1::True,
            Some(false) => RegistryBooleanV1::False,
            None => RegistryBooleanV1::Unknown,
        }
    }

    #[test]
    fn allocation_fixture_binds_exact_v1_envelope() {
        let root: Value = serde_json::from_str(ALLOCATION_SEED).expect("allocation fixture JSON");
        assert_eq!(
            root["policy"]["ipv6_current_iana_global_unicast_envelope"]
                .as_str()
                .expect("envelope"),
            IPV6_CURRENT_IANA_GLOBAL_UNICAST_ENVELOPE_V1
        );
        let entries = root["top_level_entries"].as_array().expect("top-level entries");
        assert_eq!(entries.len(), 20);
        let current: Vec<_> = entries
            .iter()
            .filter(|entry| {
                entry["policy_v1"].as_str() == Some("current_iana_global_unicast_envelope")
            })
            .collect();
        assert_eq!(current.len(), 1);
        assert_eq!(current[0]["prefix"].as_str(), Some("2000::/3"));
    }

    #[test]
    fn frozen_registry_vectors_match() {
        let root: Value = serde_json::from_str(REGISTRY_SEED).expect("registry fixture JSON");
        let policy = OrdinaryPublicEndpointPolicyV1;
        for vector in root["vectors"].as_array().expect("vectors") {
            let id = vector["id"].as_str().expect("id");
            let address = vector["address"]
                .as_str()
                .expect("address")
                .parse::<IpAddr>()
                .expect("valid address");
            let assessment = policy.classify(address).expect("valid frozen table");
            match vector["expected"].as_str().expect("expected") {
                "refuse" => {
                    let expected_prefix = vector["matched"].as_str().expect("matched");
                    let EndpointPolicyDispositionV1::RefusedByPrefix { matched, .. } =
                        assessment.disposition()
                    else {
                        panic!("{id}: expected prefix refusal");
                    };
                    assert_eq!(matched.prefix, expected_prefix, "{id}");
                }
                "ordinary_public_endpoint_eligible" => assert_eq!(
                    assessment.disposition(),
                    EndpointPolicyDispositionV1::OrdinaryPublicEndpointEligible,
                    "{id}"
                ),
                other => panic!("{id}: unknown expected state {other}"),
            }
        }
    }

    #[test]
    fn frozen_allocation_vectors_match_with_specific_reason_precedence() {
        let root: Value = serde_json::from_str(ALLOCATION_SEED).expect("allocation fixture JSON");
        let policy = OrdinaryPublicEndpointPolicyV1;

        for vector in root["vectors"].as_array().expect("vectors") {
            let id = vector["id"].as_str().expect("id");
            let address = vector["address"]
                .as_str()
                .expect("address")
                .parse::<IpAddr>()
                .expect("valid address");
            let assessment = policy.classify(address).expect("valid frozen table");
            match vector["expected"].as_str().expect("expected") {
                "ordinary_public_endpoint_eligible" => assert_eq!(
                    assessment.disposition(),
                    EndpointPolicyDispositionV1::OrdinaryPublicEndpointEligible,
                    "{id}"
                ),
                "refuse_outside_current_iana_global_unicast_envelope" => assert_eq!(
                    assessment.disposition(),
                    EndpointPolicyDispositionV1::RefusedOutsideCurrentIanaGlobalUnicastEnvelope {
                        required_envelope: IPV6_CURRENT_IANA_GLOBAL_UNICAST_ENVELOPE_V1,
                    },
                    "{id}"
                ),
                "refuse_special_purpose" => {
                    let expected_prefix = vector["matched"].as_str().expect("matched");
                    let EndpointPolicyDispositionV1::RefusedByPrefix { reason, matched } =
                        assessment.disposition()
                    else {
                        panic!("{id}: expected special-purpose refusal");
                    };
                    assert_eq!(reason, EndpointRefusalReasonV1::SpecialPurposeRegistry, "{id}");
                    assert_eq!(matched.prefix, expected_prefix, "{id}");
                }
                "refuse_multicast" => {
                    let expected_prefix = vector["matched"].as_str().expect("matched");
                    let EndpointPolicyDispositionV1::RefusedByPrefix { reason, matched } =
                        assessment.disposition()
                    else {
                        panic!("{id}: expected multicast refusal");
                    };
                    assert_eq!(reason, EndpointRefusalReasonV1::Multicast, "{id}");
                    assert_eq!(matched.prefix, expected_prefix, "{id}");
                }
                other => panic!("{id}: unknown expected state {other}"),
            }
            assert_eq!(
                assessment.authority_scope(),
                EndpointPolicyAuthorityScopeV1::ClassificationOnly,
                "{id}"
            );
        }
    }

    #[test]
    fn longest_prefix_and_embedded_ipv4_are_retained() {
        let policy = OrdinaryPublicEndpointPolicyV1;

        let specific = policy
            .classify(IpAddr::V4(Ipv4Addr::new(192, 0, 0, 9)))
            .expect("valid frozen table");
        let EndpointPolicyDispositionV1::RefusedByPrefix { matched, .. } =
            specific.disposition()
        else {
            panic!("expected refusal");
        };
        assert_eq!(matched.prefix, "192.0.0.9/32");
        assert_eq!(matched.globally_reachable, RegistryBooleanV1::True);

        let mapped = policy
            .classify("::ffff:127.0.0.1".parse().expect("mapped IPv6"))
            .expect("valid frozen table");
        let embedded = mapped.embedded_ipv4().expect("embedded IPv4");
        let EndpointPolicyDispositionV1::RefusedByPrefix { matched, .. } =
            embedded.disposition()
        else {
            panic!("embedded loopback must refuse");
        };
        assert_eq!(matched.prefix, "127.0.0.0/8");

        let rendered = format!("{embedded:?}");
        assert!(!rendered.contains("127.0.0.1"));
    }

    #[test]
    fn every_frozen_prefix_and_envelope_parse() {
        assert_eq!(POLICY_ROWS_V1.len(), TOTAL_POLICY_PREFIX_COUNT_V1);
        for row in POLICY_ROWS_V1 {
            match row.family {
                IpFamilyV1::V4 => {
                    parse_v4_prefix(row.prefix).expect("frozen IPv4 prefix must parse");
                }
                IpFamilyV1::V6 => {
                    parse_v6_prefix(row.prefix).expect("frozen IPv6 prefix must parse");
                }
            }
        }
        parse_v6_prefix(IPV6_CURRENT_IANA_GLOBAL_UNICAST_ENVELOPE_V1)
            .expect("frozen allocation envelope must parse");
    }
}
