#![forbid(unsafe_code)]
//! Zero-network special-use domain classification for public-web acquisition.
//!
//! This crate classifies a bounded, post-parser ASCII DNS-name observation.
//! It deliberately does **not** perform URL parsing, IDNA conversion, DNS,
//! socket I/O, HTTP, evidence admission, or target authorization.

use std::fmt;

/// Exact frozen domain-policy profile identifier.
pub const ORDINARY_PUBLIC_DNS_POLICY_V1: &str = "mycelix:web-domain-policy:ordinary-public-dns:v1";

/// Exact frozen source-corpus profile identifier.
pub const SPECIAL_USE_REGISTRY_PROFILE_V1: &str =
    "mycelix:iana-special-use-domain-names:2026-05-22:v1";

/// Number of normalized special-use suffixes frozen by WEB-DOMAIN-POLICY-TEST-001.
pub const SPECIAL_USE_SUFFIX_COUNT_V1: usize = 42;

/// Maximum normalized DNS presentation length accepted by this policy layer.
pub const MAX_NORMALIZED_DNS_NAME_BYTES: usize = 253;

/// Machine-readable authority ceiling for this crate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DomainPolicyAuthorityScopeV1 {
    /// Classification only; no resolver or network capability is granted.
    ClassificationOnly,
}

/// Non-authoritative explanation of why another resolver profile might be needed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResolverClassHintV1 {
    AlternateNamespace,
    HomeNetwork,
    Mdns,
    LocalhostSynthetic,
    TorOnion,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct SpecialUseSuffixV1 {
    suffix: &'static str,
    hint: Option<ResolverClassHintV1>,
}

const SPECIAL_USE_SUFFIXES_V1: &[SpecialUseSuffixV1] = &[
    SpecialUseSuffixV1 { suffix: "alt", hint: Some(ResolverClassHintV1::AlternateNamespace) },
    SpecialUseSuffixV1 { suffix: "6tisch.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "eap.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "eap-noob.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "home.arpa", hint: Some(ResolverClassHintV1::HomeNetwork) },
    SpecialUseSuffixV1 { suffix: "10.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "254.169.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "16.172.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "17.172.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "18.172.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "19.172.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "20.172.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "21.172.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "22.172.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "23.172.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "24.172.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "25.172.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "26.172.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "27.172.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "28.172.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "29.172.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "30.172.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "31.172.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "170.0.0.192.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "171.0.0.192.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "168.192.in-addr.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "8.e.f.ip6.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "9.e.f.ip6.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "a.e.f.ip6.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "b.e.f.ip6.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "ipv4only.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "resolver.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "service.arpa", hint: None },
    SpecialUseSuffixV1 { suffix: "example", hint: None },
    SpecialUseSuffixV1 { suffix: "example.com", hint: None },
    SpecialUseSuffixV1 { suffix: "example.net", hint: None },
    SpecialUseSuffixV1 { suffix: "example.org", hint: None },
    SpecialUseSuffixV1 { suffix: "invalid", hint: None },
    SpecialUseSuffixV1 { suffix: "local", hint: Some(ResolverClassHintV1::Mdns) },
    SpecialUseSuffixV1 { suffix: "localhost", hint: Some(ResolverClassHintV1::LocalhostSynthetic) },
    SpecialUseSuffixV1 { suffix: "onion", hint: Some(ResolverClassHintV1::TorOnion) },
    SpecialUseSuffixV1 { suffix: "test", hint: None },
];

/// Bounded ASCII DNS-name observation intended to be produced after URL/IDNA parsing.
///
/// Construction validates only representation shape. It does **not** establish
/// that a qualified parser produced the value.
#[derive(Clone, Eq, PartialEq)]
pub struct PostParserDnsNameV1 {
    supplied_ascii: String,
    comparison_key: String,
}

impl PostParserDnsNameV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, DomainPolicyError> {
        let supplied_ascii = value.into();
        if supplied_ascii.is_empty() {
            return Err(DomainPolicyError::EmptyName);
        }
        if !supplied_ascii.is_ascii() {
            return Err(DomainPolicyError::NonAsciiName);
        }
        if supplied_ascii.len() > MAX_NORMALIZED_DNS_NAME_BYTES + 1 {
            return Err(DomainPolicyError::NameTooLong {
                bytes: supplied_ascii.len(),
                max: MAX_NORMALIZED_DNS_NAME_BYTES + 1,
            });
        }

        let core = supplied_ascii.strip_suffix('.').unwrap_or(&supplied_ascii);
        if core.is_empty() {
            return Err(DomainPolicyError::EmptyName);
        }
        if core.len() > MAX_NORMALIZED_DNS_NAME_BYTES {
            return Err(DomainPolicyError::NameTooLong {
                bytes: core.len(),
                max: MAX_NORMALIZED_DNS_NAME_BYTES,
            });
        }

        let mut comparison_key = core.to_owned();
        comparison_key.make_ascii_lowercase();

        for label in comparison_key.split('.') {
            if label.is_empty() {
                return Err(DomainPolicyError::EmptyLabel);
            }
            if label.len() > 63 {
                return Err(DomainPolicyError::LabelTooLong { bytes: label.len() });
            }
            if !label
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || byte == b'-')
            {
                return Err(DomainPolicyError::UnsupportedAsciiLabelCharacter);
            }
        }

        Ok(Self {
            supplied_ascii,
            comparison_key,
        })
    }

    /// Exact caller-supplied ASCII observation. This may be sensitive target data.
    pub fn supplied_ascii(&self) -> &str {
        &self.supplied_ascii
    }

    /// Normalized comparison key used only by this policy profile.
    pub fn comparison_key(&self) -> &str {
        &self.comparison_key
    }

    pub fn label_count(&self) -> usize {
        self.comparison_key.split('.').count()
    }
}

impl fmt::Debug for PostParserDnsNameV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PostParserDnsNameV1")
            .field("bytes", &self.supplied_ascii.len())
            .field("labels", &self.label_count())
            .field("value", &"<redacted>")
            .finish()
    }
}

/// Typed policy disposition. No variant grants DNS or network authority.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DomainPolicyDecisionV1 {
    OrdinaryPublicDnsEligible,
    RefusedSpecialUse {
        matched_suffix: &'static str,
        resolver_class_hint: Option<ResolverClassHintV1>,
    },
    RefusedSingleLabel,
}

impl DomainPolicyDecisionV1 {
    pub const fn authority_scope(self) -> DomainPolicyAuthorityScopeV1 {
        DomainPolicyAuthorityScopeV1::ClassificationOnly
    }

    pub const fn is_ordinary_public_dns_eligible(self) -> bool {
        matches!(self, Self::OrdinaryPublicDnsEligible)
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DomainPolicyError {
    EmptyName,
    NonAsciiName,
    NameTooLong { bytes: usize, max: usize },
    EmptyLabel,
    LabelTooLong { bytes: usize },
    UnsupportedAsciiLabelCharacter,
}

impl fmt::Display for DomainPolicyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyName => f.write_str("DNS-name observation is empty"),
            Self::NonAsciiName => f.write_str("DNS-name observation is not post-parser ASCII"),
            Self::NameTooLong { bytes, max } => {
                write!(f, "DNS-name observation is {bytes} bytes; maximum is {max}")
            }
            Self::EmptyLabel => f.write_str("DNS-name observation contains an empty label"),
            Self::LabelTooLong { bytes } => {
                write!(f, "DNS label is {bytes} bytes; maximum is 63")
            }
            Self::UnsupportedAsciiLabelCharacter => {
                f.write_str("DNS-name observation contains an unsupported ASCII label character")
            }
        }
    }
}

impl std::error::Error for DomainPolicyError {}

/// Pure classifier for the frozen ordinary-public-DNS V1 profile.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct OrdinaryPublicDnsPolicyV1;

impl OrdinaryPublicDnsPolicyV1 {
    pub const fn profile_id(self) -> &'static str {
        ORDINARY_PUBLIC_DNS_POLICY_V1
    }

    pub const fn registry_profile_id(self) -> &'static str {
        SPECIAL_USE_REGISTRY_PROFILE_V1
    }

    pub const fn authority_scope(self) -> DomainPolicyAuthorityScopeV1 {
        DomainPolicyAuthorityScopeV1::ClassificationOnly
    }

    pub fn classify(self, name: &PostParserDnsNameV1) -> DomainPolicyDecisionV1 {
        let key = name.comparison_key();

        for entry in SPECIAL_USE_SUFFIXES_V1 {
            if label_suffix_match(key, entry.suffix) {
                return DomainPolicyDecisionV1::RefusedSpecialUse {
                    matched_suffix: entry.suffix,
                    resolver_class_hint: entry.hint,
                };
            }
        }

        if name.label_count() == 1 {
            return DomainPolicyDecisionV1::RefusedSingleLabel;
        }

        DomainPolicyDecisionV1::OrdinaryPublicDnsEligible
    }
}

fn label_suffix_match(name: &str, suffix: &str) -> bool {
    name == suffix
        || name
            .strip_suffix(suffix)
            .is_some_and(|prefix| prefix.ends_with('.'))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;
    use std::collections::HashSet;

    const SEED: &str = include_str!(
        "../../../docs/epistemic/fixtures/WEB_DOMAIN_POLICY_TEST_001_SEED_V0_1.json"
    );

    fn parsed(value: &str) -> PostParserDnsNameV1 {
        PostParserDnsNameV1::new(value).expect("valid bounded post-parser ASCII DNS name")
    }

    #[test]
    fn compiled_suffix_table_exactly_matches_frozen_fixture() {
        let root: Value = serde_json::from_str(SEED).expect("fixture JSON must parse");
        let entries = root["entries"].as_array().expect("entries array");
        assert_eq!(entries.len(), SPECIAL_USE_SUFFIX_COUNT_V1);
        assert_eq!(SPECIAL_USE_SUFFIXES_V1.len(), SPECIAL_USE_SUFFIX_COUNT_V1);

        let fixture: HashSet<&str> = entries
            .iter()
            .map(|entry| entry["suffix"].as_str().expect("suffix"))
            .collect();
        let compiled: HashSet<&str> = SPECIAL_USE_SUFFIXES_V1
            .iter()
            .map(|entry| entry.suffix)
            .collect();

        assert_eq!(fixture.len(), SPECIAL_USE_SUFFIX_COUNT_V1);
        assert_eq!(compiled.len(), SPECIAL_USE_SUFFIX_COUNT_V1);
        assert_eq!(fixture, compiled);
    }

    #[test]
    fn every_special_use_suffix_and_subdomain_refuses() {
        let policy = OrdinaryPublicDnsPolicyV1;
        for entry in SPECIAL_USE_SUFFIXES_V1 {
            let exact = policy.classify(&parsed(entry.suffix));
            assert_eq!(
                exact,
                DomainPolicyDecisionV1::RefusedSpecialUse {
                    matched_suffix: entry.suffix,
                    resolver_class_hint: entry.hint,
                },
                "exact suffix {}",
                entry.suffix
            );

            let child = format!("probe.{}", entry.suffix);
            let child_decision = policy.classify(&parsed(&child));
            assert_eq!(
                child_decision,
                DomainPolicyDecisionV1::RefusedSpecialUse {
                    matched_suffix: entry.suffix,
                    resolver_class_hint: entry.hint,
                },
                "subdomain {}",
                child
            );
        }
    }

    #[test]
    fn frozen_policy_vectors_match() {
        let root: Value = serde_json::from_str(SEED).expect("fixture JSON must parse");
        let vectors = root["vectors"].as_array().expect("vectors array");
        let policy = OrdinaryPublicDnsPolicyV1;

        for vector in vectors {
            let id = vector["id"].as_str().expect("id");
            let host = vector["host"].as_str().expect("host");
            let expected = vector["expected"].as_str().expect("expected");
            let decision = policy.classify(&parsed(host));

            match expected {
                "refuse_special_use" => {
                    let expected_suffix = vector["matched_suffix"].as_str().expect("matched suffix");
                    match decision {
                        DomainPolicyDecisionV1::RefusedSpecialUse { matched_suffix, .. } => {
                            assert_eq!(matched_suffix, expected_suffix, "{id}: matched suffix");
                        }
                        other => panic!("{id}: expected special-use refusal, got {other:?}"),
                    }
                }
                "refuse_single_label" => {
                    assert_eq!(decision, DomainPolicyDecisionV1::RefusedSingleLabel, "{id}");
                }
                "ordinary_public_dns_eligible" => {
                    assert_eq!(
                        decision,
                        DomainPolicyDecisionV1::OrdinaryPublicDnsEligible,
                        "{id}"
                    );
                }
                other => panic!("{id}: unknown expected vector state {other}"),
            }

            assert_eq!(
                decision.authority_scope(),
                DomainPolicyAuthorityScopeV1::ClassificationOnly,
                "{id}: authority ceiling"
            );
        }
    }

    #[test]
    fn suffix_matching_is_label_boundary_aware() {
        let policy = OrdinaryPublicDnsPolicyV1;
        let name = parsed("evil-localhost.public-synthetic.invalidtld");
        assert_eq!(
            policy.classify(&name),
            DomainPolicyDecisionV1::OrdinaryPublicDnsEligible
        );
    }

    #[test]
    fn case_and_trailing_dot_normalize_only_for_policy_comparison() {
        let name = parsed("LOCALHOST.");
        assert_eq!(name.supplied_ascii(), "LOCALHOST.");
        assert_eq!(name.comparison_key(), "localhost");
        assert_eq!(
            OrdinaryPublicDnsPolicyV1.classify(&name),
            DomainPolicyDecisionV1::RefusedSpecialUse {
                matched_suffix: "localhost",
                resolver_class_hint: Some(ResolverClassHintV1::LocalhostSynthetic),
            }
        );
    }

    #[test]
    fn single_label_refusal_is_not_special_use_by_default() {
        let decision = OrdinaryPublicDnsPolicyV1.classify(&parsed("printer"));
        assert_eq!(decision, DomainPolicyDecisionV1::RefusedSingleLabel);
    }

    #[test]
    fn input_shape_rejects_unicode_empty_labels_and_oversized_labels() {
        assert_eq!(
            PostParserDnsNameV1::new("例え.テスト"),
            Err(DomainPolicyError::NonAsciiName)
        );
        assert_eq!(
            PostParserDnsNameV1::new("foo..bar"),
            Err(DomainPolicyError::EmptyLabel)
        );
        let oversized = format!("{}.example", "a".repeat(64));
        assert_eq!(
            PostParserDnsNameV1::new(oversized),
            Err(DomainPolicyError::LabelTooLong { bytes: 64 })
        );
    }

    #[test]
    fn debug_does_not_emit_target_name() {
        let name = parsed("sensitive-investigation.public-synthetic.invalidtld");
        let debug = format!("{name:?}");
        assert!(!debug.contains("sensitive-investigation"));
        assert!(debug.contains("<redacted>"));
    }

    #[test]
    fn profile_identity_and_authority_are_explicit() {
        let policy = OrdinaryPublicDnsPolicyV1;
        assert_eq!(policy.profile_id(), ORDINARY_PUBLIC_DNS_POLICY_V1);
        assert_eq!(policy.registry_profile_id(), SPECIAL_USE_REGISTRY_PROFILE_V1);
        assert_eq!(
            policy.authority_scope(),
            DomainPolicyAuthorityScopeV1::ClassificationOnly
        );
    }
}
