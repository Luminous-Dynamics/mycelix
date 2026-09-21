#![forbid(unsafe_code)]
//! Deterministic DNS resolution-observation semantics with zero network authority.
//!
//! The model preserves query identity, CNAME ancestry, A/AAAA outcomes,
//! completeness, and attempt lineage. It performs no DNS, socket, HTTP, TLS,
//! filesystem hosts lookup, search-domain expansion, or endpoint admission.

use std::collections::HashSet;
use std::fmt;
use std::net::{Ipv4Addr, Ipv6Addr};

pub const SYNTHETIC_RESOLVER_PROFILE_V1: &str = "mycelix:synthetic-resolution:v1";
pub const MAX_CNAME_DEPTH_V1: usize = 8;
pub const MAX_ANSWERS_V1: usize = 16;
pub const MAX_DNS_NAME_BYTES_V1: usize = 254;
pub const MAX_LINEAGE_ID_BYTES_V1: usize = 64;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResolutionAuthorityScopeV1 {
    ObservationOnly,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DnssecStatusV1 {
    NotEvaluated,
}

#[derive(Clone, Eq, PartialEq)]
pub struct AbsoluteDnsNameV1 {
    supplied_ascii: String,
    comparison_key: String,
}

impl AbsoluteDnsNameV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, DnsModelError> {
        let supplied_ascii = value.into();
        if supplied_ascii.is_empty() {
            return Err(DnsModelError::EmptyName);
        }
        if !supplied_ascii.is_ascii() {
            return Err(DnsModelError::NonAsciiName);
        }
        if !supplied_ascii.ends_with('.') {
            return Err(DnsModelError::NameNotAbsolute);
        }
        if supplied_ascii.len() > MAX_DNS_NAME_BYTES_V1 {
            return Err(DnsModelError::NameTooLong {
                bytes: supplied_ascii.len(),
                max: MAX_DNS_NAME_BYTES_V1,
            });
        }

        let core = supplied_ascii
            .strip_suffix('.')
            .ok_or(DnsModelError::NameNotAbsolute)?;
        if core.is_empty() {
            return Err(DnsModelError::EmptyName);
        }

        let mut comparison_key = core.to_owned();
        comparison_key.make_ascii_lowercase();
        for label in comparison_key.split('.') {
            if label.is_empty() {
                return Err(DnsModelError::EmptyLabel);
            }
            if label.len() > 63 {
                return Err(DnsModelError::LabelTooLong { bytes: label.len() });
            }
            if !label
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || byte == b'-')
            {
                return Err(DnsModelError::UnsupportedAsciiLabelCharacter);
            }
        }

        Ok(Self {
            supplied_ascii,
            comparison_key,
        })
    }

    pub fn supplied_ascii(&self) -> &str {
        &self.supplied_ascii
    }

    pub fn comparison_key(&self) -> &str {
        &self.comparison_key
    }
}

impl fmt::Debug for AbsoluteDnsNameV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AbsoluteDnsNameV1")
            .field("bytes", &self.supplied_ascii.len())
            .field("value", &"<redacted>")
            .finish()
    }
}

#[derive(Clone, Eq, PartialEq)]
pub struct ResolutionLineageIdV1(String);

impl ResolutionLineageIdV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, DnsModelError> {
        let value = value.into();
        if value.is_empty() {
            return Err(DnsModelError::EmptyLineageId);
        }
        if value.len() > MAX_LINEAGE_ID_BYTES_V1 {
            return Err(DnsModelError::LineageIdTooLong {
                bytes: value.len(),
                max: MAX_LINEAGE_ID_BYTES_V1,
            });
        }
        if !value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_' | b':' | b'.'))
        {
            return Err(DnsModelError::UnsupportedLineageCharacter);
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Debug for ResolutionLineageIdV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("ResolutionLineageIdV1").field(&self.0).finish()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DnsFamilyOutcomeV1 {
    Answer,
    NoData,
    NameError,
    Timeout,
    TruncatedWithoutQualifiedRetry,
    Cancelled,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResolutionOutcomeV1 {
    Complete,
    NameError,
    NoData,
    Timeout,
    TruncatedWithoutQualifiedRetry,
    PartialFamilyResult,
    Cancelled,
    Indeterminate,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ResolutionFailureV1 {
    CnameLoop,
    CnameDepthExceeded { observed: usize, max: usize },
    AnswerLimitExceeded { observed: usize, max: usize },
    CnameChainDoesNotBeginAtQuery,
    AnswerStateWithoutAnswers { family: &'static str },
    NonAnswerStateWithAnswers { family: &'static str },
}

impl fmt::Display for ResolutionFailureV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::CnameLoop => f.write_str("synthetic CNAME chain contains a loop"),
            Self::CnameDepthExceeded { observed, max } => write!(
                f,
                "synthetic CNAME depth {observed} exceeds maximum {max}"
            ),
            Self::AnswerLimitExceeded { observed, max } => write!(
                f,
                "synthetic answer count {observed} exceeds maximum {max}"
            ),
            Self::CnameChainDoesNotBeginAtQuery => {
                f.write_str("CNAME ancestry does not begin at the exact query name")
            }
            Self::AnswerStateWithoutAnswers { family } => {
                write!(f, "{family} outcome is Answer but the answer set is empty")
            }
            Self::NonAnswerStateWithAnswers { family } => {
                write!(f, "{family} outcome is non-Answer but answer values are present")
            }
        }
    }
}

impl std::error::Error for ResolutionFailureV1 {}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DnsModelError {
    EmptyName,
    NonAsciiName,
    NameNotAbsolute,
    NameTooLong { bytes: usize, max: usize },
    EmptyLabel,
    LabelTooLong { bytes: usize },
    UnsupportedAsciiLabelCharacter,
    EmptyLineageId,
    LineageIdTooLong { bytes: usize, max: usize },
    UnsupportedLineageCharacter,
}

impl fmt::Display for DnsModelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyName => f.write_str("DNS name is empty"),
            Self::NonAsciiName => f.write_str("DNS name is not ASCII post-normalization data"),
            Self::NameNotAbsolute => f.write_str("DNS name is not absolute (missing trailing dot)"),
            Self::NameTooLong { bytes, max } => {
                write!(f, "DNS name is {bytes} bytes; maximum is {max}")
            }
            Self::EmptyLabel => f.write_str("DNS name contains an empty label"),
            Self::LabelTooLong { bytes } => write!(f, "DNS label is {bytes} bytes; maximum is 63"),
            Self::UnsupportedAsciiLabelCharacter => {
                f.write_str("DNS name contains an unsupported ASCII label character")
            }
            Self::EmptyLineageId => f.write_str("resolution lineage identifier is empty"),
            Self::LineageIdTooLong { bytes, max } => write!(
                f,
                "resolution lineage identifier is {bytes} bytes; maximum is {max}"
            ),
            Self::UnsupportedLineageCharacter => {
                f.write_str("resolution lineage identifier contains an unsupported character")
            }
        }
    }
}

impl std::error::Error for DnsModelError {}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SyntheticResolutionInputV1 {
    pub query_name: AbsoluteDnsNameV1,
    pub lineage_id: ResolutionLineageIdV1,
    pub cname_chain: Vec<AbsoluteDnsNameV1>,
    pub ipv4_answers: Vec<Ipv4Addr>,
    pub ipv6_answers: Vec<Ipv6Addr>,
    pub a_outcome: DnsFamilyOutcomeV1,
    pub aaaa_outcome: DnsFamilyOutcomeV1,
    pub observed_ttl_seconds: Vec<u32>,
}

#[derive(Clone, Eq, PartialEq)]
pub struct ResolutionObservationV1 {
    query_name: AbsoluteDnsNameV1,
    lineage_id: ResolutionLineageIdV1,
    cname_chain: Vec<AbsoluteDnsNameV1>,
    ipv4_answers: Vec<Ipv4Addr>,
    ipv6_answers: Vec<Ipv6Addr>,
    a_outcome: DnsFamilyOutcomeV1,
    aaaa_outcome: DnsFamilyOutcomeV1,
    outcome: ResolutionOutcomeV1,
    observed_ttl_seconds: Vec<u32>,
}

impl ResolutionObservationV1 {
    pub const fn resolver_profile_id(&self) -> &'static str {
        SYNTHETIC_RESOLVER_PROFILE_V1
    }

    pub const fn authority_scope(&self) -> ResolutionAuthorityScopeV1 {
        ResolutionAuthorityScopeV1::ObservationOnly
    }

    pub const fn dnssec_status(&self) -> DnssecStatusV1 {
        DnssecStatusV1::NotEvaluated
    }

    pub const fn network_io_performed(&self) -> bool {
        false
    }

    pub fn query_name(&self) -> &AbsoluteDnsNameV1 {
        &self.query_name
    }

    pub fn lineage_id(&self) -> &ResolutionLineageIdV1 {
        &self.lineage_id
    }

    pub fn cname_chain(&self) -> &[AbsoluteDnsNameV1] {
        &self.cname_chain
    }

    pub fn ipv4_answers(&self) -> &[Ipv4Addr] {
        &self.ipv4_answers
    }

    pub fn ipv6_answers(&self) -> &[Ipv6Addr] {
        &self.ipv6_answers
    }

    pub const fn a_outcome(&self) -> DnsFamilyOutcomeV1 {
        self.a_outcome
    }

    pub const fn aaaa_outcome(&self) -> DnsFamilyOutcomeV1 {
        self.aaaa_outcome
    }

    pub const fn outcome(&self) -> ResolutionOutcomeV1 {
        self.outcome
    }

    pub fn observed_ttl_seconds(&self) -> &[u32] {
        &self.observed_ttl_seconds
    }
}

impl fmt::Debug for ResolutionObservationV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ResolutionObservationV1")
            .field("query_name", &self.query_name)
            .field("lineage_id", &self.lineage_id)
            .field("cname_hops", &self.cname_chain.len().saturating_sub(1))
            .field("ipv4_answer_count", &self.ipv4_answers.len())
            .field("ipv6_answer_count", &self.ipv6_answers.len())
            .field("a_outcome", &self.a_outcome)
            .field("aaaa_outcome", &self.aaaa_outcome)
            .field("outcome", &self.outcome)
            .field("authority", &ResolutionAuthorityScopeV1::ObservationOnly)
            .finish()
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct SyntheticResolverV1;

impl SyntheticResolverV1 {
    pub const fn profile_id(self) -> &'static str {
        SYNTHETIC_RESOLVER_PROFILE_V1
    }

    pub const fn authority_scope(self) -> ResolutionAuthorityScopeV1 {
        ResolutionAuthorityScopeV1::ObservationOnly
    }

    pub const fn network_io_enabled(self) -> bool {
        false
    }

    pub fn observe(
        self,
        mut input: SyntheticResolutionInputV1,
    ) -> Result<ResolutionObservationV1, ResolutionFailureV1> {
        if input.cname_chain.is_empty() {
            input.cname_chain.push(input.query_name.clone());
        }
        if input.cname_chain[0].comparison_key() != input.query_name.comparison_key() {
            return Err(ResolutionFailureV1::CnameChainDoesNotBeginAtQuery);
        }

        let cname_depth = input.cname_chain.len().saturating_sub(1);
        if cname_depth > MAX_CNAME_DEPTH_V1 {
            return Err(ResolutionFailureV1::CnameDepthExceeded {
                observed: cname_depth,
                max: MAX_CNAME_DEPTH_V1,
            });
        }

        let mut seen = HashSet::with_capacity(input.cname_chain.len());
        for name in &input.cname_chain {
            if !seen.insert(name.comparison_key()) {
                return Err(ResolutionFailureV1::CnameLoop);
            }
        }

        let answer_count = input.ipv4_answers.len() + input.ipv6_answers.len();
        if answer_count > MAX_ANSWERS_V1 {
            return Err(ResolutionFailureV1::AnswerLimitExceeded {
                observed: answer_count,
                max: MAX_ANSWERS_V1,
            });
        }

        validate_family("A", input.a_outcome, input.ipv4_answers.len())?;
        validate_family("AAAA", input.aaaa_outcome, input.ipv6_answers.len())?;

        let outcome = derive_outcome(input.a_outcome, input.aaaa_outcome);
        Ok(ResolutionObservationV1 {
            query_name: input.query_name,
            lineage_id: input.lineage_id,
            cname_chain: input.cname_chain,
            ipv4_answers: input.ipv4_answers,
            ipv6_answers: input.ipv6_answers,
            a_outcome: input.a_outcome,
            aaaa_outcome: input.aaaa_outcome,
            outcome,
            observed_ttl_seconds: input.observed_ttl_seconds,
        })
    }
}

fn validate_family(
    family: &'static str,
    outcome: DnsFamilyOutcomeV1,
    answer_count: usize,
) -> Result<(), ResolutionFailureV1> {
    match (outcome, answer_count) {
        (DnsFamilyOutcomeV1::Answer, 0) => {
            Err(ResolutionFailureV1::AnswerStateWithoutAnswers { family })
        }
        (DnsFamilyOutcomeV1::Answer, _) => Ok(()),
        (_, 0) => Ok(()),
        (_, _) => Err(ResolutionFailureV1::NonAnswerStateWithAnswers { family }),
    }
}

fn derive_outcome(a: DnsFamilyOutcomeV1, aaaa: DnsFamilyOutcomeV1) -> ResolutionOutcomeV1 {
    use DnsFamilyOutcomeV1 as F;
    use ResolutionOutcomeV1 as R;

    match (a, aaaa) {
        (F::NameError, F::NameError) => R::NameError,
        (F::NoData, F::NoData) => R::NoData,
        (F::Timeout, F::Timeout) => R::Timeout,
        (F::TruncatedWithoutQualifiedRetry, F::TruncatedWithoutQualifiedRetry) => {
            R::TruncatedWithoutQualifiedRetry
        }
        (F::Cancelled, F::Cancelled) => R::Cancelled,
        (F::Answer, F::Answer | F::NoData) | (F::NoData, F::Answer) => R::Complete,
        (F::Answer, _) | (_, F::Answer) => R::PartialFamilyResult,
        _ => R::Indeterminate,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;
    use std::collections::HashSet;

    const SEED: &str = include_str!(
        "../../../docs/epistemic/fixtures/WEB_DNS_TEST_001_SYNTHETIC_V0_1.json"
    );

    fn name(value: &str) -> AbsoluteDnsNameV1 {
        AbsoluteDnsNameV1::new(value).expect("valid synthetic absolute DNS name")
    }

    fn lineage(value: &str) -> ResolutionLineageIdV1 {
        ResolutionLineageIdV1::new(value).expect("valid lineage")
    }

    fn input(
        query: &str,
        lineage_id: &str,
        ipv4_answers: Vec<Ipv4Addr>,
        ipv6_answers: Vec<Ipv6Addr>,
        a_outcome: DnsFamilyOutcomeV1,
        aaaa_outcome: DnsFamilyOutcomeV1,
    ) -> SyntheticResolutionInputV1 {
        SyntheticResolutionInputV1 {
            query_name: name(query),
            lineage_id: lineage(lineage_id),
            cname_chain: Vec::new(),
            ipv4_answers,
            ipv6_answers,
            a_outcome,
            aaaa_outcome,
            observed_ttl_seconds: Vec::new(),
        }
    }

    #[test]
    fn fixture_profile_and_case_set_are_exact() {
        let root: Value = serde_json::from_str(SEED).expect("fixture JSON");
        assert_eq!(root["resolver_profile"]["id"].as_str(), Some(SYNTHETIC_RESOLVER_PROFILE_V1));
        assert_eq!(root["resolver_profile"]["max_cname_depth"].as_u64(), Some(MAX_CNAME_DEPTH_V1 as u64));
        assert_eq!(root["resolver_profile"]["max_answers"].as_u64(), Some(MAX_ANSWERS_V1 as u64));
        assert_eq!(root["resolver_profile"]["network_io"].as_bool(), Some(false));

        let expected: HashSet<&str> = [
            "a-only-safe", "aaaa-only-safe", "dual-safe", "mixed-safe-loopback",
            "mixed-safe-ula", "cname-safe", "cname-special-use-target", "cname-loop",
            "cname-depth", "nxdomain", "nodata", "timeout", "truncated", "answer-limit",
            "partial-family", "search-disabled", "hosts-file-ignored", "rebinding-r1",
            "rebinding-r2", "cancelled",
        ]
        .into_iter()
        .collect();
        let actual: HashSet<&str> = root["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .map(|case| case["id"].as_str().expect("case id"))
            .collect();
        assert_eq!(actual, expected);
    }

    #[test]
    fn complete_answer_sets_are_preserved_without_filtering() {
        let resolver = SyntheticResolverV1;
        let mixed4 = resolver
            .observe(input(
                "mixed.public-synthetic.invalidtld.",
                "mixed-v4",
                vec![Ipv4Addr::new(8, 8, 8, 8), Ipv4Addr::new(127, 0, 0, 1)],
                vec![],
                DnsFamilyOutcomeV1::Answer,
                DnsFamilyOutcomeV1::NoData,
            ))
            .expect("observation");
        assert_eq!(mixed4.ipv4_answers().len(), 2);
        assert!(mixed4.ipv4_answers().contains(&Ipv4Addr::new(127, 0, 0, 1)));
        assert_eq!(mixed4.outcome(), ResolutionOutcomeV1::Complete);

        let dual = resolver
            .observe(input(
                "dual.public-synthetic.invalidtld.",
                "dual",
                vec![Ipv4Addr::new(8, 8, 8, 8)],
                vec!["2001:4860:4860::8888".parse().expect("IPv6")],
                DnsFamilyOutcomeV1::Answer,
                DnsFamilyOutcomeV1::Answer,
            ))
            .expect("observation");
        assert_eq!(dual.ipv4_answers().len(), 1);
        assert_eq!(dual.ipv6_answers().len(), 1);
        assert_eq!(dual.outcome(), ResolutionOutcomeV1::Complete);
    }

    #[test]
    fn a_only_and_aaaa_only_are_complete_when_other_family_is_explicit_nodata() {
        let resolver = SyntheticResolverV1;
        let a_only = resolver
            .observe(input(
                "www.public-synthetic.invalidtld.",
                "a-only",
                vec![Ipv4Addr::new(8, 8, 8, 8)],
                vec![],
                DnsFamilyOutcomeV1::Answer,
                DnsFamilyOutcomeV1::NoData,
            ))
            .expect("observation");
        assert_eq!(a_only.outcome(), ResolutionOutcomeV1::Complete);

        let aaaa_only = resolver
            .observe(input(
                "v6.public-synthetic.invalidtld.",
                "aaaa-only",
                vec![],
                vec!["2001:4860:4860::8888".parse().expect("IPv6")],
                DnsFamilyOutcomeV1::NoData,
                DnsFamilyOutcomeV1::Answer,
            ))
            .expect("observation");
        assert_eq!(aaaa_only.outcome(), ResolutionOutcomeV1::Complete);
    }

    #[test]
    fn cname_ancestry_is_preserved_but_not_policy_admitted() {
        let resolver = SyntheticResolverV1;
        let mut safe = input(
            "alias.public-synthetic.invalidtld.",
            "cname-safe",
            vec![Ipv4Addr::new(8, 8, 8, 8)],
            vec![],
            DnsFamilyOutcomeV1::Answer,
            DnsFamilyOutcomeV1::NoData,
        );
        safe.cname_chain = vec![
            name("alias.public-synthetic.invalidtld."),
            name("origin.public-synthetic.invalidtld."),
        ];
        let observed = resolver.observe(safe).expect("CNAME observation");
        assert_eq!(observed.cname_chain().len(), 2);

        let mut special_target = input(
            "alias2.public-synthetic.invalidtld.",
            "cname-special",
            vec![Ipv4Addr::new(127, 0, 0, 1)],
            vec![],
            DnsFamilyOutcomeV1::Answer,
            DnsFamilyOutcomeV1::NoData,
        );
        special_target.cname_chain = vec![
            name("alias2.public-synthetic.invalidtld."),
            name("foo.localhost."),
        ];
        let observed = resolver.observe(special_target).expect("DNS may observe special-use target");
        assert_eq!(observed.cname_chain()[1].supplied_ascii(), "foo.localhost.");
        assert_eq!(observed.authority_scope(), ResolutionAuthorityScopeV1::ObservationOnly);
    }

    #[test]
    fn cname_loop_depth_and_answer_bounds_fail_closed() {
        let resolver = SyntheticResolverV1;
        let mut looping = input(
            "loop.public-synthetic.invalidtld.",
            "loop",
            vec![],
            vec![],
            DnsFamilyOutcomeV1::NoData,
            DnsFamilyOutcomeV1::NoData,
        );
        looping.cname_chain = vec![
            name("loop.public-synthetic.invalidtld."),
            name("loop2.public-synthetic.invalidtld."),
            name("loop.public-synthetic.invalidtld."),
        ];
        assert_eq!(resolver.observe(looping), Err(ResolutionFailureV1::CnameLoop));

        let mut deep = input(
            "depth.public-synthetic.invalidtld.",
            "depth",
            vec![],
            vec![],
            DnsFamilyOutcomeV1::NoData,
            DnsFamilyOutcomeV1::NoData,
        );
        deep.cname_chain = (0..=9)
            .map(|index| name(&format!("n{index}.public-synthetic.invalidtld.")))
            .collect();
        deep.cname_chain[0] = name("depth.public-synthetic.invalidtld.");
        assert_eq!(
            resolver.observe(deep),
            Err(ResolutionFailureV1::CnameDepthExceeded { observed: 9, max: 8 })
        );

        let too_many = input(
            "large.public-synthetic.invalidtld.",
            "large",
            vec![Ipv4Addr::new(8, 8, 8, 8); 17],
            vec![],
            DnsFamilyOutcomeV1::Answer,
            DnsFamilyOutcomeV1::NoData,
        );
        assert_eq!(
            resolver.observe(too_many),
            Err(ResolutionFailureV1::AnswerLimitExceeded { observed: 17, max: 16 })
        );
    }

    #[test]
    fn nxdomain_nodata_timeout_truncation_and_cancelled_remain_distinct() {
        let resolver = SyntheticResolverV1;
        let states = [
            (DnsFamilyOutcomeV1::NameError, ResolutionOutcomeV1::NameError),
            (DnsFamilyOutcomeV1::NoData, ResolutionOutcomeV1::NoData),
            (DnsFamilyOutcomeV1::Timeout, ResolutionOutcomeV1::Timeout),
            (
                DnsFamilyOutcomeV1::TruncatedWithoutQualifiedRetry,
                ResolutionOutcomeV1::TruncatedWithoutQualifiedRetry,
            ),
            (DnsFamilyOutcomeV1::Cancelled, ResolutionOutcomeV1::Cancelled),
        ];
        for (index, (family, expected)) in states.into_iter().enumerate() {
            let observation = resolver
                .observe(input(
                    "state.public-synthetic.invalidtld.",
                    &format!("state-{index}"),
                    vec![],
                    vec![],
                    family,
                    family,
                ))
                .expect("typed negative observation");
            assert_eq!(observation.outcome(), expected);
        }
    }

    #[test]
    fn partial_family_result_never_becomes_complete() {
        let observation = SyntheticResolverV1
            .observe(input(
                "partial.public-synthetic.invalidtld.",
                "partial",
                vec![Ipv4Addr::new(8, 8, 8, 8)],
                vec![],
                DnsFamilyOutcomeV1::Answer,
                DnsFamilyOutcomeV1::Timeout,
            ))
            .expect("partial observation");
        assert_eq!(observation.a_outcome(), DnsFamilyOutcomeV1::Answer);
        assert_eq!(observation.aaaa_outcome(), DnsFamilyOutcomeV1::Timeout);
        assert_eq!(observation.outcome(), ResolutionOutcomeV1::PartialFamilyResult);
    }

    #[test]
    fn single_label_search_expansion_is_impossible_at_this_boundary() {
        assert_eq!(AbsoluteDnsNameV1::new("service"), Err(DnsModelError::NameNotAbsolute));
        assert!(AbsoluteDnsNameV1::new("service.corp.example.").is_ok());
    }

    #[test]
    fn hosts_file_has_no_input_surface_and_dnssec_is_not_established() {
        let observation = SyntheticResolverV1
            .observe(input(
                "override.public-synthetic.invalidtld.",
                "hosts-ignored",
                vec![Ipv4Addr::new(8, 8, 8, 8)],
                vec![],
                DnsFamilyOutcomeV1::Answer,
                DnsFamilyOutcomeV1::NoData,
            ))
            .expect("synthetic observation");
        assert_eq!(observation.ipv4_answers(), &[Ipv4Addr::new(8, 8, 8, 8)]);
        assert_eq!(observation.dnssec_status(), DnssecStatusV1::NotEvaluated);
        assert!(!observation.network_io_performed());
    }

    #[test]
    fn rebinding_attempts_are_distinct_lineages() {
        let resolver = SyntheticResolverV1;
        let r1 = resolver
            .observe(input(
                "rebind.public-synthetic.invalidtld.",
                "r1",
                vec![Ipv4Addr::new(8, 8, 8, 8)],
                vec![],
                DnsFamilyOutcomeV1::Answer,
                DnsFamilyOutcomeV1::NoData,
            ))
            .expect("R1");
        let r2 = resolver
            .observe(input(
                "rebind.public-synthetic.invalidtld.",
                "r2",
                vec![Ipv4Addr::new(127, 0, 0, 1)],
                vec![],
                DnsFamilyOutcomeV1::Answer,
                DnsFamilyOutcomeV1::NoData,
            ))
            .expect("R2");
        assert_ne!(r1.lineage_id(), r2.lineage_id());
        assert_ne!(r1.ipv4_answers(), r2.ipv4_answers());
    }

    #[test]
    fn ttl_observation_carries_no_authority_upgrade_and_debug_redacts_query() {
        let mut request = input(
            "private-investigation-target.public-synthetic.invalidtld.",
            "ttl",
            vec![Ipv4Addr::new(8, 8, 8, 8)],
            vec![],
            DnsFamilyOutcomeV1::Answer,
            DnsFamilyOutcomeV1::NoData,
        );
        request.observed_ttl_seconds = vec![60];
        let observation = SyntheticResolverV1.observe(request).expect("observation");
        assert_eq!(observation.observed_ttl_seconds(), &[60]);
        assert_eq!(observation.authority_scope(), ResolutionAuthorityScopeV1::ObservationOnly);
        let rendered = format!("{observation:?}");
        assert!(!rendered.contains("private-investigation-target"));
    }
}
