#![forbid(unsafe_code)]
//! Zero-network one-shot target-admission composition for public-web acquisition.
//!
//! This crate composes bounded evidence summaries and exact profile/reference
//! identities. Accepting a summary here does **not** prove that a qualified
//! upstream parser, DNS resolver, or policy classifier produced it.

use std::collections::HashSet;
use std::fmt;
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr};

pub const ADMISSION_PROFILE_V2: &str = "mycelix:ordinary-public-target-admission:v2";
pub const LOCATOR_PROFILE_V1: &str = "mycelix:web-url:https-only:v1";
pub const DOMAIN_POLICY_PROFILE_V1: &str = "mycelix:web-domain-policy:ordinary-public-dns:v1";
pub const RESOLVER_PROFILE_V1: &str = "mycelix:synthetic-resolution:v1";
pub const ENDPOINT_POLICY_PROFILE_V1: &str = "mycelix:web-endpoint-policy:ordinary-public-endpoint:v1";
pub const IPV6_ADDRESS_SPACE_PROFILE_V1: &str = "mycelix:iana-ipv6-address-space:2025-10-23:v1";
pub const IPV6_GLOBAL_UNICAST_ENVELOPE_V1: &str = "2000::/3";

pub const MAX_REF_BYTES: usize = 256;
pub const MAX_PROFILE_BYTES: usize = 192;
pub const MAX_ATTEMPT_BYTES: usize = 96;
pub const MAX_DOMAIN_BYTES: usize = 253;
pub const MAX_TARGET_BYTES: usize = 4096;
pub const MAX_CNAME_NAMES: usize = 9;
pub const MAX_ENDPOINTS: usize = 16;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AdmissionAuthorityScopeV2 {
    AdmissionDecisionOnly,
    ConnectorHandoffCandidateOnly,
}

#[derive(Clone, Eq, PartialEq)]
pub struct EvidenceRefV2 {
    profile_id: String,
    subject_ref: String,
}

impl EvidenceRefV2 {
    pub fn new(
        profile_id: impl Into<String>,
        subject_ref: impl Into<String>,
    ) -> Result<Self, AdmissionModelError> {
        let profile_id = profile_id.into();
        let subject_ref = subject_ref.into();
        validate_graphic_ascii("profile id", &profile_id, MAX_PROFILE_BYTES)?;
        validate_graphic_ascii("subject ref", &subject_ref, MAX_REF_BYTES)?;
        Ok(Self {
            profile_id,
            subject_ref,
        })
    }

    pub fn profile_id(&self) -> &str {
        &self.profile_id
    }

    pub fn subject_ref(&self) -> &str {
        &self.subject_ref
    }
}

impl fmt::Debug for EvidenceRefV2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("EvidenceRefV2")
            .field("profile_id", &self.profile_id)
            .field("subject_ref", &"<redacted>")
            .finish()
    }
}

#[derive(Clone, Eq, PartialEq)]
pub struct AttemptIdV2(String);

impl AttemptIdV2 {
    pub fn new(value: impl Into<String>) -> Result<Self, AdmissionModelError> {
        let value = value.into();
        validate_graphic_ascii("attempt id", &value, MAX_ATTEMPT_BYTES)?;
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Debug for AttemptIdV2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("AttemptIdV2").field(&"<redacted>").finish()
    }
}

#[derive(Clone, Eq, PartialEq)]
pub struct ResolutionLineageIdV2(String);

impl ResolutionLineageIdV2 {
    pub fn new(value: impl Into<String>) -> Result<Self, AdmissionModelError> {
        let value = value.into();
        validate_graphic_ascii("resolution lineage id", &value, MAX_REF_BYTES)?;
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Debug for ResolutionLineageIdV2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("ResolutionLineageIdV2")
            .field(&"<redacted>")
            .finish()
    }
}

#[derive(Clone, Eq, PartialEq, Hash)]
pub struct DomainNameKeyV2(String);

impl DomainNameKeyV2 {
    pub fn new(value: impl Into<String>) -> Result<Self, AdmissionModelError> {
        let value = value.into();
        if value.is_empty() {
            return Err(AdmissionModelError::EmptyDomain);
        }
        if !value.is_ascii() {
            return Err(AdmissionModelError::NonAsciiDomain);
        }

        let core = value.strip_suffix('.').unwrap_or(&value);
        if core.is_empty() || core.len() > MAX_DOMAIN_BYTES {
            return Err(AdmissionModelError::InvalidDomainLength {
                bytes: core.len(),
                max: MAX_DOMAIN_BYTES,
            });
        }

        let mut normalized = core.to_owned();
        normalized.make_ascii_lowercase();
        for label in normalized.split('.') {
            if label.is_empty() {
                return Err(AdmissionModelError::EmptyDomainLabel);
            }
            if label.len() > 63 {
                return Err(AdmissionModelError::DomainLabelTooLong { bytes: label.len() });
            }
            if label.starts_with('-') || label.ends_with('-') {
                return Err(AdmissionModelError::InvalidDomainHyphenPlacement);
            }
            if !label
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || byte == b'-')
            {
                return Err(AdmissionModelError::UnsupportedDomainCharacter);
            }
        }

        Ok(Self(normalized))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Debug for DomainNameKeyV2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DomainNameKeyV2")
            .field("bytes", &self.0.len())
            .field("value", &"<redacted>")
            .finish()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OriginSchemeV2 {
    Http,
    Https,
    Other,
}

#[derive(Clone, Eq, PartialEq)]
pub enum OriginHostV2 {
    Domain(DomainNameKeyV2),
    Ipv4(Ipv4Addr),
    Ipv6(Ipv6Addr),
}

impl OriginHostV2 {
    pub fn domain(value: impl Into<String>) -> Result<Self, AdmissionModelError> {
        Ok(Self::Domain(DomainNameKeyV2::new(value)?))
    }

    pub const fn ipv4(value: Ipv4Addr) -> Self {
        Self::Ipv4(value)
    }

    pub const fn ipv6(value: Ipv6Addr) -> Self {
        Self::Ipv6(value)
    }
}

impl fmt::Debug for OriginHostV2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let kind = match self {
            Self::Domain(_) => "domain",
            Self::Ipv4(_) => "ipv4",
            Self::Ipv6(_) => "ipv6",
        };
        f.debug_struct("OriginHostV2")
            .field("kind", &kind)
            .field("value", &"<redacted>")
            .finish()
    }
}

#[derive(Clone, Eq, PartialEq)]
pub struct OriginAuthorityV2 {
    locator_ref: EvidenceRefV2,
    scheme: OriginSchemeV2,
    host: OriginHostV2,
    effective_port: u16,
    target: String,
    userinfo_present: bool,
}

impl OriginAuthorityV2 {
    pub fn new(
        locator_ref: EvidenceRefV2,
        scheme: OriginSchemeV2,
        host: OriginHostV2,
        effective_port: u16,
        target: impl Into<String>,
        userinfo_present: bool,
    ) -> Result<Self, AdmissionModelError> {
        let target = target.into();
        validate_http_origin_form(&target)?;
        if effective_port == 0 {
            return Err(AdmissionModelError::InvalidPort);
        }
        Ok(Self {
            locator_ref,
            scheme,
            host,
            effective_port,
            target,
            userinfo_present,
        })
    }

    pub fn locator_ref(&self) -> &EvidenceRefV2 {
        &self.locator_ref
    }

    pub const fn scheme(&self) -> OriginSchemeV2 {
        self.scheme
    }

    pub fn host(&self) -> &OriginHostV2 {
        &self.host
    }

    pub const fn effective_port(&self) -> u16 {
        self.effective_port
    }

    pub fn target(&self) -> &str {
        &self.target
    }

    pub const fn userinfo_present(&self) -> bool {
        self.userinfo_present
    }
}

impl fmt::Debug for OriginAuthorityV2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("OriginAuthorityV2")
            .field("locator_ref", &self.locator_ref)
            .field("scheme", &self.scheme)
            .field("host", &self.host)
            .field("effective_port", &self.effective_port)
            .field("target", &"<redacted>")
            .field("userinfo_present", &self.userinfo_present)
            .finish()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DomainPolicyStateV2 {
    Eligible,
    Refused,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DomainPolicySummaryV2 {
    evidence: EvidenceRefV2,
    name: DomainNameKeyV2,
    state: DomainPolicyStateV2,
}

impl DomainPolicySummaryV2 {
    pub fn new(
        evidence: EvidenceRefV2,
        name: DomainNameKeyV2,
        state: DomainPolicyStateV2,
    ) -> Self {
        Self {
            evidence,
            name,
            state,
        }
    }

    pub fn evidence(&self) -> &EvidenceRefV2 {
        &self.evidence
    }

    pub fn name(&self) -> &DomainNameKeyV2 {
        &self.name
    }

    pub const fn state(&self) -> DomainPolicyStateV2 {
        self.state
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResolutionStatusV2 {
    Complete,
    NameError,
    NoData,
    Timeout,
    TruncatedWithoutQualifiedRetry,
    PartialFamilyResult,
    Cancelled,
    CnameLoop,
    CnameDepthExceeded,
    AnswerLimitExceeded,
    Indeterminate,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ResolutionSummaryV2 {
    DirectIpLiteral,
    Observed {
        evidence: EvidenceRefV2,
        lineage_id: ResolutionLineageIdV2,
        query_name: DomainNameKeyV2,
        status: ResolutionStatusV2,
        cname_chain: Vec<DomainNameKeyV2>,
        endpoints: Vec<IpAddr>,
    },
}

impl ResolutionSummaryV2 {
    pub fn observed(
        evidence: EvidenceRefV2,
        lineage_id: ResolutionLineageIdV2,
        query_name: DomainNameKeyV2,
        status: ResolutionStatusV2,
        cname_chain: Vec<DomainNameKeyV2>,
        endpoints: Vec<IpAddr>,
    ) -> Result<Self, AdmissionModelError> {
        if cname_chain.len() > MAX_CNAME_NAMES {
            return Err(AdmissionModelError::TooManyCnameNames {
                observed: cname_chain.len(),
                max: MAX_CNAME_NAMES,
            });
        }
        if endpoints.len() > MAX_ENDPOINTS {
            return Err(AdmissionModelError::TooManyEndpoints {
                observed: endpoints.len(),
                max: MAX_ENDPOINTS,
            });
        }
        let unique: HashSet<IpAddr> = endpoints.iter().copied().collect();
        if unique.len() != endpoints.len() {
            return Err(AdmissionModelError::DuplicateEndpoint);
        }
        Ok(Self::Observed {
            evidence,
            lineage_id,
            query_name,
            status,
            cname_chain,
            endpoints,
        })
    }

    pub const fn direct_ip_literal() -> Self {
        Self::DirectIpLiteral
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EndpointPolicyStateV2 {
    Eligible,
    Refused,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EndpointPolicySummaryV2 {
    evidence: EvidenceRefV2,
    address: IpAddr,
    state: EndpointPolicyStateV2,
    ipv6_allocation_evidence: Option<EvidenceRefV2>,
    ipv6_envelope: Option<String>,
}

impl EndpointPolicySummaryV2 {
    pub fn ipv4(
        evidence: EvidenceRefV2,
        address: Ipv4Addr,
        state: EndpointPolicyStateV2,
    ) -> Self {
        Self {
            evidence,
            address: IpAddr::V4(address),
            state,
            ipv6_allocation_evidence: None,
            ipv6_envelope: None,
        }
    }

    pub fn ipv6(
        evidence: EvidenceRefV2,
        address: Ipv6Addr,
        state: EndpointPolicyStateV2,
        allocation_evidence: EvidenceRefV2,
        envelope: impl Into<String>,
    ) -> Self {
        Self {
            evidence,
            address: IpAddr::V6(address),
            state,
            ipv6_allocation_evidence: Some(allocation_evidence),
            ipv6_envelope: Some(envelope.into()),
        }
    }

    pub fn evidence(&self) -> &EvidenceRefV2 {
        &self.evidence
    }

    pub const fn address(&self) -> IpAddr {
        self.address
    }

    pub const fn state(&self) -> EndpointPolicyStateV2 {
        self.state
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TargetAdmissionInputV2 {
    pub origin: OriginAuthorityV2,
    pub initial_domain_policy: Option<DomainPolicySummaryV2>,
    pub cname_domain_policies: Vec<DomainPolicySummaryV2>,
    pub resolution: ResolutionSummaryV2,
    pub endpoint_assessments: Vec<EndpointPolicySummaryV2>,
    pub attempt_id: AttemptIdV2,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AdmissionRefusalV2 {
    LocatorProfileMismatch,
    SchemeNotHttps,
    UserinfoPresent,
    EffectivePortNot443,
    DomainPolicyMissing,
    UnexpectedDomainPolicyForIpLiteral,
    DomainPolicyProfileMismatch,
    InitialDomainNameMismatch,
    InitialDomainRefused,
    ResolutionKindMismatch,
    ResolverProfileMismatch,
    ResolutionQueryNameMismatch,
    ResolutionNotComplete,
    CnameChainDoesNotBeginAtQuery,
    CnamePolicyCountMismatch,
    CnamePolicyProfileMismatch,
    CnameNameMismatch,
    CnameDomainRefused,
    EndpointSetEmpty,
    EndpointAssessmentCountMismatch,
    EndpointAssessmentAddressMismatch,
    EndpointPolicyProfileMismatch,
    EndpointRefused,
    MissingIpv6AllocationBinding,
    Ipv6AllocationProfileMismatch,
    Ipv6EnvelopeMismatch,
}

#[derive(Debug, Eq, PartialEq)]
pub enum AdmissionDecisionV2 {
    Admitted(AdmittedWebTargetV2),
    Refused(AdmissionRefusalV2),
}

impl AdmissionDecisionV2 {
    pub const fn authority_scope(&self) -> AdmissionAuthorityScopeV2 {
        AdmissionAuthorityScopeV2::AdmissionDecisionOnly
    }
}

/// Local, move-only admission capability.
///
/// It intentionally does not implement `Clone`. Converting it into a connector
/// handoff consumes the value. This establishes only local Rust ownership
/// semantics, not global/restart-safe replay protection for `attempt_id`.
#[derive(Eq, PartialEq)]
pub struct AdmittedWebTargetV2 {
    origin: OriginAuthorityV2,
    endpoints: Vec<IpAddr>,
    attempt_id: AttemptIdV2,
    resolution_evidence: Option<EvidenceRefV2>,
    resolution_lineage: Option<ResolutionLineageIdV2>,
}

impl fmt::Debug for AdmittedWebTargetV2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AdmittedWebTargetV2")
            .field("profile", &ADMISSION_PROFILE_V2)
            .field("origin", &self.origin)
            .field("endpoint_count", &self.endpoints.len())
            .field("attempt_id", &self.attempt_id)
            .field("resolution_evidence", &self.resolution_evidence)
            .field("resolution_lineage", &self.resolution_lineage)
            .finish()
    }
}

impl AdmittedWebTargetV2 {
    pub const fn authority_scope(&self) -> AdmissionAuthorityScopeV2 {
        AdmissionAuthorityScopeV2::AdmissionDecisionOnly
    }

    pub fn endpoint_count(&self) -> usize {
        self.endpoints.len()
    }

    /// Consumes the local admission capability and yields one move-only handoff.
    pub fn into_handoff(self) -> ConnectorHandoffV2 {
        let (tls_reference_identity, sni_expectation) = match self.origin.host() {
            OriginHostV2::Domain(name) => (
                TlsReferenceIdentityV2::DnsName(name.clone()),
                SniExpectationV2::DnsName(name.clone()),
            ),
            OriginHostV2::Ipv4(value) => (
                TlsReferenceIdentityV2::IpAddress(IpAddr::V4(*value)),
                SniExpectationV2::OmitForIpLiteral,
            ),
            OriginHostV2::Ipv6(value) => (
                TlsReferenceIdentityV2::IpAddress(IpAddr::V6(*value)),
                SniExpectationV2::OmitForIpLiteral,
            ),
        };

        ConnectorHandoffV2 {
            admission_profile: ADMISSION_PROFILE_V2,
            origin: self.origin,
            endpoints: self.endpoints,
            attempt_id: self.attempt_id,
            resolution_evidence: self.resolution_evidence,
            resolution_lineage: self.resolution_lineage,
            tls_reference_identity,
            sni_expectation,
        }
    }
}

#[derive(Clone, Eq, PartialEq)]
pub enum TlsReferenceIdentityV2 {
    DnsName(DomainNameKeyV2),
    IpAddress(IpAddr),
}

impl fmt::Debug for TlsReferenceIdentityV2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let kind = match self {
            Self::DnsName(_) => "dns-id",
            Self::IpAddress(_) => "ip-id",
        };
        f.debug_struct("TlsReferenceIdentityV2")
            .field("kind", &kind)
            .field("value", &"<redacted>")
            .finish()
    }
}

#[derive(Clone, Eq, PartialEq)]
pub enum SniExpectationV2 {
    DnsName(DomainNameKeyV2),
    OmitForIpLiteral,
}

impl fmt::Debug for SniExpectationV2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DnsName(_) => f.write_str("SniExpectationV2::DnsName(<redacted>)"),
            Self::OmitForIpLiteral => f.write_str("SniExpectationV2::OmitForIpLiteral"),
        }
    }
}

/// Move-only connector input candidate.
///
/// This type intentionally does not implement `Clone`. A future connector must
/// take it by value. The current crate still opens no socket and performs no TLS.
#[derive(Eq, PartialEq)]
pub struct ConnectorHandoffV2 {
    admission_profile: &'static str,
    origin: OriginAuthorityV2,
    endpoints: Vec<IpAddr>,
    attempt_id: AttemptIdV2,
    resolution_evidence: Option<EvidenceRefV2>,
    resolution_lineage: Option<ResolutionLineageIdV2>,
    tls_reference_identity: TlsReferenceIdentityV2,
    sni_expectation: SniExpectationV2,
}

impl ConnectorHandoffV2 {
    pub const fn admission_profile(&self) -> &'static str {
        self.admission_profile
    }

    pub fn origin(&self) -> &OriginAuthorityV2 {
        &self.origin
    }

    pub fn endpoints(&self) -> &[IpAddr] {
        &self.endpoints
    }

    pub fn attempt_id(&self) -> &AttemptIdV2 {
        &self.attempt_id
    }

    pub fn resolution_evidence(&self) -> Option<&EvidenceRefV2> {
        self.resolution_evidence.as_ref()
    }

    pub fn resolution_lineage(&self) -> Option<&ResolutionLineageIdV2> {
        self.resolution_lineage.as_ref()
    }

    pub fn tls_reference_identity(&self) -> &TlsReferenceIdentityV2 {
        &self.tls_reference_identity
    }

    pub fn sni_expectation(&self) -> &SniExpectationV2 {
        &self.sni_expectation
    }

    pub const fn authority_scope(&self) -> AdmissionAuthorityScopeV2 {
        AdmissionAuthorityScopeV2::ConnectorHandoffCandidateOnly
    }
}

impl fmt::Debug for ConnectorHandoffV2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ConnectorHandoffV2")
            .field("admission_profile", &self.admission_profile)
            .field("origin", &self.origin)
            .field("endpoint_count", &self.endpoints.len())
            .field("attempt_id", &self.attempt_id)
            .field("resolution_evidence", &self.resolution_evidence)
            .field("resolution_lineage", &self.resolution_lineage)
            .field("tls_reference_identity", &self.tls_reference_identity)
            .field("sni_expectation", &self.sni_expectation)
            .finish()
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AdmissionModelError {
    EmptyIdentifier { field: &'static str },
    IdentifierTooLong { field: &'static str, bytes: usize, max: usize },
    IdentifierNotGraphicAscii { field: &'static str },
    EmptyDomain,
    NonAsciiDomain,
    InvalidDomainLength { bytes: usize, max: usize },
    EmptyDomainLabel,
    DomainLabelTooLong { bytes: usize },
    InvalidDomainHyphenPlacement,
    UnsupportedDomainCharacter,
    InvalidHttpTarget,
    HttpTargetTooLong { bytes: usize, max: usize },
    HttpTargetNotAsciiGraphic,
    HttpTargetContainsFragment,
    InvalidPort,
    TooManyCnameNames { observed: usize, max: usize },
    TooManyEndpoints { observed: usize, max: usize },
    DuplicateEndpoint,
}

impl fmt::Display for AdmissionModelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyIdentifier { field } => write!(f, "{field} is empty"),
            Self::IdentifierTooLong { field, bytes, max } => {
                write!(f, "{field} is {bytes} bytes; maximum is {max}")
            }
            Self::IdentifierNotGraphicAscii { field } => {
                write!(f, "{field} must contain only graphic ASCII")
            }
            Self::EmptyDomain => f.write_str("domain is empty"),
            Self::NonAsciiDomain => f.write_str("domain is not post-normalization ASCII"),
            Self::InvalidDomainLength { bytes, max } => {
                write!(f, "domain is {bytes} bytes; maximum is {max}")
            }
            Self::EmptyDomainLabel => f.write_str("domain contains an empty label"),
            Self::DomainLabelTooLong { bytes } => {
                write!(f, "domain label is {bytes} bytes; maximum is 63")
            }
            Self::InvalidDomainHyphenPlacement => {
                f.write_str("domain label begins or ends with '-'")
            }
            Self::UnsupportedDomainCharacter => {
                f.write_str("domain contains unsupported ASCII data")
            }
            Self::InvalidHttpTarget => f.write_str("HTTP origin-form target must begin with '/'"),
            Self::HttpTargetTooLong { bytes, max } => {
                write!(f, "HTTP target is {bytes} bytes; maximum is {max}")
            }
            Self::HttpTargetNotAsciiGraphic => {
                f.write_str("HTTP target must contain only graphic ASCII bytes")
            }
            Self::HttpTargetContainsFragment => {
                f.write_str("HTTP request target cannot contain a literal fragment delimiter")
            }
            Self::InvalidPort => f.write_str("effective port must be non-zero"),
            Self::TooManyCnameNames { observed, max } => {
                write!(f, "CNAME chain has {observed} names; maximum is {max}")
            }
            Self::TooManyEndpoints { observed, max } => {
                write!(f, "endpoint set has {observed} addresses; maximum is {max}")
            }
            Self::DuplicateEndpoint => f.write_str("endpoint set contains a duplicate address"),
        }
    }
}

impl std::error::Error for AdmissionModelError {}

pub fn admit_target_v2(input: TargetAdmissionInputV2) -> AdmissionDecisionV2 {
    if input.origin.locator_ref().profile_id() != LOCATOR_PROFILE_V1 {
        return AdmissionDecisionV2::Refused(AdmissionRefusalV2::LocatorProfileMismatch);
    }
    if input.origin.scheme() != OriginSchemeV2::Https {
        return AdmissionDecisionV2::Refused(AdmissionRefusalV2::SchemeNotHttps);
    }
    if input.origin.userinfo_present() {
        return AdmissionDecisionV2::Refused(AdmissionRefusalV2::UserinfoPresent);
    }
    if input.origin.effective_port() != 443 {
        return AdmissionDecisionV2::Refused(AdmissionRefusalV2::EffectivePortNot443);
    }

    match input.origin.host().clone() {
        OriginHostV2::Domain(name) => admit_domain(input, name),
        OriginHostV2::Ipv4(address) => admit_ip(input, IpAddr::V4(address)),
        OriginHostV2::Ipv6(address) => admit_ip(input, IpAddr::V6(address)),
    }
}

fn admit_domain(input: TargetAdmissionInputV2, origin_name: DomainNameKeyV2) -> AdmissionDecisionV2 {
    let Some(initial) = input.initial_domain_policy.as_ref() else {
        return AdmissionDecisionV2::Refused(AdmissionRefusalV2::DomainPolicyMissing);
    };
    if initial.evidence().profile_id() != DOMAIN_POLICY_PROFILE_V1 {
        return AdmissionDecisionV2::Refused(AdmissionRefusalV2::DomainPolicyProfileMismatch);
    }
    if initial.name() != &origin_name {
        return AdmissionDecisionV2::Refused(AdmissionRefusalV2::InitialDomainNameMismatch);
    }
    if initial.state() != DomainPolicyStateV2::Eligible {
        return AdmissionDecisionV2::Refused(AdmissionRefusalV2::InitialDomainRefused);
    }

    let (resolution_evidence, resolution_lineage, cname_chain, endpoints) = match &input.resolution {
        ResolutionSummaryV2::DirectIpLiteral => {
            return AdmissionDecisionV2::Refused(AdmissionRefusalV2::ResolutionKindMismatch);
        }
        ResolutionSummaryV2::Observed {
            evidence,
            lineage_id,
            query_name,
            status,
            cname_chain,
            endpoints,
        } => {
            if evidence.profile_id() != RESOLVER_PROFILE_V1 {
                return AdmissionDecisionV2::Refused(AdmissionRefusalV2::ResolverProfileMismatch);
            }
            if query_name != &origin_name {
                return AdmissionDecisionV2::Refused(AdmissionRefusalV2::ResolutionQueryNameMismatch);
            }
            if *status != ResolutionStatusV2::Complete {
                return AdmissionDecisionV2::Refused(AdmissionRefusalV2::ResolutionNotComplete);
            }
            if !cname_chain.is_empty() && cname_chain.first() != Some(query_name) {
                return AdmissionDecisionV2::Refused(
                    AdmissionRefusalV2::CnameChainDoesNotBeginAtQuery,
                );
            }
            (
                evidence.clone(),
                lineage_id.clone(),
                cname_chain.clone(),
                endpoints.clone(),
            )
        }
    };

    if input.cname_domain_policies.len() != cname_chain.len() {
        return AdmissionDecisionV2::Refused(AdmissionRefusalV2::CnamePolicyCountMismatch);
    }
    for (name, policy) in cname_chain.iter().zip(input.cname_domain_policies.iter()) {
        if policy.evidence().profile_id() != DOMAIN_POLICY_PROFILE_V1 {
            return AdmissionDecisionV2::Refused(AdmissionRefusalV2::CnamePolicyProfileMismatch);
        }
        if policy.name() != name {
            return AdmissionDecisionV2::Refused(AdmissionRefusalV2::CnameNameMismatch);
        }
        if policy.state() != DomainPolicyStateV2::Eligible {
            return AdmissionDecisionV2::Refused(AdmissionRefusalV2::CnameDomainRefused);
        }
    }

    if endpoints.is_empty() {
        return AdmissionDecisionV2::Refused(AdmissionRefusalV2::EndpointSetEmpty);
    }
    if let Err(reason) = validate_endpoints(&endpoints, &input.endpoint_assessments) {
        return AdmissionDecisionV2::Refused(reason);
    }

    AdmissionDecisionV2::Admitted(AdmittedWebTargetV2 {
        origin: input.origin,
        endpoints,
        attempt_id: input.attempt_id,
        resolution_evidence: Some(resolution_evidence),
        resolution_lineage: Some(resolution_lineage),
    })
}

fn admit_ip(input: TargetAdmissionInputV2, address: IpAddr) -> AdmissionDecisionV2 {
    if input.initial_domain_policy.is_some() || !input.cname_domain_policies.is_empty() {
        return AdmissionDecisionV2::Refused(
            AdmissionRefusalV2::UnexpectedDomainPolicyForIpLiteral,
        );
    }
    if !matches!(input.resolution, ResolutionSummaryV2::DirectIpLiteral) {
        return AdmissionDecisionV2::Refused(AdmissionRefusalV2::ResolutionKindMismatch);
    }

    let endpoints = vec![address];
    if let Err(reason) = validate_endpoints(&endpoints, &input.endpoint_assessments) {
        return AdmissionDecisionV2::Refused(reason);
    }

    AdmissionDecisionV2::Admitted(AdmittedWebTargetV2 {
        origin: input.origin,
        endpoints,
        attempt_id: input.attempt_id,
        resolution_evidence: None,
        resolution_lineage: None,
    })
}

fn validate_endpoints(
    endpoints: &[IpAddr],
    assessments: &[EndpointPolicySummaryV2],
) -> Result<(), AdmissionRefusalV2> {
    if assessments.len() != endpoints.len() {
        return Err(AdmissionRefusalV2::EndpointAssessmentCountMismatch);
    }

    let endpoint_set: HashSet<IpAddr> = endpoints.iter().copied().collect();
    let assessment_set: HashSet<IpAddr> = assessments.iter().map(|item| item.address()).collect();
    if endpoint_set.len() != endpoints.len()
        || assessment_set.len() != assessments.len()
        || endpoint_set != assessment_set
    {
        return Err(AdmissionRefusalV2::EndpointAssessmentAddressMismatch);
    }

    for assessment in assessments {
        if assessment.evidence().profile_id() != ENDPOINT_POLICY_PROFILE_V1 {
            return Err(AdmissionRefusalV2::EndpointPolicyProfileMismatch);
        }
        if let IpAddr::V6(_) = assessment.address() {
            let Some(allocation) = assessment.ipv6_allocation_evidence.as_ref() else {
                return Err(AdmissionRefusalV2::MissingIpv6AllocationBinding);
            };
            if allocation.profile_id() != IPV6_ADDRESS_SPACE_PROFILE_V1 {
                return Err(AdmissionRefusalV2::Ipv6AllocationProfileMismatch);
            }
            if assessment.ipv6_envelope.as_deref() != Some(IPV6_GLOBAL_UNICAST_ENVELOPE_V1) {
                return Err(AdmissionRefusalV2::Ipv6EnvelopeMismatch);
            }
        }
        if assessment.state() != EndpointPolicyStateV2::Eligible {
            return Err(AdmissionRefusalV2::EndpointRefused);
        }
    }

    Ok(())
}

fn validate_graphic_ascii(
    field: &'static str,
    value: &str,
    max: usize,
) -> Result<(), AdmissionModelError> {
    if value.is_empty() {
        return Err(AdmissionModelError::EmptyIdentifier { field });
    }
    if value.len() > max {
        return Err(AdmissionModelError::IdentifierTooLong {
            field,
            bytes: value.len(),
            max,
        });
    }
    if !value.bytes().all(|byte| byte.is_ascii_graphic()) {
        return Err(AdmissionModelError::IdentifierNotGraphicAscii { field });
    }
    Ok(())
}

fn validate_http_origin_form(target: &str) -> Result<(), AdmissionModelError> {
    if target.is_empty() || !target.starts_with('/') {
        return Err(AdmissionModelError::InvalidHttpTarget);
    }
    if target.len() > MAX_TARGET_BYTES {
        return Err(AdmissionModelError::HttpTargetTooLong {
            bytes: target.len(),
            max: MAX_TARGET_BYTES,
        });
    }
    if !target.bytes().all(|byte| byte.is_ascii_graphic()) {
        return Err(AdmissionModelError::HttpTargetNotAsciiGraphic);
    }
    if target.as_bytes().contains(&b'#') {
        return Err(AdmissionModelError::HttpTargetContainsFragment);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    const SEED: &str = include_str!(
        "../../../docs/epistemic/fixtures/WEB_TARGET_ADMISSION_TEST_001_V0_2.json"
    );

    fn evidence(profile: &str, subject: &str) -> EvidenceRefV2 {
        EvidenceRefV2::new(profile, subject).expect("evidence")
    }

    fn attempt(value: &str) -> AttemptIdV2 {
        AttemptIdV2::new(value).expect("attempt")
    }

    fn domain_key(value: &str) -> DomainNameKeyV2 {
        DomainNameKeyV2::new(value).expect("domain")
    }

    fn origin_domain(host: &str) -> OriginAuthorityV2 {
        OriginAuthorityV2::new(
            evidence(LOCATOR_PROFILE_V1, "locator:fixture"),
            OriginSchemeV2::Https,
            OriginHostV2::domain(host).expect("host"),
            443,
            "/report?q=secret",
            false,
        )
        .expect("origin")
    }

    fn eligible_domain(name: &str) -> DomainPolicySummaryV2 {
        DomainPolicySummaryV2::new(
            evidence(DOMAIN_POLICY_PROFILE_V1, "domain:fixture"),
            domain_key(name),
            DomainPolicyStateV2::Eligible,
        )
    }

    fn refused_domain(name: &str) -> DomainPolicySummaryV2 {
        DomainPolicySummaryV2::new(
            evidence(DOMAIN_POLICY_PROFILE_V1, "domain:fixture"),
            domain_key(name),
            DomainPolicyStateV2::Refused,
        )
    }

    fn endpoint(address: IpAddr, state: EndpointPolicyStateV2) -> EndpointPolicySummaryV2 {
        match address {
            IpAddr::V4(v4) => EndpointPolicySummaryV2::ipv4(
                evidence(ENDPOINT_POLICY_PROFILE_V1, "endpoint:fixture"),
                v4,
                state,
            ),
            IpAddr::V6(v6) => EndpointPolicySummaryV2::ipv6(
                evidence(ENDPOINT_POLICY_PROFILE_V1, "endpoint:fixture"),
                v6,
                state,
                evidence(IPV6_ADDRESS_SPACE_PROFILE_V1, "alloc:fixture"),
                IPV6_GLOBAL_UNICAST_ENVELOPE_V1,
            ),
        }
    }

    fn complete_resolution(
        query: &str,
        lineage: &str,
        cname_chain: Vec<&str>,
        endpoints: Vec<IpAddr>,
    ) -> ResolutionSummaryV2 {
        ResolutionSummaryV2::observed(
            evidence(RESOLVER_PROFILE_V1, "resolution:fixture"),
            ResolutionLineageIdV2::new(lineage).expect("lineage"),
            domain_key(query),
            ResolutionStatusV2::Complete,
            cname_chain.into_iter().map(domain_key).collect(),
            endpoints,
        )
        .expect("resolution")
    }

    #[test]
    fn corpus_identity_and_case_count_are_frozen() {
        let root: Value = serde_json::from_str(SEED).expect("fixture JSON");
        assert_eq!(
            root["admission_profile"]["id"].as_str(),
            Some(ADMISSION_PROFILE_V2)
        );
        assert_eq!(root["cases"].as_array().expect("cases").len(), 31);
        assert_eq!(
            root["candidate_dependencies"]["web_dns_test_001"]["resolver_profile"].as_str(),
            Some(RESOLVER_PROFILE_V1)
        );
        assert_eq!(
            root["candidate_dependencies"]["web_net_alloc_001"]["ipv6_envelope"].as_str(),
            Some(IPV6_GLOBAL_UNICAST_ENVELOPE_V1)
        );
    }

    #[test]
    fn complete_domain_path_admits_and_yields_move_only_handoff() {
        let address = IpAddr::V4(Ipv4Addr::new(8, 8, 8, 8));
        let input = TargetAdmissionInputV2 {
            origin: origin_domain("www.public-synthetic.invalidtld"),
            initial_domain_policy: Some(eligible_domain("www.public-synthetic.invalidtld")),
            cname_domain_policies: vec![],
            resolution: complete_resolution(
                "www.public-synthetic.invalidtld",
                "R1",
                vec![],
                vec![address],
            ),
            endpoint_assessments: vec![endpoint(address, EndpointPolicyStateV2::Eligible)],
            attempt_id: attempt("A1"),
        };

        let AdmissionDecisionV2::Admitted(admitted) = admit_target_v2(input) else {
            panic!("expected admission");
        };
        assert_eq!(admitted.endpoint_count(), 1);
        let handoff = admitted.into_handoff();
        assert_eq!(handoff.endpoints(), &[address]);
        assert!(matches!(
            handoff.tls_reference_identity(),
            TlsReferenceIdentityV2::DnsName(_)
        ));
        assert!(matches!(handoff.sni_expectation(), SniExpectationV2::DnsName(_)));
        assert_eq!(
            handoff.authority_scope(),
            AdmissionAuthorityScopeV2::ConnectorHandoffCandidateOnly
        );
    }

    #[test]
    fn resolution_query_substitution_refuses() {
        let address = IpAddr::V4(Ipv4Addr::new(8, 8, 8, 8));
        let input = TargetAdmissionInputV2 {
            origin: origin_domain("www.public-synthetic.invalidtld"),
            initial_domain_policy: Some(eligible_domain("www.public-synthetic.invalidtld")),
            cname_domain_policies: vec![],
            resolution: complete_resolution(
                "other.public-synthetic.invalidtld",
                "R2",
                vec![],
                vec![address],
            ),
            endpoint_assessments: vec![endpoint(address, EndpointPolicyStateV2::Eligible)],
            attempt_id: attempt("A2"),
        };
        assert_eq!(
            admit_target_v2(input),
            AdmissionDecisionV2::Refused(AdmissionRefusalV2::ResolutionQueryNameMismatch)
        );
    }

    #[test]
    fn cname_special_use_refuses() {
        let address = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
        let input = TargetAdmissionInputV2 {
            origin: origin_domain("alias.public-synthetic.invalidtld"),
            initial_domain_policy: Some(eligible_domain("alias.public-synthetic.invalidtld")),
            cname_domain_policies: vec![
                eligible_domain("alias.public-synthetic.invalidtld"),
                refused_domain("foo.localhost"),
            ],
            resolution: complete_resolution(
                "alias.public-synthetic.invalidtld",
                "R3",
                vec!["alias.public-synthetic.invalidtld", "foo.localhost"],
                vec![address],
            ),
            endpoint_assessments: vec![endpoint(address, EndpointPolicyStateV2::Refused)],
            attempt_id: attempt("A3"),
        };
        assert_eq!(
            admit_target_v2(input),
            AdmissionDecisionV2::Refused(AdmissionRefusalV2::CnameDomainRefused)
        );
    }

    #[test]
    fn incomplete_resolution_refuses_before_endpoint_state_matters() {
        let address = IpAddr::V4(Ipv4Addr::new(8, 8, 8, 8));
        let resolution = ResolutionSummaryV2::observed(
            evidence(RESOLVER_PROFILE_V1, "resolution:partial"),
            ResolutionLineageIdV2::new("R4").expect("lineage"),
            domain_key("partial.public-synthetic.invalidtld"),
            ResolutionStatusV2::PartialFamilyResult,
            vec![],
            vec![address],
        )
        .expect("resolution");
        let input = TargetAdmissionInputV2 {
            origin: origin_domain("partial.public-synthetic.invalidtld"),
            initial_domain_policy: Some(eligible_domain("partial.public-synthetic.invalidtld")),
            cname_domain_policies: vec![],
            resolution,
            endpoint_assessments: vec![endpoint(address, EndpointPolicyStateV2::Eligible)],
            attempt_id: attempt("A4"),
        };
        assert_eq!(
            admit_target_v2(input),
            AdmissionDecisionV2::Refused(AdmissionRefusalV2::ResolutionNotComplete)
        );
    }

    #[test]
    fn mixed_endpoint_set_refuses_without_filtering() {
        let public = IpAddr::V4(Ipv4Addr::new(8, 8, 8, 8));
        let loopback = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
        let input = TargetAdmissionInputV2 {
            origin: origin_domain("mixed.public-synthetic.invalidtld"),
            initial_domain_policy: Some(eligible_domain("mixed.public-synthetic.invalidtld")),
            cname_domain_policies: vec![],
            resolution: complete_resolution(
                "mixed.public-synthetic.invalidtld",
                "R5",
                vec![],
                vec![public, loopback],
            ),
            endpoint_assessments: vec![
                endpoint(public, EndpointPolicyStateV2::Eligible),
                endpoint(loopback, EndpointPolicyStateV2::Refused),
            ],
            attempt_id: attempt("A5"),
        };
        assert_eq!(
            admit_target_v2(input),
            AdmissionDecisionV2::Refused(AdmissionRefusalV2::EndpointRefused)
        );
    }

    #[test]
    fn direct_ip_uses_ip_id_and_omits_sni() {
        let v4 = Ipv4Addr::new(8, 8, 8, 8);
        let input = TargetAdmissionInputV2 {
            origin: OriginAuthorityV2::new(
                evidence(LOCATOR_PROFILE_V1, "locator:ip"),
                OriginSchemeV2::Https,
                OriginHostV2::ipv4(v4),
                443,
                "/",
                false,
            )
            .expect("origin"),
            initial_domain_policy: None,
            cname_domain_policies: vec![],
            resolution: ResolutionSummaryV2::direct_ip_literal(),
            endpoint_assessments: vec![endpoint(
                IpAddr::V4(v4),
                EndpointPolicyStateV2::Eligible,
            )],
            attempt_id: attempt("A6"),
        };
        let AdmissionDecisionV2::Admitted(admitted) = admit_target_v2(input) else {
            panic!("expected admission");
        };
        let handoff = admitted.into_handoff();
        assert!(matches!(
            handoff.tls_reference_identity(),
            TlsReferenceIdentityV2::IpAddress(IpAddr::V4(value)) if *value == v4
        ));
        assert_eq!(handoff.sni_expectation(), &SniExpectationV2::OmitForIpLiteral);
    }

    #[test]
    fn ipv6_profile_substitution_refuses() {
        let v6: Ipv6Addr = "2001:4860:4860::8888".parse().expect("IPv6");
        let bad = EndpointPolicySummaryV2::ipv6(
            evidence(ENDPOINT_POLICY_PROFILE_V1, "endpoint:v6"),
            v6,
            EndpointPolicyStateV2::Eligible,
            evidence(
                "mycelix:iana-ipv6-address-space:unexpected:v9",
                "alloc:bad",
            ),
            IPV6_GLOBAL_UNICAST_ENVELOPE_V1,
        );
        let input = TargetAdmissionInputV2 {
            origin: origin_domain("v6.public-synthetic.invalidtld"),
            initial_domain_policy: Some(eligible_domain("v6.public-synthetic.invalidtld")),
            cname_domain_policies: vec![],
            resolution: complete_resolution(
                "v6.public-synthetic.invalidtld",
                "R6",
                vec![],
                vec![IpAddr::V6(v6)],
            ),
            endpoint_assessments: vec![bad],
            attempt_id: attempt("A7"),
        };
        assert_eq!(
            admit_target_v2(input),
            AdmissionDecisionV2::Refused(AdmissionRefusalV2::Ipv6AllocationProfileMismatch)
        );
    }

    #[test]
    fn origin_form_rejects_fragment_whitespace_and_non_ascii() {
        let locator = evidence(LOCATOR_PROFILE_V1, "locator:target-test");
        let host = OriginHostV2::domain("www.public-synthetic.invalidtld").expect("host");

        let fragment = OriginAuthorityV2::new(
            locator.clone(),
            OriginSchemeV2::Https,
            host.clone(),
            443,
            "/path#fragment",
            false,
        );
        assert_eq!(fragment, Err(AdmissionModelError::HttpTargetContainsFragment));

        let whitespace = OriginAuthorityV2::new(
            locator.clone(),
            OriginSchemeV2::Https,
            host.clone(),
            443,
            "/bad path",
            false,
        );
        assert_eq!(whitespace, Err(AdmissionModelError::HttpTargetNotAsciiGraphic));

        let unicode = OriginAuthorityV2::new(
            locator,
            OriginSchemeV2::Https,
            host,
            443,
            "/café",
            false,
        );
        assert_eq!(unicode, Err(AdmissionModelError::HttpTargetNotAsciiGraphic));
    }

    #[test]
    fn debug_output_redacts_sensitive_target_values() {
        let address = IpAddr::V4(Ipv4Addr::new(8, 8, 8, 8));
        let input = TargetAdmissionInputV2 {
            origin: origin_domain("sensitive.public-synthetic.invalidtld"),
            initial_domain_policy: Some(eligible_domain("sensitive.public-synthetic.invalidtld")),
            cname_domain_policies: vec![],
            resolution: complete_resolution(
                "sensitive.public-synthetic.invalidtld",
                "secret-lineage",
                vec![],
                vec![address],
            ),
            endpoint_assessments: vec![endpoint(address, EndpointPolicyStateV2::Eligible)],
            attempt_id: attempt("secret-attempt"),
        };
        let AdmissionDecisionV2::Admitted(admitted) = admit_target_v2(input) else {
            panic!("expected admission");
        };
        let rendered = format!("{admitted:?}");
        assert!(!rendered.contains("sensitive.public-synthetic.invalidtld"));
        assert!(!rendered.contains("secret-lineage"));
        assert!(!rendered.contains("secret-attempt"));
        assert!(!rendered.contains("8.8.8.8"));

        let handoff = admitted.into_handoff();
        let rendered = format!("{handoff:?}");
        assert!(!rendered.contains("sensitive.public-synthetic.invalidtld"));
        assert!(!rendered.contains("8.8.8.8"));
    }
}
