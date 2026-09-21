#![forbid(unsafe_code)]
//! Canonical audit receipts for V2 public-web target admission.
//!
//! Audit evidence only: no DNS, sockets, TLS, HTTP, browser, Holochain,
//! EPI, Xenia, or connector authority.
//!
//! Accepting receipt semantics here does not prove that a qualified B0
//! admission implementation produced them. A later interop/receipt-binding
//! tranche must establish that provenance.

use sha2::{Digest, Sha256};
use std::collections::HashSet;
use std::fmt;
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr};

pub const RECEIPT_SCHEMA_V1: &str = "mycelix:web-target-admission-receipt:v1";
pub const ADMISSION_PROFILE_V2: &str = "mycelix:ordinary-public-target-admission:v2";
pub const COMMITMENT_PROFILE_SHA256_V1: &str =
    "mycelix:web-target-admission-commitment:sha256:v1";
pub const AUDIT_AUTHORITY_SCOPE_V1: &str = "audit-evidence-only";

pub const B0_PRODUCT_HEAD: &str = "d61ebdd2097dd1986c27c4bf6b7311bb01f91a74";
pub const ADMISSION_CORPUS_HEAD: &str = "53ee3f818984fb110ef0ab697956f98fb4fbacc7";

pub const LOCATOR_PROFILE_V1: &str = "mycelix:web-url:https-only:v1";
pub const DOMAIN_POLICY_PROFILE_V1: &str =
    "mycelix:web-domain-policy:ordinary-public-dns:v1";
pub const RESOLVER_PROFILE_V1: &str = "mycelix:synthetic-resolution:v1";
pub const ENDPOINT_POLICY_PROFILE_V1: &str =
    "mycelix:web-endpoint-policy:ordinary-public-endpoint:v1";
pub const IPV6_ADDRESS_SPACE_PROFILE_V1: &str =
    "mycelix:iana-ipv6-address-space:2025-10-23:v1";
pub const IPV6_GLOBAL_UNICAST_ENVELOPE_V1: &str = "2000::/3";

const TRANSCRIPT_PREFIX: &[u8] = b"mycelix-web-target-admission-receipt-v1\0";
const COMMITMENT_PREFIX: &[u8] =
    b"mycelix-web-target-admission-commitment-sha256-v1\0";
const TRANSCRIPT_VERSION: u16 = 1;

pub const MAX_PROFILE_BYTES: usize = 192;
pub const MAX_REF_BYTES: usize = 256;
pub const MAX_ATTEMPT_BYTES: usize = 96;
pub const MAX_LINEAGE_BYTES: usize = 256;
pub const MAX_DOMAIN_BYTES: usize = 253;
pub const MAX_TARGET_BYTES: usize = 4096;
pub const MAX_CNAME_NAMES: usize = 9;
pub const MAX_ENDPOINTS: usize = 16;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AdmissionReceiptAuthorityScopeV1 {
    AuditEvidenceOnly,
}

#[derive(Clone, Eq, PartialEq)]
pub struct ReceiptEvidenceRefV1 {
    profile_id: String,
    subject_ref: String,
}

impl ReceiptEvidenceRefV1 {
    pub fn new(
        profile_id: impl Into<String>,
        subject_ref: impl Into<String>,
    ) -> Result<Self, ReceiptError> {
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

impl fmt::Debug for ReceiptEvidenceRefV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ReceiptEvidenceRefV1")
            .field("profile_id", &self.profile_id)
            .field("subject_ref", &"<redacted>")
            .finish()
    }
}

#[derive(Clone, Eq, PartialEq, Hash)]
pub struct ReceiptDomainNameV1(String);

impl ReceiptDomainNameV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, ReceiptError> {
        let value = value.into();
        if value.is_empty() || value.len() > MAX_DOMAIN_BYTES {
            return Err(ReceiptError::InvalidDomainLength);
        }
        if !value.is_ascii() {
            return Err(ReceiptError::NonAsciiDomain);
        }
        if value.ends_with('.') {
            return Err(ReceiptError::NonCanonicalDomain);
        }
        if value.bytes().any(|b| b.is_ascii_uppercase()) {
            return Err(ReceiptError::NonCanonicalDomain);
        }
        for label in value.split('.') {
            if label.is_empty() || label.len() > 63 {
                return Err(ReceiptError::InvalidDomainLabel);
            }
            if label.starts_with('-') || label.ends_with('-') {
                return Err(ReceiptError::InvalidDomainLabel);
            }
            if !label
                .bytes()
                .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'-')
            {
                return Err(ReceiptError::InvalidDomainLabel);
            }
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Debug for ReceiptDomainNameV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ReceiptDomainNameV1")
            .field("bytes", &self.0.len())
            .field("value", &"<redacted>")
            .finish()
    }
}

#[derive(Clone, Eq, PartialEq)]
pub enum ReceiptHostV1 {
    Domain(ReceiptDomainNameV1),
    Ipv4(Ipv4Addr),
    Ipv6(Ipv6Addr),
}

impl fmt::Debug for ReceiptHostV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let kind = match self {
            Self::Domain(_) => "domain",
            Self::Ipv4(_) => "ipv4",
            Self::Ipv6(_) => "ipv6",
        };
        f.debug_struct("ReceiptHostV1")
            .field("kind", &kind)
            .field("value", &"<redacted>")
            .finish()
    }
}

#[derive(Clone, Eq, PartialEq)]
pub struct ReceiptOriginV1 {
    locator_ref: ReceiptEvidenceRefV1,
    host: ReceiptHostV1,
    port: u16,
    target: String,
    userinfo_present: bool,
}

impl ReceiptOriginV1 {
    pub fn new(
        locator_ref: ReceiptEvidenceRefV1,
        host: ReceiptHostV1,
        port: u16,
        target: impl Into<String>,
        userinfo_present: bool,
    ) -> Result<Self, ReceiptError> {
        let target = target.into();
        validate_http_origin_form(&target)?;
        Ok(Self {
            locator_ref,
            host,
            port,
            target,
            userinfo_present,
        })
    }

    pub fn locator_ref(&self) -> &ReceiptEvidenceRefV1 {
        &self.locator_ref
    }

    pub fn host(&self) -> &ReceiptHostV1 {
        &self.host
    }

    pub const fn port(&self) -> u16 {
        self.port
    }

    pub fn target(&self) -> &str {
        &self.target
    }

    pub const fn userinfo_present(&self) -> bool {
        self.userinfo_present
    }
}

impl fmt::Debug for ReceiptOriginV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ReceiptOriginV1")
            .field("locator_ref", &self.locator_ref)
            .field("host", &self.host)
            .field("port", &self.port)
            .field("target", &"<redacted>")
            .field("userinfo_present", &self.userinfo_present)
            .finish()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReceiptPolicyStateV1 {
    Eligible,
    Refused,
}

impl ReceiptPolicyStateV1 {
    const fn code(self) -> u8 {
        match self {
            Self::Eligible => 0x01,
            Self::Refused => 0x02,
        }
    }
}

#[derive(Clone, Eq, PartialEq)]
pub struct ReceiptDomainPolicyV1 {
    evidence: ReceiptEvidenceRefV1,
    name: ReceiptDomainNameV1,
    state: ReceiptPolicyStateV1,
}

impl ReceiptDomainPolicyV1 {
    pub fn new(
        evidence: ReceiptEvidenceRefV1,
        name: ReceiptDomainNameV1,
        state: ReceiptPolicyStateV1,
    ) -> Self {
        Self {
            evidence,
            name,
            state,
        }
    }

    pub fn evidence(&self) -> &ReceiptEvidenceRefV1 {
        &self.evidence
    }

    pub fn name(&self) -> &ReceiptDomainNameV1 {
        &self.name
    }

    pub const fn state(&self) -> ReceiptPolicyStateV1 {
        self.state
    }
}

impl fmt::Debug for ReceiptDomainPolicyV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ReceiptDomainPolicyV1")
            .field("evidence", &self.evidence)
            .field("name", &self.name)
            .field("state", &self.state)
            .finish()
    }
}

#[derive(Clone, Eq, PartialEq)]
pub enum ReceiptResolutionV1 {
    DirectIpLiteral,
    Observed {
        evidence: ReceiptEvidenceRefV1,
        lineage_id: String,
        query_name: ReceiptDomainNameV1,
        cname_chain: Vec<ReceiptDomainNameV1>,
        endpoints: Vec<IpAddr>,
    },
}

impl ReceiptResolutionV1 {
    pub fn observed(
        evidence: ReceiptEvidenceRefV1,
        lineage_id: impl Into<String>,
        query_name: ReceiptDomainNameV1,
        cname_chain: Vec<ReceiptDomainNameV1>,
        endpoints: Vec<IpAddr>,
    ) -> Result<Self, ReceiptError> {
        let lineage_id = lineage_id.into();
        validate_graphic_ascii("resolution lineage", &lineage_id, MAX_LINEAGE_BYTES)?;
        if cname_chain.len() > MAX_CNAME_NAMES {
            return Err(ReceiptError::TooManyCnameNames);
        }
        validate_endpoint_vector(&endpoints)?;
        Ok(Self::Observed {
            evidence,
            lineage_id,
            query_name,
            cname_chain,
            endpoints,
        })
    }

    pub const fn direct_ip_literal() -> Self {
        Self::DirectIpLiteral
    }
}

impl fmt::Debug for ReceiptResolutionV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DirectIpLiteral => f.write_str("ReceiptResolutionV1::DirectIpLiteral"),
            Self::Observed {
                evidence,
                cname_chain,
                endpoints,
                ..
            } => f
                .debug_struct("ReceiptResolutionV1::Observed")
                .field("evidence", evidence)
                .field("lineage_id", &"<redacted>")
                .field("query_name", &"<redacted>")
                .field("cname_count", &cname_chain.len())
                .field("endpoint_count", &endpoints.len())
                .finish(),
        }
    }
}

#[derive(Clone, Eq, PartialEq)]
pub struct ReceiptEndpointAssessmentV1 {
    evidence: ReceiptEvidenceRefV1,
    address: IpAddr,
    state: ReceiptPolicyStateV1,
    ipv6_allocation_evidence: Option<ReceiptEvidenceRefV1>,
    ipv6_envelope: Option<String>,
}

impl ReceiptEndpointAssessmentV1 {
    pub fn ipv4(
        evidence: ReceiptEvidenceRefV1,
        address: Ipv4Addr,
        state: ReceiptPolicyStateV1,
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
        evidence: ReceiptEvidenceRefV1,
        address: Ipv6Addr,
        state: ReceiptPolicyStateV1,
        allocation_evidence: ReceiptEvidenceRefV1,
        envelope: impl Into<String>,
    ) -> Result<Self, ReceiptError> {
        let envelope = envelope.into();
        validate_graphic_ascii("ipv6 envelope", &envelope, MAX_REF_BYTES)?;
        Ok(Self {
            evidence,
            address: IpAddr::V6(address),
            state,
            ipv6_allocation_evidence: Some(allocation_evidence),
            ipv6_envelope: Some(envelope),
        })
    }

    pub fn evidence(&self) -> &ReceiptEvidenceRefV1 {
        &self.evidence
    }

    pub const fn address(&self) -> IpAddr {
        self.address
    }

    pub const fn state(&self) -> ReceiptPolicyStateV1 {
        self.state
    }
}

impl fmt::Debug for ReceiptEndpointAssessmentV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ReceiptEndpointAssessmentV1")
            .field("evidence", &self.evidence)
            .field("address", &"<redacted>")
            .field("state", &self.state)
            .field(
                "ipv6_allocation_evidence",
                &self.ipv6_allocation_evidence,
            )
            .field(
                "ipv6_envelope",
                &self.ipv6_envelope.as_ref().map(|_| "<redacted>"),
            )
            .finish()
    }
}

#[derive(Clone, Eq, PartialEq)]
pub struct AdmissionReceiptInputV1 {
    pub origin: ReceiptOriginV1,
    pub initial_domain_policy: Option<ReceiptDomainPolicyV1>,
    pub cname_domain_policies: Vec<ReceiptDomainPolicyV1>,
    pub resolution: ReceiptResolutionV1,
    pub endpoint_assessments: Vec<ReceiptEndpointAssessmentV1>,
    pub attempt_id: String,
}

impl fmt::Debug for AdmissionReceiptInputV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AdmissionReceiptInputV1")
            .field("origin", &self.origin)
            .field("initial_domain_policy", &self.initial_domain_policy)
            .field("cname_domain_policies", &self.cname_domain_policies)
            .field("resolution", &self.resolution)
            .field("endpoint_assessments", &self.endpoint_assessments)
            .field("attempt_id", &"<redacted>")
            .finish()
    }
}

#[derive(Clone, Eq, PartialEq)]
pub struct AdmissionDecisionReceiptV1 {
    input: AdmissionReceiptInputV1,
}

impl AdmissionDecisionReceiptV1 {
    pub fn new(input: AdmissionReceiptInputV1) -> Result<Self, ReceiptError> {
        validate_positive_input(&input)?;
        Ok(Self { input })
    }

    pub const fn authority_scope(&self) -> AdmissionReceiptAuthorityScopeV1 {
        AdmissionReceiptAuthorityScopeV1::AuditEvidenceOnly
    }

    pub fn input(&self) -> &AdmissionReceiptInputV1 {
        &self.input
    }

    pub fn endpoint_set_projection_bytes(&self) -> Vec<u8> {
        canonical_endpoint_set_projection_owned(&self.input)
    }

    pub fn canonical_bytes(&self) -> Vec<u8> {
        canonical_receipt_bytes(&self.input)
    }

    pub fn commitment_sha256(&self) -> AdmissionCommitmentSha256V1 {
        let transcript = self.canonical_bytes();
        let mut hasher = Sha256::new();
        hasher.update(COMMITMENT_PREFIX);
        hasher.update((transcript.len() as u64).to_be_bytes());
        hasher.update(&transcript);
        AdmissionCommitmentSha256V1(hasher.finalize().into())
    }
}

impl fmt::Debug for AdmissionDecisionReceiptV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AdmissionDecisionReceiptV1")
            .field("schema", &RECEIPT_SCHEMA_V1)
            .field("admission_profile", &ADMISSION_PROFILE_V2)
            .field("authority_scope", &AUDIT_AUTHORITY_SCOPE_V1)
            .field("origin", &self.input.origin)
            .field("cname_policy_count", &self.input.cname_domain_policies.len())
            .field(
                "endpoint_assessment_count",
                &self.input.endpoint_assessments.len(),
            )
            .field("attempt_id", &"<redacted>")
            .finish()
    }
}

#[derive(Clone, Copy, Eq, PartialEq, Hash)]
pub struct AdmissionCommitmentSha256V1([u8; 32]);

impl AdmissionCommitmentSha256V1 {
    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }

    pub fn to_hex(self) -> String {
        let mut out = String::with_capacity(64);
        const HEX: &[u8; 16] = b"0123456789abcdef";
        for byte in self.0 {
            out.push(HEX[(byte >> 4) as usize] as char);
            out.push(HEX[(byte & 0x0f) as usize] as char);
        }
        out
    }

    pub const fn profile_id() -> &'static str {
        COMMITMENT_PROFILE_SHA256_V1
    }
}

impl fmt::Debug for AdmissionCommitmentSha256V1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("AdmissionCommitmentSha256V1")
            .field(&self.to_hex())
            .finish()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReceiptError {
    EmptyField,
    FieldTooLong,
    NonGraphicAscii,
    InvalidDomainLength,
    NonAsciiDomain,
    NonCanonicalDomain,
    InvalidDomainLabel,
    InvalidTarget,
    WrongLocatorProfile,
    WrongDomainProfile,
    WrongResolverProfile,
    WrongEndpointProfile,
    WrongIpv6AllocationProfile,
    WrongIpv6Envelope,
    PortNot443,
    UserinfoPresent,
    MissingInitialDomainPolicy,
    UnexpectedInitialDomainPolicy,
    DomainNameMismatch,
    DomainPolicyRefused,
    ResolutionKindMismatch,
    ResolutionQueryMismatch,
    CnameChainDoesNotBeginAtQuery,
    CnamePolicyCountMismatch,
    CnamePolicyNameMismatch,
    TooManyCnameNames,
    TooManyEndpoints,
    DuplicateEndpoint,
    EndpointSetEmpty,
    EndpointAssessmentCountMismatch,
    EndpointAssessmentAddressMismatch,
    EndpointRefused,
    MissingIpv6AllocationEvidence,
    UnexpectedIpv6AllocationEvidence,
    InvalidAttemptId,
}

impl fmt::Display for ReceiptError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for ReceiptError {}

fn validate_positive_input(input: &AdmissionReceiptInputV1) -> Result<(), ReceiptError> {
    validate_graphic_ascii("attempt id", &input.attempt_id, MAX_ATTEMPT_BYTES)
        .map_err(|_| ReceiptError::InvalidAttemptId)?;

    if input.origin.locator_ref.profile_id() != LOCATOR_PROFILE_V1 {
        return Err(ReceiptError::WrongLocatorProfile);
    }
    if input.origin.port != 443 {
        return Err(ReceiptError::PortNot443);
    }
    if input.origin.userinfo_present {
        return Err(ReceiptError::UserinfoPresent);
    }

    match &input.origin.host {
        ReceiptHostV1::Domain(origin_name) => {
            let initial = input
                .initial_domain_policy
                .as_ref()
                .ok_or(ReceiptError::MissingInitialDomainPolicy)?;
            validate_domain_policy(initial)?;
            if initial.name != *origin_name {
                return Err(ReceiptError::DomainNameMismatch);
            }

            let ReceiptResolutionV1::Observed {
                evidence,
                query_name,
                cname_chain,
                endpoints,
                ..
            } = &input.resolution
            else {
                return Err(ReceiptError::ResolutionKindMismatch);
            };
            if evidence.profile_id() != RESOLVER_PROFILE_V1 {
                return Err(ReceiptError::WrongResolverProfile);
            }
            if query_name != origin_name {
                return Err(ReceiptError::ResolutionQueryMismatch);
            }
            if !cname_chain.is_empty() && cname_chain.first() != Some(query_name) {
                return Err(ReceiptError::CnameChainDoesNotBeginAtQuery);
            }
            if cname_chain.len() != input.cname_domain_policies.len() {
                return Err(ReceiptError::CnamePolicyCountMismatch);
            }
            for (name, policy) in cname_chain.iter().zip(&input.cname_domain_policies) {
                validate_domain_policy(policy)?;
                if policy.name != *name {
                    return Err(ReceiptError::CnamePolicyNameMismatch);
                }
            }
            validate_endpoint_alignment(endpoints, &input.endpoint_assessments)?;
        }
        ReceiptHostV1::Ipv4(address) => {
            if input.initial_domain_policy.is_some() || !input.cname_domain_policies.is_empty() {
                return Err(ReceiptError::UnexpectedInitialDomainPolicy);
            }
            if !matches!(input.resolution, ReceiptResolutionV1::DirectIpLiteral) {
                return Err(ReceiptError::ResolutionKindMismatch);
            }
            validate_endpoint_alignment(
                &[IpAddr::V4(*address)],
                &input.endpoint_assessments,
            )?;
        }
        ReceiptHostV1::Ipv6(address) => {
            if input.initial_domain_policy.is_some() || !input.cname_domain_policies.is_empty() {
                return Err(ReceiptError::UnexpectedInitialDomainPolicy);
            }
            if !matches!(input.resolution, ReceiptResolutionV1::DirectIpLiteral) {
                return Err(ReceiptError::ResolutionKindMismatch);
            }
            validate_endpoint_alignment(
                &[IpAddr::V6(*address)],
                &input.endpoint_assessments,
            )?;
        }
    }

    Ok(())
}

fn validate_domain_policy(policy: &ReceiptDomainPolicyV1) -> Result<(), ReceiptError> {
    if policy.evidence.profile_id() != DOMAIN_POLICY_PROFILE_V1 {
        return Err(ReceiptError::WrongDomainProfile);
    }
    if policy.state != ReceiptPolicyStateV1::Eligible {
        return Err(ReceiptError::DomainPolicyRefused);
    }
    Ok(())
}

fn validate_endpoint_alignment(
    endpoints: &[IpAddr],
    assessments: &[ReceiptEndpointAssessmentV1],
) -> Result<(), ReceiptError> {
    validate_endpoint_vector(endpoints)?;
    if endpoints.is_empty() {
        return Err(ReceiptError::EndpointSetEmpty);
    }
    if endpoints.len() != assessments.len() {
        return Err(ReceiptError::EndpointAssessmentCountMismatch);
    }

    for (address, assessment) in endpoints.iter().zip(assessments) {
        if assessment.evidence.profile_id() != ENDPOINT_POLICY_PROFILE_V1 {
            return Err(ReceiptError::WrongEndpointProfile);
        }
        if *address != assessment.address {
            return Err(ReceiptError::EndpointAssessmentAddressMismatch);
        }
        if assessment.state != ReceiptPolicyStateV1::Eligible {
            return Err(ReceiptError::EndpointRefused);
        }
        match address {
            IpAddr::V4(_) => {
                if assessment.ipv6_allocation_evidence.is_some()
                    || assessment.ipv6_envelope.is_some()
                {
                    return Err(ReceiptError::UnexpectedIpv6AllocationEvidence);
                }
            }
            IpAddr::V6(_) => {
                let allocation = assessment
                    .ipv6_allocation_evidence
                    .as_ref()
                    .ok_or(ReceiptError::MissingIpv6AllocationEvidence)?;
                if allocation.profile_id() != IPV6_ADDRESS_SPACE_PROFILE_V1 {
                    return Err(ReceiptError::WrongIpv6AllocationProfile);
                }
                if assessment.ipv6_envelope.as_deref()
                    != Some(IPV6_GLOBAL_UNICAST_ENVELOPE_V1)
                {
                    return Err(ReceiptError::WrongIpv6Envelope);
                }
            }
        }
    }
    Ok(())
}

fn validate_endpoint_vector(endpoints: &[IpAddr]) -> Result<(), ReceiptError> {
    if endpoints.len() > MAX_ENDPOINTS {
        return Err(ReceiptError::TooManyEndpoints);
    }
    let unique: HashSet<IpAddr> = endpoints.iter().copied().collect();
    if unique.len() != endpoints.len() {
        return Err(ReceiptError::DuplicateEndpoint);
    }
    Ok(())
}

fn validate_graphic_ascii(
    _label: &'static str,
    value: &str,
    max: usize,
) -> Result<(), ReceiptError> {
    if value.is_empty() {
        return Err(ReceiptError::EmptyField);
    }
    if value.len() > max {
        return Err(ReceiptError::FieldTooLong);
    }
    if !value.bytes().all(|b| (0x21..=0x7e).contains(&b)) {
        return Err(ReceiptError::NonGraphicAscii);
    }
    Ok(())
}

fn validate_http_origin_form(target: &str) -> Result<(), ReceiptError> {
    if target.is_empty() || target.len() > MAX_TARGET_BYTES || !target.starts_with('/') {
        return Err(ReceiptError::InvalidTarget);
    }
    if target.bytes().any(|b| !(0x21..=0x7e).contains(&b) || b == b'#') {
        return Err(ReceiptError::InvalidTarget);
    }
    Ok(())
}

fn canonical_receipt_bytes(input: &AdmissionReceiptInputV1) -> Vec<u8> {
    let mut out = Vec::new();
    out.extend_from_slice(TRANSCRIPT_PREFIX);
    out.extend_from_slice(&TRANSCRIPT_VERSION.to_be_bytes());

    push_tlv(&mut out, 1, RECEIPT_SCHEMA_V1.as_bytes());
    push_tlv(&mut out, 2, B0_PRODUCT_HEAD.as_bytes());
    push_tlv(&mut out, 3, ADMISSION_CORPUS_HEAD.as_bytes());
    push_tlv(&mut out, 4, ADMISSION_PROFILE_V2.as_bytes());
    push_tlv(&mut out, 5, &encode_origin(&input.origin));
    push_tlv(
        &mut out,
        6,
        &encode_optional_domain_policy(input.initial_domain_policy.as_ref()),
    );
    push_tlv(
        &mut out,
        7,
        &encode_list(
            input
                .cname_domain_policies
                .iter()
                .map(encode_domain_policy)
                .collect::<Vec<_>>(),
        ),
    );
    push_tlv(&mut out, 8, &encode_resolution(&input.resolution));
    push_tlv(
        &mut out,
        9,
        &encode_list(
            input
                .endpoint_assessments
                .iter()
                .map(encode_endpoint_assessment)
                .collect::<Vec<_>>(),
        ),
    );
    push_tlv(
        &mut out,
        10,
        &canonical_endpoint_set_projection_owned(input),
    );
    push_tlv(&mut out, 11, input.attempt_id.as_bytes());
    push_tlv(&mut out, 12, &[0x01]);
    push_tlv(&mut out, 13, AUDIT_AUTHORITY_SCOPE_V1.as_bytes());

    out
}

fn encode_evidence_ref(value: &ReceiptEvidenceRefV1) -> Vec<u8> {
    let mut out = Vec::new();
    push_tlv(&mut out, 1, value.profile_id.as_bytes());
    push_tlv(&mut out, 2, value.subject_ref.as_bytes());
    out
}

fn encode_host(value: &ReceiptHostV1) -> Vec<u8> {
    let mut out = Vec::new();
    match value {
        ReceiptHostV1::Domain(name) => {
            push_tlv(&mut out, 1, &[0x01]);
            push_tlv(&mut out, 2, name.as_str().as_bytes());
        }
        ReceiptHostV1::Ipv4(address) => {
            push_tlv(&mut out, 1, &[0x02]);
            push_tlv(&mut out, 2, &address.octets());
        }
        ReceiptHostV1::Ipv6(address) => {
            push_tlv(&mut out, 1, &[0x03]);
            push_tlv(&mut out, 2, &address.octets());
        }
    }
    out
}

fn encode_origin(value: &ReceiptOriginV1) -> Vec<u8> {
    let mut out = Vec::new();
    push_tlv(&mut out, 1, &encode_evidence_ref(&value.locator_ref));
    push_tlv(&mut out, 2, &[0x01]);
    push_tlv(&mut out, 3, &encode_host(&value.host));
    push_tlv(&mut out, 4, &value.port.to_be_bytes());
    push_tlv(&mut out, 5, value.target.as_bytes());
    push_tlv(
        &mut out,
        6,
        &[if value.userinfo_present { 0x01 } else { 0x00 }],
    );
    out
}

fn encode_optional_domain_policy(value: Option<&ReceiptDomainPolicyV1>) -> Vec<u8> {
    match value {
        None => vec![0x00],
        Some(value) => {
            let encoded = encode_domain_policy(value);
            let mut out = Vec::with_capacity(5 + encoded.len());
            out.push(0x01);
            out.extend_from_slice(&(encoded.len() as u32).to_be_bytes());
            out.extend_from_slice(&encoded);
            out
        }
    }
}

fn encode_domain_policy(value: &ReceiptDomainPolicyV1) -> Vec<u8> {
    let mut out = Vec::new();
    push_tlv(&mut out, 1, &encode_evidence_ref(&value.evidence));
    push_tlv(&mut out, 2, value.name.as_str().as_bytes());
    push_tlv(&mut out, 3, &[value.state.code()]);
    out
}

fn encode_resolution(value: &ReceiptResolutionV1) -> Vec<u8> {
    let mut out = Vec::new();
    match value {
        ReceiptResolutionV1::DirectIpLiteral => {
            push_tlv(&mut out, 1, &[0x01]);
        }
        ReceiptResolutionV1::Observed {
            evidence,
            lineage_id,
            query_name,
            cname_chain,
            endpoints,
        } => {
            push_tlv(&mut out, 1, &[0x02]);
            push_tlv(&mut out, 2, &encode_evidence_ref(evidence));
            push_tlv(&mut out, 3, lineage_id.as_bytes());
            push_tlv(&mut out, 4, query_name.as_str().as_bytes());
            push_tlv(&mut out, 5, &[0x01]);
            push_tlv(
                &mut out,
                6,
                &encode_list(
                    cname_chain
                        .iter()
                        .map(|name| name.as_str().as_bytes().to_vec())
                        .collect::<Vec<_>>(),
                ),
            );
            push_tlv(
                &mut out,
                7,
                &encode_list(endpoints.iter().map(encode_ip).collect::<Vec<_>>()),
            );
        }
    }
    out
}

fn encode_endpoint_assessment(value: &ReceiptEndpointAssessmentV1) -> Vec<u8> {
    let mut out = Vec::new();
    push_tlv(&mut out, 1, &encode_evidence_ref(&value.evidence));
    push_tlv(&mut out, 2, &encode_ip(&value.address));
    push_tlv(&mut out, 3, &[value.state.code()]);
    push_tlv(
        &mut out,
        4,
        &value
            .ipv6_allocation_evidence
            .as_ref()
            .map(encode_evidence_ref)
            .unwrap_or_default(),
    );
    push_tlv(
        &mut out,
        5,
        value
            .ipv6_envelope
            .as_deref()
            .unwrap_or_default()
            .as_bytes(),
    );
    out
}

fn encode_ip(value: &IpAddr) -> Vec<u8> {
    match value {
        IpAddr::V4(address) => {
            let mut out = Vec::with_capacity(5);
            out.push(0x04);
            out.extend_from_slice(&address.octets());
            out
        }
        IpAddr::V6(address) => {
            let mut out = Vec::with_capacity(17);
            out.push(0x06);
            out.extend_from_slice(&address.octets());
            out
        }
    }
}

fn canonical_endpoint_set_projection(endpoints: &[IpAddr]) -> Vec<u8> {
    let mut encoded = endpoints.iter().map(encode_ip).collect::<Vec<_>>();
    encoded.sort();
    encoded.dedup();
    encode_list(encoded)
}

fn canonical_endpoint_set_projection_owned(input: &AdmissionReceiptInputV1) -> Vec<u8> {
    match &input.resolution {
        ReceiptResolutionV1::Observed { endpoints, .. } => {
            canonical_endpoint_set_projection(endpoints)
        }
        ReceiptResolutionV1::DirectIpLiteral => match &input.origin.host {
            ReceiptHostV1::Ipv4(address) => {
                canonical_endpoint_set_projection(&[IpAddr::V4(*address)])
            }
            ReceiptHostV1::Ipv6(address) => {
                canonical_endpoint_set_projection(&[IpAddr::V6(*address)])
            }
            ReceiptHostV1::Domain(_) => encode_list(Vec::new()),
        },
    }
}

fn encode_list(items: Vec<Vec<u8>>) -> Vec<u8> {
    let mut out = Vec::new();
    out.extend_from_slice(&(items.len() as u16).to_be_bytes());
    for item in items {
        out.extend_from_slice(&(item.len() as u32).to_be_bytes());
        out.extend_from_slice(&item);
    }
    out
}

fn push_tlv(out: &mut Vec<u8>, tag: u16, value: &[u8]) {
    out.extend_from_slice(&tag.to_be_bytes());
    out.extend_from_slice(&(value.len() as u32).to_be_bytes());
    out.extend_from_slice(value);
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIRST_VECTOR_CANONICAL_HEX: &str = "6d7963656c69782d7765622d7461726765742d61646d697373696f6e2d726563656970742d76310000010001000000276d7963656c69783a7765622d7461726765742d61646d697373696f6e2d726563656970743a7631000200000028643631656264643230393764643139383663323763346266366237333131626230316639316137340003000000283533656533663831383938346662313130656630616236393739353666393866623466626163633700040000002b6d7963656c69783a6f7264696e6172792d7075626c69632d7461726765742d61646d697373696f6e3a76320005000000a000010000003a00010000001d6d7963656c69783a7765622d75726c3a68747470732d6f6e6c793a7631000200000011666978747572653a6c6f6361746f723a410002000000010100030000002c0001000000010100020000001f7777772e7075626c69632d73796e7468657469632e696e76616c6964746c6400040000000201bb0005000000122f7265706f72743f636173653d616c7068610006000000010000060000008b01000000860001000000540001000000306d7963656c69783a7765622d646f6d61696e2d706f6c6963793a6f7264696e6172792d7075626c69632d646e733a7631000200000018666978747572653a646f6d61696e2d706f6c6963793a413000020000001f7777772e7075626c69632d73796e7468657469632e696e76616c6964746c640003000000010100070000000200000008000000a80001000000010200020000003f00010000001f6d7963656c69783a73796e7468657469632d7265736f6c7574696f6e3a7631000200000014666978747572653a7265736f6c7574696f6e3a41000300000011666978747572653a6c696e656167653a4100040000001f7777772e7075626c69632d73796e7468657469632e696e76616c6964746c6400050000000101000600000002000000070000000b000100000005040808080800090000008600010000008000010000005c0001000000376d7963656c69783a7765622d656e64706f696e742d706f6c6963793a6f7264696e6172792d7075626c69632d656e64706f696e743a7631000200000019666978747572653a656e64706f696e742d706f6c6963793a41000200000005040808080800030000000101000400000000000500000000000a0000000b0001000000050408080808000b00000011666978747572652d617474656d70742d41000c0000000101000d0000001361756469742d65766964656e63652d6f6e6c79";

    fn evidence(profile: &str, subject: &str) -> ReceiptEvidenceRefV1 {
        ReceiptEvidenceRefV1::new(profile, subject).unwrap()
    }

    fn domain(name: &str) -> ReceiptDomainNameV1 {
        ReceiptDomainNameV1::new(name).unwrap()
    }

    fn first_vector() -> AdmissionDecisionReceiptV1 {
        AdmissionDecisionReceiptV1::new(AdmissionReceiptInputV1 {
            origin: ReceiptOriginV1::new(
                evidence(LOCATOR_PROFILE_V1, "fixture:locator:A"),
                ReceiptHostV1::Domain(domain("www.public-synthetic.invalidtld")),
                443,
                "/report?case=alpha",
                false,
            )
            .unwrap(),
            initial_domain_policy: Some(ReceiptDomainPolicyV1::new(
                evidence(DOMAIN_POLICY_PROFILE_V1, "fixture:domain-policy:A0"),
                domain("www.public-synthetic.invalidtld"),
                ReceiptPolicyStateV1::Eligible,
            )),
            cname_domain_policies: vec![],
            resolution: ReceiptResolutionV1::observed(
                evidence(RESOLVER_PROFILE_V1, "fixture:resolution:A"),
                "fixture:lineage:A",
                domain("www.public-synthetic.invalidtld"),
                vec![],
                vec![IpAddr::V4(Ipv4Addr::new(8, 8, 8, 8))],
            )
            .unwrap(),
            endpoint_assessments: vec![ReceiptEndpointAssessmentV1::ipv4(
                evidence(
                    ENDPOINT_POLICY_PROFILE_V1,
                    "fixture:endpoint-policy:A",
                ),
                Ipv4Addr::new(8, 8, 8, 8),
                ReceiptPolicyStateV1::Eligible,
            )],
            attempt_id: "fixture-attempt-A".to_owned(),
        })
        .unwrap()
    }

    fn dual_vector(reverse: bool) -> AdmissionDecisionReceiptV1 {
        let v4 = IpAddr::V4(Ipv4Addr::new(8, 8, 8, 8));
        let v6 = IpAddr::V6("2001:4860:4860::8888".parse().unwrap());
        let (endpoints, assessments) = if reverse {
            (
                vec![v6, v4],
                vec![
                    ReceiptEndpointAssessmentV1::ipv6(
                        evidence(
                            ENDPOINT_POLICY_PROFILE_V1,
                            "fixture:endpoint-policy:v6-google",
                        ),
                        "2001:4860:4860::8888".parse().unwrap(),
                        ReceiptPolicyStateV1::Eligible,
                        evidence(
                            IPV6_ADDRESS_SPACE_PROFILE_V1,
                            "fixture:ipv6-allocation:v6-google",
                        ),
                        IPV6_GLOBAL_UNICAST_ENVELOPE_V1,
                    )
                    .unwrap(),
                    ReceiptEndpointAssessmentV1::ipv4(
                        evidence(
                            ENDPOINT_POLICY_PROFILE_V1,
                            "fixture:endpoint-policy:v4-google",
                        ),
                        Ipv4Addr::new(8, 8, 8, 8),
                        ReceiptPolicyStateV1::Eligible,
                    ),
                ],
            )
        } else {
            (
                vec![v4, v6],
                vec![
                    ReceiptEndpointAssessmentV1::ipv4(
                        evidence(
                            ENDPOINT_POLICY_PROFILE_V1,
                            "fixture:endpoint-policy:v4-google",
                        ),
                        Ipv4Addr::new(8, 8, 8, 8),
                        ReceiptPolicyStateV1::Eligible,
                    ),
                    ReceiptEndpointAssessmentV1::ipv6(
                        evidence(
                            ENDPOINT_POLICY_PROFILE_V1,
                            "fixture:endpoint-policy:v6-google",
                        ),
                        "2001:4860:4860::8888".parse().unwrap(),
                        ReceiptPolicyStateV1::Eligible,
                        evidence(
                            IPV6_ADDRESS_SPACE_PROFILE_V1,
                            "fixture:ipv6-allocation:v6-google",
                        ),
                        IPV6_GLOBAL_UNICAST_ENVELOPE_V1,
                    )
                    .unwrap(),
                ],
            )
        };

        AdmissionDecisionReceiptV1::new(AdmissionReceiptInputV1 {
            origin: ReceiptOriginV1::new(
                evidence(LOCATOR_PROFILE_V1, "fixture:locator:dual"),
                ReceiptHostV1::Domain(domain("dual.public-synthetic.invalidtld")),
                443,
                "/dataset?q=beta",
                false,
            )
            .unwrap(),
            initial_domain_policy: Some(ReceiptDomainPolicyV1::new(
                evidence(
                    DOMAIN_POLICY_PROFILE_V1,
                    "fixture:domain-policy:dual-initial",
                ),
                domain("dual.public-synthetic.invalidtld"),
                ReceiptPolicyStateV1::Eligible,
            )),
            cname_domain_policies: vec![
                ReceiptDomainPolicyV1::new(
                    evidence(
                        DOMAIN_POLICY_PROFILE_V1,
                        "fixture:domain-policy:dual-cname0",
                    ),
                    domain("dual.public-synthetic.invalidtld"),
                    ReceiptPolicyStateV1::Eligible,
                ),
                ReceiptDomainPolicyV1::new(
                    evidence(
                        DOMAIN_POLICY_PROFILE_V1,
                        "fixture:domain-policy:dual-cname1",
                    ),
                    domain("edge.public-synthetic.invalidtld"),
                    ReceiptPolicyStateV1::Eligible,
                ),
            ],
            resolution: ReceiptResolutionV1::observed(
                evidence(RESOLVER_PROFILE_V1, "fixture:resolution:dual"),
                "fixture:lineage:dual",
                domain("dual.public-synthetic.invalidtld"),
                vec![
                    domain("dual.public-synthetic.invalidtld"),
                    domain("edge.public-synthetic.invalidtld"),
                ],
                endpoints,
            )
            .unwrap(),
            endpoint_assessments: assessments,
            attempt_id: "fixture-attempt-dual".to_owned(),
        })
        .unwrap()
    }

    fn direct_ipv6_vector() -> AdmissionDecisionReceiptV1 {
        let address: Ipv6Addr = "2001:4860:4860::8888".parse().unwrap();
        AdmissionDecisionReceiptV1::new(AdmissionReceiptInputV1 {
            origin: ReceiptOriginV1::new(
                evidence(LOCATOR_PROFILE_V1, "fixture:locator:D"),
                ReceiptHostV1::Ipv6(address),
                443,
                "/direct",
                false,
            )
            .unwrap(),
            initial_domain_policy: None,
            cname_domain_policies: vec![],
            resolution: ReceiptResolutionV1::direct_ip_literal(),
            endpoint_assessments: vec![ReceiptEndpointAssessmentV1::ipv6(
                evidence(ENDPOINT_POLICY_PROFILE_V1, "fixture:endpoint-policy:D"),
                address,
                ReceiptPolicyStateV1::Eligible,
                evidence(
                    IPV6_ADDRESS_SPACE_PROFILE_V1,
                    "fixture:ipv6-allocation:D",
                ),
                IPV6_GLOBAL_UNICAST_ENVELOPE_V1,
            )
            .unwrap()],
            attempt_id: "fixture-attempt-D".to_owned(),
        })
        .unwrap()
    }

    #[test]
    fn first_golden_vector_matches_length_projection_and_commitment() {
        let receipt = first_vector();
        assert_eq!(receipt.canonical_bytes().len(), 933);
        assert_eq!(hex(&receipt.canonical_bytes()), FIRST_VECTOR_CANONICAL_HEX);
        assert_eq!(
            hex(&receipt.endpoint_set_projection_bytes()),
            "0001000000050408080808"
        );
        assert_eq!(
            receipt.commitment_sha256().to_hex(),
            "5ddc613701686b553aba946159cea1a3d87363d297bec9ddce7b8fe081bb702e"
        );
    }

    #[test]
    fn endpoint_order_changes_receipt_but_not_set_projection() {
        let a = dual_vector(false);
        let b = dual_vector(true);
        assert_eq!(a.canonical_bytes().len(), 1623);
        assert_eq!(b.canonical_bytes().len(), 1623);
        assert_eq!(
            a.endpoint_set_projection_bytes(),
            b.endpoint_set_projection_bytes()
        );
        assert_ne!(a.canonical_bytes(), b.canonical_bytes());
        assert_eq!(
            a.commitment_sha256().to_hex(),
            "4b34ca1819ae4b1fa08436c74097f06db75289ed1af6a7687134030581529d1d"
        );
        assert_eq!(
            b.commitment_sha256().to_hex(),
            "b6c4b4365dc48cf5246c7f4e03824fa3feeffa4b1177c94c6e143f74e26800d3"
        );
    }

    #[test]
    fn direct_ipv6_vector_matches_golden_commitment() {
        let receipt = direct_ipv6_vector();
        assert_eq!(receipt.canonical_bytes().len(), 722);
        assert_eq!(
            hex(&receipt.endpoint_set_projection_bytes()),
            "0001000000110620014860486000000000000000008888"
        );
        assert_eq!(
            receipt.commitment_sha256().to_hex(),
            "3cf8e709be7c75e3f529a1e2054f6667669521fcccbe8d74222a27240f3e04b0"
        );
    }

    #[test]
    fn debug_redacts_sensitive_receipt_material() {
        let receipt = first_vector();
        let rendered = format!("{receipt:?}");
        assert!(!rendered.contains("www.public-synthetic.invalidtld"));
        assert!(!rendered.contains("/report?case=alpha"));
        assert!(!rendered.contains("fixture-attempt-A"));
        assert!(!rendered.contains("8.8.8.8"));
        assert!(!rendered.contains("fixture:locator:A"));
    }

    #[test]
    fn receipt_does_not_accept_refused_policy_state() {
        let mut input = first_vector().input().clone();
        input.endpoint_assessments[0].state = ReceiptPolicyStateV1::Refused;
        assert_eq!(
            AdmissionDecisionReceiptV1::new(input),
            Err(ReceiptError::EndpointRefused)
        );
    }

    fn hex(bytes: &[u8]) -> String {
        let mut out = String::with_capacity(bytes.len() * 2);
        const HEX: &[u8; 16] = b"0123456789abcdef";
        for byte in bytes {
            out.push(HEX[(byte >> 4) as usize] as char);
            out.push(HEX[(byte & 0x0f) as usize] as char);
        }
        out
    }
}
