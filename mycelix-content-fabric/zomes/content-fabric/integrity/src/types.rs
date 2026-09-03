use hdi::prelude::*;

pub const SCHEMA_VERSION_V1: u16 = 1;
pub const IROH_ALPN_V1: &str = "mycelix/content/1";
pub const MAX_BLOB_SIZE_V1: u64 = 16 * 1024 * 1024 * 1024 * 1024;
pub const MAX_AD_TTL_SECONDS_V1: u32 = 86_400;
pub const MAX_FAILURE_DOMAINS_V1: usize = 7;

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum DigestAlgorithmV1 {
    Blake3_256,
    Sha256,
}

impl DigestAlgorithmV1 {
    pub fn wire_tag(self) -> u8 {
        match self {
            Self::Blake3_256 => 1,
            Self::Sha256 => 2,
        }
    }

    pub fn token(self) -> &'static str {
        match self {
            Self::Blake3_256 => "blake3-256",
            Self::Sha256 => "sha256",
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ContentDigestRefV1 {
    pub algorithm: DigestAlgorithmV1,
    pub bytes: [u8; 32],
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum FailureDomainKindV1 {
    Operator,
    Machine,
    Site,
    Asn,
    Region,
    Jurisdiction,
    PowerDomain,
}

impl FailureDomainKindV1 {
    pub fn wire_tag(self) -> u8 {
        match self {
            Self::Operator => 1,
            Self::Machine => 2,
            Self::Site => 3,
            Self::Asn => 4,
            Self::Region => 5,
            Self::Jurisdiction => 6,
            Self::PowerDomain => 7,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct FailureDomainClaimV1 {
    pub kind: FailureDomainKindV1,
    pub value: String,
}

/// A signed service/reachability claim. Failure-domain values are self-claims,
/// not independent evidence of failure-domain separation.
#[hdk_entry_helper]
#[derive(Clone, PartialEq, Eq)]
pub struct ProviderAdvertisementV1 {
    pub schema_version: u16,
    pub provider: AgentPubKey,
    /// The provider source-chain head that the endpoint signed. Validation
    /// requires this to equal the Create action's `prev_action`, making the
    /// endpoint proof single-use at one chain position.
    pub binding_prev_action: ActionHash,
    pub iroh_endpoint_id: [u8; 32],
    pub endpoint_binding_signature: [u8; 64],
    pub protocol: String,
    pub supported_algorithms: Vec<DigestAlgorithmV1>,
    pub max_blob_size_bytes: u64,
    pub ttl_seconds: u32,
    pub failure_domains: Vec<FailureDomainClaimV1>,
}

/// Opt-in digest discoverability. Absence means only "not advertised".
#[hdk_entry_helper]
#[derive(Clone, PartialEq, Eq)]
pub struct ContentAvailabilityClaimV1 {
    pub schema_version: u16,
    pub provider: AgentPubKey,
    pub advertisement: ActionHash,
    pub digest: ContentDigestRefV1,
    pub size_bytes: u64,
    pub ttl_seconds: u32,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum WithdrawalReasonV1 {
    Maintenance,
    Rotated,
    Retired,
    Superseded,
    Compromised,
    Other,
}

/// Append-only early retirement of a provider advertisement.
#[hdk_entry_helper]
#[derive(Clone, PartialEq, Eq)]
pub struct ProviderWithdrawalV1 {
    pub schema_version: u16,
    pub provider: AgentPubKey,
    pub advertisement: ActionHash,
    pub reason: WithdrawalReasonV1,
    pub note: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ObservationOutcomeV1 {
    VerifiedComplete { size_bytes: u64 },
    /// Deliberately conflates absent and unauthorized/hidden service outcomes.
    UnavailableOrHidden,
    Busy,
    TransferFailed,
    ProviderReportedIntegrityFailure,
    DigestMismatch {
        observed_digest: ContentDigestRefV1,
        size_bytes: u64,
    },
}

/// Signed report of one observer's experience. It is evidence, not universal truth.
#[hdk_entry_helper]
#[derive(Clone, PartialEq, Eq)]
pub struct ReplicaObservationV1 {
    pub schema_version: u16,
    pub observer: AgentPubKey,
    pub provider: AgentPubKey,
    pub advertisement: ActionHash,
    pub digest: ContentDigestRefV1,
    pub outcome: ObservationOutcomeV1,
    pub latency_ms: Option<u64>,
}

/// Deterministic link-base entry. Only Content Fabric anchor grammars validate.
#[hdk_entry_helper]
#[derive(Clone, PartialEq, Eq)]
pub struct Anchor(pub String);

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(name = "ProviderAdvertisementV1", visibility = "public")]
    ProviderAdvertisementV1(ProviderAdvertisementV1),
    #[entry_type(name = "ContentAvailabilityClaimV1", visibility = "public")]
    ContentAvailabilityClaimV1(ContentAvailabilityClaimV1),
    #[entry_type(name = "ProviderWithdrawalV1", visibility = "public")]
    ProviderWithdrawalV1(ProviderWithdrawalV1),
    #[entry_type(name = "ReplicaObservationV1", visibility = "public")]
    ReplicaObservationV1(ReplicaObservationV1),
    #[entry_type(name = "Anchor", visibility = "public")]
    Anchor(Anchor),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllProviders,
    ProviderAdsByAgent,
    ProvidersByAlgorithm,
    AvailabilityByDigest,
    AvailabilityByAdvertisement,
    ObservationsByDigest,
    ObservationsByAdvertisement,
    WithdrawalsByAdvertisement,
}
