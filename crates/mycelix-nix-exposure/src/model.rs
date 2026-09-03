use std::collections::BTreeSet;

use mycelix_content_core::DigestAlgorithmV1;
use mycelix_infrastructure_types::{InfrastructureErrorV1, PartyIdV1, StableIdV1};
use mycelix_nix_cache::{NixCacheEntryV1, NixStoreHashV1};
use thiserror::Error;

const ENDPOINT_DOMAIN_V1: &str = "content-fabric/remote-exposure-endpoint";
const OBJECT_DOMAIN_V1: &str = "content-fabric/nix-exposure-object";
const GRANT_DOMAIN_V1: &str = "content-fabric/nix-exposure-grant";
const REVOCATION_DOMAIN_V1: &str = "content-fabric/nix-exposure-revocation";
const POLICY_DOMAIN_V1: &str = "content-fabric/nix-exposure-policy";
const SNAPSHOT_DOMAIN_V1: &str = "content-fabric/nix-exposure-snapshot";
const SCHEMA_VERSION_V1: u16 = 1;
const MAX_ENDPOINT_LABEL_LEN_V1: usize = 64;

#[derive(Debug, Error)]
pub enum ExposureErrorV1 {
    #[error("exposure authority principal must be non-zero")]
    ZeroAuthority,
    #[error("authenticated reader principal must be non-zero")]
    ZeroReaderPrincipal,
    #[error("exposure endpoint identity must be non-zero")]
    ZeroEndpoint,
    #[error("invalid exposure endpoint label: {0}")]
    InvalidEndpointLabel(String),
    #[error("exposure endpoint commitment does not match authority and label")]
    EndpointCommitmentMismatch,
    #[error("grant issuer does not own the exposure endpoint")]
    EndpointAuthorityMismatch,
    #[error("authenticated reader group identity must be non-zero")]
    ZeroReaderGroup,
    #[error("grant evidence must bind a SHA-256 raw NAR")]
    GrantDigestMustBeSha256,
    #[error("grant validity must satisfy authored_at <= valid_from < valid_until")]
    InvalidGrantWindow,
    #[error("revocation timing must satisfy authored_at <= effective_at")]
    InvalidRevocationTime,
    #[error("exposure policy timing limits must be non-zero")]
    InvalidPolicyTiming,
    #[error("exposure policy must allow at least one audience class")]
    NoAllowedAudience,
    #[error("strict remote exposure requires complete authority coverage")]
    IncompleteCoverage,
    #[error("conflicting evidence reused grant id {0:?}")]
    ConflictingGrantId(StableIdV1),
    #[error("conflicting evidence reused revocation id {0:?}")]
    ConflictingRevocationId(StableIdV1),
    #[error(transparent)]
    Infrastructure(#[from] InfrastructureErrorV1),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ExposureEndpointIdV1 {
    id: StableIdV1,
    authority: PartyIdV1,
}

impl ExposureEndpointIdV1 {
    pub fn derive(authority: PartyIdV1, label: &str) -> Result<Self, ExposureErrorV1> {
        if authority == PartyIdV1::ZERO {
            return Err(ExposureErrorV1::ZeroAuthority);
        }
        if !is_valid_endpoint_label(label) {
            return Err(ExposureErrorV1::InvalidEndpointLabel(label.to_string()));
        }
        let id = StableIdV1::derive(
            ENDPOINT_DOMAIN_V1,
            SCHEMA_VERSION_V1,
            &[&authority.0, label.as_bytes()],
        )?;
        Ok(Self { id, authority })
    }

    /// Reconstruct an endpoint from wire/storage parts while verifying that the
    /// claimed stable ID actually commits the supplied authority and label.
    pub fn from_stable_id(
        authority: PartyIdV1,
        label: &str,
        id: StableIdV1,
    ) -> Result<Self, ExposureErrorV1> {
        if id == StableIdV1::ZERO {
            return Err(ExposureErrorV1::ZeroEndpoint);
        }
        let derived = Self::derive(authority, label)?;
        if derived.id != id {
            return Err(ExposureErrorV1::EndpointCommitmentMismatch);
        }
        Ok(derived)
    }

    pub fn stable_id(self) -> StableIdV1 {
        self.id
    }

    pub fn authority(self) -> PartyIdV1 {
        self.authority
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct NixExposureObjectIdV1(StableIdV1);

impl NixExposureObjectIdV1 {
    pub fn from_entry(entry: &NixCacheEntryV1) -> Result<Self, ExposureErrorV1> {
        let mut fields = vec![
            entry.store_path().as_str().as_bytes().to_vec(),
            entry.nar_digest().algorithm.tag().as_bytes().to_vec(),
            entry.nar_digest().bytes.to_vec(),
            entry.nar_size().to_be_bytes().to_vec(),
            (entry.references().len() as u64).to_be_bytes().to_vec(),
        ];
        fields.extend(
            entry
                .references()
                .iter()
                .map(|reference| reference.as_str().as_bytes().to_vec()),
        );
        match entry.deriver() {
            Some(deriver) => {
                fields.push(b"deriver:some".to_vec());
                fields.push(deriver.as_str().as_bytes().to_vec());
            }
            None => fields.push(b"deriver:none".to_vec()),
        }
        fields.push((entry.signatures().len() as u64).to_be_bytes().to_vec());
        fields.extend(
            entry
                .signatures()
                .iter()
                .map(|signature| signature.as_str().as_bytes().to_vec()),
        );
        match entry.content_address() {
            Some(ca) => {
                fields.push(b"ca:some".to_vec());
                fields.push(ca.as_str().as_bytes().to_vec());
            }
            None => fields.push(b"ca:none".to_vec()),
        }
        let refs = fields.iter().map(Vec::as_slice).collect::<Vec<_>>();
        Ok(Self(StableIdV1::derive(
            OBJECT_DOMAIN_V1,
            SCHEMA_VERSION_V1,
            &refs,
        )?))
    }

    pub fn stable_id(self) -> StableIdV1 {
        self.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ExposureAssuranceV1 {
    SelfClaimed,
    OperatorVerified,
    CryptographicallyVerified,
}

impl ExposureAssuranceV1 {
    pub(crate) fn tag(self) -> &'static [u8] {
        match self {
            Self::SelfClaimed => b"self-claimed",
            Self::OperatorVerified => b"operator-verified",
            Self::CryptographicallyVerified => b"cryptographically-verified",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ExposureAudienceV1 {
    Public,
    AuthenticatedPrincipal(PartyIdV1),
    AuthenticatedGroup(StableIdV1),
}

impl ExposureAudienceV1 {
    fn validate(&self) -> Result<(), ExposureErrorV1> {
        match self {
            Self::Public => Ok(()),
            Self::AuthenticatedPrincipal(principal) if *principal == PartyIdV1::ZERO => {
                Err(ExposureErrorV1::ZeroReaderPrincipal)
            }
            Self::AuthenticatedGroup(group) if *group == StableIdV1::ZERO => {
                Err(ExposureErrorV1::ZeroReaderGroup)
            }
            Self::AuthenticatedPrincipal(_) | Self::AuthenticatedGroup(_) => Ok(()),
        }
    }

    pub(crate) fn canonical_parts(&self) -> (&'static [u8], &[u8]) {
        match self {
            Self::Public => (b"public", &[]),
            Self::AuthenticatedPrincipal(principal) => (b"principal", &principal.0),
            Self::AuthenticatedGroup(group) => (b"group", &group.0),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RemoteReaderV1 {
    principal: Option<PartyIdV1>,
    groups: Vec<StableIdV1>,
}

impl RemoteReaderV1 {
    pub fn anonymous() -> Self {
        Self {
            principal: None,
            groups: Vec::new(),
        }
    }

    pub fn authenticated(
        principal: PartyIdV1,
        groups: impl IntoIterator<Item = StableIdV1>,
    ) -> Result<Self, ExposureErrorV1> {
        if principal == PartyIdV1::ZERO {
            return Err(ExposureErrorV1::ZeroReaderPrincipal);
        }
        let groups = groups.into_iter().collect::<BTreeSet<_>>();
        if groups.contains(&StableIdV1::ZERO) {
            return Err(ExposureErrorV1::ZeroReaderGroup);
        }
        Ok(Self {
            principal: Some(principal),
            groups: groups.into_iter().collect(),
        })
    }

    pub fn principal(&self) -> Option<PartyIdV1> {
        self.principal
    }

    pub fn groups(&self) -> &[StableIdV1] {
        &self.groups
    }

    pub(crate) fn allows(&self, audience: &ExposureAudienceV1) -> bool {
        match audience {
            ExposureAudienceV1::Public => true,
            ExposureAudienceV1::AuthenticatedPrincipal(required) => {
                self.principal == Some(*required)
            }
            ExposureAudienceV1::AuthenticatedGroup(required) => {
                self.principal.is_some() && self.groups.binary_search(required).is_ok()
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExposureEvidenceCoverageV1 {
    Partial,
    CompleteForAuthority,
}

impl ExposureEvidenceCoverageV1 {
    pub(crate) fn tag(self) -> &'static [u8] {
        match self {
            Self::Partial => b"partial",
            Self::CompleteForAuthority => b"complete-for-authority",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RemoteExposureGrantEvidenceV1 {
    id: StableIdV1,
    issuer: PartyIdV1,
    endpoint: ExposureEndpointIdV1,
    store_hash: NixStoreHashV1,
    object_id: NixExposureObjectIdV1,
    audience: ExposureAudienceV1,
    authored_at_unix_ms: u64,
    valid_from_unix_ms: u64,
    valid_until_unix_ms: u64,
    assurance: ExposureAssuranceV1,
}

impl RemoteExposureGrantEvidenceV1 {
    #[allow(clippy::too_many_arguments)]
    pub fn for_entry(
        issuer: PartyIdV1,
        endpoint: ExposureEndpointIdV1,
        entry: &NixCacheEntryV1,
        audience: ExposureAudienceV1,
        authored_at_unix_ms: u64,
        valid_from_unix_ms: u64,
        valid_until_unix_ms: u64,
        assurance: ExposureAssuranceV1,
    ) -> Result<Self, ExposureErrorV1> {
        if issuer == PartyIdV1::ZERO {
            return Err(ExposureErrorV1::ZeroAuthority);
        }
        if endpoint.stable_id() == StableIdV1::ZERO {
            return Err(ExposureErrorV1::ZeroEndpoint);
        }
        if issuer != endpoint.authority() {
            return Err(ExposureErrorV1::EndpointAuthorityMismatch);
        }
        if entry.nar_digest().algorithm != DigestAlgorithmV1::Sha256 {
            return Err(ExposureErrorV1::GrantDigestMustBeSha256);
        }
        audience.validate()?;
        if authored_at_unix_ms > valid_from_unix_ms || valid_from_unix_ms >= valid_until_unix_ms {
            return Err(ExposureErrorV1::InvalidGrantWindow);
        }
        let store_hash = entry.store_path().hash().clone();
        let object_id = NixExposureObjectIdV1::from_entry(entry)?;
        let authored = authored_at_unix_ms.to_be_bytes();
        let valid_from = valid_from_unix_ms.to_be_bytes();
        let valid_until = valid_until_unix_ms.to_be_bytes();
        let (audience_tag, audience_id) = audience.canonical_parts();
        let endpoint_id = endpoint.stable_id();
        let object_stable_id = object_id.stable_id();
        let id = StableIdV1::derive(
            GRANT_DOMAIN_V1,
            SCHEMA_VERSION_V1,
            &[
                &issuer.0,
                &endpoint_id.0,
                store_hash.as_str().as_bytes(),
                &object_stable_id.0,
                audience_tag,
                audience_id,
                &authored,
                &valid_from,
                &valid_until,
                assurance.tag(),
            ],
        )?;
        Ok(Self {
            id,
            issuer,
            endpoint,
            store_hash,
            object_id,
            audience,
            authored_at_unix_ms,
            valid_from_unix_ms,
            valid_until_unix_ms,
            assurance,
        })
    }

    pub fn id(&self) -> StableIdV1 {
        self.id
    }

    pub fn issuer(&self) -> PartyIdV1 {
        self.issuer
    }

    pub fn endpoint(&self) -> ExposureEndpointIdV1 {
        self.endpoint
    }

    pub fn store_hash(&self) -> &NixStoreHashV1 {
        &self.store_hash
    }

    pub fn object_id(&self) -> NixExposureObjectIdV1 {
        self.object_id
    }

    pub fn audience(&self) -> &ExposureAudienceV1 {
        &self.audience
    }

    pub fn authored_at_unix_ms(&self) -> u64 {
        self.authored_at_unix_ms
    }

    pub fn valid_from_unix_ms(&self) -> u64 {
        self.valid_from_unix_ms
    }

    pub fn valid_until_unix_ms(&self) -> u64 {
        self.valid_until_unix_ms
    }

    pub fn assurance(&self) -> ExposureAssuranceV1 {
        self.assurance
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RemoteExposureRevocationEvidenceV1 {
    id: StableIdV1,
    grant_id: StableIdV1,
    issuer: PartyIdV1,
    authored_at_unix_ms: u64,
    effective_at_unix_ms: u64,
    assurance: ExposureAssuranceV1,
}

impl RemoteExposureRevocationEvidenceV1 {
    pub fn new(
        grant_id: StableIdV1,
        issuer: PartyIdV1,
        authored_at_unix_ms: u64,
        effective_at_unix_ms: u64,
        assurance: ExposureAssuranceV1,
    ) -> Result<Self, ExposureErrorV1> {
        if issuer == PartyIdV1::ZERO {
            return Err(ExposureErrorV1::ZeroAuthority);
        }
        if authored_at_unix_ms > effective_at_unix_ms {
            return Err(ExposureErrorV1::InvalidRevocationTime);
        }
        let authored = authored_at_unix_ms.to_be_bytes();
        let effective = effective_at_unix_ms.to_be_bytes();
        let id = StableIdV1::derive(
            REVOCATION_DOMAIN_V1,
            SCHEMA_VERSION_V1,
            &[
                &grant_id.0,
                &issuer.0,
                &authored,
                &effective,
                assurance.tag(),
            ],
        )?;
        Ok(Self {
            id,
            grant_id,
            issuer,
            authored_at_unix_ms,
            effective_at_unix_ms,
            assurance,
        })
    }

    pub fn id(&self) -> StableIdV1 {
        self.id
    }

    pub fn grant_id(&self) -> StableIdV1 {
        self.grant_id
    }

    pub fn issuer(&self) -> PartyIdV1 {
        self.issuer
    }

    pub fn authored_at_unix_ms(&self) -> u64 {
        self.authored_at_unix_ms
    }

    pub fn effective_at_unix_ms(&self) -> u64 {
        self.effective_at_unix_ms
    }

    pub fn assurance(&self) -> ExposureAssuranceV1 {
        self.assurance
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RemoteExposurePolicyV1 {
    id: StableIdV1,
    endpoint: ExposureEndpointIdV1,
    required_assurance: ExposureAssuranceV1,
    max_grant_lifetime_ms: u64,
    max_evidence_age_ms: u64,
    require_complete_coverage: bool,
    allow_public: bool,
    allow_principal: bool,
    allow_group: bool,
}

impl RemoteExposurePolicyV1 {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        endpoint: ExposureEndpointIdV1,
        required_assurance: ExposureAssuranceV1,
        max_grant_lifetime_ms: u64,
        max_evidence_age_ms: u64,
        require_complete_coverage: bool,
        allow_public: bool,
        allow_principal: bool,
        allow_group: bool,
    ) -> Result<Self, ExposureErrorV1> {
        if endpoint.stable_id() == StableIdV1::ZERO {
            return Err(ExposureErrorV1::ZeroEndpoint);
        }
        if endpoint.authority() == PartyIdV1::ZERO {
            return Err(ExposureErrorV1::ZeroAuthority);
        }
        if max_grant_lifetime_ms == 0 || max_evidence_age_ms == 0 {
            return Err(ExposureErrorV1::InvalidPolicyTiming);
        }
        if !(allow_public || allow_principal || allow_group) {
            return Err(ExposureErrorV1::NoAllowedAudience);
        }
        let grant_lifetime = max_grant_lifetime_ms.to_be_bytes();
        let evidence_age = max_evidence_age_ms.to_be_bytes();
        let flags = [
            u8::from(require_complete_coverage),
            u8::from(allow_public),
            u8::from(allow_principal),
            u8::from(allow_group),
        ];
        let endpoint_id = endpoint.stable_id();
        let authority = endpoint.authority();
        let id = StableIdV1::derive(
            POLICY_DOMAIN_V1,
            SCHEMA_VERSION_V1,
            &[
                &authority.0,
                &endpoint_id.0,
                required_assurance.tag(),
                &grant_lifetime,
                &evidence_age,
                &flags,
            ],
        )?;
        Ok(Self {
            id,
            endpoint,
            required_assurance,
            max_grant_lifetime_ms,
            max_evidence_age_ms,
            require_complete_coverage,
            allow_public,
            allow_principal,
            allow_group,
        })
    }

    pub fn strict(
        endpoint: ExposureEndpointIdV1,
        max_grant_lifetime_ms: u64,
        max_evidence_age_ms: u64,
    ) -> Result<Self, ExposureErrorV1> {
        Self::new(
            endpoint,
            ExposureAssuranceV1::CryptographicallyVerified,
            max_grant_lifetime_ms,
            max_evidence_age_ms,
            true,
            true,
            true,
            true,
        )
    }

    pub fn id(&self) -> StableIdV1 {
        self.id
    }

    pub fn endpoint(&self) -> ExposureEndpointIdV1 {
        self.endpoint
    }

    pub fn authority(&self) -> PartyIdV1 {
        self.endpoint.authority()
    }

    pub fn required_assurance(&self) -> ExposureAssuranceV1 {
        self.required_assurance
    }

    pub(crate) fn max_grant_lifetime_ms(&self) -> u64 {
        self.max_grant_lifetime_ms
    }

    pub(crate) fn max_evidence_age_ms(&self) -> u64 {
        self.max_evidence_age_ms
    }

    pub(crate) fn require_complete_coverage(&self) -> bool {
        self.require_complete_coverage
    }

    pub(crate) fn allows_audience(&self, audience: &ExposureAudienceV1) -> bool {
        match audience {
            ExposureAudienceV1::Public => self.allow_public,
            ExposureAudienceV1::AuthenticatedPrincipal(_) => self.allow_principal,
            ExposureAudienceV1::AuthenticatedGroup(_) => self.allow_group,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GrantExclusionReasonV1 {
    AuthorityMismatch,
    EndpointMismatch,
    AssuranceTooLow,
    GrantLifetimeTooLong,
    EvidenceTooOld,
    NotYetValid,
    Expired,
    AudienceDisallowed,
    CatalogEntryMissing,
    CatalogObjectChanged,
    Revoked,
    RevocationUnresolved,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RevocationIgnoreReasonV1 {
    UnknownGrant,
    IssuerMismatch,
    PredatesGrant,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RemoteExposureDiagnosticV1 {
    GrantExcluded {
        grant_id: StableIdV1,
        reason: GrantExclusionReasonV1,
    },
    RevocationIgnored {
        revocation_id: StableIdV1,
        reason: RevocationIgnoreReasonV1,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RemoteExposureEntryV1 {
    entry: NixCacheEntryV1,
    object_id: NixExposureObjectIdV1,
    audiences: Vec<ExposureAudienceV1>,
    grant_ids: Vec<StableIdV1>,
}

impl RemoteExposureEntryV1 {
    pub fn entry(&self) -> &NixCacheEntryV1 {
        &self.entry
    }

    pub fn object_id(&self) -> NixExposureObjectIdV1 {
        self.object_id
    }

    pub fn audiences(&self) -> &[ExposureAudienceV1] {
        &self.audiences
    }

    pub fn grant_ids(&self) -> &[StableIdV1] {
        &self.grant_ids
    }

    pub(crate) fn allows(&self, reader: &RemoteReaderV1) -> bool {
        self.audiences.iter().any(|audience| reader.allows(audience))
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RemoteExposureSnapshotV1 {
    id: StableIdV1,
    policy_id: StableIdV1,
    endpoint: ExposureEndpointIdV1,
    evaluation_time_unix_ms: u64,
    coverage: ExposureEvidenceCoverageV1,
    entries: Vec<RemoteExposureEntryV1>,
    diagnostics: Vec<RemoteExposureDiagnosticV1>,
}

impl RemoteExposureSnapshotV1 {
    pub(crate) fn new(
        policy: &RemoteExposurePolicyV1,
        evaluation_time_unix_ms: u64,
        coverage: ExposureEvidenceCoverageV1,
        entries: Vec<RemoteExposureEntryV1>,
        diagnostics: Vec<RemoteExposureDiagnosticV1>,
    ) -> Result<Self, ExposureErrorV1> {
        let evaluation = evaluation_time_unix_ms.to_be_bytes();
        let mut fields = vec![
            policy.id().0.to_vec(),
            policy.authority().0.to_vec(),
            evaluation.to_vec(),
            coverage.tag().to_vec(),
            (entries.len() as u64).to_be_bytes().to_vec(),
        ];
        for exposed in &entries {
            fields.push(exposed.entry.store_path().hash().as_str().as_bytes().to_vec());
            fields.push(exposed.object_id.stable_id().0.to_vec());
            fields.push((exposed.audiences.len() as u64).to_be_bytes().to_vec());
            for audience in &exposed.audiences {
                let (tag, id) = audience.canonical_parts();
                fields.push(tag.to_vec());
                fields.push(id.to_vec());
            }
            fields.push((exposed.grant_ids.len() as u64).to_be_bytes().to_vec());
            fields.extend(exposed.grant_ids.iter().map(|id| id.0.to_vec()));
        }
        let refs = fields.iter().map(Vec::as_slice).collect::<Vec<_>>();
        let id = StableIdV1::derive(SNAPSHOT_DOMAIN_V1, SCHEMA_VERSION_V1, &refs)?;
        Ok(Self {
            id,
            policy_id: policy.id(),
            endpoint: policy.endpoint(),
            evaluation_time_unix_ms,
            coverage,
            entries,
            diagnostics,
        })
    }

    pub fn id(&self) -> StableIdV1 {
        self.id
    }

    pub fn policy_id(&self) -> StableIdV1 {
        self.policy_id
    }

    pub fn endpoint(&self) -> ExposureEndpointIdV1 {
        self.endpoint
    }

    pub fn authority(&self) -> PartyIdV1 {
        self.endpoint.authority()
    }

    pub fn evaluation_time_unix_ms(&self) -> u64 {
        self.evaluation_time_unix_ms
    }

    pub fn coverage(&self) -> ExposureEvidenceCoverageV1 {
        self.coverage
    }

    pub fn entries(&self) -> &[RemoteExposureEntryV1] {
        &self.entries
    }

    pub fn diagnostics(&self) -> &[RemoteExposureDiagnosticV1] {
        &self.diagnostics
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn entry_for_reader(
        &self,
        store_hash: &NixStoreHashV1,
        reader: &RemoteReaderV1,
    ) -> Option<&NixCacheEntryV1> {
        self.entries
            .binary_search_by(|entry| entry.entry.store_path().hash().cmp(store_hash))
            .ok()
            .and_then(|index| {
                let exposed = &self.entries[index];
                exposed.allows(reader).then_some(exposed.entry())
            })
    }
}

pub(crate) fn make_exposed_entry(
    entry: NixCacheEntryV1,
    object_id: NixExposureObjectIdV1,
    audiences: BTreeSet<ExposureAudienceV1>,
    grant_ids: BTreeSet<StableIdV1>,
) -> RemoteExposureEntryV1 {
    RemoteExposureEntryV1 {
        entry,
        object_id,
        audiences: audiences.into_iter().collect(),
        grant_ids: grant_ids.into_iter().collect(),
    }
}

fn is_valid_endpoint_label(label: &str) -> bool {
    let bytes = label.as_bytes();
    !bytes.is_empty()
        && bytes.len() <= MAX_ENDPOINT_LABEL_LEN_V1
        && bytes[0].is_ascii_lowercase()
        && bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'-' | b'_' | b'.' | b':')
        })
}
