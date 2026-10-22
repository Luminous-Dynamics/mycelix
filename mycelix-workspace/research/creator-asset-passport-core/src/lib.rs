//! Experimental creator asset identity, split-claim, and explicit-use policy core.
//!
//! This crate checks structural invariants only. It is not a rights registry,
//! identity provider, signature verifier, legal opinion, or proof of ownership.

pub mod history;
pub mod import_ledger;
pub mod passport_envelope;
pub mod portable;
pub mod provenance;

use std::collections::BTreeMap;
use std::error::Error;
use std::fmt;

const MAX_ID_LENGTH: usize = 256;
const TOTAL_BASIS_POINTS: u32 = 10_000;

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PassportError {
    EmptyId,
    IdTooLong,
    InvalidIdCharacter,
    DuplicateAssetId,
    AssetNotFound,
    InvalidIswcSyntax,
    InvalidIsrcSyntax,
    InvalidOtherIdentifier,
    IdentifierIncompatibleWithAssetKind,
    DuplicateExternalIdentifier,
    EmptyEvidenceReference,
    InvalidEvidenceReference,
    EvidenceReferenceTooLong,
    EmptyPurpose,
    InvalidShareBasisPoints,
    DuplicatePartyInRightsPool,
    ClaimAssetOrPoolMismatch,
    SplitExceedsOneHundredPercent,
    CompleteSplitNotExactlyOneHundredPercent,
    CompleteSplitHasUnknownShares,
    KnownSharesExhaustPoolWithUnknownClaims,
    DisputedClaims,
    UnconfirmedClaims,
    InvalidGrantWindow,
    RevocationBeforeEffectiveTime,
    GrantAlreadyRevoked,
}

impl fmt::Display for PassportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl Error for PassportError {}

fn validate_token(value: &str) -> Result<String, PassportError> {
    let value = value.trim();
    if value.is_empty() {
        return Err(PassportError::EmptyId);
    }
    if value.len() > MAX_ID_LENGTH {
        return Err(PassportError::IdTooLong);
    }
    if value.chars().any(char::is_control) {
        return Err(PassportError::InvalidIdCharacter);
    }
    Ok(value.to_owned())
}

macro_rules! string_id {
    ($name:ident) => {
        #[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
        pub struct $name(String);

        impl $name {
            pub fn new(value: impl AsRef<str>) -> Result<Self, PassportError> {
                Ok(Self(validate_token(value.as_ref())?))
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }
    };
}

string_id!(AssetId);
string_id!(PartyId);
string_id!(GrantId);
string_id!(EventId);

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum AssetKind {
    MusicalWork,
    SoundRecording,
    MusicVideo,
    Release,
    VisualAsset,
    LivePerformance,
}

/// Typed identifiers prevent a work identifier from being silently treated as
/// a recording identifier. Format checks here are syntactic only.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExternalIdentifier {
    Iswc(String),
    Isrc(String),
    Other { scheme: String, value: String },
}

impl ExternalIdentifier {
    fn validate_for_kind(&self, kind: AssetKind) -> Result<(), PassportError> {
        match self {
            Self::Iswc(value) => {
                // ISWC presentation separators are optional. This deliberately
                // checks shape only; it does not verify the check digit or assignment.
                let compact: String = value
                    .chars()
                    .filter(|c| *c != '-' && *c != '.')
                    .collect::<String>()
                    .to_ascii_uppercase();
                let formatted_ok = if value.contains('-') || value.contains('.') {
                    let bytes = value.as_bytes();
                    bytes.len() == 15
                        && bytes[0] == b'T'
                        && bytes[1] == b'-'
                        && bytes[2..5].iter().all(|b| b.is_ascii_digit())
                        && bytes[5] == b'.'
                        && bytes[6..9].iter().all(|b| b.is_ascii_digit())
                        && bytes[9] == b'.'
                        && bytes[10..13].iter().all(|b| b.is_ascii_digit())
                        && bytes[13] == b'-'
                        && bytes[14].is_ascii_digit()
                } else {
                    true
                };
                if !formatted_ok
                    || compact.len() != 11
                    || !compact.starts_with('T')
                    || !compact.as_bytes()[1..].iter().all(|b| b.is_ascii_digit())
                {
                    return Err(PassportError::InvalidIswcSyntax);
                }
                if kind != AssetKind::MusicalWork {
                    return Err(PassportError::IdentifierIncompatibleWithAssetKind);
                }
            }
            Self::Isrc(value) => {
                // ISRC shape: 2-letter country, 3 alphanumeric registrant,
                // 2-digit year of reference, and 5-digit designation.
                let compact: String = value
                    .chars()
                    .filter(|c| *c != '-')
                    .collect::<String>()
                    .to_ascii_uppercase();
                let bytes = compact.as_bytes();
                let separator_layout_ok = if value.contains('-') {
                    let groups: Vec<&str> = value.split('-').collect();
                    groups.len() == 4
                        && groups[0].len() == 2
                        && groups[1].len() == 3
                        && groups[2].len() == 2
                        && groups[3].len() == 5
                        && groups
                            .iter()
                            .all(|group| group.chars().all(|c| c.is_ascii_alphanumeric()))
                } else {
                    value.len() == 12
                };
                let valid = separator_layout_ok
                    && bytes.len() == 12
                    && bytes[0..2].iter().all(|b| b.is_ascii_alphabetic())
                    && bytes[2..5].iter().all(|b| b.is_ascii_alphanumeric())
                    && bytes[5..7].iter().all(|b| b.is_ascii_digit())
                    && bytes[7..12].iter().all(|b| b.is_ascii_digit());
                if !valid {
                    return Err(PassportError::InvalidIsrcSyntax);
                }
                if !matches!(kind, AssetKind::SoundRecording | AssetKind::MusicVideo) {
                    return Err(PassportError::IdentifierIncompatibleWithAssetKind);
                }
            }
            Self::Other { scheme, value } => {
                let scheme = scheme.trim();
                let value = value.trim();
                if scheme.is_empty()
                    || value.is_empty()
                    || scheme.len() > MAX_ID_LENGTH
                    || value.len() > MAX_ID_LENGTH
                    || scheme.chars().any(char::is_control)
                    || value.chars().any(char::is_control)
                {
                    return Err(PassportError::InvalidOtherIdentifier);
                }
            }
        }
        Ok(())
    }

    fn canonical_key(&self) -> String {
        match self {
            Self::Iswc(value) => format!(
                "iswc:{}",
                value
                    .chars()
                    .filter(|c| *c != '-' && *c != '.')
                    .collect::<String>()
                    .to_ascii_uppercase()
            ),
            Self::Isrc(value) => format!(
                "isrc:{}",
                value
                    .chars()
                    .filter(|c| *c != '-')
                    .collect::<String>()
                    .to_ascii_uppercase()
            ),
            Self::Other { scheme, value } => {
                format!(
                    "other:{}:{}",
                    scheme.trim().to_ascii_lowercase(),
                    value.trim()
                )
            }
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CreativeAsset {
    pub id: AssetId,
    pub kind: AssetKind,
    pub title: Option<String>,
    identifiers: Vec<ExternalIdentifier>,
}

impl CreativeAsset {
    pub fn identifiers(&self) -> &[ExternalIdentifier] {
        &self.identifiers
    }
}

/// In-memory identity registry. Identifier collisions are surfaced, never
/// auto-merged; an external identifier does not establish ownership.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct AssetRegistry {
    assets: BTreeMap<AssetId, CreativeAsset>,
    identifier_owners: BTreeMap<String, AssetId>,
}

impl AssetRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn register_asset(
        &mut self,
        id: AssetId,
        kind: AssetKind,
        title: Option<String>,
    ) -> Result<(), PassportError> {
        if self.assets.contains_key(&id) {
            return Err(PassportError::DuplicateAssetId);
        }
        let title = title.and_then(|s| {
            let trimmed = s.trim();
            if trimmed.is_empty() {
                None
            } else {
                Some(trimmed.to_owned())
            }
        });
        self.assets.insert(
            id.clone(),
            CreativeAsset {
                id,
                kind,
                title,
                identifiers: Vec::new(),
            },
        );
        Ok(())
    }

    pub fn attach_identifier(
        &mut self,
        asset_id: &AssetId,
        identifier: ExternalIdentifier,
    ) -> Result<(), PassportError> {
        let asset = self
            .assets
            .get(asset_id)
            .ok_or(PassportError::AssetNotFound)?;
        identifier.validate_for_kind(asset.kind)?;
        let key = identifier.canonical_key();

        if let Some(existing_owner) = self.identifier_owners.get(&key) {
            if existing_owner == asset_id {
                // Replaying the same identifier assignment is idempotent.
                return Ok(());
            }
            return Err(PassportError::DuplicateExternalIdentifier);
        }

        self.assets
            .get_mut(asset_id)
            .expect("asset existence was checked above")
            .identifiers
            .push(identifier);
        self.identifier_owners.insert(key, asset_id.clone());
        Ok(())
    }

    pub fn get(&self, asset_id: &AssetId) -> Option<&CreativeAsset> {
        self.assets.get(asset_id)
    }

    /// Deterministic asset iteration in ascending AssetId order.
    pub fn assets(&self) -> impl Iterator<Item = &CreativeAsset> {
        self.assets.values()
    }

    /// Put each asset's identifier list into stable scheme/value order.
    /// This changes representation order only; it does not merge identifiers
    /// or change their recorded values.
    pub fn canonicalize_identifier_order(&mut self) {
        for asset in self.assets.values_mut() {
            asset
                .identifiers
                .sort_by_key(|identifier| match identifier {
                    ExternalIdentifier::Iswc(value) => (1u8, value.clone(), String::new()),
                    ExternalIdentifier::Isrc(value) => (2u8, value.clone(), String::new()),
                    ExternalIdentifier::Other { scheme, value } => {
                        (3u8, scheme.to_ascii_lowercase(), value.clone())
                    }
                });
        }
    }

    pub fn len(&self) -> usize {
        self.assets.len()
    }

    pub fn is_empty(&self) -> bool {
        self.assets.is_empty()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum RightsPool {
    Composition,
    MasterRecording,
    PerformerRights,
    Synchronization,
    VisualAsset,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClaimState {
    Proposed,
    Confirmed,
    Disputed,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SplitClaim {
    pub asset_id: AssetId,
    pub party_id: PartyId,
    pub pool: RightsPool,
    /// None means the share is unknown; it must not be imputed or normalized.
    pub basis_points: Option<u16>,
    pub state: ClaimState,
    pub evidence_ref: String,
}

impl SplitClaim {
    pub fn new(
        asset_id: AssetId,
        party_id: PartyId,
        pool: RightsPool,
        basis_points: Option<u16>,
        state: ClaimState,
        evidence_ref: impl AsRef<str>,
    ) -> Result<Self, PassportError> {
        let evidence_ref = evidence_ref.as_ref().trim();
        if evidence_ref.is_empty() {
            return Err(PassportError::EmptyEvidenceReference);
        }
        if evidence_ref.len() > MAX_ID_LENGTH {
            return Err(PassportError::EvidenceReferenceTooLong);
        }
        if evidence_ref.chars().any(char::is_control) {
            return Err(PassportError::InvalidEvidenceReference);
        }
        if basis_points.is_some_and(|bp| bp == 0 || u32::from(bp) > TOTAL_BASIS_POINTS) {
            return Err(PassportError::InvalidShareBasisPoints);
        }
        Ok(Self {
            asset_id,
            party_id,
            pool,
            basis_points,
            state,
            evidence_ref: evidence_ref.to_owned(),
        })
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SplitCompleteness {
    /// Calculate what is known without claiming the pool has been fully assigned.
    Partial,
    /// Caller asserts the listed claims are a complete split for this one pool.
    ClaimedComplete,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SplitAssessment {
    CompleteClaimConsistent {
        total_basis_points: u16,
    },
    Partial {
        known_basis_points: u32,
        residual_basis_points_not_attributed: u32,
        unknown_share_claims: usize,
    },
    Unconfirmed {
        known_basis_points: u32,
    },
    Disputed {
        known_basis_points: u32,
    },
}

/// Validate one asset + one rights pool at a time. A consistent complete claim
/// is not proof of legal ownership; it only means the stated arithmetic and
/// workflow statuses are internally consistent.
pub fn assess_split(
    asset_id: &AssetId,
    pool: RightsPool,
    claims: &[SplitClaim],
    completeness: SplitCompleteness,
) -> Result<SplitAssessment, PassportError> {
    let mut parties = std::collections::BTreeSet::new();
    let mut known_basis_points: u32 = 0;
    let mut unknown_share_claims = 0usize;
    let mut has_unconfirmed = false;
    let mut has_dispute = false;

    for claim in claims {
        if &claim.asset_id != asset_id || claim.pool != pool {
            return Err(PassportError::ClaimAssetOrPoolMismatch);
        }
        if claim.evidence_ref.trim().is_empty() {
            return Err(PassportError::EmptyEvidenceReference);
        }
        if claim.evidence_ref.len() > MAX_ID_LENGTH {
            return Err(PassportError::EvidenceReferenceTooLong);
        }
        if claim.evidence_ref.chars().any(char::is_control) {
            return Err(PassportError::InvalidEvidenceReference);
        }
        if !parties.insert(claim.party_id.clone()) {
            return Err(PassportError::DuplicatePartyInRightsPool);
        }
        if let Some(bp) = claim.basis_points {
            if bp == 0 || u32::from(bp) > TOTAL_BASIS_POINTS {
                return Err(PassportError::InvalidShareBasisPoints);
            }
            known_basis_points += u32::from(bp);
            // Reject as soon as the pool exceeds its legal arithmetic bound.
            // This also prevents adversarially large claim lists from overflowing
            // the accumulator before the post-loop validation can run.
            if known_basis_points > TOTAL_BASIS_POINTS {
                return Err(PassportError::SplitExceedsOneHundredPercent);
            }
        } else {
            unknown_share_claims += 1;
        }
        has_unconfirmed |= claim.state == ClaimState::Proposed;
        has_dispute |= claim.state == ClaimState::Disputed;
    }

    if has_dispute {
        return Ok(SplitAssessment::Disputed { known_basis_points });
    }
    if has_unconfirmed {
        return Ok(SplitAssessment::Unconfirmed { known_basis_points });
    }

    match completeness {
        SplitCompleteness::ClaimedComplete => {
            if unknown_share_claims > 0 {
                return Err(PassportError::CompleteSplitHasUnknownShares);
            }
            if known_basis_points != TOTAL_BASIS_POINTS {
                return Err(PassportError::CompleteSplitNotExactlyOneHundredPercent);
            }
            Ok(SplitAssessment::CompleteClaimConsistent {
                total_basis_points: TOTAL_BASIS_POINTS as u16,
            })
        }
        SplitCompleteness::Partial => {
            if known_basis_points == TOTAL_BASIS_POINTS && unknown_share_claims > 0 {
                return Err(PassportError::KnownSharesExhaustPoolWithUnknownClaims);
            }
            Ok(SplitAssessment::Partial {
                known_basis_points,
                residual_basis_points_not_attributed: TOTAL_BASIS_POINTS - known_basis_points,
                unknown_share_claims,
            })
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum UseAction {
    AnalyzeMetadata,
    Publish,
    Distribute,
    Commercialize,
    TrainModel,
    GenerateDerivative,
    SynthesizeVoiceOrLikeness,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct UsageGrant {
    id: GrantId,
    asset_id: AssetId,
    grantee_id: PartyId,
    action: UseAction,
    purpose: String,
    issued_by: PartyId,
    evidence_ref: String,
    valid_from: u64,
    expires_at: Option<u64>,
    revoked_at: Option<u64>,
}

impl UsageGrant {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: GrantId,
        asset_id: AssetId,
        grantee_id: PartyId,
        action: UseAction,
        purpose: impl AsRef<str>,
        issued_by: PartyId,
        evidence_ref: impl AsRef<str>,
        valid_from: u64,
        expires_at: Option<u64>,
    ) -> Result<Self, PassportError> {
        let purpose = purpose.as_ref().trim();
        let evidence_ref = evidence_ref.as_ref().trim();
        if purpose.is_empty() {
            return Err(PassportError::EmptyPurpose);
        }
        if evidence_ref.is_empty() {
            return Err(PassportError::EmptyEvidenceReference);
        }
        if evidence_ref.len() > MAX_ID_LENGTH {
            return Err(PassportError::EvidenceReferenceTooLong);
        }
        if evidence_ref.chars().any(char::is_control) {
            return Err(PassportError::InvalidEvidenceReference);
        }
        if purpose.len() > MAX_ID_LENGTH || purpose.chars().any(char::is_control) {
            return Err(PassportError::EmptyPurpose);
        }
        if expires_at.is_some_and(|expires| expires <= valid_from) {
            return Err(PassportError::InvalidGrantWindow);
        }
        Ok(Self {
            id,
            asset_id,
            grantee_id,
            action,
            purpose: purpose.to_owned(),
            issued_by,
            evidence_ref: evidence_ref.to_owned(),
            valid_from,
            expires_at,
            revoked_at: None,
        })
    }

    pub fn id(&self) -> &GrantId {
        &self.id
    }

    pub fn asset_id(&self) -> &AssetId {
        &self.asset_id
    }

    pub fn grantee_id(&self) -> &PartyId {
        &self.grantee_id
    }

    pub fn action(&self) -> UseAction {
        self.action
    }

    pub fn purpose(&self) -> &str {
        &self.purpose
    }

    pub fn issued_by(&self) -> &PartyId {
        &self.issued_by
    }

    pub fn evidence_ref(&self) -> &str {
        &self.evidence_ref
    }

    pub fn valid_from(&self) -> u64 {
        self.valid_from
    }

    pub fn expires_at(&self) -> Option<u64> {
        self.expires_at
    }

    pub fn revocation_timestamp(&self) -> Option<u64> {
        self.revoked_at
    }

    /// Compare immutable grant terms while treating revocation as a later
    /// history event. A repeated grant ID may not silently change its scope.
    fn same_terms_as(&self, other: &Self) -> bool {
        self.id == other.id
            && self.asset_id == other.asset_id
            && self.grantee_id == other.grantee_id
            && self.action == other.action
            && self.purpose == other.purpose
            && self.issued_by == other.issued_by
            && self.evidence_ref == other.evidence_ref
            && self.valid_from == other.valid_from
            && self.expires_at == other.expires_at
    }

    /// Returns a new revocation record. The original value remains unchanged so
    /// callers can retain a history; no signature or grantor authority is checked.
    pub fn revoked_at(&self, timestamp: u64) -> Result<Self, PassportError> {
        if timestamp < self.valid_from {
            return Err(PassportError::RevocationBeforeEffectiveTime);
        }
        if self.revoked_at.is_some() {
            return Err(PassportError::GrantAlreadyRevoked);
        }
        let mut updated = self.clone();
        updated.revoked_at = Some(timestamp);
        Ok(updated)
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct UseRequest {
    pub asset_id: AssetId,
    pub grantee_id: PartyId,
    pub action: UseAction,
    pub purpose: String,
    pub at: u64,
}

impl UseRequest {
    pub fn new(
        asset_id: AssetId,
        grantee_id: PartyId,
        action: UseAction,
        purpose: impl AsRef<str>,
        at: u64,
    ) -> Result<Self, PassportError> {
        let purpose = purpose.as_ref().trim();
        if purpose.is_empty()
            || purpose.len() > MAX_ID_LENGTH
            || purpose.chars().any(char::is_control)
        {
            return Err(PassportError::EmptyPurpose);
        }
        Ok(Self {
            asset_id,
            grantee_id,
            action,
            purpose: purpose.to_owned(),
            at,
        })
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PermissionDenial {
    AssetNotRegistered,
    NoExplicitGrant,
    NoActiveGrantForExactScope,
    ConflictingGrantId,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PermissionDecision {
    Allowed { grant_id: GrantId },
    Denied(PermissionDenial),
}

fn matches_requested_scope(grant: &UsageGrant, request: &UseRequest) -> bool {
    grant.asset_id == request.asset_id
        && grant.grantee_id == request.grantee_id
        && grant.action == request.action
        && grant.purpose == request.purpose
}

/// Evaluate a request against explicit, asset-bound grants. No permission is
/// inherited from a related asset, credit, split claim, or external identifier.
pub fn evaluate_usage(
    registry: &AssetRegistry,
    grants: &[UsageGrant],
    request: &UseRequest,
) -> PermissionDecision {
    if registry.get(&request.asset_id).is_none() {
        return PermissionDecision::Denied(PermissionDenial::AssetNotRegistered);
    }

    // Resolve history by immutable grant ID before authorizing. An original
    // grant and its revocation may both be present in an append-only history;
    // revocation must dominate regardless of input order. Conflicts block only
    // a request whose exact scope is implicated, preserving isolation between
    // unrelated assets and parties rather than turning one bad record into a
    // catalogue-wide denial of service.
    let mut effective: BTreeMap<GrantId, &UsageGrant> = BTreeMap::new();
    let mut conflicting_exact_scope = false;

    for grant in grants {
        let should_replace = match effective.get(&grant.id) {
            None => true,
            Some(existing) => {
                if !existing.same_terms_as(grant) {
                    conflicting_exact_scope |= matches_requested_scope(existing, request)
                        || matches_requested_scope(grant, request);
                    false
                } else {
                    match (existing.revoked_at, grant.revoked_at) {
                        (Some(previous), Some(next)) if previous != next => {
                            conflicting_exact_scope |= matches_requested_scope(existing, request);
                            false
                        }
                        (None, Some(_)) => true,
                        _ => false,
                    }
                }
            }
        };

        // Mutate only after the immutable lookup borrow has ended.
        if should_replace {
            effective.insert(grant.id.clone(), grant);
        }
    }

    if conflicting_exact_scope {
        return PermissionDecision::Denied(PermissionDenial::ConflictingGrantId);
    }

    let mut has_exact_scope = false;
    for grant in effective.values() {
        if !matches_requested_scope(grant, request) {
            continue;
        }
        has_exact_scope = true;

        if request.at < grant.valid_from {
            continue;
        }

        let revoked = grant.revoked_at.filter(|time| request.at >= *time);
        let expired = grant.expires_at.filter(|time| request.at >= *time);
        if revoked.is_some() || expired.is_some() {
            continue;
        }

        return PermissionDecision::Allowed {
            grant_id: grant.id.clone(),
        };
    }

    if has_exact_scope {
        PermissionDecision::Denied(PermissionDenial::NoActiveGrantForExactScope)
    } else {
        PermissionDecision::Denied(PermissionDenial::NoExplicitGrant)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: &str) -> AssetId {
        AssetId::new(value).unwrap()
    }

    fn party(value: &str) -> PartyId {
        PartyId::new(value).unwrap()
    }

    fn composition_claim(
        asset_id: &AssetId,
        participant: &str,
        basis_points: Option<u16>,
        state: ClaimState,
        evidence_ref: &str,
    ) -> SplitClaim {
        SplitClaim::new(
            asset_id.clone(),
            party(participant),
            RightsPool::Composition,
            basis_points,
            state,
            evidence_ref,
        )
        .unwrap()
    }

    fn make_request(
        asset_id: AssetId,
        grantee: &str,
        action: UseAction,
        purpose: &str,
        at: u64,
    ) -> UseRequest {
        UseRequest::new(asset_id, party(grantee), action, purpose, at).unwrap()
    }

    #[test]
    fn iswc_identifier_is_syntactic_and_work_scoped() {
        let mut registry = AssetRegistry::new();
        let work = id("work-1");
        registry
            .register_asset(work.clone(), AssetKind::MusicalWork, Some("Example".into()))
            .unwrap();
        registry
            .attach_identifier(&work, ExternalIdentifier::Iswc("T-034.524.680-1".into()))
            .unwrap();
        assert_eq!(registry.get(&work).unwrap().identifiers().len(), 1);
    }

    #[test]
    fn malformed_iswc_is_rejected() {
        let mut registry = AssetRegistry::new();
        let work = id("work-1");
        registry
            .register_asset(work.clone(), AssetKind::MusicalWork, None)
            .unwrap();
        assert_eq!(
            registry.attach_identifier(&work, ExternalIdentifier::Iswc("not-an-iswc".into())),
            Err(PassportError::InvalidIswcSyntax)
        );
    }

    #[test]
    fn iswc_does_not_attach_to_a_recording() {
        let mut registry = AssetRegistry::new();
        let recording = id("recording-1");
        registry
            .register_asset(recording.clone(), AssetKind::SoundRecording, None)
            .unwrap();
        assert_eq!(
            registry.attach_identifier(
                &recording,
                ExternalIdentifier::Iswc("T-034.524.680-1".into())
            ),
            Err(PassportError::IdentifierIncompatibleWithAssetKind)
        );
    }

    #[test]
    fn isrc_does_not_attach_to_a_composition() {
        let mut registry = AssetRegistry::new();
        let work = id("work-1");
        registry
            .register_asset(work.clone(), AssetKind::MusicalWork, None)
            .unwrap();
        assert_eq!(
            registry.attach_identifier(&work, ExternalIdentifier::Isrc("USABC2300001".into())),
            Err(PassportError::IdentifierIncompatibleWithAssetKind)
        );
    }

    #[test]
    fn identifier_assignment_replay_is_idempotent_but_cross_asset_collision_fails() {
        let mut registry = AssetRegistry::new();
        let first = id("recording-1");
        let second = id("recording-2");
        registry
            .register_asset(first.clone(), AssetKind::SoundRecording, None)
            .unwrap();
        registry
            .register_asset(second.clone(), AssetKind::SoundRecording, None)
            .unwrap();
        let isrc = ExternalIdentifier::Isrc("US-ABC-23-00001".into());
        registry.attach_identifier(&first, isrc.clone()).unwrap();
        registry.attach_identifier(&first, isrc.clone()).unwrap();
        assert_eq!(registry.get(&first).unwrap().identifiers().len(), 1);
        assert_eq!(
            registry.attach_identifier(&second, isrc),
            Err(PassportError::DuplicateExternalIdentifier)
        );
    }

    #[test]
    fn complete_claim_requires_exact_100_percent() {
        let asset = id("work-1");
        let claims = vec![
            composition_claim(&asset, "a", Some(6000), ClaimState::Confirmed, "evidence:a"),
            composition_claim(&asset, "b", Some(4000), ClaimState::Confirmed, "evidence:b"),
        ];
        assert_eq!(
            assess_split(
                &asset,
                RightsPool::Composition,
                &claims,
                SplitCompleteness::ClaimedComplete,
            )
            .unwrap(),
            SplitAssessment::CompleteClaimConsistent {
                total_basis_points: 10_000
            }
        );
    }

    #[test]
    fn partial_split_reports_residual_without_normalizing() {
        let asset = id("work-1");
        let claims = vec![composition_claim(
            &asset,
            "a",
            Some(6000),
            ClaimState::Confirmed,
            "evidence:a",
        )];
        assert_eq!(
            assess_split(
                &asset,
                RightsPool::Composition,
                &claims,
                SplitCompleteness::Partial,
            )
            .unwrap(),
            SplitAssessment::Partial {
                known_basis_points: 6000,
                residual_basis_points_not_attributed: 4000,
                unknown_share_claims: 0,
            }
        );
    }

    #[test]
    fn complete_split_cannot_be_inferred_from_a_partial_declaration() {
        let asset = id("work-1");
        let claims = vec![composition_claim(
            &asset,
            "a",
            Some(10_000),
            ClaimState::Confirmed,
            "evidence:a",
        )];
        assert!(matches!(
            assess_split(
                &asset,
                RightsPool::Composition,
                &claims,
                SplitCompleteness::Partial,
            )
            .unwrap(),
            SplitAssessment::Partial { .. }
        ));
    }

    #[test]
    fn overallocated_split_fails_closed() {
        let asset = id("work-1");
        let claims = vec![
            composition_claim(&asset, "a", Some(6000), ClaimState::Confirmed, "evidence:a"),
            composition_claim(&asset, "b", Some(5000), ClaimState::Confirmed, "evidence:b"),
        ];
        assert_eq!(
            assess_split(
                &asset,
                RightsPool::Composition,
                &claims,
                SplitCompleteness::Partial
            ),
            Err(PassportError::SplitExceedsOneHundredPercent)
        );
    }

    #[test]
    fn complete_split_with_unknown_share_fails_closed() {
        let asset = id("work-1");
        let claims = vec![
            composition_claim(&asset, "a", Some(6000), ClaimState::Confirmed, "evidence:a"),
            composition_claim(&asset, "b", None, ClaimState::Confirmed, "evidence:b"),
        ];
        assert_eq!(
            assess_split(
                &asset,
                RightsPool::Composition,
                &claims,
                SplitCompleteness::ClaimedComplete,
            ),
            Err(PassportError::CompleteSplitHasUnknownShares)
        );
    }

    #[test]
    fn duplicate_party_claim_is_not_silently_added_twice() {
        let asset = id("work-1");
        let claims = vec![
            composition_claim(
                &asset,
                "a",
                Some(5000),
                ClaimState::Confirmed,
                "evidence:a1",
            ),
            composition_claim(
                &asset,
                "a",
                Some(5000),
                ClaimState::Confirmed,
                "evidence:a2",
            ),
        ];
        assert_eq!(
            assess_split(
                &asset,
                RightsPool::Composition,
                &claims,
                SplitCompleteness::ClaimedComplete,
            ),
            Err(PassportError::DuplicatePartyInRightsPool)
        );
    }

    #[test]
    fn disputed_and_proposed_claims_cannot_be_reported_as_confirmed() {
        let asset = id("work-1");
        let proposed = vec![composition_claim(
            &asset,
            "a",
            Some(10_000),
            ClaimState::Proposed,
            "evidence:a",
        )];
        assert!(matches!(
            assess_split(
                &asset,
                RightsPool::Composition,
                &proposed,
                SplitCompleteness::ClaimedComplete,
            )
            .unwrap(),
            SplitAssessment::Unconfirmed { .. }
        ));
        let disputed = vec![composition_claim(
            &asset,
            "a",
            Some(10_000),
            ClaimState::Disputed,
            "evidence:a",
        )];
        assert!(matches!(
            assess_split(
                &asset,
                RightsPool::Composition,
                &disputed,
                SplitCompleteness::ClaimedComplete,
            )
            .unwrap(),
            SplitAssessment::Disputed { .. }
        ));
    }

    #[test]
    fn claim_for_wrong_asset_or_pool_is_rejected() {
        let asset = id("work-1");
        let claims = vec![composition_claim(
            &id("recording-1"),
            "a",
            Some(10_000),
            ClaimState::Confirmed,
            "evidence:a",
        )];
        assert_eq!(
            assess_split(
                &asset,
                RightsPool::Composition,
                &claims,
                SplitCompleteness::ClaimedComplete,
            ),
            Err(PassportError::ClaimAssetOrPoolMismatch)
        );
    }

    #[test]
    fn permission_requires_registered_asset_and_exact_grant() {
        let mut registry = AssetRegistry::new();
        let asset = id("recording-1");
        registry
            .register_asset(asset.clone(), AssetKind::SoundRecording, None)
            .unwrap();
        let request = make_request(asset.clone(), "editor", UseAction::TrainModel, "demo", 50);
        assert_eq!(
            evaluate_usage(&registry, &[], &request),
            PermissionDecision::Denied(PermissionDenial::NoExplicitGrant)
        );
        let unknown = make_request(id("unknown"), "editor", UseAction::TrainModel, "demo", 50);
        assert_eq!(
            evaluate_usage(&registry, &[], &unknown),
            PermissionDecision::Denied(PermissionDenial::AssetNotRegistered)
        );
    }

    #[test]
    fn one_action_does_not_authorize_another() {
        let mut registry = AssetRegistry::new();
        let asset = id("recording-1");
        registry
            .register_asset(asset.clone(), AssetKind::SoundRecording, None)
            .unwrap();
        let grant = UsageGrant::new(
            GrantId::new("grant-1").unwrap(),
            asset.clone(),
            party("studio"),
            UseAction::AnalyzeMetadata,
            "release-review",
            party("creator"),
            "consent:1",
            0,
            None,
        )
        .unwrap();
        let request = make_request(asset, "studio", UseAction::TrainModel, "release-review", 1);
        assert_eq!(
            evaluate_usage(&registry, &[grant], &request),
            PermissionDecision::Denied(PermissionDenial::NoExplicitGrant)
        );
    }

    #[test]
    fn grant_is_exactly_bound_to_asset_party_action_and_purpose() {
        let mut registry = AssetRegistry::new();
        let asset = id("recording-1");
        let other_asset = id("recording-2");
        registry
            .register_asset(asset.clone(), AssetKind::SoundRecording, None)
            .unwrap();
        registry
            .register_asset(other_asset.clone(), AssetKind::SoundRecording, None)
            .unwrap();
        let grant = UsageGrant::new(
            GrantId::new("grant-1").unwrap(),
            asset.clone(),
            party("studio"),
            UseAction::Distribute,
            "single-release",
            party("creator"),
            "consent:1",
            10,
            Some(100),
        )
        .unwrap();
        let exact = make_request(
            asset.clone(),
            "studio",
            UseAction::Distribute,
            "single-release",
            20,
        );
        assert_eq!(
            evaluate_usage(&registry, std::slice::from_ref(&grant), &exact),
            PermissionDecision::Allowed {
                grant_id: GrantId::new("grant-1").unwrap()
            }
        );
        let wrong_purpose = make_request(
            asset.clone(),
            "studio",
            UseAction::Distribute,
            "model-training",
            20,
        );
        assert_eq!(
            evaluate_usage(&registry, std::slice::from_ref(&grant), &wrong_purpose),
            PermissionDecision::Denied(PermissionDenial::NoExplicitGrant)
        );
        let wrong_party = make_request(
            asset.clone(),
            "other-studio",
            UseAction::Distribute,
            "single-release",
            20,
        );
        assert_eq!(
            evaluate_usage(&registry, std::slice::from_ref(&grant), &wrong_party),
            PermissionDecision::Denied(PermissionDenial::NoExplicitGrant)
        );
        let wrong_asset = make_request(
            other_asset,
            "studio",
            UseAction::Distribute,
            "single-release",
            20,
        );
        assert_eq!(
            evaluate_usage(&registry, &[grant], &wrong_asset),
            PermissionDecision::Denied(PermissionDenial::NoExplicitGrant)
        );
    }

    #[test]
    fn grant_validity_window_is_start_inclusive_and_expiry_exclusive() {
        let mut registry = AssetRegistry::new();
        let asset = id("recording-1");
        registry
            .register_asset(asset.clone(), AssetKind::SoundRecording, None)
            .unwrap();
        let grant = UsageGrant::new(
            GrantId::new("grant-1").unwrap(),
            asset.clone(),
            party("studio"),
            UseAction::Publish,
            "campaign-a",
            party("creator"),
            "consent:1",
            10,
            Some(20),
        )
        .unwrap();
        let before = make_request(asset.clone(), "studio", UseAction::Publish, "campaign-a", 9);
        let start = make_request(
            asset.clone(),
            "studio",
            UseAction::Publish,
            "campaign-a",
            10,
        );
        let expiry = make_request(asset, "studio", UseAction::Publish, "campaign-a", 20);
        assert_eq!(
            evaluate_usage(&registry, std::slice::from_ref(&grant), &before),
            PermissionDecision::Denied(PermissionDenial::NoActiveGrantForExactScope)
        );
        assert!(matches!(
            evaluate_usage(&registry, std::slice::from_ref(&grant), &start),
            PermissionDecision::Allowed { .. }
        ));
        assert_eq!(
            evaluate_usage(&registry, &[grant], &expiry),
            PermissionDecision::Denied(PermissionDenial::NoActiveGrantForExactScope)
        );
    }

    #[test]
    fn revocation_stops_future_uses_but_not_historical_evaluation() {
        let mut registry = AssetRegistry::new();
        let asset = id("recording-1");
        registry
            .register_asset(asset.clone(), AssetKind::SoundRecording, None)
            .unwrap();
        let grant = UsageGrant::new(
            GrantId::new("grant-1").unwrap(),
            asset.clone(),
            party("studio"),
            UseAction::Publish,
            "campaign-a",
            party("creator"),
            "consent:1",
            10,
            None,
        )
        .unwrap();
        let revoked = grant.revoked_at(20).unwrap();
        let historical = make_request(
            asset.clone(),
            "studio",
            UseAction::Publish,
            "campaign-a",
            19,
        );
        let future = make_request(asset, "studio", UseAction::Publish, "campaign-a", 20);
        assert!(matches!(
            evaluate_usage(&registry, &[revoked.clone()], &historical),
            PermissionDecision::Allowed { .. }
        ));
        assert_eq!(
            evaluate_usage(&registry, &[revoked.clone()], &future),
            PermissionDecision::Denied(PermissionDenial::NoActiveGrantForExactScope)
        );

        // Historical exports can contain both the original grant and a later
        // revocation event. Reordering that history must not resurrect consent.
        assert_eq!(
            evaluate_usage(&registry, &[grant.clone(), revoked.clone()], &future),
            PermissionDecision::Denied(PermissionDenial::NoActiveGrantForExactScope)
        );
        assert_eq!(
            evaluate_usage(&registry, &[revoked.clone(), grant], &future),
            PermissionDecision::Denied(PermissionDenial::NoActiveGrantForExactScope)
        );
    }

    #[test]
    fn conflicting_reuse_of_grant_id_fails_closed() {
        let mut registry = AssetRegistry::new();
        let asset = id("recording-1");
        registry
            .register_asset(asset.clone(), AssetKind::SoundRecording, None)
            .unwrap();
        let base_id = GrantId::new("grant-reused").unwrap();
        let exact = UsageGrant::new(
            base_id.clone(),
            asset.clone(),
            party("studio"),
            UseAction::TrainModel,
            "model-a",
            party("creator"),
            "consent:a",
            0,
            None,
        )
        .unwrap();
        let conflicting = UsageGrant::new(
            base_id,
            asset.clone(),
            party("studio"),
            UseAction::TrainModel,
            "model-b",
            party("creator"),
            "consent:b",
            0,
            None,
        )
        .unwrap();
        let request = make_request(asset, "studio", UseAction::TrainModel, "model-a", 1);

        assert_eq!(
            evaluate_usage(&registry, &[exact, conflicting], &request),
            PermissionDecision::Denied(PermissionDenial::ConflictingGrantId)
        );
    }

    #[test]
    fn unrelated_conflicting_grant_id_does_not_block_an_exact_valid_permission() {
        let mut registry = AssetRegistry::new();
        let requested_asset = id("recording-requested");
        let unrelated_asset = id("recording-unrelated");
        registry
            .register_asset(requested_asset.clone(), AssetKind::SoundRecording, None)
            .unwrap();
        registry
            .register_asset(unrelated_asset.clone(), AssetKind::SoundRecording, None)
            .unwrap();

        let valid = UsageGrant::new(
            GrantId::new("valid-grant").unwrap(),
            requested_asset.clone(),
            party("studio"),
            UseAction::Publish,
            "campaign-a",
            party("creator"),
            "consent:valid",
            0,
            None,
        )
        .unwrap();

        let reused_id = GrantId::new("conflicting-unrelated-id").unwrap();
        let unrelated_a = UsageGrant::new(
            reused_id.clone(),
            unrelated_asset.clone(),
            party("studio"),
            UseAction::TrainModel,
            "model-a",
            party("creator"),
            "consent:a",
            0,
            None,
        )
        .unwrap();
        let unrelated_b = UsageGrant::new(
            reused_id,
            unrelated_asset,
            party("studio"),
            UseAction::TrainModel,
            "model-b",
            party("creator"),
            "consent:b",
            0,
            None,
        )
        .unwrap();

        let request = make_request(
            requested_asset,
            "studio",
            UseAction::Publish,
            "campaign-a",
            1,
        );
        assert_eq!(
            evaluate_usage(
                &registry,
                &[unrelated_a, valid.clone(), unrelated_b],
                &request
            ),
            PermissionDecision::Allowed {
                grant_id: valid.id().clone()
            }
        );
    }

    #[test]
    fn inconsistent_revocation_timestamps_fail_closed_for_the_affected_scope() {
        let mut registry = AssetRegistry::new();
        let asset = id("recording-1");
        registry
            .register_asset(asset.clone(), AssetKind::SoundRecording, None)
            .unwrap();
        let grant = UsageGrant::new(
            GrantId::new("grant-one-revocation").unwrap(),
            asset.clone(),
            party("studio"),
            UseAction::Publish,
            "campaign-a",
            party("creator"),
            "consent:1",
            0,
            None,
        )
        .unwrap();
        let revoked_at_20 = grant.revoked_at(20).unwrap();
        let revoked_at_30 = grant.revoked_at(30).unwrap();
        let request = make_request(asset, "studio", UseAction::Publish, "campaign-a", 10);

        assert_eq!(
            evaluate_usage(&registry, &[revoked_at_20, revoked_at_30], &request),
            PermissionDecision::Denied(PermissionDenial::ConflictingGrantId)
        );
    }

    #[test]
    fn invalid_expiry_and_early_revocation_are_rejected() {
        let asset = id("recording-1");
        let grant = UsageGrant::new(
            GrantId::new("grant-1").unwrap(),
            asset.clone(),
            party("studio"),
            UseAction::Publish,
            "campaign-a",
            party("creator"),
            "consent:1",
            10,
            None,
        )
        .unwrap();
        assert_eq!(
            UsageGrant::new(
                GrantId::new("bad-window").unwrap(),
                asset,
                party("studio"),
                UseAction::Publish,
                "campaign-a",
                party("creator"),
                "consent:1",
                10,
                Some(10),
            ),
            Err(PassportError::InvalidGrantWindow)
        );
        assert_eq!(
            grant.revoked_at(9),
            Err(PassportError::RevocationBeforeEffectiveTime)
        );
    }

    #[test]
    fn evidence_references_reject_control_characters_and_oversized_fields() {
        assert_eq!(
            SplitClaim::new(
                id("work"),
                party("writer"),
                RightsPool::Composition,
                Some(10_000),
                ClaimState::Confirmed,
                "evidence\nforged",
            ),
            Err(PassportError::InvalidEvidenceReference)
        );
        assert_eq!(
            SplitClaim::new(
                id("work"),
                party("writer"),
                RightsPool::Composition,
                Some(10_000),
                ClaimState::Confirmed,
                "e".repeat(257),
            ),
            Err(PassportError::EvidenceReferenceTooLong)
        );
        assert_eq!(
            UsageGrant::new(
                GrantId::new("g").unwrap(),
                id("asset"),
                party("grantee"),
                UseAction::Publish,
                "release",
                party("issuer"),
                "evidence\r\nforged",
                0,
                None,
            ),
            Err(PassportError::InvalidEvidenceReference)
        );
    }

    #[test]
    fn ids_and_evidence_cannot_be_blank() {
        assert_eq!(AssetId::new("  "), Err(PassportError::EmptyId));
        assert_eq!(
            SplitClaim::new(
                id("work"),
                party("writer"),
                RightsPool::Composition,
                Some(10000),
                ClaimState::Confirmed,
                " ",
            ),
            Err(PassportError::EmptyEvidenceReference)
        );
        assert_eq!(
            UsageGrant::new(
                GrantId::new("g").unwrap(),
                id("asset"),
                party("grantee"),
                UseAction::Publish,
                " ",
                party("issuer"),
                "evidence",
                0,
                None,
            ),
            Err(PassportError::EmptyPurpose)
        );
    }
}
