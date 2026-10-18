//! Portable, versioned snapshot of a creator asset passport.
//!
//! This envelope includes local asset records, recorded external identifiers,
//! split claims, and the grant/revocation history envelope. A canonical digest
//! proves byte identity only; it is not a signature, ownership certificate,
//! consent proof, or statement that evidence references are true.

use std::error::Error;
use std::fmt;

use crate::history::GrantHistory;
use crate::portable::{GrantHistoryEnvelope, PortableHistoryError};
use crate::{
    AssetId, AssetKind, AssetRegistry, ClaimState, ExternalIdentifier, PassportError, RightsPool,
    SplitClaim,
};

const MAGIC: &[u8] = b"MYCELIX-CSCP-ASSET-PASSPORT\0";
const FORMAT_VERSION: u16 = 1;
const MAX_FIELD_BYTES: usize = 256;
const MAX_ASSETS: u64 = 100_000;
const MAX_CLAIMS: u64 = 1_000_000;
const MAX_IDENTIFIERS: u64 = 1_000_000;
const MAX_ENVELOPE_BYTES: usize = 256 * 1024 * 1024;
const DIGEST_DOMAIN: &[u8] = b"MYCELIX-CSCP-ASSET-PASSPORT-DIGEST-V1\0";

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PassportEnvelopeError {
    BadMagic,
    UnsupportedVersion(u16),
    TruncatedEnvelope,
    TrailingBytes,
    NonCanonicalEncoding,
    FieldTooLong,
    InvalidUtf8,
    TooManyAssets,
    TooManyClaims,
    TooManyIdentifiers,
    EnvelopeTooLarge,
    InvalidAssetKind,
    InvalidIdentifierKind,
    InvalidRightsPool,
    InvalidClaimState,
    InvalidOptionTag,
    InvalidAssetTitle,
    InvalidClaimAsset,
    InvalidGrantAsset,
    InvalidDigestFormat,
    DigestMismatch,
    InvalidPassportField(PassportError),
    InvalidGrantHistory(PortableHistoryError),
}

impl fmt::Display for PassportEnvelopeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl Error for PassportEnvelopeError {}

impl From<PassportError> for PassportEnvelopeError {
    fn from(value: PassportError) -> Self {
        Self::InvalidPassportField(value)
    }
}

impl From<PortableHistoryError> for PassportEnvelopeError {
    fn from(value: PortableHistoryError) -> Self {
        Self::InvalidGrantHistory(value)
    }
}

/// A locally validated, portable snapshot. It keeps claims (including disputed
/// claims) rather than reducing them to an ownership decision.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CreativeAssetPassportEnvelope {
    version: u16,
    assets: AssetRegistry,
    claims: Vec<SplitClaim>,
    grant_history: GrantHistoryEnvelope,
}

impl CreativeAssetPassportEnvelope {
    pub fn from_parts(
        assets: &AssetRegistry,
        claims: &[SplitClaim],
        grant_history: &GrantHistory,
    ) -> Result<Self, PassportEnvelopeError> {
        let grant_history = GrantHistoryEnvelope::from_history(grant_history)?;
        let mut claims = claims.to_vec();
        sort_claims(&mut claims);
        let mut assets = assets.clone();
        assets.canonicalize_identifier_order();
        let envelope = Self {
            version: FORMAT_VERSION,
            assets,
            claims,
            grant_history,
        };
        envelope.validate()?;
        Ok(envelope)
    }

    fn validate(&self) -> Result<(), PassportEnvelopeError> {
        if self.version != FORMAT_VERSION {
            return Err(PassportEnvelopeError::UnsupportedVersion(self.version));
        }
        if self.assets.len() as u64 > MAX_ASSETS {
            return Err(PassportEnvelopeError::TooManyAssets);
        }
        if self.claims.len() as u64 > MAX_CLAIMS {
            return Err(PassportEnvelopeError::TooManyClaims);
        }

        let mut total_identifiers = 0u64;
        for asset in self.assets.assets() {
            validate_field(asset.id.as_str())?;
            total_identifiers = total_identifiers.saturating_add(asset.identifiers().len() as u64);
            if total_identifiers > MAX_IDENTIFIERS {
                return Err(PassportEnvelopeError::TooManyIdentifiers);
            }
            if let Some(title) = asset.title.as_deref() {
                if title.trim().is_empty()
                    || title.len() > MAX_FIELD_BYTES
                    || title.chars().any(char::is_control)
                {
                    return Err(PassportEnvelopeError::InvalidAssetTitle);
                }
            }
            for identifier in asset.identifiers() {
                validate_identifier_fields(identifier)?;
            }
        }

        let mut canonical_claims = self.claims.clone();
        sort_claims(&mut canonical_claims);
        if canonical_claims != self.claims {
            return Err(PassportEnvelopeError::NonCanonicalEncoding);
        }
        for claim in &self.claims {
            if self.assets.get(&claim.asset_id).is_none() {
                return Err(PassportEnvelopeError::InvalidClaimAsset);
            }
            let checked = SplitClaim::new(
                claim.asset_id.clone(),
                claim.party_id.clone(),
                claim.pool,
                claim.basis_points,
                claim.state,
                &claim.evidence_ref,
            )?;
            if &checked != claim {
                return Err(PassportEnvelopeError::NonCanonicalEncoding);
            }
        }

        // The embedded history envelope validates both its append-derived
        // state and the separate event-stream reconstruction path. Every grant
        // must resolve to a registered asset even if the grant is revoked.
        self.grant_history.canonical_bytes()?;
        for grant in self.grant_history.replay()?.effective_grants() {
            if self.assets.get(grant.asset_id()).is_none() {
                return Err(PassportEnvelopeError::InvalidGrantAsset);
            }
        }
        Ok(())
    }

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, PassportEnvelopeError> {
        if bytes.len() > MAX_ENVELOPE_BYTES {
            return Err(PassportEnvelopeError::EnvelopeTooLarge);
        }
        let mut reader = Reader::new(bytes);
        if reader.take(MAGIC.len())? != MAGIC {
            return Err(PassportEnvelopeError::BadMagic);
        }
        let version = reader.u16()?;
        if version != FORMAT_VERSION {
            return Err(PassportEnvelopeError::UnsupportedVersion(version));
        }

        let asset_count = reader.u64()?;
        if asset_count > MAX_ASSETS || asset_count > bytes.len() as u64 {
            return Err(PassportEnvelopeError::TooManyAssets);
        }
        let mut assets = AssetRegistry::new();
        let mut total_identifiers = 0u64;
        for _ in 0..asset_count {
            let id = AssetId::new(reader.string()?)?;
            let kind = decode_asset_kind(reader.u8()?)?;
            let title = match reader.u8()? {
                0 => None,
                1 => {
                    let title = reader.string()?;
                    if title.trim().is_empty() || title.chars().any(char::is_control) {
                        return Err(PassportEnvelopeError::InvalidAssetTitle);
                    }
                    Some(title)
                }
                _ => return Err(PassportEnvelopeError::InvalidOptionTag),
            };
            assets.register_asset(id.clone(), kind, title)?;

            let identifier_count = reader.u64()?;
            total_identifiers = total_identifiers.saturating_add(identifier_count);
            if identifier_count > MAX_IDENTIFIERS
                || total_identifiers > MAX_IDENTIFIERS
                || identifier_count > bytes.len() as u64
            {
                return Err(PassportEnvelopeError::TooManyIdentifiers);
            }
            for _ in 0..identifier_count {
                let identifier = match reader.u8()? {
                    1 => ExternalIdentifier::Iswc(reader.string()?),
                    2 => ExternalIdentifier::Isrc(reader.string()?),
                    3 => ExternalIdentifier::Other {
                        scheme: reader.string()?,
                        value: reader.string()?,
                    },
                    _ => return Err(PassportEnvelopeError::InvalidIdentifierKind),
                };
                assets.attach_identifier(&id, identifier)?;
            }
        }

        let claim_count = reader.u64()?;
        if claim_count > MAX_CLAIMS || claim_count > bytes.len() as u64 {
            return Err(PassportEnvelopeError::TooManyClaims);
        }
        let mut claims = Vec::with_capacity(claim_count as usize);
        for _ in 0..claim_count {
            let asset_id = AssetId::new(reader.string()?)?;
            let party_id = crate::PartyId::new(reader.string()?)?;
            let pool = decode_rights_pool(reader.u8()?)?;
            let basis_points = match reader.u8()? {
                0 => None,
                1 => Some(reader.u16()?),
                _ => return Err(PassportEnvelopeError::InvalidOptionTag),
            };
            let state = decode_claim_state(reader.u8()?)?;
            let evidence_ref = reader.string()?;
            claims.push(SplitClaim::new(
                asset_id,
                party_id,
                pool,
                basis_points,
                state,
                evidence_ref,
            )?);
        }
        sort_claims(&mut claims);

        let history_len = reader.u64()?;
        if history_len > MAX_ENVELOPE_BYTES as u64 {
            return Err(PassportEnvelopeError::EnvelopeTooLarge);
        }
        let history_len =
            usize::try_from(history_len).map_err(|_| PassportEnvelopeError::EnvelopeTooLarge)?;
        let history_bytes = reader.take(history_len)?;
        let grant_history = GrantHistoryEnvelope::from_bytes(history_bytes)?;

        if !reader.is_empty() {
            return Err(PassportEnvelopeError::TrailingBytes);
        }

        let envelope = Self {
            version,
            assets,
            claims,
            grant_history,
        };
        envelope.validate()?;

        // Reject alternate encodings (including unsorted identifiers/claims)
        // instead of silently canonicalizing untrusted input on import.
        if envelope.canonical_bytes()? != bytes {
            return Err(PassportEnvelopeError::NonCanonicalEncoding);
        }
        Ok(envelope)
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, PassportEnvelopeError> {
        self.validate()?;
        let mut output = Vec::new();
        output.extend_from_slice(MAGIC);
        output.extend_from_slice(&self.version.to_be_bytes());

        let assets: Vec<_> = self.assets.assets().collect();
        output.extend_from_slice(&(assets.len() as u64).to_be_bytes());
        for asset in assets {
            write_string(&mut output, asset.id.as_str())?;
            output.push(encode_asset_kind(asset.kind));
            match asset.title.as_deref() {
                None => output.push(0),
                Some(title) => {
                    output.push(1);
                    write_string(&mut output, title)?;
                }
            }

            let mut identifiers: Vec<_> = asset.identifiers().iter().collect();
            identifiers.sort_by_key(|identifier| identifier_sort_key(identifier));
            output.extend_from_slice(&(identifiers.len() as u64).to_be_bytes());
            for identifier in identifiers {
                match identifier {
                    ExternalIdentifier::Iswc(value) => {
                        output.push(1);
                        write_string(&mut output, value)?;
                    }
                    ExternalIdentifier::Isrc(value) => {
                        output.push(2);
                        write_string(&mut output, value)?;
                    }
                    ExternalIdentifier::Other { scheme, value } => {
                        output.push(3);
                        write_string(&mut output, scheme)?;
                        write_string(&mut output, value)?;
                    }
                }
                if output.len() > MAX_ENVELOPE_BYTES {
                    return Err(PassportEnvelopeError::EnvelopeTooLarge);
                }
            }
            if output.len() > MAX_ENVELOPE_BYTES {
                return Err(PassportEnvelopeError::EnvelopeTooLarge);
            }
        }

        output.extend_from_slice(&(self.claims.len() as u64).to_be_bytes());
        for claim in &self.claims {
            write_string(&mut output, claim.asset_id.as_str())?;
            write_string(&mut output, claim.party_id.as_str())?;
            output.push(encode_rights_pool(claim.pool));
            match claim.basis_points {
                None => output.push(0),
                Some(value) => {
                    output.push(1);
                    output.extend_from_slice(&value.to_be_bytes());
                }
            }
            output.push(encode_claim_state(claim.state));
            write_string(&mut output, &claim.evidence_ref)?;
            if output.len() > MAX_ENVELOPE_BYTES {
                return Err(PassportEnvelopeError::EnvelopeTooLarge);
            }
        }

        let history_bytes = self.grant_history.canonical_bytes()?;
        if output
            .len()
            .saturating_add(8)
            .saturating_add(history_bytes.len())
            > MAX_ENVELOPE_BYTES
        {
            return Err(PassportEnvelopeError::EnvelopeTooLarge);
        }
        output.extend_from_slice(&(history_bytes.len() as u64).to_be_bytes());
        output.extend_from_slice(&history_bytes);
        if output.len() > MAX_ENVELOPE_BYTES {
            return Err(PassportEnvelopeError::EnvelopeTooLarge);
        }
        Ok(output)
    }

    pub fn digest_hex(&self) -> Result<String, PassportEnvelopeError> {
        let bytes = self.canonical_bytes()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DIGEST_DOMAIN);
        hasher.update(&bytes);
        Ok(hasher.finalize().to_hex().to_string())
    }

    pub fn verify_digest(&self, expected: &str) -> Result<(), PassportEnvelopeError> {
        if expected.len() != 64 || !expected.bytes().all(|byte| byte.is_ascii_hexdigit()) {
            return Err(PassportEnvelopeError::InvalidDigestFormat);
        }
        let actual = self.digest_hex()?;
        if actual.eq_ignore_ascii_case(expected) {
            Ok(())
        } else {
            Err(PassportEnvelopeError::DigestMismatch)
        }
    }

    pub fn registry(&self) -> &AssetRegistry {
        &self.assets
    }

    pub fn claims(&self) -> &[SplitClaim] {
        &self.claims
    }

    pub fn grant_history(&self) -> Result<GrantHistory, PassportEnvelopeError> {
        Ok(self.grant_history.replay()?)
    }

    pub fn version(&self) -> u16 {
        self.version
    }
}

fn validate_field(value: &str) -> Result<(), PassportEnvelopeError> {
    if value.len() > MAX_FIELD_BYTES {
        return Err(PassportEnvelopeError::FieldTooLong);
    }
    if value.chars().any(char::is_control) {
        return Err(PassportEnvelopeError::NonCanonicalEncoding);
    }
    Ok(())
}

fn validate_identifier_fields(
    identifier: &ExternalIdentifier,
) -> Result<(), PassportEnvelopeError> {
    match identifier {
        ExternalIdentifier::Iswc(value) | ExternalIdentifier::Isrc(value) => validate_field(value),
        ExternalIdentifier::Other { scheme, value } => {
            validate_field(scheme)?;
            validate_field(value)
        }
    }
}

fn identifier_sort_key(identifier: &ExternalIdentifier) -> (u8, String, String) {
    match identifier {
        ExternalIdentifier::Iswc(value) => (1, value.clone(), String::new()),
        ExternalIdentifier::Isrc(value) => (2, value.clone(), String::new()),
        ExternalIdentifier::Other { scheme, value } => {
            (3, scheme.to_ascii_lowercase(), value.clone())
        }
    }
}

fn claim_sort_key(claim: &SplitClaim) -> (String, u8, String, u8, Option<u16>, String) {
    (
        claim.asset_id.as_str().to_owned(),
        encode_rights_pool(claim.pool),
        claim.party_id.as_str().to_owned(),
        encode_claim_state(claim.state),
        claim.basis_points,
        claim.evidence_ref.clone(),
    )
}

fn sort_claims(claims: &mut [SplitClaim]) {
    claims.sort_by_key(claim_sort_key);
}

fn encode_asset_kind(value: AssetKind) -> u8 {
    match value {
        AssetKind::MusicalWork => 1,
        AssetKind::SoundRecording => 2,
        AssetKind::MusicVideo => 3,
        AssetKind::Release => 4,
        AssetKind::VisualAsset => 5,
        AssetKind::LivePerformance => 6,
    }
}

fn decode_asset_kind(value: u8) -> Result<AssetKind, PassportEnvelopeError> {
    match value {
        1 => Ok(AssetKind::MusicalWork),
        2 => Ok(AssetKind::SoundRecording),
        3 => Ok(AssetKind::MusicVideo),
        4 => Ok(AssetKind::Release),
        5 => Ok(AssetKind::VisualAsset),
        6 => Ok(AssetKind::LivePerformance),
        _ => Err(PassportEnvelopeError::InvalidAssetKind),
    }
}

fn encode_rights_pool(value: RightsPool) -> u8 {
    match value {
        RightsPool::Composition => 1,
        RightsPool::MasterRecording => 2,
        RightsPool::PerformerRights => 3,
        RightsPool::Synchronization => 4,
        RightsPool::VisualAsset => 5,
    }
}

fn decode_rights_pool(value: u8) -> Result<RightsPool, PassportEnvelopeError> {
    match value {
        1 => Ok(RightsPool::Composition),
        2 => Ok(RightsPool::MasterRecording),
        3 => Ok(RightsPool::PerformerRights),
        4 => Ok(RightsPool::Synchronization),
        5 => Ok(RightsPool::VisualAsset),
        _ => Err(PassportEnvelopeError::InvalidRightsPool),
    }
}

fn encode_claim_state(value: ClaimState) -> u8 {
    match value {
        ClaimState::Proposed => 1,
        ClaimState::Confirmed => 2,
        ClaimState::Disputed => 3,
    }
}

fn decode_claim_state(value: u8) -> Result<ClaimState, PassportEnvelopeError> {
    match value {
        1 => Ok(ClaimState::Proposed),
        2 => Ok(ClaimState::Confirmed),
        3 => Ok(ClaimState::Disputed),
        _ => Err(PassportEnvelopeError::InvalidClaimState),
    }
}

fn write_string(output: &mut Vec<u8>, value: &str) -> Result<(), PassportEnvelopeError> {
    validate_field(value)?;
    let bytes = value.as_bytes();
    let length = u32::try_from(bytes.len()).map_err(|_| PassportEnvelopeError::FieldTooLong)?;
    output.extend_from_slice(&length.to_be_bytes());
    output.extend_from_slice(bytes);
    Ok(())
}

struct Reader<'a> {
    bytes: &'a [u8],
    offset: usize,
}

impl<'a> Reader<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, offset: 0 }
    }

    fn take(&mut self, length: usize) -> Result<&'a [u8], PassportEnvelopeError> {
        let end = self
            .offset
            .checked_add(length)
            .ok_or(PassportEnvelopeError::TruncatedEnvelope)?;
        let slice = self
            .bytes
            .get(self.offset..end)
            .ok_or(PassportEnvelopeError::TruncatedEnvelope)?;
        self.offset = end;
        Ok(slice)
    }

    fn u8(&mut self) -> Result<u8, PassportEnvelopeError> {
        Ok(self.take(1)?[0])
    }

    fn u16(&mut self) -> Result<u16, PassportEnvelopeError> {
        let bytes = self.take(2)?;
        Ok(u16::from_be_bytes([bytes[0], bytes[1]]))
    }

    fn u64(&mut self) -> Result<u64, PassportEnvelopeError> {
        let bytes = self.take(8)?;
        Ok(u64::from_be_bytes([
            bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
        ]))
    }

    fn string(&mut self) -> Result<String, PassportEnvelopeError> {
        let length = u32::from_be_bytes(self.take(4)?.try_into().unwrap()) as usize;
        if length > MAX_FIELD_BYTES {
            return Err(PassportEnvelopeError::FieldTooLong);
        }
        String::from_utf8(self.take(length)?.to_vec())
            .map_err(|_| PassportEnvelopeError::InvalidUtf8)
    }

    fn is_empty(&self) -> bool {
        self.offset == self.bytes.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::history::{GrantEvent, GrantHistory};
    use crate::{EventId, GrantId, PartyId, UsageGrant, UseAction};

    fn party(value: &str) -> PartyId {
        PartyId::new(value).unwrap()
    }

    fn snapshot() -> CreativeAssetPassportEnvelope {
        let mut registry = AssetRegistry::new();
        let work = AssetId::new("work-a").unwrap();
        registry
            .register_asset(
                work.clone(),
                AssetKind::MusicalWork,
                Some("Example composition".into()),
            )
            .unwrap();
        registry
            .attach_identifier(&work, ExternalIdentifier::Iswc("T-034.524.680-1".into()))
            .unwrap();

        let recording = AssetId::new("recording-a").unwrap();
        registry
            .register_asset(
                recording.clone(),
                AssetKind::SoundRecording,
                Some("Example master".into()),
            )
            .unwrap();
        registry
            .attach_identifier(
                &recording,
                ExternalIdentifier::Isrc("US-ABC-23-00001".into()),
            )
            .unwrap();

        let claims = vec![
            SplitClaim::new(
                work.clone(),
                party("writer-a"),
                RightsPool::Composition,
                Some(6_000),
                ClaimState::Confirmed,
                "split://writer-a",
            )
            .unwrap(),
            SplitClaim::new(
                work,
                party("writer-b"),
                RightsPool::Composition,
                Some(4_000),
                ClaimState::Confirmed,
                "split://writer-b",
            )
            .unwrap(),
        ];

        let grant = UsageGrant::new(
            GrantId::new("grant-a").unwrap(),
            recording,
            party("distributor"),
            UseAction::Distribute,
            "release-a",
            party("creator"),
            "consent://release-a",
            10,
            Some(100),
        )
        .unwrap();
        let issue = GrantEvent::issue(
            EventId::new("event-issue").unwrap(),
            5,
            party("creator"),
            "evidence://issue",
            grant,
        )
        .unwrap();
        let history = GrantHistory::replay(&[issue]).unwrap();

        CreativeAssetPassportEnvelope::from_parts(&registry, &claims, &history).unwrap()
    }

    fn simple_reference_snapshot() -> CreativeAssetPassportEnvelope {
        let mut registry = AssetRegistry::new();
        let work = AssetId::new("work-a").unwrap();
        registry
            .register_asset(work.clone(), AssetKind::MusicalWork, Some("Song A".into()))
            .unwrap();
        registry
            .attach_identifier(&work, ExternalIdentifier::Iswc("T-034.524.680-1".into()))
            .unwrap();
        let claims = vec![
            SplitClaim::new(
                work,
                party("writer-a"),
                RightsPool::Composition,
                Some(10_000),
                ClaimState::Confirmed,
                "split://writer-a",
            )
            .unwrap(),
        ];
        CreativeAssetPassportEnvelope::from_parts(&registry, &claims, &GrantHistory::new()).unwrap()
    }

    fn unicode_disputed_snapshot() -> CreativeAssetPassportEnvelope {
        let mut registry = AssetRegistry::new();
        let recording = AssetId::new("recording-δ").unwrap();
        registry
            .register_asset(recording.clone(), AssetKind::SoundRecording, None)
            .unwrap();
        registry
            .attach_identifier(
                &recording,
                ExternalIdentifier::Isrc("US-ABC-23-00001".into()),
            )
            .unwrap();
        registry
            .attach_identifier(
                &recording,
                ExternalIdentifier::Other {
                    scheme: "a:".into(),
                    value: "x".into(),
                },
            )
            .unwrap();
        registry
            .attach_identifier(
                &recording,
                ExternalIdentifier::Other {
                    scheme: "a".into(),
                    value: "z".into(),
                },
            )
            .unwrap();
        let claims = vec![
            SplitClaim::new(
                recording,
                party("performer-Δ"),
                RightsPool::MasterRecording,
                None,
                ClaimState::Disputed,
                "evidence://争議",
            )
            .unwrap(),
        ];
        CreativeAssetPassportEnvelope::from_parts(&registry, &claims, &GrantHistory::new()).unwrap()
    }

    #[test]
    fn canonical_bytes_match_frozen_full_snapshot_reference_vector() {
        let envelope = simple_reference_snapshot();
        let actual = envelope
            .canonical_bytes()
            .unwrap()
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        let expected = include_str!("../tests/fixtures/creator-asset-passport-v1.hex").trim();
        assert_eq!(actual, expected);
    }

    #[test]
    fn unicode_disputed_snapshot_matches_frozen_reference_vector() {
        let envelope = unicode_disputed_snapshot();
        let actual = envelope
            .canonical_bytes()
            .unwrap()
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        let expected =
            include_str!("../tests/fixtures/creator-asset-passport-v1-unicode-disputed.hex").trim();
        assert_eq!(actual, expected);
    }

    #[test]
    fn full_passport_round_trip_preserves_assets_splits_and_grants() {
        let envelope = snapshot();
        let bytes = envelope.canonical_bytes().unwrap();
        let decoded = CreativeAssetPassportEnvelope::from_bytes(&bytes).unwrap();
        assert_eq!(decoded, envelope);
        assert_eq!(decoded.canonical_bytes().unwrap(), bytes);
        assert_eq!(decoded.registry().len(), 2);
        assert_eq!(decoded.claims().len(), 2);
        assert_eq!(decoded.grant_history().unwrap().len(), 1);
    }

    #[test]
    fn asset_identifier_claims_and_permissions_share_one_portable_digest() {
        let envelope = snapshot();
        let digest = envelope.digest_hex().unwrap();
        assert_eq!(digest.len(), 64);
        assert!(envelope.verify_digest(&digest).is_ok());
        assert_eq!(
            envelope.verify_digest("00".repeat(32).as_str()),
            Err(PassportEnvelopeError::DigestMismatch)
        );
    }

    #[test]
    fn claims_for_unknown_assets_are_rejected() {
        let mut envelope = snapshot();
        let mut claims = envelope.claims.clone();
        claims.push(
            SplitClaim::new(
                AssetId::new("missing-asset").unwrap(),
                party("writer-c"),
                RightsPool::Composition,
                Some(1_000),
                ClaimState::Proposed,
                "split://writer-c",
            )
            .unwrap(),
        );
        sort_claims(&mut claims);
        envelope.claims = claims;
        assert_eq!(
            envelope.canonical_bytes(),
            Err(PassportEnvelopeError::InvalidClaimAsset)
        );
    }

    #[test]
    fn decoder_rejects_trailing_bytes() {
        let envelope = snapshot();
        let mut bytes = envelope.canonical_bytes().unwrap();
        bytes.push(0);
        assert_eq!(
            CreativeAssetPassportEnvelope::from_bytes(&bytes),
            Err(PassportEnvelopeError::TrailingBytes)
        );
    }

    #[test]
    fn decoder_rejects_noncanonical_external_identifier_order() {
        fn skip_string(bytes: &[u8], offset: &mut usize) {
            let length =
                u32::from_be_bytes(bytes[*offset..*offset + 4].try_into().unwrap()) as usize;
            *offset += 4 + length;
        }

        let envelope = unicode_disputed_snapshot();
        let bytes = envelope.canonical_bytes().unwrap();
        let mut offset = MAGIC.len() + 2 + 8; // magic, version, asset count
        skip_string(&bytes, &mut offset); // asset ID
        offset += 2; // asset kind and absent-title tag
        let count = u64::from_be_bytes(bytes[offset..offset + 8].try_into().unwrap());
        offset += 8;
        assert_eq!(count, 3);

        let mut records = Vec::new();
        for _ in 0..count {
            let start = offset;
            let tag = bytes[offset];
            offset += 1;
            match tag {
                1 | 2 => skip_string(&bytes, &mut offset),
                3 => {
                    skip_string(&bytes, &mut offset);
                    skip_string(&bytes, &mut offset);
                }
                _ => panic!("fixture contains an unexpected identifier tag"),
            }
            records.push((start, offset));
        }

        let (second_start, second_end) = records[1];
        let (third_start, third_end) = records[2];
        let mut altered = Vec::with_capacity(bytes.len());
        altered.extend_from_slice(&bytes[..second_start]);
        altered.extend_from_slice(&bytes[third_start..third_end]);
        altered.extend_from_slice(&bytes[second_start..second_end]);
        altered.extend_from_slice(&bytes[third_end..]);

        assert_eq!(
            CreativeAssetPassportEnvelope::from_bytes(&altered),
            Err(PassportEnvelopeError::NonCanonicalEncoding)
        );
    }

    #[test]
    fn decoder_rejects_unsupported_version() {
        let envelope = snapshot();
        let mut bytes = envelope.canonical_bytes().unwrap();
        bytes[MAGIC.len() + 1] = 2;
        assert_eq!(
            CreativeAssetPassportEnvelope::from_bytes(&bytes),
            Err(PassportEnvelopeError::UnsupportedVersion(2))
        );
    }
}
