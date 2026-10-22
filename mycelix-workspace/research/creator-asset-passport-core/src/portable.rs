//! Versioned portable envelope for grant/revocation history.
//!
//! Canonical bytes are a binary, big-endian, length-prefixed format. The
//! BLAKE3 digest detects byte changes but is not a signature, authorship proof,
//! legal title, or proof that referenced consent/evidence is authentic.

use std::error::Error;
use std::fmt;

use crate::history::{
    GrantEvent, GrantEventAction, GrantHistory, HistoryError,
    independently_reconstruct_effective_grants,
};
use crate::{AssetId, EventId, GrantId, PartyId, PassportError, UsageGrant, UseAction};

const MAGIC: &[u8] = b"MYCELIX-CSCP-GRANT-HISTORY\0";
const FORMAT_VERSION: u16 = 1;
const MAX_FIELD_BYTES: usize = 256;
const MAX_EVENT_COUNT: u64 = 1_000_000;
const MAX_ENVELOPE_BYTES: usize = 256 * 1024 * 1024;
const DIGEST_DOMAIN: &[u8] = b"MYCELIX-CSCP-GRANT-HISTORY-DIGEST-V1\0";

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PortableHistoryError {
    BadMagic,
    UnsupportedVersion(u16),
    TruncatedEnvelope,
    TrailingBytes,
    FieldTooLong,
    InvalidUtf8,
    TooManyEvents,
    EnvelopeTooLarge,
    InvalidEventKind,
    InvalidUseAction,
    InvalidOptionTag,
    InvalidDigestFormat,
    DigestMismatch,
    InvalidHistory(HistoryError),
    InvalidPassportField(PassportError),
}

impl fmt::Display for PortableHistoryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl Error for PortableHistoryError {}

impl From<HistoryError> for PortableHistoryError {
    fn from(value: HistoryError) -> Self {
        Self::InvalidHistory(value)
    }
}

impl From<PassportError> for PortableHistoryError {
    fn from(value: PassportError) -> Self {
        Self::InvalidPassportField(value)
    }
}

/// A versioned export of grant/revocation events only. Asset registry entries,
/// split claims, private media, and external source payloads are not included.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GrantHistoryEnvelope {
    version: u16,
    events: Vec<GrantEvent>,
}

impl GrantHistoryEnvelope {
    pub fn from_history(history: &GrantHistory) -> Result<Self, PortableHistoryError> {
        history.verify_against_independent_replay()?;
        let envelope = Self {
            version: FORMAT_VERSION,
            events: history.events().to_vec(),
        };
        // Validate against both replay paths before making export bytes.
        envelope.validate_history()?;
        Ok(envelope)
    }

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, PortableHistoryError> {
        if bytes.len() > MAX_ENVELOPE_BYTES {
            return Err(PortableHistoryError::EnvelopeTooLarge);
        }
        let mut reader = Reader::new(bytes);
        if reader.take(MAGIC.len())? != MAGIC {
            return Err(PortableHistoryError::BadMagic);
        }
        let version = reader.u16()?;
        if version != FORMAT_VERSION {
            return Err(PortableHistoryError::UnsupportedVersion(version));
        }
        let count = reader.u64()?;
        if count > MAX_EVENT_COUNT || count > bytes.len() as u64 {
            return Err(PortableHistoryError::TooManyEvents);
        }

        let mut events = Vec::with_capacity(count as usize);
        for _ in 0..count {
            let event_id = EventId::new(reader.string()?)?;
            let occurred_at = reader.u64()?;
            let actor = PartyId::new(reader.string()?)?;
            let event_evidence = reader.string()?;
            let action_tag = reader.u8()?;

            let event = match action_tag {
                1 => {
                    let grant_id = GrantId::new(reader.string()?)?;
                    let asset_id = AssetId::new(reader.string()?)?;
                    let grantee_id = PartyId::new(reader.string()?)?;
                    let action = decode_action(reader.u8()?)?;
                    let purpose = reader.string()?;
                    let issuer = PartyId::new(reader.string()?)?;
                    let grant_evidence = reader.string()?;
                    let valid_from = reader.u64()?;
                    let expires_at = match reader.u8()? {
                        0 => None,
                        1 => Some(reader.u64()?),
                        _ => return Err(PortableHistoryError::InvalidOptionTag),
                    };
                    let grant = UsageGrant::new(
                        grant_id,
                        asset_id,
                        grantee_id,
                        action,
                        purpose,
                        issuer,
                        grant_evidence,
                        valid_from,
                        expires_at,
                    )?;
                    GrantEvent::issue(event_id, occurred_at, actor, event_evidence, grant)?
                }
                2 => {
                    let grant_id = GrantId::new(reader.string()?)?;
                    GrantEvent::revoke(event_id, occurred_at, actor, event_evidence, grant_id)?
                }
                _ => return Err(PortableHistoryError::InvalidEventKind),
            };
            events.push(event);
        }

        if !reader.is_empty() {
            return Err(PortableHistoryError::TrailingBytes);
        }

        let envelope = Self { version, events };
        envelope.validate_history()?;
        Ok(envelope)
    }

    fn validate_history(&self) -> Result<(), PortableHistoryError> {
        if self.version != FORMAT_VERSION {
            return Err(PortableHistoryError::UnsupportedVersion(self.version));
        }
        let history = GrantHistory::replay(&self.events)?;
        history.verify_against_independent_replay()?;
        let independently_rebuilt = independently_reconstruct_effective_grants(&self.events)?;
        if independently_rebuilt != history.effective_grants() {
            return Err(PortableHistoryError::InvalidHistory(
                HistoryError::SnapshotMismatch,
            ));
        }
        Ok(())
    }

    /// Canonical encoding: stable field order, explicit version, fixed enum
    /// tags, big-endian integers, and UTF-8 strings with u32 byte lengths.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, PortableHistoryError> {
        self.validate_history()?;
        if self.events.len() as u64 > MAX_EVENT_COUNT {
            return Err(PortableHistoryError::TooManyEvents);
        }
        let mut output = Vec::new();
        output.extend_from_slice(MAGIC);
        output.extend_from_slice(&self.version.to_be_bytes());
        output.extend_from_slice(&(self.events.len() as u64).to_be_bytes());

        for event in &self.events {
            write_string(&mut output, event.event_id().as_str())?;
            output.extend_from_slice(&event.occurred_at().to_be_bytes());
            write_string(&mut output, event.actor().as_str())?;
            write_string(&mut output, event.evidence_ref())?;

            match event.action() {
                GrantEventAction::Issued { grant } => {
                    output.push(1);
                    write_string(&mut output, grant.id().as_str())?;
                    write_string(&mut output, grant.asset_id().as_str())?;
                    write_string(&mut output, grant.grantee_id().as_str())?;
                    output.push(encode_action(grant.action()));
                    write_string(&mut output, grant.purpose())?;
                    write_string(&mut output, grant.issued_by().as_str())?;
                    write_string(&mut output, grant.evidence_ref())?;
                    output.extend_from_slice(&grant.valid_from().to_be_bytes());
                    match grant.expires_at() {
                        None => output.push(0),
                        Some(expires_at) => {
                            output.push(1);
                            output.extend_from_slice(&expires_at.to_be_bytes());
                        }
                    }
                }
                GrantEventAction::Revoked { grant_id } => {
                    output.push(2);
                    write_string(&mut output, grant_id.as_str())?;
                }
            }

            if output.len() > MAX_ENVELOPE_BYTES {
                return Err(PortableHistoryError::TooManyEvents);
            }
        }

        Ok(output)
    }

    /// Domain-separated BLAKE3 content digest for the exact canonical bytes.
    /// It proves equality to those bytes, not who authored them or whether
    /// the referenced consent was genuine.
    pub fn digest_hex(&self) -> Result<String, PortableHistoryError> {
        let bytes = self.canonical_bytes()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DIGEST_DOMAIN);
        hasher.update(&bytes);
        Ok(hasher.finalize().to_hex().to_string())
    }

    pub fn verify_digest(&self, expected: &str) -> Result<(), PortableHistoryError> {
        if expected.len() != 64 || !expected.bytes().all(|b| b.is_ascii_hexdigit()) {
            return Err(PortableHistoryError::InvalidDigestFormat);
        }
        let actual = self.digest_hex()?;
        if actual.eq_ignore_ascii_case(expected) {
            Ok(())
        } else {
            Err(PortableHistoryError::DigestMismatch)
        }
    }

    pub fn replay(&self) -> Result<GrantHistory, PortableHistoryError> {
        self.validate_history()?;
        Ok(GrantHistory::replay(&self.events)?)
    }

    pub fn version(&self) -> u16 {
        self.version
    }

    pub fn events(&self) -> &[GrantEvent] {
        &self.events
    }

    pub fn event_count(&self) -> usize {
        self.events.len()
    }
}

fn write_string(output: &mut Vec<u8>, value: &str) -> Result<(), PortableHistoryError> {
    let bytes = value.as_bytes();
    if bytes.len() > MAX_FIELD_BYTES {
        return Err(PortableHistoryError::FieldTooLong);
    }
    let len = u32::try_from(bytes.len()).map_err(|_| PortableHistoryError::FieldTooLong)?;
    output.extend_from_slice(&len.to_be_bytes());
    output.extend_from_slice(bytes);
    Ok(())
}

fn encode_action(action: UseAction) -> u8 {
    match action {
        UseAction::AnalyzeMetadata => 1,
        UseAction::Publish => 2,
        UseAction::Distribute => 3,
        UseAction::Commercialize => 4,
        UseAction::TrainModel => 5,
        UseAction::GenerateDerivative => 6,
        UseAction::SynthesizeVoiceOrLikeness => 7,
    }
}

fn decode_action(value: u8) -> Result<UseAction, PortableHistoryError> {
    match value {
        1 => Ok(UseAction::AnalyzeMetadata),
        2 => Ok(UseAction::Publish),
        3 => Ok(UseAction::Distribute),
        4 => Ok(UseAction::Commercialize),
        5 => Ok(UseAction::TrainModel),
        6 => Ok(UseAction::GenerateDerivative),
        7 => Ok(UseAction::SynthesizeVoiceOrLikeness),
        _ => Err(PortableHistoryError::InvalidUseAction),
    }
}

struct Reader<'a> {
    bytes: &'a [u8],
    offset: usize,
}

impl<'a> Reader<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, offset: 0 }
    }

    fn take(&mut self, length: usize) -> Result<&'a [u8], PortableHistoryError> {
        let end = self
            .offset
            .checked_add(length)
            .ok_or(PortableHistoryError::TruncatedEnvelope)?;
        let slice = self
            .bytes
            .get(self.offset..end)
            .ok_or(PortableHistoryError::TruncatedEnvelope)?;
        self.offset = end;
        Ok(slice)
    }

    fn u8(&mut self) -> Result<u8, PortableHistoryError> {
        Ok(self.take(1)?[0])
    }

    fn u16(&mut self) -> Result<u16, PortableHistoryError> {
        let bytes = self.take(2)?;
        Ok(u16::from_be_bytes([bytes[0], bytes[1]]))
    }

    fn u32(&mut self) -> Result<u32, PortableHistoryError> {
        let bytes = self.take(4)?;
        Ok(u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
    }

    fn u64(&mut self) -> Result<u64, PortableHistoryError> {
        let bytes = self.take(8)?;
        Ok(u64::from_be_bytes([
            bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
        ]))
    }

    fn string(&mut self) -> Result<String, PortableHistoryError> {
        let len = self.u32()? as usize;
        if len > MAX_FIELD_BYTES {
            return Err(PortableHistoryError::FieldTooLong);
        }
        let bytes = self.take(len)?;
        String::from_utf8(bytes.to_vec()).map_err(|_| PortableHistoryError::InvalidUtf8)
    }

    fn is_empty(&self) -> bool {
        self.offset == self.bytes.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::history::{GrantEvent, GrantHistory};
    use crate::{AssetId, EventId, GrantId, PartyId, UsageGrant, UseAction};

    fn party(value: &str) -> PartyId {
        PartyId::new(value).unwrap()
    }

    fn demo_history() -> GrantHistory {
        let grant = UsageGrant::new(
            GrantId::new("grant-a").unwrap(),
            AssetId::new("recording-a").unwrap(),
            party("distributor"),
            UseAction::Distribute,
            "release-a",
            party("creator"),
            "consent://grant-a",
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
        let revoke = GrantEvent::revoke(
            EventId::new("event-revoke").unwrap(),
            50,
            party("creator"),
            "evidence://revoke",
            GrantId::new("grant-a").unwrap(),
        )
        .unwrap();
        GrantHistory::replay(&[issue, revoke]).unwrap()
    }

    fn unicode_no_expiry_history() -> GrantHistory {
        let grant = UsageGrant::new(
            GrantId::new("grant-unicode").unwrap(),
            AssetId::new("recording-δ").unwrap(),
            party("voice-service"),
            UseAction::SynthesizeVoiceOrLikeness,
            "one-off-clone-demo",
            party("artist-Δ"),
            "consent://clip-specific",
            1,
            None,
        )
        .unwrap();
        let issue = GrantEvent::issue(
            EventId::new("event-unicode").unwrap(),
            0,
            party("artist-Δ"),
            "evidence://approved",
            grant,
        )
        .unwrap();
        GrantHistory::replay(&[issue]).unwrap()
    }

    #[test]
    fn encoding_is_deterministic_and_round_trips() {
        let envelope = GrantHistoryEnvelope::from_history(&demo_history()).unwrap();
        let bytes_a = envelope.canonical_bytes().unwrap();
        let bytes_b = envelope.canonical_bytes().unwrap();
        assert_eq!(bytes_a, bytes_b);

        let decoded = GrantHistoryEnvelope::from_bytes(&bytes_a).unwrap();
        assert_eq!(decoded, envelope);
        assert_eq!(decoded.canonical_bytes().unwrap(), bytes_a);
        assert_eq!(decoded.event_count(), 2);
    }

    #[test]
    fn canonical_bytes_match_frozen_cross_language_reference_vector() {
        let envelope = GrantHistoryEnvelope::from_history(&demo_history()).unwrap();
        let actual = envelope
            .canonical_bytes()
            .unwrap()
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        let expected = include_str!("../tests/fixtures/creator-grant-history-v1.hex").trim();
        assert_eq!(actual, expected);
    }

    #[test]
    fn unicode_and_no_expiry_match_reference_vector() {
        let envelope = GrantHistoryEnvelope::from_history(&unicode_no_expiry_history()).unwrap();
        let actual = envelope
            .canonical_bytes()
            .unwrap()
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        let expected =
            include_str!("../tests/fixtures/creator-grant-history-v1-unicode-no-expiry.hex").trim();
        assert_eq!(actual, expected);
    }

    #[test]
    fn digest_is_stable_and_detects_other_content() {
        let envelope = GrantHistoryEnvelope::from_history(&demo_history()).unwrap();
        let digest = envelope.digest_hex().unwrap();
        assert_eq!(digest.len(), 64);
        assert!(envelope.verify_digest(&digest).is_ok());
        assert_eq!(
            envelope.verify_digest("00".repeat(32).as_str()),
            Err(PortableHistoryError::DigestMismatch)
        );
    }

    #[test]
    fn digest_verification_rejects_malformed_digest_strings() {
        let envelope = GrantHistoryEnvelope::from_history(&demo_history()).unwrap();
        assert_eq!(
            envelope.verify_digest("not-a-digest"),
            Err(PortableHistoryError::InvalidDigestFormat)
        );
    }

    #[test]
    fn decoder_rejects_truncated_and_trailing_bytes() {
        let envelope = GrantHistoryEnvelope::from_history(&demo_history()).unwrap();
        let bytes = envelope.canonical_bytes().unwrap();

        assert!(matches!(
            GrantHistoryEnvelope::from_bytes(&bytes[..bytes.len() - 1]),
            Err(PortableHistoryError::TruncatedEnvelope)
                | Err(PortableHistoryError::InvalidHistory(_))
        ));

        let mut with_trailing = bytes;
        with_trailing.push(0);
        assert_eq!(
            GrantHistoryEnvelope::from_bytes(&with_trailing),
            Err(PortableHistoryError::TrailingBytes)
        );
    }

    #[test]
    fn decoder_rejects_unsupported_version_and_bad_magic() {
        let envelope = GrantHistoryEnvelope::from_history(&demo_history()).unwrap();
        let mut bytes = envelope.canonical_bytes().unwrap();
        let version_offset = MAGIC.len();
        bytes[version_offset + 1] = 2;
        assert_eq!(
            GrantHistoryEnvelope::from_bytes(&bytes),
            Err(PortableHistoryError::UnsupportedVersion(2))
        );

        let mut bad_magic = envelope.canonical_bytes().unwrap();
        bad_magic[0] ^= 0xff;
        assert_eq!(
            GrantHistoryEnvelope::from_bytes(&bad_magic),
            Err(PortableHistoryError::BadMagic)
        );
    }

    #[test]
    fn envelope_replay_keeps_revocation_state() {
        let envelope = GrantHistoryEnvelope::from_history(&demo_history()).unwrap();
        let replayed = envelope.replay().unwrap();
        let grant = replayed
            .find_grant(&GrantId::new("grant-a").unwrap())
            .unwrap();
        assert_eq!(grant.revocation_timestamp(), Some(50));
    }
}
