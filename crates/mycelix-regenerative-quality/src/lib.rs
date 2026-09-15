// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Adopted quality-profile references for Mycelix regenerative systems.
//!
//! REGEN-003 does not encode a universal biochar, compost, biomass, soil, or
//! agronomic standard. It provides a small structural contract for naming an
//! exact externally defined profile revision and recording that an institution
//! declares that revision applicable within a bounded scope.
//!
//! ```text
//! profile reference
//! != issuer authenticity
//! != adoption legitimacy
//! != conformance
//! != agronomic suitability
//! != carbon eligibility
//! != execution authority
//! ```

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use core::{fmt, str::FromStr};
use mycelix_regenerative_core::QualityProfileId;
use std::collections::BTreeSet;

#[cfg(feature = "serde")]
use serde::{Deserialize, Deserializer, Serialize, Serializer, de::Error as _};

/// Schema revision for an immutable external quality-profile reference.
pub const QUALITY_PROFILE_REFERENCE_SCHEMA_VERSION: u16 = 1;
/// Schema revision for an institutional quality-profile adoption record.
pub const QUALITY_PROFILE_ADOPTION_SCHEMA_VERSION: u16 = 1;
/// Maximum number of profile scopes carried by one reference or adoption.
pub const MAX_PROFILE_SCOPES: usize = 64;
/// Maximum byte length for issuer/adopter/evidence/source references.
pub const MAX_REFERENCE_TEXT_BYTES: usize = 1024;
/// Maximum byte length for one human/machine-readable version label.
pub const MAX_VERSION_LABEL_BYTES: usize = 128;
/// Maximum byte length for a namespaced quality-scope key.
pub const MAX_SCOPE_KEY_BYTES: usize = 128;

/// Structural errors in a quality-profile reference or adoption record.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QualityProfileError {
    /// A schema revision is unsupported.
    UnsupportedSchemaVersion {
        /// Actual unsupported schema revision.
        actual: u16,
        /// Schema revision required by this implementation.
        expected: u16,
    },
    /// A required text field is empty.
    EmptyText {
        /// Canonical field name whose value was empty.
        field: &'static str,
    },
    /// A text field exceeds its declared byte bound.
    TextTooLong {
        /// Canonical field name whose value exceeded the bound.
        field: &'static str,
        /// Actual UTF-8 byte length.
        actual: usize,
        /// Maximum permitted UTF-8 byte length.
        max: usize,
    },
    /// A text field contains an ASCII control character.
    ControlCharacter {
        /// Canonical field name containing the control character.
        field: &'static str,
        /// Zero-based byte index of the rejected control character.
        index: usize,
    },
    /// A SHA-256 digest is not in exact canonical form.
    InvalidSha256,
    /// A quality-scope key is malformed.
    InvalidScopeKey,
    /// No scopes were supplied.
    MissingScopes,
    /// Too many scopes were supplied.
    TooManyScopes {
        /// Actual number of supplied scopes.
        actual: usize,
        /// Maximum permitted number of scopes.
        max: usize,
    },
    /// Scopes are duplicated.
    DuplicateScope(String),
    /// Serialized scopes are not in canonical sorted order.
    NonCanonicalScopeOrder,
    /// A replacement claims to supersede its own exact document digest.
    SelfSupersession,
    /// An adoption scope is not contained by the referenced profile scope.
    AdoptionScopeNotCovered(String),
    /// The declared temporal interval is empty or reversed.
    InvalidValidityWindow,
}

impl fmt::Display for QualityProfileError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedSchemaVersion { actual, expected } => {
                write!(f, "unsupported schema version {actual}; expected {expected}")
            }
            Self::EmptyText { field } => write!(f, "{field} must not be empty"),
            Self::TextTooLong { field, actual, max } => {
                write!(f, "{field} is {actual} bytes; maximum is {max}")
            }
            Self::ControlCharacter { field, index } => {
                write!(f, "{field} contains a control character at byte index {index}")
            }
            Self::InvalidSha256 => f.write_str(
                "SHA-256 identity must be sha256: followed by exactly 64 lowercase hex digits",
            ),
            Self::InvalidScopeKey => f.write_str(
                "quality scope must contain at least two nonempty lowercase ASCII namespace segments",
            ),
            Self::MissingScopes => f.write_str("at least one quality scope is required"),
            Self::TooManyScopes { actual, max } => {
                write!(f, "quality scope count {actual} exceeds maximum {max}")
            }
            Self::DuplicateScope(scope) => write!(f, "duplicate quality scope {scope}"),
            Self::NonCanonicalScopeOrder => {
                f.write_str("quality scopes must be strictly sorted in canonical order")
            }
            Self::SelfSupersession => {
                f.write_str("quality profile revision cannot supersede its own document digest")
            }
            Self::AdoptionScopeNotCovered(scope) => write!(
                f,
                "adopted quality scope {scope} is not covered by the referenced profile"
            ),
            Self::InvalidValidityWindow => {
                f.write_str("valid-until-exclusive must be greater than valid-from")
            }
        }
    }
}

impl std::error::Error for QualityProfileError {}

fn validate_text(field: &'static str, value: &str, max: usize) -> Result<(), QualityProfileError> {
    if value.is_empty() {
        return Err(QualityProfileError::EmptyText { field });
    }
    if value.len() > max {
        return Err(QualityProfileError::TextTooLong {
            field,
            actual: value.len(),
            max,
        });
    }
    if let Some((index, _)) = value
        .bytes()
        .enumerate()
        .find(|(_, byte)| byte.is_ascii_control())
    {
        return Err(QualityProfileError::ControlCharacter { field, index });
    }
    Ok(())
}

fn decode_hex_nibble(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        _ => None,
    }
}

fn encode_hex_nibble(value: u8) -> char {
    match value {
        0..=9 => char::from(b'0' + value),
        10..=15 => char::from(b'a' + (value - 10)),
        _ => unreachable!("nibble is masked to four bits"),
    }
}

/// Exact SHA-256 content identity with a self-describing canonical text form.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Sha256Digest([u8; 32]);

impl Sha256Digest {
    /// Construct from already-computed SHA-256 bytes.
    pub const fn from_bytes(bytes: [u8; 32]) -> Self {
        Self(bytes)
    }

    /// Return the raw 32-byte digest.
    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }

    /// Parse exact `sha256:<64 lowercase hex>` canonical text.
    pub fn parse_canonical(value: &str) -> Result<Self, QualityProfileError> {
        const PREFIX: &str = "sha256:";
        let Some(hex) = value.strip_prefix(PREFIX) else {
            return Err(QualityProfileError::InvalidSha256);
        };
        if hex.len() != 64 {
            return Err(QualityProfileError::InvalidSha256);
        }

        let bytes = hex.as_bytes();
        let mut digest = [0_u8; 32];
        for index in 0..32 {
            let Some(high) = decode_hex_nibble(bytes[index * 2]) else {
                return Err(QualityProfileError::InvalidSha256);
            };
            let Some(low) = decode_hex_nibble(bytes[index * 2 + 1]) else {
                return Err(QualityProfileError::InvalidSha256);
            };
            digest[index] = (high << 4) | low;
        }
        Ok(Self(digest))
    }
}

impl fmt::Display for Sha256Digest {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("sha256:")?;
        for byte in self.0 {
            f.write_fmt(format_args!(
                "{}{}",
                encode_hex_nibble(byte >> 4),
                encode_hex_nibble(byte & 0x0f)
            ))?;
        }
        Ok(())
    }
}

impl FromStr for Sha256Digest {
    type Err = QualityProfileError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Self::parse_canonical(s)
    }
}

#[cfg(feature = "serde")]
impl Serialize for Sha256Digest {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_str(&self.to_string())
    }
}

#[cfg(feature = "serde")]
impl<'de> Deserialize<'de> for Sha256Digest {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let value = String::deserialize(deserializer)?;
        Self::parse_canonical(&value).map_err(D::Error::custom)
    }
}

/// Canonical namespaced key describing what material/application class a profile covers.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct QualityScopeKey(String);

impl QualityScopeKey {
    /// Construct a canonical scope such as `regen:biochar` or `wbc:biochar`.
    pub fn new(value: impl AsRef<str>) -> Result<Self, QualityProfileError> {
        let value = value.as_ref();
        if value.is_empty() || value.len() > MAX_SCOPE_KEY_BYTES {
            return Err(QualityProfileError::InvalidScopeKey);
        }

        let segments: Vec<&str> = value.split(':').collect();
        if segments.len() < 2 || segments.iter().any(|segment| segment.is_empty()) {
            return Err(QualityProfileError::InvalidScopeKey);
        }

        for segment in segments {
            let bytes = segment.as_bytes();
            let Some(first) = bytes.first().copied() else {
                return Err(QualityProfileError::InvalidScopeKey);
            };
            let Some(last) = bytes.last().copied() else {
                return Err(QualityProfileError::InvalidScopeKey);
            };
            if !(first.is_ascii_lowercase() || first.is_ascii_digit())
                || !(last.is_ascii_lowercase() || last.is_ascii_digit())
            {
                return Err(QualityProfileError::InvalidScopeKey);
            }
            if !bytes.iter().copied().all(|byte| {
                byte.is_ascii_lowercase()
                    || byte.is_ascii_digit()
                    || matches!(byte, b'-' | b'_' | b'.')
            }) {
                return Err(QualityProfileError::InvalidScopeKey);
            }
        }

        Ok(Self(value.to_owned()))
    }

    /// Return the canonical scope key.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for QualityScopeKey {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl FromStr for QualityScopeKey {
    type Err = QualityProfileError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Self::new(s)
    }
}

#[cfg(feature = "serde")]
impl Serialize for QualityScopeKey {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_str(self.as_str())
    }
}

#[cfg(feature = "serde")]
impl<'de> Deserialize<'de> for QualityScopeKey {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let value = String::deserialize(deserializer)?;
        Self::new(value).map_err(D::Error::custom)
    }
}

/// Immutable reference to one exact external or institution-authored quality-profile revision.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct QualityProfileReference {
    /// Wire-schema revision.
    pub schema_version: u16,
    /// Stable semantic profile lineage identity from REGEN-002.
    pub profile_id: QualityProfileId,
    /// Opaque issuer identity/reference; structural presence is not issuer authentication.
    pub issuer_ref: String,
    /// Exact version label declared by the profile issuer.
    pub version_label: String,
    /// SHA-256 of the exact immutable profile representation adopted by this record.
    pub content_sha256: Sha256Digest,
    /// Opaque locator/reference for the exact profile representation.
    pub source_ref: String,
    /// Canonical sorted material/application scopes covered by this reference.
    pub scopes: Vec<QualityScopeKey>,
    /// Exact predecessor document digest when this revision explicitly supersedes one.
    pub supersedes_content_sha256: Option<Sha256Digest>,
}

impl QualityProfileReference {
    /// Construct and canonicalize one exact profile reference.
    pub fn new(
        profile_id: QualityProfileId,
        issuer_ref: impl Into<String>,
        version_label: impl Into<String>,
        content_sha256: Sha256Digest,
        source_ref: impl Into<String>,
        mut scopes: Vec<QualityScopeKey>,
    ) -> Result<Self, QualityProfileError> {
        reject_duplicate_scopes(&scopes)?;
        scopes.sort();
        let reference = Self {
            schema_version: QUALITY_PROFILE_REFERENCE_SCHEMA_VERSION,
            profile_id,
            issuer_ref: issuer_ref.into(),
            version_label: version_label.into(),
            content_sha256,
            source_ref: source_ref.into(),
            scopes,
            supersedes_content_sha256: None,
        };
        reference.validate()?;
        Ok(reference)
    }

    /// Validate structural/canonical form only.
    pub fn validate(&self) -> Result<(), QualityProfileError> {
        if self.schema_version != QUALITY_PROFILE_REFERENCE_SCHEMA_VERSION {
            return Err(QualityProfileError::UnsupportedSchemaVersion {
                actual: self.schema_version,
                expected: QUALITY_PROFILE_REFERENCE_SCHEMA_VERSION,
            });
        }
        validate_text(
            "quality_profile.issuer_ref",
            &self.issuer_ref,
            MAX_REFERENCE_TEXT_BYTES,
        )?;
        validate_text(
            "quality_profile.version_label",
            &self.version_label,
            MAX_VERSION_LABEL_BYTES,
        )?;
        validate_text(
            "quality_profile.source_ref",
            &self.source_ref,
            MAX_REFERENCE_TEXT_BYTES,
        )?;
        validate_scope_list(&self.scopes)?;
        if self.supersedes_content_sha256 == Some(self.content_sha256) {
            return Err(QualityProfileError::SelfSupersession);
        }
        Ok(())
    }
}

/// Raw institutional declaration that an exact profile revision applies to a narrower scope.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct QualityProfileAdoption {
    /// Wire-schema revision.
    pub schema_version: u16,
    /// Exact profile revision being adopted.
    pub profile: QualityProfileReference,
    /// Opaque adopter identity/reference; structural presence is not legitimacy proof.
    pub adopter_ref: String,
    /// Opaque immutable or externally resolvable evidence for the adoption decision.
    pub adoption_evidence_ref: String,
    /// Canonical sorted subset of `profile.scopes` actually adopted.
    pub adopted_scopes: Vec<QualityScopeKey>,
    /// Optional inclusive lower bound of the declared validity window.
    pub valid_from_unix_ms: Option<i64>,
    /// Optional exclusive upper bound of the declared validity window.
    pub valid_until_exclusive_unix_ms: Option<i64>,
}

impl QualityProfileAdoption {
    /// Construct a structural adoption record and canonicalize its adopted scope order.
    pub fn new(
        profile: QualityProfileReference,
        adopter_ref: impl Into<String>,
        adoption_evidence_ref: impl Into<String>,
        mut adopted_scopes: Vec<QualityScopeKey>,
        valid_from_unix_ms: Option<i64>,
        valid_until_exclusive_unix_ms: Option<i64>,
    ) -> Result<Self, QualityProfileError> {
        reject_duplicate_scopes(&adopted_scopes)?;
        adopted_scopes.sort();
        let adoption = Self {
            schema_version: QUALITY_PROFILE_ADOPTION_SCHEMA_VERSION,
            profile,
            adopter_ref: adopter_ref.into(),
            adoption_evidence_ref: adoption_evidence_ref.into(),
            adopted_scopes,
            valid_from_unix_ms,
            valid_until_exclusive_unix_ms,
        };
        adoption.validate()?;
        Ok(adoption)
    }

    /// Validate structural/canonical form only.
    pub fn validate(&self) -> Result<(), QualityProfileError> {
        if self.schema_version != QUALITY_PROFILE_ADOPTION_SCHEMA_VERSION {
            return Err(QualityProfileError::UnsupportedSchemaVersion {
                actual: self.schema_version,
                expected: QUALITY_PROFILE_ADOPTION_SCHEMA_VERSION,
            });
        }
        self.profile.validate()?;
        validate_text(
            "quality_adoption.adopter_ref",
            &self.adopter_ref,
            MAX_REFERENCE_TEXT_BYTES,
        )?;
        validate_text(
            "quality_adoption.adoption_evidence_ref",
            &self.adoption_evidence_ref,
            MAX_REFERENCE_TEXT_BYTES,
        )?;
        validate_scope_list(&self.adopted_scopes)?;

        let covered: BTreeSet<_> = self.profile.scopes.iter().collect();
        for scope in &self.adopted_scopes {
            if !covered.contains(scope) {
                return Err(QualityProfileError::AdoptionScopeNotCovered(
                    scope.to_string(),
                ));
            }
        }

        if let (Some(from), Some(until)) =
            (self.valid_from_unix_ms, self.valid_until_exclusive_unix_ms)
            && until <= from
        {
            return Err(QualityProfileError::InvalidValidityWindow);
        }

        Ok(())
    }

    /// Test only the declared interval against a caller-supplied instant.
    ///
    /// This is not trusted-time/currentness evidence; the caller owns the clock semantics.
    pub fn declared_window_contains(&self, unix_ms: i64) -> bool {
        self.valid_from_unix_ms.is_none_or(|from| unix_ms >= from)
            && self
                .valid_until_exclusive_unix_ms
                .is_none_or(|until| unix_ms < until)
    }
}

/// Structurally validated adoption wrapper.
///
/// This wrapper is intentionally not serde-deserializable. Untrusted transported
/// records must cross [`QualityProfileAdoption::validate`] again before downstream
/// code treats their structure as checked.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ValidatedQualityProfileAdoption(QualityProfileAdoption);

impl ValidatedQualityProfileAdoption {
    /// Validate and wrap a raw adoption record.
    pub fn new(record: QualityProfileAdoption) -> Result<Self, QualityProfileError> {
        record.validate()?;
        Ok(Self(record))
    }

    /// Borrow the validated structural record.
    pub fn as_record(&self) -> &QualityProfileAdoption {
        &self.0
    }

    /// Consume the wrapper and return the underlying raw record.
    pub fn into_record(self) -> QualityProfileAdoption {
        self.0
    }
}

impl TryFrom<QualityProfileAdoption> for ValidatedQualityProfileAdoption {
    type Error = QualityProfileError;

    fn try_from(value: QualityProfileAdoption) -> Result<Self, Self::Error> {
        Self::new(value)
    }
}

fn reject_duplicate_scopes(scopes: &[QualityScopeKey]) -> Result<(), QualityProfileError> {
    let mut seen = BTreeSet::new();
    for scope in scopes {
        if !seen.insert(scope) {
            return Err(QualityProfileError::DuplicateScope(scope.to_string()));
        }
    }
    Ok(())
}

fn validate_scope_list(scopes: &[QualityScopeKey]) -> Result<(), QualityProfileError> {
    if scopes.is_empty() {
        return Err(QualityProfileError::MissingScopes);
    }
    if scopes.len() > MAX_PROFILE_SCOPES {
        return Err(QualityProfileError::TooManyScopes {
            actual: scopes.len(),
            max: MAX_PROFILE_SCOPES,
        });
    }
    reject_duplicate_scopes(scopes)?;
    if scopes.windows(2).any(|window| window[0] >= window[1]) {
        return Err(QualityProfileError::NonCanonicalScopeOrder);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Sha256Digest {
        Sha256Digest::from_bytes([byte; 32])
    }

    fn profile() -> QualityProfileReference {
        QualityProfileReference::new(
            QualityProfileId::new("community-biochar").unwrap(),
            "did:example:profile-issuer",
            "2026.1",
            digest(0x11),
            "https://example.invalid/profile/2026.1",
            vec![
                QualityScopeKey::new("regen:soil-amendment").unwrap(),
                QualityScopeKey::new("regen:biochar").unwrap(),
            ],
        )
        .unwrap()
    }

    #[test]
    fn sha256_text_is_exact_self_describing_and_lowercase() {
        let expected = format!("sha256:{}", "ab".repeat(32));
        let parsed = Sha256Digest::parse_canonical(&expected).unwrap();
        assert_eq!(parsed.to_string(), expected);
        assert!(Sha256Digest::parse_canonical(&expected.to_uppercase()).is_err());
        assert!(Sha256Digest::parse_canonical(&"ab".repeat(32)).is_err());
    }

    #[test]
    fn scope_keys_are_namespaced_and_canonical() {
        assert_eq!(
            QualityScopeKey::new("regen:biochar").unwrap().as_str(),
            "regen:biochar"
        );
        for bad in [
            "biochar",
            ":biochar",
            "regen:",
            "regen::biochar",
            "Regen:biochar",
            "regen:bio char",
            "regen:biochar/soil",
        ] {
            assert!(QualityScopeKey::new(bad).is_err(), "accepted {bad:?}");
        }
    }

    #[test]
    fn profile_constructor_canonicalizes_scope_order() {
        let profile = profile();
        assert_eq!(
            profile
                .scopes
                .iter()
                .map(QualityScopeKey::as_str)
                .collect::<Vec<_>>(),
            vec!["regen:biochar", "regen:soil-amendment"]
        );
        assert!(profile.validate().is_ok());
    }

    #[test]
    fn direct_noncanonical_scope_order_fails_closed() {
        let mut profile = profile();
        profile.scopes.reverse();
        assert_eq!(
            profile.validate(),
            Err(QualityProfileError::NonCanonicalScopeOrder)
        );
    }

    #[test]
    fn duplicate_scopes_and_self_supersession_fail_closed() {
        let scope = QualityScopeKey::new("regen:biochar").unwrap();
        assert!(
            QualityProfileReference::new(
                QualityProfileId::new("duplicate-test").unwrap(),
                "issuer",
                "v1",
                digest(0x22),
                "source",
                vec![scope.clone(), scope],
            )
            .is_err()
        );

        let mut profile = profile();
        profile.supersedes_content_sha256 = Some(profile.content_sha256);
        assert_eq!(
            profile.validate(),
            Err(QualityProfileError::SelfSupersession)
        );
    }

    #[test]
    fn adoption_may_narrow_but_not_widen_profile_scope() {
        let adoption = QualityProfileAdoption::new(
            profile(),
            "did:example:community",
            "mycelix:governance-decision:123",
            vec![QualityScopeKey::new("regen:biochar").unwrap()],
            Some(1000),
            Some(2000),
        )
        .unwrap();
        assert!(adoption.validate().is_ok());

        let widened = QualityProfileAdoption::new(
            profile(),
            "did:example:community",
            "mycelix:governance-decision:124",
            vec![QualityScopeKey::new("regen:compost").unwrap()],
            None,
            None,
        );
        assert_eq!(
            widened,
            Err(QualityProfileError::AdoptionScopeNotCovered(
                "regen:compost".into()
            ))
        );
    }

    #[test]
    fn declared_window_is_half_open_and_not_a_trusted_clock_claim() {
        let adoption = QualityProfileAdoption::new(
            profile(),
            "did:example:community",
            "mycelix:governance-decision:125",
            vec![QualityScopeKey::new("regen:biochar").unwrap()],
            Some(1000),
            Some(2000),
        )
        .unwrap();
        assert!(!adoption.declared_window_contains(999));
        assert!(adoption.declared_window_contains(1000));
        assert!(adoption.declared_window_contains(1999));
        assert!(!adoption.declared_window_contains(2000));
    }

    #[test]
    fn invalid_temporal_window_fails_closed() {
        assert_eq!(
            QualityProfileAdoption::new(
                profile(),
                "did:example:community",
                "mycelix:governance-decision:126",
                vec![QualityScopeKey::new("regen:biochar").unwrap()],
                Some(2000),
                Some(2000),
            ),
            Err(QualityProfileError::InvalidValidityWindow)
        );
    }

    #[test]
    fn validated_wrapper_rechecks_raw_records() {
        let mut adoption = QualityProfileAdoption::new(
            profile(),
            "did:example:community",
            "mycelix:governance-decision:127",
            vec![QualityScopeKey::new("regen:biochar").unwrap()],
            None,
            None,
        )
        .unwrap();
        adoption.adopter_ref.clear();
        assert!(ValidatedQualityProfileAdoption::new(adoption).is_err());
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trip_retains_exact_profile_revision_and_requires_revalidation() {
        let adoption = QualityProfileAdoption::new(
            profile(),
            "did:example:community",
            "mycelix:governance-decision:128",
            vec![QualityScopeKey::new("regen:biochar").unwrap()],
            None,
            None,
        )
        .unwrap();
        let encoded = serde_json::to_string(&adoption).unwrap();
        let decoded: QualityProfileAdoption = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, adoption);
        assert!(ValidatedQualityProfileAdoption::new(decoded).is_ok());
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_rejects_noncanonical_digest_and_scope_keys() {
        assert!(serde_json::from_str::<Sha256Digest>("\"sha256:ABCD\"").is_err());
        assert!(serde_json::from_str::<QualityScopeKey>("\"Biochar\"").is_err());
    }
}
