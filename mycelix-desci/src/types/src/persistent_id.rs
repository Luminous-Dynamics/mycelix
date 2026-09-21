// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Dependency-light persistent identifier types for scholarly interoperability.
//!
//! These types deliberately separate normalization from verification. A
//! syntactically valid ORCID/ROR/DOI/RAiD/etc. is not proof that a Mycelix
//! actor owns or controls that external record, and imported metadata never
//! overrides signed local assertions merely because an identifier parses.

use crate::{Error, Result};
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

const MAX_IDENTIFIER_BYTES: usize = 2_048;
const MAX_SCHEME_BYTES: usize = 128;
const CROCKFORD_BASE32: &str = "0123456789abcdefghjkmnpqrstvwxyz";

/// A known persistent-identifier namespace.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PersistentIdentifierScheme {
    Doi,
    Orcid,
    Ror,
    Raid,
    Swhid,
    Igsn,
    Rrid,
    Url,
    /// Award identifiers whose meaning depends on the issuing authority.
    FunderAward { authority: String },
    /// Identifier meaningful inside Mycelix only.
    LocalMycelix,
    /// Forward-compatible explicitly named external namespace.
    Other { scheme: String },
}

impl PersistentIdentifierScheme {
    fn validate(&self) -> Result<()> {
        match self {
            Self::FunderAward { authority } => {
                validate_label(authority, "funder award authority", MAX_SCHEME_BYTES)
            }
            Self::Other { scheme } => {
                validate_label(scheme, "persistent identifier scheme", MAX_SCHEME_BYTES)
            }
            _ => Ok(()),
        }
    }
}

/// A typed identifier. Syntactic normalization is not ownership verification.
///
/// Fields are private and deserialization routes through [`Self::new`] so an
/// invalid or non-normalized identifier cannot bypass the constructor through
/// wire data or direct struct construction.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(try_from = "PersistentIdentifierRefWire")]
pub struct PersistentIdentifierRef {
    scheme: PersistentIdentifierScheme,
    /// Canonical representation for the selected scheme.
    canonical_value: String,
}

#[derive(Debug, Clone, Deserialize)]
struct PersistentIdentifierRefWire {
    scheme: PersistentIdentifierScheme,
    canonical_value: String,
}

impl TryFrom<PersistentIdentifierRefWire> for PersistentIdentifierRef {
    type Error = Error;

    fn try_from(value: PersistentIdentifierRefWire) -> Result<Self> {
        Self::new(value.scheme, value.canonical_value)
    }
}

impl PersistentIdentifierRef {
    pub fn new(
        scheme: PersistentIdentifierScheme,
        value: impl Into<String>,
    ) -> Result<Self> {
        scheme.validate()?;
        let value = validate_identifier_text(value.into())?;
        let canonical_value = normalize_for_scheme(&scheme, &value)?;
        Ok(Self {
            scheme,
            canonical_value,
        })
    }

    /// Classify only identifiers carrying an explicit, recognizable namespace
    /// marker. Ambiguous bare strings are preserved as unclassified rather
    /// than guessed into a scholarly PID scheme.
    pub fn classify_explicit(value: impl Into<String>) -> Result<LegacyIdentifierClassification> {
        let value = validate_identifier_text(value.into())?;
        let lower = value.to_ascii_lowercase();

        let typed = if lower.starts_with("https://doi.org/")
            || lower.starts_with("http://doi.org/")
            || lower.starts_with("doi:")
        {
            Some(Self::new(PersistentIdentifierScheme::Doi, value.clone())?)
        } else if lower.starts_with("https://orcid.org/")
            || lower.starts_with("http://orcid.org/")
        {
            Some(Self::new(PersistentIdentifierScheme::Orcid, value.clone())?)
        } else if lower.starts_with("https://ror.org/") || lower.starts_with("http://ror.org/") {
            Some(Self::new(PersistentIdentifierScheme::Ror, value.clone())?)
        } else if lower.starts_with("https://raid.org/") || lower.starts_with("http://raid.org/") {
            Some(Self::new(PersistentIdentifierScheme::Raid, value.clone())?)
        } else if lower.starts_with("swh:1:") {
            Some(Self::new(PersistentIdentifierScheme::Swhid, value.clone())?)
        } else if value.starts_with("RRID:") {
            Some(Self::new(PersistentIdentifierScheme::Rrid, value.clone())?)
        } else {
            None
        };

        Ok(match typed {
            Some(identifier) => LegacyIdentifierClassification::Typed(identifier),
            None => LegacyIdentifierClassification::Unclassified(value),
        })
    }

    pub fn scheme(&self) -> &PersistentIdentifierScheme {
        &self.scheme
    }

    pub fn as_str(&self) -> &str {
        &self.canonical_value
    }
}

/// Result of cautiously classifying a legacy free-form identifier.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", tag = "classification", content = "value")]
pub enum LegacyIdentifierClassification {
    Typed(PersistentIdentifierRef),
    Unclassified(String),
}

/// Provenance for metadata fetched from an external PID/registry service.
///
/// This records where metadata came from; it does not make the external source
/// authoritative over signed local Mycelix state. Construction and
/// deserialization share the same validation path.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(try_from = "ExternalMetadataProvenanceWire")]
pub struct ExternalMetadataProvenance {
    source: String,
    retrieved_at: DateTime<Utc>,
    source_version: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
struct ExternalMetadataProvenanceWire {
    source: String,
    retrieved_at: DateTime<Utc>,
    source_version: Option<String>,
}

impl TryFrom<ExternalMetadataProvenanceWire> for ExternalMetadataProvenance {
    type Error = Error;

    fn try_from(value: ExternalMetadataProvenanceWire) -> Result<Self> {
        Self::new(value.source, value.retrieved_at, value.source_version)
    }
}

impl ExternalMetadataProvenance {
    pub fn new(
        source: impl Into<String>,
        retrieved_at: DateTime<Utc>,
        source_version: Option<String>,
    ) -> Result<Self> {
        let source = validate_identifier_text(source.into())?;
        let source_version = match source_version {
            Some(version) => {
                validate_label(&version, "external metadata source version", 256)?;
                Some(version)
            }
            None => None,
        };
        Ok(Self {
            source,
            retrieved_at,
            source_version,
        })
    }

    pub fn source(&self) -> &str {
        &self.source
    }

    pub const fn retrieved_at(&self) -> DateTime<Utc> {
        self.retrieved_at
    }

    pub fn source_version(&self) -> Option<&str> {
        self.source_version.as_deref()
    }
}

fn validate_identifier_text(value: String) -> Result<String> {
    let trimmed = value.trim();
    if trimmed.is_empty() {
        return Err(Error::Validation(
            "persistent identifier cannot be empty".to_string(),
        ));
    }
    if trimmed.len() > MAX_IDENTIFIER_BYTES {
        return Err(Error::Validation(format!(
            "persistent identifier cannot exceed {MAX_IDENTIFIER_BYTES} bytes"
        )));
    }
    if trimmed.chars().any(char::is_control) {
        return Err(Error::Validation(
            "persistent identifier cannot contain control characters".to_string(),
        ));
    }
    Ok(trimmed.to_string())
}

fn validate_label(value: &str, label: &str, max_bytes: usize) -> Result<()> {
    let trimmed = value.trim();
    if trimmed.is_empty() {
        return Err(Error::Validation(format!("{label} cannot be empty")));
    }
    if trimmed != value {
        return Err(Error::Validation(format!(
            "{label} cannot contain leading or trailing whitespace"
        )));
    }
    if value.len() > max_bytes {
        return Err(Error::Validation(format!(
            "{label} cannot exceed {max_bytes} bytes"
        )));
    }
    if value.chars().any(char::is_control) {
        return Err(Error::Validation(format!(
            "{label} cannot contain control characters"
        )));
    }
    Ok(())
}

fn normalize_for_scheme(scheme: &PersistentIdentifierScheme, value: &str) -> Result<String> {
    match scheme {
        PersistentIdentifierScheme::Doi => normalize_doi(value),
        PersistentIdentifierScheme::Orcid => normalize_orcid(value),
        PersistentIdentifierScheme::Ror => normalize_ror(value),
        PersistentIdentifierScheme::Raid => normalize_raid(value),
        PersistentIdentifierScheme::Swhid => normalize_swhid(value),
        PersistentIdentifierScheme::Rrid => normalize_rrid(value),
        PersistentIdentifierScheme::Url => normalize_url(value),
        PersistentIdentifierScheme::Igsn
        | PersistentIdentifierScheme::FunderAward { .. }
        | PersistentIdentifierScheme::LocalMycelix
        | PersistentIdentifierScheme::Other { .. } => Ok(value.to_string()),
    }
}

fn normalize_doi(value: &str) -> Result<String> {
    let lower = value.to_ascii_lowercase();
    let doi = if lower.starts_with("https://doi.org/") {
        &value[16..]
    } else if lower.starts_with("http://doi.org/") {
        &value[15..]
    } else if lower.starts_with("doi:") {
        value[4..].trim()
    } else {
        value
    };

    if !looks_like_doi(doi) {
        return Err(Error::Validation("invalid DOI syntax".to_string()));
    }
    Ok(doi.to_string())
}

fn looks_like_doi(value: &str) -> bool {
    let Some((prefix, suffix)) = value.split_once('/') else {
        return false;
    };
    prefix.starts_with("10.")
        && prefix.len() > 3
        && prefix[3..].chars().all(|c| c.is_ascii_digit())
        && !suffix.is_empty()
        && !value.chars().any(char::is_whitespace)
}

fn normalize_orcid(value: &str) -> Result<String> {
    let lower = value.to_ascii_lowercase();
    let orcid = if lower.starts_with("https://orcid.org/") {
        &value[18..]
    } else if lower.starts_with("http://orcid.org/") {
        &value[17..]
    } else {
        value
    };
    let canonical = orcid.to_ascii_uppercase();
    if !valid_orcid(&canonical) {
        return Err(Error::Validation(
            "invalid ORCID syntax or checksum".to_string(),
        ));
    }
    Ok(canonical)
}

fn valid_orcid(value: &str) -> bool {
    if value.len() != 19 {
        return false;
    }

    for (index, byte) in value.as_bytes().iter().enumerate() {
        if matches!(index, 4 | 9 | 14) {
            if *byte != b'-' {
                return false;
            }
        } else if index == 18 {
            if !byte.is_ascii_digit() && *byte != b'X' {
                return false;
            }
        } else if !byte.is_ascii_digit() {
            return false;
        }
    }

    let digits: Vec<u32> = value
        .bytes()
        .filter(|byte| *byte != b'-')
        .map(|byte| {
            if byte == b'X' {
                10
            } else {
                u32::from(byte - b'0')
            }
        })
        .collect();
    if digits.len() != 16 {
        return false;
    }

    let mut total = 0u32;
    for digit in &digits[..15] {
        total = (total + digit) * 2;
    }
    let remainder = total % 11;
    let result = (12 - remainder) % 11;
    digits[15] == result
}

fn normalize_ror(value: &str) -> Result<String> {
    let lower = value.to_ascii_lowercase();
    let id = if lower.starts_with("https://ror.org/") {
        &lower[16..]
    } else if lower.starts_with("http://ror.org/") {
        &lower[15..]
    } else {
        lower.as_str()
    };

    if !valid_ror(id) {
        return Err(Error::Validation(
            "invalid ROR identifier syntax or checksum".to_string(),
        ));
    }
    Ok(format!("https://ror.org/{id}"))
}

fn valid_ror(id: &str) -> bool {
    if id.len() != 9 || !id.starts_with('0') {
        return false;
    }
    let body = &id[1..7];
    let checksum = &id[7..9];
    if !body
        .bytes()
        .all(|byte| crockford_value(byte).is_some())
        || !checksum.bytes().all(|byte| byte.is_ascii_digit())
    {
        return false;
    }

    let mut number = 0u64;
    for byte in body.bytes() {
        let Some(value) = crockford_value(byte) else {
            return false;
        };
        number = number * 32 + u64::from(value);
    }

    let expected = 98 - ((number * 100) % 97);
    checksum.parse::<u64>().ok() == Some(expected)
}

fn crockford_value(byte: u8) -> Option<u8> {
    let byte = byte.to_ascii_lowercase();
    CROCKFORD_BASE32
        .as_bytes()
        .iter()
        .position(|candidate| *candidate == byte)
        .and_then(|index| u8::try_from(index).ok())
}

fn normalize_raid(value: &str) -> Result<String> {
    let lower = value.to_ascii_lowercase();
    let id = if lower.starts_with("https://raid.org/") {
        &value[17..]
    } else if lower.starts_with("http://raid.org/") {
        &value[16..]
    } else {
        value
    };
    if !looks_like_doi(id) {
        return Err(Error::Validation(
            "invalid RAiD identifier syntax; expected RAiD DOI form".to_string(),
        ));
    }
    Ok(format!("https://raid.org/{id}"))
}

fn normalize_swhid(value: &str) -> Result<String> {
    if !value.starts_with("swh:1:") || value.chars().any(char::is_whitespace) {
        return Err(Error::Validation("invalid SWHID syntax".to_string()));
    }
    let components: Vec<&str> = value.split(':').collect();
    if components.len() < 4 || components[2].is_empty() || components[3].is_empty() {
        return Err(Error::Validation("invalid SWHID syntax".to_string()));
    }
    Ok(value.to_string())
}

fn normalize_rrid(value: &str) -> Result<String> {
    if !value.starts_with("RRID:") || value.len() <= 5 || value.chars().any(char::is_whitespace) {
        return Err(Error::Validation("invalid RRID syntax".to_string()));
    }
    Ok(value.to_string())
}

fn normalize_url(value: &str) -> Result<String> {
    let lower = value.to_ascii_lowercase();
    if !(lower.starts_with("https://") || lower.starts_with("http://"))
        || value.chars().any(char::is_whitespace)
    {
        return Err(Error::Validation(
            "URL identifiers must use an explicit http(s) scheme".to_string(),
        ));
    }
    Ok(value.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalizes_explicit_doi_resolver_without_claiming_verification() {
        let id = PersistentIdentifierRef::new(
            PersistentIdentifierScheme::Doi,
            "https://doi.org/10.1234/Example-1",
        )
        .unwrap();
        assert_eq!(id.as_str(), "10.1234/Example-1");
    }

    #[test]
    fn validates_orcid_checksum_and_separators() {
        let id = PersistentIdentifierRef::new(
            PersistentIdentifierScheme::Orcid,
            "https://orcid.org/0000-0002-1825-0097",
        )
        .unwrap();
        assert_eq!(id.as_str(), "0000-0002-1825-0097");

        assert!(PersistentIdentifierRef::new(
            PersistentIdentifierScheme::Orcid,
            "0000-0002-1825-0098",
        )
        .is_err());
        assert!(PersistentIdentifierRef::new(
            PersistentIdentifierScheme::Orcid,
            "00000002-1825-0097-",
        )
        .is_err());
    }

    #[test]
    fn validates_ror_crockford_alphabet_and_checksum() {
        let ror = PersistentIdentifierRef::new(
            PersistentIdentifierScheme::Ror,
            "https://ror.org/038sjwq14",
        )
        .unwrap();
        assert_eq!(ror.as_str(), "https://ror.org/038sjwq14");

        // Same shape but invalid checksum.
        assert!(PersistentIdentifierRef::new(
            PersistentIdentifierScheme::Ror,
            "https://ror.org/038sjwq15",
        )
        .is_err());
        // Crockford Base32 excludes U as well as I/L/O.
        assert!(PersistentIdentifierRef::new(
            PersistentIdentifierScheme::Ror,
            "0u8sjwq14",
        )
        .is_err());
    }

    #[test]
    fn normalizes_raid_url() {
        let raid = PersistentIdentifierRef::new(
            PersistentIdentifierScheme::Raid,
            "https://raid.org/10.26259/a673754f",
        )
        .unwrap();
        assert_eq!(raid.as_str(), "https://raid.org/10.26259/a673754f");
    }

    #[test]
    fn legacy_classifier_refuses_to_guess_bare_doi_like_value() {
        let classified = PersistentIdentifierRef::classify_explicit("10.26259/a673754f").unwrap();
        assert_eq!(
            classified,
            LegacyIdentifierClassification::Unclassified("10.26259/a673754f".to_string())
        );
    }

    #[test]
    fn legacy_classifier_uses_explicit_namespace_markers() {
        let classified =
            PersistentIdentifierRef::classify_explicit("https://raid.org/10.26259/a673754f")
                .unwrap();
        assert!(matches!(
            classified,
            LegacyIdentifierClassification::Typed(ref identifier)
                if identifier.scheme() == &PersistentIdentifierScheme::Raid
        ));
    }

    #[test]
    fn funder_award_namespace_must_be_explicit() {
        assert!(PersistentIdentifierRef::new(
            PersistentIdentifierScheme::FunderAward {
                authority: "NSF".to_string(),
            },
            "1234567",
        )
        .is_ok());

        assert!(PersistentIdentifierRef::new(
            PersistentIdentifierScheme::FunderAward {
                authority: "   ".to_string(),
            },
            "1234567",
        )
        .is_err());
    }

    #[test]
    fn pid_deserialization_cannot_bypass_normalization_or_validation() {
        let forged = r#"{
            "scheme":"ror",
            "canonical_value":"https://ror.org/038sjwq15"
        }"#;
        assert!(serde_json::from_str::<PersistentIdentifierRef>(forged).is_err());

        let valid = PersistentIdentifierRef::new(
            PersistentIdentifierScheme::Orcid,
            "https://orcid.org/0000-0002-1825-0097",
        )
        .unwrap();
        let json = serde_json::to_string(&valid).unwrap();
        let round_trip: PersistentIdentifierRef = serde_json::from_str(&json).unwrap();
        assert_eq!(round_trip, valid);
    }

    #[test]
    fn provenance_deserialization_revalidates_fields() {
        let forged = format!(
            r#"{{"source":"   ","retrieved_at":"{}","source_version":null}}"#,
            Utc::now().to_rfc3339()
        );
        assert!(serde_json::from_str::<ExternalMetadataProvenance>(&forged).is_err());
    }
}
