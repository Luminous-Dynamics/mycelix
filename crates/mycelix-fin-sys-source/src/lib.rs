// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Read-only source-profile parsing for macro-financial evidence.
//!
//! This crate deliberately stops at source identity and transport grammar.
//! It does **not** define canonical Mycelix observations, financing-regime
//! classifications, distress scores, forecasts, policy recommendations, or
//! economic-system rankings.

use std::fmt;
use std::str::FromStr;

pub const BIS_DSR_AGENCY: &str = "BIS";
pub const BIS_DSR_RESOURCE: &str = "WS_DSR";
pub const BIS_DSR_VERSION: &str = "1.0";
pub const BIS_DSR_UNIT: &str = "Per cent";
pub const BIS_DSR_PARSER_PROFILE: &str = "bis-dsr-source-key-v1";

/// BIS warns that DSR movements within an economy are generally more
/// meaningful than direct rankings of absolute DSR levels across economies.
pub const BIS_DSR_COMPARABILITY_NOTE: &str =
    "absolute DSR levels are not a directly comparable cross-economy distress ranking";

const MAX_AREA_CODE_BYTES: usize = 8;

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct BisDatasetRef {
    pub agency: String,
    pub resource: String,
    pub version: String,
}

impl BisDatasetRef {
    pub fn new(
        agency: impl Into<String>,
        resource: impl Into<String>,
        version: impl Into<String>,
    ) -> Self {
        Self {
            agency: agency.into(),
            resource: resource.into(),
            version: version.into(),
        }
    }

    pub fn dsr_v1() -> Self {
        Self::new(BIS_DSR_AGENCY, BIS_DSR_RESOURCE, BIS_DSR_VERSION)
    }

    pub fn is_dsr_v1(&self) -> bool {
        self.agency == BIS_DSR_AGENCY
            && self.resource == BIS_DSR_RESOURCE
            && self.version == BIS_DSR_VERSION
    }
}

impl fmt::Display for BisDatasetRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{},{},{}", self.agency, self.resource, self.version)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BisFrequency {
    Quarterly,
}

impl BisFrequency {
    pub const fn source_code(self) -> &'static str {
        match self {
            Self::Quarterly => "Q",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BisDsrBorrower {
    PrivateNonFinancialSector,
    HouseholdsAndNpishs,
    NonFinancialCorporations,
}

impl BisDsrBorrower {
    pub const fn source_code(self) -> &'static str {
        match self {
            Self::PrivateNonFinancialSector => "P",
            Self::HouseholdsAndNpishs => "H",
            Self::NonFinancialCorporations => "N",
        }
    }

    pub const fn source_label(self) -> &'static str {
        match self {
            Self::PrivateNonFinancialSector => "Private non-financial sector",
            Self::HouseholdsAndNpishs => "Households & NPISHs",
            Self::NonFinancialCorporations => "Non-financial corporations",
        }
    }

    fn parse(code: &str) -> Result<Self, BisParseError> {
        match code {
            "P" => Ok(Self::PrivateNonFinancialSector),
            "H" => Ok(Self::HouseholdsAndNpishs),
            "N" => Ok(Self::NonFinancialCorporations),
            other => Err(BisParseError::UnsupportedBorrowerCode(other.to_owned())),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BisAreaCode(String);

impl BisAreaCode {
    pub fn new(code: impl Into<String>) -> Result<Self, BisParseError> {
        let code = code.into();
        if code.is_empty() {
            return Err(BisParseError::InvalidAreaCode(code));
        }
        if code.len() > MAX_AREA_CODE_BYTES
            || !code
                .bytes()
                .all(|byte| byte.is_ascii_uppercase() || byte.is_ascii_digit())
        {
            return Err(BisParseError::InvalidAreaCode(code));
        }
        Ok(Self(code))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for BisAreaCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct BisDsrSeriesKey {
    pub frequency: BisFrequency,
    pub area: BisAreaCode,
    pub borrower: BisDsrBorrower,
}

impl BisDsrSeriesKey {
    /// Parse a DSR source key only when the caller has bound the exact
    /// `BIS,WS_DSR,1.0` dataset profile.
    pub fn parse_for(dataset: &BisDatasetRef, source_key: &str) -> Result<Self, BisParseError> {
        if !dataset.is_dsr_v1() {
            return Err(BisParseError::WrongDataset {
                expected: BisDatasetRef::dsr_v1(),
                actual: Box::new(dataset.clone()),
            });
        }

        let mut parts = source_key.split('.');
        let frequency = parts.next().ok_or(BisParseError::WrongDimensionCount)?;
        let area = parts.next().ok_or(BisParseError::WrongDimensionCount)?;
        let borrower = parts.next().ok_or(BisParseError::WrongDimensionCount)?;
        if parts.next().is_some() {
            return Err(BisParseError::WrongDimensionCount);
        }

        if frequency != "Q" {
            return Err(BisParseError::UnsupportedFrequency(frequency.to_owned()));
        }

        Ok(Self {
            frequency: BisFrequency::Quarterly,
            area: BisAreaCode::new(area)?,
            borrower: BisDsrBorrower::parse(borrower)?,
        })
    }

    pub fn source_key(&self) -> String {
        format!(
            "{}.{}.{}",
            self.frequency.source_code(),
            self.area,
            self.borrower.source_code()
        )
    }
}

impl fmt::Display for BisDsrSeriesKey {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.source_key())
    }
}

impl FromStr for BisDsrSeriesKey {
    type Err = BisParseError;

    fn from_str(source_key: &str) -> Result<Self, Self::Err> {
        Self::parse_for(&BisDatasetRef::dsr_v1(), source_key)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BisAcquisitionMode {
    SdmxRestV2,
    BulkCsv,
    BulkCsvFlat,
    BulkSdmx21Compact,
    BulkSdmx21Generic,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BisSourceArtifactRef {
    pub dataset: BisDatasetRef,
    pub acquisition_mode: BisAcquisitionMode,
    /// Exact API URL, retained artifact reference, or equivalent source locator.
    pub source_ref: String,
    /// Source-supplied release identity/date. No ambient current-time authority.
    pub release_ref: Option<String>,
    /// Digest reference to retained response/artifact bytes where available.
    pub artifact_digest_ref: Option<String>,
    pub parser_profile: String,
}

impl BisSourceArtifactRef {
    pub fn dsr_v1(
        acquisition_mode: BisAcquisitionMode,
        source_ref: impl Into<String>,
        release_ref: Option<String>,
        artifact_digest_ref: Option<String>,
    ) -> Self {
        Self {
            dataset: BisDatasetRef::dsr_v1(),
            acquisition_mode,
            source_ref: source_ref.into(),
            release_ref,
            artifact_digest_ref,
            parser_profile: BIS_DSR_PARSER_PROFILE.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BisParseError {
    WrongDataset {
        expected: BisDatasetRef,
        actual: Box<BisDatasetRef>,
    },
    WrongDimensionCount,
    UnsupportedFrequency(String),
    InvalidAreaCode(String),
    UnsupportedBorrowerCode(String),
}

impl fmt::Display for BisParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongDataset { expected, actual } => {
                write!(f, "expected BIS dataset {expected}, got {actual}")
            }
            Self::WrongDimensionCount => {
                f.write_str("BIS DSR v1 series key must contain exactly three dimensions")
            }
            Self::UnsupportedFrequency(code) => {
                write!(f, "unsupported BIS DSR frequency code: {code}")
            }
            Self::InvalidAreaCode(code) => write!(f, "invalid BIS source area code: {code:?}"),
            Self::UnsupportedBorrowerCode(code) => {
                write!(f, "unsupported BIS DSR borrower code: {code}")
            }
        }
    }
}

impl std::error::Error for BisParseError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn south_africa_private_sector_round_trips_exactly() {
        let key = BisDsrSeriesKey::from_str("Q.ZA.P").unwrap();
        assert_eq!(key.frequency, BisFrequency::Quarterly);
        assert_eq!(key.area.as_str(), "ZA");
        assert_eq!(key.borrower, BisDsrBorrower::PrivateNonFinancialSector);
        assert_eq!(key.source_key(), "Q.ZA.P");
    }

    #[test]
    fn household_and_nfc_codes_are_preserved() {
        let households = BisDsrSeriesKey::from_str("Q.DE.H").unwrap();
        assert_eq!(households.borrower, BisDsrBorrower::HouseholdsAndNpishs);
        assert_eq!(households.source_key(), "Q.DE.H");

        let corporations = BisDsrSeriesKey::from_str("Q.SE.N").unwrap();
        assert_eq!(
            corporations.borrower,
            BisDsrBorrower::NonFinancialCorporations
        );
        assert_eq!(corporations.source_key(), "Q.SE.N");
    }

    #[test]
    fn non_quarterly_key_is_rejected() {
        assert_eq!(
            BisDsrSeriesKey::from_str("A.ZA.P").unwrap_err(),
            BisParseError::UnsupportedFrequency("A".to_owned())
        );
    }

    #[test]
    fn unknown_borrower_code_is_rejected() {
        assert_eq!(
            BisDsrSeriesKey::from_str("Q.ZA.X").unwrap_err(),
            BisParseError::UnsupportedBorrowerCode("X".to_owned())
        );
    }

    #[test]
    fn missing_or_extra_dimensions_are_rejected() {
        assert_eq!(
            BisDsrSeriesKey::from_str("Q.ZA").unwrap_err(),
            BisParseError::WrongDimensionCount
        );
        assert_eq!(
            BisDsrSeriesKey::from_str("Q.ZA.P.EXTRA").unwrap_err(),
            BisParseError::WrongDimensionCount
        );
    }

    #[test]
    fn area_code_policy_is_exact_and_bounded() {
        for invalid in ["", "za", "Z-A", "ZÄ", "ABCDEFGHI"] {
            assert!(matches!(
                BisAreaCode::new(invalid),
                Err(BisParseError::InvalidAreaCode(_))
            ));
        }

        // BIS area codes can be alphanumeric aggregates; do not force ISO country semantics.
        assert_eq!(BisAreaCode::new("5R").unwrap().as_str(), "5R");
        assert_eq!(BisAreaCode::new("XW").unwrap().as_str(), "XW");
    }

    #[test]
    fn wrong_dataset_cannot_silently_reuse_dsr_parser() {
        let other = BisDatasetRef::new("BIS", "WS_CREDIT_GAP", "1.0");
        let err = BisDsrSeriesKey::parse_for(&other, "Q.ZA.P").unwrap_err();
        assert!(matches!(err, BisParseError::WrongDataset { .. }));
    }

    #[test]
    fn dataset_identity_is_explicit() {
        assert_eq!(BisDatasetRef::dsr_v1().to_string(), "BIS,WS_DSR,1.0");
        assert_eq!(BIS_DSR_UNIT, "Per cent");
        assert_eq!(BIS_DSR_PARSER_PROFILE, "bis-dsr-source-key-v1");
    }

    #[test]
    fn source_artifact_ref_has_no_analysis_authority() {
        let source = BisSourceArtifactRef::dsr_v1(
            BisAcquisitionMode::SdmxRestV2,
            "https://stats.bis.org/api/v2/...",
            Some("2026-09-14".to_owned()),
            Some("sha256:fixture".to_owned()),
        );

        assert_eq!(source.dataset, BisDatasetRef::dsr_v1());
        assert_eq!(source.parser_profile, BIS_DSR_PARSER_PROFILE);
        assert!(source.source_ref.contains("stats.bis.org"));
    }

    #[test]
    fn labels_are_presentation_metadata_not_source_key_material() {
        let key = BisDsrSeriesKey::from_str("Q.SE.N").unwrap();
        assert_eq!(key.borrower.source_label(), "Non-financial corporations");
        assert_eq!(key.source_key(), "Q.SE.N");
    }
}
