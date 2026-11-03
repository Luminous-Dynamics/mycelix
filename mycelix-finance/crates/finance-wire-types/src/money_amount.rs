// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Exact, versioned money wire representation.
//!
//! Atomic/minor-unit values remain canonical decimal strings at the JSON
//! boundary so JavaScript clients never coerce a u64 amount through a number.

use serde::{Deserialize, Serialize};

pub const SAP_MICRO_V1_PROFILE_ID: &str = "mycelix.finance.instrument.sap.micro.v1";
pub const TEND_V1_PROFILE_ID: &str = "mycelix.finance.instrument.tend.v1";
pub const MONEY_AMOUNT_V1_MAX_MINOR_UNITS: &str = "18446744073709551615";

/// An admitted instrument profile whose immutable ID fixes its unit and scale.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MoneyAmountProfileV1 {
    pub instrument_profile_id: &'static str,
    pub instrument_code: &'static str,
    pub atomic_unit: &'static str,
    pub decimal_scale: u8,
}

/// The only instrument profile admitted by MoneyAmountV1 v1.
///
/// SAP is the internal Finance ledger unit (micro-SAP). This profile is not a
/// ZAR profile, bank-deposit claim, custody statement or external settlement rail.
pub const SAP_MICRO_PROFILE_V1: MoneyAmountProfileV1 = MoneyAmountProfileV1 {
    instrument_profile_id: SAP_MICRO_V1_PROFILE_ID,
    instrument_code: "SAP",
    atomic_unit: "micro-SAP",
    decimal_scale: 6,
};

/// Resolve an immutable instrument profile from the explicit v1 allowlist.
/// TEND is syntactically representable but deliberately has no admitted scale.
pub fn resolve_money_amount_profile_v1(
    instrument_profile_id: &str,
) -> Option<MoneyAmountProfileV1> {
    match instrument_profile_id {
        SAP_MICRO_V1_PROFILE_ID => Some(SAP_MICRO_PROFILE_V1),
        _ => None,
    }
}

/// Stable error vocabulary shared with the TypeScript SDK.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MoneyAmountV1Error {
    InvalidEnvelope,
    InvalidProfileId,
    InvalidMinorUnits,
    NonCanonicalMinorUnits,
    OutOfRange,
    ProfileMismatch,
    Overflow,
    Underflow,
    UnsupportedProfile,
}

impl MoneyAmountV1Error {
    pub const fn code(self) -> &'static str {
        match self {
            Self::InvalidEnvelope => "InvalidEnvelope",
            Self::InvalidProfileId => "InvalidProfileId",
            Self::InvalidMinorUnits => "InvalidMinorUnits",
            Self::NonCanonicalMinorUnits => "NonCanonicalMinorUnits",
            Self::OutOfRange => "OutOfRange",
            Self::ProfileMismatch => "ProfileMismatch",
            Self::Overflow => "Overflow",
            Self::Underflow => "Underflow",
            Self::UnsupportedProfile => "UnsupportedProfile",
        }
    }
}

/// Version 1 exact amount envelope.
///
/// minor_units is a canonical non-negative base-10 integer string. It is
/// intentionally not a JSON number. The profile ID, rather than a currency
/// label supplied by a caller, binds the atomic unit and decimal scale.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MoneyAmountV1 {
    pub instrument_profile_id: String,
    pub minor_units: String,
}

impl MoneyAmountV1 {
    pub fn new(
        instrument_profile_id: impl Into<String>,
        minor_units: impl Into<String>,
    ) -> Result<Self, MoneyAmountV1Error> {
        let amount = Self {
            instrument_profile_id: instrument_profile_id.into(),
            minor_units: minor_units.into(),
        };
        amount.validate()?;
        Ok(amount)
    }

    pub fn from_minor_units(
        instrument_profile_id: impl Into<String>,
        minor_units: u64,
    ) -> Result<Self, MoneyAmountV1Error> {
        Self::new(instrument_profile_id, minor_units.to_string())
    }

    pub fn validate(&self) -> Result<(), MoneyAmountV1Error> {
        if !is_valid_profile_id(&self.instrument_profile_id) {
            return Err(MoneyAmountV1Error::InvalidProfileId);
        }
        parse_canonical_u64(&self.minor_units).map(|_| ())
    }

    pub fn minor_units_u64(&self) -> Result<u64, MoneyAmountV1Error> {
        self.validate()?;
        parse_canonical_u64(&self.minor_units)
    }

    pub fn is_positive(&self) -> Result<bool, MoneyAmountV1Error> {
        Ok(self.minor_units_u64()? > 0)
    }

    /// Require an exact profile and resolve it through the v1 admission list.
    pub fn require_profile(
        &self,
        expected_profile_id: &str,
    ) -> Result<MoneyAmountProfileV1, MoneyAmountV1Error> {
        self.validate()?;
        if self.instrument_profile_id != expected_profile_id {
            return Err(MoneyAmountV1Error::ProfileMismatch);
        }
        resolve_money_amount_profile_v1(&self.instrument_profile_id)
            .ok_or(MoneyAmountV1Error::UnsupportedProfile)
    }

    /// Checked addition. Values from different instruments can never be added.
    pub fn checked_add(&self, other: &Self) -> Result<Self, MoneyAmountV1Error> {
        self.validate()?;
        other.validate()?;
        if self.instrument_profile_id != other.instrument_profile_id {
            return Err(MoneyAmountV1Error::ProfileMismatch);
        }
        self.require_profile(&self.instrument_profile_id)?;
        let left = parse_canonical_u64(&self.minor_units)?;
        let right = parse_canonical_u64(&other.minor_units)?;
        let sum = left.checked_add(right).ok_or(MoneyAmountV1Error::Overflow)?;
        Self::from_minor_units(self.instrument_profile_id.clone(), sum)
    }

    /// Checked subtraction. Underflow is an error, never a negative payment.
    pub fn checked_sub(&self, other: &Self) -> Result<Self, MoneyAmountV1Error> {
        self.validate()?;
        other.validate()?;
        if self.instrument_profile_id != other.instrument_profile_id {
            return Err(MoneyAmountV1Error::ProfileMismatch);
        }
        self.require_profile(&self.instrument_profile_id)?;
        let left = parse_canonical_u64(&self.minor_units)?;
        let right = parse_canonical_u64(&other.minor_units)?;
        let difference = left
            .checked_sub(right)
            .ok_or(MoneyAmountV1Error::Underflow)?;
        Self::from_minor_units(self.instrument_profile_id.clone(), difference)
    }

    /// Format an amount using the scale of its admitted profile.
    pub fn to_decimal_string(&self) -> Result<String, MoneyAmountV1Error> {
        let profile = self.require_profile(&self.instrument_profile_id)?;
        let value = self.minor_units_u64()?;
        if profile.decimal_scale == 0 {
            return Ok(value.to_string());
        }
        let factor = 10_u64.pow(u32::from(profile.decimal_scale));
        let whole = value / factor;
        let fraction = value % factor;
        Ok(format!(
            "{}.{:0width$}",
            whole,
            fraction,
            width = usize::from(profile.decimal_scale)
        ))
    }
}

fn is_valid_profile_id(profile_id: &str) -> bool {
    if profile_id.is_empty() || profile_id.len() > 128 {
        return false;
    }
    let mut chars = profile_id.bytes();
    let Some(first) = chars.next() else {
        return false;
    };
    if !first.is_ascii_lowercase() && !first.is_ascii_digit() {
        return false;
    }
    chars.all(|byte| {
        byte.is_ascii_lowercase()
            || byte.is_ascii_digit()
            || matches!(byte, b'.' | b':' | b'-')
    })
}

fn parse_canonical_u64(value: &str) -> Result<u64, MoneyAmountV1Error> {
    if value.is_empty() || !value.bytes().all(|byte| byte.is_ascii_digit()) {
        return Err(MoneyAmountV1Error::InvalidMinorUnits);
    }
    if value.len() > 1 && value.starts_with('0') {
        return Err(MoneyAmountV1Error::NonCanonicalMinorUnits);
    }
    value
        .parse::<u64>()
        .map_err(|_| MoneyAmountV1Error::OutOfRange)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;

    #[derive(Debug, Deserialize)]
    struct GoldenCorpus {
        positive_vectors: Vec<PositiveVector>,
        negative_vectors: Vec<NegativeVector>,
        arithmetic_vectors: Vec<ArithmeticVector>,
        exact_json_roundtrip: String,
    }

    #[derive(Debug, Deserialize)]
    struct PositiveVector {
        id: String,
        profile: String,
        minor_units: String,
        parsed_u64: String,
        positive: bool,
    }

    #[derive(Debug, Deserialize)]
    struct NegativeVector {
        id: String,
        profile: Option<String>,
        minor_units: Option<String>,
        expected: String,
    }

    #[derive(Debug, Deserialize)]
    struct ArithmeticVector {
        id: String,
        left_profile: String,
        left: String,
        right_profile: String,
        right: String,
        expected: Option<String>,
        expected_error: Option<String>,
    }

    fn golden() -> GoldenCorpus {
        serde_json::from_str(include_str!("../../../docs/money-amount-golden-v1.json"))
            .expect("MoneyAmountV1 golden corpus must parse")
    }

    #[test]
    fn every_positive_golden_vector_is_canonical_and_exact() {
        let corpus = golden();
        for vector in corpus.positive_vectors {
            let amount = MoneyAmountV1::new(&vector.profile, &vector.minor_units)
                .unwrap_or_else(|error| panic!("{}: {}", vector.id, error.code()));
            assert_eq!(
                amount.minor_units_u64().unwrap().to_string(),
                vector.parsed_u64,
                "{}",
                vector.id
            );
            assert_eq!(amount.is_positive().unwrap(), vector.positive, "{}", vector.id);
        }
    }

    #[test]
    fn every_negative_golden_vector_fails_with_the_stable_code() {
        let corpus = golden();
        for vector in corpus.negative_vectors {
            let result = match (vector.profile, vector.minor_units) {
                (Some(profile), Some(minor_units)) => MoneyAmountV1::new(profile, minor_units),
                (None, _) => Err(MoneyAmountV1Error::InvalidProfileId),
                (_, None) => Err(MoneyAmountV1Error::InvalidMinorUnits),
            };
            assert_eq!(
                result.unwrap_err().code(),
                vector.expected,
                "{}",
                vector.id
            );
        }
    }

    #[test]
    fn every_arithmetic_golden_vector_matches_checked_semantics() {
        let corpus = golden();
        for vector in corpus.arithmetic_vectors {
            let left = MoneyAmountV1::new(&vector.left_profile, &vector.left).unwrap();
            let right = MoneyAmountV1::new(&vector.right_profile, &vector.right).unwrap();
            match (left.checked_add(&right), vector.expected, vector.expected_error) {
                (Ok(actual), Some(expected), None) => {
                    assert_eq!(actual.minor_units, expected, "{}", vector.id);
                }
                (Err(error), None, Some(expected_error)) => {
                    assert_eq!(error.code(), expected_error, "{}", vector.id);
                }
                (actual, expected, expected_error) => {
                    panic!(
                        "{}: unexpected result {actual:?}, expected {expected:?}/{expected_error:?}",
                        vector.id
                    );
                }
            }
        }
    }

    #[test]
    fn maximum_u64_json_roundtrip_stays_a_string() {
        let corpus = golden();
        let amount: MoneyAmountV1 =
            serde_json::from_str(&corpus.exact_json_roundtrip).expect("wire amount");
        assert_eq!(
            serde_json::to_string(&amount).expect("serialize amount"),
            corpus.exact_json_roundtrip
        );
        assert_eq!(amount.minor_units, MONEY_AMOUNT_V1_MAX_MINOR_UNITS);
    }

    #[test]
    fn unknown_fields_and_numeric_json_amounts_are_rejected() {
        let numeric = r#"{"instrument_profile_id":"mycelix.finance.instrument.sap.micro.v1","minor_units":1}"#;
        assert!(serde_json::from_str::<MoneyAmountV1>(numeric).is_err());
        let extra = r#"{"instrument_profile_id":"mycelix.finance.instrument.sap.micro.v1","minor_units":"1","amount":1}"#;
        assert!(serde_json::from_str::<MoneyAmountV1>(extra).is_err());
    }

    #[test]
    fn profile_substitution_and_unadmitted_tend_are_distinct() {
        let tend = MoneyAmountV1::new(TEND_V1_PROFILE_ID, "1").unwrap();
        assert_eq!(
            tend.require_profile(SAP_MICRO_V1_PROFILE_ID).unwrap_err(),
            MoneyAmountV1Error::ProfileMismatch
        );
        assert_eq!(
            tend.require_profile(TEND_V1_PROFILE_ID).unwrap_err(),
            MoneyAmountV1Error::UnsupportedProfile
        );
    }

    #[test]
    fn checked_subtraction_rejects_underflow() {
        let one = MoneyAmountV1::new(SAP_MICRO_V1_PROFILE_ID, "1").unwrap();
        let two = MoneyAmountV1::new(SAP_MICRO_V1_PROFILE_ID, "2").unwrap();
        assert_eq!(
            one.checked_sub(&two).unwrap_err(),
            MoneyAmountV1Error::Underflow
        );
    }

    #[test]
    fn sap_decimal_formatting_uses_admitted_six_digit_scale() {
        let one_sap = MoneyAmountV1::new(SAP_MICRO_V1_PROFILE_ID, "1000000").unwrap();
        assert_eq!(one_sap.to_decimal_string().unwrap(), "1.000000");
    }
}
