#![deny(unsafe_code)]

//! Exact authoritative arithmetic primitives for Mycelix Finance.
//!
//! These types are intentionally dependency-light and do not depend on HDK/HDI.
//! They are designed for persisted financial amounts, reservations, exposure,
//! settlement amounts, and exact contractual/policy rates where binary floating
//! point would make authoritative state ambiguous.
//!
//! Analytical models may continue to use floating point. Converting model output
//! into authoritative Finance state requires an explicit boundary and rounding
//! policy outside this crate.

use serde::{Deserialize, Deserializer, Serialize};
use std::fmt;

/// Maximum UTF-8 byte length for an asset identifier.
pub const MAX_ASSET_ID_BYTES: usize = 128;

/// Error returned by exact authoritative arithmetic operations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExactArithmeticError {
    InvalidAssetId,
    AssetMismatch,
    Overflow,
    Underflow,
    DivisionByZero,
}

impl fmt::Display for ExactArithmeticError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidAssetId => {
                "asset identifier is empty, too long, or contains control characters"
            }
            Self::AssetMismatch => "asset identifiers do not match",
            Self::Overflow => "exact arithmetic overflow",
            Self::Underflow => "exact arithmetic underflow",
            Self::DivisionByZero => "division by zero",
        };
        f.write_str(message)
    }
}

impl std::error::Error for ExactArithmeticError {}

/// Canonical identifier for an asset/currency used by an exact financial amount.
///
/// This type deliberately does not prescribe a global currency registry. It only
/// guarantees that the identifier is non-empty, bounded, and free of control
/// characters so it can safely participate in deterministic equality checks.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(transparent)]
pub struct AssetId(String);

impl AssetId {
    pub fn new(value: impl Into<String>) -> Result<Self, ExactArithmeticError> {
        let value = value.into();
        if value.is_empty()
            || value.len() > MAX_ASSET_ID_BYTES
            || value.chars().any(char::is_control)
        {
            return Err(ExactArithmeticError::InvalidAssetId);
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    pub fn into_inner(self) -> String {
        self.0
    }
}

impl fmt::Display for AssetId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl TryFrom<&str> for AssetId {
    type Error = ExactArithmeticError;

    fn try_from(value: &str) -> Result<Self, Self::Error> {
        Self::new(value)
    }
}

impl TryFrom<String> for AssetId {
    type Error = ExactArithmeticError;

    fn try_from(value: String) -> Result<Self, Self::Error> {
        Self::new(value)
    }
}

impl<'de> Deserialize<'de> for AssetId {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let value = String::deserialize(deserializer)?;
        Self::new(value).map_err(serde::de::Error::custom)
    }
}

/// Explicit rounding mode for conversions from an exact rational result into
/// integer atomic units.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RoundingMode {
    /// Discard any fractional remainder.
    Floor,
    /// Round upward whenever a non-zero remainder exists.
    Ceiling,
    /// Round to nearest; exact ties go to the even integer.
    HalfEven,
}

/// Canonical non-negative exact rational rate.
///
/// Values are normalized on construction and deserialization, so `2/4` becomes
/// `1/2`. The denominator is always greater than zero.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub struct Rate {
    numerator: u64,
    denominator: u64,
}

impl Rate {
    pub fn new(numerator: u64, denominator: u64) -> Result<Self, ExactArithmeticError> {
        if denominator == 0 {
            return Err(ExactArithmeticError::DivisionByZero);
        }

        let divisor = gcd_u64(numerator, denominator);
        Ok(Self {
            numerator: numerator / divisor,
            denominator: denominator / divisor,
        })
    }

    pub fn zero() -> Self {
        Self {
            numerator: 0,
            denominator: 1,
        }
    }

    pub fn one() -> Self {
        Self {
            numerator: 1,
            denominator: 1,
        }
    }

    pub fn from_basis_points(basis_points: u64) -> Result<Self, ExactArithmeticError> {
        Self::new(basis_points, 10_000)
    }

    pub fn numerator(self) -> u64 {
        self.numerator
    }

    pub fn denominator(self) -> u64 {
        self.denominator
    }

    /// Multiply two exact rates using cross-reduction before multiplication to
    /// reduce avoidable overflow.
    pub fn checked_mul(self, other: Self) -> Result<Self, ExactArithmeticError> {
        let cross_left = gcd_u64(self.numerator, other.denominator);
        let cross_right = gcd_u64(other.numerator, self.denominator);

        let left_numerator = self.numerator / cross_left;
        let right_denominator = other.denominator / cross_left;
        let right_numerator = other.numerator / cross_right;
        let left_denominator = self.denominator / cross_right;

        let numerator = left_numerator
            .checked_mul(right_numerator)
            .ok_or(ExactArithmeticError::Overflow)?;
        let denominator = left_denominator
            .checked_mul(right_denominator)
            .ok_or(ExactArithmeticError::Overflow)?;

        Self::new(numerator, denominator)
    }

    /// Divide this rate by another rate.
    pub fn checked_div(self, other: Self) -> Result<Self, ExactArithmeticError> {
        if other.numerator == 0 {
            return Err(ExactArithmeticError::DivisionByZero);
        }

        let numerator_cross = gcd_u64(self.numerator, other.numerator);
        let denominator_cross = gcd_u64(other.denominator, self.denominator);

        let left_numerator = self.numerator / numerator_cross;
        let right_numerator = other.numerator / numerator_cross;
        let right_denominator = other.denominator / denominator_cross;
        let left_denominator = self.denominator / denominator_cross;

        let numerator = left_numerator
            .checked_mul(right_denominator)
            .ok_or(ExactArithmeticError::Overflow)?;
        let denominator = left_denominator
            .checked_mul(right_numerator)
            .ok_or(ExactArithmeticError::Overflow)?;

        Self::new(numerator, denominator)
    }
}

#[derive(Deserialize)]
struct RateWire {
    numerator: u64,
    denominator: u64,
}

impl<'de> Deserialize<'de> for Rate {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let wire = RateWire::deserialize(deserializer)?;
        Self::new(wire.numerator, wire.denominator).map_err(serde::de::Error::custom)
    }
}

/// Unsigned exact amount in atomic units of one asset.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AssetAmount {
    atomic_units: u64,
    asset: AssetId,
}

impl AssetAmount {
    pub fn new(atomic_units: u64, asset: AssetId) -> Self {
        Self {
            atomic_units,
            asset,
        }
    }

    pub fn atomic_units(&self) -> u64 {
        self.atomic_units
    }

    pub fn asset(&self) -> &AssetId {
        &self.asset
    }

    pub fn checked_add(&self, other: &Self) -> Result<Self, ExactArithmeticError> {
        self.ensure_same_asset(other)?;
        let atomic_units = self
            .atomic_units
            .checked_add(other.atomic_units)
            .ok_or(ExactArithmeticError::Overflow)?;
        Ok(Self::new(atomic_units, self.asset.clone()))
    }

    pub fn checked_sub(&self, other: &Self) -> Result<Self, ExactArithmeticError> {
        self.ensure_same_asset(other)?;
        let atomic_units = self
            .atomic_units
            .checked_sub(other.atomic_units)
            .ok_or(ExactArithmeticError::Underflow)?;
        Ok(Self::new(atomic_units, self.asset.clone()))
    }

    /// Apply an exact rate to this amount using an explicit rounding policy.
    pub fn checked_apply_rate(
        &self,
        rate: Rate,
        rounding: RoundingMode,
    ) -> Result<Self, ExactArithmeticError> {
        let product = u128::from(self.atomic_units)
            .checked_mul(u128::from(rate.numerator))
            .ok_or(ExactArithmeticError::Overflow)?;
        let rounded = divide_round(product, u128::from(rate.denominator), rounding)?;
        let atomic_units = u64::try_from(rounded).map_err(|_| ExactArithmeticError::Overflow)?;
        Ok(Self::new(atomic_units, self.asset.clone()))
    }

    /// Divide this amount by a non-zero exact rate using an explicit rounding policy.
    pub fn checked_divide_by_rate(
        &self,
        rate: Rate,
        rounding: RoundingMode,
    ) -> Result<Self, ExactArithmeticError> {
        if rate.numerator == 0 {
            return Err(ExactArithmeticError::DivisionByZero);
        }

        let product = u128::from(self.atomic_units)
            .checked_mul(u128::from(rate.denominator))
            .ok_or(ExactArithmeticError::Overflow)?;
        let rounded = divide_round(product, u128::from(rate.numerator), rounding)?;
        let atomic_units = u64::try_from(rounded).map_err(|_| ExactArithmeticError::Overflow)?;
        Ok(Self::new(atomic_units, self.asset.clone()))
    }

    fn ensure_same_asset(&self, other: &Self) -> Result<(), ExactArithmeticError> {
        if self.asset == other.asset {
            Ok(())
        } else {
            Err(ExactArithmeticError::AssetMismatch)
        }
    }
}

/// Signed exact amount for mutual-credit and exposure domains.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SignedAssetAmount {
    atomic_units: i64,
    asset: AssetId,
}

impl SignedAssetAmount {
    pub fn new(atomic_units: i64, asset: AssetId) -> Self {
        Self {
            atomic_units,
            asset,
        }
    }

    pub fn atomic_units(&self) -> i64 {
        self.atomic_units
    }

    pub fn asset(&self) -> &AssetId {
        &self.asset
    }

    pub fn checked_add(&self, other: &Self) -> Result<Self, ExactArithmeticError> {
        self.ensure_same_asset(other)?;
        let atomic_units = self
            .atomic_units
            .checked_add(other.atomic_units)
            .ok_or(ExactArithmeticError::Overflow)?;
        Ok(Self::new(atomic_units, self.asset.clone()))
    }

    pub fn checked_sub(&self, other: &Self) -> Result<Self, ExactArithmeticError> {
        self.ensure_same_asset(other)?;
        let atomic_units = self
            .atomic_units
            .checked_sub(other.atomic_units)
            .ok_or(ExactArithmeticError::Overflow)?;
        Ok(Self::new(atomic_units, self.asset.clone()))
    }

    fn ensure_same_asset(&self, other: &Self) -> Result<(), ExactArithmeticError> {
        if self.asset == other.asset {
            Ok(())
        } else {
            Err(ExactArithmeticError::AssetMismatch)
        }
    }
}

fn gcd_u64(mut left: u64, mut right: u64) -> u64 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left
}

fn divide_round(
    numerator: u128,
    denominator: u128,
    rounding: RoundingMode,
) -> Result<u128, ExactArithmeticError> {
    if denominator == 0 {
        return Err(ExactArithmeticError::DivisionByZero);
    }

    let quotient = numerator / denominator;
    let remainder = numerator % denominator;

    match rounding {
        RoundingMode::Floor => Ok(quotient),
        RoundingMode::Ceiling => {
            if remainder == 0 {
                Ok(quotient)
            } else {
                quotient
                    .checked_add(1)
                    .ok_or(ExactArithmeticError::Overflow)
            }
        }
        RoundingMode::HalfEven => {
            let twice_remainder = remainder
                .checked_mul(2)
                .ok_or(ExactArithmeticError::Overflow)?;
            if twice_remainder < denominator {
                Ok(quotient)
            } else if twice_remainder > denominator || quotient % 2 == 1 {
                quotient
                    .checked_add(1)
                    .ok_or(ExactArithmeticError::Overflow)
            } else {
                Ok(quotient)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn sap() -> AssetId {
        AssetId::new("SAP").expect("static asset id")
    }

    fn tend() -> AssetId {
        AssetId::new("TEND").expect("static asset id")
    }

    #[test]
    fn asset_id_rejects_invalid_values() {
        assert_eq!(AssetId::new(""), Err(ExactArithmeticError::InvalidAssetId));
        assert_eq!(
            AssetId::new("SAP\nOTHER"),
            Err(ExactArithmeticError::InvalidAssetId)
        );
        assert_eq!(
            AssetId::new("x".repeat(MAX_ASSET_ID_BYTES + 1)),
            Err(ExactArithmeticError::InvalidAssetId)
        );
    }

    #[test]
    fn rate_is_canonicalized() {
        let rate = Rate::new(2, 4).expect("valid rate");
        assert_eq!(rate.numerator(), 1);
        assert_eq!(rate.denominator(), 2);
        assert_eq!(Rate::new(0, 99).expect("valid zero rate"), Rate::zero());
    }

    #[test]
    fn rate_rejects_zero_denominator() {
        assert_eq!(Rate::new(1, 0), Err(ExactArithmeticError::DivisionByZero));
    }

    #[test]
    fn exact_asset_addition_rejects_mismatch() {
        let left = AssetAmount::new(10, sap());
        let right = AssetAmount::new(10, tend());
        assert_eq!(
            left.checked_add(&right),
            Err(ExactArithmeticError::AssetMismatch)
        );
    }

    #[test]
    fn exact_asset_addition_detects_overflow() {
        let left = AssetAmount::new(u64::MAX, sap());
        let right = AssetAmount::new(1, sap());
        assert_eq!(
            left.checked_add(&right),
            Err(ExactArithmeticError::Overflow)
        );
    }

    #[test]
    fn exact_asset_subtraction_detects_underflow() {
        let left = AssetAmount::new(0, sap());
        let right = AssetAmount::new(1, sap());
        assert_eq!(
            left.checked_sub(&right),
            Err(ExactArithmeticError::Underflow)
        );
    }

    #[test]
    fn half_even_rounding_is_deterministic() {
        let half = Rate::new(1, 2).expect("valid rate");

        let one = AssetAmount::new(1, sap())
            .checked_apply_rate(half, RoundingMode::HalfEven)
            .expect("exact operation");
        assert_eq!(one.atomic_units(), 0);

        let three = AssetAmount::new(3, sap())
            .checked_apply_rate(half, RoundingMode::HalfEven)
            .expect("exact operation");
        assert_eq!(three.atomic_units(), 2);

        let five = AssetAmount::new(5, sap())
            .checked_apply_rate(half, RoundingMode::HalfEven)
            .expect("exact operation");
        assert_eq!(five.atomic_units(), 2);
    }

    #[test]
    fn floor_and_ceiling_are_explicit() {
        let third = Rate::new(1, 3).expect("valid rate");
        let amount = AssetAmount::new(10, sap());

        assert_eq!(
            amount
                .checked_apply_rate(third, RoundingMode::Floor)
                .expect("exact operation")
                .atomic_units(),
            3
        );
        assert_eq!(
            amount
                .checked_apply_rate(third, RoundingMode::Ceiling)
                .expect("exact operation")
                .atomic_units(),
            4
        );
    }

    #[test]
    fn rate_multiplication_cross_reduces() {
        let left = Rate::new(u64::MAX, 2).expect("valid rate");
        let right = Rate::new(2, u64::MAX).expect("valid rate");
        assert_eq!(left.checked_mul(right).expect("cross reduced"), Rate::one());
    }

    #[test]
    fn rate_division_rejects_zero_rate() {
        assert_eq!(
            Rate::one().checked_div(Rate::zero()),
            Err(ExactArithmeticError::DivisionByZero)
        );
    }

    #[test]
    fn serde_round_trip_preserves_exact_values() {
        let amount = AssetAmount::new(12_345_678, sap());
        let encoded = serde_json::to_string(&amount).expect("serialize");
        let decoded: AssetAmount = serde_json::from_str(&encoded).expect("deserialize");
        assert_eq!(decoded, amount);

        let noncanonical: Rate = serde_json::from_str("{\"numerator\":2,\"denominator\":4}")
            .expect("deserialize and normalize");
        assert_eq!(noncanonical, Rate::new(1, 2).expect("valid rate"));
    }

    #[test]
    fn serde_rejects_invalid_rate_and_asset() {
        let bad_rate = serde_json::from_str::<Rate>("{\"numerator\":1,\"denominator\":0}");
        assert!(bad_rate.is_err());

        let bad_asset = serde_json::from_str::<AssetId>("\"\"");
        assert!(bad_asset.is_err());
    }

    proptest! {
        #[test]
        fn add_then_sub_round_trips(left in any::<u64>(), right in any::<u64>()) {
            prop_assume!(left.checked_add(right).is_some());
            let left_amount = AssetAmount::new(left, sap());
            let right_amount = AssetAmount::new(right, sap());
            let sum = left_amount.checked_add(&right_amount).expect("bounded by assumption");
            let restored = sum.checked_sub(&right_amount).expect("cannot underflow");
            prop_assert_eq!(restored, left_amount);
        }

        #[test]
        fn signed_add_then_sub_round_trips(left in any::<i64>(), right in any::<i64>()) {
            prop_assume!(left.checked_add(right).is_some());
            let left_amount = SignedAssetAmount::new(left, tend());
            let right_amount = SignedAssetAmount::new(right, tend());
            let sum = left_amount.checked_add(&right_amount).expect("bounded by assumption");
            let restored = sum.checked_sub(&right_amount).expect("inverse must fit");
            prop_assert_eq!(restored, left_amount);
        }

        #[test]
        fn normalized_rate_round_trips(numerator in any::<u64>(), denominator in 1_u64..=u64::MAX) {
            let rate = Rate::new(numerator, denominator).expect("non-zero denominator");
            let encoded = serde_json::to_string(&rate).expect("serialize");
            let decoded: Rate = serde_json::from_str(&encoded).expect("deserialize");
            prop_assert_eq!(decoded, rate);
        }
    }
}
