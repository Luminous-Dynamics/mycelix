//! Canonical fixed-point arithmetic for monetary valuation boundaries.
//!
//! Floating-point values may be useful for analytics, but they must not be
//! converted directly into μSAP. This module provides a deterministic integer
//! representation and explicit rounding policy for monetary conversions.

use serde::{Deserialize, Serialize};

/// Canonical rounding policy for converting a rational amount into μSAP.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum MonetaryRounding {
    /// Always round toward zero for non-negative quantities.
    Down,
    /// Round up whenever a non-zero remainder exists.
    Up,
    /// Round to nearest integer, with exact half values rounded upward.
    HalfUp,
}

/// A canonical non-negative rational rate.
///
/// The rate represents numerator / denominator.
/// Canonical construction reduces the fraction to lowest terms.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FixedRate {
    pub numerator: u64,
    pub denominator: u64,
}

impl FixedRate {
    /// Construct a canonical positive rational rate.
    pub fn new(numerator: u64, denominator: u64) -> Result<Self, FixedRateError> {
        if denominator == 0 {
            return Err(FixedRateError::ZeroDenominator);
        }
        if numerator == 0 {
            return Err(FixedRateError::ZeroNumerator);
        }
        let divisor = gcd(numerator, denominator);
        Ok(Self {
            numerator: numerator / divisor,
            denominator: denominator / divisor,
        })
    }

    /// Parse a non-negative decimal rate without going through f32/f64.
    /// Up to 18 fractional decimal places are accepted.
    pub fn from_decimal(value: &str) -> Result<Self, FixedRateError> {
        let trimmed = value.trim();
        if trimmed.is_empty() {
            return Err(FixedRateError::InvalidDecimal);
        }
        if trimmed.starts_with('-') || trimmed.contains(['e', 'E', '+']) {
            return Err(FixedRateError::InvalidDecimal);
        }
        let mut parts = trimmed.split('.');
        let whole = parts.next().ok_or(FixedRateError::InvalidDecimal)?;
        let fraction = parts.next().unwrap_or_default();
        if parts.next().is_some()
            || whole.is_empty()
            || !whole.as_bytes().iter().all(u8::is_ascii_digit)
            || !fraction.as_bytes().iter().all(u8::is_ascii_digit)
            || fraction.len() > 18
        {
            return Err(FixedRateError::InvalidDecimal);
        }
        let whole_value = whole.parse::<u64>().map_err(|_| FixedRateError::InvalidDecimal)?;
        if fraction.is_empty() {
            return Self::new(whole_value, 1);
        }
        let scale = 10u64
            .checked_pow(fraction.len() as u32)
            .ok_or(FixedRateError::InvalidDecimal)?;
        let fraction_value = fraction
            .parse::<u64>()
            .map_err(|_| FixedRateError::InvalidDecimal)?;
        let numerator = (whole_value as u128)
            .checked_mul(scale as u128)
            .and_then(|x| x.checked_add(fraction_value as u128))
            .ok_or(FixedRateError::Overflow)?;
        if numerator > u64::MAX as u128 {
            return Err(FixedRateError::Overflow);
        }
        Self::new(numerator as u64, scale)
    }

    /// Apply the rate to a non-negative integer quantity using explicit rounding.
    pub fn apply(
        &self,
        quantity: u64,
        rounding: MonetaryRounding,
    ) -> Result<u64, FixedRateError> {
        let product = (quantity as u128)
            .checked_mul(self.numerator as u128)
            .ok_or(FixedRateError::Overflow)?;
        let denominator = self.denominator as u128;
        let quotient = product / denominator;
        let remainder = product % denominator;
        let rounded = match rounding {
            MonetaryRounding::Down => quotient,
            MonetaryRounding::Up if remainder == 0 => quotient,
            MonetaryRounding::Up => quotient + 1,
            MonetaryRounding::HalfUp => {
                if remainder.saturating_mul(2) >= denominator {
                    quotient + 1
                } else {
                    quotient
                }
            }
        };
        u64::try_from(rounded).map_err(|_| FixedRateError::Overflow)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FixedRateError {
    ZeroNumerator,
    ZeroDenominator,
    InvalidDecimal,
    Overflow,
}

impl core::fmt::Display for FixedRateError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let message = match self {
            Self::ZeroNumerator => "rate numerator must be positive",
            Self::ZeroDenominator => "rate denominator must be non-zero",
            Self::InvalidDecimal => "invalid fixed-point decimal rate",
            Self::Overflow => "fixed-point monetary calculation overflowed",
        };
        f.write_str(message)
    }
}

impl std::error::Error for FixedRateError {}

const fn gcd(mut a: u64, mut b: u64) -> u64 {
    while b != 0 {
        let remainder = a % b;
        a = b;
        b = remainder;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decimal_rates_are_canonicalized() {
        assert_eq!(
            FixedRate::from_decimal("2.500").unwrap(),
            FixedRate { numerator: 5, denominator: 2 }
        );
    }

    #[test]
    fn zero_denominator_is_rejected() {
        assert_eq!(FixedRate::new(1, 0), Err(FixedRateError::ZeroDenominator));
    }

    #[test]
    fn zero_numerator_is_rejected() {
        assert_eq!(FixedRate::new(0, 1), Err(FixedRateError::ZeroNumerator));
    }

    #[test]
    fn malformed_decimal_is_rejected_without_float_parsing() {
        for value in ["", "-1", "1e-3", "1E3", "1+2", "1.2345678901234567890"] {
            assert_eq!(
                FixedRate::from_decimal(value),
                Err(FixedRateError::InvalidDecimal),
                "unexpected acceptance: {value}"
            );
        }
    }

    #[test]
    fn exact_integer_conversion_is_stable() {
        let rate = FixedRate::from_decimal("2.0").unwrap();
        assert_eq!(rate.apply(1_000, MonetaryRounding::Down), Ok(2_000));
        assert_eq!(rate.apply(1_000, MonetaryRounding::Up), Ok(2_000));
        assert_eq!(rate.apply(1_000, MonetaryRounding::HalfUp), Ok(2_000));
    }

    #[test]
    fn rounding_policy_is_explicit() {
        let rate = FixedRate::new(1, 3).unwrap();
        assert_eq!(rate.apply(1, MonetaryRounding::Down), Ok(0));
        assert_eq!(rate.apply(1, MonetaryRounding::Up), Ok(1));
        assert_eq!(rate.apply(1, MonetaryRounding::HalfUp), Ok(0));
        let half = FixedRate::new(1, 2).unwrap();
        assert_eq!(half.apply(1, MonetaryRounding::HalfUp), Ok(1));
    }

    #[test]
    fn overflow_is_rejected_at_output_boundary() {
        let rate = FixedRate::new(u64::MAX, 1).unwrap();
        assert_eq!(
            rate.apply(u64::MAX, MonetaryRounding::Down),
            Err(FixedRateError::Overflow)
        );
    }

    #[test]
    fn decimal_underflow_is_not_silently_rounded_to_zero_rate() {
        assert_eq!(
            FixedRate::from_decimal("0.000000000000000000"),
            Err(FixedRateError::ZeroNumerator)
        );
    }

    #[test]
    fn integer_arithmetic_is_replay_stable() {
        let rate = FixedRate::from_decimal("1.234567890123456789").unwrap();
        let a = rate.apply(987_654_321, MonetaryRounding::HalfUp);
        let b = rate.apply(987_654_321, MonetaryRounding::HalfUp);
        assert_eq!(a, b);
    }
}
