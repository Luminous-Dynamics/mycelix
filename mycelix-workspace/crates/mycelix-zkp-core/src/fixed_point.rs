// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fixed-point arithmetic shared across ZKP backends.
//!
//! The historical representation uses a signed `i64` raw value with 16 fractional
//! bits (`scale = 2^16`). It has commonly been called "Q16.16" in this codebase,
//! but the `i64` storage provides a wider integer range than a strict 32-bit
//! Q16.16 representation. New proof profiles should bind the representation
//! explicitly as `signed-i64 / 16 fractional bits` rather than infer width from
//! the historical name.
//!
//! RISC0 guest programs and Winterfell AIR circuits use this representation to
//! avoid floating-point non-determinism in proofs.

use serde::{Deserialize, Serialize};

/// Fixed-point scale factor: 2^16 = 65536.
pub const Q16_16_SCALE: u64 = 65_536;

// `i64::MAX as f64` rounds to 2^63, so use the exact exclusive upper boundary
// rather than comparing against that rounded conversion.
const I64_UPPER_EXCLUSIVE_F64: f64 = 9_223_372_036_854_775_808.0; // 2^63
const I64_LOWER_INCLUSIVE_F64: f64 = -9_223_372_036_854_775_808.0; // -2^63

/// Failure while converting floating-point input into the proof fixed-point domain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum FixedPointConversionError {
    /// NaN and +/-infinity are never valid proof witnesses.
    #[error("floating-point proof witness must be finite")]
    NonFinite,
    /// The scaled value cannot be represented by the signed i64 raw domain.
    #[error("scaled proof witness is outside the signed i64 fixed-point domain")]
    OutOfRange,
}

/// Indexed failure for checked slice conversion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error("invalid proof witness at index {index}: {source}")]
pub struct FixedPointSliceConversionError {
    pub index: usize,
    #[source]
    pub source: FixedPointConversionError,
}

/// A signed fixed-point value backed by `i64` with 16 fractional bits.
#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
pub struct FixedPoint(pub i64);

impl FixedPoint {
    /// Create from an f32 value using the historical unchecked compatibility path.
    ///
    /// This method intentionally preserves legacy behavior. It must not be used
    /// to admit untrusted/authority-bearing proof witnesses because float-to-int
    /// conversion can coerce non-finite or out-of-range values.
    ///
    /// Use [`Self::try_from_f32`] for new proof-bearing code.
    pub fn from_f32(v: f32) -> Self {
        Self((v * Q16_16_SCALE as f32) as i64)
    }

    /// Create from an f64 value using the historical unchecked compatibility path.
    ///
    /// Use [`Self::try_from_f64`] for new proof-bearing code.
    pub fn from_f64(v: f64) -> Self {
        Self((v * Q16_16_SCALE as f64) as i64)
    }

    /// Checked conversion from f32 for authority-bearing proof witnesses.
    pub fn try_from_f32(v: f32) -> Result<Self, FixedPointConversionError> {
        Self::try_from_f64(v as f64)
    }

    /// Checked conversion from f64 for authority-bearing proof witnesses.
    pub fn try_from_f64(v: f64) -> Result<Self, FixedPointConversionError> {
        if !v.is_finite() {
            return Err(FixedPointConversionError::NonFinite);
        }

        let scaled = v * Q16_16_SCALE as f64;
        if !scaled.is_finite()
            || !(I64_LOWER_INCLUSIVE_F64..I64_UPPER_EXCLUSIVE_F64).contains(&scaled)
        {
            return Err(FixedPointConversionError::OutOfRange);
        }

        // The legacy representation truncates toward zero. Preserve that numeric
        // convention after proving the value is inside the representable domain.
        Ok(Self(scaled as i64))
    }

    /// Create from a raw scaled integer.
    pub const fn from_raw(raw: i64) -> Self {
        Self(raw)
    }

    /// Convert back to f32.
    pub fn to_f32(self) -> f32 {
        self.0 as f32 / Q16_16_SCALE as f32
    }

    /// Convert back to f64.
    pub fn to_f64(self) -> f64 {
        self.0 as f64 / Q16_16_SCALE as f64
    }

    /// Get the raw scaled integer.
    pub const fn raw(self) -> i64 {
        self.0
    }

    /// Represent 1.0 in the fixed-point encoding.
    pub const ONE: Self = Self(Q16_16_SCALE as i64);

    /// Represent 0.0 in the fixed-point encoding.
    pub const ZERO: Self = Self(0);
}

impl std::ops::Mul for FixedPoint {
    type Output = Self;

    /// Fixed-point multiply: (a * b) >> 16
    fn mul(self, other: Self) -> Self {
        Self((self.0 as i128 * other.0 as i128 / Q16_16_SCALE as i128) as i64)
    }
}

impl std::ops::Div for FixedPoint {
    type Output = Self;

    /// Fixed-point divide: (a << 16) / b
    fn div(self, other: Self) -> Self {
        if other.0 == 0 {
            return Self(i64::MAX); // Saturate on division by zero
        }
        Self((self.0 as i128 * Q16_16_SCALE as i128 / other.0 as i128) as i64)
    }
}

impl std::ops::Add for FixedPoint {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self(self.0.saturating_add(rhs.0))
    }
}

impl std::ops::Sub for FixedPoint {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self(self.0.saturating_sub(rhs.0))
    }
}

/// Convert a slice of f32 values using the historical unchecked compatibility path.
///
/// New proof-bearing code should use [`try_f32_slice_to_fixed`].
pub fn f32_slice_to_fixed(values: &[f32]) -> Vec<FixedPoint> {
    values.iter().map(|&v| FixedPoint::from_f32(v)).collect()
}

/// Convert an f32 slice using checked proof-witness admission.
pub fn try_f32_slice_to_fixed(
    values: &[f32],
) -> Result<Vec<FixedPoint>, FixedPointSliceConversionError> {
    values
        .iter()
        .enumerate()
        .map(|(index, &value)| {
            FixedPoint::try_from_f32(value)
                .map_err(|source| FixedPointSliceConversionError { index, source })
        })
        .collect()
}

/// Convert fixed-point values back to f32.
pub fn fixed_to_f32_slice(values: &[FixedPoint]) -> Vec<f32> {
    values.iter().map(|v| v.to_f32()).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_roundtrip_f32() {
        let v = 0.85f32;
        let fp = FixedPoint::from_f32(v);
        let back = fp.to_f32();
        assert!(
            (v - back).abs() < 0.001,
            "roundtrip: {} -> {} -> {}",
            v,
            fp.0,
            back
        );
    }

    #[test]
    fn test_checked_roundtrip_f32() {
        let v = -0.85f32;
        let fp = FixedPoint::try_from_f32(v).expect("finite representable value");
        assert!((v - fp.to_f32()).abs() < 0.001);
    }

    #[test]
    fn test_checked_conversion_rejects_non_finite() {
        assert_eq!(
            FixedPoint::try_from_f32(f32::NAN),
            Err(FixedPointConversionError::NonFinite)
        );
        assert_eq!(
            FixedPoint::try_from_f32(f32::INFINITY),
            Err(FixedPointConversionError::NonFinite)
        );
        assert_eq!(
            FixedPoint::try_from_f32(f32::NEG_INFINITY),
            Err(FixedPointConversionError::NonFinite)
        );
        assert_eq!(
            FixedPoint::try_from_f64(f64::NAN),
            Err(FixedPointConversionError::NonFinite)
        );
    }

    #[test]
    fn test_checked_conversion_rejects_scaled_overflow() {
        assert_eq!(
            FixedPoint::try_from_f64(f64::MAX),
            Err(FixedPointConversionError::OutOfRange)
        );
        assert_eq!(
            FixedPoint::try_from_f64(-f64::MAX),
            Err(FixedPointConversionError::OutOfRange)
        );

        let upper_exclusive = I64_UPPER_EXCLUSIVE_F64 / Q16_16_SCALE as f64;
        assert_eq!(
            FixedPoint::try_from_f64(upper_exclusive),
            Err(FixedPointConversionError::OutOfRange)
        );
    }

    #[test]
    fn test_checked_conversion_accepts_lower_boundary() {
        let lower = I64_LOWER_INCLUSIVE_F64 / Q16_16_SCALE as f64;
        let value = FixedPoint::try_from_f64(lower).expect("i64::MIN is representable");
        assert_eq!(value.raw(), i64::MIN);
    }

    #[test]
    fn test_checked_slice_reports_index() {
        let err = try_f32_slice_to_fixed(&[0.1, 0.2, f32::NAN, 0.4]).unwrap_err();
        assert_eq!(err.index, 2);
        assert_eq!(err.source, FixedPointConversionError::NonFinite);
    }

    #[test]
    fn test_fixed_multiply() {
        let a = FixedPoint::from_f32(0.5);
        let b = FixedPoint::from_f32(0.8);
        let c = a * b;
        assert!((c.to_f32() - 0.4).abs() < 0.001);
    }

    #[test]
    fn test_fixed_divide() {
        let a = FixedPoint::from_f32(1.0);
        let b = FixedPoint::from_f32(2.0);
        let c = a / b;
        assert!((c.to_f32() - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_one_and_zero() {
        assert_eq!(FixedPoint::ONE.to_f32(), 1.0);
        assert_eq!(FixedPoint::ZERO.to_f32(), 0.0);
    }

    #[test]
    fn test_slice_conversion() {
        let values = vec![0.1f32, 0.5, 0.9, 1.0];
        let fixed = f32_slice_to_fixed(&values);
        let back = fixed_to_f32_slice(&fixed);
        for (orig, conv) in values.iter().zip(back.iter()) {
            assert!((orig - conv).abs() < 0.001);
        }
    }

    #[test]
    fn test_checked_slice_conversion() {
        let values = vec![-0.1f32, 0.5, 0.9, 1.0];
        let fixed = try_f32_slice_to_fixed(&values).expect("valid witness vector");
        let back = fixed_to_f32_slice(&fixed);
        for (orig, conv) in values.iter().zip(back.iter()) {
            assert!((orig - conv).abs() < 0.001);
        }
    }
}
