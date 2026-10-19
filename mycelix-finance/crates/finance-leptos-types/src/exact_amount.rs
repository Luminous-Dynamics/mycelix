// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact decimal presentation for integer-denominated financial amounts.
//!
//! This module deliberately avoids floating-point conversion. It is intended
//! for UI/reporting boundaries where a monetary quantity must remain exactly
//! representable and deterministically formatted.

use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ExactMinorAmount {
    minor_units: i128,
    scale: u8,
}

impl ExactMinorAmount {
    pub const MAX_SCALE: u8 = 38;

    pub const fn new(minor_units: i128, scale: u8) -> Self {
        Self { minor_units, scale }
    }

    pub const fn minor_units(self) -> i128 {
        self.minor_units
    }

    pub const fn scale(self) -> u8 {
        self.scale
    }
}

impl fmt::Display for ExactMinorAmount {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.scale > Self::MAX_SCALE {
            return Err(fmt::Error);
        }

        let magnitude = self.minor_units.unsigned_abs();
        let factor = 10u128.pow(self.scale as u32);

        if self.minor_units < 0 {
            f.write_str("-")?;
        }

        if self.scale == 0 {
            return write!(f, "{magnitude}");
        }

        let whole = magnitude / factor;
        let fraction = magnitude % factor;
        write!(
            f,
            "{whole}.{fraction:0width$}",
            width = self.scale as usize
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn render(value: ExactMinorAmount) -> String {
        value.to_string()
    }

    #[test]
    fn zero_is_exact() {
        assert_eq!(render(ExactMinorAmount::new(0, 6)), "0.000000");
    }

    #[test]
    fn subunit_is_exact() {
        assert_eq!(render(ExactMinorAmount::new(1, 6)), "0.000001");
    }

    #[test]
    fn ordinary_value_is_exact() {
        assert_eq!(render(ExactMinorAmount::new(1_234_567, 6)), "1.234567");
    }

    #[test]
    fn negative_value_is_exact() {
        assert_eq!(render(ExactMinorAmount::new(-1, 6)), "-0.000001");
        assert_eq!(render(ExactMinorAmount::new(-1_234_567, 6)), "-1.234567");
    }

    #[test]
    fn i128_boundaries_are_supported() {
        assert_eq!(
            render(ExactMinorAmount::new(i128::MAX, 0)),
            i128::MAX.to_string()
        );
        assert_eq!(
            render(ExactMinorAmount::new(i128::MIN, 0)),
            i128::MIN.to_string()
        );
    }

    #[test]
    fn scale_zero_has_no_decimal_point() {
        assert_eq!(render(ExactMinorAmount::new(42, 0)), "42");
    }

    #[test]
    fn unsupported_scale_fails_formatting() {
        let mut output = String::new();
        let result = std::fmt::write(
            &mut output,
            format_args!("{}", ExactMinorAmount::new(1, ExactMinorAmount::MAX_SCALE + 1)),
        );
        assert!(result.is_err());
        assert!(output.is_empty());
    }
}
