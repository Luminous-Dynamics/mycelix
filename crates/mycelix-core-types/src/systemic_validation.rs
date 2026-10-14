// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Structural validation for systemic evidence values.
//!
//! Kept separate from `systemic.rs` so compatibility-safe validation can evolve
//! without conflating source representation with canonical identity or hashing.

use crate::systemic::ObservationValue;

impl ObservationValue {
    /// Validate local structural invariants without changing source semantics.
    ///
    /// Ordinary numeric observations must be finite. Missing, withheld,
    /// censored, or otherwise exceptional source states must be represented
    /// explicitly by a later typed boundary rather than smuggled through NaN or
    /// infinity. This method never clamps or coerces the value.
    pub fn validate(&self) -> Result<(), &'static str> {
        match self {
            Self::Number { value, .. } if !value.is_finite() => {
                Err("ordinary numeric observation must be finite")
            }
            _ => Ok(()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finite_number_is_valid() {
        let value = ObservationValue::Number {
            value: 42.5,
            unit: None,
        };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn exceptional_floats_are_rejected_without_coercion() {
        for raw in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let value = ObservationValue::Number {
                value: raw,
                unit: None,
            };
            assert!(value.validate().is_err());
        }
    }

    #[test]
    fn non_numeric_values_are_not_reclassified() {
        assert!(ObservationValue::Text("unknown".into()).validate().is_ok());
        assert!(ObservationValue::Boolean(false).validate().is_ok());
    }
}
