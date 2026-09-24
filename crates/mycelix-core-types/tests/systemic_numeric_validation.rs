// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Never-merge RED qualifier for canonical-safe systemic numeric observations.
//!
//! Ordinary `ObservationValue::Number` values must be finite before they can
//! enter canonical identity, hashing, or downstream systemic calculations.

use mycelix_core_types::ObservationValue;

#[test]
fn finite_numeric_observation_is_structurally_valid() {
    let value = ObservationValue::Number {
        value: 42.5,
        unit: None,
    };

    assert!(value.validate().is_ok());
}

#[test]
fn nan_is_not_an_ordinary_numeric_observation() {
    let value = ObservationValue::Number {
        value: f64::NAN,
        unit: None,
    };

    assert!(value.validate().is_err());
}

#[test]
fn positive_infinity_is_not_an_ordinary_numeric_observation() {
    let value = ObservationValue::Number {
        value: f64::INFINITY,
        unit: None,
    };

    assert!(value.validate().is_err());
}

#[test]
fn negative_infinity_is_not_an_ordinary_numeric_observation() {
    let value = ObservationValue::Number {
        value: f64::NEG_INFINITY,
        unit: None,
    };

    assert!(value.validate().is_err());
}
