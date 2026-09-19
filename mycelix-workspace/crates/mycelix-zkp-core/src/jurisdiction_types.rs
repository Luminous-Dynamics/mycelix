// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//!
//! Proof-independent jurisdiction geometry.
//!
//! These types describe published geographic bounding boxes only. They carry no
//! zero-knowledge, attestation, governance, residency, tax, or legal authority.
//! Proof systems may reference these values, but proof authority must remain in a
//! separately qualified theorem.

use serde::{Deserialize, Serialize};

/// Microdegree scale used by published jurisdiction geometry.
pub const MICRODEG_SCALE: i64 = 1_000_000;

/// Bias applied to signed microdegrees so the full longitude domain maps to u64.
pub const COORD_BIAS: i64 = 180 * MICRODEG_SCALE;

/// Convert a finite WGS-84-style degree value in [-180, 180] to biased
/// microdegrees.
///
/// This is a representation helper only; it proves no location claim.
pub fn biased_microdegrees(degrees: f64) -> Option<u64> {
    if !degrees.is_finite() || !(-180.0..=180.0).contains(&degrees) {
        return None;
    }
    let scaled = (degrees * MICRODEG_SCALE as f64).round();
    if scaled < i64::MIN as f64 || scaled > i64::MAX as f64 {
        return None;
    }
    let biased = scaled as i64 + COORD_BIAS;
    u64::try_from(biased).ok()
}

/// A published axis-aligned jurisdiction bounding box in biased microdegrees.
///
/// This is geographic metadata, not a proof statement. Real jurisdictions may
/// require unions of many boxes or richer polygonal geometry.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct JurisdictionBox {
    /// Stable identifier, e.g. `ZA-SARS-v1-mainland`.
    pub id: String,
    /// Biased latitude minimum.
    pub lat_min_biased: u64,
    /// Biased latitude maximum.
    pub lat_max_biased: u64,
    /// Biased longitude minimum.
    pub lng_min_biased: u64,
    /// Biased longitude maximum.
    pub lng_max_biased: u64,
}

impl JurisdictionBox {
    /// Build a box from finite decimal-degree bounds.
    pub fn from_degrees(
        id: impl Into<String>,
        lat_min: f64,
        lat_max: f64,
        lng_min: f64,
        lng_max: f64,
    ) -> Option<Self> {
        if !lat_min.is_finite()
            || !lat_max.is_finite()
            || !lng_min.is_finite()
            || !lng_max.is_finite()
            || !(-90.0..=90.0).contains(&lat_min)
            || !(-90.0..=90.0).contains(&lat_max)
            || !(-180.0..=180.0).contains(&lng_min)
            || !(-180.0..=180.0).contains(&lng_max)
            || lat_min > lat_max
            || lng_min > lng_max
        {
            return None;
        }

        Some(Self {
            id: id.into(),
            lat_min_biased: biased_microdegrees(lat_min)?,
            lat_max_biased: biased_microdegrees(lat_max)?,
            lng_min_biased: biased_microdegrees(lng_min)?,
            lng_max_biased: biased_microdegrees(lng_max)?,
        })
    }

    /// Whether a finite valid coordinate lies inside this coarse box.
    ///
    /// This is ordinary geometry only; it does not attest where a person or
    /// device actually is.
    pub fn contains_degrees(&self, lat: f64, lng: f64) -> bool {
        if !lat.is_finite()
            || !lng.is_finite()
            || !(-90.0..=90.0).contains(&lat)
            || !(-180.0..=180.0).contains(&lng)
        {
            return false;
        }

        let Some(lat_b) = biased_microdegrees(lat) else {
            return false;
        };
        let Some(lng_b) = biased_microdegrees(lng) else {
            return false;
        };

        lat_b >= self.lat_min_biased
            && lat_b <= self.lat_max_biased
            && lng_b >= self.lng_min_biased
            && lng_b <= self.lng_max_biased
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_non_finite_and_out_of_domain_coordinates() {
        assert!(biased_microdegrees(f64::NAN).is_none());
        assert!(biased_microdegrees(f64::INFINITY).is_none());
        assert!(biased_microdegrees(-181.0).is_none());
        assert!(biased_microdegrees(181.0).is_none());

        assert!(JurisdictionBox::from_degrees("x", f64::NAN, 1.0, 0.0, 1.0).is_none());
        assert!(JurisdictionBox::from_degrees("x", 0.0, 1.0, 0.0, f64::INFINITY).is_none());
    }

    #[test]
    fn builds_and_queries_south_africa_box() {
        let b = JurisdictionBox::from_degrees(
            "ZA-SARS-v1-mainland",
            -34.833333,
            -22.125,
            16.448056,
            32.891667,
        )
        .expect("valid box");

        assert!(b.contains_degrees(-26.1625, 27.8725));
        assert!(!b.contains_degrees(40.7128, -74.0060));
        assert!(!b.contains_degrees(f64::NAN, 27.8725));
    }

    #[test]
    fn bias_encoding_has_expected_origin_and_extremes() {
        assert_eq!(biased_microdegrees(-180.0), Some(0));
        assert_eq!(biased_microdegrees(0.0), Some(COORD_BIAS as u64));
        assert_eq!(biased_microdegrees(180.0), Some((2 * COORD_BIAS) as u64));
    }

    #[test]
    fn rejects_inverted_bounds() {
        assert!(JurisdictionBox::from_degrees("x", 10.0, 5.0, 0.0, 1.0).is_none());
        assert!(JurisdictionBox::from_degrees("x", 0.0, 1.0, 10.0, 5.0).is_none());
    }
}
