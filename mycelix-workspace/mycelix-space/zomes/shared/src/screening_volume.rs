// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Versioned, orbit-regime-specific conjunction screening volumes.
//!
//! This module evaluates broad-phase candidate-volume geometry only. It does
//! not classify an orbit regime, propagate trajectories, detect a TCA, compute
//! Pc, or establish that an excluded pair is safe. Regime selection and RIC
//! frame conversion must be supplied by a separately qualified caller.

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Explicit orbit-regime key for a screening-volume profile.
///
/// The caller must provide a qualified regime classification. This enum is
/// not a heuristic classifier; perigee/eccentricity/maneuverability boundaries
/// must be handled and tested by the upstream orbit-regime contract.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
pub enum OrbitRegime {
    Leo1,
    Leo2,
    Leo3,
    Leo4,
    Meo,
    Geo,
    GeoDisposal,
    Heo1,
    Heo2NonManeuverable,
    Heo2Maneuverable,
    Heo,
}

/// Relative position in the declared radial / in-track / cross-track frame.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct RicOffsetKm {
    pub radial_km: f64,
    pub in_track_km: f64,
    pub cross_track_km: f64,
}

/// Geometric shape of a broad-phase candidate volume.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub enum ScreeningVolumeShape {
    RicRectangular {
        radial_half_width_km: f64,
        in_track_half_width_km: f64,
        cross_track_half_width_km: f64,
    },
    Spherical {
        radius_km: f64,
    },
}

/// A profile-bound volume for one explicitly classified orbit regime.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct RegimeScreeningVolume {
    pub regime: OrbitRegime,
    pub shape: ScreeningVolumeShape,
}

/// Capture design target published with a screening-volume profile.
///
/// This is profile metadata, not a guarantee for arbitrary catalogues,
/// ephemerides, Pc methods, covariance histories or future orbital populations.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct ScreeningVolumeCaptureTarget {
    /// NASA profile states Pc strictly greater than 1e-4.
    pub pc_threshold_exclusive: f64,
    /// Published capture design target (0.99 for the March-2026 NASA profile).
    pub capture_rate_target: f64,
    /// Orbital-environment scope described by the publisher.
    pub environment_scope: String,
}

/// A named, versioned set of screening volumes.
///
/// The profile ID and source revision must be retained with every screening
/// receipt. Historic and current values are deliberately separate profiles.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct ScreeningVolumeProfile {
    pub profile_id: String,
    pub source_url: String,
    pub source_revision: String,
    pub capture_target: Option<ScreeningVolumeCaptureTarget>,
    pub volumes: BTreeMap<OrbitRegime, RegimeScreeningVolume>,
}

/// Broad-phase disposition for one relative position.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum ScreeningVolumeDisposition {
    Candidate,
    OutsideVolume,
}

/// Invalid profile or geometric input.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum ScreeningVolumeError {
    EmptyProfileId,
    EmptySourceRevision,
    MissingRegime { regime: OrbitRegime },
    NonFiniteRicOffset,
    InvalidRadialHalfWidth,
    InvalidInTrackHalfWidth,
    InvalidCrossTrackHalfWidth,
    InvalidSphereRadius,
    InvalidCaptureThreshold,
    InvalidCaptureRateTarget,
    EmptyCaptureEnvironmentScope,
    RegimeKeyMismatch { map_key: OrbitRegime, declared_regime: OrbitRegime },
}

/// Build NASA CARA's public regime-volume profile implemented since March 2026.
///
/// NASA describes these volumes as designed to capture 99% of conjunctions
/// with Pc > 1e-4 in the current orbital environment. They are a reference
/// fixture, not a universal guarantee and not a Mycelix qualification result.
///
/// Source: https://www.nasa.gov/cara/unique-conjunction-events/
pub fn nasa_cara_march_2026_profile() -> ScreeningVolumeProfile {
    let mut volumes = BTreeMap::new();
    let definitions = [
        (
            OrbitRegime::Leo1,
            ScreeningVolumeShape::RicRectangular {
                radial_half_width_km: 1.0,
                in_track_half_width_km: 50.0,
                cross_track_half_width_km: 55.0,
            },
        ),
        (
            OrbitRegime::Leo2,
            ScreeningVolumeShape::RicRectangular {
                radial_half_width_km: 1.0,
                in_track_half_width_km: 30.0,
                cross_track_half_width_km: 30.0,
            },
        ),
        (
            OrbitRegime::Leo3,
            ScreeningVolumeShape::RicRectangular {
                radial_half_width_km: 1.0,
                in_track_half_width_km: 30.0,
                cross_track_half_width_km: 25.0,
            },
        ),
        (
            OrbitRegime::Leo4,
            ScreeningVolumeShape::RicRectangular {
                radial_half_width_km: 0.5,
                in_track_half_width_km: 5.0,
                cross_track_half_width_km: 5.0,
            },
        ),
        (
            OrbitRegime::Meo,
            ScreeningVolumeShape::Spherical { radius_km: 15.0 },
        ),
        (
            OrbitRegime::GeoDisposal,
            ScreeningVolumeShape::RicRectangular {
                radial_half_width_km: 5.0,
                in_track_half_width_km: 25.0,
                cross_track_half_width_km: 15.0,
            },
        ),
        (
            OrbitRegime::Heo,
            ScreeningVolumeShape::RicRectangular {
                radial_half_width_km: 30.0,
                in_track_half_width_km: 85.0,
                cross_track_half_width_km: 85.0,
            },
        ),
    ];

    for (regime, shape) in definitions {
        volumes.insert(regime, RegimeScreeningVolume { regime, shape });
    }

    ScreeningVolumeProfile {
        profile_id: "nasa-cara-screening-volume-2026-03".to_string(),
        source_url: "https://www.nasa.gov/cara/unique-conjunction-events/".to_string(),
        source_revision: "NASA CARA Unique Conjunction Events page; accessed 2026-11-20"
            .to_string(),
        capture_target: Some(ScreeningVolumeCaptureTarget {
            pc_threshold_exclusive: 1.0e-4,
            capture_rate_target: 0.99,
            environment_scope: "Current orbital environment described by NASA CARA"
                .to_string(),
        }),
        volumes,
    }
}

/// Build NASA CARA's public profile used through February 2026.
///
/// Historic GEO and HEO profiles differ from the March-2026 profile. Keeping
/// these values as a separate fixture prevents profile drift from being
/// hidden in one mutable table.
pub fn nasa_cara_pre_march_2026_profile() -> ScreeningVolumeProfile {
    let mut volumes = BTreeMap::new();
    let definitions = [
        (
            OrbitRegime::Leo1,
            ScreeningVolumeShape::RicRectangular {
                radial_half_width_km: 2.0,
                in_track_half_width_km: 44.0,
                cross_track_half_width_km: 51.0,
            },
        ),
        (
            OrbitRegime::Leo2,
            ScreeningVolumeShape::RicRectangular {
                radial_half_width_km: 0.5,
                in_track_half_width_km: 17.0,
                cross_track_half_width_km: 20.0,
            },
        ),
        (
            OrbitRegime::Leo3,
            ScreeningVolumeShape::RicRectangular {
                radial_half_width_km: 0.5,
                in_track_half_width_km: 12.0,
                cross_track_half_width_km: 10.0,
            },
        ),
        (
            OrbitRegime::Leo4,
            ScreeningVolumeShape::RicRectangular {
                radial_half_width_km: 0.5,
                in_track_half_width_km: 25.0,
                cross_track_half_width_km: 25.0,
            },
        ),
        (
            OrbitRegime::Meo,
            ScreeningVolumeShape::Spherical { radius_km: 15.0 },
        ),
        (
            OrbitRegime::Geo,
            ScreeningVolumeShape::Spherical { radius_km: 20.0 },
        ),
        (
            OrbitRegime::Heo1,
            ScreeningVolumeShape::RicRectangular {
                radial_half_width_km: 40.0,
                in_track_half_width_km: 77.0,
                cross_track_half_width_km: 107.0,
            },
        ),
        (
            OrbitRegime::Heo2NonManeuverable,
            ScreeningVolumeShape::RicRectangular {
                radial_half_width_km: 40.0,
                in_track_half_width_km: 77.0,
                cross_track_half_width_km: 107.0,
            },
        ),
        (
            OrbitRegime::Heo2Maneuverable,
            ScreeningVolumeShape::RicRectangular {
                radial_half_width_km: 5.0,
                in_track_half_width_km: 45.0,
                cross_track_half_width_km: 52.0,
            },
        ),
    ];

    for (regime, shape) in definitions {
        volumes.insert(regime, RegimeScreeningVolume { regime, shape });
    }

    ScreeningVolumeProfile {
        profile_id: "nasa-cara-screening-volume-through-2026-02".to_string(),
        source_url: "https://www.nasa.gov/cara/unique-conjunction-events/".to_string(),
        source_revision: "NASA CARA Unique Conjunction Events page; accessed 2026-11-20"
            .to_string(),
        capture_target: None,
        volumes,
    }
}

impl ScreeningVolumeProfile {
    /// Validate profile metadata and every declared geometry.
    pub fn validate(&self) -> Result<(), ScreeningVolumeError> {
        if self.profile_id.trim().is_empty() {
            return Err(ScreeningVolumeError::EmptyProfileId);
        }
        if self.source_url.trim().is_empty() {
            return Err(ScreeningVolumeError::EmptySourceRevision);
        }
        if self.source_revision.trim().is_empty() {
            return Err(ScreeningVolumeError::EmptySourceRevision);
        }
        if let Some(target) = &self.capture_target {
            if target.environment_scope.trim().is_empty() {
                return Err(ScreeningVolumeError::EmptyCaptureEnvironmentScope);
            }
            if !target.pc_threshold_exclusive.is_finite()
                || target.pc_threshold_exclusive <= 0.0
                || target.pc_threshold_exclusive > 1.0
            {
                return Err(ScreeningVolumeError::InvalidCaptureThreshold);
            }
            if !target.capture_rate_target.is_finite()
                || !(0.0..=1.0).contains(&target.capture_rate_target)
            {
                return Err(ScreeningVolumeError::InvalidCaptureRateTarget);
            }
        }

        for (map_key, volume) in &self.volumes {
            if *map_key != volume.regime {
                return Err(ScreeningVolumeError::RegimeKeyMismatch {
                    map_key: *map_key,
                    declared_regime: volume.regime,
                });
            }
            match volume.shape {
                ScreeningVolumeShape::RicRectangular {
                    radial_half_width_km,
                    in_track_half_width_km,
                    cross_track_half_width_km,
                } => {
                    if !radial_half_width_km.is_finite() || radial_half_width_km <= 0.0 {
                        return Err(ScreeningVolumeError::InvalidRadialHalfWidth);
                    }
                    if !in_track_half_width_km.is_finite() || in_track_half_width_km <= 0.0 {
                        return Err(ScreeningVolumeError::InvalidInTrackHalfWidth);
                    }
                    if !cross_track_half_width_km.is_finite() || cross_track_half_width_km <= 0.0
                    {
                        return Err(ScreeningVolumeError::InvalidCrossTrackHalfWidth);
                    }
                }
                ScreeningVolumeShape::Spherical { radius_km } => {
                    if !radius_km.is_finite() || radius_km <= 0.0 {
                        return Err(ScreeningVolumeError::InvalidSphereRadius);
                    }
                }
            }
        }

        Ok(())
    }

    /// Evaluate broad-phase candidate inclusion in the selected regime volume.
    ///
    /// Rectangular profiles use independent inclusive R/I/C half-width bounds.
    /// Spherical profiles use the Euclidean RIC offset norm. This function
    /// assumes the caller has already transformed the relative state into the
    /// exact RIC frame/realization declared by the caller's evidence receipt.
    pub fn classify(
        &self,
        regime: OrbitRegime,
        offset: RicOffsetKm,
    ) -> Result<ScreeningVolumeDisposition, ScreeningVolumeError> {
        self.validate()?;
        if !offset.radial_km.is_finite()
            || !offset.in_track_km.is_finite()
            || !offset.cross_track_km.is_finite()
        {
            return Err(ScreeningVolumeError::NonFiniteRicOffset);
        }

        let volume = self
            .volumes
            .get(&regime)
            .ok_or(ScreeningVolumeError::MissingRegime { regime })?;

        let inside = match volume.shape {
            ScreeningVolumeShape::RicRectangular {
                radial_half_width_km,
                in_track_half_width_km,
                cross_track_half_width_km,
            } => {
                offset.radial_km.abs() <= radial_half_width_km
                    && offset.in_track_km.abs() <= in_track_half_width_km
                    && offset.cross_track_km.abs() <= cross_track_half_width_km
            }
            ScreeningVolumeShape::Spherical { radius_km } => {
                offset
                    .radial_km
                    .hypot(offset.in_track_km)
                    .hypot(offset.cross_track_km)
                    <= radius_km
            }
        };

        Ok(if inside {
            ScreeningVolumeDisposition::Candidate
        } else {
            ScreeningVolumeDisposition::OutsideVolume
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn march_2026_profile_preserves_published_regime_dimensions() {
        let profile = nasa_cara_march_2026_profile();
        assert_eq!(profile.profile_id, "nasa-cara-screening-volume-2026-03");
        let leo2 = profile.volumes.get(&OrbitRegime::Leo2).unwrap();
        assert_eq!(
            leo2.shape,
            ScreeningVolumeShape::RicRectangular {
                radial_half_width_km: 1.0,
                in_track_half_width_km: 30.0,
                cross_track_half_width_km: 30.0,
            }
        );
        let meo = profile.volumes.get(&OrbitRegime::Meo).unwrap();
        assert_eq!(
            meo.shape,
            ScreeningVolumeShape::Spherical { radius_km: 15.0 }
        );
        let target = profile.capture_target.unwrap();
        assert_eq!(target.pc_threshold_exclusive, 1.0e-4);
        assert_eq!(target.capture_rate_target, 0.99);
    }

    #[test]
    fn historic_and_current_profiles_are_separate_versioned_fixtures() {
        let historic = nasa_cara_pre_march_2026_profile();
        let current = nasa_cara_march_2026_profile();
        assert_ne!(historic.profile_id, current.profile_id);
        assert_eq!(historic.capture_target, None);
        assert!(historic.volumes.contains_key(&OrbitRegime::Heo1));
        assert!(current.volumes.contains_key(&OrbitRegime::Heo));
        assert!(!current.volumes.contains_key(&OrbitRegime::Heo1));
    }

    #[test]
    fn rectangular_ric_volume_includes_exact_faces_and_rejects_outside() {
        let profile = nasa_cara_march_2026_profile();
        assert_eq!(
            profile.classify(
                OrbitRegime::Leo2,
                RicOffsetKm {
                    radial_km: 1.0,
                    in_track_km: -30.0,
                    cross_track_km: 30.0,
                },
            ),
            Ok(ScreeningVolumeDisposition::Candidate)
        );
        assert_eq!(
            profile.classify(
                OrbitRegime::Leo2,
                RicOffsetKm {
                    radial_km: 1.000_001,
                    in_track_km: 0.0,
                    cross_track_km: 0.0,
                },
            ),
            Ok(ScreeningVolumeDisposition::OutsideVolume)
        );
        assert_eq!(
            profile.classify(
                OrbitRegime::Leo2,
                RicOffsetKm {
                    radial_km: 0.0,
                    in_track_km: 30.000_001,
                    cross_track_km: 0.0,
                },
            ),
            Ok(ScreeningVolumeDisposition::OutsideVolume)
        );
        assert_eq!(
            profile.classify(
                OrbitRegime::Leo2,
                RicOffsetKm {
                    radial_km: 0.0,
                    in_track_km: 0.0,
                    cross_track_km: 30.000_001,
                },
            ),
            Ok(ScreeningVolumeDisposition::OutsideVolume)
        );
    }

    #[test]
    fn spherical_volume_uses_euclidean_ric_distance_not_axiswise_bounds() {
        let profile = nasa_cara_march_2026_profile();
        assert_eq!(
            profile.classify(
                OrbitRegime::Meo,
                RicOffsetKm {
                    radial_km: 9.0,
                    in_track_km: 12.0,
                    cross_track_km: 0.0,
                },
            ),
            Ok(ScreeningVolumeDisposition::Candidate)
        );
        assert_eq!(
            profile.classify(
                OrbitRegime::Meo,
                RicOffsetKm {
                    radial_km: 9.0,
                    in_track_km: 12.0,
                    cross_track_km: 0.1,
                },
            ),
            Ok(ScreeningVolumeDisposition::OutsideVolume)
        );
    }

    #[test]
    fn unsupported_regime_and_non_finite_coordinates_fail_closed() {
        let profile = nasa_cara_march_2026_profile();
        assert_eq!(
            profile.classify(
                OrbitRegime::Geo,
                RicOffsetKm {
                    radial_km: 0.0,
                    in_track_km: 0.0,
                    cross_track_km: 0.0,
                },
            ),
            Err(ScreeningVolumeError::MissingRegime {
                regime: OrbitRegime::Geo
            })
        );
        assert_eq!(
            profile.classify(
                OrbitRegime::Leo1,
                RicOffsetKm {
                    radial_km: f64::NAN,
                    in_track_km: 0.0,
                    cross_track_km: 0.0,
                },
            ),
            Err(ScreeningVolumeError::NonFiniteRicOffset)
        );
    }

    #[test]
    fn profile_regime_map_keys_must_match_declared_regime_values() {
        let mut profile = nasa_cara_march_2026_profile();
        let wrong = profile.volumes.get(&OrbitRegime::Leo1).unwrap().clone();
        profile.volumes.insert(OrbitRegime::Leo2, wrong);
        assert_eq!(
            profile.validate(),
            Err(ScreeningVolumeError::RegimeKeyMismatch {
                map_key: OrbitRegime::Leo2,
                declared_regime: OrbitRegime::Leo1,
            })
        );
    }

    #[test]
    fn invalid_profile_geometry_and_capture_target_are_rejected() {
        let mut profile = nasa_cara_march_2026_profile();
        profile.capture_target.as_mut().unwrap().capture_rate_target = 1.1;
        assert_eq!(
            profile.validate(),
            Err(ScreeningVolumeError::InvalidCaptureRateTarget)
        );

        let mut profile = nasa_cara_march_2026_profile();
        profile.volumes.insert(
            OrbitRegime::Leo1,
            RegimeScreeningVolume {
                regime: OrbitRegime::Leo1,
                shape: ScreeningVolumeShape::RicRectangular {
                    radial_half_width_km: 0.0,
                    in_track_half_width_km: 50.0,
                    cross_track_half_width_km: 55.0,
                },
            },
        );
        assert_eq!(
            profile.validate(),
            Err(ScreeningVolumeError::InvalidRadialHalfWidth)
        );
    }
}
