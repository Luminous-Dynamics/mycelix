// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Ranging models: convert raw sensor data to distance + uncertainty.
//!
//! Each model maps a physical measurement (RSSI, time-of-flight, etc.)
//! to a range estimate with an explicit uncertainty model. These are
//! first-pass envelopes for fusion, not guarantees of field accuracy.
//!
//! Authority-facing callers should use the `try_*` conversion functions. The
//! legacy infallible wrappers remain for compatibility, but invalid physical
//! input now produces a non-finite sentinel estimate so downstream finite-value
//! validation fails closed instead of receiving a plausible clamped distance.

use serde::{Deserialize, Serialize};

/// Speed of light in meters per second.
const C_M_S: f64 = 299_792_458.0;

/// A range estimate with uncertainty.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RangeEstimate {
    pub distance_m: f64,
    pub sigma_m: f64,
    pub method: RangingMethod,
}

impl RangeEstimate {
    /// Whether this estimate is numerically suitable for further physical
    /// qualification. This does not establish calibration or source trust.
    pub fn is_finite_and_non_negative(&self) -> bool {
        self.distance_m.is_finite()
            && self.distance_m >= 0.0
            && self.sigma_m.is_finite()
            && self.sigma_m > 0.0
    }
}

/// Ranging technology used for the measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RangingMethod {
    /// Ultra-Wideband time-of-flight (σ ≈ 0.05-0.30 m)
    UltraWideband,
    /// LoRa time-of-arrival (σ ≈ 50-200 m)
    LoRaToA,
    /// LoRa time-difference-of-arrival (σ ≈ 30-100 m)
    LoRaTDoA,
    /// RSSI log-distance path loss (σ ≈ 3-8 m)
    RssiPathLoss,
    /// WiFi Fine Timing Measurement / RTT (σ ≈ 1-3 m, IEEE 802.11mc)
    WifiRtt,
    /// Manual survey with instruments (σ = instrument accuracy)
    ManualSurvey,
    /// Meshtastic hop-count estimate (σ ≈ 500-2000 m)
    MeshtasticHops,
    /// BLE Channel Sounding (Bluetooth 6.0, σ often ≈ 0.2-1.0 m on commodity devices)
    BleChannelSounding,
    /// Acoustic time-of-flight through water or air in low-visibility environments.
    AcousticTimeOfFlight,
    /// Optical time-of-flight or laser ranging.
    OpticalTimeOfFlight,
    /// Deep-space RF round-trip timing.
    DeepSpaceRadioRtt,
}

/// Rejection reasons for raw physical/ranging inputs.
#[derive(Debug, Clone, PartialEq)]
pub enum MeasurementConversionError {
    NonFinite { field: &'static str },
    Negative { field: &'static str, value: f64 },
    NonPositive { field: &'static str, value: f64 },
    NonFiniteResult { method: RangingMethod },
}

impl std::fmt::Display for MeasurementConversionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NonFinite { field } => write!(f, "{field} must be finite"),
            Self::Negative { field, value } => {
                write!(f, "{field} must be non-negative, got {value}")
            }
            Self::NonPositive { field, value } => {
                write!(f, "{field} must be positive, got {value}")
            }
            Self::NonFiniteResult { method } => {
                write!(f, "{method:?} conversion produced a non-finite result")
            }
        }
    }
}

impl std::error::Error for MeasurementConversionError {}

fn require_finite(field: &'static str, value: f64) -> Result<f64, MeasurementConversionError> {
    if !value.is_finite() {
        return Err(MeasurementConversionError::NonFinite { field });
    }
    Ok(value)
}

fn require_non_negative(
    field: &'static str,
    value: f64,
) -> Result<f64, MeasurementConversionError> {
    require_finite(field, value)?;
    if value < 0.0 {
        return Err(MeasurementConversionError::Negative { field, value });
    }
    Ok(value)
}

fn require_positive(
    field: &'static str,
    value: f64,
) -> Result<f64, MeasurementConversionError> {
    require_finite(field, value)?;
    if value <= 0.0 {
        return Err(MeasurementConversionError::NonPositive { field, value });
    }
    Ok(value)
}

fn qualified_estimate(
    distance_m: f64,
    sigma_m: f64,
    method: RangingMethod,
) -> Result<RangeEstimate, MeasurementConversionError> {
    let estimate = RangeEstimate {
        distance_m,
        sigma_m,
        method,
    };
    if !estimate.is_finite_and_non_negative() {
        return Err(MeasurementConversionError::NonFiniteResult { method });
    }
    Ok(estimate)
}

fn invalid_legacy_estimate(method: RangingMethod) -> RangeEstimate {
    RangeEstimate {
        distance_m: f64::NAN,
        sigma_m: f64::NAN,
        method,
    }
}

// ============================================================================
// RSSI PATH-LOSS MODEL
// ============================================================================

/// Checked RSSI log-distance conversion.
pub fn try_rssi_to_range(
    rssi_dbm: f64,
    ref_rssi_dbm: f64,
    path_loss_exponent: f64,
) -> Result<RangeEstimate, MeasurementConversionError> {
    require_finite("rssi_dbm", rssi_dbm)?;
    require_finite("ref_rssi_dbm", ref_rssi_dbm)?;
    require_positive("path_loss_exponent", path_loss_exponent)?;

    let exponent = (ref_rssi_dbm - rssi_dbm) / (10.0 * path_loss_exponent);
    let distance = 10.0_f64.powf(exponent);
    let sigma_fraction = 0.1 + 0.05 * (path_loss_exponent - 2.0);
    let sigma = (distance * sigma_fraction).max(0.5);
    qualified_estimate(
        distance.max(0.1),
        sigma,
        RangingMethod::RssiPathLoss,
    )
}

/// Legacy compatibility wrapper. Prefer [`try_rssi_to_range`].
pub fn rssi_to_range(rssi_dbm: f64, ref_rssi_dbm: f64, path_loss_exponent: f64) -> RangeEstimate {
    try_rssi_to_range(rssi_dbm, ref_rssi_dbm, path_loss_exponent)
        .unwrap_or_else(|_| invalid_legacy_estimate(RangingMethod::RssiPathLoss))
}

// ============================================================================
// LoRa TIME-OF-ARRIVAL
// ============================================================================

/// Checked LoRa one-way time-of-arrival conversion.
pub fn try_lora_toa_to_range(
    toa_ns: f64,
    bandwidth_hz: f64,
) -> Result<RangeEstimate, MeasurementConversionError> {
    require_non_negative("toa_ns", toa_ns)?;
    require_positive("bandwidth_hz", bandwidth_hz)?;

    let distance = C_M_S * toa_ns * 1e-9;
    let time_resolution_s = 1.0 / bandwidth_hz;
    let sigma = (C_M_S * time_resolution_s * 0.1).max(50.0);
    qualified_estimate(distance, sigma, RangingMethod::LoRaToA)
}

/// Legacy compatibility wrapper. Prefer [`try_lora_toa_to_range`].
pub fn lora_toa_to_range(toa_ns: f64, bandwidth_hz: f64) -> RangeEstimate {
    try_lora_toa_to_range(toa_ns, bandwidth_hz)
        .unwrap_or_else(|_| invalid_legacy_estimate(RangingMethod::LoRaToA))
}

// ============================================================================
// UWB TIME-OF-FLIGHT
// ============================================================================

/// Checked UWB one-way time-of-flight conversion.
pub fn try_uwb_tof_to_range(tof_ns: f64) -> Result<RangeEstimate, MeasurementConversionError> {
    require_non_negative("tof_ns", tof_ns)?;
    let distance = C_M_S * tof_ns * 1e-9;
    let sigma = (0.10 + distance * 0.002).max(0.05);
    qualified_estimate(distance, sigma, RangingMethod::UltraWideband)
}

/// Legacy compatibility wrapper. Prefer [`try_uwb_tof_to_range`].
pub fn uwb_tof_to_range(tof_ns: f64) -> RangeEstimate {
    try_uwb_tof_to_range(tof_ns)
        .unwrap_or_else(|_| invalid_legacy_estimate(RangingMethod::UltraWideband))
}

// ============================================================================
// WiFi ROUND-TRIP TIME (IEEE 802.11mc)
// ============================================================================

/// Checked Wi-Fi RTT conversion.
pub fn try_wifi_rtt_to_range(rtt_ns: f64) -> Result<RangeEstimate, MeasurementConversionError> {
    require_non_negative("rtt_ns", rtt_ns)?;
    let distance = C_M_S * rtt_ns * 1e-9 / 2.0;
    let sigma = (1.0 + distance * 0.01).max(0.5);
    qualified_estimate(distance, sigma, RangingMethod::WifiRtt)
}

/// Legacy compatibility wrapper. Prefer [`try_wifi_rtt_to_range`].
pub fn wifi_rtt_to_range(rtt_ns: f64) -> RangeEstimate {
    try_wifi_rtt_to_range(rtt_ns)
        .unwrap_or_else(|_| invalid_legacy_estimate(RangingMethod::WifiRtt))
}

// ============================================================================
// MANUAL SURVEY
// ============================================================================

/// Checked manual-survey conversion.
pub fn try_manual_survey(
    distance_m: f64,
    instrument_accuracy_m: f64,
) -> Result<RangeEstimate, MeasurementConversionError> {
    require_non_negative("distance_m", distance_m)?;
    require_positive("instrument_accuracy_m", instrument_accuracy_m)?;
    qualified_estimate(
        distance_m,
        instrument_accuracy_m.max(0.001),
        RangingMethod::ManualSurvey,
    )
}

/// Legacy compatibility wrapper. Prefer [`try_manual_survey`].
pub fn manual_survey(distance_m: f64, instrument_accuracy_m: f64) -> RangeEstimate {
    try_manual_survey(distance_m, instrument_accuracy_m)
        .unwrap_or_else(|_| invalid_legacy_estimate(RangingMethod::ManualSurvey))
}

// ============================================================================
// BLE CHANNEL SOUNDING (Bluetooth 6.0)
// ============================================================================

/// Checked BLE Channel Sounding RTT conversion.
pub fn try_ble_channel_sounding_to_range(
    round_trip_time_ns: f64,
) -> Result<RangeEstimate, MeasurementConversionError> {
    require_non_negative("round_trip_time_ns", round_trip_time_ns)?;
    let distance = C_M_S * round_trip_time_ns * 1e-9 / 2.0;
    let sigma = (0.25 + distance * 0.025).max(0.20);
    qualified_estimate(distance, sigma, RangingMethod::BleChannelSounding)
}

/// Legacy compatibility wrapper. Prefer [`try_ble_channel_sounding_to_range`].
pub fn ble_channel_sounding_to_range(round_trip_time_ns: f64) -> RangeEstimate {
    try_ble_channel_sounding_to_range(round_trip_time_ns)
        .unwrap_or_else(|_| invalid_legacy_estimate(RangingMethod::BleChannelSounding))
}

// ============================================================================
// ACOUSTIC TIME-OF-FLIGHT
// ============================================================================

/// Checked acoustic round-trip time-of-flight conversion.
pub fn try_acoustic_tof_to_range(
    round_trip_time_s: f64,
    sound_speed_m_s: f64,
    timing_sigma_s: f64,
) -> Result<RangeEstimate, MeasurementConversionError> {
    require_non_negative("round_trip_time_s", round_trip_time_s)?;
    require_positive("sound_speed_m_s", sound_speed_m_s)?;
    require_positive("timing_sigma_s", timing_sigma_s)?;

    let distance = sound_speed_m_s * round_trip_time_s / 2.0;
    let sigma = (sound_speed_m_s * timing_sigma_s / 2.0).max(0.05);
    qualified_estimate(distance, sigma, RangingMethod::AcousticTimeOfFlight)
}

/// Legacy compatibility wrapper. Prefer [`try_acoustic_tof_to_range`].
pub fn acoustic_tof_to_range(
    round_trip_time_s: f64,
    sound_speed_m_s: f64,
    timing_sigma_s: f64,
) -> RangeEstimate {
    try_acoustic_tof_to_range(round_trip_time_s, sound_speed_m_s, timing_sigma_s)
        .unwrap_or_else(|_| invalid_legacy_estimate(RangingMethod::AcousticTimeOfFlight))
}

// ============================================================================
// OPTICAL / LASER TIME-OF-FLIGHT
// ============================================================================

/// Checked optical/laser RTT conversion.
pub fn try_optical_tof_to_range(
    round_trip_time_ns: f64,
    clock_sigma_ns: f64,
) -> Result<RangeEstimate, MeasurementConversionError> {
    require_non_negative("round_trip_time_ns", round_trip_time_ns)?;
    require_positive("clock_sigma_ns", clock_sigma_ns)?;

    let distance = C_M_S * round_trip_time_ns * 1e-9 / 2.0;
    let sigma = (C_M_S * clock_sigma_ns * 1e-9 / 2.0).max(0.001);
    qualified_estimate(distance, sigma, RangingMethod::OpticalTimeOfFlight)
}

/// Legacy compatibility wrapper. Prefer [`try_optical_tof_to_range`].
pub fn optical_tof_to_range(round_trip_time_ns: f64, clock_sigma_ns: f64) -> RangeEstimate {
    try_optical_tof_to_range(round_trip_time_ns, clock_sigma_ns)
        .unwrap_or_else(|_| invalid_legacy_estimate(RangingMethod::OpticalTimeOfFlight))
}

// ============================================================================
// DEEP-SPACE RF RTT
// ============================================================================

/// Checked deep-space RF RTT conversion.
pub fn try_deep_space_radio_rtt_to_range(
    round_trip_time_ns: f64,
    turnaround_jitter_ns: f64,
) -> Result<RangeEstimate, MeasurementConversionError> {
    require_non_negative("round_trip_time_ns", round_trip_time_ns)?;
    require_positive("turnaround_jitter_ns", turnaround_jitter_ns)?;

    let distance = C_M_S * round_trip_time_ns * 1e-9 / 2.0;
    let sigma = (C_M_S * turnaround_jitter_ns * 1e-9 / 2.0).max(0.5);
    qualified_estimate(distance, sigma, RangingMethod::DeepSpaceRadioRtt)
}

/// Legacy compatibility wrapper. Prefer [`try_deep_space_radio_rtt_to_range`].
pub fn deep_space_radio_rtt_to_range(
    round_trip_time_ns: f64,
    turnaround_jitter_ns: f64,
) -> RangeEstimate {
    try_deep_space_radio_rtt_to_range(round_trip_time_ns, turnaround_jitter_ns)
        .unwrap_or_else(|_| invalid_legacy_estimate(RangingMethod::DeepSpaceRadioRtt))
}

// ============================================================================
// MESHTASTIC HOP COUNT
// ============================================================================

/// Checked coarse hop-count conversion.
pub fn try_meshtastic_hops_to_range(
    hop_count: u32,
    avg_hop_distance_m: f64,
) -> Result<RangeEstimate, MeasurementConversionError> {
    require_positive("avg_hop_distance_m", avg_hop_distance_m)?;
    let distance = hop_count as f64 * avg_hop_distance_m;
    let sigma = (distance * 0.5).max(500.0);
    qualified_estimate(distance, sigma, RangingMethod::MeshtasticHops)
}

/// Legacy compatibility wrapper. Prefer [`try_meshtastic_hops_to_range`].
pub fn meshtastic_hops_to_range(hop_count: u32, avg_hop_distance_m: f64) -> RangeEstimate {
    try_meshtastic_hops_to_range(hop_count, avg_hop_distance_m)
        .unwrap_or_else(|_| invalid_legacy_estimate(RangingMethod::MeshtasticHops))
}

// ============================================================================
// TESTS
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rssi_at_reference_distance() {
        let est = try_rssi_to_range(-50.0, -50.0, 2.0).unwrap();
        assert!((est.distance_m - 1.0).abs() < 0.1);
    }

    #[test]
    fn rssi_farther_is_weaker() {
        let near = try_rssi_to_range(-55.0, -50.0, 2.0).unwrap();
        let far = try_rssi_to_range(-70.0, -50.0, 2.0).unwrap();
        assert!(far.distance_m > near.distance_m);
        assert!(far.sigma_m > near.sigma_m);
    }

    #[test]
    fn uwb_centimeter_accuracy() {
        let est = try_uwb_tof_to_range(33.36).unwrap();
        assert!((est.distance_m - 10.0).abs() < 0.01);
        assert!(est.sigma_m < 0.5);
    }

    #[test]
    fn lora_reasonable_accuracy() {
        let est = try_lora_toa_to_range(3336.0, 125_000.0).unwrap();
        assert!((est.distance_m - 1000.0).abs() < 1.0);
        assert!(est.sigma_m >= 50.0);
    }

    #[test]
    fn wifi_rtt_meter_class() {
        let est = try_wifi_rtt_to_range(66.7).unwrap();
        assert!((est.distance_m - 10.0).abs() < 0.1);
        assert!(est.sigma_m < 5.0);
    }

    #[test]
    fn manual_survey_preserves_accuracy() {
        let est = try_manual_survey(100.0, 0.01).unwrap();
        assert_eq!(est.distance_m, 100.0);
        assert_eq!(est.sigma_m, 0.01);
    }

    #[test]
    fn meshtastic_scales_with_hops() {
        let one = try_meshtastic_hops_to_range(1, 2000.0).unwrap();
        let three = try_meshtastic_hops_to_range(3, 2000.0).unwrap();
        assert!((one.distance_m - 2000.0).abs() < 1.0);
        assert!((three.distance_m - 6000.0).abs() < 1.0);
        assert!(three.sigma_m > one.sigma_m);
    }

    #[test]
    fn ble_channel_sounding_uses_conservative_sigma() {
        let est = try_ble_channel_sounding_to_range(66.7).unwrap();
        assert!((est.distance_m - 10.0).abs() < 0.1);
        assert!(est.sigma_m >= 0.2);
        assert_eq!(est.method, RangingMethod::BleChannelSounding);
    }

    #[test]
    fn optical_tof_supports_sub_meter_sigma() {
        let est = try_optical_tof_to_range(66.7, 0.02).unwrap();
        assert!((est.distance_m - 10.0).abs() < 0.1);
        assert!(est.sigma_m < 0.01);
    }

    #[test]
    fn acoustic_tof_supports_underwater_ranges() {
        let est = try_acoustic_tof_to_range(0.0267, 1500.0, 0.0001).unwrap();
        assert!((est.distance_m - 20.0).abs() < 0.2);
        assert!(est.sigma_m >= 0.05);
    }

    #[test]
    fn deep_space_radio_rtt_has_coarser_sigma() {
        let est = try_deep_space_radio_rtt_to_range(66.7, 10.0).unwrap();
        assert!((est.distance_m - 10.0).abs() < 0.1);
        assert!(est.sigma_m >= 0.5);
    }

    #[test]
    fn checked_conversions_reject_non_finite_inputs() {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(try_rssi_to_range(bad, -50.0, 2.0).is_err());
            assert!(try_rssi_to_range(-50.0, bad, 2.0).is_err());
            assert!(try_rssi_to_range(-50.0, -50.0, bad).is_err());
            assert!(try_lora_toa_to_range(bad, 125_000.0).is_err());
            assert!(try_lora_toa_to_range(1.0, bad).is_err());
            assert!(try_uwb_tof_to_range(bad).is_err());
            assert!(try_wifi_rtt_to_range(bad).is_err());
            assert!(try_manual_survey(bad, 1.0).is_err());
            assert!(try_manual_survey(1.0, bad).is_err());
            assert!(try_ble_channel_sounding_to_range(bad).is_err());
            assert!(try_acoustic_tof_to_range(bad, 1500.0, 0.001).is_err());
            assert!(try_acoustic_tof_to_range(1.0, bad, 0.001).is_err());
            assert!(try_acoustic_tof_to_range(1.0, 1500.0, bad).is_err());
            assert!(try_optical_tof_to_range(bad, 1.0).is_err());
            assert!(try_optical_tof_to_range(1.0, bad).is_err());
            assert!(try_deep_space_radio_rtt_to_range(bad, 1.0).is_err());
            assert!(try_deep_space_radio_rtt_to_range(1.0, bad).is_err());
            assert!(try_meshtastic_hops_to_range(1, bad).is_err());
        }
    }

    #[test]
    fn checked_conversions_reject_negative_timing_or_distance() {
        assert!(try_lora_toa_to_range(-1.0, 125_000.0).is_err());
        assert!(try_uwb_tof_to_range(-1.0).is_err());
        assert!(try_wifi_rtt_to_range(-1.0).is_err());
        assert!(try_manual_survey(-1.0, 1.0).is_err());
        assert!(try_ble_channel_sounding_to_range(-1.0).is_err());
        assert!(try_acoustic_tof_to_range(-1.0, 1500.0, 0.001).is_err());
        assert!(try_optical_tof_to_range(-1.0, 1.0).is_err());
        assert!(try_deep_space_radio_rtt_to_range(-1.0, 1.0).is_err());
    }

    #[test]
    fn checked_conversions_reject_invalid_calibration_parameters() {
        assert!(try_rssi_to_range(-50.0, -50.0, 0.0).is_err());
        assert!(try_lora_toa_to_range(1.0, 0.0).is_err());
        assert!(try_manual_survey(1.0, 0.0).is_err());
        assert!(try_acoustic_tof_to_range(1.0, 0.0, 0.001).is_err());
        assert!(try_acoustic_tof_to_range(1.0, 1500.0, 0.0).is_err());
        assert!(try_optical_tof_to_range(1.0, 0.0).is_err());
        assert!(try_deep_space_radio_rtt_to_range(1.0, 0.0).is_err());
        assert!(try_meshtastic_hops_to_range(1, 0.0).is_err());
    }

    #[test]
    fn legacy_invalid_input_is_not_laundered_into_plausible_evidence() {
        let invalid = [
            uwb_tof_to_range(-1.0),
            wifi_rtt_to_range(f64::NAN),
            manual_survey(-1.0, 1.0),
            optical_tof_to_range(-1.0, 1.0),
        ];
        for estimate in invalid {
            assert!(!estimate.is_finite_and_non_negative());
            assert!(estimate.distance_m.is_nan());
            assert!(estimate.sigma_m.is_nan());
        }
    }

    #[test]
    fn valid_checked_methods_produce_finite_estimates() {
        let estimates = [
            try_rssi_to_range(-90.0, -40.0, 3.0).unwrap(),
            try_lora_toa_to_range(0.1, 125_000.0).unwrap(),
            try_uwb_tof_to_range(0.1).unwrap(),
            try_wifi_rtt_to_range(0.1).unwrap(),
            try_acoustic_tof_to_range(0.1, 1500.0, 0.001).unwrap(),
            try_manual_survey(0.0, 1.0).unwrap(),
            try_meshtastic_hops_to_range(0, 2000.0).unwrap(),
        ];
        assert!(estimates.iter().all(RangeEstimate::is_finite_and_non_negative));
    }
}
