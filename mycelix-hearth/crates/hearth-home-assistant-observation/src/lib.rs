// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Pure read-only Home Assistant observation contract for Mycelix Hearth.
//!
//! This crate deliberately contains no WebSocket client, endpoint, token,
//! credential, service call, or actuation API. Home Assistant contributes
//! observations only; it does not contribute household authority.

use hearth_automation_types::EntityRef;
use hearth_automation_types::{
    AutomationValue, EvidenceSource, Observation, BASIS_POINTS_MAX,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;

pub const HOME_ASSISTANT_OBSERVATION_SCHEMA_VERSION: u16 = 1;
pub const HOME_ASSISTANT_NAMESPACE: &str = "home-assistant";
pub const HA_STATE_UNKNOWN: &str = "unknown";
pub const HA_STATE_UNAVAILABLE: &str = "unavailable";
pub const MAX_PROFILE_RULES: usize = 1_024;
pub const MAX_RAW_STATE_BYTES: usize = 4_096;
pub const MAX_TEXT_STATE_BYTES: usize = 4_096;
pub const MAX_METADATA_BYTES: usize = 256;
pub const MAX_PROFILE_REF_BYTES: usize = 1_024;
pub const MAX_INSTANCE_ID_BYTES: usize = 128;
pub const MAX_MAX_AGE_MS: u64 = 7 * 24 * 60 * 60 * 1_000;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ObservationError {
    InvalidProfile(String),
    InvalidFrame(String),
    MetadataMismatch {
        entity_id: String,
        field: &'static str,
        expected: String,
        actual: Option<String>,
    },
    DecodeFailed {
        entity_id: String,
        reason: String,
    },
    FreshnessOverflow,
}

impl fmt::Display for ObservationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidProfile(message) => {
                write!(f, "invalid Home Assistant observation profile: {message}")
            }
            Self::InvalidFrame(message) => {
                write!(f, "invalid Home Assistant state frame: {message}")
            }
            Self::MetadataMismatch {
                entity_id,
                field,
                expected,
                actual,
            } => write!(
                f,
                "Home Assistant metadata mismatch for {entity_id}: {field} expected {expected:?}, got {actual:?}"
            ),
            Self::DecodeFailed { entity_id, reason } => {
                write!(f, "Home Assistant state decode failed for {entity_id}: {reason}")
            }
            Self::FreshnessOverflow => write!(f, "observation freshness timestamp overflow"),
        }
    }
}

impl std::error::Error for ObservationError {}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HomeAssistantObservationProfile {
    pub schema_version: u16,
    /// Stable local identifier for one Home Assistant installation.
    /// This is not an endpoint and must not contain credentials.
    pub instance_id: String,
    /// Immutable/profile-version reference selected by the local operator.
    pub profile_ref: String,
    pub rules: Vec<EntityObservationRule>,
}

impl HomeAssistantObservationProfile {
    pub fn validate(&self) -> Result<(), ObservationError> {
        if self.schema_version != HOME_ASSISTANT_OBSERVATION_SCHEMA_VERSION {
            return Err(ObservationError::InvalidProfile(
                "unsupported schema_version".into(),
            ));
        }
        validate_instance_id(&self.instance_id)?;
        validate_bounded_text(
            "profile_ref",
            &self.profile_ref,
            MAX_PROFILE_REF_BYTES,
            false,
        )
        .map_err(ObservationError::InvalidProfile)?;
        if self.rules.is_empty() {
            return Err(ObservationError::InvalidProfile(
                "rules must not be empty".into(),
            ));
        }
        if self.rules.len() > MAX_PROFILE_RULES {
            return Err(ObservationError::InvalidProfile(format!(
                "rules exceeds {MAX_PROFILE_RULES}"
            )));
        }

        let mut entities = BTreeSet::new();
        for (index, rule) in self.rules.iter().enumerate() {
            rule.validate(index)?;
            if !entities.insert(rule.entity_id.clone()) {
                return Err(ObservationError::InvalidProfile(format!(
                    "duplicate entity rule for {:?}",
                    rule.entity_id
                )));
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EntityObservationRule {
    /// Exact Home Assistant entity ID, for example `sensor.living_room_temperature`.
    pub entity_id: String,
    /// Hearth semantic subject; never a Home Assistant service/device API object.
    pub subject: EntityRef,
    /// Hearth semantic attribute, for example `temperature_celsius`.
    pub attribute: String,
    pub codec: StateCodec,
    /// Optional metadata assertions. These are drift guards, not values emitted
    /// into the observation.
    pub expected_unit: Option<String>,
    pub expected_device_class: Option<String>,
    /// Hearth freshness is derived from local receive time plus this bound.
    pub max_age_ms: u64,
    pub confidence_bp: u32,
}

impl EntityObservationRule {
    fn validate(&self, index: usize) -> Result<(), ObservationError> {
        validate_entity_id(&self.entity_id).map_err(|message| {
            ObservationError::InvalidProfile(format!("rules[{index}].entity_id: {message}"))
        })?;
        self.subject
            .validate(&format!("rules[{index}].subject"))
            .map_err(|error| ObservationError::InvalidProfile(error.to_string()))?;
        validate_bounded_text(
            &format!("rules[{index}].attribute"),
            &self.attribute,
            128,
            false,
        )
        .map_err(ObservationError::InvalidProfile)?;
        self.codec.validate(index)?;
        validate_optional_metadata(
            &format!("rules[{index}].expected_unit"),
            self.expected_unit.as_deref(),
        )?;
        validate_optional_metadata(
            &format!("rules[{index}].expected_device_class"),
            self.expected_device_class.as_deref(),
        )?;
        if self.max_age_ms == 0 || self.max_age_ms > MAX_MAX_AGE_MS {
            return Err(ObservationError::InvalidProfile(format!(
                "rules[{index}].max_age_ms must be in 1..={MAX_MAX_AGE_MS}"
            )));
        }
        if self.confidence_bp > BASIS_POINTS_MAX {
            return Err(ObservationError::InvalidProfile(format!(
                "rules[{index}].confidence_bp must be <= {BASIS_POINTS_MAX}"
            )));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum StateCodec {
    Bool {
        true_state: String,
        false_state: String,
    },
    SignedRange {
        min: i64,
        max: i64,
    },
    UnsignedRange {
        min: u64,
        max: u64,
    },
    BasisPointsRange {
        min: u32,
        max: u32,
    },
    FixedRange {
        scale: u8,
        min_mantissa: i64,
        max_mantissa: i64,
    },
    Text {
        max_bytes: u16,
        /// Empty means any non-reserved text within the byte bound.
        allowed_values: BTreeSet<String>,
    },
}

impl StateCodec {
    fn validate(&self, rule_index: usize) -> Result<(), ObservationError> {
        let path = format!("rules[{rule_index}].codec");
        match self {
            Self::Bool {
                true_state,
                false_state,
            } => {
                validate_state_token(&format!("{path}.true_state"), true_state)?;
                validate_state_token(&format!("{path}.false_state"), false_state)?;
                if true_state == false_state {
                    return Err(ObservationError::InvalidProfile(format!(
                        "{path}: true_state and false_state must differ"
                    )));
                }
            }
            Self::SignedRange { min, max } if min > max => {
                return Err(ObservationError::InvalidProfile(format!(
                    "{path}: min must be <= max"
                )));
            }
            Self::UnsignedRange { min, max } if min > max => {
                return Err(ObservationError::InvalidProfile(format!(
                    "{path}: min must be <= max"
                )));
            }
            Self::BasisPointsRange { min, max }
                if min > max || *max > BASIS_POINTS_MAX =>
            {
                return Err(ObservationError::InvalidProfile(format!(
                    "{path}: invalid basis-point range"
                )));
            }
            Self::FixedRange {
                scale,
                min_mantissa,
                max_mantissa,
            } => {
                if *scale > 18 {
                    return Err(ObservationError::InvalidProfile(format!(
                        "{path}.scale must be <= 18"
                    )));
                }
                if min_mantissa > max_mantissa {
                    return Err(ObservationError::InvalidProfile(format!(
                        "{path}: min_mantissa must be <= max_mantissa"
                    )));
                }
            }
            Self::Text {
                max_bytes,
                allowed_values,
            } => {
                if *max_bytes == 0 || usize::from(*max_bytes) > MAX_TEXT_STATE_BYTES {
                    return Err(ObservationError::InvalidProfile(format!(
                        "{path}.max_bytes must be in 1..={MAX_TEXT_STATE_BYTES}"
                    )));
                }
                for value in allowed_values {
                    validate_state_token(&format!("{path}.allowed_values"), value)?;
                    if value.len() > usize::from(*max_bytes) {
                        return Err(ObservationError::InvalidProfile(format!(
                            "{path}: allowed value exceeds max_bytes"
                        )));
                    }
                }
            }
            _ => {}
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HomeAssistantStateFrame {
    pub entity_id: String,
    /// Home Assistant's primary string state. Arbitrary attributes are not part
    /// of this contract.
    pub state: String,
    pub unit_of_measurement: Option<String>,
    pub device_class: Option<String>,
    /// Diagnostic source timestamp only; never used to establish freshness.
    pub source_last_updated_micros: Option<i64>,
    /// Timestamp assigned by the local Hearth transport when the frame arrived.
    pub received_at_micros: i64,
}

impl HomeAssistantStateFrame {
    pub fn validate(&self) -> Result<(), ObservationError> {
        validate_entity_id(&self.entity_id).map_err(ObservationError::InvalidFrame)?;
        if self.state.is_empty() {
            return Err(ObservationError::InvalidFrame(
                "state must not be empty".into(),
            ));
        }
        if self.state.len() > MAX_RAW_STATE_BYTES {
            return Err(ObservationError::InvalidFrame(format!(
                "state exceeds {MAX_RAW_STATE_BYTES} bytes"
            )));
        }
        if self.state.contains('\0') {
            return Err(ObservationError::InvalidFrame(
                "state must not contain NUL".into(),
            ));
        }
        validate_frame_metadata("unit_of_measurement", self.unit_of_measurement.as_deref())?;
        validate_frame_metadata("device_class", self.device_class.as_deref())?;
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum HomeAssistantSourceStatus {
    Unknown,
    Unavailable,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HomeAssistantObservation {
    pub profile_ref: String,
    pub entity_id: String,
    pub source_last_updated_micros: Option<i64>,
    pub received_at_micros: i64,
    pub observation: Observation,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum MappingOutcome {
    Observation(HomeAssistantObservation),
    SourceState {
        entity_id: String,
        status: HomeAssistantSourceStatus,
        source_last_updated_micros: Option<i64>,
        received_at_micros: i64,
    },
    NotAllowlisted {
        entity_id: String,
    },
}

pub fn map_state(
    profile: &HomeAssistantObservationProfile,
    frame: &HomeAssistantStateFrame,
) -> Result<MappingOutcome, ObservationError> {
    profile.validate()?;
    frame.validate()?;

    let Some(rule) = profile.rules.iter().find(|rule| rule.entity_id == frame.entity_id) else {
        return Ok(MappingOutcome::NotAllowlisted {
            entity_id: frame.entity_id.clone(),
        });
    };

    require_metadata(
        &frame.entity_id,
        "unit_of_measurement",
        rule.expected_unit.as_deref(),
        frame.unit_of_measurement.as_deref(),
    )?;
    require_metadata(
        &frame.entity_id,
        "device_class",
        rule.expected_device_class.as_deref(),
        frame.device_class.as_deref(),
    )?;

    let status = match frame.state.as_str() {
        HA_STATE_UNKNOWN => Some(HomeAssistantSourceStatus::Unknown),
        HA_STATE_UNAVAILABLE => Some(HomeAssistantSourceStatus::Unavailable),
        _ => None,
    };
    if let Some(status) = status {
        return Ok(MappingOutcome::SourceState {
            entity_id: frame.entity_id.clone(),
            status,
            source_last_updated_micros: frame.source_last_updated_micros,
            received_at_micros: frame.received_at_micros,
        });
    }

    let value = decode_state(&rule.codec, &frame.entity_id, &frame.state)?;
    let freshness_micros = i64::try_from(rule.max_age_ms)
        .ok()
        .and_then(|value| value.checked_mul(1_000))
        .ok_or(ObservationError::FreshnessOverflow)?;
    let valid_until_micros = frame
        .received_at_micros
        .checked_add(freshness_micros)
        .ok_or(ObservationError::FreshnessOverflow)?;

    let observation = Observation {
        subject: rule.subject.clone(),
        attribute: rule.attribute.clone(),
        value,
        source: EvidenceSource {
            namespace: HOME_ASSISTANT_NAMESPACE.into(),
            id: format!("{}:{}", profile.instance_id, frame.entity_id),
        },
        // Deliberately local receive time, not Home Assistant's clock.
        observed_at_micros: frame.received_at_micros,
        valid_until_micros: Some(valid_until_micros),
        confidence_bp: rule.confidence_bp,
    };
    observation
        .validate("observation")
        .map_err(|error| ObservationError::DecodeFailed {
            entity_id: frame.entity_id.clone(),
            reason: error.to_string(),
        })?;

    Ok(MappingOutcome::Observation(HomeAssistantObservation {
        profile_ref: profile.profile_ref.clone(),
        entity_id: frame.entity_id.clone(),
        source_last_updated_micros: frame.source_last_updated_micros,
        received_at_micros: frame.received_at_micros,
        observation,
    }))
}

fn decode_state(
    codec: &StateCodec,
    entity_id: &str,
    state: &str,
) -> Result<AutomationValue, ObservationError> {
    let fail = |reason: String| ObservationError::DecodeFailed {
        entity_id: entity_id.into(),
        reason,
    };

    match codec {
        StateCodec::Bool {
            true_state,
            false_state,
        } => {
            if state == true_state {
                Ok(AutomationValue::Bool(true))
            } else if state == false_state {
                Ok(AutomationValue::Bool(false))
            } else {
                Err(fail(format!(
                    "expected exact boolean token {:?} or {:?}",
                    true_state, false_state
                )))
            }
        }
        StateCodec::SignedRange { min, max } => {
            let value = state
                .parse::<i64>()
                .map_err(|_| fail("expected signed base-10 integer".into()))?;
            if value < *min || value > *max {
                return Err(fail("signed value outside admitted range".into()));
            }
            Ok(AutomationValue::Signed(value))
        }
        StateCodec::UnsignedRange { min, max } => {
            let value = state
                .parse::<u64>()
                .map_err(|_| fail("expected unsigned base-10 integer".into()))?;
            if value < *min || value > *max {
                return Err(fail("unsigned value outside admitted range".into()));
            }
            Ok(AutomationValue::Unsigned(value))
        }
        StateCodec::BasisPointsRange { min, max } => {
            let value = state
                .parse::<u32>()
                .map_err(|_| fail("expected basis-points integer".into()))?;
            if value < *min || value > *max || value > BASIS_POINTS_MAX {
                return Err(fail("basis-points value outside admitted range".into()));
            }
            Ok(AutomationValue::BasisPoints(value))
        }
        StateCodec::FixedRange {
            scale,
            min_mantissa,
            max_mantissa,
        } => {
            let mantissa = parse_fixed(state, *scale)
                .map_err(|reason| fail(format!("invalid fixed-point decimal: {reason}")))?;
            if mantissa < *min_mantissa || mantissa > *max_mantissa {
                return Err(fail("fixed-point value outside admitted range".into()));
            }
            Ok(AutomationValue::Fixed {
                mantissa,
                scale: *scale,
            })
        }
        StateCodec::Text {
            max_bytes,
            allowed_values,
        } => {
            if state.len() > usize::from(*max_bytes) {
                return Err(fail("text state exceeds admitted byte bound".into()));
            }
            if !allowed_values.is_empty() && !allowed_values.contains(state) {
                return Err(fail("text state is not allowlisted".into()));
            }
            Ok(AutomationValue::Text(state.into()))
        }
    }
}

fn parse_fixed(input: &str, scale: u8) -> Result<i64, &'static str> {
    if input.is_empty() || input.trim() != input {
        return Err("empty or whitespace-padded input");
    }
    if input.contains('e') || input.contains('E') || input.starts_with('+') {
        return Err("exponent and explicit plus syntax are not admitted");
    }

    let (negative, digits) = match input.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, input),
    };
    if digits.is_empty() {
        return Err("missing digits");
    }

    let mut parts = digits.split('.');
    let whole = parts.next().ok_or("missing whole part")?;
    let fraction = parts.next();
    if parts.next().is_some() || whole.is_empty() {
        return Err("invalid decimal structure");
    }
    if !whole.bytes().all(|byte| byte.is_ascii_digit()) {
        return Err("whole part contains non-digits");
    }

    let fraction = fraction.unwrap_or("");
    if digits.contains('.') && fraction.is_empty() {
        return Err("decimal point requires fractional digits");
    }
    if !fraction.bytes().all(|byte| byte.is_ascii_digit()) {
        return Err("fraction contains non-digits");
    }
    if fraction.len() > usize::from(scale) {
        return Err("too many fractional digits for declared scale");
    }

    let factor = pow10_i128(scale)?;
    let whole_value = parse_digits_i128(whole)?;
    let fraction_value = if fraction.is_empty() {
        0
    } else {
        let raw = parse_digits_i128(fraction)?;
        raw.checked_mul(pow10_i128(scale - fraction.len() as u8)?)
            .ok_or("fraction overflow")?
    };
    let magnitude = whole_value
        .checked_mul(factor)
        .and_then(|value| value.checked_add(fraction_value))
        .ok_or("fixed-point overflow")?;
    let signed = if negative {
        magnitude.checked_neg().ok_or("fixed-point overflow")?
    } else {
        magnitude
    };
    i64::try_from(signed).map_err(|_| "fixed-point value exceeds i64")
}

fn parse_digits_i128(value: &str) -> Result<i128, &'static str> {
    let mut out = 0_i128;
    for byte in value.bytes() {
        if !byte.is_ascii_digit() {
            return Err("non-digit");
        }
        out = out
            .checked_mul(10)
            .and_then(|current| current.checked_add(i128::from(byte - b'0')))
            .ok_or("integer overflow")?;
    }
    Ok(out)
}

fn pow10_i128(scale: u8) -> Result<i128, &'static str> {
    let mut value = 1_i128;
    for _ in 0..scale {
        value = value.checked_mul(10).ok_or("scale overflow")?;
    }
    Ok(value)
}

fn validate_instance_id(value: &str) -> Result<(), ObservationError> {
    if value.is_empty() || value.len() > MAX_INSTANCE_ID_BYTES {
        return Err(ObservationError::InvalidProfile(format!(
            "instance_id must be in 1..={MAX_INSTANCE_ID_BYTES} bytes"
        )));
    }
    if !value
        .bytes()
        .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b'-'))
    {
        return Err(ObservationError::InvalidProfile(
            "instance_id may contain only ASCII alphanumeric, '.', '_' or '-'".into(),
        ));
    }
    Ok(())
}

fn validate_entity_id(value: &str) -> Result<(), String> {
    if value.is_empty() || value.len() > 255 {
        return Err("entity_id must be in 1..=255 bytes".into());
    }
    let Some((domain, object)) = value.split_once('.') else {
        return Err("entity_id must contain exactly one domain separator".into());
    };
    if object.contains('.') || domain.is_empty() || object.is_empty() {
        return Err("entity_id must be exactly domain.object_id".into());
    }
    let valid_half = |part: &str| {
        part.bytes()
            .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit() || byte == b'_')
    };
    if !valid_half(domain) || !valid_half(object) {
        return Err("entity_id halves may contain only [a-z0-9_]".into());
    }
    Ok(())
}

fn validate_state_token(path: &str, value: &str) -> Result<(), ObservationError> {
    validate_bounded_text(path, value, MAX_RAW_STATE_BYTES, true)
        .map_err(ObservationError::InvalidProfile)?;
    if value == HA_STATE_UNKNOWN || value == HA_STATE_UNAVAILABLE {
        return Err(ObservationError::InvalidProfile(format!(
            "{path}: reserved epistemic Home Assistant state cannot be used as a value token"
        )));
    }
    Ok(())
}

fn validate_optional_metadata(path: &str, value: Option<&str>) -> Result<(), ObservationError> {
    if let Some(value) = value {
        validate_bounded_text(path, value, MAX_METADATA_BYTES, false)
            .map_err(ObservationError::InvalidProfile)?;
    }
    Ok(())
}

fn validate_frame_metadata(path: &str, value: Option<&str>) -> Result<(), ObservationError> {
    if let Some(value) = value {
        validate_bounded_text(path, value, MAX_METADATA_BYTES, false)
            .map_err(ObservationError::InvalidFrame)?;
    }
    Ok(())
}

fn validate_bounded_text(
    path: &str,
    value: &str,
    max_bytes: usize,
    allow_space: bool,
) -> Result<(), String> {
    if value.is_empty() {
        return Err(format!("{path} must not be empty"));
    }
    if value.len() > max_bytes {
        return Err(format!("{path} exceeds {max_bytes} bytes"));
    }
    if value.chars().any(char::is_control) {
        return Err(format!("{path} must not contain control characters"));
    }
    if !allow_space && value.trim() != value {
        return Err(format!("{path} must not have leading/trailing whitespace"));
    }
    Ok(())
}

fn require_metadata(
    entity_id: &str,
    field: &'static str,
    expected: Option<&str>,
    actual: Option<&str>,
) -> Result<(), ObservationError> {
    if let Some(expected) = expected
        && actual != Some(expected)
    {
        return Err(ObservationError::MetadataMismatch {
            entity_id: entity_id.into(),
            field,
            expected: expected.into(),
            actual: actual.map(str::to_owned),
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base_rule(codec: StateCodec) -> EntityObservationRule {
        EntityObservationRule {
            entity_id: "sensor.living_room_temperature".into(),
            subject: EntityRef {
                kind: "room".into(),
                id: "living-room".into(),
            },
            attribute: "temperature_celsius".into(),
            codec,
            expected_unit: Some("°C".into()),
            expected_device_class: Some("temperature".into()),
            max_age_ms: 30_000,
            confidence_bp: 9_500,
        }
    }

    fn profile(codec: StateCodec) -> HomeAssistantObservationProfile {
        HomeAssistantObservationProfile {
            schema_version: HOME_ASSISTANT_OBSERVATION_SCHEMA_VERSION,
            instance_id: "home-main".into(),
            profile_ref: "ha-observation-profile-v1:test".into(),
            rules: vec![base_rule(codec)],
        }
    }

    fn frame(state: &str) -> HomeAssistantStateFrame {
        HomeAssistantStateFrame {
            entity_id: "sensor.living_room_temperature".into(),
            state: state.into(),
            unit_of_measurement: Some("°C".into()),
            device_class: Some("temperature".into()),
            source_last_updated_micros: Some(1),
            received_at_micros: 10_000_000,
        }
    }

    #[test]
    fn exact_boolean_tokens_map() {
        let mut rule = base_rule(StateCodec::Bool {
            true_state: "on".into(),
            false_state: "off".into(),
        });
        rule.entity_id = "binary_sensor.front_door".into();
        rule.attribute = "open".into();
        rule.expected_unit = None;
        rule.expected_device_class = Some("door".into());
        let profile = HomeAssistantObservationProfile {
            schema_version: 1,
            instance_id: "home-main".into(),
            profile_ref: "profile:door".into(),
            rules: vec![rule],
        };
        let frame = HomeAssistantStateFrame {
            entity_id: "binary_sensor.front_door".into(),
            state: "on".into(),
            unit_of_measurement: None,
            device_class: Some("door".into()),
            source_last_updated_micros: None,
            received_at_micros: 100,
        };
        let MappingOutcome::Observation(mapped) = map_state(&profile, &frame).unwrap() else {
            panic!("expected observation")
        };
        assert_eq!(mapped.observation.value, AutomationValue::Bool(true));
    }

    #[test]
    fn unknown_and_unavailable_never_become_values() {
        let profile = profile(StateCodec::FixedRange {
            scale: 2,
            min_mantissa: -5_000,
            max_mantissa: 8_000,
        });
        assert!(matches!(
            map_state(&profile, &frame(HA_STATE_UNKNOWN)).unwrap(),
            MappingOutcome::SourceState {
                status: HomeAssistantSourceStatus::Unknown,
                ..
            }
        ));
        assert!(matches!(
            map_state(&profile, &frame(HA_STATE_UNAVAILABLE)).unwrap(),
            MappingOutcome::SourceState {
                status: HomeAssistantSourceStatus::Unavailable,
                ..
            }
        ));
    }

    #[test]
    fn non_allowlisted_entity_is_ignored() {
        let profile = profile(StateCodec::Text {
            max_bytes: 32,
            allowed_values: BTreeSet::new(),
        });
        let mut frame = frame("hello");
        frame.entity_id = "sensor.other".into();
        assert_eq!(
            map_state(&profile, &frame).unwrap(),
            MappingOutcome::NotAllowlisted {
                entity_id: "sensor.other".into()
            }
        );
    }

    #[test]
    fn metadata_drift_fails_closed() {
        let profile = profile(StateCodec::FixedRange {
            scale: 1,
            min_mantissa: -500,
            max_mantissa: 800,
        });
        let mut frame = frame("21.5");
        frame.unit_of_measurement = Some("°F".into());
        assert!(matches!(
            map_state(&profile, &frame),
            Err(ObservationError::MetadataMismatch { .. })
        ));
    }

    #[test]
    fn fixed_decimal_is_integer_deterministic() {
        let profile = profile(StateCodec::FixedRange {
            scale: 2,
            min_mantissa: -5_000,
            max_mantissa: 8_000,
        });
        let MappingOutcome::Observation(mapped) = map_state(&profile, &frame("21.50")).unwrap()
        else {
            panic!("expected observation")
        };
        assert_eq!(
            mapped.observation.value,
            AutomationValue::Fixed {
                mantissa: 2_150,
                scale: 2
            }
        );
    }

    #[test]
    fn numeric_range_is_enforced() {
        let profile = profile(StateCodec::SignedRange { min: -10, max: 10 });
        assert!(matches!(
            map_state(&profile, &frame("11")),
            Err(ObservationError::DecodeFailed { .. })
        ));
    }

    #[test]
    fn duplicate_entity_rules_are_invalid() {
        let rule = base_rule(StateCodec::Text {
            max_bytes: 16,
            allowed_values: BTreeSet::new(),
        });
        let profile = HomeAssistantObservationProfile {
            schema_version: 1,
            instance_id: "home-main".into(),
            profile_ref: "profile:test".into(),
            rules: vec![rule.clone(), rule],
        };
        assert!(matches!(
            profile.validate(),
            Err(ObservationError::InvalidProfile(_))
        ));
    }

    #[test]
    fn freshness_uses_local_receive_time_not_source_clock() {
        let profile = profile(StateCodec::FixedRange {
            scale: 1,
            min_mantissa: -500,
            max_mantissa: 800,
        });
        let mut frame = frame("21.5");
        frame.source_last_updated_micros = Some(9_999_999_999);
        frame.received_at_micros = 1_000_000;
        let MappingOutcome::Observation(mapped) = map_state(&profile, &frame).unwrap() else {
            panic!("expected observation")
        };
        assert_eq!(mapped.observation.observed_at_micros, 1_000_000);
        assert_eq!(mapped.observation.valid_until_micros, Some(31_000_000));
        assert_eq!(mapped.source_last_updated_micros, Some(9_999_999_999));
    }

    #[test]
    fn reserved_boolean_token_is_rejected_at_profile_validation() {
        let profile = profile(StateCodec::Bool {
            true_state: HA_STATE_UNKNOWN.into(),
            false_state: "off".into(),
        });
        assert!(matches!(
            profile.validate(),
            Err(ObservationError::InvalidProfile(_))
        ));
    }

    #[test]
    fn bounded_text_allowlist_is_exact() {
        let allowed_values = BTreeSet::from(["idle".to_string(), "running".to_string()]);
        let profile = profile(StateCodec::Text {
            max_bytes: 16,
            allowed_values,
        });
        let MappingOutcome::Observation(mapped) = map_state(&profile, &frame("running")).unwrap()
        else {
            panic!("expected observation")
        };
        assert_eq!(
            mapped.observation.value,
            AutomationValue::Text("running".into())
        );
        assert!(matches!(
            map_state(&profile, &frame("stopped")),
            Err(ObservationError::DecodeFailed { .. })
        ));
    }
}
