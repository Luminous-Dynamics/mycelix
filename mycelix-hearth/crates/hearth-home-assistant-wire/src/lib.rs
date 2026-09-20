// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Bounded, secret-free Home Assistant WebSocket JSON codec.
//!
//! Raw JSON bytes are untrusted transport data. This crate admits only the
//! read-only message classes required by HA-0.2A and immediately minimizes Home
//! Assistant state objects to the HA-0.1 observation frame.

use hearth_home_assistant_observation::{HomeAssistantStateFrame, ObservationError};
use hearth_home_assistant_protocol::{CommandId, ReadOnlyCommand};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use std::fmt;

pub const MAX_AUTH_MESSAGE_BYTES: usize = 4 * 1024;
pub const MAX_CONTROL_MESSAGE_BYTES: usize = 64 * 1024;
pub const MAX_EVENT_MESSAGE_BYTES: usize = 4 * 1024 * 1024;
pub const MAX_BOOTSTRAP_MESSAGE_BYTES: usize = 16 * 1024 * 1024;
pub const MAX_BOOTSTRAP_STATES: usize = 65_536;
pub const MAX_HA_VERSION_BYTES: usize = 128;
pub const MAX_SERVER_ERROR_CODE_BYTES: usize = 128;
pub const MAX_SERVER_ERROR_MESSAGE_BYTES: usize = 1_024;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JsonErrorCategory {
    Io,
    Syntax,
    Data,
    Eof,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WireError {
    Oversized {
        limit: usize,
        actual: usize,
    },
    Json {
        category: JsonErrorCategory,
        line: usize,
        column: usize,
    },
    Serialize,
    UnexpectedMessageType,
    InvalidField(&'static str),
    InvalidText(&'static str),
    BootstrapTooLarge {
        limit: usize,
        actual: usize,
    },
    State(ObservationError),
}

impl fmt::Display for WireError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Oversized { limit, actual } => {
                write!(f, "Home Assistant wire message is {actual} bytes; limit is {limit}")
            }
            Self::Json {
                category,
                line,
                column,
            } => write!(
                f,
                "invalid Home Assistant JSON ({category:?}) at line {line}, column {column}"
            ),
            Self::Serialize => write!(f, "failed to serialize Home Assistant read-only command"),
            Self::UnexpectedMessageType => write!(f, "unexpected Home Assistant message type"),
            Self::InvalidField(field) => write!(f, "invalid Home Assistant field: {field}"),
            Self::InvalidText(field) => write!(f, "unsafe or oversized Home Assistant text field: {field}"),
            Self::BootstrapTooLarge { limit, actual } => write!(
                f,
                "Home Assistant bootstrap contains {actual} states; limit is {limit}"
            ),
            Self::State(error) => write!(f, "invalid Home Assistant state frame: {error}"),
        }
    }
}

impl std::error::Error for WireError {}

impl From<ObservationError> for WireError {
    fn from(value: ObservationError) -> Self {
        Self::State(value)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthRequired {
    pub ha_version: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AuthReply {
    Ok { ha_version: String },
    Invalid { message: String },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WireFailure {
    pub code: String,
    pub message: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StatesResult {
    pub id: CommandId,
    pub success: bool,
    pub states: Vec<HomeAssistantStateFrame>,
    pub error: Option<WireFailure>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SubscriptionResult {
    pub id: CommandId,
    pub success: bool,
    pub error: Option<WireFailure>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StateChangedEvent {
    pub subscription_id: CommandId,
    pub entity_id: String,
    pub new_state: Option<HomeAssistantStateFrame>,
    pub received_at_micros: i64,
}

pub fn parse_auth_required(input: &[u8]) -> Result<AuthRequired, WireError> {
    let raw: RawAuthRequired = decode_json(input, MAX_AUTH_MESSAGE_BYTES)?;
    require_type(&raw.kind, "auth_required")?;
    validate_clean_text("ha_version", &raw.ha_version, MAX_HA_VERSION_BYTES, false)?;
    Ok(AuthRequired {
        ha_version: raw.ha_version,
    })
}

pub fn parse_auth_reply(input: &[u8]) -> Result<AuthReply, WireError> {
    let probe: RawTypeProbe = decode_json(input, MAX_AUTH_MESSAGE_BYTES)?;
    match probe.kind.as_str() {
        "auth_ok" => {
            let raw: RawAuthOk = decode_json(input, MAX_AUTH_MESSAGE_BYTES)?;
            validate_clean_text("ha_version", &raw.ha_version, MAX_HA_VERSION_BYTES, false)?;
            Ok(AuthReply::Ok {
                ha_version: raw.ha_version,
            })
        }
        "auth_invalid" => {
            let raw: RawAuthInvalid = decode_json(input, MAX_AUTH_MESSAGE_BYTES)?;
            validate_clean_text(
                "auth_invalid.message",
                &raw.message,
                MAX_SERVER_ERROR_MESSAGE_BYTES,
                false,
            )?;
            Ok(AuthReply::Invalid {
                message: raw.message,
            })
        }
        _ => Err(WireError::UnexpectedMessageType),
    }
}

pub fn encode_read_only_command(command: &ReadOnlyCommand) -> Result<Vec<u8>, WireError> {
    match command {
        ReadOnlyCommand::GetStates { id, .. } => serde_json::to_vec(&GetStatesCommand {
            id: id.0,
            kind: "get_states",
        })
        .map_err(|_| WireError::Serialize),
        ReadOnlyCommand::SubscribeStateChanged { id, .. } => {
            serde_json::to_vec(&SubscribeStateChangedCommand {
                id: id.0,
                kind: "subscribe_events",
                event_type: "state_changed",
            })
            .map_err(|_| WireError::Serialize)
        }
    }
}

pub fn parse_states_result(
    input: &[u8],
    received_at_micros: i64,
) -> Result<StatesResult, WireError> {
    let raw: RawStatesResult = decode_json(input, MAX_BOOTSTRAP_MESSAGE_BYTES)?;
    require_type(&raw.kind, "result")?;

    if raw.success {
        if raw.error.is_some() {
            return Err(WireError::InvalidField("result.error"));
        }
        let raw_states = raw
            .result
            .ok_or(WireError::InvalidField("result.result"))?;
        if raw_states.len() > MAX_BOOTSTRAP_STATES {
            return Err(WireError::BootstrapTooLarge {
                limit: MAX_BOOTSTRAP_STATES,
                actual: raw_states.len(),
            });
        }
        let mut states = Vec::with_capacity(raw_states.len());
        for raw_state in raw_states {
            states.push(minimize_state(raw_state, received_at_micros)?);
        }
        Ok(StatesResult {
            id: CommandId(raw.id),
            success: true,
            states,
            error: None,
        })
    } else {
        if raw.result.is_some() {
            return Err(WireError::InvalidField("result.result"));
        }
        let error = raw
            .error
            .ok_or(WireError::InvalidField("result.error"))?;
        Ok(StatesResult {
            id: CommandId(raw.id),
            success: false,
            states: Vec::new(),
            error: Some(validate_failure(error)?),
        })
    }
}

pub fn parse_subscription_result(input: &[u8]) -> Result<SubscriptionResult, WireError> {
    let raw: RawSubscriptionResult = decode_json(input, MAX_CONTROL_MESSAGE_BYTES)?;
    require_type(&raw.kind, "result")?;

    let error = match (raw.success, raw.error) {
        (true, None) => None,
        (true, Some(_)) => return Err(WireError::InvalidField("result.error")),
        (false, Some(error)) => Some(validate_failure(error)?),
        (false, None) => return Err(WireError::InvalidField("result.error")),
    };

    Ok(SubscriptionResult {
        id: CommandId(raw.id),
        success: raw.success,
        error,
    })
}

pub fn parse_state_changed_event(
    input: &[u8],
    received_at_micros: i64,
) -> Result<StateChangedEvent, WireError> {
    let raw: RawEventEnvelope = decode_json(input, MAX_EVENT_MESSAGE_BYTES)?;
    require_type(&raw.kind, "event")?;
    if raw.event.event_type != "state_changed" {
        return Err(WireError::UnexpectedMessageType);
    }
    validate_entity_id(&raw.event.data.entity_id)?;

    let new_state = match raw.event.data.new_state {
        Some(state) => Some(minimize_state(state, received_at_micros)?),
        None => None,
    };

    Ok(StateChangedEvent {
        subscription_id: CommandId(raw.id),
        entity_id: raw.event.data.entity_id,
        new_state,
        received_at_micros,
    })
}

fn minimize_state(
    raw: RawState,
    received_at_micros: i64,
) -> Result<HomeAssistantStateFrame, WireError> {
    let frame = HomeAssistantStateFrame {
        entity_id: raw.entity_id,
        state: raw.state,
        unit_of_measurement: raw.attributes.unit_of_measurement,
        device_class: raw.attributes.device_class,
        // Source time is optional diagnostic provenance in HA-0.1 and is not
        // used for freshness. Strict RFC3339 parsing can be added independently
        // without changing the authority theorem.
        source_last_updated_micros: None,
        received_at_micros,
    };
    frame.validate()?;
    Ok(frame)
}

fn validate_failure(raw: RawFailure) -> Result<WireFailure, WireError> {
    validate_clean_text(
        "error.code",
        &raw.code,
        MAX_SERVER_ERROR_CODE_BYTES,
        false,
    )?;
    validate_clean_text(
        "error.message",
        &raw.message,
        MAX_SERVER_ERROR_MESSAGE_BYTES,
        false,
    )?;
    Ok(WireFailure {
        code: raw.code,
        message: raw.message,
    })
}

fn require_type(actual: &str, expected: &str) -> Result<(), WireError> {
    if actual == expected {
        Ok(())
    } else {
        Err(WireError::UnexpectedMessageType)
    }
}

fn validate_clean_text(
    field: &'static str,
    value: &str,
    max_bytes: usize,
    allow_empty: bool,
) -> Result<(), WireError> {
    if (!allow_empty && value.is_empty())
        || value.len() > max_bytes
        || value.chars().any(char::is_control)
        || value.trim() != value
    {
        return Err(WireError::InvalidText(field));
    }
    Ok(())
}

fn validate_entity_id(value: &str) -> Result<(), WireError> {
    if value.is_empty() || value.len() > 255 {
        return Err(WireError::InvalidField("event.data.entity_id"));
    }
    let Some((domain, object)) = value.split_once('.') else {
        return Err(WireError::InvalidField("event.data.entity_id"));
    };
    if object.contains('.') || domain.is_empty() || object.is_empty() {
        return Err(WireError::InvalidField("event.data.entity_id"));
    }
    let valid = |part: &str| {
        part.bytes()
            .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit() || byte == b'_')
    };
    if !valid(domain) || !valid(object) {
        return Err(WireError::InvalidField("event.data.entity_id"));
    }
    Ok(())
}

fn decode_json<T: DeserializeOwned>(input: &[u8], limit: usize) -> Result<T, WireError> {
    if input.len() > limit {
        return Err(WireError::Oversized {
            limit,
            actual: input.len(),
        });
    }
    serde_json::from_slice(input).map_err(|error| WireError::Json {
        category: match error.classify() {
            serde_json::error::Category::Io => JsonErrorCategory::Io,
            serde_json::error::Category::Syntax => JsonErrorCategory::Syntax,
            serde_json::error::Category::Data => JsonErrorCategory::Data,
            serde_json::error::Category::Eof => JsonErrorCategory::Eof,
        },
        line: error.line(),
        column: error.column(),
    })
}

#[derive(Debug, Deserialize)]
struct RawTypeProbe {
    #[serde(rename = "type")]
    kind: String,
}

#[derive(Debug, Deserialize)]
struct RawAuthRequired {
    #[serde(rename = "type")]
    kind: String,
    ha_version: String,
}

#[derive(Debug, Deserialize)]
struct RawAuthOk {
    #[serde(rename = "type")]
    _kind: String,
    ha_version: String,
}

#[derive(Debug, Deserialize)]
struct RawAuthInvalid {
    #[serde(rename = "type")]
    _kind: String,
    message: String,
}

#[derive(Debug, Serialize)]
struct GetStatesCommand {
    id: u64,
    #[serde(rename = "type")]
    kind: &'static str,
}

#[derive(Debug, Serialize)]
struct SubscribeStateChangedCommand {
    id: u64,
    #[serde(rename = "type")]
    kind: &'static str,
    event_type: &'static str,
}

#[derive(Debug, Deserialize)]
struct RawStatesResult {
    id: u64,
    #[serde(rename = "type")]
    kind: String,
    success: bool,
    #[serde(default)]
    result: Option<Vec<RawState>>,
    #[serde(default)]
    error: Option<RawFailure>,
}

#[derive(Debug, Deserialize)]
struct RawSubscriptionResult {
    id: u64,
    #[serde(rename = "type")]
    kind: String,
    success: bool,
    #[serde(default)]
    error: Option<RawFailure>,
}

#[derive(Debug, Deserialize)]
struct RawFailure {
    code: String,
    message: String,
}

#[derive(Debug, Deserialize)]
struct RawEventEnvelope {
    id: u64,
    #[serde(rename = "type")]
    kind: String,
    event: RawEvent,
}

#[derive(Debug, Deserialize)]
struct RawEvent {
    event_type: String,
    data: RawStateChangedData,
}

#[derive(Debug, Deserialize)]
struct RawStateChangedData {
    entity_id: String,
    #[serde(default)]
    new_state: Option<RawState>,
}

#[derive(Debug, Deserialize)]
struct RawState {
    entity_id: String,
    state: String,
    attributes: RawAttributes,
}

#[derive(Debug, Default, Deserialize)]
struct RawAttributes {
    #[serde(default)]
    unit_of_measurement: Option<String>,
    #[serde(default)]
    device_class: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_home_assistant_protocol::{SessionEpoch, STATE_CHANGED_EVENT_TYPE};

    #[test]
    fn parses_auth_required() {
        let parsed = parse_auth_required(br#"{"type":"auth_required","ha_version":"2026.9.0"}"#)
            .unwrap();
        assert_eq!(parsed.ha_version, "2026.9.0");
    }

    #[test]
    fn parses_auth_ok_and_invalid() {
        assert_eq!(
            parse_auth_reply(br#"{"type":"auth_ok","ha_version":"2026.9.0"}"#).unwrap(),
            AuthReply::Ok {
                ha_version: "2026.9.0".into()
            }
        );
        assert_eq!(
            parse_auth_reply(br#"{"type":"auth_invalid","message":"Invalid access token"}"#)
                .unwrap(),
            AuthReply::Invalid {
                message: "Invalid access token".into()
            }
        );
    }

    #[test]
    fn encodes_only_exact_read_only_commands() {
        let get = ReadOnlyCommand::GetStates {
            epoch: SessionEpoch(7),
            id: CommandId(1),
        };
        assert_eq!(
            String::from_utf8(encode_read_only_command(&get).unwrap()).unwrap(),
            r#"{"id":1,"type":"get_states"}"#
        );

        let subscribe = ReadOnlyCommand::SubscribeStateChanged {
            epoch: SessionEpoch(7),
            id: CommandId(2),
        };
        assert_eq!(
            String::from_utf8(encode_read_only_command(&subscribe).unwrap()).unwrap(),
            format!(
                r#"{{"id":2,"type":"subscribe_events","event_type":"{STATE_CHANGED_EVENT_TYPE}"}}"#
            )
        );
    }

    #[test]
    fn bootstrap_minimizes_arbitrary_attributes() {
        let input = br#"{
            "id":1,
            "type":"result",
            "success":true,
            "result":[{
                "entity_id":"sensor.room_temperature",
                "state":"21.5",
                "attributes":{
                    "unit_of_measurement":"\u00b0C",
                    "device_class":"temperature",
                    "friendly_name":"Living Room Temperature",
                    "latitude":-26.2,
                    "longitude":28.0,
                    "some_large_private_blob":"discard-me"
                },
                "last_changed":"2026-09-20T12:00:00+00:00",
                "last_updated":"2026-09-20T12:00:00+00:00",
                "context":{"id":"ignored"}
            }]
        }"#;
        let parsed = parse_states_result(input, 123).unwrap();
        assert!(parsed.success);
        assert_eq!(parsed.states.len(), 1);
        let state = &parsed.states[0];
        assert_eq!(state.entity_id, "sensor.room_temperature");
        assert_eq!(state.state, "21.5");
        assert_eq!(state.unit_of_measurement.as_deref(), Some("°C"));
        assert_eq!(state.device_class.as_deref(), Some("temperature"));
        assert_eq!(state.source_last_updated_micros, None);
        assert_eq!(state.received_at_micros, 123);
    }

    #[test]
    fn unknown_state_is_preserved_for_ha01() {
        let input = br#"{
            "id":1,
            "type":"result",
            "success":true,
            "result":[{
                "entity_id":"sensor.room_temperature",
                "state":"unknown",
                "attributes":{}
            }]
        }"#;
        let parsed = parse_states_result(input, 10).unwrap();
        assert_eq!(parsed.states[0].state, "unknown");
    }

    #[test]
    fn state_changed_null_new_state_becomes_removal_input() {
        let input = br#"{
            "id":2,
            "type":"event",
            "event":{
                "event_type":"state_changed",
                "data":{
                    "entity_id":"sensor.room_temperature",
                    "old_state":{"ignored":true},
                    "new_state":null
                }
            }
        }"#;
        let parsed = parse_state_changed_event(input, 999).unwrap();
        assert_eq!(parsed.subscription_id, CommandId(2));
        assert_eq!(parsed.entity_id, "sensor.room_temperature");
        assert_eq!(parsed.new_state, None);
        assert_eq!(parsed.received_at_micros, 999);
    }

    #[test]
    fn state_changed_minimizes_new_state() {
        let input = br#"{
            "id":2,
            "type":"event",
            "event":{
                "event_type":"state_changed",
                "data":{
                    "entity_id":"binary_sensor.front_door",
                    "new_state":{
                        "entity_id":"binary_sensor.front_door",
                        "state":"on",
                        "attributes":{
                            "device_class":"door",
                            "friendly_name":"Front Door",
                            "private":"discard-me"
                        }
                    }
                }
            }
        }"#;
        let parsed = parse_state_changed_event(input, 1_000).unwrap();
        let state = parsed.new_state.unwrap();
        assert_eq!(state.entity_id, "binary_sensor.front_door");
        assert_eq!(state.state, "on");
        assert_eq!(state.device_class.as_deref(), Some("door"));
        assert_eq!(state.unit_of_measurement, None);
    }

    #[test]
    fn unsupported_event_type_is_rejected() {
        let input = br#"{
            "id":2,
            "type":"event",
            "event":{
                "event_type":"call_service",
                "data":{"entity_id":"sensor.room_temperature","new_state":null}
            }
        }"#;
        assert_eq!(
            parse_state_changed_event(input, 0),
            Err(WireError::UnexpectedMessageType)
        );
    }

    #[test]
    fn oversized_auth_message_is_rejected_before_json() {
        let input = vec![b'x'; MAX_AUTH_MESSAGE_BYTES + 1];
        assert_eq!(
            parse_auth_required(&input),
            Err(WireError::Oversized {
                limit: MAX_AUTH_MESSAGE_BYTES,
                actual: MAX_AUTH_MESSAGE_BYTES + 1,
            })
        );
    }

    #[test]
    fn result_failure_is_bounded_and_typed() {
        let input = br#"{
            "id":1,
            "type":"result",
            "success":false,
            "error":{"code":"unauthorized","message":"Not allowed"}
        }"#;
        let parsed = parse_states_result(input, 0).unwrap();
        assert!(!parsed.success);
        assert_eq!(parsed.states, Vec::<HomeAssistantStateFrame>::new());
        assert_eq!(
            parsed.error,
            Some(WireFailure {
                code: "unauthorized".into(),
                message: "Not allowed".into(),
            })
        );
    }

    #[test]
    fn control_characters_in_server_error_are_rejected() {
        let input = br#"{
            "id":2,
            "type":"result",
            "success":false,
            "error":{"code":"bad","message":"line1\nline2"}
        }"#;
        assert_eq!(
            parse_subscription_result(input),
            Err(WireError::InvalidText("error.message"))
        );
    }

    #[test]
    fn wrong_message_class_is_rejected() {
        let input = br#"{"type":"auth_required","ha_version":"2026.9.0"}"#;
        assert!(matches!(
            parse_subscription_result(input),
            Err(WireError::Json {
                category: JsonErrorCategory::Data,
                ..
            })
        ));
    }
}
