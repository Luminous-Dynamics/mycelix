// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Pure projector for Home Assistant's compact subscribe_entities stream.
//!
//! Only profile-allowlisted entity state plus the two HA-0.1 metadata assertions
//! are retained. Arbitrary attributes, context identifiers, names, and other
//! fields are not copied into the projection. Transport code remains responsible
//! for frame limits, authentication, epoch/subscription checks and JSON decoding.

use crate::{
    EntityObservationRule, HomeAssistantObservationProfile, HomeAssistantStateFrame,
    MAX_METADATA_BYTES, MAX_RAW_STATE_BYTES,
};
use crate::websocket_protocol::EntityStreamKind;
use serde_json::{Map, Value};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EntityStreamProjectionError {
    InvalidProfile(String),
    InvalidPayload(String),
    UnknownEntityChange { entity_id: String },
    InvalidEntityState { entity_id: String, reason: String },
}

impl fmt::Display for EntityStreamProjectionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidProfile(message) => write!(f, "invalid observation profile: {message}"),
            Self::InvalidPayload(message) => write!(f, "invalid compact entity-stream payload: {message}"),
            Self::UnknownEntityChange { entity_id } => write!(
                f,
                "change received for allowlisted entity absent from current snapshot: {entity_id}"
            ),
            Self::InvalidEntityState { entity_id, reason } => {
                write!(f, "invalid compact state for {entity_id}: {reason}")
            }
        }
    }
}

impl std::error::Error for EntityStreamProjectionError {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EntityStreamProjection {
    pub initial_snapshot: bool,
    pub frames: Vec<HomeAssistantStateFrame>,
    /// Removal invalidates prior evidence. It is not evidence of a physical state.
    pub removed_entity_ids: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct CachedEntityState {
    state: String,
    unit_of_measurement: Option<String>,
    device_class: Option<String>,
}

impl CachedEntityState {
    fn from_full(entity_id: &str, value: &Value) -> Result<Self, EntityStreamProjectionError> {
        let object = value.as_object().ok_or_else(|| entity_error(entity_id, "full state must be an object"))?;
        reject_unknown_keys(object, &["s", "a", "c", "lc", "lu"])
            .map_err(|reason| entity_error(entity_id, reason))?;

        let state = object.get("s").and_then(Value::as_str)
            .ok_or_else(|| entity_error(entity_id, "compressed state requires string field s"))?;
        validate_state(entity_id, state)?;
        let attributes = object.get("a").and_then(Value::as_object)
            .ok_or_else(|| entity_error(entity_id, "full state requires an a attribute object"))?;
        validate_ignored_compressed_fields(entity_id, object)?;

        Ok(Self {
            state: state.to_owned(),
            unit_of_measurement: optional_string_attribute(entity_id, attributes, "unit_of_measurement")?,
            device_class: optional_string_attribute(entity_id, attributes, "device_class")?,
        })
    }

    fn apply_diff(&mut self, entity_id: &str, value: &Value) -> Result<(), EntityStreamProjectionError> {
        let object = value.as_object().ok_or_else(|| entity_error(entity_id, "entity diff must be an object"))?;
        reject_unknown_keys(object, &["+", "-"]).map_err(|reason| entity_error(entity_id, reason))?;

        if let Some(plus_value) = object.get("+") {
            let plus = plus_value.as_object().ok_or_else(|| entity_error(entity_id, "+ diff must be an object"))?;
            reject_unknown_keys(plus, &["s", "a", "c", "lc", "lu"])
                .map_err(|reason| entity_error(entity_id, reason))?;
            validate_ignored_compressed_fields(entity_id, plus)?;

            if let Some(value) = plus.get("s") {
                let state = value.as_str().ok_or_else(|| entity_error(entity_id, "+.s must be a string"))?;
                validate_state(entity_id, state)?;
                self.state = state.to_owned();
            }

            if let Some(value) = plus.get("a") {
                let attributes = value.as_object().ok_or_else(|| entity_error(entity_id, "+.a must be an object"))?;
                if attributes.contains_key("unit_of_measurement") {
                    self.unit_of_measurement = optional_string_attribute(entity_id, attributes, "unit_of_measurement")?;
                }
                if attributes.contains_key("device_class") {
                    self.device_class = optional_string_attribute(entity_id, attributes, "device_class")?;
                }
            }
        }

        if let Some(minus_value) = object.get("-") {
            let minus = minus_value.as_object().ok_or_else(|| entity_error(entity_id, "- diff must be an object"))?;
            reject_unknown_keys(minus, &["a"]).map_err(|reason| entity_error(entity_id, reason))?;
            if let Some(value) = minus.get("a") {
                let removed = value.as_array().ok_or_else(|| entity_error(entity_id, "-.a must be an array"))?;
                for key in removed {
                    match key.as_str().ok_or_else(|| entity_error(entity_id, "-.a values must be strings"))? {
                        "unit_of_measurement" => self.unit_of_measurement = None,
                        "device_class" => self.device_class = None,
                        _ => {}
                    }
                }
            }
        }
        Ok(())
    }

    fn frame(&self, entity_id: &str, received_at_micros: i64) -> Result<HomeAssistantStateFrame, EntityStreamProjectionError> {
        let frame = HomeAssistantStateFrame {
            entity_id: entity_id.to_owned(),
            state: self.state.clone(),
            unit_of_measurement: self.unit_of_measurement.clone(),
            device_class: self.device_class.clone(),
            source_last_updated_micros: None,
            received_at_micros,
        };
        frame.validate().map_err(|error| entity_error(entity_id, error.to_string()))?;
        Ok(frame)
    }
}

pub struct EntityStreamProjector {
    allowlisted_entity_ids: BTreeSet<String>,
    state: BTreeMap<String, CachedEntityState>,
    initial_snapshot_seen: bool,
}

impl EntityStreamProjector {
    pub fn new(profile: &HomeAssistantObservationProfile) -> Result<Self, EntityStreamProjectionError> {
        profile.validate().map_err(|error| EntityStreamProjectionError::InvalidProfile(error.to_string()))?;
        let allowlisted_entity_ids: BTreeSet<String> = profile.rules.iter()
            .map(|rule: &EntityObservationRule| rule.entity_id.clone())
            .collect();
        if allowlisted_entity_ids.is_empty() || allowlisted_entity_ids.len() != profile.rules.len() {
            return Err(EntityStreamProjectionError::InvalidProfile(
                "entity allowlist must be non-empty and unique".into()
            ));
        }
        Ok(Self {
            allowlisted_entity_ids,
            state: BTreeMap::new(),
            initial_snapshot_seen: false,
        })
    }

    pub fn initial_snapshot_seen(&self) -> bool {
        self.initial_snapshot_seen
    }

    pub fn cached_entity_count(&self) -> usize {
        self.state.len()
    }

    /// Apply one event body from the active subscription. The caller must first
    /// validate epoch and subscription ID with the protocol state machine.
    /// All changes are staged on a clone and committed only on complete success.
    pub fn apply(
        &mut self,
        kind: EntityStreamKind,
        event: &Value,
        received_at_micros: i64,
    ) -> Result<EntityStreamProjection, EntityStreamProjectionError> {
        let object = event.as_object()
            .ok_or_else(|| EntityStreamProjectionError::InvalidPayload("event body must be an object".into()))?;

        if kind == EntityStreamKind::InitialSnapshot {
            if self.initial_snapshot_seen {
                return Err(EntityStreamProjectionError::InvalidPayload("duplicate initial snapshot".into()));
            }
            if object.len() != 1 || !object.contains_key("a") {
                return Err(EntityStreamProjectionError::InvalidPayload(
                    "initial snapshot must contain only an a full-state map".into()
                ));
            }
            let added = object.get("a").and_then(Value::as_object)
                .ok_or_else(|| EntityStreamProjectionError::InvalidPayload("initial snapshot a field must be an object".into()))?;

            let mut next = BTreeMap::new();
            for (entity_id, compressed) in added {
                if !self.allowlisted_entity_ids.contains(entity_id) {
                    continue;
                }
                next.insert(entity_id.clone(), CachedEntityState::from_full(entity_id, compressed)?);
            }
            let frames = next.iter()
                .map(|(id, state)| state.frame(id, received_at_micros))
                .collect::<Result<Vec<_>, _>>()?;
            self.state = next;
            self.initial_snapshot_seen = true;
            return Ok(EntityStreamProjection {
                initial_snapshot: true,
                frames,
                removed_entity_ids: Vec::new(),
            });
        }

        if !self.initial_snapshot_seen {
            return Err(EntityStreamProjectionError::InvalidPayload(
                "incremental update arrived before initial snapshot".into()
            ));
        }
        reject_unknown_keys(object, &["a", "c", "r"])
            .map_err(EntityStreamProjectionError::InvalidPayload)?;

        let added = parse_optional_object(object, "a")?;
        let changed = parse_optional_object(object, "c")?;
        let removed = match object.get("r") {
            Some(value) => Some(value.as_array().ok_or_else(|| EntityStreamProjectionError::InvalidPayload(
                "r field must be an array of entity IDs".into()
            ))?),
            None => None,
        };

        let mut seen_ids = BTreeSet::new();
        if let Some(added) = added {
            for id in added.keys() {
                if !seen_ids.insert(id.as_str()) {
                    return Err(EntityStreamProjectionError::InvalidPayload(
                        format!("entity occurs in multiple update groups: {id}")
                    ));
                }
            }
        }
        if let Some(changed) = changed {
            for id in changed.keys() {
                if !seen_ids.insert(id.as_str()) {
                    return Err(EntityStreamProjectionError::InvalidPayload(
                        format!("entity occurs in multiple update groups: {id}")
                    ));
                }
            }
        }
        if let Some(removed) = removed {
            for raw_id in removed.iter() {
                let id = raw_id.as_str().ok_or_else(|| EntityStreamProjectionError::InvalidPayload(
                    "r entries must be entity ID strings".into()
                ))?;
                if !seen_ids.insert(id) {
                    return Err(EntityStreamProjectionError::InvalidPayload(
                        format!("entity occurs in multiple update groups: {id}")
                    ));
                }
            }
        }

        let mut next = self.state.clone();
        let mut changed_states: BTreeMap<String, CachedEntityState> = BTreeMap::new();
        let mut removed_ids = Vec::new();

        if let Some(added) = added {
            for (id, compressed) in added {
                if !self.allowlisted_entity_ids.contains(id) {
                    continue;
                }
                let state = CachedEntityState::from_full(id, compressed)?;
                next.insert(id.clone(), state.clone());
                changed_states.insert(id.clone(), state);
            }
        }

        if let Some(changed) = changed {
            for (id, diff) in changed {
                if !self.allowlisted_entity_ids.contains(id) {
                    continue;
                }
                let state = next.get_mut(id).ok_or_else(|| {
                    EntityStreamProjectionError::UnknownEntityChange { entity_id: id.clone() }
                })?;
                state.apply_diff(id, diff)?;
                changed_states.insert(id.clone(), state.clone());
            }
        }

        if let Some(removed) = removed {
            for raw_id in removed.iter() {
                let id = raw_id.as_str().expect("validated above");
                if self.allowlisted_entity_ids.contains(id) && next.remove(id).is_some() {
                    removed_ids.push(id.to_owned());
                }
            }
        }

        let frames = changed_states.iter()
            .map(|(id, state)| state.frame(id, received_at_micros))
            .collect::<Result<Vec<_>, _>>()?;
        removed_ids.sort();

        self.state = next;
        Ok(EntityStreamProjection {
            initial_snapshot: false,
            frames,
            removed_entity_ids: removed_ids,
        })
    }
}

fn entity_error(entity_id: &str, reason: impl Into<String>) -> EntityStreamProjectionError {
    EntityStreamProjectionError::InvalidEntityState {
        entity_id: entity_id.to_owned(),
        reason: reason.into(),
    }
}

fn parse_optional_object<'a>(
    event: &'a Map<String, Value>,
    key: &str,
) -> Result<Option<&'a Map<String, Value>>, EntityStreamProjectionError> {
    match event.get(key) {
        Some(value) => value.as_object().map(Some).ok_or_else(|| EntityStreamProjectionError::InvalidPayload(
            format!("{key} field must be an object")
        )),
        None => Ok(None),
    }
}

fn reject_unknown_keys(object: &Map<String, Value>, allowed: &[&str]) -> Result<(), String> {
    if let Some(key) = object.keys().find(|key| !allowed.contains(&key.as_str())) {
        return Err(format!("unsupported field: {key}"));
    }
    Ok(())
}

fn validate_state(entity_id: &str, state: &str) -> Result<(), EntityStreamProjectionError> {
    if state.is_empty() || state.len() > MAX_RAW_STATE_BYTES || state.contains('\0') {
        return Err(entity_error(entity_id, "state is empty, oversized, or contains NUL"));
    }
    Ok(())
}

fn validate_ignored_compressed_fields(
    entity_id: &str,
    object: &Map<String, Value>,
) -> Result<(), EntityStreamProjectionError> {
    if let Some(context) = object.get("c") {
        if !(context.is_string() || context.is_object()) {
            return Err(entity_error(entity_id, "compressed context must be a string or object"));
        }
    }
    for key in ["lc", "lu"] {
        if let Some(value) = object.get(key) {
            if !value.is_number() {
                return Err(entity_error(entity_id, format!("compressed {key} must be numeric")));
            }
        }
    }
    Ok(())
}

fn optional_string_attribute(
    entity_id: &str,
    attributes: &Map<String, Value>,
    key: &str,
) -> Result<Option<String>, EntityStreamProjectionError> {
    match attributes.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(value)) => {
            if value.is_empty()
                || value.len() > MAX_METADATA_BYTES
                || value.chars().any(char::is_control)
                || value.trim() != value
            {
                return Err(entity_error(entity_id, format!("{key} metadata is empty, oversized, or malformed")));
            }
            Ok(Some(value.clone()))
        }
        Some(_) => Err(entity_error(entity_id, format!("{key} metadata must be a string or null"))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        EntityObservationRule, EntityRef, StateCodec, HOME_ASSISTANT_OBSERVATION_SCHEMA_VERSION,
    };
    use serde_json::json;

    fn profile() -> HomeAssistantObservationProfile {
        HomeAssistantObservationProfile {
            schema_version: HOME_ASSISTANT_OBSERVATION_SCHEMA_VERSION,
            instance_id: "home-main".into(),
            profile_ref: "profile:stream-projector-v1".into(),
            rules: vec![EntityObservationRule {
                entity_id: "sensor.living_room_temperature".into(),
                subject: EntityRef {
                    kind: "room".into(),
                    id: "living-room".into(),
                },
                attribute: "temperature_celsius".into(),
                codec: StateCodec::FixedRange {
                    scale: 1,
                    min_mantissa: -500,
                    max_mantissa: 800,
                },
                expected_unit: Some("°C".into()),
                expected_device_class: Some("temperature".into()),
                max_age_ms: 30_000,
                confidence_bp: 9_500,
            }],
        }
    }

    fn projector() -> EntityStreamProjector {
        EntityStreamProjector::new(&profile()).expect("profile must validate")
    }

    fn snapshot() -> Value {
        serde_json::json!({
            "a": {
                "sensor.living_room_temperature": {
                    "s": "21.5",
                    "a": {
                        "unit_of_measurement": "°C",
                        "device_class": "temperature",
                        "friendly_name": "Living Room Temperature",
                        "private_marker": "must-not-cross-the-projection"
                    },
                    "c": "context-id-not-retained",
                    "lc": 1700000000,
                    "lu": 1700000001
                },
                "sensor.not_allowlisted": {
                    "s": "secret-state",
                    "a": {"private_marker": "ignore"}
                }
            }
        })
    }

    #[test]
    fn initial_snapshot_projects_only_profile_allowlisted_fields() {
        let mut projector = projector();
        let out = projector.apply(EntityStreamKind::InitialSnapshot, &snapshot(), 10_000_000).unwrap();
        assert!(out.initial_snapshot);
        assert_eq!(out.frames.len(), 1);
        assert_eq!(out.frames[0].entity_id, "sensor.living_room_temperature");
        assert_eq!(out.frames[0].state, "21.5");
        assert_eq!(out.frames[0].unit_of_measurement.as_deref(), Some("°C"));
        assert_eq!(out.frames[0].device_class.as_deref(), Some("temperature"));
        assert_eq!(out.frames[0].received_at_micros, 10_000_000);
        assert_eq!(out.frames[0].source_last_updated_micros, None);
        assert!(out.removed_entity_ids.is_empty());
        assert_eq!(projector.cached_entity_count(), 1);
    }

    #[test]
    fn compact_diff_merges_state_and_removes_selected_metadata_only() {
        let mut projector = projector();
        projector.apply(EntityStreamKind::InitialSnapshot, &snapshot(), 10).unwrap();
        let event = serde_json::json!({
            "c": {
                "sensor.living_room_temperature": {
                    "+": {
                        "s": "22.0",
                        "a": {"unit_of_measurement": "°C"},
                        "lu": 1700000002
                    },
                    "-": {"a": ["device_class", "friendly_name"]}
                }
            }
        });
        let out = projector.apply(EntityStreamKind::Changes, &event, 20).unwrap();
        assert!(!out.initial_snapshot);
        assert_eq!(out.frames.len(), 1);
        assert_eq!(out.frames[0].state, "22.0");
        assert_eq!(out.frames[0].unit_of_measurement.as_deref(), Some("°C"));
        assert_eq!(out.frames[0].device_class, None);
        assert!(out.removed_entity_ids.is_empty());
    }

    #[test]
    fn unallowlisted_entities_never_enter_the_projection() {
        let mut projector = projector();
        projector.apply(EntityStreamKind::InitialSnapshot, &snapshot(), 10).unwrap();
        let event = serde_json::json!({
            "a": {"sensor.not_allowlisted": {"s": "private", "a": {}}},
            "c": {},
            "r": []
        });
        let out = projector.apply(EntityStreamKind::Changes, &event, 20).unwrap();
        assert!(out.frames.is_empty());
        assert!(out.removed_entity_ids.is_empty());
        assert_eq!(projector.cached_entity_count(), 1);
    }

    #[test]
    fn incremental_change_before_snapshot_fails_closed() {
        let mut projector = projector();
        assert!(matches!(
            projector.apply(EntityStreamKind::Changes, &serde_json::json!({"c": {}}), 1),
            Err(EntityStreamProjectionError::InvalidPayload(_))
        ));
    }

    #[test]
    fn allowlisted_change_without_cached_snapshot_fails_closed() {
        let mut projector = projector();
        projector.apply(EntityStreamKind::InitialSnapshot, &serde_json::json!({"a": {}}), 10).unwrap();
        let event = serde_json::json!({
            "c": {"sensor.living_room_temperature": {"+": {"s": "22.0"}}}
        });
        assert!(matches!(
            projector.apply(EntityStreamKind::Changes, &event, 20),
            Err(EntityStreamProjectionError::UnknownEntityChange { .. })
        ));
        assert_eq!(projector.cached_entity_count(), 0);
    }

    #[test]
    fn malformed_changes_are_transactional() {
        let mut projector = projector();
        projector.apply(EntityStreamKind::InitialSnapshot, &snapshot(), 10).unwrap();
        let event = serde_json::json!({
            "c": {
                "sensor.living_room_temperature": {
                    "+": {"s": "22.0", "a": ["not-an-object"]}
                }
            }
        });
        assert!(projector.apply(EntityStreamKind::Changes, &event, 20).is_err());
        let out = projector.apply(EntityStreamKind::Changes, &serde_json::json!({"c": {}, "r": []}), 30).unwrap();
        assert!(out.frames.is_empty());
        assert_eq!(projector.state["sensor.living_room_temperature"].state, "21.5");
    }

    #[test]
    fn removal_invalidates_prior_state_without_claiming_physical_outcome() {
        let mut projector = projector();
        projector.apply(EntityStreamKind::InitialSnapshot, &snapshot(), 10).unwrap();
        let out = projector.apply(
            EntityStreamKind::Changes,
            &serde_json::json!({"r": ["sensor.living_room_temperature", "sensor.not_allowlisted"]}),
            20,
        ).unwrap();
        assert!(out.frames.is_empty());
        assert_eq!(out.removed_entity_ids, vec!["sensor.living_room_temperature".to_string()]);
        assert_eq!(projector.cached_entity_count(), 0);
    }

    #[test]
    fn duplicate_initial_snapshot_is_rejected() {
        let mut projector = projector();
        projector.apply(EntityStreamKind::InitialSnapshot, &snapshot(), 10).unwrap();
        assert!(matches!(
            projector.apply(EntityStreamKind::InitialSnapshot, &snapshot(), 20),
            Err(EntityStreamProjectionError::InvalidPayload(_))
        ));
    }

    #[test]
    fn duplicate_entity_across_update_groups_is_rejected_without_mutation() {
        let mut projector = projector();
        projector.apply(EntityStreamKind::InitialSnapshot, &snapshot(), 10).unwrap();
        let event = serde_json::json!({
            "a": {"sensor.living_room_temperature": {"s": "new", "a": {}}},
            "c": {"sensor.living_room_temperature": {"+": {"s": "different"}}}
        });
        assert!(matches!(
            projector.apply(EntityStreamKind::Changes, &event, 20),
            Err(EntityStreamProjectionError::InvalidPayload(_))
        ));
        assert_eq!(projector.state["sensor.living_room_temperature"].state, "21.5");
    }

    #[test]
    fn malformed_projected_metadata_fails_closed() {
        let mut projector = projector();
        let event = serde_json::json!({
            "a": {
                "sensor.living_room_temperature": {
                    "s": "21.5",
                    "a": {"unit_of_measurement": {"unexpected": "object"}}
                }
            }
        });
        assert!(matches!(
            projector.apply(EntityStreamKind::InitialSnapshot, &event, 10),
            Err(EntityStreamProjectionError::InvalidEntityState { .. })
        ));
        assert_eq!(projector.cached_entity_count(), 0);
        assert!(!projector.initial_snapshot_seen());
    }
}
