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
    decode_state, EntityObservationRule, HomeAssistantObservationProfile, HomeAssistantStateFrame,
    HA_STATE_UNAVAILABLE, HA_STATE_UNKNOWN, MAX_METADATA_BYTES, MAX_RAW_STATE_BYTES,
};
use crate::strict_json::{parse_bounded_json, StrictJsonError};
use crate::websocket_protocol::EntityStreamKind;
use hearth_automation_types::AutomationValue;
use serde_json::{Map, Value};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EntityStreamProjectionError {
    InvalidProfile(String),
    InvalidPayload(String),
    InvalidJson(StrictJsonError),
    DuplicateEntityAddition { entity_id: String },
    UnknownEntityChange { entity_id: String },
    InvalidEntityState { entity_id: String, reason: String },
}

impl fmt::Display for EntityStreamProjectionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidProfile(message) => write!(f, "invalid observation profile: {message}"),
            Self::InvalidPayload(message) => write!(f, "invalid compact entity-stream payload: {message}"),
            Self::InvalidJson(error) => write!(f, "invalid compact entity-stream JSON: {error}"),
            Self::DuplicateEntityAddition { entity_id } => write!(
                f,
                "added entity already exists in the active stream cache: {entity_id}"
            ),
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
enum SemanticState {
    Value(AutomationValue),
    Unknown,
    Unavailable,
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

    fn validate_profile_metadata(
        &self,
        entity_id: &str,
        rule: &EntityObservationRule,
    ) -> Result<(), EntityStreamProjectionError> {
        // A known-good string is not enough: expected metadata is part of the
        // semantic profile binding. Reject drift before this state can enter
        // the cache or refresh the observation's local freshness timestamp.
        if rule.expected_unit.as_deref().is_some_and(|expected| {
            self.unit_of_measurement.as_deref() != Some(expected)
        }) {
            return Err(entity_error(entity_id, "unit_of_measurement does not match the admitted profile"));
        }
        if rule.expected_device_class.as_deref().is_some_and(|expected| {
            self.device_class.as_deref() != Some(expected)
        }) {
            return Err(entity_error(entity_id, "device_class does not match the admitted profile"));
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

fn semantic_state(
    entity_id: &str,
    state: &CachedEntityState,
    rule: &EntityObservationRule,
) -> Result<SemanticState, EntityStreamProjectionError> {
    match state.state.as_str() {
        HA_STATE_UNKNOWN => Ok(SemanticState::Unknown),
        HA_STATE_UNAVAILABLE => Ok(SemanticState::Unavailable),
        _ => decode_state(&rule.codec, entity_id, &state.state)
            .map(SemanticState::Value)
            .map_err(|error| entity_error(entity_id, error.to_string())),
    }
}

pub struct EntityStreamProjector {
    allowlisted_entity_ids: BTreeSet<String>,
    rules: BTreeMap<String, EntityObservationRule>,
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
        let rules = profile.rules.iter()
            .cloned()
            .map(|rule| (rule.entity_id.clone(), rule))
            .collect();

        Ok(Self {
            allowlisted_entity_ids,
            rules,
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

    /// Decode and apply one raw compact event body. This public entry point
    /// rejects oversized frames and duplicate JSON keys before interpreting
    /// semantic fields. The caller must first validate epoch and subscription
    /// ID with the protocol state machine.
    pub fn apply_json(
        &mut self,
        kind: EntityStreamKind,
        json_bytes: &[u8],
        received_at_micros: i64,
    ) -> Result<EntityStreamProjection, EntityStreamProjectionError> {
        let event = parse_bounded_json(json_bytes)
            .map_err(EntityStreamProjectionError::InvalidJson)?;
        self.apply_value(kind, &event, received_at_micros)
    }

    /// Value-level core. Private so external transport callers cannot bypass
    /// the strict decoder and lose duplicate-key evidence before projection.
    /// All changes are staged and committed only on complete success.
    fn apply_value(
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
                let state = CachedEntityState::from_full(entity_id, compressed)?;
                let rule = self.rules.get(entity_id).expect("allowlist and rule map are built together");
                state.validate_profile_metadata(entity_id, rule)?;
                // Codec/range validation must happen before transactional cache
                // commit, not later when a caller maps the returned frame.
                semantic_state(entity_id, &state, rule)?;
                next.insert(entity_id.clone(), state);
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
                        "entity occurs in multiple update groups".into()
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

        // Stage only the touched entities rather than cloning the full cache.
        // Every payload/group is validated and every outgoing frame is built
        // before the staged write-set is committed, preserving all-or-nothing
        // semantics without O(total-cache-size) copying for a one-entity delta.
        let mut staged_states: BTreeMap<String, CachedEntityState> = BTreeMap::new();
        let mut changed_states: BTreeMap<String, CachedEntityState> = BTreeMap::new();
        let mut removed_ids = Vec::new();

        if let Some(added) = added {
            for (id, compressed) in added {
                if !self.allowlisted_entity_ids.contains(id) {
                    continue;
                }
                // Within one session, an "added" entry for an entity already in
                // the cache is either a duplicate/replayed event or an invalid
                // transition. Do not replace the cached state or refresh local
                // observation freshness; legitimate re-addition follows a
                // separate removal that evicted the prior cache entry.
                if self.state.contains_key(id) {
                    return Err(EntityStreamProjectionError::DuplicateEntityAddition {
                        entity_id: id.clone(),
                    });
                }
                let state = CachedEntityState::from_full(id, compressed)?;
                let rule = self.rules.get(id).expect("allowlisted entity must have a profile rule");
                state.validate_profile_metadata(id, rule)?;
                staged_states.insert(id.clone(), state.clone());
                changed_states.insert(id.clone(), state);
            }
        }

        if let Some(changed) = changed {
            for (id, diff) in changed {
                if !self.allowlisted_entity_ids.contains(id) {
                    continue;
                }
                let mut state = self.state.get(id).cloned().ok_or_else(|| {
                    EntityStreamProjectionError::UnknownEntityChange { entity_id: id.clone() }
                })?;
                let before = state.clone();
                state.apply_diff(id, diff)?;
                let rule = self.rules.get(id).expect("allowlisted entity must have a profile rule");
                state.validate_profile_metadata(id, rule)?;
                let before_semantic = semantic_state(id, &before, rule)?;
                let after_semantic = semantic_state(id, &state, rule)?;
                // Cache updates and semantic observation emission are separate
                // concerns. Commit valid representation metadata changes, but
                // do not mint a fresh observation when the decoded value (or
                // epistemic source status) is unchanged. This also prevents
                // alternate textual encodings such as 21.5 vs 21.50, or -0
                // vs 0, from extending freshness.
                if state != before {
                    if before_semantic != after_semantic {
                        changed_states.insert(id.clone(), state.clone());
                    }
                    staged_states.insert(id.clone(), state);
                }
            }
        }

        if let Some(removed) = removed {
            for raw_id in removed.iter() {
                let id = raw_id.as_str().expect("validated above");
                if self.allowlisted_entity_ids.contains(id) && self.state.contains_key(id) {
                    removed_ids.push(id.to_owned());
                }
            }
        }

        // Generate/validate every frame before mutating the cache. If projection
        // fails, neither prior state nor freshness can be partially advanced.
        let frames = changed_states.iter()
            .map(|(id, state)| state.frame(id, received_at_micros))
            .collect::<Result<Vec<_>, _>>()?;
        removed_ids.sort();

        for (id, state) in staged_states {
            self.state.insert(id, state);
        }
        for id in &removed_ids {
            self.state.remove(id);
        }
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
    if object.keys().any(|key| !allowed.contains(&key.as_str())) {
        return Err("unsupported field present".into());
    }
    Ok(())
}

fn validate_state(entity_id: &str, state: &str) -> Result<(), EntityStreamProjectionError> {
    if state.is_empty() || state.len() > MAX_RAW_STATE_BYTES {
        return Err(entity_error(entity_id, "state is empty or oversized"));
    }
    if state.chars().any(char::is_control) {
        return Err(entity_error(entity_id, "state contains control characters"));
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
        let out = projector.apply_value(EntityStreamKind::InitialSnapshot, &snapshot(), 10_000_000).unwrap();
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
                    "-": {"a": ["friendly_name"]}
                }
            }
        });
        let out = projector.apply(EntityStreamKind::Changes, &event, 20).unwrap();
        assert!(!out.initial_snapshot);
        assert_eq!(out.frames.len(), 1);
        assert_eq!(out.frames[0].state, "22.0");
        assert_eq!(out.frames[0].unit_of_measurement.as_deref(), Some("°C"));
        assert_eq!(out.frames[0].device_class.as_deref(), Some("temperature"));
        assert!(out.removed_entity_ids.is_empty());
    }

    #[test]
    fn nonsemantic_attribute_changes_do_not_refresh_observation_frames() {
        let mut projector = projector();
        projector.apply(EntityStreamKind::InitialSnapshot, &snapshot(), 10).unwrap();

        let event = serde_json::json!({
            "c": {
                "sensor.living_room_temperature": {
                    "+": {
                        "a": {"friendly_name": "Renamed without a new measurement"},
                        "lu": 1700000002
                    }
                }
            }
        });
        let out = projector.apply(EntityStreamKind::Changes, &event, 20).unwrap();
        assert!(out.frames.is_empty());
        assert!(out.removed_entity_ids.is_empty());
        assert_eq!(projector.state["sensor.living_room_temperature"].state, "21.5");
    }

    #[test]
    fn unasserted_unit_and_device_class_changes_update_cache_without_refreshing_observation() {
        let mut p = profile();
        p.rules[0].attribute = "status_label".into();
        p.rules[0].codec = StateCodec::Text {
            max_bytes: 64,
            allowed_values: BTreeSet::new(),
        };
        p.rules[0].expected_unit = None;
        p.rules[0].expected_device_class = None;
        let mut projector = EntityStreamProjector::new(&p).unwrap();
        projector.apply(EntityStreamKind::InitialSnapshot, &snapshot(), 10).unwrap();

        let metadata_only = json!({
            "c": {
                "sensor.living_room_temperature": {
                    "+": {
                        "a": {
                            "unit_of_measurement": "°F",
                            "device_class": "humidity"
                        },
                        "lu": 1700000123
                    }
                }
            }
        });
        let out = projector.apply(EntityStreamKind::Changes, &metadata_only, 999_999).unwrap();

        assert!(out.frames.is_empty(), "unasserted metadata is not a new semantic observation");
        assert!(out.removed_entity_ids.is_empty());
        let cached = &projector.state["sensor.living_room_temperature"];
        assert_eq!(cached.state, "21.5");
        assert_eq!(cached.unit_of_measurement.as_deref(), Some("°F"));
        assert_eq!(cached.device_class.as_deref(), Some("humidity"));

        // A subsequent semantic state change emits a frame using the latest
        // cached metadata, but its value remains the allowlisted primary state.
        let changed = projector.apply(
            EntityStreamKind::Changes,
            &json!({"c": {"sensor.living_room_temperature": {"+": {"s": "22.0"}}}}),
            1_000_000,
        ).unwrap();
        assert_eq!(changed.frames.len(), 1);
        assert_eq!(changed.frames[0].state, "22.0");
        assert_eq!(changed.frames[0].unit_of_measurement.as_deref(), Some("°F"));
        assert_eq!(changed.frames[0].device_class.as_deref(), Some("humidity"));
    }

    #[test]
    fn codec_out_of_range_initial_snapshot_never_seeds_cache() {
        let mut projector = projector();
        let mut invalid = snapshot();
        invalid["a"]["sensor.living_room_temperature"]["s"] = json!("99.9");

        assert!(matches!(
            projector.apply(EntityStreamKind::InitialSnapshot, &invalid, 10),
            Err(EntityStreamProjectionError::InvalidEntityState { .. })
        ));
        assert_eq!(projector.cached_entity_count(), 0);
        assert!(!projector.initial_snapshot_seen());
    }

    #[test]
    fn codec_out_of_range_late_entity_rolls_back_earlier_valid_delta() {
        let mut base = profile();
        let mut first = base.rules[0].clone();
        first.entity_id = "sensor.a_temperature".into();
        first.subject.id = "room-a".into();
        let mut last = base.rules[0].clone();
        last.entity_id = "sensor.z_temperature".into();
        last.subject.id = "room-z".into();
        base.rules = vec![first, last];

        let mut projector = EntityStreamProjector::new(&base).unwrap();
        let initial = json!({
            "a": {
                "sensor.a_temperature": {
                    "s": "20.0",
                    "a": {"unit_of_measurement": "°C", "device_class": "temperature"}
                },
                "sensor.z_temperature": {
                    "s": "21.0",
                    "a": {"unit_of_measurement": "°C", "device_class": "temperature"}
                }
            }
        });
        projector.apply(EntityStreamKind::InitialSnapshot, &initial, 10).unwrap();

        let invalid_late_delta = json!({
            "c": {
                "sensor.a_temperature": {"+": {"s": "22.0"}},
                "sensor.z_temperature": {"+": {"s": "99.9"}}
            }
        });
        assert!(matches!(
            projector.apply(EntityStreamKind::Changes, &invalid_late_delta, 20),
            Err(EntityStreamProjectionError::InvalidEntityState { .. })
        ));
        assert_eq!(projector.state["sensor.a_temperature"].state, "20.0");
        assert_eq!(projector.state["sensor.z_temperature"].state, "21.0");
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
    fn control_characters_in_projected_state_fail_without_cache_mutation() {
        let mut projector = projector();
        let event = json!({
            "a": {
                "sensor.living_room_temperature": {
                    "s": "21.5\nforged-line",
                    "a": {}
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
    fn profile_metadata_drift_rejects_initial_snapshot_without_seeding_cache() {
        let mut projector = projector();
        let mut event = snapshot();
        event["a"]["sensor.living_room_temperature"]["a"]["device_class"] =
            json!("humidity");

        assert!(matches!(
            projector.apply(EntityStreamKind::InitialSnapshot, &event, 10),
            Err(EntityStreamProjectionError::InvalidEntityState { .. })
        ));
        assert_eq!(projector.cached_entity_count(), 0);
        assert!(!projector.initial_snapshot_seen());
    }

    #[test]
    fn profile_metadata_drift_rejects_delta_without_refreshing_cache() {
        let mut projector = projector();
        projector.apply(EntityStreamKind::InitialSnapshot, &snapshot(), 10).unwrap();

        let event = json!({
            "c": {
                "sensor.living_room_temperature": {
                    "+": {"s": "22.0", "a": {"unit_of_measurement": "°F"}}
                }
            }
        });
        assert!(matches!(
            projector.apply(EntityStreamKind::Changes, &event, 20),
            Err(EntityStreamProjectionError::InvalidEntityState { .. })
        ));
        assert_eq!(projector.state["sensor.living_room_temperature"].state, "21.5");
        assert_eq!(
            projector.state["sensor.living_room_temperature"].unit_of_measurement.as_deref(),
            Some("°C")
        );
    }

    #[test]
    fn metadata_drift_in_later_entity_rolls_back_earlier_valid_delta() {
        let mut base = profile();
        let mut first = base.rules[0].clone();
        first.entity_id = "sensor.a_temperature".into();
        first.subject.id = "room-a".into();
        let mut last = base.rules[0].clone();
        last.entity_id = "sensor.z_temperature".into();
        last.subject.id = "room-z".into();
        base.rules = vec![first, last];

        let mut projector = EntityStreamProjector::new(&base).unwrap();
        let initial = json!({
            "a": {
                "sensor.a_temperature": {
                    "s": "20.0",
                    "a": {"unit_of_measurement": "°C", "device_class": "temperature"}
                },
                "sensor.z_temperature": {
                    "s": "21.0",
                    "a": {"unit_of_measurement": "°C", "device_class": "temperature"}
                }
            }
        });
        projector.apply(EntityStreamKind::InitialSnapshot, &initial, 10).unwrap();

        // Sorted map order visits a before z. The late metadata mismatch must
        // reject the whole delta, not commit a's valid state/freshness update.
        let update = json!({
            "c": {
                "sensor.a_temperature": {"+": {"s": "22.0"}},
                "sensor.z_temperature": {"+": {"a": {"unit_of_measurement": "°F"}}}
            }
        });
        assert!(matches!(
            projector.apply(EntityStreamKind::Changes, &update, 20),
            Err(EntityStreamProjectionError::InvalidEntityState { .. })
        ));
        assert_eq!(projector.state["sensor.a_temperature"].state, "20.0");
        assert_eq!(projector.state["sensor.z_temperature"].state, "21.0");
        assert_eq!(
            projector.state["sensor.z_temperature"].unit_of_measurement.as_deref(),
            Some("°C")
        );
    }

    #[test]
    fn multi_entity_failure_does_not_commit_earlier_valid_delta() {
        let mut base = profile();
        let mut first = base.rules[0].clone();
        first.entity_id = "sensor.a_temperature".into();
        first.subject.id = "room-a".into();
        let mut last = base.rules[0].clone();
        last.entity_id = "sensor.z_temperature".into();
        last.subject.id = "room-z".into();
        base.rules = vec![first, last];
        let mut projector = EntityStreamProjector::new(&base).unwrap();
        let snapshot = json!({
            "a": {
                "sensor.a_temperature": {
                    "s": "20.0",
                    "a": {"unit_of_measurement": "°C", "device_class": "temperature"}
                },
                "sensor.z_temperature": {
                    "s": "21.0",
                    "a": {"unit_of_measurement": "°C", "device_class": "temperature"}
                }
            }
        });
        projector.apply(EntityStreamKind::InitialSnapshot, &snapshot, 10).unwrap();

        // The valid 'a' change sorts before the malformed 'z' change. If the
        // projector mutates eagerly, the first entity would incorrectly stay at 22.0.
        let bad_update = json!({
            "c": {
                "sensor.a_temperature": {"+": {"s": "22.0"}},
                "sensor.z_temperature": {"+": {"a": ["not-an-object"]}}
            }
        });
        assert!(matches!(
            projector.apply(EntityStreamKind::Changes, &bad_update, 20),
            Err(EntityStreamProjectionError::InvalidEntityState { .. })
        ));
        assert_eq!(projector.state["sensor.a_temperature"].state, "20.0");
        assert_eq!(projector.state["sensor.z_temperature"].state, "21.0");
        assert_eq!(projector.cached_entity_count(), 2);
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
    fn duplicate_added_entity_is_rejected_without_cache_or_freshness_change() {
        let mut projector = projector();
        let initial = projector.apply(EntityStreamKind::InitialSnapshot, &snapshot(), 10).unwrap();
        assert_eq!(initial.frames.len(), 1);
        let prior = projector.state["sensor.living_room_temperature"].clone();

        let replay = serde_json::json!({
            "a": {
                "sensor.living_room_temperature": {
                    "s": "99.9",
                    "a": {
                        "unit_of_measurement": "°C",
                        "device_class": "temperature",
                        "friendly_name": "replayed payload"
                    }
                }
            }
        });
        assert!(matches!(
            projector.apply(EntityStreamKind::Changes, &replay, 999_999),
            Err(EntityStreamProjectionError::DuplicateEntityAddition { ref entity_id })
                if entity_id == "sensor.living_room_temperature"
        ));
        assert_eq!(projector.state["sensor.living_room_temperature"], prior);
        assert_eq!(projector.cached_entity_count(), 1);
    }

    #[test]
    fn replayed_identical_state_delta_does_not_refresh_freshness() {
        let mut projector = projector();
        projector.apply(EntityStreamKind::InitialSnapshot, &snapshot(), 10).unwrap();

        let delta = serde_json::json!({
            "c": {
                "sensor.living_room_temperature": {
                    "+": {"s": "22.0"}
                }
            }
        });
        let first = projector.apply(EntityStreamKind::Changes, &delta, 20).unwrap();
        assert_eq!(first.frames.len(), 1);
        assert_eq!(first.frames[0].state, "22.0");
        assert_eq!(first.frames[0].received_at_micros, 20);

        let replay = projector.apply(EntityStreamKind::Changes, &delta, 99_999).unwrap();
        assert!(replay.frames.is_empty(), "replaying the same semantic state must not mint a newer observation");
        assert!(replay.removed_entity_ids.is_empty());
        assert_eq!(projector.state["sensor.living_room_temperature"].state, "22.0");
    }

    #[test]
    fn codec_equivalent_numeric_text_does_not_refresh_freshness() {
        let mut p = profile();
        p.rules[0].codec = StateCodec::FixedRange {
            scale: 2,
            min_mantissa: -5_000,
            max_mantissa: 8_000,
        };
        let mut projector = EntityStreamProjector::new(&p).unwrap();
        projector.apply(EntityStreamKind::InitialSnapshot, &snapshot(), 10).unwrap();

        // Both lexical values decode to the same exact fixed-point mantissa.
        let equivalent = json!({
            "c": {
                "sensor.living_room_temperature": {
                    "+": {"s": "21.50"}
                }
            }
        });
        let out = projector.apply(EntityStreamKind::Changes, &equivalent, 999_999).unwrap();
        assert!(out.frames.is_empty(), "equivalent typed values must not mint newer observations");
        assert_eq!(projector.state["sensor.living_room_temperature"].state, "21.50");
    }

    #[test]
    fn explicit_removal_allows_a_later_valid_readdition() {
        let mut projector = projector();
        projector.apply(EntityStreamKind::InitialSnapshot, &snapshot(), 10).unwrap();

        let removed = projector.apply(
            EntityStreamKind::Changes,
            &serde_json::json!({"r": ["sensor.living_room_temperature"]}),
            20,
        ).unwrap();
        assert_eq!(removed.removed_entity_ids, vec!["sensor.living_room_temperature"]);
        assert_eq!(projector.cached_entity_count(), 0);

        let added = projector.apply(
            EntityStreamKind::Changes,
            &serde_json::json!({
                "a": {
                    "sensor.living_room_temperature": {
                        "s": "22.0",
                        "a": {
                            "unit_of_measurement": "°C",
                            "device_class": "temperature"
                        }
                    }
                }
            }),
            30,
        ).unwrap();
        assert_eq!(added.frames.len(), 1);
        assert_eq!(added.frames[0].state, "22.0");
        assert_eq!(added.frames[0].received_at_micros, 30);
        assert_eq!(projector.cached_entity_count(), 1);
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
