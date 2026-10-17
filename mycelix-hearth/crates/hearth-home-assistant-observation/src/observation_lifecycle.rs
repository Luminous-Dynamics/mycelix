// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Explicit lifecycle updates for normalized Home Assistant observations.
//!
//! If the provider reports unknown/unavailable, a mapped sample violates the
//! admitted codec/profile, an entity disappears, or a reconnect snapshot omits
//! an entity, downstream state must invalidate its previous positive
//! observation rather than leaving that value current until an unrelated
//! timeout. This module does not grant household authority or physical dispatch.

use crate::{
    HomeAssistantObservation, HomeAssistantObservationProfile, HomeAssistantSourceStatus,
    MappingOutcome, ObservationError, map_state,
};
use crate::entity_stream::{EntityStreamProjection, EntityStreamProjectionError};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ObservationInvalidationReason {
    SourceUnknown,
    SourceUnavailable,
    EntityRemoved,
    MissingFromInitialSnapshot,
    InvalidCurrentSample,
    /// A current provider update could not be projected transactionally; all
    /// profile observations are invalidated until a new valid snapshot arrives.
    ProjectionRejected,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ObservationUpdate {
    Upsert(HomeAssistantObservation),
    Invalidate {
        entity_id: String,
        reason: ObservationInvalidationReason,
        at_micros: i64,
    },
}

impl ObservationUpdate {
    fn entity_id(&self) -> &str {
        match self {
            Self::Upsert(value) => &value.entity_id,
            Self::Invalidate { entity_id, .. } => entity_id,
        }
    }
}

/// Total ordering token supplied by the local transport owner. `generation`
/// must increase durably across reconnects and process restarts; `sequence`
/// starts at zero for the generation's initial snapshot and increments for
/// every accepted event. This type validates shape, but does not allocate or
/// persist either counter.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct ObservationVersion {
    pub generation: u64,
    pub sequence: u64,
}

impl ObservationVersion {
    /// Accept only a strictly newer version for an entity. Equality means replay,
    /// not a new observation. Call this against the version persisted for each
    /// entity before applying an update from a batch.
    pub fn is_newer_than(self, previous: Option<Self>) -> bool {
        previous.is_none_or(|previous| self > previous)
    }

    fn validate_for(self, initial_snapshot: bool) -> Result<(), ObservationLifecycleError> {
        if self.generation == 0
            || (initial_snapshot && self.sequence != 0)
            || (!initial_snapshot && self.sequence == 0)
        {
            return Err(ObservationLifecycleError::InvalidVersion {
                generation: self.generation,
                sequence: self.sequence,
                initial_snapshot,
            });
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObservationBatch {
    /// Durable, comparable ordering token for every update in this batch.
    /// Consumers must track the last applied version per entity and reject
    /// equal or older versions so a delayed upsert cannot resurrect an
    /// observation invalidated by a newer batch.
    pub version: ObservationVersion,
    /// True when this batch reconciles an initial provider snapshot. Allowlisted
    /// profile entities absent from that snapshot receive explicit invalidations.
    pub initial_snapshot: bool,
    /// Sorted by exact entity ID, with at most one operation per entity.
    pub updates: Vec<ObservationUpdate>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ObservationLifecycleError {
    InvalidProfile(String),
    InvalidVersion {
        generation: u64,
        sequence: u64,
        initial_snapshot: bool,
    },
    EntityOutsideProfile { entity_id: String },
    ContradictoryEntityUpdate { entity_id: String },
}

impl fmt::Display for ObservationLifecycleError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidProfile(message) => write!(f, "invalid observation profile: {message}"),
            Self::InvalidVersion { generation, sequence, initial_snapshot } => write!(
                f,
                "invalid observation version ({generation}, {sequence}) for initial_snapshot={initial_snapshot}"
            ),
            Self::EntityOutsideProfile { entity_id } => write!(
                f,
                "observation frame is outside the active profile allowlist: {entity_id}"
            ),
            Self::ContradictoryEntityUpdate { entity_id } => write!(
                f,
                "projection contains multiple lifecycle updates for entity {entity_id}"
            ),
        }
    }
}

impl std::error::Error for ObservationLifecycleError {}

/// Convert a compact projection into explicit upserts and invalidations. Any
/// profile error rejects the whole batch. Invalid current samples become entity
/// invalidations, so a prior positive value cannot silently survive a known-bad
/// current sample.
///
/// A downstream store should apply the returned batch atomically. The caller
/// supplies receive time for removals and snapshot omissions; mapped frames
/// retain their own local receive timestamp.
pub fn map_projection(
    profile: &HomeAssistantObservationProfile,
    projection: &EntityStreamProjection,
    received_at_micros: i64,
    version: ObservationVersion,
) -> Result<ObservationBatch, ObservationLifecycleError> {
    profile
        .validate()
        .map_err(|error| ObservationLifecycleError::InvalidProfile(error.to_string()))?;
    version.validate_for(projection.initial_snapshot)?;

    let frame_ids: BTreeSet<&str> = projection
        .frames
        .iter()
        .map(|frame| frame.entity_id.as_str())
        .collect();
    let mut updates: BTreeMap<String, ObservationUpdate> = BTreeMap::new();

    if projection.initial_snapshot {
        for rule in &profile.rules {
            if !frame_ids.contains(rule.entity_id.as_str()) {
                insert_update(
                    &mut updates,
                    ObservationUpdate::Invalidate {
                        entity_id: rule.entity_id.clone(),
                        reason: ObservationInvalidationReason::MissingFromInitialSnapshot,
                        at_micros: received_at_micros,
                    },
                )?;
            }
        }
    }

    for entity_id in &projection.removed_entity_ids {
        insert_update(
            &mut updates,
            ObservationUpdate::Invalidate {
                entity_id: entity_id.clone(),
                reason: ObservationInvalidationReason::EntityRemoved,
                at_micros: received_at_micros,
            },
        )?;
    }

    for frame in &projection.frames {
        let update = match map_state(profile, frame) {
            Ok(MappingOutcome::Observation(observation)) => {
                ObservationUpdate::Upsert(observation)
            }
            Ok(MappingOutcome::SourceState {
                entity_id,
                status,
                received_at_micros,
                ..
            }) => ObservationUpdate::Invalidate {
                entity_id,
                reason: match status {
                    HomeAssistantSourceStatus::Unknown => ObservationInvalidationReason::SourceUnknown,
                    HomeAssistantSourceStatus::Unavailable => ObservationInvalidationReason::SourceUnavailable,
                },
                at_micros: received_at_micros,
            },
            Ok(MappingOutcome::NotAllowlisted { entity_id }) => {
                return Err(ObservationLifecycleError::EntityOutsideProfile { entity_id });
            }
            Err(ObservationError::InvalidProfile(message)) => {
                return Err(ObservationLifecycleError::InvalidProfile(message));
            }
            // A malformed/current value, drifted metadata, codec/range failure,
            // or freshness overflow invalidates the last accepted observation.
            Err(_) => ObservationUpdate::Invalidate {
                entity_id: frame.entity_id.clone(),
                reason: ObservationInvalidationReason::InvalidCurrentSample,
                at_micros: frame.received_at_micros,
            },
        };
        insert_update(&mut updates, update)?;
    }

    Ok(ObservationBatch {
        version,
        initial_snapshot: projection.initial_snapshot,
        updates: updates.into_values().collect(),
    })
}

/// Convert a failed transactional projection into a conservative invalidation
/// barrier. A projector error may name only the first bad entity even when the
/// same rejected delta also contained other state changes, so the complete
/// affected set is unknowable. Invalidate every entity in the currently valid
/// profile; the transport must keep the stream non-live until a fresh complete
/// snapshot is applied. Never use an invalid profile to author invalidations.
pub fn map_projector_failure(
    profile: &HomeAssistantObservationProfile,
    error: &EntityStreamProjectionError,
    received_at_micros: i64,
    version: ObservationVersion,
    initial_snapshot: bool,
) -> Result<ObservationBatch, ObservationLifecycleError> {
    if let EntityStreamProjectionError::InvalidProfile(message) = error {
        return Err(ObservationLifecycleError::InvalidProfile(message.clone()));
    }
    profile
        .validate()
        .map_err(|error| ObservationLifecycleError::InvalidProfile(error.to_string()))?;
    version.validate_for(initial_snapshot)?;

    let updates = profile
        .rules
        .iter()
        .map(|rule| {
            (
                rule.entity_id.clone(),
                ObservationUpdate::Invalidate {
                    entity_id: rule.entity_id.clone(),
                    reason: ObservationInvalidationReason::ProjectionRejected,
                    at_micros: received_at_micros,
                },
            )
        })
        .collect::<BTreeMap<_, _>>()
        .into_values()
        .collect();

    Ok(ObservationBatch {
        version,
        initial_snapshot,
        updates,
    })
}

fn insert_update(
    updates: &mut BTreeMap<String, ObservationUpdate>,
    update: ObservationUpdate,
) -> Result<(), ObservationLifecycleError> {
    let entity_id = update.entity_id().to_owned();
    if updates.contains_key(&entity_id) {
        return Err(ObservationLifecycleError::ContradictoryEntityUpdate { entity_id });
    }
    updates.insert(entity_id, update);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        EntityObservationRule, EntityRef, HomeAssistantStateFrame, StateCodec,
        HOME_ASSISTANT_OBSERVATION_SCHEMA_VERSION,
        entity_stream::EntityStreamProjector,
        websocket_protocol::EntityStreamKind,
    };
    use serde_json::json;

    fn rule(entity_id: &str, subject_id: &str) -> EntityObservationRule {
        EntityObservationRule {
            entity_id: entity_id.into(),
            subject: EntityRef {
                kind: "room".into(),
                id: subject_id.into(),
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
        }
    }

    fn profile() -> HomeAssistantObservationProfile {
        HomeAssistantObservationProfile {
            schema_version: HOME_ASSISTANT_OBSERVATION_SCHEMA_VERSION,
            instance_id: "home-main".into(),
            profile_ref: "profile:observation-lifecycle-v1".into(),
            rules: vec![rule("sensor.living_room_temperature", "living-room")],
        }
    }

    fn snapshot(state: &str) -> serde_json::Value {
        json!({
            "a": {
                "sensor.living_room_temperature": {
                    "s": state,
                    "a": {
                        "unit_of_measurement": "°C",
                        "device_class": "temperature",
                        "friendly_name": "must-not-become-semantic"
                    }
                }
            }
        })
    }

    fn apply_snapshot(
        profile: &HomeAssistantObservationProfile,
        state: &str,
        received_at_micros: i64,
    ) -> (EntityStreamProjector, EntityStreamProjection) {
        let mut projector = EntityStreamProjector::new(profile).unwrap();
        let projection = projector
            .apply(
                EntityStreamKind::InitialSnapshot,
                &snapshot(state),
                received_at_micros,
            )
            .unwrap();
        (projector, projection)
    }

    fn version(initial_snapshot: bool, sequence: u64) -> ObservationVersion {
        ObservationVersion {
            generation: 1,
            sequence: if initial_snapshot { 0 } else { sequence.max(1) },
        }
    }

    fn invalidation_for<'a>(
        batch: &'a ObservationBatch,
        entity_id: &str,
    ) -> Option<&'a ObservationInvalidationReason> {
        batch.updates.iter().find_map(|update| match update {
            ObservationUpdate::Invalidate {
                entity_id: found,
                reason,
                ..
            } if found == entity_id => Some(reason),
            _ => None,
        })
    }

    #[test]
    fn unknown_and_unavailable_invalidate_previous_positive_observation() {
        for (state, expected) in [
            ("unknown", ObservationInvalidationReason::SourceUnknown),
            ("unavailable", ObservationInvalidationReason::SourceUnavailable),
        ] {
            let profile = profile();
            let (_, projection) = apply_snapshot(&profile, state, 10);
            let batch = map_projection(&profile, &projection, 10, version(true, 0)).unwrap();
            assert_eq!(
                invalidation_for(&batch, "sensor.living_room_temperature"),
                Some(&expected)
            );
            assert!(!batch.updates.iter().any(|update| matches!(update, ObservationUpdate::Upsert(_))));
        }
    }

    #[test]
    fn out_of_range_current_sample_invalidates_instead_of_preserving_old_upsert() {
        let profile = profile();
        let (_, projection) = apply_snapshot(&profile, "999.0", 20);
        let batch = map_projection(&profile, &projection, 20, version(true, 0)).unwrap();
        assert_eq!(
            invalidation_for(&batch, "sensor.living_room_temperature"),
            Some(&ObservationInvalidationReason::InvalidCurrentSample)
        );
        assert_eq!(batch.updates.len(), 1);
    }

    #[test]
    fn valid_current_sample_becomes_upsert_with_local_freshness() {
        let profile = profile();
        let (_, projection) = apply_snapshot(&profile, "21.5", 1_000_000);
        let batch = map_projection(&profile, &projection, 1_000_000, version(true, 0)).unwrap();
        assert_eq!(batch.updates.len(), 1);
        let ObservationUpdate::Upsert(observation) = &batch.updates[0] else {
            panic!("valid profile-matching observation must upsert")
        };
        assert_eq!(observation.observation.observed_at_micros, 1_000_000);
        assert_eq!(observation.observation.valid_until_micros, Some(31_000_000));
    }

    #[test]
    fn provider_removal_invalidates_without_claiming_physical_state() {
        let profile = profile();
        let (mut projector, _) = apply_snapshot(&profile, "21.5", 10);
        let removal = projector
            .apply(
                EntityStreamKind::Changes,
                &json!({"r": ["sensor.living_room_temperature"]}),
                20,
            )
            .unwrap();
        let batch = map_projection(&profile, &removal, 20, version(false, 1)).unwrap();
        assert_eq!(
            invalidation_for(&batch, "sensor.living_room_temperature"),
            Some(&ObservationInvalidationReason::EntityRemoved)
        );
    }

    #[test]
    fn reconnect_snapshot_omission_invalidates_prior_observation() {
        let mut profile = profile();
        profile.rules.push(rule("sensor.bedroom_temperature", "bedroom"));
        profile.validate().unwrap();

        let mut projector = EntityStreamProjector::new(&profile).unwrap();
        let partial_snapshot = json!({
            "a": {
                "sensor.living_room_temperature": {
                    "s": "21.5",
                    "a": {"unit_of_measurement": "°C", "device_class": "temperature"}
                }
            }
        });
        let projection = projector
            .apply(EntityStreamKind::InitialSnapshot, &partial_snapshot, 50)
            .unwrap();
        let batch = map_projection(&profile, &projection, 50, version(true, 0)).unwrap();

        assert!(batch.initial_snapshot);
        assert_eq!(
            invalidation_for(&batch, "sensor.bedroom_temperature"),
            Some(&ObservationInvalidationReason::MissingFromInitialSnapshot)
        );
        assert!(batch.updates.iter().any(|update| matches!(
            update,
            ObservationUpdate::Upsert(value) if value.entity_id == "sensor.living_room_temperature"
        )));
    }

    #[test]
    fn metadata_drift_projector_failure_invalidates_the_previous_observation() {
        let profile = profile();
        let mut event = snapshot("21.5");
        event["a"]["sensor.living_room_temperature"]["a"]["unit_of_measurement"] = json!("°F");
        let mut projector = EntityStreamProjector::new(&profile).unwrap();
        let rejected = projector
            .apply(EntityStreamKind::InitialSnapshot, &event, 10)
            .unwrap_err();
        assert_eq!(projector.cached_entity_count(), 0);
        assert!(!projector.initial_snapshot_seen());
        let batch = map_projector_failure(&profile, &rejected, 10, version(true, 0), true).unwrap();
        assert_eq!(
            invalidation_for(&batch, "sensor.living_room_temperature"),
            Some(&ObservationInvalidationReason::ProjectionRejected)
        );
    }

    #[test]
    fn rejected_multi_entity_delta_invalidates_every_profile_observation() {
        let mut profile = profile();
        let mut first = profile.rules[0].clone();
        first.entity_id = "sensor.a_temperature".into();
        first.subject.id = "room-a".into();
        let mut last = profile.rules[0].clone();
        last.entity_id = "sensor.z_temperature".into();
        last.subject.id = "room-z".into();
        // Deliberately use reverse insertion order: the returned batch must
        // remain sorted by exact entity ID, independent of profile ordering.
        profile.rules = vec![last, first];

        let mut projector = EntityStreamProjector::new(&profile).unwrap();
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
        projector
            .apply(EntityStreamKind::InitialSnapshot, &initial, 10)
            .unwrap();

        // The first entity is valid, but a later malformed entity rejects the
        // full write set. The old cached positives remain only as cache data;
        // the lifecycle batch marks both entities non-current.
        let rejected = projector
            .apply(
                EntityStreamKind::Changes,
                &json!({
                    "c": {
                        "sensor.a_temperature": {"+": {"s": "22.0"}},
                        "sensor.z_temperature": {"+": {"a": ["not-an-object"]}}
                    }
                }),
                20,
            )
            .unwrap_err();
        assert_eq!(projector.state["sensor.a_temperature"].state, "20.0");
        assert_eq!(projector.state["sensor.z_temperature"].state, "21.0");

        let batch = map_projector_failure(
            &profile,
            &rejected,
            20,
            version(false, 1),
            false,
        )
        .unwrap();
        assert_eq!(batch.updates.len(), 2);
        assert_eq!(
            batch.updates.iter().map(ObservationUpdate::entity_id).collect::<Vec<_>>(),
            vec!["sensor.a_temperature", "sensor.z_temperature"]
        );
        assert!(batch.updates.iter().all(|update| matches!(
            update,
            ObservationUpdate::Invalidate {
                reason: ObservationInvalidationReason::ProjectionRejected,
                at_micros: 20,
                ..
            }
        )));
        assert!(!batch.updates.iter().any(|update| matches!(update, ObservationUpdate::Upsert(_))));
    }

    #[test]
    fn profile_error_cannot_be_converted_into_entity_lifecycle_updates() {
        let profile = profile();
        assert!(matches!(
            map_projector_failure(
                &profile,
                &EntityStreamProjectionError::InvalidProfile("invalid selected profile".into()),
                20,
                version(false, 1),
                false,
            ),
            Err(ObservationLifecycleError::InvalidProfile(_))
        ));
    }

    #[test]
    fn profile_errors_reject_the_entire_batch() {
        let mut profile = profile();
        profile.rules.push(rule("sensor.living_room_temperature", "duplicate"));
        let projection = EntityStreamProjection {
            initial_snapshot: true,
            frames: Vec::<HomeAssistantStateFrame>::new(),
            removed_entity_ids: Vec::new(),
        };
        assert!(matches!(
            map_projection(&profile, &projection, 10, version(true, 0)),
            Err(ObservationLifecycleError::InvalidProfile(_))
        ));
    }

    #[test]
    fn version_ordering_rejects_equal_or_older_batches() {
        let previous = ObservationVersion { generation: 7, sequence: 18 };
        assert!(ObservationVersion { generation: 7, sequence: 19 }.is_newer_than(Some(previous)));
        assert!(ObservationVersion { generation: 8, sequence: 0 }.is_newer_than(Some(previous)));
        assert!(!previous.is_newer_than(Some(previous)));
        assert!(!ObservationVersion { generation: 7, sequence: 17 }.is_newer_than(Some(previous)));
        assert!(ObservationVersion { generation: 1, sequence: 0 }.is_newer_than(None));
    }

    #[test]
    fn delayed_positive_upsert_cannot_overwrite_newer_invalidation_version() {
        let invalidation_version = ObservationVersion { generation: 2, sequence: 1 };
        let delayed_old_upsert = ObservationVersion { generation: 1, sequence: 500 };
        let subsequent_current_sample = ObservationVersion { generation: 2, sequence: 2 };

        assert!(
            !delayed_old_upsert.is_newer_than(Some(invalidation_version)),
            "delayed positive sample from an older connection must be rejected"
        );
        assert!(subsequent_current_sample.is_newer_than(Some(invalidation_version)));
    }

    #[test]
    fn version_shape_requires_nonzero_generation_and_snapshot_sequence_zero() {
        let profile = profile();
        let (_, snapshot_projection) = apply_snapshot(&profile, "21.5", 10);
        assert!(matches!(
            map_projection(
                &profile,
                &snapshot_projection,
                10,
                ObservationVersion { generation: 0, sequence: 0 }
            ),
            Err(ObservationLifecycleError::InvalidVersion { .. })
        ));
        assert!(matches!(
            map_projection(
                &profile,
                &snapshot_projection,
                10,
                ObservationVersion { generation: 1, sequence: 4 }
            ),
            Err(ObservationLifecycleError::InvalidVersion { .. })
        ));
        let mut projector = EntityStreamProjector::new(&profile).unwrap();
        let delta = projector.apply(
            EntityStreamKind::InitialSnapshot,
            &snapshot("21.5"),
            10,
        ).unwrap();
        let changed = projector.apply(
            EntityStreamKind::Changes,
            &json!({"c": {"sensor.living_room_temperature": {"+": {"s": "22.0"}}}}),
            20,
        ).unwrap();
        assert!(matches!(
            map_projection(&profile, &changed, 20, ObservationVersion { generation: 1, sequence: 0 }),
            Err(ObservationLifecycleError::InvalidVersion { .. })
        ));
        assert!(delta.initial_snapshot);
    }

    #[test]
    fn outside_profile_frame_rejects_entire_lifecycle_batch() {
        let profile = profile();
        let projection = EntityStreamProjection {
            initial_snapshot: false,
            frames: vec![HomeAssistantStateFrame {
                entity_id: "sensor.not_allowlisted".into(),
                state: "21.5".into(),
                unit_of_measurement: Some("°C".into()),
                device_class: Some("temperature".into()),
                source_last_updated_micros: None,
                received_at_micros: 10,
            }],
            removed_entity_ids: Vec::new(),
        };
        assert!(matches!(
            map_projection(&profile, &projection, 10, version(false, 1)),
            Err(ObservationLifecycleError::EntityOutsideProfile { .. })
        ));
    }

    #[test]
    fn contradictory_projection_operations_fail_closed() {
        let profile = profile();
        let mut projector = EntityStreamProjector::new(&profile).unwrap();
        let frame = projector
            .apply(EntityStreamKind::InitialSnapshot, &snapshot("21.5"), 10)
            .unwrap()
            .frames
            .remove(0);
        let contradictory = EntityStreamProjection {
            initial_snapshot: false,
            frames: vec![frame],
            removed_entity_ids: vec!["sensor.living_room_temperature".into()],
        };
        assert!(matches!(
            map_projection(&profile, &contradictory, 20, version(false, 1)),
            Err(ObservationLifecycleError::ContradictoryEntityUpdate { .. })
        ));
    }
}
