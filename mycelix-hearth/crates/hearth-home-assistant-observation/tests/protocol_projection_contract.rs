// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Cross-module contract tests: the protocol gate and semantic projector must
//! agree on when provider data may become an observation.

use hearth_automation_types::{AutomationValue, EntityRef};
use hearth_home_assistant_observation::{
    EntityObservationRule, HomeAssistantObservationProfile, MappingOutcome, StateCodec,
    HOME_ASSISTANT_OBSERVATION_SCHEMA_VERSION,
    entity_stream::{EntityStreamProjector, EntityStreamProjection},
    map_state,
    websocket_protocol::{
        EntityStreamKind, HomeAssistantWebSocketProtocol, InboundMessage, ProtocolAction,
        ProtocolError, ProtocolState, ResultPayloadKind,
    },
};
use serde_json::{Value, json};

fn profile() -> HomeAssistantObservationProfile {
    HomeAssistantObservationProfile {
        schema_version: HOME_ASSISTANT_OBSERVATION_SCHEMA_VERSION,
        instance_id: "home-main".into(),
        profile_ref: "profile:protocol-projection-e2e-v1".into(),
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

fn full_snapshot() -> Value {
    json!({
        "a": {
            "sensor.living_room_temperature": {
                "s": "21.5",
                "a": {
                    "unit_of_measurement": "°C",
                    "device_class": "temperature",
                    "friendly_name": "Living room",
                    "private_marker": "must-not-cross-boundary"
                },
                "c": "context-is-not-semantic-evidence",
                "lc": 1_700_000_000.0,
                "lu": 1_700_000_001.0
            },
            "sensor.not_allowlisted": {
                "s": "secret",
                "a": {"private_marker": "ignored"}
            }
        }
    })
}

fn send(
    protocol: &mut HomeAssistantWebSocketProtocol,
    message: InboundMessage,
) -> Vec<ProtocolAction> {
    let epoch = protocol.session_epoch();
    protocol.handle(epoch, message)
}

fn project_one(
    profile: &HomeAssistantObservationProfile,
    result: &EntityStreamProjection,
) -> Vec<MappingOutcome> {
    result.frames.iter()
        .map(|frame| map_state(profile, frame).expect("profile-projected frame must map"))
        .collect()
}

#[test]
fn protocol_gate_and_projector_form_one_snapshot_to_live_observation_path() {
    let profile = profile();
    let mut protocol = HomeAssistantWebSocketProtocol::new(7, &profile).unwrap();
    let mut projector = EntityStreamProjector::new(&profile).unwrap();

    assert_eq!(
        send(&mut protocol, InboundMessage::AuthRequired),
        vec![ProtocolAction::RequestAuthentication { session_epoch: 7 }]
    );
    assert_eq!(
        send(&mut protocol, InboundMessage::AuthOk),
        vec![ProtocolAction::SendSubscribeEntities {
            session_epoch: 7,
            command_id: 1,
            entity_ids: vec!["sensor.living_room_temperature".into()],
        }]
    );
    assert_eq!(
        send(&mut protocol, InboundMessage::CommandResult {
            id: 1,
            success: true,
            payload: ResultPayloadKind::Null,
        }),
        vec![ProtocolAction::AwaitInitialEntitySnapshot {
            session_epoch: 7,
            subscription_id: 1,
        }]
    );
    assert_eq!(
        send(&mut protocol, InboundMessage::EntityStreamUpdate {
            subscription_id: 1,
            kind: EntityStreamKind::InitialSnapshot,
        }),
        vec![ProtocolAction::InitialEntitySnapshotReady {
            session_epoch: 7,
            subscription_id: 1,
        }]
    );

    // A provider payload only reaches the projector after the protocol gate
    // admitted the exact subscription/epoch and expected snapshot type.
    let snapshot = projector.apply(
        EntityStreamKind::InitialSnapshot,
        &full_snapshot(),
        10_000_000,
    ).unwrap();
    assert_eq!(snapshot.frames.len(), 1);
    assert_eq!(snapshot.frames[0].state, "21.5");
    assert_eq!(snapshot.frames[0].received_at_micros, 10_000_000);
    assert_eq!(
        project_one(&profile, &snapshot),
        vec![MappingOutcome::Observation(
            hearth_home_assistant_observation::HomeAssistantObservation {
                profile_ref: "profile:protocol-projection-e2e-v1".into(),
                entity_id: "sensor.living_room_temperature".into(),
                source_last_updated_micros: None,
                received_at_micros: 10_000_000,
                observation: hearth_automation_types::Observation {
                    subject: EntityRef {
                        kind: "room".into(),
                        id: "living-room".into(),
                    },
                    attribute: "temperature_celsius".into(),
                    value: AutomationValue::Fixed { mantissa: 215, scale: 1 },
                    source: hearth_automation_types::EvidenceSource {
                        namespace: "home-assistant".into(),
                        id: "home-main:sensor.living_room_temperature".into(),
                    },
                    observed_at_micros: 10_000_000,
                    valid_until_micros: Some(40_000_000),
                    confidence_bp: 9_500,
                },
            }
        )]
    );
    assert_eq!(
        protocol.initial_snapshot_applied(7, 1),
        vec![ProtocolAction::SessionLive {
            session_epoch: 7,
            subscription_id: 1,
        }]
    );
    assert_eq!(protocol.state(), ProtocolState::Live { subscription_id: 1 });

    assert_eq!(
        send(&mut protocol, InboundMessage::EntityStreamUpdate {
            subscription_id: 1,
            kind: EntityStreamKind::Changes,
        }),
        vec![ProtocolAction::DeliverEntityChanges {
            session_epoch: 7,
            subscription_id: 1,
        }]
    );
    let changed = projector.apply(
        EntityStreamKind::Changes,
        &json!({
            "c": {
                "sensor.living_room_temperature": {
                    "+": {"s": "22.0", "lu": 1_700_000_002.0}
                }
            }
        }),
        20_000_000,
    ).unwrap();
    assert_eq!(project_one(&profile, &changed)[0], MappingOutcome::Observation(
        hearth_home_assistant_observation::HomeAssistantObservation {
            profile_ref: "profile:protocol-projection-e2e-v1".into(),
            entity_id: "sensor.living_room_temperature".into(),
            source_last_updated_micros: None,
            received_at_micros: 20_000_000,
            observation: hearth_automation_types::Observation {
                subject: EntityRef { kind: "room".into(), id: "living-room".into() },
                attribute: "temperature_celsius".into(),
                value: AutomationValue::Fixed { mantissa: 220, scale: 1 },
                source: hearth_automation_types::EvidenceSource {
                    namespace: "home-assistant".into(),
                    id: "home-main:sensor.living_room_temperature".into(),
                },
                observed_at_micros: 20_000_000,
                valid_until_micros: Some(50_000_000),
                confidence_bp: 9_500,
            },
        }
    ));

    // Attribute-only changes must never refresh semantic freshness.
    let metadata_only = projector.apply(
        EntityStreamKind::Changes,
        &json!({
            "c": {
                "sensor.living_room_temperature": {
                    "+": {
                        "a": {"friendly_name": "Renamed"},
                        "lu": 1_700_000_003.0
                    }
                }
            }
        }),
        30_000_000,
    ).unwrap();
    assert!(metadata_only.frames.is_empty());
    assert_eq!(metadata_only.frames.len(), 0);
}

#[test]
fn removal_invalidates_projected_state_without_emitting_a_positive_observation() {
    let profile = profile();
    let mut protocol = HomeAssistantWebSocketProtocol::new(12, &profile).unwrap();
    let mut projector = EntityStreamProjector::new(&profile).unwrap();

    send(&mut protocol, InboundMessage::AuthRequired);
    send(&mut protocol, InboundMessage::AuthOk);
    send(&mut protocol, InboundMessage::CommandResult {
        id: 1,
        success: true,
        payload: ResultPayloadKind::Null,
    });
    assert_eq!(
        send(&mut protocol, InboundMessage::EntityStreamUpdate {
            subscription_id: 1,
            kind: EntityStreamKind::InitialSnapshot,
        }),
        vec![ProtocolAction::InitialEntitySnapshotReady {
            session_epoch: 12,
            subscription_id: 1,
        }]
    );
    let initial = projector.apply(EntityStreamKind::InitialSnapshot, &full_snapshot(), 10_000_000).unwrap();
    assert_eq!(initial.frames.len(), 1);
    assert_eq!(protocol.initial_snapshot_applied(12, 1), vec![
        ProtocolAction::SessionLive { session_epoch: 12, subscription_id: 1 }
    ]);

    assert_eq!(
        send(&mut protocol, InboundMessage::EntityStreamUpdate {
            subscription_id: 1,
            kind: EntityStreamKind::Changes,
        }),
        vec![ProtocolAction::DeliverEntityChanges {
            session_epoch: 12,
            subscription_id: 1,
        }]
    );
    let removed = projector.apply(
        EntityStreamKind::Changes,
        &json!({"r": ["sensor.living_room_temperature"]}),
        20_000_000,
    ).unwrap();
    assert!(removed.frames.is_empty(), "removal is not a positive state observation");
    assert_eq!(removed.removed_entity_ids, vec!["sensor.living_room_temperature"]);
    assert_eq!(projector.cached_entity_count(), 0, "removed entity must no longer have cached positive state");
}

#[test]
fn mismatched_subscription_id_fails_closed_before_projector_invocation() {
    let profile = profile();
    let mut protocol = HomeAssistantWebSocketProtocol::new(9, &profile).unwrap();
    send(&mut protocol, InboundMessage::AuthRequired);
    send(&mut protocol, InboundMessage::AuthOk);
    send(&mut protocol, InboundMessage::CommandResult {
        id: 1,
        success: true,
        payload: ResultPayloadKind::Null,
    });

    let result = send(&mut protocol, InboundMessage::EntityStreamUpdate {
        subscription_id: 99,
        kind: EntityStreamKind::InitialSnapshot,
    });
    assert_eq!(
        result,
        vec![ProtocolAction::FailClosed {
            session_epoch: 9,
            error: ProtocolError::SubscriptionIdMismatch {
                expected: 1,
                actual: 99,
            },
        }]
    );
    assert!(matches!(protocol.state(), ProtocolState::Failed(_)));
}
