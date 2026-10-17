// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Pure, read-only state machine for Home Assistant entity subscriptions.
//!
//! Prefer the provider-native subscribe_entities command: the server sends an
//! initial compact entity snapshot followed by incremental changes over one
//! subscription. The validated HA-0.1 profile supplies the exact entity
//! allowlist. No separate get_states request or state_changed subscription is
//! needed, so the client does not manufacture a gap between two subscriptions.
//!
//! This module contains no socket, URL, TLS, credential, state-write,
//! service-call, or arbitrary-command API. The future transport must decode
//! the compressed initial/add/change/remove payloads under strict size bounds.
//! It must apply the initial snapshot before processing later stream events.

use super::HomeAssistantObservationProfile;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionPhase {
    AwaitAuthRequired,
    AwaitAuthResult,
    AwaitSubscriptionResult,
    AwaitInitialSnapshot,
    AwaitInitialSnapshotApply,
    Live,
    Failed,
    Closed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageKind {
    AuthRequired,
    AuthOk,
    AuthInvalid,
    CommandResult,
    CommandFailure,
    EntityStreamUpdate,
    OtherEvent,
    Unexpected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntityStreamKind {
    InitialSnapshot,
    Changes,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResultPayloadKind {
    Null,
    Other,
}

/// A bounded projection of Home Assistant's documented error code. The raw
/// server message is deliberately not retained or emitted in diagnostics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandFailureCode {
    UnknownCommand,
    NotSupported,
    Other,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnsupportedSubscriptionReason {
    UnknownCommand,
    NotSupported,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProtocolError {
    InvalidObservationProfile,
    UnexpectedMessage {
        phase: SessionPhase,
        message: MessageKind,
    },
    AuthenticationRejected,
    CommandRejected {
        phase: SessionPhase,
    },
    /// The server explicitly reported that native subscribe_entities is not
    /// implemented/supported. No legacy fallback is attempted by this machine.
    UnsupportedEntitySubscription {
        reason: UnsupportedSubscriptionReason,
    },
    CommandIdMismatch {
        expected: u64,
        actual: u64,
    },
    SubscriptionIdMismatch {
        expected: u64,
        actual: u64,
    },
    WrongResultPayload {
        phase: SessionPhase,
        expected: ResultPayloadKind,
        actual: ResultPayloadKind,
    },
    UnexpectedEntityStreamKind {
        phase: SessionPhase,
        expected: EntityStreamKind,
        actual: EntityStreamKind,
    },
    CommandIdExhausted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProtocolState {
    AwaitAuthRequired,
    AwaitAuthResult,
    AwaitSubscriptionResult {
        command_id: u64,
    },
    AwaitInitialSnapshot {
        subscription_id: u64,
    },
    AwaitInitialSnapshotApply {
        subscription_id: u64,
    },
    Live {
        subscription_id: u64,
    },
    Failed(ProtocolError),
    Closed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InboundMessage {
    AuthRequired,
    AuthOk,
    AuthInvalid,
    CommandResult {
        id: u64,
        success: bool,
        payload: ResultPayloadKind,
    },
    /// Transport maps exact HA codes `unknown_command` and `not_supported`
    /// to their typed values; all other server codes map to `Other`. Raw error
    /// messages must not cross this boundary.
    CommandFailure {
        id: u64,
        code: CommandFailureCode,
    },
    EntityStreamUpdate {
        subscription_id: u64,
        kind: EntityStreamKind,
    },
    OtherEvent,
    Unexpected,
}

impl InboundMessage {
    fn kind(self) -> MessageKind {
        match self {
            Self::AuthRequired => MessageKind::AuthRequired,
            Self::AuthOk => MessageKind::AuthOk,
            Self::AuthInvalid => MessageKind::AuthInvalid,
            Self::CommandResult { .. } => MessageKind::CommandResult,
            Self::CommandFailure { .. } => MessageKind::CommandFailure,
            Self::EntityStreamUpdate { .. } => MessageKind::EntityStreamUpdate,
            Self::OtherEvent => MessageKind::OtherEvent,
            Self::Unexpected => MessageKind::Unexpected,
        }
    }
}

/// A closed set of transport effects. There is no service-call, state-write,
/// arbitrary command, or arbitrary event constructor. Entity IDs are copied
/// only from the already-validated HA-0.1 semantic profile.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProtocolAction {
    RequestAuthentication {
        session_epoch: u64,
    },
    SendSubscribeEntities {
        session_epoch: u64,
        command_id: u64,
        entity_ids: Vec<String>,
    },
    AwaitInitialEntitySnapshot {
        session_epoch: u64,
        subscription_id: u64,
    },
    InitialEntitySnapshotReady {
        session_epoch: u64,
        subscription_id: u64,
    },
    SessionLive {
        session_epoch: u64,
        subscription_id: u64,
    },
    DeliverEntityChanges {
        session_epoch: u64,
        subscription_id: u64,
    },
    IgnoreStaleEpoch {
        received_epoch: u64,
        active_epoch: u64,
    },
    IgnoreUnrelatedEvent {
        session_epoch: u64,
    },
    IgnoreTerminalMessage {
        session_epoch: u64,
    },
    FailClosed {
        session_epoch: u64,
        error: ProtocolError,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HomeAssistantWebSocketProtocol {
    session_epoch: u64,
    allowed_entity_ids: Vec<String>,
    next_command_id: Option<u64>,
    state: ProtocolState,
}

impl HomeAssistantWebSocketProtocol {
    /// Start a fresh session using a validated semantic observation profile.
    /// An invalid or empty profile cannot generate a subscription action.
    pub fn new(
        session_epoch: u64,
        profile: &HomeAssistantObservationProfile,
    ) -> Result<Self, ProtocolError> {
        profile
            .validate()
            .map_err(|_| ProtocolError::InvalidObservationProfile)?;

        let mut allowed_entity_ids: Vec<String> = profile
            .rules
            .iter()
            .map(|rule| rule.entity_id.clone())
            .collect();
        allowed_entity_ids.sort();
        allowed_entity_ids.dedup();

        if allowed_entity_ids.is_empty() || allowed_entity_ids.len() != profile.rules.len() {
            return Err(ProtocolError::InvalidObservationProfile);
        }

        Ok(Self {
            session_epoch,
            allowed_entity_ids,
            next_command_id: Some(1),
            state: ProtocolState::AwaitAuthRequired,
        })
    }

    pub fn session_epoch(&self) -> u64 {
        self.session_epoch
    }

    pub fn state(&self) -> ProtocolState {
        self.state
    }

    pub fn phase(&self) -> SessionPhase {
        match self.state {
            ProtocolState::AwaitAuthRequired => SessionPhase::AwaitAuthRequired,
            ProtocolState::AwaitAuthResult => SessionPhase::AwaitAuthResult,
            ProtocolState::AwaitSubscriptionResult { .. } => SessionPhase::AwaitSubscriptionResult,
            ProtocolState::AwaitInitialSnapshot { .. } => SessionPhase::AwaitInitialSnapshot,
            ProtocolState::AwaitInitialSnapshotApply { .. } => SessionPhase::AwaitInitialSnapshotApply,
            ProtocolState::Live { .. } => SessionPhase::Live,
            ProtocolState::Failed(_) => SessionPhase::Failed,
            ProtocolState::Closed => SessionPhase::Closed,
        }
    }

    /// A disconnected session is terminal. Reconnect creates a fresh machine,
    /// fresh epoch, new subscription, and a new initial snapshot.
    pub fn close(&mut self) {
        if !matches!(self.state, ProtocolState::Failed(_)) {
            self.state = ProtocolState::Closed;
        }
    }

    /// Acknowledge that the initial entity snapshot has been decoded through
    /// the allowlisted mapper and applied. A serialized receive loop must call
    /// this before processing the next entity stream update.
    pub fn initial_snapshot_applied(
        &mut self,
        received_epoch: u64,
        subscription_id: u64,
    ) -> Vec<ProtocolAction> {
        if received_epoch != self.session_epoch {
            return vec![ProtocolAction::IgnoreStaleEpoch {
                received_epoch,
                active_epoch: self.session_epoch,
            }];
        }
        if matches!(self.state, ProtocolState::Failed(_) | ProtocolState::Closed) {
            return vec![ProtocolAction::IgnoreTerminalMessage {
                session_epoch: self.session_epoch,
            }];
        }

        match self.state {
            ProtocolState::AwaitInitialSnapshotApply { subscription_id: expected } => {
                if subscription_id != expected {
                    return self.fail(ProtocolError::SubscriptionIdMismatch {
                        expected,
                        actual: subscription_id,
                    });
                }
                self.state = ProtocolState::Live { subscription_id: expected };
                vec![ProtocolAction::SessionLive {
                    session_epoch: self.session_epoch,
                    subscription_id: expected,
                }]
            }
            _ => self.fail(ProtocolError::UnexpectedMessage {
                phase: self.phase(),
                message: MessageKind::Unexpected,
            }),
        }
    }

    /// Process one decoded protocol fact. The transport is responsible for
    /// bounded JSON decoding and must preserve frame order within each epoch.
    pub fn handle(
        &mut self,
        received_epoch: u64,
        message: InboundMessage,
    ) -> Vec<ProtocolAction> {
        if received_epoch != self.session_epoch {
            return vec![ProtocolAction::IgnoreStaleEpoch {
                received_epoch,
                active_epoch: self.session_epoch,
            }];
        }
        if matches!(self.state, ProtocolState::Failed(_) | ProtocolState::Closed) {
            return vec![ProtocolAction::IgnoreTerminalMessage {
                session_epoch: self.session_epoch,
            }];
        }

        let phase = self.phase();
        match (self.state, message) {
            (ProtocolState::AwaitAuthRequired, InboundMessage::AuthRequired) => {
                self.state = ProtocolState::AwaitAuthResult;
                vec![ProtocolAction::RequestAuthentication {
                    session_epoch: self.session_epoch,
                }]
            }
            (ProtocolState::AwaitAuthResult, InboundMessage::AuthOk) => {
                match self.allocate_command_id() {
                    Ok(command_id) => {
                        self.state = ProtocolState::AwaitSubscriptionResult { command_id };
                        vec![ProtocolAction::SendSubscribeEntities {
                            session_epoch: self.session_epoch,
                            command_id,
                            entity_ids: self.allowed_entity_ids.clone(),
                        }]
                    }
                    Err(error) => self.fail(error),
                }
            }
            (ProtocolState::AwaitAuthResult, InboundMessage::AuthInvalid) => {
                self.fail(ProtocolError::AuthenticationRejected)
            }
            (
                ProtocolState::AwaitSubscriptionResult { command_id: expected },
                InboundMessage::CommandResult { id, success, payload },
            ) => {
                if id != expected {
                    return self.fail(ProtocolError::CommandIdMismatch { expected, actual: id });
                }
                if !success {
                    return self.fail(ProtocolError::CommandRejected { phase });
                }
                if payload != ResultPayloadKind::Null {
                    return self.fail(ProtocolError::WrongResultPayload {
                        phase,
                        expected: ResultPayloadKind::Null,
                        actual: payload,
                    });
                }
                self.state = ProtocolState::AwaitInitialSnapshot {
                    subscription_id: expected,
                };
                vec![ProtocolAction::AwaitInitialEntitySnapshot {
                    session_epoch: self.session_epoch,
                    subscription_id: expected,
                }]
            }
            (
                ProtocolState::AwaitSubscriptionResult { command_id: expected },
                InboundMessage::CommandFailure { id, code },
            ) => {
                if id != expected {
                    return self.fail(ProtocolError::CommandIdMismatch { expected, actual: id });
                }
                match code {
                    CommandFailureCode::UnknownCommand => self.fail(
                        ProtocolError::UnsupportedEntitySubscription {
                            reason: UnsupportedSubscriptionReason::UnknownCommand,
                        },
                    ),
                    CommandFailureCode::NotSupported => self.fail(
                        ProtocolError::UnsupportedEntitySubscription {
                            reason: UnsupportedSubscriptionReason::NotSupported,
                        },
                    ),
                    CommandFailureCode::Other => self.fail(ProtocolError::CommandRejected { phase }),
                }
            }
            (
                ProtocolState::AwaitInitialSnapshot { subscription_id: expected },
                InboundMessage::EntityStreamUpdate { subscription_id, kind },
            ) => {
                if subscription_id != expected {
                    return self.fail(ProtocolError::SubscriptionIdMismatch {
                        expected,
                        actual: subscription_id,
                    });
                }
                if kind != EntityStreamKind::InitialSnapshot {
                    return self.fail(ProtocolError::UnexpectedEntityStreamKind {
                        phase,
                        expected: EntityStreamKind::InitialSnapshot,
                        actual: kind,
                    });
                }
                self.state = ProtocolState::AwaitInitialSnapshotApply { subscription_id: expected };
                vec![ProtocolAction::InitialEntitySnapshotReady {
                    session_epoch: self.session_epoch,
                    subscription_id: expected,
                }]
            }
            (
                ProtocolState::AwaitInitialSnapshotApply { subscription_id: expected },
                InboundMessage::EntityStreamUpdate { subscription_id, .. },
            ) => {
                if subscription_id != expected {
                    return self.fail(ProtocolError::SubscriptionIdMismatch {
                        expected,
                        actual: subscription_id,
                    });
                }
                self.fail(ProtocolError::UnexpectedMessage {
                    phase,
                    message: MessageKind::EntityStreamUpdate,
                })
            }
            (
                ProtocolState::Live { subscription_id: expected },
                InboundMessage::EntityStreamUpdate { subscription_id, kind },
            ) => {
                if subscription_id != expected {
                    return self.fail(ProtocolError::SubscriptionIdMismatch {
                        expected,
                        actual: subscription_id,
                    });
                }
                if kind != EntityStreamKind::Changes {
                    return self.fail(ProtocolError::UnexpectedEntityStreamKind {
                        phase,
                        expected: EntityStreamKind::Changes,
                        actual: kind,
                    });
                }
                vec![ProtocolAction::DeliverEntityChanges {
                    session_epoch: self.session_epoch,
                    subscription_id: expected,
                }]
            }
            (ProtocolState::Live { .. }, InboundMessage::OtherEvent) => {
                vec![ProtocolAction::IgnoreUnrelatedEvent {
                    session_epoch: self.session_epoch,
                }]
            }
            (_, unexpected) => self.fail(ProtocolError::UnexpectedMessage {
                phase,
                message: unexpected.kind(),
            }),
        }
    }

    fn allocate_command_id(&mut self) -> Result<u64, ProtocolError> {
        let id = self.next_command_id.ok_or(ProtocolError::CommandIdExhausted)?;
        self.next_command_id = id.checked_add(1);
        Ok(id)
    }

    fn fail(&mut self, error: ProtocolError) -> Vec<ProtocolAction> {
        self.state = ProtocolState::Failed(error);
        vec![ProtocolAction::FailClosed {
            session_epoch: self.session_epoch,
            error,
        }]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        EntityObservationRule, EntityRef, StateCodec,
        HOME_ASSISTANT_OBSERVATION_SCHEMA_VERSION,
    };

    fn profile() -> HomeAssistantObservationProfile {
        HomeAssistantObservationProfile {
            schema_version: HOME_ASSISTANT_OBSERVATION_SCHEMA_VERSION,
            instance_id: "home-main".into(),
            profile_ref: "profile:ha-observation-test-v1".into(),
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

    fn machine(epoch: u64) -> HomeAssistantWebSocketProtocol {
        HomeAssistantWebSocketProtocol::new(epoch, &profile())
            .expect("test profile must be valid")
    }

    fn send(
        machine: &mut HomeAssistantWebSocketProtocol,
        message: InboundMessage,
    ) -> Vec<ProtocolAction> {
        let epoch = machine.session_epoch();
        machine.handle(epoch, message)
    }

    fn reaches_auth_result(machine: &mut HomeAssistantWebSocketProtocol) {
        let epoch = machine.session_epoch();
        assert_eq!(
            send(machine, InboundMessage::AuthRequired),
            vec![ProtocolAction::RequestAuthentication { session_epoch: epoch }]
        );
        assert_eq!(machine.state(), ProtocolState::AwaitAuthResult);
    }

    fn reaches_subscription_result(machine: &mut HomeAssistantWebSocketProtocol) -> u64 {
        reaches_auth_result(machine);
        let epoch = machine.session_epoch();
        assert_eq!(
            send(machine, InboundMessage::AuthOk),
            vec![ProtocolAction::SendSubscribeEntities {
                session_epoch: epoch,
                command_id: 1,
                entity_ids: vec!["sensor.living_room_temperature".into()],
            }]
        );
        assert_eq!(machine.state(), ProtocolState::AwaitSubscriptionResult { command_id: 1 });
        1
    }

    fn reaches_initial_snapshot(machine: &mut HomeAssistantWebSocketProtocol) -> u64 {
        let subscription_id = reaches_subscription_result(machine);
        assert_eq!(
            send(machine, InboundMessage::CommandResult {
                id: subscription_id,
                success: true,
                payload: ResultPayloadKind::Null,
            }),
            vec![ProtocolAction::AwaitInitialEntitySnapshot {
                session_epoch: machine.session_epoch(),
                subscription_id,
            }]
        );
        assert_eq!(machine.state(), ProtocolState::AwaitInitialSnapshot { subscription_id });
        assert_eq!(
            send(machine, InboundMessage::EntityStreamUpdate {
                subscription_id,
                kind: EntityStreamKind::InitialSnapshot,
            }),
            vec![ProtocolAction::InitialEntitySnapshotReady {
                session_epoch: machine.session_epoch(),
                subscription_id,
            }]
        );
        assert_eq!(machine.state(), ProtocolState::AwaitInitialSnapshotApply { subscription_id });
        subscription_id
    }

    fn reaches_live(machine: &mut HomeAssistantWebSocketProtocol) -> u64 {
        let subscription_id = reaches_initial_snapshot(machine);
        let epoch = machine.session_epoch();
        assert_eq!(
            machine.initial_snapshot_applied(epoch, subscription_id),
            vec![ProtocolAction::SessionLive {
                session_epoch: epoch,
                subscription_id,
            }]
        );
        assert_eq!(machine.state(), ProtocolState::Live { subscription_id });
        subscription_id
    }

    #[test]
    fn requires_auth_subscription_snapshot_apply_then_live_changes() {
        let mut machine = machine(7);
        reaches_live(&mut machine);
        assert_eq!(
            send(&mut machine, InboundMessage::EntityStreamUpdate {
                subscription_id: 1,
                kind: EntityStreamKind::Changes,
            }),
            vec![ProtocolAction::DeliverEntityChanges {
                session_epoch: 7,
                subscription_id: 1,
            }]
        );
    }

    #[test]
    fn subscription_uses_only_sorted_profile_allowlist() {
        let mut p = profile();
        let mut second = p.rules[0].clone();
        second.entity_id = "binary_sensor.front_door".into();
        second.attribute = "open".into();
        second.codec = StateCodec::Bool {
            true_state: "on".into(),
            false_state: "off".into(),
        };
        second.expected_unit = None;
        second.expected_device_class = None;
        p.rules.push(second);

        let mut machine = HomeAssistantWebSocketProtocol::new(1, &p).unwrap();
        reaches_auth_result(&mut machine);
        assert_eq!(
            send(&mut machine, InboundMessage::AuthOk),
            vec![ProtocolAction::SendSubscribeEntities {
                session_epoch: 1,
                command_id: 1,
                entity_ids: vec![
                    "binary_sensor.front_door".into(),
                    "sensor.living_room_temperature".into(),
                ],
            }]
        );
    }

    #[test]
    fn invalid_profile_cannot_create_transport_session() {
        let mut p = profile();
        p.rules.clear();
        assert_eq!(
            HomeAssistantWebSocketProtocol::new(1, &p),
            Err(ProtocolError::InvalidObservationProfile)
        );
    }

    #[test]
    fn authentication_rejection_is_terminal() {
        let mut machine = machine(1);
        reaches_auth_result(&mut machine);
        assert_eq!(
            send(&mut machine, InboundMessage::AuthInvalid),
            vec![ProtocolAction::FailClosed {
                session_epoch: 1,
                error: ProtocolError::AuthenticationRejected,
            }]
        );
    }

    #[test]
    fn auth_ok_before_auth_required_fails_closed() {
        let mut machine = machine(1);
        assert!(matches!(
            send(&mut machine, InboundMessage::AuthOk).as_slice(),
            [ProtocolAction::FailClosed {
                error: ProtocolError::UnexpectedMessage { .. },
                ..
            }]
        ));
    }

    #[test]
    fn subscription_ack_requires_matching_id_success_and_null_payload() {
        let mut machine = machine(1);
        reaches_subscription_result(&mut machine);
        assert!(matches!(
            send(&mut machine, InboundMessage::CommandResult {
                id: 99,
                success: true,
                payload: ResultPayloadKind::Null,
            }).as_slice(),
            [ProtocolAction::FailClosed {
                error: ProtocolError::CommandIdMismatch { expected: 1, actual: 99 },
                ..
            }]
        ));

        let mut failed = machine(2);
        let id = reaches_subscription_result(&mut failed);
        assert!(matches!(
            send(&mut failed, InboundMessage::CommandResult {
                id,
                success: false,
                payload: ResultPayloadKind::Null,
            }).as_slice(),
            [ProtocolAction::FailClosed {
                error: ProtocolError::CommandRejected {
                    phase: SessionPhase::AwaitSubscriptionResult,
                },
                ..
            }]
        ));
    }

    #[test]
    fn unsupported_native_subscription_is_explicit_and_has_no_fallback() {
        for (code, reason) in [
            (CommandFailureCode::UnknownCommand, UnsupportedSubscriptionReason::UnknownCommand),
            (CommandFailureCode::NotSupported, UnsupportedSubscriptionReason::NotSupported),
        ] {
            let mut machine = machine(1);
            let id = reaches_subscription_result(&mut machine);
            assert_eq!(
                send(&mut machine, InboundMessage::CommandFailure { id, code }),
                vec![ProtocolAction::FailClosed {
                    session_epoch: 1,
                    error: ProtocolError::UnsupportedEntitySubscription { reason },
                }]
            );
            assert!(matches!(machine.state(), ProtocolState::Failed(_)));
        }
    }

    #[test]
    fn unsupported_subscription_result_still_requires_exact_command_id() {
        let mut machine = machine(1);
        reaches_subscription_result(&mut machine);
        assert_eq!(
            send(&mut machine, InboundMessage::CommandFailure {
                id: 77,
                code: CommandFailureCode::UnknownCommand,
            }),
            vec![ProtocolAction::FailClosed {
                session_epoch: 1,
                error: ProtocolError::CommandIdMismatch { expected: 1, actual: 77 },
            }]
        );
    }

    #[test]
    fn other_subscription_errors_remain_generic_fail_closed_rejections() {
        let mut machine = machine(1);
        let id = reaches_subscription_result(&mut machine);
        assert_eq!(
            send(&mut machine, InboundMessage::CommandFailure {
                id,
                code: CommandFailureCode::Other,
            }),
            vec![ProtocolAction::FailClosed {
                session_epoch: 1,
                error: ProtocolError::CommandRejected {
                    phase: SessionPhase::AwaitSubscriptionResult,
                },
            }]
        );
    }

    #[test]
    fn initial_snapshot_requires_exact_subscription_id() {
        let mut machine = machine(1);
        reaches_subscription_result(&mut machine);
        send(&mut machine, InboundMessage::CommandResult {
            id: 1,
            success: true,
            payload: ResultPayloadKind::Null,
        });
        assert!(matches!(
            send(&mut machine, InboundMessage::EntityStreamUpdate {
                subscription_id: 77,
                kind: EntityStreamKind::InitialSnapshot,
            }).as_slice(),
            [ProtocolAction::FailClosed {
                error: ProtocolError::SubscriptionIdMismatch { expected: 1, actual: 77 },
                ..
            }]
        ));
    }

    #[test]
    fn changes_before_initial_snapshot_fail_closed() {
        let mut machine = machine(1);
        let id = reaches_subscription_result(&mut machine);
        send(&mut machine, InboundMessage::CommandResult {
            id,
            success: true,
            payload: ResultPayloadKind::Null,
        });
        assert!(matches!(
            send(&mut machine, InboundMessage::EntityStreamUpdate {
                subscription_id: id,
                kind: EntityStreamKind::Changes,
            }).as_slice(),
            [ProtocolAction::FailClosed {
                error: ProtocolError::UnexpectedEntityStreamKind {
                    phase: SessionPhase::AwaitInitialSnapshot,
                    expected: EntityStreamKind::InitialSnapshot,
                    actual: EntityStreamKind::Changes,
                },
                ..
            }]
        ));
    }

    #[test]
    fn initial_snapshot_must_be_applied_before_live() {
        let mut machine = machine(1);
        let id = reaches_initial_snapshot(&mut machine);
        assert!(matches!(
            machine.initial_snapshot_applied(1, id + 1).as_slice(),
            [ProtocolAction::FailClosed {
                error: ProtocolError::SubscriptionIdMismatch { expected: 1, actual: 2 },
                ..
            }]
        ));
    }

    #[test]
    fn initial_snapshot_cannot_be_acknowledged_before_it_arrives() {
        let mut machine = machine(1);
        reaches_subscription_result(&mut machine);
        assert!(matches!(
            machine.initial_snapshot_applied(1, 1).as_slice(),
            [ProtocolAction::FailClosed {
                error: ProtocolError::UnexpectedMessage {
                    phase: SessionPhase::AwaitSubscriptionResult,
                    ..
                },
                ..
            }]
        ));
    }

    #[test]
    fn second_snapshot_or_event_before_acknowledgement_fails_closed() {
        let mut machine = machine(1);
        let id = reaches_initial_snapshot(&mut machine);
        assert!(matches!(
            send(&mut machine, InboundMessage::EntityStreamUpdate {
                subscription_id: id,
                kind: EntityStreamKind::Changes,
            }).as_slice(),
            [ProtocolAction::FailClosed {
                error: ProtocolError::UnexpectedMessage {
                    phase: SessionPhase::AwaitInitialSnapshotApply,
                    ..
                },
                ..
            }]
        ));
    }

    #[test]
    fn only_change_frames_are_delivered_after_live() {
        let mut machine = machine(1);
        reaches_live(&mut machine);
        assert!(matches!(
            send(&mut machine, InboundMessage::EntityStreamUpdate {
                subscription_id: 1,
                kind: EntityStreamKind::InitialSnapshot,
            }).as_slice(),
            [ProtocolAction::FailClosed {
                error: ProtocolError::UnexpectedEntityStreamKind {
                    phase: SessionPhase::Live,
                    expected: EntityStreamKind::Changes,
                    actual: EntityStreamKind::InitialSnapshot,
                },
                ..
            }]
        ));
    }

    #[test]
    fn wrong_subscription_id_fails_closed_after_live() {
        let mut machine = machine(1);
        reaches_live(&mut machine);
        assert!(matches!(
            send(&mut machine, InboundMessage::EntityStreamUpdate {
                subscription_id: 77,
                kind: EntityStreamKind::Changes,
            }).as_slice(),
            [ProtocolAction::FailClosed {
                error: ProtocolError::SubscriptionIdMismatch { expected: 1, actual: 77 },
                ..
            }]
        ));
    }

    #[test]
    fn stale_session_frames_cannot_advance_new_epoch() {
        let mut machine = machine(42);
        assert_eq!(
            machine.handle(41, InboundMessage::AuthRequired),
            vec![ProtocolAction::IgnoreStaleEpoch {
                received_epoch: 41,
                active_epoch: 42,
            }]
        );
        assert_eq!(machine.state(), ProtocolState::AwaitAuthRequired);
    }

    #[test]
    fn reconnect_requires_new_snapshot_and_prior_session_is_terminal() {
        let mut prior = machine(10);
        let old_id = reaches_live(&mut prior);
        prior.close();

        let mut current = machine(11);
        assert_eq!(
            send(&mut current, InboundMessage::AuthRequired),
            vec![ProtocolAction::RequestAuthentication { session_epoch: 11 }]
        );
        assert_eq!(
            prior.handle(10, InboundMessage::EntityStreamUpdate {
                subscription_id: old_id,
                kind: EntityStreamKind::Changes,
            }),
            vec![ProtocolAction::IgnoreTerminalMessage { session_epoch: 10 }]
        );
        assert_eq!(current.state(), ProtocolState::AwaitAuthResult);
    }

    #[test]
    fn late_old_command_results_are_ignored() {
        let mut machine = machine(52);
        assert!(matches!(
            machine.handle(51, InboundMessage::CommandResult {
                id: 1,
                success: true,
                payload: ResultPayloadKind::Null,
            }).as_slice(),
            [ProtocolAction::IgnoreStaleEpoch { .. }]
        ));
        assert_eq!(machine.state(), ProtocolState::AwaitAuthRequired);
    }

    #[test]
    fn unrelated_live_events_are_ignored() {
        let mut machine = machine(9);
        reaches_live(&mut machine);
        assert_eq!(
            send(&mut machine, InboundMessage::OtherEvent),
            vec![ProtocolAction::IgnoreUnrelatedEvent { session_epoch: 9 }]
        );
    }

    #[test]
    fn failed_session_never_emits_new_protocol_commands() {
        let mut machine = machine(1);
        send(&mut machine, InboundMessage::AuthOk);
        assert!(matches!(machine.state(), ProtocolState::Failed(_)));
        assert_eq!(
            send(&mut machine, InboundMessage::AuthRequired),
            vec![ProtocolAction::IgnoreTerminalMessage { session_epoch: 1 }]
        );
    }

    #[test]
    fn command_id_exhaustion_fails_closed() {
        let mut machine = machine(1);
        reaches_auth_result(&mut machine);
        machine.next_command_id = None;
        assert_eq!(
            send(&mut machine, InboundMessage::AuthOk),
            vec![ProtocolAction::FailClosed {
                session_epoch: 1,
                error: ProtocolError::CommandIdExhausted,
            }]
        );
    }

    #[test]
    fn exhaustively_replays_one_million_bounded_protocol_histories() {
        const EVENT_ALPHABET: u32 = 10;
        const HISTORY_LENGTH: usize = 6;
        const SCHEDULE_COUNT: u32 = 1_000_000;
        let mut live_histories = 0usize;

        for mut encoded in 0..SCHEDULE_COUNT {
            let mut events = [0u8; HISTORY_LENGTH];
            for event in &mut events {
                *event = (encoded % EVENT_ALPHABET) as u8;
                encoded /= EVENT_ALPHABET;
            }

            let mut protocol = machine(7);
            let mut live_was_authorized = false;
            for &symbol in &events {
                let before = protocol.state();
                let epoch = protocol.session_epoch();
                let actions = match symbol {
                    // 0: auth_required
                    0 => protocol.handle(epoch, InboundMessage::AuthRequired),
                    // 1: auth_ok
                    1 => protocol.handle(epoch, InboundMessage::AuthOk),
                    // 2: auth_invalid
                    2 => protocol.handle(epoch, InboundMessage::AuthInvalid),
                    // 3: matching successful subscribe_entities result
                    3 => protocol.handle(epoch, InboundMessage::CommandResult {
                        id: 1,
                        success: true,
                        payload: ResultPayloadKind::Null,
                    }),
                    // 4: a successful result with the wrong command ID
                    4 => protocol.handle(epoch, InboundMessage::CommandResult {
                        id: 99,
                        success: true,
                        payload: ResultPayloadKind::Null,
                    }),
                    // 5: the expected initial snapshot
                    5 => protocol.handle(epoch, InboundMessage::EntityStreamUpdate {
                        subscription_id: 1,
                        kind: EntityStreamKind::InitialSnapshot,
                    }),
                    // 6: an incremental update for the active subscription
                    6 => protocol.handle(epoch, InboundMessage::EntityStreamUpdate {
                        subscription_id: 1,
                        kind: EntityStreamKind::Changes,
                    }),
                    // 7: an update with a foreign subscription ID
                    7 => protocol.handle(epoch, InboundMessage::EntityStreamUpdate {
                        subscription_id: 99,
                        kind: EntityStreamKind::Changes,
                    }),
                    // 8: an unrelated event
                    8 => protocol.handle(epoch, InboundMessage::OtherEvent),
                    // 9: explicit acknowledgement that the initial snapshot was applied
                    9 => protocol.initial_snapshot_applied(epoch, 1),
                    _ => unreachable!("symbol generated outside the ten-event alphabet"),
                };

                for action in &actions {
                    match action {
                        ProtocolAction::RequestAuthentication { session_epoch } => {
                            assert_eq!(symbol, 0, "authentication requested for {events:?}");
                            assert_eq!(before, ProtocolState::AwaitAuthRequired);
                            assert_eq!(*session_epoch, epoch);
                        }
                        ProtocolAction::SendSubscribeEntities {
                            session_epoch,
                            command_id,
                            entity_ids,
                        } => {
                            assert_eq!(symbol, 1, "subscription created for {events:?}");
                            assert_eq!(before, ProtocolState::AwaitAuthResult);
                            assert_eq!(*session_epoch, epoch);
                            assert_eq!(*command_id, 1);
                            assert_eq!(entity_ids, &vec!["sensor.living_room_temperature".to_string()]);
                        }
                        ProtocolAction::AwaitInitialEntitySnapshot {
                            session_epoch,
                            subscription_id,
                        } => {
                            assert_eq!(symbol, 3, "snapshot awaited for {events:?}");
                            assert_eq!(before, ProtocolState::AwaitSubscriptionResult { command_id: 1 });
                            assert_eq!(*session_epoch, epoch);
                            assert_eq!(*subscription_id, 1);
                        }
                        ProtocolAction::InitialEntitySnapshotReady {
                            session_epoch,
                            subscription_id,
                        } => {
                            assert_eq!(symbol, 5, "snapshot accepted for {events:?}");
                            assert_eq!(before, ProtocolState::AwaitInitialSnapshot { subscription_id: 1 });
                            assert_eq!(*session_epoch, epoch);
                            assert_eq!(*subscription_id, 1);
                        }
                        ProtocolAction::DeliverEntityChanges {
                            session_epoch,
                            subscription_id,
                        } => {
                            assert_eq!(symbol, 6, "change delivered for {events:?}");
                            assert_eq!(before, ProtocolState::Live { subscription_id: 1 });
                            assert_eq!(*session_epoch, epoch);
                            assert_eq!(*subscription_id, 1);
                        }
                        ProtocolAction::SessionLive {
                            session_epoch,
                            subscription_id,
                        } => {
                            assert_eq!(symbol, 9, "session went live without explicit snapshot acknowledgement: {events:?}");
                            assert_eq!(before, ProtocolState::AwaitInitialSnapshotApply { subscription_id: 1 });
                            assert_eq!(*session_epoch, epoch);
                            assert_eq!(*subscription_id, 1);
                            live_was_authorized = true;
                        }
                        ProtocolAction::IgnoreUnrelatedEvent { session_epoch } => {
                            assert_eq!(symbol, 8, "unexpected event was ignored for {events:?}");
                            assert_eq!(before, ProtocolState::Live { subscription_id: 1 });
                            assert_eq!(*session_epoch, epoch);
                        }
                        ProtocolAction::FailClosed { .. } => {
                            assert!(matches!(protocol.state(), ProtocolState::Failed(_)));
                        }
                        ProtocolAction::IgnoreTerminalMessage { .. } => {
                            assert!(matches!(before, ProtocolState::Failed(_) | ProtocolState::Closed));
                        }
                        ProtocolAction::IgnoreStaleEpoch { .. } => {
                            panic!("no stale epoch event was generated in {events:?}");
                        }
                    }
                }

                if matches!(protocol.state(), ProtocolState::Live { .. }) {
                    assert!(
                        live_was_authorized,
                        "protocol entered Live without explicit initial-snapshot acknowledgement: {events:?}"
                    );
                }
                if matches!(protocol.state(), ProtocolState::Failed(_) | ProtocolState::Closed) {
                    // Terminal states may never be revived by an in-session message.
                    let terminal_before = protocol.state();
                    let terminal_actions = protocol.handle(epoch, InboundMessage::AuthRequired);
                    assert_eq!(
                        terminal_actions,
                        vec![ProtocolAction::IgnoreTerminalMessage { session_epoch: epoch }]
                    );
                    assert_eq!(protocol.state(), terminal_before);
                }
            }

            if matches!(protocol.state(), ProtocolState::Live { .. }) {
                live_histories += 1;
            }
        }

        assert_eq!(SCHEDULE_COUNT, EVENT_ALPHABET.pow(HISTORY_LENGTH as u32));
        assert!(
            live_histories > 0,
            "bounded generator lost the valid authentication/snapshot/apply positive control"
        );
        assert_eq!(
            live_histories, 2,
            "only a complete auth/subscription/snapshot/apply sequence followed by one live-safe event may remain live at history end"
        );
    }

    #[test]
    fn authentication_action_carries_no_credential_value() {
        let action = ProtocolAction::RequestAuthentication { session_epoch: 1 };
        assert_eq!(
            format!("{action:?}"),
            "RequestAuthentication { session_epoch: 1 }"
        );
    }
}
