// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Pure read-only Home Assistant WebSocket protocol state machine.
//!
//! This crate intentionally has no socket, JSON, URL, TLS, filesystem, token,
//! credential, service-call, REST-write, or actuation dependency. A future
//! native transport owns wire parsing and secret handling and may feed only the
//! typed events represented here.

use hearth_home_assistant_observation::{
    map_state, HomeAssistantObservationProfile, HomeAssistantStateFrame, MappingOutcome,
    ObservationError,
};
use std::fmt;

pub const HOME_ASSISTANT_PROTOCOL_SCHEMA_VERSION: u16 = 1;
pub const MAX_HA_VERSION_BYTES: usize = 128;
pub const STATE_CHANGED_EVENT_TYPE: &str = "state_changed";

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SessionEpoch(pub u64);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CommandId(pub u64);

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SessionPhase {
    Disconnected,
    AwaitingAuthRequired,
    AwaitingAuthResult,
    AwaitingStates { command_id: CommandId },
    AwaitingSubscription { command_id: CommandId },
    Live { subscription_id: CommandId },
    Failed,
}

/// The complete post-authentication command surface admitted by HA-0.2A.
///
/// There is deliberately no generic command constructor and no service-call,
/// event-fire, registry-mutation, automation-trigger, state-write, or device-
/// control variant.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReadOnlyCommand {
    GetStates {
        epoch: SessionEpoch,
        id: CommandId,
    },
    SubscribeStateChanged {
        epoch: SessionEpoch,
        id: CommandId,
    },
}

impl ReadOnlyCommand {
    pub const fn epoch(&self) -> SessionEpoch {
        match self {
            Self::GetStates { epoch, .. } | Self::SubscribeStateChanged { epoch, .. } => *epoch,
        }
    }

    pub const fn id(&self) -> CommandId {
        match self {
            Self::GetStates { id, .. } | Self::SubscribeStateChanged { id, .. } => *id,
        }
    }

    pub const fn wire_type(&self) -> &'static str {
        match self {
            Self::GetStates { .. } => "get_states",
            Self::SubscribeStateChanged { .. } => "subscribe_events",
        }
    }

    pub const fn event_type(&self) -> Option<&'static str> {
        match self {
            Self::GetStates { .. } => None,
            Self::SubscribeStateChanged { .. } => Some(STATE_CHANGED_EVENT_TYPE),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProtocolOutput {
    NeedAuthentication {
        epoch: SessionEpoch,
        ha_version: String,
    },
    Send(ReadOnlyCommand),
    Bootstrap {
        epoch: SessionEpoch,
        outcomes: Vec<MappingOutcome>,
    },
    Live {
        epoch: SessionEpoch,
        subscription_id: CommandId,
    },
    State {
        epoch: SessionEpoch,
        outcome: MappingOutcome,
    },
    EntityRemoved {
        epoch: SessionEpoch,
        entity_id: String,
        received_at_micros: i64,
    },
    AuthenticationRejected {
        epoch: SessionEpoch,
    },
    Disconnected {
        epoch: SessionEpoch,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProtocolError {
    EpochOverflow,
    CommandIdOverflow,
    StaleEpoch {
        current: SessionEpoch,
        received: SessionEpoch,
    },
    UnexpectedPhase {
        expected: &'static str,
        actual: SessionPhase,
    },
    InvalidVersion(String),
    CommandRejected {
        command: &'static str,
        id: CommandId,
    },
    CorrelationMismatch {
        expected: CommandId,
        received: CommandId,
    },
    SubscriptionMismatch {
        expected: CommandId,
        received: CommandId,
    },
    EntityMismatch {
        event_entity_id: String,
        state_entity_id: String,
    },
    InvalidEntityId(String),
    Observation(ObservationError),
}

impl fmt::Display for ProtocolError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EpochOverflow => write!(f, "Home Assistant session epoch overflow"),
            Self::CommandIdOverflow => write!(f, "Home Assistant command id overflow"),
            Self::StaleEpoch { current, received } => write!(
                f,
                "stale Home Assistant session epoch: current={}, received={}",
                current.0, received.0
            ),
            Self::UnexpectedPhase { expected, actual } => {
                write!(f, "unexpected Home Assistant phase {actual:?}; expected {expected}")
            }
            Self::InvalidVersion(reason) => write!(f, "invalid Home Assistant version: {reason}"),
            Self::CommandRejected { command, id } => {
                write!(f, "Home Assistant command {command} id={} was rejected", id.0)
            }
            Self::CorrelationMismatch { expected, received } => write!(
                f,
                "Home Assistant result id mismatch: expected {}, received {}",
                expected.0, received.0
            ),
            Self::SubscriptionMismatch { expected, received } => write!(
                f,
                "Home Assistant subscription id mismatch: expected {}, received {}",
                expected.0, received.0
            ),
            Self::EntityMismatch {
                event_entity_id,
                state_entity_id,
            } => write!(
                f,
                "state_changed entity mismatch: event={event_entity_id:?}, state={state_entity_id:?}"
            ),
            Self::InvalidEntityId(reason) => write!(f, "invalid state_changed entity id: {reason}"),
            Self::Observation(error) => write!(f, "Home Assistant observation mapping failed: {error}"),
        }
    }
}

impl std::error::Error for ProtocolError {}

impl From<ObservationError> for ProtocolError {
    fn from(value: ObservationError) -> Self {
        Self::Observation(value)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HomeAssistantReadOnlyProtocol {
    epoch: SessionEpoch,
    next_command_id: u64,
    phase: SessionPhase,
}

impl Default for HomeAssistantReadOnlyProtocol {
    fn default() -> Self {
        Self {
            epoch: SessionEpoch(0),
            next_command_id: 1,
            phase: SessionPhase::Disconnected,
        }
    }
}

impl HomeAssistantReadOnlyProtocol {
    pub fn new() -> Self {
        Self::default()
    }

    pub const fn epoch(&self) -> SessionEpoch {
        self.epoch
    }

    pub fn phase(&self) -> &SessionPhase {
        &self.phase
    }

    /// Start a fresh connection epoch. Existing command/subscription IDs from
    /// prior epochs become stale even if their integer values are reused.
    pub fn begin_session(&mut self) -> Result<SessionEpoch, ProtocolError> {
        let next = self.epoch.0.checked_add(1).ok_or(ProtocolError::EpochOverflow)?;
        self.epoch = SessionEpoch(next);
        self.next_command_id = 1;
        self.phase = SessionPhase::AwaitingAuthRequired;
        Ok(self.epoch)
    }

    pub fn on_auth_required(
        &mut self,
        epoch: SessionEpoch,
        ha_version: impl Into<String>,
    ) -> Result<ProtocolOutput, ProtocolError> {
        self.require_epoch(epoch)?;
        self.require_phase("AwaitingAuthRequired", |phase| {
            matches!(phase, SessionPhase::AwaitingAuthRequired)
        })?;
        let ha_version = ha_version.into();
        validate_version(&ha_version)?;
        self.phase = SessionPhase::AwaitingAuthResult;
        Ok(ProtocolOutput::NeedAuthentication { epoch, ha_version })
    }

    /// The native transport calls this only after receiving Home Assistant's
    /// `auth_ok`. Authentication secret material never enters this state machine.
    pub fn on_auth_ok(
        &mut self,
        epoch: SessionEpoch,
    ) -> Result<ProtocolOutput, ProtocolError> {
        self.require_epoch(epoch)?;
        self.require_phase("AwaitingAuthResult", |phase| {
            matches!(phase, SessionPhase::AwaitingAuthResult)
        })?;
        let id = self.allocate_command_id()?;
        self.phase = SessionPhase::AwaitingStates { command_id: id };
        Ok(ProtocolOutput::Send(ReadOnlyCommand::GetStates { epoch, id }))
    }

    pub fn on_auth_invalid(
        &mut self,
        epoch: SessionEpoch,
    ) -> Result<ProtocolOutput, ProtocolError> {
        self.require_epoch(epoch)?;
        self.require_phase("AwaitingAuthResult", |phase| {
            matches!(phase, SessionPhase::AwaitingAuthResult)
        })?;
        self.phase = SessionPhase::Failed;
        Ok(ProtocolOutput::AuthenticationRejected { epoch })
    }

    pub fn on_states_result(
        &mut self,
        epoch: SessionEpoch,
        id: CommandId,
        success: bool,
        states: &[HomeAssistantStateFrame],
        profile: &HomeAssistantObservationProfile,
    ) -> Result<Vec<ProtocolOutput>, ProtocolError> {
        self.require_epoch(epoch)?;
        let expected = match &self.phase {
            SessionPhase::AwaitingStates { command_id } => *command_id,
            _ => {
                return Err(ProtocolError::UnexpectedPhase {
                    expected: "AwaitingStates",
                    actual: self.phase.clone(),
                });
            }
        };
        if id != expected {
            return Err(ProtocolError::CorrelationMismatch {
                expected,
                received: id,
            });
        }
        if !success {
            self.phase = SessionPhase::Failed;
            return Err(ProtocolError::CommandRejected {
                command: "get_states",
                id,
            });
        }

        if let Err(error) = profile.validate() {
            self.phase = SessionPhase::Failed;
            return Err(error.into());
        }

        let mut outcomes = Vec::with_capacity(states.len());
        for state in states {
            match map_state(profile, state) {
                Ok(outcome) => outcomes.push(outcome),
                Err(error) => {
                    self.phase = SessionPhase::Failed;
                    return Err(error.into());
                }
            }
        }

        let subscription_id = self.allocate_command_id()?;
        self.phase = SessionPhase::AwaitingSubscription {
            command_id: subscription_id,
        };
        Ok(vec![
            ProtocolOutput::Bootstrap { epoch, outcomes },
            ProtocolOutput::Send(ReadOnlyCommand::SubscribeStateChanged {
                epoch,
                id: subscription_id,
            }),
        ])
    }

    pub fn on_subscription_result(
        &mut self,
        epoch: SessionEpoch,
        id: CommandId,
        success: bool,
    ) -> Result<ProtocolOutput, ProtocolError> {
        self.require_epoch(epoch)?;
        let expected = match &self.phase {
            SessionPhase::AwaitingSubscription { command_id } => *command_id,
            _ => {
                return Err(ProtocolError::UnexpectedPhase {
                    expected: "AwaitingSubscription",
                    actual: self.phase.clone(),
                });
            }
        };
        if id != expected {
            return Err(ProtocolError::CorrelationMismatch {
                expected,
                received: id,
            });
        }
        if !success {
            self.phase = SessionPhase::Failed;
            return Err(ProtocolError::CommandRejected {
                command: "subscribe_events(state_changed)",
                id,
            });
        }
        self.phase = SessionPhase::Live {
            subscription_id: id,
        };
        Ok(ProtocolOutput::Live {
            epoch,
            subscription_id: id,
        })
    }

    pub fn on_state_changed(
        &mut self,
        epoch: SessionEpoch,
        subscription_id: CommandId,
        event_entity_id: impl Into<String>,
        new_state: Option<&HomeAssistantStateFrame>,
        received_at_micros: i64,
        profile: &HomeAssistantObservationProfile,
    ) -> Result<ProtocolOutput, ProtocolError> {
        self.require_epoch(epoch)?;
        let expected = match &self.phase {
            SessionPhase::Live { subscription_id } => *subscription_id,
            _ => {
                return Err(ProtocolError::UnexpectedPhase {
                    expected: "Live",
                    actual: self.phase.clone(),
                });
            }
        };
        if subscription_id != expected {
            return Err(ProtocolError::SubscriptionMismatch {
                expected,
                received: subscription_id,
            });
        }

        let event_entity_id = event_entity_id.into();
        validate_entity_id(&event_entity_id)?;
        match new_state {
            Some(state) => {
                if state.entity_id != event_entity_id {
                    return Err(ProtocolError::EntityMismatch {
                        event_entity_id,
                        state_entity_id: state.entity_id.clone(),
                    });
                }
                Ok(ProtocolOutput::State {
                    epoch,
                    outcome: map_state(profile, state)?,
                })
            }
            None => Ok(ProtocolOutput::EntityRemoved {
                epoch,
                entity_id: event_entity_id,
                received_at_micros,
            }),
        }
    }

    /// Disconnecting never emits a replacement observation and therefore never
    /// extends freshness. Existing HA-0.1 observations age on their original
    /// `valid_until_micros` deadlines.
    pub fn disconnect(
        &mut self,
        epoch: SessionEpoch,
    ) -> Result<ProtocolOutput, ProtocolError> {
        self.require_epoch(epoch)?;
        self.phase = SessionPhase::Disconnected;
        Ok(ProtocolOutput::Disconnected { epoch })
    }

    fn allocate_command_id(&mut self) -> Result<CommandId, ProtocolError> {
        let id = CommandId(self.next_command_id);
        self.next_command_id = self
            .next_command_id
            .checked_add(1)
            .ok_or(ProtocolError::CommandIdOverflow)?;
        Ok(id)
    }

    fn require_epoch(&self, received: SessionEpoch) -> Result<(), ProtocolError> {
        if received != self.epoch {
            return Err(ProtocolError::StaleEpoch {
                current: self.epoch,
                received,
            });
        }
        Ok(())
    }

    fn require_phase(
        &self,
        expected: &'static str,
        predicate: impl FnOnce(&SessionPhase) -> bool,
    ) -> Result<(), ProtocolError> {
        if !predicate(&self.phase) {
            return Err(ProtocolError::UnexpectedPhase {
                expected,
                actual: self.phase.clone(),
            });
        }
        Ok(())
    }
}

fn validate_version(value: &str) -> Result<(), ProtocolError> {
    if value.is_empty() || value.len() > MAX_HA_VERSION_BYTES {
        return Err(ProtocolError::InvalidVersion(format!(
            "must be in 1..={MAX_HA_VERSION_BYTES} bytes"
        )));
    }
    if value.chars().any(char::is_control) || value.trim() != value {
        return Err(ProtocolError::InvalidVersion(
            "must not contain control characters or surrounding whitespace".into(),
        ));
    }
    Ok(())
}

fn validate_entity_id(value: &str) -> Result<(), ProtocolError> {
    if value.is_empty() || value.len() > 255 {
        return Err(ProtocolError::InvalidEntityId(
            "entity_id must be in 1..=255 bytes".into(),
        ));
    }
    let Some((domain, object)) = value.split_once('.') else {
        return Err(ProtocolError::InvalidEntityId(
            "entity_id must be domain.object_id".into(),
        ));
    };
    if object.contains('.') || domain.is_empty() || object.is_empty() {
        return Err(ProtocolError::InvalidEntityId(
            "entity_id must be exactly domain.object_id".into(),
        ));
    }
    let valid = |part: &str| {
        part.bytes()
            .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit() || byte == b'_')
    };
    if !valid(domain) || !valid(object) {
        return Err(ProtocolError::InvalidEntityId(
            "entity_id halves may contain only [a-z0-9_]".into(),
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_home_assistant_observation::{
        EntityObservationRule, HomeAssistantSourceStatus, StateCodec,
        HOME_ASSISTANT_OBSERVATION_SCHEMA_VERSION,
    };
    use hearth_automation_types::{AutomationValue, EntityRef};

    fn profile() -> HomeAssistantObservationProfile {
        HomeAssistantObservationProfile {
            schema_version: HOME_ASSISTANT_OBSERVATION_SCHEMA_VERSION,
            instance_id: "home-main".into(),
            profile_ref: "ha-observation-profile:test".into(),
            rules: vec![EntityObservationRule {
                entity_id: "sensor.room_temperature".into(),
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
                confidence_bp: 9_000,
            }],
        }
    }

    fn state(value: &str, received_at_micros: i64) -> HomeAssistantStateFrame {
        HomeAssistantStateFrame {
            entity_id: "sensor.room_temperature".into(),
            state: value.into(),
            unit_of_measurement: Some("°C".into()),
            device_class: Some("temperature".into()),
            source_last_updated_micros: None,
            received_at_micros,
        }
    }

    fn authenticated_protocol() -> (HomeAssistantReadOnlyProtocol, SessionEpoch, CommandId) {
        let mut protocol = HomeAssistantReadOnlyProtocol::new();
        let epoch = protocol.begin_session().unwrap();
        assert!(matches!(
            protocol.on_auth_required(epoch, "2026.9.0").unwrap(),
            ProtocolOutput::NeedAuthentication { .. }
        ));
        let ProtocolOutput::Send(ReadOnlyCommand::GetStates { id, .. }) =
            protocol.on_auth_ok(epoch).unwrap()
        else {
            panic!("expected get_states")
        };
        (protocol, epoch, id)
    }

    fn subscription_command(outputs: &[ProtocolOutput]) -> CommandId {
        match outputs.get(1) {
            Some(ProtocolOutput::Send(ReadOnlyCommand::SubscribeStateChanged { id, .. })) => *id,
            _ => panic!("expected subscription command"),
        }
    }

    #[test]
    fn orders_auth_bootstrap_and_subscription() {
        let (mut protocol, epoch, get_states_id) = authenticated_protocol();
        let outputs = protocol
            .on_states_result(epoch, get_states_id, true, &[state("21.5", 100)], &profile())
            .unwrap();
        assert!(matches!(outputs.first(), Some(ProtocolOutput::Bootstrap { .. })));
        let subscription_id = subscription_command(&outputs);
        assert_eq!(subscription_id, CommandId(2));
        assert!(matches!(
            protocol
                .on_subscription_result(epoch, subscription_id, true)
                .unwrap(),
            ProtocolOutput::Live { .. }
        ));
    }

    #[test]
    fn command_algebra_is_read_only_and_exact() {
        let get = ReadOnlyCommand::GetStates {
            epoch: SessionEpoch(1),
            id: CommandId(1),
        };
        let subscribe = ReadOnlyCommand::SubscribeStateChanged {
            epoch: SessionEpoch(1),
            id: CommandId(2),
        };
        assert_eq!(get.wire_type(), "get_states");
        assert_eq!(get.event_type(), None);
        assert_eq!(subscribe.wire_type(), "subscribe_events");
        assert_eq!(subscribe.event_type(), Some("state_changed"));
    }

    #[test]
    fn auth_invalid_fails_session() {
        let mut protocol = HomeAssistantReadOnlyProtocol::new();
        let epoch = protocol.begin_session().unwrap();
        protocol.on_auth_required(epoch, "2026.9.0").unwrap();
        assert!(matches!(
            protocol.on_auth_invalid(epoch).unwrap(),
            ProtocolOutput::AuthenticationRejected { .. }
        ));
        assert_eq!(protocol.phase(), &SessionPhase::Failed);
    }

    #[test]
    fn mismatched_result_id_fails_closed() {
        let (mut protocol, epoch, get_states_id) = authenticated_protocol();
        assert!(matches!(
            protocol.on_states_result(
                epoch,
                CommandId(get_states_id.0 + 1),
                true,
                &[],
                &profile()
            ),
            Err(ProtocolError::CorrelationMismatch { .. })
        ));
    }

    #[test]
    fn malformed_allowlisted_bootstrap_fails_session() {
        let (mut protocol, epoch, get_states_id) = authenticated_protocol();
        let mut bad = state("21.5", 100);
        bad.unit_of_measurement = Some("°F".into());
        assert!(matches!(
            protocol.on_states_result(epoch, get_states_id, true, &[bad], &profile()),
            Err(ProtocolError::Observation(_))
        ));
        assert_eq!(protocol.phase(), &SessionPhase::Failed);
    }

    #[test]
    fn events_are_rejected_before_subscription_ack() {
        let (mut protocol, epoch, get_states_id) = authenticated_protocol();
        let outputs = protocol
            .on_states_result(epoch, get_states_id, true, &[], &profile())
            .unwrap();
        let id = subscription_command(&outputs);
        assert!(matches!(
            protocol.on_state_changed(
                epoch,
                id,
                "sensor.room_temperature",
                Some(&state("20.0", 200)),
                200,
                &profile()
            ),
            Err(ProtocolError::UnexpectedPhase { .. })
        ));
    }

    #[test]
    fn reconnect_invalidates_old_epoch() {
        let mut protocol = HomeAssistantReadOnlyProtocol::new();
        let old = protocol.begin_session().unwrap();
        protocol.disconnect(old).unwrap();
        let current = protocol.begin_session().unwrap();
        assert_ne!(old, current);
        assert!(matches!(
            protocol.on_auth_required(old, "2026.9.0"),
            Err(ProtocolError::StaleEpoch { .. })
        ));
    }

    #[test]
    fn bootstrap_passes_only_through_observation_mapper() {
        let (mut protocol, epoch, get_states_id) = authenticated_protocol();
        let mut foreign = state("99.0", 100);
        foreign.entity_id = "sensor.not_allowlisted".into();
        let outputs = protocol
            .on_states_result(
                epoch,
                get_states_id,
                true,
                &[state("21.5", 100), foreign],
                &profile(),
            )
            .unwrap();
        let outcomes = match outputs.first() {
            Some(ProtocolOutput::Bootstrap { outcomes, .. }) => outcomes,
            _ => panic!("expected bootstrap"),
        };
        assert!(matches!(outcomes.first(), Some(MappingOutcome::Observation(_))));
        assert!(matches!(
            outcomes.get(1),
            Some(MappingOutcome::NotAllowlisted { .. })
        ));
    }

    #[test]
    fn unknown_state_remains_epistemic_live_event() {
        let (mut protocol, epoch, get_states_id) = authenticated_protocol();
        let outputs = protocol
            .on_states_result(epoch, get_states_id, true, &[], &profile())
            .unwrap();
        let id = subscription_command(&outputs);
        protocol.on_subscription_result(epoch, id, true).unwrap();
        let output = protocol
            .on_state_changed(
                epoch,
                id,
                "sensor.room_temperature",
                Some(&state("unknown", 500)),
                500,
                &profile(),
            )
            .unwrap();
        assert!(matches!(
            output,
            ProtocolOutput::State {
                outcome: MappingOutcome::SourceState {
                    status: HomeAssistantSourceStatus::Unknown,
                    ..
                },
                ..
            }
        ));
    }

    #[test]
    fn entity_removal_is_explicit_invalidation() {
        let (mut protocol, epoch, get_states_id) = authenticated_protocol();
        let outputs = protocol
            .on_states_result(epoch, get_states_id, true, &[], &profile())
            .unwrap();
        let id = subscription_command(&outputs);
        protocol.on_subscription_result(epoch, id, true).unwrap();
        assert_eq!(
            protocol
                .on_state_changed(
                    epoch,
                    id,
                    "sensor.room_temperature",
                    None,
                    900,
                    &profile(),
                )
                .unwrap(),
            ProtocolOutput::EntityRemoved {
                epoch,
                entity_id: "sensor.room_temperature".into(),
                received_at_micros: 900,
            }
        );
    }

    #[test]
    fn new_state_entity_must_match_event_entity() {
        let (mut protocol, epoch, get_states_id) = authenticated_protocol();
        let outputs = protocol
            .on_states_result(epoch, get_states_id, true, &[], &profile())
            .unwrap();
        let id = subscription_command(&outputs);
        protocol.on_subscription_result(epoch, id, true).unwrap();
        assert!(matches!(
            protocol.on_state_changed(
                epoch,
                id,
                "sensor.other",
                Some(&state("21.0", 1_000)),
                1_000,
                &profile(),
            ),
            Err(ProtocolError::EntityMismatch { .. })
        ));
    }

    #[test]
    fn disconnect_does_not_emit_new_observation() {
        let (mut protocol, epoch, _) = authenticated_protocol();
        assert_eq!(
            protocol.disconnect(epoch).unwrap(),
            ProtocolOutput::Disconnected { epoch }
        );
    }

    #[test]
    fn mapped_live_state_retains_local_freshness() {
        let (mut protocol, epoch, get_states_id) = authenticated_protocol();
        let outputs = protocol
            .on_states_result(epoch, get_states_id, true, &[], &profile())
            .unwrap();
        let id = subscription_command(&outputs);
        protocol.on_subscription_result(epoch, id, true).unwrap();
        let output = protocol
            .on_state_changed(
                epoch,
                id,
                "sensor.room_temperature",
                Some(&state("21.5", 2_000_000)),
                2_000_000,
                &profile(),
            )
            .unwrap();
        let ProtocolOutput::State {
            outcome: MappingOutcome::Observation(mapped),
            ..
        } = output
        else {
            panic!("expected mapped observation")
        };
        assert_eq!(
            mapped.observation.value,
            AutomationValue::Fixed {
                mantissa: 215,
                scale: 1,
            }
        );
        assert_eq!(mapped.observation.observed_at_micros, 2_000_000);
        assert_eq!(mapped.observation.valid_until_micros, Some(32_000_000));
    }
}
