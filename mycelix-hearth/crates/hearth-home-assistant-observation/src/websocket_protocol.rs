// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Pure, read-only state machine for the Home Assistant WebSocket session.
//!
//! This module deliberately knows neither the WebSocket transport nor the
//! credential. Its output algebra can request authentication, request the
//! initial state snapshot, and subscribe to `state_changed`; it has no
//! service-call, state-write, arbitrary-command, URL, or token constructor.
//!
//! The transport must allocate a strictly increasing session epoch for each
//! connection, tag decoded frames with that epoch, and keep credentials outside
//! this state machine. This module is a protocol contract, not a network client.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionPhase {
    AwaitAuthRequired,
    AwaitAuthResult,
    AwaitBootstrapResult,
    AwaitSubscriptionResult,
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
    StateChanged,
    OtherEvent,
    Unexpected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResultPayloadKind {
    StatesArray,
    Null,
    Other,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProtocolError {
    UnexpectedMessage {
        phase: SessionPhase,
        message: MessageKind,
    },
    AuthenticationRejected,
    CommandRejected {
        phase: SessionPhase,
    },
    CommandIdMismatch {
        expected: u64,
        actual: u64,
    },
    WrongResultPayload {
        phase: SessionPhase,
        expected: ResultPayloadKind,
        actual: ResultPayloadKind,
    },
    CommandIdExhausted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProtocolState {
    AwaitAuthRequired,
    AwaitAuthResult,
    AwaitBootstrapResult { command_id: u64 },
    AwaitSubscriptionResult { command_id: u64 },
    Live { subscription_id: u64 },
    Failed(ProtocolError),
    Closed,
}

/// Decoded protocol facts only. Frame parsing and bounded JSON decoding belong
/// to the future transport layer; payload bodies remain with that layer and
/// are forwarded only when the corresponding action is emitted.
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
    StateChanged,
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
            Self::StateChanged => MessageKind::StateChanged,
            Self::OtherEvent => MessageKind::OtherEvent,
            Self::Unexpected => MessageKind::Unexpected,
        }
    }
}

/// Closed set of protocol effects for a native transport to perform.
///
/// There is intentionally no general `SendCommand`, `CallService`, state
/// mutation, or arbitrary message variant. `RequestAuthentication` asks the
/// transport to obtain a local secret and send the protocol auth frame; the
/// secret itself never enters this state machine or its errors/debug output.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProtocolAction {
    RequestAuthentication {
        session_epoch: u64,
    },
    SendGetStates {
        session_epoch: u64,
        command_id: u64,
    },
    BootstrapReady {
        session_epoch: u64,
        command_id: u64,
    },
    SendSubscribeStateChanged {
        session_epoch: u64,
        command_id: u64,
    },
    SessionLive {
        session_epoch: u64,
        subscription_id: u64,
    },
    DeliverStateChanged {
        session_epoch: u64,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HomeAssistantWebSocketProtocol {
    session_epoch: u64,
    next_command_id: Option<u64>,
    state: ProtocolState,
}

impl HomeAssistantWebSocketProtocol {
    /// Start one fresh protocol session. The owner of the transport must
    /// allocate an epoch that differs from every prior connection epoch.
    pub fn new(session_epoch: u64) -> Self {
        Self {
            session_epoch,
            next_command_id: Some(1),
            state: ProtocolState::AwaitAuthRequired,
        }
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
            ProtocolState::AwaitBootstrapResult { .. } => SessionPhase::AwaitBootstrapResult,
            ProtocolState::AwaitSubscriptionResult { .. } => SessionPhase::AwaitSubscriptionResult,
            ProtocolState::Live { .. } => SessionPhase::Live,
            ProtocolState::Failed(_) => SessionPhase::Failed,
            ProtocolState::Closed => SessionPhase::Closed,
        }
    }

    /// Mark a disconnected session terminal. A reconnect must create a new
    /// machine with a new epoch and repeat authentication plus state bootstrap.
    pub fn close(&mut self) {
        if !matches!(self.state, ProtocolState::Failed(_)) {
            self.state = ProtocolState::Closed;
        }
    }

    /// Process a typed inbound protocol message.
    ///
    /// A frame from an older connection epoch is ignored without mutating the
    /// active session. Every unsafe ordering, failed command or mismatched
    /// correlation ID transitions the session to a terminal failed state.
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
                        self.state = ProtocolState::AwaitBootstrapResult { command_id };
                        vec![ProtocolAction::SendGetStates {
                            session_epoch: self.session_epoch,
                            command_id,
                        }]
                    }
                    Err(error) => self.fail(error),
                }
            }
            (ProtocolState::AwaitAuthResult, InboundMessage::AuthInvalid) => {
                self.fail(ProtocolError::AuthenticationRejected)
            }
            (
                ProtocolState::AwaitBootstrapResult { command_id: expected },
                InboundMessage::CommandResult { id, success, payload },
            ) => {
                if id != expected {
                    return self.fail(ProtocolError::CommandIdMismatch {
                        expected,
                        actual: id,
                    });
                }
                if !success {
                    return self.fail(ProtocolError::CommandRejected { phase });
                }
                if payload != ResultPayloadKind::StatesArray {
                    return self.fail(ProtocolError::WrongResultPayload {
                        phase,
                        expected: ResultPayloadKind::StatesArray,
                        actual: payload,
                    });
                }
                let subscription_id = match self.allocate_command_id() {
                    Ok(id) => id,
                    Err(error) => return self.fail(error),
                };
                self.state = ProtocolState::AwaitSubscriptionResult {
                    command_id: subscription_id,
                };
                vec![
                    ProtocolAction::BootstrapReady {
                        session_epoch: self.session_epoch,
                        command_id,
                    },
                    ProtocolAction::SendSubscribeStateChanged {
                        session_epoch: self.session_epoch,
                        command_id: subscription_id,
                    },
                ]
            }
            (
                ProtocolState::AwaitSubscriptionResult { command_id: expected },
                InboundMessage::CommandResult { id, success, payload },
            ) => {
                if id != expected {
                    return self.fail(ProtocolError::CommandIdMismatch {
                        expected,
                        actual: id,
                    });
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
                self.state = ProtocolState::Live {
                    subscription_id: expected,
                };
                vec![ProtocolAction::SessionLive {
                    session_epoch: self.session_epoch,
                    subscription_id: expected,
                }]
            }
            (ProtocolState::Live { .. }, InboundMessage::StateChanged) => {
                vec![ProtocolAction::DeliverStateChanged {
                    session_epoch: self.session_epoch,
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
        let id = self
            .next_command_id
            .ok_or(ProtocolError::CommandIdExhausted)?;
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

    fn send(
        machine: &mut HomeAssistantWebSocketProtocol,
        message: InboundMessage,
    ) -> Vec<ProtocolAction> {
        machine.handle(machine.session_epoch(), message)
    }

    fn reaches_auth_result(machine: &mut HomeAssistantWebSocketProtocol) {
        assert_eq!(
            send(machine, InboundMessage::AuthRequired),
            vec![ProtocolAction::RequestAuthentication { session_epoch: machine.session_epoch() }]
        );
        assert_eq!(machine.state(), ProtocolState::AwaitAuthResult);
    }

    fn reaches_bootstrap_result(machine: &mut HomeAssistantWebSocketProtocol) -> u64 {
        reaches_auth_result(machine);
        assert_eq!(
            send(machine, InboundMessage::AuthOk),
            vec![ProtocolAction::SendGetStates { session_epoch: machine.session_epoch(), command_id: 1 }]
        );
        assert_eq!(machine.state(), ProtocolState::AwaitBootstrapResult { command_id: 1 });
        1
    }

    fn reaches_subscription_result(machine: &mut HomeAssistantWebSocketProtocol) -> u64 {
        let bootstrap_id = reaches_bootstrap_result(machine);
        assert_eq!(
            send(machine, InboundMessage::CommandResult {
                id: bootstrap_id,
                success: true,
                payload: ResultPayloadKind::StatesArray,
            }),
            vec![
                ProtocolAction::BootstrapReady {
                    session_epoch: machine.session_epoch(),
                    command_id: bootstrap_id,
                },
                ProtocolAction::SendSubscribeStateChanged {
                    session_epoch: machine.session_epoch(),
                    command_id: 2,
                },
            ]
        );
        assert_eq!(machine.state(), ProtocolState::AwaitSubscriptionResult { command_id: 2 });
        2
    }

    fn reaches_live(machine: &mut HomeAssistantWebSocketProtocol) -> u64 {
        let subscription_id = reaches_subscription_result(machine);
        assert_eq!(
            send(machine, InboundMessage::CommandResult {
                id: subscription_id,
                success: true,
                payload: ResultPayloadKind::Null,
            }),
            vec![ProtocolAction::SessionLive {
                session_epoch: machine.session_epoch(),
                subscription_id,
            }]
        );
        assert_eq!(machine.state(), ProtocolState::Live { subscription_id });
        subscription_id
    }

    #[test]
    fn requires_auth_bootstrap_and_subscription_in_exact_order() {
        let mut machine = HomeAssistantWebSocketProtocol::new(7);
        assert_eq!(machine.phase(), SessionPhase::AwaitAuthRequired);
        reaches_live(&mut machine);
        assert_eq!(
            send(&mut machine, InboundMessage::StateChanged),
            vec![ProtocolAction::DeliverStateChanged { session_epoch: 7 }]
        );
    }

    #[test]
    fn auth_invalid_fails_closed() {
        let mut machine = HomeAssistantWebSocketProtocol::new(1);
        reaches_auth_result(&mut machine);
        assert_eq!(
            send(&mut machine, InboundMessage::AuthInvalid),
            vec![ProtocolAction::FailClosed {
                session_epoch: 1,
                error: ProtocolError::AuthenticationRejected,
            }]
        );
        assert!(matches!(machine.state(), ProtocolState::Failed(_)));
    }

    #[test]
    fn auth_ok_before_auth_required_is_rejected() {
        let mut machine = HomeAssistantWebSocketProtocol::new(1);
        assert!(matches!(
            send(&mut machine, InboundMessage::AuthOk).as_slice(),
            [ProtocolAction::FailClosed {
                error: ProtocolError::UnexpectedMessage { .. },
                ..
            }]
        ));
        assert!(matches!(machine.state(), ProtocolState::Failed(_)));
    }

    #[test]
    fn bootstrap_result_id_mismatch_fails_closed() {
        let mut machine = HomeAssistantWebSocketProtocol::new(1);
        reaches_bootstrap_result(&mut machine);
        let result = send(&mut machine, InboundMessage::CommandResult {
            id: 99,
            success: true,
            payload: ResultPayloadKind::StatesArray,
        });
        assert_eq!(
            result,
            vec![ProtocolAction::FailClosed {
                session_epoch: 1,
                error: ProtocolError::CommandIdMismatch { expected: 1, actual: 99 },
            }]
        );
    }

    #[test]
    fn non_array_bootstrap_payload_fails_closed() {
        let mut machine = HomeAssistantWebSocketProtocol::new(1);
        let id = reaches_bootstrap_result(&mut machine);
        assert!(matches!(
            send(&mut machine, InboundMessage::CommandResult {
                id,
                success: true,
                payload: ResultPayloadKind::Null,
            }).as_slice(),
            [ProtocolAction::FailClosed {
                error: ProtocolError::WrongResultPayload {
                    expected: ResultPayloadKind::StatesArray,
                    actual: ResultPayloadKind::Null,
                    ..
                },
                ..
            }]
        ));
    }

    #[test]
    fn failed_bootstrap_command_fails_closed() {
        let mut machine = HomeAssistantWebSocketProtocol::new(1);
        let id = reaches_bootstrap_result(&mut machine);
        assert!(matches!(
            send(&mut machine, InboundMessage::CommandResult {
                id,
                success: false,
                payload: ResultPayloadKind::Other,
            }).as_slice(),
            [ProtocolAction::FailClosed {
                error: ProtocolError::CommandRejected {
                    phase: SessionPhase::AwaitBootstrapResult,
                },
                ..
            }]
        ));
    }

    #[test]
    fn state_changed_before_subscription_ack_is_rejected() {
        let mut machine = HomeAssistantWebSocketProtocol::new(1);
        reaches_subscription_result(&mut machine);
        assert!(matches!(
            send(&mut machine, InboundMessage::StateChanged).as_slice(),
            [ProtocolAction::FailClosed {
                error: ProtocolError::UnexpectedMessage {
                    phase: SessionPhase::AwaitSubscriptionResult,
                    message: MessageKind::StateChanged,
                },
                ..
            }]
        ));
    }

    #[test]
    fn subscription_ack_requires_matching_id_success_and_null_payload() {
        let mut wrong_id = HomeAssistantWebSocketProtocol::new(1);
        let id = reaches_subscription_result(&mut wrong_id);
        assert!(matches!(
            send(&mut wrong_id, InboundMessage::CommandResult {
                id: id + 1,
                success: true,
                payload: ResultPayloadKind::Null,
            }).as_slice(),
            [ProtocolAction::FailClosed {
                error: ProtocolError::CommandIdMismatch { .. },
                ..
            }]
        ));

        let mut failed = HomeAssistantWebSocketProtocol::new(2);
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

        let mut wrong_payload = HomeAssistantWebSocketProtocol::new(3);
        let id = reaches_subscription_result(&mut wrong_payload);
        assert!(matches!(
            send(&mut wrong_payload, InboundMessage::CommandResult {
                id,
                success: true,
                payload: ResultPayloadKind::StatesArray,
            }).as_slice(),
            [ProtocolAction::FailClosed {
                error: ProtocolError::WrongResultPayload {
                    expected: ResultPayloadKind::Null,
                    actual: ResultPayloadKind::StatesArray,
                    ..
                },
                ..
            }]
        ));
    }

    #[test]
    fn reconnect_epoch_rejects_old_session_frames_without_changing_new_state() {
        let mut machine = HomeAssistantWebSocketProtocol::new(42);
        let result = machine.handle(41, InboundMessage::AuthRequired);
        assert_eq!(
            result,
            vec![ProtocolAction::IgnoreStaleEpoch {
                received_epoch: 41,
                active_epoch: 42,
            }]
        );
        assert_eq!(machine.state(), ProtocolState::AwaitAuthRequired);
    }

    #[test]
    fn reconnect_requires_a_new_full_bootstrap_and_fresh_command_ids() {
        let mut prior = HomeAssistantWebSocketProtocol::new(10);
        reaches_live(&mut prior);
        prior.close();
        assert_eq!(prior.state(), ProtocolState::Closed);

        let mut current = HomeAssistantWebSocketProtocol::new(11);
        assert_eq!(current.state(), ProtocolState::AwaitAuthRequired);
        assert_eq!(
            send(&mut current, InboundMessage::AuthRequired),
            vec![ProtocolAction::RequestAuthentication { session_epoch: 11 }]
        );
        assert_eq!(
            prior.handle(10, InboundMessage::StateChanged),
            vec![ProtocolAction::IgnoreTerminalMessage { session_epoch: 10 }]
        );
        assert_eq!(current.state(), ProtocolState::AwaitAuthResult);
    }

    #[test]
    fn late_old_command_result_cannot_advance_a_new_session() {
        let mut machine = HomeAssistantWebSocketProtocol::new(52);
        let result = machine.handle(51, InboundMessage::CommandResult {
            id: 1,
            success: true,
            payload: ResultPayloadKind::StatesArray,
        });
        assert!(matches!(result.as_slice(), [ProtocolAction::IgnoreStaleEpoch { .. }]));
        assert_eq!(machine.state(), ProtocolState::AwaitAuthRequired);
    }

    #[test]
    fn unrelated_live_events_are_ignored_without_becoming_observations() {
        let mut machine = HomeAssistantWebSocketProtocol::new(9);
        reaches_live(&mut machine);
        assert_eq!(
            send(&mut machine, InboundMessage::OtherEvent),
            vec![ProtocolAction::IgnoreUnrelatedEvent { session_epoch: 9 }]
        );
        assert_eq!(machine.state(), ProtocolState::Live { subscription_id: 2 });
    }

    #[test]
    fn terminal_failed_session_never_emits_new_protocol_actions() {
        let mut machine = HomeAssistantWebSocketProtocol::new(1);
        send(&mut machine, InboundMessage::AuthOk);
        let first_error = machine.state();
        assert!(matches!(first_error, ProtocolState::Failed(_)));
        assert_eq!(
            send(&mut machine, InboundMessage::AuthRequired),
            vec![ProtocolAction::IgnoreTerminalMessage { session_epoch: 1 }]
        );
    }

    #[test]
    fn command_id_exhaustion_is_a_fail_closed_error() {
        let mut machine = HomeAssistantWebSocketProtocol::new(1);
        machine.next_command_id = None;
        reaches_auth_result(&mut machine);
        assert_eq!(
            send(&mut machine, InboundMessage::AuthOk),
            vec![ProtocolAction::FailClosed {
                session_epoch: 1,
                error: ProtocolError::CommandIdExhausted,
            }]
        );
    }

    #[test]
    fn protocol_debug_types_have_no_credential_field() {
        let message = InboundMessage::AuthRequired;
        let action = ProtocolAction::RequestAuthentication { session_epoch: 1 };
        assert_eq!(format!("{message:?}"), "AuthRequired");
        assert_eq!(
            format!("{action:?}"),
            "RequestAuthentication { session_epoch: 1 }"
        );
    }
}
