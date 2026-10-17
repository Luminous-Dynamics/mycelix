// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Pure, read-only state machine for the Home Assistant WebSocket session.
//!
//! The transport must subscribe to state_changed before requesting the initial
//! snapshot. Events arriving after subscription acknowledgement but before the
//! snapshot result are buffered by the transport. Actions then require the
//! snapshot to be applied first and those events to be replayed in receive order.
//!
//! This module deliberately knows neither the WebSocket transport nor the
//! credential. Its output algebra has no service-call, state-write, arbitrary-
//! command, URL, or token constructor.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionPhase {
    AwaitAuthRequired,
    AwaitAuthResult,
    AwaitSubscriptionResult,
    AwaitBootstrapResult,
    AwaitBootstrapApply,
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
    SubscriptionIdMismatch {
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
    AwaitSubscriptionResult {
        command_id: u64,
    },
    AwaitBootstrapResult {
        command_id: u64,
        subscription_id: u64,
    },
    AwaitBootstrapApply {
        subscription_id: u64,
    },
    Live {
        subscription_id: u64,
    },
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
    /// HA event frame's outer `id` is the ID of the active subscription.
    StateChanged {
        subscription_id: u64,
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
            Self::StateChanged { .. } => MessageKind::StateChanged,
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
///
/// `BufferStateChanged` tells the transport to retain that event's raw payload
/// under the exact epoch/subscription pair. On bootstrap completion it must
/// apply `BootstrapReady`'s snapshot first, then drain buffered events in their
/// original receive order on `DrainBufferedStateChanges`, and only then mark
/// the observation stream live.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProtocolAction {
    RequestAuthentication {
        session_epoch: u64,
    },
    SendSubscribeStateChanged {
        session_epoch: u64,
        command_id: u64,
    },
    SendGetStates {
        session_epoch: u64,
        command_id: u64,
    },
    BufferStateChanged {
        session_epoch: u64,
        subscription_id: u64,
    },
    BootstrapReady {
        session_epoch: u64,
        command_id: u64,
    },
    DrainBufferedStateChanges {
        session_epoch: u64,
        subscription_id: u64,
    },
    SessionLive {
        session_epoch: u64,
        subscription_id: u64,
    },
    DeliverStateChanged {
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
            ProtocolState::AwaitSubscriptionResult { .. } => SessionPhase::AwaitSubscriptionResult,
            ProtocolState::AwaitBootstrapResult { .. } => SessionPhase::AwaitBootstrapResult,
            ProtocolState::AwaitBootstrapApply { .. } => SessionPhase::AwaitBootstrapApply,
            ProtocolState::Live { .. } => SessionPhase::Live,
            ProtocolState::Failed(_) => SessionPhase::Failed,
            ProtocolState::Closed => SessionPhase::Closed,
        }
    }

    /// Mark a disconnected session terminal. A reconnect must create a new
    /// machine with a new epoch and repeat authentication/subscription/bootstrap.
    pub fn close(&mut self) {
        if !matches!(self.state, ProtocolState::Failed(_)) {
            self.state = ProtocolState::Closed;
        }
    }

    /// Mark the snapshot and buffered events fully applied after the transport
    /// has executed the ordered bootstrap actions. Calling this is the only way
    /// to enter Live, so the state machine does not declare live before the
    /// snapshot is applied and the pre-snapshot event queue is drained.
    pub fn bootstrap_applied(
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
            ProtocolState::AwaitBootstrapApply { subscription_id: expected } => {
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
                        self.state = ProtocolState::AwaitSubscriptionResult { command_id };
                        vec![ProtocolAction::SendSubscribeStateChanged {
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
                let bootstrap_id = match self.allocate_command_id() {
                    Ok(id) => id,
                    Err(error) => return self.fail(error),
                };
                self.state = ProtocolState::AwaitBootstrapResult {
                    command_id: bootstrap_id,
                    subscription_id: expected,
                };
                vec![ProtocolAction::SendGetStates {
                    session_epoch: self.session_epoch,
                    command_id: bootstrap_id,
                }]
            }
            (
                ProtocolState::AwaitBootstrapResult {
                    command_id: expected,
                    subscription_id,
                },
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
                self.state = ProtocolState::AwaitBootstrapApply { subscription_id };
                vec![
                    ProtocolAction::BootstrapReady {
                        session_epoch: self.session_epoch,
                        command_id: expected,
                    },
                    ProtocolAction::DrainBufferedStateChanges {
                        session_epoch: self.session_epoch,
                        subscription_id,
                    },
                ]
            }
            (
                ProtocolState::AwaitBootstrapResult { subscription_id, .. }
                | ProtocolState::AwaitBootstrapApply { subscription_id },
                InboundMessage::StateChanged { subscription_id: received },
            ) => {
                if received != subscription_id {
                    return self.fail(ProtocolError::SubscriptionIdMismatch {
                        expected: subscription_id,
                        actual: received,
                    });
                }
                vec![ProtocolAction::BufferStateChanged {
                    session_epoch: self.session_epoch,
                    subscription_id,
                }]
            }
            (
                ProtocolState::Live { subscription_id: expected },
                InboundMessage::StateChanged { subscription_id: received },
            ) => {
                if received != expected {
                    return self.fail(ProtocolError::SubscriptionIdMismatch {
                        expected,
                        actual: received,
                    });
                }
                vec![ProtocolAction::DeliverStateChanged {
                    session_epoch: self.session_epoch,
                    subscription_id: expected,
                }]
            }
            (ProtocolState::Live { .. } | ProtocolState::AwaitBootstrapResult { .. } | ProtocolState::AwaitBootstrapApply { .. }, InboundMessage::OtherEvent) => {
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
            vec![ProtocolAction::SendSubscribeStateChanged {
                session_epoch: epoch,
                command_id: 1,
            }]
        );
        assert_eq!(
            machine.state(),
            ProtocolState::AwaitSubscriptionResult { command_id: 1 }
        );
        1
    }

    fn reaches_bootstrap_result(machine: &mut HomeAssistantWebSocketProtocol) -> (u64, u64) {
        let subscription_id = reaches_subscription_result(machine);
        let epoch = machine.session_epoch();
        assert_eq!(
            send(machine, InboundMessage::CommandResult {
                id: subscription_id,
                success: true,
                payload: ResultPayloadKind::Null,
            }),
            vec![ProtocolAction::SendGetStates {
                session_epoch: epoch,
                command_id: 2,
            }]
        );
        assert_eq!(
            machine.state(),
            ProtocolState::AwaitBootstrapResult {
                command_id: 2,
                subscription_id,
            }
        );
        (2, subscription_id)
    }

    fn reaches_live(machine: &mut HomeAssistantWebSocketProtocol) -> u64 {
        let (bootstrap_id, subscription_id) = reaches_bootstrap_result(machine);
        let epoch = machine.session_epoch();
        assert_eq!(
            send(machine, InboundMessage::CommandResult {
                id: bootstrap_id,
                success: true,
                payload: ResultPayloadKind::StatesArray,
            }),
            vec![
                ProtocolAction::BootstrapReady {
                    session_epoch: epoch,
                    command_id: bootstrap_id,
                },
                ProtocolAction::DrainBufferedStateChanges {
                    session_epoch: epoch,
                    subscription_id,
                },
                ProtocolAction::SessionLive {
                    session_epoch: epoch,
                    subscription_id,
                },
            ]
        );
        assert_eq!(machine.state(), ProtocolState::AwaitBootstrapApply { subscription_id });
        assert_eq!(
            machine.bootstrap_applied(machine.session_epoch(), subscription_id),
            vec![ProtocolAction::SessionLive {
                session_epoch: machine.session_epoch(),
                subscription_id,
            }]
        );
        assert_eq!(machine.state(), ProtocolState::Live { subscription_id });
        subscription_id
    }

    #[test]
    fn requires_auth_subscription_then_bootstrap_in_exact_order() {
        let mut machine = HomeAssistantWebSocketProtocol::new(7);
        assert_eq!(machine.phase(), SessionPhase::AwaitAuthRequired);
        let subscription_id = reaches_live(&mut machine);
        assert_eq!(
            send(&mut machine, InboundMessage::StateChanged { subscription_id }),
            vec![ProtocolAction::DeliverStateChanged {
                session_epoch: 7,
                subscription_id,
            }]
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
    fn subscription_result_id_mismatch_fails_closed() {
        let mut machine = HomeAssistantWebSocketProtocol::new(1);
        reaches_subscription_result(&mut machine);
        assert_eq!(
            send(&mut machine, InboundMessage::CommandResult {
                id: 99,
                success: true,
                payload: ResultPayloadKind::Null,
            }),
            vec![ProtocolAction::FailClosed {
                session_epoch: 1,
                error: ProtocolError::CommandIdMismatch { expected: 1, actual: 99 },
            }]
        );
    }

    #[test]
    fn subscription_ack_requires_success_and_null_payload() {
        let mut failed = HomeAssistantWebSocketProtocol::new(1);
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
    fn bootstrap_result_id_mismatch_fails_closed() {
        let mut machine = HomeAssistantWebSocketProtocol::new(1);
        reaches_bootstrap_result(&mut machine);
        assert_eq!(
            send(&mut machine, InboundMessage::CommandResult {
                id: 99,
                success: true,
                payload: ResultPayloadKind::StatesArray,
            }),
            vec![ProtocolAction::FailClosed {
                session_epoch: 1,
                error: ProtocolError::CommandIdMismatch { expected: 2, actual: 99 },
            }]
        );
    }

    #[test]
    fn non_array_bootstrap_payload_fails_closed() {
        let mut machine = HomeAssistantWebSocketProtocol::new(1);
        let (id, _) = reaches_bootstrap_result(&mut machine);
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
        let (id, _) = reaches_bootstrap_result(&mut machine);
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
        let subscription_id = reaches_subscription_result(&mut machine);
        assert!(matches!(
            send(&mut machine, InboundMessage::StateChanged { subscription_id }).as_slice(),
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
    fn bootstrap_events_are_buffered_then_drained_after_snapshot() {
        let mut machine = HomeAssistantWebSocketProtocol::new(1);
        let (bootstrap_id, subscription_id) = reaches_bootstrap_result(&mut machine);
        assert_eq!(
            send(&mut machine, InboundMessage::StateChanged { subscription_id }),
            vec![ProtocolAction::BufferStateChanged {
                session_epoch: 1,
                subscription_id,
            }]
        );

        assert_eq!(
            send(&mut machine, InboundMessage::CommandResult {
                id: bootstrap_id,
                success: true,
                payload: ResultPayloadKind::StatesArray,
            }),
            vec![
                ProtocolAction::BootstrapReady {
                    session_epoch: 1,
                    command_id: bootstrap_id,
                },
                ProtocolAction::DrainBufferedStateChanges {
                    session_epoch: 1,
                    subscription_id,
                },
            ]
        );
        assert_eq!(machine.state(), ProtocolState::AwaitBootstrapApply { subscription_id });
        assert_eq!(
            machine.bootstrap_applied(1, subscription_id),
            vec![ProtocolAction::SessionLive {
                session_epoch: 1,
                subscription_id,
            }]
        );
    }

    #[test]
    fn wrong_subscription_id_fails_closed_before_and_after_bootstrap() {
        let mut bootstrapping = HomeAssistantWebSocketProtocol::new(1);
        reaches_bootstrap_result(&mut bootstrapping);
        assert!(matches!(
            send(&mut bootstrapping, InboundMessage::StateChanged { subscription_id: 77 }).as_slice(),
            [ProtocolAction::FailClosed {
                error: ProtocolError::SubscriptionIdMismatch { expected: 1, actual: 77 },
                ..
            }]
        ));

        let mut live = HomeAssistantWebSocketProtocol::new(2);
        reaches_live(&mut live);
        assert!(matches!(
            send(&mut live, InboundMessage::StateChanged { subscription_id: 77 }).as_slice(),
            [ProtocolAction::FailClosed {
                error: ProtocolError::SubscriptionIdMismatch { expected: 1, actual: 77 },
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
    fn reconnect_requires_a_new_full_bootstrap_and_old_session_is_terminal() {
        let mut prior = HomeAssistantWebSocketProtocol::new(10);
        let old_subscription = reaches_live(&mut prior);
        prior.close();
        assert_eq!(prior.state(), ProtocolState::Closed);

        let mut current = HomeAssistantWebSocketProtocol::new(11);
        assert_eq!(current.state(), ProtocolState::AwaitAuthRequired);
        assert_eq!(
            send(&mut current, InboundMessage::AuthRequired),
            vec![ProtocolAction::RequestAuthentication { session_epoch: 11 }]
        );
        assert_eq!(
            prior.handle(10, InboundMessage::StateChanged { subscription_id: old_subscription }),
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
    fn unrelated_events_are_ignored_without_becoming_observations() {
        let mut machine = HomeAssistantWebSocketProtocol::new(9);
        reaches_live(&mut machine);
        assert_eq!(
            send(&mut machine, InboundMessage::OtherEvent),
            vec![ProtocolAction::IgnoreUnrelatedEvent { session_epoch: 9 }]
        );
        assert_eq!(machine.state(), ProtocolState::Live { subscription_id: 1 });
    }

    #[test]
    fn terminal_failed_session_never_emits_new_protocol_actions() {
        let mut machine = HomeAssistantWebSocketProtocol::new(1);
        send(&mut machine, InboundMessage::AuthOk);
        assert!(matches!(machine.state(), ProtocolState::Failed(_)));
        assert_eq!(
            send(&mut machine, InboundMessage::AuthRequired),
            vec![ProtocolAction::IgnoreTerminalMessage { session_epoch: 1 }]
        );
    }

    #[test]
    fn command_id_exhaustion_is_a_fail_closed_error() {
        let mut machine = HomeAssistantWebSocketProtocol::new(1);
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
    fn authentication_request_action_contains_no_credential_value() {
        let message = InboundMessage::AuthRequired;
        let action = ProtocolAction::RequestAuthentication { session_epoch: 1 };
        assert_eq!(format!("{message:?}"), "AuthRequired");
        assert_eq!(
            format!("{action:?}"),
            "RequestAuthentication { session_epoch: 1 }"
        );
    }
}
