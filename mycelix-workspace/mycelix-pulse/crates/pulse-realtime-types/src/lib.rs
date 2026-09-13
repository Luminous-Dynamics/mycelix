// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Transport-neutral realtime contracts for Mycelix Pulse.
//!
//! These messages are deliberately **hints, not authority**. A receiver may
//! use them to schedule reconciliation, but MUST NOT construct durable message,
//! delivery, receipt, verification, or authorization state from a realtime hint.
//!
//! Correctness requirements:
//!
//! - dropping a hint must be safe;
//! - duplicate hints must be safe;
//! - reordering hints must be safe;
//! - hints carry no message plaintext/ciphertext or delivery claim;
//! - unknown envelope fields fail closed rather than extending hint authority;
//! - oversized or malformed wire payloads fail before reconciliation is scheduled;
//! - durable state is recovered from the authoritative Holochain/DHT source;
//! - an arbitrary burst of hints during one reconciliation pass coalesces into
//!   at most one immediate follow-up pass.

use serde::{Deserialize, Serialize};

/// Current wire version for [`PulseRealtimeHintV1`].
pub const PULSE_REALTIME_HINT_V1: u16 = 1;

/// Maximum accepted encoded size for a realtime hint.
///
/// V1 carries only a version and reconciliation scope, so 1 KiB leaves ample
/// encoding headroom while bounding work before any deserializer sees the input.
pub const MAX_REALTIME_HINT_BYTES: usize = 1024;

/// Versioned envelope for non-authoritative Pulse realtime hints.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PulseRealtimeHintV1 {
    pub version: u16,
    pub hint: DurableWakeHintV1,
}

impl PulseRealtimeHintV1 {
    /// Wake a receiver so it can reconcile its authoritative V2 inbox.
    ///
    /// This does not assert that a specific message exists, was delivered,
    /// was observed, was read, or passed any verification procedure.
    pub const fn inbox_changed_v2() -> Self {
        Self {
            version: PULSE_REALTIME_HINT_V1,
            hint: DurableWakeHintV1::InboxChangedV2,
        }
    }

    pub fn validate(&self) -> Result<(), RealtimeContractError> {
        if self.version != PULSE_REALTIME_HINT_V1 {
            return Err(RealtimeContractError::UnsupportedVersion(self.version));
        }
        Ok(())
    }
}

/// Durable-state wakeups. These variants name only the reconciliation scope.
/// They intentionally do not carry the state being reconciled.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DurableWakeHintV1 {
    /// The receiver should re-query its V2 inbox authority.
    InboxChangedV2,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RealtimeContractError {
    UnsupportedVersion(u16),
}

/// Failure to admit and validate a realtime wire payload.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RealtimeDecodeError {
    /// Empty transport frames carry no reconciliation request.
    Empty,
    /// Reject before deserialization so attacker-controlled frames cannot make
    /// decoder work scale with arbitrary payload size.
    TooLarge { actual: usize, max: usize },
    /// The bytes are neither an exact MessagePack nor JSON V1 envelope.
    Malformed,
    /// The envelope decoded exactly but failed the versioned contract.
    Contract(RealtimeContractError),
}

/// Decode one bounded, exact realtime hint.
///
/// MessagePack is attempted first because Holochain signal transports normally
/// use MessagePack-shaped serialized bytes. JSON remains an explicit browser /
/// test compatibility encoding. Both paths deserialize the same strict type,
/// including `deny_unknown_fields`, and then run version validation.
///
/// This function only admits a scheduling hint. Successful decode says nothing
/// about durable message existence, delivery, read state, sender trust, or
/// cryptographic verification.
pub fn decode_realtime_hint(bytes: &[u8]) -> Result<PulseRealtimeHintV1, RealtimeDecodeError> {
    if bytes.is_empty() {
        return Err(RealtimeDecodeError::Empty);
    }
    if bytes.len() > MAX_REALTIME_HINT_BYTES {
        return Err(RealtimeDecodeError::TooLarge {
            actual: bytes.len(),
            max: MAX_REALTIME_HINT_BYTES,
        });
    }

    let hint = rmp_serde::from_slice::<PulseRealtimeHintV1>(bytes)
        .or_else(|_| serde_json::from_slice::<PulseRealtimeHintV1>(bytes))
        .map_err(|_| RealtimeDecodeError::Malformed)?;
    hint.validate().map_err(RealtimeDecodeError::Contract)?;
    Ok(hint)
}

/// A transport-neutral, single-owner reconciliation reducer.
///
/// The reducer carries **scheduling metadata only**. It never contains a
/// message identifier, thread identifier, receipt state, verification result,
/// or any other durable authority.
///
/// Callers MUST serialize access to one reducer instance. Under that single-
/// owner rule, every accepted wake is linearized either while a pass is
/// running (and therefore marks that pass dirty) or while idle (and therefore
/// starts a new pass). There is no accepted-but-uncovered wake state.
///
/// The reducer is intentionally neither `Clone` nor `Copy`: duplicating it
/// would create multiple scheduling authorities and invalidate that theorem.
#[derive(Debug, Default, PartialEq, Eq)]
pub struct ReconcileScheduler {
    state: ReconcileState,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum ReconcileState {
    #[default]
    Idle,
    Running { dirty: bool },
}

/// Effect produced when authoritative reconciliation is requested.
#[must_use = "reconciliation request effects must be acted on or a wake can be lost"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReconcileRequestEffect {
    /// The caller owns responsibility for starting one authoritative pass.
    StartPass,
    /// A pass is already running; this wake was coalesced into one dirty bit.
    Coalesced,
}

/// Effect produced when an authoritative reconciliation pass completes.
#[must_use = "reconciliation completion effects must be acted on to preserve wake coverage"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReconcileCompletionEffect {
    /// One or more wakes arrived during the completed pass. Start exactly one
    /// immediate follow-up pass; all of those wakes are represented by it.
    StartFollowUpPass,
    /// No wake arrived during the completed pass. The reducer is quiescent.
    BecameIdle,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReconcileSchedulerError {
    /// Completion was reported when no pass was running.
    CompletionWhileIdle,
}

impl ReconcileScheduler {
    pub const fn new() -> Self {
        Self {
            state: ReconcileState::Idle,
        }
    }

    /// Request reconciliation after a validated realtime hint, startup, or
    /// reconnect event.
    ///
    /// This function only schedules work. It cannot establish any message or
    /// receipt fact by construction.
    pub fn request_reconcile(&mut self) -> ReconcileRequestEffect {
        match self.state {
            ReconcileState::Idle => {
                self.state = ReconcileState::Running { dirty: false };
                ReconcileRequestEffect::StartPass
            }
            ReconcileState::Running { .. } => {
                self.state = ReconcileState::Running { dirty: true };
                ReconcileRequestEffect::Coalesced
            }
        }
    }

    /// Report completion of exactly one authoritative pass.
    ///
    /// If any number of wakes arrived during that pass, they collapse into one
    /// follow-up pass. Otherwise the reducer becomes idle.
    pub fn complete_pass(&mut self) -> Result<ReconcileCompletionEffect, ReconcileSchedulerError> {
        match self.state {
            ReconcileState::Idle => Err(ReconcileSchedulerError::CompletionWhileIdle),
            ReconcileState::Running { dirty: true } => {
                self.state = ReconcileState::Running { dirty: false };
                Ok(ReconcileCompletionEffect::StartFollowUpPass)
            }
            ReconcileState::Running { dirty: false } => {
                self.state = ReconcileState::Idle;
                Ok(ReconcileCompletionEffect::BecameIdle)
            }
        }
    }

    pub const fn is_running(&self) -> bool {
        matches!(self.state, ReconcileState::Running { .. })
    }

    pub const fn is_dirty(&self) -> bool {
        matches!(self.state, ReconcileState::Running { dirty: true })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn v2_inbox_wake_has_no_state_payload() {
        let hint = PulseRealtimeHintV1::inbox_changed_v2();
        let json = serde_json::to_string(&hint).expect("serialize wake hint");

        assert_eq!(hint.version, PULSE_REALTIME_HINT_V1);
        assert_eq!(hint.hint, DurableWakeHintV1::InboxChangedV2);
        assert!(json.contains("inbox_changed_v2"));

        for forbidden in [
            "subject",
            "body",
            "ciphertext",
            "delivered",
            "verified",
            "authorized",
            "read_receipt",
        ] {
            assert!(
                !json.contains(forbidden),
                "wake hint must not carry or imply {forbidden}: {json}"
            );
        }
    }

    #[test]
    fn round_trip_preserves_only_reconciliation_scope() {
        let original = PulseRealtimeHintV1::inbox_changed_v2();
        let bytes = serde_json::to_vec(&original).expect("serialize wake hint");
        let decoded: PulseRealtimeHintV1 =
            serde_json::from_slice(&bytes).expect("deserialize wake hint");

        assert_eq!(decoded, original);
        assert_eq!(decoded.validate(), Ok(()));
    }

    #[test]
    fn bounded_decoder_accepts_exact_json_and_messagepack() {
        let original = PulseRealtimeHintV1::inbox_changed_v2();
        let json = serde_json::to_vec(&original).expect("serialize JSON wake");
        let messagepack = rmp_serde::to_vec_named(&original).expect("serialize MessagePack wake");

        assert_eq!(decode_realtime_hint(&json), Ok(original.clone()));
        assert_eq!(decode_realtime_hint(&messagepack), Ok(original));
    }

    #[test]
    fn bounded_decoder_rejects_empty_before_deserialization() {
        assert_eq!(decode_realtime_hint(&[]), Err(RealtimeDecodeError::Empty));
    }

    #[test]
    fn bounded_decoder_rejects_oversized_before_deserialization() {
        let oversized = vec![0u8; MAX_REALTIME_HINT_BYTES + 1];
        assert_eq!(
            decode_realtime_hint(&oversized),
            Err(RealtimeDecodeError::TooLarge {
                actual: MAX_REALTIME_HINT_BYTES + 1,
                max: MAX_REALTIME_HINT_BYTES,
            })
        );
    }

    #[test]
    fn bounded_decoder_rejects_malformed_and_unknown_fields() {
        assert_eq!(
            decode_realtime_hint(b"not a realtime envelope"),
            Err(RealtimeDecodeError::Malformed)
        );

        let with_authority = serde_json::to_vec(&json!({
            "version": PULSE_REALTIME_HINT_V1,
            "hint": "inbox_changed_v2",
            "delivered": true,
        }))
        .expect("serialize stronger envelope");
        assert_eq!(
            decode_realtime_hint(&with_authority),
            Err(RealtimeDecodeError::Malformed)
        );
    }

    #[test]
    fn bounded_decoder_preserves_version_failure() {
        let unsupported = serde_json::to_vec(&json!({
            "version": PULSE_REALTIME_HINT_V1 + 1,
            "hint": "inbox_changed_v2",
        }))
        .expect("serialize unsupported wake");

        assert_eq!(
            decode_realtime_hint(&unsupported),
            Err(RealtimeDecodeError::Contract(
                RealtimeContractError::UnsupportedVersion(PULSE_REALTIME_HINT_V1 + 1)
            ))
        );
    }

    #[test]
    fn unsupported_wire_version_fails_closed() {
        let mut hint = PulseRealtimeHintV1::inbox_changed_v2();
        hint.version += 1;

        assert_eq!(
            hint.validate(),
            Err(RealtimeContractError::UnsupportedVersion(
                PULSE_REALTIME_HINT_V1 + 1
            ))
        );
    }

    #[test]
    fn stronger_unknown_fields_fail_closed() {
        for (field, injected_value) in [
            ("subject", json!("pretend subject")),
            ("delivered", json!(true)),
            ("thread_id", json!("pretend-thread")),
            ("ciphertext", json!("pretend-ciphertext")),
            ("verified", json!(true)),
            ("authorized", json!(true)),
        ] {
            let mut envelope = json!({
                "version": PULSE_REALTIME_HINT_V1,
                "hint": "inbox_changed_v2",
            });
            envelope
                .as_object_mut()
                .expect("wake fixture is an object")
                .insert(field.to_string(), injected_value);

            let error = serde_json::from_value::<PulseRealtimeHintV1>(envelope)
                .expect_err("stronger realtime fields must fail closed");
            assert!(
                error.to_string().contains("unknown field"),
                "unexpected decode error for {field}: {error}"
            );
        }
    }

    #[test]
    fn unknown_wake_scope_fails_closed() {
        let value = json!({
            "version": PULSE_REALTIME_HINT_V1,
            "hint": "message_delivered",
        });

        serde_json::from_value::<PulseRealtimeHintV1>(value)
            .expect_err("unknown wake scopes must not acquire authority by default");
    }

    #[test]
    fn first_request_starts_exactly_one_pass() {
        let mut scheduler = ReconcileScheduler::new();

        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::StartPass
        );
        assert!(scheduler.is_running());
        assert!(!scheduler.is_dirty());
    }

    #[test]
    fn arbitrary_wake_burst_collapses_to_one_dirty_bit() {
        let mut scheduler = ReconcileScheduler::new();
        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::StartPass
        );

        for _ in 0..10_000 {
            assert_eq!(
                scheduler.request_reconcile(),
                ReconcileRequestEffect::Coalesced
            );
        }

        assert!(scheduler.is_running());
        assert!(scheduler.is_dirty());
        assert_eq!(
            scheduler.complete_pass(),
            Ok(ReconcileCompletionEffect::StartFollowUpPass)
        );
        assert!(scheduler.is_running());
        assert!(!scheduler.is_dirty());
        assert_eq!(
            scheduler.complete_pass(),
            Ok(ReconcileCompletionEffect::BecameIdle)
        );
        assert!(!scheduler.is_running());
    }

    #[test]
    fn clean_completion_becomes_quiescent() {
        let mut scheduler = ReconcileScheduler::new();
        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::StartPass
        );

        assert_eq!(
            scheduler.complete_pass(),
            Ok(ReconcileCompletionEffect::BecameIdle)
        );
        assert!(!scheduler.is_running());
        assert!(!scheduler.is_dirty());
    }

    #[test]
    fn wake_during_follow_up_is_not_lost() {
        let mut scheduler = ReconcileScheduler::new();
        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::StartPass
        );
        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::Coalesced
        );
        assert_eq!(
            scheduler.complete_pass(),
            Ok(ReconcileCompletionEffect::StartFollowUpPass)
        );

        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::Coalesced
        );
        assert_eq!(
            scheduler.complete_pass(),
            Ok(ReconcileCompletionEffect::StartFollowUpPass)
        );
        assert_eq!(
            scheduler.complete_pass(),
            Ok(ReconcileCompletionEffect::BecameIdle)
        );
    }

    #[test]
    fn wake_after_quiescence_starts_a_fresh_pass() {
        let mut scheduler = ReconcileScheduler::new();
        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::StartPass
        );
        assert_eq!(
            scheduler.complete_pass(),
            Ok(ReconcileCompletionEffect::BecameIdle)
        );

        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::StartPass
        );
        assert!(scheduler.is_running());
    }

    #[test]
    fn completion_while_idle_fails_closed() {
        let mut scheduler = ReconcileScheduler::new();
        assert_eq!(
            scheduler.complete_pass(),
            Err(ReconcileSchedulerError::CompletionWhileIdle)
        );
    }
}
