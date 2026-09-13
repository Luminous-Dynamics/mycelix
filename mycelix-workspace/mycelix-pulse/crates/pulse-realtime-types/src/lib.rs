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
//! - durable state is recovered from the authoritative Holochain/DHT source.

use serde::{Deserialize, Serialize};

/// Current wire version for [`PulseRealtimeHintV1`].
pub const PULSE_REALTIME_HINT_V1: u16 = 1;

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
        for (field, value) in [
            ("subject", json!("pretend subject")),
            ("delivered", json!(true)),
            ("thread_id", json!("pretend-thread")),
            ("ciphertext", json!("pretend-ciphertext")),
            ("verified", json!(true)),
            ("authorized", json!(true)),
        ] {
            let mut value = json!({
                "version": PULSE_REALTIME_HINT_V1,
                "hint": "inbox_changed_v2",
            });
            value
                .as_object_mut()
                .expect("wake fixture is an object")
                .insert(field.to_string(), value);

            let error = serde_json::from_value::<PulseRealtimeHintV1>(value)
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
}
