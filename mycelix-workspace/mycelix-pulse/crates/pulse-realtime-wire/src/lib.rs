// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Transport framing for Pulse realtime wakes.
//!
//! This crate deliberately sits outside `pulse-realtime-types`: the latter
//! defines transport-neutral, non-authoritative reconciliation hints, while
//! this crate assigns those hints an explicit remote-signal wire family.
//!
//! The security rule is structural: once a frame starts with the V2 wire
//! prefix, every subsequent decode failure remains a V2 failure. Callers must
//! not offer those bytes to a legacy decoder. This prevents malformed,
//! truncated, future-version, or trailing-data V2 traffic from acquiring
//! legacy `MailSignal` semantics through parser fallback.

#![forbid(unsafe_code)]

use pulse_realtime_types::{
    MAX_REALTIME_HINT_BYTES, PulseRealtimeHintV1, RealtimeContractError,
};
use serde::Deserialize;
use std::io::Cursor;

/// Raw-byte namespace assigned to Pulse V2 remote reconciliation wakes.
///
/// This prefix is outside the MessagePack payload on purpose. Holochain 0.6.x
/// `ExternIO` exposes raw bytes, so a receiver can classify the protocol family
/// before invoking either the V2 or legacy deserializer.
pub const PULSE_V2_REMOTE_SIGNAL_PREFIX: &[u8] = b"MYCELIX:PULSE:WAKE:V1\0";

/// Maximum complete V2 remote-signal frame, including its discriminator.
pub const MAX_V2_REMOTE_SIGNAL_BYTES: usize =
    PULSE_V2_REMOTE_SIGNAL_PREFIX.len() + MAX_REALTIME_HINT_BYTES;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum V2WireEncodeError {
    Contract(RealtimeContractError),
    Serialization,
    TooLarge { actual: usize, max: usize },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum V2WireDecodeError {
    EmptyPayload,
    TooLarge { actual: usize, max: usize },
    Malformed,
    Contract(RealtimeContractError),
}

/// Result of classifying one raw Holochain remote-signal payload.
///
/// `PulseV2(Err(_))` is terminal for the V2 family. The original bytes are not
/// returned in that branch by design, which makes accidental decoder fallback
/// harder to express. Only frames without the V2 prefix are exposed to the
/// legacy compatibility path.
#[derive(Debug, PartialEq, Eq)]
pub enum RemoteSignalAdmission<'a> {
    PulseV2(Result<PulseRealtimeHintV1, V2WireDecodeError>),
    Legacy(&'a [u8]),
}

/// Encode one canonical Pulse V2 remote wake.
///
/// Remote V2 uses exactly one encoding: named MessagePack behind the fixed
/// raw-byte discriminator. JSON remains available in the transport-neutral
/// crate for browser/test compatibility, but is intentionally not admitted on
/// this remote-signal wire.
pub fn encode_v2_remote_signal(
    hint: &PulseRealtimeHintV1,
) -> Result<Vec<u8>, V2WireEncodeError> {
    hint.validate().map_err(V2WireEncodeError::Contract)?;

    let payload = rmp_serde::to_vec_named(hint).map_err(|_| V2WireEncodeError::Serialization)?;
    if payload.len() > MAX_REALTIME_HINT_BYTES {
        return Err(V2WireEncodeError::TooLarge {
            actual: payload.len(),
            max: MAX_REALTIME_HINT_BYTES,
        });
    }

    let mut frame = Vec::with_capacity(PULSE_V2_REMOTE_SIGNAL_PREFIX.len() + payload.len());
    frame.extend_from_slice(PULSE_V2_REMOTE_SIGNAL_PREFIX);
    frame.extend_from_slice(&payload);
    Ok(frame)
}

fn decode_v2_payload_exact(bytes: &[u8]) -> Result<PulseRealtimeHintV1, V2WireDecodeError> {
    if bytes.is_empty() {
        return Err(V2WireDecodeError::EmptyPayload);
    }
    if bytes.len() > MAX_REALTIME_HINT_BYTES {
        return Err(V2WireDecodeError::TooLarge {
            actual: bytes.len(),
            max: MAX_REALTIME_HINT_BYTES,
        });
    }

    let mut deserializer = rmp_serde::Deserializer::new(Cursor::new(bytes));
    let hint = PulseRealtimeHintV1::deserialize(&mut deserializer)
        .map_err(|_| V2WireDecodeError::Malformed)?;
    if deserializer.position() != bytes.len() as u64 {
        return Err(V2WireDecodeError::Malformed);
    }

    hint.validate().map_err(V2WireDecodeError::Contract)?;
    Ok(hint)
}

/// Classify and, for V2 traffic, fully validate one remote-signal frame.
///
/// This is the demultiplexing theorem boundary. Prefix classification happens
/// before deserialization. Any frame bearing the V2 prefix remains in the V2
/// branch even when its payload is empty, oversized, malformed, has trailing
/// bytes, or requests an unsupported future version.
pub fn admit_remote_signal(bytes: &[u8]) -> RemoteSignalAdmission<'_> {
    match bytes.strip_prefix(PULSE_V2_REMOTE_SIGNAL_PREFIX) {
        Some(payload) => RemoteSignalAdmission::PulseV2(decode_v2_payload_exact(payload)),
        None => RemoteSignalAdmission::Legacy(bytes),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pulse_realtime_types::{DurableWakeHintV1, PULSE_REALTIME_HINT_V1};

    fn prefixed(payload: &[u8]) -> Vec<u8> {
        let mut frame = Vec::with_capacity(PULSE_V2_REMOTE_SIGNAL_PREFIX.len() + payload.len());
        frame.extend_from_slice(PULSE_V2_REMOTE_SIGNAL_PREFIX);
        frame.extend_from_slice(payload);
        frame
    }

    #[test]
    fn canonical_v2_wire_round_trips_only_as_v2() {
        let hint = PulseRealtimeHintV1::inbox_changed_v2();
        let frame = encode_v2_remote_signal(&hint).expect("encode canonical V2 wake");

        assert!(frame.starts_with(PULSE_V2_REMOTE_SIGNAL_PREFIX));
        assert_eq!(
            admit_remote_signal(&frame),
            RemoteSignalAdmission::PulseV2(Ok(hint))
        );
    }

    #[test]
    fn malformed_prefixed_v2_is_terminal_not_legacy() {
        let frame = prefixed(b"not-messagepack");

        assert_eq!(
            admit_remote_signal(&frame),
            RemoteSignalAdmission::PulseV2(Err(V2WireDecodeError::Malformed))
        );
    }

    #[test]
    fn empty_prefixed_v2_is_terminal_not_legacy() {
        assert_eq!(
            admit_remote_signal(PULSE_V2_REMOTE_SIGNAL_PREFIX),
            RemoteSignalAdmission::PulseV2(Err(V2WireDecodeError::EmptyPayload))
        );
    }

    #[test]
    fn trailing_messagepack_value_is_rejected_without_downgrade() {
        let hint = PulseRealtimeHintV1::inbox_changed_v2();
        let mut frame = encode_v2_remote_signal(&hint).expect("encode canonical V2 wake");
        frame.push(0xc0); // individually valid MessagePack nil

        assert_eq!(
            admit_remote_signal(&frame),
            RemoteSignalAdmission::PulseV2(Err(V2WireDecodeError::Malformed))
        );
    }

    #[test]
    fn oversized_prefixed_v2_is_rejected_before_deserialization() {
        let frame = prefixed(&vec![0u8; MAX_REALTIME_HINT_BYTES + 1]);

        assert_eq!(frame.len(), MAX_V2_REMOTE_SIGNAL_BYTES + 1);
        assert_eq!(
            admit_remote_signal(&frame),
            RemoteSignalAdmission::PulseV2(Err(V2WireDecodeError::TooLarge {
                actual: MAX_REALTIME_HINT_BYTES + 1,
                max: MAX_REALTIME_HINT_BYTES,
            }))
        );
    }

    #[test]
    fn future_version_stays_in_v2_family_and_fails_closed() {
        let future = PulseRealtimeHintV1 {
            version: PULSE_REALTIME_HINT_V1 + 1,
            hint: DurableWakeHintV1::InboxChangedV2,
        };
        let payload = rmp_serde::to_vec_named(&future).expect("encode future-version fixture");
        let frame = prefixed(&payload);

        assert_eq!(
            admit_remote_signal(&frame),
            RemoteSignalAdmission::PulseV2(Err(V2WireDecodeError::Contract(
                RealtimeContractError::UnsupportedVersion(PULSE_REALTIME_HINT_V1 + 1)
            )))
        );
    }

    #[test]
    fn json_is_not_a_second_remote_wire_encoding() {
        let hint = PulseRealtimeHintV1::inbox_changed_v2();
        let json = serde_json::to_vec(&hint).expect("encode JSON fixture");
        let frame = prefixed(&json);

        assert_eq!(
            admit_remote_signal(&frame),
            RemoteSignalAdmission::PulseV2(Err(V2WireDecodeError::Malformed))
        );
    }

    #[test]
    fn valid_hint_without_prefix_never_self_promotes_to_v2() {
        let hint = PulseRealtimeHintV1::inbox_changed_v2();
        let raw_hint = rmp_serde::to_vec_named(&hint).expect("encode unframed hint fixture");

        assert_eq!(
            admit_remote_signal(&raw_hint),
            RemoteSignalAdmission::Legacy(raw_hint.as_slice())
        );
    }

    #[test]
    fn arbitrary_non_prefixed_bytes_are_legacy_only() {
        let legacy_fixture = b"legacy-mail-signal-fixture";
        assert_eq!(
            admit_remote_signal(legacy_fixture),
            RemoteSignalAdmission::Legacy(legacy_fixture)
        );
    }

    #[test]
    fn canonical_remote_wake_contains_no_authority_bearing_fields() {
        let frame = encode_v2_remote_signal(&PulseRealtimeHintV1::inbox_changed_v2())
            .expect("encode canonical V2 wake");
        let printable = String::from_utf8_lossy(&frame);

        for forbidden in [
            "subject",
            "body",
            "ciphertext",
            "message_hash",
            "thread_id",
            "sender",
            "delivered",
            "verified",
            "authorized",
            "read_receipt",
        ] {
            assert!(
                !printable.contains(forbidden),
                "remote wake must not carry or imply {forbidden}: {printable}"
            );
        }
    }
}
