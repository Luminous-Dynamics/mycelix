// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Holochain remote-signal framing for Pulse V2 reconciliation wakes.
//!
//! `hdk::send_remote_signal` serializes its input with `ExternIO::encode`
//! before the receiver's `recv_remote_signal(ExternIO)` callback sees it.
//! Therefore the protocol discriminator must be part of that canonical
//! MessagePack value; it cannot be a byte prefix wrapped inside an `ExternIO`.
//!
//! V2 reserves one canonical top-level shape: a two-element MessagePack tuple
//! whose first element is [`PULSE_V2_REMOTE_SIGNAL_MAGIC`] and whose second
//! element is the strict, information-poor [`PulseRealtimeHintV1`]. The
//! canonical Holochain serializer emits that tuple with the fixed byte prefix
//! in [`PULSE_V2_REMOTE_SIGNAL_PREFIX`]. Current legacy `MailSignal` values are
//! serde-tagged maps, so the top-level tuple namespace is disjoint.
//!
//! The fail-closed rule is structural: any callback frame beginning with the
//! canonical two-element-array family marker is V2-family traffic. A truncated
//! namespace, wrong magic value, malformed payload, oversized payload, trailing
//! value, or unsupported version is terminal V2 failure and is never returned
//! to a legacy decoder.

#![forbid(unsafe_code)]

use pulse_realtime_types::{
    MAX_REALTIME_HINT_BYTES, PulseRealtimeHintV1, RealtimeContractError,
};
use serde::{Deserialize, Serialize};
use std::io::Cursor;

/// Stable semantic namespace carried as the first element of the remote tuple.
pub const PULSE_V2_REMOTE_SIGNAL_MAGIC: &str = "MYCELIX:PULSE:WAKE:V1";

/// Canonical Holochain/MessagePack prefix for the two-element tuple followed by
/// the 21-byte `PULSE_V2_REMOTE_SIGNAL_MAGIC` fixstr.
///
/// `0x92` is MessagePack fixarray(2); `0xb5` is fixstr(21). Unlike the first
/// revision of this theorem, this prefix is deliberately *inside* the value
/// serialized by `ExternIO::encode`, matching HDK 0.6.1 `send_remote_signal`.
pub const PULSE_V2_REMOTE_SIGNAL_PREFIX: &[u8] = b"\x92\xb5MYCELIX:PULSE:WAKE:V1";

/// The top-level MessagePack marker reserved for Pulse V2 remote wakes.
const PULSE_V2_REMOTE_SIGNAL_FAMILY_MARKER: u8 = 0x92;

/// Maximum complete V2 callback frame, including tuple namespace overhead.
pub const MAX_V2_REMOTE_SIGNAL_BYTES: usize =
    PULSE_V2_REMOTE_SIGNAL_PREFIX.len() + MAX_REALTIME_HINT_BYTES;

/// The only value callers should pass to `hdk::send_remote_signal` for a Pulse
/// V2 reconciliation wake. Private fields prevent constructing another magic
/// string through this API.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct PulseV2RemoteSignal(&'static str, PulseRealtimeHintV1);

impl PulseV2RemoteSignal {
    pub fn new(hint: PulseRealtimeHintV1) -> Result<Self, V2WireEncodeError> {
        hint.validate().map_err(V2WireEncodeError::Contract)?;
        Ok(Self(PULSE_V2_REMOTE_SIGNAL_MAGIC, hint))
    }

    pub fn inbox_changed_v2() -> Self {
        Self(
            PULSE_V2_REMOTE_SIGNAL_MAGIC,
            PulseRealtimeHintV1::inbox_changed_v2(),
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum V2WireEncodeError {
    Contract(RealtimeContractError),
    Serialization,
    CanonicalNamespaceMismatch,
    TooLarge { actual: usize, max: usize },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum V2WireDecodeError {
    InvalidNamespace,
    EmptyPayload,
    TooLarge { actual: usize, max: usize },
    Malformed,
    Contract(RealtimeContractError),
}

/// Result of classifying one raw `recv_remote_signal(ExternIO)` callback frame.
///
/// `PulseV2(Err(_))` is terminal. The original bytes are intentionally not
/// available in that branch, making parser-fallback downgrade hard to express.
#[derive(Debug, PartialEq, Eq)]
pub enum RemoteSignalAdmission<'a> {
    PulseV2(Result<PulseRealtimeHintV1, V2WireDecodeError>),
    Legacy(&'a [u8]),
}

/// Reproduce the bytes HDK 0.6.1 will place in the callback `ExternIO` when the
/// corresponding [`PulseV2RemoteSignal`] is supplied to `send_remote_signal`.
///
/// This uses `holochain_serialized_bytes` directly because `ExternIO::encode`
/// uses the same canonical serialization layer. The explicit prefix assertion
/// turns serializer/configuration drift into a local failure rather than
/// silently routing a nominal V2 wake through the legacy branch.
pub fn encode_v2_remote_signal(
    hint: &PulseRealtimeHintV1,
) -> Result<Vec<u8>, V2WireEncodeError> {
    let wire = PulseV2RemoteSignal::new(hint.clone())?;
    let frame = holochain_serialized_bytes::encode(&wire)
        .map_err(|_| V2WireEncodeError::Serialization)?;

    if !frame.starts_with(PULSE_V2_REMOTE_SIGNAL_PREFIX) {
        return Err(V2WireEncodeError::CanonicalNamespaceMismatch);
    }
    if frame.len() > MAX_V2_REMOTE_SIGNAL_BYTES {
        return Err(V2WireEncodeError::TooLarge {
            actual: frame.len(),
            max: MAX_V2_REMOTE_SIGNAL_BYTES,
        });
    }
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

/// Classify and validate one raw Holochain remote-signal callback frame.
///
/// The first byte is classified before either payload decoder is invoked. The
/// canonical V2 serializer starts with MessagePack fixarray(2), `0x92`; that
/// marker is reserved to V2 here. Consequently even a frame truncated midway
/// through the magic string remains terminal V2-family traffic instead of
/// falling through to legacy parsing.
pub fn admit_remote_signal(bytes: &[u8]) -> RemoteSignalAdmission<'_> {
    if bytes.first().copied() != Some(PULSE_V2_REMOTE_SIGNAL_FAMILY_MARKER) {
        return RemoteSignalAdmission::Legacy(bytes);
    }

    let result = bytes
        .strip_prefix(PULSE_V2_REMOTE_SIGNAL_PREFIX)
        .ok_or(V2WireDecodeError::InvalidNamespace)
        .and_then(decode_v2_payload_exact);
    RemoteSignalAdmission::PulseV2(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use pulse_realtime_types::PULSE_REALTIME_HINT_V1;

    #[derive(Debug, Serialize)]
    #[serde(tag = "type", content = "data")]
    enum LegacyShape {
        EmailReceived { encrypted_subject: Vec<u8> },
    }

    fn encode_unchecked_tuple(hint: &PulseRealtimeHintV1) -> Vec<u8> {
        holochain_serialized_bytes::encode(&(PULSE_V2_REMOTE_SIGNAL_MAGIC, hint))
            .expect("encode adversarial canonical tuple")
    }

    #[test]
    fn canonical_holochain_encoding_establishes_the_actual_callback_namespace() {
        let hint = PulseRealtimeHintV1::inbox_changed_v2();
        let wire = PulseV2RemoteSignal::new(hint.clone()).expect("construct V2 wire value");
        let direct = holochain_serialized_bytes::encode(&wire)
            .expect("canonical Holochain serialization");
        let helper = encode_v2_remote_signal(&hint).expect("encode canonical V2 wake");

        assert_eq!(helper, direct);
        assert!(helper.starts_with(PULSE_V2_REMOTE_SIGNAL_PREFIX));
        assert_eq!(helper[0], PULSE_V2_REMOTE_SIGNAL_FAMILY_MARKER);
        assert_eq!(
            admit_remote_signal(&helper),
            RemoteSignalAdmission::PulseV2(Ok(hint))
        );
    }

    #[test]
    fn current_legacy_tagged_map_shape_is_disjoint_from_reserved_v2_tuple_shape() {
        let legacy = LegacyShape::EmailReceived {
            encrypted_subject: vec![1, 2, 3],
        };
        let bytes = holochain_serialized_bytes::encode(&legacy)
            .expect("encode legacy-shaped tagged map");

        assert_ne!(bytes.first().copied(), Some(PULSE_V2_REMOTE_SIGNAL_FAMILY_MARKER));
        assert_eq!(admit_remote_signal(&bytes), RemoteSignalAdmission::Legacy(&bytes));
    }

    #[test]
    fn every_truncated_canonical_namespace_is_terminal_v2() {
        for cut in 1..PULSE_V2_REMOTE_SIGNAL_PREFIX.len() {
            let truncated = &PULSE_V2_REMOTE_SIGNAL_PREFIX[..cut];
            assert_eq!(
                admit_remote_signal(truncated),
                RemoteSignalAdmission::PulseV2(Err(V2WireDecodeError::InvalidNamespace)),
                "cut={cut}"
            );
        }
    }

    #[test]
    fn wrong_magic_inside_reserved_family_is_terminal_v2() {
        let frame = [0x92, 0xa1, b'X', 0xc0];
        assert_eq!(
            admit_remote_signal(&frame),
            RemoteSignalAdmission::PulseV2(Err(V2WireDecodeError::InvalidNamespace))
        );
    }

    #[test]
    fn malformed_prefixed_payload_is_terminal_not_legacy() {
        let mut frame = PULSE_V2_REMOTE_SIGNAL_PREFIX.to_vec();
        frame.extend_from_slice(b"not-messagepack");
        assert_eq!(
            admit_remote_signal(&frame),
            RemoteSignalAdmission::PulseV2(Err(V2WireDecodeError::Malformed))
        );
    }

    #[test]
    fn empty_prefixed_payload_is_terminal_not_legacy() {
        assert_eq!(
            admit_remote_signal(PULSE_V2_REMOTE_SIGNAL_PREFIX),
            RemoteSignalAdmission::PulseV2(Err(V2WireDecodeError::EmptyPayload))
        );
    }

    #[test]
    fn trailing_messagepack_value_is_rejected_without_downgrade() {
        let hint = PulseRealtimeHintV1::inbox_changed_v2();
        let mut frame = encode_v2_remote_signal(&hint).expect("encode canonical wake");
        frame.push(0xc0); // one valid trailing MessagePack nil value

        assert_eq!(
            admit_remote_signal(&frame),
            RemoteSignalAdmission::PulseV2(Err(V2WireDecodeError::Malformed))
        );
    }

    #[test]
    fn oversized_v2_is_rejected_before_payload_deserialization() {
        let mut frame = PULSE_V2_REMOTE_SIGNAL_PREFIX.to_vec();
        frame.extend(std::iter::repeat_n(0u8, MAX_REALTIME_HINT_BYTES + 1));

        assert_eq!(
            admit_remote_signal(&frame),
            RemoteSignalAdmission::PulseV2(Err(V2WireDecodeError::TooLarge {
                actual: MAX_REALTIME_HINT_BYTES + 1,
                max: MAX_REALTIME_HINT_BYTES,
            }))
        );
    }

    #[test]
    fn unsupported_version_stays_v2_and_fails_closed() {
        let mut unsupported = PulseRealtimeHintV1::inbox_changed_v2();
        unsupported.version = PULSE_REALTIME_HINT_V1 + 1;
        let frame = encode_unchecked_tuple(&unsupported);
        assert!(frame.starts_with(PULSE_V2_REMOTE_SIGNAL_PREFIX));

        assert_eq!(
            admit_remote_signal(&frame),
            RemoteSignalAdmission::PulseV2(Err(V2WireDecodeError::Contract(
                RealtimeContractError::UnsupportedVersion(PULSE_REALTIME_HINT_V1 + 1)
            )))
        );
    }

    #[test]
    fn unframed_hint_cannot_self_promote_to_remote_v2() {
        let hint = PulseRealtimeHintV1::inbox_changed_v2();
        let bare = holochain_serialized_bytes::encode(&hint).expect("encode bare hint");

        assert!(!bare.starts_with(PULSE_V2_REMOTE_SIGNAL_PREFIX));
        assert!(matches!(admit_remote_signal(&bare), RemoteSignalAdmission::Legacy(_)));
    }

    #[test]
    fn json_is_not_an_alternate_v2_remote_encoding() {
        let hint = PulseRealtimeHintV1::inbox_changed_v2();
        let json = serde_json::to_vec(&hint).expect("encode JSON hint");

        assert!(matches!(admit_remote_signal(&json), RemoteSignalAdmission::Legacy(_)));
    }

    #[test]
    fn canonical_remote_wake_remains_information_poor() {
        let wire = PulseV2RemoteSignal::inbox_changed_v2();
        let json = serde_json::to_string(&wire).expect("inspect remote wake semantics");

        assert!(json.contains(PULSE_V2_REMOTE_SIGNAL_MAGIC));
        assert!(json.contains("inbox_changed_v2"));
        for forbidden in [
            "subject",
            "body",
            "ciphertext",
            "thread_id",
            "sender",
            "message_id",
            "email_hash",
            "delivered",
            "read_receipt",
            "verified",
            "authorized",
        ] {
            assert!(
                !json.contains(forbidden),
                "remote wake must not carry or imply {forbidden}: {json}"
            );
        }
    }
}
