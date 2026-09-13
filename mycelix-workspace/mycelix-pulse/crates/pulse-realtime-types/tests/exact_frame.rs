// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use pulse_realtime_types::{
    PulseRealtimeHintV1, RealtimeDecodeError, decode_realtime_hint,
};

#[test]
fn messagepack_hint_must_consume_the_entire_transport_frame() {
    let hint = PulseRealtimeHintV1::inbox_changed_v2();
    let mut frame = rmp_serde::to_vec_named(&hint).expect("serialize valid wake hint");

    // Append a second, individually valid MessagePack value. A frame decoder
    // must reject this rather than accepting the valid first-value prefix and
    // silently ignoring attacker-controlled trailing bytes.
    frame.push(0xc0); // MessagePack nil

    assert_eq!(
        decode_realtime_hint(&frame),
        Err(RealtimeDecodeError::Malformed),
        "a realtime transport frame must contain exactly one complete hint"
    );
}

#[test]
fn json_rejects_non_whitespace_trailing_data() {
    let hint = PulseRealtimeHintV1::inbox_changed_v2();
    let mut frame = serde_json::to_vec(&hint).expect("serialize valid JSON wake hint");
    frame.extend_from_slice(b"{} ");

    assert_eq!(
        decode_realtime_hint(&frame),
        Err(RealtimeDecodeError::Malformed),
        "JSON compatibility framing must not admit a second value"
    );
}
