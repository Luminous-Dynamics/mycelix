// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! HDK-specific admission seam for Pulse realtime remote signals.
//!
//! This crate is intentionally tiny. The wire-family theorem lives in
//! `pulse-realtime-wire`; this adapter proves that the Holochain `ExternIO`
//! received by `recv_remote_signal` exposes the exact callback bytes to that
//! theorem without re-serialization or speculative decoding.
//!
//! The authority rule is unchanged: a successful V2 admission is only a
//! reconciliation wake. It is never durable message state.

#![forbid(unsafe_code)]

use hdk::prelude::ExternIO;
use pulse_realtime_wire::{RemoteSignalAdmission, admit_remote_signal};

/// Classify one `recv_remote_signal(ExternIO)` input using its original
/// serialized callback bytes.
///
/// V2-family errors remain terminal in [`RemoteSignalAdmission::PulseV2`].
/// Callers may invoke `ExternIO::decode` for the legacy protocol only when this
/// function returns [`RemoteSignalAdmission::Legacy`].
pub fn admit_extern_io(signal: &ExternIO) -> RemoteSignalAdmission<'_> {
    admit_remote_signal(signal.as_bytes())
}

#[cfg(test)]
mod tests {
    use super::*;
    use pulse_realtime_wire::{
        PULSE_V2_REMOTE_SIGNAL_PREFIX, PulseV2RemoteSignal, V2WireDecodeError,
    };
    use serde::{Deserialize, Serialize};

    #[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(tag = "type", content = "data")]
    enum LegacySignal {
        Typing { is_typing: bool },
    }

    #[test]
    fn canonical_hdk_extern_io_enters_v2_family() {
        let signal = ExternIO::encode(PulseV2RemoteSignal::inbox_changed_v2())
            .expect("HDK must serialize canonical V2 wake");

        assert!(matches!(
            admit_extern_io(&signal),
            RemoteSignalAdmission::PulseV2(Ok(_))
        ));
    }

    #[test]
    fn adapter_is_exactly_the_raw_extern_io_admission() {
        let signal = ExternIO::encode(PulseV2RemoteSignal::inbox_changed_v2())
            .expect("HDK must serialize canonical V2 wake");

        assert_eq!(admit_extern_io(&signal), admit_remote_signal(signal.as_bytes()));
    }

    #[test]
    fn truncated_v2_namespace_is_terminal_and_never_legacy() {
        let signal = ExternIO::from(PULSE_V2_REMOTE_SIGNAL_PREFIX.to_vec());

        assert_eq!(
            admit_extern_io(&signal),
            RemoteSignalAdmission::PulseV2(Err(V2WireDecodeError::EmptyPayload))
        );
    }

    #[test]
    fn legacy_extern_io_remains_available_only_on_legacy_branch() {
        let expected = LegacySignal::Typing { is_typing: true };
        let signal = ExternIO::encode(expected.clone()).expect("serialize legacy signal");

        assert!(matches!(
            admit_extern_io(&signal),
            RemoteSignalAdmission::Legacy(_)
        ));

        let decoded: LegacySignal = signal.decode().expect("legacy decode after admission");
        assert_eq!(decoded, expected);
    }
}
