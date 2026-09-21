// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Non-strengthening Personal presentation for typed Holochain call failures.
//!
//! User-facing copy is derived only from typed phase and failure-class evidence.
//! Human-readable conductor payloads are deliberately not inspected here, so
//! message text cannot manufacture retryability, transaction outcome, or a
//! stronger protocol class such as `HeadMoved`.

use mycelix_leptos_client::ConductorErrorKind;
use mycelix_leptos_core::{
    HolochainCallFailureKind, HolochainCallFailureObservation, HolochainCallPhase,
};

/// Stable presentation facts for one typed Personal call failure.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PersonalDiagnosticCopy {
    pub title: &'static str,
    pub summary: &'static str,
    pub phase_label: &'static str,
    pub evidence_label: &'static str,
}

impl PersonalDiagnosticCopy {
    /// Explicit nonclaim carried with every typed diagnostic presentation.
    pub const AUTHORITY_NOTICE: &'static str =
        "Diagnostic evidence only; authorization, retry safety, and transaction outcome are not established.";
}

/// Project a typed call failure into stable Personal UI copy without inspecting
/// its human-readable source message.
pub fn personal_diagnostic_copy(
    observation: &HolochainCallFailureObservation,
) -> PersonalDiagnosticCopy {
    let error = observation.error();
    let phase = error.phase();
    let kind = observation.failure_kind();

    let (title, summary) = match kind {
        HolochainCallFailureKind::ConnectionUnavailable => (
            "Live conductor unavailable",
            "The call did not have a usable conductor connection.",
        ),
        HolochainCallFailureKind::Timeout => (
            "Live call timed out",
            "The typed call did not complete within the configured transport timeout.",
        ),
        HolochainCallFailureKind::SigningUnavailable => (
            "Signer unavailable",
            "An authorized zome-call signer was not available for this call.",
        ),
        HolochainCallFailureKind::InvalidSignature => (
            "Signature rejected",
            "The typed call reported invalid signature evidence.",
        ),
        HolochainCallFailureKind::AuthenticationFailed => (
            "Authentication failed",
            "The typed call reported an authentication failure.",
        ),
        HolochainCallFailureKind::Serialization => match phase {
            HolochainCallPhase::Encode => (
                "Request encoding failed",
                "Personal could not encode the typed call payload.",
            ),
            HolochainCallPhase::Decode => (
                "Response decoding failed",
                "Personal could not decode the typed call response.",
            ),
            _ => (
                "Serialization failed",
                "The typed call reported a serialization failure.",
            ),
        },
        HolochainCallFailureKind::TransportProtocol => (
            "Transport protocol error",
            "The browser transport reported a typed protocol or response failure.",
        ),
        HolochainCallFailureKind::SecureRandomnessUnavailable => (
            "Secure randomness unavailable",
            "The call could not obtain the secure randomness required by the client path.",
        ),
        HolochainCallFailureKind::UnknownRole => (
            "Holochain role unavailable",
            "The requested role was not present in the connected application context.",
        ),
        HolochainCallFailureKind::LegacyUnstructuredZomeCall => (
            "Unstructured zome-call failure",
            "This failure came through a legacy string-only zome-call error boundary.",
        ),
        HolochainCallFailureKind::Conductor(kind) => conductor_copy(kind),
    };

    PersonalDiagnosticCopy {
        title,
        summary,
        phase_label: phase_label(phase),
        evidence_label: evidence_label(kind),
    }
}

const fn conductor_copy(kind: ConductorErrorKind) -> (&'static str, &'static str) {
    match kind {
        ConductorErrorKind::Internal => (
            "Conductor internal error",
            "The conductor reported its broad internal-error class.",
        ),
        ConductorErrorKind::Deserialization => (
            "Conductor deserialization error",
            "The conductor reported that it could not deserialize application data.",
        ),
        ConductorErrorKind::DnaRead => (
            "Conductor DNA read error",
            "The conductor reported its broad DNA-read failure class.",
        ),
        ConductorErrorKind::Ribosome => (
            "Zome execution error",
            "The conductor reported its broad ribosome failure class; no narrower cause is inferred.",
        ),
        ConductorErrorKind::ZomeCallAuthenticationFailed => (
            "Zome-call authentication failed",
            "The conductor reported its zome-call authentication-failure class.",
        ),
        ConductorErrorKind::ZomeCallUnauthorized => (
            "Zome call unauthorized",
            "The conductor reported that this zome call was unauthorized.",
        ),
        ConductorErrorKind::CountersigningSession => (
            "Countersigning session error",
            "The conductor reported its countersigning-session failure class.",
        ),
    }
}

const fn phase_label(phase: HolochainCallPhase) -> &'static str {
    match phase {
        HolochainCallPhase::Admission => "admission",
        HolochainCallPhase::Encode => "encode",
        HolochainCallPhase::Transport => "transport",
        HolochainCallPhase::Decode => "decode",
    }
}

const fn evidence_label(kind: HolochainCallFailureKind) -> &'static str {
    match kind {
        HolochainCallFailureKind::Conductor(kind) => kind.label(),
        other => other.label(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_leptos_client::{ClientError, ConductorError};
    use mycelix_leptos_core::{
        HolochainCallAttemptSequence, HolochainCallError, HolochainCallFailureObservation,
    };

    fn observation(
        phase: HolochainCallPhase,
        source: ClientError,
    ) -> HolochainCallFailureObservation {
        let mut sequence = HolochainCallAttemptSequence::default();
        let id = sequence.allocate().expect("attempt id");
        HolochainCallFailureObservation::new(
            id,
            HolochainCallError::new(
                phase,
                "personal",
                "identity_vault",
                "set_profile_view_if_current",
                source,
            ),
        )
    }

    #[test]
    fn encode_and_decode_serialization_failures_have_distinct_copy() {
        let encode = personal_diagnostic_copy(&observation(
            HolochainCallPhase::Encode,
            ClientError::SerializationError("payload".into()),
        ));
        let decode = personal_diagnostic_copy(&observation(
            HolochainCallPhase::Decode,
            ClientError::SerializationError("response".into()),
        ));

        assert_eq!(encode.title, "Request encoding failed");
        assert_eq!(encode.phase_label, "encode");
        assert_eq!(decode.title, "Response decoding failed");
        assert_eq!(decode.phase_label, "decode");
    }

    #[test]
    fn conductor_auth_and_authorization_classes_remain_distinct() {
        let authentication = personal_diagnostic_copy(&observation(
            HolochainCallPhase::Transport,
            ClientError::Conductor(ConductorError {
                kind: ConductorErrorKind::ZomeCallAuthenticationFailed,
                message: "auth".into(),
            }),
        ));
        let unauthorized = personal_diagnostic_copy(&observation(
            HolochainCallPhase::Transport,
            ClientError::Conductor(ConductorError {
                kind: ConductorErrorKind::ZomeCallUnauthorized,
                message: "denied".into(),
            }),
        ));

        assert_eq!(authentication.evidence_label, "zome-call-authentication-failed");
        assert_eq!(unauthorized.evidence_label, "zome-call-unauthorized");
        assert_ne!(authentication.title, unauthorized.title);
    }

    #[test]
    fn ribosome_message_cannot_strengthen_user_facing_classification() {
        let ordinary = personal_diagnostic_copy(&observation(
            HolochainCallPhase::Transport,
            ClientError::Conductor(ConductorError {
                kind: ConductorErrorKind::Ribosome,
                message: "ordinary ribosome failure".into(),
            }),
        ));
        let head_text = personal_diagnostic_copy(&observation(
            HolochainCallPhase::Transport,
            ClientError::Conductor(ConductorError {
                kind: ConductorErrorKind::Ribosome,
                message: "source chain head moved".into(),
            }),
        ));

        assert_eq!(ordinary, head_text);
        assert_eq!(head_text.title, "Zome execution error");
        assert_eq!(head_text.evidence_label, "ribosome");
        assert!(!head_text.title.to_ascii_lowercase().contains("head"));
        assert!(!head_text.summary.to_ascii_lowercase().contains("head"));
    }

    #[test]
    fn presentation_contract_carries_explicit_authority_nonclaim() {
        assert!(PersonalDiagnosticCopy::AUTHORITY_NOTICE.contains("not established"));
        assert!(PersonalDiagnosticCopy::AUTHORITY_NOTICE.contains("retry safety"));
        assert!(PersonalDiagnosticCopy::AUTHORITY_NOTICE.contains("transaction outcome"));
    }
}
