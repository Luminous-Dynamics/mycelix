//! Renderer ↔ broker capability contract.
//!
//! This module deliberately models renderer requests as *untrusted claims*.
//! Authentication/integrity is supplied by BridgeCodec; authorization belongs
//! to the broker. A renderer never receives a generic browser privilege.

use serde::{Deserialize, Serialize};
use std::fmt;

pub const MAX_CAPABILITY_ID_LEN: usize = 128;
pub const MAX_ORIGIN_LEN: usize = 2048;
pub const MAX_RESOURCE_LEN: usize = 4096;

/// Monotonic per-session request identifier.
///
/// The broker must reject zero, duplicate, or regressing IDs within a live
/// authenticated renderer session. The authenticated transport prevents an
/// off-path attacker from injecting frames, while this identifier prevents a
/// renderer from replaying an old authorized operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct RequestId(pub u64);

impl RequestId {
    pub fn new(value: u64) -> Result<Self, CapabilityError> {
        if value == 0 {
            return Err(CapabilityError::InvalidRequestId);
        }
        Ok(Self(value))
    }
}

/// Security principal claimed by the renderer.
///
/// This is intentionally not a URL and not a tab identifier. The broker
/// derives/assigns it during navigation and compares the request binding
/// against its authoritative process state.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct OriginBinding {
    value: String,
}

impl OriginBinding {
    pub fn new(value: impl Into<String>) -> Result<Self, CapabilityError> {
        let value = value.into();
        if value.is_empty() || value.len() > MAX_ORIGIN_LEN {
            return Err(CapabilityError::InvalidOrigin);
        }
        if value.chars().any(char::is_control) {
            return Err(CapabilityError::InvalidOrigin);
        }
        Ok(Self { value })
    }

    pub fn as_str(&self) -> &str {
        &self.value
    }
}

/// Narrow capability names. New capabilities require an explicit protocol
/// change; there is deliberately no arbitrary stringly-typed privilege.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Capability {
    Navigate,
    NetworkFetch,
    ReadStorage,
    WriteStorage,
    PromptPermission,
    ClipboardRead,
    ClipboardWrite,
    Geolocation,
    MediaInput,
    FileOpen,
    FileSave,
}

impl Capability {
    pub const fn stable_name(self) -> &'static str {
        match self {
            Self::Navigate => "navigate",
            Self::NetworkFetch => "network-fetch",
            Self::ReadStorage => "read-storage",
            Self::WriteStorage => "write-storage",
            Self::PromptPermission => "prompt-permission",
            Self::ClipboardRead => "clipboard-read",
            Self::ClipboardWrite => "clipboard-write",
            Self::Geolocation => "geolocation",
            Self::MediaInput => "media-input",
            Self::FileOpen => "file-open",
            Self::FileSave => "file-save",
        }
    }
}

/// Resource requested by a capability operation.
///
/// Resource values are opaque to the transport layer. The broker must parse
/// and validate them according to the capability before performing an action.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CapabilityResource {
    value: String,
}

impl CapabilityResource {
    pub fn new(value: impl Into<String>) -> Result<Self, CapabilityError> {
        let value = value.into();
        if value.len() > MAX_RESOURCE_LEN || value.chars().any(char::is_control) {
            return Err(CapabilityError::InvalidResource);
        }
        Ok(Self { value })
    }

    pub fn as_str(&self) -> &str {
        &self.value
    }
}

/// Untrusted renderer request. The broker must authorize every field; a valid
/// authenticated message is not itself permission to perform the operation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CapabilityRequest {
    pub request_id: RequestId,
    pub capability: Capability,
    pub origin: OriginBinding,
    pub resource: Option<CapabilityResource>,
}

/// Explicit response. Denial is a normal protocol result, never an error path
/// that can accidentally become an implicit allow.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum CapabilityResponse {
    Allowed {
        request_id: RequestId,
        capability: Capability,
    },
    Denied {
        request_id: RequestId,
        reason: DenialReason,
    },
}

/// Stable denial categories suitable for security receipts and tests without
/// leaking implementation details or secrets.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DenialReason {
    UnknownCapability,
    InvalidRequest,
    OriginMismatch,
    CapabilityNotGranted,
    ResourcePolicyDenied,
    UserPolicyDenied,
    RendererStateDenied,
    RateLimited,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CapabilityError {
    InvalidRequestId,
    InvalidOrigin,
    InvalidResource,
    RequestIdReplay { previous: RequestId, received: RequestId },
}

impl fmt::Display for CapabilityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidRequestId => f.write_str("request id must be non-zero"),
            Self::InvalidOrigin => f.write_str("origin binding is invalid"),
            Self::InvalidResource => f.write_str("capability resource is invalid"),
            Self::RequestIdReplay { previous, received } => {
                write!(f, "request id replay/regression: previous={}, received={}", previous.0, received.0)
            }
        }
    }
}

impl std::error::Error for CapabilityError {}

/// Per-renderer request-order guard.
///
/// This is intentionally small and deterministic. It does not replace
/// transport authentication or authorization.
#[derive(Debug, Default)]
pub struct RequestSequence {
    last: Option<RequestId>,
}

impl RequestSequence {
    pub fn check_and_record(&mut self, request_id: RequestId) -> Result<(), CapabilityError> {
        if let Some(previous) = self.last {
            if request_id.0 <= previous.0 {
                return Err(CapabilityError::RequestIdReplay {
                    previous,
                    received: request_id,
                });
            }
        }
        self.last = Some(request_id);
        Ok(())
    }

    pub fn last(&self) -> Option<RequestId> {
        self.last
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn request_ids_are_nonzero() {
        assert!(RequestId::new(0).is_err());
        assert_eq!(RequestId::new(1).unwrap().0, 1);
    }

    #[test]
    fn origin_rejects_controls_and_bounds() {
        assert!(OriginBinding::new("").is_err());
        assert!(OriginBinding::new("https://example.com").is_ok());
        assert!(OriginBinding::new("https://example.com\nforged").is_err());
        assert!(OriginBinding::new("x".repeat(MAX_ORIGIN_LEN + 1)).is_err());
    }

    #[test]
    fn resource_is_bounded() {
        assert!(CapabilityResource::new("https://example.com").is_ok());
        assert!(CapabilityResource::new("x".repeat(MAX_RESOURCE_LEN + 1)).is_err());
    }

    #[test]
    fn capability_names_are_stable() {
        assert_eq!(Capability::NetworkFetch.stable_name(), "network-fetch");
        assert_eq!(Capability::FileOpen.stable_name(), "file-open");
    }

    #[test]
    fn request_sequence_is_strictly_monotonic() {
        let mut sequence = RequestSequence::default();
        let first = RequestId::new(1).unwrap();
        let second = RequestId::new(2).unwrap();

        sequence.check_and_record(first).unwrap();
        sequence.check_and_record(second).unwrap();
        assert_eq!(sequence.last(), Some(second));

        let replay = sequence.check_and_record(first).unwrap_err();
        assert!(matches!(
            replay,
            CapabilityError::RequestIdReplay {
                previous,
                received
            } if previous == second && received == first
        ));
    }

    #[test]
    fn capability_request_roundtrips() {
        let request = CapabilityRequest {
            request_id: RequestId::new(7).unwrap(),
            capability: Capability::NetworkFetch,
            origin: OriginBinding::new("https://example.com").unwrap(),
            resource: Some(CapabilityResource::new("https://cdn.example.com/app.js").unwrap()),
        };
        let encoded = rmp_serde::to_vec(&request).unwrap();
        let decoded: CapabilityRequest = rmp_serde::from_slice(&encoded).unwrap();
        assert_eq!(decoded, request);
    }

    #[test]
    fn denial_is_explicit() {
        let response = CapabilityResponse::Denied {
            request_id: RequestId::new(8).unwrap(),
            reason: DenialReason::OriginMismatch,
        };
        let encoded = rmp_serde::to_vec(&response).unwrap();
        let decoded: CapabilityResponse = rmp_serde::from_slice(&encoded).unwrap();
        assert_eq!(decoded, response);
    }
}
