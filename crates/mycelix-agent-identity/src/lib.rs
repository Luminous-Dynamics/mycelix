// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
#![forbid(unsafe_code)]
//! AGENT-002 stable-agent/runtime-instance identity kernel.
//!
//! This crate proves one deliberately narrow theorem: one exact bounded runtime
//! instance identifier is canonically bound to one exact existing Mycelix
//! `PrincipalId`. It does not authenticate the runtime, prove key possession,
//! grant authority, establish currentness, or authorize effects.

use mycelix_institutional_core::{Digest32, PrincipalId};
use serde::{Deserialize, Deserializer, Serialize, Serializer, de::Error as _};
use std::fmt;

pub const AGENT_IDENTITY_PROTOCOL_VERSION: &str = "mycelix-agent-identity-v0.1";
pub const RUNTIME_INSTANCE_IDENTITY_PROFILE: &str =
    "mycelix-agent-runtime-instance-v1-blake3-framed-semantic";
pub const MAX_RUNTIME_INSTANCE_ID_BYTES: usize = 512;
const DOMAIN_RUNTIME_INSTANCE_IDENTITY: &[u8] = b"mycelix/agent/runtime-instance/v1";

/// Opaque exact-byte identity for one runtime/workload/process instance.
///
/// Unlike legacy institutional ID tuple structs, this type preserves its
/// validation invariant across construction and deserialization. Accepted UTF-8
/// bytes are never trimmed, case-folded, or Unicode-normalized.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RuntimeInstanceId(String);

impl RuntimeInstanceId {
    pub fn new(value: impl Into<String>) -> Result<Self, RuntimeInstanceIdError> {
        let value = value.into();
        validate_runtime_instance_id(&value)?;
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    pub fn into_string(self) -> String {
        self.0
    }

    fn validate(&self) -> Result<(), RuntimeInstanceIdError> {
        validate_runtime_instance_id(&self.0)
    }
}

impl Serialize for RuntimeInstanceId {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_str(&self.0)
    }
}

impl<'de> Deserialize<'de> for RuntimeInstanceId {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let value = String::deserialize(deserializer)?;
        Self::new(value).map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AgentRuntimeInstanceClaimV1 {
    pub protocol_version: String,
    pub agent: PrincipalId,
    pub instance_id: RuntimeInstanceId,
}

/// Positive structural/canonical result.
///
/// This type intentionally implements neither `Serialize` nor `Deserialize`.
/// A consumer must reconstruct it by rerunning `qualify_runtime_instance_identity`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualifiedAgentRuntimeIdentityV1 {
    agent: PrincipalId,
    instance_id: RuntimeInstanceId,
    digest: Digest32,
}

impl QualifiedAgentRuntimeIdentityV1 {
    pub fn agent(&self) -> &PrincipalId {
        &self.agent
    }

    pub fn instance_id(&self) -> &RuntimeInstanceId {
        &self.instance_id
    }

    pub fn digest(&self) -> Digest32 {
        self.digest
    }

    pub fn profile(&self) -> &'static str {
        RUNTIME_INSTANCE_IDENTITY_PROFILE
    }

    pub fn protocol_version(&self) -> &'static str {
        AGENT_IDENTITY_PROTOCOL_VERSION
    }
}

pub fn qualify_runtime_instance_identity(
    claim: &AgentRuntimeInstanceClaimV1,
) -> Result<QualifiedAgentRuntimeIdentityV1, AgentIdentityError> {
    if claim.protocol_version != AGENT_IDENTITY_PROTOCOL_VERSION {
        return Err(AgentIdentityError::WrongProtocolVersion);
    }

    // PrincipalId currently has a public tuple field and derived Deserialize.
    // Re-run the canonical institutional constructor so malformed bypass values
    // cannot cross this qualification boundary. Store the constructor-returned
    // value so future constructor hardening cannot diverge from the qualified
    // principal retained in this result.
    let agent = PrincipalId::new(claim.agent.as_str().to_owned())
        .map_err(|_| AgentIdentityError::InvalidAgentPrincipal)?;

    claim
        .instance_id
        .validate()
        .map_err(AgentIdentityError::InvalidRuntimeInstanceId)?;

    let digest = canonical_runtime_instance_digest(&agent, &claim.instance_id);

    Ok(QualifiedAgentRuntimeIdentityV1 {
        agent,
        instance_id: claim.instance_id.clone(),
        digest,
    })
}

fn canonical_runtime_instance_digest(
    agent: &PrincipalId,
    instance_id: &RuntimeInstanceId,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_RUNTIME_INSTANCE_IDENTITY);
    frame(&mut hasher, RUNTIME_INSTANCE_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, AGENT_IDENTITY_PROTOCOL_VERSION.as_bytes());
    frame(&mut hasher, agent.as_str().as_bytes());
    frame(&mut hasher, instance_id.as_str().as_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn validate_runtime_instance_id(value: &str) -> Result<(), RuntimeInstanceIdError> {
    if value.is_empty() {
        return Err(RuntimeInstanceIdError::Empty);
    }
    if value.len() > MAX_RUNTIME_INSTANCE_ID_BYTES {
        return Err(RuntimeInstanceIdError::TooLong);
    }
    if value.trim() != value {
        return Err(RuntimeInstanceIdError::LeadingOrTrailingWhitespace);
    }
    if value.chars().any(char::is_control) {
        return Err(RuntimeInstanceIdError::ControlCharacter);
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RuntimeInstanceIdError {
    Empty,
    TooLong,
    LeadingOrTrailingWhitespace,
    ControlCharacter,
}

impl fmt::Display for RuntimeInstanceIdError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty => write!(f, "runtime instance id is empty"),
            Self::TooLong => write!(f, "runtime instance id exceeds byte limit"),
            Self::LeadingOrTrailingWhitespace => {
                write!(f, "runtime instance id has leading or trailing whitespace")
            }
            Self::ControlCharacter => write!(f, "runtime instance id contains a control character"),
        }
    }
}

impl std::error::Error for RuntimeInstanceIdError {}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AgentIdentityError {
    WrongProtocolVersion,
    InvalidAgentPrincipal,
    InvalidRuntimeInstanceId(RuntimeInstanceIdError),
}

impl fmt::Display for AgentIdentityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong agent identity protocol version"),
            Self::InvalidAgentPrincipal => write!(f, "invalid stable agent PrincipalId"),
            Self::InvalidRuntimeInstanceId(error) => {
                write!(f, "invalid runtime instance id: {error}")
            }
        }
    }
}

impl std::error::Error for AgentIdentityError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn claim(agent: &str, runtime: &str) -> AgentRuntimeInstanceClaimV1 {
        AgentRuntimeInstanceClaimV1 {
            protocol_version: AGENT_IDENTITY_PROTOCOL_VERSION.into(),
            agent: PrincipalId::new(agent).unwrap(),
            instance_id: RuntimeInstanceId::new(runtime).unwrap(),
        }
    }

    fn assert_invalid_agent_principal(value: String) {
        let direct = AgentRuntimeInstanceClaimV1 {
            protocol_version: AGENT_IDENTITY_PROTOCOL_VERSION.into(),
            agent: PrincipalId(value.clone()),
            instance_id: RuntimeInstanceId::new("runtime:1").unwrap(),
        };
        assert_eq!(
            qualify_runtime_instance_identity(&direct).unwrap_err(),
            AgentIdentityError::InvalidAgentPrincipal
        );

        let encoded = serde_json::json!({
            "protocol_version": AGENT_IDENTITY_PROTOCOL_VERSION,
            "agent": value,
            "instance_id": "runtime:1",
        });
        let deserialized: AgentRuntimeInstanceClaimV1 = serde_json::from_value(encoded).unwrap();
        assert_eq!(
            qualify_runtime_instance_identity(&deserialized).unwrap_err(),
            AgentIdentityError::InvalidAgentPrincipal
        );
    }

    #[test]
    fn independent_golden_vector_matches() {
        let qualified = qualify_runtime_instance_identity(&claim(
            "did:example:agent-alpha",
            "runtime:host-a:proc-7",
        ))
        .unwrap();

        assert_eq!(
            qualified.digest(),
            Digest32([
                0x46, 0x6b, 0xee, 0xe8, 0x62, 0xb3, 0x06, 0x83, 0x4a, 0xa7, 0x6b, 0xe0, 0x27, 0xeb,
                0x9c, 0x9f, 0x96, 0x29, 0xe6, 0xe5, 0x50, 0xe1, 0xbd, 0xd5, 0xf0, 0x33, 0xa7, 0xfe,
                0xc1, 0x39, 0x8a, 0x9e,
            ])
        );
    }

    #[test]
    fn stable_principal_substitution_changes_identity() {
        let first =
            qualify_runtime_instance_identity(&claim("did:example:a", "runtime:1")).unwrap();
        let second =
            qualify_runtime_instance_identity(&claim("did:example:b", "runtime:1")).unwrap();
        assert_ne!(first.digest(), second.digest());
    }

    #[test]
    fn runtime_instance_substitution_changes_identity() {
        let first =
            qualify_runtime_instance_identity(&claim("did:example:a", "runtime:1")).unwrap();
        let second =
            qualify_runtime_instance_identity(&claim("did:example:a", "runtime:2")).unwrap();
        assert_ne!(first.digest(), second.digest());
    }

    #[test]
    fn restart_changes_runtime_identity_without_changing_stable_principal() {
        let first =
            qualify_runtime_instance_identity(&claim("did:example:a", "runtime:boot-1")).unwrap();
        let second =
            qualify_runtime_instance_identity(&claim("did:example:a", "runtime:boot-2")).unwrap();
        assert_eq!(first.agent(), second.agent());
        assert_ne!(first.instance_id(), second.instance_id());
        assert_ne!(first.digest(), second.digest());
    }

    #[test]
    fn malformed_legacy_principal_bypasses_fail_closed() {
        for invalid in [String::new(), "   ".into(), "x".repeat(513)] {
            assert_invalid_agent_principal(invalid);
        }
    }

    #[test]
    fn wrong_protocol_fails_closed() {
        let mut value = claim("did:example:a", "runtime:1");
        value.protocol_version = "mycelix-agent-identity-v9".into();
        assert_eq!(
            qualify_runtime_instance_identity(&value).unwrap_err(),
            AgentIdentityError::WrongProtocolVersion
        );
    }

    #[test]
    fn runtime_id_constructor_and_deserializer_enforce_invariant() {
        for invalid in ["", " runtime:1", "runtime:1 ", "runtime:\n1"] {
            assert!(
                RuntimeInstanceId::new(invalid).is_err(),
                "accepted invalid {invalid:?}"
            );
            let json = serde_json::to_string(invalid).unwrap();
            assert!(serde_json::from_str::<RuntimeInstanceId>(&json).is_err());
        }

        let oversized = "x".repeat(MAX_RUNTIME_INSTANCE_ID_BYTES + 1);
        assert_eq!(
            RuntimeInstanceId::new(oversized.clone()),
            Err(RuntimeInstanceIdError::TooLong)
        );
        let json = serde_json::to_string(&oversized).unwrap();
        assert!(serde_json::from_str::<RuntimeInstanceId>(&json).is_err());
    }

    #[test]
    fn runtime_id_round_trip_preserves_exact_bytes() {
        let original = RuntimeInstanceId::new("runtime:é:α").unwrap();
        let json = serde_json::to_string(&original).unwrap();
        let decoded: RuntimeInstanceId = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, original);
        assert_eq!(decoded.as_str().as_bytes(), original.as_str().as_bytes());
    }

    #[test]
    fn unicode_normalization_is_not_silent_identity_equivalence() {
        let nfc = qualify_runtime_instance_identity(&claim("did:example:a", "runtime:é")).unwrap();
        let decomposed =
            qualify_runtime_instance_identity(&claim("did:example:a", "runtime:e\u{301}")).unwrap();
        assert_ne!(
            nfc.instance_id().as_str().as_bytes(),
            decomposed.instance_id().as_str().as_bytes()
        );
        assert_ne!(nfc.digest(), decomposed.digest());
    }
}
