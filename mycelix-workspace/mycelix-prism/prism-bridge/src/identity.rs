//! Authoritative browser security identities used by capability grants.
//!
//! These types are intentionally distinct from UI tab IDs and from renderer
//! supplied strings. The broker creates and owns them during navigation.

use serde::{Deserialize, Serialize};
use std::fmt;

pub const MAX_IDENTITY_LEN: usize = 256;

/// Stable identifier for a renderer OS process.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct RendererProcessId(pub u64);

impl RendererProcessId {
    pub fn new(value: u64) -> Result<Self, IdentityError> {
        if value == 0 {
            return Err(IdentityError::ZeroId);
        }
        Ok(Self(value))
    }
}

/// Browser security "site": scheme + registrable domain, with explicit
/// preservation of non-host schemes as distinct identities.
///
/// This is an identity token, not a parser. The broker must construct it
/// from its authoritative URL parser rather than trusting renderer input.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct SiteIdentity(String);

impl SiteIdentity {
    pub fn new(value: impl Into<String>) -> Result<Self, IdentityError> {
        let value = value.into();
        validate_identity(&value)?;
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// Agent-cluster identity. It is deliberately opaque: assignment policy
/// belongs to the browser/broker, not the renderer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct AgentClusterId(pub u128);

impl AgentClusterId {
    pub fn new(value: u128) -> Result<Self, IdentityError> {
        if value == 0 {
            return Err(IdentityError::ZeroId);
        }
        Ok(Self(value))
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IdentityError {
    ZeroId,
    InvalidValue,
}

impl fmt::Display for IdentityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroId => f.write_str("identity id must be non-zero"),
            Self::InvalidValue => f.write_str("identity value is invalid"),
        }
    }
}

impl std::error::Error for IdentityError {}

fn validate_identity(value: &str) -> Result<(), IdentityError> {
    if value.is_empty()
        || value.len() > MAX_IDENTITY_LEN
        || value.chars().any(char::is_control)
        || value.chars().any(char::is_whitespace)
    {
        return Err(IdentityError::InvalidValue);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn renderer_process_ids_are_nonzero() {
        assert!(RendererProcessId::new(0).is_err());
        assert!(RendererProcessId::new(1).is_ok());
    }

    #[test]
    fn agent_cluster_ids_are_nonzero() {
        assert!(AgentClusterId::new(0).is_err());
        assert!(AgentClusterId::new(1).is_ok());
    }

    #[test]
    fn site_identity_is_bounded_and_opaque() {
        assert!(SiteIdentity::new("https://example.com").is_ok());
        assert!(SiteIdentity::new("https://example.com/path").is_ok());
        assert!(SiteIdentity::new("").is_err());
        assert!(SiteIdentity::new("https://example.com\n").is_err());
        assert!(SiteIdentity::new("https:// example.com").is_err());
    }
}
