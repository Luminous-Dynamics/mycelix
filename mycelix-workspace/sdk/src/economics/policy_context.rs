// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Multi-context policy binding for cross-jurisdiction economic events.

use super::policy_profile::EconomicPolicyProfile;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum EconomicPolicyContextRole {
    Origin,
    Destination,
    Settlement,
    Reporting,
    Other,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicPolicyContext {
    pub role: EconomicPolicyContextRole,
    pub profile_ref: String,
    pub profile_fingerprint: String,
}

impl EconomicPolicyContext {
    /// Validate this context against the exact profile and event timestamp.
    pub fn validate_against_profile(
        &self,
        profile: &EconomicPolicyProfile,
        event_timestamp: u64,
    ) -> Result<(), String> {
        self.validate()?;
        let fingerprint = profile.fingerprint()?;
        if self.profile_ref != profile.profile_id {
            return Err("Policy context profile reference does not match supplied profile".into());
        }
        if self.profile_fingerprint != fingerprint {
            return Err("Policy context profile fingerprint does not match supplied profile".into());
        }
        if !profile.is_active_at(event_timestamp) {
            return Err("Policy context profile is not active at event timestamp".into());
        }
        Ok(())
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.profile_ref.trim().is_empty() {
            return Err("Policy context profile reference cannot be empty".into());
        }
        if self.profile_fingerprint.len() != 64
            || !self
                .profile_fingerprint
                .as_bytes()
                .iter()
                .all(u8::is_ascii_hexdigit)
        {
            return Err("Policy context fingerprint must be 64 hex characters".into());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn context() -> EconomicPolicyContext {
        EconomicPolicyContext {
            role: EconomicPolicyContextRole::Settlement,
            profile_ref: "profile:settlement:v1".into(),
            profile_fingerprint: "a".repeat(64),
        }
    }

    #[test]
    fn validates_context() {
        assert!(context().validate().is_ok());
    }

    #[test]
    fn rejects_empty_profile() {
        let mut value = context();
        value.profile_ref.clear();
        assert!(value.validate().is_err());
    }

    #[test]
    fn rejects_malformed_profile_fingerprint() {
        let mut value = context();
        value.profile_fingerprint = "bad".into();
        assert!(value.validate().is_err());
    }
}
