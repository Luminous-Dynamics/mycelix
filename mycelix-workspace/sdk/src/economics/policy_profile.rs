// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # Economic Policy Profiles
//!
//! Policy-neutral configuration boundary for the Mycelix Economic OS.
//!
//! The kernel should not decide whether an economy is sovereign-fiat,
//! dollarised, a monetary-union member, mutual-credit, local-community,
//! commodity-linked, or another regime. A policy profile identifies the
//! jurisdiction and regime context in which measurements, rules, authorities,
//! currencies, and interoperability standards are interpreted.
//!
//! This is intentionally a reference type. It does not itself confer legal
//! authority, monetary sovereignty, or regulatory compliance.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

/// Versioned context in which economic events and policy decisions are interpreted.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicPolicyProfile {
    /// Stable profile identifier.
    pub profile_id: String,
    /// Jurisdiction, union, community, or other governing context.
    pub jurisdiction_ref: String,
    /// Economic/monetary regime identifier, intentionally extensible.
    pub regime_ref: String,
    /// Human/governance-readable policy version.
    pub policy_version: String,
    /// Currencies or settlement units recognized by this profile.
    pub currency_refs: Vec<String>,
    /// Governance/authority references relevant to the profile.
    pub authority_refs: Vec<String>,
    /// Rule identifiers activated by this profile.
    pub policy_rule_refs: Vec<String>,
    /// Standards/adapters used for interoperability.
    pub interoperability_profile_refs: Vec<String>,
    /// Profile effective start.
    pub effective_from: u64,
    /// Optional profile end. An open-ended profile has no end.
    pub effective_until: Option<u64>,
    /// Evidence supporting profile declaration.
    pub evidence_refs: Vec<String>,
    /// Optional predecessor profile for explicit version continuity.
    pub supersedes_profile_ref: Option<String>,
    /// Declaration timestamp.
    pub declared_at: u64,
}

impl EconomicPolicyProfile {
    /// Validate profile structure and collection uniqueness.
    pub fn validate(&self) -> Result<(), String> {
        for (name, value) in [
            ("profile ID", self.profile_id.as_str()),
            ("jurisdiction reference", self.jurisdiction_ref.as_str()),
            ("regime reference", self.regime_ref.as_str()),
            ("policy version", self.policy_version.as_str()),
        ] {
            if value.trim().is_empty() {
                return Err(format!("Economic policy {name} cannot be empty"));
            }
        }

        for (name, values, require_nonempty) in [
            ("currency", &self.currency_refs, false),
            ("authority", &self.authority_refs, false),
            ("policy rule", &self.policy_rule_refs, false),
            (
                "interoperability profile",
                &self.interoperability_profile_refs,
                false,
            ),
            ("evidence", &self.evidence_refs, true),
        ] {
            if require_nonempty && values.is_empty() {
                return Err(format!("Economic policy requires at least one {name} reference"));
            }
            let mut seen = BTreeSet::new();
            for value in values {
                if value.trim().is_empty() {
                    return Err(format!("Economic policy {name} references cannot be empty"));
                }
                if !seen.insert(value) {
                    return Err(format!("Duplicate economic policy {name} reference: {value}"));
                }
            }
        }

        if let Some(until) = self.effective_until {
            if until < self.effective_from {
                return Err(
                    "Economic policy effective-until timestamp cannot precede effective-from"
                        .into(),
                );
            }
        }

        if let Some(supersedes) = &self.supersedes_profile_ref {
            if supersedes.trim().is_empty() {
                return Err("Superseded policy profile reference cannot be empty".into());
            }
            if supersedes == &self.profile_id {
                return Err("Economic policy profile cannot supersede itself".into());
            }
        }

        Ok(())
    }

    /// Whether this profile is active at the supplied timestamp.
    pub fn is_active_at(&self, timestamp: u64) -> bool {
        timestamp >= self.effective_from
            && self
                .effective_until
                .map(|until| timestamp <= until)
                .unwrap_or(true)
    }

    /// Return a deterministic SHA-256 content identity.
    ///
    /// Lists are canonicalized lexicographically before hashing. This gives
    /// stable identity within the Mycelix serde representation while remaining
    /// explicit that this is not a universal cross-language canonicalization
    /// protocol.
    pub fn fingerprint(&self) -> Result<String, String> {
        self.validate()?;

        let mut currency_refs = self.currency_refs.clone();
        let mut authority_refs = self.authority_refs.clone();
        let mut policy_rule_refs = self.policy_rule_refs.clone();
        let mut interoperability_profile_refs = self.interoperability_profile_refs.clone();
        let mut evidence_refs = self.evidence_refs.clone();

        for values in [
            &mut currency_refs,
            &mut authority_refs,
            &mut policy_rule_refs,
            &mut interoperability_profile_refs,
            &mut evidence_refs,
        ] {
            values.sort();
        }

        let payload = serde_json::json!({
            "version": 1,
            "profile_id": self.profile_id,
            "jurisdiction_ref": self.jurisdiction_ref,
            "regime_ref": self.regime_ref,
            "policy_version": self.policy_version,
            "currency_refs": currency_refs,
            "authority_refs": authority_refs,
            "policy_rule_refs": policy_rule_refs,
            "interoperability_profile_refs": interoperability_profile_refs,
            "effective_from": self.effective_from,
            "effective_until": self.effective_until,
            "evidence_refs": evidence_refs,
            "supersedes_profile_ref": self.supersedes_profile_ref,
            "declared_at": self.declared_at,
        });

        let canonical = serde_json::to_vec(&payload)
            .map_err(|error| format!("Economic policy profile canonicalization failed: {error}"))?;

        let mut hasher = Sha256::new();
        hasher.update(b"MYCELIX-ECONOMIC-POLICY-PROFILE-V1\0");
        hasher.update(canonical);
        Ok(hex::encode(hasher.finalize()))
    }
}

/// Stable operation classes for the Economic OS interoperability boundary.
///
/// Implementations may transport these operations through Holochain, HTTP,
/// ISO 20022, SDMX, or other adapters without changing their semantic meaning.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum EconomicOsOperation {
    /// Publish a measurement/evidence observation.
    Observe,
    /// Request or record an authorization decision.
    Authorize,
    /// Create a durable economic state transition.
    Commit,
    /// Initiate or record settlement.
    Settle,
    /// Compare expected and observed execution.
    Reconcile,
    /// Close an action after all required evidence and obligations satisfy policy.
    Finalize,
    /// Export data/evidence through an interoperability profile.
    Publish,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> EconomicPolicyProfile {
        EconomicPolicyProfile {
            profile_id: "profile:za:reference:v1".into(),
            jurisdiction_ref: "jurisdiction:ZA".into(),
            regime_ref: "regime:national-fiat".into(),
            policy_version: "1.0.0".into(),
            currency_refs: vec!["ZAR".into()],
            authority_refs: vec!["authority:treasury".into(), "authority:central-bank".into()],
            policy_rule_refs: vec!["rule:fiscal-v1".into()],
            interoperability_profile_refs: vec![
                "standard:SNA-2025".into(),
                "standard:BPM7".into(),
                "standard:SDMX-3.1".into(),
                "standard:SEEA".into(),
                "standard:ISO-20022".into(),
            ],
            effective_from: 1_000,
            effective_until: None,
            evidence_refs: vec!["evidence:profile".into()],
            supersedes_profile_ref: None,
            declared_at: 1_000,
        }
    }

    #[test]
    fn validates_profile() {
        assert!(profile().validate().is_ok());
    }

    #[test]
    fn fingerprint_is_order_independent_for_lists() {
        let mut left = profile();
        let mut right = profile();

        left.currency_refs = vec!["ZAR".into(), "ZAC".into()];
        right.currency_refs = vec!["ZAC".into(), "ZAR".into()];
        left.authority_refs.reverse();
        right.authority_refs.reverse();
        left.interoperability_profile_refs.reverse();
        assert_eq!(left.fingerprint().unwrap(), right.fingerprint().unwrap());
    }

    #[test]
    fn active_interval_is_explicit() {
        let mut value = profile();
        assert!(!value.is_active_at(999));
        assert!(value.is_active_at(1_000));
        assert!(value.is_active_at(10_000));

        value.effective_until = Some(2_000);
        assert!(value.is_active_at(2_000));
        assert!(!value.is_active_at(2_001));
    }

    #[test]
    fn rejects_invalid_effective_window() {
        let mut value = profile();
        value.effective_until = Some(999);
        assert!(value.validate().is_err());
    }

    #[test]
    fn rejects_duplicate_references() {
        let mut value = profile();
        value.policy_rule_refs.push("rule:fiscal-v1".into());
        assert!(value.validate().is_err());
    }

    #[test]
    fn operations_are_explicitly_separated() {
        assert_ne!(EconomicOsOperation::Observe, EconomicOsOperation::Authorize);
        assert_ne!(EconomicOsOperation::Commit, EconomicOsOperation::Finalize);
        assert_ne!(EconomicOsOperation::Reconcile, EconomicOsOperation::Settle);
    }
}
