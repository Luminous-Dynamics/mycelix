// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # Economic OS Event Envelope
//!
//! Stable semantic/wire envelope for moving economic events between the
//! Mycelix kernel and external payment, statistics, accounting, and policy
//! systems.
//!
//! The envelope deliberately contains semantic identity and provenance rather
//! than assuming that any one transport format is canonical. Adapters may map
//! it to Holochain entries, ISO 20022 messages, SDMX data, national systems, or
//! other rails while preserving the Mycelix event identity and declaring any
//! representation loss.

use super::{policy_context::EconomicPolicyContext, policy_profile::EconomicOsOperation};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

/// Transport-neutral identity envelope for one economic event.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicOsEnvelope {
    /// Stable event/envelope identifier.
    pub event_id: String,
    /// Economic OS semantic operation.
    pub operation: EconomicOsOperation,
    /// Domain-specific semantic event type.
    pub semantic_type: String,
    /// Policy contexts governing this event. Multiple contexts are supported
    /// for cross-jurisdiction activity.
    pub policy_contexts: Vec<EconomicPolicyContext>,
    /// Actor/entity associated with the event.
    pub actor_ref: String,
    /// Authority reference when an authoritative decision is involved.
    pub authority_ref: Option<String>,
    /// Optional economic-action or scope reference.
    pub scope_ref: Option<String>,
    /// Time at which the economic event occurred.
    pub occurred_at: u64,
    /// Time at which the event was recorded by the host.
    pub recorded_at: u64,
    /// Content identity of the operation-specific payload.
    pub payload_fingerprint: String,
    /// Evidence/provenance references.
    pub evidence_refs: Vec<String>,
    /// Causal/predecessor event references.
    pub causation_refs: Vec<String>,
    /// Forward-compatible extension values.
    pub extensions: BTreeMap<String, String>,
}

impl EconomicOsEnvelope {
    /// Validate structural and temporal invariants.
    pub fn validate(&self) -> Result<(), String> {
        for (name, value) in [
            ("event ID", self.event_id.as_str()),
            ("semantic type", self.semantic_type.as_str()),
            ("policy profile reference", self.policy_profile_ref.as_str()),
            ("actor reference", self.actor_ref.as_str()),
        ] {
            if value.trim().is_empty() {
                return Err(format!("Economic OS {name} cannot be empty"));
            }
        }

        if self.payload_fingerprint.len() != 64
            || !self
                .payload_fingerprint
                .as_bytes()
                .iter()
                .all(u8::is_ascii_hexdigit)
        {
            return Err(
                "Economic OS payload fingerprint must be a 64-character hexadecimal SHA-256"
                    .into(),
            );
        }

        if let Some(authority) = &self.authority_ref {
            if authority.trim().is_empty() {
                return Err("Economic OS authority reference cannot be empty".into());
            }
        }

        if let Some(scope) = &self.scope_ref {
            if scope.trim().is_empty() {
                return Err("Economic OS scope reference cannot be empty".into());
            }
        }

        if self.policy_contexts.is_empty() {
            return Err("Economic OS envelope requires at least one policy context".into());
        }

        let mut unique_roles = BTreeSet::new();
        for context in &self.policy_contexts {
            context.validate()?;
            if context.role != super::policy_context::EconomicPolicyContextRole::Other
                && !unique_roles.insert(context.role)
            {
                return Err(format!(
                    "Duplicate Economic OS policy context role: {:?}",
                    context.role
                ));
            }
        }

        if self.recorded_at < self.occurred_at {
            return Err("Economic OS recorded-at timestamp cannot precede occurred-at".into());
        }

        for (name, values) in [
            ("evidence", &self.evidence_refs),
            ("causation", &self.causation_refs),
        ] {
            let mut seen = BTreeSet::new();
            for value in values {
                if value.trim().is_empty() {
                    return Err(format!("Economic OS {name} references cannot be empty"));
                }
                if !seen.insert(value) {
                    return Err(format!("Duplicate Economic OS {name} reference: {value}"));
                }
            }
        }

        for key in self.extensions.keys() {
            if key.trim().is_empty() {
                return Err("Economic OS extension keys cannot be empty".into());
            }
        }

        Ok(())
    }

    /// Return a deterministic SHA-256 content identity.
    ///
    /// Reference collections are sorted before hashing. The resulting
    /// fingerprint is a tamper-evident semantic identifier within the Mycelix
    /// serialization representation, not a signature or legal proof.
    pub fn fingerprint(&self) -> Result<String, String> {
        self.validate()?;

        let mut evidence_refs = self.evidence_refs.clone();
        let mut causation_refs = self.causation_refs.clone();
        evidence_refs.sort();
        causation_refs.sort();

        let mut policy_contexts = self.policy_contexts.clone();
        policy_contexts.sort_by_key(|context| {
            (
                context.role,
                context.profile_ref.clone(),
                context.profile_fingerprint.clone(),
            )
        });

        let payload = serde_json::json!({
            "version": 2,
            "event_id": self.event_id,
            "operation": self.operation,
            "semantic_type": self.semantic_type,
            "policy_contexts": policy_contexts,
            "actor_ref": self.actor_ref,
            "authority_ref": self.authority_ref,
            "scope_ref": self.scope_ref,
            "occurred_at": self.occurred_at,
            "recorded_at": self.recorded_at,
            "payload_fingerprint": self.payload_fingerprint,
            "evidence_refs": evidence_refs,
            "causation_refs": causation_refs,
            "extensions": self.extensions,
        });

        let canonical = serde_json::to_vec(&payload)
            .map_err(|error| format!("Economic OS envelope canonicalization failed: {error}"))?;

        let mut hasher = Sha256::new();
        hasher.update(b"MYCELIX-ECONOMIC-OS-ENVELOPE-V2\0");
        hasher.update(canonical);
        Ok(hex::encode(hasher.finalize()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::policy_context::EconomicPolicyContextRole;

    fn envelope() -> EconomicOsEnvelope {
        EconomicOsEnvelope {
            event_id: "event:1".into(),
            operation: EconomicOsOperation::Authorize,
            semantic_type: "economic-policy-decision".into(),
            policy_contexts: vec![EconomicPolicyContext {
                role: super::policy_context::EconomicPolicyContextRole::Settlement,
                profile_ref: "profile:za:reference:v1".into(),
                profile_fingerprint: "a".repeat(64),
            }],
            actor_ref: "did:mycelix:actor".into(),
            authority_ref: Some("authority:dao-1".into()),
            scope_ref: Some("scope:1".into()),
            occurred_at: 1_000,
            recorded_at: 1_100,
            payload_fingerprint: "b".repeat(64),
            evidence_refs: vec!["evidence:b".into(), "evidence:a".into()],
            causation_refs: vec!["event:prior".into()],
            extensions: BTreeMap::from([("adapter".into(), "native".into())]),
        }
    }

    #[test]
    fn validates_envelope() {
        assert!(envelope().validate().is_ok());
    }

    #[test]
    fn fingerprint_is_order_independent_for_reference_collections() {
        let left = envelope();
        let mut right = envelope();
        right.evidence_refs.reverse();
        assert_eq!(left.fingerprint().unwrap(), right.fingerprint().unwrap());
    }

    #[test]
    fn multiple_policy_contexts_preserve_cross_jurisdiction_semantics() {
        let mut value = envelope();
        value.policy_contexts.push(EconomicPolicyContext {
            role: EconomicPolicyContextRole::Origin,
            profile_ref: "profile:origin:v1".into(),
            profile_fingerprint: "c".repeat(64),
        });
        value.policy_contexts.push(EconomicPolicyContext {
            role: EconomicPolicyContextRole::Destination,
            profile_ref: "profile:destination:v1".into(),
            profile_fingerprint: "d".repeat(64),
        });

        assert!(value.validate().is_ok());
        assert_ne!(value.fingerprint().unwrap(), envelope().fingerprint().unwrap());
    }

    #[test]
    fn duplicate_structural_policy_roles_are_rejected() {
        let mut value = envelope();
        value.policy_contexts.push(EconomicPolicyContext {
            role: EconomicPolicyContextRole::Settlement,
            profile_ref: "profile:other:v1".into(),
            profile_fingerprint: "c".repeat(64),
        });

        assert!(value.validate().is_err());
    }

    #[test]
    fn payload_identity_changes_envelope_fingerprint() {
        let left = envelope();
        let mut right = left.clone();
        right.payload_fingerprint = "c".repeat(64);
        assert_ne!(left.fingerprint().unwrap(), right.fingerprint().unwrap());
    }

    #[test]
    fn rejects_recorded_time_before_event_time() {
        let mut value = envelope();
        value.recorded_at = 999;
        assert!(value.validate().is_err());
    }

    #[test]
    fn rejects_malformed_profile_identity() {
        let mut value = envelope();
        value.policy_profile_fingerprint = "not-a-hash".into();
        assert!(value.validate().is_err());
    }

    #[test]
    fn rejects_duplicate_evidence_references() {
        let mut value = envelope();
        value.evidence_refs.push("evidence:a".into());
        assert!(value.validate().is_err());
    }
}
