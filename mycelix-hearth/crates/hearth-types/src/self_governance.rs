// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Privacy-preserving self-governance contracts for Hearth.
//!
//! These types deliberately store commitments and references rather than
//! reflective content. They support deliberate choice without making a
//! person's inner life a governance data source.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

/// Kind of self-governed decision being recorded.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum SelfGovernanceDecisionKind {
    PersonalDecision,
    Commitment,
    Consent,
    Delegation,
}

/// Reversibility of the decision or commitment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum SelfGovernanceReversibility {
    EasilyReversible,
    ReversibleWithCost,
    DifficultToReverse,
    Irreversible,
}

/// Disclosure boundary for the self-governance envelope.
///
/// The envelope contains no reflective prose. Private material can remain
/// outside the DHT and be represented only by a cryptographic commitment.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum SelfGovernanceVisibility {
    CommitmentOnly,
    HearthMembers,
    Specified(Vec<String>),
}

/// Privacy-preserving record of deliberate personal agency.
///
/// This is not a wellness score, consent score, reputation input, or
/// governance credential. Its fields are evidence of what the subject chose
/// to disclose, not evidence about the subject's moral or psychological state.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SelfGovernanceEnvelope {
    pub envelope_id: String,
    pub subject_ref: String,
    pub decision_kind: SelfGovernanceDecisionKind,
    pub decision_ref: String,
    pub intention_commitment: String,
    pub declared_values_commitment: Option<String>,
    pub alternatives_commitment: Option<String>,
    pub expected_consequences_commitment: Option<String>,
    pub reflection_commitment: Option<String>,
    pub evidence_refs: Vec<String>,
    pub reversibility: SelfGovernanceReversibility,
    pub visibility: SelfGovernanceVisibility,
    pub supersedes_ref: Option<String>,
    pub outcome_ref: Option<String>,
    pub created_at: u64,
}

impl SelfGovernanceEnvelope {
    /// Validate structure and privacy boundary.
    pub fn validate(&self) -> Result<(), String> {
        for (name, value) in [
            ("envelope ID", self.envelope_id.as_str()),
            ("subject reference", self.subject_ref.as_str()),
            ("decision reference", self.decision_ref.as_str()),
        ] {
            if value.trim().is_empty() {
                return Err(format!("Self-governance {name} cannot be empty"));
            }
            if value.len() > 256 {
                return Err(format!("Self-governance {name} must be <= 256 bytes"));
            }
        }

        validate_commitment("intention", &self.intention_commitment)?;
        for (name, value) in [
            ("declared values", self.declared_values_commitment.as_deref()),
            ("alternatives", self.alternatives_commitment.as_deref()),
            (
                "expected consequences",
                self.expected_consequences_commitment.as_deref(),
            ),
            ("reflection", self.reflection_commitment.as_deref()),
        ] {
            if let Some(commitment) = value {
                validate_commitment(name, commitment)?;
            }
        }

        validate_refs("evidence", &self.evidence_refs)?;
        if self.evidence_refs.len() > 32 {
            return Err("Self-governance evidence references must be <= 32".into());
        }

        if let Some(reference) = &self.supersedes_ref {
            if reference.trim().is_empty() {
                return Err("Self-governance supersedes reference cannot be empty".into());
            }
            if reference == &self.envelope_id {
                return Err("Self-governance envelope cannot supersede itself".into());
            }
        }

        if let Some(reference) = &self.outcome_ref {
            if reference.trim().is_empty() {
                return Err("Self-governance outcome reference cannot be empty".into());
            }
        }

        match &self.visibility {
            SelfGovernanceVisibility::CommitmentOnly
            | SelfGovernanceVisibility::HearthMembers => {}
            SelfGovernanceVisibility::Specified(agents) => {
                if agents.is_empty() || agents.len() > 32 {
                    return Err(
                        "Self-governance specified visibility must contain 1-32 recipients"
                            .into(),
                    );
                }
                let mut unique = BTreeSet::new();
                for agent in agents {
                    if agent.trim().is_empty() {
                        return Err(
                            "Self-governance specified visibility cannot contain empty recipients"
                                .into(),
                        );
                    }
                    if !unique.insert(agent) {
                        return Err(format!(
                            "Duplicate self-governance visibility recipient: {agent}"
                        ));
                    }
                }
            }
        }

        Ok(())
    }

    /// Deterministic content identity.
    ///
    /// Visibility, commitments, evidence, and lineage all participate in the
    /// identity. This does not reveal the committed reflective content.
    pub fn fingerprint(&self) -> Result<String, String> {
        self.validate()?;

        let mut evidence_refs = self.evidence_refs.clone();
        evidence_refs.sort();

        let visibility = match &self.visibility {
            SelfGovernanceVisibility::CommitmentOnly => {
                serde_json::json!({"kind": "CommitmentOnly"})
            }
            SelfGovernanceVisibility::HearthMembers => {
                serde_json::json!({"kind": "HearthMembers"})
            }
            SelfGovernanceVisibility::Specified(agents) => {
                let mut agents = agents.clone();
                agents.sort();
                serde_json::json!({"kind": "Specified", "agents": agents})
            }
        };

        let payload = serde_json::json!({
            "version": 1,
            "envelope_id": self.envelope_id,
            "subject_ref": self.subject_ref,
            "decision_kind": self.decision_kind,
            "decision_ref": self.decision_ref,
            "intention_commitment": self.intention_commitment,
            "declared_values_commitment": self.declared_values_commitment,
            "alternatives_commitment": self.alternatives_commitment,
            "expected_consequences_commitment": self.expected_consequences_commitment,
            "reflection_commitment": self.reflection_commitment,
            "evidence_refs": evidence_refs,
            "reversibility": self.reversibility,
            "visibility": visibility,
            "supersedes_ref": self.supersedes_ref,
            "outcome_ref": self.outcome_ref,
            "created_at": self.created_at,
        });

        let canonical = serde_json::to_vec(&payload)
            .map_err(|error| format!("Self-governance canonicalization failed: {error}"))?;

        let mut hasher = Sha256::new();
        hasher.update(b"MYCELIX-HEARTH-SELF-GOVERNANCE-V1\0");
        hasher.update(canonical);
        Ok(hex::encode(hasher.finalize()))
    }
}

fn validate_commitment(name: &str, value: &str) -> Result<(), String> {
    if value.len() != 64 || !value.as_bytes().iter().all(u8::is_ascii_hexdigit) {
        return Err(format!(
            "Self-governance {name} commitment must be a 64-character hexadecimal SHA-256"
        ));
    }
    Ok(())
}

fn validate_refs(name: &str, refs: &[String]) -> Result<(), String> {
    let mut seen = BTreeSet::new();
    for reference in refs {
        if reference.trim().is_empty() {
            return Err(format!("Self-governance {name} references cannot be empty"));
        }
        if !seen.insert(reference) {
            return Err(format!(
                "Duplicate self-governance {name} reference: {reference}"
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn envelope() -> SelfGovernanceEnvelope {
        SelfGovernanceEnvelope {
            envelope_id: "self:decision:1".into(),
            subject_ref: "did:example:alice".into(),
            decision_kind: SelfGovernanceDecisionKind::PersonalDecision,
            decision_ref: "decision:move:1".into(),
            intention_commitment: "a".repeat(64),
            declared_values_commitment: Some("b".repeat(64)),
            alternatives_commitment: Some("c".repeat(64)),
            expected_consequences_commitment: Some("d".repeat(64)),
            reflection_commitment: Some("e".repeat(64)),
            evidence_refs: vec!["evidence:1".into()],
            reversibility: SelfGovernanceReversibility::ReversibleWithCost,
            visibility: SelfGovernanceVisibility::CommitmentOnly,
            supersedes_ref: None,
            outcome_ref: None,
            created_at: 1_000,
        }
    }

    #[test]
    fn valid_envelope_passes_without_exposing_reflection_content() {
        let value = envelope();
        assert!(value.validate().is_ok());
        assert_eq!(value.fingerprint().unwrap().len(), 64);
    }

    #[test]
    fn malformed_commitment_is_rejected() {
        let mut value = envelope();
        value.reflection_commitment = Some("not-a-hash".into());
        assert!(value.validate().is_err());
    }

    #[test]
    fn specified_visibility_requires_unique_recipients() {
        let mut value = envelope();
        value.visibility = SelfGovernanceVisibility::Specified(vec![
            "agent:alice".into(),
            "agent:bob".into(),
        ]);
        assert!(value.validate().is_ok());

        value
            .visibility = SelfGovernanceVisibility::Specified(vec![
            "agent:alice".into(),
            "agent:alice".into(),
        ]);
        assert!(value.validate().is_err());
    }

    #[test]
    fn inner_reflection_text_is_not_part_of_the_contract() {
        let value = envelope();
        assert!(!serde_json::to_string(&value).unwrap().contains("reflection text"));
    }

    #[test]
    fn visibility_and_commitments_change_identity() {
        let left = envelope();
        let mut right = left.clone();
        right.visibility = SelfGovernanceVisibility::HearthMembers;
        assert_ne!(left.fingerprint().unwrap(), right.fingerprint().unwrap());

        right = left.clone();
        right.reflection_commitment = Some("f".repeat(64));
        assert_ne!(left.fingerprint().unwrap(), right.fingerprint().unwrap());
    }

    #[test]
    fn evidence_order_does_not_change_identity() {
        let left = envelope();
        let mut right = left.clone();
        right.evidence_refs = vec!["evidence:1".into(), "evidence:2".into()];
        let mut equivalent = right.clone();
        equivalent.evidence_refs.reverse();
        assert_eq!(right.fingerprint().unwrap(), equivalent.fingerprint().unwrap());
    }

    #[test]
    fn supersession_cannot_point_to_self() {
        let mut value = envelope();
        value.supersedes_ref = Some(value.envelope_id.clone());
        assert!(value.validate().is_err());
    }
}
