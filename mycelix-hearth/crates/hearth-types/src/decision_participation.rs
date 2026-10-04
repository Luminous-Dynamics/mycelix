// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Epistemically explicit participation states for Hearth decisions.
//!
//! These states are deliberately separate from Vote: abstention, evidence
//! requests, and dissent must never be silently converted into substantive
//! vote weight or treated as consent.

use serde::{Deserialize, Serialize};

/// A participant's expressed relationship to a decision.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum DecisionParticipation {
    /// Explicitly support one decision option.
    Support { option: u32 },
    /// Explicitly oppose one decision option.
    Oppose { option: u32 },
    /// The participant declines to express a substantive preference.
    Abstain,
    /// The participant requests additional evidence before making a substantive choice.
    RequestMoreEvidence,
    /// The participant records substantive dissent without automatically blocking the process.
    DissentWithoutBlock,
    /// The participant asks that a materially changed decision be reconsidered.
    ReconsiderationRequested,
}

impl DecisionParticipation {
    /// Whether this state expresses a substantive option.
    ///
    /// Only Support and Oppose carry an option index. Every other state
    /// remains epistemically distinct from a substantive vote.
    pub fn option(&self) -> Option<u32> {
        match self {
            Self::Support { option } | Self::Oppose { option } => Some(*option),
            Self::Abstain
            | Self::RequestMoreEvidence
            | Self::DissentWithoutBlock
            | Self::ReconsiderationRequested => None,
        }
    }

    /// Whether this state should ever be interpreted as consent.
    pub const fn is_consent(&self) -> bool {
        matches!(self, Self::Support { .. })
    }

    /// Whether this state explicitly asks the process to pause or reconsider.
    pub const fn requests_process_change(&self) -> bool {
        matches!(
            self,
            Self::RequestMoreEvidence | Self::ReconsiderationRequested
        )
    }

    /// Whether this state preserves dissent without itself creating a veto.
    pub const fn is_nonblocking_dissent(&self) -> bool {
        matches!(self, Self::DissentWithoutBlock)
    }

    /// Validate the participation state without interpreting its political meaning.
    pub fn validate(&self, option_count: u32) -> Result<(), String> {
        if option_count == 0 {
            return Err("Decision participation requires at least one decision option".into());
        }

        if let Some(option) = self.option() {
            if option >= option_count {
                return Err(format!(
                    "Decision participation option {option} is outside decision option range 0..{}",
                    option_count.saturating_sub(1)
                ));
            }
        }

        Ok(())
    }
}

/// Explicit request to reconsider a historical decision because material evidence changed.
///
/// Reconsideration creates a successor decision; it never mutates the historical result.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DecisionReconsideration {
    pub decision_ref: String,
    pub evidence_refs: Vec<String>,
    pub rationale_ref: Option<String>,
}

impl DecisionReconsideration {
    pub fn validate(&self) -> Result<(), String> {
        if self.decision_ref.trim().is_empty() {
            return Err("Decision reconsideration reference cannot be empty".into());
        }
        if self.evidence_refs.is_empty() {
            return Err("Decision reconsideration requires evidence references".into());
        }
        let mut seen = std::collections::BTreeSet::new();
        for reference in &self.evidence_refs {
            if reference.trim().is_empty() {
                return Err("Decision reconsideration evidence references cannot be empty".into());
            }
            if !seen.insert(reference) {
                return Err(format!(
                    "Duplicate decision reconsideration evidence reference: {reference}"
                ));
            }
        }
        if let Some(reference) = &self.rationale_ref {
            if reference.trim().is_empty() {
                return Err("Decision reconsideration rationale reference cannot be empty".into());
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn substantive_states_carry_only_valid_option_indices() {
        assert_eq!(
            DecisionParticipation::Support { option: 1 }.option(),
            Some(1)
        );
        assert_eq!(
            DecisionParticipation::Oppose { option: 0 }.option(),
            Some(0)
        );
        assert!(DecisionParticipation::Support { option: 2 }.validate(2).is_err());
        assert!(DecisionParticipation::Support { option: 1 }.validate(2).is_ok());
    }

    #[test]
    fn non_substantive_states_are_not_consent() {
        for state in [
            DecisionParticipation::Abstain,
            DecisionParticipation::RequestMoreEvidence,
            DecisionParticipation::DissentWithoutBlock,
            DecisionParticipation::ReconsiderationRequested,
        ] {
            assert!(!state.is_consent());
            assert_eq!(state.option(), None);
        }
        assert!(DecisionParticipation::Support { option: 0 }.is_consent());
    }

    #[test]
    fn evidence_and_reconsideration_requests_are_process_distinct() {
        assert!(DecisionParticipation::RequestMoreEvidence.requests_process_change());
        assert!(DecisionParticipation::ReconsiderationRequested.requests_process_change());
        assert!(!DecisionParticipation::DissentWithoutBlock.requests_process_change());
        assert!(DecisionParticipation::DissentWithoutBlock.is_nonblocking_dissent());
    }

    #[test]
    fn abstention_does_not_become_a_vote_by_default() {
        let state = DecisionParticipation::Abstain;
        assert_eq!(state.option(), None);
        assert!(!state.is_consent());
    }

    #[test]
    fn reconsideration_requires_material_evidence_reference() {
        let mut request = DecisionReconsideration {
            decision_ref: "decision:1".into(),
            evidence_refs: vec![],
            rationale_ref: None,
        };
        assert!(request.validate().is_err());

        request.evidence_refs.push("evidence:changed-fact".into());
        assert!(request.validate().is_ok());

        request.evidence_refs.push("evidence:changed-fact".into());
        assert!(request.validate().is_err());
    }
}
