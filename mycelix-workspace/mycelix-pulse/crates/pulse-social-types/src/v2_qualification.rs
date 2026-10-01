// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Evidence-accounting contract for the V2 durable-message qualification boundary.
//!
//! This module does not perform Holochain retrieval or validation. It defines
//! the information a transport adapter must account for before a V2 inbox may
//! be projected into Chat.

use std::collections::HashSet;

use serde::{Deserialize, Serialize};

use crate::chat_projection::{
    ChatProjectionError, DurableEvidenceIdV1, QualifiedV2MessageV1,
    V2InboxCompletenessV1,
};

pub const V2_QUALIFICATION_SCHEMA_VERSION_V1: u8 = 1;

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum V2CandidateEnumerationV1 {
    /// Enumeration completed for the host's observed live-link set. Deleted
    /// links are counted separately so the adapter cannot silently erase the
    /// distinction between "never observed" and "observed then deleted".
    Complete {
        live_links: u32,
        deleted_links: u32,
    },
    Incomplete {
        skipped_links: u32,
        unreadable_links: u32,
    },
}

impl V2CandidateEnumerationV1 {
    fn validate(&self) -> Result<(), ChatProjectionError> {
        match self {
            Self::Complete { .. } => Ok(()),
            Self::Incomplete {
                skipped_links,
                unreadable_links,
            } if *skipped_links == 0 && *unreadable_links == 0 => {
                Err(ChatProjectionError::IncompleteCandidateEnumeration)
            }
            Self::Incomplete { .. } => Err(ChatProjectionError::IncompleteCandidateEnumeration),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum V2CandidateDispositionV1 {
    Valid,
    Missing,
    Invalid,
    NonV2,
    WrongActionType,
    Unreadable,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct V2CandidateAccountingV1 {
    /// Some dispositions can still have durable identity even when the record
    /// itself cannot be qualified (for example, a known action that is missing).
    pub evidence_id: Option<DurableEvidenceIdV1>,
    pub disposition: V2CandidateDispositionV1,
}

/// Local qualification state observed for one durable V2 record.
///
/// This is deliberately not called a validation receipt. Holochain validation
/// receipts are author-conductor-local and are not a general current-validity certificate.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum V2RecordValidationStateV1 {
    Valid,
    Rejected,
    Unavailable,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct V2QualifiedEvidenceV1 {
    pub evidence_id: DurableEvidenceIdV1,
    pub action_timestamp_micros: i64,
    pub validation_state: V2RecordValidationStateV1,
}

impl V2QualifiedEvidenceV1 {
    pub fn validate(&self) -> Result<(), ChatProjectionError> {
        self.evidence_id.validate()?;
        if self.action_timestamp_micros < 0 {
            return Err(ChatProjectionError::InvalidTimestamp);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct V2InboxQualificationV1 {
    pub schema_version: u8,
    /// Proves that candidate enumeration itself completed. Candidate accounting
    /// cannot compensate for links that were never observed by the adapter.
    pub enumeration: V2CandidateEnumerationV1,

    /// Every discovered candidate, including candidates that cannot cross the
    /// qualification boundary. A candidate must never disappear merely because
    /// its record is missing, invalid, non-V2, or otherwise unreadable.
    pub candidates: Vec<V2CandidateAccountingV1>,

    /// Only candidates with Valid disposition may appear here.
    pub qualified: Vec<QualifiedV2MessageV1>,

    /// One evidence record for every qualified message, in the same logical
    /// set (not necessarily the same ordering) as qualified.
    pub evidence: Vec<V2QualifiedEvidenceV1>,
}

impl V2InboxQualificationV1 {
    pub fn validate(&self) -> Result<(), ChatProjectionError> {
        if self.schema_version != V2_QUALIFICATION_SCHEMA_VERSION_V1 {
            return Err(ChatProjectionError::UnsupportedSchemaVersion(self.schema_version));
        }
        self.enumeration.validate()?;

        if self.qualified.len() != self.evidence.len() {
            return Err(ChatProjectionError::QualificationCountMismatch);
        }

        let mut candidate_evidence = HashSet::new();
        let mut valid_candidate_evidence = HashSet::new();

        for candidate in &self.candidates {
            if let Some(evidence_id) = &candidate.evidence_id {
                evidence_id.validate()?;
                if !candidate_evidence.insert(evidence_id) {
                    return Err(ChatProjectionError::DuplicateEvidenceIdentity);
                }
            }

            if candidate.disposition == V2CandidateDispositionV1::Valid {
                let Some(evidence_id) = &candidate.evidence_id else {
                    return Err(ChatProjectionError::EvidenceIdentityMismatch);
                };
                valid_candidate_evidence.insert(evidence_id.clone());
            }
        }

        let mut qualified_evidence = HashSet::new();
        let mut message_ids = HashSet::new();

        for (message, evidence) in self.qualified.iter().zip(&self.evidence) {
            message.validate()?;
            evidence.validate()?;

            if !message_ids.insert(message.message_id) {
                return Err(ChatProjectionError::DuplicateMessageIdentity);
            }

            if !qualified_evidence.insert(evidence.evidence_id.clone()) {
                return Err(ChatProjectionError::DuplicateEvidenceIdentity);
            }

            if message.evidence_id != evidence.evidence_id {
                return Err(ChatProjectionError::EvidenceIdentityMismatch);
            }

            if evidence.validation_state != V2RecordValidationStateV1::Valid {
                return Err(ChatProjectionError::UnqualifiedEvidence);
            }

            if !valid_candidate_evidence.contains(&evidence.evidence_id) {
                return Err(ChatProjectionError::EvidenceIdentityMismatch);
            }
        }

        if valid_candidate_evidence.len() != self.qualified.len() {
            return Err(ChatProjectionError::MissingQualifiedCandidate);
        }

        if self
            .candidates
            .iter()
            .any(|candidate| candidate.disposition != V2CandidateDispositionV1::Valid)
        {
            return Err(ChatProjectionError::UnqualifiedCandidate);
        }

        Ok(())
    }

    pub fn into_projection_inputs(
        &self,
    ) -> Result<(V2InboxCompletenessV1, Vec<QualifiedV2MessageV1>), ChatProjectionError> {
        self.validate()?;
        Ok((V2InboxCompletenessV1::Complete, self.qualified.clone()))
    }

    pub fn candidate_count(&self) -> usize {
        self.candidates.len()
    }

    pub fn valid_candidate_count(&self) -> usize {
        self.candidates
            .iter()
            .filter(|candidate| candidate.disposition == V2CandidateDispositionV1::Valid)
            .count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chat_projection::{AgentKeyV1, AGENT_KEY_RAW_BYTES};

    fn message(byte: u8) -> QualifiedV2MessageV1 {
        QualifiedV2MessageV1 {
            message_id: [byte; 32],
            evidence_id: DurableEvidenceIdV1(format!("uhCAk{byte:02x}")),
            sender: AgentKeyV1(vec![1; AGENT_KEY_RAW_BYTES]),
            recipient: AgentKeyV1(vec![2; AGENT_KEY_RAW_BYTES]),
            thread_id: None,
            in_reply_to: None,
            created_at_micros: 10,
        }
    }

    fn evidence(byte: u8) -> V2QualifiedEvidenceV1 {
        V2QualifiedEvidenceV1 {
            evidence_id: DurableEvidenceIdV1(format!("uhCAk{byte:02x}")),
            action_timestamp_micros: 11,
            validation_state: V2RecordValidationStateV1::Valid,
        }
    }

    fn qualification(
        candidates: Vec<V2CandidateAccountingV1>,
        messages: Vec<QualifiedV2MessageV1>,
        evidence: Vec<V2QualifiedEvidenceV1>,
    ) -> V2InboxQualificationV1 {
        V2InboxQualificationV1 {
            schema_version: V2_QUALIFICATION_SCHEMA_VERSION_V1,
            enumeration: V2CandidateEnumerationV1::Complete {
                live_links: candidates.len() as u32,
                deleted_links: 0,
            },
            candidates,
            qualified: messages,
            evidence,
        }
    }

    fn valid_candidate(byte: u8) -> V2CandidateAccountingV1 {
        V2CandidateAccountingV1 {
            evidence_id: Some(DurableEvidenceIdV1(format!("uhCAk{byte:02x}"))),
            disposition: V2CandidateDispositionV1::Valid,
        }
    }

    #[test]
    fn valid_candidate_crosses_boundary() {
        let q = qualification(vec![valid_candidate(1)], vec![message(1)], vec![evidence(1)]);
        assert!(q.validate().is_ok());
        assert_eq!(q.candidate_count(), 1);
        assert_eq!(q.valid_candidate_count(), 1);
        let (completeness, inputs) = q.into_projection_inputs().unwrap();
        assert_eq!(completeness, V2InboxCompletenessV1::Complete);
        assert_eq!(inputs.len(), 1);
    }

    #[test]
    fn incomplete_enumeration_blocks_even_an_empty_inbox() {
        let q = V2InboxQualificationV1 {
            schema_version: V2_QUALIFICATION_SCHEMA_VERSION_V1,
            enumeration: V2CandidateEnumerationV1::Incomplete {
                skipped_links: 1,
                unreadable_links: 0,
            },
            candidates: vec![],
            qualified: vec![],
            evidence: vec![],
        };
        assert_eq!(q.validate(), Err(ChatProjectionError::IncompleteCandidateEnumeration));
    }

    #[test]
    fn missing_candidate_is_accounted_for_but_blocks_projection() {
        let q = qualification(
            vec![
                valid_candidate(1),
                V2CandidateAccountingV1 {
                    evidence_id: None,
                    disposition: V2CandidateDispositionV1::Missing,
                },
            ],
            vec![message(1)],
            vec![evidence(1)],
        );
        assert_eq!(q.validate(), Err(ChatProjectionError::UnqualifiedCandidate));
    }

    #[test]
    fn valid_candidate_without_evidence_identity_fails_closed() {
        let q = qualification(
            vec![V2CandidateAccountingV1 {
                evidence_id: None,
                disposition: V2CandidateDispositionV1::Valid,
            }],
            vec![message(1)],
            vec![evidence(1)],
        );
        assert_eq!(q.validate(), Err(ChatProjectionError::EvidenceIdentityMismatch));
    }

    #[test]
    fn evidence_identity_must_match_message_and_candidate() {
        let q = qualification(vec![valid_candidate(1)], vec![message(1)], vec![evidence(2)]);
        assert_eq!(q.validate(), Err(ChatProjectionError::EvidenceIdentityMismatch));
    }

    #[test]
    fn rejected_record_is_explicitly_accounted_but_blocks_projection() {
        let q = qualification(
            vec![V2CandidateAccountingV1 {
                evidence_id: Some(DurableEvidenceIdV1("uhCAk01".into())),
                disposition: V2CandidateDispositionV1::Invalid,
            }],
            vec![],
            vec![],
        );
        assert_eq!(q.validate(), Err(ChatProjectionError::UnqualifiedCandidate));
    }

    #[test]
    fn rejected_validation_state_blocks_projection() {
        let q = qualification(
            vec![valid_candidate(1)],
            vec![message(1)],
            vec![V2QualifiedEvidenceV1 {
                validation_state: V2RecordValidationStateV1::Rejected,
                ..evidence(1)
            }],
        );
        assert_eq!(q.validate(), Err(ChatProjectionError::UnqualifiedEvidence));
    }

    #[test]
    fn missing_qualified_message_is_detected() {
        let q = qualification(
            vec![valid_candidate(1), valid_candidate(2)],
            vec![message(1)],
            vec![evidence(1)],
        );
        assert_eq!(q.validate(), Err(ChatProjectionError::MissingQualifiedCandidate));
    }

    #[test]
    fn count_mismatch_between_qualified_and_evidence_fails_closed() {
        let q = qualification(
            vec![valid_candidate(1)],
            vec![message(1), message(2)],
            vec![evidence(1)],
        );
        assert_eq!(q.validate(), Err(ChatProjectionError::QualificationCountMismatch));
    }

    #[test]
    fn duplicate_candidate_evidence_fails_closed() {
        let q = qualification(
            vec![valid_candidate(1), valid_candidate(1)],
            vec![message(1)],
            vec![evidence(1)],
        );
        assert_eq!(q.validate(), Err(ChatProjectionError::DuplicateEvidenceIdentity));
    }

    #[test]
    fn duplicate_message_identity_fails_closed() {
        let q = qualification(
            vec![valid_candidate(1), valid_candidate(2)],
            vec![message(1), message(1)],
            vec![evidence(1), evidence(2)],
        );
        assert_eq!(q.validate(), Err(ChatProjectionError::DuplicateMessageIdentity));
    }

    #[test]
    fn duplicate_qualified_evidence_fails_closed() {
        let q = qualification(
            vec![valid_candidate(1), valid_candidate(2)],
            vec![message(1), message(2)],
            vec![evidence(1), evidence(1)],
        );
        assert_eq!(q.validate(), Err(ChatProjectionError::DuplicateEvidenceIdentity));
    }

    #[test]
    fn unknown_fields_are_rejected() {
        let encoded = r#"{"schema_version":1,"enumeration":"Complete","candidates":[],"qualified":[],"evidence":[],"authority":"admin"}"#;
        assert!(serde_json::from_str::<V2InboxQualificationV1>(encoded).is_err());
    }
}
