// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Tamper-evident lifecycle binding for planetary/civic response artifacts.
//!
//! This module does not decide, authorize, or execute anything. It binds the
//! immutable artifacts produced by those independent layers into one monotonic
//! lifecycle so stage skipping, artifact substitution, or retroactive rewriting
//! can be detected by consumers.
//!
//! Intended chain:
//!
//! ```text
//! ResponseProposal
//!      ↓
//! ResponseDecisionRecord
//!      ↓
//! AuthorityEvaluationRecord(s)
//!      ↓
//! ExecutionReceipt
//!      ↓
//! OutcomeAssessment
//! ```
//!
//! The concrete decision/authority types deliberately remain independently
//! reviewable modules. This ledger references them by immutable IDs + digests.

use std::{collections::HashSet, fmt};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub const RESPONSE_LIFECYCLE_SCHEMA_VERSION: u16 = 1;
pub const MAX_LIFECYCLE_ID_BYTES: usize = 256;
pub const MAX_ARTIFACT_ID_BYTES: usize = 512;
pub const MAX_AUTHORITY_EVALUATIONS: usize = 128;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum ResponseArtifactKind {
    Proposal,
    Decision,
    AuthorityEvaluation,
    ExecutionReceipt,
    OutcomeAssessment,
}

/// Immutable reference to one lifecycle artifact.
///
/// `digest` is algorithm-qualified (for example `sha256:...`). This type does
/// not prescribe serialization; the canonical-preimage layer is responsible
/// for defining what exact bytes were digested.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ResponseArtifactRef {
    pub kind: ResponseArtifactKind,
    pub id: String,
    pub digest: String,
}

impl ResponseArtifactRef {
    pub fn new(
        kind: ResponseArtifactKind,
        id: impl Into<String>,
        digest: impl Into<String>,
    ) -> Result<Self, ResponseLifecycleError> {
        let artifact = Self {
            kind,
            id: id.into(),
            digest: digest.into(),
        };
        artifact.validate()?;
        Ok(artifact)
    }

    pub fn validate(&self) -> Result<(), ResponseLifecycleError> {
        require_text("lifecycle.artifact.id", &self.id, MAX_ARTIFACT_ID_BYTES)?;
        validate_digest("lifecycle.artifact.digest", &self.digest)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum ResponseLifecycleStage {
    Proposed,
    Decided,
    AuthorityEvaluated,
    Executed,
    Assessed,
}

impl ResponseLifecycleStage {
    fn ordinal(self) -> u8 {
        match self {
            Self::Proposed => 0,
            Self::Decided => 1,
            Self::AuthorityEvaluated => 2,
            Self::Executed => 3,
            Self::Assessed => 4,
        }
    }
}

/// One append-only view of a response lifecycle.
///
/// Absence means "not yet bound", never "not required". A stage is valid only
/// when the artifact set exactly supports that stage.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ResponseLifecycleLedger {
    pub schema_version: u16,
    pub id: String,
    pub stage: ResponseLifecycleStage,
    pub proposal: ResponseArtifactRef,
    pub decision: Option<ResponseArtifactRef>,
    pub authority_evaluations: Vec<ResponseArtifactRef>,
    pub execution: Option<ResponseArtifactRef>,
    pub outcome: Option<ResponseArtifactRef>,
}

impl ResponseLifecycleLedger {
    pub fn proposed(
        id: impl Into<String>,
        proposal: ResponseArtifactRef,
    ) -> Result<Self, ResponseLifecycleError> {
        let ledger = Self {
            schema_version: RESPONSE_LIFECYCLE_SCHEMA_VERSION,
            id: id.into(),
            stage: ResponseLifecycleStage::Proposed,
            proposal,
            decision: None,
            authority_evaluations: Vec::new(),
            execution: None,
            outcome: None,
        };
        ledger.validate()?;
        Ok(ledger)
    }

    pub fn validate(&self) -> Result<(), ResponseLifecycleError> {
        if self.schema_version != RESPONSE_LIFECYCLE_SCHEMA_VERSION {
            return Err(ResponseLifecycleError::UnsupportedSchemaVersion(
                self.schema_version,
            ));
        }
        require_text("lifecycle.id", &self.id, MAX_LIFECYCLE_ID_BYTES)?;

        validate_kind(&self.proposal, ResponseArtifactKind::Proposal, "proposal")?;
        self.proposal.validate()?;

        if let Some(decision) = &self.decision {
            validate_kind(decision, ResponseArtifactKind::Decision, "decision")?;
            decision.validate()?;
        }

        if self.authority_evaluations.len() > MAX_AUTHORITY_EVALUATIONS {
            return Err(ResponseLifecycleError::TooManyAuthorityEvaluations {
                actual: self.authority_evaluations.len(),
                max: MAX_AUTHORITY_EVALUATIONS,
            });
        }
        let mut authority_ids = HashSet::with_capacity(self.authority_evaluations.len());
        for evaluation in &self.authority_evaluations {
            validate_kind(
                evaluation,
                ResponseArtifactKind::AuthorityEvaluation,
                "authority_evaluation",
            )?;
            evaluation.validate()?;
            if !authority_ids.insert((evaluation.id.as_str(), evaluation.digest.as_str())) {
                return Err(ResponseLifecycleError::DuplicateAuthorityEvaluation(
                    evaluation.id.clone(),
                ));
            }
        }

        if let Some(execution) = &self.execution {
            validate_kind(
                execution,
                ResponseArtifactKind::ExecutionReceipt,
                "execution",
            )?;
            execution.validate()?;
        }
        if let Some(outcome) = &self.outcome {
            validate_kind(
                outcome,
                ResponseArtifactKind::OutcomeAssessment,
                "outcome",
            )?;
            outcome.validate()?;
        }

        self.validate_stage_shape()
    }

    fn validate_stage_shape(&self) -> Result<(), ResponseLifecycleError> {
        let has_decision = self.decision.is_some();
        let has_authority = !self.authority_evaluations.is_empty();
        let has_execution = self.execution.is_some();
        let has_outcome = self.outcome.is_some();

        let valid = match self.stage {
            ResponseLifecycleStage::Proposed => {
                !has_decision && !has_authority && !has_execution && !has_outcome
            }
            ResponseLifecycleStage::Decided => {
                has_decision && !has_authority && !has_execution && !has_outcome
            }
            ResponseLifecycleStage::AuthorityEvaluated => {
                has_decision && has_authority && !has_execution && !has_outcome
            }
            ResponseLifecycleStage::Executed => {
                has_decision && has_authority && has_execution && !has_outcome
            }
            ResponseLifecycleStage::Assessed => {
                has_decision && has_authority && has_execution && has_outcome
            }
        };

        if valid {
            Ok(())
        } else {
            Err(ResponseLifecycleError::StageArtifactMismatch(self.stage))
        }
    }

    /// Validate an append-only successor state.
    ///
    /// The lifecycle may advance by exactly one stage. While authority is being
    /// evaluated, additional immutable evaluations may be appended without a
    /// stage change. Existing artifact bindings can never be replaced.
    pub fn validate_successor(&self, next: &Self) -> Result<(), ResponseLifecycleError> {
        self.validate()?;
        next.validate()?;

        if self.id != next.id || self.proposal != next.proposal {
            return Err(ResponseLifecycleError::ImmutableRootChanged);
        }
        if self.schema_version != next.schema_version {
            return Err(ResponseLifecycleError::ImmutableRootChanged);
        }

        ensure_optional_prefix("decision", &self.decision, &next.decision)?;
        ensure_optional_prefix("execution", &self.execution, &next.execution)?;
        ensure_optional_prefix("outcome", &self.outcome, &next.outcome)?;

        if next.authority_evaluations.len() < self.authority_evaluations.len()
            || !next
                .authority_evaluations
                .starts_with(&self.authority_evaluations)
        {
            return Err(ResponseLifecycleError::AuthorityHistoryRewritten);
        }

        let current = self.stage.ordinal();
        let following = next.stage.ordinal();
        if following < current || following > current + 1 {
            return Err(ResponseLifecycleError::InvalidStageTransition {
                from: self.stage,
                to: next.stage,
            });
        }

        if following == current {
            if self.stage != ResponseLifecycleStage::AuthorityEvaluated
                || next.authority_evaluations.len() <= self.authority_evaluations.len()
            {
                return Err(ResponseLifecycleError::NoProgress);
            }
        }

        Ok(())
    }

    /// Lifecycle metadata is evidence, not execution authority.
    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

fn ensure_optional_prefix(
    field: &'static str,
    previous: &Option<ResponseArtifactRef>,
    next: &Option<ResponseArtifactRef>,
) -> Result<(), ResponseLifecycleError> {
    match (previous, next) {
        (Some(left), Some(right)) if left != right => {
            Err(ResponseLifecycleError::ArtifactRewritten(field))
        }
        (Some(_), None) => Err(ResponseLifecycleError::ArtifactRemoved(field)),
        _ => Ok(()),
    }
}

fn validate_kind(
    artifact: &ResponseArtifactRef,
    expected: ResponseArtifactKind,
    field: &'static str,
) -> Result<(), ResponseLifecycleError> {
    if artifact.kind == expected {
        Ok(())
    } else {
        Err(ResponseLifecycleError::WrongArtifactKind {
            field,
            expected,
            actual: artifact.kind,
        })
    }
}

fn require_text(
    field: &'static str,
    value: &str,
    max: usize,
) -> Result<(), ResponseLifecycleError> {
    if value.trim().is_empty() {
        return Err(ResponseLifecycleError::EmptyField(field));
    }
    if value.len() > max {
        return Err(ResponseLifecycleError::FieldTooLong {
            field,
            actual: value.len(),
            max,
        });
    }
    Ok(())
}

fn validate_digest(field: &'static str, digest: &str) -> Result<(), ResponseLifecycleError> {
    require_text(field, digest, 256)?;
    let Some((algorithm, value)) = digest.split_once(':') else {
        return Err(ResponseLifecycleError::MalformedDigest {
            field,
            reason: "digest must be algorithm-qualified",
        });
    };
    if algorithm.trim().is_empty() || value.trim().is_empty() {
        return Err(ResponseLifecycleError::MalformedDigest {
            field,
            reason: "digest algorithm and value must both be non-empty",
        });
    }
    if !algorithm
        .bytes()
        .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'+'))
    {
        return Err(ResponseLifecycleError::MalformedDigest {
            field,
            reason: "digest algorithm contains unsupported characters",
        });
    }
    Ok(())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ResponseLifecycleError {
    UnsupportedSchemaVersion(u16),
    EmptyField(&'static str),
    FieldTooLong {
        field: &'static str,
        actual: usize,
        max: usize,
    },
    MalformedDigest {
        field: &'static str,
        reason: &'static str,
    },
    WrongArtifactKind {
        field: &'static str,
        expected: ResponseArtifactKind,
        actual: ResponseArtifactKind,
    },
    TooManyAuthorityEvaluations {
        actual: usize,
        max: usize,
    },
    DuplicateAuthorityEvaluation(String),
    StageArtifactMismatch(ResponseLifecycleStage),
    ImmutableRootChanged,
    ArtifactRewritten(&'static str),
    ArtifactRemoved(&'static str),
    AuthorityHistoryRewritten,
    InvalidStageTransition {
        from: ResponseLifecycleStage,
        to: ResponseLifecycleStage,
    },
    NoProgress,
}

impl fmt::Display for ResponseLifecycleError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedSchemaVersion(version) => {
                write!(f, "unsupported response-lifecycle schema version {version}")
            }
            Self::EmptyField(field) => write!(f, "{field} cannot be empty"),
            Self::FieldTooLong { field, actual, max } => {
                write!(f, "{field} is {actual} bytes; maximum is {max}")
            }
            Self::MalformedDigest { field, reason } => {
                write!(f, "malformed {field}: {reason}")
            }
            Self::WrongArtifactKind {
                field,
                expected,
                actual,
            } => write!(
                f,
                "{field} has artifact kind {actual:?}; expected {expected:?}"
            ),
            Self::TooManyAuthorityEvaluations { actual, max } => write!(
                f,
                "lifecycle has {actual} authority evaluations; maximum is {max}"
            ),
            Self::DuplicateAuthorityEvaluation(id) => {
                write!(f, "duplicate authority evaluation {id}")
            }
            Self::StageArtifactMismatch(stage) => {
                write!(f, "artifact set does not match lifecycle stage {stage:?}")
            }
            Self::ImmutableRootChanged => {
                write!(f, "lifecycle ID, schema, or proposal root cannot change")
            }
            Self::ArtifactRewritten(field) => write!(f, "bound {field} artifact cannot change"),
            Self::ArtifactRemoved(field) => write!(f, "bound {field} artifact cannot be removed"),
            Self::AuthorityHistoryRewritten => {
                write!(f, "authority-evaluation history must be append-only")
            }
            Self::InvalidStageTransition { from, to } => {
                write!(f, "invalid lifecycle stage transition {from:?} -> {to:?}")
            }
            Self::NoProgress => write!(f, "successor does not advance lifecycle evidence"),
        }
    }
}

impl std::error::Error for ResponseLifecycleError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn artifact(kind: ResponseArtifactKind, id: &str) -> ResponseArtifactRef {
        ResponseArtifactRef::new(kind, id, format!("sha256:{id}-digest")).unwrap()
    }

    fn proposed() -> ResponseLifecycleLedger {
        ResponseLifecycleLedger::proposed(
            "lifecycle:jhb:heat:1",
            artifact(ResponseArtifactKind::Proposal, "response:jhb:heat:1"),
        )
        .unwrap()
    }

    #[test]
    fn proposed_stage_is_minimal_and_non_authoritative() {
        let ledger = proposed();
        ledger.validate().unwrap();
        assert!(!ledger.grants_execution_authority());
    }

    #[test]
    fn stage_cannot_skip_decision() {
        let mut next = proposed();
        next.stage = ResponseLifecycleStage::AuthorityEvaluated;
        next.authority_evaluations.push(artifact(
            ResponseArtifactKind::AuthorityEvaluation,
            "authority:1",
        ));
        assert_eq!(
            next.validate(),
            Err(ResponseLifecycleError::StageArtifactMismatch(
                ResponseLifecycleStage::AuthorityEvaluated
            ))
        );
    }

    #[test]
    fn monotonic_lifecycle_accepts_one_stage_at_a_time() {
        let initial = proposed();

        let mut decided = initial.clone();
        decided.stage = ResponseLifecycleStage::Decided;
        decided.decision = Some(artifact(ResponseArtifactKind::Decision, "decision:1"));
        initial.validate_successor(&decided).unwrap();

        let mut authorized = decided.clone();
        authorized.stage = ResponseLifecycleStage::AuthorityEvaluated;
        authorized.authority_evaluations.push(artifact(
            ResponseArtifactKind::AuthorityEvaluation,
            "authority:1",
        ));
        decided.validate_successor(&authorized).unwrap();

        let mut executed = authorized.clone();
        executed.stage = ResponseLifecycleStage::Executed;
        executed.execution = Some(artifact(
            ResponseArtifactKind::ExecutionReceipt,
            "execution:1",
        ));
        authorized.validate_successor(&executed).unwrap();

        let mut assessed = executed.clone();
        assessed.stage = ResponseLifecycleStage::Assessed;
        assessed.outcome = Some(artifact(
            ResponseArtifactKind::OutcomeAssessment,
            "outcome:1",
        ));
        executed.validate_successor(&assessed).unwrap();
    }

    #[test]
    fn artifact_substitution_is_rejected() {
        let initial = proposed();
        let mut decided = initial.clone();
        decided.stage = ResponseLifecycleStage::Decided;
        decided.decision = Some(artifact(ResponseArtifactKind::Decision, "decision:1"));
        initial.validate_successor(&decided).unwrap();

        let mut rewritten = decided.clone();
        rewritten.decision = Some(artifact(ResponseArtifactKind::Decision, "decision:evil"));
        assert!(matches!(
            decided.validate_successor(&rewritten),
            Err(ResponseLifecycleError::ArtifactRewritten("decision"))
        ));
    }

    #[test]
    fn stage_skip_is_rejected_even_if_artifacts_exist() {
        let initial = proposed();
        let mut executed = initial.clone();
        executed.stage = ResponseLifecycleStage::Executed;
        executed.decision = Some(artifact(ResponseArtifactKind::Decision, "decision:1"));
        executed.authority_evaluations.push(artifact(
            ResponseArtifactKind::AuthorityEvaluation,
            "authority:1",
        ));
        executed.execution = Some(artifact(
            ResponseArtifactKind::ExecutionReceipt,
            "execution:1",
        ));
        executed.validate().unwrap();
        assert!(matches!(
            initial.validate_successor(&executed),
            Err(ResponseLifecycleError::InvalidStageTransition { .. })
        ));
    }

    #[test]
    fn authority_evaluations_are_append_only_within_authority_stage() {
        let mut current = proposed();
        current.stage = ResponseLifecycleStage::Decided;
        current.decision = Some(artifact(ResponseArtifactKind::Decision, "decision:1"));

        let mut authorized = current.clone();
        authorized.stage = ResponseLifecycleStage::AuthorityEvaluated;
        authorized.authority_evaluations.push(artifact(
            ResponseArtifactKind::AuthorityEvaluation,
            "authority:1",
        ));
        current.validate_successor(&authorized).unwrap();

        let mut more = authorized.clone();
        more.authority_evaluations.push(artifact(
            ResponseArtifactKind::AuthorityEvaluation,
            "authority:2",
        ));
        authorized.validate_successor(&more).unwrap();
    }

    #[test]
    fn authority_history_cannot_be_reordered() {
        let mut current = proposed();
        current.stage = ResponseLifecycleStage::AuthorityEvaluated;
        current.decision = Some(artifact(ResponseArtifactKind::Decision, "decision:1"));
        current.authority_evaluations = vec![
            artifact(ResponseArtifactKind::AuthorityEvaluation, "authority:1"),
            artifact(ResponseArtifactKind::AuthorityEvaluation, "authority:2"),
        ];
        current.validate().unwrap();

        let mut reordered = current.clone();
        reordered.authority_evaluations.swap(0, 1);
        assert_eq!(
            current.validate_successor(&reordered),
            Err(ResponseLifecycleError::AuthorityHistoryRewritten)
        );
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trip_preserves_stage_and_digests() {
        let ledger = proposed();
        let encoded = serde_json::to_string(&ledger).unwrap();
        let decoded: ResponseLifecycleLedger = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, ledger);
    }
}
