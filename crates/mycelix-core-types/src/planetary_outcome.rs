// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Execution receipts and measured outcome lineage for planetary response.
//!
//! This module closes the response loop without collapsing recommendation,
//! authority, execution, and outcome into one mutable object.
//!
//! ```text
//! ResponseProposal
//!      ↓
//! independently verified authority evidence
//!      ↓
//! ExecutionReceipt
//!      ↓
//! post-execution observations
//!      ↓
//! OutcomeAssessment
//!      ↓
//! calibration / institutional learning
//! ```
//!
//! Authority evidence is referenced by immutable digest. This module does not
//! mint capabilities and does not infer authorization from an executor DID.

use crate::{
    EvidenceClass, ExternalEvidenceRef, Measurement, ResponseProposal, TemporalExtent,
};
use std::{collections::HashSet, fmt};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub const EXECUTION_RECEIPT_SCHEMA_VERSION: u16 = 1;
pub const OUTCOME_ASSESSMENT_SCHEMA_VERSION: u16 = 1;
pub const MAX_OUTCOME_ID_BYTES: usize = 256;
pub const MAX_OUTCOME_LABEL_BYTES: usize = 256;
pub const MAX_EXECUTION_ACTIONS: usize = 128;
pub const MAX_RESOURCE_USES: usize = 128;
pub const MAX_AUTHORITY_REFS: usize = 128;
pub const MAX_EXECUTION_EVIDENCE: usize = 128;
pub const MAX_OUTCOME_COMPARISONS: usize = 256;
pub const MAX_UNEXPECTED_OUTCOMES: usize = 128;

/// Exact response option that was executed.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ResponseExecutionRef {
    pub proposal_id: String,
    pub proposal_digest: String,
    pub option_id: String,
}

impl ResponseExecutionRef {
    pub fn validate(&self) -> Result<(), PlanetaryOutcomeError> {
        require_text("execution.proposal_id", &self.proposal_id, MAX_OUTCOME_ID_BYTES)?;
        require_text("execution.option_id", &self.option_id, MAX_OUTCOME_ID_BYTES)?;
        validate_digest("execution.proposal_digest", &self.proposal_digest)
    }
}

/// Immutable reference to an independently evaluated authority grant/capability.
///
/// `verification_receipt_digest` is deliberately not a boolean `verified` flag.
/// The referenced verifier output remains separately inspectable and can carry
/// revocation, key-resolution and policy-evaluation context.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct AuthorityResolutionRef {
    pub domain: String,
    pub action: String,
    pub jurisdiction: Option<String>,
    pub grant_id: String,
    pub grant_digest: String,
    pub verification_receipt_digest: String,
}

impl AuthorityResolutionRef {
    pub fn validate(&self) -> Result<(), PlanetaryOutcomeError> {
        require_text("execution.authority.domain", &self.domain, MAX_OUTCOME_LABEL_BYTES)?;
        require_text("execution.authority.action", &self.action, MAX_OUTCOME_LABEL_BYTES)?;
        require_text("execution.authority.grant_id", &self.grant_id, MAX_OUTCOME_ID_BYTES)?;
        if let Some(jurisdiction) = &self.jurisdiction {
            require_text(
                "execution.authority.jurisdiction",
                jurisdiction,
                MAX_OUTCOME_LABEL_BYTES,
            )?;
        }
        validate_digest("execution.authority.grant_digest", &self.grant_digest)?;
        validate_digest(
            "execution.authority.verification_receipt_digest",
            &self.verification_receipt_digest,
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum ExecutionStatus {
    Succeeded,
    PartiallySucceeded,
    Failed,
    Aborted,
}

/// Actual resource consumption, not the earlier planned requirement.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ResourceUse {
    pub resource_type: String,
    pub quantity: Measurement,
    /// Evidence-bearing inventory/meter/receipt observation when available.
    pub evidence_observation_id: Option<String>,
}

impl ResourceUse {
    pub fn validate(&self) -> Result<(), PlanetaryOutcomeError> {
        require_text(
            "execution.resource.resource_type",
            &self.resource_type,
            MAX_OUTCOME_LABEL_BYTES,
        )?;
        self.quantity
            .validate()
            .map_err(|error| PlanetaryOutcomeError::InvalidMeasurement(error.to_string()))?;
        if self.quantity.value < 0.0 {
            return Err(PlanetaryOutcomeError::NegativeResourceUse {
                resource_type: self.resource_type.clone(),
                value: self.quantity.value,
            });
        }
        if let Some(id) = &self.evidence_observation_id {
            require_text(
                "execution.resource.evidence_observation_id",
                id,
                MAX_OUTCOME_ID_BYTES,
            )?;
        }
        Ok(())
    }
}

/// Immutable semantic receipt for an intervention that actually reached the
/// execution boundary.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ExecutionReceipt {
    pub schema_version: u16,
    pub id: String,
    pub response: ResponseExecutionRef,
    pub executor_did: String,
    pub execution_window: TemporalExtent,
    pub status: ExecutionStatus,
    pub authority: Vec<AuthorityResolutionRef>,
    pub actions_performed: Vec<String>,
    pub resources_used: Vec<ResourceUse>,
    pub execution_evidence: Vec<ExternalEvidenceRef>,
    /// Post-execution observations expected to support outcome assessment.
    pub outcome_observation_ids: Vec<String>,
}

impl ExecutionReceipt {
    pub fn validate(&self) -> Result<(), PlanetaryOutcomeError> {
        if self.schema_version != EXECUTION_RECEIPT_SCHEMA_VERSION {
            return Err(PlanetaryOutcomeError::UnsupportedExecutionSchema(
                self.schema_version,
            ));
        }
        require_text("execution.id", &self.id, MAX_OUTCOME_ID_BYTES)?;
        require_text("execution.executor_did", &self.executor_did, MAX_OUTCOME_ID_BYTES)?;
        if !self.executor_did.starts_with("did:") {
            return Err(PlanetaryOutcomeError::MalformedExecutorDid);
        }
        self.response.validate()?;
        self.execution_window
            .validate()
            .map_err(|error| PlanetaryOutcomeError::InvalidTemporal(error.to_string()))?;

        if self.authority.is_empty() {
            return Err(PlanetaryOutcomeError::MissingAuthorityEvidence);
        }
        if self.authority.len() > MAX_AUTHORITY_REFS {
            return Err(PlanetaryOutcomeError::TooManyAuthorityRefs {
                actual: self.authority.len(),
                max: MAX_AUTHORITY_REFS,
            });
        }
        let mut authority = HashSet::with_capacity(self.authority.len());
        for reference in &self.authority {
            reference.validate()?;
            if !authority.insert(reference) {
                return Err(PlanetaryOutcomeError::DuplicateAuthorityRef);
            }
        }

        if self.actions_performed.is_empty() {
            return Err(PlanetaryOutcomeError::MissingExecutionActions);
        }
        if self.actions_performed.len() > MAX_EXECUTION_ACTIONS {
            return Err(PlanetaryOutcomeError::TooManyExecutionActions {
                actual: self.actions_performed.len(),
                max: MAX_EXECUTION_ACTIONS,
            });
        }
        let mut actions = HashSet::with_capacity(self.actions_performed.len());
        for action in &self.actions_performed {
            require_text("execution.action", action, MAX_OUTCOME_LABEL_BYTES)?;
            if !actions.insert(action.as_str()) {
                return Err(PlanetaryOutcomeError::DuplicateExecutionAction(action.clone()));
            }
        }

        if self.resources_used.len() > MAX_RESOURCE_USES {
            return Err(PlanetaryOutcomeError::TooManyResourceUses {
                actual: self.resources_used.len(),
                max: MAX_RESOURCE_USES,
            });
        }
        let mut resource_types = HashSet::with_capacity(self.resources_used.len());
        for resource in &self.resources_used {
            resource.validate()?;
            if !resource_types.insert(resource.resource_type.as_str()) {
                return Err(PlanetaryOutcomeError::DuplicateResourceUse(
                    resource.resource_type.clone(),
                ));
            }
        }

        if self.execution_evidence.len() > MAX_EXECUTION_EVIDENCE {
            return Err(PlanetaryOutcomeError::TooMuchExecutionEvidence {
                actual: self.execution_evidence.len(),
                max: MAX_EXECUTION_EVIDENCE,
            });
        }
        let mut evidence = HashSet::with_capacity(self.execution_evidence.len());
        for reference in &self.execution_evidence {
            reference
                .validate()
                .map_err(|error| PlanetaryOutcomeError::InvalidExecutionEvidence(error.to_string()))?;
            if reference.content_digest.is_none() {
                return Err(PlanetaryOutcomeError::MutableExecutionEvidence(
                    reference.resource_id.clone(),
                ));
            }
            if !evidence.insert(reference) {
                return Err(PlanetaryOutcomeError::DuplicateExecutionEvidence);
            }
        }

        let mut outcomes = HashSet::with_capacity(self.outcome_observation_ids.len());
        for id in &self.outcome_observation_ids {
            require_text("execution.outcome_observation_id", id, MAX_OUTCOME_ID_BYTES)?;
            if !outcomes.insert(id.as_str()) {
                return Err(PlanetaryOutcomeError::DuplicateOutcomeObservation(id.clone()));
            }
        }
        Ok(())
    }

    /// Verify relational coverage against the exact proposal object supplied by
    /// the caller. Digest recomputation remains outside this dependency-light
    /// crate; this method checks semantic IDs and authority requirements.
    pub fn validate_against_proposal(
        &self,
        proposal: &ResponseProposal,
    ) -> Result<(), PlanetaryOutcomeError> {
        self.validate()?;
        proposal
            .validate()
            .map_err(|error| PlanetaryOutcomeError::InvalidResponseProposal(error.to_string()))?;
        if self.response.proposal_id != proposal.id {
            return Err(PlanetaryOutcomeError::ProposalIdMismatch);
        }
        let option = proposal
            .options
            .iter()
            .find(|option| option.id == self.response.option_id)
            .ok_or(PlanetaryOutcomeError::UnknownExecutedOption)?;

        for required in &option.authority_requirements {
            let covered = self.authority.iter().any(|actual| {
                actual.domain == required.domain
                    && actual.action == required.action
                    && actual.jurisdiction == required.jurisdiction
            });
            if !covered {
                return Err(PlanetaryOutcomeError::MissingRequiredAuthority {
                    domain: required.domain.clone(),
                    action: required.action.clone(),
                });
            }
        }
        Ok(())
    }
}

/// Evidence class expected for a post-execution observation.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ActualOutcomeRef {
    pub observation_id: String,
    pub expected_class: EvidenceClass,
}

impl ActualOutcomeRef {
    pub fn validate(&self) -> Result<(), PlanetaryOutcomeError> {
        require_text("outcome.actual.observation_id", &self.observation_id, MAX_OUTCOME_ID_BYTES)?;
        if matches!(
            self.expected_class,
            EvidenceClass::Forecast | EvidenceClass::Scenario
        ) {
            return Err(PlanetaryOutcomeError::ProspectiveClassUsedAsActual(
                self.expected_class,
            ));
        }
        Ok(())
    }
}

/// Pair a pre-execution projection with evidence collected after execution.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct OutcomeComparison {
    pub projected_observation_id: String,
    pub actual: ActualOutcomeRef,
    /// Optional evidence-bearing derived observation quantifying error/effect.
    pub comparison_observation_id: Option<String>,
}

impl OutcomeComparison {
    pub fn validate(&self) -> Result<(), PlanetaryOutcomeError> {
        require_text(
            "outcome.projected_observation_id",
            &self.projected_observation_id,
            MAX_OUTCOME_ID_BYTES,
        )?;
        self.actual.validate()?;
        if let Some(id) = &self.comparison_observation_id {
            require_text("outcome.comparison_observation_id", id, MAX_OUTCOME_ID_BYTES)?;
        }
        Ok(())
    }
}

/// Assessment of what happened after one execution receipt.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct OutcomeAssessment {
    pub schema_version: u16,
    pub id: String,
    pub execution_receipt_id: String,
    pub execution_receipt_digest: String,
    pub assessed_at: i64,
    pub comparisons: Vec<OutcomeComparison>,
    pub unexpected_outcomes: Vec<ActualOutcomeRef>,
    /// Optional overall impact product. If present it should be a derived or
    /// inferred evidence observation with its own PEF-2 lineage.
    pub impact_observation_id: Option<String>,
}

impl OutcomeAssessment {
    pub fn validate(&self) -> Result<(), PlanetaryOutcomeError> {
        if self.schema_version != OUTCOME_ASSESSMENT_SCHEMA_VERSION {
            return Err(PlanetaryOutcomeError::UnsupportedOutcomeSchema(
                self.schema_version,
            ));
        }
        require_text("outcome.id", &self.id, MAX_OUTCOME_ID_BYTES)?;
        require_text(
            "outcome.execution_receipt_id",
            &self.execution_receipt_id,
            MAX_OUTCOME_ID_BYTES,
        )?;
        validate_digest(
            "outcome.execution_receipt_digest",
            &self.execution_receipt_digest,
        )?;

        if self.comparisons.len() > MAX_OUTCOME_COMPARISONS {
            return Err(PlanetaryOutcomeError::TooManyComparisons {
                actual: self.comparisons.len(),
                max: MAX_OUTCOME_COMPARISONS,
            });
        }
        let mut projected = HashSet::with_capacity(self.comparisons.len());
        let mut actual = HashSet::with_capacity(self.comparisons.len());
        for comparison in &self.comparisons {
            comparison.validate()?;
            if !projected.insert(comparison.projected_observation_id.as_str()) {
                return Err(PlanetaryOutcomeError::DuplicateProjectedComparison(
                    comparison.projected_observation_id.clone(),
                ));
            }
            if !actual.insert(comparison.actual.observation_id.as_str()) {
                return Err(PlanetaryOutcomeError::DuplicateActualComparison(
                    comparison.actual.observation_id.clone(),
                ));
            }
        }

        if self.unexpected_outcomes.len() > MAX_UNEXPECTED_OUTCOMES {
            return Err(PlanetaryOutcomeError::TooManyUnexpectedOutcomes {
                actual: self.unexpected_outcomes.len(),
                max: MAX_UNEXPECTED_OUTCOMES,
            });
        }
        let mut unexpected = HashSet::with_capacity(self.unexpected_outcomes.len());
        for outcome in &self.unexpected_outcomes {
            outcome.validate()?;
            if actual.contains(outcome.observation_id.as_str())
                || !unexpected.insert(outcome.observation_id.as_str())
            {
                return Err(PlanetaryOutcomeError::DuplicateActualOutcome(
                    outcome.observation_id.clone(),
                ));
            }
        }
        if let Some(id) = &self.impact_observation_id {
            require_text("outcome.impact_observation_id", id, MAX_OUTCOME_ID_BYTES)?;
        }
        Ok(())
    }

    pub fn validate_against_receipt(
        &self,
        receipt: &ExecutionReceipt,
    ) -> Result<(), PlanetaryOutcomeError> {
        self.validate()?;
        receipt.validate()?;
        if self.execution_receipt_id != receipt.id {
            return Err(PlanetaryOutcomeError::ExecutionReceiptIdMismatch);
        }
        let declared: HashSet<&str> = receipt
            .outcome_observation_ids
            .iter()
            .map(String::as_str)
            .collect();
        for comparison in &self.comparisons {
            if !declared.contains(comparison.actual.observation_id.as_str()) {
                return Err(PlanetaryOutcomeError::UndeclaredOutcomeObservation(
                    comparison.actual.observation_id.clone(),
                ));
            }
        }
        for outcome in &self.unexpected_outcomes {
            if !declared.contains(outcome.observation_id.as_str()) {
                return Err(PlanetaryOutcomeError::UndeclaredOutcomeObservation(
                    outcome.observation_id.clone(),
                ));
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum PlanetaryOutcomeError {
    UnsupportedExecutionSchema(u16),
    UnsupportedOutcomeSchema(u16),
    EmptyField(&'static str),
    FieldTooLong { field: &'static str, actual: usize, max: usize },
    MalformedDigest { field: &'static str, reason: &'static str },
    MalformedExecutorDid,
    InvalidTemporal(String),
    InvalidMeasurement(String),
    NegativeResourceUse { resource_type: String, value: f64 },
    MissingAuthorityEvidence,
    TooManyAuthorityRefs { actual: usize, max: usize },
    DuplicateAuthorityRef,
    MissingExecutionActions,
    TooManyExecutionActions { actual: usize, max: usize },
    DuplicateExecutionAction(String),
    TooManyResourceUses { actual: usize, max: usize },
    DuplicateResourceUse(String),
    TooMuchExecutionEvidence { actual: usize, max: usize },
    InvalidExecutionEvidence(String),
    MutableExecutionEvidence(String),
    DuplicateExecutionEvidence,
    DuplicateOutcomeObservation(String),
    InvalidResponseProposal(String),
    ProposalIdMismatch,
    UnknownExecutedOption,
    MissingRequiredAuthority { domain: String, action: String },
    ProspectiveClassUsedAsActual(EvidenceClass),
    TooManyComparisons { actual: usize, max: usize },
    DuplicateProjectedComparison(String),
    DuplicateActualComparison(String),
    TooManyUnexpectedOutcomes { actual: usize, max: usize },
    DuplicateActualOutcome(String),
    ExecutionReceiptIdMismatch,
    UndeclaredOutcomeObservation(String),
}

impl fmt::Display for PlanetaryOutcomeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedExecutionSchema(v) => write!(f, "unsupported execution-receipt schema version {v}"),
            Self::UnsupportedOutcomeSchema(v) => write!(f, "unsupported outcome-assessment schema version {v}"),
            Self::EmptyField(field) => write!(f, "{field} cannot be empty"),
            Self::FieldTooLong { field, actual, max } => write!(f, "{field} is {actual} bytes; maximum is {max}"),
            Self::MalformedDigest { field, reason } => write!(f, "malformed {field}: {reason}"),
            Self::MalformedExecutorDid => write!(f, "executor identity must use a DID"),
            Self::InvalidTemporal(error) => write!(f, "invalid execution temporal extent: {error}"),
            Self::InvalidMeasurement(error) => write!(f, "invalid resource-use measurement: {error}"),
            Self::NegativeResourceUse { resource_type, value } => write!(f, "resource {resource_type} cannot have negative actual use {value}"),
            Self::MissingAuthorityEvidence => write!(f, "execution receipt requires independently resolved authority evidence"),
            Self::TooManyAuthorityRefs { actual, max } => write!(f, "execution receipt has {actual} authority refs; maximum is {max}"),
            Self::DuplicateAuthorityRef => write!(f, "execution receipt contains duplicate authority evidence"),
            Self::MissingExecutionActions => write!(f, "execution receipt requires at least one performed action"),
            Self::TooManyExecutionActions { actual, max } => write!(f, "execution receipt has {actual} actions; maximum is {max}"),
            Self::DuplicateExecutionAction(action) => write!(f, "execution receipt repeats action {action}"),
            Self::TooManyResourceUses { actual, max } => write!(f, "execution receipt has {actual} resource uses; maximum is {max}"),
            Self::DuplicateResourceUse(resource) => write!(f, "execution receipt repeats resource use {resource}"),
            Self::TooMuchExecutionEvidence { actual, max } => write!(f, "execution receipt has {actual} evidence refs; maximum is {max}"),
            Self::InvalidExecutionEvidence(error) => write!(f, "invalid execution evidence: {error}"),
            Self::MutableExecutionEvidence(resource) => write!(f, "execution evidence {resource} must be content-digest bound"),
            Self::DuplicateExecutionEvidence => write!(f, "execution receipt contains duplicate execution evidence"),
            Self::DuplicateOutcomeObservation(id) => write!(f, "execution receipt repeats outcome observation {id}"),
            Self::InvalidResponseProposal(error) => write!(f, "invalid response proposal: {error}"),
            Self::ProposalIdMismatch => write!(f, "execution receipt references a different response proposal"),
            Self::UnknownExecutedOption => write!(f, "execution receipt references an option not present in the response proposal"),
            Self::MissingRequiredAuthority { domain, action } => write!(f, "execution receipt lacks required authority for {domain}:{action}"),
            Self::ProspectiveClassUsedAsActual(class) => write!(f, "post-execution outcome cannot use prospective evidence class {class:?}"),
            Self::TooManyComparisons { actual, max } => write!(f, "outcome assessment has {actual} comparisons; maximum is {max}"),
            Self::DuplicateProjectedComparison(id) => write!(f, "outcome assessment compares projection {id} more than once"),
            Self::DuplicateActualComparison(id) => write!(f, "outcome assessment reuses actual observation {id}"),
            Self::TooManyUnexpectedOutcomes { actual, max } => write!(f, "outcome assessment has {actual} unexpected outcomes; maximum is {max}"),
            Self::DuplicateActualOutcome(id) => write!(f, "outcome assessment repeats actual observation {id}"),
            Self::ExecutionReceiptIdMismatch => write!(f, "outcome assessment references a different execution receipt"),
            Self::UndeclaredOutcomeObservation(id) => write!(f, "outcome observation {id} was not declared by the execution receipt"),
        }
    }
}

impl std::error::Error for PlanetaryOutcomeError {}

fn require_text(field: &'static str, value: &str, max: usize) -> Result<(), PlanetaryOutcomeError> {
    if value.trim().is_empty() {
        return Err(PlanetaryOutcomeError::EmptyField(field));
    }
    if value.len() > max {
        return Err(PlanetaryOutcomeError::FieldTooLong { field, actual: value.len(), max });
    }
    Ok(())
}

fn validate_digest(field: &'static str, digest: &str) -> Result<(), PlanetaryOutcomeError> {
    require_text(field, digest, 256)?;
    let Some((algorithm, value)) = digest.split_once(':') else {
        return Err(PlanetaryOutcomeError::MalformedDigest {
            field,
            reason: "digest must be algorithm-qualified",
        });
    };
    if algorithm.trim().is_empty() || value.trim().is_empty() {
        return Err(PlanetaryOutcomeError::MalformedDigest {
            field,
            reason: "digest algorithm and value must both be non-empty",
        });
    }
    if !algorithm
        .bytes()
        .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'+'))
    {
        return Err(PlanetaryOutcomeError::MalformedDigest {
            field,
            reason: "digest algorithm contains unsupported characters",
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AuthorityRequirement, GeoPoint, InterventionOption, ProjectedOutcomeRef,
        ProjectedOutcomeRole, ResponseProposalMode, Reversibility, RiskAssessmentMode,
        RiskTriggerRef, SpatialExtent,
    };

    fn proposal() -> ResponseProposal {
        ResponseProposal::new(
            "response:jhb:heat:1",
            ResponseProposalMode::OperationalRecommendation,
            RiskTriggerRef {
                assessment_id: "risk:jhb:heat:1".into(),
                assessment_digest: "sha256:risk".into(),
                risk_output_observation_id: "obs:risk:heat".into(),
                expected_mode: RiskAssessmentMode::Operational,
            },
            1_788_900_000,
            None,
            vec![InterventionOption {
                id: "cooling-centers".into(),
                intervention_type: "open_cooling_centers".into(),
                target: SpatialExtent::Point(GeoPoint::new(-26.2041, 28.0473).unwrap()),
                execution_window: TemporalExtent::new(1_788_912_000, 1_788_933_600).unwrap(),
                projected_outcomes: vec![ProjectedOutcomeRef {
                    observation_id: "obs:projected:mortality".into(),
                    expected_class: EvidenceClass::Scenario,
                    role: ProjectedOutcomeRole::Benefit,
                }],
                resources: Vec::new(),
                authority_requirements: vec![AuthorityRequirement {
                    domain: "municipal_emergency".into(),
                    action: "open_cooling_center".into(),
                    jurisdiction: Some("Johannesburg".into()),
                    policy_ref: None,
                }],
                reversibility: Reversibility::Reversible,
                assumptions: Vec::new(),
            }],
        )
        .unwrap()
    }

    fn authority() -> AuthorityResolutionRef {
        AuthorityResolutionRef {
            domain: "municipal_emergency".into(),
            action: "open_cooling_center".into(),
            jurisdiction: Some("Johannesburg".into()),
            grant_id: "grant:1".into(),
            grant_digest: "sha256:grant".into(),
            verification_receipt_digest: "sha256:verification".into(),
        }
    }

    fn receipt() -> ExecutionReceipt {
        ExecutionReceipt {
            schema_version: EXECUTION_RECEIPT_SCHEMA_VERSION,
            id: "execution:heat:1".into(),
            response: ResponseExecutionRef {
                proposal_id: "response:jhb:heat:1".into(),
                proposal_digest: "sha256:proposal".into(),
                option_id: "cooling-centers".into(),
            },
            executor_did: "did:mycelix:executor".into(),
            execution_window: TemporalExtent::new(1_788_913_000, 1_788_920_000).unwrap(),
            status: ExecutionStatus::Succeeded,
            authority: vec![authority()],
            actions_performed: vec!["open_cooling_center".into()],
            resources_used: Vec::new(),
            execution_evidence: Vec::new(),
            outcome_observation_ids: vec!["obs:actual:mortality".into()],
        }
    }

    #[test]
    fn execution_requires_authority_evidence() {
        let mut receipt = receipt();
        receipt.authority.clear();
        assert_eq!(receipt.validate(), Err(PlanetaryOutcomeError::MissingAuthorityEvidence));
    }

    #[test]
    fn execution_authority_must_cover_proposal_requirement() {
        let mut receipt = receipt();
        receipt.authority[0].action = "unrelated_action".into();
        assert!(matches!(
            receipt.validate_against_proposal(&proposal()),
            Err(PlanetaryOutcomeError::MissingRequiredAuthority { .. })
        ));
    }

    #[test]
    fn valid_execution_covers_response_authority_boundary() {
        receipt().validate_against_proposal(&proposal()).unwrap();
    }

    #[test]
    fn actual_outcome_rejects_forecast_or_scenario_classes() {
        for class in [EvidenceClass::Forecast, EvidenceClass::Scenario] {
            let actual = ActualOutcomeRef {
                observation_id: "obs:actual".into(),
                expected_class: class,
            };
            assert!(matches!(
                actual.validate(),
                Err(PlanetaryOutcomeError::ProspectiveClassUsedAsActual(_))
            ));
        }
    }

    #[test]
    fn assessment_must_use_receipt_declared_outcomes() {
        let assessment = OutcomeAssessment {
            schema_version: OUTCOME_ASSESSMENT_SCHEMA_VERSION,
            id: "outcome:1".into(),
            execution_receipt_id: "execution:heat:1".into(),
            execution_receipt_digest: "sha256:execution".into(),
            assessed_at: 1_788_940_000,
            comparisons: vec![OutcomeComparison {
                projected_observation_id: "obs:projected:mortality".into(),
                actual: ActualOutcomeRef {
                    observation_id: "obs:not-declared".into(),
                    expected_class: EvidenceClass::Observed,
                },
                comparison_observation_id: None,
            }],
            unexpected_outcomes: Vec::new(),
            impact_observation_id: None,
        };
        assert!(matches!(
            assessment.validate_against_receipt(&receipt()),
            Err(PlanetaryOutcomeError::UndeclaredOutcomeObservation(_))
        ));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn execution_receipt_round_trips() {
        let original = receipt();
        let encoded = serde_json::to_string(&original).unwrap();
        let decoded: ExecutionReceipt = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, original);
    }
}