// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Concrete semantic binding for the planetary response lifecycle.
//!
//! `planetary_loop` intentionally binds generic immutable artifact references.
//! This module closes the next semantic gap by validating the concrete
//! `ResponseDecisionRecord` and `AuthorityEvaluationRecord` objects against the
//! same proposal/lifecycle root.
//!
//! None of these helpers grant execution authority. In particular, an
//! `AuthorityEvaluationResult::Allowed` remains an unauthenticated semantic
//! claim until a separate capability/authority boundary verifies it.

use crate::{
    validate_lifecycle_execution_binding, AuthorityEvaluationRecord, AuthorityEvaluationResult,
    AuthorityRequirement, ExecutionReceipt, ResponseArtifactKind, ResponseDecisionRecord,
    ResponseDisposition, ResponseLifecycleLedger, ResponseLifecycleStage, ResponseProposal,
};
use std::{collections::HashSet, fmt};

/// Concrete meaning of the final response decision.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DecisionGate {
    Selected { option_id: String },
    Rejected,
    Deferred,
    RevisionRequested,
}

impl DecisionGate {
    pub fn selected_option_id(&self) -> Option<&str> {
        match self {
            Self::Selected { option_id } => Some(option_id.as_str()),
            Self::Rejected | Self::Deferred | Self::RevisionRequested => None,
        }
    }

    /// A decision can select an action path, but never grant execution authority.
    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

/// Bind a concrete decision to the exact lifecycle proposal root.
///
/// A non-selection decision (`RejectAll`, `Defer`, `RequestRevision`) is terminal
/// for this lifecycle instance unless a later protocol creates a new/revised
/// proposal and therefore a new lifecycle root. It cannot be followed by
/// authority evaluation or execution in the same ledger.
pub fn validate_decision_binding(
    ledger: &ResponseLifecycleLedger,
    proposal: &ResponseProposal,
    decision: &ResponseDecisionRecord,
) -> Result<DecisionGate, ConcreteLoopError> {
    ledger
        .validate()
        .map_err(|error| ConcreteLoopError::Lifecycle(error.to_string()))?;
    proposal
        .validate()
        .map_err(|error| ConcreteLoopError::Proposal(error.to_string()))?;
    decision
        .validate_against_proposal(proposal)
        .map_err(|error| ConcreteLoopError::Decision(error.to_string()))?;

    if ledger.proposal.id != proposal.id
        || decision.proposal.proposal_id != proposal.id
    {
        return Err(ConcreteLoopError::ProposalIdMismatch);
    }
    if decision.proposal.proposal_digest != ledger.proposal.digest {
        return Err(ConcreteLoopError::ProposalDigestMismatch);
    }

    let bound_decision = ledger
        .decision
        .as_ref()
        .ok_or(ConcreteLoopError::DecisionNotBound)?;
    if bound_decision.kind != ResponseArtifactKind::Decision || bound_decision.id != decision.id {
        return Err(ConcreteLoopError::DecisionArtifactMismatch);
    }

    let gate = match &decision.disposition {
        ResponseDisposition::SelectOption { option_id } => DecisionGate::Selected {
            option_id: option_id.clone(),
        },
        ResponseDisposition::RejectAll => DecisionGate::Rejected,
        ResponseDisposition::Defer { .. } => DecisionGate::Deferred,
        ResponseDisposition::RequestRevision => DecisionGate::RevisionRequested,
    };

    if gate.selected_option_id().is_none()
        && ledger.stage > ResponseLifecycleStage::Decided
    {
        return Err(ConcreteLoopError::NonSelectionAdvanced {
            stage: ledger.stage,
        });
    }

    Ok(gate)
}

/// Explainable semantic coverage for one declared authority requirement.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthorityRequirementCoverage {
    pub requirement: AuthorityRequirement,
    pub evaluation_ids: Vec<String>,
    pub current_allowed_claims: usize,
    pub inactive_allowed_claims: usize,
    pub denied_claims: usize,
    pub indeterminate_claims: usize,
    pub not_applicable_claims: usize,
}

impl AuthorityRequirementCoverage {
    pub fn was_evaluated(&self) -> bool {
        !self.evaluation_ids.is_empty()
    }

    /// Whether at least one record currently *claims* Allowed.
    /// This is deliberately not an authorization result.
    pub fn has_current_allowed_claim(&self) -> bool {
        self.current_allowed_claims > 0
    }

    pub fn has_disagreement(&self) -> bool {
        self.denied_claims > 0 || self.indeterminate_claims > 0
    }
}

/// Semantic authority-evaluation coverage for the option actually selected by
/// the decision process.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthorityCoverageReport {
    pub option_id: String,
    pub requirements: Vec<AuthorityRequirementCoverage>,
}

impl AuthorityCoverageReport {
    pub fn all_requirements_evaluated(&self) -> bool {
        self.requirements
            .iter()
            .all(AuthorityRequirementCoverage::was_evaluated)
    }

    /// Every requirement has at least one currently-Allowed *claim*.
    /// This remains weaker than authenticated authority/capability resolution.
    pub fn all_requirements_have_current_allowed_claim(&self) -> bool {
        self.requirements
            .iter()
            .all(AuthorityRequirementCoverage::has_current_allowed_claim)
    }

    pub fn has_disagreement(&self) -> bool {
        self.requirements
            .iter()
            .any(AuthorityRequirementCoverage::has_disagreement)
    }

    /// Coverage reports never grant execution authority.
    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

/// Validate every concrete authority evaluation bound into the lifecycle and
/// produce a non-authoritative coverage report.
///
/// Strong invariants:
/// - the decision must select an actual proposal option;
/// - the lifecycle must have reached the authority-evaluation stage;
/// - concrete evaluation IDs must match the lifecycle's bound authority IDs;
/// - authority IDs are unique even if an attacker supplies different digests;
/// - every evaluation binds the same proposal digest and selected option;
/// - every evaluated requirement is actually declared by that option.
pub fn evaluate_authority_coverage(
    ledger: &ResponseLifecycleLedger,
    proposal: &ResponseProposal,
    decision: &ResponseDecisionRecord,
    evaluations: &[AuthorityEvaluationRecord],
    unix_seconds: i64,
) -> Result<AuthorityCoverageReport, ConcreteLoopError> {
    let gate = validate_decision_binding(ledger, proposal, decision)?;
    let option_id = gate
        .selected_option_id()
        .ok_or(ConcreteLoopError::AuthorityAfterNonSelection)?
        .to_string();

    if ledger.stage < ResponseLifecycleStage::AuthorityEvaluated {
        return Err(ConcreteLoopError::AuthorityStageNotReached);
    }

    // LOOP-1 currently guarantees exact artifact refs but historically allowed
    // the same human ID with a different digest. Concrete binding is stricter:
    // one authority-evaluation ID names one semantic record lineage.
    let mut bound_ids = HashSet::with_capacity(ledger.authority_evaluations.len());
    for reference in &ledger.authority_evaluations {
        if reference.kind != ResponseArtifactKind::AuthorityEvaluation {
            return Err(ConcreteLoopError::AuthorityArtifactKindMismatch);
        }
        if !bound_ids.insert(reference.id.as_str()) {
            return Err(ConcreteLoopError::DuplicateBoundAuthorityId(
                reference.id.clone(),
            ));
        }
    }

    let mut concrete_ids = HashSet::with_capacity(evaluations.len());
    for evaluation in evaluations {
        if !concrete_ids.insert(evaluation.id.as_str()) {
            return Err(ConcreteLoopError::DuplicateConcreteAuthorityId(
                evaluation.id.clone(),
            ));
        }
    }

    if bound_ids.len() != concrete_ids.len() || bound_ids != concrete_ids {
        return Err(ConcreteLoopError::AuthorityArtifactSetMismatch);
    }

    let option = proposal
        .options
        .iter()
        .find(|option| option.id == option_id)
        .ok_or_else(|| ConcreteLoopError::SelectedOptionMissing(option_id.clone()))?;

    let mut requirements = option
        .authority_requirements
        .iter()
        .cloned()
        .map(|requirement| AuthorityRequirementCoverage {
            requirement,
            evaluation_ids: Vec::new(),
            current_allowed_claims: 0,
            inactive_allowed_claims: 0,
            denied_claims: 0,
            indeterminate_claims: 0,
            not_applicable_claims: 0,
        })
        .collect::<Vec<_>>();

    for evaluation in evaluations {
        evaluation
            .validate_against_proposal(proposal)
            .map_err(|error| ConcreteLoopError::Authority(error.to_string()))?;

        if evaluation.response_authority.proposal_digest != ledger.proposal.digest {
            return Err(ConcreteLoopError::AuthorityProposalDigestMismatch {
                evaluation_id: evaluation.id.clone(),
            });
        }
        if evaluation.response_authority.option_id != option_id {
            return Err(ConcreteLoopError::AuthorityOptionMismatch {
                evaluation_id: evaluation.id.clone(),
                expected: option_id.clone(),
                actual: evaluation.response_authority.option_id.clone(),
            });
        }

        let coverage = requirements
            .iter_mut()
            .find(|coverage| coverage.requirement == evaluation.response_authority.requirement)
            .ok_or_else(|| ConcreteLoopError::AuthorityRequirementMismatch {
                evaluation_id: evaluation.id.clone(),
            })?;
        coverage.evaluation_ids.push(evaluation.id.clone());

        match evaluation.result {
            AuthorityEvaluationResult::Allowed if evaluation.claims_allowed_at(unix_seconds) => {
                coverage.current_allowed_claims += 1;
            }
            AuthorityEvaluationResult::Allowed => coverage.inactive_allowed_claims += 1,
            AuthorityEvaluationResult::Denied => coverage.denied_claims += 1,
            AuthorityEvaluationResult::Indeterminate => coverage.indeterminate_claims += 1,
            AuthorityEvaluationResult::NotApplicable => coverage.not_applicable_claims += 1,
        }
    }

    Ok(AuthorityCoverageReport {
        option_id,
        requirements,
    })
}

/// Bind the execution receipt to the option that the concrete decision actually
/// selected. The existing LOOP-1 helper still validates lifecycle/proposal and
/// execution digest continuity; this function adds decision semantics.
pub fn validate_decision_execution_binding(
    ledger: &ResponseLifecycleLedger,
    proposal: &ResponseProposal,
    decision: &ResponseDecisionRecord,
    execution: &ExecutionReceipt,
) -> Result<(), ConcreteLoopError> {
    let gate = validate_decision_binding(ledger, proposal, decision)?;
    let selected = gate
        .selected_option_id()
        .ok_or(ConcreteLoopError::ExecutionAfterNonSelection)?;

    if ledger.stage < ResponseLifecycleStage::Executed {
        return Err(ConcreteLoopError::ExecutionStageNotReached);
    }

    validate_lifecycle_execution_binding(ledger, proposal, execution)
        .map_err(|error| ConcreteLoopError::Execution(error.to_string()))?;

    if execution.response.option_id != selected {
        return Err(ConcreteLoopError::ExecutedOptionMismatch {
            selected: selected.to_string(),
            executed: execution.response.option_id.clone(),
        });
    }

    Ok(())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConcreteLoopError {
    Lifecycle(String),
    Proposal(String),
    Decision(String),
    Authority(String),
    Execution(String),
    ProposalIdMismatch,
    ProposalDigestMismatch,
    DecisionNotBound,
    DecisionArtifactMismatch,
    NonSelectionAdvanced { stage: ResponseLifecycleStage },
    AuthorityAfterNonSelection,
    AuthorityStageNotReached,
    AuthorityArtifactKindMismatch,
    DuplicateBoundAuthorityId(String),
    DuplicateConcreteAuthorityId(String),
    AuthorityArtifactSetMismatch,
    SelectedOptionMissing(String),
    AuthorityProposalDigestMismatch { evaluation_id: String },
    AuthorityOptionMismatch {
        evaluation_id: String,
        expected: String,
        actual: String,
    },
    AuthorityRequirementMismatch { evaluation_id: String },
    ExecutionAfterNonSelection,
    ExecutionStageNotReached,
    ExecutedOptionMismatch { selected: String, executed: String },
}

impl fmt::Display for ConcreteLoopError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Lifecycle(error) => write!(f, "invalid lifecycle: {error}"),
            Self::Proposal(error) => write!(f, "invalid response proposal: {error}"),
            Self::Decision(error) => write!(f, "invalid response decision: {error}"),
            Self::Authority(error) => write!(f, "invalid authority evaluation: {error}"),
            Self::Execution(error) => write!(f, "invalid execution binding: {error}"),
            Self::ProposalIdMismatch => write!(f, "decision/lifecycle proposal IDs do not match"),
            Self::ProposalDigestMismatch => {
                write!(f, "decision proposal digest does not match lifecycle proposal digest")
            }
            Self::DecisionNotBound => write!(f, "lifecycle has no bound decision artifact"),
            Self::DecisionArtifactMismatch => {
                write!(f, "concrete decision does not match lifecycle-bound decision")
            }
            Self::NonSelectionAdvanced { stage } => write!(
                f,
                "non-selection decision cannot advance lifecycle to {stage:?}"
            ),
            Self::AuthorityAfterNonSelection => {
                write!(f, "authority evaluation cannot follow a non-selection decision")
            }
            Self::AuthorityStageNotReached => {
                write!(f, "lifecycle has not reached authority evaluation")
            }
            Self::AuthorityArtifactKindMismatch => {
                write!(f, "lifecycle authority slot contains a non-authority artifact")
            }
            Self::DuplicateBoundAuthorityId(id) => {
                write!(f, "lifecycle repeats authority evaluation ID {id}")
            }
            Self::DuplicateConcreteAuthorityId(id) => {
                write!(f, "concrete authority evaluations repeat ID {id}")
            }
            Self::AuthorityArtifactSetMismatch => write!(
                f,
                "concrete authority evaluations do not match lifecycle-bound authority IDs"
            ),
            Self::SelectedOptionMissing(id) => {
                write!(f, "selected response option {id} is missing")
            }
            Self::AuthorityProposalDigestMismatch { evaluation_id } => write!(
                f,
                "authority evaluation {evaluation_id} binds a different proposal digest"
            ),
            Self::AuthorityOptionMismatch {
                evaluation_id,
                expected,
                actual,
            } => write!(
                f,
                "authority evaluation {evaluation_id} targets option {actual}; expected {expected}"
            ),
            Self::AuthorityRequirementMismatch { evaluation_id } => write!(
                f,
                "authority evaluation {evaluation_id} does not correspond to a selected-option requirement"
            ),
            Self::ExecutionAfterNonSelection => {
                write!(f, "execution cannot follow a non-selection decision")
            }
            Self::ExecutionStageNotReached => write!(f, "lifecycle has not reached execution"),
            Self::ExecutedOptionMismatch { selected, executed } => write!(
                f,
                "execution used option {executed}; decision selected {selected}"
            ),
        }
    }
}

impl std::error::Error for ConcreteLoopError {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AuthorityEvidenceKind, AuthorityEvidenceRef, DecisionEvidenceRef, DecisionMakerClaim,
        DecisionMechanism, DecisionPosition, DecisionProposalRef, EvidenceClass,
        ExternalEvidenceRef, GeoPoint, InterventionOption, ProjectedOutcomeRef,
        ProjectedOutcomeRole, ResponseArtifactRef, ResponseProposalMode, Reversibility,
        RiskAssessmentMode, RiskTriggerRef, SpatialExtent, TemporalExtent,
        AUTHORITY_EVALUATION_SCHEMA_VERSION, RESPONSE_DECISION_SCHEMA_VERSION,
        RESPONSE_LIFECYCLE_SCHEMA_VERSION,
    };

    fn immutable(id: &str) -> ExternalEvidenceRef {
        ExternalEvidenceRef {
            source_system: "mycelix".into(),
            resource_id: id.into(),
            content_digest: Some(format!("sha256:{id}")),
            retrieved_at: None,
            license: None,
        }
    }

    fn requirement() -> AuthorityRequirement {
        AuthorityRequirement {
            domain: "municipal_emergency".into(),
            action: "open_cooling_center".into(),
            jurisdiction: Some("Johannesburg".into()),
            policy_ref: None,
        }
    }

    fn proposal() -> ResponseProposal {
        ResponseProposal::new(
            "response:heat:1",
            ResponseProposalMode::OperationalRecommendation,
            RiskTriggerRef {
                assessment_id: "risk:heat:1".into(),
                assessment_digest: "sha256:risk".into(),
                risk_output_observation_id: "obs:risk:heat".into(),
                expected_mode: RiskAssessmentMode::Operational,
            },
            100,
            None,
            vec![InterventionOption {
                id: "cooling-centers".into(),
                intervention_type: "open_cooling_centers".into(),
                target: SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
                execution_window: TemporalExtent::new(110, 200).unwrap(),
                projected_outcomes: vec![ProjectedOutcomeRef {
                    observation_id: "obs:projected:1".into(),
                    expected_class: EvidenceClass::Scenario,
                    role: ProjectedOutcomeRole::Benefit,
                }],
                resources: Vec::new(),
                authority_requirements: vec![requirement()],
                reversibility: Reversibility::Reversible,
                assumptions: Vec::new(),
            }],
        )
        .unwrap()
    }

    fn decision(disposition: ResponseDisposition) -> ResponseDecisionRecord {
        ResponseDecisionRecord {
            schema_version: RESPONSE_DECISION_SCHEMA_VERSION,
            id: "decision:heat:1".into(),
            proposal: DecisionProposalRef {
                proposal_id: "response:heat:1".into(),
                proposal_digest: "sha256:proposal".into(),
            },
            decided_at: 105,
            mechanism: DecisionMechanism::EmergencyProcedure,
            decision_makers: vec![DecisionMakerClaim {
                did: "did:mycelix:official".into(),
                role: "incident_commander".into(),
                constituency: Some("Johannesburg".into()),
            }],
            governing_policies: Vec::new(),
            evidence_considered: vec![DecisionEvidenceRef {
                artifact_type: "physical_risk_assessment".into(),
                artifact_id: "risk:heat:1".into(),
                artifact_digest: "sha256:risk".into(),
            }],
            disposition,
            rationale_ref: None,
            alternate_positions: Vec::<DecisionPosition>::new(),
        }
    }

    fn artifact(kind: ResponseArtifactKind, id: &str, digest: &str) -> ResponseArtifactRef {
        ResponseArtifactRef::new(kind, id, digest).unwrap()
    }

    fn authority_ledger() -> ResponseLifecycleLedger {
        ResponseLifecycleLedger {
            schema_version: RESPONSE_LIFECYCLE_SCHEMA_VERSION,
            id: "loop:heat:1".into(),
            stage: ResponseLifecycleStage::AuthorityEvaluated,
            proposal: artifact(
                ResponseArtifactKind::Proposal,
                "response:heat:1",
                "sha256:proposal",
            ),
            decision: Some(artifact(
                ResponseArtifactKind::Decision,
                "decision:heat:1",
                "sha256:decision",
            )),
            authority_evaluations: vec![artifact(
                ResponseArtifactKind::AuthorityEvaluation,
                "authority-eval:1",
                "sha256:authority-eval-1",
            )],
            execution: None,
            outcome: None,
        }
    }

    fn evaluation(result: AuthorityEvaluationResult) -> AuthorityEvaluationRecord {
        AuthorityEvaluationRecord {
            schema_version: AUTHORITY_EVALUATION_SCHEMA_VERSION,
            id: "authority-eval:1".into(),
            response_authority: crate::ResponseAuthorityRef {
                proposal_id: "response:heat:1".into(),
                proposal_digest: "sha256:proposal".into(),
                option_id: "cooling-centers".into(),
                requirement: requirement(),
            },
            subject_did: "did:mycelix:executor".into(),
            evaluator_did: "did:mycelix:authority-verifier".into(),
            evaluator_qualifications: Vec::new(),
            evidence: vec![AuthorityEvidenceRef {
                kind: AuthorityEvidenceKind::Grant,
                reference: immutable("grant:cooling-center:1"),
            }],
            result,
            evaluated_at: 105,
            valid_from: Some(100),
            valid_until: Some(200),
            rationale_ref: None,
        }
    }

    #[test]
    fn selected_decision_binds_to_same_proposal_root() {
        let ledger = authority_ledger();
        let gate = validate_decision_binding(
            &ledger,
            &proposal(),
            &decision(ResponseDisposition::SelectOption {
                option_id: "cooling-centers".into(),
            }),
        )
        .unwrap();
        assert_eq!(gate.selected_option_id(), Some("cooling-centers"));
        assert!(!gate.grants_execution_authority());
    }

    #[test]
    fn rejected_decision_cannot_advance_to_authority_stage() {
        let error = validate_decision_binding(
            &authority_ledger(),
            &proposal(),
            &decision(ResponseDisposition::RejectAll),
        )
        .unwrap_err();
        assert!(matches!(
            error,
            ConcreteLoopError::NonSelectionAdvanced {
                stage: ResponseLifecycleStage::AuthorityEvaluated
            }
        ));
    }

    #[test]
    fn authority_coverage_preserves_allowed_claim_without_becoming_authority() {
        let report = evaluate_authority_coverage(
            &authority_ledger(),
            &proposal(),
            &decision(ResponseDisposition::SelectOption {
                option_id: "cooling-centers".into(),
            }),
            &[evaluation(AuthorityEvaluationResult::Allowed)],
            150,
        )
        .unwrap();
        assert!(report.all_requirements_evaluated());
        assert!(report.all_requirements_have_current_allowed_claim());
        assert!(!report.has_disagreement());
        assert!(!report.grants_execution_authority());
    }

    #[test]
    fn denied_authority_is_visible_and_never_collapsed_into_allowed() {
        let report = evaluate_authority_coverage(
            &authority_ledger(),
            &proposal(),
            &decision(ResponseDisposition::SelectOption {
                option_id: "cooling-centers".into(),
            }),
            &[evaluation(AuthorityEvaluationResult::Denied)],
            150,
        )
        .unwrap();
        assert!(report.all_requirements_evaluated());
        assert!(!report.all_requirements_have_current_allowed_claim());
        assert!(report.has_disagreement());
    }

    #[test]
    fn same_bound_authority_id_with_different_digest_is_rejected() {
        let mut ledger = authority_ledger();
        ledger.authority_evaluations.push(artifact(
            ResponseArtifactKind::AuthorityEvaluation,
            "authority-eval:1",
            "sha256:different-content",
        ));
        assert!(ledger.validate().is_ok());

        let error = evaluate_authority_coverage(
            &ledger,
            &proposal(),
            &decision(ResponseDisposition::SelectOption {
                option_id: "cooling-centers".into(),
            }),
            &[evaluation(AuthorityEvaluationResult::Allowed)],
            150,
        )
        .unwrap_err();
        assert!(matches!(
            error,
            ConcreteLoopError::DuplicateBoundAuthorityId(_)
        ));
    }
}
