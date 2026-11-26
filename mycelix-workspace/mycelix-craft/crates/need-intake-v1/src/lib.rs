//! Evidence-traceable community-need decomposition.
//!
//! This crate validates a draft proposal and evaluates references to human
//! review decisions. It does not generate claims, authenticate source owners,
//! verify signatures, decide community priorities, or authorize work.
//! The host must authenticate every source, reviewer and local authority ref.

#![forbid(unsafe_code)]

use std::collections::{HashMap, HashSet};

/// Current schema version for need-intake proposals.
pub const NEED_INTAKE_SCHEMA_VERSION: u16 = 1;

/// A digest-pinned local authority reference.
#[derive(Clone, Debug, Eq, PartialEq, Hash)]
pub struct LocalAuthorityRefV1 {
    /// Authority policy identifier in the institution's local governance system.
    pub policy_id: String,
    /// Adopted policy revision.
    pub revision: u64,
    /// Digest of the exact adopted policy.
    pub policy_digest: [u8; 32],
    /// Local decision or delegation record identifier.
    pub decision_id: String,
    /// Digest of the local decision/delegation record.
    pub decision_digest: [u8; 32],
}

/// Original community need; source content remains in its owner-controlled store.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CommunityNeedV1 {
    /// Stable need identifier.
    pub need_id: String,
    /// Community or sponsor responsible for the expressed need.
    pub community_id: String,
    /// Digest of the exact source statement or submitted artifact.
    pub statement_digest: [u8; 32],
    /// Authenticated submitter principal identifier.
    pub submitter_id: String,
    /// Data-controller-owned visibility/disclosure profile.
    pub visibility_profile_id: String,
}

/// Kind of source artifact. A source reference establishes provenance, not truth.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum SourceKindV1 {
    /// A participant or community statement.
    ParticipantStatement,
    /// A measured or observed value.
    Measurement,
    /// A standard, regulation or technical reference.
    StandardOrPolicy,
    /// An existing project, asset or operational record.
    ExistingRecord,
    /// A domain-expert statement or assessment.
    DomainAssessment,
}

/// Immutable source reference used to support a proposed claim.
#[derive(Clone, Debug, Eq, PartialEq, Hash)]
pub struct SourceRefV1 {
    /// Stable source identifier.
    pub source_id: String,
    /// Digest of the exact source artifact or cited passage.
    pub source_digest: [u8; 32],
    /// Classification of the source artifact.
    pub kind: SourceKindV1,
    /// Data-controller-owned profile controlling retrieval/disclosure.
    pub visibility_profile_id: String,
}

/// Explicit assumptions are not presented as established facts.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AssumptionV1 {
    /// Stable assumption identifier.
    pub assumption_id: String,
    /// Plain-language assumption statement.
    pub statement: String,
    /// Person or institution accountable for reviewing the assumption.
    pub owner_id: String,
    /// Consequence if the assumption is false.
    pub risk_if_false: String,
}

/// How a proposed claim is grounded or qualified.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum TraceabilityBasisV1 {
    /// Directly points to a digest-pinned source artifact.
    Source {
        /// Source identifier.
        source_id: String,
        /// Exact source digest.
        source_digest: [u8; 32],
    },
    /// Explicitly marked assumption; not a source-backed fact.
    Assumption {
        /// Assumption identifier.
        assumption_id: String,
    },
    /// Model or human inference with method and input sources made explicit.
    Inference {
        /// Stable inference identifier.
        inference_id: String,
        /// Model, method or procedure identifier and version.
        method_id: String,
        /// Source identifiers used by the inference.
        source_ids: Vec<String>,
    },
}

/// A claim-to-source/assumption/inference link.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TraceabilityLinkV1 {
    /// Explanation of the relationship between this basis and the claim.
    pub relation: String,
    /// Grounding, assumption or inference basis.
    pub basis: TraceabilityBasisV1,
}

/// Version-pinned capability vocabulary/profile reference.
#[derive(Clone, Debug, Eq, PartialEq, Hash)]
pub struct CapabilityProfileRefV1 {
    /// Vocabulary authority identifier.
    pub vocabulary_id: String,
    /// Concept identifier in the vocabulary.
    pub concept_id: String,
    /// Exact vocabulary/profile revision.
    pub revision: String,
    /// Digest of the exact concept/profile definition.
    pub digest: [u8; 32],
}

/// Version-pinned external policy reference.
#[derive(Clone, Debug, Eq, PartialEq, Hash)]
pub struct PolicyRefV1 {
    /// Policy identifier.
    pub policy_id: String,
    /// Monotonically versioned policy revision.
    pub revision: u64,
    /// Digest of the exact policy document.
    pub digest: [u8; 32],
}

/// A proposed outcome with an observable acceptance condition.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProposedOutcomeV1 {
    /// Stable outcome identifier.
    pub outcome_id: String,
    /// Plain-language outcome; still a proposal until ratified.
    pub statement: String,
    /// Metric or indicator used to assess the outcome.
    pub indicator: String,
    /// Exact owner-defined measurement/observation contract reference.
    pub measurement_ref: String,
    /// External policy that defines outcome acceptance.
    pub acceptance_policy: PolicyRefV1,
    /// Baseline value or state; unknown values remain None.
    pub baseline: Option<String>,
    /// Target value or state.
    pub target: String,
    /// Time horizon or review interval.
    pub time_horizon: String,
    /// Owner responsible for reporting the outcome.
    pub outcome_owner_id: String,
    /// Local authority reference that will accept the measured outcome.
    pub acceptance_authority: LocalAuthorityRefV1,
    /// Every proposed outcome must have at least one explicit basis.
    pub traceability: Vec<TraceabilityLinkV1>,
}

/// Capability requirement kind; preferred/developmental are not silently mandatory.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum RequirementKindV1 {
    /// Non-compensable safety requirement.
    SafetyCritical,
    /// Mandatory for the defined work package.
    Required,
    /// May be acquired through a separately supervised pathway.
    Developmental,
    /// Useful but not a participation gate.
    Preferred,
}

/// A proposed capability requirement linked to the need and supporting sources.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProposedRequirementV1 {
    /// Stable requirement identifier.
    pub requirement_id: String,
    /// Exact versioned capability vocabulary/profile.
    pub capability: CapabilityProfileRefV1,
    /// Digest of the work context in which the capability is required.
    pub context_digest: [u8; 32],
    /// Jurisdiction relevant to the assessment and work.
    pub jurisdiction_id: String,
    /// External policy used to qualify evidence for this requirement.
    pub assessment_policy: PolicyRefV1,
    /// Number of distinct contributors required for coverage.
    pub minimum_contributors: u16,
    /// Maximum age of an issuer/status observation in seconds.
    pub max_status_age_seconds: u64,
    /// Plain-language requirement statement.
    pub statement: String,
    /// Requirement classification.
    pub kind: RequirementKindV1,
    /// Traceability to sources, explicit assumptions or labelled inference.
    pub traceability: Vec<TraceabilityLinkV1>,
}

/// A work package in a proposed outcome/dependency structure.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProposedWorkPackageV1 {
    /// Stable work-package identifier.
    pub work_package_id: String,
    /// Plain-language description.
    pub description: String,
    /// Digest of the exact work-package scope.
    pub scope_digest: [u8; 32],
    /// Outcomes this package contributes to.
    pub outcome_ids: Vec<String>,
    /// Requirements that apply to this package.
    pub requirement_ids: Vec<String>,
    /// Packages that must complete before this package.
    pub depends_on: Vec<String>,
}

/// One stakeholder or group position preserved within a contradiction.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct StakeholderPositionV1 {
    /// Community group or institution expressing this position.
    pub group_id: String,
    /// The position in the group's own terms or an attributed faithful summary.
    pub statement: String,
    /// Source, assumption or inference references supporting this position.
    pub traceability: Vec<TraceabilityLinkV1>,
}

/// A trade-off claim attached to a candidate decomposition.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProposedTradeoffV1 {
    /// Stable trade-off identifier within the proposal.
    pub tradeoff_id: String,
    /// Plain-language trade-off statement.
    pub statement: String,
    /// Traceability to sources, assumptions or labelled inference.
    pub traceability: Vec<TraceabilityLinkV1>,
}

/// A candidate decomposition presented for comparison, not an AI-selected plan.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProposedAlternativeV1 {
    /// Stable alternative identifier.
    pub alternative_id: String,
    /// Human-readable option label.
    pub label: String,
    /// Description of the proposed approach.
    pub description: String,
    /// Outcomes addressed by this option.
    pub outcome_ids: Vec<String>,
    /// Capability requirements relevant to this option.
    pub requirement_ids: Vec<String>,
    /// Work packages that implement this option.
    pub work_package_ids: Vec<String>,
    /// Explicit costs, benefits, dependencies and trade-offs.
    pub tradeoffs: Vec<ProposedTradeoffV1>,
    /// Traceability for why this option is being proposed.
    pub traceability: Vec<TraceabilityLinkV1>,
}

/// Contradiction class and resolution status.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContradictionKindV1 {
    /// Different people or groups express incompatible priorities.
    StakeholderPriorityConflict,
    /// Source artifacts or measurements disagree.
    SourceConflict,
    /// Domain experts disagree on technical feasibility or interpretation.
    TechnicalConflict,
}

/// A contradiction remains visible until an authorized resolution is recorded.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ContradictionV1 {
    /// Stable contradiction identifier.
    pub contradiction_id: String,
    /// Contradiction classification.
    pub kind: ContradictionKindV1,
    /// Claim identifiers affected (outcome:<id> or requirement:<id>).
    pub affected_claim_ids: Vec<String>,
    /// Distinct stakeholder positions; required for stakeholder-priority conflict.
    pub stakeholder_positions: Vec<StakeholderPositionV1>,
    /// True only when a separately authenticated decision has resolved it.
    pub resolved: bool,
    /// Decision record identifier required when resolved.
    pub resolution_decision_id: Option<String>,
    /// Exact digest of the resolution decision.
    pub resolution_decision_digest: Option<[u8; 32]>,
    /// Local authority reference that adopted the resolution.
    pub resolution_authority: Option<LocalAuthorityRefV1>,
}

/// Explicit unknown or unanswered question.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OpenQuestionV1 {
    /// Stable question identifier.
    pub question_id: String,
    /// The unresolved question.
    pub question: String,
    /// A blocking question must be resolved before ratification.
    pub blocks_ratification: bool,
}

/// Review roles have distinct responsibilities and cannot substitute for one another.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum ReviewRoleV1 {
    /// Confirms the need, priorities and trade-offs reflect community intent.
    Community,
    /// Reviews domain correctness and feasibility.
    DomainExpert,
    /// Reviews hazardous-work and safety constraints.
    Safety,
    /// Checks source/claim links and assumption labelling.
    Provenance,
    /// Ratifies the exact proposal under local governance authority.
    LocalRatifier,
}

/// A review may request changes, reject the proposal or approve the exact digest.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReviewDecisionV1 {
    /// Reviewer accepts the exact proposal in their role.
    Approve,
    /// Reviewer requires clarification or amendment.
    RequestChanges,
    /// Reviewer rejects this proposal revision.
    Reject,
}

/// Authenticated-by-host review reference; this crate does not verify signatures.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ReviewRefV1 {
    /// Reviewer principal.
    pub reviewer_id: String,
    /// Reviewer's role for this exact decision.
    pub role: ReviewRoleV1,
    /// Local authority reference authorizing the review.
    pub authority: LocalAuthorityRefV1,
    /// Proposal digest reviewed.
    pub proposal_digest: [u8; 32],
    /// Review decision.
    pub decision: ReviewDecisionV1,
    /// Review timestamp.
    pub reviewed_at: u64,
    /// Optional time after which this review is no longer current.
    pub valid_until: Option<u64>,
    /// Optional effective withdrawal time.
    pub withdrawn_at: Option<u64>,
    /// Alternative selected by the local ratifier; never set by a model/reviewer.
    pub selected_alternative_id: Option<String>,
}

/// Immutable proposed decomposition. A changed digest requires new reviews.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NeedProposalV1 {
    /// Must equal NEED_INTAKE_SCHEMA_VERSION.
    pub schema_version: u16,
    /// Project jurisdiction used by downstream Project Formation.
    pub jurisdiction_id: String,
    /// Original community need.
    pub need: CommunityNeedV1,
    /// Monotonic proposal revision.
    pub revision: u64,
    /// Previous revision digest, if this is an amendment.
    pub previous_proposal_digest: Option<[u8; 32]>,
    /// Digest of exact canonical proposal bytes, computed by the host.
    pub proposal_digest: [u8; 32],
    /// Source artifacts referenced by claims.
    pub sources: Vec<SourceRefV1>,
    /// Explicit assumptions; no assumption is silently elevated to fact.
    pub assumptions: Vec<AssumptionV1>,
    /// Proposed outcomes.
    pub outcomes: Vec<ProposedOutcomeV1>,
    /// Proposed capability requirements.
    pub requirements: Vec<ProposedRequirementV1>,
    /// Proposed work packages and dependency graph.
    pub work_packages: Vec<ProposedWorkPackageV1>,
    /// Alternative decompositions and explicit trade-offs; never auto-selected.
    pub alternatives: Vec<ProposedAlternativeV1>,
    /// Unresolved or resolved contradictions.
    pub contradictions: Vec<ContradictionV1>,
    /// Open questions and their blocking status.
    pub open_questions: Vec<OpenQuestionV1>,
    /// Whether a safety review is required for this proposal.
    pub safety_review_required: bool,
    /// Host-authenticated review references.
    pub reviews: Vec<ReviewRefV1>,
}

/// Structural validation errors.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NeedIntakeErrorV1 {
    /// Unsupported schema.
    UnsupportedSchema,
    /// Required identifier, statement or label is empty/padded.
    EmptyField,
    /// Required digest is absent or all-zero.
    MissingDigest,
    /// Revision is zero or lacks a previous digest when amended.
    InvalidRevision,
    /// Repeated identifier.
    DuplicateIdentifier,
    /// Traceability collection is empty.
    MissingTraceability,
    /// Source link points to no source or wrong digest.
    UnknownOrMismatchedSource,
    /// Assumption link points to no assumption.
    UnknownAssumption,
    /// Inference has no method, identifier or source.
    InvalidInference,
    /// Outcome or requirement link references no known claim.
    UnknownClaim,
    /// Work package link references unknown outcome, requirement or package.
    UnknownWorkReference,
    /// Capability requirement jurisdiction differs from the project jurisdiction.
    JurisdictionMismatch,
    /// A capability requirement has a zero minimum contributor threshold.
    InvalidRequirementThreshold,
    /// A capability requirement disables status-freshness checking.
    ZeroStatusFreshnessWindow,
    /// An outcome or requirement is not assigned to any work package.
    UnmappedClaim,
    /// Dependency cycle exists.
    DependencyCycle,
    /// Contradiction has no affected claim.
    EmptyContradiction,
    /// Stakeholder conflict lacks two or more separately attributed positions.
    InsufficientStakeholderPositions,
    /// Resolved contradiction is missing its decision reference.
    MissingContradictionResolution,
    /// Unresolved contradiction has a resolution reference.
    UnexpectedContradictionResolution,
    /// Authority reference is incomplete.
    InvalidAuthority,
    /// Review has inconsistent times or no reviewer authority.
    InvalidReview,
    /// A safety-critical requirement lacks a mandatory safety review path.
    SafetyReviewRequired,
    /// Only the local ratifier may select an existing alternative on approval.
    InvalidSelectedAlternative,
    /// The proposal has not reached a valid local ratification state.
    NotRatified,
    /// Proposal lacks a minimum project structure.
    EmptyProposalStructure,
    /// A declared baseline is blank or padded.
    InvalidBaseline,
}

/// Disposition of the structured proposal workflow.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RatificationStateV1 {
    /// Proposal has not yet been reviewed.
    Draft,
    /// Community input or a blocking question must be clarified.
    NeedsCommunityClarification,
    /// Domain correctness or technical contradiction requires review.
    NeedsDomainExpertReview,
    /// Safety review remains outstanding.
    NeedsSafetyReview,
    /// All prerequisite reviews passed; local ratification is next.
    ReadyForLocalRatification,
    /// Exact proposal digest has local ratification.
    Ratified,
    /// An authorized reviewer rejected this proposal revision.
    Rejected,
}

/// Role-specific review gaps and exact digest being evaluated.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RatificationEvaluationV1 {
    /// Workflow state.
    pub state: RatificationStateV1,
    /// Exact proposal digest considered.
    pub proposal_digest: [u8; 32],
    /// Prerequisite roles still requiring approval.
    pub missing_roles: Vec<ReviewRoleV1>,
    /// Unresolved blocking question identifiers.
    pub blocking_questions: Vec<String>,
    /// Unresolved contradiction identifiers.
    pub unresolved_contradictions: Vec<String>,
}

/// Minimal authority-bearing handoff from Need Intake into downstream project formation.
///
/// The host must still authenticate the local authority reference and perform
/// the explicit conversion into the Project Formation V1 domain schema.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RatifiedNeedEnvelopeV1 {
    /// Original community need identifier.
    pub need_id: String,
    /// Community/sponsor identity responsible for the need.
    pub community_id: String,
    /// Project jurisdiction explicitly approved in the proposal.
    pub jurisdiction_id: String,
    /// Exact proposal revision digest that was ratified.
    pub proposal_digest: [u8; 32],
    /// Proposal revision.
    pub proposal_revision: u64,
    /// Alternative explicitly selected by the local ratifier, when options exist.
    pub selected_alternative_id: Option<String>,
    /// Exact local ratification decision identifier.
    pub ratification_decision_id: String,
    /// Exact local ratification decision digest.
    pub ratification_decision_digest: [u8; 32],
    /// Local authority that adopted the proposal.
    pub ratification_authority: LocalAuthorityRefV1,
    /// Time at which the local ratifier approved this proposal digest.
    pub ratified_at: u64,
}

/// Construct the downstream handoff only after every required review and local
/// ratification is current for the exact proposal digest.
pub fn ratified_need_envelope(
    proposal: &NeedProposalV1,
    now: u64,
) -> Result<RatifiedNeedEnvelopeV1, NeedIntakeErrorV1> {
    let evaluation = evaluate_ratification(proposal, now)?;
    if evaluation.state != RatificationStateV1::Ratified {
        return Err(NeedIntakeErrorV1::NotRatified);
    }
    let ratification = proposal
        .reviews
        .iter()
        .find(|review| {
            review.role == ReviewRoleV1::LocalRatifier
                && review.decision == ReviewDecisionV1::Approve
                && review.proposal_digest == proposal.proposal_digest
                && review.reviewed_at <= now
                && review.valid_until.map(|expiry| now < expiry).unwrap_or(true)
                && review.withdrawn_at.map(|time| now < time).unwrap_or(true)
        })
        .ok_or(NeedIntakeErrorV1::NotRatified)?;

    Ok(RatifiedNeedEnvelopeV1 {
        need_id: proposal.need.need_id.clone(),
        community_id: proposal.need.community_id.clone(),
        jurisdiction_id: proposal.jurisdiction_id.clone(),
        proposal_digest: proposal.proposal_digest,
        proposal_revision: proposal.revision,
        selected_alternative_id: ratification.selected_alternative_id.clone(),
        ratification_decision_id: ratification.authority.decision_id.clone(),
        ratification_decision_digest: ratification.authority.decision_digest,
        ratification_authority: ratification.authority.clone(),
        ratified_at: ratification.reviewed_at,
    })
}

fn validate_authority(authority: &LocalAuthorityRefV1) -> Result<(), NeedIntakeErrorV1> {
    require_non_empty(&authority.policy_id)?;
    require_non_empty(&authority.decision_id)?;
    if authority.revision == 0 {
        return Err(NeedIntakeErrorV1::InvalidRevision);
    }
    require_digest(&authority.policy_digest)?;
    require_digest(&authority.decision_digest)
}

/// Validate the complete proposed decomposition before review evaluation.
pub fn validate_proposal(proposal: &NeedProposalV1) -> Result<(), NeedIntakeErrorV1> {
    if proposal.schema_version != NEED_INTAKE_SCHEMA_VERSION {
        return Err(NeedIntakeErrorV1::UnsupportedSchema);
    }
    if proposal.revision == 0
        || (proposal.revision == 1 && proposal.previous_proposal_digest.is_some())
        || (proposal.revision > 1
            && proposal
                .previous_proposal_digest
                .map(|digest| digest.iter().all(|byte| *byte == 0))
                .unwrap_or(true))
    {
        return Err(NeedIntakeErrorV1::InvalidRevision);
    }
    require_non_empty(&proposal.jurisdiction_id)?;
    require_non_empty(&proposal.need.need_id)?;
    require_non_empty(&proposal.need.community_id)?;
    require_digest(&proposal.need.statement_digest)?;
    require_non_empty(&proposal.need.submitter_id)?;
    require_non_empty(&proposal.need.visibility_profile_id)?;
    require_digest(&proposal.proposal_digest)?;
    if proposal.outcomes.is_empty() || proposal.work_packages.is_empty() {
        return Err(NeedIntakeErrorV1::EmptyProposalStructure);
    }

    let mut source_map = HashMap::new();
    for source in &proposal.sources {
        require_non_empty(&source.source_id)?;
        require_digest(&source.source_digest)?;
        require_non_empty(&source.visibility_profile_id)?;
        if source_map.insert(source.source_id.clone(), source.source_digest).is_some() {
            return Err(NeedIntakeErrorV1::DuplicateIdentifier);
        }
    }

    let mut assumption_ids = HashSet::new();
    for assumption in &proposal.assumptions {
        require_non_empty(&assumption.assumption_id)?;
        require_non_empty(&assumption.statement)?;
        require_non_empty(&assumption.owner_id)?;
        require_non_empty(&assumption.risk_if_false)?;
        if !assumption_ids.insert(assumption.assumption_id.clone()) {
            return Err(NeedIntakeErrorV1::DuplicateIdentifier);
        }
    }

    let mut claim_ids = HashSet::new();
    for outcome in &proposal.outcomes {
        require_non_empty(&outcome.outcome_id)?;
        require_non_empty(&outcome.statement)?;
        require_non_empty(&outcome.indicator)?;
        require_non_empty(&outcome.measurement_ref)?;
        validate_policy_ref(&outcome.acceptance_policy)?;
        if outcome.baseline.as_ref().map(|value| value.trim().is_empty() || value.trim() != value).unwrap_or(false) {
            return Err(NeedIntakeErrorV1::InvalidBaseline);
        }
        require_non_empty(&outcome.target)?;
        require_non_empty(&outcome.time_horizon)?;
        require_non_empty(&outcome.outcome_owner_id)?;
        validate_authority(&outcome.acceptance_authority)?;
        if !claim_ids.insert(format!("outcome:{}", outcome.outcome_id)) {
            return Err(NeedIntakeErrorV1::DuplicateIdentifier);
        }
        validate_traceability(&outcome.traceability, &source_map, &assumption_ids)?;
    }

    for requirement in &proposal.requirements {
        require_non_empty(&requirement.requirement_id)?;
        validate_capability_profile(&requirement.capability)?;
        require_digest(&requirement.context_digest)?;
        require_non_empty(&requirement.jurisdiction_id)?;
        if requirement.jurisdiction_id != proposal.jurisdiction_id {
            return Err(NeedIntakeErrorV1::JurisdictionMismatch);
        }
        validate_policy_ref(&requirement.assessment_policy)?;
        if requirement.minimum_contributors == 0 {
            return Err(NeedIntakeErrorV1::InvalidRequirementThreshold);
        }
        if requirement.max_status_age_seconds == 0 {
            return Err(NeedIntakeErrorV1::ZeroStatusFreshnessWindow);
        }
        require_non_empty(&requirement.statement)?;
        if !claim_ids.insert(format!("requirement:{}", requirement.requirement_id)) {
            return Err(NeedIntakeErrorV1::DuplicateIdentifier);
        }
        validate_traceability(&requirement.traceability, &source_map, &assumption_ids)?;
    }

    let outcome_ids = proposal.outcomes.iter().map(|x| x.outcome_id.as_str()).collect::<HashSet<_>>();
    let requirement_ids = proposal.requirements.iter().map(|x| x.requirement_id.as_str()).collect::<HashSet<_>>();
    let mut package_ids = HashSet::new();
    let mut dependencies = HashMap::new();

    for package in &proposal.work_packages {
        require_non_empty(&package.work_package_id)?;
        require_non_empty(&package.description)?;
        require_digest(&package.scope_digest)?;
        if !package_ids.insert(package.work_package_id.as_str()) {
            return Err(NeedIntakeErrorV1::DuplicateIdentifier);
        }
        for outcome_id in &package.outcome_ids {
            if !outcome_ids.contains(outcome_id.as_str()) {
                return Err(NeedIntakeErrorV1::UnknownWorkReference);
            }
        }
        for requirement_id in &package.requirement_ids {
            if !requirement_ids.contains(requirement_id.as_str()) {
                return Err(NeedIntakeErrorV1::UnknownWorkReference);
            }
        }
        dependencies.insert(package.work_package_id.clone(), package.depends_on.clone());
    }
    let mut mapped_outcomes = HashSet::new();
    let mut mapped_requirements = HashSet::new();
    for package in &proposal.work_packages {
        mapped_outcomes.extend(package.outcome_ids.iter().map(String::as_str));
        mapped_requirements.extend(package.requirement_ids.iter().map(String::as_str));
    }
    if mapped_outcomes.len() != outcome_ids.len()
        || mapped_requirements.len() != requirement_ids.len()
    {
        return Err(NeedIntakeErrorV1::UnmappedClaim);
    }

    if proposal.requirements.iter().any(|requirement| {
        requirement.kind == RequirementKindV1::SafetyCritical
    }) && !proposal.safety_review_required {
        return Err(NeedIntakeErrorV1::SafetyReviewRequired);
    }

    for package in &proposal.work_packages {
        for dependency in &package.depends_on {
            if !package_ids.contains(dependency.as_str()) || dependency == &package.work_package_id {
                return Err(NeedIntakeErrorV1::UnknownWorkReference);
            }
        }
    }
    validate_dependency_graph(&dependencies)?;

    let mut alternative_ids = HashSet::new();
    let mut tradeoff_ids = HashSet::new();
    for alternative in &proposal.alternatives {
        require_non_empty(&alternative.alternative_id)?;
        require_non_empty(&alternative.label)?;
        require_non_empty(&alternative.description)?;
        if !alternative_ids.insert(alternative.alternative_id.as_str()) {
            return Err(NeedIntakeErrorV1::DuplicateIdentifier);
        }
        if alternative.tradeoffs.is_empty() {
            return Err(NeedIntakeErrorV1::MissingTraceability);
        }
        for tradeoff in &alternative.tradeoffs {
            require_non_empty(&tradeoff.tradeoff_id)?;
            if !tradeoff_ids.insert(tradeoff.tradeoff_id.as_str()) {
                return Err(NeedIntakeErrorV1::DuplicateIdentifier);
            }
            require_non_empty(&tradeoff.statement)?;
            validate_traceability(&tradeoff.traceability, &source_map, &assumption_ids)?;
        }
        for outcome_id in &alternative.outcome_ids {
            if !outcome_ids.contains(outcome_id.as_str()) {
                return Err(NeedIntakeErrorV1::UnknownWorkReference);
            }
        }
        for requirement_id in &alternative.requirement_ids {
            if !requirement_ids.contains(requirement_id.as_str()) {
                return Err(NeedIntakeErrorV1::UnknownWorkReference);
            }
        }
        for package_id in &alternative.work_package_ids {
            if !package_ids.contains(package_id.as_str()) {
                return Err(NeedIntakeErrorV1::UnknownWorkReference);
            }
        }
        validate_traceability(&alternative.traceability, &source_map, &assumption_ids)?;
    }

    let mut contradiction_ids = HashSet::new();
    for contradiction in &proposal.contradictions {
        require_non_empty(&contradiction.contradiction_id)?;
        if !contradiction_ids.insert(contradiction.contradiction_id.as_str()) {
            return Err(NeedIntakeErrorV1::DuplicateIdentifier);
        }
        if contradiction.affected_claim_ids.is_empty() {
            return Err(NeedIntakeErrorV1::EmptyContradiction);
        }
        for claim_id in &contradiction.affected_claim_ids {
            if !claim_ids.contains(claim_id.as_str()) {
                return Err(NeedIntakeErrorV1::UnknownClaim);
            }
        }
        if contradiction.kind == ContradictionKindV1::StakeholderPriorityConflict {
            if contradiction.stakeholder_positions.len() < 2 {
                return Err(NeedIntakeErrorV1::InsufficientStakeholderPositions);
            }
            let mut position_groups = HashSet::new();
            for position in &contradiction.stakeholder_positions {
                require_non_empty(&position.group_id)?;
                if !position_groups.insert(position.group_id.as_str()) {
                    return Err(NeedIntakeErrorV1::DuplicateIdentifier);
                }
                require_non_empty(&position.statement)?;
                validate_traceability(&position.traceability, &source_map, &assumption_ids)?;
            }
        }
        match (
            contradiction.resolved,
            &contradiction.resolution_decision_id,
            &contradiction.resolution_decision_digest,
            &contradiction.resolution_authority,
        ) {
            (true, Some(id), Some(digest), Some(authority)) => {
                require_non_empty(id)?;
                require_digest(digest)?;
                validate_authority(authority)?;
            }
            (true, _, _, _) => return Err(NeedIntakeErrorV1::MissingContradictionResolution),
            (false, None, None, None) => {}
            (false, _, _, _) => return Err(NeedIntakeErrorV1::UnexpectedContradictionResolution),
        }
    }

    let mut question_ids = HashSet::new();
    for question in &proposal.open_questions {
        require_non_empty(&question.question_id)?;
        require_non_empty(&question.question)?;
        if !question_ids.insert(question.question_id.as_str()) {
            return Err(NeedIntakeErrorV1::DuplicateIdentifier);
        }
    }

    let mut reviewed_roles = HashSet::new();
    for review in &proposal.reviews {
        require_non_empty(&review.reviewer_id)?;
        validate_authority(&review.authority)?;
        require_digest(&review.proposal_digest)?;
        if !reviewed_roles.insert((review.role, review.proposal_digest)) {
            return Err(NeedIntakeErrorV1::DuplicateIdentifier);
        }
        match (review.role, review.decision, &review.selected_alternative_id) {
            (ReviewRoleV1::LocalRatifier, ReviewDecisionV1::Approve, Some(selected)) => {
                if !proposal.alternatives.iter().any(|item| &item.alternative_id == selected) {
                    return Err(NeedIntakeErrorV1::InvalidSelectedAlternative);
                }
            }
            (ReviewRoleV1::LocalRatifier, ReviewDecisionV1::Approve, None)
                if !proposal.alternatives.is_empty() =>
            {
                return Err(NeedIntakeErrorV1::InvalidSelectedAlternative);
            }
            (ReviewRoleV1::LocalRatifier, _, Some(_)) => {
                return Err(NeedIntakeErrorV1::InvalidSelectedAlternative);
            }
            (_, _, Some(_)) => return Err(NeedIntakeErrorV1::InvalidSelectedAlternative),
            _ => {}
        }
        if review.valid_until.map(|t| t < review.reviewed_at).unwrap_or(false)
            || review.withdrawn_at.map(|t| t < review.reviewed_at).unwrap_or(false)
        {
            return Err(NeedIntakeErrorV1::InvalidReview);
        }
    }
    Ok(())
}

fn validate_traceability(
    links: &[TraceabilityLinkV1],
    sources: &HashMap<String, [u8; 32]>,
    assumptions: &HashSet<String>,
) -> Result<(), NeedIntakeErrorV1> {
    if links.is_empty() {
        return Err(NeedIntakeErrorV1::MissingTraceability);
    }
    for link in links {
        require_non_empty(&link.relation)?;
        match &link.basis {
            TraceabilityBasisV1::Source { source_id, source_digest } => {
                if sources.get(source_id) != Some(source_digest) {
                    return Err(NeedIntakeErrorV1::UnknownOrMismatchedSource);
                }
            }
            TraceabilityBasisV1::Assumption { assumption_id } => {
                if !assumptions.contains(assumption_id) {
                    return Err(NeedIntakeErrorV1::UnknownAssumption);
                }
            }
            TraceabilityBasisV1::Inference { inference_id, method_id, source_ids } => {
                require_non_empty(inference_id)?;
                require_non_empty(method_id)?;
                if source_ids.is_empty() || source_ids.iter().any(|id| !sources.contains_key(id)) {
                    return Err(NeedIntakeErrorV1::InvalidInference);
                }
            }
        }
    }
    Ok(())
}

fn validate_dependency_graph(
    dependencies: &HashMap<String, Vec<String>>,
) -> Result<(), NeedIntakeErrorV1> {
    fn visit(
        id: &str,
        graph: &HashMap<String, Vec<String>>,
        active: &mut HashSet<String>,
        done: &mut HashSet<String>,
    ) -> Result<(), NeedIntakeErrorV1> {
        if done.contains(id) {
            return Ok(());
        }
        if !active.insert(id.to_owned()) {
            return Err(NeedIntakeErrorV1::DependencyCycle);
        }
        let edges = graph.get(id).ok_or(NeedIntakeErrorV1::UnknownWorkReference)?;
        for next in edges {
            visit(next, graph, active, done)?;
        }
        active.remove(id);
        done.insert(id.to_owned());
        Ok(())
    }
    let mut active = HashSet::new();
    let mut done = HashSet::new();
    for id in dependencies.keys() {
        visit(id, dependencies, &mut active, &mut done)?;
    }
    Ok(())
}

/// Evaluate whether a proposal may advance to the next human decision.
///
/// Reviews count only if they bind to this exact digest and are current at now.
/// The host must authenticate reviewer authority and source/status snapshots.
pub fn evaluate_ratification(
    proposal: &NeedProposalV1,
    now: u64,
) -> Result<RatificationEvaluationV1, NeedIntakeErrorV1> {
    validate_proposal(proposal)?;

    let blocking_questions = proposal.open_questions.iter()
        .filter(|question| question.blocks_ratification)
        .map(|question| question.question_id.clone())
        .collect::<Vec<_>>();
    let unresolved_contradictions = proposal.contradictions.iter()
        .filter(|contradiction| !contradiction.resolved)
        .map(|contradiction| contradiction.contradiction_id.clone())
        .collect::<Vec<_>>();

    let mut required_roles = vec![
        ReviewRoleV1::Community,
        ReviewRoleV1::DomainExpert,
        ReviewRoleV1::Provenance,
    ];
    if proposal.safety_review_required {
        required_roles.push(ReviewRoleV1::Safety);
    }

    let current_reviews = proposal.reviews.iter().filter(|review| {
        review.proposal_digest == proposal.proposal_digest
            && review.reviewed_at <= now
            && review.valid_until.map(|expiry| now < expiry).unwrap_or(true)
            && review.withdrawn_at.map(|time| now < time).unwrap_or(true)
    }).collect::<Vec<_>>();

    if current_reviews.is_empty() {
        return Ok(RatificationEvaluationV1 {
            state: RatificationStateV1::Draft,
            proposal_digest: proposal.proposal_digest,
            missing_roles: required_roles,
            blocking_questions,
            unresolved_contradictions,
        });
    }

    if current_reviews.iter().any(|review| review.decision == ReviewDecisionV1::Reject) {
        return Ok(RatificationEvaluationV1 {
            state: RatificationStateV1::Rejected,
            proposal_digest: proposal.proposal_digest,
            missing_roles: required_roles,
            blocking_questions,
            unresolved_contradictions,
        });
    }

    if !blocking_questions.is_empty()
        || proposal.contradictions.iter().any(|item| {
            !item.resolved && item.kind == ContradictionKindV1::StakeholderPriorityConflict
        })
    {
        return Ok(RatificationEvaluationV1 {
            state: RatificationStateV1::NeedsCommunityClarification,
            proposal_digest: proposal.proposal_digest,
            missing_roles: required_roles,
            blocking_questions,
            unresolved_contradictions,
        });
    }
    if !unresolved_contradictions.is_empty() {
        return Ok(RatificationEvaluationV1 {
            state: RatificationStateV1::NeedsDomainExpertReview,
            proposal_digest: proposal.proposal_digest,
            missing_roles: required_roles,
            blocking_questions,
            unresolved_contradictions,
        });
    }

    let missing_roles = required_roles.iter().copied().filter(|role| {
        !current_reviews.iter().any(|review| {
            review.role == *role && review.decision == ReviewDecisionV1::Approve
        })
    }).collect::<Vec<_>>();

    if missing_roles.contains(&ReviewRoleV1::Community) {
        return Ok(RatificationEvaluationV1 {
            state: RatificationStateV1::NeedsCommunityClarification,
            proposal_digest: proposal.proposal_digest,
            missing_roles,
            blocking_questions,
            unresolved_contradictions,
        });
    }
    if missing_roles.contains(&ReviewRoleV1::Safety) {
        return Ok(RatificationEvaluationV1 {
            state: RatificationStateV1::NeedsSafetyReview,
            proposal_digest: proposal.proposal_digest,
            missing_roles,
            blocking_questions,
            unresolved_contradictions,
        });
    }
    if missing_roles.contains(&ReviewRoleV1::DomainExpert)
        || missing_roles.contains(&ReviewRoleV1::Provenance)
    {
        return Ok(RatificationEvaluationV1 {
            state: RatificationStateV1::NeedsDomainExpertReview,
            proposal_digest: proposal.proposal_digest,
            missing_roles,
            blocking_questions,
            unresolved_contradictions,
        });
    }

    if current_reviews.iter().any(|review| {
        review.role == ReviewRoleV1::LocalRatifier
            && review.decision == ReviewDecisionV1::RequestChanges
    }) {
        return Ok(RatificationEvaluationV1 {
            state: RatificationStateV1::NeedsCommunityClarification,
            proposal_digest: proposal.proposal_digest,
            missing_roles: vec![ReviewRoleV1::LocalRatifier],
            blocking_questions,
            unresolved_contradictions,
        });
    }

    let local_ratified = current_reviews.iter().any(|review| {
        review.role == ReviewRoleV1::LocalRatifier && review.decision == ReviewDecisionV1::Approve
    });
    Ok(RatificationEvaluationV1 {
        state: if local_ratified {
            RatificationStateV1::Ratified
        } else {
            RatificationStateV1::ReadyForLocalRatification
        },
        proposal_digest: proposal.proposal_digest,
        missing_roles: if local_ratified { Vec::new() } else { vec![ReviewRoleV1::LocalRatifier] },
        blocking_questions,
        unresolved_contradictions,
    })
}

fn validate_policy_ref(policy: &PolicyRefV1) -> Result<(), NeedIntakeErrorV1> {
    require_non_empty(&policy.policy_id)?;
    if policy.revision == 0 {
        return Err(NeedIntakeErrorV1::InvalidRevision);
    }
    require_digest(&policy.digest)
}

fn validate_capability_profile(
    profile: &CapabilityProfileRefV1,
) -> Result<(), NeedIntakeErrorV1> {
    require_non_empty(&profile.vocabulary_id)?;
    require_non_empty(&profile.concept_id)?;
    require_non_empty(&profile.revision)?;
    require_digest(&profile.digest)
}

fn require_non_empty(value: &str) -> Result<(), NeedIntakeErrorV1> {
    if !value.is_empty() && value.trim() == value {
        Ok(())
    } else {
        Err(NeedIntakeErrorV1::EmptyField)
    }
}

fn require_digest(digest: &[u8; 32]) -> Result<(), NeedIntakeErrorV1> {
    if digest.iter().all(|byte| *byte == 0) {
        Err(NeedIntakeErrorV1::MissingDigest)
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const NOW: u64 = 100;

    fn authority(id: &str) -> LocalAuthorityRefV1 {
        LocalAuthorityRefV1 {
            policy_id: format!("policy-{id}"),
            revision: 1,
            policy_digest: [1; 32],
            decision_id: format!("decision-{id}"),
            decision_digest: [2; 32],
        }
    }

    fn valid_proposal() -> NeedProposalV1 {
        let source = SourceRefV1 {
            source_id: "community-water-survey".to_owned(),
            source_digest: [3; 32],
            kind: SourceKindV1::ParticipantStatement,
            visibility_profile_id: "community-controlled".to_owned(),
        };
        let trace = vec![TraceabilityLinkV1 {
            relation: "supports the proposed service outcome".to_owned(),
            basis: TraceabilityBasisV1::Source {
                source_id: source.source_id.clone(),
                source_digest: source.source_digest,
            },
        }];
        NeedProposalV1 {
            schema_version: NEED_INTAKE_SCHEMA_VERSION,
            jurisdiction_id: "ZA-GP".to_owned(),
            need: CommunityNeedV1 {
                need_id: "community-water-access".to_owned(),
                community_id: "community-water-committee".to_owned(),
                statement_digest: [4; 32],
                submitter_id: "community-assembly".to_owned(),
                visibility_profile_id: "community-controlled".to_owned(),
            },
            revision: 1,
            previous_proposal_digest: None,
            proposal_digest: [5; 32],
            sources: vec![source],
            assumptions: vec![
                AssumptionV1 {
                    assumption_id: "existing-parts-cost".to_owned(),
                    statement: "Existing compatible parts are available at lower cost than a full replacement".to_owned(),
                    owner_id: "maintenance-team".to_owned(),
                    risk_if_false: "Repair may not be the lower-cost option".to_owned(),
                },
                AssumptionV1 {
                    assumption_id: "replacement-maintenance".to_owned(),
                    statement: "A correctly specified replacement pump may reduce recurring maintenance".to_owned(),
                    owner_id: "maintenance-team".to_owned(),
                    risk_if_false: "Replacement may not improve lifecycle cost".to_owned(),
                },
            ],
            outcomes: vec![ProposedOutcomeV1 {
                outcome_id: "reliable-water-service".to_owned(),
                statement: "Improve reliability of the community water service".to_owned(),
                indicator: "Hours of potable water service per day".to_owned(),
                measurement_ref: "water-service-hours-observation-v1".to_owned(),
                acceptance_policy: PolicyRefV1 {
                    policy_id: "water-service-acceptance".to_owned(),
                    revision: 1,
                    digest: [30; 32],
                },
                baseline: None,
                target: "At least 20 hours/day".to_owned(),
                time_horizon: "90 days after commissioning".to_owned(),
                outcome_owner_id: "water-committee".to_owned(),
                acceptance_authority: authority("water-committee"),
                traceability: trace.clone(),
            }],
            requirements: vec![ProposedRequirementV1 {
                requirement_id: "electrical-safety".to_owned(),
                capability: CapabilityProfileRefV1 {
                    vocabulary_id: "electrical-capabilities".to_owned(),
                    concept_id: "electrical-inspection".to_owned(),
                    revision: "1.0.0".to_owned(),
                    digest: [31; 32],
                },
                context_digest: [32; 32],
                jurisdiction_id: "ZA-GP".to_owned(),
                assessment_policy: PolicyRefV1 {
                    policy_id: "licensed-electrical-inspection".to_owned(),
                    revision: 1,
                    digest: [33; 32],
                },
                minimum_contributors: 1,
                max_status_age_seconds: 60,
                statement: "Qualified electrical inspection before energizing".to_owned(),
                kind: RequirementKindV1::SafetyCritical,
                traceability: trace,
            }],
            work_packages: vec![
                ProposedWorkPackageV1 {
                    work_package_id: "inspect".to_owned(),
                    description: "Inspect the pump and electrical installation".to_owned(),
                    scope_digest: [34; 32],
                    outcome_ids: vec!["reliable-water-service".to_owned()],
                    requirement_ids: vec!["electrical-safety".to_owned()],
                    depends_on: Vec::new(),
                },
                ProposedWorkPackageV1 {
                    work_package_id: "repair".to_owned(),
                    description: "Complete the owner-approved repair".to_owned(),
                    scope_digest: [35; 32],
                    outcome_ids: vec!["reliable-water-service".to_owned()],
                    requirement_ids: vec!["electrical-safety".to_owned()],
                    depends_on: vec!["inspect".to_owned()],
                },
            ],
            alternatives: vec![
                ProposedAlternativeV1 {
                    alternative_id: "incremental-repair".to_owned(),
                    label: "Repair existing pump".to_owned(),
                    description: "Inspect and repair the existing pump where technically feasible".to_owned(),
                    outcome_ids: vec!["reliable-water-service".to_owned()],
                    requirement_ids: vec!["electrical-safety".to_owned()],
                    work_package_ids: vec!["inspect".to_owned(), "repair".to_owned()],
                    tradeoffs: vec![
                        ProposedTradeoffV1 {
                            tradeoff_id: "lower-initial-cost".to_owned(),
                            statement: "Potentially lower initial material cost".to_owned(),
                            traceability: vec![TraceabilityLinkV1 {
                                relation: "explicit assumption requiring field verification".to_owned(),
                                basis: TraceabilityBasisV1::Assumption {
                                    assumption_id: "existing-parts-cost".to_owned(),
                                },
                            }],
                        },
                        ProposedTradeoffV1 {
                            tradeoff_id: "existing-part-dependence".to_owned(),
                            statement: "Depends on condition and availability of existing parts".to_owned(),
                            traceability: vec![trace[0].clone()],
                        },
                    ],
                    traceability: vec![trace[0].clone()],
                },
                ProposedAlternativeV1 {
                    alternative_id: "replace-pump".to_owned(),
                    label: "Replace pump".to_owned(),
                    description: "Replace the existing pump after inspection confirms replacement is warranted".to_owned(),
                    outcome_ids: vec!["reliable-water-service".to_owned()],
                    requirement_ids: vec!["electrical-safety".to_owned()],
                    work_package_ids: vec!["inspect".to_owned(), "repair".to_owned()],
                    tradeoffs: vec![
                        ProposedTradeoffV1 {
                            tradeoff_id: "higher-capital-cost".to_owned(),
                            statement: "Potentially higher initial capital cost".to_owned(),
                            traceability: vec![TraceabilityLinkV1 {
                                relation: "explicit assumption requiring supplier quotation".to_owned(),
                                basis: TraceabilityBasisV1::Assumption {
                                    assumption_id: "replacement-maintenance".to_owned(),
                                },
                            }],
                        },
                        ProposedTradeoffV1 {
                            tradeoff_id: "maintenance-impact".to_owned(),
                            statement: "May reduce recurring maintenance if replacement is correctly specified".to_owned(),
                            traceability: vec![TraceabilityLinkV1 {
                                relation: "explicit lifecycle assumption requiring validation".to_owned(),
                                basis: TraceabilityBasisV1::Assumption {
                                    assumption_id: "replacement-maintenance".to_owned(),
                                },
                            }],
                        },
                    ],
                    traceability: vec![trace[0].clone()],
                },
            ],
            contradictions: Vec::new(),
            open_questions: Vec::new(),
            safety_review_required: true,
            reviews: Vec::new(),
        }
    }

    fn review(role: ReviewRoleV1, decision: ReviewDecisionV1, digest: [u8; 32]) -> ReviewRefV1 {
        ReviewRefV1 {
            reviewer_id: format!("reviewer-{role:?}"),
            role,
            authority: authority("reviewer"),
            proposal_digest: digest,
            decision,
            reviewed_at: 10,
            valid_until: None,
            withdrawn_at: None,
            selected_alternative_id: if role == ReviewRoleV1::LocalRatifier
                && decision == ReviewDecisionV1::Approve
            {
                Some("incremental-repair".to_owned())
            } else {
                None
            },
        }
    }

    fn prerequisite_reviews(proposal: &mut NeedProposalV1) {
        for role in [
            ReviewRoleV1::Community,
            ReviewRoleV1::DomainExpert,
            ReviewRoleV1::Provenance,
            ReviewRoleV1::Safety,
        ] {
            proposal.reviews.push(review(role, ReviewDecisionV1::Approve, proposal.proposal_digest));
        }
    }

    #[test]
    fn full_review_path_requires_separate_local_ratification() {
        let mut proposal = valid_proposal();
        prerequisite_reviews(&mut proposal);
        let ready = evaluate_ratification(&proposal, NOW).unwrap();
        assert_eq!(ready.state, RatificationStateV1::ReadyForLocalRatification);
        assert_eq!(ready.missing_roles, vec![ReviewRoleV1::LocalRatifier]);
        proposal.reviews.push(review(ReviewRoleV1::LocalRatifier, ReviewDecisionV1::Approve, proposal.proposal_digest));
        assert_eq!(evaluate_ratification(&proposal, NOW).unwrap().state, RatificationStateV1::Ratified);
    }

    #[test]
    fn ratified_handoff_contains_exact_proposal_and_selected_alternative() {
        let mut proposal = valid_proposal();
        prerequisite_reviews(&mut proposal);
        proposal.reviews.push(review(
            ReviewRoleV1::LocalRatifier,
            ReviewDecisionV1::Approve,
            proposal.proposal_digest,
        ));
        let envelope = ratified_need_envelope(&proposal, NOW).unwrap();
        assert_eq!(envelope.need_id, proposal.need.need_id);
        assert_eq!(envelope.proposal_digest, proposal.proposal_digest);
        assert_eq!(envelope.proposal_revision, proposal.revision);
        assert_eq!(
            envelope.selected_alternative_id.as_deref(),
            Some("incremental-repair")
        );
        assert_eq!(envelope.ratification_decision_id, "decision-reviewer-LocalRatifier");
    }

    #[test]
    fn unratified_proposal_cannot_produce_downstream_handoff() {
        let proposal = valid_proposal();
        assert_eq!(
            ratified_need_envelope(&proposal, NOW),
            Err(NeedIntakeErrorV1::NotRatified)
        );
    }

    #[test]
    fn ratification_cannot_select_an_unknown_or_model_selected_alternative() {
        let mut proposal = valid_proposal();
        let mut invalid = review(
            ReviewRoleV1::LocalRatifier,
            ReviewDecisionV1::Approve,
            proposal.proposal_digest,
        );
        invalid.selected_alternative_id = Some("model-invented-option".to_owned());
        proposal.reviews.push(invalid);
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::InvalidSelectedAlternative)
        );

        let mut proposal = valid_proposal();
        let mut model_review = review(
            ReviewRoleV1::DomainExpert,
            ReviewDecisionV1::Approve,
            proposal.proposal_digest,
        );
        model_review.selected_alternative_id = Some("incremental-repair".to_owned());
        proposal.reviews.push(model_review);
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::InvalidSelectedAlternative)
        );
    }

    #[test]
    fn unreviewed_ai_draft_cannot_be_ratified() {
        let result = evaluate_ratification(&valid_proposal(), NOW).unwrap();
        assert_ne!(result.state, RatificationStateV1::Ratified);
        assert!(result.missing_roles.contains(&ReviewRoleV1::Community));
        assert!(result.missing_roles.contains(&ReviewRoleV1::DomainExpert));
    }

    #[test]
    fn safety_critical_requirement_requires_safety_review_flag() {
        let mut proposal = valid_proposal();
        proposal.safety_review_required = false;
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::SafetyReviewRequired)
        );
    }

    #[test]
    fn empty_project_structure_cannot_be_ratified() {
        let mut proposal = valid_proposal();
        proposal.outcomes.clear();
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::EmptyProposalStructure)
        );

        let mut proposal = valid_proposal();
        proposal.work_packages.clear();
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::EmptyProposalStructure)
        );
    }

    #[test]
    fn blank_known_baseline_is_not_treated_as_unknown() {
        let mut proposal = valid_proposal();
        proposal.outcomes[0].baseline = Some(String::new());
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::InvalidBaseline)
        );
    }

    #[test]
    fn resolved_contradiction_requires_digest_and_local_authority() {
        let mut proposal = valid_proposal();
        proposal.contradictions.push(ContradictionV1 {
            contradiction_id: "resolved-source-conflict".to_owned(),
            kind: ContradictionKindV1::SourceConflict,
            affected_claim_ids: vec!["outcome:reliable-water-service".to_owned()],
            stakeholder_positions: Vec::new(),
            resolved: true,
            resolution_decision_id: Some("source-adjudication".to_owned()),
            resolution_decision_digest: Some([30; 32]),
            resolution_authority: Some(authority("domain-adjudicator")),
        });
        assert!(validate_proposal(&proposal).is_ok());

        proposal.contradictions[0].resolution_authority = None;
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::MissingContradictionResolution)
        );
    }

    #[test]
    fn every_proposed_claim_must_be_assigned_to_a_work_package() {
        let mut proposal = valid_proposal();
        proposal.work_packages[0].requirement_ids.clear();
        proposal.work_packages[1].requirement_ids.clear();
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::UnmappedClaim)
        );
    }

    #[test]
    fn project_handoff_requires_versioned_measurement_and_acceptance_policy() {
        let mut proposal = valid_proposal();
        proposal.outcomes[0].measurement_ref.clear();
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::EmptyField)
        );

        let mut proposal = valid_proposal();
        proposal.outcomes[0].acceptance_policy.digest = [0; 32];
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::MissingDigest)
        );
    }

    #[test]
    fn capability_requirement_requires_exact_context_policy_and_freshness() {
        let mut proposal = valid_proposal();
        proposal.requirements[0].jurisdiction_id = "ZA-WC".to_owned();
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::JurisdictionMismatch)
        );

        let mut proposal = valid_proposal();
        proposal.requirements[0].minimum_contributors = 0;
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::InvalidRequirementThreshold)
        );

        let mut proposal = valid_proposal();
        proposal.requirements[0].max_status_age_seconds = 0;
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::ZeroStatusFreshnessWindow)
        );

        let mut proposal = valid_proposal();
        proposal.requirements[0].capability.digest = [0; 32];
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::MissingDigest)
        );
    }

    #[test]
    fn work_package_requires_exact_scope_digest() {
        let mut proposal = valid_proposal();
        proposal.work_packages[0].scope_digest = [0; 32];
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::MissingDigest)
        );
    }

    #[test]
    fn source_digest_mismatch_is_rejected() {
        let mut proposal = valid_proposal();
        if let TraceabilityBasisV1::Source { source_digest, .. } = &mut proposal.outcomes[0].traceability[0].basis {
            *source_digest = [9; 32];
        }
        assert_eq!(validate_proposal(&proposal), Err(NeedIntakeErrorV1::UnknownOrMismatchedSource));
    }

    #[test]
    fn every_outcome_and_requirement_requires_traceability() {
        let mut proposal = valid_proposal();
        proposal.requirements[0].traceability.clear();
        assert_eq!(validate_proposal(&proposal), Err(NeedIntakeErrorV1::MissingTraceability));
    }

    #[test]
    fn explicit_assumption_is_not_promoted_to_source_fact() {
        let mut proposal = valid_proposal();
        proposal.assumptions.push(AssumptionV1 {
            assumption_id: "grid-availability".to_owned(),
            statement: "Grid supply will be available during commissioning".to_owned(),
            owner_id: "energy-team".to_owned(),
            risk_if_false: "Commissioning may be delayed".to_owned(),
        });
        proposal.requirements[0].traceability = vec![TraceabilityLinkV1 {
            relation: "explicit planning assumption".to_owned(),
            basis: TraceabilityBasisV1::Assumption { assumption_id: "grid-availability".to_owned() },
        }];
        assert!(validate_proposal(&proposal).is_ok());
    }

    #[test]
    fn inference_must_name_method_and_existing_sources() {
        let mut proposal = valid_proposal();
        proposal.requirements[0].traceability = vec![TraceabilityLinkV1 {
            relation: "inference derived from survey".to_owned(),
            basis: TraceabilityBasisV1::Inference {
                inference_id: "inference-1".to_owned(),
                method_id: "symthaea-decomposition-v1".to_owned(),
                source_ids: vec!["community-water-survey".to_owned()],
            },
        }];
        assert!(validate_proposal(&proposal).is_ok());
        if let TraceabilityBasisV1::Inference { method_id, .. } = &mut proposal.requirements[0].traceability[0].basis {
            *method_id = String::new();
        }
        assert_eq!(validate_proposal(&proposal), Err(NeedIntakeErrorV1::EmptyField));
    }

    #[test]
    fn blocking_question_prevents_ratification() {
        let mut proposal = valid_proposal();
        prerequisite_reviews(&mut proposal);
        proposal.open_questions.push(OpenQuestionV1 {
            question_id: "potability-standard".to_owned(),
            question: "Which authority defines the potable-water acceptance test?".to_owned(),
            blocks_ratification: true,
        });
        let result = evaluate_ratification(&proposal, NOW).unwrap();
        assert_eq!(result.state, RatificationStateV1::NeedsCommunityClarification);
        assert_eq!(result.blocking_questions, vec!["potability-standard"]);
    }

    #[test]
    fn stakeholder_priority_conflict_requires_community_clarification() {
        let mut proposal = valid_proposal();
        prerequisite_reviews(&mut proposal);
        proposal.contradictions.push(ContradictionV1 {
            contradiction_id: "water-priority-conflict".to_owned(),
            kind: ContradictionKindV1::StakeholderPriorityConflict,
            affected_claim_ids: vec!["outcome:reliable-water-service".to_owned()],
            stakeholder_positions: vec![
                StakeholderPositionV1 {
                    group_id: "residents-near-clinic".to_owned(),
                    statement: "Prioritize continuous service for the clinic".to_owned(),
                    traceability: valid_proposal().outcomes[0].traceability.clone(),
                },
                StakeholderPositionV1 {
                    group_id: "households-uphill".to_owned(),
                    statement: "Prioritize reliable pressure for uphill households".to_owned(),
                    traceability: valid_proposal().outcomes[0].traceability.clone(),
                },
            ],
            resolved: false,
            resolution_decision_id: None,
            resolution_decision_digest: None,
            resolution_authority: None,
        });
        assert_eq!(evaluate_ratification(&proposal, NOW).unwrap().state, RatificationStateV1::NeedsCommunityClarification);
    }

    #[test]
    fn proposed_alternatives_are_validated_and_remain_unselected_by_the_model() {
        let proposal = valid_proposal();
        assert_eq!(proposal.alternatives.len(), 2);
        assert!(validate_proposal(&proposal).is_ok());
        let review_path = evaluate_ratification(&proposal, NOW).unwrap();
        assert_eq!(review_path.state, RatificationStateV1::Draft);
    }

    #[test]
    fn every_tradeoff_requires_its_own_traceability() {
        let mut proposal = valid_proposal();
        proposal.alternatives[0].tradeoffs[0].traceability.clear();
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::MissingTraceability)
        );
    }

    #[test]
    fn alternative_cannot_reference_unknown_work_package() {
        let mut proposal = valid_proposal();
        proposal.alternatives[0].work_package_ids.push("invented-package".to_owned());
        assert_eq!(validate_proposal(&proposal), Err(NeedIntakeErrorV1::UnknownWorkReference));
    }

    #[test]
    fn stakeholder_positions_must_be_distinct() {
        let mut proposal = valid_proposal();
        proposal.contradictions.push(ContradictionV1 {
            contradiction_id: "duplicate-group-position".to_owned(),
            kind: ContradictionKindV1::StakeholderPriorityConflict,
            affected_claim_ids: vec!["outcome:reliable-water-service".to_owned()],
            stakeholder_positions: vec![
                StakeholderPositionV1 {
                    group_id: "same-group".to_owned(),
                    statement: "Prioritize clinic service".to_owned(),
                    traceability: proposal.outcomes[0].traceability.clone(),
                },
                StakeholderPositionV1 {
                    group_id: "same-group".to_owned(),
                    statement: "Prioritize household pressure".to_owned(),
                    traceability: proposal.outcomes[0].traceability.clone(),
                },
            ],
            resolved: false,
            resolution_decision_id: None,
            resolution_decision_digest: None,
            resolution_authority: None,
        });
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::DuplicateIdentifier)
        );
    }

    #[test]
    fn stakeholder_priority_conflict_requires_attributed_positions() {
        let mut proposal = valid_proposal();
        proposal.contradictions.push(ContradictionV1 {
            contradiction_id: "priority-conflict".to_owned(),
            kind: ContradictionKindV1::StakeholderPriorityConflict,
            affected_claim_ids: vec!["outcome:reliable-water-service".to_owned()],
            stakeholder_positions: vec![StakeholderPositionV1 {
                group_id: "one-group".to_owned(),
                statement: "Prioritize clinic service".to_owned(),
                traceability: proposal.outcomes[0].traceability.clone(),
            }],
            resolved: false,
            resolution_decision_id: None,
            resolution_decision_digest: None,
            resolution_authority: None,
        });
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::InsufficientStakeholderPositions)
        );
    }

    #[test]
    fn technical_contradiction_requires_domain_review() {
        let mut proposal = valid_proposal();
        prerequisite_reviews(&mut proposal);
        proposal.contradictions.push(ContradictionV1 {
            contradiction_id: "pump-capacity-conflict".to_owned(),
            kind: ContradictionKindV1::TechnicalConflict,
            affected_claim_ids: vec!["requirement:electrical-safety".to_owned()],
            stakeholder_positions: Vec::new(),
            resolved: false,
            resolution_decision_id: None,
            resolution_decision_digest: None,
            resolution_authority: None,
        });
        assert_eq!(evaluate_ratification(&proposal, NOW).unwrap().state, RatificationStateV1::NeedsDomainExpertReview);
    }

    #[test]
    fn missing_safety_review_cannot_be_replaced_by_domain_review() {
        let mut proposal = valid_proposal();
        for role in [ReviewRoleV1::Community, ReviewRoleV1::DomainExpert, ReviewRoleV1::Provenance] {
            proposal.reviews.push(review(role, ReviewDecisionV1::Approve, proposal.proposal_digest));
        }
        let result = evaluate_ratification(&proposal, NOW).unwrap();
        assert_eq!(result.state, RatificationStateV1::NeedsSafetyReview);
        assert_eq!(result.missing_roles, vec![ReviewRoleV1::Safety]);
    }

    #[test]
    fn stale_or_changed_digest_reviews_do_not_count() {
        let mut proposal = valid_proposal();
        prerequisite_reviews(&mut proposal);
        for item in &mut proposal.reviews {
            item.proposal_digest = [6; 32];
        }
        let result = evaluate_ratification(&proposal, NOW).unwrap();
        assert_eq!(result.state, RatificationStateV1::NeedsCommunityClarification);
        assert!(result.missing_roles.contains(&ReviewRoleV1::Safety));
    }

    #[test]
    fn request_changes_from_a_reviewer_role_is_not_an_approval() {
        let mut proposal = valid_proposal();
        prerequisite_reviews(&mut proposal);
        let community = proposal.reviews.iter_mut()
            .find(|item| item.role == ReviewRoleV1::Community)
            .unwrap();
        community.decision = ReviewDecisionV1::RequestChanges;
        let result = evaluate_ratification(&proposal, NOW).unwrap();
        assert_eq!(result.state, RatificationStateV1::NeedsCommunityClarification);
        assert!(result.missing_roles.contains(&ReviewRoleV1::Community));
    }

    #[test]
    fn duplicate_decisions_for_one_role_and_digest_are_rejected() {
        let mut proposal = valid_proposal();
        prerequisite_reviews(&mut proposal);
        proposal.reviews.push(review(
            ReviewRoleV1::Community,
            ReviewDecisionV1::Approve,
            proposal.proposal_digest,
        ));
        assert_eq!(
            validate_proposal(&proposal),
            Err(NeedIntakeErrorV1::DuplicateIdentifier)
        );
    }

    #[test]
    fn local_ratifier_request_changes_requires_a_new_proposal_revision() {
        let mut proposal = valid_proposal();
        prerequisite_reviews(&mut proposal);
        proposal.reviews.push(review(
            ReviewRoleV1::LocalRatifier,
            ReviewDecisionV1::RequestChanges,
            proposal.proposal_digest,
        ));
        let result = evaluate_ratification(&proposal, NOW).unwrap();
        assert_eq!(result.state, RatificationStateV1::NeedsCommunityClarification);
        assert_eq!(result.missing_roles, vec![ReviewRoleV1::LocalRatifier]);
    }

    #[test]
    fn expired_or_withdrawn_reviews_do_not_count() {
        let mut proposal = valid_proposal();
        prerequisite_reviews(&mut proposal);
        for item in &mut proposal.reviews {
            if item.role == ReviewRoleV1::Community {
                item.valid_until = Some(50);
            }
        }
        let result = evaluate_ratification(&proposal, NOW).unwrap();
        assert_eq!(result.state, RatificationStateV1::NeedsCommunityClarification);
        assert!(result.missing_roles.contains(&ReviewRoleV1::Community));
    }

    #[test]
    fn rejection_is_not_overridden_by_other_approvals() {
        let mut proposal = valid_proposal();
        prerequisite_reviews(&mut proposal);
        proposal.reviews.push(review(ReviewRoleV1::DomainExpert, ReviewDecisionV1::Reject, proposal.proposal_digest));
        assert_eq!(evaluate_ratification(&proposal, NOW).unwrap().state, RatificationStateV1::Rejected);
    }

    #[test]
    fn dependency_cycle_is_rejected() {
        let mut proposal = valid_proposal();
        proposal.work_packages[0].depends_on.push("repair".to_owned());
        assert_eq!(validate_proposal(&proposal), Err(NeedIntakeErrorV1::DependencyCycle));
    }

    #[test]
    fn proposal_amendment_requires_previous_digest() {
        let mut proposal = valid_proposal();
        proposal.revision = 2;
        assert_eq!(validate_proposal(&proposal), Err(NeedIntakeErrorV1::InvalidRevision));
        proposal.previous_proposal_digest = Some([7; 32]);
        assert!(validate_proposal(&proposal).is_ok());
    }

    #[test]
    fn changed_proposal_digest_invalidates_prior_ratification() {
        let mut proposal = valid_proposal();
        prerequisite_reviews(&mut proposal);
        proposal.reviews.push(review(ReviewRoleV1::LocalRatifier, ReviewDecisionV1::Approve, proposal.proposal_digest));
        assert_eq!(evaluate_ratification(&proposal, NOW).unwrap().state, RatificationStateV1::Ratified);
        proposal.revision = 2;
        proposal.previous_proposal_digest = Some(proposal.proposal_digest);
        proposal.proposal_digest = [8; 32];
        assert_eq!(evaluate_ratification(&proposal, NOW).unwrap().state, RatificationStateV1::NeedsCommunityClarification);
    }

    #[test]
    fn nonblocking_question_remains_visible_without_blocking_ratification() {
        let mut proposal = valid_proposal();
        prerequisite_reviews(&mut proposal);
        proposal.open_questions.push(OpenQuestionV1 {
            question_id: "pump-brand".to_owned(),
            question: "Which pump brand is preferred?".to_owned(),
            blocks_ratification: false,
        });
        let result = evaluate_ratification(&proposal, NOW).unwrap();
        assert_eq!(result.state, RatificationStateV1::ReadyForLocalRatification);
        assert!(result.blocking_questions.is_empty());
    }
}
