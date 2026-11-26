//! Need-first project formation and requirement-level capability coverage.
//!
//! This crate is a deterministic reference policy core. It evaluates a
//! host-supplied project, consent and evidence snapshot; it does not authenticate
//! those inputs, authorize work, reserve resources, accept physical outcomes or
//! execute economic effects.
//!
//! A qualified coverage result is scoped to one requirement, context,
//! jurisdiction, policy revision and evaluation time. It is not a universal
//! judgement about a person.

#![forbid(unsafe_code)]

use std::collections::HashSet;

/// Current schema version for the project-formation reference contract.
pub const PROJECT_FORMATION_SCHEMA_VERSION: u16 = 1;

/// Immutable reference to a taxonomy-neutral capability concept.
#[derive(Clone, Debug, Eq, PartialEq, Hash)]
pub struct CapabilityProfileRefV1 {
    /// Namespace or vocabulary authority selected by the project.
    pub vocabulary_id: String,
    /// Concept identifier inside that vocabulary.
    pub concept_id: String,
    /// Exact vocabulary/profile revision.
    pub revision: String,
    /// Digest of the exact vocabulary/profile definition.
    pub digest: [u8; 32],
}

impl CapabilityProfileRefV1 {
    fn validate(&self) -> Result<(), ProjectFormationErrorV1> {
        require_non_empty(&self.vocabulary_id)?;
        require_non_empty(&self.concept_id)?;
        require_non_empty(&self.revision)?;
        require_digest(&self.digest)
    }
}

/// Version-pinned reference to an external policy owned by another domain.
#[derive(Clone, Debug, Eq, PartialEq, Hash)]
pub struct PolicyRefV1 {
    /// Stable policy identifier.
    pub policy_id: String,
    /// Monotonically versioned policy revision.
    pub revision: u64,
    /// Digest of the exact policy document.
    pub digest: [u8; 32],
}

impl PolicyRefV1 {
    fn validate(&self) -> Result<(), ProjectFormationErrorV1> {
        require_non_empty(&self.policy_id)?;
        if self.revision == 0 {
            return Err(ProjectFormationErrorV1::InvalidPolicyRevision);
        }
        require_digest(&self.digest)
    }
}

/// Reference to an already authorized project-formation decision.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProjectAuthorityRefV1 {
    /// Decision or delegation record identifier.
    pub decision_id: String,
    /// Digest of the exact authorization decision.
    pub decision_digest: [u8; 32],
}

/// A measurable outcome and its independently owned acceptance policy.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProjectOutcomeV1 {
    /// Stable outcome identifier within the project.
    pub outcome_id: String,
    /// Reference to the owner-defined measurement or observation contract.
    pub measurement_ref: String,
    /// Policy that the outcome owner will use for acceptance.
    pub acceptance_policy: PolicyRefV1,
}

/// Requirement importance. These classes are policy distinctions, not scores.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RequirementKindV1 {
    /// Mandatory safety qualification. Supervision cannot substitute for it.
    SafetyCritical,
    /// Required capability for the declared work package.
    Required,
    /// A deliberate learning opportunity with a bounded supervised path.
    Developmental,
    /// Desirable capability that is not a project eligibility condition.
    Preferred,
}

/// One contextual capability requirement within a work package.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CapabilityRequirementV1 {
    /// Stable requirement identifier, unique across the project.
    pub requirement_id: String,
    /// Exact capability concept and vocabulary revision.
    pub capability: CapabilityProfileRefV1,
    /// Digest of the work context in which the capability is required.
    pub context_digest: [u8; 32],
    /// Jurisdiction relevant to the assessment and work.
    pub jurisdiction_id: String,
    /// External policy used to qualify evidence for this requirement.
    pub assessment_policy: PolicyRefV1,
    /// Number of distinct contributors required for coverage.
    pub minimum_contributors: u16,
    /// Whether the requirement is safety-critical, required, developmental or preferred.
    pub kind: RequirementKindV1,
}

/// A bounded package of work linked to one or more project outcomes.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct WorkPackageV1 {
    /// Stable work-package identifier.
    pub work_package_id: String,
    /// Digest of the exact work scope.
    pub scope_digest: [u8; 32],
    /// Outcomes this package contributes to.
    pub outcome_ids: Vec<String>,
    /// Contextual capability requirements for this package.
    pub requirements: Vec<CapabilityRequirementV1>,
}

/// Versioned, need-first project envelope.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NeedToProjectV1 {
    /// Must equal PROJECT_FORMATION_SCHEMA_VERSION.
    pub schema_version: u16,
    /// Stable project identifier.
    pub project_id: String,
    /// Community or sponsor identity responsible for the stated need.
    pub community_id: String,
    /// Declared project jurisdiction.
    pub jurisdiction_id: String,
    /// Digest of the exact project terms participants accept.
    pub terms_digest: [u8; 32],
    /// Reference to a separately authorized project-formation decision.
    pub authority: ProjectAuthorityRefV1,
    /// Measurable outcomes expected from the project.
    pub outcomes: Vec<ProjectOutcomeV1>,
    /// Work packages used to organize delivery.
    pub work_packages: Vec<WorkPackageV1>,
}

impl NeedToProjectV1 {
    /// Validate structure and references without authenticating their issuers.
    pub fn validate(&self) -> Result<(), ProjectFormationErrorV1> {
        if self.schema_version != PROJECT_FORMATION_SCHEMA_VERSION {
            return Err(ProjectFormationErrorV1::UnsupportedSchema);
        }
        require_non_empty(&self.project_id)?;
        require_non_empty(&self.community_id)?;
        require_non_empty(&self.jurisdiction_id)?;
        require_digest(&self.terms_digest)?;
        require_non_empty(&self.authority.decision_id)?;
        require_digest(&self.authority.decision_digest)?;

        if self.outcomes.is_empty() {
            return Err(ProjectFormationErrorV1::NoOutcomes);
        }
        if self.work_packages.is_empty() {
            return Err(ProjectFormationErrorV1::NoWorkPackages);
        }

        let mut outcome_ids = HashSet::new();
        for outcome in &self.outcomes {
            require_non_empty(&outcome.outcome_id)?;
            require_non_empty(&outcome.measurement_ref)?;
            outcome.acceptance_policy.validate()?;
            if !outcome_ids.insert(outcome.outcome_id.as_str()) {
                return Err(ProjectFormationErrorV1::DuplicateOutcomeId);
            }
        }

        let mut work_package_ids = HashSet::new();
        let mut requirement_ids = HashSet::new();
        for package in &self.work_packages {
            require_non_empty(&package.work_package_id)?;
            require_digest(&package.scope_digest)?;
            if !work_package_ids.insert(package.work_package_id.as_str()) {
                return Err(ProjectFormationErrorV1::DuplicateWorkPackageId);
            }
            if package.outcome_ids.is_empty() {
                return Err(ProjectFormationErrorV1::WorkPackageWithoutOutcome);
            }

            let mut package_outcomes = HashSet::new();
            for outcome_id in &package.outcome_ids {
                if !outcome_ids.contains(outcome_id.as_str()) {
                    return Err(ProjectFormationErrorV1::UnknownOutcomeReference);
                }
                if !package_outcomes.insert(outcome_id.as_str()) {
                    return Err(ProjectFormationErrorV1::DuplicateOutcomeReference);
                }
            }

            if package.requirements.is_empty() {
                return Err(ProjectFormationErrorV1::WorkPackageWithoutRequirements);
            }
            for requirement in &package.requirements {
                require_non_empty(&requirement.requirement_id)?;
                requirement.capability.validate()?;
                require_digest(&requirement.context_digest)?;
                require_non_empty(&requirement.jurisdiction_id)?;
                requirement.assessment_policy.validate()?;
                if requirement.minimum_contributors == 0 {
                    return Err(ProjectFormationErrorV1::ZeroMinimumContributors);
                }
                if !requirement_ids.insert(requirement.requirement_id.as_str()) {
                    return Err(ProjectFormationErrorV1::DuplicateRequirementId);
                }
            }
        }
        Ok(())
    }
}

/// Current disposition supplied by the trusted Work Evidence / assessment host.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvidenceDispositionV1 {
    /// The host verified the evidence against the pinned assessment policy.
    Qualified,
    /// The evidence was assessed and did not meet the requirement.
    NotQualified,
    /// The evidence is absent, not assessed or could not be resolved.
    Unknown,
    /// A dispute or material conflict is active.
    Disputed,
    /// The issuing authority or evidence owner revoked the evidence.
    Revoked,
}

/// A contextual evidence reference; this crate does not verify its signature.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CapabilityEvidenceRefV1 {
    /// Stable evidence or verification-receipt identifier.
    pub evidence_id: String,
    /// Digest of the evidence/verification receipt.
    pub evidence_digest: [u8; 32],
    /// Subject to whom the evidence applies.
    pub subject_id: String,
    /// Exact capability vocabulary/concept revision.
    pub capability: CapabilityProfileRefV1,
    /// Exact work-context digest.
    pub context_digest: [u8; 32],
    /// Jurisdiction in which the assessment is applicable.
    pub jurisdiction_id: String,
    /// Exact assessment policy used by the trusted host.
    pub assessment_policy: PolicyRefV1,
    /// Current disposition from the host's authenticated snapshot.
    pub disposition: EvidenceDispositionV1,
    /// Inclusive validity start timestamp (Unix seconds).
    pub valid_from: u64,
    /// Exclusive expiry timestamp, if bounded.
    pub valid_until: Option<u64>,
}

impl CapabilityEvidenceRefV1 {
    fn validate(&self) -> Result<(), ProjectFormationErrorV1> {
        require_non_empty(&self.evidence_id)?;
        require_digest(&self.evidence_digest)?;
        require_non_empty(&self.subject_id)?;
        self.capability.validate()?;
        require_digest(&self.context_digest)?;
        require_non_empty(&self.jurisdiction_id)?;
        self.assessment_policy.validate()?;
        if let Some(valid_until) = self.valid_until {
            if valid_until <= self.valid_from {
                return Err(ProjectFormationErrorV1::InvalidEvidenceValidity);
            }
        }
        Ok(())
    }
}

/// A learning pathway reference. It does not authorize hazardous work.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SupervisedDevelopmentRefV1 {
    /// Stable learning-plan identifier.
    pub plan_id: String,
    /// Digest of the exact learning and supervision plan.
    pub plan_digest: [u8; 32],
    /// Digest of the learner's acceptance of the learning agreement.
    pub learner_agreement_digest: [u8; 32],
    /// A different, consenting team member appointed as mentor.
    pub mentor_id: String,
    /// External, host-verified authority reference for the mentor's role.
    pub mentor_authority: PolicyRefV1,
}

/// A participant's consent to the exact project terms.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ParticipantConsentV1 {
    /// Person or organization participating in the proposal.
    pub participant_id: String,
    /// Must match the project's exact terms digest.
    pub accepted_terms_digest: [u8; 32],
    /// Acceptance timestamp (Unix seconds).
    pub accepted_at: u64,
    /// Withdrawal takes effect at this timestamp, if present.
    pub withdrawn_at: Option<u64>,
}

/// One person's offer to contribute to one exact capability requirement.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RequirementAssignmentV1 {
    /// Work package containing the requirement.
    pub work_package_id: String,
    /// Exact requirement being addressed.
    pub requirement_id: String,
    /// Consenting participant making this offer.
    pub contributor_id: String,
    /// Zero or more evidence references for this contributor.
    pub evidence: Vec<CapabilityEvidenceRefV1>,
    /// Optional developmental pathway; never substitutes for qualification.
    pub supervised_development: Option<SupervisedDevelopmentRefV1>,
}

/// Proposed team and its exact-term participant consents.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TeamProposalV1 {
    /// Project to which the proposal applies.
    pub project_id: String,
    /// Must equal the project's exact terms digest.
    pub terms_digest: [u8; 32],
    /// Current consent records for the proposed participants.
    pub members: Vec<ParticipantConsentV1>,
    /// Requirement-level contribution offers.
    pub assignments: Vec<RequirementAssignmentV1>,
}

/// Requirement-level result. There is intentionally no aggregate team score.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RequirementDispositionV1 {
    /// Enough distinct contributors have current, policy-matching evidence.
    Covered,
    /// A safety-critical requirement is not fully qualified.
    BlockedBySafetyGate,
    /// Conflicting evidence requires an independent human process.
    HumanReviewRequired,
    /// A consented supervised-development pathway is available.
    DevelopmentPathAvailable,
    /// Relevant evidence is unknown, stale, revoked or context-incompatible.
    UnknownEvidence,
    /// Evidence was assessed but fewer than the required contributors qualify.
    Uncovered,
}

/// Explainable coverage for exactly one project requirement.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RequirementCoverageV1 {
    /// Work package containing the requirement.
    pub work_package_id: String,
    /// Exact requirement identifier.
    pub requirement_id: String,
    /// The requirement category selected by the project owner.
    pub requirement_kind: RequirementKindV1,
    /// Whether an unmet requirement is a hard gate for its work package.
    /// SafetyCritical and Required are blocking; Developmental and Preferred are not.
    pub is_requirement_blocking: bool,
    /// Requirement disposition at the requested evaluation time.
    pub disposition: RequirementDispositionV1,
    /// Distinct contributors with current matching qualified evidence.
    pub qualified_contributors: Vec<String>,
    /// Distinct contributors with a valid supervised-development reference.
    pub development_contributors: Vec<String>,
    /// Number of evidence records assessed as not qualified.
    pub not_qualified_evidence: usize,
    /// Number of unknown or missing evidence records/participants.
    pub unknown_evidence: usize,
    /// Number of evidence records with an active dispute.
    pub disputed_evidence: usize,
    /// Number of expired or revoked evidence records.
    pub stale_or_revoked_evidence: usize,
    /// Number of qualified evidence records whose validity has not yet begun.
    pub not_yet_valid_evidence: usize,
    /// Number of records that did not match exact capability/context/policy scope.
    pub scope_mismatch_evidence: usize,
    /// Distinct qualified contributors required by the project.
    pub minimum_contributors: u16,
}

/// Deterministic, requirement-level evaluation output.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TeamEvaluationV1 {
    /// Project evaluated.
    pub project_id: String,
    /// Evaluation time (Unix seconds).
    pub evaluated_at: u64,
    /// One result per project requirement, in project declaration order.
    pub requirements: Vec<RequirementCoverageV1>,
}

/// Structural or consent errors. Scope-incompatible evidence is reported,
/// never silently promoted to qualified coverage.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProjectFormationErrorV1 {
    /// The schema version is not supported.
    UnsupportedSchema,
    /// A required identifier is blank or padded.
    EmptyField,
    /// A required digest is the all-zero sentinel.
    MissingDigest,
    /// A policy reference has revision zero.
    InvalidPolicyRevision,
    /// The project has no measurable outcomes.
    NoOutcomes,
    /// The project has no work packages.
    NoWorkPackages,
    /// An outcome identifier is duplicated.
    DuplicateOutcomeId,
    /// A work-package identifier is duplicated.
    DuplicateWorkPackageId,
    /// A work package references an outcome that does not exist.
    UnknownOutcomeReference,
    /// A work package repeats one of its outcome references.
    DuplicateOutcomeReference,
    /// A work package has no outcome link.
    WorkPackageWithoutOutcome,
    /// A work package has no capability requirements.
    WorkPackageWithoutRequirements,
    /// A requirement identifier is duplicated across the project.
    DuplicateRequirementId,
    /// A requirement asks for zero contributors.
    ZeroMinimumContributors,
    /// A project's work-package or requirement reference is invalid.
    InvalidProjectReference,
    /// A team proposal refers to a different project.
    ProjectMismatch,
    /// The proposal was not accepted against the exact project terms.
    ConsentDigestMismatch,
    /// A team proposal contains no participants.
    TeamHasNoParticipants,
    /// A participant identifier is duplicated.
    DuplicateParticipant,
    /// A participant accepted after the evaluation time.
    FutureConsent,
    /// A withdrawal timestamp predates acceptance.
    InvalidWithdrawalTime,
    /// A participant withdrew before the evaluation time.
    ParticipantWithdrawn,
    /// An assignment names a person without current project consent.
    ContributorNotConsented,
    /// The same participant is assigned to the same requirement twice.
    DuplicateRequirementAssignment,
    /// Evidence identifiers are duplicated within one assignment.
    DuplicateEvidenceId,
    /// Evidence is not bound to the assigned contributor.
    EvidenceSubjectMismatch,
    /// An evidence validity interval is malformed.
    InvalidEvidenceValidity,
    /// Supervision is only valid for a developmental requirement.
    SupervisionNotAllowed,
    /// A mentor cannot supervise themself.
    SelfSupervision,
    /// The mentor has not joined the team with valid consent.
    MentorNotConsented,
}

/// Evaluate a team proposal using an already authenticated host snapshot.
///
/// The host must verify project authority, participant signatures, evidence
/// signatures, assessor authority, source freshness, legal qualifications and
/// revocation state before passing these values to the evaluator. A covered
/// result is not permission to start work; the work owner still authorizes it.
pub fn evaluate_team(
    project: &NeedToProjectV1,
    proposal: &TeamProposalV1,
    now: u64,
) -> Result<TeamEvaluationV1, ProjectFormationErrorV1> {
    project.validate()?;

    if proposal.project_id != project.project_id {
        return Err(ProjectFormationErrorV1::ProjectMismatch);
    }
    if proposal.terms_digest != project.terms_digest {
        return Err(ProjectFormationErrorV1::ConsentDigestMismatch);
    }
    if proposal.members.is_empty() {
        return Err(ProjectFormationErrorV1::TeamHasNoParticipants);
    }

    let mut members = HashSet::new();
    for member in &proposal.members {
        require_non_empty(&member.participant_id)?;
        if !members.insert(member.participant_id.as_str()) {
            return Err(ProjectFormationErrorV1::DuplicateParticipant);
        }
        if member.accepted_terms_digest != project.terms_digest {
            return Err(ProjectFormationErrorV1::ConsentDigestMismatch);
        }
        if member.accepted_at > now {
            return Err(ProjectFormationErrorV1::FutureConsent);
        }
        if let Some(withdrawn_at) = member.withdrawn_at {
            if withdrawn_at < member.accepted_at {
                return Err(ProjectFormationErrorV1::InvalidWithdrawalTime);
            }
            if now >= withdrawn_at {
                return Err(ProjectFormationErrorV1::ParticipantWithdrawn);
            }
        }
    }

    let mut assignment_keys = HashSet::new();
    let mut evidence_ids = HashSet::new();
    for assignment in &proposal.assignments {
        require_non_empty(&assignment.work_package_id)?;
        require_non_empty(&assignment.requirement_id)?;
        require_non_empty(&assignment.contributor_id)?;

        let package = project
            .work_packages
            .iter()
            .find(|package| package.work_package_id == assignment.work_package_id)
            .ok_or(ProjectFormationErrorV1::InvalidProjectReference)?;
        let requirement = package
            .requirements
            .iter()
            .find(|requirement| requirement.requirement_id == assignment.requirement_id)
            .ok_or(ProjectFormationErrorV1::InvalidProjectReference)?;

        if !members.contains(assignment.contributor_id.as_str()) {
            return Err(ProjectFormationErrorV1::ContributorNotConsented);
        }

        let key = (
            assignment.work_package_id.as_str(),
            assignment.requirement_id.as_str(),
            assignment.contributor_id.as_str(),
        );
        if !assignment_keys.insert(key) {
            return Err(ProjectFormationErrorV1::DuplicateRequirementAssignment);
        }

        if let Some(development) = &assignment.supervised_development {
            if requirement.kind != RequirementKindV1::Developmental {
                return Err(ProjectFormationErrorV1::SupervisionNotAllowed);
            }
            require_non_empty(&development.plan_id)?;
            require_digest(&development.plan_digest)?;
            require_digest(&development.learner_agreement_digest)?;
            require_non_empty(&development.mentor_id)?;
            development.mentor_authority.validate()?;
            if development.mentor_id == assignment.contributor_id {
                return Err(ProjectFormationErrorV1::SelfSupervision);
            }
            if !members.contains(development.mentor_id.as_str()) {
                return Err(ProjectFormationErrorV1::MentorNotConsented);
            }
        }

        for evidence in &assignment.evidence {
            evidence.validate()?;
            if !evidence_ids.insert(evidence.evidence_id.as_str()) {
                return Err(ProjectFormationErrorV1::DuplicateEvidenceId);
            }
            if evidence.subject_id != assignment.contributor_id {
                return Err(ProjectFormationErrorV1::EvidenceSubjectMismatch);
            }
        }
    }

    let mut coverage = Vec::new();
    for package in &project.work_packages {
        for requirement in &package.requirements {
            let mut qualified = HashSet::new();
            let mut developmental = HashSet::new();
            let mut not_qualified = 0;
            let mut unknown = 0;
            let mut disputed = 0;
            let mut stale_or_revoked = 0;
            let mut not_yet_valid = 0;
            let mut scope_mismatch = 0;

            let mut matching_assignments = 0usize;
            for assignment in proposal.assignments.iter().filter(|assignment| {
                assignment.work_package_id == package.work_package_id
                    && assignment.requirement_id == requirement.requirement_id
            }) {
                matching_assignments += 1;
                if assignment.supervised_development.is_some() {
                    developmental.insert(assignment.contributor_id.clone());
                }

                if assignment.evidence.is_empty() && assignment.supervised_development.is_none() {
                    unknown += 1;
                }

                for evidence in &assignment.evidence {
                    let exact_scope = evidence.capability == requirement.capability
                        && evidence.context_digest == requirement.context_digest
                        && evidence.jurisdiction_id == requirement.jurisdiction_id
                        && evidence.assessment_policy == requirement.assessment_policy;
                    if !exact_scope {
                        scope_mismatch += 1;
                        continue;
                    }

                    match evidence.disposition {
                        EvidenceDispositionV1::Qualified => {
                            if now < evidence.valid_from {
                                not_yet_valid += 1;
                            } else if evidence
                                .valid_until
                                .map(|valid_until| now >= valid_until)
                                .unwrap_or(false)
                            {
                                stale_or_revoked += 1;
                            } else {
                                qualified.insert(assignment.contributor_id.clone());
                            }
                        }
                        EvidenceDispositionV1::NotQualified => not_qualified += 1,
                        EvidenceDispositionV1::Unknown => unknown += 1,
                        EvidenceDispositionV1::Disputed => disputed += 1,
                        EvidenceDispositionV1::Revoked => stale_or_revoked += 1,
                    }
                }
            }

            // No offer is an absence of evidence, not a negative competence finding.
            if matching_assignments == 0 {
                unknown += 1;
            }

            let mut qualified_contributors: Vec<String> = qualified.into_iter().collect();
            qualified_contributors.sort();
            let mut development_contributors: Vec<String> = developmental.into_iter().collect();
            development_contributors.sort();

            let enough_qualified =
                qualified_contributors.len() >= requirement.minimum_contributors as usize;
            let disposition = if requirement.kind == RequirementKindV1::SafetyCritical
                && (!enough_qualified || disputed > 0)
            {
                RequirementDispositionV1::BlockedBySafetyGate
            } else if disputed > 0 {
                RequirementDispositionV1::HumanReviewRequired
            } else if enough_qualified {
                RequirementDispositionV1::Covered
            } else if requirement.kind == RequirementKindV1::Developmental
                && !development_contributors.is_empty()
            {
                RequirementDispositionV1::DevelopmentPathAvailable
            } else if unknown > 0 || stale_or_revoked > 0 || not_yet_valid > 0 || scope_mismatch > 0
            {
                RequirementDispositionV1::UnknownEvidence
            } else {
                RequirementDispositionV1::Uncovered
            };

            coverage.push(RequirementCoverageV1 {
                work_package_id: package.work_package_id.clone(),
                requirement_id: requirement.requirement_id.clone(),
                requirement_kind: requirement.kind,
                is_requirement_blocking: matches!(
                    requirement.kind,
                    RequirementKindV1::SafetyCritical | RequirementKindV1::Required
                ) && disposition != RequirementDispositionV1::Covered,
                disposition,
                qualified_contributors,
                development_contributors,
                not_qualified_evidence: not_qualified,
                unknown_evidence: unknown,
                disputed_evidence: disputed,
                stale_or_revoked_evidence: stale_or_revoked,
                not_yet_valid_evidence: not_yet_valid,
                scope_mismatch_evidence: scope_mismatch,
                minimum_contributors: requirement.minimum_contributors,
            });
        }
    }

    Ok(TeamEvaluationV1 {
        project_id: project.project_id.clone(),
        evaluated_at: now,
        requirements: coverage,
    })
}

fn is_non_empty_unpadded(value: &str) -> bool {
    !value.is_empty() && value.trim() == value
}

fn require_non_empty(value: &str) -> Result<(), ProjectFormationErrorV1> {
    if is_non_empty_unpadded(value) {
        Ok(())
    } else {
        Err(ProjectFormationErrorV1::EmptyField)
    }
}

fn require_digest(value: &[u8; 32]) -> Result<(), ProjectFormationErrorV1> {
    if value.iter().all(|byte| *byte == 0) {
        Err(ProjectFormationErrorV1::MissingDigest)
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const NOW: u64 = 10;

    fn profile() -> CapabilityProfileRefV1 {
        CapabilityProfileRefV1 {
            vocabulary_id: "local:community-capabilities".to_owned(),
            concept_id: "capability:electrical-inspection".to_owned(),
            revision: "2026.1".to_owned(),
            digest: [1; 32],
        }
    }

    fn policy() -> PolicyRefV1 {
        PolicyRefV1 {
            policy_id: "assessment:electrical-v1".to_owned(),
            revision: 1,
            digest: [2; 32],
        }
    }

    fn project(kind: RequirementKindV1, minimum: u16) -> NeedToProjectV1 {
        NeedToProjectV1 {
            schema_version: PROJECT_FORMATION_SCHEMA_VERSION,
            project_id: "project-water-pump-repair".to_owned(),
            community_id: "community-a".to_owned(),
            jurisdiction_id: "ZA-GP".to_owned(),
            terms_digest: [3; 32],
            authority: ProjectAuthorityRefV1 {
                decision_id: "authority-decision-1".to_owned(),
                decision_digest: [4; 32],
            },
            outcomes: vec![ProjectOutcomeV1 {
                outcome_id: "water-service-restored".to_owned(),
                measurement_ref: "measurement:pump-availability".to_owned(),
                acceptance_policy: policy(),
            }],
            work_packages: vec![WorkPackageV1 {
                work_package_id: "inspect-and-repair".to_owned(),
                scope_digest: [5; 32],
                outcome_ids: vec!["water-service-restored".to_owned()],
                requirements: vec![CapabilityRequirementV1 {
                    requirement_id: "electrical-inspection".to_owned(),
                    capability: profile(),
                    context_digest: [6; 32],
                    jurisdiction_id: "ZA-GP".to_owned(),
                    assessment_policy: policy(),
                    minimum_contributors: minimum,
                    kind,
                }],
            }],
        }
    }

    fn member(id: &str) -> ParticipantConsentV1 {
        ParticipantConsentV1 {
            participant_id: id.to_owned(),
            accepted_terms_digest: [3; 32],
            accepted_at: 1,
            withdrawn_at: None,
        }
    }

    fn evidence(subject: &str, disposition: EvidenceDispositionV1) -> CapabilityEvidenceRefV1 {
        CapabilityEvidenceRefV1 {
            evidence_id: format!("evidence-{subject}"),
            evidence_digest: [7; 32],
            subject_id: subject.to_owned(),
            capability: profile(),
            context_digest: [6; 32],
            jurisdiction_id: "ZA-GP".to_owned(),
            assessment_policy: policy(),
            disposition,
            valid_from: 1,
            valid_until: Some(100),
        }
    }

    fn assignment(
        contributor: &str,
        evidence: Vec<CapabilityEvidenceRefV1>,
    ) -> RequirementAssignmentV1 {
        RequirementAssignmentV1 {
            work_package_id: "inspect-and-repair".to_owned(),
            requirement_id: "electrical-inspection".to_owned(),
            contributor_id: contributor.to_owned(),
            evidence,
            supervised_development: None,
        }
    }

    fn proposal(
        members: Vec<ParticipantConsentV1>,
        assignments: Vec<RequirementAssignmentV1>,
    ) -> TeamProposalV1 {
        TeamProposalV1 {
            project_id: "project-water-pump-repair".to_owned(),
            terms_digest: [3; 32],
            members,
            assignments,
        }
    }

    fn result(project: &NeedToProjectV1, proposal: &TeamProposalV1) -> RequirementCoverageV1 {
        evaluate_team(project, proposal, NOW).unwrap().requirements[0].clone()
    }

    #[test]
    fn exact_current_evidence_covers_a_requirement() {
        let project = project(RequirementKindV1::Required, 1);
        let proposal = proposal(
            vec![member("alice")],
            vec![assignment(
                "alice",
                vec![evidence("alice", EvidenceDispositionV1::Qualified)],
            )],
        );

        let coverage = result(&project, &proposal);
        assert_eq!(coverage.disposition, RequirementDispositionV1::Covered);
        assert_eq!(coverage.qualified_contributors, vec!["alice"]);
    }

    #[test]
    fn unknown_evidence_is_not_treated_as_failure_or_qualification() {
        let project = project(RequirementKindV1::Required, 1);
        let proposal = proposal(
            vec![member("alice")],
            vec![assignment(
                "alice",
                vec![evidence("alice", EvidenceDispositionV1::Unknown)],
            )],
        );

        let coverage = result(&project, &proposal);
        assert_eq!(
            coverage.disposition,
            RequirementDispositionV1::UnknownEvidence
        );
        assert!(coverage.qualified_contributors.is_empty());
    }

    #[test]
    fn safety_critical_requirement_stays_blocked_without_qualification() {
        let project = project(RequirementKindV1::SafetyCritical, 1);
        let proposal = proposal(vec![member("alice")], vec![assignment("alice", Vec::new())]);

        let coverage = result(&project, &proposal);
        assert_eq!(
            coverage.disposition,
            RequirementDispositionV1::BlockedBySafetyGate
        );
    }

    #[test]
    fn supervised_development_is_not_qualified_coverage() {
        let project = project(RequirementKindV1::Developmental, 1);
        let mut offer = assignment("learner", Vec::new());
        offer.supervised_development = Some(SupervisedDevelopmentRefV1 {
            plan_id: "plan-1".to_owned(),
            plan_digest: [8; 32],
            learner_agreement_digest: [9; 32],
            mentor_id: "mentor".to_owned(),
            mentor_authority: policy(),
        });
        let proposal = proposal(vec![member("learner"), member("mentor")], vec![offer]);

        let coverage = result(&project, &proposal);
        assert_eq!(
            coverage.disposition,
            RequirementDispositionV1::DevelopmentPathAvailable
        );
        assert!(coverage.qualified_contributors.is_empty());
        assert_eq!(coverage.development_contributors, vec!["learner"]);
    }

    #[test]
    fn supervision_cannot_satisfy_a_safety_critical_requirement() {
        let project = project(RequirementKindV1::SafetyCritical, 1);
        let mut offer = assignment("learner", Vec::new());
        offer.supervised_development = Some(SupervisedDevelopmentRefV1 {
            plan_id: "plan-1".to_owned(),
            plan_digest: [8; 32],
            learner_agreement_digest: [9; 32],
            mentor_id: "mentor".to_owned(),
            mentor_authority: policy(),
        });
        let proposal = proposal(vec![member("learner"), member("mentor")], vec![offer]);

        assert_eq!(
            evaluate_team(&project, &proposal, NOW),
            Err(ProjectFormationErrorV1::SupervisionNotAllowed)
        );
    }

    #[test]
    fn wrong_context_is_reported_but_never_counts_as_qualified() {
        let project = project(RequirementKindV1::Required, 1);
        let mut item = evidence("alice", EvidenceDispositionV1::Qualified);
        item.context_digest = [10; 32];
        let proposal = proposal(vec![member("alice")], vec![assignment("alice", vec![item])]);

        let coverage = result(&project, &proposal);
        assert_eq!(
            coverage.disposition,
            RequirementDispositionV1::UnknownEvidence
        );
        assert_eq!(coverage.scope_mismatch_evidence, 1);
        assert!(coverage.qualified_contributors.is_empty());
    }

    #[test]
    fn evidence_for_a_different_person_is_rejected() {
        let project = project(RequirementKindV1::Required, 1);
        let proposal = proposal(
            vec![member("alice")],
            vec![assignment(
                "alice",
                vec![evidence("mallory", EvidenceDispositionV1::Qualified)],
            )],
        );

        assert_eq!(
            evaluate_team(&project, &proposal, NOW),
            Err(ProjectFormationErrorV1::EvidenceSubjectMismatch)
        );
    }

    #[test]
    fn withdrawn_participant_cannot_remain_in_the_team() {
        let project = project(RequirementKindV1::Required, 1);
        let mut consent = member("alice");
        consent.withdrawn_at = Some(9);
        let proposal = proposal(vec![consent], vec![assignment("alice", Vec::new())]);

        assert_eq!(
            evaluate_team(&project, &proposal, NOW),
            Err(ProjectFormationErrorV1::ParticipantWithdrawn)
        );
    }

    #[test]
    fn consent_must_bind_the_exact_project_terms() {
        let project = project(RequirementKindV1::Required, 1);
        let mut consent = member("alice");
        consent.accepted_terms_digest = [11; 32];
        let proposal = proposal(vec![consent], vec![assignment("alice", Vec::new())]);

        assert_eq!(
            evaluate_team(&project, &proposal, NOW),
            Err(ProjectFormationErrorV1::ConsentDigestMismatch)
        );
    }

    #[test]
    fn distinct_contributor_threshold_cannot_be_met_by_one_person() {
        let project = project(RequirementKindV1::Required, 2);
        let proposal = proposal(
            vec![member("alice")],
            vec![assignment(
                "alice",
                vec![evidence("alice", EvidenceDispositionV1::Qualified), {
                    let mut second = evidence("alice", EvidenceDispositionV1::Qualified);
                    second.evidence_id = "evidence-alice-second".to_owned();
                    second
                }],
            )],
        );

        let coverage = result(&project, &proposal);
        assert_eq!(coverage.qualified_contributors, vec!["alice"]);
        assert_eq!(coverage.disposition, RequirementDispositionV1::Uncovered);
    }

    #[test]
    fn evidence_identifiers_cannot_be_replayed_across_contributors() {
        let project = project(RequirementKindV1::Required, 2);
        let mut second_evidence = evidence("bob", EvidenceDispositionV1::Qualified);
        second_evidence.evidence_id = "evidence-alice".to_owned();
        let proposal = proposal(
            vec![member("alice"), member("bob")],
            vec![
                assignment(
                    "alice",
                    vec![evidence("alice", EvidenceDispositionV1::Qualified)],
                ),
                assignment("bob", vec![second_evidence]),
            ],
        );

        assert_eq!(
            evaluate_team(&project, &proposal, NOW),
            Err(ProjectFormationErrorV1::DuplicateEvidenceId)
        );
    }

    #[test]
    fn duplicate_requirement_assignment_is_rejected() {
        let project = project(RequirementKindV1::Required, 1);
        let offer = assignment(
            "alice",
            vec![evidence("alice", EvidenceDispositionV1::Qualified)],
        );
        let proposal = proposal(
            vec![member("alice")],
            vec![offer.clone(), offer],
        );

        assert_eq!(
            evaluate_team(&project, &proposal, NOW),
            Err(ProjectFormationErrorV1::DuplicateRequirementAssignment)
        );
    }

    #[test]
    fn evidence_before_its_validity_start_is_not_yet_valid() {
        let project = project(RequirementKindV1::Required, 1);
        let mut item = evidence("alice", EvidenceDispositionV1::Qualified);
        item.valid_from = NOW + 1;
        let proposal = proposal(vec![member("alice")], vec![assignment("alice", vec![item])]);

        let coverage = result(&project, &proposal);
        assert_eq!(
            coverage.disposition,
            RequirementDispositionV1::UnknownEvidence
        );
        assert_eq!(coverage.not_yet_valid_evidence, 1);
        assert_eq!(coverage.stale_or_revoked_evidence, 0);
    }

    #[test]
    fn expired_evidence_does_not_qualify() {
        let project = project(RequirementKindV1::Required, 1);
        let mut item = evidence("alice", EvidenceDispositionV1::Qualified);
        item.valid_until = Some(NOW);
        let proposal = proposal(vec![member("alice")], vec![assignment("alice", vec![item])]);

        let coverage = result(&project, &proposal);
        assert_eq!(
            coverage.disposition,
            RequirementDispositionV1::UnknownEvidence
        );
        assert_eq!(coverage.stale_or_revoked_evidence, 1);
    }

    #[test]
    fn mentor_cannot_supervise_themself() {
        let project = project(RequirementKindV1::Developmental, 1);
        let mut offer = assignment("learner", Vec::new());
        offer.supervised_development = Some(SupervisedDevelopmentRefV1 {
            plan_id: "plan-1".to_owned(),
            plan_digest: [8; 32],
            learner_agreement_digest: [9; 32],
            mentor_id: "learner".to_owned(),
            mentor_authority: policy(),
        });
        let proposal = proposal(vec![member("learner")], vec![offer]);

        assert_eq!(
            evaluate_team(&project, &proposal, NOW),
            Err(ProjectFormationErrorV1::SelfSupervision)
        );
    }

    #[test]
    fn duplicate_requirement_ids_across_packages_are_rejected() {
        let mut project = project(RequirementKindV1::Required, 1);
        let mut package = project.work_packages[0].clone();
        package.work_package_id = "second-package".to_owned();
        project.work_packages.push(package);

        assert_eq!(
            project.validate(),
            Err(ProjectFormationErrorV1::DuplicateRequirementId)
        );
    }

    #[test]
    fn disputed_evidence_requires_human_review() {
        let project = project(RequirementKindV1::Required, 1);
        let proposal = proposal(
            vec![member("alice")],
            vec![assignment(
                "alice",
                vec![evidence("alice", EvidenceDispositionV1::Disputed)],
            )],
        );

        let coverage = result(&project, &proposal);
        assert_eq!(
            coverage.disposition,
            RequirementDispositionV1::HumanReviewRequired
        );
    }

    #[test]
    fn no_assignment_is_unknown_not_a_negative_capability_judgement() {
        let project = project(RequirementKindV1::Required, 1);
        let proposal = proposal(vec![member("alice")], Vec::new());

        let coverage = result(&project, &proposal);
        assert_eq!(
            coverage.disposition,
            RequirementDispositionV1::UnknownEvidence
        );
        assert_eq!(coverage.unknown_evidence, 1);
        assert!(coverage.qualified_contributors.is_empty());
    }

    #[test]
    fn preferred_requirement_gap_is_visible_but_not_a_hard_gate() {
        let project = project(RequirementKindV1::Preferred, 1);
        let proposal = proposal(vec![member("alice")], Vec::new());

        let coverage = result(&project, &proposal);
        assert_eq!(
            coverage.disposition,
            RequirementDispositionV1::UnknownEvidence
        );
        assert_eq!(coverage.requirement_kind, RequirementKindV1::Preferred);
        assert!(!coverage.is_requirement_blocking);
    }

    #[test]
    fn missing_required_capability_is_a_hard_gate_not_a_negative_person_judgement() {
        let project = project(RequirementKindV1::Required, 1);
        let proposal = proposal(vec![member("alice")], Vec::new());

        let coverage = result(&project, &proposal);
        assert_eq!(
            coverage.disposition,
            RequirementDispositionV1::UnknownEvidence
        );
        assert!(coverage.is_requirement_blocking);
        assert!(coverage.qualified_contributors.is_empty());
    }

    #[test]
    fn qualified_safety_evidence_plus_an_unresolved_dispute_remains_blocked() {
        let project = project(RequirementKindV1::SafetyCritical, 1);
        let proposal = proposal(
            vec![member("alice"), member("bob")],
            vec![
                assignment("alice", vec![evidence("alice", EvidenceDispositionV1::Qualified)]),
                assignment("bob", vec![evidence("bob", EvidenceDispositionV1::Disputed)]),
            ],
        );

        let coverage = result(&project, &proposal);
        assert_eq!(
            coverage.disposition,
            RequirementDispositionV1::BlockedBySafetyGate
        );
        assert!(coverage.is_requirement_blocking);
        assert_eq!(coverage.qualified_contributors, vec!["alice"]);
    }

    #[test]
    fn unqualified_evidence_is_distinct_from_unknown_evidence() {
        let project = project(RequirementKindV1::Required, 1);
        let proposal = proposal(
            vec![member("alice")],
            vec![assignment(
                "alice",
                vec![evidence("alice", EvidenceDispositionV1::NotQualified)],
            )],
        );

        let coverage = result(&project, &proposal);
        assert_eq!(coverage.disposition, RequirementDispositionV1::Uncovered);
        assert_eq!(coverage.not_qualified_evidence, 1);
        assert_eq!(coverage.unknown_evidence, 0);
    }
}
