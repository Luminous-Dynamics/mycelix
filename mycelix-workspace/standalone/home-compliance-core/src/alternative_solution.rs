//! Evidence-gated alternative-solution dossiers for homeowner design freedom.
//!
//! This module helps a project document an alternative to a prescriptive path
//! without pretending that owner preference cancels binding law. It requires an
//! explicitly identified jurisdictional submission pathway, criteria-to-evidence
//! traceability, uncertainty/counter-evidence notes, and independent review.
//! It can prepare a dossier for authority submission; it never grants a variance,
//! certifies equivalence, or changes the active code.
//!
//! All source applicability and pathway eligibility must come from separately
//! reviewed jurisdiction adapters. Evidence bytes are checked through the same
//! trusted artifact-store boundary as the home compliance gate.

use crate::{
    EvidenceRef, EvidenceVerification, EvidenceVerifier,
};
use std::collections::{HashMap, HashSet};

fn nonblank(value: &str) -> bool {
    !value.trim().is_empty()
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

/// A procedural route a jurisdiction may permit for an alternative design.
/// These routes are separate from an evidence-backed proposal to change the code.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlternativePathway {
    PerformanceBasedDemonstration,
    VarianceRequest,
    Appeal,
}

/// A capability supplied by a reviewed jurisdiction profile. "Available" means
/// the profile documents a route for submitting the case; it is not a promise
/// the authority will accept, approve, or deem the design equivalent.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PathwayEligibility {
    AvailableForSubmission,
    NotAvailable,
    Unknown,
}

/// Trusted adapter boundary for pathway eligibility. Production implementations
/// must pin the applicable source/module edition and verify source bytes,
/// jurisdiction coverage, project category, and any prerequisites.
pub trait PathwayEligibilityVerifier {
    fn assess_pathway(
        &self,
        jurisdiction_id: &str,
        rule_module_id: &str,
        rule_statement_id: &str,
        pathway: AlternativePathway,
    ) -> PathwayEligibility;
}

impl<F> PathwayEligibilityVerifier for F
where
    F: Fn(&str, &str, &str, AlternativePathway) -> PathwayEligibility,
{
    fn assess_pathway(
        &self,
        jurisdiction_id: &str,
        rule_module_id: &str,
        rule_statement_id: &str,
        pathway: AlternativePathway,
    ) -> PathwayEligibility {
        self(jurisdiction_id, rule_module_id, rule_statement_id, pathway)
    }
}

/// Performance objective or protected public outcome that the alternative must
/// address. The acceptance criterion should be testable by the named method;
/// this core does not interpret units, compute results, or establish safety.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AlternativePerformanceCriterion {
    pub criterion_id: String,
    pub protected_outcome: String,
    pub acceptance_criterion: String,
    pub method_or_standard: String,
    pub applicability_limits: String,
    pub critical_to_safety_or_health: bool,
}

/// Evaluator-produced finding for one criterion. "Pass" is a declared result
/// from an external method, not a claim made by this module.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CriterionOutcome {
    Pass,
    Fail,
    Unknown,
}

/// Criterion assessment with versioned method metadata, evidence references,
/// uncertainty, and any counter-evidence. A critical finding must not be reduced
/// to a single aggregate score.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AlternativeCriterionAssessment {
    pub criterion_id: String,
    pub outcome: CriterionOutcome,
    pub method_version: String,
    pub evaluator_version: String,
    pub result_summary: String,
    pub uncertainty_and_limitations: String,
    pub counterevidence_summary: String,
    pub evidence: Vec<EvidenceRef>,
}

/// Independent reviewers examine the exact case and design revisions. Dissent
/// and "more evidence needed" positions remain visible rather than being averaged
/// into a score. Reviewer identity/authenticity remains an external trust concern.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlternativeReviewDisposition {
    SupportsSubmission,
    ChallengesSubmission,
    RequiresMoreEvidence,
    Inconclusive,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AlternativeIndependentReview {
    pub reviewer_id: String,
    pub expertise_scope: String,
    pub reviewed_case_revision_id: String,
    pub reviewed_design_revision_id: String,
    pub disposition: AlternativeReviewDisposition,
    pub rationale: String,
    pub conflict_of_interest_disclosure: String,
}

/// Immutable logical revision of an alternative-means submission package.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AlternativeSolutionCase {
    pub case_id: String,
    pub case_revision_id: String,
    pub jurisdiction_id: String,
    pub design_revision_id: String,
    /// Exact active source module and statement the alternative responds to.
    pub rule_module_id: String,
    pub rule_statement_id: String,
    pub pathway: AlternativePathway,
    pub author_id: String,
    pub design_intent_and_rationale: String,
    pub prescriptive_path_comparison: String,
    pub alternatives_considered: String,
    pub expected_benefits: String,
    pub foreseeable_harms_and_tradeoffs: String,
    pub affected_group_impacts: String,
    pub uncertainty_register: String,
    pub counterevidence_search_summary: String,
    pub criteria: Vec<AlternativePerformanceCriterion>,
    pub assessments: Vec<AlternativeCriterionAssessment>,
    pub independent_reviews: Vec<AlternativeIndependentReview>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlternativeCaseBlocker {
    EmptyField(&'static str),
    DuplicateCriterion(String),
    NoPerformanceCriteria,
    DuplicateAssessment(String),
    MissingAssessment(String),
    AssessmentHasUnknownCriterion(String),
    InvalidCriterionMetadata { criterion_id: String, field: &'static str },
    InvalidAssessmentMetadata { criterion_id: String, field: &'static str },
    EvidenceNotSupplied(String),
    DuplicateEvidenceId(String),
    EvidenceIdNotFound { criterion_id: String, evidence_id: String },
    EvidenceDigestInvalid { criterion_id: String, evidence_id: String },
    EvidenceArtifactNotFound { criterion_id: String, evidence_id: String },
    EvidenceDigestMismatch { criterion_id: String, evidence_id: String },
    EvidenceVerificationUnavailable { criterion_id: String, evidence_id: String },
    CriterionFailed(String),
    CriterionUnknown(String),
    DuplicateReviewer(String),
    ReviewerIsAuthor(String),
    InvalidReviewMetadata { reviewer_id: String, field: &'static str },
    ReviewPinsDifferentCaseRevision(String),
    ReviewPinsDifferentDesignRevision(String),
    FewerThanTwoIndependentReviewers,
    NoReviewerSupportsSubmission,
    PathwayNotAvailable,
    PathwayEligibilityUnknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlternativeSolutionStatus {
    Blocked,
    ReadyForIndependentReview,
    ReadyForAuthoritySubmission,
}

/// The strongest positive state means a complete, evidence-byte-verified
/// submission dossier is prepared for an authorized authority to decide. It
/// does not mean the pathway has been granted, the design is compliant, or the
/// authority has accepted the evidence as equivalent.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AlternativeSolutionReport {
    pub status: AlternativeSolutionStatus,
    pub blockers: Vec<AlternativeCaseBlocker>,
    pub preserved_dissenting_reviewers: Vec<String>,
    pub criteria_passed: usize,
    pub criteria_failed: usize,
    pub criteria_unknown: usize,
}

/// Validate a performance-based alternative / variance / appeal dossier.
///
/// Preconditions come from two explicit trust adapters: the pathway checker
/// must establish that this project/jurisdiction has the specified submission
/// route, and the evidence verifier must confirm that every cited artifact
/// exists and hashes correctly. The function never decides that the physical
/// design is safe or grants a legal exception.
pub fn evaluate_alternative_solution_case<P, E>(
    case: &AlternativeSolutionCase,
    pathway_verifier: &P,
    evidence_verifier: &E,
) -> AlternativeSolutionReport
where
    P: PathwayEligibilityVerifier + ?Sized,
    E: EvidenceVerifier + ?Sized,
{
    let mut blockers = Vec::new();

    for (field, value) in [
        ("case_id", case.case_id.as_str()),
        ("case_revision_id", case.case_revision_id.as_str()),
        ("jurisdiction_id", case.jurisdiction_id.as_str()),
        ("design_revision_id", case.design_revision_id.as_str()),
        ("rule_module_id", case.rule_module_id.as_str()),
        ("rule_statement_id", case.rule_statement_id.as_str()),
        ("author_id", case.author_id.as_str()),
        ("design_intent_and_rationale", case.design_intent_and_rationale.as_str()),
        ("prescriptive_path_comparison", case.prescriptive_path_comparison.as_str()),
        ("alternatives_considered", case.alternatives_considered.as_str()),
        ("expected_benefits", case.expected_benefits.as_str()),
        ("foreseeable_harms_and_tradeoffs", case.foreseeable_harms_and_tradeoffs.as_str()),
        ("affected_group_impacts", case.affected_group_impacts.as_str()),
        ("uncertainty_register", case.uncertainty_register.as_str()),
        ("counterevidence_search_summary", case.counterevidence_search_summary.as_str()),
    ] {
        if !nonblank(value) {
            blockers.push(AlternativeCaseBlocker::EmptyField(field));
        }
    }

    let eligibility = pathway_verifier.assess_pathway(
        &case.jurisdiction_id,
        &case.rule_module_id,
        &case.rule_statement_id,
        case.pathway,
    );
    match eligibility {
        PathwayEligibility::AvailableForSubmission => {}
        PathwayEligibility::NotAvailable => {
            blockers.push(AlternativeCaseBlocker::PathwayNotAvailable)
        }
        PathwayEligibility::Unknown => {
            blockers.push(AlternativeCaseBlocker::PathwayEligibilityUnknown)
        }
    }

    if case.criteria.is_empty() {
        blockers.push(AlternativeCaseBlocker::NoPerformanceCriteria);
    }
    let mut criteria: HashMap<&str, &AlternativePerformanceCriterion> = HashMap::new();
    for criterion in &case.criteria {
        if !nonblank(&criterion.criterion_id) {
            blockers.push(AlternativeCaseBlocker::EmptyField("criterion_id"));
            continue;
        }
        if criteria.insert(criterion.criterion_id.as_str(), criterion).is_some() {
            blockers.push(AlternativeCaseBlocker::DuplicateCriterion(
                criterion.criterion_id.clone(),
            ));
        }
        for (field, value) in [
            ("protected_outcome", criterion.protected_outcome.as_str()),
            ("acceptance_criterion", criterion.acceptance_criterion.as_str()),
            ("method_or_standard", criterion.method_or_standard.as_str()),
            ("applicability_limits", criterion.applicability_limits.as_str()),
        ] {
            if !nonblank(value) {
                blockers.push(AlternativeCaseBlocker::InvalidCriterionMetadata {
                    criterion_id: criterion.criterion_id.clone(),
                    field,
                });
            }
        }
    }

    let mut assessments: HashMap<&str, &AlternativeCriterionAssessment> = HashMap::new();
    let mut seen_evidence = HashSet::new();
    let mut criteria_passed = 0;
    let mut criteria_failed = 0;
    let mut criteria_unknown = 0;
    for assessment in &case.assessments {
        let Some(criterion) = criteria.get(assessment.criterion_id.as_str()).copied() else {
            blockers.push(AlternativeCaseBlocker::AssessmentHasUnknownCriterion(
                assessment.criterion_id.clone(),
            ));
            continue;
        };
        if assessments
            .insert(assessment.criterion_id.as_str(), assessment)
            .is_some()
        {
            blockers.push(AlternativeCaseBlocker::DuplicateAssessment(
                assessment.criterion_id.clone(),
            ));
            continue;
        }
        for (field, value) in [
            ("method_version", assessment.method_version.as_str()),
            ("evaluator_version", assessment.evaluator_version.as_str()),
            ("result_summary", assessment.result_summary.as_str()),
            ("uncertainty_and_limitations", assessment.uncertainty_and_limitations.as_str()),
            ("counterevidence_summary", assessment.counterevidence_summary.as_str()),
        ] {
            if !nonblank(value) {
                blockers.push(AlternativeCaseBlocker::InvalidAssessmentMetadata {
                    criterion_id: assessment.criterion_id.clone(),
                    field,
                });
            }
        }
        if assessment.evidence.is_empty() {
            blockers.push(AlternativeCaseBlocker::EvidenceNotSupplied(
                assessment.criterion_id.clone(),
            ));
        }
        for evidence in &assessment.evidence {
            if !nonblank(&evidence.artifact_id) {
                blockers.push(AlternativeCaseBlocker::EvidenceNotSupplied(
                    assessment.criterion_id.clone(),
                ));
                continue;
            }
            if !is_sha256_hex(&evidence.artifact_sha256) {
                blockers.push(AlternativeCaseBlocker::EvidenceDigestInvalid {
                    criterion_id: assessment.criterion_id.clone(),
                    evidence_id: evidence.artifact_id.clone(),
                });
                continue;
            }
            if !seen_evidence.insert((evidence.artifact_id.as_str(), evidence.artifact_sha256.as_str())) {
                blockers.push(AlternativeCaseBlocker::DuplicateEvidenceId(
                    evidence.artifact_id.clone(),
                ));
            }
            match evidence_verifier.verify(evidence) {
                EvidenceVerification::Verified => {}
                EvidenceVerification::ArtifactNotFound => {
                    blockers.push(AlternativeCaseBlocker::EvidenceArtifactNotFound {
                        criterion_id: assessment.criterion_id.clone(),
                        evidence_id: evidence.artifact_id.clone(),
                    })
                }
                EvidenceVerification::DigestMismatch => {
                    blockers.push(AlternativeCaseBlocker::EvidenceDigestMismatch {
                        criterion_id: assessment.criterion_id.clone(),
                        evidence_id: evidence.artifact_id.clone(),
                    })
                }
                EvidenceVerification::Unavailable => {
                    blockers.push(AlternativeCaseBlocker::EvidenceVerificationUnavailable {
                        criterion_id: assessment.criterion_id.clone(),
                        evidence_id: evidence.artifact_id.clone(),
                    })
                }
            }
        }

        match assessment.outcome {
            CriterionOutcome::Pass => criteria_passed += 1,
            CriterionOutcome::Fail => {
                criteria_failed += 1;
                if criterion.critical_to_safety_or_health {
                    blockers.push(AlternativeCaseBlocker::CriterionFailed(
                        assessment.criterion_id.clone(),
                    ));
                }
            }
            CriterionOutcome::Unknown => {
                criteria_unknown += 1;
                if criterion.critical_to_safety_or_health {
                    blockers.push(AlternativeCaseBlocker::CriterionUnknown(
                        assessment.criterion_id.clone(),
                    ));
                }
            }
        }
    }

    for criterion in &case.criteria {
        if !assessments.contains_key(criterion.criterion_id.as_str()) {
            blockers.push(AlternativeCaseBlocker::MissingAssessment(
                criterion.criterion_id.clone(),
            ));
        }
    }

    let mut reviewer_ids = HashSet::new();
    let mut supporters = 0;
    let mut dissent = Vec::new();
    for review in &case.independent_reviews {
        if !nonblank(&review.reviewer_id) {
            blockers.push(AlternativeCaseBlocker::InvalidReviewMetadata {
                reviewer_id: review.reviewer_id.clone(),
                field: "reviewer_id",
            });
        } else if !reviewer_ids.insert(review.reviewer_id.as_str()) {
            blockers.push(AlternativeCaseBlocker::DuplicateReviewer(
                review.reviewer_id.clone(),
            ));
        }
        if review.reviewer_id == case.author_id {
            blockers.push(AlternativeCaseBlocker::ReviewerIsAuthor(
                review.reviewer_id.clone(),
            ));
        }
        for (field, value) in [
            ("expertise_scope", review.expertise_scope.as_str()),
            ("rationale", review.rationale.as_str()),
            ("conflict_of_interest_disclosure", review.conflict_of_interest_disclosure.as_str()),
        ] {
            if !nonblank(value) {
                blockers.push(AlternativeCaseBlocker::InvalidReviewMetadata {
                    reviewer_id: review.reviewer_id.clone(),
                    field,
                });
            }
        }
        if review.reviewed_case_revision_id != case.case_revision_id {
            blockers.push(AlternativeCaseBlocker::ReviewPinsDifferentCaseRevision(
                review.reviewer_id.clone(),
            ));
        }
        if review.reviewed_design_revision_id != case.design_revision_id {
            blockers.push(AlternativeCaseBlocker::ReviewPinsDifferentDesignRevision(
                review.reviewer_id.clone(),
            ));
        }
        match review.disposition {
            AlternativeReviewDisposition::SupportsSubmission => supporters += 1,
            AlternativeReviewDisposition::ChallengesSubmission
            | AlternativeReviewDisposition::RequiresMoreEvidence
            | AlternativeReviewDisposition::Inconclusive => dissent.push(review.reviewer_id.clone()),
        }
    }
    if case.independent_reviews.len() < 2 {
        blockers.push(AlternativeCaseBlocker::FewerThanTwoIndependentReviewers);
    }
    if supporters == 0 {
        blockers.push(AlternativeCaseBlocker::NoReviewerSupportsSubmission);
    }

    blockers.sort_by(|left, right| format!("{left:?}").cmp(&format!("{right:?}")));
    dissent.sort();

    let status = if !blockers.is_empty() {
        AlternativeSolutionStatus::Blocked
    } else if dissent.is_empty() {
        AlternativeSolutionStatus::ReadyForAuthoritySubmission
    } else {
        // Dissent and "more evidence" conclusions remain visible. A complete
        // dossier can still be submitted for authority review with dissent attached.
        AlternativeSolutionStatus::ReadyForAuthoritySubmission
    };

    AlternativeSolutionReport {
        status,
        blockers,
        preserved_dissenting_reviewers: dissent,
        criteria_passed,
        criteria_failed,
        criteria_unknown,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn evidence(id: &str) -> EvidenceRef {
        EvidenceRef {
            artifact_id: id.into(),
            artifact_sha256: "a".repeat(64),
        }
    }

    fn criterion(id: &str, critical: bool) -> AlternativePerformanceCriterion {
        AlternativePerformanceCriterion {
            criterion_id: id.into(),
            protected_outcome: format!("Protected outcome for {id}"),
            acceptance_criterion: format!("Externally defined pass/fail criterion for {id}"),
            method_or_standard: "Declared and versioned test method".into(),
            applicability_limits: "Scope, boundary conditions and known exclusions".into(),
            critical_to_safety_or_health: critical,
        }
    }

    fn assessment(id: &str, outcome: CriterionOutcome) -> AlternativeCriterionAssessment {
        AlternativeCriterionAssessment {
            criterion_id: id.into(),
            outcome,
            method_version: "method:1.0".into(),
            evaluator_version: "evaluator:1.0".into(),
            result_summary: format!("Declared result for {id}"),
            uncertainty_and_limitations: "Uncertainty intervals and limits reported".into(),
            counterevidence_summary: "Counterexamples/search results recorded".into(),
            evidence: vec![evidence(&format!("artifact:{id}"))],
        }
    }

    fn review(id: &str, case_rev: &str, design_rev: &str, disposition: AlternativeReviewDisposition) -> AlternativeIndependentReview {
        AlternativeIndependentReview {
            reviewer_id: id.into(),
            expertise_scope: "Relevant independent technical expertise".into(),
            reviewed_case_revision_id: case_rev.into(),
            reviewed_design_revision_id: design_rev.into(),
            disposition,
            rationale: "Reasoned assessment tied to this revision".into(),
            conflict_of_interest_disclosure: "Conflicts disclosed".into(),
        }
    }

    fn case() -> AlternativeSolutionCase {
        AlternativeSolutionCase {
            case_id: "alternative:window-layout:1".into(),
            case_revision_id: "case-rev:1".into(),
            jurisdiction_id: "ZA/GP/Johannesburg".into(),
            design_revision_id: "house:model:rev7".into(),
            rule_module_id: "ZA/GP/Johannesburg".into(),
            rule_statement_id: "rule:egress:opening".into(),
            pathway: AlternativePathway::PerformanceBasedDemonstration,
            author_id: "designer:1".into(),
            design_intent_and_rationale: "Owner wants a different spatial experience while protecting the required outcome".into(),
            prescriptive_path_comparison: "Prescriptive path, intended outcome, and differences documented".into(),
            alternatives_considered: "No-change, prescriptive redesign, performance-based alternative, and non-regulatory options".into(),
            expected_benefits: "Potential owner benefit, affordability and lifecycle impact recorded".into(),
            foreseeable_harms_and_tradeoffs: "Failure modes and potential trade-offs documented".into(),
            affected_group_impacts: "Occupants, visitors, emergency responders, accessibility needs considered".into(),
            uncertainty_register: "Remaining uncertainties and applicability limits listed".into(),
            counterevidence_search_summary: "Counterevidence sources and search scope recorded".into(),
            criteria: vec![criterion("egress-outcome", true), criterion("maintainability", false)],
            assessments: vec![
                assessment("egress-outcome", CriterionOutcome::Pass),
                assessment("maintainability", CriterionOutcome::Pass),
            ],
            independent_reviews: vec![
                review("reviewer:a", "case-rev:1", "house:model:rev7", AlternativeReviewDisposition::SupportsSubmission),
                review("reviewer:b", "case-rev:1", "house:model:rev7", AlternativeReviewDisposition::SupportsSubmission),
            ],
        }
    }

    fn path_available(
        _: &str,
        _: &str,
        _: &str,
        _: AlternativePathway,
    ) -> PathwayEligibility {
        PathwayEligibility::AvailableForSubmission
    }

    fn evidence_verified(_: &EvidenceRef) -> EvidenceVerification {
        EvidenceVerification::Verified
    }

    #[test]
    fn complete_alternative_is_ready_for_authority_submission_not_approval() {
        let report = evaluate_alternative_solution_case(&case(), &path_available, &evidence_verified);
        assert_eq!(report.status, AlternativeSolutionStatus::ReadyForAuthoritySubmission);
        assert!(report.blockers.is_empty());
        assert_eq!(report.criteria_passed, 2);
    }

    #[test]
    fn failing_critical_performance_criterion_blocks_submission() {
        let mut case = case();
        case.assessments[0].outcome = CriterionOutcome::Fail;
        let report = evaluate_alternative_solution_case(&case, &path_available, &evidence_verified);
        assert_eq!(report.status, AlternativeSolutionStatus::Blocked);
        assert!(report.blockers.contains(&AlternativeCaseBlocker::CriterionFailed("egress-outcome".into())));
    }

    #[test]
    fn unknown_critical_criterion_blocks_submission() {
        let mut case = case();
        case.assessments[0].outcome = CriterionOutcome::Unknown;
        let report = evaluate_alternative_solution_case(&case, &path_available, &evidence_verified);
        assert!(report.blockers.contains(&AlternativeCaseBlocker::CriterionUnknown("egress-outcome".into())));
    }

    #[test]
    fn missing_evidence_bytes_block_submission() {
        let report = evaluate_alternative_solution_case(
            &case(),
            &path_available,
            &|_: &EvidenceRef| EvidenceVerification::ArtifactNotFound,
        );
        assert!(report.blockers.iter().any(|blocker| matches!(blocker, AlternativeCaseBlocker::EvidenceArtifactNotFound { .. })));
    }

    #[test]
    fn digest_mismatch_blocks_submission() {
        let report = evaluate_alternative_solution_case(
            &case(),
            &path_available,
            &|_: &EvidenceRef| EvidenceVerification::DigestMismatch,
        );
        assert!(report.blockers.iter().any(|blocker| matches!(blocker, AlternativeCaseBlocker::EvidenceDigestMismatch { .. })));
    }

    #[test]
    fn missing_pathway_or_unknown_pathway_blocks_submission() {
        let unavailable = evaluate_alternative_solution_case(
            &case(),
            &|_, _, _, _| PathwayEligibility::NotAvailable,
            &evidence_verified,
        );
        assert!(unavailable.blockers.contains(&AlternativeCaseBlocker::PathwayNotAvailable));
        let unknown = evaluate_alternative_solution_case(
            &case(),
            &|_, _, _, _| PathwayEligibility::Unknown,
            &evidence_verified,
        );
        assert!(unknown.blockers.contains(&AlternativeCaseBlocker::PathwayEligibilityUnknown));
    }

    #[test]
    fn reviews_pinned_to_a_stale_revision_block_submission() {
        let mut case = case();
        case.independent_reviews[0].reviewed_design_revision_id = "house:model:rev6".into();
        let report = evaluate_alternative_solution_case(&case, &path_available, &evidence_verified);
        assert!(report.blockers.contains(&AlternativeCaseBlocker::ReviewPinsDifferentDesignRevision("reviewer:a".into())));
    }

    #[test]
    fn author_cannot_be_counted_as_independent_reviewer() {
        let mut case = case();
        case.independent_reviews[0].reviewer_id = case.author_id.clone();
        let report = evaluate_alternative_solution_case(&case, &path_available, &evidence_verified);
        assert!(report.blockers.contains(&AlternativeCaseBlocker::ReviewerIsAuthor("designer:1".into())));
    }

    #[test]
    fn dissent_is_preserved_instead_of_being_averaged_away() {
        let mut case = case();
        case.independent_reviews[1].disposition = AlternativeReviewDisposition::ChallengesSubmission;
        let report = evaluate_alternative_solution_case(&case, &path_available, &evidence_verified);
        assert_eq!(report.status, AlternativeSolutionStatus::ReadyForAuthoritySubmission);
        assert_eq!(report.preserved_dissenting_reviewers, vec!["reviewer:b".to_owned()]);
    }

    #[test]
    fn missing_criterion_assessment_blocks_submission() {
        let mut case = case();
        case.assessments.pop();
        let report = evaluate_alternative_solution_case(&case, &path_available, &evidence_verified);
        assert!(report.blockers.contains(&AlternativeCaseBlocker::MissingAssessment("maintainability".into())));
    }
}
