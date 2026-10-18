//! Evidence dossier contracts for proposing building-code changes.
//!
//! A dossier can organize transparent evidence and prepare a case for
//! independent review or public consultation. It cannot declare evidence
//! scientifically conclusive by a universal score, change a rule module, repeal
//! a code, or substitute for the legally authorized adoption process. Dissent,
//! counter-evidence, uncertainties, conflicts of interest and affected groups
//! are preserved as first-class records.

fn nonblank(value: &str) -> bool {
    !value.trim().is_empty()
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

fn valid_jurisdiction_path(path: &str) -> bool {
    nonblank(path)
        && path.split('/').all(|segment| {
            !segment.is_empty()
                && segment
                    .bytes()
                    .all(|byte| byte.is_ascii_alphanumeric() || byte == b'-' || byte == b'_')
        })
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvidenceKind {
    EngineeringValidation,
    IncidentOrFailureData,
    ControlledStudy,
    ObservationalStudy,
    SystematicReview,
    Simulation,
    AdministrativeData,
    Other,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvidenceDirection {
    SupportsProposal,
    ChallengesProposal,
    MixedOrInconclusive,
}

/// A source's study design and limits are recorded rather than compressed into
/// a single opaque "quality score". The source bytes must be verified by an
/// external archive; this module only checks metadata and digest syntax.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ReformEvidenceItem {
    pub evidence_id: String,
    pub source_uri: String,
    pub source_sha256: String,
    pub kind: EvidenceKind,
    pub method_summary: String,
    pub population_and_context: String,
    pub outcome_metrics: String,
    pub findings_summary: String,
    pub limitations_and_uncertainty: String,
    pub direction: EvidenceDirection,
    pub independent_replication_count: u32,
    pub conflict_of_interest_disclosure: String,
    pub published_on_day: u32,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AffectedGroupImpact {
    pub group_id: String,
    pub expected_benefits: String,
    pub foreseeable_burdens_or_harms: String,
    pub accessibility_and_equity_considerations: String,
}

/// Dissenting reviewers remain in the dossier and do not get averaged away.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IndependentReviewDisposition {
    SupportsPublicConsultation,
    ChallengesProposal,
    Inconclusive,
    RequiresMoreEvidence,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct IndependentReformReview {
    pub reviewer_id: String,
    pub expertise_scope: String,
    /// Digest of the exact immutable dossier snapshot reviewed.
    pub reviewed_snapshot_sha256: String,
    pub disposition: IndependentReviewDisposition,
    pub rationale: String,
    pub conflict_of_interest_disclosure: String,
    pub reviewed_on_day: u32,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CodeReformDossier {
    pub dossier_id: String,
    pub jurisdiction_path: String,
    pub author_id: String,
    /// Digest of the canonical dossier snapshot, supplied by the archive.
    /// It must not be naively computed over a document containing itself.
    pub snapshot_sha256: String,
    pub target_statement_ids: Vec<String>,
    pub problem_statement: String,
    pub current_rule_and_intended_outcome: String,
    pub proposed_performance_outcome: String,
    pub proposed_rule_summary: String,
    pub alternatives_considered: String,
    pub expected_benefits: String,
    pub foreseeable_harms_and_tradeoffs: String,
    /// Includes affected residents, workers, owners, emergency services, and
    /// groups who may be excluded or disproportionately burdened, as relevant.
    pub affected_groups: Vec<AffectedGroupImpact>,
    pub evidence: Vec<ReformEvidenceItem>,
    /// A structured search summary, including null or conflicting results.
    /// A blank value is never interpreted as "no counter-evidence exists".
    pub counterevidence_search_summary: String,
    /// Explicitly state unresolved questions; "none identified" is acceptable
    /// when that conclusion is justified and recorded.
    pub uncertainty_register: String,
    pub dossier_conflict_of_interest_disclosure: String,
    pub independent_reviews: Vec<IndependentReformReview>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ReformDossierBlocker {
    EmptyField(&'static str),
    InvalidJurisdictionPath,
    InvalidSnapshotDigest,
    EmptyTargetStatementId,
    DuplicateTargetStatement(String),
    NoEvidenceItems,
    EmptyEvidenceId,
    DuplicateEvidenceId(String),
    InvalidEvidenceMetadata { evidence_id: String, field: &'static str },
    InvalidEvidenceDigest(String),
    DuplicateAffectedGroup(String),
    InvalidAffectedGroupMetadata { group_id: String, field: &'static str },
    DuplicateReviewer(String),
    ReviewerIsAuthor(String),
    InvalidReviewMetadata { reviewer_id: String, field: &'static str },
    InvalidReviewedSnapshotDigest(String),
    ReviewerLookedAtDifferentSnapshot(String),
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PublicConsultationGap {
    FewerThanTwoIndependentReviewers,
    NoReviewerSupportsPublicConsultation,
}

/// "ReadyForPublicConsultation" means a transparent dossier has completed the
/// defined preparation/review steps. It is not an endorsement, a finding that
/// the science is conclusive, or a legal amendment of any code.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReformDossierStatus {
    Incomplete,
    ReadyForIndependentReview,
    ReadyForPublicConsultation,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ReformDossierReport {
    pub status: ReformDossierStatus,
    pub blockers: Vec<ReformDossierBlocker>,
    pub public_consultation_gaps: Vec<PublicConsultationGap>,
}

/// Validate provenance metadata and preparation completeness for a code-reform
/// dossier. No numeric evidence-quality threshold is universal; interpretation
/// remains with independent specialists and the authority empowered to adopt
/// or amend the relevant rule. This method never mutates an active rule pack.
pub fn evaluate_reform_dossier(dossier: &CodeReformDossier) -> ReformDossierReport {
    let mut blockers = Vec::new();
    for (field, value) in [
        ("dossier_id", dossier.dossier_id.as_str()),
        ("author_id", dossier.author_id.as_str()),
        ("problem_statement", dossier.problem_statement.as_str()),
        (
            "current_rule_and_intended_outcome",
            dossier.current_rule_and_intended_outcome.as_str(),
        ),
        (
            "proposed_performance_outcome",
            dossier.proposed_performance_outcome.as_str(),
        ),
        ("proposed_rule_summary", dossier.proposed_rule_summary.as_str()),
        ("alternatives_considered", dossier.alternatives_considered.as_str()),
        ("expected_benefits", dossier.expected_benefits.as_str()),
        (
            "foreseeable_harms_and_tradeoffs",
            dossier.foreseeable_harms_and_tradeoffs.as_str(),
        ),
        (
            "counterevidence_search_summary",
            dossier.counterevidence_search_summary.as_str(),
        ),
        ("uncertainty_register", dossier.uncertainty_register.as_str()),
        (
            "dossier_conflict_of_interest_disclosure",
            dossier.dossier_conflict_of_interest_disclosure.as_str(),
        ),
    ] {
        if !nonblank(value) {
            blockers.push(ReformDossierBlocker::EmptyField(field));
        }
    }
    if !valid_jurisdiction_path(&dossier.jurisdiction_path) {
        blockers.push(ReformDossierBlocker::InvalidJurisdictionPath);
    }
    if !is_sha256_hex(&dossier.snapshot_sha256) {
        blockers.push(ReformDossierBlocker::InvalidSnapshotDigest);
    }

    if dossier.target_statement_ids.is_empty() {
        blockers.push(ReformDossierBlocker::EmptyField("target_statement_ids"));
    }
    let mut target_ids = std::collections::HashSet::new();
    for statement_id in &dossier.target_statement_ids {
        if !nonblank(statement_id) {
            blockers.push(ReformDossierBlocker::EmptyTargetStatementId);
        } else if !target_ids.insert(statement_id.as_str()) {
            blockers.push(ReformDossierBlocker::DuplicateTargetStatement(
                statement_id.clone(),
            ));
        }
    }

    if dossier.evidence.is_empty() {
        blockers.push(ReformDossierBlocker::NoEvidenceItems);
    }
    let mut evidence_ids = std::collections::HashSet::new();
    for item in &dossier.evidence {
        if !nonblank(&item.evidence_id) {
            blockers.push(ReformDossierBlocker::EmptyEvidenceId);
            continue;
        }
        if !evidence_ids.insert(item.evidence_id.as_str()) {
            blockers.push(ReformDossierBlocker::DuplicateEvidenceId(
                item.evidence_id.clone(),
            ));
        }
        for (field, value) in [
            ("source_uri", item.source_uri.as_str()),
            ("method_summary", item.method_summary.as_str()),
            ("population_and_context", item.population_and_context.as_str()),
            ("outcome_metrics", item.outcome_metrics.as_str()),
            ("findings_summary", item.findings_summary.as_str()),
            (
                "limitations_and_uncertainty",
                item.limitations_and_uncertainty.as_str(),
            ),
            (
                "conflict_of_interest_disclosure",
                item.conflict_of_interest_disclosure.as_str(),
            ),
        ] {
            if !nonblank(value) {
                blockers.push(ReformDossierBlocker::InvalidEvidenceMetadata {
                    evidence_id: item.evidence_id.clone(),
                    field,
                });
            }
        }
        if !is_sha256_hex(&item.source_sha256) {
            blockers.push(ReformDossierBlocker::InvalidEvidenceDigest(
                item.evidence_id.clone(),
            ));
        }
    }

    let mut group_ids = std::collections::HashSet::new();
    for group in &dossier.affected_groups {
        if !nonblank(&group.group_id) {
            blockers.push(ReformDossierBlocker::InvalidAffectedGroupMetadata {
                group_id: group.group_id.clone(),
                field: "group_id",
            });
        } else if !group_ids.insert(group.group_id.as_str()) {
            blockers.push(ReformDossierBlocker::DuplicateAffectedGroup(
                group.group_id.clone(),
            ));
        }
        for (field, value) in [
            ("expected_benefits", group.expected_benefits.as_str()),
            (
                "foreseeable_burdens_or_harms",
                group.foreseeable_burdens_or_harms.as_str(),
            ),
            (
                "accessibility_and_equity_considerations",
                group.accessibility_and_equity_considerations.as_str(),
            ),
        ] {
            if !nonblank(value) {
                blockers.push(ReformDossierBlocker::InvalidAffectedGroupMetadata {
                    group_id: group.group_id.clone(),
                    field,
                });
            }
        }
    }
    if dossier.affected_groups.is_empty() {
        blockers.push(ReformDossierBlocker::EmptyField("affected_groups"));
    }

    let mut reviewer_ids = std::collections::HashSet::new();
    for review in &dossier.independent_reviews {
        if !nonblank(&review.reviewer_id) {
            blockers.push(ReformDossierBlocker::InvalidReviewMetadata {
                reviewer_id: review.reviewer_id.clone(),
                field: "reviewer_id",
            });
        } else if !reviewer_ids.insert(review.reviewer_id.as_str()) {
            blockers.push(ReformDossierBlocker::DuplicateReviewer(
                review.reviewer_id.clone(),
            ));
        }
        if review.reviewer_id == dossier.author_id {
            blockers.push(ReformDossierBlocker::ReviewerIsAuthor(
                review.reviewer_id.clone(),
            ));
        }
        for (field, value) in [
            ("expertise_scope", review.expertise_scope.as_str()),
            ("rationale", review.rationale.as_str()),
            (
                "conflict_of_interest_disclosure",
                review.conflict_of_interest_disclosure.as_str(),
            ),
        ] {
            if !nonblank(value) {
                blockers.push(ReformDossierBlocker::InvalidReviewMetadata {
                    reviewer_id: review.reviewer_id.clone(),
                    field,
                });
            }
        }
        if !is_sha256_hex(&review.reviewed_snapshot_sha256) {
            blockers.push(ReformDossierBlocker::InvalidReviewedSnapshotDigest(
                review.reviewer_id.clone(),
            ));
        } else if review.reviewed_snapshot_sha256 != dossier.snapshot_sha256 {
            blockers.push(ReformDossierBlocker::ReviewerLookedAtDifferentSnapshot(
                review.reviewer_id.clone(),
            ));
        }
    }

    if !blockers.is_empty() {
        return ReformDossierReport {
            status: ReformDossierStatus::Incomplete,
            blockers,
            public_consultation_gaps: Vec::new(),
        };
    }

    let mut gaps = Vec::new();
    if dossier.independent_reviews.len() < 2 {
        gaps.push(PublicConsultationGap::FewerThanTwoIndependentReviewers);
    }
    if !dossier
        .independent_reviews
        .iter()
        .any(|review| review.disposition == IndependentReviewDisposition::SupportsPublicConsultation)
    {
        gaps.push(PublicConsultationGap::NoReviewerSupportsPublicConsultation);
    }
    let status = if gaps.is_empty() {
        ReformDossierStatus::ReadyForPublicConsultation
    } else {
        ReformDossierStatus::ReadyForIndependentReview
    };

    ReformDossierReport {
        status,
        blockers,
        public_consultation_gaps: gaps,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn evidence(id: &str, direction: EvidenceDirection) -> ReformEvidenceItem {
        ReformEvidenceItem {
            evidence_id: id.into(),
            source_uri: format!("https://example.invalid/evidence/{id}"),
            source_sha256: "a".repeat(64),
            kind: EvidenceKind::EngineeringValidation,
            method_summary: "Independent controlled method; protocol archived".into(),
            population_and_context: "Declared reference cases and contextual limits".into(),
            outcome_metrics: "Predefined outcomes and uncertainty intervals".into(),
            findings_summary: "Result summary with effect and observed limitations".into(),
            limitations_and_uncertainty: "Uncertainty, limitations, and unresolved questions".into(),
            direction,
            independent_replication_count: 1,
            conflict_of_interest_disclosure: "Funding and interests disclosed".into(),
            published_on_day: 100,
        }
    }

    fn review(
        reviewer_id: &str,
        disposition: IndependentReviewDisposition,
    ) -> IndependentReformReview {
        IndependentReformReview {
            reviewer_id: reviewer_id.into(),
            expertise_scope: "Relevant independent engineering expertise".into(),
            reviewed_snapshot_sha256: "c".repeat(64),
            disposition,
            rationale: "Review rationale and dissent are preserved".into(),
            conflict_of_interest_disclosure: "No known conflict; disclosure recorded".into(),
            reviewed_on_day: 120,
        }
    }

    fn dossier() -> CodeReformDossier {
        CodeReformDossier {
            dossier_id: "reform:ventilation:pilot-1".into(),
            jurisdiction_path: "ZA/GP/Johannesburg".into(),
            author_id: "research-group:1".into(),
            snapshot_sha256: "c".repeat(64),
            target_statement_ids: vec!["rule:ventilation:min-rate".into()],
            problem_statement: "Observed requirement may not fit all evaluated contexts".into(),
            current_rule_and_intended_outcome: "Current rule text and intended public outcome".into(),
            proposed_performance_outcome: "Equivalent or improved measurable outcome under declared conditions".into(),
            proposed_rule_summary: "Alternative performance path, not an active code change".into(),
            alternatives_considered: "Options, no-change case, and less restrictive options assessed".into(),
            expected_benefits: "Expected safety, affordability, accessibility, and usability effects".into(),
            foreseeable_harms_and_tradeoffs: "Foreseeable harms, failure modes, and mitigations".into(),
            affected_groups: vec![AffectedGroupImpact {
                group_id: "occupants:general".into(),
                expected_benefits: "Potential benefit recorded".into(),
                foreseeable_burdens_or_harms: "Potential burden recorded".into(),
                accessibility_and_equity_considerations: "Access and distributional impacts considered".into(),
            }],
            evidence: vec![evidence("study:positive", EvidenceDirection::SupportsProposal)],
            counterevidence_search_summary: "Search protocol, dates, and counter-evidence recorded".into(),
            uncertainty_register: "Uncertainty sources and open questions preserved".into(),
            dossier_conflict_of_interest_disclosure: "Funding, interests, and relationships disclosed".into(),
            independent_reviews: Vec::new(),
        }
    }

    #[test]
    fn complete_dossier_waits_for_independent_review() {
        let report = evaluate_reform_dossier(&dossier());
        assert_eq!(report.status, ReformDossierStatus::ReadyForIndependentReview);
        assert!(report.blockers.is_empty());
        assert!(report.public_consultation_gaps.contains(
            &PublicConsultationGap::FewerThanTwoIndependentReviewers
        ));
    }

    #[test]
    fn public_consultation_requires_two_distinct_non_author_reviewers() {
        let mut dossier = dossier();
        dossier.independent_reviews = vec![
            review("reviewer:a", IndependentReviewDisposition::SupportsPublicConsultation),
            review("reviewer:b", IndependentReviewDisposition::ChallengesProposal),
        ];
        let report = evaluate_reform_dossier(&dossier);
        assert_eq!(report.status, ReformDossierStatus::ReadyForPublicConsultation);
        assert!(report.blockers.is_empty());
        assert!(report.public_consultation_gaps.is_empty());
        // A dissenting review is retained, not averaged away or silently dropped.
        assert_eq!(
            dossier.independent_reviews[1].disposition,
            IndependentReviewDisposition::ChallengesProposal
        );
    }

    #[test]
    fn reviewer_must_review_the_exact_dossier_snapshot() {
        let mut dossier = dossier();
        let mut reviewer = review("reviewer:a", IndependentReviewDisposition::SupportsPublicConsultation);
        reviewer.reviewed_snapshot_sha256 = "d".repeat(64);
        dossier.independent_reviews.push(reviewer);
        let report = evaluate_reform_dossier(&dossier);
        assert_eq!(report.status, ReformDossierStatus::Incomplete);
        assert!(report.blockers.contains(
            &ReformDossierBlocker::ReviewerLookedAtDifferentSnapshot("reviewer:a".into())
        ));
    }

    #[test]
    fn author_cannot_count_as_an_independent_reviewer() {
        let mut dossier = dossier();
        dossier.independent_reviews.push(review(
            "research-group:1",
            IndependentReviewDisposition::SupportsPublicConsultation,
        ));
        let report = evaluate_reform_dossier(&dossier);
        assert_eq!(report.status, ReformDossierStatus::Incomplete);
        assert!(report.blockers.contains(
            &ReformDossierBlocker::ReviewerIsAuthor("research-group:1".into())
        ));
    }

    #[test]
    fn blank_counterevidence_or_uncertainty_record_blocks_dossier() {
        let mut dossier = dossier();
        dossier.counterevidence_search_summary = "  ".into();
        dossier.uncertainty_register.clear();
        let report = evaluate_reform_dossier(&dossier);
        assert_eq!(report.status, ReformDossierStatus::Incomplete);
        assert!(report.blockers.contains(
            &ReformDossierBlocker::EmptyField("counterevidence_search_summary")
        ));
        assert!(report.blockers.contains(
            &ReformDossierBlocker::EmptyField("uncertainty_register")
        ));
    }

    #[test]
    fn malformed_evidence_digest_blocks_dossier() {
        let mut dossier = dossier();
        dossier.evidence[0].source_sha256 = "not-a-digest".into();
        let report = evaluate_reform_dossier(&dossier);
        assert_eq!(report.status, ReformDossierStatus::Incomplete);
        assert!(report.blockers.contains(
            &ReformDossierBlocker::InvalidEvidenceDigest("study:positive".into())
        ));
    }

    #[test]
    fn conflicting_evidence_is_preserved_not_converted_to_a_score() {
        let mut dossier = dossier();
        dossier.evidence.push(evidence(
            "study:counterevidence",
            EvidenceDirection::ChallengesProposal,
        ));
        assert_eq!(
            dossier.evidence[0].direction,
            EvidenceDirection::SupportsProposal
        );
        assert_eq!(
            dossier.evidence[1].direction,
            EvidenceDirection::ChallengesProposal
        );
        let report = evaluate_reform_dossier(&dossier);
        assert_eq!(report.status, ReformDossierStatus::ReadyForIndependentReview);
    }

    #[test]
    fn public_consultation_needs_an_explicit_supporting_review_without_erasing_dissent() {
        let mut dossier = dossier();
        dossier.independent_reviews = vec![
            review("reviewer:a", IndependentReviewDisposition::ChallengesProposal),
            review("reviewer:b", IndependentReviewDisposition::Inconclusive),
        ];
        let report = evaluate_reform_dossier(&dossier);
        assert_eq!(report.status, ReformDossierStatus::ReadyForIndependentReview);
        assert!(report.public_consultation_gaps.contains(
            &PublicConsultationGap::NoReviewerSupportsPublicConsultation
        ));
    }
}
